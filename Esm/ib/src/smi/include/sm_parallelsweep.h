/* BEGIN_ICS_COPYRIGHT2 ****************************************

Copyright (c) 2015-2020, Intel Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
// FILE NAME
//	sm_parallelsweep.h
//
// DESCRIPTION
// 	Data structures and functions needed to add support for multi-threading
// 	to the FM sweep.
//
// 	NOTES ON THE LOCKING MODEL USED IN SM PARALLEL SWEEP (AKA "PSC"):
//
// 	PSC uses two mutexes to serialize access to data. The critical_mutex
// 	and the topology_mutex. In addition, it uses the LOCKED_QUICK_LIST to
// 	serialize access to the mai_pool.
//
// 	The critical_mutex is used to serialize access to the internals of the
// 	PSC context and data, such as the counters and the work queue (we
// 	can't use a LOCKED_QUICK_LIST for the work queue because we need to
// 	"peek" at items without removing them).
//
// 	The topology_mutex is used to enforce serial access to global FM data
// 	structures - notably the old and new topologies.
//
// 	CORRECT USE OF THE SM PARALLEL SWEEP API:
//
// 	1. Don't directly access work items without removing them from the queue
// 	first. The moment an item is posted to the queue, other threads may 
// 	begin operating on it, possibly even freeing it. It's not a pretty look,
// 	don't do it!
//
// 	2. The FM was not designed for multi-threaded operation. At all. 
// 	Therefore, the basic model of parallelism is for a PSC worker to grab
// 	the topology mutex (psc_lock()) the moment it begins operating, only
// 	releasing that mutex at the moment the worker sends a MAD, and 
// 	re-acquiring the mutex the moment the send completes. You can consider
// 	releasing the mutex in other contexts IFF you are very, very sure that
// 	nothing in the code being executed will alter global data and that no
// 	other active workers will do so either. (For example, if the activate
// 	port code only reads the topology, and doesn't write to it, we could
// 	dispense with calling psc_unlock() and psc_lock().
//
//===========================================================================//

#if !defined(_SM_PARALLELSWEEP_H_)
#define _SM_PARALLELSWEEP_H_

#if !defined(__VXWORKS__)
#define ENABLE_MULTITHREADED
#endif

#include "ilist.h"
#include "iquickmap.h"
#ifdef ENABLE_MULTITHREADED
#include "imutex.h"
#include "ithreadpool.h"
#endif

// A single IB file descriptor, allocated by the MAI layer.
typedef struct {
    LIST_ITEM item;			// Used to list the item in the mai_pool.
    SmMaiHandle_t  *fd;			// An open, ready-to-use IB file descriptor.
} MaiPool_t;

// Global data for the work pool threads
typedef struct {
	QUICK_LIST work_queue;			// List of items waiting to be worked.
#ifdef ENABLE_MULTITHREADED
	LOCKED_QUICK_LIST mai_pool;		// List of available MAI handles.
	THREAD_POOL worker_pool;		// OS-abstract set of threads.
	unsigned num_threads;			// The # of threads allocated to the pool.
	unsigned active_threads;		// The # of threads currently processing
									// work items.

	EVENT wait_event;				// Triggered when callbacks complete.
	MUTEX critical_mutex;			// Used to serialize access to context data
									// so that API functions will see a
									// consistent view of the context.

	MUTEX topology_mutex;			// Used to serialize access to the global
									// topology structure.
									// TODO: Should topology_mutex be global?
#else
	MaiPool_t *mai_fd;				// Replaces MAI pool when single threaded.
#endif

	Status_t status;				// Used by users to track error status.
									// Not used by sm_parallelsweep itself.
	boolean running;				// Set to true when the workers should be
									// removing items from the work queue.
} ParallelSweepContext_t;

// Prototype for work-item specific worker functions.
//
// The worker threads themselves are generic to all parts of the sweep and are
// only responsible for initially acquiring the work item and the
// topology_mutex and then pass control to the PsWorker_t function contained
// with the work item itself. The arguments are a pointer to the (shared)
// ParallelSweepContext_t and a pointer to the work item to be processed.
struct ParallelWorkItem_s;
typedef void (*PsWorker_t)(ParallelSweepContext_t* context,
	struct ParallelWorkItem_s* item);

// A single work item. API users might wrap this with an outer structure
// to pass additional data to the worker function.
typedef struct ParallelWorkItem_s {
	LIST_ITEM 	item;		// Used to list the item in the work_queue.
	boolean		blocking;	// Work item acts as a barrier operation.
	PsWorker_t 	workfunc;	// Function to operate on this item.
} ParallelWorkItem_t;

// Acquire or release the PSC mutex. The model should be that a worker
// acquires the mutex and holds it unless it is absolutely sure it will not
// be accessing global data.
void psc_lock(ParallelSweepContext_t *psc);
void psc_unlock(ParallelSweepContext_t *psc);

// Used by the worker function to get a private MAI handle.
MaiPool_t *psc_get_mai(ParallelSweepContext_t *psc);
void psc_free_mai(ParallelSweepContext_t *psc, MaiPool_t *mpp);

// psc_trigger wakes up the worker threads to look for new work items.
void psc_trigger(ParallelSweepContext_t *psc);

// psc_wait allows another process to wait for worker callbacks to announce
// they are done. Note that work items may be added to a queue even after
// a callback announces it is done. Returns non-zero if a worker thread
// has reported an error.
Status_t psc_wait(ParallelSweepContext_t *psc);

// Tells the parallel workers to start or stop running. psc_go() also
// clears any existing status codes.
void psc_go(ParallelSweepContext_t *psc);
void psc_stop(ParallelSweepContext_t *psc);

// Gets or Sets the status variable in the context. Setting the status has no
// effect if the current status is already != VSTATUS_OK.
Status_t psc_get_status(ParallelSweepContext_t *psc);
void psc_set_status(ParallelSweepContext_t *psc, Status_t status);

// Adds an item to the work queue and triggers the workers.
// Nota Bene: the caller should not access the work item after queueing it!
void psc_add_work_item(ParallelSweepContext_t *psc,
	ParallelWorkItem_t *item);

// Adds an item to the work queue without triggering the workers.
void psc_add_work_item_no_trigger(ParallelSweepContext_t *psc,
	ParallelWorkItem_t *item);

// Adds an item to the work queue as blocking. Does not trigger the workers.
// Nota Bene: the caller should not access the work item after queueing it!
void psc_add_barrier(ParallelSweepContext_t *psc,
	ParallelWorkItem_t *item);

// Flushes the work queue and frees the work items using vs_pool_free.
// If the work items were not allocated with vs_pool_alloc or if there
// is additional bookkeeping required for the work items, do not use
// this function. Has the side effect of telling the workers to stop running.
//
void psc_drain_work_queue(ParallelSweepContext_t *psc);

// Allocates and initializes a PSC.
ParallelSweepContext_t *psc_init(void);

// Shuts down the worker threads and releases the MAI handles, then destroys
// the parallel context. The work queue should be drained before calling this
// function.
void psc_cleanup(ParallelSweepContext_t *psc);

// Returns !0 if the workers have not been told to stop.
boolean psc_is_running(ParallelSweepContext_t *psc);

#endif
