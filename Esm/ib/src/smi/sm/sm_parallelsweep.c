 /* BEGIN_ICS_COPYRIGHT2 ****************************************

Copyright (c) 2018, Intel Corporation

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
//	sm_parallelsweep.c
//
// DESCRIPTION
// 	Data structures and functions needed to add support for multi-threading
// 	to the FM sweep.
//===========================================================================//

#include "os_g.h"
#include "ib_types.h"
#include "ib_macros.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "cs_csm_log.h"
#include "sm_l.h"
#include "sm_parallelsweep.h"
#include "ithread.h"
#include "vs_g.h"
extern Pool_t	sm_pool;

#ifdef ENABLE_MULTITHREADED
#define pscListRemoveHead LQListRemoveHead
#define pscListRemoveTail LQListRemoveTail
#define pscListIsEmpty LQListIsEmpty
#define pscListInsertTail LQListInsertTail
#else
#define pscListRemoveHead QListRemoveHead
#define pscListRemoveTail QListRemoveTail
#define pscListIsEmpty QListIsEmpty
#define pscListInsertTail QListInsertTail
#endif

#if defined(IB_DEBUG) || defined(DBG)
char const *mutex_function = "<uninit>";
#define pscMutexAcquire(m) {MutexAcquire(m); mutex_function=__func__;}
#define pscMutexRelease(m) {mutex_function="<free>"; MutexRelease(m);}
#else
#define pscMutexAcquire(m) MutexAcquire(m)
#define pscMutexRelease(m) MutexRelease(m)
#endif

void
psc_lock(ParallelSweepContext_t *psc)
{
	if (!psc) return; // We are operating in the global context.

#ifdef ENABLE_MULTITHREADED
	MutexAcquire(&psc->topology_mutex);
#endif
}

void
psc_unlock(ParallelSweepContext_t *psc)
{
	if (!psc) return; // We are operating in the global context.

#ifdef ENABLE_MULTITHREADED
	MutexRelease(&psc->topology_mutex);
#endif
}

MaiPool_t *
psc_get_mai(ParallelSweepContext_t *psc)
{
#ifdef ENABLE_MULTITHREADED
	// It should be impossible to run out of MAI channels (we create one
	// per thread) but just in case...
	MaiPool_t *mpp = NULL;
	LIST_ITEM *li = NULL;
	while (psc && (li == NULL)) {
		li = pscListRemoveHead(&psc->mai_pool);
		if (li) {
			mpp = PARENT_STRUCT(li, MaiPool_t, item);
		} else {
			IB_LOG_WARN_FMT(__func__,"No available MAI channels.");
			vs_thread_sleep(100000);
		}
	}

	return mpp;
#else
	// When running single-threaded, just return the default.
	return psc->mai_fd;
#endif
}

void
psc_free_mai(ParallelSweepContext_t *psc, MaiPool_t *mpp)
{
#ifdef ENABLE_MULTITHREADED
	if (mpp && psc) pscListInsertTail(&psc->mai_pool, &mpp->item);
#endif
}

// Sets the workers to running mode and signals them to start work.
//
// Note that this does nothing if the work queue is empty.
// This prevents the callbacks from entering an infinite loop.
void
psc_trigger(ParallelSweepContext_t *psc)
{
	if (!psc) return;

#ifdef ENABLE_MULTITHREADED
	boolean b;

	pscMutexAcquire(&psc->critical_mutex);
	b = !QListIsEmpty(&psc->work_queue);
	pscMutexRelease(&psc->critical_mutex);

	if (b) {
		ThreadPoolSignal(&psc->worker_pool);
	}
#endif
}

// Generic wrapper function. Gets invoked by the threadpool threads, removes
// a work item from the work queue and passes it to the specified worker
// function. 
static void
_parallel_sweep_callback(void *context)
{
	ParallelSweepContext_t *psc = (ParallelSweepContext_t *)context;
	ParallelWorkItem_t *work_item = NULL;
#ifndef VXWORKS
	static __thread int isVsThreadNameSet = 0;	// __thread (C99-gnu extension,gcc3.3+,icc compatible) is used to
							// declare a  unique instance of the variable for every thread.
	if (!isVsThreadNameSet) {
		if(vs_thread_setname(ThreadGetName())==0)
			isVsThreadNameSet = 1;
	}
#endif
#ifdef ENABLE_MULTITHREADED
	pscMutexAcquire(&psc->critical_mutex);
	psc->active_threads += 1;
#endif

	// Loop until we run out of work items or we are told to stop.
	LIST_ITEM *i, *j;
	while (psc->running && !QListIsEmpty(&psc->work_queue)) {
		// Peek at the head to see if it's a barrier.
		i = QListHead(&psc->work_queue);
		work_item = PARENT_STRUCT(i, ParallelWorkItem_t, item);
		
		if (work_item->blocking) {
#ifdef ENABLE_MULTITHREADED
			if (psc->active_threads > 1) {
				// Need to wait at the barrier till all threads reach this point.
				goto quiesce;
			} else {
				// Execute the barrier function. 
				pscMutexRelease(&psc->critical_mutex);
#endif

				DEBUG_ASSERT(work_item->workfunc);
				work_item->workfunc(context, work_item);

				// Barrier is complete. Remove it from the work queue.
#ifdef ENABLE_MULTITHREADED
				pscMutexAcquire(&psc->critical_mutex);
#endif
				j = QListRemoveHead(&psc->work_queue);
				DEBUG_ASSERT(i == j);
#ifdef ENABLE_MULTITHREADED
				pscMutexRelease(&psc->critical_mutex);
#endif

				// For non-blocking work items, the workfunc is responsible
				// for freeing the work item. This allows the workfunc to
				// put the work item back on the queue, or to use statically
				// allocated work items, and so on. BUT...
				//
				// Because we don't remove the barrier from the work queue
				// until after its workfunc has completed, the workfunc can't
				// free or reuse the barrier. So, we have to free it now.
				vs_pool_free(&sm_pool,j);

#ifdef ENABLE_MULTITHREADED
				// Wake up the threads waiting on the barrier.
				ThreadPoolBroadcast(&psc->worker_pool);
			}
#endif
		} else {
			// Remove work item from the queue.
			j = QListRemoveHead(&psc->work_queue);
			DEBUG_ASSERT(i == j);
#ifdef ENABLE_MULTITHREADED
			pscMutexRelease(&psc->critical_mutex);
#endif

			DEBUG_ASSERT(work_item->workfunc);
			work_item->workfunc(context, work_item);
		}

#ifdef ENABLE_MULTITHREADED
		pscMutexAcquire(&psc->critical_mutex);
#endif
	}

#ifdef ENABLE_MULTITHREADED
quiesce:
	psc->active_threads -= 1;

	if (psc->active_threads == 0) EventTrigger(&psc->wait_event);

	pscMutexRelease(&psc->critical_mutex);
#endif
	return;
}

// Wait until the work queue is empty or we are told to stop. Returns whatever
// the worker functions set the status field to.
Status_t
psc_wait(ParallelSweepContext_t *psc)
{
	Status_t status;

	// Wait until we run out of work or are told to stop.
	//
	// IF we have been told to stop AND there are busy workers continue to
	// wait until all workers have quiesced. Note that in that case it is okay
	// for there to still be items on the work queue.

#ifdef ENABLE_MULTITHREADED
	pscMutexAcquire(&psc->critical_mutex);
	while ((psc->active_threads > 0) || (!QListIsEmpty(&psc->work_queue) &&
		psc->running)) {
#else
	// Pseudo-parallel version for VxWorks.
	while (!QListIsEmpty(&psc->work_queue) && psc->running) {
#endif
#ifdef ENABLE_MULTITHREADED
		pscMutexRelease(&psc->critical_mutex);
		EventWaitOnInterruptible(&(psc->wait_event), EVENT_NO_TIMEOUT);
		pscMutexAcquire(&psc->critical_mutex);
#else
		// Pseudo-parallel version for VxWorks.
		_parallel_sweep_callback((void *)psc);
#endif
	}

	status = psc->status;

#ifdef ENABLE_MULTITHREADED
	pscMutexRelease(&psc->critical_mutex);
#endif
	
	return status;
}

// Return true if the workers are permitted to make progress.
boolean
psc_is_running(ParallelSweepContext_t *psc)
{
	boolean b;

#ifdef ENABLE_MULTITHREADED
	pscMutexAcquire(&psc->critical_mutex);
#endif
	b = psc->running;
#ifdef ENABLE_MULTITHREADED
	pscMutexRelease(&psc->critical_mutex);
#endif
	return b;
}

// Adds an item to the work queue without triggering the workers.
void
psc_add_work_item_no_trigger(ParallelSweepContext_t *psc,
	ParallelWorkItem_t *item)
{
#ifdef ENABLE_MULTITHREADED
	pscMutexAcquire(&psc->critical_mutex);
#endif
	QListInsertTail(&psc->work_queue, &item->item);
#ifdef ENABLE_MULTITHREADED
	pscMutexRelease(&psc->critical_mutex);
#endif
}

// Add a new item to the work queue and wake up the worker threads.
// Note that *item is not freed after processing, allowing it to put itself
// back on the work queue if desired.
void
psc_add_work_item(ParallelSweepContext_t *psc, ParallelWorkItem_t *item)
{
	psc_add_work_item_no_trigger(psc, item);
	psc_trigger(psc);
}

// Adds an item to the work queue as blocking. Does not trigger the workers.
// Note that *item will be freed by the PSC api and cannot put itself back
// on the work queue. This is because we must guarantee that the barrier is
// complete before removing it from the work queue.
void
psc_add_barrier(ParallelSweepContext_t *psc, ParallelWorkItem_t *item)
{
	item->blocking = TRUE;
	psc_add_work_item_no_trigger(psc, item);
}

// Flushes the work queue. Note that this tells the workers to quiesce.
void
psc_drain_work_queue(ParallelSweepContext_t *psc)
{
#ifdef ENABLE_MULTITHREADED
	pscMutexAcquire(&psc->critical_mutex);
#endif
	psc->running = 0; // Tell all the workers to stop.
	while(!QListIsEmpty(&psc->work_queue)) {
		LIST_ITEM *li = QListRemoveHead(&psc->work_queue);
		ParallelWorkItem_t *pwi = PARENT_STRUCT(li, ParallelWorkItem_t, item);
#ifdef ENABLE_MULTITHREADED
	pscMutexRelease(&psc->critical_mutex);
#endif
		vs_pool_free(&sm_pool,pwi);
#ifdef ENABLE_MULTITHREADED
	pscMutexAcquire(&psc->critical_mutex);
#endif
	}

#ifdef ENABLE_MULTITHREADED
	pscMutexRelease(&psc->critical_mutex);
#endif
}

void
psc_cleanup(ParallelSweepContext_t *psc)
{
	// Just in case...
	psc_stop(psc);
	(void)psc_wait(psc);
	psc_drain_work_queue(psc);

#ifdef ENABLE_MULTITHREADED
	// Kill the workers.
	ThreadPoolDestroy(&psc->worker_pool);

	MaiPool_t *mpi;

	// Return the MAI file descriptors.
	while(!pscListIsEmpty(&psc->mai_pool))
	{
		LIST_ITEM *li = pscListRemoveHead(&psc->mai_pool);
		if (!li) break;
		mpi = PARENT_STRUCT(li, MaiPool_t, item);
		sm_mai_handle_close(&(mpi->fd));
		vs_pool_free(&sm_pool,mpi);
	}
#else
	vs_pool_free(&sm_pool, psc->mai_fd);
#endif

	vs_pool_free(&sm_pool, psc);
}

void
psc_go(ParallelSweepContext_t *psc)
{
	if (!psc) return;

#ifdef ENABLE_MULTITHREADED
	pscMutexAcquire(&psc->critical_mutex);
#endif
	psc->running=1;
	psc->status = VSTATUS_OK;
#ifdef ENABLE_MULTITHREADED
	pscMutexRelease(&psc->critical_mutex);
#endif
}

void
psc_stop(ParallelSweepContext_t *psc)
{
	if (!psc) return;

#ifdef ENABLE_MULTITHREADED
	pscMutexAcquire(&psc->critical_mutex);
#endif
	psc->running=0;
#ifdef ENABLE_MULTITHREADED
	pscMutexRelease(&psc->critical_mutex);
#endif
}

Status_t
psc_get_status(ParallelSweepContext_t *psc)
{
	Status_t status;
#ifdef ENABLE_MULTITHREADED
	pscMutexAcquire(&psc->critical_mutex);
#endif
	status = psc->status;
#ifdef ENABLE_MULTITHREADED
	pscMutexRelease(&psc->critical_mutex);
#endif
	return status;
}

void
psc_set_status(ParallelSweepContext_t *psc, Status_t status)
{
	// Only set the status if current status is OK.
#ifdef ENABLE_MULTITHREADED
	pscMutexAcquire(&psc->critical_mutex);
#endif
	if (psc->status == VSTATUS_OK) {
		psc->status = status;
	}
#ifdef ENABLE_MULTITHREADED
	pscMutexRelease(&psc->critical_mutex);
#endif
}

ParallelSweepContext_t*
psc_init(void)
{
	Status_t status=VSTATUS_OK;
	ParallelSweepContext_t *fabric_context;

	status = vs_pool_alloc(&sm_pool, sizeof(ParallelSweepContext_t),
		(void*)&fabric_context);
	if (status != VSTATUS_OK) {
		return NULL;
	}
	memset(fabric_context, 0, sizeof(ParallelSweepContext_t));

#ifdef ENABLE_MULTITHREADED
	fabric_context->num_threads = sm_config.psThreads;
	LQListInitState(&fabric_context->mai_pool);
	LQListInit(&fabric_context->mai_pool);
	MutexInitState(&fabric_context->topology_mutex);
	MutexInit(&fabric_context->topology_mutex);
	MutexInitState(&fabric_context->critical_mutex);
	MutexInit(&fabric_context->critical_mutex);
	EventInitState(&fabric_context->wait_event);
	EventInit(&fabric_context->wait_event);
#endif
	
	QListInitState(&fabric_context->work_queue);
	QListInit(&fabric_context->work_queue);

	// We want each thread to have a unique mai file descriptor but the thread
	// pool semantics give each thread the same context, so we create a common
	// pool of file descriptors instead. 
#ifdef ENABLE_MULTITHREADED
	// When using multi-threading, create 1 MAI handle per thread.
	unsigned i;
	for(i=0; i<fabric_context->num_threads; i++) {
#else
	// For VxWorks we only create a single MAI handle.
	{
#endif
		MaiPool_t *mpi;
		status = vs_pool_alloc(&sm_pool, sizeof(MaiPool_t), (void**)&mpi);
		if (status == VSTATUS_OK) {
			status = sm_mai_handle_open(0, sm_config.hca, sm_config.port, TRUE, &(mpi->fd));
		}
		if (status != VSTATUS_OK) {
			IB_FATAL_ERROR_NODUMP("Unable to open HFI for discovery threads.");
		}
#ifdef ENABLE_MULTITHREADED
		pscListInsertTail(&fabric_context->mai_pool, &(mpi->item));
#else
		fabric_context->mai_fd = mpi;
#endif
	}

#ifdef ENABLE_MULTITHREADED
	// Create the threads.
	ThreadPoolInitState(&fabric_context->worker_pool);
	if (!ThreadPoolCreate(&fabric_context->worker_pool, sm_config.psThreads, "ParallelSweep",
		THREAD_PRI_NORMAL, _parallel_sweep_callback,
		fabric_context)) {
		IB_FATAL_ERROR_NODUMP("Failed to allocate thread pool.\n");
	}
#endif

	return fabric_context;
}
