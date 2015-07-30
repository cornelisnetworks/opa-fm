/* BEGIN_ICS_COPYRIGHT3 ****************************************

Copyright (c) 2015, Intel Corporation

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_PUBLIC_IHEAP_MANAGER_H
#define _IBA_PUBLIC_IHEAP_MANAGER_H

#include "iba/public/datatypes.h"
#include "iba/public/ispinlock.h"
#include "iba/public/ilist.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* A simple heap is implemented
 * This can be given a range of addresses to manage, such as HFI local memory
 * and will provide allocate and free functions against that memory
 * No pointers are maintained in the memory managed by the heap itself
 * This means the memory can be in a slower access location, such as across
 * PCI without impacting the performance of these functions
 */

#define HEAP_DEBUG 0	/* set to 1 to enable heap debugging */
#define HEAP_MANAGER_DEBUG 0	/* set to 1 to enable heap manager logging */

/* HEAP_OBJECT flags */
#define HEAP_OBJECT_PREALLOCATED 1	/* part of initial sizing, don't release */
#define HEAP_OBJECT_IN_USE		2

#define HEAP_ERROR_RATE_LIMIT (10*60)/* report malloc errors no faster than this */

typedef struct _HEAP_OBJECT
{
	union {
		LIST_ITEM		list_item;	/* for internal use, bucket_lists */
		struct {					/* valid only when allocated */
			uint64 		addr;		/* heap start + offset */
			ATOMIC_UINT	refcount;	/* refcount for shared memory */
		} info;
	} u;
	uint8				flag; 		/* HEAP_OBJECT flags above */
	uint32				size;		/* actual size in bytes */
	uint32				offset;		/* offset from heap start address */
		/* these lists track all blocks */
	struct _HEAP_OBJECT *prev;		/* for internal use, links for buddies */
	struct _HEAP_OBJECT *next;		/* for internal use, links for buddies */
} HEAP_OBJECT;

typedef struct _HEAP_MANAGER
{
	uint32		available_memory;	/* unassigned memory at end of heap */
	uint32		max_memory;			/* total size of heap */
	uint64		start_addr;			/* physical start address of memory */
	uint32		cur_offset;			/* offset of 1st unassigned memory */
	SPIN_LOCK	heap_lock;
	QUICK_LIST	bucket_lists[32];	/* free memory chunck in power of 2 order*/
	uint32		block_size;			/* smallest block size in bytes */
	uint32		water_mark;			/* not used yet */
	uint32		tag;
	HEAP_OBJECT	*heap_object;
	const char	name[100];	
	HEAP_OBJECT *last_obj;			/* all blocks */
	uint32		last_error_time;	/* timestamp when last error reported */
#if HEAP_DEBUG	
	uint32		malloc_count;
#endif	
} HEAP_MANAGER;

typedef struct _HEAP_INIT
{
	uint64		start_addr;				/* initial address of heap */
	uint32		block_size;				/* size of blocks */
	uint32		buck_init_count[32];	/* count of initial assignment of
										 * buffers to each bucket
										 */
	uint32		max_memory;				/* should be power of 2 */
	uint32		water_mark;				/* Minimal amount memory before
										 * rearange from bucket
										 */  	
} HEAP_INIT;	

static __inline void HeapManagerIncBucketInit(HEAP_INIT *heap_init,
											uint32 log2_size)
{
	heap_init->buck_init_count[log2_size]++;
}

void HeapManagerInitState(HEAP_MANAGER *heap_mng);
FSTATUS HeapManagerInit(HEAP_MANAGER* heap_mng, const HEAP_INIT *heap_init,
						const char* name, uint32 tag);
void HeapManagerDestroy(HEAP_MANAGER *heap_mng);
HEAP_OBJECT * HeapManagerAllocate(HEAP_MANAGER *heap_mng, uint32 size);
FSTATUS HeapManagerDeallocate(HEAP_MANAGER *heap_mng, HEAP_OBJECT *heap_obj);
void HeapManagerRelease(HEAP_MANAGER *heap_mng, HEAP_OBJECT *heap_obj);

void HeapManagerPrintSummary(HEAP_MANAGER *heap_mng);
uint32 HeapManagerGetAvailable(HEAP_MANAGER *heap_mng);
uint32 HeapManagerGetUsed(HEAP_MANAGER *heap_mng);
uint32 HeapManagerGetMaxMemory(HEAP_MANAGER *heap_mng);

void HeapManagerIncRefCount(HEAP_MANAGER *heap_mng, HEAP_OBJECT *heap_obj);
void HeapManagerDecRefCount(HEAP_MANAGER *heap_mng, HEAP_OBJECT *heap_obj);
static __inline uint32 HeapManagerGetRefCount(HEAP_OBJECT *heap_obj)
{
	return AtomicRead(&heap_obj->u.info.refcount);
}
static __inline uint64 HeapManagerGetAddr(HEAP_OBJECT *heap_obj)
{
	return heap_obj->u.info.addr;
}
static __inline uint32 HeapManagerGetSize(HEAP_OBJECT *heap_obj)
{
	return heap_obj->size;
}

#if defined(__cplusplus)
}
#endif

#endif /* _IBA_PUBLIC_IHEAP_MANAGER_H */
