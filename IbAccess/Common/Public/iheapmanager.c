/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/

#include "iba/public/iheapmanager.h"
#include "iba/public/imemory.h"
#include "iba/public/idebug.h"
#include "iba/public/imath.h"
#include "iba/public/itimer.h"

/* Take size*count from available space at end of heap and put
 * on the given free list.
 */
static FSTATUS get_buffer_by_growing(HEAP_MANAGER *heap_mng,
								 uint32 size,
								 QUICK_LIST *list,
								 uint32 count,
								 uint8  flag)
{
	uint32 i = count;

#if HEAP_MANAGER_DEBUG
	MsgOut("get_buffer_by_growing (%p, %u, %p, %u, %u)[\n",
					heap_mng, size, list, count, flag);
	MsgOut("available memory: %d Try to grow by: %d\n",
					heap_mng->available_memory, size *count);
#endif

	while(i > 0)
	{		
		HEAP_OBJECT *heap_obj;

		if(heap_mng->available_memory < size)
		{
			DbgOut("ERROR: %s Heap is out of memory\n", heap_mng->name);	
			return  FINSUFFICIENT_RESOURCES;
		}
		heap_obj = (HEAP_OBJECT *)MemoryAllocateAndClear(sizeof(HEAP_OBJECT),
								heap_mng->tag,FALSE);
		if(heap_obj == NULL)
		{
			MsgOut("ERROR: Heap Object allocation error\n");	
			return FINSUFFICIENT_MEMORY;
		}
#if HEAP_DEBUG
		heap_mng->malloc_count++;
#endif
		heap_obj->size = size;
		heap_obj->offset = heap_mng->cur_offset;
		heap_obj->flag = flag;
		heap_mng->cur_offset += size;
		heap_mng->available_memory -=size;
		if(!heap_mng->last_obj)	
		{
			heap_mng->last_obj = heap_obj;
		} else {
			heap_mng->last_obj->next = heap_obj;
			heap_obj->prev = heap_mng->last_obj;
			heap_mng->last_obj = heap_obj;
		}	
		QListInsertTail(list, &heap_obj->u.list_item);
		QListSetObj(&heap_obj->u.list_item, heap_obj);
		i--;					
	}	
#if HEAP_MANAGER_DEBUG
	MsgOut("] get_buffer_by_growing\n");
#endif
	return FSUCCESS;
}

void HeapManagerPrintSummary(HEAP_MANAGER *heap_mng)
{
	uint32 i;
	QUICK_LIST *list;

	SpinLockAcquire(&heap_mng->heap_lock);
	
	MsgOut("Heap Manager: %s\n",heap_mng->name);
	MsgOut("Max memory: %u\n",heap_mng->max_memory);
	MsgOut("Available memory: %u\n",heap_mng->available_memory);
	MsgOut("Current Offset: %u\n",heap_mng->cur_offset);
	
	for(i = 0;i < 32;i++)
	{
		list = &heap_mng->bucket_lists[i];
		MsgOut("Free List size: %u count: %u\n",1<<i,QListCount(list));
	}
	SpinLockRelease(&heap_mng->heap_lock);
}

uint32 HeapManagerGetAvailable(HEAP_MANAGER *heap_mng)
{
	uint32 ret;

	SpinLockAcquire(&heap_mng->heap_lock);
	ret = heap_mng->available_memory;
	SpinLockRelease(&heap_mng->heap_lock);
	return ret;
}

uint32 HeapManagerGetUsed(HEAP_MANAGER *heap_mng)
{
	return heap_mng->max_memory - HeapManagerGetAvailable(heap_mng);
}

uint32 HeapManagerGetMaxMemory(HEAP_MANAGER *heap_mng)
{
	return heap_mng->max_memory;
}

void HeapManagerInitState(HEAP_MANAGER *heap_mng)
{
	MemoryClear(heap_mng, sizeof(*heap_mng));
	SpinLockInitState( &heap_mng->heap_lock );
}

FSTATUS HeapManagerInit(HEAP_MANAGER *heap_mng, const HEAP_INIT *heap_init,
						const char* name, uint32 tag)
{
	unsigned i;
	int size = 0,count = 0;
	QUICK_LIST *list;
	uint32 log2_block_size;
	
#if HEAP_MANAGER_DEBUG
	MsgOut("HeapManagerInit(%p, %s, %u)[\n", heap_init, name, tag);
#endif
		
	heap_mng->available_memory = heap_init->max_memory;
	heap_mng->max_memory = heap_init->max_memory;
	heap_mng->tag = tag;

	strcpy((char*)heap_mng->name,name?name:"Unknown Heap Manager");
		
	SpinLockInit(&heap_mng->heap_lock );

	for(i = 0;i < 32 ;i++)
	{
       	QListInitState( &heap_mng->bucket_lists[i] );
		if( !QListInit( &heap_mng->bucket_lists[i]) )
		{
#if HEAP_MANAGER_DEBUG
			MsgOut("ERROR: heap bucket lists init failed : %s: %u\n", name, i);	
#endif
            return FINVALID_PARAMETER;
		}
	}
	heap_mng->block_size = heap_init->block_size;
	heap_mng->water_mark  = heap_init->water_mark;
	heap_mng->start_addr = heap_init->start_addr;

	log2_block_size =  CeilLog2(heap_mng->block_size );
#if HEAP_DEBUG
	heap_mng->malloc_count = 0;
#endif
	/* buffer assignment to bucket */
	for(i = 31; i >= log2_block_size; i--)		
	{
		size = 1<<i;	
		count = heap_init->buck_init_count[i];
		list = 	&heap_mng->bucket_lists[i];

		if(FSUCCESS != get_buffer_by_growing(heap_mng,size,list,count,HEAP_OBJECT_PREALLOCATED))
		{
			MsgOut("ERROR: Initial buffer allocation error for heap_manager: %s size: 0x%x\n",
							name, size);
		}
	}

#if HEAP_MANAGER_DEBUG
	MsgOut("] init_heap_manager\n");
#endif
	return FSUCCESS;
}	

void HeapManagerDestroy(HEAP_MANAGER *heap_mng)
{
	int i;
	QUICK_LIST *list;

	for (i = 0;i < 32 ;i++)
	{
       	list = &heap_mng->bucket_lists[i] ;
		while (QListCount(list))
		{
			LIST_ITEM *item = QListRemoveHead(list);
			HEAP_OBJECT *heap_obj = (HEAP_OBJECT*)QListObj(item);
			ASSERT(item);
			ASSERT(heap_obj);
			heap_mng->available_memory += heap_obj->size;
			MemoryDeallocate(heap_obj);
#if HEAP_DEBUG	
			heap_mng->malloc_count--;
#endif
		}
		QListDestroy(list);
	}
	SpinLockDestroy(&heap_mng->heap_lock);		
#if HEAP_DEBUG	
	MsgOut("heap(%s) malloc_count: %d\n",heap_mng->name,heap_mng->malloc_count);
#endif
	if (heap_mng->available_memory != heap_mng->max_memory)
		MsgOut("heap(%s) leaked: %u\n", heap_mng->name,
							heap_mng->max_memory - heap_mng->available_memory);
}

/* Get buffer from bigger size queue and split and queue to requested one */
static FSTATUS HeapManagerSplitBucket(HEAP_MANAGER *heap_mng,uint32 log2_size)
{
	uint32 i,count=0,max_count = 0,max_index = 0;	
	QUICK_LIST *list,*max_list = NULL;
	HEAP_OBJECT *heap_obj = NULL;
	HEAP_OBJECT *new_heap_obj = NULL;
	LIST_ITEM *item;
	
#if HEAP_MANAGER_DEBUG
	MsgOut("Out of Free memory for heap(%s), need to rearrange\n",
					heap_mng->name);
#endif
	// split the bucket with the most free
	for(i = log2_size;i < 32;i++)
	{
		QUICK_LIST *list = &heap_mng->bucket_lists[i];	
		count =  QListCount(list);
		if(count > max_count)
		{
			max_count = count;
			max_index = i;
			max_list = list;
#if HEAP_MANAGER_DEBUG
			MsgOut("Found %u blocks of %u bytes\n", count, 1<<i);
#endif
		}			
	}
	if(max_count < 1)
		return FINSUFFICIENT_RESOURCES;
	
	item = QListRemoveHead(max_list);
	ASSERT(item);
	heap_obj = (HEAP_OBJECT *)QListObj(item);
	ASSERT(heap_obj);

	count = 1 << (max_index - log2_size);
	ASSERT(count > 1);
	if ( log2_size >= 32 ) {
		ASSERT( FALSE );
		return FOVERRUN;
	}
	list = &heap_mng->bucket_lists[log2_size];
	/* so we can split into count blocks and add to log2_size queue */
	for(i = 0;i< count;i++)
	{
		new_heap_obj = (HEAP_OBJECT *)MemoryAllocateAndClear(
								sizeof(HEAP_OBJECT), heap_mng->tag,FALSE);
		if (! new_heap_obj)
		{
			// unwind
			while (i--)
			{
				new_heap_obj = heap_obj->prev;
				ASSERT(new_heap_obj);
				heap_obj->prev = new_heap_obj->prev;
				QListRemoveItem(list, &new_heap_obj->u.list_item);
				MemoryDeallocate(new_heap_obj);	
#if HEAP_DEBUG	
				heap_mng->malloc_count--;
#endif
			}
			if (heap_obj->prev)
				heap_obj->prev->next = heap_obj;
			QListInsertTail(max_list, &heap_obj->u.list_item);				
			QListSetObj(&heap_obj->u.list_item, heap_obj);
			MsgOut("ERROR: Heap Object allocation error\n");	
			return FINSUFFICIENT_MEMORY;
		}
#if HEAP_DEBUG	
		heap_mng->malloc_count++;
#endif
		new_heap_obj->size = 1<<log2_size;
		new_heap_obj->offset = heap_obj->offset + (i * new_heap_obj->size);	 
		new_heap_obj->flag = heap_obj->flag;
		new_heap_obj->prev = heap_obj->prev;
		if (new_heap_obj->prev)
			new_heap_obj->prev->next = new_heap_obj;
		heap_obj->prev = new_heap_obj;	// scratch area while we build list

		QListInsertTail(list, &new_heap_obj->u.list_item);				
		QListSetObj(&new_heap_obj->u.list_item, new_heap_obj);
	}	
	new_heap_obj->next = heap_obj->next;
	if (new_heap_obj->next)
		new_heap_obj->next->prev = new_heap_obj;
	else
		heap_mng->last_obj = new_heap_obj;
	
	MemoryDeallocate(heap_obj);	
#if HEAP_DEBUG	
	heap_mng->malloc_count--;
#endif
	return FSUCCESS;
}



/* allocate buffer from Free List first.
 * If failed, then try to grow from free memory pool.
 * last option is split from larger buffer on different free list.
 */ 
HEAP_OBJECT * HeapManagerAllocate(HEAP_MANAGER *heap_mng,uint32 size)
{
	uint32 log2_size = 0;
	QUICK_LIST *list = NULL;
	HEAP_OBJECT *heap_obj;
	LIST_ITEM *item;

#if HEAP_MANAGER_DEBUG	
	MsgOut("HeapManagerAllocate(%p, %u) %s [\n",heap_mng,size,heap_mng->name);
#endif

	// round size up to next power of 2 >= block_size for heap
	if(size < heap_mng->block_size)
		size = heap_mng->block_size;
	log2_size = CeilLog2(size);
	if ( log2_size >= 32 ) {
		ASSERT( FALSE );
		return NULL;
	}
	size = 1<< log2_size;
	list = &heap_mng->bucket_lists[log2_size];

	SpinLockAcquire(&heap_mng->heap_lock);
	if(	QListCount(list) )
	{
		item = QListRemoveHead(list);
	} else if(FSUCCESS == get_buffer_by_growing(heap_mng,size,list,1,0)) {
		item = QListRemoveHead(list);
	} else if(FSUCCESS == HeapManagerSplitBucket(heap_mng,log2_size)) {
		item = QListRemoveHead(list);
	} else {
		uint32 timestamp = GetTimeStampSec();
		if (heap_mng->last_error_time == 0
			|| (timestamp - heap_mng->last_error_time) > HEAP_ERROR_RATE_LIMIT)
		{
			heap_mng->last_error_time = timestamp;
			SpinLockRelease(&heap_mng->heap_lock);	
			MsgOut("ERROR: Heap(%s) malloc failed\n", heap_mng->name);
		} else {
			SpinLockRelease(&heap_mng->heap_lock);	
		}
		return NULL;
	}		
	ASSERT(item);
	heap_obj = (HEAP_OBJECT *)QListObj(item);
	ASSERT(heap_obj);
#if HEAP_MANAGER_DEBUG
	MsgOut("Heap_Obj(%p): %u from Free List, remaining count: %u\n",
						heap_obj, heap_obj->size, QListCount(list) );
#endif
	heap_obj->flag |= (uint8)HEAP_OBJECT_IN_USE;
	SpinLockRelease(&heap_mng->heap_lock);			
#if HEAP_DEBUG		
	MsgOut("HeapManagerAllocate: head(%s) size(%d) done\n",
							heap_mng->name,(int)size);
#endif	
	heap_obj->u.info.addr  = heap_mng->start_addr + heap_obj->offset;
	AtomicWrite(&heap_obj->u.info.refcount, 1);
#if HEAP_MANAGER_DEBUG	
	MsgOut("] HeapManagerAllocate\n");
#endif
	return heap_obj;
}

void HeapManagerIncRefCount(HEAP_MANAGER *heap_mng,HEAP_OBJECT *heap_obj)
{
	ASSERT(heap_obj);
	AtomicIncrementVoid(&heap_obj->u.info.refcount);
}

void HeapManagerDecRefCount(HEAP_MANAGER *heap_mng,HEAP_OBJECT *heap_obj)
{
	ASSERT(heap_obj);
	AtomicDecrementVoid(&heap_obj->u.info.refcount);
}

FSTATUS HeapManagerDeallocate(HEAP_MANAGER *heap_mng,HEAP_OBJECT *heap_obj)
{
	uint32 log2_size = 0; 
	QUICK_LIST *list = NULL; 

#if HEAP_MANAGER_DEBUG
	MsgOut("HeapManagerDeallocate (%p, %p) [\n", heap_mng, heap_obj);
#endif

	SpinLockAcquire(&heap_mng->heap_lock);
	if (AtomicRead(&heap_obj->u.info.refcount) != 1)
	{
		MsgOut("ERROR: heap_obj(%p) refcount count(%u) is not zero\n",heap_obj,
							AtomicRead(&heap_obj->u.info.refcount) - 1);
		SpinLockRelease(&heap_mng->heap_lock);	
#if HEAP_MANAGER_DEBUG
		MsgOut("] HeapManagerDeallocate\n");
#endif
		return FBUSY;
	}
	ASSERT(heap_obj->flag & HEAP_OBJECT_IN_USE);
	heap_obj->flag &= ~HEAP_OBJECT_IN_USE;

	/* add back to free list */
	log2_size = CeilLog2(heap_obj->size);		
	if ( log2_size >= 32 ) {
		ASSERT( FALSE );
		return FOVERRUN;
	}
	list = &heap_mng->bucket_lists[log2_size];
	QListInsertTail(list, &heap_obj->u.list_item);
	QListSetObj(&heap_obj->u.list_item, heap_obj);
#if HEAP_DEBUG
	MsgOut("Put Heap_Obj(%p): %d back to queue\n",heap_obj,	heap_obj->size);
#endif	

	/* see if we can shink the pool */
	if(heap_obj->next == NULL && ! (heap_obj->flag & HEAP_OBJECT_PREALLOCATED))
	{
		ASSERT(heap_mng->last_obj == heap_obj);
		while (NULL != (heap_obj = heap_mng->last_obj)
				&& ! (heap_obj->flag & (HEAP_OBJECT_PREALLOCATED|HEAP_OBJECT_IN_USE)))
		{
			heap_mng->cur_offset -=  heap_obj->size;
			heap_mng->available_memory += heap_obj->size;
			log2_size = CeilLog2(heap_obj->size);
			if ( log2_size >= 32 ) {
				ASSERT( FALSE );
				return FOVERRUN;
			}
			list = &heap_mng->bucket_lists[log2_size];
			QListRemoveItem(list, &heap_obj->u.list_item);
			heap_mng->last_obj = heap_obj->prev;
			if(heap_obj->prev)
				heap_obj->prev->next = NULL;
#if HEAP_DEBUG
			heap_mng->malloc_count--;
			MsgOut("Heap_OBJ: %p freed cur_offset: %d available memory: %d\n",heap_obj,heap_mng->cur_offset,
							heap_mng->available_memory);
#endif			
			MemoryDeallocate(heap_obj);
		}
	}

	SpinLockRelease(&heap_mng->heap_lock);		
	
#if HEAP_MANAGER_DEBUG
	MsgOut("] HeapManagerDeallocate\n");
#endif
	return FSUCCESS;
}

void HeapManagerRelease(HEAP_MANAGER *heap_mng,HEAP_OBJECT *heap_obj)
{
#if HEAP_MANAGER_DEBUG
	MsgOut("HeapManagerRelease (%p, %p) [\n", heap_mng, heap_obj);
#endif

	if (AtomicRead(&heap_obj->u.info.refcount) == 1)
	{
		HeapManagerDeallocate(heap_mng,heap_obj);
	} else {
		AtomicDecrementVoid(&heap_obj->u.info.refcount);
	}
#if HEAP_MANAGER_DEBUG
	MsgOut("] HeapManagerRelease\n");
#endif
}
