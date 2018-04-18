/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

#include "datatypes.h"
#include "imemory.h"
#include "imath.h"
#include "idebug.h"
#include "iquickmap.h"
#include "ispinlock.h"
#include "errno.h"
#include "string.h"
#include <sys/mman.h>

#ifdef IB_STACK_IBACCESS
static uint32 s_lock_strategy = 0xffffffff;

/* 
 * Certain redhat kernels use these defines without providing them. 
 * This allows us to support these features with such kernels.
 *
 * Note that this has no impact on kernels that really don't support
 * this feature.
 */
#if !defined(MADV_DONTFORK)
#define MADV_DONTFORK 10
#endif
#if !defined(MADV_DOFORK)
#define MADV_DOFORK 11
#endif

static FSTATUS
MemoryLockPrepareMlock(uintn start, uintn length);
static FSTATUS
MemoryLockUnprepareMlock(uintn start, uintn length);

#endif /* IB_STACK_IBACCESS */

void*
MemoryAllocatePriv( IN uint32 Bytes, IN uint32 flags, IN uint32 Tag )
{
    return malloc( Bytes );
}

void
MemoryDeallocatePriv( IN void *pMemory )
{
    free( pMemory );
}

void
MemoryFill( IN void *pMemory, IN uchar Fill, IN uint32 Bytes )
{
    memset( pMemory, Fill, Bytes );
} 

void*
MemoryCopy( IN void *pDest, IN const void *pSrc, IN uint32 Bytes )
{
    return memcpy( pDest, pSrc, Bytes );
}

int32
MemoryCompare( IN const void *pMemory1, IN const void *pMemory2, IN uint32 Bytes )
{
    return memcmp( pMemory1, pMemory2, Bytes );
} 

#ifdef IB_STACK_IBACCESS
FSTATUS MemoryLockPrepare(uintn buf_org, uintn Length)
{
	uintn buf_aligned = 0;
	uintn size = Length;
	uint32 page_size = getpagesize();
	uintn tos_aligned;

	buf_aligned = ROUNDDOWNP2(buf_org, page_size);
	size = size + buf_org - buf_aligned;
	size = ROUNDUPP2(size, page_size);
	// This is an odd case, if buf_aligned is on the stack, we need to make sure
	// the stack itself is at least 1 page beyond the buf itself
	// we use &tos_aligned as an rough indication of top of stack
	// if we didn't do this, the VMA created by madvise could actually grow
	// after we locked the memory, which would confuse the memory locking code

	// for stacks which grow down:
	tos_aligned = ROUNDDOWNP2(&tos_aligned, page_size);
	if (tos_aligned == buf_aligned)
	{
		// force stack to grow by 1 page so TOS is on a different page than buf
		volatile uint8 *temp = (uint8*)alloca(page_size);
		if (temp == NULL)
			return FINSUFFICIENT_MEMORY;
		*temp = 1;	// touch new page so stack grows
	}
#if 0
	// for stacks which grow up:
	tos_aligned = ROUNDUPP2(&tos_aligned, page_size);
	if ( buf_aligned + size == tos_aligned)
	{
		// force stack to grow by 1 page so TOS is on a different page than buf
		volatile uint8 *temp = (uint8*)alloca(page_size);
		if (temp == NULL)
			return FINSUFFICIENT_MEMORY;;		
		*(temp+page_size) = 1;	// touch new page so stack grows
	}
#endif

	switch (s_lock_strategy) {
	case MEMORY_LOCK_STRAT_MADVISE:
#if MLOCK_DBG > 1
		MsgOut("%s: madvise addr:0x%"PRIxN" size:0x%"PRIxN"\n",__func__,buf_aligned,size);
#endif
		if(madvise((void *)buf_aligned, (size_t)size, MADV_SEQUENTIAL ))
		{
			MsgOut("%s: madvise failed with buf:0x%"PRIxN" size:%"PRIdN"\n",__func__,buf_aligned,size);
			return FINSUFFICIENT_RESOURCES;;		
		}
		break;

	case MEMORY_LOCK_STRAT_MLOCK:
		return MemoryLockPrepareMlock(buf_aligned, size);
		break;

	default:
		/* strategy not supported or MemoryLockSetStrategy not called first */
		return FINVALID_OPERATION;
	}
	return FSUCCESS;
}

FSTATUS MemoryLockUnprepare(uintn buf_org, uintn Length)
{
	switch (s_lock_strategy) {
	case MEMORY_LOCK_STRAT_MADVISE:
		/* no action needed */
		return FSUCCESS;
		break;

	case MEMORY_LOCK_STRAT_MLOCK:
		{
		uint32 page_size = getpagesize();
		uintn buf_aligned = ROUNDDOWNP2(buf_org,page_size);
		uintn size = Length + buf_org - buf_aligned;
		size = ROUNDUPP2(size, page_size);
		return MemoryLockUnprepareMlock(buf_aligned, size);
		break;
		}

	default:
		/* strategy not supported or MemoryLockSetStrategy not called first */
		return FINVALID_OPERATION;
	}
}

/* This provides to the User Memory module the memory lock strategy which Uvca
 * obtained via Vka from the kernel Memory module
 */
FSTATUS MemorySetLockStrategy(uint32 strategy)
{
	switch (strategy) {
	case MEMORY_LOCK_STRAT_MADVISE:
		s_lock_strategy = strategy;
		return FSUCCESS;
		break;

	case MEMORY_LOCK_STRAT_MLOCK:
		s_lock_strategy = strategy;
		return FSUCCESS;
		break;

	default:
		return FINVALID_PARAMETER;
		break;
	}
}

/* Tell UVca if MemoryLockUnprepare is necessary for the configured locking
 * strategy.  If not, UVCA has the option of optimizing its operation to
 * avoid the MemoryLockUnprepare.
 * However, if unnecessary, MemoryLockUnprepare should be a noop and should
 * not fail (hence allowing UVCA to choose not to optimize it out).
 */
boolean MemoryNeedLockUnprepare(void)
{
	switch (s_lock_strategy) {
	case MEMORY_LOCK_STRAT_MADVISE:
		return FALSE;
		break;
	case MEMORY_LOCK_STRAT_MLOCK:
		return TRUE;
		break;
	default:	/* unexpected case, play it safe */
		return TRUE;
		break;
	}
}


/* ********************************************************************** */
/* This implements the MEMORY_LOCK_STRAT_MLOCK core functionality
 * in this strategy the user space uses mlock and madvise prior to
 * the kernel MemoryLock being called.  Since mlock and madvise do not "stack"
 * we must track the presently locked regions and only call mlock/madvise for
 * the "delta" caused by the given lock/unlock of memory.
 * note that MemoryLockPrepare is called before MemoryLock
 * and MemoryLockUnprepare is called after MemoryUnlock
 *
 * This implementation of MemoryLockPrepare/MemoryLockUnprepare "stacks"
 * This assumes every MemoryLockUnprepare exactly matches a previous
 * call to MemoryLockPrepare.  It will assert otherwise.
 * UVCA follows this requirement.
 */

#define MLOCK_TAG	MAKE_MEM_TAG(k,c,l,m)

/* initial testing indicates most applications have relatively few
 * locked areas at a time, hence the cost of compressing then uncompressing
 * them (on unlock) is more expensive that having more areas to track and store
 * especially since the locked_area structure is relatively small and a
 * cl_qmap is used to search them
 */
#define COMPRESS_AREAS 0	/* conserve memory by merging areas when possible */

typedef struct _locked_area {
	cl_map_item_t MapItem;
	uintn		start;	// virtual address
	uintn		end;	// last address included in area
	uint32		lock_cnt;// number of times locked
} LOCKED_AREA;

typedef struct _addr_range {
	uintn		start;	// virtual address
	uintn		end;	// last address included in range
} ADDR_RANGE;

static cl_qmap_t	LockedArea_Map;
static SPIN_LOCK 	LockedAreaMap_Lock;
#if MLOCK_DBG
static unsigned s_num_locks = 0;	/* count of MemoryLockPrepare calls */
static unsigned s_num_split = 0;	/* count of LOCKED_AREAs split */
static unsigned s_num_merge = 0;	/* count of LOCKED_AREAs merged */
static unsigned s_max_areas = 0;	/* maximum number of areas */
#endif

#if MLOCK_DBG
/* dump all the locked areas, must be called with LockedAreaMap_Lock held */
static void LockedAreaDump(void)
{
	cl_map_item_t *pMapItem;

	MsgOut("      start              end               cnt\n");
	for (pMapItem = cl_qmap_head(&LockedArea_Map);
		 pMapItem != cl_qmap_end(&LockedArea_Map);
		 pMapItem = cl_qmap_next(pMapItem))
	{
		LOCKED_AREA *pArea = PARENT_STRUCT(pMapItem, LOCKED_AREA, MapItem);

		MsgOut("0x%16.16"PRIxN" 0x%16.16"PRIxN"  %10u\n", pArea->start, pArea->end, pArea->lock_cnt);
	}
}
#endif

/* compare start addresses of 2 entries in map
 * used as primary sort for LockedArea_Map.  The map should not have any
 * overlapping areas, hence sort by start address is sufficient
 * Return:
 *	-1: key1 < key 2
 *	 0: key1 = key 2
 *	 1: key1 > key 2
 */
static int LockedAreaCompare(uint64 key1, uint64 key2)
{
	LOCKED_AREA *pArea1 = (LOCKED_AREA*)(uintn)key1;
	LOCKED_AREA *pArea2 = (LOCKED_AREA*)(uintn)key2;

	if (pArea1->start < pArea2->start)
		return -1;
	else if (pArea1->start == pArea2->start)
		return 0;
	else
		return 1;
}

/* compare address range in key2 against address range in key1
 * key1 is a LOCKED_AREA*, key2 is a ADDR_RANGE*
 * Return:
 *	-1: key1 < key 2 (eg. key 2 is above range of key 1)
 *	 0: key1 = key 2 (eg. key 2 overlaps with range of key 1)
 *	 1: key1 > key 2 (eg. key 2 is below range of key 1)
 * Note that more than 1 range in the LockedArea_Map could overlap
 * a given value for key2
 */
static int LockedAreaRangeCompare(uint64 key1, uint64 key2)
{
	LOCKED_AREA *pArea1 = (LOCKED_AREA*)(uintn)key1;
	ADDR_RANGE *pRange2 = (ADDR_RANGE*)(uintn)key2;

	if (pArea1->start > pRange2->end)
		return 1;
	else if (pArea1->end < pRange2->start)
		return -1;
	else 
		return 0;
}

/* convert a pMapItem into a pArea, returns NULL if "end marker" */
static LOCKED_AREA *LockedAreaFromMapItem(cl_map_item_t *pMapItem)
{
	if (pMapItem != cl_qmap_end(&LockedArea_Map)) {
		return PARENT_STRUCT(pMapItem, LOCKED_AREA, MapItem);
	} else {
		return NULL;
	}
}

/* next area after pArea, returns NULL if no areas after pArea */
static _inline LOCKED_AREA *LockedAreaNext(LOCKED_AREA *pArea)
{
	return (LockedAreaFromMapItem(cl_qmap_next(&pArea->MapItem)));
}

/* prev area before pArea, returns NULL if no areas before pArea */
static _inline LOCKED_AREA *LockedAreaPrev(LOCKED_AREA *pArea)
{
	return (LockedAreaFromMapItem(cl_qmap_prev(&pArea->MapItem)));
}

/* must call with Map locked
 * merges pArea1 and pArea2 (which must be adjacent areas)
 * returns resulting merged area (Area2 is freed)
 * cannot fail
 */
static LOCKED_AREA *LockedAreaMerge(LOCKED_AREA *pArea1, LOCKED_AREA *pArea2)
{
	ASSERT(pArea1->end+1 == pArea2->start);
	ASSERT(pArea1->lock_cnt == pArea2->lock_cnt);
	pArea1->end = pArea2->end;
	cl_qmap_remove_item(&LockedArea_Map, &pArea2->MapItem);
#if MLOCK_DBG
	s_num_merge++;
#endif
	return pArea1;
}

/* create a new LOCKED_AREA and insert it into LockedArea_Map
 * returns NULL on memory allocation errors, otherwise pointer to new area
 */
static LOCKED_AREA *LockedAreaAlloc(uintn start, uintn end, uint32 lock_cnt)
{
	LOCKED_AREA *pNewArea;
	cl_map_item_t *pMapItem;

	pNewArea = (LOCKED_AREA*)MemoryAllocate2(
			sizeof(LOCKED_AREA), IBA_MEM_FLAG_NONE, MLOCK_TAG);
	if (! pNewArea)
		return NULL;
	pNewArea->start = start;
	pNewArea->end = end;
	pNewArea->lock_cnt = lock_cnt;
	pMapItem = cl_qmap_insert(&LockedArea_Map, (uintn)pNewArea, &pNewArea->MapItem);
	// assert new area is unique
	ASSERT(pMapItem == &pNewArea->MapItem);
#if MLOCK_DBG
	if (cl_qmap_count(&LockedArea_Map) > s_max_areas)
		s_max_areas = cl_qmap_count(&LockedArea_Map);
#endif
	return pNewArea;
}

/* must call with Map locked
 * splits pArea into 2 areas at addr
 * 1st area will be pArea->start to addr-1
 * 2nd area will be addr to pArea->end
 * returns 2nd area (pArea becomes 1st area)
 * returns NULL on failure to allocate memory
 */
static LOCKED_AREA *LockedAreaSplit(LOCKED_AREA *pArea, uintn addr)
{
	LOCKED_AREA *pNewArea;
	cl_map_item_t *pMapItem;

	ASSERT(pArea->start < addr);
	ASSERT(pArea->end >= addr);
	pNewArea = (LOCKED_AREA*)MemoryAllocate2(
			sizeof(LOCKED_AREA), IBA_MEM_FLAG_NONE, MLOCK_TAG);
	if (! pNewArea)
		return NULL;
	pNewArea->start = addr;
	pNewArea->end = pArea->end;
	pNewArea->lock_cnt = pArea->lock_cnt;
	pArea->end = addr-1;	// must fixup before insert pNewArea
	pMapItem = cl_qmap_insert(&LockedArea_Map, (uintn)pNewArea, &pNewArea->MapItem);
	ASSERT(pMapItem == &pNewArea->MapItem); /* assert new area is unique */
#if MLOCK_DBG
	s_num_split++;
	if (cl_qmap_count(&LockedArea_Map) > s_max_areas)
		s_max_areas = cl_qmap_count(&LockedArea_Map);
#endif
	return pNewArea;
}

/* must call with Map locked
 * find the 1st area in LockedArea_Map which overlaps start-end
 * returns NULL if no overlapping area(s)
 */
static LOCKED_AREA *FindFirstOverlap(uintn start, uintn end)
{
	ADDR_RANGE Range;
	cl_map_item_t *pMapItem;
	LOCKED_AREA *pArea;

	Range.start = start;
	Range.end = end;

	/* first find if any locked areas overlap the Range */
	pMapItem = cl_qmap_get_compare(&LockedArea_Map, (uint64)(uintn)&Range, LockedAreaRangeCompare);
	if (pMapItem == cl_qmap_end(&LockedArea_Map)) {
		/* no overlap with existing locks */
		return NULL;
	}
	pArea = PARENT_STRUCT(pMapItem, LOCKED_AREA, MapItem);
	/* because RangeCompare is not unique, we could get any area which
	 * overlaps, so we must back up to find first area which overlaps
	 */
	while ((pMapItem = cl_qmap_prev(&pArea->MapItem)) != cl_qmap_end(&LockedArea_Map)) {
		if (LockedAreaRangeCompare(cl_qmap_key(pMapItem), (uint64)(uintn)&Range) != 0) {
			// no overlap, pArea is 1st to overlap Range
			break;
		}
		pArea = PARENT_STRUCT(pMapItem, LOCKED_AREA, MapItem);
	}
	/* now pArea is 1st area to overlap Range */
	return pArea;
}

/* account for the locking/unlocking of start-end in pArea
 * as needed we could split pArea into up to 2 pieces
 * the returned value is the area which overlaps desired start/end
 * returns NULL on error
 */
static LOCKED_AREA *LockedAreaAddLock(LOCKED_AREA *pArea, uintn start, uintn end, uint32 add)
{
	LOCKED_AREA *p = NULL;

	if (pArea->start < start) {
		ASSERT(pArea->end >= start);
		p = pArea;	// save for recovery
		pArea = LockedAreaSplit(pArea, start);	// 2nd half
		if (! pArea)
			return NULL;	// failed
	}
	if (pArea->end > end) {
		ASSERT(pArea->start <= end);
		if (! LockedAreaSplit(pArea, end+1)) {
			// error, undo split above
			if (p)
				(void)LockedAreaMerge(p, pArea);
			return NULL;	// failed
		}
		// pArea is still 1st half
	}
	pArea->lock_cnt += add;
	return pArea;
}

static _inline LOCKED_AREA *LockedAreaIncLock(LOCKED_AREA *pArea, uintn start, uintn end)
{
	return LockedAreaAddLock(pArea, start, end, 1);
}

static _inline LOCKED_AREA *LockedAreaDecLock(LOCKED_AREA *pArea, uintn start, uintn end)
{
	return LockedAreaAddLock(pArea, start, end, (uint32)-1);
}

/* create a new locked area covering start-end */
static LOCKED_AREA *LockedAreaCreate(uintn start, uintn end)
{
	LOCKED_AREA *pArea;
	uintn length = (end-start)+1;

#if MLOCK_DBG > 1
	MsgOut("%s: mlock addr:0x%"PRIxN" size:0x%"PRIxN"\n",__func__,start,length);
#endif
	if (mlock((void*)start, (size_t)length) < 0) {
		MsgOut("mlock failed: start=0x%p, len=%u, errno=%s (%d)\n",
			(void*)start, (unsigned)length, strerror(errno), errno);
		return NULL;
	}
#if MLOCK_DBG > 1
	MsgOut("%s: madvise DONTFORK addr:0x%"PRIxN" size:0x%"PRIxN"\n",__func__,start,length);
#endif
	if (madvise((void*)start, (size_t)length, MADV_DONTFORK) < 0) {
		MsgOut("madvise DONTFORK failed: start=0x%p, len=%u, errno=%s (%d)\n",
			(void*)start, (unsigned)length, strerror(errno), errno);
		(void)munlock((void*)start, (size_t)length);
		return NULL;
	}
	pArea = LockedAreaAlloc(start, end, 1);
	if (! pArea) {
		(void)madvise((void*)start, (size_t)length, MADV_DOFORK);
		(void)munlock((void*)start, (size_t)length);
	}
	return pArea;
}

/* destroy a locked area */
static void LockedAreaDestroy(LOCKED_AREA *pArea)
{
	uintn length = (pArea->end - pArea->start)+1;
#if MLOCK_DBG > 1
	MsgOut("%s: madvise DOFORK addr:0x%"PRIxN" size:0x%"PRIxN"\n",__func__,pArea->start,length);
#endif
	(void)madvise((void*)pArea->start, (size_t)length, MADV_DOFORK);
#if MLOCK_DBG > 1
	MsgOut("%s: munlock addr:0x%"PRIxN" size:0x%"PRIxN"\n",__func__,pArea->start,length);
#endif
	(void)munlock((void*)pArea->start, (size_t)length);
	cl_qmap_remove_item(&LockedArea_Map, &pArea->MapItem);
	MemoryDeallocate(pArea);
}

#if COMPRESS_AREAS
/* merge Areas with same lock count.  This saves space but
 * may make for more work in unlock
 */
static void CompressAreas(LOCKED_AREA *pFirstArea, uintn end)
{
	LOCKED_AREA *pArea;

	ASSERT(pFirstArea);
	/* start merge process with 1st area before this lock (if any) */
	pArea = LockedAreaPrev(pFirstArea);
	if (pArea)
		pFirstArea = pArea;
	while (pFirstArea->start <= end) {
		pArea = LockedAreaNext(pFirstArea);
		if (! pArea)
			break;
		if (pArea->lock_cnt == pFirstArea->lock_cnt
			&& pFirstArea->end+1 == pArea->start)
			pFirstArea = LockedAreaMerge(pFirstArea, pArea);
		else
			pFirstArea = pArea;
	}
}
#endif /* COMPRESS_AREAS */


/* heart of managing mlock/madvise locked areas.
 * start and length must be pagesize aligned
 */
static FSTATUS
MemoryLockPrepareMlock(uintn start, uintn length)
{
	LOCKED_AREA *pArea;
#if COMPRESS_AREAS
	LOCKED_AREA *pFirstArea;
#endif
	uintn end = start + length-1;
	uintn addr;

#if MLOCK_DBG > 1
	MsgOut("%s: addr:0x%"PRIxN" size:0x%"PRIxN"\n",__func__,start,length);
#endif
	if (! length)
		return FSUCCESS;
// TBD - should we just mlock and madvise whole area 1st outside spin lock?
// however error unwind would be impossible if madvise failed?
// does kernel properly count mlock if done twice to same page?
	SpinLockAcquire(&LockedAreaMap_Lock);
#if MLOCK_DBG
	s_num_locks++;
#endif
	pArea = FindFirstOverlap(start, end);
#if COMPRESS_AREAS
	pFirstArea = NULL;
#endif
	for (addr=start; addr<=end; ) {
		if (! pArea) {
			pArea = LockedAreaCreate(addr, end);
		} else if (pArea->start > addr) {
			// fill gap
			pArea = LockedAreaCreate(addr, MIN(end, pArea->start-1));
		} else {
			pArea = LockedAreaIncLock(pArea, addr, end);
		}
		if (! pArea) {
			// error, unwind what we did so far
			SpinLockRelease(&LockedAreaMap_Lock);
#if MLOCK_DBG
			MsgOut("failure for MemoryLockPrepare(0x%"PRIxN", 0x%"PRIxN") for 0x%"PRIxN"-0x%"PRIxN"\n", start, length, addr, end);
			LockedAreaDump();
			MsgOut("MemoryLockUnprepare(0x%"PRIxN", 0x%"PRIxN") for 0x%"PRIxN"-0x%"PRIxN"\n", start, length, addr, end);
#endif
			if (addr != start)
				(void)MemoryLockUnprepareMlock(start, addr-start);	
			return FINSUFFICIENT_RESOURCES;
		}
		// adjust to reflect what is left to do
		addr = pArea->end+1;
#if COMPRESS_AREAS
		if (! pFirstArea)
			pFirstArea = pArea;
#endif
		pArea = LockedAreaNext(pArea);
	}
#if COMPRESS_AREAS
	/* merge Areas with same lock count.  This saves space but
	 * may make for more work in unlock
	 */
	ASSERT(pFirstArea);
	CompressAreas(pFirstArea, end);
#endif
#if MLOCK_DBG > 1
	LockedAreaDump();
#endif
	SpinLockRelease(&LockedAreaMap_Lock);
	return FSUCCESS;
}

/* start and length must be pagesize aligned
 */
static FSTATUS
MemoryLockUnprepareMlock(uintn start, uintn length)
{
	LOCKED_AREA *pArea;
	LOCKED_AREA *p;
#if COMPRESS_AREAS
	LOCKED_AREA *pFirstArea;
#endif
	uintn end = start + length-1;
	uintn addr;

#if MLOCK_DBG > 1
	MsgOut("%s: addr:0x%"PRIxN" size:0x%"PRIxN"\n",__func__,start,length);
#endif
	if (! length)
		return FSUCCESS;
	SpinLockAcquire(&LockedAreaMap_Lock);
	pArea = FindFirstOverlap(start, end);
#if COMPRESS_AREAS
	pFirstArea = NULL;
#endif
	for (addr=start; addr<=end; ) {
		ASSERT(pArea);
		ASSERT(pArea->start <= addr);
		pArea = LockedAreaDecLock(pArea, addr, end);
#if COMPRESS_AREAS
		if (! pArea) {
			// stuck, can't really re-lock it, so leave it
			// partially unlocked and give up
			DbgOut("Unable to unlock: addr=0x%p, end=0x%p\n",
				(void*)addr, (void*)end);
			break;
		}
#else
		// since never merge, should be no need to split, so should not fail
		ASSERT(pArea);
#endif
		// adjust to reflect what is left to do
		addr = pArea->end+1;
		p = LockedAreaNext(pArea);
		if (! pArea->lock_cnt) {
			LockedAreaDestroy(pArea);
#if COMPRESS_AREAS
		} else {
			if (! pFirstArea)
				pFirstArea = pArea;
#endif
		}
		pArea = p;
	}
#if COMPRESS_AREAS
	/* merge Areas with same lock count */
	if (pFirstArea)
		CompressAreas(pFirstArea, end);
#endif
#if MLOCK_DBG > 1
	LockedAreaDump();
#endif
	SpinLockRelease(&LockedAreaMap_Lock);
	return FSUCCESS;
}
/* end of MEMORY_LOCK_STRAT_MLOCK specific code */
/* ********************************************************************** */


FSTATUS
MemoryLockPrepareInit(void)
{

	cl_qmap_init(&LockedArea_Map, LockedAreaCompare);
	SpinLockInitState( &LockedAreaMap_Lock );

	if ( !SpinLockInit ( &LockedAreaMap_Lock ) )
	{
		MsgOut ("MemoryLockPrepareInit: Locked Area SpinLock init failed\n");
		return FERROR;
	}


	return FSUCCESS;
}

void
MemoryLockPrepareCleanup(void)
{

#if MLOCK_DBG
	MsgOut("MLock Stats: numLocks: %u, maxAreas: %u, numSplit: %u, numMerge: %u\n",
			s_num_locks, s_max_areas, s_num_split, s_num_merge);
#endif
	if (! cl_is_qmap_empty(&LockedArea_Map)) {
		DbgOut("MemoryLockPrepareCleanup: Locked Area Map not empty\n");
	}
	SpinLockDestroy( &LockedAreaMap_Lock );

} 
#endif /* IB_STACK_IBACCESS */
