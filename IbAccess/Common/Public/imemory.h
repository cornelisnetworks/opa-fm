/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_PUBLIC_IMEMORY_H_
#define _IBA_PUBLIC_IMEMORY_H_


#include "iba/public/datatypes.h"
#include "iba/public/imath.h"
#ifdef _NATIVE_MEMORY
#include "iba/public/imemory_osd.h"
#endif

#ifndef MAKE_MEM_TAG_DEFINED
#define MAKE_MEM_TAG( a, b, c, d )	0
#define MAKE_MEM_TAG_DEFINED
#endif	/* MAKE_MEM_TAG */

#ifdef __cplusplus
extern "C"
{
#endif
/* Enable and disables memory lock debug */
#ifndef MLOCK_DBG	/* allow to be set from CFLAGS */
#define MLOCK_DBG	0					/* 0=no, 1=validate, 2=validate+log */
#endif

/* flags for memory allocation controls */
#define IBA_MEM_FLAG_NONE			0x0000
#define IBA_MEM_FLAG_LEGACY			0x0001	/* old style mode */
#define IBA_MEM_FLAG_PREMPTABLE		0x0002	/* may sleep during alloc/free */
#define IBA_MEM_FLAG_PAGEABLE		0x0004	/* memory allocated may be pageable */
#define IBA_MEM_FLAG_SHORT_DURATION	0x0008	/* memory will be freed soon */
#define IBA_MEM_FLAG_PREFER_CONTIG	0x0010	/* prefer phys contig pages */
#define IBA_MEM_FLAG_INTR_SVC_THREAD 0x0020	/* in an IntrServiceThread, no locks */

/* Enables and disables tracking of memory allocation. */
void
MemoryTrackUsage( 
	IN boolean Start );

/* Prints all memory allocation information. */
void
MemoryDisplayUsage( int method, uint32 minSize, uint32 minTick );

/* Turn on memory allocation tracking in debug builds if not already turned on. */
#if !defined(MALLOC_TRACK_ON)
	#define MALLOC_TRACK_ON
#endif

#ifdef MEM_TRACK_ON
/* Debug versions of memory allocation calls store the file name
 * and line number where invoked in a header for use in tracking memory
 * leaks.
 */
#define MemoryAllocate( a, b, c )	\
	MemoryAllocate2Dbg( __FILE__, __LINE__, a, (b?IBA_MEM_FLAG_PAGEABLE:IBA_MEM_FLAG_NONE)|IBA_MEM_FLAG_LEGACY, c )

#define MemoryAllocateAndClear( a, b, c )	\
	MemoryAllocate2AndClearDbg( __FILE__, __LINE__, a, (b?IBA_MEM_FLAG_PAGEABLE:IBA_MEM_FLAG_NONE)|IBA_MEM_FLAG_LEGACY, c )

#define MemoryAllocateObjectArray( a, b, c, d, e, f, g, h )	\
	MemoryAllocateObjectArrayDbg( __FILE__, __LINE__, a, b, c, d, e, f, g, h )

#define MemoryAllocate2( a, b, c )	\
	MemoryAllocate2Dbg( __FILE__, __LINE__, a, b, c )

#define MemoryAllocate2AndClear( a, b, c )	\
	MemoryAllocate2AndClearDbg( __FILE__, __LINE__, a, b, c )

#else

#define MemoryAllocate( a, b, c )	\
	MemoryAllocate2Rel( a, (b?IBA_MEM_FLAG_PAGEABLE:IBA_MEM_FLAG_NONE)|IBA_MEM_FLAG_LEGACY, c )

#define MemoryAllocateAndClear( a, b, c )	\
	MemoryAllocate2AndClearRel( a, (b?IBA_MEM_FLAG_PAGEABLE:IBA_MEM_FLAG_NONE)|IBA_MEM_FLAG_LEGACY, c )

#define MemoryAllocateObjectArray( a, b, c, d, e, f, g, h )	\
	MemoryAllocateObjectArrayRel(a, b, c, d, e, f, g, h )

#define MemoryAllocate2( a, b, c )	\
	MemoryAllocate2Rel(a, b, c )

#define MemoryAllocate2AndClear( a, b, c )	\
	MemoryAllocate2AndClearRel( a, b, c )

#endif



/* Allocate memory region. */
void*
MemoryAllocateDbg( 
	IN const char *pFileName, 
	IN int32 nLine, 
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag );
void*
MemoryAllocateRel( 
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag );

void*
MemoryAllocate2Dbg( 
	IN const char *pFileName, 
	IN int32 nLine, 
	IN uint32 Bytes, 
	IN uint32 flags,
	IN uint32 Tag );
void*
MemoryAllocate2Rel( 
	IN uint32 Bytes, 
	IN uint32 flags,
	IN uint32 Tag );

/* Allocate memory region and set the memory to 0. */
void*
MemoryAllocateAndClearDbg( 
	IN const char *pFileName, 
	IN int32 nLine,
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag );
void*
MemoryAllocateAndClearRel( 
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag );

void*
MemoryAllocate2AndClearDbg( 
	IN const char *pFileName, 
	IN int32 nLine,
	IN uint32 Bytes, 
	IN uint32 flags,
	IN uint32 Tag );
void*
MemoryAllocate2AndClearRel( 
	IN uint32 Bytes, 
	IN uint32 flags,
	IN uint32 Tag );

/* Allocate an array of objects alligned on arbitrary byte boundary. */
void*
MemoryAllocateObjectArrayDbg( 
	IN const char *pFileName, 
	IN int32 nLine,
	IN uint32 ObjectCount, 
	IN OUT uint32 *pObjectSize,  
	IN uint32 ByteAlignment,
	IN uint32 AlignmentOffset,
	IN boolean IsPageable, 
	IN uint32 Tag,
	OUT void **ppFirstObject, 
	OUT uint32 *pArraySize );
void*
MemoryAllocateObjectArrayRel(
	IN uint32 ObjectCount, 
	IN OUT uint32 *pObjectSize,  
	IN uint32 ByteAlignment, 
	IN uint32 AlignmentOffset,
	IN boolean IsPageable, 
	IN uint32 Tag,
	OUT void **ppFirstObject, 
	OUT uint32 *pArraySize );

/* Release allocated memory. */
int
MemoryDeallocate( 
	IN void *pMemory );

/* Fill allocated memory with a specified value. */
void
MemoryFill( 
	IN void *pMemory, 
	IN uchar Fill, 
	IN uint32 Bytes );

/* Clear allocated memory. */
void
MemoryClear( 
	IN void *pMemory, 
	IN uint32 Bytes );

/* Copy one buffer into another. */
void*
MemoryCopy( 
	IN void *pDest, 
	IN const void *pSrc, 
	IN uint32 Bytes );

/* Compare two buffers.  Functions similarly to the standard C memcmp call. */
int32
MemoryCompare( 
	IN const void *pMemory1, 
	IN const void *pMemory2, 
	IN uint32 Bytes );

/* round address down to a page boundary and adjust Length to
 * end of a page boundary
 * page_size must be a power of 2
 */
static _inline void MemoryFixAddrLength(IN uint32 page_size,
						IN OUT void** VirtualAddress,
						IN OUT uintn* Length)
{
	/* bytes between page boundary and VirtualAddress */
	uintn start_offset = ((uintn)*VirtualAddress) & (page_size-1);

	/* round down virtual address to a page boundary */
	*VirtualAddress = (void*)((uintn)*VirtualAddress - start_offset);
	*Length += start_offset;

	/* round up Length to a page boundary */
	*Length = ROUNDUPP2(*Length, page_size);
}
/**
 * Safe version of strncpy that will always null terminate. Copies up to
 * dstsize-1 characters from source to dest plus a null terminator.
 *
 * @param dest     A pointer to destination buffer
 * @param source   A pointer to source buffer
 * @param dstsize  Size of dest buffer
 *
 * @return dest on Success or NULL on failure
 */
static _inline char * StringCopy(char * dest, const char * source, size_t dstsize)
{
	if(!dest || !dstsize || !source)
		return NULL;

#if defined(VXWORKS)
	size_t l = MIN(dstsize-1,strlen(source));
#else
	size_t l = strnlen(source, dstsize-1);
#endif
	memcpy(dest,source,l);
	dest[l] = '\0';
	return dest;
}

/**
 * Concatenates all of the given strings into one long string. The returned
 * string should be freed by #free. Variable arguments must end with NULL.
 * Otherwise the function will be appending random memory junks.
 * @param str1		first string to be concatenated
 * @param ...		a NULL-terminated list of strings
 * @return	newly allocated string containing all provided strings
 */
char* StringConcat(const char* str1, ...);


// convert a string to a uint or int
// Very similar to strtoull/strtoll except that
// base=0 implies base 10 or 16, but excludes base 8
// hence allowing leading 0's for base 10.
//
// Also provides easier to use status returns and error checking.
//
// Can also optionally skip trailing whitespace, when skip_trail_whitespace is
// FALSE, trailing whitespace is treated as a FERROR
//
// When endptr is NULL, trailing characters (after optional whitespace) are 
// considered an FERROR.  When endptr is non-NULL, for a FSUCCESS conversion,
// it points to the characters after the optional trailing whitespace.
// Errors:
//	FSUCCESS - successful conversion, *endptr points to 1st char after value
//	FERROR - invalid contents, non-numeric, *endptr and *value undefined
//	FINVALID_SETTING - value out of range, *endptr and *value undefined
//	FINVALID_PARAMETER - invalid function arguments (NULL value or str)
FSTATUS StringToUint64(uint64 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace);
FSTATUS StringToUint32(uint32 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace);
FSTATUS StringToUint16(uint16 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace);
FSTATUS StringToUint8(uint8 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace);
FSTATUS StringToInt64(int64 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace);
FSTATUS StringToInt32(int32 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace);
FSTATUS StringToInt16(int16 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace);
FSTATUS StringToInt8(int8 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace);
// GID of form hValue:lValue
// values must be base 16, as such 0x prefix is optional for both hValue and
// lValue
// whitespace is permitted before and after :
FSTATUS StringToGid(uint64 *hValue, uint64 *lValue, const char* str, char **endptr, boolean skip_trail_whitespace);
// VESWPort of form guid:port:index or desc:port:index
// guid must be base 16, as such 0x prefix is optional.
// desc max size is MAX_VFABRIC_NAME (64)
// port and index are decimal
// whitespace is permitted before and after :
// byname = 0: the format is guid:port:index
// byname = 1: the format is desc:port:index
FSTATUS StringToVeswPort(uint64 *guid, char *desc, uint32 *port, uint32 *index,
	const char* str, char **endptr, boolean skip_trail_whitespace,
	boolean byname);

// MAC Address of form %02x:%02x:%02x:%02x:%02x:%02x
// values must be base16, 0x prefix is optional
FSTATUS StringToMAC(uint8_t *MAC,const char *str, char **endptr,
					boolean skip_trail_whitespace);

// Byte Count as an integer followed by an optional suffix of:
// K, KB, M, MB, G or GB
// (K==KB, etc)
// converted to an absolute number of bytes
FSTATUS StringToUint64Bytes(uint64 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace);
/**
 * StringToDateTime - parse a string to return a date/time
 *
 * @param value  - will contain date as # of seconds since epoch if parsing successful
 * @param str    - the string representation of the date to be parsed
 *
 * @return - FSTATUS indicating success or failure of parsing:
 * 			 FSUCCESS - successful parsing
 * 			 FINVALID_PARAMETER - str not valid date/time or not in valid format
 * 			 FINVALID_SETTING - relative value out of range
 * 			 FINSUFFICIENT_MEMORY - memory could not be allocated to parse string
 * 			 FERROR - other error parsing string
 */
FSTATUS StringToDateTime(uint32 *value, const char * str);

// Tuple of form select:comparator:argument
// select must be one of "utilization", "pktrate", "integrity", "congestion", "smacongestion", "bubbles", "security", or "routing".
// comparator must be one of "gt", "ge", "lt", "le".
// argument may be any 64-bit value
FSTATUS StringToTuple(uint32 *select, uint8 *comparator, uint64 *argument, char* str, char **endptr);

#if defined(VXWORKS)
/* internal private functions not for general use, these are supplied
 * by the OS specific imemory_osd module
 */
void*
MemoryAllocatePhysContPriv( IN uint32 Bytes, void *caller );

void
MemoryDeallocatePhysContPriv( IN void *pMemory, void *caller );

/* macros for general use */
#ifdef MEM_TRACK_ON
#define MemoryAllocatePhysCont( a )	\
	MemoryAllocatePhysContDbg( __FILE__, __LINE__, a )
#else
#define MemoryAllocatePhysCont( a )	\
	MemoryAllocatePhysContRel( a )
#endif

/* internal routines, not for general use */
void*
MemoryAllocatePhysContDbg( 
	IN const char *pFileName, 
	IN int32 nLine, 
	IN uint32 Bytes );

void*
MemoryAllocatePhysContRel( 
	IN uint32 Bytes );

void
MemoryDeallocatePhysCont( 
	IN void *pMemory );

/* allocate physically contiguous memory aligned to a "align" boundary
 *
 * INPUT:
 *		size - size of memory in bytes
 *		align - power of 2 to align to
 * OUTPUT:
 *		aligned_addr - aligned address
 * RETURNS:
 *		actual start of memory allocated, use this for MemoryDeallocate
 *		NULL on error
 */
#ifdef MEM_TRACK_ON
#define MemoryAllocatePhysContAligned( a, b, c )	\
	MemoryAllocatePhysContAlignedDbg( __FILE__, __LINE__, a, b, c )
static _inline void* MemoryAllocatePhysContAlignedDbg(
	IN const char *pFileName, 
	IN int32 nLine,
#else
static _inline void* MemoryAllocatePhysContAligned(
#endif
	IN uint32 size,
	IN uint32 align,
	OUT void **aligned_addr
	)
{
	void *buf;

#ifdef MEM_TRACK_ON
	buf = MemoryAllocatePhysContDbg(pFileName, nLine, size+align-1);
#else
	buf = MemoryAllocatePhysCont(size+align-1);
#endif
	*aligned_addr = (void*)ROUNDUPP2((uintn)buf,align);
	return buf;
}
#ifdef MEM_TRACK_ON
#define MemoryAllocatePhysContAlignedAndClear( a, b, c )	\
	MemoryAllocatePhysContAlignedAndClearDbg( __FILE__, __LINE__, a, b, c )
static _inline void* MemoryAllocatePhysContAlignedAndClearDbg(
	IN const char *pFileName, 
	IN int32 nLine,
#else
static _inline void* MemoryAllocatePhysContAlignedAndClear(
#endif
	IN uint32 size,
	IN uint32 align,
	OUT void **aligned_addr
	)
{
#ifdef MEM_TRACK_ON
	void *buf = MemoryAllocatePhysContAlignedDbg(pFileName, nLine, size, align, aligned_addr);
#else
	void *buf = MemoryAllocatePhysContAligned(size, align, aligned_addr);
#endif
	if (buf)
		MemoryClear(*aligned_addr, size);
	return buf;
}
#endif /* defined(VXWORKS) */

/* allocate memory aligned to a "align" boundary
 *
 * INPUT:
 *		size - size of memory in bytes
 *		align - power of 2 to align to
 *		tag - tag for memory tracker
 * OUTPUT:
 *		aligned_addr - aligned address
 * RETURNS:
 *		actual start of memory allocated, use this for MemoryDeallocate
 *		NULL on error
 */
#ifdef MEM_TRACK_ON
#define MemoryAllocateAligned( a, b, c, d, e )	\
	MemoryAllocateAlignedDbg( __FILE__, __LINE__, a, b, c, d, e )
static _inline void* MemoryAllocateAlignedDbg(
	IN const char *pFileName, 
	IN int32 nLine,
#else
static _inline void* MemoryAllocateAligned(
#endif
	IN uint32 size,
	IN uint32 align,
	IN boolean IsPageable, 
	IN uint32 tag,
	OUT void **aligned_addr
	)
{
	void *buf;

#ifdef MEM_TRACK_ON
	buf = MemoryAllocateDbg(pFileName, nLine, size+align-1, IsPageable, tag);
#else
	buf = MemoryAllocate(size+align-1, IsPageable, tag);
#endif
	*aligned_addr = (void*)ROUNDUPP2((uintn)buf,align);
	return buf;
}

#ifdef MEM_TRACK_ON
#define MemoryAllocateAlignedAndClear( a, b, c, d, e )	\
	MemoryAllocateAlignedAndClearDbg( __FILE__, __LINE__, a, b, c, d, e )
static _inline void* MemoryAllocateAlignedAndClearDbg(
	IN const char *pFileName, 
	IN int32 nLine,
#else
static _inline void* MemoryAllocateAlignedAndClear(
#endif
	IN uint32 size,
	IN uint32 align,
	IN boolean IsPageable, 
	IN uint32 tag,
	OUT void **aligned_addr
	)
{
	void *buf;

#ifdef MEM_TRACK_ON
	buf = MemoryAllocateAlignedDbg(pFileName, nLine, size, align, IsPageable, tag, aligned_addr);
#else
	buf = MemoryAllocateAligned(size, align, IsPageable, tag, aligned_addr);
#endif
	if (buf)
		MemoryClear(*aligned_addr, size);
	return buf;
}

#if defined(MALLOC_TRACK_ON) && defined(VXWORKS)
void
MemoryAllocateVxWorksTrack(
    IN void *result,
    IN uint32 Bytes,
	IN char *reason,
	IN void *caller);

int
MemoryTrackerTrackDeallocate(
	IN void *pMemory);
#endif

#ifdef MEM_TRACK_ON
#define MemoryAllocate2Aligned( a, b, c, d, e )	\
	MemoryAllocate2AlignedDbg( __FILE__, __LINE__, a, b, c, d, e )
static _inline void* MemoryAllocate2AlignedDbg(
	IN const char *pFileName, 
	IN int32 nLine,
#else
static _inline void* MemoryAllocate2Aligned(
#endif
	IN uint32 size,
	IN uint32 align,
	IN uint32 flags,
	IN uint32 tag,
	OUT void **aligned_addr
	)
{
	void *buf;

#ifdef MEM_TRACK_ON
	buf = MemoryAllocate2Dbg(pFileName, nLine, size+align-1, flags, tag);
#else
	buf = MemoryAllocate2(size+align-1, flags, tag);
#endif
	*aligned_addr = (void*)ROUNDUPP2((uintn)buf,align);
	return buf;
}

#ifdef MEM_TRACK_ON
#define MemoryAllocate2AlignedAndClear( a, b, c, d, e )	\
	MemoryAllocate2AlignedAndClearDbg( __FILE__, __LINE__, a, b, c, d, e )
static _inline void* MemoryAllocate2AlignedAndClearDbg(
	IN const char *pFileName, 
	IN int32 nLine,
#else
static _inline void* MemoryAllocate2AlignedAndClear(
#endif
	IN uint32 size,
	IN uint32 align,
	IN uint32 flags,
	IN uint32 tag,
	OUT void **aligned_addr
	)
{
	void *buf;

#ifdef MEM_TRACK_ON
	buf = MemoryAllocate2AlignedDbg(pFileName, nLine, size, align, flags, tag, aligned_addr);
#else
	buf = MemoryAllocate2Aligned(size, align, flags, tag, aligned_addr);
#endif
	if (buf)
		MemoryClear(*aligned_addr, size);
	return buf;
}

/* internal private functions not for general use, these are supplied
 * by the OS specific imemory_osd module
 */
#if !defined(VXWORKS)
void*
MemoryAllocatePriv( IN uint32 Bytes, IN uint32 flags, IN uint32 Tag );


void
MemoryDeallocatePriv( IN void *pMemory );

#else // VXWORKS routines

void*
MemoryAllocatePriv( IN uint32 Bytes, void *caller );

void
MemoryDeallocatePriv( IN void *pMemory, void *caller );
#endif

#ifdef LINUX
/*
 * Memory Locking Strategies implemented.  These are obtained via
 * Kernel Memory module, passed via Vka to Uvca and into user Memory module
 * At this time only Linux has a few choices available. The strategy to use
 * is selected based on kernel version (hence the need for the Kernel Memory
 * module to report).  This is done such that a single object for the user
 * space libraries can work against multiple kernel versions.
 * 	MADVISE - user space does an madvise(SEQ) then calls VKA to MemoryLock
 * 				unadvise is not needed.
 *	MLOCK - user space does an mlock and madvise(DONTFORK) then calls VKA to
 *				MemoryLock.  After MemoryUnlock, user space does munlock and
 *				madvise(DOFORK).  User space must track and handle overlapping
 *				locks because mlock and madvise are not "stackable"
 */
#define MEMORY_LOCK_STRAT_MADVISE 0
#define MEMORY_LOCK_STRAT_MLOCK 1
#endif

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_IMEMORY_H_ */
