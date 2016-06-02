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

 * ** END_ICS_COPYRIGHT5   ****************************************/

//=======================================================================
//									/
// FILE NAME								/
//    pool.c								/
//									/
// DESCRIPTION								/
//    This is the pool CS routines.					/
//									/
// DATA STRUCTURES							/
//    None								/
//									/
// FUNCTIONS								/
//    vs_pool_create		initialize the pool subsystem		/
//    vs_pool_delete		delete a pool subsystem			/
//    vs_pool_alloc		allocate a buffer			/
//    vs_pool_free		free a buffer				/
//									/
// DEPENDENCIES								/
//    ib_status.h							/
//    cs_g.h								/
//									/
//									/
// HISTORY								/
//									/
//    NAME	DATE  REMARKS						/
//     jsy  02/06/01  Initial creation of file.				/
//     jsy  03/14/01  Added additional checks. 				/
//     dkj  02/07/02  PR 468. Added vs_pool_page_size()                 /
//     dkj  04/01/02  PR 1676. Updated vs_pool_page_size to OS API 2.0g /
//=======================================================================

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <signal.h>

// PAGE_SIZE and asm/page.h no longer supported on Linux
#ifndef PAGE_SIZE
#include <unistd.h>
#define PAGE_SIZE getpagesize()
#endif

#include "ib_status.h"
#include "cs_g.h"
#include "cs_log.h"

#ifdef LOCAL_MOD_ID
#undef LOCAL_MOD_ID
#endif
#define LOCAL_MOD_ID VIEO_CS_MOD_ID

#ifdef __GNUC__
#define function __FUNCTION__
#else
#define function "vs_implpool function"
#endif

typedef struct
{
	uint64_t  numBytesAlloc;	// amount allocated from pool
} Implpriv_Pool_t;

/*
 * These defines control checks made during allocations and frees
 */

/* Add an extra eight bytes (sizeof uint64_t) to allocation size -
 * This was the way the original allocation code worked.
 */
#define PAD_ALLOCATIONS 1

/* Aborts if an overwrite of the buffer is detected when it is freed */
#define ABORT_ON_OVERWRITE 0

/* Aborts if a buffer is freed for a second time */
#define ABORT_ON_DOUBLE_FREE 1

/* Zeros out memory on both allocation and free */
#define ZERO_FREED_MEM 1

#if (ABORT_ON_OVERWRITE)
static const uint32_t sentinel0 = 0xB00A110C;
static const uint32_t sentinel1 = 0xE00A110C;
#endif


Status_t
vs_implpool_create(Pool_t *poolp, uint32_t options, uint8_t *name, void *address, uint32_t size) {

	IB_ENTER (function, poolp, options, address, size);

    /* Check to be sure that we have a valid Pool_t */

	if (poolp == NULL) {
		IB_EXIT (function, VSTATUS_ILLPARM);
		return(VSTATUS_ILLPARM);
	}

	/*
	   ** this is a run-time check to ensure that the opaque section of Pool_t
	   ** is large enough to contain the Implementation private data structure.
	   ** If this test fails, adjust the OPAQUE_POOL_ELEMENTS define in
	   ** vs_g.h and rebuild.
	 */
	if(sizeof (Implpriv_Pool_t) > sizeof (poolp->opaque))
	{
		IB_LOG_ERROR ("Implpriv_Pool_t too big:", sizeof (Implpriv_Pool_t));
		IB_EXIT (function, VSTATUS_ILLPARM);
		return VSTATUS_ILLPARM;
	}

    /* Initialize the pool */
	// caller has already zeroed *poolp
	strncpy((char*)poolp->name, (char *)name, VS_NAME_MAX);
	poolp->options = options;
	poolp->buffers = NULL;

	IB_EXIT (function, VSTATUS_OK);
	return(VSTATUS_OK);
}


Status_t
vs_implpool_delete(Pool_t *poolp) {
	PBuffer_t	*bufferp;

	IB_ENTER (function, poolp, 0, 0, 0);

    /* Check to be sure that we have a valid Pool_t */
	if (poolp == NULL) {
		IB_EXIT (function, VSTATUS_ILLPARM);
		return(VSTATUS_ILLPARM);
	}

    /* Delete all of the buffers remaining in the pool */
	while ((bufferp = poolp->buffers) != NULL) {
		poolp->buffers = bufferp->next;

#if (ZERO_FREED_MEM)
		memset(bufferp, 0, bufferp->size);
#endif

		free((void *)bufferp);
	}

	IB_EXIT (function, VSTATUS_OK);
	return(VSTATUS_OK);
}


Status_t
vs_implpool_alloc(Pool_t *poolp, uint32_t reqSize, void **address) {
	int32_t		bytes;
	PBuffer_t	*bufferp;

	IB_ENTER (function, poolp, reqSize, address, 0);

    /* Check to be sure that we have a valid Pool_t */
	if (poolp == NULL) {
		IB_EXIT (function, VSTATUS_ILLPARM);
		return(VSTATUS_ILLPARM);
	}
// TBD - don't need Buffer_t header for normal case below, could save space

    /* See if we can get a buffer */
#if (PAD_ALLOCATIONS)
	/* This is the original calculation for number of bytes to allocate...
	 * Note the extra padding. This is sloppy. Also note that if both
	 * PAD_ALLOCATIONS and ABORT_ON_OVERWRITE are defined, that the sentinels
	 * are added to the the buffer _before_ the padding.
	 */
	bytes = sizeof(PBuffer_t) + reqSize + sizeof(uint64_t);
#else
	bytes = sizeof(PBuffer_t) + reqSize;
#endif

#if (ABORT_ON_OVERWRITE)
	bytes += (3 * sizeof(uint32_t));
#endif

	bufferp = (PBuffer_t *) malloc(bytes);
	if (bufferp == NULL) {
		IB_EXIT (function, VSTATUS_NOMEM);
		return(VSTATUS_NOMEM);
	}

// TBD vxworks doesn't do memset
	memset((void *)bufferp, 0, bytes);
	bufferp->next = poolp->buffers;
	bufferp->prev = NULL;
	bufferp->sentinel = DELETE_MARKER;
	bufferp->addr0 = (uint8_t *)bufferp;
	bufferp->addr1 = (uint8_t *)(bufferp+1);	// JSY - fix
	bufferp->size = bytes;

#if (ABORT_ON_OVERWRITE)
	/* We copy a constant value to the beginning of the buffer
	 * and a constant value to the end of the buffer, followed by the size
	 * of the buffer again. This allows us to make sure that we haven't
	 * overwritten the beginning or end of the allocated region when
	 * its freed.
	 *
	 * We memcpy here rather than cast and assign b/c these values may
	 * not necessarily be aligned, and that may cause problems on some
	 * architectures.
	 */
	memcpy(bufferp->addr1, &sentinel0, sizeof(uint32_t));

	/* adjust addr1 to account for the sentinel */
	bufferp->addr1 += sizeof(uint32_t);

	memcpy(bufferp->addr1 + reqSize, &sentinel1, sizeof(uint32_t));
	memcpy(bufferp->addr1 + reqSize + sizeof(uint32_t),
	       &bytes, sizeof(uint32_t));
#endif
	
	if (poolp->buffers != NULL)
		poolp->buffers->prev = bufferp;
	poolp->buffers = bufferp;
	*address = bufferp->addr1;

	((Implpriv_Pool_t *)poolp->opaque)->numBytesAlloc += bufferp->size;

	IB_EXIT (function, VSTATUS_OK);
	return(VSTATUS_OK);
}

static void freeBuffer(PBuffer_t * bufferp)
{
#if (ABORT_ON_OVERWRITE)
	uint32_t value = 0;
#endif

	IB_ENTER(function, bufferp, 0, 0, 0);

#if (ABORT_ON_OVERWRITE)
	/* Check whether we overwrote the beginning or the end of the
	 * allocated region
	 */
	memcpy(&value, bufferp->addr1 - sizeof(uint32_t), sizeof(uint32_t));
	assert(value == sentinel0);

#if (PAD_ALLOCATIONS)
	bufferp->size -= sizeof(uint64_t);
#endif /* PAD_ALLOCATIONS */

	memcpy(&value, ((uint8_t *) bufferp) + bufferp->size
	                  - (2 * sizeof(uint32_t)), sizeof(uint32_t));
	assert(value == sentinel1);

	memcpy(&value, ((uint8_t *) bufferp) + bufferp->size - sizeof(uint32_t),
	       sizeof(uint32_t));

#if (PAD_ALLOCATIONS)
	bufferp->size += sizeof(uint64_t);
#endif /* PAD_ALLOCATIONS */

	assert(value == bufferp->size);
#endif /* ABORT_ON_OVERWRITE */

#if (ZERO_FREED_MEM)
	memset(bufferp, 0, bufferp->size);
#endif
	free(bufferp);

	IB_EXIT(function, VSTATUS_OK);
}

Status_t
vs_implpool_free(Pool_t *poolp, void *address) {
	PBuffer_t	*bufferp;

	IB_ENTER (function, poolp, address, 0,0);

    /* Check to be sure that we have a valid Pool_t */
	if (poolp == NULL) {
		IB_EXIT (function, VSTATUS_ILLPARM);
		return(VSTATUS_ILLPARM);
	}

    /* Check to be sure that the address is a real one */
	if (address == NULL) {
		IB_EXIT (function, VSTATUS_ILLPARM);
		return(VSTATUS_ILLPARM);
	}

# if (ABORT_ON_OVERWRITE)
	address -= sizeof(uint32_t);
# endif

	// Pointer to the PBuffer_t should be at (address - sizeof(PBuffer_T))
	bufferp = (((PBuffer_t*) address) - 1);

	if (bufferp->sentinel != DELETE_MARKER) {
#if (ABORT_ON_DOUBLE_FREE)
    	/* Couldn't find the required buffer */
		IB_FATAL_ERROR("vs_implpool_free: could not find buffer to free");
#endif
		IB_EXIT (function, VSTATUS_ILLPARM);
		return(VSTATUS_ILLPARM);
	}
	
	bufferp->sentinel = 0;

	if (bufferp->prev) {
		bufferp->prev->next = bufferp->next;
	} else {
		poolp->buffers = bufferp->next; // at head of list
	} 
	if (bufferp->next) {
		bufferp->next->prev = bufferp->prev;
	}

	((Implpriv_Pool_t *)poolp->opaque)->numBytesAlloc -= bufferp->size;
	freeBuffer(bufferp);
	IB_EXIT (function, VSTATUS_OK);
	return VSTATUS_OK;
}

/**********************************************************************
*
* FUNCTION
*    vs_implpool_size
*
* OUTPUTS
*      number of bytes allocated from pool.
**********************************************************************/
Status_t
vs_implpool_size(Pool_t *poolp, uint64_t *numBytesAlloc)
{

	// caller already validated poolp and size
	*numBytesAlloc = ((Implpriv_Pool_t *)poolp->opaque)->numBytesAlloc;
	return VSTATUS_OK;
#if 0
	PBuffer_t	*bufferp;
	uint64_t	psize=0;

	if (poolp == NULL) {
		return psize;
	}

	for (bufferp = poolp->buffers; bufferp != NULL; bufferp = bufferp->next) {
		psize += bufferp->size;
	}
	return psize;
#endif
}

size_t vs_pool_page_size (void) {
  IB_ENTER (function, (uint32_t) 0U, (uint32_t) 0U, (uint32_t) 0U, (uint32_t) 0U);
  IB_EXIT (function, (size_t) PAGE_SIZE);
  return (size_t) PAGE_SIZE;
}
