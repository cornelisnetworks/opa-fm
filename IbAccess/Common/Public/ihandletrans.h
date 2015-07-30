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


#ifndef _IBA_PUBLIC_IHANDLETRANS_H_
#define _IBA_PUBLIC_IHANDLETRANS_H_

#include "iba/public/datatypes.h"
#include "iba/public/iarray.h"
#include "iba/public/iresmap.h"
#include "iba/public/ispinlock.h"


#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 *
 * Handle Translator definition.
 *
 * A Handle Translator facilitates translation from 32 bit handles to
 * pointers to the corresponding resource.  This can provide 2
 * benefits:
 * - a handle validation function
 * - support for APIs (such as VAPI) with 32 bit handles
 *
 * Internally a Handle Translator uses a vector and a resource map which
 * can be dynamically resized as more handles are created.
 *
 * The handle 0 is reserved such that 0 can be treated as a invalid handle
 * Similarly, an unitialized/freed/deleted handle returns NULL for its pointer
 *
 * The Locked Handle Translator has its own spinlock and is thread safe
 */


/* handle translator structure is allocated by the user and is opaque to them */
typedef struct _LOCKED_HANDLE_TRANS
{ 
	SPIN_LOCK 		m_Lock;	/* protects structure */
	ARRAY 			m_List;	/* index is handle, contained object is ptr */
	RESOURCE_MAP	m_Map;	/* efficiently track available entries in Array */
} LOCKED_HANDLE_TRANS;


/****************************************************************************
 * LHandleTransInitState
 * 
 * Description:
 *	This function performs basic initialization.  This function cannot 
 *	fail.
 * 
 * Inputs:
 *	pLHandleTrans - Pointer to handle translator
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void 
LHandleTransInitState(
	IN	LOCKED_HANDLE_TRANS* const	pLHandleTrans );


/****************************************************************************
 * LHandleTransInit
 * 
 * Description:
 *	This function initializes the handle translator.
 *	New entries in the translator are
 *	initialized to NULL.
 * 
 * Inputs:
 *	pLHandleTrans		- Pointer to handle translator
 *	InitialSize	- initial number of elements
 *	GrowSize	- number of elements to allocate when incrementally growing
 *				the handle translator
 *	MaxHandles	- maximum handles allowed, 0 if no limit
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY - failed to initialize translator
 * 
 ****************************************************************************/
FSTATUS
LHandleTransInit(
	IN	LOCKED_HANDLE_TRANS* const	pLHandleTrans,
	IN	const uint32	InitialSize,
	IN	const uint32	GrowSize,
	IN	const uint32	MaxHandles,
	IN	const uint32	MemFlags );


/****************************************************************************
 * LHandleTransDestroy
 * 
 * Description:
 *	This function deallocates all allocated memory associated with the 
 *	handle translator.
 *	If any operations did premptable memory allocate (via MemFlags), this
 *	may prempt.
 * 
 * Inputs:
 *	pLHandleTrans - Pointer to handle translator
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void
LHandleTransDestroy(
	IN	LOCKED_HANDLE_TRANS* const	pLHandleTrans );

/****************************************************************************
 * LHandleTransCreateHandle
 * 
 * Description:
 *	This function creates a new 32 bit handle to the given pointer
 *	As necessary the internal vector may grow.
 * 
 * Inputs:
 *	pLHandleTrans- Pointer to handle translator.
 *	Ptr			- Pointer of object handle is for
 *	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
 * 
 * Outputs:
 *	pHandle		- Handle created
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY	unable to grows translator
 *	FINSUFFICIENT_RESOURCES	would exceed MaxHandles specified in Init
 * 
 ****************************************************************************/
FSTATUS 
LHandleTransCreateHandle(
	IN	LOCKED_HANDLE_TRANS*	pLHandleTrans, 
	IN	const void*		Ptr,
	IN	const uint32	MemFlags,
	OUT	uint32* 		pHandle );

/****************************************************************************
 * LHandleTransReplaceHandle
 * 
 * Description:
 *	This function replaces the pointer which an existing handle points to
 * 
 * Inputs:
 *	pLHandleTrans- Pointer to handle translator.
 *	Handle		- previously created handle
 *	Ptr			- Pointer of object handle is for
 * 
 * Outputs:
 * 
 * Returns:
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
 *							specified element.
 * 
 ****************************************************************************/
FSTATUS 
LHandleTransReplaceHandle(
	IN	LOCKED_HANDLE_TRANS*	pLHandleTrans, 
	IN	uint32 			Handle,
	IN	const void*		Ptr);

/****************************************************************************
 * LHandleTransDestroyHandle
 * 
 * Description:
 *	This function destroys a handle, by replacing its pointer with NULL
 * 
 * Inputs:
 *	pLHandleTrans- Pointer to handle translator.
 *	Handle		- previously created handle
 * 
 * Outputs:
 * 
 * Returns:
 * 	FSUCCESS - successfully destroyed handle
 * 	FINVALID_STATE - handle was not valid
 * 
 ****************************************************************************/
FSTATUS
LHandleTransDestroyHandle(
	IN	LOCKED_HANDLE_TRANS*	pLHandleTrans, 
	IN	uint32 			Handle );


/****************************************************************************
 * LHandleTransGetPtr
 * 
 * Description:
 *	This function translates from a handle to its corresponding pointer
 * 
 * Inputs:
 *	pLHandleTrans- Pointer to handle translator.
 *	Handle		- previously created handle
 * 
 * Outputs:
 * 
 * Returns:
 *  pointer assigned to handle (will be NULL if invalid handle)
 * 
 ****************************************************************************/
void* 
LHandleTransGetPtr(
	IN	LOCKED_HANDLE_TRANS*	pLHandleTrans, 
	IN	uint32 			Handle);


#ifdef __cplusplus
}	/* extern "C" */
#endif


#endif	/* _IBA_PUBLIC_IHANDLETRANS_H_ */
