/* BEGIN_ICS_COPYRIGHT6 ****************************************

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

** END_ICS_COPYRIGHT6   ****************************************/


#include "ihandletrans.h"

///////////////////////////////////////////////////////////////////////////////
// LHandleTransInitState
// 
// Description:
//	This function performs basic initialization.  This function cannot 
//	fail.
// 
// Inputs:
//	pLHandleTrans - Pointer to handle translator
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void 
LHandleTransInitState(
	IN	LOCKED_HANDLE_TRANS* const	pLHandleTrans )
{
	SpinLockInitState(&pLHandleTrans->m_Lock);
	ArrayInitState(&pLHandleTrans->m_List);
	ResMapInitState(&pLHandleTrans->m_Map);
}


///////////////////////////////////////////////////////////////////////////////
// LHandleTransInit
// 
// Description:
//	This function initializes the handle translator.
//	New entries in the translator are
//	initialized to NULL.
// 
// Inputs:
//	pLHandleTrans		- Pointer to handle translator
//	InitialSize	- initial number of elements
//	GrowSize	- number of elements to allocate when incrementally growing
//				the handle translator
//	MaxHandles	- maximum handles allowed, 0 if no limit
//	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
// 
// Outputs:
//	None.
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY - failed to initialize translator
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS
LHandleTransInit(
	IN	LOCKED_HANDLE_TRANS* const	pLHandleTrans,
	IN	const uint32	InitialSize,
	IN	const uint32	GrowSize,
	IN	const uint32	MaxHandles,
	IN	const uint32	MemFlags )
{
	if (! SpinLockInit(&pLHandleTrans->m_Lock))
		goto faillock;
	if (FSUCCESS != ArrayInit(&pLHandleTrans->m_List, InitialSize, GrowSize, sizeof(void*), MemFlags))
		goto faillist;
	if (FSUCCESS != ResMapInit(&pLHandleTrans->m_Map, InitialSize, GrowSize, MaxHandles, (MemFlags&IBA_MEM_FLAG_PAGEABLE)?TRUE:FALSE))
		goto failmap;
	// reserve entry 0 so no handles end up being 0
	if (FSUCCESS != ResMapReserve(&pLHandleTrans->m_Map, 0))
		goto failres;
	return FSUCCESS;

failres:
	ResMapDestroy(&pLHandleTrans->m_Map);
failmap:
	ArrayDestroy(&pLHandleTrans->m_List);
faillist:
	SpinLockDestroy(&pLHandleTrans->m_Lock);
faillock:
	return FINSUFFICIENT_MEMORY;
}


///////////////////////////////////////////////////////////////////////////////
// LHandleTransDestroy
// 
// Description:
//	This function deallocates all allocated memory associated with the 
//	handle translator.
//	If any operations did premptable memory allocate (via MemFlags), this
//	may prempt.
// 
// Inputs:
//	pLHandleTrans - Pointer to handle translator
// 
// Outputs:
//	None.
// 
// Returns:
//	None.
// 
///////////////////////////////////////////////////////////////////////////////
void
LHandleTransDestroy(
	IN	LOCKED_HANDLE_TRANS* const	pLHandleTrans )
{
	ResMapDestroy(&pLHandleTrans->m_Map);
	ArrayDestroy(&pLHandleTrans->m_List);
	SpinLockDestroy(&pLHandleTrans->m_Lock);
}

///////////////////////////////////////////////////////////////////////////////
// LHandleTransCreateHandle
// 
// Description:
//	This function creates a new 32 bit handle to the given pointer
//	As necessary the internal vector may grow.
// 
// Inputs:
//	pLHandleTrans- Pointer to handle translator.
//	Ptr			- Pointer of object handle is for
//	MemFlags - memory allocate flags (IBA_MEM_FLAG_*)
// 
// Outputs:
//	pHandle		- Handle created
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
//							specified element.
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
LHandleTransCreateHandle(
	IN	LOCKED_HANDLE_TRANS*	pLHandleTrans, 
	IN	const void*		Ptr,
	IN	const uint32	MemFlags,
	OUT	uint32* 		pHandle )
{
	FSTATUS status;
	void *pNewElements = NULL;

	SpinLockAcquire(&pLHandleTrans->m_Lock);
	status = ResMapAllocate(&pLHandleTrans->m_Map, pHandle);
	if (status != FSUCCESS)
		goto done;
	if (MemFlags & IBA_MEM_FLAG_PREMPTABLE)
	{
		SpinLockRelease(&pLHandleTrans->m_Lock);
		status = ArrayPrepareSet(&pLHandleTrans->m_List, *pHandle, MemFlags, &pNewElements);
		SpinLockAcquire(&pLHandleTrans->m_Lock);
		if (status != FSUCCESS)
			goto failarray;
	}
	status = ArraySetPointer(&pLHandleTrans->m_List, *pHandle, (void*)Ptr, MemFlags, &pNewElements);
	if (status != FSUCCESS)
		goto failarray;
done:
	SpinLockRelease(&pLHandleTrans->m_Lock);
	if (pNewElements)
		MemoryDeallocate(pNewElements);
	return status;
failarray:
	(void)ResMapFree(&pLHandleTrans->m_Map, *pHandle);
	goto done;
}

///////////////////////////////////////////////////////////////////////////////
// LHandleTransReplaceHandle
// 
// Description:
//	This function replaces the pointer which an existing handle points to
// 
// Inputs:
//	pLHandleTrans- Pointer to handle translator.
//	Handle		- previously created handle
//	Ptr			- Pointer of object handle is for
// 
// Outputs:
// 
// Returns:
//	FSUCCESS
//	FINSUFFICIENT_MEMORY	The array could not be resized to accomodate the
//							specified element.
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
LHandleTransReplaceHandle(
	IN	LOCKED_HANDLE_TRANS*	pLHandleTrans, 
	IN	uint32 			Handle,
	IN	const void*		Ptr)
{
	FSTATUS status;

	SpinLockAcquire(&pLHandleTrans->m_Lock);
	status = ArraySetPointer(&pLHandleTrans->m_List, Handle, (void*)Ptr, IBA_MEM_FLAG_NONE, NULL);
	SpinLockRelease(&pLHandleTrans->m_Lock);
	return status;
}

///////////////////////////////////////////////////////////////////////////////
// LHandleTransDestroyHandle
// 
// Description:
//	This function destroys a handle, by replacing its pointer with NULL
// 
// Inputs:
//	pLHandleTrans- Pointer to handle translator.
//	Handle		- previously created handle
// 
// Outputs:
// 
// Returns:
// 	FSUCCESS - successfully destroyed handle
// 	FINVALID_STATE - handle was not valid
// 
///////////////////////////////////////////////////////////////////////////////
FSTATUS 
LHandleTransDestroyHandle(
	IN	LOCKED_HANDLE_TRANS*	pLHandleTrans, 
	IN	uint32 			Handle )
{
	FSTATUS status;

	SpinLockAcquire(&pLHandleTrans->m_Lock);
	status = ResMapFree(&pLHandleTrans->m_Map, Handle);
	if (FSUCCESS == status)
	{
		(void)ArraySetPointer(&pLHandleTrans->m_List, Handle, NULL, IBA_MEM_FLAG_NONE, NULL);
	}
	SpinLockRelease(&pLHandleTrans->m_Lock);
	return status;
}


///////////////////////////////////////////////////////////////////////////////
// LHandleTransGetPtr
// 
// Description:
//	This function translates from a handle to its corresponding pointer
// 
// Inputs:
//	pLHandleTrans- Pointer to handle translator.
//	Handle		- previously created handle
// 
// Outputs:
// 
// Returns:
//  pointer assigned to handle (will be NULL if invalid handle)
// 
///////////////////////////////////////////////////////////////////////////////
void* 
LHandleTransGetPtr(
	IN	LOCKED_HANDLE_TRANS*	pLHandleTrans, 
	IN	uint32 			Handle)
{
	void* Ptr;

	SpinLockAcquire(&pLHandleTrans->m_Lock);
	if (FSUCCESS != ArrayAtPointer(&pLHandleTrans->m_List, Handle, (void**)&Ptr))
		Ptr = NULL;
	SpinLockRelease(&pLHandleTrans->m_Lock);
	return Ptr;
}
