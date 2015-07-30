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


#ifndef _IBA_PUBLIC_IREQ_MGR_H_
#define _IBA_PUBLIC_IREQ_MGR_H_

#include "iba/public/datatypes.h"
#include "iba/public/igrowpool.h"
#include "iba/public/ilist.h"

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************
 * Generic request callback type, used to queue requests.
 *
 ****************************************************************************/
typedef void 
(*REQ_CALLBACK)( void );


/****************************************************************************
 * This structure is used to manage queuing requests for objects.
 *
 ****************************************************************************/
typedef struct _REQUEST_OBJECT
{
	uint32			Count;			/* Number of items requested. */
	boolean			PartialOk;		/* Is it okay to return some of the items. */
	REQ_CALLBACK	pfnCallback;	/* Notification routine when completed. */
	void			*Context1;		/* Callback context information. */
	void			*Context2;		/* Callback context information. */

} REQUEST_OBJECT;


/****************************************************************************
 * The REQ_MGR_GET_TYPE enumarated type is used to specify the type of 
 * request being performed when calling ReqMgrGet().
 *
 ****************************************************************************/
typedef enum _REQ_MGR_TYPE
{
	ReqGetSync,
	ReqGetAsync,
	ReqGetAsyncPartialOk

} REQ_MGR_TYPE;


/****************************************************************************
 * GET_AVAILABLE_COUNT
 *
 * Description:
 *	Function type declaration for the object count callback function invoked by
 *	the request manager to retrieve the number of objects available.
 *
 * Inputs:
 *	Context - User specified context.
 *
 * Outputs:
 *	None.
 *
 * Returns:
 *	Number of available objects.
 *
 ****************************************************************************/
typedef uint32 (*REQMGR_GET_COUNT_FUNC)(
	IN void *Context );


/****************************************************************************
 * REQ_MGR object.
 *
 ****************************************************************************/
typedef struct _REQ_MGR
{
	/* Pointer to the count callback function. */
	REQMGR_GET_COUNT_FUNC	m_pfnGetCount;		/* DO NOT USE!! */
	/* Context to pass as single parameter to count callback. */
	void					*m_GetContext;		/* DO NOT USE!! */

	/* Offset into each object where the chain pointers are located. */
	uint32					m_NextPtrOffset;	/* DO NOT USE!! */
	uint32					m_PrevPtrOffset;	/* DO NOT USE!! */

	/* Pending requests for elements. */
	QUEUE					m_RequestQueue;		/* DO NOT USE!! */

	/* Requests are dynamically allocated and added to the stack. */
	GROW_POOL				m_RequestPool;		/* DO NOT USE!! */

	boolean					m_Initialized;		/* DO NOT USE!! */

} REQ_MGR;


/****************************************************************************
 * ReqMgrInitState
 * 
 * Description:
 *	This function performs basic request manager initialization.  This 
 *	function cannot fail.
 * 
 * Inputs:
 *	pReqMgr - Pointer to a request manager structure.
 * 
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void
ReqMgrInitState(
	IN REQ_MGR* const pReqMgr );


/****************************************************************************
 * ReqMgrInit
 * 
 * Description:
 *	Initializes the request manager for use.
 * 
 * Inputs:
 *	pReqMgr			- Pointer to a request manager.
 *	pfnGetCount		- Pointer to a callback function invoked to 
 *					get the number of available objects.
 *	GetContext		- Context to pass into the pfnGetCount function.
 *
 * Outputs:
 *	None.
 * 
 * Returns:
 *	FSUCCESS				- the request manager was successfully initialized.
 *	FINSUFFICIENT_MEMORY	- there was not enough memory to initialize the
 *							request manager.
 * 
 ****************************************************************************/
FSTATUS
ReqMgrInit(
	IN REQ_MGR* const pReqMgr,
	IN REQMGR_GET_COUNT_FUNC pfnGetCount,
	IN void* const GetContext );


/****************************************************************************
 * ReqMgrDestroy
 * 
 * Description:
 *	Destroys a request manager.
 * 
 * Inputs:
 *	pReqMgr				- Pointer to a request manager.
 *
 * Outputs:
 *	None.
 * 
 * Returns:
 *	None.
 * 
 ****************************************************************************/
void
ReqMgrDestroy(
	IN REQ_MGR* const pReqMgr );


/****************************************************************************
 * ReqMgrGet
 * 
 * Description:
 *	Retrieves a number of objects, either synchronously or asynchronously.
 * 
 * Inputs:
 *	pReqMgr		- Pointer to a request manager.
 *	pCount		- Contains the number of objects to retrieve.
 *	ReqType		- Type of get operation.  Can be ReqGetSync, ReqGetAsync, or
 *				ReqGetAsyncPartialOk.
 *	pfnCallback	- Pointer to a callback function to store for the request.
 *	Context1	- First of two contexts. 
 *	Context2	- Second of two contexts.
 *
 * Outputs:
 *	pCount		- Contains the number of objects available.
 * 
 * Returns:
 *	FSUCCESS	- The request callback has been invoked with elements to 
 *				satisfy the request.  If the request allowed partial 
 *				completions, all elements are guaranteed to have been returned.
 *	FPENDING	- The request could not complete in it's entirety.  If the 
 *				request allowed partial completion, some elements may have been
 *				already returned.
 *	FINSUFFICIENT_RESOURCES	- There were not enough objects for the request to 
 *							succeed.
 *	FINSUFFICIENT_MEMORY	- There was not enough memory to process the 
 *							request (including queuing the request).
 * 
 ****************************************************************************/
FSTATUS
ReqMgrGet(
	IN REQ_MGR* const pReqMgr,
	IN OUT uint32* const pCount,
	IN const REQ_MGR_TYPE ReqType,
	IN REQ_CALLBACK pfnCallback,
	IN void* const Context1,
	IN void* const Context2 );


/****************************************************************************
 * ReqMgrResume
 * 
 * Description:
 *	Continues (and completes if possible) a pending request for objects, 
 *	if any.
 * 
 * Inputs:
 *	pReqMgr		- Pointer to a request manager.
 *
 * Outputs:
 *	pCount		- Number of objects available for a resuming request.
 *	pContext1	- Contains the Context1 value for the resuming request,
 *				as provided in the ReqMgrGet function.
 *	pContext2	- Contains the Context2 value for the resuming request,
 *				as provided in the ReqMgrGet function.
 * 
 * Returns:
 *	FSUCCESS				- A request was completed.
 *	FNOT_DONE				- There were no pending requests.
 *	FINSUFFICIENT_RESOURCES	- There were not enough resources to complete a
 *							pending request.
 *	FPENDING				- A request was continued, but not completed.
 *
 * Remarks:
 *	At most one requset is resumed.
 * 
 ****************************************************************************/
FSTATUS 
ReqMgrResume(
	IN REQ_MGR* const pReqMgr,
	OUT uint32* const pCount,
	OUT REQ_CALLBACK* const ppfnCallback,
	OUT void** const pContext1,
	OUT void** const pContext2 );


#ifdef __cplusplus
}
#endif

#endif	/* _IBA_PUBLIC_IREQ_MGR_H_ */
