/* BEGIN_ICS_COPYRIGHT1 ****************************************

Copyright (c) 2015-2017, Intel Corporation

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

** END_ICS_COPYRIGHT1   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_IB_GSI_H_
#define _IBA_IB_GSI_H_


#if defined(VXWORKS)

/* Kernel mode GSI interfaces */

#include "iba/vpi.h"
#include "iba/stl_mad_priv.h"

#if defined (__cplusplus)
extern "C" {
#endif

typedef enum _GSI_REGISTRATION_FLAGS
{
	GSI_REGISTER_NONE				= 0x00000000,
	GSI_REGISTER_RESPONDER			= 0x00000001,
	GSI_REGISTER_TRAP_PROCESSOR		= 0x00000002,
	GSI_REGISTER_REPORT_PROCESSOR	= 0x00000004
}	GSI_REGISTRATION_FLAGS;

#define GSI_IS_RESPONDER(RegistrationFlags) \
		((RegistrationFlags & GSI_REGISTER_RESPONDER) ? 1 : 0)

#define GSI_IS_TRAP_PROCESSOR(RegistrationFlags) \
		((RegistrationFlags & GSI_REGISTER_TRAP_PROCESSOR) ? 1 : 0)

#define GSI_IS_REPORT_PROCESSOR(RegistrationFlags) \
		((RegistrationFlags & GSI_REGISTER_REPORT_PROCESSOR) ? 1 : 0)

/*
 * Structures
 */

typedef struct _IBT_BUFFER
{
	struct _IBT_BUFFER			*pNextBuffer;
	void						*pData;
	uint32						ByteCount;

}	IBT_BUFFER, *PIBT_BUFFER;


typedef struct _IBT_ELEMENT
{
	struct _IBT_ELEMENT			*pNextElement;
	IBT_BUFFER					*pBufferList;
	void						*pContext;

	/* Completion information. */
	uint32						RecvByteCount;
	FSTATUS						Status;

}	IBT_ELEMENT, IBT_MSG_ELEMENT,
	*PIBT_ELEMENT, *PIBT_MSG_ELEMENT;

typedef struct _IBT_DGRM_ELEMENT
{
	/* The element must be first. */
	IBT_ELEMENT					Element;
	
	/* Required for Sends/ returned on receives */
	uint32						RemoteQP;
	uint32						RemoteQKey;
	/* TBD - can use an IB_ADDRESS_VECTOR here instead */
	uint64						PortGuid;
	IB_LID						RemoteLID;
	uint8						ServiceLevel;
	uint8						PathBits;
	uint8						StaticRate; /* enum IB_STATIC_RATE */
	uint16						PkeyIndex;
	boolean						GlobalRoute;	

	/* Returned on receive completion. */
	uint32						TotalRecvSize;

	/* allocate/deallocate tripwire */

	uint8						Allocated:1;
	uint8						OnRecvQ:1;
	uint8						OnSendQ:1;
	uint8						reserved:5;

	/* Global routing information. */
	/* If GlobalRoute==true, GlobalRouteInfo structure is valid. */
	/* for received MADs, GRHInfo has already been translated to a sender */
	/* perspective (eg. SrcGIDIndex is local port, DestGID is remote port) */
	IB_GLOBAL_ROUTE_INFO		GRHInfo;

}	IBT_DGRM_ELEMENT;


/*
 * GsiDgrmAddrCopy()
 * copy address information from one dgrm to another
 */
static __inline
void
GsiDgrmAddrCopy( IBT_DGRM_ELEMENT *pDgrmTo, IBT_DGRM_ELEMENT *pDgrmFrom)
{
	pDgrmTo->RemoteQP		= pDgrmFrom->RemoteQP;
	pDgrmTo->RemoteQKey		= pDgrmFrom->RemoteQKey;
	pDgrmTo->PortGuid		= pDgrmFrom->PortGuid;
	pDgrmTo->RemoteLID		= pDgrmFrom->RemoteLID;
	pDgrmTo->ServiceLevel	= pDgrmFrom->ServiceLevel;
	pDgrmTo->PathBits		= pDgrmFrom->PathBits;
	pDgrmTo->StaticRate		= pDgrmFrom->StaticRate;
	pDgrmTo->PkeyIndex		= pDgrmFrom->PkeyIndex;
	pDgrmTo->GlobalRoute	= pDgrmFrom->GlobalRoute;
	memcpy(&pDgrmTo->GRHInfo, &pDgrmFrom->GRHInfo, sizeof(pDgrmTo->GRHInfo));
}

/* For a GSI send Dgrm, the 1st buffer is the MAD packet */
static __inline
MAD*
GsiDgrmGetSendMad(IBT_DGRM_ELEMENT *pDgrm)
{
	return (MAD*)(pDgrm->Element.pBufferList->pData);
}

/* For a GSI Recv Dgrm, the 1st buffer is the GRH header */
static __inline
IB_GRH*
GsiDgrmGetRecvGrh(IBT_DGRM_ELEMENT *pDgrm)
{
	return (IB_GRH*)(pDgrm->Element.pBufferList->pData);
}

/* For a GSI Recv Dgrm, the 2nd buffer is the MAD packet */
static __inline
MAD*
GsiDgrmGetRecvMad(IBT_DGRM_ELEMENT *pDgrm)
{
	return (MAD*)(pDgrm->Element.pBufferList->pNextBuffer->pData);
}

static __inline
uint32
GsiDgrmGetRecvMadByteCount(IBT_DGRM_ELEMENT *pDgrm)
{
	return pDgrm->Element.pBufferList->pNextBuffer->ByteCount;
}

/*
 * Callbacks
 */


/*
 * GSI_SEND_COMPLETION_CALLBACK
 *
 *
 * INPUTS:
 *	ServiceContext	- Caller defined context used during register class
 *	DgrmList		- Pointer to a single/pool of Datagram(s) completed in send
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	None
 */
typedef void 
(GSI_SEND_COMPLETION_CALLBACK)(
	IN	void					*ServiceContext,
	IN	IBT_DGRM_ELEMENT		*DgrmList
	);




/*
 * GSI_RECEIVE_CALLBACK
 *
 *
 * INPUTS:
 *	ServiceContext	- Caller defined context used during register class
 *	DgrmList		- Pointer to a single/pool of Datagram(s) received
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	FCOMPLETED
 *	FPENDING
 */
typedef FSTATUS 
(GSI_RECEIVE_CALLBACK)(
	IN	void					*ServiceContext,
	IN	IBT_DGRM_ELEMENT		*DgrmList
	);



/*
 * Registration and Class routines
 */


/*
 * GsiRegisterClass
 *
 *	GsiRegisterClass registers a management class with GSI.
 *	A class is identified by a unique id, a version. The class owner
 *  can also register a callback that can be used to indicate
 *  incoming packets targeted for this class and a callback to 
 *	indicate post send completions.
 *
 * INPUTS:
 *	MgmtClass		- Management class to be registered
 *	MgmtClassVersion- Will be 1 for this version.
 *	RegistrationFlags - Indicates if this is a Responder and/or a Trap Processor
 *						and/or a Report Processor.
 *						Responders, Trap/Report Processors register for and 
 *						receive unsolicited messages. GSI supports multiple 
 *						registrations per class.
 *	SARClient		- TRUE indicates client needs SAR capability.
 *	ServiceContext	- Caller defined context passed back in callbacks.
 *	SendCompletionCallback- Callback for all datagrams completions posted 
 *					  in send.
 *	ReceiveCallback	- Callback for all datagrams received.
 *
 * OUTPUTS:
 *	ServiceHandle	- Client unique handle returned to caller on successful
 *					  registration.
 *
 * RETURNS:
 *	FSUCCESS
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_RESOURCES
 */
typedef FSTATUS  
(GSI_REGISTER_CLASS)(
	IN	uint8					MgmtClass,
	IN	uint8					MgmtClassVersion,
	IN      GSI_REGISTRATION_FLAGS  		RegistrationFlags,
	IN	boolean					SARClient,
	IN	void					*ServiceContext,
	IN	GSI_SEND_COMPLETION_CALLBACK *SendCompletionCallback,
	IN	GSI_RECEIVE_CALLBACK	*ReceiveCallback,
	OUT IB_HANDLE				*ServiceHandle
	);
IBA_API GSI_REGISTER_CLASS iba_gsi_register_class;


/*
 * GsiDeregisterClass
 *
 * INPUTS:
 *	ServiceHandle	- Handle returned upon a successful registration.
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	FSUCCESS
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_RESOURCES
 */
typedef FSTATUS 
(GSI_DEREGISTER_CLASS)(
	IN	IB_HANDLE				ServiceHandle
	);
IBA_API GSI_DEREGISTER_CLASS iba_gsi_deregister_class;


/*
 * Pool management routines
 */

/*
 * GsiCreateDgrmPool
 *
 * INPUTS:
 *	ServiceHandle	- Handle returned upon registration
 *	ElementCount	- Number of elements in pool
 *	BuffersPerElement- Number of buffers in each pool element
 *	BufferSizeArray[]- An array of buffers sizes. The length of this array must
 *					  be equal to BuffersPerElement.
 *	ContextSize		- Size of user context in each element
 *
 * OUTPUTS:
 *	DgrmPoolHandle	- Opaque handle returned to identify datagram pool created
 *
 * RETURNS:
 *	FSUCCESS
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_RESOURCES
 */
typedef FSTATUS 
(GSI_CREATE_DGRM_POOL)(
	IN	IB_HANDLE				ServiceHandle,
	IN	uint32					ElementCount,
	IN	uint32                  BuffersPerElement,
	IN	uint32					BufferSizeArray[],
	IN	uint32                  ContextSize,
	OUT IB_HANDLE				*DgrmPoolHandle
	);
IBA_API GSI_CREATE_DGRM_POOL iba_gsi_create_dgrm_pool;


/*
 * GsiDestroyDgrmPool
 *
 * INPUTS:
 *	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	FSUCCESS
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_RESOURCES
 */
typedef FSTATUS 
(GSI_DESTROY_DGRM_POOL)(
	IN	IB_HANDLE				DgrmPoolHandle
	);
IBA_API GSI_DESTROY_DGRM_POOL iba_gsi_destroy_dgrm_pool;



/*
 * GsiDgrmPoolGet
 *
 * INPUTS:
 *	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
 *	ElementCount	- Number of elements to fetch from pool
 *
 * OUTPUTS:
 *	ElementCount	- Number of elements fetched from pool
 *	DgrmList		- Pointer holds the datagram list if successful
 *
 * RETURNS:
 *	FSUCCESS
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_RESOURCES
 */
typedef FSTATUS 
(GSI_DGRM_POOL_GET)(
	IN	IB_HANDLE				DgrmPoolHandle,
	IN OUT uint32				*ElementCount,
	OUT	IBT_DGRM_ELEMENT		**DgrmList
	);
IBA_API GSI_DGRM_POOL_GET iba_gsi_dgrm_pool_get;


/*
 * GsiDgrmPoolPut
 *
 *
 * INPUTS:
 *	DgrmList		- Datagram elements to return back to pool
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	FSUCCESS
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_RESOURCES
 */
typedef FSTATUS 
(GSI_DGRM_POOL_PUT)(
	IN	IBT_DGRM_ELEMENT		*DgrmList
	);
IBA_API GSI_DGRM_POOL_PUT iba_gsi_dgrm_pool_put;


/*
 * GsiDgrmPoolCount
 *
 * INPUTS:
 *	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
 *
 * OUTPUTS:
 *
 * RETURNS:
 *	number of datagrams in the pool
 */
typedef uint32 
(GSI_DGRM_POOL_COUNT)(
	IN	IB_HANDLE				DgrmPoolHandle
	);
IBA_API GSI_DGRM_POOL_COUNT iba_gsi_dgrm_pool_count;



/*
 * GsiDgrmPoolTotal
 *
 * INPUTS:
 *	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
 *
 * OUTPUTS:
 *
 * RETURNS:
 *	total number of datagrams allocated in the pool
 */
typedef uint32 
(GSI_DGRM_POOL_TOTAL)(
	IN	IB_HANDLE				DgrmPoolHandle
	);
IBA_API GSI_DGRM_POOL_TOTAL iba_gsi_dgrm_pool_total;


/*
 * CreateGrowableDgrmPool
 *
 *
 *
 * INPUTS:
 *
 *	ServiceHandle	- Handle returned upon registration
 *
 *	ElementCount	- Initial Number of elements in pool (can be 0)
 *
 *	BuffersPerElement- Number of buffers in each pool element (required)
 *
 *	BufferSizeArray[]- An array of buffers sizes. The length of this array must
 *					  be equal to BuffersPerElement. (required)
 *
 *	ContextSize		- Size of user context in each element (required)
 *	
 *	
 *
 * OUTPUTS:
 *
 *	DgrmPoolHandle	- Opaque handle returned to identify datagram pool created
 *
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_RESOURCES
 *
 */

typedef FSTATUS 
(GSI_CREATE_GROWABLE_DGRM_POOL)(
	IN	IB_HANDLE				ServiceHandle,
	IN	uint32					ElementCount,
	IN	uint32                  BuffersPerElement,
	IN	uint32					BufferSizeArray[],
	IN	uint32                  ContextSize,
	OUT IB_HANDLE				*DgrmPoolHandle
	);
IBA_API GSI_CREATE_GROWABLE_DGRM_POOL iba_gsi_create_growable_dgrm_pool;

/*
 * DgrmPoolGrow
 * Grow the size of a Growable Dgrm Pool
 * This routine may preempt.
 *
 *
 * INPUTS:
 *	DgrmPoolHandle	- Opaque handle to datagram pool
 *
 *	ELementCount	- number of elements to add to pool
 *
 *	
 *
 * OUTPUTS:
 *
 *	None
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_RESOURCES
 */
typedef FSTATUS 
(GSI_DGRM_POOL_GROW)(
	IN	IB_HANDLE				DgrmPoolHandle,
	IN	uint32					ElementCount
	);
IBA_API GSI_DGRM_POOL_GROW iba_gsi_dgrm_pool_grow;

/*
 * DgrmPoolGrowAsNeeded
 * Checks Pool size and schedules a system callback to grow it as needed
 * This routine does not preempt.
 *
 *
 * INPUTS:
 *	DgrmPoolHandle	- Opaque handle to datagram pool
 *
 *	lowWater		- if Dgrm Pool available < this, we will grow it
 *
 *	maxElements		- limit on size of Dgrm Pool, will not grow beyond this
 *
 *	growIncrement	- amount to grow by if we are growing
 *
 *	
 *
 * OUTPUTS:
 *
 *	None
 *
 *
 * RETURNS:
 *
 *	None
 */
typedef void 
(GSI_DGRM_POOL_GROW_AS_NEEDED)(
	IN	IB_HANDLE				DgrmPoolHandle,
	IN	uint32					lowWater,
	IN	uint32					maxElements,
	IN	uint32					growIncrement
	);
IBA_API GSI_DGRM_POOL_GROW_AS_NEEDED iba_gsi_dgrm_pool_grow_as_needed;


/*
 * Post operation routines
 */

/*
 * GsiPostSendDgrm
 *
 *
 * INPUTS:
 *	ServiceHandle	- Handle returned upon registration
 *	DgrmList		- List of datagram(s) to be posted for send
 *					each must have payload in network byte order
 *					however MAD and RMPP_HEADERs should be in host
 *					byte order.  On success, GSI owns the buffer and
 *					will byte swap back the MAD header prior to send completion
 *					on failure the buffer will be restored to host byte order
 *					for the MAD and RMPP headers
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	FSUCCESS
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_RESOURCES
 *	FINVALID_STATE
 */
typedef FSTATUS 
(GSI_POST_SEND_DGRM)(
	IN	IB_HANDLE				ServiceHandle,
	IN	IBT_DGRM_ELEMENT		*DgrmList
	);
IBA_API GSI_POST_SEND_DGRM iba_gsi_post_send;





/*
 * Helper functions
 */

/*
 * iba_gsi_dgrm_len
 *
 *  This reports the length of a Dgrm sequence as total of:
 *		GRH
 *		MAD_COMMON
 *		RMPP_HEADER
 *		class Header (SA, etc)
 *		RMPP_DATA (possibly large)
 *	returned messageSize will indicate the total size of the output.
 *	RmppHdr.PayloadLen will not be adjusted, it will still reflect the
 *	total RMPP_DATA which will account for a class Header in each
 *	packet.
 *
 * INPUTS:
 *	DgrmElement		- A list of Datagrams that need to be analyzed
 * 					    data pointed to only used during duration of call
 *	ClassHeaderSize	- size of class header
 *						this is expected to be in every RMPP packet
 *						immediately after the RMPP_HEADER
 *						size will account for one copy of the Class header
 *						from the 1st RMPP DATA of the message
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	Message Size.
 *	if invalid RMPP sequence, returns 0
 */
typedef uint32									
(GSI_DGRM_LEN)(
	IN IBT_DGRM_ELEMENT			*DgrmElement,
	IN uint32					ClassHeaderSize
	);
IBA_API GSI_DGRM_LEN iba_gsi_dgrm_len;


/*
 * GsiCoalesceDgrm
 *
 *	The Coalesce function will create an output as follows:
 *		GRH
 *		MAD_COMMON
 *		RMPP_HEADER
 *		class Header (SA, etc)
 *		RMPP_DATA (possibly large)
 *	returned messageSize will indicate the total size of the output
 *	RmppHdr.PayloadLen will not be adjusted, it will reflect the
 *	total RMPP_DATA which will account for a class Header in each
 *	packet.
 *
 * INPUTS:
 *	DgrmElement		- A list of Datagrams that need to be coalesced
 * 					    data pointed to only used during duration of call
 *	ppBuffer		- *ppBuffer is Client buffer to coalesce to
 *						if *ppBuffer is NULL, this routine will
 *						compute the necessary size of the buffer and
 *						allocate it on behalf of the caller
 * 					    data pointed to only used during duration of call
 *	ClassHeaderSize	- size of class header
 *						this is expected to be in every RMPP packet
 *						immediately after the RMPP_HEADER
 *						a copy of the Class header from the 1st RMPP
 *						DATA of the message will be included in the
 *						Coalesced output buffer
 *	MemAllocFlags		- flags for memory allocate of buffer
 *						for larger responses, it is recommended caller
 *						be structured such that IBA_MEM_FLAG_PREEMPTABLE
 *						can be used
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *
 *	Message size.
 *	if unable to allocate buffer *ppBuffer is left NULL
 *	if invalid RMPP sequence, returns 0.  In which case *ppBuffer will be NULL.
 */
typedef uint32									
(GSI_COALESCE_DGRM)(
	IN IBT_DGRM_ELEMENT			*DgrmElement,
	IN uint8					**ppBuffer,
	IN uint32					ClassHeaderSize
	);
IBA_API GSI_COALESCE_DGRM iba_gsi_coalesce_dgrm;

typedef uint32									
(GSI_COALESCE_DGRM2)(
	IN IBT_DGRM_ELEMENT			*DgrmElement,
	IN uint8					**ppBuffer,
	IN uint32					ClassHeaderSize,
	IN uint32					MemAllocFlags
	);
IBA_API GSI_COALESCE_DGRM2 iba_gsi_coalesce_dgrm2;


#if defined (__cplusplus)
};
#endif

#endif /* VXWORKS */
#endif	/* _IBA_IB_GSI_H_ */
