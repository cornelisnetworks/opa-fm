/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_UMADT_H_
#define _IBA_UMADT_H_

#include <iba/public/datatypes.h>
#include <iba/stl_types.h>
#include <iba/stl_mad_priv.h>
#include <iba/stl_sa_priv.h>
#include <iba/vpi.h>

#if defined (__cplusplus)
extern "C" {
#endif

#define SEND_COMPLETION 1
#define RECV_COMPLETION 2
#define EVENT_COMPLETION 4

/* Typedefs */
typedef void* MADT_HANDLE;
/*DataStructures */

/* ClassId is used to select between SMI and GSI as well as selecting
 * which unsolicited inbound packets the consumer desires.
 * For SMI, ClassId can be MCLASS_SM_LID_ROUTED or MCLASS_SM_DIRECTED_ROUTE
 * either can receive SMI packets regardless of actual ClassId of packet
 * This helps to reduce complexity in SM's using this interface
 * Any other ClassId will register for the GSI interface
 *
 * The ClassId only affects inbound packets.  The registration can be used
 * to send any request packet in any class and corresponding responses
 * (with same transaction id) will be returned to the sender.
 * If registered for an SMI ClassId, all sends will be via QP0, if
 * registered for a GSI ClassId all sends will be via the GSI QP.
 *
 * Unsolicited packets will be delivered based on their Method and R bit fields
 * and the "is" flags below will select which packets a given consumer desires.
 * A given unsolicited packet can be delivered to more than 1 consumer.
 *
 * A given registration is for a single local port, all sends and receives
 * will be directed to that port.
 */
typedef struct RegisterClassStruct_ {
	EUI64		PortGuid;				/* PortGuid of local port */
		/* controls for unsolicited inbound packets */
	uint8		ClassId;				/* Mgmt Class ID */
	uint8		ClassVersion;			/* Mgmt Class version */
	boolean		isResponder;			/* True if this is a GSI Agent */
	boolean		isTrapProcessor;		/* True if GSI Trap msgs are handled */
	boolean		isReportProcessor;		/* True if GSI Report msgs are handled */
		/* queue sizing and management */
	uint32		SendQueueSize;			/* SendQueueSize */
	uint32		RecvQueueSize;			/* Receive Queue Size */
	boolean		NotifySendCompletion;	/* Notification for send completion */
	} RegisterClassStruct, *PRegisterClassStruct;

typedef struct MadtStruct_ {
	struct		MadtStruct_ *FLink;		/* Forward link	 */
	struct		MadtStruct_ *BLink;		/* Backward link */
	
	/* Consumer context pointer (undefined on Receives) */
	uint64		Context;				
	
	IB_GRH		Grh;					/* GRH */
	MAD			IBMad;					/* Management Data Gram */
	uint32		MadByteCount;			/* Used size of IBMad in bytes */
										/* When zero indicates MAD_BLOCK_SIZE */
	} MadtStruct;

typedef struct MadAddrStruct_ {
		STL_LID_32	DestLid;				/* DLID */
		IB_PATHBITS	PathBits;				/* PathBits */
		uint8		StaticRate;				/* The maximum static rate supported */
											/* enum IB_STATIC_RATE */
		union AddrType_ {
			struct Smi_ {
				STL_LID_32	SourceLid;		/* SLID */
				uint8		PortNumber;		/* Incomming PortNumber		 */
											/* Returned on RecvCompletion */
											/* not required for Sends */
				}Smi;
			struct Gsi_ {
				uint32		RemoteQpNumber;	/* RemoteQpNumber */
				uint32		RemoteQkey;		/* RemoteQkey  */
				IB_P_KEY	PKey;			/* Pkey for Send WQE */
											/* and Recv CQE */
				IB_SL		ServiceLevel;
				
				/* Global routing information. */
				/* If GlobalRoute==true, GlobalRouteInfo structure is valid. */
				boolean						GlobalRoute;	
				IB_GLOBAL_ROUTE_INFO		GRHInfo;
			}Gsi;
		}AddrType;
}MadAddrStruct;

typedef struct MadWorkCompletion_ {

	MadAddrStruct AddressInfo;
	/* Completion Information */
	uint32		RecvByteCount;			/* Received Byte count */
	FSTATUS		Status;					/*Status of this packet */
	
	} MadWorkCompletion;

typedef struct MadtEventRecordStruct_ {
	uint32		ResourceType;
	uint32		EventOrError;
	}MadtEventRecordStruct;

/* Function prototypes */

/*
* Register
*
* DESCRIPTION :
*
* This routine registers an agent or a manager with the associated QP. The
* returned MADT_HANDLE is later used to send and receive MAD's. 
* This routine may be expected to perform a kernel mode context switch
* 
*
* PARAMETERS :
* INPUTS :
*	registerStruct				: Data related to agent or manager being registered
*								  data pointed to only used during duration of call
*
* OUTPUTS:
*	serviceHandle				: The handle to the newly registered Agent
*								  or Manager	
*
* RETURNS: 
*	FSUCCESS				: Operation was successful
*	FINVALID_PARAMETER		: Invalid argumetns
*	FINSUFFICIENT_RESOURCES	: Insufficient resource
*
* Environment : User Mode
*
* NOTES:
*/

typedef
FSTATUS (UMADT_REGISTER)(
		IN struct RegisterClassStruct_	*registerStruct,
		OUT MADT_HANDLE	*serviceHandle
		);
IBA_API UMADT_REGISTER iba_umadt_register;

/*
* GetSendMad
*
* DESCRIPTION :
*
* This routine gets a MAD from the global MAD Pool which is a shared
* memory between provider and the proxy driver. The returned MAD is 
* filled with data and is used in PostSend call.
* This routine will not perform a kernel mode context switch.
* 
*
* PARAMETERS :
* INPUTS :
*	serviceHandle				: The handle got in Register Call
*	madCount					: Count of number of mads to return
*								  data pointed to only used during duration of call
*
* OUTPUTS:
*	madCount					: Count of number of mads returned
*	mad							: Returns pointer to MadtStruct
*	
* RETURNS: 
*	FSUCCESS				: Operation was successful
*	FINVALID_PARAMETER		: Invalid argumetns
*	FUNAVAILABLE			: SendMad unavailable
*
* Environment : User Mode
*
* NOTES:
*/

typedef
FSTATUS (UMADT_GETSENDMAD)(
		IN MADT_HANDLE serviceHandle,
		IN OUT uint32	*madCount,
		OUT MadtStruct ** mad
		);
IBA_API UMADT_GETSENDMAD iba_umadt_get_sendmad;

/*
* ReleaseSendMad
*
* DESCRIPTION :
*
* This routine puts back mads to the global mad pool.
* This routine will not perform a kernel mode context switch.
* 
*
* PARAMETERS :
* INPUTS :
*	serviceHandle			: The handle got in Register Call
*	mad						: Pointer to MadtStruct
*							  data pointed to only used during duration of call
*
* OUTPUTS:
*	None
*
* RETURNS: 
*	FSUCCESS				: Operation was successful
*	FINVALID_PARAMETER		: Invalid argumetns
* Environment : User Mode
*
* NOTES:
*/


typedef
FSTATUS (UMADT_RELEASESENDMAD)(
		IN MADT_HANDLE serviceHandle,
		IN MadtStruct *mad
		);
IBA_API UMADT_RELEASESENDMAD iba_umadt_release_sendmad;

/*
* ReleaseRecvMad
*
* DESCRIPTION :
*
* This routine puts back mads to the global receive mad pool.
* This routine will not perform a kernel mode context switch.
* 
*
* PARAMETERS :
* INPUTS :
*	serviceHandle			: The handle got in Register Call 
*	mad						: Pointer to MadtStruct
*							  data pointed to only used during duration of call
*
* OUTPUTS:
*	None
*
* RETURNS: 
*	FSUCCESS				: Operation was successful
*	FINVALID_PARAMETER		: Invalid argumetns
* Environment : User Mode
*
* NOTES:
*/

typedef
FSTATUS (UMADT_RELEASERECVMAD)(
		IN MADT_HANDLE serviceHandle,
		IN MadtStruct *mad
		);
IBA_API UMADT_RELEASERECVMAD iba_umadt_release_recvmad;

/*
* PostSend
*
* DESCRIPTION :
*
* This routine sends a management datatgram out on the wire.
* This routine may perform a kernel mode context switch.
*
* PARAMETERS :
*
* INPUTS
*	serviceHandle			: The handle got in Register Call
*	mad						: MAD info
*							  data pointed to only used during duration of call
*	destAddr				: MadAddrStruct info
*							  data pointed to only used during duration of call
*
* OUTPUTS:
*	None
*
* RETURNS: 
*	FSUCCESS				: Operation was successful
*	FINVALID_PARAMETER		: Invalid argumetns
*	FERROR					: Error while posting
*	FINSUFFICIENT_RESOURCES	: Insufficient resource
* 
* Environment : User Mode
*
* NOTES:
*/

typedef
FSTATUS (UMADT_POSTSEND)(
		IN MADT_HANDLE serviceHandle,
		IN MadtStruct *mad,
		IN MadAddrStruct *destAddr
		);
IBA_API UMADT_POSTSEND iba_umadt_post_send;

/*
* PostRecv
*
* DESCRIPTION :
*
* This routine informs the kernen mode component to post additional receive 
* buffers on the Queue pair described by the service handle
*
* This routine may perform a kernel mode context switch.
*
* PARAMETERS :
*
* INPUTS
*	serviceHandle			: The handle got in Register Call
*
* OUTPUTS:
*	None
*
* RETURNS: 
*	FSUCCESS				: Operation was successful
*	FINVALID_PARAMETER		: Invalid argumetns
*	FERROR					: Error while posting
*	FINSUFFICIENT_RESOURCES	: Insufficient resource
* 
* Environment : User Mode
*
* NOTES:
*/

typedef
FSTATUS (UMADT_POSTRECV)(
		IN MADT_HANDLE serviceHandle
		);
IBA_API UMADT_POSTRECV iba_umadt_post_recv;

/*
* PollForSendCompletion
*
* DESCRIPTION :
*
* This routine polls for any send completion mads.
* This routine will not perform a kernel mode context switch.
* 
*
* PARAMETERS :
* INPUTS :
*	serviceHandle			: The handle got in Register Call 
*	
* OUTPUTS:
*	mad						: MAD info
*	destAddr				: MadAddrStruct info
*
* RETURNS: 
*	FSUCCESS				: Operation was successful
*	FINVALID_PARAMETER		: Invalid arguments
*   FNOT_FOUND				: No more to packets
* 
* Environment : User Mode
*
* NOTES:
*/

typedef
FSTATUS (UMADT_POLLFORSENDCOMPLETION)(
		IN MADT_HANDLE serviceHandle,
		OUT MadtStruct **mad,
		OUT MadWorkCompletion **ppWorkCompInfo
		);
IBA_API UMADT_POLLFORSENDCOMPLETION iba_umadt_poll_send_compl;

/*
* PollForRecvCompletion
*
* DESCRIPTION :
*
* This routine polls and gets the receive completion mads.
* This routine will not perform a kernel mode context switch.
* 
*
* PARAMETERS :
*
* INPUTS :
*	serviceHandle			: The handle got in Register Call 
*
* OUTPUTS:
*	mad						: MAD info
*	destAddr				: MadAddrStruct info
*
* RETURNS: 
*	FSUCCESS				: Operation was successful
*	FINVALID_PARAMETER		: Invalid argumetns
*   FNOT_FOUND				: No more to packets
* 
* Environment : User Mode
*
* NOTES:
*/

typedef
FSTATUS (UMADT_POLLFORRECVCOMPLETION)(
		IN MADT_HANDLE serviceHandle,
		OUT MadtStruct **mad,
		OUT MadWorkCompletion **ppWorkCompInfo
		);
IBA_API UMADT_POLLFORRECVCOMPLETION iba_umadt_poll_recv_compl;


/*
* PollForEvent
*
* DESCRIPTION :
*
* This routine gets the eventrecord
* This routine will not perform a kernel mode context switch.
* 
*
* PARAMETERS :
*
* INPUTS :
*	serviceHandle			: The handle got in Register Call 
*
* OUTPUTS:
*	eventRecord				: MadtEventRecordStruct
*
* RETURNS: 
*	FSUCCESS				: Operation was successful
*	FINVALID_PARAMETER		: Invalid argumetns
*   FNOT_FOUND				: No more to packets
* 
* Environment : User Mode
*
* NOTES:
*/

typedef
FSTATUS (UMADT_POLLFOREVENT)(
		IN MADT_HANDLE serviceHandle,
		OUT MadtEventRecordStruct *eventRecord
		);
/* not yet implemented */
IBA_API UMADT_POLLFOREVENT iba_umadt_poll_event;


/*
* WaitForAnyCompletion
*
* DESCRIPTION :
*
* Blocks on an OS sync objects until a completion event occurs.
* This routine will not perform a kernel mode context switch.
* 
*
* PARAMETERS :
*
* INPUTS :
*	serviceHandle			: The handle got in Register Call
*	completionType			: Bit map identifying  the set of completion to wait on
*	timeout					: Time in ms to wait for an event
*
* OUTPUTS:
*	None
*
* RETURNS: 
*	FSUCCESS			: Operation was successful
*   FTIMEOUT			: Operation timedout
*	FNOT_DONE			: Operation terminated by signal
* 
* Environment : User Mode
*
* NOTES:
*/

typedef
FSTATUS (UMADT_WAITFORANYCOMPLETION)(
		IN MADT_HANDLE serviceHandle,
		IN uint32 completionType,
		IN uint32 timeout
		);
IBA_API UMADT_WAITFORANYCOMPLETION iba_umadt_wait_any_compl;


/*
* QueryOsSyncObject
*
* DESCRIPTION :
*
* This routine Queries the OS Specific synchronization object.
* This routine will not perform a kernel mode context switch.
* 
*
* PARAMETERS :
* INPUTS :
*	serviceHandle			: The handle got in Register Call 
*	completionType			: Bit map identifies the wait object to retrive
*
* OUTPUTS:
*	syncType				: Type of OS specific sync object
*	syncObject				: OS specific sync object
*
* RETURNS: 
*	FSUCCESS				: Operation was successful
*   FINVALID_PARAMETER		: Invalid argumetns
* 
* Environment : User Mode
*
* NOTES:
*/

typedef
FSTATUS (UMADT_QUERYOSSYNCOBJECT)(
		IN MADT_HANDLE serviceHandle,
		IN uint32 CompletionType,
		OUT uint32 SyncType,
		OUT void * syncObject
		);
/* not yet implemented */
IBA_API UMADT_QUERYOSSYNCOBJECT iba_umadt_query_sync_object;


/*
* Deregister
*
* DESCRIPTION :
*
* This routine deregisters an agent or a manager associated with the 
* service handle.
* This routine may be expected to perform a kernel mode context switch
*
* PARAMETERS :
* INPUTS :
*	serviceHandle			: The handle got in Register Call
*
* OUTPUTS:
*	None
*
* RETURNS: 
*	FSUCCESS				: Operation was successful
*	FINVALID_PARAMETER		: Invalid argumetns
*
* Environment : User Mode
*
* NOTES:
*/

typedef
FSTATUS (UMADT_DEREGISTER)(
		IN MADT_HANDLE	serviceHandle
		);
IBA_API UMADT_DEREGISTER iba_umadt_deregister;

static __inline void
GetSmiAddrFromPath( IN const IB_PATH_RECORD *pPathRecord,
					OUT MadAddrStruct *pAddrInfo
				)
{
	pAddrInfo->DestLid	=	pPathRecord->DLID;
	pAddrInfo->PathBits = (uint8)(pPathRecord->SLID & IB_PATHBITS_MASK);
	pAddrInfo->StaticRate = pPathRecord->Rate;
	pAddrInfo->AddrType.Smi.SourceLid = pPathRecord->SLID;
}

static __inline void
GetGsiAddrFromPath( IN const IB_PATH_RECORD *pPathRecord,
					IN uint32 RemoteQPN,
					IN IB_Q_KEY RemoteQkey,
					OUT MadAddrStruct *pAddrInfo
				)
{
	pAddrInfo->DestLid	=	pPathRecord->DLID;
	pAddrInfo->PathBits = (uint8)(pPathRecord->SLID & IB_PATHBITS_MASK);
	pAddrInfo->StaticRate = pPathRecord->Rate;
	pAddrInfo->AddrType.Gsi.RemoteQpNumber = RemoteQPN;
	pAddrInfo->AddrType.Gsi.RemoteQkey=	RemoteQkey;
	pAddrInfo->AddrType.Gsi.ServiceLevel = pPathRecord->u2.s.SL;
	pAddrInfo->AddrType.Gsi.PKey = pPathRecord->P_Key;
	pAddrInfo->AddrType.Gsi.GlobalRoute = (pPathRecord->u1.s.HopLimit > 1);
	pAddrInfo->AddrType.Gsi.GRHInfo.DestGID = pPathRecord->DGID;
	pAddrInfo->AddrType.Gsi.GRHInfo.FlowLabel = pPathRecord->u1.s.FlowLabel;
	pAddrInfo->AddrType.Gsi.GRHInfo.HopLimit = (uint8)pPathRecord->u1.s.HopLimit;
	pAddrInfo->AddrType.Gsi.GRHInfo.SrcGIDIndex = 0;	/* BUGBUG assume 0 */
	pAddrInfo->AddrType.Gsi.GRHInfo.TrafficClass = pPathRecord->TClass;
}

#if defined (__cplusplus)
};
#endif


#endif  /* _IBA_UMADT_H_ */
