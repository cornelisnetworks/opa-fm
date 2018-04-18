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

#ifndef __IBA_IB_SMI_H__
#define __IBA_IB_SMI_H__	(1)


#if defined(VXWORKS)

/* kernel mode SMI Interfaces */

#include <iba/vpi.h>
#include "iba/stl_types.h"
#include "iba/stl_sm_types.h"


#if defined (__cplusplus)
extern "C" {
#endif


/*
 * Defines 
 */



/*
 * Interface Definitions
 *
 * The SM is a trusted client and the only client to the SMA.
 * Since the data and structures maintained by SMA may be of interest
 * to the SM and most times be duplicated, it may be desirable to share
 * as much common info as possible between the two modules.
 * The use of SMA_OBJECT helps attain this goal. Only those values that
 * are of use to the SM are exposed, keeping the rest as private to SMA
 * and opaque to SM.
 */


/*
 * Per Port definitions
 */

typedef	struct	_SMA_PORT_BLOCK {
	EUI64			GUID;			/* Port GUID */
	uint8			PortNumber;
	
	uint8			NumPKeys;		/* Number of Pkeys in the Pkey table */
	uint8			NumGIDs;		/* Number of GIDs in the GID table */
	
	IB_PORT_STATE	State;
	IB_PORT_ADDRESS	Address;		/* Port addressing info */

#if 0
	/* these are not used */
	/* Sma */
	IB_HANDLE		QpHandle;
	uint32			SendQDepth;		/* Maximum outstanding WRs permitted */
									/* on the SendQ of the QP */
	uint32			RecvQDepth;		/* Maximum outstanding WRs permitted  */
									/* on the RecvQ */
#endif
	
} SMA_PORT_BLOCK;


/*
 * An array of ports based on number of ports
 */

typedef struct _SMA_PORT_TABLE {
	SMA_PORT_BLOCK	PortBlock[1];
} SMA_PORT_TABLE;


/* Per Channel Adapter List */
typedef	struct	_SMA_CA_OBJECT {
	/* */
	/* Ca Specifics */
	/* */

	EUI64		CaGuid;				/* Node GUID */
	void		*SmaCaContext;		/* Context that uniquely identifies a CA */
	uint32		Ports;				/* Number of ports on this CA */
    IB_CA_CAPABILITIES Capabilities; /* Capabilities of this CA */
	
	SMA_PORT_TABLE	*PortTbl;
} SMA_CA_OBJECT;


typedef struct _SMA_CA_TABLE {
	SMA_CA_OBJECT	CaObj[1];		/* An array of Ca structures */
} SMA_CA_TABLE;



/* Sma data structure information */
typedef	struct	_SMA_OBJECT {
	uint32			NumCa;			/* Total CA's known */
	SMA_CA_TABLE	*CaTbl;			/* Pointer to LList of CA structures */
} SMA_OBJECT;





/* All Send/Recvs use this block */
typedef struct _SMP_BLOCK {
	void		*Next;				/* pointers maintained exclusively by SMA */
	void		*Prev;				/* pointers maintained exclusively by SMA */

	FSTATUS		Status;

	void		*pSendContext;
	void		*SmaCaContext;		/* Context that identifies CA */
	uint8		PortNumber;
	
	uint32		SmpByteCount;		/* Used size of *Smp in bytes */
									/* When zero indicates MAD_BLOCK_SIZE */
	uint32		RecvByteCount;		/* Total size of receive completion */
	void		*Smp;				/* Pointer to SMP/MAD */
	STL_LID		DLID;				/* destination LID */
	STL_LID		SLID;				/* source LID */

	IB_PATHBITS	PathBits;			/* path bits */
	uint8		StaticRate;			/* enum IB_STATIC_RATE */
	IB_SL		ServiceLevel;


	IB_GRH		*Grh;				/* Pointer to GRH */
} SMP_BLOCK;


/*
 * Callback definitions
 */

/*
 * SM_RCV_CALLBACK
 *
 *	The receive callback that will be invoked for SMP's received.
 *	There may be more than one receives in the list. The SM should
 *	lookup the list till a NULL pointer is obtained on search of
 *	the link list.
 *
 * INPUTS:
 *	Context			-user's context supplied during SmaOpen()
 *	SmpBlockList	-List of SMP's received.
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *
 */
typedef 
void (SM_RCV_CALLBACK)(
	IN	void		*Context,
	IN	SMP_BLOCK	*SmpBlockList
	);


/*
 * SM_POST_CALLBACK
 *
 *	The Post callback that will be called on all successful posts. The
 *	callback will pass back the user's context and a list of SMP's that
 *	were previously posted.
 *
 *	The callback is registered with SMA during SmaPostSend()
 *
 * INPUTS:
 *	PostContext		-user's context supplied during the post.
 *	SmpBlockList	-List of SMP's posted.
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *
 */
typedef 
void (SM_POST_CALLBACK)(
	IN	void		*PostContext,
	IN	SMP_BLOCK	*SmpBlockList
	);


typedef enum _SMA_EVENT {
	EVENT_PORT,
	EVENT_CA,
	EVENT_SEND_ERROR
	/*EVENT_RECV_ERROR */
} SMA_EVENT;



/*
 * SM_EVENT_CALLBACK
 *
 *	The Event callback that will be called for any events generated as
 *	listed in SMA_EVENT. The callback will specify the type of event
 *	and cause with optional paramaters.
 *
 * INPUTS:
 *	EventCode		-The type of event as listed in SMA_EVENT
 *	Cause			-The cause can point out to a port number or SMP_BLOCK
 * 					    data pointed to only valid during duration of call
 *	Context			-If a post error, the user's context supplied during 
 *					 the post.
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *
 */
typedef 
void (SM_EVENT_CALLBACK)(
	IN	SMA_EVENT		EventCode,			/* event that occured */
	IN	void			*Cause,				/* cause; can be Smp_block/port no */
	IN	void			*Context			/* if PostSend */
	);



/*
 * Interface API's
 */

/*
 * SmaOpen
 *
 *	Returns a pointer to an Object that refereces the SMA structures. 
 *	The structures should be referenced as read-only.
 *	The SM can use the handles of CA/QP/CQ's for its own usage if required.
 *	No tracking of resources will be done if Handles are used.
 *
 * INPUTS:
 *	RcvCallback		-Callback to be invoked if SMP's are received.
 *	PostCallback	-Callback to be invoked on completion of SMP's posted.
 *	EventCallback	-Event callback for all events.
 *	Context			-User's Global context supplied back in Event callback
 *
 * OUTPUTS:
 *	SmObject		-A pointer to an Object that references into the 
 *					 SMA structures.
 *
 * RETURNS:
 *
 */
typedef FSTATUS
(SMA_OPEN)(
	IN	SM_RCV_CALLBACK		*RcvCallback,
/*	IN	SM_POST_CALLBACK	*PostCallback, */
	IN	SM_EVENT_CALLBACK	*EventCallback,
	IN	void				*Context,
	OUT	SMA_OBJECT			**SmObject
	);
IBA_API SMA_OPEN iba_smi_open;


/*
 * SmaClose
 *
 *	Dereferences usage of the SMA used previously by SmaOpen().
 *	No tracking of resources will be done if Handles are used.
 *
 * INPUTS:
 *	SmObject		-Object returned in the Open call.
 *
 * OUTPUTS:
 *
 * RETURNS:
 *
 */
typedef FSTATUS
(SMA_CLOSE)(
	IN	SMA_OBJECT	*SmObject
	);
IBA_API SMA_CLOSE iba_smi_close;


/*
 * SmaPostSend
 *
 *	All SMPs are sent out by the SM with this call. The caller can send one 
 *	to many SMPs by listing each SMP through a linked list of SMP_BLOCKs. 
 *	The user will specify the total SMPs contained in the list.
 *
 * INPUTS:
 *	SmObject		-Object returned in the Open call.
 *	NumSmps			-Number of SMPs in the list
 *	SmpBlockList	-Linked list of send requests contained in SMP_BLOCKs
 *	PostCallback	-A callback for SMPs Posted through SmpBlockList
 *	PostContext		-user context that will be passed back on a post callback
 *
 * OUTPUTS:
 *
 *
 * RETURNS:
 *
 *
 */
typedef FSTATUS
(SMA_POST_SEND)(
	IN	SMA_OBJECT			*SmObject,
	IN	uint32				NumSmps,
	IN	SMP_BLOCK			*SmpBlockList,
	IN	SM_POST_CALLBACK	*PostCallback,
	IN	void				*PostContext
	);
IBA_API SMA_POST_SEND iba_smi_post_send;


/*
 * SmaPostRecv
 *
 *	The caller can notify the SMA to recieve these many requests on this port.
 *	If the caller is expecting a known number of replies/receives, it should 
 *	inform the SMA in advance to queue additional receives so as not to drop 
 *	them. This call does not gaurantee that the SMA will not drop all
 *	incoming requests. However, it can help the SMA decide the traffic
 *	on this port.
 *
 * INPUTS:
 *	SmObject		-Object returned in the Open call.
 *	SmacaContext	-Context to Channel Adapter to post receives on
 *	PortNumber		-Incoming Port on Channel Adapter.
 *	NumSmps			-Number of Smps that the caller is expecting on port 
 *					 specified.
 *
 * OUTPUTS:
 *
 *
 * RETURNS:
 *
 *
 */
typedef FSTATUS
(SMA_POST_RECV)(
	IN	SMA_OBJECT	*SmObject,
	IN	void		*SmaCaContext,
	IN	uint8		PortNumber,
	IN	uint32		NumSmps
	);
IBA_API SMA_POST_RECV iba_smi_post_recv;

	
/*
 * SmaRecv
 *
 *	Returns any SMPs received. If the call is successful, the NumSmps will
 *	indicate the number of SMPS received and the SmpBlockList will hold 
 *	a list of SmpBlocks received.
 *
 * INPUTS:
 *	SmObject		-Object returned in the Open call.
 *	NumSmps			-Number of SMPs received.
 *	SmpBlockList	-A Linked list of receives with specific information.
 *					 The list contains individual SMP_BLOCKs and the
 *					 elements in list is indicated by NumSmps.
 *
 * OUTPUTS:
 *
 *
 * RETURNS:
 *
 *
 */
typedef FSTATUS
(SMA_RECV)(
    IN	SMA_OBJECT	*SmObject,
	IN uint32		*NumSmps,
	OUT SMP_BLOCK	**SmpBlockList
	);
IBA_API SMA_RECV iba_smi_recv;



/*
 * Common buffer calls for Smp's on requested Hca
 *
 *
 *	Usage: 
 *
 *	The SMA allocates all SMP buffers for sends and receives. Since these buffers
 *	are of a constant size (MAD size), allocation size is known at all times and hence 
 *	not specified in calls.
 *
 *	The following rules apply for all SMP buffer usage:
 *		* Cut down the requirements on system memory is reduced by half as there is no
 *		  copy administered between Sends and Receives. The same SMP buffers are 
 *		  passed to/from the hardware. All SMP buffers come from registered memory.
 *		* The buffers are not tied down to a particular Channel Adapter. Hence they can
 *		  be sent to or received from any CA. The memory registration is taken care of
 *		  by the SMA.
 *		* The buffers are not required to be maintained by the Subnet Manager. All
 *		  tracking is done by the SMA. 
 *		* Once a buffer is submitted by a SmaPost or SmaSetLocal call to the SMA, it
 *		  will be not returned back.
 *		* There is no rule maintained as to the order of these buffers. The Subnet Manager 
 *		  cannot except the same buffers to be passed in by the SMA.
 *		* All buffers allocated and returned through any send/receive calls hold the same
 *		  size and is equal to the size of a MAD structure of 256 bytes.
 *		* A buffer once allocated from pool can be rturned back to pool by SmaPost() and
 *		  SmaReturnToPool().
 *
 */


/*
 * SmaGrabFromPool
 *
 *	The SM can request registered memory from the SMA using this call. This memory can
 *	be then used to send Smp's. All memory received from this call will be in Smp sizes.
 *
 * INPUTS:
 *	SmObject		-Object returned in the Open call.
 *	NumSmps			-Number of blocks of memory that the caller wants in SMP sizes.
 *	SmpBlockList	-A pointer to hold the SMPs requested.
 *
 * OUTPUTS:
 *	NumSmps			-Number of blocks of memory allocated.
 *	SmpBlockList	-Pointer filled with the first SMP_BLOCK entry that contains
 *					 the linked list of SMA_BLOCKs allocated.
 *
 * RETURNS:
 *
 */
typedef FSTATUS
(SMA_GRAB_FROM_POOL)(
	IN	SMA_OBJECT		*SmObject,
	IN	OUT	uint32		*NumSmps,
	OUT SMP_BLOCK		**SmpBlockList
	);
IBA_API SMA_GRAB_FROM_POOL iba_smi_pool_get;


/*
 * SmaReturnToPool
 *
 *	The caller can release memory back to the SMA using this call.
 *
 * INPUTS:
 *	SmObject		-Object returned in the Open call.
 *	SmpBlockList	-A pointer to the linked list of SMA_BLOCKs to be freed.
 *
 * OUTPUTS:
 *
 *
 * RETURNS:
 *
 */
typedef FSTATUS
(SMA_RETURN_TO_POOL)(
	IN	SMA_OBJECT		*SmObject,
	IN SMP_BLOCK		*SmpBlockList
	);
IBA_API SMA_RETURN_TO_POOL iba_smi_pool_put;


/*
 * SmaUpdateInfo
 *
 *	The caller uses this function to update the SMA_OBJECT to reflect any
 *	changes to Channel adapters and their states. The call is done 
 *	immedeatly after an state change event is received by the caller.
 *
 * INPUTS:
 *	SmObject		-Object returned in the Open call.
 *
 * OUTPUTS:
 *	SmObject		-Updated Object information.
 *	Event			-Update information based on this event.
 *
 * RETURNS:
 *
 *
 */
typedef FSTATUS
(SMA_UPDATE_INFO)(
	IN OUT	SMA_OBJECT		*SmObject
	/*IN	SMA_EVENT_TYPE		Event */
	);
IBA_API SMA_UPDATE_INFO iba_smi_update_info;

#if defined (__cplusplus)
};
#endif

#endif /* VXWORKS */
#endif /* __IBA_IB_SMI_H__ */
