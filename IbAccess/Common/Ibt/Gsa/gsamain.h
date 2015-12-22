/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/
/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_IB_GSA_MAIN_H_
#define _IBA_IB_GSA_MAIN_H_


#include "datatypes.h"
#include "stl_types.h"
#include "stl_mad.h"
#include "vpi.h"
#include "sma_osd.h"
#include "ib_debug_osd.h"
#include "vpi_export.h"
#include "ib_smi.h"
#include "ib_gsi.h"
#include "ib_ibt.h"
#include "ib_generalServices.h"
#include "gsadebug.h"
#include "vca_export.h"
#include "statustext.h"
#include "ibyteswap.h"
#include "ispinlock.h"
#include "isyscallback.h"
#include "ilist.h"
#include "itimer.h"
#include "imath.h"
#include "isyscallback.h"
#include "smagsa.h"
//#include "ContextMgr.h"
#include "gsi_params.h"

#if defined (__cplusplus)
extern "C" {
#endif



//
// Definitions
//

#define	MAX_CLIENT_ID						0xff

#define GSA_RECVQ_LOW_WATER					50	// when to grow GlobalRecvQ Pool

#if defined(DBG) || defined(IB_DEBUG)

#define	GSA_CLASS_SIG						0x73616c63		// 'salc'
#define	GSA_DGRM_SIG						0x6d726764		// 'mrgd'

#endif


//
// Structures
//
extern IBT_GLOBAL IbtGlobal;


//
// Structure to holds sends
//
typedef struct _GSA_POST_SEND_LIST {
	uint32							DgrmIn;
	ATOMIC_UINT						DgrmOut;
	IBT_DGRM_ELEMENT				*DgrmList;
	boolean							SARRequest;
} GSA_POST_SEND_LIST;


//
// structure for memory pool
//
typedef struct _IBT_MEM_POOL {
	struct _IBT_MEM_POOL			*Next;			// link to Growable Pool
	struct _IBT_MEM_POOL			*Parent;		// our parent Growable Pool
	SYS_CALLBACK_ITEM				*CallbackItem;	// used for Growable Pools
	ATOMIC_UINT						GrowScheduled;	// used for Growable Pools
	
	SPIN_LOCK						Lock;			// lock
	uint32							TotalElements;	// No. of elements in pool
	uint32							Elements;		// Available for Gets
	IBT_DGRM_ELEMENT				*DgrmList;		// datagram list
	uint32							BuffersPerElement;
	uint8							ReceivePool:1;	// Send or receive pool
													// default FALSE: 
													// SENDPool-Client buffer
	uint8							Growable:1;		// head of growable pool
	GLOBAL_MEM_LIST					*MemList;		// global memlist;
	void							*UdBlock;		// registered mem block
	void							*HeaderBlock;	// header block
	IB_HANDLE						ServiceHandle;	// user's handle

#if defined(DBG) || defined(IB_DEBUG)
	uint32							SigInfo;
#endif
} IBT_MEM_POOL;


typedef struct _IBT_DGRM_ELEMENT_PRIV {
	IBT_DGRM_ELEMENT				DgrmElement;
	IBT_MEM_POOL					*MemPoolHandle;
	// TBD - replace this with a Union { pPostSendList; IsSarRecv }
	GSA_POST_SEND_LIST				*Base;			// used for sends and recvs
													// Sends: base to chain of
													// multiple sends
													// Recvs: Indicates SAR data
	GSA_POST_SEND_LIST				PostSendList;	// The actual list
	uint64							SavedSendTid;	// User's TID on a send.
	IB_HANDLE						AvHandle;		// AV Handle
	IB_LOCAL_DATASEGMENT			*DsList;		// list to Data segments
	struct	_GSA_SAR_CONTEXT		*pSarContext;	// Pointer to context data 
													// for SAR
} IBT_DGRM_ELEMENT_PRIV;



//
//   GSA_GLOBAL_INFO
//
//	This structure is used to keep track of global information and tie the 
//	various lists maintained into meaningful related entities. There is only 
//	one instance of this structure per the whole of GSA activeCAlist is used 
//	to link the list of all CA's in the system. Every time a CA is added, a 
//	new CA structure is created and linked to the end of this field in a 
//	LIFO fashion.
//
//	calock is used to guard access to the above list ( activeCAlist ).
//
//	ServiceClassList is used to link the list of all registered service 
//	classes. Every time a unique service class is registered, a service class 
//	registration record ( GSA_SERVICE_CLASS_INFO ) is created and linked to
//	this list in a LIFO fashion.
//
//	ServiceClassListLock is used to guard access to the list of service 
//	classes ( ServiceClassList ).
//
//	The interfaces of GSA MUST check this state before providing any 
//	useful functionality.
//	if FALSE, GSA services are not available.
//
//	ConfigParam contains the list of configuration parameters used to configure 
//	GSA. These values might be read from registry or from local configuration 
//	file(s) , depending on the platform we are running on.
//
//
typedef struct _GSA_GLOBAL_INFO
{
	//
	// this keeps track of internally generated client ids. Normally, a client id 
	// is an 8-bit value that is unique across a single management class. 
	// For now, this is unique across all management classes.
	//
	// ????TOBEDONE. 
	// Later, when we have more management classes , this needs to be
	// changed so that they are unique across just one management class
	//

	uint8							ClientId;


	SPIN_LOCK						CaListLock;
    SMA_OBJECT						SmObj;

	IB_HANDLE						DgrmPoolRecvQ;
    
    SPIN_LOCK						ServiceClassListLock;
    QUICK_LIST						ServiceClassList;

	// SAR Info
    SPIN_LOCK						SARSendListLock;
	QUICK_LIST						SARSendList;
    SPIN_LOCK						SARRecvListLock;
	QUICK_LIST						SARRecvList;
	ATOMIC_UINT						SARRecvDropped;

	//IB_HANDLE						NotifyHandle;	// Notification calls

	uint8							ClientIdArray[MAX_CLIENT_ID];
} GSA_GLOBAL_INFO;



//
// extern declaration for our global information structure
//
#ifndef g_GsaGlobalInfo
extern GSA_GLOBAL_INFO *g_GsaGlobalInfo;
#endif

//
// GSA_SERVICE_CLASS_INFO
//
//
//	This structure describes a registered service class . An instance of this 
//	is created every time a unique service class is registered. 
//
//	ListItem is used to link this registration record into a list.
//     
//	Serviceclasssendlist is the head of all send buffers submitted by 
//	the service class for sending. By definition these buffers are already 
//	registered with one or more CAs.
//
//	serviceclassrecvlist is the head of all receive buffers submitted by 
//	the service class for receiving. By definition, these buffers are 
//	already registered with one or more CAs.
//
//	<MgmtClass, mgmtclassversion> describe the mgmt class and version 
//	registered by the service class.
//
//	ServiceClassContext is the value supplied by the service class 
//	during registration.
//
//	ReceiveCompletionCallback is the handler supplied by the service class 
//	that is used to indicate receive completions on behalf of this service 
//	class.
//
//	sendcompletehandler is the handler supplied by the service class used 
//	to indicate send completions on behalf of this service class. 
//
//	Both ReceiveCompletionCallback and sendcompletehandler are 
//	supplied by the service class and are used by GSA to indicate the 
//	respective completion events.
//
//	MiscEventHandler is used to indicate miscellaneous events. This handler 
//	is used to indicate the following events.
//  CA being removed or disabled. In this case, the CA guid is provided 
//	as input.
//	A port being disconnected. In this case, the port GUID and the CA GUID 
//	are provided as input.
//	Other error events are also indicated. 
//
typedef struct _GSA_SERVICE_CLASS_INFO
{
	LIST_ITEM						ListItem;

	uint8							MgmtClass;
	uint8							MgmtClassVersion;

	// Flags used to indicate responder, trap processor and report processor.
	GSI_REGISTRATION_FLAGS			RegistrationFlags;

	boolean							bSARRequired;	// TRUE for SAR support

	void							*ServiceClassContext;
	GSI_SEND_COMPLETION_CALLBACK	*SendCompletionCallback;
	GSI_RECEIVE_CALLBACK			*ReceiveCompletionCallback;
	
	GSA_GLOBAL_INFO					*pGlobalInfo;

	uint8							ClientId;

#if defined(DBG) || defined(IB_DEBUG)
	uint32							SigInfo;
	ATOMIC_UINT						ErrorMsgs;
#endif
} GSA_SERVICE_CLASS_INFO;

//
// Function declarations
//
FSTATUS
GSALoad(
	IN IBT_COMPONENT_INFO		*ComponentInfo
	);

void
GSAUnload(void);

FSTATUS
GsaAddDevice(
	IN	EUI64					CaGuid,
	OUT void					**Context
	);

FSTATUS
GsaRemoveDevice(
	IN EUI64					CaGuid,
	IN void						*Context
	);

void
GsaInitGlobalSettings(void);

void
GsaOsComponentInfo(
	IN IBT_COMPONENT_INFO		*ComponentInfo
	);

FSTATUS
PrepareCaForReceives(
	IN	EUI64					CaGuid
	);

FSTATUS
GetCaContextFromAvInfo(
	IN	IB_ADDRESS_VECTOR		*AvInfo,
	IN	uint32					CaMemIndex,
	OUT	IB_HANDLE				*AvHandle,
	OUT	IB_HANDLE				*Qp1Handle,
	OUT	IB_HANDLE				*Cq1Handle,
	OUT SPIN_LOCK				**QpLock,
	OUT	IB_L_KEY				*LKey
	);

FSTATUS
GetGsiContextFromPortGuid(
	IN	EUI64					PortGuid,
	OUT	IB_HANDLE				*Qp1Handle,
	OUT	uint8					*PortNumber
	);

FSTATUS
CreateDgrmPool(
	IN	IB_HANDLE				ServiceHandle,
	IN	uint32					ElementCount,
	IN	uint32                  BuffersPerElement,
	IN	uint32					BufferSizeArray[],
	IN	uint32                  ContextSize,
	OUT IB_HANDLE				*DgrmPoolHandle
	);

FSTATUS
DestroyDgrmPool(
	IN	IB_HANDLE				DgrmPoolHandle
	);

FSTATUS
DgrmPoolGet(
	IN	IB_HANDLE				DgrmPoolHandle,
	IN OUT	uint32				*ElementCount,
	OUT	IBT_DGRM_ELEMENT		**DgrmList
	);

void
DumpDgrmElement(
	IN IBT_DGRM_ELEMENT		*DgrmElement
	);

uint32
DgrmPoolCount(
	IN	IB_HANDLE				DgrmPoolHandle
	);

uint32
DgrmPoolTotal(
	IN	IB_HANDLE				DgrmPoolHandle
	);

FSTATUS
DgrmPoolPut(
	IN	IBT_DGRM_ELEMENT		*pDgrmList
	);

FSTATUS
CreateGrowableDgrmPool(
	IN	IB_HANDLE				ServiceHandle,
	IN	uint32					ElementCount,
	IN	uint32                  BuffersPerElement,
	IN	uint32					BufferSizeArray[],
	IN	uint32                  ContextSize,
	OUT IB_HANDLE				*DgrmPoolHandle
	);

FSTATUS
DgrmPoolGrow(
	IN	IB_HANDLE				DgrmPoolHandle,
	IN	uint32					ElementCount
	);

void
DgrmPoolGrowAsNeeded(
	IN	IB_HANDLE				DgrmPoolHandle,
	IN	uint32					lowWater,
	IN	uint32					maxElements,
	IN	uint32					growIncrement
	);

FSTATUS
CreateGlobalRecvQ(void);

FSTATUS
DgrmPoolAddToGlobalRecvQ(
	IN	uint32					ElementCount
	);

void
GrowRecvQAsNeeded(void);

void
DestroyGlobalRecvQ(void);

FSTATUS
DecrementGsiRecvQPostedForPortGuid(
	IN	EUI64					PortGuid,
	OUT	uint32					*RecvQPosted
	);

FSTATUS
PostRecvMsgToPort(
	IN	uint32					ElementCount,
	IN	EUI64					PortGuid
	);

void	
GsaNotifyCallback(
	IN	IB_NOTIFY_RECORD		NotifyRecord
	);

FSTATUS
GsaNotifyClients(
	IN	uint8				ClientId,
	IN	MAD_COMMON			*pMadHdr,
	IN	IBT_DGRM_ELEMENT	*pDgrm );

// has this client requested GSI to do RMPP protocol handling for it
boolean
IsSarClient(
	IN	uint8				ClientId,
	IN	uint8				MgmtClass );

// RMPP entry points, provided by the RMPP protocol provider

// destroy all SAR Context data, used during Unload
void
DestroySARContexts(void);

// RMPP Sender - initiate a transfer which may require RMPP
FSTATUS
DoSARSend(
	IN	GSA_SERVICE_CLASS_INFO	*ServiceClass, 
	IN	GSA_POST_SEND_LIST		*PostList
	);

// Common Completion Handlers for RMPP Sender and Receiver Contexts
FSTATUS
ProcessSARRecvCompletion (
	IN	MAD						*pMad,
	IN	IBT_DGRM_ELEMENT		**ppDgrm
	);

void
ProcessSARSendCompletion (
	IN	MAD						*pMad,
	IN	IBT_DGRM_ELEMENT		*pDgrm
	);

FSTATUS
GsiDoSendDgrm(
	GSA_SERVICE_CLASS_INFO		*ServiceClass,
	IN	GSA_POST_SEND_LIST		*pPostList
	);

void GsiSendInvalid(
	IN	IBT_DGRM_ELEMENT		*pDgrm
	);

#if defined (__cplusplus)
};
#endif


#endif	// _IBA_IB_GSA_MAIN_H_
