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

#ifndef _IBA_SD_INTERNAL_
#define _IBA_SD_INTERNAL_

#include <stl_sd.h>
#include <stl_types.h>
#include <ib_gsi.h>
#include <dbg.h>
#include <ispinlock.h>
#include <itimer.h>
#include <imutex.h>
#include <ibt.h>
#include <isyscallback.h>
#include <query.h>
#include <ib_gsi.h>
#include <ib_sd_priv.h>
#include <sd_params.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SUBNET_DRIVER_TAG               ((uint32) 0x44627553) //'DbuS'

extern ATOMIC_UINT IsTimerStarted;

#define StartTimerHandler(pTimerObject, TimeInMS) {\
	if (AtomicExchange(&IsTimerStarted, 1) == 0) {\
			TimerStart(pTimerObject, /* ms*/ TimeInMS);\
		}\
	}
		
#define SD_MAX_SA_NDGRM		64	// SA query/requests - default MaxSaOutstanding
#define SD_MAX_RESP_NDGRM	64	// Notify Responses

typedef uint32  ClientId;

extern ATOMIC_UINT Outstanding;	// number of queries/operations in progress
 
extern SPIN_LOCK *pCaPortListLock;
extern QUICK_LIST *pCaPortList;

extern IB_HANDLE CaNotifyHandle;
extern uint32 NumLocalCAs;
extern EUI64 *pLocalCaGuidList;
extern SPIN_LOCK *pLocalCaInfoLock;
extern IB_CA_ATTRIBUTES *pLocalCaInfo;

// This list contains all the registered SD clients
extern SPIN_LOCK *pClientListLock;
extern QUICK_LIST *pClientList;

// This list contains all the active queries
// The lock for this list protects the list itself and the
// QueryElements on the list.  It also protects changes to QState for
// all the elements on the list.
extern LOCKED_QUICK_LIST *pQueryList;

// This list contains all the completed queries or queries scheduled for
// a defered destroy (so destroy could occur in a preemptable context)
extern LOCKED_QUICK_LIST *pCompletionList;

// This list contains all the scheduled but inactive child queries
// when a child query becomes active, it is placed on the pQueryList
extern LOCKED_QUICK_LIST *pChildQueryList;
extern ATOMIC_UINT ChildQueryActive;

// This mutex prevents races between multiple child processes updating
// their parents pClientResult at the same time.  In reality, since there
// is one thread for SD GSA callbacks, there will be no contention for
// this mutex, hence there is no advantage to putting it into the
// parent query element.  Also note that for parent queries, the
// pClientResult of the parent is not modified until all children are done
extern MUTEX *pChildGrowParentMutex;

extern TIMER *pTimerObject;

extern ATOMIC_UINT SdWorkItemQueued;
extern ATOMIC_UINT SdWorkItemsInProcess;
extern boolean SdUnloading;

//GSI related handles
extern IB_HANDLE GsiSAHandle;

extern IB_HANDLE sa_poolhandle;
extern IB_HANDLE sa_represp_poolhandle;

typedef struct _SD_SERVICECONTEXT {
	uint8 MgmtClass; //To distingusish when we receive a packet
}SD_SERVICECONTEXT, *PSD_SERVICECONTEXT;


typedef struct  _ClientListEntry  {
    LIST_ITEM ListItem; //Linked list pointers, Do not move this element
    uint32 ClientId;
    CLIENT_CONTROL_PARAMETERS ClientParameters;
    QUICK_LIST ClientTraps;
} ClientListEntry, *PClientListEntry;

typedef struct _QueryListEntry  {
    PClientListEntry pClient;
} QueryListEntry, *PQueryListEntry;


FSTATUS
SdCommonCreate(
    void
    );

FSTATUS
SdCommonClose(
    void
    );

void
SetDefaultSDDriverTunables(
    void
    );

FSTATUS
ReadDriverTunables(
    void
    );


FSTATUS
InitializeDriver(
    void
    );

FSTATUS
UnloadDriver(
    void
    );

FSTATUS
InitializeList(
	IN	QUICK_LIST **ppList,
	IN	SPIN_LOCK **ppListLock
);


void
DestroyList(
	IN	QUICK_LIST *pList,
	IN	SPIN_LOCK *pListLock
    );

FSTATUS
CommonQueryPortFabricInformation(
    IN CLIENT_HANDLE ClientHandle,
    IN EUI64 PortGuid,
    IN PQUERY Query,
    IN SDK_QUERY_CALLBACK *Callback,
    IN COMMAND_CONTROL_PARAMETERS *pQueryControlParameters OPTIONAL,
    IN void *Context OPTIONAL
	);

FSTATUS
CommonPortFabricOperation(
    IN CLIENT_HANDLE ClientHandle,
    IN EUI64 PortGuid,
    IN FABRIC_OPERATION_DATA* pFabOp,
    IN SDK_FABRIC_OPERATION_CALLBACK *Callback,
    IN COMMAND_CONTROL_PARAMETERS *pCmdControlParameters OPTIONAL,
    IN void* Context OPTIONAL
    );

FSTATUS
CommonSdDeregister(
    IN CLIENT_HANDLE ClientHandle
	);

FSTATUS
ValidateAndFillSDQuery(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	);

FSTATUS
ValidateAndFillSDFabOp(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	);

FSTATUS ValidateClientHandle(
    IN CLIENT_HANDLE ClientHandle,
    OUT CLIENT_CONTROL_PARAMETERS *pClientParams
	);

FSTATUS CheckPortActive(
    	IN OUT EUI64 *pPortGuid
		);

FSTATUS 
SdHandleChildQuery(
	PQueryDetails pQueryElement,
	void *buffer,
	uint32 bufsize
	);


FSTATUS
InitializeIbt( void);

SYS_CALLBACK_ITEM *
InitializeWorkItemqueue( void); 

void
TimerHandler( IN void *Context);

void
SdSendNextOutstanding(void);

void
ProcessChildQueryQueue( void);

//void
//ProcessDelayedWorkQueue(
	//void* Key, //Functional Device OBJECT
	//void *Context);

void
ScheduleCompletionProcessing(void);

void
ProcessCompletionQueue(
	void* Key, //Functional Device OBJECT
	void *Context
	);

boolean
FreeCancelledQueryElement(
	PQueryDetails pQueryElement
	);

void
FreeQueryElement(
	PQueryDetails pQueryElement
	);

FSTATUS
RegisterWithGSI(
	void
	);

void
DeRegisterWithGSI(
	void
	);

FSTATUS
SdGsaRecvCallback(
	IN	void					*ServiceContext,
	IN	IBT_DGRM_ELEMENT		*pDgrmList
	);

void
SdGsaSendCompleteCallback(
	IN	void					*ServiceContext,
	IN	IBT_DGRM_ELEMENT		*pDgrmList

	);

CA_PORT*
GetCaPort(
	EUI64 PortGuid
	);

void
AddCa(
	IN EUI64 CaGuid
	);

void
RemoveCa(
	IN EUI64 CaGuid
	);

FSTATUS
InitializeCaPortInfo(
    IN EUI64 PortGuid
	);

uint32
GetNumTotalPorts(
    IN void
	);

FSTATUS
GetLocalCaInfo(
    void
    );

void
LocalCaChangeCallback(
    IN IB_NOTIFY_RECORD
	);

QueryDetails**
BuildSAChildQueriesForPathRecord(
	PQueryDetails pRootQueryElement, 
	uint64 *pDestPortGuidsList,
	uint32 DestPortGuidsCount
);

FSTATUS
SubnetAdmInit(
    void
    );

FSTATUS 
SubnetAdmRecv(
	PQueryDetails pQueryElement,
	void *buffer,
	uint32 messageSize
	,IBT_DGRM_ELEMENT		*pDgrmList
    );

FSTATUS
SendQueryElement(
    PQueryDetails pQueryElement,
	boolean FirstAttempt
    );


void
FillBaseGmpValues(
    IN PQueryDetails piQuery
	);

void
SetDefaultClientParameters(
	void
	);

void
CancelClientQueries(
	IN CLIENT_HANDLE ClientHandle
	);

#ifdef __cplusplus
};
#endif

#endif  // _IBA_SD_INTERNAL_
