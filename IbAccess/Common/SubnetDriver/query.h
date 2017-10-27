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


#include <stl_sd.h>
#include <ib_sd_priv.h>
#include <stl_types.h>
#include <ib_generalServices.h>
#include <dbg.h>
#include <ilist.h>
#include <ispinlock.h>
#include <sd_params.h>

#ifndef _IBA_SD_QUERY_
#define _IBA_SD_QUERY_

#ifdef __cplusplus
extern "C" {
#endif

#define GSI_QP                  1
#define GSI_QKEY                  1

typedef enum _SD_SENT_CMD {
	UnknownCommand = 0,
	GetClassportInfo,
	UserQueryFabricInformation,
	UserFabricOperation
} SD_SENT_CMD;

typedef enum _QUERY_STATE {
	ReadyToSend = 0,	// on queue, not yet sent
	NotAbleToSend,		// try again later, port down, no buffers, etc
	WaitingForResult,	// sent, awaiting reply
	BusyRetryDelay,		// SA reported Busy, delay and retry
	WaitingForChildToComplete,	// primary request done, child queries queued
	ProcessingResponse,	// premptable memory allocation while process response
						// this means another thread holds a reference to
						// this pQueryElement, so we can change state to
						// QueryDestroy, but cannot free it nor otherwise
						// time it out.
	QueryComplete,		// query is done (or failed), ready to deliver to client
	QueryDestroy		// query cancelled while ProcessingResponse, destroy
						// when come back from preemption in processing response
} QUERY_STATE;


typedef struct _QueryDetails  {
    LIST_ITEM QueryListItem;
	boolean IsSelfCommand;
	SD_SENT_CMD SdSentCmd;
	QUERY_STATE QState;
	CLIENT_HANDLE ClientHandle;
    void *UserContext;
    COMMAND_CONTROL_PARAMETERS ControlParameters;
	uint64 CmdSentTime;	// when last sent or tried to send
						// only valid for WaitingForResult, NotAbleToSend
						// Note that ReadyToSend state is not timed out
	uint32 RetriesLeft;

	FSTATUS					Status;
	uint32					MadStatus;

	union {
		struct {	// QueryFabricInformation
    		SDK_QUERY_CALLBACK *UserCallback;
    		QUERY UserQuery;
			QUERY_RESULT_VALUES * pClientResult;
		} query;
		struct {	// FabricOperation
			FABRIC_OPERATION_DATA		FabOp;
			SDK_FABRIC_OPERATION_CALLBACK	*FabOpCallback;
		} op;
	} cmd;

    // The client query cannot be satisfied until all related serialized and
    // parallel queries have completed or timed out. Serialized queries can
    // only be issued after it's referring query has completed. Parallel
    // queries can be issued in parallel
	struct _QueryDetails *pParentQuery;
	uint32 NoOfChildQuery;
	char *ChildResults;
	uint32 TotalBytes;
	// count of Child queries which are in ProcessingResponse state and are
	// holding a reference to parent but are not holding the
	// pQueryList->m_Lock. hence parent query cannot be freed.
	uint32 ChildProcessingResponseCount;
	
	uint32 TotalBytesInGmp;
	union {
    	MAD *pMad;
    	MAD_RMPP *pGmp;
    	SA_MAD *pSaMad;
	} u;
	EUI64 PortGuid;			// port to send query via
} QueryDetails, *PQueryDetails;



typedef 
FSTATUS (*PInitHandler) (
    void
    );

typedef 
FSTATUS (*PDeinitHandler) (
    void
    );

typedef 
FSTATUS (*PSendHandler) (
    PQueryDetails
    );

typedef 
FSTATUS (*PRecvHandler) (
	PQueryDetails,
	void *buffer,
	uint32 bufsize
	,IBT_DGRM_ELEMENT		*pDgrmList
    );

typedef struct _CA_PORT {
    EUI64 CaGuid;
    EUI64 PortGuid;
	uint16 DefaultPkeyIndex;	// index of default Pkey for this port
	boolean SdSMAddressValid;
    boolean bInitialized;
    IB_LID Lid;
    IB_SL ServiceLevel;
    uint32 RemoteQp;
    uint32 RemoteQKey;  
    EUI64 SaPortGuid;
    uint8 StaticRate;  // enum IB_STATIC_RATE
    uint16 PkeyIndex;
	uint64 SubnetPrefix;
	uint32 RedirectedQP;
	uint32 RedirectedQKey;
	uint16 RedirectedLid;
	uint8 RedirectedSL;
	uint8 RedirectedStaticRate;	// enum IB_STATIC_RATE
	uint16 RedirectedPkeyIndex;
    LIST_ITEM CaPortListItem;
} CA_PORT;


extern SYS_CALLBACK_ITEM * pCallbackItemObject;

#ifdef __cplusplus
};
#endif

#endif  // _IBA_SD_QUERY_
