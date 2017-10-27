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

#ifndef _IBA_IB_SMA_SMAMAIN_H_
#define _IBA_IB_SMA_SMAMAIN_H_ (1) // suppress duplicate loading of this file




#include "datatypes.h"
#include "stl_types.h"
#include "stl_mad_priv.h"
#include "vpi.h"
#include "sma_osd.h"
#include "ib_debug_osd.h"
#include "vpi_export.h"
#include "ib_smi.h"
#include "ibt.h"
#include "smadebug.h"
#include "vca_export.h"
#include "statustext.h"
#include "ibyteswap.h"
#include "ispinlock.h"
#include "ithread.h"
#include "imutex.h"
#include "ievent.h"
#include "imath.h"
#include "smagsa.h"
#include "ibnotify.h"
//#include "vpdmad_export.h"

#ifdef __cplusplus
extern "C" {
#endif
//
// These definitions will not be exposed to the SM
//


//
// Global settings. Initialized at load time
//

typedef struct _SMA_STORED_SETTINGS {
	uint32		MaxCqSize;			// Maximum size a CQ will have

	uint32		SendQDepth;
	uint32		RecvQDepth;

	uint32		MaxUsers;			// Maximum no. of SMI users supported.
	uint32		PreAllocSMPsPerDevice;	// Minimum SMPs to Preallocate 
										// at each AddDevice
	uint32		MinSMPsPerAllocate;	// Minimum no. of SMPs that will be 
									// allocated at a time. 
									// This will be rounded up to a page size.
	uint32		MinSMPsToPostPerPort;// number of SMPs to post per port
	uint32		MaxSMPs;			// Maximum SMPs that can be allocated 
									// on a system
	uint32		MemTableSize;		// Size of memory table used across CA's
} SMA_STORED_SETTINGS;


//
// These settings are reflected as defaults into SMA_STORED_SETTINGS
// defined globally
//
#ifdef VXWORKS
#define DEFAULT_SMA_SETTINGS {													\
				(4*1024),					/* MaxCqSize */					  \
				64, 64,						/* SendQDepth, RecvQDepth */	  \
				25,							/* MaxUsers */					  \
				((32*1024)/sizeof(MAD)*8),	/* PreAllocSMPsPerDevice = 128 */ \
				(32*1024)/sizeof(MAD),		/* MinSMPsPerAllocate */		  \
				63,							/* MinSMPsToPostPerPort */		  \
				((32*1024)/sizeof(MAD)*32),	/* MaxSMPs = 512 */				  \
				(32*1024) };					/* MemTableSize */
#else
#define DEFAULT_SMA_SETTINGS {													\
				(4*1024)*5,					/* MaxCqSize */					  \
				64*5, 64*5,					/* SendQDepth, RecvQDepth */	  \
				25,							/* MaxUsers */					  \
				((4*1024)/sizeof(MAD)*8*2),	/* PreAllocSMPsPerDevice = 256 */ \
				(4*1024)/sizeof(MAD)*2,		/* MinSMPsPerAllocate = 32 */	  \
				63,							/* MinSMPsToPostPerPort */		  \
				((4*1024)/sizeof(MAD)*32)*5,/* MaxSMPs = 512*5 */			  \
				(4*1024) };					/* MemTableSize */
#endif



//
// User object maintained. This private structure is used internally
//
typedef struct _SMA_OBJECT_PRIVATE {

	// exposed to user
	struct _SMA_OBJECT		SmaObj;

	struct	_u {
		size_t				UserNo;
		SM_RCV_CALLBACK		*RcvCallback;
		void				*Context;
		SM_EVENT_CALLBACK	*EventCallback;
	}u;

} SMA_OBJECT_PRIVATE;

// User table
typedef struct _SMA_USER_TABLE {
	size_t					Total;
	size_t					Current;

	struct {
		void				*Context;
		SM_RCV_CALLBACK		*RcvCallback;
		SM_EVENT_CALLBACK	*EventCallback;
	} DefRecv;

	SMA_OBJECT_PRIVATE		*SmUser;
} SMA_USER_TABLE;


// Pool Memory
typedef struct _POOL_LIST {
	void 	*Prev;
	void 	*Next;
} POOL_LIST;

typedef struct _POOL_DATA_BLOCK {
	SMP_BLOCK	Block;

	void		*Base;				// Base to _SMP_LIST
	size_t		CaMemIndex;			// Memroy index for SMP enclosed
	IB_HANDLE	AvHandle;			// AV Handle for sends
	boolean		CQTrigger;			// Trigger CQ
}POOL_DATA_BLOCK;

typedef struct _POOL_BIN {
		// Lock heirachy, get in following order if need both
		//	 Mutex
		//   then Lock
	SPIN_LOCK			Lock;		// protects all but MemList and TotalAllocated
	uint32				NumBlocks;
	POOL_DATA_BLOCK		*Head;
	POOL_DATA_BLOCK		*Tail;
	uint32				CurrentIndex;
	MUTEX				Mutex;		// protects MemList and TotalAllocated
	GLOBAL_MEM_LIST		*MemList;
	uint32				TotalAllocated;
} POOL_BIN;

typedef struct _UD_TOTAL_BLOCK {
	MAD		Mad;
} UD_TOTAL_BLOCK;

typedef struct	_SMP_LIST {
	void		*Next;				// pointers maintained exclusively by SMA
	void		*Prev;				// pointers maintained exclusively by SMA

	SMA_OBJECT	*SmObject;			// User's info
	uint32		SmpsIn;				// Total no. of SMPs in list
	ATOMIC_UINT	SmpsOut;			// Reserved. 
	uint32		SmpsInError;		// flag for errors. uint32 used instead
									// of boolean for pointer alignment
	SMP_BLOCK	*SmpBlock;			// An array of SMP blocks

	SM_POST_CALLBACK	*PostCallback;
	void				*Context;	// Context provided by caller to track 
									// Post requests (not used by SMA)
} SMP_LIST;


typedef struct _SMP_POOL {
	uint32		NumSmps;			// Total no. of SMPs in list
	STL_SMP		*Smp[1];			// An array of SMPs
} SMP_POOL;

typedef struct _RECV_BLOCK {
	POOL_DATA_BLOCK	*Head;
	POOL_DATA_BLOCK	*Tail;
	uint32			Count;
} RECV_BLOCK;

typedef struct _SMA_Q {
	SPIN_LOCK		Lock;			// Lock used to access database
	uint32			Count;			// number of items on Q

	union {
		struct {
			// posted to send
			SMP_LIST		*Head;
			SMP_LIST		*Tail;
			// sent
		} SEND;

		// posted to recv
		// received
		struct {
			POOL_DATA_BLOCK	*Head;
			POOL_DATA_BLOCK	*Tail;
		} RECV;
	} s;

} SMA_Q;




//
// Global Info
//
typedef struct {
//	SPIN_LOCK		Lock;			// Lock used to access database

	// Memory tables
	uint32			NumCa;			// Total CA's known
	SMA_CA_OBJ_PRIVATE	*CaObj;
	SPIN_LOCK		CaListLock;		// protects NumCa, CaObj list and CaObj->UseCount
	
	// capabilities published on all CA ports
	uint32			IsConnectionManagementSupported:1;
	uint32			IsSNMPTunnelingSupported:1;
	uint32			IsDeviceManagementSupported:1;
	uint32			IsVendorClassSupported:1;

	IB_WORK_REQ		*WorkReqSend;	
	IB_WORK_REQ		*WorkReqRecv;	

	// Global GRH used for all receives
	GLOBAL_MEM_LIST	*GlobalGrh;

	
	// Memory container
	POOL_BIN		Bin;			// Common Memory block area storage

	// Software Qs for SMI
	//SMA_Q			SQ;				// Send Queue
	SMA_Q			RQ;				// Recv Queue
	
	// user info
	size_t			NumUser;		// no. of users
	SMA_USER_TABLE	*SmUserTbl;		// Table to Sm users
} SMA_GLOBAL_INFO;

typedef enum {
	SMI_DISCARD =0,		// discard packet
	SMI_SEND,			// output on wire
	SMI_TO_LOCAL_SM,	// provide to local SM
	SMI_TO_LOCAL,		// provide to local SMA/SM
	SMI_LID_ROUTED		// LID routed SMP, destination depends on context
} SMI_ACTION;

#ifndef g_Sma
	extern SMA_GLOBAL_INFO	*g_Sma;
#endif

#ifndef g_Settings
	extern SMA_STORED_SETTINGS g_Settings;
#endif

/* additional global parameters for VpdMad Sma handling */
extern uint32	SmaTrapDisable;	// Disable Sending of Traps
extern uint32	SmaTrapLimit;	// Limit on Traps sent before internally repress
extern uint32	SmaMKeyCheck;	// should MKey checking be enabled
extern uint32	SmaCache;		// should SMA Caching be enabled


// Forward declarations
FSTATUS
SMALoad(
	IN IBT_COMPONENT_INFO		*ComponentInfo
	);

void
SMAUnload(void);

FSTATUS 
SmaAddDevice(
	IN	EUI64					CaGuid, 
	OUT	void					**Context
	);

FSTATUS 
SmaRemoveDevice(
	IN EUI64					CaGuid, 
	IN void						*Context
	);

FSTATUS
SmaPostReturnSmp(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	POOL_DATA_BLOCK			*SmpBlock
	);

FSTATUS
SmaPostSmp(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	POOL_DATA_BLOCK			*Req
	);

boolean
SmaProcessSmaSmpRcvd(
	IN	POOL_DATA_BLOCK			*SmpBlock
	);

SMI_ACTION
SmaProcessSmiSmp(
	IN	SMA_CA_OBJ_PRIVATE		*pCaObj,
	IN	POOL_DATA_BLOCK			*pSmpBlock,
	IN	boolean					FromWire
	);

SMI_ACTION
SmaProcessLocalSmp(
	IN	SMA_CA_OBJ_PRIVATE		*pCaObj,
	IN	POOL_DATA_BLOCK			*pSmpBlock
	);

void
SmaAddToSendQ(
	IN	POOL_DATA_BLOCK			*SmpBlock
	);
	
void
SmaAddToRecvQ(
	IN	POOL_DATA_BLOCK			*SmpBlock
	);

void
SmaProcessCQ(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN OUT	RECV_BLOCK			*RecvBlock
	);

void 
SmaCompletionCallback(
	IN	void					*CaContext	// a SMA_CA_OBJ_PRIVATE*
	);

void 
SmaGsaCompletionCallback(
	IN	void					*CaContext, 
	IN	void					*CqContext
	);

void
SmaAsyncEventCallback(
	IN	void					*CaContext, 
	IN IB_EVENT_RECORD			*EventRecord
	);

void
SmaCreateCaMemIndex(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	OUT	uint32					*CaMemIndex
	);

void
SmaDeleteCaMemIndex( 
	IN	uint32					CaMemIndex 
	);

FSTATUS
SmaAddToCaMemTable(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	CA_MEM_LIST				*CaMemList,
	IN	uint32					CaMemIndex
	);


FSTATUS
SmaBindGlobalMem(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj
	);

FSTATUS
SmaRemoveCaObj(
	IN	SMA_CA_OBJ_PRIVATE		*pCaObj
	);


FSTATUS
SmaDeviceReserve(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj
	);

FSTATUS
SmaConfigQp(
	IN	IB_HANDLE				QpHandle,
	IN	IB_QP_STATE				QpState,
	IN	IB_QP_TYPE				QpType
	);

FSTATUS
SmaDeviceConfig(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	IB_QP_TYPE				QpType
	);

void
SetPortCapabilities ( void );


// tSM
FSTATUS TSmThreadStart( void );

void TSmThreadStop( void );

FSTATUS
SmaAllocToBin(
	IN uint32					NumMads,
	IN boolean					HoldingMutex
	);

FSTATUS
SmaReturnToBin(
	IN	POOL_DATA_BLOCK			*SmpBlockList
	);

void
SmaInitGlobalSettings(void);
	

FSTATUS
SmaGrabFromBin(
	IN OUT	uint32				*NumMad,
	IN OUT	SMP_BLOCK			*MadPool
	);

FSTATUS
SmaRemoveFromBin(void);


void
NotifyIbtCallback(
	IN	void		*Context
	);



#ifdef __cplusplus
}
#endif

#endif // _IBA_IB_SMA_SMAMAIN_H_
