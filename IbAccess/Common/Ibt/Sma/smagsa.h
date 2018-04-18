/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/
/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_IB_SMA_GSA_H_
#define _IBA_IB_SMA_GSA_H_ (1) // suppress duplicate loading of this file




#include "datatypes.h"
#include "stl_types.h"
#include "stl_mad_priv.h"
#include "vpi.h"
#include "ieventthread.h"

#ifdef __cplusplus
extern "C" {
#endif

//
// These definitions will not be exposed to the SM

typedef	struct	_CA_MEM_LIST {
	//void		*Next;				//	Pointer to next mem list
	//void		*Prev;				// pointer to previous CA

	void		*VirtualAddr;
	size_t		Length;
	IB_HANDLE	MrHandle;
	IB_L_KEY	LKey;
	IB_R_KEY	RKey;
} CA_MEM_LIST;


typedef struct _CA_MEM_BLOCK {
	IB_HANDLE	MrHandle;
	IB_L_KEY	LKey;
//	IB_R_KEY	RKey;
} CA_MEM_BLOCK;

typedef	struct	_CA_MEM_TABLE {
	size_t		 Total;
	size_t		 Current;
	CA_MEM_BLOCK MemBlock[1];
} CA_MEM_TABLE;


//
// Global Memory map
typedef	struct	_GLOBAL_MEM_LIST {
	void		*Next;				// Pointer to next mem list
	void		*Prev;				// pointer to previous CA

	void		*VirtualAddr;		// Memory for Registered memory
	uintn		Length;
	void		*HdrAddr;			// Memory for block headers to 
									// hold registered mem 

	uint32		CaMemIndex;			// the holy memindex for all 
									// Mem handles, Lkeys
	IB_ACCESS_CONTROL AccessControl;// Access control flags
} GLOBAL_MEM_LIST;


typedef enum {
	QP_NORMAL = 0,
	QP_ERROR,
	QP_DESTROY
} GLOBAL_QP_CONTROL;

/* we use the low bit of the WorkReqId to indicate recv vs send */
#define ASSERT_VALID_WORKREQID(id)	ASSERT(! ((id) & 1));
#define BUILD_RQ_WORKREQID(id)	(id)
#define BUILD_SQ_WORKREQID(id)	((id) | (uintn)1)
#define IS_SQ_WORKREQID(id) ((id) & 1)
#define IS_RQ_WORKREQID(id) ( ! ((id) & 1))
#define MASK_WORKREQID(id) ((id) & ~(uintn)1)

typedef	struct	_SMA_PORT_BLOCK_PRIV {
	SMA_PORT_BLOCK		Public;
	
	// Sma
	IB_HANDLE			QpHandle;
	uint32				Qp0SendQDepth;	// Maximum outstanding WRs permitted
										// on the SendQ of the QP
	uint32				Qp0RecvQDepth;	// Maximum outstanding WRs permitted 
										// on the RecvQ
	ATOMIC_UINT			Qp0SendQTop;	// Must match SendQDepth when Empty
	ATOMIC_UINT			Qp0RecvQTop;	// Must match RecvQDepth when empty

	SPIN_LOCK			Qp0Lock;		// lock on Qp
	GLOBAL_QP_CONTROL	Qp0Control;		// GlobalControl on Qp states
	
	IB_HANDLE			AvHandle;		// Permissive Handle

	
	// Gsa
	IB_HANDLE			Qp1Handle;
	uint32				Qp1SendQDepth;	// Maximum outstanding WRs permitted
										// on the SendQ of the QP
	uint32				Qp1RecvQDepth;	// Maximum outstanding WRs permitted 
										// on the RecvQ
	uint32				Qp1RecvQPosted;	// present number of WRs posted on RecvQ
	SPIN_LOCK			QpLock;			// lock on Qp
	GLOBAL_QP_CONTROL	Qp1Control;		// GlobalControl on Qp states

	IB_LID				SMLID;			// SM LID
} SMA_PORT_BLOCK_PRIV;


//
// An array of ports based on number of ports
//

typedef struct _SMA_PORT_TABLE_PRIV {
	SMA_PORT_BLOCK_PRIV	PortBlock[1];
} SMA_PORT_TABLE_PRIV;



typedef struct _SMA_CA_OBJ_PRIVATE {
	struct	_SMA_CA_OBJ_PRIVATE *Prev;
	struct	_SMA_CA_OBJ_PRIVATE *Next;

	SMA_CA_OBJECT	CaPublic;

	// private
	IB_HANDLE		CaHandle;		// Handle to this CA returned by the VCA

	// Sma
	IB_HANDLE		CqHandle;
	uint32			CqSize;			// CQ depth
	IB_HANDLE		PdHandle;
	EVENT_THREAD	SmaEventThread;
	

	// Gsa
	IB_HANDLE		Qp1CqHandle;
	uint32			Qp1CqSize;		// CQ depth
	EVENT_THREAD	GsaEventThread;
	
	// common
	uint64			WorkReqId;		// SMA numbering to be returned 
									// on work completion
	
	CA_MEM_TABLE	*CaMemTbl;
	uint32			Partitions;		// Maximum number of partitions
	uint32			AddressVectors;	// Maximum number of Address Vectors 
									// which may be created

	uint32			UseCount;		// for objects on g_Sma->CaObj list, number
									// of active references

} SMA_CA_OBJ_PRIVATE;


// Forward declarations

FSTATUS
RegisterMemGlobal(
	IN  void					*VirtualAddress,
	IN  size_t					Length,
	IN  IB_ACCESS_CONTROL		AccessControl,
	OUT uint32					*CaMemIndex
	 );

FSTATUS
DeregisterMemGlobal(
	IN	uint32					CaMemIndex
	 );


GLOBAL_MEM_LIST *
CreateGlobalMemList(
	void						*VirtualAddr,		
	uintn						Length,
	void						*HdrAddr,
	boolean						HoldingMutex
	);

void
AcquireMemListMutex(void);

void
ReleaseMemListMutex(void);

FSTATUS
DeleteGlobalMemList(
	GLOBAL_MEM_LIST				*MemList		
	);

FSTATUS
GetAV(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	IB_ADDRESS_VECTOR		*AvInfo,
	OUT	IB_HANDLE				*AvHandle
	);

FSTATUS
PutAV(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	IB_HANDLE				AvHandle
	);

	
void
SetPortCapabilityMask (
	IN	uint8					Class,
	IN	boolean					Enabled
	);

FSTATUS
GetMemInfoFromCaMemTable(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	uint32					CaMemIndex,
	OUT	CA_MEM_LIST				*CaMemList
	);

FSTATUS
SmaCreateSmaObj( 
	IN	OUT	SMA_OBJECT			*SmaObj
	);


FSTATUS
DeleteFromCaMemTable(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	uint32					CaMemIndex
   );


void 
GsaCompletionThreadCallback(
	IN	void				*CaContext	// SMA_CA_OBJ_PRIVATE *
	);

void 
GsaCompletionCallback(
	IN	void				*CaContext,	// SMA_CA_OBJ_PRIVATE *
	IN	IB_HANDLE			CqHandle
	);


void
GsaUpdateCaList(void);

#ifdef __cplusplus
};
#endif

#endif // _IBA_IB_SMA_GSA_H_
