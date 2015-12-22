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

#ifndef _IBA_IB_BMA_MAIN_H_
#define _IBA_IB_BMA_MAIN_H_


#include "datatypes.h"
#include "ib_types.h"
#include "ib_debug_osd.h"
#include "vpi_export.h"
#include "ib_gsi.h"
#include "ib_ibt.h"
#include "ib_mad.h"
#include "ib_generalServices.h"
#include "vca_export.h"
#include "statustext.h"
#include "ispinlock.h"
#include "ilist.h"
#include "icmdthread.h"
#include "ibyteswap.h"
#include "ib_bm.h"
#include "imemory.h"
#include "gsadebug.h"
#if defined(_DBG_LVL_PKTDUMP)
/* We want to use the _DBG_LVL_PKTDUMP for sma and not gsa */
#undef _DBG_LVL_PKTDUMP
#endif
#include "smadebug.h"
#include "gsamain.h"
#include "smamain.h"

#ifdef __cplusplus
extern "C" {
#endif

//
// Definitions
//

#define BMA_TAG	MAKE_MEM_TAG(B,m,a,M)

#define BMA_MAX_NDGRM 128

//
// Performance management data structures and definitions.
//

typedef void 
(*BMA_SEND_CALLBACK)(
	IN	void					*ServiceContext,
	IN	IBT_DGRM_ELEMENT		*DgrmList
	);

typedef FSTATUS 
(*BMA_RECEIVE_CALLBACK)(
	IN	void					*ServiceContext,
	IN	IBT_DGRM_ELEMENT		*DgrmList
	);

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
//	activeserviceclasslist is used to link the list of all registered service 
//	classes. Every time a unique service class is registered, a service class 
//	registration record ( GSA_SERVICE_CLASS_INFO ) is created and linked to
//	this list in a LIFO fashion.
//
//	activeserviceclasslistlock is used to guard access to the list of service 
//	classes ( activeserviceclasslist ).
//
//	uinumberofcas is the number of cas in the list ActiveCAList.
//	if binitsuccess is set to TRUE, it means that GSA was successful in 
//	initializing properly ( one-time initialization ). 
//	The interfaces of GSA MUST check this state before providing any 
//	useful functionality.
//	if FALSE, GSA services are not available.
//
//	ConfigParam contains the list of configuration parameters used to configure 
//	GSA. These values might be read from registry or from local configuration 
//	file(s) , depending on the platform we are running on.
//
//
typedef struct _BMA_GLOBAL_INFO
{

	void*							GsaHandle;		// Handle for Device Management Class service with GSA

    boolean							binitsuccess;

	IB_HANDLE						DgrmPoolHandle;

	CMD_THREAD						RecvThread;

	struct _BMA_DEV_INFO *			DevList;

	unsigned						NumCA;

	SPIN_LOCK						DevListLock;

} BMA_GLOBAL_INFO;

//
// Per-device info
//
typedef struct _BMA_DEV_INFO
{
	struct _BMA_DEV_INFO *			Next;

	struct _BMA_PORT_INFO *			Port;

	unsigned						NumPorts;

	EUI64							CaGuid;

} BMA_DEV_INFO;


//
// Per-port info
//
typedef struct _BMA_PORT_INFO
{
	EUI64							Guid;
	// trap settings
	struct {
		IB_GID		GID;
		uint32		TClass		:  8;
		uint32		SL			:  4;
		uint32		FlowLabel	: 20;
		uint16		LID;
		uint16		P_Key;
		uint32		HopLimit	:  8;
		uint32		QP			: 24;
		uint32		Q_Key;
	} Trap;
	uint64	BKey;
	boolean	BKeyProtect;
	uint16	BKeyViolations;
	uint16	BKeyLease;
} BMA_PORT_INFO;

//
// extern declaration for our global information structure
//
#ifndef g_BmaGlobalInfo
extern BMA_GLOBAL_INFO *g_BmaGlobalInfo;
#endif

typedef struct _BMA_GLOBAL_CONFIG_PARAMETERS
{
	uint32 MaxNDgrm;
} BMA_GLOBAL_CONFIG_PARAMETERS;

#define BMA_DEFAULT_SETTINGS { BMA_MAX_NDGRM }

#ifndef g_BmaSettings
extern BMA_GLOBAL_CONFIG_PARAMETERS g_BmaSettings;
#endif


//
// Function declarations
//
// bmamain.c
FSTATUS
BmaLoad(
	IN IBT_COMPONENT_INFO		*ComponentInfo
	);

void
BmaUnload(void);

FSTATUS
BmaAddDevice(
	IN	EUI64					CaGuid,
	OUT void					**Context
	);

FSTATUS
BmaRemoveDevice(
	IN EUI64					CaGuid,
	IN void						*Context
	);

// bmacallb.c

void
BmaSendCallback(
	void				*ServiceContext,
	IBT_DGRM_ELEMENT	*pDgrmList
	);

void
BmaFreeCallback(
	IN void* Context,
	IN void* Cmd
	);

void
BmaThreadCallback(
	IN void* Context,
	IN void* Cmd
	);

FSTATUS
BmaRecvCallback(
	void				*ServiceContext,
	IBT_DGRM_ELEMENT	*pDgrmList
	);

FSTATUS
BmaProcessDevMgmtGet(
	PDM_MAD				pMad,
	IBT_DGRM_ELEMENT	*pRecvDgrm,
	IBT_DGRM_ELEMENT	*pSendDgrm
	);

#ifdef __cplusplus
}
#endif

#endif	// _IBA_IB_BMA_MAIN_H_
