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

// Public header file
#include "ib_cm.h"

// Private header file
#include "cm_private.h"

//
// Debug setting
//
#ifdef ICS_LOGGING
//_IB_DBG_PARAM_BLOCK((_DBG_LVL_FATAL | _DBG_LVL_ERROR), _DBG_BREAK_ENABLE, CM_MEM_TAG, "Cm", MOD_CM, Cm);
_IB_DBG_PARAM_BLOCK(_DBG_LVL_ALL, _DBG_BREAK_ENABLE, CM_MEM_TAG, "Cm", MOD_CM, Cm);
#else
_IB_DBG_PARAM_BLOCK(_DBG_LVL_ALL, _DBG_BREAK_DISABLE, CM_MEM_TAG, "CM");
#endif


//
// CM setting (note on Linux Module parameters in CmLoad perform actual init
// of this structure)
//
CM_PARAMS	gCmParams={CM_MAX_NDGRM, \
						CM_DGRM_INITIAL, \
						CM_DGRM_LOW_WATER, \
						CM_DGRM_INCREMENT, \
						CM_MAX_CONNECTIONS, \
						CM_REQ_RETRY, \
						CM_REP_RETRY, \
						CM_MAX_BACKLOG, \
						SIDR_REQ_TIMEOUT_MS, \
						CM_TURNAROUND_TIME_MS, \
						CM_MIN_TURNAROUND_TIME_MS, \
						CM_MAX_TURNAROUND_TIME_MS, \
						CM_REUSE_QP, \
						CM_DISABLE_VALIDATION };

//
// Global CM context - one per system
//
CM_GLOBAL_CONTEXT* gCM=NULL;


//////////////////////////////////////////////////////////////////////////
// CmInit
//
// This routine is called once during system initialization to initialize
// the CM's global context.
//
// INPUTS:
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - Indicates a successful initialization.
//	FERROR
//	FINSUFFICIENT_RESOURCES
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//

FSTATUS
CmInit(
	IN IBT_COMPONENT_INFO *ComponentInfo
	)
{
	FSTATUS Status=FSUCCESS;
	uint32 BufSize= sizeof(MAD);

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CmInit);

	_TRC_REGISTER();

	_DBG_INFO(("SilverStorm Technologies Inc. InfiniBand Communication Manager module... <Built %s %s Version %s>\n", _DBG_PTR(__DATE__), _DBG_PTR(__TIME__), _DBG_PTR(BLD_VERSION_STRING)));

	// Verify data structure sizes of msgs on the wire
	if (sizeof(CM_MAD) > MAD_BLOCK_SIZE )
	{
		_DBG_ERROR(("Invalid data structure sizes!!!\n"));
		_DBG_ERROR(("CMM_REQ = %d\n", (int)sizeof(CMM_REQ)));
		_DBG_ERROR(("CMM_REP = %d\n", (int)sizeof(CMM_REP)));
		_DBG_ERROR(("CMM_REJ = %d\n", (int)sizeof(CMM_REJ)));
		_DBG_ERROR(("CMM_RTU = %d\n", (int)sizeof(CMM_RTU)));
		_DBG_ERROR(("CMM_MRA = %d\n", (int)sizeof(CMM_MRA)));
		_DBG_ERROR(("CMM_DREQ = %d\n", (int)sizeof(CMM_DREQ)));
		_DBG_ERROR(("CMM_DREP = %d\n", (int)sizeof(CMM_DREP)));
		_DBG_ERROR(("CMM_SIDR_REQ = %d\n", (int)sizeof(CMM_SIDR_REQ)));
		_DBG_ERROR(("CMM_SIDR_RESP = %d\n", (int)sizeof(CMM_SIDR_RESP)));
		_DBG_ERROR(("CMM_LAP = %d\n", (int)sizeof(CMM_LAP)));
		_DBG_ERROR(("CMM_APR = %d\n", (int)sizeof(CMM_APR)));
		_DBG_ERROR(("CMM_MSG = %d\n", (int)sizeof(CMM_MSG)));
		_DBG_ERROR(("CM_MAD = %d\n", (int)sizeof(CM_MAD)));

		return FERROR;
	}

	if (gCM)
	{
		_DBG_ERROR(("<cm 0x%p> Communication Manager module already initialized!!!\n", _DBG_PTR(gCM)));

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FERROR;
	}

	// Allocate the memory for the global context
	gCM = (CM_GLOBAL_CONTEXT*)MemoryAllocate2AndClear(sizeof(CM_GLOBAL_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, CM_MEM_TAG);
	if (!gCM)
	{
		_DBG_ERROR(("Unable to allocate memory for cm global context!!!\n"));
		Status = FINSUFFICIENT_MEMORY;
		goto failalloc;
	}
	
	// Initialize the global CM context

	// Set the comm ID to a pseudo-random number to prevent previous msg
	// interpreted as valid msg.
	gCM->CommID = (uint32)GetTimeStamp();

	// set the StartPSN to a pseudo-random 24 bit number
	gCM->StartPSN = (uint32)GetTimeStamp() & 0xffffff;

	gCM->Signature = GetTimeStampSec();//gCM->CommID;

	gCM->turnaround_time_us = MAX((uint64)gCmParams.TurnaroundTimeMs*1000,
								(uint64)gCmParams.MinTurnaroundTimeMs*1000);

	// TBD error checks for Init routines below
	EventInitState(&gCM->EventObj);
	EventInit(&gCM->EventObj);
	EventInitState(&gCM->CallbackEventObj);
	EventInit(&gCM->CallbackEventObj);

	SpinLockInitState(&gCM->ListLock);
	SpinLockInit(&gCM->ListLock);
	SpinLockInitState(&gCM->DeviceListLock);
	SpinLockInit(&gCM->DeviceListLock);

	QListInitState(&gCM->AllCEPs);
	QListInit(&gCM->AllCEPs);
	DListInit(&gCM->DeviceList);

	// keyed lists
	CM_MapInit(&gCM->TimerMap, NULL);
	CM_MapInit(&gCM->LocalCommIDMap, NULL);
	CM_MapInit(&gCM->LocalEndPointMap, CepLocalEndPointCompare);
	CM_MapInit(&gCM->RemoteEndPointMap, CepRemoteEndPointCompare);

	// State based lists
	CM_MapInit(&gCM->QueryMap, NULL);
	CM_MapInit(&gCM->ListenMap, CepListenAddrCompare);

	QListInitState(&gCM->ApcRetryList);
	QListInit(&gCM->ApcRetryList);
	TimerInitState(&gCM->ApcRetryTimer);
	Status = TimerInit(&gCM->ApcRetryTimer, ApcRetryTimerCallback, NULL );
	if (!Status)
	{
		_DBG_ERROR(("!!!TimerInit() failed!!! <%s>\n", _DBG_PTR(FSTATUS_MSG(Status))));
		goto failtimer;
	}

	// Register with GSA
	Status = iba_gsi_register_class(MCLASS_COMM_MGT, 
								IB_COMM_MGT_CLASS_VERSION,
								GSI_REGISTER_RESPONDER,	// Register as a Responder
								FALSE,			// No SAR cap needed
								gCM,			// Context
								GsaSendCompleteCallback,
								GsaRecvCallback,
								&gCM->GSAHandle);
							
	if (Status != FSUCCESS)
	{
		_DBG_ERROR(("<cm 0x%p> Unable to register with GSA!!! <%s>\n",
						_DBG_PTR(gCM), _DBG_PTR(FSTATUS_MSG(Status))));
		goto failregister;
	}

	// Create the datagram pool
	Status = iba_gsi_create_growable_dgrm_pool(gCM->GSAHandle,
								gCmParams.PreallocDgrms,
								1,
								&BufSize,
								0,
								&gCM->GSADgrmPoolHandle);
	if (Status != FSUCCESS)
	{
		_DBG_ERROR(("<cm 0x%p> Unable to create datagram pool!!! <%s>\n",
						_DBG_PTR(gCM), _DBG_PTR(FSTATUS_MSG(Status))));
		goto failpool;
	}

	//
	// Initialize system callback facility
	//

	// If everything goes well, setup the minimum entry pts
	ComponentInfo->AddDevice	= CmDeviceInit;
	ComponentInfo->RemoveDevice = CmDeviceDestroy;
	ComponentInfo->Unload		= CmDestroy;

	_DBG_INFO(("<cm 0x%p> *** CM module setting ***\n", _DBG_PTR(gCM)));

	_DBG_INFO(("\t\tClass Id			= %d\n", MCLASS_COMM_MGT));
	_DBG_INFO(("\t\tClass Version		= %d\n", IB_COMM_MGT_CLASS_VERSION));

	_DBG_INFO(("\t\tInitial lcid		= 0x%x\n", gCM->CommID));
	_DBG_INFO(("\t\tInitial PSN			= 0x%x\n", gCM->StartPSN));
	_DBG_INFO(("\t\tSignature			= %u\n", gCM->Signature));
	_DBG_INFO(("\t\tCEP size			= %u\n", (unsigned)sizeof(CM_CEP_OBJECT)));

	_DBG_INFO(("\t\tMaxDgrms			= %u\n", gCmParams.MaxDgrms));
	_DBG_INFO(("\t\tPreallocDgrms		= %u\n", gCmParams.PreallocDgrms));
	_DBG_INFO(("\t\tMinFreeDgrms		= %u\n", gCmParams.MinFreeDgrms));
	_DBG_INFO(("\t\tDgrmIncrement		= %u\n", gCmParams.DgrmIncrement));
	_DBG_INFO(("\t\tMaxConns			= %u\n", gCmParams.MaxConns));
	_DBG_INFO(("\t\tMaxReqRetry			= %u\n", gCmParams.MaxReqRetry));
	_DBG_INFO(("\t\tMaxRepRetry			= %u\n", gCmParams.MaxRepRetry));
	_DBG_INFO(("\t\tMaxBackLog			= %u\n", gCmParams.MaxBackLog));
	_DBG_INFO(("\t\tSidrReqTimeoutMs	= %u\n", gCmParams.SidrReqTimeoutMs));
	_DBG_INFO(("\t\tTurnaroundTimeMs	= %u\n", gCmParams.TurnaroundTimeMs));
	_DBG_INFO(("\t\tMinTurnaroundTimeMs	= %u\n", gCmParams.MinTurnaroundTimeMs));
	_DBG_INFO(("\t\tMaxTurnaroundTimeMs	= %u\n", gCmParams.MaxTurnaroundTimeMs));
	_DBG_INFO(("\t\tReuseQP				= %u\n", gCmParams.ReuseQP));

	_DBG_INFO(("<cm 0x%p> *** CM module initialized successfully ***\n", _DBG_PTR(gCM)));

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

    return FSUCCESS;

failpool:
	iba_gsi_deregister_class(gCM->GSAHandle);
failregister:
	TimerDestroy(&gCM->ApcRetryTimer);
failtimer:
	SpinLockDestroy(&gCM->DeviceListLock);
	SpinLockDestroy(&gCM->ListLock);
	EventDestroy(&gCM->CallbackEventObj);
	EventDestroy(&gCM->EventObj);
	MemoryDeallocate(gCM);
failalloc:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // CmInit()


//////////////////////////////////////////////////////////////////////////
// CmDestroy()
//
// This routine is called once during system destruction to release any resources
// used by CM.
//
// INPUTS:
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - Indicates a successful initialization.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//

void
CmDestroy(
	)
{
	LIST_ITEM* pListItem=NULL;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CmDestroy);

	// If there are oustanding callbacks, wait till those callbacks return
	if (AtomicRead(&gCM->TotalCallbackRefCnt))
	{
		_DBG_WARNING(("<cm 0x%p> *** There are still outstanding callbacks in the queue <ref cnt %d> ***\n", _DBG_PTR(gCM), AtomicRead(&gCM->TotalCallbackRefCnt)));

		EventWaitOn(&gCM->CallbackEventObj, EVENT_NO_TIMEOUT);
	}

	// Cleanup and release the resources associated with the CEP objects
	SpinLockAcquire(&gCM->ListLock);

	while ((pListItem = QListHead(&gCM->AllCEPs)) != NULL)
	{
		CM_CEP_OBJECT* pCEP= (CM_CEP_OBJECT*)QListObj(pListItem);
		_DBG_WARNING(("<cep 0x%p, %s> *** Cleaning up...***\n", _DBG_PTR(pCEP),
						_DBG_PTR(CmGetStateString(pCEP->State))));

		switch (pCEP->State)
		{
			case CMS_IDLE:
				break;

				// Listening
			case CMS_LISTEN:
			case CMS_REGISTERED:
				ASSERT(! pCEP->bPeer);
				while (!DListIsEmpty(&pCEP->PendingList))
				{
					CM_CEP_OBJECT* pCEP1=NULL;
					DLIST_ENTRY* pListEntry=NULL;

					pListEntry = DListGetNext(&pCEP->PendingList);
					pCEP1 = PARENT_STRUCT(pListEntry, CM_CEP_OBJECT, PendingListEntry);
					_DBG_WARNING(("<cep 0x%p, %s> *** Cleaning up...***\n",
									_DBG_PTR(pCEP1), _DBG_PTR(CmGetStateString(pCEP1->State))));
					ASSERT(pCEP1->pParentCEP == pCEP);
					CepRemoveFromPendingList(pCEP1);
					CepToIdle(pCEP1);
					DestroyCEP(pCEP1);
				}
				ASSERT(pCEP->PendingCount == 0);
				CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
				break;

				// Pending Connection
			case CMS_REQ_RCVD:	// passive
			case CMS_MRA_REQ_SENT:	// passive
				CepRemoveFromPendingList(pCEP);
				break;

				// Connecting
			case CMS_REQ_SENT:	// active
			case CMS_REP_WAIT:	// active
				if (pCEP->bPeer && pCEP->RemoteEndPoint.CaGUID == 0)
				{
					CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
				}
				break;

			case CMS_REP_SENT:	// passive
			case CMS_MRA_REP_RCVD:	// passive
			case CMS_REP_RCVD:	// active
			case CMS_MRA_REP_SENT:	// active
				// connected
			case CMS_ESTABLISHED:
			case CMS_LAP_SENT:
			case CMS_MRA_LAP_RCVD:
			case CMS_LAP_RCVD:
			case CMS_MRA_LAP_SENT:
				// disconnecting
			case CMS_DREQ_SENT:
			case CMS_DREQ_RCVD:
			case CMS_SIM_DREQ_RCVD:
			case CMS_TIMEWAIT:
				break;

				// SIDR query in progress
			case CMS_SIDR_REQ_SENT:
				CM_MapRemoveEntry(&gCM->QueryMap, &pCEP->MapEntry, QUERY_LIST, pCEP);
				break;

				// SIDR response in progress
			case CMS_SIDR_REQ_RCVD:
				CepRemoveFromPendingList(pCEP);
				break;
		}
		if (pCEP->State != CMS_IDLE)
			CepToIdle(pCEP);
		DestroyCEP(pCEP);
	}

	ASSERT(QListIsEmpty(&gCM->AllCEPs));
	ASSERT(CM_MapIsEmpty(&gCM->TimerMap));
	ASSERT(CM_MapIsEmpty(&gCM->LocalCommIDMap));
	ASSERT(CM_MapIsEmpty(&gCM->LocalEndPointMap));
	ASSERT(CM_MapIsEmpty(&gCM->RemoteEndPointMap));
	ASSERT(CM_MapIsEmpty(&gCM->QueryMap));
	ASSERT(CM_MapIsEmpty(&gCM->ListenMap));
	ASSERT(QListIsEmpty(&gCM->ApcRetryList));

	// Invalidate the signature so that user cannot use any cep handles it already has. 
	gCM->Signature = 0;

	SpinLockRelease(&gCM->ListLock);

	_TRC_UNREGISTER();

	// Destroy the datagram pool
	if (gCM->GSADgrmPoolHandle)
		iba_gsi_destroy_dgrm_pool(gCM->GSADgrmPoolHandle);

	_DBG_INFO(("<cm 0x%p> *** Deregister from GSA <gsa handle 0x%p> ***\n",
			_DBG_PTR(gCM), _DBG_PTR(gCM->GSAHandle)));

	iba_gsi_deregister_class(gCM->GSAHandle);

	TimerDestroy(&gCM->ApcRetryTimer);
	QListDestroy(&gCM->ApcRetryList);

	QListDestroy(&gCM->AllCEPs);

	SpinLockDestroy(&gCM->ListLock);
	SpinLockDestroy(&gCM->DeviceListLock);

	EventDestroy(&gCM->EventObj);
	EventDestroy(&gCM->CallbackEventObj);

	_DBG_INFO(("<cm 0x%p> *** Free the cm context ***\n", _DBG_PTR(gCM)));

	MemoryDeallocate(gCM);

	gCM = NULL;

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return;

} // CmDestroy()


//////////////////////////////////////////////////////////////////////////
// CmDeviceInit()
//
// This routine is called once for every new device added.
//
// INPUTS:
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - Indicates a successful initialization.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
CmDeviceInit(
	IN	EUI64			CaGuid,
	OUT void			**Context
	)
{
	CM_DEVICE_OBJECT* pDevice=NULL;
	FSTATUS	Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CmDeviceInit);

	//_DBG_BREAK;

	// Allocate the memory for the device context
	pDevice = (CM_DEVICE_OBJECT*)MemoryAllocateAndClear(sizeof(CM_DEVICE_OBJECT), FALSE, CM_MEM_TAG);
	if (!pDevice)
	{
		_DBG_ERROR(("<cm 0x%p> Unable to allocate memory for device context!!!\n", _DBG_PTR(gCM)));

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FINSUFFICIENT_MEMORY;
	}

	// TODO:
	// pDevice->pDeviceContext = ;

	pDevice->CaGUID = CaGuid;

#ifdef _CM_EVENT_ON_QP_

	Status = iba_open_ca( CaGuid, VpdCompletionCallback, VpdAsyncEventCallback,
					pDevice,							// Context
					&pDevice->CaHandle);
	if (Status != FSUCCESS)
	{

		_DBG_ERROR(("<cm 0x%p> Unable to create datagram pool!!! <%s>\n",
						_DBG_PTR(gCM), _DBG_PTR(FSTATUS_MSG(Status))));
		goto failopen;
	}
	Status = SetCAInternal(pDevice->CaHandle);
	if (FSUCCESS != Status)
	{
		_DBG_ERROR(("<cm 0x%p> Unable to set CA as internal!!! <%s>\n",
						_DBG_PTR(gCM), _DBG_PTR(FSTATUS_MSG(Status))));
		goto failinternal;
	}

	iba_register_cm(pDevice->CaHandle);

	_DBG_INFO(("<dev 0x%p> +++ Opening CA +++ <hCa 0x%p>\n", _DBG_PTR(pDevice), _DBG_PTR(pDevice->CaHandle)));

#endif

	// Put it on the device list
	SpinLockAcquire(&gCM->DeviceListLock);

	DListInsertTail(&gCM->DeviceList, &pDevice->DeviceListEntry, DEVICE_LIST, pDevice);

	SpinLockRelease(&gCM->DeviceListLock);

	*Context = (void*)pDevice;

	_DBG_INFO(("<dev 0x%p> +++ Device Added +++ <dev guid 0x%"PRIx64">\n", _DBG_PTR(pDevice), CaGuid));

done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

failinternal:
	(void)iba_close_ca(pDevice->CaHandle);
failopen:
	MemoryDeallocate(pDevice);
	goto done;
} // CmDeviceInit()


//////////////////////////////////////////////////////////////////////////
// CmDeviceDestroy()
//
// This routine is called once for every device removed.
//
// INPUTS:
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - Indicates a successful initialization.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
CmDeviceDestroy(
	IN	EUI64			CaGuid,
	IN void				*Context
	)
{
	CM_DEVICE_OBJECT* pDevice=(CM_DEVICE_OBJECT*)Context;
	// keep this const and off stack to save kernel stack space
	static CM_DREQUEST_INFO	DisconnectReq = {{0}};
	LIST_ITEM* pListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CmDeviceDestroy);

	if (!pDevice)
	{
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FERROR;
	}

	SpinLockAcquire(&gCM->ListLock);

	for (pListItem = QListHead(&gCM->AllCEPs); pListItem != NULL;)
	{
		CM_CEP_OBJECT* pCEP= (CM_CEP_OBJECT*)QListObj(pListItem);
		pListItem = QListNext(&gCM->AllCEPs, pListItem);	// in case we delete
		if (pCEP->LocalEndPoint.CaGUID != pDevice->CaGUID)
			continue;	// skip, not on this device

		_DBG_INFO(("<cep 0x%p, %s> *** CA Device Removed ***\n",
					_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State))));

		// BUGBUG - these need to be reviewed and adjusted.  Many of these
		// would surprise an application.  For example SYSE_DISCONNECT
		// would cause a FCM_DISCONNECTED without a FCM_DISCONNECT_REQUEST
		// so application would skip moving QP to Error.
		// May be better to do these operations, move CEP to Idle
		// and always set SYSE_DISCONNECT, then have SYSE_DISCONNECT return a
		// unique error code.
		switch (pCEP->State)
		{
				// not expected to have a LocalEndPoint.CaGUID assigned
			case CMS_IDLE:
			case CMS_REGISTERED:
				ASSERT(0);
				break;

				// Listening
			case CMS_LISTEN:
				ASSERT(! pCEP->bPeer);
				ASSERT(pCEP->Mode == CM_MODE_PASSIVE);
				(void)CancelP(pCEP);
				// TBD SetNotification(pCEP, SYSE_DISCONNECT);
				break;

				// Connecting
			case CMS_REQ_SENT:	// active
			case CMS_REP_WAIT:	// active
			case CMS_REQ_RCVD:	// passive
			case CMS_MRA_REQ_SENT:	// passive
			case CMS_REP_SENT:	// passive
			case CMS_MRA_REP_RCVD:	// passive
			case CMS_REP_RCVD:	// active
			case CMS_MRA_REP_SENT:	// active
				// BUGBUG - for CMS_REQ_RCVD/MRA_REQ_SENT, user has been notified,
				// they will callback against 1st in pending list
				// which could be this one
				if (pCEP->Mode == CM_MODE_PASSIVE)
					(void)CancelP(pCEP);
				else
					(void)CancelA(pCEP);
				// TBD SetNotification(pCEP, SYSE_DISCONNECT);
				break;

				// connected
			case CMS_ESTABLISHED:
			case CMS_LAP_SENT:
			case CMS_MRA_LAP_RCVD:
			case CMS_LAP_RCVD:
			case CMS_MRA_LAP_SENT:
				(void)DisconnectA(pCEP, &DisconnectReq, NULL);
				// TBD move to pCEP to Idle
				// TBD SetNotification(pCEP, SYSE_DISCONNECT);
				break;

				// disconnecting
			case CMS_DREQ_SENT:
			case CMS_DREQ_RCVD:
			case CMS_SIM_DREQ_RCVD:
			case CMS_TIMEWAIT:
				(void)ShutdownA(pCEP);
				break;

				// SIDR query in progress
			case CMS_SIDR_REQ_SENT:
				(void)CancelA(pCEP);
				// TBD SetNotification(pCEP, CME_TIMEOUT_SIDR_REQ);
				break;

				// SIDR response in progress
			case CMS_SIDR_REQ_RCVD:
				// TBD - user has been notified, they will callback
				// against 1st in pending list which could be this one
				(void)CancelP(pCEP);
				break;
		}
	}

	SpinLockRelease(&gCM->ListLock);

	// Remove the device obj
	SpinLockAcquire(&gCM->DeviceListLock);

	DListRemoveEntry(&pDevice->DeviceListEntry, DEVICE_LIST, pDevice);

	if (DListIsEmpty(&gCM->DeviceList))
	{
		_DBG_INFO(("<dev 0x%p> --- Removing last device --- <dev guid 0x%"PRIx64">\n", _DBG_PTR(pDevice), CaGuid));
	}

	SpinLockRelease(&gCM->DeviceListLock);

#ifdef _CM_EVENT_ON_QP_

	iba_close_ca( pDevice->CaHandle );

	_DBG_INFO(("<dev 0x%p> --- Closing CA --- <hCa 0x%p>\n", _DBG_PTR(pDevice), _DBG_PTR(pDevice->CaHandle)));

#endif

	MemoryDeallocate(pDevice);

	_DBG_INFO(("<dev 0x%p> --- Device Removed --- <dev guid 0x%"PRIx64">\n", _DBG_PTR(pDevice), CaGuid));

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return FSUCCESS;

} // CmDeviceDestroy()


//////////////////////////////////////////////////////////////////////////
// iba_cm_create_cep
//
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
IB_HANDLE
iba_cm_create_cep(
	CM_CEP_TYPE	TransportServiceType
	)
{
	CM_CEP_OBJECT*		pCEP=NULL;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_create_cep);

	if (!gCM)
	{
		_DBG_ERROR(("CM Context not initialized!!!\n"));
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
		return NULL;
	}

	SpinLockAcquire(&gCM->ListLock);

	pCEP = CreateCEP(TransportServiceType, NULL, NULL);

	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return (IB_HANDLE)pCEP;

} // iba_cm_create_cep()


IB_HANDLE
CmCreateCEPEx(
	CM_CEP_TYPE		TransportServiceType,
	EVENT_HANDLE	hEvent,
	EVENT_HANDLE	hWaitEvent
	)
{
	CM_CEP_OBJECT*		pCEP=NULL;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CmCreateCEPEx);

	if (!gCM)
	{
		_DBG_ERROR(("CM Context not initialized!!!\n"));
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
		return NULL;
	}

	SpinLockAcquire(&gCM->ListLock);

	pCEP = CreateCEP(TransportServiceType, hEvent, hWaitEvent);

	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return (IB_HANDLE)pCEP;

} // CmCreateCEPEx()


//////////////////////////////////////////////////////////////////////////
// iba_cm_destroy_cep
//
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_destroy_cep(IB_HANDLE hCEP)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)hCEP;
	DLIST_ENTRY* pListEntry=NULL;
	CM_CEP_OBJECT* pCEPEntry=NULL;
	FSTATUS Status=FSUCCESS;

	// keep this const and off stack to save kernel stack space
	static CM_REJECT_INFO	RejectInfo = {
		Reason:RC_USER_REJ,
		RejectInfoLen:0,
	};
	static CM_DREQUEST_INFO	DisconnectReq = {{0}};

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_destroy_cep);

	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle() failed!!! <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return Status;
	}

	SpinLockAcquire(&gCM->ListLock);

	switch (pCEP->State)
	{

	case CMS_IDLE:

		// If a callback is in progress, we cannot destroy. Mark it and defer it to the callback
		if (AtomicRead(&pCEP->CallbackRefCnt))
		{
			BitSet(pCEP->EventFlags, USRE_DESTROY);
		} else {
			DestroyCEP(pCEP);
		}

		break;

	case CMS_REQ_SENT:
	case CMS_REP_WAIT:

		// Cancel the request and destroy the obj.
		CancelA(pCEP);

		if (AtomicRead(&pCEP->CallbackRefCnt))
		{
			BitSet(pCEP->EventFlags, USRE_DESTROY);
		} else {
			DestroyCEP(pCEP);
		}

		break;

	case CMS_REP_RCVD:
	case CMS_MRA_REP_SENT:
		// Reject the reply and destroy the obj
		RejectA(pCEP, &RejectInfo);

		// If a callback is in progress, we cannot destroy. Mark it and defer it to the callback
		if (AtomicRead(&pCEP->CallbackRefCnt))
		{
			BitSet(pCEP->EventFlags, USRE_DESTROY);
		} else {
			DestroyCEP(pCEP);
		}

		break;

	case CMS_LISTEN:
	case CMS_REQ_RCVD:
	case CMS_MRA_REQ_SENT:

		CancelP(pCEP);

		// If a callback is in progress, we cannot destroy. Mark it and defer it to the callback
		if (AtomicRead(&pCEP->CallbackRefCnt))
		{
			BitSet(pCEP->EventFlags, USRE_DESTROY);
		} else {
			DestroyCEP(pCEP);
		}

		break;

	case CMS_REP_SENT:
	case CMS_MRA_REP_RCVD:

		CancelP(pCEP);

		// If a callback is in progress, we cannot destroy. Mark it and defer it to the callback
		if (AtomicRead(&pCEP->CallbackRefCnt))
		{
			BitSet(pCEP->EventFlags, USRE_DESTROY);
		} else {
			DestroyCEP(pCEP);
		}

		break;

	case CMS_ESTABLISHED:
	case CMS_LAP_SENT:
	case CMS_MRA_LAP_RCVD:
	case CMS_LAP_RCVD:
	case CMS_MRA_LAP_SENT:
	case CMS_DREQ_RCVD:
	case CMS_SIM_DREQ_RCVD:

		// Let the CEP state determine if we are sending a DREQ or DREP
		DisconnectA(pCEP, &DisconnectReq, (CM_DREPLY_INFO*)&DisconnectReq);

		_DBG_INFO(("<cep 0x%p> *** CEP object marked for destruction *** <lcid 0x%x rcid 0x%x %s>.\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID,
					_DBG_PTR(CmGetStateString(pCEP->State))));

		BitSet(pCEP->EventFlags, USRE_DESTROY);

		break;

	case CMS_DREQ_SENT:
	case CMS_TIMEWAIT:
		//
		// We cant destroy the object until timewait expires.
		//
		_DBG_INFO(("<cep 0x%p> *** CEP object marked for destruction *** <lcid 0x%x rcid 0x%x %s>.\n", 
					_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID,
					_DBG_PTR(CmGetStateString(pCEP->State))));

		BitSet(pCEP->EventFlags, USRE_DESTROY);

		break;

	case CMS_SIDR_REQ_SENT:
		//
		// Cancel the sidr request
		//

		// Cancel the SIDR_REQ timer
		CmTimerStop(pCEP, SIDR_REQ_TIMER);

		// Move from QUERY to INIT list
		CM_MapRemoveEntry(&gCM->QueryMap, &pCEP->MapEntry, QUERY_LIST, pCEP);
		// ClearEndPoint in CepToIdle is benign
		CepToIdle(pCEP);

		// Set the cancel event
		// TBD this sets CallbackRefCnt and forces destroy to occur
		// in callback path below
		SetNotification(pCEP, USRE_CANCEL);

		// If a callback is in progress, we cannot destroy. Mark it and defer it to the callback
		if (AtomicRead(&pCEP->CallbackRefCnt))
		{
			BitSet(pCEP->EventFlags, USRE_DESTROY);
		} else {
			DestroyCEP(pCEP);
		}

		break;

		/*
	case CMS_SIDR_RESP_RCVD:

		// ClearEndPoint in CepToIdle is benign
		CepToIdle(pCEP);

		// If a callback is in progress, we cannot destroy. Mark it and defer it to the callback
		if (AtomicRead(&pCEP->CallbackRefCnt))
		{
			BitSet(pCEP->EventFlags, USRE_DESTROY);
		} else {
			DestroyCEP(pCEP);
		}

		break;
		*/

	case CMS_REGISTERED:
		// Clear the pending list
		while (!DListIsEmpty(&pCEP->PendingList))
		{
			pListEntry = DListGetNext(&pCEP->PendingList);
			pCEPEntry = PARENT_STRUCT(pListEntry, CM_CEP_OBJECT, PendingListEntry);

			// Move from PENDING to INIT list and destroy it
			ASSERT(pCEPEntry->pParentCEP == pCEP);
			CepRemoveFromPendingList(pCEPEntry);
			// ClearEndPoint in CepToIdle is benign
			CepToIdle(pCEPEntry);

			// This will start the timer in timewait state
			DestroyCEP(pCEPEntry);
		}
		ASSERT(pCEP->PendingCount == 0);

		// Move from LISTEN to IDLE
		CM_MapRemoveEntry(&gCM->ListenMap, &pCEP->MapEntry, LISTEN_LIST, pCEP);
		// ClearEndPoint in CepToIdle is benign
		CepToIdle(pCEP);

		// Set the cancel event
		// TBD this sets CallbackRefCnt and forces destroy to occur
		// in callback path below
		SetNotification(pCEP, USRE_CANCEL);

		// If a callback is in progress, we cannot destroy.
		// Mark it and the callback will destroy it
		if (AtomicRead(&pCEP->CallbackRefCnt))
		{
			BitSet(pCEP->EventFlags, USRE_DESTROY);
		} else {
			DestroyCEP(pCEP);
		}

		break;

	default:

		_DBG_WARNING(("<cep 0x%p> - Invalid state to destroy cep!!", _DBG_PTR(pCEP)));
		Status = FINVALID_STATE;

		break;

	} // switch()

	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // iba_cm_destroy_cep()


//////////////////////////////////////////////////////////////////////////
// iba_cm_modify_cep
//
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_modify_cep(
	IB_HANDLE		hCEP,
	uint32			AttrType,
	const char*		AttrValue,
	uint32			AttrLen,
	uint32			Offset
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)hCEP;
	FSTATUS Status=FSUCCESS;
	uint32		Value=0;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_modify_cep);

	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle() failed!!! <%s>\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		goto done;
	}

	SpinLockAcquire(&gCM->ListLock);

	if (AttrType == CM_FLAG_LISTEN_NODISCRIM)
	{
		_DBG_INFO(("<cep 0x%p> Modifying CEP to listen with no discriminator parsing.\n", _DBG_PTR(pCEP)));
		// must set before start listening
		if (pCEP->State != CMS_IDLE)
		{
			Status = FINVALID_STATE;
			goto unlock;
		}
		pCEP->DiscriminatorLen = 0;
		pCEP->DiscrimPrivateDataOffset = 0;
		if (pCEP->Discriminator)
		{
			MemoryDeallocate(pCEP->Discriminator);
			pCEP->Discriminator = NULL;
		}
	} else if (AttrType == CM_FLAG_LISTEN_DISCRIM) {
		uint8 *p;
		// must set before start listening
		if (pCEP->State != CMS_IDLE)
		{
			Status = FINVALID_STATE;
			goto unlock;
		}
		if ((AttrLen + Offset) > CM_MAX_DISCRIMINATOR_LEN)
		{
			_DBG_ERROR(("<cep 0x%p> - Invalid discriminator length and offset!!! <len %d offset %d>\n",
						_DBG_PTR(pCEP), AttrLen, Offset));
			Status = FINVALID_PARAMETER;
			goto unlock;
		}
		p = (uint8*)MemoryAllocate(AttrLen, FALSE, CM_MEM_TAG);
		if (p == NULL)
		{
			_DBG_ERROR(("<cep 0x%p> - Unable to allocate discriminator!!! <len %d>\n",
						_DBG_PTR(pCEP), AttrLen));
			Status = FINSUFFICIENT_MEMORY;
			goto unlock;
		}
		_DBG_INFO(("<cep 0x%p> Modifying CEP to listen with discriminator parsing <len %d offset %d>.\n", 
					_DBG_PTR(pCEP), AttrLen, Offset));
		pCEP->DiscrimPrivateDataOffset = (uint8)Offset;
		pCEP->DiscriminatorLen = (uint8)AttrLen;
		if (pCEP->Discriminator)
			MemoryDeallocate(pCEP->Discriminator);
		pCEP->Discriminator = p;
		MemoryCopy(pCEP->Discriminator, AttrValue, AttrLen); 
	} else if (AttrType == CM_FLAG_LISTEN_BACKLOG) {
		if (AttrLen != sizeof(uint32))
		{
			Status = FINVALID_PARAMETER;
			goto unlock;
		}
		MemoryCopy(&pCEP->ListenBackLog, AttrValue, AttrLen);
		_DBG_INFO(("<cep 0x%p> Modifying CEP to new backlog count <backlog %d>.\n", _DBG_PTR(pCEP), pCEP->ListenBackLog));
	} else if (AttrType == CM_FLAG_SIDR_REGISTER_NOTIFY) {
		if (AttrLen != sizeof(uint32))
		{
			Status = FINVALID_PARAMETER;
			goto unlock;
		}
		MemoryCopy(&Value, AttrValue, AttrLen);
		pCEP->bSidrRegisterNotify = (boolean) Value;
		_DBG_INFO(("<cep 0x%p> Modifying CEP for Sidr Register notification <bSidrRegisterNotify %d>.\n", _DBG_PTR(pCEP), pCEP->bSidrRegisterNotify));
	} else if (AttrType == CM_FLAG_ASYNC_ACCEPT) {
		if (AttrLen != sizeof(uint32))
		{
			Status = FINVALID_PARAMETER;
			goto unlock;
		}
		MemoryCopy(&Value, AttrValue, AttrLen);
		pCEP->bAsyncAccept = (boolean) Value;
		_DBG_INFO(("<cep 0x%p> Modifying CEP for Async Accept <bAsyncAccept %d>.\n", _DBG_PTR(pCEP), pCEP->bAsyncAccept));
	} else if (AttrType == CM_FLAG_TIMEWAIT_CALLBACK) {
		if (AttrLen != sizeof(uint32))
		{
			Status = FINVALID_PARAMETER;
			goto unlock;
		}
		MemoryCopy(&Value, AttrValue, AttrLen);
		pCEP->bTimewaitCallback = (boolean) Value;
		_DBG_INFO(("<cep 0x%p> Modifying CEP for Timewait Callback <bTimewaitCallback %d>.\n", _DBG_PTR(pCEP), pCEP->bTimewaitCallback));
	} else if (AttrType == CM_FLAG_TURNAROUND_TIME) {
		if (AttrLen != sizeof(uint32))
		{
			Status = FINVALID_PARAMETER;
			goto unlock;
		}
		MemoryCopy(&Value, AttrValue, AttrLen);
		// lower bound is set by Cm parameters
		pCEP->turnaround_time_us = MAX(Value, gCmParams.MinTurnaroundTimeMs*1000);
		// application is trusted for upper bound, allows for some apps to
		// indicate they are very slow
		_DBG_INFO(("<cep 0x%p> Modifying CEP for Turnaround time %"PRIu64" us.\n", _DBG_PTR(pCEP), pCEP->turnaround_time_us));
	} else if (AttrType == CM_FLAG_APM) {
		if (AttrLen != sizeof(uint32))
		{
			Status = FINVALID_PARAMETER;
			goto unlock;
		}
		if (pCEP->Mode == CM_MODE_ACTIVE)
		{
			Status = FINVALID_STATE;
			goto unlock;
		}
		MemoryCopy(&Value, AttrValue, AttrLen);
		pCEP->bFailoverSupported = (boolean) Value;
		_DBG_INFO(("<cep 0x%p> Modifying CEP for APM <bFailoverSupported %d>.\n", _DBG_PTR(pCEP), pCEP->bFailoverSupported));
	} else {
		Status = FINVALID_PARAMETER;
	}

unlock:
	SpinLockRelease(&gCM->ListLock);

done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // iba_cm_modify_cep()

// help client build a connect request and QP Attrs to move QP to Init
FSTATUS iba_cm_prepare_request(
				IN CM_CEP_TYPE TransportServiceType,
				IN const IB_PATH_RECORD *Path,
				IN OPTIONAL const IB_PATH_RECORD *AltPath,
				OUT CM_PREPARE_REQUEST_INFO *Info
				)
{
	FSTATUS Status;
	IB_CA_ATTRIBUTES *pCaAttr = NULL;
	IB_PORT_ATTRIBUTES *pPortAttr = NULL;

	_DBG_ENTER_FUNC(iba_cm_prepare_request);

	if (TransportServiceType == CM_UD_TYPE || Info == NULL || Path == NULL)
	{
		Status= FINVALID_PARAMETER;
		goto done;
	}

	MemoryClear(Info, sizeof(*Info));
	// TBD RD not implemented/accurate
	// find CA and port based on SGID
	Status = iba_find_gid(&Path->SGID, &Info->Request.CEPInfo.CaGUID,
					&Info->Request.CEPInfo.PortGUID, NULL,
					&pPortAttr, &pCaAttr);
	if (Status != FSUCCESS)
		goto done;

	if (! LookupPKey(pPortAttr, Path->P_Key, FALSE, &Info->QpAttrInit.PkeyIndex))
	{
		Status = FNOT_FOUND;
		goto free;
	}
	if (pPortAttr) {
		MemoryDeallocate(pPortAttr);
		pPortAttr = NULL;
	}
	Info->QpAttrInit.RequestState = QPStateInit;
	Info->QpAttrInit.PortGUID = Info->Request.CEPInfo.PortGUID;
	Info->QpAttrInit.Attrs = (IB_QP_ATTR_PORTGUID|IB_QP_ATTR_PKEYINDEX);
	// application must build AccessControl

	// CaGUID set above
	Info->Request.CEPInfo.EndToEndFlowControl = (TransportServiceType != CM_UC_TYPE);
	// PortGUID set above
	SpinLockAcquire(&gCM->ListLock);
	Info->Request.CEPInfo.StartingPSN = GenerateStartPSN();
	SpinLockRelease(&gCM->ListLock);
		// provide sensible defaults for retry counts
	Info->Request.CEPInfo.RetryCount = IB_MAX_RETRY_COUNT;
	Info->Request.CEPInfo.RnrRetryCount = IB_RNR_RETRY_COUNT_INFINITE;
	if (TransportServiceType == CM_RC_TYPE) {
		Info->Request.CEPInfo.OfferedInitiatorDepth = pCaAttr->MaxQPInitiatorDepth;
		Info->Request.CEPInfo.OfferedResponderResources = pCaAttr->MaxQPResponderResources;
	} else if (TransportServiceType == CM_RD_TYPE) {
		Info->Request.CEPInfo.OfferedInitiatorDepth = pCaAttr->MaxEECInitiatorDepth;
		Info->Request.CEPInfo.OfferedResponderResources = pCaAttr->MaxEECResponderResources;
	} else {
		Info->Request.CEPInfo.OfferedInitiatorDepth = 0;
		Info->Request.CEPInfo.OfferedResponderResources = 0;
	}
	Info->Request.CEPInfo.AckTimeout = ComputeAckTimeout(Path->PktLifeTime,
												pCaAttr->LocalCaAckDelay);
	Info->Request.PathInfo.bSubnetLocal = ! IsGlobalRoute(Path);
	Info->Request.PathInfo.Path = *Path;
	if (AltPath && AltPath->SLID) {
		// we could let the Alt Path verify be defered to the iba_cm_connect
		// however its better to check it now so the client can
		// consider different path choices
		Status = iba_find_ca_gid2(&AltPath->SGID, Info->Request.CEPInfo.CaGUID,
					NULL, NULL, &pPortAttr, NULL);
		if (Status != FSUCCESS)
			goto free;
		if ( Path->P_Key != AltPath->P_Key ||
			! LookupPKey(pPortAttr, Path->P_Key, FALSE, NULL))
		{
			Status = FNOT_FOUND;
			goto free;
		}
		Info->Request.CEPInfo.AlternateAckTimeout = ComputeAckTimeout(
								AltPath->PktLifeTime, pCaAttr->LocalCaAckDelay);
		Info->Request.AlternatePathInfo.bSubnetLocal = ! IsGlobalRoute(AltPath);
		Info->Request.AlternatePathInfo.Path = *AltPath;
	}
	// caller must set up
	//Info->Request.SID
	//Info->Request.CEPInfo.QKey - for RD
	//Info->Request.CEPInfo.QPN
	//Info->Request.CEPInfo.LocalEECN - for RD
	//Info->Request.CEPInfo.RemoteEECN - for RD
	// caller may adjust other fields, especially:
	// 	Offered* - may adjust down
	// 	AckTimeout/AlternateAckTimeout - may increase by recomputing
	// 		using a larger value for LocalAckDelay for busy apps like oracle
	// 	RetryCount, RnRRetryCount
free:
	if (pCaAttr)
		MemoryDeallocate(pCaAttr);
	if (pPortAttr)
		MemoryDeallocate(pPortAttr);
done:
	_DBG_LEAVE_FUNC();
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// iba_cm_connect
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_connect(
	IB_HANDLE				hCEP,
	const CM_REQUEST_INFO*	pConnectRequest,
	PFN_CM_CALLBACK			pfnConnectCB,
	void*					Context
	)
{
	CM_CEP_OBJECT*	pCEP=(CM_CEP_OBJECT*)hCEP;
	FSTATUS			Status = FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_connect);

	// Parameter check
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return Status;
	}

	Status = RequestCheck(pConnectRequest);
	if (Status != FSUCCESS)
	{
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return Status;
	}

	SpinLockAcquire(&gCM->ListLock);

	// We must be in idle state
	if (pCEP->State != CMS_IDLE)
	{
		SpinLockRelease(&gCM->ListLock);
	
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	
		return FINVALID_STATE;
	}

	Status = Connect(pCEP, pConnectRequest, FALSE, pfnConnectCB, Context);
	
	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // iba_cm_connect()


//////////////////////////////////////////////////////////////////////////
// iba_cm_connect_peer
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_connect_peer(
	IB_HANDLE				hCEP,
	const CM_REQUEST_INFO*	pConnectRequest,
	PFN_CM_CALLBACK			pfnConnectCB,
	void*					Context
	)
{
	CM_CEP_OBJECT*	pCEP=(CM_CEP_OBJECT*)hCEP;
	FSTATUS Status=FSUCCESS;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_connect_peer);

	// Parameter check
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return Status;
	}

	SpinLockAcquire(&gCM->ListLock);

	// We must be in idle state
	if (pCEP->State != CMS_IDLE)
	{
		SpinLockRelease(&gCM->ListLock);
	
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	
		return FINVALID_STATE;
	}

	Status = Connect(pCEP, pConnectRequest, TRUE, pfnConnectCB, Context);

	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // iba_cm_connect_peer()


//////////////////////////////////////////////////////////////////////////
// iba_cm_listen
//
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_listen(
	IN IB_HANDLE				hCEP,
	IN const CM_LISTEN_INFO*	pListenInfo,
	IN PFN_CM_CALLBACK			pfnListenCB,
	IN void*					Context
	)
{
	CM_CEP_OBJECT*	pCEP=(CM_CEP_OBJECT*)hCEP;

	FSTATUS			Status=FSUCCESS;
#ifndef _CM_DIRECTED_LISTENS_
	CM_LISTEN_INFO	ListenInfo = *pListenInfo;
#endif


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_listen);

	//
	// Parameter check
	//
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s >\n",
						_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));

		return Status;
	}

	// Backward compatibility. Otherwise, incoming requests will be dropped if
	// caller does not clear the struct properly
#ifndef _CM_DIRECTED_LISTENS_
	ListenInfo.CaGUID = 0;
	
	ListenInfo.ListenAddr.Port.LID = 0;
	memset(ListenInfo.ListenAddr.Port.GID.Raw, 0, sizeof(IB_GID));

	ListenInfo.RemoteAddr.Port.LID = 0;
	memset(ListenInfo.RemoteAddr.Port.GID.Raw, 0, sizeof(IB_GID));
#endif

	// Acquire the lock since we need to check if the address space is already in use
	// and we may need to move to LISTEN list
	SpinLockAcquire(&gCM->ListLock);

	// We must be in idle state
	if (pCEP->State != CMS_IDLE)
	{
		SpinLockRelease(&gCM->ListLock);

		_DBG_ERROR(("<cep 0x%p> Invalid state!!! <%s>\n",
					_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State))));

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

		return FINVALID_STATE;
	}

#ifndef _CM_DIRECTED_LISTENS_
	Status = Listen(pCEP, &ListenInfo, pfnListenCB, Context);
#else
	Status = Listen(pCEP, pListenInfo, pfnListenCB, Context);
#endif

	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // iba_cm_listen()


//////////////////////////////////////////////////////////////////////////
// iba_cm_wait
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//		>
//
//
//	FTIMEOUT
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_wait(
	IB_HANDLE				CEPHandleArray[],
	CM_CONN_INFO*			ConnInfoArray[],
	uint32					ArrayCount,
	uint32					Timeout_us
	//EVENT_HANDLE			hWaitEvent		// Associate the array of handles to this event
	)
{
	uint32 i=0;

	FSTATUS Status=FSUCCESS;
	boolean bSignaled=FALSE;

	uint32 WaitTimeUs=Timeout_us;
	uint64 TimeoutUs=Timeout_us;
	uint64 StartTimeUs=GetTimeStamp();

	uint64 CurrTimeUs=0;
	uint64 TimeElapsedUs=0;


	_DBG_ENTER_EXT(_DBG_LVL_FUNC_TRACE, iba_cm_wait, == PASSIVE);

	_DBG_INFO(("iba_cm_wait() <cnt %d %ums %"PRIu64"us>\n", ArrayCount, Timeout_us, GetTimeStamp()));

	if ( (ArrayCount == 0) || (ArrayCount > gCmParams.MaxConns) )
		return FINVALID_PARAMETER;

	//if ( (ArrayCount > 1) && (hWaitEvent == NULL) )
	//	return FINVALID_PARAMETER;

	while (1)
	{
		// Update the timeout
		if ((int32)WaitTimeUs != EVENT_NO_TIMEOUT)
		{
			if (CurrTimeUs >= StartTimeUs)
			{
				TimeElapsedUs = (CurrTimeUs - StartTimeUs);
			}
			else // wrap-around
			{
				TimeElapsedUs = (CurrTimeUs + ((uint64)(-1) - StartTimeUs));
			}

			if (TimeoutUs > TimeElapsedUs)
			{
				WaitTimeUs = (uint32) (TimeoutUs - TimeElapsedUs);
			}
			else
			{
				WaitTimeUs = 0;
			}
		}

		// Walk thru the array to see if the object is signaled
		for (i=0; i < ArrayCount; i++)
		{
			// No need to check obj state if obj is marked for destruction
			//if (BitTest((CM_CEP_OBJECT*)CEPHandleArray[i])->EventFlags, USRE_DESTROY))
			//{
			//	continue;
			//}

			// Associate the CEP object with wait event before
			// we call wait() to prevent race condition
			//SpinLockAcquire(&gCM->ListLock);
			
			// TODO: Should we checked to make sure we dont override an existing one ???
			// Otherwise, the other thread could wait forever ??
			((CM_CEP_OBJECT*)CEPHandleArray[i])->pWaitEvent = &gCM->EventObj;//(EVENT*) hWaitEvent;

			//SpinLockRelease(&gCM->ListLock);

			// Specify 0 timeout as a side-effect to test the CEP object signaled state
			// FSUCCESS means the obj is in signaled state, FTIMEOUT means the obj is not in signaled state
			if (((CM_CEP_OBJECT*)CEPHandleArray[i])->Mode == CM_MODE_PASSIVE)
				Status = WaitP((CM_CEP_OBJECT*)CEPHandleArray[i], ConnInfoArray[i], 0);
			else
				Status = WaitA((CM_CEP_OBJECT*)CEPHandleArray[i], ConnInfoArray[i], 0);

			// The obj is in signaled state
			if (Status == FSUCCESS)
			{
				//_DBG_INFO(("<cep 0x%p> Wait() returned <wait index %d %s info status %d>\n", 
				//			_DBG_PTR(CEPHandleArray[i]), i,
				//			_DBG_PTR(FSTATUS_MSG(Status)),
				//			ConnInfoArray[i]->Status));

				Status = FCM_WAIT_OBJECT0 + i;

				// Event is signaled, there should be additional info in ConnInfo[]
				bSignaled = TRUE;
				break;
			}
			else
			{
				// Status is FTIMEOUT here
				//_DBG_INFO(("<cep 0x%p> Wait() returned <%s>\n", 
				//			_DBG_PTR(CEPHandleArray[i]),
				//			_DBG_PTR(FSTATUS_MSG(Status))));

				// No cep obj is signaled but if the event obj is signaled, we need to return it
				Status = FCM_WAIT_OBJECT0 + ArrayCount;
			}
		} // for()

		// if no objects' state is signaled..go to wait, otherwise return to the caller
		if (bSignaled)
			break;

		_DBG_INFO(("Entering wait state <timeout %ums, %"PRIu64"us>\n", Timeout_us, GetTimeStamp()));

		// TODO: Decrement the timeout value
		// Wait on all the specified objects using the wait event.
		Status = EventWaitOn(&gCM->EventObj/*(EVENT*)hWaitEvent*/, Timeout_us);
		bSignaled = TRUE;
		_DBG_INFO(("Exiting wait state <%s, %"PRIu64"us>\n",
					_DBG_PTR(FSTATUS_MSG(Status)), GetTimeStamp()));
	
		// We timed out, return to the caller
		if (Status == FTIMEOUT)
			break;

	} // while()

	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);

	return Status;

} // iba_cm_wait()


// Process a inbound request for a server
FSTATUS iba_cm_process_request(
				IN IB_HANDLE hCEP,
				IN CM_REP_FAILOVER Failover,
				OUT CM_PROCESS_REQUEST_INFO *Info
				)
{
	FSTATUS Status;
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)hCEP;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_process_request);
	if (! Info)
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s >\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		goto done;
	}

	SpinLockAcquire(&gCM->ListLock);
	if (pCEP->Mode == CM_MODE_PASSIVE)
		Status = ProcessRequestP(pCEP, Failover, Info);
	else
		Status = FINVALID_STATE;
	SpinLockRelease(&gCM->ListLock);
done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;
}

//////////////////////////////////////////////////////////////////////////
// iba_cm_accept
//
// Accept a Connection/Connection Response
// For a Client:
//		Should be invoked as the result of a FCM_CONNECT_REPLY callback
//		In which case this causes the RTU to be sent.
//		pSendConnInfo contains the RTU and pRecvConnInfo is unused
// For a Server:
//		Should be invoked as the result of a FCM_CONNECT_REQUEST callback
//		In which case this causes the REP to be sent.
//		if CM_FLAG_ASYNC_ACCEPT is set in the CEP:
//			This returns immediately
//		otherwise
//			This synchronously waits for the RTU
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - Client RTU queued to be sent.
//	FPENDING - Server Async Accept, REP queued to be sent.
//	FCM_CONNECT_ESTABLISHED - server REP sent and RTU received
//	FCM_CONNECT_REJECT - server REP sent and REJ received
//	FCM_CONNECT_TIMEOUT - server REP sent and timeout waiting for RTU
//	FCM_CONNECT_CANCEL - server listen on this CEP has been cancelled
//
//	FINVALID_STATE - The endpoint is not in the valid state for this call
//	FERROR	- Unable to send the reply ack packet
//	FTIMEOUT - The timeout interval expires
//
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_accept(
	IN IB_HANDLE				hCEP,
	IN CM_CONN_INFO*			pSendConnInfo,		// Send REP or RTU
	OUT CM_CONN_INFO*			pRecvConnInfo,		// Rcvd RTU, REJ or TIMEOUT
	IN PFN_CM_CALLBACK			pfnCallback,
	IN void*					Context,
	OUT IB_HANDLE*				hNewCEP
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)hCEP;
	FSTATUS		Status;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_accept);

	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s >\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		return Status;
	}

	if (pCEP->Mode == CM_MODE_PASSIVE)
	{
		// If we are doing a client/server accept, we will be returning a handle
		if (!pCEP->bPeer && hNewCEP == NULL)
		{
			return FINVALID_PARAMETER;
		}
		Status = AcceptP(pCEP, pSendConnInfo, pRecvConnInfo, pfnCallback, Context, (CM_CEP_OBJECT**)hNewCEP);
	} else {
		Status = AcceptA(pCEP, pSendConnInfo);
	}

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // iba_cm_accept()



//////////////////////////////////////////////////////////////////////////
// CmAcceptEx
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - The function has completed successfully. For server, this indicates the initialization.
//		>FCM_CONNECT_ESTABLISHED
//		>FCM_CONNECT_REJECT
//		>FCM_CONNECT_TIMEOUT
//		>FCM_CONNECT_CANCEL
//
//	FINVALID_STATE - The endpoint is not in the valid state for this call
//	FERROR	- Unable to send the reply ack packet
//	FTIMEOUT - The timeout interval expires
//
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
CmAcceptEx(
	IN IB_HANDLE				hCEP,
	IN CM_CONN_INFO*			pSendConnInfo,		// Send REP
	IN EVENT_HANDLE				hEvent,				// For hNewCEP
	IN EVENT_HANDLE				hWaitEvent,			// For hNewCEP
	IN PFN_CM_CALLBACK			pfnCallback,		// For hNewCEP
	IN void*					Context,			// For hNewCEP
	OUT IB_HANDLE*				hNewCEP
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)hCEP;
	FSTATUS		Status;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CmAcceptEx);

	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s>\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		return Status;
	}

	if (pCEP->Mode == CM_MODE_PASSIVE)
	{
		// If we are doing a client/server accept, we will be returning a handle
		if (!pCEP->bPeer && hNewCEP == NULL)
		{
			return FINVALID_PARAMETER;
		}

		Status = AcceptP_Async(pCEP, pSendConnInfo, hEvent, hWaitEvent, pfnCallback, Context, FALSE, (CM_CEP_OBJECT**)hNewCEP);
	} else {
		Status = AcceptA(pCEP, pSendConnInfo);
	}

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // CmAcceptEx()


//////////////////////////////////////////////////////////////////////////
// iba_cm_reject
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_reject(
	IN IB_HANDLE				hCEP,
	IN const CM_REJECT_INFO*	pConnectReject
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)hCEP;

	FSTATUS	Status=FSUCCESS;

	//
	// Parameter check
	//
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
		return Status;

	SpinLockAcquire(&gCM->ListLock);

	if (pCEP->Mode == CM_MODE_PASSIVE)
		Status = RejectP(pCEP, pConnectReject);
	else
		Status = RejectA(pCEP, pConnectReject);

	SpinLockRelease(&gCM->ListLock);

	return Status;

} // iba_cm_reject()


//////////////////////////////////////////////////////////////////////////
// iba_cm_cancel
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_cancel(
	IN IB_HANDLE				hCEP
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*) hCEP;

	FSTATUS Status=FSUCCESS;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_cancel);

	// Parameter check
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s>\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
		return Status;
	}

	SpinLockAcquire(&gCM->ListLock);

	if (pCEP->Mode == CM_MODE_PASSIVE)
		Status = CancelP(pCEP);
	else
		Status = CancelA(pCEP);

	SpinLockRelease(&gCM->ListLock);
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;
} // iba_cm_cancel()


// build QP Attributes to move client QP to RTR and RTS
FSTATUS iba_cm_process_reply(
	IN IB_HANDLE hCEP,
	OUT CM_PROCESS_REPLY_INFO* Info
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*) hCEP;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_process_reply);

	// Parameter check
	if (! Info)
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s>\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		goto done;
	}

	SpinLockAcquire(&gCM->ListLock);
	if (pCEP->Mode != CM_MODE_ACTIVE || pCEP->Type == CM_UD_TYPE)
	{
		Status = FINVALID_STATE;
	} else {
		Status = ProcessReplyA(pCEP, Info);
	}
	SpinLockRelease(&gCM->ListLock);
done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;
}


// build QP Attributes to move server QP to RTS for given CEP
FSTATUS iba_cm_prepare_rts(
				IN IB_HANDLE hCEP,
				OUT IB_QP_ATTRIBUTES_MODIFY *QpAttrRts
				)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*) hCEP;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_prepare_rts);

	// Parameter check
	if (QpAttrRts == NULL)
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s>\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		goto done;
	}

	SpinLockAcquire(&gCM->ListLock);
	// TBD - could allow ACTIVE
	if (pCEP->Mode != CM_MODE_PASSIVE)
	{
		Status = FINVALID_STATE;
		goto unlock;
	}
	if (pCEP->Type == CM_UD_TYPE)
	{
		Status = FINVALID_STATE;
		goto unlock;
	}
	// If allowed for Active these states are ok, probably wouldn't want to
	// allow CMS_LAP_SENT, CMS_MRA_LAP_RCVD
	// uncomment REP tests below to allow the RTS transtion anytime after
	// sending the REP, however it is prefered to not move to RTS until
	// the RTU is received
	if (pCEP->State != CMS_ESTABLISHED && pCEP->State != CMS_LAP_RCVD
		&& pCEP->State != CMS_MRA_LAP_SENT
		//&& pCEP->State != CMS_REP_SENT && pCEP->State != CMS_MRA_REP_RCVD
		)
	{
		Status = FINVALID_STATE;
		goto unlock;
	}

	//MemoryClear(QpAttrRts, sizeof(*QpAttrRts));	// not needed due to Attrs flags
	GetRtsAttrFromCep(pCEP, QpAttrRts);
unlock:
	SpinLockRelease(&gCM->ListLock);
done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;
}

//////////////////////////////////////////////////////////////////////////
// iba_cm_altpath_request
//
// Request a new alternate path
// Only allowed for Client/Active side of connection
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FPENDING - LAP queued to be sent.
//	FINVALID_STATE - The endpoint is not in the valid state for this call
//	FERROR	- Unable to send the lap packet
//	FINVALID_ARGUMENTS - invalid pLapInfo
//
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_altpath_request(
	IN IB_HANDLE				hCEP,
	IN const CM_ALTPATH_INFO*	pLapInfo		// Send LAP
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)hCEP;
	FSTATUS		Status;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_altpath_request);

	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s >\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		return Status;
	}

	SpinLockAcquire(&gCM->ListLock);
	if (pCEP->Mode != CM_MODE_ACTIVE)
	{
		return FINVALID_STATE;
	} else {
		Status = AltPathRequestA(pCEP, pLapInfo);
	}
	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // iba_cm_altpath_request()


// build QP Attributes to load new alternate path into server QP
// and prepare a Reply
FSTATUS iba_cm_process_altpath_request(
	IN IB_HANDLE hCEP,
	IN CM_ALTPATH_INFO*			AltPathRequest,
	OUT CM_PROCESS_ALTPATH_REQUEST_INFO* Info
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*) hCEP;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_process_altpath_request);

	// Parameter check
	if (! AltPathRequest || ! Info)
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s>\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		goto done;
	}

	SpinLockAcquire(&gCM->ListLock);
	if (pCEP->Mode != CM_MODE_PASSIVE || pCEP->Type == CM_UD_TYPE)
	{
		Status = FINVALID_STATE;
	} else {
		Status = ProcessAltPathRequestP(pCEP, AltPathRequest, Info);
	}
	SpinLockRelease(&gCM->ListLock);
done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;
}


//////////////////////////////////////////////////////////////////////////
// iba_cm_altpath_reply
//
// Reply to an alternate path request
// Only allowed for Server/Passive side of connection
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - APR queued to be sent.
//	FINVALID_STATE - The endpoint is not in the valid state for this call
//	FERROR	- Unable to send the apr packet
//
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_altpath_reply(
	IN IB_HANDLE				hCEP,
	IN CM_ALTPATH_REPLY_INFO*	pAprInfo		// Send APR
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)hCEP;
	FSTATUS		Status;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_altpath_reply);

	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s >\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		return Status;
	}

	SpinLockAcquire(&gCM->ListLock);
	if (pCEP->Mode != CM_MODE_PASSIVE)
	{
		return FINVALID_STATE;
	} else {
		Status = AltPathReplyP(pCEP, pAprInfo);
	}
	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // iba_cm_altpath_reply()

// build QP Attributes to load new alternate path into client QP
FSTATUS iba_cm_process_altpath_reply(
	IN IB_HANDLE 					hCEP,
	IN const CM_ALTPATH_REPLY_INFO*	AltPathReply,
	OUT IB_QP_ATTRIBUTES_MODIFY*	QpAttrRts
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*) hCEP;
	FSTATUS Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_process_altpath_reply);

	// Parameter check
	if (! AltPathReply || ! QpAttrRts)
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s>\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		goto done;
	}

	SpinLockAcquire(&gCM->ListLock);
	if (pCEP->Mode != CM_MODE_ACTIVE || pCEP->Type == CM_UD_TYPE)
	{
		Status = FINVALID_STATE;
	} else {
		Status = ProcessAltPathReplyA(pCEP, AltPathReply, QpAttrRts);
	}
	SpinLockRelease(&gCM->ListLock);
done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;
}

//////////////////////////////////////////////////////////////////////////
// iba_cm_migrated
//
// Inform CM that alternate path has been migrated to by client or server
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - CEP adjusted to reflect migration
//	FINVALID_STATE - The endpoint is not in the valid state for this call
//
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_migrated(
	IN IB_HANDLE				hCEP
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)hCEP;
	FSTATUS		Status;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_migrated);

	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s >\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		return Status;
	}

	SpinLockAcquire(&gCM->ListLock);
	Status = MigratedA(pCEP);
	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // iba_cm_migrated()

//////////////////////////////////////////////////////////////////////////
// iba_cm_migrated_reload
//
// Inform CM that alternate path has been migrated to by client
// and prepare a AltPath Request which has the old primary path
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - CEP adjusted to reflect migration
//	FINVALID_STATE - The endpoint is not in the valid state for this call
//
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_migrated_reload(
	IN IB_HANDLE				hCEP,
	OUT CM_PATH_INFO*			NewPrimaryPath OPTIONAL,
	OUT CM_ALTPATH_INFO*		AltPathRequest OPTIONAL
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)hCEP;
	FSTATUS		Status;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_migrated_reload);

	if (! AltPathRequest && ! NewPrimaryPath)
	{
		Status = FINVALID_PARAMETER;
		goto done;
	}
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s >\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		goto done;
	}

	SpinLockAcquire(&gCM->ListLock);
	if (pCEP->Mode != CM_MODE_ACTIVE || pCEP->Type != CM_UD_TYPE)
	{
		Status = FINVALID_STATE;
	} else {
		Status = MigratedReloadA(pCEP, NewPrimaryPath, AltPathRequest);
	}
	SpinLockRelease(&gCM->ListLock);

done:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return Status;

} // iba_cm_migrated_reload()

//////////////////////////////////////////////////////////////////////////
// iba_cm_disconnect
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
iba_cm_disconnect(
	IN IB_HANDLE				hCEP,
	IN const CM_DREQUEST_INFO*	pDRequest,	// Send DREQ
	IN const CM_DREPLY_INFO*	pDReply	// Send DREP
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*) hCEP;
	FSTATUS Status=FSUCCESS;

	// keep this const and off stack to save kernel stack space
	static const CM_DREQUEST_INFO	DisconnectReq = {{0}};


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_cm_disconnect);

	//
	// Parameter check
	//
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s>\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
		return Status;
	}

	// Allow both to be NULL or either one to be NULL
	if (pDRequest == NULL)
	{
		if (pDReply == NULL)
		{
			// Let the CEP state determine if we are sending a DREQ or DREP
			pDRequest = &DisconnectReq;

			pDReply = (CM_DREPLY_INFO*)&DisconnectReq;
		}
	}
	else
	{
		if (pDReply)
		{
			_DBG_ERROR(("<cep 0x%p> Invalid param!!!\n", _DBG_PTR(pCEP)));

			_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

			return FINVALID_PARAMETER;
		}
	}

	SpinLockAcquire(&gCM->ListLock);
	
	// DisconnectA() is applicable to both client and server endpoint
	Status = DisconnectA(pCEP, pDRequest, pDReply);

	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;

} // iba_cm_disconnect()


FSTATUS
CmGetConnInfo(
	IN IB_HANDLE				hCEP,
	OUT CM_CONN_INFO*			pConnInfo
	)
{
	CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*) hCEP;
	FSTATUS Status=FSUCCESS;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CmGetConnInfo);

	//
	// Parameter check
	//
	Status = ValidateCEPHandle(pCEP);
	if (Status != FSUCCESS)
	{
		_DBG_WARN(("<cep 0x%p> ValidateCEPHandle failed!!! <%s>\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));

		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
		return Status;
	}

	SpinLockAcquire(&gCM->ListLock);

	Status = GetConnInfo(pCEP, pConnInfo);

	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return Status;
} // CmGetConnInfo()

//////////////////////////////////////////////////////////////////////////
// CreateCEP
//
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
CM_CEP_OBJECT*
CreateCEP(
	CM_CEP_TYPE		TransportServiceType,
	EVENT_HANDLE	hEvent,
	EVENT_HANDLE	hWaitEvent
	)
{
	CM_CEP_OBJECT*		pCEP=NULL;
	FSTATUS				Status=FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CreateCEP);

	// Allocate memory for the CEP object
	pCEP = (CM_CEP_OBJECT*)MemoryAllocateAndClear(sizeof(CM_CEP_OBJECT), FALSE, CM_MEM_TAG);

	if (!pCEP)
	{
		_DBG_ERROR(("MemoryAllocateAndClear() CEP failed!!!\n"));
		goto fail_alloc;
	}
	pCEP->Signature = gCM->Signature;

	TimerInitState(&pCEP->TimerObj);
	Status = TimerInit(&pCEP->TimerObj, CmTimerCallback, pCEP );
	if (!Status)
	{
		_DBG_ERROR(("<cep 0x%p> !!!TimerInit() failed!!! - <%s>\n",
				_DBG_PTR(pCEP), _DBG_PTR(FSTATUS_MSG(Status))));
		goto fail_timer;
	}

	// Obtain callback item
#ifdef WINNT
	pCEP->pCallbackItem = SysCallbackGet(IbtGlobal.DeviceObject);
#else
	pCEP->pCallbackItem = SysCallbackGet(NULL);
#endif // WINNT

	if (pCEP->pCallbackItem == NULL)
	{
		_DBG_ERROR(("<cep 0x%p> !!!SysCallbackGet() failed!!!\n", _DBG_PTR(pCEP)));
		goto fail_syscallback;
	}

	// Obtain a dgrm from the pool for this obj
	// TODO: Delay this until user call Connect() or Listen() ???
	pCEP->pDgrmElement = CmDgrmGet();
	if (!pCEP->pDgrmElement)
	{
		_DBG_WARNING(("<cep 0x%p> CEP Create failed: Out of CM Dgrms!!!\n",
				_DBG_PTR(pCEP)));
		goto fail_dgrm;
	}

	_DBG_INFO(("<cep 0x%p> obtained dgrm from pool <dgrm 0x%p>.\n", 
					_DBG_PTR(pCEP), _DBG_PTR(pCEP->pDgrmElement)));

	if (hEvent)
	{
		pCEP->pEventObj = (EVENT*)hEvent;
		OsWaitObjAddRef((OS_WAIT_OBJ_HANDLE)pCEP->pEventObj);
		pCEP->bPrivateEvent = 0;	// user event, don't touch in DestroyCEP
	} else {
		pCEP->pEventObj = EventAlloc();
		if (pCEP->pEventObj == NULL)
		{
			_DBG_ERROR(("MemoryAllocateAndClear() Event failed!!!\n"));
			goto fail_event;
		}
		pCEP->bPrivateEvent = 1;	// DestroyCEP must deallocate this event
	}

	pCEP->pWaitEvent = (EVENT*)hWaitEvent;

	// Initialize the CEP object
	pCEP->Type = TransportServiceType;
	DListInit(&pCEP->PendingList);
	DListInit(&pCEP->PendingListEntry);

	QListSetObj(&pCEP->AllCEPsItem, pCEP);
	QListInsertTail(&gCM->AllCEPs, &pCEP->AllCEPsItem);

	CepSetState(pCEP, CMS_IDLE);

	_DBG_INFO(("<cep 0x%p> *** NEW CEP object CREATED ***.\n", _DBG_PTR(pCEP)));
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return pCEP;

fail_event:
	CmDgrmPut(pCEP->pDgrmElement);
fail_dgrm:
	SysCallbackPut(pCEP->pCallbackItem);
fail_syscallback:
	TimerDestroy(&pCEP->TimerObj);
fail_timer:
	pCEP->Signature =0;
	MemoryDeallocate(pCEP);
fail_alloc:
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return NULL;

} // CreateCEP()


// final part of destroying a CEP, called outside any possible timer callback
// context so we can safely destroy our Timer
void
ReapCEP(
	IN LIST_ITEM *pItem
	)
{
	CM_CEP_OBJECT	*pCEP = PARENT_STRUCT(pItem, CM_CEP_OBJECT, AllCEPsItem);
	
	_DBG_INFO(("<cep 0x%p> - Destroying the timer obj <timer 0x%p>\n", _DBG_PTR(pCEP), _DBG_PTR(&pCEP->TimerObj)));
	TimerDestroy(&pCEP->TimerObj);
	MemoryDeallocate(pCEP);
}

//////////////////////////////////////////////////////////////////////////
// DestroyCEP
//
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
FSTATUS
DestroyCEP(CM_CEP_OBJECT* pCEP)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DestroyCEP);

	// Assert we are in IDLE state
	ASSERT(pCEP->State == CMS_IDLE);

	// Assert no callback in progress or pending
	ASSERT(AtomicRead(&pCEP->CallbackRefCnt) == 0);

	if (pCEP->ApcRetryCount)
	{
		QListRemoveItem(&gCM->ApcRetryList, &pCEP->ApcRetryListItem);
		pCEP->ApcRetryCount = 0;
	}

	// Assert no timeout in progress or pending
	//ASSERT(pCEP->timeout_ms == 0);
	if (pCEP->timeout_ms)
	{
		CmTimerStop(pCEP, LAST_KNOWN_TIMER);
	}

	ASSERT(pCEP->LocalCommID == 0);
	ASSERT(pCEP->LocalEndPoint.QPN == 0);
	ASSERT(pCEP->LocalEndPoint.EECN == 0);

	if (pCEP->pDgrmElement)
	{
		_DBG_INFO(("<cep 0x%p> Releasing the dgrm <dgrm 0x%p>.\n", 
						_DBG_PTR(pCEP), _DBG_PTR(pCEP->pDgrmElement)));
		CmDgrmRelease(pCEP->pDgrmElement);
	}

	if (pCEP->bPrivateEvent)
	{
		_DBG_INFO(("<cep 0x%p> - Deallocating the event obj <event 0x%p>\n", _DBG_PTR(pCEP), _DBG_PTR(pCEP->pEventObj)));
		EventDealloc(pCEP->pEventObj);
	}
	else
	{
		OsWaitObjRemoveRef((OS_WAIT_OBJ_HANDLE)pCEP->pEventObj);
	}

	pCEP->pEventObj = NULL;
	
	if (pCEP->pCallbackItem)
	{
		_DBG_INFO(("<cep 0x%p> - Releasing the callback obj <callback obj 0x%p>\n", _DBG_PTR(pCEP), _DBG_PTR(pCEP->pCallbackItem)));
		SysCallbackPut(pCEP->pCallbackItem);
	}

	QListRemoveItem(&gCM->AllCEPs, &pCEP->AllCEPsItem);
	_DBG_INFO(("<cep 0x%p> *** CEP object DESTROYED *** <lcid 0x%x rcid 0x%x eventflag 0x%x>.\n", 
				_DBG_PTR(pCEP), pCEP->LocalCommID, pCEP->RemoteCommID, pCEP->EventFlags));

	pCEP->Signature =0;
	if (pCEP->Discriminator)
	{
		MemoryDeallocate(pCEP->Discriminator);
		pCEP->Discriminator = NULL;
	}

	// Defer to Reaper to complete destroy, we could be in a TIMEWAIT callback
	ReaperQueue(&pCEP->AllCEPsItem, ReapCEP);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return FSUCCESS;

} // DestroyCEP()

uint32 SidrTimeout(IN CM_CEP_OBJECT* pCEP)
{
	// use our timeout (representing ~ CPU delays) + Round-trip time for fabric
	if (pCEP->PktLifeTime >= 31) {
		// odd case, avoid overflow
		return TimeoutMultToTimeInMs(31);	// this is 2.4 hrs !!!!
	} else {
		return gCmParams.SidrReqTimeoutMs + 2*TimeoutMultToTimeInMs(pCEP->PktLifeTime);
	}
}

// Queue an APC retry
void
QueueApcRetry(IN CM_CEP_OBJECT* pCEP)
{
	if (! pCEP->ApcRetryCount)
	{
		QListSetObj(&pCEP->ApcRetryListItem, pCEP);
		QListInsertTail(&gCM->ApcRetryList, &pCEP->ApcRetryListItem);
	}
	pCEP->ApcRetryCount++;
	if (! gCM->ApcRetryTimerRunning)
	{
		TimerStart(&gCM->ApcRetryTimer, CM_APC_RETRY_INTERVAL);
		gCM->ApcRetryTimerRunning = 1;
	}
}

// APC Retry Timer handler
void
ApcRetryTimerCallback(IN void *Context)
{
	int count = 0;
	LIST_ITEM* pListItem=NULL;
	int thisPass = 0;	// number processed so far in this pass through list
	int listDepth = QListCount(&gCM->ApcRetryList);	// length of list for this pass
	int failCount = 0;	// number that failed in this pass through list

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ApcRetryTimerCallback);

	SpinLockAcquire(&gCM->ListLock);
	if (TimerCallbackStopped(&gCM->ApcRetryTimer))
		goto done;	// stale timer callback

	ASSERT(gCM->ApcRetryTimerRunning);

	// limit number of Apc retries we attempt per timer callback
	// this way a swamped system won't stay in the Lock/IRQ too long
	while (count < CM_APC_RETRY_LIMIT
			&& (pListItem = QListRemoveHead(&gCM->ApcRetryList)) != NULL)
	{
		CM_CEP_OBJECT* pCEP= (CM_CEP_OBJECT*)QListObj(pListItem);

		ASSERT(pCEP);
		ASSERT(pCEP->ApcRetryCount);
		if (BitTest(pCEP->EventFlags, USRE_DESTROY)) {
			// the CEP is marked for destruction
			// so don't attempt to issue callbacks against it anymore
			pCEP->ApcRetryCount = 0;
			_DBG_WARNING(("<cep 0x%p> Object marked for destruction - skip APC retry <%s>\n",
					_DBG_PTR(pCEP), _DBG_PTR(CmGetStateString(pCEP->State))));

		} else  {
			if (--(pCEP->ApcRetryCount) != 0) {
				// keep it on the list
				QListInsertTail(&gCM->ApcRetryList, &pCEP->ApcRetryListItem);
			}
			if (FSUCCESS != ProcessApcRetry(pCEP))
				failCount++;
		}
		// optimize by not repeating processing if no progress is being made
		if (++thisPass == listDepth) {
			// we completed a pass through the list
			if (failCount == listDepth) {
				// every CEP failed to make progress, stop processing
				break;
			}
			// initialize counters for next pass through the list
			thisPass = 0;
			listDepth = QListCount(&gCM->ApcRetryList);
			failCount = 0;
		}
		count++;
	}
	if (! QListIsEmpty(&gCM->ApcRetryList))
	{
		TimerStart(&gCM->ApcRetryTimer, CM_APC_RETRY_INTERVAL);
	} else {
		gCM->ApcRetryTimerRunning = 0;
	}
done:
	SpinLockRelease(&gCM->ListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

// Clear Statistics
void
CmClearStats(void)
{
	// clear individually for a more atomic clear than memset would offer
	AtomicWrite(&gCM->Sent.ClassPortInfo, 0);
	AtomicWrite(&gCM->Sent.ClassPortInfoResp, 0);
	AtomicWrite(&gCM->Sent.Req, 0);
	AtomicWrite(&gCM->Sent.Rep, 0);
	AtomicWrite(&gCM->Sent.Rtu, 0);
	AtomicWrite(&gCM->Sent.MraReq, 0);
	AtomicWrite(&gCM->Sent.MraRep, 0);
	AtomicWrite(&gCM->Sent.MraLap, 0);
	AtomicWrite(&gCM->Sent.Lap, 0);
	AtomicWrite(&gCM->Sent.AprAcc, 0);
	AtomicWrite(&gCM->Sent.AprRej, 0);
	AtomicWrite(&gCM->Sent.RejReq, 0);
	AtomicWrite(&gCM->Sent.RejRep, 0);
	AtomicWrite(&gCM->Sent.Dreq, 0);
	AtomicWrite(&gCM->Sent.Drep, 0);
	AtomicWrite(&gCM->Sent.SidrReq, 0);
	AtomicWrite(&gCM->Sent.SidrResp, 0);
	AtomicWrite(&gCM->Sent.Failed, 0);

	AtomicWrite(&gCM->Recv.ClassPortInfo, 0);
	AtomicWrite(&gCM->Recv.ClassPortInfoResp, 0);
	AtomicWrite(&gCM->Recv.Req, 0);
	AtomicWrite(&gCM->Recv.Rep, 0);
	AtomicWrite(&gCM->Recv.Rtu, 0);
	AtomicWrite(&gCM->Recv.MraReq, 0);
	AtomicWrite(&gCM->Recv.MraRep, 0);
	AtomicWrite(&gCM->Recv.MraLap, 0);
	AtomicWrite(&gCM->Recv.Lap, 0);
	AtomicWrite(&gCM->Recv.AprAcc, 0);
	AtomicWrite(&gCM->Recv.AprRej, 0);
	AtomicWrite(&gCM->Recv.RejReq, 0);
	AtomicWrite(&gCM->Recv.RejRep, 0);
	AtomicWrite(&gCM->Recv.Dreq, 0);
	AtomicWrite(&gCM->Recv.Drep, 0);
	AtomicWrite(&gCM->Recv.SidrReq, 0);
	AtomicWrite(&gCM->Recv.SidrResp, 0);
	AtomicWrite(&gCM->Recv.Failed, 0);
}

// copy Path information from a Connect or LoadAltPath into the given
// CEP Path (can be pCEP->PrimaryPath or pCEP->AlternatePath)
// Context of path assumes Local is Source
void CopyPathInfoToCepPath(OUT CM_CEP_PATH *pCepPath,
				IN const CM_CEP_PATHINFO *pPathInfo, IN uint8 AckTimeout)
{
	// caller handles pCepPath->LocalPortGuid;
	pCepPath->LocalGID = pPathInfo->Path.SGID;
	pCepPath->RemoteGID = pPathInfo->Path.DGID;
	pCepPath->LocalLID =  pPathInfo->Path.SLID;
	pCepPath->RemoteLID = pPathInfo->Path.DLID;
	pCepPath->FlowLabel = pPathInfo->Path.u1.s.FlowLabel;
	pCepPath->StaticRate = pPathInfo->Path.Rate;
	// caller handles pCepPath->PkeyIndex;
	pCepPath->TClass = pPathInfo->Path.TClass;
	pCepPath->SL = pPathInfo->Path.u2.s.SL;
	// pCepPath->bSubnetLocal = (! IsGlobalRoute(&pPathInfo->Path));
	pCepPath->bSubnetLocal = pPathInfo->bSubnetLocal;
	pCepPath->AckTimeout = AckTimeout;
	// will factor in TargetAckDelay when get REP
	pCepPath->LocalAckTimeout = pPathInfo->Path.PktLifeTime;
	pCepPath->HopLimit = pPathInfo->Path.u1.s.HopLimit;
	// caller handles pCepPath->LocalGidIndex;
	// caller handles pCepPath->LocalPathBits;
}

// build a QP Destination AV from a CEP Path
void GetAVFromCepPath(IN CM_CEP_PATH* CepPath, OUT IB_ADDRESS_VECTOR* DestAv)
{
	DestAv->PortGUID = CepPath->LocalPortGuid;	// not needed, but doesn't hurt
	DestAv->DestLID = CepPath->RemoteLID;
	DestAv->PathBits = CepPath->LocalPathBits;
	DestAv->ServiceLevel = CepPath->SL;
	DestAv->StaticRate = CepPath->StaticRate;
	DestAv->GlobalRoute = ! CepPath->bSubnetLocal;
	if (DestAv->GlobalRoute)
	{
		DestAv->GlobalRouteInfo.DestGID = CepPath->RemoteGID;
		DestAv->GlobalRouteInfo.FlowLabel = CepPath->FlowLabel;
		DestAv->GlobalRouteInfo.SrcGIDIndex = CepPath->LocalGidIndex;
		DestAv->GlobalRouteInfo.HopLimit = CepPath->HopLimit;
		DestAv->GlobalRouteInfo.TrafficClass = CepPath->TClass;
	}
}

// copy Path information from CEP Path to PathInfo for
// a CM FCM_ALTPATH_REPLY callback.
// Context of path assumes Local is Source
void CopyCepPathToPathInfo(OUT CM_CEP_PATHINFO *pPathInfo,
				OUT uint8* pAckTimeout, IN CM_CEP_PATH *pCepPath)
{
	pPathInfo->Path.SLID = pCepPath->LocalLID;
	pPathInfo->Path.SGID = pCepPath->LocalGID;
	pPathInfo->Path.DLID = pCepPath->RemoteLID;
	pPathInfo->Path.DGID = pCepPath->RemoteGID;
	pPathInfo->Path.NumbPath = 1; // Only one path
	pPathInfo->Path.u1.s.FlowLabel = pCepPath->FlowLabel;
	pPathInfo->Path.u1.s.HopLimit = pCepPath->HopLimit;
	pPathInfo->Path.TClass = pCepPath->TClass;
	pPathInfo->Path.u2.s.SL = pCepPath->SL;
	pPathInfo->Path.Rate = pCepPath->StaticRate;
	pPathInfo->Path.RateSelector = IB_SELECTOR_EQ;
	//pPathInfo->Path.P_Key = caller copies from CEP
	//pPathInfo->Path.Mtu = caller copies from CEP
	pPathInfo->Path.MtuSelector = IB_SELECTOR_EQ;
	*pAckTimeout = pCepPath->AckTimeout;
	// AckTimeout/2 approximates PktLifeTime with roundup
	if (pCepPath->AckTimeout > 1)
		pPathInfo->Path.PktLifeTime = pCepPath->AckTimeout-1;
	else
		pPathInfo->Path.PktLifeTime = 0;
	pPathInfo->Path.PktLifeTimeSelector = IB_SELECTOR_EQ;
}

// compare Path information from a LoadAltPath to the given
// CEP Path (can be pCEP->PrimaryPath or pCEP->AlternatePath)
// Context of path assumes Local is Source
// we focus on parameters which truely indicate a different path
// returns 0 on match, 1 on no match
int CompareCepPathToPathInfo(IN const CM_CEP_PATH *pCepPath,
				IN const CM_CEP_PATHINFO *pPathInfo)
{
	if (pCepPath->LocalLID != pPathInfo->Path.SLID)
		return 1;
	if (pCepPath->RemoteLID != pPathInfo->Path.DLID)
		return 1;
	// we must test GIDs, if ports are on different fabrics, the LIDs could
	// match but the GIDs would be different
	if (0 != MemoryCompare(&pCepPath->LocalGID, &pPathInfo->Path.SGID, sizeof(IB_GID)))
		return 1;
	if (0 != MemoryCompare(&pCepPath->RemoteGID, &pPathInfo->Path.DGID, sizeof(IB_GID)))
		return 1;
	if (pCepPath->bSubnetLocal != pPathInfo->bSubnetLocal)
		return 1;

	// ignore minor parameters
	//pCepPath->LocalPortGuid - implied by LocalGID
	//pCepPath->FlowLabel = pPathInfo->Path.u1.s.FlowLabel;
	//pCepPath->StaticRate = pPathInfo->Path.Rate;
	//pCepPath->PkeyIndex = index of pPathInfo->Path.P_Key;
	//pCepPath->TClass = pPathInfo->Path.TClass;
	//pCepPath->SL = pPathInfo->Path.u2.s.SL;
	// pCepPath->bSubnetLocal = (! IsGlobalRoute(&pPathInfo->Path));
	//pCepPath->AckTimeout = AckTimeout;
	//pCepPath->HopLimit = pPathInfo->Path.u1.s.HopLimit;
	//pCepPath->LocalGidIndex - implied by LocalGID
	return 0;
}

//
// build QP Attributes to move RC/UC/RD QP from Reset to Init
//
void GetInitAttrFromCep(IN CM_CEP_OBJECT* pCEP,
					OUT IB_QP_ATTRIBUTES_MODIFY* QpAttrInit)
{
	QpAttrInit->RequestState = QPStateInit;
	QpAttrInit->Attrs = (IB_QP_ATTR_PORTGUID|IB_QP_ATTR_PKEYINDEX);
	QpAttrInit->PortGUID = pCEP->PrimaryPath.LocalPortGuid;
	QpAttrInit->PkeyIndex = pCEP->PrimaryPath.PkeyIndex;
	// application must build AccessControl
	// application must iba_modify_qp(...)
}


//
// build QP Attributes to move RC/UC/RD QP from Init to RTR
//
void GetRtrAttrFromCep(IN CM_CEP_OBJECT* pCEP, boolean enableFailover,
						OUT IB_QP_ATTRIBUTES_MODIFY* QpAttrRtr)
{
	// TBD - RD code here is not complete/accurate
	QpAttrRtr->RequestState = QPStateReadyToRecv;
	QpAttrRtr->RecvPSN = pCEP->LocalRecvPSN;
	QpAttrRtr->DestQPNumber = pCEP->RemoteEndPoint.QPN;
	GetAVFromCepPath(&pCEP->PrimaryPath, &QpAttrRtr->DestAV);
	QpAttrRtr->PathMTU = pCEP->Mtu;
	QpAttrRtr->Attrs = (IB_QP_ATTR_RECVPSN | IB_QP_ATTR_DESTQPNUMBER
						| IB_QP_ATTR_DESTAV | IB_QP_ATTR_PATHMTU);
	if (pCEP->Type == CM_RC_TYPE) {
		QpAttrRtr->ResponderResources = pCEP->LocalResponderResources;
		QpAttrRtr->Attrs |= IB_QP_ATTR_RESPONDERRESOURCES;
	}
	if (enableFailover && pCEP->AlternatePath.RemoteLID != 0)
	{
		GetAVFromCepPath(&pCEP->AlternatePath, &QpAttrRtr->AltDestAV);
		QpAttrRtr->AltPortGUID = pCEP->AlternatePath.LocalPortGuid;
		QpAttrRtr->AltPkeyIndex = pCEP->AlternatePath.PkeyIndex;
		QpAttrRtr->Attrs |= IB_QP_ATTR_ALTDESTAV|IB_QP_ATTR_ALTPORTGUID
						| IB_QP_ATTR_ALTPKEYINDEX;
	}
	// application must set
	//QpAttr->MinRnrTimer
	//QpAttr->Attrs |= IB_QP_ATTR_MINRNRTIMER;
	// application must iba_modify_qp(...)
}

//
// build QP Attributes to move RC/UC/RD QP from RTR to RTS
//
void GetRtsAttrFromCep(IN CM_CEP_OBJECT* pCEP, 
						OUT IB_QP_ATTRIBUTES_MODIFY* QpAttrRts)
{
	// TBD - RD code here is not complete/accurate
	QpAttrRts->RequestState = QPStateReadyToSend;
	QpAttrRts->SendPSN = pCEP->LocalSendPSN;
	QpAttrRts->Attrs = IB_QP_ATTR_SENDPSN;
	if (pCEP->Type != CM_UC_TYPE) {
		QpAttrRts->FlowControl = TRUE;	// always set
		QpAttrRts->InitiatorDepth = pCEP->LocalInitiatorDepth;
		QpAttrRts->LocalAckTimeout = pCEP->PrimaryPath.LocalAckTimeout;
		QpAttrRts->RetryCount = pCEP->LocalRetryCount;
		QpAttrRts->RnrRetryCount = pCEP->LocalRnrRetryCount;
		QpAttrRts->Attrs |= (IB_QP_ATTR_INITIATORDEPTH
						| IB_QP_ATTR_FLOWCONTROL | IB_QP_ATTR_LOCALACKTIMEOUT
						| IB_QP_ATTR_RETRYCOUNT | IB_QP_ATTR_RNRRETRYCOUNT);
	}
	if (pCEP->bFailoverSupported && pCEP->AlternatePath.RemoteLID != 0)
	{
		QpAttrRts->APMState = APMStateRearm;
		QpAttrRts->Attrs |= IB_QP_ATTR_APMSTATE;
		if (pCEP->Type != CM_UC_TYPE) {
			QpAttrRts->AltLocalAckTimeout = pCEP->AlternatePath.LocalAckTimeout;
			QpAttrRts->Attrs |= IB_QP_ATTR_ALTLOCALACKTIMEOUT;
		}
	}
}

//
// Recalculate the processing time
//
uint64
UpdateTurnaroundTime(uint64 turnaround_us, uint64 elapsed_us)
{
	uint32 turnaround32_us=(uint32)turnaround_us;
	uint32 elapsed32_us=(uint32)elapsed_us;
	uint64 new_turnaround_us=0;

	// Handle max range
	if (turnaround_us >= 0xFFFFFFFF)
		return (uint64)0xFFFFFFFF;

	if (elapsed_us >= 0xFFFFFFFF)
		return turnaround_us;

	new_turnaround_us = ((103 * turnaround32_us) + (25 * elapsed32_us)) /128;
#if 0
	// alternate heuristic, drive up faster than down
	if (turnaround32_us > elapsed32_us)
	{
		// drive value down slowly, we could have some fast consumers
		new_turnaround_us = turnaround32_us - (turnaround32_us - elapsed32_us)/32;
		//printk("elapsed %"PRIu64" drop from %"PRIu64" to %"PRIu64"\n", elapsed_us, turnaround_us, new_turnaround_us);
	} else {
		// drive value up more quickly, avoid 64 bit divide, not in kernel
		new_turnaround_us = ((103 * turnaround32_us) + (25 * elapsed32_us)) /128;
		//printk("elapsed %"PRIu64" grow from %"PRIu64" to %"PRIu64"\n", elapsed_us, turnaround_us, new_turnaround_us);
	}
#endif

	if (new_turnaround_us < gCmParams.MinTurnaroundTimeMs*1000)
		new_turnaround_us = gCmParams.MinTurnaroundTimeMs*1000;
	else if (new_turnaround_us > gCmParams.MaxTurnaroundTimeMs*1000)
		new_turnaround_us = gCmParams.MaxTurnaroundTimeMs*1000;

	return new_turnaround_us;
}

// Compute worst case service time for a CEP, used as part of
// computing service timeout for MRA
uint32 CepWorstServiceTimeMult(IN CM_CEP_OBJECT *pCEP)
{
	uint32 result;

	// service time is a local concept and does not include fabric time
	if (pCEP->turnaround_time_us)
	{
		// appl supplied
		result = TimeoutTimeToMult(pCEP->turnaround_time_us );
	} else {
		// assume worst case is 8* our average turnaround
		result = TimeoutTimeToMult( (gCM->turnaround_time_us*8) );
	}
	return result;
}

FSTATUS
RequestCheck(const CM_REQUEST_INFO* pRequest)
{
	// TODO: Verify SID
	return FSUCCESS;

} // RequestCheck()

// assign/re-assign a unique LocalCommID to the given CEP
// must be called with ListLatch held
void AssignLocalCommID(IN CM_CEP_OBJECT* pCEP)
{
	if (pCEP->LocalCommID)
	{
		// we are re-assigning
		CM_MapRemoveEntry(&gCM->LocalCommIDMap, &pCEP->LocalCommIDMapEntry, LOCAL_COMM_ID_LIST, pCEP);
	}
	// usually we won't see duplicates, but just in case we loop
	// since we will run out of a lot of other resources before we have
	// 4 billion CEPs, the loop is guaranteed to end
	do {
		pCEP->LocalCommID = ++(gCM->CommID);
	} while ( pCEP->LocalCommID == 0 
			  || ! CM_MapTryInsert(&gCM->LocalCommIDMap, pCEP->LocalCommID,
			  			&pCEP->LocalCommIDMapEntry,
						"LOCAL_COMM_ID_LIST", pCEP));
}

// unassign/clear LocalCommID for the given CEP
// must be called with ListLatch held
void ClearLocalCommID(IN CM_CEP_OBJECT* pCEP)
{
	if (pCEP->LocalCommID)
	{
		CM_MapRemoveEntry(&gCM->LocalCommIDMap, &pCEP->LocalCommIDMapEntry, LOCAL_COMM_ID_LIST, pCEP);
		pCEP->LocalCommID = 0;
	}
}

CM_CEP_OBJECT* FindLocalCommID(IN uint32 LocalCommID)
{
	CM_MAP_ITEM* pMapEntry = CM_MapGet(&gCM->LocalCommIDMap, (uint64)LocalCommID);
	if (pMapEntry != CM_MapEnd(&gCM->LocalCommIDMap))
		return PARENT_STRUCT(pMapEntry, CM_CEP_OBJECT, LocalCommIDMapEntry);
	else
		return NULL;
}

uint32 GenerateStartPSN()
{
	// a simple pseudo random number generator
	gCM->StartPSN = (gCM->StartPSN+1025) & 0xffffff;
	return gCM->StartPSN;
}

// LocalEndPointMap Key Compare function
//
// CepLocalEndPointCompare - is used to insert cep entries into the LocalEndPointMap and
// is the primary key_compare function for the LocalEndPointMap
//
// Coallating order is:
// Local QPN
// Local EECN
// Local CaGUID

// A qmap key_compare function to compare the end point address for
// two CEPs
//
// key1 - CEP1 pointer
// key2 - CEP2 pointer
//
// Returns:
// -1:	cep1 end point address < cep2 end point address
//	0:	cep1 end point address = cep2 end point address
//	1:	cep1 end point address > cep2 end point address
int
CepLocalEndPointCompare(uint64 key1, uint64 key2)
{
	IN CM_CEP_OBJECT* pCEP1 = (CM_CEP_OBJECT*)(uintn)key1;
	IN CM_CEP_OBJECT* pCEP2 = (CM_CEP_OBJECT*)(uintn)key2;

	if (pCEP1->LocalEndPoint.QPN < pCEP2->LocalEndPoint.QPN)
		return -1;
	else if (pCEP1->LocalEndPoint.QPN > pCEP2->LocalEndPoint.QPN)
		return 1;
	if (pCEP1->LocalEndPoint.EECN < pCEP2->LocalEndPoint.EECN)
		return -1;
	else if (pCEP1->LocalEndPoint.EECN > pCEP2->LocalEndPoint.EECN)
		return 1;
	if (pCEP1->LocalEndPoint.CaGUID < pCEP2->LocalEndPoint.CaGUID)
		return -1;
	else if (pCEP1->LocalEndPoint.CaGUID > pCEP2->LocalEndPoint.CaGUID)
		return 1;
	else
		return 0;
}

// A qmap key_compare function to search the LocalEndPointMap for a match with
// a given EndPoint
//
// key1 - CEP pointer
// key2 - CM_END_POINT pointer
//
// Returns:
// -1:	cep1 remote address < req local address
//	0:	cep1 remote address = req local address (accounting for wildcards)
//	1:	cep1 remote address > req local address
//
int
CepAddrLocalEndPointCompare(uint64 key1, uint64 key2)
{
	IN CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)(uintn)key1;
	IN CM_END_POINT* pEndPoint = (CM_END_POINT*)(uintn)key2;

	if (pCEP->LocalEndPoint.QPN < pEndPoint->QPN)
		return -1;
	else if (pCEP->LocalEndPoint.QPN > pEndPoint->QPN)
		return 1;
	if (pCEP->LocalEndPoint.EECN < pEndPoint->EECN)
		return -1;
	else if (pCEP->LocalEndPoint.EECN > pEndPoint->EECN)
		return 1;
	if (pCEP->LocalEndPoint.CaGUID < pEndPoint->CaGUID)
		return -1;
	else if (pCEP->LocalEndPoint.CaGUID > pEndPoint->CaGUID)
		return 1;
	else
		return 0;
}

// RemoteEndPointMap Key Compare function
//
// CepRemoteEndPointCompare - is used to insert cep entries into the RemoteEndPointMap and
// is the primary key_compare function for the RemoteEndPointMap
//
// Coallating order is:
// Remote QPN
// Remote EECN
// Remote CaGUID

// A qmap key_compare function to compare the end point address for
// two CEPs
//
// key1 - CEP1 pointer
// key2 - CEP2 pointer
//
// Returns:
// -1:	cep1 end point address < cep2 end point address
//	0:	cep1 end point address = cep2 end point address
//	1:	cep1 end point address > cep2 end point address
int
CepRemoteEndPointCompare(uint64 key1, uint64 key2)
{
	IN CM_CEP_OBJECT* pCEP1 = (CM_CEP_OBJECT*)(uintn)key1;
	IN CM_CEP_OBJECT* pCEP2 = (CM_CEP_OBJECT*)(uintn)key2;

	if (pCEP1->RemoteEndPoint.QPN < pCEP2->RemoteEndPoint.QPN)
		return -1;
	else if (pCEP1->RemoteEndPoint.QPN > pCEP2->RemoteEndPoint.QPN)
		return 1;
	if (pCEP1->RemoteEndPoint.EECN < pCEP2->RemoteEndPoint.EECN)
		return -1;
	else if (pCEP1->RemoteEndPoint.EECN > pCEP2->RemoteEndPoint.EECN)
		return 1;
	if (pCEP1->RemoteEndPoint.CaGUID < pCEP2->RemoteEndPoint.CaGUID)
		return -1;
	else if (pCEP1->RemoteEndPoint.CaGUID > pCEP2->RemoteEndPoint.CaGUID)
		return 1;
	else
		return 0;
}

// A qmap key_compare function to search the RemoteEndPointMap for a match with
// a given REQ
//
// key1 - CEP pointer
// key2 - REQ pointer
//
// Returns:
// -1:	cep1 remote address < req local address
//	0:	cep1 remote address = req local address (accounting for wildcards)
//	1:	cep1 remote address > req local address
//
int
CepReqRemoteEndPointCompare(uint64 key1, uint64 key2)
{
	IN CM_CEP_OBJECT* pCEP = (CM_CEP_OBJECT*)(uintn)key1;
	IN CMM_REQ* pREQ = (CMM_REQ*)(uintn)key2;

	if (pCEP->RemoteEndPoint.QPN < pREQ->u1.s.LocalQPN)
		return -1;
	else if (pCEP->RemoteEndPoint.QPN > pREQ->u1.s.LocalQPN)
		return 1;
	if (pCEP->RemoteEndPoint.EECN < pREQ->u2.s.LocalEECN)
		return -1;
	else if (pCEP->RemoteEndPoint.EECN > pREQ->u2.s.LocalEECN)
		return 1;
	if (pCEP->RemoteEndPoint.CaGUID < pREQ->LocalCaGUID)
		return -1;
	else if (pCEP->RemoteEndPoint.CaGUID > pREQ->LocalCaGUID)
		return 1;
	else
		return 0;
}

// unassign/clear Local and Remote EndPoints for the given CEP
// must be called with ListLatch held
void ClearEndPoint(IN CM_CEP_OBJECT* pCEP)
{
	if (pCEP->LocalEndPoint.QPN || pCEP->LocalEndPoint.EECN)
	{
		// SIDRRegister sets QPN but does not put on LocalEndPointMap
		if (pCEP->State != CMS_REGISTERED)
		{
			// If RC CM_REQ is rejected, sometimes local end point is not in map.
			if (!CM_MapIsEmpty(&gCM->LocalEndPointMap))
				CM_MapRemoveEntry(&gCM->LocalEndPointMap, &pCEP->LocalEndPointMapEntry, LOCAL_END_POINT_LIST, pCEP);
		}
		pCEP->LocalEndPoint.QPN = pCEP->LocalEndPoint.EECN = 0;
		pCEP->LocalEndPoint.CaGUID = 0;
	}
	if (pCEP->RemoteEndPoint.CaGUID)
	{
		CM_MapRemoveEntry(&gCM->RemoteEndPointMap, &pCEP->RemoteEndPointMapEntry, REMOTE_END_POINT_LIST, pCEP);
		pCEP->RemoteEndPoint.QPN = pCEP->RemoteEndPoint.EECN = 0;
		pCEP->RemoteEndPoint.CaGUID = 0;
	}
}

// CmGetStateString
//
// Returns an ascii string associated with the state
//
const char*
CmGetStateString(CM_CEP_STATE state)
{
	switch(state)
	{
		case CMS_IDLE:
			return "CMS_IDLE";
		case CMS_LISTEN:
			return "CMS_LISTEN";					
		case CMS_REQ_SENT:
			return "CMS_REQ_SENT";
		case CMS_REQ_RCVD:
			return "CMS_REQ_RCVD";
		case CMS_REP_SENT:
			return "CMS_REP_SENT";
		case CMS_REP_RCVD:
			return "CMS_REP_RCVD";
		case CMS_MRA_REQ_SENT:
			return "CMS_MRA_REQ_SENT";
		case CMS_MRA_REP_SENT:
			return "CMS_MRA_REP_SENT";
		case CMS_REP_WAIT:
			return "CMS_REP_WAIT";
		case CMS_MRA_REP_RCVD:
			return "CMS_MRA_REP_RCVD";
		case CMS_LAP_SENT:
			return "CMS_LAP_SENT";
		case CMS_MRA_LAP_SENT:
			return "CMS_MRA_LAP_SENT";
		case CMS_LAP_RCVD:
			return "CMS_LAP_RCVD";
		case CMS_MRA_LAP_RCVD:
			return "CMS_MRA_LAP_RCVD";
		case CMS_ESTABLISHED:
			return "CMS_ESTABLISHED";
		case CMS_DREQ_SENT:
			return "CMS_DREQ_SENT";
		case CMS_DREQ_RCVD:
			return "CMS_DREQ_RCVD";
		case CMS_SIM_DREQ_RCVD:
			return "CMS_SIM_DREQ_RCVD";
		case CMS_TIMEWAIT:
			return "CMS_TIMEWAIT";
		case CMS_REGISTERED:
			return "CMS_REGISTERED";
		case CMS_SIDR_REQ_SENT:
			return "CMS_SIDR_REQ_SENT";
		case CMS_SIDR_REQ_RCVD:
			return "CMS_SIDR_REQ_RCVD";
		//case CMS_SIDR_RESP_SENT:
		//	return "CMS_SIDR_RESP_SENT";
		default:
			return "Unknown State";
	}
}
