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

#include "smamain.h"

// Globals


// Debug params
#ifdef ICS_LOGGING
_IB_DBG_PARAM_BLOCK((_DBG_LVL_FATAL|_DBG_LVL_ERROR), _DBG_BREAK_ENABLE, SMA_MEM_TAG, "Sma", MOD_SMA, Sma);
#else
_IB_DBG_PARAM_BLOCK((_DBG_LVL_FATAL|_DBG_LVL_ERROR), _DBG_BREAK_ENABLE, SMA_MEM_TAG, "Sma");
#endif

// Global Info
SMA_GLOBAL_INFO	*g_Sma;

// Stored settings
SMA_STORED_SETTINGS g_Settings = DEFAULT_SMA_SETTINGS;


//
// SmaLoad
//
// This routine is called by the VCA Init when the driver is first loaded. 
// This routine is to create and initialize driver data structures.
//
// INPUTS:
//
//	ComponentInfo - Pointer to ComponentInfo that is OS specific
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - Indicates a successful load.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//

FSTATUS
SMALoad(
	IN IBT_COMPONENT_INFO	 *ComponentInfo
	)
{
	FSTATUS				status = FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_MAIN, SmaLoad);
	_DBG_INIT;
	
	_TRC_REGISTER();

#if !defined(VXWORKS)
	_DBG_PRINT(_DBG_LVL_MAIN,  
	(" InfiniBand Subnet Management Agent. Built %s %s\n",\
	__DATE__, __TIME__ ));
#else
	_DBG_PRINT(_DBG_LVL_MAIN,  
	(" InfiniBand Subnet Management Agent. Built %s %s\n",\
	_DBG_PTR(__DATE__), _DBG_PTR(__TIME__) ));
#endif

    // Establish dispatch entry points for the functions supported.
	MemoryClear( ComponentInfo, sizeof(IBT_COMPONENT_INFO) );
	
	ComponentInfo->AddDevice = SmaAddDevice;
	ComponentInfo->RemoveDevice = SmaRemoveDevice;
	ComponentInfo->Unload = SMAUnload;

    // Read Global settings for the driver which may be set in a 
	// OS specific way.
	SmaInitGlobalSettings();
	
	// Allocate space for Global data
	g_Sma = (SMA_GLOBAL_INFO*)MemoryAllocate2AndClear(sizeof(SMA_GLOBAL_INFO), IBA_MEM_FLAG_PREMPTABLE, SMA_MEM_TAG);
	if ( NULL == g_Sma )
	{
		_DBG_ERROR(("MemAlloc failed for g_Sma!\n"));
		goto done;
	}

	// initialize global data
	g_Sma->NumCa = 0;
	g_Sma->CaObj = NULL;
	g_Sma->WorkReqRecv = g_Sma->WorkReqSend = NULL;

	g_Sma->SmUserTbl = NULL;
	g_Sma->NumUser = 0;

	// SpinLockInitState( &g_Sma->Lock )
	// SpinLockInit( &g_Sma->Lock )

	// Init Storage area for MADs
	g_Sma->Bin.NumBlocks = 0;
	g_Sma->Bin.Head = g_Sma->Bin.Tail = NULL;
	g_Sma->Bin.MemList = NULL;
	g_Sma->Bin.CurrentIndex = 0;			// set start mem index

	// Locks
	SpinLockInitState( &g_Sma->CaListLock );
	SpinLockInit( &g_Sma->CaListLock );
	SpinLockInitState( &g_Sma->Bin.Lock );
	SpinLockInit( &g_Sma->Bin.Lock );
	MutexInitState( &g_Sma->Bin.Mutex );
	MutexInit( &g_Sma->Bin.Mutex );
	SpinLockInitState( &g_Sma->RQ.Lock );
	SpinLockInit( &g_Sma->RQ.Lock );
	
	// Init Global Ibt user group for notifications
	IbtInitNotifyGroup(NotifyIbtCallback);

	// Allocate memory for Global GRH since the SMA does not need a GRH.
	// This memory will automatically get mapped to all CA registrations.
	g_Sma->GlobalGrh = CreateGlobalMemList( 0, sizeof(IB_GRH), 0, FALSE );
	if ( NULL == g_Sma->GlobalGrh )
	{
		status = FINSUFFICIENT_RESOURCES;
		goto failmemlist;
	}
		
	g_Sma->GlobalGrh->VirtualAddr = MemoryAllocate2AndClear(sizeof(IB_GRH), IBA_MEM_FLAG_PREMPTABLE, SMA_MEM_TAG);
	if ( NULL == g_Sma->GlobalGrh->VirtualAddr )
	{
		status = FINSUFFICIENT_RESOURCES;
		goto failgrhvirt;
	}
	g_Sma->GlobalGrh->AccessControl.AsUINT16 = 0;
	g_Sma->GlobalGrh->AccessControl.s.LocalWrite = 1;
	g_Sma->GlobalGrh->CaMemIndex = 0;

	// Increment index for future allocations
	g_Sma->Bin.CurrentIndex++;

done:
	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
    return status;

failgrhvirt:
	MemoryDeallocate( g_Sma->GlobalGrh );
failmemlist:
	IbtDestroyNotifyGroup();
	SpinLockDestroy( &g_Sma->RQ.Lock );
	MutexDestroy( &g_Sma->Bin.Mutex );
	SpinLockDestroy( &g_Sma->Bin.Lock );
	SpinLockDestroy( &g_Sma->CaListLock );
	MemoryDeallocate( g_Sma );
	goto done;
}


//
// SmaUnload
//
//	This routine should release resources allocated in the SmaLoad function. 
//
// INPUTS:
//
//	None.
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - Successful unload of SMA.
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//
void
SMAUnload(void)
{
	GLOBAL_MEM_LIST		*pMemList;
	
	_DBG_ENTER_LVL(_DBG_LVL_MAIN, SmaUnload);

	// there should be no CAs at this point, all VPD must be unloaded
	// before IbAccess
	ASSERT(0 == g_Sma->NumCa);
	ASSERT(NULL == g_Sma->CaObj);

	// Free up global memory
	//
	//Note: Global Grh gets freed automatically here
	for (pMemList = g_Sma->Bin.MemList; NULL != pMemList; )
	{
		GLOBAL_MEM_LIST	*pMemListNext;
		pMemListNext = (GLOBAL_MEM_LIST*)pMemList->Next;
		MemoryDeallocate( pMemList->VirtualAddr );	// registered SMP emm
		if (pMemList->HdrAddr )
			MemoryDeallocate( pMemList->HdrAddr );	// SmpBlock memory
		MemoryDeallocate( pMemList );
		pMemList = pMemListNext;
	}

	_TRC_UNREGISTER();

	// Remove user list
	if ( NULL != g_Sma->SmUserTbl )
		MemoryDeallocate( g_Sma->SmUserTbl );

	IbtDestroyNotifyGroup();

	// Locks
	MutexDestroy( &g_Sma->Bin.Mutex );
	SpinLockDestroy( &g_Sma->Bin.Lock );
	SpinLockDestroy( &g_Sma->RQ.Lock );
	SpinLockDestroy( &g_Sma->CaListLock );
		
	MemoryDeallocate( g_Sma );

	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
}


//
// SmaAddDevice
//
// This routine is called by the VCA Startdevice loop for all driver libs. 
// The CA device that just started will be identified by its GUID.
//
// INPUTS:
//
//	CaGUID - GUID to the CA that was added and started.
//
// OUTPUTS:
//
//	None
//
// RETURNS:
//
//	FSTATUS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//

FSTATUS
SmaAddDevice(
	IN	EUI64			CaGuid,
	OUT void			**Context
	)
{
	FSTATUS				status = FSUCCESS;
	SMA_CA_OBJ_PRIVATE	*pCaObj=NULL;
	uint32				i;
	
	_DBG_ENTER_LVL(_DBG_LVL_MAIN, SmaAddDevice);

	pCaObj = (SMA_CA_OBJ_PRIVATE*)MemoryAllocateAndClear(sizeof(SMA_CA_OBJ_PRIVATE), FALSE, SMA_MEM_TAG);
	if ( NULL == pCaObj ) 
	{
		_DBG_ERROR(("MemAlloc failed for pCaObj/WorkReq!\n"));
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}

	pCaObj->CaPublic.CaGuid = CaGuid;
	pCaObj->CaPublic.SmaCaContext = pCaObj;

	EventThreadInitState(&pCaObj->SmaEventThread);
	if (! EventThreadCreate(&pCaObj->SmaEventThread, "Smi",
		THREAD_PRI_VERY_HIGH, SmaCompletionCallback, pCaObj))
	{
		_DBG_ERROR(("Sma Event Thread Create FAILED\n"));
		goto failsmathread;
	}
		
	EventThreadInitState(&pCaObj->GsaEventThread);
	if (! EventThreadCreate(&pCaObj->GsaEventThread, "Gsi",
		THREAD_PRI_VERY_HIGH, GsaCompletionThreadCallback, pCaObj))
	{
		_DBG_ERROR(("Gsa Event Thread Create FAILED\n"));
		goto failgsathread;
	}

	status = iba_open_ca( CaGuid, SmaGsaCompletionCallback,
				SmaAsyncEventCallback, pCaObj/*Context*/, &pCaObj->CaHandle);
	if ( FSUCCESS != status )
	{
		_DBG_ERROR(("OpenCA() failed!\n"));
		goto failopen;
	}
	status = SetCAInternal(pCaObj->CaHandle);
	if ( FSUCCESS != status )
	{
		_DBG_ERROR(("SetCAInternal() failed!\n"));
		goto failinternal;
	}

	// Reserve CQ/QP/PD...
	status = SmaDeviceReserve( pCaObj );
	if ( FSUCCESS != status )
	{
		goto failreserve;
	}
	// we hold the Mutex across Bind and addition to Ca list
	// so that CreateDgrmPool cannot race and cause double
	// registration of memory in new DgrmPool
	AcquireMemListMutex();
	status = SmaBindGlobalMem( pCaObj );
	if (status != FSUCCESS)
	{
		ReleaseMemListMutex();
		// TBD need to undo SmaDeviceReserve
		goto failreserve;
	}

	// lock

	// update global info
	SpinLockAcquire(&g_Sma->CaListLock);
	g_Sma->NumCa++;
	pCaObj->Next = g_Sma->CaObj;
	pCaObj->Prev = NULL;
	if ( NULL != g_Sma->CaObj )
		g_Sma->CaObj->Prev = pCaObj;
	g_Sma->CaObj = pCaObj;
	pCaObj->UseCount++;	// hold reference so we can use object below
	SpinLockRelease(&g_Sma->CaListLock);

	ReleaseMemListMutex();

	// QPConfig 
	status = SmaDeviceConfig( pCaObj, QPTypeSMI );
	status = SmaDeviceConfig( pCaObj, QPTypeGSI );

	// Create GSA data structures
	GsaUpdateCaList();

	// Preallocate Buffers based on per device.
	// If we know the amount of buffers we need per device and go ahead
	// and allocate them in advance, subnet sweeps will not allocate
	// any buffers in the speed path
	//
	// NOTE: This call can fail if there are no system resources
	SmaAllocToBin( g_Settings.PreAllocSMPsPerDevice, FALSE );
	
	// post some descriptors on recv side if port is up or something...
	for (i=0; i<pCaObj->CaPublic.Ports; i++)
	{
		status = iba_smi_post_recv( NULL, pCaObj, (uint8)(i+1), 
							g_Settings.MinSMPsToPostPerPort );	
	}

	// Set Port capabilities across all CA's
	SetPortCapabilities();

	// Rearm CQ to receive event notifications.
	// The next Work Completion written to the CQ 
	// will generate an event
	status = iba_rearm_cq( pCaObj->CqHandle, CQEventSelNextWC);						
	SpinLockAcquire(&g_Sma->CaListLock);
	pCaObj->UseCount--;
	SpinLockRelease(&g_Sma->CaListLock);

	// Bind 
		
	// unlock

	// notify users through callbacks!
	IbtNotifyGroup( CaGuid, IB_NOTIFY_CA_ADD );
	// TBD - failure above causes bad status return here, can confuse caller
	// also does not cleanup QPs, CQ, PD, MRs, buffers
	// SmaRemoveCaObj might solve, but not clear if safe to call it
	// need some peers to the SmaDeviceReserve and SmaDeviceConfig routines
	// to undo their effects

done:
	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
      
    return status;

failreserve:
failinternal:
	iba_close_ca( pCaObj->CaHandle );
failopen:
	EventThreadDestroy(&pCaObj->GsaEventThread);
failgsathread:
	EventThreadDestroy(&pCaObj->SmaEventThread);
failsmathread:
	MemoryDeallocate( pCaObj );
	goto done;
}


//
// SmaRemoveDevice
//
// This routine is called by the VCA Removedevice loop for all driver libs. 
// The CA device that was removed will be identified by its GUID.
//
// INPUTS:
//
//	CaGUID - GUID to the CA that was added and started.
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSTATUS.
//
// IRQL:
//
//	This routine is called at IRQL_PASSIVE_LEVEL.
//

FSTATUS
SmaRemoveDevice(
		IN EUI64	CaGuid,
		IN void		*Context
		)
{
	FSTATUS				status = FSUCCESS;
	SMA_CA_OBJ_PRIVATE	*pCaObj;
	
	_DBG_ENTER_LVL(_DBG_LVL_MAIN, SmaRemoveDevice);

	// notify users through callbacks
	// SM should stop access to all registered memory on this Ca
	IbtNotifyGroup( CaGuid, IB_NOTIFY_CA_REMOVE );
	
	// lock

	// find CaObj
	SpinLockAcquire(&g_Sma->CaListLock);
	for (pCaObj = g_Sma->CaObj; NULL != pCaObj; pCaObj = pCaObj->Next)
	{
		if ( CaGuid == pCaObj->CaPublic.CaGuid )
			break;
	}

	if ( NULL != pCaObj )
	{
		pCaObj->UseCount++;
		SpinLockRelease(&g_Sma->CaListLock);
		status = SmaRemoveCaObj( pCaObj );	// will release reference to pCaObj
	} else {
		SpinLockRelease(&g_Sma->CaListLock);
		_DBG_ERROR(("Could not find CA device!\n"));
	}

	// Rebuild CA list for GSA
	GsaUpdateCaList();

	// unlock
	
	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);

    return status;
}
