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


#include "gsamain.h"


#define MAX_TID_MAPPINGS	1000000


//
// Globals
//


//
// Debug params
//
#ifdef ICS_LOGGING
_IB_DBG_PARAM_BLOCK((_DBG_LVL_FATAL|_DBG_LVL_ERROR), _DBG_BREAK_ENABLE, GSA_MEM_TAG, "Gsa", MOD_GSA, Gsa);
#else
_IB_DBG_PARAM_BLOCK((_DBG_LVL_FATAL|_DBG_LVL_ERROR), _DBG_BREAK_ENABLE, GSA_MEM_TAG, "Gsa");
#endif

//
// Global Info
//
GSA_GLOBAL_INFO  *g_GsaGlobalInfo=NULL;

//
// Stored settings
//

GSA_GLOBAL_CONFIG_PARAMETERS	g_GsaSettings = GSA_DEFAULT_SETTINGS;


//
// GsaLoad
//
// This routine is called by the VCA Init when the driver is first loaded. 
// This routine is to create and initialize driver data structures.
//
// INPUTS:
//
//	ComponentInfo - Component Info structure to common driver entry points.
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - Indicates a successful load.
//

FSTATUS
GSALoad(
	IN IBT_COMPONENT_INFO		*ComponentInfo
	)
{
	_DBG_ENTER_LVL (_DBG_LVL_MAIN, GSALoad);
	_DBG_INIT;
	
#if !defined(VXWORKS)
	_DBG_PRINT (_DBG_LVL_MAIN,  
		(" InfiniBand General Services Agent. Built %s %s\n",\
		__DATE__, __TIME__ ));
#else
	_DBG_PRINT (_DBG_LVL_MAIN,  
		(" InfiniBand General Services Agent. Built %s %s\n",\
		_DBG_PTR(__DATE__), _DBG_PTR(__TIME__) ));
#endif

	//
	// Establish dispatch entry points for the functions supported.
	//
	MemoryClear( ComponentInfo, sizeof(IBT_COMPONENT_INFO) );
	
	ComponentInfo->AddDevice	= GsaAddDevice;
	ComponentInfo->RemoveDevice = GsaRemoveDevice;
	ComponentInfo->Unload		= GSAUnload;

	GsaOsComponentInfo( ComponentInfo );	// Fill in OS specific entrypoints

	//
	// This function is called to initialize the GSA subsystem. This must be 
	// called only once and if the function does not return success, the 
	// caller must unload the component containing GSA or else not use the 
	// GSA functionality since GSA will not be available.
	//
	// We are being asked to do a one-time initialization. Here we initialize 
	// the list of channel adapters as well as initialize the list of service 
	// classes as well as initialize the spin locks guarding these lists.
	//

	//
	// Allocate space for Global data
	//
	g_GsaGlobalInfo = (GSA_GLOBAL_INFO*)MemoryAllocateAndClear(sizeof(GSA_GLOBAL_INFO), FALSE, GSA_MEM_TAG);
	if( NULL == g_GsaGlobalInfo )
	{
		_DBG_ERROR(( "Not enough memory!\n" ));
		_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
		return FINSUFFICIENT_RESOURCES;
	}

	//
	// Initialize the state of the global info.
	//
	SpinLockInitState( &g_GsaGlobalInfo->CaListLock );
	SpinLockInitState( &g_GsaGlobalInfo->ServiceClassListLock );
	SpinLockInitState( &g_GsaGlobalInfo->SARSendListLock );
	SpinLockInitState( &g_GsaGlobalInfo->SARRecvListLock );

	QListInitState( &g_GsaGlobalInfo->ServiceClassList );
	QListInitState( &g_GsaGlobalInfo->SARSendList );
	QListInitState( &g_GsaGlobalInfo->SARRecvList );

	//
	// Initialize the global locks.
	//
	if( !SpinLockInit( &g_GsaGlobalInfo->CaListLock )			||
		!SpinLockInit( &g_GsaGlobalInfo->ServiceClassListLock ) ||
		!SpinLockInit( &g_GsaGlobalInfo->SARSendListLock )		||
		!SpinLockInit( &g_GsaGlobalInfo->SARRecvListLock ) )
	{
		_DBG_ERROR(( "Unable to initialize spin locks!\n" ));
		GSAUnload();
		_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
		return FERROR;
	}

	if (FSUCCESS != CreateGlobalRecvQ())
	{
		_DBG_ERROR(( "Unable to initialize global recvQ!\n" ));
		GSAUnload();
		_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
		return FERROR;
	}

	//
	// initialize global data
	//

	//
	// Initialize global lists.
	//
	if ((QListInit( &g_GsaGlobalInfo->ServiceClassList ) == FALSE) ||
		(QListInit( &g_GsaGlobalInfo->SARSendList ) == FALSE) ||
		(QListInit( &g_GsaGlobalInfo->SARRecvList ) == FALSE))
	{
		_DBG_ERROR(( "Failed to Initialize Global Lists!\n"));
		GSAUnload();
		_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
		return FERROR;
	}

	g_GsaGlobalInfo->ClientId = 0;

	//
	// We are assuming that the Subnet Manager or the partition manager 
	// portion of the subnet manager has configured the partition table 
	// of each port in each ca that wants to communicate using QP1
	// in a way that the intersection of the partition tables of these 
	// ports is non-empty, which means that there is at least one common 
	// partition key across all ports.
	//

	//
	// Read Global settings in a OS specific calls
	//
	GsaInitGlobalSettings();

	_TRC_REGISTER();

/*
	//
	// register for notifications
	//
	if( FSUCCESS != IbtRegisterNotify (
						GsaNotifyCallback,
						NULL,				// Context
						IB_NOTIFY_PORT_DOWN | IB_NOTIFY_LID_EVENT
							| IB_NOTIFY_ON_REGISTER,
						&g_GsaGlobalInfo->NotifyHandle ))
	{
		_DBG_ERROR(("System unstable!!! "
			"Could not hook for Notifications\n"));

	}
*/

	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
    return FSUCCESS;
}





//
// GSAUnload
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
//	None
//
//
void
GSAUnload(void)
{
	_DBG_ENTER_LVL(_DBG_LVL_MAIN, GSAUnload);

#if 0
	// remove Notification Handle
	if ( g_GsaGlobalInfo->NotifyHandle )
		IbtDeregisterNotify( g_GsaGlobalInfo->NotifyHandle );
#endif

	_TRC_UNREGISTER();

	// Clean up the global information if it exists.
	if( !g_GsaGlobalInfo )
	{
		_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
		return;
	}

	DestroySARContexts();

	// Empty Global list
	if( g_GsaGlobalInfo->DgrmPoolRecvQ )
	{
		DestroyGlobalRecvQ();
	}

	// Remove user list
	QListDestroy( &g_GsaGlobalInfo->SARRecvList );
	QListDestroy( &g_GsaGlobalInfo->SARSendList );
	QListDestroy( &g_GsaGlobalInfo->ServiceClassList );

	// Locks
	SpinLockDestroy( &g_GsaGlobalInfo->SARRecvListLock );
	SpinLockDestroy( &g_GsaGlobalInfo->SARSendListLock );
	SpinLockDestroy( &g_GsaGlobalInfo->CaListLock );
	SpinLockDestroy( &g_GsaGlobalInfo->ServiceClassListLock );


	MemoryDeallocate( g_GsaGlobalInfo );
	g_GsaGlobalInfo = NULL;

	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
}




//
// GsaAddDevice
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
//

FSTATUS
GsaAddDevice(
	IN	EUI64					CaGuid,
	OUT void					**Context
	)
{
	FSTATUS						status = FSUCCESS;

	
	
	_DBG_ENTER_LVL(_DBG_LVL_MAIN, GsaAddDevice);


	//
	// Process this Channel Adapter. 
	//
    
	//
	// Build CA list is done by SMA here
	//

	//
	// Query Ca Info and pre post to receive queue
	//
	status = PrepareCaForReceives(CaGuid);


	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
      
    return status;
}


//
// GsaRemoveDevice
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
//

FSTATUS
GsaRemoveDevice(
		IN EUI64	CaGuid,
		IN void		*Context
		)
{

	FSTATUS				status = FSUCCESS;
	

	_DBG_ENTER_LVL(_DBG_LVL_MAIN, GsaRemoveDevice);


	//
	// Note:
	//	The Remove Device routine in SMA deregisters all memory held by GSA
	//	and hence does not need a seperate remove call.
	//	The SMA also deletes QPs and CQs associated with GSA
	//


	//
	// Rebuild CA list is done by SMA here
	//

	
	  	
	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
      
    return status;
}

