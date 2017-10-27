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

#include "bmamain.h"
#include "bmadebug.h"
#include "bma_provider.h"
#include "bspcommon/h/usrBootManager.h"
#include "bspcommon/ibml/h/icsApi.h"

//
// Debug params
//
#if ICS_LOGGING
_IB_DBG_PARAM_BLOCK(_DBG_LVL_ALL, _DBG_BREAK_ENABLE, BMA_TAG, "Bma", MOD_BMA, Bma);
#else
_IB_DBG_PARAM_BLOCK((_DBG_LVL_FATAL | _DBG_LVL_ERROR), _DBG_BREAK_ENABLE, BMA_TAG, "Bma");
#endif
 
//
// Global Info
//
BMA_GLOBAL_INFO  *g_BmaGlobalInfo;

//
// Stored settings
//
BMA_GLOBAL_CONFIG_PARAMETERS	g_BmaSettings = BMA_DEFAULT_SETTINGS;


//
// BmaLoad
//
// This routine is called by the VCA Init when the driver is first loaded. 
// This routine is to create and initialize the BMA agent global data structures.
//
// INPUTS:
//
//	ComponentInfo - Address of IBT_COMPONENT_INFO that needs to be updated
//	with this driver's specific handlers for add/remove of a channel adapter
//	device as well as the overall driver unload handler.
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
BmaLoad(
	IN IBT_COMPONENT_INFO		*ComponentInfo
	)
{
	FSTATUS						status = FSUCCESS;
	extern uint32 BmaDbg;
#ifdef IB_TRACE
	extern uint32 BmaTrace;
#endif


	_DBG_ENTER_LVL(_DBG_LVL_MAIN, BMALoad);
	_DBG_INIT;

	__DBG_LEVEL__ = BmaDbg;
#ifdef IB_TRACE
	__DBG_TRACE_LEVEL__ = BmaTrace;
#endif
#if defined(IB_DEBUG) || defined(DBG)
	MsgOut ("Bma:DebugFlags = 0x%8x\n", __DBG_LEVEL__);
#endif

#if defined(VXWORKS)
	_DBG_PRINT(_DBG_LVL_MAIN,  
	(" InfiniBand Baseboard Management Class agent. Built %s %s\n",\
	__DATE__, __TIME__ ));
#else
	_DBG_PRINT(_DBG_LVL_MAIN,  
	(" InfiniBand Baseboard Management Class agent. Built %s %s\n",\
	_DBG_PTR(__DATE__), _DBG_PTR(__TIME__) ));
#endif

	_TRC_REGISTER();

	//
    // Establish dispatch entry points for the functions supported.
	//

	MemoryClear( ComponentInfo, sizeof(IBT_COMPONENT_INFO) );

	ComponentInfo->AddDevice = BmaAddDevice;
	ComponentInfo->RemoveDevice = BmaRemoveDevice;
	ComponentInfo->Unload = BmaUnload;

	
	//
    // Read any registry parameters for the driver which may be present.
	//
	//BmaReadRegistryParameters();

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

	g_BmaGlobalInfo = (BMA_GLOBAL_INFO*)MemoryAllocateAndClear(
									 sizeof(BMA_GLOBAL_INFO), FALSE, BMA_TAG );
	if ( NULL != g_BmaGlobalInfo )
	{
	
		//
		// initialize global data
		//

		g_BmaGlobalInfo->binitsuccess = FALSE; 

		SpinLockInitState( &g_BmaGlobalInfo->DevListLock );
		if( !SpinLockInit( &g_BmaGlobalInfo->DevListLock ) )
		{
			_DBG_ERROR(( "Unable to initialize spin locks!\n" ));
			status = FINSUFFICIENT_RESOURCES;
			goto fail_lock;
		}

		//
		// Recv Queue and Thread
		//
		CmdThreadInitState(&g_BmaGlobalInfo->RecvThread);
		if ( ! CmdThreadCreate(&g_BmaGlobalInfo->RecvThread,
				THREAD_PRI_HIGH, "Bma", (CMD_THREAD_CALLBACK)BmaThreadCallback,
				(CMD_THREAD_CALLBACK)BmaFreeCallback, (void*)g_BmaGlobalInfo))
		{
			_DBG_ERROR(("BmaMain: Unable to Create RecvThread\n"));
			status = FINSUFFICIENT_RESOURCES;
			goto fail_thread;
		}
		

		// initialize this general service class manager with GSI
		status = iba_gsi_register_class(MCLASS_BM, 
									IB_BM_CLASS_VERSION,
									GSI_REGISTER_RESPONDER,	// Register as a Responder
									FALSE,					// No SAR cap needed
									g_BmaGlobalInfo,		// Context
									BmaSendCallback,
									BmaRecvCallback,
									&g_BmaGlobalInfo->GsaHandle);
							
		if (status != FSUCCESS)
		{
			CmdThreadDestroy(&g_BmaGlobalInfo->RecvThread);
fail_thread:
			SpinLockDestroy( &g_BmaGlobalInfo->DevListLock );
fail_lock:
			MemoryDeallocate(g_BmaGlobalInfo);
			g_BmaGlobalInfo = NULL;

			_DBG_ERROR(("BmaMain: Unable to register with GSA!!! <status %d>\n", status));

			_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);

			return status;
		}
 
	}
	else
	{
		status = FINSUFFICIENT_RESOURCES;
		_DBG_PRINT(_DBG_LVL_MAIN,("Not enough memory!\n"));
	}

    
	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
    
    return status;
}


//
// BmaUnload
//
//	This routine should release resources allocated in the BmaLoad function.
//	Also, since their is a single instance of the service for all channel
//	adapters in the system, it will also release resources allocated in the
//	BmaAddDevice function. 
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
BmaUnload(void)
{

	_DBG_ENTER_LVL(_DBG_LVL_MAIN, BmaUnload);

	_TRC_UNREGISTER();

	if ( g_BmaGlobalInfo )
	{

		// Destroy the datagram pool
		if ( g_BmaGlobalInfo->DgrmPoolHandle )
			iba_gsi_destroy_dgrm_pool( g_BmaGlobalInfo->DgrmPoolHandle );

		// Deregister this class agent with the GSA	
		if ( g_BmaGlobalInfo->GsaHandle )
			iba_gsi_deregister_class(g_BmaGlobalInfo->GsaHandle);

		//
		// Recv Thread
		//
		CmdThreadDestroy(&g_BmaGlobalInfo->RecvThread);
				
		//
		// Ca List Members
		//
		while ( g_BmaGlobalInfo->DevList )
		{
			BmaRemoveDevice( g_BmaGlobalInfo->DevList->CaGuid, NULL );
		}

		//
		// Ca List Lock
		//
		SpinLockDestroy( &g_BmaGlobalInfo->DevListLock );


		//
		// Free up global memory
		//
		MemoryDeallocate( g_BmaGlobalInfo );
		g_BmaGlobalInfo = NULL;

	} // if g_BmaGlobalInfo

	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);
  

}


//////////////////////////////////////////////////////////////////////////
// BmaAddDevice()
//
// This routine is called once for every new device added.
//
// INPUTS:
//		CaGuid - the channel adapters GUID
//
// OUTPUTS:
//
//	None.  Although function is defined to return context value, this
//	value is not set since their is nothing specifically allocated for
//	each new channel adapter, so nothing is done in the BmaRemoveDevice
//	function that needs the context.
//
// RETURNS:
//
//	FSUCCESS - Indicates a successful initialization.
//
//
FSTATUS
BmaAddDevice(
	IN	EUI64			CaGuid,
	OUT void			**Context
	)
{
	FSTATUS			status = FSUCCESS;
	uint32			BufferSize;
	BMA_DEV_INFO	*pDevInfo=NULL;

	_DBG_ENTER_LVL(_DBG_LVL_MAIN, BmaAddDevice);

	ASSERT(g_BmaGlobalInfo);

	if (!g_BmaGlobalInfo->binitsuccess)
	{
		// Create the datagram pool
		BufferSize = sizeof(MAD);
		status = iba_gsi_create_dgrm_pool(g_BmaGlobalInfo->GsaHandle,
								g_BmaSettings.MaxNDgrm,
								1,
								&BufferSize,
								0, //1,
								&g_BmaGlobalInfo->DgrmPoolHandle
								);
		if (status != FSUCCESS)
		{
			// Cleanup
			iba_gsi_deregister_class(g_BmaGlobalInfo->GsaHandle);

			MemoryDeallocate(g_BmaGlobalInfo);

			g_BmaGlobalInfo = NULL;

			_DBG_ERROR(("Unable to create BMA datagram pool!!! <status %d>\n", status));
		}
		else
		{
			g_BmaGlobalInfo->binitsuccess = TRUE;
		}

	}

	if (g_BmaGlobalInfo->binitsuccess)
	{
		pDevInfo = (BMA_DEV_INFO*)MemoryAllocateAndClear(sizeof(BMA_DEV_INFO), FALSE, BMA_TAG );

		if (pDevInfo)
		{
			IB_CA_ATTRIBUTES CaAttributes;

			pDevInfo->CaGuid = CaGuid;
			status = iba_query_ca_by_guid_alloc(CaGuid, &CaAttributes);
			if ( status == FSUCCESS )
			{
				pDevInfo->NumPorts = CaAttributes.Ports;
				pDevInfo->Port = (BMA_PORT_INFO*)
					MemoryAllocateAndClear(sizeof(BMA_PORT_INFO)*pDevInfo->NumPorts, FALSE, BMA_TAG);
				if (pDevInfo->Port)
				{
					IB_PORT_ATTRIBUTES *pPortAttr;
					unsigned i;

					for ( i = 0, pPortAttr = CaAttributes.PortAttributesList;
						  i < pDevInfo->NumPorts;
						  i++, pPortAttr = pPortAttr->Next )
					{
						// Traps disabled by default, Trap settings are all zero
						// BKey, BKeyProtect, BKeyViolations, and BKeyLease all init to zero
						pDevInfo->Port[i].Guid = pPortAttr->GUID;
					}

					MemoryDeallocate(CaAttributes.PortAttributesList);

					// Success, add this CA to the list
					SpinLockAcquire( &g_BmaGlobalInfo->DevListLock );
					pDevInfo->Next = g_BmaGlobalInfo->DevList;
					g_BmaGlobalInfo->DevList = pDevInfo;
					SpinLockRelease( &g_BmaGlobalInfo->DevListLock );
				}
				else
				{
					_DBG_ERROR(("MemAlloc failed for Port List!\n"));
					status = FINSUFFICIENT_MEMORY;
				}
			}
			else
			{
				_DBG_ERROR(("CA Query Failed!\n"));
				MemoryDeallocate(pDevInfo);
			}
		}
		else
		{
			_DBG_ERROR(("MemAlloc failed for pDevInfo!\n"));
			status = FINSUFFICIENT_MEMORY;
		}
	}

	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);

	return status;

} // BmaAddDevice()


//////////////////////////////////////////////////////////////////////////
// BmaRemoveDevice()
//
// This routine is called once for every CA device being removed.  Nothing
//	is performed in this function.  All cleanup is performed in the BmaUnload
//	function.
//
// INPUTS:
//		CaGuid - the channel adapters GUID
//		Context - driver defined unique context for this CA
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - Indicates a successful initialization.
//
//
FSTATUS
BmaRemoveDevice(
	IN	EUI64			CaGuid,
	IN void				*Context
	)
{
	FSTATUS			status = FSUCCESS;
	BMA_DEV_INFO	*pDevInfo=NULL;

	_DBG_ENTER_LVL(_DBG_LVL_MAIN, BmaRemoveDevice);

	ASSERT(g_BmaGlobalInfo);

	SpinLockAcquire( &g_BmaGlobalInfo->DevListLock );
	pDevInfo = g_BmaGlobalInfo->DevList;
	if ( g_BmaGlobalInfo->DevList->CaGuid == CaGuid )
	{
		g_BmaGlobalInfo->DevList = pDevInfo->Next;
	}
	else
	{
		while ( pDevInfo->Next )
		{
			if ( pDevInfo->Next->CaGuid == CaGuid )
			{
				BMA_DEV_INFO *Found = pDevInfo->Next;
				pDevInfo->Next = Found->Next;
				pDevInfo = Found;
				break;
			}
			pDevInfo = pDevInfo->Next;
		}
	}
	SpinLockRelease( &g_BmaGlobalInfo->DevListLock );

	if (pDevInfo && pDevInfo->CaGuid == CaGuid)
	{
		MemoryDeallocate( pDevInfo->Port );
		MemoryDeallocate( pDevInfo );
	}
	else
	{
		_DBG_ERROR(("Unable to find Ca with GUID 0x%"PRIx64"\n", CaGuid));
		status = FERROR;
	}

	_DBG_LEAVE_LVL(_DBG_LVL_MAIN);

	return status;

} // BmaRemoveDevice()
