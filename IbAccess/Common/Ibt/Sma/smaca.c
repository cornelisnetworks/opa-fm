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
#include "gsi_params.h"

extern uint32 CmMaxDgrms;

FSTATUS
SmaDeviceReserve(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj
	)
{
	FSTATUS						status				= FSUCCESS;
	IB_CA_ATTRIBUTES			*pCaAttributes		= NULL;
	IB_PORT_ATTRIBUTES			*pPortAttributesList= NULL;
	SMA_PORT_TABLE_PRIV			*pPortTbl			= NULL;
	uint32						qp0CqSize			= 0;
	IB_PORT_ATTRIBUTES			*pTempPortAttributesList;
	IB_QP_ATTRIBUTES_CREATE		qpCreateAttributes;
	IB_QP_ATTRIBUTES_QUERY		qpAttributes;
	uint32						i=0;
	
	_DBG_ENTER_LVL(_DBG_LVL_DEVICE, DeviceReserve);

	// query the device attributes from VCA
	pCaAttributes = (IB_CA_ATTRIBUTES*)MemoryAllocateAndClear(sizeof(IB_CA_ATTRIBUTES), FALSE, SMA_MEM_TAG);
	if ( !pCaAttributes )
	{
		status = FINSUFFICIENT_MEMORY;
		_DBG_ERROR (("MemAlloc failed for CaAttributes!\n" ));
		goto failcaalloc;
	}			
	
	status = iba_query_ca(CaObj->CaHandle, pCaAttributes, NULL/*pContext*/);
	if ( FSUCCESS != status )
	{
		_DBG_ERROR ((
			"iba_query_ca() failed with NULL PortAttribs! :%s\n",
			_DBG_PTR(FSTATUS_MSG (status)) ));
		goto failquery;
	}		

	pPortAttributesList = (IB_PORT_ATTRIBUTES*)MemoryAllocateAndClear(pCaAttributes->PortAttributesListSize, FALSE, SMA_MEM_TAG);
	if ( !pPortAttributesList )
	{
		status = FINSUFFICIENT_MEMORY;
		_DBG_ERROR ((
			"MemAlloc failed for PortAttributesList!\n" ));
		goto failportalloc;
	}		

	_DBG_PRINT (_DBG_LVL_DEVICE,(
		"Processing device CA      :GUID(x%"PRIx64")\n",
		CaObj->CaPublic.CaGuid ));

	// build list and add Port info and tag other requirements
	pCaAttributes->PortAttributesList	= pPortAttributesList;
	pTempPortAttributesList				= pPortAttributesList; 

	status = iba_query_ca(CaObj->CaHandle, pCaAttributes, NULL/*pContext*/);
	if ( FSUCCESS != status )
	{
		_DBG_ERROR ((
				"iba_query_ca() failed for PortAttribs!:%s\n",
				_DBG_PTR(FSTATUS_MSG (status)) ));
		goto failquery2;
	}		
		
	pPortTbl = (SMA_PORT_TABLE_PRIV*)MemoryAllocateAndClear(sizeof(SMA_PORT_BLOCK_PRIV) * pCaAttributes->Ports, FALSE, SMA_MEM_TAG);
	if ( !pPortTbl )
	{
		status = FINSUFFICIENT_MEMORY;
		_DBG_ERROR (("MemAlloc failed for PortList!\n" ));
		goto failtblalloc;
	}	
		
	// Reserve device resources

	// Fill in Ca details
	CaObj->WorkReqId			 = 0;

	CaObj->Partitions			 = pCaAttributes->Partitions;
	CaObj->AddressVectors		 = pCaAttributes->AddressVectors;
	CaObj->CaPublic.Ports		 = pCaAttributes->Ports;
	CaObj->CaPublic.Capabilities = pCaAttributes->Capabilities;

	// add port tables to list
	CaObj->CaPublic.PortTbl 	 = (SMA_PORT_TABLE *)pPortTbl;

	// Allocate PD Resource
	status = iba_alloc_sqp_pd( 
						CaObj->CaHandle,
// BUGBUG - does not account for send buffers allocated by registering Gsa/Smi managers
// CM is likely to be the largest so we account for its Dgrms
// however for SMI and GSI the Recv buffers will not use AVs, so this is
// a reasonable guess
#ifdef BUILD_CM
						g_Settings.MaxSMPs +g_GsaSettings.MaxRecvBuffers
						+ CmMaxDgrms,
#else
						g_Settings.MaxSMPs +g_GsaSettings.MaxRecvBuffers,
#endif
						&CaObj->PdHandle );
	if ( FSUCCESS != status )
	{
		_DBG_ERROR (("AllocatePD() failed! :%s\n",
				_DBG_PTR(FSTATUS_MSG (status)) ));
		goto failpd;
	}	

	// Create One CQ per CA for SMA
	status = iba_create_cq(
						CaObj->CaHandle,
								// CqReqSize
						MIN(pCaAttributes->WorkCompletions,
								(CaObj->CaPublic.Ports	
						 		* (g_Settings.SendQDepth
									 + g_Settings.RecvQDepth))),
						(void *)QPTypeSMI,					// Context
						&CaObj->CqHandle,
						&CaObj->CqSize );
	if ( FSUCCESS != status )
	{
		_DBG_ERROR (( "iba_create_cq for SMA failed! :%s\n",
								_DBG_PTR(FSTATUS_MSG (status)) ));
		goto failcq;
	}

	// Create One CQ per CA for GSA
	status = iba_create_cq(
						CaObj->CaHandle,
								// CqReqSize
						MIN(pCaAttributes->WorkCompletions,
							(CaObj->CaPublic.Ports * (g_GsaSettings.SendQDepth+g_GsaSettings.RecvQDepth))),
						(void *)QPTypeGSI,					// Context
						&CaObj->Qp1CqHandle,
						&CaObj->Qp1CqSize );
	if ( FSUCCESS != status )
	{
		_DBG_ERROR (( "iba_create_cq for GSA failed! :%s\n",
						_DBG_PTR(FSTATUS_MSG (status)) ));
		goto failcq2;
	}

	// Get a Special QP each for SMA and GSA on each port
	for ( i=0; i< CaObj->CaPublic.Ports; i++ )
	{
		_DBG_PRINT(_DBG_LVL_DEVICE,(
				"Processing device port(%d) :GUID(x%"PRIx64")\n",
				i+1, 
				pTempPortAttributesList->GUID ));

		// Fill in per Port details
		pPortTbl->PortBlock[i].Public.PortNumber	= (uint8)i+1;
		pPortTbl->PortBlock[i].Public.GUID			= \
				pTempPortAttributesList->GUID;
		pPortTbl->PortBlock[i].Public.NumGIDs		= \
				pTempPortAttributesList->NumGIDs;
		pPortTbl->PortBlock[i].Public.NumPKeys		= \
				pTempPortAttributesList->NumPkeys;

		// fill in default Port state
		pPortTbl->PortBlock[i].Public.State			= \
				pTempPortAttributesList->PortState;

		// set QP attribs for SMA
		qpCreateAttributes.Type				= QPTypeSMI;
		qpCreateAttributes.PDHandle			= CaObj->PdHandle;
		qpCreateAttributes.SendCQHandle		= CaObj->CqHandle;
		qpCreateAttributes.RecvCQHandle		= CaObj->CqHandle;
			
		qpCreateAttributes.SendQDepth		= 
						MIN(pCaAttributes->WorkReqs, g_Settings.SendQDepth);
		qpCreateAttributes.RecvQDepth		= 
						MIN(pCaAttributes->WorkReqs, g_Settings.RecvQDepth);

		qpCreateAttributes.SendDSListDepth	= 1;
		qpCreateAttributes.RecvDSListDepth	= 2;
				
		qpCreateAttributes.CompletionFlags	= IB_COMPL_CNTL_SEND;
			
		status = iba_create_special_qp(
						CaObj->CaHandle,
						pPortTbl->PortBlock[i].Public.GUID,
						&qpCreateAttributes,
						&pPortTbl->PortBlock[i].QpHandle,	// Context
						&pPortTbl->PortBlock[i].QpHandle,
						&qpAttributes );
		if ( FSUCCESS != status )
		{
			_DBG_ERROR ((
					"CreateSpecialQP() failed on port(%d) for SMA :%s!\n",
					i+1,
					_DBG_PTR(FSTATUS_MSG (status)) ));
			goto failsmi;
		}

			
		// set QP attribs for GSA
		// Note: GSA QP created with fixed settings. Changes 
		//	to QP and CQ size will be done in GSA module if applicable
		qpCreateAttributes.Type				= QPTypeGSI;
		qpCreateAttributes.PDHandle			= CaObj->PdHandle;
		qpCreateAttributes.SendCQHandle		= CaObj->Qp1CqHandle;
		qpCreateAttributes.RecvCQHandle		= CaObj->Qp1CqHandle;

		qpCreateAttributes.RecvDSListDepth	= 2; //pCaAttributes->DSListDepth;
		qpCreateAttributes.SendDSListDepth	= 3; //pCaAttributes->DSListDepth;
		qpCreateAttributes.RecvQDepth		= 
						MIN(pCaAttributes->WorkReqs, g_GsaSettings.RecvQDepth);
		qpCreateAttributes.SendQDepth		= 
						MIN(pCaAttributes->WorkReqs, g_GsaSettings.SendQDepth);
			
		qpCreateAttributes.CompletionFlags = IB_COMPL_CNTL_SEND;
						
		status = iba_create_special_qp(
						CaObj->CaHandle,
						pPortTbl->PortBlock[i].Public.GUID,
						&qpCreateAttributes,
						&pPortTbl->PortBlock[i].Qp1Handle,	// Context
						&pPortTbl->PortBlock[i].Qp1Handle,
						&qpAttributes );
		if ( FSUCCESS != status )
		{
			_DBG_ERROR ((
					"CreateSpecialQP() failed on port(%d) for GSA :%s!\n",
					i+1,
					_DBG_PTR(FSTATUS_MSG (status)) ));
			goto failgsi;
		}

		// Spinlock for Qp0 & Qp1
		SpinLockInitState( &pPortTbl->PortBlock[i].Qp0Lock );
		SpinLockInitState( &pPortTbl->PortBlock[i].QpLock );

		if( !SpinLockInit( &pPortTbl->PortBlock[i].Qp0Lock ) )
		{
			_DBG_ERROR(( "Unable to initialize spinlocks for Port(%d)!\n", i+1 ));
			goto faillock0;
		}
		if ( !SpinLockInit( &pPortTbl->PortBlock[i].QpLock ) )
		{
			_DBG_ERROR(( "Unable to initialize spinlocks for Port(%d)!\n", i+1 ));
			goto faillock;
		}

		// update info
		pPortTbl->PortBlock[i].Qp0SendQDepth = \
					qpAttributes.SendQDepth;
		AtomicWrite(&pPortTbl->PortBlock[i].Qp0SendQTop,
					qpAttributes.SendQDepth);
		pPortTbl->PortBlock[i].Qp0RecvQDepth = \
					qpAttributes.RecvQDepth;
		AtomicWrite(&pPortTbl->PortBlock[i].Qp0RecvQTop,
					qpAttributes.RecvQDepth);
			
		// keep count of All QP depths
		qp0CqSize += pPortTbl->PortBlock[i].Qp0SendQDepth + \
						pPortTbl->PortBlock[i].Qp0RecvQDepth;

		// loop
		pTempPortAttributesList			= pTempPortAttributesList->Next;

	}	// i loop

	// add port tables to list
	CaObj->CaPublic.PortTbl				= (SMA_PORT_TABLE *)pPortTbl;

	// SMA Only: GSA resize of QP and CQ handled in GSA module
	// Resize CQ if necessary
	// Validate CQ size depth with all Qp depths
	_DBG_PRINT (_DBG_LVL_DEVICE, (
				"CqSize: \n"
				"\tStored.......:%d\n"
				"\tCalculated...:%d\n"
				"\tResize.......:%s\n",
				MIN(pCaAttributes->WorkCompletions, g_Settings.MaxCqSize),
				qp0CqSize,
				_DBG_PTR(qp0CqSize < MIN(pCaAttributes->WorkCompletions, g_Settings.MaxCqSize) ? "TRUE":"FALSE") ));

	// Update Cqsize only if we have allocated more than 
	// the h/w can achieve.
	if ( qp0CqSize < MIN(pCaAttributes->WorkCompletions, g_Settings.MaxCqSize))
	{
		status = iba_resize_cq(
					CaObj->CqHandle,
					MIN(pCaAttributes->WorkCompletions, g_Settings.MaxCqSize),
					&CaObj->CqSize );
		if (FSUCCESS != status)
		{
			_DBG_ERROR ((
						"iba_resize_cq() failed for SMA :%s!\n",
						_DBG_PTR(FSTATUS_MSG (status)) ));
			goto failresize;
		}
	}
	MemoryDeallocate ( pPortAttributesList );
	MemoryDeallocate ( pCaAttributes );

done:
	_DBG_LEAVE_LVL(_DBG_LVL_DEVICE);
    return status;

failresize:
	--i;
	SpinLockDestroy( &pPortTbl->PortBlock[i].QpLock );
faillock:
	SpinLockDestroy( &pPortTbl->PortBlock[i].Qp0Lock );
faillock0:
	iba_destroy_qp( pPortTbl->PortBlock[i].Qp1Handle );
failgsi:
	iba_destroy_qp( pPortTbl->PortBlock[i].QpHandle );
failsmi:
	while (i > 0)
	{
		--i;
		SpinLockDestroy( &pPortTbl->PortBlock[i].QpLock );
		SpinLockDestroy( &pPortTbl->PortBlock[i].Qp0Lock );
		iba_destroy_qp( pPortTbl->PortBlock[i].Qp1Handle );
		iba_destroy_qp( pPortTbl->PortBlock[i].QpHandle );
	}
	iba_destroy_cq( CaObj->Qp1CqHandle );
	CaObj->Qp1CqHandle = NULL;
failcq2:
	iba_destroy_cq( CaObj->CqHandle );
	CaObj->CqHandle = NULL;
failcq:
	iba_free_pd( CaObj->PdHandle );
	CaObj->PdHandle = NULL;
failpd:
	MemoryDeallocate ( pPortTbl );
	CaObj->CaPublic.PortTbl = NULL;
failtblalloc:
failquery2:
	MemoryDeallocate ( pPortAttributesList );
failportalloc:
failquery:
	MemoryDeallocate ( pCaAttributes );
failcaalloc:
	goto done;
}



FSTATUS
SmaConfigQp(
	IN	IB_HANDLE				QpHandle,
	IN	IB_QP_STATE				QpState,
	IN	IB_QP_TYPE				QpType
	)
{
	FSTATUS						status = FSUCCESS;
	IB_QP_ATTRIBUTES_MODIFY		qpModAttributes;
	
	
	_DBG_ENTER_LVL(_DBG_LVL_DEVICE, SmaConfigQp);


	// set QP attribs based on transition and QpType
	switch (QpState)
	{
	default:
		_DBG_ERROR(("Invalid QP State requested!\n"));
		status = FINVALID_SETTING;
		goto done;
		break;
	
	case QPStateInit:
		{
			qpModAttributes.RequestState	= QPStateInit;
			qpModAttributes.Attrs = (IB_QP_ATTR_PKEYINDEX
												| IB_QP_ATTR_QKEY);
			qpModAttributes.PkeyIndex = 0;	// BUGBUG not applicable
			if ( QPTypeSMI == QpType )
				qpModAttributes.Qkey = 0;
			else
				qpModAttributes.Qkey = QP1_WELL_KNOWN_Q_KEY;
		}
		break;
	
	case QPStateReadyToRecv:
		{
			qpModAttributes.RequestState = QPStateReadyToRecv;
			qpModAttributes.Attrs = 0;
	
		}
		break;

	case QPStateReadyToSend:
		{
			qpModAttributes.RequestState = QPStateReadyToSend;
			qpModAttributes.Attrs = IB_QP_ATTR_SENDPSN;
			qpModAttributes.SendPSN = 0x00000001;
		}
		break;
	}	// switch

	// transition QP states
	status = iba_modify_qp( QpHandle, &qpModAttributes, NULL );
	if ( FSUCCESS != status )
	{
		_DBG_ERROR(("Modify Qp failed!\n"));
	}
	
done:
	_DBG_LEAVE_LVL(_DBG_LVL_DEVICE);
    return status;
}



FSTATUS
SmaDeviceConfig(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	IB_QP_TYPE				QpType
	)
{
	FSTATUS						status = FSUCCESS;
	SMA_PORT_TABLE_PRIV			*pPortTbl;
	uint32						i;
			
	_DBG_ENTER_LVL(_DBG_LVL_DEVICE, SmaDeviceConfig);


	pPortTbl = (SMA_PORT_TABLE_PRIV *)CaObj->CaPublic.PortTbl;

	// Loop thru' all QP0's 
	for ( i=0; i< CaObj->CaPublic.Ports; i++ )
	{
		// set QP to Init state
		status = SmaConfigQp(
			(( QPTypeSMI == QpType ) ? \
			pPortTbl->PortBlock[i].QpHandle:pPortTbl->PortBlock[i].Qp1Handle),
			QPStateInit,
			QpType );
		if ( FSUCCESS != status )
		{
			_DBG_ERROR((
				"Could not transition QP%s to Init on port %d!\n", 
				_DBG_PTR((( QPTypeSMI == QpType ) ? "0":"1")),
				i+1));
		} else {
			// transition QP states

			// RecvQ to RTR
			status = SmaConfigQp(
						(( QPTypeSMI == QpType ) ? \
						pPortTbl->PortBlock[i].QpHandle : \
						pPortTbl->PortBlock[i].Qp1Handle),
						QPStateReadyToRecv,
						QpType );
			if ( FSUCCESS == status )
			{
				// SendQ to RTS
				status = SmaConfigQp(
							(( QPTypeSMI == QpType ) ? \
							pPortTbl->PortBlock[i].QpHandle : \
							pPortTbl->PortBlock[i].Qp1Handle),
							QPStateReadyToSend,
							QpType );
				if ( FSUCCESS != status )
				{
					// BUGBUG: return module error
					_DBG_ERROR((
						"Error transitioning QP%s on Port[%d] to RTS!\n", 
						_DBG_PTR((( QPTypeSMI == QpType ) ? "0":"1")),
						i+1));
				}
			} else {
				// BUGBUG: return module error
				_DBG_ERROR((
					"Error transitioning QP%s on Port[%d] to RTR!\n", 
					_DBG_PTR((( QPTypeSMI == QpType ) ? "0":"1")),
					i+1));
			}
		}		// modify QP to hold CQ
	}			// for Port loop

	_DBG_LEAVE_LVL(_DBG_LVL_DEVICE);
    return status;
}



FSTATUS
GetAV(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	IB_ADDRESS_VECTOR		*AvInfo,
	OUT	IB_HANDLE				*AvHandle
	)
{
	FSTATUS						status			= FSUCCESS;
	SMA_PORT_TABLE_PRIV			*pPortTbl;
			
	_DBG_ENTER_LVL(_DBG_LVL_DEVICE, GetAV);

	pPortTbl = (SMA_PORT_TABLE_PRIV *)CaObj->CaPublic.PortTbl;

	// Create AV if one does not exist
	_DBG_PRINT(_DBG_LVL_DEVICE,(
		"AV Info:\n"
		"Local:\n"
		"\tPortGUID.....:x%"PRIx64"\n"
		"\tPathBits.....:x%X\n"
		"\tServiceLevel.:x%X\n"
		"\tStaticRate...:x%X\n"
		"Remote:\n"
		"\tLID..........:x%X\n"
		"\tGlobalRoute..:%s\n",
		AvInfo->PortGUID,
		AvInfo->PathBits,
		AvInfo->ServiceLevel,
		AvInfo->StaticRate,
		AvInfo->DestLID,
		_DBG_PTR(AvInfo->GlobalRoute == TRUE	? "TRUE":"FALSE" )));

	// Add Global Routing Info if present
	if (AvInfo->GlobalRoute)
	{
		_DBG_PRINT(_DBG_LVL_DEVICE,(
			"Global Route Info:\n"
			"\tFlowLabel....:x%X\n"
			"\tHopLimit.....:x%X\n"
			"\tSrcGIDIndex..:x%X\n"
			"\tTrafficClass.:x%X\n",
			AvInfo->GlobalRouteInfo.FlowLabel,
			AvInfo->GlobalRouteInfo.HopLimit,
			AvInfo->GlobalRouteInfo.SrcGIDIndex,
			AvInfo->GlobalRouteInfo.TrafficClass ));
	}

	status = iba_create_av( CaObj->CaHandle, CaObj->PdHandle, AvInfo, AvHandle);

	_DBG_LEAVE_LVL(_DBG_LVL_DEVICE);
    return status;
}


FSTATUS
PutAV(
	IN	SMA_CA_OBJ_PRIVATE		*CaObj,
	IN	IB_HANDLE				AvHandle
	)
{
	FSTATUS						status = FSUCCESS;
			
	_DBG_ENTER_LVL(_DBG_LVL_DEVICE, PutAV);

	// Destroy AV or cache it

	status = iba_destroy_av( AvHandle );

	_DBG_LEAVE_LVL (_DBG_LVL_DEVICE);
    return status;
}


void
NotifyIbtCallback (
	IN	void				*Context
	)
{
	SMA_CA_OBJ_PRIVATE		*pCaObj;
	SMA_PORT_TABLE_PRIV		*pPortTbl;
	SMA_PORT_BLOCK_PRIV		*pPortBlock;
	uint32					j;

	_DBG_ENTER_LVL(_DBG_LVL_DEVICE, NotifyIbtCallback);

	// Scan through all CA's
	SpinLockAcquire(&g_Sma->CaListLock);
	for (pCaObj	= g_Sma->CaObj; pCaObj != NULL; pCaObj = pCaObj->Next)
	{
		boolean exists;

		pCaObj->UseCount++;
		SpinLockRelease(&g_Sma->CaListLock);

		exists = IbtNotifyUserEvent( Context,
								pCaObj->CaPublic.CaGuid, // Port/Node GUID
								IB_NOTIFY_CA_ADD );
		if (! exists)
			goto release;

		// Now notify about port events too
		pPortTbl = (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;
		for ( j = 0; j< pCaObj->CaPublic.Ports ; j++ )
		{
			pPortBlock = &pPortTbl->PortBlock[j];

			if ( IB_PORT_ACTIVE == pPortBlock->Public.State )
			{
				exists = IbtNotifyUserEvent( Context,
								pPortBlock->Public.GUID,// Port/Node GUID
								IB_NOTIFY_LID_EVENT );
				if (! exists)
					goto release;
			}
		}

		SpinLockAcquire(&g_Sma->CaListLock);
		pCaObj->UseCount--;
	}

done:
	SpinLockRelease(&g_Sma->CaListLock);
	_DBG_LEAVE_LVL (_DBG_LVL_DEVICE);
	return;

release:
	SpinLockAcquire(&g_Sma->CaListLock);
	pCaObj->UseCount--;
	goto done;
}

void
SetClassMask (
	IN	uint8				Class,
	IN	boolean				Enabled
	)
{
	switch (Class)
	{
	case MCLASS_COMM_MGT:
		if ( TRUE == Enabled )
			g_Sma->IsConnectionManagementSupported = 1;
		else
			g_Sma->IsConnectionManagementSupported = 0;
		break;

#if 0
	case MCLASS_SNMP:
		if ( TRUE == Enabled )
			g_Sma->IsSNMPTunnelingSupported = 1;
		else
			g_Sma->IsSNMPTunnelingSupported = 0;
		break;

	case MCLASS_DEV_MGT:
		if ( TRUE == Enabled )
			g_Sma->IsDeviceManagementSupported = 1;
		else
			g_Sma->IsDeviceManagementSupported = 0;
		break;
#endif

	//
	// TBD:
	// Must handle vendor Classes here
	//case MCLASS_X:
	//
	
	default:
		_DBG_PRINT(_DBG_LVL_DEVICE,(
			"No capabilities for this Class [x%X]\n", 
			Class ));
	}
}

void
SetPortCapabilities ( void )
{
	SMA_CA_OBJ_PRIVATE		*pCaObj;
	FSTATUS					status;
	SMA_PORT_TABLE_PRIV		*pPortTbl;
	SMA_PORT_BLOCK_PRIV		*pPortBlock;
	STL_SMP					*pSmp;
	STL_PORT_INFO			*pPortInfo;
	uint32					j;
	uint8					portNo;
	uint32					smps;
	POOL_DATA_BLOCK			*pSmpBlock;


	_DBG_ENTER_LVL (_DBG_LVL_DEVICE, SetPortCapabilities);

	for ( ;; )
	{
		smps		= 1;
		pSmpBlock	= NULL;

		status = iba_smi_pool_get(
					NULL,
					&smps,			
					(SMP_BLOCK **)&pSmpBlock );

		if ( (FSUCCESS != status) || (pSmpBlock == NULL) )
		{
			_DBG_ERROR ((
					"Could not Grab from Pool!\n" ));
			break;
		}

		pSmp		= (STL_SMP*)pSmpBlock->Block.Smp;
		pPortInfo	= (STL_PORT_INFO*)pSmp->SmpExt.DirectedRoute.SMPData;
		
		// Scan through all CA's
		SpinLockAcquire(&g_Sma->CaListLock);
		for (pCaObj	= g_Sma->CaObj; pCaObj != NULL; pCaObj = pCaObj->Next)
		{
			uint32	capMaskUpdate=0;
			uint32	MadQueryLength;

			pCaObj->UseCount++;
			SpinLockRelease(&g_Sma->CaListLock);

			// Set each port
			pPortTbl = (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;

			for ( j = 0; j< pCaObj->CaPublic.Ports ; j++ )
			{
				pPortBlock							= &pPortTbl->PortBlock[j];
				portNo								= (uint8)j;	// switch port zero

				// Format MAD get PortInfo
				pSmp->common.BaseVersion				= STL_BASE_VERSION;
				pSmp->common.ClassVersion				= STL_SM_CLASS_VERSION;

				pSmp->common.mr.AsReg8				= 0;
				pSmp->common.mr.s.Method				= MMTHD_GET;

				pSmp->SmpExt.DirectedRoute.DrDLID	= STL_LID_PERMISSIVE;
				pSmp->SmpExt.DirectedRoute.DrSLID	= STL_LID_PERMISSIVE;

				pSmp->common.u.DR.s.Status		= 0;
				pSmp->common.u.DR.s.D			= 0;
				pSmp->common.u.DR.HopCount		= 0;
				pSmp->common.u.DR.HopPointer	= 0;
				//pSmp->common.TransactionID	= 0;
				pSmp->common.MgmtClass			= MCLASS_SM_DIRECTED_ROUTE;

				pSmp->common.AttributeID		= STL_MCLASS_ATTRIB_ID_PORT_INFO;
				pSmp->common.AttributeModifier	= 0x01000000 | portNo;

				STL_SMP_SET_LOCAL(pSmp);		// local request

				MadQueryLength = sizeof(*pSmp) - STL_MAX_PAYLOAD_SMP_DR;
				status = iba_get_set_mad( pPortBlock->QpHandle, portNo,
									0, // SLID ignored, local request
									pSmp,
									&MadQueryLength );
				if ( FSUCCESS != status )
				{
					_DBG_ERROR ((
						"Could not GET Port capabilities!\n" ));
					break;
				}

				// SWAP THE GETPORTINFO response
				BSWAP_STL_PORT_INFO(pPortInfo);
				ZERO_RSVD_STL_PORT_INFO(pPortInfo);

				// Set capability if necessary
				if (g_Sma->IsConnectionManagementSupported && !pPortInfo->CapabilityMask.s.IsConnectionManagementSupported) {
					pPortInfo->CapabilityMask.s.IsConnectionManagementSupported = \
						g_Sma->IsConnectionManagementSupported;
					capMaskUpdate = 1;
				}
				if (g_Sma->IsDeviceManagementSupported && !pPortInfo->CapabilityMask.s.IsDeviceManagementSupported) {
					pPortInfo->CapabilityMask.s.IsDeviceManagementSupported = \
						g_Sma->IsDeviceManagementSupported;
					capMaskUpdate = 1;
				}
				if (!capMaskUpdate)
					break;				// nothing to change
				
				// Do not change other fields
				pPortInfo->LinkWidth.Enabled = STL_LINK_WIDTH_NOP;
				pPortInfo->PortStates.s.PortState = IB_PORT_NOP;
				pPortInfo->PortStates.s.PortPhysicalState = IB_PORT_PHYS_NOP;
				pPortInfo->s4.OperationalVL = 0;
				pPortInfo->LinkSpeed.Enabled = IB_LINK_SPEED_NOP;
				
				// SWAP THE SET PORT INFO payload
				BSWAP_STL_PORT_INFO(pPortInfo);

				// Set base MAD info
				pSmp->common.BaseVersion				= STL_BASE_VERSION;
				pSmp->common.ClassVersion				= STL_SM_CLASS_VERSION;
				
				pSmp->common.mr.AsReg8				= 0;
				pSmp->common.mr.s.Method			= MMTHD_SET;

				pSmp->SmpExt.DirectedRoute.DrDLID	= STL_LID_PERMISSIVE;
				pSmp->SmpExt.DirectedRoute.DrSLID	= STL_LID_PERMISSIVE;
				
				pSmp->common.u.DR.s.Status		= 0;
				pSmp->common.u.DR.s.D			= 0;
				pSmp->common.u.DR.HopCount		= 0;
				pSmp->common.u.DR.HopPointer	= 0;
				//pSmp->common.TransactionID	= 0;
				pSmp->common.MgmtClass			= MCLASS_SM_DIRECTED_ROUTE;

				pSmp->common.AttributeID		= STL_MCLASS_ATTRIB_ID_PORT_INFO;
				pSmp->common.AttributeModifier	= 0x01000000 | portNo;

				STL_SMP_SET_LOCAL(pSmp);		// local request

				MadQueryLength = sizeof(*pSmp) - STL_MAX_PAYLOAD_SMP_DR + sizeof(*pPortInfo);
				status = iba_get_set_mad( pPortBlock->QpHandle, portNo,
										0, // SLID ignored, local request
										pSmp,
										&MadQueryLength );
				if ( FSUCCESS != status )
					break;

 			}	// loop j - ports

			SpinLockAcquire(&g_Sma->CaListLock);
			pCaObj->UseCount--;
			
			if ( FSUCCESS != status )
				break;

		}		// loop - CA's
		SpinLockRelease(&g_Sma->CaListLock);

		status = iba_smi_pool_put( NULL, (SMP_BLOCK *)pSmpBlock );
		break;
	}

	_DBG_LEAVE_LVL (_DBG_LVL_DEVICE);
}

void
SetPortCapabilityMask (
	IN	uint8				Class,
	IN	boolean				Enabled
	)
{
	_DBG_ENTER_LVL (_DBG_LVL_DEVICE, SetPortCapabilityMask );

	// Check init state of drivers
	SetClassMask ( Class, Enabled );

	if ( g_Sma->NumCa )
	{
		SetPortCapabilities ();
	} else {
		_DBG_PRINT (_DBG_LVL_DEVICE, (
			"No devices present. Caching capabilities for an Add Device\n" ));
	}

	_DBG_LEAVE_LVL (_DBG_LVL_DEVICE);
}
