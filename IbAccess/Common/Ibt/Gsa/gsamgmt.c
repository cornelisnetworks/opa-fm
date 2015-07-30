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

void
GsaUpdateCaList(void)
{
	if (g_GsaGlobalInfo)
	{
		SpinLockAcquire( &g_GsaGlobalInfo->CaListLock );
	
		if ( g_GsaGlobalInfo->SmObj.CaTbl )
		{
			MemoryDeallocate( g_GsaGlobalInfo->SmObj.CaTbl );
			g_GsaGlobalInfo->SmObj.CaTbl = NULL;
			/* for final RemoveDevice, NumCa = 0 and SmaCreateObj is a Noop
			 * for AddDevice SmaCreateSmaObj will reinitialize NumCa
			 */
			g_GsaGlobalInfo->SmObj.NumCa--;
		}

		SmaCreateSmaObj( &g_GsaGlobalInfo->SmObj );

		SpinLockRelease( &g_GsaGlobalInfo->CaListLock );
	
	}
}


FSTATUS
PrepareCaForReceives(
	IN	EUI64					CaGuid
	)
{
	FSTATUS						status			= FSUCCESS;
	SMA_PORT_TABLE_PRIV			*pPortTblPriv	= NULL;
	uint32						i;
	SMA_CA_TABLE				*pCaTbl;
	SMA_PORT_TABLE				*pPortTbl;
	SMA_CA_OBJ_PRIVATE			*pCaObj;
	uint8						j;
	

	_DBG_ENTER_LVL(_DBG_LVL_MGMT, PrepareCaForReceives);


	//
	// Query CaGUID and PortInfo
	//

	SpinLockAcquire( &g_GsaGlobalInfo->CaListLock );

	pCaTbl = g_GsaGlobalInfo->SmObj.CaTbl;

	pCaObj = NULL;		// init for parsing

	for ( i=0; i<g_GsaGlobalInfo->SmObj.NumCa; i++ )
	{
		if ( CaGuid == pCaTbl->CaObj[i].CaGuid )
		{
			pPortTbl = pCaTbl->CaObj[i].PortTbl;
			pCaObj = (SMA_CA_OBJ_PRIVATE *)pCaTbl->CaObj[i].SmaCaContext;
			pPortTblPriv = (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;
			break;
		}
	}		// i

	SpinLockRelease( &g_GsaGlobalInfo->CaListLock );
	

	ASSERT( pPortTblPriv );

	
	//
	// Post default msgs 
	//

	status = DgrmPoolAddToGlobalRecvQ( 
		g_GsaSettings.PreAllocRecvBuffersPerPort * pCaTbl->CaObj[i].Ports );
	
	if ( FSUCCESS == status )
	{
		for ( j=0; j< pCaTbl->CaObj[i].Ports; j++ )
		{
			status = PostRecvMsgToPort(
								g_GsaSettings.MinBuffersToPostPerPort,								
								pPortTblPriv->PortBlock[j].Public.GUID );
		}	// j


		//
		// Rearm CQ to receive event notifications.
		// The next Work Completion written to the CQ 
		// will generate an event
		//
		status = iba_rearm_cq(
				pCaObj->Qp1CqHandle,
				CQEventSelNextWC		
				);						

	
	}
	else
	{
		_DBG_ERROR(("Error in Global RecvQ!\n"));
	}

    _DBG_LEAVE_LVL(_DBG_LVL_MGMT);
	return status;

}    



FSTATUS
GetCaContextFromAvInfo(
	IN	IB_ADDRESS_VECTOR		*AvInfo,
	IN	uint32					CaMemIndex,
	OUT	IB_HANDLE				*AvHandle,
	OUT	IB_HANDLE				*Qp1Handle,
	OUT	IB_HANDLE				*Cq1Handle,
	OUT SPIN_LOCK				**QpLock,
	OUT	IB_L_KEY				*LKey
	)
{
	FSTATUS						status=FSUCCESS;
	SMA_CA_TABLE				*pCaTbl;
	SMA_PORT_TABLE				*pPortTbl;
	SMA_CA_OBJ_PRIVATE			*pCaObj;
	SMA_PORT_TABLE_PRIV			*pPortTblPriv;
	CA_MEM_LIST					caMemList;
	uint32						i;
	uint8						j=0;
	boolean						bFound=FALSE;
	

	_DBG_ENTER_LVL(_DBG_LVL_MGMT, GetCaContextFromAvInfo);

	//
	// Init variables
	//
	*Qp1Handle	= NULL;
	*Cq1Handle	= NULL;
	*QpLock		= NULL;
	*LKey		= 0;

	SpinLockAcquire( &g_GsaGlobalInfo->CaListLock );

	//
	// This lookup can be converted to hash table on GUIDs
	//

	pCaTbl = g_GsaGlobalInfo->SmObj.CaTbl;

	for ( i=0; i<g_GsaGlobalInfo->SmObj.NumCa; i++ )
	{
		pPortTbl = pCaTbl->CaObj[i].PortTbl;
		for ( j=0; j<pCaTbl->CaObj[i].Ports; j++ )
		{
			if ( AvInfo->PortGUID == pPortTbl->PortBlock[j].GUID )
			{
				bFound = TRUE;
				break;
			}
		}	// j

		if ( TRUE == bFound )
			break;
	}		// i

	if ( TRUE == bFound )
	{
		pCaObj			= (SMA_CA_OBJ_PRIVATE *)pCaTbl->CaObj[i].SmaCaContext;
		pPortTblPriv	= (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;

		//
		// Only post to Qp not in error or destroy state
		// hardware will handle active and armed tests, by commenting out
		// we allow a down port to still be used to talk to itself for
		// applications using IB for local communications
		//
		if ( /*(IB_PORT_ACTIVE == pPortTblPriv->PortBlock[j].Public.State 
				||IB_PORT_ARMED == pPortTblPriv->PortBlock[j].Public.State ) && */ \
			( QP_NORMAL == pPortTblPriv->PortBlock[j].Qp1Control ))
		{
			//
			// Create AV only if requested
			//
			if ( NULL != AvHandle )
			{
				//AvInfo->PortNumber = j + 1; 
				status = GetAV( pCaObj, AvInfo, AvHandle );
			}

			if ( FSUCCESS == status )
			{
				*Qp1Handle	= pPortTblPriv->PortBlock[j].Qp1Handle;
				*Cq1Handle	= pCaObj->Qp1CqHandle;
				*QpLock		= &pPortTblPriv->PortBlock[j].QpLock;

				GetMemInfoFromCaMemTable( pCaObj, CaMemIndex, &caMemList);

				*LKey = caMemList.LKey;
			}
		}
		else
		{
			//_DBG_ERROR(("Port 0x%"PRIx64" Not Active\n", AvInfo->PortGUID));
			status = FINVALID_STATE;
		}
	}
	else
	{
		//_DBG_ERROR(("Unable to find Ca for Port 0x%"PRIx64"\n", AvInfo->PortGUID));
		status = FINVALID_PARAMETER;
	}

	SpinLockRelease( &g_GsaGlobalInfo->CaListLock );
	
    _DBG_LEAVE_LVL(_DBG_LVL_MGMT);
	return status;

}


FSTATUS
GetGsiContextFromPortGuid(
	IN	EUI64					PortGuid,
	OUT	IB_HANDLE				*Qp1Handle,
	OUT	uint8					*PortNumber
	)
{
	FSTATUS						status=FSUCCESS;
	SMA_CA_TABLE				*pCaTbl;
	SMA_PORT_TABLE				*pPortTbl;
	SMA_CA_OBJ_PRIVATE			*pCaObj;
	SMA_PORT_TABLE_PRIV			*pPortTblPriv;
	uint32						i;
	uint8						j = 0;
	boolean						bFound=FALSE;
	

	_DBG_ENTER_LVL(_DBG_LVL_MGMT, GetGsiContextFromPortGuid);

	//
	// Init variables
	//
	*Qp1Handle	= NULL;

	SpinLockAcquire( &g_GsaGlobalInfo->CaListLock );

	//
	// This lookup can be converted to hash table on GUIDs
	//

	pCaTbl = g_GsaGlobalInfo->SmObj.CaTbl;

	for ( i=0; i<g_GsaGlobalInfo->SmObj.NumCa; i++ )
	{
		pPortTbl = pCaTbl->CaObj[i].PortTbl;
		for ( j=0; j<pCaTbl->CaObj[i].Ports; j++ )
		{
			if ( PortGuid == pPortTbl->PortBlock[j].GUID )
			{
				bFound = TRUE;
				break;
			}
		}	// j

		if ( TRUE == bFound )
			break;
	}		// i

	if ( TRUE == bFound )
	{
		pCaObj			= (SMA_CA_OBJ_PRIVATE *)pCaTbl->CaObj[i].SmaCaContext;
		pPortTblPriv	= (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;

		//
		// Only post to Qp not in error or destroy state
		// hardware will handle active and armed tests, by commenting out
		// we allow a down port to still be used to talk to itself for
		// applications using IB for local communications
		//
		if (/*( IB_PORT_ACTIVE == pPortTblPriv->PortBlock[j].Public.State 
			  || IB_PORT_ARMED == pPortTblPriv->PortBlock[j].Public.State ) &&*/ \
			( QP_NORMAL == pPortTblPriv->PortBlock[j].Qp1Control ))
		{
			*PortNumber = j + 1; 
			*Qp1Handle	= pPortTblPriv->PortBlock[j].Qp1Handle;
		} else {
			//_DBG_ERROR(("Port 0x%"PRIx64" Not Active\n", PortGuid));
			status = FINVALID_STATE;
		}
	}
	else
	{
		status = FINVALID_PARAMETER;
	}

	SpinLockRelease( &g_GsaGlobalInfo->CaListLock );
	
    _DBG_LEAVE_LVL(_DBG_LVL_MGMT);
	return status;
}

FSTATUS
DecrementGsiRecvQPostedForPortGuid(
	IN	EUI64					PortGuid,
	OUT	uint32					*RecvQPosted
	)
{
	FSTATUS						status=FSUCCESS;
	SMA_CA_TABLE				*pCaTbl;
	SMA_PORT_TABLE				*pPortTbl;
	SMA_CA_OBJ_PRIVATE			*pCaObj;
	SMA_PORT_TABLE_PRIV			*pPortTblPriv;
	uint32						i;
	uint8						j = 0;
	boolean						bFound=FALSE;
	
	_DBG_ENTER_LVL(_DBG_LVL_MGMT, DecrementGsiRecvQPostedForPortGuid);

	SpinLockAcquire( &g_GsaGlobalInfo->CaListLock );

	// This lookup can be converted to hash table on GUIDs
	pCaTbl = g_GsaGlobalInfo->SmObj.CaTbl;

	for ( i=0; i<g_GsaGlobalInfo->SmObj.NumCa; i++ )
	{
		pPortTbl = pCaTbl->CaObj[i].PortTbl;
		for ( j=0; j<pCaTbl->CaObj[i].Ports; j++ )
		{
			if ( PortGuid == pPortTbl->PortBlock[j].GUID )
			{
				bFound = TRUE;
				break;
			}
		}	// j

		if ( TRUE == bFound )
			break;
	}		// i

	if ( TRUE == bFound )
	{
		pCaObj			= (SMA_CA_OBJ_PRIVATE *)pCaTbl->CaObj[i].SmaCaContext;
		pPortTblPriv	= (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;

		*RecvQPosted = --pPortTblPriv->PortBlock[j].Qp1RecvQPosted;
		status = FSUCCESS;
	} else {
		status = FINVALID_PARAMETER;
	}

	SpinLockRelease( &g_GsaGlobalInfo->CaListLock );
	
    _DBG_LEAVE_LVL(_DBG_LVL_MGMT);
	return status;
}

FSTATUS
PostRecvMsgToPort(
	IN	uint32					ElementCount,
	IN	EUI64					PortGuid
	)
{
	FSTATUS						status		= FSUCCESS;
	uint32						i, j;
	IBT_DGRM_ELEMENT			*pDgrmList, *pDgrmListLink;
	IBT_DGRM_ELEMENT_PRIV		*pIbDgrmList;
	IB_WORK_REQ					workRequest;
	
	SMA_CA_TABLE				*pCaTbl;
	CA_MEM_LIST					caMemList;
	SMA_PORT_TABLE				*pPortTbl;
	SMA_CA_OBJ_PRIVATE			*pCaObj;
	uint8						k=0;
	boolean						bFound;
	IB_HANDLE					qp1Handle = NULL;
	IB_HANDLE					qp1CqHandle = NULL;
	SPIN_LOCK					*qpLock = NULL;
	

	_DBG_ENTER_LVL(_DBG_LVL_MGMT, PostRecvMsgToPort);


	//
	// Get elements from Pool
	//
	i = ElementCount;
	status = DgrmPoolGet( 
				g_GsaGlobalInfo->DgrmPoolRecvQ,
				&i,
				&pDgrmList );
	//
	// post to qp
	// 
	if ( status == FSUCCESS )
	{
		SMA_PORT_TABLE_PRIV	*pPortTblPriv = NULL;


		if (i < ElementCount)
			_DBG_WARN(("Gsi PostRecv: Only allocated %d, requested %d elements\n", i, ElementCount));

		//
		// Query PortGUID
		//

		SpinLockAcquire( &g_GsaGlobalInfo->CaListLock );

		pCaTbl = g_GsaGlobalInfo->SmObj.CaTbl;

		pCaObj = NULL;		// init for parsing
		bFound = FALSE;

		for ( j=0; j<g_GsaGlobalInfo->SmObj.NumCa; j++ )
		{
			pPortTbl = pCaTbl->CaObj[j].PortTbl;
			for ( k=0; k<pCaTbl->CaObj[j].Ports; k++ )
			{
				if ( PortGuid == pPortTbl->PortBlock[k].GUID )
				{
					pCaObj = \
					 (SMA_CA_OBJ_PRIVATE *)pCaTbl->CaObj[j].SmaCaContext;
					pPortTblPriv = \
						(SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;
					bFound = TRUE;
					break;
				}
			}	// j

			if ( TRUE == bFound )
			{
				qp1Handle = pPortTblPriv->PortBlock[k].Qp1Handle;
				qp1CqHandle = pCaObj->Qp1CqHandle;
				qpLock = &pPortTblPriv->PortBlock[k].QpLock;

				//
				// Fail request if Qp is in error or destory state
				//
				if ( QP_DESTROY == pPortTblPriv->PortBlock[k].Qp1Control )
					bFound = FALSE;

				break;
			}
		}		// i

		SpinLockRelease( &g_GsaGlobalInfo->CaListLock );

		ASSERT( pCaObj );

		if ( FALSE == bFound )
		{
			_DBG_PRINT(
				_DBG_LVL_MGMT,(\
				"An active Port could not be found OR"
				"Qp in Error on Port(?%d?)!\n",
				k+1));

			DgrmPoolPut( pDgrmList );	
		}
		else if ((NULL != qp1Handle) && (NULL != qpLock))
		{
			while ( i )
			{
				pDgrmListLink = \
					(IBT_DGRM_ELEMENT *)pDgrmList->Element.pNextElement;
				pDgrmList->Element.pNextElement = NULL;			// delink

				//
				// fill in work request details
				//

				pIbDgrmList = (IBT_DGRM_ELEMENT_PRIV *)pDgrmList;
				workRequest.DSList = pIbDgrmList->DsList;
				workRequest.Operation = WROpRecv;
				pDgrmList->OnRecvQ = TRUE;
			

				//
				// Input values to process upon recv
				//

				pIbDgrmList->DgrmElement.PortGuid	= PortGuid;

				// These lines added since Verbs does not return the Q_Key
				pIbDgrmList->DgrmElement.RemoteQP	= 1;
				pIbDgrmList->DgrmElement.RemoteQKey	= QP1_WELL_KNOWN_Q_KEY;


				//
				// get LKey and Memory handles
				//
				

				//
				// Create GRH header
				//
				GetMemInfoFromCaMemTable(
							pCaObj,
							pIbDgrmList->MemPoolHandle->MemList->CaMemIndex,
							&caMemList
							);
				

				//
				// Add GRH and Data in one header since data is contiguous
				//
				workRequest.DSList[0].Address = \
					(uintn)pIbDgrmList->DgrmElement.Element.pBufferList->pData;
				workRequest.DSList[0].Length = sizeof(IB_GRH)+sizeof(MAD);
				workRequest.DSList[0].Lkey = caMemList.LKey;

				workRequest.DSListDepth = 1;


				workRequest.MessageLen = sizeof(IB_GRH) + sizeof(MAD);

				//
				// set Id to request stucture to unload
				//

				ASSERT_VALID_WORKREQID((uintn)pIbDgrmList);
				workRequest.WorkReqId = BUILD_RQ_WORKREQID((uintn)pIbDgrmList);
				

				//
				// Post on Recv
				//

				SpinLockAcquire( qpLock );

				status = iba_post_recv( qp1Handle, &workRequest);

				SpinLockRelease( qpLock );
				if (status != FSUCCESS)
				{
					_DBG_ERROR(("PostRecvMsgToPort: iba_post_recv failed %s(%d)\n",
						_DBG_PTR(FSTATUS_MSG(status)), status));
					pDgrmList->OnRecvQ = FALSE;
					DgrmPoolPut( pDgrmList );	
					if (pDgrmListLink)
						DgrmPoolPut(pDgrmListLink);
					break;
				}
				++(pPortTblPriv->PortBlock[k].Qp1RecvQPosted);
				_DBG_PRINT(_DBG_LVL_MGMT,("0x%"PRIx64": after post count of %u\n",PortGuid, pPortTblPriv->PortBlock[k].Qp1RecvQPosted));
				pDgrmList = pDgrmListLink;
				i--;
			}	// i loop
		}
		else
		{
			_DBG_ERROR(("Failed to post to port - null pointers.\n"));
			status = FERROR;
		}
	} else {
		_DBG_ERROR(("PostRecvMsgToPort: No more elements in Pool for %d elements!!!\n",ElementCount));
	}

	GrowRecvQAsNeeded();

    _DBG_LEAVE_LVL(_DBG_LVL_MGMT);
	return status;

}
