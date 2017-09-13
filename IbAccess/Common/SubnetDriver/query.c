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


#include <datatypes.h>
#include <imemory.h>
#include <ievent.h>
#include <imutex.h>
#include <stl_sd.h>
#include <stl_sa.h>
#include <sdi.h>

LOCKED_QUICK_LIST *pQueryList = NULL;
LOCKED_QUICK_LIST *pCompletionList = NULL;
LOCKED_QUICK_LIST *pChildQueryList = NULL;
ATOMIC_UINT ChildQueryActive;

MUTEX *pChildGrowParentMutex;

ATOMIC_UINT TranId;

#define PATHRECORD_NUMBPATH				32		// max to request given 2 guids

void
FillBaseGmpValues(
	IN PQueryDetails piQuery
	);

FSTATUS
FillSADetailsForNodeRecord(
	IN PQueryDetails pQueryElement,
	IN uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForPortInfoRecord(
	IN PQueryDetails pQueryElement,
	IN uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForSMInfoRecord(
	IN PQueryDetails pQueryElement,
	IN uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForLinkRecord(
	IN PQueryDetails pQueryElement,
	IN uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForPathRecord(
	IN PQueryDetails pQueryElement,
	IN uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForServiceRecord(
	IN PQueryDetails pQueryElement,
	IN uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForMcMemberRecord(
	IN PQueryDetails pQueryElement,
	IN uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForInformInfoRecord(
	IN PQueryDetails pQueryElement,
	IN uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForInformInfo(
	IN PQueryDetails pQueryElement,
	IN uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForMultiPathRecord(
	IN PQueryDetails pQueryElement,
	IN uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForTraceRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForSwitchInfoRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForLinearFDBRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForRandomFDBRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForMCastFDBRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForVLArbTableRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	);

FSTATUS
FillSADetailsForPKeyTableRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	);

void
FillCommonSADetails(
	IN PQueryDetails pQueryElement,
	IN uint16		 AttribId,
	IN uint32		 RecordSize
	);

FSTATUS
BuildCommonSAMad(
	IN PQueryDetails pQueryElement,
	IN uint16		 AttribId,
	IN uint32		 RecordSize,
	IN uint32		 MemAllocFlags
	);

#if 0 // keep this around in case we need it for debugging again
void QueryShowPacket(char *msg, unsigned char *packet, int length)
{
	_DBG_DUMP_MEMORY(_DBG_LVL_?, ("%s", msg), (void*)packet, length);
}
#endif

FSTATUS
InitializeList(
	QUICK_LIST **ppList,
	SPIN_LOCK **ppListLock
)
{
	FSTATUS Fstatus = FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, InitializeList);
	
	*ppList = (QUICK_LIST*)MemoryAllocateAndClear(sizeof(QUICK_LIST), FALSE, 
		SUBNET_DRIVER_TAG);
	*ppListLock =  (SPIN_LOCK*)MemoryAllocateAndClear(sizeof(SPIN_LOCK), FALSE, 
		SUBNET_DRIVER_TAG);

	if ((*ppList == NULL) || (*ppListLock == NULL)) 
	{
		_DBG_ERROR(("Cannot allocate global query list\n"));
		Fstatus = FINSUFFICIENT_MEMORY;
		goto failalloc;
	}

	QListInitState(*ppList);
	if (!QListInit(*ppList)) 
	{
		_DBG_ERROR(("Cannot intialize query list, exiting\n"));
		Fstatus = FERROR;
		goto faillist;
	}

	// Initialize the the control lock for the query list 
	SpinLockInitState(*ppListLock);
	if (!SpinLockInit(*ppListLock))  
	{
		_DBG_ERROR(("Cannot initialize query list lock\n"));
		Fstatus = FERROR;
		goto faillock;
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return(FSUCCESS);

faillock:
	QListDestroy(*ppList);
faillist:
failalloc:
	if (*ppListLock != NULL)
	{
		MemoryDeallocate(*ppListLock);
		*ppListLock = NULL;
	}
	if (*ppList != NULL)
	{
		MemoryDeallocate(*ppList);
		*ppList = NULL;
	}
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return(Fstatus);
}


void
DestroyList(
	QUICK_LIST *pList,
	SPIN_LOCK *pListLock
	)
{
	void * pListEntry;
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DestroyList);

	if (pListLock != NULL)  
	{
		//Destroy the List
		SpinLockAcquire(pListLock);
		while(NULL != (pListEntry = (void *)QListRemoveHead(pList)))
		{
			MemoryDeallocate(pListEntry);
		}
		QListDestroy(pList);
		SpinLockRelease(pListLock);
		
		MemoryDeallocate(pList);
		
		//Destroy the Lock
		SpinLockDestroy(pListLock);
		MemoryDeallocate(pListLock);
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}


void
FillBaseGmpValues(
	IN PQueryDetails piQuery
	)
{
	uint64 TempTranId;
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillBaseGmpValues);
	
	piQuery->u.pGmp->common.BaseVersion = IB_BASE_VERSION;
	piQuery->u.pGmp->common.MgmtClass = MCLASS_SUBN_ADM;
	piQuery->u.pGmp->common.ClassVersion = IB_SUBN_ADM_CLASS_VERSION; 
	TempTranId = (uint64)AtomicIncrement(&TranId);
	piQuery->u.pGmp->common.TransactionID = TempTranId << 8;
	_DBG_INFO(("The TransactionID stored is <%"PRIx64">\n", TempTranId));
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

// verify the ClientHandle is valid and return the corresponding
// ClientControlParams
FSTATUS ValidateClientHandle(
	IN CLIENT_HANDLE ClientHandle,
	OUT CLIENT_CONTROL_PARAMETERS *pClientParams
	)
{
	FSTATUS Fstatus = FNOT_FOUND;
	LIST_ITEM	*pListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ValidateClientHandle);
	if(ClientHandle == NULL) 
	{
		//Handle the case for invalid parameters
		_DBG_ERROR(("Client handle is a null value\n"));
		Fstatus = FINVALID_PARAMETER;
		goto done;
	}
	pListItem = &((ClientListEntry*)ClientHandle)->ListItem;

	SpinLockAcquire(pClientListLock);
	if (QListIsItemInList(pClientList, pListItem))
	{
		ClientListEntry *pClientEntry = (ClientListEntry*)ClientHandle;

		Fstatus = FSUCCESS;
		// Get the client control parameters
		*pClientParams = pClientEntry->ClientParameters;
	}
	SpinLockRelease(pClientListLock);
done:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

// check if port is valid and active
// if *pPortGuid == (EUI64)-1, 1st active port is returned in *pPortGuid
FSTATUS CheckPortActive(
		IN OUT EUI64 *pPortGuid
		)
{
	uint64 *pLocalPortGuidsList = NULL;
	uint32 LocalPortGuidsCount;
	uint32 ii;
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CheckPortActive);
	/* Get List Of Active Port Guids */
	Fstatus = iba_sd_get_local_port_guids_alloc(&pLocalPortGuidsList, &LocalPortGuidsCount);
	if (Fstatus != FSUCCESS || LocalPortGuidsCount == 0)
	{
		_DBG_INFO(("No Active LocalPortGuids found\n"));
		if (Fstatus == FSUCCESS)
			Fstatus = FNOT_FOUND;
		goto done;
	}

	/* Look to see if we are to use the first active port */
	if (*pPortGuid == (EUI64)-1)
	{
		*pPortGuid = pLocalPortGuidsList[0];
	} else {
		/* Check to see if it's active */
		for (ii = 0; (ii < LocalPortGuidsCount); ++ii)
		{
			if (*pPortGuid == pLocalPortGuidsList[ii])
				break;
		}

		if (ii == LocalPortGuidsCount)
		{
			/* It's not active now */
			Fstatus = FNOT_FOUND;
			_DBG_INFO(("LocalPortGuid <0x%"PRIx64"> Not Currently Active.\n", *pPortGuid));
			goto done;
		}
	}

done:
	if (pLocalPortGuidsList != NULL)
	{
		MemoryDeallocate(pLocalPortGuidsList);
	}
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}


// lookup an appropriate subnet prefix for a port
// if the PortGuid is local we use its SubnetPrefix
// otherwise we use the SubnetPrefix of the port the query is going to
// be issued from
uint64 GetSubnetPrefixForPortGuid(
		EUI64 QueryPortGuid,	// local port query will be issued from
		EUI64 PortGuid			// port we want a subnet prefix for
		)
{
	CA_PORT *pCaPort;
	uint64 subnetPrefix;

	SpinLockAcquire(pCaPortListLock);
	pCaPort = GetCaPort(PortGuid);
	if (pCaPort != NULL)
	{
		// a local port
		subnetPrefix =  pCaPort->SubnetPrefix;
	} else {
		// non-local port
		// use the Subnet Prefix of the port we are issuing query from
		pCaPort = GetCaPort(QueryPortGuid);
		ASSERT(pCaPort != NULL);
		subnetPrefix =  pCaPort->SubnetPrefix;
	}
	SpinLockRelease(pCaPortListLock);
	return subnetPrefix;
}

// Query against 1st active port
FSTATUS
iba_sd_query_fabric_info(
	IN CLIENT_HANDLE ClientHandle,
	IN PQUERY pQuery,
	IN SDK_QUERY_CALLBACK *Callback,
	IN COMMAND_CONTROL_PARAMETERS *pQueryControlParameters OPTIONAL,
	IN void *Context OPTIONAL
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_query_fabric_info);

	Fstatus = CommonQueryPortFabricInformation(ClientHandle, (EUI64)-1, pQuery, Callback, pQueryControlParameters, Context);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return(Fstatus);
}

// Query against a specific port
// if Portuid == (EUI64)-1 -> against 1st active port
FSTATUS
iba_sd_query_port_fabric_info(
	IN CLIENT_HANDLE ClientHandle,
	IN EUI64 PortGuid,
	IN PQUERY pQuery,
	IN SDK_QUERY_CALLBACK *Callback,
	IN COMMAND_CONTROL_PARAMETERS *pQueryControlParameters OPTIONAL,
	IN void *Context OPTIONAL
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_query_port_fabric_info);
	Fstatus = CommonQueryPortFabricInformation(ClientHandle, PortGuid, pQuery, Callback, pQueryControlParameters, Context);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return(Fstatus);
}

// Common routine for Kernel calls
// Query against a specific port
// if Portuid == (EUI64)-1 -> against 1st active port
FSTATUS
CommonQueryPortFabricInformation(
	IN CLIENT_HANDLE ClientHandle,
	IN EUI64 PortGuid,
	IN PQUERY pQuery,
	IN SDK_QUERY_CALLBACK *Callback,
	IN COMMAND_CONTROL_PARAMETERS *pQueryControlParameters OPTIONAL,
	IN void *Context OPTIONAL
	)
{
	FSTATUS Fstatus;
	CLIENT_CONTROL_PARAMETERS ClientParams;
	PQueryDetails pQueryElement;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CommonQueryPortFabricInformation);
	
	if (SdUnloading)
	{
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return(FINVALID_STATE);
	}
	Fstatus = ValidateClientHandle(ClientHandle, &ClientParams);

	if (Fstatus != FSUCCESS)  
	{
		_SD_ERROR(ClientParams.ClientDebugFlags,
			("Client Handle 0x%p not registered\n", _DBG_PTR(ClientHandle)));
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return(Fstatus);
	}

	Fstatus = CheckPortActive(&PortGuid);
	if (Fstatus != FSUCCESS)
	{
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return(Fstatus);
	}

	pQueryElement = (QueryDetails*)MemoryAllocate2AndClear(sizeof(QueryDetails),
							IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
	if (pQueryElement == NULL)  
	{
		_SD_ERROR(ClientParams.ClientDebugFlags,
			("Cannot allocate memory for QueryElement\n"));
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return(FINSUFFICIENT_MEMORY);
	}
	QListSetObj(&pQueryElement->QueryListItem, (void *)pQueryElement);
	
	if (pQueryControlParameters != NULL)  
	{
		pQueryElement->ControlParameters = *pQueryControlParameters;
	} else {
		pQueryElement->ControlParameters = ClientParams.ControlParameters;
	}

	pQueryElement->ClientHandle = ClientHandle;
	pQueryElement->SdSentCmd = UserQueryFabricInformation;
	pQueryElement->cmd.query.UserQuery = *pQuery;
	pQueryElement->cmd.query.UserCallback =	Callback;
	pQueryElement->cmd.query.pClientResult = NULL;
	pQueryElement->UserContext = Context;
	pQueryElement->PortGuid = PortGuid;
	
	// Convert the user query into one or more detailed queries
	Fstatus = ValidateAndFillSDQuery(pQueryElement, IBA_MEM_FLAG_SHORT_DURATION);
	if (Fstatus != FSUCCESS)
	{
		FreeQueryElement(pQueryElement);
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return(Fstatus);
	}

#if defined(DBG) ||defined( IB_DEBUG)
	{
		CA_PORT *pCaPort;

		SpinLockAcquire(pCaPortListLock);
		pCaPort = GetCaPort(PortGuid);
		ASSERT(pCaPort != NULL);
		ASSERT(pQueryElement->PortGuid == pCaPort->PortGuid);
		SpinLockRelease(pCaPortListLock);
	}
#endif

	SendQueryElement(pQueryElement, TRUE /*FirstAttempt*/);

	StartTimerHandler(pTimerObject, g_SdParams.TimeInterval);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return(FPENDING);
}

// Fabric Operation against 1st active port
FSTATUS
iba_sd_fabric_operation(
	IN CLIENT_HANDLE 				ClientHandle,
	IN FABRIC_OPERATION_DATA* 		pFabOp,
	IN SDK_FABRIC_OPERATION_CALLBACK 	*Callback,
	IN COMMAND_CONTROL_PARAMETERS 	*pCmdControlParameters OPTIONAL,
	IN void*						Context OPTIONAL
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_fabric_operation);

	Fstatus = CommonPortFabricOperation(ClientHandle, (EUI64)-1, pFabOp, Callback, pCmdControlParameters, Context);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return(Fstatus);
}

// Fabric Operation against a specific port
// if Portuid == -1 -> against 1st active port
FSTATUS
iba_sd_port_fabric_operation(
	IN CLIENT_HANDLE 				ClientHandle,
	IN EUI64						PortGuid,
	IN FABRIC_OPERATION_DATA* 		pFabOp,
	IN SDK_FABRIC_OPERATION_CALLBACK 	*Callback,
	IN COMMAND_CONTROL_PARAMETERS 	*pCmdControlParameters OPTIONAL,
	IN void*						Context OPTIONAL
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_port_fabric_operation);

	Fstatus = CommonPortFabricOperation(ClientHandle, PortGuid, pFabOp, Callback, pCmdControlParameters, Context);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

// Common routine for Kernel calls
// Fabric Operation against a specific port
// if Portuid == -1 -> against 1st active port
FSTATUS
CommonPortFabricOperation(
	IN CLIENT_HANDLE 				ClientHandle,
	IN EUI64						PortGuid,
	IN FABRIC_OPERATION_DATA* 		pFabOp,
	IN SDK_FABRIC_OPERATION_CALLBACK 	*Callback,
	IN COMMAND_CONTROL_PARAMETERS 	*pCmdControlParameters OPTIONAL,
	IN void*						Context OPTIONAL
	)
{
	FSTATUS Fstatus = FNOT_FOUND;
	CLIENT_CONTROL_PARAMETERS ClientParams;
	PQueryDetails pQueryElement;
	boolean saOp;
 
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CommonPortFabricOperation);
 
	if (SdUnloading)
	{
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return(FINVALID_STATE);
	}
	Fstatus = ValidateClientHandle(ClientHandle, &ClientParams);
	if (Fstatus != FSUCCESS)
	{
		_SD_ERROR(ClientParams.ClientDebugFlags,
		    ("Client Handle 0x%p not registered\n", _DBG_PTR(ClientHandle)));
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return(Fstatus);
	}
 
	Fstatus = CheckPortActive(&PortGuid);
	if (Fstatus != FSUCCESS)
	{
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return(Fstatus);
	}

	pQueryElement = (QueryDetails*)MemoryAllocate2AndClear(sizeof(QueryDetails),
							IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
	if (pQueryElement == NULL)
	{
		_SD_ERROR(ClientParams.ClientDebugFlags,
		    ("Cannot allocate memory for QueryElement\n"));
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return(FINSUFFICIENT_MEMORY);
	}
	QListSetObj(&pQueryElement->QueryListItem, (void *)pQueryElement);
 
	if (pCmdControlParameters != NULL)
	{
		pQueryElement->ControlParameters = *pCmdControlParameters;
	} else {
		pQueryElement->ControlParameters = ClientParams.ControlParameters;
	}
 
	pQueryElement->ClientHandle = ClientHandle;
	pQueryElement->SdSentCmd= UserFabricOperation;
	pQueryElement->cmd.op.FabOp= *pFabOp;
	pQueryElement->cmd.op.FabOpCallback =   Callback;
	pQueryElement->UserContext = Context;
	pQueryElement->PortGuid = PortGuid;

	switch (pFabOp->Type)
	{
		case FabOpSetServiceRecord:
		case FabOpDeleteServiceRecord:
		case FabOpSetMcMemberRecord:
		case FabOpJoinMcGroup:
		case FabOpLeaveMcGroup:
		case FabOpDeleteMcMemberRecord:
		case FabOpSetInformInfo:
			Fstatus = ValidateAndFillSDFabOp(pQueryElement, IBA_MEM_FLAG_SHORT_DURATION);
			saOp = TRUE;
			break;
		default:
			Fstatus = FINVALID_PARAMETER;
			saOp = TRUE;
			break;
	}
 
	if (Fstatus != FSUCCESS)
	{
		FreeQueryElement(pQueryElement);
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return(Fstatus);
	}

#if defined( DBG) ||defined( IB_DEBUG)
	{
		CA_PORT *pCaPort;

		SpinLockAcquire(pCaPortListLock);
		pCaPort = GetCaPort(PortGuid);
		ASSERT(pCaPort != NULL);
		ASSERT(pQueryElement->PortGuid = pCaPort->PortGuid);
		SpinLockRelease(pCaPortListLock);
	}
#endif

	SendQueryElement(pQueryElement, TRUE);

	StartTimerHandler(pTimerObject, g_SdParams.TimeInterval);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return(FPENDING);
}

FSTATUS
ValidateAndFillSDQuery(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS 			status = FINVALID_PARAMETER;
	PQUERY 				pQuery;
	IB_NODE_RECORD*		pSaNodeRecordData;
	IB_PATH_RECORD*		pSaPathRecordData;


	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ValidateAndFillSDQuery);

	pQuery = &pQueryElement->cmd.query.UserQuery;


	_DBG_INFO(("pQuery->InputType = 0x%x, pQuery->OutputType = 0x%x\n",
											pQuery->InputType,
											pQuery->OutputType));
	switch(pQuery->InputType) 
	{
		case InputTypeNoInput: //NO INPUT
		{
			// output types which are based on a NodeRecord query
			if (pQuery->OutputType == OutputTypeLid
				|| pQuery->OutputType == OutputTypeSystemImageGuid
				|| pQuery->OutputType == OutputTypeNodeGuid
				|| pQuery->OutputType == OutputTypePortGuid
				|| pQuery->OutputType == OutputTypeNodeDesc
				|| pQuery->OutputType == OutputTypeNodeRecord
				|| pQuery->OutputType == OutputTypePathRecord
				)
			{
				status = FillSADetailsForNodeRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypePortInfoRecord)
			{
				status = FillSADetailsForPortInfoRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypeSMInfoRecord)
			{
				status = FillSADetailsForSMInfoRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypeLinkRecord)
			{
				status = FillSADetailsForLinkRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypeServiceRecord)
			{
				status = FillSADetailsForServiceRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypeMcMemberRecord)
			{
				status = FillSADetailsForMcMemberRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypeInformInfoRecord)
			{
				status = FillSADetailsForInformInfoRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypeSwitchInfoRecord)
			{
				status = FillSADetailsForSwitchInfoRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypeLinearFDBRecord)
			{
				status = FillSADetailsForLinearFDBRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypeRandomFDBRecord)
			{
				status = FillSADetailsForRandomFDBRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypeMCastFDBRecord)
			{
				status = FillSADetailsForMCastFDBRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypeVLArbTableRecord)
			{
				status = FillSADetailsForVLArbTableRecord(pQueryElement, MemAllocFlags);
			}
			else if (pQuery->OutputType == OutputTypePKeyTableRecord)
			{
				status = FillSADetailsForPKeyTableRecord(pQueryElement, MemAllocFlags);
			} else {
				status = FINVALID_PARAMETER;
			}
			break;
		}
		case InputTypeNodeType: //Input is a NodeType
		{
			// limited list of valid output types
			// all are based on a NodeRecord query
			if (! (pQuery->OutputType == OutputTypeLid
					|| pQuery->OutputType == OutputTypeSystemImageGuid
					|| pQuery->OutputType == OutputTypeNodeGuid
					|| pQuery->OutputType == OutputTypePortGuid
					|| pQuery->OutputType == OutputTypeNodeDesc
					|| pQuery->OutputType == OutputTypeNodeRecord
					|| pQuery->OutputType == OutputTypePathRecord
				))
			{
				status = FINVALID_PARAMETER;
				break;
			}
			status = FillSADetailsForNodeRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
				
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
								IB_NODE_RECORD_COMP_NODETYPE;
			pSaNodeRecordData = (IB_NODE_RECORD*)&(pQueryElement->u.pSaMad->Data);
			pSaNodeRecordData->NodeInfoData.NodeType = (uint8)pQuery->InputValue.TypeOfNode;
			BSWAP_IB_NODE_RECORD(pSaNodeRecordData);
			status = FSUCCESS;
			break;
		}

		case InputTypeSystemImageGuid:
		{
			// limited list of valid output types
			// all are based on a NodeRecord query
			if (! (pQuery->OutputType == OutputTypeLid
					|| pQuery->OutputType == OutputTypeSystemImageGuid
					|| pQuery->OutputType == OutputTypeNodeGuid
					|| pQuery->OutputType == OutputTypePortGuid
					|| pQuery->OutputType == OutputTypeNodeDesc
					|| pQuery->OutputType == OutputTypeNodeRecord
					|| pQuery->OutputType == OutputTypePathRecord
				))
			{
				status = FINVALID_PARAMETER;
				break;
			}
			status = FillSADetailsForNodeRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
								IB_NODE_RECORD_COMP_SYSIMAGEGUID;
			pSaNodeRecordData = (IB_NODE_RECORD*)&(pQueryElement->u.pSaMad->Data);
			pSaNodeRecordData->NodeInfoData.SystemImageGUID = pQuery->InputValue.Guid;
			BSWAP_IB_NODE_RECORD(pSaNodeRecordData);
			status = FSUCCESS;
			break;
		}

		case InputTypeNodeGuid:
		{
			// limited list of valid output types
			// all are based on a NodeRecord query
			if (! (pQuery->OutputType == OutputTypeLid
					|| pQuery->OutputType == OutputTypeSystemImageGuid
					|| pQuery->OutputType == OutputTypeNodeGuid
					|| pQuery->OutputType == OutputTypePortGuid
					|| pQuery->OutputType == OutputTypeNodeDesc
					|| pQuery->OutputType == OutputTypeNodeRecord
					|| pQuery->OutputType == OutputTypePathRecord
				))
			{
				status = FINVALID_PARAMETER;
				break;
			}

			status = FillSADetailsForNodeRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
								IB_NODE_RECORD_COMP_NODEGUID;
			pSaNodeRecordData = (IB_NODE_RECORD*)&(pQueryElement->u.pSaMad->Data);
			pSaNodeRecordData->NodeInfoData.NodeGUID = pQuery->InputValue.Guid;
			BSWAP_IB_NODE_RECORD(pSaNodeRecordData);
			status  = FSUCCESS;
			break;
		}

		case InputTypePortGuid:
		{
			if ( pQuery->OutputType == OutputTypePathRecord || pQuery->OutputType == OutputTypeTraceRecord)
			{
                if (pQuery->OutputType == OutputTypeTraceRecord)
                    status = FillSADetailsForTraceRecord(pQueryElement, MemAllocFlags);
                else
    				status = FillSADetailsForPathRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask =
						IB_PATH_RECORD_COMP_DGID
							| IB_PATH_RECORD_COMP_SGID
							| IB_PATH_RECORD_COMP_REVERSIBLE
							| IB_PATH_RECORD_COMP_NUMBPATH;
				pSaPathRecordData = (IB_PATH_RECORD*)&(pQueryElement->u.pSaMad->Data);

				// use local port's SubnetPrefix
				pSaPathRecordData->SGID.Type.Global.SubnetPrefix =
				pSaPathRecordData->DGID.Type.Global.SubnetPrefix =
					GetSubnetPrefixForPortGuid(pQueryElement->PortGuid, 
											pQueryElement->PortGuid);
				
				pSaPathRecordData->SGID.Type.Global.InterfaceID =
												pQueryElement->PortGuid;
				pSaPathRecordData->DGID.Type.Global.InterfaceID =
												pQuery->InputValue.Guid;
				pSaPathRecordData->Reversible = 1;
				pSaPathRecordData->NumbPath = PATHRECORD_NUMBPATH;

				BSWAP_IB_PATH_RECORD(pSaPathRecordData);
				status = FSUCCESS;
				break;
			}
			else if ( pQuery->OutputType == OutputTypeMcMemberRecord)
			{
				IB_MCMEMBER_RECORD*  pSaMcMemberRecord;

				status = FillSADetailsForMcMemberRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask =
								IB_MCMEMBER_RECORD_COMP_PORTGID;
				pSaMcMemberRecord = (IB_MCMEMBER_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pSaMcMemberRecord->RID.PortGID.Type.Global.SubnetPrefix =
					GetSubnetPrefixForPortGuid(pQueryElement->PortGuid, 
											pQuery->InputValue.Guid);
				pSaMcMemberRecord->RID.PortGID.Type.Global.InterfaceID =
											pQuery->InputValue.Guid;
				BSWAP_IB_MCMEMBER_RECORD(pSaMcMemberRecord);
				status = FSUCCESS;
				break;
			}
			else if ( pQuery->OutputType == OutputTypeServiceRecord)
			{
				IB_SERVICE_RECORD*  pSaServiceRecord;

				status = FillSADetailsForServiceRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask =
								IB_SERVICE_RECORD_COMP_SERVICEGID;
				pSaServiceRecord = (IB_SERVICE_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pSaServiceRecord->RID.ServiceGID.Type.Global.SubnetPrefix =
					GetSubnetPrefixForPortGuid(pQueryElement->PortGuid, 
											pQuery->InputValue.Guid);
				pSaServiceRecord->RID.ServiceGID.Type.Global.InterfaceID =
											pQuery->InputValue.Guid;
				BSWAP_IB_SERVICE_RECORD(pSaServiceRecord);
				status = FSUCCESS;
				break;
			}
			else if ( pQuery->OutputType == OutputTypeInformInfoRecord)
			{
				IB_INFORM_INFO_RECORD*  pSaInformInfoRecord;

				status = FillSADetailsForInformInfoRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask =
						IB_INFORMINFO_RECORD_COMP_SUBSCRIBERGID;
				pSaInformInfoRecord = (IB_INFORM_INFO_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pSaInformInfoRecord->RID.SubscriberGID.Type.Global.SubnetPrefix=
					GetSubnetPrefixForPortGuid(pQueryElement->PortGuid, 
											pQuery->InputValue.Guid);
				pSaInformInfoRecord->RID.SubscriberGID.Type.Global.InterfaceID =
											pQuery->InputValue.Guid;
				BSWAP_IB_INFORM_INFO_RECORD(pSaInformInfoRecord);
				status = FSUCCESS;
				break;
			}

			// limited list of valid output types
			// all are based on a NodeRecord query
			if (! (pQuery->OutputType == OutputTypeLid
					|| pQuery->OutputType == OutputTypeSystemImageGuid
					|| pQuery->OutputType == OutputTypeNodeGuid
					|| pQuery->OutputType == OutputTypePortGuid
					|| pQuery->OutputType == OutputTypeNodeDesc
					|| pQuery->OutputType == OutputTypeNodeRecord
				))
			{
				status = FINVALID_PARAMETER;
				break;
			}
			status = FillSADetailsForNodeRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
								IB_NODE_RECORD_COMP_PORTGUID;
			pSaNodeRecordData = (IB_NODE_RECORD*)&(pQueryElement->u.pSaMad->Data);
			pSaNodeRecordData->NodeInfoData.PortGUID = pQuery->InputValue.Guid;
			BSWAP_IB_NODE_RECORD(pSaNodeRecordData);
			status = FSUCCESS;
			break;
		}

		case InputTypePortGid:
		{
			if ( pQuery->OutputType == OutputTypePathRecord || pQuery->OutputType == OutputTypeTraceRecord)
			{
                if (pQuery->OutputType == OutputTypeTraceRecord)
                    status = FillSADetailsForTraceRecord(pQueryElement, MemAllocFlags);
                else
    				status = FillSADetailsForPathRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask = 
						IB_PATH_RECORD_COMP_DGID
							| IB_PATH_RECORD_COMP_SGID
							| IB_PATH_RECORD_COMP_REVERSIBLE
							| IB_PATH_RECORD_COMP_NUMBPATH;
				pSaPathRecordData = (IB_PATH_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pSaPathRecordData->DGID = pQuery->InputValue.Gid;
				// use local port's SubnetPrefix
				pSaPathRecordData->SGID.Type.Global.SubnetPrefix =
					GetSubnetPrefixForPortGuid(pQueryElement->PortGuid, 
											pQueryElement->PortGuid);
				pSaPathRecordData->SGID.Type.Global.InterfaceID =
												pQueryElement->PortGuid;
				pSaPathRecordData->Reversible = 1;
				pSaPathRecordData->NumbPath = PATHRECORD_NUMBPATH;

				BSWAP_IB_PATH_RECORD(pSaPathRecordData);
				status = FSUCCESS;
				break;
			}
			else if ( pQuery->OutputType == OutputTypeMcMemberRecord)
			{
				IB_MCMEMBER_RECORD*  pSaMcMemberRecord;

				status = FillSADetailsForMcMemberRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask =
								IB_MCMEMBER_RECORD_COMP_PORTGID;
				pSaMcMemberRecord = (IB_MCMEMBER_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pSaMcMemberRecord->RID.PortGID = pQuery->InputValue.Gid;
				BSWAP_IB_MCMEMBER_RECORD(pSaMcMemberRecord);
				status = FSUCCESS;
				break;
			}
			else if ( pQuery->OutputType == OutputTypeServiceRecord)
			{
				IB_SERVICE_RECORD*  pSaServiceRecord;

				status = FillSADetailsForServiceRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask =
								IB_SERVICE_RECORD_COMP_SERVICEGID;
				pSaServiceRecord = (IB_SERVICE_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pSaServiceRecord->RID.ServiceGID = pQuery->InputValue.Gid;
				BSWAP_IB_SERVICE_RECORD(pSaServiceRecord);
				status = FSUCCESS;
				break;
			}
			else if ( pQuery->OutputType == OutputTypeInformInfoRecord)
			{
				IB_INFORM_INFO_RECORD*  pSaInformInfoRecord;

				status = FillSADetailsForInformInfoRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask =
						IB_INFORMINFO_RECORD_COMP_SUBSCRIBERGID;
				pSaInformInfoRecord = (IB_INFORM_INFO_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pSaInformInfoRecord->RID.SubscriberGID = pQuery->InputValue.Gid;
				BSWAP_IB_INFORM_INFO_RECORD(pSaInformInfoRecord);
				status = FSUCCESS;
				break;
			}

			status = FINVALID_PARAMETER;
			break;
		}
		case InputTypeMcGid:
		{
			if ( pQuery->OutputType == OutputTypeMcMemberRecord)
			{
				IB_MCMEMBER_RECORD*  pSaMcMemberRecord;

				status = FillSADetailsForMcMemberRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask =
								IB_MCMEMBER_RECORD_COMP_MGID;
				pSaMcMemberRecord = (IB_MCMEMBER_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pSaMcMemberRecord->RID.MGID = pQuery->InputValue.Gid;
				BSWAP_IB_MCMEMBER_RECORD(pSaMcMemberRecord);
				status = FSUCCESS;
				break;
			}
			status = FINVALID_PARAMETER;
			break;
		}

		case InputTypeLid:
		{
			if (pQuery->OutputType == OutputTypePortInfoRecord)
			{
				IB_PORTINFO_RECORD*  pPortInfoRecord;
				status = FillSADetailsForPortInfoRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask = IB_PORTINFO_RECORD_COMP_ENDPORTLID;
				pPortInfoRecord = (IB_PORTINFO_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pPortInfoRecord->RID.s.EndPortLID = pQuery->InputValue.Lid;
				BSWAP_IB_PORTINFO_RECORD(pPortInfoRecord, TRUE);
				status = FSUCCESS;
				break;
			} else if ( pQuery->OutputType == OutputTypeMcMemberRecord)
			{
				IB_MCMEMBER_RECORD*  pSaMcMemberRecord;

				status = FillSADetailsForMcMemberRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask =
								IB_MCMEMBER_RECORD_COMP_MLID;
				pSaMcMemberRecord = (IB_MCMEMBER_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pSaMcMemberRecord->MLID = pQuery->InputValue.Lid;
				BSWAP_IB_MCMEMBER_RECORD(pSaMcMemberRecord);
				status = FSUCCESS;
				break;
			} else if (pQuery->OutputType == OutputTypeSwitchInfoRecord)
			{
				IB_SWITCHINFO_RECORD*  pSwitchInfoRecord;
				status = FillSADetailsForSwitchInfoRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask = IB_SWITCHINFO_RECORD_COMP_LID;
				pSwitchInfoRecord = (IB_SWITCHINFO_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pSwitchInfoRecord->RID.s.LID = pQuery->InputValue.Lid;
				BSWAP_IB_SWITCHINFO_RECORD(pSwitchInfoRecord);
				status = FSUCCESS;
				break;
			} else if (pQuery->OutputType == OutputTypeLinearFDBRecord)
			{
				IB_LINEAR_FDB_RECORD*  pLinearFDBRecord;
				status = FillSADetailsForLinearFDBRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask = IB_LINEARFDB_RECORD_COMP_LID;
				pLinearFDBRecord = (IB_LINEAR_FDB_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pLinearFDBRecord->RID.s.LID = pQuery->InputValue.Lid;
				BSWAP_IB_LINEAR_FDB_RECORD(pLinearFDBRecord);
				status = FSUCCESS;
				break;
			} else if (pQuery->OutputType == OutputTypeRandomFDBRecord)
			{
				IB_RANDOM_FDB_RECORD*  pRandomFDBRecord;
				status = FillSADetailsForRandomFDBRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask = IB_RANDOMFDB_RECORD_COMP_LID;
				pRandomFDBRecord = (IB_RANDOM_FDB_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pRandomFDBRecord->RID.s.LID = pQuery->InputValue.Lid;
				BSWAP_IB_RANDOM_FDB_RECORD(pRandomFDBRecord);
				status = FSUCCESS;
				break;
			} else if (pQuery->OutputType == OutputTypeMCastFDBRecord)
			{
				IB_MCAST_FDB_RECORD*  pMCastFDBRecord;
				status = FillSADetailsForMCastFDBRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask = IB_MCASTFDB_RECORD_COMP_LID;
				pMCastFDBRecord = (IB_MCAST_FDB_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pMCastFDBRecord->RID.s.LID = pQuery->InputValue.Lid;
				BSWAP_IB_MCAST_FDB_RECORD(pMCastFDBRecord);
				status = FSUCCESS;
				break;
			} else if (pQuery->OutputType == OutputTypeVLArbTableRecord)
			{
				IB_VLARBTABLE_RECORD*  pVLArbTableRecord;
				status = FillSADetailsForVLArbTableRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask = IB_VLARBTABLE_RECORD_COMP_LID;
				pVLArbTableRecord = (IB_VLARBTABLE_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pVLArbTableRecord->RID.s.LID = pQuery->InputValue.Lid;
				BSWAP_IB_VLARBTABLE_RECORD(pVLArbTableRecord);
				status = FSUCCESS;
				break;
			} else if (pQuery->OutputType == OutputTypePKeyTableRecord)
			{
				IB_P_KEY_TABLE_RECORD*  pPKeyTableRecord;
				status = FillSADetailsForPKeyTableRecord(pQueryElement, MemAllocFlags);
				if ( status != FSUCCESS)
				{
					break;
				}
				pQueryElement->u.pSaMad->SaHdr.ComponentMask = IB_PKEYTABLE_RECORD_COMP_LID;
				pPKeyTableRecord = (IB_P_KEY_TABLE_RECORD*)&(pQueryElement->u.pSaMad->Data);
				pPKeyTableRecord->RID.s.LID = pQuery->InputValue.Lid;
				BSWAP_IB_P_KEY_TABLE_RECORD(pPKeyTableRecord);
				status = FSUCCESS;
				break;
			}
			// limited list of valid output types
			// all are based on a NodeRecord query
			if (! (pQuery->OutputType == OutputTypeLid
					|| pQuery->OutputType == OutputTypeSystemImageGuid
					|| pQuery->OutputType == OutputTypeNodeGuid
					|| pQuery->OutputType == OutputTypePortGuid
					|| pQuery->OutputType == OutputTypeNodeDesc
					|| pQuery->OutputType == OutputTypeNodeRecord
					|| pQuery->OutputType == OutputTypePathRecord
				))
			{
				status = FINVALID_PARAMETER;
				break;
			}
			status = FillSADetailsForNodeRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
			pSaNodeRecordData = (IB_NODE_RECORD*)&(pQueryElement->u.pSaMad->Data);
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
								IB_NODE_RECORD_COMP_LID;
			pSaNodeRecordData->RID.s.LID = pQuery->InputValue.Lid;
			BSWAP_IB_NODE_RECORD(pSaNodeRecordData);
			status = FSUCCESS;
			break;
		}

		case InputTypeNodeDesc:
		{
			// limited list of valid output types
			// all are based on a NodeRecord query
			if (! (pQuery->OutputType == OutputTypeLid
					|| pQuery->OutputType == OutputTypeSystemImageGuid
					|| pQuery->OutputType == OutputTypeNodeGuid
					|| pQuery->OutputType == OutputTypePortGuid
					|| pQuery->OutputType == OutputTypeNodeDesc
					|| pQuery->OutputType == OutputTypeNodeRecord
					|| pQuery->OutputType == OutputTypePathRecord
				))
			{
				status = FINVALID_PARAMETER;
				break;
			}
			status = FillSADetailsForNodeRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
						IB_NODE_RECORD_COMP_NODEDESC;
			pSaNodeRecordData = (IB_NODE_RECORD*)&(pQueryElement->u.pSaMad->Data);

			if (pQuery->InputValue.NodeDesc.NameLength > NODE_DESCRIPTION_ARRAY_SIZE)
			{
				status = FINVALID_PARAMETER;
				break;
			}

			MemoryClear(pSaNodeRecordData->NodeDescData.NodeString,
					sizeof(NODE_DESCRIPTION));

			MemoryCopy(pSaNodeRecordData->NodeDescData.NodeString,
					&(pQuery->InputValue.NodeDesc.Name),
					pQuery->InputValue.NodeDesc.NameLength);

			// don't need to byte swap the Node Description!
			status = FSUCCESS;
			break;
		}

		case InputTypePortGuidPair:
		{
			if ( pQuery->OutputType != OutputTypePathRecord && pQuery->OutputType != OutputTypeTraceRecord)
			{
				status = FINVALID_PARAMETER;
				break;
			}

            if (pQuery->OutputType == OutputTypeTraceRecord)
                status = FillSADetailsForTraceRecord(pQueryElement, MemAllocFlags);
            else
    			status = FillSADetailsForPathRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
			
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
						IB_PATH_RECORD_COMP_DGID
							| IB_PATH_RECORD_COMP_SGID
							| IB_PATH_RECORD_COMP_REVERSIBLE
							| IB_PATH_RECORD_COMP_NUMBPATH;
			pSaPathRecordData = (IB_PATH_RECORD*)&(pQueryElement->u.pSaMad->Data);
			// use subnet prefix of SourcePort, if not local use our's
			pSaPathRecordData->DGID.Type.Global.SubnetPrefix =
			pSaPathRecordData->SGID.Type.Global.SubnetPrefix =
					GetSubnetPrefixForPortGuid(pQueryElement->PortGuid, 
							pQuery->InputValue.PortGuidPair.SourcePortGuid);

			pSaPathRecordData->DGID.Type.Global.InterfaceID =
							pQuery->InputValue.PortGuidPair.DestPortGuid;
			pSaPathRecordData->SGID.Type.Global.InterfaceID =
							pQuery->InputValue.PortGuidPair.SourcePortGuid;
			pSaPathRecordData->Reversible = 1;
			pSaPathRecordData->NumbPath = PATHRECORD_NUMBPATH;	// required

			BSWAP_IB_PATH_RECORD(pSaPathRecordData);
			status = FSUCCESS;
			break;
		}

		case InputTypeGidPair:
		{
			if ( pQuery->OutputType != OutputTypePathRecord && pQuery->OutputType != OutputTypeTraceRecord)
			{
				status = FINVALID_PARAMETER;
				break;
			}

            if (pQuery->OutputType == OutputTypeTraceRecord)
                status = FillSADetailsForTraceRecord(pQueryElement, MemAllocFlags);
            else
    			status = FillSADetailsForPathRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
			
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
						IB_PATH_RECORD_COMP_DGID
							| IB_PATH_RECORD_COMP_SGID
							| IB_PATH_RECORD_COMP_REVERSIBLE
							| IB_PATH_RECORD_COMP_NUMBPATH;
			pSaPathRecordData = (IB_PATH_RECORD*)&(pQueryElement->u.pSaMad->Data);
			pSaPathRecordData->DGID= pQuery->InputValue.GidPair.DestGid;
			pSaPathRecordData->SGID= pQuery->InputValue.GidPair.SourceGid;
			pSaPathRecordData->Reversible = 1;
			pSaPathRecordData->NumbPath = PATHRECORD_NUMBPATH;	// required

			BSWAP_IB_PATH_RECORD(pSaPathRecordData);
			status = FSUCCESS;
			break;
		}
		case InputTypePathRecord:
		{
			if (pQuery->OutputType != OutputTypePathRecord && pQuery->OutputType != OutputTypeTraceRecord)
			{
				status = FINVALID_PARAMETER;
				break;
			}

            if (pQuery->OutputType == OutputTypeTraceRecord)
                status = FillSADetailsForTraceRecord(pQueryElement, MemAllocFlags);
            else
    			status = FillSADetailsForPathRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
			pSaPathRecordData = (IB_PATH_RECORD*)&(pQueryElement->u.pSaMad->Data);
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
							pQuery->InputValue.PathRecordValue.ComponentMask;
			*pSaPathRecordData = pQuery->InputValue.PathRecordValue.PathRecord;
			BSWAP_IB_PATH_RECORD(pSaPathRecordData);
			status  = FSUCCESS;
			break;
		}

		case InputTypeServiceRecord:
		{
			IB_SERVICE_RECORD*  pSaServiceRecord;

			// make sure the output type is sane
			if ( pQuery->OutputType != OutputTypeServiceRecord)
			{
				status = FINVALID_PARAMETER;
				break;
			}

			// fill out the basic details of the packet request
			status = FillSADetailsForServiceRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}

			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
							pQuery->InputValue.ServiceRecordValue.ComponentMask;
			pSaServiceRecord = (IB_SERVICE_RECORD*)&(pQueryElement->u.pSaMad->Data);
			*pSaServiceRecord = pQuery->InputValue.ServiceRecordValue.ServiceRecord;
			BSWAP_IB_SERVICE_RECORD(pSaServiceRecord);
			status = FSUCCESS;
			break;
		}
		case InputTypeMcMemberRecord:
		{
			IB_MCMEMBER_RECORD*  pSaMcMemberRecord;

			// make sure the output type is sane
			if ( pQuery->OutputType != OutputTypeMcMemberRecord)
			{
				status = FINVALID_PARAMETER;
				break;
			}

			// fill out the basic details of the packet request
			status = FillSADetailsForMcMemberRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}

			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
						pQuery->InputValue.McMemberRecordValue.ComponentMask;
			pSaMcMemberRecord = (IB_MCMEMBER_RECORD*)&(pQueryElement->u.pSaMad->Data);
			*pSaMcMemberRecord = pQuery->InputValue.McMemberRecordValue.McMemberRecord;
			BSWAP_IB_MCMEMBER_RECORD(pSaMcMemberRecord);
			status = FSUCCESS;
			break;
		}
		case InputTypePortGuidList:
		{
			IB_MULTIPATH_RECORD *pSaMultiPathRecordData;
			uint8 i;

			if ( pQuery->OutputType != OutputTypePathRecord)
			{
				status = FINVALID_PARAMETER;
				break;
			}
			if ( 0 == pQuery->InputValue.PortGuidList.SourceGuidCount
				|| 0 == pQuery->InputValue.PortGuidList.DestGuidCount)
			{
				status = FINVALID_PARAMETER;
				break;
			}
			if ( pQuery->InputValue.PortGuidList.SourceGuidCount
				 + pQuery->InputValue.PortGuidList.DestGuidCount
				 > MULTIPATH_GID_LIMIT)
			{
				status = FINVALID_PARAMETER;
				break;
			}
			status = FillSADetailsForMultiPathRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
			
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
						IB_MULTIPATH_RECORD_COMP_DGIDCOUNT
							| IB_MULTIPATH_RECORD_COMP_SGIDCOUNT
							| IB_MULTIPATH_RECORD_COMP_REVERSIBLE
							| IB_MULTIPATH_RECORD_COMP_NUMBPATH
							| IB_MULTIPATH_RECORD_COMP_INDEPENDENCESELECTOR;
			pSaMultiPathRecordData = (IB_MULTIPATH_RECORD*)&(pQueryElement->u.pSaMad->Data);
			for (i =0; i< pQuery->InputValue.PortGuidList.SourceGuidCount
						+ pQuery->InputValue.PortGuidList.DestGuidCount; ++i)
			{
				// use subnet prefix of PortGuid, if not local use our's
				pSaMultiPathRecordData->GIDList[i].Type.Global.SubnetPrefix =
						GetSubnetPrefixForPortGuid(pQueryElement->PortGuid, 
								pQuery->InputValue.PortGuidList.GuidList[i]);

				pSaMultiPathRecordData->GIDList[i].Type.Global.InterfaceID =
								pQuery->InputValue.PortGuidList.GuidList[i];
			}
			pSaMultiPathRecordData->Reversible = 1;
			pSaMultiPathRecordData->NumbPath = PATHRECORD_NUMBPATH;	// required
			pSaMultiPathRecordData->IndependenceSelector = 1;
			pSaMultiPathRecordData->SGIDCount = pQuery->InputValue.PortGuidList.SourceGuidCount;
			pSaMultiPathRecordData->DGIDCount = pQuery->InputValue.PortGuidList.DestGuidCount;

			BSWAP_IB_MULTIPATH_RECORD(pSaMultiPathRecordData);
			status = FSUCCESS;
			break;
		}

		case InputTypeGidList:
		{
			IB_MULTIPATH_RECORD *pSaMultiPathRecordData;
			uint8 i;

			if ( pQuery->OutputType != OutputTypePathRecord)
			{
				status = FINVALID_PARAMETER;
				break;
			}
			if ( 0 == pQuery->InputValue.GidList.SourceGidCount
				|| 0 == pQuery->InputValue.GidList.DestGidCount)
			{
				status = FINVALID_PARAMETER;
				break;
			}
			if ( pQuery->InputValue.GidList.SourceGidCount
				 +pQuery->InputValue.GidList.DestGidCount
				 > MULTIPATH_GID_LIMIT)
			{
				status = FINVALID_PARAMETER;
				break;
			}
			status = FillSADetailsForMultiPathRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
			
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
						IB_MULTIPATH_RECORD_COMP_DGIDCOUNT
							| IB_MULTIPATH_RECORD_COMP_SGIDCOUNT
							| IB_MULTIPATH_RECORD_COMP_REVERSIBLE
							| IB_MULTIPATH_RECORD_COMP_NUMBPATH
							| IB_MULTIPATH_RECORD_COMP_INDEPENDENCESELECTOR;
			pSaMultiPathRecordData = (IB_MULTIPATH_RECORD*)&(pQueryElement->u.pSaMad->Data);
			for (i =0; i< pQuery->InputValue.GidList.SourceGidCount
				 			+ pQuery->InputValue.GidList.DestGidCount; ++i)
			{
				pSaMultiPathRecordData->GIDList[i]= pQuery->InputValue.GidList.GidList[i];
			}
			pSaMultiPathRecordData->Reversible = 1;
			pSaMultiPathRecordData->NumbPath = PATHRECORD_NUMBPATH;	// required
			pSaMultiPathRecordData->IndependenceSelector = 1;
			pSaMultiPathRecordData->SGIDCount = pQuery->InputValue.GidList.SourceGidCount;
			pSaMultiPathRecordData->DGIDCount = pQuery->InputValue.GidList.DestGidCount;

			BSWAP_IB_MULTIPATH_RECORD(pSaMultiPathRecordData);
			status = FSUCCESS;
			break;
		}
		case InputTypeMultiPathRecord:
		{
			IB_MULTIPATH_RECORD *pSaMultiPathRecordData;

			if ( pQuery->OutputType != OutputTypePathRecord)
			{
				status = FINVALID_PARAMETER;
				break;
			}
			status = FillSADetailsForMultiPathRecord(pQueryElement, MemAllocFlags);
			if ( status != FSUCCESS)
			{
				break;
			}
			pSaMultiPathRecordData = (IB_MULTIPATH_RECORD*)&(pQueryElement->u.pSaMad->Data);
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
							pQuery->InputValue.MultiPathRecordValue.ComponentMask;
			*pSaMultiPathRecordData = pQuery->InputValue.MultiPathRecordValue.MultiPathRecord;
			BSWAP_IB_MULTIPATH_RECORD(pSaMultiPathRecordData);
			status  = FSUCCESS;
			break;
		}

		default:
			status = FINVALID_PARAMETER;
			break;
		
	}
	if (FSUCCESS == status)
	{
		BSWAP_SA_HDR(&pQueryElement->u.pSaMad->SaHdr);
	}
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return status;
}

FSTATUS
ValidateAndFillSDFabOp(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS 			status = FINVALID_PARAMETER;
	FABRIC_OPERATION_DATA *pFabOp;
	MAD*				pMad;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ValidateAndFillSDFabOp);

	pFabOp = &pQueryElement->cmd.op.FabOp;

	_DBG_INFO(("pFabOp->Type = 0x%x\n", pFabOp->Type));
	switch(pFabOp->Type) 
	{
	case FabOpSetServiceRecord:
	case FabOpDeleteServiceRecord:
		{
		IB_SERVICE_RECORD*  pSaServiceRecord;

		status = FillSADetailsForServiceRecord(pQueryElement, MemAllocFlags);
		if ( status != FSUCCESS)
		{
			break;
		}

		pMad	= pQueryElement->u.pMad;
		if (pFabOp->Type == FabOpSetServiceRecord)
		{
			pMad->common.mr.s.Method = SUBN_ADM_SET;
			pQueryElement->u.pSaMad->SaHdr.ComponentMask = 0;	// unused
		} else {
			pMad->common.mr.s.Method = SUBN_ADM_DELETE;
			pQueryElement->u.pSaMad->SaHdr.ComponentMask =
							pFabOp->Value.ServiceRecordValue.ComponentMask;
		}
		pSaServiceRecord = (IB_SERVICE_RECORD*)&(pQueryElement->u.pSaMad->Data);
		*pSaServiceRecord = pFabOp->Value.ServiceRecordValue.ServiceRecord;
		BSWAP_IB_SERVICE_RECORD(pSaServiceRecord);
		status = FSUCCESS;
		break;
		}
	case FabOpSetMcMemberRecord:
	case FabOpDeleteMcMemberRecord:
		{
		IB_MCMEMBER_RECORD*  pSaMcMemberRecord;

		status = FillSADetailsForMcMemberRecord(pQueryElement, MemAllocFlags);
		if ( status != FSUCCESS)
		{
			break;
		}

		pMad	= pQueryElement->u.pMad;
		pMad->common.mr.s.Method = (pFabOp->Type == FabOpSetMcMemberRecord)
										? SUBN_ADM_SET : SUBN_ADM_DELETE;
		pQueryElement->u.pSaMad->SaHdr.ComponentMask =
							pFabOp->Value.McMemberRecordValue.ComponentMask;
		pSaMcMemberRecord = (IB_MCMEMBER_RECORD*)&(pQueryElement->u.pSaMad->Data);
		*pSaMcMemberRecord = pFabOp->Value.McMemberRecordValue.McMemberRecord;
		BSWAP_IB_MCMEMBER_RECORD(pSaMcMemberRecord);
		status = FSUCCESS;
		break;
		}
	case FabOpJoinMcGroup:
	case FabOpLeaveMcGroup:
		{
		IB_MCMEMBER_RECORD*  pSaMcMemberRecord;

		status = FillSADetailsForMcMemberRecord(pQueryElement, MemAllocFlags);
		if ( status != FSUCCESS)
		{
			break;
		}

		pMad	= pQueryElement->u.pMad;
		pMad->common.mr.s.Method = SUBN_ADM_SET;
		pMad->common.mr.s.Method = (pFabOp->Type == FabOpJoinMcGroup)
										?SUBN_ADM_SET:SUBN_ADM_DELETE;
		pQueryElement->u.pSaMad->SaHdr.ComponentMask =
									IB_MCMEMBER_RECORD_COMP_MGID
									| IB_MCMEMBER_RECORD_COMP_PORTGID
									| IB_MCMEMBER_RECORD_COMP_JOINSTATE;
		pSaMcMemberRecord = (IB_MCMEMBER_RECORD*)&(pQueryElement->u.pSaMad->Data);
		pSaMcMemberRecord->RID.MGID = pFabOp->Value.McJoinLeave.MGID;
		pSaMcMemberRecord->RID.PortGID.Type.Global.SubnetPrefix =
					GetSubnetPrefixForPortGuid(pQueryElement->PortGuid, 
											pQueryElement->PortGuid);
		pSaMcMemberRecord->RID.PortGID.Type.Global.InterfaceID =
											pQueryElement->PortGuid;
		pSaMcMemberRecord->JoinFullMember = pFabOp->Value.McJoinLeave.JoinFullMember;
		pSaMcMemberRecord->JoinNonMember = pFabOp->Value.McJoinLeave.JoinNonMember;
		pSaMcMemberRecord->JoinSendOnlyMember = pFabOp->Value.McJoinLeave.JoinSendOnlyMember;
		BSWAP_IB_MCMEMBER_RECORD(pSaMcMemberRecord);
		status = FSUCCESS;
		break;
		}
	case FabOpSetInformInfo:
		{
		IB_INFORM_INFO*  pSaInformInfo;

		status = FillSADetailsForInformInfo(pQueryElement, MemAllocFlags);
		if ( status != FSUCCESS)
		{
			break;
		}

		pMad	= pQueryElement->u.pMad;
		pMad->common.mr.s.Method = SUBN_ADM_SET;
		// ComponentMask not used
		pSaInformInfo = (IB_INFORM_INFO*)&(pQueryElement->u.pSaMad->Data);
		*pSaInformInfo = pFabOp->Value.InformInfo;
		BSWAP_INFORM_INFO(pSaInformInfo);
		status = FSUCCESS;
		break;
		}
	default:
		status = FINVALID_PARAMETER;
		break;
	}
	if (FSUCCESS == status)
	{
		BSWAP_SA_HDR(&pQueryElement->u.pSaMad->SaHdr);
	}
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return status;
}

FSTATUS
FillSADetailsForNodeRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForNodeRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_NODE_RECORD,
								sizeof(IB_NODE_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForPortInfoRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForPortInfoRecord);
 
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_PORTINFO_RECORD,
								sizeof(IB_PORTINFO_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForSMInfoRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForSMInfoRecord);
 
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_SMINFO_RECORD,
								sizeof(IB_SMINFO_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForLinkRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForLinkRecord);
 
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_LINK_RECORD,
								sizeof(IB_LINK_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForPathRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForPathRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_PATH_RECORD,
								sizeof(IB_PATH_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForServiceRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForServiceRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_SERVICE_RECORD,
								sizeof(IB_SERVICE_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForMcMemberRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForMcMemberRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_MCMEMBER_RECORD,
								sizeof(IB_MCMEMBER_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForInformInfoRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForInformInfoRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_INFORM_INFO_RECORD,
								sizeof(IB_INFORM_INFO_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForInformInfo(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForInformInfo);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_INFORM_INFO,
								sizeof(IB_INFORM_INFO), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForMultiPathRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForMultiPathRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_MULTIPATH_RECORD,
								sizeof(IB_MULTIPATH_RECORD), MemAllocFlags);
	pQueryElement->u.pMad->common.mr.s.Method = SUBN_ADM_GETMULTI;
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForTraceRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForTraceRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_TRACE_RECORD,
								sizeof(IB_PATH_RECORD), MemAllocFlags);
	pQueryElement->u.pMad->common.mr.s.Method = SUBN_ADM_GETTRACETABLE;
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForSwitchInfoRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForSwitchInfoRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_SWITCHINFO_RECORD,
								sizeof(IB_SWITCHINFO_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForLinearFDBRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForLinearFDBRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_LINEAR_FWDTBL_RECORD,
								sizeof(IB_LINEAR_FDB_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForRandomFDBRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForRandomFDBRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_RANDOM_FWDTBL_RECORD,
								sizeof(IB_RANDOM_FDB_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForMCastFDBRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForMCastFDBRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_MCAST_FWDTBL_RECORD,
								sizeof(IB_MCAST_FDB_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForVLArbTableRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForVLArbTableRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_VLARBTABLE_RECORD,
								sizeof(IB_VLARBTABLE_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

FSTATUS
FillSADetailsForPKeyTableRecord(
	PQueryDetails pQueryElement,
	uint32 MemAllocFlags
	)
{
	FSTATUS Fstatus;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillSADetailsForPKeyTableRecord);
	
	Fstatus = BuildCommonSAMad(pQueryElement, SA_ATTRIB_P_KEY_TABLE_RECORD,
								sizeof(IB_P_KEY_TABLE_RECORD), MemAllocFlags);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Fstatus;
}

// common Mad, RMPP and SAHdr for a SUBN_ADM GETTABLE
void
FillCommonSADetails(
	PQueryDetails pQueryElement,
	IN uint16		 AttribId,
	IN uint32		 RecordSize
	)
{
	MAD *mad;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FillCommonSADetails);
	
	pQueryElement->IsSelfCommand = FALSE;
	// caller has filed in pQueryElement->SdSentCmd
	DEBUG_ASSERT(pQueryElement->SdSentCmd != UnknownCommand);

	FillBaseGmpValues(pQueryElement);

	mad = pQueryElement->u.pMad;
	mad->common.mr.s.R = 0;		//since this is not a response, set it to 0;
	mad->common.mr.s.Method = SUBN_ADM_GETTABLE; 
	MAD_SET_ATTRIB_ID(pQueryElement->u.pMad, AttribId);
	MAD_SET_ATTRIB_MOD(mad, 0);	// unused in IBA1.1

	// 1 segment RMPP
	// if ComponentMask is needed caller can set after this call
	MemoryClear(&(pQueryElement->u.pSaMad->RmppHdr), sizeof(RMPP_HEADER));
	MemoryClear(&(pQueryElement->u.pSaMad->SaHdr), sizeof(SA_HDR));
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return;
}

// Allocate and Build
// common Mad, RMPP and SAHdr for a SUBN_ADM GETTABLE
FSTATUS
BuildCommonSAMad(
	IN PQueryDetails pQueryElement,
	IN uint16		 AttribId,
	IN uint32		 RecordSize,
	IN uint32		 MemAllocFlags
	)
{
	// Allocate memory for the GMP with NodeRecord as payload
	pQueryElement->u.pSaMad = (SA_MAD*)MemoryAllocate2AndClear(sizeof(SA_MAD),
							MemAllocFlags, SUBNET_DRIVER_TAG);
	if (pQueryElement->u.pSaMad == NULL)
	{
		return	FINSUFFICIENT_MEMORY;
		
	}
	FillCommonSADetails(pQueryElement, AttribId, RecordSize);

	pQueryElement->TotalBytesInGmp = (pQueryElement->u.pSaMad->common.BaseVersion == IB_BASE_VERSION)
		? IB_MAD_BLOCK_SIZE
		: (sizeof(SA_MAD) - STL_SUBN_ADM_DATASIZE) + RecordSize;

	return FSUCCESS;
}
