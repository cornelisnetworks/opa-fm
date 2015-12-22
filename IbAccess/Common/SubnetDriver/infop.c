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
#include <ilist.h>
#include <ievent.h>
#include <stl_sd.h>
#include <sdi.h>
#include <query.h>
#include <subscribe.h>
#include <multicast.h>

extern uint32 SADisable;

ATOMIC_UINT Outstanding;	// number of queries/operations in progress

uint32 NumLocalCAs = 0;
uint32 NumTotalPorts = 0;
EUI64 *pLocalCaGuidList = NULL;
IB_CA_ATTRIBUTES *pLocalCaInfo = NULL;
SPIN_LOCK *pLocalCaInfoLock = NULL; 
IB_HANDLE CaNotifyHandle = NULL;
SPIN_LOCK *pCaPortListLock = NULL;
QUICK_LIST *pCaPortList = NULL;

LIST_ITEM*
GetCaPortListItem(
	EUI64 PortGuid
	)
{
	LIST_ITEM *pCaPortListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, GetCaPortListItem);

	_DBG_INFO(("Searching for Ca Port (GUID: %"PRIx64").\n", PortGuid));

	for (pCaPortListItem = QListHead(pCaPortList);
		pCaPortListItem != NULL;
		pCaPortListItem = QListNext(pCaPortList, pCaPortListItem)
		)
	{
		CA_PORT *pCaPort = (CA_PORT *) QListObj(pCaPortListItem);
		ASSERT(pCaPort != NULL);

		if (pCaPort->PortGuid == PortGuid)
		{
			_DBG_INFO(("Found Ca Port (GUID: %"PRIx64").\n", PortGuid));
			_DBG_INFO(("SubnetAdm   Ca Port (GUID: %"PRIx64").\n", pCaPort->SaPortGuid));
			_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
			return pCaPortListItem;
		}
	}

	_DBG_INFO(("Ca Port (GUID: %"PRIx64") not found.\n", PortGuid));
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return NULL;
}

CA_PORT*
GetCaPort(
	EUI64 PortGuid
	)
{
	LIST_ITEM *pCaPortListItem = GetCaPortListItem(PortGuid);
	CA_PORT *pCaPort = NULL;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, GetCaPort);

	if (pCaPortListItem != NULL)
	{
		pCaPort = (CA_PORT *) QListObj(pCaPortListItem);
		ASSERT(pCaPort != NULL);
	}

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return pCaPort;
}

void
AddCa(
	EUI64 CaGuid
	)
{
	IB_CA_ATTRIBUTES *pNewCaInfo = NULL;
	uint32 ii;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, AddCa);

	_DBG_INFO(("Adding Ports for Channel Adapter (GUID: %"PRIx64").\n",
		CaGuid));

	if (pLocalCaInfo != NULL)
	{
		SpinLockAcquire(pLocalCaInfoLock);

		_DBG_INFO(("NumLocalCAs (%d).\n", NumLocalCAs));

		for (ii = 0; ii < NumLocalCAs; ii++)
		{
			if (pLocalCaInfo[ii].GUID == CaGuid)
			{
				_DBG_INFO(
					("Channel Adapter (GUID: %"PRIx64") Found @ Index (%d).\n",
					pLocalCaInfo[ii].GUID, ii));
				pNewCaInfo = &pLocalCaInfo[ii];
				break;
			}
		}

		if (pNewCaInfo != NULL)
		{
			IB_PORT_ATTRIBUTES	*pPortAttr;

			_DBG_INFO(("Channel Adapter (GUID: %"PRIx64") Port Count (%d).\n",
				pNewCaInfo->GUID, pNewCaInfo->Ports));

			for (ii = 0,  pPortAttr = pNewCaInfo->PortAttributesList;
				ii < pNewCaInfo->Ports;
				ii++, pPortAttr = pPortAttr->Next
				)
			{
				CA_PORT* pCaPort = (CA_PORT*)MemoryAllocateAndClear(sizeof(CA_PORT),
					TRUE, SUBNET_DRIVER_TAG);
				ASSERT(pCaPort != NULL);

				pCaPort->CaGuid = CaGuid;
				pCaPort->PortGuid = pPortAttr->GUID;
				if (! LookupPKey(pPortAttr, DEFAULT_P_KEY, TRUE, &pCaPort->DefaultPkeyIndex))
					pCaPort->DefaultPkeyIndex = 0;
				QListSetObj(&pCaPort->CaPortListItem, pCaPort);

				SpinLockAcquire(pCaPortListLock);

				QListInsertTail(pCaPortList, &pCaPort->CaPortListItem);

				_DBG_INFO(("Added Port (Number %d) (GUID: %"PRIx64
					") (Channel Adapter GUID: %"PRIx64").\n", 
					ii, 
					pCaPort->PortGuid,
					pCaPort->CaGuid));

				SpinLockRelease(pCaPortListLock);

				InitializeCaPortInfo(pPortAttr->GUID);
			}
		}

		SpinLockRelease(pLocalCaInfoLock);
	}

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

void
RemoveCa(
	EUI64 CaGuid
	)
{
	LIST_ITEM *pCaPortListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, RemoveCa);

	SpinLockAcquire(pCaPortListLock);

	pCaPortListItem = QListHead(pCaPortList);

	while (pCaPortListItem != NULL)
	{
		CA_PORT *pCaPort = (CA_PORT *) QListObj(pCaPortListItem);
		ASSERT(pCaPort != NULL);

		pCaPortListItem = QListNext(pCaPortList, pCaPortListItem);

		if (pCaPort->CaGuid == CaGuid)
		{
			QListRemoveItem(pCaPortList, &pCaPort->CaPortListItem);
			MemoryDeallocate(pCaPort);
		}
	}

	SpinLockRelease(pCaPortListLock);

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
}

uint32
GetNumTotalPorts(
	IN void
	)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, GetNumTotalPorts);
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return (NumTotalPorts);
}

FSTATUS
iba_sd_get_local_port_guids_alloc(
	IN OUT uint64 **ppLocalPortGuidsList,
	IN OUT uint32 *LocalPortGuidsCount
	)
{
	return iba_sd_get_local_port_guids_alloc2(ppLocalPortGuidsList, NULL,
							LocalPortGuidsCount);
}

FSTATUS
iba_sd_get_local_port_guids_alloc2(
	IN OUT uint64 **ppLocalPortGuidsList,
	IN OUT uint64 **ppLocalPortSubnetPrefixList,
	IN OUT uint32 *LocalPortGuidsCount
	)
{
	FSTATUS Fstatus = FNOT_DONE;
	LIST_ITEM *pCaPortListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_get_local_port_guids_alloc2);

	if (ppLocalPortGuidsList) {
		*ppLocalPortGuidsList = NULL;
	} else {
		Fstatus = FINVALID_PARAMETER;
		goto done;
	}

	if (ppLocalPortSubnetPrefixList)
		*ppLocalPortSubnetPrefixList = NULL;

	if (LocalPortGuidsCount) {
		*LocalPortGuidsCount = 0;
	} else {
		Fstatus = FINVALID_PARAMETER;
		goto done;
	}

	if (NumTotalPorts == 0)
	{
		_DBG_INFO(("No LocalPortGuids allocated.\n"));
		Fstatus = FSUCCESS;
		goto done;
	}

	*ppLocalPortGuidsList = 
		(EUI64*)MemoryAllocate2AndClear((sizeof(EUI64)*NumTotalPorts),
						IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
	if (*ppLocalPortGuidsList == NULL)
	{
		_DBG_ERROR(("Cannot allocate memory for LocalPortGuids\n"));
		Fstatus = FINSUFFICIENT_MEMORY;
		goto done;
	}
	if (ppLocalPortSubnetPrefixList)
	{
		*ppLocalPortSubnetPrefixList = 
			(EUI64*)MemoryAllocate2AndClear((sizeof(EUI64)*NumTotalPorts),
						IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
		if (*ppLocalPortSubnetPrefixList == NULL)
		{
			_DBG_ERROR(("Cannot allocate memory for LocalPortSubnetPrefix\n"));
			Fstatus = FINSUFFICIENT_MEMORY;
			goto fail;
		}
	}

	SpinLockAcquire(pCaPortListLock);

	for (pCaPortListItem = QListHead(pCaPortList);
		pCaPortListItem != NULL;
		pCaPortListItem = QListNext(pCaPortList, pCaPortListItem)
		)
	{
		IB_PORT_ATTRIBUTES *pPortAttr;
		FSTATUS status;

		CA_PORT *pCaPort = (CA_PORT *) QListObj(pCaPortListItem);
		ASSERT(pCaPort != NULL);

		status = iba_query_port_by_guid_alloc(pCaPort->PortGuid, &pPortAttr);
		if (FSUCCESS != status)
		{
			Fstatus = status;
			break;
		}

		_DBG_INFO(("The port state is %d\n",pPortAttr->PortState));
		_DBG_INFO(("BaseLid <%d>\n",pPortAttr->Address.BaseLID));
		if ((pPortAttr->Address.BaseLID != /*IB_INVALID_LID*/ 0) &&
			(pPortAttr->PortState == PortStateActive))
		{
			//NOTENOTE: Return only active ports
			(*ppLocalPortGuidsList)[*LocalPortGuidsCount] = pPortAttr->GUID;
			if (ppLocalPortSubnetPrefixList)
				(*ppLocalPortSubnetPrefixList)[*LocalPortGuidsCount] = pPortAttr->GIDTable[0].Type.Global.SubnetPrefix;
			(*LocalPortGuidsCount)++;
			Fstatus = FSUCCESS;
		}

		MemoryDeallocate(pPortAttr);
	}

	SpinLockRelease(pCaPortListLock);

fail:
	if (Fstatus != FSUCCESS) {
		if (*ppLocalPortGuidsList != NULL) {
			MemoryDeallocate(*ppLocalPortGuidsList);
			*ppLocalPortGuidsList = 0;
		}
		*LocalPortGuidsCount = 0;
		if (ppLocalPortSubnetPrefixList && *ppLocalPortSubnetPrefixList)
		{
			MemoryDeallocate(*ppLocalPortSubnetPrefixList);
			*ppLocalPortSubnetPrefixList = 0;
		}
		// enable this code in the future,
		// for now for backward compatibility
		// return an error code for no ports found (FNOT_DONE above)
		//if (Fstatus == FNOT_DONE)
		//	Fstatus = FSUCCESS;
	}

done:
	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
	return Fstatus;
}


FSTATUS
GetLocalCaInfo(
	void
	)
{
	FSTATUS Fstatus;
	uint32 ii;
	uint32 OldNumLocalCAs = NumLocalCAs;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, GetLocalCaInfo);

	// This function has been called by holding a pLocalCaInfoLock spinlock

	if (pLocalCaGuidList)  
	{
		MemoryDeallocate(pLocalCaGuidList);
		pLocalCaGuidList = NULL;
	}

	NumLocalCAs = 0;
	Fstatus = iba_get_caguids_alloc(&NumLocalCAs, &pLocalCaGuidList);
	if (Fstatus != FSUCCESS) 
	{
		_DBG_ERROR(("Fstatus <0x%x> getting local CA GUIDs\n", Fstatus));
		pLocalCaGuidList = NULL;
		NumLocalCAs = 0;
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return(Fstatus);
	}

	_DBG_INFO(("Number of Channel Adapters present <%d>\n", NumLocalCAs));
	if (NumLocalCAs <= 0) {
		_DBG_INFO(("No CA reported by GetCaGuids\n"));
		return FSUCCESS;
	}

	if (pLocalCaInfo)  
	{
		uint32 count;
		for(count=0; count<OldNumLocalCAs; count++)
		{
			if(pLocalCaInfo[count].PortAttributesList)
				MemoryDeallocate(pLocalCaInfo[count].PortAttributesList);
		}
		MemoryDeallocate(pLocalCaInfo);
	}
	pLocalCaInfo = (IB_CA_ATTRIBUTES*)MemoryAllocateAndClear(
		(sizeof(IB_CA_ATTRIBUTES) * NumLocalCAs), FALSE, SUBNET_DRIVER_TAG);
	if (pLocalCaInfo == NULL)  
	{
		_DBG_ERROR(("Cannot allocate memory for local CA info\n"));
		MemoryDeallocate(pLocalCaGuidList);
		pLocalCaGuidList = NULL;
		NumLocalCAs = 0;
		_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
		return(FINSUFFICIENT_MEMORY);
	}

	//Clear the number of Ports
	NumTotalPorts = 0;
	for (ii=0; ii<NumLocalCAs; ++ii)  
	{
		Fstatus = iba_query_ca_by_guid_alloc(pLocalCaGuidList[ii], &pLocalCaInfo[ii]);
		if (Fstatus == FSUCCESS) 
		{
			NumTotalPorts += pLocalCaInfo[ii].Ports;
		} else {
			_DBG_ERROR(("Fstatus 0x%x Query CA by Guid failed.\n", Fstatus));
		}
	} //end for(; ; ;);

	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
	return(FSUCCESS);
}


void
LocalCaChangeCallback(
	IB_NOTIFY_RECORD NotifyRecord
	)
{
	FSTATUS Status;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, LocalCaChangeCallback);

	if (SdUnloading)
	{
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return;
	}

	//
	// re-collect the information only if there is a change that impacts us.
	//

	SpinLockAcquire(pLocalCaInfoLock);
	Status = GetLocalCaInfo();
	SpinLockRelease(pLocalCaInfoLock);
	if (Status != FSUCCESS)  
	{
		_DBG_ERROR(
			("GetLocalCaInfo failed during localcachange callback<%d>\n",
			Status));
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
		return;
	}

	switch (NotifyRecord.EventType)
	{
	case IB_NOTIFY_CA_ADD:
		_DBG_INFO(("IB_NOTIFY_CA_ADD (GUID: %"PRIx64").\n",
			NotifyRecord.Guid));
		AddCa(NotifyRecord.Guid);
		break;

	case IB_NOTIFY_CA_REMOVE:
		_DBG_INFO(("IB_NOTIFY_CA_REMOVE (GUID: %"PRIx64").\n",
			NotifyRecord.Guid));
		RemoveCa(NotifyRecord.Guid);
		break;

	case IB_NOTIFY_LID_EVENT:
		_DBG_INFO(("IB_NOTIFY_LID_EVENT (GUID: %"PRIx64").\n",
			NotifyRecord.Guid));
		InitializeCaPortInfo(NotifyRecord.Guid);
		ReRegisterTrapSubscriptions(NotifyRecord.Guid);
		break;

	case IB_NOTIFY_SM_EVENT:
		_DBG_INFO(("IB_NOTIFY_SM_EVENT (GUID: %"PRIx64").\n",
			NotifyRecord.Guid));
		InitializeCaPortInfo(NotifyRecord.Guid);
		ReRegisterTrapSubscriptions(NotifyRecord.Guid);
		break;

	case IB_NOTIFY_PORT_DOWN:
		_DBG_INFO(("IB_NOTIFY_PORT_DOWN (GUID: %"PRIx64").\n",
			NotifyRecord.Guid));
		InitializeCaPortInfo(NotifyRecord.Guid);
		InvalidatePortTrapSubscriptions(NotifyRecord.Guid);
		break;

	case IB_NOTIFY_PKEY_EVENT:
		_DBG_INFO(("IB_NOTIFY_PKEY_EVENT (GUID: %"PRIx64").\n",
			NotifyRecord.Guid));
		InitializeCaPortInfo(NotifyRecord.Guid);
		ReRegisterTrapSubscriptions(NotifyRecord.Guid);
		break;

	default:
		_DBG_INFO(("Someother Event (%lu) (GUID: %"PRIx64").\n", 
			NotifyRecord.EventType, 
			NotifyRecord.Guid));
		break;
	}
#ifdef USE_SD_MULTICAST	
	McPortNotifyCallback(&NotifyRecord);
#endif	
	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);

	return;
}

FSTATUS
InitializeCaPortInfo(
	EUI64 PortGuid
	)
{
	FSTATUS Fstatus = FERROR;
	CA_PORT *pCaPort;
	IB_PORT_ATTRIBUTES	*pPortAttr = NULL;
	boolean bInitialized;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, InitializeCaPortInfo);

	if (SdUnloading)
	{
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return(FINVALID_STATE);
	}
	_DBG_INFO(("Number of Local CA are %d\n",NumLocalCAs));

	SpinLockAcquire(pCaPortListLock);

	pCaPort = GetCaPort(PortGuid);
	if (pCaPort == NULL)
	{
		SpinLockRelease(pCaPortListLock);
		/* there is a rare possibility during shutdown that the port could go
		 * up/down after the Subnet Driver Remove Device has been called but
		 * before SMARemoveDevice is called.  In which case IbtNotifyGroup is
		 * still enabled for the given CA and the SubnetDriver could get a port
		 * state change which will put is here.  In this rare case we
		 * could fail to find the CaGuid and should ignore the callback
		 */
		_DBG_INFO(("GetCaPort returned NULL for Port (GUID: %"PRIx64").\n", PortGuid));
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return (Fstatus);
	}

	//
	// Note: The calling function should not fail driver load if this
	// function returns error. It is possible that this function is called
	// before the local CA has been initialized by a remote subnet manager
	// In that case, this function will fail but will register for a local
	// CA change callback. When the local CA does get initialized, this 
	// registered callback will be invoked and the callback will call this
	// function again.
	//

	//
	// LID for the information provider is initially set to the LID of the
	// subnet manager. This can be subsequently redirected. Get the SM LID
	// using local IBT calls
	//

	Fstatus = iba_query_port_by_guid_alloc(PortGuid, &pPortAttr);
	ASSERT(Fstatus == FSUCCESS);

	_DBG_INFO(("The port state is %d\n",pPortAttr->PortState));
	_DBG_INFO(("BaseLid <%d>\n",pPortAttr->Address.BaseLID));

	if (! LookupPKey(pPortAttr, DEFAULT_P_KEY, TRUE, &pCaPort->DefaultPkeyIndex))
		pCaPort->DefaultPkeyIndex = 0;

	pCaPort->Lid = pPortAttr->SMAddress.LID;
	pCaPort->ServiceLevel = pPortAttr->SMAddress.ServiceLevel;
	pCaPort->RemoteQp = GSI_QP;
	pCaPort->RemoteQKey = GSI_QKEY;
	// IB does not have Static rate in UD LRH
	// so we can't be sure what rate of remote port is
	// we use a constant value for GSI QPs
	pCaPort->StaticRate = IB_STATIC_RATE_GSI;
	pCaPort->PkeyIndex = pCaPort->DefaultPkeyIndex;
	pCaPort->SaPortGuid = pPortAttr->GUID;
	if(pPortAttr->NumGIDs > 0)
		pCaPort->SubnetPrefix = pPortAttr->GIDTable[0].Type.Global.SubnetPrefix;
	// default redirect to default location
	pCaPort->RedirectedQP = pCaPort->RemoteQp;
	pCaPort->RedirectedQKey = pCaPort->RemoteQKey;
	pCaPort->RedirectedLid = pCaPort->Lid;
	pCaPort->RedirectedSL = pCaPort->ServiceLevel;
	pCaPort->RedirectedStaticRate = pCaPort->StaticRate;
	pCaPort->RedirectedPkeyIndex = pCaPort->PkeyIndex;

	if ((pPortAttr->Address.BaseLID != /*IB_INVALID_LID*/ 0) &&
		(pPortAttr->PortState == PortStateActive))
		//NOTENOTE: Check for pPortAttr->SMAddress.LID != 0
	{
		pCaPort->SdSMAddressValid = TRUE;
		Fstatus = FSUCCESS;
	}

	// Free the port attributes buffer.
	MemoryDeallocate( pPortAttr );

	if (Fstatus != FSUCCESS)  
	{	
		//
		// It is possible that local ports have not yet been initialized by
		// a remote subnet manager. This is OK - we have registered for a
		// callback when that happens and we will initialize the service
		// provider at that time
		//        

		pCaPort->bInitialized = FALSE;

		_DBG_INFO(("No local port currently available\n"));
		pCaPort->SdSMAddressValid = FALSE;

		SpinLockRelease(pCaPortListLock);

		//
		// We haven't collected enough information to send any message to the
		// information provider, so simply return for now.
		//
		_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
		return(Fstatus);
	}

	bInitialized = pCaPort->bInitialized;

	SpinLockRelease(pCaPortListLock);

	if (! SADisable)
	{
		if(bInitialized == FALSE)
			SubnetAdmInit();
	}

	_DBG_LEAVE_LVL(_DBG_LVL_FUNC_TRACE);
	return(FSUCCESS);
}
