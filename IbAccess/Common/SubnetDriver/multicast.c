/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/
/* [ICS VERSION STRING: unknown] */

#include "multicast.h"

#include <imemory.h>
#include <ilist.h>
#include <ievent.h>
#include <ispinlock.h>
#include <stl_sd.h>
#include <sdi.h>
#include <ib_generalServices.h>
#include <itimer.h>
#include <dbg.h>
#include <ib_debug.h>
#include <subscribe.h>

#include "ipackon.h"

typedef struct _MC_RID
{
	IB_GID MGID;
	IB_GID PortGID;
} PACK_SUFFIX MC_RID;

typedef struct _MC_GROUP_ID
{
	int JoinVersion;
	MC_RID McRID;
	EUI64 EgressPortGUID;
} MC_GROUP_ID;

typedef struct _MC_GROUP
{
	LIST_ITEM             ListItem;
	MC_GROUP_STATE        McGroupState;
	TIMER                 Timer;
	boolean               TimerActive;
	uint64                ComponentMask;
	MC_GROUP_ID           McGroupId;
	IB_MCMEMBER_RECORD    McMemberRecord;
	FABRIC_OPERATION_DATA FabOp;
	QUERY                 FabInfoQuery;
	QUICK_LIST            McClientList;
	uint32                McGroupInUse;
	boolean               McGroupDelete;
} MC_GROUP;

typedef struct _MC_CLIENT
{
	LIST_ITEM             ListItem;
	LIST_ITEM             McGroupListItem;
	CLIENT_HANDLE         SdClientHandle;
	uint16                McFlags;
	MC_GROUP              *pMcGroup;
	void                  *pContext;
	SD_MULTICAST_CALLBACK *pMulticastCallback;
	uint32                McClientInUse;
	boolean               McClientDelete;
} MC_CLIENT;

static CLIENT_HANDLE McSdHandle;

static QUICK_LIST    MasterMcGroupList;
static QUICK_LIST    MasterMcClientList;

static SPIN_LOCK     MulticastLock;

static TIMER         MaintenanceTimer;
static boolean       MaintenanceTimerActivated;

extern uint32 NumTotalPorts;

/* Assumes MulticastLock is Held */
static
void
McLockAllGroups(void)
{
	LIST_ITEM *pListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McLockAllGroups);
	
	for (pListItem = QListHead(&MasterMcGroupList);
	     pListItem != NULL;
	     pListItem = QListNext(&MasterMcGroupList, pListItem))
	{
		MC_GROUP *pMcGroup = (MC_GROUP *)QListObj(pListItem);
		
		ASSERT(pMcGroup != NULL);
		
		pMcGroup->McGroupInUse++;
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}


/* Assumes MulticastLock is Held */
static
void
McUnlockAllGroups(void)
{
	LIST_ITEM *pListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McUnlockAllGroups);
	
	for (pListItem = QListHead(&MasterMcGroupList);
	     pListItem != NULL;
	     pListItem = QListNext(&MasterMcGroupList, pListItem))
	{
		MC_GROUP *pMcGroup = (MC_GROUP *)QListObj(pListItem);
		
		ASSERT(pMcGroup != NULL);
		
		pMcGroup->McGroupInUse--;
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

/* Assumes MulticastLock is Held */
static
void
McLockAllGroupClients(MC_GROUP *pMcGroup)
{
	LIST_ITEM *pListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McLockAllGroupClients);
	
	for (pListItem = QListHead(&pMcGroup->McClientList);
	     pListItem != NULL;
	     pListItem = QListNext(&pMcGroup->McClientList, pListItem))
	{
		MC_CLIENT *pMcClient = (MC_CLIENT *)QListObj(pListItem);
		
		ASSERT(pMcClient != NULL);
		
		pMcClient->McClientInUse++;
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

/* Assumes MulticastLock is Held */
static
void
McUnlockAllGroupClients(MC_GROUP *pMcGroup)
{
	LIST_ITEM *pListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McUnlockAllGroupClients);
	
	for (pListItem = QListHead(&pMcGroup->McClientList);
	     pListItem != NULL;
	     pListItem = QListNext(&pMcGroup->McClientList, pListItem))
	{
		MC_CLIENT *pMcClient = (MC_CLIENT *)QListObj(pListItem);
		
		ASSERT(pMcClient != NULL);
		
		pMcClient->McClientInUse--;
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

/* Assumes MulticastLock is Held */
static
void
QueueSubnetDriverCall(MC_GROUP *pMcGroup, MC_GROUP_STATE State)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, QueueSubnetDriverCall);
	
	if (pMcGroup->TimerActive == TRUE)
	{
		TimerStop(&pMcGroup->Timer);
	}
	else
	{
		pMcGroup->McGroupInUse++;
		pMcGroup->TimerActive = TRUE;
	}
	
	ASSERT((State == MC_GROUP_STATE_REQUEST_JOIN) || (State == MC_GROUP_STATE_REQUEST_LEAVE));
	pMcGroup->McGroupState = State;
	TimerStart(&pMcGroup->Timer, 1000);
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

/* Assumes MulticastLock is Held */
static
void
CallMcClients(MC_GROUP *pMcGroup, FSTATUS Status, MC_GROUP_STATE State)
{
	LIST_ITEM *pMcClientListItem;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, CallMcClient);
	
	if (pMcGroup->McGroupDelete == TRUE && pMcGroup->McGroupInUse == 0 )
	{
		goto exit;
	}
	
	McLockAllGroupClients(pMcGroup);
	
	//pMcGroup->McGroupInUse++;
	pMcGroup->McGroupState = State;
	
	for (pMcClientListItem = QListHead(&pMcGroup->McClientList);
	     pMcClientListItem != NULL;
         pMcClientListItem = QListNext(&pMcGroup->McClientList, pMcClientListItem))
	{               
		SD_MULTICAST_CALLBACK *pMulticastCallback;
		void                *pContext;
		MC_CLIENT           *pMcClient = (MC_CLIENT *)QListObj(pMcClientListItem);
		
		ASSERT(pMcClient != NULL);
		
		if (pMcClient->McClientDelete == TRUE)
		{
			continue;
		}
			
		if (State == MC_GROUP_STATE_UNAVAILABLE)
		{
			if (!(pMcClient->McFlags & MC_FLAG_WANT_UNAVAILABLE))
			{
				continue;
			}
		}
			
		pMulticastCallback = pMcClient->pMulticastCallback;
		pContext = pMcClient->pContext;
		SpinLockRelease(&MulticastLock);
		(*pMulticastCallback)(pContext, Status, State, &pMcGroup->McMemberRecord);
		SpinLockAcquire(&MulticastLock);
	}

	McUnlockAllGroupClients(pMcGroup);

exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

static
void
McGroupFabOpCallback(void                  *pContext,
                     FABRIC_OPERATION_DATA *pFabOp,
                     FSTATUS               Status,
                     uint32                MadStatus)
{
	MC_GROUP *pMcGroup = (MC_GROUP*)pContext;
	LIST_ITEM *pListItem;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McGroupFabOpCallback);
	
	if (McSdHandle != NULL)
	{
		ASSERT(pMcGroup != NULL);

		SpinLockAcquire(&MulticastLock);

		//pMcGroup->McGroupInUse--;

		if ((Status == FTIMEOUT) || (MadStatus == MAD_STATUS_BUSY))
		{
			/* Has Timed Out. Try Again */
			pMcGroup->McGroupInUse--;
			QueueSubnetDriverCall(pMcGroup, pMcGroup->McGroupState);
			goto exit;
		}

		switch (pMcGroup->McGroupState)
		{
			case MC_GROUP_STATE_REQUEST_JOIN:
			{
				MC_GROUP_STATE State = MC_GROUP_STATE_AVAILABLE;
		
				if ((Status != FSUCCESS) || (MadStatus != MAD_STATUS_SUCCESS))
				{
					_DBG_ERROR(("McGroupFabOpCallback Group Join Failure. Status = %d, Mad Status = %d\n",
					            Status, MadStatus));
					State = MC_GROUP_STATE_JOIN_FAILED;
				} else {
					/* update record with full details from SM */
					// why is this necessary??? could it be causing any harm???
					pMcGroup->McMemberRecord = pFabOp->Value.McMemberRecordValue.McMemberRecord;
				}
				
				CallMcClients(pMcGroup, Status, State);
				if (State == MC_GROUP_STATE_JOIN_FAILED)
				{
					// set up the group for delete
					pMcGroup->McGroupDelete = TRUE;
					// set all clients using this group for delete
					for (pListItem = QListHead(&pMcGroup->McClientList);
	     				pListItem != NULL;
	     				pListItem = QListNext(&pMcGroup->McClientList, pListItem))
					{
						MC_CLIENT *pMcClient = (MC_CLIENT *)QListObj(pListItem);
						pMcClient->McClientDelete = TRUE;
					}
				}	
				break;
			}

			case MC_GROUP_STATE_REQUEST_LEAVE:
			{
				// Removed group from the SM
				// pMcGroup will be deallocated by McMaintenance

				CallMcClients(pMcGroup, Status, MC_GROUP_STATE_UNAVAILABLE );
				break;
			}
			default:
				_DBG_ERROR(("McGroupFabOpCallback: Unknown MC Group Request %d, FabOp Type %d, Status %d, MadStatus %d\n",
					pMcGroup->McGroupState, pFabOp->Type, Status, MadStatus));
		}
exit:
		pMcGroup->McGroupInUse--;
		SpinLockRelease(&MulticastLock);
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

/* Assumes MulticastLock is Held */
static
void
IssueMcGroupFabOp(MC_GROUP *pMcGroup, FABRIC_OPERATION_TYPE FabOpType)
{
	FSTATUS Status;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, IssueMcGroupFabOp);
	
	if (pMcGroup->McGroupDelete == TRUE && pMcGroup->McGroupInUse == 0 )
	{
		goto exit;
	}
	
	//pMcGroup->McGroupInUse++;
	
	pMcGroup->FabOp.Type = FabOpType;

	pMcGroup->FabOp.Value.McMemberRecordValue.ComponentMask = pMcGroup->ComponentMask;
	pMcGroup->FabOp.Value.McMemberRecordValue.McMemberRecord = pMcGroup->McMemberRecord;

	SpinLockRelease(&MulticastLock);
	
	Status = iba_sd_port_fabric_operation(McSdHandle,
		                         pMcGroup->McGroupId.EgressPortGUID,
		                         &pMcGroup->FabOp,
		                         McGroupFabOpCallback,
		                         NULL,
		                         pMcGroup);
								 
	SpinLockAcquire(&MulticastLock);
	
	if ((Status != FPENDING) && (Status != FSUCCESS))
	{
		pMcGroup->McGroupInUse--;
		/* Port may not yet be active. We'll get it later when it does go active */
		/* _DBG_ERROR(("PortFabricOperation Status = %d\n", Status)); */
	}

exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

/* Assumes MulticastLock is Held */
static
MC_GROUP*
FindMcGroupNext(IB_GID *pMGID,
	MC_GROUP *pMcGroup)
{
	LIST_ITEM *pListItem;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FindMcGroupNext);
	
	_DBG_INFO(("looking for MGID:%"PRIx64" : %"PRIx64"\n", pMGID->AsReg64s.H, pMGID->AsReg64s.L));
	
	pListItem = (pMcGroup == NULL) ? QListHead(&MasterMcGroupList) : QListNext(&MasterMcGroupList, &pMcGroup->ListItem);
	
	while (pListItem != NULL)
	{
		pMcGroup = (MC_GROUP *)QListObj(pListItem);
		
		ASSERT(pMcGroup != NULL);

		_DBG_INFO(("MGID:%"PRIx64" : %"PRIx64" PortGuid:%"PRIx64" inuse:%d\n", 
			pMcGroup->McMemberRecord.RID.MGID.AsReg64s.H,
			pMcGroup->McMemberRecord.RID.MGID.AsReg64s.L,
		    pMcGroup->McMemberRecord.RID.PortGID.Type.Global.InterfaceID,
			pMcGroup->McGroupInUse));
		
		if (MemoryCompare(&pMcGroup->McGroupId.McRID.MGID, pMGID, sizeof(IB_GID)) == 0)
		{
			if (pMcGroup->McGroupDelete == FALSE)
			{
				_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
				return pMcGroup;
			}
		}
		
		pListItem = QListNext(&MasterMcGroupList, pListItem);
	}
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return NULL;
}

/* Assumes MulticastLock is Held */
static
MC_GROUP*
FindMcGroup(MC_GROUP_ID *pMcGroupId)
{
	LIST_ITEM *pListItem;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FindMcGroup);
	
	_DBG_INFO(("looking for MGID:%"PRIx64" : %"PRIx64" PortGuid:%"PRIx64"\n", 
			pMcGroupId->McRID.MGID.AsReg64s.H,
			pMcGroupId->McRID.MGID.AsReg64s.L,
			pMcGroupId->McRID.PortGID.Type.Global.InterfaceID));
	for (pListItem = QListHead(&MasterMcGroupList);
	     pListItem != NULL;
	     pListItem = QListNext(&MasterMcGroupList, pListItem))
	{
		MC_GROUP *pMcGroup = (MC_GROUP *)QListObj(pListItem);
		
		ASSERT(pMcGroup != NULL);

		_DBG_INFO(("MGID:%"PRIx64" : %"PRIx64" PortGuid:%"PRIx64" inuse:%d\n", 
			pMcGroup->McMemberRecord.RID.MGID.AsReg64s.H,
			pMcGroup->McMemberRecord.RID.MGID.AsReg64s.L,
		    pMcGroup->McMemberRecord.RID.PortGID.Type.Global.InterfaceID,
			pMcGroup->McGroupInUse));
		
		if (MemoryCompare(&pMcGroup->McMemberRecord.RID, &pMcGroupId->McRID, sizeof(MC_RID)) == 0)
		{
			if (pMcGroup->McGroupDelete == FALSE)
			{
				_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
				return pMcGroup;
			}
			
			break;
		}
	}
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return NULL;
}

/* Assumes MulticastLock is Held */
static
void
McMakeGroupsAvailable(EUI64 PortGuid)
{
	LIST_ITEM *pListItem;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McMakeGroupsAvailable);
	
	for (pListItem = QListHead(&MasterMcGroupList);
	     pListItem != NULL;
         pListItem = QListNext(&MasterMcGroupList, pListItem))
	{
		MC_GROUP *pMcGroup = (MC_GROUP*)QListObj(pListItem);
		
		ASSERT(pMcGroup != NULL);
			
		if (pMcGroup->McGroupDelete == FALSE)
		{
			if (PortGuid == pMcGroup->McGroupId.EgressPortGUID)
			{
				QueueSubnetDriverCall(pMcGroup, MC_GROUP_STATE_REQUEST_JOIN);
			}
		}
	}
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

/* Assumes MulticastLock is Held */
static
void
McMakeGroupsUnavailable(EUI64 PortGuid)
{
	LIST_ITEM  *pListItem;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McMakeGroupsUnavailable);
	
	McLockAllGroups();
	
	for (pListItem = QListHead(&MasterMcGroupList);
	     pListItem != NULL;
         pListItem = QListNext(&MasterMcGroupList, pListItem))
	{
		MC_GROUP *pMcGroup = (MC_GROUP*)QListObj(pListItem);
		
		ASSERT(pMcGroup != NULL);
			
		if (pMcGroup->McGroupDelete == FALSE)
		{
			if (PortGuid == pMcGroup->McGroupId.EgressPortGUID)
			{
				CallMcClients(pMcGroup, FUNAVAILABLE, MC_GROUP_STATE_UNAVAILABLE);
			}
		}
	}
	
	McUnlockAllGroups();
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

void
McPortNotifyCallback(IB_NOTIFY_RECORD *pNotifyRecord)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, PortNotifyCallback);
	
	if (McSdHandle != NULL)
	{
		SpinLockAcquire(&MulticastLock);

		switch(pNotifyRecord->EventType)
		{
			case IB_NOTIFY_LID_EVENT:
			case IB_NOTIFY_SM_EVENT:
			case IB_NOTIFY_PKEY_EVENT:
				_DBG_INFO(("McPortNotifyCallback EventType:%d\n", 
					(int)pNotifyRecord->EventType));
				McMakeGroupsAvailable(pNotifyRecord->Guid);
				break;

			case IB_NOTIFY_PORT_DOWN:
				_DBG_INFO(("McPortNotifyCallback EventType PortDown\n"));
				McMakeGroupsUnavailable(pNotifyRecord->Guid);
				break;

			default:
				break;
		}

		SpinLockRelease(&MulticastLock);
	}
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

static
void
McNoticeReportCallback(
	void   *pContext, 
	IB_NOTICE *pNotice, 
	EUI64  PortGuid
	)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McNoticeReportCallback);
	
	if (McSdHandle != NULL)
	{
		SpinLockAcquire(&MulticastLock);

		switch(pNotice->u.Generic.TrapNumber)
		{
			case SMA_TRAP_ADD_MULTICAST_GROUP:
			{
				TRAP_66_DETAILS *pTrap66Details = (TRAP_66_DETAILS *)pNotice->Data;
				MC_GROUP *pMcGroup = NULL;
				while ((pMcGroup = FindMcGroupNext(&pTrap66Details->GIDAddress, pMcGroup)) != NULL)
				{
					QueueSubnetDriverCall(pMcGroup, MC_GROUP_STATE_REQUEST_JOIN);
				}
				break;
			}

			case SMA_TRAP_DEL_MULTICAST_GROUP:
			{
				TRAP_67_DETAILS *pTrap67Details = (TRAP_67_DETAILS *)pNotice->Data;
				MC_GROUP *pMcGroup = NULL;
				while ((pMcGroup = FindMcGroupNext(&pTrap67Details->GIDAddress, pMcGroup)) != NULL)
				{
					_DBG_ERROR(("McNoticeReportCallback Mc Group (0x%016"PRIx64"%016"PRIx64") "
					            "Deleted By SM But We Still Have Connections.\n",
					            pTrap67Details->GIDAddress.AsReg64s.H,
					            pTrap67Details->GIDAddress.AsReg64s.L));
				}
				break;
			}

			default:
				/* shouldn't ever happend */
				_DBG_ERROR(("McNoticeReportCallback: Unknown Notification Trap Number %d",
					pNotice->u.Generic.TrapNumber));
		}

		SpinLockRelease(&MulticastLock);
	}
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

static
void
IssueSubnetDriverCall(
	void *pContext
	)
{
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, IssueSubnetDriverCall);
	
	if (McSdHandle != NULL)
	{
		MC_GROUP              *pMcGroup = (MC_GROUP*)pContext;
		FABRIC_OPERATION_TYPE FabOpType;
		
		ASSERT(pMcGroup != NULL);

		SpinLockAcquire(&MulticastLock);
		if (TimerCallbackStopped(&pMcGroup->Timer))
			goto unlock; // stale timer callback

		pMcGroup->TimerActive = FALSE;

		//pMcGroup->McGroupInUse--;
				
		switch(pMcGroup->McGroupState)
		{
			case MC_GROUP_STATE_REQUEST_JOIN:
				_DBG_INFO(("issuing FabOpSetmember MGID:%"PRIx64" : %"PRIx64" PortGuid:%"PRIx64" inuse:%d\n", 
			pMcGroup->McMemberRecord.RID.MGID.AsReg64s.H,
			pMcGroup->McMemberRecord.RID.MGID.AsReg64s.L,
		    pMcGroup->McMemberRecord.RID.PortGID.Type.Global.InterfaceID,
			pMcGroup->McGroupInUse));
				FabOpType = FabOpSetMcMemberRecord;
				break;

			case MC_GROUP_STATE_REQUEST_LEAVE:
				_DBG_INFO(("issuing FabOpDeleteMcMember MGID:%"PRIx64" : %"PRIx64" PortGuid:%"PRIx64" inuse:%d\n", 
			pMcGroup->McMemberRecord.RID.MGID.AsReg64s.H,
			pMcGroup->McMemberRecord.RID.MGID.AsReg64s.L,
		    pMcGroup->McMemberRecord.RID.PortGID.Type.Global.InterfaceID,
			pMcGroup->McGroupInUse));
				FabOpType = FabOpDeleteMcMemberRecord;
				break;

			case MC_GROUP_STATE_JOIN_FAILED:
			case MC_GROUP_STATE_AVAILABLE:
			case MC_GROUP_STATE_UNAVAILABLE:
			default:
				// Gets rid of compiler warning
				_DBG_INFO(("issuing FabOpDeleteMcMember(2) MGID:%"PRIx64" : %"PRIx64" PortGuid:%"PRIx64" inuse:%d\n", 
			pMcGroup->McMemberRecord.RID.MGID.AsReg64s.H,
			pMcGroup->McMemberRecord.RID.MGID.AsReg64s.L,
		    pMcGroup->McMemberRecord.RID.PortGID.Type.Global.InterfaceID,
			pMcGroup->McGroupInUse));
				FabOpType = FabOpDeleteMcMemberRecord;
				break;
		}
		
		IssueMcGroupFabOp(pMcGroup, FabOpType);

unlock:
		SpinLockRelease(&MulticastLock);
	}
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

/* Assumes MulticastLock is Held */
static
MC_CLIENT*
FindMcClient(
	CLIENT_HANDLE SdClientHandle,
	MC_GROUP_ID *pMcGroupId
	)
{
	LIST_ITEM *pListItem;
	size_t compareSize;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, FindMcClient);

	compareSize = (pMcGroupId->JoinVersion == 1) ? sizeof(int) + sizeof(IB_GID) : sizeof(MC_GROUP_ID);

	for (pListItem = QListHead(&MasterMcClientList);
		pListItem != NULL;
		pListItem = QListNext(&MasterMcClientList, pListItem))
	{
		MC_CLIENT *pMcClient = (MC_CLIENT *)QListObj(pListItem);
		
		if ((pMcClient->SdClientHandle == SdClientHandle) &&
			(pMcClient->pMcGroup != NULL) &&
			(MemoryCompare(&pMcGroupId, &pMcClient->pMcGroup->McGroupId, compareSize) == 0))
		{
			return pMcClient;
		}
	}
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return NULL;
}

/* Assumes MulticastLock is Held */
static __inline
boolean
McJoinFlagsAdjusted(
	MC_GROUP *pMcGroup,
	IB_MCMEMBER_RECORD *pMcMemberRecord
	)
{
	boolean FlagsAdjusted = FALSE;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McJoinFlagsAdjusted);
		
	//Adjust the join/leave flags
	if (!pMcGroup->McMemberRecord.JoinFullMember)
	{
		if (pMcMemberRecord->JoinFullMember)
		{
			pMcGroup->McMemberRecord.JoinFullMember = 1;
			FlagsAdjusted = TRUE;
		}
	}
		
	if (!pMcGroup->McMemberRecord.JoinNonMember)
	{
		if (pMcMemberRecord->JoinNonMember)
		{
			pMcGroup->McMemberRecord.JoinNonMember = 1;
			FlagsAdjusted = TRUE;
		}
	}
		
	if (!pMcGroup->McMemberRecord.JoinSendOnlyMember)
	{
		if (pMcMemberRecord->JoinSendOnlyMember)
		{
			pMcGroup->McMemberRecord.JoinSendOnlyMember = 1;
			FlagsAdjusted = TRUE;
		}
	}
		
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return FlagsAdjusted;
}


static
void
McSubscribeForTraps(void)
{
	uint32 PortIndex;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McSubscribeForTraps);
	
	for (PortIndex = 0; PortIndex < NumTotalPorts; PortIndex++) 
	{
		FSTATUS Status;
		
		Status = iba_sd_trap_notice_subscribe(McSdHandle,
		                             SMA_TRAP_ADD_MULTICAST_GROUP,
		                             pLocalCaGuidList[PortIndex],
		                             NULL,
		                             McNoticeReportCallback);
		if (Status != FSUCCESS)
		{
			_DBG_ERROR(("Cannot Subscribe For Trap "
			            "SMA_TRAP_ADD_MULTICAST_GROUP.\n"));
			break;
		}

		Status = iba_sd_trap_notice_subscribe(McSdHandle,
		                             SMA_TRAP_DEL_MULTICAST_GROUP,
		                             pLocalCaGuidList[PortIndex],
		                             NULL,
		                             McNoticeReportCallback);
		if (Status != FSUCCESS)
		{
			_DBG_ERROR(("Cannot Subscribe For Trap "
			            "SMA_TRAP_DEL_MULTICAST_GROUP.\n"));
			break;
		}
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

static
void
McUnsubscribeForTraps(void)
{
	uint32 PortIndex;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McUnSubscribeForTraps);
	
	for (PortIndex = 0; PortIndex < NumTotalPorts; PortIndex++) 
	{
		FSTATUS Status;
		
		Status = iba_sd_trap_notice_unsubscribe(McSdHandle,
		                               SMA_TRAP_ADD_MULTICAST_GROUP,
		                               pLocalCaGuidList[PortIndex]);
		if (Status != FSUCCESS)
		{
			_DBG_ERROR(("Cannot Unsubscribe For Trap "
			            "SMA_TRAP_ADD_MULTICAST_GROUP.\n"));
		}

		Status = iba_sd_trap_notice_unsubscribe(McSdHandle,
		                               SMA_TRAP_DEL_MULTICAST_GROUP,
		                               pLocalCaGuidList[PortIndex]);
		if (Status != FSUCCESS)
		{
			_DBG_ERROR(("Cannot Unsubscribe For Trap "
			            "SMA_TRAP_DEL_MULTICAST_GROUP.\n"));
		}
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

static
FSTATUS
IncrementMcGroupUsageCount(CLIENT_HANDLE       SdClientHandle,
                           void                *pContext,
                           SD_MULTICAST_CALLBACK *pMulticastCallback,
                           uint16              McFlags,
                           uint64              ComponentMask,
                           IB_MCMEMBER_RECORD  *pMcMemberRecord,
                           MC_GROUP_ID         *pMcGroupId)
{
	MC_CLIENT *pMcClientTemp;
	MC_CLIENT *pMcClient = NULL;
	MC_GROUP  *pMcGroupTemp;
	MC_GROUP  *pMcGroup = NULL;
	boolean   StartTrapNotification;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, IncrementMcGroupUsageCount);
	
	/* Allocate Everything up front to avoid locking issues */
	pMcClientTemp = (MC_CLIENT *)MemoryAllocate2AndClear(sizeof(MC_CLIENT), IBA_MEM_FLAG_NONE, SUBNET_DRIVER_TAG);
	if (pMcClientTemp == NULL)
	{
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return FINSUFFICIENT_MEMORY;
	}
	
	pMcClientTemp->SdClientHandle = SdClientHandle;
	pMcClientTemp->pContext = pContext;
	pMcClientTemp->McFlags = McFlags;
	pMcClientTemp->pMulticastCallback = pMulticastCallback;

	pMcGroupTemp = (MC_GROUP*)MemoryAllocate2AndClear(sizeof(MC_GROUP), IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
	if (pMcGroupTemp == NULL)
	{
		_DBG_ERROR(("IncrementMcGroupUsageCount MemoryAllocateError\n"));
		MemoryDeallocate(pMcClientTemp);
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return FINSUFFICIENT_MEMORY;
	}
	
	SpinLockAcquire(&MulticastLock);

	pMcClient = FindMcClient(SdClientHandle, pMcGroupId);
	if (pMcClient == NULL) 
	{
		pMcClient= pMcClientTemp;
		pMcClientTemp = NULL;
	}
	else 
	{
		_DBG_ERROR(("IncrementMcGroupUsageCount Client already exists.\n"));
		/*SpinLockRelease(&MulticastLock);
		MemoryDeallocate(pMcGroupTemp);
		MemoryDeallocate(pMcClient);
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return FDUPLICATE;*/
		// continue anyway
	}
	
	StartTrapNotification = QListIsEmpty(&MasterMcGroupList);
	
	pMcGroup = FindMcGroup(pMcGroupId);
	if (pMcGroup == NULL)
	{
		// didn't find Mcgroup in list, let us add one
		pMcGroupTemp->McGroupState = MC_GROUP_STATE_REQUEST_JOIN;
		pMcGroupTemp->ComponentMask = ComponentMask;
		pMcGroupTemp->McMemberRecord = *pMcMemberRecord;
		pMcGroupTemp->McGroupId = *pMcGroupId;
		QListInit(&pMcGroupTemp->McClientList);
		QListSetObj(&pMcGroupTemp->ListItem, pMcGroupTemp);

		pMcGroup = pMcGroupTemp;
		pMcGroupTemp = NULL;

		QListInsertTail(&MasterMcGroupList, &pMcGroup->ListItem);

		TimerInitState(&pMcGroup->Timer);
		TimerInit(&pMcGroup->Timer, IssueSubnetDriverCall, pMcGroup );
	}
	else
	{
		// group already exists - check state
		if ( (pMcGroup->McGroupState != MC_GROUP_STATE_REQUEST_JOIN) &&
		     (pMcGroup->McGroupState != MC_GROUP_STATE_AVAILABLE) )
		{
			if (pMcGroup->McGroupState == MC_GROUP_STATE_JOIN_FAILED)
			{
				_DBG_ERROR(("Prior attempt to join Multicast Group failed\n"));
			}
			else
			{
				_DBG_ERROR(("IncrementMcGroupUsageCount McGroup invalid state:%d.\n",
				pMcGroup->McGroupState));
			}
			SpinLockRelease(&MulticastLock);
			MemoryDeallocate(pMcGroupTemp);
			if(pMcClientTemp)
				MemoryDeallocate(pMcClientTemp);
			_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
			return FDUPLICATE;
		}
	}

	pMcClient->pMcGroup = pMcGroup;
	
	QListSetObj(&pMcClient->ListItem, pMcClient);
	QListInsertTail(&MasterMcClientList, &pMcClient->ListItem);
	
	QListSetObj(&pMcClient->McGroupListItem, pMcClient);
	QListInsertTail(&pMcGroup->McClientList, &pMcClient->McGroupListItem);
	
	if (pMcGroupTemp == NULL)
	{
		/* First client joining the group - Get The Process Started */
		_DBG_INFO(("new mcgroup - queuing join\n"));
		QueueSubnetDriverCall(pMcGroup, MC_GROUP_STATE_REQUEST_JOIN);
	}
	else
	{
		_DBG_INFO(("existing mcgroup - checking flags %d\n",
			pMcGroup->McGroupState));
		if ((McJoinFlagsAdjusted(pMcGroup, pMcMemberRecord) == TRUE) && 
		    (pMcGroup->McGroupState == MC_GROUP_STATE_AVAILABLE)) // TODO: Check Concurrency Here
			/* we had already joined and the flags changes - so rejoin */
		{
			_DBG_INFO(("flags changed after we joined - rejoining \n"));
			QueueSubnetDriverCall(pMcGroup, MC_GROUP_STATE_REQUEST_JOIN);
		}
		else if (pMcGroup->McGroupState == MC_GROUP_STATE_AVAILABLE) 
		{
			// we already had joined but we got a new client - issue callbacks
			_DBG_INFO(("someone joined when state is available - issuing callbacks \n"));
			CallMcClients(pMcGroup, FSUCCESS, MC_GROUP_STATE_AVAILABLE);
		}
		// NOTE if state is JOIN, then when that completes all clients
		// including new one will be notified via callback
	}

	if (MaintenanceTimerActivated == FALSE)
	{
		MaintenanceTimerActivated = TRUE;
		TimerStart(&MaintenanceTimer, 5000);
	}
	
	SpinLockRelease(&MulticastLock);
	
	if (StartTrapNotification)
	{
		McSubscribeForTraps();
	}

	if (pMcGroupTemp)
	{
		TimerDestroy(&pMcGroupTemp->Timer);
		QListDestroy(&pMcGroupTemp->McClientList);
		MemoryDeallocate(pMcGroupTemp);
	}

	if (pMcClientTemp)
	{
		MemoryDeallocate(pMcClientTemp);
	}

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );

	return FSUCCESS;
}

static
FSTATUS
DecrementMcGroupUsageCount(CLIENT_HANDLE SdClientHandle, MC_GROUP_ID *pMcGroupId)
{
	FSTATUS   Status = FSUCCESS;
	MC_CLIENT *pMcClient;
	MC_GROUP  *pMcGroup;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, DecrementMcGroupUsageCount);

	SpinLockAcquire(&MulticastLock);
	
	pMcClient = FindMcClient(SdClientHandle, pMcGroupId);
	if (pMcClient == NULL)
	{
		Status = FNOT_FOUND;
		SpinLockRelease(&MulticastLock);
		_DBG_ERROR(("DecrementMcGroupUsageCount Client Not Found.\n"));
		goto exit;
	}
	
	pMcGroup = pMcClient->pMcGroup;
	pMcClient->pMcGroup = NULL;
	pMcClient->McClientDelete = TRUE;
	
	QListRemoveItem(&pMcGroup->McClientList, &pMcClient->McGroupListItem);
	if (QListIsEmpty(&pMcGroup->McClientList))
	{
		/* schedule the group removal in Maintenance thread	*/
		pMcGroup->McGroupDelete = TRUE;

		if (pMcGroup->McGroupState == MC_GROUP_STATE_AVAILABLE)
		{
			QueueSubnetDriverCall(pMcGroup, MC_GROUP_STATE_REQUEST_LEAVE);
		}
	}

	SpinLockRelease(&MulticastLock);
	
exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );

	return Status;
}

static
void
McMaintenance(void *pContext)
{
	LIST_ITEM *pListItem;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McMaintenance);
	
	if (McSdHandle != NULL)
	{
		SpinLockAcquire(&MulticastLock);

		if (TimerCallbackStopped(&MaintenanceTimer))
		{
			// stale timer callback
			SpinLockRelease(&MulticastLock);
			goto done;
		}

		// Loop Through MasterMcClientList

		pListItem = QListHead(&MasterMcClientList);

		while(pListItem != NULL)
		{
			MC_CLIENT *pMcClient = (MC_CLIENT *)QListObj(pListItem);

			ASSERT(pMcClient != NULL);

			pListItem = QListNext(&MasterMcClientList, pListItem);

			if ((pMcClient->McClientDelete == TRUE) && 
				(pMcClient->McClientInUse == 0))
			{
				QListRemoveItem(&MasterMcClientList, &pMcClient->ListItem);
				SpinLockRelease(&MulticastLock);
				MemoryDeallocate(pMcClient);
				if (McSdHandle == NULL)
				{
					break;
				}
				SpinLockAcquire(&MulticastLock);
			}
		}
	}

	if (McSdHandle != NULL)
	{
		// Loop Through MasterMcGroupList

		pListItem = QListHead(&MasterMcGroupList);

		while(pListItem != NULL)
		{
			MC_GROUP *pMcGroup = (MC_GROUP *)QListObj(pListItem);

			ASSERT(pMcGroup != NULL);

			pListItem = QListNext(&MasterMcGroupList, pListItem);

			if ((pMcGroup->McGroupDelete == TRUE) && 
				(pMcGroup->McGroupInUse == 0))
			{
				TimerDestroy(&pMcGroup->Timer);
				QListDestroy(&pMcGroup->McClientList);
				QListRemoveItem(&MasterMcGroupList, &pMcGroup->ListItem);
				SpinLockRelease(&MulticastLock);
				MemoryDeallocate(pMcGroup);
				if (McSdHandle == NULL)
				{
					break;
				}
				SpinLockAcquire(&MulticastLock);
			}
		}
	}

	if (McSdHandle != NULL)
	{
		if (!QListIsEmpty(&MasterMcGroupList) ||
			!QListIsEmpty(&MasterMcClientList))
		{
			/* Only Start It If There Is A Possibility Of Something To Do */
			TimerStart(&MaintenanceTimer, 5000);
			SpinLockRelease(&MulticastLock);
		}
		else
		{
			McUnsubscribeForTraps();
			MaintenanceTimerActivated = FALSE;
			SpinLockRelease(&MulticastLock);
		}
	}
	
done:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
}

FSTATUS
MulticastInitialize(void)
{
	FSTATUS Status;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, InitializeMulticast);

	McSdHandle = NULL;

	QListInit(&MasterMcGroupList);
	QListInit(&MasterMcClientList);
	SpinLockInitState(&MulticastLock);
	SpinLockInit(&MulticastLock);
	
	TimerInitState(&MaintenanceTimer);
	TimerInit(&MaintenanceTimer, McMaintenance, NULL);
	
	MaintenanceTimerActivated = FALSE;
	
	Status = iba_sd_register(&McSdHandle, NULL);
	if (Status != FSUCCESS)
	{
		McSdHandle = NULL;
		_DBG_ERROR(("Multicast Module Not Able To Register With Subnet Driver "
		            "Status = %d.\n", Status));
	}
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return Status;
}

FSTATUS
MulticastTerminate(void)
{
	FSTATUS       Status;
	CLIENT_HANDLE McSdHandleTemp;
	LIST_ITEM     *pListItem;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, MulticastTerminate);
	
	/* Just in case maintenance routine is running */
	SpinLockAcquire(&MulticastLock);
	McSdHandleTemp = McSdHandle;
	McSdHandle = NULL;
	SpinLockRelease(&MulticastLock);
		
	if (McSdHandleTemp == NULL)
	{
		_DBG_ERROR(("Multicast Module Not A Registered Subnet Driver "
		            "Client Process.\n"));
		Status = FINVALID_STATE;
		goto exit;
	}

	if (MaintenanceTimerActivated == TRUE)
	{
		TimerStop(&MaintenanceTimer);
		MaintenanceTimerActivated = FALSE;
	}
	
	Status = iba_sd_deregister(McSdHandleTemp);
	if (Status != FSUCCESS) {
		_DBG_ERROR(("Cannot Deregister With "
		            "Subnet Data Interface.\n"));
	}

	for (pListItem = QListRemoveHead(&MasterMcGroupList);
	     pListItem != NULL;
	     pListItem = QListRemoveHead(&MasterMcGroupList))
	{
		MC_GROUP *pMcGroup = (MC_GROUP *)QListObj(pListItem);
		
		ASSERT(pMcGroup != NULL);
		
		TimerStop(&pMcGroup->Timer);
		TimerDestroy(&pMcGroup->Timer);
		QListDestroy(&pMcGroup->McClientList);
		MemoryDeallocate(pMcGroup);
	}
	
	for (pListItem = QListRemoveHead(&MasterMcClientList);
	     pListItem != NULL;
	     pListItem = QListRemoveHead(&MasterMcClientList))
	{
		MC_CLIENT *pMcClient = (MC_CLIENT *)QListObj(pListItem);
		
		ASSERT(pMcClient != NULL);
		
		MemoryDeallocate(pMcClient);
	}
	
	QListDestroy(&MasterMcGroupList);
	QListDestroy(&MasterMcClientList);
	TimerDestroy(&MaintenanceTimer);
	SpinLockDestroy(&MulticastLock);

exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return Status;
}

static
FSTATUS
McJoinGroup(
	CLIENT_HANDLE       SdClientHandle,
	uint16              McFlags,
	uint64              ComponentMask, 
	IB_MCMEMBER_RECORD  *pMcMemberRecord,
	MC_GROUP_ID         *pMcGroupId,
	void                *pContext,
	SD_MULTICAST_CALLBACK *pMulticastCallback
	)
{
	FSTATUS                   Status;
	CLIENT_CONTROL_PARAMETERS ClientParams;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McJoinGroup);
	
	if (McSdHandle == NULL)
	{
		_DBG_ERROR(("Multicast Module Not A Registered Subnet Driver "
		            "Client Process.\n"));
		Status = FINVALID_STATE;
		goto exit;
	}
	
	Status = ValidateClientHandle(SdClientHandle, &ClientParams);
	if (Status != FSUCCESS)
	{
		_DBG_ERROR(("Invalid Client Handle Specified\n"));
		goto exit;
	}
	
	Status = IncrementMcGroupUsageCount(SdClientHandle,
	                                    pContext,
	                                    pMulticastCallback,
	                                    McFlags,
	                                    ComponentMask,
	                                    pMcMemberRecord,
	                                    pMcGroupId);

exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return Status;
}

FSTATUS
iba_sd_join_mcgroup2(
	CLIENT_HANDLE       SdClientHandle,
	uint16              McFlags,
	uint64              ComponentMask, 
	IB_MCMEMBER_RECORD  *pMcMemberRecord,
	EUI64               EgressPortGUID,
	void                *pContext,
	SD_MULTICAST_CALLBACK *pMulticastCallback
	)
{
	FSTATUS Status;
	MC_GROUP_ID McGroupId;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_join_mcgroup2);

	McGroupId.JoinVersion = 2;
	McGroupId.McRID.MGID = pMcMemberRecord->RID.MGID;
	McGroupId.McRID.PortGID = pMcMemberRecord->RID.PortGID;
	McGroupId.EgressPortGUID = EgressPortGUID;

	Status = McJoinGroup(SdClientHandle,
						 McFlags,
						 ComponentMask,
						 pMcMemberRecord,
						 &McGroupId,
						 pContext,
						 pMulticastCallback);
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return Status;
}

FSTATUS
iba_sd_join_mcgroup(
	CLIENT_HANDLE       SdClientHandle,
	uint16              McFlags,
	uint64              ComponentMask, 
	IB_MCMEMBER_RECORD  *pMcMemberRecord,
	void                *pContext,
	SD_MULTICAST_CALLBACK *pMulticastCallback
	)
{
	FSTATUS Status;
	MC_GROUP_ID McGroupId;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_join_mcgroup);

	McGroupId.JoinVersion = 1;
	McGroupId.McRID.MGID = pMcMemberRecord->RID.MGID;
	McGroupId.McRID.PortGID = pMcMemberRecord->RID.PortGID;
	McGroupId.EgressPortGUID = pMcMemberRecord->RID.PortGID.Type.Global.InterfaceID;

	Status = McJoinGroup(SdClientHandle,
						 McFlags,
						 ComponentMask,
						 pMcMemberRecord,
						 &McGroupId,
						 pContext,
						 pMulticastCallback);
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return Status;
}

static
FSTATUS
McLeaveGroup(
	CLIENT_HANDLE SdClientHandle,
	MC_GROUP_ID *pMcGroupId
	)
{
	FSTATUS                   Status;
	CLIENT_CONTROL_PARAMETERS ClientParams;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, McLeaveGroup);
	
	if (McSdHandle == NULL)
	{
		_DBG_ERROR(("Multicast Module Not A Registered Subnet Driver "
		            "Client Process.\n"));
		Status = FINVALID_STATE;
		goto exit;
	}

	Status = ValidateClientHandle(SdClientHandle, &ClientParams);
	if (Status != FSUCCESS)
	{
		_DBG_ERROR(("Invalid Client Handle Specified\n"));
		goto exit;
	}
	
	Status = DecrementMcGroupUsageCount(SdClientHandle, pMcGroupId);
	if (Status == FPENDING)
	{
		Status = FSUCCESS;
	}
	
exit:
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return Status;
}

FSTATUS
iba_sd_leave_mcgroup2(
	CLIENT_HANDLE SdClientHandle,
	IB_GID *pMGID,
	IB_GID *pPortGID,
	EUI64 EgressPortGUID
	)
{
	FSTATUS Status;
	MC_GROUP_ID McGroupId;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_leave_mcgroup2);

	McGroupId.JoinVersion = 2;
	McGroupId.McRID.MGID = *pMGID;
	McGroupId.McRID.PortGID = *pPortGID;
	McGroupId.EgressPortGUID = EgressPortGUID;
	
	Status = McLeaveGroup(SdClientHandle, &McGroupId);
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return Status;
}

FSTATUS
iba_sd_leave_mcgroup(
	CLIENT_HANDLE SdClientHandle,
	IB_GID *pMGID
	)
{
	FSTATUS Status;
	MC_GROUP_ID McGroupId;
	
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, iba_sd_leave_mcgroup);
	
	MemoryClear(&McGroupId, sizeof(McGroupId));
	McGroupId.JoinVersion = 1;
	McGroupId.McRID.MGID = *pMGID;
	
	Status = McLeaveGroup(SdClientHandle, &McGroupId);
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	
	return Status;
}
