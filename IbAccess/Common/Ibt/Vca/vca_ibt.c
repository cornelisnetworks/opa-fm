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


#include "precomp.h"


//
// Forward declarations
//

static uint32
vcaCountCAList(void);


//
//
//

FSTATUS
iba_get_caguids(
	IN OUT	uint32		*CaCount,
	OUT	EUI64			*CaGuidList
	)
{
	DEVICE_INFO	*devInfo;
	EUI64		*guidList = NULL, *pGuid;
	uint32		caCount, count;
	FSTATUS		status = FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_PNP, iba_get_caguids);

	// CaCount may point to paged pool, so make a copy of it before
	// raising IRQL.
	caCount = *CaCount;

	SpinLockAcquire(&VcaGlobal.Lock);

	count = vcaCountCAList();

	if ( caCount < count )
	{
		// Return status indicating array is too small to hold list
		status = FINSUFFICIENT_MEMORY;
	} else {
		// Array is sufficient to hold list
		if ( CaGuidList == NULL )
		{
			_DBG_ERROR(("null list ptr\n"));
			status = FINVALID_PARAMETER;
		} else if (count > 0) {
			guidList = (EUI64*)MemoryAllocate2(count * sizeof(EUI64), IBA_MEM_FLAG_SHORT_DURATION, VCA_MEM_TAG);

			if ( guidList == NULL )
			{
				status = FINSUFFICIENT_RESOURCES;
			} else {
				devInfo = VcaGlobal.DeviceList;
				pGuid = guidList;

				while (devInfo != NULL)
				{
					*pGuid = devInfo->VpInterface.CaGUID;
					pGuid++;
					devInfo = devInfo->Next;
				}
			}
		}
	}

	SpinLockRelease(&VcaGlobal.Lock);

	// Always update count
	*CaCount = count;

	if ( status == FSUCCESS )
	{
		if (count > 0)
		{
			MemoryCopy(CaGuidList, guidList, count * sizeof(EUI64));
			MemoryDeallocate(guidList);
		}
	}

	_DBG_LEAVE_LVL(_DBG_LVL_PNP);

	return status;
}


//
//
//

FSTATUS
iba_query_ca_by_guid(
	IN  EUI64				CaGuid,
	OUT IB_CA_ATTRIBUTES	*CaAttributes
	)
{
	DEVICE_INFO	*devInfo;
	FSTATUS		status = FSUCCESS;

	_DBG_ENTER_LVL(_DBG_LVL_PNP, iba_query_ca_by_guid);

	if ( CaAttributes == NULL )
	{
		_DBG_ERROR(("null attributes ptr\n"));
		status = FINVALID_PARAMETER;
	} else {
		devInfo = vcaFindCA(CaGuid);

		if ( devInfo == NULL )
		{
			status = FNOT_FOUND;
		} else {
			status = iba_query_ca( devInfo->VcaCaHandle, CaAttributes, NULL );
		}
	}

	_DBG_LEAVE_LVL(_DBG_LVL_PNP);

	return status;
}



//	Get attributes of port with given PortGuid
FSTATUS
iba_query_port_by_guid(
	IN  EUI64					PortGuid,
	OUT IB_PORT_ATTRIBUTES		*PortAttributes,
	IN OUT uint32				*ByteCount
	)
{
	DEVICE_INFO			*devInfo;
	IB_PORT_ATTRIBUTES	*portAttributes = NULL;
	uint32		portAttributesSize = 0;
	FSTATUS		status = FNOT_FOUND;

	_DBG_ENTER_LVL(_DBG_LVL_CALL, iba_query_port_by_guid);

	SpinLockAcquire(&VcaGlobal.Lock);
			
	for (devInfo = VcaGlobal.DeviceList; devInfo != NULL;
			devInfo = devInfo->Next)
	{
		for ( portAttributes = devInfo->CaAttributes.PortAttributesList;
				portAttributes != NULL; portAttributes = portAttributes->Next)
		{
			if ( portAttributes->GUID == PortGuid )
			{
				portAttributesSize = IbPortAttributesSize(portAttributes);
				
				if ( *ByteCount < portAttributesSize )
				{
					status = FINSUFFICIENT_MEMORY;
				} else {
					// get latest attributes
					status = iba_query_ca(devInfo->VcaCaHandle, &devInfo->CaAttributes, NULL);
				}
				*ByteCount = portAttributesSize;
				goto found;
			}
		}
	}
found:

	if ( status == FSUCCESS )
	{
		// Copy the port's attributes structure to the
		// user's buffer.
		if ( PortAttributes == NULL )
		{
			status = FINVALID_PARAMETER;
		} else {
			IbCopyPortAttributes(PortAttributes, portAttributes, portAttributesSize);
			PortAttributes->Next = NULL;
			//MsgOut("Size=%d, CA Size = %d\n", portAttributesSize, devInfo->CaAttributes.PortAttributesListSize);
			//MsgOut("GID[0]=0x%016"PRIx64":0x%016"PRIx64"\n", PortAttributes->GIDTable[0].Type.Global.SubnetPrefix, PortAttributes->GIDTable[0].Type.Global.InterfaceID);
			//MsgOut("PKey[0]=0x%x\n", PortAttributes->PkeyTable[0]);
		}
	}
	SpinLockRelease(&VcaGlobal.Lock);

	_DBG_LEAVE_LVL(_DBG_LVL_CALL);

	return status;
}

// locate CaGuid with given PortGuid
FSTATUS
iba_get_ca_for_port(
	IN  EUI64	PortGuid,
	OUT EUI64	*CaGuid
	)
{
	DEVICE_INFO			*devInfo;
	IB_PORT_ATTRIBUTES	*portAttributes = NULL;
	FSTATUS		status = FNOT_FOUND;

	_DBG_ENTER_LVL(_DBG_LVL_CALL, iba_get_ca_for_port);

	SpinLockAcquire(&VcaGlobal.Lock);

	for (devInfo = VcaGlobal.DeviceList; devInfo != NULL;
			devInfo = devInfo->Next)
	{
		for ( portAttributes = devInfo->CaAttributes.PortAttributesList;
				portAttributes != NULL;
				portAttributes = portAttributes->Next
				)
		{
			if ( portAttributes->GUID == PortGuid )
			{
				status = FSUCCESS;
				*CaGuid = devInfo->VpInterface.CaGUID;
				goto found;
			}
		}
	}
found:
	SpinLockRelease(&VcaGlobal.Lock);

	_DBG_LEAVE_LVL(_DBG_LVL_CALL);

	return status;
}

// update the CaAttributes for all CAs in VcaGlobal device list
// this will get us the latest port states
// To simplify processing in iba_select_ca, we set a high bit in the count
// to indicate if ports are inactive, in which case the given inactive
// port will be considered a high use port and we can test the bit at the end
static
FSTATUS 
UpdateDeviceAttributes(void)
{
	FSTATUS status = FSUCCESS;
	DEVICE_INFO	*devInfo;
	IB_PORT_ATTRIBUTES	*portAttributes;

	_DBG_ENTER_LVL(_DBG_LVL_CALL, UpdateDeviceAttributes);
	SpinLockAcquire(&VcaGlobal.Lock);
	for (devInfo = VcaGlobal.DeviceList; devInfo != NULL; devInfo = devInfo->Next)
	{
		status = iba_query_ca(devInfo->VcaCaHandle, &devInfo->CaAttributes, NULL);
		if ( status != FSUCCESS )
		{
			_DBG_ERROR(("failed to query CA for port attributes\n"));
			break;
		}
		/* assume inactive, clear flag if an active port is found */
		devInfo->CaSelectCount |= IBA_SELECT_INACTIVE;
		for (portAttributes = devInfo->CaAttributes.PortAttributesList;
			portAttributes != NULL;
			portAttributes = portAttributes->Next
			)
		{
			if (portAttributes->PortState == PortStateActive)
			{
				devInfo->CaSelectCount &= ~IBA_SELECT_INACTIVE;
			}
		}
	}
	SpinLockRelease(&VcaGlobal.Lock);
	_DBG_LEAVE_LVL(_DBG_LVL_CALL);
	return status;
}

/*
 * iba_select_ca
 *
 *	Track CA/ports being heavily used and aid in selection of least used one
 *	Usage models:
 *		pCaGuid = NULL   invalid call
 *		*pCaGuid !=0     caller has specified a CA, API increases its count
 *		*pCaGuid ==0     API will recommend a CA and increase its count
 *
 * INPUTS:
 *	*pCaGuid - CA caller plans to use
 *
 * OUTPUTS:
 *	*pCaGuid - CA recommended for use (matches supplied *pCaGuid if input != 0)
 *
 * RETURNS:
 *	FNOT_FOUND - no such CA
 *	FINVALID_PARAMETER - invalid combination of arguments
 *	FINSUFFICIENT_RESOURCES - no CAs in system with active ports
 *	FERROR - other errors (unable to query CA attributes, etc)
 */

FSTATUS
iba_select_ca(
	IN OUT	EUI64				*pCaGuid
	)
{
	FSTATUS status = FSUCCESS;
	DEVICE_INFO	*devInfo;

	_DBG_ENTER_LVL(_DBG_LVL_CALL, iba_select_ca);
	if (! pCaGuid)
	{
		status = FINVALID_PARAMETER;
		goto fail;
	}
	if (*pCaGuid == 0) {
		// will need latest port states to properly select a CA
		status = UpdateDeviceAttributes();
		if (FSUCCESS != status)
		{
			status = FERROR;
			goto fail;
		}
	}
	SpinLockAcquire(&VcaGlobal.Lock);
	if (*pCaGuid != 0) {
		// caller has specified a CA, find it
		for (devInfo = VcaGlobal.DeviceList;
			(devInfo != NULL) && (devInfo->VpInterface.CaGUID != *pCaGuid);
			devInfo = devInfo->Next)
				;
		if (NULL == devInfo) {
			status = FNOT_FOUND;
			goto unlock;
		}
		// devInfo is caller specified CA
		devInfo->CaSelectCount++;
		_DBG_INFO(("iba_select_ca for CA 0x%016"PRIx64": count=%u\n",
							devInfo->VpInterface.CaGUID, devInfo->CaSelectCount));
	} else {
		// caller wants us to select
		DEVICE_INFO	*minDevInfo = VcaGlobal.DeviceList;

		if (NULL == VcaGlobal.DeviceList)
		{
			status = FINSUFFICIENT_RESOURCES;
			goto unlock;
		}
		// select CA with lowest overall use count
		for (devInfo = VcaGlobal.DeviceList; devInfo != NULL;
			devInfo = devInfo->Next)
		{
			if (devInfo->CaSelectCount < minDevInfo->CaSelectCount)
				minDevInfo = devInfo;
		}
		devInfo = minDevInfo;	// found recommended CA
		if (devInfo->CaSelectCount & IBA_SELECT_INACTIVE)
		{
			status = FINSUFFICIENT_RESOURCES;
			goto unlock;
		}
		devInfo->CaSelectCount++;
		_DBG_INFO(("iba_select_ca picked CA 0x%016"PRIx64": count=%u\n",
						devInfo->VpInterface.CaGUID, devInfo->CaSelectCount));
		*pCaGuid = devInfo->VpInterface.CaGUID;
	}
unlock:
	SpinLockRelease(&VcaGlobal.Lock);
fail:
	_DBG_LEAVE_LVL(_DBG_LVL_CALL);
	return status;
}

/*
 * iba_deselect_ca
 *
 * undo effects of iba_select_ca
 * call should match the CaGUID from a previous iba_select_ca call
 *
 * INPUTS:
 *	CaGuid - CA output/selected via iba_select_ca
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	FNOT_FOUND - no such CA or Port, or PortGuid not within specified CA
 *	FINVALID_OPERATION - would result in negative use count
 */

FSTATUS
iba_deselect_ca(
	IN EUI64				CaGuid
	)
{
	FSTATUS status = FSUCCESS;
	DEVICE_INFO	*devInfo;

	_DBG_ENTER_LVL(_DBG_LVL_CALL, iba_deselect_ca);
	if (! CaGuid)
	{
		status = FINVALID_PARAMETER;
		goto fail;
	}
	SpinLockAcquire(&VcaGlobal.Lock);
	for (devInfo = VcaGlobal.DeviceList;
		(devInfo != NULL) && (devInfo->VpInterface.CaGUID != CaGuid);
		devInfo = devInfo->Next)
			;
	if (NULL == devInfo) {
		status = FNOT_FOUND;
		goto unlock;
	}
	// devInfo is caller specified CA
	if (! (devInfo->CaSelectCount & ~IBA_SELECT_INACTIVE))
	{
		status = FINVALID_OPERATION;
		goto unlock;
	}
	devInfo->CaSelectCount--;
	_DBG_INFO(("iba_deselect_ca CA 0x%016"PRIx64": count=%u\n",
						devInfo->VpInterface.CaGUID, devInfo->CaSelectCount));
unlock:
	SpinLockRelease(&VcaGlobal.Lock);
fail:
	_DBG_LEAVE_LVL(_DBG_LVL_CALL);
	return status;
}

//
// Support functions
//

// locate device with given Guid
DEVICE_INFO *
vcaFindCA(IN EUI64 CaGuid)
{
	DEVICE_INFO	*devInfo;

	_DBG_ENTER_LVL(_DBG_LVL_PNP, vcaFindCA);

	SpinLockAcquire(&VcaGlobal.Lock);

	for (devInfo = VcaGlobal.DeviceList;
		(devInfo != NULL) && (devInfo->VpInterface.CaGUID != CaGuid);
		devInfo = devInfo->Next)
			;

	SpinLockRelease(&VcaGlobal.Lock);

	_DBG_LEAVE_LVL(_DBG_LVL_PNP);

	return devInfo;
}


//	count number of HFIs, caller must hold VcaGlobal.Lock
static uint32
vcaCountCAList(void)
{
	DEVICE_INFO	*devInfo;
	uint32	count;

	_DBG_ENTER_LVL(_DBG_LVL_PNP, vcaCountCAList);

	count = 0;
	for (devInfo = VcaGlobal.DeviceList; devInfo != NULL;
			devInfo = devInfo->Next)
	{
		count++;
	}

	_DBG_LEAVE_LVL(_DBG_LVL_PNP);
	return count;
}
