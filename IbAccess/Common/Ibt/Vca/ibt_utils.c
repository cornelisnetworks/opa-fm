/* BEGIN_ICS_COPYRIGHT6 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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

** END_ICS_COPYRIGHT6   ****************************************/

// Common utility functions shared between user and kernel mode

#include "imemory.h"
#include "ib_debug.h"
#include "ib_debug_osd.h"
#include "ib_generalServices.h"

#include "precomp.h"
#define ALLOC_TAG VCA_MEM_TAG

/*++

Routine Description
	get the GUIDs of all the CAs on this host.
	Allocates memory for the required size
	caller must MemoryDeallocate(*CaGuidList)

Input:
	CaGuidList:	Points to array where CaGuids are to be returned
	CaCount:	this is where the count of CA s to be returned

Output:
	status of the operation

--*/
FSTATUS
iba_get_caguids_alloc(
	uint32	*CaCount,
	EUI64	**CaGuidList
	)
{
	FSTATUS		Status;

	_DBG_ENTER_FUNC(iba_get_caguids_alloc);

	if ( CaCount == NULL || CaGuidList == NULL )
	{
		_DBG_ERROR(("null list ptr\n"));
		Status = FINVALID_PARAMETER;
		goto done;
	}

	// First find out the size of memory needed
	*CaCount = 0;
	*CaGuidList = NULL;
	Status = iba_get_caguids(CaCount, NULL);
	if (Status != FINSUFFICIENT_MEMORY) {
		_DBG_INFO(("Unexpected error from GetCaGuids\n"));
		goto done;
	}

	if (*CaCount <= 0) {
		_DBG_INFO(("No CA reported by GetCaGuids\n"));
		Status = FSUCCESS;
		goto done;
	}

	*CaGuidList = (EUI64*)MemoryAllocate(*CaCount * sizeof(EUI64), FALSE, ALLOC_TAG);
	if (*CaGuidList == NULL) {
		_DBG_ERROR(("Cant allocate memory for getting GetCaGuids\n"));
		Status =FINSUFFICIENT_RESOURCES;
		goto done;
	}

	// Now make the query for all CA GUIDs 
	Status = iba_get_caguids(CaCount, *CaGuidList);
	if (Status != FSUCCESS) {
		MemoryDeallocate(*CaGuidList);
		_DBG_INFO(("FSTATUS 0x%x from GetCaGuids\n", Status));
		*CaGuidList = NULL;
		goto done;
	} 

done:
	_DBG_LEAVE_FUNC();
	return Status;
}


// Query a Ca by Guid and allocate the CaAttributes->PortAttributesList
// as needed to complete the full query of all ports on Ca
// caller must MemoryDeallocate CaAttributes->PortAttributesList
FSTATUS
iba_query_ca_by_guid_alloc(
	IN  EUI64				CaGuid,
	OUT IB_CA_ATTRIBUTES	*CaAttributes
	)
{
	FSTATUS		status;

	_DBG_ENTER_FUNC(iba_query_ca_by_guid_alloc);

	// determine size of port attributes list
	CaAttributes->PortAttributesListSize = 0;
	CaAttributes->PortAttributesList = NULL;
	status = iba_query_ca_by_guid(CaGuid, CaAttributes);

	if (status != FSUCCESS) 
	{
		goto done;
	}

	ASSERT(CaAttributes->PortAttributesListSize > 0);

	CaAttributes->PortAttributesList = (IB_PORT_ATTRIBUTES*)MemoryAllocate(
					CaAttributes->PortAttributesListSize, FALSE, ALLOC_TAG);
	if (CaAttributes->PortAttributesList == NULL)  
	{
		_DBG_ERROR(("Cannot allocate memory for port attribs, CA <0x%016"PRIx64">\n",
					CaGuid));
		CaAttributes->PortAttributesListSize = 0;
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}

	// Now get the actual port attributes
	status = iba_query_ca_by_guid(CaGuid, CaAttributes);
	if (status != FSUCCESS)  
	{
		MemoryDeallocate(CaAttributes->PortAttributesList);
		CaAttributes->PortAttributesListSize = 0;
		CaAttributes->PortAttributesList = NULL;
		goto done;
	}

done:
	_DBG_LEAVE_FUNC();
	return(status);
}


// Query a Port by Guid and allocate the PortAttributes
// as needed to complete the full query of all ports on Ca
// caller must MemoryDeallocate *PortAttributes
FSTATUS
iba_query_port_by_guid_alloc(
	IN  EUI64					PortGuid,
	OUT IB_PORT_ATTRIBUTES		**PortAttributes
	)
{
	FSTATUS status;
	uint32	PortAttrSize = 0;

	_DBG_ENTER_FUNC(iba_query_port_by_guid_alloc);

	*PortAttributes = NULL;
	status = iba_query_port_by_guid(PortGuid, NULL, &PortAttrSize);
	if (status != FINSUFFICIENT_MEMORY) // i.e. we should get FINSUFFICIENT_MEMORY
		goto done;

	*PortAttributes = (IB_PORT_ATTRIBUTES*)MemoryAllocateAndClear(PortAttrSize, FALSE, ALLOC_TAG);
	if (*PortAttributes == NULL)
	{
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}

	status = iba_query_port_by_guid(PortGuid, *PortAttributes, &PortAttrSize);
	if (status != FSUCCESS)
	{
		MemoryDeallocate(*PortAttributes);
		*PortAttributes = NULL;
		goto done;
	}

done:
	_DBG_LEAVE_FUNC();

	return status;
}

/* translate port number into a port Guid
 *
 * INPUTS:
 * 	caGuid - CA to check against
 * 	port - 1-n, port within CA
 * 			if 0, 1st active port
 *
 * OUTPUTS:
 * 	*pPortGuid - port guid for specified port
 * 	*pCaAttributes - attributes for CA,
 * 					caller must MemoryDeallocate pCaAttributes->PortAttributesList
 * 	*ppPortAtributes - attributes for port, caller must MemoryDeallocate
 * 	*pPortCount - number of ports in CA
 *
 * RETURNS:
 * 	FNOT_FOUND - *pPortCount still output
 */
FSTATUS iba_get_ca_portguid(
		IN EUI64 caGuid,
		IN uint32 port,
		OUT EUI64 *pPortGuid OPTIONAL,
		OUT IB_CA_ATTRIBUTES    *pCaAttributes OPTIONAL,
		OUT IB_PORT_ATTRIBUTES	**ppPortAttributes OPTIONAL,
		OUT uint32 *pPortCount OPTIONAL
	)
{
	IB_CA_ATTRIBUTES    caAttribs;
	IB_PORT_ATTRIBUTES *pPortAttribs;
	FSTATUS             status = FSUCCESS;
	unsigned            i;

	_DBG_ENTER_FUNC(iba_get_ca_portguid);

	_DBG_INFO(("iba_get_ca_portguid: Ca 0x%016"PRIx64"\n", caGuid));

	memset(&caAttribs, 0, sizeof(caAttribs));
	caAttribs.PortAttributesList = NULL;

	status = iba_query_ca_by_guid_alloc(caGuid, &caAttribs);
	if (status != FSUCCESS)
		goto done;

	if (pPortCount)
		*pPortCount = caAttribs.Ports;

	if (port > caAttribs.Ports)
	{
		// not on this CA
		status = FNOT_FOUND;
		goto done;
	}

	pPortAttribs = caAttribs.PortAttributesList;
	for (i = 1; i <= caAttribs.Ports; ++i)
	{
		if (i == port
			|| (port == 0 && pPortAttribs->PortState >= PortStateInit))
		{
			// found it
			if (pCaAttributes) {
				*pCaAttributes = caAttribs;
				caAttribs.PortAttributesList = NULL;	// skip dealloc
			}
			if (pPortGuid)
				*pPortGuid = pPortAttribs->GUID;
				
			if ( ppPortAttributes) {
				uint32 size = IbPortAttributesSize(pPortAttribs);
				*ppPortAttributes = (IB_PORT_ATTRIBUTES*)MemoryAllocate2AndClear(
						size, IBA_MEM_FLAG_PREMPTABLE, ALLOC_TAG);
				if (! *ppPortAttributes) {
					status = FINSUFFICIENT_MEMORY;
					goto done;
				}
				IbCopyPortAttributes(*ppPortAttributes, pPortAttribs, size);
				(*ppPortAttributes)->Next = NULL;
			}
			goto done;
		}
		pPortAttribs = pPortAttribs->Next;
	}
	status = FNOT_FOUND;	// could be port==0 case with no >=INIT ports

done:
	if (caAttribs.PortAttributesList)
		MemoryDeallocate(caAttribs.PortAttributesList);
	_DBG_LEAVE_FUNC();

	return status;
}

/* translate ca/port number into a port Guid
 *
 * INPUTS:
 * 	ca - system wide CA number 1-n, if 0 port is a system wide port #
 * 	port - 1-n, if ca is 0, system wide port number, otherwise port within CA
 * 			if 0, 1st active port
 *
 * OUTPUTS:
 * 	*pCaGuid - ca guid for specified port
 * 	*pPortGuid - port guid for specified port
 * 	*pCaAttributes - attributes for CA,
 * 					caller must MemoryDeallocate pCaAttributes->PortAttributesList
 * 	*ppPortAtributes - attributes for port, caller must MemoryDeallocate
 * 	*pCaCount - number of CA in system
 * 	*pPortCount - number of ports in system or CA (depends on ca input)
 *
 * RETURNS:
 * 	FNOT_FOUND - *pCaCount and *pPortCount still output
 * 				if ca == 0, *pPortCount = number of ports in system
 * 				if ca < *pCaCount, *pPortCount = number of ports in CA
 * 									otherwise *pPortCount will be 0
 *
 */
FSTATUS iba_get_portguid(
		IN uint32 ca,
		IN uint32 port,
		OUT EUI64 *pCaGuid OPTIONAL,
		OUT EUI64 *pPortGuid OPTIONAL,
		OUT IB_CA_ATTRIBUTES    *pCaAttributes OPTIONAL,
		OUT IB_PORT_ATTRIBUTES		**ppPortAttributes OPTIONAL,
		OUT uint32 *pCaCount OPTIONAL,
		OUT uint32 *pPortCount OPTIONAL
		)
{
	EUI64	*caGuids = NULL;
	uint32  caCount;
	FSTATUS status             = FSUCCESS;
	uint8   i;

	_DBG_ENTER_FUNC(iba_get_portguid);

	_DBG_INFO(("iba_get_portguid: ca %u, port %u\n", ca, port));

	status = iba_get_caguids_alloc(&caCount, &caGuids);
	if ((status != FSUCCESS) || (caGuids == NULL))
		goto done;
	if (pCaCount)
		*pCaCount = caCount;
	if (pPortCount)
		*pPortCount = 0;

	if (caCount == 0)
	{
		// no CAs found
		status = FNOT_FOUND;
		goto done;
	}

	if (ca > 0)
	{
		// look for port number in a specific CA
		if (ca > caCount)
		{
			// ca number too large
			status = FNOT_FOUND;
			goto done;
		}

		status = iba_get_ca_portguid(caGuids[ca-1], port, pPortGuid,
									pCaAttributes, ppPortAttributes, pPortCount);
		if (FSUCCESS != status)
			goto done;
		if (pCaGuid)
			*pCaGuid = caGuids[ca-1];
	} else {
		// system wide port number encoding
		uint32 totalPorts = 0;
		status = FNOT_FOUND;

		for (i = 0; i < caCount; i++)
		{
			EUI64 portGuid;
			uint32 portCount;
			FSTATUS status2;
			
			status2 = iba_get_ca_portguid(caGuids[i],
							port > totalPorts?port-totalPorts:port,
							&portGuid, pCaAttributes,
							ppPortAttributes, &portCount);
			if (FNOT_FOUND == status && FSUCCESS == status2) {
				// found the desired port
				if (pPortGuid)
					*pPortGuid = portGuid;
				if (pCaGuid)
					*pCaGuid = caGuids[i];
				ppPortAttributes = NULL;	// avoid overwritting on next loop
				pCaAttributes = NULL;	// avoid overwritting on next loop
				port = 1;	// a simple valid value for subsequent loops
				// we keep looping to count totalPorts
			}
			if (FNOT_FOUND != status2) {
				status = status2;
				goto done;
			}
			totalPorts += portCount;
		}
		if (pPortCount)
			*pPortCount = totalPorts;
	}
done:
	if (caGuids)
		MemoryDeallocate(caGuids);

	_DBG_LEAVE_FUNC();
	return status;
}

/* format an error message when iba_get_portguid returns FNOT_FOUND
 *
 * INPUTS:
 * 	ca, port - inputs provided to failed call to iba_get_portguid
 *	CaCount, PortCount - outputs returned by failed call to iba_get_portguid
 *
 * RETURNS:
 * 	string formatted with error message.  This is a pointer into a static
 *  string which will be invalidated upon next call to this function.
 */
const char*
iba_format_get_portguid_error(
		IN uint32 ca,
		IN uint32 port,
		IN uint32 caCount,
		IN uint32 portCount
		)
{
	static char errstr[80];

	_DBG_ENTER_FUNC(iba_format_get_portguid_error);
	if (ca) {
		if (portCount) {
			if (port) {
				sprintf(errstr, "Invalid port number: %u, Ca %u only has %u Ports",
						port, ca, portCount);
			} else {
				sprintf(errstr, "No Active Ports found on Ca %u", ca);
			}
		} else {
			sprintf(errstr,"Invalid CA number: %u, only have %u CAs",
						ca, caCount);
		}
	} else {
		if (portCount) {
			if (port) {
				sprintf(errstr, "Invalid port number: %u, System only has %u Ports",
						port, portCount);
			} else {
				sprintf(errstr, "No Active ports found in System");
			}
		} else {
			sprintf(errstr,"No CAs found in System");
		}
	}
	_DBG_LEAVE_FUNC();
	return errstr;
}

// search Ports of a given CA to find the Port with the matching GID
// if PortAtttibutes is not NULL, allocated port attributes are
// returned and caller must MemoryDeallocate(*PortAttributes)
FSTATUS
iba_find_ca_gid(
	IN const IB_GID *pGid,
	IN EUI64 CaGuid,
	OUT OPTIONAL EUI64 *pPortGuid,
	OUT OPTIONAL uint8 *pGidIndex,
	OUT OPTIONAL IB_PORT_ATTRIBUTES	**PortAttributes,
	OUT OPTIONAL uint8	*pLocalCaAckDelay
	)
{
	FSTATUS status;
	IB_CA_ATTRIBUTES *CaAttributes = NULL;

	if (pLocalCaAckDelay)
		*pLocalCaAckDelay = 0;
	status = iba_find_ca_gid2(pGid, CaGuid, pPortGuid, pGidIndex,
							PortAttributes, &CaAttributes);
	if (status == FSUCCESS) {
		if (pLocalCaAckDelay)
			*pLocalCaAckDelay = CaAttributes->LocalCaAckDelay;
		MemoryDeallocate(CaAttributes);
	}
	return status;
}

// search Ports of a given CA to find the Port with the matching GID
// if PortAtttibutes is not NULL, allocated port attributes are
// returned and caller must MemoryDeallocate(*PortAttributes)
// if CaAttributes is not NULL, allocated CA attributes are
// returned and caller must MemoryDeallocate(*CaAttributes)
FSTATUS
iba_find_ca_gid2(
	IN const IB_GID *pGid,
	IN EUI64 CaGuid,
	OUT OPTIONAL EUI64 *pPortGuid,
	OUT OPTIONAL uint8 *pGidIndex,
	OUT OPTIONAL IB_PORT_ATTRIBUTES	**PortAttributes,
	OUT OPTIONAL IB_CA_ATTRIBUTES **CaAttributes
	)
{
	IB_CA_ATTRIBUTES *caAttr = NULL;
	IB_PORT_ATTRIBUTES *portAttr;
	FSTATUS status;
	unsigned i;
	int32 index;

	_DBG_ENTER_FUNC(iba_find_ca_gid);

	if (pGidIndex)
		*pGidIndex = 0;
	if (pPortGuid)
		*pPortGuid = 0;
	if (PortAttributes)
		*PortAttributes = NULL;
	if (CaAttributes)
		*CaAttributes = NULL;

	caAttr = (IB_CA_ATTRIBUTES*)MemoryAllocate(sizeof(*caAttr), FALSE, ALLOC_TAG);
	if (caAttr == NULL)
	{
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}

	status = iba_query_ca_by_guid_alloc(CaGuid, caAttr);
	if (status != FSUCCESS)
		goto free;

	status = FNOT_FOUND;
	portAttr = caAttr->PortAttributesList;
	for (i=0; i< caAttr->Ports; ++i, portAttr = portAttr->Next)
	{
		index = GetGidIndex(portAttr, pGid);
		if (index != -1)
		{
			// found it
			status = FSUCCESS;
			if (pGidIndex)
				*pGidIndex = (uint16)index;
			if (pPortGuid)
				*pPortGuid = portAttr->GUID;
			if (PortAttributes)
			{
				size_t size = IbPortAttributesSize(portAttr);
				*PortAttributes = (IB_PORT_ATTRIBUTES*)MemoryAllocate(size, FALSE, ALLOC_TAG);
				if (*PortAttributes == NULL)
				{
					status = FINSUFFICIENT_MEMORY;
				} else {
					IbCopyPortAttributes(*PortAttributes, portAttr, size);
					(*PortAttributes)->Next = NULL;
				}
			}
			break;
		}
	}
	MemoryDeallocate(caAttr->PortAttributesList);
	caAttr->PortAttributesListSize = 0;
	caAttr->PortAttributesList = NULL;
free:
	if (status == FSUCCESS && CaAttributes)
		*CaAttributes = caAttr;
	else
		MemoryDeallocate(caAttr);
done:
	_DBG_LEAVE_FUNC();

	return status;
}

// search all Ports of all CAs to find the Port with the matching GID
// if PortAtttibutes is not NULL, allocated port attributes are
// returned and caller must MemoryDeallocate(*PortAttributes)
// if CaAttributes is not NULL, allocated CA attributes are
// returned and caller must MemoryDeallocate(*CaAttributes)
FSTATUS iba_find_gid(
	IN const IB_GID *pGid,
	OUT OPTIONAL EUI64 *pCaGuid,
	OUT OPTIONAL EUI64 *pPortGuid,
	OUT OPTIONAL uint8 *pGidIndex,
	OUT OPTIONAL IB_PORT_ATTRIBUTES	**PortAttributes,
	OUT OPTIONAL IB_CA_ATTRIBUTES **CaAttributes
	)
{
	uint32	CaCount;
	EUI64	*CaGuidList = NULL;
	FSTATUS status;
	uint32 i;

	_DBG_ENTER_FUNC(iba_find_gid);

	status = iba_get_caguids_alloc(&CaCount, &CaGuidList);
	if ((status != FSUCCESS) || (CaGuidList == NULL))
		goto done;

	for (i=0; i<CaCount; ++i)
	{
		status = iba_find_ca_gid2(pGid, CaGuidList[i], pPortGuid, pGidIndex,
									PortAttributes, CaAttributes);
		if (status != FNOT_FOUND) {
			if (pCaGuid)
				*pCaGuid = CaGuidList[i];
			goto free;
		}
	}
	status = FNOT_FOUND;
free:
	MemoryDeallocate(CaGuidList);
done:
	_DBG_LEAVE_FUNC();
	return status;
}

// Move a QP to the Reset State
FSTATUS
iba_reset_qp(
	IN IB_HANDLE QpHandle,
	OUT OPTIONAL IB_QP_ATTRIBUTES_QUERY	*QpAttributes
	)
{
	static IB_QP_ATTRIBUTES_MODIFY qpAttr = {
		RequestState: QPStateReset,
		Attrs: 0
	};

	return iba_modify_qp(QpHandle, &qpAttr, QpAttributes);
}

//initialize a previously created queue pair.
FSTATUS iba_init_qp(
	IN  IB_HANDLE						QpHandle,
	IN	EUI64							PortGuid,
	IN	uint16							PkeyIndex,
	IN	IB_ACCESS_CONTROL				AccessControl,
	IN	IB_Q_KEY						Qkey OPTIONAL,
	OUT OPTIONAL IB_QP_ATTRIBUTES_QUERY	*QpAttributes
	)
{
	FSTATUS status;
	IB_QP_ATTRIBUTES_MODIFY *QpAttrInit;

	_DBG_ENTER_FUNC(iba_init_qp);
	QpAttrInit = (IB_QP_ATTRIBUTES_MODIFY*)MemoryAllocate2AndClear(
						sizeof(*QpAttrInit), IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, ALLOC_TAG);
	if (! QpAttrInit) {
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}
	QpAttrInit->RequestState = QPStateInit;
	QpAttrInit->PortGUID = PortGuid;
	QpAttrInit->PkeyIndex = PkeyIndex;
	QpAttrInit->AccessControl = AccessControl;
	QpAttrInit->Attrs = (IB_QP_ATTR_PORTGUID|IB_QP_ATTR_PKEYINDEX
						| IB_QP_ATTR_ACCESSCONTROL);
	if (Qkey) {	// only valid for RD QPs
		QpAttrInit->Qkey = Qkey;
		QpAttrInit->Attrs |= IB_QP_ATTR_QKEY;
	}
	status = iba_modify_qp(QpHandle, QpAttrInit, QpAttributes);
	MemoryDeallocate(QpAttrInit);
done:
	_DBG_LEAVE_FUNC();
	return status;
}

//initialize a previously created UD queue pair.
FSTATUS iba_ready_ud_qp(
	IN  IB_HANDLE						QpHandle,
	IN	EUI64							PortGuid,
	IN	uint16							PkeyIndex,
	IN	IB_Q_KEY						Qkey,
	OUT OPTIONAL IB_QP_ATTRIBUTES_QUERY	*QpAttributes
	)
{
	FSTATUS status;
	IB_QP_ATTRIBUTES_MODIFY *QpAttrInit;

	static IB_QP_ATTRIBUTES_MODIFY QpAttrUdRtr = {
		RequestState: QPStateReadyToRecv,
		Attrs:0
	};
	static IB_QP_ATTRIBUTES_MODIFY QpAttrUdRts = {
		RequestState: QPStateReadyToSend,
		Attrs:IB_QP_ATTR_SENDPSN,
		SendQDepth: 0,
		RecvQDepth:0,
		SendDSListDepth:0,
		RecvDSListDepth:0,
		PortGUID:0,
		Qkey:0,
		PkeyIndex:0,
		AccessControl: {0},
		RecvPSN:0,
		DestQPNumber:0,
		PathMTU:0,
		Reserved1:0,
		Reserved2:0,
		DestAV: {0},
		ResponderResources:0,
		MinRnrTimer:0,
		Reserved3:0,
		AltPortGUID:0,
		AltDestAV:{0},
		AltPkeyIndex:0,
		Reserved4:0,
		SendPSN:1	// not critical since no checking on receiver
	};

	_DBG_ENTER_FUNC(iba_init_qp);
	QpAttrInit = (IB_QP_ATTRIBUTES_MODIFY*)MemoryAllocate2AndClear(
						sizeof(*QpAttrInit), IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, ALLOC_TAG);
	if (! QpAttrInit) {
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}
	QpAttrInit->RequestState = QPStateInit;
	QpAttrInit->PortGUID = PortGuid;
	QpAttrInit->PkeyIndex = PkeyIndex;
	QpAttrInit->Qkey = Qkey;
	QpAttrInit->Attrs = (IB_QP_ATTR_PORTGUID|IB_QP_ATTR_PKEYINDEX
						| IB_QP_ATTR_QKEY);
	status = iba_modify_qp(QpHandle, QpAttrInit, QpAttributes);
	MemoryDeallocate(QpAttrInit);

	if (status == FSUCCESS)
		status = iba_modify_qp(QpHandle, &QpAttrUdRtr, QpAttributes);

	if (status == FSUCCESS)
		status = iba_modify_qp(QpHandle, &QpAttrUdRts, QpAttributes);
done:
	_DBG_LEAVE_FUNC();
	return status;
}

// Reset and Reinitialize a previously initialized QP
FSTATUS iba_reinit_qp(
	IN  IB_HANDLE						QpHandle,
	OUT OPTIONAL IB_QP_ATTRIBUTES_QUERY	*QpAttributes
	)
{
	FSTATUS status;
	IB_QP_ATTRIBUTES_QUERY *QpAttrQuery = NULL;
	IB_QP_ATTRIBUTES_MODIFY *QpAttrInit = NULL;
	void* context;

	_DBG_ENTER_FUNC(iba_reinit_qp);
	QpAttrQuery = (IB_QP_ATTRIBUTES_QUERY*)MemoryAllocate2AndClear(
						sizeof(*QpAttrQuery), IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, ALLOC_TAG);
	if (! QpAttrQuery) {
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}
	QpAttrInit = (IB_QP_ATTRIBUTES_MODIFY*)MemoryAllocate2AndClear(
						sizeof(*QpAttrInit), IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, ALLOC_TAG);
	if (! QpAttrInit) {
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}

	status = iba_query_qp(QpHandle, QpAttrQuery, &context);
	if (status != FSUCCESS)
		goto done;

	status = iba_reset_qp(QpHandle, NULL);
	if (status != FSUCCESS)
		goto done;

	if ((QpAttrQuery->Attrs & (IB_QP_ATTR_PORTGUID|IB_QP_ATTR_PKEYINDEX
					| IB_QP_ATTR_ACCESSCONTROL))
		!= (IB_QP_ATTR_PORTGUID|IB_QP_ATTR_PKEYINDEX
					| IB_QP_ATTR_ACCESSCONTROL))
	{
		status = FINVALID_STATE;
		goto done;
	}
	QpAttrInit->RequestState = QPStateInit;
	QpAttrInit->PortGUID = QpAttrQuery->PortGUID;
	QpAttrInit->PkeyIndex = QpAttrQuery->PkeyIndex;
	QpAttrInit->AccessControl = QpAttrQuery->AccessControl;
	QpAttrInit->Attrs = QpAttrQuery->Attrs & (IB_QP_ATTR_PORTGUID
					| IB_QP_ATTR_PKEYINDEX | IB_QP_ATTR_ACCESSCONTROL
					| IB_QP_ATTR_QKEY);
	if (QpAttrInit->Attrs & IB_QP_ATTR_QKEY) {
		// must be RD/UD QP
		QpAttrInit->Qkey = QpAttrQuery->Qkey;
	}
	status = iba_modify_qp(QpHandle, QpAttrInit, QpAttributes);
done:
	if (QpAttrQuery)
		MemoryDeallocate(QpAttrQuery);
	if (QpAttrInit)
		MemoryDeallocate(QpAttrInit);
	_DBG_LEAVE_FUNC();
	return status;
}

// Move a QP to the Error State
FSTATUS
iba_error_qp(
	IN IB_HANDLE QpHandle,
	OUT OPTIONAL IB_QP_ATTRIBUTES_QUERY	*QpAttributes
	)
{
	static IB_QP_ATTRIBUTES_MODIFY qpAttr = {
		RequestState: QPStateError,
		Attrs: 0
	};

	return iba_modify_qp(QpHandle, &qpAttr, QpAttributes);
}

// Move a QP to the Send Q Drain State
FSTATUS
iba_sendq_drain_qp(
	IN  IB_HANDLE						QpHandle,
	IN  boolean							EnableSQDAsyncEvent,
	OUT OPTIONAL IB_QP_ATTRIBUTES_QUERY	*QpAttributes
	)
{
	// keep structures const so we don't need locks
	static IB_QP_ATTRIBUTES_MODIFY qpAttrEnable = {
		RequestState: QPStateSendQDrain,
		Attrs: IB_QP_ATTR_ENABLESQDASYNCEVENT,
		SendQDepth:0,
		RecvQDepth:0,
		SendDSListDepth:0,
		RecvDSListDepth:0,
		PortGUID:0,
		Qkey:0,
		PkeyIndex:0,
		AccessControl: { 0 },
		RecvPSN:0,
		DestQPNumber:0,
		PathMTU:0,
		Reserved1:0,
		Reserved2:0,
		DestAV: { 0 },
		ResponderResources:0,
		MinRnrTimer:0,
		Reserved3:0,
		AltPortGUID:0,
		AltDestAV: { 0 },
		AltPkeyIndex: 0,
		Reserved4: 0,
		SendPSN: 0,
		InitiatorDepth: 0,
		LocalAckTimeout: 0,
		RetryCount: 0,
		RnrRetryCount: 0,
		FlowControl:FALSE,
		APMState:APMStateNoTransition,
		AltLocalAckTimeout:0,
		EnableSQDAsyncEvent: TRUE
	};
	static IB_QP_ATTRIBUTES_MODIFY qpAttrDisable = {
		RequestState: QPStateSendQDrain,
		Attrs: IB_QP_ATTR_ENABLESQDASYNCEVENT,
		SendQDepth:0,
		RecvQDepth:0,
		SendDSListDepth:0,
		RecvDSListDepth:0,
		PortGUID:0,
		Qkey:0,
		PkeyIndex:0,
		AccessControl: { 0 },
		RecvPSN:0,
		DestQPNumber:0,
		PathMTU:0,
		Reserved1:0,
		Reserved2:0,
		DestAV: { 0 },
		ResponderResources:0,
		MinRnrTimer:0,
		Reserved3:0,
		AltPortGUID:0,
		AltDestAV: { 0 },
		AltPkeyIndex: 0,
		Reserved4: 0,
		SendPSN: 0,
		InitiatorDepth: 0,
		LocalAckTimeout: 0,
		RetryCount: 0,
		RnrRetryCount: 0,
		FlowControl:FALSE,
		APMState:APMStateNoTransition,
		AltLocalAckTimeout:0,
		EnableSQDAsyncEvent: FALSE
	};

	return iba_modify_qp(QpHandle,
						EnableSQDAsyncEvent?&qpAttrEnable:&qpAttrDisable,
						QpAttributes);
}

static const char* const WrOpText[] = {
	"WROpSend",
	"WROpRdmaWrite",
	"WROpRecv",
	"WROpRdmaRead",
	"WROpMWBind",
	"WROpFetchAdd",
	"WROpCompareSwap",
	"WROpRecvUnsignaled"
};

const char* iba_wr_op_msg(IB_WR_OP wr_op)
{
	if (wr_op < 0 || wr_op >= (int)(sizeof(WrOpText)/sizeof(char*)))
		return "Unknown WR OP";
	else
		return WrOpText[wr_op];
}

static const char* const WcStatusText[] = {
	"WRStatusSuccess",
	"WRStatusLocalLength",
	"WRStatusLocalOp",
	"WRStatusLocalProtection",
	"WRStatusTransport",
	"WRStatusWRFlushed",
	"WRStatusRemoteConsistency",
	"WRStatusMWBind",
	"WRStatusRemoteAccess",
	"WRStatusRemoteOp",
	"WRStatusRemoteInvalidReq",
	"WRStatusSeqErrRetry",
	"WRStatusRnRRetry",
	"WRStatusTimeoutRetry",
	"WRStatusBadResponse",
	"WRStatusInvalidLocalEEC",
	"WRStatusInvalidLocalEECState",
	"WRStatusInvalidLocalEECOp",
	"WRStatusLocalRDDViolation",
	"WRStatusRemoteInvalidRDReq",
	"WRStatusLocalAccess",
	"WRStatusRemoteAborted"
};

const char* iba_wc_status_msg(IB_WC_STATUS wc_status)
{
	if (wc_status < 0 || wc_status >= (int)(sizeof(WcStatusText)/sizeof(char*)))
		return "Unknown WRStatus";
	else
		return WcStatusText[wc_status];
}

static const char* const WcTypeText[] = {
	"WCTypeSend",
	"WCTypeRdmaWrite",
	"WCTypeRecv",
	"WCTypeRdmaRead",
	"WCTypeMWBind",
	"WCTypeFetchAdd",
	"WCTypeCompareSwap"
};

const char* iba_wc_type_msg(IB_WC_TYPE wc_type)
{
	if (wc_type < 0 || wc_type >= (int)(sizeof(WcTypeText)/sizeof(char*)))
		return "Unknown WCType";
	else
		return WcTypeText[wc_type];
}

static const char* const WcRemoteOpText[] = {
	"WCRemoteOpSend",
	"WCRemoteOpRdmaWrite"
};

const char* iba_wc_remote_op_msg(IB_WC_REMOTE_OP wc_remote_op)
{
	if (wc_remote_op < 0 || wc_remote_op >= (int)(sizeof(WcRemoteOpText)/sizeof(char*)))
		return "Unknown WCRemoteOp";
	else
		return WcRemoteOpText[wc_remote_op];
}

static const char * const QpTypeText[] = {
	"QPTypeReliableConnected",
	"QPTypeUnreliableConnected",
	"QPTypeReliableDatagram",
	"QPTypeUnreliableDatagram",
	"QPTypeSMI",
	"QPTypeGSI",
	"QPTypeRawDatagram",
	"QPTypeRawIPv6",
	"QPTypeRawEthertype"
};

const char* iba_qp_type_msg(IB_QP_TYPE qp_type)
{
	if (qp_type < 0 || qp_type >= (int)(sizeof(QpTypeText)/sizeof(char*)))
		return "Unknown QpType";
	else
		return QpTypeText[qp_type];
}

static const char * const QpStateText[] = {
	"QPStateNoTransition",
	"QPStateReset",
	"QPStateInit",
	"QPStateReadyToRecv",
	"QPStateReadyToSend",
	"QPStateSendQDrain",
	"QPStateSendQError",
	"QPStateError"
};

const char* iba_qp_state_msg(IB_QP_STATE qp_state)
{
	if (qp_state < 0 || qp_state >= (int)(sizeof(QpStateText)/sizeof(char*)))
		return "Unknown QpState";
	else
		return QpStateText[qp_state];
}

static const char * const QpApmStateText[] = {
	"APMStateNoTransition",
	"APMStateMigrated",
	"APMStateRearm",
	"APMStateArmed"
};

const char* iba_qp_apm_state_msg(IB_QP_APM_STATE qp_apm_state)
{
	if (qp_apm_state < 0 || qp_apm_state >= (int)(sizeof(QpApmStateText)/sizeof(char*)))
		return "Unknown QpApmState";
	else
		return QpApmStateText[qp_apm_state];
}

const char* iba_event_record_msg(IB_EVENT_RECORD *p_event)
{
    switch (p_event->EventType) 
    {
        default:
			return "Unknown Event";
            break;
		case AsyncEventPathMigrated:
            return "QP Path Migrated";
			break;
		case AsyncEventEEPathMigrated:
            return "EE Path Migrated";
			break;
        case AsyncEventRecvQ:
            switch (p_event->EventCode) 
            {
                case IB_AE_RQ_PATH_MIG_ERR:
            		return "QP Path Migration Error";
					break;
                case IB_AE_RQ_COMM_ESTABLISHED:
            		return "QP Communication Established";
					break;
                case IB_AE_RQ_ERROR_STATE: 
            		return "QP Error";
					break;
                case IB_AE_RQ_WQE_ACCESS:
            		return "QP Local Access Violation";
					break;
				default:
            		return "RQ Other Event";
					break;
			}
			break;
        case AsyncEventSendQ:
            switch (p_event->EventCode) 
            {
                case IB_AE_SQ_DRAIN:
            		return "Send Q Drained";
					break;
				default:
            		return "SQ Other Event";
					break;
			}
			break;
        case AsyncEventCQ:
            switch (p_event->EventCode) 
            {
                case IB_AE_CQ_ACCESS_ERR:
            		return "CQ Access Error";
					break;
                case IB_AE_CQ_OVERRUN:
            		return "CQ Overrun";
					break;
				default:
            		return "CQ Other Event";
					break;
			}
			break;
        case AsyncEventEE:
            switch (p_event->EventCode) 
            {
                case IB_AE_EE_PATH_MIG_ERR:
            		return "EEC Path Migration Error";
					break;
                case IB_AE_EE_COMM_ESTABLISHED:
            		return "EEC Communication Established";
					break;
                case IB_AE_EE_ERROR_STATE: 
            		return "EEC Error";
					break;
				default:
            		return "EEC Other Event";
					break;
			}
			break;
        case AsyncEventCA:
        {
            return "CA Cat. Error";
			break;
		}
        case AsyncEventPort:
        {
            return "Port State Change";
			break;
		}
    }
}

static const char * const MadInvalidFieldText[] = {
	"Valid Field",	// should not be used below
	"Unsupported Class or Version",
	"Unsupported Method",
	"Unsupported Method/Attribute Combination",
	"Reserved Invalid Field 4",
	"Reserved Invalid Field 5",
	"Reserved Invalid Field 6",
	"Invalid Attribute or Attribute Modifier"
};

const char* iba_mad_status_msg(MAD_STATUS madStatus)
{
	if (madStatus.AsReg16 == MAD_STATUS_SUCCESS)
		return "Success";
	else if (madStatus.S.Busy)
		return "Busy";
	else if (madStatus.S.RedirectRqrd)
		return "Redirection Required";
	else if (madStatus.S.InvalidField)
		return MadInvalidFieldText[madStatus.S.InvalidField];
	else if (madStatus.S.ClassSpecific)
		return "Class Specific Status";
	else
		return "Unknown Mad Status";
}

// this function is useful when we have DR.s.Status which is a 15 bit bitfield
const char* iba_mad_status_msg2(uint16 madStatus)
{
	MAD_STATUS mad_status;

	mad_status.AsReg16 = madStatus;
	return iba_mad_status_msg(mad_status);
}

const char* iba_rmpp_status_msg(RMPP_STATUS rmppStatus)
{
	switch (rmppStatus) {
	case RMPP_STATUS_NORMAL:
		return "Normal";
	case RMPP_STATUS_RESOURCES_EXHAUSTED:
		return "Resources Exhausted";
	case RMPP_STATUS_TOTAL_TIME_TOO_LONG:
		return "Total Time Too Long";
	case RMPP_STATUS_INCONSISTENT_PAYLOAD_LEN:
		return "Inconsistent Last and PayloadLength";
	case RMPP_STATUS_INCONSISTENT_FIRST:
		return "Inconsistent First and SegmentNumber";
	case RMPP_STATUS_BAD_RMPP_TYPE:
		return "Bad RMPPType";
	case RMPP_STATUS_NEW_WL_TOO_SMALL:
		return "NewWindowLast Too Small";
	case RMPP_STATUS_SEGMENT_NUMBER_TOO_BIG:
		return "SegmentNumber Too Big";
	case RMPP_STATUS_ILLEGAL_STATUS:
		return "Illegal Status";
	case RMPP_STATUS_UNSUPPORTED_VERSION:
		return "Unsupported Version";
	case RMPP_STATUS_TOO_MANY_RETRIES:
		return "Too Many Retries";
	case RMPP_STATUS_UNSPECIFIED:
		return "Unspecified";
	default:
		if (rmppStatus >= 128 && rmppStatus <= 191)
			return "Class Specific Status";
		else if (rmppStatus >= 192 && rmppStatus <= 255)
			return "Vendor Specific Status";
		else
			/* includes RMPP_STATUS_START_RESERVED - RMPP_STATUS_END_RESERVED */
			return "Unknown Rmpp Status";
	}
}
