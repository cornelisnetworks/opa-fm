/* BEGIN_ICS_COPYRIGHT3 ****************************************

Copyright (c) 2015-2018, Intel Corporation

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include "pm_topology.h"
#include "pa_access.h"
#include "iba/stl_pa_priv.h"
#include "fm_xml.h"
#include <stdio.h>
#include <time.h>
#ifndef __VXWORKS__
#include <strings.h>
#endif

boolean	isFirstImg = TRUE;
boolean	isUnexpectedClearUserCounters = FALSE;

/*************************************************************************************
*
* paGetGroupList - return list of group names
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     pmGroupList - pointer to caller-declared data area to return names
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paGetGroupList(Pm_t *pm, PmGroupList_t *GroupList, STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	Status_t vStatus;
	FSTATUS status = FSUCCESS;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	PmImage_t *pmimagep = NULL;
	boolean requiresLock = TRUE;
	int i;

	// check input parameters
	if (!pm || !GroupList)
		return(FINVALID_PARAMETER);

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	// collect statistics from last sweep and populate pmGroupInfo
	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, NULL, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	GroupList->NumGroups = pmimagep->NumGroups + 1; // Number of Groups plus ALL Group
	vStatus = vs_pool_alloc(&pm_pool, GroupList->NumGroups * STL_PM_GROUPNAMELEN,
		(void*)&GroupList->GroupList);
	if (vStatus != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate name list buffer for GroupList rc:", vStatus);
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}
	// no lock needed, group names are constant once PM starts
	StringCopy(GroupList->GroupList[0].Name, PA_ALL_GROUP_NAME, STL_PM_GROUPNAMELEN);
	for (i = 0; i < pmimagep->NumGroups; i++) {
		StringCopy(GroupList->GroupList[i+1].Name, pmimagep->Groups[i].Name, STL_PM_GROUPNAMELEN);
	}

	*returnImageId = retImageId;
done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

/*************************************************************************************
*
* paGetGroupInfo - return group information
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     groupName - pointer to name of group
*     pmGroupInfo - pointer to caller-declared data area to return group information
*
*  Return:
*     FSTATUS - FSUCCESS if OK, FERROR
*
*
*************************************************************************************/

FSTATUS paGetGroupInfo(Pm_t *pm, char *groupName, PmGroupInfo_t *pmGroupInfo,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS status = FSUCCESS;
	PmGroupImage_t pmGroupImage;
	PmImage_t *pmimagep = NULL;
	PmNode_t *pmnodep = NULL;
	PmPortImage_t *pmPortImageP = NULL, *pmPortImageNeighborP = NULL;
	PmPort_t *pmportp = NULL;
	uint8 portnum;
	uint32 imageIndex, imageInterval;
	int groupIndex = -1;
	boolean requiresLock = TRUE, isInternal = FALSE, isGroupAll = FALSE;
	STL_LID lid;

	// check input parameters
	if (!pm || !groupName || !pmGroupInfo)
		return(FINVALID_PARAMETER);
	if (groupName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal groupName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER) ;
	}

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	status = LocateGroup(pmimagep, groupName, &groupIndex);
	if (status != FSUCCESS) {
		IB_LOG_WARN_FMT(__func__, "Group %.*s not Found: %s", (int)sizeof(groupName), groupName, FSTATUS_ToString(status));
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
		goto done;
	}
	// If Success, but Index is still -1, then it is All Group
	if (groupIndex == -1) isInternal = isGroupAll = TRUE;

	imageInterval = pmimagep->imageInterval;

	memset(&pmGroupImage, 0, sizeof(PmGroupImage_t));
	ClearGroupStats(&pmGroupImage);

	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			pmPortImageP = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pmimagep, pmPortImageP, groupIndex, isGroupAll, &isInternal)) {
				if (isInternal) {
					if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
						PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.IntUtil.pmaNoRespPorts, IB_UINT16_MAX);
					}
					pmGroupImage.NumIntPorts++;
					UpdateInGroupStats(pm, imageIndex, pmportp, &pmGroupImage, imageInterval);
					if (pmPortImageP->neighbor == NULL && pmportp->portNum != 0) {
						PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.IntUtil.topoIncompPorts, IB_UINT16_MAX);
					}
				} else {
					if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
						PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.SendUtil.pmaNoRespPorts, IB_UINT16_MAX);
					}
					pmGroupImage.NumExtPorts++;
					if (pmPortImageP->neighbor == NULL) {
						PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.RecvUtil.topoIncompPorts, IB_UINT16_MAX);
					} else {
						pmPortImageNeighborP = &pmPortImageP->neighbor->Image[imageIndex];
						if (pmPortImageNeighborP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
							PA_INC_COUNTER_NO_OVERFLOW(pmGroupImage.RecvUtil.pmaNoRespPorts, IB_UINT16_MAX);
						}
					}
					UpdateExtGroupStats(pm, imageIndex, pmportp, &pmGroupImage, imageInterval);
				}
			}
		}
	}
	FinalizeGroupStats(&pmGroupImage);
	StringCopy(pmGroupInfo->groupName, groupName, STL_PM_GROUPNAMELEN);
	pmGroupInfo->NumIntPorts = pmGroupImage.NumIntPorts;
	pmGroupInfo->NumExtPorts = pmGroupImage.NumExtPorts;
	memcpy(&pmGroupInfo->IntUtil, &pmGroupImage.IntUtil, sizeof(PmUtilStats_t));
	memcpy(&pmGroupInfo->SendUtil, &pmGroupImage.SendUtil, sizeof(PmUtilStats_t));
	memcpy(&pmGroupInfo->RecvUtil, &pmGroupImage.RecvUtil, sizeof(PmUtilStats_t));
	memcpy(&pmGroupInfo->IntErr, &pmGroupImage.IntErr, sizeof(PmErrStats_t));
	memcpy(&pmGroupInfo->ExtErr, &pmGroupImage.ExtErr, sizeof(PmErrStats_t));
	pmGroupInfo->MinIntRate = pmGroupImage.MinIntRate;
	pmGroupInfo->MaxIntRate = pmGroupImage.MaxIntRate;
	pmGroupInfo->MinExtRate = pmGroupImage.MinExtRate;
	pmGroupInfo->MaxExtRate = pmGroupImage.MaxExtRate;

	*returnImageId = retImageId;

done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

#define PORTLISTCHUNK 256

/*************************************************************************************
*
* paGetGroupConfig - return group configuration information
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     groupName - pointer to name of group
*     pmGroupConfig - pointer to caller-declared data area to return group config info
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paGetGroupConfig(Pm_t *pm, char *groupName, PmGroupConfig_t *pmGroupConfig,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	PmImage_t *pmimagep = NULL;
	PmNode_t *pmnodep = NULL;
	PmPort_t *pmportp = NULL;
	PmPortImage_t *portImage = NULL;
	uint8 portnum;
	STL_LID lid;
	uint32 imageIndex;
	FSTATUS status = FSUCCESS;
	boolean requiresLock = TRUE, isGroupAll = FALSE;
	int groupIndex = -1;

	// check input parameters
	if (!pm || !groupName || !pmGroupConfig)
		return(FINVALID_PARAMETER);
	if (groupName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal groupName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER) ;
	}

	// initialize group config port list counts
	pmGroupConfig->NumPorts = 0;
	pmGroupConfig->portListSize = 0;
	pmGroupConfig->portList = NULL;

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	status = LocateGroup(pmimagep, groupName, &groupIndex);
	if (status != FSUCCESS) {
		IB_LOG_WARN_FMT(__func__, "Group %.*s not Found: %s", (int)sizeof(groupName), groupName, FSTATUS_ToString(status));
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
		goto done;
	}
	// If Success, but Index is still -1, then it is All Group
	if (groupIndex == -1) isGroupAll = TRUE;

	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pmimagep, portImage, groupIndex, isGroupAll, NULL)) {
				if (pmGroupConfig->portListSize == pmGroupConfig->NumPorts) {
					pmGroupConfig->portListSize += PORTLISTCHUNK;
				}
				pmGroupConfig->NumPorts++;
			}
		}
	}
	// check if there are ports to sort
	if (!pmGroupConfig->NumPorts) {
		IB_LOG_INFO_FMT(__func__, "Group %.*s has no ports", (int)sizeof(groupName), groupName);
		goto norecords;
	}
	// allocate the port list
	Status_t ret = vs_pool_alloc(&pm_pool, pmGroupConfig->portListSize * sizeof(PmPortConfig_t), (void *)&pmGroupConfig->portList);
	if (ret != VSTATUS_OK) {
		status = FINSUFFICIENT_MEMORY;
		IB_LOG_ERRORRC("Failed to allocate port list buffer for pmGroupConfig rc:", ret);
		goto done;
	}
	// copy the port list
	int i = 0;
	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pmimagep, portImage, groupIndex, isGroupAll, NULL)) {
				pmGroupConfig->portList[i].lid = lid;
				pmGroupConfig->portList[i].portNum = pmportp->portNum;
				pmGroupConfig->portList[i].guid = pmnodep->NodeGUID;
				memcpy(pmGroupConfig->portList[i].nodeDesc, (char *)pmnodep->nodeDesc.NodeString,
					   sizeof(pmGroupConfig->portList[i].nodeDesc));
				i++;
			}
		}
	}
norecords:
	StringCopy(pmGroupConfig->groupName, groupName, STL_PM_GROUPNAMELEN);
	*returnImageId = retImageId;

done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

/*************************************************************************************
*
* paGetGroupNodeInfo - return group node information
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     groupName - pointer to name of group
*     nodeGUID - GUID to select record
*     nodeLID - LID to select record
*     pmGroupNodeInfo - pointer to caller-declared data area to return group node info
*     imageId - image ID
*     returnImageId - pointer to image ID that is returned
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paGetGroupNodeInfo(Pm_t *pm, char *groupName, uint64 nodeGUID, STL_LID nodeLID, char *nodeDesc,
	PmGroupNodeInfo_t *pmGroupNodeInfo, STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	STL_LID lid, start_lid, end_lid;
	uint32 imageIndex;
	int groupIndex = -1;
	PmImage_t *pmimagep = NULL;
	FSTATUS status = FSUCCESS;
	boolean requiresLock = TRUE, isGroupAll = FALSE;
	PmNode_t *pmnodep = NULL;
	PmPort_t *pmportp = NULL;
	PmPortImage_t *portImage = NULL;
	uint8 portnum = 0;
	boolean nodeHasPortsInGroup = FALSE;

	// check input parameters
	if (!pm || !groupName || !pmGroupNodeInfo)
		return(FINVALID_PARAMETER);
	if (groupName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal groupName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER) ;
	}

	// initialize group config node list counts
	pmGroupNodeInfo->NumNodes = 0;
	pmGroupNodeInfo->nodeList = NULL;

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	status = LocateGroup(pmimagep, groupName, &groupIndex);
	if (status != FSUCCESS) {
		IB_LOG_WARN_FMT(__func__, "Group %.*s not Found: %s", (int)sizeof(groupName), groupName, FSTATUS_ToString(status));
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
		goto done;
	}
	// If Success, but Index is still -1, then it is All Group
	if (groupIndex == -1) isGroupAll = TRUE;

	start_lid = 1;
	end_lid = pmimagep->maxLid;

	//If LID or GUID is provided then that specific record is reported.  If both are provided then record that matches both is reported.
	if (nodeGUID) {
		pmnodep = pm_find_nodeguid(pm, nodeGUID);
		if (pmnodep)
			start_lid = end_lid = pmnodep->Image[imageIndex].lid;
	}
	if (nodeLID) {
		pmnodep = pmimagep->LidMap[nodeLID];
		if (pmnodep)
			start_lid = end_lid = nodeLID;
	}

	if (nodeGUID && nodeLID) {
		pmnodep = pmimagep->LidMap[nodeLID];
		if ((pmnodep) && (nodeGUID == pmnodep->NodeGUID)) {
			start_lid = end_lid = nodeLID;
		} else {
			IB_LOG_WARN_FMT(__func__, "No Nodes match Lid: 0x%08x and NodeGuid: 0x%"PRIx64, nodeLID, nodeGUID);
			goto norecords;
		}
	}

	/* If a node was found and node desc is not empty, check if nodeDesc also matches */
	if (pmnodep && nodeDesc[0] != '\0' &&
		strncmp((const char *)pmnodep->nodeDesc.NodeString, nodeDesc, STL_PM_NODEDESCLEN)) {
		/* if not, then no records */
		IB_LOG_WARN_FMT(__func__, "No Nodes match NodeDesc: %.*s",(int)sizeof(nodeDesc), nodeDesc);
		goto norecords;
	}

	if ( !pmnodep && (nodeLID || nodeGUID )) {
		IB_LOG_WARN_FMT(__func__, "No Nodes match Lid: 0x%08x or NodeGuid: 0x%"PRIx64, nodeLID, nodeGUID);
		goto norecords;
	}

	for_some_pmnodes(pmimagep, pmnodep, lid, start_lid, end_lid) {
		/* If node desc is not empty and does not match then check for next record */
		if (nodeDesc[0] != '\0') {
			if (strncmp((const char *)pmnodep->nodeDesc.NodeString, nodeDesc, STL_PM_NODEDESCLEN)) {
				continue; /* If node desc matches then only that record is reported */
			} else {
				start_lid = end_lid = lid;
			}
		}
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pmimagep, portImage, groupIndex, isGroupAll, NULL)) {
			// Once a port is in the group that node is counted. Then proceed with next node.
				pmGroupNodeInfo->NumNodes++;
				break; // Skip the rest of switch ports
			}
		}
	}

	// check if there are ports to copy
	if (!pmGroupNodeInfo->NumNodes) {
		if ( nodeDesc[0] != '\0') {
			IB_LOG_WARN_FMT(__func__, "No Nodes match NodeDesc: %.*s", (int)sizeof(nodeDesc), nodeDesc);
		} else {
			IB_LOG_WARN_FMT(__func__, "Group %.*s has no ports", (int)sizeof(groupName), groupName);
		}
		goto norecords;
	}
	// allocate the port list
	Status_t ret = vs_pool_alloc(&pm_pool, pmGroupNodeInfo->NumNodes* sizeof(PmNodeInfo_t), (void *)&pmGroupNodeInfo->nodeList);
	if (ret != VSTATUS_OK) {
		status = FINSUFFICIENT_MEMORY;
		IB_LOG_ERRORRC("Failed to allocate port list buffer for pmGroupNodeInfo rc:", ret);
		goto done;
	}

	// copy the port list
	int i = 0;
	for_some_pmnodes(pmimagep, pmnodep, lid, start_lid, end_lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pmimagep, portImage, groupIndex, isGroupAll, NULL)) {
				StlAddPortToPortMask(pmGroupNodeInfo->nodeList[i].portSelectMask, portnum);
				nodeHasPortsInGroup = TRUE;
			}
		}
		if (nodeHasPortsInGroup) {
			pmGroupNodeInfo->nodeList[i].nodeLid = lid;
			pmGroupNodeInfo->nodeList[i].nodeType = pmnodep->nodeType;
			pmGroupNodeInfo->nodeList[i].nodeGuid = pmnodep->NodeGUID;
			StringCopy(pmGroupNodeInfo->nodeList[i].nodeDesc, (char *)pmnodep->nodeDesc.NodeString,
				sizeof(pmGroupNodeInfo->nodeList[i].nodeDesc));
			nodeHasPortsInGroup = 0;
			i++;
		}
	}

norecords:
	StringCopy(pmGroupNodeInfo->groupName, groupName, STL_PM_GROUPNAMELEN);
	*returnImageId = retImageId;
done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

/*************************************************************************************
*
* paGetGroupLinkInfo - return group link information
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     groupName - pointer to name of group
*     inputLID - inputLID to select record
*     inputPort - inputPort to select record
*     pmGroupLinkInfo - pointer to caller-declared data area to return group link info
*     imageId - image ID
*     returnImageId - pointer to image ID that is returned
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paGetGroupLinkInfo(Pm_t *pm, char *groupName, STL_LID inputLID, uint8 inputPort,
	PmGroupLinkInfo_t *pmGroupLinkInfo, STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	STL_LID lid, start_lid, end_lid;
	uint8 start_port = 0, end_port = 0, portnum = 0;
	uint32 imageIndex;
	int groupIndex = -1;
	PmImage_t *pmimagep = NULL;
	FSTATUS status = FSUCCESS;
	boolean requiresLock = TRUE, isGroupAll = FALSE, isInternal = FALSE;
	PmNode_t *pmnodep = NULL;
	PmPort_t *pmportp = NULL, *nbrPort = NULL;
	PmPortImage_t *portImage = NULL;

	// check input parameters
	if (!pm || !groupName || !pmGroupLinkInfo)
		return(FINVALID_PARAMETER);
	if (groupName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal groupName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER) ;
	}

	// initialize group config node list counts
	pmGroupLinkInfo->NumLinks = 0;
	pmGroupLinkInfo->linkInfoList = NULL;

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	status = LocateGroup(pmimagep, groupName, &groupIndex);
	if (status != FSUCCESS) {
		IB_LOG_WARN_FMT(__func__, "Group %.*s not Found: %s", (int)sizeof(groupName), groupName, FSTATUS_ToString(status));
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
		goto error;
	}
	// If Success, but Index is still -1, then it is All Group
	if (groupIndex == -1) isGroupAll = TRUE;

	start_lid = 1;
	end_lid = pmimagep->maxLid;

	//If LID and portnum are provided then record with that LID and portnum is reported.
	if (inputLID) {
		start_lid = end_lid = inputLID;
		if( !inputPort) {
			inputPort = PM_ALL_PORT_SELECT;
		}
	}

	for_some_pmnodes(pmimagep, pmnodep, lid, start_lid, end_lid) {
		if (inputLID && (PM_ALL_PORT_SELECT != inputPort)) {
			start_port = end_port = inputPort;
		} else {
			start_port = 1; // Port 0 does not have a link
			end_port = pmnodep->numPorts;
		}
		if (end_port > pmnodep->numPorts) {
			status = FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;
			IB_LOG_WARN_FMT(__func__,  "end port(%d) is greater than the number of ports(%d)",
			end_port, pmnodep->numPorts);
			goto done;
		}
		for_some_pmports(pmnodep, pmportp, portnum, start_port, end_port, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pmimagep, portImage, groupIndex, isGroupAll, &isInternal)) {
				//Make sure the link is accounted for only once
				if (!inputLID && isInternal && (lid > pmportp->neighbor_lid)) continue;
				nbrPort = portImage->neighbor;
				if(!nbrPort) // Neighbor should never be NULL
					continue;
				pmGroupLinkInfo->NumLinks++;
			}
		}
	}

	// check if there are ports to sort
	if (!pmGroupLinkInfo->NumLinks) {
		IB_LOG_INFO_FMT(__func__, "Group %.*s has no ports", (int)sizeof(groupName), groupName);
		goto norecords;
	}

	// allocate the port list
	Status_t ret = vs_pool_alloc(&pm_pool, pmGroupLinkInfo->NumLinks* sizeof(PmLinkInfo_t), (void *)&pmGroupLinkInfo->linkInfoList);
	if (ret != VSTATUS_OK) {
		status = FINSUFFICIENT_MEMORY;
		IB_LOG_ERRORRC("Failed to allocate port list buffer for pmGroupLinkInfo rc:", ret);
		goto done;
	}

	// copy the port list
	int i = 0;
	for_some_pmnodes(pmimagep, pmnodep, lid, start_lid, end_lid) {
		if (inputLID && (PM_ALL_PORT_SELECT != inputPort)) {
			start_port = end_port = inputPort;
		} else {
			start_port = 1;
			end_port = pmnodep->numPorts;
		}
		for_some_pmports(pmnodep, pmportp, portnum, start_port, end_port, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pmimagep, portImage, groupIndex, isGroupAll, &isInternal)) {
				//Make sure the link is accounted for only once
				if (!inputLID && isInternal && (lid > pmportp->neighbor_lid)) continue;
				nbrPort = portImage->neighbor;
				if(!nbrPort) // Neighbor should never be NULL
					continue;

				pmGroupLinkInfo->linkInfoList[i].fromLid = lid;
				pmGroupLinkInfo->linkInfoList[i].toLid = pmportp->neighbor_lid;
				pmGroupLinkInfo->linkInfoList[i].fromPort = portnum;
				pmGroupLinkInfo->linkInfoList[i].toPort = pmportp->neighbor_portNum;
				pmGroupLinkInfo->linkInfoList[i].mtu = portImage->u.s.mtu;
				pmGroupLinkInfo->linkInfoList[i].activeSpeed = portImage->u.s.activeSpeed;
				pmGroupLinkInfo->linkInfoList[i].txLinkWidthDowngradeActive = portImage->u.s.txActiveWidth;
				pmGroupLinkInfo->linkInfoList[i].rxLinkWidthDowngradeActive = portImage->u.s.rxActiveWidth;
				pmGroupLinkInfo->linkInfoList[i].localStatus = pa_get_port_status(pmportp, imageIndex);
				pmGroupLinkInfo->linkInfoList[i].neighborStatus =
					(pmportp->portNum == 0 ? STL_PA_FOCUS_STATUS_OK : pa_get_port_status(nbrPort, imageIndex));

				i++;
			}
		}
	}

norecords:
	StringCopy(pmGroupLinkInfo->groupName, groupName, STL_PM_GROUPNAMELEN);
	*returnImageId = retImageId;
done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

/*************************************************************************************
*
* paGetPortStats - return port statistics
*			 Get Running totals for a Port.  This simulates a PMA get so
*			 that tools like opareport can work against the Running totals
*			 until we have a history feature.
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     lid, portNum - lid and portNum to select port to get
*     portCounterP - pointer to Consolidated Port Counters data area
*     delta - 1 for delta counters, 0 for running counters
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paGetPortStats(Pm_t *pm, STL_LID lid, uint8 portNum, PmCompositePortCounters_t *portCountersP,
	uint32 delta, uint32 userCntrs, STL_PA_IMAGE_ID_DATA imageId, uint32 *flagsp,
	STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = { 0 };
	FSTATUS status = FSUCCESS;
	PmPort_t *pmportp;
	PmPortImage_t *pmPortImageP;
	PmImage_t *pmimagep = NULL;
	uint32 imageIndex = PM_IMAGE_INDEX_INVALID;
	boolean requiresLock = TRUE;

	// check input parameters
	if (!pm || !portCountersP || !flagsp)
		return (FINVALID_PARAMETER);
	if (!lid) {
		IB_LOG_WARN_FMT(__func__,  "Illegal LID parameter: must not be zero");
		return (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (userCntrs && (delta || imageId.imageOffset)) {
		IB_LOG_WARN_FMT(__func__,  "Illegal combination of parameters: Offset (%d) and delta(%d) must be zero if UserCounters(%d) flag is set",
			imageId.imageOffset, delta, userCntrs);
		return (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	if (userCntrs) {
		STL_PA_IMAGE_ID_DATA liveImgId = { 0 };
		status = FindPmImage(__func__, pm, liveImgId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	} else {
		status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	}
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	pmportp = pm_find_port(pmimagep, lid, portNum);
	if (!pmportp) {
		IB_LOG_WARN_FMT(__func__, "Port not found: Lid 0x%x Port %u", lid, portNum);
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT;
		goto done;
	}
	pmPortImageP = &pmportp->Image[imageIndex];
	if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
		IB_LOG_WARN_FMT(__func__, "Port Query Status Not OK: %s: Lid 0x%x Port %u",
			(pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_SKIP ? "Skipped" :
			(pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_FAIL_QUERY ? "Failed Query" : "Failed Clear")),
			lid, portNum);
		if ( pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_FAIL_CLEAR) {
			*flagsp |= STL_PA_PC_FLAG_CLEAR_FAIL;
		} else {
			if ( pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_SKIP) {
				status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_DATA;
			} else if( pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_FAIL_QUERY){
				status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_BAD_DATA;
			}
			goto done;
		}
	}

	if (userCntrs) {
		(void)vs_rdlock(&pm->totalsLock);
		//*portCountersP = pmportp->StlPortCountersTotal;
		memcpy(portCountersP, &pmportp->StlPortCountersTotal, sizeof(PmCompositePortCounters_t));
		(void)vs_rwunlock(&pm->totalsLock);

		*flagsp |= STL_PA_PC_FLAG_USER_COUNTERS |
			(isUnexpectedClearUserCounters ? STL_PA_PC_FLAG_UNEXPECTED_CLEAR : 0);
		*returnImageId = (STL_PA_IMAGE_ID_DATA){0};
	} else {
		// Grab ImageTime from Pm Image
		retImageId.imageTime.absoluteTime = (uint32)pmimagep->sweepStart;

		if (delta) {
			//*portCountersP = pmPortImageP->DeltaStlPortCounters;
			memcpy(portCountersP, &pmPortImageP->DeltaStlPortCounters, sizeof(PmCompositePortCounters_t));
			*flagsp |= STL_PA_PC_FLAG_DELTA;
		} else {
			//*portCountersP = pmPortImageP->StlPortCounters;
			memcpy(portCountersP, &pmPortImageP->StlPortCounters, sizeof(PmCompositePortCounters_t));
		}
		*flagsp |= (pmPortImageP->u.s.UnexpectedClear ? STL_PA_PC_FLAG_UNEXPECTED_CLEAR : 0);
		*returnImageId = retImageId;
	}

done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

/*************************************************************************************
*
* paClearPortStats - Clear port statistics for a port
*			 Clear Running totals for a port.  This simulates a PMA clear so
*			 that tools like opareport can work against the Running totals
*			 until we have a history feature.
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     lid, portNum - lid and portNum to select port to clear
*     select - selects counters to clear
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paClearPortStats(Pm_t *pm, STL_LID lid, uint8 portNum, CounterSelectMask_t select)
{
	FSTATUS status = FSUCCESS;
	PmImage_t *pmimagep = NULL;
	STL_PA_IMAGE_ID_DATA liveImgId = {0};
	boolean requiresLock = TRUE;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);
	if (!lid) {
		IB_LOG_WARN_FMT(__func__,  "Illegal LID parameter: must not be zero");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!select.AsReg32) {
		IB_LOG_WARN_FMT(__func__, "Illegal select parameter: Must not be zero\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, liveImgId, NULL, &pmimagep, NULL, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	(void)vs_wrlock(&pm->totalsLock);
	if (portNum == PM_ALL_PORT_SELECT) {
		PmNode_t *pmnodep = pm_find_node(pmimagep, lid);
		if (! pmnodep) {
			IB_LOG_WARN_FMT(__func__, "Switch not found: LID: 0x%x", lid);
			status = FNOT_FOUND;
		} else if (pmnodep->nodeType != STL_NODE_SW) {
			IB_LOG_WARN_FMT(__func__, "Illegal port parameter: All Port Select (0xFF) can only be used on switches: LID: 0x%x", lid);
			status = FNOT_FOUND;
		} else {
			status = PmClearNodeRunningCounters(pmnodep, select);
			if (IB_EXPECT_FALSE(status != FSUCCESS)) {
				IB_LOG_WARN_FMT(__func__, "Unable to Clear Counters on LID: 0x%x", lid);
			}
		}
	} else {
		PmPort_t *pmportp = pm_find_port(pmimagep, lid, portNum);
		if (! pmportp) {
			IB_LOG_WARN_FMT(__func__, "Port not found LID 0x%x Port %u", lid, portNum);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT;
		} else {
			status = PmClearPortRunningCounters(pmportp, select);
			if (IB_EXPECT_FALSE(status != FSUCCESS)) {
				IB_LOG_WARN_FMT(__func__, "Unable to Clear Counters on LID: 0x%x  Port: %u", lid, portNum);
			}
		}
	}
	(void)vs_rwunlock(&pm->totalsLock);
done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	goto done;
}

/*************************************************************************************
*
* paClearAllPortStats - Clear port statistics for a port
*			 Clear Running totals for all Ports.  This simulates a PMA clear so
*			 that tools like opareport can work against the Running totals
*			 until we have a history feature.
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     lid, portNum - lid and portNum to select port to clear
*     select - selects counters to clear
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*
*************************************************************************************/

FSTATUS paClearAllPortStats(Pm_t *pm, CounterSelectMask_t select)
{
	STL_LID lid;
	FSTATUS status = FSUCCESS;
	PmImage_t *pmimagep = NULL;
	STL_PA_IMAGE_ID_DATA liveImgId = {0};
	boolean requiresLock = TRUE;
	PmNode_t *pmnodep;
	PmPort_t *pmportp;
	uint8 portnum;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);
	if (!select.AsReg32) {
		IB_LOG_WARN_FMT(__func__, "Illegal select parameter: Must not be zero\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, liveImgId, NULL, &pmimagep, NULL, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	(void)vs_wrlock(&pm->totalsLock);
	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, FALSE) {
			status = PmClearPortRunningCounters(pmportp, select);
			if (status != FSUCCESS)
				IB_LOG_WARN_FMT(__func__,"Failed to Clear Counters on LID: 0x%x Port: %u", lid, portnum);
		}
	}
	(void)vs_rwunlock(&pm->totalsLock);
done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	goto done;
}

/*************************************************************************************
*
* paFreezeFrameRenew - access FF to renew lease
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     imageId - 64 bit opaque imageId
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*************************************************************************************/

FSTATUS paFreezeFrameRenew(Pm_t *pm, STL_PA_IMAGE_ID_DATA *imageId)
{
	FSTATUS				status = FSUCCESS;
	uint32				imageIndex = PM_IMAGE_INDEX_INVALID;
	const char 			*msg;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	// just touching it will renew the image
	status = FindImage(pm, IMAGEID_TYPE_FREEZE_FRAME, *imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
	}

	if (record || (cimg && cimg !=  pm->ShortTermHistory.cachedComposite)) // not in the cache, never found
			IB_LOG_WARN_FMT(__func__, "Unable to access cached composite image: %s", msg);

	if (cimg && cimg == pm->ShortTermHistory.cachedComposite) {
		imageId->imageTime.absoluteTime = (uint32)cimg->sweepStart;
	} else if (imageIndex != PM_IMAGE_INDEX_INVALID) {
		imageId->imageTime.absoluteTime = (uint32)pm->Image[imageIndex].sweepStart;
	}
	(void)vs_rwunlock(&pm->stateLock);
done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
}

/*************************************************************************************
*
* paFreezeFrameRelease - release FreezeFrame
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     imageId - 64 bit opaque imageId
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*************************************************************************************/

FSTATUS paFreezeFrameRelease(Pm_t *pm, STL_PA_IMAGE_ID_DATA *imageId)
{
	FSTATUS				status = FSUCCESS;
	uint32				imageIndex;
	const char 			*msg;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	uint8				clientId = 0;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_wrlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_FREEZE_FRAME, *imageId, &imageIndex, &retImageId.imageNumber, &record, &msg, &clientId, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto unlock;
	}

	if (record || cimg) {
		if (cimg && cimg == pm->ShortTermHistory.cachedComposite) {
			imageId->imageTime.absoluteTime = (uint32)cimg->sweepStart;
			PmFreeComposite(pm->ShortTermHistory.cachedComposite);
			pm->ShortTermHistory.cachedComposite = NULL;
			status = FSUCCESS;
		} else {
			IB_LOG_WARN_FMT(__func__, "Unable to find freeze frame: %s", msg);
			status = FINVALID_PARAMETER;
		}
		goto unlock;
	} else if (!(pm->Image[imageIndex].ffRefCount & ((uint64)1<<(uint64)clientId)) ) { /*not frozen*/
		IB_LOG_ERROR_FMT(__func__, "Attempted to release freeze frame with no references");
		status = FINVALID_PARAMETER;
		goto unlock;
	}
	imageId->imageTime.absoluteTime = (uint32)pm->Image[imageIndex].sweepStart;
	pm->Image[imageIndex].ffRefCount &= ~((uint64)1<<(uint64)clientId);	// release image

unlock:
	(void)vs_rwunlock(&pm->stateLock);
done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
}

// Find a Freeze Frame Slot
// returns a valid index, on error returns pm_config.freeze_frame_images
static uint32 allocFreezeFrame(Pm_t *pm, uint32 imageIndex)
{
	uint8				freezeIndex;
	if (pm->Image[imageIndex].ffRefCount) {
		// there are other references
		// see if we can find a freezeFrame which already points to this image
		for (freezeIndex = 0; freezeIndex < pm_config.freeze_frame_images; freezeIndex ++) {
			if (pm->freezeFrames[freezeIndex] == imageIndex)
				return freezeIndex;	// points to index we want
		}
	}
	// need to find an empty Freeze Frame Slot
	for (freezeIndex = 0; freezeIndex < pm_config.freeze_frame_images; freezeIndex ++) {
		if (pm->freezeFrames[freezeIndex] == PM_IMAGE_INDEX_INVALID
			|| pm->Image[pm->freezeFrames[freezeIndex]].ffRefCount == 0) {
			// empty or stale FF image
			pm->freezeFrames[freezeIndex] = PM_IMAGE_INDEX_INVALID;
			return freezeIndex;
		}
	}
	return freezeIndex;	// >= pm_config.freeze_frame_images means failure
}

// get clientId which should be used to create a new freezeFrame for imageIndex
// a return >=64 indicates none available
static uint8 getNextClientId(Pm_t *pm, uint32 imageIndex)
{
	uint8				i;
	uint8				clientId;
	// to avoid a unfreeze/freeze against same imageIndex getting same clientId
	// we have a rolling count (eg. starting point) per Image
	for (i = 0; i < 64; i++) {
		clientId = (pm->Image[imageIndex].nextClientId+i)&63;
		if (0 == (pm->Image[imageIndex].ffRefCount & ((uint64)1 << (uint64)clientId))) {
			pm->Image[imageIndex].ffRefCount |= ((uint64)1 << (uint64)clientId);
			return clientId;
		}
	}
	return 255;	// none available
}

/*************************************************************************************
*
* paFreezeFrameCreate - create FreezeFrame
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     imageId - 64 bit opaque imageId
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*
*************************************************************************************/

FSTATUS paFreezeFrameCreate(Pm_t *pm, STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *retImageId)
{
	FSTATUS				status = FSUCCESS;
	uint32				imageIndex;
	const char 			*msg;
	uint8				clientId;
	uint8				freezeIndex;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}
	(void)vs_wrlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &retImageId->imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}
	if (record || cimg) {
		retImageId->imageNumber = imageId.imageNumber;
		if (record) { // cache the disk image
			status = PmFreezeComposite(pm, record);
			if (status != FSUCCESS) {
				IB_LOG_WARN_FMT(__func__, "Unable to freeze composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
			retImageId->imageNumber = record->header.imageIDs[0];
		} else if (cimg && pm->ShortTermHistory.cachedComposite) { // already frozen
			retImageId->imageNumber = pm->ShortTermHistory.cachedComposite->header.common.imageIDs[0];
		} else { // trying to freeze the current composite (unlikely but possible)
			status = PmFreezeCurrent(pm);
			if (status != FSUCCESS) {
				IB_LOG_WARN_FMT(__func__, "Unable to freeze current composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
		retImageId->imageTime.absoluteTime = (uint32)pm->ShortTermHistory.cachedComposite->sweepStart;
		(void)vs_rwunlock(&pm->stateLock);
		goto done;
	}

	// Find a Freeze Frame Slot
	freezeIndex = allocFreezeFrame(pm, imageIndex);
	if (freezeIndex >= pm_config.freeze_frame_images) {
		IB_LOG_WARN0( "Out of Freeze Frame Images");
		status = FINSUFFICIENT_MEMORY | STL_MAD_STATUS_STL_PA_NO_IMAGE;
		goto error;
	}
	clientId = getNextClientId(pm, imageIndex);
	if (clientId >= 64) {
		IB_LOG_WARN0( "Too many freezes of 1 image");
		status = FINSUFFICIENT_MEMORY | STL_MAD_STATUS_STL_PA_NO_IMAGE;
		goto error;
	}
	pm->Image[imageIndex].nextClientId = (clientId+1) & 63;
	pm->freezeFrames[freezeIndex] = imageIndex;
	retImageId->imageNumber = BuildFreezeFrameImageId(pm, freezeIndex, clientId, &retImageId->imageTime.absoluteTime);
	(void)vs_rwunlock(&pm->stateLock);

done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	retImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

/*************************************************************************************
*
* paFreezeFrameMove - atomically release and create a  FreezeFrame
*
*  Inputs:
*     pm - pointer to Pm_t (the PM main data type)
*     ffImageId - imageId containing information for FF to release (number)
*     imageId - imageId containing information for image to FreezeFrame (number, offset)
*
*  Return:
*     FSTATUS - FSUCCESS if OK
*     on error the ffImageId is not released
*
*************************************************************************************/

FSTATUS paFreezeFrameMove(Pm_t *pm, STL_PA_IMAGE_ID_DATA ffImageId, STL_PA_IMAGE_ID_DATA imageId,
	STL_PA_IMAGE_ID_DATA *returnImageId)
{
	FSTATUS				status = FSUCCESS;
	uint32				ffImageIndex = 0;
	uint8				ffClientId = 0;
	uint32				imageIndex = 0;
	uint8				clientId = 0;
	const char 			*msg = NULL;
	uint8				freezeIndex = 0;
	boolean				oldIsComp = 0, newIsComp = 0;
	PmHistoryRecord_t	*record = NULL;
	PmCompositeImage_t	*cimg = NULL;

	// check input parameters
	if (!pm)
		return(FINVALID_PARAMETER);

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_wrlock(&pm->stateLock);
	status = FindImage(pm, IMAGEID_TYPE_FREEZE_FRAME, ffImageId, &ffImageIndex, &returnImageId->imageNumber, &record, &msg, &ffClientId, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get freeze frame index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}
	if (cimg && cimg == pm->ShortTermHistory.cachedComposite) {
		oldIsComp = 1;
	} else if (cimg && cimg != pm->ShortTermHistory.cachedComposite) {
		IB_LOG_WARN_FMT(__func__, "Unable to find freeze frame for ImageID: %s", msg);
		status = FINVALID_PARAMETER;
		goto error;
	}

	record = NULL;
	cimg = NULL;
	status = FindImage(pm, IMAGEID_TYPE_ANY, imageId, &imageIndex, &returnImageId->imageNumber, &record, &msg, NULL, &cimg);
	if (FSUCCESS != status) {
		IB_LOG_WARN_FMT(__func__, "Unable to get index from ImageId: %s: %s", FSTATUS_ToString(status), msg);
		goto error;
	}
	if (record || cimg) {
		newIsComp = 1;
		if (record) { //composite isn't already cached
			status = PmFreezeComposite(pm, record);
			if (status != FSUCCESS) {
				IB_LOG_WARN_FMT(__func__, "Unable to freeze composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		} else {
			status = PmFreezeCurrent(pm);
			if (status != FSUCCESS) {
				IB_LOG_WARN_FMT(__func__, "Unable to freeze current composite image: %s", FSTATUS_ToString(status));
				goto error;
			}
		}
	}

	if (!newIsComp) {
		if (!oldIsComp && pm->Image[imageIndex].ffRefCount == ((uint64)1 << (uint64)ffClientId)) {
			// we are last/only client using this image
			// so we can simply use the same Freeze Frame Slot as old freeze
			ImageId_t id;
			id.AsReg64 = ffImageId.imageNumber;
			freezeIndex = id.s.index;
		} else {
			// Find a empty Freeze Frame Slot
			freezeIndex = allocFreezeFrame(pm, imageIndex);
		}
		clientId = getNextClientId(pm, imageIndex);
		if (clientId >= 64) {
			IB_LOG_WARN0( "Too many freezes of 1 image");
			status = FINSUFFICIENT_MEMORY | STL_MAD_STATUS_STL_PA_NO_IMAGE;
			goto error;
		}
		// freeze the selected image
		pm->Image[imageIndex].nextClientId = (clientId+1) & 63;
		pm->freezeFrames[freezeIndex] = imageIndex;

		returnImageId->imageNumber = BuildFreezeFrameImageId(pm, freezeIndex, clientId,
			&returnImageId->imageTime.absoluteTime);
	} else {
		returnImageId->imageNumber = pm->ShortTermHistory.cachedComposite->header.common.imageIDs[0];
		returnImageId->imageTime.absoluteTime = (uint32)pm->ShortTermHistory.cachedComposite->sweepStart;
	}
	if (!oldIsComp) pm->Image[ffImageIndex].ffRefCount &= ~((uint64)1 << (uint64)ffClientId); // release old image
	(void)vs_rwunlock(&pm->stateLock);
done:
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;

}

typedef uint32 (*ComputeFunc_t)(Pm_t *pm, uint32 imageIndex, PmPort_t *port, void *data);
typedef uint32 (*CompareFunc_t)(uint64 value1, uint64 value2);

uint32 compareGE(uint64 value1, uint64 value2) { return(value1 >= value2); }
uint32 compareLE(uint64 value1, uint64 value2) { return(value1 <= value2); }
uint32 compareGT(uint64 value1, uint64 value2) { return(value1 > value2); }
uint32 compareLT(uint64 value1, uint64 value2) { return(value1 < value2); }

void focusAddArrayItem(Pm_t *pm, uint32 imageIndex, focusArrayItem_t *focusArray, int index, PmPort_t *pmportp,
	ComputeFunc_t computeFunc, void *computeData)
{
	focusArrayItem_t *currItem = &focusArray[index];
	PmPort_t *neighborPort = pmportp->Image[imageIndex].neighbor;

	currItem->localValue = computeFunc(pm, imageIndex, pmportp, computeData);
	currItem->localPort = pmportp;
	currItem->localLID = pmportp->pmnodep->Image[imageIndex].lid;
	if (neighborPort) {
		currItem->neighborValue = computeFunc(pm, imageIndex, neighborPort, computeData);
		currItem->neighborPort = neighborPort;
		currItem->neighborLID = neighborPort->pmnodep->Image[imageIndex].lid;
	}
}

/* return -1 if a goes before b
 */
int compareFocusArrayItemsGT(const void *A, const void *B)
{
	focusArrayItem_t *a = (focusArrayItem_t *)A;
	focusArrayItem_t *b = (focusArrayItem_t *)B;

	uint64 a_value = MAX(a->localValue, a->neighborValue);
	uint64 b_value = MAX(b->localValue, b->neighborValue);

	if (a_value > b_value) return -1;
	if (a_value < b_value) return 1;

	if (a->localLID < b->localLID) return -1; // Lower Lid should be first to maintain order
	if (a->localLID > b->localLID) return 1;

	return 0;
}
int compareFocusArrayItemsLT(const void *A, const void *B)
{
	focusArrayItem_t *a = (focusArrayItem_t *)A;
	focusArrayItem_t *b = (focusArrayItem_t *)B;

	uint64 a_value = MIN(a->localValue, a->neighborValue);
	uint64 b_value = MIN(b->localValue, b->neighborValue);

	if (a_value > b_value) return 1;
	if (a_value < b_value) return -1;

	if (a->localLID < b->localLID) return -1; // Lower Lid should be first to maintain order
	if (a->localLID > b->localLID) return 1;

	return 0;
}
typedef int (*QsortCompareFunc_t)(const void *A, const void *B);

FSTATUS focusGetItems(PmFocusPorts_t *pmFocusPorts, focusArrayItem_t *focusArray, uint32 start, uint32 imageIndex)
{
	FSTATUS status;
	focusArrayItem_t *currItem = &focusArray[start];
	uint32 i;

	status = vs_pool_alloc(&pm_pool, pmFocusPorts->NumPorts * sizeof(PmFocusPortEntry_t),
		(void*)&pmFocusPorts->portList);
	if (status != FSUCCESS) {
		IB_LOG_ERRORRC("Failed to allocate sorted port list buffer for pmFocusPorts rc:", status);
		return FINSUFFICIENT_MEMORY;
	}
	memset(pmFocusPorts->portList, 0, pmFocusPorts->NumPorts * sizeof(PmFocusPortEntry_t));

	for (i = 0; i < pmFocusPorts->NumPorts; i++, currItem++) {
		pmFocusPorts->portList[i].lid = currItem->localPort->pmnodep->Image[imageIndex].lid;
		pmFocusPorts->portList[i].portNum = currItem->localPort->portNum;

		pmFocusPorts->portList[i].rate = PmCalculateRate(currItem->localPort->Image[imageIndex].u.s.activeSpeed,
			currItem->localPort->Image[imageIndex].u.s.rxActiveWidth);
		pmFocusPorts->portList[i].maxVlMtu = currItem->localPort->Image[imageIndex].u.s.mtu;

		pmFocusPorts->portList[i].localStatus = pa_get_port_status(currItem->localPort, imageIndex);

		pmFocusPorts->portList[i].value[0] = currItem->localValue;

		pmFocusPorts->portList[i].guid = currItem->localPort->pmnodep->NodeGUID;
		StringCopy(pmFocusPorts->portList[i].nodeDesc, (char *)currItem->localPort->pmnodep->nodeDesc.NodeString,
			STL_NODE_DESCRIPTION_ARRAY_SIZE);

		/* Neighbor */
		pmFocusPorts->portList[i].neighborStatus = (currItem->localPort->portNum == 0 ?
			STL_PA_FOCUS_STATUS_OK : pa_get_port_status(currItem->neighborPort, imageIndex));
		if (!currItem->neighborPort) {
			continue;
		}

		pmFocusPorts->portList[i].neighborLid = currItem->neighborPort->pmnodep->Image[imageIndex].lid;
		pmFocusPorts->portList[i].neighborPortNum = currItem->neighborPort->portNum;

		pmFocusPorts->portList[i].neighborValue[0] = currItem->neighborValue;

		pmFocusPorts->portList[i].neighborGuid = currItem->neighborPort->pmnodep->NodeGUID;
		StringCopy(pmFocusPorts->portList[i].neighborNodeDesc, (char *)currItem->neighborPort->pmnodep->nodeDesc.NodeString,
			STL_NODE_DESCRIPTION_ARRAY_SIZE);
	}
	return FSUCCESS;
}

/*************************************************************************************
 *
 * paGetFocusPorts - return a set of focus ports
 *
 *  Inputs:
 *     pm - pointer to Pm_t (the PM main data type)
 *     groupName - pointer to name of group
 *     pmFocusPorts - pointer to caller-declared data area to return focus port info
 *     imageId - Id of the image
 *     returnImageId - pointer Image Id that is used
 *     select  - focus select that is used
 *     start - the start index
 *     range - number of records requested
 *
 *  Return:
 *     FSTATUS - FSUCCESS if OK
 *
 *
 **************************************************************************************/
FSTATUS paGetFocusPorts(Pm_t *pm, char *groupName, PmFocusPorts_t *pmFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
	uint32 start, uint32 range)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	STL_LID lid;
	uint32 imageIndex, imageInterval;
	PmImage_t *pmimagep = NULL;
	PmNode_t *pmnodep = NULL;
	PmPort_t *pmportp = NULL;
	PmPortImage_t *portImage = NULL;
	FSTATUS status = FSUCCESS;
	Status_t allocStatus;
	ComputeFunc_t computeFunc = NULL;
	QsortCompareFunc_t compareFunc = NULL;
	void *computeData = NULL;
	boolean requiresLock = TRUE, isGroupAll = FALSE, isInternal = FALSE;
	uint32 allocatedItems = 0;
	focusArrayItem_t *focusArray = NULL;
	int items = 0, groupIndex = -1;
	uint8 portnum;

	// check input parameters
	if (!pm || !groupName || !pmFocusPorts)
		return (FINVALID_PARAMETER);
	if (groupName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal groupName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER) ;
	}
	if (!range) {
		IB_LOG_WARN_FMT(__func__, "Illegal range parameter: %d: must be greater than zero\n", range);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	switch (select) {
	case STL_PA_SELECT_UTIL_HIGH:
		computeFunc = &computeUtilizationPct10;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_UTIL_PKTS_HIGH:
		computeFunc = &computeSendKPkts;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_UTIL_LOW:
		computeFunc = &computeUtilizationPct10;
		compareFunc = &compareFocusArrayItemsLT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_CATEGORY_INTEG:
		computeFunc = &computeIntegrity;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = &pm->integrityWeights;
		break;
	case STL_PA_SELECT_CATEGORY_CONG:
		computeFunc = &computeCongestion;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = &pm->congestionWeights;
		break;
	case STL_PA_SELECT_CATEGORY_SMA_CONG:
		computeFunc = &computeSmaCongestion;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = &pm->congestionWeights;
		break;
	case STL_PA_SELECT_CATEGORY_BUBBLE:
		computeFunc = &computeBubble;
		compareFunc = &compareFocusArrayItemsGT;
		break;
	case STL_PA_SELECT_CATEGORY_SEC:
		computeFunc = &computeSecurity;
		compareFunc = &compareFocusArrayItemsGT;
		break;
	case STL_PA_SELECT_CATEGORY_ROUT:
		computeFunc = &computeRouting;
		compareFunc = &compareFocusArrayItemsGT;
		break;
	default:
		IB_LOG_WARN_FMT(__func__, "Illegal select parameter: 0x%x", select);
		return FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;
		break;
	}

	// initialize group config port list counts
	pmFocusPorts->NumPorts = 0;
	pmFocusPorts->portList = NULL;

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	status = LocateGroup(pmimagep, groupName, &groupIndex);
	if (status != FSUCCESS) {
		IB_LOG_WARN_FMT(__func__, "Group %.*s not Found: %s", (int)sizeof(groupName), groupName, FSTATUS_ToString(status));
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
		goto done;
	}
	// If Success, but Index is still -1, then it is All Group
	if (groupIndex == -1) isGroupAll = TRUE;

	imageInterval = pmimagep->imageInterval;

	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pmimagep, portImage, groupIndex, isGroupAll, &isInternal)) {
				/* If link is internal and lid is greater than neighbor's, then this link is already in the list */
				if (isInternal && portnum && (lid > pmportp->neighbor_lid)) continue;
				allocatedItems++;
			}
		}
	}
	if (allocatedItems == 0 || start >= allocatedItems) {
		goto norecords;
	}

	allocStatus = vs_pool_alloc(&pm_pool, allocatedItems * sizeof(focusArrayItem_t), (void*)&focusArray);
	if (allocStatus != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate list buffer for pmFocusPorts rc:", allocStatus);
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}
	memset(focusArray, 0, allocatedItems * sizeof(focusArrayItem_t));

	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pmimagep, portImage, groupIndex, isGroupAll, &isInternal)) {
				/* If link is internal and lid is greater than neighbor's, then this link is already in the list */
				if (isInternal && portnum && (lid > pmportp->neighbor_lid)) continue;
				focusAddArrayItem(pm, imageIndex, focusArray, items, pmportp, computeFunc, computeData);
				items++;
			}
		}
	}
	if (items != allocatedItems) {
		IB_LOG_ERROR_FMT(__func__, "Number of Processed Items (%d) does no match array size %d\n", items, allocatedItems);
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}

	/* Trim starting items */
	items = items - start;

	/* Sort Array Items */
	if (items > 1) {
		qsort(focusArray, allocatedItems, sizeof(focusArrayItem_t), compareFunc);
	}
	/* Trim ending items until within range */
	items = MIN(items, range);

	StringCopy(pmFocusPorts->name, groupName, STL_PM_GROUPNAMELEN);
	pmFocusPorts->NumPorts = items;
	status = focusGetItems(pmFocusPorts, focusArray, start, imageIndex);

norecords:
	if (status == FSUCCESS)
		*returnImageId = retImageId;
done:
	if (focusArray) {
		(void)vs_pool_free(&pm_pool, focusArray);
	}

	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

FSTATUS processMultiFocusPort(Pm_t *pm, PmImage_t *pmimagep, PmPort_t *pmportp, uint32 imageIndex,
	STL_FOCUS_PORT_TUPLE *tple, uint8 oper, PmFocusPorts_t *pmFocusPorts, uint32 start, boolean isInternal)
{
	int idx;
	ComputeFunc_t computeFunc = NULL;
	CompareFunc_t compareFunc = NULL;
	void *computeData = NULL;
	PmPort_t *neighborport = pmportp->Image[imageIndex].neighbor;
	uint32 local_inc = 0, neigh_inc = 0;
	uint64 localValues[MAX_NUM_FOCUS_PORT_TUPLES] = {0};
	uint64 neighborValues[MAX_NUM_FOCUS_PORT_TUPLES] = {0};

	for (idx = 0; idx < MAX_NUM_FOCUS_PORT_TUPLES; idx++) {
		uint32 local_cmp, neigh_cmp = 0;
		computeFunc = NULL;
		compareFunc = NULL;
		computeData = NULL;
		switch (tple[idx].select) {
		case STL_PA_SELECT_UTIL_HIGH:
			computeFunc = &computeUtilizationPct10;
			computeData = (void *)&pmimagep->imageInterval;
			break;
		case STL_PA_SELECT_UTIL_PKTS_HIGH:
			computeFunc = &computeSendKPkts;
			computeData = (void *)&pmimagep->imageInterval;
			break;
		case STL_PA_SELECT_CATEGORY_INTEG:
			computeFunc = &computeIntegrity;
			computeData = (void *)&pm->integrityWeights;
			break;
		case STL_PA_SELECT_CATEGORY_CONG:
			computeFunc = &computeCongestion;
			computeData = (void *)&pm->congestionWeights;
			break;
		case STL_PA_SELECT_CATEGORY_SMA_CONG:
			computeFunc = &computeSmaCongestion;
			computeData = (void *)&pm->congestionWeights;
			break;
		case STL_PA_SELECT_CATEGORY_BUBBLE:
			computeFunc = &computeBubble;
			break;
		case STL_PA_SELECT_CATEGORY_SEC:
			computeFunc = &computeSecurity;
			break;
		case STL_PA_SELECT_CATEGORY_ROUT:
			computeFunc = &computeRouting;
			break;
		default:
			/* if select is 0 and this is first idx this
			   is an error otherwise we are done */
			if (idx == 0) {
				return (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
			}
			break;
		}
		if (computeFunc == NULL) break;

		switch (tple[idx].comparator) {
		case FOCUS_PORTS_COMPARATOR_GREATER_THAN_OR_EQUAL:
			compareFunc = &compareGE;
			break;
		case FOCUS_PORTS_COMPARATOR_LESS_THAN_OR_EQUAL:
			compareFunc = &compareLE;
			break;
		case FOCUS_PORTS_COMPARATOR_GREATER_THAN:
			compareFunc = &compareGT;
			break;
		case FOCUS_PORTS_COMPARATOR_LESS_THAN:
			compareFunc = &compareLT;
			break;
		default:
			return (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
		}

		localValues[idx] = computeFunc(pm, imageIndex, pmportp, computeData);
		local_cmp = compareFunc(localValues[idx], tple[idx].argument);

		if (neighborport) {
			neighborValues[idx] = computeFunc(pm, imageIndex, neighborport, computeData);
			neigh_cmp = compareFunc(neighborValues[idx], tple[idx].argument);
		}

		switch (oper) {
		case FOCUS_PORTS_LOGICAL_OPERATOR_AND:
			/* Initialize include booleans to TRUE on first loop if...
			 * - At least the first tuple is valid
			 * - The Logical operator is 'AND'
			 */
			if (idx == 0) {
				local_inc = 1;
				neigh_inc = 1;
			}
			local_inc &= local_cmp;
			neigh_inc &= neigh_cmp;
			if (local_inc == 0 && neigh_inc == 0) {
				return FNOT_DONE;
			}
			break;
		case FOCUS_PORTS_LOGICAL_OPERATOR_INVALID:
			if (idx != 0) {
				/* If no operator is set for multi-tuple queries, then fail */
				return (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
			}
			/* If only one tuple, default to OR: fallthrough */
		case FOCUS_PORTS_LOGICAL_OPERATOR_OR:
			local_inc |= local_cmp;
			neigh_inc |= neigh_cmp;
			break;
		default:
			return (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
		}
	}

	// at least one side of the link will be added
	if (local_inc || (isInternal && neigh_inc)) {
		if (pmFocusPorts->portCntr >= (pmFocusPorts->NumPorts + start)) return FCOMPLETED; // end of start + range

		if (pmFocusPorts->portCntr < start) return FSUCCESS; // skip until start

		PmFocusPortEntry_t *entry = &pmFocusPorts->portList[pmFocusPorts->portCntr-start];

		entry->lid = pmportp->pmnodep->Image[imageIndex].lid;
		entry->portNum = pmportp->portNum;
		entry->rate = PmCalculateRate(pmportp->Image[imageIndex].u.s.activeSpeed, pmportp->Image[imageIndex].u.s.rxActiveWidth);
		entry->maxVlMtu = pmportp->Image[imageIndex].u.s.mtu;
		entry->guid = (uint64)pmportp->pmnodep->NodeGUID;
		StringCopy(entry->nodeDesc, (char *)pmportp->pmnodep->nodeDesc.NodeString, STL_NODE_DESCRIPTION_ARRAY_SIZE);

		entry->localStatus = pa_get_port_status(pmportp, imageIndex);

		memcpy((void *)&entry->value[0], (void *)&localValues[0], sizeof(uint64)*MAX_NUM_FOCUS_PORT_TUPLES);

		/* Neighbor */
		entry->neighborStatus = (pmportp->portNum == 0 ? STL_PA_FOCUS_STATUS_OK : pa_get_port_status(neighborport, imageIndex));
		if (!neighborport) {
			return FSUCCESS;
		}

		entry->neighborLid = neighborport->pmnodep->Image[imageIndex].lid;
		entry->neighborPortNum = neighborport->portNum;

		memcpy((void *)&entry->neighborValue[0], (void *)&neighborValues[0], sizeof(uint64)*MAX_NUM_FOCUS_PORT_TUPLES);

		entry->neighborGuid = (uint64)neighborport->pmnodep->NodeGUID;
		StringCopy(entry->neighborNodeDesc, (char *)neighborport->pmnodep->nodeDesc.NodeString, STL_NODE_DESCRIPTION_ARRAY_SIZE);
		return FSUCCESS;
	}
	return FNOT_DONE;
}
FSTATUS paGetMultiFocusPorts(Pm_t *pm, char *groupName, PmFocusPorts_t *pmFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 start,
	uint32 range, STL_FOCUS_PORT_TUPLE *tple, uint8 oper)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	STL_LID lid;
	uint32 imageIndex;
	PmImage_t *pmimagep = NULL;
	PmNode_t *pmnodep = NULL;
	PmPort_t *pmportp = NULL;
	uint8 portnum;
	PmPortImage_t *portImage = NULL;
	FSTATUS status = FSUCCESS;
	Status_t allocStatus;
	boolean requiresLock = TRUE, isGroupAll = FALSE, isInternal = FALSE;
	uint32 allocatedItems = 0;
	int groupIndex = -1;

	// check input parameters
	if (!pm || !groupName || !pmFocusPorts)
		return (FINVALID_PARAMETER);
	if (groupName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal groupName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER) ;
	}
	if (!range) {
		IB_LOG_WARN_FMT(__func__, "Illegal range parameter: %d: must be greater than zero\n", range);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	} else if (range > pm_config.subnet_size) {
		IB_LOG_WARN_FMT(__func__, "Illegal range parameter: Exceeds maxmimum subnet size of %d\n", pm_config.subnet_size);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	// initialize group config port list counts
	pmFocusPorts->NumPorts = 0;
	pmFocusPorts->portList = NULL;

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	status = LocateGroup(pmimagep, groupName, &groupIndex);
	if (status != FSUCCESS) {
		IB_LOG_WARN_FMT(__func__, "Group %.*s not Found: %s", (int)sizeof(groupName), groupName, FSTATUS_ToString(status));
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
		goto done;
	}
	// If Success, but Index is still -1, then it is All Group
	if (groupIndex == -1) isGroupAll = TRUE;

	allocatedItems = range;
	allocStatus = vs_pool_alloc(&pm_pool, allocatedItems * sizeof(PmFocusPortEntry_t), (void *)&pmFocusPorts->portList);
	if (allocStatus != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate list buffer for pmFocusPorts rc:", allocStatus);
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}
	memset(pmFocusPorts->portList, 0, allocatedItems * sizeof(PmFocusPortEntry_t));
	pmFocusPorts->NumPorts = allocatedItems;
	pmFocusPorts->portCntr = 0;
	StringCopy(pmFocusPorts->name, groupName, STL_PM_GROUPNAMELEN);

	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pmimagep, portImage, groupIndex, isGroupAll, &isInternal)) {
				/* If link is internal and lid is greater than neighbor's, then this link is already in the list */
				if (isInternal && (lid > pmportp->neighbor_lid)) continue;
				status = processMultiFocusPort(pm, pmimagep, pmportp, imageIndex,
					tple, oper, pmFocusPorts, start, isInternal);
				if (status == FSUCCESS) pmFocusPorts->portCntr++;
				if ((status & 0xFF) == FINVALID_PARAMETER) goto done;
				if (status == FCOMPLETED) goto complete;
			}
		}
	}
complete:
	status = FSUCCESS;
	/* Shrink portCntr by start */
	if (pmFocusPorts->portCntr > start) {
		pmFocusPorts->portCntr -= start;
	} else {
		pmFocusPorts->portCntr = 0;
	}
	/* Shrink NumPorts (range) if portCntr is less than range */
	pmFocusPorts->NumPorts = MIN(pmFocusPorts->portCntr, pmFocusPorts->NumPorts);
done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

// Add to the list only once. Add only if localStatus is set or when neighborStatus is set and local lid is lesser than neighbor lid.
#define CHECK_UNEXPECTED_CLEAR_STATUS() \
	((STL_PA_SELECT_UNEXP_CLR_PORT == select) && \
		(((STL_PA_FOCUS_STATUS_UNEXPECTED_CLEAR == localStatus ) && (STL_PA_FOCUS_STATUS_UNEXPECTED_CLEAR != neighborStatus)) || \
		((STL_PA_FOCUS_STATUS_UNEXPECTED_CLEAR == localStatus) && (STL_PA_FOCUS_STATUS_UNEXPECTED_CLEAR == neighborStatus) && \
		(lid < nbrPt->pmnodep->Image[imageIndex].lid))))

// Add to the list only once. Add only if localStatus is set or when neighborStatus is set and local lid is lesser than neighbor lid.
#define CHECK_PMA_NRSP_STATUS() \
	((STL_PA_SELECT_NO_RESP_PORT == select) && \
		(((STL_PA_FOCUS_STATUS_PMA_FAILURE == localStatus) && (STL_PA_FOCUS_STATUS_PMA_FAILURE != neighborStatus)) || \
		((STL_PA_FOCUS_STATUS_PMA_FAILURE == localStatus) && (STL_PA_FOCUS_STATUS_PMA_FAILURE == neighborStatus) && \
		(lid < nbrPt->pmnodep->Image[imageIndex].lid))))

// Add to the list only once. Add only if localStatus is set or when neighborStatus is set and local lid is lesser than neighbor lid.
#define CHECK_SKIPPED_STATUS() \
	((STL_PA_SELECT_SKIPPED_PORT == select) && \
		(((STL_PA_FOCUS_STATUS_PMA_IGNORE == localStatus) && (STL_PA_FOCUS_STATUS_PMA_IGNORE != neighborStatus)) || \
		((STL_PA_FOCUS_STATUS_PMA_IGNORE == localStatus) && (STL_PA_FOCUS_STATUS_PMA_IGNORE == neighborStatus) && \
		(lid < nbrPt->pmnodep->Image[imageIndex].lid))))


FSTATUS processExtFocusPort(uint32 imageIndex, PmPort_t *pmportp, uint32 select, PmFocusPorts_t *pmFocusPorts, STL_LID lid)
{
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];
	uint8 localStatus, neighborStatus;
	PmPort_t *nbrPt = portImage->neighbor;

	localStatus = pa_get_port_status(pmportp, imageIndex);
	neighborStatus = (pmportp->portNum == 0 ?
		STL_PA_FOCUS_STATUS_OK : pa_get_port_status(nbrPt, imageIndex));

	if((CHECK_UNEXPECTED_CLEAR_STATUS()) || (CHECK_SKIPPED_STATUS()) || (CHECK_PMA_NRSP_STATUS())){
		pmFocusPorts->portList[pmFocusPorts->portCntr].lid = lid;
		pmFocusPorts->portList[pmFocusPorts->portCntr].portNum = pmportp->portNum;
		pmFocusPorts->portList[pmFocusPorts->portCntr].localStatus = localStatus;
		pmFocusPorts->portList[pmFocusPorts->portCntr].rate =
			PmCalculateRate(pmportp->Image[imageIndex].u.s.activeSpeed, pmportp->Image[imageIndex].u.s.rxActiveWidth);
		pmFocusPorts->portList[pmFocusPorts->portCntr].maxVlMtu = pmportp->Image[imageIndex].u.s.mtu;
		pmFocusPorts->portList[pmFocusPorts->portCntr].guid = (uint64)(pmportp->pmnodep->NodeGUID);
		StringCopy(pmFocusPorts->portList[pmFocusPorts->portCntr].nodeDesc,
			(char *)pmportp->pmnodep->nodeDesc.NodeString,
			sizeof(pmFocusPorts->portList[pmFocusPorts->portCntr].nodeDesc));
		if (pmportp->portNum != 0 && nbrPt != NULL) {
			pmFocusPorts->portList[pmFocusPorts->portCntr].neighborStatus = neighborStatus;
			pmFocusPorts->portList[pmFocusPorts->portCntr].neighborLid = nbrPt->pmnodep->Image[imageIndex].lid;
			pmFocusPorts->portList[pmFocusPorts->portCntr].neighborPortNum = nbrPt->portNum;
			pmFocusPorts->portList[pmFocusPorts->portCntr].neighborGuid = (uint64)(nbrPt->pmnodep->NodeGUID);
			StringCopy(pmFocusPorts->portList[pmFocusPorts->portCntr].neighborNodeDesc,
				(char *)nbrPt->pmnodep->nodeDesc.NodeString,
				sizeof(pmFocusPorts->portList[pmFocusPorts->portCntr].neighborNodeDesc));
		} else {
			pmFocusPorts->portList[pmFocusPorts->portCntr].neighborStatus = neighborStatus;
			pmFocusPorts->portList[pmFocusPorts->portCntr].neighborLid = 0;
			pmFocusPorts->portList[pmFocusPorts->portCntr].neighborPortNum = 0;

			pmFocusPorts->portList[pmFocusPorts->portCntr].neighborGuid = 0;
			pmFocusPorts->portList[pmFocusPorts->portCntr].neighborNodeDesc[0] = 0;
		}
		pmFocusPorts->portCntr++;
	}

	return(FSUCCESS);
}

/*************************************************************************************
 *
 * paGetExtFocusPorts - return a set of extended focus ports
 *
 * Inputs:
 *     pm - pointer to Pm_t (the PM main data type)
 *     groupName - pointer to name of group
 *     pmFocusPorts - pointer to caller-declared data area to return focus port info
 *     imageId - Id of the image
 *     returnImageId - pointer Image Id that is used
 *     select  - focus select that is used
 *     start - the start index
 *     range - number of records requested
 *
 *  Return:
 *     FSTATUS - FSUCCESS if OK
 *
 *
 **************************************************************************************/

FSTATUS paGetExtFocusPorts(Pm_t *pm, char *groupName, PmFocusPorts_t *pmFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
	uint32 start, uint32 range)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	PmNode_t *pmnodep = NULL;
	PmPort_t *pmportp = NULL;
	uint8 portnum;
	PmPortImage_t *portImage = NULL;
	STL_LID lid;
	uint32 imageIndex;
	PmImage_t *pmimagep = NULL;
	FSTATUS status = FSUCCESS;
	boolean requiresLock = TRUE, isGroupAll = FALSE;
	uint32 allocatedItems = 0;
	int i, groupIndex = -1;

	// check input parameters
	if (!pm || !groupName || !pmFocusPorts)
		return (FINVALID_PARAMETER);
	if (groupName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal groupName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!range) {
		IB_LOG_WARN_FMT(__func__, "Illegal range parameter: %d: must be greater than zero\n", range);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	} else if (range > pm_config.subnet_size) {
		IB_LOG_WARN_FMT(__func__, "Illegal range parameter: Exceeds maxmimum subnet size of %d\n", pm_config.subnet_size);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	// initialize group config port list counts
	pmFocusPorts->NumPorts = 0;
	pmFocusPorts->portList = NULL;

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	status = LocateGroup(pmimagep, groupName, &groupIndex);
	if (status != FSUCCESS) {
		IB_LOG_WARN_FMT(__func__, "Group %.*s not Found: %s", (int)sizeof(groupName), groupName, FSTATUS_ToString(status));
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP;
		goto done;
	}
	// If Success, but Index is still -1, then it is All Group
	if (groupIndex == -1) isGroupAll = TRUE;

	if(select == STL_PA_SELECT_UNEXP_CLR_PORT){
		allocatedItems = pmimagep->UnexpectedClearPorts;
	} else if( select == STL_PA_SELECT_NO_RESP_PORT) {
		allocatedItems = pmimagep->NoRespNodes;
	} else {
		allocatedItems = pmimagep->SkippedNodes;
	}

	if(allocatedItems == 0) {
		IB_LOG_WARN_FMT(__func__, "No Matching Records\n");
		status = FSUCCESS | MAD_STATUS_SA_NO_RECORDS;
		goto done;
	}

	status = vs_pool_alloc(&pm_pool, allocatedItems * sizeof(PmFocusPortEntry_t), (void*)&pmFocusPorts->portList);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate list buffer for pmExtFocusPorts rc:", status);
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}

	pmFocusPorts->NumPorts = allocatedItems;
	pmFocusPorts->portCntr = 0;
	StringCopy(pmFocusPorts->name, groupName, STL_PM_GROUPNAMELEN);

	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInGroup(pmimagep, portImage, groupIndex, isGroupAll, NULL)) {
				processExtFocusPort(imageIndex, pmportp, select, pmFocusPorts, lid);
			}
		}
	}

	/* trim list */
	if (start >= pmFocusPorts->NumPorts) {
		IB_LOG_WARN_FMT(__func__, "Illegal start parameter: Exceeds range of available entries %d\n", pmFocusPorts->NumPorts);
		status = (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
		goto done;
	}

	pmFocusPorts->NumPorts = pmFocusPorts->NumPorts - start;

	if (start > 0) {
		/* shift list start point to start index */
		for (i = 0; i < pmFocusPorts->NumPorts; i++)
			pmFocusPorts->portList[i] = pmFocusPorts->portList[start+i];
	}

	/* truncate list */
	if (range < pmFocusPorts->NumPorts) {
		pmFocusPorts->NumPorts = range;
	}

	if (status == FSUCCESS)
		*returnImageId = retImageId;
done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}


FSTATUS paGetImageInfo(Pm_t *pm, STL_PA_IMAGE_ID_DATA imageId, PmImageInfo_t *imageInfo,
	STL_PA_IMAGE_ID_DATA *returnImageId)
{
	FSTATUS status = FSUCCESS;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	PmImage_t *pmimagep = NULL;
	boolean requiresLock = TRUE;
	int i;

	// check input parameters
	if (!pm || !imageInfo)
		return FINVALID_PARAMETER;

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, NULL, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	imageInfo->sweepStart				= pmimagep->sweepStart;
	imageInfo->sweepDuration			= pmimagep->sweepDuration;
	imageInfo->numHFIPorts	   			= pmimagep->HFIPorts;
	imageInfo->numSwitchNodes			= pmimagep->SwitchNodes;
	imageInfo->numSwitchPorts			= pmimagep->SwitchPorts;
	imageInfo->numLinks					= pmimagep->NumLinks;
	imageInfo->numSMs					= pmimagep->NumSMs;
	imageInfo->numNoRespNodes			= pmimagep->NoRespNodes;
	imageInfo->numNoRespPorts			= pmimagep->NoRespPorts;
	imageInfo->numSkippedNodes			= pmimagep->SkippedNodes;
	imageInfo->numSkippedPorts			= pmimagep->SkippedPorts;
	imageInfo->numUnexpectedClearPorts	= pmimagep->UnexpectedClearPorts;
	imageInfo->imageInterval			= pmimagep->imageInterval;
	for (i = 0; i < 2; i++) {
		STL_LID smLid = pmimagep->SMs[i].smLid;
		PmNode_t *pmnodep = pmimagep->LidMap[smLid];
		imageInfo->SMInfo[i].smLid		= smLid;
		imageInfo->SMInfo[i].priority	= pmimagep->SMs[i].priority;
		imageInfo->SMInfo[i].state		= pmimagep->SMs[i].state;
		if (pmnodep != NULL) {
			PmPort_t *pmportp = pm_node_lided_port(pmnodep);
			imageInfo->SMInfo[i].portNumber = pmportp->portNum;
			imageInfo->SMInfo[i].smPortGuid	= pmportp->guid;
			StringCopy(imageInfo->SMInfo[i].smNodeDesc, (char *)pmnodep->nodeDesc.NodeString,
				sizeof(imageInfo->SMInfo[i].smNodeDesc));
		} else {
			imageInfo->SMInfo[i].portNumber	= 0;
			imageInfo->SMInfo[i].smPortGuid	= 0;
			imageInfo->SMInfo[i].smNodeDesc[0] = 0;
		}
	}

	*returnImageId = retImageId;

done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

FSTATUS paGetVFList(Pm_t *pm, PmVFList_t *VFList, STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	Status_t vStatus;
	FSTATUS status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
	int i, j;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	PmImage_t *pmimagep = NULL;
	boolean requiresLock = TRUE;

	// check input parameters
	if (!pm || !VFList)
		return FINVALID_PARAMETER;

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, NULL, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	VFList->NumVFs = pmimagep->NumVFsActive;
	vStatus = vs_pool_alloc(&pm_pool, VFList->NumVFs * STL_PM_VFNAMELEN, (void*)&VFList->VfList);
	if (vStatus != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate name list buffer for VFList rc:", vStatus);
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}

	for (i = 0, j = 0; i < pmimagep->NumVFs; i++) {
		if (pmimagep->VFs[i].isActive) {
			StringCopy(VFList->VfList[j++].Name, pmimagep->VFs[i].Name, STL_PM_VFNAMELEN);
		}
	}
	*returnImageId = retImageId;
done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

FSTATUS paGetVFInfo(Pm_t *pm, char *vfName, PmVFInfo_t *pmVFInfo, STL_PA_IMAGE_ID_DATA imageId,
	STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS status = FSUCCESS;
	int vfIdx = -1;
	PmVFImage_t pmVFImage;
	PmImage_t *pmimagep = NULL;
	PmNode_t *pmnodep = NULL;
	PmPort_t *pmportp = NULL;
	uint8 portnum;
	PmPortImage_t *pmPortImageP = NULL;
	uint32 imageIndex, imageInterval;
	boolean requiresLock = TRUE;
	STL_LID lid;

	if (!pm || !vfName || !pmVFInfo)
		return(FINVALID_PARAMETER);
	if (vfName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal vfName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);


	status = LocateVF(pmimagep, vfName, &vfIdx);
	if (status != FSUCCESS || vfIdx == -1){
		IB_LOG_WARN_FMT(__func__, "VF %.*s not Found: %s", (int)sizeof(vfName), vfName, FSTATUS_ToString(status));
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
		goto done;
	}

	imageInterval = pmimagep->imageInterval;

	memset(&pmVFImage, 0, sizeof(PmVFImage_t));
	ClearVFStats(&pmVFImage);

	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			pmPortImageP = &pmportp->Image[imageIndex];
			if (PmIsPortInVF(pmimagep, pmPortImageP, vfIdx)) {
				if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
					PA_INC_COUNTER_NO_OVERFLOW(pmVFImage.IntUtil.pmaNoRespPorts, IB_UINT16_MAX);
				}
				pmVFImage.NumPorts++;
				UpdateVFStats(pm, imageIndex, pmportp, &pmVFImage, imageInterval);
				if (pmPortImageP->neighbor == NULL && portnum != 0) {
					PA_INC_COUNTER_NO_OVERFLOW(pmVFImage.IntUtil.topoIncompPorts, IB_UINT16_MAX);
				}
			}
		}
	}
	FinalizeVFStats(&pmVFImage);
	StringCopy(pmVFInfo->vfName, vfName, sizeof(pmVFInfo->vfName));
	pmVFInfo->NumPorts = pmVFImage.NumPorts;
	memcpy(&pmVFInfo->IntUtil, &pmVFImage.IntUtil, sizeof(PmUtilStats_t));
	memcpy(&pmVFInfo->IntErr, &pmVFImage.IntErr, sizeof(PmErrStats_t));
	pmVFInfo->MinIntRate = pmVFImage.MinIntRate;
	pmVFInfo->MaxIntRate = pmVFImage.MaxIntRate;

	*returnImageId = retImageId;

done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

FSTATUS paGetVFConfig(Pm_t *pm, char *vfName, uint64 vfSid, PmVFConfig_t *pmVFConfig,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	int vfIdx = -1;
	STL_LID lid;
	uint32 imageIndex;
	PmImage_t *pmimagep = NULL;
	PmNode_t *pmnodep = NULL;
	PmPort_t *pmportp = NULL;
	uint8 portnum;
	PmPortImage_t *portImage = NULL;
	FSTATUS status = FSUCCESS;
	boolean requiresLock = TRUE;

	// check input parameters
	if (!pm || !vfName || !pmVFConfig)
		return(FINVALID_PARAMETER);
	if (vfName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal vfName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	// initialize vf config port list counts
	pmVFConfig->NumPorts = 0;
	pmVFConfig->portListSize = 0;
	pmVFConfig->portList = NULL;

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	status = LocateVF(pmimagep, vfName, &vfIdx);
	if (status != FSUCCESS || vfIdx == -1){
		IB_LOG_WARN_FMT(__func__, "VF %.*s not Found: %s", (int)sizeof(vfName), vfName, FSTATUS_ToString(status));
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
		goto done;
	}

	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInVF(pmimagep, portImage, vfIdx)) {
				if (pmVFConfig->portListSize == pmVFConfig->NumPorts) {
					pmVFConfig->portListSize += PORTLISTCHUNK;
				}
				pmVFConfig->NumPorts++;
			}
		}
	}
	// check if there are ports to sort
	if (!pmVFConfig->NumPorts) {
		IB_LOG_INFO_FMT(__func__, "VF %.*s has no ports", (int)sizeof(vfName), vfName);
		goto norecords;
	}
	// allocate the port list
	Status_t ret = vs_pool_alloc(&pm_pool, pmVFConfig->portListSize * sizeof(PmPortConfig_t), (void *)&pmVFConfig->portList);
	if (ret !=  VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate list buffer for pmVFConfig rc:", ret);
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}
	// copy the port list
	int i = 0;
	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInVF(pmimagep, portImage, vfIdx)) {
				pmVFConfig->portList[i].lid = lid;
				pmVFConfig->portList[i].portNum = pmportp->portNum;
				pmVFConfig->portList[i].guid = pmnodep->NodeGUID;
				memcpy(pmVFConfig->portList[i].nodeDesc, (char *)pmnodep->nodeDesc.NodeString,
					sizeof(pmVFConfig->portList[i].nodeDesc));
				i++;
			}
		}
	}

norecords:
	StringCopy(pmVFConfig->vfName, vfName, STL_PM_VFNAMELEN);
	*returnImageId = retImageId;

done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

FSTATUS GetVfPortCounters(PmImage_t *pmimagep, PmCompositeVLCounters_t *vfPortCountersP, int vfIdx, boolean useHiddenVF,
	const PmPortImage_t *pmPortImageP, PmCompositeVLCounters_t *vlPortCountersP, uint32 *flagsp)
{
	uint32 SingleVLBit, vl, idx;
	uint32 VlSelectMask = 0, VlSelectMaskShared = 0, VFVlSelectMask = 0;
	// Start at -1 if using HiddenVF
	int i = (useHiddenVF ? -1 : 0);
	FSTATUS status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;

	// Iterate through All VFs in Port
	for (; i < (int)pmimagep->NumVFs; i++) {
		uint32 vlmask = (i == -1 ? 0x8000 : pmPortImageP->vfvlmap[i].vlmask);

		// Iterate through All VLs within each VF
		for (vl = 0; vl < STL_MAX_VLS && (vlmask >> vl); vl++) {
			SingleVLBit = 1 << vl;

			// Skip unassigned VLs (0) in VF
			if ((vlmask & SingleVLBit) == 0) continue;

			// If using Hidden VF only enter when i == -1 (one time for one vl[15])
			// OR is VF index matches argument
			if ((i == -1) || (i == vfIdx)) {

				// Keep track of VLs within this VF
				VFVlSelectMask |= SingleVLBit;
				idx = vl_to_idx(vl);
				vfPortCountersP->PortVLXmitData     += vlPortCountersP[idx].PortVLXmitData;
				vfPortCountersP->PortVLRcvData      += vlPortCountersP[idx].PortVLRcvData;
				vfPortCountersP->PortVLXmitPkts     += vlPortCountersP[idx].PortVLXmitPkts;
				vfPortCountersP->PortVLRcvPkts      += vlPortCountersP[idx].PortVLRcvPkts;
				vfPortCountersP->PortVLXmitWait     += vlPortCountersP[idx].PortVLXmitWait;
				vfPortCountersP->SwPortVLCongestion += vlPortCountersP[idx].SwPortVLCongestion;
				vfPortCountersP->PortVLRcvFECN      += vlPortCountersP[idx].PortVLRcvFECN;
				vfPortCountersP->PortVLRcvBECN      += vlPortCountersP[idx].PortVLRcvBECN;
				vfPortCountersP->PortVLXmitTimeCong += vlPortCountersP[idx].PortVLXmitTimeCong;
				vfPortCountersP->PortVLXmitWastedBW += vlPortCountersP[idx].PortVLXmitWastedBW;
				vfPortCountersP->PortVLXmitWaitData += vlPortCountersP[idx].PortVLXmitWaitData;
				vfPortCountersP->PortVLRcvBubble    += vlPortCountersP[idx].PortVLRcvBubble;
				vfPortCountersP->PortVLMarkFECN     += vlPortCountersP[idx].PortVLMarkFECN;
				vfPortCountersP->PortVLXmitDiscards += vlPortCountersP[idx].PortVLXmitDiscards;

				status = FSUCCESS;
			}

			// If VL bit is already set in Port's VLMask
			if (VlSelectMask & SingleVLBit) {
				// Add bit to indicate this VL is shared between 2 or more VFs
				VlSelectMaskShared |= SingleVLBit;
			} else {
				// If bit is not already set, set it
				VlSelectMask |= SingleVLBit;
			}
		}
	}
	// If one of the VLs in the shared mask is one of the VLs in the VF indicate
	//  this data is shared with at least one other VF
	if (VlSelectMaskShared & VFVlSelectMask) {
		*flagsp |= STL_PA_PC_FLAG_SHARED_VL;
	}

	return status;
}
FSTATUS paGetVFPortStats(Pm_t *pm, STL_LID lid, uint8 portNum, char *vfName,
	PmCompositeVLCounters_t *vfPortCountersP, uint32 delta, uint32 userCntrs,
	STL_PA_IMAGE_ID_DATA imageId, uint32 *flagsp, STL_PA_IMAGE_ID_DATA *returnImageId)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
	PmImage_t *pmimagep = NULL;
	PmPort_t *pmportp = NULL;
	PmPortImage_t *pmPortImageP = NULL;
	uint32 imageIndex;
	boolean requiresLock = TRUE;
	int vfIdx = -1;
	boolean useHiddenVF = !strcmp(HIDDEN_VL15_VF, vfName);

	if (!pm || !vfPortCountersP) {
		return(FINVALID_PARAMETER);
	}
	if (userCntrs && (delta || imageId.imageOffset)) {
		IB_LOG_WARN_FMT(__func__,  "Illegal combination of parameters: Offset (%d) and delta(%d) must be zero if UserCounters(%d) flag is set",
						imageId.imageOffset, delta, userCntrs);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (vfName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal vfName parameter: Empty String");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!lid) {
		IB_LOG_WARN_FMT(__func__,  "Illegal LID parameter: must not be zero");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!pm_config.process_vl_counters) {
		IB_LOG_WARN_FMT(__func__, "Processing of VL Counters has been disabled, VF Port Counters query cannot be completed");
		return(FINVALID_SETTING | STL_MAD_STATUS_STL_PA_NO_DATA);
	}

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	if (userCntrs) {
		STL_PA_IMAGE_ID_DATA liveImgId = { 0 };
		status = FindPmImage(__func__, pm, liveImgId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	} else {
		status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	}
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	if (!useHiddenVF) {
		status = LocateVF(pmimagep, vfName, &vfIdx);
		if (status != FSUCCESS || vfIdx == -1){
			IB_LOG_WARN_FMT(__func__, "VF %.*s not Found: %s", (int)sizeof(vfName), vfName, FSTATUS_ToString(status));
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
			goto done;
		}
	}

	pmportp = pm_find_port(pmimagep, lid, portNum);
	if (!pmportp) {
		IB_LOG_WARN_FMT(__func__, "Port not found: Lid 0x%x Port %u", lid, portNum);
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT;
		goto done;
	}
	pmPortImageP = &pmportp->Image[imageIndex];
	if (pmPortImageP->u.s.queryStatus != PM_QUERY_STATUS_OK) {
		IB_LOG_WARN_FMT(__func__, "Port Query Status Not OK: %s: Lid 0x%x Port %u",
			(pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_SKIP ? "Skipped" :
			(pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_FAIL_QUERY ? "Failed Query" : "Failed Clear")),
			lid, portNum);
		if ( pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_FAIL_CLEAR) {
			*flagsp |= STL_PA_PC_FLAG_CLEAR_FAIL;
		} else {
			if ( pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_SKIP) {
				status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_DATA;
			} else if( pmPortImageP->u.s.queryStatus == PM_QUERY_STATUS_FAIL_QUERY){
				status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_BAD_DATA;
			}
			goto done;
		}
	}

	if (userCntrs) {
		vs_rdlock(&pm->totalsLock);
		// Get Delta VF PortCounters From PA User Counters
		status = GetVfPortCounters(pmimagep, vfPortCountersP, vfIdx, useHiddenVF,
			pmPortImageP, pmportp->StlVLPortCountersTotal, flagsp);
		vs_rwunlock(&pm->totalsLock);

		*flagsp |= STL_PA_PC_FLAG_USER_COUNTERS |
			(isUnexpectedClearUserCounters ? STL_PA_PC_FLAG_UNEXPECTED_CLEAR : 0);
		*returnImageId = (STL_PA_IMAGE_ID_DATA){0};
	} else {
		// Grab ImageTime from Pm Image
		retImageId.imageTime.absoluteTime = (uint32)pmimagep->sweepStart;

		if (delta) {
			// Get Delta VF PortCounters From Image's PmImage Counters
			status = GetVfPortCounters(pmimagep, vfPortCountersP, vfIdx, useHiddenVF,
				pmPortImageP, pmPortImageP->DeltaStlVLPortCounters, flagsp);
			*flagsp |= STL_PA_PC_FLAG_DELTA;
		} else {
			// Get VF PortCounters From Image's PmImage Counters
			status = GetVfPortCounters(pmimagep, vfPortCountersP, vfIdx, useHiddenVF,
				pmPortImageP, pmPortImageP->StlVLPortCounters, flagsp);
		}
		*flagsp |= (pmPortImageP->u.s.UnexpectedClear ? STL_PA_PC_FLAG_UNEXPECTED_CLEAR : 0);
		*returnImageId = retImageId;
	}
done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

FSTATUS paClearVFPortStats(Pm_t *pm, STL_LID lid, uint8 portNum, STLVlCounterSelectMask select, char *vfName)
{
	FSTATUS status = FSUCCESS;
	PmImage_t *pmimagep = NULL;
	boolean requiresLock = TRUE;
	boolean useHiddenVF = !strcmp(HIDDEN_VL15_VF, vfName);
	STL_PA_IMAGE_ID_DATA liveImgId = {0};
	int vfIdx = -1;

	if(!lid) {
		IB_LOG_WARN_FMT(__func__, "Illegal LID parameter: Must not be zero");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!select.AsReg32) {
		IB_LOG_WARN_FMT(__func__, "Illegal select parameter: Must not be zero");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (vfName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal vfName parameter: Empty String");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!pm_config.process_vl_counters) {
		IB_LOG_WARN_FMT(__func__, "Processing of VL Counters has been disabled, Clear VF Port Counters query cannot be completed");
		return(FINVALID_SETTING | STL_MAD_STATUS_STL_PA_NO_DATA);
	}

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, liveImgId, NULL, &pmimagep, NULL, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	if (!useHiddenVF) {
		status = LocateVF(pmimagep, vfName, &vfIdx);
		if (status != FSUCCESS || vfIdx == -1){
			IB_LOG_WARN_FMT(__func__, "VF %.*s not Found: %s", (int)sizeof(vfName), vfName, FSTATUS_ToString(status));
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
			goto done;
		}
	}

	(void)vs_wrlock(&pm->totalsLock);
	if (portNum == PM_ALL_PORT_SELECT) {
		PmNode_t *pmnodep = pm_find_node(pmimagep, lid);
		if (! pmnodep || pmnodep->nodeType != STL_NODE_SW) {
			IB_LOG_WARN_FMT(__func__, "Switch not found: LID: 0x%x", lid);
			status = FNOT_FOUND;
		} else {
			status = PmClearNodeRunningVFCounters(pmnodep, select, vfIdx, useHiddenVF);
		}
	} else {
		PmPort_t *pmportp = pm_find_port(pmimagep, lid, portNum);
		if (! pmportp) {
			IB_LOG_WARN_FMT(__func__, "Port not found: Lid 0x%x Port %u", lid, portNum);
			status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT;
		} else {
			status = PmClearPortRunningVFCounters(pmportp, select, vfIdx, useHiddenVF);
		}
	}
	(void)vs_rwunlock(&pm->totalsLock);

done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	goto done;
}

/*************************************************************************************
 *
 * paGetVFExtFocusPorts - return a set of extended VF focus ports
 *
 * Inputs:
 *     pm - pointer to Pm_t (the PM main data type)
 *     vfName - pointer to name of virtual fabric
 *     pmVFFocusPorts - pointer to caller-declared data area to return VF focus ports
 *     imageId - Id of the image
 *     returnImageId - pointer Image Id that is used
 *     select  - focus select that is used
 *     start - the start index
 *     range - number of records requested
 *
 *  Return:
 *     FSTATUS - FSUCCESS if OK
 *
 *
 **************************************************************************************/
FSTATUS paGetExtVFFocusPorts(Pm_t *pm, char *vfName, PmFocusPorts_t *pmVFFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
	uint32 start, uint32 range)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	int i, vfIdx = -1;
	STL_LID lid;
	uint32 imageIndex;
	PmImage_t *pmimagep = NULL;
	PmNode_t *pmnodep = NULL;
	PmPort_t *pmportp = NULL;
	PmPortImage_t *portImage = NULL;
	FSTATUS status = FSUCCESS;
	boolean requiresLock = TRUE;
	uint32 allocatedItems = 0;
	uint8 portnum;

	// check input parameters
	if (!pm || !vfName || !pmVFFocusPorts)
		return(FINVALID_PARAMETER);
	if (vfName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal vfName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!range) {
		IB_LOG_WARN_FMT(__func__, "Illegal range parameter: %d: must be greater than zero\n", range);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	} else if (range > pm_config.subnet_size) {
		IB_LOG_WARN_FMT(__func__, "Illegal range parameter: Exceeds maxmimum subnet size of %d\n", pm_config.subnet_size);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	// initialize group config port list counts
	pmVFFocusPorts->NumPorts = 0;
	pmVFFocusPorts->portList = NULL;

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	status = LocateVF(pmimagep, vfName, &vfIdx);
	if (status != FSUCCESS || vfIdx == -1){
		IB_LOG_WARN_FMT(__func__, "VF %.*s not Found: %s", (int)sizeof(vfName), vfName, FSTATUS_ToString(status));
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
		goto done;
	}

	if(select == STL_PA_SELECT_UNEXP_CLR_PORT){
		allocatedItems = pmimagep->UnexpectedClearPorts;
	} else if( select == STL_PA_SELECT_NO_RESP_PORT) {
		allocatedItems = pmimagep->NoRespNodes;
	} else {
		allocatedItems = pmimagep->SkippedNodes;
	}

	if(allocatedItems == 0) {
		IB_LOG_WARN_FMT(__func__, "No Matching Records\n");
		status = FSUCCESS | MAD_STATUS_SA_NO_RECORDS;
		goto done;
	}

	status = vs_pool_alloc(&pm_pool, allocatedItems * sizeof(PmFocusPortEntry_t), (void*)&pmVFFocusPorts->portList);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate list buffer for pmExtVFFocusPorts rc:", status);
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}

	pmVFFocusPorts->NumPorts = allocatedItems;
	pmVFFocusPorts->portCntr = 0;
	StringCopy(pmVFFocusPorts->name, vfName, STL_PM_VFNAMELEN);

	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInVF(pmimagep, portImage, vfIdx)) {
				processExtFocusPort(imageIndex, pmportp, select, pmVFFocusPorts, lid);
			}
		}
	}

	/* trim list */
	if (start >= pmVFFocusPorts->NumPorts) {
		IB_LOG_WARN_FMT(__func__, "Illegal start parameter: Exceeds range of available entries %d\n", pmVFFocusPorts->NumPorts);
		status = (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
		goto done;
	}

	pmVFFocusPorts->NumPorts = pmVFFocusPorts->NumPorts - start;

	if (start > 0) {
		/* shift list start point to start index */
		for (i = 0; i < pmVFFocusPorts->NumPorts; i++)
			pmVFFocusPorts->portList[i] = pmVFFocusPorts->portList[start+i];
	}

	/* truncate list */
	if (range < pmVFFocusPorts->NumPorts) {
		pmVFFocusPorts->NumPorts = range;
	}

	if (status == FSUCCESS)
		*returnImageId = retImageId;
done:
	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

FSTATUS paGetVFFocusPorts(Pm_t *pm, char *vfName, PmFocusPorts_t *pmVFFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
	uint32 start, uint32 range)
{
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	STL_LID lid;
	uint32 imageIndex, imageInterval;
	PmImage_t *pmimagep = NULL;
	PmNode_t *pmnodep = NULL;
	PmPort_t *pmportp = NULL;
	uint8 portnum;
	PmPortImage_t *portImage = NULL;
	FSTATUS status = FSUCCESS;
	Status_t allocStatus;
	ComputeFunc_t computeFunc = NULL;
	QsortCompareFunc_t compareFunc = NULL;
	boolean requiresLock = TRUE;
	uint32 allocatedItems = 0;
	PmVFFocusPortComputeData_t vfComputeData;
	focusArrayItem_t *focusArray = NULL;
	int items = 0, vfIdx = -1;

	void *computeData = NULL;

	// check input parameters
	if (!pm || !vfName || !pmVFFocusPorts)
		return(FINVALID_PARAMETER);
	if (vfName[0] == '\0') {
		IB_LOG_WARN_FMT(__func__, "Illegal vfName parameter: Empty String\n");
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}
	if (!range) {
		IB_LOG_WARN_FMT(__func__, "Illegal range parameter: %d: must be greater than zero\n", range);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	} else if (range > pm_config.subnet_size) {
		IB_LOG_WARN_FMT(__func__, "Illegal range parameter: Exceeds maxmimum subnet size of %d\n", pm_config.subnet_size);
		return(FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER);
	}

	if ((!pm_config.process_vl_counters) && (IS_VF_FOCUS_SELECT(select))) {
		IB_LOG_WARN_FMT(__func__, "Processing of VL Counters has been disabled, Get VF Focus Port query cannot be completed");
		return(FINVALID_SETTING | STL_MAD_STATUS_STL_PA_NO_DATA);
	}

	switch (select) {
	case STL_PA_SELECT_UTIL_HIGH:
		computeFunc = &computeUtilizationPct10;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_UTIL_PKTS_HIGH:
		computeFunc = &computeSendKPkts;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_UTIL_LOW:
		computeFunc = &computeUtilizationPct10;
		compareFunc = &compareFocusArrayItemsLT;
		computeData = (void *)&imageInterval;
		break;
	case STL_PA_SELECT_CATEGORY_INTEG:
		computeFunc = &computeIntegrity;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = &pm->integrityWeights;
		break;
	case STL_PA_SELECT_CATEGORY_CONG:
		computeFunc = &computeCongestion;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = &pm->congestionWeights;
		break;
	case STL_PA_SELECT_CATEGORY_SMA_CONG:
		computeFunc = &computeSmaCongestion;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = &pm->congestionWeights;
		break;
	case STL_PA_SELECT_CATEGORY_BUBBLE:
		computeFunc = &computeBubble;
		compareFunc = &compareFocusArrayItemsGT;
		break;
	case STL_PA_SELECT_CATEGORY_SEC:
		computeFunc = &computeSecurity;
		compareFunc = &compareFocusArrayItemsGT;
		break;
	case STL_PA_SELECT_CATEGORY_ROUT:
		computeFunc = &computeRouting;
		compareFunc = &compareFocusArrayItemsGT;
		break;
	case STL_PA_SELECT_VF_UTIL_HIGH:
		computeFunc = &computeVFUtilizationPct10;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = (void *)&vfComputeData;
		break;
	case STL_PA_SELECT_VF_UTIL_PKTS_HIGH:
		computeFunc = &computeVFSendKPkts;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = (void *)&vfComputeData;
		break;
	case STL_PA_SELECT_VF_UTIL_LOW:
		computeFunc = &computeVFUtilizationPct10;
		compareFunc = &compareFocusArrayItemsLT;
		computeData = (void *)&vfComputeData;
		break;
	case STL_PA_SELECT_CATEGORY_VF_CONG:
		vfComputeData.congestionWeights = pm->congestionWeights;
		computeFunc = &computeVFCongestion;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = (void *)&vfComputeData;
		break;
	case STL_PA_SELECT_CATEGORY_VF_BUBBLE:
		computeFunc = &computeVFBubble;
		compareFunc = &compareFocusArrayItemsGT;
		computeData = (void *)&vfComputeData;
		break;
	default:
		IB_LOG_WARN_FMT(__func__, "Illegal select parameter: 0x%x", select);
		return FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;
		break;
	}

	// initialize group config port list counts
	pmVFFocusPorts->NumPorts = 0;
	pmVFFocusPorts->portList = NULL;

	AtomicIncrementVoid(&pm->refCount); // prevent engine from stopping
	if (! PmEngineRunning()) {          // see if is already stopped/stopping
		status = FUNAVAILABLE;
		goto done;
	}

	(void)vs_rdlock(&pm->stateLock);
	status = FindPmImage(__func__, pm, imageId, &retImageId, &pmimagep, &imageIndex, &requiresLock);
	if (status != FSUCCESS || !pmimagep) goto error;
	if (requiresLock) (void)vs_rdlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);

	status = LocateVF(pmimagep, vfName, &vfIdx);
	if (status != FSUCCESS || vfIdx == -1){
		IB_LOG_WARN_FMT(__func__, "VF %.*s not Found: %s", (int)sizeof(vfName), vfName, FSTATUS_ToString(status));
		status = FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF;
		goto done;
	}
	vfComputeData.vfIdx = vfIdx;

	imageInterval = pmimagep->imageInterval;
	vfComputeData.imageInterval = imageInterval;

	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInVF(pmimagep, portImage, vfIdx)) {
				if (portnum && lid > pmportp->neighbor_lid) continue;
				allocatedItems++;
			}
		}
	}
	if (allocatedItems == 0 || start >= allocatedItems) {
		goto norecords;
	}

	allocStatus = vs_pool_alloc(&pm_pool, allocatedItems * sizeof(focusArrayItem_t), (void*)&focusArray);
	if (allocStatus != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to allocate list buffer for pmFocusPorts rc:", allocStatus);
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}
	memset(focusArray, 0, allocatedItems * sizeof(focusArrayItem_t));

	for_all_pmnodes(pmimagep, pmnodep, lid) {
		for_all_pmports(pmnodep, pmportp, portnum, !requiresLock) {
			portImage = &pmportp->Image[imageIndex];
			if (PmIsPortInVF(pmimagep, portImage, vfIdx)) {
				if (portnum && lid > pmportp->neighbor_lid) continue;
				focusAddArrayItem(pm, imageIndex, focusArray, items, pmportp, computeFunc, computeData);
				items++;
			}
		}
	}
	if (items != allocatedItems) {
		IB_LOG_ERROR_FMT(__func__, "Number of Processed Items (%d) does no match array size %d\n", items, allocatedItems);
		status = FINSUFFICIENT_MEMORY;
		goto done;
	}

	/* Trim items until start */
	items = items - start;

	/* Sort Array Items */
	if (items > 1) {
		qsort(focusArray, allocatedItems, sizeof(focusArrayItem_t), compareFunc);
	}
	/* Trim ending items until within range */
	items = MIN(items, range);

	StringCopy(pmVFFocusPorts->name, vfName, STL_PM_VFNAMELEN);
	pmVFFocusPorts->NumPorts = items;
	status = focusGetItems(pmVFFocusPorts, focusArray, start, imageIndex);

norecords:
	if (status == FSUCCESS){
		*returnImageId = retImageId;
	}
done:
	if (focusArray) {
		(void)vs_pool_free(&pm_pool, focusArray);
	}

	if (requiresLock && pmimagep) (void)vs_rwunlock(&pmimagep->imageLock);
	AtomicDecrementVoid(&pm->refCount);
	return(status);
error:
	(void)vs_rwunlock(&pm->stateLock);
	returnImageId->imageNumber = BAD_IMAGE_ID;
	goto done;
}

// Append details about frozen images into memory pointed to by buffer
static void appendFreezeFrameDetails(uint8_t *buffer, uint32_t *index)
{
	extern Pm_t 	g_pmSweepData;
	Pm_t			*pm=&g_pmSweepData;
	uint8			freezeIndex;
	uint8			numFreezeImages=0;
	uint64_t		ffImageId=0;

	for (freezeIndex = 0; freezeIndex < pm_config.freeze_frame_images; freezeIndex ++) {
		if (pm->freezeFrames[freezeIndex] != PM_IMAGE_INDEX_INVALID) {
			numFreezeImages++;
		}
	}
	if (pm_config.shortTermHistory.enable && pm->ShortTermHistory.cachedComposite) {
		numFreezeImages++;
	}
	buffer[(*index)++]=numFreezeImages;
	if (numFreezeImages == 0) {
		return;
	}
	for (freezeIndex = 0; freezeIndex < pm_config.freeze_frame_images; freezeIndex ++) {
		if (pm->freezeFrames[freezeIndex] != PM_IMAGE_INDEX_INVALID) {
			ffImageId = BuildFreezeFrameImageId(pm, freezeIndex, 0 /*client id */, NULL);
			memcpy(&buffer[*index], &ffImageId, sizeof(uint64_t));
			*index += sizeof(uint64_t);
			IB_LOG_VERBOSELX("Appending freeze frame id", ffImageId);
		}
	}
	if (pm_config.shortTermHistory.enable && pm->ShortTermHistory.cachedComposite) {
		ffImageId = pm->ShortTermHistory.cachedComposite->header.common.imageIDs[0];
		memcpy(&buffer[*index], &ffImageId, sizeof(uint64_t));
		*index += sizeof(uint64_t);
		IB_LOG_VERBOSELX("Appending Hist freeze frame id", ffImageId);
	}
	return;
}

// return Master PM Sweep History image file data copied into memory pointed to by buffer
int getPMHistFileData(char *histPath, char *filename, uint32_t histindex, uint8_t *buffer, uint32_t bufflen, uint32_t *filelen)
{
    const size_t MAX_PATH = PM_HISTORY_FILENAME_LEN + 1 + 256;
    char path[MAX_PATH];
    uint32_t      index=0, nextByte = 0;
    FILE          *file;

    if (!buffer || !filelen)
        return -1;

    if (filename[0] == '\0') {
        IB_LOG_VERBOSE_FMT(__func__, "Missing hist filename.");
        return -1;
    }

    snprintf(path, MAX_PATH, "%s/%s", histPath, filename);
    file = fopen(path, "rb" );
    if (!file) {
        IB_LOG_ERROR_FMT(__func__,"Error opening PA history image file %s:%s.", path, strerror(errno));
        return -1;
    }
    while((nextByte = fgetc(file)) != EOF) {
        buffer[index++] = nextByte;
        if (index >= bufflen) {
            *filelen = 0;
            IB_LOG_ERROR("PM Composite image file overrunning buffer! rc:",0x0020);
            fclose(file);
            return -1;
        }
    }
    *filelen = index;
    fclose(file);

    return 0;

}	// End of getPMHistFileData()

// return latest Master PM Sweep Image Data copied into memory pointed to by buffer
int getPMSweepImageData(char *filename, uint32_t histindex, uint8_t isCompressed, uint8_t *buffer, uint32_t bufflen, uint32_t *filelen)
{
	extern Pm_t g_pmSweepData;
	Pm_t	*pm=&g_pmSweepData;
	uint32_t	index=0;

	if (!buffer || !filelen)
		return -1;

	if (! PmEngineRunning()) {	// see if is already stopped/stopping
		return -1;
	}

	if (filename[0] != '\0') {
		return getPMHistFileData(pm->ShortTermHistory.filepath, filename, histindex, buffer, bufflen, filelen);
	}
	else { /* file name not specified */
		(void)vs_rdlock(&pm->stateLock);
		if (pm->history[histindex] == PM_IMAGE_INDEX_INVALID) {
			(void)vs_rwunlock(&pm->stateLock);
			return -1;
		}

		(void)vs_rdlock(&pm->Image[pm->history[histindex]].imageLock);
		if (pm->Image[pm->history[histindex]].state != PM_IMAGE_VALID) {
			IB_LOG_VERBOSE_FMT(__func__,"Invalid history index :0x%x", histindex);
			(void)vs_rwunlock(&pm->Image[pm->history[histindex]].imageLock);
			(void)vs_rwunlock(&pm->stateLock);
			return -1;
		}
		IB_LOG_VERBOSE_FMT(__func__, "Going to send latest hist imageIndex=0x%x size=%"PRISZT, histindex, computeCompositeSize());
		snprintf(filename, SMDBSYNCFILE_NAME_LEN, "%s/latest_sweep", pm->ShortTermHistory.filepath);
		writeImageToBuffer(pm, histindex, isCompressed, buffer, &index);
		appendFreezeFrameDetails(buffer, &index);
		*filelen = index;
		(void)vs_rwunlock(&pm->Image[pm->history[histindex]].imageLock);
		(void)vs_rwunlock(&pm->stateLock);
	}
	return 0;
}	// End of getPMSweepImageData()

// copy latest PM Sweep Image Data received from Master PM to Standby PM.
int putPMSweepImageData(char *filename, uint8_t *buffer, uint32_t filelen)
{
	extern Pm_t g_pmSweepData;
	Pm_t		*pm = &g_pmSweepData;

	if (!buffer || !filename) {
		IB_LOG_VERBOSE_FMT(__func__, "Null buffer/file");
		return -1;
	}

	if (!pm_config.shortTermHistory.enable || !PmEngineRunning()) {	// see if is already stopped/stopping
		return -1;
	}

	return injectHistoryFile(pm, filename, buffer, filelen);

}	// End of putPMSweepImageData()

// Temporary declarations
FSTATUS compoundNewImage(Pm_t *pm);
boolean PmCompareHFIPort(PmPort_t *pmportp, char *groupName);
boolean PmCompareTCAPort(PmPort_t *pmportp, char *groupName);
boolean PmCompareSWPort(PmPort_t *pmportp, char *groupName);

extern void release_pmnode(Pm_t *pm, PmNode_t *pmnodep);
extern void free_pmportList(Pm_t *pm, PmNode_t *pmnodep);
extern void free_pmport(Pm_t *pm, PmPort_t *pmportp);
extern PmNode_t *get_pmnodep(Pm_t *pm, Guid_t guid, STL_LID lid);
extern PmPort_t *new_pmport(Pm_t *pm);
extern uint32 connect_neighbor(Pm_t *pm, PmPort_t *pmportp);

// Free Node List
void freeNodeList(Pm_t *pm, PmImage_t *pmimagep) {
	uint32_t	i;
	PmNode_t	*pmnodep;

	if (pmimagep->LidMap) {
		for (i = 0; i <= pmimagep->maxLid; i++) {
			pmnodep = pmimagep->LidMap[i];
			if (pmnodep) {
				release_pmnode(pm, pmnodep);
			}
		}
		vs_pool_free(&pm_pool, pmimagep->LidMap);
		pmimagep->LidMap = NULL;
	}
	return;

}	// End of freeNodeList()

// Find Group by name
static FSTATUS FindPmGroup(PmImage_t *pmimagep, PmGroup_t **pmgrouppp, PmGroup_t *sthgroupp) {
	uint32_t i;

	if (!pmgrouppp) return FINVALID_PARAMETER;

	*pmgrouppp = NULL;
	if (sthgroupp) {
		for (i = 0; i < pmimagep->NumGroups; i++) {
			if (!strcmp(sthgroupp->Name, pmimagep->Groups[i].Name)) {
				*pmgrouppp = &pmimagep->Groups[i];
				return FSUCCESS;
			}
		}
	}
	return FNOT_FOUND;
}	// End of FindPmGroup()


// Copy Short Term History Port to PmImage Port
FSTATUS CopyPortToPmImage(Pm_t *pm, PmImage_t *pmimagep, PmNode_t *pmnodep, PmPort_t **pmportpp, PmPort_t *sthportp) {
    FSTATUS			ret = FSUCCESS;
	uint32_t		i, j;
	PmPort_t		*pmportp = NULL;
	PmPortImage_t	*pmportimgp;
	PmPortImage_t	*sthportimgp;

	if (!pmnodep || !pmportpp) {
		return FINVALID_PARAMETER;
	}

	if (!sthportp || (!sthportp->guid && !sthportp->portNum)) {
		// No port to copy
		goto exit;
	}

	pmportp = *pmportpp;

	// Allocate port
	if (!pmportp) {
		pmportp = new_pmport(pm);
		if (!pmportp) goto exit;

		pmportp->guid = sthportp->guid;
		pmportp->portNum = sthportp->portNum;
		pmportp->capmask = sthportp->capmask;
		pmportp->pmnodep = pmnodep;
	}
	pmportp->u.AsReg8 = sthportp->u.AsReg8;
	pmportp->neighbor_lid = sthportp->neighbor_lid;
	pmportp->neighbor_portNum = sthportp->neighbor_portNum;
	pmportp->groupWarnings = sthportp->groupWarnings;
	pmportp->StlPortCountersTotal = sthportp->StlPortCountersTotal;
	for (i = 0; i < MAX_PM_VLS; i++)
		pmportp->StlVLPortCountersTotal[i] = sthportp->StlVLPortCountersTotal[i];

	// Don't Copy port counters. Reconstituted image doesn't contain counters.

	// Copy port image
	pmportimgp = &pmportp->Image[pm->SweepIndex];
	sthportimgp = &sthportp->Image[0];
	*pmportimgp = *sthportimgp;

	// Connect neighbors later
	pmportimgp->neighbor = NULL;

	// Copy port image port groups
	memset(&pmportimgp->Groups, 0, sizeof(PmGroup_t *) * PM_MAX_GROUPS_PER_PORT);
#if PM_COMPRESS_GROUPS
	pmportimgp->numGroups = 0;
	for (j = 0, i = 0; i < sthportimgp->numGroups; i++)
#else
	for (j = 0, i = 0; i < PM_MAX_GROUPS_PER_PORT; i++)
#endif
	{
		if (!sthportimgp->Groups[i]) continue;
		ret = FindPmGroup(pmimagep, &pmportimgp->Groups[j], sthportimgp->Groups[i]);
		if (ret == FSUCCESS) {
			j++;
		} else if (ret == FNOT_FOUND) {
			IB_LOG_ERROR_FMT(__func__, "Port group not found: %s", sthportimgp->Groups[i]->Name);
			ret = FSUCCESS;
		} else {
			IB_LOG_ERROR_FMT(__func__, "Error in Port Image Group:%d", ret);
			goto exit_dealloc;
		}
	}
#if PM_COMPRESS_GROUPS
	pmportimgp->numGroups = j;
#endif

	// Copy port image VF groups
	memcpy(pmportimgp->vfvlmap, sthportimgp->vfvlmap, sizeof(vfmap_t) * MAX_VFABRICS);

	goto exit;

exit_dealloc:
	free_pmport(pm, pmportp);
	pmportp = NULL;

exit:
	// Update *pmportpp
	*pmportpp = pmportp;
	return ret;

}	// End of CopyPortToPmImage()

// Copy Short Term History Node to PmImage Node
FSTATUS CopyNodeToPmImage(Pm_t *pm, PmImage_t *pmimagep, STL_LID lid, PmNode_t **pmnodepp, PmNode_t *sthnodep) {
    FSTATUS		ret = FSUCCESS;
	Status_t	rc;
	uint32_t	i;
	PmNode_t	*pmnodep;
	Guid_t		guid;

	if (!pmnodepp) {
		ret = FINVALID_PARAMETER;
		goto exit;
	}

	if (!sthnodep) {
		// No node to copy
		goto exit;
	}

	if (sthnodep->nodeType == STL_NODE_SW)
		guid = sthnodep->up.swPorts[0]->guid;
	else
		guid = sthnodep->up.caPortp->guid;

	pmnodep = get_pmnodep(pm, guid, lid);
	if (pmnodep) {
		ASSERT(pmnodep->nodeType == sthnodep->nodeType);
		ASSERT(pmnodep->numPorts == sthnodep->numPorts);
	} else {
		// Allocate node
		rc = vs_pool_alloc(&pm_pool, pm->PmNodeSize, (void *)&pmnodep);
		*pmnodepp = pmnodep;
		if (rc != VSTATUS_OK || !pmnodep) {
			IB_LOG_ERRORRC( "Failed to allocate PM Node rc:", rc);
			ret = FINSUFFICIENT_MEMORY;
			goto exit;
		}
		MemoryClear(pmnodep, pm->PmNodeSize);
		AtomicWrite(&pmnodep->refCount, 1);
		pmnodep->nodeType = sthnodep->nodeType;
		pmnodep->numPorts = sthnodep->numPorts;
		pmnodep->NodeGUID = guid;
		pmnodep->SystemImageGUID = sthnodep->SystemImageGUID;
		cl_map_item_t *mi;
    	mi = cl_qmap_insert(&pm->AllNodes, guid, &pmnodep->AllNodesEntry);
		if (mi != &pmnodep->AllNodesEntry) {
			IB_LOG_ERRORLX("duplicate Node for portGuid", guid);
			goto exit_dealloc;
		}
	}
	pmnodep->nodeDesc = sthnodep->nodeDesc;
	pmnodep->changed_count = pm->SweepIndex;
	pmnodep->deviceRevision = sthnodep->deviceRevision;
	pmnodep->u = sthnodep->u;
	pmnodep->dlid = sthnodep->dlid;
	pmnodep->pkey = sthnodep->pkey;
	pmnodep->qpn = sthnodep->qpn;
	pmnodep->qkey = sthnodep->qkey;
	pmnodep->Image[pm->SweepIndex] = sthnodep->Image[0];

	if (sthnodep->nodeType == STL_NODE_SW) {
		if (!pmnodep->up.swPorts) {
			rc = vs_pool_alloc(&pm_pool, sizeof(PmPort_t *) * (pmnodep->numPorts + 1), (void *)&pmnodep->up.swPorts);
			if (rc != VSTATUS_OK || !pmnodep->up.swPorts) {
				IB_LOG_ERRORRC( "Failed to allocate Node Port List rc:", rc);
				ret = FINSUFFICIENT_MEMORY;
				goto exit_dealloc;
			}
			MemoryClear(pmnodep->up.swPorts, sizeof(PmPort_t *) * (pmnodep->numPorts + 1));
		}
		for (i = 0; i <= pmnodep->numPorts; i++) {
			ret = CopyPortToPmImage(pm, pmimagep, pmnodep, &pmnodep->up.swPorts[i], sthnodep ? sthnodep->up.swPorts[i] : NULL);
			if (ret != FSUCCESS) {
				IB_LOG_ERROR_FMT(__func__, "Error in Port Copy:%d", ret);
				goto exit_dealloc;
			}
		}
	} else {
		ret = CopyPortToPmImage(pm, pmimagep, pmnodep, &pmnodep->up.caPortp, sthnodep ? sthnodep->up.caPortp : NULL);
		if (ret != FSUCCESS) {
			IB_LOG_ERROR_FMT(__func__, "Error in Port Copy:%d", ret);
			goto exit_dealloc;
		}
	}

	// Update *pmnodepp
	*pmnodepp = pmnodep;

	goto exit;

exit_dealloc:
	if (pmnodep)
		release_pmnode(pm, pmnodep);
	*pmnodepp = NULL;

exit:
	return ret;

}	// End of CopyNodeToPmImage()

void set_neighbor(Pm_t *pm, PmPort_t *pmportp)
{
	PmPort_t *neighbor;
	PmPortImage_t *portImage = &pmportp->Image[pm->SweepIndex];

	if (portImage->u.s.Initialized && pmportp->neighbor_lid) {
		neighbor = pm_find_port(&pm->Image[pm->SweepIndex], pmportp->neighbor_lid, pmportp->neighbor_portNum);
		if (neighbor) {
			portImage->neighbor = neighbor;
		}
	}
}

// Copy Short Term History Image to PmImage
FSTATUS CopyToPmImage(Pm_t *pm, PmImage_t *pmimagep, PmImage_t *sthimagep)
{
	FSTATUS		ret = FSUCCESS;
	Status_t	rc;
	uint32_t	i, j;
	Lock_t		orgImageLock;	// Lock image data (except state and imageId).

	if (!pmimagep || !sthimagep) {
		ret = FINVALID_PARAMETER;
		goto exit;
	}

	// retain PmImage lock
	orgImageLock = pmimagep->imageLock;

	// Shallow Copy (includes VF/PmPortGroup data)
	*pmimagep = *sthimagep;

	pmimagep->LidMap = NULL;
	rc = vs_pool_alloc(&pm_pool, sizeof(PmNode_t *) * (pmimagep->maxLid + 1), (void *)&pmimagep->LidMap);
	if (rc != VSTATUS_OK || !pmimagep->LidMap) {
		IB_LOG_ERRORRC( "Failed to allocate PM Lid Map rc:", rc);
		ret = FINSUFFICIENT_MEMORY;
		goto exit_dealloc;
	}
	MemoryClear(pmimagep->LidMap, sizeof(PmNode_t *) * (pmimagep->maxLid + 1));
	for (i = 1; i <= pmimagep->maxLid; i++) {
		ret = CopyNodeToPmImage(pm, pmimagep, i, &pmimagep->LidMap[i], sthimagep->LidMap[i]);
		if (ret != FSUCCESS) {
			IB_LOG_ERROR_FMT(__func__, "Error in Node Copy:%d", ret);
			goto exit_dealloc;
		}
	}
	for (i = 1; i <= pmimagep->maxLid; i++) {
		if (pmimagep->LidMap[i]) {
			if (pmimagep->LidMap[i]->nodeType == STL_NODE_SW) {
				for (j = 0; j <= pmimagep->LidMap[i]->numPorts; j++) {
					if (pmimagep->LidMap[i]->up.swPorts[j]) {
						set_neighbor(pm, pmimagep->LidMap[i]->up.swPorts[j]);
					}
				}
			} else {
				set_neighbor(pm, pmimagep->LidMap[i]->up.caPortp);
			}
		}
	}

	goto exit_unlock;

exit_dealloc:
	freeNodeList(pm, pmimagep);

exit_unlock:
	// restore PmImage lock
	pmimagep->imageLock = orgImageLock;

exit:
	return ret;

}	// End of CopyToPmImage()

// Integrate ShortTermHistory.LoadedImage into PM RAM-resident image storage
FSTATUS PmReintegrate(Pm_t *pm, PmShortTermHistory_t *sth)
{
	FSTATUS		ret = FSUCCESS;
	PmImage_t	*pmimagep;
	PmImage_t	*sthimagep;

	pmimagep = &pm->Image[pm->SweepIndex];
	sthimagep = sth->LoadedImage.img;

	// More image processing (from PmSweepAllPortCounters)
	pmimagep->NoRespNodes = pmimagep->NoRespPorts = 0;
	pmimagep->SkippedNodes = pmimagep->SkippedPorts = 0;
	pmimagep->UnexpectedClearPorts = 0;
	pmimagep->DowngradedPorts = 0;
//	(void)PmClearAllNodes(pm);

	freeNodeList(pm, pmimagep);	// Free old Node List (LidMap) if present

	ret = CopyToPmImage(pm, pmimagep, sthimagep);
	if (ret != FSUCCESS) {
		IB_LOG_ERROR_FMT(__func__, "Error in Image Copy:%d", ret);
		goto exit;
	}

exit:
	return ret;

}	// End of PmReintegrate()

// copy latest PM RAM-Resident Sweep Image Data received from Master PM to Standby PM.
FSTATUS putPMSweepImageDataR(uint8_t *p_img_in, uint32_t len_img_in) {
    FSTATUS		ret = FSUCCESS;
	uint64		size;
	uint32		sweep_num;
	time_t		now_time;
	extern		Pm_t g_pmSweepData;
	Pm_t		*pm = &g_pmSweepData;
	PmShortTermHistory_t *sth = &pm->ShortTermHistory;
	PmImage_t	*pmimagep;
#ifndef __VXWORKS__
	Status_t	status;
#endif
	PmCompositeImage_t	*cimg_in = (PmCompositeImage_t *)p_img_in;
	PmCompositeImage_t	*cimg_out = NULL;
	unsigned char *p_decompress = NULL;
	unsigned char *bf_decompress = NULL;
	static time_t		firstImageSweepStart;
	static uint32		processedSweepNum=0;
	uint32				history_version;
	double				isTdelta;
	boolean				skipCompounding = FALSE;
	uint8		tempInstanceId;

    if (!p_img_in || !len_img_in)
        return FINVALID_PARAMETER;

	if (!PmEngineRunning()) {	// see if is already stopped/stopping
		return -1;
	}

	// check the version
	history_version = cimg_in->header.common.historyVersion;
	BSWAP_PM_HISTORY_VERSION(&history_version);
	if (history_version != PM_HISTORY_VERSION) {
		IB_LOG_INFO_FMT(__func__, "Received image buffer version (v%u.%u) does not match current version: v%u.%u",
			((history_version >> 24) & 0xFF), (history_version & 0x00FFFFFF),
			((PM_HISTORY_VERSION >> 24) & 0xFF), (PM_HISTORY_VERSION & 0x00FFFFFF));

		return FINVALID_PARAMETER;
	}

	BSWAP_PM_FILE_HEADER(&cimg_in->header);
#ifndef __VXWORKS__
	// Decompress image if compressed
	if (cimg_in->header.common.isCompressed) {
		status = vs_pool_alloc(&pm_pool, cimg_in->header.flatSize, (void *)&bf_decompress);
		if (status != VSTATUS_OK || !bf_decompress) {
			IB_LOG_ERRORRC("Unable to allocate flat buffer rc:", status);
			ret = FINSUFFICIENT_MEMORY;
			goto exit_free;
		}
		MemoryClear(bf_decompress, cimg_in->header.flatSize);
		// copy the header
		memcpy(bf_decompress, p_img_in, sizeof(PmFileHeader_t));
		// decompress the rest
		ret = decompressAndReassemble(p_img_in + sizeof(PmFileHeader_t),
									  len_img_in - sizeof(PmFileHeader_t),
									  cimg_in->header.numDivisions,
									  cimg_in->header.divisionSizes,
									  bf_decompress + sizeof(PmFileHeader_t),
									  cimg_in->header.flatSize - sizeof(PmFileHeader_t));
		if (ret != FSUCCESS) {
			IB_LOG_ERROR0("Unable to decompress image buffer");
			goto exit_free;
		}
		p_decompress = bf_decompress;
	} else {
#endif
		p_decompress = p_img_in;
#ifndef __VXWORKS__
	}
#endif
	BSWAP_PM_COMPOSITE_IMAGE_FLAT((PmCompositeImage_t *)p_decompress, 0 /*, history_version*/);

	// Rebuild composite
	//status = vs_pool_alloc(&pm_pool, sizeof(PmCompositeImage_t), (void *)&cimg_out);
	cimg_out = calloc(1,sizeof(PmCompositeImage_t));
	if (!cimg_out) {
		IB_LOG_ERROR0("Unable to allocate image buffer");
		ret = FINSUFFICIENT_MEMORY;
		goto exit_free;
	}

	MemoryClear(cimg_out, sizeof(PmCompositeImage_t));
	memcpy(cimg_out, p_decompress, sizeof(PmFileHeader_t));
	// Get sweepNum from image ID
	sweep_num = ((ImageId_t)cimg_out->header.common.imageIDs[0]).s.sweepNum;
	// Get the instance ID from the image ID
	tempInstanceId = ((ImageId_t)cimg_out->header.common.imageIDs[0]).s.instanceId;
	ret = rebuildComposite(cimg_out, p_decompress + sizeof(PmFileHeader_t), history_version);
	if (ret != FSUCCESS) {
		IB_LOG_ERRORRC("Error rebuilding PM Composite Image rc:", ret);
		goto exit_free;
	}

	(void)vs_wrlock(&pm->stateLock);
	pmimagep = &pm->Image[pm->SweepIndex];
	(void)vs_wrlock(&pmimagep->imageLock);

	// Reconstitute composite
	ret = PmReconstitute(sth, cimg_out);
	if (ret != FSUCCESS) {
		IB_LOG_ERRORRC("Error reconstituting PM Composite Image rc:", ret);
		goto exit_unlock;
	}

	if (isFirstImg) {
		firstImageSweepStart = sth->LoadedImage.img->sweepStart;
		IB_LOG_INFO_FMT(__func__, "First New Sweep Image Received...");
		processedSweepNum = sweep_num;
	}
	else {
		/* compare received image sweepStart time with that of first image. */
		isTdelta = difftime(firstImageSweepStart, sth->LoadedImage.img->sweepStart);
		if (isTdelta > 0) { /* firstImageSweepStart time is greater; i.e. received an older RAM image.*/
			skipCompounding = TRUE;
			IB_LOG_INFO_FMT( __func__, "TDelta = %f, Older Sweep Image Received. Skip processing.", isTdelta);
		}
		else {
			if ((sweep_num > processedSweepNum) || /* wrap around */(sweep_num == (processedSweepNum+1))) {
				IB_LOG_INFO_FMT( __func__, "TDelta = %f, New Sweep Image Received. Processing..", isTdelta);
				processedSweepNum = sweep_num;
			}
			else {
				skipCompounding = TRUE;
				IB_LOG_INFO_FMT( __func__, "Same/older Sweep Image Received. Skip processing..");
			}
		}
	}

	// Reintegrate image into g_pmSweepData as a sweep image
	if (!skipCompounding) {
		ret = PmReintegrate(pm, sth);
		if (ret != FSUCCESS) {
			IB_LOG_ERRORRC("Error reintegrating PM Composite Image rc:", ret);
			goto exit_unlock;
		}
	}

	vs_stdtime_get(&now_time);

	if (!skipCompounding) {
		// Complete sweep image processing
		pm->LastSweepIndex = pm->SweepIndex;	// last valid sweep
		pm->lastHistoryIndex = (pm->lastHistoryIndex + 1) % pm_config.total_images;
		pm->history[pm->lastHistoryIndex] = pm->LastSweepIndex;
		pm->ShortTermHistory.currentInstanceId = tempInstanceId;
		pmimagep->historyIndex = pm->lastHistoryIndex;
		pmimagep->sweepNum = sweep_num;
		pmimagep->state = PM_IMAGE_VALID;

#if CPU_LE
		// Process STH files only on LE CPUs
#ifndef __VXWORKS__
		if (pm_config.shortTermHistory.enable && !isFirstImg) {
			// compound the image that was just created into the current composite image
			IB_LOG_INFO_FMT(__func__, "compoundNewImage LastSweepIndex=%d, lastHistoryIndex=%d",pm->LastSweepIndex,pm->lastHistoryIndex);
			ret = compoundNewImage(pm);
			if (ret != FSUCCESS) {
				IB_LOG_WARNRC("Error while trying to compound new sweep image rc:", ret);
			}
		} else if (isFirstImg) {
			isFirstImg = FALSE;
		}

#endif
#endif	// End of #if CPU_LE

		// find next free Image to use, skip FreezeFrame images
		do {
			pm->SweepIndex = (pm->SweepIndex + 1) % pm_config.total_images;
			if (! pm->Image[pm->SweepIndex].ffRefCount)
				break;
			// check lease
			if (now_time > pm->Image[pm->SweepIndex].lastUsed &&
					now_time - pm->Image[pm->SweepIndex].lastUsed > pm_config.freeze_frame_lease) {
				pm->Image[pm->SweepIndex].ffRefCount = 0;
				// pa_access will clean up FreezeFrame on next access or FF Create
			}
			// skip past images which are frozen
		} while (pm->Image[pm->SweepIndex].ffRefCount);

		// Mark current image valid; mark next image in-progress
		pmimagep->state = PM_IMAGE_VALID;
		pm->Image[pm->SweepIndex].state = PM_IMAGE_INPROGRESS;
		// pm->NumSweeps follows image ID sweepNum; +1 anticipates becoming master before next image
		pm->NumSweeps = sweep_num + 1;

	}	// End of if (!skipCompounding)

	// Log sweep image statistics
	vs_pool_size(&pm_pool, &size);
	IB_LOG_INFO_FMT( __func__, "Image Received From Master: Sweep: %u Memory Used: %"PRIu64" MB (%"PRIu64" KB)",
		sweep_num, size / (1024*1024), size/1024 );
	// Additional debug output can be enabled
//	IB_LOG_VERBOSE_FMT( __func__, "... SweepIndex:%u LastSweepIndex:%u LastHistoryIndex:%u IsCompound:%u ImgSweN:%u swe_n:%u",
//		pm->SweepIndex, pm->LastSweepIndex, pm->lastHistoryIndex, !skipCompounding, pmimagep->sweepNum, sweep_num );
//	IB_LOG_VERBOSE_FMT( __func__, "... H[0]:%u H[1]:%u H[2]:%u H[3]:%u H[4]:%u H[5]:%u H[6]:%u H[7]:%u H[8]:%u H[9]:%u",
//		pm->history[0], pm->history[1], pm->history[2], pm->history[3], pm->history[4],
//		pm->history[5], pm->history[6], pm->history[7], pm->history[8], pm->history[9] );

exit_unlock:
	(void)vs_rwunlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);
//    IB_LOG_DEBUG1_FMT(__func__, " EXIT-UNLOCK: ret:0x%X", ret);
// Fall-through to exit_free

exit_free:
	if (bf_decompress)
		vs_pool_free(&pm_pool, bf_decompress);
	if (cimg_out)
		PmFreeComposite(cimg_out);

	return ret;

}	// End of putPMSweepImageDataR()

