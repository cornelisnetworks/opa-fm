/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */
#include "sm_l.h"
#include "pm_l.h"
#include "pa_l.h"
#include "iba/stl_pm.h"
#include "iba/stl_mad.h"
#include "iba/stl_pa.h"
#include <iba/ibt.h>
#include "pm_topology.h"
#include "paAccess.h"
#include <limits.h>
#include <time.h>

#undef LOCAL_MOD_ID
#define LOCAL_MOD_ID VIEO_PA_MOD_ID

extern uint8_t *pa_data;

extern Pm_t g_pmSweepData;

extern uint8_t pa_respTimeValue;

Status_t pa_cntxt_data(pa_cntxt_t* pa_cntxt, void* buf, uint32_t len);
Status_t pa_send_reply(Mai_t *maip, pa_cntxt_t* pa_cntxt);

Status_t
pa_getClassPortInfoResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	FSTATUS		status = FSUCCESS;
	STL_CLASS_PORT_INFO response = {0};

	IB_ENTER(__func__, maip, 0, 0, 0);
	
	INCREMENT_PM_COUNTER(pmCounterPaRxGetClassPortInfo);
	IB_LOG_DEBUG1_FMT(__func__, "Getting Class Port Info");

	if (status == FSUCCESS) {
		records = 1;
		response.BaseVersion			= STL_BASE_VERSION;
		response.ClassVersion			= STL_PA_CLASS_VERSION;
		response.CapMask			   |= STL_PA_CPI_CAPMASK_ABSTIMEQUERY;
		response.u1.s.RespTimeValue		= pa_respTimeValue;

		IB_LOG_DEBUG2_FMT(__func__, "Base Version:  0x%x", response.BaseVersion);
		IB_LOG_DEBUG2_FMT(__func__, "Class Version: 0x%x", response.ClassVersion);
		IB_LOG_DEBUG2_FMT(__func__, "Resp Time Value: %u", response.u1.s.RespTimeValue);

		BSWAP_STL_CLASS_PORT_INFO(&response);
		memcpy(data, &response, sizeof(response));
	}

    if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_CLASS_PORT_INFO) + Calculate_Padding(sizeof(STL_CLASS_PORT_INFO));
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);

    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);

}	// End of pa_getClassPortInfoResp()

Status_t
pa_getGroupListResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	responseSize = 0;
	uint32_t	attribOffset;
	FSTATUS		status;
	Status_t	vStatus;
	PmGroupList_t	GroupList = {0};
	STL_PA_GROUP_LIST *response = NULL;
	int			i;

	IB_ENTER(__func__, maip, 0, 0, 0);
	
	INCREMENT_PM_COUNTER(pmCounterPaRxGetGrpList);
	IB_LOG_DEBUG1_FMT(__func__, "Getting group list");

	status = paGetGroupList(&g_pmSweepData, &GroupList);
	if (status == FSUCCESS) {
		records = GroupList.NumGroups;
		responseSize = GroupList.NumGroups * STL_PM_GROUPNAMELEN;
		if (responseSize) {
			vStatus = vs_pool_alloc(&pm_pool, responseSize, (void*)&response);
			if (vStatus != VSTATUS_OK) {
				IB_LOG_ERRORRC("Failed to allocate response buffer for GroupList rc:", vStatus);
				status = FINSUFFICIENT_MEMORY;
				goto done;
			}
			memset(response, 0, responseSize);
		}

		IB_LOG_DEBUG2_FMT(__func__, "Number of groups: %u", GroupList.NumGroups);
		for (i = 0; i < GroupList.NumGroups; i++) {
    		strncpy(response[i].groupName, GroupList.GroupList[i].Name, STL_PM_GROUPNAMELEN-1);
			IB_LOG_DEBUG2_FMT(__func__, "Group %d: %.*s", i+1,
				(int)sizeof(response[i].groupName), response[i].groupName);
		}
    	memcpy(data, response, responseSize);
	}

done:
    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM Engine Not Running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating array failed
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// PM image parameter incorrect
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_GROUP_LIST) + Calculate_Padding(sizeof(STL_PA_GROUP_LIST));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);

	if (GroupList.GroupList != NULL)
		vs_pool_free(&pm_pool, GroupList.GroupList);
	if (response != NULL)
		vs_pool_free(&pm_pool, response);

	IB_EXIT(__func__, status);
	return(status);

}	// End of pa_getGroupListResp()


Status_t
pa_getPortCountersResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	STL_LID_32	nodeLid;
	PORT		portNumber;
	uint32_t	delta, userCntrs;
	uint32_t	flags;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS		status;
	PmCompositePortCounters_t pmPortCounters = {0};
    STL_PORT_COUNTERS_DATA response = {0};
    STL_PORT_COUNTERS_DATA *p = (STL_PORT_COUNTERS_DATA *)&maip->data[STL_PA_DATA_OFFSET];

	IB_ENTER(__func__, maip, 0, 0, 0);

	INCREMENT_PM_COUNTER(pmCounterPaRxGetPortCtrs);
	BSWAP_STL_PA_PORT_COUNTERS(p);
	nodeLid 	= p->nodeLid;
	portNumber 	= p->portNumber;
	delta 		= p->flags & STL_PA_PC_FLAG_DELTA;
	userCntrs 	= p->flags & STL_PA_PC_FLAG_USER_COUNTERS;

	IB_LOG_DEBUG1_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d", p->imageId.imageNumber, p->imageId.imageOffset);
	IB_LOG_DEBUG1_FMT(__func__, "LID: %u  Port Number: %u  Flags: 0x%x", nodeLid, portNumber, p->flags);

	status = paGetPortStats(&g_pmSweepData, nodeLid, portNumber, &pmPortCounters, delta, userCntrs, p->imageId, &flags, &retImageId);
	if (status == FSUCCESS) {
            records = 1;
            response.nodeLid                     = nodeLid;
            response.portNumber                  = portNumber;
            response.flags                       = flags;
            response.portXmitData                = pmPortCounters.PortXmitData;
            response.portRcvData                 = pmPortCounters.PortRcvData;
            response.portXmitPkts                = pmPortCounters.PortXmitPkts;
            response.portRcvPkts                 = pmPortCounters.PortRcvPkts;
            response.portMulticastXmitPkts       = pmPortCounters.PortMulticastXmitPkts;
            response.portMulticastRcvPkts        = pmPortCounters.PortMulticastRcvPkts;
            response.localLinkIntegrityErrors    = pmPortCounters.LocalLinkIntegrityErrors;
            response.fmConfigErrors              = pmPortCounters.FMConfigErrors;
            response.portRcvErrors               = pmPortCounters.PortRcvErrors;
            response.excessiveBufferOverruns     = pmPortCounters.ExcessiveBufferOverruns;
            response.portRcvConstraintErrors     = pmPortCounters.PortRcvConstraintErrors;
            response.portRcvSwitchRelayErrors    = pmPortCounters.PortRcvSwitchRelayErrors;
            response.portXmitDiscards            = pmPortCounters.PortXmitDiscards;
            response.portXmitConstraintErrors    = pmPortCounters.PortXmitConstraintErrors;
            response.portRcvRemotePhysicalErrors = pmPortCounters.PortRcvRemotePhysicalErrors;
            response.swPortCongestion            = pmPortCounters.SwPortCongestion;
            response.portXmitWait                = pmPortCounters.PortXmitWait;
            response.portRcvFECN                 = pmPortCounters.PortRcvFECN;
            response.portRcvBECN                 = pmPortCounters.PortRcvBECN;
            response.portXmitTimeCong            = pmPortCounters.PortXmitTimeCong;
            response.portXmitWastedBW            = pmPortCounters.PortXmitWastedBW;
            response.portXmitWaitData            = pmPortCounters.PortXmitWaitData;
            response.portRcvBubble               = pmPortCounters.PortRcvBubble;
            response.portMarkFECN                = pmPortCounters.PortMarkFECN;
            response.linkErrorRecovery           = pmPortCounters.LinkErrorRecovery;
            response.linkDowned                  = pmPortCounters.LinkDowned;
            response.uncorrectableErrors         = pmPortCounters.UncorrectableErrors;
            response.lq.s.numLanesDown           = pmPortCounters.lq.s.NumLanesDown;
            response.lq.s.linkQualityIndicator   = pmPortCounters.lq.s.LinkQualityIndicator;
            response.imageId = retImageId;
            response.imageId.imageOffset = 0;

		/* debug logging */
		IB_LOG_DEBUG2_FMT(__func__, "%s Controlled Port Counters (%s) for node lid 0x%x, port number %u%s:", (userCntrs?"User":"PM"),
			delta?"delta":"total", nodeLid, portNumber,
			flags & STL_PA_PC_FLAG_UNEXPECTED_CLEAR?" (Unexpected Clear)":"");
		IB_LOG_DEBUG2_FMT(__func__, "Xmit: Data:%-10" PRIu64 " MB (%10" PRIu64 " Flits) Pkts: %-10"PRIu64" ",
			response.portXmitData / FLITS_PER_MB, response.portXmitData, response.portXmitPkts);
		IB_LOG_DEBUG2_FMT(__func__, "Recv: Data:%-10" PRIu64 " MB (%10" PRIu64 " Flits) Pkts: %-10"PRIu64" ",
			response.portRcvData / FLITS_PER_MB, response.portRcvData, response.portRcvPkts);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Local Link Integ:   %10"PRIu64" ", response.localLinkIntegrityErrors);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: FM Cfg:             %10"PRIu64" ", response.fmConfigErrors);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Rcv Errors:         %10"PRIu64" ", response.portRcvErrors);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Exc Buffer Overrun: %10"PRIu64" ", response.excessiveBufferOverruns);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Rcv Constr:         %10"PRIu64" ", response.portRcvConstraintErrors);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Rcv Sw Rel:         %10"PRIu64" ", response.portRcvSwitchRelayErrors);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Xmt Disc:           %10"PRIu64" ", response.portXmitDiscards);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Xmt Constr:         %10"PRIu64" ", response.portXmitConstraintErrors);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Rcv Rmt Phys:       %10"PRIu64" ", response.portRcvRemotePhysicalErrors);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Cong Discards:      %10"PRIu64" ", response.swPortCongestion);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Xmit Wait:          %10"PRIu64" ", response.portXmitWait);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Rcv FECN:           %10"PRIu64" ", response.portRcvFECN);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Rcv BECN:           %10"PRIu64" ", response.portRcvBECN);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Xmit Time Cong:     %10"PRIu64" ", response.portXmitTimeCong);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Xmit Wasted BW:     %10"PRIu64" ", response.portXmitWastedBW);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Xmit Wait Data:     %10"PRIu64" ", response.portXmitWaitData);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Rcv Bubble:         %10"PRIu64" ", response.portRcvBubble);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Mark FECN:          %10"PRIu64" ", response.portMarkFECN);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Link Recovery:      %10u", response.linkErrorRecovery);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Link Downed:        %10u", response.linkDowned);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Link Recovery:      %10u", response.uncorrectableErrors);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Num Lanes Down:     %10u", response.lq.s.numLanesDown);
		IB_LOG_DEBUG2_FMT(__func__, "Errors: Link Qual Ind:      %10u", response.lq.s.linkQualityIndicator);


		time_t absTime = (time_t)response.imageId.imageTime.absoluteTime;
		char buf[80];

		if (absTime) {
			snprintf(buf, sizeof(buf), " Time %s", ctime((const time_t *)&absTime));
			if ((strlen(buf)>0) && (buf[strlen(buf)-1] == '\n'))
				buf[strlen(buf)-1] = '\0';
		}

		IB_LOG_DEBUG2_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d%s",
			response.imageId.imageNumber, response.imageId.imageOffset, buf);
		/* end debug logging */

    	BSWAP_STL_PA_PORT_COUNTERS(&response);
    	memcpy(data, &response, sizeof(response));
	}

    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM Engine Not Running
	} else if (status == (FUNAVAILABLE | STL_MAD_STATUS_STL_PA_UNAVAILABLE)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM nas not completed a sweep
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// PM image parameter incorrect
	} else if (status == (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;	// PA query parameter incorrect
	} else if (status == FNOT_FOUND) {
		if (retImageId.imageNumber == BAD_IMAGE_ID)
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;			// PM image could not be found
		else
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;			// PM Port Could not be found (unlikely)
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;				// Pm Port could not be found
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PORT_COUNTERS_DATA) + Calculate_Padding(sizeof(STL_PORT_COUNTERS_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_clrPortCountersResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	FSTATUS		status;
	STL_LID_32	nodeLid;
	PORT		portNumber;
	CounterSelectMask_t select;
    STL_CLR_PORT_COUNTERS_DATA response = {0};
    STL_CLR_PORT_COUNTERS_DATA *p = (STL_CLR_PORT_COUNTERS_DATA *)&maip->data[STL_PA_DATA_OFFSET];

	IB_ENTER(__func__, maip, 0, 0, 0);

	INCREMENT_PM_COUNTER(pmCounterPaRxClrPortCtrs);
	BSWAP_STL_PA_CLR_PORT_COUNTERS(p);
	nodeLid 		= p->NodeLid;
	portNumber 		= p->PortNumber;
	select.AsReg32 	= p->CounterSelectMask.AsReg32;

	IB_LOG_DEBUG1_FMT(__func__, "Clearing counters on LID: 0x%x  Port Number: %u  Select: 0x%4x",
		nodeLid, portNumber, select.AsReg32);

	status = paClearPortStats(&g_pmSweepData, nodeLid, portNumber, select);
	if (status == FSUCCESS) {
		records = 1;
		response.CounterSelectMask.AsReg32 	= select.AsReg32;
		response.NodeLid 					= nodeLid;
		response.PortNumber 				= portNumber;

		IB_LOG_DEBUG2_FMT(__func__, "LID:         0x%x", response.NodeLid);
		IB_LOG_DEBUG2_FMT(__func__, "Port Number: %u", response.PortNumber);
		IB_LOG_DEBUG2_FMT(__func__, "Select:      0x%4x", response.CounterSelectMask.AsReg32);

		BSWAP_STL_PA_CLR_PORT_COUNTERS(&response);
		memcpy(data, &response, sizeof(response));
	}

    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM Engine Not Running
	} else if (status == (FUNAVAILABLE | STL_MAD_STATUS_STL_PA_UNAVAILABLE)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM nas not completed a sweep
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// PM image parameter incorrect
	} else if (status == (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;	// PA query parameter incorrect
	} else if (status == FNOT_FOUND) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;				// PM LID Could not be found
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;				// Pm Port could not be found
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_CLR_PORT_COUNTERS_DATA) + Calculate_Padding(sizeof(STL_CLR_PORT_COUNTERS_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_clrAllPortCountersResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	FSTATUS		status;
	CounterSelectMask_t select;
    STL_CLR_ALL_PORT_COUNTERS_DATA response = {{0}};
    STL_CLR_ALL_PORT_COUNTERS_DATA *p = (STL_CLR_ALL_PORT_COUNTERS_DATA *)&maip->data[STL_PA_DATA_OFFSET];

	IB_ENTER(__func__, maip, 0, 0, 0);

    memset(&response, 0, sizeof(response));

	INCREMENT_PM_COUNTER(pmCounterPaRxClrAllPortCtrs);
    BSWAP_STL_PA_CLR_ALL_PORT_COUNTERS(p);
	select.AsReg32 = p->CounterSelectMask.AsReg32;

	IB_LOG_DEBUG1_FMT(__func__, "Clearing all counters with select 0x%4x", select.AsReg32);

	status = paClearAllPortStats(&g_pmSweepData, select);
	if (status == FSUCCESS) {
		records = 1;
		response.CounterSelectMask.AsReg32 = select.AsReg32;

		IB_LOG_DEBUG2_FMT(__func__, "Select: 0x%4x", response.CounterSelectMask.AsReg32);

		BSWAP_STL_PA_CLR_ALL_PORT_COUNTERS(&response);
		memcpy(data, &response, sizeof(response));
	}

	// determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM Engine Not Running
	} else if (status == (FUNAVAILABLE | STL_MAD_STATUS_STL_PA_UNAVAILABLE)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM nas not completed a sweep
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// PM image parameter incorrect
	} else if (status == (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;	// PA query parameter incorrect
	} else if (status == FNOT_FOUND) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;				// PM LID Could not be found
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;				// Pm Port could not be found
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_CLR_ALL_PORT_COUNTERS_DATA) + Calculate_Padding(sizeof(STL_CLR_ALL_PORT_COUNTERS_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_getPmConfigResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	FSTATUS		status = FSUCCESS;
	Status_t	vStatus;
	uint64_t	memoryFootprint;
    STL_PA_PM_CFG_DATA response = {0};

	IB_ENTER(__func__, maip, 0, 0, 0);

	INCREMENT_PM_COUNTER(pmCounterPaRxGetPmConfig);
	IB_LOG_DEBUG1_FMT(__func__, "Getting current PM Config");

	vStatus = vs_pool_size(&pm_pool, &memoryFootprint);
	if (vStatus != VSTATUS_OK) {
		IB_LOG_WARN_FMT(__func__, "Unable to get MemmoryFootPrint vStatus: %u", vStatus );
		memoryFootprint = (uint64)-1;
	}
	if (status == FSUCCESS) {
		records = 1;
		response.sweepInterval							  	= g_pmSweepData.interval;
		response.maxClients								  	= pm_config.max_clients;
		response.sizeHistory							  	= pm_config.total_images;
		response.sizeFreeze								  	= pm_config.freeze_frame_images;
		response.lease									  	= pm_config.freeze_frame_lease;
		response.pmFlags								  	= g_pmSweepData.flags;
		response.memoryFootprint						  	= memoryFootprint;
		response.maxAttempts							  	= pm_config.MaxRetries;
		response.respTimeout							  	= pm_config.RcvWaitInterval;
		response.minRespTimeout							  	= pm_config.MinRcvWaitInterval;
		response.maxParallelNodes						  	= pm_config.MaxParallelNodes;
		response.pmaBatchSize							  	= pm_config.PmaBatchSize;
		response.errorClear								  	= pm_config.ErrorClear;

		response.congestionWeights.PortXmitWait			  	= g_pmSweepData.congestionWeights.PortXmitWait;
		response.congestionWeights.SwPortCongestion		  	= g_pmSweepData.congestionWeights.SwPortCongestion;
		response.congestionWeights.PortRcvFECN			  	= g_pmSweepData.congestionWeights.PortRcvFECN;
		response.congestionWeights.PortRcvBECN			  	= g_pmSweepData.congestionWeights.PortRcvBECN;
		response.congestionWeights.PortXmitTimeCong		  	= g_pmSweepData.congestionWeights.PortXmitTimeCong;
		response.congestionWeights.PortMarkFECN			  	= g_pmSweepData.congestionWeights.PortMarkFECN;

		response.integrityWeights.LocalLinkIntegrityErrors = g_pmSweepData.integrityWeights.LocalLinkIntegrityErrors;
		response.integrityWeights.PortRcvErrors            = g_pmSweepData.integrityWeights.PortRcvErrors;
		response.integrityWeights.ExcessiveBufferOverruns  = g_pmSweepData.integrityWeights.ExcessiveBufferOverruns;
		response.integrityWeights.LinkErrorRecovery        = g_pmSweepData.integrityWeights.LinkErrorRecovery;
		response.integrityWeights.LinkDowned               = g_pmSweepData.integrityWeights.LinkDowned;
		response.integrityWeights.UncorrectableErrors      = g_pmSweepData.integrityWeights.UncorrectableErrors;
		response.integrityWeights.FMConfigErrors           = g_pmSweepData.integrityWeights.FMConfigErrors;
		response.integrityWeights.LinkQualityIndicator     = g_pmSweepData.integrityWeights.LinkQualityIndicator;
		response.integrityWeights.LinkWidthDowngrade       = g_pmSweepData.integrityWeights.LinkWidthDowngrade;

		response.categoryThresholds.integrityErrors		  	= g_pmSweepData.Thresholds.Integrity;
		response.categoryThresholds.congestion		  	= g_pmSweepData.Thresholds.Congestion;
		response.categoryThresholds.smaCongestion	  	= g_pmSweepData.Thresholds.SmaCongestion;
		response.categoryThresholds.bubble			  	= g_pmSweepData.Thresholds.Bubble;
		response.categoryThresholds.securityErrors			  	= g_pmSweepData.Thresholds.Security;
		response.categoryThresholds.routingErrors			  	= g_pmSweepData.Thresholds.Routing;

		IB_LOG_DEBUG2_FMT(__func__, "Sweep interval:     %u", response.sweepInterval);
		IB_LOG_DEBUG2_FMT(__func__, "Max Clients:        %u", response.maxClients);
		IB_LOG_DEBUG2_FMT(__func__, "History size:       %u", response.sizeHistory);
		IB_LOG_DEBUG2_FMT(__func__, "Freeze size:        %u", response.sizeFreeze);
		IB_LOG_DEBUG2_FMT(__func__, "Lease value:        %u", response.lease);
		IB_LOG_DEBUG2_FMT(__func__, "PM Flags:           0x%08x", response.pmFlags);
		IB_LOG_DEBUG2_FMT(__func__, "Mem footprint:      %"PRIu64" ", response.memoryFootprint);
		IB_LOG_DEBUG2_FMT(__func__, "Max Attempts:       %u", response.maxAttempts);
		IB_LOG_DEBUG2_FMT(__func__, "Resp Timeout:       %u", response.respTimeout);
		IB_LOG_DEBUG2_FMT(__func__, "Min Resp Timeout:   %u", response.minRespTimeout);
		IB_LOG_DEBUG2_FMT(__func__, "Max Parallel Nodes: %u", response.maxParallelNodes);
		IB_LOG_DEBUG2_FMT(__func__, "Pma Batch Size:     %u", response.pmaBatchSize);
		IB_LOG_DEBUG2_FMT(__func__, "Error Clear:        %u", response.errorClear);
		IB_LOG_DEBUG2_FMT(__func__, "Congestion weights:");
		IB_LOG_DEBUG2_FMT(__func__, "   XmitWait                  %u", response.congestionWeights.PortXmitWait);
		IB_LOG_DEBUG2_FMT(__func__, "   CongDiscards              %u", response.congestionWeights.SwPortCongestion);
		IB_LOG_DEBUG2_FMT(__func__, "   RcvFECN                   %u", response.congestionWeights.PortRcvFECN);
		IB_LOG_DEBUG2_FMT(__func__, "   RcvBECN                   %u", response.congestionWeights.PortRcvBECN);
		IB_LOG_DEBUG2_FMT(__func__, "   XmitTimeCong              %u", response.congestionWeights.PortXmitTimeCong);
		IB_LOG_DEBUG2_FMT(__func__, "   MarkFECN                  %u", response.congestionWeights.PortMarkFECN);
		IB_LOG_DEBUG2_FMT(__func__, "Integrity weights:");
		IB_LOG_DEBUG2_FMT(__func__, "   LocalLinkIntegrityErrors  %u", response.integrityWeights.LocalLinkIntegrityErrors);
		IB_LOG_DEBUG2_FMT(__func__, "   RcvErrors                 %u", response.integrityWeights.PortRcvErrors);
		IB_LOG_DEBUG2_FMT(__func__, "   ExcessiveBufferOverruns   %u", response.integrityWeights.ExcessiveBufferOverruns);
		IB_LOG_DEBUG2_FMT(__func__, "   LinkErrorRecovery         %u", response.integrityWeights.LinkErrorRecovery);
		IB_LOG_DEBUG2_FMT(__func__, "   LinkDowned                %u", response.integrityWeights.LinkDowned);
		IB_LOG_DEBUG2_FMT(__func__, "   UncorrectableErrors       %u", response.integrityWeights.UncorrectableErrors);
		IB_LOG_DEBUG2_FMT(__func__, "   FMConfigErrors            %u", response.integrityWeights.FMConfigErrors);
		IB_LOG_DEBUG2_FMT(__func__, "   LinkQualityIndicator      %u", response.integrityWeights.LinkQualityIndicator);
		IB_LOG_DEBUG2_FMT(__func__, "   LinkWidthDowngrade        %u", response.integrityWeights.LinkWidthDowngrade);
		IB_LOG_DEBUG2_FMT(__func__, "Category thresholds:");
		IB_LOG_DEBUG2_FMT(__func__, "   Integrity:                %u", response.categoryThresholds.integrityErrors);
		IB_LOG_DEBUG2_FMT(__func__, "   Congestion:               %u", response.categoryThresholds.congestion);
		IB_LOG_DEBUG2_FMT(__func__, "   SmaCongestion:            %u", response.categoryThresholds.smaCongestion);
		IB_LOG_DEBUG2_FMT(__func__, "   Bubble:                   %u", response.categoryThresholds.bubble);
		IB_LOG_DEBUG2_FMT(__func__, "   Security:                 %u", response.categoryThresholds.securityErrors);
		IB_LOG_DEBUG2_FMT(__func__, "   Routing:                  %u", response.categoryThresholds.routingErrors);

		BSWAP_STL_PA_PM_CFG(&response);
		memcpy(data, &response, sizeof(response));
	}

    // determine reply status
	if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_PM_CFG_DATA) + Calculate_Padding(sizeof(STL_PA_PM_CFG_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_freezeImageResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	STL_PA_IMAGE_ID_DATA freezeImage = {0};
	FSTATUS		status;
    STL_PA_IMAGE_ID_DATA response = {0};
	STL_PA_IMAGE_ID_DATA *p = (STL_PA_IMAGE_ID_DATA *)&maip->data[STL_PA_DATA_OFFSET];

	IB_ENTER(__func__, maip, 0, 0, 0);

	INCREMENT_PM_COUNTER(pmCounterPaRxFreezeImage);
	BSWAP_STL_PA_IMAGE_ID(p);

	IB_LOG_DEBUG1_FMT(__func__, "Freezing ImageID: Number 0x%"PRIx64" Offset %d", p->imageNumber,
			                                                                      p->imageOffset);

	status = paFreezeFrameCreate(&g_pmSweepData, *p, &freezeImage);
	if (status == FSUCCESS) {
		records = 1;
		response = freezeImage;
		response.imageOffset = 0;

		time_t absTime = (time_t)response.imageTime.absoluteTime;
		char buf[80];

		if (absTime) {
			snprintf(buf, sizeof(buf), " Time %s", ctime((const time_t *)&absTime));
			if ((strlen(buf)>0) && (buf[strlen(buf)-1] == '\n'))
				buf[strlen(buf)-1] = '\0';
		}

		IB_LOG_DEBUG2_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d%s frozen successfully",
			freezeImage.imageNumber, freezeImage.imageOffset, buf);
    	BSWAP_STL_PA_IMAGE_ID(&response);
    	memcpy(data, &response, sizeof(response));
	} else {
		IB_LOG_WARN_FMT(__func__, "Error freezing ImageID: Number 0x%"PRIx64" Offset %d  status %s (%u)",
			freezeImage.imageNumber, freezeImage.imageOffset, iba_fstatus_msg(status), status);
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
	}

    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating failed
	} else if (status == (FINSUFFICIENT_MEMORY | STL_MAD_STATUS_STL_PA_NO_IMAGE)) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// Uanable to Freeze image (limit reached)
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;				// Image Not Found
	} else if (status == FNOT_FOUND) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;				// Image Not Found
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_IMAGE_ID_DATA) + Calculate_Padding(sizeof(STL_PA_IMAGE_ID_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_releaseImageResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	FSTATUS		status;
	STL_PA_IMAGE_ID_DATA imageId = {0};
    STL_PA_IMAGE_ID_DATA response = {0};
	STL_PA_IMAGE_ID_DATA *p = (STL_PA_IMAGE_ID_DATA *)&maip->data[STL_PA_DATA_OFFSET];

	IB_ENTER(__func__, maip, 0, 0, 0);

	INCREMENT_PM_COUNTER(pmCounterPaRxReleaseImage);
	BSWAP_STL_PA_IMAGE_ID(p);
	imageId = *p;

	/* ***** Error checks on imageNumber and imageOffset here **** */

	IB_LOG_DEBUG1_FMT(__func__, "Releasing ImageID: Number 0x%"PRIx64" Offset %d",
		imageId.imageNumber, imageId.imageOffset);

	status = paFreezeFrameRelease(&g_pmSweepData, &imageId);
	if (status == FSUCCESS) {
		records = 1;
		response = imageId;

		time_t absTime = (time_t)response.imageTime.absoluteTime;
		char buf[80];

		if (absTime) {
			snprintf(buf, sizeof(buf), " Time %s", ctime((const time_t *)&absTime));
			if ((strlen(buf)>0) && (buf[strlen(buf)-1] == '\n'))
				buf[strlen(buf)-1] = '\0';
		}

		IB_LOG_DEBUG2_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d%s released successfully",
			imageId.imageNumber, imageId.imageOffset, buf);
    	BSWAP_STL_PA_IMAGE_ID(&response);
    	memcpy(data, &response, sizeof(response));
	} else {
		IB_LOG_WARN_FMT(__func__, "Error releasing ImageID: Number 0x%"PRIx64" Offset %d  status %s (%u)",
			imageId.imageNumber, imageId.imageOffset, iba_fstatus_msg(status), status);
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
	}

    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating failed
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;				// Image Not Found
	} else if (status == FNOT_FOUND) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;				// Image Not Found
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_IMAGE_ID_DATA) + Calculate_Padding(sizeof(STL_PA_IMAGE_ID_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_renewImageResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	FSTATUS		status;
	STL_PA_IMAGE_ID_DATA imageId = {0};
    STL_PA_IMAGE_ID_DATA response = {0};
	STL_PA_IMAGE_ID_DATA *p = (STL_PA_IMAGE_ID_DATA *)&maip->data[STL_PA_DATA_OFFSET];

	IB_ENTER(__func__, maip, 0, 0, 0);

	INCREMENT_PM_COUNTER(pmCounterPaRxRenewImage);
	BSWAP_STL_PA_IMAGE_ID(p);
	imageId = *p;
	/* ***** Error checks on imageNumber and imageOffset here **** */

	IB_LOG_DEBUG1_FMT(__func__, "Renewing ImageID: Number 0x%"PRIx64" Offset %d",
		imageId.imageNumber, imageId.imageOffset);

	status = paFreezeFrameRenew(&g_pmSweepData, &imageId);
	if (status == FSUCCESS) {
		records = 1;
		response = imageId;
		response.imageOffset = 0;

		time_t absTime = (time_t)response.imageTime.absoluteTime;
		char buf[80];

		if (absTime) {
			snprintf(buf, sizeof(buf), " Time %s", ctime((const time_t *)&absTime));
			if ((strlen(buf)>0) && (buf[strlen(buf)-1] == '\n'))
				buf[strlen(buf)-1] = '\0';
		}

		IB_LOG_DEBUG2_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d%s renewed successfully",
			imageId.imageNumber, imageId.imageOffset, buf);
    	BSWAP_STL_PA_IMAGE_ID(&response);
    	memcpy(data, &response, sizeof(response));
	} else {
		IB_LOG_WARN_FMT(__func__, "Error renewing ImageID: Number 0x%"PRIx64" Offset %d  status %s (%u)",
			imageId.imageNumber, imageId.imageOffset, iba_fstatus_msg(status), status);
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
	}

    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating failed
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;				// Image Not Found
	} else if (status == FNOT_FOUND) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;				// Image Not Found
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_IMAGE_ID_DATA) + Calculate_Padding(sizeof(STL_PA_IMAGE_ID_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_moveFreezeImageResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	FSTATUS		status;
	STL_PA_IMAGE_ID_DATA retImageIdNew = {0};
    STL_MOVE_FREEZE_DATA response = {{0}};
	STL_MOVE_FREEZE_DATA *p = (STL_MOVE_FREEZE_DATA *)&maip->data[STL_PA_DATA_OFFSET];

	IB_ENTER(__func__, maip, 0, 0, 0);

	INCREMENT_PM_COUNTER(pmCounterPaRxMoveFreezeFrame);
	BSWAP_STL_PA_MOVE_FREEZE(p);

	/* ***** Error checks on imageNumber and imageOffset here **** */

	IB_LOG_DEBUG1_FMT(__func__, "Moving frozen ImageID: Number 0x%"PRIx64" Offset %d - to Offset %d",
					  p->oldFreezeImage.imageNumber, p->oldFreezeImage.imageOffset,
					  p->newFreezeImage.imageOffset);

	status = paFreezeFrameMove(&g_pmSweepData, p->oldFreezeImage, p->newFreezeImage, &retImageIdNew);
	if (status == FSUCCESS) {
		records = 1;
		response.newFreezeImage = retImageIdNew;
		response.newFreezeImage.imageOffset = 0;

		response.oldFreezeImage.imageNumber = p->oldFreezeImage.imageNumber;
		response.oldFreezeImage.imageOffset = 0;

		IB_LOG_DEBUG2_FMT(__func__, " ImageID: Number 0x%"PRIx64" Offset %d moved successfully - to 0x%"PRIx64" ",
						  p->oldFreezeImage.imageNumber, p->oldFreezeImage.imageOffset, retImageIdNew.imageNumber);
		BSWAP_STL_PA_MOVE_FREEZE(&response);
		memcpy(data, &response, sizeof(response));
	} else {
		IB_LOG_DEBUG2_FMT(__func__, "Error moving freeze ImageID: Number 0x%"PRIx64" Offset %d  status %s (%u)",
						  p->oldFreezeImage.imageNumber, p->oldFreezeImage.imageOffset, iba_fstatus_msg(status), status);
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
	}

    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating array failed
	} else if (status == (FINSUFFICIENT_MEMORY | STL_MAD_STATUS_STL_PA_NO_IMAGE)) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// Uanable to Freeze image (limit reached)
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;				// Image Not Found
	} else if (status == FNOT_FOUND) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;				// Image Not Found
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_MOVE_FREEZE_DATA) + Calculate_Padding(sizeof(STL_MOVE_FREEZE_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_getImageInfoResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	int			i;
	FSTATUS		status;
    STL_PA_IMAGE_INFO_DATA response = {{0}};
	STL_PA_IMAGE_INFO_DATA *p = (STL_PA_IMAGE_INFO_DATA *)&maip->data[STL_PA_DATA_OFFSET];
	PmImageInfo_t	imageInfo = {0};

	IB_ENTER(__func__, maip, 0, 0, 0);

	INCREMENT_PM_COUNTER(pmCounterPaRxGetImageInfo);
	BSWAP_STL_PA_IMAGE_INFO(p);

	IB_LOG_DEBUG1_FMT(__func__, "Getting Image config");
	IB_LOG_DEBUG1_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d", p->imageId.imageNumber, p->imageId.imageOffset);

	status = paGetImageInfo(&g_pmSweepData, p->imageId, &imageInfo, &retImageId);
	if (status == FSUCCESS) {
		records = 1;
		response.imageId = retImageId;
		response.imageId.imageOffset     = 0;
		response.sweepStart              = imageInfo.sweepStart;
		response.sweepDuration           = imageInfo.sweepDuration;
		response.numHFIPorts             = imageInfo.numHFIPorts;
		response.numSwitchNodes          = imageInfo.numSwitchNodes;
		response.numSwitchPorts          = imageInfo.numSwitchPorts;
		response.numLinks                = imageInfo.numLinks;
		response.numSMs                  = imageInfo.numSMs;
		response.numNoRespNodes          = imageInfo.numNoRespNodes;
		response.numNoRespPorts          = imageInfo.numNoRespPorts;
		response.numSkippedNodes         = imageInfo.numSkippedNodes;
		response.numSkippedPorts         = imageInfo.numSkippedPorts;
		response.numUnexpectedClearPorts = imageInfo.numUnexpectedClearPorts;
		response.imageInterval           = imageInfo.imageInterval;
		for (i = 0; i < 2; i++) {
			response.SMInfo[i].lid				= imageInfo.SMInfo[i].smLid;
			response.SMInfo[i].portNumber		= imageInfo.SMInfo[i].portNumber;
			response.SMInfo[i].priority			= imageInfo.SMInfo[i].priority;
			response.SMInfo[i].state			= imageInfo.SMInfo[i].state;
			response.SMInfo[i].smPortGuid		= imageInfo.SMInfo[i].smPortGuid;
			strncpy(response.SMInfo[i].smNodeDesc, imageInfo.SMInfo[i].smNodeDesc,
				sizeof(response.SMInfo[i].smNodeDesc)-1);
		}

		/* debug logging */
		time_t absTime = (time_t)response.imageId.imageTime.absoluteTime;
		char buf[80];

		if (absTime) {
			snprintf(buf, sizeof(buf), " Time %s", ctime((const time_t *)&absTime));
			if ((strlen(buf)>0) && (buf[strlen(buf)-1] == '\n'))
				buf[strlen(buf)-1] = '\0';
		}

		IB_LOG_DEBUG2_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d%s",
			response.imageId.imageNumber, response.imageId.imageOffset, buf);
		IB_LOG_DEBUG2_FMT(__func__, "  Sweep Start   %"PRIu64"     Sweep Duration %u",
						response.sweepStart, response.sweepDuration);
		IB_LOG_DEBUG2_FMT(__func__, "  HFI Ports     %u",
						response.numHFIPorts);
		IB_LOG_DEBUG2_FMT(__func__, "  Switch Nodes  %u", response.numSwitchNodes);
		IB_LOG_DEBUG2_FMT(__func__, "  Switch Ports  %u      SMs            %u",
						response.numSwitchPorts, response.numSMs);
		IB_LOG_DEBUG2_FMT(__func__, "  Links         %u      NoResp Nodes   %u",
						response.numLinks, response.numNoRespNodes);
		IB_LOG_DEBUG2_FMT(__func__, "  NoResp Ports  %u      Skipped Nodes  %u",
						response.numNoRespPorts, response.numSkippedNodes);
		IB_LOG_DEBUG2_FMT(__func__, "  Skipped Ports %u", response.numSkippedPorts);
		IB_LOG_DEBUG2_FMT(__func__, "  Unexpected Clear Ports %u", response.numUnexpectedClearPorts);
		IB_LOG_DEBUG2_FMT(__func__, "  Image Interval %u sec", response.imageInterval);
		for (i = 0; i < 2; i++) {
			IB_LOG_DEBUG2_FMT(__func__, "  SM %u", i+1);
			IB_LOG_DEBUG2_FMT(__func__, "     Lid       0x%x", response.SMInfo[i].lid);
			IB_LOG_DEBUG2_FMT(__func__, "     Port Num  %u", response.SMInfo[i].portNumber);
			IB_LOG_DEBUG2_FMT(__func__, "     Priority  %u", response.SMInfo[i].priority);
			IB_LOG_DEBUG2_FMT(__func__, "     State     %u", response.SMInfo[i].state);
			IB_LOG_DEBUG2_FMT(__func__, "     PortGuid %016"PRIx64" ", response.SMInfo[i].smPortGuid);
			IB_LOG_DEBUG2_FMT(__func__, "     Node Desc %.*s",
				(int)sizeof(response.SMInfo[i].smNodeDesc), response.SMInfo[i].smNodeDesc);
		}
		BSWAP_STL_PA_IMAGE_INFO(&response);
    	memcpy(data, &response, sizeof(response));
	}

    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// NULL pointer passed to function
	} else if (status == FNOT_FOUND) {
		if (retImageId.imageNumber == BAD_IMAGE_ID)
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;			// Failed to access/find image
		else
			maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;		// Failed to acces PM
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_IMAGE_INFO_DATA) + Calculate_Padding(sizeof(STL_PA_IMAGE_INFO_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_getGroupInfoResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	char		groupName[STL_PM_GROUPNAMELEN];
	uint32_t	records = 0;
	uint32_t	attribOffset;
	Status_t	vStatus;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS		status;
	PmGroupInfo_t groupInfo = {{0}};
	char		logBuf[80];
	char		*p1;
	STL_PA_PM_GROUP_INFO_DATA *response = NULL;
	STL_PA_PM_GROUP_INFO_DATA *p = (STL_PA_PM_GROUP_INFO_DATA *)&maip->data[STL_PA_DATA_OFFSET];
	uint32		responseSize = 0;
	int 		i;

	IB_ENTER(__func__, maip, 0, 0, 0);

	INCREMENT_PM_COUNTER(pmCounterPaRxGetGrpInfo);
	BSWAP_STL_PA_PM_GROUP_INFO(p, 1);
	strncpy(groupName, p->groupName, STL_PM_GROUPNAMELEN-1);
	groupName[STL_PM_GROUPNAMELEN-1]=0;

	IB_LOG_DEBUG1_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d", p->imageId.imageNumber, p->imageId.imageOffset);
	IB_LOG_DEBUG1_FMT(__func__, "Group: %.*s", (int)sizeof(groupName), groupName);

	status = paGetGroupInfo(&g_pmSweepData, groupName, &groupInfo, p->imageId, &retImageId);
	if (status == FSUCCESS) {
		records = 1;
		responseSize = records * sizeof(STL_PA_PM_GROUP_INFO_DATA);
		if (responseSize) {
			vStatus = vs_pool_alloc(&pm_pool, responseSize, (void*)&response);
			if (vStatus != VSTATUS_OK) {
				IB_LOG_ERRORRC("Failed to allocate response buffer for GroupInfo rc:", vStatus);
				status = FINSUFFICIENT_MEMORY;
				goto done;
			}
			memset(response, 0, responseSize);
		}

    	strncpy(response[0].groupName, groupName, STL_PM_GROUPNAMELEN-1);
		response[0].numInternalPorts							= groupInfo.NumIntPorts;
		response[0].numExternalPorts							= groupInfo.NumExtPorts;
		response[0].internalUtilStats.totalMBps					= groupInfo.IntUtil.TotMBps;
		response[0].internalUtilStats.totalKPps					= groupInfo.IntUtil.TotKPps;
		response[0].internalUtilStats.avgMBps					= groupInfo.IntUtil.AvgMBps;
		response[0].internalUtilStats.minMBps					= groupInfo.IntUtil.MinMBps;
		response[0].internalUtilStats.maxMBps					= groupInfo.IntUtil.MaxMBps;
		response[0].internalUtilStats.numBWBuckets				= STL_PM_UTIL_BUCKETS;
		for (i = 0; i < STL_PM_UTIL_BUCKETS; i++) {
			response[0].internalUtilStats.BWBuckets[i]			= groupInfo.IntUtil.BwPorts[i];
		}
		response[0].internalUtilStats.avgKPps					= groupInfo.IntUtil.AvgKPps;
		response[0].internalUtilStats.minKPps					= groupInfo.IntUtil.MinKPps;
		response[0].internalUtilStats.maxKPps					= groupInfo.IntUtil.MaxKPps;
		response[0].internalUtilStats.pmaNoRespPorts			= groupInfo.IntUtil.pmaNoRespPorts;
		response[0].internalUtilStats.topoIncompPorts			= groupInfo.IntUtil.topoIncompPorts;

		response[0].sendUtilStats.totalMBps						= groupInfo.SendUtil.TotMBps;
		response[0].sendUtilStats.totalKPps						= groupInfo.SendUtil.TotKPps;
		response[0].sendUtilStats.avgMBps						= groupInfo.SendUtil.AvgMBps;
		response[0].sendUtilStats.minMBps						= groupInfo.SendUtil.MinMBps;
		response[0].sendUtilStats.maxMBps						= groupInfo.SendUtil.MaxMBps;
		response[0].sendUtilStats.numBWBuckets					= STL_PM_UTIL_BUCKETS;
		for (i = 0; i < STL_PM_UTIL_BUCKETS; i++) {
			response[0].sendUtilStats.BWBuckets[i]				= groupInfo.SendUtil.BwPorts[i];
		}
		response[0].sendUtilStats.avgKPps						= groupInfo.SendUtil.AvgKPps;
		response[0].sendUtilStats.minKPps						= groupInfo.SendUtil.MinKPps;
		response[0].sendUtilStats.maxKPps						= groupInfo.SendUtil.MaxKPps;
		response[0].sendUtilStats.pmaNoRespPorts				= groupInfo.SendUtil.pmaNoRespPorts;
		response[0].sendUtilStats.topoIncompPorts				= groupInfo.SendUtil.topoIncompPorts;

		response[0].recvUtilStats.totalMBps						= groupInfo.RecvUtil.TotMBps;
		response[0].recvUtilStats.totalKPps						= groupInfo.RecvUtil.TotKPps;
		response[0].recvUtilStats.avgMBps						= groupInfo.RecvUtil.AvgMBps;
		response[0].recvUtilStats.minMBps						= groupInfo.RecvUtil.MinMBps;
		response[0].recvUtilStats.maxMBps						= groupInfo.RecvUtil.MaxMBps;
		response[0].recvUtilStats.numBWBuckets					= STL_PM_UTIL_BUCKETS;
		for (i = 0; i < STL_PM_UTIL_BUCKETS; i++) {
			response[0].recvUtilStats.BWBuckets[i]				= groupInfo.RecvUtil.BwPorts[i];
		}
		response[0].recvUtilStats.avgKPps						= groupInfo.RecvUtil.AvgKPps;
		response[0].recvUtilStats.minKPps						= groupInfo.RecvUtil.MinKPps;
		response[0].recvUtilStats.maxKPps						= groupInfo.RecvUtil.MaxKPps;
		response[0].recvUtilStats.pmaNoRespPorts				= groupInfo.RecvUtil.pmaNoRespPorts;
		response[0].recvUtilStats.topoIncompPorts				= groupInfo.RecvUtil.topoIncompPorts;

		/* Internal Error */
		response[0].internalCategoryStats.categoryMaximums.integrityErrors		= groupInfo.IntErr.Max.Integrity;
		response[0].internalCategoryStats.categoryMaximums.congestion		= groupInfo.IntErr.Max.Congestion;
		response[0].internalCategoryStats.categoryMaximums.smaCongestion 	= groupInfo.IntErr.Max.SmaCongestion;
		response[0].internalCategoryStats.categoryMaximums.bubble			= groupInfo.IntErr.Max.Bubble;
		response[0].internalCategoryStats.categoryMaximums.securityErrors			= groupInfo.IntErr.Max.Security;
		response[0].internalCategoryStats.categoryMaximums.routingErrors			= groupInfo.IntErr.Max.Routing;

		response[0].internalCategoryStats.categoryMaximums.utilizationPct10 = groupInfo.IntErr.Max.UtilizationPct10;
		response[0].internalCategoryStats.categoryMaximums.discardsPct10    = groupInfo.IntErr.Max.DiscardsPct10;

		for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
			response[0].internalCategoryStats.ports[i].integrityErrors		= groupInfo.IntErr.Ports[i].Integrity;
			response[0].internalCategoryStats.ports[i].congestion	= groupInfo.IntErr.Ports[i].Congestion;
			response[0].internalCategoryStats.ports[i].smaCongestion	= groupInfo.IntErr.Ports[i].SmaCongestion;
			response[0].internalCategoryStats.ports[i].bubble		= groupInfo.IntErr.Ports[i].Bubble;
			response[0].internalCategoryStats.ports[i].securityErrors		= groupInfo.IntErr.Ports[i].Security;
			response[0].internalCategoryStats.ports[i].routingErrors		= groupInfo.IntErr.Ports[i].Routing;
		}
		/* External Errors */
		response[0].externalCategoryStats.categoryMaximums.integrityErrors     = groupInfo.ExtErr.Max.Integrity;
		response[0].externalCategoryStats.categoryMaximums.congestion    = groupInfo.ExtErr.Max.Congestion;
		response[0].externalCategoryStats.categoryMaximums.smaCongestion = groupInfo.ExtErr.Max.SmaCongestion;
		response[0].externalCategoryStats.categoryMaximums.bubble        = groupInfo.ExtErr.Max.Bubble;
		response[0].externalCategoryStats.categoryMaximums.securityErrors      = groupInfo.ExtErr.Max.Security;
		response[0].externalCategoryStats.categoryMaximums.routingErrors       = groupInfo.ExtErr.Max.Routing;

		response[0].externalCategoryStats.categoryMaximums.utilizationPct10 = groupInfo.ExtErr.Max.UtilizationPct10;
		response[0].externalCategoryStats.categoryMaximums.discardsPct10    = groupInfo.ExtErr.Max.DiscardsPct10;

		for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
			response[0].externalCategoryStats.ports[i].integrityErrors		= groupInfo.ExtErr.Ports[i].Integrity;
			response[0].externalCategoryStats.ports[i].congestion	= groupInfo.ExtErr.Ports[i].Congestion;
			response[0].externalCategoryStats.ports[i].smaCongestion	= groupInfo.ExtErr.Ports[i].SmaCongestion;
			response[0].externalCategoryStats.ports[i].bubble		= groupInfo.ExtErr.Ports[i].Bubble;
			response[0].externalCategoryStats.ports[i].securityErrors		= groupInfo.ExtErr.Ports[i].Security;
			response[0].externalCategoryStats.ports[i].routingErrors		= groupInfo.ExtErr.Ports[i].Routing;
		}
		response[0].minInternalRate = groupInfo.MinIntRate;
		response[0].maxInternalRate = groupInfo.MaxIntRate;
		response[0].minExternalRate = groupInfo.MinExtRate;
		response[0].maxExternalRate = groupInfo.MaxExtRate;
		response[0].maxInternalMBps = StlStaticRateToMBps(response[0].maxInternalRate);
		response[0].maxExternalMBps = StlStaticRateToMBps(response[0].maxExternalRate);

		response[0].imageId = retImageId;
		response[0].imageId.imageOffset = 0;

		if (IB_LOG_IS_INTERESTED(VS_LOG_DEBUG2)) {
			IB_LOG_DEBUG2_FMT(__func__, "Group name %.*s", (int)sizeof(response[0].groupName), response[0].groupName);
			IB_LOG_DEBUG2_FMT(__func__, "Number of internal ports: %u", response[0].numInternalPorts);
			IB_LOG_DEBUG2_FMT(__func__, "Number of external ports: %u", response[0].numExternalPorts);
			IB_LOG_DEBUG2_FMT(__func__, "Internal utilization statistics:");
			IB_LOG_DEBUG2_FMT(__func__, "  Util: Tot %6"PRIu64" Max %6u Min %6u Avg %6u MB/s",
				response[0].internalUtilStats.totalMBps, response[0].internalUtilStats.maxMBps,
				response[0].internalUtilStats.minMBps, response[0].internalUtilStats.avgMBps);
			p1 = logBuf;
			p1 += sprintf(p1, "%s", "  Util: ");
			for (i = 0; i < STL_PM_UTIL_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalUtilStats.BWBuckets[i]);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			IB_LOG_DEBUG2_FMT(__func__, "  Pkts: Tot %6"PRIu64" Max %6u Min %6u Avg %6u KP/s",
				response[0].internalUtilStats.totalKPps, response[0].internalUtilStats.maxKPps,
				response[0].internalUtilStats.minKPps, response[0].internalUtilStats.avgKPps);
			IB_LOG_DEBUG2_FMT(__func__, "  NoResp Ports PMA: %u    NoResp Ports Topology: %u",
				response[0].internalUtilStats.pmaNoRespPorts, response[0].internalUtilStats.topoIncompPorts);
			IB_LOG_DEBUG2_FMT(__func__, "Send utilization statistics:");
			IB_LOG_DEBUG2_FMT(__func__, "  Util: Tot %6"PRIu64" Max %6u Min %6u Avg %6u MB/s",
				response[0].sendUtilStats.totalMBps, response[0].sendUtilStats.maxMBps,
				response[0].sendUtilStats.minMBps, response[0].sendUtilStats.avgMBps);
			p1 = logBuf;
			p1 += sprintf(p1, "%s", "  Util: ");
			for (i = 0; i < STL_PM_UTIL_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].sendUtilStats.BWBuckets[i]);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			IB_LOG_DEBUG2_FMT(__func__, "  Pkts: Tot %6"PRIu64" Max %6u Min %6u Avg %6u KP/s",
				response[0].sendUtilStats.totalKPps, response[0].sendUtilStats.maxKPps,
				response[0].sendUtilStats.minKPps, response[0].sendUtilStats.avgKPps);
			IB_LOG_DEBUG2_FMT(__func__, "  NoResp Ports PMA: %u    NoResp Ports Topology: %u",
				response[0].sendUtilStats.pmaNoRespPorts, response[0].sendUtilStats.topoIncompPorts);
			IB_LOG_DEBUG2_FMT(__func__, "Receive utilization statistics:");
			IB_LOG_DEBUG2_FMT(__func__, "  Util: Tot %6"PRIu64" Max %6u Min %6u Avg %6u MB/s",
				response[0].recvUtilStats.totalMBps, response[0].recvUtilStats.maxMBps,
				response[0].recvUtilStats.minMBps, response[0].recvUtilStats.avgMBps);
			p1 = logBuf;
			p1 += sprintf(p1, "%s", "  Util: ");
			for (i = 0; i < STL_PM_UTIL_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].recvUtilStats.BWBuckets[i]);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			IB_LOG_DEBUG2_FMT(__func__, "  Pkts: Tot %6"PRIu64" Max %6u Min %6u Avg %6u KP/s",
				response[0].recvUtilStats.totalKPps, response[0].recvUtilStats.maxKPps,
				response[0].recvUtilStats.minKPps, response[0].recvUtilStats.avgKPps);
			IB_LOG_DEBUG2_FMT(__func__, "  NoResp Ports PMA: %u    NoResp Ports Topology: %u",
				response[0].recvUtilStats.pmaNoRespPorts, response[0].recvUtilStats.topoIncompPorts);
			IB_LOG_DEBUG2_FMT(__func__, "Internal Category Summary:");
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Integrity       Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.integrityErrors);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].integrityErrors);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Congestion      Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.congestion);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].congestion);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg SmaCongestion   Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.smaCongestion);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].smaCongestion);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Bubble          Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.bubble);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].bubble);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Security        Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.securityErrors);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].securityErrors);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Routing         Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.routingErrors);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].routingErrors);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			IB_LOG_DEBUG2_FMT(__func__,"    Utilization: %3u.%1u%%",
							  response[0].internalCategoryStats.categoryMaximums.utilizationPct10 / 10,
							  response[0].internalCategoryStats.categoryMaximums.utilizationPct10 % 10);
			IB_LOG_DEBUG2_FMT(__func__,"    Discards: %3u.%1u%%",
							  response[0].internalCategoryStats.categoryMaximums.discardsPct10 / 10,
							  response[0].internalCategoryStats.categoryMaximums.discardsPct10 % 10);
			IB_LOG_DEBUG2_FMT(__func__, "External Category Summary:");
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Integrity       Max %10u Buckets: ", response[0].externalCategoryStats.categoryMaximums.integrityErrors);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].externalCategoryStats.ports[i].integrityErrors);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Congestion      Max %10u Buckets: ", response[0].externalCategoryStats.categoryMaximums.congestion);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].externalCategoryStats.ports[i].congestion);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg SmaCongestion   Max %10u Buckets: ", response[0].externalCategoryStats.categoryMaximums.smaCongestion);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].externalCategoryStats.ports[i].smaCongestion);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Bubble          Max %10u Buckets: ", response[0].externalCategoryStats.categoryMaximums.bubble);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].externalCategoryStats.ports[i].bubble);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Security        Max %10u Buckets: ", response[0].externalCategoryStats.categoryMaximums.securityErrors);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].externalCategoryStats.ports[i].securityErrors);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Routing         Max %10u Buckets: ", response[0].externalCategoryStats.categoryMaximums.routingErrors);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].externalCategoryStats.ports[i].routingErrors);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);

			IB_LOG_DEBUG2_FMT(__func__,"    Utilization: %3u.%1u%%",
							  response[0].externalCategoryStats.categoryMaximums.utilizationPct10 / 10,
							  response[0].externalCategoryStats.categoryMaximums.utilizationPct10 % 10);
			IB_LOG_DEBUG2_FMT(__func__,"    Discards: %3u.%1u%%",
							  response[0].externalCategoryStats.categoryMaximums.discardsPct10 / 10,
							  response[0].externalCategoryStats.categoryMaximums.discardsPct10 % 10);

			IB_LOG_DEBUG2_FMT(__func__, "  Min Internal Rate %u", response[0].minInternalRate);
			IB_LOG_DEBUG2_FMT(__func__, "  Max Internal Rate %u", response[0].maxInternalRate);
			IB_LOG_DEBUG2_FMT(__func__, "  Min External Rate %u", response[0].minExternalRate);
			IB_LOG_DEBUG2_FMT(__func__, "  Max External Rate %u", response[0].maxExternalRate);
			IB_LOG_DEBUG2_FMT(__func__, "  Max Internal MBps %u", response[0].maxInternalMBps);
			IB_LOG_DEBUG2_FMT(__func__, "  Max External MBps %u", response[0].maxExternalMBps);

			time_t absTime = (time_t)response[0].imageId.imageTime.absoluteTime;

			if (absTime) {
				snprintf(logBuf, sizeof(logBuf), " Time %s", ctime((const time_t *)&absTime));
				if ((strlen(logBuf)>0) && (logBuf[strlen(logBuf)-1] == '\n'))
					logBuf[strlen(logBuf)-1] = '\0';
			}

			IB_LOG_DEBUG2_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d%s",
				response[0].imageId.imageNumber, response[0].imageId.imageOffset, logBuf);

		} /* end debug logging */
    	BSWAP_STL_PA_PM_GROUP_INFO(&response[0], 0);
    	memcpy(data, response, responseSize);
	}

done:
	// determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating array failed
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// NULL pointer passed to function
	} else if (status == (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;	// Impropper parameter passed to function
	} else if (status == FNOT_FOUND) {
		if (retImageId.imageNumber == BAD_IMAGE_ID)
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;			// Failed to access/find image
		else
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_GROUP;			// Failed to find Group
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_GROUP;				// Failed to find Group
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_PM_GROUP_INFO_DATA) + Calculate_Padding(sizeof(STL_PA_PM_GROUP_INFO_DATA));

	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);

	if (response != NULL)
		vs_pool_free(&pm_pool, response);

	IB_EXIT(__func__, status);
	return(status);

}	// End of pa_getGroupInfoResp()

Status_t
pa_getGroupConfigResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS		status;
	Status_t	vStatus;
	PmGroupConfig_t pmGroupConfig = {{0}};
	STL_PA_PM_GROUP_CFG_RSP *response = NULL;
	STL_PA_PM_GROUP_CFG_REQ *p = (STL_PA_PM_GROUP_CFG_REQ *)&maip->data[STL_PA_DATA_OFFSET];
	uint32		responseSize = 0;
	int			i;
	char		groupName[STL_PM_GROUPNAMELEN];

	IB_ENTER(__func__, maip, 0, 0, 0);
	
	INCREMENT_PM_COUNTER(pmCounterPaRxGetGrpCfg);
	BSWAP_STL_PA_GROUP_CONFIG_REQ(p);
	strncpy(groupName, p->groupName, STL_PM_GROUPNAMELEN-1);
	groupName[STL_PM_GROUPNAMELEN-1]=0;

	IB_LOG_DEBUG1_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d", p->imageId.imageNumber, p->imageId.imageOffset);
	IB_LOG_DEBUG1_FMT(__func__, "Group: %.*s", (int)sizeof(groupName), groupName);

	status = paGetGroupConfig(&g_pmSweepData, groupName, &pmGroupConfig, p->imageId, &retImageId);
	if (status == FSUCCESS) {
		records = pmGroupConfig.NumPorts;
		responseSize = pmGroupConfig.NumPorts * sizeof(STL_PA_PM_GROUP_CFG_RSP);
		if (responseSize) {
			vStatus = vs_pool_alloc(&pm_pool, responseSize, (void*)&response);
			if (vStatus != VSTATUS_OK) {
				IB_LOG_ERRORRC("Failed to allocate response buffer for GroupConfig rc:", vStatus);
				status = FINSUFFICIENT_MEMORY;
				goto done;
			}
			memset(response, 0, responseSize);
		}

		for (i = 0; i < pmGroupConfig.NumPorts; i++) {
    		response[i].nodeLid				= pmGroupConfig.portList[i].lid;
    		response[i].portNumber			= pmGroupConfig.portList[i].portNum;
    		response[i].nodeGUID			= pmGroupConfig.portList[i].guid;
    		strncpy(response[i].nodeDesc, pmGroupConfig.portList[i].nodeDesc, sizeof(response[i].nodeDesc)-1);
			response[i].imageId = retImageId;
			response[i].imageId.imageOffset = 0;
		}

		time_t absTime = (time_t)retImageId.imageTime.absoluteTime;
		char buf[80];

		if (absTime) {
			snprintf(buf, sizeof(buf), " Time %s", ctime((const time_t *)&absTime));
			if ((strlen(buf)>0) && (buf[strlen(buf)-1] == '\n'))
				buf[strlen(buf)-1] = '\0';
		}

		IB_LOG_DEBUG2_FMT(__func__, "Group name %.*s", (int)sizeof(groupName), groupName);
		IB_LOG_DEBUG2_FMT(__func__, "Number ports: %u", pmGroupConfig.NumPorts);
		for (i = 0; i < pmGroupConfig.NumPorts; i++) {
			IB_LOG_DEBUG2_FMT(__func__, "  %d:LID:0x%08X Port:%u  Guid:0x%016"PRIx64"  NodeDesc: %.*s",
				i+1, response[i].nodeLid, response[i].portNumber, response[i].nodeGUID,
				(int)sizeof(response[i].nodeDesc), response[i].nodeDesc);
			IB_LOG_DEBUG2_FMT(__func__, "     ImageID: Number 0x%"PRIx64" Offset %d%s",
				response[i].imageId.imageNumber, response[i].imageId.imageOffset, buf);
    		BSWAP_STL_PA_GROUP_CONFIG_RSP(&response[i]);
		}
    	memcpy(data, response, responseSize);
	}

done:
	// determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating array failed
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// NULL pointer passed to function
	} else if (status == (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;	// Impropper parameter passed to function
	} else if (status == FNOT_FOUND) {
		if (retImageId.imageNumber == BAD_IMAGE_ID)
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;			// Failed to access/find image
		else
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_GROUP;			// Failed to find Group
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_GROUP;				// Failed to find Group
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;				// Group has no ports
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_PM_GROUP_CFG_RSP) + Calculate_Padding(sizeof(STL_PA_PM_GROUP_CFG_RSP));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);

	if (pmGroupConfig.portList != NULL)
		vs_pool_free(&pm_pool, pmGroupConfig.portList);
	if (response != NULL)
		vs_pool_free(&pm_pool, response);
	
	IB_EXIT(__func__, status);
	return(status);

}	// End of pa_getGroupConfigResp()

Status_t
pa_getFocusPortsResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS		status;
	Status_t	vStatus;
	uint32_t	select, start, range;
	PmFocusPorts_t pmFocusPorts = {{0}};
	STL_FOCUS_PORTS_RSP *response = NULL;
	STL_FOCUS_PORTS_REQ *p = (STL_FOCUS_PORTS_REQ *)&maip->data[STL_PA_DATA_OFFSET];
	uint32		responseSize = 0;
	int			i;
	char		groupName[STL_PM_GROUPNAMELEN];

	IB_ENTER(__func__, maip, 0, 0, 0);
	
	INCREMENT_PM_COUNTER(pmCounterPaRxGetFocusPorts);
	BSWAP_STL_PA_FOCUS_PORTS_REQ(p);
	select  = p->select;
	start   = p->start;
	range   = p->range;

	strncpy(groupName, p->groupName, STL_PM_GROUPNAMELEN-1);
	groupName[STL_PM_GROUPNAMELEN-1] = 0; 

	IB_LOG_DEBUG1_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d", p->imageId.imageNumber, p->imageId.imageOffset);
	IB_LOG_DEBUG1_FMT(__func__, "Group: %.*s  Select: 0x%x  Start: %u  Range: %u",
		(int)sizeof(groupName), groupName, select, start, range);

	status = paGetFocusPorts(&g_pmSweepData, groupName, &pmFocusPorts,
	    p->imageId, &retImageId, select, start, range);

	if (status == FSUCCESS) {
		records = pmFocusPorts.NumPorts;
		responseSize = pmFocusPorts.NumPorts * sizeof(STL_FOCUS_PORTS_RSP);
		if (responseSize) {
			vStatus = vs_pool_alloc(&pm_pool, responseSize, (void*)&response);
			if (vStatus != VSTATUS_OK) {
				IB_LOG_ERRORRC("Failed to allocate response buffer for FocusPorts rc:", vStatus);
				status = FINSUFFICIENT_MEMORY;
				goto done;
			}
		}
		memset(response, 0, responseSize);

		for (i = 0; i < pmFocusPorts.NumPorts; i++) {
    		response[i].nodeLid				= pmFocusPorts.portList[i].lid;
    		response[i].portNumber			= pmFocusPorts.portList[i].portNum;
			response[i].localStatus			= pmFocusPorts.portList[i].localStatus;
    		response[i].rate				= pmFocusPorts.portList[i].rate;
			response[i].maxVlMtu					= pmFocusPorts.portList[i].mtu;
    		response[i].value				= pmFocusPorts.portList[i].value;
    		response[i].nodeGUID			= pmFocusPorts.portList[i].guid;
    		strncpy(response[i].nodeDesc, pmFocusPorts.portList[i].nodeDesc, sizeof(response[i].nodeDesc)-1);
			response[i].neighborStatus 		= pmFocusPorts.portList[i].neighborStatus;
    		response[i].neighborLid 		= pmFocusPorts.portList[i].neighborLid;
    		response[i].neighborPortNumber	= pmFocusPorts.portList[i].neighborPortNum;
    		response[i].neighborValue		= pmFocusPorts.portList[i].neighborValue;
    		response[i].neighborGuid		= pmFocusPorts.portList[i].neighborGuid;
    		strncpy(response[i].neighborNodeDesc, pmFocusPorts.portList[i].neighborNodeDesc,
					sizeof(response[i].neighborNodeDesc)-1);
			response[i].imageId = retImageId;
			response[i].imageId.imageOffset = 0;
		}

		time_t absTime = (time_t)retImageId.imageTime.absoluteTime;
		char buf[80];

		if (absTime) {
			snprintf(buf, sizeof(buf), " Time %s", ctime((const time_t *)&absTime));
			if ((strlen(buf)>0) && (buf[strlen(buf)-1] == '\n'))
				buf[strlen(buf)-1] = '\0';
		}

		IB_LOG_DEBUG2_FMT(__func__, "Group name %.*s", (int)sizeof(groupName), groupName);
		IB_LOG_DEBUG2_FMT(__func__, "Number ports: %u", pmFocusPorts.NumPorts);
		for (i = 0; i < pmFocusPorts.NumPorts; i++) {
			IB_LOG_DEBUG2_FMT(__func__, "  %u:LID:0x%08X Port:%u Value:%"PRId64" ",
					i+1, response[i].nodeLid, response[i].portNumber, response[i].value);
			IB_LOG_DEBUG2_FMT(__func__, "     nodeGUID 0x%"PRIx64" ", response[i].nodeGUID);
			IB_LOG_DEBUG2_FMT(__func__, "     Node Description %.*s Status: %s",
				(int)sizeof(response[i].nodeDesc), response[i].nodeDesc,
				StlFocusStatusToText(response[i].localStatus));
			IB_LOG_DEBUG2_FMT(__func__, "     nLID:0x%08X nPort:%u nValue:%"PRId64" ",
					response[i].neighborLid, response[i].neighborPortNumber, response[i].neighborValue);
			IB_LOG_DEBUG2_FMT(__func__, "     rate:%u mtu:%u", response[i].rate, response[i].maxVlMtu);
			IB_LOG_DEBUG2_FMT(__func__, "     nGUID 0x%"PRIx64" ", response[i].neighborGuid);
			IB_LOG_DEBUG2_FMT(__func__, "     Nbr Node Desc    %.*s Status: %s",
				(int)sizeof(response[i].neighborNodeDesc), response[i].neighborNodeDesc,
				StlFocusStatusToText(response[i].neighborStatus));
			IB_LOG_DEBUG2_FMT(__func__, "     ImageID: Number 0x%"PRIx64" Offset %d%s",
				response[i].imageId.imageNumber, response[i].imageId.imageOffset, buf);
    		BSWAP_STL_PA_FOCUS_PORTS_RSP(&response[i]);
		}
    	memcpy(data, response, responseSize);
	}

done:
    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating array failed
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// NULL pointer passed to function
	} else if (status == (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;	// Impropper parameter passed to function
	} else if (status == FNOT_FOUND) {
		if (retImageId.imageNumber == BAD_IMAGE_ID)
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;			// Failed to access/find image
		else
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_GROUP;			// Failed to find Group
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_GROUP)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_GROUP;				// Failed to find Group
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_FOCUS_PORTS_RSP) + Calculate_Padding(sizeof(STL_FOCUS_PORTS_RSP));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);

	if (pmFocusPorts.portList != NULL)
		vs_pool_free(&pm_pool, pmFocusPorts.portList);
	if (response != NULL)
		vs_pool_free(&pm_pool, response);
	
	IB_EXIT(__func__, status);
	return(status);
}

Status_t
pa_getVFListResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	responseSize = 0;
	uint32_t	attribOffset;
	uint32_t	imageIndex;
	FSTATUS		status;
	Status_t	vStatus;
	PmVFList_t	VFList = {0};
	STL_PA_VF_LIST *response = NULL;
	int			i;

	IB_ENTER(__func__, maip, 0, 0, 0);
	
	INCREMENT_PM_COUNTER(pmCounterPaRxGetVFList);
	IB_LOG_DEBUG1_FMT(__func__, "Getting VF list");

	imageIndex = (g_pmSweepData.LastSweepIndex == PM_IMAGE_INDEX_INVALID ? 0 : g_pmSweepData.LastSweepIndex);
	status = paGetVFList(&g_pmSweepData, &VFList, imageIndex);
	if (status == FSUCCESS) {
		records = VFList.NumVFs;
		responseSize = VFList.NumVFs * STL_PM_VFNAMELEN;
		if (responseSize) {
			vStatus = vs_pool_alloc(&pm_pool, responseSize, (void*)&response);
			if (vStatus != VSTATUS_OK) {
				IB_LOG_ERRORRC("Failed to allocate response buffer for VFList rc:", vStatus);
				status = FINSUFFICIENT_MEMORY;
				goto done;
			}
			memset(response, 0, responseSize);
		}

		IB_LOG_DEBUG2_FMT(__func__, "Number of VFs: %u", VFList.NumVFs);
		for (i = 0; i < VFList.NumVFs; i++) {
    		strncpy(response[i].vfName, VFList.VfList[i].Name, STL_PM_VFNAMELEN-1);
			IB_LOG_DEBUG2_FMT(__func__, "VF %d: %.*s", i+1,
				(int)sizeof(response[i].vfName),response[i].vfName);
		}
    	memcpy(data, response, responseSize);
	}

done:
    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating array failed
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// NULL pointer passed to function
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_VF;				// Failed to find VF
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_VF_LIST) + Calculate_Padding(sizeof(STL_PA_VF_LIST));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);

	if (VFList.VfList != NULL)
		vs_pool_free(&pm_pool, VFList.VfList);
	if (response != NULL)
		vs_pool_free(&pm_pool, response);

	IB_EXIT(__func__, status);
	return(status);

}	// End of pa_getVFListResp()

Status_t
pa_getVFInfoResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	char		vfName[STL_PM_VFNAMELEN];
	uint32_t	records = 0;
	uint32_t	attribOffset;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS		status;
	Status_t	vStatus;
	PmVFInfo_t	vfInfo;
	char		logBuf[80];
	char		*p1;
	STL_PA_VF_INFO_DATA *response = NULL;
	STL_PA_VF_INFO_DATA *p = (STL_PA_VF_INFO_DATA *)&maip->data[STL_PA_DATA_OFFSET];
	uint32		responseSize = 0;
	int i;

	IB_ENTER(__func__, maip, 0, 0, 0);
	
	INCREMENT_PM_COUNTER(pmCounterPaRxGetVFList);
	BSWAP_STL_PA_VF_INFO(p, 1);
	strncpy(vfName, p->vfName, STL_PM_GROUPNAMELEN-1);
	vfName[STL_PM_GROUPNAMELEN-1]=0;

	IB_LOG_DEBUG1_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d", p->imageId.imageNumber, p->imageId.imageOffset);
	IB_LOG_DEBUG1_FMT(__func__, "VF: %.*s", (int)sizeof(vfName), vfName);

	status = paGetVFInfo(&g_pmSweepData, vfName, &vfInfo, p->imageId,
	    &retImageId);
	if (status == FSUCCESS) {
		records = 1;
		responseSize = records * sizeof(STL_PA_PM_GROUP_INFO_DATA);
		if (responseSize) {
			vStatus = vs_pool_alloc(&pm_pool, responseSize, (void*)&response);
			if (vStatus != VSTATUS_OK) {
				IB_LOG_ERRORRC("Failed to allocate response buffer for VFInfo rc:", vStatus);
				status = FINSUFFICIENT_MEMORY;
				goto done;
			}
			memset(response, 0, responseSize);
		}
		strncpy(response[0].vfName, vfName, STL_PM_GROUPNAMELEN - 1);
		response[0].numPorts											= vfInfo.NumPorts;
		response[0].internalUtilStats.totalMBps						= vfInfo.IntUtil.TotMBps;
		response[0].internalUtilStats.totalKPps						= vfInfo.IntUtil.TotKPps;
		response[0].internalUtilStats.avgMBps							= vfInfo.IntUtil.AvgMBps;
		response[0].internalUtilStats.minMBps							= vfInfo.IntUtil.MinMBps;
		response[0].internalUtilStats.maxMBps							= vfInfo.IntUtil.MaxMBps;
		response[0].internalUtilStats.numBWBuckets						= STL_PM_UTIL_BUCKETS;
		for (i = 0; i < STL_PM_UTIL_BUCKETS; i++) {
			response[0].internalUtilStats.BWBuckets[i] = vfInfo.IntUtil.BwPorts[i];
		}
		response[0].internalUtilStats.avgKPps							= vfInfo.IntUtil.AvgKPps;
		response[0].internalUtilStats.minKPps							= vfInfo.IntUtil.MinKPps;
		response[0].internalUtilStats.maxKPps							= vfInfo.IntUtil.MaxKPps;
		response[0].internalUtilStats.pmaNoRespPorts					= vfInfo.IntUtil.pmaNoRespPorts;
		response[0].internalUtilStats.topoIncompPorts					= vfInfo.IntUtil.topoIncompPorts;

		response[0].internalCategoryStats.categoryMaximums.integrityErrors		= vfInfo.IntErr.Max.Integrity;
		response[0].internalCategoryStats.categoryMaximums.congestion		= vfInfo.IntErr.Max.Congestion;
		response[0].internalCategoryStats.categoryMaximums.smaCongestion 	= vfInfo.IntErr.Max.SmaCongestion;
		response[0].internalCategoryStats.categoryMaximums.bubble	   	 	= vfInfo.IntErr.Max.Bubble;
		response[0].internalCategoryStats.categoryMaximums.securityErrors   	 	= vfInfo.IntErr.Max.Security;
		response[0].internalCategoryStats.categoryMaximums.routingErrors	   	 	= vfInfo.IntErr.Max.Routing;

		response[0].internalCategoryStats.categoryMaximums.utilizationPct10 = vfInfo.IntErr.Max.UtilizationPct10;
		response[0].internalCategoryStats.categoryMaximums.discardsPct10    = vfInfo.IntErr.Max.DiscardsPct10;

		for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
			response[0].internalCategoryStats.ports[i].integrityErrors		= vfInfo.IntErr.Ports[i].Integrity;
			response[0].internalCategoryStats.ports[i].congestion		= vfInfo.IntErr.Ports[i].Congestion;
			response[0].internalCategoryStats.ports[i].smaCongestion	= vfInfo.IntErr.Ports[i].SmaCongestion;
			response[0].internalCategoryStats.ports[i].bubble			= vfInfo.IntErr.Ports[i].Bubble;
			response[0].internalCategoryStats.ports[i].securityErrors			= vfInfo.IntErr.Ports[i].Security;
			response[0].internalCategoryStats.ports[i].routingErrors			= vfInfo.IntErr.Ports[i].Routing;
		}
		response[0].minInternalRate = vfInfo.MinIntRate;
		response[0].maxInternalRate = vfInfo.MaxIntRate;
		response[0].maxInternalMBps = StlStaticRateToMBps(response[0].maxInternalRate);

		response[0].imageId = retImageId;
		response[0].imageId.imageOffset = 0;

		if (IB_LOG_IS_INTERESTED(VS_LOG_DEBUG2)) {
			IB_LOG_DEBUG2_FMT(__func__, "VF name %.*s", (int)sizeof(response[0].vfName), response[0].vfName);
			IB_LOG_DEBUG2_FMT(__func__, "Number of ports: %u", response[0].numPorts);
			IB_LOG_DEBUG2_FMT(__func__, "Internal utilization statistics:");
			IB_LOG_DEBUG2_FMT(__func__, "  Util: Tot %6"PRIu64" Max %6u Min %6u Avg %6u MB/s",
					response[0].internalUtilStats.totalMBps, response[0].internalUtilStats.maxMBps, response[0].internalUtilStats.minMBps, response[0].internalUtilStats.avgMBps);
			p1 = logBuf;
			p1 += sprintf(p1, "%s", "  Util: ");
			for (i = 0; i < STL_PM_UTIL_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalUtilStats.BWBuckets[i]);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			IB_LOG_DEBUG2_FMT(__func__, "  Pkts: Tot %6"PRIu64" Max %6u Min %6u Avg %6u KP/s",
				response[0].internalUtilStats.totalKPps, response[0].internalUtilStats.maxKPps,
				response[0].internalUtilStats.minKPps, response[0].internalUtilStats.avgKPps);
			IB_LOG_DEBUG2_FMT(__func__, "  NoResp Ports PMA: %u    NoResp Ports Topology: %u",
				response[0].internalUtilStats.pmaNoRespPorts, response[0].internalUtilStats.topoIncompPorts);
			IB_LOG_DEBUG2_FMT(__func__, "Internal Category Summary:");
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Integrity       Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.integrityErrors);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].integrityErrors);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Congestion      Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.congestion);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].congestion);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg SmaCongestion   Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.smaCongestion);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].smaCongestion);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Bubble          Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.bubble);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].bubble);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Security        Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.securityErrors);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].securityErrors);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			p1 = logBuf;
			p1 += sprintf(p1, "  Ctg Routing         Max %10u Buckets: ", response[0].internalCategoryStats.categoryMaximums.routingErrors);
			for (i = 0; i < STL_PM_CATEGORY_BUCKETS; i++) {
				p1 += sprintf(p1, " %4d", response[0].internalCategoryStats.ports[i].routingErrors);
			}
			IB_LOG_DEBUG2_FMT(__func__, "%.*s", (int)sizeof(logBuf), logBuf);
			IB_LOG_DEBUG2_FMT(__func__,"    Utilization: %3u.%1u%%",
							  response[0].internalCategoryStats.categoryMaximums.utilizationPct10 / 10,
							  response[0].internalCategoryStats.categoryMaximums.utilizationPct10 % 10);
			IB_LOG_DEBUG2_FMT(__func__,"    Discards: %3u.%1u%%",
							  response[0].internalCategoryStats.categoryMaximums.discardsPct10 / 10,
							  response[0].internalCategoryStats.categoryMaximums.discardsPct10 % 10);

			IB_LOG_DEBUG2_FMT(__func__, "  Min Internal Rate %u", response[0].minInternalRate);
			IB_LOG_DEBUG2_FMT(__func__, "  Max Internal Rate %u", response[0].maxInternalRate);
			IB_LOG_DEBUG2_FMT(__func__, "  Max Internal MBps %u", response[0].maxInternalMBps);


			time_t absTime = (time_t)response[0].imageId.imageTime.absoluteTime;

			if (absTime) {
				snprintf(logBuf, sizeof(logBuf), " Time %s", ctime((const time_t *)&absTime));
				if ((strlen(logBuf)>0) && (logBuf[strlen(logBuf)-1] == '\n'))
					logBuf[strlen(logBuf)-1] = '\0';
			}

			IB_LOG_DEBUG2_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d%s",
				response[0].imageId.imageNumber, response[0].imageId.imageOffset, logBuf);
		} /* end debug logging */
	
    	BSWAP_STL_PA_VF_INFO(&response[0], 0);
    	memcpy(data, response, responseSize);
	}

done:
    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating array failed
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// NULL pointer passed to function
	} else if (status == (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;	// Impropper parameter passed to function
	} else if (status == FNOT_FOUND) {
		if (retImageId.imageNumber == BAD_IMAGE_ID)
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;			// Failed to access/find image
		else
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_VF;			// Failed to find VF
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_VF;				// Failed to find VF
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_VF_INFO_DATA) + Calculate_Padding(sizeof(STL_PA_VF_INFO_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);

	if (response != NULL)
		vs_pool_free(&pm_pool, response);

	IB_EXIT(__func__, status);
	return(status);

}	// End of pa_getVFInfoResp()

Status_t
pa_getVFConfigResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS		status;
	Status_t	vStatus;
	PmVFConfig_t pmVFConfig = {{0}};
	STL_PA_VF_CFG_RSP *response = NULL;
	STL_PA_VF_CFG_REQ *p = (STL_PA_VF_CFG_REQ *)&maip->data[STL_PA_DATA_OFFSET];
	uint32		responseSize = 0;
	int			i;
	char		vfName[STL_PM_VFNAMELEN];

	IB_ENTER(__func__, maip, 0, 0, 0);
	
	INCREMENT_PM_COUNTER(pmCounterPaRxGetVFCfg);
	BSWAP_STL_PA_VF_CFG_REQ(p);
	strncpy(vfName, p->vfName, STL_PM_VFNAMELEN-1);
	vfName[STL_PM_VFNAMELEN-1]=0;

	IB_LOG_DEBUG1_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d", p->imageId.imageNumber, p->imageId.imageOffset);
	IB_LOG_DEBUG1_FMT(__func__, "VF: %.*s", (int)sizeof(vfName), vfName);

	status = paGetVFConfig(&g_pmSweepData, vfName, 0, &pmVFConfig, p->imageId, &retImageId);
	if (status == FSUCCESS) {
		records = pmVFConfig.NumPorts;
		responseSize = pmVFConfig.NumPorts * sizeof(STL_PA_VF_CFG_RSP);
		if (responseSize) {
			vStatus = vs_pool_alloc(&pm_pool, responseSize, (void*)&response);
			if (vStatus != VSTATUS_OK) {
				IB_LOG_ERRORRC("Failed to allocate response buffer for VFConfig rc:", vStatus);
				status = FINSUFFICIENT_MEMORY;
				goto done;
			}
			memset(response, 0, responseSize);
		}

		for (i = 0; i < pmVFConfig.NumPorts; i++) {
    		response[i].nodeLid 			= pmVFConfig.portList[i].lid;
    		response[i].portNumber			= pmVFConfig.portList[i].portNum;
    		response[i].nodeGUID			= pmVFConfig.portList[i].guid;
    		strncpy(response[i].nodeDesc, pmVFConfig.portList[i].nodeDesc, sizeof(response[i].nodeDesc)-1);
			response[i].imageId	= retImageId;
			response[i].imageId.imageOffset	= 0;
		}

		time_t absTime = (time_t)retImageId.imageTime.absoluteTime;
		char buf[80];

		if (absTime) {
			snprintf(buf, sizeof(buf), " Time %s", ctime((const time_t *)&absTime));
			if ((strlen(buf)>0) && (buf[strlen(buf)-1] == '\n'))
				buf[strlen(buf)-1] = '\0';
		}

		IB_LOG_DEBUG2_FMT(__func__, "VF name %.*s", (int)sizeof(vfName), vfName);
		IB_LOG_DEBUG2_FMT(__func__, "Number ports: %u", pmVFConfig.NumPorts);
		for (i = 0; i < pmVFConfig.NumPorts; i++) {
			IB_LOG_DEBUG2_FMT(__func__, "  %d:LID:0x%08X Port:%u  Guid:0x%016"PRIx64"  NodeDesc: %.*s",
				i+1, response[i].nodeLid, response[i].portNumber, response[i].nodeGUID,
				(int)sizeof(response[i].nodeDesc), response[i].nodeDesc);
			IB_LOG_DEBUG2_FMT(__func__, "     ImageID: Number 0x%"PRIx64" Offset %d%s",
				response[i].imageId.imageNumber, response[i].imageId.imageOffset, buf);
    		BSWAP_STL_PA_VF_CFG_RSP(&response[i]);
		}
    	memcpy(data, response, responseSize);
	}

done:
    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating array failed
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// NULL pointer passed to function
	} else if (status == (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;	// Impropper parameter passed to function
	} else if (status == FNOT_FOUND) {
		if (retImageId.imageNumber == BAD_IMAGE_ID)
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;			// Failed to access/find image
		else
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_VF;			// Failed to find VF
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_VF;				// Failed to find VF
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;				// Failed to find port
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_VF_CFG_RSP) + Calculate_Padding(sizeof(STL_PA_VF_CFG_RSP));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

	pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);

	if (pmVFConfig.portList != NULL)
		vs_pool_free(&pm_pool, pmVFConfig.portList);
	if (response != NULL)
		vs_pool_free(&pm_pool, response);
	
	IB_EXIT(__func__, status);
	return(status);

}	// End of pa_getVFConfigResp()

Status_t
pa_getVFPortCountersResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	char		vfName[STL_PM_VFNAMELEN];
	uint32_t	delta, userCntrs;
	uint32		flags = 0;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	STL_LID_32	nodeLid;
	PORT		portNumber;
	FSTATUS		status;
	PmCompositeVLCounters_t pmVLPortCounters = {0};
    STL_PA_VF_PORT_COUNTERS_DATA response = {0};
    STL_PA_VF_PORT_COUNTERS_DATA *p = (STL_PA_VF_PORT_COUNTERS_DATA *)&maip->data[STL_PA_DATA_OFFSET];

	IB_ENTER(__func__, maip, 0, 0, 0);

	INCREMENT_PM_COUNTER(pmCounterPaRxGetVFPortCtrs);
	BSWAP_STL_PA_VF_PORT_COUNTERS(p);
	strncpy(vfName, p->vfName, STL_PM_VFNAMELEN-1);
	vfName[STL_PM_VFNAMELEN-1]=0;
	nodeLid		= p->nodeLid;
	portNumber	= p->portNumber;
	delta 		= p->flags & STL_PA_PC_FLAG_DELTA;
	userCntrs	= p->flags & STL_PA_PC_FLAG_USER_COUNTERS;

	IB_LOG_DEBUG1_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d", p->imageId.imageNumber, p->imageId.imageOffset);
	IB_LOG_DEBUG1_FMT(__func__, "VF: %.*s  LID: 0x%x  port: %u  flags: 0x%x",
		(int)sizeof(vfName), vfName, nodeLid, portNumber, p->flags);

	status = paGetVFPortStats(&g_pmSweepData, nodeLid, portNumber, vfName, &pmVLPortCounters, delta,
							  userCntrs, p->imageId, &flags, &retImageId);
	if (status == FSUCCESS) {
		records = 1;
    	response.nodeLid						= nodeLid;
    	response.portNumber						= portNumber;
    	response.flags							= flags;
		response.portVFXmitData					= pmVLPortCounters.PortVLXmitData;
		response.portVFRcvData					= pmVLPortCounters.PortVLRcvData;
		response.portVFXmitPkts					= pmVLPortCounters.PortVLXmitPkts;
		response.portVFRcvPkts					= pmVLPortCounters.PortVLRcvPkts;
		response.portVFXmitDiscards				= pmVLPortCounters.PortVLXmitDiscards;
		response.swPortVFCongestion				= pmVLPortCounters.SwPortVLCongestion;
		response.portVFXmitWait					= pmVLPortCounters.PortVLXmitWait;
		response.portVFRcvFECN					= pmVLPortCounters.PortVLRcvFECN;
		response.portVFRcvBECN					= pmVLPortCounters.PortVLRcvBECN;
		response.portVFXmitTimeCong				= pmVLPortCounters.PortVLXmitTimeCong;
		response.portVFXmitWastedBW				= pmVLPortCounters.PortVLXmitWastedBW;
		response.portVFXmitWaitData				= pmVLPortCounters.PortVLXmitWaitData;
		response.portVFRcvBubble				= pmVLPortCounters.PortVLRcvBubble;
		response.portVFMarkFECN					= pmVLPortCounters.PortVLMarkFECN;
    	response.imageId = retImageId;
		response.imageId.imageOffset = 0;
		strncpy(response.vfName, vfName, STL_PM_VFNAMELEN-1);

		/* debug logging */
		IB_LOG_DEBUG2_FMT(__func__, "%s Controlled VF Port Counters (%s) for node lid 0x%x, port number %u%s:",
			(userCntrs?"User":"PM"), (delta?"delta":"total"), nodeLid, portNumber,
			flags & STL_PA_PC_FLAG_UNEXPECTED_CLEAR?" (Unexpected Clear)":"");
		IB_LOG_DEBUG2_FMT(__func__, " VF Name: %s", vfName);
		IB_LOG_DEBUG2_FMT(__func__, "Perfromance:");
		IB_LOG_DEBUG2_FMT(__func__, " Xmit Data:       %10"PRIu64" MB (%"PRIu64" Flits)",
						  response.portVFXmitData/FLITS_PER_MB, response.portVFXmitData);
		IB_LOG_DEBUG2_FMT(__func__, " Xmit Pkts:       %10"PRIu64" ", response.portVFXmitPkts);
		IB_LOG_DEBUG2_FMT(__func__, " Rcv Data:        %10"PRIu64" MB (%"PRIu64" Flits)",
						  response.portVFRcvData/FLITS_PER_MB, response.portVFRcvData);
		IB_LOG_DEBUG2_FMT(__func__, " Rcv Pkts:        %10"PRIu64" ", response.portVFRcvPkts);
		IB_LOG_DEBUG2_FMT(__func__, "Errors:");
		IB_LOG_DEBUG2_FMT(__func__, " Xmit Discards    %10"PRIu64" ", response.portVFXmitDiscards);
		IB_LOG_DEBUG2_FMT(__func__, " Cong Discards    %10"PRIu64" ", response.swPortVFCongestion);
		IB_LOG_DEBUG2_FMT(__func__, " Xmit Wait        %10"PRIu64" ", response.portVFXmitWait);
		IB_LOG_DEBUG2_FMT(__func__, " Rcv FECN         %10"PRIu64" ", response.portVFRcvFECN);
		IB_LOG_DEBUG2_FMT(__func__, " Rcv BECN         %10"PRIu64" ", response.portVFRcvBECN);
		IB_LOG_DEBUG2_FMT(__func__, " Xmit Time Cong   %10"PRIu64" ", response.portVFXmitTimeCong);
		IB_LOG_DEBUG2_FMT(__func__, " Xmit Wasted BW   %10"PRIu64" ", response.portVFXmitWastedBW);
		IB_LOG_DEBUG2_FMT(__func__, " Xmit Wait Data   %10"PRIu64" ", response.portVFXmitWaitData);
		IB_LOG_DEBUG2_FMT(__func__, " Rcv Bubble       %10"PRIu64" ", response.portVFRcvBubble);
		IB_LOG_DEBUG2_FMT(__func__, " Mark FECN        %10"PRIu64" ", response.portVFMarkFECN);

		time_t absTime = (time_t)response.imageId.imageTime.absoluteTime;
		char buf[80];

		if (absTime) {
			snprintf(buf, sizeof(buf), " Time %s", ctime((const time_t *)&absTime));
			if ((strlen(buf)>0) && (buf[strlen(buf)-1] == '\n'))
				buf[strlen(buf)-1] = '\0';
		}

		IB_LOG_DEBUG2_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d%s",
			response.imageId.imageNumber, response.imageId.imageOffset, buf);
		/* end debug logging */

    	BSWAP_STL_PA_VF_PORT_COUNTERS(&response);
		memcpy(data, &response, sizeof(response));
	} 

    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == (FUNAVAILABLE | STL_MAD_STATUS_STL_PA_UNAVAILABLE)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM nas not completed a sweep
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// NULL pointer passed to function
	} else if (status == (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;	// Impropper parameter passed to function
	} else if (status == FNOT_FOUND) {
		if (retImageId.imageNumber == BAD_IMAGE_ID)
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;			// Failed to access/find image
		else
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_VF;			// Failed to find VF
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_VF;				// Failed to find VF
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;				// Failed to find port
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_VF_PORT_COUNTERS_DATA) + Calculate_Padding(sizeof(STL_PA_VF_PORT_COUNTERS_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_clrVFPortCountersResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	char		vfName[STL_PM_VFNAMELEN];
	FSTATUS		status;
	STL_LID_32	nodeLid;
	PORT		portNumber;
	STLVlCounterSelectMask select;
    STL_PA_CLEAR_VF_PORT_COUNTERS_DATA response = {0};
    STL_PA_CLEAR_VF_PORT_COUNTERS_DATA *p = (STL_PA_CLEAR_VF_PORT_COUNTERS_DATA *)&maip->data[STL_PA_DATA_OFFSET];

	IB_ENTER(__func__, maip, 0, 0, 0);

	INCREMENT_PM_COUNTER(pmCounterPaRxClrVFPortCtrs);
	BSWAP_STL_PA_CLEAR_VF_PORT_COUNTERS(p);
	nodeLid			= p->nodeLid;
	portNumber		= p->portNumber;
	select.AsReg32	= p->vfCounterSelectMask.AsReg32;
	strncpy(vfName, p->vfName, STL_PM_VFNAMELEN-1);
	vfName[STL_PM_VFNAMELEN-1]=0;

	IB_LOG_DEBUG1_FMT(__func__, "Clearing VF %.*s counters on LID 0x%x port %u with select 0x%04x",
		(int)sizeof(vfName), vfName, nodeLid, portNumber, select.AsReg32);

	status = paClearVFPortStats(&g_pmSweepData, nodeLid, portNumber, select, vfName);
	if (status == FSUCCESS) {
		records = 1;
		response.nodeLid						= nodeLid;
		response.portNumber 					= portNumber;
		response.vfCounterSelectMask.AsReg32	= select.AsReg32;
		strncpy(response.vfName, vfName, STL_PM_VFNAMELEN-1);
    	BSWAP_STL_PA_CLEAR_VF_PORT_COUNTERS(&response);
		memcpy(data, &response, sizeof(response));
	}

    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == (FUNAVAILABLE | STL_MAD_STATUS_STL_PA_UNAVAILABLE)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM has not completed a sweep
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// NULL pointer passed to function
	} else if (status == (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;	// Impropper parameter passed to function
	} else if (status == FNOT_FOUND) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;				// Failed to find switch node
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_VF;				// Failed to find VF
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;				// Failed to find port
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_CLEAR_VF_PORT_COUNTERS_DATA) + Calculate_Padding(sizeof(STL_PA_CLEAR_VF_PORT_COUNTERS_DATA));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_getVFFocusPortsResp(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		*data = pa_data;
	uint32_t	records = 0;
	uint32_t	attribOffset;
	STL_PA_IMAGE_ID_DATA retImageId = {0};
	FSTATUS		status;
	Status_t	vStatus;
	uint32_t	select, start, range;
	PmVFFocusPorts_t pmVFFocusPorts = {{0}};
	STL_PA_VF_FOCUS_PORTS_RSP *response = NULL;
	STL_PA_VF_FOCUS_PORTS_REQ *p = (STL_PA_VF_FOCUS_PORTS_REQ *)&maip->data[STL_PA_DATA_OFFSET];
	uint32		responseSize = 0;
	int			i;
	char		vfName[STL_PM_VFNAMELEN];

	IB_ENTER(__func__, maip, 0, 0, 0);
	
	INCREMENT_PM_COUNTER(pmCounterPaRxGetVFFocusPorts);
	BSWAP_STL_PA_VF_FOCUS_PORTS_REQ(p);
	select	= p->select;
	start	= p->start;
	range	= p->range;
	strncpy(vfName, p->vfName, STL_PM_VFNAMELEN-1);
	vfName[STL_PM_VFNAMELEN-1]=0;

	IB_LOG_DEBUG1_FMT(__func__, "ImageID: Number 0x%"PRIx64" Offset %d", p->imageId.imageNumber, p->imageId.imageOffset);
	IB_LOG_DEBUG1_FMT(__func__, "VF: %.*s  Select: 0x%x  Start: %u  Range: %u",
		(int)sizeof(vfName), vfName, select, start, range);

	status = paGetVFFocusPorts(&g_pmSweepData, vfName, &pmVFFocusPorts,
	    p->imageId, &retImageId, select, start, range);
	if (status == FSUCCESS) {
		records = pmVFFocusPorts.NumPorts;
		responseSize = pmVFFocusPorts.NumPorts * sizeof(STL_PA_VF_FOCUS_PORTS_RSP);
		if (responseSize) {
			vStatus = vs_pool_alloc(&pm_pool, responseSize, (void*)&response);
			if (vStatus != VSTATUS_OK) {
				IB_LOG_ERRORRC("Failed to allocate response buffer for VFFocusPorts rc:", vStatus);
				status = FINSUFFICIENT_MEMORY;
				goto done;
			}
			memset(response, 0, responseSize);
		}

		for (i = 0; i < pmVFFocusPorts.NumPorts; i++) {
    		response[i].nodeLid				= pmVFFocusPorts.portList[i].lid;
    		response[i].portNumber			= pmVFFocusPorts.portList[i].portNum;
			response[i].localStatus			= pmVFFocusPorts.portList[i].localStatus;
    		response[i].rate				= pmVFFocusPorts.portList[i].rate;
			response[i].maxVlMtu					= pmVFFocusPorts.portList[i].mtu;
    		response[i].value				= pmVFFocusPorts.portList[i].value;
    		response[i].nodeGUID			= pmVFFocusPorts.portList[i].guid;
    		strncpy(response[i].nodeDesc, pmVFFocusPorts.portList[i].nodeDesc,
					sizeof(response[i].nodeDesc)-1);

			response[i].neighborStatus 		= pmVFFocusPorts.portList[i].neighborStatus;
    		response[i].neighborLid 		= pmVFFocusPorts.portList[i].neighborLid;
    		response[i].neighborPortNumber 	= pmVFFocusPorts.portList[i].neighborPortNum;
    		response[i].neighborValue 		= pmVFFocusPorts.portList[i].neighborValue;
    		response[i].neighborGuid 		= pmVFFocusPorts.portList[i].neighborGuid;
    		strncpy(response[i].neighborNodeDesc, pmVFFocusPorts.portList[i].neighborNodeDesc,
					sizeof(response[i].neighborNodeDesc)-1);

			response[i].imageId = retImageId;
			response[i].imageId.imageOffset = 0;
		}

		time_t absTime = (time_t)retImageId.imageTime.absoluteTime;
		char buf[80];

		if (absTime) {
			snprintf(buf, sizeof(buf), " Time %s", ctime((const time_t *)&absTime));
			if ((strlen(buf)>0) && (buf[strlen(buf)-1] == '\n'))
				buf[strlen(buf)-1] = '\0';
		}

		IB_LOG_DEBUG2_FMT(__func__, "VF name %.*s", (int)sizeof(vfName), vfName);
		IB_LOG_DEBUG2_FMT(__func__, "Number ports: %u", pmVFFocusPorts.NumPorts);
		for (i = 0; i < pmVFFocusPorts.NumPorts; i++) {
			IB_LOG_DEBUG2_FMT(__func__, "  %d:LID:0x%08X Port:%u Value:%"PRId64" ",
							  i+1, response[i].nodeLid, response[i].portNumber, response[i].value);
			IB_LOG_DEBUG2_FMT(__func__, "     nodeGUID 0x%"PRIx64" ", response[i].nodeGUID);
			IB_LOG_DEBUG2_FMT(__func__, "     Node Description %.*s Status: %s",
				(int)sizeof(response[i].nodeDesc), response[i].nodeDesc,
				StlFocusStatusToText(response[i].localStatus));
			IB_LOG_DEBUG2_FMT(__func__, "     nLID:0x%08X nPort:%u nValue:%"PRId64" ",
					response[i].neighborLid, response[i].neighborPortNumber, response[i].neighborValue);
			IB_LOG_DEBUG2_FMT(__func__, "     rate:%u mtu:%u", response[i].rate, response[i].maxVlMtu);
			IB_LOG_DEBUG2_FMT(__func__, "     nGUID 0x%"PRIx64" ", response[i].neighborGuid);
			IB_LOG_DEBUG2_FMT(__func__, "     Nbr Node Desc    %.*s Status: %s",
				(int)sizeof(response[i].neighborNodeDesc), response[i].neighborNodeDesc,
				StlFocusStatusToText(response[i].neighborStatus));
			IB_LOG_DEBUG2_FMT(__func__, "     ImageID: Number 0x%"PRIx64" Offset %d%s",
				response[i].imageId.imageNumber, response[i].imageId.imageOffset, buf);

    		BSWAP_STL_PA_VF_FOCUS_PORTS_RSP(&response[i]);
		}
    	memcpy(data, response, responseSize);
	}

done:
    // determine reply status
	if (status == FUNAVAILABLE) {
		maip->base.status = STL_MAD_STATUS_STL_PA_UNAVAILABLE;			// PM engine not running
	} else if (status == FINSUFFICIENT_MEMORY) {
		maip->base.status = MAD_STATUS_SA_NO_RESOURCES;					// allocating array failed
	} else if (status == FINVALID_PARAMETER) {
		maip->base.status = MAD_STATUS_BAD_FIELD;						// NULL pointer passed to function
	} else if (status == (FINVALID_PARAMETER | STL_MAD_STATUS_STL_PA_INVALID_PARAMETER)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_INVALID_PARAMETER;	// Impropper parameter passed to function
	} else if (status == FNOT_FOUND) {
		if (retImageId.imageNumber == BAD_IMAGE_ID)
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_IMAGE;			// Failed to access/find image
		else
			maip->base.status = STL_MAD_STATUS_STL_PA_NO_VF;			// Failed to find VF
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_VF)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_VF;				// Failed to find VF
	} else if (status == (FNOT_FOUND | STL_MAD_STATUS_STL_PA_NO_PORT)) {
		maip->base.status = STL_MAD_STATUS_STL_PA_NO_PORT;				// Failed to find port
	} else if (maip->base.status != MAD_STATUS_OK) {
		records = 0;
	} else if (records == 0) {
		maip->base.status = MAD_STATUS_SA_NO_RECORDS;
	} else if ((maip->base.method == STL_PA_CMD_GET) && (records != 1)) {
		IB_LOG_WARN("too many records for STL_PA_CMD_GET:", records);
		records = 0;
		maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
	}

	attribOffset = sizeof(STL_PA_VF_FOCUS_PORTS_RSP) + Calculate_Padding(sizeof(STL_PA_VF_FOCUS_PORTS_RSP));
	/* setup attribute offset for possible RMPP transfer */
	pa_cntxt->attribLen = attribOffset / 8;

    pa_cntxt_data(pa_cntxt, data, records * attribOffset);
    // send response
	(void)pa_send_reply(maip, pa_cntxt);

	if (pmVFFocusPorts.portList != NULL)
		vs_pool_free(&pm_pool, pmVFFocusPorts.portList);
	if (response != NULL)
		vs_pool_free(&pm_pool, response);
	
	IB_EXIT(__func__, status);
	return(status);
}
