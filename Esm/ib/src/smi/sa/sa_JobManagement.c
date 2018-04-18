/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2017, Intel Corporation

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include "sm_l.h"
#include "sa_l.h"
#include "sm_jm.h"
#include "sm_counters.h"

#include "ibyteswap.h"

typedef enum {
	SA_JM_CMDRESP_OK = 0,
	SA_JM_CMDRESP_PARTIAL,
	SA_JM_CMDRESP_ERROR,
	SA_JM_CMDRESP_INVALID_ID
} sa_jm_cmd_resp_t;

typedef sa_jm_cmd_resp_t (*sa_jm_cmd_func_t)(Mai_t *, uint8_t *, uint32_t, uint8_t **, int *);

#define DECLARE_DISPATCH_FUNC(func) \
	static sa_jm_cmd_resp_t func(Mai_t *, uint8_t *, uint32_t, uint8_t **, int *);

DECLARE_DISPATCH_FUNC(sa_jm_cmd_get_cpi);
DECLARE_DISPATCH_FUNC(sa_jm_cmd_create);
DECLARE_DISPATCH_FUNC(sa_jm_cmd_set_use_matrix);
DECLARE_DISPATCH_FUNC(sa_jm_cmd_poll);
DECLARE_DISPATCH_FUNC(sa_jm_cmd_complete);
DECLARE_DISPATCH_FUNC(sa_jm_cmd_get_guid_list);
DECLARE_DISPATCH_FUNC(sa_jm_cmd_get_switch_map);
DECLARE_DISPATCH_FUNC(sa_jm_cmd_get_cost_matrix);
DECLARE_DISPATCH_FUNC(sa_jm_cmd_get_use_matrix);
DECLARE_DISPATCH_FUNC(sa_jm_cmd_get_jobs);

static sa_jm_cmd_func_t sa_jm_cmd_dispatch_table[] =
	{	NULL
	,	sa_jm_cmd_get_cpi
	,	sa_jm_cmd_create
	,	sa_jm_cmd_set_use_matrix
	,	sa_jm_cmd_poll
	,	sa_jm_cmd_complete
	,	sa_jm_cmd_get_guid_list
	,	sa_jm_cmd_get_switch_map
	,	sa_jm_cmd_get_cost_matrix
	,	sa_jm_cmd_get_use_matrix
	,	sa_jm_cmd_get_jobs
	};


static sa_jm_cmd_resp_t
sa_jm_cmd_get_cpi(Mai_t *mad, uint8_t *data, uint32_t len, uint8_t **outData, int *outLen)
{
	Status_t s;
	STL_CLASS_PORT_INFO cpi;
	uint8_t *buf;
	
    INCREMENT_COUNTER(smCounterJmReqClassPortInfo);

	s = vs_pool_alloc(&sm_pool, 1 + sizeof(CLASS_PORT_INFO), (void *)&buf);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to allocate space for ClassPortInfo buffer (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	memset(&cpi, 0, sizeof(cpi));
	cpi.bversion = MAD_BVERSION;
	cpi.cversion = JM_CVERSION;
	cpi.respTimeValue = sm_config.sa_resp_time_n2;

	s = cs_hton_ClassPortInfo(&cpi, buf + 1, NULL);	// FIXME: use BSWAP macros
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to endianize ClassPortInfo (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	*outData = buf;
	*outLen = 1 + sizeof(CLASS_PORT_INFO);

	return SA_JM_CMDRESP_OK;
}

static sa_jm_cmd_resp_t
sa_jm_cmd_create(Mai_t *mad, uint8_t *data, uint32_t len, uint8_t **outData, int *outLen)
{
	Status_t s;
	sa_jm_cmd_resp_t resp = SA_JM_CMDRESP_OK;
	JmEntry_t *job;
	JmMsgReqCreate_t input;
	JmWireJobOptions_t options;
	uint16_t count;
	uint16_t *cost;
	int costLen;
	time_t timestamp;

    INCREMENT_COUNTER(smCounterJmReqCreateJob);

	s = sm_jm_decode_req_create(data, len, &input);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to decode job input (status %d)", s);
		resp = SA_JM_CMDRESP_ERROR;
		goto fail1;
	}

	s = sm_jm_alloc_job(&job);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to allocate job entry (status %d)", s);
		resp = SA_JM_CMDRESP_ERROR;
		goto fail2;
	}

	options = input.options;
	job->params = input.params;
	job->state = JM_STATE_INITIALIZED;
	vs_stdtime_get(&timestamp);
	job->timestamp = timestamp;

	s = vs_wrlock(&old_topology_lock);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to lock the old topology (status %d)", s);
		resp = SA_JM_CMDRESP_ERROR;
		goto fail3;
	}

	job->passcount = topology_passcount;

	s = sm_jm_fill_ports(&old_topology, &input, job, &count);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to resolve port information from input (status %d)", s);
		resp = SA_JM_CMDRESP_ERROR;
		goto fail4;
	} else if (job->switchCount == 0) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to resolve switch information from input");
		resp = SA_JM_CMDRESP_ERROR;
		goto fail4;
	} else if (count < input.guids.count) {
		resp = SA_JM_CMDRESP_PARTIAL;
	}

	s = sm_jm_get_cost(&old_topology, job, &cost, &costLen);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to get topology cost information (status %d)", s);
		resp = SA_JM_CMDRESP_ERROR;
		goto fail4;
	}

	(void)vs_rwunlock(&old_topology_lock);
	sm_jm_free_req_create(&input);

	s = sm_jm_insert_job(job);
	if (s != VSTATUS_OK) {
		sm_jm_free_cost(cost);
		sm_jm_free_job(job);
		IB_LOG_ERROR_FMT( __func__,
			"Failed to insert the job into the job table (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	s = sm_jm_encode_resp_create(job, cost, costLen, outData, outLen);
	if (s != VSTATUS_OK) {
		sm_jm_remove_job(job);
		sm_jm_free_cost(cost);
		sm_jm_free_job(job);
		IB_LOG_ERROR_FMT( __func__,
			"Failed to create the message response (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	if (options.no_create) {
		sm_jm_remove_job(job);
		sm_jm_free_cost(cost);
		sm_jm_free_job(job);
	}

	else
		sm_jm_free_cost(cost);

	return resp;

fail4:
	(void)vs_rwunlock(&old_topology_lock);
fail3:
	sm_jm_free_job(job);
fail2:
	sm_jm_free_req_create(&input);
fail1:
	return resp;
}

static sa_jm_cmd_resp_t
sa_jm_cmd_set_use_matrix(Mai_t *mad, uint8_t *data, uint32_t len, uint8_t **outData, int *outLen)
{
	Status_t s;
	JmEntry_t * job;
	JmMsgReqSetUseMatrix_t input;

    INCREMENT_COUNTER(smCounterJmReqSetUseMatrix);

	s = sm_jm_decode_req_set_use_matrix(data, len, &input);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to decode request (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	s = sm_jm_lookup_job(input.id, &job);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to lookup JobID "FMT_U64, input.id);
		return SA_JM_CMDRESP_INVALID_ID;
	}

	if (job->state != JM_STATE_INITIALIZED) {
		IB_LOG_WARN_FMT( __func__,
			"Ignoring request: invalid job state (%d)", job->state);
		return SA_JM_CMDRESP_ERROR;
	}

	// for now, rather than calling free_resp, let's reuse
	// the existing buffer since the input structure and
	// job structure are identical
	job->useMatrix = input.matrix;
	job->state = JM_STATE_USE_AVAILABLE;

	*outData = NULL;
	*outLen = 0;

	return SA_JM_CMDRESP_OK;
}

static sa_jm_cmd_resp_t
sa_jm_cmd_poll(Mai_t *mad, uint8_t *data, uint32_t len, uint8_t **outData, int *outLen)
{
	Status_t s;
	JmEntry_t * job;
	JmMsgReqGenericQuery_t input;

    INCREMENT_COUNTER(smCounterJmReqPollReady);

	s = sm_jm_decode_req_generic_query(data, len, &input);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to decode request (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	s = sm_jm_lookup_job(input.id, &job);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to lookup JobID "FMT_U64, input.id);
		return SA_JM_CMDRESP_INVALID_ID;
	}

	s = sm_jm_encode_resp_poll(job, outData, outLen);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to create the message response (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	return SA_JM_CMDRESP_OK;
}

static sa_jm_cmd_resp_t
sa_jm_cmd_complete(Mai_t *mad, uint8_t *data, uint32_t len, uint8_t **outData, int *outLen)
{
	Status_t s;
	JmEntry_t * job;
	JmMsgReqGenericQuery_t input;

    INCREMENT_COUNTER(smCounterJmReqCompleteJob);

	s = sm_jm_decode_req_generic_query(data, len, &input);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to decode request (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	s = sm_jm_lookup_job(input.id, &job);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to lookup JobID "FMT_U64, input.id);
		return SA_JM_CMDRESP_INVALID_ID;
	}

	s = sm_jm_remove_job(job);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to remove JobID "FMT_U64" (status %d)",
			job->id, s);
		return SA_JM_CMDRESP_ERROR;
	}

	s = sm_jm_free_job(job);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to free JobID "FMT_U64" (status %d)",
			input.id, s);
		return SA_JM_CMDRESP_ERROR;
	}

	*outData = NULL;
	*outLen = 0;

	return SA_JM_CMDRESP_OK;
}

static sa_jm_cmd_resp_t
sa_jm_cmd_get_guid_list(Mai_t *mad, uint8_t *data, uint32_t len, uint8_t **outData, int *outLen)
{
	Status_t s;
	JmEntry_t * job;
	JmMsgReqGenericQuery_t input;

    INCREMENT_COUNTER(smCounterJmReqGetGuidList);

	s = sm_jm_decode_req_generic_query(data, len, &input);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to decode request (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	s = sm_jm_lookup_job(input.id, &job);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to lookup JobID "FMT_U64, input.id);
		return SA_JM_CMDRESP_INVALID_ID;
	}

	s = sm_jm_encode_resp_get_guid_list(job, outData, outLen);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to create the message response (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	return SA_JM_CMDRESP_OK;
}

static sa_jm_cmd_resp_t
sa_jm_cmd_get_switch_map(Mai_t *mad, uint8_t *data, uint32_t len, uint8_t **outData, int *outLen)
{
	Status_t s;
	JmEntry_t * job;
	JmMsgReqGenericQuery_t input;

    INCREMENT_COUNTER(smCounterJmReqGetSwitchMap);

	s = sm_jm_decode_req_generic_query(data, len, &input);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to decode request (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	s = sm_jm_lookup_job(input.id, &job);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to lookup JobID "FMT_U64, input.id);
		return SA_JM_CMDRESP_INVALID_ID;
	}

	s = sm_jm_encode_resp_get_switch_map(job, outData, outLen);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to create the message response (status %d)", s);
		return SA_JM_CMDRESP_OK;
	}

	return SA_JM_CMDRESP_OK;
}

static sa_jm_cmd_resp_t
sa_jm_cmd_get_cost_matrix(Mai_t *mad, uint8_t *data, uint32_t len, uint8_t **outData, int *outLen)
{
	Status_t s;
	JmEntry_t * job;
	JmMsgReqGenericQuery_t input;
	uint16_t *cost;
	int costLen;

    INCREMENT_COUNTER(smCounterJmReqGetCostMatrix);

	s = sm_jm_decode_req_generic_query(data, len, &input);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to decode request (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	s = sm_jm_lookup_job(input.id, &job);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to lookup JobID "FMT_U64, input.id);
		return SA_JM_CMDRESP_INVALID_ID;
	}

	s = vs_wrlock(&old_topology_lock);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to lock the old topology (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	s = sm_jm_get_cost(&old_topology, job, &cost, &costLen);
	(void)vs_rwunlock(&old_topology_lock);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to get topology cost information (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	s = sm_jm_encode_resp_get_cost_matrix(job, cost, costLen, outData, outLen);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to create the message response (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	return SA_JM_CMDRESP_OK;
}

static sa_jm_cmd_resp_t
sa_jm_cmd_get_use_matrix(Mai_t *mad, uint8_t *data, uint32_t len, uint8_t **outData, int *outLen)
{
	Status_t s;
	JmEntry_t * job;
	JmMsgReqGenericQuery_t input;

    INCREMENT_COUNTER(smCounterJmReqGetUseMatrix);

	s = sm_jm_decode_req_generic_query(data, len, &input);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to decode request (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	s = sm_jm_lookup_job(input.id, &job);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to lookup JobID "FMT_U64, input.id);
		return SA_JM_CMDRESP_INVALID_ID;
	}

	s = sm_jm_encode_resp_get_use_matrix(job, outData, outLen);
	if (s != VSTATUS_OK) {
		IB_LOG_WARN_FMT( __func__,
			"Failed to create the message response (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	return SA_JM_CMDRESP_OK;
}

static sa_jm_cmd_resp_t
sa_jm_cmd_get_jobs(Mai_t *mad, uint8_t *data, uint32_t len, uint8_t **outData, int *outLen)
{
	Status_t s;
	JmMsgRespGetJobsContext_t context;

    INCREMENT_COUNTER(smCounterJmReqGetJobs);

	s = sm_jm_iterate_over_jobs
		( sm_jm_encode_resp_get_jobs_start
		, sm_jm_encode_resp_get_jobs_iter
		, &context
		);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT( __func__,
			"Failed to create the message response (status %d)", s);
		return SA_JM_CMDRESP_ERROR;
	}

	*outData = context.buf;
	*outLen = context.blen;

	return SA_JM_CMDRESP_OK;
}

Status_t
sa_JobManagement(Mai_t *maip, sa_cntxt_t* sa_cntxt)
{
	uint32_t messageId;
	uint8_t *resp = NULL;
	int respLen = 0;
	uint8_t respStatus;

	IB_ENTER("sa_JobManagement", maip, sa_cntxt, 0, 0);

	// validate the method
	if (maip->base.method != SA_CM_GETMULTI) {
		IB_LOG_ERROR_FMT( __func__,
			"Invalid method (0x%02x)", maip->base.method);
		maip->base.status = MAD_STATUS_BAD_METHOD;
		goto reply;
	}

    INCREMENT_COUNTER(smCounterJmRequests);

	// validate the message range
	messageId = maip->base.amod;
	if (  messageId == 0
	   || messageId > JM_MAX_MESSAGE_ID
	   || sa_jm_cmd_dispatch_table[messageId] == NULL) {
		IB_LOG_ERROR_FMT( __func__,
			"Invalid message (0x%04x)", messageId);
		resp = &respStatus;
		resp[0] = JM_STATUS_ERROR;
		respLen = 1;
		goto reply;
	}

	if (messageId == JM_MSG_GET_COST_MATRIX) {
		/* PR 115089 - After disruptions the cost matrix reported for existing jobs
		 * will be incorrect because the topology switch indexes change, but the
		 * job specific switch indexes to topology switch indexes mapping is not updated.
		 * Instead of reporting an incorrect matrix, for now just do not support this query.
		 */
		IB_LOG_INFINI_INFO_FMT(__func__,
			 "Job management query for cost matrix of existing jobs is not supported !");
		resp = &respStatus;
		resp[0] = JM_STATUS_ERROR;
		respLen = 1;
		goto reply;
	}

	// dispatch the message to the appropriate function
	switch (sa_jm_cmd_dispatch_table[messageId](
		maip, (uint8_t *)sa_cntxt->reqData, sa_cntxt->reqDataLen, &resp, &respLen)) {
	case SA_JM_CMDRESP_OK:
		// respond with data and success status
		if (sm_config.debug_jm)
			IB_LOG_INFINI_INFO_FMT( __func__,
				"Successfully processed Job Management query 0x%04x", messageId);
		if (resp == NULL) { resp = &respStatus; respLen = 1; }
		resp[0] = JM_STATUS_OK;
		goto reply;
		break;
	case SA_JM_CMDRESP_PARTIAL:
		// respond with data and partial status
		IB_LOG_WARN_FMT( __func__,
			"Partially processed Job Management query 0x%04x", messageId);
		if (resp == NULL) { resp = &respStatus; respLen = 1; }
		resp[0] = JM_STATUS_PARTIAL;
		goto reply;
	case SA_JM_CMDRESP_ERROR:
		// respond with no data and error status
		IB_LOG_ERROR_FMT( __func__,
			"Failed to process Job Management query 0x%04x", messageId);
		resp = &respStatus;
		respLen = 1;
		resp[0] = JM_STATUS_ERROR;
		goto reply;
	case SA_JM_CMDRESP_INVALID_ID:
		// respond with no data and error status
		IB_LOG_ERROR_FMT( __func__,
			"Failed to process Job Management query 0x%04x: Invalid JobID", messageId);
		resp = &respStatus;
		respLen = 1;
		resp[0] = JM_STATUS_INVALID_ID;
		goto reply;
	}

reply:
	if (resp[0] & 0xfe)
	    INCREMENT_COUNTER(smCounterJmErrors);

	// note that we're leaving the attribute offset at 0
	(void)sa_cntxt_data(sa_cntxt, resp, respLen);
	(void)sa_send_reply(maip, sa_cntxt);

	if (resp != NULL && resp != (uint8_t *)&respStatus)
		sm_jm_free_resp(resp);

	IB_EXIT("sa_JobManagement", VSTATUS_OK);
	return VSTATUS_OK;
}

