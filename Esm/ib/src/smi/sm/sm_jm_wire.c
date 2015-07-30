/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include "sm_jm.h"

__inline void
hton16array(uint8_t *a, uint16_t v)
{
	a[0] = v >> 8;
	a[1] = v;
}

__inline void
hton64array(uint8_t *a, uint64_t v)
{
	a[0] = v >> 56;
	a[1] = v >> 48;
	a[2] = v >> 40;
	a[3] = v >> 32;
	a[4] = v >> 24;
	a[5] = v >> 16;
	a[6] = v >>  8;
	a[7] = v;
}

__inline uint16_t
ntoh16array(uint8_t *a)
{
	return (uint16_t)a[0] << 8 | a[1];
}

__inline uint64_t
ntoh64array(uint8_t *a)
{
	return (uint64_t)a[0] << 56
	     | (uint64_t)a[1] << 48
	     | (uint64_t)a[2] << 40
	     | (uint64_t)a[3] << 32
	     | (uint64_t)a[4] << 24
	     | (uint64_t)a[5] << 16
	     | (uint64_t)a[6] <<  8
	     | (uint64_t)a[7];
}

//=============================================================================
// GENERAL PURPOSE RESPONSE FREE
//

void
sm_jm_free_resp(uint8_t * buf)
{
	if (buf != NULL)
		vs_pool_free(&sm_pool, buf);
}

//=============================================================================
// TYPES
//

#define JM_NSIZE_STATUS 1
#define JM_NSIZE_JOBID 8
#define JM_NSIZE_TIMESTAMP 8
#define JM_NSIZE_OPTIONS 2
#define JM_NSIZE_PARAMS 144
#define JM_NSIZE_POLLSTATUS 1

#define JM_NSIZE_GUIDLIST_HEADER 2
#define JM_NSIZE_GUIDLIST_ELEMENT 8
#define JM_NSIZE_GUIDLIST_ELEMENTS(N) \
	((N) * JM_NSIZE_GUIDLIST_ELEMENT)
#define JM_NSIZE_GUIDLIST(N) \
	(JM_NSIZE_GUIDLIST_HEADER + JM_NSIZE_GUIDLIST_ELEMENTS(N))

#define JM_NSIZE_SWITCHMAP_HEADER 4
#define JM_NSIZE_SWITCHMAP_ELEMENT 2
#define JM_NSIZE_SWITCHMAP_ELEMENTS(N) \
	((N) * JM_NSIZE_SWITCHMAP_ELEMENT)
#define JM_NSIZE_SWITCHMAP(N) \
	(JM_NSIZE_SWITCHMAP_HEADER + JM_NSIZE_SWITCHMAP_ELEMENTS(N))

#define JM_NSIZE_COSTMATRIX_HEADER 2
#define JM_NSIZE_COSTMATRIX_ELEMENT 2
#define JM_NSIZE_COSTMATRIX_ELEMENTS(N) \
	((N) * JM_NSIZE_COSTMATRIX_ELEMENT)
#define JM_NSIZE_COSTMATRIX(N) \
	(JM_NSIZE_COSTMATRIX_HEADER + JM_NSIZE_COSTMATRIX_ELEMENTS(N))

#define JM_NSIZE_USEMATRIX_HEADER 5
#define JM_NSIZE_USEMATRIX_ELEMENT 5
#define JM_NSIZE_USEMATRIX_ELEMENTS(N) \
	((N) * JM_NSIZE_USEMATRIX_ELEMENT)
#define JM_NSIZE_USEMATRIX(N) \
	(JM_NSIZE_USEMATRIX_HEADER + JM_NSIZE_USEMATRIX_ELEMENTS(N))

#define JM_NSIZE_JOBLIST_HEADER 2
#define JM_NSIZE_JOBLIST_ELEMENT \
	(2 + JM_NSIZE_JOBID + JM_NSIZE_TIMESTAMP + JM_NSIZE_PARAMS)
#define JM_NSIZE_JOBLIST_ELEMENTS(N) \
	((N) * JM_NSIZE_JOBLIST_ELEMENT)
#define JM_NSIZE_JOBLIST(N) \
	(JM_NSIZE_JOBLIST_HEADER + JM_NSIZE_JOBLIST_ELEMENTS(N))

static Status_t
sm_jm_decode_type_job_options
	( uint8_t * buf
	, int blen
	, JmWireJobOptions_t * options
	, int * bytes
	)
{
	if (blen < JM_NSIZE_OPTIONS) {
		IB_LOG_ERROR_FMT(__func__, "Buffer length too short: %d", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}
	options->no_create = ntoh16array(buf) & 0x0001;
	*bytes = JM_NSIZE_OPTIONS;
	return VSTATUS_OK;
}

static Status_t
sm_jm_decode_type_job_params
	( uint8_t * buf
	, int blen
	, JmWireJobParams_t * params
	, int * bytes
	)
{
	if (blen < JM_NSIZE_PARAMS) {
		IB_LOG_ERROR_FMT(__func__, "Buffer length too short: %d", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}
	memcpy(params->name, buf, 64); params->name[64] = 0;
	memcpy(params->appName, buf + 64, 64); params->appName[64] = 0;
	params->pid = ntoh64array(buf + 128);
	params->uid = ntoh64array(buf + 136);
	*bytes = JM_NSIZE_PARAMS;
	return VSTATUS_OK;
}

static Status_t
sm_jm_encode_type_job_params
	( uint8_t * buf
	, int blen
	, JmEntry_t * job
	, int * bytes
	)
{
	if (blen < JM_NSIZE_PARAMS) {
		IB_LOG_ERROR_FMT(__func__, "Buffer length too short: %d", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}
	memcpy(buf, job->params.name, 64);
	memcpy(buf + 64, job->params.appName, 64);
	hton64array(buf + 128, job->params.pid);
	hton64array(buf + 136, job->params.uid);
	*bytes = JM_NSIZE_PARAMS;
	return VSTATUS_OK;
}

static Status_t
sm_jm_encode_type_timestamp
	( uint8_t * buf
	, int blen
	, JmEntry_t * job
	, int * bytes
	)
{
	if (blen < JM_NSIZE_TIMESTAMP) {
		IB_LOG_ERROR_FMT(__func__, "Buffer length too short: %d", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}
	hton64array(buf, job->timestamp);
	*bytes = JM_NSIZE_TIMESTAMP;
	return VSTATUS_OK;
}

static Status_t
sm_jm_decode_type_guid_list
	( uint8_t * buf
	, int blen
	, JmWireGuidList_t * guids
	, int * bytes
	)
{
	Status_t s;
	int i;

	if (blen < JM_NSIZE_GUIDLIST_HEADER) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short to decode count", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}

	guids->count = ntoh16array(buf);

	buf += JM_NSIZE_GUIDLIST_HEADER;
	blen -= JM_NSIZE_GUIDLIST_HEADER;

	if (blen < JM_NSIZE_GUIDLIST_ELEMENTS(guids->count)) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short to decode %d guids", blen, guids->count);
		*bytes = 0;
		return VSTATUS_BAD;
	}

	s = vs_pool_alloc(&sm_pool, guids->count * sizeof(uint64_t),
		(void *)&guids->entries);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate GUID buffer (status %d)", s);
		*bytes = 0;
		return s;
	}

	for (i = 0; i < guids->count; ++i, buf += JM_NSIZE_GUIDLIST_ELEMENT)
		guids->entries[i] = ntoh64array(buf);

	*bytes = JM_NSIZE_GUIDLIST(guids->count);

	return VSTATUS_OK;
}

static void
sm_jm_free_type_guid_list(JmWireGuidList_t * guids)
{
	if (guids->count > 0 && guids->entries != NULL)
		vs_pool_free(&sm_pool, guids->entries);
}

static Status_t
sm_jm_encode_type_guid_list
	( uint8_t * buf
	, int blen
	, JmEntry_t * job
	, int * bytes
	)
{
	int i;

	if (blen < JM_NSIZE_GUIDLIST(job->portCount)) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short to encode %d guids", blen, job->portCount);
		*bytes = 0;
		return VSTATUS_BAD;
	}

	hton16array(buf, job->portCount);

	buf += JM_NSIZE_GUIDLIST_HEADER;

	for (i = 0; i < job->portCount; ++i, buf += JM_NSIZE_GUIDLIST_ELEMENT)
		hton64array(buf, job->ports[i].guid);

	*bytes = JM_NSIZE_GUIDLIST(job->portCount);

	return VSTATUS_OK;
}

static Status_t
sm_jm_decode_type_job_id(uint8_t * buf, int blen, uint64_t * id, int * bytes)
{
	if (blen < JM_NSIZE_JOBID) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}
	*id = ntoh64array(buf);
	*bytes = JM_NSIZE_JOBID;
	return VSTATUS_OK;
}

static Status_t
sm_jm_encode_type_job_id(uint8_t * buf, int blen, JmEntry_t * job, int * bytes)
{
	if (blen < JM_NSIZE_JOBID) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}
	hton64array(buf, job->id);
	*bytes = JM_NSIZE_JOBID;
	return VSTATUS_OK;
}

static Status_t
sm_jm_encode_type_switch_map
	( uint8_t *buf
	, int blen
	, JmEntry_t * job
	, int * bytes
	)
{
	int i;
	if (blen < JM_NSIZE_SWITCHMAP(job->portCount)) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short to encode %d entries", blen, job->portCount);
		*bytes = 0;
		return VSTATUS_BAD;
	}
	hton16array(buf, job->portCount);
	hton16array(buf + 2, job->switchCount);
	buf += JM_NSIZE_SWITCHMAP_HEADER;
	for (i = 0; i < job->portCount; ++i, buf += JM_NSIZE_SWITCHMAP_ELEMENT)
		hton16array(buf, job->ports[i].jobSwIdx);
	*bytes = JM_NSIZE_SWITCHMAP(job->portCount);
	return VSTATUS_OK;
}

static Status_t
sm_jm_encode_type_cost_matrix
	( uint8_t * buf
	, int blen
	, JmEntry_t * job
	, uint16_t * cost
	, int clen
	, int * bytes
	)
{
	int i;
	if (blen < JM_NSIZE_COSTMATRIX(clen)) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short to encode %d entries", blen, clen);
		*bytes = 0;
		return VSTATUS_BAD;
	}
	hton16array(buf, job->switchCount);
	buf += JM_NSIZE_COSTMATRIX_HEADER;
	for (i = 0; i < clen; ++i, buf += JM_NSIZE_COSTMATRIX_ELEMENT)
		hton16array(buf, cost[i]);
	*bytes = JM_NSIZE_COSTMATRIX(clen);
	return VSTATUS_OK;
}

static Status_t
sm_jm_encode_type_poll_status
	( uint8_t * buf
	, int blen
	, JmPollStatus_t ps
	, int * bytes
	)
{
	if (blen < JM_NSIZE_POLLSTATUS) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}
	buf[0] = (uint8_t)ps;
	*bytes = JM_NSIZE_POLLSTATUS;
	return VSTATUS_OK;
}

static Status_t
sm_jm_decode_type_use_element
	( uint8_t * buf
	, int blen
	, JmWireUseElement_t * element
	, int * bytes
	)
{
	if (blen < JM_NSIZE_USEMATRIX_ELEMENT) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short to decode use element", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}

	element->bursty = buf[0] & 0x80 ? 1 : 0;
	element->swidx = ((buf[0] & 0x7f) << 8) | buf[1];
	element->dlid = ntoh16array(buf + 2);
	element->use = buf[4];

	*bytes = JM_NSIZE_USEMATRIX_ELEMENT;

	return VSTATUS_OK;
}

static Status_t
sm_jm_encode_type_use_element
	( uint8_t * buf
	, int blen
	, JmWireUseElement_t * element
	, int * bytes
	)
{
	if (blen < JM_NSIZE_USEMATRIX_ELEMENT) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short to encode use element", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}

	hton16array(buf, element->swidx & 0x7fff);
	buf[0] |= element->bursty << 7;
	hton16array(buf + 2, element->dlid);
	buf[4] = element->use;

	*bytes = JM_NSIZE_USEMATRIX_ELEMENT;

	return VSTATUS_OK;
}

static Status_t
sm_jm_decode_type_use_matrix
	( uint8_t * buf
	, int blen
	, JmWireUseMatrix_t * matrix
	, int * bytes
	)
{
	Status_t s;
	int i, j;

	if (blen < JM_NSIZE_USEMATRIX_HEADER) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short to decode use matrix header", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}

	matrix->multiplier = ntoh16array(buf);
	matrix->base = buf[2];
	matrix->count = ntoh16array(buf + 3);
	matrix->elements = NULL;

	buf += JM_NSIZE_USEMATRIX_HEADER;
	blen -= JM_NSIZE_USEMATRIX_HEADER;

	if (matrix->count == 0) {
		*bytes = JM_NSIZE_USEMATRIX_HEADER;
		return VSTATUS_OK;
	}

	if (blen < JM_NSIZE_USEMATRIX_ELEMENTS(matrix->count)) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short to decode %d use elements", blen, matrix->count);
		*bytes = 0;
		return VSTATUS_BAD;
	}

	s = vs_pool_alloc(&sm_pool, matrix->count * sizeof(JmWireUseMatrix_t),
		(void *)&matrix->elements);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate use element buffer (status %d)", s);
		*bytes = 0;
		return s;
	}

	for (i = j = 0; i < matrix->count; ++i, buf += j, blen -= j) {
		s = sm_jm_decode_type_use_element(buf, blen, matrix->elements + i, &j);
		if (s != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__,
				"Failed to decode use element %d (status %d)", i, s);
			*bytes = 0;
			return s;
		}
	}

	*bytes = JM_NSIZE_USEMATRIX(matrix->count);

	return VSTATUS_OK;
}

// not currently used.  we don't process the use matrix, so rather than
// process and free it, we just copy the buffer into the job
#if 0
static void
sm_jm_free_type_use_matrix(JmWireUseMatrix_t * matrix)
{
	if (matrix->count > 0 && matrix->elements != NULL)
		vs_pool_free(&sm_pool, matrix->elements);
}
#endif

static Status_t
sm_jm_encode_type_use_matrix
	( uint8_t * buf
	, int blen
	, JmEntry_t * job
	, int * bytes
	)
{
	Status_t s;
	int i, j;

	if (blen < JM_NSIZE_USEMATRIX(job->useMatrix.count)) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short to encode use matrix of %d elements",
			blen, job->useMatrix.count);
		*bytes = 0;
		return VSTATUS_BAD;
	}

	hton16array(buf, job->useMatrix.multiplier);
	buf[2] = job->useMatrix.base;
	hton16array(buf + 3, job->useMatrix.count);

	buf += JM_NSIZE_USEMATRIX_HEADER;
	blen -= JM_NSIZE_USEMATRIX_HEADER;

	for (i = j = 0; i < job->useMatrix.count; ++i, buf += j, blen -= j) {
		s = sm_jm_encode_type_use_element(buf, blen, job->useMatrix.elements + i, &j);
		if (s != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__,
				"Failed to encode use element %d (status %d)", i, s);
			*bytes = 0;
			return s;
		}
	}

	*bytes = JM_NSIZE_USEMATRIX(job->useMatrix.count);

	return VSTATUS_OK;
}

static Status_t
sm_jm_encode_type_job_info
	( uint8_t * buf
	, int blen
	, JmEntry_t * job
	, int * bytes
	)
{
	Status_t s;
	int b;

	if (blen < JM_NSIZE_JOBLIST_ELEMENT) {
		IB_LOG_ERROR_FMT(__func__,
			"Buffer length (%d) too short", blen);
		*bytes = 0;
		return VSTATUS_BAD;
	}

	// job id
	s = sm_jm_encode_type_job_id(buf, blen, job, &b);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to encode JobID (status %d)", s);
		*bytes = 0;
		return s;
	}
	buf += b; blen -= b;

	// timestamp
	s = sm_jm_encode_type_timestamp(buf, blen, job, &b);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to encode JobID (status %d)", s);
		*bytes = 0;
		return s;
	}
	buf += b; blen -= b;

	// flags
	switch (job->state) {
	case JM_STATE_USE_AVAILABLE: hton16array(buf, 0x0001); break;
	case JM_STATE_ROUTED:        hton16array(buf, 0x0003); break;
	default:                     hton16array(buf, 0x0000); break;
	}
	buf += 2; blen -= 2;

	// params
	s = sm_jm_encode_type_job_params(buf, blen, job, &b);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to encode params (status %d)", s);
		*bytes = 0;
		return s;
	}

	*bytes = JM_NSIZE_JOBLIST_ELEMENT;

	return VSTATUS_OK;
}

//=============================================================================
// MESSAGE: GENERIC QUERY (by job id)
//

Status_t
sm_jm_decode_req_generic_query
	( uint8_t * buf
	, int blen
	, JmMsgReqGenericQuery_t * msg
	)
{
	Status_t s;
	int bytes;

	s = sm_jm_decode_type_job_id(buf, blen, &msg->id, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to decode JobId (status %d)", s);
		return s;
	}

	return VSTATUS_OK;
}

//=============================================================================
// MESSAGE: CREATE
//

Status_t
sm_jm_decode_req_create(uint8_t * buf, int blen, JmMsgReqCreate_t * msg)
{
	Status_t s;
	int bytes;

	s = sm_jm_decode_type_job_options(buf, blen, &msg->options, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to decode options (status %d)", s);
		return s;
	}

	buf += bytes; blen -= bytes;

	s = sm_jm_decode_type_job_params(buf, blen, &msg->params, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to decode name (status %d)", s);
		return s;
	}

	buf += bytes; blen -= bytes;

	s = sm_jm_decode_type_guid_list(buf, blen, &msg->guids, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to decode GUID list (status %d)", s);
		return s;
	}

	return VSTATUS_OK;
}

void
sm_jm_free_req_create(JmMsgReqCreate_t * msg)
{
	sm_jm_free_type_guid_list(&msg->guids);
	memset(msg, 0, sizeof(*msg));
}

Status_t
sm_jm_encode_resp_create
	( JmEntry_t * job
	, uint16_t * cost
	, int clen
	, uint8_t ** outBuf
	, int * outLen
	)
{
	Status_t s;
	int len, bytes;
	uint8_t * buf;

	len = JM_NSIZE_STATUS
	    + JM_NSIZE_JOBID
	    + JM_NSIZE_SWITCHMAP(job->portCount)
	    + JM_NSIZE_COSTMATRIX(clen);
	s = vs_pool_alloc(&sm_pool, len, (void *)&buf);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate response buffer (status %d)", s);
		return s;
	}

	*outBuf = buf;
	*outLen = len;

	// skip status
	buf += JM_NSIZE_STATUS;
	len -= JM_NSIZE_STATUS;

	s = sm_jm_encode_type_job_id(buf, len, job, &bytes);
	if (s != VSTATUS_OK) {
		vs_pool_free(&sm_pool, outBuf);
		IB_LOG_ERROR_FMT(__func__,
			"Failed to encode name (status %d)", s);
		return s;
	}

	buf += bytes; len -= bytes;

	s = sm_jm_encode_type_switch_map(buf, len, job, &bytes);
	if (s != VSTATUS_OK) {
		vs_pool_free(&sm_pool, outBuf);
		IB_LOG_ERROR_FMT(__func__,
			"Failed to encode switch map (status %d)", s);
		return s;
	}

	buf += bytes; len -= bytes;

	s = sm_jm_encode_type_cost_matrix(buf, len, job, cost, clen, &bytes);
	if (s != VSTATUS_OK) {
		vs_pool_free(&sm_pool, outBuf);
		IB_LOG_ERROR_FMT(__func__,
			"Failed to encode cost matrix (status %d)", s);
		return s;
	}

	return VSTATUS_OK;
}

//=============================================================================
// MESSAGE: SET USE MATRIX
//

Status_t
sm_jm_decode_req_set_use_matrix
	( uint8_t * buf
	, int blen
	, JmMsgReqSetUseMatrix_t * msg
	)
{
	Status_t s;
	int bytes;

	s = sm_jm_decode_type_job_id(buf, blen, &msg->id, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to decode job id (status %d)", s);
		return s;
	}

	buf += bytes; blen -= bytes;

	s = sm_jm_decode_type_use_matrix(buf, blen, &msg->matrix, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to decode use matrix (status %d)", s);
		return s;
	}

	return VSTATUS_OK;
}

//=============================================================================
// MESSAGE: POLL
//

Status_t
sm_jm_encode_resp_poll(JmEntry_t * job, uint8_t ** outBuf, int * outLen)
{
	Status_t s;
	int len, bytes;
	uint8_t * buf;
	JmPollStatus_t ps;

	ps = job->state == JM_STATE_ROUTED
	   ? JM_POLL_READY
	   : JM_POLL_NOT_READY;

	len = JM_NSIZE_STATUS + JM_NSIZE_POLLSTATUS;
	s = vs_pool_alloc(&sm_pool, len, (void *)&buf);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate response buffer (status %d)", s);
		return s;
	}

	*outBuf = buf;
	*outLen = len;

	buf += JM_NSIZE_STATUS;
	len -= JM_NSIZE_STATUS;

	s = sm_jm_encode_type_poll_status(buf, len, ps, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Failed to encode poll status (status %d)", s);
		return s;
	}

	return VSTATUS_OK;
}

//=============================================================================
// MESSAGE: GET GUID LIST
//

Status_t
sm_jm_encode_resp_get_guid_list
	( JmEntry_t * job
	, uint8_t ** outBuf
	, int * outLen
	)
{
	Status_t s;
	int bytes, blen;
	uint8_t * buf;

	blen = JM_NSIZE_STATUS
	     + JM_NSIZE_GUIDLIST(job->portCount);
	s = vs_pool_alloc(&sm_pool, blen, (void *)&buf);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate response buffer (status %d)", s);
		return s;
	}

	*outBuf = buf;
	*outLen = blen;

	buf += JM_NSIZE_STATUS;
	blen -= JM_NSIZE_STATUS;

	s = sm_jm_encode_type_guid_list(buf, blen, job, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to encode guid list (status %d)", s);
		return s;
	}

	return VSTATUS_OK;
}

//=============================================================================
// MESSAGE: GET SWITCH MAP
//

Status_t
sm_jm_encode_resp_get_switch_map
	( JmEntry_t * job
	, uint8_t ** outBuf
	, int * outLen
	)
{
	Status_t s;
	int bytes, blen;
	uint8_t * buf;

	blen = JM_NSIZE_STATUS
	     + JM_NSIZE_SWITCHMAP(job->portCount);
	s = vs_pool_alloc(&sm_pool, blen, (void *)&buf);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate response buffer (status %d)", s);
		return s;
	}

	*outBuf = buf;
	*outLen = blen;

	buf += JM_NSIZE_STATUS;
	blen -= JM_NSIZE_STATUS;

	s = sm_jm_encode_type_switch_map(buf, blen, job, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Failed to encode switch map (status %d)", s);
		return s;
	}

	return VSTATUS_OK;
}

//=============================================================================
// MESSAGE: GET COST MATRIX
//

Status_t
sm_jm_encode_resp_get_cost_matrix
	( JmEntry_t * job
	, uint16_t * cost
	, int clen
	, uint8_t ** outBuf
	, int * outLen
	)
{
	Status_t s;
	int bytes, blen;
	uint8_t * buf;

	blen = JM_NSIZE_STATUS
	     + JM_NSIZE_COSTMATRIX(clen);
	s = vs_pool_alloc(&sm_pool, blen, (void *)&buf);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate response buffer (status %d)", s);
		return s;
	}

	*outBuf = buf;
	*outLen = blen;

	buf += JM_NSIZE_STATUS;
	blen -= JM_NSIZE_STATUS;

	s = sm_jm_encode_type_cost_matrix(buf, blen, job, cost, clen, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to encode cost matrix (status %d)", s);
		return s;
	}

	return VSTATUS_OK;
}

//=============================================================================
// MESSAGE: GET USE MATRIX
//

Status_t
sm_jm_encode_resp_get_use_matrix
	( JmEntry_t * job
	, uint8_t ** outBuf
	, int * outLen
	)
{
	Status_t s;
	int bytes, blen;
	uint8_t * buf;

	blen = JM_NSIZE_STATUS
	     + JM_NSIZE_USEMATRIX(job->useMatrix.count);
	s = vs_pool_alloc(&sm_pool, blen, (void *)&buf);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate response buffer (status %d)", s);
		return s;
	}

	*outBuf = buf;
	*outLen = blen;

	buf += JM_NSIZE_STATUS;
	blen -= JM_NSIZE_STATUS;

	s = sm_jm_encode_type_use_matrix(buf, blen, job, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to encode use matrix (status %d)", s);
		return s;
	}

	return VSTATUS_OK;
}

//=============================================================================
// MESSAGE: GETJOBS
//

Status_t
sm_jm_encode_resp_get_jobs_start(uint32_t count, void * context)
{
	Status_t s;
	JmMsgRespGetJobsContext_t * c = context;

	c->count = (uint16_t)count;
	c->blen = JM_NSIZE_STATUS
	        + JM_NSIZE_JOBLIST(count);
	c->bpos = JM_NSIZE_STATUS + JM_NSIZE_JOBLIST_HEADER;
	
	s = vs_pool_alloc(&sm_pool, c->blen, (void *)&c->buf);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate response buffer (status %d)", s);
		return s;
	}

	hton16array(c->buf + JM_NSIZE_STATUS, count);

	return VSTATUS_OK;
}

Status_t
sm_jm_encode_resp_get_jobs_iter(JmEntry_t * job, void * context)
{
	Status_t s;
	int bytes;
	JmMsgRespGetJobsContext_t * c = context;

	s = sm_jm_encode_type_job_info(c->buf + c->bpos, c->blen - c->bpos, job, &bytes);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to encode job info (status %d)", s);
		return s;
	}

	c->bpos += bytes;

	return VSTATUS_OK;
}

