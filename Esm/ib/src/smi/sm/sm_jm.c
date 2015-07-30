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

static JmTable_t smJobTable;

//=============================================================================
// UTILITY FUNCTIONS
//

static __inline__ uint64_t
sm_jm_generate_random_id(void)
{
	return ((uint64_t)rand() << 33)
	     | ((uint64_t)rand() << 2)
	     | ((uint64_t)rand() & 3);
}

//=============================================================================
// INITIALIZATION
//

static uint64_t
sm_jm_hash_from_key(void * k)
{
    return ((JmEntry_t *)k)->id;
}

static int32_t
sm_jm_hash_compare(void * k1, void * k2)
{
	if (((JmEntry_t *)k1)->id == ((JmEntry_t *)k2)->id)
		return 1;
	else
		return 0;
}

Status_t
sm_jm_init_job_table(void)
{
	Status_t s;

	smJobTable.jobs = cs_create_hashtable("sm_job_table", 16,
		sm_jm_hash_from_key, sm_jm_hash_compare, CS_HASH_KEY_NOT_ALLOCATED);
	if (smJobTable.jobs == NULL) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to create job table hash");
		s = VSTATUS_BAD;
		goto fail1;
	}

	s = vs_lock_init(&smJobTable.lock, VLOCK_FREE, VLOCK_THREAD);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to create job table lock (status %d)", s);
		goto fail2;
	}

	return VSTATUS_OK;

fail2:
	cs_hashtable_destroy(smJobTable.jobs, 0);
fail1:
	memset(&smJobTable, 0, sizeof(smJobTable));
	return s;
}

void
sm_jm_destroy_job_table(void)
{
    if (smJobTable.jobs) {
		(void)vs_lock(&smJobTable.lock);
		cs_hashtable_destroy(smJobTable.jobs, 0);
		smJobTable.jobs = 0;
		(void)vs_unlock(&smJobTable.lock);
		(void)vs_lock_delete(&smJobTable.lock);
	}
	memset(&smJobTable, 0, sizeof(smJobTable));
}

//=============================================================================
// JOB MANAGEMENT
//

Status_t
sm_jm_iterate_over_jobs(sm_jm_start_func_t fs, sm_jm_iter_func_t fi, void *context)
{
	Status_t s = VSTATUS_OK;
	CS_HashTableItr_t iter;
	JmEntry_t * job;
	uint32_t count;

	s = vs_lock(&smJobTable.lock);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to take job table lock (status %d)", s);
		return s;
	}

	count = cs_hashtable_count(smJobTable.jobs);

	s = fs(count, context);
	if (s != VSTATUS_OK) {
		(void)vs_unlock(&smJobTable.lock);
		return s;
	}

	if (count) {
		cs_hashtable_iterator(smJobTable.jobs, &iter);
		do {
			job = cs_hashtable_iterator_value(&iter);
			s = fi(job, context);
			if (s != VSTATUS_OK) break;
		} while (cs_hashtable_iterator_advance(&iter));
	}

	(void)vs_unlock(&smJobTable.lock);

	return s;
}

//=============================================================================
// JOB MANAGEMENT
//

Status_t
sm_jm_insert_job(JmEntry_t *job)
{
	Status_t s;

	s = vs_lock(&smJobTable.lock);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to take job table lock (status %d)", s);
		return s;
	}

	if (cs_hashtable_count(smJobTable.jobs) >= 65536) {
		IB_LOG_ERROR_FMT(__func__,
			"Maximum job count (65536) reached (status %d)", s);
		return s;
	}


	do job->id = sm_jm_generate_random_id();
	while (cs_hashtable_search(smJobTable.jobs, job) != NULL);

	if (!cs_hashtable_insert(smJobTable.jobs, job, job)) {
		(void)vs_unlock(&smJobTable.lock);
		job->id = 0;
		IB_LOG_ERROR_FMT(__func__, "Failed to add job to job table");
		return s;
	}

	(void)vs_unlock(&smJobTable.lock);

	return VSTATUS_OK;
}

Status_t
sm_jm_remove_job(JmEntry_t *job)
{
	Status_t s;
	JmEntry_t *j;

	s = vs_lock(&smJobTable.lock);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to take job table lock (status %d)", s);
		return s;
	}

	j = cs_hashtable_remove(smJobTable.jobs, job);
	if (j == NULL) {
		IB_LOG_WARN_FMT(__func__,
			"Job does not exist in job table");
	} else if (j != job) {
		IB_LOG_WARN_FMT(__func__,
			"Removed an unexpected job (%p != %p); Job table potentially inconsistent", job, j);
	}

	(void)vs_unlock(&smJobTable.lock);

	return VSTATUS_OK;
}

Status_t
sm_jm_lookup_job(uint64_t id, JmEntry_t ** job)
{
	Status_t s;
	JmEntry_t j_query;
	JmEntry_t *j;

	s = vs_lock(&smJobTable.lock);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to take job table lock (status %d)", s);
		return s;
	}
	j_query.id = id;
	j = cs_hashtable_search(smJobTable.jobs, &j_query);
	(void)vs_unlock(&smJobTable.lock);
	if (j == NULL) return VSTATUS_BAD;
	*job = j;
	return VSTATUS_OK;
}

Status_t
sm_jm_alloc_job(JmEntry_t **job)
{
	Status_t s;
	JmEntry_t *j;

	s = vs_lock(&smJobTable.lock);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to take job table lock (status %d)", s);
		return s;
	}

	s = vs_pool_alloc(&sm_pool, sizeof(JmEntry_t), (void *)&j);
	if (s != VSTATUS_OK) {
		(void)vs_unlock(&smJobTable.lock);
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate job entry (status %d)", s);
		return s;
	}

	(void)vs_unlock(&smJobTable.lock);
	memset(j, 0, sizeof(*j));
	*job = j;

	return VSTATUS_OK;
}

Status_t
sm_jm_free_job(JmEntry_t *job)
{
	if (job->ports != NULL) vs_pool_free(&sm_pool, job->ports);
	if (job->jobSwToTopoSwMap != NULL) vs_pool_free(&sm_pool, job->jobSwToTopoSwMap);
	if (job->useMatrix.elements != NULL) vs_pool_free(&sm_pool, job->useMatrix.elements);
	vs_pool_free(&sm_pool, job);

	return VSTATUS_OK;
}

//=============================================================================
// TOPOLOGY DATA COLLECTION
//

// outCount wil be set to the number of ports actually found and valid in the
// topology
//
Status_t
sm_jm_fill_ports
	( Topology_t *topop
	, JmMsgReqCreate_t *input
	, JmEntry_t *job
	, uint16_t *outCount
	)
{
	Status_t s;
	int i;
	Node_t *nodep;
	Port_t *portp;
	int count = 0, nextIdx = 0;
	uint16_t *topoSwToJobSwMap;

	s = vs_pool_alloc(&sm_pool, input->guids.count * sizeof(JmPort_t),
		(void *)&job->ports);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate space for PortGuid list (status %d)", s);
		goto fail1;
	}

	s = vs_pool_alloc(&sm_pool, input->guids.count * sizeof(uint16_t),
		(void *)&job->jobSwToTopoSwMap);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate space for switch translation map (status %d)", s);
		goto fail2;
	}

	s = vs_pool_alloc(&sm_pool, topop->max_sws * sizeof(uint16_t),
		(void *)&topoSwToJobSwMap);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate space for switch translation map (status %d)", s);
		goto fail3;
	}
	memset(topoSwToJobSwMap, 0xff, topop->max_sws * sizeof(uint16_t));

	// for each portguid: find the port/switch in the topology
	for (i = 0; i < input->guids.count; ++i) {
		job->ports[i].guid = input->guids.entries[i];
		portp = sm_find_port_guid(topop, input->guids.entries[i]);
		if (  !sm_valid_port(portp)
		   || portp->state < IB_PORT_INIT
		   || portp->portData->nodePtr->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			// port not found; clear port in job
			job->ports[i].portp = NULL;
			job->ports[i].jobSwIdx = 0xffff;
		} else {
			// port found; find neighbor switch
			job->ports[i].portp = portp;
			nodep = sm_find_node(topop, portp->nodeno);
			if (nodep == NULL) {
				// switch not found; clear port in job
				job->ports[i].jobSwIdx = 0xffff;
			} else {
				// switch found; determine job-specific switch index
				if (topoSwToJobSwMap[nodep->swIdx] != 0xffff) {
					// index already created; use it
					job->ports[i].jobSwIdx = topoSwToJobSwMap[nodep->swIdx];
				} else {
					// index not created; use next available
					job->ports[i].jobSwIdx = topoSwToJobSwMap[nodep->swIdx] = nextIdx;
					job->jobSwToTopoSwMap[nextIdx] = nodep->swIdx;
					++nextIdx;
				}
				++count;
			}
		}
	}

	(void)vs_pool_free(&sm_pool, topoSwToJobSwMap);

	job->portCount = input->guids.count;
	job->switchCount = nextIdx;
	*outCount = count;

	return VSTATUS_OK;

fail3:
	(void)vs_pool_free(&sm_pool, job->jobSwToTopoSwMap);
fail2:
	(void)vs_pool_free(&sm_pool, job->ports);
fail1:
	return VSTATUS_BAD;
}

Status_t
sm_jm_get_cost
	( Topology_t *topop
	, JmEntry_t *job
	, uint16_t **outCost
	, int *outLen
	)
{
	Status_t s;
	uint16_t *cost;
	int len;
	int pos = 0;
	uint16_t js1, js2; // job switch indicies
	uint16_t ts1, ts2; // topology switch indices

	// don't allocate for the case of 1 switch
	if (job->switchCount <= 1) {
		*outCost = NULL;
		*outLen = 0;
		return VSTATUS_OK;
	}

	// allocate space for triangular matrix minus the diagonal, so:
	//   1 + 2 + ... + (n - 1) ==> n * (n - 1) / 2
	len = job->switchCount * (job->switchCount - 1) / 2;
	s = vs_pool_alloc(&sm_pool, len * sizeof(uint16_t), (void *)&cost);
	if (s != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate space for cost matrix (status %d)", s);
		return VSTATUS_BAD;
	}

	// encode all <src,dst> pairs where src < dst (upper-right triangle)
	for (js1 = 0; js1 < job->switchCount; ++js1) {
		ts1 = job->jobSwToTopoSwMap[js1];
		if (ts1 == 0xffff) {
			memset(cost + pos, 0xff, (job->switchCount - js1 - 1) * sizeof(uint16_t));
			pos += job->switchCount - js1 - 1;
			continue;
		}
		for (js2 = js1 + 1; js2 < job->switchCount; ++js2) {
			ts2 = job->jobSwToTopoSwMap[js2];
			if (ts2 == 0xffff)
				cost[pos++] = 0xffff;
			else
				cost[pos++] = topop->cost[Index(ts1, ts2)];
		}
	}

	*outCost = cost;
	*outLen = len;

	return VSTATUS_OK;
}

void
sm_jm_free_cost(uint16_t *cost)
{
	(void)vs_pool_free(&sm_pool, cost);
}

