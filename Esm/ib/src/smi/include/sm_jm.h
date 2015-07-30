/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef	_SM_MPI_H_
#define	_SM_MPI_H_

#include "sm_l.h"

//=============================================================================
// GENERAL CONSTANTS
//

#define JM_CVERSION 1

//=============================================================================
// PROTOCOL CONSTANTS
//

#define JM_MAX_MESSAGE_ID      0x000a

#define JM_MSG_CLASS_PORT_INFO 0x0001
#define JM_MSG_CREATE_JOB      0x0002
#define JM_MSG_SET_USE_MATRIX  0x0003
#define JM_MSG_POLL_READY      0x0004
#define JM_MSG_COMPLETE_JOB    0x0005
#define JM_MSG_GET_GUID_LIST   0x0006
#define JM_MSG_GET_SWITCH_MAP  0x0007
#define JM_MSG_GET_COST_MATRIX 0x0008
#define JM_MSG_GET_USE_MATRIX  0x0009
#define JM_MSG_GET_JOBS        0x000a

#define JM_STATUS_OK           0x00000000
#define JM_STATUS_PARTIAL      0x00000001
#define JM_STATUS_ERROR        0x00000002
#define JM_STATUS_INVALID_ID   0x00000003

//=============================================================================
// WIRE/INPUT STRUCTURES
//

typedef struct _JmWireGuidList {
	uint16_t count;
	uint64_t * entries;
} JmWireGuidList_t;

typedef struct _JmWireUseElement {
	uint16_t bursty : 1;
	uint16_t swidx : 15;
	uint16_t dlid;
	uint8_t use;
} JmWireUseElement_t;

typedef struct _JmWireUseMatrix {
	uint16_t multiplier;
	uint8_t base;
	uint16_t count;
	JmWireUseElement_t * elements;
} JmWireUseMatrix_t;

typedef struct _JmWireJobOptions {
	uint16_t rsvd      : 15;
	uint16_t no_create : 1;
} JmWireJobOptions_t;

typedef struct _JmWireJobParams {
	char name[65];
	char appName[65];
	uint64_t pid;
	uint64_t uid;
} JmWireJobParams_t;

typedef struct _JmWireJobInfo {
	uint64_t id;
	uint64_t timestamp;
	uint16_t rsvd    : 14;
	uint16_t routed  : 1;
	uint16_t has_use : 1;
	JmWireJobParams_t params;
} JmWireJobInfo_t;

typedef struct _JmMsgReqCreate {
	JmWireJobOptions_t options;
	JmWireJobParams_t params;
	JmWireGuidList_t guids;
} JmMsgReqCreate_t;

typedef struct _JmMsgReqGenericQuery {
	uint64_t id;
} JmMsgReqGenericQuery_t;

typedef struct _JmMsgReqSetUseMatrix {
	uint64_t id;
	JmWireUseMatrix_t matrix;
} JmMsgReqSetUseMatrix_t;

typedef struct _JmMsgRespGetJobsContext {
	uint16_t count;
	uint8_t * buf;
	int bpos;
	int blen;
} JmMsgRespGetJobsContext_t;

//=============================================================================
// CORE JOB MANAGEMENT STRUCTURES
//

typedef struct _JmUseElement {
	// true if bursy, otherwise sustained
	uint16_t bursty :  1;
	// index of the source switch
	uint16_t swidx  : 15;
	// lid of the destination
	uint16_t dlid;
	// relative weight of path with respect to other paths
	uint8_t use;
} JmUseElement_t;

typedef struct _JmUseMatrix {
	// scales the specified use_elements (leaving unspecified
	// elements alone)
	uint16_t multiplier;
	// base value for non-specified elements
	uint8_t base;
	// number of elements in sparse use matrix input
	uint16_t count;
	// elements of the use matrix, one per switch/dlid pair
	JmUseElement_t * elements;
} JmUseMatrix_t;

typedef enum {
	JM_STATE_UNINITIALIZED = 0,
	JM_STATE_CANCELLED     = 1,
	JM_STATE_COMPLETE      = 2,
	JM_STATE_INITIALIZED   = 3,
	JM_STATE_ROUTED        = 4,
	JM_STATE_USE_AVAILABLE = 5
} JmState_t;

typedef enum {
	JM_POLL_NOT_READY = 0,
	JM_POLL_READY     = 1
} JmPollStatus_t;

typedef struct _JmPort {
	// guid as specified in input
	uint64_t guid;
	// port in old topology (if exists)
	Port_t *portp;
	// job-specific switch index
	uint16_t jobSwIdx;
} JmPort_t;

typedef struct _JmEntry {
	// unique identifier for the job
	uint64_t id;
	// timestamp of job creation in seconds since 1970
	uint64_t timestamp;
	// user-specified job metadata
	JmWireJobParams_t params;
	// state of the job
	JmState_t state;
	// topology passcount associated with job creation
	uint32_t passcount;
	// the number of end ports in the job
	uint16_t portCount;
	// the number of edge switches in the job
	uint16_t switchCount;
	// list of ports assigned to the job
	// [length = portCount]
	JmPort_t * ports;
	// the mapping from job-specific switch indices to
	// topology-specific switch indices
	// [length = switchCount]
	uint16_t * jobSwToTopoSwMap;
	// the use matrix (if present)
	JmWireUseMatrix_t useMatrix;
} JmEntry_t;

typedef struct _JmTable {
	// mutex protecting job table
	Lock_t lock;
	// set of jobs by JobID
	CS_HashTable_t * jobs;
} JmTable_t;

//=============================================================================
// FUNCTIONS DECLARATIONS
//

// sm_jm.c

typedef Status_t (*sm_jm_start_func_t)(uint32_t, void *);
typedef Status_t (*sm_jm_iter_func_t)(JmEntry_t *, void *);

Status_t sm_jm_init_job_table(void);
void sm_jm_destroy_job_table(void);
Status_t sm_jm_iterate_over_jobs(sm_jm_start_func_t, sm_jm_iter_func_t, void *);
Status_t sm_jm_insert_job(JmEntry_t *);
Status_t sm_jm_remove_job(JmEntry_t *);
Status_t sm_jm_lookup_job(uint64_t, JmEntry_t **);
Status_t sm_jm_alloc_job(JmEntry_t **);
Status_t sm_jm_free_job(JmEntry_t *);
Status_t sm_jm_fill_ports(Topology_t *, JmMsgReqCreate_t *, JmEntry_t *, uint16_t *);
Status_t sm_jm_get_cost(Topology_t *, JmEntry_t *, uint16_t **, int *);
void sm_jm_free_cost(uint16_t *);

// sm_jm_wire.c

void sm_jm_free_resp(uint8_t *);

Status_t sm_jm_decode_req_generic_query(uint8_t *, int, JmMsgReqGenericQuery_t *);

Status_t sm_jm_decode_req_create(uint8_t *, int, JmMsgReqCreate_t *);
void sm_jm_free_req_create(JmMsgReqCreate_t *);
Status_t sm_jm_encode_resp_create(JmEntry_t *, uint16_t *, int, uint8_t **, int *);

Status_t sm_jm_decode_req_set_use_matrix(uint8_t *, int, JmMsgReqSetUseMatrix_t *);

Status_t sm_jm_encode_resp_poll(JmEntry_t *, uint8_t **, int *);

Status_t sm_jm_encode_resp_get_guid_list(JmEntry_t *, uint8_t **, int *);
Status_t sm_jm_encode_resp_get_switch_map(JmEntry_t *, uint8_t **, int *);
Status_t sm_jm_encode_resp_get_cost_matrix(JmEntry_t *, uint16_t *, int, uint8_t **, int *);
Status_t sm_jm_encode_resp_get_use_matrix(JmEntry_t *, uint8_t **, int *);

Status_t sm_jm_encode_resp_get_jobs_start(uint32_t, void *);
Status_t sm_jm_encode_resp_get_jobs_iter(JmEntry_t *, void *);

#endif // _SM_MPI_H_

