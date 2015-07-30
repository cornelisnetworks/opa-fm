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

/* [ICS VERSION STRING: unknown] */

/* Suppress duplicate loading of this file */
#ifndef _INSI_IB_SD_PARAMS_H_
#define _INSI_IB_SD_PARAMS_H_


#if defined (__cplusplus)
extern "C" {
#endif

#include "datatypes.h"

	/* default client control options */
#define SD_DEFAULT_RETRY_INTERVAL          10000				/* ms */
#define SD_DEFAULT_RETRY_COUNT             3
#if defined(DBG) ||defined( IB_DEBUG)
#define SD_DEFAULT_DEBUG_FLAGS				0xffffffff
#else
#define SD_DEFAULT_DEBUG_FLAGS				0
#endif
#define SD_DEFAULT_OPTION_FLAGS			0

	/* interval for checking queued queries and retries/timeouts in ms */
#define SD_DEFAULT_TIME_INTERVAL			500 /*1000 */

/* max child queries in main queue, helps to prevent child queries from
 * overrunning primary queries
 */
#define SD_DEFAULT_MAX_SUB_QUERY_QUEUED		10

#define SD_DEFAULT_MAX_SA_OUTSTANDING		1

#define SD_DEFAULT_SA_BUSY_BACKOFF          10000				/* ms */
#define SD_DEFAULT_VALIDATE_NODE_RECORD     1
#define SD_DEFAULT_DUMP_INVALID_NODE_RECORD     0
#define SD_DEFAULT_MIN_PKTLIFETIME          60					/* ms */

typedef struct _SD_CONFIG_PARAMS {
		/* parameters for queries where ClientControlParameters omitted */
	uint32		DefaultRetryCount;
	uint32		DefaultRetryInterval;
	uint32		DefaultDebugFlags;
	uint32		DefaultOptionFlags;

	uint32		TimeInterval;	/* retry/timeout check rate in ms */
	uint32		SaBusyBackoff;	/* backoff before retries when Busy reported */
	uint32		ValidateNodeRecord; /* Validation of GUIDs in Node Record */
	uint32		DumpInvalidNodeRecord; /* Full dump of any Invalid Node Records */
	uint32		MinPktLifeTime; /* Minimum PathRecord.PktLifeTime to report, in ms */
		/* maximum outstanding queries/operations */
	uint32		MaxChildQuery;
	uint32		MaxOutstanding;
} SD_CONFIG_PARAMS;

#define SD_DEFAULT_CONFIG_PARAMS {	\
	SD_DEFAULT_RETRY_INTERVAL, SD_DEFAULT_RETRY_COUNT, SD_DEFAULT_DEBUG_FLAGS, \
	SD_DEFAULT_OPTION_FLAGS, \
	SD_DEFAULT_TIME_INTERVAL, \
	SD_DEFAULT_SA_BUSY_BACKOFF, \
	SD_DEFAULT_VALIDATE_NODE_RECORD, \
	SD_DEFAULT_DUMP_INVALID_NODE_RECORD, \
	SD_DEFAULT_MIN_PKTLIFETIME, \
	SD_DEFAULT_MAX_SUB_QUERY_QUEUED, \
	SD_DEFAULT_MAX_SA_OUTSTANDING \
	}

extern	SD_CONFIG_PARAMS	g_SdParams;

#if defined (__cplusplus)
};
#endif

#endif  /* _INSI_IB_SD_PARAMS_H_ */
