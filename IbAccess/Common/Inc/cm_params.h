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
#ifndef _IBA_IB_CM_PARAMS_H_
#define _IBA_IB_CM_PARAMS_H_


#if defined (__cplusplus)
extern "C" {
#endif

#include "datatypes.h"

/* default values for CM tunable parameters */
#define CM_MAX_NDGRM					30000	/* This should be >= # of QP */
#define CM_MAX_CONNECTIONS				2048	/* CEP limit in CmWait */
#define CM_REQ_RETRY					8		/* REQ and SIDR_REQ */
#define CM_REP_RETRY					8		/* REP */
	/* controls over growth of Cm Dgrm Pool */
#define CM_DGRM_INITIAL					2048 /* initial size */
#define CM_DGRM_LOW_WATER				100 /* low water point */
#define CM_DGRM_INCREMENT				500	/* amount to grow by */

/* Default Max backlog for a passive endpoint before we start */
/* discarding incoming connection requests or mark the passive */
/* endpoint as dead (adjustable per CEP via CmModifyCEP) */
#define CM_MAX_BACKLOG					1024

/* Initial average turnaround time */
#define CM_TURNAROUND_TIME_MS			60		/* 60 ms */

/* Default Min turnaround time setting (bounds avg and CmModifyCEP setting) */
#define CM_MIN_TURNAROUND_TIME_MS		40		/* 40 ms */

/* Default Max turnaround time setting (bounds avg and CmModifyCEP setting) */
#define CM_MAX_TURNAROUND_TIME_MS		4000	/* 4 seconds */

/* Default SIDR req timeout */
#define SIDR_REQ_TIMEOUT_MS				8		/* 8 ms */

/* if 1, allow QP to be reused in TimeWait state */
#define CM_REUSE_QP						0

/* if 1, don't check PKey and GID validity */
#define CM_DISABLE_VALIDATION			0

typedef struct _CM_PARAMS {
	uint32		MaxDgrms;
	uint32		PreallocDgrms;
	uint32		MinFreeDgrms;
	uint32		DgrmIncrement;
	uint32		MaxConns;
	uint32		MaxReqRetry;
	uint32		MaxRepRetry;
	uint32		MaxBackLog;
	uint32		SidrReqTimeoutMs;	/* CPU processing part of timeout */
	uint32		TurnaroundTimeMs;
	uint32		MinTurnaroundTimeMs;
	uint32		MaxTurnaroundTimeMs;
	uint32		ReuseQP;
	uint32		DisableValidation;
} CM_PARAMS;

extern CM_PARAMS	gCmParams;

#if defined (__cplusplus)
};
#endif

#endif  /* _IBA_IB_CM_PARAMS_H_ */
