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
#ifndef _INSI_IB_GSI_PARAMS_H_
#define _INSI_IB_GSI_PARAMS_H_


#if defined (__cplusplus)
extern "C" {
#endif

#include "datatypes.h"

/*
 * Configuration Parameters 
 *
 *	WorkQEntries is the maximum number of entries on each WQ
 *	The smaller of this and the maximum described in the channel
 *	adapter attributes is used in the create attributes when creating
 *	a special QP. 
 */

typedef struct _GSA_GLOBAL_CONFIG_PARAMETERS {
    uint32      SendQDepth;
    uint32      RecvQDepth;

    /*uint32      DefaultQP1PKeyIndex; */

    uint32      PreAllocRecvBuffersPerPort;		/* Number of Buffers to create*/
												/* in Pool for each port */
	uint32		MinBuffersToPostPerPort;		/* Number of buffers to post */
												/* per port */
	uint32		MaxRecvBuffers;					/* Max. buffers that can be  */
												/* allocated on a system */
	uint32		FullPacketDump;					/* make error dump more verbose */
			/* BUGBUG, some of these are not used */
} GSA_GLOBAL_CONFIG_PARAMETERS;


/* default values for GSI tunable parameters */
/* module parameters can override */
#define GSA_SENDQ_DEPTH 8192
#define GSA_RECVQ_DEPTH 8192
#ifdef VXWORKS
#define GSA_PREALLOC_RECV_PER_PORT 200
#define GSA_MIN_RECV_PER_PORT 128
#define GSA_MAX_RECV_BUFFERS 512
#else
#define GSA_PREALLOC_RECV_PER_PORT 500
#define GSA_MIN_RECV_PER_PORT 60
#define GSA_MAX_RECV_BUFFERS 8000
#endif

#define GSA_DEFAULT_SETTINGS {	\
	GSA_SENDQ_DEPTH, GSA_RECVQ_DEPTH,		\
	/*0,*/									\
	GSA_PREALLOC_RECV_PER_PORT,				\
	GSA_MIN_RECV_PER_PORT,					\
	GSA_MAX_RECV_BUFFERS					\
	}

extern	GSA_GLOBAL_CONFIG_PARAMETERS	g_GsaSettings;

#if defined (__cplusplus)
};
#endif

#endif  /* _INSI_IB_GSI_PARAMS_H_ */
