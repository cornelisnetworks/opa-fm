/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

#ifndef CS_CSM_LOG_L_H
#define CS_CSM_LOG_L_H

/* enumerations */
typedef enum {
	CSM_SEV_INFO = 1,
	CSM_SEV_NOTICE = 2,
	CSM_SEV_WARNING = 3,
	CSM_SEV_ERROR = 4,
	CSM_SEV_MAX
} SmCsmMsgType_t;

typedef enum {
	CSM_COND_REDUNDANCY_LOST = 1,
	CSM_COND_REDUNDANCY_RESTORED = 2,
	CSM_COND_APPEARANCE = 3,
	CSM_COND_DISAPPEARANCE = 4,
	CSM_COND_SM_STATE_TO_MASTER = 5,
	CSM_COND_SM_STATE_TO_STANDBY = 6,
	CSM_COND_SM_SHUTDOWN = 7,
	CSM_COND_FABRIC_INIT_ERROR = 8,
	CSM_COND_LINK_ERROR = 9,
	CSM_COND_SECURITY_ERROR = 10,
	CSM_COND_OTHER_ERROR = 11,
	CSM_COND_FABRIC_SUMMARY = 12,
	CSM_COND_SM_STATE_TO_NOTACTIVE = 13,
	CSM_COND_STANDBY_SM_DEACTIVATION = 14,
	CSM_COND_STANDBY_SM_VF_DEACTIVATION = 15,
	CSM_COND_SECONDARY_BM_DEACTIVATION = 16,
	CSM_COND_SECONDARY_PM_DEACTIVATION = 17,
	CSM_COND_DEACTIVATION_OF_STANDBY = 18,
	CSM_COND_SECONDARY_EM_DEACTIVATION = 19,
	CSM_COND_MAX
} SmCsmMsgCondition_t;

/* structure definitions */
typedef struct {
	char description[64]; /* 64 == Maximum length of node description in IBTA v1.2 */
	uint8_t port;
	uint64_t guid;
} SmCsmNodeId_t;

void smCsmFormatNodeId(SmCsmNodeId_t * id, uint8_t * desc, int port, uint64_t guid);

/* prototypes */
int smCsmSetLogSmDesc(const char * nodeDesc, int port, uint64_t guid);
SmCsmNodeId_t * getMyCsmNodeId(void);

int smCsmLogMessage( SmCsmMsgType_t msgType, SmCsmMsgCondition_t cond,
                     SmCsmNodeId_t * node, SmCsmNodeId_t * linkedTo,
                     char * detailFmt, ... );


#endif
