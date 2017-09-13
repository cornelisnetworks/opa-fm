/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "ib_types.h"
#include "cs_log.h"
#include "cs_csm_log.h"
#include "cs_g.h"

#ifdef __VXWORKS__

#include "ESM_Messages.h"

#else

#include <syslog.h>
#include <time.h>

#define FORMAT1 "%s; MSG:%s|SM:%s|COND:%s|NODE:%s:port %u:"FMT_U64"|LINKEDTO:%s:port %u:"FMT_U64"|DETAIL:%s"
#define FORMAT2 "%s; MSG:%s|SM:%s|COND:%s|NODE:%s:port %u:"FMT_U64"|DETAIL:%s"
#define FORMAT3 "%s; MSG:%s|SM:%s|COND:%s|DETAIL:%s"

static const char * severityStrings[CSM_SEV_MAX] = {
	"", // 0th string reserved
	"INFORMATION",
	"NOTICE",
	"WARNING",
	"ERROR",
};
#endif

// Note that MSG_BUFFER_SZ is huge... We probably want to rethink this at some point
#define MSG_BUFFER_SZ 1024
#define MAX_SM_DESCRIPTION_SZ 256
#define DEFAULT_SM_DESCRIPTION "Default SM"
#define MSG_FIELD_SEPARATOR '|'
#define MSG_NODE_SEPARATOR ':'

static const char translationFromChars[] = { MSG_FIELD_SEPARATOR, MSG_NODE_SEPARATOR };
static const char translationToChars[] = { ' ', ' ' };
static const char DefaultDesc[] = DEFAULT_SM_DESCRIPTION;
static char smDescription [MAX_SM_DESCRIPTION_SZ] = DEFAULT_SM_DESCRIPTION ":port 1";
static SmCsmNodeId_t myNodeId;

static const char * conditionStrings[CSM_COND_MAX] = {
	"", // 0th string reserved
	"#1 Redundancy lost",
	"#2 Redundancy restored",
	"#3 Appearance in fabric",
	"#4 Disappearance from fabric",
	"#5 SM state to master",
	"#6 SM state to standby",
	"#7 SM shutdown",
	"#8 Fabric initialization error",
	"#9 Link integrity/symbol error",
	"#10 Security error",
	"#11 Other Exception",
	"#12 Fabric Summary",
	"#13 SM state to inactive",
	"#14 SM standby configuration inconsistency",
	"#15 SM standby virtual fabric configuration inconsistency",
	"#16 BM secondary configuration inconsistency",
	"#17 PM secondary configuration inconsistency",
	"#18 Master SM deactivating standby that is not responding",

};


static void convertChars(char * str, int numVals, const char fromValues[], const char toValues[])
{
	int i = 0, j = 0;

	if (str == NULL)
		return;

	for (i = 0; i < strlen(str); ++i)
	{
		for (j = 0; j < numVals; ++j)
		{
			if (str[i] == fromValues[j])
				str[i] = toValues[j];
		}
	}
}

/*
 * Returns the SM Node description that was set by smCsmSetLogSmDesc for
 * use by the actual logging functions
 */
SmCsmNodeId_t * getMyCsmNodeId(void)
{
	return &myNodeId;
}

/*
 * sets the SM description for CSM logging
 */
int smCsmSetLogSmDesc(const char * nodeDesc, int port, uint64_t guid)
{
	int rc = 0;

	if (nodeDesc == NULL)
		nodeDesc = DefaultDesc;

	strncpy(myNodeId.description, nodeDesc, 64);
	myNodeId.description[63] = '\0';

	convertChars(myNodeId.description, sizeof(translationFromChars),
	             translationFromChars, translationToChars);

	myNodeId.port = port;
	myNodeId.guid = guid;
#ifdef __VXWORKS__
	sprintf(smDescription, "%s:port %d", myNodeId.description, port);
#else
	snprintf(smDescription, MAX_SM_DESCRIPTION_SZ, "%s:port %d", myNodeId.description, port);
#endif

	return rc;
}

void smCsmFormatNodeId(SmCsmNodeId_t * id, uint8_t * desc, int port, uint64_t guid)
{
	strncpy(id->description, (char *)desc, 64);
	id->description[63] = '\0';

	convertChars(id->description, sizeof(translationFromChars),
	             translationFromChars, translationToChars);

	id->port = port;
	id->guid = guid;
}


#ifdef __VXWORKS__
static int smCsmLogHelper(SmCsmMsgType_t msgType, const char * smDesc, const char * condition,
                          const char * nodeDesc, const char * linkedToDesc, const char * details)
{
	switch (msgType)
	{
	 case CSM_SEV_INFO:
		if (linkedToDesc != NULL)
		{
			ESM_LOG_CSM_ESM_INFO_TWO(smDesc, condition, Log_StrDup(nodeDesc),
			                         Log_StrDup(linkedToDesc),
			                         Log_StrDup(details));
		} else if(nodeDesc != NULL)
		{
			ESM_LOG_CSM_ESM_INFO_ONE(smDesc, condition, Log_StrDup(nodeDesc),
			                         Log_StrDup(details));
			                         
		} else
		{
			ESM_LOG_CSM_ESM_INFO_ZERO(smDesc, condition, 
			                         Log_StrDup(details));
		}
		break;

	 case CSM_SEV_NOTICE:
		if (linkedToDesc != NULL)
		{
			ESM_LOG_FINAL_CSM_ESM_NOTICE_TWO(smDesc, condition, Log_StrDup(nodeDesc),
			                           Log_StrDup(linkedToDesc), 
			                         Log_StrDup(details));
		} else if(nodeDesc != NULL)
		{
			ESM_LOG_FINAL_CSM_ESM_NOTICE_ONE(smDesc, condition, Log_StrDup(nodeDesc),
			                         Log_StrDup(details));
			                         
		} else
		{
			ESM_LOG_FINAL_CSM_ESM_NOTICE_ZERO(smDesc, condition, 
			                         Log_StrDup(details));
		}
		break;

	 case CSM_SEV_WARNING:
		if (linkedToDesc != NULL)
		{
			ESM_LOG_FINAL_CSM_ESM_WARNING_TWO(smDesc, condition, Log_StrDup(nodeDesc),
			                            Log_StrDup(linkedToDesc), 
			                         Log_StrDup(details));
		} else if(nodeDesc != NULL)
		{
			ESM_LOG_FINAL_CSM_ESM_WARNING_ONE(smDesc, condition, Log_StrDup(nodeDesc),
			                         Log_StrDup(details));
			                         
		} else
		{
			ESM_LOG_FINAL_CSM_ESM_WARNING_ZERO(smDesc, condition, 
			                         Log_StrDup(details));
		}
		break;

	 case CSM_SEV_ERROR: /* fall through to default case */
	 default:
		if (linkedToDesc != NULL)
		{
			ESM_LOG_FINAL_CSM_ESM_ERROR_TWO(smDesc, condition, Log_StrDup(nodeDesc),
			                          Log_StrDup(linkedToDesc), 
			                         Log_StrDup(details));
		} else if(nodeDesc != NULL)
		{
			ESM_LOG_FINAL_CSM_ESM_ERROR_ONE(smDesc, condition, Log_StrDup(nodeDesc),
			                         Log_StrDup(details));
			                         
		} else
		{
			ESM_LOG_FINAL_CSM_ESM_ERROR_ZERO(smDesc, condition, 
			                         Log_StrDup(details));
		}
		break;
	}
	return 0;
}
#endif

/*
 * Logs a csm message
 */
int smCsmLogMessage( SmCsmMsgType_t msgType, SmCsmMsgCondition_t cond,
                     SmCsmNodeId_t * node, SmCsmNodeId_t * linkedTo,
                     char * detailFmt, ... )
{
	int rc = 0;
	va_list ap;

#ifdef __VXWORKS__
	char detailBuffer[MSG_BUFFER_SZ];
	if (detailFmt != NULL)
	{
		cs_strlcpy(detailBuffer, "|DETAIL:", sizeof(detailBuffer));
		va_start(ap, detailFmt);
		vsprintf(detailBuffer + strlen(detailBuffer), detailFmt, ap);
		va_end(ap);
	} else
	{
		detailBuffer[0] = '\0';
	}

	if (node == NULL)
	{
		smCsmLogHelper(msgType, smDescription, conditionStrings[cond],
		               NULL, NULL, detailBuffer);
	} else
	{
		char nodeDescBuffer[100];
		sprintf(nodeDescBuffer, "%s:port %u:"FMT_U64,
		        node->description, node->port, node->guid);

		if (linkedTo == NULL)
		{
			smCsmLogHelper(msgType, smDescription, conditionStrings[cond], nodeDescBuffer,
			               NULL, detailBuffer);
		} else
		{
			char linkedToDescBuffer[100];
			sprintf(linkedToDescBuffer, "%s:port %u:"FMT_U64,
			        linkedTo->description, linkedTo->port, linkedTo->guid);

			smCsmLogHelper(msgType, smDescription, conditionStrings[cond], nodeDescBuffer,
			               linkedToDescBuffer, detailBuffer);
		}
	}

#else
	char buffer[MSG_BUFFER_SZ];
	int priority = LOG_ERR;
	int flush = 0;
	extern FILE *log_file;	// from vslog.c
	extern int output_fd;	// from vslog.c
	extern int logMode;	// from vslog.c
	FILE *f = NULL;
	extern uint32_t log_to_console; 

	switch (msgType) {
 	 case CSM_SEV_INFO:
		priority = LOG_INFO;
		if((VS_LOG_CSM_INFO & cs_log_masks[VIEO_APP_MOD_ID]) == 0)
			 return 0;
		flush = 0;
		break;

	 case CSM_SEV_NOTICE:
		priority = LOG_NOTICE;
		if((VS_LOG_CSM_NOTICE & cs_log_masks[VIEO_APP_MOD_ID]) == 0)
			 return 0;
		flush = 1;
		break;

	 case CSM_SEV_WARNING:
		priority = LOG_WARNING;
		if((VS_LOG_CSM_WARN & cs_log_masks[VIEO_APP_MOD_ID]) == 0)
			 return 0;
		flush = 1;
		break;

	 case CSM_SEV_ERROR:
	 default:
		priority = LOG_ERR;
		if((VS_LOG_CSM_ERROR & cs_log_masks[VIEO_APP_MOD_ID]) == 0)
			 return 0;
		flush = 1;
		break;
	}

	if (node != NULL)
	{
		if (linkedTo != NULL)
		{
			snprintf(buffer, MSG_BUFFER_SZ, FORMAT1, myNodeId.description,
			         severityStrings[msgType], smDescription, conditionStrings[cond], 
			         node->description, node->port, node->guid,
			         linkedTo->description, linkedTo->port, linkedTo->guid,
			         detailFmt);
		} else
		{
			snprintf(buffer, MSG_BUFFER_SZ, FORMAT2, myNodeId.description,
			         severityStrings[msgType], smDescription, conditionStrings[cond], 
			         node->description, node->port, node->guid,
			         detailFmt);
		}
	} else
	{
		snprintf(buffer, MSG_BUFFER_SZ, FORMAT3, myNodeId.description,
		         severityStrings[msgType], smDescription, conditionStrings[cond], 
		         detailFmt);
	}
	if(log_to_console){
		f = stdout;
	} else if(output_fd != -1) {
		f = log_file;
	}
	if(f)
	{
    	time_t    theCalTime=0;
    	struct tm *locTime;
    	char      strTime[28];

    	time(&theCalTime);
    	locTime = localtime(&theCalTime);
		if (locTime) {
			size_t l = strftime(strTime,
				sizeof(strTime),
				"%a %b %d %H:%M:%S %Y", 
				locTime);
			if (l==0) {
				strncpy(strTime,"(unknown)", sizeof(strTime));
			}
		} else {
			strncpy(strTime,"(unknown)", sizeof(strTime));
		}
		// TBD - later output in one print so can't intermingle with others
		fprintf(f, "%s: ", strTime);
		va_start(ap, detailFmt);
		vfprintf(f, buffer, ap);
		va_end(ap);
		fputc('\n', f);
	    if (flush)
            fflush(f);
    }
	if(output_fd == -1 || (logMode & 0x2) == 0)
	{
		va_start(ap, detailFmt);
		vsyslog(priority, buffer, ap);
		va_end(ap);
	}

#endif

	return rc;
}
