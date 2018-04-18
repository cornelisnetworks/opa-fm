/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef IBPRINT_H
#define IBPRINT_H

#include <stdio.h>
#include <iba/public/idebug_osd.h>
#include <iba/ibt.h>
#include <iba/stl_sm_priv.h>
#include <iba/stl_pm.h>

typedef enum {
	PL_NONE = 0,	// no output
	PL_FILE,		// output to a FILE
	PL_BUFFER,		// output to a buffer
	PL_CALLBACK,	// output via callback
#ifdef LINUX
	PL_SYSLOG,		// output to syslog
#endif
#ifdef VXWORKS
	PL_LOG,			// output to embedded log
	// TBD add VxWorks sysprint or simply print?
#endif
} PrintType_t;

typedef struct PrintBuf_s {
	uint16 length;	// size of buffer
	uint16 offset;	// next place to write in buffer
	char *buffer;
} PrintBuf_t;

typedef void (*PrintCallbackFunc_t)(void *context, const char *buf);

typedef struct PrintCallback_s {
	PrintCallbackFunc_t func;
	void *context;
} PrintCallback_t;

typedef struct PrintDest_s {
	PrintType_t type;
	union {
		FILE *file;
		PrintBuf_t buf;
		PrintCallback_t callback;
#ifdef LINUX
		int priority;	// for syslog
#endif
#ifdef VXWORKS
		// TBD
#endif
	} u;
} PrintDest_t;


extern void PrintDestInitNone(PrintDest_t *dest);	// no output
extern void PrintDestInitFile(PrintDest_t *dest, FILE *file);
extern void PrintDestInitBuffer(PrintDest_t *dest, char *buffer, uint16 length);
extern void PrintDestInitCallback(PrintDest_t *dest, PrintCallbackFunc_t func,
				void *context);
#ifdef LINUX
extern void PrintDestInitSyslog(PrintDest_t *dest, int priority);
#endif

#ifdef VXWORKS
extern void PrintDestInitLog(PrintDest_t *dest);
#endif
extern void PrintFunc(PrintDest_t *dest, const char *format, ...) __attribute__((format(printf,2,3)));

///////////////////////////////////////////////////////////////////////////////
// some basic IB types
extern void PrintGuid(PrintDest_t *dest, int indent, EUI64 Guid);
extern void PrintLid(PrintDest_t *dest, int indent, IB_LID Lid);
extern void PrintGid(PrintDest_t *dest, int indent, const IB_GID *pGid);
extern void PrintLongMaskBits(PrintDest_t *dest, int indent, const char* prefix, const uint8 *bits, unsigned size);
extern void PrintSeparator(PrintDest_t *dest);

///////////////////////////////////////////////////////////////////////////////
// common MAD headers and MADs
//extern const char* IbMgmtClassToText(uint8 class);
//extern const char* IbMadMethodToText(uint8 method);
//extern const char* IbMadStatusToText(uint16 status);
extern void PrintMad(PrintDest_t *dest, int indent, const MAD *pMad);
extern void PrintMadHeader(PrintDest_t *dest, int indent, const MAD_COMMON *hdr);
extern void PrintSmpHeader(PrintDest_t *dest, int indent, const SMP *smp);
typedef void (*formatcapmask_func_t)(char buf[80], uint16 cmask);
typedef void (*formatcapmask2_func_t)(char buf[80], uint32 cmask);
extern void FormatClassPortInfoCapMask(char buf[80], uint16 cmask);
extern void FormatClassPortInfoCapMask2(char buf[80], uint32 cmask);
extern void PrintClassPortInfo2(PrintDest_t *dest, int indent,
			const IB_CLASS_PORT_INFO *pClassPortInfo, formatcapmask_func_t func,
			formatcapmask2_func_t func2);
extern void PrintClassPortInfo(PrintDest_t *dest, int indent,
			const IB_CLASS_PORT_INFO *pClassPortInfo);
// TBD extern void PrintNotice(PrintDest_t *dest, int indent, const NOTICE *pNotice);
// TBD also trap 64-67 details

///////////////////////////////////////////////////////////////////////////////
// SMA
extern void PrintSmp(PrintDest_t *dest, int indent, const SMP *smp);
extern void PrintNodeDesc(PrintDest_t *dest, int indent,
				const NODE_DESCRIPTION *pNodeDesc);
extern void PrintNodeInfo(PrintDest_t *dest, int indent,
				const NODE_INFO *pNodeInfo);
// style=0 or 1
// TBD - key off portGuid!=0 to decide style instead?
extern void PrintPortInfo(PrintDest_t *dest, int indent,
				const PORT_INFO *pPortInfo, EUI64 portGuid, int style);
extern void PrintPortInfoSmp(PrintDest_t *dest, int indent,
				const SMP *smp, EUI64 portGuid);
extern void PrintSMInfo(PrintDest_t *dest, int indent,
				const SM_INFO *pSMInfo, IB_LID lid);
extern void PrintSwitchInfo(PrintDest_t *dest, int indent,
				const SWITCH_INFO *pSwitchInfo, IB_LID lid);
extern void PrintLinearFDB(PrintDest_t *dest, int indent,
				const FORWARDING_TABLE *pLinearFDB, uint16 blockNum);
extern void PrintRandomFDB(PrintDest_t *dest, int indent,
				const FORWARDING_TABLE *pRandomFDB, uint16 blockNum);
extern boolean isMCastFDBEmpty(const FORWARDING_TABLE *pMCastFDB);
extern void PrintMCastFDB(PrintDest_t *dest, int indent,
				const FORWARDING_TABLE *pMCastFDB, uint16 blockNum);
extern void PrintMCastFDBSmp(PrintDest_t *dest, int indent,
				const SMP *smp);
extern boolean isMCastFDBEmptyRows(const FORWARDING_TABLE *pMCastFDB,
				uint8 numPositions);
extern void PrintPortGroupTable(PrintDest_t *dest, int indent, 
				const PORT_GROUP_TABLE *pPortGroup, uint16 tier, uint16 blockNum);
extern void PrintAdaptiveRoutingLidmask(PrintDest_t *dest, int indent, 
				const ADAPTIVE_ROUTING_LIDMASK *pLidmask, uint16 blockNum);


// prints full rows in table
// pMcastFDB expected to be FORWARDING_TABLE[1+endPosition-startPosition] entry
extern void PrintMCastFDBRows(PrintDest_t *dest, int indent,
				const FORWARDING_TABLE *pMCastFDB, uint16 blockNum,
				uint8 startPosition, uint8 endPosition, boolean showHeading);
extern void PrintVLArbTable(PrintDest_t *dest, int indent,
				const VLARBTABLE *pVLArbTable, uint8 blockNum);
extern void PrintVLArbTableSmp(PrintDest_t *dest, int indent,
				const SMP *smp, NODE_TYPE nodetype);
extern void PrintPKeyTable(PrintDest_t *dest, int indent,
				const PARTITION_TABLE *pPKeyTable, uint16 blockNum);
extern void PrintPKeyTableSmp(PrintDest_t *dest, int indent,
				const SMP *smp, NODE_TYPE nodetype,
				boolean showHeading, boolean showBlock);
extern void PrintGuidInfo(PrintDest_t *dest, int indent,
				const GUID_INFO *pGuidInfo, uint8 blockNum);

///////////////////////////////////////////////////////////////////////////////
// SA one print function for each SA query OutputType
extern void PrintPathRecord(PrintDest_t *dest, int indent,
				const IB_PATH_RECORD *pPathRecord);
extern void PrintNodeRecord(PrintDest_t *dest, int indent,
				const IB_NODE_RECORD *pNodeRecord);

extern void PrintPortInfoRecord(PrintDest_t *dest, int indent,
				const IB_PORTINFO_RECORD *pPortInfoRecord);
extern void PrintSMInfoRecord(PrintDest_t *dest, int indent,
				const IB_SMINFO_RECORD *pSMInfoRecord);
extern void PrintLinkRecord(PrintDest_t *dest, int indent,
				const IB_LINK_RECORD *pLinkRecord);
extern void PrintTraceRecord(PrintDest_t *dest, int indent,
				const IB_TRACE_RECORD *pTraceRecord);
extern void PrintSwitchInfoRecord(PrintDest_t *dest, int indent,
				const IB_SWITCHINFO_RECORD *pSwitchInfoRecord);
extern void PrintLinearFDBRecord(PrintDest_t *dest, int indent,
				const IB_LINEAR_FDB_RECORD *pLinearFDBRecord);
extern void PrintRandomFDBRecord(PrintDest_t *dest, int indent,
				const IB_RANDOM_FDB_RECORD *pRandomFDBRecord);
extern void PrintMCastFDBRecord(PrintDest_t *dest, int indent,
				const IB_MCAST_FDB_RECORD *pMCastFDBRecord);
extern void PrintVLArbTableRecord(PrintDest_t *dest, int indent,
				const IB_VLARBTABLE_RECORD *pVLArbTableRecord);
extern void PrintPKeyTableRecord(PrintDest_t *dest, int indent,
				const IB_P_KEY_TABLE_RECORD *pPKeyTableRecord);
//extern char HexToChar(uint8 c);
//extern void FormatChars(char *buf, const uint8* data, uint32 len);
extern void PrintServiceRecord(PrintDest_t *dest, int indent,
				const IB_SERVICE_RECORD *pServiceRecord);
extern void PrintMcMemberRecord(PrintDest_t *dest, int indent,
				const IB_MCMEMBER_RECORD *pMcMemberRecord);
extern void PrintInformInfo(PrintDest_t *dest, int indent,
				const IB_INFORM_INFO *pInformInfo);
extern void PrintInformInfoRecord(PrintDest_t *dest, int indent,
				const IB_INFORM_INFO_RECORD *pInformInfoRecord);

///////////////////////////////////////////////////////////////////////////////
// IbAccess SubnetDriver interface

// display the results from a query
// Note: csv is ignored for all but OutputTypeVfInfoRecord
extern void PrintQueryResultValue(PrintDest_t *dest, int indent,
				PrintDest_t *dbgDest,
				QUERY_RESULT_TYPE OutputType, int csv,
				const QUERY_RESULT_VALUES *pResult);
// display the results from a query
// returns potential exit status:
// 	0 - success
// 	1 - query failed
// 	2 - query failed due to invalid query
// Note: csv is ignored for all but OutputTypeVfInfoRecord
extern int PrintQueryResult(PrintDest_t *dest, int indent,
				PrintDest_t *dbgDest,
				QUERY_RESULT_TYPE OutputType, int csv, FSTATUS status,
				QUERY_RESULT_VALUES *pResult);

///////////////////////////////////////////////////////////////////////////////
// Print with additional information.
extern void PrintExtendedPathRecord(PrintDest_t *dest, int indent,
				const IB_PATH_RECORD *pPathRecord);
#endif /* IBPRINT_H */
