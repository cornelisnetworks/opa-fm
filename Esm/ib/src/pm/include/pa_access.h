/* BEGIN_ICS_COPYRIGHT3 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _PAACCESS_H
#define _PAACCESS_H

#include <iba/ibt.h>

#include <iba/ipublic.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <ctype.h>
#define _GNU_SOURCE

#include <iba/stl_pm.h>
#include <iba/stl_pa_priv.h>
#include "pm_topology.h"

#ifdef __cplusplus
extern "C" {
#endif

// DEFINES

#define	BAD_IMAGE_ID	(uint64)-1

#define PA_INC_COUNTER_NO_OVERFLOW(cntr, max) do { if (cntr >= max) { cntr = max; } else { cntr++; } } while(0)

// DATA TYPES

typedef struct _pmNameListEntry_s {
	char			Name[STL_PM_GROUPNAMELEN];	// \0 terminated
} PmNameListEntry_t;

typedef struct _pmGroupList_s {
	uint32				NumGroups;
	PmNameListEntry_t   *GroupList;
} PmGroupList_t;

typedef struct _pmGroupInfo_s {
	char			groupName[STL_PM_GROUPNAMELEN];	// \0 terminated
	uint32			NumIntPorts;
	uint32			NumExtPorts;
	PmUtilStats_t	IntUtil;	// when both ports in group
	PmUtilStats_t	SendUtil;	// send from group to outside
	PmUtilStats_t	RecvUtil;	// recv by group from outside
	// for Internal (in-group) we count one each port (both are in group)
	// for External (between-group), we count worst of our port and its neighbor
	PmErrStats_t	IntErr;// in group
	PmErrStats_t	ExtErr;// between groups
	uint8			MinIntRate;
	uint8			MaxIntRate;
	uint8			MinExtRate;
	uint8			MaxExtRate;
} PmGroupInfo_t;

typedef struct _pmPortConfig_s {
	STL_LID 		lid;
	uint8			portNum;
	uint8			reserved2[3];
	uint64			guid;
	char			nodeDesc[64];	// can be 64 char w/o \0
} PmPortConfig_t;

typedef struct _pmGroupConfig_s {
	char			groupName[STL_PM_GROUPNAMELEN];	// \0 terminated
	uint32			NumPorts;
	uint32			portListSize;
	PmPortConfig_t	*portList;
} PmGroupConfig_t;

typedef struct _pmFocusPortEntry_s {
	STL_LID 		lid;
	uint8			portNum;
	uint8   		rate;			// IB_STATIC_RATE
	uint8   		maxVlMtu;		// enum STL MTU
	uint8			neighborPortNum;
	STL_LID 		neighborLid;
	uint8			localStatus:4;
	uint8			neighborStatus:4;
	uint8           reserved[3];
	uint64			value[MAX_NUM_FOCUS_PORT_TUPLES];
	uint64			guid;
	char			nodeDesc[64];	// can be 64 char w/o \0
	uint64			neighborValue[MAX_NUM_FOCUS_PORT_TUPLES];
	uint64			neighborGuid;
	char			neighborNodeDesc[64];	// can be 64 char w/o \0
} PmFocusPortEntry_t;

typedef struct _pmFocusPorts_s {
	char			name[STL_PM_GROUPNAMELEN];	// \0 terminated
	uint32			NumPorts;
	uint32			portCntr;
	PmFocusPortEntry_t	*portList;
} PmFocusPorts_t;

typedef struct _focusArrayItem_s {
	uint64 localValue;
	uint64 neighborValue;
	STL_LID localLID;
	STL_LID neighborLID;
	PmPort_t *localPort;
	PmPort_t *neighborPort;
} focusArrayItem_t;

typedef struct smInfo_s {
	STL_LID	smLid;		// implies port, 0 if empty record
	uint8	priority:4;		// present priority
	uint8	state:4;		// present state
	uint8	portNumber;
	uint8	reserved2;
	uint64	smPortGuid;
	char	smNodeDesc[64];	// can be 64 char w/o \0
} PmSmInfo_t;

typedef struct _pmImageInfo_s {
	uint64						sweepStart;
	uint32						sweepDuration;
	uint16						numHFIPorts;
//	TFI not included in Gen1
//	uint16						numTFIPorts;
	uint16						numSwitchNodes;
	uint32						numSwitchPorts;
	uint32						numLinks;
	uint32						numSMs;
	uint32						numNoRespNodes;
	uint32						numNoRespPorts;
	uint32						numSkippedNodes;
	uint32						numSkippedPorts;
	uint32						numUnexpectedClearPorts;
	uint32						imageInterval;
	PmSmInfo_t					SMInfo[2];
} PmImageInfo_t;

typedef struct _pmVFList_s {
	uint32			   NumVFs;
	PmNameListEntry_t  *VfList;	// \0 terminated
} PmVFList_t;

typedef struct _pmVFConfig_s {
	char			vfName[STL_PM_VFNAMELEN];	// \0 terminated
	uint32			NumPorts;
	uint32			portListSize;
	PmPortConfig_t	*portList;
} PmVFConfig_t;

typedef struct _pmVFInfo_s {
	char			vfName[STL_PM_VFNAMELEN];	// \0 terminated
	uint32			NumPorts;
	PmUtilStats_t	IntUtil;	// when both ports in group
	PmErrStats_t	IntErr;
	uint8			MinIntRate;
	uint8			MaxIntRate;
} PmVFInfo_t;

typedef struct _pmNodeInfo_s {
	STL_LID 		nodeLid;
	uint8			nodeType;
	uint8			reserved2[3];
	uint64			nodeGuid;
	uint64		portSelectMask[4];
	char			nodeDesc[64];	// can be 64 char w/o \0
} PmNodeInfo_t;


typedef struct _pmGroupNodeInfo_s {
	char			groupName[STL_PM_GROUPNAMELEN];	// \0 terminated
	uint32			NumNodes;
	PmNodeInfo_t		*nodeList;
} PmGroupNodeInfo_t;

typedef struct _pmLinkInfo_s {
	STL_LID 		fromLid;
	STL_LID 		toLid;
	uint8			fromPort;
	uint8			toPort;
	uint8			mtu:4;
	uint8			activeSpeed:4;
	uint8			txLinkWidthDowngradeActive:4;
	uint8			rxLinkWidthDowngradeActive:4;
	uint8			localStatus:4;
	uint8			neighborStatus:4;
	uint8			reserved2[3];
} PmLinkInfo_t;


typedef struct _pmGroupLinkInfo_s {
	char			groupName[STL_PM_GROUPNAMELEN];	// \0 terminated
	uint32			NumLinks;
	PmLinkInfo_t		*linkInfoList;
} PmGroupLinkInfo_t;


// FUNCTION PROTOTYPES

/* BEGIN pa_utility.c */
const char *FSTATUS_ToString(FSTATUS status);

FSTATUS FindImage(Pm_t *pm, uint8 type, STL_PA_IMAGE_ID_DATA imageId, uint32 *imageIndex,
	uint64 *retImageId, PmHistoryRecord_t **record, const char **msg, uint8 *clientId,
	PmCompositeImage_t **cimg);
FSTATUS FindPmImage(const char *func, Pm_t *pm, STL_PA_IMAGE_ID_DATA req_img,
	STL_PA_IMAGE_ID_DATA *rsp_img, PmImage_t **pm_image, uint32 *imageIndex,
	boolean *requiresLock);

int getCachedCimgIdx(Pm_t *pm, PmCompositeImage_t *cimg);
uint64 BuildFreezeFrameImageId(Pm_t *pm, uint32 freezeIndex, uint8 clientId, uint32 *imageTime);
FSTATUS LocateGroup(PmImage_t *pmimagep, const char *groupName, int *groupIndex);
FSTATUS LocateVF(PmImage_t *pmimagep, const char *vfName, int *vfIdx);
/* END pa_utility.c */

// Note - API expects caller to define a Pm_t structure, which apps will do for now
//        Later, PM will handle this, and the parameter can be removed

// get group list - caller declares Pm_T and PmGroupList_t, and passes pointers
FSTATUS paGetGroupList(Pm_t *pm, PmGroupList_t *GroupList, STL_PA_IMAGE_ID_DATA imageId,
	STL_PA_IMAGE_ID_DATA *returnImageId);

// get group info - caller declares Pm_T and PmGroupInfo_t, and passes pointers
FSTATUS paGetGroupInfo(Pm_t *pm, char *groupName, PmGroupInfo_t *pmGroupInfo,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId);

// get group config - caller declares Pm_T and PmGroupConfig_t, and passes pointers
FSTATUS paGetGroupConfig(Pm_t *pm, char *groupName, PmGroupConfig_t *pmGroupConfig,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId);

// get group node info - caller declares Pm_T and PmGroupConfig_t, and passes pointers
FSTATUS paGetGroupNodeInfo(Pm_t *pm, char *groupName, uint64 nodeGUID, STL_LID nodeLID,
	char *nodeDesc, PmGroupNodeInfo_t *pmGroupNodeInfo, STL_PA_IMAGE_ID_DATA imageId,
	STL_PA_IMAGE_ID_DATA *returnImageId);

// get group link info - caller declares Pm_T and PmGroupConfig_t, and passes pointers
FSTATUS paGetGroupLinkInfo(Pm_t *pm, char *groupName, STL_LID inputLID, uint8 inputPort,
	PmGroupLinkInfo_t *pmGroupLinkInfo, STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId);

// get port stats - caller declares Pm_T and PmCompositePortCounters_t
//                  delta - 1 requests delta counters, 0 gets raw total
//                  userCntrs - 1 requests PA user controled counters, 0 gets Pm Controlled Image Counters. if 1 delta and ofset must be 0
FSTATUS paGetPortStats(Pm_t *pm, STL_LID lid, uint8 portNum, PmCompositePortCounters_t *portCountersP,
	uint32 delta, uint32 userCntrs, STL_PA_IMAGE_ID_DATA imageId, uint32 *flagsp,
	STL_PA_IMAGE_ID_DATA *returnImageId);

// Clear Running totals for a port.  This simulates a PMA clear so
// that tools like opareport can work against the Running totals
FSTATUS paClearPortStats(Pm_t *pm, STL_LID lid, uint8 portNum,
	CounterSelectMask_t select);

// Clear Running totals for all Ports.  This simulates a PMA clear so
// that tools like opareport can work against the Running totals
FSTATUS paClearAllPortStats(Pm_t *pm, CounterSelectMask_t select);

FSTATUS paFreezeFrameRenew(Pm_t *pm, STL_PA_IMAGE_ID_DATA *imageId);

FSTATUS paFreezeFrameRelease(Pm_t *pm, STL_PA_IMAGE_ID_DATA *imageId);

FSTATUS paFreezeFrameCreate(Pm_t *pm, STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *retImageId);

FSTATUS paFreezeFrameMove(Pm_t *pm, STL_PA_IMAGE_ID_DATA ffImageId, STL_PA_IMAGE_ID_DATA imageId,
	STL_PA_IMAGE_ID_DATA *returnImageId);

FSTATUS paGetFocusPorts(Pm_t *pm, char *groupName, PmFocusPorts_t *pmFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
	uint32 start, uint32 range);

FSTATUS paGetExtFocusPorts(Pm_t *pm, char *groupName, PmFocusPorts_t *pmFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
	uint32 start, uint32 range);

FSTATUS paGetMultiFocusPorts(Pm_t *pm, char *groupName, PmFocusPorts_t *pmFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 start,
	uint32 range, STL_FOCUS_PORT_TUPLE *tple, uint8 oper);

FSTATUS paGetImageInfo(Pm_t *pm, STL_PA_IMAGE_ID_DATA imageId, PmImageInfo_t *imageInfo,
	STL_PA_IMAGE_ID_DATA *returnImageId);

// get vf list - caller declares Pm_T and PmGroupList_t, and passes pointers
FSTATUS paGetVFList(Pm_t *pm, PmVFList_t *pmVFList, STL_PA_IMAGE_ID_DATA imageId,
	STL_PA_IMAGE_ID_DATA *returnImageId);

// get vf config - caller declares Pm_T and PmVFConfig_t, and passes pointers
FSTATUS paGetVFConfig(Pm_t *pm, char *vfName, uint64 vfSid, PmVFConfig_t *pmVFConfig,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId);

// get vf info - caller declares Pm_T and PmGroupInfo_t, and passes pointers
FSTATUS paGetVFInfo(Pm_t *pm, char *vfName, PmVFInfo_t *pmVFInfo, STL_PA_IMAGE_ID_DATA imageId,
	STL_PA_IMAGE_ID_DATA *returnImageId);

// get vf port stats - caller declares Pm_T and PmCompositeVLCounters_t
//                  delta - 1 requests delta counters, 0 gets raw total
//                  userCntrs - 1 requests PA user controled counters, 0 gets Pm Controlled Image Counters. if 1 delta and ofset must be 0
FSTATUS paGetVFPortStats(Pm_t *pm, STL_LID lid, uint8 portNum, char *vfName,
	PmCompositeVLCounters_t *vfPortCountersP, uint32 delta, uint32 userCntrs,
	STL_PA_IMAGE_ID_DATA imageId, uint32 *flagsp, STL_PA_IMAGE_ID_DATA *returnImageId);

FSTATUS paClearVFPortStats(Pm_t *pm, STL_LID lid, uint8 portNum, STLVlCounterSelectMask select,
	char *vfName);

FSTATUS paGetVFFocusPorts(Pm_t *pm, char *vfName, PmFocusPorts_t *pmVFFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
	uint32 start, uint32 range);

FSTATUS paGetExtVFFocusPorts(Pm_t *pm, char *vfName, PmFocusPorts_t *pmVFFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
	uint32 start, uint32 range);

static inline boolean pa_valid_port(PmPort_t *pmportp, boolean sth) {
	if (!pmportp ||
		/* if this is a sth image, the port may be 'empty' but not null
		   'Empty' ports should be excluded from the count, and can be
		   indentified by their having a port num and guid of 0 */
		(sth && !pmportp->guid && !pmportp->portNum)) return FALSE;
	return TRUE;
}

static inline uint8_t pa_get_port_status(PmPort_t *pmportp, uint32_t imageIndex)
{
	PmPortImage_t *portImage;
	if (!pmportp) {
		return STL_PA_FOCUS_STATUS_TOPO_FAILURE;
	} else if (pmportp->u.s.PmaAvoid) {
		// This means the PM was told to ignore this port during a sweep
		return STL_PA_FOCUS_STATUS_PMA_IGNORE;
	} else {
		portImage = &pmportp->Image[imageIndex];
		if (portImage->u.s.queryStatus != PM_QUERY_STATUS_OK) {
			// This means there was a failure during the PM sweep when querying this port
			return STL_PA_FOCUS_STATUS_PMA_FAILURE;
		} else if (portImage->u.s.UnexpectedClear) {
			// This means unexpected clear was set
			return STL_PA_FOCUS_STATUS_UNEXPECTED_CLEAR;
		}
	}
	return STL_PA_FOCUS_STATUS_OK;
}

/* PA STH MACROs for loops */
#define for_some_pmports_sth(PMNODE, PMPORT, PORTNUM, START, END, STH_BOOL) \
	for (PORTNUM = START, PMPORT = pm_get_port(PMNODE, PORTNUM); \
		PORTNUM <= END; \
		++PORTNUM, PMPORT = pm_get_port(PMNODE, PORTNUM)) \
	if (pa_valid_port(PMPORT, STH_BOOL))

#define for_all_pmports_sth(PMNODE, PMPORT, PORTNUM, STH_BOOL) \
	for_some_pmports_sth(PMNODE, PMPORT, PORTNUM, pm_get_port_idx(PMNODE), (PMNODE)->numPorts, STH_BOOL)

/** Example usage:
 *
 *  for_all_pmnodes(pmimagep, pmnodep lid) {
 *  	for_all_pmports(pmnodep, pmportp, portnum, sth) {
 *  		portImage = &pmportp->Image[imageIndex];
 *  		// DO WORK
 *  	}
 *  }
 */
#ifdef __cplusplus
};
#endif

#endif /* _PAACCESS_H */

