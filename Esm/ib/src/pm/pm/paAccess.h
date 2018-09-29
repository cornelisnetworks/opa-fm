/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

#ifdef __cplusplus
extern "C" {
#endif

#define FSTATUS_ToString(status) ((status & 0xFF) < FSTATUS_COUNT ? FSTATUS_Strings[status] : "")

// DEFINES

#define	BAD_IMAGE_ID	(uint64)-1

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
//	STL_FOCUS_PORT_TUPLE	tuple[MAX_NUM_FOCUS_PORT_TUPLES];

} PmFocusPortEntry_t;

typedef struct _pmFocusPorts_s {
	char			groupName[STL_PM_GROUPNAMELEN];	// \0 terminated
	uint32			NumPorts;
	uint32			portCntr;
	PmFocusPortEntry_t	*portList;
} PmFocusPorts_t;

typedef struct _sortedValueEntry_s {
	STL_LID					lid;
	uint8						portNum;
	uint8						localStatus : 4;
	uint8						neighborStatus : 4;
	uint8						reserved2[2];
	uint64						value[MAX_NUM_FOCUS_PORT_TUPLES];
	uint64						neighborValue[MAX_NUM_FOCUS_PORT_TUPLES];
	uint64						sortValue;
	PmPort_t					*portp;
	PmPort_t					*neighborPortp;
	struct _sortedValueEntry_s	*next;
	struct _sortedValueEntry_s	*prev;
} sortedValueEntry_t;

typedef struct _sortInfo_s {
	sortedValueEntry_t			*sortedValueListPool;
	sortedValueEntry_t			*sortedValueListHead;
	sortedValueEntry_t			*sortedValueListTail;
	uint32						sortedValueListSize;
	uint32						numValueEntries;
} sortInfo_t;

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

typedef struct _pmVFFocusPorts_s {
	char			vfName[STL_PM_VFNAMELEN];	// \0 terminated
	uint32			NumPorts;
	uint32			portCntr;
	PmFocusPortEntry_t	*portList;
} PmVFFocusPorts_t;

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

// Note - API expects caller to define a Pm_t structure, which apps will do for now
//        Later, PM will handle this, and the parameter can be removed

// get group list - caller declares Pm_T and PmGroupList_t, and passes pointers
FSTATUS paGetGroupList(Pm_t *pm, PmGroupList_t *GroupList, uint32 imageIndex);

// get group info - caller declares Pm_T and PmGroupInfo_t, and passes pointers
FSTATUS paGetGroupInfo(Pm_t *pm, char *groupName, PmGroupInfo_t *pmGroupInfo,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId);

// get group config - caller declares Pm_T and PmGroupConfig_t, and passes pointers
FSTATUS paGetGroupConfig(Pm_t *pm, char *groupName, PmGroupConfig_t *pmGroupConfig,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId);

// get group node info - caller declares Pm_T and PmGroupConfig_t, and passes pointers
FSTATUS paGetGroupNodeInfo(Pm_t *pm, char *groupName, uint64 nodeGUID, STL_LID nodeLID, char *nodeDesc,
	PmGroupNodeInfo_t *pmGroupNodeInfo, STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId);

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
	uint32 start, uint32 range, STL_FOCUS_PORT_TUPLE *tuple, uint8 logical_operator);

FSTATUS paGetExtFocusPorts(Pm_t *pm, char *groupName, PmFocusPorts_t *pmFocusPorts,
        STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
        uint32 start, uint32 range);

FSTATUS paGetImageInfo(Pm_t *pm, STL_PA_IMAGE_ID_DATA imageId, PmImageInfo_t *imageInfo,
	STL_PA_IMAGE_ID_DATA *returnImageId);

// get vf list - caller declares Pm_T and PmGroupList_t, and passes pointers
FSTATUS paGetVFList(Pm_t *pm, PmVFList_t *pmVFList, uint32 imageIndex);

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

FSTATUS paClearVFPortStats(Pm_t *pm, STL_LID lid, uint8 portNum, STLVlCounterSelectMask select, char *vfName);

FSTATUS paGetVFFocusPorts(Pm_t *pm, char *vfName, PmVFFocusPorts_t *pmVFFocusPorts,
	STL_PA_IMAGE_ID_DATA imageId, STL_PA_IMAGE_ID_DATA *returnImageId, uint32 select,
	uint32 start, uint32 range);

FSTATUS paGetExtVFFocusPorts(Pm_t *pm, char *vfName, PmVFFocusPorts_t *pmVFFocusPorts,
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


#ifdef __cplusplus
};
#endif

#endif /* _PAACCESS_H */

