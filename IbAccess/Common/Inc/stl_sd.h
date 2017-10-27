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

 * ** END_ICS_COPYRIGHT7   ****************************************/
/* [ICS VERSION STRING: unknown] */

#if !defined(__STL_SD_H__)
#define __STL_SD_H__

#include "iba/ib_sd.h"
#include "iba/stl_sa_types.h"

#define NO_STL_MCMEMBER_OUTPUT      // Don't output STL McMember Records in SA tools
#define NO_STL_SERVICE_OUTPUT      // Don't output STL Service Records in SA tools

/*
 * Extensions to the QUERY_RESULT_TYPE found in ib_sd.h. 
 * NOTE: Also update iba2ibo_saquery_helper.c with text version of any additions! 
 */
#define OutputTypeStlBase 					0x1000
#define OutputTypeStlNodeRecord				(OutputTypeStlBase+1)
#define OutputTypeStlNodeDesc				(OutputTypeStlBase+2)
#define OutputTypeStlPortInfoRecord			(OutputTypeStlBase+3)
#define OutputTypeStlSwitchInfoRecord		(OutputTypeStlBase+4)
#define OutputTypeStlPKeyTableRecord		(OutputTypeStlBase+5)
#define OutputTypeStlSLSCTableRecord		(OutputTypeStlBase+6)
#define OutputTypeStlSMInfoRecord			(OutputTypeStlBase+7)
#define OutputTypeStlLinearFDBRecord		(OutputTypeStlBase+8)
#define OutputTypeStlVLArbTableRecord		(OutputTypeStlBase+9)
#ifndef NO_STL_MCMEMBER_OUTPUT       // Don't output STL McMember if defined
#define OutputTypeStlMcMemberRecord			(OutputTypeStlBase+10)
#endif
#define OutputTypeStlLid					(OutputTypeStlBase+11)
#define OutputTypeStlMCastFDBRecord			(OutputTypeStlBase+12)
#define OutputTypeStlLinkRecord				(OutputTypeStlBase+13)
#define OutputTypeStlSystemImageGuid		(OutputTypeStlBase+14)
#define OutputTypeStlPortGuid				(OutputTypeStlBase+15)
#define OutputTypeStlNodeGuid				(OutputTypeStlBase+16)
#ifndef NO_STL_SERVICE_OUTPUT       // Don't output STL Service if defined
#define OutputTypeStlServiceRecord			(OutputTypeStlBase+17)
#endif
#define OutputTypeStlInformInfoRecord		(OutputTypeStlBase+18)
#define OutputTypeStlVfInfoRecord			(OutputTypeStlBase+19)
#define OutputTypeStlTraceRecord			(OutputTypeStlBase+20)
#define OutputTypeStlQuarantinedNodeRecord	(OutputTypeStlBase+21)
#define OutputTypeStlCongInfoRecord         (OutputTypeStlBase+22)
#define OutputTypeStlSwitchCongRecord       (OutputTypeStlBase+23)
#define OutputTypeStlSwitchPortCongRecord   (OutputTypeStlBase+24)
#define OutputTypeStlHFICongRecord          (OutputTypeStlBase+25)
#define OutputTypeStlHFICongCtrlRecord      (OutputTypeStlBase+26)
#define OutputTypeStlBufCtrlTabRecord       (OutputTypeStlBase+27)
#define OutputTypeStlCableInfoRecord		(OutputTypeStlBase+28)
#define OutputTypeStlPortGroupRecord		(OutputTypeStlBase+29)
#define OutputTypeStlPortGroupFwdRecord		(OutputTypeStlBase+30)
#define OutputTypeStlSCSLTableRecord		(OutputTypeStlBase+31)
#define OutputTypeStlSCVLtTableRecord		(OutputTypeStlBase+32)
#define OutputTypeStlSCVLntTableRecord		(OutputTypeStlBase+33)
#define OutputTypeStlSCSCTableRecord		(OutputTypeStlBase+34)
#define OutputTypeStlClassPortInfo			(OutputTypeStlBase+35)
#define OutputTypeStlFabricInfoRecord		(OutputTypeStlBase+36)
#define OutputTypeStlSwitchCostRecord		(OutputTypeStlBase+53)

typedef union _OMGT_QUERY_INPUT_VALUE
{
	union _IbNodeRecord {
		IB_LID Lid; /* InputType */
		EUI64  PortGUID; /* InputType */
		EUI64  NodeGUID; /* InputType */
		EUI64  SystemImageGUID; /* InputType */
		NODE_TYPE NodeType; /* InputType */
		char NodeDesc[STL_NODE_DESCRIPTION_ARRAY_SIZE]; /* InputType */
	} IbNodeRecord; /* OutputType */
	union _IbNodeRecord NodeRecord; /* OutputType */
	union _LidOnly {
		IB_LID Lid; /* InputType */
	} IbPortInfoRecord; /* OutputType */
	union _LidOnly PortInfoRecord; /* OutputType */
	union _LidOnly LinkRecord; /* OutputType */
	union _LidOnly SwitchInfoRecord; /* OutputType */
	union _IbPathRecord {
		IB_GID SourceGid; /* InputType SourceGid */
		struct {
			uint8 SourceGuidCount; /* number of Source GUIDs in GuidList */
			uint8 DestGuidCount; /* number of Dest GUIDs in GuidList */
			EUI64 GuidList[MULTIPATH_GID_LIMIT]; /* Src GUIDs, followed by Dest GUIDs */
			EUI64 SharedSubnetPrefix;
		} PortGuidList; /* InputType */
		struct {
			uint8 SourceGidCount; /* number of Source GIDs in GidList */
			uint8 DestGidCount; /* number of Dest GIDs in GidList */
			IB_GID GidList[MULTIPATH_GID_LIMIT]; /* Src GIDs, followed by Dest GIDs */
		} GidList; /* InputType */
		struct {
			uint64 ComponentMask;
			IB_MULTIPATH_RECORD MultiPathRecord;
			/* Gids below allows up to 8 SGID and/or DGID in MultiPathRecord.GIDList */
			/* do not use Gids field directly, instead use */
			/* MultiPathRecord.GIDList[0-7] */
			IB_GID Gids[MULTIPATH_GID_LIMIT-1];
		} MultiPathRecord; /* InputType */
		struct {
			IB_GID SourceGid;
			uint16 PKey;
		} PKey; /* InputType */
		struct {
			IB_GID SourceGid;
			uint8 SL;
		} SL; /*  InputType */
		struct {
			IB_GID SourceGid;
			uint64 ServiceId;
		} ServiceId; /* InputType */
		struct {
			EUI64 SharedSubnetPrefix;
			EUI64 SourcePortGuid;
			EUI64 DestPortGuid;
		} PortGuidPair, PortGuid; /* InputType */
		struct {
			IB_GID SourceGid;
			IB_GID DestGid;
		} GidPair, PortGid; /* InputType */
		struct {
			IB_GID SourceGid;
			IB_LID DLid;
		} Lid; /* InputType */
		struct {
			uint64 ComponentMask;
			IB_PATH_RECORD PathRecord;
		} PathRecord;
	} IbPathRecord; /* OutputType */
	union _IbPathRecord TraceRecord; /* OutputType */
	union _IbServiceRecord {
		IB_GID ServiceGid; /* InputType */
	} IbServiceRecord; /* OutputType */
#ifndef NO_STL_SERVICE_OUTPUT
	union _StlServiceRecord {
		IB_LID Lid; /* InputType */
		IB_GID ServiceGid; /* InputType */
	} StlServiceRecord; /* OutputType */
#endif
	union _IbMcMemberRecord {
		IB_GID PortGid; /* InputType */
		IB_GID McGid; /* InputType */
		IB_LID Lid; /* InputType */
		uint16 PKey; /* InputType */
		uint8 SL;
	} IbMcMemberRecord; /* OutputType */
#ifndef NO_STL_MCMEMBER_OUTPUT
	union _IbMcMemberRecord StlMcMemberRecord; /* OutputType */
#endif
	union _IbInforInfoRecord {
		IB_GID SubscriberGID;
	} IbInformInfoRecord;
	union _StlInforInfoRecord {
		IB_LID SubscriberLID;
	} StlInformInfoRecord;
	union _LidOnly ScScTableRecord;
	union _LidOnly SlScTableRecord;
	union _LidOnly ScSlTableRecord;
	union _LidOnly ScVlxTableRecord;
	union _LidOnly VlArbTableRecord;
	union _LidOnly PKeyTableRecord;
	union _LidOnly LinFdbTableRecord;
	union _LidOnly McFdbTableRecord;
	union _VfInfoRecord {
		uint16 PKey;
		IB_LID Lid;
		uint8 SL;
		uint64 ServiceId;
		IB_GID McGid;
		uint16 vfIndex;
		char vfName[STL_NODE_DESCRIPTION_ARRAY_SIZE];
	} VfInfoRecord;
	union _LidOnly CongInfoRecord;
	union _LidOnly SwCongRecord;
	union _LidOnly SwPortCongRecord;
	union _LidOnly HFICongRecord;
	union _LidOnly HFICongCtrlRecord;
	union _LidOnly BufCtrlTableRecord;
	union _LidOnly CableInfoRecord;
	union _LidOnly PortGroupRecord;
	union _LidOnly PortGroupFwdRecord;
	union _LidOnly SwitchCostRecord;
} OMGT_QUERY_INPUT_VALUE;

typedef struct _OMGT_QUERY  {
    QUERY_INPUT_TYPE InputType;     /* Type of input (i.e. query based on) */
    QUERY_RESULT_TYPE OutputType;   /* Type of output (i.e. info requested) */
    OMGT_QUERY_INPUT_VALUE InputValue;   /* input record selection value input query */
} OMGT_QUERY, *POMGT_QUERY;



typedef struct {
	uint32							NumClassPortInfo;	/* Should always be 1 or 0 */
	STL_CLASS_PORT_INFO				ClassPortInfo;		/* Should never have more than 1 */
} STL_CLASS_PORT_INFO_RESULT, *PSTL_CLASS_PORT_INFO_RESULT;

typedef struct {
	uint32							NumFabricInfoRecords;/* Should always be 1 or 0 */
	STL_FABRICINFO_RECORD			FabricInfoRecord;	/* Should never have more than 1 */
} STL_FABRICINFO_RECORD_RESULT, *PSTL_FABRICINFO_RECORD_RESULT;

typedef struct {
    uint32 							NumNodeRecords;		/* Number of NodeRecords returned */
    STL_NODE_RECORD 				NodeRecords[1];		/* list of Node records returned */
} STL_NODE_RECORD_RESULTS, *PSTL_NODE_RECORD_RESULTS;

typedef struct {
    uint32							NumDescs;			/* Number of NodeDescs returned */
    STL_NODE_DESCRIPTION 			NodeDescs[1];		/* (STL) NodeDesc, null terminated. */
} STL_NODEDESC_RESULTS, *PSTL_NODEDESC_RESULTS;
	
typedef struct {
    uint32							NumPortInfoRecords;	/* Number of PortInfoRecords returned */
    STL_PORTINFO_RECORD 			PortInfoRecords[1];	/* list of PortInfoRecords returned */
} STL_PORTINFO_RECORD_RESULTS, *PSTL_PORTINFO_RECORD_RESULTS;

typedef struct {
	uint32							NumSwitchInfoRecords;
	STL_SWITCHINFO_RECORD			SwitchInfoRecords[1];

} STL_SWITCHINFO_RECORD_RESULTS, *PSTL_SWITCHINFO_RECORD_RESULTS;

typedef struct {
	uint32							NumPKeyTableRecords;
	STL_P_KEY_TABLE_RECORD			PKeyTableRecords[1];

} STL_PKEYTABLE_RECORD_RESULTS, *PSTL_PKEYTABLE_RECORD_RESULTS;

typedef struct {
	uint32							NumSCSCTableRecords;
	STL_SC_MAPPING_TABLE_RECORD		SCSCRecords[1];
} STL_SC_MAPPING_TABLE_RECORD_RESULTS, *PSTL_SC_MAPPING_TABLE_RECORD_RESULTS;

typedef struct {
	uint32							NumSLSCTableRecords;
	STL_SL2SC_MAPPING_TABLE_RECORD	SLSCRecords[1];
} STL_SL2SC_MAPPING_TABLE_RECORD_RESULTS, *PSTL_SL2SC_MAPPING_TABLE_RECORD_RESULTS;

typedef struct {
	uint32							NumSCSLTableRecords;
	STL_SC2SL_MAPPING_TABLE_RECORD	SCSLRecords[1];
} STL_SC2SL_MAPPING_TABLE_RECORD_RESULTS, *PSTL_SC2SL_MAPPING_TABLE_RECORD_RESULTS;

typedef struct {
	uint32							NumSCVLtTableRecords;
	STL_SC2PVL_T_MAPPING_TABLE_RECORD	SCVLtRecords[1];
} STL_SC2PVL_T_MAPPING_TABLE_RECORD_RESULTS, *PSTL_SC2PVL_T_MAPPING_TABLE_RECORD_RESULTS;

typedef struct {
	uint32							NumSCVLntTableRecords;
	STL_SC2PVL_NT_MAPPING_TABLE_RECORD	SCVLntRecords[1];
} STL_SC2PVL_NT_MAPPING_TABLE_RECORD_RESULTS, *PSTL_SC2PVL_NT_MAPPING_TABLE_RECORD_RESULTS;

typedef struct {
   uint32					NumLinearFDBRecords;
   STL_LINEAR_FORWARDING_TABLE_RECORD		LinearFDBRecords[1];
} STL_LINEAR_FDB_RECORD_RESULTS, *PSTL_LINEAR_FDB_RECORD_RESULTS;

typedef struct {
	uint32					NumSMInfoRecords;	/* Number of SmInfoRecords returned */
	STL_SMINFO_RECORD    			SMInfoRecords[1];	/* list of SMInfoRecords returned */
} STL_SMINFO_RECORD_RESULTS, *PSTL_SMINFO_RECORD_RESULTS;

typedef struct {
	uint32					NumVLArbTableRecords;
	STL_VLARBTABLE_RECORD			VLArbTableRecords[1];
} STL_VLARBTABLE_RECORD_RESULTS, *PSTL_VLARBTABLE_RECORD_RESULTS;

typedef struct {
	uint32					NumMcMemberRecords;
	STL_MCMEMBER_RECORD			McMemberRecords[1];
} STL_MCMEMBER_RECORD_RESULTS, *PSTL_MCMEMBER_RECORD_RESULTS;

typedef struct {
	uint32					NumLids;
	uint32					Lids[1];
} STL_LID_RESULTS, *PSTL_LID_RESULTS;

typedef struct {
	uint32					NumMCastFDBRecords;
	STL_MULTICAST_FORWARDING_TABLE_RECORD	MCastFDBRecords[1];
} STL_MCAST_FDB_RECORD_RESULTS, *PSTL_MCAST_FDB_RECORD_RESULTS;

typedef struct {
    uint32 					NumLinkRecords;
    STL_LINK_RECORD 		LinkRecords[1];		
} STL_LINK_RECORD_RESULTS, *PSTL_LINK_RECORD_RESULTS;

typedef struct {
	uint32					NumServiceRecords; 
	STL_SERVICE_RECORD		ServiceRecords[1];  
} STL_SERVICE_RECORD_RESULTS, *PSTL_SERVICE_RECORD_RESULTS;

typedef struct {
	uint32					NumInformInfoRecords; 
	STL_INFORM_INFO_RECORD	InformInfoRecords[1];  
} STL_INFORM_INFO_RECORD_RESULTS, *PSTL_INFORM_INFO_RECORD_RESULTS;

typedef struct {
	uint32					NumVfInfoRecords;
	STL_VFINFO_RECORD		VfInfoRecords[1];
} STL_VFINFO_RECORD_RESULTS, *PSTL_VFINFO_RECORD_RESULTS;

typedef struct {
	uint32					NumTraceRecords;
	STL_TRACE_RECORD		TraceRecords[1];
} STL_TRACE_RECORD_RESULTS, *STL_PTRACE_RECORD_RESULTS;

typedef struct {
	uint32					NumQuarantinedNodeRecords;
	STL_QUARANTINED_NODE_RECORD QuarantinedNodeRecords[1];
} STL_QUARANTINED_NODE_RECORD_RESULTS, *PSTL_QUARANTINED_NODE_RECORD_RESULTS;

typedef struct {
	uint32					NumRecords;
	STL_CONGESTION_INFO_RECORD Records[1];
} STL_CONGESTION_INFO_RECORD_RESULTS, *PSTL_CONGESTION_INFO_RECORD_RESULTS;
typedef struct {
	uint32					NumRecords;
	STL_SWITCH_CONGESTION_SETTING_RECORD Records[1];
} STL_SWITCH_CONGESTION_SETTING_RECORD_RESULTS, *PSTL_SWITCH_CONGESTION_SETTING_RECORD_RESULTS;
typedef struct {
	uint32					NumRecords;
	STL_SWITCH_PORT_CONGESTION_SETTING_RECORD Records[1];
} STL_SWITCH_PORT_CONGESTION_SETTING_RECORD_RESULTS, *PSTL_SWITCH_PORT_CONGESTION_SETTING_RECORD_RESULTS;
typedef struct {
	uint32					NumRecords;
	STL_HFI_CONGESTION_SETTING_RECORD Records[1];
} STL_HFI_CONGESTION_SETTING_RECORD_RESULTS, *PSTL_HFI_CONGESTION_SETTING_RECORD_RESULTS;
typedef struct {
	uint32					NumRecords;
	STL_HFI_CONGESTION_CONTROL_TABLE_RECORD Records[1];
} STL_HFI_CONGESTION_CONTROL_TABLE_RECORD_RESULTS, *PSTL_HFI_CONGESTION_CONTROL_TABLE_RECORD_RESULTS;

typedef struct {
	uint32					NumBufferControlRecords;
	STL_BUFFER_CONTROL_TABLE_RECORD BufferControlRecords[1];
} STL_BUFFER_CONTROL_TABLE_RECORD_RESULTS, *PSTL_BUFFER_CONTROL_TABLE_RECORD_RESULTS;

typedef struct {
	uint32					NumCableInfoRecords;
	STL_CABLE_INFO_RECORD	CableInfoRecords[1];
} STL_CABLE_INFO_RECORD_RESULTS, *PSTL_CABLE_INFO_RECORD_RESULTS;

typedef struct {
	uint32					NumRecords;
	STL_PORT_GROUP_TABLE_RECORD Records[1];
} STL_PORT_GROUP_TABLE_RECORD_RESULTS, *PSTL_PORT_GROUP_TABLE_RECORD_RESULTS;
typedef struct {
	uint32					NumRecords;
	STL_PORT_GROUP_FORWARDING_TABLE_RECORD Records[1];
} STL_PORT_GROUP_FORWARDING_TABLE_RECORD_RESULTS, *PSTL_PORT_GROUP_FORWARDING_TABLE_RECORD_RESULTS;

typedef struct {
	uint32					NumRecords;
	STL_SWITCH_COST_RECORD	Records[1];
} STL_SWITCH_COST_RECORD_RESULTS, *PSTL_SWITCH_COST_RECORD_RESULTS;

#endif
