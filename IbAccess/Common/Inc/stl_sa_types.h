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

/*
 * SA MADs
 *
 * General Notes:
 *
 * (1)	Structures are defined in the most readable manner rather than 
 *		what might be most useful for the coder or the compiler. Compiler 
 *		hints (i.e., PACK_SUFFIX) are omitted, blocks of data may be 
 *		defined as multi-dimensional arrays when useful, and so on.
 *
 * (2)	An effort has been made to ensure that 32-bit fields fall on a 4 byte
 *		boundary and 64-bit fields fall on an 8 byte boundary. In addition,
 *		SA records are set as multiples of 8-bytes in order to preserve
 *		alignment across multi-record responses.
 *
 * (3)	When extending a LID to 32-bits would break word alignment of other
 *		fields, the process was to:
 *			(a) First, remove existing reserved fields if this will restore
 *				alignment.
 *			(b) If (a) is not possible, consider moving fields if this will
 *				restore alignment.
 *			(c) If (b) fails, add new reserved fields to restore alignment.
 * 
 * (4)	In this document, attributes are listed in the same order they appear
 *		in the IB spec. Where a new attribute is added, it is added after
 *		the existing attribute it was modeled after.
 *
 * (5)	The use of "Jumbo" (>256 byte) MADs are only used when needed. For
 *		table-based MADs, RMPP is the preferred solution.
 *
 */

#ifndef __STL_SA_H__
#define __STL_SA_H__

#include "iba/stl_mad_types.h"
#include "iba/ib_sa_records.h"

#if defined (__cplusplus)
extern "C" {
#endif

#include "iba/public/ipackon.h"

#define STL_SA_CLASS_VERSION				0x80

/* 
 * Subnet Administration Attribute IDs Adapted from IB
 */
#define	STL_SA_ATTR_CLASS_PORT_INFO				0x0001
#define	STL_SA_ATTR_NOTICE						0x0002
#define	STL_SA_ATTR_INFORM_INFO					0x0003
#define	STL_SA_ATTR_NODE_RECORD					0x0011
#define STL_SA_ATTR_PORTINFO_RECORD				0x0012
#define	STL_SA_ATTR_SC_MAPTBL_RECORD			0x0013 // REPLACES SL TO VL!
#define	STL_SA_ATTR_SWITCHINFO_RECORD			0x0014
#define	STL_SA_ATTR_LINEAR_FWDTBL_RECORD		0x0015
//#define	STL_SA_ATTR_RANDOM_FWD_TBL_RECORD	0x0016 // Undefined in STL.
#define	STL_SA_ATTR_MCAST_FWDTBL_RECORD			0x0017
#define	STL_SA_ATTR_SMINFO_RECORD				0x0018
#define STL_SA_ATTR_LINK_SPD_WDTH_PAIRS_RECORD	0x0019 // Defined but never impl'ed
//Available										0x001A-0x001F
#define	STL_SA_ATTR_LINK_RECORD					0x0020
//#define	STL_SA_ATTR_GUIDINFO_RECORD			0x0030 // Undefined in STL.
#define	STL_SA_ATTR_SERVICE_RECORD				0x0031
#define	STL_SA_ATTR_P_KEY_TABLE_RECORD			0x0033
#define	STL_SA_ATTR_PATH_RECORD					0x0035
#define	STL_SA_ATTR_VLARBTABLE_RECORD			0x0036
#define	STL_SA_ATTR_MCMEMBER_RECORD				0x0038
#define	STL_SA_ATTR_TRACE_RECORD				0x0039
#define	STL_SA_ATTR_MULTIPATH_GID_RECORD		0x003A 
#define	STL_SA_ATTR_SERVICEASSOCIATION_RECORD	0x003B	/* not implemented */
//Available										0x003C-0x007F
#define	STL_SA_ATTR_INFORM_INFO_RECORD			0x00F3

/*
 * Subnet Administration Attribute IDs New for STL
 */
#define STL_SA_ATTR_SL2SC_MAPTBL_RECORD			0x0080
#define STL_SA_ATTR_SC2SL_MAPTBL_RECORD			0x0081 
#define STL_SA_ATTR_SC2VL_NT_MAPTBL_RECORD		0x0082 
#define STL_SA_ATTR_SC2VL_T_MAPTBL_RECORD		0x0083 
#define STL_SA_ATTR_SC2VL_R_MAPTBL_RECORD		0x0084 
#define	STL_SA_ATTR_PGROUP_FWDTBL_RECORD		0x0085 
#define	STL_SA_ATTR_MULTIPATH_GUID_RECORD		0x0086 	/* not implemented */
#define	STL_SA_ATTR_MULTIPATH_LID_RECORD		0x0087 	/* not implemented */
#define STL_SA_ATTR_CABLE_INFO_RECORD			0x0088 
#define STL_SA_ATTR_VF_INFO_RECORD				0x0089 // Previously vendor specific
#define STL_SA_ATTR_PORT_STATE_INFO_RECORD		0x008A 
#define STL_SA_ATTR_PORTGROUP_TABLE_RECORD		0x008B 
#define STL_SA_ATTR_BUFF_CTRL_TAB_RECORD		0x008C 
#define STL_SA_ATTR_FABRICINFO_RECORD			0x008D
#define STL_SA_ATTR_QUARANTINED_NODE_RECORD		0x0090 // Previously vendor specific
#define STL_SA_ATTR_CONGESTION_INFO_RECORD		0x0091 // Previously vendor specific
#define STL_SA_ATTR_SWITCH_CONG_RECORD			0x0092 // Previously vendor specific
#define STL_SA_ATTR_SWITCH_PORT_CONG_RECORD		0x0093 // Previously vendor specific
#define STL_SA_ATTR_HFI_CONG_RECORD				0x0094 // Previously vendor specific
#define STL_SA_ATTR_HFI_CONG_CTRL_RECORD		0x0095 // Previously vendor specific



#define STL_SA_ATTR_DG_MEMBER_RECORD			0x009B
#define STL_SA_ATTR_DG_NAME_RECORD				0x009C
#define STL_SA_ATTR_DT_MEMBER_RECORD			0x009D


#define STL_SA_ATTR_SWITCH_COST_RECORD			0x00A3

//#define STL_SA_ATTR_JOB_ROUTE_RECORD			0xffb2  // Never implemented.
//#define STL_SA_ATTR_CG_RECORD        			0xff40  // Never implemented.
//#define STL_SA_ATTR_CG_STATUS_RECORD 			0xff41  // Never implemented.
//#define STL_SA_ATTR_CFT_RECORD       			0xff42  // Never implemented.

/* Subnet Administration MAD status values */
#define STL_MAD_STATUS_STL_SA_UNAVAILABLE	    0x0100  // SA unavailable

/*
 * SA capability mask defines
 */
#define STL_SA_CAPABILITY_MULTICAST_SUPPORT      0x0200
#define STL_SA_CAPABILITY_MULTIPATH_SUPPORT      0x0400
#define STL_SA_CAPABILITY_PORTINFO_CAPMASK_MATCH 0x2000
#define STL_SA_CAPABILITY_PA_SERVICES_SUPPORT    0x8000

/* 32 bit values, lower 27 bits are capabilities*/
#define STL_SA_CAPABILITY2_QOS_SUPPORT            0x0000002
#define STL_SA_CAPABILITY2_MFTTOP_SUPPORT         0x0000008
#define STL_SA_CAPABILITY2_FULL_PORTINFO          0x0000040
#define STL_SA_CAPABILITY2_EXT_SUPPORT            0x0000080
#define STL_SA_CAPABILITY2_DGDTRECORD_SUPPORT	  0x1000000
#define STL_SA_CAPABILITY2_SWCOSTRECORD_SUPPORT   0x2000000

static __inline void
StlSaClassPortInfoCapMask(char buf[80], uint16 cmask)
{
	if (!cmask) {
		snprintf(buf, 80, "-");
	} else {
		snprintf(buf, 80, "%s%s%s%s%s%s%s",
			(cmask & STL_CLASS_PORT_CAPMASK_TRAP) ? "Trap " : "",
			(cmask & STL_CLASS_PORT_CAPMASK_NOTICE) ? "Notice " : "",
			(cmask & STL_CLASS_PORT_CAPMASK_CM2) ? "CapMask2 " : "",
			/* Class Specific */
			(cmask & STL_SA_CAPABILITY_MULTICAST_SUPPORT) ? "MultiCast " : "",
			(cmask & STL_SA_CAPABILITY_MULTIPATH_SUPPORT) ? "MultiPath " : "",
			(cmask & STL_SA_CAPABILITY_PORTINFO_CAPMASK_MATCH) ? "PortInfoMask " : "",
			(cmask & STL_SA_CAPABILITY_PA_SERVICES_SUPPORT) ? "PartService " : "");
	}
}
static __inline void
StlSaClassPortInfoCapMask2(char buf[80], uint32 cmask)
{
	if (!cmask) {
		snprintf(buf, 80, "-");
	} else {
		snprintf(buf, 80, "%s%s%s%s%s%s",
			(cmask & STL_SA_CAPABILITY2_QOS_SUPPORT) ? "QoS " : "",
			(cmask & STL_SA_CAPABILITY2_MFTTOP_SUPPORT) ? "MFTTop " : "",
			(cmask & STL_SA_CAPABILITY2_FULL_PORTINFO) ? "FullPortInfo " : "",
			(cmask & STL_SA_CAPABILITY2_EXT_SUPPORT) ? "ExtSpeed " : "",
			(cmask & STL_SA_CAPABILITY2_DGDTRECORD_SUPPORT) ? "DG/DT " : "",
			(cmask & STL_SA_CAPABILITY2_SWCOSTRECORD_SUPPORT) ? "SwCost " : "");
	}
}

/*
 * NodeRecord
 *
 * STL Differences:
 * 		Extended LID to 32 bits.
 *		Reserved added to 8-byte-align structures.
 */
typedef struct {
	struct {
		STL_LID	LID;
	} PACK_SUFFIX RID;
	
	uint32		Reserved;				

	STL_NODE_INFO NodeInfo;
	
	STL_NODE_DESCRIPTION NodeDesc;

} PACK_SUFFIX STL_NODE_RECORD;

/* ComponentMask bits */
#define STL_NODE_RECORD_COMP_LID					0x00000001
/* reserved field									0x00000002 */
#define STL_NODE_RECORD_COMP_BASEVERSION			0x00000004
#define STL_NODE_RECORD_COMP_CLASSVERSION			0x00000008
#define STL_NODE_RECORD_COMP_NODETYPE				0x00000010
#define STL_NODE_RECORD_COMP_NUMPORTS				0x00000020
/* reserved field									0x00000040 */
#define STL_NODE_RECORD_COMP_SYSIMAGEGUID			0x00000080
#define STL_NODE_RECORD_COMP_NODEGUID				0x00000100
#define STL_NODE_RECORD_COMP_PORTGUID				0x00000200
#define STL_NODE_RECORD_COMP_PARTITIONCAP			0x00000400
#define STL_NODE_RECORD_COMP_DEVICEID				0x00000800
#define STL_NODE_RECORD_COMP_REVISION				0x00001000
#define STL_NODE_RECORD_COMP_LOCALPORTNUM			0x00002000
#define STL_NODE_RECORD_COMP_VENDORID				0x00004000
#define STL_NODE_RECORD_COMP_NODEDESC				0x00008000

/*
 * PortInfoRecord
 *
 * STL Differences:
 * 		EndPortLID extended to 32 bits.
 *		RID.Reserved field changed to a bitfield to comply with proposed
 *			IBTA 1.3 spec.
 *		Reserved added to qword-align PortInfoData.
 *		RID.PortNum for HFI will return HFI port number
 */
#define STL_NUM_LINKDOWN_REASONS 8

typedef struct {
	uint8 Reserved[6];
	uint8 NeighborLinkDownReason;
	uint8 LinkDownReason;
	uint64 Timestamp;
} PACK_SUFFIX STL_LINKDOWN_REASON;

/**
 * Returns the next index in an array of STL_LINKDOWN_REASONS of size
 * STL_NUM_LINKDOWN_REASONS to use. Uses the first free space or the
 * LinkDownReason with the oldest timestamp.
 */
static __inline__ int STL_LINKDOWN_REASON_NEXT_INDEX(STL_LINKDOWN_REASON* ldr) {
	int i;
	int selectedIndex = 0;
	uint64 earliestTimestamp = 0xffffffffffffffffll;

	for(i = 0; i < STL_NUM_LINKDOWN_REASONS; i++) {
		if(ldr[i].Timestamp == 0) {
			return i;
		} else if(ldr[i].Timestamp < earliestTimestamp){
			selectedIndex = i;
			earliestTimestamp = ldr[i].Timestamp;
		}
	}

	return selectedIndex;
}

/**
 * Returns the most recent stored LinkDownReason
 */
static __inline__ int STL_LINKDOWN_REASON_LAST_INDEX(STL_LINKDOWN_REASON* ldr) {
	int i;
	int selectedIndex = -1;
	uint64 earliestTimestamp = 0x0ll;

	for(i = 0; i < STL_NUM_LINKDOWN_REASONS; i++) {
		if(ldr[i].Timestamp >  earliestTimestamp){
			selectedIndex = i;
			earliestTimestamp = ldr[i].Timestamp;
		}
	}

	return selectedIndex;
}

typedef struct {
	struct {
		STL_LID	EndPortLID;				
		uint8	PortNum;			/* for switch or HFI: port numnber */
		uint8	Reserved;
	} PACK_SUFFIX RID;
	
	uint16		Reserved;	

	STL_PORT_INFO PortInfo;
    STL_LINKDOWN_REASON LinkDownReasons[STL_NUM_LINKDOWN_REASONS];
	
} PACK_SUFFIX STL_PORTINFO_RECORD;

/* ComponentMask bits */
/* 
 * The component mask bits for port info deviate from the IB standard
 * because there are too many fields in port info to enumerate. At
 * this time only the following mask bits are defined. All other bits
 * are reserved for future expansion.
 */
#define STL_PORTINFO_RECORD_COMP_ENDPORTLID				0x0000000000000001ll
#define STL_PORTINFO_RECORD_COMP_PORTNUM				0x0000000000000002ll
#define STL_PORTINFO_RECORD_COMP_OPTIONS				0x0000000000000004ll
#define STL_PORTINFO_RECORD_COMP_CAPABILITYMASK			0x0000000000000008ll

/*
 * P_KeyTableRecord
 * 
 * STL Differences:
 *		LID extended to 32 bits.
 *		Reserved shortened to restore alignment.
 *		RID.PortNum for HFI will return HFI port number
 */
typedef struct {
	struct {
		uint32	LID;
		uint16	Blocknum;
		uint8	PortNum;			/* for switch or HFI: port numnber */
	} PACK_SUFFIX RID;
	
	uint8		Reserved;	 
	
	STL_PARTITION_TABLE	PKeyTblData;
	
} PACK_SUFFIX STL_P_KEY_TABLE_RECORD;

#define STL_PKEYTABLE_RECORD_COMP_LID					0x00000001
#define STL_PKEYTABLE_RECORD_COMP_BLOCKNUM				0x00000002
#define STL_PKEYTABLE_RECORD_COMP_PORTNUM				0x00000004

/*
 * SCMappingTableRecord
 *
 * Used for querying the SA for SC to SC Mapping information about a 
 * single input/output pair.
 *
 *
 * STL Differences:
 * 		LID extended to 32 bits. Each entry in the table is widened to 8 bits, 
 *			the top 3 bits of each entry MUST be zero. This was done in 
 *			preference to trying to pack 5 bit fields into multiples of 8 bits.
 *
 */
typedef struct {
	struct {
		STL_LID	LID;	
		uint8	InputPort;				
		uint8	OutputPort;
	} PACK_SUFFIX RID;
	uint16		Reserved;
	
	STL_SC		Map[STL_MAX_SCS]; 	
	
} PACK_SUFFIX STL_SC_MAPPING_TABLE_RECORD;

#define STL_SC2SC_RECORD_COMP_LID 			0x0000000000000001ull
#define STL_SC2SC_RECORD_COMP_INPUTPORT		0x0000000000000002ull
#define STL_SC2SC_RECORD_COMP_OUTPUTPORT	0x0000000000000004ull

/*
 * SL2SCMappingTableRecord
 *
 * STL Differences:
 * 		New for STL.
 */
typedef struct {
	struct {
		STL_LID	LID;	
		uint16	Reserved;				
	} PACK_SUFFIX RID;
	
	uint16		Reserved2;
	
	STL_SC		SLSCMap[STL_MAX_SLS]; 	
	
} PACK_SUFFIX STL_SL2SC_MAPPING_TABLE_RECORD;

#define STL_SL2SC_RECORD_COMP_LID 0x0000000000000001ull

/*
 * SC2SLMappingTableRecord
 *
 * STL Differences:
 * 		New for STL.
 */
typedef struct {
	struct {
		STL_LID	LID;	
		uint16	Reserved;				
	} PACK_SUFFIX RID;
	
	uint16		Reserved2;
	
	STL_SL		SCSLMap[STL_MAX_SCS]; 	
	
} PACK_SUFFIX STL_SC2SL_MAPPING_TABLE_RECORD;

#define STL_SC2SL_RECORD_COMP_LID 0x0000000000000001ull

/*
 * SC2VL Mapping Table Records
 *
 * There are three possible SC to VL mapping tables: NT, T and R. SC2VL_R 
 * will not be implemented in STL Gen 1. While they are all three separate 
 * SA MAD attributes, they all have identical structure.
 */
typedef struct {
	struct {
		STL_LID	LID;
		uint8	Port; 				/* for switch or HFI: port numnber */
	} PACK_SUFFIX RID;
	
	uint8		Reserved[3];
	
	STL_VL		SCVLMap[STL_MAX_SCS]; 	
	
} PACK_SUFFIX STL_SC2VL_R_MAPPING_TABLE_RECORD;

typedef STL_SC2VL_R_MAPPING_TABLE_RECORD STL_SC2PVL_T_MAPPING_TABLE_RECORD;
typedef STL_SC2VL_R_MAPPING_TABLE_RECORD STL_SC2PVL_NT_MAPPING_TABLE_RECORD;
typedef STL_SC2VL_R_MAPPING_TABLE_RECORD STL_SC2PVL_R_MAPPING_TABLE_RECORD;

#define STL_SC2VL_R_RECORD_COMP_LID 0x0000000000000001ull
#define STL_SC2VL_R_RECORD_COMP_PORT 0x0000000000000002ull

/*
 * SwitchInfoRecord
 *
 * STL Differences
 *		Old LID/Reserved RID replaced with 32 bit LID.
 *		Reserved added to align SwitchInfoData.
 */
typedef struct {
	struct {
		STL_LID 	LID;
	} PACK_SUFFIX RID;

	uint32		Reserved;		

	STL_SWITCH_INFO	SwitchInfoData; 	
} PACK_SUFFIX STL_SWITCHINFO_RECORD;

#define STL_SWITCHINFO_RECORD_COMP_LID 				0x0000000000000001ull
/* Reserved											0x0000000000000002ull */
#define STL_SWITCHINFO_RECORD_COMP_LFDBCAP			0x0000000000000004ull
/* Reserved											0x0000000000000008ull */
#define STL_SWITCHINFO_RECORD_COMP_MFDBCAP			0x0000000000000010ull
#define STL_SWITCHINFO_RECORD_COMP_LFDBTOP			0x0000000000000020ull
/* Reserved											0x0000000000000040ull */
#define STL_SWITCHINFO_RECORD_COMP_MFDBTOP			0x0000000000000080ull
#define STL_SWITCHINFO_RECORD_COMP_COLLCAP			0x0000000000000100ull
#define STL_SWITCHINFO_RECORD_COMP_COLLTOP			0x0000000000000200ull
/* Reserved											0x0000000000000400ull */
#define STL_SWITCHINFO_RECORD_COMP_IPPRIMARY		0x0000000000000800ull
#define STL_SWITCHINFO_RECORD_COMP_IPSECONDARY		0x0000000000001000ull
#define STL_SWITCHINFO_RECORD_COMP_PORTSTATECHG		0x0000000000020000ull
#define STL_SWITCHINFO_RECORD_COMP_LIFETIME			0x0000000000040000ull
#define STL_SWITCHINFO_RECORD_COMP_PENFCAP			0x0000000000080000ull
#define STL_SWITCHINFO_RECORD_COMP_PORTGROUPCAP 	0x0000000000100000ull
#define STL_SWITCHINFO_RECORD_COMP_PORTGROUPTOP 	0x0000000000200000ull
#define STL_SWITCHINFO_RECORD_COMP_RMODESUPPORTEED	0x0000000000400000ull
#define STL_SWITCHINFO_RECORD_COMP_RMODEENABLED		0x0000000000800000ull
/* Reserved											0x0000000001000000ull */
/* Reserved											0x0000000002000000ull */
/* Reserved											0x0000000004000000ull */
/* Reserved											0x0000000008000000ull */
#define STL_SWITCHINFO_RECORD_COMP_EP0				0x0000000010000000ull
/* Reserved											0x0000000020000000ull */
/* Reserved											0x0000000040000000ull */
#define STL_SWITCHINFO_RECORD_COMP_COLLMASK			0x0000000080000000ull
#define STL_SWITCHINFO_RECORD_COMP_MCMASK 			0x0000000100000000ull
#define STL_SWITCHINFO_RECORD_COMP_ARLOSTONLY 		0x0000000200000000ull
#define STL_SWITCHINFO_RECORD_COMP_ARPAUSE 			0x0000000400000000ull
#define STL_SWITCHINFO_RECORD_COMP_ARENABLE 		0x0000000800000000ull
#define STL_SWITCHINFO_RECORD_COMP_ARALGORITHM 		0x0000001000000000ull
#define STL_SWITCHINFO_RECORD_COMP_ARFREQ 			0x0000002000000000ull
#define STL_SWITCHINFO_RECORD_COMP_ARTHRESHOLD 		0x0000004000000000ull
/* Reserved											0x0000008000000000ull */
#define STL_SWITCHINFO_RECORD_COMP_CAPMASK 			0x0000010000000000ull
#define STL_SWITCHINFO_RECORD_COMP_CMCOLLECTIVES	0x0000020000000000ull

/*
 * LFTRecord 
 *
 * Blocks are still defined as 64 bytes long to be consistent with IB.
 *
 * STL Differences:
 *
 * 	LID extended to 32 bits.
 *	BlockNum extended to 18 bits.
 */
typedef struct {
	struct {
		STL_LID	LID;	
		IB_BITFIELD2(uint32, 
				Reserved:14,
				BlockNum:18);
	} PACK_SUFFIX RID;

	/* 8 bytes */

	uint8 		LinearFdbData[64];
	
	/* 72 bytes */
} PACK_SUFFIX STL_LINEAR_FORWARDING_TABLE_RECORD;

#define STL_LFT_RECORD_COMP_LID 			0x0000000000000001ull
/* Reserved 								0x0000000000000002ull */
#define STL_LFT_RECORD_COMP_BLOCKNUM		0x0000000000000004ull


/*
 * MFTRecord 
 * 
 * NOTES:
 * 		In IB the width of the PORTMASK data type was defined as only 16 
 *		bits, requiring the SM to iterate over 3 different positions values 
 *		to retrieve the MFTs for a 48-port switch. 
 *		For this reason PORTMASK is now defined as 64 bits wide, eliminating 
 *		the need to use the "position" attribute in the Gen 1 & Gen 2 
 *		generations of hardware. 
 *
 *		As above, a "block" is defined as 64 bytes; therefore a single block 
 *		will contain 8 MFT records. The consumer should use GetTable() and 
 *		RMPP to retrieve more than one block. As with the RFT, BlockNum is 
 *		defined as 21 bits, providing for a total of 2^24 LIDs.
 *
 * STL Differences:
 *		PORTMASK is now 64 bits.
 *		LID is now 32 bits.
 *		Position is now 2 bits.
 *		Reserved is now 9 bits.
 *		BlockNum is now 21 bits.
 *		Reserved2 removed to preserve word alignment.
 */
 
#define STL_MFTB_WIDTH 64
#define STL_MFTB_MAX_POSITION 4
typedef struct _STL_MULTICAST_FORWARDING_TABLE_RECORD {
	struct {
		STL_LID		LID; 				// Port 0 of the switch.	
	
		STL_FIELDUNION3(u1, 32,
				Position:2,			
				Reserved:9,
				BlockNum:21);
	} PACK_SUFFIX RID;

	STL_MULTICAST_FORWARDING_TABLE MftTable;
	
} PACK_SUFFIX STL_MULTICAST_FORWARDING_TABLE_RECORD;

#define STL_MFTB_RECORD_COMP_LID 			0x0000000000000001ull
#define STL_MFTB_RECORD_COMP_POSITIONl		0x0000000000000002ull
/* Reserved 								0x0000000000000004ull */
#define STL_MFTB_RECORD_COMP_BLOCKNUM		0x0000000000000008ull

/*
 * Port Group Table Record 
 * 
 * Defines which ports are associated with which port groups. 
 * For each query, BlockNum defines a block of port groups, from 0-31,
 * with each block containing 8 port groups. Position identifies a group 
 * of ports with 0 being ports 1-64, 1 being ports 65-128, etc.
 *
 * This is nearly optimal; only switches that support more than 248 port groups
 * will require two queries to read the entire table.
 *
 */
#define STL_PGTB_NUM_ENTRIES_PER_BLOCK  8
typedef struct {
	struct {
		STL_LID	LID;
		IB_BITFIELD3(uint16,
				Position:2,
				Reserved:9,
				BlockNum:5);
	} PACK_SUFFIX RID;
	
	uint16		Reserved2;
				
	STL_PORTMASK GroupBlock[STL_PGTB_NUM_ENTRIES_PER_BLOCK];
	
} PACK_SUFFIX STL_PORT_GROUP_TABLE_RECORD;

#define STL_PGTB_RECORD_COMP_LID 			0x0000000000000001ull
#define STL_PGTB_RECORD_COMP_POSITION		0x0000000000000002ull
#define STL_PGTB_RECORD_COMP_BLOCKNUM		0x0000000000000004ull

/*
 * Port Group Forwarding Table Record 
 * 
 * Maps LIDs to Port Groups.
 *
 * New for STL. Similar to the preceding forwarding table records. 
 *
 */

#define STL_PGFDB_NUM_ENTRIES_PER_BLOCK 64
typedef struct {	
	struct {
		STL_LID	LID;	
	
		STL_FIELDUNION2(u1, 32,
				Reserved:14, 
				BlockNum:18);
	} PACK_SUFFIX RID;

	PORT 		PGFdbData[STL_PGFDB_NUM_ENTRIES_PER_BLOCK];
	
} PACK_SUFFIX STL_PORT_GROUP_FORWARDING_TABLE_RECORD;

#define STL_PGFWDTB_RECORD_COMP_LID 			0x0000000000000001ull
#define STL_PGFWDTB_RECORD_COMP_BLOCKNUM		0x0000000000000002ull

/*
 * VLArbitrationRecord
 * 
 * STL Differences:
 *		Switch LID extended.
 *		Blocknum now defined as 0 - 3 as per the VL Arbitration Table MAD.
 *		Length of Low, High tables extended to 128 bytes.
 *		Preempt table added.
 *		RID.OutputPortNum for HFI will return port number
 */
typedef struct {
	struct {
		STL_LID	LID;				
		uint8	OutputPortNum;		/* for switch or HFI: port numnber */
		uint8	BlockNum;
	} PACK_SUFFIX RID;

	uint16		Reserved;
	
	STL_VLARB_TABLE VLArbTable;
	
} PACK_SUFFIX STL_VLARBTABLE_RECORD;
#define STL_VLARB_COMPONENTMASK_LID 		0x0000000000000001ul
#define STL_VLARB_COMPONENTMASK_OUTPORTNUM 	0x0000000000000002ul
#define STL_VLARB_COMPONENTMASK_BLOCKNUM 	0x0000000000000004ul


/*
 * MCMemberRecord
 * 
 * STL Differences:
 *		MLID moved to new location for flit alignment.
 *		SL Lengthened to 5 bits.
 *		FlowLabel removed.
 *		Record is now 16 bits longer.
 */
typedef struct {
	struct {
		IB_GID MGID;
		/* 16 bytes */

		IB_GID PortGID;

		/* 32 bytes */

	} PACK_SUFFIX RID;

	uint32 Q_Key;

	uint16 Reserved; // Used to be MLID.

	IB_BITFIELD2(uint8,
			MtuSelector:2,
			Mtu:6);
	uint8 TClass;

	/* 40 bytes */
	uint16 P_Key;

	IB_BITFIELD2(uint8,
			RateSelector:2,
			Rate:6);

	IB_BITFIELD2(uint8,
			PktLifeTimeSelector:2,
			PktLifeTime:6);

	IB_BITFIELD3(uint32,
			SL:5,
			Reserved5:19,
			HopLimit:8);

	/* 48 bytes */
	IB_BITFIELD5(uint8,
			Scope:4,
			Reserved4:1,
			JoinSendOnlyMember:1, // NOTE: Treat these 4 JoinStates as 1 attr.
			JoinNonMember:1,
			JoinFullMember:1);

	IB_BITFIELD2(uint8,
			ProxyJoin:1,
			Reserved2:7);

	uint16 Reserved3; 

	STL_LID MLID; // Moved for alignment.

	/* 56 bytes */
} PACK_SUFFIX STL_MCMEMBER_RECORD;
#define STL_MCMEMBER_COMPONENTMASK_MGID		0x0000000000000001ull
#define STL_MCMEMBER_COMPONENTMASK_PORTGID 	0x0000000000000002ull
#define STL_MCMEMBER_COMPONENTMASK_QKEY		0x0000000000000004ull
/* Reserved									0x0000000000000008ull */
#define STL_MCMEMBER_COMPONENTMASK_MTU_SEL 	0x0000000000000010ull
#define STL_MCMEMBER_COMPONENTMASK_MTU 		0x0000000000000020ull
#define STL_MCMEMBER_COMPONENTMASK_OK_MTU	( STL_MCMEMBER_COMPONENTMASK_MTU_SEL \
						| STL_MCMEMBER_COMPONENTMASK_MTU )
#define STL_MCMEMBER_COMPONENTMASK_TCLASS	0x0000000000000040ull
#define STL_MCMEMBER_COMPONENTMASK_PKEY 	0x0000000000000080ull
#define STL_MCMEMBER_COMPONENTMASK_RATE_SEL 0x0000000000000100ull
#define STL_MCMEMBER_COMPONENTMASK_RATE 	0x0000000000000200ull
#define STL_MCMEMBER_COMPONENTMASK_OK_RATE 	( STL_MCMEMBER_COMPONENTMASK_RATE \
						| STL_MCMEMBER_COMPONENTMASK_RATE_SEL )
#define STL_MCMEMBER_COMPONENTMASK_LIFE_SEL	0x0000000000000400ull
#define STL_MCMEMBER_COMPONENTMASK_LIFE		0x0000000000000800ull
#define STL_MCMEMBER_COMPONENTMASK_OK_LIFE	( STL_MCMEMBER_COMPONENTMASK_LIFE_SEL \
						| STL_MCMEMBER_COMPONENTMASK_LIFE )
#define STL_MCMEMBER_COMPONENTMASK_SL		0x0000000000001000ull
/* Reserved									0x0000000000002000ull */
#define STL_MCMEMBER_COMPONENTMASK_HOP		0x0000000000004000ull
#define STL_MCMEMBER_COMPONENTMASK_SCOPE	0x0000000000008000ull
/* Reserved									0x0000000000010000ull */
#define STL_MCMEMBER_COMPONENTMASK_JNSTATE	0x0000000000020000ull
#define STL_MCMEMBER_COMPONENTMASK_OK_JOIN	( STL_MCMEMBER_COMPONENTMASK_MGID \
						| STL_MCMEMBER_COMPONENTMASK_JNSTATE \
						| STL_MCMEMBER_COMPONENTMASK_PORTGID )
#define STL_MCMEMBER_COMPONENTMASK_PROXYJN	0x0000000000040000ull
#define STL_MCMEMBER_COMPONENTMASK_MLID		0x0000000000080000ull

#define STL_MCMEMBER_COMPONENTMASK_OK_CREATE	( STL_MCMEMBER_COMPONENTMASK_PKEY \
						| STL_MCMEMBER_COMPONENTMASK_QKEY \
						| STL_MCMEMBER_COMPONENTMASK_SL \
						| STL_MCMEMBER_COMPONENTMASK_TCLASS \
						| STL_MCMEMBER_COMPONENTMASK_JNSTATE \
						| STL_MCMEMBER_COMPONENTMASK_PORTGID )

#define STL_MCMRECORD_GETJOINSTATE(REC)		( (REC)->JoinSendOnlyMember \
						| (REC)->JoinNonMember \
						| (REC)->JoinFullMember )

/*
 * SMInfoRecord
 *
 * STL Differences:
 * 		LID extended to 32 bits.
 * 		Added Reserved to ensure word-alignment of SMInfo.
 * 		Added Reserved2 to ensure word-alignment of GetTable() responses.
 */
typedef struct {
	struct {
		STL_LID	LID;
	} PACK_SUFFIX RID;
	
	uint32		Reserved;
	
	STL_SM_INFO		SMInfo;
	
} PACK_SUFFIX STL_SMINFO_RECORD;

#define STL_SMINFO_RECORD_COMP_LID 					0x0000000000000001ull
/* Reserved											0x0000000000000002ull */
#define STL_SMINFO_RECORD_COMP_GUID					0x0000000000000004ull
#define STL_SMINFO_RECORD_COMP_SMKEY				0x0000000000000008ull
#define STL_SMINFO_RECORD_COMP_ACTCOUNT 			0x0000000000000010ull
#define STL_SMINFO_RECORD_COMP_ETIME				0x0000000000000020ull
#define STL_SMINFO_RECORD_COMP_PRIORITY				0x0000000000000040ull
#define STL_SMINFO_RECORD_COMP_SMSTATEELEV			0x0000000000000080ull
#define STL_SMINFO_RECORD_COMP_SMSTATEINIT			0x0000000000000100ull
#define STL_SMINFO_RECORD_COMP_SMSTATECURR			0x0000000000000200ull

/*
 * InformInfoRecord
 *
 * Length is TBD.
 *
 * STL Differences:
 *		Replaced SubscriberGID with SubscriberLID.
 *		Reserved adjusted to preserve alignment.
 *
 */
typedef struct {
	struct {
		STL_LID	SubscriberLID;	
		uint16	Enum;
	} PACK_SUFFIX RID;
	
	uint16		Reserved;

	STL_INFORM_INFO InformInfoData;	
} PACK_SUFFIX STL_INFORM_INFO_RECORD;

#define STL_INFORM_INFO_REC_COMP_SUBSCRIBER_LID		0x0000000000000001ll
#define STL_INFORM_INFO_REC_COMP_SUBSCRIBER_ENUM	0x0000000000000002ll
/* reserved											0x0000000000000004ll */
#define STL_INFORM_INFO_REC_COMP_GID				0x0000000000000008ll
#define STL_INFORM_INFO_REC_COMP_LID_RANGE_BEGIN	0x0000000000000010ll
#define STL_INFORM_INFO_REC_COMP_LID_RANGE_END		0x0000000000000020ll
#define STL_INFORM_INFO_REC_COMP_LID_RANGE_GENERIC	0x0000000000000040ll
#define STL_INFORM_INFO_REC_COMP_LID_RANGE_SUBSCRIBE 0x0000000000000080ll
#define STL_INFORM_INFO_REC_COMP_LID_RANGE_TYPE		0x0000000000000100ll
/* reserved											0x0000000000000200ll */
#define STL_INFORM_INFO_REC_COMP_LID_TRAP_NUMBER	0x0000000000000400ll
#define STL_INFORM_INFO_REC_COMP_LID_QPN			0x0000000000000800ll
/* reserved											0x0000000000001000ll */
#define STL_INFORM_INFO_REC_COMP_LID_RESP_TIME		0x0000000000002000ll
/* reserved											0x0000000000004000ll */
#define STL_INFORM_INFO_REC_COMP_LID_PRODUCER_TYPE	0x0000000000008000ll

/*
 * LinkRecord
 *
 * STL Differences:
 *		LIDs lengthened
 *		Reserved field added to preserve alignment.
 *		RID.FromPort and ToPort for HFI will return HFI port number
 *		LinkCondition field added to describe which link errors apply to the link when asking for this data.
 *		ErrorMask field added to provide detailed information on the particular errors that led to the LinkCondition error.
 */
typedef struct _STL_LINK_RECORD {
	struct {
		STL_LID	FromLID;		
		uint8	FromPort;		/* for switch or HFI: port number */
	} PACK_SUFFIX RID;
	
	uint8		ToPort;			/* for switch or HFI: port number */

	uint16		Reserved;
	
	STL_LID		ToLID;	
} PACK_SUFFIX STL_LINK_RECORD;

#define STL_LINK_REC_COMP_FROM_LID				0x0000000000000001ll
#define STL_LINK_REC_COMP_FROM_PORT				0x0000000000000002ll
#define STL_LINK_REC_COMP_TO_PORT				0x0000000000000004ll
#define STL_LINK_REC_LINK_CONDITION				0x0000000000000008ll
#define STL_LINK_REC_COMP_TO_LID				0x0000000000000010ll
#define STL_LINK_REC_COMP_ERROR_MASK			0x0000000000000020ll

/* These constants are used to describe the applicable link condition(s) */
#define STL_LINK_REC_SLOWLINKS					0x0000000000000001ll
#define STL_LINK_REC_MISCONFIGLINKS				0x0000000000000002ll
#define STL_LINK_REC_MISCONNLINKS				0x0000000000000004ll

/*
 * ServiceRecord
 *
 * STL Differences
 *		Added of ServiceLID
 *		Moved Reserved field to maintain alignment.
 */
typedef struct {
	struct {
		uint64	ServiceID;
	
		STL_LID	ServiceLID;
		uint16	ServiceP_Key;
		uint16	Reserved;

		IB_GID	ServiceGID;

	} PACK_SUFFIX RID;
	
	uint32		ServiceLease;	
							
	uint32		Reserved;
	
	uint8		ServiceKey[16];		
	
	uint8		ServiceName[64];	
	
	uint8		ServiceData8[16];

	uint16		ServiceData16[8];
	
	uint32		ServiceData32[4];
	
	uint64		ServiceData64[2];
	
} PACK_SUFFIX STL_SERVICE_RECORD;

#define STL_SERVICE_RECORD_COMP_SERVICEID			0x0000000000000001ull
#define STL_SERVICE_RECORD_COMP_SERVICELID			0x0000000000000002ull
#define STL_SERVICE_RECORD_COMP_SERVICEPKEY			0x0000000000000004ull
/* Reserved											0x0000000000000008ull */
#define STL_SERVICE_RECORD_COMP_SERVICEGID			0x0000000000000010ull
#define STL_SERVICE_RECORD_COMP_SERVICELEASE		0x0000000000000020ull
/* Reserved											0x0000000000000040ull */
#define STL_SERVICE_RECORD_COMP_SERVICEKEY			0x0000000000000080ull
#define STL_SERVICE_RECORD_COMP_SERVICENAME			0x0000000000000100ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_1			0x0000000000000200ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_2			0x0000000000000400ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_3			0x0000000000000800ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_4			0x0000000000001000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_5			0x0000000000002000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_6			0x0000000000004000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_7			0x0000000000008000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_8			0x0000000000010000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_9			0x0000000000020000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_10		0x0000000000040000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_11		0x0000000000080000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_12		0x0000000000100000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_13		0x0000000000200000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_14		0x0000000000400000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_15		0x0000000000800000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_8_16		0x0000000001000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_16_1		0x0000000002000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_16_2		0x0000000004000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_16_3		0x0000000008000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_16_4		0x0000000010000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_16_5		0x0000000020000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_16_6		0x0000000040000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_16_7		0x0000000080000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_16_8		0x0000000100000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_32_1		0x0000000200000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_32_2		0x0000000400000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_32_3		0x0000000800000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_32_4		0x0000001000000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_64_1		0x0000002000000000ull
#define STL_SERVICE_RECORD_COMP_SERVICE_64_2		0x0000004000000000ull
 
/*
 * ServiceAssociationRecord
 *
 * NOTE: No changes from IB version.
 */
typedef IB_SERVICEASSOCIATION_RECORD STL_SERVICEASSOCIATION_RECORD;
 
/*
 * TraceRecord
 *
 * STL Differences
 *
 *		GIDPrefix deleted.
 *		EntryPort, ExitPort moved for alignment.
 * 		Reserved2 added to word/qword-align NodeID.
 */
typedef struct {
	uint16		IDGeneration;
	uint8		Reserved;
	uint8		NodeType;
	uint8		EntryPort;
	uint8		ExitPort;
	uint16		Reserved2;
	
	uint64		NodeID;
	uint64		ChassisID;
	uint64		EntryPortID;
	uint64		ExitPortID;
	
} PACK_SUFFIX STL_TRACE_RECORD;

/*
 * MultiPathRecord
 *
 * STL Differences:
 * 		RawTraffic is now reserved.
 * 		Reserved2 moved, shortened.
 * 		SL lengthened.
 * 		L2 settings added.
 * 		Reserved3 added to ensure alignment of gids.
 */
typedef struct {
	IB_BITFIELD3(uint32,
				Reserved1:4,
				FlowLabel:20,
				HopLimit:8);		

	uint8		TClass;				

	IB_BITFIELD2(uint8,
				Reversible:1,
				NumbPath:7);

	uint16		P_Key;
	
	IB_BITFIELD4(uint16,
				QoSType:2,
				QoSPriority:8,
				Reserved2:1,
				SL:5);				

	IB_BITFIELD2(uint8,
				MtuSelector:2,
				Mtu:6);				

	IB_BITFIELD2(uint8,
				RateSelector:2,
				Rate:6);

	IB_BITFIELD2(uint8,
				PktLifeTimeSelector:2,
				PktLifeTime:6);
	
	IB_BITFIELD3(uint8,
				IndependenceSelector:2,
				SGIDScope:3,
				DGIDScope:3);
	
	uint8		SGIDCount;

	uint8		DGIDCount;

	uint64		ServiceID;	
	
	IB_BITFIELD5(uint8,
				L2_8B:1,			// True if path supports this L2.
				L2_10B:1,			// True if path supports this L2.
				L2_9B:1,			// True if path supports this L2.
				L2_16B:1,			// True if path supports this L2.
				Reserved:4);		

	uint8		Reserved3[7];				
				
	IB_GID		GIDList[0];		// SGIDCount + DGIDCount entries.
} PACK_SUFFIX STL_MULTIPATH_RECORD_GID;

/*
 * MultiPathRecord (GUID)
 *
 * STL Differences:
 *		New for STL
 */
typedef struct {
	IB_BITFIELD3(uint32,
				Reserved1:4,
				FlowLabel:20,
				HopLimit:8);		

	uint8		TClass;				

	IB_BITFIELD2(uint8,
				Reversible:1,
				NumbPath:7);

	uint16		P_Key;
	
	IB_BITFIELD4(uint16,
				QoSType:2,
				QoSPriority:8,
				Reserved2:1,
				SL:5);				

	IB_BITFIELD2(uint8,
				MtuSelector:2,
				Mtu:6);				

	IB_BITFIELD2(uint8,
				RateSelector:2,
				Rate:6);

	IB_BITFIELD2(uint8,
				PktLifeTimeSelector:2,
				PktLifeTime:6);
	
	IB_BITFIELD3(uint8,
				IndependenceSelector:2,
				SGUIDScope:3,
				DGUIDScope:3);
	
	uint16		SGUIDCount;

	uint16		DGUIDCount;

	IB_BITFIELD5(uint8,
				L2_8B:1,			// True if path supports this L2.
				L2_10B:1,			// True if path supports this L2.
				L2_9B:1,			// True if path supports this L2.
				L2_16B:1,			// True if path supports this L2.
				Reserved:4);		

	uint8		Reserved3[5];				
				
	uint64		ServiceID;	
	
	uint64		SubnetPrefix;

	uint64		GUIDList[0];		// SGUIDCount + DGUIDCount entries.
} PACK_SUFFIX STL_MULTIPATH_RECORD_GUID;

/*
 * MultiPathRecord (LID)
 *
 * STL Differences:
 *		New for STL
 */
typedef struct {
	IB_BITFIELD3(uint32,
				Reserved1:4,
				FlowLabel:20,
				HopLimit:8);		

	uint8		TClass;				

	IB_BITFIELD2(uint8,
				Reversible:1,
				NumbPath:7);

	uint16		P_Key;
	
	IB_BITFIELD4(uint16,
				QoSType:2,
				QoSPriority:8,
				Reserved2:1,
				SL:5);				

	IB_BITFIELD2(uint8,
				MtuSelector:2,
				Mtu:6);				

	IB_BITFIELD2(uint8,
				RateSelector:2,
				Rate:6);

	IB_BITFIELD2(uint8,
				PktLifeTimeSelector:2,
				PktLifeTime:6);
	
	IB_BITFIELD3(uint8,
				IndependenceSelector:2,
				SGUIDScope:3,
				DGUIDScope:3);
	
	uint16		SLIDCount;

	uint16		DLIDCount;

	IB_BITFIELD5(uint8,
				L2_8B:1,			// True if path supports this L2.
				L2_10B:1,			// True if path supports this L2.
				L2_9B:1,			// True if path supports this L2.
				L2_16B:1,			// True if path supports this L2.
				Reserved:4);		

	uint8		Reserved3[5];				
				
	uint64		ServiceID;	
	
	STL_LID		LIDList[0];			// SLIDCount + DLIDCount entries
} PACK_SUFFIX STL_MULTIPATH_RECORD_LID;

/*
 * CableInfoRecord
 *
 * STL Differences:
 * 		LID lengthened to 32 bits.
 * 		Reserved2 field shortened from 20 bits to 4 to preserve word-alignment.
 * 		RID.Port for HFI will return HFI port number
 */
#define STL_CIR_DATA_SIZE		64
typedef struct {
	struct {
		STL_LID	LID;
		uint8	Port;				/* for switch or HFI: port numnber */
		IB_BITFIELD2(uint8,
				Length:7,
				Reserved:1);
		STL_FIELDUNION2(u1, 16,
				 Address:12,
				 PortType:4); /* Port type for response only */
	};
	
	uint8		Data[STL_CIR_DATA_SIZE];

} PACK_SUFFIX STL_CABLE_INFO_RECORD;

#define STL_CIR_COMP_LID 	0x1ul
#define STL_CIR_COMP_PORT	0x2ul
#define STL_CIR_COMP_LEN	0x4ul
//Reserved					0x8ul
#define STL_CIR_COMP_ADDR	0x10ul
//Reserved2					0x20ul

#define STL_VFABRIC_NAME_LEN 64
/*
 * VFInfoRecord
 * 
 * STL Differences:
 *		Was a Vendor-specific MAD in IB
 *		SL lengthened to 5 bits.
 *		Reserved fields renamed.
 *
 *	TBD - adjust to report multiple SLs (perhaps a mask) for TrafficClass 
 *	concept in prep for STL2
 */
typedef struct {
	uint16		vfIndex;		/* The index assigned to the VF */
	uint16		pKey;			/* PKey associated with the VF */
	uint32		rsvd1;
	
	/* 8 bytes */
	
	uint8		vfName[STL_VFABRIC_NAME_LEN]; /* The name of the VF. Must be \0 terminated. */
	
	/* 72 bytes */
	
	uint64		ServiceID;		/* for query only */
	
	/* 80 bytes */
	
	IB_GID		MGID;			/* for query only */
	
	/* 96 bytes */
	
	struct {
		IB_BITFIELD3(uint8,
				selectFlags:2, 	/* 1 bit to indicate SL in queries, 1 bit for pkey */
				rsvd2:1,
				slBase:5);		/* service level - 5 bits */
		IB_BITFIELD3(uint8,
				mtuSpecified:1,	/* mtu specified for VF - 1 bit */
				rsvd3:1,    
				mtu:6);			/* max mtu assigned to VF - 6 bits */

		IB_BITFIELD3(uint8,
				rateSpecified:1, /* rate specified for VF - 1 bit */
				rsvd4:1,
				rate:6);		/* max rate assigned to VF - 6 bits */

		IB_BITFIELD3(uint8,
				pktLifeSpecified:1, /* pkt life time specified for VF - 1 bit */
				rsvd5:4,
				pktLifeTimeInc:3); /* pkt life time assigned to VF - 3 bits */
	} PACK_SUFFIX s1;

	uint8		optionFlags;		/* security bit, QoS bit, reliable flow disable, 5 reserved */
	uint8		bandwidthPercent;	/* bandwidth percentage, 8 bits */

	IB_BITFIELD2(uint8,
				rsvd6:7,
				priority:1);		/* priority, 1 bit */

	uint8		routingSLs; /* Always 1 */

	IB_BITFIELD2(uint8,
				rsvd7:1,
				preemptionRank:7);
	
	IB_BITFIELD2(uint8,
				rsvd8:3,
				hoqLife:5);

	IB_BITFIELD3(uint8,
				slResponseSpecified:1, /* slResponse field is set - 1 bit */
				rsvd9:2,
				slResponse:5); /* service level - 5 bits */

	IB_BITFIELD3(uint8,
				slMulticastSpecified:1, /* slMulticast field is set - 1 bit */
				rsvd10:2,
				slMulticast:5); /* service level - 5 bits */

	uint8		rsvd11[20];

} PACK_SUFFIX STL_VFINFO_RECORD;

#define STL_VFINFO_REC_COMP_INDEX				0x0000000000000001ll
#define STL_VFINFO_REC_COMP_PKEY				0x0000000000000002ll
#define STL_VFINFO_REC_COMP_NAME				0x0000000000000008ll
#define STL_VFINFO_REC_COMP_SERVICEID			0x0000000000000010ll
#define STL_VFINFO_REC_COMP_MGID				0x0000000000000020ll
#define STL_VFINFO_REC_COMP_SL					0x0000000000000080ll

/* selectFlags */
#define STL_VFINFO_REC_SEL_PKEY_QUERY			0x01
#define STL_VFINFO_REC_SEL_SL_QUERY				0x02

/* optionFlags */
#define STL_VFINFO_REC_OPT_SECURITY				0x01
#define STL_VFINFO_REC_OPT_QOS					0x02
#define STL_VFINFO_REC_OPT_FLOW_DISABLE			0x04

/*
 * QuarantinedNodeRecord
 *
 * STL Differences:
 *		New for STL
 *
 * NOTE: GET TABLE is the only supported method for this query.
 * Component Mask must be zero.
 */

#define STL_QUARANTINE_REASON_SPOOF_GENERIC 		0x00000001
#define STL_QUARANTINE_REASON_TOPO_NODE_GUID 		0x00000002
#define STL_QUARANTINE_REASON_TOPO_NODE_DESC 		0x00000004
#define STL_QUARANTINE_REASON_TOPO_PORT_GUID 		0x00000008
#define STL_QUARANTINE_REASON_TOPO_UNDEFINED_LINK 	0x00000010
#define STL_QUARANTINE_REASON_VL_COUNT 				0x00000020
#define STL_QUARANTINE_REASON_SMALL_MTU_SIZE 		0x00000040
#define STL_QUARANTINE_REASON_BAD_PACKET_FORMATS    0x00000080
#define STL_QUARANTINE_REASON_MAXLID				0x00000100
#define STL_QUARANTINE_REASON_UNKNOWN 				0x00000000

typedef struct {
	STL_NODE_DESCRIPTION nodeDesc;
	uint64 nodeGUID;
	uint64 portGUID;
} PACK_SUFFIX STL_EXPECTED_NODE_INFO;

typedef struct {
	STL_LID trustedLid;
	uint8 trustedPortNum;
	uint8 Reserved[3];
	uint64 trustedNodeGUID;
	uint64 trustedNeighborNodeGUID;

	STL_NODE_DESCRIPTION NodeDesc;
	STL_NODE_INFO NodeInfo;

	uint32 quarantineReasons;
	// expectedNodeInfo only valid if quarantineReasons != 0
	STL_EXPECTED_NODE_INFO expectedNodeInfo;

} PACK_SUFFIX STL_QUARANTINED_NODE_RECORD;

/*
 * Congestion Info Record
 *
 * STL Differences:
 * 		Was Vendor Specific in IB.
 */
typedef struct {
	STL_LID				LID;
	uint32				reserved;				
	STL_CONGESTION_INFO CongestionInfo;
} PACK_SUFFIX STL_CONGESTION_INFO_RECORD;

#define CIR_COMPONENTMASK_COMP_LID	    0x00000001

/*
 * Switch Congestion Setting Record
 *
 * STL Differences:
 * 		Was Vendor Specific in IB.
 */
typedef struct {	/* all fields are RW */
	STL_LID				LID;
	uint32				reserved;				
	STL_SWITCH_CONGESTION_SETTING SwitchCongestionSetting;
} PACK_SUFFIX STL_SWITCH_CONGESTION_SETTING_RECORD;

#define SWCSR_COMPONENTMASK_COMP_LID	    0x00000001

/*
 * Switch Port Congestion Setting Record
 *
 * STL Differences:
 * 		Was Vendor Specific in IB.
 */
typedef struct {
	struct {
		STL_LID			LID;
		uint8			Port;
	} PACK_SUFFIX RID;
	
	uint8				Reserved[3];
	
	/*	8 bytes */
	
	STL_SWITCH_PORT_CONGESTION_SETTING SwitchPortCongestionSetting;

} PACK_SUFFIX STL_SWITCH_PORT_CONGESTION_SETTING_RECORD;

#define SWPCSR_COMPONENTMASK_COMP_LID	    0x00000001
#define SWPCSR_COMPONENTMASK_COMP_PORT	    0x00000002

/*
 * HFI Congestion Setting Record
 *
 * STL Differences:
 * 		Was Vendor Specific in IB.
 */
typedef struct {	/* all fields are RW */
	STL_LID				LID;	
	uint32				reserved;			
	STL_HFI_CONGESTION_SETTING HFICongestionSetting;
} PACK_SUFFIX STL_HFI_CONGESTION_SETTING_RECORD;

#define HCSR_COMPONENTMASK_COMP_LID	    0x00000001

/*
 * HFI Congestion Control Table Record
 *
 * STL Differences:
 * 		Was Vendor Specific in IB.
 */
typedef struct {	/* all fields are RW */
	struct {
		STL_LID			LID;
		uint16			BlockNum;
	} PACK_SUFFIX RID;
	uint16				reserved;				
	STL_HFI_CONGESTION_CONTROL_TABLE HFICongestionControlTable;
} PACK_SUFFIX STL_HFI_CONGESTION_CONTROL_TABLE_RECORD;
#define HCCTR_COMPONENTMASK_COMP_LID	    0x00000001
#define HCCTR_COMPONENTMASK_COMP_BLOCK	    0x00000002

/*
 * Buffer Control Table Record
 *
 * STL Differences:
 * 		New for STL
 * 		At this time, Component Mask is only supported for LID and Port.
 */
typedef struct {
	struct {
		STL_LID			LID;
		uint8			Port;			/* for switch or HFI: port numnber */
	} PACK_SUFFIX RID;

	uint8				Reserved[3];

	STL_BUFFER_CONTROL_TABLE BufferControlTable;

} PACK_SUFFIX STL_BUFFER_CONTROL_TABLE_RECORD;

#define BFCTRL_COMPONENTMASK_COMP_LID	0x00000001
#define BFCTRL_COMPONENTMASK_COMP_PORT	0x00000002

/*
 * FabricInfo Record
 *
 * STL Differences:
 * 		New for STL
 * 		supports only Get, Component Mask N/A
 */
typedef struct {
	uint32	NumHFIs;				/* HFI Nodes */
	uint32	NumSwitches;			/* Switch Nodes (ASICs) */
		/* Internal = in same SystemImageGuid */
		/* HFI = HFI to switch and HFI to HFI links */
		/* ISL = switch to switch links */
		/* links which are Omitted will not be considered for Degraded checks */
		/* switch port 0 is not counted as a link */
	uint32	NumInternalHFILinks;	/* HFI to switch (or HFI) links */
	uint32	NumExternalHFILinks;	/* HFI to switch (or HFI) links */
	uint32	NumInternalISLs;		/* switch to switch links */
	uint32	NumExternalISLs;		/* switch to switch links */
	uint32	NumDegradedHFILinks;	/* links with one or both sides below best enabled */
	uint32	NumDegradedISLs;		/* links with one or both sides below best enabled */
	uint32	NumOmittedHFILinks;		/* links quarantined or left in Init */
	uint32	NumOmittedISLs;			/* links quarantined or left in Init */
	uint32	rsvd5[92];
} PACK_SUFFIX STL_FABRICINFO_RECORD;


#define MAX_DG_NAME 64

/*
 * DeviceGroupNameRecord
 */ 
typedef struct {
	uint8 	DeviceGroupName[MAX_DG_NAME];		/* Must be \0 terminated. */
} PACK_SUFFIX STL_DEVICE_GROUP_NAME_RECORD;


#define STL_DEVICE_GROUP_COMPONENTMASK_LID		0x0000000000000001ull
#define STL_DEVICE_GROUP_COMPONENTMASK_PORT		0x0000000000000002ull
/* reserved field								0x0000000000000004ull */
#define STL_DEVICE_GROUP_COMPONENTMASK_DGNAME	0x0000000000000008ull
#define STL_DEVICE_GROUP_COMPONENTMASK_GUID		0x0000000000000010ull
#define STL_DEVICE_GROUP_COMPONENTMASK_NODEDESC	0x0000000000000020ull

/*
 * DeviceGroupMemberRecord
 */
typedef struct {
	STL_LID 				LID;
	uint8					Port;
	uint8					Reserved1[3];
	uint8					DeviceGroupName[MAX_DG_NAME];	/* Must be \0 terminated. */
	uint64					GUID;
	STL_NODE_DESCRIPTION	NodeDescription;
} PACK_SUFFIX STL_DEVICE_GROUP_MEMBER_RECORD;

#define STL_DEVICE_TREE_COMPONENTMASK_LID		0x0000000000000001ull
#define STL_DEVICE_TREE_COMPONENTMASK_NUMPORTS		0x0000000000000002ull
#define STL_DEVICE_TREE_COMPONENTMASK_NODETYPE		0x0000000000000004ull
/* reserved field					0x0000000000000008ull */
#define STL_DEVICE_TREE_COMPONENTMASK_PORTSTATEMASKACT	0x0000000000000010ull
#define STL_DEVICE_TREE_COMPONENTMASK_PORTSTATEMASKETH	0x0000000000000020ull
#define STL_DEVICE_TREE_COMPONENTMASK_GUID		0x0000000000000040ull
#define STL_DEVICE_TREE_COMPONENTMASK_SYSIMAGEGUID	0x0000000000000080ull
#define STL_DEVICE_TREE_COMPONENTMASK_NODEDESC		0x0000000000000100ull
/* reserved field					0x0000000000000200ull */

/*
 * DeviceTreeMemberRecord
 */
typedef struct {
	STL_LID					LID;
	uint8					NumPorts;
	uint8					NodeType;
	uint8					Reserved1[2];
	STL_PORTMASK			portMaskAct[STL_MAX_PORTMASK];

	STL_PORTMASK			portMaskReserved[STL_MAX_PORTMASK];

	STL_PORTMASK			portMaskPortLinkMode[STL_MAX_PORTMASK];

	uint64					GUID;
	uint64					SystemImageGUID;
	STL_NODE_DESCRIPTION	NodeDescription;
	uint64					Reserved2[4];

} PACK_SUFFIX STL_DEVICE_TREE_MEMBER_RECORD;


/*
 * SwitchCost Record
 */
#define STL_SWITCH_COST_NUM_ENTRIES 64
typedef struct _STL_SWITCH_COST {
	STL_LID DLID;
	uint16 value;
	uint16 Reserved;
} PACK_SUFFIX STL_SWITCH_COST;

typedef struct _STL_SWITCH_COST_RECORD {
	STL_LID SLID;
	STL_SWITCH_COST Cost[STL_SWITCH_COST_NUM_ENTRIES];
} PACK_SUFFIX STL_SWITCH_COST_RECORD;

#define STL_SWITCH_COST_REC_COMP_SLID 0x0000000000000001ull

#if defined (__cplusplus)
}
#endif
#endif // __STL_SA_H__
