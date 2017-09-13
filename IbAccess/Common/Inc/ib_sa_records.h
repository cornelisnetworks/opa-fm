/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#if defined(CHECK_HEADERS)

#ifndef __STL_SA_H__
#warning FIX ME!!! Your includes should use the stl_sa.h header and not the ib_sa_records.h header for STL builds
#endif

#endif

#ifndef _IBA_IB_SA_RECORDS_H_
#define _IBA_IB_SA_RECORDS_H_

/* IB Subnet Adminstration records and methods */

#include "iba/public/datatypes.h"				/* Portable datatypes */
#include "iba/stl_types.h"				/* IBA specific datatypes */
#include "iba/stl_sm.h"
#include "iba/stl_helper.h"
#ifndef IB_STACK_OPENIB
#include "iba/vpi.h"
#endif
#include "iba/public/ibyteswap.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "iba/public/ipackon.h"

/* Subnet Administration methods */
#define	SUBN_ADM_GET					0x01
#define	SUBN_ADM_GET_RESP				0x81
#define	SUBN_ADM_SET					0x02
#define	SUBN_ADM_REPORT					0x06
#define	SUBN_ADM_REPORT_RESP			0x86
#define	SUBN_ADM_GETTABLE				0x12
#define	SUBN_ADM_GETTABLE_RESP			0x92
#define	SUBN_ADM_GETTRACETABLE			0x13	/* optional */
#define	SUBN_ADM_GETMULTI				0x14	/* optional */
#define	SUBN_ADM_GETMULTI_RESP			0x94	/* optional */
#define	SUBN_ADM_DELETE					0x15
#define	SUBN_ADM_DELETE_RESP			0x95

/* Subnet Administration Attribute IDs */
#define	SA_ATTRIB_CLASS_PORT_INFO				0x0001
#define	SA_ATTRIB_NOTICE						0x0002
#define	SA_ATTRIB_INFORM_INFO					0x0003
#define	SA_ATTRIB_NODE_RECORD					0x0011
#define SA_ATTRIB_PORTINFO_RECORD				0x0012
#define	SA_ATTRIB_SWITCHINFO_RECORD				0x0014
#define	SA_ATTRIB_LINEAR_FWDTBL_RECORD			0x0015
#define	SA_ATTRIB_RANDOM_FWDTBL_RECORD			0x0016
#define	SA_ATTRIB_MCAST_FWDTBL_RECORD			0x0017
#define	SA_ATTRIB_SMINFO_RECORD					0x0018
#define	SA_ATTRIB_INFORM_INFO_RECORD			0x00F3
#define	SA_ATTRIB_LINK_RECORD					0x0020
// was	SA_ATTRIB_GUIDINFO_RECORD				0x0030
#define	SA_ATTRIB_SERVICE_RECORD				0x0031
#define	SA_ATTRIB_P_KEY_TABLE_RECORD			0x0033
#define	SA_ATTRIB_PATH_RECORD					0x0035
#define	SA_ATTRIB_VLARBTABLE_RECORD				0x0036
#define	SA_ATTRIB_MCMEMBER_RECORD				0x0038
#define	SA_ATTRIB_TRACE_RECORD					0x0039
#define	SA_ATTRIB_MULTIPATH_RECORD				0x003A
#define	SA_ATTRIB_SERVICEASSOCIATION_RECORD		0x003B

#define SA_ATTRIB_VFABRIC_RECORD				0xff02
#define	SA_ATTRIB_JOB_ROUTE_RECORD				0xFFB2  // Reserved AID used by opasadb_route

#define SA_ATTRIB_CG_RECORD        				0xff40  // Vendor unique collective group record
#define SA_ATTRIB_CG_STATUS_RECORD 				0xff41  // Vendor unique collective group status record
#define SA_ATTRIB_CFT_RECORD       				0xff42  // Vendor unique CFT record

/* SA ClassSpecific MAD_STATUS (AsReg16) */
#define MAD_STATUS_SA_NO_RESOURCES				0x0100
#define MAD_STATUS_SA_REQ_INVALID				0x0200
#define MAD_STATUS_SA_NO_RECORDS				0x0300
#define MAD_STATUS_SA_TOO_MANY_RECORDS			0x0400
#define MAD_STATUS_SA_REQ_INVALID_GID			0x0500
#define MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS 0x0600
/* 0x0700-0xff00 reserved */

/* SA specific CapMask bits for IB_CLASS_PORT_INFO */
#define CLASS_PORT_CAPMASK_SA_OPTIONAL_RECORDS	0x00100
#define CLASS_PORT_CAPMASK_SA_UD_MULTICAST 		0x00200
#define CLASS_PORT_CAPMASK_SA_MULTIPATH 		0x00400 /*TraceRecord and MultiPathRecord */
#define CLASS_PORT_CAPMASK_SA_REINIT	 		0x00800
#define CLASS_PORT_CAPMASK2_SA_MCFDBTOP_SUPPORT	0x00008

/* ---------------------------------------------------------------------------
 * Basic SA types and constants
 */

/* selector for use in assorted queries for Mtu, Rate, Lifetime */
typedef enum {
	IB_SELECTOR_GT=0,
	IB_SELECTOR_LT=1,
	IB_SELECTOR_EQ=2,
	IB_SELECTOR_MAX=3	/* largest available for Mtu/Rate, smallest packet life */
} IB_SELECTOR;

/* --------------------------------------------------------------------------
 * SA records themselves
 */


/* --------------------------------------------------------------------------
 * Path Record - describes path between 2 end nodes in the fabric
 */

/* ComponentMask bits */
/*
 * CA13-6: The component mask bits for ServiceID8MSB and ServiceID56LSB
 * shall have the same value. They shall be either both set to one or
 * both cleared.
 */
#define IB_PATH_RECORD_COMP_SERVICEID					0x00000003
#define IB_PATH_RECORD_COMP_DGID						0x00000004
#define IB_PATH_RECORD_COMP_SGID						0x00000008
#define IB_PATH_RECORD_COMP_DLID						0x00000010
#define IB_PATH_RECORD_COMP_SLID						0x00000020
#define IB_PATH_RECORD_COMP_RAWTRAFFIC					0x00000040
	/* reserved field									0x00000080 */
#define IB_PATH_RECORD_COMP_FLOWLABEL					0x00000100
#define IB_PATH_RECORD_COMP_HOPLIMIT					0x00000200
#define IB_PATH_RECORD_COMP_TCLASS						0x00000400
#define IB_PATH_RECORD_COMP_REVERSIBLE					0x00000800
#define IB_PATH_RECORD_COMP_NUMBPATH					0x00001000
#define IB_PATH_RECORD_COMP_PKEY						0x00002000
#define IB_PATH_RECORD_COMP_QOS_CLASS					0x00004000
#define IB_PATH_RECORD_COMP_SL							0x00008000
#define IB_PATH_RECORD_COMP_MTUSELECTOR					0x00010000
#define IB_PATH_RECORD_COMP_MTU							0x00020000
#define IB_PATH_RECORD_COMP_RATESELECTOR				0x00040000
#define IB_PATH_RECORD_COMP_RATE						0x00080000
#define IB_PATH_RECORD_COMP_PKTLIFESELECTOR				0x00100000
#define IB_PATH_RECORD_COMP_PKTLIFE						0x00200000
#define IB_PATH_RECORD_COMP_PREFERENCE					0x00400000
	/* reserved field									0x00800000 */
#define IB_PATH_RECORD_COMP_ALL							0x007fff7f

typedef struct _IB_PATH_RECORD {
	uint64		ServiceID;
	IB_GID		DGID;				/* Destination GID */
	IB_GID		SGID;				/* Source GID */
	uint16		DLID;				/* Destination LID */
	uint16		SLID;				/* Source LID */
	union {
		uint32 AsReg32;
		struct {
#if CPU_BE
			uint32		RawTraffic	:1;	/* 0 for IB Packet (P_Key must be valid) */
										/* 1 for Raw Packet (No P_Key) */
			uint32		Reserved	:3;
			uint32		FlowLabel	:20;	/* used in GRH		 */
			uint32		HopLimit	:8;		/* Hop limit used in GRH */
#else
			uint32		HopLimit	:8;		/* Hop limit used in GRH */
			uint32		FlowLabel	:20;	/* used in GRH		 */
			uint32		Reserved	:3;
			uint32		RawTraffic	:1;	/* 0 for IB Packet (P_Key must be valid) */
										/* 1 for Raw Packet (No P_Key) */
#endif
		} PACK_SUFFIX s;
	} u1;

	uint8		TClass;				/* Traffic Class used in GRH */
#if CPU_BE
	uint8		Reversible	:1;
	uint8		NumbPath	:7;		/* Max number of paths to (be) return(ed) */

#else /* CPU_BE */
	uint8		NumbPath	:7;		/* Max number of paths to (be) return(ed) */
	uint8		Reversible	:1;
#endif /* CPU_BE */
	uint16		P_Key;				/* Partition Key for this path */
	union {
		uint16			AsReg16;
		struct {
#if CPU_BE
			uint16		QosType		: 2;
			uint16		Reserved2	: 2;
			uint16		QosPriority	: 8;
			uint16		SL			: 4;
#else
			uint16		SL			: 4;
			uint16		QosPriority	: 8;
			uint16		Reserved2	: 2;
			uint16		QosType		: 2;
#endif
		} PACK_SUFFIX s;
	} u2;

#if CPU_BE
	uint8		MtuSelector	:2;		/* enum IB_SELECTOR */
	uint8		Mtu			:6;		/* enum IB_MTU */
#else
	uint8		Mtu			:6;		/* enum IB_MTU */
	uint8		MtuSelector	:2;		/* enum IB_SELECTOR */
#endif

#if CPU_BE
	uint8		RateSelector:2;		/* enum IB_SELECTOR */
	uint8		Rate		:6;		/* enum IB_STATIC_RATE */
#else
	uint8		Rate		:6;		/* enum IB_STATIC_RATE */
	uint8		RateSelector:2;		/* enum IB_SELECTOR */
#endif

	/* ***************************************** */
	/* *** User be aware that the CM LifeTime & */
	/* *** TimeOut values are only 5-bit wide. */
	/* ***************************************** */
	/* */
	/* Accumulated packet life time for the path specified by an enumeration  */
	/* deried from 4.096 usec * 2^PktLifeTime */
#if CPU_BE
	uint8		PktLifeTimeSelector:2;	/* enum IB_SELECTOR */
	uint8		PktLifeTime	:6;
#else
	uint8		PktLifeTime	:6;
	uint8		PktLifeTimeSelector:2;	/* enum IB_SELECTOR */
#endif

	uint8		Preference;	/* 1.1 specific. see page 800 of volume 1 */
	uint8		Reserved2 [6];
} PACK_SUFFIX IB_PATH_RECORD;


/* --------------------------------------------------------------------------
 * Node Record - describes a port on a node in the fabric
 */

/* ComponentMask bits */
#define IB_NODE_RECORD_COMP_LID							0x00000001
	/* reserved field									0x00000002 */
	/* fields in NodeInfo */
#define IB_NODE_RECORD_COMP_BASEVERSION					0x00000004
#define IB_NODE_RECORD_COMP_CLASSVERSION				0x00000008
#define IB_NODE_RECORD_COMP_NODETYPE					0x00000010
#define IB_NODE_RECORD_COMP_NUMPORTS					0x00000020
#define IB_NODE_RECORD_COMP_SYSIMAGEGUID				0x00000040
#define IB_NODE_RECORD_COMP_NODEGUID					0x00000080
#define IB_NODE_RECORD_COMP_PORTGUID					0x00000100
#define IB_NODE_RECORD_COMP_PARTITIONCAP				0x00000200
#define IB_NODE_RECORD_COMP_DEVICEID					0x00000400
#define IB_NODE_RECORD_COMP_REVISION					0x00000800
#define IB_NODE_RECORD_COMP_LOCALPORTNUM				0x00001000
#define IB_NODE_RECORD_COMP_VENDORID					0x00002000
	/* fields in Node Description */
#define IB_NODE_RECORD_COMP_NODEDESC					0x00004000

typedef struct _IB_NODE_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		Reserved:16;
#else
			uint32		Reserved:16;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	NODE_INFO		NodeInfoData;
	NODE_DESCRIPTION NodeDescData;
} PACK_SUFFIX IB_NODE_RECORD;


/* --------------------------------------------------------------------------
 * PortInfo Record - detailed description of a port on a node in the fabric
 */

/* ComponentMask bits */
#define IB_PORTINFO_RECORD_COMP_ENDPORTLID				0x00000001
#define IB_PORTINFO_RECORD_COMP_PORTNUM					0x00000002
	/* reserved field									0x00000004 */
	/* fields in PortInfo */
#define IB_PORTINFO_RECORD_COMP_MKEY					0x00000008
#define IB_PORTINFO_RECORD_COMP_GIDPREFIX				0x00000010
#define IB_PORTINFO_RECORD_COMP_LID						0x00000020
#define IB_PORTINFO_RECORD_COMP_MASTERSMLID				0x00000040
#define IB_PORTINFO_RECORD_COMP_CAPABILITYMASK			0x00000080
#define IB_PORTINFO_RECORD_COMP_DIAGCODE				0x00000100
#define IB_PORTINFO_RECORD_COMP_MKEYLEASEPERIOD			0x00000200
#define IB_PORTINFO_RECORD_COMP_LOCALPORTNUM			0x00000400
#define IB_PORTINFO_RECORD_COMP_LINKWIDTHENABLED		0x00000800
#define IB_PORTINFO_RECORD_COMP_LINKWIDTHSUPPORTED		0x00001000
#define IB_PORTINFO_RECORD_COMP_LINKWIDTHACTIVE			0x00002000
#define IB_PORTINFO_RECORD_COMP_LINKSPEEDSUPPORTED		0x00004000
#define IB_PORTINFO_RECORD_COMP_PORTSTATE				0x00008000
#define IB_PORTINFO_RECORD_COMP_PORTPHYSSTATE	 		0x00010000
#define IB_PORTINFO_RECORD_COMP_LINKDOWNDEFAULTSTATE	0x00020000
#define IB_PORTINFO_RECORD_COMP_MKEYPROTECTBITS			0x00040000
	/* reserved field									0x00080000 */
#define IB_PORTINFO_RECORD_COMP_LMC						0x00100000
#define IB_PORTINFO_RECORD_COMP_LINKSPEEDACTIVE			0x00200000
#define IB_PORTINFO_RECORD_COMP_LINKSPEEDENABLED		0x00400000
#define IB_PORTINFO_RECORD_COMP_NEIGHBORMTU				0x00800000
#define IB_PORTINFO_RECORD_COMP_MASTERSMSL				0x01000000
#define IB_PORTINFO_RECORD_COMP_VLCAP					0x02000000

/* Option bits */
#define IB_PORTINFO_RECORD_OPTIONS_QDRNOTOVERLOADED		0x80

typedef struct _IB_PORTINFO_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		EndPortLID:16;
			uint32		PortNum:8;
			uint32		Options:8;		// Ver 1.2.1+
#else
			uint32		Options:8;
			uint32		PortNum:8;
			uint32		EndPortLID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	PORT_INFO		PortInfoData;
} PACK_SUFFIX IB_PORTINFO_RECORD;


/* --------------------------------------------------------------------------
 * Switch Info Record - information about a switch in the fabric
 */

/* ComponentMask bits */
#define IB_SWITCHINFO_RECORD_COMP_LID								0x00000001
	/* reserved field												0x00000002*/
	/* switch info fields */
#define IB_SWITCHINFO_RECORD_COMP_LINEARFDBCAP						0x00000004
#define IB_SWITCHINFO_RECORD_COMP_RANDOMFDBCAP						0x00000008
#define IB_SWITCHINFO_RECORD_COMP_MULTICASTFDBCAP					0x00000010
#define IB_SWITCHINFO_RECORD_COMP_LINEARFDBTOP						0x00000020
#define IB_SWITCHINFO_RECORD_COMP_DEFAULTPORT						0x00000040
#define IB_SWITCHINFO_RECORD_COMP_DEFAULTMULTICASTPRIMARYPORT		0x00000080
#define IB_SWITCHINFO_RECORD_COMP_DEFAULTMULTICASTNOTPRIMARYPORT	0x00000100
#define IB_SWITCHINFO_RECORD_COMP_LIFETIMEVALUE						0x00000200
#define IB_SWITCHINFO_RECORD_COMP_PORTSTATECHANGE					0x00000400
	/* reserved field												0x00000800*/
#define IB_SWITCHINFO_RECORD_COMP_LIDSPERPORT						0x00001000
#define IB_SWITCHINFO_RECORD_COMP_PARTITIONENFORCEMENTCAP			0x00002000
#define IB_SWITCHINFO_RECORD_COMP_INBOUNDENFORCEMENTCAP				0x00004000
#define IB_SWITCHINFO_RECORD_COMP_OUTBOUNDENFORCEMENTCAP			0x00008000
#define IB_SWITCHINFO_RECORD_COMP_FILTERRAWINBOUNDCAP				0x00010000
#define IB_SWITCHINFO_RECORD_COMP_FILTERRAWOUTBOUNDCAP				0x00020000
#define IB_SWITCHINFO_RECORD_COMP_ENHANCEDPORT0						0x00040000
	/* reserved field												0x00080000*/

typedef struct _IB_SWITCHINFO_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		Reserved:16;
#else
			uint32		Reserved:16;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	SWITCH_INFO		SwitchInfoData;
} PACK_SUFFIX IB_SWITCHINFO_RECORD;


/* --------------------------------------------------------------------------
 * Linear Forwarding Table Record - linear forwarding table for a switch in the fabric
 */

/* ComponentMask bits */
#define IB_LINEARFDB_RECORD_COMP_LID						0x00000001
#define IB_LINEARFDB_RECORD_COMP_BLOCKNUM					0x00000002
	/* reserved field 										0x00000004 */
	/* linear forwarding table fields */
	/* Note insufficient bits in component mask to select */
	/* all 64 entries in record */

typedef struct _IB_LINEAR_FDB_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		BlockNum:16;
#else
			uint32		BlockNum:16;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	uint32			Reserved;
	FORWARDING_TABLE	LinearFdbData;
} PACK_SUFFIX IB_LINEAR_FDB_RECORD;


/* --------------------------------------------------------------------------
 * Random Forwarding Table Record - random forwarding table for a switch in the fabric
 */

/* ComponentMask bits */
#define IB_RANDOMFDB_RECORD_COMP_LID						0x00000001
#define IB_RANDOMFDB_RECORD_COMP_BLOCKNUM					0x00000002
	/* reserved field 										0x00000004 */
	/* random forwarding table fields */
	/* Note insufficient bits in component mask to select */
	/* all 5 fields in all 16 entries in record */

typedef struct _IB_RANDOM_FDB_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		BlockNum:16;
#else
			uint32		BlockNum:16;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	uint32			Reserved;
	FORWARDING_TABLE	RandomFdbData;
} PACK_SUFFIX IB_RANDOM_FDB_RECORD;


/* --------------------------------------------------------------------------
 * Multicast Forwarding Table Record - multicast forwarding table for a switch in the fabric
 */

/* ComponentMask bits */
#define IB_MCASTFDB_RECORD_COMP_LID							0x00000001
#define IB_MCASTFDB_RECORD_COMP_POSITION					0x00000002
	/* reserved field 										0x00000004 */
#define IB_MCASTFDB_RECORD_COMP_BLOCKNUM					0x00000008
	/* reserved field 										0x00000010 */
	/* multicast forwarding table fields */
	/* limited value to select on these, so omitted defines */

typedef struct _IB_MCAST_FDB_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		Position:4;
			uint32		Reserved0:3;
			uint32		BlockNum:9;
#else
			uint32		BlockNum:9;
			uint32		Reserved0:3;
			uint32		Position:4;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	uint32			Reserved;
	FORWARDING_TABLE	MCastFdbData;
} PACK_SUFFIX IB_MCAST_FDB_RECORD;


/* --------------------------------------------------------------------------
 * VL Arbitration Table Record - VL priority controls for a port on a node in the fabric
 */

/* ComponentMask bits */
#define IB_VLARBTABLE_RECORD_COMP_LID						0x00000001
#define IB_VLARBTABLE_RECORD_COMP_OUTPUTPORTNUM				0x00000002
#define IB_VLARBTABLE_RECORD_COMP_BLOCKNUM					0x00000004
	/* reserved field 										0x00000008 */
	/* Note insufficient bits in component mask to select */
	/* all 3 fields in all 32 entries in record */

typedef struct _IB_VLARBTABLE_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		OutputPortNum:8;
			uint32		BlockNum:8;
#else
			uint32		BlockNum:8;
			uint32		OutputPortNum:8;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	uint32			Reserved;
	VLARBTABLE		VLArbData;				/* VLArbitration attribute */
} PACK_SUFFIX IB_VLARBTABLE_RECORD;


/* --------------------------------------------------------------------------
 * SM Info Record - basic information about an SM in the fabric
 */

/* ComponentMask bits */
#define IB_SMINFO_RECORD_COMP_LID						0x00000001
	/* reserved field									0x00000002 */
	/* SM Info fields */
#define IB_SMINFO_RECORD_COMP_GUID						0x00000004
#define IB_SMINFO_RECORD_COMP_SMKEY						0x00000008
#define IB_SMINFO_RECORD_COMP_ACTCOUNT					0x00000010
#define IB_SMINFO_RECORD_COMP_PRIORITY					0x00000020
#define IB_SMINFO_RECORD_COMP_SMSTATE					0x00000040

typedef struct _IB_SMINFO_RECORD {
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		Reserved:16;
#else
			uint32		Reserved:16;
			uint32		LID:16;
#endif
		} PACK_SUFFIX s;
	} RID;
	SM_INFO			SMInfoData;				/* SMInfo attribute */
} PACK_SUFFIX IB_SMINFO_RECORD;


/* --------------------------------------------------------------------------
 * P_Key Table Record - P-Key configuration for a port on a node in the fabric
 */

/* ComponentMask bits */
#define IB_PKEYTABLE_RECORD_COMP_LID						0x00000001
#define IB_PKEYTABLE_RECORD_COMP_BLOCKNUM					0x00000002
#define IB_PKEYTABLE_RECORD_COMP_PORTNUM					0x00000004
	/* reserved field										0x00000008 */
	/* P Key Table fields */
	/* limited value to select on these, so omitted defines */

typedef struct _IB_P_KEY_TABLE_RECORD {
	union {
		struct {
			uint32	AsReg32;
			uint8	Byte;
		} PACK_SUFFIX s2;
		struct {
#if CPU_BE
			uint32		LID:16;
			uint32		BlockNum:16;
#else
			uint32		BlockNum:16;
			uint32		LID:16;
#endif
			uint8		PortNum;
		} PACK_SUFFIX s;
	} PACK_SUFFIX RID;
	uint8		Reserved[3];
	PARTITION_TABLE	PKeyTblData;			/* PartitionTable for this port */
} PACK_SUFFIX IB_P_KEY_TABLE_RECORD;


/* --------------------------------------------------------------------------
 * Inform Info (event subscription) Record - SA events which have been subscribed to
 */

/* ComponentMask bits */
#define IB_INFORMINFO_RECORD_COMP_SUBSCRIBERGID				0x00000001
#define IB_INFORMINFO_RECORD_COMP_ENUM						0x00000002
	/* reserved field										0x00000004 */
	/* Inform Info fields */
#define IB_INFORMINFO_RECORD_COMP_GID						0x00000008
#define IB_INFORMINFO_RECORD_COMP_LIDRANGEBEGIN				0x00000010
#define IB_INFORMINFO_RECORD_COMP_LIDRANGEEND				0x00000020
	/* reserved field										0x00000040 */
#define IB_INFORMINFO_RECORD_COMP_ISGENERIC					0x00000080
#define IB_INFORMINFO_RECORD_COMP_SUBSCRIBE					0x00000100
#define IB_INFORMINFO_RECORD_COMP_TYPE						0x00000200
#define IB_INFORMINFO_RECORD_COMP_TRAPNUMBER_DEVICEID		0x00000400
#define IB_INFORMINFO_RECORD_COMP_QPN						0x00000800
	/* reserved field										0x00001000 */
#define IB_INFORMINFO_RECORD_COMP_RESPTIMEVALUE				0x00002000
	/* reserved field										0x00004000 */
#define IB_INFORMINFO_RECORD_COMP_PRODUCERTYPE_VENDORID		0x00008000

typedef struct _IB_INFORM_INFO_RECORD {
	struct {
		IB_GID		SubscriberGID;
		uint16		Enum;
	} PACK_SUFFIX RID;
	uint8			Reserved[6];
	IB_INFORM_INFO	InformInfoData;			/* InformInfo (event subscription) */
											/* for this port */
} PACK_SUFFIX IB_INFORM_INFO_RECORD;


/* --------------------------------------------------------------------------
 * Link Record - details about a link in the fabric
 */

/* ComponentMask bits */
#define IB_LINK_RECORD_COMP_FROMLID							0x00000001
#define IB_LINK_RECORD_COMP_FROMPORT						0x00000002
#define IB_LINK_RECORD_COMP_TOPORT							0x00000004
#define IB_LINK_RECORD_COMP_TOLID							0x00000008

typedef struct _IB_LINK_RECORD {
	struct {
		uint16			FromLID;			/* From this LID */
		uint8			FromPort;				/* From port number */
	} PACK_SUFFIX RID;
	uint8			ToPort;					/* To port number */
	uint16			ToLID;				/* To this LID */
} PACK_SUFFIX IB_LINK_RECORD;


/* --------------------------------------------------------------------------
 * Service Record - services provided by nodes in the fabric
 */

/* ComponentMask bits */
#define IB_SERVICE_RECORD_COMP_SERVICEID					0x00000001
#define IB_SERVICE_RECORD_COMP_SERVICEGID					0x00000002
#define IB_SERVICE_RECORD_COMP_SERVICEPKEY					0x00000004
	/* reserved field										0x00000008 */
#define IB_SERVICE_RECORD_COMP_SERVICELEASE					0x00000010
#define IB_SERVICE_RECORD_COMP_SERVICEKEY					0x00000020
#define IB_SERVICE_RECORD_COMP_SERVICENAME					0x00000040
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_1				0x00000080
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_2				0x00000100
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_3				0x00000200
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_4				0x00000400
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_5				0x00000800
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_6				0x00001000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_7				0x00002000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_8				0x00004000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_9				0x00008000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_10				0x00010000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_11				0x00020000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_12				0x00040000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_13				0x00080000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_14				0x00100000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_15				0x00200000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_16				0x00400000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA8_ALL				0x007fff80
#define IB_SERVICE_RECORD_COMP_SERVICEDATA16_1				0x00800000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA16_2				0x01000000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA16_3				0x02000000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA16_4				0x04000000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA16_5				0x08000000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA16_6				0x10000000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA16_7				0x20000000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA16_8				0x40000000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA16_ALL			0x7f800000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA32_1				0x80000000
#define IB_SERVICE_RECORD_COMP_SERVICEDATA32_2			   0x100000000ll
#define IB_SERVICE_RECORD_COMP_SERVICEDATA32_3			   0x200000000ll
#define IB_SERVICE_RECORD_COMP_SERVICEDATA32_4			   0x400000000ll
#define IB_SERVICE_RECORD_COMP_SERVICEDATA32_ALL		   0x780000000ll
#define IB_SERVICE_RECORD_COMP_SERVICEDATA64_1			   0x800000000ll
#define IB_SERVICE_RECORD_COMP_SERVICEDATA64_2			  0x1000000000ll
#define IB_SERVICE_RECORD_COMP_SERVICEDATA64_ALL		  0x1800000000ll

#define SERVICE_LEASE_INFINITE 0xffffffff

typedef struct _IB_SERVICE_RECORD {
	struct {
		uint64		ServiceID;
		IB_GID		ServiceGID;
		uint16		ServiceP_Key;
	} PACK_SUFFIX RID;
	uint16			Reserved;
	uint32			ServiceLease;			/* Lease period remaining for this */
											/*		service (in secconds). */
	uint8			ServiceKey[16];			/* binary service key */
	uint8			ServiceName[64];		/* UTF8 null terminated */
	uint8			ServiceData8[16];
	uint16			ServiceData16[8];
	uint32			ServiceData32[4];
	uint64			ServiceData64[2];
} PACK_SUFFIX IB_SERVICE_RECORD;


/* --------------------------------------------------------------------------
 * Service Association Record - map between service keys and names
 */

/* ComponentMask bits */
#define IB_SERVICEASSOC_RECORD_COMP_SERVICEKEY				0x00000001
#define IB_SERVICEASSOC_RECORD_COMP_SERVICENAME				0x00000002

typedef struct _IB_SERVICEASSOCIATION_RECORD {
	uint8			ServiceKey[16];			/* binary service key */
	uint8			ServiceName[16];		/* UTF8 null terminated */
} PACK_SUFFIX IB_SERVICEASSOCIATION_RECORD;


/* --------------------------------------------------------------------------
 * Multicast Member Record - members in a multicast group
 */

/* ComponentMask bits */
#define IB_MCMEMBER_RECORD_COMP_MGID						0x00000001
#define IB_MCMEMBER_RECORD_COMP_PORTGID						0x00000002
#define IB_MCMEMBER_RECORD_COMP_QKEY						0x00000004
#define IB_MCMEMBER_RECORD_COMP_MLID						0x00000008
#define IB_MCMEMBER_RECORD_COMP_MTUSELECTOR					0x00000010
#define IB_MCMEMBER_RECORD_COMP_MTU							0x00000020
#define IB_MCMEMBER_RECORD_COMP_TCLASS						0x00000040
#define IB_MCMEMBER_RECORD_COMP_PKEY						0x00000080
#define IB_MCMEMBER_RECORD_COMP_RATESELECTOR				0x00000100
#define IB_MCMEMBER_RECORD_COMP_RATE						0x00000200
#define IB_MCMEMBER_RECORD_COMP_PKTLIFESELECTOR				0x00000400
#define IB_MCMEMBER_RECORD_COMP_PKTLIFE						0x00000800
#define IB_MCMEMBER_RECORD_COMP_SL							0x00001000
#define IB_MCMEMBER_RECORD_COMP_FLOWLABEL					0x00002000
#define IB_MCMEMBER_RECORD_COMP_HOPLIMIT					0x00004000
#define IB_MCMEMBER_RECORD_COMP_SCOPE						0x00008000
#define IB_MCMEMBER_RECORD_COMP_JOINSTATE					0x00010000
#define IB_MCMEMBER_RECORD_COMP_PROXYJOIN					0x00020000
	/* reserved field										0x00040000 */

typedef struct _IB_MCMEMBER_RECORD {
	struct {
		IB_GID			MGID;
		IB_GID			PortGID;
	} PACK_SUFFIX RID;
	uint32			Q_Key;					/* Q_Key supplied at Multicast Group */
											/*	creation time by the creator. */
	uint16			MLID;					/* Multicast LID.  Assigned by SM/SA */
											/* at creation time. */
#if CPU_BE
	uint8		MtuSelector	:2;		/* enum IB_SELECTOR */
	uint8		Mtu			:6;		/* enum IB_MTU */
#else
	uint8		Mtu			:6;		/* enum IB_MTU */
	uint8		MtuSelector	:2;		/* enum IB_SELECTOR */
#endif
	uint8		TClass;				/* GRH Traffic Class */
	uint16		P_Key;				/* Partition Key */
#if CPU_BE
	uint8		RateSelector:2;		/* enum IB_SELECTOR */
	uint8		Rate		:6;		/* enum IB_STATIC_RATE */
#else
	uint8		Rate		:6;		/* enum IB_STATIC_RATE */
	uint8		RateSelector:2;		/* enum IB_SELECTOR */
#endif

	/* ***************************************** */
	/* *** User be aware that the CM LifeTime & */
	/* *** TimeOut values are only 5-bit wide. */
	/* ***************************************** */
	/* */
	/* Accumulated packet life time for the path specified by an enumeration  */
	/* deried from 4.096 usec * 2^PktLifeTime */
#if CPU_BE
	uint8		PktLifeTimeSelector:2;	/* enum IB_SELECTOR */
	uint8		PktLifeTime	:6;
#else
	uint8		PktLifeTime	:6;
	uint8		PktLifeTimeSelector:2;	/* enum IB_SELECTOR */
#endif
	union {
		uint32	AsReg32;
		struct {
#if CPU_BE
			uint32		SL:4;
			uint32		FlowLabel:20;
			uint32		HopLimit:8;
#else
			uint32		HopLimit:8;
			uint32		FlowLabel:20;
			uint32		SL:4;
#endif
		} PACK_SUFFIX s;
	} u1;
#if CPU_BE
	uint8	Scope:4;
	uint8	Reserved:1;
	uint8	JoinSendOnlyMember:1;
	uint8	JoinNonMember:1;
	uint8	JoinFullMember:1;
#else
	uint8	JoinFullMember:1;
	uint8	JoinNonMember:1;
	uint8	JoinSendOnlyMember:1;
	uint8	Reserved:1;
	uint8	Scope:4;
#endif
#if CPU_BE
	uint8	ProxyJoin:1;
	uint8	Reserved2:7;
#else
	uint8	Reserved2:7;
	uint8	ProxyJoin:1;
#endif
	uint8	Reserved3[2];	/* TBD spec has odd size here */
} PACK_SUFFIX IB_MCMEMBER_RECORD;




/* --------------------------------------------------------------------------
 * Trace Record - trace a path through the fabric
 */

/* ComponentMask bits */
#define IB_TRACE_RECORD_COMP_GIDPREFIX						0x00000001
#define IB_TRACE_RECORD_COMP_IDGENERATION					0x00000002
	/* reserved field										0x00000004 */
#define IB_TRACE_RECORD_COMP_NODETYPE						0x00000008
#define IB_TRACE_RECORD_COMP_NODEID							0x00000010
#define IB_TRACE_RECORD_COMP_CHASSISID						0x00000020
#define IB_TRACE_RECORD_COMP_ENTRYPORTID					0x00000040
#define IB_TRACE_RECORD_COMP_EXITPORTID						0x00000080
#define IB_TRACE_RECORD_COMP_ENTRYPORT						0x00000100
#define IB_TRACE_RECORD_COMP_EXITPORT						0x00000200

#define IB_TRACE_RECORD_COMP_ENCRYPT_MASK                   0x55555555

typedef struct _IB_TRACE_RECORD {
	uint64			GIDPrefix;			/* subnet in which IDs are consistent */
	uint16			IDGeneration;
	uint8			Reserved;
	uint8			NodeType;			/* see NODE_INFO */
	uint64			NodeID;
	uint64			ChassisID;
	uint64			EntryPortID;
	uint64			ExitPortID;
	uint8			EntryPort;
	uint8			ExitPort;
} PACK_SUFFIX IB_TRACE_RECORD;


/* --------------------------------------------------------------------------
 * Multipath Record - a set of paths between nodes in the fabric
 */

/* ComponentMask bits */
#define IB_MULTIPATH_RECORD_COMP_RAWTRAFFIC					0x00000001
	/* reserved field										0x00000002 */
#define IB_MULTIPATH_RECORD_COMP_FLOWLABEL					0x00000004
#define IB_MULTIPATH_RECORD_COMP_HOPLIMIT					0x00000008
#define IB_MULTIPATH_RECORD_COMP_TCLASS						0x00000010
#define IB_MULTIPATH_RECORD_COMP_REVERSIBLE					0x00000020
#define IB_MULTIPATH_RECORD_COMP_NUMBPATH					0x00000040
#define IB_MULTIPATH_RECORD_COMP_PKEY						0x00000080
	/* reserved field										0x00000100 */
#define IB_MULTIPATH_RECORD_COMP_SL							0x00000200
#define IB_MULTIPATH_RECORD_COMP_MTUSELECTOR				0x00000400
#define IB_MULTIPATH_RECORD_COMP_MTU						0x00000800
#define IB_MULTIPATH_RECORD_COMP_RATESELECTOR				0x00001000
#define IB_MULTIPATH_RECORD_COMP_RATE						0x00002000
#define IB_MULTIPATH_RECORD_COMP_PKTLIFESELECTOR			0x00004000
#define IB_MULTIPATH_RECORD_COMP_PKTLIFE					0x00008000
#define IB_MULTIPATH_RECORD_COMP_SERVICEID8MSB				0x00010000
#define IB_MULTIPATH_RECORD_COMP_INDEPENDENCESELECTOR		0x00020000
	/* reserved field										0x00040000 */
#define IB_MULTIPATH_RECORD_COMP_SGIDCOUNT					0x00080000
#define IB_MULTIPATH_RECORD_COMP_DGIDCOUNT					0x00100000
#define IB_MULTIPATH_RECORD_COMP_SERVICEID56LSB				0x00200000
	/* SDGID list follows, no component mask bits */
	/* since SGID/DGIDCOUNT indicates number present */

// Convenience define that combines the 8MSB and 56LSB bits.
#define IB_MULTIPATH_RECORD_COMP_SERVICEID					0x00210000

// The most sgids or dgids you can have. Thus, the total # of GIDS is 
// twice this.
#define IB_MULTIPATH_RECORD_MAX_GIDCOUNT					255

typedef struct _IB_MULTIPATH_RECORD {
	union {
		uint32		AsReg32;
		struct {
#if CPU_BE
			uint32	RawTraffic:1;
			uint32	Reserved:3;
			uint32	FlowLabel:20;
			uint32	HopLimit:8;
#else
			uint32	HopLimit:8;
			uint32	FlowLabel:20;
			uint32	Reserved:3;
			uint32	RawTraffic:1;
#endif
		} PACK_SUFFIX s;
	} u1;
	uint8			TClass;
#if CPU_BE
	uint8			Reversible:1;
	uint8			NumbPath:7;
#else
	uint8			NumbPath:7;
	uint8			Reversible:1;
#endif
	uint16			P_Key;
	union {
		uint16		AsReg16;
		struct {
#if CPU_BE
			uint16	Reserved2:12;
			uint16	SL:4;
#else
			uint16	SL:4;
			uint16	Reserved2:12;
#endif
		} PACK_SUFFIX s;
	} u2;
#if CPU_BE
	uint8		MtuSelector	:2;		/* enum IB_SELECTOR */
	uint8		Mtu			:6;		/* enum IB_MTU */
#else
	uint8		Mtu			:6;		/* enum IB_MTU */
	uint8		MtuSelector	:2;		/* enum IB_SELECTOR */
#endif

#if CPU_BE
	uint8		RateSelector:2;		/* enum IB_SELECTOR */
	uint8		Rate		:6;		/* enum IB_STATIC_RATE */
#else
	uint8		Rate		:6;		/* enum IB_STATIC_RATE */
	uint8		RateSelector:2;		/* enum IB_SELECTOR */
#endif

	/* *****************************************
	 * *** User be aware that the CM LifeTime &
	 * *** TimeOut values are only 5-bit wide.
	 * ***************************************** */
	/* Accumulated packet life time for the path specified by an enumeration  */
	/* deried from 4.096 usec * 2^PktLifeTime */
#if CPU_BE
	uint8		PktLifeTimeSelector:2;	/* enum IB_SELECTOR */
	uint8		PktLifeTime	:6;
#else
	uint8		PktLifeTime	:6;
	uint8		PktLifeTimeSelector:2;	/* enum IB_SELECTOR */
#endif
	uint8		serviceID8msb;
#if CPU_BE
	uint8		IndependenceSelector:2;	/* 1->as fault independent as possible */
										/* 0,2,3 reserved */
	uint8		Reserved4:6;
#else
	uint8		Reserved4:6;
	uint8		IndependenceSelector:2;	/* 1->as fault independent as possible */
										/* 0,2,3 reserved */
#endif
	uint8		SGIDCount;
	uint8		DGIDCount;
	uint8		serviceID56lsb[7];
	IB_GID		GIDList[1];				/* SGIDCount source GIDs */
										/* followed by DGIDCount dest GIDs */
} PACK_SUFFIX IB_MULTIPATH_RECORD;




/* --------------------------------------------------------------------------
 * VFabric Record
 */
/* ComponentMask bits */
#define VEND_VF_RECORD_COMP_INDEX				0x00000001
#define VEND_VF_RECORD_COMP_PKEY				0x00000002
#define VEND_VF_RECORD_COMP_NAME				0x00000008
#define VEND_VF_RECORD_COMP_SERVICEID			0x00000010
#define VEND_VF_RECORD_COMP_MGID				0x00000020
#define VEND_VF_RECORD_COMP_SL					0x00000080

#define VEND_VFABRIC_NAME_COUNT					64

#define VEND_PKEY_SEL							0x01
#define VEND_SL_SEL								0x02

#define OPT_VF_SECURITY							0x01
#define OPT_VF_QOS								0x02
#define OPT_VF_FLOW_DISABLE						0x04


typedef struct _VF_INFO_RECORD {
	uint16	vfIndex;		 		/* The index assigned to the VF */
	uint16	pKey;					/* PKey associated with the VF */
	uint32  rsvd6;
	uint8	vfName[64];				/* The name of the VF */
	uint64		ServiceID;			/* for query only */
	IB_GID		MGID;				/* for query only */
	struct {
#if CPU_BE      
    	uint8	selectFlags:4;  	/* 1 bit to indicate SL in queries, 1 bit for pkey, - 4bits total */
    	uint8	sl:			4;		/* service level - 4 bits */
#else
    	uint8	sl:			4;		/* service level - 4 bits */
    	uint8	selectFlags:4; 	 	/* 1 bit to indicate SL in queries, 1 bit for pkey, - 4bits total */
#endif
#if CPU_BE      
    	uint8	mtuSpecified:1;		/* mtu specified for VF - 1 bit */
    	uint8	rsvd1:1;	    
    	uint8	mtu:6;				/* max mtu assigned to VF - 6 bits */
#else
		uint8	mtu:6;				/* max mtu assigned to VF - 6 bits */
    	uint8	rsvd1:1;	    
		uint8	mtuSpecified:1;		/* mtu specified for VF - 1 bit */
#endif
#if CPU_BE      
		uint8	rateSpecified:1;	/* rate specified for VF - 1 bit */
		uint8	rsvd2:1;
		uint8	rate:6;				/* max rate assigned to VF - 6 bits */
#else
		uint8	rate:6;				/* max rate assigned to VF - 6 bits */
		uint8	rsvd2:1;
		uint8	rateSpecified:1;	/* rate specified for VF - 1 bit */
#endif
#if CPU_BE      
		uint8	pktLifeSpecified:1; /* pkt life time specified for VF - 1 bit */
		uint8	rsvd3:4;
		uint8	pktLifeTimeInc:3;   /* pkt life time assigned to VF - 3 bits */
#else
		uint8	pktLifeTimeInc:3;   /* pkt life time assigned to VF - 3 bits */
		uint8	rsvd3:4;
		uint8	pktLifeSpecified:1; /* pkt life time specified for VF - 1 bit */
#endif
	} s1;
    uint8       optionFlags;		/* security bit, QoS bit, 6 reserved */
    uint8       bandwidthPercent;	/* bandwidth percentage, 8 bits */
	struct {
#if CPU_BE      
        uint8   rsvd4:7;	
        uint8   priority:1;			/* priority, 1 bit */
#else
        uint8   priority:1;			/* priority, 1 bit */
        uint8   rsvd4:7;
#endif
	} s2;
	uint8		routingSLs;
	uint8		rsvd5[24];

} PACK_SUFFIX VEND_VFINFO_RECORD;

/* --------------------------------------------------------------------------
 * Vendor unique collective group Record
 */

#define VEND_CG_RECORD_COMPONENTMASK_CGID        0x0000000000000001ull
#define VEND_CG_RECORD_COMPONENTMASK_NODE        0x0000000000000002ull

typedef struct _COLLECTIVE_GROUP_RECORD {
    uint64_t    cgid;                           // a hash which uniquely identifies the group
    uint64_t    nodeGuid;                       // node making the request
    uint16_t    clid;                           // collective group lid (assigned by SA/SM on create)
    uint16_t    numMembers;                     // the number of node members in the group
    uint8_t     auxStatus;                      // auxiliary status info returned on set/delete
    uint8_t     isFinal;                        // Valid on Delete, indicates release Immediate - 1 bit
    uint32_t    rsvd1;                          // Reserved - 19 bits
    uint8_t     sl;                             // service level - 4 bits
    uint16_t    pKey;
    uint8_t     mtuSelector;
    uint8_t     mtuValue;
    uint8_t     rateSelector;
    uint8_t     rateValue;
} PACK_SUFFIX VEND_COLLECTIVE_GROUP_RECORD;


/* --------------------------------------------------------------------------
 * Vendor unique collective group status Record
 */

#define VEND_CGS_RECORD_COMPONENTMASK_CLID       0x0000000000000001ull
#define VEND_CGS_RECORD_COMPONENTMASK_LID        0x0000000000000002ull

#define VEND_CG_STATUS_PROTOCOL_ERROR            0x1

typedef struct _COLLECTIVE_GROUP_STATUS_RECORD {
    uint16_t    clid;                           // collective group lid
    uint16_t    lid;                            // lid of node reporting protocol error
    uint8_t     cgStatus;                       // collective group status
} PACK_SUFFIX VEND_COLLECTIVE_GROUP_STATUS_RECORD;


/* --------------------------------------------------------------------------
 * Vendor unique CFT Record
 */

#define VEND_CFT_RECORD_COMPONENTMASK_CLID       0x0000000000000001ull
#define VEND_CFT_RECORD_COMPONENTMASK_LID        0x0000000000000002ull

typedef struct _COLLECTIVE_GROUP_FORWARDING_TABLE_RECORD {
    uint16_t    clid;                           // collective group lid
    uint16_t    lid;                            // lid of node reporting protocol error
    COLLECTIVE_FORWARDING_TABLE       cft;      // collective forwarding table
} PACK_SUFFIX VEND_COLLECTIVE_GROUP_FORWARDING_TABLE_RECORD;


#include "iba/public/ipackoff.h"

/*---------------------------------------------------------------------------
 * Swap between CPU and network byte ordering
 */

static __inline
void
BSWAP_IB_NODE_RECORD(
    IB_NODE_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
    BSWAP_NODE_INFO(&Dest->NodeInfoData);
#endif /* CPU_LE */
}

static __inline
void
BSWAP_IB_PORTINFO_RECORD(
    IB_PORTINFO_RECORD  *Dest, int extended
    )
{
#if CPU_LE

	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
    BSWAP_PORT_INFO(&Dest->PortInfoData, extended);
#endif
}

static __inline
void
BSWAP_IB_SWITCHINFO_RECORD(
    IB_SWITCHINFO_RECORD  *Dest
    )
{
#if CPU_LE

	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
	BSWAP_SWITCH_INFO(&Dest->SwitchInfoData);
#endif
}

static __inline
void
BSWAP_IB_LINEAR_FDB_RECORD(
    IB_LINEAR_FDB_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
	BSWAP_LINEAR_FWD_TABLE(&Dest->LinearFdbData);
#endif
}

static __inline
void
BSWAP_IB_RANDOM_FDB_RECORD(
    IB_RANDOM_FDB_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
	BSWAP_RANDOM_FWD_TABLE(&Dest->RandomFdbData);
#endif
}

static __inline
void
BSWAP_IB_MCAST_FDB_RECORD(
    IB_MCAST_FDB_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
	BSWAP_MCAST_FWD_TABLE(&Dest->MCastFdbData);
#endif
}

static __inline
void
BSWAP_IB_VLARBTABLE_RECORD(
    IB_VLARBTABLE_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
	BSWAP_VLARBTABLE(&Dest->VLArbData);
#endif
}

static __inline
void
BSWAP_IB_SMINFO_RECORD(
    IB_SMINFO_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.AsReg32 = ntoh32(Dest->RID.AsReg32);
    BSWAP_SM_INFO(&Dest->SMInfoData);
#endif
}

static __inline
void
BSWAP_IB_P_KEY_TABLE_RECORD(
    IB_P_KEY_TABLE_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.s2.AsReg32 = ntoh32(Dest->RID.s2.AsReg32);
    BSWAP_PART_TABLE(&Dest->PKeyTblData);
#endif
}

static __inline
void
BSWAP_IB_INFORM_INFO_RECORD(
    IB_INFORM_INFO_RECORD  *Dest
    )
{
#if CPU_LE
	BSWAP_IB_GID(&Dest->RID.SubscriberGID);
	Dest->RID.Enum = ntoh16(Dest->RID.Enum);
    BSWAP_INFORM_INFO(&Dest->InformInfoData);
#endif
}

static __inline
void
BSWAP_IB_LINK_RECORD(
    IB_LINK_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->RID.FromLID = ntoh16(Dest->RID.FromLID);
	Dest->ToLID = ntoh16(Dest->ToLID);
#endif /* CPU_LE */
}

static __inline
void
BSWAP_IB_SERVICE_RECORD(
    IB_SERVICE_RECORD  *Dest
    )
{
#if CPU_LE
	uint8 i;

	Dest->RID.ServiceID = ntoh64(Dest->RID.ServiceID);
	BSWAP_IB_GID(&Dest->RID.ServiceGID);
	Dest->RID.ServiceP_Key = ntoh16(Dest->RID.ServiceP_Key);
	Dest->ServiceLease = ntoh32(Dest->ServiceLease);
	ntoh(&Dest->ServiceKey[0], &Dest->ServiceKey[0], 16);

	for (i=0; i<8; ++i)
	{
		Dest->ServiceData16[i] = ntoh16(Dest->ServiceData16[i]);
	}
	for (i=0; i<4; ++i)
	{
		Dest->ServiceData32[i] = ntoh32(Dest->ServiceData32[i]);
	}
	for (i=0; i<2; ++i)
	{
		Dest->ServiceData64[i] = ntoh64(Dest->ServiceData64[i]);
	}
#endif /* CPU_LE */
}

static __inline
void
BSWAP_IB_SERVICEASSOCIATION_RECORD(
    IB_SERVICEASSOCIATION_RECORD  *Dest
    )
{
#if CPU_LE
	ntoh(&Dest->ServiceKey[0], &Dest->ServiceKey[0], 16);
#endif
}

static __inline
void
BSWAP_IB_MCMEMBER_RECORD(
    IB_MCMEMBER_RECORD  *Dest
    )
{
#if CPU_LE
	BSWAP_IB_GID(&Dest->RID.MGID);
	BSWAP_IB_GID(&Dest->RID.PortGID);
	Dest->Q_Key = ntoh32(Dest->Q_Key);
	Dest->MLID = ntoh16(Dest->MLID);
	Dest->P_Key = ntoh16(Dest->P_Key);
	Dest->u1.AsReg32 = ntoh32(Dest->u1.AsReg32);
#endif /* CPU_LE */
}

static __inline
void
BSWAPCOPY_IB_MCMEMBER_RECORD(IB_MCMEMBER_RECORD *Src, IB_MCMEMBER_RECORD *Dest)
{
	memcpy(Dest, Src, sizeof(IB_MCMEMBER_RECORD));
	BSWAP_IB_MCMEMBER_RECORD(Dest);
}

static __inline
void
BSWAP_IB_TRACE_RECORD(
    IB_TRACE_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->GIDPrefix = ntoh64(Dest->GIDPrefix);
	Dest->IDGeneration = ntoh16(Dest->IDGeneration);
	Dest->NodeID = ntoh64(Dest->NodeID);
	Dest->ChassisID = ntoh64(Dest->ChassisID);
	Dest->EntryPortID = ntoh64(Dest->EntryPortID);
	Dest->ExitPortID = ntoh64(Dest->ExitPortID);
#endif
}

static __inline
void
BSWAP_IB_MULTIPATH_RECORD(
    IB_MULTIPATH_RECORD  *Dest
    )
{
#if CPU_LE
	uint8 i;
	Dest->u1.AsReg32 = ntoh32(Dest->u1.AsReg32);
	Dest->P_Key = ntoh16(Dest->P_Key);
	Dest->u2.AsReg16 = ntoh16(Dest->u2.AsReg16);

	/* TBD - sanity check Counts */
	for (i=0; i<(Dest->SGIDCount + Dest->DGIDCount); ++i)
	{
		BSWAP_IB_GID(&Dest->GIDList[i]);
	}
#endif
}

static __inline
void
BSWAPCOPY_IB_MULTIPATH_RECORD(
    IB_MULTIPATH_RECORD  *Src, IB_MULTIPATH_RECORD  *Dest
    )
{
    memcpy(Dest, Src, sizeof(IB_MULTIPATH_RECORD));
    BSWAP_IB_MULTIPATH_RECORD(Dest);
}

static __inline
void
BSWAP_IB_PATH_RECORD(
    IB_PATH_RECORD  *Dest
    )
{
#if CPU_LE
	Dest->ServiceID = ntoh64(Dest->ServiceID);
    BSWAP_IB_GID(&Dest->DGID);
    BSWAP_IB_GID(&Dest->SGID);
 
    Dest->DLID =   ntoh16(Dest->DLID);
    Dest->SLID =   ntoh16(Dest->SLID);
	Dest->u1.AsReg32 =	ntoh32(Dest->u1.AsReg32);
    Dest->P_Key =  ntoh16(Dest->P_Key);
    Dest->u2.AsReg16 =  ntoh16(Dest->u2.AsReg16);
#endif /* CPU_LE */
}

static __inline
void
BSWAPCOPY_IB_PATH_RECORD(
    IB_PATH_RECORD  *Src, IB_PATH_RECORD  *Dest
    )
{
    memcpy(Dest, Src, sizeof(IB_PATH_RECORD));
    BSWAP_IB_PATH_RECORD(Dest);
}

static __inline
void
BSWAP_VEND_VFINFO_RECORD(
    VEND_VFINFO_RECORD  *Dest
    )
{
#if CPU_LE
    Dest->vfIndex =  ntoh16(Dest->vfIndex);
    Dest->pKey =     ntoh16(Dest->pKey);
    Dest->ServiceID =  ntoh64(Dest->ServiceID);
	BSWAP_IB_GID(&Dest->MGID);
#endif /* CPU_LE */
}

static __inline
void
BSWAP_VEND_COLLECTIVE_GROUP_RECORD(
    VEND_COLLECTIVE_GROUP_RECORD  *Dest
    )
{
#if CPU_LE
    Dest->cgid = ntoh64(Dest->cgid);
    Dest->nodeGuid = ntoh64(Dest->nodeGuid);
    Dest->clid = ntoh16(Dest->clid);
    Dest->numMembers = ntoh16(Dest->numMembers);
    Dest->rsvd1 = ntoh32(Dest->rsvd1);
    Dest->pKey = ntoh16(Dest->pKey);
#endif /* CPU_LE */
}

static __inline
void
BSWAP_VEND_COLLECTIVE_GROUP_STATUS_RECORD(
    VEND_COLLECTIVE_GROUP_STATUS_RECORD  *Dest
    )
{
#if CPU_LE
    Dest->clid = ntoh16(Dest->clid);
    Dest->lid = ntoh16(Dest->lid);
#endif /* CPU_LE */
}

static __inline
void
BSWAP_VEND_COLLECTIVE_GROUP_FORWARDING_TABLE_RECORD(
    VEND_COLLECTIVE_GROUP_FORWARDING_TABLE_RECORD  *Dest
    )
{
#if CPU_LE
    Dest->clid = ntoh16(Dest->clid);
    Dest->lid = ntoh16(Dest->lid);
	BSWAP_COLLECTIVE_FORWARDING_TABLE(&Dest->cft);
#endif /* CPU_LE */
}



/* determine if the given PATH_RECORD represents a global route
 * (eg. where GRH is used to go through an IB Router)
 */
static __inline boolean
IsGlobalRoute( IN const IB_PATH_RECORD *pPathRecord)
{
	return (pPathRecord->u1.s.HopLimit > 1);
}

/* compute LocalAckTimeout from PktLifeTime
 * note PktLifetime is one directional on wire, while LocalAckTimeout is
 * total round trip including CA Ack Delay
 * for client REQ.AckTimeout, caAckDelay should be our local CA's AckDelay
 * for client QP, use REP.TargetAckDelay
 * for server QP, use REQ.AckTimeout directly (no need to call this)
 * pktLifeTime, caAckDelay and returned values are IB timeout multipliers
 */
static __inline uint8
ComputeAckTimeout(IN uint8 pktLifeTime, IN uint8 caAckDelay)
{
    /* return TimeoutTimeToMult(TimeoutMultToTimeInUsec(pktLifeTime)*2 */
	/* 				+ TimeoutMultToTimeInUsec(caAckDelay)); */
  	/*return MIN(pktLifeTime + 2, 0x1f); */
	uint8 ackTimeout;

	/* tests also handle if remote endpoint didn't set REP.TargetAckDelay */
	if (pktLifeTime+1 >= caAckDelay)
	{
 		/* since its a log2 value, +1 doubles the timeout */
 		/* Additional +1 is to account for remote CA Ack delay */
		ackTimeout = pktLifeTime+2;
	} else {
		/* since caAckDelay > 2*pktLifetime, +1 simply doubles ca Ack Delay */
		ackTimeout = caAckDelay+1;
	}
	/* limit value to 5 bits (approx 2.4 hours) */
	if (ackTimeout > 0x1f)
		ackTimeout = 0x1f;
	return ackTimeout;
}

#ifndef IB_STACK_OPENIB
/* helper function, converts a PATH_RECORD to an IB_ADDRESS_VECTOR to facilitate
 * connection establishment and UD traffic
 * PathMTU is an IB_MTU enum
 */
static __inline void
GetAVFromPath2( IN uint64 PortGuid,	/* only needed for UD AVs */
	IN const IB_PATH_RECORD *pPathRecord,
	OUT uint8* PathMTU OPTIONAL,
	OUT IB_ADDRESS_VECTOR* DestAv OPTIONAL )
{
	if (DestAv)
	{
		DestAv->PortGUID = PortGuid; /* only needed for UD AVs */
		DestAv->DestLID = pPathRecord->DLID;
		DestAv->PathBits = (uint8)(pPathRecord->SLID&IB_PATHBITS_MASK);
		DestAv->ServiceLevel = pPathRecord->u2.s.SL;
		DestAv->StaticRate = pPathRecord->Rate;

		/* Global route information. */
		DestAv->GlobalRoute = IsGlobalRoute(pPathRecord);
		DestAv->GlobalRouteInfo.DestGID = pPathRecord->DGID;
		DestAv->GlobalRouteInfo.FlowLabel = pPathRecord->u1.s.FlowLabel;
		DestAv->GlobalRouteInfo.HopLimit = (uint8)pPathRecord->u1.s.HopLimit;
		DestAv->GlobalRouteInfo.SrcGIDIndex = 0;	/* BUGBUG assume 0 */
		DestAv->GlobalRouteInfo.TrafficClass = pPathRecord->TClass;
	}

	/* Reliable connection information, N/A for UD */
	if (PathMTU)
		*PathMTU = pPathRecord->Mtu;
}

/* This function is depricated, will be dropped in 3.1 release,
 * use GetAVFromPath2 above
 */
static __inline void
GetAVFromPath( IN uint64 PortGuid,	/* only needed for UD AVs */
	IN const IB_PATH_RECORD *pPathRecord,
	OUT uint8* PathMTU OPTIONAL, OUT uint8* LocalAckTimeout OPTIONAL,
	OUT IB_ADDRESS_VECTOR* DestAv OPTIONAL )
{
	GetAVFromPath2(PortGuid, pPathRecord, PathMTU, DestAv);
	if (LocalAckTimeout)
		*LocalAckTimeout = ComputeAckTimeout(pPathRecord->PktLifeTime,
								TimeoutTimeToMult(8));	/* guess: 8 ms */
}

/* helper function, converts a MCMEMBER_RECORD to an IB_ADDRESS_VECTOR to
 * facilitate UD traffic
 */
static __inline void
GetAVFromMcMemberRecord(EUI64              PortGuid,
                        IB_MCMEMBER_RECORD *pMcMemberRecord,
                        IB_ADDRESS_VECTOR  *pDestAv )
{
	if (pDestAv != NULL)
	{
		pDestAv->PortGUID = pMcMemberRecord->RID.PortGID.Type.Global.InterfaceID;
		pDestAv->DestLID = pMcMemberRecord->MLID;
/*		pDestAv->PathBits = (uint8)pPathRecord->SLID; */
		pDestAv->PathBits = 0;
		pDestAv->ServiceLevel = pMcMemberRecord->u1.s.SL;
		pDestAv->StaticRate = pMcMemberRecord->Rate;

		/* Global route information. */
		pDestAv->GlobalRoute = TRUE;
		pDestAv->GlobalRouteInfo.DestGID = pMcMemberRecord->RID.MGID;
		pDestAv->GlobalRouteInfo.FlowLabel = pMcMemberRecord->u1.s.FlowLabel;
		pDestAv->GlobalRouteInfo.HopLimit = (uint8)pMcMemberRecord->u1.s.HopLimit;
		pDestAv->GlobalRouteInfo.SrcGIDIndex = 0;	/* BUGBUG assume 0 */
		pDestAv->GlobalRouteInfo.TrafficClass = pMcMemberRecord->TClass;
	}
}



#endif /* IB_STACK_OPENIB */

#ifdef __cplusplus
}
#endif

#endif /* _IBA_IB_SA_RECORDS_H_ */
