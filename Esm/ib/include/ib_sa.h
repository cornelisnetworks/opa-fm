/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

//----------------------------------------------------------------------/
//                                                                      /
// FILE NAME                                                            /
//    ib_sa.h                                                           /
//                                                                      /
// DESCRIPTION                                                          /
//    SA specific structures                                            /
//                                                                      /
// DATA STRUCTURES                                                      /
//    SAMad_t			Common SA Mad header			/
//    NodeRecord_t		NodeInfo Record				/
//    InformRecord_t		Inform Record				/
//    ServiceRecord_t		Service Record				/
//    PathRecord_t		Path Record				/
//    McMemberRecord_t		McMember Record				/
//    		                                                        	/
// FUNCTIONS                                                            /
//    None                                                              /
//                                                                      /
// DEPENDENCIES                                                         /
//    ib_types.h                                                        /
//    ib_mad.h                                                          /
//                                                                      /
//                                                                      /
// HISTORY                                                              /
//                                                                      /
//    NAME      DATE  REMARKS                                           /
//     jsy  12/28/00  Initial creation of file.                         /
//                                                                      /
//----------------------------------------------------------------------/

// -------------------------------------------------------------------- //
//                                                                      //
//	WARNING:  These structures do NOT represent the IBTA defined	//
//		  structures as they appear in the Volume1 spec.	//
//                                                                      //
// -------------------------------------------------------------------- //

#ifndef	_IB_SA_H_
#define	_IB_SA_H_

#include "ib_types.h"
#include "ib_mad.h"
#include "cs_hashtable.h"
#include "iba/stl_sm.h"
#include "iba/stl_sa.h"

#if defined(__KERNEL__) && ! defined(__VXWORKS__)
#include "linux/iba/public/ipackon.h"
#else
#include "iba/public/ipackon.h"
#endif

// IBTA:  Volume 1, Section 15.2.1

//
//	Subnet Administration(SA) Mad format
//

#define IB_SAMAD_DATA_COUNT	     200
#define IB_SA_HEADER_SIZE        20      // SA header fields after RMPP header and before data
#define IB_SA_FULL_HEADER_SIZE   32      // SA header including RMPP header

#define STL_SAMAD_BLOCK_SIZE     2048
#define STL_SAMAD_DATA_COUNT     (STL_SAMAD_BLOCK_SIZE - (IB_SA_FULL_HEADER_SIZE + 8)) // Original SA header is one byte short

#define SAMAD_DATA_COUNT	     IB_SAMAD_DATA_COUNT
#define SA_HEADER_SIZE           IB_SA_HEADER_SIZE       // SA header fields after RMPP header and before data
#define SA_FULL_HEADER_SIZE      IB_SA_FULL_HEADER_SIZE  // SA header including RMPP header

// SA MAD header only
typedef	struct {
    uint8_t     rmppVersion;
    uint8_t     rmppType;
    union {
        struct {
			#if CPU_BE
			uint8_t  rmppRespTime : 5;   // 5 bit field
            uint8_t  rmppFlags : 3;      // 3 bit field
			#else
            uint8_t  rmppFlags : 3;      // 3 bit field
            uint8_t  rmppRespTime : 5;   // 5 bit field
			#endif
        } tf;
        uint8_t timeFlag;       // entire 8 bits
    } u;
    uint8_t     rmppStatus;     
    uint32_t    segNum;
    uint32_t    length;
	uint64_t	smKey;			// SM key
    uint16_t    offset;         // num 8-byte words from begin of attrib to begin of next attrib
    uint16_t    rservd;         // reserved
	uint64_t	mask;			// attribute components for queries
} PACK_SUFFIX SAMadh_t;

// (old = 192) Maximum data in a single SA Mad
#define	IB_SA_DATA_LEN		200	    

#define	SA_AMOD_TEMPLATE	0xffffffff

#define	SA_EM_ADD		0x0	// add record modifier
#define	SA_EM_DELETE 	0x1	// delete record modifier
#define	SA_EM_EDIT		0x2	// edit record modifier

// IBTA:  Volume 1, Section 15.2.3

#define	MAD_STATUS_SA_NO_ERROR	    0x0000	// no error
#define	MAD_STATUS_SA_NO_RESOURCES	0x0100	// No resources
#define	MAD_STATUS_SA_REQ_INVALID	0x0200	// Request invalid
#define MAD_STATUS_SA_NO_RECORDS    0x0300  // No records found
#define MAD_STATUS_SA_TOO_MANY_RECS 0x0400  // Too many records
#define MAD_STATUS_SA_REQ_INVALID_GID 0x0500 // invalid GID
#define MAD_STATUS_SA_REQ_INSUFFICIENT_COMPONENTS 0x0600 // Insufficient Components
#define MAD_STATUS_SA_REQ_DENIED	0x0700	// request is denied - new in IBTA 1.2.1
#define MAD_STATUS_SA_REQ_PRIORITY	0x0800	// suggested QoS is unobtainable - IBTA 1.2.1 Annex A13

// IBTA:  Volume 1, Section 13.6.2.1 - RMPP
#define RMPP_VERSION        1

#define RMPP_TYPE_NOT       0
#define RMPP_TYPE_DATA      1
#define RMPP_TYPE_ACK       2
#define RMPP_TYPE_STOP      3
#define RMPP_TYPE_ABORT     4

#define RMPP_RESPTIME_NONE  0x1F

#define RMPP_FLAGS_ACTIVE   0x01
#define RMPP_FLAGS_FIRST    0x02
#define RMPP_FLAGS_LAST     0x04

// IBTA:  Volume 1, Section 13.6.2.1 - RMPP Status
#define RMPP_STATUS_NO_ERROR                                0
#define RMPP_STATUS_STOP_NORESOURCES                        1
#define RMPP_STATUS_ABORT_TIMETOOLONG                       118
#define RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH   119
#define RMPP_STATUS_ABORT_INCONSISTENT_FIRST_SEGNUM         120
#define RMPP_STATUS_ABORT_BADTYPE                           121
#define RMPP_STATUS_ABORT_NEWWINDOWLAST_TOOSMALL            122
#define RMPP_STATUS_ABORT_SEGNUM_TOOBIG                     123
#define RMPP_STATUS_ABORT_ILLEGAL_STATUS                    124
#define RMPP_STATUS_ABORT_UNSUPPORTED_VERSION               125
#define RMPP_STATUS_ABORT_TOO_MANY_RETRIES                  126
#define RMPP_STATUS_ABORT_UNSPECIFIED                       127

// IBTA:  Volume 1, Section 15.2.2

//
//	SA class methods
//
#define	SA_CM_GET		    0x01	// Get an attribute from a node
#define	SA_CM_GET_RESP	    0x81	// Get/Set response
#define	SA_CM_SET		    0x02	// Set an attribute from a node
#define	SA_CM_REPORT		0x06	// Forward a subscribed event
#define	SA_CM_REPORT_RESP	0x86	// Report response
#define	SA_CM_GETTABLE		0x12	// SM table request
#define	SA_CM_GETTABLE_RESP	0x92	// Gettable response
#define	SA_CM_GETTRACETABLE	0x13	// Request path trace table - returns GETTABLE_RESP
#define	SA_CM_GETMULTI  	0x14	// Multi-packet request
#define	SA_CM_GETMULTI_RESP 0x94	// Multi-packet response
#define	SA_CM_DELETE		0x15	// Request to delete an attribute
#define	SA_CM_DELETE_RESP	0x95	// Response to delete an attribute

// IBTA:  Volume 1, Section 15.2.5.1

//
//	SA Attributes
//
#define	SA_CLASSPORTINFO	0x0001	// Class information
#define	SA_NOTICE		0x0002	// Notice information
#define	SA_INFORMINFO		0x0003	// Subscription information
#define	SA_NODE_RECORD		0x0011	// NodeInfo record
#define	SA_PORTINFO_RECORD	0x0012	// PortInfo record
#define	SA_SWITCH_RECORD	0x0014	// SwitchInfo record
#define	SA_LFT_RECORD		0x0015	// LFT record
#define	SA_RFT_RECORD		0x0016	// RFT record
#define	SA_MFT_RECORD		0x0017	// MFT record
#define	SA_SMINFO_RECORD	0x0018	// SmInfo record
#define	SA_INFORM_RECORD	0x00f3	// InforInfo record
//was	SA_NOTICE_RECORD	0x00f4	// Notice record
#define	SA_LINK_RECORD		0x0020	// Link record
//was 	SA_GUIDINFO_RECORD	0x0030	// GUIDs assigned to a port
#define	SA_SERVICE_RECORD	0x0031	// Service Ad record
#define	SA_PARTITION_RECORD	0x0033	// Partition record
//was	SA_RANGE_RECORD		0x0034	// Range record
#define	SA_PATH_RECORD		0x0035	// Subnet path information
#define	SA_VLARBITRATION_RECORD	0x0036	// VL arbitration record
//was	SA_MCGROUP_RECORD	0x0037	// MC Group record
#define	SA_MCMEMBER_RECORD	0x0038	// MC Member record
#define SA_TRACE_RECORD     0x0039  // Trace Record
#define SA_MULTIPATH_RECORD 0x003A  // Request for multi paths
#define	SA_SARESPONSE		0x8001	// Container for query responses
#define SA_VFABRIC_RECORD	0xFF02  // "Vendor unique" vfabric record
#define SA_JOB_MANAGEMENT	0xFFB2  // job management extensions

// IBTA:  Volume 1, Section 15.2.5.1

//
//	NodeRecord
//
typedef	struct {
    Lid_t    lid;        // LID of CA port or LID of switch port 0
    uint16_t    reserved;
    STL_NODE_INFO	nodeInfo;	// NodeInfo
    NodeDesc_t	nodeDesc;	// NodeDesc
} NodeRecord_t;
#define NR_COMPONENTMASK_LID            0x0000000000000001ull
#define NR_COMPONENTMASK_NI_NODETYPE    0x0000000000000010ull
#define NR_COMPONENTMASK_NODEGUID       0x0000000000000080ull
#define NR_COMPONENTMASK_PORTGUID       0x0000000000000100ull
#define NR_COMPONENTMASK_PORTNUMBER     0x0000000000001000ull

// IBTA:  Volume 1, Section 15.2.5.10

//
//	PartitionRecord
//
typedef	struct {
	Lid_t		portLid;	    // port LID
    uint16_t    blockNum;       // Pkey table block number
    uint8_t     portNum;        // switch port number
    uint32_t    rsrvd1;       
	STL_PARTITION_TABLE	pkeyTable;	// contents of PKey table
} PartitionRecord_t;
#define	PKEY_COMPONENTMASK_PORTLID	0x0000000000000001ull
#define	PKEY_COMPONENTMASK_BLOCKNUM	0x0000000000000002ull
#define	PKEY_COMPONENTMASK_PORTNUM	0x0000000000000004ull

// IBTA:  Volume 1, Section 15.2.5.11

//
//	InformRecord
//
typedef	struct {
	Gid_t		    subscriberGid;	// subscriber GID
	uint16_t	    id;	            // unique id
    uint8_t         reserved[6];    // 48 bits reserved
	InformInfo_t	informInfo;	    // Inform Info
} InformRecord_t;
typedef InformRecord_t * InformRecordp;


// IBTA:  Volume 1, Section 15.2.5.14

//
//	ServiceRecord
//

#define SERVICE_RECORD_NAME_COUNT	64
//#define SERVICE_RECORD_GID_COUNT	16
#define SERVICE_RECORD_KEY_COUNT	16

typedef	struct {
    uint64_t    serviceId;          // Id of service on port specified by serviceGid
	Gid_t		serviceGid;	        // String of 16 hex digits
    uint16_t    servicep_key;       // p_key used in contacting service
	uint16_t	reserved;
	uint32_t	serviceLease;		// seconds remaining in lease
    uint8_t     serviceKey[SERVICE_RECORD_KEY_COUNT];  // key associated with service name
	uint8_t		name[SERVICE_RECORD_NAME_COUNT];// Null terminated service name
    uint8_t     data8[16];			// data8[2] and data8[3] used by config consistency check
    uint16_t    data16[8];
    uint32_t    data32[4];			// data32[0] and data32[1] used by config consistency check
    uint64_t    data64[2];
} ServiceRecord_t;
typedef ServiceRecord_t * ServiceRecordp;


#define SR_COMPONENTMASK_ID 	0x01ull
#define SR_COMPONENTMASK_GID 	0x02ull
#define SR_COMPONENTMASK_PKEY 	0x04ull
#define SR_COMPONENTMASK_LEASE 	0x10ull
#define SR_COMPONENTMASK_KEY 	0x20ull
#define SR_COMPONENTMASK_NAME 	0x40ull
#define	SR_GFLAG_INDIRECTION	0x1	// set if service may redirect
#define	SR_GFLAG_DHCP_CAPABLE	0x2	// set if DHCP server may register

// IBTA:  Volume 1, Section 15.2.5.15


//
//	PathRecord
//

#define PATH_RECORD_RSVD4_COUNT	6

typedef struct {
	uint64_t    serviceId;  // service ID  - IBTA: Section A13.5.3.1
	Gid_t		dstGid;		// destination GID
	Gid_t		srcGid;		// source GID
	Lid_t		dstLid;		// destination LID
	Lid_t		srcLid;		// source LID
	uint8_t		raw;		// raw flag
	uint8_t		rsvd2;		// reserved
	uint32_t	flowLabel;	// flow label
	uint8_t		hopLimit;	// hop limit
	uint8_t		tClass;		// tClass (GRH)
	uint8_t		reversible;	// reversible path indicator/requirement
	uint8_t		numPath;	// numbPath
	PKey_t		pKey;		// partition key for this path
    uint16_t	qosClass;	// QoS class - 12 bits, defined in A13.5.3.1
	uint8_t	    sl;		    // service level - 4 bits
	uint8_t		mtuSelector;	// mtu selector
	uint8_t		mtuValue;	// mtu value
	uint8_t		rateSelector;	// rate selector
	uint8_t		rateValue;	// rate value
	uint8_t		lifeSelector;	// life time selector
	uint8_t		lifeValue;	// life time value
    uint8_t     preference; // order of preference amongst multi paths
	uint8_t		rsvd4[PATH_RECORD_RSVD4_COUNT];	// reserved
} PathRecord_t;

/* IB 1.1 */
#define	PR_COMPONENTMASK_NO_DST	0x0000000000000014ull
#define	PR_COMPONENTMASK_OK_SRC	0x0000000000001008ull

#define	PR_COMPONENTMASK_SRV_ID	0x0000000000000001ull
#define	PR_COMPONENTMASK_DGID	0x0000000000000004ull
#define	PR_COMPONENTMASK_SGID	0x0000000000000008ull
#define	PR_COMPONENTMASK_DLID	0x0000000000000010ull
#define	PR_COMPONENTMASK_SLID	0x0000000000000020ull
#define	PR_COMPONENTMASK_RAW	0x0000000000000040ull
#define	PR_COMPONENTMASK_FLOW	0x0000000000000100ull
#define	PR_COMPONENTMASK_HOP	0x0000000000000200ull
#define	PR_COMPONENTMASK_TCLASS	0x0000000000000400ull
#define	PR_COMPONENTMASK_REVERSIBLE	0x0000000000000800ull
#define	PR_COMPONENTMASK_PATHS	0x0000000000001000ull
#define	PR_COMPONENTMASK_PKEY	0x0000000000002000ull
#define	PR_COMPONENTMASK_QOS	0x0000000000004000ull
#define	PR_COMPONENTMASK_SL	    0x0000000000008000ull
#define	PR_COMPONENTMASK_MTU_SEL  0x0000000000010000ull
#define	PR_COMPONENTMASK_MTU	0x0000000000020000ull
#define	PR_COMPONENTMASK_RATE_SEL 0x0000000000040000ull
#define	PR_COMPONENTMASK_RATE	0x0000000000080000ull
#define	PR_COMPONENTMASK_LIFE_SEL 0x0000000000100000ull
#define	PR_COMPONENTMASK_LIFE	0x0000000000200000ull

#define	PR_GT			0	// value is greater than
#define	PR_LT			1	// value is less than
#define	PR_EQ			2	// value is equal
#define	PR_MAX			3	// maximum possible

#define PR_LIFE_MAX			0x3F

// IBTA:  Volume 1, Section 15.2.5.17

//
//	MCMemberRecord
//
typedef	struct {
	Gid_t		mGid;
	Gid_t		portGid;
	uint32_t	qKey;
	Lid_t	mLid;
	uint8_t		mtuSelector;
	uint8_t		mtuValue;
	uint8_t		tClass;
	uint16_t	pKey;
	uint8_t		rateSelector;
	uint8_t		rateValue;
	uint8_t		lifeSelector;
	uint8_t		lifeValue;
	uint8_t		sl;
	uint32_t	flowLabel;
	uint8_t		hopLimit;
	uint8_t		scope;
	uint8_t		joinState;
	uint8_t		proxyJoin;
	uint32_t	rsvd;
} McMemberRecord_t;
//
//	Multicast dbsync structures.
//
typedef	struct {
	Gid_t			mGid;
	uint32_t		members_full;
	uint32_t		qKey;
	uint16_t		pKey;
	Lid_t			mLid;
	uint8_t			mtu;
	uint8_t			rate;
	uint8_t			life;
	uint8_t			sl;
	uint32_t		flowLabel;
	uint8_t			hopLimit;
	uint8_t			tClass;
	uint8_t			scope;
    uint8_t         filler;
    uint16_t        membercount;
	uint32_t		index_pool; /* Next index to use for new Mc Member records */
} McGroupSync_t;

#define MC_COMPONENTMASK_MGID			0x0000000000000001ull
#define MC_COMPONENTMASK_PGID			0x0000000000000002ull
#define MC_COMPONENTMASK_QKEY			0x0000000000000004ull
#define MC_COMPONENTMASK_MLID			0x0000000000000008ull
#define MC_COMPONENTMASK_MTU_SEL		0x0000000000000010ull
#define MC_COMPONENTMASK_MTU			0x0000000000000020ull
#define MC_COMPONENTMASK_TCLASS			0x0000000000000040ull
#define MC_COMPONENTMASK_PKEY			0x0000000000000080ull
#define MC_COMPONENTMASK_RATE_SEL		0x0000000000000100ull
#define MC_COMPONENTMASK_RATE			0x0000000000000200ull
#define MC_COMPONENTMASK_LIFE_SEL		0x0000000000000400ull
#define MC_COMPONENTMASK_LIFE			0x0000000000000800ull
#define MC_COMPONENTMASK_SL				0x0000000000001000ull
#define MC_COMPONENTMASK_FLOW			0x0000000000002000ull
#define MC_COMPONENTMASK_HOP			0x0000000000004000ull
#define MC_COMPONENTMASK_SCOPE			0x0000000000008000ull
#define MC_COMPONENTMASK_STATE			0x0000000000010000ull
#define MC_COMPONENTMASK_PROXY			0x0000000000020000ull

/* P_Key,  Q_Key, SL, FlowLabel, TClass, JoinState, PortGID */
#define MC_COMPONENTMASK_OK_CREATE		(MC_COMPONENTMASK_PKEY   | \
										 MC_COMPONENTMASK_QKEY   | \
										 MC_COMPONENTMASK_SL     | \
										 MC_COMPONENTMASK_FLOW   | \
										 MC_COMPONENTMASK_TCLASS | \
										 MC_COMPONENTMASK_STATE  | \
										 MC_COMPONENTMASK_PGID)

/* MGID, JoinState, PortGID */
#define MC_COMPONENTMASK_OK_JOIN		(MC_COMPONENTMASK_MGID   | \
										 MC_COMPONENTMASK_STATE  | \
										 MC_COMPONENTMASK_PGID)

/* All of MTU */
#define MC_COMPONENTMASK_OK_MTU			(MC_COMPONENTMASK_MTU_SEL | \
										 MC_COMPONENTMASK_MTU)
										 
/* All of Rate */
#define MC_COMPONENTMASK_OK_RATE		(MC_COMPONENTMASK_RATE_SEL | \
										 MC_COMPONENTMASK_RATE)
										 
/* All of Life */
#define MC_COMPONENTMASK_OK_LIFE		(MC_COMPONENTMASK_LIFE_SEL | \
										 MC_COMPONENTMASK_LIFE)


#define	MCMEMBER_JOIN_FULL_MEMBER		0x01
#define	MCMEMBER_JOIN_NON_MEMBER		0x02
#define	MCMEMBER_JOIN_SENDONLY_MEMBER		0x04

#define IB_LINK_LOCAL_SCOPE             0x2
#define IB_SITE_LOCAL_SCOPE             0x5
#define IB_ORG_LOCAL_SCOPE              0x8
#define IB_GLOBAL_SCOPE                 0xE

//
// IBTA:  Volume 1r1.1, Section 15.2.5.20

//
//	MultiPathRecord
//

typedef	struct {
	uint32_t    raw;	    // raw flag
	uint32_t	rsvd;		// reserved
	uint32_t	flowLabel;	// flow label
	uint8_t		hopLimit;	// hop limit
	uint8_t		tClass;		// tClass (GRH)
	uint8_t		reversible;	// reversible path indicator/requirement
	uint8_t		numPath;	// numbPath
	PKey_t		pKey;		// partition key for this path
	uint16_t	qosClass;	// qos - 12 bits - as per IBTA: A13.5.3.2
	uint8_t		sl;			// service level - 4 bits
	uint8_t		mtuSelector;	// mtu selector
	uint8_t		mtuValue;	// mtu value
	uint8_t		rateSelector;	// rate selector
	uint8_t		rateValue;	// rate value
	uint8_t		lifeSelector;	// life time selector
	uint8_t		lifeValue;	// life time value
	uint8_t		serviceID8msb;	// serviceId 8 MSB
	uint8_t		indepSelector;	// independence selector
	uint8_t		rsvd3;		// reserved
	uint8_t		sGidCount;	// sgid count
	uint8_t		dGidCount;	// dgid count
	uint64_t	serviceID56lsb; // 7 LSB of ServiceID
#define SGID1_OFFSET 24     // 192 bits - Table 179 of Vol1r1_1.pdf
	Gid_t	    *sGidList;	// pointer to sgid table
	Gid_t	    *dGidList;	// pointer to dgid table
} MultiPathRecord_t;
#define	MR_COMPONENTMASK_RAW	    0x0000000000000001ull
#define	MR_COMPONENTMASK_FLOW	    0x0000000000000004ull
#define	MR_COMPONENTMASK_HOP	    0x0000000000000008ull
#define	MR_COMPONENTMASK_TCLASS	    0x0000000000000010ull
#define	MR_COMPONENTMASK_REVERSIBLE	0x0000000000000020ull
#define	MR_COMPONENTMASK_PATHS	    0x0000000000000040ull
#define	MR_COMPONENTMASK_PKEY	    0x0000000000000080ull
#define	MR_COMPONENTMASK_QOS	    0x0000000000000100ull
#define	MR_COMPONENTMASK_SL	        0x0000000000000200ull
#define	MR_COMPONENTMASK_MTU_SEL	0x0000000000000400ull
#define	MR_COMPONENTMASK_MTU     	0x0000000000000800ull
#define	MR_COMPONENTMASK_RATE_SEL	0x0000000000001000ull
#define	MR_COMPONENTMASK_RATE	    0x0000000000002000ull
#define	MR_COMPONENTMASK_LIFE_SEL	0x0000000000004000ull
#define	MR_COMPONENTMASK_LIFE	    0x0000000000008000ull
#define	MR_COMPONENTMASK_SRVID	    0x0000000000010000ull
#define MR_COMPONENTMASK_INDEP_SEL  0x0000000000020000ull

//
//	OPA specific internal SA records.
//
//	***** WARNING ***** WARNING ***** WARNING ***** WARNING *****
//
//	These records can be changed at any time without notice.
//
//	***** WARNING ***** WARNING ***** WARNING ***** WARNING *****
//

//
//	OpaServiceRecord_t - This is the same as the original with a
//			      timestamp appended.
//
typedef	struct {
	STL_SERVICE_RECORD	serviceRecord;	
	uint64_t			expireTime;	
	uint8_t				pkeyDefined;// was the record created using pkey mask?
} OpaServiceRecord_t;
typedef OpaServiceRecord_t * OpaServiceRecordp; /* pointer to record */

/*
 * subscriber table (informInfo)
 */
typedef struct {
    CS_HashTablep   subsMap;  /* pointer to hashmap of subscriptions */
    Lock_t          subsLock;
} SubscriberTable_t;

/*
 *	event subscription entry key structure 
 */
typedef struct {
    Gid_t           subscriberGid;
	uint16_t	    lid;		// destination lid
    uint16_t        trapnum;    // trap number
    uint32_t        qpn;        // queue pair number - 24 bits
    uint8_t         producer;   // producer of traps/notices - using 3 bits
	uint8_t         rtv;        // time to respond to requests - 5 bits
	uint16_t        pkey;		// original PKey
	uint32_t	    qkey;	    // queue pair key (32)
	uint16_t	    startLid;	// calculated start Lid
	uint16_t	    endLid;		// calculated end Lid
	uint8_t			ibMode;		// true if subscriber needs IB notices.
} SubscriberKey_t;
typedef SubscriberKey_t * SubscriberKeyp;

/*
 * service record table (ServiceRecord)
 */
typedef struct {
    CS_HashTablep   serviceRecMap;  /* pointer to hashmap of service records */
    Lock_t          serviceRecLock;
} ServiceRecTable_t;

/*
 *	event subscription entry key structure 
 */
typedef struct {
    uint64_t    serviceId;          // Id of service on port specified by serviceGid
	IB_GID		serviceGid;	        
    uint16_t    servicep_key;       // p_key used in contacting service
} ServiceRecKey_t;
typedef ServiceRecKey_t * ServiceRecKeyp;

//
// Calculate 8-byte multiple padding for multi record SA responses
//
#define Calculate_Padding(RECSIZE)  ( (((RECSIZE)%8) == 0 ) ? 0 : (8 - ((RECSIZE)%8)) )

#endif	// _IB_SA_H_
