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
#include "iba/stl_sm_types.h"
#include "iba/stl_sa_priv.h"

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

#define IB_LINK_LOCAL_SCOPE             0x2
#define IB_SITE_LOCAL_SCOPE             0x5
#define IB_ORG_LOCAL_SCOPE              0x8
#define IB_GLOBAL_SCOPE                 0xE

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
	STL_LID			lid;		// destination lid
    uint16_t        trapnum;    // trap number
    uint32_t        qpn;        // queue pair number - 24 bits
    uint8_t         producer;   // producer of traps/notices - using 3 bits
	uint8_t         rtv;        // time to respond to requests - 5 bits
	uint16_t        pkey;		// original PKey
	uint32_t	    qkey;	    // queue pair key (32)
	STL_LID		    startLid;	// calculated start Lid
	STL_LID		    endLid;		// calculated end Lid
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
