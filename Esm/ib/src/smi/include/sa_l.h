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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									     //
// FILE NAME								     //
//    sa_l.h								     //
//									     //
// DESCRIPTION								     //
//    This file contains the localized SA definitions.  These definitions    //
//    do NOT appear in the IBTA specs.  They are implementation dependent.   //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    None								     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//    ib_const.h							     //
//    sm_pool.h								     //
//    sm_event.h							     //
//    sm_thread.h							     //
//									     //
//									     //
//===========================================================================//

#ifndef	_SA_L_H_
#define	_SA_L_H_

#include "ib_const.h"
#include "ib_types.h"
#include "cs_g.h"
#include "cs_hashtable.h"

// JSY - this is temporary
#define	mai_poll(FD, MAIP)	mai_recv(FD, MAIP, 1)

/* Fake portGuid to use when auto-creating the multicast group */
#define SA_FAKE_MULTICAST_GROUP_MEMBER 0x00066A00FACADE01ull

//
// SA capability mask defines
//
#define SA_CAPABILITY_OPTIONAL_RECORDS      0x0100
#define SA_CAPABILITY_MULTICAST_SUPPORT     0x0200
#define SA_CAPABILITY_MULTIPATH_SUPPORT     0x0400
#define SA_CAPABILITY_REINIT_SUPPORT        0x0800
#define	SA_CAPABILITY_PORTINFO_CAPMASK_MATCH 0x2000		
#define SA_CAPABILITY_PA_SERVICES_SUPPORT   0x8000

#define SA_CAPABILITY2_QOS_SUPPORT			0x0002
#define SA_CAPABILITY2_MFTTOP_SUPPORT		0x0008
#define SA_CAPABILITY2_FULL_PORTINFO		0x0040
#define SA_CAPABILITY2_EXT_SUPPORT			0x0080

//
//	This struct is for the queued MAD requests to the SA>
//
#define SA_MADS         256

typedef struct {
	Mai_t		mad;
	uint64_t	timer;
} SAQueue_t;

#define	SA_Queue_Empty()	(sarp == sawp)
#define	SA_Queue_Full()		(((sarp+1) == sawp) || 				\
				((sarp == &sa_queue[SA_MADS-1]) && 		\
				(sawp == &sa_queue[0])))
#define	SA_Queue_Inc(QP)	((QP) != &sa_queue[SA_MADS - 1]) ?		\
				(QP)+1 : &sa_queue[0]


//
//	A frequently used debugging macro.
//
#define	SAPrint(HDR, MAIP)							\
	printf("%s: %02x %04x %08x %016Lx -> %02x\n", (HDR),			\
		(MAIP)->base.method, 						\
		(MAIP)->base.aid, 						\
		(MAIP)->base.amod, 						\
		(MAIP)->base.tid, 						\
		(MAIP)->base.status);

//
//	Comparison mask definitions.
//
//      These arrays contain the number of bytes to compare for the template
//      query case.
//
typedef struct _FieldMask {
	uint16_t	offset;			// offset from 'bit 0'
	uint16_t	length;			// number of bits in field
} FieldMask_t;

//
//      Scratch pad for template queries.
//
extern	uint8_t         template_mask[4096];
extern	uint32_t        template_offset;
extern	uint32_t        template_length;
extern	FieldMask_t     *template_fieldp;

//
// SA Caching
//

#define SA_NUM_CACHES 2         // number of cached query types

#define SA_CACHE_FI_NODES     0 // all NodeRecords where NodeType == FI
#define SA_CACHE_SWITCH_NODES 1 // all NodeRecords where NodeType == Switch

#define SA_CACHE_CLEAN_INTERVAL 30*VTIMER_1S // time between clearing of the
                                       // "previous" list of in-use elements
								   
#define SA_CACHE_NAME_LEN 16 // max length of cache name

// represents a cache containing the response data for a specific SA query
typedef struct SACacheEntry {
	char     name[SA_CACHE_NAME_LEN]; // name of cache for debugging
	uint8_t  valid;     // true if cache contains data
	uint8_t  transient; // marks a temporary cache structure
	uint8_t  *data;     // wire-ready cached data
	uint32_t len;       // length of cached data
	uint32_t refCount;  // # of outstanding references to this cache
	uint32_t records;   // number of records represented by this cache
	struct SACacheEntry *next; // used for "previous" list
} SACacheEntry_t;

// consolidation of cached data sets into a single structure
typedef struct {
	SACacheEntry_t *current[SA_NUM_CACHES]; // array of current caches
	SACacheEntry_t *build[SA_NUM_CACHES]; // array of caches being built in new topology
	SACacheEntry_t *previous; // linked list of all previous caches still in
	                          // use.  should be minimal as all queries using
	                          // the caches should finish between sweeps
	Lock_t         lock; // lock mediating topology and query threads
} SACache_t;

// prototype of function to be called to build each cache
typedef Status_t (*SACacheBuildFunc_t)(SACacheEntry_t *, Topology_t *);

//
//	Authentication structure.
//
typedef	struct {
	Lid_t		lid;		// 16 bit Lid
	Node_t		*nodep;		// node for this Lid
	Port_t		*portp;		// port on the node for this Lid
	PKey_t		pKeys[SM_PKEYS];	// PKeys for comparison
	uint32_t	numKeys;
} Authenticator_t;

#define SA_CNTXT_HASH_TABLE_DEPTH	64
extern uint32_t sa_max_cntxt;
extern uint32_t sa_data_length;	// maximum SA response size
extern uint32_t sa_max_path_records;// maximum path records in one response
#define SA_CNTXT_MAX_GUIDINFO		256
#define SA_CNTXT_MAX_STANDARD		256
typedef struct sa_cntxt {
	uint64_t	tstamp ;
	uint64_t	tid ;		// Tid for hash table search
	Lid_t		lid ;		// Lid for hash table search
    uint16_t    method;     // initial method requested by initiator
    IBhandle_t	sendFd;     // mai handle to use for sending packets (fd_sa for 1st seg and fd_sa_w threafter)
	uint8_t		hashed ;	// Entry is inserted into the hash table
	uint32_t	ref ;		// Reference count for the structure
    uint32_t    reqDataLen; // length of the getMulti request MAD
	char*		reqData ;	// Data pointer for input getMulti MAD
	char*		data ;		// Data pointer for MAD rmpp response
	uint32_t	len ;		// Length of the MAD response
    uint16_t    attribLen;  // num 8-byte words from start of one attrib to start of next
	Mai_t		mad ;
    /* 1.1 related fields */
    uint32_t    WF;         // Window First: segNum that is first packet in current window
    uint32_t    WL;         // Window Last: segNum that is last packet in current window
    uint32_t    NS;         // Next segNum to be tranmitted by Sender
    uint32_t    ES;         // segNum expected next (Receiver side)
    uint16_t    isDS;       // Double sided getMulti in effect
    uint16_t    reqInProg;  // receipt of request in progress
    uint64_t    RespTimeout;// current response timeout value (13.6.3.1)
    uint64_t    tTime;      // total transaction timeout value (13.6.3.2)
	uint16_t	retries;    // retry count
	uint16_t	last_ack;   // last segment number acked
    uint16_t    segTotal;   // total segments in response
	struct sa_cntxt *next ;	// Link List next pointer
	struct sa_cntxt *prev ;	// Link List prev pointer
    uint8_t     chkSum;     // checksum of rmpp response 
	SACacheEntry_t *cache;  // pointer to cache structure if applicable
	Status_t (*freeDataFunc)(struct sa_cntxt *); // func to call to free data. may
	                        // either free locally allocated data, or defer to
	                        // the cache mechanism to decref the cache
	Status_t (*processFunc)(Mai_t *, struct sa_cntxt *); // function to call
	                        // to process the incoming packet.
} sa_cntxt_t ;

//
//	Macros for SA filter creation.
//
#define	SA_Filter_Init(FILTERP) {					\
	Filter_Init(FILTERP, 0, 0);					\
									\
	(FILTERP)->active |= MAI_ACT_ADDRINFO;				\
	(FILTERP)->active |= MAI_ACT_BASE;				\
	(FILTERP)->active |= MAI_ACT_TYPE;				\
	(FILTERP)->active |= MAI_ACT_DATA;				\
	(FILTERP)->active |= MAI_ACT_DEV;				\
	(FILTERP)->active |= MAI_ACT_PORT;				\
	(FILTERP)->active |= MAI_ACT_QP;				\
	(FILTERP)->active |= MAI_ACT_FMASK;				\
									\
	(FILTERP)->type = MAI_TYPE_EXTERNAL;					\
	(FILTERP)->type = MAI_TYPE_ANY;	/* JSY - temp fix for CAL */	\
									\
	(FILTERP)->dev = sm_config.hca;					\
	(FILTERP)->port = (sm_config.port == 0) ? MAI_TYPE_ANY : sm_config.port;	\
	(FILTERP)->qp = 1;						\
}

typedef enum {
	ContextAllocated = 1,       // new context allocated
	ContextNotAvailable = 2,    // out of context
	ContextExist = 3,           // general duplicate request
    ContextExistGetMulti = 4    // existing getMult request
} SAContextGet_t;

//
// Rate macros - these allow us to compare rates easily
//
#define linkrate_eq(r1, r2) ((r1) == (r2))
#define linkrate_ne(r1, r2) ((r1) != (r2))
#define linkrate_lt(r1, r2) (linkrate_to_ordinal(r1) < linkrate_to_ordinal(r2))
#define linkrate_le(r1, r2) (linkrate_to_ordinal(r1) <= linkrate_to_ordinal(r2))
#define linkrate_gt(r1, r2) (linkrate_to_ordinal(r1) > linkrate_to_ordinal(r2))
#define linkrate_ge(r1, r2) (linkrate_to_ordinal(r1) >= linkrate_to_ordinal(r2))

// This function maps an IBTA linkrate value to an integer for the sake of
// comparing rates
static __inline__ uint32_t linkrate_to_ordinal(uint32_t rate) {
	// We map these to 10 times the actual rate so that in the future, if
	// new rates get added, we don't have to shuffle any of the existing values
	switch (rate) {
	 case IB_STATIC_RATE_2_5G: return 25;
	 case IB_STATIC_RATE_5G:   return 50;
	 case IB_STATIC_RATE_10G:  return 100;
	 case IB_STATIC_RATE_14G:  return 125; // STL_STATIC_RATE_12_5G
	 case IB_STATIC_RATE_20G:  return 200;
	 case IB_STATIC_RATE_25G:  return 250;
	 case IB_STATIC_RATE_30G:  return 300;
	 case IB_STATIC_RATE_40G:  return 375; // STL_STATIC_RATE_37_5G
	 case IB_STATIC_RATE_56G:  return 500; // STL_STATIC_RATE_50G
	 case IB_STATIC_RATE_60G:  return 600;
	 case IB_STATIC_RATE_80G:  return 750; // STL_STATIC_RATE_75G
	 case IB_STATIC_RATE_100G: return 1040;
	 case IB_STATIC_RATE_112G: return 1120;
	 case IB_STATIC_RATE_120G: return 1200;
	 case IB_STATIC_RATE_168G: return 1680;
	 case IB_STATIC_RATE_200G: return 2080;
	 case IB_STATIC_RATE_300G: return 3120;
	 default:             return 25;
	}
	return 25;
}

// iterators to help construct Path Record responses
typedef struct lid_iterator_s {
	uint16	src_start;
	uint16	src_endp1;
	uint16	src_lmc_mask;
	uint16	dst_start;
	uint16	dst_endp1;
	uint16	dst_lmc_mask;
	uint16	slid;
	uint16	dlid;
	uint8	phase;
	uint8	end_phase;
} lid_iterator_t;

// these are defined in fm_xml.h
//#define PATH_MODE_MINIMAL	  0	// no more than 1 path per lid
//#define PATH_MODE_PAIRWISE  1	// cover every lid on "bigger side" exactly once
//#define PATH_MODE_ORDERALL  2	// PAIRWISE, then all src, all dst (skip dups)
//#define PATH_MODE_SRCDSTALL 3	// all src, all dst

void lid_iterator_init(lid_iterator_t *iter,
				Port_t *src_portp, uint16 src_start_lid, uint16 src_lid_len,
				Port_t *dst_portp, uint16 dst_start_lid, uint16 dst_lid_len,
				int path_mode,
			   	uint16 *slid, uint16 *dlid);
int lid_iterator_done(lid_iterator_t *iter);
void lid_iterator_next(lid_iterator_t *iter, uint16 *slid, uint16 *dlid);
// This handles 1 fixed lid, 1 GID or wildcard style queries.
// src is the fixed side, dst is the iterated side.
// doesn't really matter if src is source or dest and visa versa
void lid_iterator_init1(lid_iterator_t *iter,
				Port_t *src_portp, uint16 slid, uint16 src_lid_start, uint16 src_lid_len,
				Port_t *dst_portp, uint16 dst_lid_start, uint16 dst_lid_len,
			   	int path_mode,
			   	uint16 *dlid);
int lid_iterator_done1(lid_iterator_t *iter);
void lid_iterator_next1(lid_iterator_t *iter, uint16 *dlid);

//
//	External declarations.
//
extern SubscriberTable_t   saSubscribers;
extern ServiceRecTable_t   saServiceRecords;
extern  uint32_t    saDebugRmpp;    // controls output of SA INFO RMPP+ messages
extern uint32_t     saRmppCheckSum; // rmppp response checksum control
extern	uint8_t		*sa_data;
extern SACache_t	saCache;
extern SACacheBuildFunc_t	saCacheBuildFunctions[];	

extern  Status_t    sa_SubscriberInit(void);
extern  void        sa_SubscriberDelete(void);
extern  void        sa_SubscriberClear(void);
extern  Status_t    sa_ServiceRecInit(void);
extern  void        sa_ServiceRecDelete(void);
extern  void        sa_ServiceRecClear(void);
extern  Status_t    sa_McGroupInit(void);
extern  void        sa_McGroupDelete(void);
extern	Status_t	sa_Authenticate_Path(Lid_t, Lid_t);
extern	Status_t	sa_Authenticate_Access(uint32_t, Lid_t, Lid_t, Lid_t);
extern	Status_t	sa_Compare_Node_Port_PKeys(Node_t*, Port_t*);
extern	Status_t	sa_Compare_Port_PKeys(Port_t*, Port_t*);
extern  uint32_t    saDebugPerf;  // control SA performance messages; default is off

extern	Status_t	sa_data_offset(uint16_t class, uint16_t type);
extern	Status_t	sa_create_template_mask(uint16_t, uint64_t);
extern	Status_t	sa_template_test_mask(uint64_t, uint8_t *, uint8_t **, uint32_t, uint32_t, uint32_t *);
extern	Status_t	sa_template_test(uint8_t *, uint8_t **, uint32_t, uint32_t, uint32_t *);
extern  Status_t    sa_template_test_noinc(uint8_t *, uint8_t *, uint32_t);
extern  void        sa_increment_and_pad(uint8_t **, uint32_t, uint32_t, uint32_t *);
extern  Status_t    sa_check_len(uint8_t *, uint32_t, uint32_t);

extern  Status_t    sa_getMulti_resend_ack(sa_cntxt_t *);
extern	Status_t	sa_send_reply(Mai_t *, sa_cntxt_t*);
extern	Status_t	sa_send_multi(Mai_t *, sa_cntxt_t*);
extern	Status_t	sa_send_single(Mai_t *, sa_cntxt_t*);

extern	Status_t	sa_ClassPortInfo(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_InformInfo(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_InformRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_LFTableRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_LinkRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_McMemberRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_MFTableRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_NodeRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_PartitionRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_PathRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_PortInfoRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_SAResponse(Mai_t *, sa_cntxt_t* );
extern  Status_t	sa_SCSCTableRecord(Mai_t *, sa_cntxt_t* );
extern  Status_t	sa_SLSCTableRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_SCSLTableRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_SCVLTableRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_SMInfoRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_ServiceRecord(Mai_t *, sa_cntxt_t* );
extern  Status_t    sa_ServiceRecord_Age(uint32_t *);
extern	Status_t	sa_SwitchInfoRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_Trap(Mai_t *);
extern	Status_t	sa_VLArbitrationRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_TraceRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_VFabricRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_JobManagement(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_FabricInfoRecord(Mai_t *, sa_cntxt_t* );
extern	Status_t	sa_QuarantinedNodeRecord(Mai_t *, sa_cntxt_t* );
extern  Status_t	sa_CongInfoRecord(Mai_t *, sa_cntxt_t* );
extern 	Status_t	sa_CableInfoRecord(Mai_t *, sa_cntxt_t* );
extern  Status_t	sa_SwitchCongRecord(Mai_t *, sa_cntxt_t* );
extern  Status_t	sa_SwitchPortCongRecord(Mai_t *, sa_cntxt_t* );
extern  Status_t	sa_HFICongRecord(Mai_t *, sa_cntxt_t* );
extern  Status_t	sa_HFICongCtrlRecord(Mai_t *, sa_cntxt_t* );
extern  Status_t	sa_BufferControlTableRecord(Mai_t *, sa_cntxt_t* );
extern  Status_t	sa_PortGroupRecord(Mai_t *, sa_cntxt_t* );
extern  Status_t	sa_PortGroupFwdRecord(Mai_t *, sa_cntxt_t* );

extern	Status_t	sa_NodeRecord_BuildCACache(SACacheEntry_t *, Topology_t *);
extern	Status_t	sa_NodeRecord_BuildSwitchCache(SACacheEntry_t *, Topology_t *);

extern	Status_t	pathrecord_userexit(uint8_t *, uint32_t *);
extern	Status_t	multipathrecord_userexit(uint8_t *, uint32_t *);
extern	Status_t	servicerecord_userexit(Mai_t *);
extern	Status_t	tracerecord_userexit(uint8_t *, uint32_t *);

extern 	Status_t	sa_process_getmulti(Mai_t *, sa_cntxt_t*);
extern 	Status_t	sa_process_mad(Mai_t *, sa_cntxt_t*);
extern  Status_t    sa_process_inflight_rmpp_request(Mai_t *, sa_cntxt_t*);
extern  void		sa_main_reader(uint32_t, uint8_t **);
extern  void		sa_main_writer(uint32_t argc, uint8_t ** argv);
extern  void        sa_cntxt_age(void);
extern  sa_cntxt_t* sa_cntxt_find( Mai_t* );
extern	SAContextGet_t	sa_cntxt_get( Mai_t*, void**);
extern	Status_t   	sa_cntxt_release( sa_cntxt_t* );
extern 	Status_t	sa_cntxt_reserve( sa_cntxt_t* );
extern  Status_t	sa_cntxt_data( sa_cntxt_t*, void*, uint32_t );
extern  Status_t	sa_cntxt_data_cached(sa_cntxt_t *, void *, uint32_t, SACacheEntry_t *);
extern  void        sa_cntxt_clear(void);
extern  Status_t	sa_SetDefBcGrp( void );

extern  Status_t	sa_cache_init(void);
extern  Status_t	sa_cache_alloc_transient(SACacheEntry_t *[], int, SACacheEntry_t **);
extern  Status_t	sa_cache_get(int, SACacheEntry_t **);
extern  void		sa_cache_clean(void);
extern  Status_t	sa_cache_release(SACacheEntry_t *);
extern	Status_t    sa_cache_cntxt_free(sa_cntxt_t *);

extern  char *      sa_getMethodText(int method);
extern  char *      sa_getAidName(uint16_t aid);

extern	void		dumpGid(IB_GID gid);
extern	void		dumpGuid(Guid_t guid);
extern	void		dumpBytes(uint8_t * bytes, size_t num_bytes);
extern	void		dumpHWords(uint16_t * hwords, size_t num_hwords);
extern	void		dumpWords(uint32_t * words, size_t num_words);
extern	void		dumpGuids(Guid_t * guids, size_t num_guids);  

extern STL_SERVICE_RECORD* getNextService(uint64_t *serviceId, IB_GID *serviceGid, uint16_t *servicep_key, STL_SERVICE_RECORD *pSrp);
extern STL_SERVICE_RECORD* getService(uint64_t *serviceId, IB_GID *serviceGid, uint16_t *servicep_key, STL_SERVICE_RECORD *pSrp);

extern McGroup_t*	getBroadCastGroup(IB_GID * pGid, McGroup_t *pGroup);
extern McGroup_t*	getNextBroadCastGroup(IB_GID * pGid, McGroup_t *pGroup);
extern McMember_t*	getBroadCastGroupMember(IB_GID * pGid, uint32_t index, McMember_t *pMember);
extern McMember_t*	getNextBroadCastGroupMember(IB_GID * pGid, uint32_t *index, McMember_t *pMember);
extern Status_t     clearBroadcastGroups(int);  // TRUE=recreate broadcast group

#endif	// _SA_L_H_

