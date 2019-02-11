/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

#ifndef _RMPP_L_H_
#define _RMPP_L_H_

#include <cs_g.h>		/* Global common services functions */
#include <vs_g.h>
#include "ib_const.h"
#include "ib_types.h"
#include "cs_g.h"
#include "cs_hashtable.h"
#include "if3.h"


#define RMPP_MAX_USERS              MAX_MANAGER
#define RMPP_SRV_MAX_RECORD_SZ	    512

struct rmpp_user_info_s;
typedef struct rmpp_cntxt {
	uint64_t	tstamp ;
	uint64_t	tid ;		// Tid for hash table search
	STL_LID		lid ;		// Lid for hash table search
    uint16_t    method;     // initial method requested by initiator
    IBhandle_t	sendFd;     // mai handle to use for sending packets (fd_sa for 1st seg and fd_rmpp_w threafter)
	uint8_t		hashed ;	// Entry is inserted into the hash table
	uint32_t	ref ;		// Reference count for the structure
    uint32_t    reqDataLen; // length of the getMulti request MAD
	char*		reqData ;	// Data pointer for input getMulti MAD
	char*		data ;		// Data pointer for MAD rmpp response
	uint32_t	len ;		// Length of the MAD response
    uint16_t    attribLen;  // num 8-byte words from start of one attrib to start of next
	Mai_t		mad ;
    int         usrId;      // RMPP user ID
    struct rmpp_user_info_s *info;  // RMPP user info
    /* 1.1 related fields */
    uint32_t    WF;         // Window First: segNum that is first packet in current window
    uint32_t    WL;         // Window Last: segNum that is last packet in current window
    uint32_t    NS;         // Next segNum to be tranmitted by Sender
    uint32_t    ES;         // segNum expected next (Receiver side)
    uint16_t    isDS;       // Double sided getMulti in effect
    uint16_t    reqInProg;  // receipt of request in progress
    uint64_t    RespTimeout;// current response timeout value (13.6.3.1)
    uint64_t    tTime;      // total transaction timeout value (13.6.3.2)
    uint32_t    bytesRcvd;  // Number of bytes received so far
    uint16_t	retries;    // retry count
    uint16_t	last_ack;   // last segment number acked
    uint16_t    segTotal;   // total segments in response
    struct rmpp_cntxt *next ;	// Link List next pointer
    struct rmpp_cntxt *prev ;	// Link List prev pointer
    uint8_t     chkSum;     // checksum of rmpp response
	//SACacheEntry_t *cache;  // pointer to cache structure if applicable
    Status_t (*freeDataFunc)(struct rmpp_cntxt *);  // func to call to free data. may
                        	                        // either free locally allocated data, or defer to
                        	                        // the cache mechanism to decref the cache
    Status_t(*processFunc)(Mai_t *, struct rmpp_cntxt *); // function to call
                                                          // to process the incoming packet.
} rmpp_cntxt_t ;

typedef enum {
	ContextAllocated = 1,       // new context allocated
	ContextNotAvailable = 2,    // out of context
	ContextExist = 3,           // general duplicate request
    ContextExistGetMulti = 4    // existing getMult request
} rmpp_context_get_t;
static __inline char *
rmpp_context_get_totext(rmpp_context_get_t status)
{
	switch (status) {
	case ContextAllocated: return "Allocated";
	case ContextNotAvailable: return "NotAvailable";
	case ContextExist: return "Exist";
	case ContextExistGetMulti: return "ExistGetMulti";
	default: return "Unknown";
	}
}

/**
 * @brief This indicates the type of
 * 		the RMPP request being sent.
 * 		The format for SA RMPP packets
 * 		is handled differently from the
 * 		format for VENDOR and other
 * 		RMPP packets. The SA will put its
 * 		header in every RMPP packet whereas
 * 		vendor RMPP packets will only put
 * 		their header in the first RMPP packet.
 * 		This is accomplished by adding the
 * 		header as data before passing it to
 * 		the RMPP library. These values are
 * 		passed into the rmpp_open_cnx()
 * 		function and handled internally
 * 		by the rmpp library.
 */
typedef enum {
	RMPP_FORMAT_UNKNOWN = 0,
	RMPP_FORMAT_SA = 1,
	RMPP_FORMAT_VENDOR = 2
} rmpp_format_t;

extern uint32_t rmpp_sma_spoofing_check_get(void);
extern void rmpp_sma_spoofing_check_set(uint32_t value);
extern int rmpp_is_cnx_open(IBhandle_t *fd);
extern int rmpp_is_cnx_partial_open(int usrId);
extern rmpp_cntxt_t *rmpp_mngr_get_cmd(int usrId, Mai_t * mad, uint8_t * processMad);
extern rmpp_cntxt_t* rmpp_cntxt_get(int usrId, Mai_t *mad, uint8_t *processMad);
extern Status_t rmpp_cntxt_data(int usrId, rmpp_cntxt_t* rmpp_cntxt, void* buf, uint32_t len);
Status_t rmpp_pre_process_request(int usrId, Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt);
extern Status_t rmpp_send_reply(Mai_t *maip, rmpp_cntxt_t* rmpp_cntxt);
extern Status_t rmpp_send_request(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt);
extern Status_t rmpp_receive_response(int usrId, ManagerInfo_t *mi, Mai_t *maip, uint8_t *buffer, uint32_t *bufferLength, CBTxRxFunc_t cb, void *context);
extern Status_t rmpp_cntxt_release(rmpp_cntxt_t *rmpp_cntxt);
extern Status_t rmpp_cntxt_full_release(rmpp_cntxt_t *rmpp_cntxt);
extern void rmpp_mngr_close_cnx(ManagerInfo_t *mi, uint8_t complete);
extern void rmpp_init(void);

extern int rmpp_gsi_check_packet_filter(Mai_t *maip);
extern int rmpp_mngr_open_cnx(
                                 IBhandle_t *fd,
                                 uint32_t qp,
                                 uint32_t dev,
                                 uint32_t port,
                                 Pool_t *pool,
                                 uint32_t data_length,
                                 uint32_t max_cntxt,
                                 IBhandle_t *fhRmppRec,
                                 IBhandle_t *fhRmppTable,
                                 uint8_t mclass,
                                 char * (*get_method_text)(int),
                                 char * (*get_aid_name)(uint16_t, uint16_t),
                                 Status_t(*pre_process_get)(Mai_t *, rmpp_cntxt_t *),
                                 Status_t(*pre_process_response)(Mai_t *, rmpp_cntxt_t *),
                                 uint8_t (*rmpp_is_master)(void),
                                 uint32_t vieo_mod_id,
                                 char *wtName
                             );

extern void rmpp_close_cnx(int usrId, uint8_t complete);
extern int rmpp_open_cnx(
                                 IBhandle_t *fd,
                                 uint32_t qp,
                                 uint32_t dev,
                                 uint32_t port,
                                 Pool_t *pool,
                                 uint32_t data_length,
                                 uint32_t max_cntxt,
                                 IBhandle_t *fh_req_get,
                                 IBhandle_t *fh_req_gettable,
                                 uint8_t mclass,
                                 char* (*get_method_text)(int),
                                 char* (*get_aid_name)(uint16_t, uint16_t),
                                 Status_t(*pre_process_request)(Mai_t *, rmpp_cntxt_t *),
                                 Status_t(*pre_process_response)(Mai_t *, rmpp_cntxt_t *),
                                 uint8_t(*rmpp_is_master)(void),
                                 uint32_t vieo_mod_id,
                                 uint32_t rmpp_format,
                                 char *wtName
                             );

extern Status_t rmpp_receive_request(IBhandle_t fd, Mai_t *in_mad);
extern void rmpp_prepare_response(rmpp_cntxt_t *rmpp_cntxt);
extern Status_t rmpp_send_reply_EA(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt);

#endif
