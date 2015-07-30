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
#ifndef	_PA_L_H_
#define	_PA_L_H_

#include "ib_const.h"
#include "ib_types.h"
#include "cs_g.h"
#include "cs_hashtable.h"


#define PA_SRV_MAX_RECORD_SZ	512

typedef struct pa_cntxt {
	uint64_t	tstamp ;
	uint64_t	tid ;		// Tid for hash table search
	uint16_t	lid ;		// Lid for hash table search
    uint16_t    method;     // initial method requested by initiator
    IBhandle_t	sendFd;     // mai handle to use for sending packets (fd_sa for 1st seg and fd_pa_w threafter)
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
	struct pa_cntxt *next ;	// Link List next pointer
	struct pa_cntxt *prev ;	// Link List prev pointer
    uint8_t     chkSum;     // checksum of rmpp response 
	//SACacheEntry_t *cache;  // pointer to cache structure if applicable
	Status_t (*freeDataFunc)(struct pa_cntxt *); // func to call to free data. may
	                        // either free locally allocated data, or defer to
	                        // the cache mechanism to decref the cache
	//Status_t (*processFunc)(Mai_t *, struct pa_cntxt *); // function to call
	                        // to process the incoming packet.
} pa_cntxt_t ;

typedef enum {
	ContextAllocated = 1,       // new context allocated
	ContextNotAvailable = 2,    // out of context
	ContextExist = 3,           // general duplicate request
    ContextExistGetMulti = 4    // existing getMult request
} PAContextGet_t;

extern uint32_t pa_max_cntxt;
extern uint32_t pa_data_length;

extern void pa_delete_filters(void);
extern Status_t pa_process_mad(Mai_t *maip, pa_cntxt_t* sa_cntxt);
extern pa_cntxt_t * pa_mngr_get_cmd(Mai_t * mad, uint8_t * processMad);
extern Status_t pa_mngr_open_cnx(void);
extern void pa_mngr_close_cnx(void);
extern void pa_main_kill(void);

#endif	// _PA_L_H_

