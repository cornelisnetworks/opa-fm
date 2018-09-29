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

#if defined(__VXWORKS__) || defined(__LINUX__)
#include <ib_types.h>
#include <ib_mad.h>
#include <ib_status.h>
#include <cs_g.h>
#include <mai_g.h>
#include <ib_sa.h>
#include <ib_mad.h>
#include <ib_macros.h>
#include <if3.h>
#include "if3_def.h"
#include "rmpp_l.h"
#include "vs_g.h"
#include "vfi_g.h"
#include "rmpp_counters.h"
#include "iba/ib_rmpp.h"
#include "iba/stl_rmpp.h"
#include <fm_xml.h>

#ifdef __LINUX__
#define static
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#endif

#ifdef __VXWORKS__
#include <stdio.h>
#endif

#undef LOCAL_MOD_ID
#define LOCAL_MOD_ID    VIEO_IF3_MOD_ID

#define RMPP_NO_RRESP_TIME_VALUE 0x1f

extern uint32_t if3DebugRmpp; 
extern int smValidateGsiMadPKey(Mai_t *maip, uint8_t mgmntAllowedRequired, uint8_t antiSpoof);
extern Status_t if3_set_rmpp_minfo(ManagerInfo_t *mi);

#if defined(__VXWORKS__)
#define RMPP_STACK_SIZE (24 * 1024)
#else
#define RMPP_STACK_SIZE (256 * 1024)
#endif

#define RMPP_CNTXT_HASH_TABLE_DEPTH	64

#define	INCR_RMPP_CNTXT_NFREE() {  \
    if (info->rmpp_cntxt_nfree < info->rmpp_max_cntxt) {        \
        info->rmpp_cntxt_nfree++;        \
		SET_RMPP_PEAK_COUNTER(rmppMaxContextsFree, info->rmpp_cntxt_nfree); \
    } else {                      \
		IB_LOG_ERROR_FMT(__func__, "rmpp_cntxt_nfree already at max: %d", info->rmpp_cntxt_nfree);\
    }\
}

#define	DECR_RMPP_CNTXT_NFREE() {  \
    if (info->rmpp_cntxt_nfree) {        \
        info->rmpp_cntxt_nfree--;        \
    } else {                      \
		IB_LOG_ERROR_FMT(__func__, "can't decrement rmpp_cntxt_nfree, already zero");\
    }\
}

#define	INCR_RMPP_CNTXT_NALLOC() {  \
    if (info->rmpp_cntxt_nalloc < info->rmpp_max_cntxt) {        \
        info->rmpp_cntxt_nalloc++;        \
		SET_RMPP_PEAK_COUNTER(rmppMaxContextsInUse, info->rmpp_cntxt_nalloc); \
    } else {                      \
		IB_LOG_ERROR_FMT(__func__, "rmpp_cntxt_nalloc already at max: %d", info->rmpp_cntxt_nalloc);\
    }\
}

#define	DECR_RMPP_CNTXT_NALLOC() {  \
    if (info->rmpp_cntxt_nalloc) {        \
        info->rmpp_cntxt_nalloc--;        \
    } else {                      \
		IB_LOG_ERROR_FMT(__func__, "can't decrement rmpp_cntxt_nalloc, already zero");\
    }\
}

#define rmpp_cntxt_delete_entry( list, entry )	\
	if( (list) == (entry) ) (list) = (list)->next ; \
	else { \
		(entry)->prev->next = (entry)->next ; \
		if( (entry)->next ) (entry)->next->prev = (entry)->prev ; \
	} \
	(entry)->next = (entry)->prev = NULL 

#define rmpp_cntxt_insert_head( list, entry ) 	\
	if( (list) ) (list)->prev = (entry) ;	\
	(entry)->next = (list) ;		\
	(list) = (entry) ;  \
	(entry)->prev = NULL ;

#define	RMPP_Filter_Init(FILTERP) {					\
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
	(FILTERP)->dev = info->dev;					\
	(FILTERP)->port = (info->port == 0) ? MAI_TYPE_ANY : info->port;	\
	(FILTERP)->qp = info->qp; /*1;*/						\
}

typedef struct _FieldMask {
   uint16_t	offset;         // offset from 'bit 0'
   uint16_t	length;         // number of bits in field
} FieldMask_t; 

typedef struct rmpp_user_info_s {
   int usrId; 
   int inuse; 
   int partial_close; 
   // logging related fields
   uint32_t vieo_mod_id; 
   // IB related fields
   IBhandle_t *fd; 
   uint8_t mclass; 
   uint32_t qp; 
   uint32_t dev; 
   uint32_t port; 
   // filter related fields
   IBhandle_t *fh_req_get; 
   IBhandle_t *fh_req_gettable; 
   // pool related fields
   Pool_t *rmpp_pool; 
   uint8_t *rmpp_data; 
   // writer thread related fields
   IBhandle_t rmpp_fd_w; 
   Thread_t wt_thrd; 
   uint8_t wt_thrd_exit; 
   char wt_thrd_name[50]; 
   // context related fields
   Lock_t rmpp_cntxt_lock; 
   int rmpp_cntxt_nalloc; 
   int rmpp_cntxt_nfree; 
   uint32_t rmpp_data_length;
   uint32_t rmpp_max_cntxt;
   rmpp_cntxt_t *rmpp_cntxt_pool; 
   rmpp_cntxt_t *rmpp_cntxt_free_list; 
   rmpp_cntxt_t *rmpp_hash[RMPP_CNTXT_HASH_TABLE_DEPTH]; 
   // callback related fields
   char* (*rmpp_get_method_text)(int); 
   char* (*rmpp_get_aid_name)(uint16_t, uint16_t); 
   Status_t(*rmpp_pre_process_request)(Mai_t *, rmpp_cntxt_t *); 
   Status_t(*rmpp_pre_process_response)(Mai_t *, rmpp_cntxt_t *); 
   uint8_t(*rmpp_is_master)(void); 
   // TBD - template fields initialized, but not used
   uint16_t template_type; 
   uint32_t template_length; 
   FieldMask_t	*template_fieldp;
   uint32_t rmpp_format;
} rmpp_user_info_t; 


uint32_t rmpp_sma_spoofing_check = 0;

static Status_t rmpp_process_response(int usrId, Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt); 
static Status_t rmpp_process_getmulti_response(Mai_t *maip, rmpp_cntxt_t* rmpp_cntxt);

static Lock_t rmpp_user_lock; 
static uint8_t rmpp_initialized = 0; 
static uint8_t rmpp_userCount = 0; 
static uint8_t rmpp_packetLifetime = 18; 
static uint8_t rmpp_respTimeValue = 18; 
static uint64_t rmpp_reqTimeToLive = 0; 
static uint32_t rmppMaxRetries = 3;   // max number of retries for failed receives
static uint64_t	rmpp_timeLastAged = 0; 
static uint8_t g_usrId; 
static uint8_t *argv[10]; 
static uint32_t rmppCheckSum = 0;  // control whether to checksum each rmpp response at start and end of transfer
static rmpp_cntxt_t ignoreCntxt = { 0 }; 
static rmpp_user_info_t rmpp_users[RMPP_MAX_USERS] = { { 0 }, { 0 } }; 

static FieldMask_t recordFieldMask[] = { 
   {     0,    64 }, 
   {     0,     0 }, 
}; 

static FieldMask_t tableRecordFieldMask[] = { 
   {     0,    64 }, 
   {    64,    512 }, 
   {     0,     0 }, 
}; 



static void rmpp_main_writer(uint32_t argc, uint8_t **argv); 
static Status_t rmpp_send_single_vendor(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt);
static Status_t rmpp_send_multi_vendor(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt);
static Status_t rmpp_send_single_sa(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt);
static Status_t rmpp_send_multi_sa(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt);
static Status_t rmpp_process_multi_response(Mai_t *maip,
					    rmpp_cntxt_t *rmpp_cntxt);


static void
rmpp_call_user_callback(rmpp_cntxt_t *rmpp_cntxt, CBTxRxFunc_t cb, void *context)
{
    CBTxRxData_t s;

    if (cb == NULL || context == NULL)
        return;

    // the user is using a call back to handle data reception
    s.msize  = rmpp_cntxt->reqDataLen;
    s.offset = 0;
    s.data   = (uint8_t *)rmpp_cntxt->reqData;
    s.dlen   = rmpp_cntxt->reqDataLen;
    s.dir    = CB_RX_DIR;
    cb(&s,context);

    // adjust the data length of data
    rmpp_cntxt->reqDataLen = s.dlen;
}

static rmpp_user_info_t* rmpp_get_userinfo(int usrId) 
{ 
   if (usrId >= 1 && usrId <= RMPP_MAX_USERS) 
      return &rmpp_users[usrId - 1]; 
   else 
      return NULL;
}

static rmpp_user_info_t* rmpp_mclass_get_userinfo(uint8_t mclass) 
{ 
   int i; 
   rmpp_user_info_t *info = NULL; 
   
   for (i = 0; i < RMPP_MAX_USERS; i++) {
      rmpp_user_info_t *usr = &rmpp_users[i]; 
      
      if (usr->inuse && usr->mclass == mclass) {
         info = usr; 
         break;
      }
   }
   
   return info;
}

// TBD - template_* not used
static Status_t
rmpp_data_offset(rmpp_user_info_t *info, uint16_t type) 
{ 
   
   IB_ENTER(__func__, type, 0, 0, 0); 
   
   if (!info) {
      IB_LOG_ERROR_FMT(__func__, "user info is NULL!"); 
      return VSTATUS_BAD;
   }
   
   info->template_type = type; 
   info->template_fieldp = NULL; 
   
   //	create the mask for the comparisons.
   switch (type) {
   case RMPP_CMD_GET:
      info->template_length = RMPP_RECORD_NSIZE; 
      info->template_fieldp = recordFieldMask; 
      break; 
   case RMPP_CMD_GETTABLE:
      info->template_length = RMPP_TABLE_RECORD_NSIZE; 
      info->template_fieldp = tableRecordFieldMask; 
      break;
   }
   
   if (info->template_fieldp != NULL) {
      IB_EXIT(__func__, VSTATUS_OK); 
      return (VSTATUS_OK);
   } else {
      IB_EXIT(__func__, VSTATUS_BAD); 
      return (VSTATUS_BAD);
   }
}

static Status_t
rmpp_cntxt_free_data(rmpp_cntxt_t *cntxt) 
{ 
   rmpp_user_info_t *info = NULL; 
   
   IB_ENTER(__func__, cntxt, 0, 0, 0); 
   
   if (!cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return VSTATUS_BAD;
   } else if (!(info = (rmpp_user_info_t *)cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", cntxt->usrId); 
      return VSTATUS_BAD;
   }
   
   if (cntxt->data) {
      if (info->rmpp_pool) {
         vs_pool_free(info->rmpp_pool, cntxt->data); 
      } else {
         IB_LOG_WARN_FMT(__func__,
			 "Free failed, invalid pool received in %s[%s] request from LID [0x%x], TID ["FMT_U64"]!",
			 info->rmpp_get_method_text((int)cntxt->method),
			 info->rmpp_get_aid_name((int)cntxt->mad.base.mclass, (int)cntxt->mad.base.aid),
			 cntxt->lid, cntxt->tid); 
         IB_EXIT(__func__, VSTATUS_BAD); 
         return (VSTATUS_BAD);
      }
   }
   
   IB_EXIT(__func__, VSTATUS_OK); 
      return VSTATUS_OK;
}

static Status_t
rmpp_validate_response_mad(rmpp_user_info_t *info, Mai_t *maip) 
{ 
   Status_t rc = VSTATUS_OK; 
   
   IB_ENTER(__func__, maip, 0, 0, 0); 
   
   if (!info) {
      IB_LOG_ERROR_FMT(__func__, "user info is NULL!"); 
      return VSTATUS_BAD;
   }
   
   if (maip->base.cversion > STL_SA_CLASS_VERSION) {
      IB_LOG_WARN_FMT(__func__, 
                      "Invalid Class Version %d received in %s[%s] request from LID [0x%x], TID ["FMT_U64"], ignoring request!", 
                      maip->base.cversion, info->rmpp_get_method_text((int)maip->base.method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.slid, maip->base.tid); 
      rc = VSTATUS_BAD;
   } else if (maip->base.mclass != MAD_CV_VENDOR_DBSYNC) {
      //  drop unsupported MADs
      switch (maip->base.method) {
      case RMPP_CMD_GET:
      case RMPP_CMD_GETTABLE:
         if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "Unsupported or invalid %s[%s] request from LID [0x%x], TID["FMT_U64"]", 
                                   info->rmpp_get_method_text((int)maip->base.method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
         }
         IB_EXIT(__func__, VSTATUS_OK); 
         rc = VSTATUS_BAD; 
         break; 
      default:
         break;
      } 
   }
   
   IB_EXIT(__func__, rc); 
   return (rc);
}

/*
 * reserving context inserts it into the hash table if !hashed
 * and increments reference count
*/
static Status_t
rmpp_cntxt_reserve(rmpp_cntxt_t *rmpp_cntxt) 
{ 
   int         bucket; 
   Status_t	status; 
   rmpp_user_info_t *info = NULL; 
   
   IB_ENTER(__func__, rmpp_cntxt, 0, 0, 0); 
   
   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return VSTATUS_BAD;
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
      return VSTATUS_BAD;
   }
   
   if ((status = vs_lock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, "Failed to lock context rc: %d", status);
   } else {
      rmpp_cntxt->ref++; 
      if (rmpp_cntxt->hashed == 0) {
         // This context needs to be inserted into the hash table
         bucket = rmpp_cntxt->lid % RMPP_CNTXT_HASH_TABLE_DEPTH; 
         rmpp_cntxt->hashed = 1; 
         vs_time_get(&rmpp_cntxt->tstamp); 
         rmpp_cntxt_insert_head(info->rmpp_hash[bucket], rmpp_cntxt);
      }
      if ((status = vs_unlock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
         IB_LOG_ERROR_FMT(__func__, "Failed to unlock context rc: %d", status);
      }
   }
   
   IB_EXIT(__func__, VSTATUS_OK); 
   return VSTATUS_OK;
}

static void
rmpp_cntxt_retire(rmpp_cntxt_t *lcl_cntxt) 
{ 
   rmpp_user_info_t *info = NULL; 
   
   IB_ENTER(__func__, lcl_cntxt, 0, 0, 0); 
   
   if (!lcl_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return;
   } else if (!(info = (rmpp_user_info_t *)lcl_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", lcl_cntxt->usrId); 
      return;
   }
   
   // release getMulti request buffer if allocated
   if (lcl_cntxt->reqData) {
      vs_pool_free(info->rmpp_pool, lcl_cntxt->reqData);
   }
   // If data was privately allocated, free it
   if (lcl_cntxt->freeDataFunc) {
      (void)lcl_cntxt->freeDataFunc(lcl_cntxt);
   }
   
   // Reset all fields
   lcl_cntxt->usrId = -1; 
   lcl_cntxt->info = NULL; 
   lcl_cntxt->ref = 0; 
   lcl_cntxt->isDS = 0; 
   lcl_cntxt->reqDataLen = 0; 
   lcl_cntxt->reqData = NULL; 
   lcl_cntxt->data = NULL; 
   lcl_cntxt->len = 0; 
   lcl_cntxt->lid = 0; 
   lcl_cntxt->tid = 0; 
   lcl_cntxt->hashed = 0; 
   //lcl_cntxt->cache = NULL;
   lcl_cntxt->freeDataFunc = NULL; 
   
   rmpp_cntxt_insert_head(info->rmpp_cntxt_free_list, lcl_cntxt); 
   INCR_RMPP_CNTXT_NFREE(); 
   DECR_RMPP_CNTXT_NALLOC(); 
   
   IB_EXIT(__func__, 0);
}

Status_t
rmpp_cntxt_release(rmpp_cntxt_t *rmpp_cntxt) 
{ 
   int         bucket; 
   Status_t	status; 
   rmpp_user_info_t *info = NULL; 
   
   IB_ENTER(__func__, rmpp_cntxt, 0, 0, 0); 
   
   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return VSTATUS_BAD;
   } else if (rmpp_cntxt->usrId == -1) {
       return VSTATUS_OK;   // context already released
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
      return VSTATUS_BAD;
   }
   
   if ((status = vs_lock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, "Failed to lock context rc: %d", status);
   } else {
      if (rmpp_cntxt->ref == 0) {
         IB_LOG_INFINI_INFO_FMT(__func__, "reference count is already zero");
      } else {
         --rmpp_cntxt->ref; 
         if (if3DebugRmpp) IB_LOG_INFINI_INFO_FMT(__func__, "rmpp_cntxt->ref=%d", rmpp_cntxt->ref);
         if (rmpp_cntxt->ref == 0) {
            
            // this context needs to be removed from hash
            if (rmpp_cntxt->hashed) {
               bucket = rmpp_cntxt->lid % RMPP_CNTXT_HASH_TABLE_DEPTH; 
               rmpp_cntxt_delete_entry(info->rmpp_hash[bucket], rmpp_cntxt);
            }
            
            rmpp_cntxt->prev = rmpp_cntxt->next = NULL; 
            rmpp_cntxt_retire(rmpp_cntxt);
            
         }
      }
      if ((status = vs_unlock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
         IB_LOG_ERROR_FMT(__func__, "Failed to unlock context rc: %d", status);
      }
   }
   
   IB_EXIT(__func__, VSTATUS_OK); 
   return VSTATUS_OK;
}

Status_t
rmpp_cntxt_full_release(rmpp_cntxt_t *rmpp_cntxt) 
{ 
   rmpp_user_info_t *info = NULL; 
   
   IB_ENTER(__func__, rmpp_cntxt, 0, 0, 0); 
   
   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return VSTATUS_BAD;
   } else if (rmpp_cntxt->usrId == -1) {
       return VSTATUS_OK;   // context already released
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
      return VSTATUS_BAD;
   }

   // if necessary, adjust the context reference counter to ensure that the
   // context will be deallocated
   if (rmpp_cntxt->ref > 1)
       rmpp_cntxt->ref = 1;

   rmpp_cntxt_release(rmpp_cntxt);
   
   IB_EXIT(__func__, VSTATUS_OK); 
   return VSTATUS_OK;
}

Status_t
rmpp_cntxt_data(int usrId, rmpp_cntxt_t *rmpp_cntxt, void *buf, uint32_t len) 
{ 
   Status_t	status; 
   rmpp_user_info_t *info = NULL; 
   
   IB_ENTER(__func__, rmpp_cntxt, buf, len, 0); 
   
   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return VSTATUS_BAD;
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
      return VSTATUS_BAD;
   }
   
   status = VSTATUS_OK; 
   
   rmpp_cntxt->info = info; 
   
   if (!buf || !len) {
      rmpp_cntxt->data = NULL; 
      rmpp_cntxt->len = 0; 
      rmpp_cntxt->freeDataFunc = NULL; 
      goto done;
   }
   
   status = vs_pool_alloc(info->rmpp_pool, len, (void *)&rmpp_cntxt->data); 
   
   if (status == VSTATUS_OK) {
      rmpp_cntxt->len = len; 
      memcpy(rmpp_cntxt->data, buf, len); 
      rmpp_cntxt->freeDataFunc = rmpp_cntxt_free_data;
   } else {
      rmpp_cntxt->len = 0; 
      rmpp_cntxt->data = NULL; /* If data is NULL, rmpp_send_reply will send error response to caller */
      rmpp_cntxt->freeDataFunc = NULL;
   }
   
done:
   IB_EXIT(__func__, status); 
   return status;
}

static Status_t
rmpp_send_single_sa(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{ 
   Status_t	status; 
   STL_SA_MAD mad; 
   rmpp_user_info_t *info = NULL; 
   /* set the mai handle to use for sending - use reader handle PM if no context */
   
   IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0); 
   
   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return VSTATUS_BAD;
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
      return VSTATUS_BAD;
   }
   
   // send the response MAD.
   (void)BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &mad, STL_SA_DATA_LEN);

   mad.header.rmppVersion = 0; 
   mad.header.rmppType = 0; 
   mad.header.u.timeFlag = 0; 
   mad.header.rmppStatus = 0; 
   mad.header.segNum = 0; 
   mad.header.length = 0; 
   
   (void)memset(mad.data, 0, sizeof(mad.data)); 
   if ((rmpp_cntxt == NULL) || (rmpp_cntxt->len != 0 && rmpp_cntxt->data == NULL)) {
      if (maip->base.status == MAD_STATUS_OK) 
         maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
   } else if (rmpp_cntxt->len != 0) {
      if (rmpp_cntxt->len > sizeof(mad.data)) {
         // caller should have checked this, just to be safe
         IB_LOG_ERROR_FMT(__func__, "mad.data=%"PRISZT" len=%d LEN=%d", sizeof(mad.data), rmpp_cntxt->len, IB_SA_DATA_LEN); 
         IB_EXIT(__func__, VSTATUS_OK); 
         return (VSTATUS_OK);
      }
      (void)memcpy(mad.data, rmpp_cntxt->data, rmpp_cntxt->len);
   }
   
   (void)BSWAPCOPY_STL_SA_MAD(&mad, (STL_SA_MAD*)maip->data, STL_SA_DATA_LEN);

   if (if3DebugRmpp) {
      IB_LOG_INFINI_INFO_FMT(__func__, 
                             "sending reply to %s request to LID[0x%x] for TID["FMT_U64"]\n", 
                             info->rmpp_get_method_text(maip->base.method), (int)maip->addrInfo.dlid, maip->base.tid);
   }
   
   if ((status = mai_send(rmpp_cntxt->sendFd, maip)) != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, 
                       "can't send reply to %s request to LID[0x%x] for TID["FMT_U64"]", 
                       info->rmpp_get_method_text(maip->base.method), (int)maip->addrInfo.dlid, maip->base.tid); 
      IB_EXIT(__func__, VSTATUS_OK); 
      return (VSTATUS_OK);
   }
   
   IB_EXIT(__func__, VSTATUS_OK); 
   return (VSTATUS_OK);
}

static Status_t
rmpp_send_multi_sa(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{ 
    int         i; 
    int         wl = 0; 
    uint8_t     chkSum = 0; 
    uint32_t	dlen, datalen = sizeof(SAMadh_t); 
    Status_t	status; 
    STL_SA_MAD  mad; 
    STL_SA_MAD  resp; 
    uint16_t    sendAbort = 0; 
    uint16_t    releaseContext = 1;  /* release context here unless we are in resend mode */
    uint64_t    tnow, delta, ttemp; 
    rmpp_user_info_t *info = NULL; 
    size_t      sa_data_size = IB_SA_DATA_LEN; 
    
    IB_ENTER(__func__, maip, rmpp_cntxt, rmpp_cntxt->len, 0); 
    
    if (!rmpp_cntxt) {
        IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
        return VSTATUS_BAD;
    } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
        IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
        return VSTATUS_BAD;
    }
    
    /*
      * At this point, the maip->addrInfo.slid/dlid and maip->base.method have already 
      * been changed in sa_send_reply to reflect a response packet, so use rmpp_cntxt 
      * lid and method values
     */
    memset((void *)&mad, 0, sizeof(mad)); 
    memset((void *)&resp, 0, sizeof(resp)); 
    
    /* maip is NULL if Ack timeout */
    if (maip) {
        BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER *)maip->data, 
                                    (STL_SA_MAD_HEADER *)&resp); 
        
        if (maip->base.bversion == STL_BASE_VERSION) 
            sa_data_size = MIN(STL_SA_DATA_LEN, rmpp_cntxt->len);
    }
    
    /*
     * 	See if answer to the request coming in
     *  normal getTable requests are not hashed while getMulti's have isDS bit set
     */
    if (rmpp_cntxt->hashed == 0 || rmpp_cntxt->isDS) {
        /* init/save parameters and reserve context */
        if (maip) {
            //INCREMENT_MAD_STATUS_COUNTERS(maip);
            memcpy(&rmpp_cntxt->mad, maip, sizeof(Mai_t)); 
            if (rmpp_cntxt->data) {
                memcpy(rmpp_cntxt->mad.data + SA_FULL_HEADER_SIZE, rmpp_cntxt->data, MIN(sa_data_size, rmpp_cntxt->len));
            }
            
            /* get original mad with appropriate offset */
            (void)BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER *)rmpp_cntxt->mad.data, (STL_SA_MAD_HEADER *)&mad); 
        }
        rmpp_cntxt->WF = 1; 
        /* use value set in getMulti request ACK */
        if (!rmpp_cntxt->isDS) 
            rmpp_cntxt->WL = 1; 
        rmpp_cntxt->NS = rmpp_cntxt->WF;    // Next packet segment to send
        rmpp_cntxt->ES = 0;               // Expected segment number (Receiver only)
        rmpp_cntxt->last_ack = 0;         // last packet acked by receiver
        rmpp_cntxt->retries = 0;          // current retry count
        if (rmpp_cntxt->len == 0) {
            rmpp_cntxt->segTotal = 1;
        } else if (rmpp_cntxt->len <= info->rmpp_data_length) {
            rmpp_cntxt->segTotal = (sa_data_size) ? ((rmpp_cntxt->len + sa_data_size - 1) / sa_data_size) : 1;
        } else {
            IB_LOG_WARN("NO RESOURCES--> rmpp_cntxt->len too large:", rmpp_cntxt->len); 
            //INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
            rmpp_cntxt->len = 0; 
            mad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE; 
            rmpp_cntxt->segTotal = 1; 
            sendAbort = 1;
        }
        
        /* calculate packet and total transaction timeouts C13-13.1.1 */
        rmpp_cntxt->RespTimeout = 4ull * (1 << 20); // ~4.3 seconds
        rmpp_cntxt->tTime = 0;            // receiver only
        ttemp = rmpp_cntxt->mad.intime;   // save the time from original mad in context
        if (rmpp_cntxt->isDS) {
            rmpp_cntxt->mad.intime = ttemp;   // we want the time when getMulti request really started
            rmpp_cntxt->isDS = 0;
        }
        // we must return AID=SA_PATH_RECORD in response to getMultiPathRecord
        if (rmpp_cntxt->mad.base.aid == SA_MULTIPATH_RECORD) {
            rmpp_cntxt->mad.base.aid = SA_PATH_RECORD;
        }
        /* 8-bit cheksum of the rmpp response */
        rmpp_cntxt->chkSum = 0; 
        if (rmppCheckSum && rmpp_cntxt->data) {
            for (i = 0; i < rmpp_cntxt->len; i++) {
                rmpp_cntxt->chkSum += rmpp_cntxt->data[i];
            }
        }
        rmpp_cntxt_reserve(rmpp_cntxt); 
        mad.header.rmppVersion = RMPP_VERSION; 
        mad.header.u.tf.rmppRespTime = 0x1F; // no time provided
        mad.header.offset = rmpp_cntxt->attribLen / 8; // setup attribute offset for RMPP xfer
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "Lid[0x%x] STARTING RMPP %s[%s] with TID="FMT_U64", CHKSUM[%d]", 
                                   (int)rmpp_cntxt->lid, info->rmpp_get_method_text((int)rmpp_cntxt->method), 
                                   (maip?info->rmpp_get_aid_name((int)maip->base.mclass, maip->base.aid):"<null>"), rmpp_cntxt->tid, rmpp_cntxt->chkSum);
        }
    } else if (maip) {
        /* get original mad with appropriate offset */
        (void)BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER *)rmpp_cntxt->mad.data, (STL_SA_MAD_HEADER *)&mad);
         
        /*
           * Check the response and set the context parameters according to it   
           */
        if (resp.header.rmppType == RMPP_TYPE_NOT && (resp.header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE)) {
            // invalid RMPP type
            IB_LOG_WARN_FMT(__func__, 
                            "ABORTING - RMPP protocol error; RmppType is NULL in %s[%s] from Lid[0x%x] for TID="FMT_U64,
                            info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), (int)rmpp_cntxt->lid, rmpp_cntxt->tid); 
            //INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
            sendAbort = 1; 
            mad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
        } else if (resp.header.rmppVersion != RMPP_VERSION) {
            /* ABORT transaction with BadT status */
            //INCREMENT_COUNTER(smCounterRmppStatusAbortUnsupportedVersion);
            sendAbort = 1; 
            mad.header.rmppStatus = RMPP_STATUS_ABORT_UNSUPPORTED_VERSION; 
            IB_LOG_WARN_FMT(__func__, 
                            "ABORTING - Unsupported Version %d in %s[%s] request from LID[0x%x], TID["FMT_U64"]", 
                            resp.header.rmppVersion, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), (int)rmpp_cntxt->lid, rmpp_cntxt->tid);
        } else if (!(resp.header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE)) {
            /* invalid RMPP type */
            IB_LOG_WARN_FMT(__func__, 
                            "RMPP protocol error, RMPPFlags.Active bit is NULL in %s[%s] from LID[0x%x] for TID["FMT_U64"]", 
                            info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), (int)rmpp_cntxt->lid, rmpp_cntxt->tid); 
            //INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
            mad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE; 
            sendAbort = 1;
        } else if (resp.header.rmppType == RMPP_TYPE_ACK) {
            /* Got an ACK packet from receiver */
            if (resp.header.segNum < rmpp_cntxt->WF) {
                /* silently discard the packet */
                if (if3DebugRmpp) {
                    IB_LOG_INFINI_INFO_FMT(__func__,
						"LID[0x%x] sent ACK for seg %d which is less than Window First (%d) for TID: "FMT_U64,
						rmpp_cntxt->lid, (int)resp.header.segNum, (int)rmpp_cntxt->WF, rmpp_cntxt->tid);
                }
            } else if (resp.header.segNum > rmpp_cntxt->WL) {
                /* ABORT the transaction with S2B status */
                //INCREMENT_COUNTER(smCounterRmppStatusAbortSegnumTooBig);
                sendAbort = 1; 
                mad.header.rmppStatus = RMPP_STATUS_ABORT_SEGNUM_TOOBIG; 
                IB_LOG_INFINI_INFO_FMT(__func__,
					"ABORT - LID[0x%x] sent invalid seg %d in ACK, should be <= than %d, ABORTING TID:"FMT_U64,
					rmpp_cntxt->lid, (int)resp.header.segNum, (int)rmpp_cntxt->WL, rmpp_cntxt->tid);
            } else if (resp.header.length < rmpp_cntxt->WL /*|| resp.header.length > rmpp_cntxt->segTotal*/) {
                /* length is NewWindowLast (NWL) in ACK packet */
                /* ABORT transaction with W2S status */
                sendAbort = 1; 
                if (resp.header.length < rmpp_cntxt->WL) {
                    //INCREMENT_COUNTER(smCounterRmppStatusAbortNewWindowLastTooSmall);
                    mad.header.rmppStatus = RMPP_STATUS_ABORT_NEWWINDOWLAST_TOOSMALL;
                } else {
                    //INCREMENT_COUNTER(smCounterRmppStatusAbortUnspecified);
                    mad.header.rmppStatus = RMPP_STATUS_ABORT_UNSPECIFIED;
                }
                IB_LOG_INFINI_INFO_FMT(__func__,
					"ABORT - LID[0x%x] sent invalid NWL %d in ACK, should be >=%d and <=%d, ABORTING TID:"FMT_U64,
					rmpp_cntxt->lid, (int)resp.header.length, (int)rmpp_cntxt->WL, rmpp_cntxt->segTotal, rmpp_cntxt->tid);
            } else if (resp.header.segNum >= rmpp_cntxt->last_ack) {
                rmpp_cntxt->last_ack = resp.header.segNum; 
                rmpp_cntxt->retries = 0;  /* reset the retry count  after receipt of ack */
                /* is it ack of very last packet? */
                if (resp.header.segNum == rmpp_cntxt->segTotal) {
                    /* we are done */
                    if (if3DebugRmpp) {
                        IB_LOG_INFINI_INFO_FMT(__func__, 
                                               " Received seg %d ACK, %s[%s] transaction from LID[0x%x], TID["FMT_U64"] has completed", 
                                               resp.header.segNum, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid), 
                                               rmpp_cntxt->lid, rmpp_cntxt->tid);
                    }
                } else {
                    /* update WF, WL, and NS and resume sends */
                    rmpp_cntxt->WF = rmpp_cntxt->last_ack + 1; 
                    rmpp_cntxt->WL = (resp.header.length > rmpp_cntxt->segTotal) ? rmpp_cntxt->segTotal : resp.header.length; 
                    /* see if new Response time needs to be calculated */
                    if (resp.header.u.tf.rmppRespTime && resp.header.u.tf.rmppRespTime != 0x1f) {
                        rmpp_cntxt->RespTimeout = 4ull * ((2 * (1 << rmpp_packetLifetime)) + (1 << resp.header.u.tf.rmppRespTime)); 
                        if (if3DebugRmpp) {
                            IB_LOG_INFINI_INFO_FMT(__func__, 
                                                   "LID[0x%x] set RespTimeValue (%d usec) in ACK of seg %d for %s[%s], TID["FMT_U64"]", 
                                                   rmpp_cntxt->lid, (int)rmpp_cntxt->RespTimeout, resp.header.segNum, 
                                                   info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid), 
                                                   rmpp_cntxt->tid);
                        }
                    }
                }
            }
        } else if (resp.header.rmppType == RMPP_TYPE_STOP || resp.header.rmppType == RMPP_TYPE_ABORT) {
            /* got a STOP or ABORT */
            if (if3DebugRmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__,
					"STOP/ABORT received for %s[%s] from LID[0x%x], status code = %x, for TID["FMT_U64"]",
					info->rmpp_get_method_text((int)rmpp_cntxt->method),
					info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid),
					rmpp_cntxt->lid, resp.header.rmppStatus, rmpp_cntxt->tid);
            }
            rmpp_cntxt_release(rmpp_cntxt); 
            IB_EXIT(__func__, VSTATUS_OK); 
            return VSTATUS_OK;
        } else {
            /* invalid RmppType received */
            IB_LOG_WARN_FMT(__func__, 
                            "ABORT - Invalid rmppType %d received for %s[%s] from LID[0x%x] for TID["FMT_U64"]", 
                            resp.header.rmppType, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), 
                            rmpp_cntxt->lid, rmpp_cntxt->tid); 
            // abort with badtype status
            //INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
            sendAbort = 1; 
            mad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
        }
    } else {
        /* We are timing out, retry till retry  count expires */
        /* get original mad from context with correct offset */
        (void)BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER *)rmpp_cntxt->mad.data, (STL_SA_MAD_HEADER *)&mad);

        ++rmpp_cntxt->retries; 
        if (rmpp_cntxt->retries > rmppMaxRetries) {
            if (if3DebugRmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__,
					"ABORT - MAX RETRIES EXHAUSTED; no ACK for seg %d of %s[%s] request from LID[0x%X], TID["FMT_U64"]",
					(int)rmpp_cntxt->WL, info->rmpp_get_method_text((int)rmpp_cntxt->method),
					info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid),
					rmpp_cntxt->lid, rmpp_cntxt->tid);
            }
            /* ABORT transaction with too many retries status */
            //INCREMENT_COUNTER(smCounterRmppStatusAbortTooManyRetries);
            sendAbort = 1; 
            mad.header.rmppStatus = RMPP_STATUS_ABORT_TOO_MANY_RETRIES; 
            /* let context_age do the releasing;  It already holds the lock */
            releaseContext = 0;
        } else {
            //INCREMENT_COUNTER(smCounterSaRmppTxRetries);
            rmpp_cntxt->NS = rmpp_cntxt->WF;            // reset Next packet segment to send
            if (if3DebugRmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__, 
                                       "Timed out waiting for ACK of seg %d of %s[%s] from LID[0x%x], TID["FMT_U64"], retry #%d", 
                                       (int)rmpp_cntxt->WL, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid), 
                                       rmpp_cntxt->lid, rmpp_cntxt->tid, rmpp_cntxt->retries);
            }
        }
    }
    
    /* see if we're done (last segment was acked) */
    if (rmpp_cntxt->last_ack == rmpp_cntxt->segTotal && !sendAbort) {
        if (if3DebugRmpp) {
            vs_time_get(&tnow); 
            delta = tnow - rmpp_cntxt->mad.intime; 
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "%s[%s] RMPP [CHKSUM=%d] TRANSACTION from LID[0x%x], TID["FMT_U64"] has completed in %d.%.3d seconds (%"CS64"d usecs)", 
                                   info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid), 
                                   rmpp_cntxt->chkSum, rmpp_cntxt->lid, rmpp_cntxt->tid, 
                                   (int)(delta / 1000000), (int)((delta - delta / 1000000 * 1000000)) / 1000, delta);
        }
        /* validate that the 8-bit cheksum of the rmpp response is still the same as when we started */
        if (rmppCheckSum && rmpp_cntxt->data) {
            chkSum = 0; 
            for (i = 0; i < rmpp_cntxt->len; i++) {
                chkSum += rmpp_cntxt->data[i];
            }
            if (chkSum != rmpp_cntxt->chkSum) {
                IB_LOG_ERROR_FMT(__func__, 
                                 "CHECKSUM FAILED [%d vs %d] for completeted %s[%s] RMPP TRANSACTION from LID[0x%x], TID["FMT_U64"]", 
                                 chkSum, rmpp_cntxt->chkSum, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid), 
                                 rmpp_cntxt->lid, rmpp_cntxt->tid);
            }
        }
        if (releaseContext) 
            rmpp_cntxt_release(rmpp_cntxt); 
        IB_EXIT(__func__, VSTATUS_OK); 
        return VSTATUS_OK;
    }
    
    /* we must use the Mad_t that was saved in the context */
    maip = &rmpp_cntxt->mad; 
    /* 
      * send segments up till Window Last (WL) and wait for ACK 
      * Due to possible concurrency issue if reader is pre-empted while 
      * sending segement 1 and writer runs to process ACK of seg 1, use 
      * a local var for WL.
      * In the future we need to add individual context locking if we intend 
      * to go with a pool of SA threads.
      */
    wl = rmpp_cntxt->WL; 
    while (rmpp_cntxt->NS <= wl && !sendAbort) {
        
        /*
           * calculate amount of data length to send in this segment and put in mad;
           * dlen=payloadLength in first packet, dlen=remainder in last packet
           */
        if (rmpp_cntxt->NS == rmpp_cntxt->segTotal) {
            dlen = (sa_data_size) ? (rmpp_cntxt->len % sa_data_size) : 0; 
            dlen = (dlen) ? dlen : sa_data_size;
        } else 
            dlen = sa_data_size; 
        
        if (dlen > info->rmpp_data_length) {
            IB_LOG_WARN("dlen is too large dlen:", dlen);
        }
        // make sure there is data to send; could just be error case with no data
        if (rmpp_cntxt->len && rmpp_cntxt->data) {
            (void)memcpy(mad.data, rmpp_cntxt->data + ((rmpp_cntxt->NS - 1) * sa_data_size), dlen);
        }
        
        mad.header.rmppVersion = RMPP_VERSION; 
        mad.header.rmppType = RMPP_TYPE_DATA; 
        mad.header.rmppStatus = 0; 
        mad.header.segNum = rmpp_cntxt->NS; 
        if (rmpp_cntxt->NS == 1 && rmpp_cntxt->NS == rmpp_cntxt->segTotal) {
            /* 
                * first and last segment to transfer, set length to payload length
                * add 20 bytes of SA header to each segment for total payload 
                */
            mad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_FIRST | RMPP_FLAGS_LAST; 
            mad.header.length = rmpp_cntxt->len + (rmpp_cntxt->segTotal * SA_HEADER_SIZE);   
            //if (if3DebugRmpp) IB_LOG_INFINI_INFO( "SA Transaction First and Last Frag len:", mad.header.length );
        } else if (rmpp_cntxt->NS == 1) {
            /* 
                * first segment to transfer, set length to payload length
                * add 20 bytes of SA header to each segment for total payload 
                */
            mad.header.length = rmpp_cntxt->len + (rmpp_cntxt->segTotal * SA_HEADER_SIZE);   
            mad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_FIRST; 
            //if (if3DebugRmpp) IB_LOG_INFINI_INFO( "SA Transaction First Frag len:", mad.header.length );
        } else if (rmpp_cntxt->NS == rmpp_cntxt->segTotal) {
            /* last segment to go; len=bytes remaining */
            mad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_LAST; 
            mad.header.length = dlen + SA_HEADER_SIZE;  // add the extra 20 bytes of SA header
            if (mad.header.length == 0) {
                mad.header.length = sa_data_size;
            }
            //if (if3DebugRmpp) IB_LOG_INFINI_INFO( "SA Transaction Last Frag len:", mad.header.length );
        }  else {
            /* middle segments */
            mad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE; 
            mad.header.length = 0; 
            //if (if3DebugRmpp) IB_LOG_INFINI_INFO( "SA Transaction Middle Frag len:", mad.header.length );
        }
        /* put mad back into Mad_t in the context */
        datalen += sa_data_size; 
        BSWAPCOPY_STL_SA_MAD((STL_SA_MAD *)&mad, 
                             (STL_SA_MAD *)maip->data, 
                             sa_data_size); 
        
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__,
				"sending fragment %d of %d segments, len %d to LID[0x%x] for TID["FMT_U64"]",
               (int)mad.header.segNum, rmpp_cntxt->segTotal, (int)mad.header.length, (int)rmpp_cntxt->lid, 
               rmpp_cntxt->tid);
        }
        /* increment NS */
        ++rmpp_cntxt->NS; 
        if (maip->base.bversion == IB_BASE_VERSION) {
            status = mai_send(rmpp_cntxt->sendFd, maip);
        } else {
            status = mai_stl_send(rmpp_cntxt->sendFd, maip, &datalen);
        }
        
        if (status != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__, 
                             "mai_send error [%d] while processing %s[%s] request from LID[0x%x], TID["FMT_U64"]", 
                             status, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), 
                             rmpp_cntxt->lid, rmpp_cntxt->tid); 
            if (releaseContext) 
                rmpp_cntxt_release(rmpp_cntxt); 
            IB_EXIT(__func__, VSTATUS_OK); 
            return VSTATUS_OK;
        }
    }
    
    /*
      * Send Abort if desired
      */
    if (sendAbort) {
        mad.header.rmppVersion = RMPP_VERSION; 
        mad.header.rmppType = RMPP_TYPE_ABORT; 
        mad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE; 
        mad.header.u.tf.rmppRespTime = 0; 
        mad.header.segNum = 0; 
        mad.header.length = 0; 
        (void)BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER *)&mad, (STL_SA_MAD_HEADER *)maip->data);
        
        if ((status = mai_send(rmpp_cntxt->sendFd, maip)) != VSTATUS_OK) 
            IB_LOG_ERROR_FMT(__func__,
				"error[%d] from mai_send while sending ABORT of %s[%s] request to LID[0x%x], TID["FMT_U64"]",
				status, info->rmpp_get_method_text((int)rmpp_cntxt->method),
				info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid),
				rmpp_cntxt->lid, maip->base.tid); 
        /*
           * We are done with this RMPP xfer.  Release the context here if 
           * in flight transaction (maip not NULL) or let sa_cntxt_age do it 
           * for us because retries is > SA_MAX_RETRIES. 
           */
        if (releaseContext) 
            rmpp_cntxt_release(rmpp_cntxt);
    }
    
    IB_EXIT(__func__, VSTATUS_OK); 
    return (VSTATUS_OK);
}

Status_t
rmpp_send_reply(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt) 
{ 
   uint8_t		method; 
   STL_LID		lid; 
   rmpp_user_info_t *info = NULL; 
   
   IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0); 
   
   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return VSTATUS_BAD;
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
      return VSTATUS_BAD;
   }
   
   // setup the out-bound MAD, we need to reverse the LRH addresses.
   if (maip)  {
      lid = maip->addrInfo.slid; 
      maip->addrInfo.slid = maip->addrInfo.dlid; 
      maip->addrInfo.dlid = lid; 
      method = maip->base.method; 
      if (rmpp_cntxt) 
         rmpp_cntxt->method = method; 
      switch (maip->base.method) {
      case RMPP_CMD_GET:
         maip->base.method = RMPP_CMD_GET_RESP; 
         break; 
      case RMPP_CMD_GETTABLE:
         maip->base.method = RMPP_CMD_GETTABLE_RESP; 
         break; 
      default:
         // everybody else, OR in 0x80 to inbound; i.e., 14->94
         maip->base.method |= MAD_CM_REPLY; 
         break;
      }
   } else if (rmpp_cntxt) {
      // this is the timeout case
      method = rmpp_cntxt->method;
   } else {
      return (VSTATUS_BAD);
   }
   
   // always send rmpp responses to multi-packet responses
   if ((rmpp_cntxt && method == RMPP_CMD_GETTABLE) || (maip && maip->base.method == RMPP_CMD_GETTABLE_RESP)
       /*|| rmpp_cntxt->len > SAMAD_DATA_COUNT*/) {
      // as per Table 154, page 784 of spec, can only return this error in response to a GET/SET
      // for all other requests, we simply return a 1 segment rmpp response with header only 
      // additional spec references relating to handling of rmpp responses are (C15-0.1.19, C15-0.1.29)
      if (maip && 
          (maip->base.method == RMPP_CMD_GETTABLE_RESP) && 
          (maip->base.status == MAD_STATUS_SA_NO_RECORDS)) {
         maip->base.status = MAD_STATUS_SA_NO_ERROR;
      }
      if (rmpp_cntxt->len > info->rmpp_data_length) {
         IB_LOG_WARN_FMT(__func__, 
                         "rmpp_cntxt->len[%d] too large, returning no resources error to caller to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!", 
                         rmpp_cntxt->len, info->rmpp_get_method_text((int)method), (maip?info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid):"<null>"), rmpp_cntxt->lid, rmpp_cntxt->tid); 
         if (maip) maip->base.status = MAD_STATUS_SA_NO_RESOURCES; 
         rmpp_cntxt->len = 0;
      }
      
      if (info->rmpp_format == RMPP_FORMAT_SA) {
         (void)rmpp_send_multi_sa(maip, rmpp_cntxt);
      } else if (info->rmpp_format == RMPP_FORMAT_VENDOR) {
         (void)rmpp_send_multi_vendor(maip, rmpp_cntxt);
      } else {
         IB_LOG_WARN_FMT(__func__,
               "info->rmpp_format = %d not supported. Should be RMPP_FORMAT_SA(%d) or RMPP_FORMAT_VENDOR(%d).",
               info->rmpp_format, RMPP_FORMAT_SA, RMPP_FORMAT_VENDOR);
         IB_EXIT(__func__, VSTATUS_NOSUPPORT);
         return VSTATUS_NOSUPPORT;
      }
      IB_EXIT(__func__, VSTATUS_OK); 
      return (VSTATUS_OK);
   }
   
   if (rmpp_cntxt->len > STL_SAMAD_DATA_COUNT) {
      IB_LOG_WARN_FMT(__func__, 
                      "rmpp_cntxt->len[%d] too large for GET, returning too many recs error to caller to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!", 
                      rmpp_cntxt->len, info->rmpp_get_method_text((int)method), (maip?info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid):"<null>"), rmpp_cntxt->lid, rmpp_cntxt->tid); 
      if (maip) maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS; 
      rmpp_cntxt->len = 0;
   }
   
   if (maip) {
      if (info->rmpp_format == RMPP_FORMAT_SA) {
         rmpp_send_single_sa(maip, rmpp_cntxt);
      } else if (info->rmpp_format == RMPP_FORMAT_VENDOR) {
         rmpp_send_single_vendor(maip, rmpp_cntxt);
      } else {
         IB_LOG_WARN_FMT(__func__,
               "info->rmpp_format = %d not supported. Should be RMPP_FORMAT_SA(%d) or RMPP_FORMAT_VENDOR(%d).",
               info->rmpp_format, RMPP_FORMAT_SA, RMPP_FORMAT_VENDOR);
         IB_EXIT(__func__, VSTATUS_NOSUPPORT);
         return VSTATUS_NOSUPPORT;
      }
      IB_EXIT(__func__, VSTATUS_OK); 
      return (VSTATUS_OK);
   }
   
   IB_LOG_WARN_FMT(__func__, "Maip is null. Cannot continue.");
   IB_EXIT(__func__, VSTATUS_BAD); 
   return (VSTATUS_BAD);
}

Status_t
rmpp_send_request(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt) 
{
   Status_t status = VSTATUS_BAD;
   rmpp_user_info_t *info = NULL; 
   
   IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0); 
   
   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      goto done;
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
      goto done;
   } else if (!maip) {
      IB_LOG_ERROR_FMT(__func__, "handle is NULL!"); 
      goto done;
   }
   
   // setup the out-bound MAD, we need to reverse the LRH addresses.
   rmpp_cntxt->method = maip->base.method; 
      
   //      
   // send multi-packet or single-packet response
   if (maip->base.method == RMPP_CMD_GETTABLE || maip->base.method == RMPP_CMD_GETTRACETABLE) {
      if (rmpp_cntxt->len > info->rmpp_data_length) {
         IB_LOG_WARN_FMT(__func__, 
                         "rmpp_cntxt->len[%d] too large, returning no resources error to caller to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!", 
                         rmpp_cntxt->len, info->rmpp_get_method_text((int)maip->base.method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), rmpp_cntxt->lid, rmpp_cntxt->tid); 
         maip->base.status = MAD_STATUS_SA_NO_RESOURCES; 
         rmpp_cntxt->len = 0;
      }
      
      if (info->rmpp_format == RMPP_FORMAT_SA) {
         status = rmpp_send_multi_sa(maip, rmpp_cntxt);
      } else if (info->rmpp_format == RMPP_FORMAT_VENDOR) {
         status = rmpp_send_multi_vendor(maip, rmpp_cntxt);
      } else { // Not supported
         status = VSTATUS_NOSUPPORT;
         IB_LOG_WARN_FMT(__func__,
                         "info->rmpp_format = %d not supported. Should be RMPP_FORMAT_SA(%d) or RMPP_FORMAT_VENDOR(%d).",
                         info->rmpp_format, RMPP_FORMAT_SA, RMPP_FORMAT_VENDOR);
      }
      // deallocation of the context will be handled after completion of the
      // transmission. 
      return status;
   }
   
   if (rmpp_cntxt->len > STL_SAMAD_DATA_COUNT) {
      IB_LOG_WARN_FMT(__func__, 
                      "rmpp_cntxt->len[%d] too large for GET, returning too many recs error to caller to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!", 
                      rmpp_cntxt->len, info->rmpp_get_method_text((int)maip->base.method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), rmpp_cntxt->lid, rmpp_cntxt->tid); 
      maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS; 
      rmpp_cntxt->len = 0;
   }

   if (info->rmpp_format == RMPP_FORMAT_SA) {
       status = rmpp_send_single_sa(maip, rmpp_cntxt);
   } else if (info->rmpp_format == RMPP_FORMAT_VENDOR) {
       status = rmpp_send_single_vendor(maip, rmpp_cntxt);
   } else { // Not supported
       status = VSTATUS_NOSUPPORT;
       IB_LOG_WARN_FMT(__func__,
                       "info->rmpp_format = %d not supported. Should be RMPP_FORMAT_SA(%d) or RMPP_FORMAT_VENDOR(%d).",
                       info->rmpp_format, RMPP_FORMAT_SA, RMPP_FORMAT_VENDOR);
   }

done:
   if (rmpp_cntxt)
      rmpp_cntxt_release(rmpp_cntxt);

   IB_EXIT(__func__, status); 
   return (status);
}

/*
 * safely release context while already under lock protection
 */
static void
rmpp_lcl_cntxt_release(rmpp_cntxt_t *rmpp_cntxt) 
{ 
   int         bucket; 
   rmpp_user_info_t *info = NULL; 
   
   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return;
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
      return;
   }
   
   if (rmpp_cntxt->ref == 0) {
      IB_LOG_INFINI_INFO_FMT(__func__, "reference count is already zero"); 
      return;   /* context already retired */
   } else {
      --rmpp_cntxt->ref; 
      if (rmpp_cntxt->ref == 0) {
         // this context needs to be removed from hash
         if (rmpp_cntxt->hashed) {
            bucket = rmpp_cntxt->lid % RMPP_CNTXT_HASH_TABLE_DEPTH; 
            rmpp_cntxt_delete_entry(info->rmpp_hash[bucket], rmpp_cntxt);
         }
         rmpp_cntxt->prev = rmpp_cntxt->next = NULL; 
         rmpp_cntxt_retire(rmpp_cntxt);
      }
   }
}

/*
 * Age context and do resends for those that timed out
 */
static void
rmpp_cntxt_age(rmpp_user_info_t *info) 
{ 
   rmpp_cntxt_t *rmpp_cntxt = NULL; 
   rmpp_cntxt_t *tout_cntxt; 
   int i; 
   Status_t	status; 
   
   
   if (!info) {
      IB_LOG_ERROR_FMT(__func__, "user info is NULL!"); 
      return;
   }
   
   if ((status = vs_lock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, "Failed to lock context rc: %d", status);
   } else {
      vs_time_get(&rmpp_timeLastAged); 
      for (i = 0; i < RMPP_CNTXT_HASH_TABLE_DEPTH; i++) {
         rmpp_cntxt = info->rmpp_hash[i]; 
         while (rmpp_cntxt) {
            // Iterate before the pointers are destroyed ;
            tout_cntxt = rmpp_cntxt; 
            rmpp_cntxt = rmpp_cntxt->next; 
            
            // Though list is sorted, it does not hurt to 
            // do a simple comparison
            if (rmpp_timeLastAged - tout_cntxt->tstamp > tout_cntxt->RespTimeout) { // used to be VTIMER_1S
                                                                                    // Timeout this entry
               rmpp_cntxt_delete_entry(info->rmpp_hash[i], tout_cntxt); 
               rmpp_cntxt_insert_head(info->rmpp_hash[i], tout_cntxt); 
               // Touch the entry
               tout_cntxt->tstamp = rmpp_timeLastAged; 
               // Call timeout
               tout_cntxt->sendFd = info->rmpp_fd_w;       // use writer mai handle for restransmits
                                                           // resend the reply
               rmpp_send_reply(NULL, tout_cntxt); 
               if (tout_cntxt->retries > rmppMaxRetries) {
                  /* need to release the context.  Call local safe release */
                  rmpp_lcl_cntxt_release(tout_cntxt);
               }
            }
         }
      }
      if ((status = vs_unlock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
         IB_LOG_ERROR_FMT(__func__, "Failed to unlock context rc: %d", status);
      }
   }
}

/*
 * Deallocate outstanding contexts
 */
static void
rmpp_cntxt_free(rmpp_user_info_t *info) 
{ 
   rmpp_cntxt_t *rmpp_cntxt = NULL; 
   rmpp_cntxt_t *tout_cntxt; 
   int i; 
   Status_t    status; 
   
   IB_ENTER(__func__, info, 0, 0, 0); 
   
   if (!info) {
      IB_LOG_ERROR_FMT(__func__, "user info is NULL!"); 
      return;
   }
   
   if ((status = vs_lock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, "Failed to lock context rc: %d", status);
   } else {
      for (i = 0; i < RMPP_CNTXT_HASH_TABLE_DEPTH; i++) {
         rmpp_cntxt = info->rmpp_hash[i]; 
         while (rmpp_cntxt) {
            // iterate before the pointers are destroyed ;
            tout_cntxt = rmpp_cntxt; 
            rmpp_cntxt = rmpp_cntxt->next; 
            
            // adjust reference count in order to force deallocation 
            tout_cntxt->ref = 1; 
            // release the context
            rmpp_lcl_cntxt_release(tout_cntxt);
         }
      }
      if ((status = vs_unlock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
         IB_LOG_ERROR_FMT(__func__, "Failed to unlock context rc: %d", status);
      }
   }
}

//
// find context entry matching input mad
//
static rmpp_cntxt_t* 
rmpp_cntxt_find(rmpp_user_info_t *info, Mai_t *mad) 
{ 
   uint64_t	now; 
   int 		bucket; 
   Status_t	status; 
   rmpp_cntxt_t *rmpp_cntxt; 
   rmpp_cntxt_t *req_cntxt = NULL; 
   
   IB_ENTER(__func__, info, mad, 0, 0); 
   
   if (!info) {
      IB_LOG_ERROR_FMT(__func__, "user info is NULL!"); 
      return req_cntxt;
   }
   
   if ((status = vs_lock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, "Failed to lock context rc: %d", status);
   } else {
      vs_time_get(&now); 
      // search the hash table for the context 	
      bucket = mad->addrInfo.slid % RMPP_CNTXT_HASH_TABLE_DEPTH; 
      rmpp_cntxt = info->rmpp_hash[bucket]; 
      while (rmpp_cntxt) {
         if (rmpp_cntxt->lid == mad->addrInfo.slid  && rmpp_cntxt->tid == mad->base.tid) {
            req_cntxt = rmpp_cntxt; 
            rmpp_cntxt = rmpp_cntxt->next; 
            req_cntxt->tstamp = now; 
            break;
         } else {
            rmpp_cntxt = rmpp_cntxt->next;
         }
      }
      // table is sorted with respect to timeout
      if (req_cntxt) {
         // touch current context
         req_cntxt->tstamp = now; 
         rmpp_cntxt_delete_entry(info->rmpp_hash[bucket], req_cntxt); 
         rmpp_cntxt_insert_head(info->rmpp_hash[bucket], req_cntxt); 
         // a get on an existing context reserves it
         req_cntxt->ref++;
      }
      if ((status = vs_unlock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
         IB_LOG_ERROR_FMT(__func__, "Failed to unlock context rc: %d", status);
      }
   }
   
   IB_EXIT(__func__, req_cntxt); 
   return req_cntxt;
}

//
// Process only inflight rmpp requests
// Called by the rmpp_main_writer thread
//
static Status_t
rmpp_process_inflight_rmpp_request(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt) 
{ 
   rmpp_user_info_t *info = NULL; 
   
   IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0); 
   
   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return VSTATUS_BAD;
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
      return VSTATUS_BAD;
   }
   
   //
   //	Process only inflight rmpp requests 
   //  ACKs of inprogress GETMULTI requests also come here
   //
   if (rmpp_cntxt->hashed  && !rmpp_cntxt->reqInProg) {
      rmpp_send_reply(maip, rmpp_cntxt);
   } else {
      IB_LOG_INFINI_INFO_FMT(__func__, 
                             "SA_WRITER received %s[%s] RMPP packet from LID [0x%x] TID ["FMT_U64"] after transaction completion", 
                             info->rmpp_get_method_text((int)maip->base.method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
   }

   IB_EXIT(__func__, VSTATUS_OK); 
   return (VSTATUS_OK);
}

/*
 * Very simple hashing implemented. This func is modular and can be 
 * changed to use any algorithm, if hashing turns out to be bad.
 * Returns new context for new requests, existing context for in progress 
 * getMulti requests and NULL for duplicate requests
 */
static rmpp_context_get_t
cntxt_get(rmpp_user_info_t *info, Mai_t *mad, void **context) 
{ 
   uint64_t	now; 
   int 		bucket; 
   Status_t	status; 
   rmpp_cntxt_t *rmpp_cntxt; 
   rmpp_context_get_t getStatus = ContextNotAvailable; 
   rmpp_cntxt_t *req_cntxt = NULL; 
   
   IB_ENTER(__func__, mad, 0, 0, 0); 
   
   if (!info) {
      IB_LOG_ERROR_FMT(__func__, "user info is NULL!"); 
      return ContextNotAvailable;
   }
   
   if ((status = vs_lock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, "Failed to lock context rc: %d", status);
   } else {
      vs_time_get(&now); 
      // search the hash table for the context 	
      bucket = mad->addrInfo.slid % RMPP_CNTXT_HASH_TABLE_DEPTH; 
      rmpp_cntxt = info->rmpp_hash[bucket]; 
      while (rmpp_cntxt) {
         if (rmpp_cntxt->lid == mad->addrInfo.slid  && 
             rmpp_cntxt->tid == mad->base.tid) {
            req_cntxt = rmpp_cntxt; 
            break;
         } else {
            rmpp_cntxt = rmpp_cntxt->next;
         }
      }
      if (req_cntxt) {
         if ((req_cntxt->method != RMPP_CMD_GET && req_cntxt->method != RMPP_CMD_GET_RESP)  && req_cntxt->reqInProg) {
            // in progress getMulti request. Touch and reserve it.
            getStatus = ContextExistGetMulti; 
            req_cntxt->tstamp = now; 
            rmpp_cntxt_delete_entry(info->rmpp_hash[bucket], req_cntxt); 
            rmpp_cntxt_insert_head(info->rmpp_hash[bucket], req_cntxt); 
            // A get on an existing context reserves it
            req_cntxt->ref++;
         } else {
            // dup of an existing request
            getStatus = ContextExist; 
            req_cntxt = NULL;
         }
      } else {
         /* Allocate a new context and set appropriate status */
         req_cntxt = info->rmpp_cntxt_free_list; 
         if (req_cntxt) {
            getStatus = ContextAllocated; 
            rmpp_cntxt_delete_entry(info->rmpp_cntxt_free_list, req_cntxt); 
            req_cntxt->ref = 1;  // set ref count to 1
            INCR_RMPP_CNTXT_NALLOC(); 
            DECR_RMPP_CNTXT_NFREE(); 
            req_cntxt->lid = mad->addrInfo.slid; 
            req_cntxt->tid = mad->base.tid; 
            req_cntxt->method = mad->base.method; 
            req_cntxt->tstamp = now; 
            req_cntxt->info = info; 
            req_cntxt->usrId = info->usrId;
         } else {
            // out of context
            getStatus = ContextNotAvailable;
         }
      }
      if ((status = vs_unlock(&info->rmpp_cntxt_lock)) != VSTATUS_OK) {
         IB_LOG_ERROR_FMT(__func__, "Failed to unlock context rc: %d", status);
      }
   }
   // return new context or existing getmulti context or NULL
   *context = req_cntxt; 
   
   IB_EXIT(__func__, getStatus); 
   return getStatus;
}

static int
rmpp_writer_filter(Mai_t *maip) 
{ 
   STL_SA_MAD_HEADER mad; 

   // the client writer thread is only responsible for in flight rmpp packets
   if (maip->base.method == RMPP_CMD_GETTABLE || maip->base.method == (RMPP_CMD_GETTABLE_RESP ^ MAD_RMPP_REPLY)) {
      rmpp_user_info_t *info = rmpp_mclass_get_userinfo(maip->base.mclass); 
      
      if (!info) {
         IB_LOG_WARN_FMT(__func__, "failed to get user info for MCLASS 0x%x!", maip->base.mclass); 
         return 1;   // ignore
      }
      
      // check for inflight rmpp
      (void)BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER *)maip->data, (STL_SA_MAD_HEADER *)&mad);

      if ((mad.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE) && 
          ((mad.rmppType == RMPP_TYPE_ACK && mad.segNum > 0) ||
           mad.rmppType == RMPP_TYPE_STOP ||
           mad.rmppType == RMPP_TYPE_ABORT)) {
         if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "Processing inflight Rmpp packet for %s[%s] from LID[0x%x], TID="FMT_U64, 
                                   info->rmpp_get_method_text((int)maip->base.method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
         }
         return 0;  // process inflight rmpp responses
      }
   }
   
   return 1;
}

static void
rmpp_main_writer(uint32_t argc, uint8_t **argv) 
{ 
   Status_t status; 
   Mai_t in_mad; 
   Filter_t filter; 
   rmpp_cntxt_t *rmpp_cntxt; 
   uint64_t now; 
   uint8_t usrId = *argv[0];   // parameter: user ID
   rmpp_user_info_t *info = NULL; 
   
   
   IB_ENTER(__func__, usrId, 0, 0, 0); 
   
   if (!(info = rmpp_get_userinfo(usrId))) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", usrId); 
      return;
   }
   
   //
   // create the MAD filter for the RMPP writer thread.
   RMPP_Filter_Init(&filter); 
   filter.value.mclass = info->mclass; 
   filter.mask.mclass = 0xff; 
   filter.value.method = 0x00; 
   filter.mask.method = 0x80; 
   filter.mai_filter_check_packet = rmpp_writer_filter; 
   MAI_SET_FILTER_NAME(&filter, "RMPP Writer"); 
   
   if (mai_filter_create(info->rmpp_fd_w, &filter, VFILTER_SHARE) != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, "can't create filter for writer thread"); 
      (void)vs_thread_exit(&info->wt_thrd);
   }
   
   while (1) {
      status = mai_recv(info->rmpp_fd_w, &in_mad, VTIMER_1S / 4); 
      if (status != VSTATUS_OK && status != VSTATUS_TIMEOUT) {
         IB_LOG_ERROR_FMT(__func__, "error on mai_recv rc: %d", status); 
         vs_thread_sleep(VTIMER_1S / 10);
      }
      
      if (info->wt_thrd_exit == 1) {
         info->wt_thrd_exit++; 
         IB_LOG_INFINI_INFO_FMT(__func__, "%s Writer Task exiting OK.", info->wt_thrd_name); 
         break;
      }
      
      /* don't process messages if RMPP user not master, or still doing first sweep */
      if (!info->rmpp_is_master()) {
         continue;
      }
      
      /* 
       * process the rmpp ack and send out the next set of segments
       */
      if (status == VSTATUS_OK) {
         /* locate and process in flight rmpp request */
         rmpp_cntxt = rmpp_cntxt_find(info, &in_mad); 
         if (rmpp_cntxt) {
            rmpp_process_inflight_rmpp_request(&in_mad, rmpp_cntxt); 
            /*
             * This may not necessarily release context
             * based on if someone else has reserved it
             */
            rmpp_cntxt_release(rmpp_cntxt);
         } else {
            //INCREMENT_PM_COUNTER(pmCounterPaDeadRmppPacket);
            if (if3DebugRmpp) {
               IB_LOG_INFINI_INFO_FMT(__func__, 
                                      "dropping %s[%s] RMPP packet from LID[0x%x], TID ["FMT_U64"] already completed/aborted", 
                                      info->rmpp_get_method_text((int)in_mad.base.method), info->rmpp_get_aid_name((int)in_mad.base.mclass, (int)in_mad.base.aid), 
                                      in_mad.addrInfo.slid, in_mad.base.tid);
            }
         }
      }
      
      /* age contexts if more than 1 second since last time */
      vs_time_get(&now); 
      if ((now - rmpp_timeLastAged) > (VTIMER_1S)) {
         (void)rmpp_cntxt_age(info);
      }
   }
   
   // delete filter
   mai_filter_delete(info->rmpp_fd_w, &filter, filter.flags); 
   // close channel
   (void)mai_close(info->rmpp_fd_w); 
   
   //IB_LOG_INFINI_INFO_FMT(__func__, "Exiting OK");
}

int rmpp_is_cnx_open(IBhandle_t *fd) 
{ 
   int i; 
   int usrId = -1; 
   
   if (!rmpp_initialized) {
      //IB_LOG_ERROR_FMT(__func__, "RMPP library not initialized!"); 
      return usrId;
   } else if (!fd) {
      IB_LOG_ERROR_FMT(__func__, "IF3 connection handle is NULL!"); 
      return usrId;
   }
   
   vs_lock(&rmpp_user_lock); 
   for (i = 0; i < RMPP_MAX_USERS; i++) {
      rmpp_user_info_t *usr = &rmpp_users[i]; 
      
      if (usr->inuse && (usr->fd && (usr->fd == fd))) 
         usrId = usr->usrId;
   }
   vs_unlock(&rmpp_user_lock); 
   
   return usrId;
}

int rmpp_is_cnx_partial_open(int usrId) 
{ 
   rmpp_user_info_t *info = rmpp_get_userinfo(usrId); 
   
   if (!info) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", usrId); 
      return 0;
   }
      
   return info->partial_close;
}

static int rmpp_get_new_user(void) 
{ 
   int i, usrId = -1; 
   rmpp_user_info_t *info = NULL; 
   
   if (!rmpp_initialized) {
      IB_LOG_ERROR_FMT(__func__, "RMPP library not initialized!"); 
      return usrId;
   }
   
   vs_lock(&rmpp_user_lock); 
   // find available user structure
   for (i = 0; i < RMPP_MAX_USERS; i++) {
      if (!rmpp_users[i].inuse) {
         usrId = i + 1;  // user ID is 1 relative
         rmpp_userCount++; 
         break;
      }
   }
   
   // initialize new user structure
   if (-1 != usrId) {
      if (!(info = rmpp_get_userinfo(usrId))) {
         vs_unlock(&rmpp_user_lock); 
         IB_LOG_ERROR_FMT(__func__, "failed to get info for new user %d!", usrId); 
         return -1;
      }
      
      memset(info, 0, sizeof(rmpp_user_info_t)); 
      info->inuse = 1; 
      info->usrId = usrId;
   }
   vs_unlock(&rmpp_user_lock); 
   
   return usrId;
}

static void rmpp_delete_user(rmpp_user_info_t *info) 
{ 
   
   if (!rmpp_initialized) {
      IB_LOG_ERROR_FMT(__func__, "RMPP library not initialized!"); 
      return;
   }
   
   if (!info) 
      IB_LOG_ERROR_FMT(__func__, "user info is NULL!"); 
   else {
      vs_lock(&rmpp_user_lock); 
      // reset user structure
      memset(info, 0, sizeof(rmpp_user_info_t)); 
      info->inuse = 0; 
      rmpp_userCount--; 
      vs_unlock(&rmpp_user_lock);
   }
}

static Status_t
rmpp_protocol_init(
   rmpp_user_info_t *info, 
   Pool_t *pool, 
   uint32_t data_length,
   uint32_t max_cntxt,
   uint32_t qp, 
   uint32_t dev, 
   uint32_t port, 
   char* (*get_method_text)(int), 
   char* (*get_aid_name)(uint16_t, uint16_t), 
   Status_t(*pre_process_request)(Mai_t *, rmpp_cntxt_t *), 
   Status_t(*pre_process_response)(Mai_t *, rmpp_cntxt_t *), 
   uint8_t(*rmpp_is_master)(void), 
   uint32_t vieo_mod_id, 
   char *wtName
   ) 
{ 
   int i; 
   Status_t status; 
   uint8_t *thread_id = (uint8_t *)wtName;  
   
   
   if (!info) {
      IB_LOG_ERROR_FMT(__func__, "user info is NULL!"); 
      return VSTATUS_BAD;
   }
   
   info->qp = qp; 
   info->dev = dev; 
   info->port = port; 
   info->rmpp_get_method_text = get_method_text; 
   info->rmpp_get_aid_name = get_aid_name; 
   info->rmpp_pre_process_request = pre_process_request; 
   info->rmpp_pre_process_response = pre_process_response; 
   info->rmpp_is_master = rmpp_is_master; 
   info->vieo_mod_id = vieo_mod_id; 
   
   // initialize context pool to inflight RMPP requests
   memset(info->rmpp_hash, 0, sizeof(info->rmpp_hash)); 
   info->rmpp_cntxt_pool = NULL; 
   info->rmpp_pool = pool; 
   info->rmpp_max_cntxt = max_cntxt;

   IB_LOG_INFINI_INFO_FMT(__func__, "Allocating context pool with num entries=%d", info->rmpp_max_cntxt); 
   status = vs_pool_alloc(info->rmpp_pool, sizeof(rmpp_cntxt_t) * info->rmpp_max_cntxt, (void *)&info->rmpp_cntxt_pool); 
   if (status != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, "Can't allocate context pool"); 
      return VSTATUS_NOMEM;
   }
   
   memset(info->rmpp_cntxt_pool, 0, sizeof(rmpp_cntxt_t) * info->rmpp_max_cntxt); 
   info->rmpp_cntxt_free_list = NULL; 
   
   for (i = 0; i < info->rmpp_max_cntxt; ++i) {
      rmpp_cntxt_insert_head(info->rmpp_cntxt_free_list, &info->rmpp_cntxt_pool[i]);
   }
   info->rmpp_cntxt_nfree = info->rmpp_max_cntxt; 
   info->rmpp_cntxt_nalloc = 0; 
   
   /* 
    * calculate request time to live on queue
    * ~ 3.2secs for defaults: sa_packetLifetime=18 and sa_respTimeValue=18 
    */
   rmpp_reqTimeToLive = 4ull * ((2 * (1 << rmpp_packetLifetime)) + (1 << rmpp_respTimeValue)); 
   
   // initialize context pool lock
   status = vs_lock_init(&info->rmpp_cntxt_lock, VLOCK_FREE, VLOCK_THREAD); 
   if (status != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, "can't initialize context pool lock rc: %d", status); 
      return VSTATUS_BAD;
   }
   
   //	allocate the data storage pool.
   info->rmpp_data_length = data_length;
   status = vs_pool_alloc(info->rmpp_pool, info->rmpp_data_length, (void *)&info->rmpp_data); 
   if (status != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, "can't allocate data"); 
      return VSTATUS_NOMEM;
   }
   
   // open a channel that is used by the user to handle RMPP responses and acks
   if ((status = mai_open(qp, dev, port, &info->rmpp_fd_w)) != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__, "can't open mai connection"); 
      return VSTATUS_BAD;
   }
   
   if (wtName == NULL)
      return status;

   //
   //	start the RMPP writer thread.
   
   //	sleep for half a second to let the previous thread startup.
   (void)vs_thread_sleep(VTIMER_1S / 2); 
   
   g_usrId = (uint8_t)info->usrId; 
   argv[0] = &g_usrId;
   StringCopy(info->wt_thrd_name, wtName, sizeof(info->wt_thrd_name)); 

   IB_LOG_INFINI_INFO_FMT(__func__, "Starting RMPP Writer thread for user %d", g_usrId); 
   if ((status = vs_thread_create(&info->wt_thrd, (unsigned char *)thread_id, 
                                  rmpp_main_writer, 1, argv, RMPP_STACK_SIZE)) != VSTATUS_OK) 
      IB_LOG_ERROR_FMT(__func__, "failed to start RMPP Writer thread");
    
   return status;
}

static Status_t
rmpp_create_filters(rmpp_user_info_t *info, IBhandle_t *fd, IBhandle_t *fh_req_get, IBhandle_t *fh_req_gettable, uint8_t mclass) 
{ 
   int rc = VSTATUS_BAD; 
   
   if (!info) {
      IB_LOG_ERROR_FMT(__func__, "user info is NULL!"); 
      return VSTATUS_BAD;
   } else if (!fd) {
      IB_LOG_ERROR_FMT(__func__, "IF3 connection handle is NULL!"); 
      return VSTATUS_BAD;
   }
   
   // create the filters that would allow us to recieve RMPP related requests on the handle
   if (*fd <= 0)
       IB_LOG_ERROR_FMT(__func__, "invalid handle %"PRIdN, *fd );
   else {
      // creation of filters is optional; these filters are not necessary
      // for threads not interested in receiving MAD requests (i.e., FE) 
      if (!fh_req_get || !fh_req_gettable) {
          info->fd = fd; 
          info->mclass = mclass;
          return VSTATUS_OK; 
      }

      if ((rc = mai_filter_method(*fd, VFILTER_SHARE, MAI_TYPE_ANY, fh_req_get, mclass, RMPP_CMD_GET)) != VSTATUS_OK) {
         IB_LOG_ERROR_FMT(__func__, "can't create filter for RMPP_CMD_GET for user %d: rc %d", info->usrId, rc);
      }
      
      if (rc == VSTATUS_OK) {
         if ((rc = mai_filter_method(*fd, VFILTER_SHARE, MAI_TYPE_ANY, fh_req_gettable, mclass, RMPP_CMD_GETTABLE)) != VSTATUS_OK) {
            mai_filter_hdelete(*fd, *fh_req_get); 
            IB_LOG_ERROR_FMT(__func__, "can't create filter for RMPP_CMD_GETTABLE for user %d: rc %d", info->usrId, rc);
         } else {
            info->fd = fd; 
            info->mclass = mclass; 
            info->fh_req_get = fh_req_get; 
            info->fh_req_gettable = fh_req_gettable;
         }
      }
   }
   
   return rc;
}

void
rmpp_delete_filters(rmpp_user_info_t *info) 
{ 
   
   if (!info) {
      IB_LOG_ERROR_FMT(__func__, "user info is NULL!"); 
   } else if (info->fd && *info->fd > 0) {
      if (info->fh_req_get) 
         mai_filter_hdelete(*info->fd, *info->fh_req_get); 
      if (info->fh_req_gettable) 
         mai_filter_hdelete(*info->fd, *info->fh_req_gettable);
   }
}

void 
rmpp_deinit(void)
{ 
    if (rmpp_initialized) {       
        // reset RMPP related global variables        
        rmpp_userCount = 0; 
        rmpp_initialized = 0;
        memset(&rmpp_users, 0, sizeof(rmpp_users));

        // deallocate the RMPP lock
        vs_lock_delete(&rmpp_user_lock);
    }
}

int rmpp_mngr_open_cnx (
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
   char *wtName
   )
{
   return rmpp_open_cnx(fd, qp, dev, port, pool, data_length,
                        max_cntxt, fh_req_get, fh_req_gettable,
                        mclass, get_method_text, get_aid_name,
                        pre_process_request, pre_process_response,
                        rmpp_is_master, vieo_mod_id, RMPP_FORMAT_SA,
                        wtName);
}

int
rmpp_open_cnx(
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
   ) 
{ 
   Status_t status = VSTATUS_NOHANDLE; 
   int usrId; 
   rmpp_user_info_t *info = NULL; 
   
   IB_ENTER(__func__, 0, 0, 0, 0); 
   
   // if necessary, initialize global variables
   if (!rmpp_initialized) {
      // create necessary semaphore lock(s)
      if ((status = vs_lock_init(&rmpp_user_lock, VLOCK_FREE, VLOCK_THREAD)) != VSTATUS_OK) {
         IB_LOG_ERROR_FMT(__func__, "can't initialize user table lock"); 
         return -1;
      }
      
      // reset necessary global variables
      rmpp_initialized = 1; 
      rmpp_userCount = 0; 
      memset(&rmpp_users, 0, sizeof(rmpp_users));
   }
   if (!fd) {
      IB_LOG_ERROR_FMT(__func__, "IF3 connection handle is NULL!"); 
      return -1;
   }
   
   // check whether connection has already been established
   if ((-1 != (usrId = rmpp_is_cnx_open(fd))) && (info = rmpp_get_userinfo(usrId))) {
       if (info->partial_close) {
           if (if3DebugRmpp) 
               IB_LOG_WARN_FMT(__func__, "RMPP connection already openned for user %d", usrId); 
           // simply create the filters for a partially open RMPP connection
           if ((status = rmpp_create_filters(info, fd, fh_req_get, fh_req_gettable, mclass)) == VSTATUS_OK) {
               info->partial_close = 0;
               return usrId;
           } else {
               IB_LOG_ERROR_FMT(__func__, "failed to create filter in partial open of RMPP connection"); 
               return -1;
           }
       }
   }
   
   // get ID for new RMPP user
   usrId = rmpp_get_new_user(); 
   
   // initialize new RMPP user
   if (usrId != -1 && (info = rmpp_get_userinfo(usrId))) {
      // Set RMPP Format (either SA or vendor)
      info->rmpp_format = rmpp_format;

      // create filters to enable the reception of RMPP related requests
      if ((status = rmpp_create_filters(info, fd, fh_req_get, fh_req_gettable, mclass)))
          IB_LOG_ERROR_FMT(__func__, "failed to create filter in open of RMPP connection"); 
      else {
         // initialize PMPP Protocol related structures to be associated with the user
          if ((status = rmpp_protocol_init(info, pool, data_length, max_cntxt, 
                                           qp, dev, port, 
                                           get_method_text, get_aid_name, 
                                           pre_process_request, pre_process_response, 
                                           rmpp_is_master, vieo_mod_id, wtName))) 
             IB_LOG_ERROR_FMT(__func__, "failed to initialize RMPP connection");
      }
      
      // if initialization failed, deallocate new RMPP user
      if (status) 
         (void)rmpp_close_cnx(usrId, TRUE);
   } else {
       IB_LOG_ERROR_FMT(__func__, "failed to create new user");
   }
   
   IB_EXIT(__func__, status); 
   
   return (status) ? -1 : usrId;
}

void 
rmpp_main_kill(rmpp_user_info_t *info) 
{ 
    if (info) 
        info->wt_thrd_exit = 1;
}

static void
rmpp_remove_cnx(rmpp_user_info_t *info)
{
    if (info) {
        (void)rmpp_delete_filters(info); 
        // deallocate all outstanding context requests
        (void)rmpp_cntxt_free(info); 
        // deallocate context pool
        vs_pool_free(info->rmpp_pool, (void *)info->rmpp_cntxt_pool); 
        // deallocate the data storage pool
        vs_pool_free(info->rmpp_pool, (void *)info->rmpp_data); 
        // reset user structure
        (void)rmpp_delete_user(info);
    }
}

void
rmpp_close_cnx(int usrId, uint8_t complete)
{
    rmpp_user_info_t *info = NULL; 
    
    if (!(info = rmpp_get_userinfo(usrId))) 
        IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", usrId); 
    else {
        uint8_t retry; 
        
        if (!complete) {
            // paritial close of a connection 
            (void)rmpp_delete_filters(info); 
            info->partial_close = 1; 
            return;
        }
        
        if (strlen(info->wt_thrd_name) == 0) {
            // elminate all resources used the RMPP connection
            (void)rmpp_remove_cnx(info);
        } else {
            // submit request to terminate RMPP writer thread
            (void)rmpp_main_kill(info); 

            for (retry = 0; retry < 5; retry++) {
                vs_thread_sleep(VTIMER_1S); 

                // clean up user related resources
                if (info->wt_thrd_exit > 1) {
                    // elminate all resources used the RMPP connection
                    (void)rmpp_remove_cnx(info);
                    break;
                }
            }
        }
    }
}

void
rmpp_mngr_close_cnx(ManagerInfo_t *mi, uint8_t complete)
{
    int usrId = -1;
    
    if (mi) {
        // setup RMPP related fields 
        if (!if3_set_rmpp_minfo(mi)) {
            if (mi->rmppMngrfd && ((usrId = rmpp_is_cnx_open(mi->rmppMngrfd)) != -1))
                (void)rmpp_close_cnx(usrId, complete);
        }
    }
}

static Status_t
rmpp_process_response(int usrId, Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt) 
{ 
   Status_t status;
   rmpp_user_info_t *info = NULL; 
   
   IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0); 
   
   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return VSTATUS_BAD;
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
      return VSTATUS_BAD;
   }
   
   // validate the MAD received.  If it is not valid, just drop it.
   if ((status = rmpp_validate_response_mad(info, maip)) != VSTATUS_OK) {
      IB_EXIT(__func__, status); 
      return (status);
   }
   
   // use primary mai handle for sending out 1st packet of responses
   if (rmpp_cntxt) 
      rmpp_cntxt->sendFd = *info->fd; 
   
   // process specific command request.
   (void)rmpp_data_offset(info, maip->base.method);
     
    if (VSTATUS_ILLPARM == info->rmpp_pre_process_response(maip, rmpp_cntxt)) {
        IB_LOG_INFINI_INFO_FMT(__func__, 
                               "Unsupported or invalid %s[%s] response from LID [0x%x], TID["FMT_U64"]", 
                             info->rmpp_get_method_text((int)maip->base.method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.slid, maip->base.tid); 
      maip->base.status = MAD_STATUS_SA_REQ_INVALID; 
      IB_EXIT(__func__, VSTATUS_ILLPARM); 
      return (VSTATUS_ILLPARM);
   }

    // for now, always assume processing a multi-packet response
    (void)rmpp_process_getmulti_response(maip, rmpp_cntxt); 
    
    IB_EXIT(__func__, VSTATUS_OK); 
   return (status);
}

Status_t
rmpp_pre_process_request(int usrId, Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt) 
{ 
   rmpp_user_info_t *info = NULL; 
   uint64_t startTime = 0, endTime = 0; 
   
   IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0); 
   
   // performance timestamp
   if (if3DebugRmpp) 
      (void)vs_time_get(&startTime);
    
   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
      return VSTATUS_BAD;
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
      return VSTATUS_BAD;
   } else if (!maip) {
      IB_LOG_ERROR_FMT(__func__, "handle is NULL!"); 
      return VSTATUS_BAD;
   }
   
   // validate the MAD received.  If it is not valid, just drop it.
   if (maip->base.cversion > STL_SA_CLASS_VERSION) {
      IB_LOG_ERROR_FMT(__func__, 
                      "Invalid Class Version %d received in %s[%s] request from LID [0x%x], TID ["FMT_U64"], ignoring request!", 
                      maip->base.cversion, info->rmpp_get_method_text((int)maip->base.method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.slid, maip->base.tid); 
      IB_EXIT(__func__, VSTATUS_BAD_VERSION); 
      return VSTATUS_BAD_VERSION;
   }
   
   // use primary mai handle for sending out 1st packet of request
   rmpp_cntxt->sendFd = *info->fd; 
   
   // process specific command request.
   (void)rmpp_data_offset(info, maip->base.method);
    
   if (info->rmpp_pre_process_request(maip, rmpp_cntxt)) {
       IB_LOG_INFINI_INFO_FMT(__func__, 
                             "Unsupported or invalid %s[%s] request from LID [0x%x], TID["FMT_U64"]", 
                             info->rmpp_get_method_text((int)maip->base.method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.slid, maip->base.tid); 
      return VSTATUS_ILLPARM;
   }

   if (if3DebugRmpp) {
      // use the time received from umadt as start time if available
      startTime = (maip->intime) ? maip->intime : startTime; 
      (void)vs_time_get(&endTime); 
      IB_LOG_INFINI_INFO_FMT(__func__, 
                             "%ld microseconds to process %s[%s] request from LID 0x%.4X, TID="FMT_U64, 
                             (long)(endTime - startTime), info->rmpp_get_method_text((int)rmpp_cntxt->method), 
                             info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.dlid, maip->base.tid);
   }
   
   IB_EXIT(__func__, VSTATUS_OK); 
   return (VSTATUS_OK);
}

rmpp_cntxt_t* 
rmpp_mngr_get_cmd(int usrId, Mai_t *mad, uint8_t *processMad) 
{ 
   rmpp_cntxt_t *rmpp_cntxt = NULL; 
   uint64_t now, delta, max_delta; 
   int	tries = 0, retry = 0; 
   rmpp_context_get_t  cntxGetStatus = 0; 
   int numContextBusy = 0; 
   STL_SA_MAD_HEADER samad; 
   rmpp_user_info_t *info = rmpp_get_userinfo(usrId); 
   
   IB_ENTER(__func__, mad, processMad, 0, 0); 
   
   if (mad == NULL || processMad == NULL) {
      IB_EXIT(__func__, rmpp_cntxt); 
      return rmpp_cntxt;
   } else if (!info) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", usrId); 
      return rmpp_cntxt;
   }
   
   switch (mad->base.method) {
   case RMPP_CMD_GET:
   case RMPP_CMD_GETTABLE:
      // drop inflight rmpp requests these are handled by the rmpp writer task
      if (mad->base.method == RMPP_CMD_GETTABLE) {
         // check for inflight rmpp
         (void)BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER *)mad->data, (STL_SA_MAD_HEADER *)&samad);
         if ((samad.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE) && 
             ((samad.rmppType == RMPP_TYPE_ACK && samad.segNum != 0)
              || 
              samad.rmppType == RMPP_TYPE_STOP || samad.rmppType == RMPP_TYPE_ABORT)) {
            if (if3DebugRmpp) {
               IB_LOG_INFINI_INFO_FMT(__func__, 
                                      "Ignoring inflight request for %s[%s] from LID[0x%x], TID="FMT_U64, 
                                      info->rmpp_get_method_text((int)mad->base.method), info->rmpp_get_aid_name((int)mad->base.mclass, (int)mad->base.aid), mad->addrInfo.slid, mad->base.tid);
            }
            /* ignore inflight request */
            *processMad = FALSE; 
            return &ignoreCntxt;    // not processing inflight rmpp stuff either
         }
      }
      
      if (if3DebugRmpp) {
         IB_LOG_INFINI_INFO_FMT(__func__, 
                                "Processing request for %s[%s] from LID[0x%x], TID="FMT_U64, 
                                info->rmpp_get_method_text((int)mad->base.method), info->rmpp_get_aid_name((int)mad->base.mclass, (int)mad->base.aid), mad->addrInfo.slid, mad->base.tid);
      }
      // Drop new requests that have been sitting on PM receive queue for too long
      if (mad->intime) {
         /* PR 110586 - On some RHEL 5 systems, we've seen  weird issues with gettimeofday() [used by vs_time_get()]
          * where once in a while the time difference calculated from successive calls to gettimeofday()
          * results in a negative value. Due to this, we might actually consider a request stale even if
          * its not. Work around this by making calls to gettimeofday() till it returns us some
          * sane values. Just to be extra cautious, bound the retries so that we don't get stuck in the loop.  
          */
         tries = 0; 
         /* Along with negative values also check for unreasonably high values of delta*/
         max_delta = 30 * rmpp_reqTimeToLive; 
         do {
            vs_time_get(&now); 
            delta = now - mad->intime; 
            tries++; 
            
            if ((now < mad->intime) || (delta > max_delta)) {
               vs_thread_sleep(1); 
               retry = 1;
            } else {
               retry = 0;
            }	
         } 
         while (retry && tries < 20); 
         
         if (delta > rmpp_reqTimeToLive) {
            //INCREMENT_PM_COUNTER(pmCounterPaDroppedRequests);
            if (if3DebugRmpp) {
               IB_LOG_INFINI_INFO_FMT(__func__, 
                                      "Dropping stale %s[%s] request from LID[0x%x], TID="FMT_U64"; On queue for %d.%d seconds.", 
                                      info->rmpp_get_method_text((int)mad->base.method), info->rmpp_get_aid_name((int)mad->base.mclass, (int)mad->base.aid), mad->addrInfo.slid, 
                                      mad->base.tid, (int)(delta / 1000000), (int)((delta - delta / 1000000 * 1000000)) / 1000);
            }
            /* drop the request without returning a response; sender will retry */
            *processMad = FALSE; 
            return &ignoreCntxt;
         }
      }
      /* 
       * get a context to process request; rmpp_cntxt can be:
       *   1. NULL if resources are scarce
       *   2. NULL if request is dup of existing request
       *   3. in progress getMulti request context
       *   4. New context for a brand new request 
       */
      cntxGetStatus = cntxt_get(info, mad, (void *)&rmpp_cntxt); 
      if (cntxGetStatus == ContextAllocated) {
         /* process the new request */
         *processMad = TRUE; 
         //rmpp_process_mad(&in_mad, rmpp_cntxt);
         // this may not necessarily release context based on if someone else has reserved it
         //if(rmpp_cntxt)
         //    rmpp_cntxt_release( rmpp_cntxt );
      } else if (cntxGetStatus == ContextExist) {
         /* this is a duplicate request */
         *processMad = FALSE; 
         
         if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "received duplicate %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                                   info->rmpp_get_method_text((int)mad->base.method), info->rmpp_get_aid_name((int)mad->base.mclass, (int)mad->base.aid), mad->addrInfo.slid, mad->base.tid);
         }
      } else if (cntxGetStatus == ContextNotAvailable) {
         /* we are swamped, return BUSY to caller */
         *processMad = TRUE; 
         if (if3DebugRmpp) { /* log msg before send changes method and lids */
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "NO CONTEXT AVAILABLE, returning MAD_STATUS_BUSY to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!", 
                                   info->rmpp_get_method_text((int)mad->base.method), info->rmpp_get_aid_name((int)mad->base.mclass, (int)mad->base.aid), mad->addrInfo.slid, mad->base.tid);
         }
         mad->base.status = MAD_STATUS_BUSY; 
         //rmpp_send_reply( &in_mad, rmpp_cntxt );
         if ((++numContextBusy % info->rmpp_max_cntxt) == 0) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "Had to drop %d PA requests since start due to no available contexts", 
                                   numContextBusy);
         }
      } else {
         IB_LOG_ERROR_FMT(__func__, "Invalid rmpp_cntxt_get return code: %d", cntxGetStatus);
      }
      break;
   }
   
   IB_EXIT(__func__, rmpp_cntxt); 
   return rmpp_cntxt;
}

rmpp_cntxt_t* 
rmpp_cntxt_get(int usrId, Mai_t *mad, uint8_t *processMad) 
{ 
   rmpp_cntxt_t *rmpp_cntxt = NULL; 
   rmpp_context_get_t  cntxGetStatus = 0; 
   int numContextBusy = 0; 
   rmpp_user_info_t *info = rmpp_get_userinfo(usrId); 
   
   IB_ENTER(__func__, mad, processMad, 0, 0); 
   
   if (mad == NULL || processMad == NULL) {
      IB_LOG_ERROR_FMT(__func__, "NULL parameter specified!"); 
      IB_EXIT(__func__, rmpp_cntxt); 
      return rmpp_cntxt;
   } else if (!info) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", usrId); 
      return rmpp_cntxt;
   }
   
   switch (mad->base.method) {
   case RMPP_CMD_GET:
   case RMPP_CMD_SET:
   case RMPP_CMD_GETTABLE:
   case RMPP_CMD_GETTRACETABLE:
   case RMPP_CMD_DELETE:

      if (if3DebugRmpp) {
         IB_LOG_INFINI_INFO_FMT(__func__, 
                                "Processing request for %s[%s] from LID[0x%x], TID="FMT_U64, 
                                info->rmpp_get_method_text((int)mad->base.method), info->rmpp_get_aid_name((int)mad->base.mclass, (int)mad->base.aid), mad->addrInfo.slid, mad->base.tid);
      }

      //
      // get a context to process request; rmpp_cntxt can be:
      //   1. new context for a brand new request 
      //   2. NULL if request is dup of existing request
      //   3. NULL if resources are scarce
      cntxGetStatus = cntxt_get(info, mad, (void *)&rmpp_cntxt); 
      if (cntxGetStatus == ContextAllocated) {
         // process the new request
         *processMad = TRUE; 
      } else if (cntxGetStatus == ContextExist) {
         // this is a duplicate request
         *processMad = FALSE;          
         if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "received duplicate %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                                   info->rmpp_get_method_text((int)mad->base.method), info->rmpp_get_aid_name((int)mad->base.mclass, (int)mad->base.aid), mad->addrInfo.slid, mad->base.tid);
         }
      } else if (cntxGetStatus == ContextNotAvailable) {
         // we are swamped, return BUSY to caller
         *processMad = TRUE; 
         if (if3DebugRmpp) { /* log msg before send changes method and lids */
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "NO CONTEXT AVAILABLE, returning MAD_STATUS_BUSY to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!", 
                                   info->rmpp_get_method_text((int)mad->base.method), info->rmpp_get_aid_name((int)mad->base.mclass, (int)mad->base.aid), mad->addrInfo.slid, mad->base.tid);
         }
         mad->base.status = MAD_STATUS_BUSY; 
         if ((++numContextBusy % info->rmpp_max_cntxt) == 0) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "Had to drop %d PA requests since start due to no available contexts", 
                                   numContextBusy);
         }
      } else {
         IB_LOG_ERROR_FMT(__func__, "Invalid rmpp_cntxt_get return code: %d", cntxGetStatus);
      }
      break;

   default:
       IB_LOG_ERROR_FMT(__func__, "Method 0x%x not supported for RMPP", mad->base.method);
      break;
   }
   
   IB_EXIT(__func__, rmpp_cntxt); 
   return rmpp_cntxt;
}

static Status_t rmpp_receive_getmuli_response_send_ack(Mai_t *maip, STL_SA_MAD *samad, rmpp_cntxt_t* rmpp_cntxt, uint16_t wsize) {
    Status_t    rc=VSTATUS_OK;
    STL_LID     lid;
    rmpp_user_info_t *info = NULL; 
    IBhandle_t  fd;

    if (!rmpp_cntxt) {
       IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
       return VSTATUS_BAD;
    } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
       IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
       return VSTATUS_BAD;
    }

    // validate the MAD received.  If it is not valid, just drop it.
    if (rmpp_validate_response_mad(info, maip) != VSTATUS_OK) {
       IB_EXIT(__func__, VSTATUS_OK); 
       return (VSTATUS_OK);
    }

    // set the mai handle to use for sending - use primary handle if no send handle
    fd = (rmpp_cntxt && rmpp_cntxt->sendFd) ? rmpp_cntxt->sendFd : *info->fd;

    samad->header.rmppType = RMPP_TYPE_ACK;
    // set NewWindowLast (next ACK)
    if (samad->header.u.tf.rmppFlags & RMPP_FLAGS_LAST) {
        // last segment, leave ack window alone
        samad->header.length = samad->header.segNum;
    } else if (wsize == 1) {
        samad->header.length = samad->header.segNum + 1;
    } else {
        // increment ack window
        samad->header.length = MIN(((samad->header.segNum == 1) ? wsize : (samad->header.segNum + wsize)), rmpp_cntxt->segTotal);
    }
    samad->header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
    if (if3DebugRmpp) {
        IB_LOG_INFINI_INFO_FMT(__func__, 
               "ACK window[%d] reached for LID[0x%x] TID="FMT_U64", New ACK window[%d] timeFlag[0x%x] RmppFlags[0x%x, %s]",
               samad->header.segNum, (int)maip->addrInfo.slid, maip->base.tid, samad->header.length,
               samad->header.u.timeFlag, samad->header.u.tf.rmppFlags, (samad->header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE) ? "ACTIVE" : "NULL");
    }
    // set the desired response time value for client to use
    if (samad->header.segNum == 1)
        samad->header.u.tf.rmppRespTime = rmpp_respTimeValue;    // just do it once
    else
        samad->header.u.tf.rmppRespTime = 0x1f;     // none supplied

    // flip the source and dest lids before sending
    samad->header.rmppStatus = 0;
    lid = maip->addrInfo.slid;
    maip->addrInfo.slid = maip->addrInfo.dlid;
    maip->addrInfo.dlid = lid;
    maip->base.method ^= MAD_CM_REPLY;  // send as response
    if (if3DebugRmpp) {
       IB_LOG_INFINI_INFO_FMT(__func__, 
                              "sending ACK to %s request to LID[0x%x] for TID["FMT_U64"], mclass=0x%x\n", 
                              info->rmpp_get_method_text(maip->base.method), (int)maip->addrInfo.dlid, maip->base.tid, maip->base.mclass);
    }

    (void)BSWAPCOPY_STL_SA_MAD(samad, (STL_SA_MAD*)maip->data, STL_SA_DATA_LEN);

    if ((rc = mai_send(fd, maip)) == VSTATUS_OK) {
        // update lastSegAcked
        rmpp_cntxt->last_ack = samad->header.segNum;
    }
    IB_EXIT(__func__,rc);
    return rc;
}

static Status_t
rmpp_receive_getmulti_response(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
    STL_SA_MAD samad; 
    uint16_t bytesRcvd = 0, badPayloadLen = 0; 
    uint16_t sendStopAbort = 0; // 2=ABORT, 1=STOP
    uint16_t wsize = 1;         // ACK window of 1
    STL_LID  lid; 
    uint32_t len; 
    rmpp_user_info_t *info = NULL; 
    Status_t rc = VSTATUS_OK; 
    IBhandle_t fd;
    uint32_t rmppDataLen = (maip->base.cversion == STL_SA_CLASS_VERSION) ? STL_SA_DATA_LEN : IB_SA_DATA_LEN;
    
    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0); 
    
    if (!rmpp_cntxt) {
        IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
        return VSTATUS_BAD;
    } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
        IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
        return VSTATUS_BAD;
    }

    if (rmpp_cntxt->info->rmpp_format == RMPP_FORMAT_VENDOR) {
        return (rmpp_process_multi_response(maip, rmpp_cntxt));
    }
    
    // validate the MAD received.  If it is not valid, just drop it.
    if (rmpp_validate_response_mad(info, maip) != VSTATUS_OK) {
        IB_EXIT(__func__, VSTATUS_OK); 
        return (VSTATUS_OK);
    }
    
    // set the mai handle to use for sending - use primary handle if no send handle
    fd = (rmpp_cntxt->sendFd) ? rmpp_cntxt->sendFd : *info->fd; 
    
    if (if3DebugRmpp)
        IB_LOG_INFINI_INFO_FMT(__func__, "Data length inuse %d, bversion=0x%x, cversion=0x%x",
                               rmppDataLen, maip->base.bversion, maip->base.cversion);
        
    (void)BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)maip->data, &samad, rmppDataLen);
    
    /* check for possible non-rmpp GetMulti request */
    if (samad.header.rmppType == RMPP_TYPE_NOT && rmpp_cntxt->hashed == 0) {
        /* single packet non RMPP request */
        rmpp_cntxt->method = maip->base.method; 
        rmpp_cntxt->segTotal = 1; 
        rmpp_cntxt->reqInProg = 0; 
        memcpy(&rmpp_cntxt->mad, maip, sizeof(Mai_t)); 
        rmpp_cntxt->reqDataLen = rmppDataLen; 
        /* allocate the space for the request */
        rc = vs_pool_alloc(info->rmpp_pool, rmpp_cntxt->reqDataLen, (void *)&rmpp_cntxt->reqData); 
        if (!rc) {
            /* move IB_SA_DATA_LEN of data from first packet into buffer */
            memcpy(rmpp_cntxt->reqData, samad.data, rmpp_cntxt->reqDataLen/*IB_SA_DATA_LEN*/); 
            /* get the request to processing routine */
            if (rmpp_cntxt->processFunc) 
                rc = rmpp_cntxt->processFunc((Mai_t *)&rmpp_cntxt->mad, rmpp_cntxt); 
            if (if3DebugRmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__, 
                                       "Non RMPP GetMulti from Lid[%d], TID="FMT_U64" processed", 
                                       (int)maip->addrInfo.slid, rmpp_cntxt->tid); 
                //INCREMENT_COUNTER(smCounterSaGetMultiNonRmpp);
            }
        } else {
            IB_LOG_ERRORX("couldn't allocate resources for Non RMPP GetMulti request from LID:", rmpp_cntxt->lid);
        }
        IB_EXIT(__func__, rc); 
        return rc;
    } else {
        /*
         * Must be an RMPP request at this point, do the basic RMPP validation of header fields.
         */
        if (samad.header.rmppType == RMPP_TYPE_NOT) {
            // invalid RMPP type
            IB_LOG_WARN_FMT(__func__, "ABORTING - RMPP protocol error; type is NULL from Lid[0x%x] for TID="FMT_U64,
                            (int)maip->addrInfo.slid, maip->base.tid); 
            //INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
            samad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
        } else if (samad.header.rmppVersion != RMPP_VERSION) {
            IB_LOG_WARN_FMT(__func__, "RMPP protocol error, received RMPP Version %d from LID[0x%x] for getMulti TID["FMT_U64"]", 
                            (int)samad.header.rmppVersion, (int)maip->addrInfo.slid, maip->base.tid); 
            //INCREMENT_COUNTER(smCounterRmppStatusAbortUnsupportedVersion);
            samad.header.rmppStatus = RMPP_STATUS_ABORT_UNSUPPORTED_VERSION;
        } else if (!(samad.header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE)) {
            // invalid RMPP type
            IB_LOG_WARN_FMT(__func__, "RMPP protocol error, RMPPFlags.Active bit is NULL from LID[0x%x] for TID["FMT_U64"]  timeFlag[0x%x] RmppFlags[0x%x, %s]", 
                            (int)maip->addrInfo.slid, maip->base.tid, samad.header.u.timeFlag, samad.header.u.tf.rmppFlags, (samad.header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE) ? "ACTIVE" : "NULL"); 
            //INCREMENT_COUNTER(smCounterRmppStatusAbortBadType);
            samad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
        }
        if (samad.header.rmppStatus) {
            samad.header.rmppType = RMPP_TYPE_ABORT; 
            sendStopAbort = 1; 
            goto sendAbort;     //  just send the abort
        }
    }  
    
    /*
     * process the RMPP packet
     */
    if (samad.header.rmppType == RMPP_TYPE_STOP || samad.header.rmppType == RMPP_TYPE_ABORT) {
        /* got a STOP or ABORT */
        if (if3DebugRmpp) {
            IB_LOG_WARN_FMT(__func__, 
                            "Processing STOP OR ABORT with status code[0x%x] from LID[0x%x] for TID["FMT_U64"]", 
                            (int)samad.header.rmppStatus, (int)maip->addrInfo.slid, maip->base.tid); 
            //INCREMENT_COUNTER(smCounterSaRxGetMultiInboundRmppAbort);
        }
        rmpp_cntxt_release(rmpp_cntxt); 
        IB_EXIT(__func__, VSTATUS_OK); 
        return VSTATUS_OK;
    } else if (samad.header.rmppType == RMPP_TYPE_ACK) {
        // process ACK of segment zero from sender at end of request receipt indicating time to process request
        if (samad.header.segNum == 0) {
            // invalid ACK; must be segment zero
            IB_LOG_WARN_FMT(__func__, "ABORTING - Invalid segment number %d in ACK when expecting [0] from LID[0x%x] for TID["FMT_U64"]", 
                            samad.header.segNum, 
                            (int)maip->addrInfo.slid, maip->base.tid); 
            //INCREMENT_COUNTER(smCounterRmppStatusAbortSegnumTooBig);
            samad.header.rmppType = RMPP_TYPE_ABORT; 
            samad.header.rmppStatus = RMPP_STATUS_ABORT_SEGNUM_TOOBIG; 
            sendStopAbort = 1;
        } else if (samad.header.length == 0) {
            // invalid window size
            IB_LOG_WARN_FMT(__func__, "ABORTING - A window size of zero was specified in ACK from LID[0x%x] for TID["FMT_U64"]", 
                            (int)maip->addrInfo.slid, maip->base.tid); 
            //INCREMENT_COUNTER(smCounterRmppStatusAbortNewWindowLastTooSmall);
            samad.header.rmppType = RMPP_TYPE_ABORT; 
            samad.header.rmppStatus = RMPP_STATUS_ABORT_NEWWINDOWLAST_TOOSMALL; 
            sendStopAbort = 1;
        /*} else if (!rmpp_cntxt->reqInProg) {
            // ACK received before end of request
            IB_LOG_WARN_FMT(__func__, "ABORTING - ACK received before finished receiving request from LID[0x%x] for TID["FMT_U64"]", 
                            (int)maip->addrInfo.slid, maip->base.tid); 
            //INCREMENT_COUNTER(smCounterRmppStatusAbortUnspecified);
            samad.header.rmppType = RMPP_TYPE_ABORT; 
            samad.header.rmppStatus = RMPP_STATUS_ABORT_UNSPECIFIED; 
            sendStopAbort = 1;*/
        } else {
            /*
             * OK to process the request now
             * set DS bit, set Window Last to desired, and clear request in progress bit
             */
            rmpp_cntxt->isDS = 1; 
            rmpp_cntxt->WL = samad.header.length; 
            rmpp_cntxt->reqInProg = 0;
        }
    } else {
        // processing DATA packet
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "Processing RMPP GETMULTI request from Lid[0x%x], TID="FMT_U64,
                                   (int)maip->addrInfo.slid, maip->base.tid);
        }
        if (rmpp_cntxt->hashed == 0) {
            /*
             * 	This is the start of request, init/save parameters for context entry
             */
            rmpp_cntxt->method = maip->base.method; 
            rmpp_cntxt->WF = 1;               // window start/first
            rmpp_cntxt->WL = 1;               // window last/end - the one to ACK
            rmpp_cntxt->NS = 0;               // Next packet segment to send
            rmpp_cntxt->ES = 1;               // Expected segment number (Receiver only)
            rmpp_cntxt->last_ack = 0;         // last packet acked by receiver
            rmpp_cntxt->retries = 0;          // current retry count
            rmpp_cntxt->segTotal = 0; 
            rmpp_cntxt->reqInProg = 1; 
            rmpp_cntxt->sendFd = *info->fd;       // use reader mai handle for receiving request
            /* calculate packet and total transaction timeouts */
            rmpp_cntxt->RespTimeout = 4ull * ((2 * (1 << rmpp_packetLifetime)) + (1 << rmpp_respTimeValue)); 
            rmpp_cntxt->tTime = 0;            // for now just use max retries
            memcpy(&rmpp_cntxt->mad, maip, sizeof(Mai_t)); 
            rmpp_cntxt_reserve(rmpp_cntxt); 
            samad.header.rmppVersion = RMPP_VERSION; 
            samad.header.u.tf.rmppRespTime = rmpp_respTimeValue; // use classPortInfo setting
            samad.header.offset = 0;
        } 
        /* see if this is the segment expected. */
        if (samad.header.segNum == rmpp_cntxt->ES) {
            if (samad.header.segNum == 1) {
                /* check to make sure that RMPPFlags.First is set */
                if (!(samad.header.u.tf.rmppFlags & RMPP_FLAGS_FIRST)) {
                    IB_LOG_WARN_FMT(__func__, 
                                    "ABORTING - invalid muli pkt first flag[%d] with segnum 1 from LID[0x%x] for TID["FMT_U64"]", 
                                    samad.header.u.tf.rmppFlags, (int)maip->addrInfo.slid, maip->base.tid); 
                    //INCREMENT_COUNTER(smCounterRmppStatusAbortInconsistentFirstSegnum);
                    rc = RMPP_STATUS_ABORT_INCONSISTENT_FIRST_SEGNUM; 
                    /*  Send an ABORT now */
                    samad.header.rmppType = RMPP_TYPE_ABORT; 
                    samad.header.rmppStatus = RMPP_STATUS_ABORT_INCONSISTENT_FIRST_SEGNUM; 
                    sendStopAbort = 1;
                } else {
                    /* extract the number of bytes in payload and calculate total segments */
                    rc = VSTATUS_OK;
                    len = samad.header.length; 
                    if (len) {
                        rmpp_cntxt->segTotal = (len + rmppDataLen + SA_HEADER_SIZE - 1) / (rmppDataLen + SA_HEADER_SIZE); 
                        len = rmpp_cntxt->reqDataLen = len - (rmpp_cntxt->segTotal * SA_HEADER_SIZE);
                        if (if3DebugRmpp) {
                            IB_LOG_INFINI_INFO_FMT(__func__, 
                                                   "GetMulti from LID[0x%x], TID="FMT_U64", %d total bytes, %d segments, %d data bytes", 
                                                   maip->addrInfo.slid, maip->base.tid, samad.header.length, rmpp_cntxt->segTotal, rmpp_cntxt->reqDataLen);
                        }
                    } else {
                        if (samad.header.u.tf.rmppFlags & RMPP_FLAGS_LAST) {
                            /* send ABORT with status of inconsistent payloadLength */
                            IB_LOG_WARN_FMT(__func__, 
                                            "ABORTING - First and Last segment received with no length from LID[0x%x] for TID["FMT_U64"]", 
                                            (int)maip->addrInfo.slid, maip->base.tid); 
                            //INCREMENT_COUNTER(smCounterRmppStatusAbortInconsistentLastPayloadLength);
                            rc = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH; 
                            samad.header.rmppType = RMPP_TYPE_ABORT; 
                            samad.header.rmppStatus = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH; 
                            sendStopAbort = 1; 
                            goto sendAbort;
                        } else {
                            /* no payload length specified, use max 8184 for GetMulti */
                            len = rmpp_cntxt->reqDataLen = rmppDataLen; 
                            rmpp_cntxt->segTotal = 0; 
                            if (if3DebugRmpp) {
                                IB_LOG_INFINI_INFO0("PayloadLength is not specified, using default max payload");
                            }
                        }
                    }
                    /* allocate the space for the request */
                    if (len)
                        rc = vs_pool_alloc(info->rmpp_pool, len, (void *)&rmpp_cntxt->reqData); 

                    /* send STOP if unable to alloc resources */ 
                    if (rc != VSTATUS_OK) {
                        rmpp_cntxt->reqData = NULL; 
                        rc = RMPP_STATUS_STOP_NORESOURCES; 
                        //INCREMENT_COUNTER(smCounterRmppStatusStopNoresources);
                        /*  Send a STOP now */
                        samad.header.rmppType = RMPP_TYPE_STOP; 
                        samad.header.rmppStatus = RMPP_STATUS_STOP_NORESOURCES; 
                        sendStopAbort = 1;
                    } else {
                        rmpp_cntxt->reqDataLen = len; 
                        if (samad.header.u.tf.rmppFlags & RMPP_FLAGS_LAST) {
                            /* move len indicated of data from first/last packet into buffer */
                            memcpy(rmpp_cntxt->reqData + (samad.header.segNum - 1) * rmppDataLen, samad.data, len);
                        } else {
                            /* move IB_SA_DATA_LEN of data from first packet into buffer */
                            memcpy(rmpp_cntxt->reqData + (samad.header.segNum - 1) * rmppDataLen, samad.data, rmppDataLen); 
                            /* increment next ES */
                            ++rmpp_cntxt->ES;
                        }
                    }
                } /* good first segment received */
            } else {
                /* handle middle/last segments */
                if ((samad.header.u.tf.rmppFlags & RMPP_FLAGS_LAST)) {
                    len = samad.header.length - SA_HEADER_SIZE; 
                    /* We have last packet and the segment numbers line up. We are done */
                    if (if3DebugRmpp) {
                        IB_LOG_INFINI_INFO_FMT(__func__, 
                                               "GetMulti from LID[0x%x], TID="FMT_U64" has %d Total data bytes in last packet segment number %d", 
                                               maip->addrInfo.slid, maip->base.tid, len, samad.header.segNum);
                    }
                    /* abort if pass expected length and is at the last segment */
                    bytesRcvd = (samad.header.segNum - 1) * rmppDataLen + len; 
                    if (bytesRcvd > rmpp_cntxt->reqDataLen) {
                        /* send ABORT with status of inconsistent payloadLength */
                        badPayloadLen = 1;
                    } else {
                        /* move len indicated of data from first/last packet into buffer */
                        memcpy(rmpp_cntxt->reqData + (samad.header.segNum - 1) * rmppDataLen, samad.data, len);
                    }
                } else {
                    /* abort if pass expected length and is a middle segment */
                    bytesRcvd = samad.header.segNum * rmppDataLen; 
                    if (bytesRcvd > rmpp_cntxt->reqDataLen) {
                        /* send ABORT with status of inconsistent payloadLength */
                        badPayloadLen = 1;
                    } else {
                        /* move IB_SA_DATA_LEN of data from packet into buffer */
                        memcpy(rmpp_cntxt->reqData + (samad.header.segNum - 1) * rmppDataLen, samad.data, rmppDataLen); 
                        /* increment next ES */
                        ++rmpp_cntxt->ES;
                    }
                } /* move received data into buffer */
                if (badPayloadLen) {
                    /* send ABORT with status of inconsistent payloadLength */
                    badPayloadLen = 0; 
                    IB_LOG_WARN_FMT(__func__, 
                                    "ABORTING - data received [%d] inconsistent with payloadLength [%d] from LID[0x%x] for TID["FMT_U64"]", 
                                    bytesRcvd, rmpp_cntxt->reqDataLen, (int)maip->addrInfo.slid, maip->base.tid); 
                    //INCREMENT_COUNTER(smCounterRmppStatusAbortInconsistentLastPayloadLength);
                    rc = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH; 
                    samad.header.rmppType = RMPP_TYPE_ABORT; 
                    samad.header.rmppStatus = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH; 
                    sendStopAbort = 1;
                }
            }  /* middle and last segments */
            
            /* ACK the first, last segments and window hits */
            if (((samad.header.segNum % wsize) == 0 || samad.header.segNum == 1 || samad.header.segNum == rmpp_cntxt->segTotal) && !sendStopAbort) {
                if (if3DebugRmpp) {
                    IB_LOG_INFINI_INFO_FMT(__func__, 
                                           "(samad.segNum MOD wsize) = %d, (samad.segNum EQ 1) = %d, (samad.segNum EQ rmpp_cntxt->segTotal) = %d", 
                                           (int)(samad.header.segNum % wsize), (samad.header.segNum == 1), (samad.header.segNum == rmpp_cntxt->segTotal)); 
                }
                
                if (samad.header.segNum == rmpp_cntxt->segTotal) {
                    rmpp_cntxt->reqInProg = 0;
                }
                
                if (samad.header.segNum == 1 &&
                    (samad.header.u.tf.rmppFlags & RMPP_FLAGS_FIRST) &&
                    (samad.header.u.tf.rmppFlags & RMPP_FLAGS_LAST)) {
                    if (if3DebugRmpp)
                        IB_LOG_INFINI_INFO_FMT(__func__, "Received single packet request");
                } else {
                    if (if3DebugRmpp)
                        IB_LOG_INFINI_INFO_FMT(__func__, 
                                               "Received multi-packet request"); 
                }

                if (if3DebugRmpp)
                    IB_LOG_INFINI_INFO_FMT(__func__, 
                                           "Sending ACK with segNum %d, samad.length %d, reqInProg %d, ES %d", 
                                           (int)samad.header.segNum, samad.header.length, rmpp_cntxt->reqInProg, rmpp_cntxt->ES);

                // sent ACK to sender
                if ((rc = rmpp_receive_getmuli_response_send_ack(maip, &samad, rmpp_cntxt, wsize)) != VSTATUS_OK) {
                    IB_LOG_WARN_FMT(__func__, 
                                    "error %d while sending ACK to LID[0x%x] for TID["FMT_U64"], terminating transaction", 
                                    rc, (int)maip->addrInfo.dlid, maip->base.tid); 
                    /* release the context, done with this RMPP xfer */
                    if (rmpp_cntxt) 
                        rmpp_cntxt_release(rmpp_cntxt); 
                    IB_EXIT(__func__, rc); 
                    return rc;
                }
            }  /* ACK window reached */
            
        } else {
            /*
             * got segment number out of sequence
             * we can either not send ACK and sender should retransmit from WF to WL
             * or we send ack of (ES-1) as per figure 178 of receiver main flow diagram
             */
            if (if3DebugRmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__, 
                                       "got seg[%d] when expecting seg[%d] from lID[0x%x] TID="FMT_U64, 
                                       samad.header.segNum, rmpp_cntxt->ES, (int)maip->addrInfo.slid, maip->base.tid);
            }
            /* resend ack of (ES-1) here */
            samad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE; 
            samad.header.segNum = rmpp_cntxt->ES - 1; 
            if ((rc = rmpp_receive_getmuli_response_send_ack(maip, &samad, rmpp_cntxt, wsize)) != VSTATUS_OK) {
                IB_LOG_WARN_FMT(__func__, 
                                "error %d while sending ACK to LID[0x%x] for TID["FMT_U64"]", 
                                rc, (int)maip->addrInfo.dlid, maip->base.tid); 
                /* release the context, done with this RMPP xfer */
                if (rmpp_cntxt) 
                    rmpp_cntxt_release(rmpp_cntxt); 
                IB_EXIT(__func__, rc); 
                return rc;
            }
        }
    } /* good or bad RMPP packet */
    /*
     * Send Abort if desired
     */
sendAbort:
    if (sendStopAbort) {
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                             "Sending ABORT to LID[0x%x] for TID["FMT_U64"]", 
                             (int)maip->addrInfo.dlid, maip->base.tid);
        }
        /*
         *	Setup the out-bound MAD.  We need to reverse the LRH addresses.
         */
        lid = maip->addrInfo.slid; 
        maip->addrInfo.slid = maip->addrInfo.dlid; 
        maip->addrInfo.dlid = lid; 
        maip->base.method |= MAD_CM_REPLY;  // send as response
        samad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE; 
        samad.header.u.timeFlag = 0; 
        samad.header.segNum = 0; 
        samad.header.length = 0; 

        (void)BSWAPCOPY_STL_SA_MAD(&samad, (STL_SA_MAD*)maip->data, rmppDataLen);
        
        if ((rc = mai_send(fd, maip)) != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__, 
                             "error from mai_send while sending ABORT to LID[0x%x] for TID["FMT_U64"]", 
                             (int)maip->addrInfo.dlid, maip->base.tid);
        }
        /* release the context, done with this RMPP xfer */
        rmpp_cntxt_release(rmpp_cntxt);
    }
    
    IB_EXIT(__func__, rc); 
    return (rc);
}

static Status_t
rmpp_process_getmulti_response(Mai_t *maip, rmpp_cntxt_t* rmpp_cntxt)
{
   rmpp_user_info_t *info = NULL; 

    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

    if (!rmpp_cntxt) {
       IB_LOG_ERROR_FMT(__func__, "context is NULL!"); 
       return VSTATUS_BAD;
    } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
       IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId); 
       return VSTATUS_BAD;
    }

    // validate the MAD received.  If it is not valid, just drop it.
    if (rmpp_validate_response_mad(info, maip) != VSTATUS_OK) {
       IB_EXIT(__func__, VSTATUS_OK); 
       return (VSTATUS_OK);
    }

    // process incoming multi-packet response
    (void)rmpp_receive_getmulti_response(maip, rmpp_cntxt);

    IB_EXIT(__func__, VSTATUS_OK);
    return(VSTATUS_OK);
}

Status_t
rmpp_receive_response(int usrId, ManagerInfo_t *mi, Mai_t *maip, uint8_t *buffer, uint32_t *bufferLength, CBTxRxFunc_t cb, void *context)
{ 
    Status_t status; 
    Mai_t mad; 
    rmpp_cntxt_t *rmpp_cntxt = NULL; 
    int retry = 0, firstMad = 1; 
    rmpp_context_get_t cntxGetStatus = 0; 
    int numContextBusy = 0; 
    uint64_t timeout;
    rmpp_user_info_t *info = rmpp_get_userinfo(usrId); 
    
    IB_ENTER(__func__, 0, 0, 0, 0); 
    
    if (mi->cpi.u1.s.RespTimeValue == 0) {
        timeout = RC_MAD_TIMEOUT; 
        if (if3DebugRmpp) 
            IB_LOG_INFINI_INFO_FMT(__func__, "Using default RC_MAD_TIMEOUT = %"PRId64, timeout);
    } else {
        timeout = ((1 << mi->cpi.u1.s.RespTimeValue) * 4ull) + mi->SubnetTO; 
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, "used RespTimeValue=%d, SubnetTO=%d to calculate timeout=%"PRId64, 
                                   (int)mi->cpi.u1.s.RespTimeValue, (int)mi->SubnetTO, timeout);
        }
    }
    mi->timeout = timeout; 
    
    while (1) {
        rmpp_cntxt = NULL;

        // attempt to receive response from manager 
        status = if3_recv(mi, &mad, timeout);       
        if (status != VSTATUS_OK) {
            if (status != VSTATUS_TIMEOUT) {
                IB_LOG_ERROR_FMT(__func__, "Failed to receive response from manager: rc %d", status); 
                status = VSTATUS_BAD; 
                goto bail;
            } else if (retry++ > 3) {
                goto bail;
            }
        } else {
            /* 
             * get a context to process request; rmpp_cntxt can be:
             *   1. NULL if resources are scarce
             *   2. NULL if request is dup of existing request
             *   3. in progress multi packet request context
             *   4. New context for a brand new request 
             */
            cntxGetStatus = cntxt_get(info, &mad, (void *)&rmpp_cntxt); 
            
            if (!rmpp_cntxt) {
                status = VSTATUS_NORESOURCE; 
                IB_LOG_ERROR_FMT(__func__, "Failed to acquire context pool"); 
                goto bail;
            }
            
            if (cntxGetStatus == ContextAllocated) {
                if (if3DebugRmpp) {
                    IB_LOG_INFINI_INFO_FMT(__func__, 
                                           "RMPP READER got response from manager %s[%s] from LID [0x%x] with TID ["FMT_U64"] mclass=0x%x", 
                                           (info?info->rmpp_get_method_text((int)mad.base.method):"<null>"), (info?info->rmpp_get_aid_name((int)mad.base.mclass, (int)mad.base.aid):"<null>"), mad.addrInfo.slid, mad.base.tid, 
                                           mad.base.mclass);
                }

                // retain contents of the first MAD to return back to the
                // caller, because this is the most informative MAD.
                if (firstMad) {
                    firstMad = 0;
                    memcpy(maip, &mad, sizeof(mad));
                }
                
                //
                // process the new response
                status = rmpp_process_response(usrId, &mad, rmpp_cntxt); 
                if (status) {
                    IB_LOG_ERROR_FMT(__func__, 
                                     "RMPP READER failed to process response response from manager %s[%s] from LID [0x%x] with TID ["FMT_U64"] mclass=0x%x", 
                                     (info?info->rmpp_get_method_text((int)mad.base.method):"<null>"), (info?info->rmpp_get_aid_name((int)mad.base.mclass, (int)mad.base.aid):"<null>"), mad.addrInfo.slid, mad.base.tid, 
                                     mad.base.mclass); 
                    goto bail;
                } else {
                    // replicate response data into the return buffer
                    if (rmpp_cntxt->reqInProg) {
                        if (if3DebugRmpp) 
                            IB_LOG_INFINI_INFO_FMT(__func__, 
                                                   "RMPP READER processing multi-packet response from manager %s[%s] from LID [0x%x] with TID ["FMT_U64"] mclass=0x%x", 
                                                   (info?info->rmpp_get_method_text((int)mad.base.method):"<null>"), (info?info->rmpp_get_aid_name((int)mad.base.mclass, (int)mad.base.aid):"<null>"), mad.addrInfo.slid, mad.base.tid, 
                                                   mad.base.mclass);
                    } else {
                        if (if3DebugRmpp) 
                            IB_LOG_INFINI_INFO_FMT(__func__, "RMPP READER received single-packet message, len=%d, bufferLength=%d, reqDataLen=%d", 
                                                   rmpp_cntxt->len, *bufferLength, rmpp_cntxt->reqDataLen); 
                        
                        // move response packet data to return buffer 
                        if (!buffer) 
                            rmpp_call_user_callback(rmpp_cntxt, cb, context); 
                        else if (rmpp_cntxt->reqData) 
                            memcpy(buffer, rmpp_cntxt->reqData, MIN(*bufferLength, rmpp_cntxt->reqDataLen));
                        // set return buffer data length
                        *bufferLength = MIN(*bufferLength, rmpp_cntxt->reqDataLen);
                        // release allocated context 
                        rmpp_cntxt_release(rmpp_cntxt); 
                        goto bail;
                    }
                    rmpp_cntxt_release(rmpp_cntxt);
                }
            } else if (cntxGetStatus == ContextExist) {
                // this is a duplicate request
                if (if3DebugRmpp) {
                    IB_LOG_INFINI_INFO_FMT(__func__, 
                                           "RMPP READER received duplicate %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                                           (info?info->rmpp_get_method_text((int)mad.base.method):"<null>"), (info?info->rmpp_get_aid_name((int)mad.base.mclass, (int)mad.base.aid):"<null>"), mad.addrInfo.slid, mad.base.tid);
                }
            } else if (cntxGetStatus == ContextNotAvailable) {
                // we are swamped, return BUSY to caller
                if (if3DebugRmpp) {
                    // log msg before send changes method and lids
                    IB_LOG_INFINI_INFO_FMT(__func__, 
                                           "RMPP READER NO CONTEXT AVAILABLE, returning MAD_STATUS_BUSY to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!", 
                                           (info?info->rmpp_get_method_text((int)mad.base.method):"<null>"), (info?info->rmpp_get_aid_name((int)mad.base.mclass, (int)mad.base.aid):"<null>"), mad.addrInfo.slid, mad.base.tid);
                }
                mad.base.status = MAD_STATUS_BUSY; 
                rmpp_send_reply(&mad, rmpp_cntxt); 
                ++numContextBusy;
                if (info && ((numContextBusy % info->rmpp_max_cntxt) == 0)) {
                    IB_LOG_INFINI_INFO_FMT(__func__, 
                                           "RMPP READER had to drop %d RMPP requests since start due to no available contexts", 
                                           numContextBusy);
                }
            } else if (cntxGetStatus == ContextExistGetMulti) {
                // continue processing the getMulti request
                if (if3DebugRmpp)
                    IB_LOG_INFINI_INFO_FMT(__func__, 
                                           "RMPP READER got another SEGMENT from manager %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                                           (info?info->rmpp_get_method_text((int)mad.base.method):"<null>"), (info?info->rmpp_get_aid_name((int)mad.base.mclass, (int)mad.base.aid):"<null>"), mad.addrInfo.slid, mad.base.tid); 

                (void)rmpp_process_getmulti_response(&mad, rmpp_cntxt); 
                if (rmpp_cntxt->reqInProg) {
                    if (if3DebugRmpp)
                        IB_LOG_INFINI_INFO_FMT(__func__, 
                                               "RMPP READER processing next multi-packet response from manager %s[%s] from LID [0x%x] with TID ["FMT_U64"] mclass=0x%x", 
                                               (info?info->rmpp_get_method_text((int)mad.base.method):"<null>"), (info?info->rmpp_get_aid_name((int)mad.base.mclass, (int)mad.base.aid):"<null>"), mad.addrInfo.slid, mad.base.tid,
                                               mad.base.mclass);
                } else {
                    if (if3DebugRmpp) 
                        IB_LOG_INFINI_INFO_FMT(__func__, "RMPP READER received multi-packet message, len=%d, bufferLength=%d,reqDataLen=%d", 
                                               rmpp_cntxt->len, *bufferLength, rmpp_cntxt->reqDataLen);

                    // move response packet data to return buffer 
                    if (!buffer) 
                        rmpp_call_user_callback(rmpp_cntxt, cb, context); 
                    else if (rmpp_cntxt->reqData)
                        memcpy(buffer, rmpp_cntxt->reqData, MIN(*bufferLength, rmpp_cntxt->reqDataLen));

                    // set return buffer data length
                    *bufferLength = MIN(*bufferLength, rmpp_cntxt->reqDataLen); 
                    // release allocated context 
                    rmpp_cntxt_release(rmpp_cntxt); 
                    goto bail;
                }

                rmpp_cntxt_release(rmpp_cntxt);
            } else {
                IB_LOG_ERROR_FMT(__func__, "Invalid RMPP cntxt_get return code: %s",
					rmpp_context_get_totext(cntxGetStatus));
            }
            
        }
    }
    
bail:
    if (rmpp_cntxt) 
        rmpp_cntxt_release(rmpp_cntxt); 
    
    return status;
}

int rmpp_gsi_check_packet_filter(Mai_t *maip)
{
    int rc = 0;
    STL_SA_MAD_HEADER mad; 
    rmpp_user_info_t *info = rmpp_get_userinfo(1);  // just always use the first user 

    if (!info) {
       IB_LOG_WARN_FMT(__func__, "failed to get user info for MCLASS 0x%x!", maip->base.mclass); 
       return 1;   // ignore
    }

    memset(&mad, 0, sizeof(mad));

    //The following P_Key related security features shall be supported by all GSI administrators (such as a PA):
    //   - The GSI administrator shall be a full member of the management P_Key (0xffff),
    //     and shall use P_Key 0xffff when interacting with clients for responses, RMPP
    //     responses, Notices, etc.
    //   - All GSI administrator clients shall be full members of the management P_Key,
    //     and shall use P_Key 0xffff when interacting with the GSI Administrator (PA, etc)
    //     for requests, RMPP Acks, Notice Replies, etc. This restriction prevents non-management nodes from
    //     accessing administrative data for the fabric beyond the name services and information which the
    //     SA makes available.
    //   - The GSI administrator shall verify that all inbound requests, RMPP Acks,
    //     Notices replies use P_Key 0xffff and shall discard any packets with other
    //     P_keys without processing them.
    switch (maip->base.mclass) {
    case MAD_CV_SUBN_ADM:
        (void)BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER *)maip->data, (STL_SA_MAD_HEADER *)&mad); 

        if (mad.smKey) 
            rc = 1; 
        else if (!smValidateGsiMadPKey(maip, 0, rmpp_sma_spoofing_check)) 
            rc = 3; 
        else {
            if (maip->addrInfo.pkey != STL_DEFAULT_FM_PKEY) {
                switch (maip->base.aid) {
                case STL_SA_ATTR_PATH_RECORD:
                case STL_SA_ATTR_MULTIPATH_GID_RECORD:
                case STL_SA_ATTR_NODE_RECORD:
                case STL_SA_ATTR_INFORM_INFO_RECORD:
                case STL_SA_ATTR_INFORM_INFO:
                case STL_SA_ATTR_NOTICE:
                case STL_SA_ATTR_SERVICE_RECORD:
                case STL_SA_ATTR_MCMEMBER_RECORD:
                case STL_SA_ATTR_CLASS_PORT_INFO:
                    break; 
                default:
                    rc = 2; 
                    break;
                }
            }
        }
        break;
    case MAD_CV_VENDOR_FE:
    case MAD_CV_PERF:
    case MAD_CV_VFI_PM:
    case MAD_CV_VENDOR_DBSYNC:
        if (!smValidateGsiMadPKey(maip, 1, rmpp_sma_spoofing_check)) // Full Mgmt exchanges with full only.
            rc = 3;
        break;
    default:
        // Unexpected MAD
        rc = 4;
    }

    if (rc && if3DebugRmpp) {
        switch (rc) {
        case 1:
            IB_LOG_WARN_FMT(__func__, 
                            "Dropping packet, invalid SM_Key "FMT_U64" %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                            mad.smKey, info->rmpp_get_method_text((int)maip->base.method),
                            info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.slid, maip->base.tid); 
            break; 
        case 2:
            IB_LOG_WARN_FMT(__func__, 
                            "Dropping packet, invalid limited member attribute 0x%x %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                            maip->base.aid, info->rmpp_get_method_text((int)maip->base.method), 
                            info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.slid, maip->base.tid); 
            break; 
        case 3:
            IB_LOG_WARN_FMT(__func__, 
                            "Dropping packet, invalid P_Key 0x%x %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                            maip->addrInfo.pkey, info->rmpp_get_method_text((int)maip->base.method),
                            info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.slid, maip->base.tid); 
            break; 
        case 4:
        default:
            IB_LOG_WARN_FMT(__func__, 
                            "Dropping packet, Unexpected class 0x%x AID:0x%x %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                            maip->base.mclass, maip->base.aid, info->rmpp_get_method_text((int)maip->base.method), 
                            info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), maip->addrInfo.slid, maip->base.tid); 
            break;
        }
    }
     
    if (rc) {
        // if the MAD is not valid, then indicate to the MAI filter handling
        // code that no match has been found.  Also, change the MAI type to
        // indicate to the MAI filter handling code to just drop the packet.  
        maip->type = MAI_TYPE_DROP;
    }
    
    return (rc) ? 1 : 0;
}

uint32_t rmpp_sma_spoofing_check_get(void)
{
    return rmpp_sma_spoofing_check;
}

void rmpp_sma_spoofing_check_set(uint32_t value)
{
    rmpp_sma_spoofing_check = value;
}

/**
 * @brief rmpp_receive_request - This function
 * receives non-rmpp packets used to
 * initiate an rmpp transaction.
 *
 * @param fd - file descriptor to receive on
 * @param in_mad - MAD packet to store data received
 *
 * @return VSTATUS_OK on success, other on failure
 */
Status_t
rmpp_receive_request(IBhandle_t fd, Mai_t *in_mad)
{
   return mai_recv(fd, in_mad, VTIMER_1S/4);
}

/**
 * @brief rmpp_prepare_response - This function
 * does some needed modification of the rmpp
 * context so that the sending file descriptor
 * correctly points to the writer's file
 * descriptor. This allows the rmpp writer
 * thread to send out the response packets.
 *
 * @param rmpp_cntxt - allocated rmpp context.
 */
void
rmpp_prepare_response(rmpp_cntxt_t *rmpp_cntxt)
{
   rmpp_user_info_t *info = NULL;
   info = rmpp_get_userinfo(rmpp_cntxt->usrId);
   if (info == NULL) {
      IB_LOG_WARN_FMT(__func__, "Couldn't get info for usrId %d", rmpp_cntxt->usrId);
   } else {
      rmpp_cntxt->sendFd = info->rmpp_fd_w;
   }
}

/**
 * @brief rmpp_send_single_vendor - This function is adapted
 * from rmpp_send_single_sa(). The difference between this
 * one and the other is that STL_SA_MAD is replaced
 * with STL_RMPP_MAD. All of the associated STL_SA
 * constants are also replaced by STL_RMPP constants.
 * This function handles RMPP_FORMAT_VENDOR RMPP
 * packets.
 *
 * @param maip - received MAD
 * @param rmpp_cntxt - associated rmpp context.
 *
 * @return - VSTATUS_OK on success, other on failure.
 */
static Status_t
rmpp_send_single_vendor(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
   Status_t status;
   STL_RMPP_VENDOR_PACKET mad;
   rmpp_user_info_t *info = NULL;
   /* set the mai handle to use for sending - use reader handle PM if no context */

   IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

   if (!rmpp_cntxt) {
      IB_LOG_ERROR_FMT(__func__, "context is NULL!");
      return VSTATUS_BAD;
   } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
      IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId);
      return VSTATUS_BAD;
   }

   // send the response MAD.
   (void)BSWAPCOPY_STL_RMPP_VENDOR_PACKET((STL_RMPP_VENDOR_PACKET *)maip->data,
				&mad, STL_RMPP_VENDOR_DATA_LEN);

   mad.header.rmppVersion = 0;
   mad.header.rmppType = 0;
   mad.header.u.timeFlag = 0;
   mad.header.rmppStatus = 0;
   mad.header.segNum = 0;
   mad.header.length = 0;

   (void)memset(mad.data, 0, sizeof(mad.data));
   if ((rmpp_cntxt == NULL) || (rmpp_cntxt->len != 0 && rmpp_cntxt->data == NULL)) {
      if (maip->base.status == MAD_STATUS_OK)
         maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
   } else if (rmpp_cntxt->len != 0) {
      if (rmpp_cntxt->len > sizeof(mad.data)) {
         // caller should have checked this, just to be safe
         IB_LOG_ERROR_FMT(__func__, "mad.data=%"PRISZT" len=%d LEN=%d", sizeof(mad.data), rmpp_cntxt->len, (int)(STL_RMPP_VENDOR_DATA_LEN));
         IB_EXIT(__func__, VSTATUS_OK);
         return (VSTATUS_OK);
      }
      (void)memcpy(mad.data, rmpp_cntxt->data, rmpp_cntxt->len);
   }

   (void)BSWAPCOPY_STL_RMPP_VENDOR_PACKET(&mad,
					  (STL_RMPP_VENDOR_PACKET *)maip->data,
					  STL_RMPP_VENDOR_DATA_LEN);

   if (if3DebugRmpp) {
      IB_LOG_INFINI_INFO_FMT(__func__,
                             "sending reply to %s request to LID[0x%x] for TID["FMT_U64"]\n",
                             info->rmpp_get_method_text(maip->base.method), (int)maip->addrInfo.dlid, maip->base.tid);
   }

   if ((status = mai_send(rmpp_cntxt->sendFd, maip)) != VSTATUS_OK) {
      IB_LOG_ERROR_FMT(__func__,
                       "can't send reply to %s request to LID[0x%x] for TID["FMT_U64"]",
                       info->rmpp_get_method_text(maip->base.method), (int)maip->addrInfo.dlid, maip->base.tid);
      IB_EXIT(__func__, VSTATUS_OK);
      return (VSTATUS_OK);
   }

   IB_EXIT(__func__, VSTATUS_OK);
   return (VSTATUS_OK);
}

/**
 * @brief rmpp_send_multi_vendor - This function is adapted
 * from rmpp_send_multi_sa(). The difference between this
 * one and the other is that STL_SA_MAD is replaced
 * with STL_RMPP_MAD. All of the associated STL_SA
 * constants are also replaced by STL_RMPP constants.
 * The rmpp_cntxt.attribLen is not set because
 * this is handled by the function that packs in
 * the data records instead. Instead of adding the
 * SA header per segment, the vendor OUI is added.
 * This function handles RMPP_FORMAT_VENDOR RMPP packets.
 *
 * Most of the functionality is similiar to
 * rmpp_send_multi_sa because there didn't
 * seem to be a good way of adapting the previous
 * function to handle RMPP sequences for both the
 * SA and Vendor headers. The mad and resp variables in
 * this function would have to be abstracted
 * out, and changing this requires quite a bit
 * work/testing.
 *
 * @param maip - received MAD
 * @param rmpp_cntxt - associated rmpp context.
 *
 * @return - VSTATUS_OK on success, other on failure.
 */
static Status_t
rmpp_send_multi_vendor(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
    int i;
    int wl = 0;
    uint8_t chkSum = 0;
    uint32_t dlen, datalen = sizeof(STL_RMPP_VENDOR_HEADER);
    Status_t status;
    STL_RMPP_VENDOR_PACKET mad;
    STL_RMPP_VENDOR_PACKET resp;
    uint16_t sendAbort = 0;
    uint16_t releaseContext = 1; /* release context here unless we are in resend mode */
    uint64_t tnow, delta, ttemp;
    rmpp_user_info_t *info = NULL;
    size_t rmpp_data_size = STL_RMPP_VENDOR_DATA_LEN;

    IB_ENTER(__func__, maip, rmpp_cntxt, rmpp_cntxt->len, 0);

    if (!rmpp_cntxt) {
        IB_LOG_ERROR_FMT(__func__, "context is NULL!");
        return VSTATUS_BAD;
    } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
        IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!", rmpp_cntxt->usrId);
        return VSTATUS_BAD;
    }

    // Initialize packet data structures on the stack
    memset((void *)&mad, 0, sizeof(mad));
    memset((void *)&resp, 0, sizeof(resp));

    // maip is NULL if Ack timeout
    if (maip) {
        BSWAPCOPY_STL_RMPP_VENDOR_HEADER((STL_RMPP_VENDOR_HEADER *)maip->data,
                                    (STL_RMPP_VENDOR_HEADER *)&resp);

        if (maip->base.bversion == STL_BASE_VERSION)
            rmpp_data_size = MIN(STL_RMPP_VENDOR_DATA_LEN, rmpp_cntxt->len);
    }

    // Check if this is an incoming request
    // normal getTable requests are not hashed while
    // getMulti requests have the isDS bit set
    if (rmpp_cntxt->hashed == 0 || rmpp_cntxt->isDS) {
        /* init/save parameters and reserve context */
        if (maip) {
            memcpy(&rmpp_cntxt->mad, maip, sizeof(Mai_t));
            if (rmpp_cntxt->data) {
                memcpy(rmpp_cntxt->mad.data + sizeof(STL_RMPP_VENDOR_HEADER),
		       rmpp_cntxt->data, MIN(rmpp_data_size, rmpp_cntxt->len));
            }

            /* get original mad with appropriate offset */
            (void)BSWAPCOPY_STL_RMPP_VENDOR_HEADER((STL_RMPP_VENDOR_HEADER *)rmpp_cntxt->mad.data,
						(STL_RMPP_VENDOR_HEADER *)&mad);
        }
        rmpp_cntxt->WF = 1;
        /* use value set in getMulti request ACK */
        if (!rmpp_cntxt->isDS)
            rmpp_cntxt->WL = 1;
        rmpp_cntxt->NS = rmpp_cntxt->WF;  // Next packet segment to send
        rmpp_cntxt->ES = 0;               // Expected segment number (Receiver only)
        rmpp_cntxt->last_ack = 0;         // last packet acked by receiver
        rmpp_cntxt->retries = 0;          // current retry count
        if (rmpp_cntxt->len == 0) {
            rmpp_cntxt->segTotal = 1;
        } else if (rmpp_cntxt->len <= info->rmpp_data_length) {
            rmpp_cntxt->segTotal = (rmpp_data_size) ? ((rmpp_cntxt->len + rmpp_data_size - 1) / rmpp_data_size) : 1;
        } else {
            IB_LOG_WARN("NO RESOURCES--> rmpp_cntxt->len too large:", rmpp_cntxt->len);
            rmpp_cntxt->len = 0;
            mad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
            rmpp_cntxt->segTotal = 1;
            sendAbort = 1;
        }

        /* calculate packet and total transaction timeouts C13-13.1.1 */
        rmpp_cntxt->RespTimeout = 4ull * (1 << 20); // ~4.3 seconds
        rmpp_cntxt->tTime = 0;            // receiver only
        ttemp = rmpp_cntxt->mad.intime;   // save the time from original mad in context
        if (rmpp_cntxt->isDS) {
            rmpp_cntxt->mad.intime = ttemp;   // we want the time when getMulti request really started
            rmpp_cntxt->isDS = 0;
        }

        /* 8-bit cheksum of the rmpp response */
        rmpp_cntxt->chkSum = 0;
        if (rmppCheckSum && rmpp_cntxt->data) {
            for (i = 0; i < rmpp_cntxt->len; i++) {
                rmpp_cntxt->chkSum += rmpp_cntxt->data[i];
            }
        }
        rmpp_cntxt_reserve(rmpp_cntxt);
        mad.header.rmppVersion = RMPP_VERSION;
        mad.header.u.tf.rmppRespTime = 0x1F; // no time provided
        //mad.header.offset = rmpp_cntxt->attribLen / 8; // setup attribute offset for RMPP xfer
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__,
                                   "Lid[0x%x] STARTING RMPP %s[%s] with TID="FMT_U64", CHKSUM[%d]",
                                   (int)rmpp_cntxt->lid, info->rmpp_get_method_text((int)rmpp_cntxt->method),
                                   (maip?info->rmpp_get_aid_name((int)maip->base.mclass, maip->base.aid):"<null>"), rmpp_cntxt->tid, rmpp_cntxt->chkSum);
        }
    } else if (maip) {
        /* get original mad with appropriate offset */
        (void)BSWAPCOPY_STL_RMPP_VENDOR_HEADER((STL_RMPP_VENDOR_HEADER *)rmpp_cntxt->mad.data,
					    (STL_RMPP_VENDOR_HEADER *)&mad);

        /*
           * Check the response and set the context parameters according to it
           */
        if (resp.header.rmppType == RMPP_TYPE_NOT && (resp.header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE)) {
            // invalid RMPP type
            IB_LOG_WARN_FMT(__func__,
                            "ABORTING - RMPP protocol error; RmppType is NULL in %s[%s] from Lid[0x%x] for TID="FMT_U64,
                            info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), (int)rmpp_cntxt->lid, rmpp_cntxt->tid);
            sendAbort = 1;
            mad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
        } else if (resp.header.rmppVersion != RMPP_VERSION) {
            /* ABORT transaction with BadT status */
            sendAbort = 1;
            mad.header.rmppStatus = RMPP_STATUS_ABORT_UNSUPPORTED_VERSION;
            IB_LOG_WARN_FMT(__func__,
                            "ABORTING - Unsupported Version %d in %s[%s] request from LID[0x%x], TID["FMT_U64"]",
                            resp.header.rmppVersion, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), (int)rmpp_cntxt->lid, rmpp_cntxt->tid);
        } else if (!(resp.header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE)) {
            /* invalid RMPP type */
            IB_LOG_WARN_FMT(__func__,
                            "RMPP protocol error, RMPPFlags.Active bit is NULL in %s[%s] from LID[0x%x] for TID["FMT_U64"]",
                            info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid), (int)rmpp_cntxt->lid, rmpp_cntxt->tid);
            mad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
            sendAbort = 1;
        } else if (resp.header.rmppType == RMPP_TYPE_ACK) {
            /* Got an ACK packet from receiver */
            if (resp.header.segNum < rmpp_cntxt->WF) {
                /* silently discard the packet */
                if (if3DebugRmpp) {
                    IB_LOG_INFINI_INFO_FMT(__func__,
                       "LID[0x%x] sent ACK for seg %d which is less than Window First (%d) for TID: "FMT_U64,
                       rmpp_cntxt->lid, (int)resp.header.segNum, (int)rmpp_cntxt->WF,
                       rmpp_cntxt->tid);
                }
            } else if (resp.header.segNum > rmpp_cntxt->WL) {
                /* ABORT the transaction with S2B status */
                sendAbort = 1;
                mad.header.rmppStatus = RMPP_STATUS_ABORT_SEGNUM_TOOBIG;
                IB_LOG_INFINI_INFO_FMT(__func__,
                   "ABORT - LID[0x%x] sent invalid seg %d in ACK, should be <= than %d, ABORTING TID:"FMT_U64,
                   rmpp_cntxt->lid, (int)resp.header.segNum, (int)rmpp_cntxt->WL,
                   rmpp_cntxt->tid);
            } else if (resp.header.length < rmpp_cntxt->WL /*|| resp.header.length > rmpp_cntxt->segTotal*/) {
                /* length is NewWindowLast (NWL) in ACK packet */
                /* ABORT transaction with W2S status */
                sendAbort = 1;
                if (resp.header.length < rmpp_cntxt->WL) {
                    mad.header.rmppStatus = RMPP_STATUS_ABORT_NEWWINDOWLAST_TOOSMALL;
                } else {
                    mad.header.rmppStatus = RMPP_STATUS_ABORT_UNSPECIFIED;
                }
                IB_LOG_INFINI_INFO_FMT(__func__,
                   "ABORT - LID[0x%x] sent invalid NWL %d in ACK, should be >=%d and <=%d, ABORTING TID:"FMT_U64,
                   rmpp_cntxt->lid, (int)resp.header.length, (int)rmpp_cntxt->WL, rmpp_cntxt->segTotal, rmpp_cntxt->tid);
            } else if (resp.header.segNum >= rmpp_cntxt->last_ack) {
                rmpp_cntxt->last_ack = resp.header.segNum;
                rmpp_cntxt->retries = 0;  /* reset the retry count  after receipt of ack */
                /* is it ack of very last packet? */
                if (resp.header.segNum == rmpp_cntxt->segTotal) {
                    /* we are done */
                    if (if3DebugRmpp) {
                        IB_LOG_INFINI_INFO_FMT(__func__,
                                               " Received seg %d ACK, %s[%s] transaction from LID[0x%x], TID["FMT_U64"] has completed",
                                               resp.header.segNum, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid),
                                               rmpp_cntxt->lid, rmpp_cntxt->tid);
                    }
                } else {
                    /* update WF, WL, and NS and resume sends */
                    rmpp_cntxt->WF = rmpp_cntxt->last_ack + 1;
                    rmpp_cntxt->WL = (resp.header.length > rmpp_cntxt->segTotal) ? rmpp_cntxt->segTotal : resp.header.length;
                    /* see if new Response time needs to be calculated */
                    if (resp.header.u.tf.rmppRespTime && resp.header.u.tf.rmppRespTime != 0x1f) {
                        rmpp_cntxt->RespTimeout = 4ull * ((2 * (1 << rmpp_packetLifetime)) + (1 << resp.header.u.tf.rmppRespTime));
                        if (if3DebugRmpp) {
                            IB_LOG_INFINI_INFO_FMT(__func__,
                                                   "LID[0x%x] set RespTimeValue (%d usec) in ACK of seg %d for %s[%s], TID["FMT_U64"]",
                                                   rmpp_cntxt->lid, (int)rmpp_cntxt->RespTimeout, resp.header.segNum,
                                                   info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid),
                                                   rmpp_cntxt->tid);
                        }
                    }
                }
            }
        } else if (resp.header.rmppType == RMPP_TYPE_STOP || resp.header.rmppType == RMPP_TYPE_ABORT) {
            /* got a STOP or ABORT */
            if (if3DebugRmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__,
					"STOP/ABORT received for %s[%s] from LID[0x%x], status code = %x, for TID["FMT_U64"]",
					info->rmpp_get_method_text((int)rmpp_cntxt->method),
					info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid),
					rmpp_cntxt->lid, resp.header.rmppStatus, rmpp_cntxt->tid);
            }
            rmpp_cntxt_release(rmpp_cntxt);
            IB_EXIT(__func__, VSTATUS_OK);
            return VSTATUS_OK;
        } else {
            /* invalid RmppType received */
            IB_LOG_WARN_FMT(__func__,
                            "ABORT - Invalid rmppType %d received for %s[%s] from LID[0x%x] for TID["FMT_U64"]",
                            resp.header.rmppType, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid),
                            rmpp_cntxt->lid, rmpp_cntxt->tid);
            // abort with badtype status
            sendAbort = 1;
            mad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
        }
    } else {
        /* We are timing out, retry till retry  count expires */
        /* get original mad from context with correct offset */
        (void)BSWAPCOPY_STL_RMPP_VENDOR_HEADER((STL_RMPP_VENDOR_HEADER *)rmpp_cntxt->mad.data,
					    (STL_RMPP_VENDOR_HEADER *)&mad);

        ++rmpp_cntxt->retries;
        if (rmpp_cntxt->retries > rmppMaxRetries) {
            if (if3DebugRmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__,
                   "ABORT - MAX RETRIES EXHAUSTED; no ACK for seg %d of %s[%s] request from LID[0x%X], TID = "FMT_U64,
                   (int)rmpp_cntxt->WL, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid), rmpp_cntxt->lid, rmpp_cntxt->tid);
            }
            /* ABORT transaction with too many retries status */
            sendAbort = 1;
            mad.header.rmppStatus = RMPP_STATUS_ABORT_TOO_MANY_RETRIES;
            /* let context_age do the releasing;  It already holds the lock */
            releaseContext = 0;
        } else {
            rmpp_cntxt->NS = rmpp_cntxt->WF;            // reset Next packet segment to send
            if (if3DebugRmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__,
                                       "Timed out waiting for ACK of seg %d of %s[%s] from LID[0x%x], TID["FMT_U64"], retry #%d",
                                       (int)rmpp_cntxt->WL, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid),
                                       rmpp_cntxt->lid, rmpp_cntxt->tid, rmpp_cntxt->retries);
            }
        }
    }

    /* see if we're done (last segment was acked) */
    if (rmpp_cntxt->last_ack == rmpp_cntxt->segTotal && !sendAbort) {
        if (if3DebugRmpp) {
            vs_time_get(&tnow);
            delta = tnow - rmpp_cntxt->mad.intime;
            IB_LOG_INFINI_INFO_FMT(__func__,
                                   "%s[%s] RMPP [CHKSUM=%d] TRANSACTION from LID[0x%x], TID["FMT_U64"] has completed in %d.%.3d seconds (%"CS64"d usecs)",
                                   info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid),
                                   rmpp_cntxt->chkSum, rmpp_cntxt->lid, rmpp_cntxt->tid,
                                   (int)(delta / 1000000), (int)((delta - delta / 1000000 * 1000000)) / 1000, delta);
        }
        /* validate that the 8-bit cheksum of the rmpp response is still the same as when we started */
        if (rmppCheckSum && rmpp_cntxt->data) {
            chkSum = 0;
            for (i = 0; i < rmpp_cntxt->len; i++) {
                chkSum += rmpp_cntxt->data[i];
            }
            if (chkSum != rmpp_cntxt->chkSum) {
                IB_LOG_ERROR_FMT(__func__,
                                 "CHECKSUM FAILED [%d vs %d] for completeted %s[%s] RMPP TRANSACTION from LID[0x%x], TID["FMT_U64"]",
                                 chkSum, rmpp_cntxt->chkSum, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)rmpp_cntxt->mad.base.mclass, (int)rmpp_cntxt->mad.base.aid),
                                 rmpp_cntxt->lid, rmpp_cntxt->tid);
            }
        }
        if (releaseContext)
            rmpp_cntxt_release(rmpp_cntxt);
        IB_EXIT(__func__, VSTATUS_OK);
        return VSTATUS_OK;
    }

    /* we must use the Mad_t that was saved in the context */
    maip = &rmpp_cntxt->mad;
    /*
      * send segments up till Window Last (WL) and wait for ACK
      * Due to possible concurrency issue if reader is pre-empted while
      * sending segement 1 and writer runs to process ACK of seg 1, use
      * a local var for WL.
      * In the future we need to add individual context locking if we intend
      * to go with a pool threads.
      */
    wl = rmpp_cntxt->WL;
    while (rmpp_cntxt->NS <= wl && !sendAbort) {

       /*
        * calculate amount of data length to send in this segment and put in mad;
        * dlen=payloadLength in first packet, dlen=remainder in last packet
        */
        if (rmpp_cntxt->NS == rmpp_cntxt->segTotal) {
            dlen = (rmpp_data_size) ? (rmpp_cntxt->len % rmpp_data_size) : 0;
            dlen = (dlen) ? dlen : rmpp_data_size;
        } else
            dlen = rmpp_data_size;

        if (dlen > info->rmpp_data_length) {
            IB_LOG_WARN("dlen is too large dlen:", dlen);
        }
        // make sure there is data to send; could just be error case with no data
        if (rmpp_cntxt->len && rmpp_cntxt->data) {
            (void)memcpy(mad.data, rmpp_cntxt->data + ((rmpp_cntxt->NS - 1) * rmpp_data_size), dlen);
        }

        mad.header.rmppVersion = RMPP_VERSION;
        mad.header.rmppType = RMPP_TYPE_DATA;
        mad.header.rmppStatus = 0;
        mad.header.segNum = rmpp_cntxt->NS;
        if (rmpp_cntxt->NS == 1 && rmpp_cntxt->NS == rmpp_cntxt->segTotal) {
            // first and last segment to transfer, set length to payload length
            // The OUI is counted as part of the RMPP data length for Vendor classes
            mad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_FIRST | RMPP_FLAGS_LAST;
            mad.header.length = rmpp_cntxt->len + STL_RMPP_SIZEOF_OUI;
        } else if (rmpp_cntxt->NS == 1) {
            // first segment to transfer, set length to payload length
            // The OUI is counted as part of the RMPP data length for Vendor classes,
            // and it is sent per packet.
            mad.header.length = rmpp_cntxt->len + rmpp_cntxt->segTotal*STL_RMPP_SIZEOF_OUI;
            mad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_FIRST;
        } else if (rmpp_cntxt->NS == rmpp_cntxt->segTotal) {
            // last segment to go; len=bytes remaining
            // The OUI is counted as part of the RMPP data length for Vendor classes
            mad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_LAST;
            mad.header.length = dlen + STL_RMPP_SIZEOF_OUI;
            if (mad.header.length == 0) {
                mad.header.length = rmpp_data_size;
            }
        } else {
            /* middle segments */
            mad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
            mad.header.length = 0;
        }
        /* put mad back into Mad_t in the context */
        datalen = sizeof(STL_RMPP_VENDOR_HEADER) + dlen;
        BSWAPCOPY_STL_RMPP_VENDOR_PACKET((STL_RMPP_VENDOR_PACKET *)&mad,
                             (STL_RMPP_VENDOR_PACKET *)maip->data,
                             rmpp_data_size);

        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__,
               "sending fragment %d of %d segments, len (in rmpp header) %d, datalen (to be sent) %d to LID[0x%x] for TID = "FMT_U64,
               (int)mad.header.segNum, rmpp_cntxt->segTotal, (int)mad.header.length, (int)datalen, (int)rmpp_cntxt->lid,
               rmpp_cntxt->tid);
        }
        /* increment NS */
        ++rmpp_cntxt->NS;
        if (maip->base.bversion == IB_BASE_VERSION) {
            status = mai_send(rmpp_cntxt->sendFd, maip);
        } else {
            status = mai_stl_send(rmpp_cntxt->sendFd, maip, &datalen);
        }

        if (status != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__,
                             "mai_send error [%d] while processing %s[%s] request from LID[0x%x], TID["FMT_U64"]",
                             status, info->rmpp_get_method_text((int)rmpp_cntxt->method), info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid),
                             rmpp_cntxt->lid, rmpp_cntxt->tid);
            if (releaseContext)
                rmpp_cntxt_release(rmpp_cntxt);
            IB_EXIT(__func__, VSTATUS_OK);
            return VSTATUS_OK;
        }
    }

    /*
     * Send Abort if desired
     */
    if (sendAbort) {
        mad.header.rmppVersion = RMPP_VERSION;
        mad.header.rmppType = RMPP_TYPE_ABORT;
        mad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
        mad.header.u.tf.rmppRespTime = 0;
        mad.header.segNum = 0;
        mad.header.length = 0;
        (void)BSWAPCOPY_STL_RMPP_VENDOR_HEADER((STL_RMPP_VENDOR_HEADER *)&mad,
					    (STL_RMPP_VENDOR_HEADER *)maip->data);

        if ((status = mai_send(rmpp_cntxt->sendFd, maip)) != VSTATUS_OK)
            IB_LOG_ERROR_FMT(__func__,
				"error[%d] from mai_send while sending ABORT of %s[%s] request to LID[0x%x], TID["FMT_U64"]",
				status, info->rmpp_get_method_text((int)rmpp_cntxt->method),
				info->rmpp_get_aid_name((int)maip->base.mclass, (int)maip->base.aid),
				rmpp_cntxt->lid, maip->base.tid);
        /*
         * We are done with this RMPP xfer.  Release the context here if
         * in flight transaction (maip not NULL) or let cntxt_age do it
         * for us.
         */
        if (releaseContext)
            rmpp_cntxt_release(rmpp_cntxt);
    }

    IB_EXIT(__func__, VSTATUS_OK);
    return (VSTATUS_OK);
}

/**
 * @brief  Gets the maximum data length of first packet.
 *
 * Check class version and the rmpp_format and return the correct
 * maximum data length the first packet contained in maip can hold.
 * Only the first EA packet has an EA header and the length does not
 * include the header
 *
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 *
 * @return   Maximum data length the packet can hold
 *
 *
 */
static inline int32_t
get_first_pkt_datalen(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
    int32_t rmppDataLen;

    if (maip->base.cversion  == STL_SA_CLASS_VERSION) {
        if (rmpp_cntxt->info->rmpp_format == RMPP_FORMAT_VENDOR) {

		{
                rmppDataLen = STL_RMPP_VENDOR_DATA_LEN;
            }
        } else {
            rmppDataLen = STL_SA_DATA_LEN;
        }
    } else {
        rmppDataLen = IB_SA_DATA_LEN;
    }

    return rmppDataLen;
}

/**
 * @brief  Gets the maximum data length of the packet.
 *
 * Check class version and the rmpp_format and return the correct
 * maximum data length the packet contained in maip can hold.
 * This is the length only of the data, and does not include any headers
 *
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 *
 * @return   Maximum data length the packet can hold
 *
 *
 */
static inline int32_t
get_pkt_datalen(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
    int32_t rmppDataLen;

    if (maip->base.cversion  == STL_SA_CLASS_VERSION) {
        if (rmpp_cntxt->info->rmpp_format == RMPP_FORMAT_VENDOR) {
                rmppDataLen = STL_RMPP_VENDOR_DATA_LEN;
        } else {
            rmppDataLen = STL_SA_DATA_LEN;
        }
    } else {
        rmppDataLen = IB_SA_DATA_LEN;
    }

    return rmppDataLen;
}

/**
 * @brief  Gets the header size depending on the packet type
 *
 *
 * Check class version and the rmpp format and return the size of the header.
 * The header size does not include the COMMON MAD and the RMPP header.
 *
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 *
 * @return  size of the header used for the packet type.
 *
 */
static inline int32_t
get_pkt_type_hdr_size(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
    int32_t hdr_size = 0;

        hdr_size = sizeof(SA_HDR);

    return hdr_size;
}

/**
 * @brief   Gets the pointer to the data present in the first packet.
 *
 * Returns a pointer to the actual data present in the packet. This is the
 * data present after the COMMON MAD, the RMPP and the packet header.
 * Only the first EA packet has an EA header.
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 *
 * @return  ptr to the data in the packet.
 */
static inline void *
get_ptr_to_first_pkt_mad_data(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
    void *src_data;

    if (rmpp_cntxt->info->rmpp_format == RMPP_FORMAT_VENDOR) {
		{
            src_data = (void *)(((STL_RMPP_VENDOR_PACKET *)maip->data)->data);
        }
    } else {
        src_data = (void *)(((STL_SA_MAD *)maip->data)->data);
    }

    return src_data;
}

/**
 * @brief   Gets the pointer to the data present in the packet.
 *
 * Returns a pointer to the actual data present in the packet. This is the
 * data present after the COMMON MAD, the RMPP and the packet header if
 * there is one. Only the first EA packet in an RMPP sequence has an EA
 * header.
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 *
 * @return  ptr to the data in the packet.
 */
static inline void *
get_ptr_to_mad_data(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
    void *src_data;

    if (rmpp_cntxt->info->rmpp_format == RMPP_FORMAT_VENDOR) {
        src_data = (void *)(((STL_RMPP_VENDOR_PACKET *)maip->data)->data);
    } else {
        src_data = (void *)(((STL_SA_MAD *)maip->data)->data);
    }

    return src_data;
}



/**
 * @brief    Sends a stop or a abort response to the peer.
 *
 * This function sends a stop or an abort depending on the status field
 * passed to it. This packet is sent to the connected peer,
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 * @param[in]  type       STOP or ABORT code.
 * @param[in]  status     RMPP status codes (See Infiniband Arch. Spec )
 *
 * @return  VSTATUS_OK if send successful else error.
 */
static Status_t
rmpp_send_stop_abort(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt,
                     uint8_t type, uint8_t status)
{
    STL_RMPP_DATA_HEADER *rmpp_hdr;
    Status_t             rc = VSTATUS_OK;
    STL_LID              lid;
    IBhandle_t           fd;

    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

    rmpp_hdr = (STL_RMPP_DATA_HEADER *)maip->data;

    /*
     * set the mai handle to use for sending - use primary handle if no send handle
     */
    fd = (rmpp_cntxt->sendFd) ? rmpp_cntxt->sendFd : *rmpp_cntxt->info->fd;

    if (if3DebugRmpp) {
        IB_LOG_INFINI_INFO_FMT(__func__,
                               "Sending ABORT to LID[0x%x] for TID["FMT_U64"]",
                               (int)maip->addrInfo.slid, maip->base.tid);
    }
    /*
     * Setup the out-bound MAD.  We need to reverse the LRH addresses.
     */
    lid = maip->addrInfo.slid;
    maip->addrInfo.slid = maip->addrInfo.dlid;
    maip->addrInfo.dlid = lid;
    maip->base.method |= MAD_CM_REPLY;  // send as response
    rmpp_hdr->rmppType = type;
    rmpp_hdr->rmppStatus = status;
    rmpp_hdr->u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
    rmpp_hdr->u.timeFlag = 0;
    rmpp_hdr->segNum = 0;
    rmpp_hdr->length = 0;

    if ((rc = mai_send(fd, maip)) != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__,
                         "error from mai_send while sending STOP OR ABORT to"
                         "LID[0x%x] for TID["FMT_U64"]",
                         (int)maip->addrInfo.dlid, maip->base.tid);
    }
    /* release the context, done with this RMPP xfer */
    rmpp_cntxt_release(rmpp_cntxt);

    IB_EXIT(__func__, rc);
    return (rc);
}


/**
 * @brief   Checks if packet is a valid RMPP packet
 *
 * Checks to see if the packet is an RMPP packet, whether the RMPP version
 * field has the right value and if the RMPP flags in the RMPP header are set
 * correctly. Should return a bool but not sure if Vxworks C compiler supports
 * C99 bool definition.
 *
 * @param[in]  maip        ptr to packet and its important fields.
 * @param[out] abortStatus RMPP status code (See InfiniBand Arc. Spec)
 *
 * @return  1 if not a valid pkt else 0
 */
static uint32_t
is_not_valid_rmpp_pkt(Mai_t *maip, uint8_t *abortStatus)
{
    STL_RMPP_DATA_HEADER *rmpp_hdr;
    uint32_t             invalid = 0;

    IB_ENTER(__func__, maip, 0, 0, 0);

    *abortStatus = 0;
    /*
     * Must be an RMPP request at this point, do the basic RMPP
     * validation of header fields.
     */
    rmpp_hdr = (STL_RMPP_DATA_HEADER *)maip->data;

    if (rmpp_hdr->rmppType == RMPP_TYPE_NOT) {
        IB_LOG_WARN_FMT(__func__, "ABORTING - RMPP protocol error;"
                        "type is NULL from Lid[0x%x] for TID="FMT_U64,
                        (int)maip->addrInfo.slid, maip->base.tid);
        *abortStatus = RMPP_STATUS_ABORT_BADTYPE;
    } else if (rmpp_hdr->rmppVersion != RMPP_VERSION) {
        IB_LOG_WARN_FMT(__func__, "RMPP protocol error, received RMPP"
                        "Version %d from LID[0x%x] for"
                        "getMulti TID["FMT_U64"]",
                        (int)rmpp_hdr->rmppVersion,
                        (int)maip->addrInfo.slid, maip->base.tid);
        *abortStatus = RMPP_STATUS_ABORT_UNSUPPORTED_VERSION;
    } else if (!(rmpp_hdr->u.tf.rmppFlags & RMPP_FLAGS_ACTIVE)) {
        IB_LOG_WARN_FMT(__func__,
                        "RMPP protocol error, RMPPFlags.Active bit is"
                        "NULL from LID[0x%x] for TID["FMT_U64"]"
                        "timeFlag[0x%x] RmppFlags[0x%x, %s]",
                        (int)maip->addrInfo.slid, maip->base.tid,
                        rmpp_hdr->u.timeFlag,
                        rmpp_hdr->u.tf.rmppFlags,
                        (rmpp_hdr->u.tf.rmppFlags & RMPP_FLAGS_ACTIVE) ?
                        "ACTIVE" : "NULL");
        *abortStatus = RMPP_STATUS_ABORT_BADTYPE;
    }
    if (*abortStatus) {
        invalid = 1;
    }

    IB_EXIT(__func__, invalid);
    return invalid;
}

/**
 * @brief   Checks if the rmpp ack packet received is invalid
 *
 * This function checks to see if the rmpp ack packet received from the sender
 * in response to segment 0 has the right segment number and whether
 * NewWindowLast field is non zero. Should ideally return "bool"
 *
 * @param[in]  maip        ptr to packet and its important fields.
 * @param[out] abortStatus RMPP status code (See InfiniBand Arc. Spec)
 *
 * @return  1 if not a valid pkt else 0
 */
static uint32_t
is_invalid_rmpp_ack_pkt(Mai_t *maip, uint8_t *abortStatus)
{
    STL_RMPP_ACK_HEADER *rmpp_hdr;
    uint32_t            invalid = 1;

    IB_ENTER(__func__, maip,0, 0, 0);

    rmpp_hdr = (STL_RMPP_ACK_HEADER *)maip->data;
    *abortStatus = 0;

    // process ACK of segment num 0 from sender
    if (rmpp_hdr->segNum == 0) {
        IB_LOG_WARN_FMT(__func__, "ABORTING - Invalid segment number %d in ACK"
                        "when expecting [0] from LID[0x%x] for TID["FMT_U64"]",
                        rmpp_hdr->segNum,
                        (int)maip->addrInfo.slid, maip->base.tid);
        *abortStatus = RMPP_STATUS_ABORT_SEGNUM_TOOBIG;
    } else if (rmpp_hdr->newwindowlast == 0) {
        // invalid window size
        IB_LOG_WARN_FMT(__func__, "ABORTING - A window size of zero was "
                        "specified in ACK from LID[0x%x] for TID["FMT_U64"]",
                        (int)maip->addrInfo.slid, maip->base.tid);
        *abortStatus = RMPP_STATUS_ABORT_NEWWINDOWLAST_TOOSMALL;
    } else {
        invalid = 0;
    }

    IB_EXIT(__func__, invalid);
    return invalid;
}

/**
 * @brief     Acknowledge the reception of a packet to peer.
 *
 * Send acknowledgement to peer for the receipt of a packet. If this is the last
 * segment of the transmission set the window to the last segment. Otherwise if
 * the window size is 1 set it to the next segment. If wsize is not 1 then set
 * the ack's newwindowlast to the minimum of the associated rmpp context's
 * segTotal field or the segment number + wsize or the wsize if it's the first
 * segment received. Update the response time value and set the rmpp_cntxt last
 * acked value to the current segment number.
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 * @param[in]  wsize      window size
 *
 * @return  VSTATUS_OK if send successful else error.
 *
 */
static Status_t
rmpp_ack_multi_packet_recv(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt, int32_t wsize)
{
    Status_t            rc = VSTATUS_OK;
    STL_LID             lid;
    IBhandle_t          fd;
    STL_RMPP_ACK_HEADER *rmpp_mad;
    uint32_t            segnum;
    uint32_t            window;
    rmpp_user_info_t    *info = NULL;

    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

    /*
     * set the mai handle for sending - use primary handle if no send handle
     */
    info = (rmpp_user_info_t *)rmpp_cntxt->info;
    fd = (rmpp_cntxt->sendFd) ? rmpp_cntxt->sendFd : *info->fd;
    rmpp_mad = (STL_RMPP_ACK_HEADER *) maip->data;
    segnum = ntoh32(rmpp_mad->segNum);
    window = ntoh32(rmpp_mad->newwindowlast);

    rmpp_mad->rmppType = RMPP_TYPE_ACK;

    if (rmpp_mad->u.tf.rmppFlags & RMPP_FLAGS_LAST) {
        rmpp_mad->newwindowlast = rmpp_mad->segNum;
    } else if (wsize == 1) {
        rmpp_mad->newwindowlast = hton32(segnum + 1);
    } else {
        rmpp_mad->newwindowlast = hton32(MIN(((segnum == 1) ?
                                    wsize :
                                    (segnum + wsize)),
                                   rmpp_cntxt->segTotal));
    }

    rmpp_mad->u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
    if (if3DebugRmpp) {
        IB_LOG_INFINI_INFO_FMT(__func__,
               "ACK window[%d] reached for LID[0x%x] TID="FMT_U64", New ACK"
               "window[%d] timeFlag[0x%x] RmppFlags[0x%x, %s]",
               window, (int)maip->addrInfo.slid, maip->base.tid,
               ntoh32(rmpp_mad->newwindowlast), rmpp_mad->u.timeFlag,
               rmpp_mad->u.tf.rmppFlags,
               (rmpp_mad->u.tf.rmppFlags & RMPP_FLAGS_ACTIVE) ?
               "ACTIVE" : "NULL");
    }
    /* set the desired response time value for client to use*/
    if (segnum == 1)
        rmpp_mad->u.tf.rmppRespTime = rmpp_respTimeValue;
    else
        rmpp_mad->u.tf.rmppRespTime = RMPP_NO_RRESP_TIME_VALUE;

    rmpp_mad->rmppStatus = 0;
    lid = maip->addrInfo.slid;
    maip->addrInfo.slid = maip->addrInfo.dlid;
    maip->addrInfo.dlid = lid;
    maip->base.method ^= MAD_CM_REPLY;
    if (if3DebugRmpp) {
       IB_LOG_INFINI_INFO_FMT(__func__,
                              "sending ACK to %s request to LID[0x%x]"
                              "for TID["FMT_U64"], mclass=0x%x\n",
                              info->rmpp_get_method_text(maip->base.method),
                              (int)maip->addrInfo.dlid, maip->base.tid,
                              maip->base.mclass);
    }


    if ((rc = mai_send(fd, maip)) == VSTATUS_OK) {
        rmpp_cntxt->last_ack = segnum;
    }
    IB_EXIT(__func__,rc);
    return rc;

}


/**
 * @brief  Process non rmpp multi request
 *
 * This function processes a non rmpp request.It allocates memory sets up
 * fields in the rmpp_cntxt to point to it copies the data to the allocated
 * memory.It assumes that the rmpp_cntxt has a pointer to a processing
 * function for such packets and calls that function.
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 *
 * @return  VSTATUS_OK if send successful else error.
 *
 */
static Status_t
rmpp_process_non_rmpp_req(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
    Status_t rc = VSTATUS_OK;
    void     *src_data;

    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

    rmpp_cntxt->method = maip->base.method;
    rmpp_cntxt->segTotal = 1;
    rmpp_cntxt->reqInProg = 0;
    memcpy(&rmpp_cntxt->mad, maip, sizeof(Mai_t));
    rmpp_cntxt->reqDataLen = (uint32_t)get_pkt_datalen(maip, rmpp_cntxt);

    src_data = get_ptr_to_mad_data(maip, rmpp_cntxt);

    rc = vs_pool_alloc(rmpp_cntxt->info->rmpp_pool, rmpp_cntxt->reqDataLen,
                       (void *) &rmpp_cntxt->reqData);

    if (!rc) {
        memcpy(rmpp_cntxt->reqData, src_data, rmpp_cntxt->reqDataLen);
        if (rmpp_cntxt->processFunc) {
            rc = rmpp_cntxt->processFunc((Mai_t *) &rmpp_cntxt->mad,
                                         rmpp_cntxt);
        }
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__,
                                   "Non RMPP GetMulti from Lid[0x%x],"
                                   "TID="FMT_U64" processed",
                                   (int)maip->addrInfo.slid,
                                   rmpp_cntxt->tid);
        }
    } else {
        IB_LOG_ERRORX("Couldn't allocate resources for Non RMPP"
                      "GetMulti request from LID:", rmpp_cntxt->lid);
    }

    IB_EXIT(__func__, rc);

    return rc;
}

/**
 * @brief  Intialize rmpp_cntxt fields before receiving packet.
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 */
static void
rmpp_init_recv_data_rmpp_cntxt(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

    rmpp_cntxt->method = maip->base.method;
    rmpp_cntxt->WF = 1;
    rmpp_cntxt->WL = 1;
    rmpp_cntxt->NS = 0;
    rmpp_cntxt->ES = 1;
    rmpp_cntxt->last_ack = 0;
    rmpp_cntxt->retries = 0;
    rmpp_cntxt->segTotal = 0;
    rmpp_cntxt->reqInProg = 1;
    rmpp_cntxt->sendFd = *rmpp_cntxt->info->fd;
    rmpp_cntxt->RespTimeout = 4ull * (2 * (1 << rmpp_packetLifetime) +
                                      (1 << rmpp_respTimeValue));
    rmpp_cntxt->tTime = 0;
    memcpy(&rmpp_cntxt->mad, maip, sizeof(*maip));
    rmpp_cntxt_reserve(rmpp_cntxt);

    IB_EXIT(__func__, 0);

}

/**
 * @brief   Acknowledge  received packet
 *
 * Checks the received packet for to see whether it is the last packet in the
 * multi packet chain or whether it is a single packet. Updates rmpp_cntxt
 * fields and then calls rmpp_ack_multi_packet_recv to send the actual ack. If
 * the send is successful good else free the rmpp_cntxt and report error.
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 * @param[in]  wsize      window size
 *
 *
 * @return  VSTATUS_OK if send successful else error.
 */
static Status_t
rmpp_multi_response_send_ack(Mai_t *maip, rmpp_cntxt_t* rmpp_cntxt,
                             int32_t wsize)
{
    Status_t             rc = VSTATUS_OK;
    uint32_t             segnum;
    uint32_t             datalen;
    STL_RMPP_DATA_HEADER *rmpp_mad;

    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

    rmpp_mad = (STL_RMPP_DATA_HEADER *)maip->data;
    segnum = ntoh32(rmpp_mad->segNum);
    datalen = ntoh32(rmpp_mad->length);

    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

    if (if3DebugRmpp) {
        IB_LOG_INFINI_INFO_FMT(__func__, "(segNum MOD wsize) = %d, (segNum EQ 1)"
                               "= %d, (segNum EQ rmpp_cntxt->segTotal) = %d",
                               (int)(segnum % wsize),
                               (segnum == 1),
                               (segnum == rmpp_cntxt->segTotal));
    }

    if (segnum == rmpp_cntxt->segTotal) {
        rmpp_cntxt->reqInProg = 0;
    }

    if (segnum == 1 &&
        (rmpp_mad->u.tf.rmppFlags & RMPP_FLAGS_FIRST) &&
        (rmpp_mad->u.tf.rmppFlags & RMPP_FLAGS_LAST)) {
        if (if3DebugRmpp)
            IB_LOG_INFINI_INFO_FMT(__func__, "Received single packet request");
    } else {
        if (if3DebugRmpp)
            IB_LOG_INFINI_INFO_FMT(__func__,
                                   "Received multi-packet request");
    }

    if (if3DebugRmpp)
        IB_LOG_INFINI_INFO_FMT(__func__,
                               "Sending ACK with segNum %d, length %d,"
                               "reqInProg %d, ES %d",
                               (int)segnum,
                               datalen,
                               rmpp_cntxt->reqInProg,
                               rmpp_cntxt->ES);

    // sent ACK to sender
    if ((rc = rmpp_ack_multi_packet_recv(maip, rmpp_cntxt,
                                         wsize)) != VSTATUS_OK) {
        IB_LOG_WARN_FMT(__func__,
                        "error %d while sending ACK to"
                        "LID[0x%x] for TID["FMT_U64"], terminating transaction",
                        rc, (int)maip->addrInfo.dlid, maip->base.tid);
        /* release the context, done with this RMPP xfer */
        rmpp_cntxt_release(rmpp_cntxt);
    }


    IB_EXIT(__func__, rc);
    return rc;
}

/**
 * @brief  Processes an out of sequence packet.
 *
 * When we receive and out of sequence segment then we can either not send
 * an ACK  and discard the packet and sender can retransmit from WF to WL
 * or we send ack of (ES-1) as per figure 178 of receiver main flow diagram
 * Here we opt to send an ack of (ES-1).
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 * @param[in]  wsize      window size
 *
 *
 * @return  VSTATUS_OK if send successful else error.
 */
static Status_t
rmpp_process_oos_segment(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt,
                        int32_t wsize)
{
    Status_t             rc = VSTATUS_OK;
    STL_RMPP_ACK_HEADER *rmpp_mad;

    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

    rmpp_mad = (STL_RMPP_ACK_HEADER *)maip->data;
    if (if3DebugRmpp) {
        IB_LOG_INFINI_INFO_FMT(__func__,
                               "got seg[%d] when expecting seg[%d] from"
                               "lID[0x%x] TID="FMT_U64,
                               ntoh32(rmpp_mad->segNum), rmpp_cntxt->ES,
                               (int)maip->addrInfo.slid, maip->base.tid);
    }
    /* resend ack of (ES-1) here */
    rmpp_mad->u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
    rmpp_mad->newwindowlast = hton32(rmpp_cntxt->ES - 1);
    if ((rc = rmpp_ack_multi_packet_recv(maip, rmpp_cntxt,
                                         wsize)) != VSTATUS_OK) {
        IB_LOG_WARN_FMT(__func__,
                        "error %d while sending ACK to LID[0x%x]"
                        "for TID["FMT_U64"]", rc, (int)maip->addrInfo.dlid,
                        maip->base.tid);
        /* release the context, done with this RMPP xfer */
        rmpp_cntxt_release(rmpp_cntxt);
    }
    IB_EXIT(__func__, rc);
    return rc;
}

/**
 * @brief   Process the first data segment of the response.
 *
 * Check if the packet has its header setup correctly to indicate that it is
 * the first packet in the segment.
 *
 * Next extract the Payload Length that will be there in the response and
 * calculate the total number of segments and use this to allocate the size of
 * the buffer needed to store the entire response.
 *
 * If the Payload Length is zero then check if it is also the last packet and if
 * so flag an error and send an abort and return. We don't handle packets with
 * the payload length equal to zero, though the IB spec states that we can.
 * If the Payload Length is zero and it is the last packet then treat it as a
 * single packet.
 *
 * Allocate buffer for packet from pool and copy the data from the packet into
 * the buffer.
 * If it is not a single packet response the increment the expected segment
 * field in the rmpp_cntxt.
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 * @param[in]  wsize      window size
 *
 *
 * @return  VSTATUS_OK if send successful else error.
 */
static Status_t
rmpp_process_first_data_segment(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt,
                                __attribute__((unused)) int32_t wsize)
{
    Status_t             rc = VSTATUS_OK;
    STL_RMPP_DATA_HEADER *rmpp_mad;
    uint32_t             hdr_size;
    int32_t              max_pkt_datalen;
    void                 *src_data;
    uint32_t             max_payload_len;
    rmpp_user_info_t     *info = NULL;

    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

    rmpp_mad = (STL_RMPP_DATA_HEADER *) maip->data;

    if (!(rmpp_mad->u.tf.rmppFlags & RMPP_FLAGS_FIRST)) {
        IB_LOG_WARN_FMT(__func__,
                        "ABORTING - invalid multi pkt first flag[%d] with"
                        "segnum 1 from LID[0x%x] for TID["FMT_U64"]",
                        rmpp_mad->u.tf.rmppFlags, (int)maip->addrInfo.slid,
                        maip->base.tid);

        rc = RMPP_STATUS_ABORT_INCONSISTENT_FIRST_SEGNUM;
        rmpp_send_stop_abort(maip, rmpp_cntxt, RMPP_TYPE_ABORT,
                             RMPP_STATUS_ABORT_INCONSISTENT_FIRST_SEGNUM);
        goto func_exit;
    }

    rmpp_cntxt->bytesRcvd = 0;
    hdr_size = get_pkt_type_hdr_size(maip, rmpp_cntxt);
    max_pkt_datalen = get_first_pkt_datalen(maip, rmpp_cntxt);
    max_payload_len = ntoh32(rmpp_mad->length);
    src_data = get_ptr_to_first_pkt_mad_data(maip, rmpp_cntxt);
    info = (rmpp_user_info_t *)rmpp_cntxt->info;

    if (max_payload_len) {
        uint32_t totsize = max_pkt_datalen + hdr_size;

        rmpp_cntxt->segTotal = (max_payload_len / totsize) +
                               ((max_payload_len % totsize) ? 1:0);

        if (info->rmpp_format == RMPP_FORMAT_VENDOR) {
            rmpp_cntxt->reqDataLen = max_payload_len - hdr_size;
        } else {
            rmpp_cntxt->reqDataLen = max_payload_len -
                (rmpp_cntxt->segTotal * hdr_size);
        }

        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__,
                                   "GetMulti from LID[0x%x], TID="FMT_U64", %d total bytes,"
                                   "%d segments, %d data bytes",
                                   maip->addrInfo.slid, maip->base.tid,
                                   max_payload_len, rmpp_cntxt->segTotal,
                                   rmpp_cntxt->reqDataLen);
        }

    } else {
        if (rmpp_mad->u.tf.rmppFlags & RMPP_FLAGS_LAST) {
            /* send ABORT with status of inconsistent payloadLength */
            IB_LOG_WARN_FMT(__func__,
                            "ABORTING - First and Last segment received with"
                            "no length from LID[0x%x] for TID["FMT_U64"]",
                            (int)maip->addrInfo.slid, maip->base.tid);
            rc = rmpp_send_stop_abort(maip, rmpp_cntxt, RMPP_TYPE_ABORT,
                                      RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH);
            if (rc == VSTATUS_OK) {
                rc = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH;
            }

            goto func_exit;

        } else {
            /* no payload length specified, terminate at single packet*/
            rmpp_cntxt->reqDataLen = max_pkt_datalen;
            rmpp_cntxt->segTotal = 0;
            if (if3DebugRmpp) {
                IB_LOG_INFINI_INFO0("PayloadLength is not specified, using"
                                    "single packet data length as payload");
            }
        }
    }

    if (rmpp_cntxt->reqDataLen)
        rc = vs_pool_alloc(info->rmpp_pool, rmpp_cntxt->reqDataLen,
                           (void *)&rmpp_cntxt->reqData);

    if (rc != VSTATUS_OK) {
        rmpp_cntxt->reqData = NULL;
        rc = rmpp_send_stop_abort(maip, rmpp_cntxt, RMPP_TYPE_STOP,
                                  RMPP_STATUS_STOP_NORESOURCES);
        if (rc == VSTATUS_OK) {
            rc = RMPP_STATUS_STOP_NORESOURCES;
        }
    } else {
        if (rmpp_mad->u.tf.rmppFlags & RMPP_FLAGS_LAST) {
            memcpy(rmpp_cntxt->reqData , src_data, rmpp_cntxt->reqDataLen);
            rmpp_cntxt->bytesRcvd = rmpp_cntxt->reqDataLen;
        } else {
            memcpy(rmpp_cntxt->reqData, src_data, max_pkt_datalen);
            ++rmpp_cntxt->ES;          /* increment expected seg */
            rmpp_cntxt->bytesRcvd = max_pkt_datalen;
        }
    }


func_exit:
    IB_EXIT(__func__, rc);
    return rc;
}

/**
 * @brief Process the middle or last segment of a response.
 *
 * Process the received packet and check that the Payload Length field which
 * should only be non zero in the case of the last packet in the sequence has
 * the right value. If this is incorrect send an abort and return.
 * If all else is good, copy the data into the buffer that was allocated
 * when the first segment was received at the right offset. Update the expected
 * sequence number if it is not the last packet.
 *
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 * @param[in]  wsize      window size
 *
 *
 * @return  VSTATUS_OK if send successful else error.
 */
static Status_t
rmpp_process_mid_last_segment(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt,
                              __attribute__((unused)) int32_t wsize)
{
    Status_t             rc = VSTATUS_OK;
    STL_RMPP_DATA_HEADER *rmpp_mad;
    uint32_t             segnum;
    uint32_t             last_pkt_dlen;
    void                 *src_data;
    int                  badPayloadLen = 0;
    uint32_t             bytesRcvd = 0;
    int                  max_pkt_datalen;
    int                  hdr_size;

    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

    rmpp_mad = (STL_RMPP_DATA_HEADER *) maip->data;
    segnum = ntoh32(rmpp_mad->segNum);
    src_data = get_ptr_to_mad_data(maip, rmpp_cntxt);
    max_pkt_datalen = get_pkt_datalen(maip, rmpp_cntxt);
    hdr_size = get_pkt_type_hdr_size(maip, rmpp_cntxt);


    if ((rmpp_mad->u.tf.rmppFlags & RMPP_FLAGS_LAST)) {
        if (rmpp_cntxt->info->rmpp_format == RMPP_FORMAT_VENDOR) {
            last_pkt_dlen = ntoh32(rmpp_mad->length);
        } else {
            last_pkt_dlen = ntoh32(rmpp_mad->length) - hdr_size;
        }
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__,
                                   "GetMulti from LID[0x%x], TID="FMT_U64" has"
                                   "%d Total data bytes in last packet "
                                   "segment number %d",
                                   maip->addrInfo.slid, maip->base.tid,
                                   last_pkt_dlen, segnum);
        }

        bytesRcvd = rmpp_cntxt->bytesRcvd + last_pkt_dlen;
        if (bytesRcvd > rmpp_cntxt->reqDataLen) {
            badPayloadLen = 1;
        } else {
            memcpy(rmpp_cntxt->reqData + rmpp_cntxt->bytesRcvd,
                   src_data, last_pkt_dlen);
            rmpp_cntxt->bytesRcvd += last_pkt_dlen;
        }

    } else {

        bytesRcvd = rmpp_cntxt->bytesRcvd + max_pkt_datalen;
        if (bytesRcvd > rmpp_cntxt->reqDataLen) {
            badPayloadLen = 1;
        } else {
            memcpy(rmpp_cntxt->reqData + rmpp_cntxt->bytesRcvd,
                   src_data, max_pkt_datalen);
            rmpp_cntxt->bytesRcvd += max_pkt_datalen;
            ++rmpp_cntxt->ES;          /* increment expected seg */
        }

    }

    if (badPayloadLen) {
        IB_LOG_WARN_FMT(__func__,
                        "ABORTING - data received [%d] inconsistent"
                        "with payloadLength [%d] from LID[0x%x] for"
                        "TID["FMT_U64"]", bytesRcvd, rmpp_cntxt->reqDataLen,
                        (int)maip->addrInfo.slid, maip->base.tid);
        rc = rmpp_send_stop_abort(maip, rmpp_cntxt, RMPP_TYPE_ABORT,
                                  RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH);
        if (rc == VSTATUS_OK)
            rc = RMPP_STATUS_ABORT_INCONSISTENT_LAST_PAYLOADLENGTH;
    }

    IB_EXIT(__func__, rc);
    return rc;
}

/**
 * @brief  Process the received data packet.
 *
 * Check if this is the start of the request/response and if so set up the
 * associated rmpp_cntxt fields for context entry.
 *
 * Check that the expected segment value in the context tallies with the segment
 * number in the recieved packet RMPP header and process it by calling the
 * appropriate processing function for the segments. Acknowledge the receipt of
 * the packet to the peer.
 *
 * If the received segment is out of sequence call the out of sequence
 * processing function.
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 *
 * @return  VSTATUS_OK if send successful else error.
 */
static Status_t
rmpp_process_data_pkt(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
    STL_RMPP_DATA_HEADER *rmpp_mad;
    uint32_t             segnum;
    Status_t             rc = VSTATUS_OK;
    int32_t              wsize = 1;

    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);
    rmpp_mad = (STL_RMPP_DATA_HEADER *) maip->data;

    segnum = ntoh32(rmpp_mad->segNum);

    if (rmpp_cntxt->hashed == 0) {
        /*
         * This is the start of request, init/save parameters for context entry
         */
        rmpp_init_recv_data_rmpp_cntxt(maip, rmpp_cntxt);
        rmpp_mad->rmppVersion = RMPP_VERSION;
        rmpp_mad->u.tf.rmppRespTime = rmpp_respTimeValue; // use classPortInfo setting
    }

    if (segnum == rmpp_cntxt->ES) {   /* is this segment expected */
        if (segnum == 1) {
            rc = rmpp_process_first_data_segment(maip, rmpp_cntxt, wsize);
        } else {
            rc = rmpp_process_mid_last_segment(maip, rmpp_cntxt, wsize);
        }

        /* ACK the first, last segments and window hits */
        if (((segnum % wsize) == 0 || segnum == 1 ||
             segnum == rmpp_cntxt->segTotal) && (rc == VSTATUS_OK)) {
            rmpp_multi_response_send_ack(maip, rmpp_cntxt, wsize);
        }

    } else {

        rc = rmpp_process_oos_segment(maip, rmpp_cntxt, wsize);

    }

    IB_EXIT(__func__, rc);
    return rc;
}

/**
 * @brief   Processes the received data packet
 *
 * First validate the received packet to ensure that the method is supported.
 * If not drop it and return.
 *
 * Check if it is by chance a non rmpp Get Multi request and if so process it by
 * calling the right function and return.
 *
 * Next check if it is a valid rmpp packet and if not respond with an abort and
 * return.
 *
 * If the received packet is a STOP or ABORT packet then release the rmpp
 * context associated and stop the processing and return.
 *
 * If the packet is a valid RMPP ACK packet update fields in the rmpp_cntxt
 * else send ABORT and return.
 *
 * If it is a data packet then process it.
 *
 * @param[in]  maip       ptr to packet and its important fields.
 * @param[in]  rmpp_cntxt ptr to the rmpp context to which the packet belongs
 *
 * @return  VSTATUS_OK if send successful else error.
 */
static Status_t
rmpp_process_multi_response(Mai_t *maip, rmpp_cntxt_t *rmpp_cntxt)
{
    STL_RMPP_DATA_HEADER *rmpp_mad;
    rmpp_user_info_t     *info = NULL;
    Status_t             rc = VSTATUS_OK;
    uint32_t             rmppDataLen;
    uint8_t             rmppStatus;

    IB_ENTER(__func__, maip, rmpp_cntxt, 0, 0);

    if (!rmpp_cntxt) {
        IB_LOG_ERROR_FMT(__func__, "context is NULL!");
        rc =  VSTATUS_BAD;
        goto func_exit;
    } else if (!(info = (rmpp_user_info_t *)rmpp_cntxt->info)) {
        IB_LOG_ERROR_FMT(__func__, "failed to get info for user %d!",
                         rmpp_cntxt->usrId);
        rc =  VSTATUS_BAD;
        goto func_exit;
    }


    // validate the MAD received.  If it is not valid, just drop it.
    if (rmpp_validate_response_mad(info, maip) != VSTATUS_OK) {
        goto func_exit;
    }

    rmpp_mad = (STL_RMPP_DATA_HEADER *) maip->data;
    rmppDataLen =  (uint32_t) get_pkt_datalen(maip, rmpp_cntxt);

    if (if3DebugRmpp)
        IB_LOG_INFINI_INFO_FMT(__func__, "Data length inuse %d, bversion=0x%x,"
                               "cversion=0x%x", rmppDataLen,
                               maip->base.bversion,
                               maip->base.cversion);


    /*
     * check if possible non-rmpp GetMulti request  or invalid pkt
     */
    if (rmpp_mad->rmppType == RMPP_TYPE_NOT && rmpp_cntxt->hashed == 0) {
       if ((rc = rmpp_process_non_rmpp_req(maip, rmpp_cntxt)) != VSTATUS_OK) {
            goto func_exit;
        }
    } else {
        if (is_not_valid_rmpp_pkt(maip, &rmppStatus)) {
            rc = rmpp_send_stop_abort(maip, rmpp_cntxt, RMPP_TYPE_ABORT,
                                      rmppStatus);
            goto func_exit;
        }
    }


    /*
     * Exit if packet is stop or abort type
     */
    if (rmpp_mad->rmppType == RMPP_TYPE_STOP ||
        rmpp_mad->rmppType == RMPP_TYPE_ABORT) {
        /* got a STOP or ABORT */
        if (if3DebugRmpp) {
            IB_LOG_WARN_FMT(__func__,
                            "Processing STOP OR ABORT with status code[0x%x]"
                            "from LID[0x%x] for TID["FMT_U64"]",
                            (int)rmpp_mad->rmppStatus,
                            (int)maip->addrInfo.slid, maip->base.tid);
        }
        rmpp_cntxt_release(rmpp_cntxt);
        rc =  VSTATUS_OK;
        goto func_exit;
    }


    if (rmpp_mad->rmppType == RMPP_TYPE_ACK) {
        if (is_invalid_rmpp_ack_pkt(maip, &rmppStatus)) {
            rc = rmpp_send_stop_abort(maip, rmpp_cntxt, RMPP_TYPE_ABORT,
                                      rmppStatus);
        } else {
            /*
             * set DS bit, set Window Last to desired, and clear request in progress bit
             */
            rmpp_cntxt->isDS = 1;
            rmpp_cntxt->WL = rmpp_mad->length;
            rmpp_cntxt->reqInProg = 0;
        }

    } else {
        // processing DATA packet
        if (if3DebugRmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__,
                                   "Processing RMPP GETMULTI request from Lid[0x%x], TID="FMT_U64,
                                   (int)maip->addrInfo.slid, maip->base.tid);
        }
        rc = rmpp_process_data_pkt(maip, rmpp_cntxt);
    }

func_exit:
        IB_EXIT(__func__, rc);
        return (rc);
}


#endif
