/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */
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
#include "pa_l.h"
#include "vs_g.h"
#include "vfi_g.h"
#include "pm_l.h"
#include "pm_counters.h"
#include "iba/ib_pa.h"
#include "iba/stl_pa.h"
#include "paServer.h"
#include "fm_xml.h"

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
#define LOCAL_MOD_ID VIEO_PA_MOD_ID

extern Pool_t pm_pool;
extern uint8_t vfi_mclass;
extern IBhandle_t pm_fd;
extern IBhandle_t hpma;
extern PMXmlConfig_t pm_config;
extern int master;  /* Master PM */
extern uint32_t pm_numEndPort;
extern Thread_t pm_expr_thread;

extern FSTATUS pm_expr_start_thread(void);
extern SMXmlConfig_t sm_config;
extern int smValidateGsiMadPKey(Mai_t *maip, uint8_t mgmntAllowedRequired, uint8_t antiSpoof);


#if defined(__VXWORKS__)
#define PA_STACK_SIZE (24 * 1024)
#else
#define PA_STACK_SIZE (256 * 1024)
#endif

#define PA_CNTXT_HASH_TABLE_DEPTH	64

#define	INCR_PA_CNTXT_NFREE() {  \
    if (pa_cntxt_nfree < pa_max_cntxt) {        \
        pa_cntxt_nfree++;        \
		SET_PM_PEAK_COUNTER(pmMaxPaContextsFree, pa_cntxt_nfree); \
    } else {                      \
		IB_LOG_ERROR("pa_cntxt_nfree already at max:", pa_cntxt_nfree);\
    }\
}

#define	DECR_PA_CNTXT_NFREE() {  \
    if (pa_cntxt_nfree) {        \
        pa_cntxt_nfree--;        \
    } else {                      \
		IB_LOG_ERROR0("can't decrement pa_cntxt_nfree, already zero");\
    }\
}

#define	INCR_PA_CNTXT_NALLOC() {  \
    if (pa_cntxt_nalloc < pa_max_cntxt) {        \
        pa_cntxt_nalloc++;        \
		SET_PM_PEAK_COUNTER(pmMaxPaContextsInUse, pa_cntxt_nalloc); \
    } else {                      \
		IB_LOG_ERROR("pa_cntxt_nalloc already at max:", pa_cntxt_nalloc);\
    }\
}

#define	DECR_PA_CNTXT_NALLOC() {  \
    if (pa_cntxt_nalloc) {        \
        pa_cntxt_nalloc--;        \
    } else {                      \
		IB_LOG_ERROR0("can't decrement pa_cntxt_nalloc, already zero");\
    }\
}

#define pa_cntxt_delete_entry( list, entry )	\
	if( (list) == (entry) ) (list) = (list)->next ; \
	else { \
		(entry)->prev->next = (entry)->next ; \
		if( (entry)->next ) (entry)->next->prev = (entry)->prev ; \
	} \
	(entry)->next = (entry)->prev = NULL 

#define pa_cntxt_insert_head( list, entry ) 	\
	if( (list) ) (list)->prev = (entry) ;	\
	(entry)->next = (list) ;		\
	(list) = (entry) ;  \
	(entry)->prev = NULL ;

#define	PA_Filter_Init(FILTERP) {					\
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
	(FILTERP)->dev = pm_config.hca;					\
	(FILTERP)->port = (pm_config.port == 0) ? MAI_TYPE_ANY : pm_config.port;	\
	(FILTERP)->qp = 1;						\
}

typedef struct _FieldMask {
	uint16_t	offset;			// offset from 'bit 0'
	uint16_t	length;			// number of bits in field
} FieldMask_t;


uint32_t pa_data_length;
uint32_t pa_max_cntxt = 2 * MIN_SUPPORTED_ENDPORTS;

static uint8_t pa_packetLifetime = 18;
uint8_t pa_respTimeValue = 18;
static uint64_t pa_reqTimeToLive = 0;
static uint32_t paMaxRetries = 3;   // max number of retries for failed receives

static int pa_main_writer_exit = 0;
static Thread_t pa_writer_thread;
static uint64_t	timeLastAged=0;

static int pa_protocol_inited = 0;
static uint32_t paRmppCheckSum = 0;  // control whether to checksum each rmpp response at start and end of transfer
static IBhandle_t fhPaRec = -1;
static IBhandle_t fhPaTable = -1;
static int pa_cntxt_nalloc = 0 ;
static int pa_cntxt_nfree = 0 ;
uint8_t *pa_data;
static IBhandle_t fd_pa_w;
static pa_cntxt_t *pa_cntxt_pool;
static pa_cntxt_t *pa_cntxt_free_list;
static Lock_t pa_cntxt_lock;
static pa_cntxt_t *pa_hash[PA_CNTXT_HASH_TABLE_DEPTH];

// TBD - template_* not used
static uint16_t patemplate_type;
static uint32_t patemplate_length;
static FieldMask_t	*patemplate_fieldp;

static FieldMask_t PaRecordFieldMask[] = { 
	{     0,    64 },
	{     0,     0 },
};

static FieldMask_t PaTableRecordFieldMask[] = { 
	{     0,    64 },
    {    64,    512},
	{     0,     0 },
};

static void pa_main_writer(uint32_t argc, uint8_t ** argv);



static char *
pa_getAidName(uint16_t aid)
{
    switch (aid) {
    case STL_PA_ATTRID_GET_CLASSPORTINFO:
        return "STL_GET_CLASSPORTINFO";
        break;
    case STL_PA_ATTRID_GET_GRP_LIST:
        return "STL_GET_GRP_LIST";
        break;
    case STL_PA_ATTRID_GET_GRP_INFO:
        return "STL_GET_GRP_INFO";
        break;
    case STL_PA_ATTRID_GET_GRP_CFG:
        return "STL_GET_GRP_CONFIG";
        break;
    case STL_PA_ATTRID_GET_PORT_CTRS:
        return "STL_GET_PORT_CNTRS";
        break;
    case STL_PA_ATTRID_CLR_PORT_CTRS:
        return "STL_GET_CLR_PORT_CNTRS";
        break;
    case STL_PA_ATTRID_CLR_ALL_PORT_CTRS:
        return "STL_GET_CLR_ALL_PORT_CNTRS";
        break;
    case STL_PA_ATTRID_GET_PM_CONFIG:
        return "STL_GET_PM_CONFIG";
        break;
	case STL_PA_ATTRID_FREEZE_IMAGE:
        return "STL_FREEZE_IMAGE";
        break;
	case STL_PA_ATTRID_RELEASE_IMAGE:
        return "STL_RELEASE_IMAGE";
        break;
	case STL_PA_ATTRID_RENEW_IMAGE:
        return "STL_RENEW_IMAGE";
        break;
	case STL_PA_ATTRID_GET_FOCUS_PORTS:
        return "STL_GET_FOCUS_PORTS";
        break;
	case STL_PA_ATTRID_GET_IMAGE_INFO:
        return "STL_GET_IMAGE_INFO";
        break;
	case STL_PA_ATTRID_MOVE_FREEZE_FRAME:
        return "STL_MOVE_FREEZE_FRAME";
        break;
	case STL_PA_ATTRID_GET_VF_LIST:
        return "STL_GET_VF_LIST";
        break;
	case STL_PA_ATTRID_GET_VF_INFO:
        return "STL_GET_VF_INFO";
        break;
	case STL_PA_ATTRID_GET_VF_CONFIG:
        return "STL_GET_VF_CONFIG";
        break;
	case STL_PA_ATTRID_GET_VF_PORT_CTRS:
        return "STL_GET_VF_PORT_CTRS";
        break;
	case STL_PA_ATTRID_CLR_VF_PORT_CTRS:
        return "STL_CLR_VF_PORT_CTRS";
        break;
	case STL_PA_ATTRID_GET_VF_FOCUS_PORTS:
        return "STL_GET_VF_FOCUS_PORTS";
        break;
    default:
        return "UNKNOWN AID";
        break;
    }
}

static char *
pa_getMethodText(int method)
{
    switch (method) {
    case STL_PA_CMD_GET:			return "STL_PA_CMD_GET";
    case STL_PA_CMD_SET:			return "STL_PA_CMD_SET";
    case STL_PA_CMD_GET_RESP:		return "STL_PA_CMD_GET_RESP";
    case STL_PA_CMD_GETTABLE:		return "STL_PA_CMD_GETTABLE";
    case STL_PA_CMD_GETTABLE_RESP:	return "STL_PA_CMD_GETTABLE_RESP";
    default: return "UNKNOWN METHOD";
    }
}

// TBD - template_* not used
static Status_t
pa_data_offset(uint16_t type)
{

	IB_ENTER(__func__, type, 0, 0, 0);

	patemplate_type = type;
	patemplate_fieldp = NULL;

    //	create the mask for the comparisons.
	switch (type) {
	case STL_PA_CMD_GET:
	case STL_PA_CMD_SET:
		patemplate_length = PA_RECORD_NSIZE;
		patemplate_fieldp = PaRecordFieldMask;
		break;
	case STL_PA_CMD_GETTABLE:
		patemplate_length = PA_TABLE_RECORD_NSIZE;
		patemplate_fieldp = PaTableRecordFieldMask;
		break;
	}

	if (patemplate_fieldp != NULL) {
		IB_EXIT(__func__, VSTATUS_OK);
		return(VSTATUS_OK);
	} else {
		IB_EXIT(__func__, VSTATUS_BAD);
		return(VSTATUS_BAD);
	}
}

static Status_t
pa_cntxt_free_data(pa_cntxt_t *cntxt)
{
	IB_ENTER(__func__, cntxt, 0, 0, 0 );
	
	if (cntxt->data)
		vs_pool_free(&pm_pool, cntxt->data);
	
	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

static Status_t
pa_validate_mad(Mai_t *maip)
{
    Status_t	rc = VSTATUS_OK;

	IB_ENTER(__func__, maip, 0, 0, 0);

    if (maip->base.cversion != STL_PA_CLASS_VERSION) {
        IB_LOG_WARN_FMT(__func__,
               "Invalid PA Class Version %d received in %s[%s] request from LID [0x%x], TID ["FMT_U64"], ignoring request!",
               maip->base.cversion, pa_getMethodText((int)maip->base.method), pa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
		rc = VSTATUS_BAD;
    } else {
        //  drop unsupported MADs
    	switch (maip->base.method) {
    	case PA_CMD_GET_RESP:
    	case PA_CMD_GETTABLE_RESP:
            if (pm_config.debug_rmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__,
                       "Unsupported or invalid %s[%s] request from LID [0x%x], TID["FMT_U64"]", 
                       pa_getMethodText((int)maip->base.method), pa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
            }
    		IB_EXIT(__func__, VSTATUS_OK);
    		rc = VSTATUS_BAD;
            break;
        default:
            break;
    	}
    }

    IB_EXIT(__func__, rc);
	return(rc);
}

/*
 * reserving context inserts it into the hash table if !hashed
 * and increments reference count
*/
static Status_t
pa_cntxt_reserve(pa_cntxt_t* pa_cntxt)
{
	int         bucket;
	Status_t	status;

	IB_ENTER(__func__, pa_cntxt, 0, 0, 0 );

    if ((status = vs_lock(&pa_cntxt_lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock PA context rc:", status);
    } else {
        pa_cntxt->ref ++;
        if( pa_cntxt->hashed == 0 ) {
            // This context needs to be inserted into the hash table
            bucket = pa_cntxt->lid % PA_CNTXT_HASH_TABLE_DEPTH;
            pa_cntxt->hashed = 1 ;
            vs_time_get( &pa_cntxt->tstamp );
            pa_cntxt_insert_head( pa_hash[ bucket ], pa_cntxt );
        }
        if ((status = vs_unlock(&pa_cntxt_lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("Failed to unlock PA context rc:", status);
        }
    }

	IB_EXIT(__func__, VSTATUS_OK );
	return VSTATUS_OK ;
}

static void
pa_cntxt_retire(pa_cntxt_t* lcl_cntxt)
{
	IB_ENTER(__func__, lcl_cntxt, 0, 0, 0 );

	// release getMulti request buffer if allocated
	if( lcl_cntxt->reqData ) {
		vs_pool_free( &pm_pool, lcl_cntxt->reqData );
	}
	// If data was privately allocated, free it
	if (lcl_cntxt->freeDataFunc) {
		(void)lcl_cntxt->freeDataFunc(lcl_cntxt);
	}

	// Reset all fields
	lcl_cntxt->ref = 0 ;
    lcl_cntxt->isDS = 0;
    lcl_cntxt->reqDataLen = 0;
	lcl_cntxt->reqData = NULL ;
	lcl_cntxt->data = NULL ;
	lcl_cntxt->len = 0 ;
	lcl_cntxt->lid = 0 ;
	lcl_cntxt->tid = 0 ;
	lcl_cntxt->hashed = 0 ;
	//lcl_cntxt->cache = NULL;
	lcl_cntxt->freeDataFunc = NULL;

	pa_cntxt_insert_head( pa_cntxt_free_list, lcl_cntxt );
	INCR_PA_CNTXT_NFREE();
    DECR_PA_CNTXT_NALLOC();

	IB_EXIT(__func__, 0 );
}

static Status_t
pa_cntxt_release(pa_cntxt_t* pa_cntxt)
{
	int         bucket;
	Status_t	status;

	IB_ENTER(__func__, pa_cntxt, 0, 0, 0 );

    if (!pa_cntxt) {
        IB_LOG_ERROR0("PA context is NULL!!!");
        return VSTATUS_OK ;
    }
    if ((status = vs_lock(&pa_cntxt_lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock PA context rc:", status);
    } else {
        if( pa_cntxt->ref == 0 ) {
            IB_LOG_INFINI_INFO0("reference count is already zero");
        } else {
            --pa_cntxt->ref;
            if( pa_cntxt->ref == 0 ) {

                // This context needs to be removed from hash
                if( pa_cntxt->hashed ) {
                    bucket = pa_cntxt->lid % PA_CNTXT_HASH_TABLE_DEPTH;
                    pa_cntxt_delete_entry( pa_hash[ bucket ], pa_cntxt );
                }

                pa_cntxt->prev = pa_cntxt->next = NULL ;
                pa_cntxt_retire( pa_cntxt );

            }
        }
        if ((status = vs_unlock(&pa_cntxt_lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("Failed to unlock PA context rc:", status);
        }
    }

	IB_EXIT(__func__, VSTATUS_OK );
	return VSTATUS_OK ;
}

Status_t
pa_cntxt_data(pa_cntxt_t* pa_cntxt, void* buf, uint32_t len)
{
	Status_t	status ;

	IB_ENTER(__func__, pa_cntxt, buf, len, 0 );

	status = VSTATUS_OK ;

	if (!buf || !len) {
		pa_cntxt->data = NULL;
        pa_cntxt->len = 0;
		pa_cntxt->freeDataFunc = NULL;
		goto done;
	}

	status = vs_pool_alloc(&pm_pool, len, (void*)&pa_cntxt->data);
	if (status == VSTATUS_OK) {
		pa_cntxt->len = len;
		memcpy(pa_cntxt->data, buf, len);
		pa_cntxt->freeDataFunc = pa_cntxt_free_data;
	} else {
        pa_cntxt->len = 0;
		pa_cntxt->data = NULL; /* If data is NULL, pa_send_reply will send error response to caller */
		pa_cntxt->freeDataFunc = NULL;
	}

done:
	IB_EXIT(__func__, status);
	return status ;
}

static Status_t
pa_send_single(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	Status_t	status;
	STL_SA_MAD	pamad;
    uint32_t    datalen = sizeof(STL_SA_MAD_HEADER);
    /* set the mai handle to use for sending - use reader handle PM if no context */
    IBhandle_t  fd = (pa_cntxt->sendFd) ? pa_cntxt->sendFd : pm_fd;

	IB_ENTER(__func__, maip, pa_cntxt, 0, 0);

    // send the response MAD.
	if (maip) {
		BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)(maip->data), &pamad, STL_SA_DATA_LEN);
		pamad.header.rmppVersion = 0;
		pamad.header.rmppType = 0;
		pamad.header.u.timeFlag = 0;
		pamad.header.rmppStatus = 0;
		pamad.header.segNum = 0;
		pamad.header.length = 0;

		(void)memset(pamad.data, 0, sizeof(pamad.data));
		if( pa_cntxt->data == NULL ) {
			if( maip->base.status == MAD_STATUS_OK )
				maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
		} else if (pa_cntxt->len != 0 ) {
			if (pa_cntxt->len > sizeof(pamad.data)) {
				// caller should have checked this, just to be safe
				IB_LOG_ERROR_FMT( __func__,  "Packet size=%"PRISZT" Buffer size=%u Expected=%u", sizeof(pamad.data), pa_cntxt->len, (uint32)STL_SA_DATA_LEN);
				IB_EXIT(__func__, VSTATUS_OK);
				return(VSTATUS_OK);
			}
			(void)memcpy(pamad.data, pa_cntxt->data, pa_cntxt->len);
			datalen += pa_cntxt->len;
		}

		INCREMENT_PM_MAD_STATUS_COUNTERS(maip);
		BSWAPCOPY_STL_SA_MAD(&pamad, (STL_SA_MAD*)(maip->data), (pa_cntxt->len ? pa_cntxt->len : STL_SA_DATA_LEN));
		if (pm_config.debug_rmpp) {
			IB_LOG_INFINI_INFO_FMT( __func__, 
			"sending reply to %s request to LID[0x%x] for TID["FMT_U64"]\n",
				   pa_getMethodText(maip->base.method), (int)maip->addrInfo.dlid, maip->base.tid);
		}

		if ((status = mai_stl_send(fd, maip, &datalen)) != VSTATUS_OK) {
			IB_LOG_ERROR_FMT( __func__, 
				   "can't send reply to %s request to LID[0x%x] for TID["FMT_U64"]",
				   pa_getMethodText(maip->base.method), (int)maip->addrInfo.dlid, maip->base.tid);
			IB_EXIT(__func__, VSTATUS_OK);
			return(VSTATUS_OK);
		}

		IB_EXIT(__func__, VSTATUS_OK);
		return(VSTATUS_OK);
	}
	IB_EXIT(__func__, VSTATUS_BAD);
	return(VSTATUS_BAD);
}
/*
 * Multi-paket RMPP protocol transfer
 */
static Status_t
pa_send_multi(Mai_t *maip, pa_cntxt_t *pa_cntxt)
{
    int         i;
    int         wl=0;
    uint8_t     chkSum=0;
	uint32_t	dlen, datalen = sizeof(STL_SA_MAD_HEADER);
	Status_t	status;
	STL_SA_MAD		pamad;
	STL_SA_MAD		paresp;
    uint16_t    sendAbort=0;
    uint16_t    releaseContext=1;  /* release context here unless we are in resend mode */
    uint64_t    tnow, delta, ttemp;
    IBhandle_t  fd = (pa_cntxt->sendFd) ? pa_cntxt->sendFd : pm_fd;
	size_t      pa_data_size = STL_SA_DATA_LEN;

	IB_ENTER(__func__, maip, pa_cntxt, pa_cntxt->len, 0);

    /*
     * At this point, the maip->addrInfo.slid/dlid and maip->base.method have already 
     * been changed in pa_send_reply to reflect a response packet, so use pa_cntxt 
     * lid and method values
    */
    memset((void *)&pamad, 0, sizeof(pamad));
	paresp = pamad;
	
    /* maip is NULL if Ack timeout */
	if( maip ) {
		BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER *)(maip->data),
									(STL_SA_MAD_HEADER *)&paresp);

		pa_data_size = MIN(STL_SA_DATA_LEN, pa_cntxt->len);
	}

    /*
     * 	See if answer to the request coming in
     *  normal getTable requests are not hashed while getMulti's have isDS bit set
     */
    if( pa_cntxt->hashed == 0 || pa_cntxt->isDS ) {
        /* init/save parameters and reserve context */
		if( maip ) {
			INCREMENT_PM_MAD_STATUS_COUNTERS(maip);
			memcpy( &pa_cntxt->mad, maip, sizeof( Mai_t ));
		}
        pa_cntxt->WF = 1;
        /* use value set in getMulti request ACK */
        if (!pa_cntxt->isDS) pa_cntxt->WL = 1;
        pa_cntxt->NS = pa_cntxt->WF;    // Next packet segment to send
        pa_cntxt->ES = 0;               // Expected segment number (Receiver only)
		pa_cntxt->last_ack = 0;         // last packet acked by receiver
		pa_cntxt->retries = 0;          // current retry count
		if ( pa_cntxt->len == 0 ) {
            pa_cntxt->segTotal = 1;
        } else if (pa_cntxt->len <= pa_data_length) {
            pa_cntxt->segTotal = (pa_data_size ? ((pa_cntxt->len + pa_data_size - 1)/pa_data_size) : 1);
        } else {
            IB_LOG_WARN("NO RESOURCES--> pa_cntxt->len too large len:", pa_cntxt->len);
			INCREMENT_PM_COUNTER(pmCounterRmppStatusAbortBadType);
            pa_cntxt->len = 0;
            pamad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
            pa_cntxt->segTotal = 1;
            sendAbort = 1;
        }

        /* calculate packet and total transaction timeouts C13-13.1.1 */
        pa_cntxt->RespTimeout = 4ull * (1<<20); // ~4.3 seconds
        pa_cntxt->tTime = 0;            // receiver only
        ttemp = pa_cntxt->mad.intime;   // save the time from original mad in context
        if (pa_cntxt->isDS) {
            pa_cntxt->mad.intime = ttemp;   // we want the time when getMulti request really started
            pa_cntxt->isDS = 0;
        }
        /* 8-bit cheksum of the rmpp response */
        pa_cntxt->chkSum = 0;
        if (paRmppCheckSum) {
            for (i=0; i<pa_cntxt->len; i++) {
                pa_cntxt->chkSum += pa_cntxt->data[i];
            }
        }
		pa_cntxt_reserve( pa_cntxt );
        pamad.header.rmppVersion = RMPP_VERSION;
        pamad.header.u.tf.rmppRespTime = 0x1F; // no time provided
        pamad.header.offset = pa_cntxt->attribLen; // setup attribute offset for RMPP xfer
        if (pm_config.debug_rmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__, 
                   "Lid[0x%x] STARTING RMPP %s[%s] with TID="FMT_U64", CHKSUM[%d]",
                   (int)pa_cntxt->lid, pa_getMethodText((int)pa_cntxt->method), 
                   (maip ? pa_getAidName(maip->base.aid) : ""), pa_cntxt->tid, pa_cntxt->chkSum);
        }
	} else if( maip ) {
        /* get original pamad with appropriate offset */
		BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*)pa_cntxt->mad.data, &pamad, STL_SA_DATA_LEN);
        /*
         * Check the response and set the context parameters according to it   
         */
       if (paresp.header.rmppType == RMPP_TYPE_NOT && (paresp.header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE)) {
            // invalid RMPP type
            IB_LOG_WARN_FMT(__func__, 
                   "ABORTING - RMPP protocol error; RmppType is NULL in %s[%s] from Lid[%d] for TID="FMT_U64,
                   pa_getMethodText((int)pa_cntxt->method), pa_getAidName(maip->base.aid), (int)pa_cntxt->lid, pa_cntxt->tid);
			INCREMENT_PM_COUNTER(pmCounterRmppStatusAbortBadType);
            sendAbort = 1;
            pamad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
       } else if (paresp.header.rmppVersion != RMPP_VERSION) {
            /* ABORT transaction with BadT status */
			INCREMENT_PM_COUNTER(pmCounterRmppStatusAbortUnsupportedVersion);
            sendAbort = 1;
            pamad.header.rmppStatus = RMPP_STATUS_ABORT_UNSUPPORTED_VERSION;
            IB_LOG_WARN_FMT(__func__, 
                   "ABORTING - Unsupported Version %d in %s[%s] request from LID[0x%x], TID["FMT_U64"]",
                   paresp.header.rmppVersion, pa_getMethodText((int)pa_cntxt->method), pa_getAidName(maip->base.aid), (int)pa_cntxt->lid, pa_cntxt->tid);
       } else if (!(paresp.header.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE)) {
           /* invalid RMPP type */
           IB_LOG_WARN_FMT(__func__, 
                  "RMPP protocol error, RMPPFlags.Active bit is NULL in %s[%s] from LID[0x%x] for TID["FMT_U64"]",
                  pa_getMethodText((int)pa_cntxt->method), pa_getAidName(maip->base.aid), (int)pa_cntxt->lid, pa_cntxt->tid);
			INCREMENT_PM_COUNTER(pmCounterRmppStatusAbortBadType);
           pamad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
           sendAbort = 1;
       } else if (paresp.header.rmppType == RMPP_TYPE_ACK) {
            /* Got an ACK packet from receiver */
            if (paresp.header.segNum < pa_cntxt->WF) {
                /* silently discard the packet */
                if (pm_config.debug_rmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__,
                        "LID[0x%x] sent ACK for seg %d which is less than Window First (%d) for TID: "FMT_U64, 
                        pa_cntxt->lid, (int)paresp.header.segNum, (int)pa_cntxt->WF,
                		pa_cntxt->tid);
                }
            } else if (paresp.header.segNum > pa_cntxt->WL) {
                /* ABORT the transaction with S2B status */
				INCREMENT_PM_COUNTER(pmCounterRmppStatusAbortSegnumTooBig);
                sendAbort = 1;
                pamad.header.rmppStatus = RMPP_STATUS_ABORT_SEGNUM_TOOBIG;
                IB_LOG_INFINI_INFO_FMT(__func__,
                        "ABORT - LID[0x%x] sent invalid seg %d in ACK, should be <= than %d, ABORTING TID:"FMT_U64,
                        pa_cntxt->lid, (int)paresp.header.segNum, (int)pa_cntxt->WL,
                		pa_cntxt->tid);
            } else if (paresp.header.length < pa_cntxt->WL /*|| paresp.length > pa_cntxt->segTotal*/) { 
                /* length is NewWindowLast (NWL) in ACK packet */
                /* ABORT transaction with W2S status */
                sendAbort = 1;
                if (paresp.header.length < pa_cntxt->WL) {
					INCREMENT_PM_COUNTER(pmCounterRmppStatusAbortNewWindowLastTooSmall);
                    pamad.header.rmppStatus = RMPP_STATUS_ABORT_NEWWINDOWLAST_TOOSMALL;
                } else {
					INCREMENT_PM_COUNTER(pmCounterRmppStatusAbortUnspecified);
                    pamad.header.rmppStatus = RMPP_STATUS_ABORT_UNSPECIFIED;
				}
                IB_LOG_INFINI_INFO_FMT(__func__,
                        "ABORT - LID[0x%x] sent invalid NWL %d in ACK, should be >=%d and <=%d, ABORTING TID:"FMT_U64,
                        pa_cntxt->lid, (int)paresp.header.length, (int)pa_cntxt->WL, pa_cntxt->segTotal, pa_cntxt->tid);
            } else if (paresp.header.segNum >= pa_cntxt->last_ack) { 
				pa_cntxt->last_ack = paresp.header.segNum;
                pa_cntxt->retries = 0;  /* reset the retry count  after receipt of ack */
                /* is it ack of very last packet? */
                if (paresp.header.segNum == pa_cntxt->segTotal) {
                    /* we are done */
                    if (pm_config.debug_rmpp) {
                        IB_LOG_INFINI_INFO_FMT(__func__,
                               " Received seg %d ACK, %s[%s] transaction from LID[0x%x], TID["FMT_U64"] has completed",
                               paresp.header.segNum, pa_getMethodText((int)pa_cntxt->method), pa_getAidName(pa_cntxt->mad.base.aid),
                               pa_cntxt->lid, pa_cntxt->tid);
                    }
                } else {
                    /* update WF, WL, and NS and resume sends */
                    pa_cntxt->WF = pa_cntxt->last_ack + 1;
                    pa_cntxt->WL = (paresp.header.length > pa_cntxt->segTotal) ? pa_cntxt->segTotal : paresp.header.length;
                    /* see if new Response time needs to be calculated */
                    if (paresp.header.u.tf.rmppRespTime && paresp.header.u.tf.rmppRespTime != 0x1f) {
                        pa_cntxt->RespTimeout = 4ull * ( (2*(1<<pa_packetLifetime)) + (1<<paresp.header.u.tf.rmppRespTime) );
                        if (pm_config.debug_rmpp) {
                            IB_LOG_INFINI_INFO_FMT(__func__,
                                   "LID[0x%x] set RespTimeValue (%d usec) in ACK of seg %d for %s[%s], TID["FMT_U64"]",
                                   pa_cntxt->lid, (int)pa_cntxt->RespTimeout, paresp.header.segNum, 
                                   pa_getMethodText((int)pa_cntxt->method), pa_getAidName((int)pa_cntxt->mad.base.aid),
                                   pa_cntxt->tid);
                        }
                    }
                }
            }
		} else if (paresp.header.rmppType == RMPP_TYPE_STOP || paresp.header.rmppType == RMPP_TYPE_ABORT) {
            /* got a STOP or ABORT */
            if (pm_config.debug_rmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__,
					"STOP/ABORT received for %s[%s] from LID[0x%x], status code = %x, for TID["FMT_U64"]",
					pa_getMethodText((int)pa_cntxt->method), pa_getAidName((int)maip->base.aid),
					pa_cntxt->lid, paresp.header.rmppStatus, pa_cntxt->tid);
            }
            pa_cntxt_release( pa_cntxt );
            IB_EXIT(__func__, VSTATUS_OK );
            return VSTATUS_OK ;
		} else {
            /* invalid RmppType received */
            IB_LOG_WARN_FMT(__func__,
                   "ABORT - Invalid rmppType %d received for %s[%s] from LID[0x%x] for TID["FMT_U64"]",
                   paresp.header.rmppType, pa_getMethodText((int)pa_cntxt->method), pa_getAidName((int)maip->base.aid),
                   pa_cntxt->lid, pa_cntxt->tid);
            // abort with badtype status
			INCREMENT_PM_COUNTER(pmCounterRmppStatusAbortBadType);
            sendAbort = 1;
            pamad.header.rmppStatus = RMPP_STATUS_ABORT_BADTYPE;
		}
	} else {
		/* We are timing out, retry till retry  count expires */
		/* get original pamad from context with correct offset */
		BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*) (pa_cntxt->mad.data), &pamad, STL_SA_DATA_LEN);
		++pa_cntxt->retries;
		if( pa_cntxt->retries > paMaxRetries ) {
			if (pm_config.debug_rmpp) {
				IB_LOG_INFINI_INFO_FMT(__func__
                        , "ABORT - MAX RETRIES EXHAUSTED; no ACK for seg %d of %s[%s] request from LID[0x%X], TID = "FMT_U64, 
				        (int)pa_cntxt->WL, pa_getMethodText((int)pa_cntxt->method), pa_getAidName(pa_cntxt->mad.base.aid), pa_cntxt->lid, pa_cntxt->tid);
			}
			/* ABORT transaction with too many retries status */
			INCREMENT_PM_COUNTER(pmCounterRmppStatusAbortTooManyRetries);
			sendAbort = 1;
			pamad.header.rmppStatus = RMPP_STATUS_ABORT_TOO_MANY_RETRIES;
			/* let context_age do the releasing;  It already holds the lock */
			releaseContext=0;
		} else {
			INCREMENT_PM_COUNTER(pmCounterPaRmppTxRetries);
			pa_cntxt->NS = pa_cntxt->WF;            // reset Next packet segment to send
			if (pm_config.debug_rmpp) {
				IB_LOG_INFINI_INFO_FMT(__func__,
				       "Timed out waiting for ACK of seg %d of %s[%s] from LID[0x%x], TID["FMT_U64"], retry #%d",
				       (int)pa_cntxt->WL, pa_getMethodText((int)pa_cntxt->method), pa_getAidName((int)pa_cntxt->mad.base.aid),
				       pa_cntxt->lid, pa_cntxt->tid, pa_cntxt->retries);
			}
		}
	}

    /* see if we're done (last segment was acked) */
	if( pa_cntxt->last_ack == pa_cntxt->segTotal && !sendAbort) {
        if (pm_config.debug_rmpp) {
            vs_time_get (&tnow);
            delta = tnow-pa_cntxt->mad.intime;
            IB_LOG_INFINI_INFO_FMT(__func__, 
                   "%s[%s] RMPP [CHKSUM=%d] TRANSACTION from LID[0x%x], TID["FMT_U64"] has completed in %d.%.3d seconds (%"CS64"d usecs)",
                   pa_getMethodText((int)pa_cntxt->method), pa_getAidName(pa_cntxt->mad.base.aid), 
                   pa_cntxt->chkSum, pa_cntxt->lid, pa_cntxt->tid,
                   (int)(delta/1000000), (int)((delta - delta/1000000*1000000))/1000, delta);
        }
        /* validate that the 8-bit cheksum of the rmpp response is still the same as when we started */
        if (paRmppCheckSum) {
            chkSum = 0;
            for (i=0; i<pa_cntxt->len; i++) {
                chkSum += pa_cntxt->data[i];
            }
            if (chkSum != pa_cntxt->chkSum) {
                IB_LOG_ERROR_FMT(__func__, 
                       "CHECKSUM FAILED [%d vs %d] for completeted %s[%s] RMPP TRANSACTION from LID[0x%x], TID["FMT_U64"]",
                       chkSum, pa_cntxt->chkSum, pa_getMethodText((int)pa_cntxt->method), pa_getAidName(pa_cntxt->mad.base.aid), 
                       pa_cntxt->lid, pa_cntxt->tid);
            }
        }
        if (releaseContext) pa_cntxt_release( pa_cntxt );
		IB_EXIT(__func__, VSTATUS_OK );
		return VSTATUS_OK ;
	}

    /* we must use the Mad_t that was saved in the context */
	maip = &pa_cntxt->mad ;
    /* 
     * send segments up till Window Last (WL) and wait for ACK 
     * Due to possible concurrency issue if reader is pre-empted while 
     * sending segement 1 and writer runs to process ACK of seg 1, use 
     * a local var for WL.
     * In the future we need to add individual context locking if we intend 
     * to go with a pool of PA thread.
     */
    wl = pa_cntxt->WL;
	while (pa_cntxt->NS <= wl && !sendAbort) {

        /*
         * calculate amount of data length to send in this segment and put in mad;
         * dlen=payloadLength in first packet, dlen=remainder in last packet
         */
		if( pa_cntxt->NS == pa_cntxt->segTotal ) {
			dlen = (pa_data_size)?(pa_cntxt->len % pa_data_size):0;
			dlen = (dlen)? dlen : pa_data_size ;
		} else 
			dlen = pa_data_size;

        if (dlen > pa_data_length) {
            IB_LOG_WARN("dlen is too large:", dlen);
        }
        // make sure there is data to send; could just be error case with no data
        if (pa_cntxt->len && pa_cntxt->data) {
            (void)memcpy(pamad.data, pa_cntxt->data + ((pa_cntxt->NS - 1) * pa_data_size), dlen );
        }

        pamad.header.rmppVersion = RMPP_VERSION;
        pamad.header.rmppType = RMPP_TYPE_DATA;
        pamad.header.rmppStatus = 0;
        pamad.header.segNum = pa_cntxt->NS;
        if (pa_cntxt->NS == 1 && pa_cntxt->NS == pa_cntxt->segTotal) {
            /* 
             * first and last segment to transfer, set length to payload length
             * add 20 bytes of SA header to each segment for total payload 
             */
            pamad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_FIRST | RMPP_FLAGS_LAST;
			pamad.header.length = pa_cntxt->len + (pa_cntxt->segTotal * SA_HEADER_SIZE);   
        } else if( pa_cntxt->NS == 1 ) {
            /* 
             * first segment to transfer, set length to payload length
             * add 20 bytes of SA header to each segment for total payload 
             */
			pamad.header.length = pa_cntxt->len + (pa_cntxt->segTotal * SA_HEADER_SIZE);   
            pamad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_FIRST;
		} else if( pa_cntxt->NS == pa_cntxt->segTotal ) {
            /* last segment to go; len=bytes remaining */
            pamad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE | RMPP_FLAGS_LAST;
			pamad.header.length = dlen + SA_HEADER_SIZE;  // add the extra 20 bytes of SA header
			if( pamad.header.length == 0 ) {
				pamad.header.length = pa_data_size;
			}
		}  else {
            /* middle segments */
            pamad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
			pamad.header.length = 0;
		}
        /* put pamad back into Mad_t in the context */
        datalen += pa_data_size;
		BSWAPCOPY_STL_SA_MAD(&pamad, (STL_SA_MAD*)(maip->data), pa_data_size);

		if (pm_config.debug_rmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__,
                    "sending fragment %d, len %d to LID[0x%x] for TID = "FMT_U64, 
                    (int)pamad.header.segNum, (int)pamad.header.length, (int)pa_cntxt->lid,
            		pa_cntxt->tid);
        }
        /* increment NS */
        ++pa_cntxt->NS;
		if ((status = mai_stl_send(fd, maip, &datalen)) != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__, 
                   "mai_send error [%d] while processing %s[%s] request from LID[0x%x], TID["FMT_U64"]",
                   status, pa_getMethodText((int)pa_cntxt->method), pa_getAidName(maip->base.aid), 
                   pa_cntxt->lid, pa_cntxt->tid);
            if (releaseContext) pa_cntxt_release( pa_cntxt );
			IB_EXIT(__func__, VSTATUS_OK );
			return VSTATUS_OK ;
		}
	}

	/*
     * Send Abort if desired
     */
    if (sendAbort) {
        pamad.header.rmppVersion = RMPP_VERSION;
        pamad.header.rmppType = RMPP_TYPE_ABORT;
        pamad.header.u.tf.rmppFlags = RMPP_FLAGS_ACTIVE;
        pamad.header.u.tf.rmppRespTime = 0;
        pamad.header.segNum = 0;
        pamad.header.length = 0;
		BSWAPCOPY_STL_SA_MAD(&pamad, (STL_SA_MAD*)(maip->data), STL_SA_DATA_LEN);

		if ((status = mai_send(fd, maip)) != VSTATUS_OK)
            IB_LOG_ERROR_FMT(__func__,
				"error[%d] from mai_send while sending ABORT of %s[%s] request to LID[0x%x], TID["FMT_U64"]",
				status, pa_getMethodText((int)pa_cntxt->method), pa_getAidName(maip->base.aid),
				pa_cntxt->lid, maip->base.tid);
        /*
         * We are done with this RMPP xfer.  Release the context here if 
         * in flight transaction (maip not NULL) or let pa_cntxt_age do it 
         * for us because retries is > SA_MAX_RETRIES. 
         */
        if (releaseContext) pa_cntxt_release( pa_cntxt );
    }

	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
pa_send_reply(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	uint8_t		method;
	uint16_t	lid;

	IB_ENTER(__func__, maip, pa_cntxt, 0, 0);

    // setup the out-bound MAD, we need to reverse the LRH addresses.
	if( maip )  {
		lid = maip->addrInfo.slid;
		maip->addrInfo.slid = maip->addrInfo.dlid;
		maip->addrInfo.dlid = lid;
		method = maip->base.method;
		pa_cntxt->method = method;
        switch (maip->base.method) {
        case STL_PA_CMD_GET:
        case STL_PA_CMD_SET:
			maip->base.method = STL_PA_CMD_GET_RESP;
            break;
        case STL_PA_CMD_GETTABLE:
			maip->base.method = STL_PA_CMD_GETTABLE_RESP;
            break;
        default:
            // everybody else, OR in 0x80 to inbound; i.e., 14->94
			maip->base.method |= MAD_CM_REPLY;
            break;
        }
	} else {
		// this is the timeout case
        method = pa_cntxt->method;
    } 

    // always send rmpp responses to multi-packet responses
    if ((method == STL_PA_CMD_GETTABLE) || (maip && method == STL_PA_CMD_GETTABLE_RESP)
		/*|| pa_cntxt->len > SAMAD_DATA_COUNT*/) {
        // as per Table 154, page 784 of spec, can only return this error in response to a GET/SET
        // for all other requests, we simply return a 1 segment rmpp response with header only 
        // additional spec references relating to handling of rmpp responses are (C15-0.1.19, C15-0.1.29)
        if (maip && (method == STL_PA_CMD_GETTABLE_RESP) &&
            (maip->base.status == MAD_STATUS_SA_NO_RECORDS)) {
            maip->base.status = MAD_STATUS_SA_NO_ERROR;
        }
        if (pa_cntxt->len > pa_data_length) {
            IB_LOG_WARN_FMT( __func__,
                   "pa_cntxt->len[%d] too large, returning no resources error to caller to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!",
                   pa_cntxt->len, pa_getMethodText((int)method), (maip ? pa_getAidName(maip->base.aid) : ""), pa_cntxt->lid, pa_cntxt->tid);
            if (maip) maip->base.status = MAD_STATUS_SA_NO_RESOURCES;
            pa_cntxt->len = 0;
        }
        (void)pa_send_multi(maip, pa_cntxt);
        IB_EXIT(__func__, VSTATUS_OK);
        return(VSTATUS_OK);
    }

	if (pa_cntxt->len > STL_SAMAD_DATA_COUNT) {
        IB_LOG_WARN_FMT( __func__,
               "pa_cntxt->len[%d] too large for GET, returning too many recs error to caller to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!",
               pa_cntxt->len, pa_getMethodText((int)method), (maip ? pa_getAidName(maip->base.aid) : ""), pa_cntxt->lid, pa_cntxt->tid);
        if (maip) maip->base.status = MAD_STATUS_SA_TOO_MANY_RECS;
        pa_cntxt->len = 0;
	}
	(void)pa_send_single(maip, pa_cntxt);
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

/*
 * safely release context while already under lock protection
 */
static void
cntxt_release(pa_cntxt_t* pa_cntxt)
{
	int         bucket;

    if( pa_cntxt->ref == 0 ) {
        IB_LOG_INFINI_INFO0("reference count is already zero");
        return;   /* context already retired */
    } else {
        --pa_cntxt->ref;
        if( pa_cntxt->ref == 0 ) {
            // This context needs to be removed from hash
            if( pa_cntxt->hashed ) {
                bucket = pa_cntxt->lid % PA_CNTXT_HASH_TABLE_DEPTH;
                pa_cntxt_delete_entry( pa_hash[ bucket ], pa_cntxt );
            }
            pa_cntxt->prev = pa_cntxt->next = NULL ;
            pa_cntxt_retire( pa_cntxt );
        }
    }
}

/*
 * Age context and do resends for those that timed out
 */
static void
pa_cntxt_age(void)
{
	pa_cntxt_t* pa_cntxt = NULL ;
	pa_cntxt_t*	tout_cntxt ;
    int i;
	Status_t	status;

    if ((status = vs_lock(&pa_cntxt_lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock PA context rc:", status);
    } else {
        vs_time_get( &timeLastAged );
        for (i=0; i<PA_CNTXT_HASH_TABLE_DEPTH; i++) {
            pa_cntxt = pa_hash[ i ];
            while( pa_cntxt ) {
                // Iterate before the pointers are destroyed ;
                tout_cntxt = pa_cntxt ;
                pa_cntxt = pa_cntxt->next ;

                // Though list is sorted, it does not hurt to 
                // do a simple comparison
                if( timeLastAged - tout_cntxt->tstamp > tout_cntxt->RespTimeout) { // used to be VTIMER_1S
                    // Timeout this entry
                    pa_cntxt_delete_entry( pa_hash[ i ], tout_cntxt );
                    pa_cntxt_insert_head( pa_hash[ i ], tout_cntxt );
                    // Touch the entry
                    tout_cntxt->tstamp = timeLastAged ;
                    // Call timeout
                    tout_cntxt->sendFd = fd_pa_w;       // use pa writer mai handle for restransmits
                    // resend the reply
                    pa_send_reply( NULL, tout_cntxt );
                    if( tout_cntxt->retries > paMaxRetries ) {
                        /* need to release the context.  Call local safe release */
                        cntxt_release(tout_cntxt);
                    }
                }
            }
        }
        if ((status = vs_unlock(&pa_cntxt_lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("Failed to unlock PA context rc:", status);
        }
    }
}

//
// find context entry matching input mad
//
static pa_cntxt_t *
pa_cntxt_find(Mai_t* mad)
{
	uint64_t	now ;
	int 		bucket;
	Status_t	status;
	pa_cntxt_t* pa_cntxt;
	pa_cntxt_t*	req_cntxt = NULL;

    if ((status = vs_lock(&pa_cntxt_lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock PA context rc:", status);
    } else {
        vs_time_get( &now );
        // Search the hash table for the context 	
        bucket = mad->addrInfo.slid % PA_CNTXT_HASH_TABLE_DEPTH ;
        pa_cntxt = pa_hash[ bucket ];
        while( pa_cntxt ) {
            if( pa_cntxt->lid == mad->addrInfo.slid  && pa_cntxt->tid == mad->base.tid ) {
                req_cntxt = pa_cntxt ;
                pa_cntxt = pa_cntxt->next ;
                req_cntxt->tstamp = now ;
                break ;
            } else {
                pa_cntxt = pa_cntxt->next;
            }
        }
        // Table is sorted with respect to timeout
        if( req_cntxt ) {
            // Touch current context
            req_cntxt->tstamp = now ;
            pa_cntxt_delete_entry( pa_hash[ bucket ], req_cntxt );
            pa_cntxt_insert_head( pa_hash[ bucket ], req_cntxt );
            // A get on an existing context reserves it
            req_cntxt->ref ++ ;
        }
        if ((status = vs_unlock(&pa_cntxt_lock)) != VSTATUS_OK) {
            IB_LOG_ERRORRC("Failed to unlock PA context rc:", status);
        }
    }
	IB_EXIT(__func__, req_cntxt );
    return req_cntxt;
}

//
// Process only inflight rmpp requests
// Called by the pa_main_writer thread
//
static Status_t
pa_process_inflight_rmpp_request(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{

	IB_ENTER(__func__, maip, pa_cntxt, 0, 0);
    //
    //	Process only inflight rmpp requests 
    //  ACKs of inprogress GETMULTI requests also come here
    //
    if (pa_cntxt->hashed  && !pa_cntxt->reqInProg) {
		pa_send_reply( maip, pa_cntxt );
	} else {
        IB_LOG_INFINI_INFO_FMT(__func__,
               "SA_WRITER received %s[%s] RMPP packet from LID [0x%x] TID ["FMT_U64"] after transaction completion", 
               pa_getMethodText((int)maip->base.method), pa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
    }
    IB_EXIT(__func__, VSTATUS_OK);
    return(VSTATUS_OK);
}

/*
 * Very simple hashing implemented. This function is modular and can be 
 * changed to use any algorithm, if hashing turns out to be bad.
 * Returns new context for new requests, existing context for in progress 
 * getMulti requests and NULL for duplicate requests
 */
static PAContextGet_t
pa_cntxt_get(Mai_t* mad, void **context)
{
	uint64_t	now ;
	int 		bucket;
	Status_t	status;
	pa_cntxt_t* pa_cntxt;
    PAContextGet_t getStatus=0;
	pa_cntxt_t*	req_cntxt = NULL;

	IB_ENTER(__func__, mad, 0, 0, 0 );

    if ((status = vs_lock(&pa_cntxt_lock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Failed to lock PA context rc:", status);
    } else {
        vs_time_get( &now );
        // Search the hash table for the context 	
        bucket = mad->addrInfo.slid % PA_CNTXT_HASH_TABLE_DEPTH ;
        pa_cntxt = pa_hash[ bucket ];
        while( pa_cntxt ) {
            if( pa_cntxt->lid == mad->addrInfo.slid  &&
                    pa_cntxt->tid == mad->base.tid ) {
                req_cntxt = pa_cntxt ;
                break ;
            } else {
                pa_cntxt = pa_cntxt->next;
            }
        }
		if( req_cntxt ) {
            /* dup of an existing request */
            getStatus = ContextExist;
            req_cntxt = NULL;
		} else {
			/* Allocate a new context and set appropriate status */
			req_cntxt = pa_cntxt_free_list ;
			if( req_cntxt ) {
				getStatus = ContextAllocated;
				pa_cntxt_delete_entry( pa_cntxt_free_list, req_cntxt );
				req_cntxt->ref = 1;  /* set ref count to 1 */
				INCR_PA_CNTXT_NALLOC();
				DECR_PA_CNTXT_NFREE();
				req_cntxt->lid = mad->addrInfo.slid ;
				req_cntxt->tid = mad->base.tid ;
				req_cntxt->method = mad->base.method ;
				req_cntxt->tstamp = now;
			} else {
				/* out of context */
				getStatus = ContextNotAvailable;
			}
		}
		if ((status = vs_unlock(&pa_cntxt_lock)) != VSTATUS_OK) {
			IB_LOG_ERRORRC("Failed to unlock PA context rc:", status);
		}
	}
    /* return new context or existing getmulti context or NULL */
    *context = req_cntxt;

	IB_EXIT(__func__, getStatus );
	return getStatus;
}

static int
pa_filter_validate_mad(Mai_t *maip)
{
    int rc = 0;

    // The following P_Key related security features shall be supported by all GSI administrators:
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
    if (!smValidateGsiMadPKey(maip, 1, sm_config.sma_spoofing_check)) {
        rc = 1;
        if (pm_config.debug_rmpp) {
            IB_LOG_WARN_FMT(__func__, 
                            "Dropping packet, invalid P_Key 0x%x %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                            maip->addrInfo.pkey, pa_getMethodText((int)maip->base.method),
                            pa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid); 
        }
    }

    if (rc && pm_config.debug_rmpp) {
        IB_LOG_WARN_FMT(__func__,
                        "Dropping packet, invalid limited member attribute 0x%x %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                         maip->base.aid, pa_getMethodText((int)maip->base.method), 
                         pa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid); 
    }

    if (rc) {
        // if the MAD is not valid, then indicate to the MAI filter handling
        // code that no match has been found.  Also, change the MAI type to
        // indicate to the MAI filter handling code to just drop the packet.  
        maip->type = MAI_TYPE_DROP;
    }
    
    return (rc) ? 1 : 0;
}

static int
pa_writer_filter(Mai_t *maip)
{
	STL_SA_MAD_HEADER pamad;

    if (pa_filter_validate_mad(maip))
        return 1;   // indicate that no match has been found

    // the pa writer thread is only responsible for in flight rmpp packets
	if (maip->base.method == PA_CMD_GETTABLE || maip->base.method == (PA_CMD_GETTABLE_RESP ^ MAD_PA_REPLY))
	{
        // check for inflight rmpp
		BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER*) (maip->data), &pamad);
        if ((pamad.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE) && 
            ((pamad.rmppType == RMPP_TYPE_ACK && pamad.segNum > 0) || pamad.rmppType == RMPP_TYPE_STOP 
             || pamad.rmppType == RMPP_TYPE_ABORT)) {
            if (pm_config.debug_rmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__,
                       "Processing inflight Rmpp packet for %s[%s] from LID[0x%x], TID="FMT_U64, 
                       pa_getMethodText((int)maip->base.method), pa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
            }
            return 0;  // process inflight rmpp responses
        }
	}
	return 1;
}

static void
pa_main_writer(uint32_t argc, uint8_t ** argv)
{
	Status_t	status;
	Mai_t		in_mad;
	Filter_t	filter;
	pa_cntxt_t	*pa_cntxt;
	uint64_t	now;

	IB_ENTER(__func__, 0, 0, 0, 0);

	pa_main_writer_exit = 0;
    
    //
    // create the MAD filter for the PA writer thread.
	PA_Filter_Init(&filter);
	filter.value.mclass = vfi_mclass;
	filter.mask.mclass = 0xff;
	filter.value.method = 0x00;
	filter.mask.method = 0x80;
	filter.mai_filter_check_packet = pa_writer_filter;
	MAI_SET_FILTER_NAME (&filter, "PA Writer");

	if (mai_filter_create(fd_pa_w, &filter, VFILTER_SHARE) != VSTATUS_OK) {
		IB_LOG_ERROR0("can't create PM(*) filter");
		(void)vs_thread_exit(&pa_writer_thread);
	}

	while (1) {
		status = mai_recv(fd_pa_w, &in_mad, VTIMER_1S/4);
        if (status != VSTATUS_OK && status != VSTATUS_TIMEOUT) {
            IB_LOG_ERRORRC("error on mai_recv rc:", status);
            vs_thread_sleep(VTIMER_1S/10);
        }

        if (pa_main_writer_exit == 1) {
#ifdef __VXWORKS__
            ESM_LOG_ESMINFO("PA Writer Task exiting OK.", 0);
#endif
            break;
        }

        /* don't process messages if not master PM or still doing first sweep */
		if (!master) {
            continue;
        }

		/* 
         * process the rmpp ack and send out the next set of segments
         */
        if (status == VSTATUS_OK) {
            /* locate and process in flight rmpp request */
            pa_cntxt = pa_cntxt_find(&in_mad);
            if (pa_cntxt) {
                pa_process_inflight_rmpp_request(&in_mad, pa_cntxt);
                /*
                 * This may not necessarily release context
                 * based on if someone else has reserved it
                 */
                pa_cntxt_release(pa_cntxt);
            } else {
				INCREMENT_PM_COUNTER(pmCounterPaDeadRmppPacket);
                if (pm_config.debug_rmpp) {
                    IB_LOG_INFINI_INFO_FMT(__func__, 
                           "dropping %s[%s] RMPP packet from LID[0x%x], TID ["FMT_U64"] already completed/aborted",
                           pa_getMethodText((int)in_mad.base.method), pa_getAidName(in_mad.base.aid), 
                           in_mad.addrInfo.slid, in_mad.base.tid);
                }
            }
        }

        /* age contexts if more than 1 second since last time */
        vs_time_get( &now );
        if ((now - timeLastAged) > (VTIMER_1S)) {
            (void) pa_cntxt_age();
        }
	}

    // delete filter
    mai_filter_delete(fd_pa_w, &filter, filter.flags);
    // close channel
	(void)mai_close(fd_pa_w);

	//IB_LOG_INFINI_INFO0("thread: Exiting OK");
}

static Status_t
pa_protocol_init(void)
{
    int i;
	Status_t status;
#ifdef __VXWORKS__
	uint8_t *thread_id = (uint8_t*)"esm_paw";
#else
	uint8_t *thread_id = (void *)"pawriter";
#endif


    if (pa_protocol_inited)
        return VSTATUS_OK;

    pa_protocol_inited = 1;
    // initialize PA context pool to inflight RMPP requests
	memset(pa_hash, 0, sizeof(pa_hash));
	pa_cntxt_pool = NULL;

    IB_LOG_VERBOSE("Allocating PA context pool with num entries=", pa_max_cntxt);
	status = vs_pool_alloc(&pm_pool, sizeof(pa_cntxt_t) * pa_max_cntxt, (void *)&pa_cntxt_pool);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR("PM: Can't allocate PA context pool");
		return VSTATUS_NOMEM;
	}
	memset(pa_cntxt_pool, 0, sizeof(pa_cntxt_t) * pa_max_cntxt);
	pa_cntxt_free_list = NULL ;
	for( i = 0 ; i < pa_max_cntxt ; ++i ) {
		pa_cntxt_insert_head( pa_cntxt_free_list, &pa_cntxt_pool[i] );
	}
    pa_cntxt_nfree = pa_max_cntxt;
    pa_cntxt_nalloc = 0;
    /* 
     * calculate request time to live on queue
     * ~ 3.2secs for defaults: sa_packetLifetime=18 and sa_respTimeValue=18 
     */
    pa_reqTimeToLive = 4ull * ( (2*(1<<pa_packetLifetime)) + (1<<pa_respTimeValue) ); 

    // initialize PA context pool lock
	status = vs_lock_init(&pa_cntxt_lock, VLOCK_FREE, VLOCK_THREAD);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't initialize PA context pool lock rc:", status);
        return VSTATUS_BAD;
	}

    //	allocate the PA data storage pool.
	status = vs_pool_alloc(&pm_pool, pa_data_length, (void*)&pa_data);
	if (status != VSTATUS_OK) {
		IB_FATAL_ERROR("PM: can't allocate pa data");
		return VSTATUS_NOMEM;
	}

	// open a channel that is used by the PA to handle RMPP responses and acks
	if ((status = mai_open(MAI_GSI_QP, pm_config.hca, pm_config.port, &fd_pa_w)) != VSTATUS_OK) {
		IB_FATAL_ERROR("can't open fd_sa_w");
        return VSTATUS_BAD;
	}
    //
    //	Start the PA writer thread.

    //	Sleep for half a second to let the previous thread startup.
	(void)vs_thread_sleep(VTIMER_1S/2);

    if ((status = vs_thread_create(&pa_writer_thread, (unsigned char *)thread_id,
		                          pa_main_writer, 0, NULL, PA_STACK_SIZE)) != VSTATUS_OK)
		IB_FATAL_ERROR("failed to start PA Writer thread");

    return status;
}

static Status_t
pa_create_filters(IBhandle_t fd)
{
    int rc=0;

	// create the filters that would allow us to recieve PA commands on the handle
    if (fd > 0) {
    	if ((rc = mai_filter_method(fd, VFILTER_SHARE, MAI_TYPE_ANY, &fhPaRec, vfi_mclass, STL_PA_CMD_GET)) != VSTATUS_OK) {
    		IB_LOG_ERRORRC("can't create filter for STL_PA_CMD_GET rc:", rc);
    	}
    
        if (rc == VSTATUS_OK) {
            if ((rc = mai_filter_method(fd, VFILTER_SHARE, MAI_TYPE_ANY, &fhPaTable, vfi_mclass, STL_PA_CMD_GETTABLE)) != VSTATUS_OK) {
                IB_LOG_ERRORRC("can't create filter for STL_PA_CMD_GETTABLE rc:", rc);
            }
        }
		if (rc == VSTATUS_OK) {
            if ((rc = mai_filter_method(fd, VFILTER_SHARE, MAI_TYPE_ANY, &fhPaRec, vfi_mclass, STL_PA_CMD_SET)) != VSTATUS_OK) {
                IB_LOG_ERRORRC("can't create filter for STL_PA_CMD_SET rc:", rc);
            }
        }
    }

    return rc;
}

void pa_delete_filters()
{
    if (pm_fd > 0) {
        if (fhPaRec)
            mai_filter_hdelete(pm_fd, fhPaRec);
        if (fhPaTable)
            mai_filter_hdelete(pm_fd, fhPaTable);
    }
}

Status_t
pa_mngr_open_cnx(void)
{
    Status_t status = FSUCCESS;

    // create filters to PA Server for PA clients to access
    if ((status = pa_create_filters(pm_fd)) == VSTATUS_OK) {
        // initialize PA Protocol related structures
        if ((status = pa_protocol_init()) == VSTATUS_OK) {
#if 0   // DEBUG_CODE --- Temporary patch until implementation completed
            //	start the PM Expr thread.  sleep for half a second
            // to let the previous thread startup.
            (void)vs_thread_sleep(VTIMER_1S/2);
            status = pm_expr_start_thread();
#endif
        }
    }

    return status;
}

void
pa_mngr_close_cnx(void)
{
    (void)pa_delete_filters();
}

void 
pa_main_kill(void)
{
    pa_protocol_inited = 0;
	pa_main_writer_exit = 1;
}

Status_t
pa_process_mad(Mai_t *maip, pa_cntxt_t* pa_cntxt)
{
	STL_SA_MAD pamad;
    uint64_t startTime=0, endTime=0;

	IB_ENTER(__func__, maip, pa_cntxt, 0, 0);

    // performance timestamp
    if (pm_config.debug_rmpp)
        (void) vs_time_get (&startTime);

    // validate the MAD received.  If it is not valid, just drop it.
	if (pa_validate_mad(maip) != VSTATUS_OK) {
		IB_EXIT(__func__, VSTATUS_OK);
		return(VSTATUS_OK);
	}

    /* use PM mai handle for sending out 1st packet of responses */
	pa_cntxt->sendFd = pm_fd;

    /* get network Mad into structure */
	BSWAPCOPY_STL_SA_MAD((STL_SA_MAD*) (maip->data), &pamad, STL_SA_DATA_LEN);

    // process specific command request.
	(void)pa_data_offset(maip->base.method);
    
	switch (maip->base.method) {
	case STL_PA_CMD_SET:
    	if (maip->base.aid == PA_ATTRID_CLR_PORT_CTRS) {
		    (void)pa_clrPortCountersResp(maip, pa_cntxt);
    	} else if (maip->base.aid == PA_ATTRID_CLR_ALL_PORT_CTRS) {
		    (void)pa_clrAllPortCountersResp(maip, pa_cntxt);
    	} else if (maip->base.aid == PA_ATTRID_FREEZE_IMAGE) {
		    (void)pa_freezeImageResp(maip, pa_cntxt);
    	} else if (maip->base.aid == PA_ATTRID_RELEASE_IMAGE) {
		    (void)pa_releaseImageResp(maip, pa_cntxt);
    	} else if (maip->base.aid == PA_ATTRID_RENEW_IMAGE) {
		    (void)pa_renewImageResp(maip, pa_cntxt);
    	} else if (maip->base.aid == PA_ATTRID_MOVE_FREEZE_FRAME) {
		    (void)pa_moveFreezeImageResp(maip, pa_cntxt);
    	} else if (maip->base.aid == STL_PA_ATTRID_CLR_VF_PORT_CTRS) {
		    (void)pa_clrVFPortCountersResp(maip, pa_cntxt);
		} else {
		    //(void)pa_getSingleMadResp(maip, pa_cntxt);
			goto invalid;
		}
		break;
	case STL_PA_CMD_GET:
    	if (maip->base.aid == STL_PA_ATTRID_GET_CLASSPORTINFO) {
		    (void)pa_getClassPortInfoResp(maip, pa_cntxt);
    	} else if (maip->base.aid == PA_ATTRID_GET_PORT_CTRS) {
		    (void)pa_getPortCountersResp(maip, pa_cntxt);
    	} else if (maip->base.aid == PA_ATTRID_GET_PM_CONFIG) {
		    (void)pa_getPmConfigResp(maip, pa_cntxt);
    	} else if (maip->base.aid == STL_PA_ATTRID_GET_VF_PORT_CTRS) {
		    (void)pa_getVFPortCountersResp(maip, pa_cntxt);
		} else {
		    //(void)pa_getSingleMadResp(maip, pa_cntxt);
			goto invalid;
		}
		break;
	case STL_PA_CMD_GETTABLE:
		if (maip->base.aid == STL_PA_ATTRID_GET_GRP_LIST) {
		    (void)pa_getGroupListResp(maip, pa_cntxt);
		} else if (maip->base.aid == STL_PA_ATTRID_GET_GRP_INFO) {
		    (void)pa_getGroupInfoResp(maip, pa_cntxt);
		} else if (maip->base.aid == STL_PA_ATTRID_GET_GRP_CFG) {
		    (void)pa_getGroupConfigResp(maip, pa_cntxt);
    	} else if (maip->base.aid == PA_ATTRID_GET_FOCUS_PORTS) {
		    (void)pa_getFocusPortsResp(maip, pa_cntxt);
    	} else if (maip->base.aid == PA_ATTRID_GET_IMAGE_INFO) {
		    (void)pa_getImageInfoResp(maip, pa_cntxt);
    	} else if (maip->base.aid == STL_PA_ATTRID_GET_VF_LIST) {
		    (void)pa_getVFListResp(maip, pa_cntxt);
    	} else if (maip->base.aid == STL_PA_ATTRID_GET_VF_INFO) {
		    (void)pa_getVFInfoResp(maip, pa_cntxt);
    	} else if (maip->base.aid == STL_PA_ATTRID_GET_VF_CONFIG) {
		    (void)pa_getVFConfigResp(maip, pa_cntxt);
    	} else if (maip->base.aid == STL_PA_ATTRID_GET_VF_FOCUS_PORTS) {
		    (void)pa_getVFFocusPortsResp(maip, pa_cntxt);
		} else {
		    //(void)pa_getMultiMadResp(maip, pa_cntxt);
			goto invalid;
		}
		break;
    default:
invalid:
        IB_LOG_INFINI_INFO_FMT(__func__,
               "Unsupported or invalid %s[%s] request from LID [0x%x], TID["FMT_U64"]", 
               pa_getMethodText((int)maip->base.method), pa_getAidName((int)maip->base.aid), maip->addrInfo.slid, maip->base.tid);
		maip->base.status = MAD_STATUS_SA_REQ_INVALID;
		pa_cntxt_data(pa_cntxt, pa_data, 0);
		(void)pa_send_reply(maip, pa_cntxt);
		goto release;
	}

    // switch to pa writer mai handle for sending out remainder of rmpp responses
    if (pa_cntxt->method == PA_CMD_GETTABLE) {
        pa_cntxt->sendFd = fd_pa_w;
    }

    if (pm_config.debug_rmpp) {
        /* use the time received from umadt as start time if available */
        startTime = (maip->intime) ? maip->intime : startTime;
        /* lids have been swapped, so use dlid here */
        (void) vs_time_get (&endTime);
        IB_LOG_INFINI_INFO_FMT(__func__, 
               "%ld microseconds to process %s[%s] request from LID 0x%.4X, TID="FMT_U64,
               (long)(endTime - startTime), pa_getMethodText((int)pa_cntxt->method), 
               pa_getAidName(maip->base.aid), maip->addrInfo.dlid, maip->base.tid);
    }

    // this may not necessarily release context based on if someone else has reserved it
release:
	pa_cntxt_release(pa_cntxt);

	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

pa_cntxt_t *
pa_mngr_get_cmd(Mai_t * mad, uint8_t * processMad)
{
	pa_cntxt_t	*pa_cntxt=NULL;
	uint64_t	now, delta, max_delta;
	int			tries=0, retry=0;
    PAContextGet_t  cntxGetStatus=0;
    int         numContextBusy=0;
	STL_SA_MAD_HEADER samad;

	IB_ENTER ("pa_mngr_get_cmd", mad, processMad, 0, 0);

	if (mad == NULL || processMad == NULL) {
		IB_EXIT ("MngrGetFECmd", pa_cntxt);
		return pa_cntxt;
	}

    switch (mad->base.method) {
    case STL_PA_CMD_GET:
    case STL_PA_CMD_SET:
    case STL_PA_CMD_GETTABLE:
        // drop inflight rmpp requests these are handled by the pawriter task
        if (mad->base.method == STL_PA_CMD_GETTABLE) {
            // check for inflight rmpp
			BSWAPCOPY_STL_SA_MAD_HEADER((STL_SA_MAD_HEADER*) (mad->data), &samad);
            if ((samad.u.tf.rmppFlags & RMPP_FLAGS_ACTIVE) && 
                ((samad.rmppType == RMPP_TYPE_ACK && samad.segNum != 0) 
                                      ||
                  samad.rmppType == RMPP_TYPE_STOP || samad.rmppType == RMPP_TYPE_ABORT)) {
                if (pm_config.debug_rmpp) {
                    IB_LOG_INFINI_INFO_FMT(__func__,
                           "Ignoring inflight request for %s[%s] from LID[0x%x], TID="FMT_U64, 
                           pa_getMethodText((int)mad->base.method), pa_getAidName((int)mad->base.aid), mad->addrInfo.slid, mad->base.tid);
                }
                return pa_cntxt;  // not processing inflight rmpp stuff either
            }
        }
        if (pm_config.debug_rmpp) {
            IB_LOG_INFINI_INFO_FMT(__func__,
                   "Processing request for %s[%s] from LID[0x%x], TID="FMT_U64, 
                   pa_getMethodText((int)mad->base.method), pa_getAidName((int)mad->base.aid), mad->addrInfo.slid, mad->base.tid);
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
            max_delta = 30*pa_reqTimeToLive;
            do {
                vs_time_get( &now );
                delta = now - mad->intime;
                tries++;

                if ((now < mad->intime) || (delta > max_delta)) {
                    vs_thread_sleep(1);
                    retry = 1;
                } else {
                    retry = 0;
                }	
            } while (retry && tries < 20);

            if (delta > pa_reqTimeToLive) {
				INCREMENT_PM_COUNTER(pmCounterPaDroppedRequests);
                if (pm_config.debug_rmpp) {
                    IB_LOG_INFINI_INFO_FMT(__func__,
                           "Dropping stale %s[%s] request from LID[0x%x], TID="FMT_U64"; On queue for %d.%d seconds.", 
                           pa_getMethodText((int)mad->base.method), pa_getAidName((int)mad->base.aid), mad->addrInfo.slid, 
                           mad->base.tid, (int)(delta/1000000), (int)((delta - delta/1000000*1000000))/1000);
                }
                /* drop the request without returning a response; sender will retry */
                return pa_cntxt;
            }
        }
        /* 
         * get a context to process request; pa_cntxt can be:
         *   1. NULL if resources are scarce
         *   2. NULL if request is dup of existing request
         *   3. in progress getMulti request context
         *   4. New context for a brand new request 
         */
        cntxGetStatus = pa_cntxt_get(mad, (void *)&pa_cntxt);
        if (cntxGetStatus == ContextAllocated) {
            /* process the new request */
            *processMad = TRUE;
            //pa_process_mad(&in_mad, pa_cntxt);
            // this may not necessarily release context based on if someone else has reserved it
            //if(pa_cntxt)
            //    pa_cntxt_release( pa_cntxt );
        } else if (cntxGetStatus == ContextExist) {
			INCREMENT_PM_COUNTER(pmCounterPaDuplicateRequests);
            /* this is a duplicate request */
            *processMad = FALSE;

            if (pm_config.debug_rmpp) {
                IB_LOG_INFINI_INFO_FMT(__func__,
                       "received duplicate %s[%s] from LID [0x%x] with TID ["FMT_U64"] ", 
                       pa_getMethodText((int)mad->base.method), pa_getAidName((int)mad->base.aid),mad->addrInfo.slid, mad->base.tid);
            }
        } else if (cntxGetStatus == ContextNotAvailable) {
			INCREMENT_PM_COUNTER(pmCounterPaContextNotAvailable);
            /* we are swamped, return BUSY to caller */
            *processMad = TRUE;
            if (pm_config.debug_rmpp) { /* log msg before send changes method and lids */
                IB_LOG_INFINI_INFO_FMT(__func__,
                       "NO CONTEXT AVAILABLE, returning MAD_STATUS_BUSY to %s[%s] request from LID [0x%x], TID ["FMT_U64"]!",
                       pa_getMethodText((int)mad->base.method), pa_getAidName((int)mad->base.aid), mad->addrInfo.slid, mad->base.tid);
            }
            mad->base.status = MAD_STATUS_BUSY;
            //pa_send_reply( &in_mad, pa_cntxt );
            if ((++numContextBusy % pa_max_cntxt) == 0) {
                IB_LOG_INFINI_INFO_FMT(__func__,
                       "Had to drop %d PA requests since start due to no available contexts",
                       numContextBusy);
            }
        } else {
            IB_LOG_WARN("Invalid pa_cntxt_get return code:", cntxGetStatus);
        }
        break;
    }

	IB_EXIT ("pa_mngr_get_cmd", pa_cntxt);
	return pa_cntxt;
}

#endif
