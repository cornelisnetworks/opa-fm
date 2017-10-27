/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//
// FILE NAME
//    sm_dbsync.c
//
// DESCRIPTION
//    This thread, when SM is the master SM, processes sync requests from the 
//    dbsync queue and synchronizes the indicated data with the remote standby SM
//    When SM is in the standby state, it processes sync messages from the master SM
//    and updates it's SA DB.
//
////
//
//===========================================================================//

#include "os_g.h"
#include "vs_g.h"
#include "ib_types.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "ib_macros.h"
#include "iba/ib_rmpp.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_l.h"
#include "sa_l.h"
#include "sm_counters.h"
#include "if3.h"
#include "cs_queue.h"
#include "sm_dbsync.h"
#include "time.h"

#ifdef IB_STACK_OPENIB
#include "mal_g.h"
#endif

extern	IBhandle_t	fd_dbsync;


/*
 * sm sb sync parameters 
 */
int dbsync_main_exit = 0;                   /* dbsync thread exit flag */
int dbsync_initialized_flag = 0;            /* dbsync initialized indicator */
static  uint32_t sm_dbsync_queue_size = 0;  /* dbsync request queue max size */
IBhandle_t  dbsyncfd_if3 = -1;      /* handle to an open if3 MAI connection */
cs_Queue_ptr sm_dbsync_queue;               /* dbsync request queue */
static  uint8_t *msgbuf=NULL;               /* intra SM message receive buffer */
static  int     buflen=0;                   /* intra Sm buffer lendth */
/*
 * hash table of SM's in fabric
 */
SmTable_t   smRecords;

extern uint32_t sm_control_cmd;


// IF3 Wrapper
static inline
Status_t dbSyncCmdToMgr(IBhandle_t mhdl, uint16_t cmd, uint32_t mod,
	      uint8_t * outbuff, uint32_t outlen, uint8_t * inbuff,
	      uint32_t * inlen, uint32_t * resp_status)
{
	Status_t s;

	INCREMENT_COUNTER(smCountersDbSyncSendCmd);
	s = if3_dbsync_cmd_to_mngr(mhdl, cmd, mod, outbuff, outlen, inbuff, inlen, resp_status);

	if (s != VSTATUS_OK)
		INCREMENT_COUNTER(smCountersDbSyncSendCmdFailure);
	return s;
}

// IF3 Wrapper
static inline
Status_t dbSyncMngrRcvData (IBhandle_t fhdl, Mai_t * fimad, uint8_t * inbuff, uint32_t * inlen)
{
	Status_t s;

	INCREMENT_COUNTER(smCountersDbSyncRcvData);
	s = if3_mngr_rcv_fe_data(fhdl, fimad, inbuff, inlen);

	if (s != VSTATUS_OK)
		INCREMENT_COUNTER(smCountersDbSyncRcvDataFailure);
	return s;
}

// IF3 Wrapper
static inline
Status_t dbSyncMngrReply(IBhandle_t fhdl, Mai_t * fmad, uint8_t * outbuff,
                           uint32_t outlen, uint32_t resp_status)
{
	Status_t s;

	INCREMENT_COUNTER(smCountersDbSyncReply);
	s = if3_dbsync_reply_to_mngr(fhdl, fmad, outbuff, outlen, resp_status);

	if (s != VSTATUS_OK)
		INCREMENT_COUNTER(smCountersDbSyncReplyFailure);
	return s;
}

/*
 * sm_dbsync thread startup initialization
 */
void sm_dbsync_init(void) {

    IB_ENTER(__func__, 0, 0, 0, 0);

    if (!dbsync_initialized_flag) {
        dbsync_main_exit = 0;
        if (sm_dbsync_recInit() != VSTATUS_OK) {
            IB_FATAL_ERROR("sa_main: Can't allocate SM Records hash table");
        }
        /* initialize the SM-SA dbsync request queue */
        sm_dbsync_queue_size = 5 * sm_config.subnet_size; /* Be able to handle each end port ...
                                                      sending 4 registrations at once + extra */
        if ((sm_dbsync_queue = cs_queue_CreateQueue( &sm_pool, sm_dbsync_queue_size )) == NULL) {
            IB_FATAL_ERROR("sm_dbsync: failed to initialize the SM-SA dbsync request queue, terminating!");
        }
        dbsyncfd_if3 = -1;
        dbsync_initialized_flag = 1;
    } else {
        IB_LOG_INFO0("dbsync thread already initialized!");
    }
    IB_EXIT(__func__, 0);
}


/*
 * sm_dbsync thread termination control
 */
void sm_dbsync_kill(void){
    dbsync_main_exit = 1;
}


/*
 * make 64 bit Hash key from input
 */
static uint64_t
sm_DbsyncRecHashFromKey(void *ky)
{
    /* key is the SM node's portGuid */
    return *(uint64_t *)ky;
}


/*
 * Sm map compare function
 * return 1 if the 2 entries matche, 0 otherwise
 */
static int32_t
sm_DbsyncRecCompare(void *key1, void *key2) {
    if (*(uint64_t *)key1 == *(uint64_t *)key2) {
        return 1;  /* we have a match */
    } else {
        return 0;  /* no match found */
    }
}


Status_t sm_dbsync_checksums(uint32_t vfCsum, uint32_t smCsum, uint32_t pmCsum) {
    Status_t    status=VSTATUS_OK;

    if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_FATAL_ERROR("sm_dbsync_checksum: Can't lock SM Record table");
    } else {
		smRecords.ourChecksums.protocolVersion = FM_PROTOCOL_VERSION;
    	smRecords.ourChecksums.smVfChecksum = vfCsum;
    	smRecords.ourChecksums.smConfigChecksum = smCsum;
    	smRecords.ourChecksums.pmConfigChecksum = pmCsum;
	}
    (void)vs_unlock(&smRecords.smLock);
    return status;
}

/*
 * SM records hash table initialization
 */
Status_t sm_dbsync_recInit(void) {
    Status_t    status=VSTATUS_OK;
    memset(&smRecords, 0, sizeof(SmTable_t));
    if ((status = vs_lock_init(&smRecords.smLock, VLOCK_FREE, VLOCK_THREAD)) != VSTATUS_OK) {
        IB_FATAL_ERROR("can't initialize SM table lock");
    } else {
        if (NULL == (smRecords.smMap = 
                     cs_create_hashtable("sm_table", 16, sm_DbsyncRecHashFromKey, 
                                         sm_DbsyncRecCompare, CS_HASH_KEY_ALLOCATED))) {
            status = VSTATUS_NOMEM;
            IB_FATAL_ERROR("sa_main: Can't allocate SM table");
        }
    }

	// Don't need to lock old_topology, we are still running single threaded
	sm_dbsync_checksums(old_topology.vfs_ptr->consistency_checksum,
						sm_config.consistency_checksum,
						pm_config.consistency_checksum);

    return status;
}


/*
 * Sm records hash table delete
 */
void sm_dbsync_recDelete(void) {
    if (!smRecords.smMap || vs_lock(&smRecords.smLock) != VSTATUS_OK) return;
    (void) cs_hashtable_destroy(smRecords.smMap, TRUE);  /* free everything */
    smRecords.smMap = 0;
    (void)vs_unlock(&smRecords.smLock);
    (void)vs_lock_delete(&smRecords.smLock);
    memset((void *)&smRecords, 0, sizeof(SmTable_t));
}


/*
 * SM records hash table clear
 */
void sm_dbsync_recClear(void) {
    if (!smRecords.smMap || vs_lock(&smRecords.smLock) != VSTATUS_OK) return;
    /* easier to destroy and recreate */
    (void) cs_hashtable_destroy(smRecords.smMap, TRUE);  /* free everything */
    if (NULL == (smRecords.smMap = 
                 cs_create_hashtable("sm_table", 16, sm_DbsyncRecHashFromKey, 
                                     sm_DbsyncRecCompare, CS_HASH_KEY_ALLOCATED))) {
        IB_FATAL_ERROR("sm_DbsyncRecClear: Can't reallocate SM table");
    }
    (void)vs_unlock(&smRecords.smLock);
}


/*
 * get the sync capability of a standby SM
*/
static Status_t dbsync_getSMDBSync(SMSyncReq_t *syncReqp) {
    Status_t    status=VSTATUS_OK;
    uint32_t    resp_status=0;
    uint32_t    len = SMDBSYNC_NSIZE;
    SMDBSync_t  smSyncCap;

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    memset((void *)&smSyncCap, 0, sizeof(SMDBSync_t));
    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
        if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
            IB_FATAL_ERROR("dbsync_getSMDBSync: can't set destination lid on management info handle");
        }
        if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_SYNC, DBSYNC_AMOD_GET, 
                                   NULL, 0, msgbuf, &len, &resp_status)) != VSTATUS_OK) {
            IB_LOG_INFO_FMT(__func__, "get of standby SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                    syncReqp->portguid, syncReqp->standbyLid, status);
        } else if (len <= 0) {
            status=VSTATUS_BAD;
            if (smDebugPerf) {
                IB_LOG_INFINI_INFO_FMT(__func__, 
                        "Get of sync capability of SM at portGuid "FMT_U64", LID=[0x%x] returned no data",
                        syncReqp->portguid, syncReqp->standbyLid);
            }
        } else if (resp_status == VSTATUS_OK) {
            /* extract the sync capability of standby SM */
            (void)BSWAPCOPY_SM_DBSYNC_DATA((SMDBSyncp)msgbuf, &smSyncCap);
            /* update the standby SM's sync capability and settings in Sm table */
                status = sm_dbsync_upSmDbsync((SmRecKey_t)syncReqp->portguid, &smSyncCap);
                if (status == VSTATUS_OK) {
                    /* set SM sync status to in progress now */
                    if (sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_ALL, DBSYNC_STAT_INPROGRESS)) {
                        IB_LOG_WARN_FMT(__func__, 
                               "Failed to set sync status of SM node at Lid 0x%x, portGuid "FMT_U64" to SM table",
                               syncReqp->standbyLid, syncReqp->portguid);
                    } else {
                        (void) sm_dbsync_queueMsg(DBSYNC_TYPE_FULL, DBSYNC_DATATYPE_ALL, syncReqp->standbyLid, syncReqp->portguid, syncReqp->isEmbedded, NULL);
                        IB_LOG_INFO_FMT(__func__, 
                               "requested full sync of SM node at Lid 0x%x, portGuid "FMT_U64" to SM table",
                               syncReqp->standbyLid, syncReqp->portguid);
                    }
				} else if (status == VSTATUS_AGAIN) {
                    IB_LOG_WARN_FMT(__func__,
                           "Standby sync status unavailable, will retry later, SM at portGuid="FMT_U64" in SM list", syncReqp->portguid);
            	} else {
                    IB_LOG_WARN_FMT(__func__,
                           "Can't update the sync settings for SM at portGuid="FMT_U64" in SM list", syncReqp->portguid);
                }
        } else {
            IB_LOG_WARN_FMT(__func__, 
                    "Get of sync capability of SM at portGuid "FMT_U64", LID=[0x%x] returned error status 0x%x",
                    syncReqp->portguid, syncReqp->standbyLid, resp_status);
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM", 
                syncReqp->portguid, syncReqp->standbyLid);
    }
    IB_EXIT(__func__, 0);
    return status;
}

/*
 * Tell the Standby SM to reread its configuration.
*/
static Status_t dbsync_reconfigSMDBSync(SMSyncReq_t *syncReqp) {
    Status_t    status=VSTATUS_OK;
    uint32_t    resp_status=0;
    uint32_t    len = 0;
    SMDBSync_t  smSyncCap;

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    memset((void *)&smSyncCap, 0, sizeof(SMDBSync_t));
    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
        if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
            IB_FATAL_ERROR("dbsync_getSMDBSync: can't set destination lid on management info handle");
        }
        if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_SYNC, DBSYNC_AMOD_RECONFIG, 
                                   NULL, 0, msgbuf, &len, &resp_status)) != VSTATUS_OK) {
            IB_LOG_INFO_FMT(__func__, "reconfig of standby SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                    syncReqp->portguid, syncReqp->standbyLid, status);
        } else if (resp_status != VSTATUS_OK) {
            IB_LOG_WARN_FMT(__func__, 
                    "Reconfig of Standby SM at portGuid "FMT_U64", LID=[0x%x] returned error status 0x%x",
                    syncReqp->portguid, syncReqp->standbyLid, resp_status);
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM", 
                syncReqp->portguid, syncReqp->standbyLid);
    }
	/* Mark its sync capability as unknown. We'll have to rediscover it after it has reconfigured */
	sm_dbsync_upSmDbsyncCap(syncReqp->portguid, DBSYNC_CAP_UNKNOWN);
    sm_trigger_sweep(SM_SWEEP_REASON_UPDATED_STANDBY);
    IB_EXIT(__func__, 0);
    return status;
}

/*
 * get the sync Config Consistency Checksum info of a standby SM
*/
static Status_t dbsync_getSMDBCCCSync(SMSyncReq_t *syncReqp) {
    Status_t    status=VSTATUS_OK;
    uint32_t    resp_status=0;
    uint32_t    len = SMDBSYNC_CCC_NSIZE;
    SMDBCCCSync_t  smSyncConsistency;

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    memset((void *)&smSyncConsistency, 0, sizeof(SMDBCCCSync_t));
    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
        if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
            IB_FATAL_ERROR("dbsync_getSMDBSync: can't set destination lid on management info handle");
        }
        if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_SYNC, DBSYNC_AMOD_GET_CCC, 
			NULL, 0, msgbuf, &len, &resp_status)) != VSTATUS_OK) {
            IB_LOG_INFO_FMT(__func__, 
            	"get of stanby SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                syncReqp->portguid, syncReqp->standbyLid, status);
        } else if (len <= 0) {
            status=VSTATUS_BAD;
            if (smDebugPerf) {
            	IB_LOG_INFO_FMT(__func__, 
                        "Get of sync config consistency checksum info of SM at portGuid "FMT_U64", LID=[0x%x] returned no data",
                        syncReqp->portguid, syncReqp->standbyLid);
            }
        } else if (resp_status == VSTATUS_OK) {
            /* extract the sync checksums of standby SM */
            (void)BSWAPCOPY_SM_DBSYNC_CCC_DATA((SMDBCCCSyncp)msgbuf, &smSyncConsistency);
            /* check configuration checksum info if set to that level */
			if ((status = sm_dbsync_configCheck((SmRecKey_t)syncReqp->portguid, &smSyncConsistency)) != VSTATUS_OK) {
            		IB_LOG_WARN_FMT(__func__, 
                           "Can't perform config consistency checking for SM at portGuid="FMT_U64" in SM list", syncReqp->portguid);
            }
        } else {
          	IB_LOG_WARN_FMT(__func__, 
                    "Get of sync configuration consistency checking info of SM at portGuid "FMT_U64", LID=[0x%x] returned error status 0x%x",
                    syncReqp->portguid, syncReqp->standbyLid, resp_status);
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM", 
                syncReqp->portguid, syncReqp->standbyLid);
    }
    IB_EXIT(__func__, 0);
    return status;
}

/*
 * set the sync capability of a standby SM
*/
static Status_t dbsync_setSMDBSync(SMSyncReq_t *syncReqp) {
    Status_t    status=VSTATUS_OK;
    uint32_t    resp_status=0;
    uint32_t    len = buflen;
    SMDBSync_t  dbsync={0};

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
        if ((status = sm_dbsync_getSmDbsync(syncReqp->portguid, &dbsync)) != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__,
                   "Failed to get SMDBSync settings for SM at portGuid "FMT_U64", LID=[0x%x] from SM table rc: %u", 
                   syncReqp->portguid, syncReqp->standbyLid, status);
        } else {
            if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
                IB_FATAL_ERROR("dbsync_setSMDBSync: can't set destination lid on management info handle");
            }
            /* setup the sync data for output */
            (void)BSWAPCOPY_SM_DBSYNC_DATA(&dbsync, (SMDBSyncp)msgbuf);

            if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_SYNC, DBSYNC_AMOD_SET, 
                                       msgbuf, SMDBSYNC_NSIZE, msgbuf, &len, &resp_status)) != VSTATUS_OK) {
                IB_LOG_WARN_FMT(__func__, "DBSYNC SET of standby SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                        syncReqp->portguid, syncReqp->standbyLid, status);
            } else if (resp_status != 0) {
                status=VSTATUS_BAD;
                IB_LOG_WARN_FMT(__func__, 
                        "SET of sync capability of SM at portGuid "FMT_U64", LID=[0x%x] returned mad status 0x%x",
                        syncReqp->portguid, syncReqp->standbyLid, resp_status);
            } else {
                /* successfully updated the sync capability of standby SM */
                IB_LOG_INFO_FMT(__func__, 
                        "Updated sync capability of SM at portGuid "FMT_U64", LID=[0x%x]",
                        syncReqp->portguid, syncReqp->standbyLid);
            }
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM", 
                syncReqp->portguid, syncReqp->standbyLid);
    }
    IB_EXIT(__func__, status);
    return status;
}

/*
 * send a file to a standby SM
 */
extern int getPMSweepImageData(char *filename, uint32_t imageIndex, uint8_t isCompressed, uint8_t *buffer, uint32_t bufflen, uint32_t *filelen);
extern int putPMSweepImageData(char *filename, uint8_t *buffer, uint32_t filelen);
extern FSTATUS putPMSweepImageDataR(uint8_t *p_img_in, uint32_t len_img_in);

static Status_t dbsync_sendFileSMDBSync(SMSyncReq_t *syncReqp) {
    Status_t    status=VSTATUS_OK;
    uint32_t    resp_status=0;
    uint32_t    len = buflen;
	uint32_t	outlen;
    SMDBSync_t  dbsync={0};
	SMDBSyncFile_t syncFile;

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
        if ((status = sm_dbsync_getSmDbsync(syncReqp->portguid, &dbsync)) != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__,
                   "Failed to get SMDBSync settings for SM at portGuid "FMT_U64", LID=[0x%x] from SM table rc: %u", 
                   syncReqp->portguid, syncReqp->standbyLid, status);
        } else {
            if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
                IB_FATAL_ERROR("dbsync_sendFileSMDBSync: can't set destination lid on management info handle");
            }

			memcpy(&syncFile, syncReqp->data, sizeof(SMDBSyncFile_t));
			/* see if it the XML configuration file */
			if (syncFile.type == DBSYNC_FILE_XML_CONFIG) {
				if (getXMLConfigData(msgbuf + sizeof(SMDBSyncFile_t), buflen - sizeof(SMDBSyncFile_t), &syncFile.size) < 0) {
					status = VSTATUS_BAD;
					IB_EXIT(__func__, status);
					return status;
				}
			} 
			else if (syncFile.type == DBSYNC_PM_SWEEP_IMAGE || syncFile.type == DBSYNC_PM_HIST_IMAGE) {
				// Composites are not compressed for embedded, compressed otherwise
				uint8_t isCompressed = !syncReqp->isEmbedded;
#ifdef __VXWORKS__
				isCompressed = 0;
#endif
				if (getPMSweepImageData(syncFile.name, syncFile.activate /* hist index */, isCompressed, msgbuf + sizeof(SMDBSyncFile_t), buflen - sizeof(SMDBSyncFile_t), &syncFile.size) < 0) {
					status = VSTATUS_BAD;
					IB_EXIT(__func__, status);
					return status;
				}
			}
			/* handle other file types here */

			/* copy header to buffer */
            (void)BSWAPCOPY_SM_DBSYNC_FILE_DATA(&syncFile, (SMDBSyncFilep)msgbuf);

			outlen = sizeof(SMDBSyncFile_t) + syncFile.size;

            if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_SYNC, DBSYNC_AMOD_SEND_FILE, 
                	msgbuf, outlen, msgbuf, &len, &resp_status)) != VSTATUS_OK) {
                IB_LOG_WARN_FMT(__func__, "DBSYNC send of file %s to standby SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                        syncFile.name, syncReqp->portguid, syncReqp->standbyLid, status);
            } else if (resp_status != 0) {
                status=VSTATUS_BAD;
                IB_LOG_WARN_FMT(__func__, 
                        "Send of file %s to SM at portGuid "FMT_U64", LID=[0x%x] returned mad status 0x%x",
                        syncFile.name, syncReqp->portguid, syncReqp->standbyLid, resp_status);
            } else {
                
                /* successfully sent file to standby SM */
                IB_LOG_INFO_FMT(__func__, 
                        "Send file %s [%d bytes] to SM at portGuid "FMT_U64", LID=[0x%x] ",
                        syncFile.name, outlen, syncReqp->portguid, syncReqp->standbyLid);
            }
			if (syncFile.type == DBSYNC_PM_SWEEP_IMAGE || syncFile.type == DBSYNC_PM_HIST_IMAGE) {
				(void)sm_send_pm_image_complete(syncReqp->portguid, status);
			}
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_WARN_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM", 
                syncReqp->portguid, syncReqp->standbyLid);
    }
    IB_EXIT(__func__, status);
    return status;
}
	

/*
 * STANDBY: process request for GET and SET our sync capability/state
*/
static Status_t processSMDBSyncGetSet(Mai_t *maip, uint8_t *msgbuf, uint32_t len) {
    Status_t    status=VSTATUS_OK;
    SmRecKey_t  reckey = sm_smInfo.PortGUID;  /* our guid */
    SmRecp      smrecp=NULL;
    SmRec_t     smrec={0};
    SMDBSync_t  smSyncCap={0};
	SMDBSyncFile_t syncFile;

    IB_ENTER(__func__, 0, 0, 0, 0);

    if (maip->base.amod == DBSYNC_AMOD_GET) {
        /* lock out SM record table */
        if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
            status = dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
            IB_FATAL_ERROR("dbsync_getSMDBSyncResp: Can't lock SM Record table");
        }
        /* return our current dbsync settings to caller */
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &reckey)) != NULL) {
            memcpy((void *)&smrec, (void *)smrecp, sizeof(SmRec_t));
            smrecp = &smrec;

			if (sm_control_cmd == SM_CONTROL_RECONFIG) {
				/* We haven't finished rereading our configuration, stall the caller */
				smrec.dbsync.version = SM_DBSYNC_UNKNOWN;
			}

            (void)BSWAPCOPY_SM_DBSYNC_DATA(&smrecp->dbsync, (SMDBSyncp)msgbuf);
            /* send the sync capability of standby SM */
			status = dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, SMDBSYNC_NSIZE, VSTATUS_OK);
			IB_LOG_INFO_FMT(__func__,
				"sent sync settings of [version %u, informTimeLastSync=%u, groupTimeLastSync=%u, serviceTimeLastSync=%u]",
				smrecp->dbsync.version, smrecp->dbsync.informTimeLastSync, smrecp->dbsync.groupTimeLastSync,
				smrecp->dbsync.serviceTimeLastSync);
		} else {
            /* return error */
            (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
            status = VSTATUS_BAD;
            IB_LOG_ERROR_FMT(__func__,
							 "Can't find SM record for portGuid="FMT_U64" in SM list", reckey);
        }
        /* unlock the SM table */
        (void)vs_unlock(&smRecords.smLock);
    } else if (maip->base.amod == DBSYNC_AMOD_GET_CCC) {
        /* lock out SM record table */
        if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
            status = dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
            IB_FATAL_ERROR("dbsync_getSMDBSyncResp: Can't lock SM Record table");
        }
		if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &reckey)) != NULL) {
        	/* return our config consistency checksum info to caller */
            memcpy((void *)&smrec, (void *)smrecp, sizeof(SmRec_t));
            smrecp = &smrec;

			memcpy((void *)&smrecp->dbsyncCCC, &smRecords.ourChecksums, sizeof(SMDBCCCSync_t));

            (void)BSWAPCOPY_SM_DBSYNC_CCC_DATA(&smrecp->dbsyncCCC, (SMDBCCCSyncp)msgbuf);

            /* send the sync capability of standby SM */
                status = dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, SMDBSYNC_CCC_NSIZE, VSTATUS_OK);
                IB_LOG_INFO_FMT(__func__,
					"sent sync settings of [version %u, informTimeLastSync=%u, groupTimeLastSync=%u, serviceTimeLastSync=%u]",
					smrecp->dbsync.version, smrecp->dbsync.informTimeLastSync, smrecp->dbsync.groupTimeLastSync,
					smrecp->dbsync.serviceTimeLastSync);
        } else {
            /* return error */
            (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
            status = VSTATUS_BAD;
            IB_LOG_ERROR_FMT(__func__,
                   "Can't find SM record for portGuid="FMT_U64" in SM list", reckey);
        }
        /* unlock the SM table */
        (void)vs_unlock(&smRecords.smLock);
    } else if (maip->base.amod == DBSYNC_AMOD_SET) {
        /* SET: update our data with what is received */
        if (len < SMDBSYNC_NSIZE) {
            /* not enough data sent */
            IB_LOG_ERROR_FMT(__func__, "Expecting %d bytes of data, received %d bytes instead", 
                    SMDBSYNC_NSIZE, len);
            status = VSTATUS_BAD;
            (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
        } else {
            (void)BSWAPCOPY_SM_DBSYNC_DATA((SMDBSyncp)msgbuf, &smSyncCap);
            IB_LOG_INFO_FMT(__func__,
				"Got sync settings of [version %u, INFORM=%u, GROUP=%u, SERVICE=%u]",
				smSyncCap.version, smSyncCap.informTimeLastSync, smSyncCap.groupTimeLastSync,
				smSyncCap.serviceTimeLastSync);
                /* update our SM's sync capability and settings in SM table */
                if ((status = sm_dbsync_upSmDbsync(reckey, &smSyncCap)) != VSTATUS_OK) {
                    IB_LOG_WARN_FMT(__func__,
                           "Can't update the sync settings for our SM at portGuid="FMT_U64" in SM list", sm_smInfo.PortGUID);
                    /* return error */
                    (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
                } else {
                    /* return positive status */
                    (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_OK);
            }
        }
    } else if (maip->base.amod == DBSYNC_AMOD_SEND_FILE) {
		if (len < SMDBSYNC_SEND_FILE_NSIZE) {
			/* not enough data sent */
			IB_LOG_ERROR_FMT(__func__, "Expecting %d bytes of data, received %d bytes instead",
				SMDBSYNC_SEND_FILE_NSIZE, len);
			status = VSTATUS_BAD;
			(void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
		} else {
            (void)BSWAPCOPY_SM_DBSYNC_FILE_DATA((SMDBSyncFilep)msgbuf, &syncFile);
            /* if XML config file then capy it into the appropriate place */
			if (syncFile.type == DBSYNC_FILE_XML_CONFIG) {
				if (putXMLConfigData(msgbuf + syncFile.length, syncFile.size) < 0) {
					(void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
				} else {
        			/* return positive status */
        			(void)dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_OK);
				}
			}
			else if (syncFile.type == DBSYNC_PM_SWEEP_IMAGE) {
				if (putPMSweepImageDataR(msgbuf + syncFile.length, syncFile.size) != 0) {
					(void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
				} else {
					/* return positive status */
					(void)dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_OK);
				}
			}
			else if (syncFile.type == DBSYNC_PM_HIST_IMAGE) {
#if CPU_LE
				// Process STH files only on LE CPUs
				if (putPMSweepImageData(syncFile.name, msgbuf + syncFile.length, syncFile.size) < 0) {
					(void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
				} else {
					/* return positive status */
					(void)dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_OK);
				}
#else
				/* Do not process DBSYNC_PM_HIST_IMAGE on BE CPUs, but return positive status */
				(void)dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_OK);
#endif	// End of #if CPU_LE
			}
			/* handle other file types here in the future */
		}
    } else if (maip->base.amod == DBSYNC_AMOD_RECONFIG) {
		sm_control_reconfig();
        status = dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_OK);
        IB_LOG_INFO_FMT(__func__, 
                        "Reconfiguration request processed");
    } else {
        IB_LOG_WARN_FMT(__func__,
               "Invalid amod [%d] received; should be 1=GET, 2=SET, 5=GET_CCC", maip->base.amod);
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
    }
    IB_EXIT(__func__, status);
    return status;
}


/*
 * sync all INFORM records to a standby SM
*/
static Status_t dbsync_setSMDBInform(SMSyncReq_t *syncReqp) {
    Status_t        status=VSTATUS_OK;
    uint32_t        resp_status=0;
	STL_INFORM_INFO_RECORD *	iRecordp = NULL;
    SubscriberKeyp  subsKeyp = NULL;
    CS_HashTableItr_t itr;
    uint32_t        numRecs=0, outlen=0, inlen=0;
    uint8_t        *buff;

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
        /*
         *	create output buffer from hashtable records
         */
        (void)vs_lock(&saSubscribers.subsLock);
        if ((numRecs = cs_hashtable_count(saSubscribers.subsMap)) > 0)
        {
			
            outlen = (sizeof(*subsKeyp) + sizeof(STL_INFORM_INFO_RECORD)) * numRecs;
			outlen += sizeof(numRecs);
            /* make sure data will fit in output buffer */
            if (outlen > buflen) {
                IB_LOG_WARN_FMT(__func__, 
                        "size of subscriptions sync buffer (%d) larger than output buffer (%d)! Increase subnet size",
                         outlen, buflen);
                status = VSTATUS_NOMEM;
                (void)vs_unlock(&saSubscribers.subsLock);
                IB_EXIT(__func__, status);
                return status;
            }
            buff = msgbuf;
			BSWAPCOPY_SM_DBSYNC_RECORD_CNT(&numRecs, (uint32_t*)buff);
			buff += sizeof(numRecs);
            cs_hashtable_iterator(saSubscribers.subsMap, &itr);
            do {
                subsKeyp = cs_hashtable_iterator_key(&itr);
                iRecordp = cs_hashtable_iterator_value(&itr);
                (void)BSWAPCOPY_SM_DBSYNC_SUBSKEY_DATA(subsKeyp, (SubscriberKeyp)buff);
                buff += sizeof(*subsKeyp);
				((STL_INFORM_INFO_RECORD*)buff)->Reserved = 0;
				BSWAPCOPY_STL_INFORM_INFO_RECORD(iRecordp, (STL_INFORM_INFO_RECORD*)buff);
                buff += sizeof(STL_INFORM_INFO_RECORD);
            } while (cs_hashtable_iterator_advance(&itr));
        }
        (void)vs_unlock(&saSubscribers.subsLock);

        /* send the records to the standby SM */
        if (numRecs) {
            if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
                IB_LOG_ERROR("can't set destination lid on management info handle=", dbsyncfd_if3);
            } else {
                if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_INFORM, DBSYNC_AMOD_SET, 
                                           msgbuf, outlen, msgbuf, &inlen, &resp_status)) != VSTATUS_OK) {
                    IB_LOG_WARN_FMT(__func__, 
                            "FULL sync of %d [len=%d] INFORM records to SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                            numRecs, outlen, syncReqp->portguid, syncReqp->standbyLid, status);
                } else if (resp_status != 0) {
                    status=VSTATUS_BAD;
                    IB_LOG_WARN_FMT(__func__, 
                            "Full sync of %d INFORM records to SM at portGuid "FMT_U64", LID=[0x%x] returned mad status 0x%x",
                            numRecs, syncReqp->portguid, syncReqp->standbyLid, resp_status);
                } else {
                    /* successfully sync'd all INFORM records to standby SM */
                    IB_LOG_INFO_FMT(__func__, 
                            "Full sync of %d INFORM records to SM at portGuid "FMT_U64", LID=[0x%x] was successfull",
                            numRecs, syncReqp->portguid, syncReqp->standbyLid);
                }
            }
        } else {
            IB_LOG_INFO_FMT(__func__, 
                    "NO INFORM records to sync to SM at portGuid "FMT_U64", LID=[0x%x]",
                    syncReqp->portguid, syncReqp->standbyLid);
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM", 
                syncReqp->portguid, syncReqp->standbyLid);
    }

    IB_EXIT(__func__, status);
    return status;
}


/*
 * sync a creation, deletion, or update of an INFORM record to a standby SM
*/
static Status_t dbsync_updateSMDBInform(SMSyncReq_t *syncReqp) {
    Status_t        status=VSTATUS_OK;
    uint32_t        resp_status=0;
	STL_INFORM_INFO_RECORD	iRecord;
    SubscriberKey_t subsKey;
    uint32_t        numRecs=0, outlen=0, inlen=0;
    uint8_t         *buff = msgbuf;
    void            *ptr = NULL;

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
        numRecs++;
        ptr = (void *)syncReqp->data;
        memcpy((void *)&subsKey, ptr, sizeof(SubscriberKey_t));
        memcpy((void *)&iRecord, ptr+sizeof(SubscriberKey_t), sizeof(STL_INFORM_INFO_RECORD));
        BSWAPCOPY_SM_DBSYNC_RECORD_CNT(&numRecs, (uint32_t*)buff);
        buff += sizeof(numRecs);
        (void)BSWAPCOPY_SM_DBSYNC_SUBSKEY_DATA(&subsKey, (SubscriberKeyp)buff);
        buff += sizeof(subsKey);;
		BSWAPCOPY_STL_INFORM_INFO_RECORD(&iRecord, (STL_INFORM_INFO_RECORD*)buff);
        buff += sizeof(STL_INFORM_INFO_RECORD);
        outlen = sizeof(subsKey) + sizeof(STL_INFORM_INFO_RECORD);
        outlen += sizeof(numRecs);
        if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
            IB_LOG_ERROR("can't set destination lid on management info handle=", dbsyncfd_if3);
        } else {
            if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_INFORM, syncReqp->type, 
                                       msgbuf, outlen, msgbuf, &inlen, &resp_status)) != VSTATUS_OK) {
                IB_LOG_INFO_FMT(__func__, 
                        "%s of INFORM record to SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                        ((syncReqp->type == DBSYNC_TYPE_UPDATE) ? "UPDATE":"DELETE"), syncReqp->portguid, syncReqp->standbyLid, status);
            } else if (resp_status != 0) {
                status=VSTATUS_BAD;
                IB_LOG_WARN_FMT(__func__, 
                        "%s sync of INFORM records to SM at portGuid "FMT_U64", LID=[0x%x] returned mad status 0x%x",
                        ((syncReqp->type == DBSYNC_TYPE_UPDATE) ? "UPDATE":"DELETE"), syncReqp->portguid, syncReqp->standbyLid, resp_status);
            } else {
                /* successfully sync'd all INFORM records to standby SM */
                IB_LOG_INFO_FMT(__func__, 
                        "%s of INFORM record to SM at portGuid "FMT_U64", LID=[0x%x] was successfull",
                        ((syncReqp->type == DBSYNC_TYPE_UPDATE) ? "UPDATE":"DELETE"), syncReqp->portguid, syncReqp->standbyLid);
            }
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM", 
                syncReqp->portguid, syncReqp->standbyLid);
    }
    /* update INFORM sync status if failure */
    if (status) {
        (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_INFORM, DBSYNC_STAT_FAILURE);
    }

    IB_EXIT(__func__, status);
    return status;
}


/*
 * STANDBY: process request for FULL sync of INFORM records
*/
static Status_t processInformSync(Mai_t *maip, uint8_t *msgbuf, uint32_t reclen) {
    Status_t        status=VSTATUS_OK;
    uint32_t        bufidx=0, numRecs;
	STL_INFORM_INFO_RECORD	iRecord;
    STL_INFORM_INFO_RECORD *   iRecordp;
    SubscriberKey_t skey;
    SubscriberKeyp  subsKeyp;

    IB_ENTER(__func__, 0, 0, 0, 0);

	memset(&iRecord,0,sizeof(iRecord));
    if (reclen > buflen) {
        IB_LOG_ERROR_FMT(__func__, 
                "Received length (%d bytes) greater than allocated (%d bytes), PLEASE INCREASE THE FABRIC SUPPORTED END PORT COUNT", 
                SMDBSYNC_NSIZE, reclen);
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
        IB_EXIT(__func__, VSTATUS_NXIO);
        return VSTATUS_NXIO;
    }
    IB_LOG_INFO_FMT(__func__,
		"Processing sync of INFORM records with (%d) bytes of data (%d records) \n",
		reclen, (int)(reclen / (sizeof(STL_INFORM_INFO_RECORD) + sizeof(skey))));
    BSWAPCOPY_SM_DBSYNC_RECORD_CNT((uint32_t*)&msgbuf[bufidx],&numRecs);
    bufidx += sizeof(numRecs);
    if (maip->base.amod == DBSYNC_AMOD_SET) {
        /* clear out the subscription table */
        (void) sa_SubscriberClear();
        while (numRecs-- && (bufidx < reclen)) {
            (void)BSWAPCOPY_SM_DBSYNC_SUBSKEY_DATA((SubscriberKeyp)&msgbuf[bufidx], &skey);
            bufidx += sizeof(skey);
			BSWAPCOPY_STL_INFORM_INFO_RECORD((STL_INFORM_INFO_RECORD*)&msgbuf[bufidx], &iRecord);
            bufidx += sizeof(STL_INFORM_INFO_RECORD);
            if (vs_lock(&saSubscribers.subsLock) != VSTATUS_OK) return VSTATUS_BAD;
            /* allocate subscriber key storage */
            if ((subsKeyp = (SubscriberKeyp) malloc(sizeof(SubscriberKey_t))) == NULL) {
                (void)vs_unlock(&saSubscribers.subsLock);
                status = VSTATUS_NOMEM;
                IB_LOG_ERROR0("Can't allocate subscriber entry/key for SET");
                break;
            } else {
                /* move over the key data */
                memcpy((void *)subsKeyp, (void *)&skey, sizeof(SubscriberKey_t));
                /* allocate subscriber iRecord for adding to hash table */
                if ((iRecordp = (STL_INFORM_INFO_RECORD *) malloc(sizeof(STL_INFORM_INFO_RECORD))) == NULL) {
                    free(subsKeyp);
                    (void)vs_unlock(&saSubscribers.subsLock);
                    IB_LOG_ERROR0("Can't allocate informInfoRecord entry for SET");
                    status = VSTATUS_NOMEM;
                    break;
                }
                memcpy((void *)iRecordp, (void *)&iRecord, sizeof(STL_INFORM_INFO_RECORD));
                if (!cs_hashtable_insert(saSubscribers.subsMap, subsKeyp, iRecordp)) {
                    free(subsKeyp);
                    free(iRecordp);
                    (void)vs_unlock(&saSubscribers.subsLock);
                    IB_LOG_ERROR_FMT(__func__, 
                           "Failed to insert sync'd subscription ID %d in Subscriber table (SET)",
                           iRecord.RID.Enum);
                    status = VSTATUS_BAD;
                    break;
                } else {
                    IB_LOG_INFO_FMT(__func__, 
                           "ADDED (SET) subscription ID %d with subscriber lid 0x%.8X to subscription table",
                           iRecordp->RID.Enum, subsKeyp->lid);
                    status = VSTATUS_OK;
                    (void)vs_unlock(&saSubscribers.subsLock);
                }
            }
        }
        /* return appropriate status to caller */
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, ((status == VSTATUS_OK) ? VSTATUS_OK : VSTATUS_DROP));
    } else if (maip->base.amod == DBSYNC_AMOD_ADD) {
        /* ADD: add/update entry in dataset */
        while (numRecs -- && (bufidx < reclen)) {
            (void)BSWAPCOPY_SM_DBSYNC_SUBSKEY_DATA((SubscriberKeyp)&msgbuf[bufidx], &skey);
            bufidx += sizeof(skey);
            BSWAPCOPY_STL_INFORM_INFO_RECORD((STL_INFORM_INFO_RECORD*)&msgbuf[bufidx], &iRecord);
            bufidx += sizeof(STL_INFORM_INFO_RECORD);
            if (vs_lock(&saSubscribers.subsLock) != VSTATUS_OK) return VSTATUS_BAD;
            /* replace existing entry or create a new one */
            if (NULL == (iRecordp = (STL_INFORM_INFO_RECORD *)cs_hashtable_search(saSubscribers.subsMap, &skey))) {
                /* allocate a subscriber key and iRecord for adding to hash table */
                if ((subsKeyp = (SubscriberKeyp) malloc(sizeof(SubscriberKey_t))) == NULL) {
                    (void)vs_unlock(&saSubscribers.subsLock);
                    status = VSTATUS_NOMEM;
                    IB_LOG_ERROR0("Can't allocate subscriber entry/key for ADD");
                    break;
                } else {
                    /* move over the key data */
                    memcpy((void *)subsKeyp, (void *)&skey, sizeof(SubscriberKey_t));
                    /* allocate subscriber iRecord for adding to hash table */
                    if ((iRecordp = (STL_INFORM_INFO_RECORD *) malloc(sizeof(STL_INFORM_INFO_RECORD))) == NULL) {
                        free(subsKeyp);
                        (void)vs_unlock(&saSubscribers.subsLock);
                        status = VSTATUS_NOMEM;
                        IB_LOG_ERROR0("Can't allocate informInfoRecord entry for ADD");
                        break;
                    } else {
                        memcpy((void *)iRecordp, (void *)&iRecord, sizeof(STL_INFORM_INFO_RECORD));
                        if (!cs_hashtable_insert(saSubscribers.subsMap, subsKeyp, iRecordp)) {
                            free(subsKeyp);
                            free(iRecordp);
                            (void)vs_unlock(&saSubscribers.subsLock);
                            status = VSTATUS_BAD;
                            IB_LOG_ERROR_FMT(__func__, 
                                "Failed to ADD sync'd subscription ID %d to Subscriber table", 
								iRecord.RID.Enum);
                            break;
                        } else {
                            IB_LOG_INFO_FMT(__func__, 
                                   "ADDED subscription ID %d with subscriber lid of 0x%.8X to subscription table",
                                   iRecordp->RID.Enum, skey.lid);
                            (void)vs_unlock(&saSubscribers.subsLock);
                        }
                    }
                }
            } else {
                /* replace existing record contents */
                iRecordp->RID.SubscriberLID = iRecord.RID.SubscriberLID;
                iRecordp->RID.Enum = iRecord.RID.Enum;
                iRecordp->InformInfoData = iRecord.InformInfoData;
                IB_LOG_INFO_FMT(__func__, 
                       "UPDATED (ADD) subscription ID %d with subscriber lid of 0x%.8X in subscription table",
                       iRecordp->RID.Enum, skey.lid);
                (void)vs_unlock(&saSubscribers.subsLock);
            }
        }
        /* return appropriate status to caller */
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, ((status == VSTATUS_OK) ? VSTATUS_OK : VSTATUS_DROP));
    } else if (maip->base.amod == DBSYNC_AMOD_DELETE) {
        /* DELETE: remove record from dataset */
        while (numRecs-- && (bufidx < reclen)) {
            (void)BSWAPCOPY_SM_DBSYNC_SUBSKEY_DATA((SubscriberKeyp)&msgbuf[bufidx], &skey);
            bufidx += sizeof(skey);
            BSWAPCOPY_STL_INFORM_INFO_RECORD((STL_INFORM_INFO_RECORD*)&msgbuf[bufidx], &iRecord);
            bufidx += sizeof(STL_INFORM_INFO_RECORD);
            if (vs_lock(&saSubscribers.subsLock) != VSTATUS_OK) return VSTATUS_BAD;
            if (NULL == (iRecordp = (STL_INFORM_INFO_RECORD *)cs_hashtable_remove(saSubscribers.subsMap, &skey))) {
                IB_LOG_INFO_FMT(__func__, 
                       "Could not find subscription ID %d with subscriber lid of 0x%.8X",
                       iRecord.RID.Enum, skey.lid);
            } else {
                /* free the actual subscriber iInformInfoRecord - remove only frees the key */
                free(iRecordp);
                IB_LOG_INFO_FMT(__func__, 
                       "DELETED subscription ID %d with subscriber lid of 0x%.8X",
                       iRecord.RID.Enum, skey.lid);
            }
            (void)vs_unlock(&saSubscribers.subsLock);
        }
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_OK);
    } else {
        IB_LOG_WARN_FMT(__func__,
               "Invalid amod [%d] received; should be 2=SET, 3=ADD, or 4=DELETE", maip->base.amod);
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
    }
    IB_EXIT(__func__, status);
    return status;
}


/*
 * get number of members in group
 */
static int getGrpMembCount(McGroup_t *mcgp) {
	McMember_t  *mcmp;
    int         cnt=0;

    for_all_multicast_members(mcgp, mcmp) {
        ++cnt;
    }
    return cnt;
}


/*
 * sync MCAST GROUP records to a standby SM
*/
static Status_t dbsync_setSMDBGroup(SMSyncReq_t *syncReqp) {
    Status_t        status=VSTATUS_OK;
	McGroup_t		*mcgp;
	McMember_t		*mcmp;
    McGroupSync_t   grpSync;
    STL_MCMEMBER_SYNCDB membSync;
    uint32_t        resp_status=0;
    uint32_t        grpcnt=0, outlen=0, inlen=0;
    uint8_t         *buff=msgbuf;

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
    	if ((status = vs_lock(&sm_McGroups_lock)) != VSTATUS_OK) {
    		IB_LOG_ERROR0("Failed to get sm_McGroups_lock");
            return status;
    	}
		
        buff+=sizeof(grpcnt);
        for_all_multicast_groups(mcgp)
        {
            ++grpcnt;
            memcpy((void *)&grpSync.mGid, (void *)&mcgp->mGid, sizeof(IB_GID));
            grpSync.members_full = mcgp->members_full;
            grpSync.qKey = mcgp->qKey;
            grpSync.pKey = mcgp->pKey;
            grpSync.mLid = mcgp->mLid;
            grpSync.mtu = mcgp->mtu;
            grpSync.rate = mcgp->rate;
            grpSync.life = mcgp->life;
            grpSync.sl = mcgp->sl;
            grpSync.flowLabel = mcgp->flowLabel;
            grpSync.hopLimit = mcgp->hopLimit;
            grpSync.tClass = mcgp->tClass;
            grpSync.scope = mcgp->scope;
            grpSync.membercount = getGrpMembCount(mcgp);
            grpSync.index_pool = mcgp->index_pool;
			grpSync.filler = 0;
            /* make sure we're not over running the out buf */
            if ( ((buff+sizeof(McGroupSync_t)) - msgbuf) > buflen) {
                IB_LOG_WARN_FMT(__func__,
					"size of group records sync buffer (%d) larger than output buffer(%d)! Increase subnet size",
					(int)(buff-msgbuf), buflen);
                status = VSTATUS_NOMEM;
                vs_unlock(&sm_McGroups_lock);
                IB_EXIT(__func__, status);
                return status;
            }
            (void)BSWAPCOPY_SM_DBSYNC_MC_GROUP_DATA(&grpSync, (McGroupSync_t *)buff);
            buff += sizeof(McGroupSync_t);
            for_all_multicast_members(mcgp, mcmp) {
                membSync.index = mcmp->index;
                membSync.slid = mcmp->slid;
                membSync.proxy = mcmp->proxy;
                membSync.state = mcmp->state;
                membSync.nodeGuid = mcmp->nodeGuid;
                membSync.portGuid = mcmp->portGuid;
                memcpy((void *)&membSync.member, (void *)&mcmp->record, sizeof(STL_MCMEMBER_RECORD));
                /* make sure we're not over running the out buf */
                if ( ((buff+sizeof(STL_MCMEMBER_SYNCDB)) - msgbuf) > buflen) {
                    IB_LOG_WARN_FMT(__func__,
						"size of group records sync buffer (%d) larger than output buffer(%d)! Increase subnet size",
						(int)(buff-msgbuf), buflen);
                    status = VSTATUS_NOMEM;
                    vs_unlock(&sm_McGroups_lock);
                    IB_EXIT(__func__, status);
                    return status;
                }
                BSWAPCOPY_STL_MCMEMBER_SYNCDB(&membSync, (STL_MCMEMBER_SYNCDB*)buff);
                buff += sizeof(STL_MCMEMBER_SYNCDB);
            }
        }
        vs_unlock(&sm_McGroups_lock);
        /* Add record count to start of message */
        BSWAPCOPY_SM_DBSYNC_RECORD_CNT(&grpcnt, (uint32_t*)msgbuf);
        /* calculate the length of the output data */
        outlen = buff - msgbuf;
        /* send the records to the standby SM */
        if (outlen) {
            if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
                IB_LOG_ERROR("can't set destination lid on management info handle=", dbsyncfd_if3);
            } else {
                if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_GROUP, DBSYNC_AMOD_SET, 
                                           msgbuf, outlen, msgbuf, &inlen, &resp_status)) != VSTATUS_OK) {
                    IB_LOG_WARN_FMT(__func__, 
                            "DBSYNC of %d [len=%d] GROUP records to SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                            grpcnt, outlen, syncReqp->portguid, syncReqp->standbyLid, status);
                } else if (resp_status != 0) {
                    status=VSTATUS_BAD;
                    IB_LOG_WARN_FMT(__func__, 
                            "Full sync of %d GROUP records to SM at portGuid "FMT_U64", LID=[0x%x] returned mad status 0x%x",
                            grpcnt, syncReqp->portguid, syncReqp->standbyLid, resp_status);
                } else {
                    /* successfully sync'd all INFORM records to standby SM */
                    IB_LOG_INFO_FMT(__func__, 
                            "Full sync of %d GROUP records to SM at portGuid "FMT_U64", LID=[0x%x] was successfull",
                            grpcnt, syncReqp->portguid, syncReqp->standbyLid);
                }
            }
        } else {
            IB_LOG_INFO_FMT(__func__, 
                    "NO GROUP records to sync to SM at portGuid "FMT_U64", LID=[0x%x]",
                    syncReqp->portguid, syncReqp->standbyLid);
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM in request to sync GROUP records", 
                syncReqp->portguid, syncReqp->standbyLid);
    }

    IB_EXIT(__func__, status);
    return status;
}


/*
 * sync a creation, deletion, or update of a GROUP record to a standby SM
*/
static Status_t dbsync_updateSMDBGroup(SMSyncReq_t *syncReqp) {
    Status_t        status=VSTATUS_OK;
    uint32_t        resp_status=0;
    IB_GID           grpgid;
	McGroup_t		*mcgp;
	McMember_t		*mcmp;
    McGroupSync_t   grpSync;
    STL_MCMEMBER_SYNCDB  membSync;
    uint32_t        outlen=0, inlen=0, grpcnt = 0;
    uint8_t         *buff = msgbuf;

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
        memset((void *)&grpSync, 0, sizeof(grpSync));
        memset((void *)&membSync, 0, sizeof(membSync));
        if (syncReqp->type == DBSYNC_TYPE_UPDATE) {
            if ((status = vs_lock(&sm_McGroups_lock)) != VSTATUS_OK) {
                IB_LOG_ERRORRC("Can't lock GROUP table, rc:",status);
                IB_EXIT(__func__, status);
                return status;
            }
            memcpy((void *)&grpgid, syncReqp->data, sizeof(IB_GID));
            if ((mcgp = sm_find_multicast_gid(grpgid))) {
                grpcnt++;;
                buff+=sizeof(grpcnt);
                memcpy((void *)&grpSync.mGid, (void *)&mcgp->mGid, sizeof(IB_GID));
                grpSync.members_full = mcgp->members_full;
                grpSync.qKey = mcgp->qKey;
                grpSync.pKey = mcgp->pKey;
                grpSync.mLid = mcgp->mLid;
                grpSync.mtu = mcgp->mtu;
                grpSync.rate = mcgp->rate;
                grpSync.life = mcgp->life;
                grpSync.sl = mcgp->sl;
                grpSync.flowLabel = mcgp->flowLabel;
                grpSync.hopLimit = mcgp->hopLimit;
                grpSync.tClass = mcgp->tClass;
                grpSync.scope = mcgp->scope;
                grpSync.membercount = getGrpMembCount(mcgp);
                grpSync.index_pool = mcgp->index_pool;
                (void)BSWAPCOPY_SM_DBSYNC_MC_GROUP_DATA(&grpSync, (McGroupSync_t *)buff);
                buff += sizeof(McGroupSync_t);
                for_all_multicast_members(mcgp, mcmp) {
                    membSync.index = mcmp->index;
                    membSync.slid = mcmp->slid;
                    membSync.proxy = mcmp->proxy;
                    membSync.state = mcmp->state;
                    membSync.nodeGuid = mcmp->nodeGuid;
                    membSync.portGuid = mcmp->portGuid;
                    memcpy((void *)&membSync.member, (void *)&mcmp->record, sizeof(STL_MCMEMBER_RECORD));
                    (void)BSWAPCOPY_STL_MCMEMBER_SYNCDB(&membSync, (STL_MCMEMBER_SYNCDB*)buff);
                    buff += sizeof(STL_MCMEMBER_SYNCDB);
                }
            } else {
                /* group not found */
                outlen = 0;
            }
            (void)vs_unlock(&sm_McGroups_lock);
        } else if (syncReqp->type == DBSYNC_TYPE_DELETE) {
            /* all we have is the group's gid */ 
            buff+=sizeof(grpcnt);
            grpcnt++;
            memcpy((void *)&grpSync.mGid, syncReqp->data, sizeof(IB_GID));
            (void)BSWAPCOPY_SM_DBSYNC_MC_GROUP_DATA(&grpSync, (McGroupSync_t *)buff);
            buff += sizeof(McGroupSync_t);
        } else {
            /* invalid sync type */
            outlen = 0;
        }
        /* Add record count to start of message */
        BSWAPCOPY_SM_DBSYNC_RECORD_CNT(&grpcnt, (uint32_t*)msgbuf);
        /* calculate the length of the output data */
        outlen = buff - msgbuf;
        /* send the data to the standby SM */
        if (outlen) {
            if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
                IB_LOG_ERROR("can't set destination lid on management info handle=", dbsyncfd_if3);
            } else {
                if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_GROUP, syncReqp->type, 
                                           msgbuf, outlen, msgbuf, &inlen, &resp_status)) != VSTATUS_OK) {
                    IB_LOG_WARN_FMT(__func__, 
                            "%s of GROUP record to SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                            ((syncReqp->type == DBSYNC_TYPE_UPDATE) ? "UPDATE":"DELETE"), syncReqp->portguid, syncReqp->standbyLid, status);
                } else if (resp_status != 0) {
                    status=VSTATUS_BAD;
                    IB_LOG_WARN_FMT(__func__, 
                            "%s of GROUP record to SM at portGuid "FMT_U64", LID=[0x%x] returned mad status 0x%x",
                            ((syncReqp->type == DBSYNC_TYPE_UPDATE) ? "UPDATE":"DELETE"), syncReqp->portguid, syncReqp->standbyLid, resp_status);
                } else {
                    /* successfully sync'd all INFORM records to standby SM */
                    IB_LOG_INFO_FMT(__func__, 
                            "%s of GROUP record to SM at portGuid "FMT_U64", LID=[0x%x] was successfull",
                            ((syncReqp->type == DBSYNC_TYPE_UPDATE) ? "UPDATE":"DELETE"), syncReqp->portguid, syncReqp->standbyLid);
                }
            }
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM", 
                syncReqp->portguid, syncReqp->standbyLid);
    }

    IB_EXIT(__func__, status);
    return status;
}


/*
 * STANDBY: process request for sync of GROUP records
*/
static Status_t processGroupSync(Mai_t *maip, uint8_t *msgbuf, uint32_t reclen) {
    Status_t        status=VSTATUS_OK;
    uint32_t        bufidx=0;
	McGroupSync_t	mcgs;
	STL_MCMEMBER_SYNCDB	mcms;
	McGroup_t       *mcGroup;
	McMember_t      *mcMember;
    uint32_t        memcnt=0;
	int				vf = 0;
	uint64_t		vfmGid[2];
	uint32			grpcnt;

    IB_ENTER(__func__, maip, 0, 0, 0);

    if (reclen > buflen) {
        IB_LOG_ERROR_FMT(__func__, 
                "Received length (%d bytes) greater than allocated (%d bytes), PLEASE INCREASE THE FABRIC SUPPORTED END PORT COUNT", 
                SMDBSYNC_NSIZE, buflen);
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
        IB_EXIT(__func__, VSTATUS_NXIO);
        return VSTATUS_NXIO;
    }
    IB_LOG_INFO_FMT(__func__, 
           "Processing sync of GROUP records with (%d) bytes of data", reclen);

    BSWAPCOPY_SM_DBSYNC_RECORD_CNT((uint32_t*)&msgbuf[bufidx],&grpcnt);
    bufidx += sizeof(grpcnt);
    if (maip->base.amod == DBSYNC_AMOD_SET) {
        /* clear out and rebuild the Group table */
        (void)clearBroadcastGroups(FALSE);
        while (grpcnt-- && (bufidx < reclen)) {
            memcnt = 0;     /* new group, reset member count */
            (void)BSWAPCOPY_SM_DBSYNC_MC_GROUP_DATA((McGroupSync_t *)&msgbuf[bufidx], &mcgs);
            bufidx += sizeof(McGroupSync_t);
            /* create the group */
            (void)vs_lock(&sm_McGroups_lock);
            if ((status = sm_multicast_sync_lid(*((IB_GID*)mcgs.mGid), mcgs.pKey, mcgs.mtu, mcgs.rate, mcgs.mLid)) == VSTATUS_OK) {
                McGroup_Create(mcGroup);
                memcpy(&(mcGroup->mGid), &(mcgs.mGid), sizeof(IB_GID));
                mcGroup->members_full = mcgs.members_full;
                mcGroup->qKey = mcgs.qKey;
                mcGroup->pKey = mcgs.pKey;
                mcGroup->mLid = mcgs.mLid;
                mcGroup->mtu = mcgs.mtu;
                mcGroup->rate = mcgs.rate;
                mcGroup->life = mcgs.life;
                mcGroup->sl = mcgs.sl;
                mcGroup->flowLabel = mcgs.flowLabel;
                mcGroup->tClass = mcgs.tClass;
                mcGroup->hopLimit = mcgs.hopLimit;
                mcGroup->scope = mcgs.scope;
                mcGroup->index_pool = mcgs.index_pool;
                /* add members to group */
                while (bufidx < reclen && memcnt < mcgs.membercount) {
				    BSWAPCOPY_STL_MCMEMBER_SYNCDB((STL_MCMEMBER_SYNCDB*)&msgbuf[bufidx], &mcms);
                    bufidx += sizeof(STL_MCMEMBER_SYNCDB);
                    ++memcnt;
                    McMember_Create(mcGroup, mcMember);
                    mcMember->slid = mcms.slid;
                    mcMember->proxy = mcms.proxy;
                    mcMember->state = mcms.state;
                    mcMember->nodeGuid = mcms.nodeGuid;
                    mcMember->portGuid = mcms.portGuid;
                    mcMember->record = mcms.member;
                    mcMember->index = mcms.index;
                }
				// Remove this usage once ib_sa.h:McGroupSync_t:mGid changes type
                IB_LOG_INFO_FMT(__func__,
                       "created group " FMT_GID " with %d members", ((IB_GID*)mcgs.mGid)->Type.Global.SubnetPrefix, ((IB_GID*)mcgs.mGid)->Type.Global.InterfaceID, mcgs.membercount);

				(void)vs_rdlock(&old_topology_lock);
				VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;
				if (VirtualFabrics) {
					vfmGid[0] = mcGroup->mGid.AsReg64s.H;
					vfmGid[1] = mcGroup->mGid.AsReg64s.L;
        			for (vf=0; vf < VirtualFabrics->number_of_vfs; vf++) {
            			if ((PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey) == PKEY_VALUE(mcGroup->pKey)) &&
                			(smVFValidateMcDefaultGroup(vf, vfmGid) == VSTATUS_OK)) {
                			uint32 vfIdx=VirtualFabrics->v_fabric[vf].index;
                			bitset_set(&mcGroup->vfMembers, vfIdx);
            			}
        			}
				}
				(void)vs_rwunlock(&old_topology_lock);

            } else {
				// Remove this usage once ib_sa.h:McGroupSync_t:mGid changes type
                IB_LOG_WARN_FMT(__func__,
                       "can't create group " FMT_GID " with %d members", ((IB_GID*)mcgs.mGid)->Type.Global.SubnetPrefix, ((IB_GID*)mcgs.mGid)->Type.Global.InterfaceID, mcgs.membercount);
                (void)vs_unlock(&sm_McGroups_lock);
                /* can't create the group on this side, drop the sync packet */
                (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
                IB_EXIT(__func__, status);
                return status;
            }
            (void)vs_unlock(&sm_McGroups_lock);
        }
        /* return appropriate status to caller */
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, ((status == VSTATUS_OK) ? VSTATUS_OK : VSTATUS_DROP));
    } else if (maip->base.amod == DBSYNC_AMOD_ADD) {
        /* ADD: add/update entry in dataset */
        while (grpcnt-- && (bufidx < reclen)) {
            (void)BSWAPCOPY_SM_DBSYNC_MC_GROUP_DATA((McGroupSync_t *)&msgbuf[bufidx], &mcgs);
            bufidx += sizeof(McGroupSync_t);
            /* delete group if exist */
            (void)vs_lock(&sm_McGroups_lock);
    		if ((mcGroup = sm_find_multicast_gid(*((IB_GID*)mcgs.mGid)))) {
                while (mcGroup->mcMembers) {
                    /* 
                     * MUST copy head pointer into temp
                     * Passing mcGroup->members directly to the delete macro will corrupt the list
                     */
                    mcMember = mcGroup->mcMembers;
                    McMember_Delete(mcGroup, mcMember);
                }
                McGroup_Delete(mcGroup);
            }
            /* create the group with members */
            if ((status = sm_multicast_sync_lid(*((IB_GID*)mcgs.mGid), mcgs.pKey, mcgs.mtu, mcgs.rate, mcgs.mLid)) == VSTATUS_OK) {
                McGroup_Create(mcGroup);
                memcpy(&(mcGroup->mGid), &(mcgs.mGid), sizeof(IB_GID));
                mcGroup->members_full = mcgs.members_full;
                mcGroup->qKey = mcgs.qKey;
                mcGroup->pKey = mcgs.pKey;
                mcGroup->mLid = mcgs.mLid;
                mcGroup->mtu = mcgs.mtu;
                mcGroup->rate = mcgs.rate;
                mcGroup->life = mcgs.life;
                mcGroup->sl = mcgs.sl;
                mcGroup->flowLabel = mcgs.flowLabel;
                mcGroup->tClass = mcgs.tClass;
                mcGroup->hopLimit = mcgs.hopLimit;
                mcGroup->scope = mcgs.scope;
                mcGroup->index_pool = mcgs.index_pool;
                /* add members to group */
                while (bufidx < reclen && memcnt < mcgs.membercount) {
        		    BSWAPCOPY_STL_MCMEMBER_SYNCDB((STL_MCMEMBER_SYNCDB*)&msgbuf[bufidx], &mcms);
                    bufidx += sizeof(STL_MCMEMBER_SYNCDB);
                    McMember_Create(mcGroup, mcMember);
                    mcMember->slid = mcms.slid;
                    mcMember->proxy = mcms.proxy;
                    mcMember->state = mcms.state;
                    mcMember->nodeGuid = mcms.nodeGuid;
                    mcMember->portGuid = mcms.portGuid;
                    mcMember->record = mcms.member;
                    mcMember->index = mcms.index;
                }
                (void)vs_rdlock(&old_topology_lock);
                VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;
                if (VirtualFabrics) {
                    vfmGid[0] = mcGroup->mGid.AsReg64s.H;
                    vfmGid[1] = mcGroup->mGid.AsReg64s.L;
                    for (vf=0; vf < VirtualFabrics->number_of_vfs; vf++) {
                        if ((PKEY_VALUE(VirtualFabrics->v_fabric[vf].pkey) == PKEY_VALUE(mcGroup->pKey)) &&
                            (smVFValidateMcDefaultGroup(vf, vfmGid) == VSTATUS_OK)) {
                            uint32 vfIdx=VirtualFabrics->v_fabric[vf].index;
                            bitset_set(&mcGroup->vfMembers, vfIdx);
                        }
                    }
                }
                (void)vs_rwunlock(&old_topology_lock);

                IB_LOG_INFO_FMT(__func__,
                       "ADD/UPDATE group " FMT_GID " with %d members", ((IB_GID *)mcgs.mGid)->Type.Global.SubnetPrefix, ((IB_GID *)mcgs.mGid)->Type.Global.InterfaceID, mcgs.membercount);
            } else {
                IB_LOG_WARN_FMT(__func__,
                       "sm_multicast_sync_lid failed, Can't ADD/UPDATE group " FMT_GID " with %d members", ((IB_GID *)mcgs.mGid)->Type.Global.SubnetPrefix, ((IB_GID*)mcgs.mGid)->Type.Global.InterfaceID, mcgs.membercount);
                /* can't create the group on this side, drop the sync packet */
                (void)vs_unlock(&sm_McGroups_lock);
                (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
                IB_EXIT(__func__, status);
                return status;
            }
            (void)vs_unlock(&sm_McGroups_lock);
        }
        /* return appropriate status to caller */
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, ((status == VSTATUS_OK) ? VSTATUS_OK : VSTATUS_DROP));
    } else if (maip->base.amod == DBSYNC_AMOD_DELETE) {
        /* DELETE: remove record from dataset */
        while (grpcnt-- && (bufidx < reclen)) {
            (void)BSWAPCOPY_SM_DBSYNC_MC_GROUP_DATA((McGroupSync_t *)&msgbuf[bufidx], &mcgs);
            bufidx += sizeof(McGroupSync_t);
            /* delete group if exist */
            (void)vs_lock(&sm_McGroups_lock);
    		if ((mcGroup = sm_find_multicast_gid(*((IB_GID *)mcgs.mGid)))) {
                while (mcGroup->mcMembers) {
                    /* 
                     * MUST copy head pointer into temp
                     * Passing mcGroup->members directly to the delete macro will corrupt the list
                     */
                    mcMember = mcGroup->mcMembers;
                    McMember_Delete(mcGroup, mcMember);
                }
                McGroup_Delete(mcGroup);
                IB_LOG_INFO_FMT(__func__,
                       "deleted group " FMT_GID " record", ((IB_GID *)mcgs.mGid)->Type.Global.SubnetPrefix, ((IB_GID *)mcgs.mGid)->Type.Global.InterfaceID);
            }
            (void)vs_unlock(&sm_McGroups_lock);
        }
        /* return appropriate status to caller */
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, ((status == VSTATUS_OK) ? VSTATUS_OK : VSTATUS_DROP));
    } else {
        IB_LOG_WARN_FMT(__func__,
               "Invalid amod [%d] received; should be 2=SET, 3=ADD, or 4=DELETE", maip->base.amod);
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
    }

    IB_EXIT(__func__, status);
    return status;
}


/*
 * sync SERVICE records with a standby SM
*/
static Status_t dbsync_setSMDBService(SMSyncReq_t *syncReqp) {
    Status_t        status=VSTATUS_OK;
    uint8_t *       buff;
    uint32_t        numRecs=0, outlen=0;
    uint32_t        resp_status=0;
	OpaServiceRecord_t	*osrp;
    ServiceRecKeyp  srkeyp;
    CS_HashTableItr_t itr;

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
        /*
         *	create output buffer from hashtable records
         */
        if (vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) return VSTATUS_BAD;
        if ((numRecs = cs_hashtable_count(saServiceRecords.serviceRecMap)) > 0)
        {
            outlen = sizeof(STL_SERVICE_RECORD) * numRecs;
			outlen += sizeof (numRecs);
            /* make sure data will fit in output buffer */
            if (outlen > buflen) {
                IB_LOG_ERROR("size of service records sync buffer too big! Increase subnet size. message size:", outlen);
                status = VSTATUS_NOMEM;
                (void)vs_unlock(&saServiceRecords.serviceRecLock);
                IB_EXIT(__func__, status);
                return status;
            }
            buff = msgbuf;
            BSWAPCOPY_SM_DBSYNC_RECORD_CNT(&numRecs, (uint32_t*)buff);
            buff += sizeof(numRecs);
            cs_hashtable_iterator(saServiceRecords.serviceRecMap, &itr);
            do {
                srkeyp = cs_hashtable_iterator_key(&itr);
				// Skip SM service records
				if (srkeyp->serviceId == SM_SERVICE_ID) {
					numRecs -= 1;
					outlen -= sizeof(STL_SERVICE_RECORD);
					continue;
				}
                osrp = cs_hashtable_iterator_value(&itr);
            	BSWAPCOPY_STL_SERVICE_RECORD(&osrp->serviceRecord, 
					(STL_SERVICE_RECORD*)buff);
                buff += sizeof(STL_SERVICE_RECORD);
            } while (cs_hashtable_iterator_advance(&itr));
        }
        (void)vs_unlock(&saServiceRecords.serviceRecLock);

        /* send the records to the standby SM */
        if (numRecs) {
            if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
                IB_LOG_ERROR("can't set destination lid on management info handle=", dbsyncfd_if3);
            } else {
                if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_SERVICE, DBSYNC_AMOD_SET, 
                                           msgbuf, outlen, msgbuf, &outlen, &resp_status)) != VSTATUS_OK) {
                    IB_LOG_INFO_FMT(__func__, 
                            "DBSYNC SET of SERVICE records with SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                            syncReqp->portguid, syncReqp->standbyLid, status);
                } else if (resp_status != 0) {
                    status=VSTATUS_BAD;
                    IB_LOG_WARN_FMT(__func__, 
                            "Full sync of SERVICE records with SM at portGuid "FMT_U64", LID=[0x%x] returned mad status 0x%x",
                            syncReqp->portguid, syncReqp->standbyLid, resp_status);
                } else {
                    /* successfully sync'd all INFORM records to standby SM */
                    IB_LOG_INFO_FMT(__func__, 
                            "Full sync of %d SERVICE records with SM at portGuid "FMT_U64", LID=[0x%x] was successfull",
                            numRecs, syncReqp->portguid, syncReqp->standbyLid);
                }
            }
        } else {
            IB_LOG_INFO_FMT(__func__, 
                    "NO SERVICE records to sync to SM at portGuid "FMT_U64", LID=[0x%x]",
                    syncReqp->portguid, syncReqp->standbyLid);
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM", 
                syncReqp->portguid, syncReqp->standbyLid);
    }

    IB_EXIT(__func__, 0);
    return status;
}


/*
 * sync an creation, deletion, or update of a SERVICE record to a standby SM
*/
static Status_t dbsync_updateSMDBService(SMSyncReq_t *syncReqp) {
    Status_t        status=VSTATUS_OK;
    uint32_t        resp_status=0;
	OpaServiceRecord_t	*osrp=NULL;
    uint32_t        numRecs=0, outlen=0, inlen=0;
    uint8_t        *buff = msgbuf;

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
        numRecs++;
        osrp = (OpaServiceRecord_t *)(syncReqp->data);
        BSWAPCOPY_SM_DBSYNC_RECORD_CNT(&numRecs, (uint32_t*)buff);
        buff += sizeof(numRecs);
		BSWAPCOPY_STL_SERVICE_RECORD(&osrp->serviceRecord, (STL_SERVICE_RECORD*)buff);
        outlen = sizeof(STL_SERVICE_RECORD);
		outlen += sizeof(numRecs);
        if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
            IB_LOG_ERROR("can't set destination lid on management info handle=", dbsyncfd_if3);
        } else {
            if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_SERVICE, syncReqp->type, 
                                       msgbuf, outlen, msgbuf, &inlen, &resp_status)) != VSTATUS_OK) {
                IB_LOG_INFO_FMT(__func__, 
                        "%s of SERVICE record to SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                        ((syncReqp->type == DBSYNC_TYPE_UPDATE) ? "UPDATE":"DELETE"), syncReqp->portguid, syncReqp->standbyLid, status);
            } else if (resp_status != 0) {
                status=VSTATUS_BAD;
                IB_LOG_WARN_FMT(__func__, 
                        "%s of SERVICE record to SM at portGuid "FMT_U64", LID=[0x%x] returned mad status 0x%x",
                        ((syncReqp->type == DBSYNC_TYPE_UPDATE) ? "UPDATE":"DELETE"), syncReqp->portguid, syncReqp->standbyLid, resp_status);
            } else {
                /* successfully sync'd all INFORM records to standby SM */
                IB_LOG_INFO_FMT(__func__, 
                        "%s of SERVICE record to SM at portGuid "FMT_U64", LID=[0x%x] was successfull",
                        ((syncReqp->type == DBSYNC_TYPE_UPDATE) ? "UPDATE":"DELETE"), syncReqp->portguid, syncReqp->standbyLid);
            }
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM", 
                syncReqp->portguid, syncReqp->standbyLid);
    }

    IB_EXIT(__func__, status);
    return status;
}


/*
 * STANDBY: process request for FULL sync of SERVICE records
*/
static Status_t processServiceSync(Mai_t *maip, uint8_t *msgbuf, uint32_t reclen) {
    Status_t        status=VSTATUS_OK;
    uint32_t        bufidx=0, numRecs;
	OpaServiceRecord_t	osr = {.pkeyDefined = 0, .expireTime = 0l};
	OpaServiceRecord_t	*osrp;
    ServiceRecKeyp  srkeyp;
    ServiceRecKey_t srkey = { .serviceId=0, .servicep_key=0, .serviceGid={.AsReg64s.H=0, .AsReg64s.L=0 }} ;
    uint64_t        now;

    IB_ENTER(__func__, maip, 0, 0, 0);

    if (reclen > buflen) {
        IB_LOG_ERROR_FMT(__func__, 
                "Received length (%d bytes) greater than allocated (%d bytes), PLEASE INCREASE THE FABRIC SUPPORTED END PORT COUNT", 
                SMDBSYNC_NSIZE, buflen);
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
        IB_EXIT(__func__, VSTATUS_NXIO);
        return VSTATUS_NXIO;
    }
    IB_LOG_INFO_FMT(__func__,
		"Processing sync of SERVICE records with (%d) bytes of data (%d records)\n",
		reclen, (int)(reclen/sizeof(STL_SERVICE_RECORD)));
    BSWAPCOPY_SM_DBSYNC_RECORD_CNT((uint32_t*)&msgbuf[bufidx],&numRecs);
    bufidx += sizeof(numRecs);
    if (maip->base.amod == DBSYNC_AMOD_SET) {
        /* clear out the SERVICE table */
        (void) sa_ServiceRecClear();
        while (numRecs-- && (bufidx < reclen)) {
            BSWAPCOPY_STL_SERVICE_RECORD((STL_SERVICE_RECORD*)&msgbuf[bufidx], 
				&osr.serviceRecord);
            bufidx += sizeof(STL_SERVICE_RECORD);
            /* calculate the expiration time for the record */
            (void)vs_time_get(&now);
            if (osr.serviceRecord.ServiceLease != 0xffffffff) {
                osr.expireTime = now + (1000000 * (uint64_t)osr.serviceRecord.ServiceLease);
            } else {
                osr.expireTime = VTIMER_ETERNITY;
            }
            /* allocate service key storage */
            if ((srkeyp = (ServiceRecKeyp) malloc(sizeof(ServiceRecKey_t))) == NULL) {
                status = VSTATUS_NOMEM;
                IB_LOG_ERROR_FMT(__func__, "Can't allocate SERVICE key for SET");
                break;
            } else {
                /* fill in the key data */
                memcpy(&srkeyp->serviceGid, &osr.serviceRecord.RID.ServiceGID, sizeof(IB_GID));
                srkeyp->serviceId = osr.serviceRecord.RID.ServiceID;
                srkeyp->servicep_key = osr.serviceRecord.RID.ServiceP_Key;
                /* allocate vieo service iRecord for adding to hash table */
                if ((osrp = (OpaServiceRecord_t *) malloc(sizeof(OpaServiceRecord_t))) == NULL) {
                    free(srkeyp);
                    IB_LOG_ERROR_FMT(__func__, "Can't allocate SERVICE entry for SET");
                    status = VSTATUS_NOMEM;
                    break;
                }
                /* fill in the allocated record for hash table insertion */
                memcpy((void *)osrp, (void *)&osr, sizeof(OpaServiceRecord_t));
                if (vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) {
                    free(srkeyp);
                    free(osrp);
                    IB_LOG_ERROR_FMT(__func__, "Can't lock SERVICE table");
                    status = VSTATUS_NOMEM;
                    break;
                }
                if (!cs_hashtable_insert(saServiceRecords.serviceRecMap, srkeyp, osrp)) {
                    free(srkeyp);
                    free(osrp);
                    (void)vs_unlock(&saServiceRecords.serviceRecLock);
                    IB_LOG_ERROR_FMT(__func__, 
                           "Failed to insert sync'd %s, serviceID="FMT_U64" with gid " FMT_GID " in SERVICE table (SET)",
                           osr.serviceRecord.ServiceName, osr.serviceRecord.RID.ServiceID, STLGIDPRINTARGS(osr.serviceRecord.RID.ServiceGID));
                    status = VSTATUS_BAD;
                    break;
                } else {
                    IB_LOG_INFO_FMT(__func__, 
                           "ADDED (SET) %s, serviceID="FMT_U64" with gid " FMT_GID ", p_key 0x%.8X to SERVICE table",
                           osrp->serviceRecord.ServiceName, srkey.serviceId, STLGIDPRINTARGS(srkey.serviceGid), srkey.servicep_key);
                    status = VSTATUS_OK;
                    (void)vs_unlock(&saServiceRecords.serviceRecLock);
                }
            }
        }
        /* return appropriate status to caller */
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, ((status == VSTATUS_OK) ? VSTATUS_OK : VSTATUS_DROP));
    } else if (maip->base.amod == DBSYNC_AMOD_ADD) {
        /* ADD: add/update entry in dataset */
        while (numRecs-- &&(bufidx < reclen)) {
            BSWAPCOPY_STL_SERVICE_RECORD((STL_SERVICE_RECORD*)&msgbuf[bufidx], 
				&osr.serviceRecord);
            bufidx += sizeof(STL_SERVICE_RECORD);
            /* calculate the expiration time for the record */
            (void)vs_time_get(&now);
            if (osr.serviceRecord.ServiceLease != 0xffffffff) {
                osr.expireTime = now + (1000000 * (uint64_t)osr.serviceRecord.ServiceLease);
            } else {
                osr.expireTime = VTIMER_ETERNITY;
            }
            if (vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) {
                IB_LOG_ERROR_FMT(__func__, "Can't lock SERVICE table");
                status = VSTATUS_BAD;
                break;
            }
            /* fill in the key data */
            memcpy(&srkey.serviceGid, &osr.serviceRecord.RID.ServiceGID, sizeof(IB_GID));
            srkey.serviceId = osr.serviceRecord.RID.ServiceID;
            srkey.servicep_key = osr.serviceRecord.RID.ServiceP_Key;
            /* replace existing entry or create a new one */
            if (NULL == (osrp = (OpaServiceRecord_t *)cs_hashtable_search(saServiceRecords.serviceRecMap, &srkey))) {
                /* allocate a service key and osr for adding to hash table */
                if ((srkeyp = (ServiceRecKeyp) malloc(sizeof(ServiceRecKey_t))) == NULL) {
                    (void)vs_unlock(&saServiceRecords.serviceRecLock);
                    status = VSTATUS_NOMEM;
                    IB_LOG_ERROR_FMT(__func__, "Can't allocate SERVICE key for ADD");
                    break;
                } else {
                    /* fill in the key data */
                    memcpy((void *)srkeyp, (void *)&srkey, sizeof(ServiceRecKey_t));
                    /* allocate vieo service for adding to hash table */
                    if ((osrp = (OpaServiceRecord_t *) malloc(sizeof(OpaServiceRecord_t))) == NULL) {
                        free(srkeyp);
                        (void)vs_unlock(&saServiceRecords.serviceRecLock);
                        status = VSTATUS_NOMEM;
                        IB_LOG_ERROR_FMT(__func__, "Can't allocate SERVICE record for ADD");
                        break;
                    } else {
                        memcpy((void *)osrp, (void *)&osr, sizeof(OpaServiceRecord_t));
                        if (!cs_hashtable_insert(saServiceRecords.serviceRecMap, srkeyp, osrp)) {
                            free(srkeyp);
                            free(osrp);
                            (void)vs_unlock(&saServiceRecords.serviceRecLock);
                            status = VSTATUS_BAD;
                            IB_LOG_ERROR_FMT(__func__, 
                                   "Failed to ADD sync'd serviceID="FMT_U64", gid "FMT_GID " to service table", 
                                   osr.serviceRecord.RID.ServiceID, STLGIDPRINTARGS(osr.serviceRecord.RID.ServiceGID));
                            break;
                        } else {
                            IB_LOG_INFO_FMT(__func__, 
                                   "ADDED %s, serviceID="FMT_U64", gid " FMT_GID " to service table",
                                   osr.serviceRecord.ServiceName ,osr.serviceRecord.RID.ServiceID, STLGIDPRINTARGS(osr.serviceRecord.RID.ServiceGID));
                            status = VSTATUS_OK;
                            (void)vs_unlock(&saServiceRecords.serviceRecLock);
                        }
                    }
                }
            } else {
                /* replace existing record contents */
                osrp->expireTime = osr.expireTime;
                osrp->serviceRecord = osr.serviceRecord;
                IB_LOG_INFO_FMT(__func__, 
                       "UPDATED (ADD) serviceID="FMT_U64", gid " FMT_GID " in service table",
                       osr.serviceRecord.RID.ServiceID, STLGIDPRINTARGS(osr.serviceRecord.RID.ServiceGID));
                status = VSTATUS_OK;
                (void)vs_unlock(&saServiceRecords.serviceRecLock);
            }
        }
        /* return appropriate status to caller */
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, ((status == VSTATUS_OK) ? VSTATUS_OK : VSTATUS_DROP));
    } else if (maip->base.amod == DBSYNC_AMOD_DELETE) {
        /* DELETE: remove record from dataset */
        while (numRecs-- && (bufidx < reclen)) {
            BSWAPCOPY_STL_SERVICE_RECORD((STL_SERVICE_RECORD*)&msgbuf[bufidx], 
				&osr.serviceRecord);
            bufidx += sizeof(STL_SERVICE_RECORD);
            memcpy(&srkey.serviceGid, &osr.serviceRecord.RID.ServiceGID, sizeof(IB_GID));
            srkey.serviceId = osr.serviceRecord.RID.ServiceID;
            srkey.servicep_key = osr.serviceRecord.RID.ServiceP_Key;
            if (vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) {
                IB_LOG_ERROR_FMT(__func__, "Can't lock SERVICE table");
                status = VSTATUS_BAD;
                break;
            } else {
                if (NULL == (osrp = (OpaServiceRecord_t *)cs_hashtable_remove(saServiceRecords.serviceRecMap, &srkey))) {
                    IB_LOG_INFO_FMT(__func__, 
                           "Could not find serviceID="FMT_U64", gid " FMT_GID " in service table",
                           srkey.serviceId, STLGIDPRINTARGS(srkey.serviceGid));
                } else {
                    /* free the actual serviceRecord - remove only frees the key */
                    free(osrp);
                    IB_LOG_INFO_FMT(__func__, 
						"Free'd %s, serviceID="FMT_U64", gid " FMT_GID ", inservice table",
						osr.serviceRecord.ServiceName, srkey.serviceId, 
						STLGIDPRINTARGS(srkey.serviceGid));
                }
                (void)vs_unlock(&saServiceRecords.serviceRecLock);
            }
        }
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_OK);
    } else {
        IB_LOG_WARN_FMT(__func__,
               "Invalid SERVICE amod [%d] received; should be 2=SET, 3=ADD, or 4=DELETE", maip->base.amod);
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
    }

    IB_EXIT(__func__, status);
    return status;
}

/*
 * sync multicast root with a standby SM
*/
static Status_t dbsync_setSMDBMCRoot(SMSyncReq_t *syncReqp) {
    Status_t        status=VSTATUS_OK;
    uint8_t *       buff;
    uint32_t        outlen=0;
    uint32_t        resp_status=0;
	uint64_t		mcroot_guid;

    IB_ENTER(__func__, syncReqp, 0, 0, 0);

    if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
		if (vs_lock(&sm_mcSpanningTreeRootGuidLock) != VSTATUS_OK) return VSTATUS_BAD;
		outlen = sizeof (sm_mcSpanningTreeRootGuid);

		  /* make sure data will fit in output buffer */
        if (outlen > buflen) {
            IB_LOG_ERROR("size of MC Root Guid buffer too big! Increase subnet size. message size:", outlen);
            status = VSTATUS_NOMEM;
            (void)vs_unlock(&sm_mcSpanningTreeRootGuidLock);
            IB_EXIT(__func__, status);
            return status;
        }
        buff = msgbuf;
		mcroot_guid = hton64(sm_mcSpanningTreeRootGuid);
		memcpy(buff, &mcroot_guid, sizeof(mcroot_guid));
        (void)vs_unlock(&sm_mcSpanningTreeRootGuidLock);

        /* send the record to the standby SM */
        if (if3_set_dlid (dbsyncfd_if3, syncReqp->standbyLid)) {
            IB_LOG_ERROR("can't set destination lid on management info handle=", dbsyncfd_if3);
        } else {
            if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_MCROOT, DBSYNC_AMOD_SET, 
                                       msgbuf, outlen, msgbuf, &outlen, &resp_status)) != VSTATUS_OK) {
                IB_LOG_INFO_FMT(__func__, 
                        "DBSYNC SET of MC ROOT with SM at portGuid "FMT_U64", LID=[0x%x] Failed (rc=%d)",
                        syncReqp->portguid, syncReqp->standbyLid, status);
            } else if (resp_status != 0) {
                status=VSTATUS_BAD;
                IB_LOG_WARN_FMT(__func__, 
                        "Full sync of MC ROOT with SM at portGuid "FMT_U64", LID=[0x%x] returned mad status 0x%x",
                        syncReqp->portguid, syncReqp->standbyLid, resp_status);
            } else {
                /* successfully sync'd to standby SM */
                IB_LOG_INFO_FMT(__func__, 
                        "Full sync of MC ROOT GUID "FMT_U64" with SM at portGuid "FMT_U64", LID=[0x%x] was successfull",
                        sm_mcSpanningTreeRootGuid, syncReqp->portguid, syncReqp->standbyLid);
            }
        }
    } else {
        status = VSTATUS_BAD;
        IB_LOG_ERROR_FMT(__func__, 
                "NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM", 
                syncReqp->portguid, syncReqp->standbyLid);
    }

    IB_EXIT(__func__, 0);
    return status;
}

static Status_t processMCRootSync(Mai_t *maip, uint8_t *msgbuf, uint32_t reclen) {
    Status_t        status=VSTATUS_OK;
    uint64_t		mcroot_guid;

    IB_ENTER(__func__, maip, 0, 0, 0);

	if (reclen > 0 && ((reclen % sizeof(uint64_t)) != 0)) {
        IB_LOG_ERROR_FMT(__func__,
			"Expecting %"PRISZT" bytes of data, received %d bytes instead",
			sizeof(uint64_t), reclen);
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
        IB_EXIT(__func__, VSTATUS_NXIO);
        return VSTATUS_NXIO;
    } else if (reclen > buflen) {
        IB_LOG_ERROR_FMT(__func__, 
                "Received length (%d bytes) greater than allocated (%d bytes), PLEASE INCREASE THE FABRIC SUPPORTED END PORT COUNT", 
                SMDBSYNC_NSIZE, buflen);
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
        IB_EXIT(__func__, VSTATUS_NXIO);
        return VSTATUS_NXIO;
    }
    IB_LOG_INFO_FMT(__func__, "Processing sync of MC ROOT with (%d) bytes\n", reclen);

    if (maip->base.amod == DBSYNC_AMOD_SET) {
		if (vs_lock(&sm_mcSpanningTreeRootGuidLock) != VSTATUS_OK) return VSTATUS_BAD;
		memcpy(&mcroot_guid, msgbuf, sizeof (mcroot_guid));
		sm_mcSpanningTreeRootGuid = ntoh64(mcroot_guid);
		if (smDebugPerf) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Standby rcvd MC ROOT guid as "FMT_U64, sm_mcSpanningTreeRootGuid);
		}
		vs_unlock(&sm_mcSpanningTreeRootGuidLock);
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_OK);
   	} else {
        IB_LOG_WARN_FMT(__func__,
               "Invalid MC Root amod [%d] received; should be 2=SET", maip->base.amod);
        (void) dbSyncMngrReply (dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
    }

	return status;
}

/*
 * sync dateline switch GUID with a standby SM
 */
static Status_t dbsync_setSMDBDatelineSwitchGUID(SMSyncReq_t *syncReqp) {
	Status_t	status=VSTATUS_OK;
	uint8_t *	buff;
	uint32_t	outlen=0;
	uint32_t	resp_status=0;
	uint64_t	datelineGUID;

	IB_ENTER(__func__, syncReqp, 0, 0, 0);

	if (syncReqp->portguid && (syncReqp->standbyLid >= UNICAST_LID_MIN && syncReqp->standbyLid <= UNICAST_LID_MAX)) {
		if (vs_lock(&sm_datelineSwitchGUIDLock) != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__, "Failed to get lock for Dateline Switch GUID");
			return VSTATUS_BAD;
		}
		outlen = sizeof(sm_datelineSwitchGUID);

		/* make sure data will fit in output buffer */
		if (outlen > buflen) {
			IB_LOG_ERROR("size of Dateline Switch GUID too big! Increase subnet size. message size:", outlen);
			status = VSTATUS_NOMEM;
			(void)vs_unlock(&sm_datelineSwitchGUIDLock);
			IB_EXIT(__func__, status);
			return status;
		}
		buff = msgbuf;
		datelineGUID = hton64(sm_datelineSwitchGUID);
		memcpy(buff, &datelineGUID, sizeof(datelineGUID));
		(void)vs_unlock(&sm_datelineSwitchGUIDLock);

		/* send the record to the standby SM */
		if (if3_set_dlid(dbsyncfd_if3, syncReqp->standbyLid)) {
			IB_LOG_ERROR("can't set destination lid on management info handle=", dbsyncfd_if3);
		} else {
			if ((status = dbSyncCmdToMgr(dbsyncfd_if3, DBSYNC_AID_DATELINE, DBSYNC_AMOD_SET,
					msgbuf, outlen, msgbuf, &outlen, &resp_status)) != VSTATUS_OK) {
				IB_LOG_WARN_FMT(__func__,
					"DBSYNC SET of Dateline Switch GUID with SM at portGUID "FMT_U64", LID=[0x%x] Failed (rc=%d)",
					syncReqp->portguid, syncReqp->standbyLid, status);
			} else if (resp_status != 0) {
				status=VSTATUS_BAD;
				IB_LOG_WARN_FMT(__func__,
					"Full sync of Dateline Switch GUID with SM at portGUID "FMT_U64", LID=[0x%x] returned mad status 0x%x",
					syncReqp->portguid, syncReqp->standbyLid, resp_status);
			} else {
				/* successfully sync'd to standby SM */
				IB_LOG_INFO_FMT(__func__,
					"Full sync of Dateline Switch GUID "FMT_U64" with SM at portGuid "FMT_U64", LID=[0x%x] was successful",
					sm_datelineSwitchGUID, syncReqp->portguid, syncReqp->standbyLid);
			}
		}
	} else {
		status = VSTATUS_BAD;
		IB_LOG_ERROR_FMT(__func__,
			"NULL portGuid "FMT_U64" or bad LID=[0x%x] for standby SM",
			syncReqp->portguid, syncReqp->standbyLid);
	}
	IB_EXIT(__func__, 0);
	return status;
}

static Status_t processDatelineSwitchGUIDSync(Mai_t *maip, uint8_t *msgbuf, uint32_t reclen) {
	Status_t	status=VSTATUS_OK;
	uint64_t	datelineGUID;

	IB_ENTER(__func__, maip, 0, 0, 0);

	if (reclen > 0 && ((reclen %sizeof(uint64_t)) != 0)) {
		IB_LOG_ERROR_FMT(__func__,
			"Expecting %"PRISZT" bytes of data, received %d bytes instead",
			sizeof(uint64_t), reclen);
		(void) dbSyncMngrReply(dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
		IB_EXIT(__func__, VSTATUS_NXIO);
		return VSTATUS_NXIO;
	} else if (reclen > buflen) {
		IB_LOG_ERROR_FMT(__func__,
			"Received length (%d bytes) greater than allocated (%d bytes), PLEASE INCREASE THE FABRIC SUPPORTED END PORT COUNT",
			SMDBSYNC_NSIZE, buflen);
		(void)dbSyncMngrReply(dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
		IB_EXIT(__func__, VSTATUS_NXIO);
		return VSTATUS_NXIO;
	}
	IB_LOG_INFO_FMT(__func__, "Processing sync of Dateline Switch GUID with (%d) bytes\n", reclen);

	if (maip->base.amod == DBSYNC_AMOD_SET) {
		if (vs_lock(&sm_datelineSwitchGUIDLock) != VSTATUS_OK) return VSTATUS_BAD;
		memcpy(&datelineGUID, msgbuf, sizeof(datelineGUID));
		sm_datelineSwitchGUID = ntoh64(datelineGUID);
		if (smDebugPerf) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Standby rcvd Dateline Switch GUID as "FMT_U64, sm_datelineSwitchGUID);
		}
		vs_unlock(&sm_datelineSwitchGUIDLock);
		(void) dbSyncMngrReply(dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_OK);
	} else {
		IB_LOG_WARN_FMT(__func__,
			"Invalid Dateline Switch GUID amod [%d] received; should be 2=SET", maip->base.amod);
		(void) dbSyncMngrReply(dbsyncfd_if3, maip, msgbuf, 0, VSTATUS_DROP);
	}
	return status;
		
}

/*
 * process db sync requests from the SM, PM, and SA
 */
static void dbsync_procReqQ(void) {
    SMSyncReqp      syncReqp = NULL;
    SMDBSync_t      dbsync = {0};         /* sync state of an SM */
    Status_t        status = VSTATUS_OK;

    IB_ENTER(__func__, 0, 0, 0, 0);

    while ((syncReqp = (SMSyncReqp)cs_queue_FrontAndDequeue( sm_dbsync_queue )) != NULL) {
        /* don't process anything if sync is off */
        if (!sm_config.db_sync_interval) {
            /* just free the syncReq space and continue */
        } else if (syncReqp->type == DBSYNC_TYPE_GET_CAPABILITY) {
			status = dbsync_getSMDBSync(syncReqp);
			if (status == VSTATUS_OK) {
				if (smDebugPerf) {
					IB_LOG_INFINI_INFO_FMT(__func__,
						"Successful getting sync capability of SM at portGuid "FMT_U64", LID=[0x%x]",
						syncReqp->portguid, syncReqp->standbyLid);
				}

            	/* get the sync Config Consistency Check info of the standby SM if configured to do so */

				if (dbsync_getSMDBCCCSync(syncReqp)) {
                   	IB_LOG_INFINI_INFO_FMT(__func__,
                        	"failed to get sync Config Consistency Check info of SM at portGuid "FMT_U64", LID=[0x%x]", 
                        	syncReqp->portguid, syncReqp->standbyLid);
				} else {
					if (smDebugPerf) {
						IB_LOG_INFINI_INFO_FMT(__func__,
							"Successful getting sync Config Consistency Check info of SM at portGuid "FMT_U64", LID=[0x%x]",
							syncReqp->portguid, syncReqp->standbyLid);
					}
				}
			} else if (status != VSTATUS_AGAIN) {

				/* Never received the DBSync Capability Response from Standby SM
				 * Removing remote SM from list, triggering resweep which will
				 * find the standby SM the second time and request capability again
				 */

                IB_LOG_INFINI_INFO_FMT(__func__, 
                        "failed to get sync capability of SM at portGuid "FMT_U64", LID=[0x%x]", 
                        syncReqp->portguid, syncReqp->standbyLid);

                if (sm_dbsync_upSmDbsyncCap(syncReqp->portguid, DBSYNC_CAP_NOTSUPPORTED)) {
                    IB_LOG_WARN_FMT(__func__,
                           "Can't update sync capability of SM at portGuid="FMT_U64" in SM list", syncReqp->portguid);
                }

				IB_LOG_WARN_FMT(__func__,
						"Removing standby SM at portGuid "FMT_U64" from SM list and triggering resweep; failed to get sync capability",
						syncReqp->portguid);

				sm_dbsync_deleteSm(syncReqp->portguid);

				sm_trigger_sweep(SM_SWEEP_REASON_SECONDARY_TROUBLE);
			}
		} else if (sm_dbsync_getDbsyncSupport(syncReqp->portguid) != DBSYNC_CAP_SUPPORTED) {
            /* just free the syncReq space and continue */
        } else if (syncReqp->type == DBSYNC_TYPE_FULL) {
            /* full sync request - all data types */
            switch (syncReqp->datatype) {
            case DBSYNC_DATATYPE_ALL:
                if ((status = dbsync_setSMDBInform(syncReqp))) {
                    IB_LOG_WARN_FMT(__func__, 
                            "failed to sync (full) INFORM records with SM at portGuid "FMT_U64", LID=[0x%x]", 
                            syncReqp->portguid, syncReqp->standbyLid);
                    /* update the sync status of this SM node */
                    (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_INFORM, DBSYNC_STAT_FAILURE);
                } else {
                    (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_INFORM, DBSYNC_STAT_SYNCHRONIZED);
                    if ((status = dbsync_setSMDBGroup(syncReqp))) {
                        IB_LOG_WARN_FMT(__func__, 
                                "failed to sync (full) GROUP records with SM at portGuid "FMT_U64", LID=[0x%x]", 
                                syncReqp->portguid, syncReqp->standbyLid);
                        /* update the sync status of this SM node */
                        (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_GROUP, DBSYNC_STAT_FAILURE);
                    } else {
                        (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_GROUP, DBSYNC_STAT_SYNCHRONIZED);
                        if ((status = dbsync_setSMDBService(syncReqp))) {
                            IB_LOG_WARN_FMT(__func__, 
                                    "failed to sync (full) SERVICE records with SM at portGuid "FMT_U64", LID=[0x%x]", 
                                    syncReqp->portguid, syncReqp->standbyLid);
                            /* update the sync status of this SM node */
                            (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_SERVICE, DBSYNC_STAT_FAILURE);
                        } else {
                            (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_SERVICE, DBSYNC_STAT_SYNCHRONIZED);
                    	    if ((status = dbsync_setSMDBMCRoot(syncReqp))) {
            	                IB_LOG_WARN_FMT(__func__, 
        	                            "failed to sync (full) MC Root records with SM at portGuid "FMT_U64", LID=[0x%x]", 
                	                    syncReqp->portguid, syncReqp->standbyLid);
    	                        /* update the sync status of this SM node */
	                            (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_MCROOT, DBSYNC_STAT_FAILURE);
							} else {
                            	(void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_MCROOT, DBSYNC_STAT_SYNCHRONIZED);
				if ((status = dbsync_setSMDBDatelineSwitchGUID(syncReqp))) {
					IB_LOG_WARN_FMT(__func__,
						"failed to sync (full) Dateline Switch GUID with SM at portGuid "FMT_U64", LID=[0x%x]",
						syncReqp->portguid, syncReqp->standbyLid);
					/* update the sync status of this SM node */
						(void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_DATELINE_GUID, DBSYNC_STAT_FAILURE);
							} else {
						(void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_DATELINE_GUID, DBSYNC_STAT_SYNCHRONIZED);
							}
				}
                        }
                    }
                }
                /* update the full sync status of this SM node */
                (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_ALL, 
                                                (status) ? DBSYNC_STAT_FAILURE:DBSYNC_STAT_SYNCHRONIZED);
                if (status == VSTATUS_OK) {
                    /* update the sync status of the remote SM */
                    if (dbsync_setSMDBSync(syncReqp)) {
                        IB_LOG_WARN_FMT(__func__, 
                                "failed to SET current sync capability of SM at portGuid "FMT_U64", LID=[0x%x]", 
                                syncReqp->portguid, syncReqp->standbyLid);
                    }
                }
                break;
            case DBSYNC_DATATYPE_INFORM:
                if ((status = dbsync_setSMDBInform(syncReqp))) {
                    IB_LOG_WARN_FMT(__func__, 
                            "failed to sync (full) INFORM records with SM at portGuid "FMT_U64", LID=[0x%x]", 
                            syncReqp->portguid, syncReqp->standbyLid);
                }
                /* update the sync status of this SM node */
                (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_INFORM, 
                                                (status != VSTATUS_OK) ? DBSYNC_STAT_FAILURE:DBSYNC_STAT_SYNCHRONIZED);
                if (status == VSTATUS_OK) {
                    /* update the sync status of the remote SM */
                    if (dbsync_setSMDBSync(syncReqp)) {
                        IB_LOG_WARN_FMT(__func__, 
                                "failed to SET current sync capability of SM at portGuid "FMT_U64", LID=[0x%x]", 
                                syncReqp->portguid, syncReqp->standbyLid);
                    }
                }
                break;
            case DBSYNC_DATATYPE_GROUP:
                if ((status = dbsync_setSMDBGroup(syncReqp))) {
                    IB_LOG_WARN_FMT(__func__, 
                            "failed to sync (full) GROUP records with SM at portGuid "FMT_U64", LID=[0x%x]", 
                            syncReqp->portguid, syncReqp->standbyLid);
                }
                /* update the sync status of this SM node */
                (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_GROUP, 
                                                (status != VSTATUS_OK) ? DBSYNC_STAT_FAILURE:DBSYNC_STAT_SYNCHRONIZED);
                if (status == VSTATUS_OK) {
                    /* update the sync status of the remote SM */
                    if (dbsync_setSMDBSync(syncReqp)) {
                        IB_LOG_WARN_FMT(__func__, 
                                "failed to SET current sync capability of SM at portGuid "FMT_U64", LID=[0x%x]", 
                                syncReqp->portguid, syncReqp->standbyLid);
                    }
                }
                break;
            case DBSYNC_DATATYPE_SERVICE:
                if ((status = dbsync_setSMDBService(syncReqp))) {
                    IB_LOG_WARN_FMT(__func__, 
                            "failed to sync (full) SERVICE records with SM at portGuid "FMT_U64", LID=[0x%x]", 
                            syncReqp->portguid, syncReqp->standbyLid);
                }
                /* update the sync status of this SM node */
                (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_SERVICE, 
                                                (status != VSTATUS_OK) ? DBSYNC_STAT_FAILURE:DBSYNC_STAT_SYNCHRONIZED);
                if (status == VSTATUS_OK) {
                    /* update the sync status of the remote SM */
                    if (dbsync_setSMDBSync(syncReqp)) {
                        IB_LOG_WARN_FMT(__func__, 
                                "failed to SET current sync capability of SM at portGuid "FMT_U64", LID=[0x%x]", 
                                syncReqp->portguid, syncReqp->standbyLid);
                    }
                }
                break;
            case DBSYNC_DATATYPE_MCROOT:
				 if ((status = dbsync_setSMDBMCRoot(syncReqp))) {
                    IB_LOG_WARN_FMT(__func__, 
                            "failed to sync (full) Multicast root with SM at portGuid "FMT_U64", LID=[0x%x]", 
                            syncReqp->portguid, syncReqp->standbyLid);
                }
                /* update the sync status of this SM node */
                (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_MCROOT, 
                                                (status != VSTATUS_OK) ? DBSYNC_STAT_FAILURE:DBSYNC_STAT_SYNCHRONIZED);
                if (status == VSTATUS_OK) {
                    /* update the sync status of the remote SM */
                    if (dbsync_setSMDBSync(syncReqp)) {
                        IB_LOG_WARN_FMT(__func__, 
                                "failed to SET current sync capability of SM at portGuid "FMT_U64", LID=[0x%x]", 
                                syncReqp->portguid, syncReqp->standbyLid);
                    }
                }
				break;
            case DBSYNC_DATATYPE_DATELINE_GUID:
				if ((status = dbsync_setSMDBDatelineSwitchGUID(syncReqp))) {
			IB_LOG_WARN_FMT(__func__,
				"failed to sync (full) Dateline Switch GUID with SM at portGUID "FMT_U64", LID=[0x%x]",
				syncReqp->portguid, syncReqp->standbyLid);
		}
		/* update the sync status of this SM node */
		(void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_DATELINE_GUID,
						(status != VSTATUS_OK) ? DBSYNC_STAT_FAILURE:DBSYNC_STAT_SYNCHRONIZED);
		if (status == VSTATUS_OK) {
			/* update the sync status of the remote SM */
			if (dbsync_setSMDBSync(syncReqp)) {
				IB_LOG_WARN_FMT(__func__,
					"failed to SET current sync capability of SM at portGUID "FMT_U64", LID=[0x%x]",
					syncReqp->portguid, syncReqp->standbyLid);
			}
		}
			break;
            default:
                IB_LOG_ERROR_FMT(__func__, 
                        "dbsync received invalid sync data type of %d (should be 1=all, thu 4=service)", 
                        syncReqp->datatype);
                break;
            }
        } else if (syncReqp->type == DBSYNC_TYPE_UPDATE || syncReqp->type == DBSYNC_TYPE_DELETE) {
            /* do not do updates with SM currently in failure state */
            if ((sm_dbsync_getSmDbsync(syncReqp->portguid, &dbsync)) != VSTATUS_OK ||
                dbsync.fullSyncStatus != DBSYNC_STAT_SYNCHRONIZED ||
                dbsync.groupSyncStatus != DBSYNC_STAT_SYNCHRONIZED ||
                dbsync.informSyncStatus != DBSYNC_STAT_SYNCHRONIZED ||
                dbsync.serviceSyncStatus != DBSYNC_STAT_SYNCHRONIZED) break;
            /* process update request */
            switch (syncReqp->datatype) {
            case DBSYNC_DATATYPE_INFORM:
                if ((status = dbsync_updateSMDBInform(syncReqp))) {
                    IB_LOG_WARN_FMT(__func__, 
                            "failed to sync UPDATE/DELETE of INFORM records with SM at portGuid "FMT_U64", LID=[0x%x]", 
                            syncReqp->portguid, syncReqp->standbyLid);
                    /* update INFORM sync status if failure */
                    (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_INFORM, DBSYNC_STAT_FAILURE);
                }
                break;
            case DBSYNC_DATATYPE_GROUP:
                if ((status = dbsync_updateSMDBGroup(syncReqp))) {
                    IB_LOG_WARN_FMT(__func__, 
                            "failed to sync UPDATE/DELETE of GROUP records with SM at portGuid "FMT_U64", LID=[0x%x]", 
                            syncReqp->portguid, syncReqp->standbyLid);
                    /* update the sync status of this SM node */
                    (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_GROUP, DBSYNC_STAT_FAILURE);
                }
                break;
            case DBSYNC_DATATYPE_SERVICE:
                if ((status = dbsync_updateSMDBService(syncReqp))) {
                    IB_LOG_WARN_FMT(__func__, 
                            "failed to sync UPDATE/DELETE of SERVICE records with SM at portGuid "FMT_U64", LID=[0x%x]", 
                            syncReqp->portguid, syncReqp->standbyLid);
                    /* update the sync status of this SM node */
                    (void) sm_dbsync_upSmDbsyncStat(syncReqp->portguid, DBSYNC_DATATYPE_SERVICE, DBSYNC_STAT_FAILURE);
                }
                break;
            default:
                IB_LOG_ERROR_FMT(__func__, 
                        "dbsync received invalid sync update data type of %d (should be 2=inform, 3=group, 4=service)", 
                        syncReqp->datatype);
                break;
            }
        } else if (syncReqp->type == DBSYNC_TYPE_BROADCAST_FILE) {
			/* process update request */
			// JPW
			switch (syncReqp->datatype) {
			case DBSYNC_DATATYPE_FILE:
				if ((status = dbsync_sendFileSMDBSync(syncReqp))) {
					IB_LOG_WARN_FMT(__func__,
					"failed to sync file with SM at portGuid "FMT_U64", LID=[0x%x]",
					syncReqp->portguid, syncReqp->standbyLid);
				}
				break;
			default:
				IB_LOG_ERROR_FMT(__func__,
					"dbsync received invalid sync update data type of %d", syncReqp->datatype);
				break;
			}
        } else if (syncReqp->type == DBSYNC_TYPE_RECONFIG) {
			if (dbsync_reconfigSMDBSync(syncReqp)) {
                IB_LOG_INFINI_INFO_FMT(__func__, 
                        "failed to reconfigure sync capability of SM at portGuid "FMT_U64", LID=[0x%x]", 
                        syncReqp->portguid, syncReqp->standbyLid);
            }
        } else {
            IB_LOG_WARN_FMT(__func__, "invalid db sync type %d (should be 1=full, 2=update)", (int)syncReqp->type);
        }
        /* free the syncReq space */
        vs_pool_free(&sm_pool, (void *)syncReqp);
    }
    IB_EXIT(__func__, 0);
    return;
}


/*
 * see if remote SM still around
*/
//static dbsync_pingSm(STL_LID lid) {
//    (void)if3_set_dlid(dbsyncfd_if3, lid);
//    /* ping the remote sm's dbsync thread */
//    if(If3CntrlCmdSend(dbsyncfd_if3, FE_MNGR_PROBE_CMD) != VSTATUS_OK) {
//        // get the portInfo of 
//    }
//}


/*
 * sm_dbsync main function
 * processes requests off the dbsync queue when in master mode, sending sync messages to stanby SMs
 * processes sync messages from master when in standby mode
*/
void sm_dbsync(uint32_t argc, uint8_t ** argv) {
    Status_t    status;
    Mai_t       mad;
    uint32_t    madrc, msglen;
#ifdef __SIMULATOR__
	uint64_t	if3open_retry_time = (VTIMER_1S * 10);
	uint64_t	if3timeout_retry_time = (VTIMER_1S * 15);
	uint32_t	if3_timeout_retry_count = 2;
#else
	uint64_t	if3open_retry_time = (VTIMER_1S * 2);
	uint64_t	if3timeout_retry_time = 333000ull;
	uint32_t	if3_timeout_retry_count = 3;
#endif

    IB_ENTER(__func__, 0, 0, 0, 0);

    /* Get my thread name. */
    (void)vs_thread_name(&sm_threads[SM_THREAD_DBSYNC].name);

    /*
     * Open a connection to the IF3 interface. 
     * Stay here until connection is opened and we have our port guid
     */
    dbsyncfd_if3 = -1;

#ifndef VXWORKS
    // wait until SM is in MASTER/STANDBY state
    sm_wait_ready(VIEO_IF3_MOD_ID);
#endif

    while (!dbsync_main_exit && dbsyncfd_if3 == -1) {
        if ((status = if3_open (sm_config.hca, sm_config.port, MAD_CV_VENDOR_DBSYNC, &dbsyncfd_if3)) != VSTATUS_OK) {
            IB_LOG_INFINI_INFO_FMT(__func__, "Can't open connection to IF3 driver (status = %d), retrying", status);
            (void) vs_thread_sleep (if3open_retry_time);
        } else {
            /* set for 3 retry on 1/3 second interval */
            (void)if3_timeout_retry(dbsyncfd_if3, if3timeout_retry_time, if3_timeout_retry_count);
            if (smDebugPerf) 
                IB_LOG_INFINI_INFO_FMT(__func__, "dbsyncfd_if3 = %"PRIdN, dbsyncfd_if3);
        }
    }

    /* exit if necessary */
    if (dbsync_main_exit) goto bail;

    /*
     * allocate buffer for intra SM communication
     * we are assuming that INFORM messages are the largest messages block and 
     * are therefore calculating the buffer length based on 4 subscriptions per end port
	 * and additional uint32_t for record count
    */
    buflen = sizeof(uint32_t) + ((sizeof(STL_INFORM_INFO_RECORD) + sizeof(SubscriberKey_t)) * 4 *(sm_config.subnet_size+(sm_config.subnet_size/4)));
	if (pm_config.image_update_interval) {
		buflen = 75000000; /* FIXME: PM Sweep/History data is using SM Dbsync. */
	}

    if ((status = vs_pool_alloc(&sm_pool, buflen, (void *)&msgbuf)) != VSTATUS_OK) {
        IB_FATAL_ERROR("sm_dbsync: Can't allocate space for SM to SM sync messages");
        IB_EXIT(__func__, VSTATUS_NOMEM);
        return;
    } else {
        memset((void *)msgbuf, 0, buflen);
        if (smDebugPerf) {
            IB_LOG_INFINI_INFO("allocated dbsync receive buffer of length ", buflen);
        }
    }

    dbsync_main_exit = 0;
    while (!dbsync_main_exit) {
        /*
         * master mode processing
         * process sync request from the dbsync queue and synchronize the appropriate
         * standby SM
         */
        while (sm_config.db_sync_interval && sm_state == SM_STATE_MASTER && !dbsync_main_exit) {
            /* we don't want any filters on the FD when we're master, this is
             * handled at the if3 layer.
             */
            (void) dbsync_procReqQ();
            (void) vs_thread_sleep (VTIMER_1S);
        }
        /*
         * standby mode processing
         */
        while (sm_config.db_sync_interval && sm_state == SM_STATE_STANDBY && !dbsync_main_exit) {
            /* 
             * filter handling for receiving dbsync messages when we're standby 
             * is handled at the if3 layer.
             */

            /* wait for command from the master SM */
            msglen = buflen;
            status = if3_dbsync_cmd_from_mngr(dbsyncfd_if3, &mad, msgbuf, &msglen, &madrc, NULL, NULL);
            if (status == VSTATUS_OK) {
#if defined(IB_STACK_OPENIB)
				status = ib_refresh_devport();
				if (status != VSTATUS_OK) {
					IB_LOG_ERRORRC("cannot refresh sm pkeys rc:", status);
				}
#endif
                /* process dbsync request from master SM */
                switch (mad.type) {
                case MAI_TYPE_EXTERNAL:
                    if (mad.base.method != RMPP_CMD_GET && mad.base.method != RMPP_CMD_GETTABLE) {
                        IB_LOG_INFO_FMT(__func__, 
                                "dbsync received method 0x%x, dbsync message CMD(aid)=%d, amod=%d", 
                                mad.base.method, mad.base.aid, mad.base.amod);
                        continue;
                    }
                    switch (mad.base.aid) {
                    case DBSYNC_AID_SYNC:
                        status = processSMDBSyncGetSet(&mad, msgbuf, msglen);
                        break;
                    case DBSYNC_AID_INFORM:
                        status = processInformSync(&mad, msgbuf, msglen);
                        break;
                    case DBSYNC_AID_GROUP:
                        status = processGroupSync(&mad, msgbuf, msglen);
                        break;
                    case DBSYNC_AID_SERVICE:
                        status = processServiceSync(&mad, msgbuf, msglen);
                        break;
                    case DBSYNC_AID_MCROOT:
                        status = processMCRootSync(&mad, msgbuf, msglen);
                        break;
                   case DBSYNC_AID_DATELINE:
                        status = processDatelineSwitchGUIDSync(&mad, msgbuf, msglen);
                        break;
                    default:
                        IB_LOG_ERRORX("dbsync received invalid command AID of ", mad.base.aid);
                        break;
                    }
                    break;
                case MAI_TYPE_INTERNAL:
                    IB_LOG_INFINI_INFO("dbsync received INTERNAL control mad_type=", mad.type);
                    break;
                default:
                    IB_LOG_ERROR("dbsync received invalid mad_type=", mad.type);
                    break;
                }
            } else if (status != VSTATUS_TIMEOUT) {
                IB_LOG_WARNRC("receive error on dbsync file descriptor, rc:", status);
            }
        } /* process sync requests while in standby mode */

        /*
         * do nothing until we transition to standby or master
         */
        while ( (!sm_config.db_sync_interval || sm_state < SM_STATE_STANDBY) && !dbsync_main_exit ) {
            (void) vs_thread_sleep (VTIMER_1S);
        }
    }

    /* free the SM dbsync request queue, sm record table, and close if3 handle and associated filters */
    cs_queue_DisposeQueue( &sm_pool, sm_dbsync_queue );
bail:
    if (dbsyncfd_if3 > 0) (void)if3_dbsync_close(dbsyncfd_if3);
    dbsyncfd_if3 = -1;
    dbsync_initialized_flag = 0;
    sm_dbsync_recDelete();
    if (msgbuf) {
        vs_pool_free(&sm_pool, msgbuf);
    }
    IB_EXIT(__func__, 0);
}

