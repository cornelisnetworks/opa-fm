/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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
#include "cs_g.h"
#include "cs_csm_log.h"
#include "mai_g.h"
#include "sm_l.h"
#include "sa_l.h"
#include "sm_dbsync.h"
#include "time.h"
#ifdef __VXWORKS__
#include "UiUtil.h"
#endif

extern	SmMaiHandle_t	*fd_dbsync;

/*
 * sync capability text display value
*/
char *getSyncCapText(DBSyncCap_t value) {
    switch (value) {
    case DBSYNC_CAP_UNKNOWN:
        return("UNKNOWN");
    case DBSYNC_CAP_NOTSUPPORTED:
        return("NOT SUPPORTED");
    case DBSYNC_CAP_SUPPORTED:
        return("SUPPORTED");
    default:
        return("invalid sync capability value");
    }
}


/*
 * sync satus text display value
*/
char *getSyncStatText(DBSyncStat_t value) {
    switch (value) {
    case DBSYNC_STAT_UNINITIALIZED:
        return("UNINITIALIZED");
    case DBSYNC_STAT_INPROGRESS:
        return("IN PROGRESS");
    case DBSYNC_STAT_SYNCHRONIZED:
        return("SYNCHRONIZED");
    case DBSYNC_STAT_FAILURE:
        return("FAILURE");
    default:
        return("invalid sync capability value");
    }
}


static const char * getSmSyncTime (uint32_t epoc_time)
{
#ifdef __VXWORKS__
    UiUtil_ConvertGMTToLocal((time_t *)(void *)&epoc_time);
#endif
    return (ctime((const time_t *)(void *)&epoc_time));
}

/*
 * Routine      : dbsync_filter_add
 *
 * Description  : Add up a Filter.
 * Pre-process  : None
 * Input        : fd         - MAI handle
 *                filterp    - pointer to a filter
 *                mclass     - class to filter on
 *                method     - method to filter on
 *                aid        - aid to filter on
 *                amod       - amod to filter on
 *                tid        - tid to filter on
 *                subRoutine - name of routine that called us
 *                instance   - instance within sub routine
 * Output       : mai_filter_add status
 * Post-process : The Filter is updated.
 * Reference    : None
 */
Status_t
sm_dbsync_filter_add (IBhandle_t fd, Filter_t *filter, uint8_t mclass, 
                   uint8_t method, uint16_t aid, uint32_t amod, uint64_t tid, const char *subRoutine){
    Status_t    status;

    IB_ENTER (__func__, mclass, method, aid, amod);

    memset (filter, 0, sizeof (Filter_t));
    memcpy(filter->fname, "dbsync", 7);
    filter->active  = MAI_ACT_FMASK;
    filter->type    = MAI_TYPE_EXTERNAL;
    filter->dev     = sm_config.hca;
    filter->port    = (sm_config.port == 0) ? MAI_TYPE_ANY : sm_config.port;
    filter->qp      = 1;
    if (mclass) {
        filter->value.mclass = mclass;
        filter->mask.mclass  = 0xff;
    }
    if (method) {
        filter->value.method = method;
        filter->mask.method  = 0xff;
    }
    if (aid) {
        filter->value.aid    = aid;
        filter->mask.aid     = 0xffff;
    }
    if (amod) {
        filter->value.amod   = amod;
        filter->mask.amod    = 0xffffffff;
    }
    if (tid) {
        filter->value.tid   = tid;
        filter->mask.tid    = 0xffffffffffffffffull;
    }

    status = mai_filter_create (fd, filter, VFILTER_SHARE|VFILTER_PURGE);
    if (status != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__, 
                "In '%s', can't create filter rc: %d)\n", subRoutine, status);
    }

    IB_EXIT (__func__, status);
    return(status);
}

/*
 * Initialize SmRec_t structure
*/
void sm_dbsync_initSmRec(SmRecp smrecp) {
	smrecp->dbsync.version = SM_DBSYNC_VERSION;
	smrecp->syncCapability = (sm_config.db_sync_interval > 0) ? DBSYNC_CAP_SUPPORTED : DBSYNC_CAP_NOTSUPPORTED;
	/* clear the record's dbsync structure */
	smrecp->dbsync.fullSyncStatus=smrecp->dbsync.fullSyncFailCount=smrecp->dbsync.fullTimeSyncFail=smrecp->dbsync.fullTimeLastSync=0;
	smrecp->dbsync.informSyncStatus=smrecp->dbsync.informSyncFailCount=smrecp->dbsync.informTimeSyncFail=smrecp->dbsync.informTimeLastSync=0;
	smrecp->dbsync.groupSyncStatus=smrecp->dbsync.groupSyncFailCount=smrecp->dbsync.groupTimeSyncFail=smrecp->dbsync.groupTimeLastSync=0;
	smrecp->dbsync.serviceSyncStatus=smrecp->dbsync.serviceSyncFailCount=smrecp->dbsync.serviceTimeSyncFail=smrecp->dbsync.serviceTimeLastSync=0;
	smrecp->dbsync.datelineGuidSyncStatus=smrecp->dbsync.datelineGuidSyncFailCount=smrecp->dbsync.datelineGuidTimeSyncFail=smrecp->dbsync.datelineGuidTimeLastSync=0;
	/* fill in the XML consistency check version and clear data */
	smrecp->dbsyncCCC.protocolVersion = FM_PROTOCOL_VERSION;
	smrecp->dbsyncCCC.smVfChecksum = smrecp->dbsyncCCC.smConfigChecksum = 0;
	smrecp->dbsyncCCC.pmConfigChecksum = 0;
	smrecp->dbsyncCCC.spare1 = smrecp->dbsyncCCC.spare2 = smrecp->dbsyncCCC.spare3 = smrecp->dbsyncCCC.spare4 = 0;
	smrecp->dbsyncCCC.spare5 = smrecp->dbsyncCCC.spare6 = smrecp->dbsyncCCC.spare7 = 0;
}

/*
 * send sync request on sync queue to db_sync thread
*/
void sm_dbsync_queueMsg(DBSyncType_t syncType, DBSyncDatTyp_t syncDataTye, STL_LID lid, Guid_t portguid, uint8_t isEmbedded, SMSyncData_t data) {
    SMSyncReqp      srp;
    Status_t        status;

    /* just drop the request if sync is off */
    if (!sm_config.db_sync_interval || dbsync_main_exit) return;
    if ((status = vs_pool_alloc(&sm_pool, sizeof(SMSyncReq_t), (void *)&srp)) != VSTATUS_OK) {
		IB_LOG_ERRORRC("unable to allocate space for SM sync request queue entry, rc:", status);
    } else {
        memset((void *)srp, 0, sizeof(SMSyncReq_t));
        srp->type = syncType;
        srp->datatype = syncDataTye;
        srp->portguid = portguid;
        srp->standbyLid = lid;
        srp->isEmbedded = isEmbedded;
        /* copy the data if not null */
        if (data) {
            memcpy(srp->data, data, sizeof(SMSyncData_t));
        }
        if ((status = cs_queue_Enqueue((cs_QueueElement_ptr)srp, sm_dbsync_queue)) != VSTATUS_OK) {
            IB_LOG_WARN_FMT(__func__,
                   "unable to queue sync request for transmission, status=%d", status);
            /* free the entry */
            vs_pool_free(&sm_pool, srp);
        }
    }
    IB_EXIT(__func__, VSTATUS_OK);
    return;
}


/*
 * delete an SM entry
*/
void sm_dbsync_deleteSm(uint64_t portguid){
	SmRecKey_t      smreckey=portguid;
    SmRecp          smrecp;
    Status_t        status=VSTATUS_OK;

    IB_ENTER(__func__, portguid, 0, 0, 0);

    if (dbsync_main_exit) return;
    /* lock out service record hash table */
    if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Can't lock SM Record table, rc:", status);
    } else {
        if (NULL != (smrecp = (SmRecp)cs_hashtable_remove(smRecords.smMap, &smreckey))) {
            /* free the SM record */
            free(smrecp);
        }
        (void)vs_unlock(&smRecords.smLock);
    }
    IB_EXIT(__func__, VSTATUS_OK);
    return;
}


/*
 * add new SM entry or update data of existing entry
*/
void 
sm_dbsync_addSm(Node_t *nodep, Port_t * portp, STL_SMINFO_RECORD *sminforec)
{
	SmRecKeyp       smreckeyp;
    SmRecp          smrecp;
	uint8_t         *path;

    IB_ENTER(__func__, nodep, portp, sminforec, 0);

    if (dbsync_main_exit) return;
    /* lock out service record hash table */
	if (vs_lock(&smRecords.smLock) != VSTATUS_OK) {
        IB_LOG_INFINI_INFO0("Can't lock SM Record table");
        return;
    }
    /* allocate a SM table key for searching hash table */
    smreckeyp = (SmRecKeyp) malloc(sizeof(SmRecKey_t));
    if (smreckeyp == NULL) {
        (void)vs_unlock(&smRecords.smLock);
        IB_LOG_ERRORLX("Can't allocate SM table entry key smguid:", sminforec->SMInfo.PortGUID);
        return;
    }
    /* get the sm table entry key = guid */
    *smreckeyp = sminforec->SMInfo.PortGUID;
    /* see if this is a new SM; If it already exist, update it's smrec with data passed in */
    if (NULL == (smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, smreckeyp))) {
        /* allocate an SM record for adding to hash table */
        if ((smrecp = (SmRecp) malloc(sizeof(SmRec_t))) == NULL) {
            (void)vs_unlock(&smRecords.smLock);
            free(smreckeyp);
            IB_FATAL_ERROR_NODUMP("sm_dbsync_addSm: Can't allocate SM Record table entry");
            IB_EXIT(__func__, VSTATUS_NOMEM);
            return;
        }
        memset((void *)smrecp, 0, sizeof(SmRec_t));
        smrecp->portguid = sminforec->SMInfo.PortGUID;
        smrecp->lid = sminforec->RID.LID;
		smrecp->portNumber = portp->index;
        if (nodep == sm_newTopology.node_head) {
#ifdef __VXWORKS__
			smrecp->isEmbedded = 1;
#else
			smrecp->isEmbedded = 0;
#endif
		} else {
			smrecp->isEmbedded = portp->index ? 0 : 1;
		}

        /* fill in dbsync for our node */
        if (nodep == sm_newTopology.node_head) {
			sm_dbsync_initSmRec(smrecp);
        }
        smrecp->smInfoRec = *sminforec;
        memcpy(smrecp->nodeDescString, nodep->nodeDesc.NodeString, STL_NODE_DESCRIPTION_ARRAY_SIZE);
        smrecp->nodeDescString[NODE_DESCRIPTION_ARRAY_SIZE - 1] = '\0';
        path = PathToPort(nodep, portp);
        memcpy(smrecp->path, path, 64);
        if (!cs_hashtable_insert(smRecords.smMap, smreckeyp, smrecp)) {
            (void)vs_unlock(&smRecords.smLock);
            IB_LOG_ERROR_FMT(__func__, 
                   "Failed to ADD SM record at portGuid "FMT_U64", lid 0x%.8X to SM record table",
                   *smreckeyp, sminforec->RID.LID);
            free(smrecp);
            free(smreckeyp);
            IB_EXIT(__func__, VSTATUS_BAD);
            return;
        } else {
            IB_LOG_INFO_FMT(__func__, 
                   "added SM node at Lid 0x%x, portGuid "FMT_U64" to SM table",
                   sminforec->RID.LID, *smreckeyp);

			if (topology_passcount != 0)
			{
				SmCsmNodeId_t csmNode;
				smCsmFormatNodeId(&csmNode, smrecp->nodeDescString, smrecp->portNumber, smrecp->portguid);
				smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_APPEARANCE, &csmNode, NULL, 
				                "SM at Lid 0x%x, priority: %d", sminforec->RID.LID,
				                sminforec->SMInfo.u.s.Priority);
			}
        }
    } else {
        /* 
         * entry existed already
         * If we are master and we still don't know this SM's sync capability, queue request to get it.
         * If it sync is supported but still unitialized, request a full sync for this SM
         * Must always replace sminfo record value in SM record with input 
         */
        smrecp->lid = sminforec->RID.LID;
        smrecp->smInfoRec = *sminforec;
        memcpy(smrecp->nodeDescString, nodep->nodeDesc.NodeString, STL_NODE_DESCRIPTION_ARRAY_SIZE);
        smrecp->nodeDescString[NODE_DESCRIPTION_ARRAY_SIZE - 1] = '\0';
        path = PathToPort(nodep, portp);
        memcpy(smrecp->path, path, 64);
        IB_LOG_INFO_FMT(__func__, 
               "Updated SM node at Lid 0x%x, portGuid "FMT_U64" in SM table",
               sminforec->RID.LID, *smreckeyp);
        free(smreckeyp);   /* free the previously allocated key */
    }
	(void)vs_unlock(&smRecords.smLock);

    IB_EXIT(__func__, VSTATUS_OK);
    return;
}


/*
 * update smInfoRecord of an SM in the Sm table
*/
Status_t sm_dbsync_fullSyncCheck(SmRecKey_t recKey) {
    SmRecp          smrecp = NULL;
    time_t          tnow;
    uint32_t        collectiveFailStatus=0;
	uint32_t 		timeLastFail=0;
    Status_t        status=VSTATUS_OK;

    IB_ENTER(__func__, recKey, 0, 0, 0);

    /* do nothing if sync is off or exiting */
    if (!sm_config.db_sync_interval || dbsync_main_exit) return status;
    /* lock out SM record table */
	if (vs_lock(&smRecords.smLock)) {
        IB_LOG_ERROR0("Can't lock SM Record table");
    } else {
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) != NULL) {
            if ((tnow = time(NULL)) < 0) {
                IB_LOG_ERROR_FMT(__func__, "failed to get the system time");
            } else {
                /* go ahead with periodic full sync if every thing is in good standing */
                if (smrecp->dbsync.groupSyncStatus == DBSYNC_STAT_FAILURE || 
                    smrecp->dbsync.informSyncStatus == DBSYNC_STAT_FAILURE || 
                    smrecp->dbsync.serviceSyncStatus == DBSYNC_STAT_FAILURE) {
                    collectiveFailStatus = 1;
					/* determine latest fail time amongst the record groupings */
					timeLastFail = MAX(MAX(smrecp->dbsync.groupTimeSyncFail,smrecp->dbsync.informTimeSyncFail),smrecp->dbsync.serviceTimeSyncFail);
                }
                if (!collectiveFailStatus && smrecp->syncCapability == DBSYNC_CAP_SUPPORTED && 
                    smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_UNINITIALIZED) {
                    /* request full sync of this SM */
                    smrecp->dbsync.fullSyncStatus = DBSYNC_STAT_INPROGRESS;
                    (void) sm_dbsync_queueMsg(DBSYNC_TYPE_FULL, DBSYNC_DATATYPE_ALL, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, NULL);
                    IB_LOG_INFO_FMT(__func__, 
                           "requested initial full sync of SM node at Lid 0x%x, portGuid "FMT_U64,
                           smrecp->lid, recKey);
                } else if (!collectiveFailStatus && smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                    if (tnow - smrecp->dbsync.fullTimeLastSync > sm_config.db_sync_interval) {
                        smrecp->dbsync.fullSyncStatus = DBSYNC_STAT_INPROGRESS;
                        (void) sm_dbsync_queueMsg(DBSYNC_TYPE_FULL, DBSYNC_DATATYPE_ALL, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, NULL);
                        IB_LOG_INFO_FMT(__func__, 
                               "requested full sync of SM node at Lid 0x%x, portGuid "FMT_U64" on configured (%d seconds) interval",
                               smrecp->lid, recKey, sm_config.db_sync_interval);
                    }
                } else if ((collectiveFailStatus && (smrecp->dbsync.fullSyncStatus != DBSYNC_STAT_INPROGRESS)) ||
					       (smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_FAILURE)) {
					/* we want to use the latest last fail time here */
					if (smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_FAILURE) 
						timeLastFail = MAX(timeLastFail, smrecp->dbsync.fullTimeSyncFail);
                    /* if were in failure state we will retry a full sync sooner.
                       if we've exhausted retries, just sync on the regular interval */
                    if (  (  smrecp->dbsync.fullSyncFailCount < DBSYNC_MAX_FAIL_SYNC_ATTEMPTS
                          && tnow - timeLastFail > DBSYNC_TIME_BETWEEN_SYNC_FAILURES)
                       || (  smrecp->dbsync.fullSyncFailCount >= DBSYNC_MAX_FAIL_SYNC_ATTEMPTS
                          && tnow - timeLastFail > sm_config.db_sync_interval)) {
                        smrecp->dbsync.fullSyncStatus = DBSYNC_STAT_INPROGRESS;
                        (void) sm_dbsync_queueMsg(DBSYNC_TYPE_FULL, DBSYNC_DATATYPE_ALL, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, NULL);
                        IB_LOG_INFO_FMT(__func__, 
                               "requested full sync of SM node at Lid 0x%x, portGuid "FMT_U64" on failure retry (%d seconds) interval",
                               smrecp->lid, recKey, DBSYNC_TIME_BETWEEN_SYNC_FAILURES);
                    }
                }
            }
        } else {
            IB_LOG_INFO_FMT(__func__,
                   "Can't find SM record for portGuid="FMT_U64" in SM list", recKey);
            status = VSTATUS_NOT_FOUND;
        }
        (void)vs_unlock(&smRecords.smLock);
    }
    IB_EXIT(__func__, status);
    return status;
}


/*
 * see if the standby SM that just ping us needs a periodic full sync
*/
void sm_dbsync_updateSm(SmRecKey_t recKey, STL_SM_INFO *sminfop) {
    SmRecp          smrecp = NULL;

    IB_ENTER(__func__, recKey, sminfop, 0, 0);

    /* do nothing if exiting */
    if (dbsync_main_exit) return;
    /* lock out SM record table */
	if (vs_lock(&smRecords.smLock)) {
        IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table");
    } else {
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) != NULL) {
            /* update the smInfo data of the SM */
            smrecp->smInfoRec.SMInfo = *sminfop;
        }
        (void)vs_unlock(&smRecords.smLock);
    }
    IB_EXIT(__func__, VSTATUS_OK);
    return;
}

/*
 * Is an SM/PM synchronized.
*/
boolean sm_dbsync_isUpToDate(SmRecKey_t recKey, char **reason) {
    SmRecp          smrecp = NULL;
    Status_t        status = VSTATUS_OK;
    boolean         uptodate = 0;

    IB_ENTER(__func__, recKey, 0, 0, 0);

	if (reason) *reason = NULL;

    /* lock out SM record table */
	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
		if (reason) *reason = "Can't lock SM Record table";
    } else {
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) == NULL) {
			if (reason) *reason = "SM not in SM Record table";
		} else if (smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent == SM_STATE_MASTER) {
			/* Already thinks it's master */
			uptodate = 1;
		} else if (smrecp->syncCapability == DBSYNC_CAP_NOTSUPPORTED) {
			/* Sync is disabled */
			uptodate = 0;
		} else if (smrecp->syncCapability == DBSYNC_CAP_SUPPORTED) {
			if (smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_FAILURE) {
				if (reason) *reason = "SM synchronization failed";
			} else if (smrecp->dbsync.fullSyncStatus != DBSYNC_STAT_SYNCHRONIZED) {
				if (reason) *reason = "SM not fully synchronization";
			} else if (pm_config.start && pm_config.image_update_interval
			&& smrecp->pmdbsync.firstUpdateState != DBSYNC_STAT_SYNCHRONIZED) {
				if (reason) *reason = "PM not synchronized";
			} else {
					uptodate = 1;
			}
        } else if (smrecp->syncCapability <= DBSYNC_CAP_ASKING) {
				if (reason) *reason = "SM synchronization capabilities unknown";
		}
        (void)vs_unlock(&smRecords.smLock);
    }
    return uptodate;
}


/*
 * return dbsync capability of an SM
*/
DBSyncCap_t sm_dbsync_getDbsyncSupport(SmRecKey_t recKey) {
    SmRecp          smrecp = NULL;
    Status_t        status = VSTATUS_OK;
    DBSyncCap_t     syncCap=DBSYNC_CAP_UNKNOWN;

    IB_ENTER(__func__, recKey, 0, 0, 0);

    /* lock out SM record table */
	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
    } else {
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) != NULL) {
            if (smrecp->syncCapability == DBSYNC_CAP_UNKNOWN) {
				/* If Capability is unknown, try to find out */
                smrecp->syncCapability = DBSYNC_CAP_ASKING;
                (void) sm_dbsync_queueMsg(DBSYNC_TYPE_GET_CAPABILITY, DBSYNC_DATATYPE_NONE, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, NULL);
            }
            syncCap = smrecp->syncCapability;
        }
        (void)vs_unlock(&smRecords.smLock);
    }
    return syncCap;
}

/*
 * return config consistency of an SM
*/
uint8_t sm_dbsync_getDbsyncConfigConsistency(SmRecKey_t recKey) {
    SmRecp          smrecp = NULL;
    Status_t        status = VSTATUS_OK;
    uint8_t     	configDiff = 0;

    IB_ENTER(__func__, recKey, 0, 0, 0);

    /* lock out SM record table */
	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
    } else {
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) != NULL) {
            configDiff = smrecp->configDiff;
        }
        (void)vs_unlock(&smRecords.smLock);
    }
    return configDiff;
}

/*
 * Get the current sync settings of an SM
*/
Status_t sm_dbsync_getSmDbsync(SmRecKey_t recKey, SMDBSync_t *dbsync) {
    SmRecp          smrecp = NULL;
    Status_t        status = VSTATUS_OK;

    IB_ENTER(__func__, recKey, dbsync, 0, 0);

    /* lock out SM record table */
	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
    } else {
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) != NULL) {
            *dbsync = smrecp->dbsync;
        }
        (void)vs_unlock(&smRecords.smLock);
    }
    return status;
}


/*
 * Get the full sync status of an SM
*/
DBSyncStat_t sm_dbsync_getSmDbsyncStat(SmRecKey_t recKey) {
    SmRecp          smrecp = NULL;
    Status_t        status = VSTATUS_OK;
    DBSyncStat_t    sstat = DBSYNC_STAT_UNINITIALIZED;

    IB_ENTER(__func__, recKey, 0, 0, 0);

    /* lock out SM record table */
	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
    } else {
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) != NULL) {
            sstat = smrecp->dbsync.fullSyncStatus;
        }
        (void)vs_unlock(&smRecords.smLock);
    }
    return sstat;
}


/*
 * update sync status of an SM in the Sm table
*/
Status_t sm_dbsync_upSmDbsyncStat(SmRecKey_t recKey, DBSyncDatTyp_t type, DBSyncStat_t syncStatus) {
    SmRecp          smrecp = NULL;
    Status_t        status = VSTATUS_OK;

    IB_ENTER(__func__, recKey, type, syncStatus, 0);

    /* do nothing if exiting */
    if (dbsync_main_exit) return VSTATUS_OK;
    /* lock out SM record table */
	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
    } else {
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) != NULL) {
            /* update the sync status of the SM */
            switch (type) {
            case DBSYNC_DATATYPE_ALL:
                smrecp->dbsync.fullSyncStatus = syncStatus;
                if (syncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                    smrecp->dbsync.fullSyncFailCount = 0;
                    smrecp->dbsync.fullTimeSyncFail = 0;
                    smrecp->dbsync.fullTimeLastSync = (uint32_t)time(NULL);
                } else if (syncStatus == DBSYNC_STAT_FAILURE) {
                    ++smrecp->dbsync.fullSyncFailCount;
                    smrecp->dbsync.fullTimeSyncFail = (uint32_t)time(NULL);
                }
                break;
            case DBSYNC_DATATYPE_INFORM:
                smrecp->dbsync.informSyncStatus = syncStatus;
                if (syncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                    smrecp->dbsync.informSyncFailCount = 0;
                    smrecp->dbsync.informTimeSyncFail = 0;
                    smrecp->dbsync.informTimeLastSync = (uint32_t)time(NULL);
                } else if (syncStatus == DBSYNC_STAT_FAILURE) {
                    ++smrecp->dbsync.informSyncFailCount;
                    smrecp->dbsync.informTimeSyncFail = (uint32_t)time(NULL);
                }
                break;
            case DBSYNC_DATATYPE_GROUP:
                smrecp->dbsync.groupSyncStatus = syncStatus;
                if (syncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                    smrecp->dbsync.groupSyncFailCount = 0;
                    smrecp->dbsync.groupTimeSyncFail = 0;
                    smrecp->dbsync.groupTimeLastSync = (uint32_t)time(NULL);
                } else if (syncStatus == DBSYNC_STAT_FAILURE) {
                    ++smrecp->dbsync.groupSyncFailCount;
                    smrecp->dbsync.groupTimeSyncFail = (uint32_t)time(NULL);
                }
                break;
            case DBSYNC_DATATYPE_SERVICE:
                smrecp->dbsync.serviceSyncStatus = syncStatus;
                if (syncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                    smrecp->dbsync.serviceSyncFailCount = 0;
                    smrecp->dbsync.serviceTimeSyncFail = 0;
                    smrecp->dbsync.serviceTimeLastSync = (uint32_t)time(NULL);
                } else if (syncStatus == DBSYNC_STAT_FAILURE) {
                    ++smrecp->dbsync.serviceSyncFailCount;
                    smrecp->dbsync.serviceTimeSyncFail = (uint32_t)time(NULL);
                }
                break;
		    case DBSYNC_DATATYPE_MCROOT:
                smrecp->dbsync.mcrootSyncStatus = syncStatus;
                if (syncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                    smrecp->dbsync.mcrootSyncFailCount = 0;
                    smrecp->dbsync.mcrootTimeSyncFail = 0;
                    smrecp->dbsync.mcrootTimeLastSync = (uint32_t)time(NULL);
                } else if (syncStatus == DBSYNC_STAT_FAILURE) {
                    ++smrecp->dbsync.mcrootSyncFailCount;
                    smrecp->dbsync.mcrootTimeSyncFail = (uint32_t)time(NULL);
                }
                break;
	    case DBSYNC_DATATYPE_DATELINE_GUID:
		smrecp->dbsync.serviceSyncStatus = syncStatus;
		if (syncStatus == DBSYNC_STAT_SYNCHRONIZED) {
		    smrecp->dbsync.datelineGuidSyncFailCount = 0;
		    smrecp->dbsync.datelineGuidTimeSyncFail = 0;
		    smrecp->dbsync.datelineGuidTimeLastSync = (uint32_t)time(NULL);
		} else if (syncStatus == DBSYNC_STAT_FAILURE) {
		    ++smrecp->dbsync.datelineGuidSyncFailCount;
		    smrecp->dbsync.datelineGuidTimeSyncFail = (uint32_t)time(NULL);
		}
		break;
            default:
                status = VSTATUS_ILLPARM;
                break;
            }
        } else {
            status = VSTATUS_BAD;
            IB_LOG_INFINI_INFO_FMT(__func__,
                   "Can't find SM record for portGuid="FMT_U64" in SM list", recKey);
        }
        (void)vs_unlock(&smRecords.smLock);
    }
    IB_EXIT(__func__, status);
    return status;
}

/*
 * perform a configuration consistency check
 */
Status_t sm_dbsync_configCheck(SmRecKey_t recKey, SMDBCCCSyncp smSyncSetting) {
	SmRecp          smrecp = NULL;
	Status_t        status = VSTATUS_OK;
	uint8_t			deactivateSM = 0;
	SmCsmMsgCondition_t condition;
	boolean         mismatchDetected = 0;

	IB_ENTER(__func__, recKey, smSyncSetting, 0, 0);

	/* do nothing if exiting */
	if (dbsync_main_exit) return status;

    /* lock out SM record table */
	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_LOG_ERROR("Can't lock SM Record table, rc:", status);
    } else {
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) != NULL) {
			/* assume the config are consistent until proven otherwise */
			smrecp->configDiff = 0;

			/* check checksum of Standby SM with Master SM checksum */
			if (smSyncSetting->protocolVersion == 0) {
				IB_LOG_WARN_FMT(__func__,
					"SM at %s, portGuid="FMT_U64" does not support config consistency checking so config could be out of sync",
					smrecp->nodeDescString, recKey);
			} else if (smSyncSetting->protocolVersion != FM_PROTOCOL_VERSION) {
				IB_LOG_WARN_FMT(__func__,
					"SM at %s, portGuid="FMT_U64" is running incompatible FM version.  Our version [%d], Their version [%d]. Deactivating remote FM",
					smrecp->nodeDescString, recKey, FM_PROTOCOL_VERSION, smSyncSetting->protocolVersion);

				condition = CSM_COND_STANDBY_SM_DEACTIVATION;
				smrecp->syncCapability = DBSYNC_CAP_NOTSUPPORTED;
				sm_dbsync_disableStandbySm(smrecp, condition); 
			} else {
				if (smSyncSetting->smVfChecksum != smRecords.ourChecksums.smVfChecksum) {

					IB_LOG_WARN_FMT(__func__,
						"SM at %s, portGuid="FMT_U64" has a different enabled Virtual Fabric configuration consistency checksum [%u] from us [%u]",
						smrecp->nodeDescString, recKey, smSyncSetting->smVfChecksum, smRecords.ourChecksums.smVfChecksum);

					mismatchDetected = 1;
					deactivateSM = 1;
					condition = CSM_COND_STANDBY_SM_VF_DEACTIVATION;
					smrecp->syncCapability = DBSYNC_CAP_NOTSUPPORTED;
				}

				if (smSyncSetting->smConfigChecksum != smRecords.ourChecksums.smConfigChecksum) {

					IB_LOG_WARN_FMT(__func__,
						"SM at %s, portGuid="FMT_U64" has a different SM configuration consistency checksum [%u] from us [%u]",
						smrecp->nodeDescString, recKey, smSyncSetting->smConfigChecksum, smRecords.ourChecksums.smConfigChecksum);

					mismatchDetected = 1;
					deactivateSM = 1;
					condition = CSM_COND_STANDBY_SM_DEACTIVATION;
					smrecp->syncCapability = DBSYNC_CAP_NOTSUPPORTED;
				}
				if (smSyncSetting->pmConfigChecksum != smRecords.ourChecksums.pmConfigChecksum) {

					IB_LOG_WARN_FMT(__func__,
						"SM at %s, portGuid="FMT_U64" detected a different PM configuration consistency checksum [%u] from us [%u]",
						smrecp->nodeDescString, recKey, smSyncSetting->pmConfigChecksum, smRecords.ourChecksums.pmConfigChecksum);

					mismatchDetected = 1;
					deactivateSM = 1;
					condition = CSM_COND_SECONDARY_PM_DEACTIVATION;
					smrecp->syncCapability = DBSYNC_CAP_NOTSUPPORTED;
				}

				if (mismatchDetected) {
					IB_LOG_WARN_FMT(__func__,
									"opafmconfigdiff tool can be used to show differences in the configuration files that could be causing the checksum mismatches");
				}

				if (deactivateSM) {
					if (sm_config.config_consistency_check_level == CHECK_ACTION_CCC_LEVEL)
						sm_dbsync_disableStandbySm(smrecp, condition); 
					else {
						IB_LOG_WARN_FMT(__func__,
										"Skipping DEACTIVATION of standby SM %s : "FMT_U64" consistency check level not set",
										smrecp->nodeDescString, recKey);
					}
				}
			}
        } else {
            status = VSTATUS_BAD;
			IB_LOG_INFO_FMT(__func__,
                   "Can't find SM record for portGuid="FMT_U64" in SM list", recKey);
        }
        (void)vs_unlock(&smRecords.smLock);
    }

    IB_EXIT(__func__, status);
	return status;
}


/*
 * disable a standby SM
*/
Status_t sm_dbsync_disableStandbySm(SmRecp smRecp, SmCsmMsgCondition_t condition) {

	STL_SM_INFO		smInfoCopy;
	SmCsmNodeId_t	csmNodeId;
    Status_t        status = VSTATUS_OK;

	/* Calling function needs to have acquired the smRecords.smLock */

	smRecp->configDiff = 1;
	smInfoCopy = sm_smInfo;
	SmpAddr_t addr = SMP_ADDR_CREATE_DR(smRecp->path);
	status = SM_Set_SMInfo(fd_dbsync, SM_AMOD_DISABLE, &addr, &smInfoCopy, sm_config.mkey);
	if (status != VSTATUS_OK) {
		IB_LOG_WARN_FMT(__func__,
						"could not DEACTIVATE standby SM %s : "FMT_U64"",
						smRecp->nodeDescString, smRecp->portguid);
	} else {
		smCsmFormatNodeId(&csmNodeId, smRecp->nodeDescString, smRecp->portNumber, smRecp->portguid);
		smCsmLogMessage(CSM_SEV_WARNING, condition, &csmNodeId, NULL,
						"Deactivating standby SM %s : "FMT_U64""
						"The secondary PM will also be deactivated.", smRecp->nodeDescString, smRecp->portguid);
		/* trigger a sweep so the inactive status gets updated */
		sm_trigger_sweep(SM_SWEEP_REASON_UPDATED_STANDBY);
	}

	return status;
}


/*
 * update sync settings of an SM in the Sm table
*/
Status_t sm_dbsync_upSmDbsync(SmRecKey_t recKey, SMDBSyncp smSyncSetting) {
    SmRecp          smrecp = NULL;
    Status_t        status = VSTATUS_OK;

    IB_ENTER(__func__, recKey, smSyncSetting, 0, 0);

    /* do nothing if exiting */
    if (dbsync_main_exit) return VSTATUS_OK;
    /* lock out SM record table */
	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Can't lock SM Record table, rc:", status);
    } else {
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) != NULL) {
            /* update the sync settings */
            smrecp->dbsync = *smSyncSetting;
            if (smSyncSetting->version == SM_DBSYNC_UNKNOWN) {
                smrecp->syncCapability = DBSYNC_CAP_UNKNOWN;
				/* Failed to get the status, tell caller to try again later */
				status = VSTATUS_AGAIN;
            } else if (smSyncSetting->version == SM_DBSYNC_VERSION) {
                smrecp->syncCapability = DBSYNC_CAP_SUPPORTED;
            	IB_LOG_INFO_FMT(__func__,
                   	"Updated sync settings of SM record for portGuid="FMT_U64" in SM list, timeLastFull Sync is %s", 
                   	recKey, getSmSyncTime(smSyncSetting->fullTimeLastSync));
            } else {
                IB_LOG_WARN_FMT(__func__,
                       "SM at %s, portGuid="FMT_U64" supports a different dbsync VERSION [%d] from us [%d]", 
                       smrecp->nodeDescString, recKey, smSyncSetting->version, SM_DBSYNC_VERSION);
                smrecp->syncCapability = DBSYNC_CAP_NOTSUPPORTED;
            }
        } else {
            status = VSTATUS_BAD;
            IB_LOG_INFINI_INFO_FMT(__func__,
                   "Can't find SM record for portGuid="FMT_U64" in SM list", recKey);
        }
        (void)vs_unlock(&smRecords.smLock);
    }
    IB_EXIT(__func__, status);
    return status;
}


/*
 * update sync capability of an SM in the Sm table
*/
Status_t sm_dbsync_upSmDbsyncCap(SmRecKey_t recKey, DBSyncCap_t cap) {
    SmRecp          smrecp = NULL;
    Status_t        status = VSTATUS_OK;

    IB_ENTER(__func__, recKey, cap, 0, 0);

    /* do nothing if exiting */
    if (dbsync_main_exit) return VSTATUS_OK;
    /* lock out SM record table */
	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Can't lock SM Record table, rc:", status);
    } else {
        if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) != NULL) {
            /* update the sync capability */
            smrecp->syncCapability = cap;
        } else {
            status = VSTATUS_BAD;
            IB_LOG_INFINI_INFO_FMT(__func__,
                   "Can't find SM record for portGuid="FMT_U64" in SM list", recKey);
        }
        (void)vs_unlock(&smRecords.smLock);
    }
    IB_EXIT(__func__, status);
    return status;
}


/*
 * update the lids of the SMs on the fabric
 * It should only be called from within the topology discovery loop.
 * It makes use of the current topology (new) being built to validate whether
 * the entries in the SM table are still valid.
*/
Status_t sm_dbsync_upsmlist(SweepContext_t *sweep_context) {
	SmRecKeyp       smreckeyp;
    SmRecp          smrecp;
    CS_HashTableItr_t itr;
    Port_t          *portp=NULL;
	uint32_t		moreEntries;
    Status_t        status=VSTATUS_OK;
	STL_INFORM_INFO_RECORD	*iRecordp = NULL;
    SubscriberKeyp  subsKeyp = NULL;
	OpaServiceRecord_t	*osrp;
    ServiceRecKeyp  srkeyp;
	Guid_t		    mcastGid[2], portguid;
	McGroup_t       *mcGroup=NULL, *prevMcGroup=NULL;
	McMember_t      *mcMember=NULL, *mcmp=NULL; 
	SmCsmNodeId_t	csmNodeId;

    IB_ENTER(__func__, 0, 0, 0, 0);

    /* do nothing if exiting */
    if (dbsync_main_exit) return VSTATUS_OK;
	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
        IB_LOG_ERRORRC("Can't lock SM Record table, rc:",status);
    } else {
        if (cs_hashtable_count(smRecords.smMap) > 0)
        {
            cs_hashtable_iterator(smRecords.smMap, &itr);
            do {
                smreckeyp = cs_hashtable_iterator_key(&itr);
                smrecp = cs_hashtable_iterator_value(&itr);
                /* delete all entries except for ours if we are in standby state */
                if (smrecp->smInfoRec.SMInfo.PortGUID != sm_smInfo.PortGUID  && sm_smInfo.u.s.SMStateCurrent == SM_STATE_STANDBY) {
                    if (smDebugPerf) {
                        IB_LOG_INFINI_INFO_FMT(__func__, 
                               "Removing SM node at Lid 0x%x, portGuid "FMT_U64" from SM table",
                               smrecp->lid, *smreckeyp);
                    }
                    moreEntries = cs_hashtable_iterator_remove(&itr);
                    free(smrecp);
                    continue;
                } else if (smrecp->smInfoRec.SMInfo.PortGUID == sm_smInfo.PortGUID  && sm_smInfo.u.s.SMStateCurrent == SM_STATE_STANDBY) {
                    /* reset my values to initial state */
					sm_dbsync_initSmRec(smrecp);
                    moreEntries = cs_hashtable_iterator_advance(&itr);
                    continue;
                } else if ((portp = sm_find_port_guid(&sm_newTopology, *smreckeyp))) {
                    /* PR 108167 - standby sm segfault */
                    if (!portp->portData || !(portp->portData->capmask & PI_CM_IS_SM)) {
                        /* SM is no longer running on this node, remove it from the list and free record */
                        if (smDebugPerf) {
                            IB_LOG_INFINI_INFO_FMT(__func__, 
                                   "Removing SM node at Lid 0x%x, portGuid "FMT_U64" from SM table",
                                   smrecp->lid, *smreckeyp);
                        }

						smCsmFormatNodeId(&csmNodeId, smrecp->nodeDescString, smrecp->portNumber, smrecp->portguid);
						smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_DISAPPEARANCE, &csmNodeId, NULL,
						                "SM at Lid 0x%x has disappeared", smrecp->lid);

                        moreEntries = cs_hashtable_iterator_remove(&itr);
                        free(smrecp);
                        continue;
                    }
                    /* 
                     * update our smInfo data since its always changing 
                     */
                    if (smrecp->portguid == sm_smInfo.PortGUID) {
                        smrecp->smInfoRec.SMInfo = sm_smInfo;
                    }
                    if (smrecp->lid != portp->portData->lid) {
                        IB_LOG_INFO_FMT(__func__, 
                           "Updating SM node Lid 0x%x, portGuid "FMT_U64" in SM table",
                           portp->portData->lid, *smreckeyp);
                    }
                    /* the lid can change for anybody */
                    smrecp->lid = portp->portData->lid;
                    smrecp->smInfoRec.RID.LID = portp->portData->lid;
                    moreEntries = cs_hashtable_iterator_advance(&itr);  /* iterate to next entry */
                    /* 
                     * Get the sync capability of new SMs other than us that are not in master state
                     */
                    if (sm_config.db_sync_interval) {  /* only if sync is turned on */
                        if (portp->portData->guid != sm_smInfo.PortGUID &&     /* not us */ 
                            sm_smInfo.u.s.SMStateCurrent == SM_STATE_MASTER &&        /* and we're master */
                            smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent <= SM_STATE_STANDBY &&  /* and remote is not master */
                            smrecp->syncCapability == DBSYNC_CAP_UNKNOWN) {/* and we don't know remote sync capability yet */
                            /* request GET of the sync capability of this SM */
                            smrecp->syncCapability = DBSYNC_CAP_ASKING;
                            (void) sm_dbsync_queueMsg(DBSYNC_TYPE_GET_CAPABILITY, DBSYNC_DATATYPE_NONE, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, NULL);
                            IB_LOG_INFO_FMT(__func__, 
                                   "queued request for sync capability of SM node at Lid 0x%x, portGuid "FMT_U64" to SM table",
                                   smrecp->lid, *smreckeyp);
                        } else if (portp->portData->guid != sm_smInfo.PortGUID && sm_smInfo.u.s.SMStateCurrent == SM_STATE_MASTER && 
                                   smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent == SM_STATE_STANDBY && smrecp->syncCapability == DBSYNC_CAP_SUPPORTED && 
                                   smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_UNINITIALIZED && !smrecp->configDiff) {
                            /* request full sync of this SM unless deactivated */
                            smrecp->dbsync.fullSyncStatus = DBSYNC_STAT_INPROGRESS;
                            (void) sm_dbsync_queueMsg(DBSYNC_TYPE_FULL, DBSYNC_DATATYPE_ALL, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, NULL);
                            IB_LOG_INFO_FMT(__func__, 
                                   "requested full sync of SM node at Lid 0x%x, portGuid "FMT_U64" to SM table",
                                   portp->portData->lid, *smreckeyp);
                        }
                    } else {
                        smrecp->syncCapability = DBSYNC_CAP_NOTSUPPORTED;
                    }
                } else {
					smCsmFormatNodeId(&csmNodeId, smrecp->nodeDescString, smrecp->portNumber, smrecp->portguid);
					smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_DISAPPEARANCE, &csmNodeId, NULL,
					                "SM at Lid 0x%x has disappeared", smrecp->lid);

                    /* remove the Sm from the list and free the record */
                    if (smDebugPerf) {
                        IB_LOG_INFINI_INFO_FMT(__func__, 
                               "Removing SM node at Lid 0x%x, portGuid "FMT_U64" from SM table",
                               smrecp->lid, *smreckeyp);
                    }
                    moreEntries = cs_hashtable_iterator_remove(&itr);
                    free(smrecp);
                }
            } while (moreEntries);
        }
    }
    /*
     * For standby SM that has just taken over from a master SM, we need to 
     * insure that the service, informInfo, and group records are still valid.
     * We have to validate that the nodes are still in the topology.
     * We only want to do this for topology_passcount=0.
    */
    if (topology_passcount == 0 && sm_state == SM_STATE_MASTER) {
        /*
         * check the informInfo records to make sure subsciber Gid is still in fabric
         */
        if (!saSubscribers.subsMap || vs_lock(&saSubscribers.subsLock) != VSTATUS_OK) 
            return VSTATUS_BAD;
        if (cs_hashtable_count(saSubscribers.subsMap) > 0)
        {
            cs_hashtable_iterator(saSubscribers.subsMap, &itr);
            do {
                subsKeyp = cs_hashtable_iterator_key(&itr);
                iRecordp = cs_hashtable_iterator_value(&itr);
                portguid = ntoh64(*(uint64_t *)&subsKeyp->subscriberGid[8]);
                if ((sm_find_port_guid(&sm_newTopology, portguid) == NULL)) {
                    /* remove the entry from hashtable and free the value - key is freed by remove func */
                    moreEntries = cs_hashtable_iterator_remove(&itr);
                    IB_LOG_INFO_FMT(__func__,
                           "Deleted subscription record ID 0x%x for GID="FMT_GID, 
                           iRecordp->RID.Enum, ntoh64(*(uint64_t *)&subsKeyp->subscriberGid[0]), 
                           ntoh64(*(uint64_t *)&subsKeyp->subscriberGid[8]));
                    free(iRecordp);
                } else {
                    /* advance the iterator */
                    moreEntries = cs_hashtable_iterator_advance(&itr);
                }
            } while (moreEntries);
        }
        (void)vs_unlock(&saSubscribers.subsLock);
        /*
         * validate the service records
         */
        if (!saServiceRecords.serviceRecMap || vs_lock(&saServiceRecords.serviceRecLock) != VSTATUS_OK) 
            return VSTATUS_BAD;
        if (cs_hashtable_count(saServiceRecords.serviceRecMap) > 0)
        {
            cs_hashtable_iterator(saServiceRecords.serviceRecMap, &itr);
            do {
                srkeyp = cs_hashtable_iterator_key(&itr);
                osrp = cs_hashtable_iterator_value(&itr);
                portguid = srkeyp->serviceGid.Type.Global.InterfaceID;
                if ((sm_find_port_guid(&sm_newTopology, portguid) == NULL)) {
                    /* remove the entry from hashtable and free the value - key is freed by remove func */
                    moreEntries = cs_hashtable_iterator_remove(&itr);
                    IB_LOG_INFO_FMT(__func__,
						"Deleted service record ID="FMT_U64" for GID="FMT_GID", Name=%s", 
						osrp->serviceRecord.RID.ServiceID, 
						STLGIDPRINTARGS(osrp->serviceRecord.RID.ServiceGID),
						osrp->serviceRecord.ServiceName);
						free(osrp);
                } else {
                    /* advance the iterator */
                    moreEntries = cs_hashtable_iterator_advance(&itr);
                }
            } while (moreEntries);
        }
        (void)vs_unlock(&saServiceRecords.serviceRecLock);
        /*
         * validate the group entries
        */
        if ((status = vs_lock(&sm_McGroups_lock)) != VSTATUS_OK) {
            return VSTATUS_BAD;
        } else {
            mcGroup = sm_McGroups;
            while (mcGroup) { /* Walk through all groups and ... */
                mcastGid[0] = mcGroup->mGid.AsReg64s.H;
                mcastGid[1] = mcGroup->mGid.AsReg64s.L;
                mcmp = mcGroup->mcMembers;
                while (mcmp) {
                    mcMember = mcmp;
                    if (mcMember->portGuid == SA_FAKE_MULTICAST_GROUP_MEMBER) {
                        /* skip FACADE member; move the mcMember pointer to next member */
                        mcmp = mcMember->next;
                    } else if (sm_find_port_lid(&sm_newTopology, mcMember->slid) == NULL) {
						// FIXME: mcMember->slid can refer to the proxy requestor for the mcMember...
						// This check should most likely be:
						//   if (sm_find_port_guid(member->portGuid) == NULL)


                        // delete this member
                        mcmp = mcMember->next;
                        if (!(mcMember->state & MCMEMBER_STATE_FULL_MEMBER)) {
                            IB_LOG_INFO_FMT(__func__, "Non full mcMember "FMT_U64" of multicast group "
                                   "GID "FMT_U64":"FMT_U64" is no longer in fabric",
                                   mcMember->portGuid, mcastGid[0], mcastGid[1]);
                        } else {
                            mcGroup->members_full--;
                            IB_LOG_INFO_FMT(__func__, "Full mcMember "FMT_U64" of multicast group "
                                   "GID "FMT_U64":"FMT_U64" is no longer in fabric",
                                   mcMember->portGuid, mcastGid[0], mcastGid[1]);
                        }
                        /* save the portguid before deleting the member */
                        portguid = mcMember->portGuid;
                        McMember_Delete(mcGroup, mcMember);
                        if (mcGroup->members_full == 0) {
                            IB_LOG_INFO_FMT(__func__, "Last full member "FMT_U64" of multicast group "
                                   "GID "FMT_GID" is no longer in fabric, deleting all members",
                                   portguid, mcastGid[0], mcastGid[1]);
                            while (mcGroup->mcMembers) {
                                /* MUST copy head pointer into temp
                                 * Passing mcGroup->members directly to the delete macro will corrupt the list
                                 */
                                mcMember = mcGroup->mcMembers;
                                McMember_Delete(mcGroup, mcMember);
                            }
                            /* set mcmp to null; no one left */
                            mcmp = NULL;
                        } else {
                            IB_LOG_INFO_FMT(__func__, "Full member "FMT_U64" of multicast group "
                                   "GID "FMT_GID" is no longer in fabric",
                                   portguid, mcastGid[0], mcastGid[1]);
                        }
                        if (mcGroup->mcMembers == NULL) { /* Group is empty, delete it */
                            IB_LOG_INFO_FMT(__func__, "deleting multicast group GID "FMT_GID,
                                   mcastGid[0], mcastGid[1]);
                            if (mcGroup == sm_McGroups) {
                                McGroup_Delete(mcGroup);
                                mcGroup = sm_McGroups;
                            } else {
                                McGroup_Delete(mcGroup);
                                mcGroup = prevMcGroup;
                            }

                        }
                    } else {
                        /* move the mcMember pointer to next member */
                        mcmp = mcMember->next;
                    }
                }  // while mcmp
                /* move the group pointer and remember the previous */
                prevMcGroup = mcGroup;
                if (prevMcGroup) {
                    mcGroup = prevMcGroup->next;
                } else {
                    mcGroup = NULL;
                }
            }  // while mcGroup
            (void)vs_unlock(&sm_McGroups_lock);
        }  // if-else lock group table
    }  // topology_passcount == 0
    (void)vs_unlock(&smRecords.smLock);

    IB_EXIT(__func__, status);
    return status;
}


/*
 * sync a service record update or addition with standby SMs
*/
Status_t sm_dbsync_syncService(DBSyncType_t synctype, OpaServiceRecordp osrp) {
    Status_t        status=VSTATUS_OK;
    SmRecp          smrecp;
    CS_HashTableItr_t itr;
    SMSyncData_t    syncData={0};
//  time_t          tnow=0;

    IB_ENTER(__func__, synctype, osrp, 0, 0);

    /* do nothing if sync is not turned on or we are exiting */
    if (!sm_config.db_sync_interval || dbsync_main_exit) return VSTATUS_OK;
    if ((sizeof(OpaServiceRecord_t)) > sizeof(SMSyncData_t)) {
        status = VSTATUS_NOMEM;
        IB_LOG_ERROR_FMT(__func__, "SMSyncData_t type not large enough");
    } else {
        if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
        } else {
            /* must have Sms besides us */
            if (cs_hashtable_count(smRecords.smMap) > 1) {
//              tnow = time(NULL);
                if (osrp) memcpy(syncData, (void *)osrp, sizeof(OpaServiceRecord_t));
                cs_hashtable_iterator(smRecords.smMap, &itr);
                do {
                    smrecp = cs_hashtable_iterator_value(&itr);
                    /* request sync of the service record for standby SMs in good standing */
                    if (smrecp->portguid != sm_smInfo.PortGUID && 
                        smrecp->syncCapability == DBSYNC_CAP_SUPPORTED && 
                        smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                        if (smrecp->dbsync.serviceSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                            (void) sm_dbsync_queueMsg(synctype, DBSYNC_DATATYPE_SERVICE, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
                        } 
                        #if 0
                        else if (smrecp->dbsync.serviceSyncStatus == DBSYNC_STAT_FAILURE &&
                                   smrecp->dbsync.serviceSyncFailCount < DBSYNC_MAX_FAIL_SYNC_ATTEMPTS &&
                                   ((tnow - smrecp->dbsync.serviceTimeSyncFail) > DBSYNC_TIME_BETWEEN_SYNC_FAILURES)) {
                            /* 
                             * see if a full sync can clear things up after DBSYNC_TIME_BETWEEN_SYNC_FAILURES seconds 
                             */
                            (void) sm_dbsync_queueMsg(DBSYNC_TYPE_FULL, DBSYNC_DATATYPE_SERVICE, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
                        } else if (smrecp->dbsync.serviceSyncStatus == DBSYNC_STAT_FAILURE &&
                                   smrecp->dbsync.serviceSyncFailCount >= DBSYNC_MAX_FAIL_SYNC_ATTEMPTS &&
                                   ((tnow - smrecp->dbsync.serviceTimeSyncFail) > DBSYNC_TIME_BETWEEN_SYNC_FAILURES)) {
                            /* last attempt, try a full sync of everything */
                            (void) sm_dbsync_queueMsg(DBSYNC_TYPE_FULL, DBSYNC_DATATYPE_ALL, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
                        }
                        #endif
                    }
                } while (cs_hashtable_iterator_advance(&itr));
            }
            (void)vs_unlock(&smRecords.smLock);
        }
    }
    IB_EXIT(__func__, status);
    return status;
}


/*
 * sync an INFORM record update or addition with standby SMs
*/
Status_t sm_dbsync_syncInform(DBSyncType_t synctype, SubscriberKeyp keyp, STL_INFORM_INFO_RECORD *recp) {
    Status_t        status=VSTATUS_OK;
    SmRecp          smrecp;
    CS_HashTableItr_t itr;
    SMSyncData_t    syncData={0};
//  time_t          tnow=0;

    IB_ENTER(__func__, synctype, keyp, recp, 0);

    /* do nothing if sync is not turned on or we are exiting */
    if (!sm_config.db_sync_interval || dbsync_main_exit) return VSTATUS_OK;
    if ((sizeof(SubscriberKey_t)+sizeof(STL_INFORM_INFO_RECORD)) > sizeof(SMSyncData_t)) {
        status = VSTATUS_NOMEM;
        IB_LOG_ERROR_FMT(__func__, "SMSyncData_t type not large enough");
    } else {
        if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
        } else {
            if (cs_hashtable_count(smRecords.smMap) > 1) {
                if (keyp) memcpy(syncData, (void *)keyp, sizeof(SubscriberKey_t));
                if (recp) memcpy(syncData+sizeof(SubscriberKey_t), (void *)recp, sizeof(STL_INFORM_INFO_RECORD));
//              tnow = time(NULL);
                cs_hashtable_iterator(smRecords.smMap, &itr);
                do {
                    smrecp = cs_hashtable_iterator_value(&itr);
                    /* request sync of the service record for SMs in good standing */
                    if (smrecp->portguid != sm_smInfo.PortGUID &&    /* make sure it's not us */ 
                        smrecp->syncCapability == DBSYNC_CAP_SUPPORTED && 
                        smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                        if (smrecp->dbsync.informSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                            (void) sm_dbsync_queueMsg(synctype, DBSYNC_DATATYPE_INFORM, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
                        }
                        #if 0
                        else if (smrecp->dbsync.informSyncStatus == DBSYNC_STAT_FAILURE &&
                                   smrecp->dbsync.informSyncFailCount < DBSYNC_MAX_FAIL_SYNC_ATTEMPTS &&
                                   ((tnow - smrecp->dbsync.informTimeSyncFail) > DBSYNC_TIME_BETWEEN_SYNC_FAILURES)) {
                            /* 
                             * see if a full sync can clear things up after DBSYNC_TIME_BETWEEN_SYNC_FAILURES seconds 
                             */
                            (void) sm_dbsync_queueMsg(DBSYNC_TYPE_FULL, DBSYNC_DATATYPE_INFORM, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
                        } else if (smrecp->dbsync.informSyncStatus == DBSYNC_STAT_FAILURE &&
                                   smrecp->dbsync.informSyncFailCount >= DBSYNC_MAX_FAIL_SYNC_ATTEMPTS &&
                                   ((tnow - smrecp->dbsync.informTimeSyncFail) > DBSYNC_TIME_BETWEEN_SYNC_FAILURES)) {
                            /* last attempt, try a full sync of everything */
                            (void) sm_dbsync_queueMsg(DBSYNC_TYPE_FULL, DBSYNC_DATATYPE_ALL, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
                        }
                        #endif
                    }
                } while (cs_hashtable_iterator_advance(&itr));
            }
            (void)vs_unlock(&smRecords.smLock);
        }
    }
    IB_EXIT(__func__, status);
    return status;
}


/*
 * sync an GROUP record update/addition/deletion with standby SMs
*/
Status_t sm_dbsync_syncGroup(DBSyncType_t synctype, IB_GID * pGid) {
    Status_t        status=VSTATUS_OK;
    SmRecp          smrecp;
    CS_HashTableItr_t itr;
    SMSyncData_t    syncData={0};
//  time_t          tnow=0;

    IB_ENTER(__func__, synctype, pGid, 0, 0);

    /* do nothing if sync is not turned on or we are exiting */
    if (!sm_config.db_sync_interval || dbsync_main_exit) return VSTATUS_OK;
    if (sizeof(IB_GID) > sizeof(SMSyncData_t)) {
        status = VSTATUS_NOMEM;
        IB_LOG_ERROR_FMT(__func__, "SMSyncData_t type not large enough");
    } else {
        if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
        } else {
            if (cs_hashtable_count(smRecords.smMap) > 1) {
                if (pGid) memcpy(syncData, pGid, sizeof(IB_GID));
//              tnow = time(NULL);
                cs_hashtable_iterator(smRecords.smMap, &itr);
                do {
                    smrecp = cs_hashtable_iterator_value(&itr);
                    /* request sync of the GROUP record for SMs in good standing */
                    if (smrecp->portguid != sm_smInfo.PortGUID &&    /* make sure it's not us */ 
                        smrecp->syncCapability == DBSYNC_CAP_SUPPORTED && 
                        smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                        if (smrecp->dbsync.groupSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                            (void) sm_dbsync_queueMsg(synctype, DBSYNC_DATATYPE_GROUP, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
                        } 
                        #if 0
                        else if (smrecp->dbsync.groupSyncStatus == DBSYNC_STAT_FAILURE &&
                                   smrecp->dbsync.groupSyncFailCount < DBSYNC_MAX_FAIL_SYNC_ATTEMPTS &&
                                   ((tnow - smrecp->dbsync.groupTimeSyncFail) > DBSYNC_TIME_BETWEEN_SYNC_FAILURES)) {
                            /* 
                             * see if a full sync can clear things up after DBSYNC_TIME_BETWEEN_SYNC_FAILURES seconds 
                             */
                            (void) sm_dbsync_queueMsg(DBSYNC_TYPE_FULL, DBSYNC_DATATYPE_GROUP, smrecp->lid, smrecp->portguid, syncData);
                        } else if (smrecp->dbsync.groupSyncStatus == DBSYNC_STAT_FAILURE &&
                                   smrecp->dbsync.groupSyncFailCount >= DBSYNC_MAX_FAIL_SYNC_ATTEMPTS &&
                                   ((tnow - smrecp->dbsync.groupTimeSyncFail) > DBSYNC_TIME_BETWEEN_SYNC_FAILURES)) {
                            /* last attempt, try a full sync of everything */
                            (void) sm_dbsync_queueMsg(DBSYNC_TYPE_FULL, DBSYNC_DATATYPE_ALL, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
                        }
                        #endif
                    }
                } while (cs_hashtable_iterator_advance(&itr));
            }
            (void)vs_unlock(&smRecords.smLock);
        }
    }
    IB_EXIT(__func__, status);
    return status;
}

Status_t sm_dbsync_syncMCRoot(DBSyncType_t synctype) {
    Status_t        status=VSTATUS_OK;
    SmRecp          smrecp;
    CS_HashTableItr_t itr;
    SMSyncData_t    syncData={0};

    IB_ENTER(__func__, synctype, 0, 0, 0);

    /* do nothing if sync is not turned on or we are exiting */
    if (!sm_config.db_sync_interval || dbsync_main_exit) return VSTATUS_OK;
    if (sizeof(IB_GID) > sizeof(SMSyncData_t)) {
        status = VSTATUS_NOMEM;
        IB_LOG_ERROR_FMT(__func__, "SMSyncData_t type not large enough");
    } else {
        if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
        } else {
            if (cs_hashtable_count(smRecords.smMap) > 1) {
                cs_hashtable_iterator(smRecords.smMap, &itr);
                do {
                    smrecp = cs_hashtable_iterator_value(&itr);
                    /* request sync of MC Root switch for SMs in good standing */
                    if (smrecp->portguid != sm_smInfo.PortGUID &&    /* make sure it's not us */ 
                        smrecp->syncCapability == DBSYNC_CAP_SUPPORTED && 
                        smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                        if (smrecp->dbsync.groupSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                            (void) sm_dbsync_queueMsg(synctype, DBSYNC_DATATYPE_MCROOT, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
                        } 
                    }
                } while (cs_hashtable_iterator_advance(&itr));
            }
            (void)vs_unlock(&smRecords.smLock);
        }
    }
    IB_EXIT(__func__, status);
    return status;
}

/*
 * sync a file with standby SMs
*/
Status_t sm_dbsync_syncFile(DBSyncType_t synctype, SMDBSyncFile_t *syncFile) {
    Status_t        status=VSTATUS_OK;
    SmRecp          smrecp;
    CS_HashTableItr_t itr;
    SMSyncData_t    syncData={0};

    IB_ENTER(__func__, synctype, syncFile, 0, 0);

    /* do nothing if sync is not turned on or we are exiting */
    if (!sm_config.db_sync_interval || dbsync_main_exit) return VSTATUS_OK;
    if ((sizeof(SMDBSyncFile_t)) > sizeof(SMSyncData_t)) {
        status = VSTATUS_NOMEM;
        IB_LOG_ERROR_FMT(__func__, "SMSyncData_t type not large enough");
    } else {
        if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
            IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
        } else {
            /* must have Sms besides us */
            if (cs_hashtable_count(smRecords.smMap) > 1) {
                if (syncFile) {
					 memcpy(syncData, (void *)syncFile, sizeof(SMDBSyncFile_t));
				}
                cs_hashtable_iterator(smRecords.smMap, &itr);
                do {
                    smrecp = cs_hashtable_iterator_value(&itr);
                    /* request sync of the service record for standby SMs in good standing */
                    if (smrecp->portguid != sm_smInfo.PortGUID && 
                        smrecp->syncCapability == DBSYNC_CAP_SUPPORTED && 
                        smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                        if (smrecp->dbsync.serviceSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
                            (void) sm_dbsync_queueMsg(synctype, DBSYNC_DATATYPE_FILE, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
                        } 
/*
					// even allow a SM that has been set Inactive to receive a new file
                    } else if (smrecp->portguid != sm_smInfo.PortGUID && smrecp->configDiff) {
						(void) sm_dbsync_queueMsg(synctype, DBSYNC_DATATYPE_FILE, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
*/
					}
		
                } while (cs_hashtable_iterator_advance(&itr));
            }
            (void)vs_unlock(&smRecords.smLock);
        }
    }
    IB_EXIT(__func__, status);
    return status;
}

Status_t sm_dbsync_syncDatelineSwitchGUID(DBSyncType_t synctype) {
	Status_t	status=VSTATUS_OK;
	SmRecp		smrecp;
	CS_HashTableItr_t itr;
	SMSyncData_t	syncData={0};

	IB_ENTER(__func__, synctype, 0, 0, 0);

	/* do nothing if sync is not turned on or we are exiting */
	if (!sm_config.db_sync_interval || dbsync_main_exit) return VSTATUS_OK;
	if ((sizeof(uint64_t)) > sizeof(SMSyncData_t)) {
		status = VSTATUS_NOMEM;
		IB_LOG_ERROR_FMT(__func__, "SMSyncData_t type not large enough");
	} else {
		if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
		} else {
			if (cs_hashtable_count(smRecords.smMap) > 1) {
				cs_hashtable_iterator(smRecords.smMap, &itr);
				do {
					smrecp = cs_hashtable_iterator_value(&itr);
					if (smrecp->portguid != sm_smInfo.PortGUID &&
						smrecp->syncCapability == DBSYNC_CAP_SUPPORTED &&
						smrecp->dbsync.fullSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
						if (smrecp->dbsync.serviceSyncStatus == DBSYNC_STAT_SYNCHRONIZED) {
							(void) sm_dbsync_queueMsg(synctype, DBSYNC_DATATYPE_DATELINE_GUID, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);
						}
					}
				} while (cs_hashtable_iterator_advance(&itr));
			}
			(void)vs_unlock(&smRecords.smLock);
		}
	}
	IB_EXIT(__func__, status);
	return status;
}

/*
 * sync a PA image data or a history file with all standby SMs. Needed for PA Fail over feature.
 */
Status_t sm_dbsync_queuePmFile(SmRecp smrecp, DBSyncType_t synctype, SMSyncData_t syncData) {
    Status_t        status=VSTATUS_OK;

	if (smDebugPerf)
		IB_LOG_INFINI_INFO_FMT(__func__, "BEGIN: Queuing PM DBSYNC_DATATYPE_FILE request: lid=0x%x", smrecp->lid);

	if (smrecp->pmdbsync.firstUpdateState != DBSYNC_STAT_SYNCHRONIZED) {
		if (smrecp->pmdbsync.firstUpdateState == DBSYNC_STAT_INPROGRESS) {
			if (smDebugPerf)
				IB_LOG_INFINI_INFO_FMT(__func__, "ABORT: Queuing PM DBSYNC_DATATYPE_FILE request, already in progress");
			return status;
		}
		smrecp->pmdbsync.firstUpdateState = DBSYNC_STAT_INPROGRESS;
	}
	(void) sm_dbsync_queueMsg(synctype, DBSYNC_DATATYPE_FILE, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, syncData);

	if (smDebugPerf)
		IB_LOG_INFINI_INFO_FMT(__func__, "COMPLETED: Queuing PM DBSYNC_DATATYPE_FILE request");

    return status;
}

/*
 * sync a PA image data or a history file with standby SMs. If portguid is specified, send to only that SM.
 */
Status_t sm_dbsync_pmdata_standby(DBSyncType_t synctype, SMSyncData_t syncData, uint64_t portguid) {
	Status_t        status=VSTATUS_OK;
	SmRecp          smrecp;
	CS_HashTableItr_t itr;

	IB_ENTER(__func__, synctype, syncData, 0, 0);

	/* do nothing if sync is not turned on or we are exiting */
	if (!sm_config.db_sync_interval || dbsync_main_exit) return VSTATUS_OK;

	if ((status = vs_lock(&smRecords.smLock)) != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
		return VSTATUS_BAD;
	}
	if (portguid) {
		if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &portguid)) != NULL) {
			(void)sm_dbsync_queuePmFile(smrecp, synctype, syncData);
		}
	} else {
		/* must have SMs besides us */
		if (cs_hashtable_count(smRecords.smMap) > 1) {
			cs_hashtable_iterator(smRecords.smMap, &itr);
			do {
				smrecp = cs_hashtable_iterator_value(&itr);
				if (smrecp->portguid != sm_smInfo.PortGUID) { // Don't send back to us
					(void)sm_dbsync_queuePmFile(smrecp, synctype, syncData);
				}
			} while (cs_hashtable_iterator_advance(&itr));
		}
	}
	(void)vs_unlock(&smRecords.smLock);

	IB_EXIT(__func__, status);
	return status;
}

// send the PM Sweep image to all standby SM's. Needed for PA fail over feature.
int sm_send_pm_image(uint64_t portguid, uint32_t histindex, char *histImgFile) {
	
	SMSyncData_t syncData;

	if ((sizeof(SMDBSyncFile_t)) > sizeof(SMSyncData_t)) {
		IB_LOG_ERROR_FMT(__func__, "SMSyncData_t type not large enough");
		return VSTATUS_NOMEM;
	}

	if (sm_state != SM_STATE_MASTER) {
		return -VSTATUS_BAD;
	}

	memset (syncData, 0, sizeof (SMSyncData_t));

	SMDBSyncFilep syncFile = (SMDBSyncFilep)syncData;

	syncFile->version = DBSYNC_FILE_TRANSPORT_VERSION;
	syncFile->length = sizeof(SMDBSyncFile_t);
	if (histImgFile) {
		strncpy(syncFile->name, histImgFile, sizeof(syncFile->name)-1);
		syncFile->name[sizeof(syncFile->name)-1] = 0;
		syncFile->type = DBSYNC_PM_HIST_IMAGE;
	}
	else {
		syncFile->name[0]='\0';
		syncFile->type = DBSYNC_PM_SWEEP_IMAGE;
	}
	syncFile->activate = histindex;

	sm_dbsync_pmdata_standby(DBSYNC_TYPE_BROADCAST_FILE, syncData, portguid);

	return VSTATUS_OK;
}

// On completion of sending a PM image file
int sm_send_pm_image_complete(uint64_t portguid, Status_t status) {
	SmRecp          smrecp;


	if (vs_lock(&smRecords.smLock) != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Can't lock SM Record table, rc: %d", status);
		return VSTATUS_BAD;
	}
	if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &portguid)) != NULL) {
		if (smrecp->pmdbsync.firstUpdateState != DBSYNC_STAT_SYNCHRONIZED) {
			if (status == VSTATUS_OK) {
				smrecp->pmdbsync.firstUpdateState = DBSYNC_STAT_SYNCHRONIZED;
			} else {
				smrecp->pmdbsync.firstUpdateState = DBSYNC_STAT_FAILURE;
			}
		}
	}
	(void)vs_unlock(&smRecords.smLock);

	return status;
}

/*
 * get the number of SM's currently in fabric
*/
int sm_dbsync_getSmCount(void) {
    int smcount=0;

    if (dbsync_initialized_flag && !dbsync_main_exit) {
        if (vs_lock(&smRecords.smLock)) {
            IB_LOG_ERROR0("Can't lock SM Record table");
        } else {
            smcount = cs_hashtable_count(smRecords.smMap);
            (void)vs_unlock(&smRecords.smLock);
        }
    }
    return smcount;
}

/*
 * get the number of active SM's on the fabric
*/
int sm_dbsync_getActiveSmCount(void) {
	SmRecp          smrecp;
	CS_HashTableItr_t itr;
	int             smcount=0;
	int             smactive=0;

	if (vs_lock(&smRecords.smLock)) {
        IB_LOG_ERROR0("Can't lock SM Record table");
    } else {
		smcount = cs_hashtable_count(smRecords.smMap);
		if (smcount) {
			cs_hashtable_iterator(smRecords.smMap, &itr);
			do {
				smrecp = cs_hashtable_iterator_value(&itr);
				//printf("*************** sm_dbsync_getActiveSmCount() smrecp %x diff %u\n", (unsigned int)smrecp, smrecp->configDiff);
				if (!smrecp->configDiff)
					smactive++;
			} while (cs_hashtable_iterator_advance(&itr));
		}
      	(void)vs_unlock(&smRecords.smLock);
	}
	//printf("************** sm_dbsync_getActiveSmCount() %u\n", smactive);
	return smactive;
}

/*
 * display the current SM's on the fabric
*/
void sm_dbsync_showSms(void) {
    SmRecp          smrecp;
    CS_HashTableItr_t itr;
    int             smcount=0;

    if (!dbsync_initialized_flag || dbsync_main_exit) {
        sysPrintf("SM is not currently running on this node\n");
        return;
    }

    sysPrintf("\n");          
	sysPrintf("-----SM DB SYNCHRONIZATION interval set to %d minute(s)", (int)(sm_config.db_sync_interval/60));

	if (vs_lock(&smRecords.smLock)) {
		sysPrintf("-----\n");
        IB_LOG_ERROR0("Can't lock SM Record table");
    } else {
        smcount = cs_hashtable_count(smRecords.smMap);
		/* PR 111477 - standby SMs only have their own information in smRecords.smMap
		 * and not other SMs [refer sm_dbsync_upsmlist()]. Hence standby SMs
		 * do not have the correct count of SMs in fabric, only master will
		 * have the correct count.
		 */
		if (sm_smInfo.u.s.SMStateCurrent == SM_STATE_MASTER)
		       sysPrintf(", %d SM(s) in fabric", smcount);

		sysPrintf("-----\n");

        if (smcount)
        {
            cs_hashtable_iterator(smRecords.smMap, &itr);
            do {
                smrecp = cs_hashtable_iterator_value(&itr);
                /* dump the sm records */
                sysPrintf("%s SM node at %s, LID 0x%08X, PortGuid "FMT_U64"\n", 
                          sm_getStateText(smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent), smrecp->nodeDescString, 
                          (int)smrecp->smInfoRec.RID.LID, smrecp->smInfoRec.SMInfo.PortGUID);
                sysPrintf("     Sync Capability is  %s\n", getSyncCapText(smrecp->syncCapability));
                /* don't display anymore if master */
                if (smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent == SM_STATE_MASTER) continue;
                if (smrecp->syncCapability == DBSYNC_CAP_SUPPORTED && smrecp->dbsync.fullSyncStatus != DBSYNC_STAT_UNINITIALIZED) {
                    sysPrintf("     Full sync status is      %s\n", getSyncStatText(smrecp->dbsync.fullSyncStatus));
                    if ( (smrecp->dbsync.fullSyncStatus > DBSYNC_STAT_UNINITIALIZED && smrecp->dbsync.fullTimeLastSync )) {
                        sysPrintf("     Time of last Full sync is %s", getSmSyncTime(smrecp->dbsync.fullTimeLastSync));
                    }
                    if (smrecp->dbsync.fullSyncFailCount) {
                        sysPrintf("     Full sync consecutive failure count is     %d\n", (int)smrecp->dbsync.fullSyncFailCount);
                        sysPrintf("     Time of last Full sync failure is   %s", getSmSyncTime(smrecp->dbsync.fullTimeSyncFail));
                    }
                    /* INFORM status */
                    if ( (smrecp->dbsync.informSyncStatus > DBSYNC_STAT_UNINITIALIZED && smrecp->dbsync.informTimeLastSync )) {
                        sysPrintf("     Time of last INFORM records sync is %s", getSmSyncTime(smrecp->dbsync.informTimeLastSync));
                    }
                    if (smrecp->dbsync.informSyncFailCount) {
                        sysPrintf("     INFORM sync consecutive failure count is     %d\n", (int)smrecp->dbsync.informSyncFailCount);
                        sysPrintf("     Time of last INFORM sync failure is   %s", getSmSyncTime(smrecp->dbsync.informTimeSyncFail));
                    }
                    /* GROUP status */
                    if ( (smrecp->dbsync.groupSyncStatus > DBSYNC_STAT_UNINITIALIZED && smrecp->dbsync.groupTimeLastSync ))
                        sysPrintf("     Time of last GROUP records sync is %s", getSmSyncTime(smrecp->dbsync.groupTimeLastSync));
                    if (smrecp->dbsync.groupSyncFailCount) {
                        sysPrintf("     INFORM sync consecutive failure count is     %d\n", (int)smrecp->dbsync.groupSyncFailCount);
                        sysPrintf("     Time of last GROUP sync failure is   %s", getSmSyncTime(smrecp->dbsync.groupTimeSyncFail));
                    }
                    /* SERVICE status */
                    if ( (smrecp->dbsync.serviceSyncStatus > DBSYNC_STAT_UNINITIALIZED && smrecp->dbsync.serviceTimeLastSync ))
                        sysPrintf("     Time of last SERVICE records sync is %s", getSmSyncTime(smrecp->dbsync.serviceTimeLastSync));
                    if (smrecp->dbsync.serviceSyncFailCount) {
                        sysPrintf("     SERVICE sync consecutive failure count is     %d\n", (int)smrecp->dbsync.serviceSyncFailCount);
                        sysPrintf("     Time of last SERVICE sync failure is   %s", getSmSyncTime(smrecp->dbsync.serviceTimeSyncFail));
                    }
                    /* DATELINE SWITCH GUID status */
                    if ( (smrecp->dbsync.datelineGuidSyncStatus > DBSYNC_STAT_UNINITIALIZED && smrecp->dbsync.datelineGuidTimeLastSync ))
                        sysPrintf("    Time of last DATELINE SWITCH GUID records sync in %s", getSmSyncTime(smrecp->dbsync.datelineGuidTimeLastSync));
                    if (smrecp->dbsync.datelineGuidSyncFailCount) {
                        sysPrintf("    DATELINE SWITCH GUID sync consecutive failure count is      %d\n", (int)smrecp->dbsync.datelineGuidSyncFailCount);
                        sysPrintf("    Time of last DATELINE SWITCH GUID sync failure is   %s", getSmSyncTime(smrecp->dbsync.datelineGuidTimeSyncFail));
                    }
                }
                sysPrintf("\n");          
            } while (cs_hashtable_iterator_advance(&itr));
        }
        (void)vs_unlock(&smRecords.smLock);
    }
    return;
}

void sm_dbsync_standbyHello(SmRecKey_t recKey) {
	SmRecp          smrecp = NULL;

	IB_ENTER(__func__, recKey, 0, 0, 0);

	if (dbsync_main_exit) return;

	if (vs_lock(&smRecords.smLock) != VSTATUS_OK) {
		IB_LOG_ERROR0("Can't lock SM Record table");
		return;
	}

	if ((smrecp = (SmRecp)cs_hashtable_search(smRecords.smMap, &recKey)) != NULL) {

		if (sm_config.monitor_standby_enable)
			(void)vs_time_get(&smrecp->lastHello);

		if (smrecp->syncCapability == DBSYNC_CAP_UNKNOWN) {
			smrecp->syncCapability = DBSYNC_CAP_ASKING;
			(void) sm_dbsync_queueMsg(DBSYNC_TYPE_GET_CAPABILITY, DBSYNC_DATATYPE_NONE, smrecp->lid, smrecp->portguid, smrecp->isEmbedded, NULL);
			IB_LOG_INFO_FMT(__func__, 
		                    "queued request for sync capability of SM node at Lid 0x%x, portGuid "FMT_U64" to SM table",
		                    smrecp->lid, smrecp->portguid);
		}
	}

	(void)vs_unlock(&smRecords.smLock);
}

void sm_dbsync_standbyCheck(void) {
	SmRecp          smrecp = NULL;
	SmRecKeyp       smreckeyp;
	uint64_t		now;
	CS_HashTableItr_t itr;

	IB_ENTER(__func__, 0, 0, 0, 0);

	if (dbsync_main_exit) return;

	/* lock out SM record table */
	if (vs_lock(&smRecords.smLock)) {
		IB_LOG_ERROR0("Can't lock SM Record table");
		return;
	} 

	(void)vs_time_get(&now);

	if (cs_hashtable_count(smRecords.smMap) > 1) {
		cs_hashtable_iterator(smRecords.smMap, &itr);
	    do {
			smreckeyp = cs_hashtable_iterator_key(&itr);
			smrecp = cs_hashtable_iterator_value(&itr);
			if (smrecp->portguid != sm_smInfo.PortGUID &&    /* make sure it's not us */ 
				smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent == SM_STATE_STANDBY &&  /* is a standby sm */
				smrecp->lastHello) {

				if ((smrecp->lastHello > 0) &&
					((now - smrecp->lastHello) > (sm_masterCheckInterval * sm_config.master_ping_max_fail))) {
					IB_LOG_INFINI_INFO_FMT(__func__, 
						"triggering a sweep, no activity from remote standby SM node at Lid 0x%x, portGuid "FMT_U64"",
						smrecp->lid, *smreckeyp);
					sm_trigger_sweep(SM_SWEEP_REASON_SECONDARY_TROUBLE);
					break;
				}
			}
		} while (cs_hashtable_iterator_advance(&itr));
	}
	(void)vs_unlock(&smRecords.smLock);
}

