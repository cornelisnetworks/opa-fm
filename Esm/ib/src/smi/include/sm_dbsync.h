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
//
// FILE NAME
//    sm_dbsync.h
//
// DESCRIPTION
//    This file contains the localized SM data base synchronization definitions.
//
// DATA STRUCTURES
//    NONE
//
// FUNCTIONS
//    NONE
//
// DEPENDENCIES
//    ib_types.h
//    ib_mad.h
//    ib_status.h
//
//
//===========================================================================//
#ifndef	_SM_DBSYNC_H_
#define	_SM_DBSYNC_H_

#include "os_g.h"

//
// SM DBSYNC structure conversion routines
// 
#define SMDBSYNC_NSIZE 84
#define SMDBSYNC_CCC_NSIZE 56
#define SMDBSYNC_SEND_FILE_NSIZE 100

#define SM_DBSYNC_VERSION   		1
#define SM_DBSYNC_UNKNOWN   		0xffffffff

typedef enum {
    DBSYNC_AID_SYNC = 1,
    DBSYNC_AID_GROUP,
    DBSYNC_AID_SERVICE,
    DBSYNC_AID_INFORM,
    DBSYNC_AID_MCROOT,
    DBSYNC_AID_DATELINE
} DBSyncAid_t;              /* Attribute ID (or cmd) to send */

typedef enum {
    DBSYNC_AMOD_GET = 1,    /* get the contents of a dataset */
    DBSYNC_AMOD_SET,        /* replace entire dataset with input */
    DBSYNC_AMOD_ADD,        /* add (or update existing) a record to the dataset */
    DBSYNC_AMOD_DELETE,     /* delete a record from the dataset */
    DBSYNC_AMOD_GET_CCC,    /* get the config consistency check data */
    DBSYNC_AMOD_SEND_FILE,  /* send a file to standby SM */
    DBSYNC_AMOD_RECONFIG    /* request standby SM read local config file */
} DBSyncAmod_t;             /* modifier to AID (cmd) to send */

typedef enum { 
    DBSYNC_CAP_UNKNOWN = 0,
    DBSYNC_CAP_ASKING,
    DBSYNC_CAP_NOTSUPPORTED, 
    DBSYNC_CAP_SUPPORTED 
} DBSyncCap_t;

typedef enum { 
    DBSYNC_STAT_UNINITIALIZED = 0, 
    DBSYNC_STAT_INPROGRESS, 
    DBSYNC_STAT_SYNCHRONIZED, 
    DBSYNC_STAT_FAILURE
} DBSyncStat_t;

typedef enum {
    DBSYNC_TYPE_GET_CAPABILITY = 1,
    DBSYNC_TYPE_FULL, 
    DBSYNC_TYPE_UPDATE,
    DBSYNC_TYPE_DELETE,
	DBSYNC_TYPE_BROADCAST_FILE,
	DBSYNC_TYPE_RECONFIG
} DBSyncType_t;

typedef enum {
    DBSYNC_DATATYPE_NONE,
    DBSYNC_DATATYPE_ALL, 
    DBSYNC_DATATYPE_INFORM, 
    DBSYNC_DATATYPE_GROUP, 
    DBSYNC_DATATYPE_SERVICE,
    DBSYNC_DATATYPE_MCROOT,
	DBSYNC_DATATYPE_FILE,
	DBSYNC_DATATYPE_DATELINE_GUID
} DBSyncDatTyp_t;

/*
 * Fabric SM's hash table
 */
 typedef struct {
     CS_HashTablep  smMap;          /* pointer to hash map of SM's currently in fabric */
	 SMDBCCCSync_t  ourChecksums;
     Lock_t         smLock;         /* exclusive lock for when manipulating hash table */
 } SmTable_t;

 typedef struct {
     DBSyncStat_t    syncStatus;     /* 0=un-initialized, 1=in progress, 2=synchronized, 3=failure */
     uint32_t        syncFailCount;  /* count of consecutive sync attempt failures */
     time_t          timeSyncFail;   /* time of last sync attempt failure */
     time_t          timeLastSync;   /* time of last scheduled sync - 0 if never sync'd */
 } SmSyncStat_t;
typedef SmSyncStat_t *SmSyncStatp;     /* pointer to sync status structure */

typedef struct {
	uint8   firstUpdateState;                // present state
} PmDbSync_t;
typedef PmDbSync_t *PmDbSyncp;

#define DBSYNC_MAX_GET_CAP_ATTEMPTS 2  /* try the GET a few times before saying dbsync not supported by SM */
#define DBSYNC_MAX_FAIL_SYNC_ATTEMPTS 3  /* try to sync a few times before giving up */
#define DBSYNC_TIME_BETWEEN_SYNC_FAILURES 60    /* retry interval if were in failure status */

typedef struct {
    uint64_t        portguid;       /* port guid of SM node (used as hash key) */
    uint32_t        lid;            /* lid of SM - may not be set yet */
    DBSyncCap_t     syncCapability; /* 0=unknown, 1=not supported, 2=supported */

    SMDBSync_t      dbsync;         /* db sync data of SM node */
    SMDBCCCSync_t   dbsyncCCC;      /* db sync CCC data of SM node */
	PmDbSync_t		pmdbsync;		/* db sync data of the PM node */
	uint8_t			configDiff;		/* is the config different from master */
    STL_SMINFO_RECORD  smInfoRec;      /* smInfoRecs of SM instance */
    uint8_t			nodeDescString[ND_LEN+1]; /* node description of SM node */
    uint8_t         path[64];       /* directed path to SM node */
	uint8_t			portNumber;		/* port Number of remote node - used only for reporting */
	uint8_t			isEmbedded;		/* 1: SM is embedded */
	uint64_t		lastHello;		/* if standby, last hello check */
} SmRec_t;
typedef SmRec_t     *SmRecp;        /* sm record pointer type */
typedef uint64_t    SmRecKey_t;     /* sm record key type */
typedef SmRecKey_t  *SmRecKeyp;     /* pointer to sm record key */

/* FIXME PR123899 - convert felib to STL sized MADs. */
typedef uint8_t     SMSyncData_t[200];  /* opaque holder of data to sync */

typedef struct {
    DBSyncType_t    type;           /* 1=get capability, 2=full sync, 3=update, 4=delete */
    DBSyncDatTyp_t  datatype;       /* 1=all (full sync only), 2=InformInfo, 3=groups, 4=services */
    uint64_t        portguid;       /* portguid of standby SM */
    STL_LID           standbyLid;     /* lid of the standby SM */
	uint8_t			isEmbedded;		/* 1: standby SM is embedded */
    SMSyncData_t    data;           /* opaque data buffer to hold specific sync data */
}SMSyncReq_t;
typedef SMSyncReq_t *SMSyncReqp;    /* Sm Sync Request pointer type */

//
// Multicast dbsync structure
//

typedef	struct {
	uint8_t			mGid[16];
	uint32_t		members_full;
	uint32_t		qKey;
	uint16_t		pKey;
	STL_LID			mLid;
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

//**********  SM topology asynchronous send receive response queue **********
extern  cs_Queue_ptr sm_dbsync_queue;
#define SM_DBSYNC_QUEUE_SIZE  128

/* SM table */
extern SmTable_t   smRecords;
extern int dbsync_main_exit;                /* dbsync thread exit flag */
extern int dbsync_initialized_flag;         /* dbsync initialized indicator */

static __inline
void
BSWAPCOPY_SM_DBSYNC_DATA(SMDBSyncp Src, SMDBSyncp Dest)
{
	memcpy(Dest, Src, sizeof(*Src));

    Dest->version = ntoh32(Dest->version);
    Dest->fullSyncStatus = ntoh32(Dest->fullSyncStatus);
    Dest->fullSyncFailCount = ntoh32(Dest->fullSyncFailCount);
    Dest->fullTimeSyncFail = ntoh32(Dest->fullTimeSyncFail);
    Dest->fullTimeLastSync = ntoh32(Dest->fullTimeLastSync);
    Dest->informSyncStatus = ntoh32(Dest->informSyncStatus);
    Dest->informSyncFailCount = ntoh32(Dest->informSyncFailCount);
    Dest->informTimeSyncFail = ntoh32(Dest->informTimeSyncFail);
    Dest->informTimeLastSync = ntoh32(Dest->informTimeLastSync);

    Dest->groupSyncStatus = ntoh32(Dest->groupSyncStatus);
    Dest->groupSyncFailCount = ntoh32(Dest->groupSyncFailCount);
    Dest->groupTimeSyncFail = ntoh32(Dest->groupTimeSyncFail);
    Dest->groupTimeLastSync = ntoh32(Dest->groupTimeLastSync);

    Dest->serviceSyncStatus = ntoh32(Dest->serviceSyncStatus);
    Dest->serviceSyncFailCount = ntoh32(Dest->serviceSyncFailCount);
    Dest->serviceTimeSyncFail = ntoh32(Dest->serviceTimeSyncFail);
    Dest->serviceTimeLastSync = ntoh32(Dest->serviceTimeLastSync);

	Dest->mcrootSyncStatus = ntoh32(Dest->mcrootSyncStatus);
    Dest->mcrootSyncFailCount = ntoh32(Dest->mcrootSyncFailCount);
    Dest->mcrootTimeSyncFail = ntoh32(Dest->mcrootTimeSyncFail);
    Dest->mcrootTimeLastSync = ntoh32(Dest->mcrootTimeLastSync);

    Dest->datelineGuidSyncStatus = ntoh32(Dest->datelineGuidSyncStatus);
    Dest->datelineGuidSyncFailCount = ntoh32(Dest->datelineGuidSyncFailCount);
    Dest->datelineGuidTimeSyncFail = ntoh32(Dest->datelineGuidTimeSyncFail);
    Dest->datelineGuidTimeLastSync = ntoh32(Dest->datelineGuidTimeLastSync);
}

static __inline
void
BSWAPCOPY_SM_DBSYNC_CCC_DATA(SMDBCCCSyncp Src, SMDBCCCSyncp Dest)
{
	memcpy(Dest, Src, sizeof(*Src));

    Dest->protocolVersion = ntoh32(Dest->protocolVersion);
    Dest->smVfChecksum = ntoh32(Dest->smVfChecksum);
    Dest->smConfigChecksum = ntoh32(Dest->smConfigChecksum);
    Dest->pmConfigChecksum = ntoh32(Dest->pmConfigChecksum);

    Dest->spare1 = ntoh32(Dest->spare1);
    Dest->spare2 = ntoh32(Dest->spare2);
    Dest->spare3 = ntoh32(Dest->spare3);
    Dest->spare4 = ntoh32(Dest->spare4);
    Dest->spare5 = ntoh32(Dest->spare5);
    Dest->spare6 = ntoh32(Dest->spare6);
    Dest->spare7 = ntoh32(Dest->spare7);
    Dest->spare8 = ntoh32(Dest->spare8);
}

static __inline
void
BSWAPCOPY_SM_DBSYNC_FILE_DATA(SMDBSyncFilep Src, SMDBSyncFilep Dest)
{
	memcpy(Dest, Src, sizeof(*Src));

    Dest->version = ntoh32(Dest->version);
    Dest->length = ntoh32(Dest->length);
    Dest->activate = ntoh32(Dest->activate);
    Dest->type = ntoh32(Dest->type);
    Dest->size = ntoh32(Dest->size);
    Dest->spare1 = ntoh32(Dest->spare1);
    Dest->spare2 = ntoh32(Dest->spare2);
    Dest->spare3 = ntoh32(Dest->spare3);
    Dest->spare4 = ntoh32(Dest->spare4);
}

static __inline
void
BSWAPCOPY_SM_DBSYNC_SUBSKEY_DATA(SubscriberKeyp Src, SubscriberKeyp Dest)
{
	memcpy(Dest, Src, sizeof(*Src));

    Dest->lid = ntoh16(Dest->lid);
    Dest->trapnum = ntoh16(Dest->trapnum);
    Dest->qpn = ntoh32(Dest->qpn);
    Dest->pkey = ntoh16(Dest->pkey);
    Dest->qkey = ntoh32(Dest->qkey);
    Dest->startLid = ntoh16(Dest->startLid);
    Dest->endLid = ntoh16(Dest->endLid);
}

static __inline
void
BSWAPCOPY_SM_DBSYNC_MC_GROUP_DATA(McGroupSync_t *Src, McGroupSync_t *Dest)
{
	memcpy(Dest, Src, sizeof(*Src));

    Dest->members_full = ntoh32(Dest->members_full);
    Dest->qKey = ntoh32(Dest->qKey);
    Dest->pKey = ntoh16(Dest->pKey);
    Dest->mLid = ntoh32(Dest->mLid);
    Dest->flowLabel = ntoh32(Dest->flowLabel);
    Dest->membercount = ntoh16(Dest->membercount);
    Dest->index_pool = ntoh32(Dest->index_pool);
}

static __inline
void
BSWAPCOPY_SM_DBSYNC_RECORD_CNT(uint32_t *Src, uint32_t *Dest)
{
    memcpy(Dest, Src, sizeof(*Src));

    *Dest = ntoh32(*Dest);
}

/*
 *  Sm dbsync prototypes
*/
void		sm_dbsync(uint32_t, uint8_t **);    /* sm db sync main function */
Status_t    sm_dbsync_filter_add(IBhandle_t fd, Filter_t *filter, uint8_t mclass, 
                                 uint8_t method, uint16_t aid, uint32_t amod, uint64_t tid, const char *subRoutine);
void        sm_dbsync_queueMsg(DBSyncType_t syncType, DBSyncDatTyp_t syncDataTye, STL_LID lid, Guid_t portguid, uint8_t isEmbedded, SMSyncData_t);
void        sm_dbsync_addSm(Node_t *, Port_t *, STL_SMINFO_RECORD *);
void        sm_dbsync_deleteSm(uint64_t);
void        sm_dbsync_updateSm(SmRecKey_t, STL_SM_INFO *);
Status_t    sm_dbsync_fullSyncCheck(SmRecKey_t);
Status_t    sm_dbsync_upSmDbsync(SmRecKey_t, SMDBSyncp);
Status_t    sm_dbsync_upSmDbsyncCap(SmRecKey_t, DBSyncCap_t);
Status_t    sm_dbsync_upSmDbsyncStat(SmRecKey_t, DBSyncDatTyp_t, DBSyncStat_t);
Status_t	sm_dbsync_configCheck(SmRecKey_t, SMDBCCCSyncp);
Status_t    sm_dbsync_disableStandbySm(SmRecp, SmCsmMsgCondition_t);
DBSyncStat_t sm_dbsync_getSmDbsyncStat(SmRecKey_t);
Status_t    sm_dbsync_getSmDbsync(SmRecKey_t, SMDBSyncp);
boolean     sm_dbsync_isUpToDate(SmRecKey_t, char **reason);
DBSyncCap_t sm_dbsync_getDbsyncSupport(SmRecKey_t);
Status_t    sm_dbsync_checksums(uint32_t vfCsum, uint32_t smCsum, uint32_t pmCsum);
Status_t    sm_dbsync_recInit(void);
void        sm_dbsync_kill(void);
void        sm_dbsync_init(void);
void        sm_dbsync_recDelete(void);
void        sm_dbsync_recClear(void);
void        sm_dbsync_procReq(void);
int         sm_dbsync_getSmCount(void);
int         sm_dbsync_getActiveSmCount(void);
Status_t    sm_dbsync_upsmlist(void);
Status_t    sm_dbsync_syncService(DBSyncType_t, OpaServiceRecordp);
Status_t    sm_dbsync_syncInform(DBSyncType_t, SubscriberKeyp, STL_INFORM_INFO_RECORD *);
Status_t    sm_dbsync_syncGroup(DBSyncType_t, IB_GID*);
Status_t	sm_dbsync_syncMCRoot(DBSyncType_t synctype);
Status_t	sm_dbsync_syncFile(DBSyncType_t synctype, SMDBSyncFile_t*);
Status_t	sm_dbsync_syncDatelineSwitchGUID(DBSyncType_t);
void        sm_dbsync_showSms(void);
void		sm_dbsync_standbyHello(SmRecKey_t);
void		sm_dbsync_standbyCheck(void);
int 		sm_send_pm_image(uint64_t portguid, uint32_t histindex, char *histImgFile);
int			sm_send_pm_image_complete(uint64_t portguid, Status_t status);
#endif	// _SM_DBSYNC_H_
