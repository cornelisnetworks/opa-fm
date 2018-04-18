/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

// TBD - keep a timestamp in Port_t indicating when we first found port active
// this will indicate uptime for the port.  TBD - if port bounces between PM
// sweeps uptime could be inaccurate.  Maybe SM needs to track this statistic
// and PM can report it?
//
// TBD - option to log when error counters exceed threshold, with limit on
// number of logs per sweep, etc
//
// TBD - add some logging to show:
// 		sweep time, number of nodes in cl_qmap, number of nodes in LidMap
// 		number of active ports in LidMap
// 		perhaps keep as statistics too (or maybe this duplicates group port
// 		counts already)

// TBD - Add some time tracking as in pmPerfDebug for various phases of sweep.

#include "sm_l.h"
#include "pm_l.h"
#include <iba/ibt.h>
#include "pm_topology.h"
#ifndef __VXWORKS__
#include "pa_sth_old.h"
#endif
#include "sm_dbsync.h"
#include "fm_xml.h"
#include <limits.h>
#include <string.h>
#include "zlib.h"
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>

#define ENABLE_PMA_SWEEP 1
#if defined(__VXWORKS__)
#define ENABLE_ENGINE 1
#else
#define ENABLE_ENGINE 1
#endif

#if defined(__VXWORKS__)
#define PM_ENGINE_STACK_SIZE (24 * 1024)
#define PM_ASYNC_RCV_STACK_SIZE (24 * 1024)
#define PM_DBSYNC_THREAD_STACK_SIZE (24 * 1024)
#else
#define PM_ENGINE_STACK_SIZE (256 * 1024)
#define PM_ASYNC_RCV_STACK_SIZE (256 * 1024)
#define PM_DBSYNC_THREAD_STACK_SIZE (256 * 1024)
#define PM_COMPRESS_THREAD_STACK_SIZE (4 * 1024)
#endif

extern uint16_t sm_masterPmSl;

uint32 s_StaticRateToMBps[IB_STATIC_RATE_MAX+1];

Pm_t g_pmSweepData;
int	g_pmEngineState = PM_ENGINE_STOPPED;
Thread_t g_pmEngineThread;
Event_t g_pmEngineShutdownEvent;
boolean g_pmEngineThreadRunning = FALSE;
boolean g_pmFirstSweepAsMaster = TRUE;
extern boolean isUnexpectedClearUserCounters;
CounterSelectMask_t LinkDownIgnoreMask;

Thread_t g_pmAsyncRcvThread;
boolean g_pmAsyncRcvThreadRunning = FALSE;
Sema_t g_pmAsyncRcvSema;	// indicates AsyncRcvThread is ready

// Max Compress/Decompress threads possible
#define PM_MAX_NUM_COMPRESSION_DIVISIONS 32
#define PM_HALF_SECOND ((uint64_t) 500000U)

// default Thresholds
ErrorSummary_t g_pmThresholds = {
    Integrity:  100,
    Congestion: 100,
    SmaCongestion: 100,
    Bubble: 100,
    Security: 10,
    Routing: 100
};

// default ThresholdsExceededMsgLimit
PmThresholdsExceededMsgLimitXmlConfig_t g_pmThresholdsExceededMsgLimit = {
    Integrity:  10,
    Congestion: 0,
    SmaCongestion: 0,
    Bubble: 0,
    Security: 10,
    Routing: 10
};

// default Weights for components of Integrity
IntegrityWeights_t g_pmIntegrityWeights = {
    LocalLinkIntegrityErrors: 0,
    PortRcvErrors: 100,
    ExcessiveBufferOverruns: 100,
    LinkErrorRecovery: 0,
    LinkDowned: 25,
    UncorrectableErrors: 100,
    FMConfigErrors: 100,
    LinkQualityIndicator: 40,
    LinkWidthDowngrade: 100
};

// default Weights for components of Congestion
CongestionWeights_t g_pmCongestionWeights = {
    PortXmitWait: 10,
    SwPortCongestion: 100,
    PortRcvFECN: 5,
    PortRcvBECN: 1,
    PortXmitTimeCong: 25,
    PortMarkFECN: 25
};

/* DN445 - PA Failover feature. Thread to sync RAM/Disk short term PA history data per SM. */
Thread_t              g_PmDbsyncThread;
boolean               g_PmDbsyncThreadRunning = FALSE;
static boolean        pmDbsync_exit = 0;
#define               MAX_NUMBER_SMS_FOR_PMDBSYNC 20
typedef struct PmDbsyncSmInfo_s {
	uint32	NumStandbySMs;            // count of Standby SMs
	uint32	ChangeinStandbySMs;       // Detected a change in Standby SMs
	uint32	sweepNumLastSweepImageSent;  // sweep number of the last sweep image that was sent to all Standby SMs.
	struct DbsyncSmInfo {
		uint64	PortGUID;               // PortGUID in SMInfo
		STL_LID	smLid;                  // implies port, 0 if empty record
		uint8	priority:4;             // present priority
		uint8	state:4;                // present state
        DBSyncCap_t	syncCapability;         // 0=unknown, 1=not supported, 2=supported
		uint8	numRImagesForSync;      // Number of RAM images to be sent to Standby SM
		uint8	rImgPmDbsyncNeeded;     // 0-no RAM image sync operations needed. 1-sync needed.
		uint8	histPmDbsyncNeeded;     // 0-no PM history sync operations needed. 1-sync needed.
		char	histImgListFile[300];   // name of file that contains list of history image file names
		uint32	histImgFileIndex;       // history image file index
		uint32	numSweepImagesSent;     // Number of latest sweep images sent to Standby SM from point when it was discovered.
		uint32	numHistImagesSent;      // Number of history images sent to Standby SM from point when it was discovered.
		uint32	sweepNumHistImageSent;  // Sweep number of last history image sent to Standby SM.
	} SMs[MAX_NUMBER_SMS_FOR_PMDBSYNC];
} PmDbsyncSmInfo_t;
PmDbsyncSmInfo_t      basePmDbsyncSmInfo;
PmDbsyncSmInfo_t      newPmDbsyncSmInfo;

/* Helper function to check if any add/delete changes happened to standby SMs. */
static void checkForChangesInStandbySMs(PmDbsyncSmInfo_t *base, PmDbsyncSmInfo_t *new)
{
	uint32    newIndex, baseIndex;
	STL_LID   smLid;

	if ((new->NumStandbySMs == 0) && (base->NumStandbySMs == 0)) {
		/* no standby SMs; nothing to do. */
		return;
	}
	if ((new->NumStandbySMs == 1) && (base->NumStandbySMs == 0)) {
		new->ChangeinStandbySMs = TRUE;
		newIndex=0;
		new->SMs[newIndex].rImgPmDbsyncNeeded = TRUE;
		new->SMs[newIndex].numRImagesForSync = (g_pmSweepData.NumSweeps<pm_config.total_images)?
												g_pmSweepData.NumSweeps : pm_config.total_images;
		new->SMs[newIndex].histPmDbsyncNeeded = FALSE; /* Set to TRUE, if short term PA hist feature is enabled && after RAM resident images are synced up */
		new->SMs[newIndex].histImgFileIndex = 0;
		new->SMs[newIndex].numSweepImagesSent = 0;
		new->SMs[newIndex].numHistImagesSent = 0;
		new->SMs[newIndex].sweepNumHistImageSent = 0;
		IB_LOG_VERBOSE_FMT(__func__, "One New Standby SM lid 0x%x at Index=%d", new->SMs[newIndex].smLid, newIndex);
		goto update_base;
	}

	new->sweepNumLastSweepImageSent = base->sweepNumLastSweepImageSent;
	new->ChangeinStandbySMs = base->ChangeinStandbySMs;
	for (newIndex=0; newIndex<new->NumStandbySMs; newIndex++) {
		smLid = new->SMs[newIndex].smLid;
		if (!smLid) break; /* zero smLid marks end of list */
		/* see if that standby SM is present in base database */
		for (baseIndex=0; baseIndex<base->NumStandbySMs; baseIndex++) {
			if (!base->SMs[baseIndex].smLid) break;
			if (smLid == base->SMs[baseIndex].smLid) { /* copy status flags over */
				new->SMs[newIndex].histPmDbsyncNeeded = base->SMs[baseIndex].histPmDbsyncNeeded;
				new->SMs[newIndex].rImgPmDbsyncNeeded = base->SMs[baseIndex].rImgPmDbsyncNeeded;
				new->SMs[newIndex].numRImagesForSync = base->SMs[baseIndex].numRImagesForSync;
				cs_strlcpy(new->SMs[newIndex].histImgListFile, base->SMs[newIndex].histImgListFile, 300);
				new->SMs[newIndex].histImgFileIndex = base->SMs[newIndex].histImgFileIndex;
				new->SMs[newIndex].numSweepImagesSent = base->SMs[baseIndex].numSweepImagesSent;
				new->SMs[newIndex].numHistImagesSent = base->SMs[baseIndex].numHistImagesSent;
				new->SMs[newIndex].sweepNumHistImageSent = base->SMs[baseIndex].sweepNumHistImageSent;
				IB_LOG_VERBOSE_FMT(__func__, "SM lid 0x%x moved from baseIndex=%d to newIndex=%d", new->SMs[newIndex].smLid, baseIndex, newIndex);
				break; /* match found */
			}
			else if (baseIndex == (base->NumStandbySMs-1)) { /* a new standby SM discovered. */
				new->ChangeinStandbySMs = TRUE;
				new->SMs[newIndex].rImgPmDbsyncNeeded = TRUE;
				new->SMs[newIndex].numRImagesForSync = (g_pmSweepData.NumSweeps<pm_config.total_images)?
														g_pmSweepData.NumSweeps : pm_config.total_images;
				new->SMs[newIndex].histPmDbsyncNeeded = FALSE; /* Set to TRUE, if short term PA hist feature is enabled && after RAM resident images are synced up */
				new->SMs[newIndex].histImgFileIndex = 0;
				new->SMs[newIndex].numSweepImagesSent = 0;
				new->SMs[newIndex].numHistImagesSent = 0;
				new->SMs[newIndex].sweepNumHistImageSent = 0;
				IB_LOG_VERBOSE_FMT(__func__, "New Standby SM lid 0x%x at Index=%d", new->SMs[newIndex].smLid, newIndex);
			}
		}
	}
update_base:
	memcpy(base, new, sizeof(PmDbsyncSmInfo_t)); /* update the database for reference against next iteration. */
	return;
}

/* Obtain a new snapshot of standby SMs in the fabric.
 * Compare to the previous standby SM snapshot and mark appropriate flags for PM dbsync. 
 * If base_line snapshot, set the previous/base and new snapshots to same standby SM info.
 */
static void PmDbsyncSmInfo_copy(Pm_t *pm, boolean base_line)
{
	PmDbsyncSmInfo_t  *PmDbsyncSmInfo_p = &newPmDbsyncSmInfo;
	SmRecp            smrecp;
	CS_HashTableItr_t itr;

	/* 
	* Find all standby SMs in list with the proper SMKey.
	* Should be master when this function is called.
	*/
	memset(PmDbsyncSmInfo_p, 0, sizeof(PmDbsyncSmInfo_t));

	// TBD - sm_topology.c should lock this too?
	vs_lock(&smRecords.smLock);
	if (cs_hashtable_count(smRecords.smMap) > 1) { /* master and at least one standby SM */
		cs_hashtable_iterator(smRecords.smMap, &itr);
		do {
			smrecp = cs_hashtable_iterator_value(&itr);
			if (smrecp->portguid == sm_smInfo.PortGUID) {   /* this is Master entry */
				continue;
			}
			if (smrecp->smInfoRec.SMInfo.SM_Key != sm_smInfo.SM_Key) {
				continue;	// not a viable SM
			}
			if (smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent != SM_STATE_STANDBY) {
				continue;	// not a standby SM
			}
			if (smrecp->syncCapability != DBSYNC_CAP_SUPPORTED) {
				continue;	// not a standby SM
			}
			else { /* standby SM */

				/* histPmDbsyncNeeded etc flags will be set after comparing with basePmDbsyncSmInfo data */
				/* for base_line, they would all be zero. */

				PmDbsyncSmInfo_p->SMs[PmDbsyncSmInfo_p->NumStandbySMs].PortGUID = smrecp->portguid;
				PmDbsyncSmInfo_p->SMs[PmDbsyncSmInfo_p->NumStandbySMs].smLid = smrecp->smInfoRec.RID.LID;
				PmDbsyncSmInfo_p->SMs[PmDbsyncSmInfo_p->NumStandbySMs].priority = smrecp->smInfoRec.SMInfo.u.s.Priority;
				PmDbsyncSmInfo_p->SMs[PmDbsyncSmInfo_p->NumStandbySMs].state = smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent;
				PmDbsyncSmInfo_p->SMs[PmDbsyncSmInfo_p->NumStandbySMs].syncCapability = smrecp->syncCapability;
				PmDbsyncSmInfo_p->NumStandbySMs++; // counts Master or Standby SMs
				IB_LOG_VERBOSE_FMT(__func__, "NumStandbySMs=%d.", PmDbsyncSmInfo_p->NumStandbySMs);
				if (PmDbsyncSmInfo_p->NumStandbySMs == MAX_NUMBER_SMS_FOR_PMDBSYNC) {
					break;
				}
			}
		} while (cs_hashtable_iterator_advance(&itr));
	}
	else {
		PmDbsyncSmInfo_p->NumStandbySMs=0; // only Master; set Standby SMs to zero.
		PmDbsyncSmInfo_p->SMs[PmDbsyncSmInfo_p->NumStandbySMs].PortGUID = 0;
		PmDbsyncSmInfo_p->SMs[PmDbsyncSmInfo_p->NumStandbySMs].smLid = 0;
	}
	vs_unlock(&smRecords.smLock);
	IB_LOG_VERBOSE_FMT(__func__, "Done performing PmDbsyncSmInfo_copy()..");
	if (base_line) {
		memcpy(&basePmDbsyncSmInfo, PmDbsyncSmInfo_p, sizeof(PmDbsyncSmInfo_t));
	} else { /* check if any add/delete changes happened to standby SMs. */
		checkForChangesInStandbySMs(&basePmDbsyncSmInfo, PmDbsyncSmInfo_p);
		IB_LOG_VERBOSE_FMT(__func__, "Done performing checkForChangesInStandbySMs()..");
	}
	return;
}
 
static boolean isShortTermHistoryEnabled(void)
{
	return pm_config.shortTermHistory.enable;
}

static boolean isShortTermHistoryCompositeBufferEmpty(Pm_t *pm)
{
	if (!pm_config.shortTermHistory.enable) {
		return TRUE;	/* short term history feature is disabled. */
	}

	// if there is no image, or the current one has already been written
	if (!pm->ShortTermHistory.currentComposite ||
		pm->ShortTermHistory.compositeWritten) {
		return TRUE;
	}
	else {
		return FALSE;
	}
}

int i2a(char *s, int n){
	div_t qr;
	int pos;

	if(n == 0) return 0;

	qr = div(n, 10);
	pos = i2a(s, qr.quot);
	s[pos] = qr.rem + '0';
	return pos + 1;
}

char* my_itoa(char *output_buff, int num){
	char *p = output_buff;
	if(num < 0){
		*p++ = '-';
		num *= -1;
	} else if(num == 0)
		*p++ = '0';
	p[i2a(p, num)]='\0';
	return output_buff;
}

int hist_filter (const struct dirent *d) {
	// used by scandir to filter for .hist and .zhist files
	char *c = strchr(d->d_name, '.');
	if (c && (!strcmp(c, ".hist") || !strcmp(c, ".zhist"))) {
			return 1;
	}
	return 0;
}

#ifndef __VXWORKS__
/* Create a file snapshot that contains the list of history image files to be sent to smLid Standby SM */
/* File list name contains smLid to make it unique. Each file in the file list contains full path (latest to oldest sorted using file name). */
int createHistFileListForSM(char *histPath, STL_LID smLid, char *fileList)
{
	char suff[20];
	struct dirent **d;
	int n, ret = 0;

	if (!histPath || !fileList)
		return -1;

	snprintf(fileList, 120, "smLid%s", my_itoa(suff, smLid));

	n = scandir(histPath, &d, hist_filter, alphasort);

	block_sm_exit();
	if (n < 0) {
		ret = -1;
	} else {
		const size_t MAX_PATH = PM_HISTORY_FILENAME_LEN + 1 + 256;
		char path[MAX_PATH];
		FILE *fp;

		snprintf(path, MAX_PATH, "%s/%s", histPath, fileList);

		if ( !(fp = fopen(path, "w")) ) {
			ret = -1;
		} else {
			while (n--) {
				if ( fprintf(fp, "%s\n", d[n]->d_name) < 0) {
					ret = -1;
					break;
				}
				free(d[n]);
				d[n] = NULL;
			}
			fclose(fp);
		}
	}
	unblock_sm_exit();

	if (d) {
		int i;

		for (i = 0; i < n; i++)
			if (d[i]) free(d[i]);
		free(d);
	}

	return ret;
}
#endif

uint32 getSweepNumFromHistIndex(Pm_t *pm, uint32 lastHistoryIndex)
{
	if (pm->history[lastHistoryIndex] == PM_IMAGE_INDEX_INVALID) {
		return 0;
	}
	if (pm->Image[pm->history[lastHistoryIndex]].state != PM_IMAGE_VALID) {
		return 0;
	}
	return pm->Image[pm->history[lastHistoryIndex]].sweepNum;
}

/* gets fIndex number file name from the fileList file */
/* file name returned from the file list contains full path. */
void getHistFileName(char *histPath, char *fileList, int fIndex, char *file)
{
	FILE *fd;
	int num=-1;
	char tmpName[200];

	if (!file) {
		return;
	}
	file[0]='\0';
	if (!histPath || !fileList) {
		return;
	}
	tmpName[0]='\0';
	snprintf(tmpName, 200, "%s/%s", histPath, fileList);
	fd = fopen(tmpName, "r");
	if (fd) {
		do {
			if (fgets(file, 120, fd) != NULL) {
				file[strlen(file)-1]='\0'; /* get rid of new line char. */
				num++;
			}
			else {
				break;
			}
		} while (num != fIndex);
		if (num != fIndex) {
			file[0]='\0';
		}
		fclose(fd);
	}
	return;
}

static void PmDbsyncStatusFlagsUpdate(Pm_t *pm)
{
	PmDbsyncSmInfo_t  *PmDbsyncSmInfo_p = &basePmDbsyncSmInfo;
	uint32            smIndex;
	STL_LID           smLid;

	PmDbsyncSmInfo_p->ChangeinStandbySMs = FALSE;
	for (smIndex=0; smIndex<PmDbsyncSmInfo_p->NumStandbySMs; smIndex++) {
		smLid = PmDbsyncSmInfo_p->SMs[smIndex].smLid;
		if (!smLid) break; /* zero smLid marks end of list */
		if (PmDbsyncSmInfo_p->SMs[smIndex].numSweepImagesSent == 0) { /* still haven't started sending images for atleast one standby SM */
			PmDbsyncSmInfo_p->ChangeinStandbySMs = TRUE;
		}
		else {
			if (PmDbsyncSmInfo_p->SMs[smIndex].rImgPmDbsyncNeeded &&
				(PmDbsyncSmInfo_p->SMs[smIndex].numRImagesForSync <
				(PmDbsyncSmInfo_p->SMs[smIndex].numSweepImagesSent+PmDbsyncSmInfo_p->SMs[smIndex].numHistImagesSent))) {
				IB_LOG_VERBOSE_FMT(__func__, "Synced up RAM images %d.", PmDbsyncSmInfo_p->SMs[smIndex].numRImagesForSync);
				PmDbsyncSmInfo_p->SMs[smIndex].rImgPmDbsyncNeeded = FALSE; /* RAM resident images are synced up */

				#ifndef __VXWORKS__
				/* If short term history feature is enabled, initialize histImgListFile and histImgFileIndex here. */
				if (isShortTermHistoryEnabled() && !PmDbsyncSmInfo_p->SMs[smIndex].histImgFileIndex) {
					PmDbsyncSmInfo_p->SMs[smIndex].histPmDbsyncNeeded = TRUE;

					/* Initialize histImgListFile and histImgFileIndex */
					createHistFileListForSM(pm->ShortTermHistory.filepath,
											PmDbsyncSmInfo_p->SMs[smIndex].smLid,
											PmDbsyncSmInfo_p->SMs[smIndex].histImgListFile);
					IB_LOG_VERBOSE_FMT(__func__, "Hist image list file created %s.", PmDbsyncSmInfo_p->SMs[smIndex].histImgListFile);
					PmDbsyncSmInfo_p->SMs[smIndex].histImgFileIndex = 0;
				}
				#endif
			}
		}
	}
	IB_LOG_VERBOSE_FMT(__func__, "Done performing PmDbsyncStatusFlagsUpdate()..");
	return;
}

/* Helper function that determines which image needs to be sent to which standby SM etc. */
static void PmDbsyncOperation(Pm_t *pm)
{
	PmDbsyncSmInfo_t  *PmDbsyncSmInfo_p = &basePmDbsyncSmInfo;
	uint32            smIndex;
	STL_LID           smLid;
	uint32            numImagesSent;
	int               offset;
	char              histFile[300];

	/* Check if standby SMs are present? */
	if (!PmDbsyncSmInfo_p->NumStandbySMs) {
		IB_LOG_VERBOSE_FMT(__func__, "No standby SMs detected..Nothing to do.");
		return;
	}
	if (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID) {
		IB_LOG_VERBOSE_FMT(__func__, "Master hasn't started sweeps..Nothing to do.");
		return;
	}
	/* if a latest PM sweep image is available, send it to all standby SMs. */
	if (PmDbsyncSmInfo_p->sweepNumLastSweepImageSent < getSweepNumFromHistIndex(pm, pm->lastHistoryIndex)) {
		if (isShortTermHistoryEnabled() && PmDbsyncSmInfo_p->ChangeinStandbySMs) { /* a new standby SM was discovered */

			/* Short term PA history is enabled, need to check for composite buffer to be empty to start
				sending sweep images to the newly discovered standby SM. */
			for (smIndex=0; smIndex<MAX_NUMBER_SMS_FOR_PMDBSYNC; smIndex++) {
				smLid = PmDbsyncSmInfo_p->SMs[smIndex].smLid;
				if (!smLid) break; /* zero smLid marks end of list */
				if (PmDbsyncSmInfo_p->SMs[smIndex].numSweepImagesSent == 0) {
					/* Short term PA history is enabled, check for composite buffer to be empty */
					if (isShortTermHistoryCompositeBufferEmpty(pm)) {
						IB_LOG_VERBOSE_FMT(__func__, "Composite buffer is empty. Sending sweepnum %d image now..", getSweepNumFromHistIndex(pm, pm->lastHistoryIndex));
						sm_send_pm_image(PmDbsyncSmInfo_p->SMs[smIndex].PortGUID, pm->lastHistoryIndex, NULL);
						PmDbsyncSmInfo_p->SMs[smIndex].numSweepImagesSent++;
					}
					else { /* skip this SM. */
						IB_LOG_VERBOSE_FMT(__func__, "Composite buffer is NOT empty. Skipping SM..");
						if (PmDbsyncSmInfo_p->SMs[smIndex].numRImagesForSync < pm_config.total_images) {
							PmDbsyncSmInfo_p->SMs[smIndex].numRImagesForSync++; /* one more RAM image to send */
						}
						continue;
					}
				}
				else {/* this is a standby SM that has begun to receive sweep images. Sending latest image now. */
					IB_LOG_VERBOSE_FMT(__func__, "Already started sending sweeps. Sending image to SM..");
					sm_send_pm_image(PmDbsyncSmInfo_p->SMs[smIndex].PortGUID, pm->lastHistoryIndex, NULL);
					PmDbsyncSmInfo_p->SMs[smIndex].numSweepImagesSent++;
				}
			}
		}
		else { /* no change in SMs. Send the image to all existing SMs. */
			IB_LOG_VERBOSE_FMT(__func__, "New Sweep Image (sweepnum=%d) Sending it to all SMs", getSweepNumFromHistIndex(pm, pm->lastHistoryIndex));
			sm_send_pm_image(0 /* all standby SMs*/, pm->lastHistoryIndex, NULL);
			for (smIndex=0; smIndex<MAX_NUMBER_SMS_FOR_PMDBSYNC; smIndex++) {
				smLid = PmDbsyncSmInfo_p->SMs[smIndex].smLid;
				if (!smLid) break; /* zero smLid marks end of list */
				PmDbsyncSmInfo_p->SMs[smIndex].numSweepImagesSent++;
			}
		}
		PmDbsyncSmInfo_p->sweepNumLastSweepImageSent = getSweepNumFromHistIndex(pm, pm->lastHistoryIndex);
	}
	else { /* non-latest sweep image handling. */
		for (smIndex=0; smIndex<MAX_NUMBER_SMS_FOR_PMDBSYNC; smIndex++) {
			smLid = PmDbsyncSmInfo_p->SMs[smIndex].smLid;
			if (!smLid) break; /* zero smLid marks end of list */

			/* First, check if RAM resident images are yet to be synced for this standby SM. */
			if (PmDbsyncSmInfo_p->SMs[smIndex].rImgPmDbsyncNeeded && PmDbsyncSmInfo_p->SMs[smIndex].numSweepImagesSent) {
				IB_LOG_VERBOSE_FMT(__func__, "Older sweep image to be sent to SM[%d]. rImg=%d, numSweepImgSent=%d", smIndex, PmDbsyncSmInfo_p->SMs[smIndex].rImgPmDbsyncNeeded, PmDbsyncSmInfo_p->SMs[smIndex].numSweepImagesSent);
				/* Must start sending hist images to Standby SMs only after sending a latest sweep image first. */
				/* determine the latest unsent RAM image and send it to this standby SM. */
				numImagesSent = PmDbsyncSmInfo_p->SMs[smIndex].numSweepImagesSent +
								PmDbsyncSmInfo_p->SMs[smIndex].numHistImagesSent;
				IB_LOG_VERBOSE_FMT(__func__, "Calc hist offset using lastHistoryIndex=%d, numImagesSent %d.", pm->lastHistoryIndex, numImagesSent);
				offset = pm->lastHistoryIndex-numImagesSent;
				if (offset < 0) {
					offset = (pm_config.total_images+offset)%pm_config.total_images;
				}
				else {
					offset %= pm_config.total_images;
				}
				// check hist sweep num is < last hist image sweep num that was sent. PR126270: Ensure sweepnum is greater than 0. 
				if ( getSweepNumFromHistIndex(pm, offset) && ( !PmDbsyncSmInfo_p->SMs[smIndex].sweepNumHistImageSent ||
					(getSweepNumFromHistIndex(pm, offset) < PmDbsyncSmInfo_p->SMs[smIndex].sweepNumHistImageSent) ) ) {
					IB_LOG_VERBOSE_FMT(__func__, "History Image offset %d. Sending sweepnum %d image now..", offset, getSweepNumFromHistIndex(pm, offset));
					sm_send_pm_image(PmDbsyncSmInfo_p->SMs[smIndex].PortGUID, offset, NULL);
					PmDbsyncSmInfo_p->SMs[smIndex].numHistImagesSent++;
					PmDbsyncSmInfo_p->SMs[smIndex].sweepNumHistImageSent = getSweepNumFromHistIndex(pm, offset);
				}
			}
#if CPU_LE
			// Process STH files only on LE CPUs
			else if (PmDbsyncSmInfo_p->SMs[smIndex].histPmDbsyncNeeded) {
				/* determine the next image (in histImgListFile) and send it to this standby SM */
				getHistFileName(pm->ShortTermHistory.filepath,
								PmDbsyncSmInfo_p->SMs[smIndex].histImgListFile,
								PmDbsyncSmInfo_p->SMs[smIndex].histImgFileIndex, histFile);
				if (histFile[0] != '\0') {
					IB_LOG_VERBOSE_FMT(__func__, "History image file %s to be sent to SM", histFile);
					sm_send_pm_image(PmDbsyncSmInfo_p->SMs[smIndex].PortGUID, 0, histFile);
					PmDbsyncSmInfo_p->SMs[smIndex].histImgFileIndex++;
				}
				else { /* last file was sent */
					IB_LOG_VERBOSE_FMT(__func__, "History files synced up for SM index=%d",smIndex);
					PmDbsyncSmInfo_p->SMs[smIndex].histPmDbsyncNeeded = FALSE;
				}
			}
#endif	// End of CPU_LE
		}
	}

	/* Update status flags based on images sent to Standby SMs */
	PmDbsyncStatusFlagsUpdate(pm);
	return;
}

/* Main function for PM Dbsync operation.
 * Master mode operation: send RAM/history PM sweep image files to Standby SMs.
 * Standby mode operation: receive RAM/history PM sweep image files and process them.
 */
static FSTATUS PmDbsyncMain(Pm_t *pm)
{
	FSTATUS ret = FSUCCESS;
	static boolean base_line=TRUE;

	// Check if this SM is Master and has a valid topology.
	if (sm_util_isTopologyValid()) {	/* Dbsync thread processing on Master PM */
										/* Start of Master PM; the standby PMs form the baseline
											for which no additional image sync is needed beyond latest PM sweep images.
											PM Sweep images are sent to all standby PMs. 
											Any new SMs that get added, maintain a rImgPmDbsyncNeeded/histPmDbsyncNeeded flags per SM.
											if (rImg or hist PmDbsyncNeeded) {
												Send RAM resident images first to the Standby SM.
												Send disk resident images to the Standby SM.
												After all files are synced, rImg / histPmDbsyncNeeded=0;
											}
										*/
		IB_LOG_VERBOSE_FMT(__func__, "PM Dbsync processing on Master PM..");
		PmDbsyncSmInfo_copy(pm, base_line); /* obtain information about other SMs. */
		base_line = FALSE;
		IB_LOG_VERBOSE_FMT(__func__, "Calling PmDbsyncOperation()..");
		PmDbsyncOperation(pm); /* perform image sync operation. */

		extern boolean	isFirstImg;
		isFirstImg = TRUE; /* reset the flag to allow multiple master/standby transitions. */
	}
	else { /* Dbsync thread processing on standby PMs. */
		base_line=TRUE; /* reset the flag to allow multiple master/standby transitions. */
		IB_LOG_VERBOSE_FMT(__func__, "PM Dbsync processing on standby PM..");
	}
	return ret;
}

/* Periodic thread for PM Dbsync operation. Runs at a configurable (opafm.xml) period of image_update_interval. */
static void
PmDbsyncThread(uint32_t args, uint8_t **argv)
{
	uint64_t next, now;

	if (!pm_config.image_update_interval) {
		vs_log_output_message("PM: Dbsync operation disabled.", FALSE);
		return;
	}
	g_PmDbsyncThreadRunning = TRUE;
	pmDbsync_exit = 0;
	vs_log_output_message("PM: Dbsync thread starting up, will be syncing RAM/Disk PA history data", FALSE);
	IB_LOG_VERBOSE_FMT(__func__, "PM Dbsync thread period=%d seconds.",pm_config.image_update_interval);

	while (! pm_shutdown && g_pmEngineState == PM_ENGINE_STARTED && (pmDbsync_exit == 0)) {
		(void)vs_time_get(&now);
		next = now + VTIMER_1S * pm_config.image_update_interval;
		if (FSUCCESS != PmDbsyncMain(&g_pmSweepData) ) {
			next =  now + VTIMER_1S;  // wait only 1 second if there was an error in pm dbsync function
		}
		while (! pm_shutdown && g_pmEngineState == PM_ENGINE_STARTED && now < next && (pmDbsync_exit == 0)) {
			vs_thread_sleep(VTIMER_1S/2);
			(void)vs_time_get(&now);
		}
	}
	IB_LOG_VERBOSE_FMT(__func__, "PM Dbsync thread done.");
	g_PmDbsyncThreadRunning = FALSE;
}

void
PmDbsync_kill(void){
	pmDbsync_exit = 1;
}

void PM_InitStaticRateToMBps(void)
{
	uint8 i;

	for (i=0; i<= IB_STATIC_RATE_MAX; ++i)
		s_StaticRateToMBps[i] = StlStaticRateToMBps(i);
}

char *getDGName(int dgIndex)
{
	DGConfig_t* dgp;

	dgp = dg_config.dg[dgIndex];  // index always valid??
	return(dgp->name);
}

int getUserPGIndex(char *groupName)
{
	int res = -1;
	int i;
	PMXmlConfig_t *pmp = &pm_config;

	for (i = 0; i < pmp->number_of_pm_groups; i++) {
		if (strcmp(pmp->pm_portgroups[i].Name, groupName) == 0) {
			res = i;
			break;
		}
	}

	return(res);
}

// sometimes for inactive ports we may see a port with a 0 lid, ignore them
// until they get a lid assigned on the next SM sweep
// some Switch Port 0's report wrong state, so assume active if have a LID
#define sm_port_active(portp) \
	((portp)->state == IB_PORT_ACTIVE || \
	 	((portp)->index ==0 && (portp)->portData->lid))

// remove this port from all groups its in for pm->SweepIndex
// TBD - get portImage and pmImage from caller
static void RemoveFromGroups(Pm_t *pm, PmPort_t *pmportp)
{
	int grpIndex;
	int loopMax;
	uint32 imageIndex = pm->SweepIndex;
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];

#if PM_COMPRESS_GROUPS
	loopMax = portImage->numGroups;
#else
	loopMax = PM_MAX_GROUPS_PER_PORT;
#endif
	for (grpIndex=0; grpIndex<loopMax; grpIndex++) {
		PmGroup_t *groupp = portImage->Groups[grpIndex];
#if PM_COMPRESS_GROUPS
		DEBUG_ASSERT(groupp);
#else
		if (! groupp)
			continue;
#endif

		if (portImage->InternalBitMask & (1<<grpIndex)) {
            IB_LOG_VERBOSE_FMT(__func__, "%s Int. %.*s Guid "FMT_U64" LID 0x%x Port %u",
				groupp->Name, (int)sizeof(pmportp->pmnodep->nodeDesc.NodeString),
				pmportp->pmnodep->nodeDesc.NodeString,
				pmportp->pmnodep->NodeGUID,
				pmportp->pmnodep->Image[imageIndex].lid, pmportp->portNum);
			PmRemoveIntPortFromGroupIndex(portImage, grpIndex, groupp, imageIndex, 0);
		} else {
			IB_LOG_VERBOSE_FMT(__func__, "%s Ext. %.*s Guid "FMT_U64" LID 0x%x Port %u",
				groupp->Name, (int)sizeof(pmportp->pmnodep->nodeDesc.NodeString),
				pmportp->pmnodep->nodeDesc.NodeString,
				pmportp->pmnodep->NodeGUID,
				pmportp->pmnodep->Image[imageIndex].lid, pmportp->portNum);
			PmRemoveExtPortFromGroupIndex(portImage, grpIndex, groupp, imageIndex, 0);
		}
	}
	// remove from AllPorts group
	pm->AllPorts->Image[imageIndex].NumIntPorts--;
	portImage->u.s.Initialized = 0;	// force analysis of groups when reappears
}

// add port to proper groups for current sweep Index
// TBD - get portImage and pmImage from caller
static void AddToGroups(Pm_t *pm, PmPort_t *pmportp)
{
	int i;
	int grpIndex;
	char *groupName;
	uint32 imageIndex = pm->SweepIndex;
	PmPortImage_t *portImage = &pmportp->Image[imageIndex];

	// add to AllPorts group
	pm->AllPorts->Image[imageIndex].NumIntPorts++;

	// identify other groups port should be added to
	for (i=0,grpIndex=0; i<pm->NumGroups; i++) {
		PmGroup_t *groupp = pm->Groups[i];
		groupName = groupp->Name;
		// TBD - should a compare_node function be used so can skip
		// iterating on all ports of a switch?  However does not fit in
		// how this is presently called
		if (groupp->ComparePortFunc && ! (*groupp->ComparePortFunc)(pmportp, groupp->Name))
			continue;

		// add to group
		if (grpIndex >= PM_MAX_GROUPS_PER_PORT) {
			// port matches too many groups
			goto fail;
		} else if (portImage->neighbor) {
			if (groupp->ComparePortFunc && ! (*groupp->ComparePortFunc)(portImage->neighbor, groupp->Name)) {
				IB_LOG_VERBOSE_FMT(__func__, "%s Ext. %.*s Guid "FMT_U64" LID 0x%x Port %u",
					groupp->Name, (int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), pmportp->pmnodep->nodeDesc.NodeString,
					pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[imageIndex].lid, pmportp->portNum);
				PmAddExtPortToGroupIndex(portImage, grpIndex, groupp, imageIndex);
			} else {
				IB_LOG_VERBOSE_FMT(__func__, "%s Int. %.*s Guid "FMT_U64" LID 0x%x Port %u",
					groupp->Name, (int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), pmportp->pmnodep->nodeDesc.NodeString,
					pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[imageIndex].lid, pmportp->portNum);
				PmAddIntPortToGroupIndex(portImage, grpIndex, groupp, imageIndex);
			}
		} else {
			// TBD - should Port 0 of a switch be Ext in SWs?
			// perhaps this case should be Ext in general
			IB_LOG_VERBOSE_FMT(__func__, "%s Int. %.*s Guid "FMT_U64" LID 0x%x Port %u",
				groupp->Name, (int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), pmportp->pmnodep->nodeDesc.NodeString,
				pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[imageIndex].lid, pmportp->portNum);
			PmAddIntPortToGroupIndex(portImage, grpIndex, groupp, imageIndex);
		}
		grpIndex++;
	}
	return;

fail:
	if (!(pmportp->groupWarnings++ % (300 / pm_config.sweep_interval)))  // every 5 minutes
		IB_LOG_WARN_FMT(__func__, "Failed to Add port to group %.*s (too many groups for port) %.*s Guid "FMT_U64" LID 0x%x Port %u",
			(int)sizeof(groupName), groupName,
			(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), pmportp->pmnodep->nodeDesc.NodeString,
			pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[imageIndex].lid, pmportp->portNum);
	return;
}


// clear out port state for a port which has disappeared from fabric
static
void clear_pmport(Pm_t *pm, PmPort_t *pmportp)
{
	PmPortImage_t *portImage = &pmportp->Image[pm->SweepIndex];

	portImage->u.s.active = 0;
	portImage->u.s.mtu = 0;
	portImage->u.s.txActiveWidth = 0;
	portImage->u.s.rxActiveWidth = 0;
	portImage->u.s.activeSpeed = 0;
	pmportp->neighbor_lid = 0;	// no neighbor
}

void update_pm_vfvlmap(Pm_t *pm, PmPortImage_t *portImage, VlVfMap_t *vlvfmap)
{
	int vl, vf;
	PmVF_t *vfp;

	portImage->numVFs = 0;
	memset(&portImage->vfvlmap, 0, sizeof(vfmap_t) * MAX_VFABRICS);
	portImage->vlSelectMask = 1 << 15;

	for (vl = 0; vl < STL_MAX_VLS; vl++) {
		/* loop and add VFs - may be duplicates if VF is using more than 1 vl on this port */
		for(vf = 0; (vf = bitset_find_next_one(&vlvfmap->vf[vl], vf)) != -1; vf++){
			vfp = pm->VFs[vf];

			if(portImage->vfvlmap[vf].pVF != vfp){
				portImage->vfvlmap[vf].pVF = vfp;
				portImage->numVFs++;
			}

			portImage->vfvlmap[vf].vlmask |= 1 << vl;
			portImage->vlSelectMask |= 1 << vl;
		}
	}
	return;
}

// Update a Port's information (neighbor, state) for current SweepIndex
// caller should verify portp is valid
static
void update_pmport(Pm_t *pm, PmPort_t *pmportp, Node_t *nodep, Port_t *portp)
{
	Node_t *neigh_nodep;
	Port_t *neigh_portp;
	PmPortImage_t *portImage = &pmportp->Image[pm->SweepIndex];
	int vl;

	DEBUG_ASSERT(sm_valid_port(portp));
	portImage->u.s.active = sm_port_active(portp);
	portImage->u.s.mtu = portp->portData->maxVlMtu;
	portImage->u.s.txActiveWidth = portp->portData->portInfo.LinkWidthDowngrade.TxActive;
	portImage->u.s.rxActiveWidth = portp->portData->portInfo.LinkWidthDowngrade.RxActive;
	portImage->StlPortCounters.lq.s.NumLanesDown =
		(portp->portData->portInfo.LinkWidthDowngrade.RxActive < portp->portData->portInfo.LinkWidth.Active ?
		StlLinkWidthToInt(portp->portData->portInfo.LinkWidth.Active) -
		StlLinkWidthToInt(portp->portData->portInfo.LinkWidthDowngrade.RxActive) : 0);
	portImage->u.s.activeSpeed = portp->portData->portInfo.LinkSpeed.Active;
	memcpy(portImage->dgMember,  portp->portData->dgMemberList, sizeof(portp->portData->dgMemberList));
	if (pm_config.process_vl_counters) {
		VlVfMap_t vlvfmap;
		old_topology.routingModule->funcs.select_vlvf_map(&old_topology, nodep, portp, &vlvfmap); // select is based on appropriate instance of _select_vlvf_map()
		update_pm_vfvlmap(pm, portImage, &vlvfmap);
		// free bitsets allocated in select
		for (vl = 0; vl < STL_MAX_VLS; vl++){
			bitset_free(&vlvfmap.vf[vl]);
		}
	}

	// TBD - can't avoid sm_find_node, but it itself could be optimized

	// sm_get_port is low cost, so no need to try and optimize this by
	// keeping neighbor guid or other info in PmPort_t, easiest to just
	// reconnect
	neigh_nodep = sm_find_node(&old_topology, portp->nodeno);
	if (neigh_nodep && portp->portno) {	// port 0 can't be a neighbor
		neigh_portp = sm_get_port(neigh_nodep, portp->portno);
		if (sm_valid_port(neigh_portp) && sm_port_active(neigh_portp)) {
			if (neigh_nodep->nodeInfo.NodeType == STL_NODE_SW) {
				pmportp->neighbor_lid = sm_get_port(neigh_nodep, 0)->portData->lid;
			} else {
				pmportp->neighbor_lid = neigh_portp->portData->lid;
			}
			pmportp->neighbor_portNum = portp->portno;
		} else {
			pmportp->neighbor_lid = 0;	// no neighbor
		}
	} else {
		pmportp->neighbor_lid = 0;	// no neighbor
	}
	IB_LOG_DEBUG1_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x Port %u neighbor lid 0x%x Port %u",
		(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), pmportp->pmnodep->nodeDesc.NodeString,
		pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[pm->SweepIndex].lid, pmportp->portNum,
		pmportp->neighbor_lid, pmportp->neighbor_portNum);
}

// allocate a single port
PmPort_t *new_pmport(Pm_t *pm)
{
	Status_t status;
	PmPort_t *pmportp;

	status = vs_pool_alloc(&pm_pool, pm->PmPortSize, (void*)&pmportp);
	if (status != VSTATUS_OK || ! pmportp) {
		IB_LOG_ERRORRC("Failed to allocate PM Port rc:", status);
		return NULL;
	}
	memset(pmportp, 0, pm->PmPortSize);

	return pmportp;
}

// allocate and initialize a single port
static
PmPort_t *allocate_pmport(Pm_t *pm, Node_t *nodep, Port_t *portp, PmNode_t *pmnodep)
{
	PmPort_t *pmportp;

	pmportp = new_pmport(pm);
	if (!pmportp) {
		return NULL;
	}

	pmportp->guid = portp->portData->guid;
	pmportp->portNum = portp->index;
	pmportp->capmask = portp->portData->capmask;
	//pmportp->u.AsReg8 = 0;	// memset above covers this
		//Initialized = 0;
	pmportp->pmnodep = pmnodep;

	update_pmport(pm, pmportp, nodep, portp);	// update neighbor, state, rate and mtu 
	// assumes pmportp->nodep PmaCapabilities already updated
	PmUpdatePortPmaCapabilities(pmportp, portp);
	IB_LOG_DEBUG1_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x Port %u neighbor lid 0x%x Port %u",
		(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), pmportp->pmnodep->nodeDesc.NodeString,
		pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[pm->SweepIndex].lid, pmportp->portNum,
		pmportp->neighbor_lid, pmportp->neighbor_portNum);
	return pmportp;
}

// frees a port
void free_pmport(Pm_t *pm, PmPort_t *pmportp)
{
	IB_LOG_DEBUG1_FMT(__func__, "freeing %.*s Port %u Guid "FMT_U64" LID 0x%x",
		(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), pmportp->pmnodep->nodeDesc.NodeString,
		pmportp->portNum, pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[pm->SweepIndex].lid);
	RemoveFromGroups(pm, pmportp);
	vs_pool_free(&pm_pool, pmportp);
}

// free all the ports on a node
void free_pmportList(Pm_t *pm, PmNode_t *pmnodep) {
	if (!pmnodep) return;
	if (pmnodep->nodeType == STL_NODE_SW && pmnodep->up.swPorts) {
		uint8 i;
		for (i=0; i<= pmnodep->numPorts; ++i) {
			PmPort_t *pmportp = pmnodep->up.swPorts[i];
			if (pmportp) {
					free_pmport(pm, pmportp);
			}
		}
		vs_pool_free(&pm_pool, pmnodep->up.swPorts);
	} else if (pmnodep->up.caPortp) {
		free_pmport(pm, pmnodep->up.caPortp);
	}
}

static
void initialize_path(PmNode_t *pmnodep, STL_LID lid, uint32 imageIndex)
{
	// default path information (no redirect)
	pmnodep->dlid = pmnodep->Image[imageIndex].lid = lid;
	pmnodep->qpn = 1;
	pmnodep->sl = 0;	// can be adjusted when update_pmnode
	pmnodep->qkey = QP1_WELL_KNOWN_Q_KEY;
	pmnodep->pkey = 0xffff;
}

// Update a Node's information (nodeDesc, path) for current SweepIndex
// caller should verify portp is valid and that lided port for node is active
static
void update_pmnode(Pm_t *pm, PmNode_t *pmnodep, Node_t *nodep, Port_t *portp, Port_t *sm_portp)
{

	// this is unlikely to change, but we update it just in case
	// keep only 1 copy per node, not worth keeping per image
	// Note this is a potential race, but since array is defined to
	// not necessarily be null terminated, and its a byte array, worse case
	// a query gets a half of old and half of new name for 1 query.
	pmnodep->nodeDesc = nodep->nodeDesc;

	// TBD - better way of identifying and handling node reboots
	// technically anytime a node reboots (even if revision doesn't change
	// we need to do this.  TBD - if we can identify node reboot, no
	// need to track deviceRevision in pmnodep.
	//
	// Since we use device revision to track VendorPortCounters support
	// when device revision changes we need to update the Pma Capabilities
	if (pmnodep->deviceRevision != nodep->nodeInfo.Revision || g_pmFirstSweepAsMaster) {
		pmnodep->deviceRevision = nodep->nodeInfo.Revision;
		PmUpdateNodePmaCapabilities(pmnodep,nodep,(pm->flags & STL_PM_PROCESS_HFI_COUNTERS));
#if 1
		// to be safe, clear out any previous ClassPortInfo
		// be safe, will update capabilities when get class port info again
		pmnodep->u.s.PmaGotClassPortInfo = 0;
#else
		// Skip getting ClassPortInfo as in Gen1 there are no valid fields
		pmnodep->u.s.PmaGotClassPortInfo = 1;
#endif

		// reset path information
		initialize_path(pmnodep, portp->portData->lid, pm->SweepIndex);
	}

	pmnodep->sl = sm_masterPmSl;

	IB_LOG_DEBUG1_FMT(__func__, "%.*s Guid "FMT_U64" LID 0x%x: Path: DLID 0x%x PKEY 0x%x SL %u",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->NodeGUID, pmnodep->Image[pm->SweepIndex].lid,
		pmnodep->dlid, pmnodep->pkey, pmnodep->sl);
}

// free a node.  Frees the node and all its ports
// if still referenced, merely removes all ports from all groups for SweepIndex
void release_pmnode(Pm_t *pm, PmNode_t *pmnodep)
{
	
	uint32 refCount = AtomicDecrement(&pmnodep->refCount);
	IB_LOG_DEBUG1_FMT(__func__, "%s %.*s Guid "FMT_U64" LID 0x%x",
		refCount?"still referenced":"freeing",
		(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
		pmnodep->NodeGUID, pmnodep->Image[pm->SweepIndex].lid);
	if (refCount) {
		if (pmnodep->nodeType == STL_NODE_SW) {
			uint8 i;
			for (i=0; i<= pmnodep->numPorts; ++i) {
				PmPort_t *pmportp = pmnodep->up.swPorts[i];
				if (pmportp) {
					// TBD - for safety pmportp->Image[pm->SweepIndex].neighbor = NULL;
					RemoveFromGroups(pm, pmportp);
				}
			}
		} else {
			// TBD - for safety pmportp->Image[pm->SweepIndex].neighbor = NULL;
			RemoveFromGroups(pm, pmnodep->up.caPortp);
		}
	} else {
		free_pmportList(pm, pmnodep);
		if (pmnodep->AllNodesEntry.key)
			cl_qmap_remove_item(&pm->AllNodes, &pmnodep->AllNodesEntry);
		vs_pool_free(&pm_pool, pmnodep);
	}
}

// allocate a node and associated with the given "lid"'ed port
// for a switch, also allocates and initializes all the other ports
// for a switch portp must be port 0 (only lid'ed port on switch)
static
PmNode_t *allocate_pmnode(Pm_t *pm, Node_t *nodep, Port_t *portp)
{
	Status_t status;
	PmNode_t *pmnodep;
	int i;
	uint32 imageIndex = pm->SweepIndex;
	cl_map_item_t *mi;

	DEBUG_ASSERT(sm_valid_port(portp) && sm_port_active(portp));

	status = vs_pool_alloc(&pm_pool, pm->PmNodeSize, (void*)&pmnodep);
	if (status != VSTATUS_OK || ! pmnodep) {
		IB_LOG_ERRORRC("Failed to allocate PM Node rc:", status);
		return NULL;
	}
	memset(pmnodep, 0, pm->PmNodeSize);

	AtomicWrite(&pmnodep->refCount, 1);
	if (nodep->nodeInfo.NodeType == STL_NODE_SW)
		ASSERT(portp == sm_get_port(nodep, 0));	// portp should be lid'ed port

	pmnodep->NodeGUID = nodep->nodeInfo.NodeGUID;
	pmnodep->SystemImageGUID = nodep->nodeInfo.SystemImageGUID;
	pmnodep->nodeType = nodep->nodeInfo.NodeType;
	pmnodep->deviceRevision = nodep->nodeInfo.Revision;
	pmnodep->numPorts= nodep->nodeInfo.NumPorts;	// only used for switch
	pmnodep->nodeDesc = nodep->nodeDesc;
	pmnodep->changed_count = 0;
	pmnodep->up.swPorts = NULL;
	pmnodep->u.AsReg16 = 0;
		//PmaGotClassPortInfo = 0;
	PmUpdateNodePmaCapabilities(pmnodep, nodep, (pm->flags & STL_PM_PROCESS_HFI_COUNTERS));

	// default path information (no redirect)
	initialize_path(pmnodep, portp->portData->lid, imageIndex);

	if (nodep->nodeInfo.NodeType == STL_NODE_SW) {
		// allocate numPorts+1 pointers for pmnodep->up.swPorts
		status = vs_pool_alloc(&pm_pool,
					   	sizeof(PmPort_t*)*(pmnodep->numPorts+1),
					   	(void*)&pmnodep->up.swPorts);
		if (status != VSTATUS_OK || ! pmnodep->up.swPorts) {
			IB_LOG_ERRORRC("Failed to allocate PM Port ptrs rc:", status);
			vs_pool_free(&pm_pool, pmnodep);
			return NULL;
		}
		memset(pmnodep->up.swPorts, 0, 
					   	sizeof(PmPort_t*)*(pmnodep->numPorts+1));
		for (i=0; i<=pmnodep->numPorts; ++i) {
			Port_t *portp2 = (i==0)?portp:sm_find_node_port(&old_topology, nodep, i);
			if (sm_valid_port(portp2) && sm_port_active(portp2)
				&& (NULL == (pmnodep->up.swPorts[i] = allocate_pmport(pm, nodep, portp2, pmnodep)))) {
				IB_LOG_ERRORRC("Failed to allocate PM Port ptrs rc:", status);
				goto cleanup;
			}
		}
	} else {
		if (NULL == (pmnodep->up.caPortp = allocate_pmport(pm, nodep, portp, pmnodep))) {
			IB_LOG_ERRORRC("Failed to allocate PM Port ptrs rc:", status);
			goto cleanup;
		}
	}
	mi = cl_qmap_insert(&pm->AllNodes, portp->portData->guid, &pmnodep->AllNodesEntry);
	if (mi != &pmnodep->AllNodesEntry) {
		IB_LOG_ERRORLX("duplicate Node for portGuid", portp->portData->guid);
		goto cleanup;
	}
	IB_LOG_DEBUG1_FMT(__func__, "name: %.*s",
		(int)sizeof(nodep->nodeDesc.NodeString), nodep->nodeDesc.NodeString);
	return pmnodep;

cleanup:
	if (pmnodep->up.swPorts) {
		for (i=0; i<=pmnodep->numPorts; ++i) {
			if (pmnodep->up.swPorts[i]) {
				//free_pmport(pm, pmnodep->up.swPorts[i]);
				vs_pool_free(&pm_pool, pmnodep->up.swPorts[i]);
				pmnodep->up.swPorts[i] = NULL;
			}
		}
		vs_pool_free(&pm_pool, pmnodep->up.swPorts);
	}
	vs_pool_free(&pm_pool, pmnodep);
	return NULL;
}

PmNode_t *get_pmnodep(Pm_t *pm, Guid_t guid, STL_LID lid)
{
	PmNode_t *pmnodep = NULL;
	cl_map_item_t *mi;

	mi = cl_qmap_get(&pm->AllNodes, guid);
	if (mi != cl_qmap_end(&pm->AllNodes)) { // found
		pmnodep = PARENT_STRUCT(mi, PmNode_t, AllNodesEntry);
		AtomicIncrementVoid(&pmnodep->refCount);
		if (pm->LastSweepIndex == PM_IMAGE_INDEX_INVALID
			|| pmnodep != pm->Image[pm->LastSweepIndex].LidMap[lid]) {
			// lid changed since last pm sweep
#if 1
			// clear out any previous redirect
			// be safe, will update capabilities when get class port info again
			pmnodep->u.s.PmaGotClassPortInfo = 0;
#else
			// Skip getting ClassPortInfo as in Gen1 there are no valid fields
			pmnodep->u.s.PmaGotClassPortInfo = 1;
#endif

			// reset path information
			initialize_path(pmnodep, lid, pm->SweepIndex);
		} else {
			pmnodep->Image[pm->SweepIndex].lid = lid;
		}
	}
	return pmnodep;
}

// check "lid"'ed port, if lid is now assigned to a new guid
// free the old "lid"'ed port and for a switch free all its ports
// Note that SM tries to avoid lid changes, so the fact we loose stats
// for node is only going to happen in rare cases, such as fabric merge
static
PmNode_t *check_pmnodep(Pm_t *pm, Port_t *portp)
{
	PmNode_t *pmnodep;
	PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];
	STL_LID lid = portp->portData->lid;

	ASSERT(lid);
	ASSERT(lid <= old_topology.maxLid);
	ASSERT(portp->portData->guid);
	pmnodep = pmimagep->LidMap[lid];
	if (pmnodep
		&& (pmnodep->NodeGUID != portp->portData->nodePtr->nodeInfo.NodeGUID
			|| (pmnodep->nodeType != STL_NODE_SW
				   	&& pmnodep->up.caPortp->guid != portp->portData->guid))) {
		IB_LOG_VERBOSEX("Lid reused, free lid", lid);
		// still release_pmnode here, if it has any useful history it will have
		// some references so we should be able to find it again if lid changed
		release_pmnode(pm, pmnodep);	// releases pmnodep and any switch ports
		pmnodep = NULL;
	}
	if (! pmnodep) {
		pmnodep = get_pmnodep(pm, portp->portData->guid, lid);
	}
	pmimagep->LidMap[lid] = pmnodep;
	return pmnodep;
}

// Lookup a node in Pm Topology based on lid
// caller should have imageLock held
PmNode_t *pm_find_node(PmImage_t *pmimagep, STL_LID lid)
{
	if (lid > pmimagep->maxLid)
		return NULL;
	return pmimagep->LidMap[lid];
}

// Lookup a node in Pm Topology based on nodeguid
PmNode_t *pm_find_nodeguid(Pm_t *pm, uint64 nodeGUID)
{
	uint8 port_num = 0;
	int j;
	uint64 portGUID;
	PmNode_t *pmnodep = NULL;
	cl_map_item_t *mi;

	for ( j=0; j < 4; j++) {
		portGUID = (nodeGUID & ~PORTGUID_PNUM_MASK) | (((EUI64)port_num << PORTGUID_PNUM_SHIFT) & PORTGUID_PNUM_MASK);
		mi = cl_qmap_get(&pm->AllNodes, portGUID);
		if (mi != cl_qmap_end(&pm->AllNodes)) { // found
			pmnodep = PARENT_STRUCT(mi, PmNode_t, AllNodesEntry);
		}
	}
	return pmnodep;
}

// Lookup a port in Pm Topology based on lid and portNum
// does not have to be a "lid"'ed port
// caller should have imageLock held
PmPort_t *pm_find_port(PmImage_t *pmimagep, STL_LID lid, uint8 portNum)
{
	PmNode_t *pmnodep = pm_find_node(pmimagep, lid);
	if (pmnodep) {
		if (pmnodep->nodeType == STL_NODE_SW) {
			if (pmnodep->numPorts < portNum) {
				return NULL;	// incorrect port number
			} else {
				return pmnodep->up.swPorts[portNum];
			}
		} else {
			if (pmnodep->up.caPortp->portNum != portNum) {
				return NULL;	// incorrect port number
			} else {
				return pmnodep->up.caPortp;
			}
		}
	}
	return NULL;
}

// copy a "lid"'ed port (port 0 of switch, or port on FI)
// caller has verified portp supplied is valid and active
static Status_t
pm_copy_topology_port(Pm_t *pm, Port_t *portp, Port_t *sm_portp)
{
	Status_t status = VSTATUS_OK;
	PmNode_t *pmnodep;
	Node_t *nodep = portp->portData->nodePtr;

	IB_LOG_VERBOSE_FMT(__func__, "lid 0x%x Port %u", portp->portData->lid, portp->index);
	pmnodep = check_pmnodep(pm, portp);
	if (! pmnodep) {
		pmnodep = allocate_pmnode(pm, nodep, portp);
		if (! pmnodep)
			return VSTATUS_NOMEM;
		update_pmnode(pm, pmnodep, nodep, portp, sm_portp);
		// assumes pmportp->nodep PmaCapabilities already updated
		pm->Image[pm->SweepIndex].LidMap[portp->portData->lid] = pmnodep;
	} else if (pmnodep->nodeType == STL_NODE_SW) {
		// check state on all ports of switch
		int i;
		PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];
		update_pmnode(pm, pmnodep, nodep, portp, sm_portp);
		update_pmport(pm, pmnodep->up.swPorts[0], nodep, portp);
		pmimagep->SwitchPorts++;	// include Port 0
		for (i=1; i<=pmnodep->numPorts; ++i) {
			Port_t *portp2 = sm_find_node_port(&old_topology, nodep, i);
			PmPort_t *pmportp2 = pmnodep->up.swPorts[i];
			if (! sm_valid_port(portp2) || ! sm_port_active(portp2))
				portp2 = NULL;
			else
				pmimagep->SwitchPorts++;
			if (portp2 && ! pmportp2) {
				pmnodep->up.swPorts[i] = allocate_pmport(pm, nodep, portp2, pmnodep);
				if (! pmnodep->up.swPorts[i]) {
					IB_LOG_ERRORRC("Failed to allocate PM Port ptrs rc:", status);
                    			return VSTATUS_NOMEM;
				}
			} else if (! portp2 && pmportp2) {
				// we can't free port because other images may be using it
				// so clear out image portinfo and set to Down
				clear_pmport(pm, pmportp2);
				//free_pmport(pm, pmportp2);
				//pmnodep->up.swPorts[i] = NULL;
			} else if (portp2 && pmportp2) {
				update_pmport(pm, pmportp2, nodep, portp2);
			}
		}
	} else {
		update_pmnode(pm, pmnodep, nodep, portp, sm_portp);
		update_pmport(pm, pmnodep->up.caPortp, nodep, portp);
	}
	pmnodep->changed_count = topology_passcount; // topology_changed_count;
	return status;
}

// returns 1 if has a neighbor, 0 if not
uint32 connect_neighbor(Pm_t *pm, PmPort_t *pmportp)
{
	PmPort_t *new_neighbor;
	PmPortImage_t *portImage = &pmportp->Image[pm->SweepIndex];



	if (pmportp->neighbor_lid) {
		new_neighbor = pm_find_port(&pm->Image[pm->SweepIndex], pmportp->neighbor_lid, pmportp->neighbor_portNum);
		if (new_neighbor) {
			if (new_neighbor == portImage->neighbor)
                IB_LOG_DEBUG1_FMT(__func__, "reconnect neighbors.  lid 0x%x Port %u neighbor lid 0x%x Port %u", pmportp->pmnodep->Image[pm->SweepIndex].lid, pmportp->portNum, new_neighbor->pmnodep->Image[pm->SweepIndex].lid, new_neighbor->portNum);
			else
                IB_LOG_VERBOSE_FMT(__func__, "connect neighbors.  lid 0x%x Port %u neighbor lid 0x%x Port %u", pmportp->pmnodep->Image[pm->SweepIndex].lid, pmportp->portNum, new_neighbor->pmnodep->Image[pm->SweepIndex].lid, new_neighbor->portNum);
		} else {
			IB_LOG_VERBOSE_FMT(__func__, "unable to find neighbor of lid 0x%x Port %u", pmportp->pmnodep->Image[pm->SweepIndex].lid, pmportp->portNum);
		}
	} else {
		new_neighbor = NULL;
	}
	if (portImage->u.s.Initialized && new_neighbor != portImage->neighbor) {
		// neighbor changed and hence ports style of
		// group membership may change
		// ??? should we clear counters if neighbor changes???
		RemoveFromGroups(pm, pmportp);
		portImage->u.s.Initialized = 0;	// trigger add below
	}
	portImage->neighbor = new_neighbor;
	if (! portImage->u.s.Initialized && (new_neighbor || pmportp->portNum == 0)) {
		AddToGroups(pm, pmportp);
		portImage->u.s.Initialized = 1;
	}
	return (portImage->neighbor != NULL);
}

// copy all the ports which are in the new SM topology
// does not handle disappeared ports
static Status_t
pm_topology_copy_active_ports(Pm_t *pm, Port_t *sm_portp)
{
	Node_t *nodep;
	PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];

	pmimagep->HFIPorts = 0;
	pmimagep->SwitchNodes = 0;
	pmimagep->SwitchPorts = 0;

	for_all_nodes(&old_topology, nodep) {
		Port_t *portp;

		if (nodep->nodeInfo.NodeType == STL_NODE_SW) {
			portp = sm_get_port(nodep, 0);
			if (sm_valid_port(portp) && sm_port_active(portp)) {
				pmimagep->SwitchNodes++;
				if (pm_copy_topology_port(pm, portp, sm_portp) == VSTATUS_NOMEM)
                    return VSTATUS_NOMEM;
			}
		} else {
			for_all_ports(nodep, portp) {
				if (sm_valid_port(portp) && sm_port_active(portp)) {
					if (nodep->nodeInfo.NodeType == STL_NODE_FI) {
						pmimagep->HFIPorts++;
					}
					if (pm_copy_topology_port(pm, portp, sm_portp) == VSTATUS_NOMEM)
                        return VSTATUS_NOMEM;
				}
			}
		}
	}

    return VSTATUS_OK;
}

// re-connect neighbors and free disappeared ports
static
void pm_topology_reconnect(Pm_t *pm, PmImage_t *pmimagep)
{
	STL_LID lid;

	pmimagep->NumLinks = 0;

	for (lid = 1; lid <= pmimagep->maxLid; ++lid) {
		PmNode_t *pmnodep = pmimagep->LidMap[lid];
		if (! pmnodep)
			continue;
		if (pmnodep->changed_count != pm->changed_count) {
			// port disappeared in last fabric sweep
			// remove from this image, if it stays away for many
			// sweeps, reference count will hit 0 and it will be freed.
			IB_LOG_VERBOSEX("disappeared, free Lid:", lid);
			release_pmnode(pm, pmnodep);
			pmimagep->LidMap[lid] = NULL;
			continue;
		}
		if (pmnodep->nodeType == STL_NODE_SW) {
			uint8 i;
			for (i=0; i<= pmnodep->numPorts; ++i) {
				PmPort_t *pmportp = pmnodep->up.swPorts[i];
				if (pmportp)	// clear_pmport will setup so no neighbor
					pmimagep->NumLinks += connect_neighbor(pm, pmportp);
			}
		} else {
			pmimagep->NumLinks += connect_neighbor(pm, pmnodep->up.caPortp);
		}
	}
	// for simplicity during loop we counted both sides of each link
	// now divide to result in number of links
	pmimagep->NumLinks /= 2;
}

static void pm_sminfo_copy(Pm_t *pm)
{
	PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];
    SmRecp      smrecp;
    CS_HashTableItr_t itr;
    SmRecp      topSm=NULL; 

    /* 
     * Find highest priority SM in list with the proper SMKey other than us
     * We are the first entry in the list and should be master when this is
	 * called
     */
	pmimagep->NumSMs = 1;	// count us

	// TBD - sm_topology.c should lock this too?
	vs_lock(&smRecords.smLock);
    if (cs_hashtable_count(smRecords.smMap) >= 1) {
        cs_hashtable_iterator(smRecords.smMap, &itr);
        do {
            smrecp = cs_hashtable_iterator_value(&itr);
            if (smrecp->portguid == sm_smInfo.PortGUID) {   /* this is our entry */
				pmimagep->SMs[0].smLid = smrecp->smInfoRec.RID.LID;
				pmimagep->SMs[0].priority = smrecp->smInfoRec.SMInfo.u.s.Priority;
				pmimagep->SMs[0].state = smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent;
				continue;
			}
            if (smrecp->smInfoRec.SMInfo.SM_Key != sm_smInfo.SM_Key)
                continue;	// not a viable SM
            if (smrecp->smInfoRec.SMInfo.u.s.SMStateCurrent == SM_STATE_NOTACTIVE) {
				if (! topSm)
                	topSm = smrecp;
            } else if (topSm) {
                if (topSm->smInfoRec.SMInfo.u.s.Priority > smrecp->smInfoRec.SMInfo.u.s.Priority) {
                    /* topSm is still the one */
                } else if (topSm->smInfoRec.SMInfo.u.s.Priority < smrecp->smInfoRec.SMInfo.u.s.Priority) {
                    topSm = smrecp;
                } else if (topSm->smInfoRec.SMInfo.PortGUID > smrecp->smInfoRec.SMInfo.PortGUID) {
                    topSm = smrecp;
                }
            } else {
                topSm = smrecp;
            }
			pmimagep->NumSMs++;	// count active or inactive SM which is not us
        } while (cs_hashtable_iterator_advance(&itr));
    }
	if (topSm) {
		pmimagep->SMs[1].smLid = topSm->smInfoRec.RID.LID;
		pmimagep->SMs[1].priority = topSm->smInfoRec.SMInfo.u.s.Priority;
		pmimagep->SMs[1].state = topSm->smInfoRec.SMInfo.u.s.SMStateCurrent;
	} else {
		pmimagep->SMs[1].smLid = 0;
		pmimagep->SMs[1].priority = 0;
		pmimagep->SMs[1].state = 0;
	}
	vs_unlock(&smRecords.smLock);
}

// returns FSUCCESS if we are Master SM and have a valid topology
// returns FNOT_DONE if we are not the Master SM, in which case topology is not
// updated
static FSTATUS pm_copy_topology(Pm_t *pm)
{
	int changed = 0;
	FSTATUS ret = FNOT_DONE;
	STL_LID oldMaxLid = 0;	// keep compiler happy, initialized below when used
	PmNode_t **oldLidMap = NULL;
	uint32 newSize = 0;
	uint32 imageIndex = pm->SweepIndex;
	PmImage_t *pmimagep = &pm->Image[imageIndex];
	int i;
	extern SMXmlConfig_t sm_config;

	(void)vs_rdlock(&old_topology_lock);
	// TBD - may be possible to add optimization in SM itself to skip updates
	// to new_topology if topology did not change during a sweep.

	// sm_isMaster and topology_passcount are protected by new_topology_lock,
	// but a quick read should be ok
	// let SM become master and get a sweep done 1st
    if (sm_isMaster() && topology_passcount >= 1)
		ret = FSUCCESS;

#ifdef __VXWORKS__
    if (old_topology.num_nodes >= MAX_SUBNET_SIZE) {
        // a SM shutdown race condition exists, so as a safty precaution
        // verify the fabric size.
        IB_LOG_ERROR_FMT(__func__, "TT: aborting - fabric size exceeds the %d maximum nodes supported by the ESM", MAX_SUBNET_SIZE);
        pm_shutdown = TRUE;
        ret = FINSUFFICIENT_RESOURCES;
        goto bail;
    }
#endif

	// must copy below so Image has copy of latest topology
    if (ret == FSUCCESS) {
		Port_t *sm_portp;
		PmNode_t **newLidMap;
		Status_t rc;

		IB_LOG_INFO_FMT(__func__, "START copy new PM topology: size=%u", pmimagep->lidMapSize);
        IB_LOG_VERBOSE_FMT(__func__, "sm passcount=%u", topology_passcount);
		sm_portp = sm_get_port(old_topology.node_head, pm_config.port);
		pm->pm_slid = sm_lid;

		oldMaxLid = pmimagep->maxLid;
		// as needed grow or shink array, with some extra space for growth
		if (pmimagep->lidMapSize < old_topology.maxLid+1
			|| pmimagep->lidMapSize > old_topology.maxLid + PM_LID_MAP_FREE_THRESHOLD) {
			newSize = MIN(old_topology.maxLid+1+PM_LID_MAP_SPARE,sm_config.max_supported_lid+1);
			IB_LOG_INFO_FMT(__func__, "resize lidMap oldSize=%u newSize=%u", pmimagep->lidMapSize, newSize);
			rc = vs_pool_alloc(&pm_pool, newSize*sizeof(PmNode_t *),
							(void*)&newLidMap);
			if (rc != VSTATUS_OK || ! newLidMap) {
                ret = FINSUFFICIENT_MEMORY;
				IB_LOG_ERRORRC("Failed to allocate PM Lid Map rc:", rc);
				goto bail;
			}
			if (newSize > pmimagep->lidMapSize) {
				// growing
				if (pmimagep->LidMap) {
					// no need to copy above oldMaxLid
					memcpy(newLidMap, pmimagep->LidMap, (oldMaxLid+1)*sizeof(*pmimagep->LidMap));
					oldLidMap = pmimagep->LidMap;
				}
				memset(newLidMap+(oldMaxLid+1), 0, (newSize-(oldMaxLid+1))*sizeof(*pmimagep->LidMap));
			} else {
				// shrinking
				memcpy(newLidMap, pmimagep->LidMap, newSize*sizeof(*pmimagep->LidMap));
				oldLidMap = pmimagep->LidMap;
			}
			pmimagep->LidMap = newLidMap;
			pmimagep->lidMapSize = newSize;
		}

		//Update Active VFs
		VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;
		pm->numVFs = 0;
		pm->numVFsActive = 0;
		for (i = 0; i < VirtualFabrics->number_of_vfs_all; i++) {
			PmVF_t *vfp = pm->VFs[i];
			if(! vfp) continue;

			if (strncmp(vfp->Name, VirtualFabrics->v_fabric_all[i].name, STL_PM_VFNAMELEN) == 0){
				if(VirtualFabrics->v_fabric_all[i].standby){
					vfp->Image[imageIndex].isActive = 0;
				} else {
					vfp->Image[imageIndex].isActive = 1;
					pm->numVFsActive++;
				}
				pm->numVFs++;
			} else{
				IB_LOG_WARN_FMT(__func__, "PM VF list does not match SM VF: VF %d, SM: %.*s PM: %.*s ", i,
					(int)sizeof(VirtualFabrics->v_fabric_all[i].name), VirtualFabrics->v_fabric_all[i].name,
					(int)sizeof(vfp->Name), vfp->Name);
				// TODO: Handle when VF list changes order.
			}
		}
		for ( ; i < MAX_VFABRICS; i++) {
			pm->VFs[i]->Image[imageIndex].isActive = 0;
		}
		if (pm->numVFs != VirtualFabrics->number_of_vfs_all) {
			IB_LOG_WARN_FMT(__func__, "PM VF list count does not match SM VF list: SM count: %u PM count: %u",
				VirtualFabrics->number_of_vfs_all, pm->numVFs);
		}
		if (pm->numVFsActive != VirtualFabrics->number_of_vfs) {
			IB_LOG_WARN_FMT(__func__, "PM VF list Active count does not match SM VF list: SM count: %u PM count: %u",
				VirtualFabrics->number_of_vfs, pm->numVFsActive);
		}

		// copy all the ports which are in the new SM topology
		// we will handle ports which have disappeared below
		if (pm_topology_copy_active_ports(pm, sm_portp) == VSTATUS_NOMEM) {
            ret = FINSUFFICIENT_MEMORY;
            IB_LOG_ERRORRC("Failed to allocate copy of active ports rc:", VSTATUS_NOMEM);
            goto bail;
        }

		// neighbors are handled below
		pm->changed_count = topology_passcount; // topology_changed_count;
		IB_LOG_INFO_FMT(__func__,"DONE copy new PM topology");

		pmimagep->maxLid = old_topology.maxLid;
		IB_LOG_INFOX("new maxLid:", pmimagep->maxLid);

		changed = 1;
	}
	(void)vs_rwunlock(&old_topology_lock);

	if (changed) {
		STL_LID lid;
		PmNode_t *pmnodep;

		// free lids above new maxLid
		for (lid = pmimagep->maxLid+1; lid <= oldMaxLid; ++lid) {
			pmnodep = oldLidMap?oldLidMap[lid]:pmimagep->LidMap[lid];
			if (! pmnodep)
				continue;
			IB_LOG_VERBOSEX("above max, free Lid:", lid);
			release_pmnode(pm, pmnodep);
			if (! oldLidMap || lid < newSize)
				pmimagep->LidMap[lid] = NULL;
		}
		if (oldLidMap)
			vs_pool_free(&pm_pool, oldLidMap);

		// re-connect neighbors and free disappeared ports
		pm_topology_reconnect(pm, pmimagep);

		// copy sm Infomation
		pm_sminfo_copy(pm);
	}
	return ret;
bail:
	(void)vs_rwunlock(&old_topology_lock);
	return ret;
}

/* ------------------ Short-Term History ----------------------------- */

/*************************************************************************************
*   computeCompositeSize - Compute how much memory a 
*   composite image will need, using pm_config 
*    
*   Inputs: 
*   	none
*    
*   Return: 
*   	The size of a composite image
*  
*   Note: This is based off of PM Config values and may be
*   	larger than needed. It could also be smaller than
*   	needed if the PM is not configured properly.
*************************************************************************************/
size_t computeCompositeSize(void) {
	return	sizeof(PmCompositeImage_t) 
			+ (sizeof(PmCompositeNode_t) * cs_numNodeRecords(pm_config.subnet_size))
			+ (sizeof(PmCompositePort_t) * cs_numPortRecords(pm_config.subnet_size) + 1);
}

/************************************************************************************* 
*   computeFlatSize - Compute how much memory a 
*   composite image will take up once it has been flattened 
*    
*   Inputs: 
*   	cimg - The composite image to be flattened
*    
*   Return: 
*   	The size of the flattened image
*  
*   Note:
*   	The 'flat' size accounts for all of the nodes and ports,
*   	but does not include the size of the pointers themselves,
*   	since they won't be included in the file
*************************************************************************************/
size_t computeFlatSize(PmCompositeImage_t *cimg) {
	return ((sizeof(PmCompositeImage_t) - sizeof(PmCompositeNode_t**)) + 
		((cimg->maxLid + 1) * (sizeof(PmCompositeNode_t) - sizeof(PmCompositePort_t**))) + 
		((cimg->numPorts) * sizeof(PmCompositePort_t)));
}

/************************************************************************************* 
*   buildShortTermHistoryImageId - build an image ID for use with short term history
* 
*   Inputs:
*   	pm - the PM
*   	imageIndex - the index of the image
* 
*   Return:
*   	the ImageId as a 64bit integer
*
*************************************************************************************/ 
static uint64 buildShortTermHistoryImageId(Pm_t *pm, uint32 imageIndex) {
	ImageId_t id;

	id.AsReg64 = 0;
	id.s.type = IMAGEID_TYPE_HISTORY;
	id.s.sweepNum = pm->Image[pm->history[imageIndex]].sweepNum;
	id.s.instanceId = pm->ShortTermHistory.currentInstanceId;
	return id.AsReg64;
}

#ifndef __VXWORKS__
/************************************************************************************* 
*   decompressComposite - decompress a composite History file
*  
*   Inputs:
*   	input_data - buffer of compressed data
*   	input_size - the size of the input buffer
*   	output_data - output data buffer
*   	output_size - the size of the output data buffer
*  
*   Returns:
*   	Status - FSUCCESS if okay
*  
*   Note:
*   	The output_data buffer will be filled with the uncompressed data
*   	Unlike compressData, output_size must be provided and will not
*   	be updated & output_data must have already been allocated
*  
*************************************************************************************/
FSTATUS decompressData(unsigned char *input_data, size_t input_size, unsigned char *output_data, size_t output_size) {
	int ret;
	z_stream strm;

	// initialize
	strm.zalloc = Z_NULL;
	strm.zfree = Z_NULL;
	strm.opaque = Z_NULL;
	strm.avail_in = 0;
	strm.next_in = Z_NULL;
	ret = inflateInit(&strm);
	if (ret != Z_OK) {
		IB_LOG_ERROR0("Error decompressing PM history image: Unable to initialize inflation");
		return FERROR;
	}

	// get ready to decompress
	strm.avail_in = input_size;
	strm.next_in = input_data;
	strm.avail_out = output_size;
	strm.next_out = output_data;

	// decompress the data
	ret = inflate(&strm, Z_FINISH);
	if (ret != Z_STREAM_END) {
		IB_LOG_ERROR0("Error decompressing PM history image");
		(void)inflateEnd(&strm);
		return FERROR;
	}
	inflateEnd(&strm);
	return FSUCCESS;
}

struct decompress_args {
	int thread_index;
	Thread_t *thread_ptr;
	unsigned char *input_data;
	size_t input_size;
	unsigned char *output_data;
	size_t output_size;
};

void threadDecompress(uint32_t argc, uint8_t **argv) {
	FSTATUS ret; 
	int id;
	Thread_t *thread_ptr;

	if (argc != 6)
	{
		IB_LOG_ERROR ("Internal errror, invalid arguments", argc);
		return;
	}

	id =  ((struct decompress_args*)argv)->thread_index;
	thread_ptr = ((struct decompress_args*)argv)->thread_ptr;

	ret = decompressData(
	  ((struct decompress_args*)argv)->input_data,
	  ((struct decompress_args*)argv)->input_size,
	  ((struct decompress_args*)argv)->output_data,
	  ((struct decompress_args*)argv)->output_size
	  );

	if(ret != FSUCCESS)
	{
		IB_LOG_ERROR ("PM decompress thread did not decompress ID:", id);
		return;
	}

	vs_thread_exit(thread_ptr);
	return;
}

/************************************************************************************* 
*   decompressAndReassemble - decompress each piece of a History file and put it back
*   	together
*  
*   Inputs:
*   	input_data - buffer of compressed data - must NOT contain the uncompressed header section
*   	input_size - total size of the input data
*   	divs - number of divisions of the compressed input data
*   	input_sizes - an array of length 'divs' that lists the size of each division
*   	output_data - output data buffer
*   	output_size - the size of the output data buffer
*  
*   Returns:
*   	Status - FSUCCESS if okay
*  
*   Note:
*   	The output_data buffer will be filled with the uncompressed data
*   	Unlike compressData, output_size must be provided and will not
*   	be updated & output_data must have already been allocated
*  
*  
*************************************************************************************/
FSTATUS decompressAndReassemble(unsigned char *input_data, size_t input_size, uint8 divs, uint64 *input_sizes, unsigned char *output_data, size_t output_size) {
	uint32_t argc;
	unsigned char name[VS_NAME_MAX] = "";
	Thread_t *decompression_threads;

	// first check divs, make sure it isn't over the max
	if (divs > PM_MAX_COMPRESSION_DIVISIONS) {
		IB_LOG_ERROR_FMT(NULL, "Unable to decompress, invalid number of divisions: %d", divs);
		return FERROR;
	}
	// also need to check output size, if it is 0 that could cause some troubles
	if (output_size == 0) {
		IB_LOG_ERROR0("Unable to decompress, invalid data output size");
		return FERROR;
	}
	unsigned char *input_pieces[divs];
	// if numDivisions is 0, treat it as 1
	if (divs == 0) divs = 1;
	// the length of each division is the output size / number of divisions, rounded up
	size_t len = (output_size % divs ? (output_size / divs) + 1 : (output_size / divs));
	unsigned char *loc = input_data;
	// need to total up all of the input sizes to make sure they do not exceed the expected input size
	size_t total_in = 0;

	int i;
	for (i = 0; i < divs; i++) {
		// input
		input_pieces[i] = loc;
		loc += input_sizes[i];
		total_in += (size_t)input_sizes[i];
	}
	if (total_in > input_size) {
		IB_LOG_ERROR0("Unable to decompress, invalid division sizes");
		return FERROR;
	}
	decompression_threads = calloc(divs, sizeof(Thread_t));
	if (decompression_threads == NULL) {
		IB_LOG_ERROR0("Failed to allocate Thread_t Array for Decompression");
		return FINSUFFICIENT_MEMORY;
	}
	// in parallel: call decompress on each piece
	struct decompress_args args[divs];
	int ret;

	for (i = 0; i < divs; i++) {
		//vthread_create does not accept thread_index = 0
		args[i].thread_index = i+1;
		args[i].thread_ptr = &decompression_threads[i];
		args[i].input_data = input_pieces[i];
		args[i].input_size = (size_t)input_sizes[i];
		args[i].output_data = output_data + (i * len);
		args[i].output_size = MIN(len, output_size - (i * len));

		//argument count is 6
		argc = 6;
		snprintf((char *)name,VS_NAME_MAX, "DecompressThr%d", i );

		ret = vs_thread_create(&decompression_threads[args[i].thread_index-1], name, threadDecompress, argc ,(uint8_t **)&args[i], PM_COMPRESS_THREAD_STACK_SIZE);
		if (ret!= VSTATUS_OK) IB_LOG_ERROR_FMT(__func__, "Failed to create decompression thread (%d): %d", i, ret);
	}

	// wait for all of the threads to complete
	for (i = 0; i < divs; i++) {
		ret = vs_thread_join(&decompression_threads[args[i].thread_index-1], NULL);
		if (ret) IB_LOG_ERROR_FMT(__func__, "Failed to join Decompression thread (%d): %d", i, ret);
	}

	if (decompression_threads) free(decompression_threads);
	return FSUCCESS;
}

/************************************************************************************* 
    compressData - Compress data and write it to a buffer
 
    Inputs:
    	input_data - the data to be compressed
    	input_size - the number of bytes of data
    	output_data - pointer to the output data
    	output_size - pointer to the output size
 
    Returns:
    	Status - FSUCCESS if okay
 
    The function will compress what is in input_data, place that in output_data, and
    update output_size to reflect the size of the output.
 
*************************************************************************************/ 
static FSTATUS compressData(unsigned char *input_data, size_t input_size, unsigned char **output_data, size_t *output_size)  {
	int ret;
	z_stream strm;
	size_t bound;

	// initialize the z stream
	strm.zalloc = Z_NULL;
	strm.zfree = Z_NULL;
	strm.opaque = Z_NULL;
	ret = deflateInit(&strm, 3);
	if (ret != Z_OK) {
		return FERROR;
	}

	// allocate output
	// deflate bound gives an upper bound on how much space the compressed data will take up
	bound = (size_t)deflateBound(&strm, (uLong)input_size);
	*output_data = (unsigned char*)calloc(1, bound);
	if (!(*output_data)) {
		*output_size = 0;
		(void)deflateEnd(&strm);
		return FERROR;
	}

	// get ready to compress
	strm.avail_in = input_size;
	strm.next_in = input_data;
	strm.avail_out = bound;
	strm.next_out = *output_data;

	// compress the data
	ret = deflate(&strm, Z_FINISH);
	if (ret != Z_STREAM_END) {
		IB_LOG_ERROR0("Error while deflating PM History Image");
		free(*output_data);
		return FERROR;
	}

	// resize the output buffer to match what was actually used
	*output_size = (uint64)strm.total_out;
	*output_data = (unsigned char*)realloc(*output_data, *output_size);

	(void)deflateEnd(&strm);
	return FSUCCESS;
}

// data needed for the compress thread
struct compress_args {
	int thread_index;
	Thread_t *thread_ptr;
	unsigned char *input_data;
	size_t input_size;
	unsigned char **output_data;
	size_t *output_size;
};

void threadCompress(uint32_t argc, uint8_t **argv) {
	FSTATUS ret; 
	int id;
	Thread_t *thread_ptr;

	if (argc != 6) // the check avoids variable not used warning
	{
		IB_LOG_ERROR ("Internal errror, invalid arguments", 0);
		return;
	}

	id = ((struct compress_args*)argv)->thread_index;
	thread_ptr = ((struct compress_args*)argv)->thread_ptr;

	ret = compressData(
	   ((struct compress_args*)argv)->input_data,
	   ((struct compress_args*)argv)->input_size,
	   ((struct compress_args*)argv)->output_data,
	   ((struct compress_args*)argv)->output_size
	   );
	if(ret != FSUCCESS)
	{
		IB_LOG_ERROR ("PM compress thread did not compress ID:", id);
		return;
	}
	vs_thread_exit(thread_ptr);
	return;
}

/************************************************************************************* 
    divideAndcompress - Take a flattened image, compress it, and write it to a buffer
 
    Inputs:
    	input_data - the data to be compressed, does not include header
    	input_size - size of input data
    	compressed_divisions - pointer to the array of pointers to compressed divisions
    	compressed_lengths - pointer to the array of sizes for the compressed divisions
 
    Returns:
    	Status - FSUCCESS if okay
 
    The function will divide the input data into the number of chunks defined in the
    xml config file. The divisions will be compressed in parallel. The array
    compressed_divisions will point to each compressed piece, and the array
    compressed_lengths will tell how long each compressed piece is.
 
*************************************************************************************/ 
FSTATUS divideAndCompress(unsigned char *input_data, size_t input_size, unsigned char **compressed_divisions, size_t *compressed_lengths) {
	uint32_t argc;
	unsigned char name[VS_NAME_MAX] = "";
	Thread_t *compression_threads;

	if (pm_config.shortTermHistory.compressionDivisions == 0) {
		// really this shouldn't be possible, but better to be safe
		IB_LOG_ERROR0("Invalid compression divisions, must not be 0");
		return FERROR;
	}
	// determine the length of each division
	// the last section may be a little bit shorter than the others if the data doesn't divide equally
	size_t len = (input_size % pm_config.shortTermHistory.compressionDivisions ? 
				  (input_size / pm_config.shortTermHistory.compressionDivisions) + 1:
				  (input_size / pm_config.shortTermHistory.compressionDivisions));
	int i;

	compression_threads = calloc(pm_config.shortTermHistory.compressionDivisions, sizeof(Thread_t));
	if (compression_threads == NULL) {
		IB_LOG_ERROR0("Failed to allocate Thread_t Array for Compression");
		return FINSUFFICIENT_MEMORY;
	}
	// in parallel: call compressData on each division
	struct compress_args args[pm_config.shortTermHistory.compressionDivisions];
	int ret;

	for (i = 0; i < pm_config.shortTermHistory.compressionDivisions; i++) {
		//vthread_create does not accept thread_index = 0
		args[i].thread_index = i+1;
		args[i].thread_ptr = &compression_threads[i];
		args[i].input_data = input_data + (i * len);
		args[i].input_size = MIN(len, input_size - (i * len));
		args[i].output_data = &compressed_divisions[i];
		args[i].output_size = &compressed_lengths[i];

		//argument count is 6
		argc = 6;
		snprintf((char *)name,VS_NAME_MAX, "CompressThr%d", i );
		ret = vs_thread_create(&compression_threads[args[i].thread_index-1], name, threadCompress, argc ,(uint8_t **)&args[i], PM_COMPRESS_THREAD_STACK_SIZE);
		if (ret != VSTATUS_OK) IB_LOG_ERROR_FMT(__func__, "Failed to create compression thread (%d): %d", i, ret);
	}

	// wait for all of the threads to complete
	for (i = 0; i < pm_config.shortTermHistory.compressionDivisions; i++) {
		ret = vs_thread_join(&compression_threads[args[i].thread_index-1], NULL);
		if (ret) IB_LOG_ERROR_FMT(__func__, "Failed to join compression thread (%d): %d", i, ret);
	}

	if (compression_threads) free(compression_threads);
	return FSUCCESS;
}

#endif

/************************************************************************************* 
*   rebuildComposite - rebuild a composite image from a data buffer
*  
*   Inputs:
*   	cimg - the composite image to create
*   	data - the data buffer
*   	history_version - version of cimg
*  
*   Returns:
*   	Status - FSUCCESS if okay
*  
*************************************************************************************/
FSTATUS rebuildComposite(PmCompositeImage_t *cimg, unsigned char *data, uint32 history_version) {
	uint8_t *loc = data;
	int i, j;

	if (history_version == PM_HISTORY_VERSION) {
		// Copy everything except for the node pointer at the end of the struct
		memcpy((unsigned char *)cimg + sizeof(PmFileHeader_t), loc, (sizeof(PmCompositeImage_t) - (sizeof(PmFileHeader_t) + sizeof(PmCompositeNode_t **))));
		loc += (sizeof(PmCompositeImage_t) - (sizeof(PmFileHeader_t) + sizeof(PmCompositeNode_t **)));
#ifndef __VXWORKS__
	} else if (history_version == PM_HISTORY_VERSION_OLD) {
		// Handle change in MAX_VFABRICS form 32 to 1000
		loc += PaSthOldCompositeImageToCurrent(cimg, loc - sizeof(PmFileHeader_t));
#endif /* __VXWORKS__ */
	} else {
		return FINVALID_PARAMETER;
	}
	// Nodes
	cimg->nodes = calloc(1, sizeof(cimg->nodes)*(cimg->maxLid+1));
	if (!cimg->nodes) {
		return FINSUFFICIENT_MEMORY;
	}
	for (i = 0; i <= cimg->maxLid; i++) {
		cimg->nodes[i] = calloc(1, sizeof(PmCompositeNode_t));
		if (!cimg->nodes[i]) {
			return FINSUFFICIENT_MEMORY;
		}
		if (history_version == PM_HISTORY_VERSION) {
			memcpy(cimg->nodes[i], loc, (sizeof(PmCompositeNode_t) - sizeof(PmCompositePort_t **)));
			loc += (sizeof(PmCompositeNode_t) - sizeof(PmCompositePort_t**));
#ifndef __VXWORKS__
		} else if (history_version == PM_HISTORY_VERSION_OLD) {
			loc += PaSthOldCompositeNodeToCurrent(cimg->nodes[i], loc);
#endif /* __VXWORKS__ */
		}
		if (cimg->nodes[i]->numPorts == 0)
			continue;
		if (cimg->nodes[i]->nodeType == STL_NODE_SW) {
			cimg->nodes[i]->ports = calloc(1, (sizeof(cimg->nodes[i]->ports)*(cimg->nodes[i]->numPorts + 1)));
			if (!cimg->nodes[i]->ports) {
				return FINSUFFICIENT_MEMORY;
			}
			// Ports
			for (j = 0; j <= cimg->nodes[i]->numPorts; j++) {
				cimg->nodes[i]->ports[j] = calloc(1, sizeof(PmCompositePort_t));
				if (!cimg->nodes[i]->ports[j]) {
					return FINSUFFICIENT_MEMORY;
				}
				if (history_version == PM_HISTORY_VERSION) {
					memcpy(cimg->nodes[i]->ports[j], loc, sizeof(PmCompositePort_t));
					loc += sizeof(PmCompositePort_t);
#ifndef __VXWORKS__
				} else if (history_version == PM_HISTORY_VERSION_OLD) {
					loc += PaSthOldCompositePortToCurrent(cimg->nodes[i]->ports[j], loc);
#endif /* __VXWORKS__ */
				}
			}
		} else {
			cimg->nodes[i]->ports = calloc(1, sizeof(cimg->nodes[i]->ports));
			if (!cimg->nodes[i]->ports) {
				return FINSUFFICIENT_MEMORY;
			}
			cimg->nodes[i]->ports[0] = calloc(1, sizeof(PmCompositePort_t));
			if (!cimg->nodes[i]->ports[0]) {
				return FINSUFFICIENT_MEMORY;
			}
			if (history_version == PM_HISTORY_VERSION) {
				memcpy(cimg->nodes[i]->ports[0], loc, sizeof(PmCompositePort_t));
				loc += sizeof(PmCompositePort_t);
#ifndef __VXWORKS__
			} else if (history_version == PM_HISTORY_VERSION_OLD) {
				loc += PaSthOldCompositePortToCurrent(cimg->nodes[i]->ports[0], loc);
#endif /* __VXWORKS__ */
			}
		}
	}
	return FSUCCESS;
}

/************************************************************************************* 
*   flattenComposite - Flatten a composite image into a data 
*    buffer
*    
*   Inputs: 
*		cimg - The composite image to flatten
*		data - The data buffer to flatten into
*   	history_version - version of cimg
*    
*   Return: 
*   	FSUCCESS if okay
*  
*   Note:
*   	'data' should already have memory allocated to it (use
*   	computeFlatSize)
*************************************************************************************/
static FSTATUS flattenComposite(PmCompositeImage_t *cimg, unsigned char *data, uint32 history_version) {
	//TODO: probably should figure out a way to check for overflow?
	if (!data) {
		IB_LOG_WARN0("Unable to flatten image");
		return FINVALID_PARAMETER;
	}
	int i;
	unsigned char *loc = data;
	if (history_version == PM_HISTORY_VERSION) {
		// Copy everything except for the node pointer at the end of the struct
		memcpy(loc, cimg, sizeof(PmCompositeImage_t) - sizeof(PmCompositeNode_t **));
		// update the location pointer
		loc += (sizeof(PmCompositeImage_t) - sizeof(PmCompositeNode_t **));
	} else {
		// Only Current Version should call this function
		return FINVALID_PARAMETER;
	}
	// copy every node
	for (i = 0; i <= cimg->maxLid; i++) {
		PmCompositeNode_t *cnode = cimg->nodes[i];
		if (!cnode) 
			continue;
		// copy everything for the node except for the pointer at the end of the struct
		memcpy(loc, cnode, sizeof(PmCompositeNode_t) - sizeof(PmCompositePort_t**));
		// update the location pointer
		loc += (sizeof(PmCompositeNode_t) - sizeof(PmCompositePort_t**));
		if (!cnode->ports) 
			continue;
		// copy every port
		int j;
		if (cnode->nodeType == STL_NODE_SW) {
			for (j = 0; j <= cnode->numPorts; j++) {
				PmCompositePort_t *cport = cnode->ports[j];
				if (!cport)
					continue;
				// copy everything for the port
				memcpy(loc, cport, sizeof(PmCompositePort_t));
				// update the location pointer
				loc += sizeof(PmCompositePort_t);
			}
		} else {
			PmCompositePort_t *cport = cnode->ports[0];
			if (!cport) 
				continue;
			memcpy(loc, cport, sizeof(PmCompositePort_t));
			loc += sizeof(PmCompositePort_t);
		}
	}
	return FSUCCESS;
}

/************************************************************************************* 
    createCompositePort - Allocate and create a composite port
 
    Inputs:
    	port - the port that to use to create the composite port
    	cimg - the parent composite image
    	imageIndex - index into the images
 
    Returns:
    	The created composite port
 
    Returns and empty, non-null composite port if input port is null
 
*************************************************************************************/ 
static PmCompositePort_t *createCompositePort(PmPort_t *port, PmCompositeImage_t *cimg, uint64 imageIndex) {
	PmCompositePort_t *cport;
	// allocate the memory for this port
	cport = calloc(1, sizeof(PmCompositePort_t));
	if (!cport) 
		return NULL;

	if (!port) 
		return cport; // return empty (non-null) port

	// set the values
	cport->guid = port->guid;
	cport->portNum = port->portNum;
	cport->u.AsReg32 = port->Image[imageIndex].u.AsReg32;

	if (port->Image[imageIndex].neighbor && port->Image[imageIndex].neighbor->pmnodep) {
		cport->neighborLid = port->Image[imageIndex].neighbor->pmnodep->Image[imageIndex].lid;
		cport->neighborPort = port->Image[imageIndex].neighbor->portNum;
	} else {
		cport->neighborLid = 0;
		cport->neighborPort = 0;
	}
	cport->numVFs = port->Image[imageIndex].numVFs;
	cport->numGroups = port->Image[imageIndex].numGroups;
	cport->InternalBitMask = port->Image[imageIndex].InternalBitMask;

	cport->vlSelectMask = port->Image[imageIndex].vlSelectMask;
	cport->clearSelectMask.AsReg32 = port->Image[imageIndex].clearSelectMask.AsReg32;

	memcpy(&(cport->stlPortCounters), &(port->Image[imageIndex].StlPortCounters), sizeof(PmCompositePortCounters_t));
	memcpy(&(cport->stlVLPortCounters), &(port->Image[imageIndex].StlVLPortCounters), sizeof(PmCompositeVLCounters_t)*MAX_PM_VLS);
	memcpy(&(cport->DeltaStlPortCounters), &(port->Image[imageIndex].DeltaStlPortCounters), sizeof(PmCompositePortCounters_t));
	memcpy(&(cport->DeltaStlVLPortCounters), &(port->Image[imageIndex].DeltaStlVLPortCounters), sizeof(PmCompositeVLCounters_t)*MAX_PM_VLS);
	
	// Groups and VFs
	// these are actually lists of indexes into the list of groups/list of VFs 
	int x, y;
	for (x = 0; x < PM_MAX_GROUPS_PER_PORT; x++) {
		// use -1 to indicated no assignment
		cport->groups[x] = -1;
		PmGroup_t *group = port->Image[imageIndex].Groups[x];
		if (!group) {
			continue;
		}
		for (y = 0; y < PM_MAX_GROUPS; y++) {
			// look through all of the Groups that are listed in this composite
			PmCompositeGroup_t cgroup = cimg->groups[y];
			// if the group names match, then assign the index
			if (strcmp(cgroup.name, group->Name) == 0) {
				cport->groups[x] = y;
				break;
			}
		}
	}
	for (x = 0; x < MAX_VFABRICS; x++) {
		// use -1 to indicated no assignment
		cport->compVfVlmap[x].VF = -1;
		PmVF_t *VF = port->Image[imageIndex].vfvlmap[x].pVF;
		uint32 vlmask = port->Image[imageIndex].vfvlmap[x].vlmask;
		if (!VF) {
			continue;
		}

		PmCompositeVF_t cVF = cimg->VFs[x];

		// if the VF names match, then assign the index
		if (strcmp(cVF.name, VF->Name) == 0) {
			cport->compVfVlmap[x].VF = x;
			cport->compVfVlmap[x].vlmask = vlmask;
		} else {
			IB_LOG_WARN_FMT(__func__, "Composite VF name doesn't match current VF: VF %d, PM: %.*s, Composite %.*s",
					x, (int)sizeof(VF->Name), VF->Name, (int)sizeof(cVF.name), cVF.name);
		}
	}
	return cport;
}

/************************************************************************************* 
    createCompositeNode - Allocate and create a composite node
 
    Inputs:
    	node - the node that to use to create the composite node
    	cimg - the parent composite image
    	imageIndex - index into the images
 
    Returns:
    	The created composite node
 
    Returns and empty, non-null composite node if input node is null
 
*************************************************************************************/ 
static PmCompositeNode_t *createCompositeNode(PmNode_t *node, PmCompositeImage_t *cimg, uint64 imageIndex) {
	PmCompositeNode_t *cnode;

	cnode = calloc(1, sizeof(PmCompositeNode_t));
	if (!cnode) 
		return NULL;

	if (!node) 
		return cnode;	// return an empty (but non-null) composite node

	// set the values
	cnode->NodeGUID = node->NodeGUID;
	cnode->SystemImageGUID = node->SystemImageGUID;
	memcpy(&(cnode->nodeDesc), &(node->nodeDesc), sizeof(cnode->nodeDesc));
	cnode->lid = node->Image[imageIndex].lid;
	cnode->nodeType = node->nodeType;
	cnode->numPorts = node->numPorts;


	if (node->nodeType == STL_NODE_SW) {
		// use numPorts+1 because of port 0
		cnode->ports = calloc(1, (sizeof(cnode->ports)*(node->numPorts+1)));
		if (!(cnode->ports)) {
			free(cnode);
			return NULL;
		}
		int i;
		for (i = 0; i <= node->numPorts; i++) {
			cnode->ports[i] = createCompositePort(node->up.swPorts[i], cimg, imageIndex);
			if (!cnode->ports[i]) 
				goto cleanup;
			cimg->numPorts++;
		}
	} else {
		// only 1 port
		cnode->ports = calloc(1, sizeof(cnode->ports));
		if (!(cnode->ports)) {
			free(cnode);
			return NULL;
		}
		cnode->ports[0] = createCompositePort(node->up.caPortp, cimg, imageIndex);
		if (!cnode->ports[0])
			goto cleanup;

		cimg->numPorts++;
	}

	return cnode;

cleanup:
	if (cnode->nodeType == STL_NODE_SW) {
		int i;
		for (i = 0; i <= cnode->numPorts; i++) {
			if (cnode->ports[i]) {
				free(cnode->ports[i]);
				cnode->ports[i] = NULL;
			}
		}
		free(cnode->ports);
		cnode->ports = NULL;
	} else {
		if (cnode->ports[0]) {
			free(cnode->ports[0]);
			cnode->ports[0] = NULL;
		}
		free(cnode->ports);
		cnode->ports = NULL;
	}
	free(cnode);
	return NULL;
}

static void copyCompositeGroup(PmGroup_t *group, PmCompositeGroup_t *cgroup, uint64 imageIndex) {
	memcpy(cgroup->name, group->Name, STL_PM_GROUPNAMELEN);
	cgroup->numIntPorts = group->Image[imageIndex].NumIntPorts;
	cgroup->numExtPorts = group->Image[imageIndex].NumExtPorts;
	cgroup->minIntRate = group->Image[imageIndex].MinIntRate;
	cgroup->maxIntRate = group->Image[imageIndex].MaxIntRate;
	cgroup->minExtRate = group->Image[imageIndex].MinExtRate;
	cgroup->maxExtRate = group->Image[imageIndex].MaxExtRate;
	memcpy(&(cgroup->intUtil), &(group->Image[imageIndex].IntUtil), sizeof(cgroup->intUtil));
	memcpy(&(cgroup->sendUtil), &(group->Image[imageIndex].SendUtil), sizeof(cgroup->sendUtil));
	memcpy(&(cgroup->recvUtil), &(group->Image[imageIndex].RecvUtil), sizeof(cgroup->recvUtil));
	memcpy(&(cgroup->intErr), &(group->Image[imageIndex].IntErr), sizeof(cgroup->intErr));
	memcpy(&(cgroup->extErr), &(group->Image[imageIndex].ExtErr), sizeof(cgroup->extErr));
}

static void copyCompositeVF(PmVF_t *VF, PmCompositeVF_t *cVF, uint64 imageIndex) {
	memcpy(&(cVF->name), &(VF->Name), MAX_VFABRIC_NAME);
	cVF->isActive = VF->Image->isActive;
	cVF->numPorts = VF->Image->NumPorts;
	cVF->minIntRate = VF->Image->MinIntRate;
	cVF->maxIntRate = VF->Image->MaxIntRate;
	memcpy(&(cVF->intUtil), &(VF->Image->IntUtil), sizeof(cVF->intUtil));
	memcpy(&(cVF->intErr), &(VF->Image->IntErr), sizeof(cVF->intErr));
}

#ifndef __VXWORKS__
/************************************************************************************* 
*   setFilename - set the filename for a composite image
*  
*   Inputs:
*   	sth - ShortTermHistory, used to get the filepath
*   	cimg - the image whose name to set
*  
*   Returns:
*   	none
*************************************************************************************/
static void setFilename(PmShortTermHistory_t *sth, PmCompositeImage_t *cimg) {
	struct tm *now_tm;
	now_tm = gmtime((time_t *)&cimg->sweepStart);
	if (!now_tm) {
		// This should never happen
		IB_LOG_ERROR0("Unable to get current time");
		snprintf(cimg->header.common.filename, PM_HISTORY_FILENAME_LEN, "%s/c0.hist", sth->filepath);
		return;
	}
	if (cimg->header.common.isCompressed) {
		snprintf(cimg->header.common.filename, PM_HISTORY_FILENAME_LEN, "%s/c%4d%02d%02d%02d%02d%02d.zhist", sth->filepath, 
				now_tm->tm_year + 1900,
				now_tm->tm_mon+1,
				now_tm->tm_mday,
				now_tm->tm_hour,
				now_tm->tm_min,
				now_tm->tm_sec);
	} else {
		snprintf(cimg->header.common.filename, PM_HISTORY_FILENAME_LEN, "%s/c%4d%02d%02d%02d%02d%02d.hist", sth->filepath, 
				now_tm->tm_year + 1900,
				now_tm->tm_mon+1,
				now_tm->tm_mday,
				now_tm->tm_hour,
				now_tm->tm_min,
				now_tm->tm_sec);
	}
}
#endif

/************************************************************************************* 
*   PmFreeComposite - free a composite image
*  
*   Inputs:
*   	pool - the memory pool that the composite is allocated in
*   	cimg - the composite to free
*   Returns:
*   	FSUCCESS if okay
*************************************************************************************/
void PmFreeComposite(PmCompositeImage_t *cimg) {
	if (!cimg) {
		// no image 
		return;
	}
	if (cimg->nodes) {
		int i;
		for (i = 0; i <= cimg->maxLid; i++) {
			PmCompositeNode_t *cnode = cimg->nodes[i];
			if (!cnode) 
				continue;
			if (cnode->nodeType == STL_NODE_SW) {
				if (cnode->ports) {
					int j;
					for (j = 0; j <= cnode->numPorts; j++) {
						if (cnode->ports[j]) {
							free(cnode->ports[j]);
							cnode->ports[j] = NULL;
						}
					}
					free(cnode->ports);
					cnode->ports = NULL;
				}
			} else {
				if (cnode->ports) {
					if (cnode->ports[0]) {
						free(cnode->ports[0]);
						cnode->ports[0] = NULL;
					}
					free(cnode->ports);
					cnode->ports = NULL;
				}
			}
			free(cnode);
			cimg->nodes[i] = NULL;
		}
		free(cimg->nodes);
		cimg->nodes = NULL;
	}
	free(cimg);
	return;
}

/************************************************************************************* 
*   PmCreateComposite - Create a new composite image from a single image
*  
*   Inputs:
*   	pm - the PM
*   	imageIndex - the index of the image to use to create the composite
*   	isCompressed - the image is compressed
*  
*   Returns:
*		A pointer to the newly created composite
*  
*   Note: filename, recordID, timestamp and previousFileName are -not- handled by this function
*************************************************************************************/ 
PmCompositeImage_t *PmCreateComposite(Pm_t *pm, uint32 imageIndex, uint8 isCompressed) {
	PmCompositeImage_t *cimg;
	PmImage_t *img = &(pm->Image[pm->history[imageIndex]]);
	int i;

	cimg = calloc(1, sizeof(PmCompositeImage_t));
	if (!cimg)
		return NULL;

	// intialize some of the header fields
	cimg->header.common.historyVersion = PM_HISTORY_VERSION;
	cimg->header.common.isCompressed = isCompressed;
	cimg->header.common.imagesPerComposite = 1;
	cimg->header.common.imageSweepInterval = img->imageInterval;
	cimg->header.common.imageTime = (uint32)img->sweepStart;
	cimg->header.common.imageIDs[0] = buildShortTermHistoryImageId(pm, imageIndex);
	
	// initialize the composite values from the image
	cimg->sweepStart = (uint64)img->sweepStart;
	cimg->sweepDuration = img->sweepDuration;
	cimg->HFIPorts = img->HFIPorts;
	cimg->switchNodes = img->SwitchNodes;
	cimg->switchPorts = img->SwitchPorts;
	cimg->numLinks = img->NumLinks;
	cimg->numSMs = img->NumSMs;
	cimg->noRespNodes = img->NoRespNodes;
	cimg->noRespPorts = img->NoRespPorts;
	cimg->skippedNodes = img->SkippedNodes;
	cimg->skippedPorts = img->SkippedPorts;
	cimg->unexpectedClearPorts = img->UnexpectedClearPorts;
	cimg->downgradedPorts = img->DowngradedPorts;
	cimg->numGroups = pm->NumGroups;
	cimg->numVFs = pm->numVFs;
	cimg->numVFsActive = pm->numVFsActive;
	cimg->maxLid = img->maxLid;
	cimg->numPorts = 0;
	memcpy(cimg->SMs, img->SMs, sizeof(cimg->SMs[0]) * PM_HISTORY_MAX_SMS_PER_COMPOSITE);
	
	// copy all of the groups
	copyCompositeGroup(pm->AllPorts, &(cimg->allPortsGroup), pm->history[imageIndex]);
	for (i = 0; i < pm->NumGroups; i++) {
		PmGroup_t *group = pm->Groups[i];
		if (!group) 
			continue;
		copyCompositeGroup(group, &(cimg->groups[i]), pm->history[imageIndex]);
	}
	// do the same thing for VFs
	for (i = 0; i < pm->numVFs; i++) {
		PmVF_t *VF = pm->VFs[i];
		if (!VF)
			continue;
		copyCompositeVF(VF, &(cimg->VFs[i]), pm->history[imageIndex]);
	}
	// nodes
	// allocate the memory for the list of pointers first
	cimg->nodes = calloc(1, (sizeof(cimg->nodes)*(img->maxLid+1)));
	if (!cimg->nodes){
		goto error;
	}

	for (i = 0; i <= img->maxLid; i++) {
		cimg->nodes[i] = createCompositeNode(img->LidMap[i], cimg, pm->history[imageIndex]);
		if (!cimg->nodes[i]) {
			goto error;
		}
	}

	return cimg;

error:
	PmFreeComposite(cimg);
	return NULL;
}

/*************************************************************************************
 *	compoundDeltaPortCounters
 *
 *
 ************************************************************************************/
void compoundDeltaPortCounters(PmCompositePortCounters_t *dest, PmCompositePortCounters_t *src)
{
	dest->PortXmitData                += src->PortXmitData;
	dest->PortRcvData                 += src->PortRcvData;
	dest->PortXmitPkts                += src->PortXmitPkts;
	dest->PortRcvPkts                 += src->PortRcvPkts;
	dest->PortMulticastXmitPkts       += src->PortMulticastXmitPkts;
	dest->PortMulticastRcvPkts        += src->PortMulticastRcvPkts;
	dest->PortXmitWait                += src->PortXmitWait;
	dest->SwPortCongestion            += src->SwPortCongestion;
	dest->PortRcvFECN                 += src->PortRcvFECN;
	dest->PortRcvBECN                 += src->PortRcvBECN;
	dest->PortXmitTimeCong            += src->PortXmitTimeCong;
	dest->PortXmitWastedBW            += src->PortXmitWastedBW;
	dest->PortXmitWaitData            += src->PortXmitWaitData;
	dest->PortRcvBubble               += src->PortRcvBubble;
	dest->PortMarkFECN                += src->PortMarkFECN;
	dest->PortRcvConstraintErrors     += src->PortRcvConstraintErrors;
	dest->PortRcvSwitchRelayErrors    += src->PortRcvSwitchRelayErrors;
	dest->PortXmitDiscards            += src->PortXmitDiscards;
	dest->PortXmitConstraintErrors    += src->PortXmitConstraintErrors;
	dest->PortRcvRemotePhysicalErrors += src->PortRcvRemotePhysicalErrors;
	dest->LocalLinkIntegrityErrors    += src->LocalLinkIntegrityErrors;
	dest->PortRcvErrors               += src->PortRcvErrors;
	dest->ExcessiveBufferOverruns     += src->ExcessiveBufferOverruns;
	dest->FMConfigErrors              += src->FMConfigErrors;
	dest->LinkErrorRecovery           += src->LinkErrorRecovery;
	dest->LinkDowned                  += src->LinkDowned;
	dest->UncorrectableErrors         += src->UncorrectableErrors;

	// Use MIN LQI for Delta
	UPDATE_MIN(dest->lq.s.LinkQualityIndicator, src->lq.s.LinkQualityIndicator);
	// Use MAX Number of Lanes Down for Delta
	UPDATE_MAX(dest->lq.s.NumLanesDown, src->lq.s.NumLanesDown);

}
/*************************************************************************************
 *	compoundPort
 *
 *
 ************************************************************************************/
void compoundDeltaVLCounters(PmCompositeVLCounters_t *dest, PmCompositeVLCounters_t *src, uint8 numVls)
{
	uint8 i;
	for (i = 0; i < numVls; i++) {
		dest[i].PortVLXmitData     += src[i].PortVLXmitData;
		dest[i].PortVLRcvData      += src[i].PortVLRcvData;
		dest[i].PortVLXmitPkts     += src[i].PortVLXmitPkts;
		dest[i].PortVLRcvPkts      += src[i].PortVLRcvPkts;
		dest[i].PortVLXmitWait     += src[i].PortVLXmitWait;
		dest[i].SwPortVLCongestion += src[i].SwPortVLCongestion;
		dest[i].PortVLRcvFECN      += src[i].PortVLRcvFECN;
		dest[i].PortVLRcvBECN      += src[i].PortVLRcvBECN;
		dest[i].PortVLXmitTimeCong += src[i].PortVLXmitTimeCong;
		dest[i].PortVLXmitWastedBW += src[i].PortVLXmitWastedBW;
		dest[i].PortVLXmitWaitData += src[i].PortVLXmitWaitData;
		dest[i].PortVLRcvBubble    += src[i].PortVLRcvBubble;
		dest[i].PortVLMarkFECN     += src[i].PortVLMarkFECN;
		dest[i].PortVLXmitDiscards += src[i].PortVLXmitDiscards;
	}
}
/*************************************************************************************
 *	compoundDeltaVLCounters
 *
 *
 ************************************************************************************/
static void compoundPort(PmPort_t *port, PmCompositePort_t *cport, uint32 imageIndex)
{
	cport->stlPortCounters.lq.s.LinkQualityIndicator =
		MIN(cport->stlPortCounters.lq.s.LinkQualityIndicator,
			port->Image[imageIndex].StlPortCounters.lq.s.LinkQualityIndicator);

	compoundDeltaPortCounters(&cport->DeltaStlPortCounters, &port->Image[imageIndex].DeltaStlPortCounters);
	compoundDeltaVLCounters(&cport->DeltaStlVLPortCounters[0], &port->Image[imageIndex].DeltaStlVLPortCounters[0], MAX_PM_VLS);
}

/*************************************************************************************
*   compoundImage - Compound the data from an image into an existing composite
*
*   Inputs:
*   	pm - the PM
*   	imageIndex - the index of the image to compound
*   	cimg - the composite image
*
*   Returns:
*   	Status - FSUCCESS if okay
*
*   Note: cimg needs to have already been created (with PmCreateComposite)
*************************************************************************************/
FSTATUS compoundImage(Pm_t *pm, uint32 imageIndex, PmCompositeImage_t *cimg) {
	PmImage_t *img = &(pm->Image[pm->history[imageIndex]]);

	// make sure the composite has already been created
	if (!cimg || cimg->header.common.imagesPerComposite == 0)
		return FINVALID_PARAMETER;

	// update the header
	cimg->header.common.imageIDs[cimg->header.common.imagesPerComposite] = buildShortTermHistoryImageId(pm, imageIndex);
	cimg->header.common.imageSweepInterval += img->imageInterval;

	//store average sweep duration
	cimg->sweepDuration = (cimg->sweepDuration * cimg->header.common.imagesPerComposite + img->sweepDuration)
		                 /(cimg->header.common.imagesPerComposite + 1);

	// groups
	int i;
	for (i = 0; i < PM_MAX_GROUPS; i++) {
		PmCompositeGroup_t *cgroup = &(cimg->groups[i]);
		PmGroup_t *group = pm->Groups[i];
		if (strcmp(cgroup->name, group->Name)) {
			IB_LOG_WARN_FMT(__func__, "Group names are different- Expected: %s, Got: %s", cgroup->name, group->Name);
			memcpy(cgroup->name, group->Name, STL_PM_GROUPNAMELEN);
		}
	}

	// VFs
	for (i = 0; i < MAX_VFABRICS; i++) {
		PmCompositeVF_t *cVF = &(cimg->VFs[i]);
		PmVF_t *VF = pm->VFs[i];
		if (strcmp(cVF->name, VF->Name)) {
			IB_LOG_WARN_FMT(__func__, "VF Names are different: %s %s", cVF->name, VF->Name);
			memcpy(cVF->name, VF->Name, MAX_VFABRIC_NAME);
		}
		cVF->isActive = VF->Image[pm->history[imageIndex]].isActive;
	}

	// Nodes/Ports
	STL_LID lid;
	int j;
	for (lid = 1; lid <= MIN(cimg->maxLid, img->maxLid); ++lid){
		PmCompositeNode_t *cnode = cimg->nodes[lid];
		PmNode_t *node = img->LidMap[lid];

		if (!node || !cnode || (cnode->NodeGUID != node->NodeGUID)) continue;

		for (j = 0; j < cnode->numPorts; ++j){
			PmCompositePort_t *cport = cnode->ports[j];
			PmPort_t *port = node->nodeType == STL_NODE_SW ? node->up.swPorts[j] : node->up.caPortp;

			if (!port || !cport) continue;

			compoundPort(port, cport, imageIndex);
		}
	}

	// increment the number of images in this composite
	cimg->header.common.imagesPerComposite++;
	return FSUCCESS;
}

/************************************************************************************* 
    clearLoadedImage - Clear out the image that is currently loaded in short term history

    Inputs:
    	sth - Short Term History

*************************************************************************************/ 
void clearLoadedImage(PmShortTermHistory_t *sth) {
	int i;
	if (sth->LoadedImage.img) {
		if (sth->LoadedImage.img->LidMap) {
			for (i = 0; i <= sth->LoadedImage.img->maxLid; i++) {
				PmNode_t *pmnodep = sth->LoadedImage.img->LidMap[i];
				if (!pmnodep) 
					continue;
				if (pmnodep->nodeType == STL_NODE_SW) {
					if (pmnodep->up.swPorts) {
						int j;
						for (j = 0; j <= pmnodep->numPorts; j++) {
							PmPort_t *pmportp = pmnodep->up.swPorts[j];
							if (pmportp) {
								free(pmportp);
								pmnodep->up.swPorts[j] = NULL;
							}
						}
						free(pmnodep->up.swPorts);
						pmnodep->up.swPorts = NULL;
					}
				} else {
					if (pmnodep->up.caPortp) {
						free(pmnodep->up.caPortp);
						pmnodep->up.caPortp = NULL;
					}
		 		}
				free(pmnodep);
				sth->LoadedImage.img->LidMap[i] = NULL;
			}
			free(sth->LoadedImage.img->LidMap);
			sth->LoadedImage.img->LidMap = NULL;
		}
		free(sth->LoadedImage.img);
		sth->LoadedImage.img = NULL;
	}
	if (sth->LoadedImage.AllGroup) {
		free(sth->LoadedImage.AllGroup);
		sth->LoadedImage.AllGroup = NULL;
	}
	for (i = 0; i < PM_MAX_GROUPS; i++) {
		if (sth->LoadedImage.Groups[i]) {
			free(sth->LoadedImage.Groups[i]);
			sth->LoadedImage.Groups[i] = NULL;
		}
	}
	for (i = 0; i < MAX_VFABRICS; i++) {
		if (sth->LoadedImage.VFs[i]) {
			free(sth->LoadedImage.VFs[i]);
			sth->LoadedImage.VFs[i] = NULL;
		}
	}
}

/************************************************************************************* 
    PmReconstituteVFImage - Reconstitute a compoiste VF into a VF
 
    Inputs:
    	cVF - the composite VF
    	
    Returns:
    	The reconstituted VF
 
*************************************************************************************/ 
PmVF_t *PmReconstituteVFImage(PmCompositeVF_t *cVF) {
	PmVF_t *pmVFP;
	pmVFP = calloc(1, sizeof(PmVF_t));
	if (!pmVFP) {
		IB_LOG_ERROR0("Unable to allocate memory to reconstitute PM VF Image");
		return NULL;
	}
	cs_strlcpy(pmVFP->Name, cVF->name, MAX_VFABRIC_NAME);
	pmVFP->Image[0].isActive = cVF->isActive;
	pmVFP->Image[0].NumPorts = cVF->numPorts;
	pmVFP->Image[0].MinIntRate = cVF->minIntRate;
	pmVFP->Image[0].MaxIntRate = cVF->maxIntRate;
	memcpy(&(pmVFP->Image[0].IntUtil), &(cVF->intUtil), sizeof(PmUtilStats_t));
	memcpy(&(pmVFP->Image[0].IntErr), &(cVF->intErr), sizeof(PmErrStats_t));
	return pmVFP;
}

/************************************************************************************* 
    PmReconstituteGroupImage - Reconstitute a compoiste group into a group
 
    Inputs:
    	cgroup - the composite group
    	
    Returns:
    	The reconstituted group
 
*************************************************************************************/ 
PmGroup_t *PmReconstituteGroupImage(PmCompositeGroup_t *cgroup)  {
	PmGroup_t *pmGroupP;
	pmGroupP = calloc(1, sizeof(PmGroup_t));
	if (!pmGroupP) {
		IB_LOG_ERROR0("Unable to allocate memory to reconstitute PM Group Image");
		return NULL;
	}
	cs_strlcpy(pmGroupP->Name, cgroup->name, STL_PM_GROUPNAMELEN);
	pmGroupP->Image[0].NumIntPorts = cgroup->numIntPorts;
	pmGroupP->Image[0].NumExtPorts = cgroup->numExtPorts;
	pmGroupP->Image[0].MinIntRate = cgroup->minIntRate;
	pmGroupP->Image[0].MaxIntRate = cgroup->maxIntRate;
	pmGroupP->Image[0].MinExtRate = cgroup->minExtRate;
	pmGroupP->Image[0].MaxExtRate = cgroup->maxExtRate;
	memcpy(&(pmGroupP->Image[0].IntUtil), &(cgroup->intUtil), sizeof(PmUtilStats_t));
	memcpy(&(pmGroupP->Image[0].SendUtil), &(cgroup->sendUtil), sizeof(PmUtilStats_t));
	memcpy(&(pmGroupP->Image[0].RecvUtil), &(cgroup->recvUtil), sizeof(PmUtilStats_t));
	memcpy(&(pmGroupP->Image[0].IntErr), &(cgroup->intErr), sizeof(PmErrStats_t));
	memcpy(&(pmGroupP->Image[0].ExtErr), &(cgroup->extErr), sizeof(PmErrStats_t));

	return pmGroupP;
}

/************************************************************************************* 
    PmReconstitutePortImage - Reconstitute a compoiste port into a port
 
    Inputs:
	    sth - PM Short Term History
    	cport - the composite port
    	
    Returns:
    	The reconstituted port
 
    Note:
    	Groups and VFs should have already been reconstituted into the loaded
    	image in order for them to get properly assigned for this port
 
*************************************************************************************/ 
PmPort_t *PmReconstitutePortImage(PmShortTermHistory_t *sth, PmCompositePort_t *cport) {
	PmPort_t *pmportp;
	pmportp = calloc(1, sizeof(PmPort_t)); 
	if (!pmportp) {
		IB_LOG_ERROR0("Unable to allocate memory to reconstitute PM Port Image");
		return NULL;
	}
	pmportp->guid = cport->guid;
	pmportp->portNum = cport->portNum;
	pmportp->Image[0].u.AsReg32 = cport->u.AsReg32;
	// neighbors will be handled later, store the lid/port in the temp fields
	pmportp->neighbor_lid = cport->neighborLid;
	pmportp->neighbor_portNum = cport->neighborPort;
	pmportp->Image[0].numVFs = cport->numVFs;
	pmportp->Image[0].numGroups = cport->numGroups;
	pmportp->Image[0].InternalBitMask = cport->InternalBitMask;
	// groups and VFS
	int i;
	for (i = 0; i < PM_MAX_GROUPS_PER_PORT; i++) {
		if (cport->groups[i] != (uint8)-1) { // -1 means no group
			pmportp->Image[0].Groups[i] = sth->LoadedImage.Groups[cport->groups[i]];
		} else {
			pmportp->Image[0].Groups[i] = NULL;
		}
	}
	uint8 sthVFidx;
	for (i=0; i<MAX_VFABRICS; i++) {
		sthVFidx = cport->compVfVlmap[i].VF;
		if (sthVFidx == (uint8) -1) {
			pmportp->Image[0].vfvlmap[i].pVF =  NULL;
		} else if (sthVFidx >= MAX_VFABRICS) {
			IB_LOG_ERROR_FMT(__func__,"Invalid VF index %d",sthVFidx);
			pmportp->Image[0].vfvlmap[i].pVF =  NULL;
		} else {
			pmportp->Image[0].vfvlmap[i].vlmask = cport->compVfVlmap[i].vlmask;
			pmportp->Image[0].vfvlmap[i].pVF = sth->LoadedImage.VFs[sthVFidx];
		}
	}

	pmportp->Image[0].vlSelectMask = cport->vlSelectMask;
	pmportp->Image[0].clearSelectMask.AsReg32 = cport->clearSelectMask.AsReg32;
	// counters
	memcpy(&(pmportp->Image[0].StlPortCounters), &(cport->stlPortCounters), sizeof(PmCompositePortCounters_t));
	memcpy(&(pmportp->Image[0].StlVLPortCounters), &(cport->stlVLPortCounters), sizeof(PmCompositeVLCounters_t) * MAX_PM_VLS);

	memcpy(&(pmportp->Image[0].DeltaStlPortCounters), &(cport->DeltaStlPortCounters), sizeof(PmCompositePortCounters_t));
	memcpy(&(pmportp->Image[0].DeltaStlVLPortCounters), &(cport->DeltaStlVLPortCounters), sizeof(PmCompositeVLCounters_t) * MAX_PM_VLS);

	return pmportp;
}

/************************************************************************************* 
    PmReconstituteNodeImage - Reconstitute a compoiste node into a node
 
    Inputs:
	    sth - PM Short Term History
    	cnode - the composite node
    	
    Returns:
    	The reconstituted node
 
*************************************************************************************/ 
PmNode_t *PmReconstituteNodeImage(PmShortTermHistory_t *sth, PmCompositeNode_t *cnode) {
	PmNode_t *pmnodep;

	if (!cnode->NodeGUID) {
		// if there is no guid, then it was a NULL node
		return NULL;
	}

	pmnodep = calloc(1, sizeof(PmNode_t));
	if (!pmnodep) {
		IB_LOG_ERROR0("Unable to allocate memory to reconstitute a Node Image");
		return NULL;
	}

	memcpy(&pmnodep->nodeDesc, &cnode->nodeDesc, sizeof(STL_NODE_DESCRIPTION));
	pmnodep->nodeType = cnode->nodeType;
	pmnodep->numPorts = cnode->numPorts;
	pmnodep->NodeGUID = cnode->NodeGUID;
	pmnodep->SystemImageGUID = cnode->SystemImageGUID;
	pmnodep->Image[0].lid = cnode->lid;


	// ports
	if (cnode->ports) {			
		if (pmnodep->nodeType == STL_NODE_SW) {
			pmnodep->up.swPorts = calloc(1, sizeof(PmPort_t*)*(pmnodep->numPorts+1));
			if (!pmnodep->up.swPorts) {
				free(pmnodep);
				return NULL;
			}
			int i;
			for (i = 0; i <= pmnodep->numPorts; i++) {
				if (!cnode->ports[i]) 
					continue;
				pmnodep->up.swPorts[i] = PmReconstitutePortImage(sth, cnode->ports[i]);
				if (!pmnodep->up.swPorts[i]) {
					//reconstituting the port failed!!!
					goto cleanup;
				}
			}
		} else {
			pmnodep->up.caPortp = PmReconstitutePortImage(sth, cnode->ports[0]);
			if (!pmnodep->up.caPortp) {
				//reconstituting the port failed!!!
				goto cleanup;
			}
		}
	}
	return pmnodep;

cleanup:
	if (pmnodep->nodeType == STL_NODE_SW) {
		if (pmnodep->up.swPorts) {
			int j;
			for (j = 0; j <= pmnodep->numPorts; j++) {
				PmPort_t *pmportp = pmnodep->up.swPorts[j];
				if (pmportp) {
					free(pmportp);					
					pmnodep->up.swPorts[j] = NULL;
				}
			}
			free(pmnodep->up.swPorts);
			pmnodep->up.swPorts = NULL;
		}
	} else {
		if (pmnodep->up.caPortp) {
			free(pmnodep->up.caPortp);	
			pmnodep->up.caPortp = NULL;
		}
	}
	free(pmnodep);
	return NULL;
}

/************************************************************************************* 
*  reconstituteImage - convert a composite image into a 'regular' image
*  
*   Inputs:
*   	cimg - the composite to convert
*  
*   Returns:
*   	Status - FSUCCESS if okay
* 
*************************************************************************************/
PmImage_t *PmReconstituteImage(PmShortTermHistory_t *sth, PmCompositeImage_t *cimg) {
	PmImage_t *img;
	img = calloc(1, sizeof(PmImage_t));
	if (!img) {
		IB_LOG_ERROR0("Unable to allocate memory to reconstitute image");
		return NULL;
	}
	img->maxLid = cimg->maxLid;
	img->sweepStart = (time_t)cimg->sweepStart;
	img->sweepDuration = cimg->sweepDuration;
	img->imageInterval = cimg->header.common.imageSweepInterval;
	img->HFIPorts = cimg->HFIPorts;
	img->SwitchNodes = cimg->switchNodes;
	img->SwitchPorts = cimg->switchPorts;
	img->NumLinks = cimg->numLinks;
	img->NumSMs = cimg->numSMs;
	memcpy(img->SMs, cimg->SMs, sizeof(img->SMs[0]) * 2);
	img->NoRespNodes = cimg->noRespNodes;
	img->NoRespPorts = cimg->noRespPorts;
	img->SkippedNodes = cimg->skippedNodes;
	img->SkippedPorts = cimg->skippedPorts;
	img->UnexpectedClearPorts  = cimg->unexpectedClearPorts;
	img->DowngradedPorts  = cimg->downgradedPorts;
	// lid map
	img->LidMap = calloc(1, sizeof(PmNode_t*)*(img->maxLid+1));
	if (!img->LidMap) {
		free(img);
		return NULL;
	}
	img->lidMapSize = img->maxLid + 1;

	int i;
	for (i = 0; i <= img->maxLid; i++) {
		img->LidMap[i] = PmReconstituteNodeImage(sth, cimg->nodes[i]);
	}

	return img;
}

/************************************************************************************* 
    PmReconstitute - Reconstitute a compoiste image into a PM image
 
    Inputs:
    	sth - Short Term History where the image will be loaded
    	cimg - the image to reconstitute
    	
    Returns:
    	FSUCCESS if okay
 
*************************************************************************************/ 
FSTATUS PmReconstitute(PmShortTermHistory_t *sth, PmCompositeImage_t *cimg) {
	int i;

	// clear out whatever is in there first
	clearLoadedImage(sth);

	sth->LoadedImage.AllGroup = PmReconstituteGroupImage(&cimg->allPortsGroup);
	if (!sth->LoadedImage.AllGroup) {
		goto cleanup;
	}
	for (i = 0; i < PM_MAX_GROUPS; i++) {
		sth->LoadedImage.Groups[i] = PmReconstituteGroupImage(&cimg->groups[i]);
		if (!sth->LoadedImage.Groups[i]) {
			goto cleanup;
		}
	}
	for (i = 0; i < MAX_VFABRICS; i++) {
		sth->LoadedImage.VFs[i] = PmReconstituteVFImage(&cimg->VFs[i]);
		if (!sth->LoadedImage.VFs[i]) {
			goto cleanup;
		}
	}
	// convert the data in cimg to image data
	sth->LoadedImage.img = PmReconstituteImage(sth, cimg);
	if (!sth->LoadedImage.img) {
		goto cleanup;
	}

	// need to go back and fill in the neighbor info for each port
	for (i = 0; i <= sth->LoadedImage.img->maxLid; i++) {
		PmNode_t *pmnodep = sth->LoadedImage.img->LidMap[i];
		if (!pmnodep) 
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			int j;
			for (j = 0; j <= pmnodep->numPorts; j++) {
				PmPort_t *pmportp = pmnodep->up.swPorts[j];
				if (!pmportp) 
					continue;
				// find the neighbor
				if (pmportp->neighbor_lid != 0) {
					pmportp->Image[0].neighbor = pm_find_port(sth->LoadedImage.img, pmportp->neighbor_lid, pmportp->neighbor_portNum);
					// I supposed it is okay for neighbor to be NULL? 
				}
				// set the node
				pmportp->pmnodep = pmnodep;
			}
		} else {
			PmPort_t *pmportp= pmnodep->up.caPortp;
			if (!pmportp) 
				continue;
			if (pmportp->neighbor_lid != 0) {
				pmportp->Image[0].neighbor = pm_find_port(sth->LoadedImage.img, pmportp->neighbor_lid, pmportp->neighbor_portNum);
			}
			pmportp->pmnodep = pmnodep;
		}
	}
	return FSUCCESS;

cleanup:
	clearLoadedImage(sth);
	return FERROR;
}

/************************************************************************************* 
*	markImagesAsDisk - Mark composite images / historyRecords as originating from disk
*
*	Inputs:
*		ch - PmHistoryHeaderCommon_t from composite image or History Record.
*
*	This function will modify imageIDs contained in the the given header structure 
*	to indicate they originated from a disk stored image. 
*************************************************************************************/ 
static void markImagesAsDisk(PmHistoryHeaderCommon_t *ch)
{
	size_t i;

	for (i = 0; i < PM_HISTORY_MAX_IMAGES_PER_COMPOSITE; ++i) {
		ImageId_t temp = {.AsReg64 = ch->imageIDs[i] };
		if (temp.AsReg64) {
			temp.s.type = IMAGEID_TYPE_HISTORY_DISK;
			ch->imageIDs[i] = temp.AsReg64;
		}
	}
}
/************************************************************************************* 
*   loadComposite - load a composite image from a file
*  
*   Inputs:
*   	pm - the PM
*   	record - the history record corresponding to the composite to load
*   	cimg - pointer to the composite to be loaded
*  
*   Returns:
*   	FSUCCESS if okay
*  
*   This function will load the composite image from the file pointed to by 'record'
*   into 'cimg'. 
*************************************************************************************/
FSTATUS PmLoadComposite(Pm_t *pm, PmHistoryRecord_t *record, PmCompositeImage_t **cimg) {
	FILE *fp;
	unsigned char *raw_data, *img_data;
	size_t raw_len, img_len;
	FSTATUS ret = FSUCCESS;
	uint32 history_version;
#ifndef __VXWORKS__
	char errbuf[256]; 
#endif

	fp = fopen(record->header.filename, "rb");
	if (!fp) {
#ifdef __VXWORKS__
		IB_LOG_ERROR0("Unable to open PM history file");
#else
		if (errno) {
			strerror_r(errno, errbuf, sizeof(errbuf));
		} else {
			snprintf(errbuf,sizeof(errbuf),"Unknown error");
		}
		IB_LOG_ERROR_FMT(__func__, "Unable to open PM history file %s: %d/%s", 
			record->header.filename, errno, errbuf);
#endif
		return FNOT_FOUND;
	}

	// find out how big the file is
	fseek(fp, 0, SEEK_END);
	raw_len = ftell(fp);
	rewind(fp);

	// allocate the raw data
	raw_data = calloc(1, sizeof(unsigned char) * raw_len);
	if (!raw_data) {
		IB_LOG_ERROR0("Unable to allocate PM History image raw data");
		fclose(fp);
		return FINSUFFICIENT_MEMORY;
	}
	//read the raw data
	if (fread(raw_data, 1, raw_len, fp)!=raw_len || ferror(fp) ) {
#ifdef __VXWORKS__
		IB_LOG_ERROR0("Error reading PM History file");
#else
		if (ferror(fp)) {
			strerror_r(errno, errbuf, sizeof(errbuf));
		} else {
			snprintf(errbuf,sizeof(errbuf),"Short read");
		}
		IB_LOG_ERROR_FMT(__func__, "Error reading PM History file %s: %s",
			record->header.filename,errbuf); 
#endif
		free(raw_data);
		fclose(fp);
		return FERROR;
	}

	// check the version
	history_version = ((PmFileHeader_t *)raw_data)->common.historyVersion;
	if (history_version != PM_HISTORY_VERSION && history_version != PM_HISTORY_VERSION_OLD) {
#ifdef __VXWORKS__
		IB_LOG_ERROR0("Loaded PM history image does not match current supported versions");
#else
		IB_LOG_ERROR_FMT(__func__,
			"Loaded PM history image (%s, v%u.%u) does not match current supported versions: v%u.%u or v%u.%u",
			record->header.filename, ((history_version >> 24) & 0xFF), (history_version & 0x00FFFFFF),
			((PM_HISTORY_VERSION >> 24) & 0xFF), (PM_HISTORY_VERSION & 0x00FFFFFF),
			((PM_HISTORY_VERSION_OLD >> 24) & 0xFF), (PM_HISTORY_VERSION_OLD & 0x00FFFFFF));
#endif
		free(raw_data);
		fclose(fp);
		return FERROR;
	}

	// allocate the img_data buffer
	img_len = (size_t)((PmFileHeader_t*)raw_data)->flatSize;
	// checkout the flat size - it needs to be at least enough to hold the image header
	if (img_len < sizeof(PmFileHeader_t)) {
#ifdef __VXWORKS__
		IB_LOG_ERROR0("Invalid history file");
#else
		IB_LOG_ERROR_FMT(__func__, "Invalid history file %s",
			record->header.filename);
#endif
		fclose(fp);
		free(raw_data);
		return FERROR;
	}
	img_data = calloc(1, img_len); 	
	if (!img_data) {
		IB_LOG_ERROR0("Unable to allocate flat PM History Image");
		free(raw_data);
		fclose(fp);
		return FINSUFFICIENT_MEMORY;
	}
	// check to see if the data is compressed
	if (((PmFileHeader_t*)raw_data)->common.isCompressed) {
#ifndef __VXWORKS__
		// copy the header first
		memcpy(img_data, raw_data, sizeof(PmFileHeader_t));
		// decompress the data into the image data buffer
		ret = decompressAndReassemble(raw_data + sizeof(PmFileHeader_t),
									  raw_len - sizeof(PmFileHeader_t), 
									  ((PmFileHeader_t*)raw_data)->numDivisions,
									  ((PmFileHeader_t*)raw_data)->divisionSizes,
									  img_data + sizeof(PmFileHeader_t), 
									  img_len - sizeof(PmFileHeader_t));
		// free raw data now that it is not being used
		free(raw_data);
		if (ret != FSUCCESS) {
			IB_LOG_ERRORRC("Unable to decompress PM History Image rc:", ret);
			goto end;
		}
#else
		IB_LOG_ERROR0("Unable to decompress PM history Image");
		goto end;
#endif
	} else {
		// raw data is image data
		memcpy(img_data, raw_data, img_len);
		// free raw data now that it is not being used
		free(raw_data);
	}

	*cimg = calloc(1 , sizeof(PmCompositeImage_t));
	if (!(*cimg)) {
		IB_LOG_ERROR0("Unable to allocate memory for the cached PM Composite Image");
		ret = FINSUFFICIENT_MEMORY;
		goto end;
	}
	memcpy(*cimg, img_data, sizeof(PmFileHeader_t));
	// rebuild the composite from the data buffer
	ret = rebuildComposite(*cimg, img_data+sizeof(PmFileHeader_t), history_version);
	if (ret != FSUCCESS) {
		IB_LOG_ERRORRC("Error rebuilding PM Composite History Image rc:", ret);
		PmFreeComposite(*cimg);
		*cimg = NULL;
		ret = FNOT_FOUND;
	}
	markImagesAsDisk(&((*cimg)->header.common));
end:
	free(img_data);	
	fclose(fp);

	return ret;
}

/************************************************************************************* 
    PmFreezeComposite - 'Freeze' a composite image by caching it
 
    Inputs:
    	pm - the PM
    	record - the recond of the image to cache
    	
    Returns:
    	FSUCCESS if okay
 
    Note:
    	If there is already a cached composite, it will be freed before the
    	new composite is loaded
 
*************************************************************************************/ 
FSTATUS PmFreezeComposite(Pm_t *pm, PmHistoryRecord_t *record) {
	if (pm->ShortTermHistory.cachedComposite) { 
		PmFreeComposite(pm->ShortTermHistory.cachedComposite);
		pm->ShortTermHistory.cachedComposite = NULL;
	}
	return PmLoadComposite(pm, record, &pm->ShortTermHistory.cachedComposite);
}
/************************************************************************************* 
    PmFreezeComposite - 'Freeze' a the current composite image by caching it
 
    Inputs:
    	pm - the PM
    	
    Returns:
    	FSUCCESS if okay
 
    Note:
    	If there is already a cached composite, it will be freed before the
    	new composite is loaded
 
*************************************************************************************/ 
FSTATUS PmFreezeCurrent(Pm_t *pm) {
	if (pm->ShortTermHistory.cachedComposite) {
		PmFreeComposite(pm->ShortTermHistory.cachedComposite);
		pm->ShortTermHistory.cachedComposite = NULL;
	}
	
	// we need to copy the entire current composite into the cached composite
	pm->ShortTermHistory.cachedComposite = calloc(1, sizeof(PmCompositeImage_t));
	if (!pm->ShortTermHistory.cachedComposite) {
		return FINSUFFICIENT_MEMORY;
	}
	// check that current composite is valid
	if (!pm->ShortTermHistory.currentComposite) {
		IB_LOG_ERROR_FMT(__func__, "Trying to freeze current composite, but it is missing");
		return FERROR;
	}
	memcpy(pm->ShortTermHistory.cachedComposite, pm->ShortTermHistory.currentComposite, sizeof(PmCompositeImage_t) - sizeof(PmCompositeNode_t **)); 

	// allocate the node pointer array
	pm->ShortTermHistory.cachedComposite->nodes = calloc(1, (sizeof(pm->ShortTermHistory.cachedComposite->nodes)*(pm->ShortTermHistory.cachedComposite->maxLid+1)));
	if (!pm->ShortTermHistory.cachedComposite->nodes) {
		return FINSUFFICIENT_MEMORY;
	}
	// check that node array is valid
	if (!pm->ShortTermHistory.currentComposite->nodes) {
		IB_LOG_ERROR_FMT(__func__, "Trying to freeze current composite, but it has no nodes");
		return FERROR;
	}
	// fill in the nodes
	int i;
	for (i = 0; i <= pm->ShortTermHistory.cachedComposite->maxLid; i++) {
		if (!pm->ShortTermHistory.currentComposite->nodes[i]) {
			IB_LOG_ERROR_FMT(__func__, "Trying to freeze current composite, but it is missing node at index %d", i);
			return FERROR;
		}
		pm->ShortTermHistory.cachedComposite->nodes[i] = calloc(1, sizeof(PmCompositeNode_t)); 
		if (!pm->ShortTermHistory.cachedComposite->nodes[i]) {
			return FINSUFFICIENT_MEMORY;
		}
		memcpy(pm->ShortTermHistory.cachedComposite->nodes[i], pm->ShortTermHistory.currentComposite->nodes[i], (sizeof(PmCompositeNode_t) - sizeof(PmCompositePort_t**)));
		if (pm->ShortTermHistory.currentComposite->nodes[i]->ports) {
			if (pm->ShortTermHistory.cachedComposite->nodes[i]->nodeType == STL_NODE_SW) {
				// node is a switch
				pm->ShortTermHistory.cachedComposite->nodes[i]->ports = calloc(1, (sizeof(pm->ShortTermHistory.cachedComposite->nodes[i]->ports) * (pm->ShortTermHistory.cachedComposite->nodes[i]->numPorts + 1))); 
				if (!pm->ShortTermHistory.cachedComposite->nodes[i]->ports) {
					return FINSUFFICIENT_MEMORY;
				}
				// fill in the ports
				int j;
				for (j = 0; j <= pm->ShortTermHistory.cachedComposite->nodes[i]->numPorts; j++) {
					if (!pm->ShortTermHistory.currentComposite->nodes[i]->ports[j]) {
						IB_LOG_ERROR_FMT(__func__, "Trying to freeze current composite at node index %d, but it is missing port %d", i, j);
						return FERROR;
					}
					pm->ShortTermHistory.cachedComposite->nodes[i]->ports[j] = calloc(1, sizeof(PmCompositePort_t)); 
					if (!pm->ShortTermHistory.cachedComposite->nodes[i]->ports[j]) {
						return FINSUFFICIENT_MEMORY;
					}
					memcpy(pm->ShortTermHistory.cachedComposite->nodes[i]->ports[j], pm->ShortTermHistory.currentComposite->nodes[i]->ports[j], sizeof(PmCompositePort_t));
				}
			} else {
				// node is an hfi
				pm->ShortTermHistory.cachedComposite->nodes[i]->ports = calloc(1, sizeof(pm->ShortTermHistory.cachedComposite->nodes[i]->ports)); 
				if (!pm->ShortTermHistory.cachedComposite->nodes[i]->ports) {
					return FINSUFFICIENT_MEMORY;
				}
				pm->ShortTermHistory.cachedComposite->nodes[i]->ports[0] = calloc(1, sizeof(PmCompositePort_t));
				if (!pm->ShortTermHistory.cachedComposite->nodes[i]->ports[0]) {
					return FINSUFFICIENT_MEMORY;
				}
				memcpy(pm->ShortTermHistory.cachedComposite->nodes[i]->ports[0], pm->ShortTermHistory.currentComposite->nodes[i]->ports[0], sizeof(PmCompositePort_t));
			}
		}

	}
	return FSUCCESS;
}

#ifndef __VXWORKS__

/************************************************************************************* 
 * 	pruneOneStoredHistoryFile - removes a single stored history file
 *
 *	Input/Output:
 *		pSth - The PM's shortTermHistory object
 *		idx  - Index of the entry to remove
 * 	Returns
 * 		Status - VSTATUS_OK if okay
 * 		Status - VSTATUS_EIO if the referenced file couldn't be removed
*************************************************************************************/
static Status_t pruneOneStoredHistoryFile(PmShortTermHistory_t *pSth, uint32 idx)
{
	struct stat fileInfo;
	int i = 0;
	char *filename;

	// find the record, adjusting for the fact that we may have invalid files
	PmHistoryRecord_t *rec = pSth->historyRecords[(idx + (pSth->totalHistoryRecords - pSth->oldestInvalid)) % pSth->totalHistoryRecords];
	if (rec->index == INDEX_NOT_IN_USE && pSth->oldestInvalid != 0) {
		// Entry is already free, nothing to do
		return VSTATUS_OK;
	}

	// if oldestInvalid == totalHistoryRecords then there are no invalid files
	if (pSth->oldestInvalid != pSth->totalHistoryRecords && // if oldestInvalid == totalHistoryRecords then there are no invalid files
			pSth->invalidFiles[pSth->oldestInvalid] != NULL &&
			(pSth->oldestInvalid == 0 ||	// if oldestInvalid == 0 then ALL files are invalid
			strncmp(rec->header.filename, pSth->invalidFiles[pSth->oldestInvalid], PM_HISTORY_FILENAME_LEN) > 0)) {
		filename = pSth->invalidFiles[pSth->oldestInvalid];
		pSth->oldestInvalid++;
	} else {
		filename = rec->header.filename;
		rec->index = INDEX_NOT_IN_USE;
		// update the map
		for (i = 0; i < rec->header.imagesPerComposite; i++) {
			if (rec->historyImageEntries[i].inx != INDEX_NOT_IN_USE) {
				cl_qmap_remove_item(&pSth->historyImages, &rec->historyImageEntries[i].historyImageEntry);
				rec->historyImageEntries[i].inx = INDEX_NOT_IN_USE;
			}
		}
		cl_map_item_t * mi;
		mi = cl_qmap_get(&pSth->imageTimes, (uint64_t)rec->header.imageTime << 32 | rec->header.imageSweepInterval);
		if (mi != cl_qmap_end(&pSth->imageTimes))
			cl_qmap_remove_item(&pSth->imageTimes, &rec->imageTimeEntry);
	}
	if (stat(filename, &fileInfo) < 0) {
		return VSTATUS_EIO;
	}
	// Lock to prevent a shutdown mid file write (HSM only)
	block_sm_exit();
	if (remove(filename) != 0) {
		// Can't remove file?
		IB_LOG_WARN_FMT(__func__, "Unable to remove expired history file: %s", filename);
		unblock_sm_exit();
		return VSTATUS_EIO;
	}
	unblock_sm_exit();
	pSth->totalDiskUsage -= fileInfo.st_size;
	
	return VSTATUS_OK;
}
/************************************************************************************* 
 * 	prunePartialStoredHistory - brings stored PA history back under quota
 *  only looks from space from idxbegin (inclusive) to idxend (not inclusive)
 *  if idxbegin == idxend, then the entire list is included
 *
 *	Input/Output:
 *		pSth - The PM's shortTermHistory object
 *		idxbegin - Index to start at
 *		idxend - Index to end at
 *		req - Need at least this much space pruned away.
 * 	Returns
 * 		Status - VSTATUS_OK if okay
 *				 VSTATUS_EIO couldn't find enough space
 *				 VSTATUS_INVALID_ARG amount of space request exceeds total
*************************************************************************************/
static Status_t prunePartialStoredHistory(PmShortTermHistory_t *pSth, uint32 idxbegin, uint32 idxend, size_t req)
{
	uint64_t *pDiskUsage = &pSth->totalDiskUsage;
	uint64_t maxUsage = pm_config.shortTermHistory.maxDiskSpace << 20;
	uint64_t targetUsage = maxUsage - req;
	Status_t status = VSTATUS_OK;

	if (req > maxUsage) {
		IB_LOG_ERROR0("Single composite image size exceeds configured max disk "
					  "usage! Increase ShortTermHistory's MaxDiskSpace!");
		return VSTATUS_INVALID_ARG;
	}

	uint32 cindex = idxbegin;

	// Start at the end of the history, and work toward the front
	while (*pDiskUsage > targetUsage) {
		pruneOneStoredHistoryFile(pSth, cindex);
		cindex = (cindex+1)%pSth->totalHistoryRecords;
		if (cindex == idxend) {
			// Went the whole way through the list
			break;
		}
	}
	if (*pDiskUsage > targetUsage) {
		status = VSTATUS_EIO;
	}
	return status;
}

/************************************************************************************* 
 * 	pruneStoredHistory - brings stored PA history back under quota
 *
 *	Input/Output:
 *		pSth - The PM's shortTermHistory object
 *		req - Need at least this much space pruned away.
 * 	Returns
 * 		Status - VSTATUS_OK if okay
*************************************************************************************/
static Status_t pruneStoredHistory(PmShortTermHistory_t *pSth, size_t req)
{
	uint64_t *pDiskUsage = &pSth->totalDiskUsage;
	Status_t status = VSTATUS_OK;

	status = prunePartialStoredHistory(pSth, pSth->currentRecordIndex, pSth->currentRecordIndex, req);
	if (status == VSTATUS_EIO) {
		// purged all the files, and still not enough space
		// files must have been deleted manually
		*pDiskUsage = 0;
		return VSTATUS_OK;
	}
	return status;
}

/************************************************************************************* 
*   storeComposite - store a composite image
*  
*   Inputs:
*   	pm	- the PM.
*   	cimg - the composite image to store
*  
*   Returns:
*   	Status - FSUCCESS if okay
*************************************************************************************/
FSTATUS storeComposite(Pm_t *pm, PmCompositeImage_t *cimg) {
	unsigned char *data;
	size_t len, writeLen;
	FILE *fp = NULL;
	FSTATUS ret = FSUCCESS;
	unsigned char **compressed_divisions = NULL;
	size_t *compressed_sizes = NULL;
	int i;
	
	// figure out how big the flattened image will be
	writeLen = len = computeFlatSize(cimg);
	// update the header
	cimg->header.flatSize = len;
	vs_stdtime_get((time_t *)&(cimg->header.common.timestamp));
	
	// data will hold the flattened image
	data = calloc(1, len);
	if (!data) {
		IB_LOG_ERROR0("Failed to allocate data for flattened composite");
		ret = FINSUFFICIENT_MEMORY;
		goto error;
	}
	// flatten the image
	ret = flattenComposite(cimg, data, cimg->header.common.historyVersion);
	if (ret != FSUCCESS)
		goto error;

	if (cimg->header.common.isCompressed) {
#ifdef __VXWORKS__
		IB_LOG_ERROR0("Compression not available for embedded builds");
		ret = FERROR;
		goto error;
#else
		compressed_divisions = calloc(1, sizeof(unsigned char*) * pm_config.shortTermHistory.compressionDivisions);
		compressed_sizes = calloc(1, sizeof(size_t) * pm_config.shortTermHistory.compressionDivisions);
		if (!compressed_divisions || !compressed_sizes) {
			IB_LOG_ERROR0("Failed to allocate data for compression");
			ret = FINSUFFICIENT_MEMORY;
			goto error;
		}

		// don't compress the header
		ret = divideAndCompress(data + sizeof(PmFileHeader_t), len - sizeof(PmFileHeader_t), compressed_divisions, compressed_sizes);
		if (ret) goto error;

		writeLen = sizeof(PmFileHeader_t);
		for (i=0; i < pm_config.shortTermHistory.compressionDivisions; i++) {
			writeLen += compressed_sizes[i];
			((PmFileHeader_t*)data)->divisionSizes[i] = (uint64)compressed_sizes[i];
		}

		// check disk space
		if ((ret = pruneStoredHistory(&pm->ShortTermHistory, writeLen)) != VSTATUS_OK) goto error;

		// update header with division info
		((PmFileHeader_t*)data)->numDivisions = pm_config.shortTermHistory.compressionDivisions;

		// Lock to prevent a shutdown mid file write (HSM only)
		block_sm_exit();
		// open file
		if (!(fp = fopen(cimg->header.common.filename, "wb"))) {
			IB_LOG_ERROR0("Failed to create new PM history file");
			ret = FERROR;
			unblock_sm_exit();
			goto error;
		}

		// write the header to the file
		if (fwrite(data, 1, sizeof(PmFileHeader_t), fp) != sizeof(PmFileHeader_t) || ferror(fp)) {
			IB_LOG_ERROR0("Encountered error while storing PM history file");
			ret = FERROR;
			unblock_sm_exit();
			goto error;
		}

		// write each division
		for (i=0; i < pm_config.shortTermHistory.compressionDivisions; i++) {
			if (compressed_divisions[i]) {
				if (fwrite(compressed_divisions[i], 1, compressed_sizes[i], fp) != compressed_sizes[i] || ferror(fp)) {
					IB_LOG_ERROR0("Encountered error while storing PM history file");
					ret = FERROR;
					unblock_sm_exit();
					goto error;
				}
			}
		}
		unblock_sm_exit();

#endif
	} else {
#ifndef __VXWORKS__
		if ((ret = pruneStoredHistory(&pm->ShortTermHistory, writeLen)) != VSTATUS_OK) goto error;
#endif
		// Lock to prevent a shutdown mid file write (HSM only)
		block_sm_exit();
		if (!(fp = fopen(cimg->header.common.filename, "wb"))) {
			IB_LOG_ERROR0("Failed to create new PM history file");
			ret = FERROR;
			unblock_sm_exit();
			goto error;
		}
		if (fwrite(data, 1, writeLen, fp) != writeLen || ferror(fp)) {
			IB_LOG_ERROR0("Encountered error while storing PM history file");
			ret = FERROR;
			unblock_sm_exit();
			goto error;
		}
		unblock_sm_exit();
	}

	pm->ShortTermHistory.totalDiskUsage += writeLen;

error:
	if (compressed_divisions) {
		for (i=0; i < pm_config.shortTermHistory.compressionDivisions; i++) 
			if (compressed_divisions[i]) free(compressed_divisions[i]);
		free(compressed_divisions);
	}
	if (compressed_sizes) free(compressed_sizes);
	if (data) free(data);
	if (fp) fclose(fp);
	return ret;
}

/************************************************************************************* 
*  	compoundNewImage - take a newly create PM image and compound it into the current image
*  
*  	Inputs:
*   	pm - the PM
*  
*   Returns:
*   	FSUCCESS if okay
*************************************************************************************/
// non-static is temporary
FSTATUS compoundNewImage(Pm_t *pm) {
	FSTATUS ret = FSUCCESS;
#ifdef __VXWORKS__
	// composites are not compressed for embedded
	uint8_t isCompressed = 0;
#else
	// composites are always compressed otherwise
	uint8_t isCompressed = 1;
#endif

	// if there is no image, or the current one has already been written
	if (!pm->ShortTermHistory.currentComposite || pm->ShortTermHistory.compositeWritten) {
		if (pm->ShortTermHistory.currentComposite) {
			//free the old image now that it is no longer being used
			PmFreeComposite(pm->ShortTermHistory.currentComposite);
			pm->ShortTermHistory.currentComposite = NULL;
		}
		//create a new composite from the latest image
		pm->ShortTermHistory.currentComposite = PmCreateComposite(pm, pm->lastHistoryIndex, isCompressed);
		if (!pm->ShortTermHistory.currentComposite) {
			IB_LOG_WARN0("Failed to create a new composite Image");
			return FERROR;
		}
		pm->ShortTermHistory.compositeWritten = 0;
	} else {
		// compound
		ret = compoundImage(pm, pm->lastHistoryIndex, pm->ShortTermHistory.currentComposite);
		if (ret != FSUCCESS) {
			IB_LOG_WARNRC("Failed to compound composite rc:", ret);
		}
	}

	// check if it is time to write the composite yet
	if (pm->ShortTermHistory.currentComposite->header.common.imagesPerComposite == pm_config.shortTermHistory.imagesPerComposite) {
		uint32 cindex = pm->ShortTermHistory.currentRecordIndex;
		PmHistoryRecord_t *rec = pm->ShortTermHistory.historyRecords[cindex];

		// delete old record (if one is present)
		pruneOneStoredHistoryFile(&pm->ShortTermHistory, cindex);

		// finalize the header
		setFilename(&(pm->ShortTermHistory), pm->ShortTermHistory.currentComposite);

		ret = storeComposite(pm, pm->ShortTermHistory.currentComposite);
		if (ret != FSUCCESS) {
			IB_LOG_ERRORRC("Error compounding image into current composite rc:", ret);
			return ret;
		}
		pm->ShortTermHistory.compositeWritten = 1;
		MemoryClear(rec, sizeof(PmHistoryRecord_t));
		// update the record
		memcpy(&rec->header, &(pm->ShortTermHistory.currentComposite->header.common), sizeof(PmHistoryHeaderCommon_t));
		markImagesAsDisk(&rec->header);
		rec->index = cindex;
		// update the map
		cl_map_item_t *mi;
		int i;
		for (i = 0; i < rec->header.imagesPerComposite; i++) {
			uint64 imageID = rec->header.imageIDs[i];

			mi = cl_qmap_get(&pm->ShortTermHistory.historyImages, imageID);
			if (mi != cl_qmap_end(&pm->ShortTermHistory.historyImages)) { // found
        		PmHistoryImageEntry_t *entry = PARENT_STRUCT(mi, PmHistoryImageEntry_t, historyImageEntry);
				cl_qmap_remove_item(&pm->ShortTermHistory.historyImages, mi);
				entry->inx = INDEX_NOT_IN_USE;
			}
			mi = cl_qmap_insert(&pm->ShortTermHistory.historyImages, imageID, &rec->historyImageEntries[i].historyImageEntry);
			if (mi != &rec->historyImageEntries[i].historyImageEntry) {
				IB_LOG_ERROR0("Error updating image/record map");
			}
			rec->historyImageEntries[i].inx = i;
		}
		uint32 imgTime = rec->header.imageTime, imgIval=rec->header.imageSweepInterval;
		mi = cl_qmap_get(&pm->ShortTermHistory.imageTimes, (uint64_t)imgTime << 32 | imgIval);
		if (mi != cl_qmap_end(&pm->ShortTermHistory.imageTimes)) { // found
			cl_qmap_remove_item(&pm->ShortTermHistory.imageTimes, mi);
		}
		mi = cl_qmap_insert(&pm->ShortTermHistory.imageTimes, (uint64_t)imgTime << 32 | imgIval, &rec->imageTimeEntry);
		if (mi != &rec->imageTimeEntry) {
			IB_LOG_ERROR0("Error updating time map");
		}
		// update the record index
		pm->ShortTermHistory.currentRecordIndex = (cindex+1)%pm->ShortTermHistory.totalHistoryRecords;
	}
	return ret;
}
#endif

#ifdef __VXWORKS__
FSTATUS injectHistoryFile(Pm_t *pm, char *filename, uint8_t *buffer, uint32_t filelen) {
	return VSTATUS_EIO;
}
#else
/************************************************************************************* 
*  	injectHistoryFile - insert a shortterm history file from the Master PM into
*	the local history filelist at the appropriate location. We do not need to
*	parse the file for images because if we become Master PM, we'll reload
*	history at that time. Putting the filename in the list at the appropriate
*	place insures that we will delete it at the appropriate time, and we will
*	never exceed the configured maximum number of history files, or configured
*	disk usage.
*  
*  	Inputs:
*   	pm - the PM
*		filename - name of the file to inject
*  
*   Returns:
*   	FSUCCESS if okay
*************************************************************************************/
FSTATUS injectHistoryFile(Pm_t *pm, char *filename, uint8_t *buffer, uint32_t filelen) {
	FSTATUS ret = FSUCCESS;
	PmShortTermHistory_t *pSth = &pm->ShortTermHistory;

	uint32 i;
	uint32 cindex = pSth->currentRecordIndex;
	uint32 stop;
	uint32 len; // Don't compare the file extention
	char newfilename[PM_HISTORY_FILENAME_LEN];

	if (strstr(filename, ".hist")) {
		snprintf(newfilename, PM_HISTORY_FILENAME_LEN, "%s/%s",
				 pm->ShortTermHistory.filepath,
				 (strstr(filename, ".hist") - PM_HISTORY_STHFILE_LEN));
		len = strstr(newfilename, ".hist") - newfilename;
	} else if (strstr(filename, ".zhist")) {
		snprintf(newfilename, PM_HISTORY_FILENAME_LEN, "%s/%s",
				 pm->ShortTermHistory.filepath,
				 (strstr(filename, ".zhist") - PM_HISTORY_STHFILE_LEN));
		len = strstr(newfilename, ".zhist") - newfilename;
	} else {
		snprintf(newfilename, PM_HISTORY_FILENAME_LEN, "%s/c%s", 
				pm->ShortTermHistory.filepath,
				filename + strlen(filename) - PM_HISTORY_STHFILE_LEN);
		len = strlen(newfilename);
	}

	// Special case: We already have the maximum number of history files, and this
	// one is older than all of them.
	PmHistoryRecord_t *rec = pSth->historyRecords[cindex];
	if ((rec->index != INDEX_NOT_IN_USE) && strncmp(newfilename, rec->header.filename, len) < 0) {
		// Ignore this incoming file
		return FSUCCESS;
	}
	
	// Start at the end of the list (oldest) and move forward
	// as long as entries are empty, or in use and older than
	// this file.
	while ((rec->index == INDEX_NOT_IN_USE) || strncmp(newfilename, rec->header.filename, len) > 0) {
		cindex = (cindex+1)%pSth->totalHistoryRecords;
		rec = pSth->historyRecords[cindex];
		if (cindex == pSth->currentRecordIndex) {
			// Special case: This file is newer than all of the current files.
			// We can just put this file on the head of the list.
			pruneOneStoredHistoryFile(pSth, cindex);
			pSth->currentRecordIndex = (cindex+1)%pSth->totalHistoryRecords;
			goto store_file;
		}
	}
	if (rec->index != INDEX_NOT_IN_USE && !strncmp(newfilename, rec->header.filename, len)) {
		// If we found the same filename, purge the old one, save the new one
		pruneOneStoredHistoryFile(pSth, cindex);
	} else {
		// cindex points to a newer filename, so back up one entry
		if (cindex) cindex--;
		else cindex = pSth->totalHistoryRecords - 1;
		rec = pSth->historyRecords[cindex];
	}

	if (rec->index != INDEX_NOT_IN_USE) {
		// Doing a middle insertion,
		// All of the entries from this point
		// back need to be shifted.

		// Scan backwards from cindex until we find an
		// empty slot, or the front of the list.
		uint32 begin = cindex;
		while (pSth->historyRecords[begin]->index != INDEX_NOT_IN_USE) {
			if (begin) begin--;
			else begin = pSth->totalHistoryRecords - 1;
			if (begin == pSth->currentRecordIndex) {
				// We are all the way at the end of the list, make sure this
				// slot is empty.
				pruneOneStoredHistoryFile(pSth, begin);
			}
		}

		// Now we have to shift every entry from begin+1 to cindex
		uint32 dst = begin, src = (dst+1)%pSth->totalHistoryRecords;
		do {
			PmHistoryRecord_t *srcrec = pSth->historyRecords[src];
			PmHistoryRecord_t *dstrec = pSth->historyRecords[dst];
			ASSERT(dstrec->index == INDEX_NOT_IN_USE);
			ASSERT(srcrec->index != INDEX_NOT_IN_USE);
			// Remove from map (if necessary)
			for (i = 0; i < srcrec->header.imagesPerComposite; i++) {
				if (srcrec->historyImageEntries[i].inx != INDEX_NOT_IN_USE) {
					cl_qmap_remove_item(&pSth->historyImages, &srcrec->historyImageEntries[i].historyImageEntry);
					srcrec->historyImageEntries[i].inx = INDEX_NOT_IN_USE;
				}
			}
			cl_map_item_t * mi;
			mi = cl_qmap_get(&pSth->imageTimes, (uint64_t)srcrec->header.imageTime << 32 | srcrec->header.imageSweepInterval);
			if (mi != cl_qmap_end(&pSth->imageTimes))
				cl_qmap_remove_item(&pSth->imageTimes, &srcrec->imageTimeEntry);

			// Only copy the header from srcrec to dstrec.
			dstrec->header = srcrec->header;
			snprintf(dstrec->header.filename,sizeof(dstrec->header.filename),"%s",srcrec->header.filename);
			dstrec->index = dst;
			// Then initialize empty srcrec
			MemoryClear(srcrec, sizeof(PmHistoryRecord_t));
			for (i = 0; i < PM_HISTORY_MAX_IMAGES_PER_COMPOSITE; i++) {
				srcrec->historyImageEntries[i].inx = INDEX_NOT_IN_USE;
			}
			srcrec->index = INDEX_NOT_IN_USE;
			dst = src;
			src = (src+1)%pSth->totalHistoryRecords;
		} while (dst != cindex);
	}

store_file:

	/* Make sure there is enough space */
	/* Only prune older files! */
	stop = (cindex+1)%pSth->totalHistoryRecords;
	if (prunePartialStoredHistory(pSth, pSth->currentRecordIndex, stop, filelen) != VSTATUS_OK) {
		// No space for this file
        return -1;
	}

	block_sm_exit();
	/* Create the new file */
    IB_LOG_VERBOSE_FMT(__func__, "Received sweep image data (%s) at standby filelen=0x%x", newfilename,filelen);
    FILE *file = fopen(newfilename, "w");
    if (!file) {
        IB_LOG_ERROR("Error opening PM history image file to write data! rc:",errno);
		unblock_sm_exit();
        return -1;
    }
    if (fwrite(buffer, sizeof(uint8_t), filelen, file) != filelen) {
        IB_LOG_ERROR("Error failed writing PM history image file! rc:",errno);
		fclose(file);
		remove(newfilename);
		unblock_sm_exit();
		return -1;
	}
    fclose(file);
	unblock_sm_exit();

	/* Add the new file into the ShortTermHistory */
	ASSERT(rec->index == INDEX_NOT_IN_USE);
	MemoryClear(rec, sizeof(PmHistoryRecord_t));
	// Update the record.  Only fill in the filename. Leave the images
	// per composite as zero. This is all the Standby cares about.
	// If/When we become Master, we'll reread the history.
	snprintf(rec->header.filename, sizeof(rec->header.filename), newfilename);
	for (i = 0; i < PM_HISTORY_MAX_IMAGES_PER_COMPOSITE; i++) {
		rec->historyImageEntries[i].inx = INDEX_NOT_IN_USE;
	}
	pSth->totalDiskUsage += filelen;
	rec->index = cindex;

	return ret;
}

/************************************************************************************* 
*   PmLoadHistory - load existing short term history files into the PM
*  
*   Inputs:
*   	pm - the PM
*		master - are we loading history as master?
*  
*   Returns:
*   	VSTATUS_OK if okay
*************************************************************************************/
static Status_t PmLoadHistory(Pm_t *pm, boolean master) {
	struct dirent **d;
	int i, n, x, ii, tot;
	uint64_t diskUsage = 0; /* In bytes */
	boolean isOverQuota = 0;
	struct stat fileInfo = {0};
	Status_t ret = VSTATUS_OK;

	// Discard any history previously loaded
	for (i = 0; i < pm->ShortTermHistory.totalHistoryRecords; i++) {
		PmHistoryRecord_t *rec = pm->ShortTermHistory.historyRecords[i];
		if (rec->index != INDEX_NOT_IN_USE) {
			for (x = 0; x < rec->header.imagesPerComposite; x++) {
				if (rec->historyImageEntries[x].inx != INDEX_NOT_IN_USE) {
					cl_qmap_remove_item(&pm->ShortTermHistory.historyImages, &rec->historyImageEntries[x].historyImageEntry);
					rec->historyImageEntries[x].inx = INDEX_NOT_IN_USE;
				}
			}
			cl_map_item_t * mi;
			mi = cl_qmap_get(&pm->ShortTermHistory.imageTimes, (uint64_t)rec->header.imageTime << 32 | rec->header.imageSweepInterval);
			if (mi != cl_qmap_end(&pm->ShortTermHistory.imageTimes))
				cl_qmap_remove_item(&pm->ShortTermHistory.imageTimes, &rec->imageTimeEntry);
			rec->index = INDEX_NOT_IN_USE;
		}
	}
	// scandir will read all of the files in order, and filter out any that don't end in .hist or .zhist
	n = scandir(pm->ShortTermHistory.filepath, &d, hist_filter, alphasort);
	if (n < 0) {
		ret = VSTATUS_EIO;
	} else {
		// add 1 for the '/', 256 for d_name
		char filename[PM_HISTORY_FILENAME_LEN + 1 + 256];
		// it is possible that there are more history files than we can hold,
		// in which case, we want to get the most recent ones
		// History images will be loaded in reverse order from the tail of the history
		// ring toward the head.
		// i indexes valid files, ii indexes invalid files, tot keeps track of the total of both
		i = ii = tot = pm->ShortTermHistory.totalHistoryRecords - 1;
		// New entries will be added at the head of the history ring
		pm->ShortTermHistory.currentRecordIndex = 0;
		while ((ret == VSTATUS_OK) && n--) {
			snprintf(filename, PM_HISTORY_FILENAME_LEN + 1 + 256, "%s/%s", pm->ShortTermHistory.filepath, d[n]->d_name);
			free(d[n]);
			if (tot >= 0) {
				FILE *fp;

				if (!(fp = fopen(filename, "r"))) {
					ret = VSTATUS_EIO;
				} else {
					const size_t readSize = sizeof(PmHistoryHeaderCommon_t);
					uint8 bf_header[sizeof(PmHistoryHeaderCommon_t)];
					PmHistoryRecord_t *rec = pm->ShortTermHistory.historyRecords[i];
					if (stat(filename, &fileInfo) < 0) ret = VSTATUS_EIO;
					if (fileInfo.st_size == 0) {
						// empty file, remove
						fclose(fp);
						if (remove(filename) < 0) ret = VSTATUS_EIO;
						continue;
					}
					// read the header of the file
					if (fread(bf_header, 1, readSize, fp) != readSize) {
						IB_LOG_WARN_FMT(__func__, "Encountered a problem while loading a Short-Term History file: %s", filename);
						fclose(fp);
						continue;
					}

					// Enforce max disk usage
					if (!isOverQuota)  {
						diskUsage += fileInfo.st_size;
						isOverQuota = diskUsage >= (pm_config.shortTermHistory.maxDiskSpace * (1<<20));
					}

					if (!isOverQuota) {
						pm->ShortTermHistory.totalDiskUsage = diskUsage;
					} else {
						if (remove(filename) < 0) ret = VSTATUS_EIO;
						fclose(fp);
						continue;
					}

					// check the version
					if (((PmHistoryHeaderCommon_t *)bf_header)->historyVersion != PM_HISTORY_VERSION &&
						((PmHistoryHeaderCommon_t *)bf_header)->historyVersion != PM_HISTORY_VERSION_OLD) {
						// found history file with invalid version
						ret = vs_pool_alloc(&pm_pool, (sizeof(char) * PM_HISTORY_FILENAME_LEN), (void *)&(pm->ShortTermHistory.invalidFiles[ii]));
						if (ret != VSTATUS_OK) {
							IB_LOG_WARN_FMT(__func__, "Failed to allocate PM Short-Term filename for file: %s", filename);
							fclose(fp);
							continue;
						}
						MemoryClear(pm->ShortTermHistory.invalidFiles[ii], (sizeof(char) * PM_HISTORY_FILENAME_LEN));

						strncpy(pm->ShortTermHistory.invalidFiles[ii], filename, PM_HISTORY_FILENAME_LEN);
						fclose(fp);
						ii--;
						tot--;
						continue;
					}
					// Move header into the record entry
					memcpy(pm->ShortTermHistory.historyRecords[i], bf_header, readSize);

					markImagesAsDisk(&(pm->ShortTermHistory.historyRecords[i]->header));

					// if this was a failover the record may have the wrong filepath, so overwrite it
					snprintf(pm->ShortTermHistory.historyRecords[i]->header.filename, PM_HISTORY_FILENAME_LEN, "%s", filename);
					fclose(fp);

					pm->ShortTermHistory.historyRecords[i]->index = i;
					// update the image ID map (only if master)
					if (master){
						for (x = 0; x < rec->header.imagesPerComposite; x++) {
							cl_map_item_t *mi;
							mi = cl_qmap_get(&pm->ShortTermHistory.historyImages, rec->header.imageIDs[x]);
							if (mi != cl_qmap_end(&pm->ShortTermHistory.historyImages)) { // found
								// This ID already exists. Since we read history
								// in reverse order (newest first), if we hit a
								// duplicate, keep the first one loaded, and discard
								// the duplicate.
								rec->historyImageEntries[x].inx = INDEX_NOT_IN_USE;
								continue;
							}

							mi = cl_qmap_insert(&pm->ShortTermHistory.historyImages, rec->header.imageIDs[x], &rec->historyImageEntries[x].historyImageEntry);
							if (mi != &rec->historyImageEntries[x].historyImageEntry) {
								IB_LOG_WARN0("Error adding image ID to map");
							}
							rec->historyImageEntries[x].inx = x;
						}
						if (rec->header.imageTime != 0){ //don't add files without time information included
							cl_map_item_t *mi;
							mi = cl_qmap_get(&pm->ShortTermHistory.imageTimes, (uint64_t)rec->header.imageTime << 32 | rec->header.imageSweepInterval);
							if (mi == cl_qmap_end(&pm->ShortTermHistory.imageTimes)){
								mi = cl_qmap_insert(&pm->ShortTermHistory.imageTimes, (uint64_t)rec->header.imageTime << 32 | rec->header.imageSweepInterval, &rec->imageTimeEntry);
								if (mi != &rec->imageTimeEntry){
									IB_LOG_WARN0("Error adding time to map");
								}
							}
						}
					}
					i--; // This slot is now consumed
					tot--;
				}
			} else {
					// More than pm->ShortTermHistory.totalHistoryRecords files
					// Just remove any extra files
					if (remove(filename) < 0) ret = VSTATUS_EIO;
			}
		}
		pm->ShortTermHistory.oldestInvalid = ii + 1; // if no invalid files were found, then oldestInvalid == totalHistoryRecords
		
		ImageId_t temp;
		temp.AsReg64 =
			pm->ShortTermHistory.historyRecords[pm->ShortTermHistory.currentRecordIndex ?
				pm->ShortTermHistory.currentRecordIndex - 1 :
				pm->ShortTermHistory.totalHistoryRecords - 1]->header.imageIDs[0];
		if (!master) {
			// set the instance ID
			pm->ShortTermHistory.currentInstanceId = temp.s.instanceId;
		} else {
			// Increment instance ID
			pm->ShortTermHistory.currentInstanceId = (temp.s.instanceId + 1) % IMAGEID_MAX_INSTANCE_ID;
		}
	}

	if (ret != VSTATUS_OK) {
		for (i = 0; i < n; n++) if (d[n]) free(d[n]);
	}
	if (d) free(d);

	return ret;
}

/************************************************************************************* 
*   PmInitHistory - Initializes PmShortTermHistory
*  
*   Inputs:
*   	pm - the PM
*  
*   Returns:
*   	VSTATUS_OK if okay
* 
*************************************************************************************/
Status_t PmInitHistory(Pm_t *pm) {
	Status_t status = VSTATUS_OK;
	int i;
	size_t storage_len;
	struct stat dirInfo;
	char *basename;
	char storageLocation[FILENAME_SIZE];

	// check the history filepath
	snprintf(storageLocation, FILENAME_SIZE, "%s", pm_config.shortTermHistory.StorageLocation);
	storage_len = strnlen(storageLocation, FILENAME_SIZE);
	if ((storage_len < 3) || (storageLocation[0] != '/')) {
		IB_LOG_ERROR_FMT(__func__, "Invalid storage location for PM Short-Term History %s - must be non-NULL absolute path",
			pm_config.shortTermHistory.StorageLocation);
		return VSTATUS_ILLPARM;
	}
	if (storageLocation[storage_len - 1] == '/')
		storageLocation[storage_len - 1] = 0;

	// check that the location exists, try to create it if it doesn't
	basename = strrchr(storageLocation, '/');
	if (basename == storageLocation || basename == NULL) {
		IB_LOG_ERROR_FMT(__func__, "Invalid Short-Term History storage location %s - no directory name",
			storageLocation);
		status = VSTATUS_ILLPARM;
		goto fail;
	} else {
		*basename = 0;
		if (stat(storageLocation, &dirInfo)) {
			if (errno == ENOENT) {
				// directory doesn't exist, try to create it
				if(mkdir(storageLocation, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
					// failed to create the directory
					IB_LOG_ERROR_FMT(__func__, "Invalid Short-Term History storage location %s - unable to create",
						storageLocation);
					status = VSTATUS_ILLPARM;
					goto fail;
				}
			} else {
				IB_LOG_ERROR_FMT(__func__, "Invalid Short-Term History storage location %s",
					storageLocation);
				status = VSTATUS_ILLPARM;
				goto fail;
			}
		} else {
			if (!S_ISDIR(dirInfo.st_mode)) {
				// not a directory
				IB_LOG_ERROR_FMT(__func__, "Invalid Short-Term History storage location %s - must be a directory",
					storageLocation);
				status = VSTATUS_ILLPARM;
				goto fail;
			}
		}
	}

	// set up the storage directory if needed
	snprintf(pm->ShortTermHistory.filepath, PM_HISTORY_MAX_LOCATION_LEN, "%s", pm_config.shortTermHistory.StorageLocation);

	// check that images per composite doesn't exceed the limit
	if (pm_config.shortTermHistory.imagesPerComposite > PM_HISTORY_MAX_IMAGES_PER_COMPOSITE) {
		IB_LOG_ERROR0("Invalid PM Short-Term History 'imagesPerComposite' configuration");
		return VSTATUS_ILLPARM;
	}

	// check that compression division doesn't exceed maximum allowed divisions
	if (pm_config.shortTermHistory.compressionDivisions > PM_MAX_COMPRESSION_DIVISIONS || pm_config.shortTermHistory.compressionDivisions < 1) {
		IB_LOG_ERROR0("Invalid PM Short-Term History 'compressionDivisions' configuration");
		return VSTATUS_ILLPARM;
	}

	// initialize the instanceId to 0
	pm->ShortTermHistory.currentInstanceId = 0;	  

	// calculate now many composite files there will be
	pm->ShortTermHistory.totalHistoryRecords = (3600*pm_config.shortTermHistory.totalHistory)/(pm_config.shortTermHistory.imagesPerComposite*pm_config.sweep_interval);
	if (pm->ShortTermHistory.totalHistoryRecords < 1) {
		IB_LOG_ERROR0("Invalid PM Short-Term History configuration: 'totalHistory' must be greater than 'imagesPerComposite' * 'SweepInterval'");
		return VSTATUS_ILLPARM;
	}
	cl_qmap_init(&pm->ShortTermHistory.historyImages, NULL);

	// initialize map of sweep times to records
	cl_qmap_init(&pm->ShortTermHistory.imageTimes, NULL);

	status = vs_pool_alloc(&pm_pool, sizeof(PmHistoryRecord_t**) * (pm->ShortTermHistory.totalHistoryRecords), (void *)&pm->ShortTermHistory.historyRecords);
	if (status != VSTATUS_OK || !pm->ShortTermHistory.historyRecords) {
		IB_LOG_ERRORRC("Failed to allocate PM Short-Term History records rc:", status);
		status = VSTATUS_NOMEM;
		goto fail;
	}
	MemoryClear(pm->ShortTermHistory.historyRecords, sizeof(PmHistoryRecord_t**) * (pm->ShortTermHistory.totalHistoryRecords));
	for (i = 0; i < pm->ShortTermHistory.totalHistoryRecords; i++) {
		status = vs_pool_alloc(&pm_pool, sizeof(PmHistoryRecord_t), (void*)&(pm->ShortTermHistory.historyRecords[i]));
		if (status != VSTATUS_OK) {
			IB_LOG_ERRORRC("Failed to allocate PM Short-Term History Record rc:", status);
			status = VSTATUS_NOMEM;
			goto fail;
		}
		MemoryClear(pm->ShortTermHistory.historyRecords[i], sizeof(PmHistoryRecord_t));
		int j;
		pm->ShortTermHistory.historyRecords[i]->index = INDEX_NOT_IN_USE;
		for (j = 0; j < PM_HISTORY_MAX_IMAGES_PER_COMPOSITE; j++) {
			pm->ShortTermHistory.historyRecords[i]->historyImageEntries[j].inx = INDEX_NOT_IN_USE;
		}
	}
	status = vs_pool_alloc(&pm_pool, sizeof(char**) * (pm->ShortTermHistory.totalHistoryRecords), (void *)&pm->ShortTermHistory.invalidFiles);
	if (status != VSTATUS_OK || !pm->ShortTermHistory.invalidFiles) {
		IB_LOG_ERRORRC("Failed to allocate PM Short-Term History invalid files rc:", status);
		status = VSTATUS_NOMEM;
		goto fail;
	}
	MemoryClear(pm->ShortTermHistory.invalidFiles, sizeof(char**) * (pm->ShortTermHistory.totalHistoryRecords));
	if (stat(pm->ShortTermHistory.filepath, &dirInfo)) {
		if (errno == ENOENT) {
			// directory doesn't exist, try to create it
			if (mkdir(pm->ShortTermHistory.filepath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
				// failed to create the directory
				IB_LOG_ERROR_FMT(__func__, "Invalid Short-Term History storage location %s - unable to create",
					pm->ShortTermHistory.filepath);
				status = VSTATUS_ILLPARM;
				goto fail;
			}
		} else {
			IB_LOG_ERROR_FMT(__func__, "Invalid Short-Term History storage location %s",
				pm->ShortTermHistory.filepath);
			status = VSTATUS_ILLPARM;
			goto fail;
		}
	} else {
		if (!S_ISDIR(dirInfo.st_mode)) {
			// not a directory
			IB_LOG_ERROR_FMT(__func__, "Invalid Short-Term History storage location %s - must be a directory",
				pm->ShortTermHistory.filepath);
			status = VSTATUS_ILLPARM;
			goto fail;
		}
	}
	PmLoadHistory(pm, 0);
	return status;
fail:
	if (pm->ShortTermHistory.historyRecords) {
		for (i=0; i < pm->ShortTermHistory.totalHistoryRecords; i++) {
			if (pm->ShortTermHistory.historyRecords[i]) {
				vs_pool_free(&pm_pool, pm->ShortTermHistory.historyRecords[i]);
			}
		}
		vs_pool_free(&pm_pool, pm->ShortTermHistory.historyRecords);
	}
	return status;
}
#endif

FSTATUS storeCompositeToBuffer(PmCompositeImage_t *cimg, uint8_t *buffer, uint32_t *bIndex) {
	FSTATUS ret = FSUCCESS;
	unsigned char *data;
	size_t len;
#ifndef __VXWORKS__
	int i;
	uint8_t *bufferLoc = buffer;
#endif

	// figure out how big the flattened image will be
	len = computeFlatSize(cimg);
	// update the header
	cimg->header.flatSize = len;
	vs_stdtime_get((time_t *)&(cimg->header.common.timestamp));
	
	// data will hold the flattened image
	data = calloc(1, len);
	if (!data) {
		IB_LOG_ERROR0("Failed to allocate data for flattened composite");
		return FINSUFFICIENT_MEMORY;
	}
	// flatten the image
	ret = flattenComposite(cimg, data, cimg->header.common.historyVersion);
	if (ret != FSUCCESS) {
		free(data);
		return ret;
	}

	BSWAP_PM_COMPOSITE_IMAGE_FLAT((PmCompositeImage_t *)data, 1 /*, cimg->header.common.historyVersion*/);

	if (cimg->header.common.isCompressed) {
#ifdef __VXWORKS__
		IB_LOG_ERROR0("Compression not available for embedded builds");
		ret = FERROR;
		goto done;
#else
		unsigned char **compressed_divisions = calloc(1, sizeof(unsigned char*) * pm_config.shortTermHistory.compressionDivisions);
		size_t *compressed_sizes = calloc(1, sizeof(size_t) * pm_config.shortTermHistory.compressionDivisions);

		if (!compressed_divisions || !compressed_sizes) {
			IB_LOG_ERROR0("Failed to allocate data for compression");
			if (compressed_divisions) free(compressed_divisions);
			if (compressed_sizes) free(compressed_sizes);
			ret = FINSUFFICIENT_MEMORY;
			goto done;
		}
		ret = divideAndCompress(data + sizeof(PmFileHeader_t), len - sizeof(PmFileHeader_t), compressed_divisions, compressed_sizes);

		// update header with division info
		((PmFileHeader_t*)data)->numDivisions = pm_config.shortTermHistory.compressionDivisions;
		for (i = 0; i < pm_config.shortTermHistory.compressionDivisions; i++) 
			((PmFileHeader_t*)data)->divisionSizes[i] = compressed_sizes[i];

		// copy the header to the buffer
		memcpy(bufferLoc, data, sizeof(PmFileHeader_t));
		*bIndex += sizeof(PmFileHeader_t);
		bufferLoc += sizeof(PmFileHeader_t);

		// copy each piece to the buffer
		for (i = 0; i < pm_config.shortTermHistory.compressionDivisions; i++) {
			if (compressed_divisions[i]) {
				memcpy(bufferLoc, compressed_divisions[i], compressed_sizes[i]);
				*bIndex += compressed_sizes[i];
				bufferLoc += compressed_sizes[i];
				// no longer needed
				free(compressed_divisions[i]);
			}
		}

		free(compressed_divisions);
		free(compressed_sizes);
#endif // else ifdef __VXWORKS__
	} else {
		// don't compress, just write data to the buffer
		memcpy(buffer, data, len);
		*bIndex += len;
	}

	BSWAP_PM_FILE_HEADER(&((PmCompositeImage_t *)buffer)->header);

done:
	free(data);
	return ret;
}

void writeImageToBuffer(Pm_t *pm, uint32 histindex, uint8_t isCompressed, uint8_t *buffer, uint32_t *bIndex) {
	FSTATUS ret;

	if (!buffer || !bIndex || histindex == PM_IMAGE_INDEX_INVALID) {
		IB_LOG_VERBOSE_FMT(__func__, "Invalid parameters/image index.");
		return;
	}

	PmCompositeImage_t *cimg1;
	cimg1 = PmCreateComposite(pm, histindex, isCompressed);
	if (cimg1 == NULL) {
		IB_LOG_ERROR0("Failed to create PM Composite Image");
		return;
	}

	ret = storeCompositeToBuffer(cimg1, buffer, bIndex);
	PmFreeComposite(cimg1);
	if (ret != FSUCCESS) {
		IB_LOG_ERROR0("Failed to store PM Composite Image into buffer");
	}

	return;

}	// End of writeImageToBuffer()
/* --------------------------------------------------------------------------------------------- */

// can be called for a PmaAvoid neighbor, but a PmaAvoid port will never
// be added to any group
// non-static is temporary
boolean PmCompareHFIPort(PmPort_t *pmportp, char *groupName)
{
	// groupName parameter not used in this function
	return (pmportp->pmnodep->nodeType == STL_NODE_FI
			&& ! (pmportp->capmask & PI_CM_IS_DEVICE_MGT_SUPPORTED));
}

#if 0 // TFI not included in Gen1
// can be called for a PmaAvoid neighbor, but a PmaAvoid port will never
// be added to any group
// non-static is temporary
boolean PmCompareTCAPort(PmPort_t *pmportp, char *groupName)
{
	// groupName parameter not used in this function
	return (pmportp->pmnodep->nodeType == STL_NODE_FI
			&& (pmportp->capmask & PI_CM_IS_DEVICE_MGT_SUPPORTED));
}
#endif

// can be called for a PmaAvoid neighbor, but a PmaAvoid port will never
// be added to any group
// non-static is temporary
boolean PmCompareSWPort(PmPort_t *pmportp, char *groupName)
{
	// groupName parameter not used in this function

	// PmaAvoid can indicate non-enhanced Port 0
	return (pmportp->pmnodep->nodeType == STL_NODE_SW);
}

static boolean PmCompareUserPGPort(PmPort_t *pmportp, char *groupName)
{
	int i, j;
	int pgIndex;
	boolean res = 0;
	Pm_t *pmp = &g_pmSweepData;
	PMXmlConfig_t *pmxmlp = &pm_config;
	PmPortImage_t *portImage = &pmportp->Image[pmp->SweepIndex];

	for (i = 0; i < MAX_DEVGROUPS; i++) {
		if (portImage->dgMember[i] == DEFAULT_DEVGROUP_ID)
			break;
		pgIndex = getUserPGIndex(groupName);
		if ((pgIndex < 0) || (pgIndex >= STL_PM_MAX_CUSTOM_PORT_GROUPS))
			break;
		for (j = 0; j < STL_PM_MAX_DG_PER_PMPG; j++) {
			if(pmxmlp->pm_portgroups[pgIndex].Monitors[j].dg_Index == portImage->dgMember[i]){
				res = 1;
				break;
			}
		}
	}
	return(res);
}

static FSTATUS PmCreateGroup(Pm_t *pm, const char* name, PmComparePortFunc_t ComparePortFunc)
{
	PmGroup_t *groupp;

	if (pm->NumGroups >= PM_MAX_GROUPS)
		return FUNAVAILABLE;

	groupp = pm->Groups[pm->NumGroups];
	cs_strlcpy(groupp->Name, name, STL_PM_GROUPNAMELEN);
	groupp->ComparePortFunc = ComparePortFunc;
	pm->NumGroups++;
	return FSUCCESS;
}

// Initialize Pm_t.
//
// clearThreshold can set a point at which error counters are cleared
// If NULL, we will compute an appropriate threshold based on error thresholds
// Otherwise explicit thresholds for each counter can be provided.
// The counter will be cleared when it exceeds the threshold.
// Hence a 0 for a threshold means a clear when any non-zero is observed.
// Similarly a 255 for an 8 bit counter means the counter will never be cleared
// (eg. an 8 bit counter can never exceed 255).
// Note that if any one error counter triggers a clear, all error counters are
// cleared.
//
// For example, SwitchRelayErrors might have a clearThreshold of 0xffff
// While LinkErrorRecovery may have a clearThreshold of 0.
// Others could have a threshold at a midway point in range
// Note that when a threshold is non-zero, max value for an interval which
// can be reported will be (counterMax-clearThreshold).
// Hence clearThresholds should be carefully selected based on the error
// thresholds set in the PM as well as any anticipated error thresholds in
// opareport or other tools.
//
// The goal is to limit clear operations, especially for stable clean
// fabrics.  If it is desired to clear error counters every time they
// are non-zero simply fill in 0 for all clearThreshold values.
static Status_t PmInit(Pm_t *pm, EUI64 portguid, uint16 pmflags,
				uint16 interval, const ErrorSummary_t *thresholds,
			   	const IntegrityWeights_t *integrityWeights,
			   	const CongestionWeights_t *congestionWeights,
				uint8 errorClear)
{
	Status_t status = VSTATUS_OK;
	uint32 i;
	uint32 size;
	PMXmlConfig_t *pmp = &pm_config;

	IB_LOG_INFO0("PmInit");
	PM_InitStaticRateToMBps();

	MemoryClear(pm, sizeof(*pm));
	pm->flags = pmflags;
	pm->interval = interval;
	pm->Thresholds = *thresholds;
	pm->integrityWeights = *integrityWeights;
	pm->congestionWeights = *congestionWeights;

	IB_LOG_INFO_FMT(__func__, "SweepInterval=%u", pm->interval);
	IB_LOG_INFO_FMT(__func__, "Thresholds.Integrity=%u", pm->Thresholds.Integrity);
	IB_LOG_INFO_FMT(__func__, "Thresholds.Congestion=%u", pm->Thresholds.Congestion);
	IB_LOG_INFO_FMT(__func__, "Thresholds.SmaCongestion=%u", pm->Thresholds.SmaCongestion);
	IB_LOG_INFO_FMT(__func__, "Thresholds.Bubble=%u", pm->Thresholds.Bubble);
	IB_LOG_INFO_FMT(__func__, "Thresholds.Security=%u", pm->Thresholds.Security);
	IB_LOG_INFO_FMT(__func__, "Thresholds.Routing=%u", pm->Thresholds.Routing);

	IB_LOG_INFO_FMT(__func__, "ThresholdsExceededMsgLimit.Integrity=%u", (unsigned)pm_config.thresholdsExceededMsgLimit.Integrity);
	IB_LOG_INFO_FMT(__func__, "ThresholdsExceededMsgLimit.Congestion=%u", (unsigned)pm_config.thresholdsExceededMsgLimit.Congestion);
	IB_LOG_INFO_FMT(__func__, "ThresholdsExceededMsgLimit.SmaCongestion=%u", (unsigned)pm_config.thresholdsExceededMsgLimit.SmaCongestion);
	IB_LOG_INFO_FMT(__func__, "ThresholdsExceededMsgLimit.Bubble=%u", (unsigned)pm_config.thresholdsExceededMsgLimit.Bubble);
	IB_LOG_INFO_FMT(__func__, "ThresholdsExceededMsgLimit.Security=%u", (unsigned)pm_config.thresholdsExceededMsgLimit.Security);
	IB_LOG_INFO_FMT(__func__, "ThresholdsExceededMsgLimit.Routing=%u", (unsigned)pm_config.thresholdsExceededMsgLimit.Routing);

	IB_LOG_INFO_FMT(__func__, "IntegrityWeights.LocalLinkIntegrityErrors=%u", pm->integrityWeights.LocalLinkIntegrityErrors);
	IB_LOG_INFO_FMT(__func__, "IntegrityWeights.RcvErrors=%u", pm->integrityWeights.PortRcvErrors);
	IB_LOG_INFO_FMT(__func__, "IntegrityWeights.ExcessiveBufferOverruns=%u", pm->integrityWeights.ExcessiveBufferOverruns);
	IB_LOG_INFO_FMT(__func__, "IntegrityWeights.LinkErrorRecovery=%u", pm->integrityWeights.LinkErrorRecovery);
	IB_LOG_INFO_FMT(__func__, "IntegrityWeights.LinkDowned=%u", pm->integrityWeights.LinkDowned);
	IB_LOG_INFO_FMT(__func__, "IntegrityWeights.UncorrectableErrors=%u", pm->integrityWeights.UncorrectableErrors);
	IB_LOG_INFO_FMT(__func__, "IntegrityWeights.FMConfigErrors=%u", pm->integrityWeights.FMConfigErrors);
	IB_LOG_INFO_FMT(__func__, "IntegrityWeights.LinkQualityIndicator=%u", pm->integrityWeights.LinkQualityIndicator);
	IB_LOG_INFO_FMT(__func__, "IntegrityWeights.LinkWidthDowngrade=%u", pm->integrityWeights.LinkWidthDowngrade);

	IB_LOG_INFO_FMT(__func__, "CongestionWeights.XmitWait=%u", pm->congestionWeights.PortXmitWait);;
	IB_LOG_INFO_FMT(__func__, "CongestionWeights.SwPortCongestion=%u", pm->congestionWeights.SwPortCongestion);
	IB_LOG_INFO_FMT(__func__, "CongestionWeights.RcvFECN=%u", pm->congestionWeights.PortRcvFECN);
	IB_LOG_INFO_FMT(__func__, "CongestionWeights.RcvBECN=%u", pm->congestionWeights.PortRcvBECN);
	IB_LOG_INFO_FMT(__func__, "CongestionWeights.XmitTimeCong=%u", pm->congestionWeights.PortXmitTimeCong);
	IB_LOG_INFO_FMT(__func__, "CongestionWeights.MarkFECN=%u", pm->congestionWeights.PortMarkFECN);

	IB_LOG_INFO_FMT(__func__, "ErrorClear=%u", errorClear);


	pm->PmPortSize = sizeof(PmPort_t)+sizeof(PmPortImage_t)*(pm_config.total_images-1);
	pm->PmNodeSize = sizeof(PmNode_t)+sizeof(PmNodeImage_t)*(pm_config.total_images-1);

	size = sizeof(PmGroup_t)+sizeof(PmGroupImage_t)*(pm_config.total_images-1);
	status = vs_pool_alloc(&pm_pool, size, (void*)&pm->AllPorts);
	if (status != VSTATUS_OK || ! pm->AllPorts) {
		IB_LOG_ERRORRC("Failed to allocate PM AllPorts group rc:", status);
		status = VSTATUS_NOMEM;
		goto fail;
	}
	MemoryClear(pm->AllPorts, size);

	for (i=0; i < PM_MAX_GROUPS; ++i) {
		status = vs_pool_alloc(&pm_pool, size, (void*)&pm->Groups[i]);
		if (status != VSTATUS_OK || ! pm->Groups[i]) {
			IB_LOG_ERRORRC("Failed to allocate PM Groups rc:", status);
			status = VSTATUS_NOMEM;
			goto fail;
		}
		MemoryClear(pm->Groups[i], size);
	}

	size = sizeof(PmVF_t)+sizeof(PmVFImage_t)*(pm_config.total_images-1);
	for (i=0; i < MAX_VFABRICS; ++i) {
		status = vs_pool_alloc(&pm_pool, size, (void*)&pm->VFs[i]);
		if (status != VSTATUS_OK || ! pm->VFs[i]) {
			IB_LOG_ERRORRC("Failed to allocate PM VFs rc:", status);
			status = VSTATUS_NOMEM;
			goto fail;
		}
		MemoryClear(pm->VFs[i], size);
	}

	size = sizeof(PmImage_t)*pm_config.total_images;
	status = vs_pool_alloc(&pm_pool, size, (void*)&pm->Image);
	if (status != VSTATUS_OK || ! pm->Image) {
		IB_LOG_ERRORRC("Failed to allocate PM Images rc:", status);
		status = VSTATUS_NOMEM;
		goto fail;
	}
	MemoryClear(pm->Image, size);

	size = sizeof(uint32)*pm_config.total_images;
	status = vs_pool_alloc(&pm_pool, size, (void*)&pm->history);
	if (status != VSTATUS_OK || ! pm->Image) {
		IB_LOG_ERRORRC("Failed to allocate PM history indexes rc:", status);
		status = VSTATUS_NOMEM;
		goto fail;
	}
	MemoryClear(pm->history, size);

	size = sizeof(uint32)*pm_config.freeze_frame_images;
	status = vs_pool_alloc(&pm_pool, size, (void*)&pm->freezeFrames);
	if (status != VSTATUS_OK || ! pm->Image) {
		IB_LOG_ERRORRC("Failed to allocate PM freeze frame indexes rc:", status);
		status = VSTATUS_NOMEM;
		goto fail;
	}
	MemoryClear(pm->freezeFrames, size);

	// On the transition into LinkUp State, the following counters shall be
	// cleared: PortRcvErrors, LinkErrorRecovery, LocalLinkIntegrityErrors,
	// and ExcessiveBufferOverruns
	LinkDownIgnoreMask.AsReg32 = 0;
	LinkDownIgnoreMask.s.PortRcvErrors = 1;
	LinkDownIgnoreMask.s.LinkErrorRecovery = 1;
	LinkDownIgnoreMask.s.LocalLinkIntegrityErrors = 1;
	LinkDownIgnoreMask.s.ExcessiveBufferOverruns = 1;

	PM_BuildClearCounterSelect(&pm->clearCounterSelect, (pmflags & STL_PM_PROCESS_CLR_DATA_COUNTERS?1:0),
							   (pmflags & STL_PM_PROCESS_CLR_64BIT_COUNTERS?1:0),
							   (pmflags & STL_PM_PROCESS_CLR_32BIT_COUNTERS?1:0),
							   (pmflags & STL_PM_PROCESS_CLR_8BIT_COUNTERS?1:0) );

	pm->ErrorClear = errorClear;
	PmComputeClearThresholds(&pm->ClearThresholds, &pm->clearCounterSelect, errorClear);
	pm->LastSweepIndex = PM_IMAGE_INDEX_INVALID;
	pm->lastHistoryIndex = pm_config.total_images-1;
	//pm->SweepIndex = 0;	// next sweep index to use - zeroed by MemoryClear
	status = vs_lock_init(&pm->stateLock, VLOCK_FREE, VLOCK_RWTHREAD);
	if (status != VSTATUS_OK)
		IB_FATAL_ERROR_NODUMP("Can't initialize PM sweep lock");
	status = vs_lock_init(&pm->totalsLock, VLOCK_FREE, VLOCK_RWTHREAD);
	if (status != VSTATUS_OK)
		IB_FATAL_ERROR_NODUMP("Can't initialize PM totals lock");


	cl_qmap_init(&pm->AllNodes, NULL);

	for (i=0; i < pm_config.total_images; ++i)
		pm->history[i] = PM_IMAGE_INDEX_INVALID;
	for (i=0; i < pm_config.freeze_frame_images; ++i)
		pm->freezeFrames[i] = PM_IMAGE_INDEX_INVALID;

	for (i = 0; i < pm_config.total_images; ++i)
	{
		PmImage_t *pmimagep = &pm->Image[i];
		status = vs_lock_init(&pmimagep->imageLock, VLOCK_FREE, VLOCK_RWTHREAD);
		if (status != VSTATUS_OK)
			IB_FATAL_ERROR_NODUMP("Can't initialize PM Fabric lock");
		pmimagep->state = PM_IMAGE_INVALID;
	}

	cs_strlcpy(pm->AllPorts->Name, "All", STL_PM_GROUPNAMELEN);
	PmCreateGroup(pm, "HFIs", PmCompareHFIPort);
	//PmCreateGroup(pm, "TFIs", PmCompareTCAPort); //Temporary removal as TFIs do not exist in Gen1
	PmCreateGroup(pm, "SWs", PmCompareSWPort);
	for (i = 0; i < pmp->number_of_pm_groups; i++) {
		PmCreateGroup(pm, pmp->pm_portgroups[i].Name, PmCompareUserPGPort);
	}
	// Create VFs
	(void)vs_rdlock(&old_topology_lock);
	VirtualFabrics_t *VirtualFabrics = old_topology.vfs_ptr;
	for (i = 0; i < VirtualFabrics->number_of_vfs_all; i++) {
		PmVF_t *vfp = pm->VFs[i];
		cs_strlcpy(vfp->Name, VirtualFabrics->v_fabric_all[i].name, STL_PM_VFNAMELEN);
		pm->numVFs++;
		if (!VirtualFabrics->v_fabric_all[i].standby) pm->numVFsActive++;
	}
	(void)vs_rwunlock(&old_topology_lock);

#if CPU_LE
	// Process STH files only on LE CPUs
#ifndef __VXWORKS__
	// initialize short term history
	if (pm_config.shortTermHistory.enable) {
		status = PmInitHistory(pm);
	}
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR0("Can't initialize PM Short-Term History");
		goto fail;
	}
#endif
#endif	// End of #if CPU_LE


	if (VSTATUS_OK != (status = PmDispatcherInit(pm)))
		IB_FATAL_ERROR_NODUMP("Can't initialize PM Dispatcher");
	return status;

fail:
	if (pm->freezeFrames) {
		vs_pool_free(&pm_pool, pm->freezeFrames);
		pm->freezeFrames = NULL;
	}
	if (pm->history) {
		vs_pool_free(&pm_pool, pm->history);
		pm->history = NULL;
	}
	if (pm->Image) {
		vs_pool_free(&pm_pool, pm->Image);
		pm->Image = NULL;
	}
	for (i=0; i < MAX_VFABRICS; ++i) {
		if (pm->VFs[i]) {
			vs_pool_free(&pm_pool, pm->VFs[i]);
			pm->VFs[i] = NULL;
		}
	}
	for (i=0; i < PM_MAX_GROUPS; ++i) {
		if (pm->Groups[i]) {
			vs_pool_free(&pm_pool, pm->Groups[i]);
			pm->Groups[i] = NULL;
		}
	}
	if (pm->AllPorts) {
		vs_pool_free(&pm_pool, pm->AllPorts);
		pm->AllPorts = NULL;
	}

	return status;
}

void PmDestroy(Pm_t *pm)
{
	STL_LID lid;
	uint32 i;

	IB_LOG_INFO0(__func__);

	PmDispatcherDestroy(pm);

	// to be safe, we get locks to make sure nothing is using PM data
	vs_wrlock(&pm->stateLock);
	for (pm->SweepIndex = 0; pm->SweepIndex < pm_config.total_images; ++pm->SweepIndex)
	{
		PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];
		// free topology
		pmimagep->state = PM_IMAGE_INVALID;
		vs_wrlock(&pmimagep->imageLock);	// make sure no clients in here
		if (pmimagep->LidMap) {
			for (lid = 1; lid <= pmimagep->maxLid; ++lid) {
				PmNode_t *pmnodep = pmimagep->LidMap[lid];
				if (! pmnodep)
					continue;
				IB_LOG_VERBOSEX("Free lid:", lid);
				release_pmnode(pm, pmnodep);	// frees pmnodep and any ports
			}
			vs_pool_free(&pm_pool, pmimagep->LidMap);
			pmimagep->LidMap = NULL;
		}
		vs_rwunlock(&pmimagep->imageLock);
		(void)vs_lock_delete(&pmimagep->imageLock);
	}
	vs_rwunlock(&pm->stateLock);
	if (cl_qmap_count(&pm->AllNodes)) {
		IB_LOG_ERROR("Some PM Nodes not freed:", cl_qmap_count(&pm->AllNodes));
		// list should be empty, but if want could try to fixup leak by freeing
	}
	(void)vs_lock_delete(&pm->totalsLock);
	(void)vs_lock_delete(&pm->stateLock);

	if (pm->freezeFrames) {
		vs_pool_free(&pm_pool, pm->freezeFrames);
		pm->freezeFrames = NULL;
	}
	if (pm->history) {
		vs_pool_free(&pm_pool, pm->history);
		pm->history = NULL;
	}
	if (pm->Image) {
		vs_pool_free(&pm_pool, pm->Image);
		pm->Image = NULL;
	}
	for (i=0; i < MAX_VFABRICS; ++i) {
		if (pm->VFs[i]) {
			vs_pool_free(&pm_pool, pm->VFs[i]);
			pm->VFs[i] = NULL;
		}
	}
	for (i=0; i < PM_MAX_GROUPS; ++i) {
		if (pm->Groups[i]) {
			vs_pool_free(&pm_pool, pm->Groups[i]);
			pm->Groups[i] = NULL;
		}
	}
	if (pm->AllPorts) {
		vs_pool_free(&pm_pool, pm->AllPorts);
		pm->AllPorts = NULL;
	}
#ifndef __VXWORKS__
	if (pm->ShortTermHistory.currentComposite) {
		PmFreeComposite(pm->ShortTermHistory.currentComposite);
		pm->ShortTermHistory.currentComposite = NULL;
	}
	if (pm->ShortTermHistory.historyRecords) {
		for (i=0; i < pm->ShortTermHistory.totalHistoryRecords; i++) {
			if (pm->ShortTermHistory.historyRecords[i]) {
				vs_pool_free(&pm_pool, pm->ShortTermHistory.historyRecords[i]);
			}
		}
		vs_pool_free(&pm_pool, pm->ShortTermHistory.historyRecords);
	}
	if (pm->ShortTermHistory.cachedComposite) {
		PmFreeComposite(pm->ShortTermHistory.cachedComposite);
		pm->ShortTermHistory.cachedComposite = NULL;
	}
	if (pm->ShortTermHistory.LoadedImage.img) {
		clearLoadedImage(&pm->ShortTermHistory);
	}
	if (pm->ShortTermHistory.invalidFiles) {
		for (i=0; i < pm->ShortTermHistory.totalHistoryRecords; i++) {
			if (pm->ShortTermHistory.invalidFiles[i]) {
				vs_pool_free(&pm_pool, pm->ShortTermHistory.invalidFiles[i]);
			}
		}
		vs_pool_free(&pm_pool, pm->ShortTermHistory.invalidFiles);
	}
#endif
}

// After all individual ports have been tabulated, we tabulate totals for
// all groups.  We must do this after port tabulation because some counters
// need to look at both sides of a link to pick the max or combine error
// counters.  Hence we could not accuratly indicate buckets nor totals for a
// group until first pass through all ports has been completed.
//
// caller should have imageLock held write for this image and
// held read for previous image
static void PmFinalizeAllPortStats(Pm_t *pm)
{
	STL_LID lid;
	PmPort_t *pmportp;
	PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];
	PmThresholdsExceededMsgLimitXmlConfig_t thresholdsExceededMsgCount = {0};

	// get totalsLock so we can update RunningTotals and avoid any races
	// with paClearPortCounters
	(void)vs_wrlock(&pm->totalsLock);
	for (lid = 1; lid <= pmimagep->maxLid; ++lid) {
		PmNode_t *pmnodep = pmimagep->LidMap[lid];
		if (!pmnodep)
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			uint8 i;
			for (i = 0; i <= pmnodep->numPorts; ++i) {
				pmportp = pmnodep->up.swPorts[i];
				if (pmportp && !pmportp->u.s.PmaAvoid)
					PmFinalizePortStats(pm, pmportp, pm->SweepIndex);
			}
		} else {
			pmportp = pmnodep->up.caPortp;
			if (pmportp && !pmportp->u.s.PmaAvoid)
				PmFinalizePortStats(pm, pmportp, pm->SweepIndex);
		}
	}
	(void)vs_rwunlock(&pm->totalsLock);

	// Log Ports which exceeded threshold up to ThresholdExceededMsgLimit
#define LOG_EXCEEDED_THRESHOLD(stat) \
	do { \
		if (thresholdsExceededMsgCount.stat < pm_config.thresholdsExceededMsgLimit.stat) { \
			uint32 stat = compute##stat(pm, pm->SweepIndex, pmportp, NULL); \
			if (ComputeErrBucket(stat, pm->Thresholds.stat) >= (STL_PM_CATEGORY_BUCKETS-1)) { \
				PmPort_t *pmportp2 = pmportp->Image[pm->SweepIndex].neighbor; \
				PmPrintExceededPort(pmportp, pm->SweepIndex, #stat, pm->Thresholds.stat, stat); \
				PmPrintExceededPortDetails##stat(pmportp, pmportp2, pm->SweepIndex, pm->LastSweepIndex); \
				thresholdsExceededMsgCount.stat++; \
			} \
		} \
	} while (0)

	for (lid = 1; lid <= pmimagep->maxLid; ++lid) {
		PmNode_t *pmnodep = pmimagep->LidMap[lid];
		if (!pmnodep)
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			uint8 i;
			for (i = 0; i <= pmnodep->numPorts; ++i) {
				pmportp = pmnodep->up.swPorts[i];
				if (pmportp && !pmportp->u.s.PmaAvoid
					&& pmportp->Image[pm->SweepIndex].u.s.active) {
					LOG_EXCEEDED_THRESHOLD(Integrity);
					LOG_EXCEEDED_THRESHOLD(Congestion);
					LOG_EXCEEDED_THRESHOLD(SmaCongestion);
					LOG_EXCEEDED_THRESHOLD(Bubble);
					LOG_EXCEEDED_THRESHOLD(Security);
					LOG_EXCEEDED_THRESHOLD(Routing);
				}
			}
		} else {
			pmportp = pmnodep->up.caPortp;
			if (pmportp && !pmportp->u.s.PmaAvoid
				&& pmportp->Image[pm->SweepIndex].u.s.active) {
				LOG_EXCEEDED_THRESHOLD(Integrity);
				LOG_EXCEEDED_THRESHOLD(Congestion);
				LOG_EXCEEDED_THRESHOLD(SmaCongestion);
				LOG_EXCEEDED_THRESHOLD(Bubble);
				LOG_EXCEEDED_THRESHOLD(Security);
				LOG_EXCEEDED_THRESHOLD(Routing);
			}
		}
	}
#undef LOG_EXCEEDED_THRESHOLD
}

// for all nodes clear tabulated counters which can tabulate information
// from both sides of a link
// this sends no packets on the wire
void PmClearAllNodes(Pm_t *pm)
{
	STL_LID lid;
	PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];

	for (lid = 1; lid <= pmimagep->maxLid; ++lid) {
		PmNode_t *pmnodep = pmimagep->LidMap[lid];
		PmPort_t *pmportp;
		if (! pmnodep)
			continue;
		if (pmnodep->nodeType == STL_NODE_SW) {
			uint8 i;
			for (i=0; i<= pmnodep->numPorts; ++i) {
				pmportp = pmnodep->up.swPorts[i];
				if (pmportp) {
					PmClearPortImage(&pmportp->Image[pm->SweepIndex]);
				}
			}
		} else {
			pmportp = pmnodep->up.caPortp;
			PmClearPortImage(&pmportp->Image[pm->SweepIndex]);
		}
	}
}

static void PmPrintFailNode(Pm_t *pm, PmNode_t *pmnodep, uint8 method, uint16 aid)
{
	PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];

	if (pmimagep->NoRespNodes + pmimagep->NoRespPorts +
            pmimagep->UnexpectedClearPorts
                < pm_config.SweepErrorsLogThreshold)
	{
		IB_LOG_WARN_FMT(__func__, "Unable to %s(%s) %.*s Guid "FMT_U64" LID 0x%x",
			StlPmMadMethodToText(method), StlPmMadAttributeToText(aid),
			(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
			pmnodep->NodeGUID, pmnodep->Image[pm->SweepIndex].lid);
	} else {
		IB_LOG_INFO_FMT(__func__, "Unable to %s(%s) %.*s Guid "FMT_U64" LID 0x%x",
			StlPmMadMethodToText(method), StlPmMadAttributeToText(aid),
			(int)sizeof(pmnodep->nodeDesc.NodeString), pmnodep->nodeDesc.NodeString,
			pmnodep->NodeGUID, pmnodep->Image[pm->SweepIndex].lid);
	}
}

static void PmPrintFailPort(Pm_t *pm, PmPort_t *pmportp, uint8 method, uint16 aid)
{
	PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];

	if (pmimagep->NoRespNodes + pmimagep->NoRespPorts +
            pmimagep->UnexpectedClearPorts
                < pm_config.SweepErrorsLogThreshold)
	{
		IB_LOG_WARN_FMT(__func__, "Unable to %s(%s) %.*s Guid "FMT_U64" LID 0x%x Port %u",
			StlPmMadMethodToText(method), StlPmMadAttributeToText(aid),
			(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), pmportp->pmnodep->nodeDesc.NodeString,
			pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[pm->SweepIndex].lid, pmportp->portNum);
	} else {
		IB_LOG_INFO_FMT(__func__, "Unable to %s(%s) %.*s Guid "FMT_U64" LID 0x%x Port %u",
			StlPmMadMethodToText(method), StlPmMadAttributeToText(aid),
			(int)sizeof(pmportp->pmnodep->nodeDesc.NodeString), pmportp->pmnodep->nodeDesc.NodeString,
			pmportp->pmnodep->NodeGUID, pmportp->pmnodep->Image[pm->SweepIndex].lid, pmportp->portNum);
	}
}

// Port is filtered or does not support a PMA
void PmSkipPort(Pm_t *pm, PmPort_t *pmportp)
{
	DEBUG_ASSERT(pmportp->Image[pm->SweepIndex].u.s.queryStatus == PM_QUERY_STATUS_OK);
	pm->Image[pm->SweepIndex].SkippedPorts++;
	pmportp->Image[pm->SweepIndex].u.s.queryStatus = PM_QUERY_STATUS_SKIP;
}

// Node is filtered or does not support PMA
void PmSkipNode(Pm_t *pm, PmNode_t *pmnodep)
{

	if (pmnodep->nodeType == STL_NODE_SW) {
		int i;
		for (i=0; i<=pmnodep->numPorts; ++i) {
			PmPort_t *pmportp;
			if (NULL != (pmportp = pmnodep->up.swPorts[i])) {
				PmSkipPort(pm, pmnodep->up.swPorts[i]);
			}
		}
	} else {
		PmSkipPort(pm, pmnodep->up.caPortp);
	}
	pm->Image[pm->SweepIndex].SkippedNodes++;
}

void PmFailPort(Pm_t *pm, PmPort_t *pmportp, uint8 queryStatus, uint8 method, uint16 aid)
{

	PmPrintFailPort(pm, pmportp, method, aid);
	// don't tabulate if we already skipped or failed.  if query fail its
	// reported 1st and more important than clear fail
	if (pmportp->Image[pm->SweepIndex].u.s.queryStatus == PM_QUERY_STATUS_OK) {
		pm->Image[pm->SweepIndex].NoRespPorts++;
		INCREMENT_PM_COUNTER(pmCounterPmNoRespPorts);
		pmportp->Image[pm->SweepIndex].u.s.queryStatus = queryStatus;
	}
}	// End of PmFailPort()

void PmFailPacket(Pm_t *pm, PmDispatcherPacket_t *disppacket, uint8 queryStatus, uint8 method, uint16 aid)
{	
	int i, j;
	uint64 PortSelectMask;
	PmPort_t *pmportp;
	if (disppacket->dispnode->info.state == PM_DISP_NODE_CLR_PORT_STATUS) {
		for (j = 3; j >= 0; j--) {
			for (i = 0, PortSelectMask = disppacket->PortSelectMask[j]; i < 64 && (PortSelectMask); i++, PortSelectMask >>= (uint64)1 ) {
				if (PortSelectMask & (uint64)1) {
					pmportp = disppacket->dispnode->info.pmnodep->up.swPorts[i+(3-j)*64];
					PmPrintFailPort(pm, pmportp, method, aid);
					// don't tabulate if we already skipped or failed.  if query fail its
					// reported 1st and more important than clear fail
					if (pmportp->Image[pm->SweepIndex].u.s.queryStatus == PM_QUERY_STATUS_OK) {
						pm->Image[pm->SweepIndex].NoRespPorts++;
						INCREMENT_PM_COUNTER(pmCounterPmNoRespPorts);
						pmportp->Image[pm->SweepIndex].u.s.queryStatus = queryStatus;
					}
				}
			}
		}
	}
	else {
		for (i = 0; i < disppacket->numPorts; i++) {
			pmportp = disppacket->DispPorts[i].pmportp;
			PmPrintFailPort(pm, pmportp, method, aid);
			// don't tabulate if we already skipped or failed.  if query fail its
			// reported 1st and more important than clear fail
			if (pmportp->Image[pm->SweepIndex].u.s.queryStatus == PM_QUERY_STATUS_OK) {
				pm->Image[pm->SweepIndex].NoRespPorts++;
				INCREMENT_PM_COUNTER(pmCounterPmNoRespPorts);
				pmportp->Image[pm->SweepIndex].u.s.queryStatus = queryStatus;
			}
		}
	}
}	// End of PmFailPacket()

void PmFailNode(Pm_t *pm, PmNode_t *pmnodep, uint8 queryStatus, uint8 method, uint16 aid)
{
	PmImage_t *pmimagep = &pm->Image[pm->SweepIndex];

	PmPrintFailNode(pm, pmnodep, method, aid);
	if (pmnodep->nodeType == STL_NODE_SW) {
		int i;
		for (i=0; i<=pmnodep->numPorts; ++i) {
			PmPort_t *pmportp;
			if (NULL != (pmportp = pmnodep->up.swPorts[i])) {
				if (pmportp->u.s.PmaAvoid) // ! PortHasPma(portp)
					continue;
				// don't tabulate if we already skipped or failed.  if query
				// fail its reported 1st and more important than clear fail
				if (pmportp->Image[pm->SweepIndex].u.s.queryStatus == PM_QUERY_STATUS_OK) {
					pmimagep->NoRespPorts++;
					INCREMENT_PM_COUNTER(pmCounterPmNoRespPorts);
					pmportp->Image[pm->SweepIndex].u.s.queryStatus = queryStatus;
				}
			}
		}
	} else {
		// don't tabulate if we already skipped or failed.  if query fail its
		// reported 1st and more important than clear fail
		if (pmnodep->up.caPortp->Image[pm->SweepIndex].u.s.queryStatus == PM_QUERY_STATUS_OK) {
			pmnodep->up.caPortp->Image[pm->SweepIndex].u.s.queryStatus = queryStatus;
			pmimagep->NoRespPorts++;
			INCREMENT_PM_COUNTER(pmCounterPmNoRespPorts);
		}
	}
	//pmimagep->NoRespNodes++;	// incremented in caller once for whole node
	//INCREMENT_PM_COUNTER(pmCounterPmNoRespNodes);
}	// End of PmFailNode()

// perform a "minor sweep", fetch and analyze all port counters
// (clear as needed).  Assumes topology has not changed since last "major sweep"
// caller should have imageLock
static FSTATUS PmSweepCounters(Pm_t *pm)
{
	FSTATUS status;

	status = PmSweepAllPortCounters(pm);
	if (status != FSUCCESS)
		return status;
	if (pm_shutdown || g_pmEngineState != PM_ENGINE_STARTED) {
		IB_LOG_INFO0("PM Engine shut down requested");
		return FNOT_DONE;
	}

	PmFinalizeAllPortStats(pm);
	return FSUCCESS;
}

// perform a Pm Major sweep, checks for topology changes, copies SA topology
// then sweeps PMAs and tabulates results
// returns TRUE if have topology and swept, FALSE otherwise
static FSTATUS PmSweep(Pm_t *pm)
{
    FSTATUS ret = FSUCCESS;
	PmImage_t *pmimagep;
	uint64_t sweepStart;
	uint64_t now;
	time_t now_time;

	(void)vs_wrlock(&pm->stateLock);
	pmimagep = &pm->Image[pm->SweepIndex];
	pmimagep->sweepNum = pm->NumSweeps;
	pmimagep->state = PM_IMAGE_INPROGRESS;

	(void)vs_wrlock(&pmimagep->imageLock);
	(void)vs_rwunlock(&pm->stateLock);
	vs_stdtime_get(&pmimagep->sweepStart);
	vs_time_get(&sweepStart);

	// now since we marked the image state, we could technically unlock
	// imageLock, but no harm in holding it, easier to maintain and review code
	ret = pm_copy_topology(pm);

	if (ret == FINSUFFICIENT_MEMORY || pm_shutdown || g_pmEngineState != PM_ENGINE_STARTED) {
        if (ret == FINSUFFICIENT_MEMORY)
            pm_shutdown = 1;
		IB_LOG_INFO0("PM Engine shut down requested");
		ret = FNOT_DONE;;
		goto unlock_image;
	}

#if ENABLE_PMA_SWEEP
	if (ret != FSUCCESS) {	/* standby */
		g_pmFirstSweepAsMaster = TRUE;
		goto unlock_image;
	}

#if CPU_LE
	// Process STH files only on LE CPUs
#ifndef __VXWORKS__
	if (pm_config.shortTermHistory.enable && g_pmFirstSweepAsMaster) {
		if (pm->ShortTermHistory.currentComposite) {
			//became master; free the old image now that it is no longer being used
			PmFreeComposite(pm->ShortTermHistory.currentComposite);
			pm->ShortTermHistory.currentComposite = NULL;
		}
		if (pm->ShortTermHistory.cachedComposite) {
			PmFreeComposite(pm->ShortTermHistory.cachedComposite);
			pm->ShortTermHistory.cachedComposite = NULL;
		}
		if (pm->ShortTermHistory.LoadedImage.img) {
			clearLoadedImage(&pm->ShortTermHistory);
		}

		ret = PmLoadHistory(pm, 1);
		if (ret != VSTATUS_OK) {
			IB_LOG_ERROR0("Failed while loading existing PM Short-Term History records");
			/* clean up */
			if (pm->ShortTermHistory.historyRecords) {
				int i;
				for (i=0; i < pm->ShortTermHistory.totalHistoryRecords; i++) {
					if (pm->ShortTermHistory.historyRecords[i]) {
						vs_pool_free(&pm_pool, pm->ShortTermHistory.historyRecords[i]);
					}
				}
				vs_pool_free(&pm_pool, pm->ShortTermHistory.historyRecords);
			}
		}
	}
#endif
#endif	// End of #if CPU_LE

	// Set Unexpected Clear Flag for User Counters on a Failover
	if (g_pmFirstSweepAsMaster && pm->NumSweeps > 1) {
		IB_LOG_INFO_FMT(__func__, "PA User Counters were reset on Failover: NumSweeps %u", pm->NumSweeps);
		isUnexpectedClearUserCounters = TRUE;
	} else {
		isUnexpectedClearUserCounters = FALSE;
	}
	// do Pma queries for all ports in device and tabulate
	ret = PmSweepCounters(pm);
	if (ret != FSUCCESS)
		goto unlock_image;
	if (pm->NumSweeps && IB_LOG_IS_INTERESTED(VS_LOG_INFO))
		DisplayPm(pm);
	vs_time_get(&now);
	vs_stdtime_get(&now_time);
	pmimagep->sweepDuration = (uint32)(now - sweepStart);
	IB_LOG_INFO_FMT(__func__, "Sweep Done: %d packets, %d retries, duration %d.%.3d ms",
					(int)AtomicRead(&pmCounters[pmCounterPmPacketTransmits].sinceLastSweep),
					(int)AtomicRead(&pmCounters[pmCounterPacketRetransmits].sinceLastSweep),
				   	pmimagep->sweepDuration/1000, pmimagep->sweepDuration%1000);
	SET_PM_PEAK_COUNTER(pmMaxSweepTime, (uint32)(pmimagep->sweepDuration/1000));
	if (pmimagep->sweepDuration > VTIMER_1S*pm->interval) {
		IB_LOG_WARN_FMT(__func__, "Sweep Done: Duration exceeds SweepInterval: %d packets, %d retries, duration %d.%.3d sec, interval %d sec",
					(int)AtomicRead(&pmCounters[pmCounterPmPacketTransmits].sinceLastSweep),
					(int)AtomicRead(&pmCounters[pmCounterPacketRetransmits].sinceLastSweep),
				   	pmimagep->sweepDuration/1000000, (pmimagep->sweepDuration/1000)%1000,
					pm->interval);
		// Sweep exceeded expected interval, set it to this image's interval
		pmimagep->imageInterval = pmimagep->sweepDuration/VTIMER_1S;
	} else {
		// Sweep was within the expected interval
		pmimagep->imageInterval = pm->interval;
	}
	(void)vs_rwunlock(&pmimagep->imageLock);

#if 0
	pm_print_counters_to_stream(stdout);
#endif
	pm_process_sweep_counters();

	(void)vs_wrlock(&pm->stateLock);
	pmimagep->state = PM_IMAGE_VALID;
	if (!g_pmFirstSweepAsMaster ||
			(pm->LastSweepIndex != PM_IMAGE_INDEX_INVALID && pm->Image[pm->LastSweepIndex].state == PM_IMAGE_VALID)) {
		pm->LastSweepIndex = pm->SweepIndex;	// last valid sweep
		pm->lastHistoryIndex = (pm->lastHistoryIndex+1)%pm_config.total_images;
		pm->history[pm->lastHistoryIndex] = pm->LastSweepIndex;
		pmimagep->historyIndex = pm->lastHistoryIndex;
	}
#if CPU_LE
	// Process STH files only on LE CPUs
#ifndef __VXWORKS__
	if (pm_config.shortTermHistory.enable && (!g_pmFirstSweepAsMaster ||
			(pm->LastSweepIndex != PM_IMAGE_INDEX_INVALID && pm->Image[pm->LastSweepIndex].state == PM_IMAGE_VALID))) {
		// compound the image that was just created into the current composite image
		ret = compoundNewImage(pm);
		if (ret != FSUCCESS) {
			IB_LOG_WARNRC("Error while trying to compound new sweep image rc:", ret);
		}
	}
#endif
#endif	// End of #if CPU_LE
	g_pmFirstSweepAsMaster = FALSE;
	// find next free Image to use, skip FreezeFrame images
	do {
		pm->SweepIndex = (pm->SweepIndex+1)%pm_config.total_images;
		if (! pm->Image[pm->SweepIndex].ffRefCount)
			break;
		// check lease
		if (now_time > pm->Image[pm->SweepIndex].lastUsed
			&& now_time - pm->Image[pm->SweepIndex].lastUsed > pm_config.freeze_frame_lease) {
			pm->Image[pm->SweepIndex].ffRefCount = 0;
			// paAccess will clean up FreezeFrame on next access or FF Create
		}
		// skip past images which are frozen
	} while (pm->Image[pm->SweepIndex].ffRefCount);

	// let everyone get out and stay out of our next image
	pm->Image[pm->SweepIndex].state = PM_IMAGE_INPROGRESS;

	pm->NumSweeps++;
	vs_rwunlock(&pm->stateLock);

	// TBD - if reboot of node occurs between sweeps without PM noticing
	// the counters obtained from last sweep could be inaccurate as now the
	// counters could be zeroed due to reboot
	// TBD - add a flag to PmPortData.SweepData[] to indicate if sweep data is valid.  When a port 1st appears, its 1st sweep will not have a baseline to compute delta against so it should be ignored.  Or just treat as if CLEARED_DATAXFER and CLEARED_ERRORS for port

#endif
	return ret;

unlock_image:
	(void)vs_rwunlock(&pmimagep->imageLock);
	return ret;
}

static void
PmEngineMain(uint32_t args, uint8_t **argv)
{
	uint64_t next, now;
	Eventset_t events;
	Status_t rc;

	g_pmEngineThreadRunning = TRUE;
	vs_log_output_message("PM: Engine starting up, will be monitoring and clearing port counters", FALSE);
#if CPU_BE
	// Warn about no STH file processing on BE CPUs
	if (pm_config.shortTermHistory.enable)
		IB_LOG_WARN_FMT(__func__, "Short-term history file processing bypassed on big-endian CPU\n");
#endif
	while (! pm_shutdown && g_pmEngineState == PM_ENGINE_STARTED) {
		(void)vs_time_get(&now);
		next = now + VTIMER_1S*g_pmSweepData.interval;
		if (FSUCCESS != PmSweep(&g_pmSweepData) )
			next =  now + VTIMER_1S;	// wait only 1 second if we can't sweep
		(void)vs_time_get(&now);
		if (next > now) {
			rc=vs_event_wait(&g_pmEngineShutdownEvent, (next - now), (Eventset_t)1, &events);
			IB_LOG_DEBUG1_FMT(__func__, "Waited for event for %"PRIu64" sec and %"PRIu64" ms, rc: %s", (next-now)/(uint64_t)VTIMER_1S, ((next-now)%(uint64_t)VTIMER_1S)/(uint64_t)1000, cs_convert_status(rc));
		}
	}
	IB_LOG_INFO0("PM Engine thread done");
	g_pmEngineThreadRunning = FALSE;
}

// TBD - start order, should let engine get running before announce PM
// availability and start accepting PA requests
void PmEngineStart(void)
{
	Status_t status;

	if (ENABLE_ENGINE) {
		status = PmInit(&g_pmSweepData, pm_config.port_guid,
						(pm_config.process_hfi_counters ? STL_PM_PROCESS_HFI_COUNTERS : 0)
						| (pm_config.process_vl_counters ? STL_PM_PROCESS_VL_COUNTERS : 0)
						| (pm_config.ClearDataXfer ? STL_PM_PROCESS_CLR_DATA_COUNTERS : 0)
						| (pm_config.Clear64bit ? STL_PM_PROCESS_CLR_64BIT_COUNTERS : 0)
						| (pm_config.Clear32bit ? STL_PM_PROCESS_CLR_32BIT_COUNTERS : 0)
						| (pm_config.Clear8bit ? STL_PM_PROCESS_CLR_8BIT_COUNTERS : 0),
						pm_config.sweep_interval,
						&g_pmThresholds, &g_pmIntegrityWeights, &g_pmCongestionWeights,
						pm_config.ErrorClear);
		if (status != VSTATUS_OK)
			IB_FATAL_ERROR_NODUMP("Unable to initialize Pm Sweep Engine");

		status = cs_sema_create(&g_pmAsyncRcvSema, 0);
		if (status != VSTATUS_OK)
			IB_FATAL_ERROR_NODUMP("Unable to create sema for Pm Async Rcv Thread");
		pm_async_send_rcv_cntxt = &g_pmSweepData.Dispatcher.cntx;
		status = vs_thread_create(&g_pmAsyncRcvThread, (void*)"PmAsyncRcv",
			   		pm_async_rcv, 0, NULL, PM_ASYNC_RCV_STACK_SIZE);
		if (status != VSTATUS_OK)
			IB_FATAL_ERROR_NODUMP("Unable to start Pm Async Rcv Thread");
		while ((status = cs_psema(&g_pmAsyncRcvSema)) != VSTATUS_OK)
			IB_LOG_ERRORRC("timeout waiting for Async Rcv Thread to start rc:", status);
		status = vs_event_create(&g_pmEngineShutdownEvent,
					(unsigned char*)"PM Engine Stop", (Eventset_t)0);
		if (status != VSTATUS_OK)
			IB_FATAL_ERROR_NODUMP("Unable to create event for Pm Sweep Engine");

		g_pmEngineState = PM_ENGINE_STARTED;
		status = vs_thread_create(&g_pmEngineThread, (void*)"PmEngine",
				   		PmEngineMain, 0, NULL, PM_ENGINE_STACK_SIZE);
		if (status != VSTATUS_OK) {
			g_pmEngineState = PM_ENGINE_STOPPED;
			IB_FATAL_ERROR_NODUMP("Unable to start Pm Sweep Engine");
		}
		status = vs_thread_create(&g_PmDbsyncThread, (void*)"PmDbsyncThread",
								  PmDbsyncThread, 0, NULL, PM_DBSYNC_THREAD_STACK_SIZE);
		if (status != VSTATUS_OK)
			IB_FATAL_ERROR_NODUMP("Unable to start Pm Dbsync Thread");
	} else {
		vs_log_output_message("PM: Engine disabled, configure Pm.SweepInterval to enable PM Engine", FALSE);
	}
}

boolean PmEngineRunning(void)
{
	return (g_pmEngineThreadRunning && ! pm_shutdown && g_pmEngineState == PM_ENGINE_STARTED);
}

void PmEngineStop(void)
{
	int i;
	Status_t rc;

	if (g_pmEngineState != PM_ENGINE_STOPPED) {
		IB_LOG_INFINI_INFO0("PM Engine Shutting down");

		// let PA know we are going down
		g_pmEngineState = PM_ENGINE_STOPPING;
		// wait for any PA queries in progress to finish
		while (AtomicRead(&g_pmSweepData.refCount)) {
			vs_thread_sleep(VTIMER_1S/100);
		}

		if (g_pmEngineThreadRunning) {
			// now tell engine to stop
			rc = vs_event_post(&g_pmEngineShutdownEvent, VEVENT_WAKE_ONE, (Eventset_t)1);
			IB_LOG_VERBOSERC("posted shutdown event rc:", rc);
			// wait up to 10 seconds for thread to exit gracefully
			for (i=0; g_pmEngineThreadRunning && i < 10; i++) {
				vs_thread_sleep(VTIMER_1S/2);
			}
			// nail it if it didn't exit gracefully
			vs_thread_kill(&g_pmEngineThread);
		}

		if (g_PmDbsyncThreadRunning) {
			PmDbsync_kill();
			// wait up to 5 seconds for thread to exit gracefully
			for (i=0; g_PmDbsyncThreadRunning && i < 10; i++) {
				vs_thread_sleep(VTIMER_1S/2);
			}
			// kill PM Dbsync thread, if it didn't exit gracefully
			vs_thread_kill(&g_PmDbsyncThread);
		}

		pm_async_rcv_kill();
		// wait up to 10 seconds for thread to exit gracefully
		for (i=0; g_pmAsyncRcvThreadRunning && i < 10; i++) {
			vs_thread_sleep(VTIMER_1S/2);
		}
		// nail it if it didn't exit gracefully
		vs_thread_kill(&g_pmAsyncRcvThread);
		cs_sema_delete(&g_pmAsyncRcvSema);

		// cleanup resources
		PmDestroy(&g_pmSweepData);
		(void)vs_event_delete(&g_pmEngineShutdownEvent);

		IB_LOG_INFO0("PM Engine stopped");
		g_pmEngineState = PM_ENGINE_STOPPED;
	}
}

// TBD - when tracking count of ports in a group should we also try to
// track count of Nodes in the group?

// query running counters fetches the values
// perhaps even a summarize running counters query
// Could perhaps even treat "running counters" as a saved value + present HW counters.  esp if only clear HW counters only when must
