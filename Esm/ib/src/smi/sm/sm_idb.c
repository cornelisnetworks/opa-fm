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
//									                                         //
// FILE NAME								                                 //
//    sm_idb.c								                                 //
//									                                         //
// DESCRIPTION								                                 //
//    SM to IDB interface on VxWorks.                                        //
//									                                         //
// DATA STRUCTURES							                                 //
//    None								                                     //
//									                                         //
// FUNCTIONS								                                 //
//    sm_idb_SetSmStatus     set SM state in IDB		                     //
//									                                         //
// DEPENDENCIES								                                 //
//    ib_mad.h								                                 //
//    ib_status.h							                                 //
//									                                         //
//									                                         //
//===========================================================================//	   
#ifdef __VXWORKS__
#include "vxWorks.h"
#include "stdlib.h"
#include "stdio.h"
#include "ioLib.h"
#include "tms/common/tmsTypes.h"
#include "tms/idb/idbLib.h"
#include "tms/nvm/nvmLib.h"
#include "tms/idb/icsSmMib.h"
#include "bspcommon/h/usrBootManager.h"
#include "bspcommon/ibml/h/icsApi.h"
#include "BootCfgMgr.h"
#endif

#include "cs_log.h"
#include "sm_l.h"
#include "sa_l.h"
#include "ib_sa.h"

#ifdef __VXWORKS__

extern void sm_setPriority(uint32_t);

extern uint32_t sm_state;

extern uint32_t sm_getPriority(void);
extern uint32_t sm_getElevatedPriority(void);
extern uint32_t sm_getPmPriority(void); 
extern uint32_t sm_getPmElevatedPriority(void);
extern uint32_t sm_getLMC(void);
extern uint8_t sm_getSwitchLifetime(void);
extern int8_t sm_getHoqLife(void); 
extern uint8_t sm_getVLStall(void); 
extern uint32_t sm_getPltValue(uint32_t index); 
extern int sm_getDynamicPltSupport(void); 
extern unsigned long long sm_getMKey(void);
extern unsigned long long sm_getKey(void);
extern unsigned long long sm_getGidPrefix(void);
extern uint32_t sm_getSubnetSize(void); 
extern uint32_t sm_getTopoErrorThreshold(void);
extern uint32_t sm_getTopoAbandonThreshold(void);
extern uint32_t sm_getMaxRetries(void);
extern uint32_t sm_getRcvWaitTime(void);
extern uint32_t sm_getNonRespDropTime(void);
extern uint32_t sm_getNonRespDropSweeps(void); 
//extern uint32_t sm_getLogLevel(void);
extern uint32_t sm_getMcLidTableCap(void);
extern uint32_t sm_getMasterPingInterval(void);
extern uint32_t sm_getMasterPingFailures(void);
extern uint32_t sm_getDbSyncInterval(void);
extern int32_t sm_getTrapThreshold(void);
extern uint32_t sm_getAppearanceMsgThresh(void);
uint32_t sm_numberOfVfs(void);
char* sm_VfName(uint32_t index);
uint32_t sm_mumberOfVfMcastGroups(char* vfName);
uint32_t sm_getMibOptionFlags(void);
uint16_t sm_getMibPKey(char* vfName);
char* sm_getMibPKeyDescription(char* vfName);
uint8_t sm_getDefMcGrpPKey(char* vfName, uint32_t group);
uint8_t sm_getDefMcGrpMtu(char* vfName, uint32_t group);
uint8_t sm_getDefMcGrpRate(char* vfName, uint32_t group);
uint8_t sm_getDefMcGrpSl(char* vfName, uint32_t group);
uint8_t sm_getDefMcGrpQKey(char* vfName, uint32_t group);
uint8_t sm_getDefMcGrpFlowLabel(char* vfName, uint32_t group);
uint8_t sm_getDefMcGrpTClass(char* vfName, uint32_t group);
uint32_t sm_getMcastCheck(void);
uint32_t sm_getLidOffset(void);

extern STATUS idbErrorNoAccess();
extern STATUS idbErrorNotWritable();

uint32_t idbSetUint32(char * idbName, uint32_t value);
uint32_t idbGetUint32(char * idbName);
int idbGetSmStatus() ;

/*
 * Get Index utility routine
 */
static STATUS getIdbIndex(
    char    *mibsrc,
    ulong_t *idx,
    ulong_t *idxLen,
    uchar_t  idxNum,
    ulong_t *len )
{
	IB_ENTER(__func__, 0, 0, 0, 0);

    char        buf[256]={0};
    STATUS      rc    = OK;
    uchar_t     count = 0;

    for( count = 0; count < idxNum; count++ )
    {
        rc = idbGetNext( mibsrc, idx, idxLen, buf, len );
    }

	IB_EXIT(__func__, rc);
    return rc;
} /* idbGetIndex */


/*
 * Update Sm state
 */
void idbSetSmState(ulong_t newState) {
    ulong_t   idxBuf[MAX_SUBID_INDEX_COUNT];
    ulong_t   idxcount;
    STATUS    rc=OK;
    ulong_t   len=0;

	IB_ENTER(__func__, newState, 0, 0, 0);

    memset ( (char *)idxBuf, 0, sizeof(idxBuf) );
    len = 4;
    idxcount = 0;
    if ((rc = getIdbIndex( "icsSmMib:icsSmStatusSm", idxBuf, &idxcount, 1, &len )) == OK) {
        if ((rc = idbForceSet( "icsSmMib:icsSmStatusSm", idxBuf, idxcount, &newState, len )) != OK) 
            IB_LOG_ERROR("failed to set SM State in IDB idbrc:", rc);
    } else IB_LOG_ERROR("idbGetNext failed for SM State idbrc:", rc);

	IB_EXIT(__func__, rc);
    return;
}


/*
 * Get Sm priority
 */
uint32_t idbGetSmPriority() {
    uint32_t  smPriority;

	IB_ENTER(__func__, 0, 0, 0, 0);
	smPriority = 0;
	if (idbGetSmStatus() == EicsSmControlStatus_running) {
        smPriority = sm_getPriority() ;
    }
    else {
        smPriority = 0;
	}
	IB_EXIT(__func__, smPriority);
    return smPriority;
}

/*
 * Update Sm priority
 */
uint32_t idbSetSmPriority(uint32_t newPriority) {
    uint32_t  returnVal = 1;

	IB_ENTER(__func__, newPriority, 0, 0, 0);
    returnVal = 0 ;
	IB_EXIT(__func__, returnVal);
    return(returnVal);
}

uint32_t idbSetSmElevatedPriority(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmElevatedPriority() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	if (idbGetSmStatus() == EicsSmControlStatus_running) {
		value = sm_getElevatedPriority() ;
	}
	else {
    	value = idbErrorNotWritable();
	}
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetPmPriority(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}

uint32_t idbGetPmPriority() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getPmPriority() ;
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetPmElevatedPriority(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}

uint32_t idbGetPmElevatedPriority() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getPmElevatedPriority();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

/*
 * Get GidPrefix;
 */
uint64_t idbGetGidPrefix() {
    uint64_t prefix=0xfe80000000000000LL;

	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        prefix = sm_getGidPrefix() ;
    }
    else {
        prefix = 0 ;
    }
	IB_EXIT(__func__, 0);
    return prefix;
}

/*
 * Update Sm Gid Prefix
 */
uint32_t idbSetGidPrefix(uint64_t prefix) {
    uint32_t  returnVal = 1;
    STATUS    rc=OK;

	IB_ENTER(__func__, prefix, 0, 0, 0);
    rc = idbErrorNotWritable();
    returnVal = 0 ;
	IB_EXIT(__func__, rc);
    return(returnVal);
}


/*
 * Get SmMKey; PortInfo Management key
 */
uint64_t idbGetSmMKey() {
    uint64_t  smkey=0;

	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        smkey = sm_getMKey() ;
    }
    else {
        smkey = 0 ;
    }
	IB_EXIT(__func__, 0);
    return smkey;
}

/*
 * Update SmMKey: PortInfo Management key
 */
uint32_t idbSetSmMKey(uint64_t newSmkey) {
    uint32_t  returnVal = 1;
    STATUS    rc=OK;

	IB_ENTER(__func__, newSmkey, 0, 0, 0);
    rc = idbErrorNotWritable();
    returnVal = 0 ;
	IB_EXIT(__func__, rc);
    return(returnVal);
}


/*
 * Get SmKey: key used in SMInfo
 */
uint64_t idbGetSmKey() {
    uint64_t  smkey=0;

	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        smkey = sm_getKey() ;
    }
    else {
        smkey = 0 ;
    }
	IB_EXIT(__func__, 0);
    return smkey;
}

/*
 * Update SmKey: key used in SMInfo
 */
uint32_t idbSetSmKey(uint64_t newSmkey) {
    uint32_t  returnVal = 1;
    STATUS    rc=OK;

	IB_ENTER(__func__, newSmkey, 0, 0, 0);
    rc = idbErrorNotWritable();
    returnVal = 0 ;
	IB_EXIT(__func__, rc);
    return(returnVal);
}

int idbGetSmStatus() {
    ulong_t idxBuf[32];
    ulong_t idxLen=0;
    char    valBuf[256];
    ulong_t valLen=256;
    ulong_t status;

    memset(idxBuf,0,sizeof(idxBuf));

    if (idbGetNext("icsSmMib:icsSmControlStatus",idxBuf,&idxLen,valBuf,&valLen) != OK) {
        return -1;
    }

    if (valLen != sizeof(status)) {
        return -1;
    } else {
        memcpy(&status,valBuf,sizeof(status));
    }

    return(int)status;
}


/*
 * Get Sm Sweep Rate
 */
uint32_t idbGetSmSweepRate() {
    uint32_t  smSweepRate;

	IB_ENTER(__func__, 0, 0, 0, 0);
	if (idbGetSmStatus() == EicsSmControlStatus_running) {
		smSweepRate = sm_getSweepRate() ;
	}
	else {
	 	smSweepRate = 0 ;	
	}
	IB_EXIT(__func__, smSweepRate);
    return smSweepRate;
}

/*
 * Update Sm Sweep Rate
 */
uint32_t idbSetSmSweepRate(uint32_t newSweepRate) {
    uint32_t  returnVal = 1;
    STATUS    rc=OK;

	IB_ENTER(__func__, newSweepRate, 0, 0, 0);
	rc = idbErrorNotWritable();
	returnVal = 0 ;
	IB_EXIT(__func__, rc);
    return(returnVal);
}

/*
 * Get Sm Switch Lifetime
 */
uint8_t idbGetSmSwitchLifetime() {
    uint32_t  smSwitchLifetime;
    uint8_t   returnSwitchLifetime;

	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        smSwitchLifetime = sm_getSwitchLifetime();
    }
    else {
        smSwitchLifetime = 0 ;
    }
	IB_EXIT(__func__, smSwitchLifetime);
    return returnSwitchLifetime = (uint8_t)smSwitchLifetime;
}

/*
 * Update Sm Switch Lifetime
 */
uint32_t idbSetSmSwitchLifetime(uint8_t newSwitchLifetime) {
    STATUS    rc=OK;
    uint32_t  returnVal = 1;

	IB_ENTER(__func__, newSwitchLifetime, 0, 0, 0);
    rc = idbErrorNotWritable();
    returnVal = 0;
	IB_EXIT(__func__, rc);
    return(returnVal);
}

/*
 * Get Sm Hoq Life
 */
uint8_t idbGetSmHoqLife() {
    uint8_t   returnHoqLife;
    uint32_t  smHoqLife;

	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        smHoqLife = sm_getHoqLife();
    }
    else {
        smHoqLife = 0 ;
    }
	IB_EXIT(__func__, smHoqLife);
    return returnHoqLife = (uint8_t)smHoqLife;
}

/*
 * Update Sm Hoq Life
 */
uint32_t idbSetSmHoqLife(uint8_t newHoqLife) {
    uint32_t  returnVal = 1;
    STATUS    rc=OK;

	IB_ENTER(__func__, newHoqLife, 0, 0, 0);
    rc = idbErrorNotWritable();
    returnVal = 0;
	IB_EXIT(__func__, rc);
    return(returnVal);
}

/*
 * Get Sm VL Stall
 */
uint8_t idbGetSmVLStall() {
    uint8_t   returnVLStall;
    uint32_t  smVLStall;

	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        smVLStall = sm_getVLStall();
    }
    else {
        smVLStall = 0 ;
    }

	IB_EXIT(__func__, smVLStall);
    return returnVLStall = (uint8_t)smVLStall;
}

/*
 * Update Sm VL Stall
 */
uint32_t idbSetSmVLStall(uint8_t newVLStall) {
    uint32_t  returnVal = 1;
    STATUS    rc=OK;

	IB_ENTER(__func__, newVLStall, 0, 0, 0);
    rc = idbErrorNotWritable();
    returnVal = 0;

	IB_EXIT(__func__, rc);
    return(returnVal);
}

/*
 * Get Sm Subnet Size
 */
uint32_t idbGetSmSubnetSize() {
    uint32_t  smSubnetSize;
    uint32_t  returnSubnetSize;

	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        smSubnetSize = sm_getSubnetSize() ;
    }
    else {
        smSubnetSize = 0 ;
    }
	IB_EXIT(__func__, smSubnetSize);
    return returnSubnetSize = (uint32_t)smSubnetSize;
}

/*
 * Update Sm Subnet Size
 */
uint32_t idbSetSmSubnetSize(uint32_t newSubnetSize) {
    uint32_t  returnVal = 1;
    STATUS    rc=OK;

	IB_ENTER(__func__, newSubnetSize, 0, 0, 0);
    rc = idbErrorNotWritable();
    returnVal = 0 ;
	IB_EXIT(__func__, rc);
    return(returnVal);
}

/*
 * Get Sm LMC
 */
uint32_t idbGetSmLMC() {
    uint32_t  smLMC;
    uint32_t  returnLMC;

	IB_ENTER(__func__, 0, 0, 0, 0);
	if (topology_passcount < 1) {
		sysPrintf("\nSM is currently in the %s state, count = %d\n\n", sm_getStateText(sm_state), (int)sm_smInfo.ActCount);
        smLMC = (uint32_t)-1 ;
	} else if (idbGetSmStatus() == EicsSmControlStatus_running) {
        smLMC = sm_getLMC() ;
    }
    else {
        smLMC = 0 ;
    }
	IB_EXIT(__func__, smLMC);
    return returnLMC = (uint32_t)smLMC;
}

/*
 * Update Sm LMC
 */
uint32_t idbSetSmLMC(uint32_t newLMC) {
    uint32_t  returnVal = 1;
    STATUS    rc=OK;

	IB_ENTER(__func__, newLMC, 0, 0, 0);
    rc = idbErrorNotWritable();
    returnVal = 0 ;
	IB_EXIT(__func__, rc);
    return(returnVal);
}

uint32_t idbSetUint32(char * idbName, uint32_t value) {
	ulong_t   idxBuf[MAX_SUBID_INDEX_COUNT];
	ulong_t   idxcount;
	STATUS    rc=OK;
	ulong_t   len=0;
	uint32_t  returnVal = 1;

	IB_ENTER(__func__, value, 0, 0, 0);

	memset ( (char *)idxBuf, 0, sizeof(idxBuf) );
	len = 4;
	idxcount = 0;
	if ((rc = getIdbIndex(idbName, idxBuf, &idxcount, 1, &len )) == OK) {
		if ((rc = idbSet(idbName, idxBuf, idxcount, &value, len )) != OK) {
            IB_LOG_WARN_FMT(__func__,
                   "failed to set %s to %d in IDB", idbName, value);
			returnVal = 0;
		}
	} else  {
		IB_LOG_ERROR("idbGetNext failed for value idbrc:", rc);
		returnVal = 0;
	}

	IB_EXIT(__func__, returnVal);
	return returnVal;
}

uint32_t idbGetUint32(char * idbName) {
	ulong_t   idxBuf[MAX_SUBID_INDEX_COUNT];
	uchar_t   buf[8];
	ulong_t   idxcount;
	STATUS    rc=OK;
	ulong_t   len=0;
	uint32_t  value = 0xFFFFFFFF;

	IB_ENTER(__func__, 0, 0, 0, 0);
	
	memset ( (char *)idxBuf, 0, sizeof(idxBuf) );
	len = 4;
	idxcount = 0;
	if ((rc = getIdbIndex( idbName, idxBuf, &idxcount, 1, &len )) == OK) {
		if ((rc = idbGet( idbName, idxBuf, idxcount, buf, &len )) != OK) 
		{
			IB_LOG_ERROR("failed to get value from IDB idbrc:", rc);
		} else
			memcpy(&value, buf, sizeof(value));
	} else
		IB_LOG_ERROR("idbGetNext failed idbrc:", rc);
	
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetTabularUint32(char * idbName, uint32_t index, uint32_t value) {
	ulong_t   idxBuf[3];
	ulong_t   idxLen = 3;
	uint32_t  returnVal = 1;

	IB_ENTER(__func__, index, value, 0, 0);

	idxBuf[0] = 1;
	idxBuf[1] = Ics_GetMySlotId();
	idxBuf[2] = index;

	if (idbSet(idbName, idxBuf, idxLen, &value, sizeof(value)) != OK) {
		IB_LOG_WARN_FMT(__func__,
		       "failed to set %s[%d] to %d in IDB", idbName,
		       (int) index, (int) value);
		returnVal = 0;
	}

	IB_EXIT(__func__, returnVal);
	return returnVal;
}

uint32_t idbGetTabularUint32(char * idbName, uint32_t index) {
	ulong_t   idxBuf[3];
	ulong_t   idxLen = 3;
	uchar_t   buf[8];
	ulong_t   len = 4;
	uint32_t  value = 0xFFFFFF;

	IB_ENTER(__func__, index, 0, 0, 0);

	idxBuf[0] = 1;
	idxBuf[1] = Ics_GetMySlotId();
	idxBuf[2] = index;

	if (idbGet(idbName, idxBuf, idxLen, buf, &len) != OK) {
		IB_LOG_WARN_FMT(__func__,
		       "failed to retrieve %s[%d] from IDB", idbName,
		       (int) index);
	} else {
		memcpy(&value, buf, sizeof(value));
	}

	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetSmLidOffset(uint32_t value)
{
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}

uint32_t idbGetSmLidOffset() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = sm_getLidOffset();
	IB_EXIT(__func__, value);
	return value;
}

/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmTopoErrorThreshold(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}

uint32_t idbGetSmTopoErrorThreshold() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getTopoErrorThreshold() ;
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmTopoAbandonThreshold(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}

uint32_t idbGetSmTopoAbandonThreshold() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getTopoAbandonThreshold() ;
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetSmMaxRetries(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmMaxRetries() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getMaxRetries();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetSmRcvWaitTime(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmRcvWaitTime() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getRcvWaitTime();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetSmNonrespDropTime(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmNonrespDropTime() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getNonRespDropTime();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetSmNonrespDropSweeps(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmNonrespDropSweeps() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getNonRespDropSweeps();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmLogLevel(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmLogLevel() {
	uint32_t value=0;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        //value = sm_getLogLevel();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmLogFilter(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmLogFilter", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmLogFilter() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmLogFilter");
	IB_EXIT(__func__, value);
	return value;
}

/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmLogMask(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmLogMask", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmLogMask() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmLogMask");
	IB_EXIT(__func__, value);
	return value;
}
uint32_t idbSetSmMcLidTableCap(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmMcLidTableCap() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getMcLidTableCap();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}
/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmDefMcGrpPKey(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmDefMcGrpPKey", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmDefMcGrpPKey() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmDefMcGrpPKey");
	IB_EXIT(__func__, value);
	return value;
}
/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmDefMcGrpQKey(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmDefMcGrpQKey", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmDefMcGrpQKey() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmDefMcGrpQKey");
	IB_EXIT(__func__, value);
	return value;
}
/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmDefMcGrpMtu(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmDefMcGrpMtu", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmDefMcGrpMtu() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmDefMcGrpMtu");
	IB_EXIT(__func__, value);
	return value;
}
/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmDefMcGrpRate(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmDefMcGrpRate", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmDefMcGrpRate() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmDefMcGrpRate");
	IB_EXIT(__func__, value);
	return value;
}
/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmDefMcGrpSl(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmDefMcGrpSl", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmDefMcGrpSl() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmDefMcGrpSl");
	IB_EXIT(__func__, value);
	return value;
}
uint32_t idbSetSmDefMcGrpFlowLabel(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmDefMcGrpFlowLabel", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmDefMcGrpFlowLabel() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmDefMcGrpFlowLabel");
	IB_EXIT(__func__, value);
	return value;
}
uint32_t idbSetSmDefMcGrpTClass(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmDefMcGrpTClass", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmDefMcGrpTClass() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmDefMcGrpTClass");
	IB_EXIT(__func__, value);
	return value;
}
/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmLoopTestPackets(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmLoopTestPackets", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmLoopTestPackets() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmLoopTestPackets");
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetSmMasterPingInterval(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmMasterPingInterval() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getMasterPingInterval();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetSmMaxMasterPingFailures(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmMaxMasterPingFailures() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getMasterPingFailures();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}
uint32_t idbSetSmDbSyncInterval(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmDbSyncInterval() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getDbSyncInterval();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetSmPltValue(uint32_t index, uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, index, value, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmPltValue(uint32_t index) {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getPltValue(index) ;
    }
    else {
        value = idbErrorNoAccess() ;
    }
	IB_EXIT(__func__, value);
	return value;
}


/* FIXME - Determine whether this can be set on the fly or needs sm restart */
uint32_t idbSetSmStartAtBootSelector(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmStartAtBootSelector", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmStartAtBootSelector() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmStartAtBootSelector");
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetSmControlManagerSelector(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmControlManagerSelector", value);
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmControlManagerSelector() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
	value = idbGetUint32( "icsSmMib:icsSmControlManagerSelector");
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbSetSmAppearanceMsgThresh(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmAppearanceMsgThresh() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        value = sm_getAppearanceMsgThresh();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

void idbSmGetManagersToStart(int * pm, int * fe) {

	uint32_t value;

	IB_ENTER(__func__, 0, 0, 0, 0);

	value = idbGetSmControlManagerSelector();

	*pm = (value & SM_SELECT_PM);
	*fe = (value & SM_SELECT_FE);
	
	IB_EXIT(__func__, value);
}

uint32_t idbGetSmSupportOptions() {
	uint32_t rc;
	IB_ENTER(__func__, 0, 0, 0, 0);
	rc = idbGetUint32("icsSmMib:icsSmSupportOptions");
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbSetSmSupportOptions(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
	rc = idbSetUint32("icsSmMib:icsSmSupportOptions", value);
	IB_EXIT(__func__, rc);
	return rc;
}

uint32_t idbGetSmOptionFlags() {
	uint32_t rc;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
        rc = sm_getMibOptionFlags();
    }
    else {
        rc = idbErrorNoAccess();
    }
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbSetSmOptionFlags(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}

uint32_t idbSetSmTrapThreshold(uint32_t value) {
	uint32_t rc;
	IB_ENTER(__func__, value, 0, 0, 0);
    rc = idbErrorNotWritable();
	IB_EXIT(__func__, rc);
	return rc;
}
uint32_t idbGetSmTrapThreshold() {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
         value = sm_getTrapThreshold();
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}
uint32_t idbGetMcGrpCreate(void) {
	uint32_t value;
	IB_ENTER(__func__, 0, 0, 0, 0);
    if (idbGetSmStatus() == EicsSmControlStatus_running) {
    	value = (sm_getMibOptionFlags() & SM_CREATE_MCGRP_MASK) ? 1 : 0;
    }
    else {
        value = idbErrorNoAccess();
    }
	IB_EXIT(__func__, value);
	return value;
}

uint32_t idbGetDynamicPltSupport(void) {
	return (sm_getMibOptionFlags() &  SM_DYNAMICPLT_MASK) ? 1 : 0;
}

uint32_t idbGetDisableCommonMcastMtuAndRate(void) {
	return sm_getMcastCheck();
}


#endif /*__VXWORKS__*/

Node_t *
sm_idb_find_guid(uint64_t guid, Node_t *pNode) {
	Node_t *pFoundNode;

	if(sm_util_isTopologyValid()){
        (void)vs_rdlock(&old_topology_lock);

        if((pFoundNode = sm_find_guid(&old_topology, guid)) != NULL){
            memcpy(pNode,pFoundNode,sizeof(Node_t));
        }

        (void)vs_rwunlock(&old_topology_lock);

        if(pFoundNode){
            return pNode;
        }
	}
	return NULL;
}

Node_t *
sm_idb_find_node_id(uint32_t node_id, Node_t *pNode) {
	Node_t *pFoundNode;


	if(sm_util_isTopologyValid()){
        (void)vs_rdlock(&old_topology_lock);

        if((pFoundNode = sm_find_node(&old_topology, node_id)) != NULL){
            memcpy(pNode,pFoundNode,sizeof(Node_t));
        }

        (void)vs_rwunlock(&old_topology_lock);

        if(pFoundNode){
            return pNode;
        }
	}
	return NULL;
}

Node_t *
sm_idb_find_next_guid(uint64_t guid, Node_t *pNode) {
	Node_t *pFoundNode;

	if(sm_util_isTopologyValid()){
        (void)vs_rdlock(&old_topology_lock);

        if((pFoundNode = sm_find_next_guid(&old_topology, guid)) != NULL){
            memcpy(pNode,pFoundNode,sizeof(Node_t));
        }

        (void)vs_rwunlock(&old_topology_lock);

        if(pFoundNode){
            return pNode;
        }
	}
	return NULL;
}

Node_t *
sm_idb_find_switch(uint64_t guid, Node_t *pNode) {
	Node_t *pFoundNode;


	if(sm_util_isTopologyValid()){
        (void)vs_rdlock(&old_topology_lock);

        if((pFoundNode = sm_find_guid(&old_topology, guid)) != NULL){
            if(pFoundNode->nodeInfo.NodeType == NI_TYPE_SWITCH){
                memcpy(pNode,pFoundNode,sizeof(Node_t));
            }else{
                pFoundNode = NULL;
            }
        }

        (void)vs_rwunlock(&old_topology_lock);

        if(pFoundNode){
            return pNode;
        }
	}
	return NULL;
}


Node_t *
sm_idb_find_next_switch(uint64_t guid, Node_t *pNode) {
	Node_t *pFoundNode;

	if(sm_util_isTopologyValid()){
        (void)vs_rdlock(&old_topology_lock);

        while((pFoundNode = sm_find_next_guid(&old_topology, guid)) != NULL){
            if(pFoundNode->nodeInfo.NodeType == NI_TYPE_SWITCH){
                memcpy(pNode,pFoundNode,sizeof(Node_t));
                break;
            }
            guid = pFoundNode->nodeInfo.NodeGUID;
        }

        (void)vs_rwunlock(&old_topology_lock);

        if(pFoundNode){
            return pNode;
        }
	}
	return NULL;
}


void
sm_idb_dump_node_map(void) {

	if(sm_util_isTopologyValid()){
        (void)vs_rdlock(&old_topology_lock);
        sm_dump_node_map(&old_topology);
        (void)vs_rwunlock(&old_topology_lock);
	}
}


Port_t *
sm_idb_find_port(uint64_t *guid, uint32_t *port, Port_t *pPort) {
	Node_t *pFoundNode;
	Port_t *pFoundPort = NULL;

	if(sm_util_isTopologyValid()){
        (void)vs_rdlock(&old_topology_lock);

        // Lookup the guid and then the port
        if((pFoundNode = sm_find_guid(&old_topology, *guid)) != NULL){
            pFoundPort = sm_find_node_port(&old_topology,pFoundNode,*port);
        }

        if(pFoundPort){
            memcpy(pPort,pFoundPort,sizeof(Port_t));
        }

        (void)vs_rwunlock(&old_topology_lock);

        if(sm_valid_port(pFoundPort)){
            return pPort;
        }
	}
	return NULL;
}

int getStartPort(int type){

    if(type != NI_TYPE_SWITCH){
	return 1;
    }

    return 0;
}


Port_t *
sm_idb_find_next_port(uint64_t *guid, uint32_t *port, Port_t *pPort) {
	Node_t *pFoundNode;
	Port_t *pFoundPort = NULL;
	uint32_t nextPort;

	nextPort = *port + 1;

	if(sm_util_isTopologyValid()){
        (void)vs_rdlock(&old_topology_lock);

        // First see if there is another port for this Node...
        if((pFoundNode = sm_find_guid(&old_topology, *guid)) == NULL){
            if((pFoundNode = sm_find_next_guid(&old_topology, *guid)) != NULL){
                nextPort = getStartPort(pFoundNode->nodeInfo.NodeType);
            }

        }

        while(pFoundNode){
            if((pFoundPort = sm_find_node_port(&old_topology,pFoundNode,nextPort)) == NULL){
                while((pFoundNode = sm_find_next_guid(&old_topology, pFoundNode->nodeInfo.NodeGUID)) != NULL){
                    nextPort = getStartPort(pFoundNode->nodeInfo.NodeType);
                    if((pFoundPort = sm_find_node_port(&old_topology,pFoundNode,nextPort)) != NULL){
                        break;
                    }
                }
            }else{
                if((pFoundNode->nodeInfo.NodeType != NI_TYPE_SWITCH) && (pFoundPort->state == 0)){
                    nextPort++;
                }else{
                    break;
                }
            }
        }

        if(pFoundPort){
            memcpy(pPort,pFoundPort,sizeof(Port_t));
            *guid = pFoundNode->nodeInfo.NodeGUID;
            *port = nextPort;
        }

        (void)vs_rwunlock(&old_topology_lock);

        if(sm_valid_port(pFoundPort)){
            return pPort;
        }
	}
	return NULL;
}


STL_SERVICE_RECORD*
sm_idb_find_service_record(uint64_t *serviceId, IB_GID *serviceGid, uint16_t *servicep_key, STL_SERVICE_RECORD *pSrp){

    /* with dbsync, we can display the service records in stanby */
	if(sm_util_get_state() < SM_STATE_STANDBY){
		return NULL;
	}

	return getService(serviceId, serviceGid, servicep_key, pSrp);
}

STL_SERVICE_RECORD*
sm_idb_find_next_service_record(uint64_t *serviceId, IB_GID *serviceGid, uint16_t *servicep_key, STL_SERVICE_RECORD *pSrp){

    /* with dbsync, we can display the service records in stanby */
	if(sm_util_get_state() < SM_STATE_STANDBY){
		return NULL;
	}

	return getNextService(serviceId, serviceGid, servicep_key, pSrp);
}


McGroup_t*	
sm_idb_find_broadCast_group(IB_GID * pGid, McGroup_t *pGroup){

    /* with dbsync, we can display the group records in stanby */
	if(sm_util_get_state() < SM_STATE_STANDBY){
		return NULL;
	}

	return getBroadCastGroup(pGid,pGroup);
}

McGroup_t*	
sm_idb_find_next_broadCast_group(IB_GID * pGid, McGroup_t *pGroup){

    /* with dbsync, we can display the group records in stanby */
	if(sm_util_get_state() < SM_STATE_STANDBY){
		return NULL;
	}

	return getNextBroadCastGroup(pGid,pGroup);
}

McMember_t*	
sm_idb_find_broadCast_group_member(IB_GID * pGid, uint32_t *index, McMember_t *pMember){

    /* with dbsync, we can display the group records in stanby */
	if(sm_util_get_state() < SM_STATE_STANDBY){
		return NULL;
	}

	return getBroadCastGroupMember(pGid,*index,pMember);
}

McMember_t*	
sm_idb_find_next_broadCast_group_member(IB_GID * pGid, uint32_t *index, McMember_t *pMember){

    /* with dbsync, we can display the group records in stanby */
	if(sm_util_get_state() < SM_STATE_STANDBY){
		return NULL;
	}

	return getNextBroadCastGroupMember(pGid,index,pMember);
}



