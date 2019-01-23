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

/*
 * FILE NAME
 *    sm_diag.c
 *
 * DESCRIPTION
 *    This module provides the realtime diagnostic and control of the SM/SA using the  
 *    FMI api.  The FMI sm_diag utility make use of these functions.
 *
 *
 */
#include "os_g.h"
#include "ib_types.h"
#include "ib_status.h"
#include "vs_g.h"
#include "cs_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "hsm_config_srvr_api.h"
#include "hsm_config_srvr_data.h"
#include "ibyteswap.h"
#include "ifs_g.h"

extern void setLoopPathLength(int len);
extern void setLoopMinISLRedundancy(int num);
extern int setLoopTestFastMode(int flag);
extern char* printTopology(int buffer);
extern char* printLoopPaths(int nodeIdx, int buffer);
extern char* printSwitchLft(int nodeIdx, int useNew, int haveLock, int buffer);
extern char* printLoopTestConfig(int buffer);
extern void bufferLargeSmDiagOutputs(fm_config_interation_data_t *interationData);
extern void smAdaptiveRoutingUpdate(uint32_t, fm_ar_config_t);

extern uint32_t sm_looptest_disabled_ar;

static uint32_t sm_saved_sweep_rate = 0;
static uint32_t sm_diag_loop_start = 0;
static uint32_t	sm_diag_fast_mode_start;

/*
 * sm controlled loop test start
 */
char* sm_looptest_start(int numPkts) {
	char * buf = NULL;
	int len = 500, pos = 0;
	fm_ar_config_t ar_config;

	if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("sm_looptest_start: CAN'T ALLOCATE SPACE.");
		return NULL;
	}
	buf[0] = '\0';

    if (!esmLoopTestOn) {
		/* Disable AR if it is running */
		if (sm_adaptiveRouting.enable) {
			ar_config.enable = 0;
			ar_config.frequency = -1;
			ar_config.threshold = -1;
			smAdaptiveRoutingUpdate(0, ar_config);
			sm_looptest_disabled_ar = 1;
		}

        /* set the loop test global control flag */
        esmLoopTestOn = 1;
		esmLoopTestTotalPktsInjected = 0;
		esmLoopTestForceInject = 0;
        if (numPkts < 0) 
            esmLoopTestNumPkts = 0;        /* inject would need to be used to send packets */
        else { 
            esmLoopTestNumPkts = (numPkts > 10) ? 10 : numPkts;
        	IB_LOG_INFINI_INFO("Number of packets being adjusted to a maximum of 10 from ", numPkts);
        }
		sm_saved_sweep_rate = sm_getSweepRate();
		sm_diag_loop_start = 1;
		sm_forceSweep("SM Loop Test Start");
		vs_thread_sleep(VTIMER_1S*3);   /* wait a bit for sweep to complete */
		sm_setSweepRate(0);
		pos = snprintf(buf, len, "Loop Test has been started with %u packets\n", esmLoopTestNumPkts);
		if (esmLoopTestNumPkts == 0) {
			if (sm_diag_fast_mode_start)
				snprintf(buf + pos, len - pos, "Loop Test Fast Mode is setup, but no packets have been injected and no traffic is running\n)");
			else
				snprintf(buf + pos, len - pos, "Loop Test is setup, but no packets have been injected and no traffic is running\n");
		}
    } else {
		IB_LOG_WARN0("Loop Test is already running!");
		snprintf(buf, len, "Loop Test has already been started - ignoring command!\n");
    }
	return buf;
}

/*
 * sm controlled loop test 
 * Inject a number of packets at a specific switch's loops or all switch's loops
 */
char* sm_looptest_inject_packets(int numPkts) {
	char * buf = NULL;
	int len = 500;

	if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("sm_looptest_inject_packets: CAN'T ALLOCATE SPACE.");
		return NULL;
	}
	buf[0] = '\0';

    if (esmLoopTestOn) {
        esmLoopTestInjectNode = -1;
        if (numPkts <= 0) 
            esmLoopTestNumPkts = 1;
        else { 
            esmLoopTestNumPkts = (numPkts > 10) ? 10 : numPkts;
        	IB_LOG_INFINI_INFO("Number of packets being adjusted to a maximum of 10 from ", numPkts);
        }
        IB_LOG_INFINI_INFO("Sending to all loops, numPackets=", esmLoopTestNumPkts);
        /* force a sweep to get packets injected */
		esmLoopTestForceInject = 1;
		sm_forceSweep("SM Loop Test Inject Packets");
		sprintf(buf, "Injecting %d packets into Loop Test\n", esmLoopTestNumPkts);
    }   else {
		IB_LOG_WARN0("Loop Test is NOT running!");
		sprintf(buf, "Loop Test is not running - ignoring command!\n");
    }
	return buf;
}

/*
 * sm controlled loop test 
 * Inject a number of packets at a specific switch's loops
 */
char* sm_looptest_inject_at_node(int nodeIdx) {
	char * buf = NULL;
	int len = 500;

	if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("sm_looptest_inject_at_node: CAN'T ALLOCATE SPACE.");
		return NULL;
	}
	buf[0] = '\0';

    if (esmLoopTestOn) {
        esmLoopTestInjectNode = (nodeIdx < 0) ? 0 : nodeIdx;
        if (esmLoopTestNumPkts <= 0)
            esmLoopTestNumPkts = 1;
        IB_LOG_INFINI_INFO_FMT(__func__,
			   "Sending %d packets to node index %d\n", 
			   esmLoopTestNumPkts, nodeIdx);
        /* force a sweep to get packets injected */
		esmLoopTestForceInject = 1;
		sm_forceSweep("SM Loop Test Inject Packets at Node");
		sprintf(buf, "Injecting %d packets into Loop Test at node %d\n", esmLoopTestNumPkts, nodeIdx);
    }   else {
		IB_LOG_WARN0("Loop Test is NOT running!");
		sprintf(buf, "Loop Test is not running - ignoring command!\n");
    }
	return buf;
}

/*
 * sm controlled loop test 
 * Control if each sweep will inject a new packet or not
 */
char* sm_looptest_inject_packets_each_sweep(int inject) {
	char * buf = NULL;
	int len = 500;

	if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("sm_looptest_inject_packets_each_sweep: CAN'T ALLOCATE SPACE.");
		return NULL;
	}
	buf[0] = '\0';

	if (inject != 0) {
		esmLoopTestInjectEachSweep = 1;
        IB_LOG_INFINI_INFO("loop test will inject packets every sweep, numPackets=", esmLoopTestNumPkts);
		sprintf(buf, "Loop Test will inject packets every sweep\n");
	}
	else {
		esmLoopTestInjectEachSweep = 0;
        IB_LOG_INFINI_INFO0("loop test will NOT inject packets every sweep");
		sprintf(buf, "Loop Test will not inject packets every sweep\n");
    }
    
	return buf;
}

/*
 * sm controlled loop test 
 * Set the loop path length
 */
char* sm_set_loop_path_length(int length) {
	int loop_length = 3;
	char * buf = NULL;
	int len = 500;

	if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("sm_set_loop_path_length: CAN'T ALLOCATE SPACE.");
		return NULL;
	}
	buf[0] = '\0';

	loop_length = (length < 5 && length > 1) ? length : 3;
	sprintf(buf, "Loop Test path length has been set to %d\n", loop_length);
	setLoopPathLength(loop_length);
	return buf;
}

/*
 * sm controlled loop test 
 * Set the loop path minimum redundancy
 */
char* sm_set_loop_min_redundancy(int redundancy) {
	int loop_redundancy = 0;
	char * buf = NULL;
	int len = 500;

	if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("sm_set_loop_min_redundancy: CAN'T ALLOCATE SPACE.");
		return NULL;
	}
	buf[0] = '\0';

	loop_redundancy = (redundancy > 0) ? redundancy : 4;
	sprintf(buf, "Loop Test loop path minimum ISL redundancy has been set to %d\n", loop_redundancy);
	setLoopMinISLRedundancy(loop_redundancy);
	return buf;
}


/*
 * Esm controlled loop test end
 */
char* sm_looptest_stop(void) {
	char * buf = NULL;
	int len = 500;
	fm_ar_config_t ar_config;

	if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
		IB_FATAL_ERROR_NODUMP("sm_looptest_stop: CAN'T ALLOCATE SPACE.");
		return NULL;
	}
	buf[0] = '\0';

    if (esmLoopTestOn) {
        /* set the loop test global control flag to off */
        esmLoopTestOn = 0;
        esmLoopTestInjectNode = -1;
        esmLoopTestNumPkts = 1;

		// if started in fast mode then cancel fast mode
		if (sm_diag_fast_mode_start) {
			setLoopTestFastMode(0);
			sm_diag_fast_mode_start = 0;
		}

		/* force a sweep to clear the forwarding tables */
		sm_forceSweep("SM Loop Test Stop");
		vs_thread_sleep(VTIMER_1S*3);   /* wait a bit for sweep to complete */
        loopPathLidEnd = 0;				/* give sm chance to clear loop lids */

		/* Re-enable AR if it is running */
		if (sm_looptest_disabled_ar) {
			ar_config.enable = 1;
			ar_config.frequency = -1;
			ar_config.threshold = -1;
			smAdaptiveRoutingUpdate(0, ar_config);
			sm_looptest_disabled_ar = 0;
		}

		if (!sm_diag_loop_start) {
			// started via config?
			sm_saved_sweep_rate = sm_getSweepRate();
		}
		sm_diag_loop_start = 0;

		/* put sweep rate back and force sweep to clear forwarding tables */
		sm_setSweepRate(sm_saved_sweep_rate);
		sprintf(buf, "Loop Test has been stopped\n");
    }  else {
		IB_LOG_WARN0("Loop Test is NOT running!");
		sprintf(buf, "Loop Test is not running - ignoring command!\n");
    }
	return buf;
}

fm_msg_ret_code_t sm_looptest_fast_mode(int flag) {
	fm_msg_ret_code_t	ret_code=FM_RET_OK;
	int status = 0;

	status = setLoopTestFastMode(flag);

	if (status < 0) {
		ret_code = FM_RET_ERR_WRONGTYPE;
	}

	return ret_code;
}

/*
 * loop test remote control
 */
fm_msg_ret_code_t sm_diag_ctrl(fm_config_datagram_t *msg) {
	int tmpvar=0, length = 0;
	char * tmpData = NULL;
	fm_config_interation_data_t *iterationData = NULL;
	fm_ar_config_t ar_config;
	boolean pauseSweeps = FALSE;

	switch (msg->header.data_id) {
	case FM_DT_SM_LOOP_TEST_START:
	case FM_DT_SM_LOOP_TEST_FAST_MODE_START:
	case FM_DT_SM_LOOP_TEST_FAST:
	case FM_DT_SM_LOOP_TEST_INJECT_PACKETS:
	case FM_DT_SM_LOOP_TEST_INJECT_ATNODE:
	case FM_DT_SM_LOOP_TEST_INJECT_EACH_SWEEP:
	case FM_DT_SM_LOOP_TEST_PATH_LEN:
	case FM_DT_SM_LOOP_TEST_MIN_ISL_REDUNDANCY:
	case FM_DT_LOG_LEVEL:
	case FM_DT_LOG_MODE:
	case FM_DT_LOG_MASK:
	case FM_DT_SM_FORCE_ATTRIBUTE_REWRITE:
	case FM_DT_SM_SKIP_ATTRIBUTE_WRITE:
		// Consolidate argument size checks for commands that
		// use uint32_t arguments here. Other commands should
		// do their checks inline
		if (msg->header.data_len < sizeof(uint32_t))
			goto bail_bad_len;
	}


	switch (msg->header.data_id) {
	case FM_DT_SM_LOOP_TEST_START:
		tmpvar = (*(uint32_t *)&msg->data[0]);
    	if (!esmLoopTestOn) {
			IB_LOG_INFINI_INFO("starting loopTest with num inject packets", tmpvar);
		}
		tmpData = sm_looptest_start(tmpvar);
		if (tmpData != NULL) {
			length = strlen(tmpData) + 1;
			memcpy(&msg->data[0], tmpData, MIN(msg->header.data_len, length));
			msg->data[msg->header.data_len - 1] = '\0';
			vs_pool_free(&sm_pool, tmpData);
		}
		break;

	case FM_DT_SM_LOOP_TEST_FAST_MODE_START:
		tmpvar = (*(uint32_t *)&msg->data[0]);
    	if (!esmLoopTestOn) {
			IB_LOG_INFINI_INFO("starting loopTest in fast mode with num inject packets", tmpvar);
			sm_looptest_fast_mode(1);
			sm_diag_fast_mode_start = 1;
		}
		tmpData = sm_looptest_start(tmpvar);
		if (tmpData != NULL) {
			length = strlen(tmpData) + 1;
			memcpy(&msg->data[0], tmpData, MIN(msg->header.data_len, length));
			msg->data[msg->header.data_len - 1] = '\0';
			vs_pool_free(&sm_pool, tmpData);
		}
		break;

	case FM_DT_SM_LOOP_TEST_STOP:
		tmpData = sm_looptest_stop();
		if (tmpData != NULL) {
			length = strlen(tmpData) + 1;
			memcpy(&msg->data[0], tmpData, MIN(msg->header.data_len, length));
			msg->data[msg->header.data_len - 1] = '\0';
			vs_pool_free(&sm_pool, tmpData);
		}
		break;

	case FM_DT_SM_LOOP_TEST_FAST:
		tmpvar = (*(uint32_t *)&msg->data[0]);
		return sm_looptest_fast_mode(tmpvar);

	case FM_DT_SM_LOOP_TEST_INJECT_PACKETS:
		tmpvar = (*(uint32_t *)&msg->data[0]);
		tmpData = sm_looptest_inject_packets(tmpvar);
		if (tmpData != NULL) {
			length = strlen(tmpData) + 1;
			memcpy(&msg->data[0], tmpData, MIN(msg->header.data_len, length));
			msg->data[msg->header.data_len - 1] = '\0';
			vs_pool_free(&sm_pool, tmpData);
		}
		break;

	case FM_DT_SM_LOOP_TEST_INJECT_ATNODE:
		tmpvar = (*(uint32_t *)&msg->data[0]);
		tmpData = sm_looptest_inject_at_node(tmpvar);
		if (tmpData != NULL) {
			length = strlen(tmpData) + 1;
			memcpy(&msg->data[0], tmpData, MIN(msg->header.data_len, length));
			msg->data[msg->header.data_len - 1] = '\0';
			vs_pool_free(&sm_pool, tmpData);
		}
		break;

	case FM_DT_SM_LOOP_TEST_INJECT_EACH_SWEEP:
		tmpvar = (*(uint32_t *)&msg->data[0]);
		tmpData = sm_looptest_inject_packets_each_sweep(tmpvar);
		if (tmpData != NULL) {
			length = strlen(tmpData) + 1;
			memcpy(&msg->data[0], tmpData, MIN(msg->header.data_len, length));
			msg->data[msg->header.data_len - 1] = '\0';
			vs_pool_free(&sm_pool, tmpData);
		}
		break;

	case FM_DT_SM_LOOP_TEST_PATH_LEN:
		tmpvar = (*(uint32_t *)&msg->data[0]);
		tmpData = sm_set_loop_path_length (tmpvar);
		if (tmpData != NULL) {
			length = strlen(tmpData) + 1;
			memcpy(&msg->data[0], tmpData, MIN(msg->header.data_len, length));
			msg->data[msg->header.data_len - 1] = '\0';
			vs_pool_free(&sm_pool, tmpData);
		}
		break;

	case FM_DT_SM_LOOP_TEST_MIN_ISL_REDUNDANCY:
		tmpvar = (*(uint32_t *)&msg->data[0]);
		tmpData = sm_set_loop_min_redundancy (tmpvar);
		if (tmpData != NULL) {
			length = strlen(tmpData) + 1;
			memcpy(&msg->data[0], tmpData, MIN(msg->header.data_len, length));
			msg->data[msg->header.data_len - 1] = '\0';
			vs_pool_free(&sm_pool, tmpData);
		}
		break;

	case FM_DT_SM_LOOP_TEST_SHOW_PATHS:
		if (msg->header.data_len < sizeof(fm_config_interation_data_t))
			goto bail_bad_len;
		// get interation data and then get the return buffer
		iterationData = (fm_config_interation_data_t *)&msg->data[0];
		//if start bit is set then we are starting
		if (iterationData->start) {
			iterationData->largeBuffer = printLoopPaths(iterationData->index, /* buffer */ 1);
			if (iterationData->largeBuffer == NULL) {
				iterationData->largeLength = 0;
				iterationData->done = 1;
				iterationData->start = 0;
				break;
			}
			iterationData->largeLength = strlen(iterationData->largeBuffer);
			iterationData->start = 0;
			iterationData->done = 0;
			iterationData->offset = 0;
		}
		bufferLargeSmDiagOutputs(iterationData);
		if (iterationData->intermediateLength == 0) {
			vs_pool_free(&sm_pool, iterationData->largeBuffer);
			iterationData->largeBuffer = NULL;
			iterationData->largeLength = 0;
			iterationData->done = 1;
			break;
		}
		
		msg->data[msg->header.data_len - 1] = '\0';
		
		if (iterationData->done) {
			vs_pool_free(&sm_pool, iterationData->largeBuffer);
			iterationData->largeBuffer = NULL;
			iterationData->largeLength = 0;
		}
		break;

	case FM_DT_SM_LOOP_TEST_SHOW_LFTS:
		if (msg->header.data_len < sizeof(fm_config_interation_data_t))
			goto bail_bad_len;
		// get interation data and then get the return buffer
		iterationData = (fm_config_interation_data_t *)&msg->data[0];
		//if start bit is set then we are starting
		if (iterationData->start) {
			iterationData->largeBuffer = printSwitchLft(iterationData->index, 0, 0, /* buffer */ 1);
			if (iterationData->largeBuffer == NULL) {
				iterationData->largeLength = 0;
				iterationData->done = 1;
				iterationData->start = 0;
				break;
			}
			iterationData->largeLength = strlen(iterationData->largeBuffer);
			iterationData->start = 0;
			iterationData->done = 0;
			iterationData->offset = 0;
		}
		bufferLargeSmDiagOutputs(iterationData);
		if (iterationData->intermediateLength == 0) {
			vs_pool_free(&sm_pool, iterationData->largeBuffer);
			iterationData->largeBuffer = NULL;
			iterationData->largeLength = 0;
			iterationData->done = 1;
			break;
		}
		
		msg->data[msg->header.data_len - 1] = '\0';
		
		if (iterationData->done) {
			vs_pool_free(&sm_pool, iterationData->largeBuffer);
			iterationData->largeBuffer = NULL;
			iterationData->largeLength = 0;
		}
		break;

	case FM_DT_SM_LOOP_TEST_SHOW_TOPO:
		if (msg->header.data_len < sizeof(fm_config_interation_data_t))
			goto bail_bad_len;
		// get interation data and then get the return buffer
		iterationData = (fm_config_interation_data_t *)&msg->data[0];
		//if start bit is set then we are starting
		if (iterationData->start) {
			iterationData->largeBuffer = printTopology(/* buffer */ 1);
			if (iterationData->largeBuffer == NULL) {
				iterationData->largeLength = 0;
				iterationData->done = 1;
				iterationData->start = 0;
				break;
			}
			iterationData->largeLength = strlen(iterationData->largeBuffer);
			iterationData->start = 0;
			iterationData->done = 0;
			iterationData->offset = 0;
		}
		bufferLargeSmDiagOutputs(iterationData);
		if (iterationData->intermediateLength == 0) {
			vs_pool_free(&sm_pool, iterationData->largeBuffer);
			iterationData->largeBuffer = NULL;
			iterationData->largeLength = 0;
			iterationData->done = 1;
			break;
		}
		
		msg->data[msg->header.data_len - 1] = '\0';
		
		if (iterationData->done) {
			vs_pool_free(&sm_pool, iterationData->largeBuffer);
			iterationData->largeBuffer = NULL;
			iterationData->largeLength = 0;
		}
		break;

	case FM_DT_SM_LOOP_TEST_SHOW_CONFIG:
		tmpData = printLoopTestConfig(/* buffer */ 1);
		if (tmpData != NULL) {
			length = strlen(tmpData) + 1;
			memcpy(&msg->data[0], tmpData, MIN(msg->header.data_len, length));
			msg->data[msg->header.data_len - 1] = '\0';
			vs_pool_free(&sm_pool, tmpData);
		}
		break;

	case FM_DT_FORCE_SWEEP:
		sm_forceSweep("User Requested Sweep");
		break;

	case FM_DT_LOG_LEVEL:
		sm_set_log_level(*(uint32_t *)&msg->data[0]);
		break;

	case FM_DT_LOG_MODE:
		sm_set_log_mode(*(uint32_t *)&msg->data[0]);
		break;

	case FM_DT_LOG_MASK:
		if (msg->header.data_len < (sizeof(uint32_t) + sizeof(char)))
			goto bail_bad_len;
		sm_set_log_mask((char*)&msg->data[sizeof(uint32_t)],
							*(uint32_t *)&msg->data[0]);
		break;

	case FM_DT_SM_PERF_DEBUG_TOGGLE:
		smPerfDebugToggle();
		break;

	case FM_DT_SA_PERF_DEBUG_TOGGLE:
		saPerfDebugToggle();
		break;

	case FM_DT_RMPP_DEBUG_TOGGLE:
		saRmppDebugToggle();
		break;

	case FM_DT_SM_RESTORE_PRIORITY:
		sm_restorePriority();
		break;

	case FM_DT_SM_BROADCAST_XML_CONFIG:
		// JPW - may use in future for maint mode feature - can optionally activate config with argument
		sm_send_xml_file(0);
		break;

	case FM_DT_SM_GET_COUNTERS:
		tmpData = sm_print_counters_to_buf();

		printf("%s\n", tmpData);
		if (tmpData != NULL) {
			length = strlen(tmpData) + 1;

			memcpy(&msg->data[0], tmpData, MIN(msg->header.data_len, length));
			msg->data[msg->header.data_len - 1] = '\0';
			vs_pool_free(&sm_pool, tmpData);
		}
		break;

	case FM_DT_SM_RESET_COUNTERS:
		sm_reset_counters();
		break;

	case FM_DT_SM_DUMP_STATE:
		tmpData = (char *) &(msg->data[0]);
		IB_LOG_INFINI_INFO_FMT(__func__, "Received Dump State command, "
		       "dumping state to directory %s", tmpData);

		if (sm_dump_state(tmpData) != VSTATUS_OK)
			IB_LOG_ERROR_FMT(__func__, "State dump to directory %s failed.",
			       tmpData);
		break;

	case FM_DT_SM_FORCE_REBALANCE_TOGGLE:
		smForceRebalanceToggle();	
		break;

	case FM_DT_SM_GET_ADAPTIVE_ROUTING:
		if (msg->header.data_len < sizeof(fm_ar_config_t))
			goto bail_bad_len;
		sm_get_smAdaptiveRouting(&ar_config);
		memcpy(&msg->data[0], &ar_config, sizeof(fm_ar_config_t));
		break;

	case FM_DT_SM_SET_ADAPTIVE_ROUTING:
		if (msg->header.data_len < sizeof(fm_ar_config_t))
			goto bail_bad_len;
		memcpy(&ar_config, (fm_ar_config_t *)&msg->data[0], sizeof(fm_ar_config_t));
		smAdaptiveRoutingUpdate(1, ar_config);
		break;

	case FM_DT_SM_FORCE_ATTRIBUTE_REWRITE:
		if(*(uint32_t *)&msg->data[0] == 0 || *(uint32_t *)&msg->data[0] == 1)
			sm_set_force_attribute_rewrite(*(uint32_t *)&msg->data[0]);
		break;

	case FM_DT_SM_SKIP_ATTRIBUTE_WRITE:
        sm_set_skip_attribute_write(*(uint32_t*)&msg->data[0]);
		break;

	case FM_DT_PAUSE_SWEEPS:
		pauseSweeps = TRUE;
		smPauseResumeSweeps(pauseSweeps);
		break;

	case FM_DT_RESUME_SWEEPS:
		pauseSweeps = FALSE;
		smPauseResumeSweeps(pauseSweeps);
		break;

	default:
		return FM_RET_UNKNOWN_DT;
	}

	return FM_RET_OK;

bail_bad_len:
	return FM_RET_BAD_LEN;
}
