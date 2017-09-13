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
//									     //
// FILE NAME								     //
//    sm_control.c							     //
//									     //
// DESCRIPTION								     //
//    This file contains the infrastructure needed for startup and shutdown. //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    None								     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//									     //
//===========================================================================//

#include "os_g.h"
#include "ib_types.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "ib_macros.h"
#include "cs_g.h"
#include "cs_csm_log.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sm_dbsync.h"

extern	uint32_t	sm_control_cmd;
extern  Status_t sm_clearIsSM(void);
#ifdef __LINUX__
extern void mai_umadt_read_kill(void);
#endif
extern void sm_shutdown(void);

Status_t
sm_control(Mai_t *maip)
{
	uint8_t		*cp;
	uint32_t	dev;
	uint32_t	port;
	uint32_t	amod;

	IB_ENTER(__func__, maip, maip->base.aid, 0, 0);

//
//	See if this is a loopback to me.
//
	if (maip->base.tid == (uint64_t)sm_threads[SM_THREAD_ASYNC].name) {
		IB_EXIT(__func__, 0);
		return(VSTATUS_OK);
	}

//
//	See if this is for the SM.
//
	cp = maip->data;
	if ((*(cp+0) != 'S') || (*(cp+1) != 'M') || (*(cp+2) != '\0')) {
		IB_EXIT(__func__, 0);
		return(VSTATUS_OK);
	}

//
//	See if it is for my dev/port combination.
//
	amod = ntoh32(maip->base.amod);
	dev = (amod >> 8) & 0xff;
	port = amod & 0xff;
	if ((dev != sm_config.hca) || (port != sm_config.port)) {
		IB_EXIT(__func__, amod);
		return(VSTATUS_OK);
	}

//
//	This is from another instance.  Let's see what we need to do.
//
	switch (maip->base.aid) {
	case SM_CONTROL_SHUTDOWN:
		(void)sm_control_shutdown(maip);
		break;
	case SM_CONTROL_STANDBY:
		(void)sm_control_standby(maip);
		break;
	case SM_CONTROL_RESTART:
		(void)sm_control_restart(maip);
		break;
	case SM_CONTROL_HEARTBEAT:
		(void)sm_control_heartbeat(maip);
		break;
	case SM_CONTROL_REGISTER:
		(void)sm_control_register(maip);
		break;
	case SM_CONTROL_RECONFIG:
		(void)sm_control_reconfig();
		break;
	default:
		break;
	}

	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}

Status_t
sm_control_shutdown(Mai_t *maip)
{
	IB_ENTER(__func__, 0, 0, 0, 0);

#ifdef __LINUX__	
	// Under Linux just completely bail. Let the OS clean up memory and threads. Results in clean
	// and expedient shutdown on large clusters.
    IB_LOG_INFINI_INFO0("FM exiting.");
	// Let file I/O that needs to complete finish prior to exiting to prevent issues when FM runs again
	// next (HSM Only)
	vs_wrlock(&linux_shutdown_lock);
	exit(0);
#else

    /* turn off isSm bit in portInfo and kill umadt reader */
    IB_LOG_INFINI_INFO0("turning off isSm bit in portInfo");
    sm_clearIsSM();

    sm_shutdown();
    /* Signal the main thread that it is time to die. */
    sm_control_cmd = SM_CONTROL_SHUTDOWN;

	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
#endif
}

Status_t
sm_control_standby(Mai_t *maip)
{

	IB_ENTER(__func__, 0, 0, 0, 0);
	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}

Status_t
sm_control_restart(Mai_t *maip)
{

	IB_ENTER(__func__, 0, 0, 0, 0);
	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}

Status_t
sm_control_heartbeat(Mai_t *maip)
{

	IB_ENTER(__func__, 0, 0, 0, 0);
	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}

Status_t
sm_control_register(Mai_t *maip)
{

	IB_ENTER(__func__, 0, 0, 0, 0);
	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}

Status_t
sm_control_reconfig(void)
{
	IB_ENTER(__func__, 0, 0, 0, 0);

    /* Signal the main thread to re-read and apply new configuration */
    sm_control_cmd = SM_CONTROL_RECONFIG;

	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}

Status_t
sm_control_init()
{
	Status_t	status;
	Filter_t	filter;

	IB_ENTER(__func__, 0, 0, 0, 0);

//
//	Create the Control(*) MAD filters for controlling instances.
//
	SM_Filter_Init(&filter);
	filter.type = MAI_TYPE_INTERNAL;
	filter.active = MAI_ACT_BASE | MAI_ACT_TYPE | MAI_ACT_PORT |
			MAI_ACT_DEV  | MAI_ACT_QP   | MAI_ACT_FMASK;

	filter.value.bversion = MAD_BVERSION;
	filter.value.cversion = MAD_CVERSION;
	filter.value.mclass = MAD_CV_SUBN_LR;
	filter.value.method = MAD_CM_SEND;

	filter.mask.bversion  = 0xff;
	filter.mask.cversion  = 0xff;
	filter.mask.mclass  = 0xff;
	filter.mask.method  = 0xff;

	status = mai_filter_create(fd_async, &filter, VFILTER_SHARE | VFILTER_PURGE);
	if (status != VSTATUS_OK) {
		smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
			"can't create control filter %d", status);
		return(status);
	}

	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}

Status_t
sm_control_notify()
{
	Mai_t		out_mad;
	Status_t	status;

	IB_ENTER(__func__, 0, 0, 0, 0);

//
//	Send a SHUTDOWN message to other instances.
//
	memset((void *)&out_mad, 0, sizeof(Mai_t));
	out_mad.type = MAI_TYPE_INTERNAL;
	out_mad.port = sm_config.port;
	out_mad.dev = sm_config.hca;
	out_mad.active |= MAI_ACT_BASE;
	out_mad.active |= MAI_ACT_TYPE;
	out_mad.active |= MAI_ACT_PORT;
	out_mad.active |= MAI_ACT_DEV;
	out_mad.active |= MAI_ACT_QP;

	out_mad.base.bversion = MAD_BVERSION;
	out_mad.base.cversion = MAD_CVERSION;
	out_mad.base.mclass = MAD_CV_SUBN_LR;
	out_mad.base.method = MAD_CM_SEND;
	out_mad.base.aid = SM_CONTROL_SHUTDOWN;
	out_mad.base.amod = ntoh32((sm_config.hca << 8) | sm_config.port);
	out_mad.base.tid = (uint64_t)sm_threads[SM_THREAD_ASYNC].name;
	out_mad.data[0] = 'S';
	out_mad.data[1] = 'M';
	out_mad.data[2] = '\0';

	INCREMENT_COUNTER(smCounterSmPacketTransmits);
	status = mai_send(fd_async, &out_mad);
	if (status != VSTATUS_OK) {
		smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_OTHER_ERROR, getMyCsmNodeId(), NULL,
			"can't send NOTIFY mad %d", status);
		return(status);
	}

	IB_EXIT(__func__, 0);
	return(VSTATUS_OK);
}
