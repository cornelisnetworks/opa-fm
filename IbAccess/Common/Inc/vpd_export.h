/* BEGIN_ICS_COPYRIGHT1 ****************************************

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

** END_ICS_COPYRIGHT1   ****************************************/

/* [ICS VERSION STRING: unknown] */

/* Suppress duplicate loading of this file */
#ifndef _IBA_VPD_EXPORT_H_
#define _IBA_VPD_EXPORT_H_

/* This file defines the API which must be provided by a kernel mode Verbs
 * provider driver.
 */
#if defined(VXWORKS)
#include "iba/vpi_export.h"

#ifdef __cplusplus
extern "C" {
#endif

/* this number changes each time the structure below changes
 * it is used to verify the VPD is the same revision as IbAccess
 */
typedef enum {
	VPD_INTERFACE_VERSION_NONE=0,
	VPD_INTERFACE_VERSION_3=3,
	VPD_INTERFACE_VERSION_4=4,
	VPD_INTERFACE_VERSION_5=5,
	VPD_INTERFACE_VERSION_6=6,
	VPD_INTERFACE_VERSION_7=7,
	VPD_INTERFACE_VERSION_8=8,
	VPD_INTERFACE_VERSION_9=9,
	VPD_INTERFACE_VERSION_10=10,
	VPD_INTERFACE_VERSION_LATEST=10
} VPD_INTERFACE_VERSION;

typedef struct _VPD_KM_INTERFACE {
	EUI64						CaGUID;

	VPI_OPENCA					*OpenCA;
	VPI_QUERYCA					*QueryCA;
	VPI_SETCOMPLETIONHANDLER	*SetCompletionHandler;
	VPI_SETASYNCEVENTHANDLER	*SetAsyncEventHandler;
	VPI_SETCACONTEXT			*SetCAContext;
	VPI_SETCAINTERNAL			*SetCAInternal;
	VPI_MODIFYCA				*ModifyCA;
	VPI_CLOSECA					*CloseCA;
	VPI_ALLOCATEPD2				*AllocatePD;
	VPI_ALLOCATESQPPD			*AllocateSqpPD;
	VPI_FREEPD					*FreePD;
	VPI_ALLOCATERDD				*AllocateRDD;
	VPI_FREERDD					*FreeRDD;
	VPI_CREATEAV				*CreateAV;
	VPI_MODIFYAV				*ModifyAV;
	VPI_QUERYAV					*QueryAV;
	VPI_DESTROYAV				*DestroyAV;
	VPI_CREATEQP				*CreateQP;
	VPI_MODIFYQP				*ModifyQP;
	VPI_SETQPCONTEXT			*SetQPContext;
	VPI_QUERYQP					*QueryQP;
	VPI_DESTROYQP				*DestroyQP;
	VPI_CREATESPECIALQP			*CreateSpecialQP;
	VPI_ATTACHQPTOMULTICASTGROUP2 *AttachQPToMulticastGroup;
	VPI_DETACHQPFROMMULTICASTGROUP2 *DetachQPFromMulticastGroup;
	VPI_CREATECQ				*CreateCQ;
	VPI_RESIZECQ				*ResizeCQ;
	VPI_SETCQCONTEXT			*SetCQContext;
	VPI_SETCQCOMPLETIONHANDLER	*SetCQCompletionHandler;
	VPI_QUERYCQ					*QueryCQ;
	VPI_DESTROYCQ				*DestroyCQ;
	VPI_REGISTERMEMREGION		*RegisterMemRegion;
	VPI_REGISTERPHYSMEMREGION2	*RegisterPhysMemRegion;
	VPI_REGISTERCONTIGPHYSMEMREGION2	*RegisterContigPhysMemRegion;
	VPI_QUERYMEMREGION2			*QueryMemRegion;
	VPI_DEREGISTERMEMREGION		*DeregisterMemRegion;
	VPI_MODIFYMEMREGION			*ModifyMemRegion;
	VPI_MODIFYPHYSMEMREGION2	*ModifyPhysMemRegion;
	VPI_MODIFYCONTIGPHYSMEMREGION2 *ModifyContigPhysMemRegion;
	VPI_REGISTERSHAREDMEMREGION2 *RegisterSharedMemRegion;
	VPI_CREATEMEMWINDOW			*CreateMemWindow;
	VPI_QUERYMEMWINDOW			*QueryMemWindow;
	VPI_POSTMEMWINDOWBIND2		*PostMemWindowBind;
	VPI_DESTROYMEMWINDOW		*DestroyMemWindow;
	VPI_POSTSEND2				*PostSend;
	VPI_POSTRECV2				*PostRecv;
	VPI_POLLCQ					*PollCQ;
	VPI_PEEKCQ					*PeekCQ;
	VPI_REARMCQ					*RearmCQ;
	VPI_REARMNCQ				*RearmNCQ;

	/* Private interfaces for the SMA, PMA and CM */
	VPI_GETSETMAD				*GetSetMad;
	VPI_REGISTERCONNMGR			*RegisterConnMgr;

	/* Private interface to the user-mode VP component */
	VPI_QUERYCA_PRIVATE_INFO	*QueryCAPrivateInfo;	/* Optional */
	VPI_QUERYPD_PRIVATE_INFO	*QueryPDPrivateInfo;	/* Optional */
	VPI_QUERYQP_PRIVATE_INFO	*QueryQPPrivateInfo;	/* Optional */
	VPI_QUERYCQ_PRIVATE_INFO	*QueryCQPrivateInfo;	/* Optional */
	VPI_QUERYAV_PRIVATE_INFO	*QueryAVPrivateInfo;	/* Optional */

	/* other new interfaces */
	VPI_POLLANDREARMCQ2			*PollAndRearmCQ;	/* Optional */

	VPI_QUERYCA2				*QueryCA2;

} VPD_KM_INTERFACE;

#ifdef __cplusplus
};
#endif

#endif /* defined(VXWORKS) */

#endif /* _IBA_VPD_EXPORT_H_ */
