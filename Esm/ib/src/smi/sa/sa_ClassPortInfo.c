/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									     //
// FILE NAME								     //
//    sa_ClassPortInfo.c						     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA requests for 	     //
//    records of the ClassPortInfo type.				     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_ClassPortInfo							     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//									     //
//===========================================================================//


#include "os_g.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_counters.h"
#include "sm_l.h"
#include "sa_l.h"

extern	STL_CLASS_PORT_INFO	saClassPortInfo;

Status_t
sa_ClassPortInfo(Mai_t *maip, sa_cntxt_t* sa_cntxt) {
	uint16_t				attribOffset;
	union {
		STL_CLASS_PORT_INFO		stl_version;
		IB_CLASS_PORT_INFO		ib_version;
	} myCPI;

	IB_ENTER("sa_ClassPortInfo", maip, 0, 0, 0);

//
//	Check the method.  For ClassPortInfo, you can only do a Get().
//
	if (maip->base.method == SA_CM_GET) {
		INCREMENT_COUNTER(smCounterSaRxGetClassPortInfo);
		if (maip->base.bversion == IB_BASE_VERSION && maip->base.cversion == SA_MAD_CVERSION) {
			memset(&myCPI.ib_version,0,sizeof(IB_CLASS_PORT_INFO));
			myCPI.ib_version.BaseVersion = saClassPortInfo.BaseVersion;
			myCPI.ib_version.ClassVersion = saClassPortInfo.ClassVersion;
			myCPI.ib_version.CapMask =
				STL_CLASS_PORT_CAPMASK_CM2 |
				STL_SA_CAPABILITY_MULTICAST_SUPPORT |
				STL_SA_CAPABILITY_PORTINFO_CAPMASK_MATCH |
				STL_SA_CAPABILITY_PA_SERVICES_SUPPORT;
			myCPI.ib_version.u1.s.CapMask2 =
				STL_SA_CAPABILITY2_QOS_SUPPORT |
				STL_SA_CAPABILITY2_MFTTOP_SUPPORT |
				STL_SA_CAPABILITY2_FULL_PORTINFO |
				STL_SA_CAPABILITY2_EXT_SUPPORT;
			myCPI.ib_version.u1.s.RespTimeValue = saClassPortInfo.u1.s.RespTimeValue;
			myCPI.ib_version.u3.s.RedirectQP = saClassPortInfo.u3.s.RedirectQP;
			myCPI.ib_version.u5.s.TrapHopLimit = saClassPortInfo.u5.s.TrapHopLimit;
			myCPI.ib_version.u5.s.TrapQP = saClassPortInfo.u5.s.TrapQP;
			BSWAP_IB_CLASS_PORT_INFO(&myCPI.ib_version);

			attribOffset = sizeof(IB_CLASS_PORT_INFO) + 
							Calculate_Padding(sizeof(IB_CLASS_PORT_INFO));
			sa_cntxt_data( sa_cntxt, &myCPI.ib_version, attribOffset);
			sa_cntxt->attribLen = attribOffset;
			maip->base.status = MAD_STATUS_OK;
		} else if (maip->base.bversion == STL_BASE_VERSION && maip->base.cversion == STL_SA_CLASS_VERSION) {
			myCPI.stl_version = saClassPortInfo;
			BSWAP_STL_CLASS_PORT_INFO(&myCPI.stl_version);

			attribOffset = sizeof(STL_CLASS_PORT_INFO) + 
							Calculate_Padding(sizeof(STL_CLASS_PORT_INFO));

			sa_cntxt_data( sa_cntxt, &myCPI.stl_version, attribOffset);
			sa_cntxt->attribLen = attribOffset;
			maip->base.status = MAD_STATUS_OK;
		} else {
			IB_LOG_WARN_FMT(__func__, "invalid Base and/or Class Versions: Base %u, Class %u",
				maip->base.bversion, maip->base.cversion);
			maip->base.status = MAD_STATUS_BAD_CLASS;
		}
	} else {
		IB_LOG_WARN_FMT(__func__, "invalid Method: %s (%u)",
			cs_getMethodText(maip->base.method), maip->base.method);
		maip->base.status = MAD_STATUS_BAD_METHOD;
	}
	(void)sa_send_reply(maip, sa_cntxt);

	IB_EXIT("sa_ClassPortInfo", VSTATUS_OK);
	return(VSTATUS_OK);
}
