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
//    sm_mkey.c								     //
//									     //
// DESCRIPTION								     //
//    This file contains the MKey manipulation routines.  These routines     //
//    implement the lease period and mkey checking.			     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sm_mkey_check			check an MKey against our PortInfo   //
//    mkey_lease_decrement		decrement the lease period one tick  //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//    ib_const.h							     //
//									     //
//									     //
//===========================================================================//

#include "os_g.h"
#include "ib_types.h"
#include "ib_mad.h"
#include "ib_sa.h"
#include "ib_status.h"
#include "cs_g.h"
#include "sm_l.h"


// maip is a SMA SMInfo packet
Status_t
sm_mkey_check(Mai_t *maip, uint64_t *mkey) {
	uint64_t	request_mkey=0;
	Status_t	status=VSTATUS_OK;

	IB_ENTER(__func__, maip, mkey, 0, 0);

	/* we only care about MKey, so can use LR or DR byte swap */
	/* LR is slightly faster, so use it */
	/* FIXME: perhaps caller can pass in mkey from their byteswapped header */
	/* and avoid need for this call here */
    BSWAPCOPY_STL_MKEY(STL_GET_MAI_KEY(maip), &request_mkey);
    *mkey = request_mkey;
	
    /*
     *	C14-5 & C14-66 state that if PortInfo:M_Key == 0, then no check will be performed.
     */
    if (sm_config.mkey == 0ull) {
		IB_EXIT(__func__, VSTATUS_OK);
		return(VSTATUS_OK);
	}

    /*
     *	If the supplied M_Key equals our M_Key, then we are done.
     */
	if (request_mkey == sm_config.mkey) {
		IB_EXIT(__func__, VSTATUS_OK);
		return(VSTATUS_OK);
	}

    /*
     *	At this point, we have a non-zero mkey.  So in this case, we have to
     *	implement C14-16.  Note that in table 128, cases 2 and 3 are identical.
     */
    switch ((uint32_t)sm_mkey_protect_level) {
    case 0:
        if (maip->base.method == MAD_CM_GET) {
            status = VSTATUS_OK;
        } else {
            return(VSTATUS_MISMATCH);
        }
        break;
    case 1:
        if (maip->base.method == MAD_CM_GET) {
            return(VSTATUS_CONDITIONAL);
        } else {
            return(VSTATUS_MISMATCH);
        }
        break;
    default:  /* case 2 and 3 are the same; fail all if mkey mismatch */
        return(VSTATUS_MISMATCH);
        break;
    }

	IB_EXIT(__func__, status);
	return(status);
}
