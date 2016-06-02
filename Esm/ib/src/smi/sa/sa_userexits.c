/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									     //
// FILE NAME								     //
//    sa_userexits.c							     //
//									     //
// DESCRIPTION								     //
//    This file contains the user exits as outlined in the design doc.	     //
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
#include "ib_status.h"
#include "ib_mad.h"
#include "ib_macros.h"
#include "ib_sa.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_l.h"
#include "sa_l.h"

extern	Lock_t		tid_lock;

//---------------------------------------------------------------------------//

Status_t
pathrecord_userexit(uint8_t *data, uint32_t *records) {

	IB_ENTER("pathrecord_userexit", data, *records, 0, 0);

	IB_EXIT("pathrecord_userexit", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
multipathrecord_userexit(uint8_t *data, uint32_t *records) {

	IB_ENTER("multipathrecord_userexit", data, *records, 0, 0);

	IB_EXIT("multipathrecord_userexit", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
servicerecord_userexit(Mai_t *maip) {

	IB_ENTER("servicerecord_usrexit", maip, 0, 0, 0);

	IB_EXIT("servicerecord_userexit", VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
tracerecord_userexit(uint8_t *data, uint32_t *records) {

	IB_ENTER("tracerecord_userexit", data, *records, 0, 0);

	IB_EXIT("tracerecord_userexit", VSTATUS_OK);
	return(VSTATUS_OK);
}

