/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_PUBLIC_STATUS_TEXT_H_
#define _IBA_PUBLIC_STATUS_TEXT_H_

#include "iba/ib_status.h"		/* IB-defined status code */

#ifdef __cplusplus
extern "C"
{
#endif	/* __cplusplus */

/* Status values above converted to text for easier printing. */
extern const char* FStatusText[];
extern const char* FCmStatusText[];

/* prefered way to convert FSTATUS to a string */
IBA_API const char* iba_fstatus_msg(FSTATUS status);

/* depricated macro to convert FSTATUS_MSG to a string */
/* FCM_WAIT_OBJECT0 == 64, < 74 allows for 0-9 */
#define FSTATUS_MSG(errcode) \
	(((GET_MODULE(errcode) == IB_MOD_CM) && (GET_MODERR(errcode) < 74)) ? \
		FCmStatusText[GET_MODERR(errcode)]: \
	(((errcode & 0xFF) < FSTATUS_COUNT) ? \
		FStatusText[(errcode) & 0xFF] : "invalid status code"))


#ifdef __cplusplus
}
#endif	/* __cplusplus */

#endif /* _IBA_PUBLIC_STATUS_TEXT_H_ */
