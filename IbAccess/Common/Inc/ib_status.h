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

#ifndef _IBA_IB_STATUS_H_
#define _IBA_IB_STATUS_H_

#include "iba/public/datatypes.h"

/*
 * The following values defines different IB software stack modules.
 * These are the values that would be found in bits 16 - 23
 */

#define IB_MOD_VPI		0x01	/* Verbs provider Interface */
#define IB_MOD_VCA		0x02	/* Verbs Consumer Agent */
#define IB_MOD_VKA		0x03	/* Verbs Kernel Agent */
#define IB_MOD_GSA		0x04	/* General Services Agent */
#define IB_MOD_SMA		0x05	/* Subnet Management Agent */
#define IB_MOD_TSL		0x06	/* IB Transport Services */
#define IB_MOD_SM		0x07	/* Subnet Manager */
#define IB_MOD_CM		0x08	/* Connection Manager */

/*
 * The low 8 bits define generic codes defined in the datatypes.h
 * file. The top 24bits are application specific. For IB error and status 
 * types we use the low 8 bits of this 24 bits for module identification. we
 * use the values defined above. The low 16 bits are module specific 
 * error codes that the module can choose to implement in their own way 
 * as applicable.
 *
 *   31               16        8        0
 *     +--------+--------+--------+--------+
 *     | module err code | mod #  | fstatus|
 *     +--------+--------+--------+--------+
 */

#define SET_FSTATUS(fstatus, mod, mstatus)		\
		((fstatus) & 0x000000FF) |				\
		(((mod) << 8) & 0x0000FF00)	|			\
		(((mstatus) << 16) & 0xFFFF0000)  		

#define GET_STATUS(fstatus)						\
		(fstatus & 0x000000FF)

#define GET_MODULE(fstatus)						\
		((fstatus & 0x0000FF00) >> 8)

#define GET_MODERR(fstatus)						\
		((fstatus & 0xFFFF0000) >> 16)
	
#endif /* _IBA_IB_STATUS_H_ */
