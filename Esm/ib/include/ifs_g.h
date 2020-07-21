/* BEGIN_ICS_COPYRIGHT2 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

//==============================================================================/
//                                                                          	/
// FILE NAME                                               			/
//      ifs_g.h                                             			/
//                                                                          	/
// DESCRIPTION                                                              	/
//      This file contains global structure and definitions used in the HSM and /
//      the ESM and other tools.	                                 	/
//==============================================================================/

#ifndef _IFS_G_H_
#define _IFS_G_H_

// For dynamic Adaptive Routing configuration for the HSM and ESM
#define AR_FREQUENCY_LOWER 0
#define AR_FREQUENCY_UPPER 7

#define AR_THRESHOLD_LOWER 0
#define AR_THRESHOLD_UPPER 7

typedef struct fm_ar_config {
	uint32_t	enable;
	uint32_t	frequency;
	uint32_t	threshold;
}fm_ar_config_t;

#endif /* _IFS_G_H_ */

