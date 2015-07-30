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

#ifndef _IBA_PUBLIC_ISYSUTIL_H_
#define _IBA_PUBLIC_ISYSUTIL_H_

#include "iba/public/datatypes.h"
#include "iba/public/isysutil_osd.h"

/* Assorted System Utility Functions */

#ifdef __cplusplus
extern "C" {
#endif

#if defined(VXWORKS)
/* obtain the System Node Name.  Any domain name suffix is removed
 * remainder is copied to dest for up to dest_len characters
 * if dest_len is < strlen(node name), dest is not \0 terminated.
 * returns dest
 */
extern char *SystemGetNodeName(char *dest, uint32 dest_len);
extern void SystemSeedRandom(unsigned long entropy);
extern unsigned long SystemGetRandom(void);
/* OS specific header will define 2 functions:
 * boolean SystemIncModuleRefCount(void)
 * void SystemDecModuleRefCount(void)
 */
#endif /* defined(VXWORKS) */

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_ISYSUTIL_H_ */
