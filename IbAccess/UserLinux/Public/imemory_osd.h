/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_PUBLIC_IMEMORY_OSD_H_
#define _IBA_PUBLIC_IMEMORY_OSD_H_

#include "iba/public/datatypes.h"
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define MEM_TRACK_NO_FTR

/*
 * OS Defined PAGE_SIZE.
 */
#define MemoryGetOsPageSize()		getpagesize()

#ifdef __cplusplus
extern "C" {
#endif

#ifdef IB_STACK_IBACCESS
FSTATUS MemoryLockPrepare(uintn buf_org, uintn Length);
FSTATUS MemoryLockUnprepare(uintn buf_org, uintn Length);

/* This provides to the User Memory module the memory lock strategy which Uvca
 * obtained via Vka from the kernel Memory module
 */
FSTATUS MemorySetLockStrategy(uint32 strategy);

/* Tell UVca is MemoryLockUnprepare is necessary for the configured locking
 * strategy.  If not, UVCA has the option of optimizing its operation to
 * avoid the MemoryLockUnprepare.
 * However, if unnecessary, MemoryLockUnprepare should be a noop and should
 * not fail (hence allowing UVCA to choose not to optimize it out).
 */
boolean MemoryNeedLockUnprepare(void);

/* Initialization routines which libibt will call to initialize user space
 * memory locking component
 */
FSTATUS MemoryLockPrepareInit(void);
void MemoryLockPrepareCleanup(void);
#endif /* IB_STACK_IBACCESS */

#ifdef __cplusplus
};
#endif

#endif /* _IBA_PUBLIC_IMEMORY_OSD_H_ */
