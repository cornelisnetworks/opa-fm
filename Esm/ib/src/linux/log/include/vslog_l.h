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

 * ** END_ICS_COPYRIGHT2   ****************************************/

#ifndef __VS_LOGL__
#define __VS_LOGL__

#define MASK_UPDATE_INTERVAL   (200)

#define MAX_FILTER_PID         (32)

#define DEFAULT_VALUE (-1)       /* default value use to initialize variables */

#if defined(__LINUX__) 

#include <unistd.h>
#define MOD64(x,y) ((x)%(y))
#define PDEBUG1 printf



#if 0
#define USR_LOG_DEBUG
#endif

#endif

#define MAX_MOD_FPID    (8)
#define MAX_MODULE_NAMES (32)


typedef struct _assoc { 
    uint32_t        modid;
    uint8_t         name[6];
   /* True, to filter out process ID traces associated with ID */
    int             filter_pid; 
  /* pids associated with module */ 
  
  uint32_t        pids[MAX_MOD_FPID];
  
  /* number of valid pids in pids array */
  int             pid_cnt; 

} VslogModNameId_t;


#define MAX_MOD_EXFPID    (16)

typedef struct _assoc_ex { 
    uint32_t        modid;
    uint8_t         name[6+1];
   /* True, to filter out process ID traces associated with ID */
  int             filter_pid; 
  /* pids associated with module */ 
  
  uint32_t        pids[MAX_MOD_EXFPID];
  
  /* number of valid pids in pids array */
  int             pid_cnt; 

} VslogExModNameId_t;


#if 0
//--------------------------------------------------------------------
// vs_log_time_get
//--------------------------------------------------------------------
//   Return a 64 bit number of uSecs since some epoch.  This function
// returns a monotonically increasing sequence.
//
// INPUTS
//      loc         Where to put the time value
//
// RETURNS
//      VSTATUS_OK
//      VSTATUS_ILLPARM
//--------------------------------------------------------------------
Status_t
vs_log_time_get (uint64_t * loc);
#endif


//--------------------------------------------------------------------
// vs_log_thread_pid
//--------------------------------------------------------------------
//   Return the current thread's PID.
//
// INPUTS
//      tag      Pointer to pid var
//
// RETURNS
//      VSTATUS_OK
//--------------------------------------------------------------------
Status_t
vs_log_thread_pid (uint32_t * tag);


#if 0
/*
 * vslog_time_cvt
 *    Converts us tho hr:min:sec
 * 
 * INPUTS
 *
 * RETURNS
 *
 *   
 */
void vslog_time_cvt(uint64_t val,
			uint64_t *hr, 
			uint64_t *min, 
			uint64_t *sec,
			uint64_t *us);
#endif

#endif /* _VS_LOGL_ */
