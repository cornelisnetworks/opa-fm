/* BEGIN_ICS_COPYRIGHT2 ****************************************

Copyright (c) 2015-2018, Intel Corporation

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
#ifndef _PM_L_H_
#define _PM_L_H_

#include <cs_g.h>		/* Global common services functions */
#include <string.h>
#include <ib_mad.h>
#include "iquickmap.h"
#include "pm_counters.h"
#include "fm_xml.h"

// 200 bytes per port is more than our current data structures need by a
// reasonable margin (approx 1k for 24 ports).
#define PM_MAX_DATA_LEN (36*200)

/*
 * PM debug flag 
 * re-routes all verbose messages to InfiniInfo level
 */
void pmDebugOn(void);
void pmDebugOff(void);
uint32_t pmDebugGet(void);

/* debug trace control for embeded PM */
#define _PM_TRACE(l) (l == 0) ? VS_LOG_VERBOSE : VS_LOG_INFINI_INFO

// lease time for FreezeFrame, if idle more than this, discard Freeze
extern int pm_shutdown;
extern Pool_t pm_pool;


#define DEFAULT_INTERVAL_USEC       (2000000)	// 2sec

#define PM_CONN_ATTEMPTS       (10)          //how often to try to open connection
#define PM_CONN_INTERVAL       (10000000)   //interval bettween attempts

extern void PmEngineStart(void);
extern void PmEngineStop(void);
extern boolean PmEngineRunning(void);

extern void pm_update_dyn_config(PMXmlConfig_t *new_pm_config);

#endif
