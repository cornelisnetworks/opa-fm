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

 * ** END_ICS_COPYRIGHT5   ****************************************/

//=======================================================================
//									/
// FILE NAME								/
//    vs_utility.c							/
//									/
// DESCRIPTION								/
//    This is the CS routines which don't fit a common category.	/
//									/
// DATA STRUCTURES							/
//    None								/
//									/
// FUNCTIONS								/
//    vs_enter		log an entry into a routine			/
//    vs_exit		log an exit from a routine			/
//    vs_time_get	get the current epoch time			/
//									/
// DEPENDENCIES								/
//    ib_mad.h								/
//    ib_status.h							/
//									/
//									/
// HISTORY								/
//									/
//    NAME	DATE  REMARKS						/
//     jsy  02/14/01  Initial creation of file.				/
//     jsy  04/04/01  Added vs_getpid function.				/
//     trp  08/20/01  Removed vs_getpid (vs_thread_name) is one to use	/
//=======================================================================
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/resource.h>
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_g.h"
#include "cs_log.h"
#include <features.h>
#ifdef LOCAL_MOD_ID
#undef LOCAL_MOD_ID
#endif
#define LOCAL_MOD_ID VIEO_CS_MOD_ID

//----------------------------------------------------------------------------//

Status_t
vs_time_get(uint64_t *address)
{
	uint64_t	usecs;
	struct	timespec	now;

        if (address == 0)
          {
            return(VSTATUS_ILLPARM);
          }
	clock_gettime(CLOCK_MONOTONIC, &now);
	usecs = now.tv_sec;
	usecs *= 1000000;
	usecs += (now.tv_nsec/1000);
	/*round up anything greater than 0.5 usecs*/
	if (now.tv_nsec % 1000 > 500)
		usecs++;

	*address = usecs;

	return(VSTATUS_OK);
}

/*
 * vs_stdtime_get - get current time as a time_t, seconds since 1970
 */
Status_t
vs_stdtime_get(time_t *address)
{
	if (-1 == time(address))
		return VSTATUS_BAD;
	return(VSTATUS_OK);
}

// Translate CoreDumpLimit string to a rlimit value
// returns 0 on success, -1 on error
// value argument is optional, if NULL just checks str is valid CoreDumpLimit
// str can be a byte count or "unlimited"
int vs_getCoreDumpLimit(const char* str, uint64_t* value)
{
	uint64_t rlimit;

	if (!str)
		return -1;

	if (! strlen(str))
		return -1;

	if (strcasecmp(str, "unlimited") == 0) {
		rlimit = RLIM_INFINITY;
	} else {
		if (FSUCCESS != StringToUint64Bytes(&rlimit, str, NULL, 0, TRUE))
			return -1;
		// treat small values as Megabytes
		if (rlimit < 8192)
			rlimit = rlimit * (1024*1024);
	}

	if (value)
		*value = rlimit;
	return 0;
}

/*
 *  vs_init_coredump_settings - setup coredump config for this process
 *  note this changes the current directory
 */
void vs_init_coredump_settings(const char* mgr, const char* limit, const char *dir)
{
	struct rlimit rlimit;
	char buf[140];
	struct stat statbuf;
   
	rlimit.rlim_max = RLIM_SAVED_MAX;
	if (0 != vs_getCoreDumpLimit(limit, &rlimit.rlim_cur)) {
		// parser should have already validated, but just to be safe
		snprintf(buf, sizeof(buf), "%s: Disabling CoreDumps: Invalid CoreDumpLimit: '%s'",
				mgr, limit);
		vs_log_output_message(buf, FALSE);
		rlimit.rlim_cur = 0; limit = "0";
	} else if (rlimit.rlim_cur == 0) {
		snprintf(buf, sizeof(buf), "%s: Disabling CoreDumps: CoreDumpLimit: %s", mgr, limit);
		vs_log_output_message(buf, FALSE);
	} else if (strlen(dir) == 0 || 0 == strcmp(dir, "/dev/null")) {
		snprintf(buf, sizeof(buf), "%s: Disabling CoreDumps: No CoreDumpDir", mgr);
		vs_log_output_message(buf, FALSE);
		rlimit.rlim_cur = 0; limit = "0";
	} else {
		if (-1 == stat(dir, &statbuf)) {
			if (mkdir(dir, 0644) != 0) {
				snprintf(buf, sizeof(buf), "%s: Disabling CoreDumps: Unable to create CoreDumpDir: '%s' %m", mgr, dir);
				vs_log_output_message(buf, FALSE);
				rlimit.rlim_cur = 0; limit = "0";
			}
		} else if (! S_ISDIR(statbuf.st_mode)) {
			snprintf(buf, sizeof(buf), "%s: Disabling CoreDumps: CoreDumpDir is not a directory: '%s'", mgr, dir);
			vs_log_output_message(buf, FALSE);
			rlimit.rlim_cur = 0; limit = "0";
		}
		if (0 != chdir(dir)) {
			snprintf(buf, sizeof(buf), "%s: Disabling CoreDumps: Unable to cd CoreDumpDir: '%s' %m", mgr, dir);
			vs_log_output_message(buf, FALSE);
			rlimit.rlim_cur = 0; limit = "0";
		}
	}
	if (setrlimit(RLIMIT_CORE, &rlimit) != 0) {
		snprintf(buf, sizeof(buf), "%s: Unable to change CoreDumpLimit to '%s' %m", mgr, limit);
		vs_log_output_message(buf, FALSE);
	} else if (rlimit.rlim_cur) {
		snprintf(buf, sizeof(buf), "%s: Enabling CoreDumps to %s up to %s bytes", mgr, dir, limit);
		vs_log_output_message(buf, FALSE);
	}
}
