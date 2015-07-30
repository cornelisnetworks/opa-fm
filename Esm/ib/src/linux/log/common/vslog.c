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

 * ** END_ICS_COPYRIGHT5   ****************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include "sys/file.h"
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <syslog.h>
#include <sys/stat.h>

#include <limits.h>

#include "ib_types.h"
#include "ib_status.h"
#include "cs_g.h"
#include "cs_log.h"
#include "vslog_l.h"


uint32_t		log_to_console = 0;

int				logMode = 0;
uint32_t		logLevel=1;	// just used for show_masks option
FILE            *log_file = NULL;
int 			output_fd = -1;

#define MAX_SYSLOG_NAME 64
char vs_log_syslog_name[MAX_SYSLOG_NAME + 1] = "opafm";
uint32_t        syslog_facility = LOG_LOCAL6;

int             vs_log_initialized;

#define	min(a,b) (((a)<(b))?(a):(b))

void vs_log_set_log_mode(int mode)
{
	logMode = mode;
}

int 
vs_log_get_syslog_level(uint32_t sev)
{
	switch(sev) {
	 case VS_LOG_NONE:			return LOG_INFO;
	 case VS_LOG_FATAL:			return LOG_CRIT;
	 case VS_LOG_CSM_ERROR:		return LOG_ERR;
	 case VS_LOG_CSM_WARN:		return LOG_WARNING;
	 case VS_LOG_CSM_NOTICE:	return LOG_NOTICE;
	 case VS_LOG_CSM_INFO:		return LOG_INFO;
	 case VS_LOG_ERROR:			return LOG_ERR;
	 case VS_LOG_WARN:			return LOG_WARNING;
	 case VS_LOG_NOTICE:		return LOG_NOTICE;
	 case VS_LOG_INFINI_INFO:	return LOG_INFO;
	 case VS_LOG_INFO:			return LOG_DEBUG;
	 case VS_LOG_VERBOSE:		return LOG_DEBUG;
	 case VS_LOG_DATA:			return LOG_DEBUG;
	 case VS_LOG_DEBUG1:		return LOG_DEBUG;
	 case VS_LOG_DEBUG2:		return LOG_DEBUG;
	 case VS_LOG_DEBUG3:		return LOG_DEBUG;
	 case VS_LOG_DEBUG4:		return LOG_DEBUG;
	 case VS_LOG_ENTER:			return LOG_DEBUG;
	 case VS_LOG_ARGS:			return LOG_DEBUG;
	 case VS_LOG_EXIT:			return LOG_DEBUG;
	 default:					return LOG_DEBUG;
	}
}

// avoid conflict with vssappl.c fprintf version used in tests
#ifdef LINUX_USR_REL
void
vs_log_output(uint32_t sev, /* severity */
		uint32_t modid,	/* optional Module id */
		const char *function, /* optional function name */
		const char *vf,	/* optional vFabric name */
		const char *format, ...
		)
{
	char buffer[1024];
	va_list args;
	FILE *f = NULL;
	char vfstr[128];

	va_start(args, format);
	(void) vsnprintf (buffer, sizeof(buffer), format, args);
	va_end (args);
	buffer[sizeof(buffer)-1] = '\0';

	if (vf)
		sprintf(vfstr, "[VF:%s] ", vf);
	else
		*vfstr='\0';

	if(log_to_console){
		f = stdout;
	} else if(output_fd != -1) {
		f = log_file;
	}

	if (f) {
		struct tm *locTime;
		char		strTime[28];
		time_t		theCalTime=0;
		uint32_t	pid;
		size_t		lt=0;

		time(&theCalTime);
		locTime = localtime(&theCalTime);
		if (locTime) {
			lt = strftime(strTime,
				sizeof(strTime),
				"%a %b %d %H:%M:%S %Y",
				locTime);
		}
		if (lt==0) {
			strncpy(strTime,"(unknown)", sizeof(strTime));
		}

		(void)vs_log_thread_pid(&pid);

		fprintf(f, "%s: %s(%u): %s[%s]: %s%s%s%s%s\n",
				strTime, vs_log_syslog_name, pid,
				cs_log_get_sev_name(sev), vs_thread_name_str(),
			   	cs_log_get_module_prefix(modid), vfstr,
			   	function?function:"", function?": ":"",
				buffer);
		if (! sev || (sev & NONDEBUG_LOG_MASK))
			fflush(f);
	} else {
		syslog(vs_log_get_syslog_level(sev), "%s[%s]: %s%s%s%s%s", 
				cs_log_get_sev_name(sev), vs_thread_name_str(),
			   	cs_log_get_module_prefix(modid), vfstr,
			   	function?function:"", function?": ":"",
				buffer);
	}
}
#endif

/* Outputs a messages to the syslog regardless of settings */
/* Also outputs to LogFile if configured */
void
vs_log_output_message(char *msg, int show_masks)
{
	if (show_masks) {
		uint32_t log_masks[VIEO_LAST_MOD_ID+1];
		uint32_t modid;

		cs_log_set_log_masks(logLevel, logMode, log_masks);
		vs_log_output(VS_LOG_NONE, VIEO_NONE_MOD_ID,NULL,NULL,
					"%s LogLevel: %u LogMode: %u", msg, logLevel, logMode);
		for (modid=0; modid <= VIEO_LAST_MOD_ID; ++modid) {
			if ( modid == VIEO_NONE_MOD_ID)
				continue;
			if (log_masks[modid] != cs_log_masks[modid])
				vs_log_output(VS_LOG_NONE, VIEO_NONE_MOD_ID,NULL,NULL,
						"%s %s_LogMask: 0x%x", msg, cs_log_get_module_name(modid), cs_log_masks[modid]);
		}
	} else {
		vs_log_output(VS_LOG_NONE, VIEO_NONE_MOD_ID,NULL,NULL, "%s", msg);
	}
}

void
vs_log_output_memory(uint32_t sev, /* severity */
		uint32_t modid,	/* optional Module id */
		const char *function, /* optional function name */
		const char *vf,	/* optional vFabric name */
		const char *prefix,
		const void* addr,
		uint32_t len
		)
{
	char hex_buffer[80];
	unsigned hex_offset = 0;
	char char_buffer[80];
	unsigned char_offset = 0;
	unsigned offset;
	const uint8_t *p = (uint8_t*)addr;

	if (! len) {
		vs_log_output(sev, modid, function, vf, "%s: length is 0", prefix);
		return;
	}

	for (offset=0; offset < len; offset++ ) {
		if ((offset & 15) == 0 && offset != 0 ) {
			vs_log_output(sev, modid, function, vf, "%s: 0x%4.4x%s %s", prefix, offset-16, hex_buffer, char_buffer);
			hex_offset = char_offset = 0;
		}
		hex_offset += sprintf(&hex_buffer[hex_offset], " %2.2x", p[offset]);
		if (p[offset] >= ' ' && p[offset] <= '~')
			char_offset += sprintf(&char_buffer[char_offset], "%c", p[offset]);
		else
			char_offset += sprintf(&char_buffer[char_offset], ".");
	}
	if (offset & 15) {
		// output residual
		uint32 pad = 16 - (offset & 15);
		hex_offset += sprintf(&hex_buffer[hex_offset], "%*s", pad*3, "");
		vs_log_output(sev, modid, function, vf, "%s: 0x%4.4x%s %s", prefix, offset&~15, hex_buffer, char_buffer);
	}
}

#if 0
// --------------------------------------------------------------------
// vs_log_time_get
// --------------------------------------------------------------------
// Return a 64 bit number of uSecs since some epoch.  This function
// returns a monotonically increasing sequence.
// 
// INPUTS
// loc Where to put the time value
// 
// RETURNS
// VSTATUS_OK
// VSTATUS_ILLPARM
// --------------------------------------------------------------------
Status_t
vs_log_time_get(uint64_t * loc)
{
    struct timeval  tv;

    if (NULL == loc)
      {
	  return VSTATUS_ILLPARM;
      }

    gettimeofday(&tv, NULL);
    *loc = ((uint64_t) (tv.tv_sec) * 1000000L) + (uint64_t) tv.tv_usec;

    return (VSTATUS_OK);
}
#endif


// --------------------------------------------------------------------
// vs_log_thread_pid
// --------------------------------------------------------------------
// Return the PID for the current thread.
// 
// INPUTS
// Pointer to PID
// 
// RETURNS
// VSTATUS_OK
// --------------------------------------------------------------------
Status_t
vs_log_thread_pid(uint32_t * pid)
{

    if (pid == NULL)
      {
	  return (VSTATUS_ILLPARM);
      }

    *pid = (uint32_t) getpid();
    return (VSTATUS_OK);
}

//--------------------------------------------------------------------
// vs_fatal_error
//--------------------------------------------------------------------
//   This function will panic() the process and report through the trace
// system a reason.  The trace log (if being used) will be preserved so
// that perhaps a debugger can be connected to the system to figure out
// what went wrong.
//
// INPUTS
//      string      Pointer to string.  Truncated to 63 characters
//
// RETURNS
//      NEVER RETURNS
//--------------------------------------------------------------------
void
vs_fatal_error (uint8_t * string)
{
  	/* make sure we get an entry to syslog, */
	/* just in case logging not fully initialzed */
	openlog("FATAL:", (LOG_NDELAY | LOG_PID), LOG_USER);
	syslog(LOG_CRIT, "%s", string);
	closelog();

	abort();
}


#if 0
/*
 * vslog_time_cvt
 *    Converts usec to hr:min:sec
 * 
 * INPUTS
 * 
 * RETURNS
 * 
 *   
 */
void
vslog_time_cvt(uint64_t val,
	       uint64_t * hh, uint64_t * mm, uint64_t * ss, uint64_t * us)
{

    uint64_t        t,
                    tmp_us = 1000000ull;

    *us = (val % tmp_us);
    t = (val / (tmp_us));	/* get seconds */
    *ss = (t % (60ull));
    t = (t / 60ull);		/* get minutes */
    *mm = ((t) % (60ull));
    t = ((t) / (60ull));	/* get hrs */
    *hh = (t % (24ull));
}
#endif


/*
 * vs_log_init
 *
 */
Status_t
vs_log_usr_init(void)
{
	if (strlen(vs_log_syslog_name) == 0)
	 	sprintf(vs_log_syslog_name,"opafm");
	//printf("openlog(%s, %x, %u)\n", vs_log_syslog_name, (LOG_CONS | LOG_PID), syslog_facility);
	openlog(vs_log_syslog_name, (LOG_CONS | LOG_PID), syslog_facility);
    vs_log_initialized = 1;
    return VSTATUS_OK;
}

/*
 * vs_log_control
 *    Function used to pass cmds to the logging subsystem. 
 *
 * INPUTS
 *    cmd               Defines the action to carried out in the subsystem
 *    arg1              First arg type               
 *    arg2              Second arg 
 *
 * cmd == VS_LOG_STARTSYSLOG
 *
 *        arg1, arg2 and arg3 is NULL.
 *
 * cmd == VS_LOG_SETFACILITY
 *
 *        arg1 Contains facility
 *        arg2 and arg3 is NULL.
 *
 * cmd == VS_LOG_SETMASK
 *
 *        arg1 Contains a new log_masks array
 *        arg2 log_level
 *        arg3 kernel module filter mask.
 *      
 * RETURNS
 *      VSTATUS_OK
 *      VSTATUS_BAD
 */

Status_t
vs_log_control(int cmd, void *arg1, void *arg2, void *arg3)
{
	Status_t status;
	FILE *old_log_file, *temp_file;
	int old_output_fd, temp_fd;

    status = VSTATUS_BAD;

	switch (cmd)
		 {
		 case VS_LOG_STARTSYSLOG:
		 {
			if (! vs_log_initialized)
				(void) vs_log_usr_init();
			if (vs_log_initialized)
				status = VSTATUS_OK;
			else
				return VSTATUS_BAD;
		 }
		 break;
		 case VS_LOG_SETFACILITY:
		 {
			syslog_facility = (uint32_t) (unint)arg1;
		 }
		 break;
		 case VS_LOG_SETMASK:
		 {

             if (arg1)
                 memcpy(cs_log_masks, (uint32_t*)arg1, sizeof(cs_log_masks));
			 logLevel = (uint32_t) (unint)arg2;
			 log_to_console = (int)(unint)arg3;

			 status = VSTATUS_OK;
		 }
		 break;
		 case  VS_LOG_SETOUTPUTFILE:
		 {
		     old_log_file = log_file;
			 old_output_fd = output_fd;

			 if((arg1 != NULL) && (strlen(arg1) > 0))
			 {
                  
				 temp_fd = open((char*)arg1, 
                                   O_WRONLY | O_CREAT| O_APPEND,
                                   S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
				
				 if(temp_fd == -1)
				 {
					 status = VSTATUS_BAD;
				 }
				 else
				 {
					status = VSTATUS_OK;
					if((temp_file = fdopen(temp_fd, (void *)"a")) == NULL)
					{
						close(temp_fd);
						temp_fd = -1;
						status = VSTATUS_BAD;
					}


				 }
				 if(status != VSTATUS_OK)
  			     {
				   	  syslog(LOG_ERR,"Failed to open output log file: %s\n",(char*)arg1);
					  printf("Failed to open output log file: %s\n",(char*)arg1);

				 } else	
				 {
					FILE *tempf;
				 
					// Remap stdout and stderr to our logfile to catch lower level messages
					tempf = freopen((char*)arg1, (void *)"a", stdout);
					if (!tempf) {
				   	  syslog(LOG_ERR,"Failed to redirect stdout to %s\n",(char*)arg1);
					  printf("Failed to redirect stdout to %s\n",(char*)arg1);
					}
					tempf = freopen((char*)arg1, (void *)"a", stderr);
					if (!tempf) {
				   	  syslog(LOG_ERR,"Failed to redirect stderr to %s\n",(char*)arg1);
					  printf("Failed to redirect stderr to %s\n",(char*)arg1);
					}

					output_fd = temp_fd;

					log_file = temp_file;
					if(old_output_fd != -1)
					{
						if(old_log_file != NULL)
							fclose(log_file);
						log_file = NULL;
						close(old_output_fd);
						old_output_fd = -1;
					}
				 }
			}

		 }
		 break;
		 case VS_LOG_SETSYSLOGNAME:
		 {
			if((arg1 != NULL) && (strlen((char*)arg1) <= MAX_SYSLOG_NAME))
			{
				memset(vs_log_syslog_name,0,sizeof(vs_log_syslog_name));
				sprintf(vs_log_syslog_name,"%s",(char *)arg1);
			}
			else
			{
				return VSTATUS_BAD;
			}
		 }
		 break;
		 default:
		 return VSTATUS_BAD;
		 }
    return status;
}

FILE* vs_log_get_logfile_fd(void)
{
	return log_file;
}

