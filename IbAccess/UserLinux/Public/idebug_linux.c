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

#include <syslog.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <stdlib.h>
#include "datatypes.h"
#include "ib_debug.h"

static int use_syslog = 0;
/*!
Very linux specific way to get the pathname when we
don't have access to the arglist.
*/
void
pgmname(char *name, size_t max)
{
	char procname[64];
	char buf[PATH_MAX];
	int len;
	char *p;

	use_syslog = 1;
	sprintf(procname,"/proc/%d/exe", getpid());
	len = readlink(procname, buf, sizeof(buf));
	if (len > 0)
	{
		buf[len] = '\0';
		p = strrchr(buf, '/');
		strncpy(name, p == NULL?buf:p+1, max);
		name[max-1] = 0;
	}
	else
	{
		/* reasonable default? */
		strncpy(name, "ibt", max);
		name[max-1]=0;
	}
}

//static char *default_log_name = "ibt";

void IbLogPrintf(uint32 level, const char* format, ...)
{
	va_list args;

	va_start(args, format);
	if (use_syslog)
	{
		int priority;

		if (level & _DBG_LVL_FATAL)
			priority = LOG_CRIT;
		if (level & _DBG_LVL_ERROR)
			priority = LOG_ERR;
		else if (level & (_DBG_LVL_WARN))
			priority = LOG_WARNING;
		else if (! level || level & (_DBG_LVL_INFO))
			priority = LOG_INFO;
		else
			priority = LOG_DEBUG;

		vsyslog(priority, format, args);
	} else
	{
		vprintf(format, args);
	}
	va_end(args);
}

// Only used for direct calls to DbgOut
void
PrintUDbg(char *Message, ...)
{
	va_list args;

	va_start(args, Message);
	if (use_syslog)
	{
		vsyslog(LOG_DEBUG, Message, args);
	} else
	{
		vprintf(Message, args);
	}
	va_end(args);
}

// only used for direct calls to MsgOut
void
PrintUMsg(char *Message, ...)
{
	va_list args;

	va_start(args, Message);
	if (use_syslog)
	{
		vsyslog(LOG_INFO, Message, args);
	} else
	{
		vprintf(Message, args);
	}
	va_end(args);
}

#include <execinfo.h>
void BackTrace(FILE *file)
{
	void *buffer[100];
	int size;

	if (file && ! use_syslog)
	{
		fprintf(file, "Stack Backtrace:\n");
		fflush(file);
	} else {
		syslog(LOG_INFO, "Stack Backtrace:\n");
	}
	size = backtrace(buffer, 100);
	if (size <= 0 || size > 100) {
		fprintf(stderr, "unable to get backtrace\n");
		return;
	}
	if (file && ! use_syslog)
	{
		backtrace_symbols_fd(buffer, size, fileno(file));
		fprintf(file, "\n");
	} else {
		char **symbols = backtrace_symbols(buffer, size);
		int i;

		for (i=0; i<size; ++i)
			syslog(LOG_INFO, "%s", symbols[i]);
		free(symbols);
	}
}

void DumpStack(void)
{
	BackTrace(stderr);
}
