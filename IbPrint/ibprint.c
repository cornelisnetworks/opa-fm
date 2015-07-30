/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <iba/ibt.h>
#include <stdarg.h>
#include "ibprint.h"
#ifdef LINUX
#include <syslog.h>
#endif

// TBD: now universally using 0x---:---- for GIDs
// saquery and used to use 0x---:0x--- in a few places but
// mostly used 0x----:-----

void PrintDestInitNone(PrintDest_t *dest)
{
	dest->type = PL_NONE;
}

void PrintDestInitFile(PrintDest_t *dest, FILE *file)
{
	dest->type = PL_FILE;
	dest->u.file = file;
}

void PrintDestInitBuffer(PrintDest_t *dest, char *buffer, uint16 length)
{
	dest->type = PL_BUFFER;
	dest->u.buf.length = length;
	dest->u.buf.offset = 0;
	dest->u.buf.buffer = buffer;
}

void PrintDestInitCallback(PrintDest_t *dest, PrintCallbackFunc_t func, void *context)
{
	dest->type = PL_CALLBACK;
	dest->u.callback.func = func;
	dest->u.callback.context = context;
}

#ifdef LINUX
void PrintDestInitSyslog(PrintDest_t *dest, int priority)
{
	dest->type = PL_SYSLOG;
	dest->u.priority = priority;
}
#endif

#ifdef VXWORKS
void PrintDestInitLog(PrintDest_t *dest)
{
	dest->type = PL_LOG;
	// TBD
}
#endif

// TBD - for logging should \n be output?
// all calls output an entire line, including the \n
// TBD - add the \n here, that way callbacks for syslog will not include \n
// Note - syslog will only add if needed, so might be ok, what about vxworks?
void PrintFunc(PrintDest_t *dest, const char *format, ...)
{
	va_list args;

	va_start(args, format);
	switch (dest->type) {
	case PL_NONE:
		break;
	case PL_FILE:
		(void)vfprintf(dest->u.file, format, args);
		break;
	case PL_BUFFER:
		{
		uint16 avail = dest->u.buf.length - dest->u.buf.offset;
		if (avail)
			dest->u.buf.offset += vsnprintf(&dest->u.buf.buffer[dest->u.buf.offset],
					   avail, format, args);
		}
		break;
	case PL_CALLBACK:
		{
		char buf[140];
		vsnprintf(buf, sizeof(buf), format, args);
		(dest->u.callback.func)(dest->u.callback.context, buf);
		}
		break;
#ifdef LINUX
	case PL_SYSLOG:
		(void)vsyslog(dest->u.priority, format, args);
		break;
#endif
#ifdef VXWORKS
	case PL_LOG:
		// TBD
		break;
#endif
	}
	va_end(args);
}
