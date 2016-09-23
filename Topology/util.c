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

#include "topology.h"
#include "topology_internal.h"

extern int	vsnprintf (char *, size_t, const char *, va_list);

#define ESC "\033["
#define LINE_LEN 80

const char *g_Top_cmdname = "Topology";	// our default
Top_FreeCallbacks g_Top_FreeCallbacks = {};

/* output a progress message.  The message should not include a newline
 * This is intended to allow for overlapping progress messages
 * those without a newline will be written by the next message
 * if a newline is desired after the message pass newline=TRUE
 */
void ProgressPrint(boolean newline, const char *format, ...)
{
	va_list args;
	static char buffer[LINE_LEN];
	static int first_progress = 1;	// has 1st progress been output yet

	va_start(args, format);
	(void)vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	if (first_progress) {
		first_progress = 0;
	} else {
		fprintf(stderr, ESC "%dD", LINE_LEN);
	}
	fprintf(stderr, "%-*s", LINE_LEN, buffer);
	if (newline) {
		first_progress = 1;
			fprintf(stderr, "\n");
	}
	fflush(stderr);
}

// truncate a str to fit within LINE_LEN-20 characters
// useful for inclusion of strings and filenames in otherwise
// brief ProgressPrint output
// uses statics, so not thread safe and can't be used multiple times in one
// function call's arguments
const char* Top_truncate_str(const char *name)
{
	int len = strlen(name);
	if (len <= (LINE_LEN-20)) {
		return name;
	} else {
		static char buf[LINE_LEN-19];
	
		(void)snprintf(buf, sizeof buf, "...%s", name+len-(LINE_LEN-23));
		return buf;
	}
}

// set command name to be used for error messages
void Top_setcmdname(const char *name)
{
	g_Top_cmdname = name;
}

void Top_setFreeCallbacks(Top_FreeCallbacks *callbacks)
{
	g_Top_FreeCallbacks = *callbacks;
}
