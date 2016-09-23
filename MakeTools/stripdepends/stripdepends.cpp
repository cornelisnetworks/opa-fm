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

// This is an optimized program which edits the input file to accomplish
// the following set of edits
// sed -e '{ :start
//			s|[^ 	]*usr/include[^ 	]*||
//			s|[^ 	]*usr/lib[^ 	]*||
//			s|[^ 	]*/cygwin/usr[^ 	]*||
//			s|[^ 	]*/Tornado[^ 	]*||
//			t start
//		}
//		/^[ 	]*$/d
//		/^[ 	]*\\$/d' < $1 > $tmp
//mv $tmp $1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include <ctype.h>
#include <errno.h>

#define	BUFSIZE		256
#define SBUFSIZE	64

#define	TRUE		1
#define	FALSE		0

#define BACKSLASH	'\\'
#define NL		'\n'

#define SYSTEM_STRING1	"usr/include"
#define SYSTEM_STRING2	"usr/lib"
#define CYGWIN_STRING	"/cygwin/usr"
#define TORNADO_STRING	"/Tornado"

#define FIND_BAD_STRING(p, buf)		\
{					\
	if (((p) = strstr((buf), SYSTEM_STRING1)) == NULL)	\
	   if (((p) = strstr((buf), SYSTEM_STRING2)) == NULL)	\
	      if (((p) = strstr((buf), CYGWIN_STRING)) == NULL)	\
	         (p) = strstr((buf), TORNADO_STRING);		\
}

int compress_buf(char *buf)
{
	char *p = buf;

	while (isspace(*p))
		p++;
	if ((*p == NL) ||
	    ((*p == BACKSLASH) && (*(p+1) == NL)))
		return(0);
	else
		return(1);
}

int main(int argc, char *argv[])
{

	FILE *fp_in, *fp_out;
	char inbuf[BUFSIZE];
	char outbuf[BUFSIZE];
	char *ip, *op;
	char infname[BUFSIZE];
	char outfname[BUFSIZE];
	char *outbase;
	int move_len;

	if (argc != 2)
	{
		fprintf(stderr, "Usage: stripdepends filename\n");
		exit(1);
	}

	strncpy(infname, argv[1], BUFSIZE-1);
	infname[BUFSIZE-1] = 0;

	if ((fp_in = fopen(argv[1], "r")) == NULL)
	{
		fprintf(stderr, "Error opening file <%s> for input: %s\n",
			argv[1], strerror(errno));
		exit(1);
	}

	outbase = basename(argv[1]);
	snprintf(outfname, sizeof outfname, ".%s", outbase);
	if ((fp_out = fopen(outfname, "w")) == NULL)
	{
		fprintf(stderr, "Error opening file <%s> for output: %s\n",
			outfname, strerror(errno));
		exit(1);
	}

	while (fgets(inbuf, BUFSIZE, fp_in) != NULL)
	{
		snprintf(outbuf, sizeof outbuf, "%s", inbuf);
		FIND_BAD_STRING(ip, outbuf);
		while (ip)
		{
			op = ip;
			/* put ip at beginning, op at end */
			while (!isspace(*ip))
				ip--;
			while (!isspace(*op))
				op++;
			/* move op to ip */
			move_len = strlen(op);
			memmove(ip, op, move_len + 1); /* add 1 to include NUL */
			FIND_BAD_STRING(ip, outbuf);
		}
		if (compress_buf(outbuf))
			fprintf(fp_out, "%s", outbuf);
	}


	fclose(fp_in);
	fclose(fp_out);
	if (rename(outfname, infname) < 0)
	{
		fprintf(stderr, "Error renaming file %s to %s: %s\n",
			outfname, infname, strerror(errno));
		exit(1);
	}

	exit(0);
}
