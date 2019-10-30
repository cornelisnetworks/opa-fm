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
/*!
 @file    $Source: /cvs/ics/MakeTools/rm_version/rm_version.c,v $
 $Name:  $
 $Revision: 1.5 $
 $Date: 2015/01/22 18:04:00 $
 @brief   Program to remove version strings appended at end of file by prep
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>

extern int ftruncate(int __fd, off_t __length);


/*!
Remove the version string from a .uid or Linux/Cygwin/MIPS a.out file

This scans the last 1024 bytes of the given file for a
version string which starts with "version_marker:"
If the pattern is matched, the file is truncated such that
"version_marker:" and all bytes which follow it are
removed from the file.

THIS SHOULD ONLY BE RUN AGAINST FILES WHICH prep MAY HAVE
APPENDED A VERSION STRING TO.  FOR OTHER FILES WHICH PREP
EDITS WITH mcs OR sed, THIS MAY CORRUPT THE FILE.

The scan for "version_marker" is done by a reverse scan
of the last 1024 bytes of the file, it stops on the first match

@heading Usage
	rm_version [-m version_marker] file ...
@param version_marker - version marker to look for, default is
		"ICS VERSION STRING"
@param file - name of a file which may have a version string
		appended to it
*/

/* the string we are looking for */
#define MAGIC_STRING "ICS VERSION STRING:"
int magiclen;	/* length of MAGIC_STRING */
const char* magic=MAGIC_STRING;

/*!
remove the version string from the specified file

@param filename - name of file to fix

@return 0 on success, 1 on failure
*/
int fix_file(char* filename)
{
	int fd;
	struct stat statbuf;
	int end;	/* offset of last 1024 bytes of file */
	int size;	/* size of file starting at offset end */
#define MAXBUF 1024
	char buffer[MAXBUF];	/* holding area for scan */
	int offset;	/* current offset from end for scan */

	if (stat(filename, &statbuf) == -1
	    || (fd = open(filename, O_RDWR)) == -1)
	{
		fprintf(stderr, "rm_version: Can't open: ");
		perror(filename);
		return(1);
	}
	if (statbuf.st_size < MAXBUF)
	{
		/* small file, limit to file size */
		end = 0;
		size = statbuf.st_size;
	} else {
		end = statbuf.st_size - MAXBUF;
		size = MAXBUF;
	}
	if (lseek(fd, end, SEEK_SET) == -1)
	{
		fprintf(stderr, "rm_version: Can't seek: ");
		perror(filename);
		close(fd);
		return(1);
	}
	/* unfortunately CYGWIN or NT seem to be off on st_size since
	 * you typically can't read size bytes here.  Linux is fine
	 */
	/*if (read(fd, buffer, size) != size)*/
	if (read(fd, buffer, size) == -1)
	{
		fprintf(stderr, "rm_version: Can't read: ");
		perror(filename);
		close(fd);
		return(1);
	}
	/* scan buffer for 'ICS VERSION STRING' */
	for (offset = size-magiclen; offset >= 0; offset--)
	{
		if (strncmp(buffer+offset, magic, magiclen) == 0)
		{
			goto found;
		}
	}
	/* not found */
	close(fd);
	return(0);

found:
	/* found the string at offset */
	if (ftruncate(fd, end+offset) == -1)
	{
		fprintf(stderr, "rm_version: Can't truncate: ");
		perror(filename);
		close(fd);
		return(1);
	}
	close(fd);
	printf("%s: %d bytes removed\n", filename, size-offset);
	return(0);
}

void usage(void)
{
	fprintf(stderr, "Usage: rm_version [-m version_marker] file ...");
	exit(2);
}

int main(int argc, char** argv)
{
	int i;
	int res;
	int c;
	extern char* optarg;
	extern int optind;

	while ((c = getopt(argc, argv, "m:")) != -1)
	{
		switch (c)
		{
			case 'm':	/* version_marker */
				magic = optarg;
				break;
			case '?':
				usage();
		}
	}
	
	if (argc == optind)
	{
		usage();
	}
	magiclen = strlen(magic);
	res = 0;
	for (i=optind; i< argc; i++)
	{
		res |= fix_file(argv[i]);
	}

	return res;
}
