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

// This file parses the opafm.info file
#ifndef __VXWORKS__
#include "cs_info_file.h"

#define INFO_FILE "/etc/sysconfig/opa/opafm.info"
static uint8_t		buffer [256];			// cleaned up read_buffer
uint8_t		hfm_install_dir [256] = "/usr/lib/opa-fm";	// base directory where everything is

static Status_t	read_line		(FILE *fd_config);


// This opens, reads and parses the info file
// and updates the hfm_install_dir global
Status_t read_info_file (void) {
	Status_t	status;
	FILE		*fd_config;			// file handle for reading the config file
	uint8_t		*equals;

	fd_config = fopen (INFO_FILE, "r");
	if (fd_config == NULL) {
		//log_error ("Could not open the info file ('%s') for reading\n", INFO_FILE);
		printf ("Could not open the info file ('%s') for reading\n", INFO_FILE);
		return (VSTATUS_BAD);
	}
	
	status = VSTATUS_OK;
	while (read_line (fd_config) == VSTATUS_OK) {
		if (strncasecmp ((void *)buffer, "IFS_FM_BASE", 13) == 0) {
			equals = (uint8_t *)index ((void *)buffer, '=');
			if (equals != NULL) {
				strcpy ((void *)hfm_install_dir, (void *)equals + 1);
			}
		}
	}

	(void) fclose (fd_config);

	return (status);
}

//------------------------------------------------------------------------------//
//										//
// Routine      : read_line							//
//										//
// Description  : Read a line of input, ignoring blank lines and lines that	//
//                start with a pound sign (#).					//
// Input        : None								//
// Output       : Status of the startup process					//
//                  0 - a line of content was read				//
//                  1 - found EOF						//
//										//
//------------------------------------------------------------------------------//

static Status_t
read_line (FILE *fd_config) {
	uint8_t		*slash;			// start of line comments
	uint8_t		*start;			// start of read buffer
	uint8_t		*stop;			// end of read buffer
	Status_t	status;			// return code status
	uint8_t		read_buffer [256];		// a buffer for reading the file a line-at-a-time
	
	while (TRUE) {
		// clean memory is good memory
		memset (read_buffer, 0, 256);
		memset (buffer,      0, 256);
		
		// NULL means we read past EOF
		if (fgets ((void *)read_buffer, 255, fd_config) == NULL) {
			status = VSTATUS_BAD;
			break;
		}
		//line_number++;
		
		start = read_buffer;
		stop  = read_buffer + strlen ((void *)read_buffer);
		
		// terminate after line comments (//)
		slash = (uint8_t *)strstr ((void *)start, "//");
		if (slash != NULL) {
			*slash = '\0';
			stop = read_buffer + strlen ((void *)read_buffer);
		}
		
		// remove leading whitespace
		while (isspace (*start)) {
			start++;
		}
		
		// ignore lines that start with a pound sign (#)
		if (*start == '#') {
			continue;
		}
		
		// remove trailing whitespace
		while (stop > start && isspace (*--stop)) {
			*stop = '\0';
		}
		
		// ignore blank lines
		if (*start == '\0') {
			continue;
		}
		
		// we have a line of input to work with
		strncpy ((void *)buffer, (void *)start, 256);
		
		// see if we found an EOF (used by ATI)
		if (strncasecmp ((void *)buffer, "EOF", 3) == 0) {
			status = VSTATUS_BAD;
		} else {
			status = VSTATUS_OK;
		}
		break;
	}
	
	return (status);
}
#endif /* __VXWORKS__ */
