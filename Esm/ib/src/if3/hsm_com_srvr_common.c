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

#ifdef __LINUX__
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include "hsm_com_srvr_api.h"
#include "hsm_com_srvr_data.h"


	
int
validate_header(char *buf, int len)
{
	hsm_com_common_t	*common;
	int					expect;


	if(len < sizeof(hsm_com_common_t)){
		return(-1);
	}

	common = (hsm_com_common_t*)buf;
	expect = common->payload_len + sizeof(hsm_com_common_t);


	if(expect == len){
		return(0);
	}else if(expect < len){
		return(expect); // expect is the offset into the buffer where the next packet starts.
	}else{
		return(-1); // Didn't get the whole thing yet.
	}



	
}


int
unix_sck_read_data(int fd, hsm_com_scratch_t *scratch, char *buf, int buf_len, int *offset)
{
	char	*temp = NULL;
	int		len = 0;
	int		nread;
	int		next_packet = 0;
	char    *data_ptr;

	*offset = 0;

	if((nread = read(fd, buf, buf_len)) <= 0){
		// Error on socket...
		return(0);
	}
	//printf("Got: %d bytes + prefill of %d:%d\n",nread,scratch->scratch_fill,scratch->scratch_len);

	if(scratch->scratch_fill){
		// Malloc space for fill + new data...
		if((temp = malloc(scratch->scratch_fill + nread)) == NULL){
			// Error..
			//printf("Could not allocate temp buffer\n");
			if (temp)
				free(temp);
			return(0);
		}

		//printf("Got: %d bytes + prefill of %d\n",nread,scratch->scratch_fill);
		memcpy(&temp[0],scratch->scratch,scratch->scratch_fill);
		memcpy(&temp[scratch->scratch_fill],buf,nread);
		data_ptr = temp;
		len = nread + scratch->scratch_fill;

	}else{
		data_ptr = buf;
		len = nread;
	}
	

	while((next_packet = validate_header(data_ptr,len)) > 0){
		if(next_packet > len){
			// Error 
			//printf("unix_sck_read_data: mis-formatted data. packet len too big\n");
			if (temp) free(temp);
			return(-1);
		}
		
		*offset += next_packet;
		data_ptr = &buf[*offset];
		len -= next_packet;
	
	}

	if(next_packet == 0){
		
		if(temp){
			memcpy(buf,data_ptr,len);
			free(temp);
			*offset = 0;
			scratch->scratch_fill = 0;
		}

		return len;
	}else{
		if(len <= scratch->scratch_len){
			memcpy(scratch->scratch,data_ptr,len);
			scratch->scratch_fill = len;
		}
		if(temp)
			free(temp);
		return(-2);
	}

	if(temp)
		free(temp);

	return (-1);


}
#endif
