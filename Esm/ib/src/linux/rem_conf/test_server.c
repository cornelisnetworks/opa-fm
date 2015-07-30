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
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <signal.h>
#include "hsm_com_api.h"
#include "hsm_com_test_data.h"

hsm_com_errno_t 
test_callback(hsm_com_datagram_t *data)
{
	printf("Test callback: LEN: %d, BUFLEN: %d \n",
		   data->data_len,data->buf_size);


	return HSM_COM_OK;
}
void
usage(char *cmd)
{
	printf("%s path\n",cmd);
}

int
main(int argc, char *argv[])
{
	int		serv_fd;
	int		client_fd;
	uid_t	client_uid;
	p_hsm_com_server_hdl_t server_hdl;

	if(argc < 2){
		usage(argv[0]);
		return(-1);
	}



	if(hcom_server_init(&server_hdl,argv[1],5,1024,&test_callback) != HSM_COM_OK){
		printf("Could not allocate server handle\n");
		return(-1);
	}

	if(hcom_server_start(server_hdl) != HSM_COM_OK){
		printf("Server exited with error\n");
	}

	printf("Server exiting\n");

	return 0;


}

