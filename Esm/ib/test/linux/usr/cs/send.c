/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

//=======================================================================
//									/
// FILE NAME                                                            /
//    send.c                                                            /
//                                                                      /
// DESCRIPTION                                                          /
//    This program will send a LID routed MAD using the SEND method.    /
//    The 'recv' program will receive the MAD and then send a reply.    /
//    To use the programs, first start the recv, then the send.         /
//									/
// DATA STRUCTURES							/
//    None								/
//									/
// FUNCTIONS								/
//    main				main entry point		/
//									/
// DEPENDENCIES								/
//    ib_mad.h								/
//    ib_status.h							/
//    sm_pool.h								/
//    sm_event.h							/
//    sm_thread.h							/
//									/
//									/
// HISTORY								/
//									/
//    NAME	DATE  REMARKS						/
//     jsy  04/28/01  Initial creation of file.				/
//									/
//=======================================================================


#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include "ib_status.h"
#include "ib_types.h"
#include "ib_mad.h"
#include "mai_g.h"
#include "cs_g.h"
#include "ib_macros.h"

int main(int argc, char *argv[]) {
	IBhandle_t	fd;
	uint32_t	slid;
	uint32_t	dlid;
	Mai_t		out_mai;
	Status_t	status;

//
//	Get the slid and dlid.
//
	if (argc != 3) {
		fprintf(stderr, "send <slid> <dlid>\n");
		exit(1);
	}

	slid = strtoul(argv[1], NULL, 0);
	dlid = strtoul(argv[2], NULL, 0);

//
//	Open my log file.
//
	//vs_log_open("/tmp/send.log", 0);

//
//	Initialize the MAI subsystem and open the port.
//
	mai_init();
	status = mai_open(0, 0, 5, &fd);
	if (status != VSTATUS_OK) {
		fprintf(stderr, "Can't open MAI (%d)\n", status);
		exit(0);
	}

//
//	Setup the data for a MAD.
//
	Mai_Init(&out_mai);
	AddrInfo_Init(&out_mai, slid, dlid, SMI_MAD_SL, STL_DEFAULT_FM_PKEY, MAI_SMI_QP, MAI_SMI_QP, 0x0);
	LRMad_Init(&out_mai, MAD_CV_SUBN_LR, MAD_CM_SEND, 0x0102030405060708ull, 0x10, 0x0, 0x0);

//
//	Send the request.
//
	status = mai_send(fd, &out_mai);
	if (status != VSTATUS_OK) {
		fprintf(stderr, "Can't send a MAD (%d)\n", status);
		mai_close(fd);
		exit(1);
	}

	mai_close(fd);
	exit(0);
}
