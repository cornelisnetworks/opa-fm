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
//                      
// FILE NAME   
//    qp1source.c 
//          
// DESCRIPTION  
//    This program will send a LID routed MAD using the SEND method.
//    The 'recv' program will receive the MAD and then send a reply.
//    To use the programs, first start the recv, then the send.    
//                                                                      
// DATA STRUCTURES                              
//    None                                      
//                                              
// FUNCTIONS            
//    main                              main entry point
//                              
// DEPENDENCIES                                         
//    ib_mad.h                          
//    ib_status.h               
//    sm_pool.h         
//    sm_event.h
//    sm_thread
//                      
//                                                      
// HISTORY                              
//                                      
//    NAME      DATE  REMARKS                                   
//     pjg  06/19/02  Initial creation (from modified qp1send) of file.
//                                      
//=======================================================================


#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include "ib_status.h"
#include "ib_types.h"
#include "ib_mad.h"
#include "mai_g.h"
#include "mal_g.h"
#include "cs_g.h"
#include "ib_macros.h"
#include "unistd.h"

int main(int argc, char *argv[]) {
        IBhandle_t      fd;
        uint32_t        slid;
        uint32_t        dlid;
        Mai_t           out_mai;
        Status_t        status;
    int         count;
    unsigned long delay;
    unsigned long long tid = 0x0000000000000000ull;
    uint32_t    dev = 0;
    uint32_t    port;
    uint8_t     tid_index;

//
//      Get the slid and dlid.
//
        if (argc != 7) {
                fprintf(stderr, "qp1source <port> <slid> <dlid> <count, negative for continuous> <delay (in uS)> <tid_index>\n");
                exit(1);
        }

    port = strtoul(argv[1], NULL, 0);
        slid = strtoul(argv[2], NULL, 0);
        dlid = strtoul(argv[3], NULL, 0);
    count = atoi(argv[4]);
    if (count<0) count = 1;
    delay = strtoul(argv[5], NULL, 0);
    tid_index = strtoul(argv[6], NULL, 0);
    tid = tid_index * 0x0100000000000000ull;

//
//      Initialize the MAI subsystem and open the port.
//
        mai_init();
        status=ib_init_devport(&dev, &port, NULL, NULL);
        if (status)
          {
            printf("ib_init_devport failed, %d\n",status);
            exit(1);
          }
        status = mai_open(1, 0, port, &fd);
        if (status != VSTATUS_OK) {
                fprintf(stderr, "Can't open MAI (%d)\n", status);
                exit(0);
        }

//
//      Setup the data for a MAD.
//
        Mai_Init(&out_mai);
        AddrInfo_Init(&out_mai, slid, dlid, 0, STL_DEFAULT_FM_PKEY, MAI_GSI_QP, MAI_GSI_QP, GSI_WELLKNOWN_QKEY);

    while (count != 0)
    {
        usleep(delay);
        LRMad_Init(&out_mai, MAD_CV_SUBN_LR, MAD_CM_SEND, tid++, 0x10, 0x0, 0x0);

        //
        //      Send the request.
        //
        status = mai_send(fd, &out_mai);
        if (status != VSTATUS_OK) {
            fprintf(stderr, "Can't send a MAD (%d)\n", status);
            mai_close(fd);
            exit(1);
        }

        if (count > 0)
            count--;
    }

        mai_close(fd);
        exit(0);
}
