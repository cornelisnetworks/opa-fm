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
//                                                                      /
// FILE NAME                                                            /
//    qp1sink.c                                                         /
//                                                                      /
// DESCRIPTION                                                          /
//    This program will wait 1 seconds for a message to come in off of  /
//    the fabric.  It will print it out and exit.                       /
//                                                                      /
// DATA STRUCTURES                                                      /
//    None                                                              /
//                                                                      /
// FUNCTIONS                                                            /
//    main                              main entry point                /
//                                                                      /
// DEPENDENCIES                                                         /
//    ib_mad.h                                                          /
//    ib_status.h                                                       /
//    sm_pool.h                                                         /
//    sm_event.h                                                        /
//    sm_thread.h                                                       /
//                                                                      /
//                                                                      /
// HISTORY                                                              /
//                                                                      /
//    NAME      DATE  REMARKS                                           /
//     pjg  06/19/02  Initial creation (from modified qp1recv) of file. /
//                                                                      /
//=======================================================================


#include <stdio.h>
#include <errno.h>
#include <linux/ioctl.h>
#include "ib_status.h"
#include "ib_types.h"
#include "ib_mad.h"
#include "mai_g.h"
#include "mal_g.h"
#include "cs_g.h"
#include "ib_macros.h"
#include <stdlib.h>

extern  int dumpit(Mai_t *);

unsigned long long want_tid[16] = {0};
int main(int argc, char *argv[]) {
        IBhandle_t      fd;
        Mai_t           in_mai;
        Filter_t        filter;
        Status_t        status;
    uint32_t    port;
	uint32_t	dev=0;

    if (argc != 2)
    {
        fprintf(stderr, "usage: qp1sink <port>\n");
        exit(-1);
    }
    port = strtoul(argv[1], NULL, 0);

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
//      Create the filter so we can receive the reply to our request.
//
        Filter_Init(&filter, MAI_ACT_FMASK, MAI_TYPE_EXTERNAL);
        filter.value.tid = 0xF002030405060708ull;
        filter.mask.tid = 0xF000000000000000ull;

        status = mai_filter_create(fd, &filter, VFILTER_SHARE);
        if (status != VSTATUS_OK) {
                fprintf(stderr, "Can't create a filter (%d)\n", status);
                mai_close(fd);
                exit(1);
        }

        while (1)
    {

        //
        //      Wait five second for an answer.
        //
            status = mai_recv(fd, &in_mai, 1000000);
            if (status != VSTATUS_OK) {
                    fprintf(stderr, "Didn't receive a MAD (%d)\n", status);
            continue;
            }

        //
        //      Display the MAD that came in.
        //
        dumpit(&in_mai);
        }

//
//      Delete the filter.
//
        status = mai_filter_delete(fd, &filter, VFILTER_SHARE);
        if (status != VSTATUS_OK) {
                fprintf(stderr, "Can't delete a filter (%d)\n", status);
                mai_close(fd);
                exit(1);
        }

        mai_close(fd);
        exit(0);
}

//
//      Dump out the received data.
//
int
dumpit(Mai_t *maip) {
    int     tid_index;


#if 1
        unsigned long long *ullptr = ((unsigned long long *) &maip->base) + 1;
        tid_index = ((*ullptr / 0x0100000000000000ull) & 0x0f);
        if (want_tid[tid_index] != *ullptr)
        {
           fprintf(stderr, "\nExpected %016llX, got %016llX\n", want_tid[tid_index], *ullptr);
           want_tid[tid_index] = *ullptr;
        }
        else
        {
           fputc(tid_index + (tid_index >= 10 ? ('A' - 10) : ('0')), stderr);
           fflush(stderr);
        }
        want_tid[tid_index]++;
#else
		printf("Addr: SLID %04x DLID %04x SL %u PKEY %04x\n",
			maip->addrInfo.slid, maip->addrInfo.dlid, maip->addrInfo.sl, maip->addrInfo.pkey);
		printf("      SrcQP %u DestQP %u QKEY %08x\n",
			maip->addrInfo.srcqp, maip->addrInfo.destqp, maip->addrInfo.qkey);
        cp = (uint8_t *)&maip->base;
        printf("%02x %02x %02x %02x %02x %02x %02x %02x\n",
                *(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7));

        cp += 8;
        printf("%02x %02x %02x %02x %02x %02x %02x %02x               // TID\n",
                *(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7));
        cp += 8;
        printf("%02x %02x %02x %02x %02x %02x %02x %02x               // AID, AMOD\n\n",
                *(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7));

        cp += 8;
        cp = (uint8_t *)maip->data;
        printf("%02x %02x %02x %02x %02x %02x %02x %02x               // MKEY\n\n",
                *(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7));

        cp += 8;
        printf("%02x %02x %02x %02x %02x %02x %02x %02x               // RSVD\n",
                *(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7));
        cp += 8;
        printf("%02x %02x %02x %02x %02x %02x %02x %02x\n",
                *(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7));
        cp += 8;
        printf("%02x %02x %02x %02x %02x %02x %02x %02x\n",
                *(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7));
        cp += 8;
        printf("%02x %02x %02x %02x %02x %02x %02x %02x\n",
                *(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7));

//      Data
        cp += 8;
        printf("\n");
        printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x   // DATA\n",
                *(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7),
                *(cp+8), *(cp+9), *(cp+10), *(cp+11), *(cp+12), *(cp+13), *(cp+14), *(cp+15));
        for (i = 0; i < 3; i++) {
                cp += 16;
                printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                        *(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7),
                        *(cp+8), *(cp+9), *(cp+10), *(cp+11), *(cp+12), *(cp+13), *(cp+14), *(cp+15));
        }

        printf("============================================================\n");
#endif

        return(0);
}
