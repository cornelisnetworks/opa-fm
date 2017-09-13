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
//======================================================================= 1
//									/
// FILE NAME								/
//    qp0.c								/
//									/
// DESCRIPTION								/
//    This program will send a LID or directed routed MAD.		/
//    The MAD uses Get(NodeDescription) as the method and attribute ID.	/
//    For LID routed, the source LID and destination LID are provided	/
//    For directed route, a path is specified. path[0] is the number of	/
//    hops and path[1], path[2], ... are the ports to use.		/
//									/
//=======================================================================


#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include "ib_status.h"
#include "ib_types.h"
#include "ib_mad.h"
#include "mai_g.h"
#include "mal_g.h"
#include "cs_g.h"
#include "ib_macros.h"

uint32_t ib_dev   = 0;
uint32_t ib_port  = 1;
char *mai_host_name;
int  mai_host_port=4999;
uint32_t	slid;
uint32_t	dlid;
uint8_t		path[64];
unsigned aid = 0x10;	// NodeDesc
unsigned amod = 0x0;

extern	int dumpit(Mai_t *);

void help(void)
{
  printf("\nqp0test - send and receive QP0 packets via MAI\n");
  printf("Syntax: qp0test [ -d <ibdev> -p <ibport> \n\t\t-s <sport> -n <sname> -h] slid dlid\n\n");
  printf("or qp0test [ -d <ibdev> -p <ibport> \n\t\t-s <sport> -n <sname> -h] 0xffff path...\n\n");
  printf("        -d    Specify iba device to open\n");
  printf("        -p    Specify iba port  on the device\n");
  printf("        -s    Specify simulator IP port to use\n");
  printf("        -n    Spefify simulaor host machine name\n");
  printf("        -h    Display this help message\n");
  printf("        slid  slid to put in packet\n");
  printf("        dlid  dlid to put in packet\n\n");
  printf("        path  zero or more ports for directed route outbound path\n\n");

  exit(1);
}

/*-------------------------------------------------------------------*
	parse_cmd_line - Get command line args
 *-------------------------------------------------------------------*/

void parse_cmd_line(int argc,char **argv)
{
  int     op;                        /* option return from getopt    */
  int i, j;
  extern  char *optarg;


  while ((op = getopt(argc,argv,"d:D:p:P:hH?")) != EOF)
    {
      switch (op)
	{
	case 'h':
	case 'H':
	case '?':
	  help();
	  break;
 
	case 'D':
	case 'd':
	  ib_dev = atoi(optarg);
	  break;

	case 'p':
	case 'P':
	  ib_port = atoi(optarg);
	  break;

	case 's':
	case 'S':
	  mai_host_port = atoi(optarg);
	  break;
			
	case 'n':
	case 'N':
	  mai_host_name= optarg;
	  break;
			
	default:
	  help();
	  break;
	}
    }

    if (argc <=optind ) {
	help();
	exit(1);
    }

    slid = strtoul(argv[optind++], NULL, 0);
    if (slid != 0xffff) {
	// LID routed
    	if (argc <=optind ) {
		help();
		exit(1);
    	}
    	dlid = strtoul(argv[optind], NULL, 0);
    } else {
	// directed route
	dlid=0xffff;
	for (i = optind, j = 1; i < argc; i++, j++) {
		path[0]++;
		path[j] = strtoul(argv[i], NULL, 0);
	}
    }
}

int main(int argc, char *argv[]) {
	IBhandle_t	fd;
	Mai_t		in_mai;
	Mai_t		out_mai;
	Filter_t	filter;
	Status_t	rc;

//
//	Get the slid and dlid.
//
    	parse_cmd_line(argc,argv); 

//
//	Initialize the MAI subsystem and open the port.
//
	mai_init();
        rc=ib_init_devport(&ib_dev, &ib_port, NULL, NULL);
        if (rc)
          {
            printf("ERROR: ib_init_devport failed, %s\n",cs_convert_status(rc));
            exit(1);
          }

	rc = mai_open(0, ib_dev, ib_port, &fd);
	if (rc != VSTATUS_OK) {
		printf("ERROR: Can't open MAI (%s)\n", cs_convert_status(rc));
		exit(0);
	}

//
//	Setup the data for a MAD.
//
	Mai_Init(&out_mai);
	AddrInfo_Init(&out_mai, slid, dlid, SMI_MAD_SL, STL_DEFAULT_FM_PKEY, MAI_SMI_QP, MAI_SMI_QP, 0x0);
	if (slid != 0xffff) {
		LRMad_Init(&out_mai, MAD_CV_SUBN_LR, MAD_CM_GET, 0xdeadbeefdeadbeefull, aid, amod, 0x0);
		printf("Sending LID routed Get(NodeDesc) from 0x%x to 0x%x\n", slid, dlid);
	} else {
		DRMad_Init(&out_mai, MAD_CM_GET, 0xdeadbeefdeadbeefull, aid, amod, 0x0, path);
		printf("Sending Directed routed Get(NodeDesc)\n");
	}

	dumpit(&out_mai);

//
//	Create the filter so we can receive the reply to our request.
//
	Filter_Init(&filter, MAI_ACT_FMASK, MAI_TYPE_EXTERNAL);
	filter.value.method = MAD_CM_GET_RESP;
	filter.mask.method  = 0xff;
	filter.value.tid = out_mai.base.tid;
	filter.mask.tid = 0xffffffffffffffffull;

	rc = mai_filter_create(fd, &filter, VFILTER_SHARE);
	if (rc != VSTATUS_OK) {
		printf("ERROR: Can't create a filter (%s)\n", cs_convert_status(rc));
		mai_close(fd);
		exit(1);
	}

//	Send the request.
	rc = mai_send(fd, &out_mai);
	if (rc != VSTATUS_OK) {
		printf("ERROR: Can't send a MAD (%s)\n", cs_convert_status(rc));
		mai_close(fd);
		exit(1);
	}

//	Wait up to one second for an answer.
	rc = mai_recv(fd, &in_mai, 1000000);
	if (rc != VSTATUS_OK) {
		printf("ERROR: Can't receive a MAD (%s)\n", cs_convert_status(rc));
		mai_close(fd);
		exit(1);
	}

//	Display the reply from the SMA on the other end.
	dumpit(&in_mai);

//	Delete the filter.
	rc = mai_filter_delete(fd, &filter, VFILTER_SHARE);
	if (rc != VSTATUS_OK) {
		printf("ERROR: Can't delete a filter (%s)\n", cs_convert_status(rc));
		mai_close(fd);
		exit(1);
	}

	mai_close(fd);
	exit(0);
}

//
//	Dump out the received data.
//
int
dumpit(Mai_t *maip) {
	int		i;
	uint8_t		*cp;

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
	printf("%02x %02x %02x %02x %02x %02x %02x %02x               // DR LIDs & RSVD\n",
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

//	Data
	cp += 8;
	printf("\n");
	printf("NodeDesc: %s\n", cp);
	printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x   // DATA\n",
		*(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7),
		*(cp+8), *(cp+9), *(cp+10), *(cp+11), *(cp+12), *(cp+13), *(cp+14), *(cp+15));
	for (i = 0; i < 3; i++) {
		cp += 16;
		printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			*(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7),
			*(cp+8), *(cp+9), *(cp+10), *(cp+11), *(cp+12), *(cp+13), *(cp+14), *(cp+15));
	}

//	Ipath
	cp += 16;
	printf("\n");
	printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x   // IPATH\n",
		*(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7),
		*(cp+8), *(cp+9), *(cp+10), *(cp+11), *(cp+12), *(cp+13), *(cp+14), *(cp+15));
	for (i = 0; i < 3; i++) {
		cp += 16;
		printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			*(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7),
			*(cp+8), *(cp+9), *(cp+10), *(cp+11), *(cp+12), *(cp+13), *(cp+14), *(cp+15));
	}

//	Rpath
	cp += 16;
	printf("\n");
	printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x   // RPATH\n",
		*(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7),
		*(cp+8), *(cp+9), *(cp+10), *(cp+11), *(cp+12), *(cp+13), *(cp+14), *(cp+15));
	for (i = 0; i < 3; i++) {
		cp += 16;
		printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			*(cp+0), *(cp+1), *(cp+2), *(cp+3), *(cp+4), *(cp+5), *(cp+6), *(cp+7),
			*(cp+8), *(cp+9), *(cp+10), *(cp+11), *(cp+12), *(cp+13), *(cp+14), *(cp+15));
	}

	printf("============================================================\n");

	return(0);
}
