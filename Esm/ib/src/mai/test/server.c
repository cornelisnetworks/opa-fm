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

** END_ICS_COPYRIGHT5   ****************************************/

/************************************************************************/
/*                                                                      */
/* FILE NAME                                                            */
/*    server.c                                                          */
/*                                                                      */
/* DESCRIPTION                                                          */
/*    Test MAI server - program executes a loop which basically        */
/*                      receives MADs based on filters that are         */
/*                      specified by command line argumnets             */
/*                                                                      */
/*                                                                      */
/* DEPENDENCIES                                                         */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

#include "ib_types.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "ib_macros.h"
#include "cs_g.h"
#include "cs_log.h"
#include "mai_g.h"
#include "mal_g.h"
#include <ib_sa.h>
#include <ib_mad.h>
#include "cs_log.h"
#include "test.h"


extern void srand(unsigned int seed);
extern int rand(void);


/* Default values */

int loop = 10;               /* how many times to loop */
uint32_t ib_dev = 0;              /* device that I am running from */
uint32_t ib_port = 5;             /* port that I am attached to */
uint8_t mclass = 4;          /* my management class */
uint32_t timeout = 15000000; /* how long to wait for messages */
uint32_t filter = 1;         /* which filter to create */


/*-------------------------------------------------------------------*
* help - SYNTAX error message
*-------------------------------------------------------------------*/


void
help(void)
{
  printf("\nMAI Test Receiver\n");
  printf("Syntax: mai_recvr [ -d <ibdev> -p <ibport>  \n");
  printf("\t\t -m <manager class> -?] ...\n\n");
  printf("        -d    Specify IBA device to open\n");
  printf("        -p    Specify IBA port  on the device\n");
  printf("        -k    Specify how many times to execute loop\n");
  printf("        -m    Specify manager class\n");
  printf("        -f    Specify filter to attach to handle\n");
  printf("              1 - mclass filter\n");
  printf("              2 - method filter for SEND_ONE\n");
  printf("              3 - option filter for SEND_ONE, AID_ONE\n");
  printf("        -t    Specify how long to wait for message (msecs)\n");
  printf("        -?    Display this help message\n\n");

  exit(1);
}



/*-------------------------------------------------------------------*
parse_cmd_line - Get command line args
*-------------------------------------------------------------------*/

void
parse_cmd_line(int argc, char** argv)
{
  int op;                        /* option return from getopt    */
  extern  char* optarg;


  while ((op = getopt(argc,argv,"d:p:k:m:f:t:?")) != EOF)
    {
      switch (op)
        {
        case '?':
          help();
          break;

        case 'd':
          ib_dev = atoi(optarg);
          break;


        case 'k':
          loop = atoi(optarg);
          break;

        case 't':
          timeout = atoi(optarg);
          break;

        case 'p':
          ib_port = atoi(optarg);
          break;

        case 'm':
          mclass = atoi(optarg);
          break;

        case 'f':
          filter = atoi(optarg);
          break;

        default:
          help();
          break;
        }
    }
}



int
main(int argc, char* argv[])
{
  int rc,i;
  IBhandle_t fd,fh;
  Mai_t mad;
  uint16_t taid;
  uint16_t method,dlid,slid;


  if (argc > 1)
      parse_cmd_line(argc,argv);


  /* Initialize MAD */
  memset(&mad,0,sizeof(mad));

  /* Initialize MAI subsystem */
  mai_init();

  rc=ib_init_devport(&ib_dev, &ib_port, NULL, NULL);
  if (rc)
    {
      printf("ib_init_devport failed, %d\n",rc);
      return rc;
    }


  /* Open connection to listen for messages on the device and port  */
  /* passed in. We want to use the general services (GS) queue pair */
  rc = mai_open(MAI_GSI_QP, ib_dev, ib_port, &fd);

  printf("Opened a handle: %d \n",(int)fd);

  if (rc)
    {
      printf("MAI_OPEN failed, %d\n",rc);
      return rc;
    }


  /* Now create filters that would allow us to receive commands on */
  /* the handle.                                                   */
  switch (filter)
    {
    case 1:
      rc = mai_filter_mclass(fd,VFILTER_SHARE,MAI_TYPE_ANY, &fh, mclass);
      if (rc)
        {
          printf("Cannot create MCLASS filter\n");
          return rc ;
        }
      break;
    case 2:
      rc = mai_filter_method(fd,VFILTER_SHARE,MAI_TYPE_ANY, &fh, mclass,
                             SEND_ONE);
      if (rc)
        {
          printf("Cannot create METHOD filter\n");
          return rc;
        }
      break;
    case 3:
      rc = mai_filter_aid(fd,VFILTER_SHARE,MAI_TYPE_ANY, &fh, mclass,
                          SEND_ONE,AID_ONE);
      if (rc)
        {
          printf("Cannot create MCLASS filter\n");
          return rc;
        }
      break;
    default:
      printf("Invalid filter type\n");
      return 0;
      break;
    }


  /* Loop to receive messages */
  for (i = 0; i < loop; i++)
    {
      printf("Waiting for MADs\n");

      /* Get messages */
      rc = mai_recv(fd,&mad,timeout);

      if (rc == VSTATUS_TIMEOUT)
        {
          printf("Loop %d: timeout \n",i);
          continue;
        }

      if (rc != VSTATUS_OK)
        {
          return rc;
        }


      taid = mad.base.aid;
      method = mad.base.method;

      /* Find out what method was received */
      switch (method)
        {
        case SEND_ONE:
          printf("SEND_ONE received\n");
          break;

        case GET_ONE:
          printf("GET_ONE received\n");
          break;

        case RESPONSE:
          printf("RESPONSE received\n");
          break;

        default:
          printf("Unknown method received\n");
          break;
        }

      /* Find out what AID we received */
      switch (taid)
        {
        case AID_ONE:
          printf("AID_ONE received\n");
          break;

        case AID_TWO:
          printf("AID_TWO received\n");
          break;

        default:
          printf("Unknown AID received\n");
          break;
        }

      /* Prepare to send acknowledgement */
      mad.base.method = RESPONSE;

      /* Swizzle destination and source lid */
      dlid = mad.addrInfo.slid;
      slid = mad.addrInfo.dlid;
      mad.addrInfo.slid = slid;
      mad.addrInfo.dlid = dlid;
      mad.active |= MAI_ACT_ADDRINFO;

      /* Send acknowledgement MAD */
      rc = mai_send(fd,&mad);
      if (rc)
        {
          printf("MAI send failed %d \n",rc);
          return rc;
        }

      printf("Loop %d: \n",i);
    }

  /* Delete filter */
  mai_filter_hdelete(fd,fh);

  /* Close MAI channel */
  mai_close(fd);
  printf("Test successful!!\n");
  return 0;
}
