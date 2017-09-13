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
/*    client.c                                                          */
/*                                                                      */
/* DESCRIPTION                                                          */
/*    Test MAI client - program sends MADS to a server and waits for    */
/*                      an acknowledgement.                             */
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
#include "test.h"

extern void srand(unsigned int seed);
extern int rand(void);

/* Default values */
int loop = 1;                   /* how many times should I loop */
uint32_t ib_dev = 0;                 /* what device am I running from */
uint32_t ib_port = 5;                /* what port am I running from */
uint8_t mclass = 4;             /* mclass of receiver */
uint16_t slid = 1;              /* my lid */
uint16_t dlid = 1;              /* lid of receiver */
uint16_t aid = AID_ONE;         /* Attribute ID */
uint16_t method = SEND_ONE;     /* Method to use in MAD */
uint32_t timeout = 15000000;    /* how long to wait for acknowledgement */
int      use_event;
int oob;

/*-------------------------------------------------------------------*
* help - SYNTAX error message
*-------------------------------------------------------------------*/


void
help(void)
{
  printf("\n MAI Test Sender \n");
  printf("Syntax: mai_sender [ -d <ibdev> -p <ibport>  \n");
  printf("\t\t-c <command> -m <manager class> -l <dlid>  \n");
  printf("\t\t-h]\n\n");
  printf("      -d    Specify iba device to open\n");
  printf("      -p    Specify iba port  on the device\n");
  printf("      -m    Specify manager class\n");
  printf("      -l    Specify sender lid (slid)\n");
  printf("      -r    Specify receiver lid (dlidi)\n");
  printf("      -e    Use global event\n");
  printf("      -k    Loops to execute\n");
  printf("      -c    Specify method to use\n");
  printf("                1 - Send SEND_ONE.\n");
  printf("                2 - Send GET_ONE.\n");
  printf("      -a    Specify aid to send\n");
  printf("                1 - Send AID_ONE.\n");
  printf("                2 - Send AID_TWO.\n");
  printf("      -t    Specify how long to wait for acknowledgement (msecs) \n");
  printf("      -o    Use INTERNAL messages\n");
  printf("      -?    Display this help message\n\n");


  exit(1);
}

/*-------------------------------------------------------------------*
parse_cmd_line - Get command line args
*-------------------------------------------------------------------*/

void
parse_cmd_line(int argc, char** argv)
{
  int op, command;
  extern  char* optarg;


  while ((op = getopt(argc,argv,"t:a:k:d:p:c:m:r:l:eo?")) != EOF)
    {
      switch (op)
        {
        case '?':
          help();
          break;

        case 't':
          timeout = atoi(optarg);
          break;

        case 'd':
          ib_dev = atoi(optarg);
          break;

        case 'p':
          ib_port = atoi(optarg);
          break;

	case 'e':
          use_event=1;
          break;


	case 'o':
          oob=1;
          break;

        case 'c':
          command = atoi(optarg);
          switch (command)
            {
            case 1:
              method = SEND_ONE;
              break;
            case 2:
              method = GET_ONE;
              break;
            default:
              printf("Exiting: invalid method\n");
              exit(1);
              break;
            }
          break;

        case 'a':
          command = atoi(optarg);
          switch (command)
            {
            case 1:
              aid = AID_ONE;
              break;
            case 2:
              aid = AID_TWO;
              break;
            default:
              printf("Exiting: invalid aid\n");
              exit(1);
              break;
            }
          break;

        case 'k':
          loop = atoi(optarg);
          break;

        case 'r':
          dlid = atoi(optarg);
          break;

        case 'l':
          slid = atoi(optarg);
          break;

        case 'm':
          mclass = atoi(optarg);
          break;

        default:
          help();
          break;
        }
    }
}


Event_t   event;
int
main(int argc, char* argv[])
{
  int rc,i;
  uint64_t tid;
  IBhandle_t fd = -1,fh;
  Mai_t mad, rmad;
  uint8_t         name[16];


  /* Parse command line arguments */
  if (argc > 1)
      parse_cmd_line(argc,argv);


   /* Initialize MAD */
   memset(&rmad,0,sizeof(rmad));

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

  /* Create MAI handle to used to communicate */
  rc = mai_open(MAI_GSI_QP,ib_dev,ib_port,&fd);

  if (rc)
    {
      printf("MAI_OPEN failed, %d\n",rc);
      return rc;
    }

  /* Initialize MAD that will be sent */
  if(oob)
    mad.type = MAI_TYPE_INTERNAL;
  else
    mad.type = MAI_TYPE_EXTERNAL;


  mad.base.method = method;
  mad.base.mclass = mclass;
  mad.base.aid = aid;
  AddrInfo_Init(&mad, slid, dlid, 0, STL_DEFAULT_FM_PKEY, MAI_GSI_QP, MAI_GSI_QP, GSI_WELLKNOWN_QKEY);
  mad.base.bversion = MAD_BVERSION;
  mad.base.cversion = MAD_CVERSION;
  mad.active = MAI_ACT_BASE | MAI_ACT_DATA | MAI_ACT_TYPE |
               MAI_ACT_ADDRINFO;


  /* Create filter to listen for response */
  rc = mai_filter_method(fd,VFILTER_SHARE,MAI_TYPE_ANY, &fh, mclass,
                         RESPONSE);
  if (rc)
    {
      printf("Cannot create filter to listen for responses \n");
      return rc;
    }


  if(use_event)
    {
      sprintf((char *) name, "client");
      rc = vs_event_create(&event, name, (Eventset_t) 0x00U);

      if (rc)
	{
	  IB_LOG_ERRORRC("vs_event_create failed rc:", rc);
	  return rc;
	}

    }


  /* Loop which sends MADS to receiver */
  for (i = 0; i < loop; i++)
    {
      /* Get a transaction ID for message we are about to send */
      rc = mai_alloc_tid(fd, mclass, &tid);

      /* Assign transaction ID to the MAD to be sent */
      mad.base.tid = tid;

      /* Send command to receiver */
      rc = mai_send(fd,&mad);
      if (rc)
        {
          printf("MAI send failed %d \n",rc);
          return rc;
        }

      printf("MAD sent\n");
      printf("Waiting for acknowledgement\n");


      if(use_event)
	{
	  Eventset_t      events = (Eventset_t) 0U;

	   do{
	    rc = vs_event_wait(event.event_handle,
			       timeout,0x1,&events);
	  }while(rc == VSTATUS_AGAIN);

	  if(rc == VSTATUS_OK)
	    {
	      rc = mai_recv(fd,&rmad,MAI_RECV_NOWAIT);
	    }
	  printf("Event wait returned %d\n",rc);
	}
      else
	{
	  /* Get acknowledgement */
	  rc = mai_recv(fd,&rmad,timeout);
	}


      if (rc != VSTATUS_OK)
        {
          printf("Acknowledgement not received \n");
          return rc;
        }

      printf("Acknowledgement received\n");
      printf("Passed Loop %d\n ",i);

      //sleep(1);
    }

  /* Delete filter */
  mai_filter_hdelete(fd,fh);

  /* Close MAI channel */
  rc = mai_close(fd);
  if (rc)
    {
      printf("Close failed\n");
      return rc;
    }

  printf("Test successful!!\n");
  return rc;
}
