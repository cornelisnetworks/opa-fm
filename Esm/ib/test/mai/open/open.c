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
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

#include "ib_types.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "mal_g.h"


uint32_t ib_dev   = 0;
uint32_t ib_port  = 1;
int ib_qp    = 0;

/*-------------------------------------------------------------------*
 * help - SYNTAX error message 
 *-------------------------------------------------------------------*/


void help(void)
{
  printf("\notest - Exercise open in MAI\n");
  printf("Syntax: ftest [ -d <ibdev> -p <ibport> -q <qp>\n\t\t -h]\n\n");
  printf("        -d    Specify iba device to open\n");
  printf("        -p    Specify iba port  on the device\n");
  printf("        -q    Specify qp\n");
  printf("        -h    Display this help message\n\n");

  exit(1);
}

/*-------------------------------------------------------------------*
	parse_cmd_line - Get command line args
 *-------------------------------------------------------------------*/

void parse_cmd_line(int argc,char **argv)
{
  int     op;                        /* option return from getopt    */
  extern  char *optarg;


  while ((op = getopt(argc,argv,"d:D:p:P:q:Q:hH?")) != EOF)
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

	case 'q':
	case 'Q':
	  ib_qp = atoi(optarg);
	  break;

	case 'p':
	case 'P':
	  ib_port = atoi(optarg);
	  break;

	default:
	  help();
	  break;
	}
    }
}

int main(int argc, char *argv[])
{	

  int rc,i,q;
  IBhandle_t fd, fd2;
  uint64_t tid;
    
  if(argc > 1)
    parse_cmd_line(argc,argv); 

  mai_init();

  rc=ib_init_devport(&ib_dev, &ib_port, NULL, NULL);
  if (rc)
    {
      printf("ERROR: ib_init_devport failed, %s\n",cs_convert_status(rc));
      exit(1);
    }

  IB_ENTER("otest", ib_qp, ib_dev, ib_port, 0);
  
  for(q=0;q<10;q++)
    {
      rc = mai_open(ib_qp,ib_dev,ib_port,&fd);
  
      printf("otest: open got returned status %s\n",cs_convert_status(rc));

      if(rc != VSTATUS_OK)
	{
	  printf("ERROR: open failed 1: %s\n",cs_convert_status(rc));
	  return rc;
	}
  
      rc = mai_open(ib_qp,ib_dev,ib_port,&fd2);
  
      printf("otest: open got returned status %s\n",cs_convert_status(rc));

      if(rc != VSTATUS_OK)
	{
	  printf("ERROR: open failed 2: %s\n",cs_convert_status(rc));
	  return rc;
	}
  

      for(i=0;i<3;i++)
	{
      
	  rc=mai_alloc_tid(fd,0xff,&tid);
      
	  if(rc != VSTATUS_OK)
	    {
	      printf("ERROR: Could not allocate TID: %s\n",cs_convert_status(rc));
	    }
	  else
	    {
	  
	      printf("%d: Got tid %llx\n",i,(long long unsigned int)tid);
	  
	    }
#ifdef PAW
		// optional check to make sure nothing appears on Q
	  {
	    Mai_t mad;
	    
	    rc = mai_recv(fd,&mad,5000000);
	    
	    if(rc != VSTATUS_TIMEOUT)
	      {
	      printf("ERROR: otest: bad status %s\n",cs_convert_status(rc));
	      printf("otest:  shutting down.. mai_recv status\n");
	      IB_EXIT("otest",rc); 
	      return rc;
	      }
	  }
#endif
	}
  
      printf("otest: shutting down.. \n");
      rc =mai_close(fd);
      rc =mai_close(fd2);

      sleep(4); //need by simulator.. 
    }
  IB_EXIT("otest",rc); 
  return rc;

}
