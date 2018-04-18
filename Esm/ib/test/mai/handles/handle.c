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
 

#define MAX_FD     32
#define MAX_FILTERS 4096

#define FILTER_EXPOSED (0)
#define FILTER_METHOD  (1)
#define FILTER_AID     (2)
#define FILTER_AMOD    (3)
#define FILTER_ONCE    (4)
#define FILTER_ALL     (5)

static Filter_t all_filter[MAX_FILTERS];
static IBhandle_t all_handles[MAX_FILTERS];


extern void srand(unsigned int seed);
extern int rand(void);


int loop     = 100;
uint32_t ib_dev   = 0;
uint32_t ib_port  = 1;
int flags    = 0;
int fdcount = 4;   //number of handles to spreed test over
int delete_order=2;
int fmode=0;
int timeout = 2000000; //15 sececonds


int mads=0;
int threads;
// since we use INTERNAL, must have reader and writer in same process
int readers = 1;
int writer  = 1;

void ThreadStart(uint32_t argc, int8_t** argv);
void shutdown_test(int i);
int Reader(int idx, int count);
int Writer(void);


/*-------------------------------------------------------------------*
 * help - SYNTAX error message 
 *-------------------------------------------------------------------*/


void help(void)
{
  printf("\nhtest - Exercise filters in MAI\n");
  printf("Syntax: htest [ -d <ibdev> -p <ibport> -l <loop>  -f <flag> \n\t\t-c <handle> -m <filter mode> -h]\n\n");
  printf("        -d    Specify iba device to open\n");
  printf("        -p    Specify iba port  on the device\n");
  //printf("        -f    Specify exclusive (1) or shared (0)\n");
  printf("        -f    ignored\n");
  printf("        -l    Specify number of times to loop\n"); 
  printf("        -c    Specify number of handles to open (32 Max)\n");
  printf("        -m    Specify filter  to use\n");
  printf("                0 - exposed, 1 - method, 2 - AID, 3 - AMOD\n");
  //printf("        -w    Creater writer thread 0 -no 1= yes\n");
  printf("        -w    ignored\n");
  //printf("        -r    Creater reader threads 0 -no 1= yes\n");
  printf("        -r    ignored\n");
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


  while ((op = getopt(argc,argv,"l:L:d:D:p:P:f:F:c:C:m:M:w:W:r:R:hHvV?")) != EOF)
    {
      switch (op)
	{
	case 'h':
	case 'H':
	case '?':
	  help();
	  break;

	case 'L':
	case 'l':
	  loop = atoi(optarg);
	  break;

	case 'm':
	case 'M':
	  fmode = atoi(optarg);
	  break;
	case 'd':
	case 'D':
	  ib_dev = atoi(optarg);
	  break;
	case 'c':
	case 'C':
	  fdcount = atoi(optarg);
	  if(fdcount <= 0 || fdcount >= MAX_FD)
	    help();
	  break;

	case 'w':
	case 'W':
	  //writer = atoi(optarg);
	  break;
	  
	case 'r':
	case 'R':
	  //readers = atoi(optarg);
	  break;

	case 'p':
	case 'P':
	  ib_port = atoi(optarg);
	  break;

	case 'f':
	case 'F':
	  //flags = atoi(optarg);
	  break;	

	default:
	  help();
	  break;
	}
    }
}

#define MAX_THREADS (8)
#define STACK_SIZE (16*1024)
static Thread_t thandle[MAX_THREADS];
static uint8_t *thread_argv[MAX_THREADS][2];
int thread_cnt;

IBhandle_t fd[MAX_FD];
int mode;

int main(int argc, char *argv[])
{	
  int i,rc;
  Filter_t *fh;
  int max;


  if(argc > 1)
    parse_cmd_line(argc,argv); 

  srand(getpid());

  mai_init();

  rc=ib_init_devport(&ib_dev, &ib_port, NULL, NULL);
  if (rc)
    {
      printf("ERROR: ib_init_devport failed, %s\n",cs_convert_status(rc));
      exit(1);
    }


		
  for(i=0;i<fdcount;i++)
    {
      if ((rc = mai_open(0, ib_dev, ib_port, &fd[i])) != VSTATUS_OK) 
	{
	  printf("ERROR: can't open mai %s\n", cs_convert_status(rc));
	  exit(1);
	}
    }

 printf("\nINFO: PRE-TEST  layers charecteristics %d\n",0);



  mode = fmode;
      
      
      switch(mode)
	{
	case FILTER_EXPOSED:
	  printf("********** EXPOSED FILTER MODE  *************\n");
	  break;
	case FILTER_METHOD:
	  printf("********** METHOD MODE  *********************\n");
	  break;
	case FILTER_AID:
	  printf("********** AID MODE  ************************\n");
	  break;
	case FILTER_AMOD:
	  printf("********** AMOD MODE  ************************\n");
	  break;
	case FILTER_ONCE:
	  printf("********** ONCE MODE not supported  *************************\n");
	  exit(-1);
	  break;
	default:
	  printf("********** UNKOWN MODE  ***********************\n");
	  exit(-1);
	  break;
	}
  
      
      memset(&all_filter,0,sizeof(all_filter));
      max = fdcount;

      fh = &all_filter[0];
  
      //First we linearly create the filters and delete them in the order
      //they were created.

      if(readers)
	{
	  for(i=0;i<max;i++)
	    {
	      char name[32];

	      switch(mode)
		{
		case FILTER_EXPOSED:
		  {
		    IBhandle_t hd;

		    fh->dev = ib_dev;
		    fh->qp  = MAI_FILTER_ANY;
		    fh->port= ib_port;
		    fh->type = MAI_TYPE_ANY;

	      
		    fh->value.bversion = i%2;
		    fh->value.mclass   = i%256;
		    fh->value.cversion = i%256;
		    fh->value.method   = i%256;
		    fh->value.status   = i%(1<<16);
		    fh->value.hopPointer = i%64;
		    fh->value.hopCount   = i%64;
		    fh->value.tid        = i;
		    fh->value.aid        = i%(1<<16);
		    fh->value.amod       = i;

		    fh->mask.bversion = MAI_FMASK_ALL;
		    fh->mask.mclass   = MAI_FMASK_ALL;
		    fh->mask.cversion = MAI_FMASK_ALL;
		    fh->mask.method   = MAI_FMASK_ALL;
		    fh->mask.status   = MAI_FMASK_ALL;
		    fh->mask.hopPointer = MAI_FMASK_ALL;
		    fh->mask.hopCount   = MAI_FMASK_ALL;
		    fh->mask.tid        = MAI_FMASK_ALL;
		    fh->mask.aid        = MAI_FMASK_ALL;
		    fh->mask.amod       = MAI_FMASK_ALL;
  
	      
		    fh->active = (MAI_ACT_QP | MAI_ACT_DEV | MAI_ACT_TYPE | MAI_ACT_PORT |
				  MAI_ACT_FMASK);
	      
		    sprintf(name,"Fil %d",i);
		    MAI_SET_FILTER_NAME(fh,name);
	      
	      
		    //now create the filter
	      
		    rc = mai_filter_hcreate(fd[i%fdcount],fh,flags,&all_handles[i]);
	      
		    if(rc != VSTATUS_OK)
		      {
			printf("ERROR: %d - Unable to create filter\n",i);
			printf("ERROR: %s - mai_filter_create return status\n",cs_convert_status(rc));
			exit(-1);
		      }

		    //now get the handle and compare .. make sure they
		    //point to the same thing.

		    rc = mai_filter_handle(fd[i%fdcount],fh,flags,&hd);
	      
		    if(rc != VSTATUS_OK)
		      {
			printf("ERROR: %d - Unable to get filter handle\n",i);
			printf("ERROR: %s - mai_filter_handle return status\n",cs_convert_status(rc));
			exit(-1);
		      }

		    if(hd != all_handles[i])
		      {

			printf("ERROR: %d - Filter handles do not agree\n",i);
			exit(-1);
		      }

		    fh++;
		  }
		  break;
		case FILTER_METHOD:
		  {
		    rc = mai_filter_method(fd[i%fdcount],flags,MAI_TYPE_INTERNAL,&all_handles[i],i/256,i%256);
	    
		    if(rc != VSTATUS_OK)
		      {
			printf("ERROR: %d - Unable to create filter\n",i);
			printf("ERROR: %s - mai_filter_method return status\n",cs_convert_status(rc));
			exit(-1);
		      }
	    
		  }
		  break;
		case FILTER_AID:
		  {
		    rc = mai_filter_aid(fd[i%fdcount],flags,MAI_TYPE_INTERNAL,&all_handles[i],i/256,i%256,i);
	    
		    if(rc != VSTATUS_OK)
		      {
			printf("ERROR: %d - Unable to create filter\n",i);
			printf("ERROR: %s - mai_filter_aid return status\n",cs_convert_status(rc));
			exit(-1);
		      }
		  }
		  break;
		case FILTER_AMOD:
		  {
		    rc = mai_filter_amod(fd[i%fdcount],flags,MAI_TYPE_INTERNAL,&all_handles[i],i/256,i%256,i,i);
	    
		    if(rc != VSTATUS_OK)
		      {
			printf("ERROR: %d - Unable to create filter\n",i);
			printf("ERROR: %s - mai_filter_amod return status\n",cs_convert_status(rc));
			exit(-1);
		      }

		  }
		  break;
		case FILTER_ONCE:
		  {
		    rc = mai_filter_once(fd[i%fdcount],flags,MAI_TYPE_INTERNAL,&all_handles[i],i/256,i);
	    
		    if(rc != VSTATUS_OK)
		      {
			printf("ERROR: %d - Unable to create filter\n",i);
			printf("ERROR: %s - mai_filter_method return status\n",cs_convert_status(rc));
			exit(-1);
		      }
	    
		  }
		  break;
		default:
		  {
		    printf("ERROR: %d - Unknown filter test mode\n",mode);
		    exit(-1);	    
		  }
		}

	    }
	}


      if(writer)
	{
	  //start the writer thread
	  if(readers==0)
	    {
	      i=Writer();
	      shutdown_test(i);
	    }
	  else
	    {
	       //start the workert thread
	      rc = vs_thread_create(&thandle[thread_cnt],
				   (unsigned char*)"Writer",//name
				   (void (*) (uint32_t,uint8_t**)) Writer,   //func
				    0,                       //argc
				    NULL,                   //argptr
				    STACK_SIZE              //stacksize
				    );


	      
	      if(rc)
		{
		  printf("ERROR (%s): creating Writer thread\n",
			 cs_convert_status(rc));
		  exit(-1);
		} 
	      printf("\nINFO: Writer thread created %s\n",cs_convert_status(rc)); 
	      thread_cnt++;
	    }

	}


      if(readers)
	{
	  int threads=0;
	  int idx;
	  
	  if(fdcount > 2)
	    {
	      int threads;
	      //start 2 normal readers and then
	      //a wait_handle reader
	      if(fdcount == 3)
		{
		  threads = 1;
		  idx = 2;
		}
	      else
		{
		  threads = 2;
		  idx = fdcount - 2;
		}
	      //now start single handle readers 
	      for(i=0;i<threads;i++)
		{
	           thread_argv[thread_cnt][0] = (uint8_t*)(unsigned long)idx++;
	           thread_argv[thread_cnt][1] = (uint8_t*)(unsigned long)1;
		   rc = vs_thread_create(&thandle[thread_cnt],
					 (unsigned char*)"Reader",//name
					 (void (*) (uint32_t,uint8_t**))ThreadStart,   //func
					 2,                     //argc
					 (uint8_t **)&thread_argv[thread_cnt],     //argptr
					 STACK_SIZE             //stacksize
					 );


		   if(rc)
		    {
		      printf("ERROR (%s): creating reader thread\n", cs_convert_status(rc));
		      return -1;
		    } 
		   thread_cnt++;
		}
	    }	  
	  
	  
	  //now do multi handle recv
	  Reader(0,fdcount-threads);
	  while(threads);
	}

      shutdown_test(mads-loop);
      return 0;
}


void ThreadStart(uint32_t argc, int8_t** argv)
{
  Reader((unsigned long)argv[0], (unsigned long)argv[1]);
  threads--;
}

void shutdown_test(int val)
{

  int i,rc;

  

   //now close the handles 
  
  for(i=0;i<fdcount;i++)
    {
      if ((rc = mai_close(fd[i])) != VSTATUS_OK) 
	{
	  printf("ERROR: %s - can't close MAI\n", cs_convert_status(rc));
	  exit(-1);
	}
    }
  
#if 0	// code below is broken, used to test i, but val is not correct either
  if(val<0)
    {
      printf("\nERROR: Not all mads were received - increase timeout %d\n",mads);
      exit(-1);
    }
  else
#endif
    {
      printf("\nINFO: Test completed successfully %d\n",0);
      exit(0);
    }

}





/*
 * FUNCTION
 *      mai_wait_handle
 *
 * DESCRIPTION
 *      This function wait for message to be posted to any of  the handles 
 *      specified in argument. It returns the index of the lowest handle
 *      that has a message on it, if a message is received before timeout.
 *      This index is posted to the location the user passed in, pointed
 *      by the location pfirst.
 *
 * CALLED BY
 *
 * CALLS
 *
 *
 * INPUTS
 *      ha      Array of handles returned from mai_open
 *      count   Number of handles in array.
 *      timout  How long to wait. 0 means return immediately
 *      pfirst  Where to post the index handle with message on in 
 *      maip    Where to post the MAD recved on the handle
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *      VSTATUS_TIMEOUT
 *
 * HISTORY
 *      NAME      DATE          REMARKS
 *      PAW     01/21/01        Initial entry
 */

#define WAIT_HANDLE_TO  (20000)  /* 20 ms */

Status_t mai_wait_handle(IBhandle_t *ha, int count, 
		       uint64_t timeout, int *pfirst, Mai_t *maip)

{
  uint32_t          *p_to = (uint32_t *)&timeout;
  uint64_t          timenow;
  uint64_t          wakeup;
  int               rc,i;

  IB_ENTER("mai_wait_handle", ha, count, p_to[0], p_to[1]);
  (void)(p_to); // remove unused warning.
 
  
  if (ha    == NULL || pfirst == NULL ||
      count <=0     || maip   == NULL)
    {
      rc = VSTATUS_ILLPARM;
      goto done;
    }
  
  
  /* Update the absolute timeout value to wait for.			*/
  
  if(timeout == MAI_RECV_NOWAIT)
    {
      wakeup  = 0;
    }
  else
    {
      vs_time_get(&timenow);
      wakeup = timenow + timeout;
    }
       
 mai_wait_retry:  
  
  /* 
   * The handles are valid. Now Try and see if there is data
   * already waiting on any.
   */
  
  for(i=0;i<count;i++)
    {
      rc = mai_recv(ha[i],maip,MAI_RECV_NOWAIT);
      
      if (rc == VSTATUS_OK)
	{
	  *pfirst = i;
	  goto done;
	}           

      if(rc != VSTATUS_TIMEOUT)
	{
	  /* An error has occurred  */
	  printf("ERROR: mai_wait_handle: recv err on index %d\n",i);
	  goto done;
	}
    }
  
  

  /* 
   * If we get here then the handles did not have data on them. So let's 
   * proceed to the next stage of waiting.
   */
  
  /* See if we have reached (or exceeded timeout)	  	       */
  if(wakeup == 0)
    {
      rc =  VSTATUS_TIMEOUT;
      goto done;
    }
  else
    {
      vs_time_get(&timenow);
      if (timenow > wakeup)
	{
	  rc = VSTATUS_TIMEOUT;
	  goto done;
	}
    }
  
  /* Increment waiters count                                            */

  rc = mai_recv(ha[0],maip, WAIT_HANDLE_TO);
  
  if (rc == VSTATUS_OK)
    {
      *pfirst = 0;
      goto done;
    }       
  
  goto mai_wait_retry;     /* And try it again			*/
  
 done:
  IB_EXIT("mai_wait_handle",rc);
  return(rc); 
}



int Reader(int idx, int count)
{

  Mai_t in_mad;
  int i,rc;
  int nHandles=0;
  IBhandle_t hdl[MAX_FD];

  printf("\nINFO: Reader thread starting  idx %d\n",idx); 

  for(i=idx;i<(idx+count);i++)
    {
      hdl[nHandles++] = fd[i];
    }

  
  for(i=0;i<loop;i++)
    {
      int first=-1;

      memset(&in_mad,0,sizeof(Mai_t));

      if(nHandles==1)
	{

	  //Now that we have sent the Mad we should receive it on the handle      
	  rc = mai_recv(hdl[0],&in_mad,timeout);

	  if(rc == VSTATUS_TIMEOUT)
	    continue;

	  if(rc != VSTATUS_MAI_INTERNAL  &&  rc != VSTATUS_OK)
	    {
	      printf("ERROR:  - Unable to recv INTERNAL mad as expected %d\n",first);
	      printf("ERROR:  - mai_recv return status %s\n",cs_convert_status(rc));
	      exit(-1);
	    }
	  else
	    {
	      //we got a mad .. print the loop count
	      printf("-count -> %u\n", *(uint32_t*)in_mad.data);
	      mads++;
	      if(*(uint32_t*)in_mad.data == loop)
		{
		  printf("Final Mad received\n");
		  shutdown_test(mads-*(uint32_t*)in_mad.data);
		}	      
	    }
	}
      else
	{
	  rc = mai_wait_handle(hdl,nHandles,timeout,&first,&in_mad);
      
	  if(rc == VSTATUS_OK)
	    {

	      //Now that we have sent the Mad we should receive it on the handle      
	      //rc = mai_recv(fd[first],&in_mad,0);

	      if(rc != VSTATUS_MAI_INTERNAL  && rc != VSTATUS_OK)
		{
		  printf("ERROR:  - Unable to recv INTERNAL mad as expected %d\n",first);
		  printf("ERROR:  - mai_recv return status %s\n",cs_convert_status(rc));
		  exit(-1);
		} 

	      //we got a mad .. print the loop count
	      printf("+count -> %u\n", *(uint32_t*)in_mad.data);
	      mads++;
	      if(*(uint32_t*)in_mad.data == loop)
		{
		  printf("Final Mad received\n");
		  shutdown_test(mads-*(uint32_t*)in_mad.data);
		}
	    }
	  else
	    {
	      if(rc == VSTATUS_TIMEOUT)
		{
		  printf("\nINFO: wait returned %d\n",rc); 
		}
	      else
		{
		  printf("ERROR: %s - mai_wait_handle return status\n",cs_convert_status(rc)); 
		  exit(-1);
		}
	    }
	}
    }  

  rc = (mads - *(uint32_t*)in_mad.data);
  printf("\nINFO: Reader done .. returning %d\n",rc); 
  return rc;
}




int Writer()
{
  
  Mai_t out_mad,*fh;
  int i=0,rc,v;


  printf("\nINFO: Writer thread starting  loops %d\n",loop); 
  
  memset(&out_mad,0,sizeof(Mai_t));
      
  fh = &out_mad;
     
  fh->type = MAI_TYPE_INTERNAL;
  fh->active = (MAI_ACT_TYPE |  MAI_ACT_FMASK);
  
  fh->addrInfo.destqp = 0;
  // FIXME: should initialize rest of Mai_t, especially addrInfo
  // however since we are using INTERNAL, we get away without them

  for(v=0;v<loop;v++)
    {
      *(uint32_t*)fh->data = v+1;

      //vary the parameters up to the number of filters created
      i = v%fdcount;

      switch(mode)
	{
	case FILTER_EXPOSED:
	  {
	
	    fh->base.bversion = i%2;
	    fh->base.mclass   = i%256;
	    fh->base.cversion = i%256;
	    fh->base.method   = i%256;
	    fh->base.status   = i%(1<<16);
	    fh->base.hopPointer = i%64;
	    fh->base.hopCount   = i%64;
	    fh->base.tid        = i;
	    fh->base.aid        = i%(1<<16);
	    fh->base.amod       = i;
	      
	  }
	  break;
	case FILTER_METHOD:
	  {
	  
	    fh->base.mclass   = i/256;
	    fh->base.method   = i%256;
  	     
	  }
	  break;
	case FILTER_AID:
	  {
	  
	    fh->base.mclass   = i/256;
	    fh->base.method   = i%256;
	    fh->base.aid        = i;

	  }
	  break;
	case FILTER_AMOD:
	  {
	    
	    fh->base.mclass   = i/256;
	    fh->base.method   = i%256;
	    fh->base.aid      = i;
	    fh->base.amod     = i;	    
	  }
	  break;
	case FILTER_ONCE:
	  {
	    fh->base.mclass   = i/256;
	    fh->base.tid      = i;
 	
	  }
	}
      
      //Now we have the mad send it from any handle and we should receive it.
      
      printf("\nINFO: Sending  %d INTERNAL mad\n",i); 
       
      rc = mai_send(fd[(i+rand())%fdcount],&out_mad);
  
      if(rc != VSTATUS_OK)
	{
	  printf("ERROR: %d - Unable to send INTERNAL mad\n",i);
	  printf("ERROR: %s - mai_send return status\n",cs_convert_status(rc));
	  sleep(1);
	  //exit(-1);
	}      
    }
  
  printf("\nINFO: Writer done - sent  %d INTERNAL mads\n",i); 
  return i;  
}
