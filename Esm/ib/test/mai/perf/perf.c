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

static Filter_t all_filter[MAX_FILTERS];



extern void srand(unsigned int seed);
extern int rand(void);
//extern int atoi(char *nptr);


int loop     = 100;
uint32_t ib_dev   = 0;
uint32_t ib_port  = 1;
int flags    = 0;
int fdcount = 1;   //number of handles to spread test over
int delete_order=2;
int fmode=0;
int timeout = 2000000; //15 sececonds
int mclass = MAD_CV_SUBN_LR;
int mads=0;
int threads;
// since we use INTERNAL, must have reader and writer in same process
int readers = 1;
int writer  = 1;
int write_burst_count = 10,reader_print_rate=10;

void ThreadStart(uint32_t argc, int8_t** argv);
void shutdown_test(int i);
int Reader(int idx, int count);
int Writer(void);

#define DEBUG_PRINTF(format, args...) do { if (0) printf(format, ##args); } while (0)

/*-------------------------------------------------------------------*
 * help - SYNTAX error message 
 *-------------------------------------------------------------------*/


void help(void)
{
  printf("\nftest - Exercise filters in MAI\n");
  printf("Syntax: htest [ -d <ibdev> -p <ibport> -l <loop>  -f <flag> \n\t\t -m <mclass> -h]\n\n");
  printf("        -d    Specify iba device to open\n");
  printf("        -p    Specify iba port  on the device\n");
  //printf("        -f    Specify exclusive (1) or shared (0)\n");
  printf("        -f    ignored\n");
  printf("        -l    Specify number of times to loop\n"); 
  printf("        -m    Specify mclass to use\n");
  //printf("        -w    Creater writer thread 0 -no 1= yes\n");
  printf("        -w    ignored\n");
  //printf("        -r    Creater reader thread 0 -no 1= yes\n");
  printf("        -r    ignored\n");
  printf("        -q    How mads to send/receive before stats is printed\n");
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


  while ((op = getopt(argc,argv,"l:d:p:f:m:w:r:q:hvV?")) != EOF)
    {
      switch (op)
	{
	case 'h':
	case 'H':
	case '?':
	  help();
	  break;


	case 'l':
	  loop = atoi(optarg);
	  break;

	case 'm':
	  mclass = atoi(optarg);
	  break;

	case 'd':
	  ib_dev = atoi(optarg);
	  break;

	case 'w':
	  //writer = atoi(optarg);
	  break;
	  
	case 'r':
	  // The test can be run with multiple readers, however when this is
	  // done, each reader gets a copy of all the packets and respond
	  // so the writer exits before all the readers are done and many
	  // of the reader's responses are discarded
	  //readers = atoi(optarg);
	  break;

	case 'p':
	  ib_port = atoi(optarg);
	  break;

	case 'f':
	  //flags = atoi(optarg);
	  break;	

	case 'q':
	  write_burst_count =  reader_print_rate = atoi(optarg);
	  
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

  fdcount = readers + writer + 1;
		
  for(i=0;i<fdcount;i++)
    {
      if ((rc = mai_open(0, ib_dev, ib_port, &fd[i])) != VSTATUS_OK) 
	{
	  printf("ERROR: can't open mai (%s)\n", cs_convert_status(rc));
	  exit(1);
	}
    }

  mode = fmode;
      switch(mode)
	{
	case 0:
	case 1:
	case 2:
	  printf("********** SHARED FILTER MODE  *************\n");
	  break;
	default:
	  printf("********** UNKOWN MODE  ***********************\n");
	  exit(-1);
	  break;
	}
  
      
      memset(&all_filter,0,sizeof(all_filter));

      fh = &all_filter[0];
  
      //First we linearly create the filters and delete them in the order
      //they were created.

      if(readers)
	{
	  for(i=writer;i<writer+readers;i++)
	    {
	  
	      char name[32];
	      IBhandle_t h;

	      mode = 0;
		flags = VFILTER_SHARE;

              memset(fh,0,sizeof(*fh));
	      fh->dev =  MAI_FILTER_ANY;
	      fh->qp  = MAI_FILTER_ANY;
	      fh->port=  MAI_FILTER_ANY;
	      fh->type = MAI_TYPE_INTERNAL;
	      
	      fh->value.method   = 1;       /* One for response to sends */
	      fh->mask.method   = MAI_FMASK_ALL;
	      
	      fh->value.mclass   = mclass;
	      fh->mask.mclass    = MAI_FMASK_ALL;
	      
	      fh->active = (MAI_ACT_QP | MAI_ACT_DEV | MAI_ACT_TYPE | MAI_ACT_PORT |
			    MAI_ACT_FMASK);
	      
	      sprintf(name,"Reader");
	      MAI_SET_FILTER_NAME(fh,name);
	      
	      //now create the filter
	      DEBUG_PRINTF("create method=1 filter on fd %d\n", i);
	      rc = mai_filter_hcreate(fd[i],fh,flags,&h);

	       if(rc != VSTATUS_OK)
		 {
		   printf("ERROR: %d - Unable to listen filter\n",i);
		   printf("ERROR: %s - mai_filter_create return status\n",cs_convert_status(rc));
		   exit(-1);
		 }

	    }
	}

      if(writer)
	{
	  /* Set filter for responses */
	  char name[32];
	  IBhandle_t hd;

          memset(fh,0,sizeof(*fh));
	  fh->dev =  MAI_FILTER_ANY;
	  fh->qp  = MAI_FILTER_ANY;
	  fh->port=  MAI_FILTER_ANY;
	  fh->type = MAI_TYPE_INTERNAL;

	  sleep(1);

	  fh->value.method   = 2;       /* One for response to sends */
	  fh->mask.method    = MAI_FMASK_ALL;
  
	  fh->value.mclass   = mclass;
	  fh->mask.mclass    = MAI_FMASK_ALL;
	      
	  fh->active = (MAI_ACT_QP | MAI_ACT_DEV | MAI_ACT_TYPE | MAI_ACT_PORT |
			MAI_ACT_FMASK);
	      
	  sprintf(name,"Response");
	  MAI_SET_FILTER_NAME(fh,name);
	      
	  //now create the filter
	  DEBUG_PRINTF("create method=2 filter on fd %d\n", 0);
	  rc = mai_filter_hcreate(fd[0],fh,0,&hd);
	      
	  if(rc != VSTATUS_OK)
	    {
	      printf("ERROR: %d - Unable to response filter\n",i);
	      printf("ERROR: %s - mai_filter_create return status\n",cs_convert_status(rc));
	      exit(-1);
	    }

	  //start the writer thread
	  if(readers==0)
	    {
	       (void)Writer();
	      //shutdown_test(i);
	    }
	  else
	    {
	       //start the workert thread
	      rc = vs_thread_create(&thandle[thread_cnt],
				    (unsigned char*)"Writer",         //name
				   (void (*) (uint32_t,uint8_t**)) Writer,   //func
				    0,                       //argc
				    NULL,                       //argptr
				    STACK_SIZE              //stacksize
				    );
	      
	      if(rc)
		{
		  printf("ERROR (%s): creating Writer thread\n", cs_convert_status(rc));
		  exit(-1);
		}
	      thread_cnt++;

	      printf("\nINFO: Writer thread created\n");
	    }
	}

      if(readers)
	{
	      // start readers 1 to n, then this thread will become reader 0
	      for(i=1;i<readers;i++)
		{

 	           thread_argv[thread_cnt][0] = (uint8_t*)(unsigned long)writer+i;
 	           thread_argv[thread_cnt][1] = (uint8_t*)(unsigned long)1;
		   rc = vs_thread_create(&thandle[thread_cnt],
					 (unsigned char*)"Reader",         //name
					 (void (*) (uint32_t,uint8_t**))ThreadStart,   //func
					 2,                   //argc
					 (uint8_t **)&thread_argv[thread_cnt],     //argptr
					 STACK_SIZE             //stacksize
					 );

		  if(rc)
		    {
		      printf("ERROR (%s): creating reader thread\n", cs_convert_status(rc));
		      exit(-1);
		    } 
		  thread_cnt++;
		}

	      // start reader 0 in this thread
	      Reader(writer,1);
	}

      // shutdown_test(mads-loop);
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

  

      printf("\nINFO: POST TEST  layers characteristics\n");

   //now close the handles 
  
  for(i=0;i<fdcount;i++)
    {
      if ((rc = mai_close(fd[i])) != VSTATUS_OK) 
	{
	  printf("ERROR: %s - can't close MAI \n", cs_convert_status(rc));
	  exit(-1);
	}
    }
  
  if(i<0)
    {
      printf("ERROR: Not all mads were received - increase timeout %d\n",mads);
      exit(-1);
    }
  else
    {
      printf("\nINFO: Test completed successfully\n");
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

#define WAIT_HANDLE_TO  (500000)  /* 500 ms */

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
	  DEBUG_PRINTF("recv method=%d tid=%"PRIu64"\n", maip->base.method, maip->base.tid);
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
      DEBUG_PRINTF("recv method=%d tid=%"PRIu64"\n", maip->base.method, maip->base.tid);
      goto done;
    }       
  
  goto mai_wait_retry;     /* And try it again			*/
  
 done:
  IB_EXIT("mai_wait_handle",rc);
  return(rc); 
}


uint64_t  max,min,avg;
void perf_update(uint64_t rate)
{
  if(rate > max)
    max = rate;
  
  if(min == 0)
    min = rate;
  else
  if(rate != 0 && rate < min)
    min = rate;

  if(avg == 0)
    avg = rate;
  else
    avg = (avg + rate)/2;
}

void perf_print(const char* process, int idx)
{
  printf("\n%s idx %d:\n",process, idx);
  printf("Max rate : %llu mads/sec\n",(long long unsigned int)max);
  printf("Avg rate : %llu mads/sec\n",(long long unsigned int)avg);
  printf("Min rate : %llu mads/sec\n\n",(long long unsigned int)min);

}



// The reader will read packets from MAI fds idx to idx+count-1
// send send a response on MAI fd idx for each packet received
int Reader(int idx, int count)
{

  Mai_t in_mad;
  int rc,cnt=0;
  int burst;
  uint64_t  curtime,last_recv=0;

  printf("\nINFO: Reader thread starting  idx %d\n",idx); 

  burst= reader_print_rate;

  while(1)
    {
      int first;

      memset(&in_mad,0,sizeof(Mai_t));

	  if (count > 1)
	    rc = mai_wait_handle(&fd[idx],count,timeout,&first,&in_mad);
	  else
	    rc = mai_recv(fd[idx],&in_mad,timeout);
	  if(rc == VSTATUS_TIMEOUT) {
		break;	// end of test
	    } else if (rc != VSTATUS_OK) {
	      printf("ERROR: reader error in mai_recv: %s\n", cs_convert_status(rc));
	      exit(-1);
	  } else
	    {
	      uint64_t delta;
	      uint64_t rate;

	      DEBUG_PRINTF("recv method=%d tid=%"PRIu64"\n", in_mad.base.method, in_mad.base.tid);
	      vs_time_get(&curtime);
	      
	      if(last_recv)
		{
		  cnt++;

		  if(burst <= 0)
		    burst = reader_print_rate;

		  burst--; 
		  DEBUG_PRINTF("%d\n",burst);

		  if(burst == 0)
		    {
		      delta = curtime - last_recv;
		      
		      if(delta)
			{
			  // Convert to seconds
			  rate = (cnt*1000000)/delta;
			  
			  printf("arrival rate (idx %d): %llu %d-mads/sec\n",idx,(long long unsigned int)rate,cnt);
			  perf_update(rate);
			  last_recv = curtime;
			  cnt = 0;		  
			}
		    }
		}
	      else
		{
		  last_recv = curtime;
		  cnt = 0;		  
		}
	      
	      mads++;
	      in_mad.base.method = 2; // Set method to reponse
	      
	      rc = mai_send(fd[idx],&in_mad);
	      DEBUG_PRINTF("send method=%d tid=%"PRIu64"\n", in_mad.base.method, in_mad.base.tid);

	      if(rc != VSTATUS_OK)
		{
		  printf("ERROR: %d - Unable to send INTERNAL response mad\n",idx);
		  printf("ERROR: %s - mai_send return status\n",cs_convert_status(rc));
		  exit(-1);
		}      
	    }
    } 
  perf_print("Reader", idx);

  rc = (mads - in_mad.base.tid);
  printf("\nINFO: Reader done .. returning %d\n",rc); 
  return rc;
}




// The writer will write packets using MAI fds 0 to fdcount
// Once it has sent a complete burst of packets, it will wait for
// a burst of responses on MAI fd 0
int Writer()
{
  
  Mai_t out_mad,in_mad,*fh;
  int i=0,rc=0,v,q,cnt=0;
  uint64_t  curtime,last_recv=0;

  printf("\nINFO: Writer thread starting  loops %d\n",loop); 
  
  memset(&out_mad,0,sizeof(Mai_t));
      
  fh = &out_mad;
     
  fh->type = MAI_TYPE_INTERNAL;
  fh->active = (MAI_ACT_TYPE |  MAI_ACT_FMASK);
	  
  vs_time_get(&curtime);
  last_recv = curtime;

  for(v=0;v<loop;v++)
    {
      //send a burst of MADs
      for(q=0;q<write_burst_count;q++)
	{
	  fh->base.tid = v+q+1;

	  //vary the parameters up to the number of filters created
	  i = (v+q)%fdcount;

	  fh->base.mclass   = mclass;
	  fh->base.method   = 1;
    
	  // writer could use any MAI fd, but its more realistic for
	  // writer to use its single fd which it is also using for recv
	  //rc = mai_send(fd[(i+rand())%fdcount],&out_mad);
	  rc = mai_send(fd[0],&out_mad);
	  DEBUG_PRINTF("send method=%d tid=%"PRIu64"\n", out_mad.base.method, out_mad.base.tid);
  
	  if(rc != VSTATUS_OK)
	    {
	      printf("ERROR: %d - Unable to send INTERNAL mad\n",i);
	      printf("ERROR: %s - mai_send return status\n",cs_convert_status(rc));
	      exit(-1);
	    }      
	}

      // now wait for response

      vs_time_get(&curtime);

      for(q=0;q<write_burst_count;q++)
	{

	  do{
	    //Now that we have sent the Mad we should receive it on the handle      
	    rc = mai_recv(fd[0],&in_mad,timeout);
	    
	    if(rc == VSTATUS_TIMEOUT)
	      {
		printf("writer: time out waiting for resp\n"); 
		continue;
	      }
	  }while(rc==VSTATUS_TIMEOUT);
	  
	  if(rc != VSTATUS_OK)
	    break;
	  DEBUG_PRINTF("recv method=%d tid=%"PRIu64"\n", in_mad.base.method, in_mad.base.tid);
	}

      if(rc == VSTATUS_OK)
	{
	      uint64_t delta;
	      uint64_t rate;

	      vs_time_get(&curtime);
	      
	      if(last_recv)
		{
		  cnt += write_burst_count;

		  delta = curtime - last_recv;
		  
		  if(delta)
		    {
		      // Convert to seconds
		      rate = (cnt*1000000)/delta;
		      
		      perf_update(rate);
		      printf("burst turn around rate : %llu (%d-mads-burst)/sec\n",(long long unsigned int)rate,write_burst_count);
		      
		      last_recv = curtime;
		      cnt = 0;		  
		    }
		}
	      else
		{
		  last_recv = curtime;
		  cnt = 0;		  
		}
	}
    }

  perf_print("Writer", 0);
  return 0;
}
