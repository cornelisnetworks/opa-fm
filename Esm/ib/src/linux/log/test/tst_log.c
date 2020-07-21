/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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
#include <stdio.h>
#include <unistd.h>
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"

#include "cs_log.h"

typedef void           (*threadfunc_t)(uint32_t argc, uint8_t *argv[]);

#include "cs_log.h"

#ifndef FALSE
#define FALSE (0)
#endif
				/* Quick and dirty string translation 	*/
struct lut
	{
	int	val;
	char	name[20];
	};


//void vs_fatal_error(uint8_t *string);
char *lvl2name(uint32_t );

#define vs_fatal_error(x)

/*
 * log level to name lookup
 */


struct lut ltn[9] = {
  {VS_LOG_OFF,   "VS_LOG_OFF"},
  {VS_LOG_FATAL, "VS_LOG_FATAL"},
  {VS_LOG_CSM_ERROR, "VS_LOG_CSM_ERROR"},
  {VS_LOG_CSM_WARN, "VS_LOG_CSM_WARN"},
  {VS_LOG_CSM_NOTICE, "VS_LOG_CSM_NOTICE"},
  {VS_LOG_CSM_INFO, "VS_LOG_CSM_INFO"},
  {VS_LOG_ERROR, "VS_LOG_ERROR"},
  {VS_LOG_WARN,  "VS_LOG_WARN"},
  {VS_LOG_NOTICE,  "VS_LOG_NOTICE"},
  {VS_LOG_INFINI_INFO,  "VS_LOG_INFINI_INFO"},
  {VS_LOG_VERBOSE,"VS_LOG_VERBOSE"},
  {VS_LOG_DATA,  "VS_LOG_DATA "},
  {VS_LOG_DEBUG1,"VS_LOG_DEBUG1"},
  {VS_LOG_DEBUG2,"VS_LOG_DEBUG2"},
  {VS_LOG_DEBUG3,"VS_LOG_DEBUG3"},
  {VS_LOG_DEBUG4,"VS_LOG_DEBUG4"},
  {VS_LOG_TRACE, "VS_LOG_TRACE"},
  {VS_LOG_ALL,   "VS_LOG_ALL  "}
};

char *error_message = "** ERROR **";

char *lvl2name(uint32_t lvl)
{
  int i;

  for(i=0;i<9;i++)
	if(lvl == ltn[i].val)
		return(ltn[i].name);
  return(error_message);
}



static int loops = 1;
static uint8_t data[1024];
/*
 * test_main_group - run a set of debug messages at a given level
 */

int test_main_group(uint32_t mask)
{
  char *msg = "This is a log very long @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ data message";
  int i;

  log_mask = mask;




  for(i=400;i<(1024-32);)    
    {
      printf("log length %d\n",i);
	IB_LOG_INFO("logging ",i);
	IB_LOG_DATA("data",data,i);
	i+=32;
    }

  
  printf("\n");
  printf("----------- Starting main test cycle with mask: %s ---------\n\n",
	lvl2name(mask));

  printf("Testing IB_ENTER       with mask %s\n",lvl2name(mask));
  IB_ENTER("testmod:",2,3,4,5);
  printf("Testing IB_ENTER       with mask %s\n",lvl2name(mask));
  IB_ENTER("averylongmodulename:",2,3,4,5);

  printf("Testing IB_EXIT        with mask %s\n",lvl2name(mask));
  IB_EXIT("Goodbyecruelworld",2);
  printf("Testing IB_FATAL_ERROR with mask %s\n",lvl2name(mask));
  IB_FATAL_ERROR("fatalexit:");

  printf("Testing IB_LOG_ERROR   with mask %s\n",lvl2name(mask));
  IB_LOG_ERROR("This is a log error message",2);
  
  printf("Testing IB_LOG_INFO    with mask %s\n",lvl2name(mask));
  IB_LOG_INFO("This is a log info message",2);
  printf("Testing IB_LOG_VERBOSE   with mask %s\n",lvl2name(mask));
  IB_LOG_VERBOSE("This is a log info one message",2);
  printf("Testing IB_LOG_WARN   with mask %s\n",lvl2name(mask));
  IB_LOG_WARN("This is a log info two message",2);
  printf("Testing IB_LOG_DATA    with mask %s\n",lvl2name(mask));
  IB_LOG_DATA("str","This is a log data message",27);

  
  IB_LOG_DATA("msg", msg,strlen(msg));
  
  return VSTATUS_OK;
}

/*
 * test_middle - test the logging calls using the middle macors
 */

int test_middle(uint32_t mask)
{

  log_mask = mask;

  printf("\n");
  printf("------------- Starting middle test cycle with mask: %s -----\n\n",
	lvl2name(mask));

  printf("Testing IB_LOG_ARGS1   with mask %s\n",lvl2name(mask));
  IB_LOG_ARGS1(1);
  printf("Testing IB_LOG_ARGS2   with mask %s\n",lvl2name(mask));
  IB_LOG_ARGS2(1,2);
  printf("Testing IB_LOG_ARGS3   with mask %s\n",lvl2name(mask));
  IB_LOG_ARGS3(1,2,3);
  printf("Testing IB_LOG_ARGS4   with mask %s\n",lvl2name(mask));
  IB_LOG_ARGS4(1,2,3,4);
  printf("Testing IB_LOG_ARGS5   with mask %s\n",lvl2name(mask));
  IB_LOG_ARGS5(1,2,3,4,5);

  return VSTATUS_OK;
}

/*
 * test_base - test the logging calls using the base macors
 */

int test_base(uint32_t mask)
{

  log_mask = mask;

  printf("\n");
  printf("----------- Starting base test cycle with mask: %s ---------\n\n",
	lvl2name(mask));


  printf("Testing VS_LOG_VDATA  with mask %s\n",lvl2name(mask));
  VS_LOG_VDATA(VS_LOG_INFO, "teststr", 22);

  printf("Testing VS_TRC_ARGS   with mask %s\n",lvl2name(mask));
  VS_TRC_ARGS(VS_LOG_ARGS, 2, 11,22,0,0,0,0,0);

  printf("Testing VS_TRC_VDATA  with mask %s\n",lvl2name(mask));
  VS_TRC_VDATA(VS_LOG_WARN, "teststr", 22);

  printf("Testing VS_TRC_ENTRY  with mask %s\n",lvl2name(mask));
  VS_TRC_ENTRY(VS_LOG_DATA, "teststr", 22, 33, 44, 55);

  return VSTATUS_OK;
}

static uint8_t data[1024];


void thread_log(void)
{
  int i;

  for(i=0; i<loops; i++)
    {
      test_main_group(0);
      test_main_group(VS_LOG_ALL);
      test_main_group(VS_LOG_FATAL);
      test_main_group(VS_LOG_ERROR);
      test_main_group(VS_LOG_WARN);
      test_main_group(VS_LOG_INFO);
      test_main_group(VS_LOG_VERBOSE);
      test_main_group(VS_LOG_DATA);
      test_main_group(VS_LOG_DEBUG1);
      test_main_group(VS_LOG_DEBUG2);
      test_main_group(VS_LOG_DEBUG3);
      test_main_group(VS_LOG_DEBUG4);
      test_main_group(VS_LOG_TRACE);
      
      test_middle(0);
      test_middle(VS_LOG_ALL);
      test_middle(VS_LOG_FATAL);
      test_middle(VS_LOG_ERROR);
      test_middle(VS_LOG_WARN);
      test_middle(VS_LOG_INFO);
      test_middle(VS_LOG_VERBOSE);
      test_middle(VS_LOG_DATA);
      test_middle(VS_LOG_DEBUG1);
      test_middle(VS_LOG_DEBUG2);
      test_middle(VS_LOG_DEBUG3);
      test_middle(VS_LOG_DEBUG4);
      test_middle(VS_LOG_TRACE);
      
      test_base(0);
      test_base(VS_LOG_ALL);
      test_base(VS_LOG_FATAL);
      test_base(VS_LOG_ERROR);
      test_base(VS_LOG_WARN);
      test_base(VS_LOG_INFO);
      test_base(VS_LOG_VERBOSE);
      test_base(VS_LOG_DATA);
      test_base(VS_LOG_DEBUG1);
      test_base(VS_LOG_DEBUG2);
      test_base(VS_LOG_DEBUG3);
      test_base(VS_LOG_DEBUG4);
      test_base(VS_LOG_TRACE);
    }

  printf("DONE....\n");


}

#if 0
#define USR_LOG_DEBUG
#endif

uint8_t stack[8096];

#ifdef USR_LOG_DEBUG
/* See vslog_l.h */
/*****************************************************************************
*
*    Function to write the formatted log data
*
*****************************************************************************/
void OutFormatDataCb(uint8_t *arg,  uint16_t len,  uint16_t dtype)
{
  int rc;


    rc = fwrite(arg,len,1,stdout);
  
  if(rc < 0 )
    {
      fprintf(stderr,"Error writing format out %d \n",rc);
    }
}


#endif

FILE    *stream;
uint32_t mask;
Thread_t thread;

int main(int argc, char *argv[])
{
  int j;

  log_mask =  VS_LOG_ALL;

  if(argc > 1)
    loops = atoi(argv[1]);


  test_base(VS_LOG_ALL);

#if 0
  if(argc > 2)
    {
      int i;

      j = atoi(argv[2]);
      j -= 1;


      for(i=0;i<j;i++)
	{	  
	    Thread_t thread;
	    
	    vs_thread_create(&thread,
			     NULL,                 //name
			     (threadfunc_t)thread_log,   //func
			     0,                    //argc
			     NULL,                  //argptr
			     0,                    //stacksize
			       VTHREAD_RUN);
	}  
      
    }

#endif
 
  thread_log();

#ifdef USR_LOG_DEBUG
  while(1)
#else
   for(j=0;j<6;j++)    
#endif
     sleep(5);

  
  return VSTATUS_OK;
}









