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

#ifdef __LINUX__
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#endif

#include "ib_types.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_g.h"
#include "mai_g.h"
#include "mal_g.h"

#define MAX_LAYERS 5
#define MAX_FD     32
#define MAX_DC     32
#define MAX_FILT   32
#define MAX_FILTERS 512

#define FILTER_EXPOSED (0)
#define FILTER_METHOD  (1)
#define FILTER_AID     (2)
#define FILTER_AMOD    (3)
#define FILTER_ONCE    (4)
#define FILTER_ALL     (5)

#define FORWARD_LINEAR   (0)
#define REVERSE_LINEAR   (1)
#define RANDOM           (2)
#define MAX_DELETE_ORDER (3)

#define MAIN main
#define EXIT(a) exit(a)

static Filter_t all_filter[MAX_FILTERS];
static IBhandle_t all_handles[MAX_FILTERS];
static int      inactive[MAX_FILTERS];	// filters that have been deleted

extern void     srand(unsigned int seed);
extern int      rand(void);

int             lp = 1;
uint32_t             dev = 0;
uint32_t             port = 1;
int             flags = 0;

int             fdcount = 4;	// number of handles to spreed test over
int             delete_order = 2;
int             fmode = 0;
uint64_t        timeout = 2000000;	// 500 milliseconds
uint64_t        scale = 1;

/*-------------------------------------------------------------------*
 * help - SYNTAX error message 
 *-------------------------------------------------------------------*/

int
help(void)
{
    printf("\nftest - Exercise filters in MAI\n");
    printf
	("Syntax: ftest [ -d <ibdev> -p <ibport> -l <loop>  -f <flag> \n\t\t-c <handle> -m <filter mode> -h]\n\n");
    printf("        -d    Specify iba device to open\n");
    printf("        -p    Specify iba port  on the device\n");
    printf("        -f    Bit or of  shared (0) and purge (2) ie 0,3 \n");
    printf("        -l    Specify number of times to loop\n");
    printf("        -c    Specify number of handles to open (31 Max)\n");
    printf("        -m    Specify filter test to use\n");
    printf("                0 - exposed, 1 - method, 2 - AID, 3 - AMOD\n");
    printf("                4 - once, 5 - all\n");
    printf("        -g    Specify debug level in format 0xffff\n");
    printf("        -h    Display this help message\n\n");

    EXIT(1);
}

/*-------------------------------------------------------------------*
	parse_cmd_line - Get command line args
 *-------------------------------------------------------------------*/

void
parse_cmd_line(int argc, char **argv)
{
    int             op;		/* option return from getopt */
    extern char    *optarg;

    while ((op =
	    getopt(argc, argv,
		   "l:L:d:D:p:P:f:F:c:C:m:M:g:hHvV?")) != EOF)
      {
	  switch (op)
	    {
	    case 'h':
	    case 'H':
	    case '?':
		help();
		break;

	    case 'g':
		{
		    int             v;

		    sscanf(optarg, "0x%08x", (uint32_t *) & v);
		    IB_SET_LOG_MASK(v);
		}
		break;
	    case 'L':
	    case 'l':
		lp = atoi(optarg);
		if (lp != 1)
		  {
		      printf
			  ("Loop value of one is the only value currently supported\n");
		      help();
		  }
		break;

	    case 'm':
	    case 'M':
		fmode = atoi(optarg);
		break;
	    case 'd':
	    case 'D':
		dev = atoi(optarg);
		break;
	    case 'c':
	    case 'C':
		fdcount = atoi(optarg);
		if (fdcount < 1 || fdcount >= MAX_FD)
		    help();
		break;

	    case 'p':
	    case 'P':
		port = atoi(optarg);
		break;

	    case 'f':
	    case 'F':
		flags = atoi(optarg);
		break;

	    default:
		help();
		break;
	    }
      }
}

int
MAIN(int argc, char *argv[])
{

    IBhandle_t      fd[MAX_FD];
    int             i,
                    rc;
    Filter_t       *fh,
                    ft;
    int             max = MAX_FILTERS;
    int             l;
    int             v;
    int             mode;
    Mai_t           out_mad,
                    in_mad;

    if (argc > 1)
	parse_cmd_line(argc, argv);


    srand(getpid());
    printf("FILTER: calling mai_init\n");
    mai_init();
    if (max >= mai_get_max_filters())
	max=mai_get_max_filters();
    else
      {
	printf("ERROR: %d - Max filters to to create too small\n", max);
	printf("ERROR: %d - MAI Max filters\n", mai_get_max_filters());
        EXIT(1);
      }

    rc=ib_init_devport(&dev, &port, NULL, NULL);
    if (rc)
      {
        printf("ERROR: ib_init_devport failed, %s\n",cs_convert_status(rc));
        EXIT(1);
      }

    /*
     * Determine how many handles to open 
     */
    if (flags & VFILTER_PURGE)
      {
	  /*
	   * We will use handle fd[fdcount] to do receives without filters
	   * This way we can pull in mads into the MAI without 
	   * actually picking them up. When the purge filter is subsequently
	   * deleted the data for the filter should be
	   * deleted silently - if the filter is the only filter matching the data
	   */
	  l = fdcount + 1;
      }
    else
      l = fdcount;
     
    if (l<1)
      {
      printf("ERROR: fdcount == 0.\n");
      EXIT(1);
      }

    for (i = 0; i < l; i++)
      {
	  printf("FILTER: calling mai_open dev:%d port:%d\n", dev, port);
	  if ((rc = mai_open(0, dev, port, &fd[i])) != VSTATUS_OK)
	    {
		printf("ERROR: can't open mai (%s)\n", cs_convert_status(rc));
		EXIT(1);
	    }
      }

    for (l = 0; l < lp; l++)
      {
	  printf("******** Executing loop pass %d **************\n", lp);

	  if (fmode == FILTER_ALL)
		mode = l % FILTER_ALL;
	  else
		mode = fmode;

	  switch (mode)
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
		printf("********** ONCE MODE  *************************\n");
		break;
	    default:
		printf("********** UNKOWN MODE  ***********************\n");
		EXIT(-1);
		break;
	    }

	  memset(&ft, 0, sizeof(ft));
	  /*
	   * first we test what is suppose to be correct behavior 
	   */

	  // With no active flag the create should fail 

	  rc = mai_filter_create(fd[0], &ft, flags);

	  if (rc == VSTATUS_OK)
	    {
		printf("ERROR: Filter Create passed on empty struct\n");
		EXIT(-1);
	    }

	  ft.qp = 2;
	  ft.active = MAI_ACT_QP;

	  rc = mai_filter_create(fd[0], &ft, flags);

	  if (rc == VSTATUS_OK)
	    {
		printf("ERROR: %s - Filter Create passed on invalid QP\n",
		     cs_convert_status(rc));
		EXIT(-1);
	    }

	  ft.dev = MAI_MAX_DEV + 1;
	  ft.active = MAI_ACT_DEV;

	  rc = mai_filter_create(fd[0], &ft, flags);

	  if (rc == VSTATUS_OK)
	    {
		printf("ERROR: %s - Filter Create passed on invalid DEV\n",
		     cs_convert_status(rc));
		EXIT(-1);
	    }

	  ft.port = MAI_MAX_PORT + 1;
	  ft.active = MAI_ACT_PORT;

	  rc = mai_filter_create(fd[0], &ft, flags);

	  if (rc == VSTATUS_OK)
	    {
		printf("ERROR: %s - Filter Create passed on invalid PORT\n",
		     cs_convert_status(rc));
		EXIT(-1);
	    }

	  printf("INFO: *****  Basic Filter validation passed *******\n\n");

	  // Now that we know that filters work .. start the test suite

	  memset(&all_filter, 0, sizeof(all_filter));

	  fh = &all_filter[0];

	  // First we linearly create the filters and delete them in the
	  // order
	  // they were created.

	  for (i = 0; i < max; i++)
	    {
		char            name[32];

		switch (mode)
		  {
		  case FILTER_EXPOSED:
		      {
			  IBhandle_t      hd;

			  fh->dev = dev;
			  fh->qp = MAI_FILTER_ANY;
			  fh->port = port;
			  fh->type = MAI_TYPE_ANY;

			  fh->value.bversion = i % 2;
			  fh->value.mclass = i % 256;
			  fh->value.cversion = i % 256;
			  fh->value.method = i % 256;
			  fh->value.status = i % (1 << 16);
			  fh->value.hopPointer = i % 64;
			  fh->value.hopCount = i % 64;
			  fh->value.tid = i;
			  fh->value.aid = i % (1 << 16);
			  fh->value.amod = i;

			  fh->mask.bversion = MAI_FMASK_ALL;
			  fh->mask.mclass = MAI_FMASK_ALL;
			  fh->mask.cversion = MAI_FMASK_ALL;
			  fh->mask.method = MAI_FMASK_ALL;
			  fh->mask.status = MAI_FMASK_ALL;
			  fh->mask.hopPointer = MAI_FMASK_ALL;
			  fh->mask.hopCount = MAI_FMASK_ALL;
			  fh->mask.tid = MAI_FMASK_ALL;
			  fh->mask.aid = MAI_FMASK_ALL;
			  fh->mask.amod = MAI_FMASK_ALL;

			  fh->active =
			      (MAI_ACT_QP | MAI_ACT_DEV | MAI_ACT_TYPE |
			       MAI_ACT_PORT | MAI_ACT_FMASK);

			  sprintf(name, "Fil %d", i);
			  MAI_SET_FILTER_NAME(fh, name);

			  // now create the filter

			  rc = mai_filter_hcreate(fd[i % fdcount], fh,
						  flags, &all_handles[i]);

			  if (rc != VSTATUS_OK)
			    {
				printf("ERROR: %d - Unable to create filter\n",
				     i);
				printf("ERROR: %s - mai_filter_create return status\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }

			  // now get the handle and compare .. make sure
			  // they
			  // point to the same thing.

			  rc = mai_filter_handle(fd[i % fdcount], fh,
						 flags, &hd);

			  if (rc != VSTATUS_OK)
			    {
				printf("ERROR: %d - Unable to get filter handle \n",
				     i);
				printf("ERROR: %s - mai_filter_handle return status\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }

			  if (hd != all_handles[i])
			    {
			      
			      printf("ERROR: %d - Filter handles do not agree %d != %d\n",
				      i,(int)hd,(int)all_handles[i]);
			      EXIT(-1);
			    }

			  fh++;
		      }
		      break;
		  case FILTER_METHOD:
		      {
			  rc = mai_filter_method(fd[i % fdcount], flags,
						 MAI_TYPE_INTERNAL,
						 &all_handles[i], i / 256,
						 i % 256);

			  if (rc != VSTATUS_OK)
			    {
				printf("ERROR: %d - Unable to create filter\n",
				     i);
				printf("ERROR: %s - mai_filter_method return status\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }

		      }
		      break;
		  case FILTER_AID:
		      {
			  rc = mai_filter_aid(fd[i % fdcount], flags,
					      MAI_TYPE_INTERNAL,
					      &all_handles[i], i / 256,
					      i % 256, i);

			  if (rc != VSTATUS_OK)
			    {
				printf("ERROR: %d - Unable to create filter\n",
				     i);
				printf("ERROR: %s - mai_filter_aid return status\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }
		      }
		      break;
		  case FILTER_AMOD:
		      {
			  rc = mai_filter_amod(fd[i % fdcount], flags,
					       MAI_TYPE_INTERNAL,
					       &all_handles[i], i / 256,
					       i % 256, i, i);

			  if (rc != VSTATUS_OK)
			    {
				printf("ERROR: %d - Unable to create filter\n",
				     i);
				printf("ERROR: %s - mai_filter_amod return status\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }

		      }
		      break;
		  case FILTER_ONCE:
		      {
			  rc = mai_filter_once(fd[i % fdcount], flags,
					       MAI_TYPE_INTERNAL,
					       &all_handles[i], i / 256,
					       i);

			  if (rc != VSTATUS_OK)
			    {
				printf("ERROR: %d - Unable to create filter\n",
				     i);
				printf("ERROR: %s - mai_filter_method return status\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }

		      }
		      break;
		  default:
		      {
			  printf("ERROR: %d - Unknown filter test mode\n",
			       mode);
			  EXIT(-1);
		      }
		  }

	    }

	  printf("\n\n************* Filter create phase 1 passed ***********\n\n");

	  // We get here then we have max out the filters on the system. 
	  // Creating a distinct filter here should now fail.

	  switch (mode)
	    {
	    case FILTER_EXPOSED:
		{
		    char            name[32];

		    fh->dev = dev;
		    fh->qp = MAI_FILTER_ANY;
		    fh->port = port;
		    fh->type = MAI_TYPE_ANY;

		    fh->value.bversion = i % 2;
		    fh->value.mclass = i % 256;
		    fh->value.cversion = i % 256;
		    fh->value.method = i % 256;
		    fh->value.status = i % (1 << 16);
		    fh->value.hopPointer = i % 64;
		    fh->value.hopCount = i % 64;
		    fh->value.tid = i;
		    fh->value.aid = i % (1 << 16);
		    fh->value.amod = i;

		    fh->mask.bversion = MAI_FMASK_ALL;
		    fh->mask.mclass = MAI_FMASK_ALL;
		    fh->mask.cversion = MAI_FMASK_ALL;
		    fh->mask.method = MAI_FMASK_ALL;
		    fh->mask.status = MAI_FMASK_ALL;
		    fh->mask.hopPointer = MAI_FMASK_ALL;
		    fh->mask.hopCount = MAI_FMASK_ALL;
		    fh->mask.tid = MAI_FMASK_ALL;
		    fh->mask.aid = MAI_FMASK_ALL;
		    fh->mask.amod = MAI_FMASK_ALL;

		    fh->active =
			(MAI_ACT_QP | MAI_ACT_DEV | MAI_ACT_TYPE |
			 MAI_ACT_PORT | MAI_ACT_FMASK);

		    sprintf(name, "Fil %d", i);
		    MAI_SET_FILTER_NAME(fh, name);

		    // now create the filter

		    rc = mai_filter_hcreate(fd[i % fdcount], fh, flags,
					    &all_handles[i]);

		    if (rc == VSTATUS_OK)
		      {
			  printf
			      ("ERROR: %d - Excess Create filter passed .. should fail\n",
			       i);
			  printf
			      ("ERROR: %s - mai_filter_create return status\n",
			       cs_convert_status(rc));
			  EXIT(-1);
		      }

		}
		break;
	    case FILTER_METHOD:
		{
		    rc = mai_filter_method(fd[i % fdcount], flags,
					   MAI_TYPE_INTERNAL, &all_handles[i],
					   i / 256, i % 256);

		    if (rc == VSTATUS_OK)
		      {
			  printf
			      ("ERROR: %d - Excess  create filter  passed ..should fail\n",
			       i);
			  printf
			      ("ERROR: %s - mai_filter_method return status\n",
			       cs_convert_status(rc));
			  EXIT(-1);
		      }

		}
		break;
	    case FILTER_AID:
		{
		    rc = mai_filter_aid(fd[i % fdcount], flags,
					MAI_TYPE_INTERNAL, &all_handles[i],
					i / 256, i % 256, i);

		    if (rc == VSTATUS_OK)
		      {
			  printf
			      ("ERROR: %d - Excess  create filter  passed ..should fail\n",
			       i);
			  printf
			      ("ERROR: %s - mai_filter_aid return status\n",
			       cs_convert_status(rc));
			  EXIT(-1);
		      }
		}
		break;
	    case FILTER_AMOD:
		{
		    rc = mai_filter_amod(fd[i % fdcount], flags,
					 MAI_TYPE_INTERNAL, &all_handles[i],
					 i / 256, i % 256, i, i);

		    if (rc == VSTATUS_OK)
		      {
			  printf
			      ("ERROR: %d - Excess  create filter  passed ..should fail\n",
			       i);
			  printf
			      ("ERROR: %s - mai_filter_amod return status\n",
			       cs_convert_status(rc));
			  EXIT(-1);
		      }

		}
		break;
	    case FILTER_ONCE:
		{
		    rc = mai_filter_once(fd[i % fdcount], flags,
					 MAI_TYPE_INTERNAL, &all_handles[i],
					 i / 256, i);

		    if (rc == VSTATUS_OK)
		      {
			  printf
			      ("ERROR: %d - Excess  create filter  passed ..should fail\n",
			       i);
			  printf
			      ("ERROR: %s - mai_filter_method return status\n",
			       cs_convert_status(rc));
			  EXIT(-1);
		      }

		}
	    }

	  // We get here the system behaved properly..
	  printf
	      ("\n\n************* Filter create phase 2 passed ***********\n\n"
	       );

	  // Now try and create duplicates of one of the filters ..
	  // Since the the filters are identical, it should pass.

	  // Chose on of the filters to duplicate
	  i = rand() % max;

	  printf("INFO: Testing DUPLICATING filter %d\n", i);

	  switch (mode)
	    {
	    case FILTER_EXPOSED:
		{
		    IBhandle_t      hd;

		    fh = &all_filter[i];

		    // now create the filter

		    rc = mai_filter_hcreate(fd[i % fdcount], fh, flags,
					    &hd);

		    if (rc != VSTATUS_OK)
		      {
			  printf
			      ("ERROR: %d - Unable to create duplicate filter\n",
			       i);
			  printf
			      ("ERROR: %s - mai_filter_create return status\n",
			       cs_convert_status(rc));
			  EXIT(-1);
		      }

		    // now get the handle and compare .. make sure they
		    // point to the same thing.

		    if (hd != all_handles[i])
		      {

			  printf
			      ("ERROR: %d - Filter handles do not agree\n",
			       i);
			  EXIT(-1);
		      }

		}
		break;
	    case FILTER_METHOD:
		{
		    IBhandle_t      hd;

		    rc = mai_filter_method(fd[i % fdcount], flags,
					   MAI_TYPE_INTERNAL, &hd, i / 256,
					   i % 256);

		    if (rc != VSTATUS_OK)
		      {
			  printf
			      ("ERROR: %d - Unable to create duplicate filter\n",
			       i);
			  printf
			      ("ERROR: %s - mai_filter_method return status\n",
			       cs_convert_status(rc));
			  EXIT(-1);
		      }

		    if (hd != all_handles[i])
		      {

			  printf
			      ("ERROR: %d - Filter handles do not agree\n",
			       i);
			  EXIT(-1);
		      }
		}
		break;
	    case FILTER_AID:
		{
		    IBhandle_t      hd;

		    rc = mai_filter_aid(fd[i % fdcount], flags,
					MAI_TYPE_INTERNAL, &hd, i / 256,
					i % 256, i);

		    if (rc != VSTATUS_OK)
		      {
			  printf
			      ("ERROR: %d - Unable to create duplicate filter\n",
			       i);
			  printf
			      ("ERROR: %s - mai_filter_aid return status\n",
			       cs_convert_status(rc));
			  EXIT(-1);
		      }

		    if (hd != all_handles[i])
		      {

			  printf
			      ("ERROR: %d - Filter handles do not agree\n",
			       i);
			  EXIT(-1);
		      }
		}
		break;
	    case FILTER_AMOD:
		{
		    IBhandle_t      hd;

		    rc = mai_filter_amod(fd[i % fdcount], flags,
					 MAI_TYPE_INTERNAL, &hd, i / 256,
					 i % 256, i, i);

		    if (rc != VSTATUS_OK)
		      {
			  printf
			      ("ERROR: %d - Unable to create duplicate filter\n",
			       i);
			  printf
			      ("ERROR: %s - mai_filter_amod return status\n",
			       cs_convert_status(rc));
			  EXIT(-1);
		      }

		    if (hd != all_handles[i])
		      {

			  printf
			      ("ERROR: %d - Filter handles do not agree\n",
			       i);
			  EXIT(-1);
		      }
		}
		break;
	    case FILTER_ONCE:
		{
		    IBhandle_t      hd;

		    rc = mai_filter_once(fd[i % fdcount], flags,
					 MAI_TYPE_INTERNAL, &hd, i / 256, i);

		    if (rc != VSTATUS_OK)
		      {
			  printf
			      ("ERROR: %d - Unable to create duplicate filter\n",
			       i);
			  printf
			      ("ERROR: %s - mai_filter_method return status\n",
			       cs_convert_status(rc));
			  EXIT(-1);
		      }
		    if (hd != all_handles[i])
		      {

			  printf
			      ("ERROR: %d - Filter handles do not agree\n",
			       i);
			  EXIT(-1);
		      }
		}
	    }

	  // We get here then the duplicate worked 

	  // Now delete one instance of the filter that was duplicated

	  rc = mai_filter_hdelete(fd[i % fdcount],all_handles[i]);

	  if (rc != VSTATUS_OK)
	    {
		printf
		    ("ERROR: %d - Unable to delete duplicate filter\n",
		     i);
		printf
		    ("ERROR: %s - mai_filter_hdelete return status\n",
		     cs_convert_status(rc));
		EXIT(-1);
	    }
	  else
	    {
		printf
		    ("INFO:  Duplicate filter passed successsfully %d\n",
		     i);
	    }

	  // We get here the system behaved properly..
	  printf
	      ("\n\n************* Filter create phase %d passed ***********\n\n",
	       3);

	  // Whe need to test that the filters are indeed active by
	  // sending
	  // an INTERNAL message that the filter describes.
	  // We should then receive the message

	  delete_order = (rand()) % MAX_DELETE_ORDER;
	  // delete_order= (l)%MAX_DELETE_ORDER;
	  // delete_order= RANDOM;

	  switch (delete_order)
	    {
	    case FORWARD_LINEAR:
		printf
		    ("INFO: %d ***** Entering phase 4 with FORWARD linear delete order ****\n",
		     delete_order);
		break;
	    case REVERSE_LINEAR:
		printf
		    ("INFO: %d **** Entering phase 4 with REVERSE linear delete order ****\n",
		     delete_order);
		break;
	    case RANDOM:
		printf
		    ("INFO: %d **** Entering phase 4 with RANDOM  delete order ****\n",
		     delete_order);
		break;
	    }

	  // chose an arbitrary starting point
	  i = rand() % max;

	  memset(&inactive, 0, sizeof(inactive));

	  for (v = 0; v < max; v++)
	    {
		Mai_t          *fh;

		memset(&out_mad, 0, sizeof(Mai_t));
		memset(&in_mad, 0, sizeof(Mai_t));

		// figure out which filter to delete first
		switch (delete_order)
		  {
		  case FORWARD_LINEAR:
		      {
			  i++;
			  if (i >= max)
			      i = 0;
			  // this should not be set ...
			  if (inactive[i])
			    {
				printf
				    ("ERROR: %d - Test all messed up...\n",
				     i);
				EXIT(-1);
			    }
			  inactive[i] = 1;
		      }
		      break;
		  case REVERSE_LINEAR:
		      {
			  i--;
			  if (i < 0)
			      i = max - 1;

			  // this should not be set ...
			  if (inactive[i])
			    {
				printf
				    ("ERROR: %d - Test all messed up...\n",
				     i);
				EXIT(-1);
			    }
			  inactive[i] = 1;
		      }
		      break;
		  case RANDOM:
		      {
			  int             start,
			                  dir;

			  start = rand() % max;
			  dir = rand() % 2;

			  // first chose a random starting point
			  // Then chose a random direction
			  if (dir)
			    {
				// search left.
				for (i = start; i >= 0; i--)
				  {
				      if (!inactive[i])
					  break;
				  }

				if (i < 0)
				  {
				      // try the othe direction
				      // search right.
				      for (i = start; i < max; i++)
					{
					    if (!inactive[i])
						break;
					}
				      if (i >= max)
					{
					    printf
						("ERROR: %d - Random Test all messed up...\n",
						 i);
					    EXIT(-1);
					}
				  }
			    }
			  else
			    {

				// search right.
				for (i = start; i < max; i++)
				  {
				      if (!inactive[i])
					  break;
				  }

				if (i >= max)
				  {
				      // try the othe direction
				      // search left.
				      for (i = start; i >= 0; i--)
					{
					    if (!inactive[i])
						break;
					}
				      if (i < 0)
					{
					    printf
						("ERROR: %d - Random Test all messed up...\n",
						 i);
					    EXIT(-1);
					}

				  }
			    }

			  inactive[i] = 1;
		      }
		      break;
		  }

		if (flags & VFILTER_PURGE)
		  {
		      printf(" .. testing filter %i with PURGE\n", i);
		  }
		else
		  {
		      printf(" .. testing filter %d\n", i);
		  }
		// send the mad filter

		fh = &out_mad;

		fh->type = MAI_TYPE_INTERNAL;
		fh->active = (MAI_ACT_TYPE | MAI_ACT_FMASK);

		switch (mode)
		  {
		  case FILTER_EXPOSED:
		      {

			  fh->base.bversion = i % 2;
			  fh->base.mclass = i % 256;
			  fh->base.cversion = i % 256;
			  fh->base.method = i % 256;
			  fh->base.status = i % (1 << 16);
			  fh->base.hopPointer = i % 64;
			  fh->base.hopCount = i % 64;
			  fh->base.tid = i;
			  fh->base.aid = i % (1 << 16);
			  fh->base.amod = i;

		      }
		      break;
		  case FILTER_METHOD:
		      {

			  fh->base.mclass = i / 256;
			  fh->base.method = i % 256;

		      }
		      break;
		  case FILTER_AID:
		      {

			  fh->base.mclass = i / 256;
			  fh->base.method = i % 256;
			  fh->base.aid = i;

		      }
		      break;
		  case FILTER_AMOD:
		      {

			  fh->base.mclass = i / 256;
			  fh->base.method = i % 256;
			  fh->base.aid = i;
			  fh->base.amod = i;
		      }
		      break;
		  case FILTER_ONCE:
		      {
			  fh->base.mclass = i / 256;
			  fh->base.tid = i;

		      }
		  }

		// Now we have the mad send it from any handle and we
		// should receive it.

		rc = mai_send(fd[(i + rand()) % fdcount], &out_mad);

		if (rc != VSTATUS_OK)
		  {
		      printf("ERROR: %d - Unable to send INTERNAL mad\n",
				i);
		      printf("ERROR: %s - mai_send return status\n",
				cs_convert_status(rc));
		      EXIT(-1);
		  }

		// when once and purge are both set, the MAD upon receipt
		// deletes the filter, which in turn purges the input queue
		// before delivering the MAD via mai_recv, so in this one
		// odd case, we will not receive the MAD.
		// The combination of ONCE and PURGE is not used by FM
		// but we can test it for completeness
		if ((! (flags & VFILTER_PURGE)) || mode != FILTER_ONCE)
		{
		// Now that we have sent the Mad we should receive it on
		// the handle 
		rc = mai_recv(fd[i % fdcount], &in_mad, timeout * scale);

		if (rc != VSTATUS_OK)
		  {
		      printf
			  ("ERROR: %d - Unable to recv  INTERNAL mad as expected\n",
			   i);
		      printf("ERROR: %s - mai_recv return status\n",
				cs_convert_status(rc));
		      EXIT(-1);
		  }

		// We got the mad... not compare it what was sent

		if (memcmp
		    (&(in_mad.base), &(out_mad.base),
		     sizeof(out_mad.base)))
		  {
		      printf
			  ("ERROR: %d - Unable to recv  INTERNAL mad as expected\n",
			   i);
		      printf("ERROR: %s - mai_recv return status\n",
				cs_convert_status(rc));
		      EXIT(-1);
		  }
		}

		// Now check to filter purge is being tested

		if (flags & VFILTER_PURGE)
		  {
		      int             sendcount = rand() % 6;
		      // Send the mad again .. then receive it from 

		      if (sendcount == 0)
			  sendcount = 1;

		      for (; sendcount; sendcount--)
			{
			    rc = mai_send(fd[(i + rand()) % fdcount],
					  &out_mad);

			    if (rc != VSTATUS_OK)
			      {
				  printf
				      ("ERROR: %d - Unable to send INTERNAL mad\n",
				       i);
				  printf
				      ("ERROR: %s - mai_send return status\n",
				       cs_convert_status(rc));
				  EXIT(-1);
			      }
			}
		      // Now that we have sent the Mad see if we receive
		      // it on the handle. long timeout since we don't expect it
		      rc = mai_recv(fd[fdcount], &in_mad, 20000 * scale);

		      /*
		       * We should not get a mad on this handle since we 
		       * did not place a filter on it.
		       */
		      if (rc != VSTATUS_TIMEOUT)
			{
			    printf
				("ERROR: %d - Unexpected behavior \n", i);
			    printf
				("ERROR: %s - mai_recv should timeout\n",
				 cs_convert_status(rc));
			    EXIT(-1);
			}

		  }

		// now delete the filter and send the mad again.. this
		// time we
		// should timeout

		switch (mode)
		  {
		  case FILTER_EXPOSED:
		      {

			  // First delete the filter by description
			  rc = mai_filter_delete(fd[i % fdcount],
						 &all_filter[i], flags);

			  if (rc != VSTATUS_OK)
			    {
				printf
				    ("ERROR: %d - Unable to delete  filter\n",
				     i);
				printf
				    ("ERROR: %s - mai_filter_delete return status\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }

			  // now try and delete the filter by handle. This 
			  // 
			  // 
			  // should fail
			  rc = mai_filter_hdelete(fd[i % fdcount],all_handles[i]);

			  if (rc == VSTATUS_OK)
			    {
				printf
				    ("ERROR: %d - Filter delete succeeded .. should fail\n",
				     i);
				printf
				    ("ERROR: %s - mai_filter_hhelete return status\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }

		      }
		      break;
		  case FILTER_METHOD:
		  case FILTER_AID:
		  case FILTER_AMOD:
		      {
			  rc = mai_filter_hdelete(fd[i % fdcount],all_handles[i]);

			  if (rc != VSTATUS_OK)
			    {
				printf
				    ("ERROR: %d - Unable to delete duplicate filter\n",
				     i);
				printf
				    ("ERROR: %s - mai_filter_hdelete return status\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }

			  // now try and delete the filter by handle. This 
			  // 
			  // 
			  // should fail
			  rc = mai_filter_hdelete(fd[i % fdcount],all_handles[i]);

			  if (rc == VSTATUS_OK)
			    {
				printf
				    ("ERROR: %d - Filter delete succeeded .. should fail\n",
				     i);
				printf
				    ("ERROR: %s - mai_filter_hhelete return status\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }

		      }
		      break;
		  case FILTER_ONCE:
		      {
			  // this should fail.. once the mad was received
			  // the filter should
			  // have been deleted.

			  rc = mai_filter_hdelete(fd[i % fdcount],all_handles[i]);

			  if (rc == VSTATUS_OK)
			    {
				printf
				    ("ERROR: %d - Filter delete succeeded .. should fail\n",
				     i);
				printf
				    ("ERROR: %s - mai_filter_hhelete return status\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }
		      }

		      // We deleted the filter. If it was tagged as purge
		      // no data should be
		      // received for that filter if we now do a receive
		      if (flags & VFILTER_PURGE)
			{
			    rc = mai_recv(fd[i % fdcount], &in_mad,
					  10 * scale);

			    if (rc != VSTATUS_TIMEOUT)
			      {
				  printf
				      ("ERROR: %d - un expected expected behavior\n",
				       i);
				  printf
				      ("ERROR: %s - mai_recv return MAD after purge filter deleted\n",
				       cs_convert_status(rc));
				  EXIT(-1);
			      }

			}

		      // now send the mad again. This time it should 
		      // disappear as no one is listening for it.

		      rc = mai_send(fd[(i + rand()) % fdcount], &out_mad);

		      if (rc != VSTATUS_OK)
			{
			    printf
				("ERROR: %d - Unable to send INTERNAL mad\n",
				 i);
			    printf
				("ERROR: %s - mai_send return status\n",
				 cs_convert_status(rc));
			    EXIT(-1);
			}

		      rc = mai_recv(fd[i % fdcount], &in_mad,
				    timeout * scale);

		      if (rc != VSTATUS_TIMEOUT)
			{
			    printf
				("ERROR: %d - Expected timeout on receive\n",
				 i);
			    printf
				("ERROR: %s - mai_recv return status\n",
				 cs_convert_status(rc));
			    EXIT(-1);
			}

		  }
	    }

	  // We get here then the everything went OK.
	  // make sure all the inactive flags are set

	  for (i = 0; i < max; i++)
	    {
		if (inactive[i] == 0)
		  {
		      printf
			  ("ERROR: %d - Test missed handle!! Arrgggh\n",
			   i);
		  }

	    }

	  printf
	      ("\n************* Filter create phase %d passed ***********\n",
	       4);

	  if (flags & VFILTER_PURGE)
	    {
		IBhandle_t      hd[2];
		int             class = 18;
		int             method = 21;
		int             loop,
		                sendcount;

		printf
		    ("\n************* Filter PURGE phase %d starting  ***********\n",
		     4);

		// If purge was active we need to execise the different
		// cases
		// Case 1. Only the purge filter is on the handle
		// (validated above)
		// Case 2. The purge filter and another filter is on the
		// handle and
		// the pass the same data. 
		// Subcase a: In this case purge must not be done if
		// the purge filter is deleted first it the data must
		// persist
		// Subcase b:
		// If the purge filter is deleted last the matching data
		// must disappear
		// 
		// Case 3: A mix of purged filter MADs and non purged MADs 
		// 
		// 
		// on the handle.
		// Irrespective of the fileter delete order the purged
		// MADs should
		// not be seen.

		printf
		    ("\n************* Testing case of overlapping PURGE/Non PURGE Filter start ***********\n");

		for (loop = 0; loop < 2; loop++)
		  {

		      rc = mai_filter_method(fd[0], flags, MAI_TYPE_INTERNAL,	// Only 
										// 
					     // 
					     // exclusive 
					     // filter
					     &hd[0], class, method);

		      if (rc != VSTATUS_OK)
			{
			    printf
				("ERROR: %d - Unable to create  filter case 2\n",
				 i);
			    printf
				("ERROR: %s - mai_filter_method return status\n",
				 cs_convert_status(rc));
			    EXIT(-1);
			}

		      rc = mai_filter_method(fd[0], flags & (~VFILTER_PURGE), MAI_TYPE_ANY,	// Note 
												// 
					     // this 
					     // will 
					     // pass 
					     // INTERNAL 
					     // like 
					     // above 
					     // filter
					     &hd[1], class, method);

		      if (rc != VSTATUS_OK)
			{
			    printf
				("ERROR: %d - Unable to create  filter case 2\n",
				 i);
			    printf
				("ERROR: %s - mai_filter_method return status\n",
				 cs_convert_status(rc));
			    EXIT(-1);
			}

		      out_mad.base.method = method;
		      out_mad.base.mclass = class;

		      sendcount = 3;

		      {

			  // Send the mad again .. then receive it from 

			  for (; sendcount; sendcount--)
			    {
				rc = mai_send(fd[(rand()) % fdcount],
					      &out_mad);

				if (rc != VSTATUS_OK)
				  {
				      printf
					  ("ERROR: %d - Unable to send INTERNAL mad\n",
					   i);
				      printf
					  ("ERROR: %s - mai_send return status\n",
					   cs_convert_status(rc));
				      EXIT(-1);
				  }
			    }

			  rc = mai_recv(fd[fdcount], &in_mad,
					20000 * scale);

			  /*
			   * We should not get a mad on this handle since we 
			   * did not place a filter on it.
			   */

			  if (rc != VSTATUS_TIMEOUT)
			    {
				printf
				    ("ERROR: %d - Unexpected behavior \n",
				     i);
				printf
				    ("ERROR: %s - mai_recv should timeout\n",
				     cs_convert_status(rc));
				EXIT(-1);
			    }

		      }

		      /*
		       * Now that we have sent the MADs delete the filters and then
		       * do recv on the handle we sent it on.
		       */

		      switch (loop)
			{
			case 0:
			    {
				// Delete the purge filter first.
				printf
				    ("\n************* Case delete purge filter first  *******\n");
				rc = mai_filter_hdelete(fd[0],hd[0]);

				if (rc != VSTATUS_OK)
				  {
				      printf
					  ("ERROR: %d - Purge Filter delete  ..  failed\n",
					   i);
				      printf
					  ("ERROR: %s - mai_filter_hdelete return status\n",
					   cs_convert_status(rc));
				      EXIT(-1);
				  }

				rc = mai_filter_hdelete(fd[0],hd[1]);

				if (rc != VSTATUS_OK)
				  {
				      printf
					  ("ERROR: %d - None Purge Filter delete  ..  failed\n",
					   i);
				      printf
					  ("ERROR: %s - mai_filter_hdelete return status\n",
					   cs_convert_status(rc));
				      EXIT(-1);
				  }

				sendcount = 3;

				for (; sendcount; sendcount--)
				  {
				      rc = mai_recv(fd[0], &in_mad,
						    100000 * scale);

				      if (rc != VSTATUS_OK)
					{
					    printf
						("ERROR: %d - Purge Case 2:a Unable to recv INTERNAL mad\n",
						 i);
					    printf
						("ERROR: %s - mai_recv return status\n",
						 cs_convert_status(rc));
					    EXIT(-1);
					}
				  }

				// Should not get anything here
				rc = mai_recv(fd[0], &in_mad,
					      100000 * scale);

				if (rc == VSTATUS_OK)
				  {
				      printf
					  ("ERROR: %s - Purge Case 2:a Too many  INTERNAL mad\n",
					   cs_convert_status(rc));
				      printf
					  ("ERROR: %s - mai_recv return status\n",
					   cs_convert_status(rc));
				      EXIT(-1);
				  }

				printf
				    ("\n*************Passed delete purge filter first case *******\n");
			    }
			    break;
			case 1:
			    {
				// Delete the purge filter last
				printf
				    ("\n************* Case delete purge filter last *******\n");
				rc = mai_filter_hdelete(fd[0],hd[1]);

				if (rc != VSTATUS_OK)
				  {
				      printf
					  ("ERROR: %d - None Purge Filter delete  ..  failed\n",
					   i);
				      printf
					  ("ERROR: %s - mai_filter_hdelete return status\n",
					   cs_convert_status(rc));
				      EXIT(-1);
				  }

				rc = mai_filter_hdelete(fd[0],hd[0]);

				if (rc != VSTATUS_OK)
				  {
				      printf
					  ("ERROR: %d - Purge Filter delete  ..  failed\n",
					   i);
				      printf
					  ("ERROR: %s - mai_filter_hdelete return status\n",
					   cs_convert_status(rc));
				      EXIT(-1);
				  }

				sendcount = 3;

				for (; sendcount; sendcount--)
				  {
				      rc = mai_recv(fd[0], &in_mad,
						    1000 * scale);

				      if (rc == VSTATUS_OK)
					{
					    printf
						("ERROR: %s - Purge Case 2:b PURGE did not work\n",
						 cs_convert_status(rc));
					    printf
						("ERROR: %s - mai_recv return status\n",
						 cs_convert_status(rc));
					    EXIT(-1);
					}
				  }

				printf
				    ("\n*************Passed Case delete purge filter last *******\n");
			    }
			    break;
			default:
			    break;

			}

		  }

		printf
		    ("\n************* Testing case non matching  filters (one purge) with  MADs pending *******\n");
		// Case 3:
		// Now create two filters with different parms.

		rc = mai_filter_method(fd[0], flags, MAI_TYPE_INTERNAL,	// Only 
									// 
				       // 
				       // exclusive 
				       // filter
				       &hd[0], class, method);

		if (rc != VSTATUS_OK)
		  {
		      printf
			  ("ERROR: %d - Unable to create  filter case 3\n",
			   0);
		      printf
			  ("ERROR: %s - mai_filter_method return status\n",
			   cs_convert_status(rc));
		      EXIT(-1);
		  }

		rc = mai_filter_method(fd[0], flags & (~VFILTER_PURGE), MAI_TYPE_ANY,	// Note 
											// 
				       // 
				       // this 
				       // will 
				       // pass 
				       // INTERNAL 
				       // like 
				       // above 
				       // filter
				       &hd[1], class + 1, method);

		if (rc != VSTATUS_OK)
		  {
		      printf
			  ("ERROR: %d - Unable to create  filter case 3\n",
			   1);
		      printf
			  ("ERROR: %s - mai_filter_method return status\n",
			   cs_convert_status(rc));
		      EXIT(-1);
		  }

		sendcount = 0;	// keep track of non-purged sent

		for (i = 0; i < 7; i++)
		  {

		      out_mad.base.method = method;
		      out_mad.base.mclass = class;

		      if (rand() % 2)
			{
			    /*
			     * Send to Match NON purge 
			     */
			    out_mad.base.mclass = class + 1;
			    sendcount++;
			}

		      // Send the mad 

		      rc = mai_send(fd[(rand()) % fdcount], &out_mad);

		      if (rc != VSTATUS_OK)
			{
			    printf
				("ERROR: %d - Unable to send INTERNAL mad\n",
				 i);
			    printf
				("ERROR: %s - mai_send return status\n",
				 cs_convert_status(rc));
			    EXIT(-1);
			}
		  }

		// Now pull the MADs into MAI 
		rc = mai_recv(fd[fdcount], &in_mad, 100000 * scale);

		// Now that they are in MAI delete the purge filter..
		// all associated MADS should go away

		rc = mai_filter_hdelete(fd[0],hd[0]);

		if (rc != VSTATUS_OK)
		  {
		      printf
			  ("ERROR: %s - Purge Filter delete  ..  failed case 3\n",
			   cs_convert_status(rc));
		      printf
			  ("ERROR: %s - mai_filter_hdelete return status\n",
			   cs_convert_status(rc));
		      EXIT(-1);
		  }

		// We should now recv the other MADS

		for (; sendcount; sendcount--)
		  {
		      rc = mai_recv(fd[0], &in_mad, 100000 * scale);

		      if (rc != VSTATUS_OK)
			{
			    printf
				("ERROR: %s - Purge Case 3: PURGE deleted too many not work\n",
				 cs_convert_status(rc));
			    printf
				("ERROR: %s - mai_recv return status\n",
				 cs_convert_status(rc));
			    EXIT(-1);
			}
		      else
			{
			    if (in_mad.base.mclass == class)
			      {
				  printf
				      ("ERROR: %s - Purge Case 3: PURGE did not delete\n",
				       cs_convert_status(rc));
				  EXIT(-1);
			      }

			}
		  }

		// We get here we got the number of non-purged MAD sent.
		// Now make sure there
		// is nothing left in the pipe.

		rc = mai_recv(fd[0], &in_mad, 50000 * scale);

		if (rc == VSTATUS_OK)
		  {
		      printf
			  ("ERROR: %s - Purge Case 3: PURGE did not delete all \n",
			   cs_convert_status(rc));
		      printf("ERROR: %d - mai_recv return class\n",
				in_mad.base.mclass);
		      EXIT(-1);
		  }

		rc = mai_filter_hdelete(fd[0],hd[1]);

		if (rc != VSTATUS_OK)
		  {
		      printf
			  ("ERROR: %s - None Purge Filter delete  case 3..  failed\n",
			   cs_convert_status(rc));
		      printf
			  ("ERROR: %s - mai_filter_hdelete return status\n",
			   cs_convert_status(rc));
		      EXIT(-1);
		  }

		// If we get here we have passsed the purge test.
		printf
		    ("\n********* Filter PURGE phase %d completed sucessfully  ********\n",
		     4);

	    }

	  // We need at least two open handles to test this
	  if (fdcount != 1)
	    {
		int             count;
		printf
		    ("\n********* Testing Close with Pending Data & Filters  ********\n");

		// Now add some filters and data on a handle so that when
		// the handle is close the
		// the clean up logic will be exercised.
		count = max / 2;
		if (count > 8)
		    count = 8;

		for (i = 0; i < count; i++)
		  {
		      char            name[32];
		      IBhandle_t      hdl;

		      switch (mode)
			{
			case FILTER_EXPOSED:
			    {

				fh->dev = dev;
				fh->qp = MAI_FILTER_ANY;
				fh->port = port;
				fh->type = MAI_TYPE_ANY;

				fh->value.bversion = i % 2;
				fh->value.mclass = i % 256;
				fh->value.cversion = i % 256;
				fh->value.method = i % 256;
				fh->value.status = i % (1 << 16);
				fh->value.hopPointer = i % 64;
				fh->value.hopCount = i % 64;
				fh->value.tid = i;
				fh->value.aid = i % (1 << 16);
				fh->value.amod = i;

				fh->mask.bversion = MAI_FMASK_ALL;
				fh->mask.mclass = MAI_FMASK_ALL;
				fh->mask.cversion = MAI_FMASK_ALL;
				fh->mask.method = MAI_FMASK_ALL;
				fh->mask.status = MAI_FMASK_ALL;
				fh->mask.hopPointer = MAI_FMASK_ALL;
				fh->mask.hopCount = MAI_FMASK_ALL;
				fh->mask.tid = MAI_FMASK_ALL;
				fh->mask.aid = MAI_FMASK_ALL;
				fh->mask.amod = MAI_FMASK_ALL;

				fh->active =
				    (MAI_ACT_QP | MAI_ACT_DEV |
				     MAI_ACT_TYPE | MAI_ACT_PORT |
				     MAI_ACT_FMASK);

				sprintf(name, "Fil %d", i);
				MAI_SET_FILTER_NAME(fh, name);

				memcpy(&out_mad.base, &fh->value,
				       sizeof(out_mad.base));

				// now create the filter

				rc = mai_filter_hcreate(fd[1], fh, flags,
							&hdl);

				if (rc != VSTATUS_OK)
				  {
				      printf
					  ("ERROR: %d - Unable to create filter\n",
					   i);
				      printf
					  ("ERROR: %s - mai_filter_create return status\n",
					   cs_convert_status(rc));
				      EXIT(-1);
				  }
			    }
			    break;
			case FILTER_METHOD:
			    {
				rc = mai_filter_method(fd[1], flags,
						       MAI_TYPE_INTERNAL, &hdl,
						       i / 256, i % 256);

				if (rc != VSTATUS_OK)
				  {
				      printf
					  ("ERROR: %d - Unable to create filter\n",
					   i);
				      printf
					  ("ERROR: %s - mai_filter_method return status\n",
					   cs_convert_status(rc));
				      EXIT(-1);
				  }

				out_mad.base.mclass = i / 256;
				out_mad.base.method = i % 256;
			    }
			    break;
			case FILTER_AID:
			    {
				rc = mai_filter_aid(fd[1], flags,
						    MAI_TYPE_INTERNAL, &hdl,
						    i / 256, i % 256, i);

				if (rc != VSTATUS_OK)
				  {
				      printf
					  ("ERROR: %d - Unable to create filter\n",
					   i);
				      printf
					  ("ERROR: %s - mai_filter_aid return status\n",
					   cs_convert_status(rc));
				      EXIT(-1);
				  }

				out_mad.base.mclass = i / 256;
				out_mad.base.method = i % 256;
				out_mad.base.aid = i;

			    }
			    break;
			case FILTER_AMOD:
			    {
				rc = mai_filter_amod(fd[1], flags,
						     MAI_TYPE_INTERNAL, &hdl,
						     i / 256, i % 256, i,
						     i);

				if (rc != VSTATUS_OK)
				  {
				      printf
					  ("ERROR: %d - Unable to create filter\n",
					   i);
				      printf
					  ("ERROR: %s - mai_filter_amod return status\n",
					   cs_convert_status(rc));
				      EXIT(-1);
				  }

				out_mad.base.mclass = i / 256;
				out_mad.base.method = i % 256;
				out_mad.base.aid = i;
				out_mad.base.amod = i;
			    }
			    break;
			case FILTER_ONCE:
			    {
				rc = mai_filter_once(fd[1], flags,
						     MAI_TYPE_INTERNAL, &hdl,
						     i / 256, i);

				if (rc != VSTATUS_OK)
				  {
				      printf
					  ("ERROR: %d - Unable to create filter\n",
					   i);
				      printf
					  ("ERROR: %s - mai_filter_method return status\n",
					   cs_convert_status(rc));
				      EXIT(-1);
				  }
				out_mad.base.mclass = i / 256;
				out_mad.base.tid = i;
			    }
			    break;
			default:
			    {
				printf
				    ("ERROR: %d - Unknown filter test mode\n",
				     mode);
				EXIT(-1);
			    }
			}

		      out_mad.type = MAI_TYPE_INTERNAL;
		      rc = mai_send(fd[0], &out_mad);

		      if (rc != VSTATUS_OK)
			{
			    printf
				("ERROR: %d - Unable to send INTERNAL mad\n",
				 i);
			    printf
				("ERROR: %s - mai_send return status\n",
				 cs_convert_status(rc));
			    EXIT(-1);
			}

		  }

		// Now do a receive to pull in all the MADs sent. We
		// should not see
		// any mad as the filters are on a different handle

		rc = mai_recv(fd[0], &in_mad, 1000000 * scale);

		if (rc != VSTATUS_TIMEOUT)
		  {
		      printf
			  ("ERROR: %s - Received non-timeout value on handle\n",
			   cs_convert_status(rc));
		      EXIT(-1);
		  }

		// Now close the hanle with the stuff on it.
		rc = mai_close(fd[1]);

		if (rc != VSTATUS_OK)
		  {
		      printf("ERROR: %s - mai_close return status\n",
				cs_convert_status(rc));
		      EXIT(-1);
		  }

		// We get here then the close worked OK

		printf
		    ("\n********* Close with Pending Data & Filters Passed  ********\n");

	    }
	}

	  // Determine the we have the same number of resouces
	  // as we did before we started all this stuff.

    // now close the handles 

    for (i = 0; i < fdcount; i++)
      {
	  if (i == 1)
	    {
		// * skip this because we closed it in the close test
		// above */
		continue;
	    }
	  if ((rc = mai_close(fd[i])) != VSTATUS_OK)
	    {
		printf("ERROR: %s - can't close MAI \n", cs_convert_status(rc));
		EXIT(-1);
	    }
      }

    printf("INFO: Filter test completed successfully\n");
    EXIT(0);
}
