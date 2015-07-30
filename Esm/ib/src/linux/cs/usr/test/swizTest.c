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

 * ** END_ICS_COPYRIGHT5   ****************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <getopt.h>

#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ib_status.h"

#include "cs_g.h"
#include "cs_swiz_test.h"

static int debug=0;
static int list=0;

typedef enum {
	OP_STANDALONE=0,
	OP_SERVER,
	OP_CLIENT,
	OP_BAD
} oper_mode_t;

oper_mode_t mode_op=OP_STANDALONE; 



int swiz_op_client(int, testType_t , in_addr_t *);
void do_swiz_table_dump(void);
void  swiz_op_server( void );


void
usage()
{
	printf("Usage: \n");
	printf("-a   run all test\n");
	printf("-d   dump memory\n");
	printf("-c  <num> number of times to do test\n");	
	printf("-t  <num> test to perform\n");
	printf("-l  dump test table\n");
	return;
}

//
// All the cosmic, groovy action starts here..
//
int
main( int  argc,
      char *argv[] )
{
	extern    char    *optarg;
	
	in_addr_t in_addr, *p_addr=NULL;

	int       do_all=0, c, iter=1;
	
	testType_t tt=0;


	mode_op = OP_STANDALONE;
	
	while ( (c=getopt(argc, argv, "hSCladc:i:t:")) != EOF ) {
		switch(c) {

		case 'S' : // Operate in server mode.
			mode_op = OP_SERVER;
			break;

		case 'C' : // Operate in client mode.
			mode_op = OP_CLIENT;
			break;
			
		case 'a' : // Perform all tests.
			do_all++;
			break;
			
		case 'd' : // Bump the debug flag.
			debug++;
			break;

		case 'c' : // Perform test(s) 'c' number of times.
			iter=atoi(optarg);
			break;

		case 'l' : // Perform test(s) 'c' number of times.
		        list=1;
			break;

		case 'i' : // Used to specify remote server IP Address.
			if ( inet_aton(optarg, (struct in_addr *)&in_addr) == 0 ) {
				printf("invalid ipaddress (%s)\n", optarg);
				usage();
				exit(2);
			}
			p_addr=&in_addr;
			break;
			
		case 't' : // Test to perform (valid testType_t)
			tt=atoi(optarg);
			
			if ( tt >= E_BADVALUE ) {
				printf("invalid test number %d\n", tt);
				usage();
				exit(2);
			}
			break;

		default: // something is messed up.
			usage();
			exit(2);
		}
	}

	if(list)
	  {
	    do_swiz_table_dump();
	  }

	switch( mode_op ) {
	case OP_SERVER : 
		{
			// any of these options are invalid with OP_SERVER.
			if ( do_all || iter || tt ) {
				usage();
				exit(10);
			}
			
			// swiz_op_server();
			printf("OP_SERVER mode not supported\n");
		}
		break;
		
	case OP_CLIENT : 
		{
			// swiz_op_client( iter, tt, p_addr);
			printf("OP_CLIENT mode not supported\n");
		}
		break;
		
	case OP_STANDALONE : 
		{
			
			if (! do_all) {
				swiz_op_standalone( iter, tt, debug );
			}
			else {
				for (tt = 0; tt < E_BADVALUE; tt++) 
					swiz_op_standalone( iter, tt , debug);
			}
		}
		break;

	case OP_BAD :
	default:
		fprintf(stderr,"Invalid operation mode\n");
		exit(2);
		break;
	}
	
	exit( 0 );
}














