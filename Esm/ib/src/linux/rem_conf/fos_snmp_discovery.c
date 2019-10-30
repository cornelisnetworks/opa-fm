/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

#include "snmp_com_api.h"
#include "snmp_com_discovery.h"
#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>



/*
	1. Provide callback function.
	2. Update the data-structure to return the contents of the sys-oid?
	3. Write a routine to initiate the discovery
	4. Data structure to store the results. (dynamically grow the list?)
	5. Dump out system information for each in list?
	6. Reverse lookup for ip-address
*/

snmp_com_errno_t 
discovery_cb(void *client_hdl, snmp_com_discovery_data *data)
{
	int i;
	// For each create our own connection.
	printf("Found target: %-16s  SYSOID: ",data->address);
	for(i=0;i<(data->oid_len - 1);i++){
		   printf("%u.",data->system_oid[i]);
	}
	printf("%u\n",data->system_oid[i]);



	return SNMP_COM_OK;
}






void
usage(char *cmd)
{
    fprintf(stderr, "USAGE: %s", cmd);
    fprintf(stderr, " [OPTIONS] START_IP END_IP\n\n");

	fprintf(stderr,
            "OPTIONS:\n");
    fprintf(stderr, "  -h, --help\t\tdisplay this help message\n");
    fprintf(stderr, "  -v 1|2c\t\tspecifies SNMP version to use\n");
    fprintf(stderr, "  -c COMMUNITY\t\tset the community string\n");
    fprintf(stderr,
            "  -m MIB[:...]\t\tload given list of MIBs (ALL loads everything)\n");
    fprintf(stderr,
            "  -M DIR[:...]\t\tlook in given list of directories for MIBs\n");
    fprintf(stderr,
            "  -t <VAL>\t\ttimeout in microseconds per snmp packet\n");
    fprintf(stderr,
            "  -r <VAL>\t\tNumber of retrys per request\n");
    fprintf(stderr,
            " \n\n"); 
    fflush(stderr);

}



/*****************************************************************************/

int main (int argc, char **argv)
{
	p_snmp_disc_hdl_t		session;
	snmp_com_errno_t		res;
	char					*mibs = NULL;
	char					*mibdir = NULL;
	uint16_t				port = 161;
	long					version = 1;
	char					*community = "public";
	int						retries = 0;
	long					timeout = 0;
    int             		arg;
	char					*start_ip, *end_ip;
	int						num_found = 0;
	snmp_com_discovery_data	*disc_targets;
	int						i,j;

	// Get options at the command line (overide default values)

    while ((arg = getopt(argc, argv, "m:M:c:v:o:t:r:h-")) != EOF) {
        switch (arg) {
			case 'h':
			case '-':
				usage(argv[0]);
				return(0);
			case 't':
				timeout = atol(optarg);
				break;
			case 'r':
				retries = atol(optarg);
				break;
			case 'm':
				mibs = optarg;
				break;
			case 'M':
				mibdir = optarg;
				break;
			case 'c':
				community = optarg;
				break;
			case 'v':
				version = -1;
				if (!strcmp(optarg, "1")) {
					version = SNMP_VER_1;
				}
				if (!strcasecmp(optarg, "2c")) {
					version = SNMP_VER_2C;
				}
				// Note: V3 Support not fully in yet. Plus we need to handle the additional 
				// 	     options
				//if (!strcasecmp(optarg, "3")) {
				//	session.version = SNMP_VERSION_3;
				//}
				if (version == -1) {
					fprintf(stderr,
							"Invalid version specified after -v flag: %s\n",
							optarg);
					return (-1);
				}

				break;
			default:
				usage(argv[0]);
				return(-1);
		}

	}


	if(optind != (argc - 2)){
        fprintf(stderr, "Start end addresses required\n");
		usage(argv[0]);
		return -1;
	}

	start_ip = argv[optind++];
	end_ip = argv[optind++];


	if((res = snmp_com_discovery_init(&session,start_ip,end_ip,port,version,community,retries,timeout)) != SNMP_COM_OK)
	{
		printf("Could not initialize session: %d", res);
		return -1;
	}

	snmp_com_discovery_async(session,discovery_cb,NULL);
	snmp_com_discovery_sync(session,&num_found);

	printf("Found: %d\n",num_found);

	if((disc_targets = calloc(num_found,sizeof(snmp_com_discovery_data))) != NULL){
		snmp_com_discovery_fill(session,disc_targets,&num_found);
		for(i=0;i<num_found;i++){
			printf("Found target: %-16s  SYSOID: ",disc_targets[i].address);
			for(j=0;j<(disc_targets[i].oid_len - 1);j++){
				printf("%u.",disc_targets[i].system_oid[j]);
			}
			printf("%u\n",disc_targets[i].system_oid[j]);
		}
	}

	free(disc_targets);

	return 0;
}

