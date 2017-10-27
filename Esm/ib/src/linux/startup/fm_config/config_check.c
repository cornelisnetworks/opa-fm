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

//==============================================================================//
//										//
// FILE NAME									//
//    config_check.c								//
//										//
// DESCRIPTION									//
//    This file parses and verifies the opafm.xml config file //
//										//
// DATA STRUCTURES								//
//    None									//
//==============================================================================//


#include <getopt.h>

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/limits.h>
#include "cs_g.h"
#include <fm_xml.h>

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define MAX_INSTANCES 8
#define FM_CONFIG_FILENAME "/etc/opa-fm/opafm.xml"

uint8_t		debug;					// debugging mode - default is FALSE
uint8_t		checksum;				// checksum mode - default is FALSE
uint8_t		embedded;				// is it called internally ?- default is FALSE
char		config_file [PATH_MAX+1]; // location and name of the config file
Pool_t		startup_pool;			// a generic pool for malloc'ing memory
#define		PROG_NAME_MAX 25
char 		prog_name[PROG_NAME_MAX+1];

int instance = 0;

// XML configuration data structure
FMXmlCompositeConfig_t *xml_config = NULL;
FMXmlConfig_t *fmp;
SMXmlConfig_t *smp;
PMXmlConfig_t *pmp;
FEXmlConfig_t *fep;

void		Usage		(void);
Status_t	alloc_mem		(void);
void		free_mem		(void);
void		check_multicast_VFs (int num_instances, VirtualFabrics_t **vf_configs);
void		print_startup_information		(void);
void		print_checksum_information (int num_instances, VirtualFabrics_t **vf_configs);
void* getXmlParserMemory(uint32_t size, char* info);
void freeXmlParserMemory(void *address, uint32_t size, char* info);

//command line options
struct option options[]={
	{"help", no_argument, NULL, '$'},
	{0}
};

void
Usage (void)
{
	printf ("Usage %s [-s] [-c config_file] [-v] [-d]\n", prog_name);
	printf ("        or %s --help\n",prog_name);
	printf ("\n");
	printf ("   -c config file    (default=" FM_CONFIG_FILENAME ")\n");
	printf ("   -v                display debugging and status information\n");
	printf ("   -s                strict check mode (validate multicast and VF settings)\n");
	printf ("                     This option will point out inconsistencies or\n"); 
	printf ("                     invalid settings in VF and multicast config.\n");
	printf ("   -d                display configuration checksum information\n");
	printf ("\n");
	printf ("Examples:\n");
	printf ("  %s\n", prog_name);
	printf ("  %s -v\n", prog_name);
	printf ("  %s -sv\n", prog_name);
	// Note - the -e option is not presented to user since it is for embedded use only
	exit(2);
}


void
Usage_full (void)
{
	printf ("Usage %s [-s] [-c config_file] [-v] [-d]\n", prog_name);
	printf ("        or %s --help\n",prog_name);
	printf ("\n");
	printf ("   -c config file    (default=" FM_CONFIG_FILENAME ")\n");
	printf ("   -v                display debugging and status information\n");
	printf ("   -s                strict check mode (validate multicast and VF settings)\n");
	printf ("                     This is option will point out inconsistencies or\n"); 
	printf ("                     invalid settings in VF and multicast config.\n");
	printf ("   -d                display configuration checksum information\n");
	printf ("\n");
	printf (" %s parses and verifies the configuration file of a FM.\n", prog_name);
	printf ("Displays debugging and status information.\n");
	printf ("\n");
	printf ("Examples:\n");
	printf ("  %s\n", prog_name);
	printf ("  %s -v\n", prog_name);
	printf ("  %s -sv\n", prog_name);
	// Note - the -e option is not presented to user since it is for embedded use only
	exit(0);
}

static void print_warn(const char *msg)
{
	fprintf(stdout, "FM Instance %d: WARNING: %s",instance,msg);
}

static void print_error(const char *msg)
{
	fprintf(stdout, "FM Instance %d: ERROR: %s",instance,msg);
}

 
int
main (int argc, char *argv []) {
	int		c;		// used to parse the command line
	char 		tmp[PATH_MAX+1];
	char *		ptr;
	int		index;
	int ret = 0;
	IXmlParserFlags_t parser_flags = IXML_PARSER_FLAG_NONE;
	
	debug       = FALSE;
	checksum    = FALSE;
	embedded    = FALSE;
	strncpy (config_file, FM_CONFIG_FILENAME, PATH_MAX);
	strncpy (tmp, argv[0], PATH_MAX);
	tmp[PATH_MAX] = 0;
	ptr = strrchr(tmp, '/');
	strncpy (prog_name, ptr ? ptr+1 : tmp, PROG_NAME_MAX);
	prog_name[PROG_NAME_MAX]=0;
	
	VirtualFabrics_t *vf_configs[MAX_INSTANCES] = { NULL };

	while ((c = getopt_long (argc, argv, "c:vsde", options, &index)) != -1) {
		switch (c) {
		case '$':
			Usage_full ();
			break;
		// input config file
		case 'c':
			strncpy(config_file, optarg, sizeof(config_file));
			config_file[PATH_MAX-1]=0;
			break;
		case 'v':
			debug = TRUE;
			break;
		case 's':
			parser_flags = IXML_PARSER_FLAG_STRICT;
			break;
		case 'd':
			checksum = TRUE;
			break;
		case 'e':
			embedded = TRUE;
			break;
		default:
			fprintf(stderr, "invalid argument -%c\n", c);
			Usage ();
		}
	}
	if (optind < argc) {
		Usage ();
	}
	
	// Allocate memory for reading and parsing the config file.
	if (alloc_mem () != VSTATUS_OK) {
		exit(1);
	}

	//if (read_info_file() != VSTATUS_OK) {
	//	exit(1);
	//}

	// init callback function for XML parser so it can get pool memory
	initXmlPoolGetCallback(&getXmlParserMemory);
	initXmlPoolFreeCallback(&freeXmlParserMemory);

	// parse the XML config file
	xml_config = parseFmConfig(config_file, parser_flags, /* instance does not matter for startup */ 0, /* full parse */ 1, /* embedded */ embedded);
	if (!xml_config) {
		fprintf(stdout, "Could not open or there was a parse error while reading configuration file ('%s')\n", config_file);
		ret = 1;
		goto failed;
	}

	for (instance = 0; instance < xml_config->num_instances; instance++) {
		vf_configs[instance] = renderVirtualFabricsConfig(instance, xml_config, &xml_config->fm_instance[instance]->sm_config, NULL);
		if (vf_configs[instance] == NULL) {
			fprintf(stdout, "Failed to allocate virtual fabrics config for instance %d.\n",instance);
			ret = 1;
			goto failed;
		}
		ret |= !applyVirtualFabricRules(vf_configs[instance], print_error, print_warn);
	}

	if (parser_flags & IXML_PARSER_FLAG_STRICT) {
		check_multicast_VFs(xml_config->num_instances, vf_configs);
	}

	if (debug) {
		fprintf(stdout, "Successfully parsed %s\n", config_file);
		print_startup_information();
	}

	if (checksum)
		print_checksum_information(xml_config->num_instances, vf_configs);

failed:
	// if we have an XML config then release it
	if (xml_config != NULL) {
		for (instance = 0; instance < xml_config->num_instances; instance++) {
			if (vf_configs[instance]) {
				releaseVirtualFabricsConfig(vf_configs[instance]);
				vf_configs[instance] = NULL;
			}
		}
		releaseXmlConfig(xml_config, /* full */ 1);
		xml_config = NULL;
	}

	//	Delete a pool of memory to work with.
	free_mem ();

	exit(ret);
}

// get memory for XML parser
void*
getXmlParserMemory(uint32_t size, char* info) {
	void        *address;
	Status_t    status;

#ifdef XML_MEMORY
	printf("called getXmlParserMemory() size (%u) (%s) from config_check.c\n", size, info);
#endif
	status = vs_pool_alloc(&startup_pool, size, (void*)&address);
	if (status != VSTATUS_OK || !address)
		return NULL;
	return address;
}

// free memory for XML parser
void
freeXmlParserMemory(void *address, uint32_t size, char* info) {

#ifdef XML_MEMORY
	printf("called freeXmlParserMemory() size (%u) (%s) from config_check.c\n", size, info);
#endif
	vs_pool_free(&startup_pool, address);
}

//------------------------------------------------------------------------------//
//										//
// Routine      : alloc_mem							//
//										//
// Description  : Allocate memory.						//
// Input        : None								//
// Output       : Status of the startup process					//
//                  0 - memory was allocated					//
//                  1 - error allocating memory					//
//										//
//------------------------------------------------------------------------------//

Status_t
alloc_mem (void) {
	uint32_t	pool_size;
	Status_t	status;
	
//
//	Create a pool of memory to work with.
//
	
	// calculate pool size for all instances since this is the initial parsing
	// check for all instances of FM and VF configuration.
	pool_size = xml_compute_pool_size(/* for all instances of sm */ 1);
	status = vs_pool_create (&startup_pool, 0, (uint8_t *)"startup_pool", NULL, pool_size);
	if (status != VSTATUS_OK) {
		fprintf ( stderr, "Could not create a pool (status = %d)\n", status);
		return (VSTATUS_BAD);
	}
	
	return (VSTATUS_OK);
}


//------------------------------------------------------------------------------//
//										//
// Routine      : free_mem							//
//										//
// Description  : Free up allocated memory.					//
// Input        : None								//
// Output       : None								//
//										//
//------------------------------------------------------------------------------//

void
free_mem (void) {
	
	(void) vs_pool_delete (&startup_pool);
	
	return;
}

//------------------------------------------------------------------------------//
//										//
// Routine      : check_multicast_VFs						//
//										//
// Description  : do strict checking of consistency of multicast and VFs
// Input        : None								//
// Output       : None								//
//										//
//------------------------------------------------------------------------------//

void
check_multicast_VFs (int num_instances, VirtualFabrics_t **vf_configs) {
	uint16_t	i;

	for (i = 0; i < num_instances; i++)
	{
		FMXmlInstance_t *fmip = xml_config->fm_instance[i];
		smp = &xml_config->fm_instance[i]->sm_config;
		uint32_t j;

		fmp = &fmip->fm_config;;

		// if not starting instance of FM then continue to next FM instance
		if (!fmp->start)
			continue;

		for (j=0; j< fmip->sm_mdg_config.number_of_groups; ++j) {
			SMMcastDefGrp_t *mdgp = &fmip->sm_mdg_config.group[j];
			if (! mdgp->def_mc_create)
				continue;
			if (strlen(mdgp->virtual_fabric)) {
				if (! findVfPointer(vf_configs[i], mdgp->virtual_fabric)) {
					fprintf(stdout,"Warning: FM instance %u (%s) MulticastGroup Ignored\n    references undefined VirtualFabric: %s\n",
				   	i, fmp->fm_name, mdgp->virtual_fabric);
				}
			}
			if (mdgp->def_mc_pkey != 0xffffffff) {
				uint32_t vf;
				int match = 0;
				for (vf=0; vf < vf_configs[i]->number_of_vfs; vf++) {
					if (PKEY_VALUE(vf_configs[i]->v_fabric[vf].pkey) == PKEY_VALUE(mdgp->def_mc_pkey)) {
						match=1;
						break;
					}
				}
				if (! match) {
					fprintf(stdout,"Warning: FM instance %u (%s) MulticastGroup Ignored\n    references undefined VirtualFabric PKey: 0x%x\n",
				   	i, fmp->fm_name, mdgp->def_mc_pkey);
				}
			}
		}
	}
}

//------------------------------------------------------------------------------//
//										//
// Routine      : print_startup_information						//
//										//
// Description  : Add a line of startup information to our data structures.		//
// Input        : None								//
// Output       : None								//
//										//
//------------------------------------------------------------------------------//

void
print_startup_information (void) {
	uint16_t	i;

	for (i = 0; i < MAX_INSTANCES; i++)
	{
		if (!xml_config->fm_instance[i]) continue;
		fmp = &xml_config->fm_instance[i]->fm_config;
		smp = &xml_config->fm_instance[i]->sm_config;
		pmp = &xml_config->fm_instance[i]->pm_config;
		fep = &xml_config->fm_instance[i]->fe_config;

		// if none of these are set, then it was not in config file
		if (! fmp->start
		  	&& fmp->hca == 0xffffffff && fmp->port == 0xffffffff
			&& fmp->port_guid == 0xffffffffffffffffULL)
			continue;

		fprintf(stdout,"FM instance %u (%s) will%s be started\n",
					   i, fmp->fm_name, fmp->start?"":" not");

		// if not starting instance of FM then continue to next FM instance
		if (!fmp->start)
			continue;

		if (fmp->port_guid) {
			fprintf(stdout, "   PortGuid: 0x%016"PRIx64"\n", fmp->port_guid);
		} else {
			fprintf(stdout, "   HFI: %u Port: %u\n", fmp->hca, fmp->port);
		}

		fprintf(stdout, "   SM will%s be started\n", 
				smp->start?"":" not");
	
		//fprintf(stdout, "   PM will%s be started\n", 
		//		pmp->start?"":" not");
	
		fprintf(stdout, "   FE will%s be started\n", 
				fep->start?"":" not");
	}
}

//------------------------------------------------------------------------------//
//										//
// Routine      : print_checksum_information						//
//										//
// Description  : Prints checksums of each parsed XML component//
// Input        : None								//
// Output       : None								//
//										//
//------------------------------------------------------------------------------//

void
print_checksum_information (int num_instances, VirtualFabrics_t **vf_configs) {
	uint16_t	i;
	uint16_t	v;

	fprintf(stdout,"\n");
	for (i = 0; i < num_instances; i++) {

		fmp = &xml_config->fm_instance[i]->fm_config;;
		smp = &xml_config->fm_instance[i]->sm_config;
		pmp = &xml_config->fm_instance[i]->pm_config;
		fep = &xml_config->fm_instance[i]->fe_config;

		fprintf(stdout,"FM instance %u\n", i);
		fprintf(stdout, "   SM overall checksum %8u    consistency checksum %8u\n", smp->overall_checksum, smp->consistency_checksum);
		fprintf(stdout, "   PM overall checksum %8u    consistency checksum %8u\n", pmp->overall_checksum, pmp->consistency_checksum);
		fprintf(stdout, "   VF database consistency checksum %8u\n", vf_configs[i]->consistency_checksum);
		for (v = 0; v < vf_configs[i]->number_of_vfs; v++) 
			fprintf(stdout, "       VF %s consistency checksum %8u\n", vf_configs[i]->v_fabric[v].name, vf_configs[i]->v_fabric[v].consistency_checksum);
		fprintf(stdout,"\n");
	}
}
