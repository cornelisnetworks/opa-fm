/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2018, Intel Corporation

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

//===========================================================================//
//									     //
// FILE NAME								     //
//    sm_linux_proc.c							     //
//									     //
// DESCRIPTION								     //
//    The main function part of  the OS dependent	//
//    part of SM.  This version parses the command	     //
//    line and starts the OS independent part.				     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    main				main entry point		     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//===========================================================================//

#include "os_g.h"
#include "ib_types.h"
#include "ib_mad.h"
#include "ib_status.h"
#include "cs_g.h"
#include "if3.h"
#include "sm_l.h"
#include "sm_dbsync.h"
#include "sa_l.h"

#include "signal.h"
#include "ibyteswap.h"


extern  uint32_t sm_nodaemon;

/*
 *	Simulator definitions.
 */
extern time_t	sm_starttime;

/*
 *	External SM independent routine.
 */
Status_t		sm_main(void);

/*
 *	Posix defines for argument cracking.
 */
extern	int		optind;
extern	int		opterr;
extern	int		optopt;
extern	char		*optarg;
extern  uint32_t sm_nodaemon;
extern  uint32_t xml_trace;

extern void* getSmXmlParserMemory(uint32_t size, char* info);
extern void freeSmXmlParserMemory(void *address, uint32_t size, char* info);
extern Status_t initSmXmlMemoryPool(void);
extern Status_t sm_parse_xml_config(void);
extern Status_t sm_initialize_sm_pool(void);
extern void smLogLevelOverride(void);

/*
 * sigterm signal handler setup
 */
void sm_linux_signal_handler(int a) {
	if (a == SIGHUP) {
		if (sm_state == SM_STATE_MASTER)
			sm_control_reconfig();
		else {
			IB_LOG_WARN_FMT(__func__, "Reconfigure request ignored: SM is not master (%s)", sm_getStateText(sm_state));
		}
	}
	else {
		sm_control_shutdown(NULL);
	}
}

/*
 * SM main
 */
int
main(int argc, char *argv[]) {
	int		c;
	Status_t    status;
	char Opts[] = "Dnzie:d:p:k:r:t:x:y:s:l:C:X:";

	sm_starttime = time(NULL);

	/* Set the default environment name. */
	StringCopy((void *)sm_env, "sm_0", sizeof(sm_env));

	// initialize XML memory pool here since we need it for XML parsing
	status = initSmXmlMemoryPool();
	if (status != VSTATUS_OK) {
		printf("exiting\n");
		exit(2);
	}

	// init callback function for XML parser so it can get pool memory
	initXmlPoolGetCallback(&getSmXmlParserMemory);
	initXmlPoolFreeCallback(&freeSmXmlParserMemory);

	// extract the instance number from the command line
	while ((c = getopt(argc, argv, Opts)) != -1) {
		switch (c) {
		case 'e':
			strncpy((void *)sm_env, optarg, 32);
			break;
		case 'X':
			sscanf(optarg, "%256s", sm_config_filename);
			break;
		case '?':
			IB_LOG_ERROR("invalid command line parameter specified:", optopt);
			exit(1);
		default:
			break;
		}
	}

	// Parse the XML configuration
	status = sm_parse_xml_config();
	if (status != VSTATUS_OK) {
		printf("exiting\n");
		exit(1);
	}

	// Initialize SM memory pool so that we can allocate and copy dg_config data
	// This parallels Esm_Init which also allocates the sm_pool early giving it the ability to
	// allocate memory that will be used for the dg_config copy from the xml_config
	status = sm_initialize_sm_pool();
	if (status != VSTATUS_OK) {
		printf("sm_initialize_sm_pool not successful; exiting\n");
		exit(2);
	}

	// reset getopt()
	optind = 1;

	int overrideLogSetting = 0;

	/* Parse the command line.*/
	while ((c = getopt(argc, argv, Opts)) != -1) {
		switch (c) {
		case 'D':
			if (!xml_trace)
				sm_nodaemon = 0;
			break;
		case 'n':
			sm_nodaemon = 1;
			break;
		case 'd':
			sscanf(optarg, "%u", (uint32_t *)&sm_config.hca);
			break;
		case 'p':
			sscanf(optarg, "%u", (uint32_t *)&sm_config.port);
			break;
		case 'x':
			sscanf(optarg, "%u", (uint32_t *)&sm_lid);
			break;
		case 'k':
			sscanf(optarg, "0x%016"CS64"x", &sm_config.sm_key);
			break;
		case 't':
			sscanf(optarg, "%u", (uint32_t *)&sm_config.timer);
			break;
		case 'r':
			sscanf(optarg, "%u", (uint32_t *)&sm_config.priority);
			break;
		case 'y':
			sscanf(optarg, "%u", (uint32_t *)&sm_config.lmc);
			break;
		case 'z':
			sm_config.debug = 1;
			break;
		case 'e':
			break;
		case 'l':
			sscanf(optarg, "%u", (uint32_t *)&sm_log_level);
			sm_log_level_override = 1;
			overrideLogSetting = 1;
			break;
		case 'C':
			sscanf(optarg, "%u", (uint32_t *)&sm_log_to_console);
			overrideLogSetting = 1;
			break;
		case 'X':
			break;
		}
	}

	if (overrideLogSetting)
		smLogLevelOverride();

#ifdef SIGTERM
    signal(SIGTERM, sm_linux_signal_handler);
#else
	#error SIGNTERM NOT INCLUDED
#endif
#ifdef SIGINT
    signal(SIGINT, sm_linux_signal_handler);
#endif
#ifdef SIGHUP
    signal(SIGHUP, sm_linux_signal_handler);
#endif
#ifdef SIGUSR1
    signal(SIGUSR1, sm_linux_signal_handler);
#endif
#ifdef SIGPIPE
    signal(SIGPIPE, SIG_IGN);   /* 'Inline' failure of wayward readers */
#endif

	/* Start up the independent SM code.*/
	(void)sm_main();
	printf("exiting\n");
	exit(0);
}
