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

/************************************************************************
 *									 
 * FILE NAME								 
 *    vssappl.c								 
 *									 
 * DESCRIPTION								 
 *									 
 *    This file constructs a compatibilty library for to allow 
 *    application programs using VIEO vss framework functions to
 *    be run as normal user space applications without requiring the
 *    loading of the drivers. 
 *									 
 *									 
 * Functions defined							 
 *									 
 *			    ** Logging functions
 *									 
 *    vs_log_args		multi argument logging call
 *
 *			    ** OS abstraction calls
 *									 
 *    vs_pool_create	initialize the pool subsystem
 *    vs_pool_delete	delete a pool subsystem
 *    vs_pool_alloc		allocate a buffer
 *    vs_pool_free		free a buffer
 *
 * DEPENDENCIES								 
 *									 
 *     stdio.h 
 *     string.h 
 *     ib_types.h 
 *     ib_status.h 
 *     cs_g.h 
 *									 
 * HISTORY								 
 *									 
 *    NAME	DATE  REMARKS						 
 *     joc   3/14/02  Initial creation of file.				 
 *     dkj   4/02/02  Updated OS API 2.0g vs_pool_page_size prototype
 ************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <netinet/in.h>


#include "ib_types.h"
#include "ib_status.h"
#include "vs_g.h"
#include "cs_g.h"

#define MAX_LOG_OUT	256		/* Max entry output size	*/

extern	void *malloc(size_t);
extern	void free(void *);
void hexout(uint8_t *,int);

uint32_t cs_log_masks[VIEO_LAST_MOD_ID+1] = {
		DEFAULT_LOG_MASK,	// VIEO_NONE_MOD_ID for vs_syslog_output_message
		0,					// VIEO_CS_MOD_ID   /* Library Modules */
		DEFAULT_LOG_MASK,	// VIEO_MAI_MOD_ID  
		DEFAULT_LOG_MASK,	// VIEO_CAL_MOD_ID 
		DEFAULT_LOG_MASK,	// VIEO_DRIVER_MOD_ID
		DEFAULT_LOG_MASK,	// VIEO_IF3_MOD_ID  
		DEFAULT_LOG_MASK,	// VIEO_SM_MOD_ID  /* Subnet Mgr */
		DEFAULT_LOG_MASK,	// VIEO_SA_MOD_ID  /* Subnet Administrator */
		DEFAULT_LOG_MASK,	// VIEO_PM_MOD_ID  /* Performance Mgr */
		DEFAULT_LOG_MASK,	// VIEO_PA_MOD_ID  /* Performance Administrator */
		DEFAULT_LOG_MASK,	// VIEO_BM_MOD_ID  /* Baseboard Mgr */
		DEFAULT_LOG_MASK,	// VIEO_FE_MOD_ID  /* Fabric Executive */
		DEFAULT_LOG_MASK,	// VIEO_APP_MOD_ID /* Generic VIEO mod id */
};

void
vs_log_output(uint32_t sev, /* severity */
	    uint32_t modid,	/* optional Module id */
		const char *function, /* optional function name */
		const char *vf,	/* optional vFabric name */
		const char *format, ...
		)
{
	char buffer[1024];
	va_list args;
	char vfstr[128];
	const char *sev_str;

  	switch(sev)
    {
    case VS_LOG_FATAL: /* Fatal Error 			*/
		sev_str = "FATAL";
    	break;
    case VS_LOG_CSM_ERROR: /* CSM Error 			*/
		sev_str = "ERROR";
    	break;
    case VS_LOG_CSM_WARN: /* CSM WARN 			*/
		sev_str = "WARN";
    	break;
    case VS_LOG_CSM_NOTICE: /* CSM IB_NOTICE 			*/
		sev_str = "NOTICE";
    	break;
    case VS_LOG_CSM_INFO: /* CSM INFO 			*/
		sev_str = "INFO";
    	break;
    case VS_LOG_ERROR: /* Log error data 		*/
		sev_str = "ERROR";
    	break;
    case VS_LOG_WARN: /* Log warn information 	*/
		sev_str = "WARN";
    	break;
    case VS_LOG_NOTICE: /* Log notice information 	*/
		sev_str = "NOTICE";
    	break;
    case VS_LOG_INFINI_INFO: /* Log information 		*/
		sev_str = "PROGRESS";
    	break;
    case VS_LOG_INFO: /* Log information 		*/
		sev_str = "INFO";
    	break;
    case VS_LOG_VERBOSE: /* Log verbose */
		sev_str = "VRBSE";
    	break;
    case VS_LOG_DATA: /* Log binary data		*/
		sev_str = "DATA";
    	break;
    case VS_LOG_DEBUG1: /* Log debug1 */
		sev_str = "DBG1 ";
    	break;
    case VS_LOG_DEBUG2: /* Log debug2 */
		sev_str = "DBG2 ";
    	break;
    case VS_LOG_DEBUG3: /* Log debug3 */
		sev_str = "DBG3 ";
    	break;
    case VS_LOG_DEBUG4: /* Log debug4 */
		sev_str = "DBG4 ";
    	break;
    case VS_LOG_ENTER: /* Function entry 		*/
		sev_str = "ENTER";
    	break;
    case VS_LOG_ARGS:
		sev_str = "ARGS";
    	break;
    case VS_LOG_EXIT: /* Function exit 		*/
		sev_str = "EXIT";
    	break;
    default:
        return;
    	break;
    }
      
	va_start(args, format);
#ifdef __VXWORKS__
	(void) vsprintf (buffer, format, args);
#else
	(void) vsnprintf (buffer, sizeof(buffer), format, args);
#endif
	va_end (args);
	buffer[sizeof(buffer)-1] = '\0';

	if (vf) {
		snprintf(vfstr, sizeof(vfstr), "[VF:%s] ", vf);
		vfstr[sizeof(vfstr)-1] = '\0';
	} else
		*vfstr='\0';

	fprintf(stderr, "%s: %s%s%s%s\n",
				sev_str, vfstr,
			   	function?function:"", function?": ":"",
				buffer);
}

Status_t vs_pool_create (Pool_t *poolp, 
		uint32_t options, 
		unsigned char *name, 
		void *address, 
		size_t size) 
{

       /* This function is not needed in the library.
 	* just absorb the call and return
 	*/

  return VSTATUS_OK;

}

Status_t vs_pool_delete(Pool_t *poolp) 
{

       /* This function is not needed in the library.
 	* just absorb the call and return
 	*/

  return VSTATUS_OK;

}

Status_t vs_pool_alloc(Pool_t *poolp, size_t size, void **address) 
{
  uint8_t	       *bufferp;

  bufferp = (uint8_t *)malloc(size);
  if (bufferp == NULL) 
	{
	IB_LOG_ERROR0 ("vs_pool_alloc");
	return(VSTATUS_NOMEM);
	}

  *address = bufferp;

  return(VSTATUS_OK);
}


Status_t vs_pool_free(Pool_t *poolp, void *address) 
{
  free((void *)address);
  return VSTATUS_OK;
}

size_t vs_pool_page_size (void)
{

       /* This function is not needed in the library.
 	* just absorb the call and return
 	*/

  return (size_t) 4096;
}


/***********************************************************************/
/**                                                                   **/
/**  hexout(cmd,cc)   - Debug hex dump routine			      **/
/**                                                                   **/
/***********************************************************************/

#define HSTR_LEN 96
void hexout(uint8_t *cmd, int cc) {
	int    i,j;
	char   hstr[HSTR_LEN+5];
	char   astr[19];
	int    tcnt;

	tcnt = 0;
	fprintf(stderr, "-----------------\n");
	while(cc) {
		cs_strlcpy(astr, "[................]", sizeof(hstr));
		cs_strlcpy(hstr, "", sizeof(astr));
		
		j = 0;
		for(i=0;i<16 && cc>0;i++){
			cc--;
			j += snprintf(hstr + j, 6, "%02X%s", cmd[i], i == 7 ? " - " : " ");
			if(isprint(cmd[i])){
				astr[i+1] = cmd[i];
			}
		}
		cmd += 16;
		fprintf(stderr, "%04X: %-50s %s\n", tcnt, hstr, astr);
		tcnt += 16;
	}
	fprintf(stderr,"\n");
}
