/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef __OIB_UTILS_PA_H__
#define __OIB_UTILS_PA_H__

#include <iba/ibt.h>
#include <iba/ipublic.h>
#include <ib_pa.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <ctype.h>

#include "iba/public/datatypes.h"

// DEFINES
#define PA_REQ_HEADER_SIZE (sizeof(MAD_COMMON) + sizeof(SA_MAD_HDR))

#define PACLIENT_UNKNOWN        0
#define PACLIENT_OPERATIONAL    1
#define PACLIENT_DOWN           (-1)
#define PACLIENT_INVALID_PARAM  (-2)

#define PACLIENT_IMAGE_CURRENT  0   // Most recent sweep image
#define PACLIENT_IMAGE_TIMED   -1   // Image with particular time

#define PACLIENT_SEL_ALL                0x00010000  // All ports in group
#define PACLIENT_SEL_UTIL_HIGH          0x00020001  // Highest first
#define PACLIENT_SEL_UTIL_PKTS_HIGH     0x00020082  // Highest first
#define PACLIENT_SEL_UTIL_LOW           0x00020101  // Lowest first
#define PACLIENT_SEL_ERR_INTEG          0x00030001  // Highest first
#define PACLIENT_SEL_ERR_CONGST         0x00030002  // Highest first
#define PACLIENT_SEL_ERR_SMACONG        0x00030003  // Highest first
#define PACLIENT_SEL_ERR_BUBBLE			0x00030004	// Highest first
#define PACLIENT_SEL_ERR_SECURE         0x00030005  // Highest first
#define PACLIENT_SEL_ERR_ROUTING        0x00030006  // Highest first

struct oib_port;

/***************************************************************************
 * OIB utils PA interface from paClient.h
 ***************************************************************************/

/**
 * initialize Pa Client
 *
 * @param port          Pointer to the port to operate on. Set upon successful exit. 
 * @param hfi_num       The host channel adaptor number to use to access fabric
 * @param port_num      The port number to use to access fabric.
 * @param verbose_file  Pointer to FILE for verbose output
 *
 * @return 
 *  PACLIENT_OPERATIONAL - initialization successful
 *         PACLIENT_DOWN - initialization not successful/PaServer not available 
 */
int 
oib_pa_client_init(
    struct oib_port     **port, 
    int                 hfi_num, 
    uint8_t             port_num,
    FILE                *verbose_file
    );

/**
 * initialize Pa Client
 *
 * @param port          Pointer to the port to operate on. Set upon successful exit. 
 * @param port_guid     The port guid to use to access fabric.
 * @param verbose_file  Pointer to FILE for verbose output
 *
 * @return 
 *  PACLIENT_OPERATIONAL - initialization successful
 *         PACLIENT_DOWN - initialization not successful/PaServer not available 
 */
int 
oib_pa_client_init_by_guid(
    struct oib_port     **port, 
    uint64_t            port_guid,
    FILE                *verbose_file
    );

/** 
 *  Get PM configuration data
 *
 * @param port          Port to operate on. 
 * @param pm_config     Pointer to PM config data to fill
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_get_pm_config(
    struct oib_port           *port, 
    STL_PA_PM_CFG_DATA  *pm_config
    );

/** 
 *  Get image info
 *
 * @param port              Port to operate on. 
 * @param pm_image_Id       Image ID of image info to get 
 * @param pm_image_info     Pointer to image info to fill 
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_get_image_info(
    struct oib_port         *port, 
    STL_PA_IMAGE_ID_DATA    pm_image_id,
    STL_PA_IMAGE_INFO_DATA     *pm_image_info 
    );

/** 
 *  Get list of group names
 *
 * @param port              Port to operate on. 
 * @param pm_group_list     Pointer to group list to fill 
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS
pa_client_get_group_list(
    struct oib_port          *port,
	uint32					 *pNum_Groups,
    STL_PA_GROUP_LIST       **pm_group_list
    );

/**
 *  Release group list
 *
 * @param port                  Port to operate on.
 * @param pm_group_list         Pointer to pointer to the group list to free.
 *
 * @return
 *   None
 */
void
pa_client_release_group_list(
    struct oib_port          *port,
    STL_PA_GROUP_LIST	    **pm_group_list
    );

/**
 *  Get list of vf names
 *
 * @param port              Port to operate on.
 * @param pm_vf_list     Pointer to vf list to fill
 *
 * @return
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS
pa_client_get_vf_list(
    struct oib_port          *port,
	uint32					 *pNum_VFs,
    STL_PA_VF_LIST  		**pm_vf_list
    );

/**
 *  Release vf list
 *
 * @param port                  Port to operate on.
 * @param pm_vf_list         Pointer to pointer to the vf list to free.
 *
 * @return
 *   None
 */
void
pa_client_release_vf_list(
    struct oib_port      *port,
    STL_PA_VF_LIST	    **pm_vf_list
    );

/** 
 *  Get group info
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of group info to get. 
 * @param group_name            Pointer to group name 
 * @param pm_image_id_resp      Pointer to image ID of group info returned. 
 * @param pm_group_info         Pointer to group info to fill.
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_get_group_info( 
    struct oib_port            *port, 
    STL_PA_IMAGE_ID_DATA        pm_image_id_query,
    char                       *group_name, 
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp, 
    STL_PA_PM_GROUP_INFO_DATA     *pm_group_info
    );

/** 
 *  Get group config info
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of group config to get. 
 * @param group_name            Pointer to group name 
 * @param pm_image_id_resp      Pointer to image ID of group info returned. 
 * @param pm_group_config       Pointer to group config to fill. Upon successful return, a memory to 
 *                              contain the group config is allocated. The caller must call
 *                              pa_client_release_group_config to free the memory later. 
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_get_group_config( 
    struct oib_port            *port, 
    STL_PA_IMAGE_ID_DATA        pm_image_id_query,
    char                       *group_name, 
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp, 
	uint32						*pNum_ports,
    STL_PA_PM_GROUP_CFG_RSP  * *pm_group_config 
    );

/** 
 *  Release group config info
 *
 * @param port                  Port to operate on. 
 * @param pm_group_config       Pointer to pointer to the group config to free. 
 *
 * @return 
 *   None
 */
void 
pa_client_release_group_config(
    struct oib_port             *port, 
    STL_PA_PM_GROUP_CFG_RSP    **pm_group_config
    );

/* Get VF config info
 *
 * @param port					Port to operate on
 * @param pm_image_id_query		Image ID of VF config to get.
 * @param vf_name				Pointer to VF name.
 * @param pm_image_id_resp		Pointer to image ID of VF info returned.
 * @param pm_vf_config			Pointer to VF config to fill. Upon successful return, a memory to
 * 								contain the VF config is allocated. The caller must call
 * 								pa_client_release_vf_config to free the memory later.
 *
 * 	@return
 * 	  FSUCCESS - Get successful
 * 	    FERROR - Error
 */
FSTATUS
pa_client_get_vf_config(
	struct oib_port				*port,
	STL_PA_IMAGE_ID_DATA		pm_image_id_query,
	char						*vf_name,
	STL_PA_IMAGE_ID_DATA		*pm_image_id_resp,
	uint32						*pNum_ports,
	STL_PA_VF_CFG_RSP		**pm_vf_config
	);

/* Release VF config info
 *
 * @param port					Port to operate on.
 * @param pm_vf_config			Pointer to pointer to the VF config to free
 *
 * @return
 *   None
 */
void
pa_client_release_vf_config(
	struct oib_port				*port,
	STL_PA_VF_CFG_RSP 		**pm_vf_config
	);

/**
 *  Get group focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of group focus portlist to get. 
 * @param group_name            Pointer to group name.
 * @param select                Select value for focus portlist. 
 * @param start                 Start index value of portlist 
 * @param range                 Index range of portlist. 
 * @param pm_image_id_resp      Pointer to image ID of group focus portlist returned. 
 * @param pm_group_focus        Pointer to pointer to focus portlist to fill. Upon 
 *                              successful return, a memory to contain the group focus
 *                              portlist is allocated. The caller must call
 *                              pa_client_release_group_focus to free the memory later.
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */

FSTATUS 
pa_client_get_group_focus( 
    struct oib_port           *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                      *group_name, 
    uint32                     select, 
    uint32                     start, 
    uint32                     range,
    STL_PA_IMAGE_ID_DATA      *pm_image_id_resp, 
	uint32						*pNum_ports,
    STL_FOCUS_PORTS_RSP      **pm_group_focus 
    );

/** 
 *  Release group focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_group_config       Pointer to pointer to the group focus portlist to free. 
 *
 * @return 
 *   None
 */
void 
pa_client_release_group_focus(
    struct oib_port           *port, 
    STL_FOCUS_PORTS_RSP      **pm_group_focus
   );

/*
 *  Get VF focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of vf focus portlist to get. 
 * @param vf_name            Pointer to vf name.
 * @param select                Select value for focus portlist. 
 * @param start                 Start index value of portlist 
 * @param range                 Index range of portlist. 
 * @param pm_image_id_resp      Pointer to image ID of group focus portlist returned. 
 * @param pm_vf_focus        Pointer to pointer to focus portlist to fill. Upon 
 *                              successful return, a memory to contain the group focus
 *                              portlist is allocated. The caller must call
 *                              pa_client_release_vf_focus to free the memory later.
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */

FSTATUS 
pa_client_get_vf_focus( 
    struct oib_port           *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                      *vf_name, 
    uint32                     select, 
    uint32                     start, 
    uint32                     range,
    STL_PA_IMAGE_ID_DATA      *pm_image_id_resp, 
	uint32						*pNum_ports,
    STL_PA_VF_FOCUS_PORTS_RSP     **pm_vf_focus 
    );


/* 
 *  Release vf focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_vf_config       Pointer to pointer to the vf focus portlist to free. 
 *
 * @return 
 *   None
 */
void 
pa_client_release_vf_focus(
    struct oib_port           *port, 
    STL_PA_VF_FOCUS_PORTS_RSP     **pm_group_focus
   );

/**
 *  Get port statistics (counters)
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of port counters to get. 
 * @param lid                   LID of node.
 * @param port_num              Port number. 
 * @param pm_image_id_resp      Pointer to image ID of port counters returned. 
 * @param port_counters         Pointer to port counters to fill. 
 * @param flags                 Pointer to flags
 * @param delta                 1 for delta counters, 0 for raw image counters.
 * @param user_cntrs            1 for running counters, 0 for image counters. (delta must be 0)
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS
pa_client_get_port_stats(
    struct oib_port         *port,
    STL_PA_IMAGE_ID_DATA     pm_image_id_query,
    uint16                   lid,
    uint8                    port_num,
    STL_PA_IMAGE_ID_DATA    *pm_image_id_resp,
    STL_PORT_COUNTERS_DATA  *port_counters,
    uint32                  *flags,
    uint32                   delta,
	uint32                   user_cntrs
    );

/**
 *  Clear specified port counters for specified port
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of port counters to clear. 
 * @param lid                   LID of port.
 * @param port_num              Port number. 
 * @param selct_flags           Port's counters to clear. 
 *
 * @return 
 *   FSUCCESS - Clear successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_clr_port_counters( 
    struct oib_port        *port,
    STL_PA_IMAGE_ID_DATA    pm_image_id_query, 
    uint16                  lid,
    uint8                   port_num, 
    uint32                  select_flag 
    );

/**
 *  Get vf port statistics (counters)
 *
 * @param port                  Port to operate on.
 * @param pm_image_id_query     Image ID of port counters to get.
 * @param vf_name				Pointer to VF name.
 * @param lid                   LID of node.
 * @param port_num              Port number.
 * @param pm_image_id_resp      Pointer to image ID of port counters returned.
 * @param vf_port_counters      Pointer to vf port counters to fill.
 * @param flags                 Pointer to flags
 * @param delta                 1 for delta counters, 0 for raw image counters.
 * @param user_cntrs            1 for running counters, 0 for image counters. (delta must be 0)
 *
 * @return
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS
pa_client_get_vf_port_stats(
    struct oib_port              *port,
    STL_PA_IMAGE_ID_DATA          pm_image_id_query,
	char				         *vf_name,
    uint16                        lid,
    uint8                         port_num,
    STL_PA_IMAGE_ID_DATA         *pm_image_id_resp,
    STL_PA_VF_PORT_COUNTERS_DATA *vf_port_counters,
    uint32                       *flags,
    uint32                        delta,
	uint32                        user_cntrs
    );

/**
 *  Freeze specified image
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to free. 
 * @param pm_image_id_resp      Pointer to image ID of image frozen. 
 *
 * @return 
 *   FSUCCESS - Freeze successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_freeze_image( 
    struct oib_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp 
    );

/**
 *   Move freeze of image 1 to image 2
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of frozen image 1. 
 * @param pm_image_id_resp      Pointer to image ID of image2. 
 *  
 * @return 
 *   FSUCCESS       - Move image Freeze successful
 *   FUNAVAILABLE   - Image 2 unavailable freeze
 *     FERROR       - Error
 */
FSTATUS 
pa_client_move_image_freeze(
    struct oib_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id1,
    STL_PA_IMAGE_ID_DATA       *pm_image_Id2 
    );

/**
 *   Release specified image.
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to release. 
 *  
 * @return 
 *   FSUCCESS       - Release successful
 *     FERROR       - Error
 */
FSTATUS 
pa_client_release_image(
    struct oib_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query
    );

/**
 *   Renew lease of specified image.
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to renew. 
 *  
 * @return 
 *   FSUCCESS       - Renew successful
 *     FERROR       - Error
 */
FSTATUS 
pa_client_renew_image(
    struct oib_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query
    );


/************************************************************ 
 * The following come from iba2ibo_paquery_helper.h 
 * Mainly called by opapaquery tool and PA client above. 
 ************************************************************/

/**
 *   Enable debugging output in the oib_utils library.
 *
 * @param   port    The local port to operate on.
 *  
 * @return 
 *   None
 */
void 
set_opapaquery_debug (
    IN struct oib_port  *port
    );

/**
 *  Get the classportinfo from the given port.
 *
 * @param port                  Local port to operate on.  
 *  
 * @return 
 *   Valid pointer  -   A pointer to the ClassPortInfo. The caller must free it after use.
 *   NULL           -   Error.
 */
STL_CLASS_PORT_INFO *
iba_pa_classportinfo_response_query(
    IN struct oib_port  *port
    );

/**
 *  Get port statistics (counters)
 *
 * @param port                  Local port to operate on. 
 * @param node_lid              Remote node LID.
 * @param port_num              Remote port number. 
 * @param port_counters         Pointer to port counters to fill. 
 * @param delta_flag            1 for delta counters, 0 for raw image counters.
 * @param user_cntrs_flag       1 for running counters, 0 for image counters. (delta must be 0)
 * @param image_id              Pointer to image ID of port counters to get. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the counters data. The caller must free it.
 *     NULL         - Error
 */
STL_PORT_COUNTERS_DATA *
iba_pa_single_mad_port_counters_response_query(
    IN struct oib_port  *port,
    IN uint32_t          node_lid,
    IN uint8_t           port_number,
    IN uint32_t          delta_flag,
    IN uint32_t          user_cntrs_flag,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    );
/**
 *  Clear port statistics (counters)
 *
 * @param port                  Local Port to operate on. 
 * @param node_lid              Remote node LID.
 * @param port_num              Remote port number. 
 * @param select                Port's counters to clear. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the clear port counters data (containing the
 *                    info as which counters have been cleared).  The caller must free it.
 *     NULL         - Error
 */
STL_CLR_PORT_COUNTERS_DATA *
iba_pa_single_mad_clr_port_counters_response_query(
    IN struct oib_port  *port,
    IN uint32_t         node_lid,
    IN uint8_t         port_number,
    IN uint32_t         select
    );

/**
 *  Clear all ports, statistics (counters)
 *
 * @param port                  Local port to operate on. 
 * @param select                Port's counters to clear. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the clear all port counters data (containing the
 *                    info as which counters have been cleared).  The caller must free it.
 *     NULL         - Error
 */
STL_CLR_ALL_PORT_COUNTERS_DATA *
iba_pa_single_mad_clr_all_port_counters_response_query(
    IN struct oib_port  *port,
    IN uint32_t         select
    );


/**
 *  Get the PM config data.
 *
 * @param port                  Local port to operate on. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for pm config data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_PM_CFG_DATA *
iba_pa_single_mad_get_pm_config_response_query(
    IN struct oib_port  *port
    );

/**
 *  Free a sweep image.
 *
 * @param port                  Local port to operate on. 
 * @param image_id              ID of image to freeze. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the frozen image id data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_ID_DATA *
iba_pa_single_mad_freeze_image_response_query(
    IN struct oib_port  *port,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    );

/**
 *  Release a sweep image.
 *
 * @param port                  Local port to operate on. 
 * @param image_id              Ponter to id of image to release. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the released image id data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_ID_DATA *
iba_pa_single_mad_release_image_response_query(
    IN struct oib_port  *port,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    );

/**
 *  Renew a sweep image.
 *
 * @param port                  Local port to operate on. 
 * @param image_id              Ponter to id of image to renew. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the renewed image id data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_ID_DATA *
iba_pa_single_mad_renew_image_response_query(
    IN struct oib_port  *port,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    );

/**
 *  Move a frozen image 1 to image 2.
 *
 * @param port                  Local port to operate on. 
 * @param move_info             Ponter to move info (src and dest image ID). 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the image  move info.  The caller must free it.
 *     NULL         - Error
 */
STL_MOVE_FREEZE_DATA *
iba_pa_single_mad_move_freeze_response_query(
    IN struct oib_port  *port,
    IN STL_MOVE_FREEZE_DATA *move_info
    );

/**
 *  Get image info
 *
 * @param port                  Local port to operate on. 
 * @param image_info            Ponter to image info (containing valid image ID). 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the image info data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_INFO_DATA *
iba_pa_multi_mad_get_image_info_response_query (
    IN struct oib_port  *port,
    IN STL_PA_IMAGE_INFO_DATA  *image_info
    );

/**
 *  Get master pm LID
 *
 * @param port                  Local port to operate on. 
 * @param primary_pm_lid        Pointer to master pm lid to be filled. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_query_master_pm_lid(
    IN struct oib_port  *port,
    OUT uint16_t        *primary_pm_lid
    );

/**
 *  Get multi-record response for pa group data (group list).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to free the buffer. 
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_group_list_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL
    );

/**
 *  Get multi-record response for pa group info (stats).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param group_name                Group name 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to free the buffer.
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 * @param image_id                  Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_group_stats_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *group_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA                *image_id
    );

/**
 *  Get multi-record response for pa group config.
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param group_name                Group name 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to free the buffer. 
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 * @param imageID                   Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_group_config_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *group_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA                *image_id
    );

/**
 *  Get multi-record response for pa group focus portlist.
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param group_name                Group name 
 * @param select                    Select value for focus portlist. 
 * @param start                     Start index value of portlist 
 * @param range                     Index range of portlist.
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to fill the buffer. 
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 * @param imageID                   Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_focus_ports_response_query (
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *group_name,
    IN uint32                       select,
    IN uint32                       start,
    IN uint32                       range,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA                *image_id
    );


/**
 *  Get multi-record response for pa vf data (vf list).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to free the buffer. 
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_vf_list_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL
    );

/**
 *  Get multi-record response for pa vf info (vf info).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to free the buffer.
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS
iba_pa_multi_mad_vf_info_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *vf_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA                *image_id
    );

/** 
 *  Get vf config info
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of vf config to get. 
 * @param vf_name            	Pointer to vf name 
 * @param pm_image_id_resp      Pointer to image ID of vf info returned. 
 * @param pm_vf_config       	Pointer to vf config to fill. Upon successful return, a memory to 
 *                              contain the vf config is allocated. The caller must call
 *                              pa_client_release_vf_config to free the memory later. 
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS
iba_pa_multi_mad_vf_config_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *vf_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA                *image_id
    );

STL_PA_VF_PORT_COUNTERS_DATA *
iba_pa_single_mad_vf_port_counters_response_query(
    IN struct oib_port      *port,
    IN uint32_t              node_lid,
    IN uint8_t               port_number,
    IN uint32_t              delta_flag,
    IN uint32_t              user_cntrs_flag,
    IN char                 *vfName,
    IN STL_PA_IMAGE_ID_DATA *image_id
    );

STL_PA_CLEAR_VF_PORT_COUNTERS_DATA *
iba_pa_single_mad_clr_vf_port_counters_response_query(
    IN struct oib_port  *port,
    IN uint32_t         node_lid,
    IN uint8_t         port_number,
    IN uint32_t         select,
    IN char             *vfName
    );

FSTATUS
iba_pa_multi_mad_vf_focus_ports_response_query (
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *vf_name,
    IN uint32                       select,
    IN uint32                       start,
    IN uint32                       range,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA                *image_id
    );


/**
 *  Get and Convert a MAD status code into string.
 *
 * @param port                    Local port to operate on. 
 * 
 * @return 
 *   The corresponding status string.
 */
const char* 
iba_pa_mad_status_msg(
    IN struct oib_port              *port
    );

#endif /* __OIB_UTILS_PA_H__ */
