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


#ifndef __OPAMGT_PRIV_PA_H__
#define __OPAMGT_PRIV_PA_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <ctype.h>
#include "opamgt_priv.h"
#include <iba/stl_pa_priv.h>
#include <iba/stl_sd.h>

#define PA_REQ_HEADER_SIZE (sizeof(MAD_COMMON) + sizeof(SA_MAD_HDR))

/**
 * @brief PA query input type to string
 * @param code          Input type code value
 * @return
 *  Input type static string
 */
const char*
iba_pa_query_input_type_msg(
    QUERY_INPUT_TYPE    code
    );

/**
 * @brief PA query output type to string
 * @param code          Output type code value
 * @return
 *  Output type static string
 */
const char*
iba_pa_query_result_type_msg(
    QUERY_RESULT_TYPE   code
    );

/**
 * @brief Initialize PA connection on existing omgt_port session
 *
 * @param port             omgt_port session to start PA interface on
 *
 * @return
 *  PACLIENT_OPERATIONAL - initialization successful
 *         PACLIENT_DOWN - initialization not successful/PaServer not available
 */
int
omgt_pa_service_connect(
    struct omgt_port     *port
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
    struct omgt_port        *port,
    STL_PA_IMAGE_ID_DATA    pm_image_id_query, 
    uint16                  lid,
    uint8                   port_num, 
    uint32                  select_flag 
    );

/************************************************************ 
 * The following come from iba2ibo_paquery_helper.h 
 * Mainly called by opapaquery tool and PA client above. 
 ************************************************************/

/**
 *   Enable debugging output in the opamgt library.
 *
 * @param   port    The local port to operate on.
 *  
 * @return 
 *   None
 */
void 
set_opapaquery_debug (
    IN struct omgt_port  *port
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
    IN struct omgt_port  *port,
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
    IN struct omgt_port  *port,
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
    IN struct omgt_port  *port,
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
    IN struct omgt_port  *port
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
    IN struct omgt_port  *port,
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
    IN struct omgt_port  *port,
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
    IN struct omgt_port  *port,
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
    IN struct omgt_port  *port,
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
    IN struct omgt_port  *port,
    IN STL_PA_IMAGE_INFO_DATA  *image_info
    );

/**
 *  Get master pm LID
 *
 * @param port                  Local port to operate on.
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_query_master_pm_lid(struct omgt_port *port);

/**
 *  Get multi-record response for pa group data (group list).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to free the buffer. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_group_list_response_query(
    IN struct omgt_port              *port,
    IN POMGT_QUERY                  query,
    OUT PQUERY_RESULT_VALUES        *pquery_result
    );

/**
 *  Get multi-record response for pa group info (stats).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param group_name                Group name 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to free the buffer.
 * @param image_id                  Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_group_stats_response_query(
    IN struct omgt_port              *port,
    IN POMGT_QUERY                  query,
    IN char                         *group_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
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
 * @param imageID                   Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_group_config_response_query(
    IN struct omgt_port              *port,
    IN POMGT_QUERY                  query,
    IN char                         *group_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
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
 * @param imageID                   Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_focus_ports_response_query (
    IN struct omgt_port              *port,
    IN POMGT_QUERY                  query,
    IN char                         *group_name,
    IN uint32                       select,
    IN uint32                       start,
    IN uint32                       range,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN STL_PA_IMAGE_ID_DATA                *image_id
    );


/**
 *  Get multi-record response for pa vf data (vf list).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to free the buffer.
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_vf_list_response_query(
    IN struct omgt_port              *port,
    IN POMGT_QUERY                  query,
    OUT PQUERY_RESULT_VALUES        *pquery_result
    );

/**
 *  Get multi-record response for pa vf info (vf info).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to free the buffer.
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS
iba_pa_multi_mad_vf_info_response_query(
    IN struct omgt_port              *port,
    IN POMGT_QUERY                  query,
    IN char                         *vf_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
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
 *                              omgt_pa_release_vf_config to free the memory later.
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS
iba_pa_multi_mad_vf_config_response_query(
    IN struct omgt_port              *port,
    IN POMGT_QUERY                  query,
    IN char                         *vf_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN STL_PA_IMAGE_ID_DATA                *image_id
    );

STL_PA_VF_PORT_COUNTERS_DATA *
iba_pa_single_mad_vf_port_counters_response_query(
    IN struct omgt_port      *port,
    IN uint32_t              node_lid,
    IN uint8_t               port_number,
    IN uint32_t              delta_flag,
    IN uint32_t              user_cntrs_flag,
    IN char                 *vfName,
    IN STL_PA_IMAGE_ID_DATA *image_id
    );

STL_PA_CLEAR_VF_PORT_COUNTERS_DATA *
iba_pa_single_mad_clr_vf_port_counters_response_query(
    IN struct omgt_port  *port,
    IN uint32_t         node_lid,
    IN uint8_t         port_number,
    IN uint32_t         select,
    IN char             *vfName
    );

FSTATUS
iba_pa_multi_mad_vf_focus_ports_response_query (
    IN struct omgt_port              *port,
    IN POMGT_QUERY                  query,
    IN char                         *vf_name,
    IN uint32                       select,
    IN uint32                       start,
    IN uint32                       range,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
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
    IN struct omgt_port              *port
    );

#endif //__OPAMGT_PRIV_PA_H__
