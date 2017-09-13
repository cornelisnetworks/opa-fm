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

#ifndef __OPAMGT_PA_H__
#define __OPAMGT_PA_H__

#ifdef __cplusplus
extern "C" {
#endif

//opamgt includes
#include <iba/stl_pa.h>
#include "opamgt.h"


/** 
 * @brief Get PM configuration data
 *
 * @param port          Port to operate on. 
 * @param pm_config     Pointer to PM config data to fill
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_pm_config(
    struct omgt_port           *port, 
    STL_PA_PM_CFG_DATA  *pm_config
    );

/** 
 * @brief Get image info
 *
 * @param port              Port to operate on. 
 * @param pm_image_Id       Image ID of image info to get 
 * @param pm_image_info     Pointer to image info to fill 
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_image_info(
    struct omgt_port         *port, 
    STL_PA_IMAGE_ID_DATA    pm_image_id,
    STL_PA_IMAGE_INFO_DATA     *pm_image_info 
    );

/** 
 * @brief Get list of group names
 *
 * @param port              Port to operate on. 
 * @param pm_group_list     Pointer to group list to fill 
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_group_list(
    struct omgt_port          *port,
	uint32_t					 *pNum_Groups,
    STL_PA_GROUP_LIST       **pm_group_list
    );

/**
 * @brief Release group list
 *
 * @param pm_group_list         Pointer to pointer to the group list to free.
 *
 * @return
 *   None
 */
void
omgt_pa_release_group_list(
    STL_PA_GROUP_LIST	    **pm_group_list
    );

/**
 * @brief Get list of vf names
 *
 * @param port              Port to operate on.
 * @param pm_vf_list     Pointer to vf list to fill
 *
 * @return
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_vf_list(
    struct omgt_port          *port,
	uint32_t					 *pNum_VFs,
    STL_PA_VF_LIST  		**pm_vf_list
    );

/**
 * @brief Release vf list
 *
 * @param pm_vf_list         Pointer to pointer to the vf list to free.
 *
 * @return
 *   None
 */
void
omgt_pa_release_vf_list(
    STL_PA_VF_LIST	    **pm_vf_list
    );

/** 
 * @brief Get group info
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of group info to get. 
 * @param group_name            Pointer to group name 
 * @param pm_image_id_resp      Pointer to image ID of group info returned. 
 * @param pm_group_info         Pointer to group info to fill.
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_group_info(
    struct omgt_port            *port, 
    STL_PA_IMAGE_ID_DATA        pm_image_id_query,
    char                       *group_name, 
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp, 
    STL_PA_PM_GROUP_INFO_DATA     *pm_group_info
    );

/**
 * @brief Get vf info
 *
 * @param port                  Port to operate on.
 * @param pm_image_id_query     Image ID of vf info to get.
 * @param vf_name               Pointer to vf name
 * @param pm_image_id_resp      Pointer to image ID of vf info returned.
 * @param pm_vf_info            Pointer to vf info to fill.
 *
 * @return
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_vf_info(
    struct omgt_port           *port,
    STL_PA_IMAGE_ID_DATA        pm_image_id_query,
    char                       *vf_name,
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp,
    STL_PA_VF_INFO_DATA        *pm_vf_info
    );

/** 
 * @brief Get group config info
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of group config to get. 
 * @param group_name            Pointer to group name 
 * @param pm_image_id_resp      Pointer to image ID of group info returned. 
 * @param pm_group_config       Pointer to group config to fill. Upon successful return, a memory to 
 *                              contain the group config is allocated. The caller must call
 *                              omgt_pa_release_group_config to free the memory later.
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_group_config(
    struct omgt_port            *port, 
    STL_PA_IMAGE_ID_DATA        pm_image_id_query,
    char                       *group_name, 
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp, 
	uint32_t						*pNum_ports,
    STL_PA_PM_GROUP_CFG_RSP  * *pm_group_config 
    );

/** 
 * @brief Release group config info
 *
 * @param pm_group_config       Pointer to pointer to the group config to free. 
 *
 * @return 
 *   None
 */
void 
omgt_pa_release_group_config(
    STL_PA_PM_GROUP_CFG_RSP    **pm_group_config
    );

/* @brief Get VF config info
 *
 * @param port					Port to operate on
 * @param pm_image_id_query		Image ID of VF config to get.
 * @param vf_name				Pointer to VF name.
 * @param pm_image_id_resp		Pointer to image ID of VF info returned.
 * @param pm_vf_config			Pointer to VF config to fill. Upon successful return, a memory to
 * 								contain the VF config is allocated. The caller must call
 * 								omgt_pa_release_vf_config to free the memory later.
 *
 * 	@return
 * 	  OMGT_STATUS_SUCCESS - Get successful
 * 	    OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_vf_config(
	struct omgt_port				*port,
	STL_PA_IMAGE_ID_DATA		pm_image_id_query,
	char						*vf_name,
	STL_PA_IMAGE_ID_DATA		*pm_image_id_resp,
	uint32_t						*pNum_ports,
	STL_PA_VF_CFG_RSP		**pm_vf_config
	);

/* @brief Release VF config info
 *
 * @param pm_vf_config			Pointer to pointer to the VF config to free
 *
 * @return
 *   None
 */
void
omgt_pa_release_vf_config(
	STL_PA_VF_CFG_RSP 		**pm_vf_config
	);

/**
 * @brief Get group focus portlist
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
 *                              omgt_pa_release_group_focus to free the memory later.
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */

OMGT_STATUS_T
omgt_pa_get_group_focus(
    struct omgt_port           *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                      *group_name, 
    uint32_t                     select, 
    uint32_t                     start, 
    uint32_t                     range,
    STL_PA_IMAGE_ID_DATA      *pm_image_id_resp, 
	uint32_t						*pNum_ports,
    STL_FOCUS_PORTS_RSP      **pm_group_focus 
    );

/** 
 * @brief Release group focus portlist
 *
 * @param pm_group_config       Pointer to pointer to the group focus portlist to free. 
 *
 * @return 
 *   None
 */
void 
omgt_pa_release_group_focus(
    STL_FOCUS_PORTS_RSP      **pm_group_focus
   );

/*
 * @brief Get VF focus portlist
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
 *                              omgt_pa_release_vf_focus to free the memory later.
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */

OMGT_STATUS_T
omgt_pa_get_vf_focus(
    struct omgt_port           *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                      *vf_name, 
    uint32_t                     select, 
    uint32_t                     start, 
    uint32_t                     range,
    STL_PA_IMAGE_ID_DATA      *pm_image_id_resp, 
	uint32_t						*pNum_ports,
    STL_PA_VF_FOCUS_PORTS_RSP     **pm_vf_focus 
    );


/* 
 * @brief Release vf focus portlist
 *
 * @param pm_vf_config       Pointer to pointer to the vf focus portlist to free. 
 *
 * @return 
 *   None
 */
void 
omgt_pa_release_vf_focus(
    STL_PA_VF_FOCUS_PORTS_RSP     **pm_group_focus
   );

/**
 * @brief Get port statistics (counters)
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
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_port_stats(
    struct omgt_port         *port,
    STL_PA_IMAGE_ID_DATA     pm_image_id_query,
    uint16_t                   lid,
    uint8_t                    port_num,
    STL_PA_IMAGE_ID_DATA    *pm_image_id_resp,
    STL_PORT_COUNTERS_DATA  *port_counters,
    uint32_t                  *flags,
    uint32_t                   delta,
	uint32_t                   user_cntrs
    );



/**
 * @brief Get vf port statistics (counters)
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
 *   OMGT_STATUS_SUCCESS - Get successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_get_vf_port_stats(
    struct omgt_port              *port,
    STL_PA_IMAGE_ID_DATA          pm_image_id_query,
	char				         *vf_name,
    uint16_t                        lid,
    uint8_t                         port_num,
    STL_PA_IMAGE_ID_DATA         *pm_image_id_resp,
    STL_PA_VF_PORT_COUNTERS_DATA *vf_port_counters,
    uint32_t                       *flags,
    uint32_t                        delta,
	uint32_t                        user_cntrs
    );

/**
 * @brief Freeze specified image
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to freeze. 
 * @param pm_image_id_resp      Pointer to image ID of image frozen. 
 *
 * @return 
 *   OMGT_STATUS_SUCCESS - Freeze successful
 *     OMGT_STATUS_ERROR - Error
 */
OMGT_STATUS_T
omgt_pa_freeze_image(
    struct omgt_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp 
    );

/**
 * @brief Move freeze of image 1 to image 2
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of frozen image 1. 
 * @param pm_image_id_resp      Pointer to image ID of image2. 
 *  
 * @return 
 *   OMGT_STATUS_SUCCESS       - Move image Freeze successful
 *   OMGT_STATUS_UNAVAILABLE   - Image 2 unavailable freeze
 *     OMGT_STATUS_ERROR       - Error
 */
OMGT_STATUS_T
omgt_pa_move_image_freeze(
    struct omgt_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id1,
    STL_PA_IMAGE_ID_DATA       *pm_image_Id2 
    );

/**
 * @brief Release specified image.
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to release. 
 *  
 * @return 
 *   OMGT_STATUS_SUCCESS       - Release successful
 *     OMGT_STATUS_ERROR       - Error
 */
OMGT_STATUS_T
omgt_pa_release_image(
    struct omgt_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query
    );

/**
 * @brief Renew lease of specified image.
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to renew. 
 *  
 * @return 
 *   OMGT_STATUS_SUCCESS       - Renew successful
 *     OMGT_STATUS_ERROR       - Error
 */
OMGT_STATUS_T
omgt_pa_renew_image(
    struct omgt_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query
    );


/**
 * @brief Get the classportinfo from the given port.
 *
 * @param port                  Local port to operate on.  
 * @param cpi                   A pointer to the ClassPortInfo. The caller must free it after use.
 *  
 * @return 
 *   OMGT_STATUS_SUCCESS       - Renew successful
 *     OMGT_STATUS_ERROR       - Error
 */
OMGT_STATUS_T
omgt_pa_get_classportinfo(
    struct omgt_port  *port,
	STL_CLASS_PORT_INFO **cpi
    );


/*
 * @brief Get MAD status code from most recent PA operation
 *
 * @param port                    Local port to operate on. 
 * 
 * @return 
 *   The corresponding status code.
 */
uint16_t
omgt_get_pa_mad_status(
    struct omgt_port              *port
    );

#ifdef __cplusplus
}
#endif

#endif /* __OPAMGT_PA_H__ */
