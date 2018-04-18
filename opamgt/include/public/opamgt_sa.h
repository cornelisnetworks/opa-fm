/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef __OPAMGT_SA_H__
#define __OPAMGT_SA_H__

#ifdef __cplusplus
extern "C" {
#endif

//opamgt includes
#include "opamgt.h"
#include <iba/stl_sd.h>

typedef struct omgt_sa_selector {
	QUERY_INPUT_TYPE InputType;		/* Type of input (i.e. query based on) */
	OMGT_QUERY_INPUT_VALUE InputValue;	/* input record selection value input query */
} omgt_sa_selector_t;

/*
 * @brief Get MAD status code from most recent SA operation
 *
 * @param port			Local port to operate on.
 *
 * @return
 *	 The corresponding status code.
 */
uint16_t
omgt_get_sa_mad_status(struct omgt_port *port);


/**
 * @brief Free memory associated with an SA query result
 *
 * @param record		 pointer to records returned from omgt_sa_get_* call
 *
 */
void
omgt_sa_free_records(void *record);

/**
 * @brief Query SA for Node Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid, PortGuid, NodeGuid, SystemImageGuid,
 *                		 	NodeType, NodeDesc
 *                		 	(e.g. InputTypeNoInput)
 * @param num_records	 Output: The number of records returned in query
 * @param records	 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */

OMGT_STATUS_T
omgt_sa_get_node_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_NODE_RECORD **records
	);
/**
 * @brief Query SA for PortInfo Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records	 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */

OMGT_STATUS_T
omgt_sa_get_portinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_PORTINFO_RECORD **records
	);


/**
 * @brief Query SA for System Image GUIDs
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select GUIDs. Valid InputType values:
 *                		 	NoInput, NodeType, SystemImageGuid, NodeGuid,
 *                		 	PortGuid, Lid, NodeDesc
 * @param num_records	 Output: The number of GUIDs returned in query
 * @param guids			 Output: Pointer to array of GUIDs
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */

OMGT_STATUS_T
omgt_sa_get_sysimageguid_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	uint64_t **guids
	);

/**
 * @brief Query SA for Node GUIDs
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select GUIDs. Valid InputType values:
 *                		 	NoInput, NodeType, SystemImageGuid, NodeGuid,
 *                		 	PortGuid, Lid, NodeDesc
 * @param num_records	 Output: The number of GUIDs returned in query
 * @param guids			 Output: Pointer to array of GUIDs
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */

OMGT_STATUS_T
omgt_sa_get_nodeguid_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	uint64_t **guids
	);

/**
 * @brief Query SA for Port GUIDs
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select GUIDs. Valid InputType values:
 *                		 	NoInput, NodeType, SystemImageGuid, NodeGuid,
 *                		 	PortGuid, Lid, NodeDesc
 * @param num_records	 Output: The number of GUIDs returned in query
 * @param guids			 Output: Pointer to array of GUIDs
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */

OMGT_STATUS_T
omgt_sa_get_portguid_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	uint64_t **guids
	);

/**
 * @brief Query SA for ClassPortInfo Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput
 * @param num_records	 Output: The number of records returned in query
 *                               Will either be 0 or 1
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_classportinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_CLASS_PORT_INFO **records
	);


/**
 * @brief Query SA for Lid Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, NodeType, SystemImageGuid, NodeGuid,
 *                		 	PortGuid, Lid, NodeDesc
 * @param num_records	 Output: The number of records returned in query
 * @param lids			 Output: Pointer to lids.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_lid_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	uint32 **lids
	);


/**
 * @brief Query SA for Node Description Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, NodeType, SystemImageGuid, NodeGuid,
 *                		 	PortGuid, Lid, NodeDesc
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_nodedesc_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_NODE_DESCRIPTION **records
	);



/**
 * @brief Query SA for IB Path Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, PortGuid, PortGid, PortGuidPair, GidPair,
 *                		 	PathRecord, Lid, PKey, SL, ServiceId
 * @note
 * A sourcegid is always required with this query
 *
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_ib_path_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	IB_PATH_RECORD **records
	);


/**
* @brief Query SA for SM Info Records
*
* @param port			port opened by omgt_open_port_*
* @param selector		Criteria to select records.
*                		Valid Input Types:
*                			NoInput
* @param num_records	Output: The number of records returned in query
* @param records		Output: Pointer to records.
*								Must be freed by calling omgt_sa_free_records
*
*@return		  OMGT_STATUS_SUCCESS if success, else error code
*/
OMGT_STATUS_T
omgt_sa_get_sminfo_records(
   struct omgt_port *port,
   omgt_sa_selector_t *selector,
   int32_t *num_records,
   STL_SMINFO_RECORD **records
   );


/**
 * @brief Query SA for Link Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_link_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_LINK_RECORD **records
	);


/**
 * @brief Query SA for Service Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, PortGid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_service_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	IB_SERVICE_RECORD **records
	);


/**
 * @brief Query SA for Multicast Member Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, PortGid, McGid, Lid, PKey
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_ib_mcmember_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	IB_MCMEMBER_RECORD **records
	);


/**
 * @brief Query SA for Inform Info Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, PortGid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_informinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_INFORM_INFO_RECORD **records
	);

/**
 * @brief Query SA for Trace Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	PathRecord, PortGuid, GidPair, PortGid
 * @note
 * A sourcegid is always required with this query
 *
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_trace_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_TRACE_RECORD **records
	);

/**
 * @brief Query SA for SCSC Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_scsc_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SC_MAPPING_TABLE_RECORD **records
	);


/**
 * @brief Query SA for SLSC Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_slsc_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SL2SC_MAPPING_TABLE_RECORD **records
	);


/**
 * @brief Query SA for SCSL Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_scsl_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SC2SL_MAPPING_TABLE_RECORD **records
	);


/**
 * @brief Query SA for SCVLt Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_scvlt_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SC2PVL_T_MAPPING_TABLE_RECORD **records
	);


/**
 * @brief Query SA for SCVLnt Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_scvlnt_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SC2PVL_NT_MAPPING_TABLE_RECORD **records
	);


/**
 * @brief Query SA for SwitchInfo Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_switchinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SWITCHINFO_RECORD **records
	);


/**
 * @brief Query SA for Linear Forwarding Database Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_lfdb_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_LINEAR_FORWARDING_TABLE_RECORD **records
	);


/**
 * @brief Query SA for Multicast Forwarding Database Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_mcfdb_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_MULTICAST_FORWARDING_TABLE_RECORD **records
	);


/**
 * @brief Query SA for VL Arbitration Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_vlarb_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_VLARBTABLE_RECORD **records
	);


/**
 * @brief Query SA for PKEY Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid Input Types:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_pkey_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_P_KEY_TABLE_RECORD **records
	);


/**
 * @brief Query SA for VFInfo Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, PKey, SL, ServiceId, McGid, Index, NodeDesc
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_vfinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_VFINFO_RECORD **records
	);


/**
 * @brief Query SA for FabricInfo Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_fabric_info_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_FABRICINFO_RECORD **records
	);


/**
 * @brief Query SA for Quarantine Node Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_quarantinenode_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_QUARANTINED_NODE_RECORD **records
	);


/**
 * @brief Query SA for Congestion Info Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_conginfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_CONGESTION_INFO_RECORD **records
	);


/**
 * @brief Query SA for Switch Congestion Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_swcong_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SWITCH_CONGESTION_SETTING_RECORD **records
	);



/**
 * @brief Query SA for Switch Port Congestion Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_swportcong_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SWITCH_PORT_CONGESTION_SETTING_RECORD **records
	);



/**
 * @brief Query SA for HFI Congestion Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_hficong_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_HFI_CONGESTION_SETTING_RECORD **records
	);



/**
 * @brief Query SA for HFI Congestion Control Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_hficongctrl_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_HFI_CONGESTION_CONTROL_TABLE_RECORD **records
	);



/**
 * @brief Query SA for Buffer Control Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_buffctrl_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_BUFFER_CONTROL_TABLE_RECORD **records
	);



/**
 * @brief Query SA for Cable Info Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_cableinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_CABLE_INFO_RECORD **records
	);



/**
 * @brief Query SA for PortGroup Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_portgroup_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_PORT_GROUP_TABLE_RECORD **records
	);



/**
 * @brief Query SA for PortGroup forwarding table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                		 Valid InputType values:
 *                		 	NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_sa_free_records
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_portgroupfwd_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_PORT_GROUP_FORWARDING_TABLE_RECORD **records
	);



/**
 * @brief Query SA for SwitchCost Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                       Valid InputType values:
 *                         NoInput, Lid
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *                               Must be freed by calling omgt_sa_free_records
 *
 * @return		OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_switchcost_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SWITCH_COST_RECORD **records
	);


#ifdef __cplusplus
}
#endif

#endif /* __OPAMGT_SA_H__ */
