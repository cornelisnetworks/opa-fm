/* BEGIN_ICS_COPYRIGHT2 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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


#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>


#include <iba/ib_mad.h>
#include <stl_sd.h>

#define OPAMGT_PRIVATE 1


#include "ib_utils_openib.h"
#include "opamgt_sa_priv.h"
#include "opamgt_sa.h"

/*
 * @brief Get MAD status code from most recent SA operation
 *
 * @param port		Local port to operate on.
 *
 * @return
 *	 The corresponding status code.
 */
uint16_t
omgt_get_sa_mad_status(struct omgt_port *port)
{
	return port->sa_mad_status;
}



static OMGT_STATUS_T
omgt_sa_query_helper(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	QUERY_RESULT_TYPE output_type,
	QUERY_RESULT_VALUES **query_result)
{
	OMGT_QUERY query;
	OMGT_STATUS_T status;

	query.InputType = selector->InputType;
	query.OutputType = output_type;
	query.InputValue = selector->InputValue;

	status = omgt_query_sa(port, &query, query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	if(!*query_result)	
		return OMGT_STATUS_ERROR;

	if(OMGT_STATUS_SUCCESS != port->sa_mad_status)
		return OMGT_STATUS_ERROR;

	return status;
}

void
omgt_sa_free_records(void *record)
{
	free(record);
	return;
}


/**
 * @brief Query SA for Node Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param ni_records	 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_node_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_NODE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_NODE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlNodeRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_NODE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumNodeRecords;

	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->NodeRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}
/**
 * @brief Query SA for PortInfo Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param ni_records	 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_portinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_PORTINFO_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_PORTINFO_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlPortInfoRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_PORTINFO_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumPortInfoRecords;

	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->PortInfoRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}



/**
 * @brief Query SA for System Image GUIDs
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select GUIDs.
 * @param num_records	 Output: The number of GUIDs returned in query
 * @param guids			 Output: Pointer to array of GUIDs
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_sysimageguid_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	uint64_t **guids
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	GUID_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSystemImageGuid, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (GUID_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumGuids;
	if(*num_records == 0) {
		*guids = NULL;
		goto exit;
	}

	int buf_size = sizeof(**guids) * (*num_records);
	*guids = malloc(buf_size);
	if(*guids == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*guids, record_results->Guids, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for Node GUIDs
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select GUIDs.
 * @param num_records	 Output: The number of GUIDs returned in query
 * @param guids			 Output: Pointer to array of GUIDs
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_nodeguid_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	uint64_t **guids
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	GUID_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlNodeGuid, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (GUID_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumGuids;
	if(*num_records == 0) {
		*guids = NULL;
		goto exit;
	}

	int buf_size = sizeof(**guids) * (*num_records);
	*guids = malloc(buf_size);
	if(*guids == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*guids, record_results->Guids, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for Port GUIDs
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select GUIDs.
 * @param num_records	 Output: The number of GUIDs returned in query
 * @param guids			 Output: Pointer to array of GUIDs
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */OMGT_STATUS_T
omgt_sa_get_portguid_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	uint64_t **guids
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	GUID_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlPortGuid, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (GUID_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumGuids;
	if(*num_records == 0) {
		*guids = NULL;
		goto exit;
	}

	int buf_size = sizeof(**guids) * (*num_records);
	*guids = malloc(buf_size);
	if(*guids == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*guids, record_results->Guids, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for ClassPortInfo Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_classportinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_CLASS_PORT_INFO **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_CLASS_PORT_INFO_RESULT *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlClassPortInfo, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_CLASS_PORT_INFO_RESULT*)query_result->QueryResult;
	*num_records = record_results->NumClassPortInfo;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}


	memcpy(*records, &record_results->ClassPortInfo, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for Lid Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param lids			 Output: Pointer to lids.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_lid_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	uint32 **lids
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_LID_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlLid, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_LID_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumLids;
	if(*num_records == 0) {
		*lids = NULL;
		goto exit;
	}

	int buf_size = sizeof(**lids) * (*num_records);
	*lids = malloc(buf_size);
	if(*lids == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}


	memcpy(*lids, record_results->Lids, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for Node Description Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_nodedesc_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_NODE_DESCRIPTION **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_NODEDESC_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlNodeDesc, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_NODEDESC_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumDescs;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->NodeDescs, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for IB Path Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_ib_path_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	IB_PATH_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	PATH_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypePathRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (PATH_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumPathRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}


	memcpy(*records, record_results->PathRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
* @brief Query SA for SM Info Records
*
* @param port			port opened by omgt_open_port_*
* @param selector		Criteria to select records. NULL == all records
* @param num_records	Output: The number of records returned in query
* @param records		Output: Pointer to records.
*								Must be freed by calling omgt_free_query_result_buffer
*
*@return		  OMGT_STATUS_SUCCESS if success, else error code
*/
OMGT_STATUS_T
omgt_sa_get_sminfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SMINFO_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_SMINFO_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSMInfoRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_SMINFO_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumSMInfoRecords;

	int buf_size = sizeof(**records) * (*num_records);
	if(*num_records == 0) {
		*records = NULL;
		return status;
	}

	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}
		

	memcpy(*records, record_results->SMInfoRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for Link Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_link_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_LINK_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_LINK_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlLinkRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_LINK_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumLinkRecords;

	int buf_size = sizeof(**records) * (*num_records);
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}
		

	memcpy(*records, record_results->LinkRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for Service Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_service_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	IB_SERVICE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	SERVICE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeServiceRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (SERVICE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumServiceRecords;

	int buf_size = sizeof(**records) * (*num_records);
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}


	memcpy(*records, record_results->ServiceRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for Multicast Member Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_ib_mcmember_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	IB_MCMEMBER_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	MCMEMBER_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeMcMemberRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (MCMEMBER_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumMcMemberRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){ 
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->McMemberRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for Inform Info Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_informinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_INFORM_INFO_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_INFORM_INFO_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlInformInfoRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_INFORM_INFO_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumInformInfoRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->InformInfoRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}
/**
 * @brief Query SA for Trace Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_trace_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_TRACE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_TRACE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlTraceRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_TRACE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumTraceRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->TraceRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}
/**
 * @brief Query SA for SCSC Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_scsc_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SC_MAPPING_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_SC_MAPPING_TABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSCSCTableRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_SC_MAPPING_TABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumSCSCTableRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->SCSCRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for SLSC Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_slsc_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SL2SC_MAPPING_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_SL2SC_MAPPING_TABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSLSCTableRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_SL2SC_MAPPING_TABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumSLSCTableRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->SLSCRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for SCSL Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_scsl_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SC2SL_MAPPING_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_SC2SL_MAPPING_TABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSCSLTableRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_SC2SL_MAPPING_TABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumSCSLTableRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->SCSLRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for SCVLt Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_scvlt_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SC2PVL_T_MAPPING_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_SC2PVL_T_MAPPING_TABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSCVLtTableRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_SC2PVL_T_MAPPING_TABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumSCVLtTableRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->SCVLtRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for SCVLr Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_scvlr_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SC2PVL_R_MAPPING_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_SC2PVL_R_MAPPING_TABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSCVLrTableRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_SC2PVL_R_MAPPING_TABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumSCVLrTableRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->SCVLrRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for SCVLnt Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_scvlnt_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SC2PVL_NT_MAPPING_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_SC2PVL_NT_MAPPING_TABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSCVLntTableRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_SC2PVL_NT_MAPPING_TABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumSCVLntTableRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){ 
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->SCVLntRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for SwitchInfo Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_switchinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SWITCHINFO_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_SWITCHINFO_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSwitchInfoRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_SWITCHINFO_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumSwitchInfoRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->SwitchInfoRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for Linear Forwarding Database Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_lfdb_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_LINEAR_FORWARDING_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_LINEAR_FDB_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlLinearFDBRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_LINEAR_FDB_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumLinearFDBRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->LinearFDBRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for Multicast Forwarding Database Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_mcfdb_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_MULTICAST_FORWARDING_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_MCAST_FDB_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlMCastFDBRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_MCAST_FDB_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumMCastFDBRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->MCastFDBRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for VL Arbitration Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_vlarb_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_VLARBTABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_VLARBTABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlVLArbTableRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_VLARBTABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumVLArbTableRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->VLArbTableRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for PKEY Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_pkey_table_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_P_KEY_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_PKEYTABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlPKeyTableRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_PKEYTABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumPKeyTableRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->PKeyTableRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for VFInfo Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_vfinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_VFINFO_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_VFINFO_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlVfInfoRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_VFINFO_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumVfInfoRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->VfInfoRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for FabricInfo Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_fabric_info_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_FABRICINFO_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_FABRICINFO_RECORD_RESULT *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlFabricInfoRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_FABRICINFO_RECORD_RESULT*)query_result->QueryResult;
	*num_records = record_results->NumFabricInfoRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, &record_results->FabricInfoRecord, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for Quarantine Node Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_quarantinenode_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_QUARANTINED_NODE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_QUARANTINED_NODE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlQuarantinedNodeRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_QUARANTINED_NODE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumQuarantinedNodeRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->QuarantinedNodeRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for Congestion Info Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_conginfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_CONGESTION_INFO_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_CONGESTION_INFO_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlCongInfoRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_CONGESTION_INFO_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->Records, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

/**
 * @brief Query SA for Switch Congestion Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_swcong_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SWITCH_CONGESTION_SETTING_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_SWITCH_CONGESTION_SETTING_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSwitchCongRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_SWITCH_CONGESTION_SETTING_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->Records, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for Switch Port Congestion Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_swportcong_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_SWITCH_PORT_CONGESTION_SETTING_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_SWITCH_PORT_CONGESTION_SETTING_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSwitchPortCongRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_SWITCH_PORT_CONGESTION_SETTING_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->Records, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for HFI Congestion Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_hficong_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_HFI_CONGESTION_SETTING_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_HFI_CONGESTION_SETTING_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlHFICongRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_HFI_CONGESTION_SETTING_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->Records, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for HFI Congestion Control Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_hficongctrl_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_HFI_CONGESTION_CONTROL_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_HFI_CONGESTION_CONTROL_TABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlHFICongCtrlRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_HFI_CONGESTION_CONTROL_TABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->Records, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for Buffer Control Table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_buffctrl_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_BUFFER_CONTROL_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_BUFFER_CONTROL_TABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlBufCtrlTabRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_BUFFER_CONTROL_TABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumBufferControlRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->BufferControlRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for Cable Info Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_cableinfo_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_CABLE_INFO_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_CABLE_INFO_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlCableInfoRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_CABLE_INFO_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumCableInfoRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->CableInfoRecords, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for PortGroup Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_portgroup_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_PORT_GROUP_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_PORT_GROUP_TABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlPortGroupRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_PORT_GROUP_TABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->Records, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for PortGroup forwarding table Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records. NULL == all records
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *								 Must be freed by calling omgt_free_query_result_buffer
 *
 *@return		   OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_portgroupfwd_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_PORT_GROUP_FORWARDING_TABLE_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_PORT_GROUP_FORWARDING_TABLE_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlPortGroupFwdRecord, &query_result);

	if(OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_PORT_GROUP_FORWARDING_TABLE_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->Records, buf_size);
exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for DeviceGroupName Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                       Valid InputType values:
 *                         NoInput
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *                               Must be freed by calling omgt_sa_free_records
 *
 * @return		OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_devicegroupname_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_DEVICE_GROUP_NAME_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_DEVICE_GROUP_NAME_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlDeviceGroupNameRecord, &query_result);

	if (OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_DEVICE_GROUP_NAME_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->Records, buf_size);

exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for DeviceGroupMember Records
 *
 * @param port			 port opened by omgt_open_port_*
 * @param selector		 Criteria to select records.
 *                       Valid InputType values:
 *                         NoInput, Lid, PortGuid, NodeDesc, DeviceGroup
 * @param num_records	 Output: The number of records returned in query
 * @param records		 Output: Pointer to records.
 *                               Must be freed by calling omgt_sa_free_records
 *
 * @return		OMGT_STATUS_SUCCESS if success, else error code
 */
OMGT_STATUS_T
omgt_sa_get_devicegroupmember_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_DEVICE_GROUP_MEMBER_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_DEVICE_GROUP_MEMBER_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlDeviceGroupMemberRecord, &query_result);

	if (OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_DEVICE_GROUP_MEMBER_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->Records, buf_size);

exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


/**
 * @brief Query SA for DeviceTreeMember Records
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
omgt_sa_get_devicetreemember_records(
	struct omgt_port *port,
	omgt_sa_selector_t *selector,
	int32_t *num_records,
	STL_DEVICE_TREE_MEMBER_RECORD **records
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_DEVICE_TREE_MEMBER_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlDeviceTreeMemberRecord, &query_result);

	if (OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_DEVICE_TREE_MEMBER_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->Records, buf_size);

exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}


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
	)
{

	OMGT_STATUS_T status;
	QUERY_RESULT_VALUES *query_result;
	STL_SWITCH_COST_RECORD_RESULTS *record_results = NULL;

	status = omgt_sa_query_helper(port, selector, OutputTypeStlSwitchCostRecord, &query_result);

	if (OMGT_STATUS_SUCCESS != status)
		return status;

	record_results = (STL_SWITCH_COST_RECORD_RESULTS*)query_result->QueryResult;
	*num_records = record_results->NumRecords;
	if(*num_records == 0) {
		*records = NULL;
		goto exit;
	}

	int buf_size = sizeof(**records) * (*num_records);
	*records = malloc(buf_size);
	if(*records == NULL){
		status = OMGT_STATUS_INSUFFICIENT_MEMORY;
		goto exit;
	}

	memcpy(*records, record_results->Records, buf_size);

exit:
	omgt_free_query_result_buffer(query_result);
	return status;
}

