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

#ifndef __OPAMGT_PRIV_SA_H__
#define __OPAMGT_PRIV_SA_H__

#include "opamgt_priv.h"
#include <iba/stl_sd.h>

#define OPAMGT_DEFAULT_PORT_NUM 1

struct ibv_sa_event {
	void *context;
	int status;
	int attr_count;
	int attr_size;
	int attr_offset;
	uint16_t attr_id;
	void *attr;
};

/*
 * Convert old QUERY_INPUT_VALUE to new OMGT_QUERY_INPUT_VALUE
 *
 * @param output_query      Query structure to be passed to omgt_query_sa
 * @param old_query         old query InputValue holder
 * @param source_gid        local gid for in-band queries or a source gid for
 *  						out-of-band queries. Queries like path and trace
 *  						record require a SourceGid.
 *
 * @return FSTATUS: FNOT_FOUND (Type Failure), FERROR (source_gid Failure)
 */
FSTATUS omgt_input_value_conversion(OMGT_QUERY *output_query, QUERY_INPUT_VALUE *old_query, IB_GID source_gid);

/**
 * Pose a query to the fabric, expect a response.
 *
 * @param port           port opened by omgt_open_port_*
 * @param pQuery         pointer to the query structure
 * @param ppQueryResult  pointer where the response will go
 *
 * @return          0 if success, else error code
 */
FSTATUS omgt_query_sa(struct omgt_port *port,
					 OMGT_QUERY *pQuery,
					 QUERY_RESULT_VALUES **ppQueryResult);

/**
 * Free the memory used in the query result
 *
 * @param pQueryResult    pointer to the SA query result buffer
 *
 * @return          none
 */
void omgt_free_query_result_buffer(IN void * pQueryResult);

/**
 * Initiates a registration for the specified trap.
 *
 * @param port      port opened by omgt_open_port_*
 * @param trap_num  The trap number to register for
 * @param context   optional opaque info
 *
 * @return   0 if success, else error code
 */
FSTATUS omgt_sa_register_trap(struct omgt_port *port,
							 uint16_t trap_num,
							 void *context);

/**
 * Unregisters for the specified trap.
 *
 * @param port      port opened by omgt_open_port_*
 * @param trap_num  The trap number to unregister 
 *  
 * @return   0 if success, else error code 
 */
FSTATUS omgt_sa_unregister_trap(struct omgt_port *port, uint16_t trap_num);

/**
 * Fetches the next available notice.  Blocks in kernel (interrupted on
 * process signal).
 *
 * @param port           port opened by omgt_open_port_*
 * @param target_buf     Pointer to buffer to place notice into
 * @param buf_size       Size of the target buffer
 * @param bytes_written  OUTPUT: Set to the number of bytes written
 *                       written into the buffer
 *  
 * @return   0 if success, else error code 
 */ 
FSTATUS omgt_sa_get_event(struct omgt_port *port, void *target_buf,
						 size_t buf_size, int *bytes_written);

/**
 * Set the buffer counts for QP creation.
 *
 * This function must be called after omgt_open_port but
 * before omgt_sa_register_trap.
 *
 * @param port       port opened by omgt_open_port_*
 * @param send_count number of send buffers to use on port (0 for no change) 
 * @param recv_count number of receive buffers to use on port (0 for no change)
 *  
 * @return   0 if success, else error code
 */
FSTATUS omgt_set_sa_buf_cnt(struct omgt_port *port, 
						   int send_count,
						   int recv_count);

/**
 * Get the buffer counts for QP creation.
 *
 * @param port       port opened by omgt_open_port_*
 * @param send_count number of send buffers used on port 
 * @param recv_count number of receive buffers used on port
 *  
 * @return   0 if success, else error code
 */
FSTATUS omgt_get_sa_buf_cnt(struct omgt_port *port, 
						   int *send_count,
						   int *recv_count);

/**
 * Retrieves the next pending event, if no event is pending
 * blocks waiting for an event.
 *  
 * @param port      port opened by omgt_open_port_* 
 * @param event     Allocated information about the next event.
 *  
 * @return          0 if success, else error code
 */
FSTATUS omgt_get_sa_event(struct omgt_port *port, struct ibv_sa_event **event);

/**
 * Cleans up allocated ibv_sa_event structure.
 *  
 * @param event     Allocated information about the next event.
 *  
 * @return          0 if success, else error code
 */
FSTATUS omgt_ack_sa_event(struct ibv_sa_event *event);

/**
 * Retrieves the next pending event, if no event is pending
 * waits for an event.
 *  
 * @param port      port opened by omgt_open_port_* 
 * @param event     Allocated information about the next event.
 *  
 * @return          0 if success, else error code
 */
int ibv_sa_get_event(struct omgt_port *port, struct ibv_sa_event **event);


#endif // __OPAMGT_PRIV_SA_H__

