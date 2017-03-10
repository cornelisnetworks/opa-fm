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

#ifndef __OIB_UTILS_SA_H__
#define __OIB_UTILS_SA_H__

#include <oib_utils.h>


#define OIB_UTILS_DEFAULT_PORT_NUM 1

struct ibv_sa_event {
	void *context;
	int status;
	int attr_count;
	int attr_size;
	int attr_offset;
	uint16_t attr_id;
	void *attr;
};

struct _QUERY;
struct _QUERY_RESULT_VALUES;

/** =========================================================================
 * OIB utils SA interface
 */

/**
 * Initiates a registration for the specified trap.
 *
 * @param port      port opened by oib_open_port_*
 * @param trap_num  The trap number to register for 
 * @param context   optional opaque info 
 *  
 * @return   0 if success, else error code
 */
FSTATUS oib_sa_register_trap(struct oib_port *port, 
							 uint16_t trap_num,
							 void *context);

/**
 * Unregisters for the specified trap.
 *
 * @param port      port opened by oib_open_port_*
 * @param trap_num  The trap number to unregister 
 *  
 * @return   0 if success, else error code 
 */
FSTATUS oib_sa_unregister_trap(struct oib_port *port, uint16_t trap_num);

/**
 * Fetches the next available notice.  Blocks in kernel (interrupted on
 * process signal).
 *
 * @param port           port opened by oib_open_port_*
 * @param target_buf     Pointer to buffer to place notice into
 * @param buf_size       Size of the target buffer
 * @param bytes_written  OUTPUT: Set to the number of bytes written
 *                       written into the buffer
 *  
 * @return   0 if success, else error code 
 */ 
FSTATUS oib_sa_get_event(struct oib_port *port, void *target_buf,
						 size_t buf_size, int *bytes_written);

/**
 * Set the buffer counts for QP creation.
 *
 * This function must be called after oib_open_port but
 * before oib_sa_register_trap.
 *
 * @param port       port opened by oib_open_port_*
 * @param send_count number of send buffers to use on port (0 for no change) 
 * @param recv_count number of receive buffers to use on port (0 for no change)
 *  
 * @return   0 if success, else error code
 */
FSTATUS oib_set_sa_buf_cnt(struct oib_port *port, 
						   int send_count,
						   int recv_count);

/**
 * Get the buffer counts for QP creation.
 *
 * @param port       port opened by oib_open_port_*
 * @param send_count number of send buffers used on port 
 * @param recv_count number of receive buffers used on port
 *  
 * @return   0 if success, else error code
 */
FSTATUS oib_get_sa_buf_cnt(struct oib_port *port, 
						   int *send_count,
						   int *recv_count);

/**
 * Retrieves the next pending event, if no event is pending
 * blocks waiting for an event.
 *  
 * @param port      port opened by oib_open_port_* 
 * @param event     Allocated information about the next event.
 *  
 * @return          0 if success, else error code
 */
FSTATUS oib_get_sa_event(struct oib_port *port, struct ibv_sa_event **event);

/**
 * Cleans up allocated ibv_sa_event structure.
 *  
 * @param event     Allocated information about the next event.
 *  
 * @return          0 if success, else error code
 */
FSTATUS oib_ack_sa_event(struct ibv_sa_event *event);

/**
 * Retrieves the next pending event, if no event is pending
 * waits for an event.
 *  
 * @param port      port opened by oib_open_port_* 
 * @param event     Allocated information about the next event.
 *  
 * @return          0 if success, else error code
 */
int ibv_sa_get_event(struct oib_port *port, struct ibv_sa_event **event);

/**
 * Pose a query to the fabric, expect a response.
 *  
 * @param port           port opened by oib_open_port_* 
 * @param pQuery         pointer to the query structure
 * @param ppQueryResult  pointer where the response will go 
 *  
 * @return          0 if success, else error code
 */
FSTATUS oib_query_sa(struct oib_port *port,
					 struct _QUERY *pQuery,
					 struct _QUERY_RESULT_VALUES **ppQueryResult);

/**
 * Free the memory used in the query result
 *  
 * @param pQueryResult    pointer to the SA query result buffer
 *  
 * @return          none
 */
void oib_free_query_result_buffer(IN void * pQueryResult);

#endif /* __OIB_UTILS_SA_H__ */
