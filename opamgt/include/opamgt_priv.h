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

#ifndef __OPAMGT_PRIV_H__
#define __OPAMGT_PRIV_H__

#include <syslog.h>
/* Needed for getpid() */
#include <sys/types.h>
#include <unistd.h>
#include "opamgt.h"

#include "iba/ib_types.h"
#include "iba/ib_generalServices.h"

#ifndef OMGT_OUTPUT_ERROR
#define OMGT_OUTPUT_ERROR(port, format, args...) \
	do { \
		FILE *current_log_file = port ? port->error_file : stderr; \
		if (port && current_log_file) { \
			if (current_log_file == OMGT_DBG_FILE_SYSLOG) { \
				syslog(LOG_ERR, "opamgt ERROR: [%d] %s: " format, \
					(int)getpid(), __func__, ##args); \
			} else { \
				fprintf(current_log_file, "opamgt ERROR: [%d] %s: " format, \
					(int)getpid(), __func__, ##args); \
			} \
		} \
	} while(0)
#endif

/* NOTE we keep this at LOG_INFO and reserve LOG_DEBUG for packet dump */
#ifndef OMGT_DBGPRINT
#define OMGT_DBGPRINT(port, format, args...) \
	do { \
		FILE *current_log_file = port ? port->dbg_file : NULL; \
		if (port && current_log_file) { \
			if (current_log_file == OMGT_DBG_FILE_SYSLOG) { \
				syslog(LOG_INFO, "opamgt: [%d] %s: " format, \
				    (int)getpid(), __func__, ##args); \
			} else { \
				fflush(current_log_file); fprintf(current_log_file, "opamgt: [%d] %s: " format, \
				    (int)getpid(), __func__, ##args); \
			} \
		} \
	} while(0)
#endif

#ifndef OMGT_OUTPUT_INFO
#define OMGT_OUTPUT_INFO(port, format, args...) \
	do { \
		FILE *current_log_file = port ? port->dbg_file : NULL; \
		if (port && current_log_file) { \
			if (current_log_file == OMGT_DBG_FILE_SYSLOG) { \
				syslog(LOG_INFO, "opamgt: [%d] %s: " format, \
				    (int)getpid(), __func__, ##args); \
			} else { \
				fprintf(current_log_file, "opamgt: [%d] %s: " format, \
				    (int)getpid(), __func__, ##args); \
			} \
		} \
	} while(0)
#endif


#ifndef DBG_ENTER_FUNC
#define DBG_ENTER_FUNC(port)  OMGT_DBGPRINT(port, "Entering %s\n",__func__)
#endif

#ifndef DBG_EXIT_FUNC
#define DBG_EXIT_FUNC(port)  OMGT_DBGPRINT(port, "Exiting %s\n",__func__)
#endif


/**
 * OMGT Port Accessor functions
 * Data is valid once port is opened and until port object is closed.
 */

/**
 * @brief Gets Port's SM LID for the given port.
 * Retrieves port's SM lid for an open in-band port.
 * @param port           previously initialized port object for an in-band
 *  					 connection
 * @param sm_lid         SM lid to be retruned
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_port_get_port_sm_lid(struct omgt_port *port, uint32_t *sm_lid);
/**
 * @brief Gets Port's SM SL for the given port.
 * Retrieves port's SM sl for an open in-band port.
 * @param port           previously initialized port object for an in-band
 *  					 connection
 * @param sm_sl          SM sl to be retruned
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_port_get_port_sm_sl(struct omgt_port *port, uint8_t *sm_sl);

/**
 * @brief Gets Port's SM SL for the given port.
 * Retrieves port's SM sl for an open in-band port.
 * @param port           previously initialized port object for an in-band
 *  					 connection
 * @param node_type      Node Type of the omgt_port
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_port_get_node_type(struct omgt_port *port, uint8_t *node_type);

/**
 * Gets the ISSM device corresponding to the specified port.
 *
 * @param port      Pointer to the opened local port object.
 * @param path      Buffer in which to place the device path
 * @param path_max  Maximum length of the output buffer
 *
 * @return FSUCCESS on success, or UMad error code on failure
 */
FSTATUS omgt_get_issm_device(struct omgt_port *port, char *path, int path_max);

/**
 * Function to refresh the pkey table for the MAD interface
 * for a given hfi name and port.
 * To use with omgt_xxx_mad, and the umad OFED library. 
 * 
 * @param *port     pointer to port object 
 *  
 * @return 0    success 
 *         -1   Failure 
 */ 
extern int omgt_mad_refresh_port_pkey(struct omgt_port *port);

/**
 * Given a port and a pkey return if this pkey is in the ports local
 * pkey_table
 *
 * @param *port     pointer to port object
 * @param  pkey     pkey to search for
 *
 * @return pkey_idx   success
 *         -1         Failure
 */
extern int omgt_find_pkey(struct omgt_port *port, uint16_t pkey);


/**
 * Given a port, destination lid, hopcount, return the most 
 * appropriate management pkey to use. 
 *
 * @param *port     pointer to port object
 * @param  dlid     destination lid 
 * @param  hopCnt   hop count 
 *
 * @return pkey     success 
 *         0        Failure  (Not mgmt allowed for request)
 */
extern uint16_t omgt_get_mgmt_pkey(struct omgt_port *port, uint16_t dlid, uint8_t hopCnt);


/** =========================================================================
 * Send Recv interface
 */

/**
 * bind a list of managment classes/methods with the omgt_port object
 *
 * "bind" must be done if a client wants to respond to unsolicited MAD's
 * 
 * User may call this fn multiple times to register for different class/method
 * combinations.
 *
 * However, registering the same class/method combination twice will result in
 * an error.
 *
 * 3 special flags are used to ease registration for a simple responding
 * client, trap processing client or report processing client.  If these bits
 * are set, methods are set automatically as documented for the client.
 *
 * @param   port          port opened by omgt_open_port_*
 * @param  *mgmt_classes  pointer to a list of managment classes to be registered.
 *						  The list is terminated by a record that has 0 for the 
 *						  base_version.
 * 
 * Returns 0 if successful, +errno status
 */ 
#define OMGT_CLASS_ARG_MAX_METHODS 64
struct omgt_class_args {
	uint8_t  base_version;                       // IB_BASE_VERSION or STL_BASE_VERSION
	uint8_t  mgmt_class;                         // MCLASS_SUBN_ADM, STL_SA_CLASS, etc...
	uint8_t  class_version;                      // IB_BM_CLASS_VERSION, STL_SA_CLASS_VERSION, etc...
	int      is_responding_client;               // client responds to get's and sets
                                                 //     GET && SET
                                                 //        if (SA class)
												 //            GET && SET && GET_TABLE && DELETE && GET_TRACE_TABLE
	int      is_trap_client;                     // client processes traps
                                                 //     TRAP && TRAP_REPRESS
	int      is_report_client;                   // client processes report
                                                 //     REPORT
	int      kernel_rmpp;                         // nonzero if kernel rmpp coalescing support is desired.
	uint8_t *oui;                                // vendor-specific OUI for vendor classes.
	int      use_methods;                        // use the method list instead of default
	uint8_t  methods[OMGT_CLASS_ARG_MAX_METHODS]; // list of methods.
	uint8_t  res[64];
};
int omgt_bind_classes(struct omgt_port *port, struct omgt_class_args *mgmt_classes);


/**
 * Address vector for sending MAD's
 * Also used to return address information on received MAD's
 */
struct omgt_mad_addr {
	uint32_t lid;
	uint32_t qpn;
	uint32_t qkey;
	uint16_t pkey;
	uint8_t  sl;
	uint8_t  age;
	uint32_t flags;
	uint16_t entropy;
	uint8_t  res[18];
};
enum omgt_mad_addr_flags {
	OMGT_MAD_ADDR_SM = (1 << 0), /* use SM addr values defined in PortInfo (SMLID/SMSL)*/
};
#define OMGT_DEFAULT_PKEY        0xffff
/**
 * Send and wait for response MAD
 *    (response is allocated and returned to user)
 * 
 * @param   port        port opened by omgt_open_port_*
 * @param  *send_mad    pointer to outbound MAD
 * @param   send_size   size of send_mad buffer
 * @param  *addr        destination address information
 * @param **recv_mad    pointer to inbound MAD (Allocated based on inbound size)
 * @param  *recv_size   size of recv_mad buffer returned
 * @param   timeout_ms  OFED send timeout in ms (if timeout_ms < 0 wait forever)
 * @param   retries     number of retries to attempt for MAD
 *
 * NOTE: function will wait up to timeout_ms * retries for a response
 *
 * @return 0 (SUCCESS) or error code 
 *           [Error codes are as specified in omgt_recv_mad_alloc]
 */
FSTATUS omgt_send_recv_mad_alloc(struct omgt_port *port,
			uint8_t *send_mad, size_t send_size,
			struct omgt_mad_addr *addr,
			uint8_t **recv_mad, size_t *recv_size,
			int timeout_ms, int retries);

/**
 * Send and wait for response MAD
 *     (response is allocated by user)
 * 
 * @param   port        port opened by omgt_open_port_*
 * @param  *send_mad    pointer to outbound mad
 * @param   send_size   size of send_mad buffer
 * @param  *addr        destination address information
 * @param  *recv_mad    pointer to buffer to hold response MAD
 * @param  *recv_size   IN size of recv_mad buffer
 *                      OUT sizeof actual data received.
 * @param   timeout_ms  OFED send timeout in ms (if timeout_ms < 0 wait forever)
 * @param   retries     number of retries to attempt for MAD
 *
 * NOTE: function will wait up to timeout_ms * retries for a response
 *
 * @return 0 (SUCCESS) or error code 
 *           [Error codes are as specified in omgt_recv_mad_no_alloc]
 */
FSTATUS omgt_send_recv_mad_no_alloc(struct omgt_port *port,
			uint8_t *send_mad, size_t send_size,
			struct omgt_mad_addr *addr,
			uint8_t *recv_mad, size_t *recv_size,
			int timeout_ms, int retries);

/**
 * Send a MAD
 * 
 * @param  port        port opened by omgt_open_port_*
 * @param *send_mad    pointer to mad to send                                        
 * @param  send_size   Length of buffer to send 
 * @param  addr        destination address information
 * @param  timeout_ms  OFED send timeout in ms (if timeout_ms < 0 wait forever)
 * @param  retries     number of retries to attempt for MAD
 * 
 * @return FSTATUS (0 if successful, else error code)
 */
FSTATUS omgt_send_mad2(struct omgt_port *port, uint8_t *send_mad, size_t send_size,
			struct omgt_mad_addr *addr, int timeout_ms, int retries);

/**
 * Receive a packet from the specified port
 *     (response is allocated by user)
 * 
 * Function will allocate an appropriately sized MAD to hold the status.
 * The entire MAD is returned, including all headers.
 * recv_size is initialized with the size of the returned buffer.
 *
 * @param    port        port opened by omgt_open_port_*
 * @param  **recv_mad    pointer to inbound MAD (Allocated based on inbound size)
 * @param   *recv_size   size of recv_mad buffer returned
 * @param    timeout_ms  OFED send timeout in ms (if timeout_ms < 0 wait forever)
 * @param    addr        if supplied recv'd MAD address information is filled in
 *                       here.
 * 
 * @return   0 if success, else error code
 *
 * FINVALID_PARAMETER - incorrect or missing arguments to function
 * FERROR - other atypical errors
 * FNOT_DONE - no packet available within timeout
 * FOVERRUN - error receiving a large rmpp MAD. *recv_size indicates length
 * 			reported by the failed umad_recv.
 * FINSUFFICIENT_MEMORY - unable to allocate memory (*recv_size is length of
 * 					MAD attempting to be allocated), but no MAD is returned
 *
 * For the FTIMEOUT, FREJECT and FSUCCESS return cases, *recv_mad will be
 * updated with pointer to the MAD (coallesced RMPP if applicable).  Caller
 * must free() this buffer.  *recv_size will indicate the length of the MAD.
 *
 * FTIMEOUT - a previous send's response exceeded timeout/retry, the sent MAD
 *  				is returned
 * FREJECT - unexpected error processing a previous send or its response, the
 * 					sent MAD is returned
 * FSUCCESS - mad successfully received
 */
FSTATUS omgt_recv_mad_alloc(struct omgt_port *port, uint8_t **recv_mad, size_t *recv_size,
			int timeout_ms, struct omgt_mad_addr *addr);
/**
 * Free a mad received from omgt_*
 *
 * @param    port        port opened by omgt_open_port_*
 * @param  **recv_mad    Pointer to rcv buffer
 */
void omgt_free_recv_mad(struct omgt_port *port, uint8_t *recv_mad);

/**
 * Receive a MAD into an allocated buffer
 *     (response is allocated by user)
 * 
 * @param    port        port opened by omgt_open_port_*
 * @param   *recv_mad    Allocated buffer to place MAD in
 * @param   *recv_size   IN size of recv_mad buffer
 *                       OUT sizeof actual data received.
 * @param    timeout_ms  OFED send timeout in ms (if timeout_ms < 0 wait forever)
 * @param    addr        if supplied recv'd MAD address information is filled in
 *                       here.
 * 
 * @return   0 if success, else error code
 *
 * FINVALID_PARAMETER - incorrect or missing arguments to function
 * FERROR - other atypical errors
 * FNOT_DONE - no packet available within timeout
 * FOVERRUN - large MAD (rmpp) found and discarded, 1st MAD in sequence
 * 				returned
 * FINSUFFICIENT_MEMORY - unable to allocate memory, no MAD returned
 *
 * For the FTIMEOUT, FREJECT and FSUCCESS return cases, recv_mad will be
 * filled in with MAD data.
 *
 * FTIMEOUT - a previous send's response exceeded timeout/retry, the sent MAD
 *  				is returned
 * FREJECT - unexpected error processing a previous send or its response, the
 * 					sent MAD is returned
 * FSUCCESS - MAD successfully received
 */
FSTATUS omgt_recv_mad_no_alloc(struct omgt_port *port, uint8_t *recv_mad, size_t *recv_size,
			int timeout_ms, struct omgt_mad_addr *addr);

/** =========================================================================
 * Need TBD...  Right now the global port has cached data for things like
 * pkey, smlid, smsl, portstate...
 * It is up to the user of the lib to "refresh" this data.
 * It would be nice if the lib could do that on it's own.
 * Regardless we could have a call to update this information if the user
 * feels the need.
 *
FSTATUS omgt_mad_refresh_port_details(struct omgt_port *port);
 */


/** =========================================================================
 * Generic HELPER FUNCTIONs
 */


/**
 * @brief Get the HFI number
 *
 * @param hfi_name       Name of the HFI device
 *
 * @return
 *  On Success  the HFI Number
 *  On Error    -1
 */
int omgt_get_hfi_num(char *hfi_name);

/**
 * MACRO used to get the specific record in a multi-record response
 * @param p      pointer to SA_MAD struct that contains the response
 * @param i      index of record
 */
#define GET_RESULT_OFFSET(p,i) (p->Data+(p->SaHdr.AttributeOffset*sizeof (uint64_t)*i))

/* translate ca/port number into a port Guid. Also return other useful structs
 * if non-null pointer passed-in.
 * Warning: Endian conversion not done
 *
 * INPUTS:
 * 	ca - system wide CA number 1-n, if 0 port is a system wide port #
 * 	port - 1-n, if ca is 0, system wide port number, otherwise port within CA
 * 			if 0, 1st active port
 * 	*pCaName - ca name for specified port
 * 	*omgtport - optional omgt_port, will use log targets set up for this port
 *
 * OUTPUTS:
 * 	*pCaGuid - ca guid for specified port
 * 	*pPortGuid - port guid for specified port (Warning: Endian conversion not done)
 * 	*pCaAttributes - attributes for CA,
 * 					If PortAttributesList not null, caller must MemoryDeallocate pCaAttributes->PortAttributesList
 * 	*ppPortAtributes - attributes for port, if ppPortAtributes not null, caller must MemoryDeallocate ppPortAtributes
 * 	*pCaCount - number of CA in system
 * 	*pPortCount - number of ports in system or CA (depends on ca input)
 *
 * RETURNS:
 * 	FNOT_FOUND - *pCaCount and *pPortCount still output
 * 				if ca == 0, *pPortCount = number of ports in system
 * 				if ca < *pCaCount, *pPortCount = number of ports in CA
 * 									otherwise *pPortCount will be 0
 *
 */
FSTATUS omgt_get_portguid(uint32 ca, uint32 port, char *pCaName, struct omgt_port *omgtport,
	EUI64 *pCaGuid, EUI64 *pPortGuid, IB_CA_ATTRIBUTES *pCaAttributes,
	IB_PORT_ATTRIBUTES **ppPortAttributes, uint32 *pCaCount, uint32 *pPortCount,
	char *pRetCaName, int *pRetPortNum, uint64 *pRetGIDPrefix);



#endif /* __OPAMGT_PRIV_H__ */
