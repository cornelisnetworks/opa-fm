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

#ifndef __OIB_UTILS_H__
#define __OIB_UTILS_H__

#include <stdint.h>

#include <infiniband/umad.h>
#include "iba/public/datatypes.h"
#include "iba/ib_types.h"

/**
 * Set output for oib_utils
 * @param file   file pointer to send output to
 *               if == NULL => turn off output
 *               if == OIB_DBG_FILE_SYSLOG => send output to syslog
 *               NOTE on using syslog:
 *                  LOG_DEBUG can generate a lot of data as this will dump
 *                  packets
 *                  OIB uses only LOG_ERR, LOG_INFO, and LOG_DEBUG
 *                  It is advisable, but not required, to call "openlog" prior to
 *                  setting this option
 *                  You can have Errors go to syslog and debug go to a file
 *                  or vice versa
 */
#define OIB_DBG_FILE_SYSLOG ((FILE *)-1)
void oib_set_dbg(FILE *file); /* syslog(LOG_INFO|LOG_DEBUG, ... */
void oib_set_err(FILE *file); /* syslog(LOG_ERR, ... */

/** =========================================================================
 * Open/Close an oib_port
 */

/* opaque data defined internally */
struct oib_port;

/**
 * oib_open_port*
 *
 * Open an oib port
 *
 * @param port       port object is allocated and returned in 'port'
 * @param hfiName    hfi name (eg "qib0", "wfr0")
 * @param hfiNum     hfi number based on the order of the cards.
 * @param port       port number of the hfi
 * @param port_guid  port GUID to open
 *
 * @return +errno status
 *
 * NOTE: user must call oib_close_port to release port resources
 *       use of port after oib_close_port will result in undefined behavior.
 */
int oib_open_port        (struct oib_port **port, char *hfi_name, uint8_t port_num);
int oib_open_port_by_num (struct oib_port **port, int hfi_num, uint8_t port_num);
int oib_open_port_by_guid(struct oib_port **port, uint64_t port_guid);

/**
 * oib_close_port
 *
 * @param port       port object to free.
 */
int oib_close_port       (struct oib_port *port);

/**
 * Port Accessor functions
 * Data is valid until port object is closed.
 */
uint32_t  oib_get_port_lid(struct oib_port *port);
uint32_t  oib_get_port_sm_lid(struct oib_port *port);
uint8_t   oib_get_port_sm_sl(struct oib_port *port);
uint64_t  oib_get_port_guid(struct oib_port *port);
uint8_t   oib_get_hfi_port_num(struct oib_port *port);
int       oib_get_hfi_num(struct oib_port *port);
uint64_t  oib_get_port_prefix(struct oib_port *port);
uint8_t   oib_get_port_state(struct oib_port *port);
FSTATUS   oib_get_hfi_node_type(struct oib_port *port, OUT int *nodeType);
char     *oib_get_hfi_name(struct oib_port *port);

/**
 * Gets the ISSM device corresponding to the specified port.
 *
 * @param port      Pointer to the opened local port object.
 * @param path      Buffer in which to place the device path
 * @param path_max  Maximum length of the output buffer
 *
 * @return FSUCCESS on success, or UMad error code on failure
 */
FSTATUS oib_get_issm_device(struct oib_port *port, char *path, int path_max);

/**
 * Function to refresh the pkey table for the MAD interface
 * for a given hfi name and port.
 * To use with oib_xxx_mad, and the umad OFED library. 
 * 
 * @param *port     pointer to port object 
 *  
 * @return 0    success 
 *         -1   Failure 
 */ 
extern int oib_mad_refresh_port_pkey(struct oib_port *port);

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
extern int oib_find_pkey(struct oib_port *port, uint16_t pkey);

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
extern uint16_t oib_get_mgmt_pkey(struct oib_port *port, uint16_t dlid, uint8_t hopCnt);


/** =========================================================================
 * Send Recv interface
 */

/**
 * bind a list of managment classes/methods with the oib_port object
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
 * @param   port          port opened by oib_open_port_*
 * @param  *mgmt_classes  pointer to a list of managment classes to be registered.
 *						  The list is terminated by a record that has 0 for the 
 *						  base_version.
 * 
 * Returns 0 if successful, +errno status
 */ 
#define OIB_CLASS_ARG_MAX_METHODS 64
struct oib_class_args {
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
	uint8_t  methods[OIB_CLASS_ARG_MAX_METHODS]; // list of methods.
	uint8_t  res[64];
};
int oib_bind_classes(struct oib_port *port, struct oib_class_args *mgmt_classes);


/**
 * Address vector for sending MAD's
 * Also used to return address information on received MAD's
 */
struct oib_mad_addr {
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
enum oib_mad_addr_flags {
	OIB_MAD_ADDR_SM = (1 << 0), /* use SM addr values defined in PortInfo (SMLID/SMSL)*/
};
#define OIB_DEFAULT_PKEY        0xffff
/**
 * Send and wait for response MAD
 *    (response is allocated and returned to user)
 * 
 * @param   port        port opened by oib_open_port_*
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
 *           [Error codes are as specified in oib_recv_mad_alloc]
 */
FSTATUS oib_send_recv_mad_alloc(struct oib_port *port,
			uint8_t *send_mad, size_t send_size,
			struct oib_mad_addr *addr,
			uint8_t **recv_mad, size_t *recv_size,
			int timeout_ms, int retries);

/**
 * Send and wait for response MAD
 *     (response is allocated by user)
 * 
 * @param   port        port opened by oib_open_port_*
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
 *           [Error codes are as specified in oib_recv_mad_no_alloc]
 */
FSTATUS oib_send_recv_mad_no_alloc(struct oib_port *port,
			uint8_t *send_mad, size_t send_size,
			struct oib_mad_addr *addr,
			uint8_t *recv_mad, size_t *recv_size,
			int timeout_ms, int retries);

/**
 * Send a MAD
 * 
 * @param  port        port opened by oib_open_port_*
 * @param *send_mad    pointer to mad to send                                        
 * @param  send_size   Length of buffer to send 
 * @param  addr        destination address information
 * @param  timeout_ms  OFED send timeout in ms (if timeout_ms < 0 wait forever)
 * @param  retries     number of retries to attempt for MAD
 * 
 * @return FSTATUS (0 if successful, else error code)
 */
FSTATUS oib_send_mad2(struct oib_port *port, uint8_t *send_mad, size_t send_size,
			struct oib_mad_addr *addr, int timeout_ms, int retries);

/**
 * Receive a packet from the specified port
 *     (response is allocated by user)
 * 
 * Function will allocate an appropriately sized MAD to hold the status.
 * The entire MAD is returned, including all headers.
 * recv_size is initialized with the size of the returned buffer.
 *
 * @param    port        port opened by oib_open_port_*
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
FSTATUS oib_recv_mad_alloc(struct oib_port *port, uint8_t **recv_mad, size_t *recv_size,
			int timeout_ms, struct oib_mad_addr *addr);
/**
 * Free a mad received from oib_*
 *
 * @param    port        port opened by oib_open_port_*
 * @param  **recv_mad    Pointer to rcv buffer
 */
void oib_free_recv_mad(struct oib_port *port, uint8_t *recv_mad);

/**
 * Receive a MAD into an allocated buffer
 *     (response is allocated by user)
 * 
 * @param    port        port opened by oib_open_port_*
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
FSTATUS oib_recv_mad_no_alloc(struct oib_port *port, uint8_t *recv_mad, size_t *recv_size,
			int timeout_ms, struct oib_mad_addr *addr);

/** =========================================================================
 * Need TBD...  Right now the global port has cached data for things like
 * pkey, smlid, smsl, portstate...
 * It is up to the user of the lib to "refresh" this data.
 * It would be nice if the lib could do that on it's own.
 * Regardless we could have a call to update this information if the user
 * feels the need.
 *
FSTATUS oib_mad_refresh_port_details(struct oib_port *port);
 */


/**
 * Exact same function semantics as umad_get_cas_names
 * However, the list returned is OPA HFIs only.
 *
 * See man umad_get_cas_names for details
 */
int oib_get_hfi_names(char hfis[][UMAD_CA_NAME_LEN], int max);


/** =========================================================================
 * Generic HELPER FUNCTIONs
 */

/* translate ca/port number into a port Guid. Also return other useful structs
 * if non-null pointer passed-in.
 * Warning: Endian conversion not done
 *
 * INPUTS:
 * 	ca - system wide CA number 1-n, if 0 port is a system wide port #
 * 	port - 1-n, if ca is 0, system wide port number, otherwise port within CA
 * 			if 0, 1st active port
 * 	*pCaName - ca name for specified port
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
FSTATUS oib_get_portguid(
		uint32_t             ca,
		uint32_t             port,
		char                *pCaName,
		uint64_t            *pCaGuid,
		uint64_t            *pPortGuid,
		IB_CA_ATTRIBUTES    *pCaAttributes,
		IB_PORT_ATTRIBUTES **ppPortAttributes,
		uint32_t            *pCaCount,
		uint32_t            *pPortCount,
		char                *pRetCaName,
		int                 *pRetPortNum,
		uint64_t            *pRetGIDPrefix
		);

/*
 *  Get MAD status code from most recent PA operation
 *
 * @param port                    Local port to operate on. 
 * 
 * @return 
 *   The corresponding status code.
 */
uint16_t
oib_get_mad_status(
    IN struct oib_port              *port
    );
#endif /* __OIB_UTILS_H__ */
