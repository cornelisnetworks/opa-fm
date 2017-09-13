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

#ifndef __OPAMGT_H__
#define __OPAMGT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <infiniband/umad.h>
#include <infiniband/verbs.h>

typedef uint32_t OMGT_STATUS_T;

#define OMGT_STATUS_SUCCESS                 0x00
#define OMGT_STATUS_ERROR                   0x01
#define OMGT_STATUS_INVALID_STATE           0x02
#define OMGT_STATUS_INVALID_OPERATION       0x03
#define OMGT_STATUS_INVALID_SETTING         0x04
#define OMGT_STATUS_INVALID_PARAMETER       0x05
#define OMGT_STATUS_INSUFFICIENT_RESOURCES  0x06
#define OMGT_STATUS_INSUFFICIENT_MEMORY     0x07
#define OMGT_STATUS_COMPLETED               0x08
#define OMGT_STATUS_NOT_DONE                0x09
#define OMGT_STATUS_PENDING                 0x0A
#define OMGT_STATUS_TIMEOUT                 0x0B
#define OMGT_STATUS_CANCELED                0x0C
#define OMGT_STATUS_REJECT                  0x0D
#define OMGT_STATUS_OVERRUN                 0x0E
#define OMGT_STATUS_PROTECTION              0x0F
#define OMGT_STATUS_NOT_FOUND               0x10
#define OMGT_STATUS_UNAVAILABLE             0x11
#define OMGT_STATUS_BUSY                    0x12
#define OMGT_STATUS_DISCONNECT              0x13
#define OMGT_STATUS_DUPLICATE               0x14
#define OMGT_STATUS_POLL_NEEDED             0x15

#define OMGT_STATUS_COUNT                   0x16 /* should be the last value */


/* opaque data defined internally */
struct omgt_port;

#define OMGT_DBG_FILE_SYSLOG ((FILE *)-1)
/**
 * @brief Configuration settings used when opening an omgt_port
 *
 * Configuration options are passed to omgt_port open functions. Current
 * capabilility of settings is limited to configuring logging.
 *
 * error_file and debug_file can be specified as either an open linux FILE, or
 * can use the following special values:
 *   NULL: Disable output of this class of log messages
 *   OMGT_DBG_FILE_SYSLOG: Send this class of messages to syslog.
 *   Note: It is advisable, but not required, to call "openlog" prior to
 *     setting this option
 *
 * @note Enabling debug_file logging will generate a lot of data and may
 * overwhelm syslog. error_file logging with syslog will use the LOG_ERR
 * facility. debug_file logging with syslog will use the LOG_INFO facility.
 *
 */
struct omgt_params {
	FILE             *error_file; /**File to send ERROR log messages to. */
	FILE             *debug_file; /**File to send DEBUG log messages to. */
};


#define OMGT_OOB_SSL_DIR_SIZE 256
#define OMGT_OOB_SSL_FILENAME_SIZE 256
#define OMGT_OOB_SSL_PATH_SIZE (OMGT_OOB_SSL_DIR_SIZE + OMGT_OOB_SSL_FILENAME_SIZE)
/**
 * @brief SSL configuration options and locations of files.
 *
 * A structure to contain the SSL parameters used during setup. An enable flag
 * can be set to turn off SSL.
 */
struct omgt_ssl_params {
	uint32_t enable; /* To enable/disable this feature */
	char     directory[OMGT_OOB_SSL_DIR_SIZE]; /* Directory location of OpenSSL-related files */
	char     certificate[OMGT_OOB_SSL_FILENAME_SIZE]; /* Certificate PEM file */
	char     private_key[OMGT_OOB_SSL_FILENAME_SIZE]; /* Private key PEM file */
	char     ca_certificate[OMGT_OOB_SSL_FILENAME_SIZE]; /* Certificate Authority (CA) certificate PEM file */
	uint32_t cert_chain_depth; /* Limit up to which depth certificates in a chain are used during the
								* verification procedure. If the certificate chain is longer than
								* allowed, the certificates above the limit are ignored. */
	char     dh_params[OMGT_OOB_SSL_FILENAME_SIZE]; /* Diffie-Hellman parameters PEM file */
	uint32_t ca_crl_enable; /* To enable/disable the usage of the CRL PEM file */
	char     ca_crl[OMGT_OOB_SSL_FILENAME_SIZE]; /* CA CRL PEM file */
};

/**
 * @brief OOB conection info
 *
 * A structure to contain all input parameters to connect to the FE through an
 * out-of-band connection.
 */
struct omgt_oob_input {
	char *host; /* ipv4, ipv6, or hostname */
	uint16_t port; /* TCP port of the FE */
	struct omgt_ssl_params ssl_params; /* SSL parameters */
	int is_esm_fe; /* is the FE an ESM */
};


/**
 * @brief Open an in-band opamgt port using the name of the hfi device.
 *
 * This function will allocate and initilize a connection to the local HFI using
 * the HFI device's name and port number. Additionally, per port object logging
 * can be setup using session_params.
 *
 * @param port              port object is allocated and returned
 * @param hfi_name          HFI device name (e.g. "hfi1_0")
 * @param port_num          port number of the hfi starting at 1 (0 is a
 *  						wildcard meaning first active)
 * @param session_params    Parameters to open port with (e.g. Logging streams)
 *
 * @return OMGT_STATUS_T
 *
 * @see omgt_params session_params
 * @see omgt_close_port
 */
OMGT_STATUS_T omgt_open_port(struct omgt_port **port, char *hfi_name, uint8_t port_num, struct omgt_params *session_params);

/**
 * @brief Open an in-band opamgt port by HFI and port number
 *
 * This function will allocate and initilize a connection to the local HFI using
 * the HFI device's number and port's number. Additionally, per port object
 * logging can be setup using session_params.
 *
 * @param port              port object is allocated and returned
 * @param hfi_num           HFI number based on the order of the cards starting
 *  						at 1 (0 is a wildcard meaning first active)
 * @param port_num          port number of the hfi starting at 1 (0 is a
 *  						wildcard meaning first active)
 * @param session_params    Parameters to open port with (e.g. Logging streams)
 *
 * @return OMGT_STATUS_T
 *
 * @see omgt_params session_params
 * @see omgt_close_port
 */
OMGT_STATUS_T omgt_open_port_by_num(struct omgt_port **port, int32_t hfi_num, uint8_t port_num, struct omgt_params *session_params);

/**
 * @brief Open an in-band opamgt port by port GUID
 *
 * This function will allocate and initilize a connection to the local HFI using
 * the HFI device's Port GUID. Additionally, per port object logging can be
 * setup using session_params.
 *
 * @param port              port object is allocated and returned
 * @param port_guid         port GUID of the port
 * @param session_params    Parameters to open port with (e.g. Logging streams)
 *
 * @return OMGT_STATUS_T
 *
 * @see omgt_params session_params
 * @see omgt_close_port
 */
OMGT_STATUS_T omgt_open_port_by_guid(struct omgt_port **port, uint64_t port_guid, struct omgt_params *session_params);

/**
 * @brief Open a out-of-band opamgt port
 *
 * This function will allocate and initilize a connection to the FE through an
 * out-of-band interface. Additionally, per port object logging can be setup
 * using session_params.
 *
 * @param port              port object is allocated and returned
 * @param oob_input         OOB conection info
 * @param session_params    Parameters to open port with (e.g. Logging streams)
 *
 * @return OMGT_STATUS_T
 *
 * @see omgt_oob_input oob_input
 * @see omgt_params session_params
 * @see omgt_close_port
 */
OMGT_STATUS_T omgt_oob_connect(struct omgt_port **port, struct omgt_oob_input *oob_input, struct omgt_params *session_params);

/**
 * @brief Close and free port object
 *
 * This function will close, disconnect, and free any previously allocated and
 * opened connections for both in-band and out-of-band port objects.
 *
 * @param port       port object to close and free.
 *
 * @see omgt_open_port
 * @see omgt_open_port_by_num
 * @see omgt_open_port_by_guid
 * @see omgt_oob_connect
 */
void omgt_close_port(struct omgt_port *port);


/** ============================================================================
 * omgt_port accessor functions for use while in either in-band or out-of-band
 * mode
 */

/**
 * @brief Set debug logging output for an opamgt port
 *
 * Allows Dynamic modification of the debug log target file. Log Settings are
 * initially configured during port open and can be changed at any time with
 * this function. Target file can either be a standard linux flat FILE, NULL to
 * disable, or OMGT_DBG_FILE_SYSLOG to send debug logging to syslog.
 *
 * @param port   port instance to modify logging configuration
 * @param file   Target file for debug logging output
 *
 * @see omgt_params
 */
void omgt_set_dbg(struct omgt_port *port, FILE *file);

/**
 * @brief Set error logging output for an opamgt port
 *
 * Allows Dynamic modification of the error log target file. Log Settings are
 * initially configured during port open and can be changed at any time with
 * this function. Target file can either be a standard Linux flat FILE, NULL to
 * disable, or OMGT_DBG_FILE_SYSLOG to send error logging to syslog.
 *
 * @param port   port instance to modify logging configuration
 * @param file   Target file for error logging
 *
 * @see omgt_params
 */
void omgt_set_err(struct omgt_port *port, FILE *file);


/** ============================================================================
 * omgt_port accessor functions for use while in in-band mode
 *
 * OMGT_STATUS_INVALID_STATE will be returned for calling this function on an
 * out-of-band port
 */

/**
 * @brief Gets Port Prefix for the given port.
 * Retrieves port prefix for an open in-band port.
 * @param port           previously initialized port object for an in-band
 *  					 connection
 * @param prefix         port prefix to be returned
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_port_get_port_prefix(struct omgt_port *port, uint64_t *prefix);
/**
 * @brief Gets Port GUID for the given port.
 * Retrieves port GUID for an open in-band port.
 * @param port           previously initialized port object for an in-band
 *  					 connection
 * @param port_guid      port guid to be returned
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_port_get_port_guid(struct omgt_port *port, uint64_t *port_guid);
/**
 * @brief Gets Port's LID for the given port.
 * Retrieves port's lid for an open in-band port.
 * @param port           previously initialized port object for an in-band
 *  					 connection
 * @param port_lid       lid to be returned
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_port_get_port_lid(struct omgt_port *port, uint32_t *port_lid);
/**
 * @brief Gets HFI's port number for the given port.
 * Retrieves HFI's port number for an open in-band port.
 * @param port           previously initialized port object for an in-band
 *  					 connection
 * @param port_num       port number to be returned (indexed starting at 1)
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_port_get_hfi_port_num(struct omgt_port *port, uint8_t *port_num);
/**
 * @brief Gets HFI's number for the given port.
 * Retrieves HFI's number for an open in-band port.
 * @param port           previously initialized port object for an in-band
 *  					 connection
 * @param hfi_num        hfi number to be returned (indexed starting at 1)
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_port_get_hfi_num(struct omgt_port *port, int32_t *hfi_num);
/**
 * @brief Gets Port's State for the given port.
 * Retrieves Port's State for an open in-band port.
 * @param port           previously initialized port object for an in-band
 *  					 connection
 * @param port_state     Port state to be returned
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_port_get_port_state(struct omgt_port *port, uint8_t *port_state);
/**
 * @brief Gets HFI's Name for the given port.
 * Retrieves HFI's Name for an open in-band port.
 * @param port           previously initialized port object for an in-band
 *  					 connection
 * @param hfi_name       buffer to be filled with HFI's name
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_port_get_hfi_name(struct omgt_port *port, char hfi_name[IBV_SYSFS_NAME_MAX]);


/** ============================================================================
 * omgt_port accessor functions for use while in out-of-band mode
 *
 * OMGT_STATUS_INVALID_STATE will be returned for calling this function on an
 * in-band port
 */

/**
 * @brief Gets IP Version for the given port.
 * Retrieves the IP version currently in use for the open out-of-band port.
 * @param port           previously initialized port object for an out-of-band
 *  					 connection
 * @param ip_version     version of the ip protocal currently in use. Possible
 *  					 output either 4 or 6
 * @return OMGT_STATUS_T
 * @see omgt_port_get_ipv6_addr
 * @see omgt_port_get_ipv4_addr
 */
OMGT_STATUS_T omgt_port_get_ip_version(struct omgt_port *port, uint8_t *ip_version);
/**
 * @brief Gets IPv4 Address for the given port.
 * Retrieves the IPv4 Address currently in use for the open out-of-band port.
 * @param port           previously initialized port object for an out-of-band
 *  					 connection
 * @param ipv4_addr      IPv4 address of the connection currently in use
 * @return OMGT_STATUS_T
 * @see omgt_port_get_ip_version
 * @see omgt_port_get_ipv6_addr
 */
OMGT_STATUS_T omgt_port_get_ipv4_addr(struct omgt_port *port, struct in_addr *ipv4_addr);
/**
 * @brief Gets IPv6 Address for the given port.
 * Retrieves the IPv6 Address currently in use for the open out-of-band port.
 * @param port           previously initialized port object for an out-of-band
 *  					 connection
 * @param ipv6_addr      IPv6 address of the connection currently in use
 * @return OMGT_STATUS_T
 * @see omgt_port_get_ip_version
 * @see omgt_port_get_ipv4_addr
 */
OMGT_STATUS_T omgt_port_get_ipv6_addr(struct omgt_port *port, struct in6_addr *ipv6_addr);
/**
 * @brief Gets IP Address for the given port in text.
 * Retrieves the IP Address currently in use for the open out-of-band port and
 * returns it in text format in a text buffer.
 * @param port           previously initialized port object for an out-of-band
 *  					 connection
 * @param buf            buffer to store the ip address of the connection
 *  					 currently in use
 * @param buf_len        size of the buffer
 * @return OMGT_STATUS_T
 * @see omgt_port_get_ip_version
 * @see omgt_port_get_ipv4_addr
 * @see omgt_port_get_ipv6_addr
 */
OMGT_STATUS_T omgt_port_get_ip_addr_text(struct omgt_port *port, char buf[], size_t buf_len);


/** ============================================================================
 * general functions that do not require an omgt_port
 */

/**
 * @brief Gets an array of all HFI names available on system
 *
 * This function preforms the same as umad_get_cas_names(), but is restricted to
 * OPA ports only. HFIs will be in HFI num order.
 *
 * @param hfis       Pointer to array of size char [max][UMAD_CA_NAME_LEN]
 * @param max        Maximum number of names to return
 * @param hfi_count  The number of valid entries in hfis
 *
 * @see umad_get_cas_names
 */
OMGT_STATUS_T omgt_get_hfi_names(char hfis[][UMAD_CA_NAME_LEN], int32_t max, int32_t *hfi_count);

#ifdef __cpluspluc
}
#endif

#endif /* __OPAMGT_H__ */
