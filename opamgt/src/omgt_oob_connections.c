/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include <errno.h>
#include <sys/time.h>

#define OPAMGT_PRIVATE 1
#include "ib_utils_openib.h"
#include "omgt_oob_net.h"
#include "omgt_oob_protocol.h"
#include "opamgt_dump_mad.h"

OMGT_STATUS_T omgt_oob_connect(struct omgt_port **port, struct omgt_oob_input *oob_input, struct omgt_params *session_params)
{
	struct omgt_port *prt = calloc(1, sizeof(struct omgt_port));
	struct net_connection *conn = NULL;
	FSTATUS status = FSUCCESS;

	if (prt == NULL) {
		return OMGT_STATUS_INSUFFICIENT_MEMORY;
	}

	/* Copy session_params into port struct */
	if (session_params) {
		prt->dbg_file = session_params->debug_file;
		prt->error_file = session_params->error_file;
	} else {
		prt->dbg_file = NULL;
		prt->error_file = NULL;
	}

	prt->ms_timeout = OMGT_DEF_TIMEOUT_MS;
	prt->retry_count = OMGT_DEF_RETRY_CNT;

	/* copy login info into port struct */
	prt->oob_input = *oob_input;

	prt->is_oob_enabled = TRUE;
	/* Establish a connection to the host */
	OMGT_DBGPRINT(prt, "establish a connection to host\n");
	if ((status = omgt_oob_net_connect(prt, &conn)) != FSUCCESS) {
		OMGT_OUTPUT_ERROR(prt, "failed to establish a connection to the host: %u\n", status);
		if (prt->is_ssl_enabled && prt->is_ssl_initialized) {
			if (prt->x509_store) {
				X509_STORE_free(prt->x509_store);
				prt->x509_store = NULL;
				prt->is_x509_store_initialized = 0;
			}
			if (prt->dh_params) {
				DH_free(prt->dh_params);
				prt->dh_params = NULL;
				prt->is_dh_params_initialized = 0;
			}
			if (prt->ssl_context) {
				SSL_CTX_free(prt->ssl_context);
				prt->ssl_context = NULL;
			}

			ERR_free_strings();
			prt->is_ssl_initialized = 0;
		}
		free(prt);
		return OMGT_STATUS_UNAVAILABLE;
	}
	prt->conn = conn;
	prt->is_oob_notice_setup = FALSE;
	*port = prt;
	return OMGT_STATUS_SUCCESS;
}

FSTATUS omgt_oob_disconnect(struct omgt_port *port, struct net_connection *conn)
{
	FSTATUS status = FSUCCESS;

	if (conn) {
		if (port->is_ssl_enabled) {
			if (conn->ssl_session) {
				(void)SSL_shutdown(conn->ssl_session);
				SSL_free(conn->ssl_session);
				conn->ssl_session = NULL;
			}
		}
		if ((status = omgt_oob_net_disconnect(port, conn)) != FSUCCESS) {
			OMGT_OUTPUT_ERROR(port, "failed to disconnect from socket: %u\n", status);
		}
	}
	return status;
}

FSTATUS omgt_oob_send_packet(struct omgt_port *port, uint8_t *data, size_t len)
{
	OOB_PACKET packet;

	memset(&packet, 0, sizeof(OOB_PACKET));
	packet.MadData = *((MAD_RMPP *)data);
	packet.Header.HeaderVersion = STL_BASE_VERSION;

	/* Update length of payload in header */
	packet.Header.Length = len;
	BSWAP_OOB_HEADER(&(packet.Header));

	len += sizeof(OOB_HEADER) + sizeof(RMPP_HEADER);

	/* Do nothing if no conn */
	if (!port || !port->conn)
		return FINVALID_PARAMETER;

	/* Send the header and payload */
	return omgt_oob_net_send(port, (uint8_t *)&packet, len);
}

FSTATUS omgt_oob_receive_response(struct omgt_port *port, uint8_t **data, uint32_t *len)
{
	OOB_PACKET *packet = NULL;
	FSTATUS e;

	boolean data_null;
	boolean response_acquired;
	struct timeval start, current;
	uint64_t delta_ms, total_timeout;

	/* Do nothing if no conn */
	if (!port || !port->conn)
		return FINVALID_PARAMETER;

	/* Get response from server */
	response_acquired = FALSE;
	while (response_acquired != TRUE) {
		e = FSUCCESS;
		data_null = TRUE;
		(void)gettimeofday(&start, NULL);
		(void)gettimeofday(&current, NULL);
		delta_ms = 0;
		total_timeout = port->ms_timeout * (port->retry_count + 1);
		while (delta_ms < total_timeout && data_null) {
			omgt_oob_net_process(port, port->conn, 100, 1);
			omgt_oob_net_get_next_message(port->conn, (uint8_t **)&packet, (int *)len);
			if (packet != NULL) {
				data_null = FALSE;
			} else {
				if (port->conn->err) {
					return FERROR;
				}
				(void)gettimeofday(&current, NULL);
				/* Get Delta in ms */
				delta_ms = ((current.tv_sec - start.tv_sec) * 1000) + ((current.tv_usec - start.tv_usec) / 1000);
			}
		}
		if (data_null == TRUE) {
			e = FTIMEOUT;
			response_acquired = TRUE;
		} else {
			*len = 0;
			BSWAP_OOB_HEADER(&(packet->Header));
			*len = packet->Header.Length;

			if (port->dbg_file) {
				OMGT_DBGPRINT(port, "Received MAD: socket %d, length=%u\n", port->conn->sock, *len);
				omgt_dump_mad(port->dbg_file, (uint8_t *)&(packet->MadData), *len, "rcv mad\n");
			}

			// Allocate and copy to new buffer
			*data = calloc(1, *len);
			if (*data == NULL) {
				OMGT_OUTPUT_ERROR(port, "can't alloc return buffer length %u\n", *len);
				return FINSUFFICIENT_MEMORY;
			}
			memcpy(*data, (uint8_t *)&(packet->MadData), *len);

			response_acquired = TRUE;
		}
	}
	if (packet != NULL) {
		free(packet);
	}
	return e;
}

