/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include <errno.h>

#include "fe_connections.h"
#include "fe_error_types.h"
#include "fe_net.h"
#include "fe_protocol.h"

/* Global variables */
extern int g_verbose;

extern boolean timeout(time_t wait, time_t start);

FSTATUS fe_oob_connect(struct login_attr *p_attr, struct net_connection **conn)
{
	FSTATUS e;

	e = SUCCESS;

	/* Establish a connection to the host */
	if (g_verbose) fprintf(stderr, "%s: establish a connection to host\n", __func__);
	if (fe_oob_net_connect(p_attr->host, p_attr->port, conn)) {
		if (g_verbose) fprintf(stderr, "Error, failed to establish a connection to the host\n");
		return CONNECTION | HOST_NAME | PORT_NAME;
	}
	fe_oob_add_to_connection_list(*conn);

	return e;
}

FSTATUS fe_oob_disconnect(struct net_connection *conn)
{
	FSTATUS e = SUCCESS;

	if (fe_oob_net_disconnect(conn)) {
		e |= NO_COMPLETE;
	}
	fe_oob_remove_from_connection_list(conn);
	free(conn);

	return e;
}

FSTATUS fe_oob_send_packet(struct net_connection *conn, uint8_t *data, size_t len)
{
	int err;
	OOB_PACKET packet;

	memset(&packet, 0, sizeof(OOB_PACKET));
	packet.MadData = *((MAD_RMPP *)data);
	packet.Header.HeaderVersion = STL_BASE_VERSION;

	/* Update length of payload in header */
	packet.Header.Length = len;
	BSWAP_OOB_HEADER(&(packet.Header));

	len += sizeof(OOB_HEADER) + sizeof(RMPP_HEADER);

	/* Do nothing if no conn */
	if (!conn)
		return CONNECTION;

	/* Send the header and payload */
	if (fe_oob_net_send(conn, (char *)&packet, len, &err))
		return NO_COMPLETE;

	return(SUCCESS);
}

FSTATUS fe_oob_receive_response(struct net_connection *conn, uint8_t **data, uint32_t *len)
{
	OOB_PACKET *packet;
	FSTATUS e;

	boolean data_null;
	boolean response_acquired;
	time_t current_time;
	time_t start_time;

	/* Get response from server */
	response_acquired = FALSE;
	while (response_acquired != TRUE) {
		e = SUCCESS;
		data_null = TRUE;
		(void)time(&start_time);
		(void)time(&current_time);
		while ((timeout(current_time, start_time) != TRUE) && data_null) {
			(void)fe_oob_net_process(100, 1);
			fe_oob_net_get_next_message(conn, (uint8_t **)data, (int *)len, NULL);
			if (*data != NULL) {
				data_null = FALSE;
			} else {
				if (conn->err) {
					e |= CONNECTION;
					return e;
				}
				(void)time(&current_time);
			}
		}
		if (data_null == TRUE) {
			e |= TIMEOUT;
			response_acquired = TRUE;
		} else {
			*len = 0;
			packet = (OOB_PACKET *)*data;
			BSWAP_OOB_HEADER(&(packet->Header));
			*len = packet->Header.Length;

			/* Reuse pointer for data payload */
			*data = (uint8_t *)&(packet->MadData);

			response_acquired = TRUE;
		}
	}

	return e;
}
