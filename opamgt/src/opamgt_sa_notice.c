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

#include <poll.h>
#include <errno.h>
#include <string.h> /* strerror() */


#define OPAMGT_PRIVATE 1
#include "ib_utils_openib.h"
#include "opamgt_dump_mad.h"
#include "omgt_oob_net.h"
#include "omgt_oob_protocol.h"

#include <opamgt.h>
#include <opamgt_sa.h>
#include <opamgt_sa_notice.h>
#include <iba/stl_sa_priv.h>


/**===========================================================================*/
omgt_sa_registration_t* omgt_sa_find_reg(struct omgt_port *port, uint16_t trap_num)
{
	omgt_sa_registration_t *curr = port->regs_list;
	while (curr != NULL) {
		if (curr->trap_num == trap_num) {
			return curr;
		}
		curr = curr->next;
	}
	return NULL;
}
/**===========================================================================*/
OMGT_STATUS_T omgt_sa_register_trap(struct omgt_port *port, uint16_t trap_num,
	void *context)
{
	OMGT_STATUS_T status;
	int ret;
	omgt_sa_registration_t *reg;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, Trap registration not Supported\n");
		return OMGT_STATUS_INVALID_STATE;
	}

	reg = (omgt_sa_registration_t *)calloc(1, sizeof*reg);
	if (reg == NULL) {
		OMGT_OUTPUT_ERROR(port, "failed to allocate reg structure\n");
		return OMGT_STATUS_ERROR;
	}

	status = FTIMEOUT;
	if (omgt_lock_sem(&port->lock)) {
		OMGT_OUTPUT_ERROR(port, "failed to acquire lock (status: %d)\n", status);
		free(reg);
		return OMGT_STATUS_ERROR;
	}

	if (omgt_sa_find_reg(port, trap_num)) {
		omgt_unlock_sem(&port->lock);
		free(reg);
		return OMGT_STATUS_SUCCESS;
	}

	if ((ret = create_sa_qp(port)) != 0) {
		omgt_unlock_sem(&port->lock);
		OMGT_OUTPUT_ERROR(port, "failed to create notice QP for trap (%u) registration (status: %d)\n",
			trap_num, ret);
	} else if ((ret = userspace_register(port, trap_num, reg)) != 0) {
		omgt_unlock_sem(&port->lock);
		OMGT_OUTPUT_ERROR(port, "failed to register for trap (%u) (status: %d)\n",
			trap_num, ret);
	}

	if (ret) {
		free(reg);
		return OMGT_STATUS_ERROR;
	}

	reg->user_context = context;
	reg->trap_num = trap_num;
	omgt_sa_add_reg_unsafe(port, reg);

	omgt_unlock_sem(&port->lock);
	return OMGT_STATUS_SUCCESS;
}
/**===========================================================================*/
OMGT_STATUS_T omgt_sa_unregister_trap(struct omgt_port *port, uint16_t trap_num)
{
	OMGT_STATUS_T status;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, Trap (un)registration not Supported\n");
		return OMGT_STATUS_INVALID_STATE;
	}

	if (omgt_lock_sem(&port->lock))
		return OMGT_STATUS_ERROR;

	status = omgt_sa_remove_reg_by_trap_unsafe(port, trap_num);

	omgt_unlock_sem(&port->lock);
	return status;
}
/**===========================================================================*/
OMGT_STATUS_T omgt_sa_get_notice_report(struct omgt_port *port, STL_NOTICE **notice,
	size_t *notice_len, void **context, int poll_timeout_ms)
{
	int rc;
	int bytes_read;
	struct pollfd pollfd[1];
	uint8_t rcv_buf[2048];
	struct omgt_thread_msg *msg;
	STL_NOTICE *notice_buf = NULL;
	omgt_sa_registration_t *reg;
	uint16_t trap_num;

	if (port->is_oob_enabled) {
		uint32_t oob_recv_len = 0;
		OOB_PACKET *packet = NULL;

		if (!port->is_oob_notice_setup) {
			OMGT_STATUS_T status;
			if ((status = omgt_oob_net_connect(port, &port->notice_conn)) != FSUCCESS) {
				OMGT_OUTPUT_ERROR(port, "failed to establish a connection to the host: %u\n", status);
				return status;
			}
			port->is_oob_notice_setup = TRUE;
		}

		do {
			/* Process IP 'blobs' (TCP Segments) */
			omgt_oob_net_process(port, port->notice_conn, poll_timeout_ms, 1);
			/* If failure return error */
			if (port->notice_conn->err) {
				return OMGT_STATUS_ERROR;
			}
		} while (port->notice_conn->blob_in_progress);

		/* Check if we are done getting all the 'blobs' */
		omgt_oob_net_get_next_message(port->notice_conn, (uint8_t **)&packet, (int *)&oob_recv_len);
		/* Handle Timeout Case */
		if (packet == NULL) {
			return OMGT_STATUS_TIMEOUT;
		}

		/* Handle packet */
		oob_recv_len = 0;
		BSWAP_OOB_HEADER(&(packet->Header));
		oob_recv_len = packet->Header.Length;
		BSWAP_MAD_HEADER((MAD *)&packet->MadData.common);

		/* Handle Trap/Notice Report */
		if (packet->MadData.common.AttributeID == STL_MCLASS_ATTRIB_ID_NOTICE) {
			/* Remove Header Bytes */
			oob_recv_len -= (sizeof(MAD_COMMON) - sizeof(RMPP_HEADER) - sizeof(SA_HDR));

			/* Allocate the Notice */
			notice_buf = (STL_NOTICE *)calloc(1, oob_recv_len);
			if (notice_buf == NULL) {
				OMGT_OUTPUT_ERROR(port, "failed to allocate notice buffer\n");
				return OMGT_STATUS_INSUFFICIENT_MEMORY;
			}

			/* Copy and allocate the Data */
			SA_MAD *samad = (SA_MAD *)&packet->MadData.common;
			memcpy(notice_buf, samad->Data, oob_recv_len);
			BSWAP_STL_NOTICE(notice_buf);
			trap_num = notice_buf->Attributes.Generic.TrapNumber;
			OMGT_DBGPRINT(port, "trap message %u: %d bytes\n", trap_num, oob_recv_len);

			/* Find Context (Not in OOB) */
			if (context) {
				*context = NULL;
			}
			/* Set Notice struct and length */
			*notice = notice_buf;
			*notice_len = oob_recv_len;
			return OMGT_STATUS_SUCCESS;
		/* Handle Inform Info */
		} else if (packet->MadData.common.AttributeID == STL_MCLASS_ATTRIB_ID_INFORM_INFO) {
			SA_MAD *samad = (SA_MAD *)&packet->MadData.common;
			STL_INFORM_INFO *informinfo = (STL_INFORM_INFO *)samad->Data;
			trap_num = ntoh16(informinfo->u.Generic.TrapNumber);

			/* Free Packet */
			free(packet);

			OMGT_OUTPUT_ERROR(port, "Registration of Trap message timed out: Trap %u\n", trap_num);
			return OMGT_STATUS_DISCONNECT;
		/* Otherwise error */
		} else {
			SA_MAD *samad = (SA_MAD *)&packet->MadData.common;
			OMGT_OUTPUT_ERROR(port, "Unexpected OOB MAD received: %s %s(%s)\n",
				stl_class_str(samad->common.BaseVersion, samad->common.MgmtClass),
				stl_method_str(samad->common.BaseVersion, samad->common.MgmtClass, samad->common.mr.AsReg8),
				stl_attribute_str(samad->common.BaseVersion, samad->common.MgmtClass, hton16(samad->common.AttributeID)));

			/* Free Packet */
			free(packet);
			return OMGT_STATUS_ERROR;
		}
	}
	pollfd[0].fd = port->umad_port_sv[0];
	pollfd[0].events = POLLIN;
	pollfd[0].revents = 0;

	/* Poll for event */
	if ((rc = poll(pollfd, 1, poll_timeout_ms)) < 0) {
		OMGT_OUTPUT_ERROR(port, "trap poll failed : %s\n", strerror(errno));
		return FERROR;

	} else if (rc == 0) {
		/* Handle Timeout Case */
		return OMGT_STATUS_TIMEOUT;
	} else if (pollfd[0].revents & POLLIN) {
		/* Read from Stream */
		bytes_read = read(port->umad_port_sv[0], rcv_buf, sizeof(rcv_buf));
		if (bytes_read <= 0) {
			OMGT_OUTPUT_ERROR(port, "user event read failed : %s\n", strerror(errno));
			return OMGT_STATUS_ERROR;
		}

		/* Determine Message Type */
		msg = (struct omgt_thread_msg *)rcv_buf;
		if (OMGT_TH_EVT_TRAP_MSG == msg->evt) {
			int bufSize = MIN(msg->size,sizeof(rcv_buf)-sizeof(*msg));
			/* Thread Event Type : Trap Message */

			/* Allocate the Notice */
			notice_buf = (STL_NOTICE *)calloc(1, bufSize);
			if (notice_buf == NULL) {
				OMGT_OUTPUT_ERROR(port, "failed to allocate notice buffer\n");
				return OMGT_STATUS_INSUFFICIENT_MEMORY;
			}
			/* Copy and allocate the Data */
			memcpy(notice_buf, rcv_buf + sizeof(*msg), bufSize);
			BSWAP_STL_NOTICE(notice_buf);
			trap_num = notice_buf->Attributes.Generic.TrapNumber;
			OMGT_DBGPRINT(port, "trap message %u: %d bytes\n", trap_num, (int)(msg->size));

			/* Find Context */
			if (context) {
				reg = omgt_sa_find_reg(port, trap_num);
				if (reg == NULL) {
					OMGT_OUTPUT_ERROR(port, "failed to retrieve registration: trap %u\n", trap_num);
					*context = NULL;
				} else {
					/* Set context */
					*context = reg->user_context;
				}
			}

			/* Set Notice struct and length */
			*notice = notice_buf;
			*notice_len = msg->size;
			return OMGT_STATUS_SUCCESS;

		} else if (OMGT_TH_EVT_TRAP_REG_ERR_TIMEOUT == msg->evt) {
			/* Thread Event Type : Registration Timeout */
			STL_INFORM_INFO *informinfo = (STL_INFORM_INFO *)msg->data;
			trap_num = ntoh16(informinfo->u.Generic.TrapNumber);
			OMGT_OUTPUT_ERROR(port, "Registration of Trap message timed out: Trap %u\n", trap_num);

			return OMGT_STATUS_DISCONNECT;
		} else {
			OMGT_OUTPUT_ERROR(port, "user event read invalid message: %u\n", msg->evt);
			return OMGT_STATUS_ERROR;
		}
	} else {
		OMGT_OUTPUT_ERROR(port, "trap poll unexpected result : %d\n", pollfd[0].revents);
		return OMGT_STATUS_ERROR;
	}
	return OMGT_STATUS_SUCCESS;
}
