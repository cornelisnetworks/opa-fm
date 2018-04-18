/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/
/* [ICS VERSION STRING: unknown] */

/*
 * OFED "User SA" support.
 *
 * This module provides userspace access to the "User SA" components, which
 * exposes SA Notice multiplexing functionality to userspace.
 *
 * Note that this implementation follows the opamgt model using the omgt_port
 * object to track the connection.  It creates the kernel channel on
 * initialization and holds it open.  Explicit deregistration and shutdown is
 * not required, as libibsa and the kernel will automatically clear
 * registrations when a channel is closed.
 */

#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>
#include <infiniband/umad_types.h>
#include <infiniband/umad_sa.h>
#include <poll.h>

#define OPAMGT_PRIVATE 1

#include "ib_utils_openib.h"
#include "ib_notice_net.h"
#include "iba/public/ibyteswap.h"
#include "iba/stl_sa_types.h"
#include "iba/stl_mad_priv.h"

#define OMGT_SA_MAX_REGISTRANTS 10

#define OMGT_SA_DEVICE_NAME_LEN_MAX 32

#define OMGT_SA_DEFAULT_LOCK_TIMEOUT 5000000ull // 5 seconds

#ifndef container_of
#define container_of(ptr, type, field) \
    ((type *) ((void *) ptr - offsetof(type, field)))
#endif

struct ibv_sa_event {
	void *context;
	int status;
	int attr_count;
	int attr_size;
	int attr_offset;
	uint16_t attr_id;
	void *attr;
};
struct omgt_sa_event {
    struct ibv_sa_event event;
    struct ibv_sa_event_channel *channel;
    void *data;
};

int omgt_lock_sem(SEMAPHORE* const	pSema)
{
	int rc = EIO;
	unsigned int ms = (OMGT_SA_DEFAULT_LOCK_TIMEOUT + 999)/1000;
	do {
		if (0 == sem_trywait( pSema )) {
			rc = 0;
			break;
		} else if (errno != EAGAIN) {
			rc = errno;
			break;
		}
        usleep(1000);
	} while (ms--);
	return rc;
}

void omgt_unlock_sem(SEMAPHORE * const pSema)
{
    sem_post(pSema);
}

/**
 * port->lock must be held
 */
static int reg_sa_msg_mr(struct omgt_port *port, struct omgt_sa_msg *msg,
                          enum ibv_wr_opcode opcode,
                          uint32_t rem_qpn, uint32_t rem_qkey)
{
    msg->mr = ibv_reg_mr(port->sa_qp_pd, msg->data, sizeof(msg->data),
                        IBV_ACCESS_LOCAL_WRITE);
    if (!msg->mr) {
        OMGT_OUTPUT_ERROR(port, "Notice: sa msg register memory region failed\n");
        return (-EIO);
    }

    msg->sge.addr = (uint64_t)msg->data;
    msg->sge.length = sizeof(msg->data);
    msg->sge.lkey = msg->mr->lkey;
    msg->in_q = 0;

    if (opcode == IBV_WR_SEND) {
        msg->wr.send.wr_id = (uint64_t)msg;
        msg->wr.send.next = NULL;
        msg->wr.send.sg_list = &msg->sge;
        msg->wr.send.num_sge = 1;
        msg->wr.send.opcode = IBV_WR_SEND;
        msg->wr.send.send_flags = IBV_SEND_SIGNALED;
        msg->wr.send.wr.ud.ah = port->sa_ah;
        msg->wr.send.wr.ud.remote_qpn = rem_qpn;
        msg->wr.send.wr.ud.remote_qkey = rem_qkey;
    } else {
        msg->wr.recv.wr_id = (uint64_t)msg;
        msg->wr.recv.next = NULL;
        msg->wr.recv.sg_list = &msg->sge;
        msg->wr.recv.num_sge = 1;
    }

    return (0);
}

/**
 * port->lock must be held
 */
static struct omgt_sa_msg * alloc_send_sa_msg(struct omgt_port *port)
{
    struct omgt_sa_msg *msg;

    if (!port->sa_ah) {
        struct ibv_ah_attr attr;
	int err = 0;
        memset(&attr, 0, sizeof(attr));

        attr.dlid = (uint16_t)port->umad_port_cache.sm_lid;
	if (omgt_is_ext_lid(port->umad_port_cache.base_lid) ||
		omgt_is_ext_lid(port->umad_port_cache.sm_lid)) {
       		attr.is_global = 1;
		attr.grh.hop_limit = 1;
		attr.grh.sgid_index = 0;
		attr.grh.dgid.global.subnet_prefix =
				port->umad_port_cache.gid_prefix;
		/* Not too sure why ntoh64 is required */
		attr.grh.dgid.global.interface_id =
				ntoh64(omgt_create_gid(port->umad_port_cache.sm_lid));
	}

	if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "failed to acquire lock (err: %d)\n", err);
		return (NULL);
	}
	attr.sl = port->umad_port_cache.sm_sl;
	omgt_unlock_sem(&port->umad_port_cache_lock);

        attr.port_num = port->hfi_port_num;
        port->sa_ah = ibv_create_ah(port->sa_qp_pd, &attr);
        if (!port->sa_ah) {
		OMGT_OUTPUT_ERROR(port, "failed to create SA AH (err: %d)\n", errno);
		return (NULL);
	}
    }

    msg = calloc(1, sizeof(*msg));
    if (!msg)
        return (NULL);

    if (reg_sa_msg_mr(port, msg, IBV_WR_SEND, 1, UMAD_QKEY)) {
        free(msg);
        return (NULL);
    }

    msg->prev = msg;
    msg->next = msg;

    return (msg);
}

/**
 * port->lock must be held
 */
static void free_sa_msg(struct omgt_sa_msg *msg)
{
	if (msg->mr)
        ibv_dereg_mr(msg->mr);
    free(msg);
}


/**
 * Adds a registration to the list.
 * NOTE: Caller must hold the lock.
 *
 * @param port   port opened by omgt_open_port_ 
 * @param reg    Pointer to the registration structure. 
 *  
 * @return       none 
 */
void omgt_sa_add_reg_unsafe(struct omgt_port *port, omgt_sa_registration_t *reg)
{
	reg->next = port->regs_list;
	port->regs_list = reg;
}

static void set_sa_msg_tid(struct omgt_port *port, struct umad_sa_packet *sa_pkt)
{
    port->next_tid++;
    if (port->next_tid == 0)
        port->next_tid++;
    sa_pkt->mad_hdr.tid = hton64((uint64_t)port->next_tid);
}

static void set_sa_common_stl_inform_info(struct omgt_port *port, struct umad_sa_packet *sa_pkt)
{
     STL_INFORM_INFO *informinfo;

    sa_pkt->mad_hdr.base_version = STL_BASE_VERSION;
    sa_pkt->mad_hdr.mgmt_class = MCLASS_SUBN_ADM;
    sa_pkt->mad_hdr.class_version = STL_SA_CLASS_VERSION;
    sa_pkt->mad_hdr.method = MMTHD_SET;
    sa_pkt->mad_hdr.attr_id = hton16(STL_MCLASS_ATTRIB_ID_INFORM_INFO);

    sa_pkt->rmpp_hdr.rmpp_version = UMAD_RMPP_VERSION;
    sa_pkt->rmpp_hdr.rmpp_type = 0;

    informinfo = (STL_INFORM_INFO *)sa_pkt->data;

    informinfo->LIDRangeBegin = UINT32_MAX;
    informinfo->IsGeneric = 1;
    informinfo->Type = UINT16_MAX;
    informinfo->u.Generic.u2.s.ProducerType = 0xFFFFFF;
}

static void set_sa_common_stl_response_notice(struct omgt_port *port, struct umad_sa_packet *sa_pkt)
{
    STL_NOTICE *notice;

    sa_pkt->mad_hdr.base_version = STL_BASE_VERSION;
    sa_pkt->mad_hdr.mgmt_class = MCLASS_SUBN_ADM;
    sa_pkt->mad_hdr.class_version = STL_SA_CLASS_VERSION;
    sa_pkt->mad_hdr.method = MMTHD_REPORT_RESP;
    sa_pkt->mad_hdr.attr_id = hton16(STL_MCLASS_ATTRIB_ID_NOTICE);

    sa_pkt->rmpp_hdr.rmpp_version = UMAD_RMPP_VERSION;
    sa_pkt->rmpp_hdr.rmpp_type = 0;

    notice = (STL_NOTICE *)sa_pkt->data;

    // if the Type is set to 0x7f (empty) all other fields are unused
    notice->Attributes.Generic.u.AsReg32 = 0;
    notice->Attributes.Generic.u.s.Type = 0x7f;
}

static void post_send_sa_msg(struct omgt_port *port,
                             struct omgt_sa_msg *msg,
                             enum omgt_reg_retry_state resend)
{
    int rc;
    struct ibv_send_wr *bad_wr = NULL;

    if (msg->in_q) {
        OMGT_OUTPUT_ERROR(port, "msg (%p) is already in the send Q!!!\n", msg);
        return;
    }

    if (!msg->retries) {
        OMGT_OUTPUT_ERROR(port, "msg (%p) has timed out!!!\n", msg);
        return;
    }

    if (port->outstanding_sends_cnt >= port->num_userspace_send_buf) {
        OMGT_OUTPUT_ERROR(port, "no send buffers\n");
        return;
    }

    if (OMGT_RRS_SEND_RETRY == resend) {
    	msg->retries--;
    	if (!msg->retries) {
    		OMGT_DBGPRINT(port, "Timeout sending SA msg.\n");
    		return;
    	}
    }
    set_sa_msg_tid(port, (struct umad_sa_packet *)msg->data);
    if ((rc = ibv_post_send(port->sa_qp, &(msg->wr.send), &bad_wr)) == 0) {
        port->outstanding_sends_cnt++;
        msg->in_q = 1;
    } else {
        OMGT_OUTPUT_ERROR(port, "Notice: post send WR failed: %s: Aborting send.\n",
                     strerror(rc));
    }
}

static int omgt_post_notice_recvs(struct omgt_port *port)
{
    int i = 0;
    int post_count = 0;

    /* post recv msgs */
    for (i = 0; i < port->num_userspace_recv_buf; i++) {
        if (reg_sa_msg_mr(port, &port->recv_bufs[i], 0, 0, 0))
            goto init_error;
    }
    for (i = 0; i < port->num_userspace_recv_buf; i++) {
        if (ibv_post_recv(port->sa_qp, &port->recv_bufs[i].wr.recv, NULL)) {
            port->recv_bufs[i].in_q = 0;
        } else {
            post_count++;
            port->recv_bufs[i].in_q = 1;
        }
    }
    if (!post_count)
        goto init_error;

    return (0);

init_error:
    for (/* */; i >= 0; i--) {
        ibv_dereg_mr(port->recv_bufs[i].mr);
    }
    return (-EIO);
}

static struct omgt_sa_msg *
find_req_by_tid(struct omgt_port *port, uint32_t tid)
{
    struct omgt_sa_msg *rc = NULL;
    struct omgt_sa_msg *msg;

    OMGT_DBGPRINT(port, "find req tid 0x%x\n", tid);

    LIST_FOR_EACH(&port->pending_reg_msg_head, msg) {
        struct umad_sa_packet *sa_pkt = (struct umad_sa_packet *)msg->data;
        uint32_t mtid = ntoh64(sa_pkt->mad_hdr.tid) & 0xffffffff;
        OMGT_DBGPRINT(port, "found tid 0x%x\n", mtid);
        if (mtid == tid) {
            rc = msg;
            break;
        }
    }

    return rc;
}

static void process_sa_get_resp(struct omgt_port *port, struct umad_sa_packet *sa_pkt)
{
    struct omgt_sa_msg *req;
    STL_INFORM_INFO *informinfo = (STL_INFORM_INFO *)sa_pkt->data;
    uint16_t trap_num = ntoh16(informinfo->u.Generic.TrapNumber);

    omgt_lock_sem(&port->lock);

    /* find the registration for this response */
    req = find_req_by_tid(port, ntoh64(sa_pkt->mad_hdr.tid) & 0xffffffff);
    if (req) {
        if (informinfo->Subscribe == 1) {
            OMGT_DBGPRINT(port, "registration complete for trap %d; req %p\n", trap_num, req);
        } else {
            OMGT_DBGPRINT(port, "UN-registration complete for trap %d; req %p\n", trap_num, req);
        }
        /* Check if the registration has been freed */
        if (req->reg)
            req->reg->reg_msg = NULL;
        LIST_DEL(req);
        free_sa_msg(req);
    } else {
        OMGT_OUTPUT_ERROR(port, "Unknown get response; 'trap num' %d\n", trap_num);
    }
    omgt_unlock_sem(&port->lock);
}

/* Recv'ed messages will a GRH on them or at least space (40 bytes) */
/* for a GRH which we need to remove.                               */
static struct umad_sa_packet *
sa_pkt_from_recv_msg(struct omgt_sa_msg *msg)
{
    return (struct umad_sa_packet *)&msg->data[sizeof(struct ibv_grh)];
}

static void process_sa_report(struct omgt_port *port, struct umad_sa_packet *sa_pkt)
{
    struct omgt_sa_msg *response_msg = NULL;
    struct umad_sa_packet *response_pkt = NULL;
    struct ibv_send_wr *bad_wr = NULL;

    STL_NOTICE *notice = (STL_NOTICE *)sa_pkt->data;
    STL_TRAP_GID *notice_gid = (STL_TRAP_GID *)&notice->Data[0];
    struct omgt_thread_msg thread_msg;
    struct iovec iov[2];
    size_t write_count, write_size;
    uint16_t trap_num = ntoh16(notice->Attributes.Generic.TrapNumber);

    // create and send ReportResp to trap notify
    if (omgt_lock_sem(&port->lock)) {
        OMGT_OUTPUT_ERROR(port, "failed to acquire lock (status: %d)\n", FTIMEOUT);
        return;
    }

    response_msg = alloc_send_sa_msg(port);
    if (response_msg)
    {
        STL_NOTICE *notice_resp;
        int rc;
        memset(response_msg->data, 0, sizeof(response_msg->data));
        response_pkt = (struct umad_sa_packet *)response_msg->data;
        set_sa_common_stl_response_notice(port, response_pkt);
        notice_resp = (STL_NOTICE *)response_pkt->data;
        BSWAP_STL_NOTICE(notice_resp);
        response_pkt->mad_hdr.tid = sa_pkt->mad_hdr.tid;

        if ((rc = ibv_post_send(port->sa_qp, &(response_msg->wr.send), &bad_wr)) != 0) {
            OMGT_OUTPUT_ERROR(port, "Notice: post send WR failed: %s: Aborting send.\n",
                         strerror(rc));
        }

        free_sa_msg(response_msg);
    }
    omgt_unlock_sem(&port->lock);

    thread_msg.size = sizeof *notice;
    thread_msg.evt  = OMGT_TH_EVT_TRAP_MSG;

    iov[0].iov_base = &thread_msg;
    iov[0].iov_len  = sizeof thread_msg;
    iov[1].iov_base = notice;
    iov[1].iov_len  = sizeof *notice;
    write_size = iov[0].iov_len + iov[1].iov_len;

    if ( write_size !=
        (write_count = writev(port->umad_port_sv[1], iov, 2)) )
         OMGT_OUTPUT_ERROR(port, "bad write count %d\n", (int)write_count);

    OMGT_DBGPRINT(port, "process_sa_report: msg queued - trap %d gid %02x%02x%02x%02x%02x%02x%02x%02x\n",
        trap_num, notice_gid->Gid.Raw[8], notice_gid->Gid.Raw[9], notice_gid->Gid.Raw[10], notice_gid->Gid.Raw[11],
        notice_gid->Gid.Raw[12], notice_gid->Gid.Raw[13], notice_gid->Gid.Raw[14], notice_gid->Gid.Raw[15]);
}

static void process_sa_rcv_msg(struct omgt_port *port, struct omgt_sa_msg *msg)
{
    struct umad_sa_packet *sa_pkt = sa_pkt_from_recv_msg(msg);

    switch (sa_pkt->mad_hdr.method) {
        case UMAD_METHOD_GET_RESP:
            process_sa_get_resp(port, sa_pkt);
            break;
        case UMAD_METHOD_REPORT:
            process_sa_report(port, sa_pkt);
            break;
        default:
            OMGT_OUTPUT_ERROR(port, "unknown 'message' received : method 0x%x\n", sa_pkt->mad_hdr.method);
            break;
    }
}

int repost_pending_registrations(struct omgt_port *port)
{
    int new_timeout_ms = -1;
    struct omgt_sa_msg *msg;
    struct omgt_sa_msg *del_msg;
    struct omgt_thread_msg thread_msg;
    omgt_sa_registration_t *reg;
    struct iovec iov[2];
    size_t write_size, write_count;

    omgt_lock_sem(&port->lock);

    LIST_FOR_EACH(&port->pending_reg_msg_head, msg) {
    	if (msg->retries) {
	        new_timeout_ms = NOTICE_REG_TIMEOUT_MS;
	        post_send_sa_msg(port, msg, OMGT_RRS_SEND_RETRY);
    	} else {
		/*
		 * When the registration is unregistered (in
		 * userspace_unregister()), msg->reg is NULL and we could not
		 * send the timeout event anymore (the caller is not waiting
		 * for notification anymore).
		 */
		if (msg->reg) {
			reg = msg->reg;
			thread_msg.size = sizeof *reg;
			thread_msg.evt  = OMGT_TH_EVT_TRAP_REG_ERR_TIMEOUT;

			iov[0].iov_base = &thread_msg;
			iov[0].iov_len  = sizeof thread_msg;
			iov[1].iov_base = reg;
			iov[1].iov_len  = sizeof *reg;
			write_size = iov[0].iov_len + iov[1].iov_len;

			write_count = writev(port->umad_port_sv[1], iov, 2);
			if ( write_size != write_count)
				OMGT_OUTPUT_ERROR(port, "bad write count %d\n",
					     (int)write_count);
		}

		// detach the msg to be deleted from the list first
		del_msg = msg;
		msg = msg->prev;
		if (del_msg->reg) {
		OMGT_DBGPRINT(port, "registration timeout on trap %d : req %p\n",
			 del_msg->reg->trap_num, del_msg->reg);
		} else {
			OMGT_DBGPRINT(port, "registration timeout on trap: No information available.\n");
		}
		if (del_msg->reg) 
			del_msg->reg->reg_msg = NULL;
		LIST_DEL(del_msg);
		free_sa_msg(del_msg);
        }
    }

    omgt_unlock_sem(&port->lock);

    return new_timeout_ms;
}

/* This function is only called after all registrations have been
   freed and the port thread has terminated. It is called to free
   all pending registration/unregistration messages */
void omgt_sa_remove_all_pending_reg_msgs(struct omgt_port *port)
{
	struct omgt_sa_msg *msg;
	struct omgt_sa_msg *del_msg;

	omgt_lock_sem(&port->lock);

	LIST_FOR_EACH(&port->pending_reg_msg_head, msg) {
		/* detach the msg to be deleted from the list first */
		del_msg = msg;
		msg = msg->prev;
		LIST_DEL(del_msg);
		free_sa_msg(del_msg);
	}

	omgt_unlock_sem(&port->lock);
}

static void process_wc(struct omgt_port *port, struct ibv_wc *wc)
{
    struct omgt_sa_msg *msg = (struct omgt_sa_msg *)wc->wr_id;
    int i;
    if (wc->opcode == IBV_WC_SEND) {
        OMGT_DBGPRINT(port, "Notice Send Completion %p : %s\n",
                    msg,
                    ibv_wc_status_str(wc->status));

        omgt_lock_sem(&port->lock);
        port->outstanding_sends_cnt--;
        msg->in_q = 0;
        omgt_unlock_sem(&port->lock);
        if (wc->status != IBV_WC_SUCCESS) {
			/* Thread will handle reposting */
            OMGT_OUTPUT_ERROR(port, "Notice Send Completion not success : %s... ",
                    ibv_wc_status_str(wc->status));
        }
    } else if (wc->opcode == IBV_WC_RECV) {
        OMGT_DBGPRINT(port, "Notice Recv Completion %p flags %d: %s\n", msg, wc->wc_flags, ibv_wc_status_str(wc->status));
        process_sa_rcv_msg(port, msg);
        if (ibv_post_recv(port->sa_qp, &msg->wr.recv, NULL)) {
            OMGT_OUTPUT_ERROR(port, "Failed to repost recv buffer - mark as unqueued\n");
            msg->in_q = 0;
        } else {
            msg->in_q = 1;
        }

        // If any receive buffers failed to repost cycle through the list here
        // and retry the post to keep the buffer queue full.
        for (i = 0; i < port->num_userspace_recv_buf; i++)
            if (!port->recv_bufs[i].in_q)
                if (0 == ibv_post_recv(port->sa_qp, &port->recv_bufs[i].wr.recv, NULL)) {
                    port->recv_bufs[i].in_q = 1;
                }
    } else {
        OMGT_OUTPUT_ERROR(port, "Unknown work completion event: 0x%x\n", wc->opcode);
    }
}

void handle_sa_ud_qp(struct omgt_port *port)
{
    struct ibv_wc wc;
    struct ibv_cq *ev_cq = port->sa_qp_cq;
    struct omgt_port *ev_port;

    if (ibv_get_cq_event(port->sa_qp_comp_channel, &ev_cq, (void **)&ev_port))
        goto request_notify;

    ibv_ack_cq_events(ev_cq, 1);

    if (port != ev_port || ev_cq != port->sa_qp_cq) {
        OMGT_OUTPUT_ERROR(port, "ibv_get_cq_event failed to get event for our notice/CQ\n");
        goto request_notify;
    }

    while (ibv_poll_cq(port->sa_qp_cq, 1, &wc) > 0) {
        process_wc(port, &wc);
    }

request_notify:
    if (ibv_req_notify_cq(port->sa_qp_cq, 0))
        OMGT_OUTPUT_ERROR(port, "ibv_req_notify_cq failed\n");
}

int start_ud_cq_monitor(struct omgt_port *port)
{
    int rc;
    struct omgt_thread_msg msg = {0};

    msg.size = sizeof(msg);
    msg.evt = OMGT_TH_EVT_UD_MONITOR_ON;

    rc = write(port->umad_port_sv[0], &msg, sizeof(msg));
    if (rc <= 0) {
        OMGT_OUTPUT_ERROR(port, "Failed to start CQ Monitoring...\n");
        return 1;
    }
    return 0;
}

int stop_ud_cq_monitor(struct omgt_port *port)
{
    int rc;
    struct omgt_thread_msg msg = {0};

    msg.size = sizeof(msg);
    msg.evt = OMGT_TH_EVT_UD_MONITOR_OFF;

    rc = write(port->umad_port_sv[0], &msg, sizeof(msg));
    if (rc <= 0) {
        OMGT_OUTPUT_ERROR(port, "Failed to stop CQ Monitoring...\n");
        return 1;
    }
    return 0;
}

int create_sa_qp(struct omgt_port *port)
{
    int i;
    int flags, rc;
    int buf_cnt;
    struct ibv_qp_init_attr init_attr = {0};
    struct ibv_qp_attr attr = {0};

    if (port->sa_qp) {
        return 0;
    }

    port->sa_qp_comp_channel = ibv_create_comp_channel(port->verbs_ctx);
    if (!port->sa_qp_comp_channel) {
        OMGT_OUTPUT_ERROR(port, "Notice: create comp_channel failed\n");
        return -EIO;
    }

    flags = fcntl(port->sa_qp_comp_channel->fd, F_GETFL);
    rc = fcntl(port->sa_qp_comp_channel->fd, F_SETFL, flags | O_NONBLOCK);
    if (rc < 0) {
        OMGT_OUTPUT_ERROR(port, "Notice: create QP failed\n");
        goto cq_fail;
    }

    port->recv_bufs = calloc(port->num_userspace_recv_buf,
                             sizeof *port->recv_bufs);
    if (!port->recv_bufs){
        OMGT_OUTPUT_ERROR(port, "Notice: recv message buffer allocation failed\n");
        goto cq_fail;        
    }

    buf_cnt = port->num_userspace_recv_buf + port->num_userspace_send_buf + 10;
    port->sa_qp_cq = ibv_create_cq(port->verbs_ctx, buf_cnt, (void *)port,
                                port->sa_qp_comp_channel, 0);
    if (!port->sa_qp_cq) {
        OMGT_OUTPUT_ERROR(port, "Notice: create QP failed\n");
        goto buf_fail;
    }

    if (ibv_req_notify_cq(port->sa_qp_cq, 0)) {
        OMGT_OUTPUT_ERROR(port, "Notice: req_notifiy_cq: failed\n");
        goto pd_fail;
    }

    port->sa_qp_pd = ibv_alloc_pd(port->verbs_ctx);
    if (!port->sa_qp_pd) {
        OMGT_OUTPUT_ERROR(port, "Notice: Alloc PD failed\n");
        goto pd_fail;
    }

    memset(&init_attr, 0, sizeof(init_attr));
    init_attr.qp_context = (void *)port;
    init_attr.send_cq = port->sa_qp_cq;
    init_attr.recv_cq = port->sa_qp_cq;
    init_attr.cap.max_send_wr = port->num_userspace_send_buf +1;
    init_attr.cap.max_recv_wr = port->num_userspace_recv_buf +1;
    init_attr.cap.max_send_sge = 1;
    init_attr.cap.max_recv_sge = 1;
    init_attr.qp_type = IBV_QPT_UD;
    init_attr.sq_sig_all = 1;

    port->sa_qp = ibv_create_qp(port->sa_qp_pd, &init_attr);
    if (!port->sa_qp) {
        OMGT_OUTPUT_ERROR(port, "Notice: create QP failed\n");
        goto qp_fail;
    }

    attr.qp_state = IBV_QPS_INIT;
    attr.port_num = port->hfi_port_num;
    attr.qkey = UMAD_QKEY;
    if ( 0xffff == (attr.pkey_index = omgt_find_pkey(port, 0xffff)) )
    	attr.pkey_index = omgt_find_pkey(port, 0x7fff);
    if (ibv_modify_qp(port->sa_qp, &attr, IBV_QP_STATE | IBV_QP_PKEY_INDEX |
                                          IBV_QP_PORT | IBV_QP_QKEY)) {
        OMGT_OUTPUT_ERROR(port, "Notice: failed to modify QP to init\n");
        goto destroy_qp;
    }

    attr.qp_state = IBV_QPS_RTR;
    if (ibv_modify_qp(port->sa_qp, &attr, IBV_QP_STATE)) {
        OMGT_OUTPUT_ERROR(port, "Notice: failed to modify QP to rtr\n");
        goto destroy_qp;
    }

    attr.qp_state = IBV_QPS_RTS;
    attr.sq_psn = 0;
    if (ibv_modify_qp(port->sa_qp, &attr, IBV_QP_STATE | IBV_QP_SQ_PSN)) {
        OMGT_OUTPUT_ERROR(port, "Notice: failed to modify QP to rts\n");
        goto destroy_qp;
    }

    if (omgt_post_notice_recvs(port)) {
        OMGT_OUTPUT_ERROR(port, "Notice: post recv buffers failed\n");
        goto destroy_qp;
    }

    if (start_ud_cq_monitor(port))
        goto unreg_recv;

    return (0);

unreg_recv:
    OMGT_DBGPRINT(port, "create_sa_qp: unreg_recv\n");
    for (i = 0; i<port->num_userspace_recv_buf; i++)
        ibv_dereg_mr(port->recv_bufs[i].mr);
destroy_qp:
    OMGT_DBGPRINT(port, "create_sa_qp: destroy_qp\n");
    ibv_destroy_qp(port->sa_qp);
	port->sa_qp = NULL;
qp_fail:
    OMGT_DBGPRINT(port, "create_sa_qp: qp_fail\n");
    ibv_dealloc_pd(port->sa_qp_pd);
pd_fail:
    OMGT_DBGPRINT(port, "create_sa_qp: pd_fail\n");
    ibv_destroy_cq(port->sa_qp_cq);
buf_fail:
    OMGT_DBGPRINT(port, "create_sa_qp: buf_fail\n");
    free(port->recv_bufs);
cq_fail:
    OMGT_DBGPRINT(port, "create_sa_qp: cq_fail\n");
    ibv_destroy_comp_channel(port->sa_qp_comp_channel);
    return (-EIO);
}

static int start_outstanding_req_timer(struct omgt_port *port)
{
    int rc;
    struct omgt_thread_msg msg = {0};

    msg.size = sizeof(msg);
    msg.evt = OMGT_TH_EVT_START_OUTSTANDING_REQ_TIME;

    rc = write(port->umad_port_sv[0], &msg, sizeof(msg));
    if (rc <= 0) {
        OMGT_OUTPUT_ERROR(port, "Failed to start outstanding request timer...\n");
        return 1;
    }
    return 0;
}

/**
 * port->lock must be held
 */
int userspace_register(struct omgt_port *port, uint16_t trap_num, omgt_sa_registration_t *reg)
{
    struct omgt_sa_msg *sa_msg;
    struct umad_sa_packet *sa_pkt;
    STL_INFORM_INFO *informinfo;

    sa_msg = alloc_send_sa_msg(port);
    if (!sa_msg)
        return (-EIO);

    memset(sa_msg->data, 0, sizeof(sa_msg->data));
    sa_pkt = (struct umad_sa_packet *)sa_msg->data;
    set_sa_common_stl_inform_info(port, sa_pkt);
    informinfo = (STL_INFORM_INFO *)sa_pkt->data;
    informinfo->Subscribe = 1;
    informinfo->u.Generic.TrapNumber = trap_num;
    informinfo->u.Generic.u1.s.RespTimeValue = 19;
    BSWAP_STL_INFORM_INFO(informinfo);

    LIST_ADD(&port->pending_reg_msg_head, sa_msg);

    reg->reg_msg = sa_msg;
    sa_msg->reg = reg;
    sa_msg->retries = NOTICE_REG_RETRY_COUNT;
    sa_msg->status = 0;
    post_send_sa_msg(port, sa_msg, OMGT_RRS_SEND_INITIAL);

    OMGT_DBGPRINT(port, "starting timer to register %d\n", trap_num);
    start_outstanding_req_timer(port);

    return (0);
}

/**
 * port->lock must be held
 */
static int userspace_unregister(struct omgt_port *port, omgt_sa_registration_t *reg)
{
    struct omgt_sa_msg *sa_msg;
    struct umad_sa_packet *sa_pkt;
    STL_INFORM_INFO *informinfo;
    uint16_t trap_num;

    if (reg->reg_msg) {
        LIST_DEL(reg->reg_msg);
        /* Registration never completed just free the oustanding mad */
        free_sa_msg(reg->reg_msg);
        return 0;
    }

    sa_msg = alloc_send_sa_msg(port);
    if (!sa_msg) {
        OMGT_OUTPUT_ERROR(port, "Notice: failed to allocate SA message\n");
        return (-EIO);
    }

    trap_num = reg->trap_num;
    memset(sa_msg->data, 0, sizeof(sa_msg->data));
    sa_pkt = (struct umad_sa_packet *)sa_msg->data;
    set_sa_common_stl_inform_info(port, sa_pkt);
    informinfo = (STL_INFORM_INFO *)sa_pkt->data;
    informinfo->Subscribe = 0;
    informinfo->u.Generic.TrapNumber = trap_num;
    informinfo->u.Generic.u1.s.RespTimeValue = 19;
    informinfo->u.Generic.u1.s.QPNumber = port->sa_qp->qp_num;
    BSWAP_STL_INFORM_INFO(informinfo);

    LIST_ADD(&port->pending_reg_msg_head, sa_msg);

    /* By the time the response comes back, the variable "reg" is already
       freed by omgt_sa_remove_gre_by_trap (the caller), and we should not
       set it here to avoid segfault in process_sa_gret_resp() */
    sa_msg->reg = NULL;
    sa_msg->retries = NOTICE_REG_RETRY_COUNT;
    sa_msg->status = 0;
    post_send_sa_msg(port, sa_msg, OMGT_RRS_SEND_INITIAL);

    OMGT_DBGPRINT(port, "starting timer to un-register %d\n", trap_num);
    start_outstanding_req_timer(port);

    return 0;
}

/**
 * Removes a registration based on its trap number.
 * NOTE: Caller must hold the lock.
 *
 * @param port      port opened by omgt_open_port_*
 * @param trap_num  The trap number to search for
 *
 * @return          0 if success, else error code
 */
FSTATUS omgt_sa_remove_reg_by_trap_unsafe(struct omgt_port *port, uint16_t trap_num)
{
	omgt_sa_registration_t *curr = port->regs_list, *prev = NULL;
	while (curr != NULL) {
		if (curr->trap_num == trap_num) {
			if (prev != NULL)
				prev->next = curr->next;
			else
				port->regs_list = curr->next;

            userspace_unregister(port, curr);
			free(curr);
			return FSUCCESS;
		}
		prev = curr;
		curr = curr->next;
	}

	return FERROR;
}

/**
 * Clear all registrations from the port.
 */
void omgt_sa_clear_regs_unsafe(struct omgt_port *port)
{
    FSTATUS status = FTIMEOUT;
    if (omgt_lock_sem(&port->lock)) {
        OMGT_OUTPUT_ERROR(port, "failed to acquire lock (status: %d)\n", status);
        return;
    }

	while (port->regs_list != NULL) {
		/* The called function takes care of adjusting the head pointer
		   and freeing the entry*/
        omgt_sa_remove_reg_by_trap_unsafe(port, port->regs_list->trap_num);
	}

    omgt_unlock_sem(&port->lock);
}

/**
 * Re-register all current registrations for the port.
 */
int reregister_traps(struct omgt_port *port)
{
    int ret;
    int status = -1;
    if (omgt_lock_sem(&port->lock)) {
        OMGT_OUTPUT_ERROR(port, "failed to acquire lock (status: %d)\n", status);
        return status;
    }

    omgt_sa_registration_t *curr = port->regs_list;
    while (curr != NULL) {
        if ((ret = userspace_register(port, curr->trap_num, curr)) != 0) {
            OMGT_OUTPUT_ERROR(port, "omgt_sa_reregister_trap_regs: failed to register for trap (%u) (status: %d)\n",
                         curr->trap_num, ret);
        }
        curr = curr->next;
    }

    omgt_unlock_sem(&port->lock);
    return 0;
}
