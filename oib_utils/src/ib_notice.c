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

 * ** END_ICS_COPYRIGHT5   ****************************************/
/* [ICS VERSION STRING: unknown] */

/*
 * OFED "User SA" support.
 *
 * This module provides userspace access to the "User SA" components, which
 * exposes SA Notice multiplexing functionality to userspace.
 *
 * Note that this implementation follows the oib_utils model using the oib_port
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

#define OIB_UTILS_PRIVATE 1

#include "ib_utils_openib.h"
#include "ib_notice_net.h"
#include "oib_utils_sa.h"
#include "iba/public/ibyteswap.h"

#define OIB_SA_MAX_REGISTRANTS 10

#define OIB_SA_DEVICE_NAME_LEN_MAX 32

#define OIB_SA_DEFAULT_LOCK_TIMEOUT 5000000ull // 5 seconds

#ifndef container_of
#define container_of(ptr, type, field) \
    ((type *) ((void *) ptr - offsetof(type, field)))
#endif

struct oib_sa_event {
    struct ibv_sa_event event;
    struct ibv_sa_event_channel *channel;
    void *data;
};

int oib_lock_sem(SEMAPHORE* const	pSema)
{
	int rc = EIO;
	unsigned int ms = (OIB_SA_DEFAULT_LOCK_TIMEOUT + 999)/1000;
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

void oib_unlock_sem(SEMAPHORE * const pSema)
{
    sem_post(pSema);
}

/**
 * port->lock must be held
 */
static int reg_sa_msg_mr(struct oib_port *port, struct oib_sa_msg *msg,
                          enum ibv_wr_opcode opcode,
                          uint32_t rem_qpn, uint32_t rem_qkey)
{
    msg->mr = ibv_reg_mr(port->sa_qp_pd, msg->data, sizeof(msg->data),
                        IBV_ACCESS_LOCAL_WRITE);
    if (!msg->mr) {
        OUTPUT_ERROR("Notice: sa msg register memory region failed\n");
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
static struct oib_sa_msg * alloc_send_sa_msg(struct oib_port *port)
{
    struct oib_sa_msg *msg;

    if (!port->sa_ah) {
		int err = 0;
        struct ibv_ah_attr attr;
        memset(&attr, 0, sizeof(attr));

		if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
			OUTPUT_ERROR("failed to acquire lock (err: %d)\n", err);
			return (NULL);
		}
        attr.dlid = port->umad_port_cache.sm_lid;
        attr.sl = port->umad_port_cache.sm_sl;
		oib_unlock_sem(&port->umad_port_cache_lock);

        attr.port_num = port->hfi_port_num;
        port->sa_ah = ibv_create_ah(port->sa_qp_pd, &attr);
        if (!port->sa_ah) {
			OUTPUT_ERROR("failed to create SA AH (err: %d)\n", errno);
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
static void free_sa_msg(struct oib_sa_msg *msg)
{
	if (msg->mr)
        ibv_dereg_mr(msg->mr);
    free(msg);
}


/**
 * Adds a registration to the list.
 * NOTE: Caller must hold the lock.
 *
 * @param port   port opened by oib_open_port_ 
 * @param reg    Pointer to the registration structure. 
 *  
 * @return       none 
 */
static void oib_sa_add_reg_unsafe(struct oib_port *port, oib_sa_registration_t *reg)
{
	reg->next = port->regs_list;
	port->regs_list = reg;
}

static void set_sa_msg_tid(struct oib_port *port, struct umad_sa_packet *sa_pkt)
{
    port->next_tid++;
    if (port->next_tid == 0)
        port->next_tid++;
    sa_pkt->mad_hdr.tid = htonll((uint64_t)port->next_tid);
}

static void set_sa_common_inform_info(struct oib_port *port, struct umad_sa_packet *sa_pkt)
{
    struct ibv_sa_net_inform_info *ii;

    sa_pkt->mad_hdr.base_version = UMAD_BASE_VERSION;
    sa_pkt->mad_hdr.mgmt_class = UMAD_CLASS_SUBN_ADM;
    sa_pkt->mad_hdr.class_version = UMAD_SA_CLASS_VERSION;
    sa_pkt->mad_hdr.method = UMAD_METHOD_SET;
    sa_pkt->mad_hdr.attr_id = htons(UMAD_ATTR_INFORM_INFO);

    sa_pkt->rmpp_hdr.rmpp_version = UMAD_RMPP_VERSION;
    sa_pkt->rmpp_hdr.rmpp_type = 0;

    ii = (struct ibv_sa_net_inform_info *)sa_pkt->data;

    ii->lid_range_begin = htons(0xffff);
    ii->is_generic = 1;
    ii->type = htons(0xffff);
    ii->producer_type_vendor_id = htonl(0xffffff);
}

static void set_sa_common_response_notice(struct oib_port *port, struct umad_sa_packet *sa_pkt)
{
    struct ibv_sa_net_notice *nn;

    sa_pkt->mad_hdr.base_version = UMAD_BASE_VERSION;
    sa_pkt->mad_hdr.mgmt_class = UMAD_CLASS_SUBN_ADM;
    sa_pkt->mad_hdr.class_version = UMAD_SA_CLASS_VERSION;
    sa_pkt->mad_hdr.method = UMAD_METHOD_REPORT_RESP;
    sa_pkt->mad_hdr.attr_id = htons(UMAD_ATTR_NOTICE);

    sa_pkt->rmpp_hdr.rmpp_version = UMAD_RMPP_VERSION;
    sa_pkt->rmpp_hdr.rmpp_type = 0;

    nn = (struct ibv_sa_net_notice *)sa_pkt->data;

    // if the Type is set to 0x7f (empty) all other fields are unused
    nn->generic_type_producer = htonl(0x7f000000);
}

static void post_send_sa_msg(struct oib_port *port,
                             struct oib_sa_msg *msg,
                             enum oib_reg_retry_state resend)
{
    int rc;
    struct ibv_send_wr *bad_wr;

    if (msg->in_q) {
        OUTPUT_ERROR("msg (%p) is already in the send Q!!!\n", msg);
        return;
    }

    if (!msg->retries) {
        OUTPUT_ERROR("msg (%p) has timed out!!!\n", msg);
        return;
    }

    if (port->outstanding_sends_cnt >= port->num_userspace_send_buf) {
        OUTPUT_ERROR("no send buffers\n");
        return;
    }

    if (OIB_RRS_SEND_RETRY == resend) {
    	msg->retries--;
    	if (!msg->retries) {
    		DBGPRINT("Timeout sending SA msg.\n");
    		return;
    	}
    }
    set_sa_msg_tid(port, (struct umad_sa_packet *)msg->data);
    if ((rc = ibv_post_send(port->sa_qp, &(msg->wr.send), &bad_wr)) == 0) {
        port->outstanding_sends_cnt++;
        msg->in_q = 1;
    } else {
        OUTPUT_ERROR("Notice: post send WR failed: %s: Aborting send.\n",
                     strerror(rc));
    }
}

static int oib_post_notice_recvs(struct oib_port *port)
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

static struct oib_sa_msg *
find_req_by_tid(struct oib_port *port, uint32_t tid)
{
    struct oib_sa_msg *rc = NULL;
    struct oib_sa_msg *msg;

    DBGPRINT("find req tid 0x%x\n", tid);

    LIST_FOR_EACH(&port->pending_reg_msg_head, msg) {
        struct umad_sa_packet *sa_pkt = (struct umad_sa_packet *)msg->data;
        uint32_t mtid = ntohll(sa_pkt->mad_hdr.tid) & 0xffffffff;
        DBGPRINT("found tid 0x%x\n", mtid);
        if (mtid == tid) {
            rc = msg;
            break;
        }
    }

    return rc;
}

static void process_sa_get_resp(struct oib_port *port, struct umad_sa_packet *sa_pkt)
{
    struct oib_sa_msg *req;
    struct ibv_sa_net_inform_info *ii = (struct ibv_sa_net_inform_info *)sa_pkt->data;
    uint16_t trap_num = ntohs(ii->trap_num_device_id);

    oib_lock_sem(&port->lock);

    /* find the registration for this response */
    req = find_req_by_tid(port, ntohll(sa_pkt->mad_hdr.tid) & 0xffffffff);
    if (req) {
        if (ii->subscribe == 1) {
            DBGPRINT("registration complete for trap %d; req %p\n", trap_num, req);
        } else {
            DBGPRINT("UN-registration complete for trap %d; req %p\n", trap_num, req);
        }
	/* Check if the registration has been freed */
	if (req->reg) 
		req->reg->reg_msg = NULL;
        LIST_DEL(req);
        free_sa_msg(req);
    } else {
        OUTPUT_ERROR("Unknown get response; 'trap num' %d\n", trap_num);
    }
    oib_unlock_sem(&port->lock);
}

/* Recv'ed messages will a GRH on them or at least space (40 bytes) */
/* for a GRH which we need to remove.                               */
static struct umad_sa_packet *
sa_pkt_from_recv_msg(struct oib_sa_msg *msg)
{
    return (struct umad_sa_packet *)&msg->data[sizeof(struct ibv_grh)];
}

static void process_sa_report(struct oib_port *port, struct umad_sa_packet *sa_pkt)
{
    struct oib_sa_msg *response_msg;
    struct umad_sa_packet *response_pkt;
    struct ibv_send_wr *bad_wr;

    struct ibv_sa_net_notice *nn = (struct ibv_sa_net_notice *)sa_pkt->data;
    struct ibv_sa_net_notice_data_gid *nngd = (struct ibv_sa_net_notice_data_gid *)&nn->data_details[0];
    struct oib_thread_msg thread_msg;
    struct iovec iov[2];
    size_t write_count, write_size;
    uint16_t trap_num = ntohs(nn->trap_num_device_id);

    // create and send ReportResp to trap notify
    if (oib_lock_sem(&port->lock)) {
        OUTPUT_ERROR("failed to acquire lock (status: %d)\n", FTIMEOUT);
        return;
    }

    response_msg = alloc_send_sa_msg(port);
    if (response_msg)
    {
        int rc;
        memset(response_msg->data, 0, sizeof(response_msg->data));
        response_pkt = (struct umad_sa_packet *)response_msg->data;
        set_sa_common_response_notice(port, response_pkt);
        response_pkt->mad_hdr.tid = sa_pkt->mad_hdr.tid;

        if ((rc = ibv_post_send(port->sa_qp, &(response_msg->wr.send), &bad_wr)) != 0) {
            OUTPUT_ERROR("Notice: post send WR failed: %s: Aborting send.\n",
                         strerror(rc));
        }

        free_sa_msg(response_msg);
    }
    oib_unlock_sem(&port->lock);

    thread_msg.size = sizeof *nn;
    thread_msg.evt  = OIB_TH_EVT_TRAP_MSG;

	iov[0].iov_base = &thread_msg;
	iov[0].iov_len  = sizeof thread_msg;
	iov[1].iov_base = nn;
	iov[1].iov_len  = sizeof *nn;
	write_size = iov[0].iov_len + iov[1].iov_len;

    if ( write_size !=
        (write_count = writev(port->umad_port_sv[1], iov, 2)) )
         OUTPUT_ERROR("bad write count %d\n", (int)write_count);

    DBGPRINT("process_sa_report: msg queued - trap %d gid %02x%02x%02x%02x%02x%02x%02x%02x\n",
     trap_num, nngd->gid[8],nngd->gid[9],nngd->gid[10],nngd->gid[11],nngd->gid[12],nngd->gid[13],nngd->gid[14],nngd->gid[15]);
}

static void process_sa_rcv_msg(struct oib_port *port, struct oib_sa_msg *msg)
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
            OUTPUT_ERROR("unknown 'message' received : method 0x%x\n", sa_pkt->mad_hdr.method);
            break;
    }
}

int repost_pending_registrations(struct oib_port *port)
{
    int new_timeout_ms = -1;
    struct oib_sa_msg *msg;
    struct oib_sa_msg *del_msg;
    struct oib_thread_msg thread_msg;
    oib_sa_registration_t *reg;
    struct iovec iov[2];
    size_t write_size, write_count;

    oib_lock_sem(&port->lock);

    LIST_FOR_EACH(&port->pending_reg_msg_head, msg) {
    	if (msg->retries) {
	        new_timeout_ms = NOTICE_REG_TIMEOUT_MS;
	        post_send_sa_msg(port, msg, OIB_RRS_SEND_RETRY);    		
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
			thread_msg.evt  = OIB_TH_EVT_TRAP_REG_ERR_TIMEOUT;

			iov[0].iov_base = &thread_msg;
			iov[0].iov_len  = sizeof thread_msg;
			iov[1].iov_base = reg;
			iov[1].iov_len  = sizeof *reg;
			write_size = iov[0].iov_len + iov[1].iov_len;

			write_count = writev(port->umad_port_sv[1], iov, 2);
			if ( write_size != write_count)
				OUTPUT_ERROR("bad write count %d\n",
					     (int)write_count);
		}

		// detach the msg to be deleted from the list first
		del_msg = msg;
		msg = msg->prev;
		if (del_msg->reg) {
			DBGPRINT("registration timeout on trap %d : req %p\n",
			 	del_msg->reg->trap_num, del_msg->reg);
		} else {
			DBGPRINT("registration timeout on trap: No information available.\n");
		}
		if (del_msg->reg) 
			del_msg->reg->reg_msg = NULL;
		LIST_DEL(del_msg);
		free_sa_msg(del_msg);
        }
    }

    oib_unlock_sem(&port->lock);

    return new_timeout_ms;
}

/* This function is only called after all registrations have been
   freed and the port thread has terminated. It is called to free
   all pending registration/unregistration messages */
void oib_sa_remove_all_pending_reg_msgs(struct oib_port *port)
{
	struct oib_sa_msg *msg;
	struct oib_sa_msg *del_msg;

	oib_lock_sem(&port->lock);

	LIST_FOR_EACH(&port->pending_reg_msg_head, msg) {
		/* detach the msg to be deleted from the list first */
		del_msg = msg;
		msg = msg->prev;
		LIST_DEL(del_msg);
		free_sa_msg(del_msg);
	}

	oib_unlock_sem(&port->lock);
}

static void process_wc(struct oib_port *port, struct ibv_wc *wc)
{
    struct oib_sa_msg *msg = (struct oib_sa_msg *)wc->wr_id;
    int i;
    if (wc->opcode == IBV_WC_SEND) {
        DBGPRINT("Notice Send Completion %p : %s\n",
                    msg,
                    ibv_wc_status_str(wc->status));

        oib_lock_sem(&port->lock);
        port->outstanding_sends_cnt--;
        msg->in_q = 0;
        oib_unlock_sem(&port->lock);
        if (wc->status != IBV_WC_SUCCESS) {
			/* Thread will handle reposting */
            OUTPUT_ERROR("Notice Send Completion not success : %s... ",
                    ibv_wc_status_str(wc->status));
        }
    } else if (wc->opcode == IBV_WC_RECV) {
        DBGPRINT("Notice Recv Completion %p flags %d: %s\n", msg, wc->wc_flags, ibv_wc_status_str(wc->status));
        process_sa_rcv_msg(port, msg);
        if (ibv_post_recv(port->sa_qp, &msg->wr.recv, NULL)) {
            OUTPUT_ERROR("Failed to repost recv buffer - mark as unqueued\n");
            msg->in_q = 0;
        } else {
            msg->in_q = 1;
        }

        // If any receive buffers failed to repost cycle through the list here
        // and retry the post to keep the buffer queue full.
        for (i = 0; i < port->num_userspace_recv_buf; i++)
            if (!&port->recv_bufs[i].in_q)
                if (0 == ibv_post_recv(port->sa_qp, &port->recv_bufs[i].wr.recv, NULL)) {
                    port->recv_bufs[i].in_q = 1;
                }
    } else {
        OUTPUT_ERROR("Unknown work completion event: 0x%x\n", wc->opcode);
    }
}

void handle_sa_ud_qp(struct oib_port *port)
{
    struct ibv_wc wc;
    struct ibv_cq *ev_cq = port->sa_qp_cq;
    struct oib_port *ev_port;

    if (ibv_get_cq_event(port->sa_qp_comp_channel, &ev_cq, (void **)&ev_port))
        goto request_notify;

    ibv_ack_cq_events(ev_cq, 1);

    if (port != ev_port || ev_cq != port->sa_qp_cq) {
        OUTPUT_ERROR("ibv_get_cq_event failed to get event for our notice/CQ\n");
        goto request_notify;
    }

    while (ibv_poll_cq(port->sa_qp_cq, 1, &wc) > 0) {
        process_wc(port, &wc);
    }

request_notify:
    if (ibv_req_notify_cq(port->sa_qp_cq, 0))
        OUTPUT_ERROR("ibv_req_notify_cq failed\n");
}

int start_ud_cq_monitor(struct oib_port *port)
{
    int rc;
    struct oib_thread_msg msg;

    msg.size = sizeof(msg);
    msg.evt = OIB_TH_EVT_UD_MONITOR_ON;

    rc = write(port->umad_port_sv[0], &msg, sizeof(msg));
    if (rc <= 0) {
        OUTPUT_ERROR("Failed to start CQ Monitoring...\n");
        return 1;
    }
    return 0;
}

int stop_ud_cq_monitor(struct oib_port *port)
{
    int rc;
    struct oib_thread_msg msg;

    msg.size = sizeof(msg);
    msg.evt = OIB_TH_EVT_UD_MONITOR_OFF;

    rc = write(port->umad_port_sv[0], &msg, sizeof(msg));
    if (rc <= 0) {
        OUTPUT_ERROR("Failed to stop CQ Monitoring...\n");
        return 1;
    }
    return 0;
}

static int create_sa_qp(struct oib_port *port)
{
    int i;
    int flags, rc;
    int buf_cnt;
    struct ibv_qp_init_attr init_attr;
    struct ibv_qp_attr attr;

    if (port->sa_qp) {
        return 0;
    }

    port->sa_qp_comp_channel = ibv_create_comp_channel(port->verbs_ctx);
    if (!port->sa_qp_comp_channel) {
        OUTPUT_ERROR("Notice: create comp_channel failed\n");
        return -EIO;
    }

    flags = fcntl(port->sa_qp_comp_channel->fd, F_GETFL);
    rc = fcntl(port->sa_qp_comp_channel->fd, F_SETFL, flags | O_NONBLOCK);
    if (rc < 0) {
        OUTPUT_ERROR("Notice: create QP failed\n");
        goto cq_fail;
    }

    port->recv_bufs = calloc(port->num_userspace_recv_buf,
                             sizeof *port->recv_bufs);
    if (!port->recv_bufs){
        OUTPUT_ERROR("Notice: recv message buffer allocation failed\n");
        goto cq_fail;        
    }

    buf_cnt = port->num_userspace_recv_buf + port->num_userspace_send_buf + 10;
    port->sa_qp_cq = ibv_create_cq(port->verbs_ctx, buf_cnt, (void *)port,
                                port->sa_qp_comp_channel, 0);
    if (!port->sa_qp_cq) {
        OUTPUT_ERROR("Notice: create QP failed\n");
        goto buf_fail;
    }

    if (ibv_req_notify_cq(port->sa_qp_cq, 0)) {
        OUTPUT_ERROR("Notice: req_notifiy_cq: failed\n");
        goto pd_fail;
    }

    port->sa_qp_pd = ibv_alloc_pd(port->verbs_ctx);
    if (!port->sa_qp_pd) {
        OUTPUT_ERROR("Notice: Alloc PD failed\n");
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
        OUTPUT_ERROR("Notice: create QP failed\n");
        goto qp_fail;
    }

    attr.qp_state = IBV_QPS_INIT;
    attr.port_num = port->hfi_port_num;
    attr.qkey = UMAD_QKEY;
    if ( 0xffff == (attr.pkey_index = oib_find_pkey(port, 0xffff)) )
    	attr.pkey_index = oib_find_pkey(port, 0x7fff);
    if (ibv_modify_qp(port->sa_qp, &attr, IBV_QP_STATE | IBV_QP_PKEY_INDEX |
                                          IBV_QP_PORT | IBV_QP_QKEY)) {
        OUTPUT_ERROR("Notice: failed to modify QP to init\n");
        goto destroy_qp;
    }

    attr.qp_state = IBV_QPS_RTR;
    if (ibv_modify_qp(port->sa_qp, &attr, IBV_QP_STATE)) {
        OUTPUT_ERROR("Notice: failed to modify QP to rtr\n");
        goto destroy_qp;
    }

    attr.qp_state = IBV_QPS_RTS;
    attr.sq_psn = 0;
    if (ibv_modify_qp(port->sa_qp, &attr, IBV_QP_STATE | IBV_QP_SQ_PSN)) {
        OUTPUT_ERROR("Notice: failed to modify QP to rts\n");
        goto destroy_qp;
    }

    if (oib_post_notice_recvs(port)) {
        OUTPUT_ERROR("Notice: post recv buffers failed\n");
        goto destroy_qp;
    }

    if (start_ud_cq_monitor(port))
        goto unreg_recv;

    return (0);

unreg_recv:
    DBGPRINT("create_sa_qp: unreg_recv\n");
    for (i = 0; i<port->num_userspace_recv_buf; i++)
        ibv_dereg_mr(port->recv_bufs[i].mr);
destroy_qp:
    DBGPRINT("create_sa_qp: destroy_qp\n");
    ibv_destroy_qp(port->sa_qp);
	port->sa_qp = NULL;
qp_fail:
    DBGPRINT("create_sa_qp: qp_fail\n");
    ibv_dealloc_pd(port->sa_qp_pd);
pd_fail:
    DBGPRINT("create_sa_qp: pd_fail\n");
    ibv_destroy_cq(port->sa_qp_cq);
buf_fail:
    DBGPRINT("create_sa_qp: buf_fail\n");
    free(port->recv_bufs);
cq_fail:
    DBGPRINT("create_sa_qp: cq_fail\n");
    ibv_destroy_comp_channel(port->sa_qp_comp_channel);
    return (-EIO);
}

static int start_outstanding_req_timer(struct oib_port *port)
{
    int rc;
    struct oib_thread_msg msg;

    msg.size = sizeof(msg);
    msg.evt = OIB_TH_EVT_START_OUTSTANDING_REQ_TIME;

    rc = write(port->umad_port_sv[0], &msg, sizeof(msg));
    if (rc <= 0) {
        OUTPUT_ERROR("Failed to start outstanding request timer...\n");
        return 1;
    }
    return 0;
}

/**
 * port->lock must be held
 */
static int userspace_register(struct oib_port *port, uint16_t trap_num,
							oib_sa_registration_t *reg)
{
    struct oib_sa_msg *sa_msg;
    struct umad_sa_packet *sa_pkt;
    struct ibv_sa_net_inform_info *ii;

    sa_msg = alloc_send_sa_msg(port);
    if (!sa_msg)
        return (-EIO);

    memset(sa_msg->data, 0, sizeof(sa_msg->data));
    sa_pkt = (struct umad_sa_packet *)sa_msg->data;
    set_sa_common_inform_info(port, sa_pkt);
    ii = (struct ibv_sa_net_inform_info *)sa_pkt->data;
    ii->subscribe = 1;
    ii->trap_num_device_id = htons(trap_num);
    ii->qpn_resptime = htonl(19);

    LIST_ADD(&port->pending_reg_msg_head, sa_msg);

    reg->reg_msg = sa_msg;
    sa_msg->reg = reg;
    sa_msg->retries = NOTICE_REG_RETRY_COUNT;
    sa_msg->status = 0;
    post_send_sa_msg(port, sa_msg, OIB_RRS_SEND_INITIAL);

    DBGPRINT("starting timer to register %d\n", trap_num);
    start_outstanding_req_timer(port);

    return (0);
}

/**
 * port->lock must be held
 */
static int userspace_unregister(struct oib_port *port, oib_sa_registration_t *reg)
{
    struct oib_sa_msg *sa_msg;
    struct umad_sa_packet *sa_pkt;
    struct ibv_sa_net_inform_info *ii;
    uint16_t trap_num;

    if (reg->reg_msg) {
        LIST_DEL(reg->reg_msg);
        /* Registration never completed just free the oustanding mad */
        free_sa_msg(reg->reg_msg);
        return 0;
    }

    sa_msg = alloc_send_sa_msg(port);
    if (!sa_msg) {
        OUTPUT_ERROR("Notice: failed to allocate SA message\n");
        return (-EIO);
    }

    trap_num = reg->trap_num;
    memset(sa_msg->data, 0, sizeof(sa_msg->data));
    sa_pkt = (struct umad_sa_packet *)sa_msg->data;
    set_sa_common_inform_info(port, sa_pkt);
    ii = (struct ibv_sa_net_inform_info *)sa_pkt->data;
    ii->subscribe = 0;
    ii->trap_num_device_id = htons(trap_num);
    ii->qpn_resptime = htonl((port->sa_qp->qp_num << 8) | 19);

    LIST_ADD(&port->pending_reg_msg_head, sa_msg);

    /* By the time the response comes back, the variable "reg" is already
       freed by oib_sa_remove_gre_by_trap (the caller), and we should not
       set it here to avoid segfault in process_sa_gret_resp() */
    sa_msg->reg = NULL;
    sa_msg->retries = NOTICE_REG_RETRY_COUNT;
    sa_msg->status = 0;
    post_send_sa_msg(port, sa_msg, OIB_RRS_SEND_INITIAL);

    DBGPRINT("starting timer to un-register %d\n", trap_num);
    start_outstanding_req_timer(port);

    return 0;
}

/**
 * Removes a registration based on its trap number.
 * NOTE: Caller must hold the lock.
 *
 * @param port      port opened by oib_open_port_*
 * @param trap_num  The trap number to search for
 *
 * @return          0 if success, else error code
 */
static FSTATUS oib_sa_remove_reg_by_trap_unsafe(struct oib_port *port, uint16_t trap_num)
{
	oib_sa_registration_t *curr = port->regs_list, *prev = NULL;
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
 * Get the buffer counts for QP creation.
 */
FSTATUS oib_get_sa_buf_cnt(struct oib_port *port, int *send_cnt, int *recv_cnt)
{
    FSTATUS status = FTIMEOUT;
    if (oib_lock_sem(&port->lock)) {
        OUTPUT_ERROR("failed to acquire lock (status: %d)\n", status);
        return status;
    }
    status = FSUCCESS;

    if (send_cnt)
        *send_cnt = port->num_userspace_send_buf;
    if (recv_cnt)
        *recv_cnt = port->num_userspace_recv_buf;

    oib_unlock_sem(&port->lock);

    return status;
}

/**
 * Set the buffer counts for QP creation.
 */
FSTATUS oib_set_sa_buf_cnt(struct oib_port *port, int send_cnt, int recv_cnt)
{
    FSTATUS status = FTIMEOUT;
    if (oib_lock_sem(&port->lock)) {
        OUTPUT_ERROR("failed to acquire lock (status: %d)\n", status);
        return status;
    }
    status = FSUCCESS;

    if (port->sa_qp){
        OUTPUT_ERROR("Cannot modify port buffer counts once a trap is registered.\n");
        status = FERROR;
        goto buf_cnt_too_late;
    }

    if (send_cnt)
        port->num_userspace_send_buf = send_cnt;
    if (recv_cnt)
        port->num_userspace_recv_buf = recv_cnt;

buf_cnt_too_late:
    oib_unlock_sem(&port->lock);

    return status;
}

/**
 * Clear all registrations from the port.
 */
void oib_sa_clear_regs_unsafe(struct oib_port *port)
{
    FSTATUS status = FTIMEOUT;
    if (oib_lock_sem(&port->lock)) {
        OUTPUT_ERROR("failed to acquire lock (status: %d)\n", status);
        return;
    }

	while (port->regs_list != NULL) {
		/* The called function takes care of adjusting the head pointer
		   and freeing the entry*/
        oib_sa_remove_reg_by_trap_unsafe(port, port->regs_list->trap_num);
	}

    oib_unlock_sem(&port->lock);
}

/**
 * Re-register all current registrations for the port.
 */
int reregister_traps(struct oib_port *port)
{
    int ret;
    int status = -1;
    if (oib_lock_sem(&port->lock)) {
        OUTPUT_ERROR("failed to acquire lock (status: %d)\n", status);
        return status;
    }

    oib_sa_registration_t *curr = port->regs_list;
    while (curr != NULL) {
        if ((ret = userspace_register(port, curr->trap_num, curr)) != 0) {
            OUTPUT_ERROR("oib_sa_reregister_trap_regs: failed to register for trap (%u) (status: %d)\n",
                         curr->trap_num, ret);
        }
        curr = curr->next;
    }

    oib_unlock_sem(&port->lock);
    return 0;
}

/**
 * Find a previous registration
 *
 * @return          0 if success, else error code
 */
static oib_sa_registration_t *oib_sa_find_reg(struct oib_port *port, uint16_t trap_num)
{
    oib_sa_registration_t *curr = port->regs_list;
    while (curr != NULL) {
        if (curr->trap_num == trap_num) {
            return curr;
        }
        curr = curr->next;
    }
    return NULL;
}

/**
 * Initiates a registration for the specified trap.
 *  
 * @param port      port opened by oib_open_port_*
 * @param trap_num  The trap number to register for 
 * @param port_num  port number requesting trap 
 * @param context   optional opaque info 
 *  
 * @return   0 if success, else error code
 */
FSTATUS oib_sa_register_trap(struct oib_port *port, 
							 uint16_t trap_num,
							 void *context)
{
	FSTATUS status;
	int ret;
	oib_sa_registration_t *reg;

    reg = (oib_sa_registration_t *)calloc(1, sizeof *reg);
	if (reg == NULL) {
		OUTPUT_ERROR("failed to allocate reg structure\n");
		return FERROR;
	}

	status = FTIMEOUT;
    if (oib_lock_sem(&port->lock)) {
		OUTPUT_ERROR("failed to acquire lock (status: %d)\n", status);
		free(reg);
        return (FERROR);
    }

    if (oib_sa_find_reg(port, trap_num)) {
        oib_unlock_sem(&port->lock);
        free(reg);
        return FSUCCESS;
    }

    if ((ret = create_sa_qp(port)) != 0) {
        oib_unlock_sem(&port->lock);
        OUTPUT_ERROR("failed to create notice QP for trap (%u) registration (status: %d)\n",
                     trap_num, ret);
    }
    else if ((ret = userspace_register(port, trap_num, reg)) != 0) {
        oib_unlock_sem(&port->lock);
        OUTPUT_ERROR("failed to register for trap (%u) (status: %d)\n",
                     trap_num, ret);
    }

    if (ret) {
        free(reg);
        return FERROR;
    }
	
    reg->user_context = context;
	reg->trap_num = trap_num;
	oib_sa_add_reg_unsafe(port, reg);
	
    oib_unlock_sem(&port->lock);
	return FSUCCESS;
}

/**
 * Unregisters for the specified trap.
 *  
 * @param port      port opened by oib_open_port_*
 * @param trap_num  The trap number to unregister 
 *  
 * @return 
 */
FSTATUS oib_sa_unregister_trap(struct oib_port *port, uint16_t trap_num)
{
	FSTATUS status;
	status = FTIMEOUT;

    if (oib_lock_sem(&port->lock))
        return (FERROR);
	
	status = oib_sa_remove_reg_by_trap_unsafe(port, trap_num);
	
	oib_unlock_sem(&port->lock);
	return status;
}

/**
 * Fetches the next available notice.  Blocks in kernel (interrupted on
 * process signal).
 *
 * @param port      port opened by oib_open_port_*
 * @param target_buf     Pointer to buffer to place notice into
 * @param buf_size       Size of the target buffer
 * @param bytes_written  OUTPUT: Set to the number of bytes written
 *  					 written into the buffer
 *
 * @return   0 if success, else error code
 */
FSTATUS oib_sa_get_event(struct oib_port *port, void *target_buf,
						 size_t buf_size, int *bytes_written)
{
    int rc;
    int s;
    struct pollfd pollfd[1];
    struct oib_thread_msg *msg;
    struct ibv_sa_net_notice *nn;
    uint8_t rdbuf[2048];

    pollfd[0].fd = port->umad_port_sv[0];
    pollfd[0].events = POLLIN;
    pollfd[0].revents = 0;

    if ((rc = poll(pollfd, 1, -1)) < 0) {
        OUTPUT_ERROR ("trap poll failed : %s\n", strerror(errno));
        return FERROR;
    }
    else if (pollfd[0].revents & POLLIN) {
        s = read(port->umad_port_sv[0], rdbuf, sizeof rdbuf);
        if (s <= 0) {
            OUTPUT_ERROR ("user event read failed : %s\n", strerror(errno));
            return FERROR;
        }

        msg = (struct oib_thread_msg *)rdbuf;
        if (OIB_TH_EVT_TRAP_MSG == msg->evt) {
	        if ( msg->size > buf_size)
	            msg->size = buf_size;

	        memcpy(target_buf, (rdbuf + sizeof *msg), msg->size);
	        *bytes_written = msg->size;

	        nn = (struct ibv_sa_net_notice *)target_buf;
	        DBGPRINT("trap message %u: %d bytes\n", ntohs(nn->trap_num_device_id), (int)(msg->size));        	
        }
        else if (OIB_TH_EVT_TRAP_REG_ERR_TIMEOUT == msg->evt) {
	        if ( msg->size > buf_size)
	            msg->size = buf_size;

	        memcpy(target_buf, (rdbuf + sizeof *msg), msg->size);
	        *bytes_written = msg->size;

	        DBGPRINT("registration of trap message timed out\n");
	        return FTIMEOUT;
        }
    	else {
            OUTPUT_ERROR ("user event read invalid message.\n");
            return FERROR;
        }
    }
    else {
        DBGPRINT("trap poll unexpected result : %d\n", pollfd[0].revents);
        return FERROR;
    }

    return FSUCCESS;
}

/**
 * Fetches the next available notice.  Blocks (interrupted on
 * process signal).
 *
 * @param port       port opened by oib_open_port_*
 * @param event      Pointer to structure to place event notice into
 *
 * @return   0 if success, else error code
 */
FSTATUS oib_get_sa_event(struct oib_port *port, struct ibv_sa_event **event)
{
    struct ibv_sa_event_channel *channel;
    struct oib_sa_event *evt;
    uint8_t target_buf[2048];
    int bytes_written;
    oib_sa_registration_t *reg;
    struct ibv_sa_net_notice *nn;
    uint16_t trap_num;
    FSTATUS ret;

    channel = port->channel;

    evt = (struct oib_sa_event *)calloc(1, sizeof *evt);
    if (!evt)
        return ENOMEM;
    else
        *event = &evt->event;

    ret = oib_sa_get_event(port, target_buf,
                           sizeof target_buf, &bytes_written);
    if ( FSUCCESS == ret) {
	    nn = (struct ibv_sa_net_notice *)target_buf;
	    trap_num = ntohs(nn->trap_num_device_id);

	    evt->channel = channel;
	    evt->event.status = 0;
	    reg = oib_sa_find_reg(port, trap_num);
	    if (NULL == reg) {
		    evt->event.context = NULL;
	    } else {
    	    evt->event.context = reg->user_context;
	    }
	    evt->event.attr_id = IBV_SA_ATTR_NOTICE;

	    if (bytes_written) {
	        evt->data = malloc(bytes_written);
	        if (!evt->data) {
	            ret = FINSUFFICIENT_MEMORY;
	            goto err;
	        }

	        memcpy(evt->data, target_buf, bytes_written);
			(*event)->attr = evt->data;
			(*event)->attr_size = bytes_written;
			(*event)->attr_count = 1;
	    }
	    
	    return FSUCCESS;
    }
    else if (FTIMEOUT == ret) {
    	reg = (oib_sa_registration_t *)target_buf;
    	
    	evt->data = NULL;
	    evt->channel = channel;
    	evt->event.context = reg->user_context;
	    evt->event.attr_id = IBV_SA_ATTR_INFORM_INFO;
	    evt->event.status = -ETIMEDOUT;
	    return FSUCCESS;
    }
    ret = FERROR;
err:
    free(evt);
    return ret;
}

/**
 * Clean up ibv_sa_event structure
 *
 * @param event      Pointer to structure to place event notice into
 *
 * @return   0 if success, else error code
 */
FSTATUS oib_ack_sa_event(struct ibv_sa_event *event)
{
    struct oib_sa_event *evt = container_of(event, struct oib_sa_event, event);

    if (evt->data)
        free(evt->data);

    free(event);
    return FSUCCESS;
}
