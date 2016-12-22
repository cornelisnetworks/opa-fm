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

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <semaphore.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <infiniband/verbs.h>
#include <infiniband/umad.h>
#include <infiniband/umad_types.h>
#include <infiniband/umad_sa.h>
#include <infiniband/umad_sm.h>
#include <iba/ib_mad.h>

#define OIB_UTILS_PRIVATE 1

#include "ib_utils_openib.h"
#include "oib_utils_dump_mad.h"
#include "oib_utils_sa.h"
#include "stl_convertfuncs.h"
#include "iba/ib_sm.h"

FILE *dbg_file = NULL;
FILE *error_file = NULL;
/** ========================================================================= */
void oib_set_dbg(FILE *file)
{
	dbg_file = file;
}
/** ========================================================================= */
void oib_set_err(FILE *file)
{
	error_file = file;
}

/** =========================================================================
 * Init sub libraries like umad here
 * */
static int init_sub_lib(void)
{
	static int done = 0;

	if (done)
		return 0;

    if (umad_init() < 0) {
        OUTPUT_ERROR("can't init UMAD library\n");
        return EIO;
    }

	done = 1;
	return (0);
}

/** ========================================================================= 
 * Cache the pkeys associated with the port passed into the port object.
 *
 * @param port
 * @return 0 on success +ERRNO on failure
 */
static int cache_port_details(struct oib_port *port)
{
	int err = 0;

    if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OUTPUT_ERROR("Cannot cache pkeys, failed to acquire lock (err: %d)\n", err);
        return EIO;
    }

    if (port->umad_port_cache_valid) {
        umad_release_port(&port->umad_port_cache);
    }
    port->umad_port_cache.pkeys_size = 0;
    port->umad_port_cache_valid = 0;

    if (umad_get_port(port->hfi_name, port->hfi_port_num, &port->umad_port_cache) < 0) {
        OUTPUT_ERROR ("can't get UMAD port information (%s:%d)\n",
				port->hfi_name, port->hfi_port_num);
        err = EIO;
        goto fail;
    }

    if (!port->umad_port_cache.pkeys) {
        OUTPUT_ERROR ("no UMAD pkeys for (%s:%d)\n",
				port->hfi_name, port->hfi_port_num);
		err = EIO;
        goto fail2;
    }

	/* NOTE the umad interface returns network byte order here; fix it */
	port->umad_port_cache.port_guid = ntoh64(port->umad_port_cache.port_guid);
	port->umad_port_cache.gid_prefix = ntoh64(port->umad_port_cache.gid_prefix);

    port->umad_port_cache_valid = 1;
    oib_unlock_sem(&port->umad_port_cache_lock);
    return 0;

fail2:
    umad_release_port(&port->umad_port_cache);
    port->umad_port_cache.pkeys_size = 0;
    port->umad_port_cache_valid = 0;
fail:
    oib_unlock_sem(&port->umad_port_cache_lock);
    return err;
}

/** ========================================================================= */
static int open_verbs_ctx(struct oib_port *port)
{
	int i;
	int num_devices;
	struct  ibv_device **dev_list;

	dev_list = ibv_get_device_list(&num_devices);
	for (i = 0; i < num_devices; ++i)
		if (dev_list[i] != NULL
		    && (strncmp(dev_list[i]->name, port->hfi_name, sizeof(dev_list[i]->name)) == 0) )
			break;

	if (i >= num_devices) {
		ibv_free_device_list(dev_list);
		OUTPUT_ERROR("failed to find verbs device\n");
		return EIO;
	}

	port->verbs_ctx = ibv_open_device(dev_list[i]);

	ibv_free_device_list(dev_list);

	if (port->verbs_ctx == NULL)
	{
		OUTPUT_ERROR("failed to open verbs device\n");
		return EIO;
	}

	if (sem_init(&port->lock,0,1) != 0)
	{
		ibv_close_device(port->verbs_ctx);
		OUTPUT_ERROR("failed to init registry lock\n");
		return EIO;
	}

	return 0;
}

/** ========================================================================= */
static void close_verbs_ctx(struct oib_port *port)
{
	sem_destroy(&port->lock);
	ibv_close_device(port->verbs_ctx);
}

static void update_umad_port_cache(struct oib_port *port)
{
    int rc;
    struct ibv_async_event event;

    ibv_get_async_event(port->verbs_ctx, &event);
    ibv_ack_async_event(&event);
    switch (event.event_type) {
        case IBV_EVENT_PORT_ACTIVE:
        case IBV_EVENT_PORT_ERR:
        case IBV_EVENT_LID_CHANGE:
        case IBV_EVENT_PKEY_CHANGE:
            rc = cache_port_details(port);
            if (rc != 0)
                OUTPUT_ERROR ("umad port cache data invalid!\n");
            break;

        case IBV_EVENT_SM_CHANGE:
        case IBV_EVENT_CLIENT_REREGISTER:
            if (reregister_traps(port))
                OUTPUT_ERROR("failed to reregister traps.\n");
            rc = cache_port_details(port);
            if (rc != 0)
                OUTPUT_ERROR ("umad port cache data invalid!\n");
            break;
        case IBV_EVENT_CQ_ERR:
            OUTPUT_ERROR("got IBV_EVENT_CQ_ERR\n");
            break;
        default:
            break;
    }
}


static enum oib_th_event handle_th_msg(struct oib_port *port)
{
    struct oib_thread_msg msg;
    int s;

    memset(&msg, 0, sizeof(msg));

    s = read(port->umad_port_sv[1], &msg, sizeof(msg));
    if (s <= 0 || s != sizeof(msg)) {
        OUTPUT_ERROR ("th event read failed : %s\n", strerror(errno));
        return OIB_TH_EVT_NONE;
    }

    switch (msg.evt) {
    case OIB_TH_EVT_SHUTDOWN:
        break;
    case OIB_TH_EVT_UD_MONITOR_ON:
        port->run_sa_cq_poll = 1;
        break;
    case OIB_TH_EVT_UD_MONITOR_OFF:
        port->run_sa_cq_poll = 0;
        break;
    case OIB_TH_EVT_START_OUTSTANDING_REQ_TIME:
        port->poll_timeout_ms = NOTICE_REG_TIMEOUT_MS;
        break;
    default:
        OUTPUT_ERROR ("Unknown msg : %d\n", msg.evt);
        break;
    }

    return msg.evt;
}

static void *umad_port_thread(void *arg)
{
    struct oib_port *port = (struct oib_port *)arg;
    int err;

    port->poll_timeout_ms = -1;
    while (1) {
        int nfds = 0;
        int rc;
        struct pollfd pollfd[3];

        /* Verbs event */
        pollfd[0].fd = port->verbs_ctx->async_fd;
        pollfd[0].events = POLLIN;
        pollfd[0].revents = 0;
        nfds++;

        /* thread message */
        pollfd[1].fd = port->umad_port_sv[1];
        pollfd[1].events = POLLIN;
        pollfd[1].revents = 0;
        nfds++;

		/* Notice QP event */
        if (port->run_sa_cq_poll && port->sa_qp_comp_channel) {
            pollfd[2].fd = port->sa_qp_comp_channel->fd;
            pollfd[2].events = POLLIN;
            nfds++;
        }
        pollfd[2].revents = 0; /* this always needs to be cleared */

        if ((rc = poll(pollfd, nfds, port->poll_timeout_ms)) < 0) {
            if (rc < 0)
                OUTPUT_ERROR ("event poll failed : %s\n", strerror(errno));
            continue;
        }

        if (rc == 0) {
            /*  Timed out means we have periodic work to do */
            port->poll_timeout_ms = repost_pending_registrations(port);
            continue;
        }

        if (pollfd[1].revents & POLLIN) {
            if (handle_th_msg(port) == OIB_TH_EVT_SHUTDOWN)
                goto quit_thread;
        } else if (pollfd[0].revents & POLLNVAL) {
            goto quit_thread;
        } else if (pollfd[0].revents & POLLIN) {
            update_umad_port_cache(port);
        } else if (pollfd[2].revents & POLLIN) {
            handle_sa_ud_qp(port);
        } else {
            DBGPRINT("poll returned but no ready file desc: %d - %d, %d - %d, %d - %d\n",
                         pollfd[0].revents, port->verbs_ctx->async_fd,
                         pollfd[1].revents, port->umad_port_sv[1],
                         port->sa_qp_comp_channel? pollfd[2].revents : 0, 
			 port->sa_qp_comp_channel? port->sa_qp_comp_channel->fd : -1);
        }
    }

quit_thread:
    err = close(port->umad_port_sv[1]);
    if (err != 0) {
        OUTPUT_ERROR("Failed to close thread sock pair(1) : %s\n", strerror(errno));
    }
    return NULL;
}

/** ========================================================================= */
static int start_port_thread(struct oib_port *port)
{
    int err, flags;
    pthread_attr_t attr;

    err = socketpair(AF_UNIX, SOCK_DGRAM, 0, port->umad_port_sv);
    if (err != 0) {
        OUTPUT_ERROR("Failed to open thread sock pair : %s\n", strerror(errno));
        return EIO;
    }

    /* change the blocking mode of the async event queue */
    flags = fcntl(port->verbs_ctx->async_fd, F_GETFL);
    err = fcntl(port->verbs_ctx->async_fd, F_SETFL, flags | O_NONBLOCK);
    if (err < 0) {
        OUTPUT_ERROR("Failed to change file descriptor of async event queue\n");
        return EIO;
    }

    pthread_attr_init(&attr);
    err = pthread_create(&port->umad_port_thread, &attr, umad_port_thread, (void *)port);
    pthread_attr_destroy(&attr);
    return err;
}

static int find_pkey_from_umad_port(umad_port_t *umad_port, uint16_t pkey)
{
    int i = -1;

    if (pkey == 0xffff || pkey == 0x7fff) {
        /* Mgmt P_Keys are to be an exact match */
        for (i = 0; i < umad_port->pkeys_size; i++) {
            if (umad_port->pkeys[i] == pkey)
                goto done;
		}
        i = -1;
        goto done;
    }

    for (i = 0; i < umad_port->pkeys_size; i++) {
        if ((umad_port->pkeys[i] & OIB_PKEY_MASK) == (pkey & OIB_PKEY_MASK))
            goto done;
    }

    i = -1;

done:
	return i;
}

/*
 * Register to send IB DR SMAs
 * This is used only to send DR Packets to the local ports to determine OPA
 * support
 */
static int register_ib_smi_dr(int fd, uint32_t *aid)
{
    struct umad_reg_attr reg_attr;
	int err;

	memset(&reg_attr, 0, sizeof(reg_attr));
	reg_attr.mgmt_class = UMAD_CLASS_SUBN_DIRECTED_ROUTE;
    reg_attr.mgmt_class_version = 0x01;

	err = umad_register2(fd, &reg_attr, aid);
	if (err !=0 )
		DBGPRINT("OPA Port check: SMA register failed\n");

	return err;
}

/* Returns 0 (invalid base version) on error */
static uint8_t get_base_version_from_ni(int fd, uint32_t aid, int pkey_index)
{
	uint8_t rc;
    void *umad_p = NULL;
	struct umad_smp *send_mad;
	size_t length;

    umad_p = umad_alloc(1, sizeof(*send_mad) + umad_size());
    if (!umad_p) {
        OUTPUT_ERROR ("can't alloc umad for OPA check; send_size %ld\n", sizeof(*send_mad));
        return 0;
    }
    memset(umad_p, 0, sizeof(*send_mad) + umad_size());
    umad_set_grh(umad_p, 0);

    send_mad = umad_get_mad(umad_p);
	send_mad->base_version = UMAD_BASE_VERSION;
	send_mad->mgmt_class = UMAD_CLASS_SUBN_DIRECTED_ROUTE;
	send_mad->class_version = 0x01;
	send_mad->method = UMAD_METHOD_GET;
	send_mad->tid = htonl(0xDEADBEEF);
	send_mad->attr_id = htons(UMAD_SM_ATTR_NODE_INFO);
	send_mad->dr_slid = 0xffff;
	send_mad->dr_dlid = 0xffff;

    umad_set_pkey(umad_p, pkey_index);
    umad_set_addr(umad_p, 0xffff, 0, 0, 0);

	rc = 0;
    if (umad_send(fd, aid, umad_p, sizeof(*send_mad), 100, 1) < 0)
		goto free_mad;

	length = sizeof(*send_mad);
	if (umad_recv(fd, umad_p, (int *)&length, 100) < 0)
		goto free_mad;

	if (length < sizeof(*send_mad))
		goto free_mad;

	if (umad_status(umad_p) != 0)
		goto free_mad;

	rc = ((NODE_INFO *)(send_mad->data))->BaseVersion;

free_mad:
	free(umad_p);
	return rc;
}

static int port_is_opa(char *hfi_name, int port_num)
{
	int fd;
	int rc;
	int pkey_index;
    uint32_t aid;
	uint8_t base_version;
	umad_port_t umad_port;

	if (umad_get_port(hfi_name, port_num, &umad_port) != 0)
		return 0;

	pkey_index = find_pkey_from_umad_port(&umad_port, 0x7fff);
	umad_release_port(&umad_port);

	if (pkey_index < 0)
		return 0;

    if ((fd = umad_open_port(hfi_name, port_num)) < 0)
		return 0;

	if (register_ib_smi_dr(fd, &aid) != 0) {
		rc = 0;
		goto close;
	}

	base_version = get_base_version_from_ni(fd, aid, pkey_index);

	umad_unregister(fd, (int)aid);

	rc = (base_version == 0x80);

close:
	umad_close_port(fd);
	return rc;
}

static int hfi_is_opa(char *hfi_name)
{
	return port_is_opa(hfi_name, 1);
}

int oib_get_hfi_names(char hfis[][UMAD_CA_NAME_LEN], int max)
{
	int i;
	int caCount;
	int hfiCount = 0;
	char (*ca_names)[UMAD_CA_NAME_LEN];

	ca_names = calloc(max, UMAD_CA_NAME_LEN * sizeof(char));

	if (!ca_names)
		return 0;

    if ((caCount = umad_get_cas_names((void *)ca_names, max)) <= 0) {
		hfiCount = caCount;
		goto out;
	}

	for (i = 0; i < caCount; i++) {
		if (hfi_is_opa(ca_names[i])) {
			memcpy(hfis[hfiCount], ca_names[i], UMAD_CA_NAME_LEN);
			hfiCount++;
		}
	}

out:
	free(ca_names);

	return hfiCount;
}

static int open_first_opa_hfi(int port_num, char *hfi_name, size_t name_len)
{
	int ca_cnt;
	char hfis[UMAD_MAX_DEVICES][UMAD_CA_NAME_LEN];

	ca_cnt = oib_get_hfi_names(hfis, UMAD_MAX_DEVICES);
	if (ca_cnt < 0)
		return ca_cnt;

	ca_cnt = umad_open_port(hfis[0], port_num);

	if (ca_cnt >= 0) {
		strncpy(hfi_name, hfis[0], name_len);
		hfi_name[name_len-1] = '\0';
	}

	return ca_cnt;
}

/** ========================================================================= */
int oib_open_port(struct oib_port **port, char *hfi_name, uint8_t port_num)
{
	int i,j;
	int err;
	struct oib_port *rc;

	if (init_sub_lib())
		return (EIO);

	if ((rc = calloc(1, sizeof(*rc))) == NULL)
		return (ENOMEM);

    if ((rc->umad_fd = umad_open_port(hfi_name, port_num)) < 0) {
        OUTPUT_ERROR ("can't open UMAD port (%s:%d)\n", hfi_name, port_num);
        err = EIO;
		goto free_rc;
    }

	if (!port_is_opa(hfi_name, port_num)) {
		umad_close_port(rc->umad_fd);

		if (hfi_name) {
			OUTPUT_ERROR ("Port is not OPA (%s:%d)\n", hfi_name, port_num);
			err = EIO;
			goto free_rc;
		}

		/* hfi was wild carded, attempt to open the first OPA HFI */
		rc->umad_fd = open_first_opa_hfi(port_num, rc->hfi_name, sizeof(rc->hfi_name));
		if (rc->umad_fd < 0) {
			OUTPUT_ERROR ("OPA port not found (%d)\n", port_num);
			err = EIO;
			goto free_rc;
		}
	} else if (!hfi_name) {
		/* get the actual name from a null hfi_name */
		umad_port_t umad_port;

		if (umad_get_port(NULL, port_num, &umad_port) < 0) {
			OUTPUT_ERROR ("Failed to get umad port name (<null>:%d)\n", port_num);
			err = EIO;
			goto close_port;
		}

		snprintf(rc->hfi_name, sizeof(rc->hfi_name), "%s", umad_port.ca_name);

		umad_release_port(&umad_port);
	} else {
		snprintf(rc->hfi_name, sizeof(rc->hfi_name), "%s", hfi_name);
	}

	rc->hfi_port_num = port_num;
    rc->num_userspace_recv_buf = DEFAULT_USERSPACE_RECV_BUF;
    rc->num_userspace_send_buf = DEFAULT_USERSPACE_SEND_BUF;

	for (i = 0; i < OIB_MAX_CLASS_VERSION; i++) {
		for (j = 0; j < OIB_MAX_CLASS; j++) {
			rc->umad_agents[i][j] = OIB_INVALID_AGENTID;
		}
	}

	if (sem_init(&rc->umad_port_cache_lock,0,1) != 0) {
		OUTPUT_ERROR("failed to init umad_port_cache_lock\n");
		err = EIO;
        goto close_port;
	}

	if ((err = open_verbs_ctx(rc)) != 0)
		goto destroy_cache_lock;

	if ((err = cache_port_details(rc)) != 0)
		goto close_verbs;

    if ((err = start_port_thread(rc)) != 0)
        goto release_cache;

	rc->hfi_num = oib_get_hfiNum(rc->hfi_name);

    LIST_INIT(&rc->pending_reg_msg_head);
	*port = rc;
	return (0);

release_cache:
    umad_release_port(&rc->umad_port_cache);
close_verbs:
    close_verbs_ctx(rc);
destroy_cache_lock:
    sem_destroy(&rc->umad_port_cache_lock);
close_port:
    umad_close_port(rc->umad_fd);
free_rc:
	free(rc);
	return (err);
}

/** ========================================================================= */
int oib_open_port_by_num (struct oib_port **port, int hfiNum, uint8_t port_num)
{
	FSTATUS fstatus;
	char name[IBV_SYSFS_NAME_MAX];
	int  num = -1;
	uint32_t ca_count;
	uint32_t port_count;
	IB_PORT_ATTRIBUTES	*attrib = NULL;
	int ret_code;
	
    fstatus = oib_get_portguid(hfiNum, port_num, NULL, NULL, NULL, NULL, &attrib,
			       &ca_count, &port_count, name, &num, NULL);
	if (fstatus != FSUCCESS) {
		if (fstatus != FNOT_FOUND ||
		(ca_count == 0 || port_count == 0) ||
		hfiNum > ca_count || port_num > port_count) {
			ret_code=EIO;
			goto done;
		}
		/* no active port was found for wildcard ca/port.
		 * return EAGAIN so caller could query specific port/ca
		 */
		ret_code=EAGAIN;
		goto done;
	}

	ret_code = oib_open_port(port, name, num);

	// Remember the local guid for pa client
	if (ret_code >= 0 && port && *port && attrib) 
	{
		(*port)->local_gid = attrib->GIDTable[0];
	}
	
	done:
		if (attrib)
			MemoryDeallocate(attrib);
		return ret_code;
}

/** ========================================================================= */
int oib_open_port_by_guid(struct oib_port **port, uint64_t port_guid)
{
	FSTATUS fstatus;
	char      name[IBV_SYSFS_NAME_MAX];
	int       ca_num;
	int       num;
	uint64_t  prefix;
	uint16    sm_lid;
	uint8     sm_sl;
	uint8     port_state;
	int       rc = 0;
	
    fstatus = oib_get_hfi_from_portguid(port_guid, name, &ca_num, &num,
										&prefix, &sm_lid, &sm_sl, &port_state);
    if (fstatus != FSUCCESS) {
		return (EIO);
	}

	rc = oib_open_port(port, name, num);

	if (rc == 0) {
		(*port)->hfi_num = ca_num;
		snprintf((*port)->hfi_name, IBV_SYSFS_NAME_MAX, "%s", name);
		(*port)->hfi_port_num = num;
		(*port)->local_gid.Type.Global.SubnetPrefix = prefix;
		(*port)->local_gid.Type.Global.InterfaceID = port_guid;
	}

	return rc;
}

static void join_port_thread(struct oib_port *port)
{
    struct oib_thread_msg msg;
    int rc, err;
    int retries = 3;

    msg.size = sizeof(msg);
    msg.evt = OIB_TH_EVT_SHUTDOWN;

    do {
        rc = write(port->umad_port_sv[0], &msg, sizeof(msg));
        if (rc <= 0)
		    OUTPUT_ERROR("failed to send Thread shutdown to cache thread\n");
        /* I don't think we need to wait for a response */
    } while (rc <= 0 && retries--);

    if (rc <= 0) {
        OUTPUT_ERROR("Thread NOT SHUTDOWN aborting join...\n");
        return;
    }

    pthread_join(port->umad_port_thread, NULL);

    // close the sockets
    err = close(port->umad_port_sv[0]);
    if (err != 0) {
        OUTPUT_ERROR("Failed to close thread sock pair(0) : %s\n", strerror(errno));
    }
}

/** ========================================================================= */
static void destroy_sa_qp(struct oib_port *port)
{
    int i;

    // if the user just unregistered trap messages those messages may still
    // be on this list, wait 5 seconds for the thread to handle the response.
    for (i = 0; i < 5000; i++) {    
        if (!LIST_EMPTY(&port->pending_reg_msg_head)) {
            usleep(1000);
        }
        else {
            DBGPRINT("destroy_sa_qp: wait %d ms for LIST_EMPTY\n", i);
            break;
        }
    }

    stop_ud_cq_monitor(port);

    join_port_thread(port);

    /* Free any remaining unregistration messages */
    if (!LIST_EMPTY(&port->pending_reg_msg_head)) {
        OUTPUT_ERROR("Ignoring Pending Notice un-registation requests\n");
        oib_sa_remove_all_pending_reg_msgs(port);
    }

    if (port->sa_ah)
       ibv_destroy_ah(port->sa_ah);

    if (port->sa_qp)
        ibv_destroy_qp(port->sa_qp);

    for (i = 0; i<port->num_userspace_recv_buf; i++)
    	if (port->recv_bufs)
            ibv_dereg_mr(port->recv_bufs[i].mr);

    if (port->sa_qp_pd)
        ibv_dealloc_pd(port->sa_qp_pd);

    if (port->sa_qp_cq)
        ibv_destroy_cq(port->sa_qp_cq);

    if (port->recv_bufs) {
        free(port->recv_bufs);
        port->recv_bufs = NULL;
    }

    if (port->sa_qp_comp_channel)
        ibv_destroy_comp_channel(port->sa_qp_comp_channel);
}

/** ========================================================================= */
int oib_close_port(struct oib_port *port)
{
	int i,j;

    oib_sa_clear_regs_unsafe(port);

    destroy_sa_qp(port);

	close_verbs_ctx(port);

	for (i = 0; i < OIB_MAX_CLASS_VERSION; i++) {
		for (j = 0; j < OIB_MAX_CLASS; j++) {
			if (port->umad_agents[i][j] != OIB_INVALID_AGENTID) {
				umad_unregister(port->umad_fd, port->umad_agents[i][j]);
			}
		}
	}

	umad_close_port(port->umad_fd);
	sem_destroy(&port->umad_port_cache_lock);
	free(port);

	return 0;
}

/** ========================================================================= */
uint32_t oib_get_port_lid(struct oib_port *port)
{
    uint32_t rc;
	int err = 0;

    if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OUTPUT_ERROR("Cannot get port LID, failed to acquire lock (err: %d)\n", err);
        return 0;
    }

	rc = port->umad_port_cache.base_lid;

    oib_unlock_sem(&port->umad_port_cache_lock);
    return rc;
}

/** ========================================================================= */
uint16_t oib_get_mgmt_pkey(struct oib_port *port, uint16_t dlid, uint8_t hopCnt)
{
    uint16_t mgmt = 0;
	int err = 0;
    int i = 0;

    if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OUTPUT_ERROR("Cannot get port LID, failed to acquire lock (err: %d)\n", err);
        return mgmt;
    }

    // Look for full mgmt pkey first.
    for (i=0; i<port->umad_port_cache.pkeys_size; i++) {
        if (port->umad_port_cache.pkeys[i] == 0xffff) {
            mgmt = 0xffff;
            goto unlock;
        }
    }

    // If there is a hop count, is not local access,
    // requires a full pkey, and there is no full mgmt pkey in the table, 
    if (hopCnt != 0) 
        goto unlock;

    // Look for limited mgmt pkey only if local query
    if ((dlid==0) || (dlid==port->umad_port_cache.base_lid) || (dlid==0xffff)) {
        for (i=0; i<port->umad_port_cache.pkeys_size; i++) {
            if (port->umad_port_cache.pkeys[i] == 0x7fff) {
                mgmt = 0x7fff;
                goto unlock;
            }
        }
    }
unlock:
    oib_unlock_sem(&port->umad_port_cache_lock);
    return mgmt;
}

/** ========================================================================= */
uint32_t oib_get_port_sm_lid(struct oib_port *port)
{
    uint32_t rc;
	int err = 0;

    if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OUTPUT_ERROR("Cannot get port SM LID, failed to acquire lock (err: %d)\n", err);
        return 0;
    }

	rc = port->umad_port_cache.sm_lid;

    oib_unlock_sem(&port->umad_port_cache_lock);
    return rc;
}

/** ========================================================================= */
uint8_t oib_get_port_sm_sl(struct oib_port *port)
{
    uint8_t rc;
	int err = 0;

    if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OUTPUT_ERROR("Cannot get port SM SL, failed to acquire lock (err: %d)\n", err);
        return 0;
    }

	rc = port->umad_port_cache.sm_sl;

    oib_unlock_sem(&port->umad_port_cache_lock);
    return rc;
}

/** ========================================================================= */
uint64_t oib_get_port_guid(struct oib_port *port)
{
    uint64_t rc;
	int err = 0;

    if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OUTPUT_ERROR("Cannot get port GUID, failed to acquire lock (err: %d)\n", err);
        return 0;
    }

	rc = port->umad_port_cache.port_guid;

    oib_unlock_sem(&port->umad_port_cache_lock);
    return rc;
}
/** ========================================================================= */
uint8_t oib_get_hfi_port_num(struct oib_port *port)
{
	return (port->hfi_port_num);
}

/** ========================================================================= */
char *oib_get_hfi_name(struct oib_port *port)
{
	return (port->hfi_name);
}

/** ========================================================================= */
int oib_get_hfi_num(struct oib_port *port)
{
	return (port->hfi_num);
}

/** ========================================================================= */
uint64_t oib_get_port_prefix(struct oib_port *port)
{
    uint64_t rc;
	int err = 0;

    if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OUTPUT_ERROR("Cannot get port prefix, failed to acquire lock (err: %d)\n", err);
        return 0;
    }

	rc = port->umad_port_cache.gid_prefix;

    oib_unlock_sem(&port->umad_port_cache_lock);
    return rc;
}

/** ========================================================================= */
uint8_t oib_get_port_state(struct oib_port *port)
{
    uint8_t rc;
	int err = 0;

    if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OUTPUT_ERROR("Cannot get port state, failed to acquire lock (err: %d)\n", err);
        return 0;
    }

	rc = port->umad_port_cache.state;

    oib_unlock_sem(&port->umad_port_cache_lock);
    return rc;
}

/**
 * Gets the node type for the specified HFI.
 *
 * @param port          The pointer to the port object for which the node type is asked.
 * @param nodeType
 *
 * @return FSTATUS
 */
FSTATUS oib_get_hfi_node_type(IN struct oib_port *port, OUT int *nodeType)
{
	FSTATUS status;
	umad_ca_t cainfo;
	
	if (port == NULL)
	{
		OUTPUT_ERROR("Port must be specified.\n");
		return FINVALID_PARAMETER;
	}
	else if (nodeType == NULL)
	{
		OUTPUT_ERROR("Invalid output pointer.\n");
		return FINVALID_PARAMETER;
	}
	
	status = umad_get_ca(port->hfi_name, &cainfo);
	if (status != FSUCCESS)
	{
		OUTPUT_ERROR("umad_get_ca() failed (returned: %u)\n", status);
		return status;
	}
	
	*nodeType = cainfo.node_type;
	umad_release_ca(&cainfo);
	return FSUCCESS;
}

/**
 * Gets the ISSM device corresponding to the specified port.
 *
 * @param port      Pointer to the opened local port object.
 * @param path      Buffer in which to place the device path
 * @param path_max  Maximum length of the output buffer
 *
 * @return FSUCCESS on success, or UMad error code on failure
 */
FSTATUS oib_get_issm_device(struct oib_port *port, char *path, int path_max)
{
    int status;
	
	if (port == NULL)
	{
		OUTPUT_ERROR("port must be specified.\n");
		return FINVALID_PARAMETER;
	}
	else if (path == NULL)
	{
		OUTPUT_ERROR("Invalid output path buffer.\n");
		return FINVALID_PARAMETER;
	}
	
	status = umad_get_issm_path(port->hfi_name, port->hfi_port_num, path, path_max);
	if (status) {
		OUTPUT_ERROR("Failed to resolve ISSM device name (status: %d)\n", status);
		return status;
	}

	return FSUCCESS;
}

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
int oib_mad_refresh_port_pkey(struct oib_port *port)
{
    if (!port) 
    {
        OUTPUT_ERROR("%s: Port must be specified\n", __func__);
        return (-1);
    }

    // The local function takes care of everything.
    return cache_port_details(port);
}

/** ========================================================================= */
static int bind_single_class(struct oib_port *port,
						struct oib_class_args *mgmt_class, uint64_t *mask)
{
	int rereg = 0;
	int err;
    uint32_t aid;
    struct umad_reg_attr reg_attr;
	uint8_t mclass = mgmt_class->mgmt_class;
	uint8_t class_ver = mgmt_class->class_version;

	if (mclass >= OIB_MAX_CLASS ) {
		OUTPUT_ERROR("bad mgmt class : 0x%x\n", mclass);
		return EINVAL;
	}
	if (class_ver >= OIB_MAX_CLASS_VERSION ) {
		OUTPUT_ERROR("bad mgmt class version : 0x%x\n", class_ver);
		return EINVAL;
	}

	memset(&reg_attr, 0, sizeof(reg_attr));
    reg_attr.mgmt_class = mclass;
    reg_attr.mgmt_class_version = class_ver;
    reg_attr.flags = 0; 
    memcpy(reg_attr.method_mask, mask, sizeof(reg_attr.method_mask));
    reg_attr.rmpp_version = 0;
    if (mgmt_class->oui) {
        //memcpy(&reg_attr.oui, mgmt_class->oui, sizeof(reg_attr.oui));
        reg_attr.oui = ((uint32_t)mgmt_class->oui[0]) << 16;
        reg_attr.oui |= ((uint32_t)mgmt_class->oui[1]) << 8;
        reg_attr.oui |= (uint32_t)(mgmt_class->oui[2]);
    }

	// Exceptions to default values.
	switch (mgmt_class->mgmt_class) {
	case UMAD_CLASS_SUBN_ADM:
		if (!mgmt_class->kernel_rmpp) {
			reg_attr.flags = UMAD_USER_RMPP;
		}
		reg_attr.rmpp_version = 1;
		break;
    default:
		{
			/* Check for all RMPP based vendor classes */
			if (mgmt_class->mgmt_class >= 0x30 && mgmt_class->mgmt_class <= 0x4F)
			{
				if (!mgmt_class->kernel_rmpp) {
					reg_attr.flags = UMAD_USER_RMPP;
				}
				reg_attr.rmpp_version = 1;
			}
			break;
		}
	}

	if (port->umad_agents[class_ver][mclass] != OIB_INVALID_AGENTID) {
		OUTPUT_ERROR("WARNINIG re-register of class 0x%x; version 0x%x; was %d\n",
					mclass, class_ver, port->umad_agents[class_ver][mclass]);
		umad_unregister(port->umad_fd, port->umad_agents[class_ver][mclass]);
		rereg = 1;
	}

	if ((err = umad_register2(port->umad_fd, &reg_attr, &aid)) !=0) {
		OUTPUT_ERROR("Can't register agent for class 0x%x; version 0x%x; %s\n",
					mclass, class_ver, strerror(err));
		return err;
	}

	if (rereg)
		OUTPUT_ERROR("WARNINIG re-register new %d\n", aid);

	// Store the agent id; be able to correlate it to mgmt class/version.
	port->umad_agents[class_ver][mclass] = (int)aid;

    return 0;
}

static void set_bit64(int b, uint64_t *buf)
{
	uint64_t mask;
	uint64_t *addr = buf;

	addr += b >> 6;
	mask = 1ULL << (b & 0x3f);
	*addr |= mask;
}

/** ========================================================================= */
int oib_bind_classes(struct oib_port *port, struct oib_class_args *mgmt_classes)
{
	int rc;
    int i = 1;

    if (!port || port->umad_fd < 0) {
        OUTPUT_ERROR("Mad port is not initialized / opened\n");
        return EINVAL;
    }

    while (mgmt_classes && mgmt_classes->base_version != 0) {
        uint64_t method_mask[2];

		DBGPRINT("Registering 0x%x/0x%x with umad layer\n",
            mgmt_classes->mgmt_class, mgmt_classes->class_version);

        if (i >= OIB_MAX_CLASS) {
            OUTPUT_ERROR("too many classes %d requested\n", i);
            return EIO;
        }

        // Initialize method mask.
	    memset(method_mask, 0, sizeof (method_mask));
        if (mgmt_classes->use_methods) {
            int j=0;

			for (j = 0; j < OIB_CLASS_ARG_MAX_METHODS; j++) {
	        	if (mgmt_classes->methods[j])
		        	set_bit64(mgmt_classes->methods[j], method_mask);
	        }
        } else { 
	        if (mgmt_classes->is_responding_client) {
		        set_bit64(UMAD_METHOD_GET, method_mask);
		        set_bit64(UMAD_METHOD_SET, method_mask);
	        }
	        if (mgmt_classes->is_trap_client) {
		        set_bit64(UMAD_METHOD_TRAP, method_mask);
		        set_bit64(UMAD_METHOD_TRAP_REPRESS, method_mask);
	        }

    	    if (mgmt_classes->is_report_client) {
    		    set_bit64(UMAD_METHOD_REPORT, method_mask);
    	    }

		    if (mgmt_classes->is_responding_client && mgmt_classes->mgmt_class == UMAD_CLASS_SUBN_ADM) {
			    set_bit64(UMAD_SA_METHOD_GET_TABLE, method_mask);
			    set_bit64(UMAD_SA_METHOD_DELETE, method_mask);
			    set_bit64(UMAD_SA_METHOD_GET_TRACE_TABLE, method_mask);
		    }
        }

		rc = bind_single_class(port, mgmt_classes, method_mask);
		if (rc != 0)
			return rc;

		mgmt_classes++;
        i++;
	}

	return 0;
}

/** ========================================================================= */
FSTATUS oib_send_recv_mad_alloc(struct oib_port *port,
			uint8_t *send_mad, size_t send_size,
			struct oib_mad_addr *addr,
			uint8_t **recv_mad, size_t *recv_size,
			int timeout_ms, int retries)
{
	FSTATUS rc = oib_send_mad2(port, send_mad, send_size, addr, timeout_ms, retries);
	if (rc)
		return (rc);

	return (oib_recv_mad_alloc(port, recv_mad, recv_size, timeout_ms * (retries+2), addr));
}

/** ========================================================================= */
FSTATUS oib_send_recv_mad_no_alloc(struct oib_port *port,
			uint8_t *send_mad, size_t send_size,
			struct oib_mad_addr *addr,
			uint8_t *recv_mad, size_t *recv_size,
			int timeout_ms, int retries)
{
	FSTATUS rc = oib_send_mad2(port, send_mad, send_size, addr, timeout_ms, retries);
	if (rc)
		return (rc);

	return (oib_recv_mad_no_alloc(port, recv_mad, recv_size, timeout_ms * (retries+2), addr));
}

static uint16_t oib_find_pkey_from_idx(struct oib_port *port, unsigned idx)
{
    uint16_t rc;
    int err;

    if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OUTPUT_ERROR("Cannot locate pKey, failed to acquire lock (err: %d)\n", err);
        return 0x0000;
    }

    if (idx < port->umad_port_cache.pkeys_size)
        rc = port->umad_port_cache.pkeys[idx];
    else
        rc = 0x0000;

    oib_unlock_sem(&port->umad_port_cache_lock);

    return rc;
}

/** ========================================================================= */
int oib_find_pkey(struct oib_port *port, uint16_t pkey)
{
    int err;
    int i = -1;

    if (pkey == 0)
        return -1;

    if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OUTPUT_ERROR("Cannot find pKey, failed to acquire lock (err: %d)\n", err);
        return -1;
    }

	i = find_pkey_from_umad_port(&port->umad_port_cache, pkey);

    oib_unlock_sem(&port->umad_port_cache_lock);
    return i;
}

/** ========================================================================= */
static inline int is_my_lid_port(struct oib_port *port, uint32_t lid)
{
    int rc;
	int err = 0;
    unsigned last_lid;

    if ((err = oib_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OUTPUT_ERROR("Cannot check LID information, failed to acquire lock (err: %d)\n", err);
        return 0;
    }

    last_lid = port->umad_port_cache.base_lid + (1<<port->umad_port_cache.lmc)-1;

    if (lid >= port->umad_port_cache.base_lid && lid <= last_lid)
        rc = 1;
    else
        rc = 0;

    oib_unlock_sem(&port->umad_port_cache_lock);
    return rc;
}

/** ========================================================================= */
FSTATUS oib_send_mad2(struct oib_port *port, uint8_t *send_mad, size_t send_size,
			struct oib_mad_addr *addr, int timeout_ms, int retries)
{
	FSTATUS          status = FSUCCESS;
    void            *umad_p = NULL;
    int              response;
	uint8_t          mclass, class_ver;
    int              aid;
    int              correctedTimeout;
    struct umad_hdr *mad_hdr = (struct umad_hdr *)send_mad;
	uint16_t         ib_lid;
    int              pkey_idx;
    size_t           padded_size;

	if (!port || !send_mad || !send_size || !addr)
		return FINVALID_PARAMETER;

    ib_lid = addr->lid & 0xffff;

    // Make sure we are registered for this class/version...
	mclass = mad_hdr->mgmt_class;
	class_ver = mad_hdr->class_version;
	response = (mad_hdr->method & 0x80)
				|| (mad_hdr->method == UMAD_METHOD_TRAP_REPRESS)
				|| (mclass == UMAD_CLASS_BM &&
					ntohl(mad_hdr->attr_mod) & BM_ATTRIB_MOD_RESPONSE);
    aid  = port->umad_agents[class_ver][mclass];
    DBGPRINT (" Management Class 0x%x method 0x%x attrId 0x%x attrM 0x%x\n",mclass, mad_hdr->method,
              ntohs(mad_hdr->attr_id), ntohl(mad_hdr->attr_mod));
    DBGPRINT (" base_version 0x%x class_version 0x%x\n",mad_hdr->base_version, mad_hdr->class_version);

    if (aid == OIB_INVALID_AGENTID) {
		// automatically register for "send" only
		int err = 0;
		struct oib_class_args mgmt_class[2];

		memset(mgmt_class, 0, sizeof(mgmt_class));

		mgmt_class[0].base_version = mad_hdr->base_version;
		mgmt_class[0].mgmt_class = mad_hdr->mgmt_class;
		mgmt_class[0].class_version = mad_hdr->class_version;
		mgmt_class[0].is_responding_client = 0;
		mgmt_class[0].is_trap_client = 0;
		mgmt_class[0].is_report_client = 0;
		mgmt_class[0].kernel_rmpp = 1;
		mgmt_class[0].use_methods = 0;

		DBGPRINT ("auto registering class 0x%02x; version 0x%x for send only\n",
				mclass, class_ver);
		if ((err = oib_bind_classes(port, mgmt_class)) != 0) {
        	OUTPUT_ERROR ("Failed to auto register for class 0x%02x: %s\n",
							mclass, strerror(err));
        	status = FERROR;
        	goto done;
		}
		aid = port->umad_agents[class_ver][mclass];
    }

    // Initialize the user mad.
    // umad has limititation that outgoing packets must be > 36 bytes.
    padded_size = ( MAX(send_size,36) + 7) & ~0x7;
    DBGPRINT ("dlid %d qpn %d qkey %x sl %d\n", ib_lid, addr->qpn, addr->qkey, addr->sl);
    umad_p = umad_alloc(1, padded_size + umad_size());
    if (!umad_p) {
        OUTPUT_ERROR ("can't alloc umad send_size %ld\n", padded_size);
        status = FINSUFFICIENT_MEMORY;
        goto done;
    }
    memset(umad_p, 0, padded_size + umad_size());
    memcpy (umad_get_mad(umad_p), send_mad, send_size); /* Copy mad to umad */
    umad_set_grh(umad_p, 0);   

    pkey_idx = oib_find_pkey(port, addr->pkey);
    if (pkey_idx < 0) {
        DBGPRINT("P_Key 0x%x not found in pkey table\n", addr->pkey);
        if (addr->pkey == 0xffff) {
            pkey_idx = oib_find_pkey(port, 0x7fff);
            if (pkey_idx < 0) {
                OUTPUT_ERROR("Failed to find 0x7fff pkey defaulting to index 1\n");
                pkey_idx = 1;
            } else {
               DBGPRINT("... using 0x7fff found at index %d\n", pkey_idx);
            }
        } else {
            // Previously, this code would try to find the limited management pkey 
            //   if it could not find the requested pkey, and use that pkey instead.
            // This would often "work" because all nodes should have the limited
            //   management pkey, but b/c it was a limited member, this would result
            //   in potential timeouts - especially where the full management pkey was
            //   required.
            // Changed this code fail immediately without retrying a new pkey.
            OUTPUT_ERROR("Failed to find requested pkey:0x%x, class 0x%x aid:0x%x \n",
                         addr->pkey, mclass, ntohs(mad_hdr->attr_id));
            status = FPROTECTION;
            goto done;
        }
    }
    umad_set_pkey(umad_p, pkey_idx);

    umad_set_addr(umad_p, ib_lid?ib_lid:0xffff, addr->qpn, addr->sl, addr->qkey);

    if (dbg_file) {
        DBGPRINT(">>> sending: len %ld pktsz %zu\n", send_size, umad_size() + padded_size);
		umad_dump(umad_p);
        oib_dump_mad(dbg_file, umad_get_mad(umad_p), send_size, "send mad\n");
    }

    correctedTimeout = (timeout_ms == OIB_SEND_TIMEOUT_DEFAULT)
                     ? OIB_UTILS_DEF_TIMEOUT_MS : timeout_ms;

    if (umad_send(port->umad_fd, aid, umad_p, padded_size, (response ? 0 : correctedTimeout), retries) < 0) {
        OUTPUT_ERROR("send failed; %s, agent id %u MClass 0x%x method 0x%x attrId 0x%x attrM 0x%x\n",
				strerror(errno), aid, mclass, mad_hdr->method, 
				ntohs(mad_hdr->attr_id), ntohl(mad_hdr->attr_mod)); 
        status = FNOT_DONE;
        goto done;
    }

done:
    // Free umad if allocated.
    if (umad_p != NULL) {
        umad_free(umad_p);
    }

    return status;
}

/** ========================================================================= */
FSTATUS oib_recv_mad_alloc(struct oib_port *port, uint8_t **recv_mad, size_t *recv_size,
			int timeout_ms, struct oib_mad_addr *addr)
{
	#define STL_MAD_SIZE    2048        // Note, hack should reference size value OFED standard header file
	FSTATUS        status = FSUCCESS; 
	ib_user_mad_t *umad = NULL; 
	int            mad_agent; 
	uint32_t       my_umad_status = 0; 
	size_t         length;
	
	if (!port || !recv_mad || !recv_size)
		return FINVALID_PARAMETER;

	length = STL_MAD_SIZE; 
	umad = umad_alloc(1, umad_size() + length); 
	if (!umad) {
		OUTPUT_ERROR("can't alloc MAD sized umad\n"); 
		status = FINSUFFICIENT_MEMORY; 
		goto done;
	}
   
retry:
   mad_agent = umad_recv(port->umad_fd, umad, (int *)&length, timeout_ms); 
   // There are 4 combinations:
   //	assorted errors: mad_agent < 0, length <= MAD_SIZE 
   //	large RMPP response: mad_agent < 0, length > MAD_SIZE, umad_status==0
   //	got response: mad_agent >= 0, length <= MAD_SIZE, umad_status==0
   //	no response: mad_agent >= 0, length <= MAD_SIZE, umad_status == error
   if (mad_agent < 0) {
      if (length <= STL_MAD_SIZE) {
			// no MAD returned.  None available.
            DBGPRINT ("recv error on MAD sized umad (%s) length=%ld\n",
			  strerror(errno), length);
			if (errno == EINTR)
				goto retry;

            status = (errno == ETIMEDOUT)?FNOT_DONE:FERROR;
            goto done;
        } else {
            /* Need a larger buffer for RMPP */
            DBGPRINT ("Received 1st MAD length=%ld\n",length);
            umad_free(umad);

            umad = umad_alloc(1, umad_size() + length);
            if (!umad) {
                OUTPUT_ERROR ("can't alloc umad length %ld\n", length);
                status = FINSUFFICIENT_MEMORY;
                goto done;
            }

			// just to be safe, we supply a timeout.  However it
			// should be unnecessary since we know we have a packet
retry2:
            if ((mad_agent = umad_recv(port->umad_fd, umad, (int *)&length, OIB_UTILS_DEF_TIMEOUT_MS)) < 0) {
                OUTPUT_ERROR ("recv error on umad length %ld (%s)\n", length, strerror(errno));
				if (errno == EINTR)
					goto retry2;

                status = FOVERRUN;
				*recv_size = length;
                goto done;
            }
        }
    }
    if (mad_agent >= UMAD_CA_MAX_AGENTS) { 
        OUTPUT_ERROR ("invalid mad agent %d - dropping\n", mad_agent);
        status = FERROR;
        goto done;
    }

    my_umad_status = umad_status(umad);
    DBGPRINT("UMAD Status: %s (%d)\n", strerror(my_umad_status), my_umad_status);
	if (my_umad_status != 0) {
        status = (my_umad_status == ETIMEDOUT) ? FTIMEOUT : FREJECT;
	}

    DBGPRINT("Received MAD length=%ld, total umad size=%ld\n",length, length + umad_size());

    if (dbg_file) {
        struct umad_hdr * umad_hdr = (struct umad_hdr *)umad_get_mad(umad);
        DBGPRINT("  Base_Version 0x%x Class 0x%x Method 0x%x attrId 0x%x attr_mod 0x%x status 0x%x\n",
                 umad_hdr->base_version, umad_hdr->mgmt_class, umad_hdr->method,
                 umad_hdr->attr_id, umad_hdr->attr_mod, umad_hdr->status);
        umad_dump(umad);
        oib_dump_mad(dbg_file, umad_get_mad(umad), length, "rcv mad\n");
        
    }

    // Allocate and copy to new buffer.
    *recv_mad = calloc (1, length);

    if (*recv_mad == NULL) {
        OUTPUT_ERROR ("can't alloc return buffer length %ld\n", length);
        status = FINSUFFICIENT_MEMORY;
        goto done;
    }

    memcpy (*recv_mad, umad_get_mad(umad), length); 
	*recv_size = length;

	if (addr != NULL) {
		addr->lid  = IB2STL_LID(ntoh16(umad->addr.lid));
		addr->sl   = umad->addr.sl;
		addr->qkey = ntoh32(umad->addr.qkey);
		addr->qpn  = ntoh32(umad->addr.qpn);
		addr->pkey = oib_find_pkey_from_idx(port, umad_get_pkey(umad));
	}

done:
    if (umad != NULL) {
        umad_free(umad);
    }
    return status;
}

/** ========================================================================= */
FSTATUS oib_recv_mad_no_alloc(struct oib_port *port, uint8_t *recv_mad, size_t *recv_size,
			int timeout_ms, struct oib_mad_addr *addr)
{
    size_t         length = *recv_size;
	ib_user_mad_t *umad = NULL; 
    int            mad_agent;
    uint32_t       my_umad_status = 0;
    FSTATUS        status = FSUCCESS;

	if (!port || !recv_mad || !*recv_size)
		return FINVALID_PARAMETER;

    umad = umad_alloc(1, length + umad_size());
    if (!umad) {
        OUTPUT_ERROR ("can't alloc umad length %ld\n", length);
        status = FINSUFFICIENT_MEMORY;
        goto done;
    }

retry:
    mad_agent = umad_recv(port->umad_fd, umad, (int *)&length, timeout_ms);
	// There are 4 combinations:
	//	assorted errors: mad_agent < 0, length <= MAD_SIZE 
	//	large RMPP response: mad_agent < 0, length > MAD_SIZE, umad_status==0
	//	got response: mad_agent >= 0, length <= MAD_SIZE, umad_status==0
	//	no response: mad_agent >= 0, length <= MAD_SIZE, umad_status == error
    if (mad_agent < 0) {
        if (length <= *recv_size) {
			// no MAD returned.  None available.
            DBGPRINT ("recv error on umad (size %zu) (%s)\n", *recv_size,
			  strerror(errno));
			if (errno == EINTR)
				goto retry;

            status = (errno == ETIMEDOUT) ? FNOT_DONE:FERROR;
            goto done;
        } else {
			// this routine is not expecting large responses
            OUTPUT_ERROR ("Rx Packet size %zu larger than mad-size %zu\n",
						  length, *recv_size);
            status = FOVERRUN;
			if (recv_mad)
                memcpy(recv_mad, umad_get_mad(umad), *recv_size);

            // Clean out Rx packet 'cause it will never go away..
            umad_free(umad);
            umad = umad_alloc(1, umad_size() + length);
            if (!umad) {
                OUTPUT_ERROR ("can't alloc umad for rx cleanup, length %ld\n", length);
                status = FINSUFFICIENT_MEMORY;
                goto done;
            }

			// just to be safe, we supply a timeout.  However it
			// should be unnecessary since we know we have a packet
retry2:
            if (umad_recv(port->umad_fd, umad, (int *)&length, OIB_UTILS_DEF_TIMEOUT_MS) < 0) {
                OUTPUT_ERROR ("recv error on cleanup, length %ld (%s)\n", length,
			      strerror(errno));
				if (errno == EINTR)
					goto retry2;

                goto done;
            }

    		if (dbg_file) {
				umad_dump(umad);
		        oib_dump_mad(dbg_file, umad_get_mad(umad), length, "rcv mad discarded\n");
			}
            goto done;
        }
    }
    if (mad_agent >= UMAD_CA_MAX_AGENTS) { 
        OUTPUT_ERROR ("invalid mad agent %d\n", mad_agent);
        status = FERROR;
        goto done;
    }

    my_umad_status = umad_status(umad);
    DBGPRINT("UMAD Status: %s (%d)\n", strerror(my_umad_status), my_umad_status);
	if (my_umad_status != 0) {
        status = (my_umad_status == ETIMEDOUT) ? FTIMEOUT : FREJECT;
    }

    DBGPRINT("Received MAD: Agent %d, length=%ld\n", mad_agent, length);
    if (dbg_file) {
        umad_dump(umad);
        oib_dump_mad(dbg_file, umad_get_mad(umad), length, "rcv mad\n");
    }

    // Copy the data
    if (recv_mad && length > 0) {
        *recv_size = length;
        memcpy(recv_mad, umad_get_mad(umad), length);
    }

	if (addr != NULL) {
		addr->lid  = IB2STL_LID(ntoh16(umad->addr.lid));
		addr->sl   = umad->addr.sl;
		addr->qkey = ntoh32(umad->addr.qkey);
		addr->qpn  = ntoh32(umad->addr.qpn);
		addr->pkey = oib_find_pkey_from_idx(port, umad_get_pkey(umad));
	}

done:
    if (umad != NULL) {
        umad_free(umad);
    }
    return status;
}

/** =========================================================================
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
		uint64_t            *pRetGIDPrefix)
{

}
*/

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
    )
{
    return port->mad_status;
}
