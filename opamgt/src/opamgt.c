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

#define OPAMGT_PRIVATE 1
#include "ib_utils_openib.h"
#include "omgt_oob_net.h"
#include "opamgt_dump_mad.h"
#include "opamgt_sa_priv.h"
#include "opamgt_pa_priv.h"
#include "stl_convertfuncs.h"
#include "iba/ib_sm_priv.h"
#include "iba/ib_generalServices.h"


/** ========================================================================= */
void omgt_set_dbg(struct omgt_port *port, FILE *file)
{
	if (port) port->dbg_file = file;
}
/** ========================================================================= */
void omgt_set_err(struct omgt_port *port, FILE *file)
{
	if (port) port->error_file = file;
}
/** ========================================================================= */
void omgt_set_timeout(struct omgt_port *port, int ms_timeout)
{
	if (port) {
		port->ms_timeout = (ms_timeout > 0 ? ms_timeout : OMGT_DEF_TIMEOUT_MS);
	}
}
/** ========================================================================= */
void omgt_set_retry_count(struct omgt_port *port, int retry_count)
{
	if (port) {
		port->retry_count = (retry_count >= 0 ? retry_count : OMGT_DEF_RETRY_CNT);
	}
}
/** ========================================================================= */
const char* omgt_status_totext(OMGT_STATUS_T status)
{
	switch (status) {
	case OMGT_STATUS_SUCCESS:                return "Success";
	case OMGT_STATUS_ERROR:                  return "Error";
	case OMGT_STATUS_INVALID_STATE:          return "Invalid State";
	case OMGT_STATUS_INVALID_OPERATION:      return "Invalid Operation";
	case OMGT_STATUS_INVALID_SETTING:        return "Invalid Setting";
	case OMGT_STATUS_INVALID_PARAMETER:      return "Invalid Parameter";
	case OMGT_STATUS_INSUFFICIENT_RESOURCES: return "Insufficient Resources";
	case OMGT_STATUS_INSUFFICIENT_MEMORY:    return "Insufficient Memory";
	case OMGT_STATUS_COMPLETED:              return "Completed";
	case OMGT_STATUS_NOT_DONE:               return "Not Done";
	case OMGT_STATUS_PENDING:                return "Pending";
	case OMGT_STATUS_TIMEOUT:                return "Timeout";
	case OMGT_STATUS_CANCELED:               return "Canceled";
	case OMGT_STATUS_REJECT:                 return "Reject";
	case OMGT_STATUS_OVERRUN:                return "Overrun";
	case OMGT_STATUS_PROTECTION:             return "Protection";
	case OMGT_STATUS_NOT_FOUND:              return "Not Found";
	case OMGT_STATUS_UNAVAILABLE:            return "Unavailable";
	case OMGT_STATUS_BUSY:                   return "Busy";
	case OMGT_STATUS_DISCONNECT:             return "Disconnect";
	case OMGT_STATUS_DUPLICATE:              return "Duplicate";
	case OMGT_STATUS_POLL_NEEDED:            return "Poll Needed";
	default: return "Unknown";
	}
}
/** ========================================================================= */
const char* omgt_service_state_totext(int service_state)
{
	switch (service_state) {
	case OMGT_SERVICE_STATE_OPERATIONAL: return "Operational";
	case OMGT_SERVICE_STATE_DOWN:        return "Down";
	case OMGT_SERVICE_STATE_UNAVAILABLE: return "Unavailable";
	case OMGT_SERVICE_STATE_UNKNOWN:
	default:
		return "Unknown";
	}
}
/** =========================================================================
 * Init sub libraries like umad here
 * */
static int init_sub_lib(struct omgt_port *port)
{
	static int done = 0;

	if (done)
		return 0;

    if (umad_init() < 0) {
        OMGT_OUTPUT_ERROR(port, "can't init UMAD library\n");
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
static int cache_port_details(struct omgt_port *port)
{
	int err = 0;

    if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot cache pkeys, failed to acquire lock (err: %d)\n", err);
        return EIO;
    }

    if (port->umad_port_cache_valid) {
        umad_release_port(&port->umad_port_cache);
    }
    port->umad_port_cache.pkeys_size = 0;
    port->umad_port_cache_valid = 0;

    if (umad_get_port(port->hfi_name, port->hfi_port_num, &port->umad_port_cache) < 0) {
        OMGT_OUTPUT_ERROR(port, "can't get UMAD port information (%s:%d)\n",
				port->hfi_name, port->hfi_port_num);
        err = EIO;
        goto fail;
    }

    if (!port->umad_port_cache.pkeys) {
        OMGT_OUTPUT_ERROR(port, "no UMAD pkeys for (%s:%d)\n",
				port->hfi_name, port->hfi_port_num);
		err = EIO;
        goto fail2;
    }

    /* NOTE the umad interface returns network byte order here; fix it */
    port->umad_port_cache.port_guid = ntoh64(port->umad_port_cache.port_guid);
    port->umad_port_cache.gid_prefix = ntoh64(port->umad_port_cache.gid_prefix);

    port->umad_port_cache_valid = 1;
    omgt_unlock_sem(&port->umad_port_cache_lock);
    
    return 0;

fail2:
    umad_release_port(&port->umad_port_cache);
    port->umad_port_cache.pkeys_size = 0;
    port->umad_port_cache_valid = 0;
fail:
    omgt_unlock_sem(&port->umad_port_cache_lock);
    return err;
}

/** ========================================================================= */
static int open_verbs_ctx(struct omgt_port *port)
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
		OMGT_OUTPUT_ERROR(port, "failed to find verbs device\n");
		return EIO;
	}

	port->verbs_ctx = ibv_open_device(dev_list[i]);

	ibv_free_device_list(dev_list);

	if (port->verbs_ctx == NULL)
	{
		OMGT_OUTPUT_ERROR(port, "failed to open verbs device\n");
		return EIO;
	}

	if (sem_init(&port->lock,0,1) != 0)
	{
		ibv_close_device(port->verbs_ctx);
		OMGT_OUTPUT_ERROR(port, "failed to init registry lock\n");
		return EIO;
	}

	return 0;
}

/** ========================================================================= */
static void close_verbs_ctx(struct omgt_port *port)
{
	sem_destroy(&port->lock);
	ibv_close_device(port->verbs_ctx);
}

static void update_umad_port_cache(struct omgt_port *port)
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
        case IBV_EVENT_GID_CHANGE:
            rc = cache_port_details(port);
            if (rc != 0)
                OMGT_OUTPUT_ERROR(port, "umad port cache data invalid!\n");
            break;

        case IBV_EVENT_SM_CHANGE:
        case IBV_EVENT_CLIENT_REREGISTER:
            if (reregister_traps(port))
                OMGT_OUTPUT_ERROR(port, "failed to reregister traps.\n");
            rc = cache_port_details(port);
            if (rc != 0)
                OMGT_OUTPUT_ERROR(port, "umad port cache data invalid!\n");
            break;
        case IBV_EVENT_CQ_ERR:
            OMGT_OUTPUT_ERROR(port, "got IBV_EVENT_CQ_ERR\n");
            break;
        default:
            break;
    }
}


static enum omgt_th_event handle_th_msg(struct omgt_port *port)
{
    struct omgt_thread_msg msg;
    int s;

    memset(&msg, 0, sizeof(msg));

    s = read(port->umad_port_sv[1], &msg, sizeof(msg));
    if (s <= 0 || s != sizeof(msg)) {
        OMGT_OUTPUT_ERROR(port, "th event read failed : %s\n", strerror(errno));
        return OMGT_TH_EVT_NONE;
    }

    switch (msg.evt) {
    case OMGT_TH_EVT_SHUTDOWN:
        break;
    case OMGT_TH_EVT_UD_MONITOR_ON:
        port->run_sa_cq_poll = 1;
        break;
    case OMGT_TH_EVT_UD_MONITOR_OFF:
        port->run_sa_cq_poll = 0;
        break;
    case OMGT_TH_EVT_START_OUTSTANDING_REQ_TIME:
        port->poll_timeout_ms = NOTICE_REG_TIMEOUT_MS;
        break;
    default:
        OMGT_OUTPUT_ERROR(port, "Unknown msg : %d\n", msg.evt);
        break;
    }

    return msg.evt;
}

static void *umad_port_thread(void *arg)
{
    struct omgt_port *port = (struct omgt_port *)arg;
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
                OMGT_OUTPUT_ERROR(port, "event poll failed : %s\n", strerror(errno));
            continue;
        }

        if (rc == 0) {
            /*  Timed out means we have periodic work to do */
            port->poll_timeout_ms = repost_pending_registrations(port);
            continue;
        }

        if (pollfd[1].revents & POLLIN) {
            if (handle_th_msg(port) == OMGT_TH_EVT_SHUTDOWN)
                goto quit_thread;
        } else if (pollfd[0].revents & POLLNVAL) {
            goto quit_thread;
        } else if (pollfd[0].revents & POLLIN) {
            update_umad_port_cache(port);
        } else if (pollfd[2].revents & POLLIN) {
            handle_sa_ud_qp(port);
        } else {
            OMGT_DBGPRINT(port, "poll returned but no ready file desc: %d - %d, %d - %d, %d - %d\n",
                         pollfd[0].revents, port->verbs_ctx->async_fd,
                         pollfd[1].revents, port->umad_port_sv[1],
                         port->sa_qp_comp_channel? pollfd[2].revents : 0, 
			 port->sa_qp_comp_channel? port->sa_qp_comp_channel->fd : -1);
        }
    }

quit_thread:
    err = close(port->umad_port_sv[1]);
    if (err != 0) {
        OMGT_OUTPUT_ERROR(port, "Failed to close thread sock pair(1) : %s\n", strerror(errno));
    }
    return NULL;
}

/** ========================================================================= */
static int start_port_thread(struct omgt_port *port)
{
    int err, flags;
    pthread_attr_t attr;

    err = socketpair(AF_UNIX, SOCK_DGRAM, 0, port->umad_port_sv);
    if (err != 0) {
        OMGT_OUTPUT_ERROR(port, "Failed to open thread sock pair : %s\n", strerror(errno));
        return EIO;
    }

    /* change the blocking mode of the async event queue */
    flags = fcntl(port->verbs_ctx->async_fd, F_GETFL);
    err = fcntl(port->verbs_ctx->async_fd, F_SETFL, flags | O_NONBLOCK);
    if (err < 0) {
        OMGT_OUTPUT_ERROR(port, "Failed to change file descriptor of async event queue\n");
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
        if ((umad_port->pkeys[i] & OMGT_PKEY_MASK) == (pkey & OMGT_PKEY_MASK))
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
	send_mad->dr_slid = LID_PERMISSIVE;
	send_mad->dr_dlid = LID_PERMISSIVE;

    umad_set_pkey(umad_p, pkey_index);
    umad_set_addr(umad_p, LID_PERMISSIVE, 0, 0, 0);

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
/** ========================================================================= */
OMGT_STATUS_T omgt_get_hfi_names(char hfis[][UMAD_CA_NAME_LEN], int32_t max, int32_t *hfi_count)
{
	OMGT_STATUS_T status = OMGT_STATUS_SUCCESS;
	int i;
	int caCount;
	int hfiCount = 0;
	char (*ca_names)[UMAD_CA_NAME_LEN];

	if (hfi_count) *hfi_count = -1;
	ca_names = calloc(max, UMAD_CA_NAME_LEN * sizeof(char));
	if (!ca_names) {
		return OMGT_STATUS_INSUFFICIENT_MEMORY;
	}

    if ((caCount = umad_get_cas_names((void *)ca_names, max)) <= 0) {
		hfiCount = caCount;
		status = OMGT_STATUS_NOT_FOUND;
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

	if (hfi_count) *hfi_count = hfiCount;
	return status;
}
/** ========================================================================= */
static int open_first_opa_hfi(int port_num, char *hfi_name, size_t name_len)
{
	int ca_cnt;
	char hfis[UMAD_MAX_DEVICES][UMAD_CA_NAME_LEN];

	(void)omgt_get_hfi_names(hfis, UMAD_MAX_DEVICES, &ca_cnt);
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
static OMGT_STATUS_T omgt_open_port_internal(struct omgt_port *port, char *hfi_name, uint8_t port_num)
{
	int i,j;
	OMGT_STATUS_T err;

	if (init_sub_lib(port)) {
		err = OMGT_STATUS_UNAVAILABLE;
		goto free_port;
	}

	/* Set Timeout and retry to default values */
	port->ms_timeout = OMGT_DEF_TIMEOUT_MS;
	port->retry_count = OMGT_DEF_RETRY_CNT;

	if ((port->umad_fd = umad_open_port(hfi_name, port_num)) < 0) {
		OMGT_OUTPUT_ERROR(port, "can't open UMAD port (%s:%d)\n", hfi_name, port_num);
		err = OMGT_STATUS_INVALID_PARAMETER;
		goto free_port;
    }

	if (!port_is_opa(hfi_name, port_num)) {
		umad_close_port(port->umad_fd);

		if (hfi_name) {
			OMGT_OUTPUT_ERROR(port, "Port is not OPA (%s:%d)\n", hfi_name, port_num);
			err = OMGT_STATUS_INVALID_PARAMETER;
			goto free_port;
		}

		/* hfi was wild carded, attempt to open the first OPA HFI */
		port->umad_fd = open_first_opa_hfi(port_num, port->hfi_name, sizeof(port->hfi_name));
		if (port->umad_fd < 0) {
			OMGT_OUTPUT_ERROR (port, "OPA port not found (%d)\n", port_num);
			err = OMGT_STATUS_NOT_FOUND;
			goto free_port;
		}
	} else if (!hfi_name) {
		/* get the actual name from a null hfi_name */
		umad_port_t umad_port;

		if (umad_get_port(NULL, port_num, &umad_port) < 0) {
			OMGT_OUTPUT_ERROR (port ,"Failed to get umad port name (<null>:%d)\n", port_num);
			err = OMGT_STATUS_INVALID_PARAMETER;
			goto close_port;
		}

		snprintf(port->hfi_name, sizeof(port->hfi_name), "%s", umad_port.ca_name);

		umad_release_port(&umad_port);
	} else {
		snprintf(port->hfi_name, sizeof(port->hfi_name), "%s", hfi_name);
	}

	port->hfi_port_num = port_num;
	port->num_userspace_recv_buf = DEFAULT_USERSPACE_RECV_BUF;
	port->num_userspace_send_buf = DEFAULT_USERSPACE_SEND_BUF;

	for (i = 0; i < OMGT_MAX_CLASS_VERSION; i++) {
		for (j = 0; j < OMGT_MAX_CLASS; j++) {
			port->umad_agents[i][j] = OMGT_INVALID_AGENTID;
		}
	}

	if (sem_init(&port->umad_port_cache_lock,0,1) != 0) {
		OMGT_OUTPUT_ERROR(port, "failed to init umad_port_cache_lock\n");
		err = OMGT_STATUS_INVALID_STATE;
        goto close_port;
	}

	if (open_verbs_ctx(port) != 0) {
		err = OMGT_STATUS_ERROR;
		goto destroy_cache_lock;
	}

	if (cache_port_details(port) != 0) {
		err = OMGT_STATUS_ERROR;
		goto close_verbs;
	}

    if (start_port_thread(port) != 0) {
		err = OMGT_STATUS_ERROR;
        goto release_cache;
	}

	port->hfi_num = omgt_get_hfi_num(port->hfi_name);

	LIST_INIT(&port->pending_reg_msg_head);
	return (OMGT_STATUS_SUCCESS);

release_cache:
	umad_release_port(&port->umad_port_cache);
close_verbs:
	close_verbs_ctx(port);
destroy_cache_lock:
	sem_destroy(&port->umad_port_cache_lock);
close_port:
	 umad_close_port(port->umad_fd);
free_port:
	return (err);
}

/** ========================================================================= */
OMGT_STATUS_T omgt_open_port(struct omgt_port **port, char *hfi_name, uint8_t port_num, struct omgt_params *session_params)
{
	OMGT_STATUS_T status;
	struct omgt_port *rc;

	if ((rc = calloc(1, sizeof(*rc))) == NULL)
		return (OMGT_STATUS_INSUFFICIENT_MEMORY);

	if (session_params) {
		rc->dbg_file = session_params->debug_file;
		rc->error_file = session_params->error_file;
	} else {
		rc->dbg_file = NULL;
		rc->error_file = NULL;
	}

	status = omgt_open_port_internal(rc, hfi_name, port_num);
	if (status == OMGT_STATUS_SUCCESS) {
		rc->is_oob_enabled = FALSE;
		*port = rc;
	} else {
		free(rc);
		*port = NULL;
	}

	return status;
}

/** ========================================================================= */
OMGT_STATUS_T omgt_open_port_by_num(struct omgt_port **port, int32_t hfi_num, uint8_t port_num, struct omgt_params *session_params)
{
	OMGT_STATUS_T status;
	char name[IBV_SYSFS_NAME_MAX];
	int num = -1;
	uint32_t hfi_count = 0, port_count = 0;
	struct omgt_port *rc = NULL;

	if ((rc = calloc(1, sizeof(*rc))) == NULL)
		return (OMGT_STATUS_INSUFFICIENT_MEMORY);

	if (session_params) {
		rc->dbg_file = session_params->debug_file;
		rc->error_file = session_params->error_file;
	} else {
		rc->dbg_file = NULL;
		rc->error_file = NULL;
	}

	status = omgt_get_portguid(hfi_num, port_num, NULL, rc, NULL, NULL, NULL,
		NULL, &hfi_count, &port_count, name, &num, NULL);
	if (status != OMGT_STATUS_SUCCESS) {
		if (status != OMGT_STATUS_NOT_FOUND ||
			(hfi_count == 0 || port_count == 0) ||
			hfi_num > hfi_count || port_num > port_count) {
				goto done;
		}
		/* No active port was found for wildcard hfi/port.
		 * return NOT_DONE so caller could query specific hfi/port
		 */
		status = OMGT_STATUS_NOT_DONE;
		goto done;
	}

	status = omgt_open_port_internal(rc, name, num);

done:
	if (status == OMGT_STATUS_SUCCESS) {
		rc->is_oob_enabled = FALSE;
		*port = rc;
	} else if (rc) {
		free(rc);
		rc = NULL;
	}

	return status;
}

/** ========================================================================= */
OMGT_STATUS_T omgt_open_port_by_guid(struct omgt_port **port, uint64_t port_guid, struct omgt_params *session_params)
{
	OMGT_STATUS_T status;
	char name[IBV_SYSFS_NAME_MAX];
	int num = -1;
	struct omgt_port *rc;

	if ((rc = calloc(1, sizeof(*rc))) == NULL)
		return (OMGT_STATUS_INSUFFICIENT_MEMORY);

	if (session_params) {
		rc->dbg_file = session_params->debug_file;
		rc->error_file = session_params->error_file;
	} else {
		rc->dbg_file = NULL;
		rc->error_file = NULL;
	}

	status = omgt_get_hfi_from_portguid(port_guid, rc, name, NULL, &num,
		NULL, NULL, NULL, NULL);
	if (status != OMGT_STATUS_SUCCESS) {
		goto done;
	}

	status = omgt_open_port_internal(rc, name, num);

done:
	if (status == OMGT_STATUS_SUCCESS) {
		rc->is_oob_enabled = FALSE;
		*port = rc;
	} else {
		free(rc);
		rc = NULL;
	}
	return status;
}

static void join_port_thread(struct omgt_port *port)
{
    struct omgt_thread_msg msg;
    int rc, err;
    int retries = 3;

    msg.size = sizeof(msg);
    msg.evt = OMGT_TH_EVT_SHUTDOWN;

    do {
        rc = write(port->umad_port_sv[0], &msg, sizeof(msg));
        if (rc <= 0)
		   OMGT_OUTPUT_ERROR(port, "failed to send Thread shutdown to cache thread\n");
        /* I don't think we need to wait for a response */
    } while (rc <= 0 && retries--);

    if (rc <= 0) {
       OMGT_OUTPUT_ERROR(port, "Thread NOT SHUTDOWN aborting join...\n");
        return;
    }

    pthread_join(port->umad_port_thread, NULL);

    // close the sockets
    err = close(port->umad_port_sv[0]);
    if (err != 0) {
       OMGT_OUTPUT_ERROR(port, "Failed to close thread sock pair(0) : %s\n", strerror(errno));
    }
}

/** ========================================================================= */
static void destroy_sa_qp(struct omgt_port *port)
{
    int i;

    // if the user just unregistered trap messages those messages may still
    // be on this list, wait 5 seconds for the thread to handle the response.
    for (i = 0; i < 5000; i++) {    
        if (!LIST_EMPTY(&port->pending_reg_msg_head)) {
            usleep(1000);
        }
        else {
            OMGT_DBGPRINT(port, "destroy_sa_qp: wait %d ms for LIST_EMPTY\n", i);
            break;
        }
    }

    stop_ud_cq_monitor(port);

    join_port_thread(port);

    /* Free any remaining unregistration messages */
    if (!LIST_EMPTY(&port->pending_reg_msg_head)) {
        OMGT_OUTPUT_ERROR(port, "Ignoring Pending Notice un-registration requests\n");
        omgt_sa_remove_all_pending_reg_msgs(port);
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
void omgt_close_port(struct omgt_port *port)
{
	int i, j;
	FSTATUS status = FSUCCESS;

	/* If port is in OOB mode just close connction */
	if (port->is_oob_enabled) {
		if (port->is_ssl_enabled && port->is_ssl_initialized) {
			if (port->x509_store) {
				X509_STORE_free(port->x509_store);
				port->x509_store = NULL;
				port->is_x509_store_initialized = 0;
			}
			if (port->dh_params) {
				DH_free(port->dh_params);
				port->dh_params = NULL;
				port->is_dh_params_initialized = 0;
			}
			if (port->ssl_context) {
				SSL_CTX_free(port->ssl_context);
				port->ssl_context = NULL;
			}
			/* Clean up SSL_load_error_strings() from omgt_oob_ssl_init() */
			ERR_free_strings();
			port->is_ssl_initialized = 0;
		}
		if ((status = omgt_oob_disconnect(port, port->conn)) != FSUCCESS) {
			OMGT_OUTPUT_ERROR(port, "Failed to disconnect from OOB connection: %u\n", status);
		}
		port->conn = NULL;
		if (port->is_oob_notice_setup &&
			(status = omgt_oob_disconnect(port, port->notice_conn)) != FSUCCESS)
		{
			OMGT_OUTPUT_ERROR(port, "Failed to disconnect from OOB Notice connection: %u\n", status);
		}
		port->notice_conn = NULL;
		free(port);
		return;
	}

	omgt_sa_clear_regs_unsafe(port);

	destroy_sa_qp(port);

	close_verbs_ctx(port);

	for (i = 0; i < OMGT_MAX_CLASS_VERSION; i++) {
		for (j = 0; j < OMGT_MAX_CLASS; j++) {
			if (port->umad_agents[i][j] != OMGT_INVALID_AGENTID) {
				umad_unregister(port->umad_fd, port->umad_agents[i][j]);
			}
		}
	}

	umad_close_port(port->umad_fd);
	sem_destroy(&port->umad_port_cache_lock);
	free(port);

}

/** ========================================================================= */
uint16_t omgt_get_mgmt_pkey(struct omgt_port *port, STL_LID dlid, uint8_t hopCnt)
{
	uint16_t mgmt = 0;
	int err = 0;
	int i = 0;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no pkey\n");
		return 0;
	}
	if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot get port LID, failed to acquire lock (err: %d)\n", err);
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
	if ((dlid == 0) || (dlid == port->umad_port_cache.base_lid) || (dlid == STL_LID_PERMISSIVE)) {
		for (i=0; i<port->umad_port_cache.pkeys_size; i++) {
			if (port->umad_port_cache.pkeys[i] == 0x7fff) {
				mgmt = 0x7fff;
				goto unlock;
			}
		}
	}
unlock:
	omgt_unlock_sem(&port->umad_port_cache_lock);
	return mgmt;
}

/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_port_lid(struct omgt_port *port, uint32_t *port_lid)
{
	int err = 0;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no LID\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot get port LID, failed to acquire lock (err: %d)\n", err);
		return OMGT_STATUS_PROTECTION;
	}

	*port_lid = port->umad_port_cache.base_lid;

	omgt_unlock_sem(&port->umad_port_cache_lock);
	return OMGT_STATUS_SUCCESS;
}

/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_port_lmc(struct omgt_port *port, uint32_t *port_lmc)
{
	int err = 0;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no LMC\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot get port LMC, failed to acquire lock (err: %d)\n", err);
		return OMGT_STATUS_PROTECTION;
	}
    *port_lmc = port->umad_port_cache.lmc;

	omgt_unlock_sem(&port->umad_port_cache_lock);
	return OMGT_STATUS_SUCCESS;
}

/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_port_sm_lid(struct omgt_port *port, uint32_t *sm_lid)
{
	int err = 0;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no SM LID\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot get port SM LID, failed to acquire lock (err: %d)\n", err);
		return OMGT_STATUS_PROTECTION;
	}

	*sm_lid = port->umad_port_cache.sm_lid;

	omgt_unlock_sem(&port->umad_port_cache_lock);
	return OMGT_STATUS_SUCCESS;
}
/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_port_sm_sl(struct omgt_port *port, uint8_t *sm_sl)
{
	int err = 0;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no SM SL\n");
		return OMGT_STATUS_INVALID_STATE;
	}
    if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot get port SM SL, failed to acquire lock (err: %d)\n", err);
        return OMGT_STATUS_PROTECTION;
    }

	*sm_sl = port->umad_port_cache.sm_sl;

    omgt_unlock_sem(&port->umad_port_cache_lock);
    return OMGT_STATUS_SUCCESS;
}
/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_port_prefix(struct omgt_port *port, uint64_t *prefix)
{
	int err = 0;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no Port Prefix\n");
		return OMGT_STATUS_INVALID_STATE;
	}
    if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot get Port Prefix, failed to acquire lock (err: %d)\n", err);
        return OMGT_STATUS_PROTECTION;
    }

	*prefix = port->umad_port_cache.gid_prefix;

    omgt_unlock_sem(&port->umad_port_cache_lock);
    return OMGT_STATUS_SUCCESS;
}
/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_port_guid(struct omgt_port *port, uint64_t *port_guid)
{
	int err = 0;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no Port GUID\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot get Port GUID, failed to acquire lock (err: %d)\n", err);
		return OMGT_STATUS_PROTECTION;
	}

	*port_guid = port->umad_port_cache.port_guid;

	omgt_unlock_sem(&port->umad_port_cache_lock);
	return OMGT_STATUS_SUCCESS;
}
/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_hfi_port_num(struct omgt_port *port, uint8_t *port_num)
{
	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no HFI Port number\n");
		return OMGT_STATUS_INVALID_STATE;
	}

	*port_num = port->hfi_port_num;

	if (port->hfi_port_num <= 0) {
		OMGT_OUTPUT_ERROR(port, "HFI Port Number not properly initialized: %d\n", port->hfi_port_num);
		return OMGT_STATUS_ERROR;
	}

    return OMGT_STATUS_SUCCESS;
}
/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_hfi_num(struct omgt_port *port, int32_t *hfi_num)
{
	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no HFI number\n");
		return OMGT_STATUS_INVALID_STATE;
	}

	*hfi_num = port->hfi_num;

	if (port->hfi_num <= 0) {
		OMGT_OUTPUT_ERROR(port, "HFI Number not properly initialized: %d\n", port->hfi_num);
		return OMGT_STATUS_ERROR;
	}

    return OMGT_STATUS_SUCCESS;
}

/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_hfi_name(struct omgt_port *port, char hfi_name[IBV_SYSFS_NAME_MAX])
{
	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no HFI name\n");
		return OMGT_STATUS_INVALID_STATE;
	}

	snprintf(hfi_name, IBV_SYSFS_NAME_MAX, "%s", port->hfi_name);

	if (strnlen(port->hfi_name, IBV_SYSFS_NAME_MAX) <= 0) {
		OMGT_OUTPUT_ERROR(port, "HFI name not properly initialized\n");
		return OMGT_STATUS_ERROR;
	}

    return OMGT_STATUS_SUCCESS;
}

/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_port_state(struct omgt_port *port, uint8_t *port_state)
{
	int err = 0;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no HFI number\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot get Port State, failed to acquire lock (err: %d)\n", err);
		return OMGT_STATUS_PROTECTION;
	}

	*port_state = (uint8_t)port->umad_port_cache.state;

	omgt_unlock_sem(&port->umad_port_cache_lock);
	return OMGT_STATUS_SUCCESS;
}
/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_node_type(struct omgt_port *port, uint8_t *node_type)
{
	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no NodeType\n");
		return OMGT_STATUS_INVALID_STATE;
	}

	*node_type = STL_NODE_FI;
	return OMGT_STATUS_SUCCESS;
}
OMGT_STATUS_T omgt_port_get_sa_service_state(struct omgt_port *port, int *sa_service_state, uint32_t refresh)
{
	int need_refresh = 0;
	OMGT_STATUS_T query_status = OMGT_STATUS_SUCCESS;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no SA Service State\n");
		return OMGT_STATUS_INVALID_STATE;
	}

	/* Check if we should attempt to refresh */
	switch (refresh) {
	case OMGT_REFRESH_SERVICE_NOP:
		break;
	case OMGT_REFRESH_SERVICE_BAD_STATE:
		if (port->sa_service_state != OMGT_SERVICE_STATE_OPERATIONAL) {
			need_refresh = 1;
		}
		break;
	case OMGT_REFRESH_SERVICE_ANY_STATE:
		need_refresh = 1;
		break;
	default:
		OMGT_OUTPUT_ERROR(port, "Invalid Refresh Flags: 0x%x\n", refresh);
		return OMGT_STATUS_INVALID_PARAMETER;
	}
	if (need_refresh == 1) {
		/* Refresh SA Client */
		query_status = omgt_query_sa(port, NULL, NULL);
		if (query_status != OMGT_STATUS_SUCCESS) {
			OMGT_OUTPUT_ERROR(port, "Failed to refresh SA Service State: %u\n", query_status);
			return query_status;
		}
	}

	*sa_service_state = port->sa_service_state;
	return OMGT_STATUS_SUCCESS;
}

OMGT_STATUS_T omgt_port_get_pa_service_state(struct omgt_port *port, int *pa_service_state, uint32_t refresh)
{
	int need_refresh = 0;

	if (port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in Out-of-Band Mode, no PA Service State\n");
		return OMGT_STATUS_INVALID_STATE;
	}

	/* Check if we should attempt to refresh */
	switch (refresh) {
	case OMGT_REFRESH_SERVICE_NOP:
		break;
	case OMGT_REFRESH_SERVICE_BAD_STATE:
		if (port->pa_service_state != OMGT_SERVICE_STATE_OPERATIONAL) {
			need_refresh = 1;
		}
		break;
	case OMGT_REFRESH_SERVICE_ANY_STATE:
		need_refresh = 1;
		break;
	default:
		OMGT_OUTPUT_ERROR(port, "Invalid Refresh Flags: 0x%x\n", refresh);
		return OMGT_STATUS_INVALID_PARAMETER;
	}
	if (need_refresh == 1) {
		/* Refresh PA Client */
		(void)omgt_pa_service_connect(port);
	}

	*pa_service_state = port->pa_service_state;
	return OMGT_STATUS_SUCCESS;
}
/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_ip_version(struct omgt_port *port, uint8_t *ip_version)
{
	if (!port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in In-Band Mode, no IP Version\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if (port->conn == NULL || port->conn->sock == INVALID_SOCKET) {
		OMGT_OUTPUT_ERROR(port, "Net Connection not initialized\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if (port->conn->err) {
		OMGT_DBGPRINT(port, "Net Connection has the Error Flag set: %d\n", port->conn->err);
	}

	*ip_version = (port->conn->ipv6 ? 6 : 4);

	return OMGT_STATUS_SUCCESS;
}
/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_ipv4_addr(struct omgt_port *port, struct in_addr *ipv4_addr)
{
	if (!port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in In-Band Mode, no IPv4 Address\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if (port->conn == NULL || port->conn->sock == INVALID_SOCKET) {
		OMGT_OUTPUT_ERROR(port, "Net Connection not initialized\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if (port->conn->ipv6) {
		OMGT_OUTPUT_ERROR(port, "Net Connection is using IPv6, no IPv4 Address\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if (port->conn->err) {
		OMGT_DBGPRINT(port, "Net Connection has the Error Flag set: %d\n", port->conn->err);
	}

	*ipv4_addr = port->conn->v4_addr.sin_addr;

	return OMGT_STATUS_SUCCESS;
}
/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_ipv6_addr(struct omgt_port *port, struct in6_addr *ipv6_addr)
{
	if (!port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in In-Band Mode, no IPv4 Address\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if (port->conn == NULL || port->conn->sock == INVALID_SOCKET) {
		OMGT_OUTPUT_ERROR(port, "Net Connection not initialized\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if (!port->conn->ipv6) {
		OMGT_OUTPUT_ERROR(port, "Net Connection is using IPv4, no IPv6 Address\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if (port->conn->err) {
		OMGT_DBGPRINT(port, "Net Connection has the Error Flag set: %d\n", port->conn->err);
	}

	*ipv6_addr = port->conn->v6_addr.sin6_addr;

	return OMGT_STATUS_SUCCESS;
}
/** ========================================================================= */
OMGT_STATUS_T omgt_port_get_ip_addr_text(struct omgt_port *port, char buf[], size_t buf_len)
{
	if (!port->is_oob_enabled) {
		OMGT_OUTPUT_ERROR(port, "Port in In-Band Mode, no IP Address\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if (port->conn == NULL || port->conn->sock == INVALID_SOCKET) {
		OMGT_OUTPUT_ERROR(port, "Net Connection not initialized\n");
		return OMGT_STATUS_INVALID_STATE;
	}
	if (port->conn->err) {
		OMGT_DBGPRINT(port, "Net Connection has the Error Flag set: %d\n", port->conn->err);
	}

	if (port->conn->ipv6) {
		inet_ntop(AF_INET6, &(port->conn->v6_addr.sin6_addr), buf, buf_len);
	} else {
		inet_ntop(AF_INET, &(port->conn->v4_addr.sin_addr), buf, buf_len);
	}

	return OMGT_STATUS_SUCCESS;
}
/**
 * Gets the ISSM device corresponding to the specified port.
 *
 * @param port      Pointer to the opened local port object.
 * @param path      Buffer in which to place the device path
 * @param path_max  Maximum length of the output buffer
 *
 * @return OMGT_STATUS_SUCCESS on success, or (umad) error code on failure
 */
OMGT_STATUS_T omgt_get_issm_device(struct omgt_port *port, char *path, int32_t path_max)
{
    int status;
	
	if (port == NULL)
	{
		OMGT_OUTPUT_ERROR(port, "port must be specified.\n");
		return OMGT_STATUS_INVALID_PARAMETER;
	}
	else if (path == NULL)
	{
		OMGT_OUTPUT_ERROR(port, "Invalid output path buffer.\n");
		return OMGT_STATUS_INVALID_PARAMETER;
	}
	if (port->is_oob_enabled) {
		OMGT_DBGPRINT(port, "Port in Out-of-Band Mode, no NodeType\n");
		return FINVALID_STATE;
	}
	
	status = umad_get_issm_path(port->hfi_name, port->hfi_port_num, path, path_max);
	if (status) {
		OMGT_OUTPUT_ERROR(port, "Failed to resolve ISSM device name (status: %d)\n", status);
		return status;
	}

	return OMGT_STATUS_SUCCESS;
}

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
int32_t omgt_mad_refresh_port_pkey(struct omgt_port *port)
{
    if (!port) 
    {
        OMGT_OUTPUT_ERROR(port, "%s: Port must be specified\n", __func__);
        return (-1);
    }

    // The local function takes care of everything.
    return cache_port_details(port);
}

/** ========================================================================= */
static int bind_single_class(struct omgt_port *port,
						struct omgt_class_args *mgmt_class, uint64_t *mask)
{
	int rereg = 0;
	int err;
    uint32_t aid;
    struct umad_reg_attr reg_attr;
	uint8_t mclass = mgmt_class->mgmt_class;
	uint8_t class_ver = mgmt_class->class_version;

	if (mclass >= OMGT_MAX_CLASS ) {
		OMGT_OUTPUT_ERROR(port, "bad mgmt class : 0x%x\n", mclass);
		return EINVAL;
	}
	if (class_ver >= OMGT_MAX_CLASS_VERSION ) {
		OMGT_OUTPUT_ERROR(port, "bad mgmt class version : 0x%x\n", class_ver);
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

	if (port->umad_agents[class_ver][mclass] != OMGT_INVALID_AGENTID) {
		OMGT_OUTPUT_ERROR(port, "WARNINIG re-register of class 0x%x; version 0x%x; was %d\n",
					mclass, class_ver, port->umad_agents[class_ver][mclass]);
		umad_unregister(port->umad_fd, port->umad_agents[class_ver][mclass]);
		rereg = 1;
	}

	if ((err = umad_register2(port->umad_fd, &reg_attr, &aid)) !=0) {
		OMGT_OUTPUT_ERROR(port, "Can't register agent for class 0x%x; version 0x%x; %s\n",
					mclass, class_ver, strerror(err));
		return err;
	}

	if (rereg)
		OMGT_OUTPUT_ERROR(port, "WARNINIG re-register new %d\n", aid);

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
int omgt_bind_classes(struct omgt_port *port, struct omgt_class_args *mgmt_classes)
{
	int rc;
    int i = 1;

    if (!port || port->umad_fd < 0) {
        OMGT_OUTPUT_ERROR(port, "Mad port is not initialized / opened\n");
        return EINVAL;
    }

    while (mgmt_classes && mgmt_classes->base_version != 0) {
        uint64_t method_mask[2];

		OMGT_DBGPRINT(port, "Registering 0x%x/0x%x with umad layer\n",
            mgmt_classes->mgmt_class, mgmt_classes->class_version);

        if (i >= OMGT_MAX_CLASS) {
            OMGT_OUTPUT_ERROR(port, "too many classes %d requested\n", i);
            return EIO;
        }

        // Initialize method mask.
	    memset(method_mask, 0, sizeof (method_mask));
        if (mgmt_classes->use_methods) {
            int j=0;

			for (j = 0; j < OMGT_CLASS_ARG_MAX_METHODS; j++) {
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
FSTATUS omgt_send_recv_mad_alloc(struct omgt_port *port,
	uint8_t *send_mad, size_t send_size, struct omgt_mad_addr *addr,
	uint8_t **recv_mad, size_t *recv_size, int timeout_ms, int retries)
{
	FSTATUS rc = FNOT_DONE;
	if (port->is_oob_enabled) {
		rc = omgt_oob_send_packet(port, send_mad, send_size);
		if (rc) return rc;

		return (omgt_oob_receive_response(port, recv_mad, (uint32_t *)recv_size));
	} else {
		rc = omgt_send_mad2(port, send_mad, send_size, addr, timeout_ms, retries);
		if (rc) return (rc);

		return (omgt_recv_mad_alloc(port, recv_mad, recv_size, timeout_ms * (retries + 2), addr));
	}
}

/** ========================================================================= */
FSTATUS omgt_send_recv_mad_no_alloc(struct omgt_port *port,
	uint8_t *send_mad, size_t send_size, struct omgt_mad_addr *addr,
	uint8_t *recv_mad, size_t *recv_size, int timeout_ms, int retries)
{
	FSTATUS rc = FNOT_DONE;
	if (port->is_oob_enabled) {
		return FINVALID_PARAMETER;
	} else {
		rc = omgt_send_mad2(port, send_mad, send_size, addr, timeout_ms, retries);
		if (rc) return (rc);

		return (omgt_recv_mad_no_alloc(port, recv_mad, recv_size, timeout_ms * (retries + 2), addr));
	}
}

static uint16_t omgt_find_pkey_from_idx(struct omgt_port *port, unsigned idx)
{
    uint16_t rc;
    int err;

    if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot locate pKey, failed to acquire lock (err: %d)\n", err);
        return 0x0000;
    }

    if (idx < port->umad_port_cache.pkeys_size)
        rc = port->umad_port_cache.pkeys[idx];
    else
        rc = 0x0000;

    omgt_unlock_sem(&port->umad_port_cache_lock);

    return rc;
}

/** ========================================================================= */
int32_t omgt_find_pkey(struct omgt_port *port, uint16_t pkey)
{
    int err;
    int i = -1;

    if (pkey == 0)
        return -1;

    if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot find pKey, failed to acquire lock (err: %d)\n", err);
        return -1;
    }

	i = find_pkey_from_umad_port(&port->umad_port_cache, pkey);

    omgt_unlock_sem(&port->umad_port_cache_lock);
    return i;
}

/** ========================================================================= */
static inline int is_my_lid_port(struct omgt_port *port, uint32_t lid)
{
    int rc;
	int err = 0;
    unsigned last_lid;

    if ((err = omgt_lock_sem(&port->umad_port_cache_lock)) != 0) {
		OMGT_OUTPUT_ERROR(port, "Cannot check LID information, failed to acquire lock (err: %d)\n", err);
        return 0;
    }

    last_lid = port->umad_port_cache.base_lid + (1<<port->umad_port_cache.lmc)-1;

    if (lid >= port->umad_port_cache.base_lid && lid <= last_lid)
        rc = 1;
    else
        rc = 0;

    omgt_unlock_sem(&port->umad_port_cache_lock);
    return rc;
}

static int omgt_extract_lid(ib_user_mad_t *umad)
{
	uint64_t dest_if_id;

	if (umad->addr.grh_present) {
		memcpy(&dest_if_id, umad->addr.gid + 8, sizeof(dest_if_id));
		dest_if_id = ntoh64(dest_if_id);
		if ((dest_if_id >> 40) == OMGT_STL_OUI)
			return dest_if_id & 0xFFFFFFFF;
	}
	return IB2STL_LID(ntoh16(umad->addr.lid));
}

/** ========================================================================= */
FSTATUS omgt_send_mad2(struct omgt_port *port, uint8_t *send_mad, size_t send_size,
			struct omgt_mad_addr *addr, int timeout_ms, int retries)
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
    struct ib_mad_addr *mad_addr;
    int slid_is_permissive = 0;
    int dlid_is_permissive = 0;
    IB_GID gid;
    struct umad_smp *ib_mad;
    STL_SMP *stl_mad;

    if (!port || !send_mad || !send_size || !addr)
        return FINVALID_PARAMETER;

    // Make sure we are registered for this class/version...
	mclass = mad_hdr->mgmt_class;
	class_ver = mad_hdr->class_version;
	response = (mad_hdr->method & 0x80)
				|| (mad_hdr->method == UMAD_METHOD_TRAP_REPRESS)
				|| (mclass == UMAD_CLASS_BM &&
					ntohl(mad_hdr->attr_mod) & BM_ATTRIB_MOD_RESPONSE);
    aid  = port->umad_agents[class_ver][mclass];
    OMGT_DBGPRINT(port, " Management Class 0x%x method 0x%x attrId 0x%x attrM 0x%x\n",mclass, mad_hdr->method,
              ntohs(mad_hdr->attr_id), ntohl(mad_hdr->attr_mod));
    OMGT_DBGPRINT(port, " base_version 0x%x class_version 0x%x\n",mad_hdr->base_version, mad_hdr->class_version);

    if (aid == OMGT_INVALID_AGENTID) {
		// automatically register for "send" only
		int err = 0;
		struct omgt_class_args mgmt_class[2];

		memset(mgmt_class, 0, sizeof(mgmt_class));

		mgmt_class[0].base_version = mad_hdr->base_version;
		mgmt_class[0].mgmt_class = mad_hdr->mgmt_class;
		mgmt_class[0].class_version = mad_hdr->class_version;
		mgmt_class[0].is_responding_client = 0;
		mgmt_class[0].is_trap_client = 0;
		mgmt_class[0].is_report_client = 0;
		mgmt_class[0].kernel_rmpp = 1;
		mgmt_class[0].use_methods = 0;

		OMGT_DBGPRINT(port, "auto registering class 0x%02x; version 0x%x for send only\n",
				mclass, class_ver);
		if ((err = omgt_bind_classes(port, mgmt_class)) != 0) {
			OMGT_OUTPUT_ERROR(port, "Failed to auto register for class 0x%02x: %s\n",
							mclass, strerror(err));
        	status = FERROR;
        	goto done;
		}
		aid = port->umad_agents[class_ver][mclass];
    }

    // Initialize the user mad.
    // umad has limititation that outgoing packets must be > 36 bytes.
    padded_size = ( MAX(send_size,36) + 7) & ~0x7;
    OMGT_DBGPRINT (port, "dlid %d qpn %d qkey %x sl %d\n", addr->lid, addr->qpn, addr->qkey, addr->sl);

    umad_p = umad_alloc(1,  padded_size + umad_size());
    if (!umad_p) {
        OMGT_OUTPUT_ERROR(port, "can't alloc umad send_size %ld\n", padded_size + umad_size());
        status = FINSUFFICIENT_MEMORY;
        goto done;
    }
    memset(umad_p, 0, padded_size + umad_size());

	memcpy (umad_get_mad(umad_p), send_mad, send_size); /* Copy mad to umad */

    /**
     *  If incoming dlid is 0, set it to a permissive LID.
     * If slid is an extended LID, set it to 0xFFFFFFFF.
     * If not, set it to 0xFFFF
     */
    if (addr->lid == 0) {
	if (omgt_is_ext_lid(port->umad_port_cache.base_lid))
		addr->lid = STL_LID_PERMISSIVE;
	else
		addr->lid = LID_PERMISSIVE;
    }

    ib_lid = addr->lid & 0xffff;

    /* Pure Directed Route packets dont need GRH */
    if (mad_hdr->mgmt_class == UMAD_CLASS_SUBN_DIRECTED_ROUTE) {
    	if (mad_hdr->base_version == UMAD_BASE_VERSION) {
	    /* IB MAD */
    	    ib_mad = (struct umad_smp *)send_mad;
    	    slid_is_permissive = (ib_mad->dr_slid == LID_PERMISSIVE);
    	    dlid_is_permissive = (ib_mad->dr_dlid == LID_PERMISSIVE);
        } else {
	    /* OPA MAD */
 	    stl_mad = (STL_SMP *)send_mad;
    	    slid_is_permissive = (stl_mad->SmpExt.DirectedRoute.DrSLID
				== STL_LID_PERMISSIVE);
    	    dlid_is_permissive = (stl_mad->SmpExt.DirectedRoute.DrDLID
				== STL_LID_PERMISSIVE);
        }
    }
	OMGT_DBGPRINT(port, "dlid: 0x%x, slid: 0x%x\n",
		addr->lid, port->umad_port_cache.base_lid);
	if ((!slid_is_permissive) || (!dlid_is_permissive)) {
		/* Not a pure DR packet */
		if ((omgt_is_ext_lid(addr->lid)) ||
			omgt_is_ext_lid(port->umad_port_cache.base_lid) ||
			addr->flags & OMGT_MAD_ADDR_16B) {
			mad_addr = umad_get_mad_addr(umad_p);
			mad_addr->gid_index = 0;
			mad_addr->grh_present = 1;
			mad_addr->hop_limit = 1;
			gid.Type.Global.SubnetPrefix = port->umad_port_cache.gid_prefix;
			gid.Type.Global.InterfaceID = omgt_create_gid(addr->lid);
			BSWAP_IB_GID(&gid);
			OMGT_DBGPRINT(port, "Assigned DGID: 0x%lx:0x%lx\n",
				ntoh64(gid.Type.Global.InterfaceID),
				ntoh64(gid.Type.Global.SubnetPrefix));
			memcpy(mad_addr->gid, &gid, sizeof(gid));
		} else {
			umad_set_grh(umad_p, 0);
		}
	} else {
		umad_set_grh(umad_p, 0);
	}

    pkey_idx = omgt_find_pkey(port, addr->pkey);
    if (pkey_idx < 0) {
        OMGT_DBGPRINT(port, "P_Key 0x%x not found in pkey table\n", addr->pkey);
        if (addr->pkey == 0xffff) {
            pkey_idx = omgt_find_pkey(port, 0x7fff);
            if (pkey_idx < 0) {
                OMGT_OUTPUT_ERROR(port, "Failed to find 0x7fff pkey defaulting to index 1\n");
                pkey_idx = 1;
            } else {
               OMGT_DBGPRINT(port, "... using 0x7fff found at index %d\n", pkey_idx);
            }
        } else {
            // Previously, this code would try to find the limited management pkey 
            //   if it could not find the requested pkey, and use that pkey instead.
            // This would often "work" because all nodes should have the limited
            //   management pkey, but b/c it was a limited member, this would result
            //   in potential timeouts - especially where the full management pkey was
            //   required.
            // Changed this code fail immediately without retrying a new pkey.
            OMGT_OUTPUT_ERROR(port, "Failed to find requested pkey:0x%x, class 0x%x aid:0x%x \n",
                         addr->pkey, mclass, ntohs(mad_hdr->attr_id));
            status = FPROTECTION;
            goto done;
        }
    }
    umad_set_pkey(umad_p, pkey_idx);

    umad_set_addr(umad_p, ib_lid, addr->qpn, addr->sl, addr->qkey);

    correctedTimeout = (timeout_ms == OMGT_SEND_TIMEOUT_DEFAULT)
                     ? OMGT_DEF_TIMEOUT_MS : timeout_ms;

    if (port->dbg_file) {
        OMGT_DBGPRINT(port, ">>> sending: len %ld pktsz %zu\n", send_size, umad_size() + padded_size);
		umad_dump(umad_p);
        omgt_dump_mad(port->dbg_file, umad_get_mad(umad_p), send_size, "send mad\n");
    }

	if (umad_send(port->umad_fd, aid, umad_p, padded_size, (response ? 0 : correctedTimeout), retries) < 0) {
		OMGT_OUTPUT_ERROR(port, "send failed; %s, agent id %u MClass 0x%x method 0x%x attrId 0x%x attrM 0x%x\n",
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
FSTATUS omgt_recv_mad_alloc(struct omgt_port *port, uint8_t **recv_mad, size_t *recv_size,
			int timeout_ms, struct omgt_mad_addr *addr)
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
		OMGT_OUTPUT_ERROR(port, "can't alloc MAD sized umad\n");
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
			OMGT_DBGPRINT(port, "recv error on MAD sized umad (%s) length=%ld\n",
				strerror(errno), length);
			if (errno == EINTR)
				goto retry;

			status = (errno == ETIMEDOUT) ? FNOT_DONE : FERROR;
			goto done;
		} else {
			/* Need a larger buffer for RMPP */
			OMGT_DBGPRINT(port, "Received 1st MAD length=%ld\n", length);
			umad_free(umad);

			umad = umad_alloc(1, umad_size() + length);
			if (!umad) {
				OMGT_OUTPUT_ERROR(port, "can't alloc umad length %ld\n", length);
				status = FINSUFFICIENT_MEMORY;
				goto done;
			}

			// just to be safe, we supply a timeout.  However it
			// should be unnecessary since we know we have a packet
retry2:
			if ((mad_agent = umad_recv(port->umad_fd, umad, (int *)&length, OMGT_DEF_TIMEOUT_MS)) < 0) {
				OMGT_OUTPUT_ERROR(port, "recv error on umad length %ld (%s)\n", length, strerror(errno));
				if (errno == EINTR)
					goto retry2;

				status = FOVERRUN;
				*recv_size = length;
				goto done;
			}

			/*
			 * PA RMPP MADs need to be handled differently as the Kernel
			 * RMPP module reassembles the PA MADs believing the header to
			 * only be 40 bytes (MAD[24] + RMPP[12] + OUI[4]). However, OPA uses
			 * 56 bytes (MAD[24] + RMPP[12] + PA_HEADER[20]), like the SA Class.
			 *
			 * As a result the extra 16 bytes ends up in the payload returned by
			 * umad_recv() before every segment (packet). Here is where we
			 * remove the extra 16 bytes per packet segment.
			 */
			uint8_t mgmt_class = ((SA_MAD *)umad_get_mad(umad))->common.MgmtClass;
			if (mgmt_class == MCLASS_VFI_PM) {
#define RMPP_VENDOR_HDR_LEN 40
#define RMPP_PA_HDR_LEN 56
#define RMPP_PA_DATA_OFFSET (RMPP_PA_HDR_LEN - RMPP_VENDOR_HDR_LEN)

				SA_MAD *rcv_mad = (SA_MAD *)umad_get_mad(umad);
				size_t final_len, len = length;
				/* Get the number of full packet segments in the response */
				int i, full_segments = (len - RMPP_VENDOR_HDR_LEN) / (STL_SUBN_ADM_DATASIZE + RMPP_PA_DATA_OFFSET);
				/* Get Size of Final Segment if it is not a full segment (size != 1992 + 16) */
				size_t last_seg =
					((len - RMPP_VENDOR_HDR_LEN) %
						(STL_SUBN_ADM_DATASIZE + RMPP_PA_DATA_OFFSET)) <= RMPP_PA_DATA_OFFSET ? 0 :
					((len - RMPP_VENDOR_HDR_LEN) %
						(STL_SUBN_ADM_DATASIZE + RMPP_PA_DATA_OFFSET)) - RMPP_PA_DATA_OFFSET;

				/* Using Segments find the size of what the real payload should be */
				final_len = RMPP_PA_HDR_LEN + (full_segments * STL_SUBN_ADM_DATASIZE) + last_seg;

				OMGT_DBGPRINT(port, "PA Mad RMPP Adjustment:\n");
				OMGT_DBGPRINT(port, " Length:  Before %zu v. After %zu\n", len, final_len);
				OMGT_DBGPRINT(port, " Segments: %u%s\n", full_segments, (last_seg ? " + 1" : ""));

				uint8_t *new_data = (uint8_t *)umad_alloc(1, umad_size() + final_len);
				if (new_data == NULL) {
					OMGT_OUTPUT_ERROR(port, "error allocating query result buffer\n");
					status = FINSUFFICIENT_MEMORY;
					goto done;
				}
				/* Copy UMAD and Class Header Data */
				memcpy(new_data, (uint8_t *)umad, umad_size() + RMPP_PA_HDR_LEN);
				/* Move old_ptr to start of data past the first Header */
				uint8_t *old_ptr = ((uint8_t *)rcv_mad) + RMPP_PA_HDR_LEN;
				uint8_t *new_ptr = new_data + umad_size() + RMPP_PA_HDR_LEN;

				for (i = 0; i < full_segments; i++) {
					/* Copy Full Segments */
					memcpy(new_ptr, old_ptr, STL_SUBN_ADM_DATASIZE);
					/* Remove 16 byte offset from old segement */
					old_ptr += RMPP_PA_DATA_OFFSET;
					/* Move Pointers */
					old_ptr += STL_SUBN_ADM_DATASIZE;
					new_ptr += STL_SUBN_ADM_DATASIZE;
				}
				/* Copy Last Segment if not full. old_ptr updated above */
				if (last_seg) {
					memcpy(new_ptr, old_ptr, last_seg);
				}
				/* Clean up Old Data*/
				free(umad);
				/* Copy New_Data to umad pointer */
				umad = (ib_user_mad_t *)new_data;
				/* Update Length */
				length = final_len;
				umad->length = final_len + umad_size();
			}
		}
	}

    if (mad_agent >= UMAD_CA_MAX_AGENTS) { 
        OMGT_OUTPUT_ERROR(port, "invalid mad agent %d - dropping\n", mad_agent);
        status = FERROR;
        goto done;
    }

    my_umad_status = umad_status(umad);
    OMGT_DBGPRINT(port, "UMAD Status: %s (%d)\n", strerror(my_umad_status), my_umad_status);
	if (my_umad_status != 0) {
		status = (my_umad_status == ETIMEDOUT) ? FTIMEOUT : FREJECT;
	}

    OMGT_DBGPRINT(port, "Received MAD length=%ld, total umad size=%ld\n",length, length + umad_size());

    if (port->dbg_file) {
        struct umad_hdr * umad_hdr = (struct umad_hdr *)umad_get_mad(umad);
        OMGT_DBGPRINT(port, "  Base_Version 0x%x Class 0x%x Method 0x%x attrId 0x%x attr_mod 0x%x status 0x%x\n",
                 umad_hdr->base_version, umad_hdr->mgmt_class, umad_hdr->method,
                 umad_hdr->attr_id, umad_hdr->attr_mod, umad_hdr->status);
        umad_dump(umad);
        omgt_dump_mad(port->dbg_file, umad_get_mad(umad), length, "rcv mad\n");
        
    }

    // Allocate and copy to new buffer.
    *recv_mad = calloc (1, length);

    if (*recv_mad == NULL) {
        OMGT_OUTPUT_ERROR(port, "can't alloc return buffer length %ld\n", length);
        status = FINSUFFICIENT_MEMORY;
        goto done;
    }

    memcpy (*recv_mad, umad_get_mad(umad), length); 
	*recv_size = length;

	if (addr != NULL) {
		addr->lid = omgt_extract_lid(umad);
		addr->sl   = umad->addr.sl;
		addr->qkey = ntoh32(umad->addr.qkey);
		addr->qpn  = ntoh32(umad->addr.qpn);
		addr->pkey = omgt_find_pkey_from_idx(port, umad_get_pkey(umad));
	}

done:
    if (umad != NULL) {
        umad_free(umad);
    }
    return status;
}

/** ========================================================================= */
FSTATUS omgt_recv_mad_no_alloc(struct omgt_port *port, uint8_t *recv_mad, size_t *recv_size,
			int timeout_ms, struct omgt_mad_addr *addr)
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
        OMGT_OUTPUT_ERROR(port, "can't alloc umad length %ld\n", length);
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
            OMGT_DBGPRINT(port, "recv error on umad (size %zu) (%s)\n", *recv_size,
			  strerror(errno));
			if (errno == EINTR)
				goto retry;

            status = (errno == ETIMEDOUT) ? FNOT_DONE:FERROR;
            goto done;
        } else {
			// this routine is not expecting large responses
            OMGT_OUTPUT_ERROR(port, "Rx Packet size %zu larger than mad-size %zu\n",
						  length, *recv_size);
            status = FOVERRUN;
			if (recv_mad)
                memcpy(recv_mad, umad_get_mad(umad), *recv_size);

            // Clean out Rx packet 'cause it will never go away..
            umad_free(umad);
            umad = umad_alloc(1, umad_size() + length);
            if (!umad) {
                OMGT_OUTPUT_ERROR(port, "can't alloc umad for rx cleanup, length %ld\n", length);
                status = FINSUFFICIENT_MEMORY;
                goto done;
            }

			// just to be safe, we supply a timeout.  However it
			// should be unnecessary since we know we have a packet
retry2:
            if (umad_recv(port->umad_fd, umad, (int *)&length, OMGT_DEF_TIMEOUT_MS) < 0) {
                OMGT_OUTPUT_ERROR(port, "recv error on cleanup, length %ld (%s)\n", length,
			      strerror(errno));
				if (errno == EINTR)
					goto retry2;

                goto done;
            }

			if (port->dbg_file) {
				umad_dump(umad);
		        omgt_dump_mad(port->dbg_file, umad_get_mad(umad), length, "rcv mad discarded\n");
			}
            goto done;
        }
    }
    if (mad_agent >= UMAD_CA_MAX_AGENTS) { 
        OMGT_OUTPUT_ERROR(port, "invalid mad agent %d\n", mad_agent);
        status = FERROR;
        goto done;
    }

    my_umad_status = umad_status(umad);
    OMGT_DBGPRINT(port, "UMAD Status: %s (%d)\n", strerror(my_umad_status), my_umad_status);
	if (my_umad_status != 0) {
        status = (my_umad_status == ETIMEDOUT) ? FTIMEOUT : FREJECT;
    }

    OMGT_DBGPRINT(port, "Received MAD: Agent %d, length=%ld\n", mad_agent, length);
    if (port->dbg_file) {
        umad_dump(umad);
        omgt_dump_mad(port->dbg_file, umad_get_mad(umad), length, "rcv mad\n");
    }

    // Copy the data
    if (recv_mad && length > 0) {
        *recv_size = length;
        memcpy(recv_mad, umad_get_mad(umad), length);
    }

	if (addr != NULL) {
		addr->lid = omgt_extract_lid(umad);
		addr->sl   = umad->addr.sl;
		addr->qkey = ntoh32(umad->addr.qkey);
		addr->qpn  = ntoh32(umad->addr.qpn);
		addr->pkey = omgt_find_pkey_from_idx(port, umad_get_pkey(umad));
	}

done:
    if (umad != NULL) {
        umad_free(umad);
    }
    return status;
}

/** =========================================================================
FSTATUS omgt_get_portguid(
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
omgt_get_pa_mad_status(
    IN struct omgt_port              *port
    )
{
    return port->pa_mad_status;
}
