/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include "ib_types.h"
#include "sm_l.h"

//---------------------------------------------------------------------------//

static void sm_dispatch_free_req(sm_dispatch_req_t *req);
static void sm_dispatch_cntxt_callback(cntxt_entry_t *, Status_t, void *, Mai_t *);
static Status_t sm_dispatch_send(sm_dispatch_req_t *req);
static Status_t sm_dispatch_passthrough(sm_dispatch_req_t *req);

//---------------------------------------------------------------------------//

static void sm_dispatch_free_req(sm_dispatch_req_t *req)
{
	vs_pool_free(&sm_pool, (void *)req);
}

// called with sm_async_send_rcv_cntxt.lock held
static void sm_dispatch_cntxt_callback(cntxt_entry_t *cntxt, Status_t cntxtStatus, void *data, Mai_t *mad)
{
	sm_dispatch_req_t *req = (sm_dispatch_req_t *)data;
	sm_dispatch_t *disp = req->disp;
	Node_t *nodep = NULL;
	uint64_t now;

	if (req->sweepPasscount == disp->sweepPasscount) {
		nodep = req->nodep;
	} else {
		IB_LOG_WARN_FMT(__func__,
			"Stale MAD response callback for LID 0x%x, ATTR %d, AMOD 0x%08x",
              	cntxt->lid, cntxt->mad.base.aid, cntxt->mad.base.amod);
	}

	if (cntxtStatus == VSTATUS_TIMEOUT) {
		vs_time_get(&now);
		sm_popo_report_timeout(&sm_popo, MAX(0, now - req->sendTime));
	}

	if (cntxt && cntxt->mad.base.aid == MAD_SMA_LFT) {
		if (cntxtStatus == VSTATUS_TIMEOUT) {
			if (nodep) {
				IB_LOG_ERROR_FMT(__func__,
				   "Timeout occurred trying to set LFT block for NodeGUID "FMT_U64" [%s] AMOD 0x%08x",
				   nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), cntxt->mad.base.amod);
			} else {
				IB_LOG_ERROR_FMT(__func__,
					"Timeout occurred trying to set LFT block for LID 0x%x, AMOD 0x%08x",
					cntxt->lid, cntxt->mad.base.amod);
			}
			sm_request_resweep(1, 0, SM_SWEEP_REASON_ROUTING_FAIL);
		} else if (cntxtStatus == VSTATUS_OK && (mad && (mad->base.status & MAD_STATUS_MASK))) {
			if (nodep) {
				IB_LOG_ERROR_FMT(__func__,
					"Bad MAD status (%d) while trying to set LFT block for NodeGUID"FMT_U64" [%s] AMOD 0x%08x",
					mad->base.status, nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), cntxt->mad.base.amod);
			} else {
				IB_LOG_ERROR_FMT(__func__,
					"Bad MAD status (%d) while trying to set LFT block for LID 0x%x, AMOD 0x%08x",
                	mad->base.status, cntxt->lid, cntxt->mad.base.amod);
			}

			sm_request_resweep(1, 0, SM_SWEEP_REASON_ROUTING_FAIL);
		}
	} else if (sm_config.check_mft_responses && cntxt && cntxt->mad.base.aid == MAD_SMA_MFT) {
		if (cntxtStatus == VSTATUS_TIMEOUT) {
			if (nodep) {
				IB_LOG_ERROR_FMT(__func__,
				   "Timeout occurred trying to set MFT block for NodeGUID "FMT_U64" [%s] AMOD 0x%08x",
				   nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), cntxt->mad.base.amod);
			} else {
				IB_LOG_ERROR_FMT(__func__,
					"Timeout occurred trying to set MFT block for LID 0x%x, AMOD 0x%08x",
					cntxt->lid, cntxt->mad.base.amod);
			}
			sm_request_resweep(0, 1, SM_SWEEP_REASON_MC_ROUTING_FAIL);
		} else if (cntxtStatus == VSTATUS_OK && (mad && (mad->base.status & MAD_STATUS_MASK))) {
			if (nodep) {
				IB_LOG_ERROR_FMT(__func__,
				   "Bad MAD status (%d) while trying to set MFT block for NodeGUID"FMT_U64" [%s] AMOD 0x%08x",
				   mad->base.status, nodep->nodeInfo.NodeGUID,
				   sm_nodeDescString(nodep), cntxt->mad.base.amod);
			} else {
				IB_LOG_ERROR_FMT(__func__,
					"Bad MAD status (%d) while trying to set MFT block for LID 0x%x, AMOD 0x%08x",
                	mad->base.status, cntxt->lid, cntxt->mad.base.amod);
			}

			sm_request_resweep(0, 1, SM_SWEEP_REASON_MC_ROUTING_FAIL);
		}
	}

//	IB_LOG_INFINI_INFO0("received ack");

	if (req->disp->reqsOutstanding) --req->disp->reqsOutstanding;
	if (nodep && nodep->asyncReqsOutstanding) --nodep->asyncReqsOutstanding;

	sm_dispatch_free_req(req);

	if (QListIsEmpty(&disp->queue) && !disp->reqsOutstanding) {
//		IB_LOG_INFINI_INFO_FMT(__func__, "queue empty at %d outstanding, queue length %d",
//		       disp->reqsOutstanding, QListCount(&disp->queue));
		vs_event_post(&disp->evtEmpty, VEVENT_WAKE_ALL, (Eventset_t)1u);
	}
}

// can only be called under lock from the topology_rcv thread
static Status_t sm_dispatch_send(sm_dispatch_req_t *req)
{
	vs_time_get(&req->sendTime);

    if (req->sendParams.bversion == STL_BASE_VERSION) {
        SmpAddr_t addr = SMP_ADDR_CREATE(req->sendParams.path, req->sendParams.slid, req->sendParams.dlid);
        return sm_send_stl_request_impl(
           req->sendParams.fd, req->sendParams.method, req->sendParams.aid, 
           req->sendParams.amod, &addr,
           req->sendParams.bufferLength, req->sendParams.buffer, &req->sendParams.bufferLength, 
           RCV_REPLY_AYNC, req->sendParams.mkey, sm_dispatch_cntxt_callback, 
           (void *)req, NULL);
    } else {
		return VSTATUS_BAD;
    }
}

// caller already has context lock
static Status_t sm_dispatch_passthrough(sm_dispatch_req_t *req)
{
	Status_t status;

    if (req->sendParams.bversion == STL_BASE_VERSION) {
        SmpAddr_t addr = SMP_ADDR_CREATE(req->sendParams.path, req->sendParams.slid, req->sendParams.dlid);
        status = sm_send_stl_request_impl(
           req->sendParams.fd, req->sendParams.method, req->sendParams.aid, 
           req->sendParams.amod, &addr,
           req->sendParams.bufferLength, req->sendParams.buffer, &req->sendParams.bufferLength,
           RCV_REPLY_AYNC, req->sendParams.mkey, NULL, NULL, NULL);
    } else {
		status = VSTATUS_BAD;
    }

	sm_dispatch_free_req(req);
	return status;
}

Status_t sm_dispatch_init(sm_dispatch_t *disp, uint32_t reqsSupported)
{
	Status_t status;

	if (disp->initialized) {
		IB_LOG_WARN0("dispatcher not previously destroyed properly");
		sm_dispatch_destroy(disp);
	}

	memset(disp, 0, sizeof(*disp));

	status = vs_event_create(&disp->evtEmpty, (uint8_t*)"SmDispatch", (Eventset_t)0u);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("failed to initialize event, rc:", status);
		return status;
	}

	QListInit(&disp->queue);

	disp->reqsSupported = reqsSupported;
	disp->initialized = 1;

	return VSTATUS_OK;
}

void sm_dispatch_destroy(sm_dispatch_t *disp)
{
	Status_t status;

	if (!disp->initialized)
		return;

	QListRemoveAll(&disp->queue);
	disp->initialized = 0;

	status = vs_event_delete(&disp->evtEmpty);
	if (status != VSTATUS_OK)
		IB_LOG_WARNRC("failed to delete event, rc:", status);

	QListDestroy(&disp->queue);

	memset(disp, 0, sizeof(*disp));
}

Status_t sm_dispatch_new_req(
	sm_dispatch_t *disp, sm_dispatch_send_params_t *sendParams,
	Node_t *nodep, sm_dispatch_req_t **outReq)
{
	Status_t status;
	sm_dispatch_req_t *req;

	// probably need a simple block allocator, as one-off
	// allocs are expensive at large scales
	status = vs_pool_alloc(&sm_pool, sizeof(sm_dispatch_req_t), (void *)&req);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"failed to allocate dispatch request (rc %d) for fd 0x%08"PRIxN", method 0x%02x, aid 0x%04x\n",
			status, sendParams->fd, sendParams->method, sendParams->aid);
		return status;
	}

	memcpy(&req->sendParams, sendParams, sizeof(req->sendParams));
	req->nodep = nodep;
	req->disp = disp;
	req->sweepPasscount = disp->sweepPasscount;
	QListSetObj(&req->item, req);

	*outReq = req;

	return VSTATUS_OK;
}

Status_t sm_dispatch_enqueue(sm_dispatch_req_t *req)
{
	Status_t status;
	sm_dispatch_t *disp = req->disp;

	cs_cntxt_lock(&sm_async_send_rcv_cntxt);

	if (!disp->initialized) {
		status = sm_dispatch_passthrough(req);
		cs_cntxt_unlock(&sm_async_send_rcv_cntxt);
		return status;
	}

//	IB_LOG_INFINI_INFO_FMT(__func__,
//	       "enqueuing (dispatcher %d, node %d)",
//	       disp->reqsOutstanding, req->nodep->asyncReqsOutstanding);
	QListInsertTail(&disp->queue, &req->item);

	cs_cntxt_unlock(&sm_async_send_rcv_cntxt);

	return VSTATUS_OK;
}

Status_t sm_dispatch_wait(sm_dispatch_t *disp)
{
	Status_t status;
	Eventset_t es;
	uint8_t savedCount = 0;
	uint64_t save = 0;

//	IB_LOG_INFINI_INFO_FMT(__func__, "waiting at %d outstanding, queue length %d",
//	       disp->reqsOutstanding, QListCount(&disp->queue));

	while (1) {
		// calls are made when no requests were sent.
		// due to a race, the signal may be stale.
		// check the exit condition manually prior to and after waiting.
		cs_cntxt_lock(&sm_async_send_rcv_cntxt);
		if (QListIsEmpty(&disp->queue) && !disp->reqsOutstanding) {
//			IB_LOG_INFINI_INFO_FMT(__func__, "done (1) at %d outstanding, queue length %d",
//			       disp->reqsOutstanding, QListCount(&disp->queue));
			cs_cntxt_unlock(&sm_async_send_rcv_cntxt);
			return VSTATUS_OK;
		} else if (!QListIsEmpty(&disp->queue)) {
			// track now many times in a row we've noticed that the dispatcher
			// state has not changed, and once at a threshold, abort
			if (save == QListCount(&disp->queue)) {
				if (savedCount++ == SM_DISPATCH_STALL_THRESHOLD) {
//					IB_LOG_INFINI_INFO_FMT(__func__, "aborting (no progress) at %d outstanding, queue length %d",
//					       disp->reqsOutstanding, QListCount(&disp->queue));
					cs_cntxt_unlock(&sm_async_send_rcv_cntxt);
					return VSTATUS_TIMEOUT;
				}
			} else {
				savedCount = 1;
				save = QListCount(&disp->queue);
			}
//			IB_LOG_INFINI_INFO_FMT(__func__, "still waiting at %d outstanding, queue length %d",
//			       disp->reqsOutstanding, QListCount(&disp->queue));
		}
		cs_cntxt_unlock(&sm_async_send_rcv_cntxt);

		// wait briefly for a signal
		status = vs_event_wait(&disp->evtEmpty, SM_DISPATCH_EVENT_TIMEOUT, (Eventset_t)1u, &es);
		if (status == VSTATUS_OK) {
//			IB_LOG_INFINI_INFO_FMT(__func__, "signaled at %d outstanding, queue length %d",
//			       disp->reqsOutstanding, QListCount(&disp->queue));
			// PRs - 118838 - fix hsm crash
			// Potential race condition with wait/post, clear event may be stale.
			// Fall thru so that we check if request processing actually complete.
		} else if (status != VSTATUS_TIMEOUT) {
			IB_LOG_ERRORRC("failed to wait on event, rc:", status);
			return status;
		}
	}

	return VSTATUS_BAD;
}

void sm_dispatch_bump_passcount(sm_dispatch_t *disp) 
{
	cs_cntxt_lock(&sm_async_send_rcv_cntxt);

	++disp->sweepPasscount;

	cs_cntxt_unlock(&sm_async_send_rcv_cntxt);
}

static void _dispatch_clear_unsafe(sm_dispatch_t *disp)
{
	LIST_ITEM *item;
	sm_dispatch_req_t *req;

	item = QListRemoveHead(&disp->queue);
	while (item != NULL) {
		req = (sm_dispatch_req_t *)QListObj(item);
		sm_dispatch_free_req(req);
		item = QListRemoveHead(&disp->queue);
	}
}

void sm_dispatch_clear(sm_dispatch_t *disp)
{
	cs_cntxt_lock(&sm_async_send_rcv_cntxt);
	_dispatch_clear_unsafe(disp);
	cs_cntxt_unlock(&sm_async_send_rcv_cntxt);
}

Status_t sm_dispatch_update(sm_dispatch_t *disp)
{
	LIST_ITEM *item;
	sm_dispatch_req_t *req;

	if (QListIsEmpty(&disp->queue))
		return VSTATUS_OK;

	cs_cntxt_lock(&sm_async_send_rcv_cntxt);

	if (QListIsEmpty(&disp->queue)) {
		cs_cntxt_unlock(&sm_async_send_rcv_cntxt);
		return VSTATUS_OK;
	}

	if (sm_popo_should_abandon(&sm_popo)) {
		_dispatch_clear_unsafe(disp);
		cs_cntxt_unlock(&sm_async_send_rcv_cntxt);
		return VSTATUS_OK;
	}

	if (disp->reqsOutstanding >= disp->reqsSupported) {
		cs_cntxt_unlock(&sm_async_send_rcv_cntxt);
		IB_LOG_VERBOSE_FMT(__func__, "maximum requests outstanding");
		return VSTATUS_OK;
	}

	item = QListHead(&disp->queue);
	while (item != NULL && disp->reqsOutstanding < disp->reqsSupported) {
		req = (sm_dispatch_req_t *)QListObj(item);
		item = QListNext(&disp->queue, item);
		if ((disp->sweepPasscount != req->sweepPasscount) ||
			(req->nodep->asyncReqsOutstanding < req->nodep->asyncReqsSupported)) {
			IB_LOG_VERBOSE_FMT(__func__, "servicing incoming queue");
			QListRemoveItem(&disp->queue, &req->item);
			++disp->reqsOutstanding;
			if (disp->sweepPasscount == req->sweepPasscount) ++req->nodep->asyncReqsOutstanding;
			sm_dispatch_send(req);
		}
	}

	cs_cntxt_unlock(&sm_async_send_rcv_cntxt);
	return VSTATUS_OK;
}

