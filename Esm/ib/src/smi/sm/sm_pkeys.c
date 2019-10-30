/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2018, Intel Corporation

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

#include "sm_l.h"
#include "sm_pkeys.h"

typedef struct {
	ParallelWorkItem_t item;
	Node_t *nodep;
} GetPkeyWorkItem_t;

static GetPkeyWorkItem_t *
_get_pkey_workitem_alloc(Node_t *nodep, PsWorker_t workFunc)
{
	GetPkeyWorkItem_t *workItem = NULL;

	if (vs_pool_alloc(&sm_pool, sizeof(GetPkeyWorkItem_t),
		(void **)&workItem) != VSTATUS_OK) {
		return NULL;
	}
	memset(workItem, 0, sizeof(GetPkeyWorkItem_t));

	workItem->nodep = nodep;
	workItem->item.workfunc = workFunc;

	return workItem;
}

static void
_get_pkey_work_item_free(GetPkeyWorkItem_t *workitem)
{
	vs_pool_free(&sm_pool, workitem);
}

static void
_get_pkeys_worker(ParallelSweepContext_t *psc, ParallelWorkItem_t *pwi)
{
	uint8_t buffer[STL_MAX_PAYLOAD_SMP_DR] = {0};
	STL_AGGREGATE *aggrHdr = (STL_AGGREGATE*)buffer;
	STL_AGGREGATE *lastSeg;
	Port_t *portp;
	Node_t *nodep;
	size_t pkeyBlkCnt;
	uint8_t pkeyCap;
	bitset_t outstanding;
	boolean aggrFailure = FALSE;
	Status_t status = VSTATUS_OK;

	IB_ENTER(__func__, psc, pwi, 0, 0);
	DEBUG_ASSERT(psc && pwi);

	GetPkeyWorkItem_t *getPkeyWorkItem =
		PARENT_STRUCT(pwi, GetPkeyWorkItem_t, item);
	portp = NULL;
	nodep = getPkeyWorkItem->nodep;

	MaiPool_t *maiPoolp = NULL;

	maiPoolp = psc_get_mai(psc);
	if (maiPoolp == NULL) {
		IB_LOG_ERROR_FMT(__func__, "Failed to get MAI channel for %s\n",
			sm_nodeDescString(nodep));
		_get_pkey_work_item_free(getPkeyWorkItem);
		IB_EXIT(__func__, VSTATUS_BAD);
		return;
	}

	bitset_init(&sm_pool, &outstanding, MAX_STL_PORTS + 1);
	psc_lock(psc);
	Node_t *cache_nodep = sm_find_guid(&old_topology, nodep->nodeInfo.NodeGUID);
	Port_t *cache_portp = NULL;

	for_all_ports(nodep, portp) {
		bool_t skipPort = 0;

		if (!sm_valid_port(portp) || portp->state == IB_PORT_DOWN)
			skipPort = 1;

		if ( (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && portp->index == 0) ||
			 (nodep->nodeInfo.NodeType == NI_TYPE_CA) ) {
			pkeyCap = nodep->nodeInfo.PartitionCap;
		} else {
			/* for switch external ports, use switchInfo->partCap */
			pkeyCap = nodep->switchInfo.PartitionEnforcementCap;
		}

		pkeyBlkCnt = (pkeyCap + NUM_PKEY_ELEMENTS_BLOCK - 1)/ NUM_PKEY_ELEMENTS_BLOCK;

		if (!skipPort)
			memset(portp->portData->pPKey, 0, sizeof(PKey_t) * SM_PKEYS);

		if ((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) &&
			!(portp->index == 0 && nodep->nodeInfo.PartitionCap) &&
			!(portp->index > 0 && nodep->switchInfo.PartitionEnforcementCap))
			skipPort = 1;

		if(!skipPort && sm_find_cached_node_port(nodep, portp, &cache_nodep, &cache_portp) && sm_valid_port(cache_portp)) {
			memcpy(portp->portData->pPKey, cache_portp->portData->pPKey, sizeof(portp->portData->pPKey));
			portp->portData->current.pkeys = 1;

			// Keep going in case we need to flush out Aggregates.
			skipPort = 1;
		}

		// If we decided to skip this port, just process the next one UNLESS we're doing aggregate processing.
		// We may need to send out a completed aggregate stil.
		if (skipPort && !sm_config.use_aggregates)
			continue;

		// Skip Aggregates for HFIs (only one port)
		aggrFailure = aggrFailure || (nodep->nodeInfo.NodeType == STL_NODE_FI);

		if (sm_config.use_aggregates && !aggrFailure) {
			const size_t blockSize = sizeof(STL_AGGREGATE) + 8 * ((sizeof(STL_PARTITION_TABLE) * pkeyBlkCnt + 7)/8);
			const size_t portsPerAggr = (STL_MAX_PAYLOAD_SMP_DR / blockSize);

			if (!skipPort) {
				aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_PART_TABLE;
				aggrHdr->Result.s.Error = 0;
				aggrHdr->Result.s.Reserved = 0;

				aggrHdr->Result.s.RequestLength = ((sizeof(STL_PARTITION_TABLE) * pkeyBlkCnt) + 7)/ 8;
				aggrHdr->AttributeModifier = ((0xff & pkeyBlkCnt)<<24) |
											((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH ? portp->index : 0)<<16);

				aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);

				bitset_set(&outstanding, portp->index);
			}

			if ((portp == sm_get_port(nodep, PORT_A1(nodep)) && bitset_nset(&outstanding) != 0) ||
				bitset_nset(&outstanding) == portsPerAggr) {
				uint32_t madStatus = 0;
				uint8_t cport = bitset_find_first_one(&outstanding);

				lastSeg = NULL;

				psc_unlock(psc);
				status = SM_Get_Aggregate_DR(maiPoolp->fd, (STL_AGGREGATE*)buffer, aggrHdr, 0, nodep->path, &lastSeg, &madStatus);
				psc_lock(psc);

				status = sm_popo_port_error(&sm_popo, sm_topop,
					nodep->nodeInfo.NodeType == NI_TYPE_SWITCH ? sm_get_port(nodep, 0) : portp, status);
				if (status == VSTATUS_TIMEOUT_LIMIT) {
					IB_LOG_WARN_FMT(__func__, "Timeout limit getting Partition Table for "
								"node %s guid "FMT_U64" node index "
								"%d port index %d", sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
								nodep->index, portp->index);
					psc_set_status(psc, VSTATUS_TIMEOUT_LIMIT);
					psc_stop(psc);
					goto exit;
				}

				if (lastSeg) {
					for (aggrHdr = (STL_AGGREGATE*)buffer; aggrHdr < lastSeg; aggrHdr = STL_AGGREGATE_NEXT(aggrHdr)) {
						Port_t *current = sm_get_port(nodep, cport);
						uint8_t i;

						if (!sm_valid_port(current)) {
							// should never happen
							bitset_clear(&outstanding, cport);
							cport = bitset_find_next_one(&outstanding, cport + 1);
							continue;
						}

						if (aggrHdr->Result.s.Error) {
							// Stop processing, rest of this Aggregate is bunk. Reset portp pointer so we can build
							// a new aggregate excluding the failed port.
							bitset_clear(&outstanding, cport);
							portp = current;
							portp->portData->current.pkeys = 0;
							IB_LOG_WARN_FMT(__func__, "Failed to get Partition Table for node %s guid "FMT_U64" node index"
											" %d port index %d", sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
											nodep->index, cport);
							break;
						}

						memcpy(current->portData->pPKey, aggrHdr->Data, sizeof(STL_PARTITION_TABLE) * pkeyBlkCnt);

						for (i = 0; i < pkeyBlkCnt; ++i)
							BSWAP_STL_PARTITION_TABLE((STL_PARTITION_TABLE*)&(current->portData->pPKey[i * NUM_PKEY_ELEMENTS_BLOCK]));

						current->portData->current.pkeys = 1;
						bitset_clear(&outstanding, cport);

						cport = bitset_find_next_one(&outstanding, cport + 1);
					}

					aggrHdr = (STL_AGGREGATE*)buffer;
				} else {
					// The Aggregate itself failed.
					portp = sm_get_port(nodep, cport);
					IB_LOG_WARN_FMT(__func__, "Get(Aggregate) failed for node %s guid "FMT_U64".",
									sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
					aggrFailure = TRUE;
				}
			}
		}

		// Trouble with Aggregates or disabled - try manually.
		if (!skipPort && portp && (aggrFailure || !sm_config.use_aggregates)) {
			uint32_t amod = ((0xff & pkeyBlkCnt)<<24) |
				((nodep->nodeInfo.NodeType == NI_TYPE_SWITCH ? portp->index : 0)<<16);
			boolean isNewError = TRUE;

			SmpAddr_t addr = SMP_ADDR_CREATE_DR(nodep->path);

			if (!sm_popo_is_nonresp_this_sweep(&sm_popo, cache_nodep)) {
				psc_unlock(psc);
				status = SM_Get_PKeyTable(maiPoolp->fd, amod, &addr, (STL_PARTITION_TABLE*)portp->portData->pPKey);
				psc_lock(psc);
			} else {
				// If node was previously nonResp this Sweep, then use cache for Pkeys
				status = VSTATUS_TIMEOUT;
				isNewError = FALSE;
			}
			if (sm_popo_inc_and_use_cache_nonresp(&sm_popo, nodep, &status)
				&& (topology_passcount && cache_nodep
					&& (cache_portp = sm_find_node_port(&old_topology, cache_nodep, portp->index))))
			{
				IB_LOG_INFINI_INFO_FMT(__func__, "%s get Partition Table for node %s guid "FMT_U64
					" node index %d port index %d; using cached data",
					isNewError ? "Failed to" : "Nonresponsive, skipping", sm_nodeDescString(nodep),
					nodep->nodeInfo.NodeGUID, nodep->index, portp->index);

				memcpy(portp->portData->pPKey, cache_portp->portData->pPKey, sizeof(STL_PKEY_ELEMENT) * SM_PKEYS);
				portp->portData->current.pkeys = 1;
				status = VSTATUS_OK;

			} else if (status != VSTATUS_OK) {
				portp->portData->current.pkeys = 0;
				IB_LOG_WARN_FMT(__func__, "Failed to get Partition Table for node %s guid "FMT_U64" node index"
								"%d port index %d", sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
								nodep->index, portp->index);
				if ((status = sm_popo_port_error(&sm_popo, sm_topop, portp, status)) == VSTATUS_TIMEOUT_LIMIT) {
					IB_LOG_WARN_FMT(__func__, "Timeout limit getting Partition Table for "
								"node %s guid "FMT_U64" node index "
								"%d port index %d", sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
								nodep->index, portp->index);
					psc_set_status(psc, VSTATUS_TIMEOUT_LIMIT);
					psc_stop(psc);
					goto exit;
				}
			} else {
				sm_popo_clear_cache_nonresp(&sm_popo, nodep);
				portp->portData->current.pkeys = 1;
			}
		}
	} // for_all_ports

exit:
	psc_unlock(psc);
	bitset_free(&outstanding);

	psc_free_mai(psc, maiPoolp);
	_get_pkey_work_item_free(getPkeyWorkItem);

	IB_EXIT(__func__, status);
}

Status_t
sweep_get_pkey(SweepContext_t *sweep_context)
{
	Status_t status = VSTATUS_OK;
	Node_t *nodep;
	GetPkeyWorkItem_t *wip;

	IB_ENTER(__func__, 0, 0, 0, 0);

	if (sm_state != SM_STATE_MASTER)
		return VSTATUS_NOT_MASTER;

	psc_go(sweep_context->psc);

	for_all_nodes(sm_topop, nodep) {
		wip = _get_pkey_workitem_alloc(nodep, _get_pkeys_worker);
		if (wip == NULL) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(sweep_context->psc, &wip->item);
	}

	if (status == VSTATUS_OK) {
		status = psc_wait(sweep_context->psc);
	} else {
		psc_stop(sweep_context->psc);
		(void)psc_wait(sweep_context->psc);
	}

	psc_drain_work_queue(sweep_context->psc);

	IB_EXIT(__func__, status);

	return status;
}
