/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2018, Intel Corporation

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

//======================================================================//
//                                                                      //
// FILE NAME                                                            //
//    sm_update_fields                                                  //
//                                                                      //
// DESCRIPTION                                                          //
//    This file contains SM QoS routines for getting up SL/SC/VL        //
//    mapping and VL Arbitration tables.                                //
//                                                                      //
//======================================================================//

#include "os_g.h"
#include "ib_status.h"
#include "sm_l.h"
#include "sm_qos.h"
#include "sm_update_fields.h"

typedef struct {
    ParallelWorkItem_t item;
    Node_t *nodep;
} UpdateFieldsWorkItem_t;

/**
    Do LR Get(Aggregate) of attributes on @c nodep with values from SMA at @c smaportp.

    See @fn sm_node_updateFields() for interface details.

    @return VSTATUS_OK only if all operations succeed, non-OK otherwise.  Will return VSTATUS_BAD if an individual aggregate-segment operation failed.
*/

static Status_t
_node_update_from_sma_aggregate(SmMaiHandle_t *fd, ParallelSweepContext_t *psc, STL_LID slid, Node_t * nodep, Port_t * smaportp);

/**
    Fallback code for SMAs that don't support aggregate operations.

    See @fn sm_node_updateFields() for interface details.
*/
static Status_t
_node_update_from_sma_solo(SmMaiHandle_t *fd, ParallelSweepContext_t *psc, STL_LID slid, Node_t * nodep, Port_t * smaportp);

/**
    Update @c nodep SLSC, SCSL, SCVLt, and SCVLnt values from @c nodep found in @c srcTopop (usually topology from prior sweep).

    @return VSTATUS_OK on success (even if not updated), something else if things go really wrong.
*/
static Status_t
_node_update_from_topo(Node_t * nodep, Topology_t * oldTopop, Topology_t * curTopop);

/**
  @return Number of aggregate segments required to send @c n blocks of size @c s per block
*/
static inline size_t
_req_aggr_seg_count(size_t n, size_t s)
{
    if (s == 0) return 0;
    size_t blksPerSeg = STL_MAX_PAYLOAD_AGGREGATE/s;
    return (n + (blksPerSeg - 1))/blksPerSeg;
}

/**
  @return Amount of memory required to do aggregate send @c n blocks of size @c s per block
*/
static inline size_t
_req_aggr_seg_mem(size_t n, size_t s)
{
    size_t segsReq = _req_aggr_seg_count(n, s);
    return segsReq * sizeof(STL_AGGREGATE) + n * s;
}

static Status_t
_node_update_from_sma_aggregate(SmMaiHandle_t *fd, ParallelSweepContext_t *psc, STL_LID slid, Node_t * nodep, Port_t * smaportp)
{
    Status_t s = VSTATUS_BAD;

    uint8 startPort, numPorts;

    startPort = smaportp->index;

    switch (nodep->nodeInfo.NodeType) {
        case NI_TYPE_CA:
            numPorts = 1;
            break;
        case NI_TYPE_SWITCH:
            numPorts = nodep->nodeInfo.NumPorts + 1;
            break;
        default:
            return VSTATUS_BAD;
    }

    if (!sm_valid_port(smaportp)) {
        IB_LOG_ERROR_FMT(__func__,
            "Failed to get SMA port for node %s, node GUID "FMT_U64", port %d",
            sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, smaportp->index);
        return VSTATUS_BAD;
    }

    size_t reqMem =
        (sizeof(STL_AGGREGATE) + sizeof(STL_SLSCMAP)) +
        (sizeof(STL_AGGREGATE) + sizeof(STL_SCSLMAP)) +
        3 * _req_aggr_seg_mem(numPorts, sizeof(STL_SCVLMAP)) + // 3 * -> SCVLt and SCVLnt and SCVLr

        // Not optimal; in the worst case, can do 2 VLArb blocks/segment
        4 * (numPorts * (sizeof(STL_AGGREGATE) + sizeof(STL_VLARB_TABLE))) + // vlarbLow, vlarbHigh, and preempt matrix/table use the same wire-size structure even though they are not the same size
        _req_aggr_seg_mem(numPorts, STL_BFRCTRLTAB_PAD_SIZE);

    Port_t * portp = NULL;
    boolean getScvlt = FALSE;
    boolean getScvlnt = FALSE;
    boolean getScvlr = FALSE;
    boolean getVlarbLow = FALSE;
    boolean getVlarbHigh = FALSE;
    boolean getVlarbPre = FALSE;
    boolean getVlarbMatrix = FALSE;
    boolean getBufferCtrl = FALSE;

    boolean getSlsc = !smaportp->portData->current.slsc;
    boolean getScsl = !smaportp->portData->current.scsl;
    {
        uint8 i;
        for (i = 0; i < numPorts; ++i) {
            portp = sm_get_port(nodep, startPort + i);
            if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                continue;

            getScvlt |= !portp->portData->current.scvlt;
            getScvlnt |= !portp->portData->current.scvlnt;
            getScvlr |= (!portp->portData->current.scvlr && smaportp->portData->portInfo.CapabilityMask3.s.IsVLrSupported);
            getBufferCtrl |= !portp->portData->current.bfrctrl;

            if (portp->portData->vl0 > 1) {
                if (nodep->vlArb) {
                    getVlarbHigh |= !portp->portData->current.vlarbHigh;
                    getVlarbLow |=  !portp->portData->current.vlarbLow;
                    getVlarbPre |=  !portp->portData->current.vlarbPre;
                    getVlarbMatrix |= !portp->portData->current.vlarbMatrix;

                }
            }

            if (getScvlt && getScvlnt &&
                (getScvlr || !smaportp->portData->portInfo.CapabilityMask3.s.IsVLrSupported) &&
                (getVlarbLow && getVlarbHigh && getVlarbPre && getVlarbMatrix) &&
                getBufferCtrl)
                break;
        }
    }

    if (!getScvlt && !getScvlnt && !getScvlr &&
        !getVlarbLow && !getVlarbHigh && !getVlarbPre && !getVlarbMatrix &&
        !getBufferCtrl && !getSlsc && !getScsl) {
        // Nothing to update
        return VSTATUS_OK;
    }

    STL_AGGREGATE *aggrBuffer;
    vs_pool_alloc(&sm_pool, reqMem, (void*)&aggrBuffer);
    if (!aggrBuffer)
        return VSTATUS_BAD;

    memset(aggrBuffer, 0, reqMem);
    STL_AGGREGATE *segHdr = aggrBuffer;
    size_t lastSegReqLen = 0;

    const struct s_aggrList { boolean get; size_t size; uint32_t aid; boolean multiport; } aggrList[] = {
        {getScvlt, sizeof(STL_SCVLMAP), STL_MCLASS_ATTRIB_ID_SC_VLT_MAPPING_TABLE, TRUE},
        {getScvlnt, sizeof(STL_SCVLMAP), STL_MCLASS_ATTRIB_ID_SC_VLNT_MAPPING_TABLE, TRUE},
        {getScvlr, sizeof(STL_SCVLMAP), STL_MCLASS_ATTRIB_ID_SC_VLR_MAPPING_TABLE, TRUE},
        // NOTE: Bfrctrl tables are not 8 byte aligned, so must round up.
        {getBufferCtrl, STL_BFRCTRLTAB_PAD_SIZE, STL_MCLASS_ATTRIB_ID_BUFFER_CONTROL_TABLE, TRUE},
        {getSlsc, sizeof(STL_SLSCMAP), STL_MCLASS_ATTRIB_ID_SL_SC_MAPPING_TABLE, FALSE},
        {getScsl, sizeof(STL_SCSLMAP), STL_MCLASS_ATTRIB_ID_SC_SL_MAPPING_TABLE, FALSE} };
    size_t aggrListIndx = 0;

    do {
        const size_t portsPerSeg = STL_MAX_PAYLOAD_AGGREGATE/aggrList[aggrListIndx].size;
        const size_t segsReq = aggrList[aggrListIndx].multiport ? _req_aggr_seg_count(numPorts, aggrList[aggrListIndx].size) : 1;
        size_t i;
        uint8 port = startPort;
        uint8 blkCount = 1;

        if (!aggrList[aggrListIndx].get) continue;

        for (i = 0; i < segsReq; ++i) {

            if (aggrList[aggrListIndx].multiport) {
                port = (i * portsPerSeg) + startPort;
                blkCount = (segsReq > (i + 1)? portsPerSeg : numPorts - ((numPorts > 1) ? port : 0));
            }

            segHdr->AttributeID = aggrList[aggrListIndx].aid;
            segHdr->Result.s.Error = 0;
            segHdr->Result.s.RequestLength = (blkCount * aggrList[aggrListIndx].size + 7)/8;
            segHdr->AttributeModifier = aggrList[aggrListIndx].multiport ? ( (blkCount << 24) | port ) : 0;
            segHdr = STL_AGGREGATE_NEXT(segHdr);
        }
    } while (++aggrListIndx < (sizeof(aggrList)/sizeof(aggrList[0])) );


    if (nodep->vlArb) {
        const int SEC_COUNT = 4;
        boolean getVlarb[] = { getVlarbHigh, getVlarbLow, getVlarbPre, getVlarbMatrix };
        uint8_t vlarbSec[] = { STL_VLARB_HIGH_ELEMENTS, STL_VLARB_LOW_ELEMENTS, STL_VLARB_PREEMPT_ELEMENTS, STL_VLARB_PREEMPT_MATRIX };
        int i;
        for (i = 0; i < SEC_COUNT; ++i) {
            if (!getVlarb[i])
                continue;

            size_t blkSize = sizeof(STL_VLARB_TABLE);
            size_t portsPerSeg = STL_MAX_PAYLOAD_AGGREGATE/blkSize;

            assert(portsPerSeg > 0);

            uint8 blkCount = 0;

            Port_t * portp, * END_PORT; // END_PORT is one past the end

            if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
                portp = sm_get_port(nodep, PORT_A0(nodep));
                END_PORT = sm_get_port(nodep, PORT_A1(nodep)) + 1;
            }
            else {
                portp = smaportp;
                END_PORT = smaportp + 1;
            }

            Port_t * endPortp = NULL;

            // SMA will return an error for a Get(VLArb) on a port with only one data VL
            // So have to build request selectively
            while (portp && portp != END_PORT) {
                // Find first port for which we can get vlarb data
                while (portp != END_PORT &&
                    (!sm_valid_port(portp) || portp->portData->vl0 <= 1 || portp->state <= IB_PORT_DOWN)) {
                    ++portp;
                }

                if (portp == END_PORT)
                    break;

                // Find up to blkCount contiguous ports that support more than one data VL
                endPortp = portp;
                while (endPortp != END_PORT && blkCount < portsPerSeg &&
                    sm_valid_port(endPortp) && endPortp->portData->vl0 > 1 && portp->state > IB_PORT_DOWN) {
                    ++blkCount;
                    ++endPortp;
                }

                segHdr->AttributeID = STL_MCLASS_ATTRIB_ID_VL_ARBITRATION;
                segHdr->Result.s.Error = 0;
                    segHdr->Result.s.RequestLength = (blkCount * blkSize + 7)/8;
                segHdr->AttributeModifier = (blkCount << 24) | (vlarbSec[i] << 16) | portp->index;
                segHdr = STL_AGGREGATE_NEXT(segHdr);

                blkCount = 0;
                portp = endPortp;
            }
        }

    }

    uint32_t madStatus;
    STL_AGGREGATE * lastSeg = NULL;

    STL_LID dlid = smaportp->portData->lid;
    psc_unlock(psc);
    s = SM_Get_Aggregate_LR(fd, aggrBuffer, segHdr, lastSegReqLen, slid, dlid, &lastSeg, &madStatus);
    psc_lock(psc);

    if (!lastSeg && s == VSTATUS_OK) {
        s = VSTATUS_BAD;
        goto bail;
    }

    if (lastSeg) {
        // Can still process partial aggregate response on MAD error
        Status_t tmpStatus = sm_node_handleGetRespAggr(nodep, smaportp, aggrBuffer, lastSeg);
        s = (s != VSTATUS_OK? s : tmpStatus);
    }

    if (s != VSTATUS_OK || madStatus != MAD_STATUS_SUCCESS) {
        IB_LOG_ERROR_FMT(__func__,
            "Error on Get(Aggregate): NodeGUID : "FMT_U64"; NodeDesc : \"%s\"; port : %d; status : %d; madStatus : 0x%02x",
            nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), smaportp->index, s, madStatus);
        if (lastSeg) {
            IB_LOG_ERROR_FMT(__func__,
                "First error: AttributeID : 0x%02x; AttributeModifier : 0x%08x",
                lastSeg->AttributeID, lastSeg->AttributeModifier);
        }

        // Set s to BAD if it was an SMA error on a particular segment
        if (s == VSTATUS_OK)
            s = VSTATUS_BAD;
        goto bail;
    }

bail:
    vs_pool_free(&sm_pool, aggrBuffer);

    return s;
}

// NOTE:  Why is this done every sweep?
//
static Status_t
_node_update_from_sma_solo(SmMaiHandle_t *fd, ParallelSweepContext_t *psc, STL_LID slid, Node_t * nodep, Port_t * smaportp)
{
    uint32_t amod;
    Status_t s = VSTATUS_OK;
    uint8 buffer[STL_MAX_PAYLOAD_SMP_LR];
    uint8 startPort, numPorts;

    startPort = smaportp->index;

    switch (nodep->nodeInfo.NodeType) {
        case NI_TYPE_CA:
            numPorts = 1;
            break;
        case NI_TYPE_SWITCH:
            numPorts = nodep->nodeInfo.NumPorts + 1;
            break;
        default:
            return VSTATUS_BAD;
    }

    if (!sm_valid_port(smaportp)) {
        IB_LOG_ERROR_FMT(__func__,
            "Failed to get SMA port for node %s, node GUID "FMT_U64", port %d",
            sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, smaportp->index);
        return VSTATUS_BAD;
    }

    STL_LID dlid = smaportp->portData->lid;
    SmpAddr_t addr = SMP_ADDR_CREATE_LR(slid, dlid);

    if (!smaportp->portData->current.slsc
    ) {

        psc_unlock(psc);
        s = SM_Get_SLSCMap(fd, 0, &addr, (STL_SLSCMAP*)buffer);
        psc_lock(psc);
        if (s != VSTATUS_OK)
            return s;

        smaportp->portData->slscMap = *((STL_SLSCMAP*)buffer);
        smaportp->portData->current.slsc = 1;
    }

    if (!smaportp->portData->current.scsl
        ) {
        // Get(SCSL)
        psc_unlock(psc);
        s = SM_Get_SCSLMap(fd, 0, &addr, (STL_SCSLMAP*)buffer);
        psc_lock(psc);
        if (s != VSTATUS_OK)
            return s;
        smaportp->portData->scslMap = *((STL_SCSLMAP*)buffer);
        smaportp->portData->current.scsl = 1;
    }

    boolean getScvlt = FALSE;
    boolean getScvlnt = FALSE;
    boolean getScvlr = FALSE;
    boolean getVlarbLow = FALSE;
    boolean getVlarbHigh = FALSE;
    boolean getVlarbPre = FALSE;
    boolean getVlarbMatrix = FALSE;
    boolean getBufferCtrl= FALSE;

    Port_t * portp = NULL;

    {
        uint8 i;
        for (i = 0; i < numPorts; ++i) {
            portp = sm_get_port(nodep, startPort + i);
            if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                continue;

            getScvlt |= !portp->portData->current.scvlt;
            getScvlnt |= !portp->portData->current.scvlnt;
            getScvlr |= (!smaportp->portData->current.scvlr && portp->portData->portInfo.CapabilityMask3.s.IsVLrSupported);
            getBufferCtrl |= !portp->portData->current.bfrctrl;

            if (portp->portData->vl0 > 1) {
                if (nodep->vlArb) {
                    getVlarbHigh |= !portp->portData->current.vlarbHigh;
                    getVlarbLow |=  !portp->portData->current.vlarbLow;
                    getVlarbPre |=  !portp->portData->current.vlarbPre;
                    getVlarbMatrix |= !portp->portData->current.vlarbMatrix;
                }
            }

            if (getScvlt && getScvlnt &&
                (getScvlr || !smaportp->portData->portInfo.CapabilityMask3.s.IsVLrSupported) &&
                (getVlarbLow && getVlarbHigh && getVlarbPre && getVlarbMatrix) &&
                getBufferCtrl)
                break;
        }
    }

    if (getScvlt) {
        // Get(SCVLt) - fallback code so get ports individually
        uint8 i;
        for (i = 0; i < numPorts; ++i) {
            portp = sm_get_port(nodep, i + startPort);
            if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                continue;

            amod = (1 << 24) | portp->index;

            psc_unlock(psc);
            s = SM_Get_SCVLtMap(fd, amod, &addr, (STL_SCVLMAP*)buffer);
            psc_lock(psc);

            if (s != VSTATUS_OK) {
                IB_LOG_ERROR_FMT(__func__,
                    "Failed to get SCVLt for nodeGUID 0x"FMT_U64", port %d",
                    nodep->nodeInfo.NodeGUID, portp->index);
                //TODO: should continue but make sure non-zero status is returned
            }
            else {
                portp->portData->current.scvlt = 1;
                portp->portData->scvltMap = *((STL_SCVLMAP*)buffer);
            }
        }
    }

    if (getScvlnt) {
        // Get(SCVLnt) - fallback code so get ports individually
        uint8 i;
        for (i = 0; i < numPorts; ++i) {
            portp = sm_get_port(nodep, i + startPort);
            if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                continue;

            amod = (1 << 24) | portp->index;

            psc_unlock(psc);
            s = SM_Get_SCVLntMap(fd, amod, &addr, (STL_SCVLMAP*)buffer);
            psc_lock(psc);

            if (s != VSTATUS_OK) {
                IB_LOG_ERROR_FMT(__func__,
                    "Failed to get SCVLnt for nodeGUID 0x"FMT_U64", port %d",
                    nodep->nodeInfo.NodeGUID, portp->index);
                //TODO: should continue but make sure non-zero status is returned
            }
            else {
                portp->portData->current.scvlnt = 1;
                portp->portData->scvlntMap = *((STL_SCVLMAP*)buffer);
            }
        }
    }

    if (getScvlr) {
        // Get(SCVLr) - fallback code so get ports individually
        uint8 i;
        for (i = 0; i < numPorts; ++i) {
            portp = sm_get_port(nodep, i + startPort);
            if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                continue;

            amod = (1 << 24) | portp->index;

            psc_unlock(psc);
            s = SM_Get_SCVLrMap(fd, amod, &addr, (STL_SCVLMAP*)buffer);
            psc_lock(psc);

            if (s != VSTATUS_OK) {
                IB_LOG_ERROR_FMT(__func__,
                    "Failed to get SCVLr for nodeGUID 0x"FMT_U64", port %d",
                    nodep->nodeInfo.NodeGUID, portp->index);
                //TODO: should continue but make sure non-zero status is returned
            }
            else {
                portp->portData->current.scvlr = 1;
                portp->portData->scvlrMap = *((STL_SCVLMAP*)buffer);
            }
        }
    }

    if (getBufferCtrl) {
        s = sm_get_buffer_control_tables(fd, nodep, 1, PORT_P1(nodep));

        if (s == VSTATUS_OK) {
            Port_t *temp;

            for_all_ports(nodep, temp) {
                if (sm_valid_port(temp))
                    temp->portData->current.bfrctrl = 1;
            }
        }
    }

    if (nodep->vlArb) {
        const int SEC_COUNT = 4;
        boolean getVlarb[] = { getVlarbHigh, getVlarbLow, getVlarbPre, getVlarbMatrix };
        uint8_t vlarbSec[] = { STL_VLARB_HIGH_ELEMENTS, STL_VLARB_LOW_ELEMENTS, STL_VLARB_PREEMPT_ELEMENTS, STL_VLARB_PREEMPT_MATRIX };

        int i;
        for (i = 0; i < SEC_COUNT; ++i) {

            if (!getVlarb[i])
                continue;

            // Wire block size is the same for low, high, and preempt matrix even though
            // internal sizes may be different
            size_t blkSize = sizeof(STL_VLARB_TABLE);

            // SMA will return an error on a multiport request if any of the ports do not
            // support more than one data VL.  So hardcode blksPerMad to '1' for now to avoid this problem
            size_t blksPerMad = 1; // = STL_MAX_PAYLOAD_SMP_LR/blkSize

            uint8 j;
            for (j = 0; j < numPorts; ++j) {
                portp = sm_get_port(nodep, j + startPort);

                if (blksPerMad == 1) {
                    if (!sm_valid_port(portp) || portp->portData->vl0 <= 1 || portp->state <= IB_PORT_DOWN)
                        continue;
                }

                // May have to do multiple blocks for switches to get VLArb values for all ports
                if (j % blksPerMad == 0) {
                    uint8_t blkCount = MIN(numPorts - j, blksPerMad);
                    amod = (blkCount << 24) | (vlarbSec[i] << 16) | ( j + startPort);
                    psc_unlock(psc);
                    s = SM_Get_VLArbitration(fd, amod, &addr, (STL_VLARB_TABLE*)buffer);
                    psc_lock(psc);
                    if (s != VSTATUS_OK) {
                        //TODO: should continue but make sure non-OK status is returned
                        return s;
                    }
                }

                if (!sm_valid_port(portp) || (portp->state <= IB_PORT_DOWN))
                    continue;

                struct _PortDataVLArb * arbp = &portp->portData->curArb;
                uint8_t * dest = NULL;
                size_t cpySize = 0;
                switch (vlarbSec[i]) {
                    case STL_VLARB_LOW_ELEMENTS:
                            dest = (uint8_t*)arbp->u.vlarb.vlarbLow;
                            cpySize = sizeof(arbp->u.vlarb.vlarbLow);
                            portp->portData->current.vlarbLow = 1;
                        break;
                    case STL_VLARB_HIGH_ELEMENTS:
                            dest = (uint8_t*)arbp->u.vlarb.vlarbHigh;
                            cpySize = sizeof(arbp->u.vlarb.vlarbHigh);
                            portp->portData->current.vlarbHigh = 1;
                        break;
                    case STL_VLARB_PREEMPT_ELEMENTS:
                            dest = (uint8_t*)arbp->u.vlarb.vlarbPre;
                            cpySize = sizeof(arbp->u.vlarb.vlarbPre);
                            portp->portData->current.vlarbPre = 1;
                        break;
                    case STL_VLARB_PREEMPT_MATRIX:
                            dest = (uint8_t*)arbp->vlarbMatrix;
                            cpySize = sizeof(arbp->vlarbMatrix);
                            portp->portData->current.vlarbMatrix = 1;
                        break;
                    default:
                        return VSTATUS_BAD;
                }

                memcpy(dest, ((uint8_t*)buffer) + (j%blksPerMad)*blkSize, cpySize);
            }
        }

    }

    return s;
}

static Status_t
_node_update_from_topo(Node_t * nodep, Topology_t * oldTopop, Topology_t * curTopop)
{
    Port_t * portp;

    if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH && !sm_config.use_cached_hfi_node_data)
        return VSTATUS_OK;

    if (&old_topology != oldTopop)
        return VSTATUS_BAD; // only support copying from old_topology now

    // If we have a valid topology, and we believe that nothing has
    // changed, iterate through the ports copying data from the old topo to
    // the new one.
    uint32_t skipWrite = (sm_config.skipAttributeWrite & (SM_SKIP_WRITE_MAPS | SM_SKIP_WRITE_VLARB) ? 1 : 0);

    for_all_physical_ports(nodep, portp) {
        if (!sm_valid_port(portp)) continue;
        if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
            if (!QListInit(&portp->portData->scscMapList[0])) return VSTATUS_BAD;

            if (nodep->switchInfo.CapabilityMask.s.IsExtendedSCSCSupported &&
                curTopop->routingModule->funcs.extended_scsc_in_use()) {
                    if (!QListInit(&portp->portData->scscMapList[1])) return VSTATUS_BAD;
            }
        }
    }

    if ((topology_passcount >= 1) &&
        (!sm_config.forceAttributeRewrite || (sm_config.forceAttributeRewrite == skipWrite)) &&
        old_topology.vfs_ptr && curTopop->vfs_ptr &&
        (old_topology.vfs_ptr->qosEnabled == curTopop->vfs_ptr->qosEnabled)) {

        if (nodep->oldExists) {
            Node_t * oldNodep = nodep->old;
            if (oldNodep && !oldNodep->slscChange) {
                // Already programmed, copy the mappings and return.
                for_all_ports(nodep, portp) {
                    if (!sm_valid_port(portp) || portp->state < IB_PORT_ACTIVE)
                        continue;

                    Port_t * oldPortp = sm_get_port(oldNodep, portp->index);
                    if (!sm_valid_port(oldPortp) || oldPortp->state < IB_PORT_ARMED)
                        continue;

                    // Copy sl, sc, and vl related mapping tables
                    portp->portData->slscMap = oldPortp->portData->slscMap;
                    portp->portData->current.slsc = 1;
                    portp->portData->scslMap = oldPortp->portData->scslMap;
                    portp->portData->current.scsl = 1;
                    portp->portData->scvltMap = oldPortp->portData->scvltMap;
                    portp->portData->current.scvlt = 1;
                    portp->portData->scvlntMap = oldPortp->portData->scvlntMap;
                    portp->portData->current.scvlnt = 1;
                    portp->portData->scvlrMap = oldPortp->portData->scvlrMap;
                    portp->portData->current.scvlr = 1;
                    portp->portData->bufCtrlTable = oldPortp->portData->bufCtrlTable;
                    portp->portData->current.bfrctrl = 1;

                    if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && !QListIsEmpty(&oldPortp->portData->scscMapList[0])) {
                        sm_copyPortDataSCSCMap(oldPortp, portp, 0);
                        portp->portData->current.scsc = 1;

                        if (!QListIsEmpty(&oldPortp->portData->scscMapList[1]))
                            sm_copyPortDataSCSCMap(oldPortp, portp, 1);
                    }

                    if (nodep->vlArb) {
                        //
                        // Copy vlarb
                        memcpy(portp->portData->curArb.u.vlarb.vlarbLow, oldPortp->portData->curArb.u.vlarb.vlarbLow, sizeof(portp->portData->curArb.u.vlarb.vlarbLow));
                        portp->portData->current.vlarbLow = 1;
                        memcpy(portp->portData->curArb.u.vlarb.vlarbHigh, oldPortp->portData->curArb.u.vlarb.vlarbHigh, sizeof(portp->portData->curArb.u.vlarb.vlarbHigh));
                        portp->portData->current.vlarbHigh = 1;
                        memcpy(portp->portData->curArb.u.vlarb.vlarbPre, oldPortp->portData->curArb.u.vlarb.vlarbPre, sizeof(portp->portData->curArb.u.vlarb.vlarbPre));
                        portp->portData->current.vlarbPre = 1;
                        memcpy(portp->portData->curArb.vlarbMatrix, oldPortp->portData->curArb.vlarbMatrix, sizeof(portp->portData->curArb.vlarbMatrix));
                        portp->portData->current.vlarbMatrix = 1;
                    }
                }
            }
        }
    }

    return VSTATUS_OK;
}

static Status_t
_sm_node_updateFields(ParallelSweepContext_t *psc, SmMaiHandle_t *fd, STL_LID slid, Node_t * nodep, Port_t * smaportp)
{
    Status_t s = _node_update_from_topo(nodep, &old_topology, sm_topop);

    if (s != VSTATUS_OK)
        return s;

    if (!sm_valid_port(smaportp))
        return VSTATUS_BAD;

    /*
     * For switches, smaportp refers to port 0.
     */
    if (nodep->aggregateEnable) {
        s = _node_update_from_sma_aggregate(fd, psc, slid, nodep, smaportp);
    }

    // Note that for a multi-port HFI, if one aggregate operation fails and non-aggregate
    // succeeds, aggregates will be disabled for all ports on that HFI
    if (!nodep->aggregateEnable || s != VSTATUS_OK) {
        s = _node_update_from_sma_solo(fd, psc, slid, nodep, smaportp);

        // Aggregate update failed but non-aggregate update succeeded, disable aggregates on this node
        if (nodep->aggregateEnable && s == VSTATUS_OK) {
            IB_LOG_WARN_FMT(__func__,
                "Disabling aggregate support on node %s nodeGUID "FMT_U64,
                sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
            nodep->aggregateEnable = 0;
        }
    }

    return s;
}

static UpdateFieldsWorkItem_t *
_update_node_fields_workitem_alloc(Node_t *nodep, PsWorker_t workFunc)
{
    UpdateFieldsWorkItem_t *workItem = NULL;

    if (vs_pool_alloc(&sm_pool, sizeof(UpdateFieldsWorkItem_t),
        (void **)&workItem) != VSTATUS_OK) {
        return NULL;
    }
    memset(workItem, 0, sizeof(UpdateFieldsWorkItem_t));

    workItem->nodep = nodep;
    workItem->item.workfunc = workFunc;

    return workItem;
}

static void
_update_node_fields_work_item_free(UpdateFieldsWorkItem_t *workitem)
{
    vs_pool_free(&sm_pool, workitem);
}

static void
_update_node_fields_worker(ParallelSweepContext_t *psc, ParallelWorkItem_t *pwi)
{
    Node_t *nodep = NULL;
    Port_t * smaportp = NULL;
    Status_t status = VSTATUS_OK;

    IB_ENTER(__func__, psc, pwi, 0, 0);
    DEBUG_ASSERT(psc && pwi);

    UpdateFieldsWorkItem_t *updateNodeFieldsWorkItem =
        PARENT_STRUCT(pwi, UpdateFieldsWorkItem_t, item);
    nodep = updateNodeFieldsWorkItem->nodep;

    MaiPool_t *maiPoolp = psc_get_mai(psc);

    if (maiPoolp == NULL) {
        IB_LOG_ERROR_FMT(__func__, "Failed to get MAI channel for %s\n",
            sm_nodeDescString(nodep));
        psc_set_status(psc, VSTATUS_NOMEM);
        psc_stop(psc);
        _update_node_fields_work_item_free(updateNodeFieldsWorkItem);
        IB_EXIT(__func__, VSTATUS_NOMEM);
        return;
    }

    psc_lock(psc);

    for_all_sma_ports(nodep, smaportp) {
        if (!sm_valid_port(smaportp) || (smaportp->state <= IB_PORT_DOWN)) {
            continue;
        }

        status = _sm_node_updateFields(psc, maiPoolp->fd, sm_lid, nodep, smaportp);

        if (status != VSTATUS_OK) {
            if (smaportp->index) {
                Port_t *neigh = sm_find_port(sm_topop, smaportp->nodeno, smaportp->portno);
                IB_LOG_WARN_FMT(__func__,
                    "Non-OK status returned on updating node attached to %s port %d via SMA:"
                    "  Node %s, nodeGuid "FMT_U64", port %d, status %d",
                    sm_valid_port(neigh) ? sm_nodeDescString(neigh->portData->nodePtr) : "???",
                    neigh ? neigh->index : -1, sm_nodeDescString(nodep),
                nodep->nodeInfo.NodeGUID, smaportp->index, status);
            } else { // switch port 0; no neighbor
                IB_LOG_WARN_FMT(__func__,
                    "Non-OK status returned on updating node via SMA:"
                    "  Node %s, nodeGuid "FMT_U64", port %d, status %d",
                    sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, smaportp->index, status);
            }

            status = sm_popo_port_error(&sm_popo, sm_topop, smaportp, status);
            if (status == VSTATUS_TIMEOUT_LIMIT) {
                IB_LOG_WARN_FMT(__func__, "Timeout limit getting Partition Table for "
                    "node %s guid "FMT_U64" node index "
                    "%d port index %d", sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
                    nodep->index, smaportp->index);
                goto exit;
            }

            continue;
        }
    }

exit:
    psc_set_status(psc, status);
	// Only abort if we are in an unrecoverable situation.
    if (status == VSTATUS_TIMEOUT_LIMIT || status == VSTATUS_UNRECOVERABLE) {
        psc_stop(psc);
    }
    psc_unlock(psc);
    psc_free_mai(psc, maiPoolp);
    _update_node_fields_work_item_free(updateNodeFieldsWorkItem);
    IB_EXIT(__func__, status);
}

Status_t
sweep_assignments_update_fields(SweepContext_t *sweep_context)
{
    Status_t status = VSTATUS_OK;
    Node_t *nodep;
    UpdateFieldsWorkItem_t *wip;

    IB_ENTER(__func__, 0, 0, 0, 0);

    if (sm_state != SM_STATE_MASTER)
        return VSTATUS_NOT_MASTER;

    psc_go(sweep_context->psc);

    for_all_nodes(sm_topop, nodep) {
        // status = _assignments_update_node_fields(nodep);
        wip = _update_node_fields_workitem_alloc(nodep, _update_node_fields_worker);
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
