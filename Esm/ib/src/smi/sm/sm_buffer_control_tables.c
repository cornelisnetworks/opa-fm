/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									     //
// DESCRIPTION								     //
//   Functions to implement Set/Get and programming of buffer control Tables //
//									     //
//===========================================================================//


#include <stdlib.h>
#include <string.h>

#include "ib_status.h"
#include "os_g.h"
#include "iba/stl_sm_priv.h"
#include "sm_l.h"
#include "sm_parallelsweep.h"

typedef struct {
	ParallelWorkItem_t item;
	Node_t *nodep;
} BctWorkItem_t;

static BctWorkItem_t *
_bctworkitem_alloc(Node_t *nodep, PsWorker_t workfunc)
{
	BctWorkItem_t *workitem = NULL;
	if (vs_pool_alloc(&sm_pool, sizeof(BctWorkItem_t),
		(void**)&workitem) != VSTATUS_OK) {
		return NULL;
	}
	memset(workitem, 0, sizeof(BctWorkItem_t));

	workitem->nodep = nodep;
	workitem->item.workfunc = workfunc;

	return workitem;
}

static void
_bctworkitem_free(BctWorkItem_t *workitem)
{
	vs_pool_free(&sm_pool, workitem);
}

static Status_t
_get_mkey(Node_t *nodep, uint64_t *mkey, uint8_t portNum)
{
	Port_t *portp;
	uint8_t p;
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		p = 0;
	} else {
		p = portNum;
	}
	if ((portp = sm_get_port(nodep, p)) == NULL) {
		cs_log(VS_LOG_ERROR, __func__,
			"Failed to find mkey for for node %s ; port %u",
			sm_nodeDescString(nodep), p);
		return (VSTATUS_NODEV);
	}
	*mkey = (portp->portData->portInfo.M_Key==0) ? sm_config.mkey : portp->portData->portInfo.M_Key;
	return (VSTATUS_OK);
}

Status_t
sm_get_buffer_control_tables(SmMaiHandle_t *fd, Node_t *nodep,
	uint8_t start_port, uint8_t end_port)
{
	int port;
	Status_t status = FERROR;
	uint8_t num_ports;
	STL_BUFFER_CONTROL_TABLE *pbct;
    uint8 maxcount = STL_NUM_BFRCTLTAB_BLOCKS_PER_DRSMP;
	Port_t *portp;

	if (!nodep || !start_port || start_port > end_port) {
		cs_log(VS_LOG_ERROR, __func__,
				"Invalid Parameters for node %s ; %p, %u, %u",
				nodep?sm_nodeDescString(nodep):"<NULL>", nodep, start_port, end_port);
		return (VSTATUS_ILLPARM);
	}


	num_ports = (end_port - start_port) +1;
	pbct = malloc(num_ports * sizeof(STL_BUFFER_CONTROL_TABLE));
	if (!pbct) {
		cs_log(VS_LOG_ERROR, __func__,
				"Failed to allocate buffer control table; node %s; no memory",
				sm_nodeDescString(nodep));
		return (VSTATUS_NOMEM);
	}

	port = nodep->nodeInfo.NodeType == NI_TYPE_CA ? start_port : 0;
	portp = sm_get_port(nodep, port);
	if (!sm_valid_port(portp)) {
		cs_log(VS_LOG_VERBOSE, __func__,
			   "Failed to get Port %d of node %s",
			   port, sm_nodeDescString(nodep));
		free(pbct);
		return VSTATUS_BAD;
	}
	SmpAddr_t addr = SMP_ADDR_CREATE_DR(PathToPort(nodep, portp));

	/*
	 * this could be updated to query only ports which are LinkUp.
	 * But that makes the block algorithm much more difficult and for most
	 * large fabrics you will only be missing a small number of links which
	 * means it is likely just as efficient to query all ports in large ranges.
	 */
	for (port = start_port; port <= end_port; port += maxcount) {
		uint8_t n = MIN(maxcount, (end_port - port)+1);
		uint32_t am = (n << 24) | port;
		STL_BUFFER_CONTROL_TABLE *buf = &pbct[port-start_port];

		status = SM_Get_BufferControlTable(fd, am, &addr, buf);
		if (status != VSTATUS_OK) {
			cs_log(VS_LOG_ERROR, __func__,
					"buffer control table query failed; node %s; %s",
					sm_nodeDescString(nodep),
					cs_convert_status (status));
			goto error;
		}
	}

	for (port = start_port; port <= end_port; port++) {
		portp = sm_get_port(nodep, port);
		if (!sm_valid_port(portp)) {
			cs_log(VS_LOG_VERBOSE, __func__,
				   "Failed to get port %d of node %s",
				   port, sm_nodeDescString(nodep));
			continue;
		}
		memcpy(&portp->portData->bufCtrlTable, &(pbct[port-start_port]),
			sizeof(portp->portData->bufCtrlTable));
	}

error:
	free(pbct);
	return (status);
}


static Status_t
_set_buffer_control_tables(Node_t *nodep, uint8_t start_port, uint8_t end_port,
	STL_BUFFER_CONTROL_TABLE bcts[], uint8_t uniform_bcts,
	ParallelSweepContext_t *psc)
{
	Status_t status = VSTATUS_OK;
    uint8 maxcount = STL_NUM_BFRCTLTAB_BLOCKS_PER_LID_SMP;
	uint16_t port;
	uint8_t num_ports;
	uint64_t mkey;
	uint32_t madStatus;
	extern char * sm_getMadStatusText(uint16_t status);
	Port_t *destPortp;
	uint32_t dlid = 0;

	IB_ENTER(__func__, nodep, start_port, end_port, 0);

	if (!nodep || start_port > end_port) {
		cs_log(VS_LOG_ERROR, __func__,
				"Invalid Parameters for node %s ; %p, ports %u - %u",
				nodep?sm_nodeDescString(nodep):"<NULL>", nodep, start_port, end_port);
		return (VSTATUS_ILLPARM);
	}

	num_ports = (end_port - start_port) +1;

	if (nodep->nodeInfo.NodeType == NI_TYPE_CA && num_ports > 1 ) {
		cs_log(VS_LOG_ERROR, __func__,
			"Failed to Set(BufferControlTable) (HFI %s) num_ports > 1 (%d)",
			sm_nodeDescString(nodep), num_ports);
		return (VSTATUS_ILLPARM);
	}

	if ((status = _get_mkey(nodep, &mkey, start_port)) != VSTATUS_OK)
		return (status);

	port = nodep->nodeInfo.NodeType == NI_TYPE_CA ? start_port : 0;
	destPortp = sm_get_port(nodep, port);
	if (!sm_valid_port(destPortp)) {
		cs_log(VS_LOG_VERBOSE, __func__,
			   "Failed to get Port %d of node %s",
			   port, sm_nodeDescString(nodep));
		return VSTATUS_BAD;
	}

	dlid = destPortp->portData->lid;
	
	MaiPool_t *maip = NULL;
	SmMaiHandle_t * fd = NULL;

	if (psc) maip = psc_get_mai(psc);
	if (maip) fd = maip->fd;
	if (!fd) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate MAI handle.");
		return VSTATUS_NOMEM;
	}

	if (uniform_bcts) {
		// All the ports have identical BCts, so send one block with the all
		// ports bit set.
		uint32_t amod = (1<<24) + (1<<8) + start_port;
		SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);

		psc_unlock(psc);
		status = SM_Set_BufferControlTable(fd, amod, &addr, &bcts[0],
			mkey, &madStatus);
		psc_lock(psc);

		if (status != VSTATUS_OK) {
			cs_log(VS_LOG_ERROR, __func__,
				"buffer control table query failed; node %s; %s (madStatus=%s)",
				sm_nodeDescString(nodep),
				cs_convert_status (status), sm_getMadStatusText(madStatus));
			goto error;
		} else {
			/**
			 * If the Set was successful update port data
			 */
			for (port = start_port; port <= end_port; port++) {
				Port_t *portp;
				portp = sm_get_port(nodep, port);
				if (!sm_valid_port(portp)) {
					cs_log(VS_LOG_VERBOSE, __func__,
									"Failed to get port %d of node %s",
									port, sm_nodeDescString(nodep));
					continue;
				}
				memcpy(&portp->portData->bufCtrlTable, &bcts[0],
								sizeof(portp->portData->bufCtrlTable));
			}
		}
	} else {
		// Send the BCTs in the fewest MADs possible. A single MAD can hold
		// up to maxcount BCTs, but may hold fewer because we need to omit
		// ports that are not up.
		//
		// In this loop, the value of num_ports is the # of BCTs that can be
		// sent in the next MAD, including the current port. We may end up
		// sending less than that if the current port is not valid or isn't up.
		for (num_ports = 1, port = start_port; port <= end_port;
			num_ports++, port ++) {
			Port_t *portp = sm_get_port(nodep, port);

			if (!sm_valid_port(portp)) {
				cs_log(VS_LOG_VERBOSE, __func__,
					"Failed to get port %d of node %s",
					port, sm_nodeDescString(nodep));
			}

			if (!sm_valid_port(portp) || (portp->state < IB_PORT_INIT)) {
				// We have to skip down ports, so send what we have, not
				// including the bad port.  If this is the only port
				// (num_ports == 1) then skip the send.
				if (num_ports > 1) {
					uint16_t first_port = port - num_ports + 1;
					STL_BUFFER_CONTROL_TABLE *buf = &bcts[first_port-start_port];
					uint32_t amod = ((num_ports-1)<<24) | first_port;
					SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);

					psc_unlock(psc);
					status = SM_Set_BufferControlTable(fd, amod,
						&addr, buf, mkey, &madStatus);
					psc_lock(psc);
					if (status != VSTATUS_OK) {
						cs_log(VS_LOG_ERROR, __func__,
							"buffer control table query failed; node %s; %s (madStatus=%s)",
							sm_nodeDescString(nodep),
							cs_convert_status (status), sm_getMadStatusText(madStatus));
						goto error;
					} else {
						/*
						 * If the Set was successful update port data
						 */
						uint8_t p;
						for (p = first_port; p < port; p++) {
							Port_t *portp2 = sm_get_port(nodep, p);
							if (sm_valid_port(portp2)) {
								memcpy(&portp2->portData->bufCtrlTable, &buf[p-first_port],
									sizeof(portp2->portData->bufCtrlTable));
							}
						}
					}
				}

				// Note that this will get set back to 1 by the for loop.
				num_ports = 0;
			} else if ((num_ports == maxcount) || port == end_port) {
				// Unlike the previous case, here we include the current port.
				uint16_t first_port = port - num_ports + 1;
				STL_BUFFER_CONTROL_TABLE *buf = &bcts[first_port-start_port];
				uint32_t amod = (num_ports<<24) | first_port;
				SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);

				psc_unlock(psc);
				status = SM_Set_BufferControlTable(fd, amod, &addr, buf,
					mkey, &madStatus);
				psc_lock(psc);
				if (status != VSTATUS_OK) {
					cs_log(VS_LOG_ERROR, __func__,
						"buffer control table query failed; node %s; %s (madStatus=%s)",
						sm_nodeDescString(nodep),
						cs_convert_status (status), sm_getMadStatusText(madStatus));
					goto error;
				} else {
					/*
					 * If the Set was successful update port data
					 */
					uint8_t p;
					for (p = first_port; p <= port; p++) {
						Port_t *portp2 = sm_get_port(nodep, p);
						if (!portp2) {
							cs_log(VS_LOG_ERROR, __func__,
								"invalid pointer for node %s, port %u",
								sm_nodeDescString(nodep), p);
							status = VSTATUS_BAD;
							goto error;
						}
						memcpy(&portp2->portData->bufCtrlTable, &buf[p-first_port],
							sizeof(portp2->portData->bufCtrlTable));
					}
				}

				// Note that this will get set back to 1 by the for loop.
				num_ports = 0;
			}
		}
	}

error:
	psc_free_mai(psc, maip);
	return (status);
}

// FIXME: We need to consider the right way to handle these variables.
extern int topo_errors;
extern int topo_abandon_count;

// The following defines are represented in terms of bytes
//  and will later be converted to buffer credits.
// [Buffer credit sizes may change in future generations]
//
// These defines are derived from tuning algorithms to determine the
//  additional latency incurred from using sideband credit returns and
//  packet credit returns.
// The buffer allocation schemes much account for this latency.
#define CR_LATENCY_SIDEBAND 2304	// 36 credits
#define CR_LATENCY_PACKET 4224		// 66 credits

#define PROTOCOL_HEADER_SIZE 128

static int
_bw_compare(const void * a, const void * b) { return (*(int16_t*)a - *(int16_t*)b);}

static Status_t
_setup_buffer_control(int32_t memSize, int16_t * pbw, uint8_t* pmtu, int32_t wd, int32_t au,
                   bool_t shmem, bool_t wh, bool_t vl15,
				   STL_BUFFER_CONTROL_TABLE * pBfrCtrl)
{
    int       i, j;
    int32_t   intervalSize;
    int32_t   remainingSize;
    int32_t   packetSize[STL_MAX_VLS];
    int16_t   rankBw[STL_MAX_VLS];
    int32_t   header = PROTOCOL_HEADER_SIZE;
    int32_t   maxNoDed;
    uint8_t   mult = (uint8_t)sm_config.dedicatedVLMemMulti;
    int32_t   minShared;

    enum RULES {
        RULE_A = 1,
        RULE_B = 2,
        RULE_C = 3
    } applied;

	// If this port does not support shared memory,
	// Ensure use of dedicated memory
    if ((shmem==FALSE) && (mult==0)) {
		mult = 1;
    }

    // Ensure consistency of requests.
    minShared = (sm_config.minSharedVLMem * memSize)/100;
    if ((minShared>0) && (shmem==FALSE)) {
        minShared=0;
    }

    for (i=0; i<STL_MAX_VLS; i++) {
        // Initialize VLs buffer space to 0
        pBfrCtrl->VL[i].TxDedicatedLimit=0;
        pBfrCtrl->VL[i].TxSharedLimit=0;

        // Initialize the packet sizes.
        // Do everything in terms of au's
        // Ensure dedicated buffer space for VL15 if the neighbor needs VL15.
        if (i == 15) {
			if (TRUE == vl15) {
				packetSize[i] = (2048+header)*2;
				packetSize[i] = (packetSize[i] + au - 1)/au; /* round-up */

				// check for unexpected limitations
				//  which prevent even VL15 from having dedicated memory
				if (memSize-minShared-packetSize[i]-wd<0) {
					if (minShared > (packetSize[i] + wd)) {
						minShared -= (packetSize[i] + wd);
					} else {
						minShared = 0;
					}
				}
			} else {
				packetSize[i] = 0;
			}
        } else if (pbw[i]>0) {
                packetSize[i] = (GetBytesFromMtu(pmtu[i])+header)*mult;

            packetSize[i] = (packetSize[i] + au - 1)/au; /* round-up */
        } else {
            packetSize[i] = 0;
        }
    }


    // Attempt Rule A: allocate a packet-size plus wire depth
    remainingSize = memSize-minShared;
    intervalSize = 1;
    applied = RULE_A;
    for (i=0; i<STL_MAX_VLS; i++) {
        if (packetSize[i]>0) {
            remainingSize -= (packetSize[i]+wd);
            if (remainingSize < 0) break;

            intervalSize  += packetSize[i];
            pBfrCtrl->VL[i].TxDedicatedLimit = packetSize[i]+wd;
        }
    }

    // If no shared memory, apply RULE_A iteratively until all memory consumed.
    intervalSize = remainingSize/intervalSize;
    if ((shmem==FALSE) && (intervalSize >0)) {
        for (i=0; i<STL_MAX_VLS; i++) {
            if (packetSize[i]>0) {
                pBfrCtrl->VL[i].TxDedicatedLimit+=packetSize[i]*intervalSize;
            }
        }
    }

    // Rank the QOS for subsequent rules
    if (remainingSize < 0) {
        memcpy(rankBw, pbw, sizeof(rankBw));
        qsort(rankBw, STL_MAX_VLS, sizeof(int16_t), _bw_compare);
    }

    // Attempt Rule B
    // (only applicable if Wire Depth is non-zero, otherwise result is rule A)
    if ((remainingSize < 0) && (wd > 0)){
        applied = RULE_B;

        for (i=0; i<STL_MAX_VLS;i++) {
            // optimize for duplicate rank values.
            if ((i<STL_MAX_VLS-1) && (rankBw[i]==rankBw[i+1])) continue;

            remainingSize = memSize-minShared;

            for (j=0; j<STL_MAX_VLS;j++) {
                if (j == 15 && !vl15) {
                    continue;
                }
                if (pbw[j]>rankBw[i] || j == 15) {
                    remainingSize -= (packetSize[j]+wd);
                    pBfrCtrl->VL[j].TxDedicatedLimit = packetSize[j]+wd;
                } else {
                    remainingSize -= packetSize[j];
                    pBfrCtrl->VL[j].TxDedicatedLimit = packetSize[j];
                }
                if (remainingSize<0) break;
            }

            if (remainingSize>=0) break;
        }
    }

    // Attempt Rule C (must work)
    if (remainingSize < 0) {
        applied = RULE_C;

        for (i=0; i<STL_MAX_VLS;i++) {
            // optimize for duplicate rank values.
            if ((i<STL_MAX_VLS-1) && (rankBw[i]==rankBw[i+1])) continue;

            maxNoDed = 0;
            remainingSize = memSize-minShared;

            for (j=0; j<STL_MAX_VLS;j++) {
                if (j == 15 && !vl15) {
                    continue;
                }
                if (pbw[j]>rankBw[i] || j == 15) {
                    remainingSize -= (packetSize[j]+wd);
                    if (remainingSize<0) break;

                    pBfrCtrl->VL[j].TxDedicatedLimit = packetSize[j]+wd;
                } else {
                    maxNoDed = MAX(maxNoDed, (packetSize[j]+wd));
                    pBfrCtrl->VL[j].TxDedicatedLimit = 0;
                }
            }

            if (remainingSize>=0) {
                if (shmem==FALSE)
                    break;
                else if (remainingSize+minShared >= maxNoDed)
                     break;
            }
        }
    }

    if (shmem==TRUE) {
        remainingSize+=minShared;
    } else {
        remainingSize=0;
    }
    pBfrCtrl->TxOverallSharedLimit = remainingSize;

    for (i = 0; i < STL_MAX_VLS; i++) {
        if (i == 15 && !vl15) {
            continue;
        }
        if (i == 15 || pbw[i]>0) {
            if (applied == RULE_C) {
                // In worst case scenarios, only allow VL to shared if dedicated is 0.
                if (pBfrCtrl->VL[i].TxDedicatedLimit==0) {
                    pBfrCtrl->VL[i].TxSharedLimit=remainingSize;
                }
            } else {
                pBfrCtrl->VL[i].TxSharedLimit=remainingSize;
            }
        }
    }

    // Rule C works, but it might prevent forward progress.
    // We need to alert the user of this condition, but do not have
    //  meaningful information to put in the log / error messsage.
    // Return an error here so that caller can create appropriate log msg.
    if (applied == RULE_C) {
        return (VSTATUS_BAD);
    }

    return (VSTATUS_OK);
}

static Status_t
_initialize_port_bfrctrl(Topology_t * topop, Node_t * nodep, Port_t * portp,
	STL_BUFFER_CONTROL_TABLE *bct)
{
	Port_t*     neighborPort = 0;
	Node_t*     neighborNode = 0;
	int32_t     rxMemSize;
	int32_t     wd;
	int32_t     au;
	bool_t      shmem;
	int16_t     bw[STL_MAX_VLS];
	uint8_t     mtu[STL_MAX_VLS];
	int         vf, vl;
	VlVfMap_t   vlvfmap;
	VlBwMap_t   vlbwmap;
	VirtualFabrics_t *VirtualFabrics = topop->vfs_ptr;
	bool_t      needWd;
	bool_t      needVl15 = TRUE;

	// Find the neighbor port (Get neighbors recv buffer size and allocation units)
	neighborPort = sm_find_neighbor_node_and_port(topop, portp, &neighborNode);
	if (neighborPort==NULL) {
		IB_LOG_ERROR_FMT(__func__,
			"Unable to find neighbor port for node %s guid "
			FMT_U64 " Port number=%d", sm_nodeDescString(nodep),
			nodep->nodeInfo.NodeGUID, portp->index);

		return (VSTATUS_BAD);
	}


	au=8*(1 << neighborPort->portData->portInfo.BufferUnits.s.BufferAlloc);
	shmem = neighborPort->portData->portInfo.CapabilityMask3.s.IsSharedSpaceSupported;


	// Wire Depth is calculated based on the modes the port pairs
	// are capable of and not what is currently enabled.
	needWd = (portp->portData->portInfo.PortLinkMode.s.Supported &
		neighborPort->portData->portInfo.PortLinkMode.s.Supported &
		STL_PORT_LINK_MODE_STL) == STL_PORT_LINK_MODE_STL;

	if (neighborNode->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		// Switch port zero has mask3 bits, look up switch port zero.
		// Note: PRR is setting up external switch ports instead of switch port zero,
		// so check both.
		Port_t* smaportp;
		smaportp = sm_get_port(neighborNode, 0);
		if (sm_valid_port(smaportp))
			shmem |= smaportp->portData->portInfo.CapabilityMask3.s.IsSharedSpaceSupported;
	}

	rxMemSize = neighborPort->portData->portInfo.OverallBufferSpace;
	if (rxMemSize==0) {
		IB_LOG_ERROR_FMT(__func__,
			"Overall Rx Buffer size is zero for node %s guid "
			FMT_U64 " Port number=%d", sm_nodeDescString(neighborNode),
			neighborNode->nodeInfo.NodeGUID, neighborPort->index);
		return (VSTATUS_BAD);
	}

	// Intent is to initialize the initial WireDepth if:
	//  - First time SM is run (restart SM case, in which WD will be defaulted)
	//  - First time encountering the port (which may be active, WD will be defaulted)
	//  - Bounced ports (Port will be in init)
	//  - Initial configuration (Port will be in init)
	//
	// We always want to use the same value for Wire Depth even though it may vary
	// throughout the life of the connection even though the port may not bounce.
	// This value will be the max wire depth of the port pair.
	if ((portp->portData->initWireDepth == -1) ||
		(portp->state == IB_PORT_INIT)){
		if (!needWd) {
			wd = portp->portData->initWireDepth = 0;
		} else {
			// Tests to override wire depth / replay depths from configuration.
			int32_t bufferDepth = (portp->portData->portInfo.ReplayDepthH.BufferDepthH << 8) |
					portp->portData->portInfo.ReplayDepth.BufferDepth;
			if ((int32_t)(sm_config.wireDepthOverride) == -1) {
				// No override for Wire depth
				// Choose the largest LTP Round Trip among self and neighbor.
				int32_t wireDepth = (portp->portData->portInfo.ReplayDepthH.WireDepthH << 8) |
						portp->portData->portInfo.ReplayDepth.WireDepth;
				int32_t neighborWireDepth = (neighborPort->portData->portInfo.ReplayDepthH.WireDepthH << 8) |
						neighborPort->portData->portInfo.ReplayDepth.WireDepth;
				wd = MAX(wireDepth, neighborWireDepth);
				if ((int32_t)(sm_config.replayDepthOverride) == -1) {
					// No override for Replay depth
					// Choose the min of wire depth / replay depth, and covert to bytes.
					wd = BYTES_PER_LTP * MIN(wd, bufferDepth);
				} else if (sm_config.replayDepthOverride == 0) {
					// Do not consider replay depth, convert wire depth to bytes.
					wd = wd * BYTES_PER_LTP;
				}
				else {
					// Replay depth overriden.
					// Choose the min of wire depth / replay depth, and covert to bytes.
					wd = MIN(wd * BYTES_PER_LTP, sm_config.replayDepthOverride);
				}
			} else if (sm_config.wireDepthOverride == 0) {
				// Do not consider wire depth
				if ((int32_t)(sm_config.replayDepthOverride) == -1) {
					// No override for replay depth. Choose replay depth; convert to bytes.
					wd = bufferDepth * BYTES_PER_LTP;
				} else if (sm_config.replayDepthOverride == 0) {
					// Do not consider either wire depth or replay depth from port info.
					wd = 0;
				} else {
					// Replay depth overridden. (Already in bytes)
					wd = sm_config.replayDepthOverride;
				}
			} else {
				// Wire Depth overridden
				if ((int32_t)(sm_config.replayDepthOverride) == -1) {
					// No override for replay depth.
					// Choose min of wire depth (override) and replay depth. Convert to bytes.
					wd = MIN(sm_config.wireDepthOverride, bufferDepth * BYTES_PER_LTP);
				} else if (sm_config.replayDepthOverride == 0) {
					// Do not consider replay depth; only overridden wire depth remains, already in bytes.
					wd = sm_config.wireDepthOverride;
				} else {
					// Both wire depth and reply depth overridden. Choose min, already in bytes.
					wd = MIN(sm_config.wireDepthOverride,
									sm_config.replayDepthOverride);
				}
			}

			// Add in "Extra Credits" to account for credit return latencies.
			// This is based on whether credits are returned in-band through idle flits
			// or through idle packets, which is based on CRC mode.
			// The "Extra Credits" are expressed in terms of bytes to allow for future expansion.
			if (portp->portData->portInfo.PortLTPCRCMode.s.Active ==
							STL_PORT_LTP_CRC_MODE_14) {
				wd += CR_LATENCY_SIDEBAND;
			} else {
				wd += CR_LATENCY_PACKET;
			}

			// Convert WD from bytes to AU's (rounding up)
			wd = (wd + au - 1) / au;

			portp->portData->initWireDepth = wd;
		}
	} else {
		// Assumes port state is ARMED or ACTIVE
		// and that SM has seen this port before.
		wd = portp->portData->initWireDepth;
	}

	// Setup BW and MTU per VL based on this ports VL membership in VFs
	topop->routingModule->funcs.select_vlvf_map(topop, nodep, portp, &vlvfmap);
	topop->routingModule->funcs.select_vlbw_map(topop, nodep, portp, &vlbwmap);

	// Evaluate MTU and QOS for this VL.
	// Note mtu[...] has buffer space requirement in units of MTU
	for (vl=0;vl<STL_MAX_VLS;vl++) {
		mtu[vl]=0;
		bw[vl]=0;

		// If VL has no VFs associated with it, VL is not in use
		if (!bitset_nset(&vlvfmap.vf[vl])) {
			bitset_free(&vlvfmap.vf[vl]);
			continue;
		}

		for(vf = 0; (vf = bitset_find_next_one(&vlvfmap.vf[vl], vf)) != -1; ++vf) {
			mtu[vl] = MAX(mtu[vl], VirtualFabrics->v_fabric_all[vf].max_mtu_int);
		}
		if (vlbwmap.highPriority[vl]) {
			bw[vl] = 100;
		} else {
			bw[vl] = MAX(vlbwmap.bw[vl], 1);
		}
		mtu[vl] = MIN(mtu[vl], portp->portData->maxVlMtu);

		//free bitsets allocated in select
		bitset_free(&vlvfmap.vf[vl]);
	}

	// Setup the buffer control map.
	if (_setup_buffer_control(rxMemSize, bw, mtu,  wd, au, shmem, FALSE, needVl15,
							bct)!= VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Errors encountered for setup Buffer Control for node %s guid "
			FMT_U64 " Port number=%d", sm_nodeDescString(nodep),
			nodep->nodeInfo.NodeGUID, portp->index);
	}

	return VSTATUS_OK;
}

/** =========================================================================
 * NOTE: bcts array must store all tables for start - end port.
 *
 * Side affect: on success port objects within the node object will be updated
 * with the tables sent to the SMA.
 */
static void
_bctctrl_assignment_worker(ParallelSweepContext_t *psc, ParallelWorkItem_t *pwi)
{
	Status_t	status;
	BctWorkItem_t *bwi = PARENT_STRUCT(pwi, BctWorkItem_t, item);
	Node_t		*nodep = bwi->nodep;

	int 		needSet = 0;
	uint8_t		uniformBcts;

	// We really shouldn't access the topology in parallel with other
	// worker threads, so immediately grab the lock.
	psc_lock(psc);

	STL_BUFFER_CONTROL_TABLE *tmp_bcts;
	size_t bcts_size = sizeof(STL_BUFFER_CONTROL_TABLE) *
		(nodep->nodeInfo.NumPorts+1);
	status = vs_pool_alloc(&sm_pool, bcts_size, (void**)&tmp_bcts);
	if (status != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__,
			"Failed to allocate buffer control table temp mem; node %s",
			sm_nodeDescString(nodep));
		goto exit;
	}
	memset(tmp_bcts,0,bcts_size);

	// Switches can potentially have identical BCTs on every port, but
	// we exclude edge switches because BCTs for ports linked with HFIs are
	// usually different from BCTs for ISLs.
	uniformBcts = (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
		!nodep->edgeSwitch);

	Port_t *portp;
	for_all_ports(nodep, portp) {
		if (!sm_valid_port(portp) || portp->state <= IB_PORT_DOWN)
			continue;

		if (nodep->nodeInfo.NodeType == STL_NODE_SW && portp->index == 0)
			continue;

		if ((status=_initialize_port_bfrctrl(sm_topop, nodep, portp,
			&tmp_bcts[portp->index]))
			!= VSTATUS_OK) {
			if (++topo_errors > sm_config.topo_errors_threshold &&
				++topo_abandon_count <= sm_config.topo_abandon_threshold) {
				goto exit;
			}
			sm_mark_link_down(sm_topop, portp);
			topology_changed = 1;   /* indicates a fabric change has been detected */
			IB_LOG_ERROR_FMT(__func__, "Failed to init Buffer Control "
				"(ignoring port) on node %s nodeGuid "FMT_U64" node index "
				"%d port index %d", sm_nodeDescString(nodep),
				nodep->nodeInfo.NodeGUID, nodep->index, portp->index);
		} else {
			if (portp->portData->current.bfrctrl) {
				if (memcmp(&tmp_bcts[portp->index],
					&portp->portData->bufCtrlTable,
					sizeof(portp->portData->bufCtrlTable)))
					needSet = 1;
			} else {
				needSet = 1;
			}

			// If we still have uniformBcts, check to see if this BCT
			// matches the others. If it does not, turn off the
			// uniformBcts flag.
			if (uniformBcts && portp->index > 1 &&
				memcmp(&tmp_bcts[1], &tmp_bcts[portp->index],
				sizeof(portp->portData->bufCtrlTable))) {
				uniformBcts = 0;
			}
		}

		if (needSet)
			portp->portData->current.bfrctrl = 0;

		if ((sm_config.forceAttributeRewrite || needSet) &&
			(nodep->nodeInfo.NodeType == NI_TYPE_CA ||
			!sm_config.optimized_buffer_control)) {
			if ((status = _set_buffer_control_tables(nodep, portp->index,
				portp->index, &tmp_bcts[portp->index], 0, psc)) != VSTATUS_OK)
			{
				status = sm_popo_port_error(&sm_popo, sm_topop, portp, status);
				if (status == VSTATUS_TIMEOUT_LIMIT) {
					psc_stop(psc);
					goto exit;
				}
			}
			needSet = 0;
			portp->portData->current.bfrctrl = 1;
		}
	}

	if ((needSet || sm_config.forceAttributeRewrite) &&
		(nodep->nodeInfo.NodeType == NI_TYPE_SWITCH &&
		sm_config.optimized_buffer_control)) {
			if (_set_buffer_control_tables(nodep, 1, nodep->nodeInfo.NumPorts,
				&tmp_bcts[1], uniformBcts, psc) != VSTATUS_OK)
			{
				status = sm_popo_port_error(&sm_popo, sm_topop, portp, status);
				if (status == VSTATUS_TIMEOUT_LIMIT) {
					psc_stop(psc);
					goto exit;
				}
			}

			Port_t* tempPort = NULL;
			for_all_end_ports(nodep, tempPort) {
				if (!sm_valid_port(tempPort) || tempPort->state <= IB_PORT_DOWN)
					continue;

				tempPort->portData->current.bfrctrl = 1;
			}
		}

exit:
	psc_unlock(psc);
	if (tmp_bcts) vs_pool_free(&sm_pool, tmp_bcts);
	_bctworkitem_free(bwi);
	psc_set_status(psc, status);
}

Status_t
sweep_assignments_buffer_control(SweepContext_t *sweep_context)
{
	Status_t	status = VSTATUS_OK;
	Node_t		*nodep;

	psc_go(sweep_context->psc);

	for_all_nodes(sm_topop, nodep) {
		if (sm_state != SM_STATE_MASTER) {
			status = VSTATUS_NOT_MASTER;
			break;
		}

		BctWorkItem_t *bwi = _bctworkitem_alloc(nodep,
			_bctctrl_assignment_worker);
		if (!bwi) {
			status = VSTATUS_NOMEM;
			break;
		}
		psc_add_work_item(sweep_context->psc, &bwi->item);
	}

	if (status == VSTATUS_OK) {
		// Wait for the workers to set the bcts.
		status = psc_wait(sweep_context->psc);
	} else {
		// We aborted for some reason. Wait for the workers
		// to clear out the work queue. Note that we preserve
		// the old status, rather than updating with any errors
		// from the workers.
		psc_stop(sweep_context->psc);
		(void)psc_wait(sweep_context->psc);
	}

	// When we get here, we're either done and the work queue is empty,
	// or we hit an error and stopped early. Either way, drain the work
	// item queue just to be sure.
	psc_drain_work_queue(sweep_context->psc);

	return status;
}

