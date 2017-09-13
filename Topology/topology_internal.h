/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _TOPOLOGY_INTERNAL_H
#define _TOPOLOGY_INTERNAL_H

#include <topology.h>

// unpublished interfaces internal to topology library
#ifdef __cplusplus
extern "C" {
#endif

#define MYTAG MAKE_MEM_TAG('T','o', 'p', 'o')

#define PROGRESS_FREQ 25	// how many nodes between progress reports

extern const char *g_Top_cmdname;
extern Top_FreeCallbacks g_Top_FreeCallbacks;

extern void PortDataFree(FabricData_t *fabricp, PortData *portp);
extern FSTATUS AllLidsAdd(FabricData_t *fabricp, PortData *portp, boolean force);
extern void AllLidsRemove(FabricData_t *fabricp, PortData *portp);
#if !defined(VXWORKS) || defined(BUILD_DMC)
extern void IocDataFree(FabricData_t *fabricp, IocData *iocp);
extern void IouDataFreeIocs(FabricData_t *fabricp, IouData *ioup);
extern void IouDataFree(FabricData_t *fabricp, IouData *ioup);
#endif
extern void NodeDataFreePorts(FabricData_t *fabricp, NodeData *nodep);
extern void NodeDataFreeSwitchData(FabricData_t *fabricp, NodeData *nodep);
extern void NodeDataFree(FabricData_t *fabricp, NodeData *nodep);
extern void NodeDataFreeAll(FabricData_t *fabricp);
extern void SMDataFree(FabricData_t *fabricp, SMData *smp);
extern void SMDataFreeAll(FabricData_t *fabricp);
extern void MCDataFreeAll(FabricData_t *fabricp);
extern void MCMemberFree(FabricData_t *fabricp, McMemberData *mcmemberp);
extern void VFDataFreeAll(FabricData_t *fabricp);
extern void MCGroupFree(FabricData_t *fabricp, McGroupData *mcgroupp);
extern void CableDataFree(CableData *cablep);
extern void PortSelectorFree(PortSelector *portselp);
extern void ExpectedLinkFree(FabricData_t *fabricp, ExpectedLink *elinkp);
extern void ExpectedLinkFreeAll(FabricData_t *fabricp);
extern void ExpectedNodeFree(FabricData_t *fabricp, ExpectedNode *enodep, QUICK_LIST *listp);
extern void ExpectedNodesFreeAll(FabricData_t *fabricp, QUICK_LIST *listp);
extern void ExpectedSMFree(FabricData_t *fabricp, ExpectedSM *esmp);
extern void ExpectedSMsFreeAll(FabricData_t *fabricp);

// create SystemData as needed
// add this Node to the appropriate System
// This should only be invoked once per node (eg. not per NodeRecord)
extern FSTATUS AddSystemNode(FabricData_t *fabricp, NodeData *nodep);

// direct SMA queries
extern FSTATUS SmaGetNodeDesc(struct omgt_port *port, NodeData *nodep, uint32_t lid, STL_NODE_DESCRIPTION *pNodeDesc);
extern FSTATUS SmaGetNodeInfo(struct omgt_port *port, NodeData *nodep, uint32_t lid, STL_NODE_INFO *pNodeInfo);
extern FSTATUS SmaGetSwitchInfo(struct omgt_port *port, NodeData *nodep, uint32_t lid, STL_SWITCH_INFO *pSwitchInfo);
extern FSTATUS SmaGetPortInfo(struct omgt_port *port, NodeData *nodep, uint32_t lid, uint8_t portNum, STL_PORT_INFO *pPortInfo);
extern FSTATUS SmaGetPartTable(struct omgt_port *port, NodeData *nodep, uint32_t lid, uint8_t portNum, uint16 block, STL_PARTITION_TABLE *pPartTable);
extern FSTATUS SmaGetVLArbTable(struct omgt_port *port, NodeData *nodep, uint32_t lid, uint8_t portNum, uint8 part, STL_VLARB_TABLE *pVLArbTable);
extern FSTATUS SmaGetSLSCMappingTable(struct omgt_port *port, NodeData *nodep, uint32_t lid, STL_SLSCMAP *pSLSCMap);
extern FSTATUS SmaGetSCSLMappingTable(struct omgt_port *port, NodeData *nodep, uint32_t lid, STL_SCSLMAP *pSCSLMap);
extern FSTATUS SmaGetSCSCMappingTable(struct omgt_port *port, NodeData *nodep, uint32_t lid, uint8_t in_port, uint8_t out_port, STL_SCSCMAP *pSCSCMap);
extern FSTATUS SmaGetSCVLMappingTable(struct omgt_port *port, NodeData *nodep, uint32_t lid, uint8_t port_num, STL_SCVLMAP *pSCVLMap, uint16_t attr);
extern FSTATUS SmaGetLinearFDBTable(struct omgt_port *port, NodeData *nodep, uint32_t lid, uint16 block, STL_LINEAR_FORWARDING_TABLE *pFDB);
extern FSTATUS SmaGetMulticastFDBTable(struct omgt_port *port, NodeData *nodep, uint32_t lid, uint32 block, uint8 position, STL_MULTICAST_FORWARDING_TABLE *pFDB);
extern FSTATUS SmaGetBufferControlTable(struct omgt_port *port, NodeData *nodep, uint32_t lid, 
							   uint8_t startPort, uint8_t endPort, STL_BUFFER_CONTROL_TABLE pBCT[]);
extern FSTATUS SmaGetCableInfo(struct omgt_port *port, NodeData *nodep, uint32_t lid, uint8_t portnum, uint16_t addr, uint8_t len, uint8_t *data);

#ifdef __cplusplus
};
#endif

#endif /* _TOPOLOGY_INTERNAL_H */
