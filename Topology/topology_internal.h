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

#ifdef __cplusplus
};
#endif

#endif /* _TOPOLOGY_INTERNAL_H */
