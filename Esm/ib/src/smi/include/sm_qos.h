/* BEGIN_ICS_COPYRIGHT1 ****************************************

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

** END_ICS_COPYRIGHT1   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _SM_QOS_H_
#define _SM_QOS_H_

#define SCVLMAP_BASE        8
#define SCVLMAP_MAX_INDEX   32

// Used for routing modules
Status_t sm_assign_scs_to_sls_FixedMap(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics);
Status_t sm_assign_scs_to_sls_NonFixedMap(RoutingModule_t *rm, VirtualFabrics_t *VirtualFabrics);

extern uint8_t sm_SLtoSC[STL_MAX_SLS];
extern uint8_t sm_SCtoSL[STL_MAX_SCS];
extern Status_t sm_node_handleGetRespAggr(Node_t *, Port_t *, STL_AGGREGATE *, STL_AGGREGATE *);

void sm_printf_vf_debug(VirtualFabrics_t *vfs);

Qos_t* sm_get_qos(uint8_t vl);

void sm_fill_SCVLMap(Qos_t *qos);

Status_t sweep_assignments_vlarb(SweepContext_t *);
Status_t sweep_assignments_buffer_control(SweepContext_t *);
#endif //_SM_QOS_H_
