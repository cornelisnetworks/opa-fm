/* BEGIN_ICS_COPYRIGHT1 ****************************************

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

** END_ICS_COPYRIGHT1   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _SM_QOS_H_
#define _SM_QOS_H_

#define SCVLMAP_BASE        8
#define SCVLMAP_MAX_INDEX   32

extern uint8_t sm_SLtoSC[STL_MAX_SLS];
extern uint8_t sm_SCtoSL[STL_MAX_SCS];

void	sm_printf_vf_debug(VirtualFabrics_t *vfs);

Qos_t*	sm_alloc_qos(void);
void	sm_free_qos(Qos_t* qos);
void	sm_install_qos(Qos_t* qos);
Qos_t*	sm_get_qos(uint8_t vl);

void	sm_fill_SCVLMap(Qos_t *qos);

void 	sm_setup_qos_1vl(RoutingModule_t *rm, Qos_t * qos,
			VirtualFabrics_t *VirtualFabrics);
void 	sm_setup_qos(RoutingModule_t *rm, Qos_t * qos,
			VirtualFabrics_t *VirtualFabrics, const uint8_t *SLtoSC);

void	DivideBwUp(RoutingModule_t *rm, Qos_t *qos, int bw, int base_sl,
			int resp_sl, int mcast_sl, const uint8_t *SLtoSC);

void	sm_DbgPrintQOS(Qos_t * qos);

#endif //_SM_QOS_H_
