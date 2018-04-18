/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _PASERVER_H
#define _PASERVER_H

Status_t pa_getClassPortInfoResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getGroupListResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getPortCountersResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getGroupInfoResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getGroupConfigResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getGroupNodeInfoResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getGroupLinkInfoResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_clrPortCountersResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_clrAllPortCountersResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getPmConfigResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_freezeImageResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_releaseImageResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_renewImageResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getFocusPortsResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getFocusPortsMultiSelectResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getImageInfoResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_moveFreezeImageResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getVFListResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getVFInfoResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getVFConfigResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getVFPortCountersResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_clrVFPortCountersResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);
Status_t pa_getVFFocusPortsResp(Mai_t *maip, pa_cntxt_t* pa_cntxt);

#endif /* _PASERVER_H */
