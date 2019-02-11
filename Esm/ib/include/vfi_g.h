/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/************************************************************************
*                                                                      *
* FILE NAME                                                            *
*    if3.h                                                             *
*                                                                      *
* DESCRIPTION                                                          *
*    Library calls for interface 3                                     *
*                                                                      *
*                                                                      *
* DEPENDENCIES                                                         *
*    ib_types.h                                                        *
*                                                                      *
*                                                                      *
************************************************************************/

#ifndef ___VFI_G_H___
#define ___VFI_G_H___

#include "ib_types.h"
#include "mai_g.h"
#include "ib_sa.h"
#include "stl_sa_priv.h"

#define VFI_DEFAULT_GUID     (0)

#define VFI_SERVICE_RECORD	IB_SERVICE_RECORD
#define VFI_SVRREC_ID		IB_SERVICE_RECORD_COMP_SERVICEID
#define VFI_SVRREC_GID		IB_SERVICE_RECORD_COMP_SERVICEGID
#define VFI_SVREC_NAME		IB_SERVICE_RECORD_COMP_SERVICENAME
#define VFI_SVREC_PARTITION	IB_SERVICE_RECORD_COMP_SERVICEPKEY
#define VFI_SVREC_LEASE 	IB_SERVICE_RECORD_COMP_SERVICELEASE
#define VFI_SVREC_KEY		IB_SERVICE_RECORD_COMP_SERVICEKEY

#define VFI_REGFORCE_FABRIC (0x01)
#define VFI_REGTRY_FABRIC   (0x00)

#define VFI_LMC_BASELID      (0x00)
#define VFI_MAX_LMC          (7)       /* Maximum value the lmc 3 bits allow */

#define VFI_MAX_SERVICE_PATH (128)
#define VFI_MAX_PORT_GUID    (7)      /* The maximum guid index */

typedef int (*VfiSvcRecCmp_t)(VFI_SERVICE_RECORD *, VFI_SERVICE_RECORD *);

Status_t        vfi_mngr_register(IBhandle_t fd, uint8_t mclass,
				  int gididx, VFI_SERVICE_RECORD * service,
				  uint64_t mask, int option);
Status_t        vfi_mngr_unregister(IBhandle_t fd, uint8_t mclass,
				    int gididx, VFI_SERVICE_RECORD * service,
				    uint64_t mask);
Status_t        vfi_mngr_find(IBhandle_t fd, uint8_t mclass,
			      int slmc, VFI_SERVICE_RECORD * service, uint64_t mask,
			      int *count, IB_PATH_RECORD * pbuff);

Status_t        vfi_mngr_find_cmp(IBhandle_t fd, uint8_t mclass,
			      int slmc, VFI_SERVICE_RECORD * service, uint64_t mask,
			      int *count, IB_PATH_RECORD * pbuff, VfiSvcRecCmp_t cmp);

#endif
