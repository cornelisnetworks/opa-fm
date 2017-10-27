/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

/************************************************************************
 *                                                                      *
 * FILE NAME                                                            *
 *    if3_def.h                                                         *
 *                                                                      *
 * DESCRIPTION                                                          *
 *    Constants used for  Interface 3 multi-mad transport Library       *
 *                                                                      *
 *                                                                      *
 * DEPENDENCIES                                                         *
 *                                                                      *
 *                                                                      *
 *                                                                      *
 ************************************************************************/

#ifndef __IF3_DEF_H__
#define __IF3_DEF_H__
#include <ib_const.h>
#include <ib_sa.h>
#include <cs_g.h>
#include <cs_log.h>
#include "if3.h"
#undef LOCAL_MOD_ID
#define LOCAL_MOD_ID VIEO_IF3_MOD_ID

#ifdef DGIF
#undef IB_LOG_VERBOSE
#define IB_LOG_VERBOSE(_x,_y) printf (" %s : %u \n",_x,_y);
#undef IB_LOG_INFO
#define IB_LOG_INFO(_x,_y) printf (" %s : %u \n",_x,_y);
#undef IB_LOG_ERROR
#define IB_LOG_ERROR(_x,_y) printf (" %s : %u \n",_x,(int)_y);
#endif

#define MAX_MANAGER       (12)
#define FE_MANAGER_SL     (0)
#define FE_MANAGER_PKEY   (mai_get_default_pkey())
#define FE_MANAGER_QKEY   (GSI_WELLKNOWN_QKEY)
#define FE_MANAGER_VL     (0)

#define MAX_RETRY         (3)
#define RC_MAD_TIMEOUT    (2621440)
#define DEFAULT_RTV       (19)      // default response time value corresponding to 2.6 seconds
#define SA_BUSY_ATTEMPTS  (2)
#define DEFAULT_SUBNET_TIMEOUT  (18)

// sub commands set in the AID field
#define FE_MNGR_DISCONNECT  (0xBB01)
#define FM_MNGR_MDATAREADY  (0xBB02)
#define FM_MNGR_DATAREADY   (0xCC01)
#define FM_ASYNC_DATA       (0xCC03)
#define FM_ASYNC_MDATA      (0xCC04)

#define  RC_DEFAULT_WINDOW  (3)
#define  RC_MIN_WINDOW      (1)
#define  SA_MAX_WINDOW      (64)
#define  MAX_RESEND         (3)

#define IF3_PORT_SERVICE    (1)
#define IF3_FABRIC_SERVICE  (2)


#define CB_TX_DIR     (0)  /*copy send data to buffer*/
#define CB_RX_DIR     (1)  /*recv data is being passed in*/
#define CB_TO_DIR     (3)  /*a timeout occurred; use this call to*/
                           /*manage your time critical state machines*/
/*
 * Send a note to call back so that he can update 
 * time critical states machines.
 */
#define CALL_BACK_KEEPALIVE(cb,ctx) if(cb){  \
		   CBTxRxData_t s;           \
		   s.msize  = 0;             \
		   s.offset = 0;             \
		   s.data   = NULL;          \
		   s.dlen   = 0;             \
		   s.dir    = CB_TO_DIR;     \
		   cb(&s,ctx);               \
		 }


Status_t if3_mngr_get_sa_classportInfo(ManagerInfo_t * mi);
Status_t if3_recv(ManagerInfo_t * mi, Mai_t * t, uint64_t timeout);
Status_t if3_mngr_get_port_guid(ManagerInfo_t * fp);
Status_t if3_mngr_reg_service(IBhandle_t fd, uint8_t * servName, uint64_t servID);
Status_t if3_mngr_del_service(IBhandle_t fd, uint8_t * servName, uint64_t servID, uint32_t mode);
Status_t if3_mngr_query_service(IBhandle_t fd, uint8_t * servName, uint64_t servID, uint32_t mode, IB_SERVICE_RECORD * serviceFoundp, uint32_t * count);
Status_t if3_mngr_query_srv_path(IBhandle_t fd, IB_SERVICE_RECORD * srp, STL_LID * lid, uint16_t * sl);

Status_t if3_mngr_register_sa(IBhandle_t fd, uint8_t *servName, uint64_t servID, uint32_t option);
Status_t if3_mngr_deregister_sa(IBhandle_t fd);
Status_t if3_mngr_get_fe_cmd_option(Mai_t *mad, uint32_t *cmd);
#endif
