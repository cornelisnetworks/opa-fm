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
* 
* FILE NAME
*       fe_mad.h
*
* DESCRIPTION
*       Fabric Executive MAD support data structures
* 
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   1/23/2000       Initial version for checkin to source control
* jrw   12/03/2001  Changes from code review    
***********************************************************************/
#ifndef FE_MAD_H
#define FE_MAD_H

#include "net/libnet.h"
#include "fe_main.h"
#include "fe.h"
#include "sa/if3_sa.h"

/* Variables    */
extern uint8_t *g_fe_oob_send_buf;
extern uint8_t *g_fe_recv_buf;
extern Pool_t fe_pool;

/* Function Prototypes */
uint32_t fe_vieo_init(uint8_t *);
void fe_shutdown(void);
uint32_t fe_sa_passthrough(uint8_t *, FE_ConnList *, IBhandle_t);
uint32_t fe_pa_passthrough(uint8_t *, FE_ConnList *, IBhandle_t);
uint32_t fe_ea_passthrough(uint8_t *, FE_ConnList *, IBhandle_t);
uint32_t fe_unsolicited(FE_ConnList *, uint32_t *);
uint32_t fe_get_config_file_checksum(NetConnection *conn, uint32_t mid);
uint32_t fe_get_config_file_timestamp(NetConnection *conn, uint32_t mid);

#endif
