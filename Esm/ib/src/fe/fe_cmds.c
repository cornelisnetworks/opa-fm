/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

/************************************************************************
* 
* FILE NAME
*       fe_cmds.c
*
* DESCRIPTION
*       Command processor for Fabric Executice Messaging System
*
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   01/16/2000      Initial version for checkin
* jrw   12/03/2001  Changes from code review
***********************************************************************/
#include "fe_main.h"
#include "fe.h"
#include "fe_mad.h"
#include "fe_cmds.h"

uint32_t fe_send_packet(NetConnection *conn, uint8_t *buf, int32_t len)
{
    NetError    err;
    OOBPacket* packet; 		/* Pointer to out of band packet */
    uint32_t    rc = SUCCESS;

    IB_ENTER(__func__, buf, len, 0, 0);
	packet = (OOBPacket *) buf;
	packet->Header.Length = len - sizeof(OOBHeader);
	BSWAP_OOB_PACKET(packet);

    IB_LOG_INFO0("Sending packet");

    /* Do nothing if no conn */
    if (!conn) {
        IB_LOG_ERROR0("failed: no connection");
        return(FAILED); 
    }

    if (fe_net_send(conn,(void *)buf,len,&err)) {
        IB_LOG_ERROR("Error err:", err);
        rc = FAILED; 
    }

    IB_EXIT(__func__, rc);
     return(rc);
}
   
