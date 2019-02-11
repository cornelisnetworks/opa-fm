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
*       fe_cmds.h
*
* DESCRIPTION
*       Common defines for FE messaging interface.  Defined 
*       commands/responses and return codes to intereface 2.
*
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   01/16/2000      Initial version for checkin to source control
* jrw   12/03/2001      Changes from code review
*
***********************************************************************/
#ifndef __FE_CMDS_H
#define __FE_CMDS_H

/* FE Return Codes -------------------------------------------------------*/

#define FE_SUCCESS                      0x0000
#define FE_UNSUPPORTED                  0x0001
#define FE_NO_COMPLETE                  0x0002
#define FE_TIMEOUT                      0x0003
#define FE_GUID_INVALID                 0x0004
#define FE_PORT_INDEX_INVALID           0x0005
#define FE_INVALID_PARAMETER            0x0006
#define FE_FATAL                        0x0007
#define FE_MANAGER_DOWN                 0x0008
#define FE_UNLOGGED                     0x000E  /* Connection not logged in  */
#define FE_UNSYNCHONIZED_VERSION        0x0014  
#define FE_NO_RETRIEVE                  0x0020

/* Unsolicited Message type --------------------------------------------------*/

#define SM_CONN_LOST                    0x00051001
#define SM_CONN_ESTABLISH               0x00051002
#define UNSOLICITED_TOPO_CHANGE         0x00053001
#define SUBNET_PORT_ACTIVE              0x00053002
#define SUBNET_PORT_INACTIVE            0x00053003

#endif

