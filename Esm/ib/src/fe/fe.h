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

/*******************************************************************************
*
* FILE NAME
*       fe.h
*
* DESCRIPTION
*       Common structures and defines for the messaging interface. 
*       Interface 3 populates these structures which are in turn are sent 
*       through interface 2.            
*
* DATA STRUCTURES
*
* FUNCTIONS
*       None 
*
* DEPENDENCIES
*       None 
*
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* jrw   09/27/2001      Initial version for checkin to source control
* jrw   10/30/2001      Code cleanup  
* jrw   12/03/2001      Changes from code review 
*
***********************************************************************/
#ifndef FE_H 
#define FE_H 
#include "net/libnet.h"

/* Configuration Defines    */
#define PSWD_SIZE       16
#define USERNAME_SIZE   16


/**************************************************************************
* 		OOB PROTOCOL STRUCTURES
**************************************************************************/

/* Header byte swap for OOB network transmission */
static __inline
void BSWAP_OOB_HEADER(OOBHeader *header)
{
    header->HeaderVersion = ntoh32(header->HeaderVersion);
    header->Length = ntoh32(header->Length);
    header->Reserved[0] = 0;
    header->Reserved[1] = 0;
}

/* Byte swap for OOB packet */
static __inline
void BSWAP_OOB_PACKET(OOBPacket *packet)
{
    BSWAP_OOB_HEADER(&(packet->Header));
    BSWAP_MAD_HEADER((MAD*) &(packet->MadData));
}

/**************************************************************************
*       UNSOLICITED NOTICE STRUCTURES
**************************************************************************/

/* Unsolicited Notice / Traps Structure */
struct Trap {
    uint32_t    trapType;           /* Type of Trap                 */
    STL_LID     lidAddr;            /* LID Address                  */
    uint8_t     portNum;            /* Port number                  */
	STL_NOTICE  notice;
    struct Trap *next;
};
typedef struct Trap FE_Trap_t;

#endif

