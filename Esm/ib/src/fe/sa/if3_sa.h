/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/***********************************************************************
* 
* FILE NAME
*   if3_sa.h
*
* DESCRIPTION
*    Interface 3 functions which commuincate with the SA 
*
*
***********************************************************************/
#ifndef IF3_SA_H
#define IF3_SA_H

#include "net/libnet.h"
#include "fe.h"

/* Packet Structure */

struct Packet_t
{
  uint16_t aid;           /* Attribute ID */
  uint32_t amod;          /* Attribute Modifier */
  uint8_t method;         /* Method */
  uint64_t mask;          /* Make */
  uint8_t* rec;           /* In Data */
  uint32_t recsize;       /* Size of In Data */
  uint8_t* data;         /* Out Data */
  uint32_t sz;           /* Size of Out Data */
  uint32_t status;      /* Status */
};

typedef struct Packet_t Packet_t;

typedef struct
{
	uint32_t found;
	uint32_t toBeProcessed;
	Lid_t smlid;
	uint32_t trap128Received;
} FE_Trap_Processing_State_t;

extern uint32_t fe_if3_getInfo(Packet_t* );
extern uint32_t fe_if3_subscribe_sa(void);
extern uint32_t fe_if3_unsubscribe_sa(int);
extern void     fe_if3_ib_notice_to_fe_trap(STL_NOTICE *notice, FE_Trap_t *current, FE_Trap_Processing_State_t *state);
extern uint32_t fe_if3_get_traps(FE_Trap_t *trapinfo,uint32_t* found);
extern uint32_t fe_if3_sa_check(void);
extern IBhandle_t filterh;

#endif






























