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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _OMGT_OOB_PROTOCOL_H_
#define _OMGT_OOB_PROTOCOL_H_

#include "iba/ib_generalServices.h"

/**************************************************************************
*		OOB PROTOCOL STRUCTURES
**************************************************************************/
typedef struct __OOB_HEADER {
	uint32_t HeaderVersion; /* Version of the FE protocol header */
	uint32_t Length;        /* Length of the message data payload */
	uint32_t Reserved[2];   /* Reserved */
} OOB_HEADER;

/* Header byte swap for OOB network transmission */
static __inline
void BSWAP_OOB_HEADER(OOB_HEADER *header)
{
	header->HeaderVersion = ntoh32(header->HeaderVersion);
	header->Length = ntoh32(header->Length);
	header->Reserved[0] = 0;
	header->Reserved[1] = 0;
}

typedef struct _OOB_PACKET {
	OOB_HEADER Header;
	MAD_RMPP MadData;
} OOB_PACKET;

#endif /* _OMGT_OOB_PROTOCOL_H_ */
