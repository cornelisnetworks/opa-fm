/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_PUBLIC_ITCPIP_H_
#define _IBA_PUBLIC_ITCPIP_H_

#include "iba/public/datatypes.h"
#include "iba/public/ibyteswap.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "iba/public/ipackon.h"

typedef struct _IP_HEADER
{
	IB_BITFIELD2(uint8,
					Version:4,
					HeaderLength:4
				)
	uint8	TypeOfService;
	uint16	TotalLength;
	uint16	Id;
	uint16	FragmentOffset;
	uint8	TimeToLive;
	uint8	Protocol;
	uint16	CheckSum;
	uint32	SrcAddr;
	uint32	DestAddr;
} PACK_SUFFIX IP_HEADER;

typedef struct _IPV6_HEADER
{
    IB_BITFIELD2(uint8,
                    Version:4,
                    Priority:4
                )
    uint8   FlowLabel[3];
    uint16  PayloadLength;
    uint8   NextHeader;
    uint8   HopLimit;
    uint8   SrcAddr[16];
    uint8   DestAddr[16];
} PACK_SUFFIX IPV6_HEADER;

/*
 *      IPV6 extension headers
 */
#define IPPROTO_ICMPV6          58      /* ICMPv6 */


#define IP_PROTOCOL_UDP	17
typedef struct _UDP_HEADER
{
	uint16	SrcPort;
	uint16	DestPort;
	uint16	UdpLength;
	uint16	UdpCheckSum;
} PACK_SUFFIX UDP_HEADER;

/* ARP */

/* opcodes */
#define ARP_OPCODE_REQUEST			1
#define ARP_OPCODE_REPLY			2
#define ARP_OPCODE_RARP_REQUEST		3
#define ARP_OPCODE_RARP_REPLY		4
#define ARP_OPCODE_INARP_REQUEST	8
#define ARP_OPCODE_INARP_REPLY		9
#define ARP_OPCODE_NAK				10

/* protocol Hardware Ids */
#define ARP_HARDWARE_ETHERNET		1

#include "iba/public/ipackoff.h"

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_ITCPIP_H_ */
