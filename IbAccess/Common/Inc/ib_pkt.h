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

/* Infiniband low level Link, Network and Transport Layer headers.
 * Generally these are handled by the CA hardware.  However in some cases
 * for special QPs, the Verbs Provider must hand build these headers.
 * Hence this file provides a common implementation for them
 *
 * The GRH header is more visible and is defined in ib_types.h
 */

#ifndef _IBA_IB_PKT_H_
#define _IBA_IB_PKT_H_	(1)

#include "iba/public/datatypes.h"
#include "iba/stl_types.h"
#include "iba/stl_helper.h"
#include "iba/public/ibyteswap.h"
#include "iba/public/imemory.h"
#include "iba/public/ispinlock.h"

#if defined (__cplusplus)
extern "C" {
#endif

	
#include "iba/public/ipackon.h"

/*
 * IB packet headers
 */

/*
 * Local Route Header
 */
typedef struct _IB_LRH {
	struct _IB_LRH_V { IB_BITFIELD2(uint8,
		VL:				4,	/* Virtual lane */
		LinkVersion:	4)	/* Link version */
	} v;

	struct _IB_LRH_LNH { IB_BITFIELD3(uint8,
		ServiceLevel:	4,	/* service level */
		Reserved:		2,
		LNH:			2)	/* this field identifies the headers  */
							/* that follow LRH */
	} l;

	IB_LID		DestLID;			/* destination LID */

	union _IB_LRH_PKT {
		uint16	AsUINT16;

		struct _IB_LRH_PKT_S {IB_BITFIELD2(uint16,
			Reserved:	5,
			Length:		11)	/* Packet length. Identifies sizeof */
							/* packet in 4 byte increments. */
							/* This field includes first byte */
							/* of LRH to last byte before the  */
							/* variant CRC. */
		} s;
	} Packet;

	IB_LID		SrcLID;			/* source LID */
} PACK_SUFFIX IB_LRH;

/*
 * Link Next Header types (LRH.l.LNH)
 */
typedef enum {
	LNH_RAW_ETHERTYPE = 0,
	LNH_RAW_GRH,
	LNH_BTH,
	LNH_GRH
} LNH;

#define LRH_IS_GLOBAL(lrh)	(lrh->l.LNH == LNH_GRH)

static __inline void
BSWAP_IB_LRH(
	IB_LRH		*Header
	)
{
	Header->DestLID = ntoh16(Header->DestLID);
	Header->Packet.AsUINT16 = ntoh16(Header->Packet.AsUINT16);
	Header->SrcLID = ntoh16(Header->SrcLID);
}


/* Network Header (IB_GRH) is defined in ib_types.h
 * The IB_GRH structure is more visible since it is the 1st 40 bytes of all
 * UD receive buffers
 */

/*
 * Base Transport Header
 */
typedef struct _IB_BTH {
	uint8		OpCode;

	struct _IB_BTH_V {IB_BITFIELD4(uint8,
		SolicitedEvent:	1,
		Migrate:		1,
		PadCount:		2,
		HeaderVersion:	4)	/* Transport Header version */
	} v;

	IB_P_KEY	Pkey;

	union _IB_BTH_QP {
		uint32	AsUINT32;

		struct _IB_BTH_QP_S {IB_BITFIELD2(uint32,
			Reserved:		8,
			DestQPNumber:	24)
		} s;
	} Qp;

	union _IB_BTH_PSN {
		uint32	AsUINT32;

		struct _IB_BTH_PSN_S {IB_BITFIELD3(uint32,
			AckReq:			1,
			Reserved:		7,
			PSN:			24)
		} s;
	} Psn;

} PACK_SUFFIX IB_BTH;

/* opcode is composed of a 3 bit QP TYPE and a 5 bit message type */
#define BTH_OPCODE_QP_TYPE_MASK 0xE0
#define BTH_OPCODE_MSG_TYPE_MASK 0x1F

/* value for upper 3 bits of BTH_OPCODE */
typedef enum {
	BTH_OPCODE_RC = 0x00,
	BTH_OPCODE_UC = 0x20,
	BTH_OPCODE_RD = 0x40,
	BTH_OPCODE_UD = 0x60
	/* 0x80, 0xA0 reserved */
	/* 0xC0, 0xE0 manufacturer specific opcodes */
} BTH_OPCODE_QP_TYPE;


/* value for low 5 bits of BTH_OPCODE */
typedef enum {
	BTH_OPCODE_SEND_FIRST=0x00,
	BTH_OPCODE_SEND_MIDDLE=0x01,
	BTH_OPCODE_SEND_LAST=0x02,
	BTH_OPCODE_SEND_LAST_IMMED=0x03,
	BTH_OPCODE_SEND_ONLY=0x04,
	BTH_OPCODE_SEND_ONLY_IMMED=0x05,
	BTH_OPCODE_RDMA_WRITE_FIRST=0x06,
	BTH_OPCODE_RDMA_WRITE_MIDDLE=0x07,
	BTH_OPCODE_RDMA_WRITE_LAST=0x08,
	BTH_OPCODE_RDMA_WRITE_LAST_IMMED=0x09,
	BTH_OPCODE_RDMA_WRITE_ONLY=0x0A,
	BTH_OPCODE_RDMA_WRITE_ONLY_IMMED=0x0B,
	BTH_OPCODE_RDMA_READ_REQ=0x0C,
	BTH_OPCODE_RDMA_READ_RESP_FIRST=0x0D,
	BTH_OPCODE_RDMA_READ_RESP_MIDDLE=0x0E,
	BTH_OPCODE_RDMA_READ_RESP_LAST=0x0F,
	BTH_OPCODE_RDMA_READ_RESP_ONLY=0x10,
	BTH_OPCODE_ACK=0x11,
	BTH_OPCODE_ATOMIC_ACK=0x12,
	BTH_OPCODE_CMP_SWAP=0x13,
	BTH_OPCODE_FETCH_ADD=0x14,
	BTH_OPCODE_RESYNC=0x15,
	/* 0x15 - 0x1f reserved */
} BTH_OPCODE_MSG_TYPE;

/*
 * OpCode (BTH.OpCode)
 * Note not all combinations of QP_TYPE and MSG_TYPE are valid
 */
#define BTH_OPCODE_RC_SEND_FIRST (BTH_OPCODE_RC|BTH_OPCODE_SEND_FIRST)
#define BTH_OPCODE_RC_SEND_MIDDLE (BTH_OPCODE_RC|BTH_OPCODE_SEND_MIDDLE)
#define BTH_OPCODE_RC_SEND_LAST (BTH_OPCODE_RC|BTH_OPCODE_SEND_LAST)
#define BTH_OPCODE_RC_SEND_LAST_IMMED (BTH_OPCODE_RC|BTH_OPCODE_SEND_LAST_IMMED)
#define BTH_OPCODE_RC_SEND_ONLY (BTH_OPCODE_RC|BTH_OPCODE_SEND_ONLY)
#define BTH_OPCODE_RC_SEND_ONLY_IMMED (BTH_OPCODE_RC | BTH_OPCODE_SEND_ONLY_IMMED)
#define BTH_OPCODE_RC_RDMA_WRITE_FIRST (BTH_OPCODE_RC | BTH_OPCODE_RDMA_WRITE_FIRST)
#define BTH_OPCODE_RC_RDMA_WRITE_MIDDLE (BTH_OPCODE_RC | BTH_OPCODE_RDMA_WRITE_MIDDLE)
#define BTH_OPCODE_RC_RDMA_WRITE_LAST (BTH_OPCODE_RC | BTH_OPCODE_RDMA_WRITE_LAST)
#define BTH_OPCODE_RC_RDMA_WRITE_LAST_IMMED (BTH_OPCODE_RC | BTH_OPCODE_RDMA_WRITE_LAST_IMMED)
#define BTH_OPCODE_RC_RDMA_WRITE_ONLY (BTH_OPCODE_RC | BTH_OPCODE_RDMA_WRITE_ONLY)
#define BTH_OPCODE_RC_RDMA_WRITE_ONLY_IMMED (BTH_OPCODE_RC | BTH_OPCODE_RDMA_WRITE_ONLY_IMMED)
#define BTH_OPCODE_RC_RDMA_READ_REQ (BTH_OPCODE_RC | BTH_OPCODE_RDMA_READ_REQ)
#define BTH_OPCODE_RC_RDMA_READ_RESP_FIRST (BTH_OPCODE_RC | BTH_OPCODE_RDMA_READ_RESP_FIRST)
#define BTH_OPCODE_RC_RDMA_READ_RESP_MIDDLE (BTH_OPCODE_RC | BTH_OPCODE_RDMA_READ_RESP_MIDDLE)
#define BTH_OPCODE_RC_RDMA_READ_RESP_LAST (BTH_OPCODE_RC | BTH_OPCODE_RDMA_READ_RESP_LAST)
#define BTH_OPCODE_RC_RDMA_READ_RESP_ONLY (BTH_OPCODE_RC | BTH_OPCODE_RDMA_READ_RESP_ONLY)
#define BTH_OPCODE_RC_ACK (BTH_OPCODE_RC | BTH_OPCODE_ACK)
#define BTH_OPCODE_RC_ATOMIC_ACK (BTH_OPCODE_RC | BTH_OPCODE_ATOMIC_ACK)
#define BTH_OPCODE_RC_CMP_SWAP (BTH_OPCODE_RC | BTH_OPCODE_CMP_SWAP)
#define BTH_OPCODE_RC_FETCH_ADD (BTH_OPCODE_RC | BTH_OPCODE_FETCH_ADD)
	/* 0x15 - 0x1f reserved */
#define BTH_OPCODE_UC_SEND_FIRST	(BTH_OPCODE_UC | BTH_OPCODE_SEND_FIRST)
#define BTH_OPCODE_UC_SEND_MIDDLE	(BTH_OPCODE_UC | BTH_OPCODE_SEND_MIDDLE)
#define BTH_OPCODE_UC_SEND_LAST	(BTH_OPCODE_UC | BTH_OPCODE_SEND_LAST)
#define BTH_OPCODE_UC_SEND_LAST_IMMED	(BTH_OPCODE_UC | BTH_OPCODE_SEND_LAST_IMMED)
#define BTH_OPCODE_UC_SEND_ONLY	(BTH_OPCODE_UC | BTH_OPCODE_SEND_ONLY)
#define BTH_OPCODE_UC_SEND_ONLY_IMMED	(BTH_OPCODE_UC | BTH_OPCODE_SEND_ONLY_IMMED)
#define BTH_OPCODE_UC_RDMA_WRITE_FIRST	(BTH_OPCODE_UC | BTH_OPCODE_RDMA_WRITE_FIRST)
#define BTH_OPCODE_UC_RDMA_WRITE_MIDDLE	(BTH_OPCODE_UC | BTH_OPCODE_RDMA_WRITE_MIDDLE)
#define BTH_OPCODE_UC_RDMA_WRITE_LAST	(BTH_OPCODE_UC | BTH_OPCODE_RDMA_WRITE_LAST)
#define BTH_OPCODE_UC_RDMA_WRITE_LAST_IMMED	(BTH_OPCODE_UC | BTH_OPCODE_RDMA_WRITE_LAST_IMMED)
#define BTH_OPCODE_UC_RDMA_WRITE_ONLY	(BTH_OPCODE_UC | BTH_OPCODE_RDMA_WRITE_ONLY)
#define BTH_OPCODE_UC_RDMA_WRITE_ONLY_IMMED	(BTH_OPCODE_UC | BTH_OPCODE_RDMA_WRITE_ONLY_IMMED)
	/* 0x2C - 0x3f reserved */
#define BTH_OPCODE_RD_SEND_FIRST	(BTH_OPCODE_RD | BTH_OPCODE_SEND_FIRST)
#define BTH_OPCODE_RD_SEND_MIDDLE	(BTH_OPCODE_RD | BTH_OPCODE_SEND_MIDDLE)
#define BTH_OPCODE_RD_SEND_LAST	(BTH_OPCODE_RD | BTH_OPCODE_SEND_LAST)
#define BTH_OPCODE_RD_SEND_LAST_IMMED	(BTH_OPCODE_RD | BTH_OPCODE_SEND_LAST_IMMED)
#define BTH_OPCODE_RD_SEND_ONLY	(BTH_OPCODE_RD | BTH_OPCODE_SEND_ONLY)
#define BTH_OPCODE_RD_SEND_ONLY_IMMED	(BTH_OPCODE_RD | BTH_OPCODE_SEND_ONLY_IMMED)
#define BTH_OPCODE_RD_RDMA_WRITE_FIRST	(BTH_OPCODE_RD | BTH_OPCODE_RDMA_WRITE_FIRST)
#define BTH_OPCODE_RD_RDMA_WRITE_MIDDLE	(BTH_OPCODE_RD | BTH_OPCODE_RDMA_WRITE_MIDDLE)
#define BTH_OPCODE_RD_RDMA_WRITE_LAST	(BTH_OPCODE_RD | BTH_OPCODE_RDMA_WRITE_LAST)
#define BTH_OPCODE_RD_RDMA_WRITE_LAST_IMMED	(BTH_OPCODE_RD | BTH_OPCODE_RDMA_WRITE_LAST_IMMED)
#define BTH_OPCODE_RD_RDMA_WRITE_ONLY	(BTH_OPCODE_RD | BTH_OPCODE_RDMA_WRITE_ONLY)
#define BTH_OPCODE_RD_RDMA_WRITE_ONLY_IMMED	(BTH_OPCODE_RD | BTH_OPCODE_RDMA_WRITE_ONLY_IMMED)
#define BTH_OPCODE_RD_RDMA_READ_REQ	(BTH_OPCODE_RD | BTH_OPCODE_RDMA_READ_REQ)
#define BTH_OPCODE_RD_RDMA_READ_RESP_FIRST	(BTH_OPCODE_RD | BTH_OPCODE_RDMA_READ_RESP_FIRST)
#define BTH_OPCODE_RD_RDMA_READ_RESP_MIDDLE	(BTH_OPCODE_RD | BTH_OPCODE_RDMA_READ_RESP_MIDDLE)
#define BTH_OPCODE_RD_RDMA_READ_RESP_LAST	(BTH_OPCODE_RD | BTH_OPCODE_RDMA_READ_RESP_LAST)
#define BTH_OPCODE_RD_RDMA_READ_RESP_ONLY	(BTH_OPCODE_RD | BTH_OPCODE_RDMA_READ_RESP_ONLY)
#define BTH_OPCODE_RD_ACK	(BTH_OPCODE_RD | BTH_OPCODE_ACK)
#define BTH_OPCODE_RD_ATOMIC_ACK	(BTH_OPCODE_RD | BTH_OPCODE_ATOMIC_ACK)
#define BTH_OPCODE_RD_CMP_SWAP	(BTH_OPCODE_RD | BTH_OPCODE_CMP_SWAP)
#define BTH_OPCODE_RD_FETCH_ADD	(BTH_OPCODE_RD | BTH_OPCODE_FETCH_ADD)
#define BTH_OPCODE_RD_RESYNC	(BTH_OPCODE_RD | BTH_OPCODE_RESYNC)
	/* 0x56 - 0x5f reserved */
	/* 0x60 - 0x63 reserved */
#define BTH_OPCODE_UD_SEND_ONLY	(BTH_OPCODE_UD | BTH_OPCODE_SEND_ONLY)
#define BTH_OPCODE_UD_SEND_ONLY_IMMED	(BTH_OPCODE_UD | BTH_OPCODE_SEND_ONLY_IMMED)
	/* 0x66 - 0x7f reserved */
	/* 0x80 - 0xBF reserved */
	/* 0xC0 - 0xFF manufacturer specific codes */

static __inline void
BSWAP_IB_BTH(
	IB_BTH		*Header
	)
{
	Header->Pkey = ntoh16(Header->Pkey);
	Header->Qp.AsUINT32 = ntoh32(Header->Qp.AsUINT32);
	Header->Psn.AsUINT32 = ntoh32(Header->Psn.AsUINT32);
}

/*
 * Datagram Extended Transport Header
 */
typedef struct _IB_DETH {
	IB_Q_KEY	Qkey;

	union _IB_DETH_QP {
		uint32	AsUINT32;

		struct _IB_DETH_QP_S {IB_BITFIELD2(uint32,
			Reserved:		8,
			SrcQPNumber:	24)
			
		} s;
	} Qp;

} PACK_SUFFIX IB_DETH;

static __inline void
BSWAP_IB_DETH(
	IB_DETH		*Header
	)
{
	Header->Qkey = ntoh32(Header->Qkey);
	Header->Qp.AsUINT32 = ntoh32(Header->Qp.AsUINT32);
}

/*
 * Swap Immediate Data
 */
static __inline void
BSWAP_IMMED_DATA(
	uint32		*ImmDt
	)
{
	*ImmDt = ntoh32(*ImmDt);
}

typedef struct _IB_LOCAL_PKT_HEADERS {
	IB_LRH	Lrh;
	IB_BTH	Bth;
} PACK_SUFFIX IB_LOCAL_PKT_HEADERS;

typedef struct _IB_GLOBAL_PKT_HEADERS {
	IB_LRH	Lrh;
	IB_GRH	Grh;
	IB_BTH	Bth;
} PACK_SUFFIX IB_GLOBAL_PKT_HEADERS;

typedef struct _IB_LOCAL_UD_HEADERS {
	IB_LRH	Lrh;
	IB_BTH	Bth;
	IB_DETH	Deth;
} PACK_SUFFIX IB_LOCAL_UD_HEADERS;

typedef struct _IB_GLOBAL_UD_HEADERS {
	IB_LRH	Lrh;
	IB_GRH	Grh;
	IB_BTH	Bth;
	IB_DETH	Deth;
} PACK_SUFFIX IB_GLOBAL_UD_HEADERS;

typedef struct _IB_LOCAL_UD_IMM_HEADERS {
	IB_LRH	Lrh;
	IB_BTH	Bth;
	IB_DETH	Deth;
	uint32	ImmDt;
} PACK_SUFFIX IB_LOCAL_UD_IMM_HEADERS;

typedef struct _IB_GLOBAL_UD_IMM_HEADERS {
	IB_LRH	Lrh;
	IB_GRH	Grh;
	IB_BTH	Bth;
	IB_DETH	Deth;
	uint32	ImmDt;
} PACK_SUFFIX IB_GLOBAL_UD_IMM_HEADERS;


#include "iba/public/ipackoff.h"

#if defined (__cplusplus)
};
#endif

#endif /* _IBA_IB_PKT_H_ */
