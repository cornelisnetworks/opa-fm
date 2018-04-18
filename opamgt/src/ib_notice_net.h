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
/* [ICS VERSION STRING: unknown] */

#if !defined(SA_NET_H)
#define SA_NET_H

#include <asm/byteorder.h>

#include <infiniband/sa.h>

enum {
	IBV_SA_ATTR_NOTICE		= __constant_cpu_to_be16(0x02),
	IBV_SA_ATTR_INFORM_INFO		= __constant_cpu_to_be16(0x03),
	IBV_SA_ATTR_MC_MEMBER_REC	= __constant_cpu_to_be16(0x38),
	IBV_SA_ATTR_INFORM_INFO_REC	= __constant_cpu_to_be16(0xf3)
};

enum {
	IBV_SA_METHOD_GET		= 0x01,
	IBV_SA_METHOD_SET		= 0x02,
	IBV_SA_METHOD_DELETE		= 0x15
};

/* Length of SA attributes on the wire */
enum {
	IBV_SA_ATTR_NOTICE_LEN		= 80,
	IBV_SA_ATTR_INFORM_INFO_LEN	= 36,
	IBV_SA_ATTR_MC_MEMBER_REC_LEN	= 52,
	IBV_SA_ATTR_INFORM_INFO_REC_LEN	= 60
};

#define IBV_SA_COMP_MASK(n)		__constant_cpu_to_be64(1ull << n)

struct ibv_sa_net_mcmember_rec {
	uint8_t		mgid[16];
	uint8_t		port_gid[16];
	uint32_t	qkey;
	uint16_t	mlid;
	/* MtuSelector: 2:304, MTU: 6:306 */
	uint8_t		mtu_info;
	uint8_t		tclass;
	uint16_t	pkey;
	/* RateSelector: 2:336, Rate: 6:338 */
	uint8_t		rate_info;
	/* PacketLifeTimeSelector: 2:344, PacketLifeTime: 6:346 */
	uint8_t		packetlifetime_info;
	/* SL: 4:352, FlowLabel: 20:356, HopLimit: 8:376 */
	uint32_t	sl_flow_hop;
	/* Scope: 4:384, JoinState: 4:388 */
	uint8_t		scope_join;
	/* ProxyJoin: 1:392, rsvd: 7:393 */
	uint8_t		proxy_join;
	uint8_t		rsvd[2];
};

enum {
	IBV_SA_MCMEMBER_REC_MTU_SELECTOR_OFFSET		= 304,
	IBV_SA_MCMEMBER_REC_MTU_SELECTOR_LENGTH		= 2,
	IBV_SA_MCMEMBER_REC_MTU_OFFSET			= 306,
	IBV_SA_MCMEMBER_REC_MTU_LENGTH			= 6,
	IBV_SA_MCMEMBER_REC_RATE_SELECTOR_OFFSET	= 336,
	IBV_SA_MCMEMBER_REC_RATE_SELECTOR_LENGTH	= 2,
	IBV_SA_MCMEMBER_REC_RATE_OFFSET			= 338,
	IBV_SA_MCMEMBER_REC_RATE_LENGTH			= 6,
	IBV_SA_MCMEMBER_REC_PACKETLIFE_SELECTOR_OFFSET	= 344,
	IBV_SA_MCMEMBER_REC_PACKETLIFE_SELECTOR_LENGTH	= 2,
	IBV_SA_MCMEMBER_REC_PACKETLIFE_OFFSET		= 346,
	IBV_SA_MCMEMBER_REC_PACKETLIFE_LENGTH		= 6,
	IBV_SA_MCMEMBER_REC_SL_OFFSET			= 352,
	IBV_SA_MCMEMBER_REC_SL_LENGTH			= 4,
	IBV_SA_MCMEMBER_REC_FLOW_LABEL_OFFSET		= 356,
	IBV_SA_MCMEMBER_REC_FLOW_LABEL_LENGTH		= 20,
	IBV_SA_MCMEMBER_REC_HOP_LIMIT_OFFSET		= 376,
	IBV_SA_MCMEMBER_REC_HOP_LIMIT_LENGTH		= 8,
	IBV_SA_MCMEMBER_REC_SCOPE_OFFSET		= 384,
	IBV_SA_MCMEMBER_REC_SCOPE_LENGTH		= 4,
	IBV_SA_MCMEMBER_REC_JOIN_STATE_OFFSET		= 388,
	IBV_SA_MCMEMBER_REC_JOIN_STATE_LENGTH		= 4,
	IBV_SA_MCMEMBER_REC_PROXY_JOIN_OFFSET		= 392,
	IBV_SA_MCMEMBER_REC_PROXY_JOIN_LENGTH		= 1
};

enum {
	IBV_SA_MCMEMBER_REC_MGID			= IBV_SA_COMP_MASK(0),
	IBV_SA_MCMEMBER_REC_PORT_GID			= IBV_SA_COMP_MASK(1),
	IBV_SA_MCMEMBER_REC_QKEY			= IBV_SA_COMP_MASK(2),
	IBV_SA_MCMEMBER_REC_MLID			= IBV_SA_COMP_MASK(3),
	IBV_SA_MCMEMBER_REC_MTU_SELECTOR		= IBV_SA_COMP_MASK(4),
	IBV_SA_MCMEMBER_REC_MTU				= IBV_SA_COMP_MASK(5),
	IBV_SA_MCMEMBER_REC_TRAFFIC_CLASS		= IBV_SA_COMP_MASK(6),
	IBV_SA_MCMEMBER_REC_PKEY			= IBV_SA_COMP_MASK(7),
	IBV_SA_MCMEMBER_REC_RATE_SELECTOR		= IBV_SA_COMP_MASK(8),
	IBV_SA_MCMEMBER_REC_RATE			= IBV_SA_COMP_MASK(9),
	IBV_SA_MCMEMBER_REC_PACKET_LIFE_TIME_SELECTOR	= IBV_SA_COMP_MASK(10),
	IBV_SA_MCMEMBER_REC_PACKET_LIFE_TIME		= IBV_SA_COMP_MASK(11),
	IBV_SA_MCMEMBER_REC_SL				= IBV_SA_COMP_MASK(12),
	IBV_SA_MCMEMBER_REC_FLOW_LABEL			= IBV_SA_COMP_MASK(13),
	IBV_SA_MCMEMBER_REC_HOP_LIMIT			= IBV_SA_COMP_MASK(14),
	IBV_SA_MCMEMBER_REC_SCOPE			= IBV_SA_COMP_MASK(15),
	IBV_SA_MCMEMBER_REC_JOIN_STATE			= IBV_SA_COMP_MASK(16),
	IBV_SA_MCMEMBER_REC_PROXY_JOIN			= IBV_SA_COMP_MASK(17)
};

/* InformInfo:Type */
enum {
	IBV_SA_EVENT_TYPE_FATAL		= __constant_cpu_to_be16(0x0),
	IBV_SA_EVENT_TYPE_URGENT	= __constant_cpu_to_be16(0x1),
	IBV_SA_EVENT_TYPE_SECURITY	= __constant_cpu_to_be16(0x2),
	IBV_SA_EVENT_TYPE_SM		= __constant_cpu_to_be16(0x3),
	IBV_SA_EVENT_TYPE_INFO		= __constant_cpu_to_be16(0x4),
	IBV_SA_EVENT_TYPE_EMPTY		= __constant_cpu_to_be16(0x7F),
	IBV_SA_EVENT_TYPE_ALL		= __constant_cpu_to_be16(0xFFFF)
};

/* InformInfo:TrapNumber */
enum {
	IBV_SA_SM_TRAP_GID_IN_SERVICE		   = __constant_cpu_to_be16(64),
	IBV_SA_SM_TRAP_GID_OUT_OF_SERVICE	   = __constant_cpu_to_be16(65),
	IBV_SA_SM_TRAP_CREATE_MC_GROUP		   = __constant_cpu_to_be16(66),
	IBV_SA_SM_TRAP_DELETE_MC_GROUP		   = __constant_cpu_to_be16(67),
	IBV_SA_SM_TRAP_PORT_CHANGE_STATE	   = __constant_cpu_to_be16(128),
	IBV_SA_SM_TRAP_LINK_INTEGRITY		   = __constant_cpu_to_be16(129),
	IBV_SA_SM_TRAP_EXCESSIVE_BUFFER_OVERRUN	   = __constant_cpu_to_be16(130),
	IBV_SA_SM_TRAP_FLOW_CONTROL_UPDATE_EXPIRED = __constant_cpu_to_be16(131),
	IBV_SA_SM_TRAP_BAD_M_KEY		   = __constant_cpu_to_be16(256),
	IBV_SA_SM_TRAP_BAD_P_KEY		   = __constant_cpu_to_be16(257),
	IBV_SA_SM_TRAP_BAD_Q_KEY		   = __constant_cpu_to_be16(258),
	IBV_SA_SM_TRAP_ALL			   = __constant_cpu_to_be16(0xFFFF)
};

/* InformInfo:ProducerType */
enum {
	IBV_SA_EVENT_PRODUCER_TYPE_CA		 = __constant_cpu_to_be32(0x1),
	IBV_SA_EVENT_PRODUCER_TYPE_SWITCH	 = __constant_cpu_to_be32(0x2),
	IBV_SA_EVENT_PRODUCER_TYPE_ROUTER	 = __constant_cpu_to_be32(0x3),
	IBV_SA_EVENT_PRODUCER_TYPE_CLASS_MANAGER = __constant_cpu_to_be32(0x4),
	IBV_SA_EVENT_PRODUCER_TYPE_ALL		 = __constant_cpu_to_be32(0xFFFFFF)
};

struct ibv_sa_net_inform_info {
	uint8_t		gid[16];
	uint16_t	lid_range_begin;
	uint16_t	lid_range_end;
	uint16_t	rsvd;
	uint8_t		is_generic;
	uint8_t		subscribe;
	uint16_t	type;
	/* TrapNumber/DeviceID */
	uint16_t	trap_num_device_id;
	/* QPN: 24:224, rsvd: 3:248, RespTimeValue: 5:251 */
	uint32_t	qpn_resptime;
	/* rsvd: 8:256, ProducerType/VendorID: 24:264 */
	uint32_t	producer_type_vendor_id;
};

enum {
	IBV_SA_INFORM_INFO_QPN_OFFSET			= 224,
	IBV_SA_INFORM_INFO_QPN_LENGTH			= 24,
	IBV_SA_INFORM_INFO_RESP_TIME_OFFSET		= 251,
	IBV_SA_INFORM_INFO_RESP_TIME_LENGTH		= 5,
	IBV_SA_INFORM_INFO_PRODUCER_TYPE_OFFSET		= 264,
	IBV_SA_INFORM_INFO_PRODUCER_TYPE_LENGTH		= 24,
	IBV_SA_INFORM_INFO_VENDOR_ID_OFFSET		= 264,
	IBV_SA_INFORM_INFO_VENDOR_ID_LENGTH		= 24
};

struct ibv_sa_net_notice {
	/* IsGeneric: 1:0, Type: 7:1, ProducerType/VendorID: 24:8 */
	uint32_t	generic_type_producer;
	/* TrapNumber/DeviceID */
	uint16_t	trap_num_device_id;
	uint16_t	issuer_lid;
	/* NoticeToggle: 1:64, NoticeCount: 15:65 */
	uint16_t	toggle_count;
	uint8_t		data_details[54];
	uint8_t		issuer_gid[16];
};

enum {
	IBV_SA_NOTICE_IS_GENERIC_OFFSET			= 0,
	IBV_SA_NOTICE_IS_GENERIC_LENGTH			= 1,
	IBV_SA_NOTICE_TYPE_OFFSET			= 1,
	IBV_SA_NOTICE_TYPE_LENGTH			= 7,
	IBV_SA_NOTICE_PRODUCER_TYPE_OFFSET		= 8,
	IBV_SA_NOTICE_PRODUCER_TYPE_LENGTH		= 24,
	IBV_SA_NOTICE_VENDOR_ID_OFFSET			= 8,
	IBV_SA_NOTICE_VENDOR_IF_LENGTH			= 24,
	IBV_SA_NOTICE_TOGGLE_OFFSET			= 64,
	IBV_SA_NOTICE_TOGGLE_LENGTH			= 1,
	IBV_SA_NOTICE_COUNT_OFFSET			= 65,
	IBV_SA_NOTICE_COUNT_LENGTH			= 15,
};

/*
 * SM notice data details for:
 *
 * IB_SA_SM_TRAP_GID_IN_SERVICE		= 64
 * IB_SA_SM_TRAP_GID_OUT_OF_SERVICE	= 65
 * IB_SA_SM_TRAP_CREATE_MC_GROUP	= 66
 * IB_SA_SM_TRAP_DELETE_MC_GROUP	= 67
 */
struct ibv_sa_net_notice_data_gid {
	uint8_t	reserved[6];
	uint8_t	gid[16];
	uint8_t	padding[32];
};

/*
 * SM notice data details for:
 *
 * IB_SA_SM_TRAP_PORT_CHANGE_STATE	= 128
 */
struct ibv_sa_net_notice_data_port_change {
	uint16_t lid;
	uint8_t	 padding[52];
};

/*
 * SM notice data details for:
 *
 * IB_SA_SM_TRAP_LINK_INTEGRITY			= 129
 * IB_SA_SM_TRAP_EXCESSIVE_BUFFER_OVERRUN	= 130
 * IB_SA_SM_TRAP_FLOW_CONTROL_UPDATE_EXPIRED	= 131
 */
struct ibv_sa_net_notice_data_port_error {
	uint8_t	 reserved[2];
	uint16_t lid;
	uint8_t	 port_num;
	uint8_t	 padding[49];
};

/**
 * ibv_sa_get_field - Extract a bit field value from a structure.
 * @data: Pointer to the start of the structure.
 * @offset: Bit offset of field from start of structure.
 * @size: Size of field, in bits.
 *
 * The structure must be in network-byte order.  The returned value is in
 * host-byte order.
 */
uint32_t ibv_sa_get_field(void *data, int offset, int size);

/**
 * ibv_sa_set_field - Set a bit field value in a structure.
 * @data: Pointer to the start of the structure.
 * @value: Value to assign to field.
 * @offset: Bit offset of field from start of structure.
 * @size: Size of field, in bits.
 *
 * The structure must be in network-byte order.  The value to set is in
 * host-byte order.
 */
void ibv_sa_set_field(void *data, uint32_t value, int offset, int size);

#endif /* SA_NET_H */
