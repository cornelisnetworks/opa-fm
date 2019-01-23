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

#include <iba/ipublic.h>
#if !defined(VXWORKS) || defined(BUILD_DMC)
#include <iba/ib_dm.h>
#endif
#include <iba/ib_sm_priv.h>
#include <iba/ib_sa_records_priv.h>
#include "iba/stl_sa_priv.h"
#include "iba/stl_pm.h"
#if defined(USE_NETF1_IP_STACK)
/* add this to avoid implicit declaration warning in mips/netf1 build */
extern int snprintf (char *str, size_t count, const char *fmt, ...);
#endif
#include "iba/stl_helper.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#define _GNU_SOURCE

#include "ixml_ib.h"

#ifdef VXWORKS
#include "bspcommon/h/icsBspUtil.h"
#endif

/* should be defined in ib_dm.h */
#ifndef IOC_IDSTRING_SIZE
#define IOC_IDSTRING_SIZE 64
#endif
#ifndef IOC_SERVICE_NAME_SIZE
#define IOC_SERVICE_NAME_SIZE 40
#endif

#ifndef SCNx8
#define SCNx8 "x"
#endif

/* sometimes a bitfield, so need to call with value instead of ptr */
void IXmlOutputLIDValue(IXmlOutputState_t *state, const char *tag, STL_LID value)
{
	IXmlOutputHexPad32(state, tag, value);
}

// only output if value != 0
void IXmlOutputOptionalLIDValue(IXmlOutputState_t *state, const char *tag, STL_LID value)
{
	if (value)
		IXmlOutputLIDValue(state, tag, value);
}

void IXmlOutputLID(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputHexPad32(state, tag, *(STL_LID*)data);
}

// only output if value != 0
void IXmlOutputOptionalLID(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalHexPad32(state, tag, *(STL_LID*)data);
}


void IXmlOutputPKey(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputHexPad16(state, tag, (*(STL_PKEY_ELEMENT*)data).AsReg16);
}

// only output if value != 0
void IXmlOutputOptionalPKey(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalHexPad16(state, tag, (*(STL_PKEY_ELEMENT*)data).AsReg16);
}


void IXmlOutputGID(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputPrintIndent(state, "<%s>0x%016"PRIx64":0x%016"PRIx64"</%s>\n", tag,
				((IB_GID*)data)->AsReg64s.H,
				((IB_GID*)data)->AsReg64s.L, tag);
}

// only output if value != 0
void IXmlOutputOptionalGID(IXmlOutputState_t *state, const char *tag, void *data)
{
	if (0ULL != ((IB_GID*)data)->AsReg64s.H || 0ULL != ((IB_GID*)data)->AsReg64s.L)
		IXmlOutputGID(state, tag, data);
}

void IXmlOutputNodeType(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStrUint(state, tag, StlNodeTypeToText(*(uint8*)data), *(uint8*)data);
}

void IXmlOutputOptionalNodeType(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalStrUint(state, tag, StlNodeTypeToText(*(uint8*)data), *(uint8*)data);
}

// parse NodeType string into a uint8 field and validate value
void IXmlParserEndNodeType(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (0 == strcasecmp(content, "FI")) {
		value = STL_NODE_FI;
	} else if (0 == strcasecmp(content, "SW")) {
		value = STL_NODE_SW;
	} else {
		IXmlParserPrintError(state, "Invalid Node type: '%s'  Must be FI or SW", content);
		goto fail;
	}
	ASSERT(field->size == 1);
	*(uint8 *)IXmlParserGetField(field, object) = value;
fail:
	return;
}

// parse NodeType_Int into a uint8 field and validate value
void IXmlParserEndNodeType_Int(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value)) {
		if (value != STL_NODE_FI
			&& value != STL_NODE_SW) {
			IXmlParserPrintError(state, "Invalid Node type: %u  Must be (FI or SW): %u or %u", value, STL_NODE_FI, STL_NODE_SW);
		} else {
			ASSERT(field->size == 1);
			*(uint8 *)IXmlParserGetField(field, object) = value;
		}
	}
}


void IXmlOutputNodeDesc(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStrLen(state, tag, (const char*)data, NODE_DESCRIPTION_ARRAY_SIZE);
}

// only output if value != ""
void IXmlOutputOptionalNodeDesc(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalStrLen(state, tag, (const char*)data, NODE_DESCRIPTION_ARRAY_SIZE);
}

void IXmlOutputIocIDString(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStrLen(state, tag, (const char*)data, IOC_IDSTRING_SIZE);
}

// only output if value != ""
void IXmlOutputOptionalIocIDString(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalStrLen(state, tag, (const char*)data, IOC_IDSTRING_SIZE);
}

void IXmlOutputServiceName(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStrLen(state, tag, (const char*)data, IOC_SERVICE_NAME_SIZE);
}

// only output if value != ""
void IXmlOutputOptionalServiceName(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalStrLen(state, tag, (const char*)data, IOC_SERVICE_NAME_SIZE);
}

/* typically a bitfield, so need to call with value instead of ptr */
void IXmlOutputPortStateValue(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputStrUint(state, tag, StlPortStateToText(value), value);
}

// only output if value != 0
void IXmlOutputOptionalPortStateValue(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputOptionalStrUint(state, tag, IbPortStateToText(value), value);
}

void IXmlOutputPortState(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputPortStateValue(state, tag, *(uint8 *)data);
}

// only output if value != 0
void IXmlOutputOptionalPortState(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalPortStateValue(state, tag, *(uint8 *)data);
}

/* link init reason */
void IXmlOutputInitReasonValue(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputStrUint(state, tag, StlLinkInitReasonToText(value), value);
}

void IXmlOutputOptionalInitReasonValue(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputOptionalStrUint(state, tag, StlLinkInitReasonToText(value), value);
}

void IXmlOutputInitReason(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputInitReasonValue(state, tag, *(uint8 *)data);
}

void IXmlOutputOptionalInitReason(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalInitReasonValue(state, tag, *(uint8 *)data);
}

/* typically a bitfield, so need to call with value instead of ptr */
void IXmlOutputPortPhysStateValue(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputStrUint(state, tag, StlPortPhysStateToText(value), value);
}

// only output if value != 0
void IXmlOutputOptionalPortPhysStateValue(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputOptionalStrUint(state, tag, StlPortPhysStateToText(value), value);
}

void IXmlOutputPortPhysState(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputPortPhysStateValue(state, tag, *(uint8 *)data);
}

// only output if value != 0
void IXmlOutputOptionalPortPhysState(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalPortPhysStateValue(state, tag, *(uint8 *)data);
}

/* typically a bitfield, so need to call with value instead of ptr */
void IXmlOutputPortDownDefaultValue(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputStrUint(state, tag, IbPortDownDefaultToText(value), value);
}

// only output if value != 0
void IXmlOutputOptionalPortDownDefaultValue(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputOptionalStrUint(state, tag, IbPortDownDefaultToText(value), value);
}

void IXmlOutputPortDownDefault(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputPortDownDefaultValue(state, tag, *(uint8 *)data);
}

// only output if value != 0
void IXmlOutputOptionalPortDownDefault(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalPortDownDefaultValue(state, tag, *(uint8 *)data);
}


/* typically a bitfield, so need to call with value instead of ptr */
/* 0 has meaning, so no 'Optional' variations of this function */
void IXmlOutputMKeyProtectValue(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputStrUint(state, tag, IbMKeyProtectToText(value), value);
}

void IXmlOutputMKeyProtect(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputMKeyProtectValue(state, tag, *(uint8 *)data);
}
/* typically a bitfield, so need to call with value instead of ptr */
void IXmlOutputRateValue(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputStrUint(state, tag, StlStaticRateToText(value), value);
}

// only output if value != 0
void IXmlOutputOptionalRateValue(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputOptionalStrUint(state, tag, StlStaticRateToText(value), value);
}

void IXmlOutputRate(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputRateValue(state, tag, *(uint8 *)data);
}

// only output if value != 0
void IXmlOutputOptionalRate(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalRateValue(state, tag, *(uint8 *)data);
}

// parse rates to uint8 multiplier string - validate rate inputs
boolean IXmlParseRateMult_Str(IXmlParserState_t *state, XML_Char *content, uint8 *value)
{
	if (!content || strlen(content) == 0) {
		IXmlParserPrintError(state, "Empty contents");
		return FALSE;
	}

	// skip whitespace
	while (isspace(*content)) {
		content++;
	}

	if (0 == strcasecmp(content, "25g") || 0 == strcasecmp(content, "25")) {
		*value = IB_STATIC_RATE_25G;
	} else if (0 == strcasecmp(content, "50g") || 0 == strcasecmp(content, "50")) {
		*value = IB_STATIC_RATE_56G; // STL_STATIC_RATE_50G
	} else if (0 == strcasecmp(content, "12.5g") || 0 == strcasecmp(content, "12.5")) {
		*value = IB_STATIC_RATE_14G; // STL_STATIC_RATE_12_5G
	} else if (0 == strcasecmp(content, "37.5g") || 0 == strcasecmp(content, "37.5")) {
		*value = IB_STATIC_RATE_40G; // STL_STATIC_RATE_37_5G
	} else if (0 == strcasecmp(content, "75g") || 0 == strcasecmp(content, "75")) {
		*value = IB_STATIC_RATE_80G; // STL_STATIC_RATE_75G
	} else if (0 == strcasecmp(content, "100g") || 0 == strcasecmp(content, "100")) {
		*value = IB_STATIC_RATE_100G;
#if 0
	// future
	} else if (0 == strcasecmp(content, "225g") || 0 == strcasecmp(content, "250")) {
		*value = STL_STATIC_RATE_225G;
	} else if (0 == strcasecmp(content, "300g") || 0 == strcasecmp(content, "300")) {
		*value = IB_STATIC_RATE_300G;
	} else if (0 == strcasecmp(content, "400g") || 0 == strcasecmp(content, "400")) {
		*value = STL_STATIC_RATE_400G;
#endif
	} else {
		IXmlParserPrintError(state, "Invalid Rate: '%s'  Must be 25g, 50g, 75g, 100g", content);
		return FALSE;
	}
	return TRUE;
}

// parse Rate string into a uint8 field and validate value
void IXmlParserEndRate(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (0 == strcasecmp(content, "12.5g") || 0 == strcasecmp(content, "12.5")) {
		value = IB_STATIC_RATE_14G; // STL_STATIC_RATE_12_5G
	} else if (0 == strcasecmp(content, "25g") || 0 == strcasecmp(content, "25")) {
		value = IB_STATIC_RATE_25G;
	} else if (0 == strcasecmp(content, "37.5g") || 0 == strcasecmp(content, "37.5")) {
		value = IB_STATIC_RATE_40G; // STL_STATIC_RATE_37_5G
	} else if (0 == strcasecmp(content, "50g") || 0 == strcasecmp(content, "50")) {
		value = IB_STATIC_RATE_56G; // STL_STATIC_RATE_50G
	} else if (0 == strcasecmp(content, "75g") || 0 == strcasecmp(content, "75")) {
		value = IB_STATIC_RATE_80G; // STL_STATIC_RATE_75G
	} else if (0 == strcasecmp(content, "100g") || 0 == strcasecmp(content, "100")) {
		value = IB_STATIC_RATE_100G;
#if 0
	// future
	} else if (0 == strcasecmp(content, "200g") || 0 == strcasecmp(content, "200")) {
		value = IB_STATIC_RATE_200G;
	} else if (0 == strcasecmp(content, "225g") || 0 == strcasecmp(content, "225")) {
		value = STL_STATIC_RATE_225G;
	} else if (0 == strcasecmp(content, "300g") || 0 == strcasecmp(content, "300")) {
		value = IB_STATIC_RATE_300G;
	} else if (0 == strcasecmp(content, "400g") || 0 == strcasecmp(content, "400")) {
		value = STL_STATIC_RATE_400G;
#endif
	} else {
		IXmlParserPrintError(state, "Invalid Rate: '%s'  Must be 12.5g, 25g, 37.5g, 50g, 75g, 100g", content);
		goto fail;
	}
	ASSERT(field->size == 1);
	*(uint8 *)IXmlParserGetField(field, object) = value;
fail:
	return;
}

// parse Rate_Int into a uint8 field and validate value
void IXmlParserEndRate_Int(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value)) {
		if (value != IB_STATIC_RATE_14G && value != IB_STATIC_RATE_25G
			&& value != IB_STATIC_RATE_40G && value != IB_STATIC_RATE_80G
			&& value != IB_STATIC_RATE_56G && value != IB_STATIC_RATE_100G ) {
			IXmlParserPrintError(state, "Invalid Rate: %u  Must be (12.5g, 25g, 37.5g, 50g, 75g, 100g): %u, %u, %u, %u, %u, %u", value,
					IB_STATIC_RATE_14G, IB_STATIC_RATE_25G, 
					IB_STATIC_RATE_40G, IB_STATIC_RATE_56G,
					IB_STATIC_RATE_80G, IB_STATIC_RATE_100G ); 
		} else {
			ASSERT(field->size == 1);
			*(uint8 *)IXmlParserGetField(field, object) = value;
		}
	}
}

/* typically a bitfield, so need to call with value instead of ptr */
/* 0 has meaning, so no 'Optional' variations of this function */
void IXmlOutputLinkWidthValue(IXmlOutputState_t *state, const char* tag, uint16 value)
{
	char tempBuf[64];
	IXmlOutputStrUint(state, tag, StlLinkWidthToText(value, tempBuf, sizeof(tempBuf)), value);
}

void IXmlOutputLinkWidth(IXmlOutputState_t *state, const char* tag, void *data)
{
	char tempBuf[64];
	IXmlOutputStrUint(state, tag, StlLinkWidthToText(*(uint16*)data, tempBuf, sizeof(tempBuf)), *(uint16*)data);
}

/* typically a bitfield, so need to call with value instead of ptr */
/* 0 has meaning, so no 'Optional' variations of this function */
void IXmlOutputLinkSpeedValue(IXmlOutputState_t *state, const char* tag, uint16 value)
{
	char tempBuf[64];
	IXmlOutputStrUint(state, tag, StlLinkSpeedToText(value, tempBuf, sizeof(tempBuf)), value);
}

void IXmlOutputLinkSpeed(IXmlOutputState_t *state, const char* tag, void *data)
{
	char tempBuf[64];
	IXmlOutputStrUint(state, tag, StlLinkSpeedToText(*(uint16*)data, tempBuf, sizeof(tempBuf)), *(uint16*)data);
}

/* typically a bitfield, so need to call with value instead of ptr */
void IXmlOutputMtuValue(IXmlOutputState_t *state, const char* tag, uint16 value)
{
	IXmlOutputUint(state, tag, GetBytesFromMtu(value));
}

// only output if value != 0
void IXmlOutputOptionalMtuValue(IXmlOutputState_t *state, const char* tag, uint16 value)
{
	IXmlOutputOptionalUint(state, tag, GetBytesFromMtu(value));
}

void IXmlOutputMtu(IXmlOutputState_t *state, const char* tag, void *data)
{
	IXmlOutputUint(state, tag, GetBytesFromMtu(*(uint8*)data));
}

// only output if value != 0
void IXmlOutputOptionalMtu(IXmlOutputState_t *state, const char* tag, void *data)
{
	IXmlOutputOptionalUint(state, tag, GetBytesFromMtu(*(uint8*)data));
}

// parse Mtu field, validate value and store as an IB_MTU
void IXmlParserEndMtu(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint16 value;
	
	if (IXmlParseUint16(state, content, len, &value)) {
		if (value != 256 && value != 512 && value != 1024
			&& value != 2048 && value != 4096 && value != 8192 && value != 10240) {
			IXmlParserPrintError(state, "Invalid Mtu: %u  Must be (256, 512, 1024, 2048, 4096, 8192, or 10240)", value);
		} else {
			ASSERT(field->size == 1);
			*(uint8 *)IXmlParserGetField(field, object) = GetMtuFromBytes(value);
		}
	}
}

/* typically a bitfield, so need to call with value instead of ptr */
void IXmlOutputVLsValue(IXmlOutputState_t *state, const char* tag, uint8 value)
{
	char buf[8];

	snprintf(buf, sizeof(buf), "%u+1", value);
	IXmlOutputStrUint(state, tag, buf, value);
}

// only output if value != 0
void IXmlOutputOptionalVLsValue(IXmlOutputState_t *state, const char* tag, uint8 value)
{
	if (value)
		IXmlOutputVLsValue(state, tag, value);
}

void IXmlOutputVLs(IXmlOutputState_t *state, const char* tag, void *data)
{
	IXmlOutputVLsValue(state, tag, *(uint8*)data);
}

// only output if value != 0
void IXmlOutputOptionalVLs(IXmlOutputState_t *state, const char* tag, void *data)
{
	IXmlOutputOptionalVLsValue(state, tag, *(uint8*)data);
}

/* typically a bitfield, so need to call with value instead of ptr */
/* 0 has meaning, so no 'Optional' variations of this function */
void IXmlOutputInitTypeValue(IXmlOutputState_t *state, const char* tag, uint8 value)
{
	char buf[80];
 
	snprintf(buf, 80, "%s%s%s%s",
		value & PORT_INIT_TYPE_NOLOAD?"NL ": "",
		value & PORT_INIT_TYPE_PRESERVE_CONTENT?"PC ": "",
		value & PORT_INIT_TYPE_PRESERVE_PRESENCE?"PP ": "",
		value & PORT_INIT_TYPE_DO_NOT_RESUSCITATE?"NR ": "");

	IXmlOutputStrUint(state, tag, buf, value);
}

void IXmlOutputInitType(IXmlOutputState_t *state, const char* tag, void *data)
{
	IXmlOutputInitTypeValue(state, tag, *(uint8*)data);
}

/* typically a bitfield, so need to call with value instead of ptr */
/* 0 has meaning, so no 'Optional' variations of this function */
void IXmlOutputTimeoutMultValue(IXmlOutputState_t *state, const char* tag, uint8 value)
{
	char buf[8];
	FormatTimeoutMult(buf, value);
	IXmlOutputStrUint(state, tag, buf, value);
}

void IXmlOutputTimeoutMult(IXmlOutputState_t *state, const char* tag, void *data)
{
	IXmlOutputTimeoutMultValue(state, tag, *(uint8*)data);
}

/* typically a bitfield, so need to call with value instead of ptr */
/* 0 has meaning, so no 'Optional' variations of this function */
void IXmlOutputHOQLifeValue(IXmlOutputState_t *state, const char* tag, uint8 value)
{
	char buf[9];
	if (value > IB_LIFETIME_MAX) {
		memcpy(buf, "Infinite", 9);
	} else {
		FormatTimeoutMult(buf, value);
	}
	IXmlOutputStrUint(state, tag, buf, value);
}

void IXmlOutputHOQLife(IXmlOutputState_t *state, const char* tag, void *data)
{
	IXmlOutputHOQLifeValue(state, tag, *(uint8*)data);
}

void IXmlOutputIPAddrIPV6(IXmlOutputState_t *state, const char *tag, void *data)
{
	int i;
	IXmlOutputStartTag(state, tag);
	for (i = 0; i < 16; ++i){
		IXmlOutputPrint(state, "%02x", ((STL_IPV6_IP_ADDR *)data)->addr[i]);
	}
	IXmlOutputEndTag(state, tag);
}

void IXmlOutputIPAddrIPV4(IXmlOutputState_t *state, const char *tag, void *data)
{
	int i;
	IXmlOutputStartTag(state, tag);
	for (i = 0; i < 4; ++i){
		IXmlOutputPrint(state, "%02x", ((STL_IPV4_IP_ADDR *)data)->addr[i]);
	}
	IXmlOutputEndTag(state, tag);
}

void IXmlParserEndIPAddrIPV6(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	int i, ret;
	for (i = 0; i < 16; ++i){//read in hexadecimal address 2 digits at a time
		ret = sscanf((char *)(content + 2*i), "%2"SCNx8, &(((STL_IPV6_IP_ADDR *)IXmlParserGetField(field, object))->addr[i]));
		if (ret != 1){
			IXmlParserPrintError(state, "Error parsing IPV6 address:%s", strerror(errno));
			break;
		}
	}
}

void IXmlParserEndIPAddrIPV4(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	int i, ret;
	for (i = 0; i < 4; ++i){ //read in hexadecimal address 2 digits at a time
		ret = sscanf((char *)(content + 2*i), "%2"SCNx8, &(((STL_IPV4_IP_ADDR *)IXmlParserGetField(field, object))->addr[i]));
		if (ret != 1){
			IXmlParserPrintError(state, "Error parsing IPV4 address:%s", strerror(errno));
			break;
		}
	}
}

// parse TimeoutMult string into a uint8 field and validate value
// does not handle "infinite".  Returns value between 0 and 31 inclusive
boolean IXmlParseTimeoutMult_Str(IXmlParserState_t *state, XML_Char *content, unsigned len, uint8 *value)
{
	char *endptr = NULL;
	uint64 temp;
	FSTATUS status;

	if (! len) {
		IXmlParserPrintError(state, "Empty contents");
		return FALSE;
	}
	status = StringToUint64(&temp, content, &endptr, 0, TRUE /*skip_trail_whitespace */);
	if (status == FINVALID_SETTING) {
		IXmlParserPrintError(state, "Numeric Value too large for field: %s", content);
		return FALSE;
	} else if (status != FSUCCESS) {
		IXmlParserPrintError(state, "Invalid contents: %s", content);
		return FALSE;
	} else if (*endptr == '\0') {
		IXmlParserPrintError(state, "Invalid contents, no units: %s", content);
		return FALSE;
	}

	// now parse units
	content = endptr;
	// we round up to next valid multiplier
	// values over 2.4 hours get the max multiplier
	if (strncasecmp(content,"ns",2) == 0) {
		content += 2;
		temp = TimeoutTimeToMult(temp/1000);
	} else if (strncasecmp(content,"us",2) == 0) {
		content += 2;
		temp = TimeoutTimeToMult(temp);
	} else if (strncasecmp(content,"ms",2) == 0) {
		content += 2;
		if (temp >= IB_MAX_TIMEOUT_MULT_MS)
			temp = 31;
		else
			temp = TimeoutTimeMsToMult((uint32)temp);
	} else if (strncasecmp(content,"s",1) == 0) {
		content++;
		if (temp >= IB_MAX_TIMEOUT_MULT_MS/1000)
			temp = 31;
		else
			temp = TimeoutTimeMsToMult(temp*1000);
	} else if (strncasecmp(content,"m",1) == 0) {
		content++;
		if (temp >= IB_MAX_TIMEOUT_MULT_MS/(60*1000))
			temp = 31;
		else
			temp = TimeoutTimeMsToMult(temp*60*1000);
	} else if (strncasecmp(content,"h",1) == 0) {
		content++;
		if (temp >= IB_MAX_TIMEOUT_MULT_MS/(60*60*1000))
			temp = 31;
		else
			temp = TimeoutTimeMsToMult(temp*60*60*1000);
	} else {
		IXmlParserPrintError(state, "Invalid contents, invalid units: %s", content);
		return FALSE;
	}
	
	// make sure rest is trailing whitespace
	while (isspace(*content))
		content++;
	if (*content != '\0')
		goto fail;
	*value = (uint8)temp;
	return TRUE;
fail:
	IXmlParserPrintError(state, "Invalid contents, invalid text after units: %s", content);
	return FALSE;
}

// parse TimeoutMult string into a uint8 field and validate value
// does not handle "infinite".  Returns value between 0 and 31 inclusive
void IXmlParserEndTimeoutMult_Str(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (! IXmlParseTimeoutMult_Str(state, content, len, &value))
		goto fail;
	ASSERT(field->size == 1);
	*(uint8 *)IXmlParserGetField(field, object) = value;
fail:
	return;
}

// parse TimeoutMult string into a uint32 field and validate value
// does not handle "infinite".  Returns value between 0 and 31 inclusive
void IXmlParserEndTimeoutMult32_Str(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (! IXmlParseTimeoutMult_Str(state, content, len, &value))
		goto fail;
	ASSERT(field->size == 4);
	*(uint32 *)IXmlParserGetField(field, object) = value;
fail:
	return;
}

// parse TimeoutMult string into a uint8 field and validate value
// treat "infinite" or multipliers > IB_LIFETIME_MAX as IB_LIFETIME_MAX+1
boolean IXmlParseTimeoutMultInf_Str(IXmlParserState_t *state, XML_Char *content, unsigned len, uint8 *value)
{
	XML_Char *p = content;

	if (! len) {
		IXmlParserPrintError(state, "Empty contents");
		return FALSE;
	}
	// ignore leading whitespace
	while (isspace(*content)) {
		content++;
	}
	if (strncasecmp(content,"infinite",8) == 0) {
		content += 8;
		// make sure rest is trailing whitespace
		while (isspace(*content))
			content++;
		if (*content != '\0')
			goto fail;
		*value = IB_LIFETIME_MAX+1;
		return TRUE;
	} else {
		uint8 temp;
		if (! IXmlParseTimeoutMult_Str(state, content, len, &temp))
			return FALSE;
		if (temp > IB_LIFETIME_MAX)
			temp = IB_LIFETIME_MAX+1;
		*value = temp;
		return TRUE;
	}
fail:
	IXmlParserPrintError(state, "Invalid contents: %s", p);
	return FALSE;
}

void IXmlParserEndHoqTimeout_Str(IXmlParserState_t *state,
	const IXML_FIELD *field, void *object, void *parent, XML_Char *content,
	unsigned int len, boolean valid)
{
	uint8_t value;
	if (!IXmlParseTimeoutMultInf_Str(state, content, len, &value)) {
		return;
	}

	if (value > IB_LIFETIME_MAX) {
		char minBuf[8];
		char maxBuf[8];
		FormatTimeoutMult(minBuf, 0);
		FormatTimeoutMult(maxBuf, IB_LIFETIME_MAX);
		IXmlParserPrintWarning(state, "%s is outside allowed range %s-%s. Setting to %s.",
			content, minBuf, maxBuf, maxBuf);
		value = IB_LIFETIME_MAX;
	}

	*(uint32_t *)IXmlParserGetField(field, object) = value;
}

void IXmlParserEndHoqTimeout_Int(IXmlParserState_t *state,
	const IXML_FIELD *field, void *object, void *parent, XML_Char *content,
	unsigned int len, boolean valid)
{
	uint32_t value;
	if (!IXmlParseUint32(state, content, len, &value)) {
		return;
	}

	if (value > IB_LIFETIME_MAX) {
		IXmlParserPrintWarning(state, "%u is outside allowed range 0-%u. Setting to %u.",
			value, IB_LIFETIME_MAX, IB_LIFETIME_MAX);
		value = IB_LIFETIME_MAX;
	}

	*(uint32_t *)IXmlParserGetField(field, object) = value;
}


/* typically a bitfield, so need to call with value instead of ptr */
/* 0 has meaning, so no 'Optional' variations of this function */
void IXmlOutputSMStateValue(IXmlOutputState_t *state, const char* tag, uint8 value)
{
	IXmlOutputStrUint(state, tag, IbSMStateToText(value), value);
}

void IXmlOutputSMState(IXmlOutputState_t *state, const char* tag, void *data)
{
	IXmlOutputSMStateValue(state, tag, *(uint8*) data);
}

/* typically a bitfield, so need to call with value instead of ptr */
/* 0 has meaning, so no 'Optional' variations of this function */
void IXmlOutputOnOffValue(IXmlOutputState_t *state, const char* tag, uint8 value)
{
	IXmlOutputStrUint(state, tag, value?"On":"Off", value);
}

void IXmlOutputOnOff(IXmlOutputState_t *state, const char* tag, void *data)
{
	IXmlOutputOnOffValue(state, tag, *(uint8*)data);
}

/****************************************************************************/
void IXmlOutputPathRecord(IXmlOutputState_t *state, const char* tag, void *data)
{
	IB_PATH_RECORD *pPathRecord = (IB_PATH_RECORD*)data;

	IXmlOutputStartTag(state, tag);
	IXmlOutputGID(state, "SGID", &pPathRecord->SGID);
	IXmlOutputGID(state, "DGID", &pPathRecord->DGID);
	IXmlOutputLID(state, "SLID", &pPathRecord->SLID);
	IXmlOutputLID(state, "DLID", &pPathRecord->DLID);
	IXmlOutputStrUint(state, "Reversible", pPathRecord->Reversible?"Y":"N", pPathRecord->Reversible);
	IXmlOutputPKey(state, "PKey", &pPathRecord->P_Key);
	IXmlOutputStrUint(state, "Raw", pPathRecord->u1.s.RawTraffic?"Y":"N", pPathRecord->u1.s.RawTraffic);
	IXmlOutputHex(state, "FlowLabel", pPathRecord->u1.s.FlowLabel);
	IXmlOutputHex(state, "HopLimit", pPathRecord->u1.s.HopLimit);
	IXmlOutputHex(state, "TClass", pPathRecord->TClass);
	IXmlOutputUint(state, "SL", pPathRecord->u2.s.SL);
	IXmlOutputUint(state, "Mtu", GetBytesFromMtu(pPathRecord->Mtu));
	IXmlOutputRateValue(state, "Rate", pPathRecord->Rate);
	IXmlOutputTimeoutMultValue(state, "PktLifeTime", pPathRecord->PktLifeTime);
	IXmlOutputUint(state, "Preference", pPathRecord->Preference);
	IXmlOutputEndTag(state, tag);
}

// only output if value != NULL
void IXmlOutputOptionalPathRecord(IXmlOutputState_t *state, const char* tag, void *data)
{
	if (data)
		IXmlOutputPathRecord(state, tag, data);
}

/****************************************************************************/
IXML_FIELD XmlTraceRecordFields[] = {
	{ tag:"IDGeneration", format:'H', IXML_FIELD_INFO(STL_TRACE_RECORD, IDGeneration) },
	{ tag:"NodeType", format:'k', IXML_FIELD_INFO(STL_TRACE_RECORD, NodeType), format_func:IXmlOutputNodeType, end_func:IXmlParserEndNoop },	// outputs both
	{ tag:"NodeType_Int", format:'K', IXML_FIELD_INFO(STL_TRACE_RECORD, NodeType), format_func:IXmlOutputNoop, end_func:IXmlParserEndNodeType_Int },	// inputs Int, ignore string on input
	{ tag:"NodeID", format:'H', IXML_FIELD_INFO(STL_TRACE_RECORD, NodeID) },
	{ tag:"ChassisID", format:'H', IXML_FIELD_INFO(STL_TRACE_RECORD, ChassisID) },
	{ tag:"EntryPortID", format:'H', IXML_FIELD_INFO(STL_TRACE_RECORD, EntryPortID) },
	{ tag:"ExitPortID", format:'H', IXML_FIELD_INFO(STL_TRACE_RECORD, ExitPortID) },
	{ tag:"EntryPort", format:'D', IXML_FIELD_INFO(STL_TRACE_RECORD, EntryPort) },
	{ tag:"ExitPort", format:'D', IXML_FIELD_INFO(STL_TRACE_RECORD, ExitPort) },
	{ NULL }
};

void IXmlOutputTraceRecord(IXmlOutputState_t *state, const char* tag, void *data)
{
	IXmlOutputStruct(state, tag, data, NULL, XmlTraceRecordFields);
}

// only output if value != NULL
void IXmlOutputOptionalTraceRecord(IXmlOutputState_t *state, const char* tag, void *data)
{
	IXmlOutputOptionalStruct(state, tag, data, NULL, XmlTraceRecordFields);
}

#if !defined(VXWORKS) || defined(BUILD_DMC)
/****************************************************************************/
/* IocService Input/Output functions */

static void IocServiceXmlFormatAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " id=\"");
	IXmlOutputPrintStrLen(state, (char*)((IOC_SERVICE*)data)->Name, IOC_SERVICE_NAME_SIZE);
	IXmlOutputPrint(state, "\"");
}

IXML_FIELD IocServiceFields[] = {
	{ tag:"Name", format:'C', IXML_FIELD_INFO(IOC_SERVICE, Name) },
	{ tag:"Id", format:'H', IXML_FIELD_INFO(IOC_SERVICE, Id) },
	{ NULL }
};

void IocServiceXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (IOC_SERVICE*)data, IocServiceXmlFormatAttr, IocServiceFields);
}

// only output if value != NULL
void IocServiceXmlOutputOptional(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalStruct(state, tag, (IOC_SERVICE*)data, IocServiceXmlFormatAttr, IocServiceFields);
}
#endif

/* caller must supply IocServiceXmlParserStart and IocServiceXmlParserEnd */

/****************************************************************************/
/* SwitchInfo Input/Output functions */

/* bitfields needs special handling: LID */
static void SwitchInfoXmlOutputLID(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLIDValue(state, tag, ((STL_SWITCHINFO_RECORD *)data)->RID.LID);
}

static void SwitchInfoXmlParserEndLID(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 value;
	
	if (IXmlParseUint32(state, content, len, &value))
		((STL_SWITCHINFO_RECORD *)object)->RID.LID = value;
}

static void IXmlParserEndMulticastFDBTop(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
        uint32 value;

	if (IXmlParseUint32(state, content, len, &value)) {
		if (IS_MCAST16(value))
			value = MCAST16_TO_MCAST32(value);
		if (value && ((value < STL_LID_MULTICAST_BEGIN) || (value > STL_LID_MULTICAST_END)))
			IXmlParserPrintError(state, "MulticastFDBTop value 0x%08x out of range.  Must be 0 or in the range 0x%08x through 0x%08x\n", value, STL_LID_MULTICAST_BEGIN, STL_LID_MULTICAST_END);
		else
			((STL_SWITCHINFO_RECORD *)object)->SwitchInfoData.MulticastFDBTop = value;
	}
}

/* bitfields needs special handling: LifeTimeValue */
static void SwitchInfoXmlOutputLifeTimeValue(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputTimeoutMultValue(state, tag,
		((STL_SWITCHINFO_RECORD*)data)->SwitchInfoData.u1.s.LifeTimeValue);
}

static void SwitchInfoXmlParserEndLifeTimeValue(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((STL_SWITCHINFO_RECORD*)object)->SwitchInfoData.u1.s.LifeTimeValue = value;
}

/* bitfields needs special handling: PortStateChange */
static void SwitchInfoXmlOutputPortStateChange(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag, ((STL_SWITCHINFO_RECORD *)data)->SwitchInfoData.u1.s.PortStateChange);
}

static void SwitchInfoXmlParserEndPortStateChange(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((STL_SWITCHINFO_RECORD *)object)->SwitchInfoData.u1.s.PortStateChange = value;
}

/* bitfields needs special handling: MulticastMask */
static void SwitchInfoXmlOutputMulticastMask(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputHex(state, tag, ((STL_SWITCHINFO_RECORD *)data)->SwitchInfoData.MultiCollectMask.MulticastMask);
}

static void SwitchInfoXmlParserEndMulticastMask(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value))
		((STL_SWITCHINFO_RECORD *)object)->SwitchInfoData.MultiCollectMask.MulticastMask = value;
}

/* bitfields needs special handling: CollectiveMask */
static void SwitchInfoXmlOutputCollectiveMask(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputHex(state, tag, ((STL_SWITCHINFO_RECORD *)data)->SwitchInfoData.MultiCollectMask.CollectiveMask);
}

static void SwitchInfoXmlParserEndCollectiveMask(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value))
		((STL_SWITCHINFO_RECORD *)object)->SwitchInfoData.MultiCollectMask.CollectiveMask = value;
}

IXML_FIELD SwitchInfoFields[] = {
	{ tag:"LID", format:'K', format_func:SwitchInfoXmlOutputLID, end_func:SwitchInfoXmlParserEndLID }, // bitfield
	{ tag:"LinearFDBCap", format:'U', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.LinearFDBCap) },
	{ tag:"MulticastFDBCap", format:'U', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.MulticastFDBCap) },
	{ tag:"LinearFDBTop", format:'U', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.LinearFDBTop) },
	{ tag:"MulticastFDBTop", format:'U', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.MulticastFDBTop), end_func:IXmlParserEndMulticastFDBTop},
	{ tag:"IPAddrIPV6", format:'k', format_func:IXmlOutputIPAddrIPV6, IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.IPAddrIPV6.addr), end_func:IXmlParserEndIPAddrIPV6},
	{ tag:"IPAddrIPV4", format:'k', format_func:IXmlOutputIPAddrIPV4, IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.IPAddrIPV4.addr), end_func:IXmlParserEndIPAddrIPV4},
	{ tag:"LifeTimeValue", format:'k', format_func:SwitchInfoXmlOutputLifeTimeValue, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"LifeTimeValue_Int", format:'K', format_func:IXmlOutputNoop, end_func:SwitchInfoXmlParserEndLifeTimeValue }, // input only bitfield
	{ tag:"PortStateChange", format:'K', format_func:SwitchInfoXmlOutputPortStateChange, end_func:SwitchInfoXmlParserEndPortStateChange }, // bitfield
	{ tag:"PartitionEnforcementCap", format:'U', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.PartitionEnforcementCap) },
	{ tag:"U2", format:'X', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.u2.AsReg8) },
	{ tag:"CapabilityMask", format:'X', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.CapabilityMask) },
	{ tag:"MulticastMask", format:'k', format_func:SwitchInfoXmlOutputMulticastMask, end_func:SwitchInfoXmlParserEndMulticastMask }, // bitfield
	{ tag:"CollectiveMask", format:'k', format_func:SwitchInfoXmlOutputCollectiveMask, end_func:SwitchInfoXmlParserEndCollectiveMask }, // bitfield
	{ tag:"RoutingModeSupported", format:'X', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.RoutingMode.Supported) },
	{ tag:"RoutingModeEnabled", format:'X', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.RoutingMode.Enabled) },
	{ tag:"PortGroupFDBCap", format:'u', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.PortGroupFDBCap) }, // optional to retain compatibility with snapshots made by older versions of OPA.
	{ tag:"PortGroupCap", format:'U', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.PortGroupCap) },
	{ tag:"PortGroupTop", format:'U', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.PortGroupTop) },
	{ tag:"AdaptiveRouting", format:'x', IXML_FIELD_INFO(STL_SWITCHINFO_RECORD, SwitchInfoData.AdaptiveRouting.AsReg16) },
	{ NULL }
};

void SwitchInfoXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (STL_SWITCHINFO_RECORD*)data, NULL, SwitchInfoFields);
}

// only output if value != NULL
void SwitchInfoXmlOutputOptional(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalStruct(state, tag, (STL_SWITCHINFO_RECORD*)data, NULL, SwitchInfoFields);
}

/* caller must supply SwitchInfoXmlParserStart and SwitchInfoXmlParserEnd */
