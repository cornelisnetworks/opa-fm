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

#ifndef _IBA_PUBLIC_IXML_IB_H_
#define _IBA_PUBLIC_IXML_IB_H_

#include <iba/stl_types.h>
#include <iba/ipublic.h>
#include "ixml.h"

#ifdef __cplusplus
extern "C" {
#endif

/* sometimes a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputLIDValue(IXmlOutputState_t *state, const char *tag, STL_LID value);
extern void IXmlOutputOptionalLIDValue(IXmlOutputState_t *state, const char *tag, STL_LID value);
extern void IXmlOutputLID(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputOptionalLID(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputPKey(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputOptionalPKey(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputGID(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputOptionalGID(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputNodeType(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputOptionalNodeType(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlParserEndNodeType(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid);
extern void IXmlParserEndNodeType_Int(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid);
extern void IXmlOutputNodeDesc(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputOptionalNodeDesc(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputIocIDString(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputOptionalIocIDString(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputServiceName(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputOptionalServiceName(IXmlOutputState_t *state, const char *tag, void *data);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputPortStateValue(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputOptionalPortStateValue(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputPortState(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputOptionalPortState(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputInitReasonValue(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputOptionalInitReasonValue(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputInitReason(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputOptionalInitReason(IXmlOutputState_t *state, const char *tag, void *data);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputPortPhysStateValue(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputPortOptionalPhysStateValue(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputPortPhysState(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputPortOptionalPhysState(IXmlOutputState_t *state, const char *tag, void *data);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputPortDownDefaultValue(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputOptionalPortDownDefaultValue(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputPortDownDefault(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputOptionalPortDownDefault(IXmlOutputState_t *state, const char *tag, void *data);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputMKeyProtectValue(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputMKeyProtect(IXmlOutputState_t *state, const char *tag, void *data);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputRateValue(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputOptionalRateValue(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputRate(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputOptionalRate(IXmlOutputState_t *state, const char *tag, void *data);
extern boolean IXmlParseRateMult_Str(IXmlParserState_t *state, XML_Char *content, uint8 *value);
extern void IXmlParserEndRate(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid);
extern void IXmlParserEndRate_Int(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputLinkWidthValue(IXmlOutputState_t *state, const char* tag, uint16 value);
extern void IXmlOutputLinkWidth(IXmlOutputState_t *state, const char* tag, void *data);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputLinkSpeedValue(IXmlOutputState_t *state, const char* tag, uint16 value);
extern void IXmlOutputLinkSpeed(IXmlOutputState_t *state, const char* tag, void *data);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputMtuValue(IXmlOutputState_t *state, const char* tag, uint16 value);
extern void IXmlOutputOptionalMtuValue(IXmlOutputState_t *state, const char* tag, uint16 value);
extern void IXmlOutputMtu(IXmlOutputState_t *state, const char* tag, void *data);
extern void IXmlOutputOptionalMtu(IXmlOutputState_t *state, const char* tag, void *data);
extern void IXmlParserEndMtu(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputVLsValue(IXmlOutputState_t *state, const char* tag, uint8 value);
extern void IXmlOutputOptionalVLsValue(IXmlOutputState_t *state, const char* tag, uint8 value);
extern void IXmlOutputVLs(IXmlOutputState_t *state, const char* tag, void *data);
extern void IXmlOutputOptionalVLs(IXmlOutputState_t *state, const char* tag, void *data);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputInitTypeValue(IXmlOutputState_t *state, const char* tag, uint8 value);
extern void IXmlOutputInitType(IXmlOutputState_t *state, const char* tag, void *data);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputTimeoutMultValue(IXmlOutputState_t *state, const char* tag, uint8 value);
extern void IXmlOutputTimeoutMult(IXmlOutputState_t *state, const char* tag, void *data);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputHOQLifeValue(IXmlOutputState_t *state, const char* tag, uint8 value);
extern void IXmlOutputHOQLife(IXmlOutputState_t *state, const char* tag, void *data);
extern void IXmlOutputIPAddrIPV6(IXmlOutputState_t *state, const char* tag, void *data);
extern void IXmlOutputIPAddrIPV4(IXmlOutputState_t *state, const char* tag, void *data);
extern void IXmlParserEndIPAddrIPV6(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid);
extern void IXmlParserEndIPAddrIPV4(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid);
extern boolean IXmlParseTimeoutMult_Str(IXmlParserState_t *state, XML_Char *content, unsigned len, uint8 *value);
extern void IXmlParserEndTimeoutMult_Str(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid);
extern void IXmlParserEndTimeoutMult32_Str(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid);
extern boolean IXmlParseTimeoutMultInf_Str(IXmlParserState_t *state, XML_Char *content, unsigned len, uint8 *value);

/* Specialized timeout parsing for SwitchLifetime and HoQLife elements. Parse infinite but map any values > IB_LIFETIME_MAX to IB_LIFETIME_MAX. */
extern void IXmlParserEndHoqTimeout_Int(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned int len, boolean valid);
extern void IXmlParserEndHoqTimeout_Str(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned int len, boolean valid);

/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputSMStateValue(IXmlOutputState_t *state, const char* tag, uint8 value);
extern void IXmlOutputSMState(IXmlOutputState_t *state, const char* tag, void *data);
/* typically a bitfield, so need to call with value instead of ptr */
extern void IXmlOutputOnOffValue(IXmlOutputState_t *state, const char* tag, uint8 value);
extern void IXmlOutputOnOff(IXmlOutputState_t *state, const char* tag, void *data);
extern void IXmlOutputPathRecord(IXmlOutputState_t *state, const char* tag, void *data);
extern void IXmlOutputOptionalPathRecord(IXmlOutputState_t *state, const char* tag, void *data);
extern void IXmlOutputTraceRecord(IXmlOutputState_t *state, const char* tag, void *data);
extern void IXmlOutputOptionalTraceRecord(IXmlOutputState_t *state, const char* tag, void *data);

extern IXML_FIELD IocServiceFields[];
extern void IocServiceXmlOutput(IXmlOutputState_t *state, const char *tag, void *data/*IOC_SERVICE*/);
extern void IocServiceXmlOutputOptional(IXmlOutputState_t *state, const char *tag, void *data/*IOC_SERVICE*/);
/* caller must supply IocServiceXmlParserStart and IocServiceXmlParserEnd */

extern IXML_FIELD SwitchInfoFields[];
extern void SwitchInfoXmlOutput(IXmlOutputState_t *state, const char *tag, void *data /*STL_SWITCHINFO_RECORD*/);
extern void SwitchInfoXmlOutputOptional(IXmlOutputState_t *state, const char *tag, void *data /*STL_SWITCHINFO_RECORD*/);
/* caller must supply SwitchInfoXmlParserStart and SwitchInfoXmlParserEnd */

#ifdef __cplusplus
};
#endif

#endif /* _IBA_PUBLIC_IXML_IB_H_ */
