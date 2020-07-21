/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include "topology.h"
#include "topology_internal.h"
#include <stl_helper.h>

/* this file supports fabric snapshot generation and parsing */

/****************************************************************************/
/* PortStatusData Input/Output functions */

/* bitfields needs special handling: LinkQualityIndicator */
static void PortStatusDataXmlOutputLinkQualityIndicator(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag, ((STL_PORT_COUNTERS_DATA *)data)->lq.s.linkQualityIndicator);
}

static void PortStatusDataXmlParserEndLinkQualityIndicator(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((STL_PORT_COUNTERS_DATA *)object)->lq.s.linkQualityIndicator = value;
}

IXML_FIELD PortStatusDataFields[] = {
	{ tag:"XmitData", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portXmitData) },
	{ tag:"RcvData", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portRcvData) },
	{ tag:"XmitPkts", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portXmitPkts) },
	{ tag:"RcvPkts", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portRcvPkts) },
	{ tag:"MulticastXmitPkts", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portMulticastXmitPkts) },
	{ tag:"MulticastRcvPkts", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portMulticastRcvPkts) },
	{ tag:"XmitWait", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portXmitWait) },
	{ tag:"CongDiscards", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, swPortCongestion) },
	{ tag:"RcvFECN", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portRcvFECN) },
	{ tag:"RcvBECN", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portRcvBECN) },
	{ tag:"XmitTimeCong", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portXmitTimeCong) },
	{ tag:"XmitWastedBW", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portXmitWastedBW) },
	{ tag:"XmitWaitData", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portXmitWaitData) },
	{ tag:"RcvBubble", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portRcvBubble) },
	{ tag:"MarkFECN", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portMarkFECN) },
	{ tag:"RcvConstraintErrors", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portRcvConstraintErrors) },
	{ tag:"RcvSwitchRelayErrors", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portRcvSwitchRelayErrors) },
	{ tag:"XmitDiscards", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portXmitDiscards) },
	{ tag:"XmitConstraintErrors", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portXmitConstraintErrors) },
	{ tag:"RcvRemotePhysicalErrors", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portRcvRemotePhysicalErrors) },
	{ tag:"LocalLinkIntegrityErrors", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, localLinkIntegrityErrors) },
	{ tag:"RcvErrors", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, portRcvErrors) },
	{ tag:"ExcessiveBufferOverruns", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, excessiveBufferOverruns) },
	{ tag:"FMConfigErrors", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, fmConfigErrors) },
	{ tag:"LinkErrorRecovery", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, linkErrorRecovery) },
	{ tag:"LinkDowned", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, linkDowned) },
	{ tag:"UncorrectableErrors", format:'U', IXML_FIELD_INFO(STL_PORT_COUNTERS_DATA, uncorrectableErrors) },
	{ tag:"LinkQualityIndicator", format:'K', format_func:PortStatusDataXmlOutputLinkQualityIndicator, end_func:PortStatusDataXmlParserEndLinkQualityIndicator }, // bitfield
	{ NULL }
};

void PortStatusDataXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (STL_PORT_COUNTERS_DATA *)data, NULL, PortStatusDataFields);
}

// only output if value != NULL
void PortStatusDataXmlOutputOptional(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalStruct(state, tag, (STL_PORT_COUNTERS_DATA *)data, NULL, PortStatusDataFields);
}

static void PortStatusDataXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	STL_PORT_COUNTERS_DATA *pPortCountersData = (STL_PORT_COUNTERS_DATA *)object;
	PortData *portp = (PortData*)parent;

	if (! valid)	// missing mandatory fields
		goto failvalidate;

	if (portp->pPortCounters) {
		IXmlParserPrintError(state, "More than 1 PortStatus for Port");
		goto failinsert;
	}
	portp->pPortCounters = pPortCountersData;
	// NumLanesDown is set in PortDataXmlParserEnd()

	return;

failinsert:
failvalidate:
	MemoryDeallocate(pPortCountersData);
}

/****************************************************************************/
/* QOS (SL2SCMap, SC2SLMap, SC2SCMap, VLArb Table) and PKey Table
 * Input/Output functions
 */

uint8 ixVLArb;		// Index for VLArb entry being parsed
uint8 vlVLArb;		// VL for VLArb entry being parsed
uint8 ixPKey;		// Index for P_Key entry being parsed

/****************************************************************************/
/* PortData SLtoSC Input/Output functions */

static void *SLtoSCMapXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData
	QOSData *pQOS = portp->pQOS;

	if (portp->nodep->NodeInfo.NodeType == STL_NODE_SW
		&& portp->PortNum != 0) {
		IXmlParserPrintError(state, "SLtoSCMap not valid for switch external ports");
		return (NULL);
	}

	if (!pQOS) {
		if ( !( pQOS = portp->pQOS = (QOSData *)MemoryAllocate2AndClear(
				sizeof(QOSData), IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
			IXmlParserPrintError(state, "Unable to allocate memory");
			return (NULL);
		}
	}

	if (pQOS->SL2SCMap) {
		IXmlParserPrintError(state, "SLtoSCMap improperly allocated");
		return (NULL);
	}

	if ( !( pQOS->SL2SCMap = (STL_SLSCMAP *)MemoryAllocate2(
			sizeof(STL_SLSCMAP), IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return (NULL);
	}
	memset(pQOS->SL2SCMap, 0xff, sizeof(STL_SLSCMAP));

	return (pQOS->SL2SCMap);

}	// End of SLtoSCMapXmlParserStart

static void SLtoSCMapXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to PortData
	if (! valid)
		goto failvalidate;
	return;

failvalidate:
	PortDataFreeQOSData(IXmlParserGetContext(state), (PortData *)parent);
	return;

}	// End of SLtoSCMapXmlParserEnd

static void *SLtoSCMapXmlParserStartSC(IXmlParserState_t *state, void *parent, const char **attr)
{
	STL_SLSCMAP *pSLSC = (STL_SLSCMAP *)parent;	// parent points to STL_SLSCMAP
	uint8 sl;

	if ( !attr || !attr[0] || (0 != strcmp(attr[0], "SL"))) {
		IXmlParserPrintError(state, "Missing SL attribute for SLtoSCMap.SC");
		return (NULL);
	}

	if (FSUCCESS != StringToUint8(&sl, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid SL attribute in SLtoSCMap.SC  SL: %s", attr[1]);
		return (NULL);
	}
	if (sl >= STL_MAX_SLS) {
		IXmlParserPrintError(state, "SL attribute Out-of-Range in SLtoSCMap.SC  SL: %s", attr[1]);
		return (NULL);
	}

	return &(pSLSC->SLSCMap[sl]);

}	// End of SLtoSCMapXmlParserStartSC

static void SLtoSCMapXmlParserEndSC(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	STL_SLSCMAP *pSLSC = (STL_SLSCMAP *)parent;	// parent points to STL_SLSCMAP
	STL_SC *pSC = (STL_SC*)object;	// object points to specific SLs entry
	uint8 sl = pSC - pSLSC->SLSCMap;
	uint8 sc;

	if (! valid)
		goto failvalidate;

	if (!content || !len) {
		IXmlParserPrintError(state, "No SC Value in SLtoSCMap.SC for SL %u", sl);
		return;
	}

	if (FSUCCESS != StringToUint8(&sc, content, NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid SC Value in SLtoSCMap.SC for SL %u  SC: %s", sl, content);
		return;
	}

	if (sc >= STL_MAX_SCS) {
		IXmlParserPrintError(state, "SC Out-of-range in SLtoSCMap.SC for SL:%u  SC:%u", sl, sc);
		return;
	}

	*((uint8 *)pSC) = sc;

	return;

failvalidate:
	// SLtoSCMapXmlParserEnd will free as needed
	return;

}	// End of SLtoSCMapXmlParserEndSC

IXML_FIELD SLtoSCMapSCFields[] = {
	{ tag:"SC", format:'k', start_func:SLtoSCMapXmlParserStartSC, end_func:SLtoSCMapXmlParserEndSC },
	{ NULL }
};

static void SLtoSCMapXmlOutputSLAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " SL=\"%u\"", *(uint8 *)data);
}

void SLtoSCMapXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortData *portp = (PortData *)data;	// data points to PortData
	NodeData *nodep = portp->nodep;
	STL_SLSCMAP *pSLSC = portp->pQOS->SL2SCMap;
	uint8 sl;

	// SL2SC only applicable to HFIs and switch port 0
	ASSERT(nodep->NodeInfo.NodeType != STL_NODE_SW || portp->PortNum == 0);

	IXmlOutputStartTag(state, tag);

	for(sl = 0; sl < STL_MAX_SLS; sl++)
	{
		IXmlOutputStartAttrTag(state, "SC", &sl, SLtoSCMapXmlOutputSLAttr);
		IXmlOutputPrint(state, "%u", pSLSC->SLSCMap[sl].SC);
		IXmlOutputEndTag(state, "SC");
	}

	IXmlOutputEndTag(state, tag);

}	// End of SLtoSCMapXmlOutput

/****************************************************************************/
/* PortData SCtoSL Input/Output functions */

static void *SCtoSLMapXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData
	QOSData *pQOS = portp->pQOS;

	if (portp->nodep->NodeInfo.NodeType == STL_NODE_SW
		&& portp->PortNum != 0) {
		IXmlParserPrintError(state, "SCtoSLMap not valid for switch external ports");
		return (NULL);
	}

	if (!pQOS) {
		if ( !( pQOS = portp->pQOS = (QOSData *)MemoryAllocate2AndClear(
				sizeof(QOSData), IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
			IXmlParserPrintError(state, "Unable to allocate memory");
			return (NULL);
		}
	}

	if (pQOS->SC2SLMap) {
		IXmlParserPrintError(state, "SCtoSLMap improperly allocated");
		return (NULL);
	}

	if ( !( pQOS->SC2SLMap = (STL_SCSLMAP *)MemoryAllocate2AndClear(
			sizeof(STL_SCSLMAP), IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return (NULL);
	}

	return (pQOS->SC2SLMap);

}	// End of SCtoSLMapXmlParserStart

static void SCtoSLMapXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to PortData
	if (! valid)
		goto failvalidate;
	return;

failvalidate:
	PortDataFreeQOSData(IXmlParserGetContext(state), (PortData *)parent);
	return;

}	// End of SCtoSLMapXmlParserEnd

static void *SCtoSLMapXmlParserStartSL(IXmlParserState_t *state, void *parent, const char **attr)
{
	STL_SCSLMAP *pSCSL = (STL_SCSLMAP *)parent;	// parent points to STL_SCSLMAP
	uint8 sc;

	if ( !attr || !attr[0] || (0 != strcmp(attr[0], "SC"))) {
		IXmlParserPrintError(state, "Missing SC attribute for SCtoSLMap.SL");
		return (NULL);
	}

	if (FSUCCESS != StringToUint8(&sc, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid SC attribute in SCtoSLMap.SL  SC: %s", attr[1]);
		return (NULL);
	}
	if (sc >= STL_MAX_SCS) {
		IXmlParserPrintError(state, "SC attribute Out-of-Range in SCtoSLMap.SL  SC: %s", attr[1]);
		return (NULL);
	}

	return &(pSCSL->SCSLMap[sc]);

}	// End of SCtoSLMapXmlParserStartSL

static void SCtoSLMapXmlParserEndSL(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	STL_SCSLMAP *pSCSL = (STL_SCSLMAP *)parent;	// parent points to STL_SCSLMAP
	STL_SL *pSL = (STL_SL*)object;	// object points to specific SCs entry
	uint8 sc = pSL - pSCSL->SCSLMap;
	uint8 sl;

	if (! valid)
		goto failvalidate;

	if (!content || !len) {
		IXmlParserPrintError(state, "No SL Value in SCtoSLMap.SL for SC %u", sc);
		return;
	}

	if (FSUCCESS != StringToUint8(&sl, content, NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid SL Value in SCtoSLMap.SL for SC %u  SL: %s", sc, content);
		return;
	}

	if (sl >= STL_MAX_SLS) {
		IXmlParserPrintError(state, "SL Out-of-range in SCtoSLMap.SL for SC:%u  SL:%u", sc, sl);
		return;
	}

	pSL->SL = sl;

	return;

failvalidate:
	// SCtoSLMapXmlParserEnd will free as needed
	return;

}	// End of SCtoSLMapXmlParserEndSL

IXML_FIELD SCtoSLMapSLFields[] = {
	{ tag:"SL", format:'k', start_func:SCtoSLMapXmlParserStartSL, end_func:SCtoSLMapXmlParserEndSL },
	{ NULL }
};

static void SCtoSLMapXmlOutputSCAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " SC=\"%u\"", *(uint8 *)data);
}

void SCtoSLMapXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortData *portp = (PortData *)data;	// data points to PortData
	NodeData *nodep = portp->nodep;
	STL_SCSLMAP *pSCSL = portp->pQOS->SC2SLMap;
	uint8 sc;

	// SC2SL only applicable to HFIs and switch port 0
	ASSERT(nodep->NodeInfo.NodeType != STL_NODE_SW || portp->PortNum == 0);

	IXmlOutputStartTag(state, tag);

	for(sc = 0; sc < STL_MAX_SCS; sc++)
	{
		IXmlOutputStartAttrTag(state, "SL", &sc, SCtoSLMapXmlOutputSCAttr);
		IXmlOutputPrint(state, "%u", pSCSL->SCSLMap[sc].SL);
		IXmlOutputEndTag(state, "SL");
	}

	IXmlOutputEndTag(state, tag);

}	// End of SCtoSLMapXmlOutput

/****************************************************************************/
/* PortData SCtoSC Input/Output functions */

static void *SCtoSCMapXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData
	NodeData *nodep = portp->nodep;
	QOSData *pQOS = portp->pQOS;

	if (nodep->NodeInfo.NodeType != STL_NODE_SW) {
		IXmlParserPrintError(state, "SCtoSCMap only valid for switches");
		return (NULL);
	}

	if (portp->PortNum == 0) {
		IXmlParserPrintError(state, "SCtoSCMap only valid for switch external ports");
		return (NULL);
	}

	if (!pQOS) {
		if ( !( pQOS = portp->pQOS = (QOSData *)MemoryAllocate2AndClear(
				sizeof(QOSData), IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
			IXmlParserPrintError(state, "Unable to allocate memory");
			return (NULL);
		}
	}

	QListInitState(&pQOS->SC2SCMapList[0]);
	if (!QListInit(&pQOS->SC2SCMapList[0])) {
		IXmlParserPrintError(state, "Unable to initialize SC2SCMaps list");
		MemoryDeallocate(pQOS);
		return (NULL);
	}

	return (parent);

}	// End of SCtoSCMapXmlParserStart

static void SCtoSCMapXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (! valid)
		goto failvalidate;

	return;

failvalidate:
	PortDataFreeQOSData(IXmlParserGetContext(state), (PortData *)parent);
	return;

}	// End of SCtoSCMapXmlParserEnd

static void *SCtoSCMapXmlParserStartOutPort(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortMaskSC2SCMap *pSC2SC;
	uint8_t outport;

	if ( !attr || !attr[0] || ((0 != strcmp(attr[0], "ports")) && (0 != strcmp(attr[0], "port")))) {
		IXmlParserPrintError(state, "Missing port attribute for SCtoSCMap.OutputPort");
		return (NULL);
	}

	if (!(pSC2SC = (PortMaskSC2SCMap *)MemoryAllocate2AndClear(sizeof(PortMaskSC2SCMap), IBA_MEM_FLAG_PREMPTABLE, MYTAG))) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return (NULL);
	}

	if (!(pSC2SC->SC2SCMap = (STL_SCSCMAP *)MemoryAllocate2AndClear(sizeof(STL_SCSCMAP), IBA_MEM_FLAG_PREMPTABLE, MYTAG))) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		MemoryDeallocate(pSC2SC);
		return (NULL);
	}

	if (0 == strcmp(attr[0], "ports")) { // new format, parse the ports
		if (FSUCCESS != StringToStlPortMask(pSC2SC->outports, attr[1])) {
			IXmlParserPrintError(state, "Invalid ports list attribute in SCtoSCMap.OutputPorts: %s", attr[1]);
			MemoryDeallocate(pSC2SC->SC2SCMap);
			pSC2SC->SC2SCMap = NULL;
			MemoryDeallocate(pSC2SC);
			return NULL;
		}
	} else { // old format
		if (FSUCCESS != StringToUint8(&outport, attr[1], NULL, 0, TRUE)) {
			IXmlParserPrintError(state, "Invalid port list attribute in SCtoSCMap.OutputPort: %s", attr[1]);
			MemoryDeallocate(pSC2SC->SC2SCMap);
			pSC2SC->SC2SCMap = NULL;
			MemoryDeallocate(pSC2SC);
			return (NULL);
		} // parser end function will combine single ports with the same tables
		StlAddPortToPortMask(pSC2SC->outports, outport);
	}

	// Initialize the list entry
	ListItemInitState(&pSC2SC->SC2SCMapListEntry);
	QListSetObj(&pSC2SC->SC2SCMapListEntry, pSC2SC);
	// Initialize all SCs to 15
	memset(&pSC2SC->SC2SCMap->SCSCMap, 15, STL_MAX_SCS);

	return (pSC2SC);

}	// End of SCtoSCMapXmlParserStartOutPort

static void SCtoSCMapXmlParserEndOutPort(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData
	PortMaskSC2SCMap *pSC2SC = (PortMaskSC2SCMap *)object;	// object points to PortMaskSCSCMap
	uint8_t numports, outport;

	numports = StlNumPortsSetInPortMask(pSC2SC->outports, portp->nodep->NodeInfo.NumPorts);
	// Insert into the maps list
	if (numports == 1) {
		// only one port
		outport = StlGetFirstPortInPortMask(pSC2SC->outports);
		QOSDataAddSCSCMap(portp, outport, 0, pSC2SC->SC2SCMap);

		// clear out the unneeded stuff created in parser start
		MemoryDeallocate(pSC2SC->SC2SCMap);
		pSC2SC->SC2SCMap = NULL;
		MemoryDeallocate(pSC2SC);
	} else {
		QListInsertTail(&portp->pQOS->SC2SCMapList[0], &pSC2SC->SC2SCMapListEntry);
	}

	if (! valid)
		goto failvalidate;
	return;

failvalidate:
	// SCtoSCMapXmlParserEnd will free as needed
	return;

}	// End of SCtoSCMapXmlParserEndOutPort

static void *SCtoSCMapXmlParserStartOutPortSC(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortMaskSC2SCMap *pSC2SC = (PortMaskSC2SCMap *)parent;	// parent points to PortMaskSCSCMap for given port mask
	uint8 sc;

	if ( !attr || !attr[0] || (0 != strcmp(attr[0], "SC"))) {
		IXmlParserPrintError(state, "Missing SC attribute for SCtoSCMap.OutputPort.SC");
		return (NULL);
	}

	if (FSUCCESS != StringToUint8(&sc, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid SC attribute in SCtoSCMap.OutputPort.SC  SC: %s", attr[1]);
		return (NULL);
	}
	if (sc >= STL_MAX_SCS) {
		IXmlParserPrintError(state, "SC attribute Out-of-Range in SCtoSCMap.outputPort.SC  SC: %s", attr[1]);
		return (NULL);
	}

	return &(pSC2SC->SC2SCMap->SCSCMap[sc]);

}	// End of SCtoSCMapXmlParserStartOutPortSC

static void SCtoSCMapXmlParserEndOutPortSC(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to STL_SCSCMAP for given output port
	PortMaskSC2SCMap *pSC2SC = (PortMaskSC2SCMap *)parent;	// parent points to PortMaskSCSCMap for given port mask
	STL_SC *pSC = (STL_SC*)object;	// object points to specific SCs entry
	uint8 isc = pSC - pSC2SC->SC2SCMap->SCSCMap;	// input SC
	uint8 sc;	// output SC

	if (! valid)
		goto failvalidate;

	if (!content || !len) {
		IXmlParserPrintError(state, "No SC Value in SCtoSCMap.OutputPort.SC for SC %u", isc);
		return;
	}

	if (FSUCCESS != StringToUint8(&sc, content, NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid SC' Value in SCtoSCMap.OutputPort.SC for SC %u  SC': %s", isc, content);
		return;
	}

	if (sc >= STL_MAX_SCS) {
		IXmlParserPrintError(state, "SC Out-of-range in SCtoSCMap.OutputPort.SC for SC %u  SC':%u", isc, sc);
		return;
	}

	pSC->SC = sc;

	return;

failvalidate:
	// SCtoSCMapXmlParserEnd will free as needed
	return;

}	// End of SCtoSCMapXmlParserEndOutPortSC

IXML_FIELD SCtoSCMapOutPortSCFields[] = {
	{ tag:"SC", format:'k', start_func:SCtoSCMapXmlParserStartOutPortSC, end_func:SCtoSCMapXmlParserEndOutPortSC },
	{ NULL }
};

IXML_FIELD SCtoSCMapOutPortFields[] = {
	{ tag:"OutputPort", format:'C', subfields:SCtoSCMapOutPortSCFields, start_func:SCtoSCMapXmlParserStartOutPort, end_func:SCtoSCMapXmlParserEndOutPort},
	{ NULL }
};

static void SCtoSCMapXmlOutputPortAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " ports=\"%s\"", (char *)data);
}

static void SCtoSCMapXmlOutputSCAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " SC=\"%u\"", *(uint8 *)data);
}

void SCtoSCMapXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortData *portp = (PortData *)data;	// data points to PortData
	NodeData *nodep = portp->nodep;
	LIST_ITEM *p;
	uint8 sc;

	// SC2SC only applicable to switch external ports
	ASSERT(nodep->NodeInfo.NodeType == STL_NODE_SW && portp->PortNum != 0);

	if (QListHead(&portp->pQOS->SC2SCMapList[0]) != NULL) {
		IXmlOutputStartTag(state, tag);

		for (p = QListHead(&portp->pQOS->SC2SCMapList[0]); p != NULL; p = QListNext(&portp->pQOS->SC2SCMapList[0], p)) {
			PortMaskSC2SCMap *pSC2SC = (PortMaskSC2SCMap *)QListObj(p);
			int buflen = portp->nodep->NodeInfo.NumPorts*3;
			char buf[buflen];

			FormatStlPortMask(buf, pSC2SC->outports, portp->nodep->NodeInfo.NumPorts, buflen);

			IXmlOutputStartAttrTag(state, "OutputPort", &buf[7],
				SCtoSCMapXmlOutputPortAttr);

			for (sc = 0; sc < STL_MAX_SCS; sc++) {
				// don't output mappings to SC15
				if (sc == 0 || pSC2SC->SC2SCMap->SCSCMap[sc].SC != 15) {
					IXmlOutputStartAttrTag(state, "SC", &sc, SCtoSCMapXmlOutputSCAttr);
					IXmlOutputPrint(state, "%u", pSC2SC->SC2SCMap->SCSCMap[sc].SC);
					IXmlOutputEndTag(state, "SC");
				}
			}

			IXmlOutputEndTag(state, "OutputPort");
		}

		IXmlOutputEndTag(state, tag);
	}

}	// End of SCtoSCMapXmlOutput

/****************************************************************************/
/* PortData SCtoVLx Input/Output functions */

static void *SCtoVLxMapXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr, ScvlEnum_t scvlx)
{
	PortData *portp = (PortData *)parent; // parent points to PortData
	NodeData *nodep = (NodeData *)portp->nodep;
	QOSData *pQOS = portp->pQOS;

	if (scvlx == Enum_SCVLnt && nodep->NodeInfo.NodeType == STL_NODE_SW && portp->PortNum == 0) {
		IXmlParserPrintError(state, "SCtoVLtMap not valid for Switch port 0");
		return (NULL);
	}

	if (scvlx == Enum_SCVLr && !getIsVLrSupported(nodep, portp)) {
		IXmlParserPrintError(state, "SCtoVLrMap not supported");
		return (NULL);
	}

	if (!pQOS) {
		if (!(pQOS = portp->pQOS = (QOSData*)MemoryAllocate2AndClear(
				sizeof(QOSData), IBA_MEM_FLAG_PREMPTABLE, MYTAG ))) {
			IXmlParserPrintError(state, "Unable to allocate memory");
			return (NULL);
		}
	}

	return (&pQOS->SC2VLMaps[scvlx]);

} // End of SCtoVLxMapXmlParserStart

static void *SCtoVLrMapXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return SCtoVLxMapXmlParserStart(state, parent, attr, Enum_SCVLr);
}

static void *SCtoVLtMapXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return SCtoVLxMapXmlParserStart(state, parent, attr, Enum_SCVLt);
}

static void *SCtoVLntMapXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return SCtoVLxMapXmlParserStart(state, parent, attr, Enum_SCVLnt);
}

static void SCtoVLxMapXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to PortData
	if (!valid)
		goto failvalidate;
	return;

failvalidate:
	PortDataFreeQOSData(IXmlParserGetContext(state), (PortData *)parent);
	return;
} // End of SCtoVLxMapXmlParserEnd

static void *SCtoVLxMapXmlParserStartVLx(IXmlParserState_t *state, void *parent, const char **attr)
{
	STL_SCVLMAP *pSCVL = (STL_SCVLMAP *)parent; // parent points to STL_SCVLMAP
	uint8 sc;

	if (!attr || !attr[0] || (0 != strcmp(attr[0], "SC"))) {
		IXmlParserPrintError(state, "Missing SC attribute for SCtoVLMAp.VL");
		return (NULL);
	}

	if (StringToUint8(&sc, attr[1], NULL, 0, TRUE) != FSUCCESS) {
		IXmlParserPrintError(state, "Invalid SC attribute in SCtoVLMap.VL SC: %s", attr[1]);
		return (NULL);
	}
	if (sc >= STL_MAX_SCS) {
		IXmlParserPrintError(state, "SC attribute Out-of-Range in SCtoVLMap.VL SC: %s", attr[1]);
		return (NULL);
	}

	return &(pSCVL->SCVLMap[sc]);
} // End of SCtoVLxMapXmlParserStartVLx

static void SCtoVLxMapXmlParserEndVLx(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	STL_SCVLMAP *pSCVL = (STL_SCVLMAP *)parent; // parent points to STL_SCVLMAP
	STL_VL *pVL = (STL_VL*)object; // object points to specific VLs entry
	uint8 sc = pVL - pSCVL->SCVLMap;
	uint8 vl;

	if (!valid)
		goto failvalidate;

	if (!content || !len) {
		IXmlParserPrintError(state, "No VL Value in SCtoVLMap.VL for SC %u", sc);
		return;
	}

	if (StringToUint8(&vl, content, NULL, 0, TRUE) != FSUCCESS) {
		IXmlParserPrintError(state, "Invalid VL Value in SCtoVLMap.VL for SC: %u VL: %s", sc, content);
		return;
	}

	if (vl >= STL_MAX_VLS) {
		IXmlParserPrintError(state, "VL Out-of-range in SCtoVLMap.VL for SC: %u VL: %u", sc, vl);
		return;
	}

	pVL->VL = vl;
	
	return;

failvalidate:
	// SCtoVLMapXmlParserEnd will free as needed
	return;
}


IXML_FIELD SCtoVLrMapVLrFields[] = {
	{ tag:"VLr", format:'k', start_func:SCtoVLxMapXmlParserStartVLx, end_func:SCtoVLxMapXmlParserEndVLx },
	{ NULL }
};

IXML_FIELD SCtoVLtMapVLtFields[] = {
	{ tag:"VLt", format:'k', start_func:SCtoVLxMapXmlParserStartVLx, end_func:SCtoVLxMapXmlParserEndVLx },
	{ NULL }
};

IXML_FIELD SCtoVLntMapVLntFields[] = {
	{ tag:"VLnt", format:'k', start_func:SCtoVLxMapXmlParserStartVLx, end_func:SCtoVLxMapXmlParserEndVLx },
	{ NULL }
};

static void SCtoVLxMapXmlOutputSCAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " SC=\"%u\"", *(uint8 *)data);
}

void SCtoVLxMapXmlOutput(IXmlOutputState_t *state, const char *tag, void *data, ScvlEnum_t scvlx)
{
	PortData *portp = (PortData *)data; // data points to PortData
	NodeData *nodep = portp->nodep;
	STL_SCVLMAP *pSCVL = &(portp->pQOS->SC2VLMaps[scvlx]);
	uint8 sc;
	char *vlname = "VL";
	switch (scvlx) {
		case Enum_SCVLr: vlname = "VLr";
		break;
		case Enum_SCVLt: vlname = "VLt";
		break;
		case Enum_SCVLnt: vlname = "VLnt";
		break;
	}

	// SC2VLnt doesn't apply to switch port 0
	ASSERT(!(scvlx == Enum_SCVLnt && nodep->NodeInfo.NodeType == STL_NODE_SW && portp->PortNum == 0));
	ASSERT(!(scvlx == Enum_SCVLr && !getIsVLrSupported(nodep, portp)));

	IXmlOutputStartTag(state, tag);

	for(sc = 0; sc < STL_MAX_SCS; sc++)
	{
		IXmlOutputStartAttrTag(state, vlname, &sc, SCtoVLxMapXmlOutputSCAttr);
		IXmlOutputPrint(state, "%u", pSCVL->SCVLMap[sc].VL);
		IXmlOutputEndTag(state, vlname);
	}

	IXmlOutputEndTag(state, tag);

} // End of SCtoVLxMapXmlOutput

/****************************************************************************/
/* PortData VLArbitration Weight Input/Output functions */

static void *VLArbXmlParserStartWeight(IXmlParserState_t *state, void *parent, const char **attr)
{
	// parent points to STL_VLARB_TABLE

	if ( !attr || !attr[0] || (0 != strcmp(attr[0], "VL"))) {
		IXmlParserPrintError(state, "Missing VL attribute for VLArbitration Weight");
		return (NULL);
	}

	if (FSUCCESS != StringToUint8(&vlVLArb, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid VL attribute in VLArbitration Weight   VL: %s", attr[1]);
		return (NULL);
	}

	if (vlVLArb >= STL_MAX_VLS) {
		IXmlParserPrintError( state,
			"VL attribute Out-of-range in VLArbitration Weight  VL: %s", attr[1] );
		return (NULL);
	}

	// ideally we could return &pVLArbTable->Elements[ixVLArb]
	// and even save vVLArb in the entry here.
	// but we need to keep ixVLArb so we can increment it
	// so we simply keep ixVLArb and vlVLArb in globals and don't really use
	// "object" as our return
	return (parent);

}	// End of VLArbXmlParserStartWeight

static void VLArbXmlParserEndWeight(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to STL_VLARB_TABLE
	STL_VLARB_TABLE *pVLArb = (STL_VLARB_TABLE *)parent;
	uint8 weight;

	if (! valid)
		goto failvalidate;

	// Assign VLArbTable->ArbTable[ixVLArb] = vl,weight
	if (!content || !len) {
		IXmlParserPrintError(state, "No Weight Value in VLArbitration Weight for VL %u", vlVLArb);
		return;
	}

	if (FSUCCESS != StringToUint8(&weight, content, NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid Weight Value in VLArbitration Weight for VL %u  Weight: %s", vlVLArb, content);
		return;
	}

	pVLArb->Elements[ixVLArb].s.VL = vlVLArb;
	pVLArb->Elements[ixVLArb].Weight = weight;
	ixVLArb++;
	return;

failvalidate:
	// VLArb*XmlParserEnd will free as needed
	return;

}	// End of VLArbXmlParserEndWeight

IXML_FIELD VLArbFields[] = {
	{ tag:"Weight", format:'k', start_func:VLArbXmlParserStartWeight, end_func:VLArbXmlParserEndWeight},
	{ NULL }
};

static void VLArbXmlOutputVLAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " VL=\"%u\"", *(uint8 *)data);
}

void VLArbXmlOutput(IXmlOutputState_t *state, const char *tag, void *data, int capacity)
{
	int ix;
	uint8 vl;
	STL_VLARB_TABLE *pVLArb = (STL_VLARB_TABLE*)data;

	IXmlOutputStartTag(state, tag);
	for(ix = 0; ix < capacity; ix++)
	{
		vl = pVLArb->Elements[ix].s.VL;
		IXmlOutputStartAttrTag(state, "Weight", &vl, VLArbXmlOutputVLAttr);
		IXmlOutputPrint(state, "%u", pVLArb->Elements[ix].Weight);
		IXmlOutputEndTag(state, "Weight");
	}
	IXmlOutputEndTag(state, tag);

}	// End of VLArbXmlOutput

/****************************************************************************/
/* PortData VLArbitrationLow Input/Output functions */

static void *VLArbLowXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData
	QOSData *pQOS = portp->pQOS;

	if (!pQOS) {
		if ( !( pQOS = portp->pQOS = (QOSData *)MemoryAllocate2AndClear(
				sizeof(QOSData), IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
			IXmlParserPrintError(state, "Unable to allocate memory");
			return (NULL);
		}
	}

	ixVLArb = 0;
	return ((void *)&(pQOS->u.VLArbTable[STL_VLARB_LOW_ELEMENTS]));

}	// End of VLArbLowXmlParserStart

static void VLArbLowXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to PortData
	if (! valid)
		goto failvalidate;
	return;

failvalidate:
	PortDataFreeQOSData(IXmlParserGetContext(state), (PortData *)parent);
	return;

}	// End of VLArbLowXmlParserEnd


void VLArbLowXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortData *portp = (PortData *)data;	// data points to PortData
	STL_VLARB_TABLE *pVLArb = portp->pQOS->u.VLArbTable;

	VLArbXmlOutput(state, tag, &pVLArb[STL_VLARB_LOW_ELEMENTS],
						portp->PortInfo.VL.ArbitrationLowCap);
}	// End of VLArbLowXmlOutput

/****************************************************************************/
/* PortData VLArbitrationHigh Input/Output functions */

static void *VLArbHighXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData
	QOSData *pQOS = portp->pQOS;

	if (!pQOS) {
		if ( !( pQOS = portp->pQOS = (QOSData *)MemoryAllocate2AndClear(
				sizeof(QOSData), IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
			IXmlParserPrintError(state, "Unable to allocate memory");
			return (NULL);
		}
	}

	ixVLArb = 0;
	return ((void *)&(pQOS->u.VLArbTable[STL_VLARB_HIGH_ELEMENTS]));

}	// End of VLArbHighXmlParserStart

static void VLArbHighXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to PortData
	if (! valid)
		goto failvalidate;
	return;

failvalidate:
	PortDataFreeQOSData(IXmlParserGetContext(state), (PortData *)parent);
	return;

}	// End of VLArbHighXmlParserEnd

void VLArbHighXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortData *portp = (PortData *)data;	// data points to PortData
	STL_VLARB_TABLE *pVLArb = portp->pQOS->u.VLArbTable;

	VLArbXmlOutput(state, tag, &pVLArb[STL_VLARB_HIGH_ELEMENTS],
			portp->PortInfo.VL.ArbitrationHighCap);

}	// End of VLArbHighXmlOutput

/****************************************************************************/
/* PortData VLArbitrationPreemptElements Input/Output functions */

static void *VLArbPreemptElementsXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData
	QOSData *pQOS = portp->pQOS;

	if (!pQOS) {
		if ( !( pQOS = portp->pQOS = (QOSData *)MemoryAllocate2AndClear(
				sizeof(QOSData), IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
			IXmlParserPrintError(state, "Unable to allocate memory");
			return (NULL);
		}
	}

	ixVLArb = 0;
	return ((void *)&(pQOS->u.VLArbTable[STL_VLARB_PREEMPT_ELEMENTS]));

}	// End of VLArbPreemptElementsXmlParserStart

static void VLArbPreemptElementsXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to PortData
	if (! valid)
		goto failvalidate;
	return;

failvalidate:
	PortDataFreeQOSData(IXmlParserGetContext(state), (PortData *)parent);
	return;

}	// End of VLArbPreemptElementsXmlParserEnd

void VLArbPreemptElementsXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortData *portp = (PortData *)data; // data points to PortData
	STL_VLARB_TABLE *pVLArb = portp->pQOS->u.VLArbTable;

	VLArbXmlOutput(state, tag, &pVLArb[STL_VLARB_PREEMPT_ELEMENTS],
			STL_MAX_PREEMPT_CAP);
}	// End of VLArbPreemptElementsXmlOutput

/****************************************************************************/
/* PortData VLArbitrationPreemptMatrix Input/Output functions */

static void *VLArbPreemptMatrixXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData
	QOSData *pQOS = portp->pQOS;

	if (!pQOS) {
		if ( !( pQOS = portp->pQOS = (QOSData *)MemoryAllocate2AndClear(
				sizeof(QOSData), IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
			IXmlParserPrintError(state, "Unable to allocate memory");
			return (NULL);
		}
	}

	return ((void *)&(pQOS->u.VLArbTable[STL_VLARB_PREEMPT_MATRIX]));
	
}	// End of VLArbPreemptMatrixXmlParserStart

static void VLArbPreemptMatrixXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to PortData
	if (! valid)
		goto failvalidate;
	return;

failvalidate:
	PortDataFreeQOSData(IXmlParserGetContext(state), (PortData *)parent);
	return;

}	// End of VLArbPreemptMatrixXmlParserEnd

static void *VLArbPreemptMatrixXmlParserStartEntry(IXmlParserState_t *state, void *parent, const char **attr)
{
	// parent points to VLARBTABLE for PreemptMatrix
	STL_VLARB_TABLE *pVLArb = (STL_VLARB_TABLE *)parent;
	uint8 vl;

	if ( !attr || !attr[0] || (0 != strcmp(attr[0], "VL"))) {
		IXmlParserPrintError(state, "Missing VL attribute for VLArbitrationPreemptMatrix.MatrixEntry");
		return (NULL);
	}

	if (FSUCCESS != StringToUint8(&vl, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid VL attribute in VLArbitrationPreemptMatrix.MatrixEntry  VL: %s", attr[1]);
		return (NULL);
	}

	if (vl >= STL_MAX_VLS) {
		IXmlParserPrintError( state,
			"VL attribute Out-of-range in VLArbitrationPreemptMatrix.MatrixEntry VL:%u", vl );
		return (NULL);
	}

	return &(pVLArb->Matrix[vl]);

}	// End of VLArbPreemptMatrixXmlParserStartEntry

static void VLArbPreemptMatrixXmlParserEndEntry(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to VLARBTABLE for PreemptMatrix
	STL_VLARB_TABLE *pVLArb = (STL_VLARB_TABLE *)parent;
	// object points to PreemptMatrix entry for specific VL
	uint32 *pEntry = (uint32 *)object;
	uint32 vl = pEntry - pVLArb->Matrix;
	uint32 value;

	if (! valid)
		goto failvalidate;

	if (!content || !len) {
		IXmlParserPrintError(state, "No entry in VLArbitrationPreemptMatrix.MatrixEntry for VL %u", vl);
		return;
	}

	if (FSUCCESS != StringToUint32(&value, content, NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid entry in VLArbitrationPreemptMatrix.MatrixEntry for VL %u  MatrixEntry: %s", vl, content);
		return;
	}

	*pEntry = value;
	return;

failvalidate:
	// VLArbPreemptMatrixXmlParserEnd will free as needed
	return;

}

IXML_FIELD VLArbPreemptMatrixFields[] = {
	{ tag:"MatrixEntry", format:'k', start_func:VLArbPreemptMatrixXmlParserStartEntry, end_func:VLArbPreemptMatrixXmlParserEndEntry },
	{ NULL }
};

void VLArbPreemptMatrixXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	uint8 ix;
	PortData *portp = (PortData *)data; // data points to PortData
	STL_VLARB_TABLE *pVLArb = portp->pQOS->u.VLArbTable;

	IXmlOutputStartTag(state, tag);

	for(ix = 0; ix < STL_MAX_VLS; ix++)
	{
		IXmlOutputStartAttrTag(state, "MatrixEntry", &ix, VLArbXmlOutputVLAttr);
		IXmlOutputPrint(state, "%u", pVLArb[STL_VLARB_PREEMPT_MATRIX].Matrix[ix]);
		IXmlOutputEndTag(state, "MatrixEntry");
	}

	IXmlOutputEndTag(state, tag);
}	// end of VLArbPreemptMatrixXmlOutput

/****************************************************************************/
/* PortData PKeyTable Input/Output functions */

static void *PKeyTableXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData
	STL_PKEY_ELEMENT *pPartitionTable;
	uint32 num_pkeys;

	if (portp->pPartitionTable) {
		IXmlParserPrintError(state, "PKeyTable improperly allocated");
		return (NULL);
	}
	num_pkeys = PortPartitionTableSize(portp);

	if ( !( pPartitionTable = portp->pPartitionTable = (STL_PKEY_ELEMENT *)MemoryAllocate2AndClear(
			sizeof(STL_PKEY_ELEMENT) * num_pkeys, IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return (NULL);
	}

	ixPKey = 0;
	return ((void *)pPartitionTable);

}	// End of PKeyTableXmlParserStart

static void PKeyTableXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to PortData
	if (! valid)
		goto failvalidate;
	return;

failvalidate:
	PortDataFreePartitionTable(IXmlParserGetContext(state), (PortData *)parent);
	return;

}	// End of PKeyTableXmlParserEnd

static void *PKeyTableXmlParserStartPKey(IXmlParserState_t *state, void *parent, const char **attr)
{
	// parent points to PKeyTable

	return (parent);

}	// End of PKeyTableXmlParserStartPKey

static void PKeyTableXmlParserEndPKey(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to PKeyTable
	STL_PKEY_ELEMENT *pPartitionTable = (STL_PKEY_ELEMENT *)parent;
	uint16 pkey;

	if (! valid)
		goto failvalidate;

	// Assign pPartitionTable[ixPKey] = pkey
	if (!content || !len) {
		IXmlParserPrintError(state, "No PKey Value in PKeyTable.PKey");
		return;
	}

	if (FSUCCESS != StringToUint16(&pkey, content, NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid PKey Value in PKeyTable.PKey: %s", content);
		return;
	}

	pPartitionTable[ixPKey].AsReg16 = pkey;
	ixPKey++;
	return;

failvalidate:
	// PKeyTableXmlParserEnd will free as needed
	return;

}	// End of PKeyTableXmlParserEndPKey

IXML_FIELD PKeyTableFields[] = {
	{ tag:"PKey", format:'k', start_func:PKeyTableXmlParserStartPKey, end_func:PKeyTableXmlParserEndPKey },
	{ NULL }
};

void PKeyTableXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	int ix, last=0;
	PortData *portp = (PortData *)data;	// data points to PortData
	STL_PKEY_ELEMENT *pPKey = portp->pPartitionTable;
	int ix_capacity = PortPartitionTableSize(portp);

	IXmlOutputStartTag(state, tag);

	// find the last non-zero pkey in the table
	// we will output all pkeys, even if zero, up to the last
	// so that we properly retain the pkey indexes
	for (ix = 0; ix < ix_capacity; ix++)
	{
		if (pPKey[ix].AsReg16 & 0x7FFF)
			last = ix;
	}
	for (ix = 0; ix <= last; ix++)
	{
		IXmlOutputHexPad16(state, "PKey", pPKey[ix].AsReg16);
	}

	IXmlOutputEndTag(state, tag);

}	// End of PKeyTableXmlOutput

/****************************************************************************/
/* PortData Input/Output functions */

static void PortDataXmlFormatAttr(IXmlOutputState_t *state, void *data)
{
	PortData *portp = (PortData *)data;

	IXmlOutputPrint(state, " id=\"0x%016"PRIx64":%u\"", portp->nodep->NodeInfo.NodeGUID, portp->PortNum);
}

/* bitfields needs special handling: LID */
static void PortDataXmlOutputEndPortLID(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLIDValue(state, tag, ((PortData *)data)->EndPortLID);
}

static void PortDataXmlParserEndEndPortLID(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 value;
	
	if (IXmlParseUint32(state, content, len, &value))
		((PortData *)object)->EndPortLID = value;
}

/* bitfields needs special handling: PortState */
static void PortDataXmlOutputPortState(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputPortStateValue(state, tag, ((PortData *)data)->PortInfo.PortStates.s.PortState);
}

static void PortDataXmlParserEndPortState(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData *)object)->PortInfo.PortStates.s.PortState = value;
}

/* bitfields needs special handling: InitReason */
static void PortDataXmlOutputInitReason(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputInitReasonValue(state, tag, ((PortData *)data)->PortInfo.s3.LinkInitReason);
}

static void PortDataXmlParserEndInitReason(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData *)object)->PortInfo.s3.LinkInitReason = value;
}


/* bitfields needs special handling: PortPhysicalState */
static void PortDataXmlOutputPhysState(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputPortPhysStateValue(state, tag, ((PortData *)data)->PortInfo.PortStates.s.PortPhysicalState);
}

static void PortDataXmlOutputPortPhysConfig(IXmlOutputState_t *state, const char *tag, void *data)
{
	const uint8_t pt = ((PortData *)data)->PortInfo.PortPhysConfig.s.PortType;
	IXmlOutputStr(state, tag, StlPortTypeToText(pt));
}

static void PortDataXmlParserEndPhysState(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData *)object)->PortInfo.PortStates.s.PortPhysicalState = value;
}

/* bitfields needs special handling: LMC */
static void PortDataXmlOutputLMC(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag, ((PortData *)data)->PortInfo.s1.LMC);
}

static void PortDataXmlParserEndLMC(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData *)object)->PortInfo.s1.LMC = value;
}

/* bitfields needs special handling: M_KeyProtectBits */
static void PortDataXmlOutputMKeyProtect(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputMKeyProtectValue(state, tag, ((PortData *)data)->PortInfo.s1.M_KeyProtectBits);
}

static void PortDataXmlParserEndMKeyProtect(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData *)object)->PortInfo.s1.M_KeyProtectBits = value;
}

/* special handling: LinkWidthEnabled */
static void PortDataXmlOutputLinkWidthEnabled(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLinkWidthValue(state, tag,
		((PortData*)data)->PortInfo.LinkWidth.Enabled);
}

/* special handling: LinkWidthSupportedd */
static void PortDataXmlOutputLinkWidthSupported(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLinkWidthValue(state, tag,
		((PortData*)data)->PortInfo.LinkWidth.Supported);
}

/* special handling: LinkWidthActive */
static void PortDataXmlOutputLinkWidthActive(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLinkWidthValue(state, tag,
		((PortData*)data)->PortInfo.LinkWidth.Active);
}

/* special handling: LinkWidthEnabled */
static void PortDataXmlOutputLinkWidthDowngradeEnabled(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLinkWidthValue(state, tag,
		((PortData*)data)->PortInfo.LinkWidthDowngrade.Enabled);
}

/* special handling: LinkWidthSupportedd */
static void PortDataXmlOutputLinkWidthDowngradeSupported(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLinkWidthValue(state, tag,
		((PortData*)data)->PortInfo.LinkWidthDowngrade.Supported);
}

/* special handling: LinkWidthTxActive */
static void PortDataXmlOutputLinkWidthDowngradeTxActive(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLinkWidthValue(state, tag,
		((PortData*)data)->PortInfo.LinkWidthDowngrade.TxActive);
}

/* special handling: LinkWidthRxActive */
static void PortDataXmlOutputLinkWidthDowngradeRxActive(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLinkWidthValue(state, tag,
		((PortData*)data)->PortInfo.LinkWidthDowngrade.RxActive);
}

/* bitfields needs special handling: LinkSpeedSupported */
static void PortDataXmlOutputLinkSpeedSupported(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLinkSpeedValue( state, tag, ((PortData*)data)->PortInfo.LinkSpeed.Supported);
}

static void PortDataXmlParserEndLinkSpeedSupported(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint16 value;
	
	if (IXmlParseUint16(state, content, len, &value))
		((PortData*)object)->PortInfo.LinkSpeed.Supported = value;
}

/* bitfields needs special handling: LinkSpeedEnabled */
static void PortDataXmlOutputLinkSpeedEnabled(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLinkSpeedValue( state, tag, ((PortData*)data)->PortInfo.LinkSpeed.Enabled);
}

static void PortDataXmlParserEndLinkSpeedEnabled(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint16 value;
	
	if (IXmlParseUint16(state, content, len, &value))
		((PortData*)object)->PortInfo.LinkSpeed.Enabled = value;
}

/* bitfields needs special handling: LinkSpeedActive */
static void PortDataXmlOutputLinkSpeedActive(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLinkSpeedValue( state, tag, ((PortData*)data)->PortInfo.LinkSpeed.Active);
}

static void PortDataXmlParserEndLinkSpeedActive(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint16 value;
	
	if (IXmlParseUint16(state, content, len, &value))
		((PortData*)object)->PortInfo.LinkSpeed.Active = value;
}

static void PortDataXmlOutputMTUActiveVLAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " VL=\"%02u\"", *(uint8 *) data);
}

static void PortDataXmlOutputMTUs(IXmlOutputState_t *state, const char *tag, void *data)
{
	int outputVL;

	IXmlOutputStartTag(state, tag);

	for(outputVL = 0; outputVL < STL_MAX_VLS; outputVL++)
	{
		IXmlOutputStartAttrTag(state, "MTUActive", &outputVL, PortDataXmlOutputMTUActiveVLAttr);
		IXmlOutputPrint(state, "%04u", GetBytesFromMtu(GET_STL_PORT_INFO_NeighborMTU(&(((PortData*)data)->PortInfo), outputVL)));
		IXmlOutputEndTag(state, "MTUActive");
	}

	IXmlOutputEndTag(state, tag);

}	// End of PortDataXmlOutputMTUs

static void *PortDataXmlParserStartMTUActive(IXmlParserState_t *state, void *parent, const char **attr)
{
	// parent points to PortData
	/* since MTUActives don't nest, we can get away with a single static */
	/* since VL is selector for a packed array of 4 bits fields, can't return
	 * pointer to actual field
	 */
	static uint8 mtuVL;

	if( !attr | !attr[0] || (0 != strcmp(attr[0], "VL"))) {
		IXmlParserPrintError(state, "Missing VL attribute for MTUActive");
		return NULL;
	}

	if (FSUCCESS != StringToUint8(&mtuVL, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid VL attribute in MTUActive: %s", attr[1]);
		return NULL;
	}

	if (mtuVL >= STL_MAX_VLS) {
		IXmlParserPrintError(state, "VL attribute Out-of-Range in MTUActive: %s", attr[1]);
		return (NULL);
	}

	return &mtuVL;

}	// End of PortDataXmlParserStartMTUActive

static void PortDataXmlParserEndMTUActive(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData
	uint8 mtuVL = *(uint8*)object;
	uint16 value;
	
	if(IXmlParseUint16(state, content, len, &value))
		PUT_STL_PORT_INFO_NeighborMTU(&(portp->PortInfo), mtuVL, GetMtuFromBytes(value));
}	// End of PortDataXmlParserEndMTUActive

IXML_FIELD PortDataMTUActiveXmlFields[] = {
	{ tag:"MTUActive", format:'k', start_func:PortDataXmlParserStartMTUActive, end_func:PortDataXmlParserEndMTUActive },
	{ NULL }
};

/* bitfields needs special handling: MTUSupported */
static void PortDataXmlOutputMTUSupported(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputMtuValue(state, tag,
		((PortData*)data)->PortInfo.MTU.Cap);
}

static void PortDataXmlParserEndMTUSupported(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint16 value;
	
	if (IXmlParseUint16(state, content, len, &value))
		((PortData*)object)->PortInfo.MTU.Cap = GetMtuFromBytes(value);
}

/* bitfields needs special handling: SMSL */
static void PortDataXmlOutputSMSL(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag,
		((PortData*)data)->PortInfo.s2.MasterSMSL);
}

static void PortDataXmlParserEndSMSL(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.s2.MasterSMSL = value;
}

/* bitfields needs special handling: VLsActive */
static void PortDataXmlOutputVLsActive(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputVLsValue(state, tag,
		((PortData*)data)->PortInfo.s4.OperationalVL);
}

static void PortDataXmlParserEndVLsActive(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.s4.OperationalVL = value;
}

/* bitfields needs special handling: VLsSupported */
static void PortDataXmlOutputVLsSupported(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputVLsValue(state, tag,
		((PortData*)data)->PortInfo.VL.s2.Cap);
}

static void PortDataXmlParserEndVLsSupported(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.VL.s2.Cap = value;
}

/* VLStall */
static void PortDataXmlOutputVLStallVLAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " VL=\"%02u\"", *(uint8 *) data);
}

static void PortDataXmlOutputVLStalls(IXmlOutputState_t *state, const char *tag, void *data)
{
	uint8 vl;

	IXmlOutputStartTag(state, tag);
	for(vl = 0; vl < STL_MAX_VLS; vl++)
	{
		IXmlOutputStartAttrTag(state, "VLStall", &vl, PortDataXmlOutputVLStallVLAttr);
		IXmlOutputPrint(state, "%u", ((PortData*)data)->PortInfo.XmitQ[vl].VLStallCount);
		IXmlOutputEndTag(state, "VLStall");
	}
	IXmlOutputEndTag(state, tag);
}

static void *PortDataXmlParserStartVLStall(IXmlParserState_t *state, void *parent, const char **attr)
{
	// parent points to PortData
	uint8 vl;

	if( !attr | !attr[0] || (0 != strcmp(attr[0], "VL"))) {
		IXmlParserPrintError(state, "Missing VL attribute for VLStall");
		return NULL;
	}

	if (FSUCCESS != StringToUint8(&vl, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid VL attribute in VLStall: %s", attr[1]);
		return NULL;
	}

	if (vl >= STL_MAX_VLS) {
		IXmlParserPrintError(state, "VL attribute Out-of-Range in VLStall: %s", attr[1]);
		return (NULL);
	}

	return &((PortData *)parent)->PortInfo.XmitQ[vl];

}	// End of PortDataXmlParserStartVLStall

static void PortDataXmlParserEndVLStall(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to PortData, object points to VL specific XmitQ_s
	struct XmitQ_s *xmitQ = (struct XmitQ_s*)object;
	uint8 value;
	
	if(IXmlParseUint8(state, content, len, &value))
		xmitQ->VLStallCount = value;
}	// End of PortDataXmlParserEndVLStall

IXML_FIELD PortDataVLStallXmlFields[] = {
	{ tag:"VLStall", format:'k', start_func:PortDataXmlParserStartVLStall, end_func:PortDataXmlParserEndVLStall },
	{ NULL }
};

/* bitfields needs special handling: MulticastMask */
static void PortDataXmlOutputMulticastMask(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputHex(state, tag,
		((PortData*)data)->PortInfo.MultiCollectMask.MulticastMask);
}

static void PortDataXmlParserEndMulticastMask(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.MultiCollectMask.MulticastMask = value;
}

/* bitfields needs special handling: CollectiveMask */
static void PortDataXmlOutputCollectiveMask(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputHex(state, tag,
		((PortData*)data)->PortInfo.MultiCollectMask.CollectiveMask);
}

static void PortDataXmlParserEndCollectiveMask(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.MultiCollectMask.CollectiveMask = value;
}

/* HoQLife */
static void PortDataXmlOutputHoQLifeVLAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " VL=\"%02u\"", *(uint8 *) data);
}

static void PortDataXmlOutputHoQLifes(IXmlOutputState_t *state, const char *tag, void *data)
{
	uint8 vl;

	IXmlOutputStartTag(state, tag);
	for(vl = 0; vl < STL_MAX_VLS; vl++)
	{
		IXmlOutputStartAttrTag(state, "HoQLife_Int", &vl, PortDataXmlOutputHoQLifeVLAttr);
		IXmlOutputPrint(state, "%u", ((PortData*)data)->PortInfo.XmitQ[vl].HOQLife);
		IXmlOutputEndTag(state, "HoQLife_Int");
	}
	IXmlOutputEndTag(state, tag);
}

static void *PortDataXmlParserStartHoQLife(IXmlParserState_t *state, void *parent, const char **attr)
{
	// parent points to PortData
	uint8 vl;

	if( !attr | !attr[0] || (0 != strcmp(attr[0], "VL"))) {
		IXmlParserPrintError(state, "Missing VL attribute for HoQLife");
		return NULL;
	}

	if (FSUCCESS != StringToUint8(&vl, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid VL attribute in HoQLife: %s", attr[1]);
		return NULL;
	}

	if (vl >= STL_MAX_VLS) {
		IXmlParserPrintError(state, "VL attribute Out-of-Range in HoQLife: %s", attr[1]);
		return (NULL);
	}

	return &((PortData *)parent)->PortInfo.XmitQ[vl];

}	// End of PortDataXmlParserStartHoQLife

static void PortDataXmlParserEndHoQLife(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to PortData, object points to VL specific XmitQ_s
	struct XmitQ_s *xmitQ = (struct XmitQ_s*)object;
	uint8 value;
	
	if(IXmlParseUint8(state, content, len, &value))
		xmitQ->HOQLife = value;
}	// End of PortDataXmlParserEndHoQLife

IXML_FIELD PortDataHoQLifeXmlFields[] = {
	//{ tag:"HoQLife", format:'k', format_func: PortDataXmlOutputHoQLife, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"HoQLife_Int", format:'K', format_func: IXmlOutputNoop, start_func:PortDataXmlParserStartHoQLife, end_func:PortDataXmlParserEndHoQLife }, // input only bitfield
	{ NULL }
};

static void PortDataXmlOutputNeighborMTU(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortData * portp = (PortData *)data;
	STL_VL_TO_MTU * table = portp->PortInfo.NeighborMTU;
	uint8_t len = sizeof(portp->PortInfo.NeighborMTU) / sizeof(portp->PortInfo.NeighborMTU[0]);
	uint8_t i;

	for(i = 0; i < len; i++)
	{
		IXmlOutputStartTag(state, "NeighborMTU");
		IXmlOutputAttrFmt(state, "VL", "%u", i);
		IXmlOutputPrint(state, "0x%02x", table[i].AsReg8);
		IXmlOutputEndTag( state, "NeighborMTU");
	}
}

static void * PortDataXmlParserStartNeighborMtu(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortData * portp = (PortData *)parent;
	uint8_t len = sizeof(portp->PortInfo.NeighborMTU) / sizeof(portp->PortInfo.NeighborMTU[0]);
	uint8_t i = 0;
	if (!attr || !attr[0] || 0 != strcmp(attr[0], "VL")) {
		IXmlParserPrintError(state, "Missing VL attribute for delay element");
		return NULL;
	}
	if (FSUCCESS != StringToUint8(&i, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid VL in Neighbor MTU element: index:%s", attr[1]);
		return NULL;
	}
	if (i >= len) {
		IXmlParserPrintError(state, "VL value is out of range: index:%u len:%u", i, len);
		return NULL;
	}
	return (void *)(portp->PortInfo.NeighborMTU + i);
}

static void PortDataXmlParserEndNeighborMtu(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8_t value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VL_TO_MTU *)object)->AsReg8 = value;
}

IXML_FIELD FlitControlPreemptionXmlFields[] = {
	{ tag:"MinInitial", format:'U', IXML_FIELD_INFO(PortData, PortInfo.FlitControl.Preemption.MinInitial) },
	{ tag:"MinTail", format:'U', IXML_FIELD_INFO(PortData, PortInfo.FlitControl.Preemption.MinTail) },
	{ tag:"LargePktLimit", format:'U', IXML_FIELD_INFO(PortData, PortInfo.FlitControl.Preemption.LargePktLimit) },
	{ tag:"SmallPktLimit", format:'U', IXML_FIELD_INFO(PortData, PortInfo.FlitControl.Preemption.SmallPktLimit) },
	{ tag:"MaxSmallPktLimit", format:'U', IXML_FIELD_INFO(PortData, PortInfo.FlitControl.Preemption.MaxSmallPktLimit) },
	{ tag:"PreemptionLimit", format:'U', IXML_FIELD_INFO(PortData, PortInfo.FlitControl.Preemption.PreemptionLimit) },
	{ NULL }
};

static void FlitControlXmlOutputPreemption(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, data, NULL, FlitControlPreemptionXmlFields);
}

static void * FlitControlXmlParserStartPreemption(IXmlParserState_t *state, void *parent, const char **attr)
{
	return parent;
}

IXML_FIELD PortDataFlitControlXmlFields[] = {
	{ tag:"Interleave", format:'H', IXML_FIELD_INFO(PortData, PortInfo.FlitControl.Interleave.AsReg16) },
	{ tag:"Preemption", format:'K', format_func:FlitControlXmlOutputPreemption, subfields:FlitControlPreemptionXmlFields, start_func:FlitControlXmlParserStartPreemption, end_func:IXmlParserEndNoop },
	{ NULL }
};

static void PortDataXmlOutputFlitControl(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, data, NULL, PortDataFlitControlXmlFields);
}

static void * PortDataXmlParserStartFlitControl(IXmlParserState_t *state, void *parent, const char **attr)
{
	return parent;
}

/* bitfields needs special handling: P_KeyEnforcementInbound */
static void PortDataXmlOutputP_KeyEnforcementInbound(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOnOffValue(state, tag,
		((PortData*)data)->PortInfo.s3.PartitionEnforcementInbound);
}

static void PortDataXmlParserEndP_KeyEnforcementInbound(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.s3.PartitionEnforcementInbound = value;
}

/* bitfields needs special handling: P_KeyEnforcementOutbound */
static void PortDataXmlOutputP_KeyEnforcementOutbound(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOnOffValue(state, tag,
		((PortData*)data)->PortInfo.s3.PartitionEnforcementOutbound);
}

static void PortDataXmlParserEndP_KeyEnforcementOutbound(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.s3.PartitionEnforcementOutbound = value;
}

/* bitfields needs special handling: SubnetTimeout */
static void PortDataXmlOutputSubnetTimeout(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputTimeoutMultValue(state, tag,
		((PortData*)data)->PortInfo.Subnet.Timeout);
}

static void PortDataXmlParserEndSubnetTimeout(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.Subnet.Timeout = value;
}

/* bitfields needs special handling: RespTimeout */
static void PortDataXmlOutputRespTimeout(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputTimeoutMultValue(state, tag,
		((PortData*)data)->PortInfo.Resp.TimeValue);
}

static void PortDataXmlParserEndRespTimeout(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.Resp.TimeValue = value;
}

static void PortDataXmlOutputSLtoSCMap(IXmlOutputState_t *state, const char *tag, void *data)
{
	if ( ((PortData*)data)->nodep && ((PortData*)data)->pQOS &&
			((PortData*)data)->pQOS->SL2SCMap )
		SLtoSCMapXmlOutput(state, tag, data);

}	// End of PortDataXmlOutputSLtoSCMap

static void PortDataXmlOutputSCtoSLMap(IXmlOutputState_t *state, const char *tag, void *data)
{
	if ( ((PortData*)data)->nodep && ((PortData*)data)->pQOS &&
			((PortData*)data)->pQOS->SC2SLMap )
		SCtoSLMapXmlOutput(state, tag, data);

}	// End of PortDataXmlOutputSCtoSLMap

static void PortDataXmlOutputSCtoSCMap(IXmlOutputState_t *state, const char *tag, void *data)
{
	if ( ((PortData*)data)->nodep && ((PortData*)data)->pQOS &&
			((PortData*)data)->nodep->NodeInfo.NodeType == STL_NODE_SW &&
			((PortData*)data)->PortNum != 0)
		SCtoSCMapXmlOutput(state, tag, data);

}	// End of PortDataXmlOutputSCtoSLMap

static void PortDataXmlOutputSCtoVLxMap(IXmlOutputState_t *state, const char *tag, void *data, ScvlEnum_t scvlx)
{
	if( ((PortData*)data)->nodep && ((PortData*)data)->pQOS)
		SCtoVLxMapXmlOutput(state, tag, data, scvlx);
}

static void PortDataXmlOutputSCtoVLrMap(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortData *portp = (PortData *)data;
	NodeData *nodep = portp->nodep;
	if(getIsVLrSupported(nodep, portp))
		PortDataXmlOutputSCtoVLxMap(state, tag, data, Enum_SCVLr);
}

static void PortDataXmlOutputSCtoVLtMap(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortDataXmlOutputSCtoVLxMap(state, tag, data, Enum_SCVLt);
}

static void PortDataXmlOutputSCtoVLntMap(IXmlOutputState_t *state, const char *tag, void *data)
{
	if( !(((PortData*)data)->nodep->NodeInfo.NodeType == STL_NODE_SW &&
			((PortData*)data)->PortNum == 0))
		PortDataXmlOutputSCtoVLxMap(state, tag, data, Enum_SCVLnt);
}

/** =========================================================================
 * Buffer Control Table I/O functions
 */
void BCTXmlOutputVLLimitAttr(struct IXmlOutputState *state, void *data)
{
	IXmlOutputPrint(state, " VL=\"%u\"", *(uint8 *)data);
}

static void *BCTXmlParserStart(IXmlParserState_t *state, void *parent, const char **atr)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData

	if (portp->pBufCtrlTable) {
		IXmlParserPrintError(state, "BufCtrlTable improperly allocated");
		return (NULL);
	}

	if ( !( portp->pBufCtrlTable = (STL_BUFFER_CONTROL_TABLE*)MemoryAllocate2AndClear(
			sizeof(STL_BUFFER_CONTROL_TABLE), IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return (NULL);
	}

	return ((void *)portp->pBufCtrlTable);

}

static void PortDataXmlOutputBCT(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortData *portp = (PortData *)data;	// data points to PortData
	STL_BUFFER_CONTROL_TABLE *bct = portp->pBufCtrlTable;
	uint8_t vl;

	if (! bct)
		return;

	IXmlOutputStartTag(state, tag);

	IXmlOutputStartTag(state, "TxOverallSharedLimit");
	IXmlOutputPrint(state, "%u", bct->TxOverallSharedLimit);
	IXmlOutputEndTag(state, "TxOverallSharedLimit");

	for (vl = 0; vl < STL_MAX_VLS; vl++)
	{
		IXmlOutputStartAttrTag( state, "VLLimit", &vl, BCTXmlOutputVLLimitAttr );
		IXmlOutputStartTag(state, "TxDedicatedLimit");
		IXmlOutputPrint(state, "%u", bct->VL[vl].TxDedicatedLimit);
		IXmlOutputEndTag(state, "TxDedicatedLimit");
		IXmlOutputStartTag(state, "TxSharedLimit");
		IXmlOutputPrint(state, "%u", bct->VL[vl].TxSharedLimit);
		IXmlOutputEndTag(state, "TxSharedLimit");
		IXmlOutputEndTag( state, "VLLimit");
	}
	IXmlOutputEndTag(state, tag);
}

static void *BCTVLLimitStartFunc(IXmlParserState_t *state, void *parent, const char **attr)
{
	STL_BUFFER_CONTROL_TABLE *bct = (STL_BUFFER_CONTROL_TABLE *)parent;	// parent points to BufCtrlTable
	uint8_t vl;

	if ( !attr || !attr[0] || (0 != strcmp(attr[0], "VL"))) {
		IXmlParserPrintError(state, "Missing VL attribute in BufferControlTable.VLLimit");
		return (NULL);
	}

	if (FSUCCESS != StringToUint8(&vl, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid VL attribute in BufferControlTable.VLLimit  VL %s", attr[1]);
		return (NULL);
	}

	if (vl >= STL_MAX_VLS) {
		IXmlParserPrintError(state, "VL attribute Out-of-Range in BufferControlTable.VLLimit  VL: %s", attr[1]);
		return (NULL);
	}

	return ((void *)&bct->VL[vl]);
}

/** =========================================================================
 * Buffer Control Table Feild Definitions
 */
// parent is STL_BUFFER_CONTROL_TABLE_VL_s
static IXML_FIELD BCTVLLimitFields[] = {
	{ tag:"TxDedicatedLimit", format:'U', IXML_FIELD_INFO(struct STL_BUFFER_CONTROL_TABLE_VL_s, TxDedicatedLimit) },
	{ tag:"TxSharedLimit", format:'U', IXML_FIELD_INFO(struct STL_BUFFER_CONTROL_TABLE_VL_s, TxSharedLimit) },
	{ NULL }
};

// parent is BufCtrlTable
static IXML_FIELD BufferControlTableFields[] = {
	{ tag:"TxOverallSharedLimit", format:'U', IXML_FIELD_INFO(STL_BUFFER_CONTROL_TABLE, TxOverallSharedLimit) },
	{ tag:"VLLimit", format:'k', subfields:BCTVLLimitFields, start_func:BCTVLLimitStartFunc },
	{ NULL }
};

/* End Buffer Control Table Definitions */
/* ========================================================================= */


static void PortDataXmlOutputVLArbLow(IXmlOutputState_t *state, const char *tag, void *data)
{
	int vlarb;
	int res = getVLArb((PortData*)data, &vlarb);
	DEBUG_ASSERT(!res);
	if (res) {
		fprintf(stderr, "Error-%s: failed to determine "
			"Failed to determine if vlarb is supported. Not outputting VLArbLow\n", __func__);
		return;
	}

	if (((PortData*)data)->pQOS && vlarb)
		VLArbLowXmlOutput(state, tag, data);

}	// End of PortDataXmlOutputVLArbLow

static void PortDataXmlOutputVLArbHigh(IXmlOutputState_t *state, const char *tag, void *data)
{
	int vlarb;
	int res = getVLArb((PortData*)data, &vlarb);
	DEBUG_ASSERT(!res);
	if (res) {
		fprintf(stderr, "Error-%s: failed to determine "
			"Failed to determine if vlarb is supported. Not outputting VLArbHigh\n", __func__);
		return;
	}

	if (((PortData*)data)->pQOS && vlarb)
		VLArbHighXmlOutput(state, tag, data);

}	// End of PortDataXmlOutputVLArbHigh

static void PortDataXmlOutputVLArbPreemptElements(IXmlOutputState_t *state, const char *tag, void *data)
{
	int vlarb;
	int res = getVLArb((PortData*)data, &vlarb);
	DEBUG_ASSERT(!res);
	if (res) {
		fprintf(stderr, "Error-%s: failed to determine "
			"Failed to determine if vlarb is supported. Not outputting VLArbPreempt\n", __func__);
		return;
	}

	if (((PortData*)data)->pQOS && vlarb)
		VLArbPreemptElementsXmlOutput(state, tag, data);

}	// End of PortDataXmlOutputVLArbPreemptElements

static void PortDataXmlOutputVLArbPreemptMatrix(IXmlOutputState_t *state, const char *tag, void *data)
{
	int vlarb;
	int res = getVLArb((PortData*)data, &vlarb);
	DEBUG_ASSERT(!res);
	if (res) {
		fprintf(stderr, "Error-%s: failed to determine "
			"Failed to determine if vlarb is supported. Not outputting VLArbPreemptMatrix\n", __func__);
		return;
	}

	if (((PortData*)data)->pQOS && vlarb)
		VLArbPreemptMatrixXmlOutput(state, tag, data);
}

static void PortDataXmlOutputPKeyTable(IXmlOutputState_t *state, const char *tag, void *data)
{
	if (((PortData*)data)->nodep && ((PortData*)data)->pPartitionTable)
		PKeyTableXmlOutput(state, tag, data);

}	// End of PortDataXmlOutputPKeyTable

static void PortDataXmlOutputPortStatusData(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortData *portp = (PortData*)data;

	PortStatusDataXmlOutputOptional(state, "PortStatus", portp->pPortCounters);
}

/* bitfields needs special handling: PassThroughControlDRControl */
static void PortDataXmlOutputPassThroughControlDRControl(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag, ((PortData*)data)->PortInfo.PassThroughControl.DRControl);
}

static void PortDataXmlParserEndPassThroughControlDRControl(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.PassThroughControl.DRControl = value;
}

/* bitfields needs special handling: SubnetMulticastPKeyTrapSuppressionEnabled */
static void PortDataXmlOutputSubnetMulticastPKeyTrapSuppressionEnabled(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag, ((PortData*)data)->PortInfo.Subnet.MulticastPKeyTrapSuppressionEnabled);
}

static void PortDataXmlParserEndSubnetMulticastPKeyTrapSuppressionEnabled(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.Subnet.MulticastPKeyTrapSuppressionEnabled = value;
}

/* bitfields needs special handling: Subnet.ClientReregister */
static void PortDataXmlOutputSubnetClientReregister(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag, ((PortData*)data)->PortInfo.Subnet.ClientReregister);
}

static void PortDataXmlParserEndSubnetClientReregister(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value))
		((PortData*)object)->PortInfo.Subnet.ClientReregister = value;
}

/** =========================================================================
 * CableInfo definitions
 */

static void *CableInfoXmlParserStartSegValue(IXmlParserState_t *state, void *parent, const char **attr)
{
	// parent points to CableInfoData
	uint32 seg;

	if ( !attr || !attr[0] || 0 != strcmp(attr[0], "Seg")) {
		IXmlParserPrintError(state, "Missing Seg attribute for CableInfo.Value");
		return NULL;
	}
	if (FSUCCESS != StringToUint32(&seg, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid Seg attribute for CableInfo.Value  Seg: %s", attr[1]);
		return NULL;
	}
	if (seg >= STL_CIB_STD_LEN / sizeof(uint64) ) {
		IXmlParserPrintError(state, "Seg attribute Out-of-Range in CableInfo: %s", attr[1]);
		return NULL;
	}

	return (&((uint8 *)parent)[seg * sizeof(uint64)]);

}	// End of CableInfoXmlParserStartSegValue()

static void CableInfoXmlParserEndSegValue(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to CableInfoData; object points to segment-specific CableInfoData
	uint8 *pCableInfoData = (uint8 *)parent;
	uint8 *pCableInfoSegData = (uint8 *)object;
	uint64 value;

	if (! valid)
		return;
	if ( !pCableInfoData || !pCableInfoSegData || !content || !len ||
			(pCableInfoSegData < pCableInfoData) ||
			(pCableInfoSegData > pCableInfoData + STL_CIB_STD_LEN) ) {
		IXmlParserPrintError(state, "CableInfo improperly allocated");
		return;
	}

	if(IXmlParseUint64(state, content, len, &value)) {
#if CPU_LE
		value = ntoh64(value);
#endif
		*(uint64*)pCableInfoSegData = value;
	}

}	// End of CableInfoXmlParserEndSegValue()

const IXML_FIELD CableInfoFields[] = {
	{ tag:"Value", start_func:CableInfoXmlParserStartSegValue, end_func:CableInfoXmlParserEndSegValue },
	{ NULL }
};

static void *CableInfoXmlParserStart(IXmlParserState_t *state, void *parent, const char **atr)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData

	if (portp->pCableInfoData) {
		IXmlParserPrintError(state, "CableInfo improperly allocated");
		return (NULL);
	}

	if ( !( portp->pCableInfoData = MemoryAllocate2AndClear(
			STL_CABLE_INFO_PAGESZ, IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return (NULL);
	}

	return ((void *)portp->pCableInfoData);
}

static void CableInfoOutputSegAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " Seg=\"0x%x\"", *(unsigned int *)data);
}

static void PortDataXmlOutputCableInfo(IXmlOutputState_t *state, const char *tag, void *data)
{
	unsigned int ix_seg;
	PortData *portp = (PortData*)data;
	uint64 *pData = (uint64 *)portp->pCableInfoData;
	uint64 temp;

	if (! pData)
		return;

	IXmlOutputStartTag(state, tag);

	for ( ix_seg = 0; ix_seg < STL_CABLE_INFO_PAGESZ / sizeof(uint64);
			pData++, ix_seg++ ) {
		temp = *pData;
#if CPU_LE
		temp = ntoh64(temp);
#endif
		IXmlOutputStartAttrTag(state, "Value", &ix_seg, CableInfoOutputSegAttr);
		IXmlOutputPrint(state, "0x%016lx", temp);
		IXmlOutputEndTag(state, "Value");
	}

	IXmlOutputEndTag(state, tag);
}


/** =========================================================================
 * HFI Congestion Control Table definitions
 */

static void *HFICCTXmlParserStartSegValue(IXmlParserState_t *state, void *parent, const char **attr)
{
	uint32 seg = 0;

	if (parent == NULL)
		return NULL;

	const uint32 seg_max = ((STL_CONGESTION_CONTROL_ENTRY_MAX_VALUE * sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_ENTRY)) /
			 sizeof(uint64));

	if ( !attr || !attr[0] || 0 != strcmp(attr[0], "Seg")) {
		IXmlParserPrintError(state, "Missing Seg attribute for HFICCT.Value");
		return NULL;
	}
	if (FSUCCESS != StringToUint32(&seg, attr[1], NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid Seg attribute for HFICCT.Value Seg: %s", attr[1]);
		return NULL;
	}
	if (seg >= seg_max) {
		IXmlParserPrintError(state, "Seg attribute Out-of-Range in HFICCT: %s", attr[1]);
		return NULL;
	}

	return &(((uint8 *)parent)[seg * sizeof(uint64)]);
}

static void HFICCTXmlParserEndSegValue(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	// parent points to HFICCTData; object points to segment-specific HFICCTData
	uint8 *pHFICCTData = (uint8 *)parent;
	uint8 *pHFICCTSegData = (uint8 *)object;
	uint64 value;

	const uint32 seg_max = ((STL_CONGESTION_CONTROL_ENTRY_MAX_VALUE * sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_ENTRY)) /
			 sizeof(uint64));

	if (!valid)
		return;

	if (!pHFICCTData)
		return;

	if (!pHFICCTSegData || !content || !len ||
			(pHFICCTSegData < pHFICCTData) ||
			(pHFICCTSegData > pHFICCTData + seg_max)) {
		IXmlParserPrintError(state, "HFICCT improperly allocated");
		return;
	}

	if(IXmlParseUint64(state, content, len, &value)) {
#if CPU_LE
		value = ntoh64(value);
#endif
		*(uint64*)pHFICCTSegData = value;
	}

}	// End of HFICCTXmlParserEndSegValue()

static void *HFICCTXmlParserStart(IXmlParserState_t *state, void *parent, const char **atr)
{
	PortData *portp = (PortData *)parent;	// parent points to PortData
	unsigned int adjust = 0;

	if (portp->CCTI_Limit == 0)
		return (NULL);

	if (portp->CCTI_Limit >= STL_CONGESTION_CONTROL_ENTRY_MAX_VALUE) {
		IXmlParserPrintError(state, "HFI Congestion Control Table Limit Index (CCTI_Limit) is invalid");
		return (NULL);
	}

	if (portp->pCongestionControlTableEntries) {
		IXmlParserPrintError(state, "HFI Congestion Control Table improperly allocated");
		return (NULL);
	}

	// Minimum allocation is 128 entries
	if (portp->CCTI_Limit < (STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES * 2))
		adjust = STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES * 2;
	else
		adjust = portp->CCTI_Limit + 1;

	// allocation must be adjusted to a multiple of blocks entries
	if (adjust % STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES) {
		adjust = ((adjust/STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES) + 1) *
			 STL_NUM_CONGESTION_CONTROL_ELEMENTS_BLOCK_ENTRIES;
	}

	if (!(portp->pCongestionControlTableEntries = MemoryAllocate2AndClear(
			adjust * sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_ENTRY),
			IBA_MEM_FLAG_PREMPTABLE, MYTAG))) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return (NULL);
	}

	return ((void *)portp->pCongestionControlTableEntries);
}

static void HFICCTOutputSegAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " Seg=\"0x%x\"", *(unsigned int *)data);
}

static void PortDataXmlOutputHFICCT(IXmlOutputState_t *state, const char *tag, void *data)
{
	unsigned int ix_seg;
	PortData *portp = (PortData*)data;
	uint64 *pData = (uint64 *)portp->pCongestionControlTableEntries;
	uint64 temp;
	uint32 len = 0;

	if (!pData)
		return;

	if (portp->CCTI_Limit == 0  || portp->CCTI_Limit >= STL_CONGESTION_CONTROL_ENTRY_MAX_VALUE)
		return;

	len = (portp->CCTI_Limit + 1) * sizeof(STL_HFI_CONGESTION_CONTROL_TABLE_ENTRY);

	IXmlOutputStartTag(state, tag);

	for (ix_seg = 0; ix_seg < len / sizeof(uint64); pData++, ix_seg++) {
		temp = *pData;
#if CPU_LE
		temp = ntoh64(temp);
#endif
		IXmlOutputStartAttrTag(state, "Value", &ix_seg, HFICCTOutputSegAttr);
		IXmlOutputPrint(state, "0x%016lx", temp);
		IXmlOutputEndTag(state, "Value");
	}

	IXmlOutputEndTag(state, tag);
}

const IXML_FIELD HFICCTFields[] = {
	{ tag:"Value", start_func:HFICCTXmlParserStartSegValue, end_func:HFICCTXmlParserEndSegValue },
	{ NULL }
};

// parse Rate_Int into a uint8 field and validate value
void McgMCRateXmlParserEnd_Int(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint16 value;
	McGroupData *mcgmemberp = (McGroupData*) object;

	if (IXmlParseUint16(state, content, len, &value)) {
		if (value <= IB_STATIC_RATE_LAST)
			mcgmemberp->GroupInfo.Rate = value;
		else
			IXmlParserPrintError(state, "Invalid Rate: %u\n", value);
	}
	else
		IXmlParserPrintError(state, "Invalid Rate: %s\n", content);
}

static void McgMLIDXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 value;
	McGroupData *mcgmemberp = (McGroupData *)object;

	if (IXmlParseUint32(state, content, len, &value)) {
		// MLID must be either 16-bit format (0xCXXX) or 32-bit format (0xFXXXXXXX)
		if (IS_MCAST16(value))
			mcgmemberp->MLID = MCAST16_TO_MCAST32(value);
		else if (IS_MCAST32(value))
			mcgmemberp->MLID = value;
		else IXmlParserPrintError(state, "Invalid MLID value: 0x%04x\n", value);
	}
	else
		IXmlParserPrintError(state, "Cannot parse \"%s\" as MLID\n", content);
}

static void McgMGIDXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	McGroupData *mcgmemberp = (McGroupData *)object;
	uint64 mgid[2];

	if (FSUCCESS != StringToGid(&mgid[0], &mgid[1], content, NULL, TRUE)) {
		IXmlParserPrintError(state, "Cannot parse \"%s\" as MGID\n", content);
		return;
	}

	mcgmemberp->MGID.AsReg64s.H=mgid[0];
	mcgmemberp->MGID.AsReg64s.L=mgid[1];

	return;
}

static void McgMtuXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint16 value;

	if (IXmlParseUint16(state, content, len, &value))
			((McGroupData *)object)->GroupInfo.Mtu = GetMtuFromBytes(value);
	else IXmlParserPrintError(state, "Invalid Mtu: %s\n", content);

}

static void McgPktLifeTimeXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value))
		((McGroupData *)object)->GroupInfo.PktLifeTime = value;
	else IXmlParserPrintError(state, "Invalid PktLifeTime: %s\n", content);

}

static void McgSLXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 value;

	if (IXmlParseUint32(state, content, len, &value))
		((McGroupData *)object)->GroupInfo.u1.s.SL = value;
	else IXmlParserPrintError(state, "Invalid SL: %s\n", content);

}

static void McgHopLimitXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid){
	uint32 value;

	if (IXmlParseUint32(state, content, len, &value))
		((McGroupData *)object)->GroupInfo.u1.s.HopLimit = value;
	else IXmlParserPrintError(state, "Invalid HopLimit: %s\n", content);

}

static void McgFlowLabelXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 value;

	if (IXmlParseUint32(state, content, len, &value))
		((McGroupData *)object)->GroupInfo.u1.s.FlowLabel = value;
	else IXmlParserPrintError(state, "Invalid Flow Level: %s\n", content);

}

static void *McPortGIDXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	McMemberData *mcmemberp = (McMemberData*)MemoryAllocate2AndClear(sizeof(McMemberData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);


	if (!mcmemberp) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	if ( (!attr | !attr[0]) || (0 != strcmp(attr[0], "id"))) {
		IXmlParserPrintError(state, "Missing PortGID id");
		MemoryDeallocate(mcmemberp);
		return NULL;
	}

	// basic initialization of the MCG structure
	ListItemInitState(&mcmemberp->McMembersEntry);
	QListSetObj(&mcmemberp->McMembersEntry,mcmemberp);

	// no preexisting membership info
	mcmemberp->MemberInfo.JoinFullMember=0;
	mcmemberp->MemberInfo.JoinNonMember=0;
	mcmemberp->MemberInfo.JoinSendOnlyMember=0;

	return mcmemberp;
}

static void McPortGIDXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	FabricData_t *fabricp = IXmlParserGetContext(state);
	McMemberData *mcmemberp = (McMemberData *)object;
	McGroupData *mcgmemberp = (McGroupData*)parent;

	if (!valid) {
		MemoryDeallocate(mcmemberp);
		return;
	}

	mcmemberp->MemberInfo.RID.MGID = mcgmemberp->MGID;
	mcmemberp->MemberInfo.MLID = mcgmemberp->MLID;
	mcmemberp->MemberInfo.P_Key = mcgmemberp->GroupInfo.P_Key;
	mcmemberp->MemberInfo.Q_Key = mcgmemberp->GroupInfo.Q_Key;
	mcmemberp->MemberInfo.Mtu = mcgmemberp->GroupInfo.Mtu;
	mcmemberp->MemberInfo.Rate = mcgmemberp->GroupInfo.Rate;
	mcmemberp->MemberInfo.PktLifeTime = mcgmemberp->GroupInfo.PktLifeTime;
	mcmemberp->MemberInfo.u1.s.SL = mcgmemberp->GroupInfo.u1.s.SL;
	mcmemberp->MemberInfo.u1.s.HopLimit = mcgmemberp->GroupInfo.u1.s.HopLimit;
	mcmemberp->MemberInfo.u1.s.FlowLabel = mcgmemberp->GroupInfo.u1.s.FlowLabel;
	mcmemberp->MemberInfo.TClass = mcgmemberp->GroupInfo.TClass;

	// inset mcmember in the groups structure
	QListInsertTail(&mcgmemberp->AllMcGroupMembers, &mcmemberp->McMembersEntry);

	mcmemberp->pPort = NULL;

	if ((mcmemberp->MemberInfo.RID.PortGID.AsReg64s.H != 0) && (mcmemberp->MemberInfo.RID.PortGID.AsReg64s.L != 0)) {
		// attach port to the PortGID
		mcmemberp->pPort = FindPortGuid(fabricp, mcmemberp->MemberInfo.RID.PortGID.AsReg64s.L);
		//verify there is a port and that there is a neighbor
		if (mcmemberp->pPort && mcmemberp->pPort->neighbor) {
			// add switches that belong to this group
			if (mcmemberp->pPort->neighbor->nodep->NodeInfo.NodeType == STL_NODE_SW) {
				NodeData *groupswitch = mcmemberp->pPort->neighbor->nodep;
				uint8 switchentryport = mcmemberp->pPort->neighbor->PortNum;
				if (FSUCCESS != AddEdgeSwitchToGroup(fabricp, mcgmemberp, groupswitch, switchentryport)) {
					IXmlParserPrintError(state, "No switch found for MC Group\n");
					return;
				}
			}
		}
		mcgmemberp->NumOfMembers++;
	}

	return;
}

static void McPGIDXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	McMemberData *mcmemberp = (McMemberData*)object;
	uint64 pgid[2];

	if (! valid)	//missing mandatory fields
		return;

	if (FSUCCESS != StringToGid(&pgid[0], &pgid[1], content, NULL, TRUE)) {
		IXmlParserPrintError(state, "Invalid PortGID found in hex string %s\n", content);
		return;
	}

	mcmemberp->MemberInfo.RID.PortGID.AsReg64s.H=pgid[0];
	mcmemberp->MemberInfo.RID.PortGID.AsReg64s.L=pgid[1];

	return;
}

static void McMembershipXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	McMemberData *mcmemberp = (McMemberData*)object;
	uint8 value;

	if (! IXmlParseUint8(state, content, len, &value)) {
		IXmlParserPrintError(state, "Illegal Member type found: (%s)\n", content);
		return;
	}

	if (value != 0) {
		mcmemberp->MemberInfo.JoinFullMember = value & 1;
		value = value >> 1;
		mcmemberp->MemberInfo.JoinNonMember = value & 1;
		value = value >> 1;
		mcmemberp->MemberInfo.JoinSendOnlyMember = value & 1;
	}

	return;
}

static void *McgXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	McGroupData *mcgmemberp = (McGroupData*)MemoryAllocate2AndClear(sizeof(McGroupData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);

	if (! mcgmemberp) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	if ( (!attr | !attr[0]) || (0 != strcmp(attr[0], "id"))) {
		IXmlParserPrintError(state, "Missing MGID id");
		MemoryDeallocate(mcgmemberp);
		return NULL;
	}
	ListItemInitState(&mcgmemberp->AllMcGMembersEntry);
	QListSetObj(&mcgmemberp->AllMcGMembersEntry, mcgmemberp);
		// init LIST of Group members to NULL
	QListInitState(&mcgmemberp->AllMcGroupMembers);
	if ( !QListInit(&mcgmemberp->AllMcGroupMembers)) {
		IXmlParserPrintError(state, "Unable to initialize MCGroup member list");
		MemoryDeallocate(mcgmemberp);
		return NULL;
	}

	//init list of switches in a group
	QListInitState(&mcgmemberp->EdgeSwitchesInGroup);
	if ( !QListInit(&mcgmemberp->EdgeSwitchesInGroup)) {
		IXmlParserPrintError(state, "Unable to initialize list of switches for the current MC group");
		MemoryDeallocate(mcgmemberp);
		return NULL;
	}

	//init number of member counters for groups and group members
	mcgmemberp->NumOfMembers = 0;
	return mcgmemberp;
}

static void McgXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field,
	void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	McGroupData *mcgmemberp = (McGroupData*)object;
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (!valid) {	// missing mandatory fields
		MemoryDeallocate(mcgmemberp);
		return;
	}

	// TODO is this correct? Should NumOfMcGroups be incremented only if the head McMemberData
	// object GID != 0? Should NumOfMcGroups be incremented if there's at least one McMemberData
	// object whose GID != 0 in AllMcGroupMembers?
	McMemberData *pMCH = (McMemberData *)QListObj(QListHead(&mcgmemberp->AllMcGroupMembers));
	if ((pMCH->MemberInfo.RID.PortGID.AsReg64s.H !=0) || (pMCH->MemberInfo.RID.PortGID.AsReg64s.L != 0))
		fabricp->NumOfMcGroups++;
	// insert mcmember in the fabric structure
	QListInsertTail(&fabricp->AllMcGroups, &mcgmemberp->AllMcGMembersEntry);
	return;
}

static void GIDXmlOutput(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " id=\"0x%016"PRIx64":0x%016"PRIx64"\"",
			((IB_GID*)data)->AsReg64s.H,
			((IB_GID*)data)->AsReg64s.L);
}

static void McMembershipXmlOutput(IXmlOutputState_t *state, const char* tag, void *data)
{
	McMemberData *pMcMemberRecord = (McMemberData *)data;
	uint8	Memberstatus;

	Memberstatus = (pMcMemberRecord->MemberInfo.JoinSendOnlyMember<<2 |
				pMcMemberRecord->MemberInfo.JoinNonMember<<1 |
				pMcMemberRecord->MemberInfo.JoinFullMember);

	IXmlOutputUint(state, tag, Memberstatus);
}


static void McGroupDataXmlOutput(IXmlOutputState_t *state, McGroupData *pMcGroupRecord)
{
	IXmlOutputGID(state, "MGID", &pMcGroupRecord->MGID );
	IXmlOutputLIDValue(state, "MLID", pMcGroupRecord->MLID);
	IXmlOutputPKey(state, "P_Key", &pMcGroupRecord->GroupInfo.P_Key);
	IXmlOutputUint(state, "Mtu", GetBytesFromMtu(pMcGroupRecord->GroupInfo.Mtu));
	IXmlOutputRateValue(state, "Rate", pMcGroupRecord->GroupInfo.Rate);
	IXmlOutputTimeoutMultValue(state, "PktLifeTime", pMcGroupRecord->GroupInfo.PktLifeTime);
	IXmlOutputHexPad32(state, "Q_Key", pMcGroupRecord->GroupInfo.Q_Key);
	IXmlOutputHex(state, "SL", pMcGroupRecord->GroupInfo.u1.s.SL);
	IXmlOutputHex(state, "HopLimit", pMcGroupRecord->GroupInfo.u1.s.HopLimit);
	IXmlOutputHex(state, "FlowLabel", pMcGroupRecord->GroupInfo.u1.s.FlowLabel);
	IXmlOutputHexPad8(state, "TClass", pMcGroupRecord->GroupInfo.TClass);
}

static void McGroupMemberXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	McGroupData *pMcGroupRecord = (McGroupData *)data;
	McMemberData *mcmemberp;
	LIST_ITEM *p;

	IXmlOutputStartAttrTag(state, tag, &pMcGroupRecord->MGID, GIDXmlOutput);
	p=QListHead(&pMcGroupRecord->AllMcGroupMembers);
	McGroupDataXmlOutput(state, pMcGroupRecord);

	for (p=QListHead(&pMcGroupRecord->AllMcGroupMembers); p != NULL; p = QListNext(&pMcGroupRecord->AllMcGroupMembers, p)) {
		mcmemberp = (McMemberData *)QListObj(p);
		IXmlOutputStartAttrTag(state, "PortGID", &mcmemberp->MemberInfo.RID.PortGID, GIDXmlOutput);
		IXmlOutputGID(state,"GID",&mcmemberp->MemberInfo.RID.PortGID );
		McMembershipXmlOutput(state, "Membership_Int", mcmemberp);
		IXmlOutputEndTag(state,"PortGID");
	}
	IXmlOutputEndTag(state,tag);
}

static IXML_FIELD McPortGIDFields[] = {
		{ tag:"GID", format:'k', end_func:McPGIDXmlParserEnd },
		{ tag:"Membership_Int", format:'k', end_func:McMembershipXmlParserEnd},
		{ NULL},
};

static IXML_FIELD McgFields[] = {
{ tag:"MGID", format:'k', end_func: McgMGIDXmlParserEnd },
{ tag:"MLID", format:'h', end_func: McgMLIDXmlParserEnd },
{ tag:"P_Key", format:'h', IXML_FIELD_INFO(McGroupData, GroupInfo.P_Key), format_func:IXmlOutputPKey},
{ tag:"Rate_Int", format:'k', end_func: McgMCRateXmlParserEnd_Int },
{ tag:"Mtu", format: 'h', end_func:McgMtuXmlParserEnd },
{ tag:"PktLifeTime_Int", format:'h', end_func:McgPktLifeTimeXmlParserEnd },
{ tag:"Q_Key", format:'h', IXML_FIELD_INFO(McGroupData, GroupInfo.Q_Key) },
{ tag:"SL", format:'h', end_func: McgSLXmlParserEnd },
{ tag:"HopLimit", format:'h', end_func: McgHopLimitXmlParserEnd },
{ tag:"FlowLabel", format:'h', end_func: McgFlowLabelXmlParserEnd },
{ tag:"TClass", format:'h', IXML_FIELD_INFO(McGroupData, GroupInfo.TClass) },
{ tag:"PortGID", format:'k', subfields:McPortGIDFields, start_func:McPortGIDXmlParserStart, end_func:McPortGIDXmlParserEnd },
{ NULL }
};


static IXML_FIELD MulticastFields[] = {
	{ tag:"MulticastGroup", format:'k', subfields:McgFields, start_func:McgXmlParserStart, end_func:McgXmlParserEnd }, // structure
	{ NULL }
};


static void PortDataXmlOutputDownReason(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputDownReasonValue(state, tag, ((STL_LINKDOWN_REASON *)data)->LinkDownReason);
}
static void PortDataXmlParserEndDownReason(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value))
		((STL_LINKDOWN_REASON *)object)->LinkDownReason = value;
}
static void PortDataXmlOutputNeighborDownReason(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputDownReasonValue(state, tag, ((STL_LINKDOWN_REASON *)data)->NeighborLinkDownReason);
}
static void PortDataXmlParserEndNeighborDownReason(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value))
		((STL_LINKDOWN_REASON *)object)->NeighborLinkDownReason = value;
}

// LinkDownReasonLog
static IXML_FIELD LDRLogFields[] = {
	{ tag:"LinkDownReason", format:'k', format_func:PortDataXmlOutputDownReason, end_func:IXmlParserEndNoop },
	{ tag:"LinkDownReason_Int", format:'K', format_func:IXmlOutputNoop, end_func:PortDataXmlParserEndDownReason },
	{ tag:"NeighborLinkDownReason", format:'k', format_func:PortDataXmlOutputNeighborDownReason, end_func:IXmlParserEndNoop },
	{ tag:"NeighborLinkDownReason_Int", format:'K', format_func:IXmlOutputNoop, end_func:PortDataXmlParserEndNeighborDownReason },
	{ tag:"Timestamp", format:'U', IXML_FIELD_INFO(STL_LINKDOWN_REASON, Timestamp) },
	{ NULL }
};

static void LDRLogEntryXmlFormatAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " idx=\"%d\"", *(int *)data);
}
static void PortDataXmlOutputLDRLog(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortData *portp = (PortData *)data;	// data points to PortData

	int i;
	for (i = 0; i < STL_NUM_LINKDOWN_REASONS; ++i) {
		STL_LINKDOWN_REASON *ldr = &portp->LinkDownReasons[i];

		if (ldr->Timestamp) {
			IXmlOutputStartAttrTag(state, tag, &i, LDRLogEntryXmlFormatAttr);

			IXmlOutputStrUint(state, "LinkDownReason",
				StlLinkDownReasonToText(ldr->LinkDownReason), ldr->LinkDownReason);
			IXmlOutputStrUint(state, "NeighborLinkDownReason",
				StlLinkDownReasonToText(ldr->NeighborLinkDownReason), ldr->NeighborLinkDownReason);

			IXmlOutputUint64(state, "Timestamp", ldr->Timestamp);

			IXmlOutputEndTag(state, tag);
		}
	}
}

static void *LDRLogXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortData *pdata = (PortData *)parent;
	int idx = -1;

	if (attr == NULL) {
		IXmlParserPrintError(state, "Failed to parse idx Attribute");
		return NULL;
	}

	int i = 0;
	while (attr[i]) {
		if (!strcmp("idx", attr[i]) && attr[i+1] != NULL) {
			if (attr[i + 1][1] == '\0') {
				idx = attr[i + 1][0] - '0';
				if (idx >= 0 && idx < STL_NUM_LINKDOWN_REASONS) {
					return &pdata->LinkDownReasons[idx];
				}
			}
		}
		i++;
	}

	IXmlParserPrintError(state, "Failed to parse idx Attribute: %d", idx);
	return NULL;
}
static void LDRLogXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	return;
}
/** =========================================================================
 * PortData definitions
 */
static IXML_FIELD PortDataFields[] = {
	{ tag:"PortNum", format:'U', IXML_FIELD_INFO(PortData, PortNum) },
	{ tag:"GUID", format:'H', IXML_FIELD_INFO(PortData, PortGUID) },
	{ tag:"EndPortLID", format:'K', format_func:PortDataXmlOutputEndPortLID, end_func:PortDataXmlParserEndEndPortLID }, // bitfield
	{ tag:"M_Key", format:'H', IXML_FIELD_INFO(PortData, PortInfo.M_Key) },
	{ tag:"SubnetPrefix", format:'H', IXML_FIELD_INFO(PortData, PortInfo.SubnetPrefix) },
	{ tag:"LID", format:'H', IXML_FIELD_INFO(PortData, PortInfo.LID) },
	{ tag:"SMLID", format:'H', IXML_FIELD_INFO(PortData, PortInfo.MasterSMLID) },
	{ tag:"IPAddrIPV6", format:'k', format_func:IXmlOutputIPAddrIPV6, IXML_FIELD_INFO(PortData, PortInfo.IPAddrIPV6.addr), end_func:IXmlParserEndIPAddrIPV6},
	{ tag:"IPAddrIPV4", format:'k', format_func:IXmlOutputIPAddrIPV4, IXML_FIELD_INFO(PortData, PortInfo.IPAddrIPV4.addr), end_func:IXmlParserEndIPAddrIPV4},
	{ tag:"CapabilityMask", format:'H', IXML_FIELD_INFO(PortData, PortInfo.CapabilityMask.AsReg32) },
	{ tag:"CapabilityMask3", format:'H', IXML_FIELD_INFO(PortData, PortInfo.CapabilityMask3.AsReg16) },
	{ tag:"DiagCode", format:'H', IXML_FIELD_INFO(PortData, PortInfo.DiagCode.AsReg16) },
	{ tag:"Lease", format:'U', IXML_FIELD_INFO(PortData, PortInfo.M_KeyLeasePeriod) },
	{ tag:"PortState", format:'k', format_func: PortDataXmlOutputPortState, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"PortState_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndPortState }, // input only bitfield
	{ tag:"InitReason", format:'k', format_func: PortDataXmlOutputInitReason, end_func:IXmlParserEndNoop },
	{ tag:"InitReason_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndInitReason },
	{ tag:"PhysState", format:'k', format_func: PortDataXmlOutputPhysState, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"PhysState_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndPhysState }, // input only bitfield
	{ tag:"PortType", format:'k', format_func:PortDataXmlOutputPortPhysConfig, end_func:IXmlParserEndNoop }, 
	{ tag:"PortType_Int", format:'H', IXML_FIELD_INFO(PortData, PortInfo.PortPhysConfig.AsReg8)},
	{ tag:"LMC", format:'K', format_func:PortDataXmlOutputLMC, end_func:PortDataXmlParserEndLMC }, // bitfield
	{ tag:"Protect", format:'k', format_func: PortDataXmlOutputMKeyProtect, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"Protect_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndMKeyProtect }, // input only bitfield
	{ tag:"LinkWidthEnabled", format:'k', format_func: PortDataXmlOutputLinkWidthEnabled, end_func:IXmlParserEndNoop }, // output only
	{ tag:"LinkWidthEnabled_Int", format:'U', format_func: IXmlOutputNoop, IXML_FIELD_INFO(PortData, PortInfo.LinkWidth.Enabled) }, // input only
	{ tag:"LinkWidthSupported", format:'k', format_func: PortDataXmlOutputLinkWidthSupported, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"LinkWidthSupported_Int", format:'U', format_func: IXmlOutputNoop, IXML_FIELD_INFO(PortData, PortInfo.LinkWidth.Supported) }, // input only
	{ tag:"LinkWidthActive", format:'k', format_func: PortDataXmlOutputLinkWidthActive, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"LinkWidthActive_Int", format:'U', format_func: IXmlOutputNoop, IXML_FIELD_INFO(PortData, PortInfo.LinkWidth.Active) }, // input only
	{ tag:"LinkWidthDowngradeEnabled", format:'k', format_func: PortDataXmlOutputLinkWidthDowngradeEnabled, end_func:IXmlParserEndNoop }, // output only
	{ tag:"LinkWidthDowngradeEnabled_Int", format:'U', format_func: IXmlOutputNoop, IXML_FIELD_INFO(PortData, PortInfo.LinkWidthDowngrade.Enabled) }, // input only
	{ tag:"LinkWidthDowngradeSupported", format:'k', format_func: PortDataXmlOutputLinkWidthDowngradeSupported, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"LinkWidthDowngradeSupported_Int", format:'U', format_func: IXmlOutputNoop, IXML_FIELD_INFO(PortData, PortInfo.LinkWidthDowngrade.Supported) }, // input only
	{ tag:"LinkWidthDowngradeTxActive", format:'k', format_func: PortDataXmlOutputLinkWidthDowngradeTxActive, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"LinkWidthDowngradeTxActive_Int", format:'U', format_func: IXmlOutputNoop, IXML_FIELD_INFO(PortData, PortInfo.LinkWidthDowngrade.TxActive) }, // input only
	{ tag:"LinkWidthDowngradeRxActive", format:'k', format_func: PortDataXmlOutputLinkWidthDowngradeRxActive, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"LinkWidthDowngradeRxActive_Int", format:'U', format_func: IXmlOutputNoop, IXML_FIELD_INFO(PortData, PortInfo.LinkWidthDowngrade.RxActive) }, // input only

	{ tag:"PortPacketFormatsSupported", format:'u', IXML_FIELD_INFO(PortData, PortInfo.PortPacketFormats.Supported) },
	{ tag:"PortPacketFormatsEnabled", format:'u', IXML_FIELD_INFO(PortData, PortInfo.PortPacketFormats.Enabled) },

	// PortLinkMode is a union of sub-byte fields; if more than one way of inputting PortLinkMode is provided, code should check that only one is used
	{ tag:"PortLinkMode_Int", format:'U', IXML_FIELD_INFO(PortData, PortInfo.PortLinkMode.AsReg16) },
	{ tag:"LinkSpeedEnabled", format:'k', format_func: PortDataXmlOutputLinkSpeedEnabled, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"LinkSpeedEnabled_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndLinkSpeedEnabled }, // input only bitfield
	{ tag:"LinkSpeedSupported", format:'k', format_func: PortDataXmlOutputLinkSpeedSupported, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"LinkSpeedSupported_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndLinkSpeedSupported }, // input only bitfield
	{ tag:"LinkSpeedActive", format:'k', format_func: PortDataXmlOutputLinkSpeedActive, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"LinkSpeedActive_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndLinkSpeedActive }, // input only bitfield
	{ tag:"MTUs", format:'K', format_func:PortDataXmlOutputMTUs, subfields:PortDataMTUActiveXmlFields }, // bitfield(s)
	{ tag:"MTUSupported", format:'K', format_func:PortDataXmlOutputMTUSupported, end_func:PortDataXmlParserEndMTUSupported }, // bitfield
	{ tag:"SMSL", format:'K', format_func:PortDataXmlOutputSMSL, end_func:PortDataXmlParserEndSMSL }, // bitfield
	{ tag:"VLsActive", format:'k', format_func: PortDataXmlOutputVLsActive, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"VLsActive_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndVLsActive }, // input only bitfield
	{ tag:"VLsSupported", format:'k', format_func: PortDataXmlOutputVLsSupported, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"VLsSupported_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndVLsSupported }, // input only bitfield
	{ tag:"VLArbHighLimit", format:'U', IXML_FIELD_INFO(PortData, PortInfo.VL.HighLimit) },
	{ tag:"VLArbHighCap", format:'U', IXML_FIELD_INFO(PortData, PortInfo.VL.ArbitrationHighCap) },
	{ tag:"VLArbLowCap", format:'U', IXML_FIELD_INFO(PortData, PortInfo.VL.ArbitrationLowCap) },
	{ tag:"VLPreemptingLimit", format:'u', IXML_FIELD_INFO(PortData, PortInfo.VL.PreemptingLimit) },
	{ tag:"VLPreemptCap", format:'u', IXML_FIELD_INFO(PortData, PortInfo.VL.PreemptCap) },
	{ tag:"VLStalls", format:'K', format_func:PortDataXmlOutputVLStalls, subfields:PortDataVLStallXmlFields },
	{ tag:"VLFlowControlDisabledMask", format:'H', IXML_FIELD_INFO(PortData,PortInfo.FlowControlMask) },
	{ tag:"MulticastMask", format:'k', format_func:PortDataXmlOutputMulticastMask, end_func:PortDataXmlParserEndMulticastMask }, // bitfield
	{ tag:"CollectiveMask", format:'k', format_func:PortDataXmlOutputCollectiveMask, end_func:PortDataXmlParserEndCollectiveMask }, // bitfield
	{ tag:"HoQLifes", format:'K', format_func:PortDataXmlOutputHoQLifes, subfields:PortDataHoQLifeXmlFields },
	{ tag:"P_KeyEnforcementInbound", format:'k', format_func: PortDataXmlOutputP_KeyEnforcementInbound, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"P_KeyEnforcementInbound_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndP_KeyEnforcementInbound }, // input only bitfield
	{ tag:"P_KeyEnforcementOutbound", format:'k', format_func: PortDataXmlOutputP_KeyEnforcementOutbound, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"P_KeyEnforcementOutbound_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndP_KeyEnforcementOutbound }, // input only bitfield
	{ tag:"ViolationsM_Key", format:'U', IXML_FIELD_INFO(PortData, PortInfo.Violations.M_Key) },
	{ tag:"ViolationsP_Key", format:'U', IXML_FIELD_INFO(PortData, PortInfo.Violations.P_Key) },
	{ tag:"ViolationsQ_Key", format:'U', IXML_FIELD_INFO(PortData, PortInfo.Violations.Q_Key) },
	{ tag:"BufferUnits", format:'x', IXML_FIELD_INFO(PortData, PortInfo.BufferUnits.AsReg32) }, // bitfield
	{ tag:"OverallBufferSpace", format:'U', IXML_FIELD_INFO(PortData, PortInfo.OverallBufferSpace) },
	{ tag:"ReplayDepthBufferH", format:'u', IXML_FIELD_INFO(PortData, PortInfo.ReplayDepthH.BufferDepthH) },
	{ tag:"ReplayDepthWireH", format:'u', IXML_FIELD_INFO(PortData, PortInfo.ReplayDepthH.WireDepthH) },
	{ tag:"ReplayDepthBuffer", format:'U', IXML_FIELD_INFO(PortData, PortInfo.ReplayDepth.BufferDepth) },
	{ tag:"ReplayDepthWire", format:'U', IXML_FIELD_INFO(PortData, PortInfo.ReplayDepth.WireDepth) },
	{ tag:"SubnetTimeout", format:'k', format_func: PortDataXmlOutputSubnetTimeout, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"SubnetTimeout_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndSubnetTimeout }, // input only bitfield
	{ tag:"RespTimeout", format:'k', format_func: PortDataXmlOutputRespTimeout, end_func:IXmlParserEndNoop }, // output only bitfield
	{ tag:"RespTimeout_Int", format:'K', format_func: IXmlOutputNoop, end_func:PortDataXmlParserEndRespTimeout }, // input only bitfield
	{ tag:"SLtoSCMap", format:'k', format_func:PortDataXmlOutputSLtoSCMap, subfields:SLtoSCMapSCFields, start_func:SLtoSCMapXmlParserStart, end_func:SLtoSCMapXmlParserEnd }, // structure
	{ tag:"SCtoSLMap", format:'k', format_func:PortDataXmlOutputSCtoSLMap, subfields:SCtoSLMapSLFields, start_func:SCtoSLMapXmlParserStart, end_func:SCtoSLMapXmlParserEnd }, // structure
	{ tag:"SCtoSCMap", format:'k', format_func:PortDataXmlOutputSCtoSCMap, subfields:SCtoSCMapOutPortFields, start_func:SCtoSCMapXmlParserStart, end_func:SCtoSCMapXmlParserEnd }, // structure
	{ tag:"SCtoVLrMap", format:'k', format_func:PortDataXmlOutputSCtoVLrMap, subfields:SCtoVLrMapVLrFields, start_func:SCtoVLrMapXmlParserStart, end_func:SCtoVLxMapXmlParserEnd }, // structure
	{ tag:"SCtoVLtMap", format:'k', format_func:PortDataXmlOutputSCtoVLtMap, subfields:SCtoVLtMapVLtFields, start_func:SCtoVLtMapXmlParserStart, end_func:SCtoVLxMapXmlParserEnd }, // structure
	{ tag:"SCtoVLntMap", format:'k', format_func:PortDataXmlOutputSCtoVLntMap, subfields:SCtoVLntMapVLntFields, start_func:SCtoVLntMapXmlParserStart, end_func:SCtoVLxMapXmlParserEnd }, // structure
	{ tag:"BufferControlTable", format:'k', format_func:PortDataXmlOutputBCT, subfields:BufferControlTableFields, start_func:BCTXmlParserStart }, // structure
	{ tag:"VLArbitrationLow", format:'k', format_func:PortDataXmlOutputVLArbLow, subfields:VLArbFields, start_func:VLArbLowXmlParserStart, end_func:VLArbLowXmlParserEnd }, // structure
	{ tag:"VLArbitrationHigh", format:'k', format_func:PortDataXmlOutputVLArbHigh, subfields:VLArbFields, start_func:VLArbHighXmlParserStart, end_func:VLArbHighXmlParserEnd }, // structure
	{ tag:"VLArbitrationPreemptElements", format:'k', format_func:PortDataXmlOutputVLArbPreemptElements, subfields:VLArbFields, start_func:VLArbPreemptElementsXmlParserStart, end_func:VLArbPreemptElementsXmlParserEnd },
	{ tag:"VLArbitrationPreemptMatrix", format:'k', format_func:PortDataXmlOutputVLArbPreemptMatrix, subfields:VLArbPreemptMatrixFields, start_func:VLArbPreemptMatrixXmlParserStart, end_func:VLArbPreemptMatrixXmlParserEnd },
	{ tag:"PKeyTable", format:'k', format_func:PortDataXmlOutputPKeyTable, subfields:PKeyTableFields, start_func:PKeyTableXmlParserStart, end_func:PKeyTableXmlParserEnd }, // structure
	{ tag:"PortStatus", format:'k', size:sizeof(STL_PORT_COUNTERS_DATA), format_func:PortDataXmlOutputPortStatusData, subfields:PortStatusDataFields, start_func:IXmlParserStartStruct, end_func:PortStatusDataXmlParserEnd }, // structure
	{ tag:"CableInfo", format:'k', size:128, format_func:PortDataXmlOutputCableInfo, subfields:(IXML_FIELD*)CableInfoFields, start_func:CableInfoXmlParserStart}, 
	{ tag:"LocalPortNum", format:'u', IXML_FIELD_INFO(PortData, PortInfo.LocalPortNum) },
	{ tag:"PortStates", format:'h', IXML_FIELD_INFO(PortData, PortInfo.PortStates.AsReg32) },
	{ tag:"SMTrapQP", format:'h', IXML_FIELD_INFO(PortData, PortInfo.SM_TrapQP.AsReg32) },
	{ tag:"SAQP", format:'h', IXML_FIELD_INFO(PortData, PortInfo.SA_QP.AsReg32) },
	{ tag:"PortNeighborMode", format:'h', IXML_FIELD_INFO(PortData, PortInfo.PortNeighborMode) },
	{ tag:"PortMode", format:'h', IXML_FIELD_INFO(PortData, PortInfo.PortMode.AsReg16) },
	{ tag:"PortErrorAction", format:'h', IXML_FIELD_INFO(PortData, PortInfo.PortErrorAction.AsReg32) },
	{ tag:"FlitControl", format:'k', format_func:PortDataXmlOutputFlitControl, subfields:PortDataFlitControlXmlFields, start_func:PortDataXmlParserStartFlitControl, end_func:IXmlParserEndNoop },
	{ tag:"NeighborNodeGUID", format:'h', IXML_FIELD_INFO(PortData, PortInfo.NeighborNodeGUID) },
	{ tag:"NeighborMTU", format:'k', format_func:PortDataXmlOutputNeighborMTU, start_func:PortDataXmlParserStartNeighborMtu, end_func:PortDataXmlParserEndNeighborMtu },
	{ tag:"PKey_8B", format:'h', IXML_FIELD_INFO(PortData, PortInfo.P_Keys.P_Key_8B) },
	{ tag:"PKey_10B", format:'h', IXML_FIELD_INFO(PortData, PortInfo.P_Keys.P_Key_10B) },
	{ tag:"PortLTPCRCMode_Int", format:'u', IXML_FIELD_INFO(PortData, PortInfo.PortLTPCRCMode.AsReg16) },
	{ tag:"VLArbCap_Int", format:'u', IXML_FIELD_INFO(PortData, PortInfo.VL.ArbitrationHighCap) },
	{ tag:"NeighborPortNum", format:'u', IXML_FIELD_INFO(PortData, PortInfo.NeighborPortNum) },
	{ tag:"LinkDownReason_Int", format:'u', IXML_FIELD_INFO(PortData, PortInfo.LinkDownReason) },
	{ tag:"NeighborLinkDownReason_Int", format:'u', IXML_FIELD_INFO(PortData, PortInfo.NeighborLinkDownReason) },
	{ tag:"PassThroughControlEgressPort", format:'u', IXML_FIELD_INFO(PortData, PortInfo.PassThroughControl.EgressPort) },
	{ tag:"SubnetClientReregister_Int", format:'k', format_func:PortDataXmlOutputSubnetClientReregister, end_func:PortDataXmlParserEndSubnetClientReregister },
	{ tag:"SubnetMulticastPKeyTrapSuppressionEnabled_Int", format:'u', format_func:PortDataXmlOutputSubnetMulticastPKeyTrapSuppressionEnabled, end_func:PortDataXmlParserEndSubnetMulticastPKeyTrapSuppressionEnabled },
	{ tag:"PassThroughControlDRControl_Int", format:'k', format_func:PortDataXmlOutputPassThroughControlDRControl, end_func:PortDataXmlParserEndPassThroughControlDRControl },
	{ tag:"HFICongestionControlTableIndexLimit", format:'h', IXML_FIELD_INFO(PortData, CCTI_Limit) },
	{ tag:"HFICongestionControlTable", format:'k', format_func:PortDataXmlOutputHFICCT, subfields:(IXML_FIELD*)HFICCTFields, start_func:HFICCTXmlParserStart},
	{ tag:"BundleNextPort", format:'u', IXML_FIELD_INFO(PortData, PortInfo.BundleNextPort) },
	{ tag:"BundleLane", format:'u', IXML_FIELD_INFO(PortData, PortInfo.BundleLane) },
	{ tag:"LinkDownReasonLog", format:'k', format_func:PortDataXmlOutputLDRLog, subfields:LDRLogFields, start_func:LDRLogXmlParserStart, end_func:LDRLogXmlParserEnd }, // structure
	{ NULL }
};

static void PortDataXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (PortData*)data, PortDataXmlFormatAttr, PortDataFields);
}

static void *PortDataXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	PortData *portp = (PortData*)MemoryAllocate2AndClear(sizeof(PortData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);

	if (! portp) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	ListItemInitState(&portp->AllPortsEntry);
	QListSetObj(&portp->AllPortsEntry, portp);
	portp->nodep = (NodeData *)parent;
	//printf("portp=%p, nodep=%p\n", portp, portp->nodep);

	return portp;
}

static int Snapshot_PortDataComplete(IXmlParserState_t * state, void * object, void * parent);
static ParseCompleteFn portDataCompleteFn = Snapshot_PortDataComplete;

void SetPortDataComplete(ParseCompleteFn fn)
{
	portDataCompleteFn = fn;
}

static ParseCompleteFn GetPortDataComplete()
{
	return portDataCompleteFn;
}

/**
	"Destructors" for some topology structs.

	They do not free the targets, only their members.
*/
void Snapshot_PortDataFree(PortData * portp, FabricData_t * fabricp);
void Snapshot_NodeDataFree(NodeData * nodep, FabricData_t * fabricp);

/**
	Completion handler for PortData inside a snapshot.
*/
static int Snapshot_PortDataComplete(IXmlParserState_t * state, void * object, void * parent)
{
	const IXML_FIELD *p;
	unsigned int i;
	PortData *portp = (PortData*)object;
	NodeData *nodep = portp->nodep;
	FabricData_t *fabricp = IXmlParserGetContext(state);

	ASSERT(nodep == (NodeData*)parent);

	// technically <Port> could preceed <NodeType> within <Node>
	if (nodep->NodeInfo.NodeType == STL_NODE_SW) {
		// a switch only gets 1 port Guid, we save it for switch
		// port 0 (the "virtual management port")
		// should not have PortGUID if PortNum != 0
		if (portp->PortGUID && portp->PortNum != 0) {
			IXmlParserPrintError(state, "Invalid fields in Port: PortGUID not allowed for Switch port!=0");
			return FERROR;
		}
	} else {
		portp->PortInfo.LocalPortNum = portp->PortNum ;
		// should have PortGUID
		if (portp->PortGUID == 0 || portp->PortNum == 0) {
			IXmlParserPrintError(state, "Invalid/missing fields in Port, PortGUID required for non-Switch");
			return FERROR;
		}
	}

	if (cl_qmap_insert(&nodep->Ports, portp->PortNum, &portp->NodePortsEntry) != &portp->NodePortsEntry)
	{
		IXmlParserPrintError(state, "Duplicate PortNum: %u", portp->PortNum);
		goto failinsert2;
	}
	if (FSUCCESS != AllLidsAdd(fabricp, portp, FALSE))
	{
		IXmlParserPrintError(state, "Duplicate LIDs found in portRecords: LID 0x%x Port %u Node: %.*s\n",
					portp->EndPortLID,
					portp->PortNum, STL_NODE_DESCRIPTION_ARRAY_SIZE,
					(char*)nodep->NodeDesc.NodeString);
		goto failinsert2;
	}

	// Handling to deal with fields not defined in STL Gen 1, but required in Gen 2. Allows forward compatability
	// of snapshots.
	for (p=state->current.subfields,i=0; p->tag != NULL; ++i,++p) {
		if (strcmp(p->tag, "PortPacketFormatsSupported") == 0) {
			if (! (state->current.fields_found & ((uint64_t)1)<<i) ) {
				portp->PortInfo.PortPacketFormats.Supported = STL_PORT_PACKET_FORMAT_9B;
				portp->PortInfo.PortPacketFormats.Enabled = STL_PORT_PACKET_FORMAT_9B;
			}
		} else if (strcmp(p->tag, "MaxLID") == 0) {
			if (! (state->current.fields_found & ((uint64_t)1)<<i) )
				if (portp->PortInfo.CapabilityMask3.s.IsMAXLIDSupported) {
					IXmlParserPrintError(state, "Missing fields in Port, MaxLID required for Gen2 HFI");
					return FERROR;
				}
		}
	}

	return FSUCCESS;

failinsert2:
	cl_qmap_remove_item(&nodep->Ports, &portp->NodePortsEntry);
	return FERROR;
}

static void PortDataXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	PortData *portp = (PortData*)object;
	FabricData_t *fabricp = IXmlParserGetContext(state);

	ParseCompleteFn parseCompleteFn = GetPortDataComplete();

	if (! valid)	// missing mandatory fields
		goto failvalidate;

	portp->rate = StlLinkSpeedWidthToStaticRate(
			portp->PortInfo.LinkSpeed.Active,
			portp->PortInfo.LinkWidth.Active);

	if (portp->pPortCounters) {
		portp->pPortCounters->lq.s.numLanesDown = StlGetNumLanesDown(&portp->PortInfo);
	}

	if (parseCompleteFn) {
		if (parseCompleteFn(state, object, parent) != FSUCCESS) {
			goto failvalidate;
		}
	}

	QListInsertTail(&fabricp->AllPorts, &portp->AllPortsEntry);

	return;

failvalidate:
	Snapshot_PortDataFree(portp, fabricp);
	MemoryDeallocate(portp);
}

/**
	Destructor for @c PortData inside parser.

	Frees members but not port data itself.
*/
void Snapshot_PortDataFree(PortData * portp, FabricData_t * fabricp)
{
	if (portp->pPortCounters)
		MemoryDeallocate(portp->pPortCounters);
	PortDataFreeQOSData(fabricp, portp);
	PortDataFreePartitionTable(fabricp, portp);
}

#if !defined(VXWORKS) || defined(BUILD_DMC)
/****************************************************************************/
/* IocService Input/Output functions */

/* most are in ixml_ib */

// a better approach would be to allocate a "super structure which includes
// IocData and an extra field after it to count service entries as they
// arrive
static unsigned g_service_index = 0;	// this is a hack, not Thread safe

static void *IocServiceXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	IocData *iocp = (IocData*)parent;

	if (! iocp->Services) {
		IXmlParserPrintError(state, "Services must follow ServicesCount");
		return NULL;
	}
	if (g_service_index >= iocp->IocProfile.ServiceEntries) {
		IXmlParserPrintError(state, "Number of Services inconsistent with ServicesCount");
		return NULL;
	}
	return (&iocp->Services[g_service_index]);
}

static void IocServiceXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	if (! valid)	// missing mandatory fields
		goto failvalidate;
	g_service_index++;
	return;

failvalidate:
	/* nothing to free, Iocp will take care of Services array */
	return;
}

/****************************************************************************/
/* IocData Input/Output functions */

static void IocDataXmlFormatAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " id=\"0x%016"PRIx64"\"", ((IocData*)data)->IocProfile.IocGUID);
}

/* bitfields needs special handling: VendorId */
static void IocDataXmlOutputVendorId(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputHex(state, tag, ((IocData *)data)->IocProfile.ven.v.VendorId);
}

static void IocDataXmlParserEndVendorId(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 value;
	
	if (IXmlParseUint32(state, content, len, &value))
		((IocData *)object)->IocProfile.ven.v.VendorId = value;
}

/* bitfields needs special handling: SubSystemVendorId */
static void IocDataXmlOutputSubSystemVendorId(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputHex(state, tag, ((IocData *)data)->IocProfile.sub.s.SubSystemVendorID);
}

static void IocDataXmlParserEndSubSystemVendorId(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 value;
	
	if (IXmlParseUint32(state, content, len, &value))
		((IocData *)object)->IocProfile.sub.s.SubSystemVendorID = value;
}

/* this is a cheat, we need to allocate IocData.Services array.
 * we depend on ServicesCount preceeding 1st Service entry in XML input
 */
static void IocDataXmlParserEndServicesCount(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	IocData * iocp = (IocData *)object;
	
	if (IXmlParseUint8(state, content, len, &iocp->IocProfile.ServiceEntries)) {
		iocp->Services = (IOC_SERVICE*)MemoryAllocate2AndClear(sizeof(IOC_SERVICE)*iocp->IocProfile.ServiceEntries, IBA_MEM_FLAG_PREMPTABLE, MYTAG);
		if (! iocp->Services) {
			IXmlParserPrintError(state, "Unable to allocate memory");
		}
	}
}

static void IocDataXmlOutputServices(IXmlOutputState_t *state, const char *tag, void *data)
{
	IocData *iocp = (IocData*)data;
	unsigned i;

	IXmlOutputStartAttrTag(state, "Services", NULL, NULL);
	for (i=0; i < iocp->IocProfile.ServiceEntries; i++)
		IocServiceXmlOutput(state, "Service", &iocp->Services[i]);
	IXmlOutputEndTag(state, "Services");
}

static IXML_FIELD ServicesFields[] = {
	{ tag:"Service", format:'k', subfields:IocServiceFields, start_func:IocServiceXmlParserStart, end_func:IocServiceXmlParserEnd }, // structure, only used on input
	{ NULL }
};

static IXML_FIELD IocDataFields[] = {
	{ tag:"IocSlot", format:'U', IXML_FIELD_INFO(IocData, IocSlot) },
	{ tag:"IocGUID", format:'H', IXML_FIELD_INFO(IocData, IocProfile.IocGUID) },
	{ tag:"VendorId", format:'K', format_func:IocDataXmlOutputVendorId, end_func:IocDataXmlParserEndVendorId }, // bitfield
	{ tag:"DeviceId", format:'X', IXML_FIELD_INFO(IocData, IocProfile.DeviceId) },
	{ tag:"DeviceVersion", format:'X', IXML_FIELD_INFO(IocData, IocProfile.DeviceVersion) },
	{ tag:"SubSystemVendorID", format:'K', format_func:IocDataXmlOutputSubSystemVendorId, end_func:IocDataXmlParserEndSubSystemVendorId }, // bitfield
	{ tag:"SubSystemID", format:'X', IXML_FIELD_INFO(IocData, IocProfile.SubSystemID) },
	{ tag:"IOClass", format:'X', IXML_FIELD_INFO(IocData, IocProfile.IOClass) },
	{ tag:"IOSubClass", format:'X', IXML_FIELD_INFO(IocData, IocProfile.IOSubClass) },
	{ tag:"Protocol", format:'X', IXML_FIELD_INFO(IocData, IocProfile.Protocol) },
	{ tag:"ProtocolVer", format:'X', IXML_FIELD_INFO(IocData, IocProfile.ProtocolVer) },
	{ tag:"SendMsgDepth", format:'U', IXML_FIELD_INFO(IocData, IocProfile.SendMsgDepth) },
	{ tag:"SendMsgSize", format:'U', IXML_FIELD_INFO(IocData, IocProfile.SendMsgSize) },
	{ tag:"RDMAReadDepth", format:'U', IXML_FIELD_INFO(IocData, IocProfile.RDMAReadDepth) },
	{ tag:"RDMASize", format:'U', IXML_FIELD_INFO(IocData, IocProfile.RDMASize) },
	{ tag:"CapabilityMask", format:'X', IXML_FIELD_INFO(IocData, IocProfile.ccm.CntlCapMask) },
	{ tag:"ServicesCount", format:'U', IXML_FIELD_INFO(IocData, IocProfile.ServiceEntries), end_func:IocDataXmlParserEndServicesCount },
	{ tag:"IDString", format:'C', IXML_FIELD_INFO(IocData, IocProfile.IDString) },
	{ tag:"Services", format:'k', format_func:IocDataXmlOutputServices, subfields:ServicesFields }, // list
	{ NULL }
};

static void IocDataXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (IocData*)data, IocDataXmlFormatAttr, IocDataFields);
}

static void *IocDataXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	IocData *iocp = (IocData*)MemoryAllocate2AndClear(sizeof(IocData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);

	if (! iocp) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}
	iocp->ioup = (IouData *)parent;
	ListItemInitState(&iocp->IouIocsEntry);
	QListSetObj(&iocp->IouIocsEntry, iocp);

	g_service_index = 0;
	return iocp;
}

static void IocDataXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	IocData *iocp = (IocData*)object;
	IouData *ioup = iocp->ioup;
	FabricData_t *fabricp = IXmlParserGetContext(state);
	
	ASSERT(ioup == (IouData*)parent);

	if (! valid)	// missing mandatory fields
		goto failvalidate;

	if (cl_qmap_insert(&fabricp->AllIOCs, (uint64_t)iocp, &iocp->AllIOCsEntry) != &iocp->AllIOCsEntry)
	{
		IXmlParserPrintError(state, "Duplicate IOC Guids found: 0x%016"PRIx64, iocp->IocProfile.IocGUID);
		goto failinsert;
	}
	QListInsertTail(&ioup->Iocs, &iocp->IouIocsEntry);
	return;

failinsert:
failvalidate:
	if (iocp->Services)
		MemoryDeallocate(iocp->Services);
	MemoryDeallocate(iocp);
}

/****************************************************************************/
/* IouData Input/Output functions */

static void IouDataXmlFormatAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " id=\"0x%016"PRIx64"\"", ((IouData*)data)->nodep->NodeInfo.NodeGUID);
}

/* bitfields needs special handling: DiagDeviceId */
static void IouDataXmlOutputDiagDeviceId(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag, ((IouData *)data)->IouInfo.DiagDeviceId);
}

static void IouDataXmlParserEndDiagDeviceId(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((IouData *)object)->IouInfo.DiagDeviceId = value;
}

/* bitfields needs special handling: OptionRom */
static void IouDataXmlOutputOptionRom(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag, ((IouData *)data)->IouInfo.OptionRom);
}

static void IouDataXmlParserEndOptionRom(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((IouData *)object)->IouInfo.OptionRom = value;
}

static void IouDataXmlOutputIocs(IXmlOutputState_t *state, const char *tag, void *data)
{
	IouData *ioup = (IouData*)data;
	LIST_ITEM *p;

	for (p=QListHead(&ioup->Iocs); p != NULL; p = QListNext(&ioup->Iocs, p)) {
		IocDataXmlOutput(state, "Ioc", QListObj(p));
	}
}

static IXML_FIELD IouDataFields[] = {
	{ tag:"ChangeID", format:'U', IXML_FIELD_INFO(IouData, IouInfo.Change_ID) },
	{ tag:"MaxControllers", format:'U', IXML_FIELD_INFO(IouData, IouInfo.MaxControllers) },
	{ tag:"DiagDeviceId", format:'K', format_func:IouDataXmlOutputDiagDeviceId, end_func:IouDataXmlParserEndDiagDeviceId }, // bitfield
	{ tag:"OptionRom", format:'K', format_func:IouDataXmlOutputOptionRom, end_func:IouDataXmlParserEndOptionRom }, // bitfield
	{ tag:"Ioc", format:'k', format_func:IouDataXmlOutputIocs, subfields:IocDataFields, start_func:IocDataXmlParserStart, end_func:IocDataXmlParserEnd }, // structures
	{ NULL }
};

static void IouDataXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (IouData*)data, IouDataXmlFormatAttr, IouDataFields);
}

static void *IouDataXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	IouData *ioup = (IouData*)MemoryAllocate2AndClear(sizeof(IouData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);

	if (! ioup) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}
	ListItemInitState(&ioup->AllIOUsEntry);
	QListSetObj(&ioup->AllIOUsEntry, ioup);
	ioup->nodep = (NodeData *)parent;
	QListInitState(&ioup->Iocs);
	if (! QListInit(&ioup->Iocs))
	{
		MemoryDeallocate(ioup);
		IXmlParserPrintError(state, "Unable to initialize object");
		return NULL;
	}

	return ioup;
}

static void IouDataXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	IouData *ioup = (IouData*)object;
	NodeData *nodep = ioup->nodep;
	FabricData_t *fabricp = IXmlParserGetContext(state);
	
	ASSERT(nodep == (NodeData*)parent);

	if (! valid)	// missing mandatory fields
		goto failvalidate;

	if (nodep->ioup) {
		IXmlParserPrintError(state, "More than 1 IOU for Node");
		goto failinsert;
	}
	nodep->ioup = ioup;
	return;

failinsert:
failvalidate:
	IouDataFreeIocs(fabricp, ioup);
	MemoryDeallocate(ioup);
}
#endif

/****************************************************************************/
/* SwitchInfo Input/Output functions */

/* most are in ixml_ib */

static void *SwitchInfoXmlParserStart(IXmlParserState_t *state,
	void *parent, const char **attr)
{
	void *p = IXmlParserStartStruct(state, parent, attr);

	if (!p) return p;

	STL_SWITCHINFO_RECORD *pSwitchInfo = (STL_SWITCHINFO_RECORD*)p;
	// For compatibility with snapshots from older versions of OPA,
	// provide default values for optional fields.
	pSwitchInfo->SwitchInfoData.PortGroupFDBCap = DEFAULT_MAX_PGFT_LID + 1;
	return p;
}

static void SwitchInfoXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	STL_SWITCHINFO_RECORD *pSwitchInfo = (STL_SWITCHINFO_RECORD*)object;
	NodeData *nodep = (NodeData*)parent;
	
	if (! valid)	// missing mandatory fields
		goto failvalidate;

	if (nodep->pSwitchInfo) {
		IXmlParserPrintError(state, "More than 1 SwitchInfo for Node");
		goto failinsert;
	}
	nodep->pSwitchInfo = pSwitchInfo;
	return;

failinsert:
failvalidate:
	MemoryDeallocate(pSwitchInfo);
}

/****************************************************************************/
/* SwitchData Input/Output functions */

boolean fbFDB = FALSE;			// Flag for LinearFDB or MulticastFDB parsed
static uint32 lidFDB = 0xFFFFFFFF;	// LID for Linear and PortGroup FDB entries
static uint32 portPGT = 0xFFFFFFFF; // PortGroupNumber for PortGroup Element entry

// Multicast FDB Value parsing context data
static struct McvParseCtx {
	uint32 lid;
	uint8 pos;
} mcvParseCtx;

static void *SwitchDataXmlParserStart(IXmlParserState_t *state,
									void *parent, const char **attr)
{
	void *p = IXmlParserStartStruct(state, parent, attr);
	if (!p) return p;

	SwitchData *switchp = (SwitchData*)p;

	// For compatibility with snapshots from older versions of OPA,
	// provide default values for optional fields.
	switchp->PortGroupFDBSize = DEFAULT_MAX_PGFT_LID + 1;

	return p;
}

static void SwitchDataXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SwitchData *switchp = (SwitchData*)object;
	NodeData *nodep = (NodeData*)parent;
	
	if (! valid)	// missing mandatory fields
		goto failvalidate;

	if (nodep->switchp) {
		IXmlParserPrintError(state, "More than 1 SwitchData for Node");
		goto failinsert;
	}

	nodep->switchp = switchp;


	return;

failinsert:
failvalidate:
	MemoryDeallocate(switchp);
}

static void SwitchDataOutputLIDAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " LID=\"0x%x\"", *(unsigned int *)data);
}

static void SwitchDataOutputPortGroupNumberAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " PortGroupNumber=\"%d\"", *(unsigned int *)data);
}

static void *SwitchDataXmlParserStartLinearValue(IXmlParserState_t *state, void *parent, const char **attr)
{
	SwitchData *switchp = (SwitchData *)parent;	// parent points to SwitchData

	// Save Value attribute LID value for use in SwitchDataXmlParserEndLinearValue()
	if ( !attr || !attr[0] || 0 != strcmp(attr[0], "LID")) {
		IXmlParserPrintError(state, "Missing LID attribute for LinearFDB.Value");
		return NULL;
	}
	if ( FSUCCESS != StringToUint32(&lidFDB, attr[1], NULL, 0, TRUE)
			|| lidFDB >= switchp->LinearFDBSize ) {
		IXmlParserPrintError(state, "Invalid LID attribute for LinearFDB.Value  LID: %s", attr[1]);
		return NULL;
	}
	return ((void *)&lidFDB);
}

static void SwitchDataXmlParserEndLinearValue(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 *pLID = (uint32 *)object;	// object points to LID
	SwitchData *switchp = (SwitchData *)parent;	// parent points to SwitchData
	uint8 temp;

	if (! valid)
		return;
	// Assign LinearFDB[LID] = port
	if ( !switchp || !switchp->LinearFDB || !pLID || !content ||
			!len || (*pLID >= switchp->LinearFDBSize) ) {
		IXmlParserPrintError(state, "LinearFDB improperly allocated");
		return;
	}

	if (FSUCCESS != StringToUint8(&temp, content, NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid Port number in LinearFDB.Value for LID %u  Value: %s", *pLID, content);
		return;
	}
	STL_LFT_PORT_BLOCK(switchp->LinearFDB, *pLID) = (PORT)temp;
}

IXML_FIELD LinearFDBFields[] = {
	{ tag:"Value", start_func:SwitchDataXmlParserStartLinearValue, end_func:SwitchDataXmlParserEndLinearValue },
	{ NULL }
};

/**
	arrays need special handling: LinearFDB
*/
static void SwitchDataOutputLinearFDB(IXmlOutputState_t *state, const char *tag, void *data)
{
	unsigned int ix_lid;
	SwitchData *switchp = (SwitchData *)data;
	PORT *pPort = (PORT *)switchp->LinearFDB;

	IXmlOutputStartTag(state, tag);

	for ( ix_lid = 0; ix_lid < switchp->LinearFDBSize;
			pPort++, ix_lid++ ) {
		if (*pPort == 0xFF)
			continue;
		IXmlOutputStartAttrTag(state, "Value", &ix_lid, SwitchDataOutputLIDAttr);
		IXmlOutputPrint(state, "%u", *pPort);
		IXmlOutputEndTag(state, "Value");
	}

	IXmlOutputEndTag(state, tag);
}

static void *SwitchDataXmlParserStartLinearFDB(IXmlParserState_t *state, void *parent, const char **attr)
{
	SwitchData *switchp = (SwitchData *)parent;	// parent points to SwitchData
	STL_LINEAR_FORWARDING_TABLE *pPort;

	// Allocate LinearFDB
	if (!switchp || switchp->LinearFDB) {
		IXmlParserPrintError(state, "SwitchData improperly allocated");
		return NULL;
	}
	if ( !( pPort = (STL_LINEAR_FORWARDING_TABLE *)MemoryAllocate2AndClear(     //Make sure to allocate a full block of LftBLOCK not just PORTs
			ROUNDUP(switchp->LinearFDBSize, MAX_LFT_ELEMENTS_BLOCK), IBA_MEM_FLAG_PREMPTABLE, MYTAG ) ) ) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	memset(pPort, 255, ROUNDUP(switchp->LinearFDBSize, MAX_LFT_ELEMENTS_BLOCK) ); //Make sure to memset a full block not just whats used
	switchp->LinearFDB = pPort;
	fbFDB = TRUE;
	return (switchp);
}

static void SwitchDataXmlParserEndLinearFDB(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SwitchData *switchp = (SwitchData *)object;
	
	if (! valid)
		goto fail;

	return;

fail:
	if (switchp && switchp->LinearFDB) {
		MemoryDeallocate(switchp->LinearFDB);
		switchp->LinearFDB = NULL;
	}
}


static void *SwitchDataXmlParserStartMulticastValue(IXmlParserState_t *state, void *parent, const char **attrs)
{
	SwitchData *switchp = (SwitchData *)parent;	// parent points to SwitchData
	int i;
	int lidParsed = 0;
	memset(&mcvParseCtx, 0, sizeof(mcvParseCtx));

	for (i = 0; attrs && attrs[i] != NULL;) {
		uint32 temp;
	// Save Value attribute LID value for use in SwitchDataXmlParserEndMulticastValue()
		if (strcmp(attrs[i], "LID") == 0) {
			if (!attrs[i+1]) {
				IXmlParserPrintError(state, "No LID specified for MulticastFDB.Value");
			return NULL;
			}

			if (StringToUint32(&temp, attrs[i+1], NULL, 0, TRUE) != FSUCCESS) {
				IXmlParserPrintError(state, "Invalid LID attribute for MulticastFDB.Value  LID: %s", attrs[i+1]);
				return NULL;
			}

			if (IS_MCAST16(temp))
				temp = MCAST16_TO_MCAST32(temp);

			if ((temp < STL_LID_MULTICAST_BEGIN) || (temp >= switchp->MulticastFDBSize)) {
				IXmlParserPrintError(state, "Invalid LID attribute for MulticastFDB.Value  LID: %s", attrs[i+1]);
				return NULL;
			}

			mcvParseCtx.lid = temp - STL_LID_MULTICAST_BEGIN;
			lidParsed = 1;
			i += 2;
		} else if (strcmp(attrs[i], "pos") == 0) {
			if (!attrs[i+1]) {
				IXmlParserPrintError(state, "No pos specified for MulticastFDB.Value");
				return NULL;
			}

			if (StringToUint32(&temp, attrs[i+1], NULL, 0, TRUE) != FSUCCESS ||
				temp >= STL_NUM_MFT_POSITIONS_MASK) {
				IXmlParserPrintError(state, "Invalid pos attribute for MulticastFDB.Value  pos: %s", attrs[i+1]);
				return NULL;
			}
			mcvParseCtx.pos = (uint8)temp;
			i += 2;
		} else
			i++; // TODO print unrecognized attr warning?
	}

	if (!lidParsed) {
		IXmlParserPrintError(state, "Missing LID attribute for MulticastFDB.Value");
		return NULL;
	}


	return ((void *)&mcvParseCtx);
}

static void SwitchDataXmlParserEndMulticastValue(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	struct McvParseCtx *ctx = (struct McvParseCtx*) object;
	SwitchData *switchp = (SwitchData *)parent;	// parent points to SwitchData
	STL_PORTMASK *pPortMask;
	uint64 portMask;				// Port masks up to 64 bits can be input

	if (! valid)
		return;

	assert(switchp && switchp->MulticastFDB[0]);

	if (!ctx || !content || !len) {
		IXmlParserPrintError(state, "Empty multicast FDB value");
		return;
	}

	if (ctx->lid >= switchp->MulticastFDBSize) {
		IXmlParserPrintError(state, "Multicast LID exceeds multicast FDB range");
		return;
	}

	if (ctx->pos >= STL_NUM_MFT_POSITIONS_MASK) {
		IXmlParserPrintError(state, "Unsupported MulticastFDB pos: %u",
			ctx->pos);
		return;
	}

	pPortMask = switchp->MulticastFDB[ctx->pos] + ctx->lid;

	if (FSUCCESS != StringToUint64(&portMask, content, NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid PortMask in MulticastFDB: %s", content);
		return;
	}

	*pPortMask = (STL_PORTMASK)portMask;
}

IXML_FIELD MulticastFDBFields[] = {
	{ tag:"Value", start_func:SwitchDataXmlParserStartMulticastValue, end_func:SwitchDataXmlParserEndMulticastValue },
	{ NULL }
};

/**
	Arrays need special handling: MulticastFDB.

	@param data @c SwitchData * to node of interest.
	Assumes that ((SwitchData*)data)->parent is not NULL.
*/
static void SwitchDataOutputMulticastFDB(IXmlOutputState_t *state, const char *tag, void *data)
{
	STL_LID mcastLid;
	SwitchData *switchp = (SwitchData *)data;
	uint64 portMask;

	// MulticastFDBTop can be less than STL_LID_MULTICAST_BEGIN at POD.
	// May not be an error but then there aren't any Mcast entries to be
	// output either.
	if ((switchp->MulticastFDBSize <= STL_LID_MULTICAST_BEGIN) || (switchp->MulticastFDBSize > STL_LID_MULTICAST_END + 1)) {
		return;
	}

	IXmlOutputStartTag(state, tag);

	uint32 i;
	for (i = 0; i < STL_NUM_MFT_POSITIONS_MASK;  ++i) {
		uint32 j;
		for (j = 0; j < (switchp->MulticastFDBSize - STL_LID_MULTICAST_BEGIN); ++j) {
			portMask = (uint64)switchp->MulticastFDB[i][j];

			if (!portMask)
				continue;

			mcastLid = STL_LID_MULTICAST_BEGIN + j;

			IXmlOutputStartTag(state, "Value");
			IXmlOutputAttrFmt(state, "pos", "%d", i);
			IXmlOutputAttrFmt(state, "LID", "0x%x", mcastLid);
			IXmlOutputPrint(state, "0x%"PRIx64, portMask);
			IXmlOutputEndTag(state, "Value");
		}
	}
	IXmlOutputEndTag(state, tag);
}

static void *SwitchDataXmlParserStartMulticastFDB(IXmlParserState_t *state, void *parent, const char **attr)
{
	SwitchData *switchp = (SwitchData *)parent;	// parent points to SwitchData
	STL_PORTMASK *pPortMask;
	size_t allocSize;
	uint8_t i;

	// Allocate MulticastFDB
	if (!switchp || switchp->MulticastFDB[0]) {
		IXmlParserPrintError(state, "SwitchData improperly allocated");
		return NULL;
	}
	allocSize = switchp->MulticastFDBSize - STL_LID_MULTICAST_BEGIN;

	if ((switchp->MulticastFDBSize <= STL_LID_MULTICAST_BEGIN) || (switchp->MulticastFDBSize > STL_LID_MULTICAST_END + 1)) {
		IXmlParserPrintError(state, "Invalid MulticastFDBSize, cannot allocate MulticastFDB");
		return NULL;
	}

	for (i = 0; i < STL_NUM_MFT_POSITIONS_MASK; ++i) {
		
		if ( !( pPortMask = (STL_PORTMASK *)MemoryAllocate2AndClear(
				allocSize * sizeof(STL_PORTMASK), IBA_MEM_FLAG_PREMPTABLE,
				MYTAG ) ) ) {
			IXmlParserPrintError(state, "Unable to allocate memory");
			return NULL;
		}

		switchp->MulticastFDB[i] = pPortMask;
	}


	fbFDB = TRUE;
	return (switchp);
}

static void SwitchDataXmlParserEndMulticastFDB(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SwitchData *switchp = (SwitchData *)object;
	
	if (! valid)
		goto fail;

	return;

fail:
	if (switchp) {
		uint8_t i;
		for (i = 0; i < STL_NUM_MFT_POSITIONS_MASK; ++i) {
			if (switchp->MulticastFDB[i]) {
				MemoryDeallocate(switchp->MulticastFDB[i]);
				switchp->MulticastFDB[i] = NULL;
			}
		}
	}
}


static void *SwitchDataXmlParserStartPortGroupElementsValue(IXmlParserState_t *state, void *parent, const char **attr)
{
	SwitchData *switchp = (SwitchData *)parent;
	uint32 temp;

	if (!attr || !attr[0] || 0 != strcmp(attr[0], "PortGroupNumber")) {
		IXmlParserPrintError(state, "Missing PortGroupNumber attribute for PortGroupElements.Value");
		return NULL;
	}

	// we depend on PortGroupSize preceeding this tag in XML
	if (FSUCCESS != StringToUint32(&temp, attr[1], NULL, 0, TRUE)
		|| temp >= switchp->PortGroupSize) {

		IXmlParserPrintError(state, "Invalid PortGroupNumber attribute for PortGroupElements.Value  PortGroupNumber: %s", attr[1]);
		return NULL;
	}
	portPGT = temp;

	return ((void *)&portPGT);
}

static void SwitchDataXmlParserEndPortGroupElementsValue(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 portGroupNum = *(uint32 *)object;
	SwitchData *switchp = (SwitchData *)parent;
	STL_PORTMASK *pPortMask;
	uint64 portMask;

	if (!valid) {
		return;
	}

	assert (switchp && switchp->PortGroupElements);

	if (!content || !len) {
		IXmlParserPrintError(state, "Empty PortGroupElements.Value");
		return;
	}

	pPortMask = switchp->PortGroupElements + portGroupNum;

	if (FSUCCESS != StringToUint64(&portMask, content, NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "invalid PortMask in PortGroupElements.Value for PortGroupNumber %u  Value: %s", portGroupNum, content);
		return;
	}
	*pPortMask = (STL_PORTMASK)portMask;
}

IXML_FIELD PortGroupElementsFields[] = {
	{ tag:"Value", start_func:SwitchDataXmlParserStartPortGroupElementsValue, end_func:SwitchDataXmlParserEndPortGroupElementsValue },
	{ NULL }
};

static void SwitchDataOutputPortGroupElements(IXmlOutputState_t *state, const char *tag, void *data)
{
	uint32 pgPortGroupNum;
	SwitchData *switchp = (SwitchData *)data;
	uint64 portMask;

	IXmlOutputStartTag(state, tag);
	uint32 i;
	for (i = 0; i < switchp->PortGroupSize; i++) {
		portMask = (uint64)switchp->PortGroupElements[i];
		if (!portMask) {
			continue;
		}

		pgPortGroupNum = i;
		IXmlOutputStartAttrTag(state, "Value", &pgPortGroupNum, SwitchDataOutputPortGroupNumberAttr);
		IXmlOutputPrint(state, "0x%"PRIx64, portMask);
		IXmlOutputEndTag(state, "Value");
	}
	IXmlOutputEndTag(state, tag);
}

static void *SwitchDataXmlParserStartPortGroupElements(IXmlParserState_t *state, void *parent, const char **attr)
{
	SwitchData *switchp = (SwitchData *)parent;
	STL_PORTMASK *pPortMask;

	if (!switchp || switchp->PortGroupElements) {
		IXmlParserPrintError(state, "SwitchData improperly allocated");
		return NULL;
	}

	size_t allocSize = switchp->PortGroupSize;

	if (!(pPortMask = (STL_PORTMASK *)MemoryAllocate2AndClear(
		allocSize * sizeof(STL_PORTMASK),
		IBA_MEM_FLAG_PREMPTABLE, MYTAG))) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	switchp->PortGroupElements = pPortMask;

	return (switchp);
}

static void SwitchDataXmlParserEndPortGroupElements(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SwitchData *switchp = (SwitchData *)object;

	if (! valid) {
		goto fail;
	}
	return;

fail:
	if (switchp && switchp->PortGroupElements) {
		MemoryDeallocate(switchp->PortGroupElements);
		switchp->PortGroupElements = NULL;
	}
}
// end of port group

static void *SwitchDataXmlParserStartPortGroupFDBValue(IXmlParserState_t *state, void *parent, const char **attr)
{
	SwitchData *switchp = (SwitchData *)parent;

	if ( !attr || !attr[0] || 0!= strcmp(attr[0], "LID")) {
		IXmlParserPrintError(state, "Missing LID attribute for PortGroupFDB.Value");
		return NULL;
	}

	if (FSUCCESS != StringToUint32(&lidFDB, attr[1], NULL, 0, TRUE)
		|| lidFDB >= switchp->PortGroupFDBSize) {
		IXmlParserPrintError(state, "Invalid LID attribute for PortGroupFDB.Value  LID: %s", attr[1]);
		return NULL;
	}
	return ((void *)&lidFDB);
}

static void SwitchDataXmlParserEndPortGroupFDBValue(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 *pLID = (uint32 *)object; //points to LID
	SwitchData *switchp = (SwitchData *)parent;
	uint8 temp;

	if (!valid) {
		return;
	}
	
	if (!switchp || !switchp->PortGroupFDB || !pLID || !content ||
		!len || (*pLID >= switchp->PortGroupFDBSize)) {
		IXmlParserPrintError(state, "PortGroupFDB improperly allocated");
		return;
	}

	if (FSUCCESS != StringToUint8(&temp, content, NULL, 0, TRUE)) {
		IXmlParserPrintError(state, "Invalid Port Group number in PortGroupFDB.Value for LID %u Value: %s", *pLID, content);
		return;
	}
	STL_PGFT_PORT_BLOCK(switchp->PortGroupFDB, *pLID) = (PORT)temp;
}

IXML_FIELD PortGroupFDBFields[] = {
	{ tag:"Value", start_func:SwitchDataXmlParserStartPortGroupFDBValue, end_func:SwitchDataXmlParserEndPortGroupFDBValue},
	{ NULL }

};

static void SwitchDataOutputPortGroupFDB(IXmlOutputState_t *state, const char *tag, void *data) 
{
	unsigned int ix_lid;
	SwitchData *switchp = (SwitchData *)data;
	PORT *pPortGroup = (PORT *)switchp->PortGroupFDB;

	IXmlOutputStartTag(state, tag);

	for (ix_lid = 0; ix_lid <  switchp->PortGroupFDBSize;
		  pPortGroup++, ix_lid++) {
		if (*pPortGroup == 0xFF) {
			continue;
		}
		IXmlOutputStartAttrTag(state, "Value", &ix_lid, SwitchDataOutputLIDAttr);
		IXmlOutputPrint(state, "%u", *pPortGroup);
		IXmlOutputEndTag(state, "Value");
	}

	IXmlOutputEndTag(state, tag);
}

static void *SwitchDataXmlParserStartPortGroupFDB(IXmlParserState_t *state, void *parent, const char **attr)
{
	SwitchData *switchp = (SwitchData *)parent;
	STL_PORT_GROUP_FORWARDING_TABLE *pPortGroup;

	if (!switchp || switchp->PortGroupFDB) {
		IXmlParserPrintError(state, "SwitchData improperly allocated");
		return NULL;
	}

	if (!switchp->PortGroupFDBSize) {
		// PortGroupFDB is not in snapshot.  For earlier
		// versions of STL1, use LinearFDB but cap at 8k
		switchp->PortGroupFDBSize = MIN(switchp->LinearFDBSize, DEFAULT_MAX_PGFT_LID+1);
	}

	if ( !(pPortGroup = (STL_PORT_GROUP_FORWARDING_TABLE *)MemoryAllocate2AndClear(
	   ROUNDUP(switchp->PortGroupFDBSize, MAX_LFT_ELEMENTS_BLOCK), IBA_MEM_FLAG_PREMPTABLE, MYTAG))) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	memset(pPortGroup, 255, ROUNDUP(switchp->PortGroupFDBSize, MAX_LFT_ELEMENTS_BLOCK));
	switchp->PortGroupFDB = pPortGroup;

	return (switchp);
}

static void SwitchDataXmlParserEndPortGroupFDB(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SwitchData *switchp = (SwitchData *)object;

	if (!valid) {
		goto fail;
	}
	return;

fail:
	if (switchp && switchp->PortGroupFDB) {
		MemoryDeallocate(switchp->PortGroupFDB);
		switchp->PortGroupFDB = NULL;
	}
}


static void IXmlParserEndMulticastFDBSize(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 value;
	SwitchData *switchp = (SwitchData *)object;

	if (IXmlParseUint32(state, content, len, &value)) {
		if (IS_MCAST16(value))
			value = MCAST16_TO_MCAST32(value);
		if (value && ((value <= STL_LID_MULTICAST_BEGIN) || (value > STL_LID_MULTICAST_END + 1)))
			IXmlParserPrintError(state, "MulticastFDBSize value 0x%08x out of range.  Must be 0 or in the range 0x%08x through 0x%08x\n", value, STL_LID_MULTICAST_BEGIN + 1, STL_LID_MULTICAST_END + 1);
		else
			switchp->MulticastFDBSize = value;
	}
}


IXML_FIELD SwitchDataFields[] = {
	{ tag:"LinearFDBSize", format:'U', IXML_FIELD_INFO(SwitchData, LinearFDBSize) },
	{ tag:"MulticastFDBSize", format:'U', IXML_FIELD_INFO(SwitchData, MulticastFDBSize), end_func:IXmlParserEndMulticastFDBSize},
	{ tag:"PortGroupFDBSize", format:'u', IXML_FIELD_INFO(SwitchData, PortGroupFDBSize) },
	{ tag:"PortGroupSize", format:'U', IXML_FIELD_INFO(SwitchData, PortGroupSize) },
	{ tag:"LinearFDB", format:'k', format_func:SwitchDataOutputLinearFDB, subfields:LinearFDBFields, start_func:SwitchDataXmlParserStartLinearFDB, end_func:SwitchDataXmlParserEndLinearFDB },
	{ tag:"MulticastFDB", format:'k', format_func:SwitchDataOutputMulticastFDB, subfields:MulticastFDBFields, start_func:SwitchDataXmlParserStartMulticastFDB, end_func:SwitchDataXmlParserEndMulticastFDB },
	{ tag:"PortGroupElements", format:'k', format_func:SwitchDataOutputPortGroupElements, subfields:PortGroupElementsFields, start_func:SwitchDataXmlParserStartPortGroupElements, end_func:SwitchDataXmlParserEndPortGroupElements },
	{ tag:"PortGroupFDB", format:'k', format_func:SwitchDataOutputPortGroupFDB, subfields:PortGroupFDBFields, start_func:SwitchDataXmlParserStartPortGroupFDB, end_func:SwitchDataXmlParserEndPortGroupFDB },
	{ NULL }
};

void SwitchDataXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (SwitchData*)data, NULL, SwitchDataFields);
}

// only output if value != NULL
void SwitchDataXmlOutputOptional(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalStruct(state, tag, (SwitchData*)data, NULL, SwitchDataFields);
}

/****************************************************************************/
/* NodeData Input/Output functions */

static void NodeDataXmlOutputPorts(IXmlOutputState_t *state, const char *tag, void *data)
{
	NodeData *nodep = (NodeData*)data;
	cl_map_item_t *p;

	for (p=cl_qmap_head(&nodep->Ports); p != cl_qmap_end(&nodep->Ports); p = cl_qmap_next(p)) {
		PortDataXmlOutput(state, "PortInfo", PARENT_STRUCT(p, PortData, NodePortsEntry));
	}
}

#if !defined(VXWORKS) || defined(BUILD_DMC)
static void NodeDataXmlOutputIou(IXmlOutputState_t *state, const char *tag, void *data)
{
	NodeData *nodep = (NodeData*)data;

	if (nodep->ioup)
		IouDataXmlOutput(state, "Iou", nodep->ioup);
}
#endif

static void NodeDataXmlOutputSwitchInfo(IXmlOutputState_t *state, const char *tag, void *data)
{
	NodeData *nodep = (NodeData*)data;

	if (nodep->pSwitchInfo)
		SwitchInfoXmlOutput(state, "SwitchInfo", nodep->pSwitchInfo);
}

static void NodeDataXmlOutputSwitchData(IXmlOutputState_t *state, const char *tag, void *data)
{
	NodeData *nodep = (NodeData*)data;

	if (nodep->switchp)
		SwitchDataXmlOutput(state, "SwitchData", nodep->switchp);
}

/* bitfields needs special handling: VendorID */
static void NodeDataXmlOutputVendorID(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag, ((NodeData *)data)->NodeInfo.u1.s.VendorID);
}

static void NodeDataXmlParserEndVendorID(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint32 value;
	
	if (IXmlParseUint32(state, content, len, &value))
		((NodeData *)object)->NodeInfo.u1.s.VendorID = value;
}

IXML_FIELD CongestionInfoFields[] = {
	{ tag:"IBACongestionInfo", format:'X', IXML_FIELD_INFO(STL_CONGESTION_INFO, CongestionInfo) },
	{ tag:"ControlTableCap", format:'U', IXML_FIELD_INFO(STL_CONGESTION_INFO, ControlTableCap) },
	{ tag:"CongestionLogLength", format:'U', IXML_FIELD_INFO(STL_CONGESTION_INFO, CongestionLogLength) },
	{ NULL }
};

static void NodeDataXmlOutputCongestionInfo(IXmlOutputState_t *state, const char *tag, void *data)
{
	NodeData *nodep = (NodeData*)data;
	IXmlOutputStruct(state, tag, (STL_CONGESTION_INFO *)&nodep->CongestionInfo, NULL, CongestionInfoFields);
}

static void CongestionInfoXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	STL_CONGESTION_INFO *pci = (STL_CONGESTION_INFO*)object;
	NodeData *nodep = (NodeData*)parent;

	if (! valid)	// missing mandatory fields
		goto failvalidate;

	memcpy(&nodep->CongestionInfo, pci, sizeof(STL_CONGESTION_INFO));

failvalidate:
	MemoryDeallocate(pci);
	return;
}

const IXML_FIELD NodeDataFields[] = {
	{ tag:"NodeDesc", format:'C', IXML_FIELD_INFO(NodeData, NodeDesc.NodeString) },
	{ tag:"NodeGUID", format:'H', IXML_FIELD_INFO(NodeData, NodeInfo.NodeGUID) },
	{ tag:"NodeType", format:'k', IXML_FIELD_INFO(NodeData, NodeInfo.NodeType), format_func:IXmlOutputNodeType, end_func:IXmlParserEndNoop },	// output only
	{ tag:"NodeType_Int", format:'U', IXML_FIELD_INFO(NodeData, NodeInfo.NodeType), format_func:IXmlOutputNoop },	// input only
	{ tag:"BaseVersion", format:'U', IXML_FIELD_INFO(NodeData, NodeInfo.BaseVersion) },
	{ tag:"ClassVersion", format:'U', IXML_FIELD_INFO(NodeData, NodeInfo.ClassVersion) },
	{ tag:"NumPorts", format:'U', IXML_FIELD_INFO(NodeData, NodeInfo.NumPorts) },
	{ tag:"SystemImageGUID", format:'H', IXML_FIELD_INFO(NodeData, NodeInfo.SystemImageGUID) },
	// NodeData.NodeInfo.PortGUID is not used
	{ tag:"PartitionCap", format:'U', IXML_FIELD_INFO(NodeData, NodeInfo.PartitionCap) },
	{ tag:"DeviceID", format:'H', IXML_FIELD_INFO(NodeData, NodeInfo.DeviceID) },
	{ tag:"Revision", format:'H', IXML_FIELD_INFO(NodeData, NodeInfo.Revision) },
	// NodeData.NodeInfo.u1.s.LocalPortNum is not used
	{ tag:"VendorID", format:'H', format_func:NodeDataXmlOutputVendorID, end_func:NodeDataXmlParserEndVendorID },
	{ tag:"PortInfo", format:'k', format_func:NodeDataXmlOutputPorts, subfields:PortDataFields, start_func:PortDataXmlParserStart, end_func:PortDataXmlParserEnd }, // structures
#if !defined(VXWORKS) || defined(BUILD_DMC)
	{ tag:"Iou", format:'k', format_func:NodeDataXmlOutputIou, subfields:IouDataFields, start_func:IouDataXmlParserStart, end_func:IouDataXmlParserEnd }, // structure
#endif
	{ tag:"SwitchInfo", format:'k', size:sizeof(STL_SWITCHINFO_RECORD), format_func:NodeDataXmlOutputSwitchInfo, subfields:SwitchInfoFields, start_func:SwitchInfoXmlParserStart, end_func:SwitchInfoXmlParserEnd }, // structure
	{ tag:"SwitchData", format:'k', size:sizeof(SwitchData), format_func:NodeDataXmlOutputSwitchData, subfields:SwitchDataFields, start_func:SwitchDataXmlParserStart, end_func:SwitchDataXmlParserEnd }, // structure
	{ tag:"CongestionInfo", format:'k', size:sizeof(STL_CONGESTION_INFO), format_func:NodeDataXmlOutputCongestionInfo, subfields:CongestionInfoFields, start_func:IXmlParserStartStruct, end_func:CongestionInfoXmlParserEnd }, // structure
	{ NULL }
};

static void NodeDataXmlFormatAttr(IXmlOutputState_t *state, void *data)
{
	IXmlOutputPrint(state, " id=\"0x%016"PRIx64"\"", ((NodeData*)data)->NodeInfo.NodeGUID);
}

static void NodeDataXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (NodeData*)data, NodeDataXmlFormatAttr, NodeDataFields);
}

static void *NodeDataXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	NodeData *nodep = (NodeData*)MemoryAllocate2AndClear(sizeof(NodeData), IBA_MEM_FLAG_PREMPTABLE, MYTAG);

	// TBD - if enable then need quiet arg in a static global
	//if (i%PROGRESS_FREQ == 0)
		//ProgressPrint(FALSE, "Processed %6d of %6d Nodes...", i, p->NumNodeRecords);
	if (! nodep) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	cl_qmap_init(&nodep->Ports, NULL);
	ListItemInitState(&nodep->AllTypesEntry);
	QListSetObj(&nodep->AllTypesEntry, nodep);

	return nodep;
}

static void NodeDataXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	cl_map_item_t *mi;
	NodeData *nodep = (NodeData*)object;
	FabricData_t *fabricp = IXmlParserGetContext(state);
	//FSTATUS status;

	if (! valid)	// missing mandatory fields
		goto failvalidate;

	// TODO should this enforce if NodeType == NI_TYPE_SWITCH that 
	// SwitchData has been parsed (nodep->switchp != NULL)?

	mi = cl_qmap_insert(&fabricp->AllNodes, nodep->NodeInfo.NodeGUID, &nodep->AllNodesEntry);
	if (mi != &nodep->AllNodesEntry)
	{
		IXmlParserPrintError(state, "Duplicate NodeGuid: 0x%"PRIx64"\n", nodep->NodeInfo.NodeGUID);
		goto failinsert;
	}

	//printf("processed NodeRecord GUID: 0x%"PRIx64"\n", nodep->NodeInfo.NodeGUID);
	if (FSUCCESS != AddSystemNode(fabricp, nodep)) {
		IXmlParserPrintError(state, "Unable to track systems for NodeGuid: 0x%"PRIx64"\n", nodep->NodeInfo.NodeGUID);
		goto failsystem;
	}

	// Set FF_ROUTES for cases (older opareport) where it was not set at
	//  snapshot generation time
	if (fbFDB)
		fabricp->flags |= FF_ROUTES;
	return;

failsystem:
	cl_qmap_remove_item(&fabricp->AllNodes, &nodep->AllNodesEntry);
failinsert:
failvalidate:
	MemoryDeallocate(nodep);
}

static IXML_FIELD NodesFields[] = {
	{ tag:"Node", format:'K', subfields:(IXML_FIELD*)NodeDataFields, start_func:NodeDataXmlParserStart, end_func:NodeDataXmlParserEnd }, // structure
	{ NULL }
};

void Snapshot_NodeDataFree(NodeData * nodep, FabricData_t * fabricp)
{
	NodeDataFreePorts(fabricp, nodep);
#if !defined(VXWORKS) || defined(BUILD_DMC)
	if (nodep->ioup)
		IouDataFree(fabricp, nodep->ioup);
#endif
	if (nodep->pSwitchInfo)
		MemoryDeallocate(nodep->pSwitchInfo);
	NodeDataFreeSwitchData(fabricp, nodep);
}

/****************************************************************************/
/* SM Data Input/Output functions */

static void SMDataXmlFormatAttr(IXmlOutputState_t *state, void *data)
{
	PortDataXmlFormatAttr(state, ((SMData *)data)->portp);
}

#if 0
static void SMDataXmlOutputNodeGUID(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputHexPad64(state, tag, ((SMData *)data)->portp->nodep->NodeInfo.NodeGUID);
}
#endif

#if 0
// since SM's don't nest, we can use a simple global
static struct {
	EUI64 NodeGUID;
	uint8 PortNum;
} TempSMData;

static void SMDataXmlParserEndNodeGUID(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	IXmlParseUint64(state, content, len, &TempSMData.NodeGUID);
}

static void SMDataXmlParserEndPortNum(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	IXmlParseUint8(state, content, len, &TempSMData.PortNum);
}
#endif

#if 0
static void SMDataXmlOutputPortNum(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag, ((SMData *)data)->portp->PortNum);
}
#endif

/* bitfields needs special handling: LID */
static void SMDataXmlOutputLID(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputLIDValue(state, tag, ((SMData *)data)->SMInfoRecord.RID.LID);
}

static void SMDataXmlParserEndLID(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	STL_LID value;
	
	if (IXmlParseUint32(state, content, len, &value))
		((SMData *)object)->SMInfoRecord.RID.LID = value;
}


/* bitfields needs special handling: SMState */
static void SMDataXmlOutputSMState(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputSMStateValue(state, tag, ((SMData *)data)->SMInfoRecord.SMInfo.u.s.SMStateCurrent);
}

static void SMDataXmlParserEndSMState(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((SMData *)object)->SMInfoRecord.SMInfo.u.s.SMStateCurrent = value;
}

/* bitfields needs special handling: Priority */
static void SMDataXmlOutputPriority(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputUint(state, tag, ((SMData *)data)->SMInfoRecord.SMInfo.u.s.Priority);
}

static void SMDataXmlParserEndPriority(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value))
		((SMData *)object)->SMInfoRecord.SMInfo.u.s.Priority = value;
}

static IXML_FIELD SMDataFields[] = {
	//{ tag:"NodeGUID", format:'k', format_func:SMDataXmlOutputNodeGUID, end_func:IXmlParserEndNoop }, // output only
	//{ tag:"PortNum", format:'k', format_func:SMDataXmlOutputPortNum, end_func:IXmlParserEndNoop }, // output only
	{ tag:"LID", format:'K', format_func:SMDataXmlOutputLID, end_func:SMDataXmlParserEndLID }, // bitfield
	{ tag:"PortGUID", format:'H', IXML_FIELD_INFO(SMData, SMInfoRecord.SMInfo.PortGUID) },
	{ tag:"State", format:'k', format_func:SMDataXmlOutputSMState, end_func:IXmlParserEndNoop },// output only bitfield
	{ tag:"State_Int", format:'K', format_func:IXmlOutputNoop, end_func:SMDataXmlParserEndSMState }, // input only bitfield
	{ tag:"Priority", format:'K', format_func:SMDataXmlOutputPriority, end_func:SMDataXmlParserEndPriority }, // bitfield
	{ tag:"SM_Key", format:'H', IXML_FIELD_INFO(SMData, SMInfoRecord.SMInfo.SM_Key) },
	{ tag:"ActCount", format:'X', IXML_FIELD_INFO(SMData, SMInfoRecord.SMInfo.ActCount) },
	{ NULL }
};

static void SMDataXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (SMData*)data, SMDataXmlFormatAttr, SMDataFields);
}

static void SMDataXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	SMData *smp = (SMData*)object;
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (! valid)
		goto invalid;
	smp->portp = FindLidPort(fabricp, smp->SMInfoRecord.RID.LID, 0);
	if (! smp->portp) {
		IXmlParserPrintError(state, "SM LID not found: 0x%x\n", smp->SMInfoRecord.RID.LID);
		goto badport;
	}
	if (&smp->AllSMsEntry != cl_qmap_insert(&fabricp->AllSMs, smp->SMInfoRecord.SMInfo.PortGUID, &smp->AllSMsEntry)) {
		IXmlParserPrintError(state, "Duplicate SM Port Guids: 0x%016"PRIx64"\n", smp->SMInfoRecord.SMInfo.PortGUID);
		goto failinsert;
	}
	return;

failinsert:
badport:
invalid:
	MemoryDeallocate(smp);
}

static IXML_FIELD SMsFields[] = {
	{ tag:"SM", format:'k', size:sizeof(SMData), subfields:SMDataFields, start_func:IXmlParserStartStruct, end_func:SMDataXmlParserEnd }, // structure
	{ NULL }
};

/****************************************************************************/
/* Link Input/Output functions */
static void LinkXmlOutputNodeGUID(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputHexPad64(state, tag, ((PortData*)data)->nodep->NodeInfo.NodeGUID);
}

/* Output fields for PortData to include in To and From Link */
static IXML_FIELD LinkPortFields[] = {
	{ tag:"NodeGUID", format:'K', format_func:LinkXmlOutputNodeGUID },
	{ tag:"PortNum", format:'U', IXML_FIELD_INFO(PortData, PortNum) },
	{ NULL }
};

#ifndef JFORMAT
static void LinkXmlOutputFrom(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (PortData*)data, NULL /*PortDataXmlFormatAttr*/, LinkPortFields);
}
#endif

static void LinkXmlOutputTo(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, ((PortData*)data)->neighbor, NULL /*PortDataXmlFormatAttr*/, LinkPortFields);
}

struct portref {
	EUI64 NodeGUID;
	uint8 PortNum;
};

typedef struct TempLinkData {
	struct portref from;
	struct portref to;
} TempLinkData_t;

/* Input fields for TempLinkData */
static IXML_FIELD TempLinkDataFromFields[] = {
	{ tag:"NodeGUID", format:'H', IXML_FIELD_INFO(TempLinkData_t, from.NodeGUID)},
	{ tag:"PortNum", format:'U', IXML_FIELD_INFO(TempLinkData_t, from.PortNum) },
	{ NULL }
};
static IXML_FIELD TempLinkDataToFields[] = {
	{ tag:"NodeGUID", format:'H', IXML_FIELD_INFO(TempLinkData_t, to.NodeGUID)},
	{ tag:"PortNum", format:'U', IXML_FIELD_INFO(TempLinkData_t, to.PortNum) },
	{ NULL }
};

static void *LinkFromXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return parent;
}

static void *LinkToXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	return parent;
}


/* <Link> description */
static IXML_FIELD LinkFields[] = {
#ifdef JFORMAT
	{ tag:"From", format:'J', subfields: LinkPortFields, format_attr:PortDataXmlFormatAttr },
	{ tag:"To", format:'K', subfields: LinkPortFields, format_func:LinkXmlOutputTo },
#else
	{ tag:"From", format:'K', subfields: TempLinkDataFromFields, format_func:LinkXmlOutputFrom, start_func:LinkFromXmlParserStart, end_func:IXmlParserEndNoop }, // special handling
	{ tag:"To", format:'K', subfields: TempLinkDataToFields, format_func:LinkXmlOutputTo, start_func:LinkToXmlParserStart, end_func:IXmlParserEndNoop }, // special handling
#endif
	{ NULL }
};



static void LinkXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag,  (PortData*)data, PortDataXmlFormatAttr, LinkFields);
}


static void *LinkXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	/* since links don't nest, we can get away with a single static */
	/* if we ever use this multi-threaded, will need to allocate */
	static TempLinkData_t link;

	MemoryClear(&link, sizeof(link));
	return &link;
}

static void LinkXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	TempLinkData_t *link = (TempLinkData_t*)object;
	PortData *p1, *p2;
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (! valid)
		goto invalid;
	p1 = FindNodeGuidPort(fabricp, link->from.NodeGUID, link->from.PortNum);
	if (! p1) {
		IXmlParserPrintError(state, "Port not found: 0x%016"PRIx64":%u\n",
								link->from.NodeGUID, link->from.PortNum);
		goto badport;
	}
	if (p1->neighbor) {
		IXmlParserPrintError(state, "Duplicate Port found: 0x%016"PRIx64":%u\n",
								link->from.NodeGUID, link->from.PortNum);
		goto badport;
	}
	p2 = FindNodeGuidPort(fabricp, link->to.NodeGUID, link->to.PortNum);
	if (! p2) {
		IXmlParserPrintError(state, "Port not found: 0x%016"PRIx64":%u\n",
								link->to.NodeGUID, link->to.PortNum);
		goto badport;
	}
	if (p2->neighbor) {
		IXmlParserPrintError(state, "Duplicate Port found: 0x%016"PRIx64":%u\n",
								link->to.NodeGUID, link->to.PortNum);
		goto badport;
	}
	p1->neighbor = p2;
	p2->neighbor = p1;
	p1->from = 1;
	if (p1->rate != p2->rate) {
		fprintf(stderr, "%s: Warning: Ignoring Inconsistent Active Speed/Width for link between:\n", g_Top_cmdname);
		fprintf(stderr, "  %4s 0x%016"PRIx64" %3u %s %.*s\n",
				StlStaticRateToText(p1->rate),
				p1->nodep->NodeInfo.NodeGUID,
				p1->PortNum,
				StlNodeTypeToText(p1->nodep->NodeInfo.NodeType),
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)p1->nodep->NodeDesc.NodeString);
		fprintf(stderr, "  %4s 0x%016"PRIx64" %3u %s %.*s\n",
				StlStaticRateToText(p2->rate),
				p2->nodep->NodeInfo.NodeGUID,
				p2->PortNum,
				StlNodeTypeToText(p2->nodep->NodeInfo.NodeType),
				STL_NODE_DESCRIPTION_ARRAY_SIZE,
				(char*)p2->nodep->NodeDesc.NodeString);
	}
	++(fabricp->LinkCount);
	if (! isInternalLink(p1))
		++(fabricp->ExtLinkCount);
	if (isFILink(p1))
		++(fabricp->FILinkCount);
	if (isISLink(p1))
		++(fabricp->ISLinkCount);
	if (! isInternalLink(p1)&& isISLink(p1))
		++(fabricp->ExtISLinkCount);

badport:
invalid:
	/* nothing to free */
	return;
}


static IXML_FIELD LinksFields[] = {
	{ tag:"Link", format:'K', subfields:LinkFields, start_func:LinkXmlParserStart, end_func:LinkXmlParserEnd }, // structure
	{ NULL }
};

/****************************************************************************/
/* Virtual Fabrics Input/Output functions */

static void VFInfoXmlOutputSelectFlags(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->s1.selectFlags);
}
static void VFInfoXmlParserEndSelectFlags(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->s1.selectFlags = value;
}

static void VFInfoXmlOutputSL(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->s1.slBase);
}

static void VFInfoXmlParserEndSL(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->s1.slBase = value;
}

static void VFInfoXmlOutputRespSL(IXmlOutputState_t *state, const char *tag, void *data) {
	if (((STL_VFINFO_RECORD*)data)->slResponseSpecified)
		IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->slResponse);
}

static void VFInfoXmlParserEndRespSL(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value)) {
		((STL_VFINFO_RECORD*)object)->slResponse = value;
		((STL_VFINFO_RECORD*)object)->slResponseSpecified = 1;
	}
}

static void VFInfoXmlOutputMulticastSL(IXmlOutputState_t *state, const char *tag, void *data) {
	if (((STL_VFINFO_RECORD*)data)->slMulticastSpecified)
		IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->slMulticast);
}

static void VFInfoXmlParserEndMulticastSL(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value)) {
		((STL_VFINFO_RECORD*)object)->slMulticast = value;
		((STL_VFINFO_RECORD*)object)->slMulticastSpecified = 1;
	}
}

static void VFInfoXmlOutputMTUSpecified(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->s1.mtuSpecified);
}
static void VFInfoXmlParserEndMTUSpecified(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->s1.mtuSpecified = value;
}

static void VFInfoXmlOutputMTU(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->s1.mtu);
}
static void VFInfoXmlParserEndMTU(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->s1.mtu = value;
}

static void VFInfoXmlOutputRateSpecified(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->s1.rateSpecified);
}
static void VFInfoXmlParserEndRateSpecified(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->s1.rateSpecified = value;
}

static void VFInfoXmlOutputRate(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->s1.rate);
}
static void VFInfoXmlParserEndRate(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->s1.rate = value;
}

static void VFInfoXmlOutputPacketLifeSpecified(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->s1.pktLifeSpecified);
}
static void VFInfoXmlParserEndPacketLifeSpecified(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->s1.pktLifeSpecified = value;
}

static void VFInfoXmlOutputPacketLifeTimeInc(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->s1.pktLifeTimeInc);
}
static void VFInfoXmlParserEndPacketLifeTimeInc(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->s1.pktLifeTimeInc = value;
}

static void VFInfoXmlOutputSlResponse(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->slResponse);
}
static void VFInfoXmlParserEndSlResponse(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->slResponse = value;
}

static void VFInfoXmlOutputPriority(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->priority);
}
static void VFInfoXmlParserEndPriority(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->priority = value;
}

static void VFInfoXmlOutputPreemptionRank(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->preemptionRank);
}
static void VFInfoXmlParserEndPreemptionRank(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->preemptionRank = value;
}

static void VFInfoXmlOutputHoqLife(IXmlOutputState_t *state, const char *tag, void *data) {
	IXmlOutputUint(state, tag, ((STL_VFINFO_RECORD *)data)->hoqLife);
}
static void VFInfoXmlParserEndHoqLife(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid) {
	uint8 value;
	if (IXmlParseUint8(state, content, len, &value))
		((STL_VFINFO_RECORD *)object)->hoqLife = value;
}

static IXML_FIELD VFFields[] = {
	{ tag:"Index", format:'U', IXML_FIELD_INFO(STL_VFINFO_RECORD, vfIndex) },
	{ tag:"PKey", format:'u', IXML_FIELD_INFO(STL_VFINFO_RECORD, pKey) },
	{ tag:"Name", format:'S', IXML_FIELD_INFO(STL_VFINFO_RECORD, vfName) },
	{ tag:"ServiceID", format:'h', IXML_FIELD_INFO(STL_VFINFO_RECORD, ServiceID) },
	{ tag:"MGIDHigh", format:'h', IXML_FIELD_INFO(STL_VFINFO_RECORD, MGID.AsReg64s.H) },
	{ tag:"MGIDLow", format:'h', IXML_FIELD_INFO(STL_VFINFO_RECORD, MGID.AsReg64s.L) },
	{ tag:"SelectFlags", format:'k', format_func:VFInfoXmlOutputSelectFlags, end_func:VFInfoXmlParserEndSelectFlags },
	{ tag:"SL", format:'k', format_func:VFInfoXmlOutputSL, end_func:VFInfoXmlParserEndSL },
	{ tag:"RespSL", format:'k', format_func:VFInfoXmlOutputRespSL, end_func:VFInfoXmlParserEndRespSL },
	{ tag:"MulticastSL", format:'k', format_func:VFInfoXmlOutputMulticastSL, end_func:VFInfoXmlParserEndMulticastSL },
	{ tag:"MTUSpecified", format:'k', format_func:VFInfoXmlOutputMTUSpecified, end_func:VFInfoXmlParserEndMTUSpecified },
	{ tag:"MTU", format:'k', format_func:VFInfoXmlOutputMTU, end_func:VFInfoXmlParserEndMTU },
	{ tag:"RateSpecified", format:'k', format_func:VFInfoXmlOutputRateSpecified, end_func:VFInfoXmlParserEndRateSpecified },
	{ tag:"Rate", format:'k', format_func:VFInfoXmlOutputRate, end_func:VFInfoXmlParserEndRate },
	{ tag:"PacketLifeSpecified", format:'k', format_func:VFInfoXmlOutputPacketLifeSpecified, end_func:VFInfoXmlParserEndPacketLifeSpecified },
	{ tag:"PacketLifeTimeInc", format:'k', format_func:VFInfoXmlOutputPacketLifeTimeInc, end_func:VFInfoXmlParserEndPacketLifeTimeInc },
	{ tag:"OptionFlags", format:'h', IXML_FIELD_INFO(STL_VFINFO_RECORD, optionFlags) },
	{ tag:"BandwidthPercent", format:'u', IXML_FIELD_INFO(STL_VFINFO_RECORD, bandwidthPercent) },
	{ tag:"SLResponse", format:'k', format_func:VFInfoXmlOutputSlResponse, end_func:VFInfoXmlParserEndSlResponse },
	{ tag:"Priority", format:'k', format_func:VFInfoXmlOutputPriority, end_func:VFInfoXmlParserEndPriority },
	{ tag:"PreemptionRank", format:'k', format_func:VFInfoXmlOutputPreemptionRank, end_func:VFInfoXmlParserEndPreemptionRank },
	{ tag:"HOQLife", format:'k', format_func:VFInfoXmlOutputHoqLife, end_func:VFInfoXmlParserEndHoqLife },
	{ NULL }
};

static void *VFDataXmlParserStartVF(IXmlParserState_t *state, void *parent, const char **attr)
{
	VFData_t *vf = MemoryAllocate2AndClear(sizeof(VFData_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (!vf) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	return &vf->record;
}

static void VFDataXmlParserEndVF(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	FabricData_t *fabricp = IXmlParserGetContext(state);
	STL_VFINFO_RECORD *vfinfo = object;
	VFData_t *vf = PARENT_STRUCT(vfinfo, VFData_t, record);

	if (valid) {
		QListSetObj(&vf->AllVFsEntry, vf);
		QListInsertTail(&fabricp->AllVFs, &vf->AllVFsEntry);
	} else if (vf) {
		MemoryDeallocate(vf);
	}
}

static IXML_FIELD VFsFields[] = {
	{ tag:"VF", format:'K', subfields:VFFields, start_func:VFDataXmlParserStartVF, end_func:VFDataXmlParserEndVF },
	{ NULL }
};

/****************************************************************************/
/* Overall Serialization Input/Output functions */

/* only used for input parsing */
static IXML_FIELD SnapshotFields[] = {
	{ tag:"Nodes", format:'K', subfields:NodesFields }, // list
	{ tag:"SMs", format:'K', subfields:SMsFields }, // list
	{ tag:"Links", format:'K', subfields:LinksFields }, // list
	{ tag:"McMembers", format:'k', subfields:MulticastFields }, // list
	{ tag:"VirtualFabrics", format:'k', subfields:VFsFields },
	{ NULL }
};

static void *SnapshotXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	int i;
	boolean gottime = FALSE;
	boolean gotstats = FALSE;
	FabricData_t *fabricp = IXmlParserGetContext(state);
	uint64 temp;

	// process unixtime attribute and update fabricp->time
	for (i = 0; attr[i]; i += 2) {
		if (strcmp(attr[i], "unixtime") == 0) {
			gottime = TRUE;
			// typically time_t is 32 bits, but allow 64 bits
			if (FSUCCESS != StringToUint64(&temp, attr[i+1], NULL, 0, TRUE))
				IXmlParserPrintError(state, "Invalid unixtime attribute: %s", attr[i+1]);
			else
				fabricp->time = (time_t)temp;
		} else if (strcmp(attr[i], "stats") == 0) {
			gotstats = TRUE;
			if (FSUCCESS != StringToUint64(&temp, attr[i+1], NULL, 0, TRUE))
				IXmlParserPrintError(state, "Invalid stats attribute: %s", attr[i+1]);
			else
				fabricp->flags |= temp?FF_STATS:FF_NONE;
		} else if (strcmp(attr[i], "routes") == 0) {
			if (FSUCCESS != StringToUint64(&temp, attr[i+1], NULL, 0, TRUE))
				IXmlParserPrintError(state, "Invalid routes attribute: %s", attr[i+1]);
			else
				fabricp->flags |= temp?FF_ROUTES:FF_NONE;
		} else if (strcmp(attr[i], "qosdata") == 0) {
			if (FSUCCESS != StringToUint64(&temp, attr[i+1], NULL, 0, TRUE))
				IXmlParserPrintError(state, "Invalid qosdata attribute: %s", attr[i+1]);
			else
				fabricp->flags |= temp?FF_QOSDATA:FF_NONE;
		} else if (strcmp(attr[i], "bfrctrl") == 0) {
			if (FSUCCESS != StringToUint64(&temp, attr[i+1], NULL, 0, TRUE))
				IXmlParserPrintError(state, "Invalid bfrctrl attribute: %s", attr[i+1]);
			else
				fabricp->flags |= temp?FF_BUFCTRLTABLE:FF_NONE;
		} else if (strcmp(attr[i], "downports") == 0) {
			if (FSUCCESS != StringToUint64(&temp, attr[i+1], NULL, 0, TRUE))
				IXmlParserPrintError(state, "Invalid downports attribute: %s", attr[i+1]);
			else
				fabricp->flags |= temp?FF_DOWNPORTINFO:FF_NONE;
		}
	}
	if (! gottime) {
		IXmlParserPrintError(state, "Missing unixtime attribute");
	}
	if (! gotstats) {
		IXmlParserPrintError(state, "Missing stats attribute");
	}
	fabricp->NumOfMcGroups = 0;
	return NULL;
}

static void SnapshotXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (! valid) {
		// This free's everything we built while parsing, leaving empty lists
		SMDataFreeAll(fabricp);
		NodeDataFreeAll(fabricp);
		MCDataFreeAll(fabricp);
		VFDataFreeAll(fabricp);
		fabricp->LinkCount = 0;
		fabricp->ExtLinkCount = 0;
	}
}

static IXML_FIELD TopLevelFields[] = {
	{ tag:"Snapshot", format:'K', subfields:SnapshotFields, start_func:SnapshotXmlParserStart, end_func:SnapshotXmlParserEnd }, // structure
	{ NULL }
};
#if 0
static IXML_FIELD TopLevelFields[] = {
	{ tag:"Report", format:'K', subfields:ReportFields }, // structure
	{ NULL }
};
#endif

static void SnapshotInfoXmlFormatAttr(IXmlOutputState_t *state, void *data)
{
	SnapshotOutputInfo_t *info = (SnapshotOutputInfo_t *)IXmlOutputGetContext(state);
	char datestr[80] = "";
	int i;

	Top_formattime(datestr, sizeof(datestr), info->fabricp->time);
	IXmlOutputPrint( state, " date=\"%s\" unixtime=\"%ld\" stats=\"%d\""
		" routes=\"%d\" qosdata=\"%d\" bfrctrl=\"%d\" downports=\"%d\" options=\"",
		datestr, info->fabricp->time, (info->fabricp->flags & FF_STATS) ? 1:0,
		(info->fabricp->flags & FF_ROUTES) ? 1:0,
		(info->fabricp->flags & FF_QOSDATA) ? 1:0,
		(info->fabricp->flags & FF_BUFCTRLTABLE) ? 1:0,
		(info->fabricp->flags & FF_DOWNPORTINFO) ? 1:0 );
	for (i=1; i<info->argc; i++)
		IXmlOutputPrint(state, "%s%s", i>1?" ":"", info->argv[i]);
	IXmlOutputPrint(state, "\"");
}

static void Xml2PrintAll(IXmlOutputState_t *state, const char *tag, void *data)
{
	SnapshotOutputInfo_t *info = (SnapshotOutputInfo_t *)IXmlOutputGetContext(state);
	FabricData_t *fabricp = info->fabricp;

	IXmlOutputStartAttrTag(state, tag, NULL, SnapshotInfoXmlFormatAttr);

	{
		cl_map_item_t *p;

		IXmlOutputStartAttrTag(state, "Nodes", NULL, NULL);
		for (p=cl_qmap_head(&fabricp->AllNodes); p != cl_qmap_end(&fabricp->AllNodes); p = cl_qmap_next(p)) {
			NodeData *nodep = PARENT_STRUCT(p, NodeData, AllNodesEntry);
#if 0
			if (! CompareNodePoint(nodep, info->focus))
				continue;
#endif
			NodeDataXmlOutput(state, "Node", nodep);
		}
		IXmlOutputEndTag(state, "Nodes");
	}

	{
		cl_map_item_t *p;

		IXmlOutputStartAttrTag(state, "SMs", NULL, NULL);
		for (p=cl_qmap_head(&fabricp->AllSMs); p != cl_qmap_end(&fabricp->AllSMs); p = cl_qmap_next(p)) {
			SMDataXmlOutput(state, "SM", PARENT_STRUCT(p, SMData, AllSMsEntry));
		}
		IXmlOutputEndTag(state, "SMs");
	}

	{
		LIST_ITEM *p;

		IXmlOutputStartAttrTag(state, "Links", NULL, NULL);
		for (p=QListHead(&fabricp->AllPorts); p != NULL; p = QListNext(&fabricp->AllPorts, p)) {
			PortData *portp = (PortData *)QListObj(p);
			// to avoid duplicated processing, only process "from" ports in link
			if (! portp->from)
				continue;
			LinkXmlOutput(state, "Link", portp);
		}
		IXmlOutputEndTag(state, "Links");
	}

	{
		LIST_ITEM *p;

		IXmlOutputStartAttrTag(state, "McMembers", NULL, NULL);
		for (p=QListHead(&fabricp->AllMcGroups); p != NULL; p = QListNext(&fabricp->AllMcGroups, p)) {
			McGroupData *mcgroupp = (McGroupData *)QListObj(p);
			McGroupMemberXmlOutput(state, "MulticastGroup", mcgroupp);
		}
		IXmlOutputEndTag(state, "McMembers");
	}

	{
		LIST_ITEM *p;

		IXmlOutputStartTag(state, "VirtualFabrics");
		for (p=QListHead(&fabricp->AllVFs); p != NULL; p = QListNext(&fabricp->AllVFs, p)) {
			VFData_t *vf = (VFData_t *)QListObj(p);
			IXmlOutputStruct(state, "VF", &vf->record, NULL, VFFields);
		}
		IXmlOutputEndTag(state, "VirtualFabrics");
	}

	IXmlOutputEndTag(state, tag);
}

void Xml2PrintSnapshot(FILE *file, SnapshotOutputInfo_t *info)
{
	IXmlOutputState_t state;

	/* using SERIALIZE with no indent makes output less pretty but 1/2 the size */
	if (FSUCCESS != IXmlOutputInit(&state, file, 0, IXML_OUTPUT_FLAG_SERIALIZE, info))
	//if (FSUCCESS != IXmlOutputInit(&state, file, 4, IXML_OUTPUT_FLAG_NONE, info))
		goto fail;
	
	//IXmlOutputStartAttrTag(&state, "Report", NULL, NULL);
	Xml2PrintAll(&state, "Snapshot", NULL);
	//IXmlOutputEndTag(&state, "Report");

	IXmlOutputDestroy(&state);

	return;

fail:
	return;
}



#ifndef __VXWORKS__
FSTATUS Xml2ParseSnapshot(const char *input_file, int quiet, FabricData_t *fabricp, FabricFlags_t flags, boolean allocFull)
{
	unsigned tags_found, fields_found;
	const char *filename=input_file;

	if (FSUCCESS != InitFabricData(fabricp, flags)) {
		fprintf(stderr, "%s: Unable to initialize fabric data memory\n", g_Top_cmdname);
		return FERROR;
	}
	if (strcmp(input_file, "-") == 0) {
		filename="stdin";
		if (! quiet) ProgressPrint(TRUE, "Parsing stdin...");
		if (FSUCCESS != IXmlParseFile(stdin, "stdin", IXML_PARSER_FLAG_NONE, TopLevelFields, NULL, fabricp, NULL, NULL, &tags_found, &fields_found)) {
			return FERROR;
		}
	} else {
		if (! quiet) ProgressPrint(TRUE, "Parsing %s...", Top_truncate_str(input_file));
		if (FSUCCESS != IXmlParseInputFile(input_file, IXML_PARSER_FLAG_NONE, TopLevelFields, NULL, fabricp, NULL, NULL, &tags_found, &fields_found)) {
			return FERROR;
		}
	}
	if (tags_found != 1 || fields_found != 1) {
		fprintf(stderr, "Warning: potentially inaccurate input '%s': found %u recognized top level tags, expected 1\n", filename, tags_found);
	}
	BuildFabricDataLists(fabricp);

	/*
		Resize the switch FDB tables to their full capacity.
	*/
	if (allocFull) {
		LIST_ITEM * n;
		for (n = QListHead(&fabricp->AllSWs); n != NULL; n = QListNext(&fabricp->AllSWs, n)) {
			NodeData * node = (NodeData*)QListObj(n);
			STL_SWITCH_INFO * swInfo = &node->pSwitchInfo->SwitchInfoData;
			
			// The snapshot may not have SwitchData in it. Tables will have to be provided by application (e.g.
			// fabric_sim).
			if (!node->switchp) continue;
			assert(NodeDataSwitchResizeFDB(node, swInfo->LinearFDBCap, swInfo->MulticastFDBCap) == FSUCCESS);
		}
	}

	return FSUCCESS;
}
#else
FSTATUS Xml2ParseSnapshot(const char *input_file, int quiet, FabricData_t *fabricp, FabricFlags_t flags, boolean allocFull, XML_Memory_Handling_Suite* memsuite)
{
	unsigned tags_found, fields_found;
	const char *filename=input_file;

	if (FSUCCESS != InitFabricData(fabricp, flags)) {
		fprintf(stderr, "%s: Unable to initialize fabric data memory\n", g_Top_cmdname);
		return FERROR;
	}
	if (strcmp(input_file, "-") == 0) {
		filename="stdin";
		if (! quiet) ProgressPrint(TRUE, "Parsing stdin...");
		if (FSUCCESS != IXmlParseFile(stdin, "stdin", IXML_PARSER_FLAG_NONE, TopLevelFields, NULL, fabricp, NULL, NULL, &tags_found, &fields_found, memsuite)) {
			return FERROR;
		}
	} else {
		if (! quiet) ProgressPrint(TRUE, "Parsing %s...", Top_truncate_str(input_file));
		if (FSUCCESS != IXmlParseInputFile(input_file, IXML_PARSER_FLAG_NONE, TopLevelFields, NULL, fabricp, NULL, NULL, &tags_found, &fields_found, memsuite)) {
			return FERROR;
		}
	}
	if (tags_found != 1 || fields_found != 1) {
		fprintf(stderr, "Warning: potentially inaccurate input '%s': found %u recognized top level tags, expected 1\n", filename, tags_found);
	}
	BuildFabricDataLists(fabricp);

	/*
		Resize the switch FDB tables to their full capacity.
	*/
	if (allocFull) {
		LIST_ITEM * n;
		for (n = QListHead(&fabricp->AllSWs); n != NULL; n = QListNext(&fabricp->AllSWs, n)) {
			NodeData * node = (NodeData*)QListObj(n);
			STL_SWITCH_INFO * swInfo = &node->pSwitchInfo->SwitchInfoData;
			
			// The snapshot may not have SwitchData in it. Tables will have to be provided by application (e.g.
			// fabric_sim).
			if (!node->switchp) continue;
			assert(NodeDataSwitchResizeFDB(node, swInfo->LinearFDBCap, swInfo->MulticastFDBCap) == FSUCCESS);
		}
	}

	return FSUCCESS;
}
#endif
