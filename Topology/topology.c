/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#include "topology.h"
#include "topology_internal.h"

typedef enum {
	MATCH_NONE=0,	// worst - nothing matched
	MATCH_NODE=1,	// better - exact match on Node
	MATCH_PORT=2	// best - exact match on Port
} PortSelMatchLevel_t;

/* this file supports fabric topology input file parsing and output */

/****************************************************************************/
/* Cable Data Input/Output functions */

IXML_FIELD CableDataFields[] = {
	{ tag:"CableLength", format:'p', IXML_P_FIELD_INFO(CableData, length, CABLE_LENGTH_STRLEN) },
	{ tag:"CableLabel", format:'p', IXML_P_FIELD_INFO(CableData, label, CABLE_LABEL_STRLEN) },
	{ tag:"CableDetails", format:'p', IXML_P_FIELD_INFO(CableData, details, CABLE_DETAILS_STRLEN) },
	{ NULL }
};

static void CableDataXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (CableData*)data, NULL, CableDataFields);
}

static void *CableDataXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	ExpectedLink *elinkp = (ExpectedLink*)parent;

	return &(elinkp->CableData);
}

static void CableDataXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	CableData *cablep = (CableData*)object;
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (! valid)
		goto invalid;
	if (cablep->length || cablep->label || cablep->details)
		fabricp->flags |= FF_CABLEDATA;
	return;

invalid:
	CableDataFree(cablep);
	return;
}

/****************************************************************************/
/* PortSelector Input/Output functions */

// for use in error messages, returns pointer to static string
static char *FormatPortSelector(PortSelector *portselp)
{
	static char format[256];
	int offset = 0;

	if (portselp->NodeDesc) {
		offset += sprintf(&format[offset], "NodeDesc: %s\n", portselp->NodeDesc);
	}
	if (portselp->NodeGUID) {
		offset += sprintf(&format[offset], "NodeGUID: 0x%016"PRIx64"\n", portselp->NodeGUID);
	}
	if (portselp->PortGUID) {
		offset += sprintf(&format[offset], "PortGUID: 0x%016"PRIx64"\n", portselp->PortGUID);
	}
	if (portselp->gotPortNum) {
		offset += sprintf(&format[offset], "PortNum: %u\n", portselp->PortNum);
	}
	return format;
}

/* PortNum needs special handling since have gotPortNum flag to maintain */
static void PortSelectorXmlOutputPortNum(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortSelector *portselp = (PortSelector*)data;
	if (portselp->gotPortNum)
		IXmlOutputUint(state, tag, portselp->PortNum);
}

static void PortSelectorXmlParserEndPortNum(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value)) {
		((PortSelector *)object)->PortNum = value;
		((PortSelector *)object)->gotPortNum = 1;
	}
}

// returns 1st matching node name found
static NodeData* LookupNodeName(FabricData_t *fabricp, char *name)
{
	cl_map_item_t *p;

	for (p=cl_qmap_head(&fabricp->AllNodes); p != cl_qmap_end(&fabricp->AllNodes); p = cl_qmap_next(p)) {
		NodeData *nodep = PARENT_STRUCT(p, NodeData, AllNodesEntry);
		if (strncmp((char*)nodep->NodeDesc.NodeString,
					name, STL_NODE_DESCRIPTION_ARRAY_SIZE) == 0)
		{
			return nodep;
		}
	}
	return NULL;
}

// resolve as much as we can about the given Port Selector
static void ResolvePortSelector(FabricData_t *fabricp, PortSelector *portselp, NodeData **nodepp, PortData **portpp, PortSelMatchLevel_t *matchLevel)
{
	*nodepp = NULL;
	*portpp = NULL;
	*matchLevel=MATCH_NONE;

	if (! portselp)
		return;
	// order of preference if all specified is: NodeGUID, PortGUID, Desc
	// This way if Node Descriptions have not been fully setup and there
	// are duplicates, a "opareport -o links -x" can still be used
	// as basic template for topology input
	// Valid combos in order of consideration:
	// 		NodeGUID, PortNum
	// 		NodeGUID, PortGUID - only if PortGUID on same node as NodeGUID
	// 		NodeGUID (fully resolves to port if only 1 port on node)
	// 		NodeDesc, PortNum
	// 		NodeDesc, PortGUID - only if PortGUID on same node as NodeDesc
	// 		NodeDesc (fully resolves to port if only 1 port on node)
	// 		PortGUID, PortNum (useful if PortGUID is switch port 0)
	// 		PortGUID
	// presently:
	//		NodeGuid, NodeDesc - NodeDesc is ignored
	//		NodeGuid/Desc, PortNum, PortGUID - PortGUID is ignored
	//		NodeType is ignored
	if (portselp->NodeGUID) {
		*nodepp = FindNodeGuid(fabricp, portselp->NodeGUID);
 		if (*nodepp)
 			*matchLevel=MATCH_NODE;
	}
	if (! *nodepp && portselp->NodeDesc) {
		*nodepp = LookupNodeName(fabricp, portselp->NodeDesc);
 		if (*nodepp)
 			*matchLevel=MATCH_NODE;
	}
	if (! *nodepp && portselp->PortGUID) {
		*portpp = FindPortGuid(fabricp, portselp->PortGUID);
		if (*portpp) {
			*nodepp = (*portpp)->nodep;
 			*matchLevel=MATCH_PORT;
		}
	}
	if (*nodepp && portselp->gotPortNum) {
		PortData *portp = FindNodePort(*nodepp, portselp->PortNum);
		if (portp) {
			*portpp = portp;	// overrides PortGUID
 			*matchLevel=MATCH_PORT;
		}
	}
	if (*nodepp && ! *portpp && portselp->PortGUID) {
		PortData *portp = FindPortGuid(fabricp, portselp->PortGUID);
		if (portp && portp->nodep == *nodepp) {
			*portpp = portp;
 			*matchLevel=MATCH_PORT;
		}
	}
	if (*nodepp && ! *portpp) {
		// node matched, if only 1 active port, resolve to it
		// we are purposely soft about PortGUID and PortNum in this case
		// as such we can better report links which were Changed
		if  (cl_qmap_count(&(*nodepp)->Ports) == 1) {
			*portpp = PARENT_STRUCT(cl_qmap_head(&(*nodepp)->Ports), PortData, NodePortsEntry);
			// fuzzy match, so do not update matchLevel
		}
	}
}

// table to translate from per PortSelector matchLevel into overall rank
// of ExpectedLink match level.  This is used to compare in case of conflicts
// to decide which expected link matched "best" with an actual link
static int g_linkMatchLevel[3][3] = {
		{								// Table is
			0, // None, None			// 5 = Port,Port
			1, // None, Node			// 4 = Port,Node; Node,Port
			2, // None, Port			// 3 = Node,Node
		},								// 2 = Port,None; None,Port
		{								// 1 = Node,None; None,Node
			1, // Node, None			// 0 = None, None
			3, // Node, Node
			4, // Node, Port
		},
		{
			2, // Port, None
			4, // Port, Node
			5, // Port, Port
		}
};

static boolean isCompletePortSelector(PortSelector *portselp)
{
	if (! portselp)
		return FALSE;
	return ((portselp->NodeGUID && portselp->gotPortNum)
		|| (portselp->PortGUID)
		|| (portselp->NodeDesc && portselp->gotPortNum));
	// the two below are indeterminate
	// 		NodeGUID (fully resolves to port if only 1 port on node)
	// 		NodeDesc (fully resolves to port if only 1 port on node)
}


IXML_FIELD PortSelectorFields[] = {
	{ tag:"NodeGUID", format:'h', IXML_FIELD_INFO(PortSelector, NodeGUID) },
	{ tag:"PortGUID", format:'h', IXML_FIELD_INFO(PortSelector, PortGUID) },
	{ tag:"PortNum", format:'k', format_func: PortSelectorXmlOutputPortNum, end_func:PortSelectorXmlParserEndPortNum },
	{ tag:"NodeDesc", format:'p', IXML_P_FIELD_INFO(PortSelector, NodeDesc, STL_NODE_DESCRIPTION_ARRAY_SIZE) },
	{ tag:"PortDetails", format:'p', IXML_P_FIELD_INFO(PortSelector, details, PORT_DETAILS_STRLEN) },
	{ tag:"NodeType", format:'k', IXML_FIELD_INFO(PortSelector, NodeType), format_func:IXmlOutputOptionalNodeType, end_func:IXmlParserEndNodeType },	// input str output both
	{ tag:"NodeType_Int", format:'k', IXML_FIELD_INFO(PortSelector, NodeType), format_func:IXmlOutputNoop, end_func:IXmlParserEndNodeType_Int},	// input Int, output none (above does both)
	{ NULL }
};

static void PortSelectorXmlFormatAttr(IXmlOutputState_t *state, void *data)
{
	PortSelector *portselp = (PortSelector *)data;

	// perl and other tools need unique id
	IXmlOutputPrint(state, " id=\"0x%016"PRIx64"\"", (uint64)(uintn)portselp);
}

static void PortSelectorXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputOptionalStruct(state, tag, (PortSelector*)data, PortSelectorXmlFormatAttr, PortSelectorFields);
}

static void PortSelectorXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	PortSelector *portselp = (PortSelector*)object;
	ExpectedLink *elinkp = (ExpectedLink*)parent;

	if (! valid)
		goto invalid;
	if (! elinkp->portselp1)
		elinkp->portselp1 = portselp;
	else  if (! elinkp->portselp2)
		elinkp->portselp2 = portselp;
	else {
		IXmlParserPrintError(state, "Too many Ports specified for Link, ignoring: %s\n",
								FormatPortSelector(portselp));
		goto badport;
	}
	return;

badport:
invalid:
	PortSelectorFree(portselp);
	return;
}

/****************************************************************************/
/* Link Input/Output functions */

static void LinkXmlOutputCableData(IXmlOutputState_t *state, const char *tag, void *data)
{
	CableDataXmlOutput(state, tag, &((ExpectedLink*)data)->CableData);
}

static void LinkXmlOutputPortSelectors(IXmlOutputState_t *state, const char *tag, void *data)
{
	PortSelectorXmlOutput(state, tag, ((ExpectedLink*)data)->portselp1);
	PortSelectorXmlOutput(state, tag, ((ExpectedLink*)data)->portselp2);
}

/* <Link> description */
static IXML_FIELD LinkFields[] = {
	{ tag:"Rate", format:'k', IXML_FIELD_INFO(ExpectedLink, expected_rate), format_func:IXmlOutputOptionalRate, end_func:IXmlParserEndRate },	// input str output both
	{ tag:"Rate_Int", format:'k', IXML_FIELD_INFO(ExpectedLink, expected_rate), format_func:IXmlOutputNoop, end_func:IXmlParserEndRate_Int},	// input Int, output none (above does both)
	{ tag:"MTU", format:'k', IXML_FIELD_INFO(ExpectedLink, expected_mtu), format_func:IXmlOutputOptionalMtu, end_func:IXmlParserEndMtu },
	{ tag:"Internal", format:'u', IXML_FIELD_INFO(ExpectedLink, internal) },
	{ tag:"LinkDetails", format:'p', IXML_P_FIELD_INFO(ExpectedLink, details, LINK_DETAILS_STRLEN) },
	{ tag:"Cable", format:'k', size:sizeof(CableData), format_func:LinkXmlOutputCableData, subfields:CableDataFields, start_func:CableDataXmlParserStart, end_func:CableDataXmlParserEnd }, // structure
	{ tag:"Port", format:'K', size:sizeof(PortSelector), format_func:LinkXmlOutputPortSelectors, subfields:PortSelectorFields, start_func:IXmlParserStartStruct, end_func:PortSelectorXmlParserEnd }, // structure
	{ NULL }
};

static void LinkXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag,  (ExpectedLink*)data, NULL /*ExpectedLinkXmlFormatAttr*/ , LinkFields);
}

static void *LinkXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	ExpectedLink *elinkp = (ExpectedLink*)MemoryAllocate2AndClear(sizeof(ExpectedLink), IBA_MEM_FLAG_PREMPTABLE, MYTAG);

	if (! elinkp) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	ListItemInitState(&elinkp->ExpectedLinksEntry);
	QListSetObj(&elinkp->ExpectedLinksEntry, elinkp);

	return elinkp;
}

static void ResolvePorts(FabricData_t *fabricp, ExpectedLink *elinkp)
{
	NodeData *nodep1, *nodep2;
	PortData *portp1, *portp2;
	PortSelMatchLevel_t matchLevel1, matchLevel2;
	uint8 linkMatchLevel;

	// First resolve the two Port Selectors in elinkp to ports in the fabric
	ResolvePortSelector(fabricp, elinkp->portselp1, &nodep1, &portp1, &matchLevel1);
	ResolvePortSelector(fabricp, elinkp->portselp2, &nodep2, &portp2, &matchLevel2);
	linkMatchLevel = g_linkMatchLevel[matchLevel1][matchLevel2];

	if (! portp1 && ! portp2) {
		// we can't resolve either side
		// will report as missing link during verification
		return;
	}

	// we purposely have some fuzzy logic here.  This attempts to associate
	// the link with some link in the fabric.  This permits the verification
	// to better report links which are changed by moving one end of a cable.
	// if only 1 side of link is specified, we assume any neighbor is acceptable
	// we will check other details (NodeType, etc) during verification
	if (portp1 && ! isCompletePortSelector(elinkp->portselp2)) {
		portp2 = portp1->neighbor;
	} else if (portp2 && ! isCompletePortSelector(elinkp->portselp1)) {
		portp1 = portp2->neighbor;
	}

	// if one side resolved to a port and other resolved to neighbor node
	// we treat this as a match.  Hence we can identify changed links
	// and also can handle incomplete specification of selector for "other side"
	if (portp1 && ! portp2 && nodep2 && portp1->neighbor && portp1->neighbor->nodep == nodep2) {
		portp2 = portp1->neighbor;
	}
	if (portp2 && ! portp1 && nodep1 && portp2->neighbor && portp2->neighbor->nodep == nodep1) {
		portp1 = portp2->neighbor;
	}

	// If portp1 and/or portp2 already have a good elinkp, evaluate completeness
	// of match and associate ports with the "better" matching elinkp.
	if (portp1 && portp1->elinkp && portp2 && portp2->elinkp
		&& portp1->elinkp == portp2->elinkp) {

		ExpectedLink *elinkp2 = portp1->elinkp;
 		// we have a potentially duplicate match.  This elinkp or elinkp2
 		// match could be due to fuzzy logic or a duplicate in input.
 		// if one elinkp is due to fuzzy logic and other is due to exact match
 		// the exact match wins.  Hence preventing incorrect reporting of
 		// a missing link such as when a CA has both ports connected and 1 port
 		// goes down.
		// Hence we compare the quality of the match and pick the better one.
 		// If both are at the same level of match, associate both links with
 		// same ports and leave ports as is and we will report duplicate/changed
		if (elinkp2->matchLevel > linkMatchLevel) {
			// elinkp2 is a better match, leave elinkp unassociated so
			// reported as a missing link
			return;
		} else if (elinkp2->matchLevel < linkMatchLevel) {
			// new match is better, unlink old from elinkp2
			// will associate ports below
			portp1->elinkp = portp2->elinkp = NULL;
			elinkp2->portp1 = elinkp2->portp2 = NULL;
			elinkp2->matchLevel = 0;
		} else {
			// equal quality of match, associate ports below and will report as
			// a potential duplicate in input
		}
	}
	elinkp->portp1 = portp1;
	elinkp->portp2 = portp2;
	elinkp->matchLevel = linkMatchLevel;

	// if we have properly resolved both ports, we update PortData
	// if this resolves to ports which already have a PortData->elinkp, we do
	// report that during verification and leave elinkp pointing to port
	// but Port will not point to the duplicate elinkp
	// during verify we will check for each link that
	// elinkp->portp1->elinkp==elinkp && elinkp->portp2->elinkp==elinkp
	// those which do not (or have NULL portp1/2) are dup, missing, etc
	if (portp1 && portp2 && portp1->neighbor == portp2
		&& ! portp1->elinkp && ! portp2->elinkp) {
		// we resolved properly to both sides of a link */
		ASSERT(portp2->neighbor == portp1);
		elinkp->portp1->elinkp = elinkp;
		elinkp->portp2->elinkp = elinkp;
	}
}

static void LinkXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	ExpectedLink *elinkp = (ExpectedLink*)object;
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (! valid)
		goto invalid;
	ResolvePorts(fabricp, elinkp);
	QListInsertTail(&fabricp->ExpectedLinks, &elinkp->ExpectedLinksEntry);
	return;

invalid:
	ExpectedLinkFree(fabricp, elinkp);
	return;
}


static IXML_FIELD LinksFields[] = {
	{ tag:"Link", format:'K', format_func: LinkXmlOutput, subfields:LinkFields, start_func:LinkXmlParserStart, end_func:LinkXmlParserEnd }, // structure
	{ NULL }
};

/****************************************************************************/
/* Overall Links lists Input/Output functions */

static void Xml2PrintAllLinks(IXmlOutputState_t *state, const char *tag, void *data)
{
	LIST_ITEM *p;
	FabricData_t *fabricp = (FabricData_t *)IXmlOutputGetContext(state);

	IXmlOutputStartAttrTag(state, tag, NULL, NULL);
	for (p=QListHead(&fabricp->ExpectedLinks); p != NULL; p = QListNext(&fabricp->ExpectedLinks, p)) {
		ExpectedLink *elinkp = (ExpectedLink *)QListObj(p);
#if 0
		if (! CompareLinkPoint(elinkp, info->focus))
			continue;
#endif
		LinkXmlOutput(state, "Link", elinkp);
	}
	IXmlOutputEndTag(state, tag);
}

static void *LinkSummaryXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (fabricp->flags & (FF_EXPECTED_LINKS|FF_EXPECTED_EXTLINKS)) {
		IXmlParserPrintError(state, "Only one of <LinkSummary> or <ExtLinkSummary> can be supplied\n");
		return NULL;
	}
	return parent;
}

static void LinkSummaryXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (! valid)
		goto invalid;
	fabricp->flags |= FF_EXPECTED_LINKS;
	return;

invalid:
	return;
}

static void *ExternalLinkSummaryXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (fabricp->flags & (FF_EXPECTED_LINKS|FF_EXPECTED_EXTLINKS)) {
		IXmlParserPrintError(state, "Only one of <LinkSummary> or <ExtLinkSummary> can be supplied\n");
		return NULL;
	}
	return NULL;
}

static void ExternalLinkSummaryXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (! valid)
		goto invalid;
	fabricp->flags |= FF_EXPECTED_EXTLINKS;
	return;

invalid:
	return;
}


/****************************************************************************/
/* Node Input/Output functions */

/* <Node> fields */
static IXML_FIELD ExpectedNodeFields[] = {
	{ tag:"NodeGUID", format:'h', IXML_FIELD_INFO(ExpectedNode, NodeGUID) },
	{ tag:"NodeDesc", format:'p', IXML_P_FIELD_INFO(ExpectedNode, NodeDesc, STL_NODE_DESCRIPTION_ARRAY_SIZE) },
	{ tag:"NodeDetails", format:'p', IXML_P_FIELD_INFO(ExpectedNode, details, NODE_DETAILS_STRLEN) },
	{ NULL }
};

static void ExpectedNodeXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag,  (ExpectedNode*)data, NULL /*ExpectedNodeXmlFormatAttr*/ , ExpectedNodeFields);
}

static void *ExpectedNodeXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	ExpectedNode *enodep = (ExpectedNode*)MemoryAllocate2AndClear(sizeof(ExpectedNode), IBA_MEM_FLAG_PREMPTABLE, MYTAG);

	if (! enodep) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	ListItemInitState(&enodep->ExpectedNodesEntry);
	QListSetObj(&enodep->ExpectedNodesEntry, enodep);

	return enodep;
}

// resolve the Expected Node to a node in fabric
static void ResolveNode(FabricData_t *fabricp, ExpectedNode *enodep)
{
	NodeData *nodep = NULL;

	if (! enodep)
		return;
	// order of preference if all specified is: NodeGUID, Desc
	// This way if Node Descriptions have not been fully setup and there
	// are duplicates, a "opareport -o brnodes -x" can still be used
	// as basic template for topology input
	// Valid combos in order of consideration:
	// 		NodeGUID
	// 		NodeDesc
	// presently:
	//		NodeGuid, NodeDesc - NodeDesc is ignored
	//		NodeType is ignored
	if (enodep->NodeGUID) {
		nodep = FindNodeGuid(fabricp, enodep->NodeGUID);
	}
	if (! nodep && enodep->NodeDesc) {
		nodep = LookupNodeName(fabricp, enodep->NodeDesc);
	}

	enodep->nodep = nodep;
	// if we have properly resolved the node, we update NodeData
	// if this resolves to a node which already have a NodeData->enodep, we do
	// report that during verification and leave enodep pointing to node
	// but Node will not point to the duplicate enodep
	// during verify we will check for each node that
	// enodep->nodep->enodep==enodep
	// those which do not (or have NULL nodep) are dup, missing, etc
	if (nodep && ! nodep->enodep) {
		nodep->enodep = enodep;
	}
}
	
static void ExpectedCAXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	ExpectedNode *enodep = (ExpectedNode*)object;
	FabricData_t *fabricp = IXmlParserGetContext(state);

	enodep->NodeType = STL_NODE_FI;
	if (! valid)
		goto invalid;
	ResolveNode(fabricp, enodep);
	QListInsertTail(&fabricp->ExpectedFIs, &enodep->ExpectedNodesEntry);
	return;

invalid:
	ExpectedNodeFree(enodep, &fabricp->ExpectedFIs);
	return;
}

static void ExpectedSWXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	ExpectedNode *enodep = (ExpectedNode*)object;
	FabricData_t *fabricp = IXmlParserGetContext(state);

	enodep->NodeType = STL_NODE_SW;
	if (! valid)
		goto invalid;
	ResolveNode(fabricp, enodep);
	QListInsertTail(&fabricp->ExpectedSWs, &enodep->ExpectedNodesEntry);
	return;

invalid:
	ExpectedNodeFree(enodep, &fabricp->ExpectedSWs);
	return;
}

static IXML_FIELD FIsFields[] = {
	{ tag:"Node", format:'k', format_func: ExpectedNodeXmlOutput, subfields:ExpectedNodeFields, start_func:ExpectedNodeXmlParserStart, end_func:ExpectedCAXmlParserEnd }, // structure
	{ NULL }
};

static IXML_FIELD SWsFields[] = {
	{ tag:"Node", format:'k', format_func: ExpectedNodeXmlOutput, subfields:ExpectedNodeFields, start_func:ExpectedNodeXmlParserStart, end_func:ExpectedSWXmlParserEnd }, // structure
	{ NULL }
};

/****************************************************************************/
/* SM Input/Output functions */

/* PortNum needs special handling since have gotPortNum flag to maintain */
static void ExpectedSMXmlOutputPortNum(IXmlOutputState_t *state, const char *tag, void *data)
{
	ExpectedSM *esmp = (ExpectedSM*)data;
	if (esmp->gotPortNum)
		IXmlOutputUint(state, tag, esmp->PortNum);
}

static void ExpectedSMXmlParserEndPortNum(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;
	
	if (IXmlParseUint8(state, content, len, &value)) {
		((ExpectedSM *)object)->PortNum = value;
		((ExpectedSM *)object)->gotPortNum = 1;
	}
}

/* <SM> fields */
static IXML_FIELD ExpectedSMFields[] = {
	{ tag:"NodeGUID", format:'h', IXML_FIELD_INFO(ExpectedSM, NodeGUID) },
	{ tag:"PortGUID", format:'h', IXML_FIELD_INFO(ExpectedSM, PortGUID) },
	{ tag:"PortNum", format:'k', format_func: ExpectedSMXmlOutputPortNum, end_func:ExpectedSMXmlParserEndPortNum },
	{ tag:"NodeDesc", format:'p', IXML_P_FIELD_INFO(ExpectedSM, NodeDesc, STL_NODE_DESCRIPTION_ARRAY_SIZE) },
	{ tag:"SMDetails", format:'p', IXML_P_FIELD_INFO(ExpectedSM, details, SM_DETAILS_STRLEN) },
	{ tag:"NodeType", format:'k', IXML_FIELD_INFO(ExpectedSM, NodeType), format_func:IXmlOutputOptionalNodeType, end_func:IXmlParserEndNodeType },	// input str output both
	{ tag:"NodeType_Int", format:'k', IXML_FIELD_INFO(ExpectedSM, NodeType), format_func:IXmlOutputNoop, end_func:IXmlParserEndNodeType_Int},	// input Int, output none (above does both)
	{ NULL }
};

static void ExpectedSMXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag,  (ExpectedSM*)data, NULL /*ExpectedSMXmlFormatAttr*/ , ExpectedSMFields);
}

static void *ExpectedSMXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	ExpectedSM *esmp = (ExpectedSM*)MemoryAllocate2AndClear(sizeof(ExpectedSM), IBA_MEM_FLAG_PREMPTABLE, MYTAG);

	if (! esmp) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	ListItemInitState(&esmp->ExpectedSMsEntry);
	QListSetObj(&esmp->ExpectedSMsEntry, esmp);

	return esmp;
}

// resolve the Expected SM to a node in fabric
static void ResolveSM(FabricData_t *fabricp, ExpectedSM *esmp)
{
	SMData *smp = NULL;
	NodeData *nodep = NULL;
	PortData *portp = NULL;

	if (! esmp)
		return;
	// order of preference if all specified is: NodeGUID, PortGUID, Desc
	// This way if Node Descriptions have not been fully setup and there
	// are duplicates, a "opareport -o brnodes -x" can still be used
	// as basic template for topology input
	// Valid combos in order of consideration:
	// 		NodeGUID, PortNum
	// 		NodeGUID, PortGUID - only if PortGUID on same node as NodeGUID
	// 		NodeGUID (fully resolves to port if only 1 active port on node or port 0 on a Switch)
	// 		NodeDesc, PortNum
	// 		NodeDesc, PortGUID - only if PortGUID on same node as NodeDesc
	// 		NodeDesc (fully resolves to port if only 1 active port on node or port 0 on a Switch)
	// 		PortGUID, PortNum (limited use)
	// 		PortGUID
	// presently:
	//		NodeGuid, NodeDesc - NodeDesc is ignored
	//		NodeGuid/Desc, PortNum, PortGUID - PortGUID is ignored
	//		NodeType is ignored
	if (esmp->NodeGUID) {
		nodep = FindNodeGuid(fabricp, esmp->NodeGUID);
	}
	if (! nodep && esmp->NodeDesc) {
		nodep = LookupNodeName(fabricp, esmp->NodeDesc);
	}
	if (! nodep && esmp->PortGUID) {
		portp = FindPortGuid(fabricp, esmp->PortGUID);
		if (portp)
			nodep = portp->nodep;
	}
	if (nodep && esmp->gotPortNum) {
		PortData *p = FindNodePort(nodep, esmp->PortNum);
		if (p)
			portp = p;	// overrides PortGUID
	}
	if (nodep && ! portp && esmp->PortGUID) {
		PortData *p = FindPortGuid(fabricp, esmp->PortGUID);
		if (p && p->nodep == nodep)
			portp = p;
	}
	if (nodep && ! portp) {
		// if switch, default to port 0
		if (nodep->NodeInfo.NodeType == STL_NODE_SW)
			portp = FindNodePort(nodep, 0);
		// if only 1 active port, default to it
		if  (! portp && cl_qmap_count(&nodep->Ports) == 1) {
			portp = PARENT_STRUCT(cl_qmap_head(&nodep->Ports), PortData, NodePortsEntry);
		}
	}

	if (! portp)
		return;
	smp = FindSMPort(fabricp, portp->PortGUID);

	esmp->smp = smp;
	// if we have properly resolved the SM, we update SMData
	// if this resolves to a SM which already have a SMData->esmp, we do
	// report that during verification and leave esmp pointing to SM
	// but SM will not point to the duplicate esmp
	// during verify we will check for each SM that
	// esmp->smp->esmp==esmp
	// those which do not (or have NULL smp) are dup, missing, etc
	if (smp && ! smp->esmp) {
		smp->esmp = esmp;
	}
}
	
static void ExpectedSMXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	ExpectedSM *esmp = (ExpectedSM*)object;
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (! valid)
		goto invalid;
	ResolveSM(fabricp, esmp);
	QListInsertTail(&fabricp->ExpectedSMs, &esmp->ExpectedSMsEntry);
	return;

invalid:
	ExpectedSMFree(fabricp, esmp);
	return;
}

static IXML_FIELD SMsFields[] = {
	{ tag:"SM", format:'k', format_func: ExpectedSMXmlOutput, subfields:ExpectedSMFields, start_func:ExpectedSMXmlParserStart, end_func:ExpectedSMXmlParserEnd }, // structure
	{ NULL }
};

/****************************************************************************/
/* Overall Nodes lists Input/Output functions */

static void Xml2PrintAllType(IXmlOutputState_t *state, const char *listtag, const char *itemtag, QUICK_LIST *listp)
{
	LIST_ITEM *p;

	IXmlOutputStartAttrTag(state, listtag, NULL, NULL);
	for (p=QListHead(listp); p != NULL; p = QListNext(listp, p)) {
		ExpectedNode *enodep = (ExpectedNode *)QListObj(p);
#if 0
		if (! CompareNodePoint(nodep, info->focus))
			continue;
#endif
		ExpectedNodeXmlOutput(state, itemtag, enodep);
	}
	IXmlOutputEndTag(state, listtag);
}

static void Xml2PrintAllSMs(IXmlOutputState_t *state, const char *tag, void *data)
{
	LIST_ITEM *p;
	FabricData_t *fabricp = (FabricData_t *)IXmlOutputGetContext(state);

	IXmlOutputStartAttrTag(state, tag, NULL, NULL);
	for (p=QListHead(&fabricp->ExpectedSMs); p != NULL; p = QListNext(&fabricp->ExpectedSMs, p)) {
		ExpectedSM *esmp = (ExpectedSM *)QListObj(p);
#if 0
		if (! CompareSmPoint(smp, info->focus))
			continue;
#endif
		ExpectedSMXmlOutput(state, "SM", esmp);
	}
	IXmlOutputEndTag(state, tag);
}

static void Xml2PrintAllNodes(IXmlOutputState_t *state, const char *tag, void *data)
{
	FabricData_t *fabricp = (FabricData_t *)IXmlOutputGetContext(state);

	IXmlOutputStartAttrTag(state, tag, NULL, NULL);

	Xml2PrintAllType(state, "FIs", "Node", &fabricp->ExpectedFIs);
	Xml2PrintAllType(state, "Switches", "Node", &fabricp->ExpectedSWs);
	Xml2PrintAllSMs(state, "SMs", &fabricp->ExpectedSMs);

	IXmlOutputEndTag(state, tag);
}

static IXML_FIELD NodesFields[] = {
	{ tag:"FIs", format:'k', subfields:FIsFields, }, // list
	{ tag:"Switches", format:'k', subfields:SWsFields, }, // list
	{ tag:"SMs", format:'k', subfields:SMsFields, }, // list
	{ NULL }
};

static void *NodesXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (fabricp->flags & FF_EXPECTED_NODES)
		IXmlParserPrintError(state, "Only one <Nodes> list can be supplied\n");
	return NULL;
}

static void NodesXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (! valid)
		goto invalid;
	fabricp->flags |= FF_EXPECTED_NODES;
	return;

invalid:
	return;
}

/****************************************************************************/
/* Overall topology Input/Output functions */

/* only used for input parsing */
static IXML_FIELD TopologyFields[] = {
		// allow opareport -o brnodes -x -d 1 style input for Nodes and SMs
	{ tag:"LinkSummary", format:'k', subfields:LinksFields, start_func:LinkSummaryXmlParserStart, end_func:LinkSummaryXmlParserEnd }, // list
	{ tag:"ExternalLinkSummary", format:'k', subfields:LinksFields, start_func:ExternalLinkSummaryXmlParserStart, end_func:ExternalLinkSummaryXmlParserEnd }, // list
	{ tag:"Nodes", format:'k', subfields:NodesFields, start_func:NodesXmlParserStart, end_func:NodesXmlParserEnd }, // list
//TBD - IOCs?
	{ NULL }
};

static void *TopologyXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	// we generate unixtime attribute on output, but ignore it on input
	return NULL;
}

static void TopologyXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	FabricData_t *fabricp = IXmlParserGetContext(state);

	if (! valid)
		goto invalid;
	// only ExtLinks or Links is allowed, ParserStart for these tags checks
	ASSERT(! ((fabricp->flags & (FF_EXPECTED_LINKS|FF_EXPECTED_EXTLINKS))
				== (FF_EXPECTED_LINKS|FF_EXPECTED_EXTLINKS)));
	return;

invalid:
	// free parsed data
	ExpectedLinkFreeAll(fabricp);
	ExpectedNodesFreeAll(&fabricp->ExpectedFIs);
	ExpectedNodesFreeAll(&fabricp->ExpectedSWs);
	ExpectedSMsFreeAll(fabricp);
}

static IXML_FIELD TopLevelFields[] = {
	// "Topology" is the prefered input main tag
	// however we also allow Report, this permits a -o links and/or -o brnodes
	// report to be edited and used as input as the topology
	{ tag:"Topology", format:'k', subfields:TopologyFields, start_func:TopologyXmlParserStart, end_func:TopologyXmlParserEnd }, // structure
	{ tag:"Report", format:'k', subfields:TopologyFields, start_func:TopologyXmlParserStart, end_func:TopologyXmlParserEnd }, // structure
	{ NULL }
};

#if TBD
static void TopologyInfoXmlFormatAttr(IXmlOutputState_t *state, void *data)
{
	TopologyOutputInfo_t *info = (TopologyOutputInfo_t *)IXmlOutputGetContext(state);
	char datestr[80] = "";
	int i;

	strftime(datestr, sizeof(datestr), "%c", localtime(&info->time));// %+ or %c or %x %X
	IXmlOutputPrint(state, " date=\"%s\" unixtime=\"%ld\" options=\"", datestr, info->time);
	for (i=1; i<info->argc; i++)
		IXmlOutputPrint(state, "%s%s", i>1?" ":"", info->argv[i]);
	IXmlOutputPrint(state, "\"");
}
#endif

static void Xml2PrintAll(IXmlOutputState_t *state, const char *tag, void *data)
{
#if 0
	TopologyOutputInfo_t *info = (TopologyOutputInfo_t *)IXmlOutputGetContext(state);
#endif

	IXmlOutputStartAttrTag(state, tag, NULL, NULL /*TopologyInfoXmlFormatAttr*/);

	Xml2PrintAllLinks(state, "LinkSummary", NULL);
	Xml2PrintAllNodes(state, "Nodes", NULL);


	IXmlOutputEndTag(state, tag);
}

void Xml2PrintTopology(FILE *file, FabricData_t *fabricp)
{
	IXmlOutputState_t state;

	/* using SERIALIZE with no indent makes output less pretty but 1/2 the size */
	//if (FSUCCESS != IXmlOutputInit(&state, file, 0, IXML_OUTPUT_FLAG_SERIALIZE, info))
	if (FSUCCESS != IXmlOutputInit(&state, file, 4, IXML_OUTPUT_FLAG_NONE, fabricp))
		goto fail;
	
	Xml2PrintAll(&state, "Topology", NULL);

	IXmlOutputDestroy(&state);

	return;

fail:
	return;
}

#ifndef __VXWORKS__

FSTATUS Xml2ParseTopology(const char *input_file, int quiet, FabricData_t *fabricp)
{
	unsigned tags_found, fields_found;
	const char *filename=input_file;

	if(fabricp == NULL || fabricp->AllNodes.state != CL_INITIALIZED) {
		if (!quiet) ProgressPrint(TRUE, "Error: input FabricData_t was null or uninitialized!");
		return FERROR;
	}

	if (strcmp(input_file, "-") == 0) {
		if (! quiet) ProgressPrint(TRUE, "Parsing stdin...");
		filename = "stdin";
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
	return FSUCCESS;
}

#else

FSTATUS Xml2ParseTopology(const char *input_file, int quiet, FabricData_t *fabricp, XML_Memory_Handling_Suite* memsuite)
{
	unsigned tags_found, fields_found;
	const char *filename=input_file;

	if(fabricp == NULL || fabricp->AllNodes.state != CL_INITIALIZED) {
		if (!quiet) ProgressPrint(TRUE, "Error: input FabricData_t was null or uninitialized!");
		return FERROR;
	}

	if (strcmp(input_file, "-") == 0) {
		if (! quiet) ProgressPrint(TRUE, "Parsing stdin...");
		filename = "stdin";
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
	if (! quiet) ProgressPrint(TRUE, "Done Parsing");
	return FSUCCESS;
}

#endif

// indicate if the given enodep matches the portselp
// if provided NodeDesc, NodeGuid and NodeType must match
// if neither NodeDesc nor NodeGuid is provided, returns FALSE
static boolean MatchExpectedNode(ExpectedNode *enodep, PortSelector *portselp)
{
	boolean match = FALSE;
	if (enodep->NodeDesc && portselp->NodeDesc) {
		match = (0 == strcmp(enodep->NodeDesc, portselp->NodeDesc));
		if (enodep->NodeGUID && portselp->NodeGUID) {
			match &= (enodep->NodeGUID == portselp->NodeGUID);
		}
	} else if (enodep->NodeGUID && portselp->NodeGUID) {
		match = (enodep->NodeGUID == portselp->NodeGUID);
	}
	if (enodep->NodeType && portselp->NodeType)
		match &= (enodep->NodeType == portselp->NodeType);
	return match;
}

// This routine takes the ExpectedLinks and attempts to resolve each
// against a pair of ExpectedNodes, then fills in ExpectedLinks.enodep1, enodep2
// By design this does not look at the NodeData (that is already handled on
// parsing and generates ExpectedLinks.portp1,portp2).  Hence this is useful to
// help build the graph for applications which only use the topology file.
// Given the limited information in ExpectedNodes, the matching is based on
// NodeDesc and/or NodeGUID matching as well as NodeType.
void ResolveExpectedLinks(FabricData_t *fabricp, int quiet)
{
	LIST_ITEM *q;
	LIST_ITEM *p;
	uint32 input_checked = 0;
	uint32 resolved = 0;
	uint32 bad_input = 0;
	uint32 ix = 0;

	if (! quiet) ProgressPrint(TRUE, "Resolving Links...");
	for (q=QListHead(&fabricp->ExpectedLinks); q != NULL; q = QListNext(&fabricp->ExpectedLinks, q)) {
		ExpectedLink *elinkp = (ExpectedLink *)QListObj(q);
		uint32 ends = 0;	// # ends of a link we have found enodep for

		if (! quiet && (ix++ % PROGRESS_FREQ) == 0) {
			ProgressPrint(FALSE, "Resolved %6d of %6d Links...",
				ix, QListCount(&fabricp->ExpectedLinks));
		}

		if (! elinkp->portselp1 || ! elinkp->portselp2) {
			fprintf(stderr, "Skipping Link, incomplete\n");
			bad_input++;
			continue;
		}
		if ((! elinkp->portselp1->NodeDesc && ! elinkp->portselp1->NodeGUID)
			|| (! elinkp->portselp2->NodeDesc && ! elinkp->portselp2->NodeGUID)) {
			fprintf(stderr, "Skipping Link, unspecified NodeDesc and NodeGUID\n");
			bad_input++;
			continue;
		}
		input_checked++;
		// find enodep for both ends of link
		for (p=QListHead(&fabricp->ExpectedSWs); ends < 2 && p != NULL; p = QListNext(&fabricp->ExpectedSWs, p)) {
			ExpectedNode *enodep = (ExpectedNode *)QListObj(p);

			if (! elinkp->enodep1 && MatchExpectedNode(enodep, elinkp->portselp1)) {
				elinkp->enodep1 = enodep;
				ends++;
			}
			if (! elinkp->enodep2 && MatchExpectedNode(enodep, elinkp->portselp2)) {
				elinkp->enodep2 = enodep;
				ends++;
			}
		}
		for (p=QListHead(&fabricp->ExpectedFIs); ends < 2 && p != NULL; p = QListNext(&fabricp->ExpectedFIs, p)) {
			ExpectedNode *enodep = (ExpectedNode *)QListObj(p);

			if (! elinkp->enodep1 && MatchExpectedNode(enodep, elinkp->portselp1)) {
				elinkp->enodep1 = enodep;
				ends++;
			}
			if (! elinkp->enodep2 && MatchExpectedNode(enodep, elinkp->portselp2)) {
				elinkp->enodep2 = enodep;
				ends++;
			}
		}
		if (ends >= 2)
			resolved++;
	}
	if (! quiet) ProgressPrint(TRUE, "Done Resolving Links");
	if (! quiet || bad_input)
		fprintf(stderr, "%u of %u Input Links Checked, %u Resolved, %u Bad Input Skipped\n",
				input_checked, QListCount(&fabricp->ExpectedLinks),
				resolved, bad_input);
	return;
}
