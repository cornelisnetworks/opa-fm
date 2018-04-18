/* BEGIN_ICS_COPYRIGHT7 ****************************************

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
// Beware, can't call this more than twice in a single function argument list
// (such as printf)
// This is also not thread safe
static const char *FormatPortSelector(PortSelector *portselp)
{
	static char format1[256];
	static char format2[256];
	static int f_num = 0;
	int offset = 0;
	char *format;

	// This trick allows us to alternate which static char we use and return
	// hence two calls in the same function argument list (eg. printf) work
	// correctly
	if (f_num) {
		format=format2;
		f_num=0;
	} else {
		format=format1;
		f_num=1;
	}

	if (! portselp) {
		offset += sprintf(&format[offset], "%*sunspecified", offset?1:0, "");
		return format;
	}
	if (portselp->NodeDesc) {
		offset += sprintf(&format[offset], "%*sNodeDesc: %s", offset?1:0, "", portselp->NodeDesc);
	}
	if (portselp->NodeGUID) {
		offset += sprintf(&format[offset], "%*sNodeGUID: 0x%016"PRIx64, offset?1:0, "", portselp->NodeGUID);
	}
	if (portselp->gotPortNum) {
		offset += sprintf(&format[offset], "%*sPortNum: %u", offset?1:0, "", portselp->PortNum);
	}
	if (portselp->PortGUID) {
		offset += sprintf(&format[offset], "%*sPortGUID: 0x%016"PRIx64, offset?1:0, "", portselp->PortGUID);
	}
	if (portselp->NodeType) {
		offset += sprintf(&format[offset], "%*sNodeType: %s", offset?1:0, "",
			StlNodeTypeToText(portselp->NodeType));
	}
	if (portselp->details) {
		offset += sprintf(&format[offset], "%*sPortDetails: %s", offset?1:0, "",
			portselp->details);
	}
	if (! offset)
		offset += sprintf(&format[offset], "%*sunspecified", offset?1:0, "");
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
		if (value > 254) {
			IXmlParserPrintError(state, "PortNum must be in range [0,254]");
		} else {
			((PortSelector *)object)->PortNum = value;
			((PortSelector *)object)->gotPortNum = 1;
		}
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

	elinkp->lineno = (uint64)XML_GetCurrentLineNumber(state->parser);

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

static void ExpectedPortXmlParserEndPortNum(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value)) {
		if (value > 254) {
			IXmlParserPrintError(state, "PortNum must be in range [0,254]");
		} else {
			((ExpectedPort *)object)->PortNum = value;
		}
	}
}

static void ExpectedPortXmlParserEndLmc(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	uint8 value;

	if (IXmlParseUint8(state, content, len, &value))
		((ExpectedPort *)object)->lmc = value;
}

static IXML_FIELD ExpectedPortFields[] = {
	{ tag:"PortNum", format:'u', end_func:ExpectedPortXmlParserEndPortNum },
	{ tag:"LID", format:'h', IXML_FIELD_INFO(ExpectedPort, lid) },
	{ tag:"LMC", format:'u', end_func:ExpectedPortXmlParserEndLmc },
	{ tag:"PortGUID", format:'h', IXML_FIELD_INFO(ExpectedPort, PortGuid) },
	{ NULL }
};


/****************************************************************************/
/* Node Input/Output functions */

// for use in error messages, returns pointer to static string
// Beware, can't call this more than twice in a single function argument list
// (such as printf)
// This is also not thread safe
static const char *FormatExpectedNode(ExpectedNode *enodep)
{
	static char format1[256];
	static char format2[256];
	static int f_num = 0;
	int offset = 0;
	char *format;

	// This trick allows us to alternate which static char we use and return
	// hence two calls in the same function argument list (eg. printf) work
	// correctly
	if (f_num) {
		format=format2;
		f_num=0;
	} else {
		format=format1;
		f_num=1;
	}


	if (! enodep) {
		offset += sprintf(&format[offset], "%*sunspecified", offset?1:0, "");
		return format;
	}
	if (enodep->NodeDesc) {
		offset += sprintf(&format[offset], "%*sNodeDesc: %s", offset?1:0, "", enodep->NodeDesc);
	}
	if (enodep->NodeGUID) {
		offset += sprintf(&format[offset], "%*sNodeGUID: 0x%016"PRIx64, offset?1:0, "", enodep->NodeGUID);
	}
	if (enodep->NodeType) {
		offset += sprintf(&format[offset], "%*sNodeType: %s", offset?1:0, "",
			StlNodeTypeToText(enodep->NodeType));
	}
	if (enodep->details) {
		offset += sprintf(&format[offset], "%*sNodeDetails: %s", offset?1:0, "",
			enodep->details);
	}
	if (! offset)
		offset += sprintf(&format[offset], "%*sunspecified", offset?1:0, "");
	return format;
}

/* <Node> fields */

static void *ExpectedPortXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	ExpectedPort *eportp = MemoryAllocate2AndClear(sizeof(ExpectedPort),
		IBA_MEM_FLAG_PREMPTABLE, MYTAG);

	if (!eportp) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	return eportp;
}

// check enodep->ports size and grow as needed to accomidate adding portNum
static FSTATUS ExpectedNodePrepareSize(ExpectedNode *enodep, uint8 portNum)
{
	if ((portNum + 1) > enodep->portsSize) {
		int allocCount = MAX((enodep->portsSize == 0 ? 1 : 2 * enodep->portsSize),
			portNum + 1);
		ExpectedPort **newMem = MemoryAllocate2AndClear(allocCount * sizeof(ExpectedPort*),
			IBA_MEM_FLAG_PREMPTABLE, MYTAG);
		if (!newMem) {
			return FINSUFFICIENT_MEMORY;
		}

		if (enodep->ports) {
			memcpy(newMem, enodep->ports, enodep->portsSize * sizeof(ExpectedPort*));
			MemoryDeallocate(enodep->ports);
		}
		enodep->ports = newMem;
		enodep->portsSize = allocCount;
	}
	return FSUCCESS;
}

static FSTATUS ExpectedNodeAddPort(ExpectedNode *enodep, ExpectedPort *eportp)
{
	if (FSUCCESS != ExpectedNodePrepareSize(enodep, eportp->PortNum))
		return FINSUFFICIENT_MEMORY;

	if (enodep->ports[eportp->PortNum] != NULL)
		return FDUPLICATE;

	enodep->ports[eportp->PortNum] = eportp;
	return FSUCCESS;
}

static ExpectedPort *ExpectedNodeGetPort(ExpectedNode *enodep, uint8 portNum)
{
	if(portNum >= enodep->portsSize)
		return NULL;
	else
		return enodep->ports[portNum];
}

static void ExpectedPortXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	ExpectedNode *enodep = (ExpectedNode*)parent;
	ExpectedPort *eportp = (ExpectedPort*)object;

	if (!valid)
		goto invalid;

	switch (ExpectedNodeAddPort(enodep, eportp)) {
	case FSUCCESS:
		break;
	case FINVALID_PARAMETER:
		break;
	case FINSUFFICIENT_MEMORY:
		IXmlParserPrintError(state, "Unable to allocate memory");
		goto invalid;
		break;
	case FDUPLICATE:
		// depending on order of tags, we may not yet know the complete enodep
		IXmlParserPrintError(state, "Duplicate PortNums (%u) found in Node: %s\n", eportp->PortNum, FormatExpectedNode(enodep));
		goto invalid;
		break;
	default:	// unexpected
		// depending on order of tags, we may not yet know the complete enodep
		IXmlParserPrintError(state, "Unable to add port %u to Node: %s\n", eportp->PortNum, FormatExpectedNode(enodep));
		goto invalid;
		break;
	}

	enodep->ports[eportp->PortNum] = eportp;
	return;

invalid:
	MemoryDeallocate(eportp);
}

static IXML_FIELD ExpectedNodeFields[] = {
	{ tag:"NodeGUID", format:'h', IXML_FIELD_INFO(ExpectedNode, NodeGUID) },
	{ tag:"NodeDesc", format:'p', IXML_P_FIELD_INFO(ExpectedNode, NodeDesc, STL_NODE_DESCRIPTION_ARRAY_SIZE) },
	{ tag:"NodeDetails", format:'p', IXML_P_FIELD_INFO(ExpectedNode, details, NODE_DETAILS_STRLEN) },
	{ tag:"Port", format:'k', subfields:ExpectedPortFields, start_func:ExpectedPortXmlParserStart, end_func:ExpectedPortXmlParserEnd },
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

	enodep->lineno = (uint64)XML_GetCurrentLineNumber(state->parser);

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
	
static void ExpectedFIXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	ExpectedNode *enodep = (ExpectedNode*)object;
	FabricData_t *fabricp = IXmlParserGetContext(state);


	enodep->NodeType = STL_NODE_FI;
	if (! valid)
		goto invalid;
	ResolveNode(fabricp, enodep);
	QListInsertTail(&fabricp->ExpectedFIs, &enodep->ExpectedNodesEntry);


	if(enodep->NodeGUID) {
		//Attempts to insert duplicates will not be detected here. Duplicates can be detected later if topology
		//file validation is enabled
		cl_qmap_insert(&fabricp->ExpectedNodeGuidMap, enodep->NodeGUID, &enodep->ExpectedNodeGuidMapEntry);
	}

	return;

invalid:
	ExpectedNodeFree(fabricp, enodep, &fabricp->ExpectedFIs);
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
	
	if(enodep->NodeGUID) {
		//Attempts to insert duplicates will not be detected here. Duplicates can be detected later if topology
		//file validation is enabled
		cl_qmap_insert(&fabricp->ExpectedNodeGuidMap, enodep->NodeGUID, &enodep->ExpectedNodeGuidMapEntry);
	}
	return;

invalid:
	ExpectedNodeFree(fabricp, enodep, &fabricp->ExpectedSWs);
	return;
}

static IXML_FIELD FIsFields[] = {
	{ tag:"Node", format:'k', format_func: ExpectedNodeXmlOutput, subfields:ExpectedNodeFields, start_func:ExpectedNodeXmlParserStart, end_func:ExpectedFIXmlParserEnd }, // structure
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
	ExpectedNodesFreeAll(fabricp, &fabricp->ExpectedFIs);
	ExpectedNodesFreeAll(fabricp, &fabricp->ExpectedSWs);
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

static FSTATUS TopologyValidateLinkPort(FabricData_t *fabricp, ExpectedLink *elinkp, PortSelector *portselp, TopoVal_t validation)
{
	ExpectedNode *enodep;

	if(!portselp){
		// only 1 side will be found, we'll show that side
		fprintf(stderr, "Topology file line %"PRIu64": incomplete link specification, only 1 port provided: %s\n", elinkp->lineno,
				elinkp->portselp1?
					FormatPortSelector(elinkp->portselp1)
					: elinkp->portselp2?
						FormatPortSelector(elinkp->portselp2)
						:""
				);
		return FINVALID_PARAMETER;
	}

	if(portselp->NodeGUID) {
		enodep = FindExpectedNodeByNodeGuid(fabricp, portselp->NodeGUID);
		if(!enodep){
			fprintf(stderr, "Topology file line %"PRIu64": No node found with matching NodeGUID for link port: %s\n", elinkp->lineno, FormatPortSelector(portselp));
			return FNOT_FOUND;	
		} 
		if(portselp->NodeDesc){
			if(enodep->NodeDesc && strcmp(portselp->NodeDesc, enodep->NodeDesc) != 0) {	
				fprintf(stderr, "Topology file line %"PRIu64": Node GUIDs match, but inconsistent NodeDesc for Node: %.*s link port: %s\n",
					elinkp->lineno, NODE_DESCRIPTION_ARRAY_SIZE, enodep->NodeDesc, FormatPortSelector(portselp));
				return FERROR;
			}
		}		
	} else if (portselp->NodeDesc) {
		enodep = FindExpectedNodeByNodeDesc(fabricp,portselp->NodeDesc, portselp->NodeType);
		if(!enodep){
			fprintf(stderr, "Topology file line %"PRIu64": No node found with matching NodeDesc for link port: %s\n",
				elinkp->lineno, FormatPortSelector(portselp));
			return FNOT_FOUND;
		}
	} else {
		fprintf(stderr, "Topology file line %"PRIu64": Link port specification with no NodeGUID or NodeDesc: %s; other side of link: %s\n",
			elinkp->lineno, FormatPortSelector(portselp),
			elinkp->portselp1 == portselp?
				FormatPortSelector(elinkp->portselp2)
				: FormatPortSelector(elinkp->portselp1));
		return FINVALID_PARAMETER;
	}

	if ((enodep->NodeType && portselp->NodeType)
		 && enodep->NodeType != portselp->NodeType) {
		fprintf(stderr, "Topology file line %"PRIu64": Nodes match but inconsistent NodeType for Node: %s link port: %s\n",
				elinkp->lineno, StlNodeTypeToText(portselp->NodeType),
				FormatPortSelector(portselp));
		return FERROR;
	}

	if (! portselp->gotPortNum) {
		fprintf(stderr, "Topology file line %"PRIu64": Link port specification with no PortNum: %s\n", elinkp->lineno, FormatPortSelector(portselp));
		return FINVALID_PARAMETER;
	} else {
		//	store a pointer to the ExpectedLink in the ExpectedNode to allow
		//	for constant time lookup later
		ExpectedPort *eportp = ExpectedNodeGetPort(enodep, portselp->PortNum);
		if (TOPOVAL_STRICT == validation) {
			if(NULL == eportp ) {
				fprintf(stderr, "Topology file line %"PRIu64": Referenced port %u not specified in Node (line %"PRIu64"): %s\n",
					elinkp->lineno, portselp->PortNum,
					enodep->lineno, FormatExpectedNode(enodep));
				return FERROR;
			}
		} else if (NULL == eportp) {
			// did not have port in enode section, we'll add it here
			eportp = (ExpectedPort*)MemoryAllocate2AndClear(sizeof(ExpectedPort), IBA_MEM_FLAG_PREMPTABLE, MYTAG);
			if (NULL == eportp)
				goto unable2add;
			eportp->PortNum = portselp->PortNum;
			if (FSUCCESS != ExpectedNodeAddPort(enodep, eportp)) {
				MemoryDeallocate(eportp);
				// Can't be FDUPLICATE, so must be out of memory
				goto unable2add;
			}
		}
		// found a matching port on enodep
		if(enodep->ports[portselp->PortNum]->elinkp != NULL) {
			fprintf(stderr, "Topology file line %"PRIu64": More than one link to port %u on Node: %s\n", elinkp->lineno, portselp->PortNum, FormatExpectedNode(enodep));
			return FDUPLICATE;
		}
		enodep->ports[portselp->PortNum]->elinkp = elinkp;
	}
	portselp->enodep = enodep;

	return FSUCCESS;

unable2add:
	fprintf(stderr, "Topology file line %"PRIu64": Unable to add port %u on Node: %s\n", elinkp->lineno, portselp->PortNum, FormatExpectedNode(enodep));
	return FINSUFFICIENT_MEMORY;
}


//Do graph search over ExpectedNodes for a given root note, 
//and mark all found as "connected" to the graph
void TopologyValidateLinkHelper(ExpectedNode *enodep)
{
	int i;
	if(!enodep)
		return;
	if(enodep->connected)
		return; //already processsed this node
	enodep->connected = 1;

	//recurse on all nodes with links from this node
	for(i=0; i<enodep->portsSize; i++){
		if(enodep->ports[i] && enodep->ports[i]->elinkp){
			TopologyValidateLinkHelper(enodep->ports[i]->elinkp->portselp1->enodep);	
			TopologyValidateLinkHelper(enodep->ports[i]->elinkp->portselp2->enodep);
		}
	}

	return;
}


FSTATUS TopologyValidateNoLinksDisjoint(FabricData_t *fabricp) 
{
	FSTATUS status = FSUCCESS;
	LIST_ITEM *it;
	//Algorithm:
	//1) Pick any node N in ExpectedSWs
	//2) Do a graph search from N 
	//	2a)For each unique node found mark it as found
	//3) Iterate over all nodes in ExpectedSWs and ExpectedSWs
	//		and if any were not found they are disjoint
	

	ExpectedNode* erootnodep = PARENT_STRUCT(QListHead(&fabricp->ExpectedSWs), ExpectedNode, ExpectedNodesEntry);

	//Handle b2b case
	if(!erootnodep){
		erootnodep = PARENT_STRUCT(QListHead(&fabricp->ExpectedFIs), ExpectedNode, ExpectedNodesEntry);
		if(!erootnodep)
			return FSUCCESS;
	}

	// if we happen to pick a erootnodep which is disconnected from rest of
	// fabric, we will report everything else as disjoint from it
	TopologyValidateLinkHelper(erootnodep);

	for(it = QListHead(&fabricp->ExpectedSWs); it != NULL; it = QListNext(&fabricp->ExpectedSWs, it)) {
		ExpectedNode *enodep = PARENT_STRUCT(it, ExpectedNode, ExpectedNodesEntry);
		if(!enodep->connected){
			fprintf(stderr, "Switch: %s is disjoint from Node: %s\n",
				FormatExpectedNode(enodep), FormatExpectedNode(erootnodep));
			status = FERROR;
		}
	}

	for(it = QListHead(&fabricp->ExpectedFIs); it != NULL; it = QListNext(&fabricp->ExpectedFIs, it)) {
		ExpectedNode *enodep = PARENT_STRUCT(it, ExpectedNode, ExpectedNodesEntry);
		if(!enodep->connected){
			fprintf(stderr, "FI: %s is disjoint from Node: %s\n",
				FormatExpectedNode(enodep), FormatExpectedNode(erootnodep));
			status = FERROR;
		}
	}

	return status;
}

static FSTATUS TopologyValidate(FabricData_t *fabricp, int quiet, TopoVal_t validation)
{
	FSTATUS status = FSUCCESS;
	LIST_ITEM *it;
	int ix=0;
	int resolved = 0;
	int bad_input = 0;
	int input_checked = 0;

	//make sure input file contains at least one link
	if(QListHead(&fabricp->ExpectedLinks) == NULL) {
		fprintf(stderr, "Topology file does not have link definitions\n");
		return FERROR;
	}

	// Validate that all SW0's have matching Node and Port GUIDs.
	// This is meant to catch hand-edited topologies with errorneous switch
	// GUIDs as SW0 is not checked by pre-defined topology.
	for(it = QListHead(&fabricp->ExpectedSWs); it != NULL; it = QListNext(&fabricp->ExpectedSWs, it)) {
		ExpectedNode *enodep = PARENT_STRUCT(it, ExpectedNode, ExpectedNodesEntry);
        	if (enodep->portsSize > 0 && enodep->ports[0] != NULL
				&& enodep->ports[0]->PortGuid && enodep->NodeGUID
				&& enodep->ports[0]->PortGuid != NodeGUIDtoPortGUID(enodep->NodeGUID, 0)) {
			fprintf(stderr, "Topology file line %"PRIu64": mismatched NodeGUID and PortGUID for switch port 0: %s\n", enodep->lineno, FormatExpectedNode(enodep));
			if (TOPOVAL_SOMEWHAT_STRICT <= validation)
				status = FERROR;
		}
	}

	//Validate each link has a matching node entry, and ensure
	//node descriptions and port numbers match
	//This also validates that there is not more than one link to a given
	//(Node,Port) pair
	//Each (ExpectedNode,port) pair that matches a link will have a pointer
	//to that link stored in its ExpectedPort struct, and a pointer to the
	//node stored in the ExpectedLink portselp
	if (! quiet) ProgressPrint(TRUE, "Resolving Links against Nodes...");
	for(it = QListHead(&fabricp->ExpectedLinks); it != NULL; it = QListNext(&fabricp->ExpectedLinks, it)) {
		ExpectedLink* elinkp = PARENT_STRUCT(it, ExpectedLink, ExpectedLinksEntry);
		int ends = 0;
		int bad_end = 0;

		if (! quiet && (ix++ % PROGRESS_FREQ) == 0) {
			ProgressPrint(FALSE, "Resolved %6d of %6d Links...",
				ix, QListCount(&fabricp->ExpectedLinks));
		}
		if (! elinkp->portselp1 && ! elinkp->portselp2) {
			fprintf(stderr, "Topology file line %"PRIu64": empty link specification\n", elinkp->lineno);
			bad_input++;
			continue;
		}
		switch (TopologyValidateLinkPort(fabricp, elinkp, elinkp->portselp1, validation)) {
		case FSUCCESS:
			ends++;
			break;
		case FINVALID_PARAMETER:
			if (TOPOVAL_SOMEWHAT_STRICT <= validation)
				status = FERROR;
			bad_end++;
			break;
		default:
			status = FERROR;
			break;
		}
		switch (TopologyValidateLinkPort(fabricp, elinkp, elinkp->portselp2, validation)) {
		case FSUCCESS:
			ends++;
			break;
		case FINVALID_PARAMETER:
			bad_end++;
			if (TOPOVAL_SOMEWHAT_STRICT <= validation)
				status = FERROR;
			break;
		default:
			status = FERROR;
			break;
		}
		if (bad_end)
			bad_input++;
		else
			input_checked++;
		if (ends >= 2)
			resolved++;
	}
	if (! quiet) ProgressPrint(TRUE, "Done Resolving Links");
	if (! quiet || bad_input)
		fprintf(stderr, "%u of %u Input Links Checked, %u Resolved, %u Bad Input Skipped\n",
				input_checked, QListCount(&fabricp->ExpectedLinks),
				resolved, bad_input);

	// If successful to this point, Validate all nodes are reachable
	if(FSUCCESS == status && TOPOVAL_SOMEWHAT_STRICT <= validation) {
		if(TopologyValidateNoLinksDisjoint(fabricp) != FSUCCESS)
			status = FERROR;
	}

	if (TOPOVAL_SOMEWHAT_STRICT <= validation)
		return status;
	else
		return FSUCCESS;
}

FSTATUS Xml2ParseTopology(const char *input_file, int quiet, FabricData_t *fabricp,
#ifdef __VXWORKS__
	XML_Memory_Handling_Suite* memsuite,
#endif
	TopoVal_t validation)
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
		if (FSUCCESS != IXmlParseFile(stdin, "stdin", IXML_PARSER_FLAG_NONE, TopLevelFields, NULL, fabricp, NULL, NULL, &tags_found, &fields_found
#ifdef __VXWORKS__
																	, memsuite
#endif
																			)) {
			return FERROR;
		}
	} else {
		if (! quiet) ProgressPrint(TRUE, "Parsing %s...", Top_truncate_str(input_file));
		if (FSUCCESS != IXmlParseInputFile(input_file, IXML_PARSER_FLAG_NONE, TopLevelFields, NULL, fabricp, NULL, NULL, &tags_found, &fields_found
#ifdef __VXWORKS__
																	, memsuite
#endif
																			)) {
			return FERROR;
		}
	}
	if (tags_found != 1 || fields_found != 1) {
		fprintf(stderr, "Warning: potentially inaccurate input '%s': found %u recognized top level tags, expected 1\n", filename, tags_found);
	}
	if(TOPOVAL_NONE != validation)
		return TopologyValidate(fabricp, quiet, validation);
	else
		return FSUCCESS;
}
