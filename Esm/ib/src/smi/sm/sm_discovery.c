/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2018, Intel Corporation

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

//===========================================================================//
// FILE NAME
//  sm_discovery.c
//
// DESCRIPTION
//  Data structures and functions used in the initial discovery of the OPA
//  fabric.
//===========================================================================//

#include <time.h>
#include "sm_l.h"
#include "sa_l.h"
#include "sm_dbsync.h"
#include "sm_parallelsweep.h"
#include "sm_discovery.h"

#define PORTINFO_UNAVAILABLE    0x01
#define SWINFO_UNAVAILABLE      0x02
#define NODEDESC_UNAVAILABLE    0x04

extern int		sm_peer_quarantined;
extern Pool_t	sm_pool;

typedef struct {
	boolean new_level_found;	// Used to enforce BFS discovery.
	void *routing_context;		// Passes data for post-process discovery.
	cl_qmap_t dup_map;			// Ensures nodes are only discovered once.
} DiscoveryContext_t;

typedef struct {
	ParallelWorkItem_t item;
	DiscoveryContext_t *ctx;
	cl_map_item_t dup_item; // Used for tracking in-progress nodes.
	uint8_t path[64];       // Used for DR routing.
	Node_t *upstream_nodep;  // Existing node to operate on.
	Port_t *upstream_portp;  // Existing port to operate on.
} DiscoveryWorkItem_t;

static void
_log_predef_field_violation(Topology_t* topop, uint32_t logQuarantineReasons,
	boolean forceLogAsWarn, Node_t *upstreamNodep, Port_t *upstreamPortp, STL_NODE_INFO* nodeInfo,
	STL_NODE_DESCRIPTION* nodeDesc, PortSelector* validationPort)
{
	int nodeDescViolation = logQuarantineReasons & STL_QUARANTINE_REASON_TOPO_NODE_DESC;
	int nodeGuidViolation = logQuarantineReasons & STL_QUARANTINE_REASON_TOPO_NODE_GUID;
	int portGuidViolation = logQuarantineReasons & STL_QUARANTINE_REASON_TOPO_PORT_GUID;
	int undefinedLinkViolation = logQuarantineReasons & STL_QUARANTINE_REASON_TOPO_UNDEFINED_LINK;
	boolean logError = !forceLogAsWarn && // Override and Force to only Log as a Warn as the port will be bounced
		((nodeDescViolation && (sm_config.preDefTopo.fieldEnforcement.nodeDesc == FIELD_ENF_LEVEL_ENABLED)) ||
		(nodeGuidViolation && (sm_config.preDefTopo.fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_ENABLED)) ||
		(portGuidViolation && (sm_config.preDefTopo.fieldEnforcement.portGuid == FIELD_ENF_LEVEL_ENABLED)) ||
		(undefinedLinkViolation && (sm_config.preDefTopo.fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_ENABLED)));

	char expectedNodeDescPrint[128] = {0};
	char expectedNodeGuidPrint[64] = {0};
	char expectedPortGuidPrint[64] = {0};
	char undefinedLinkPrint[64] = {0};
	char foundPortGuidPrint[64] = {0};
	char basePortGuidPrint[64] = {0};

	// We shouldn't log if we don't have any of these
	if (upstreamNodep == NULL || upstreamPortp == NULL || nodeInfo == NULL
			|| nodeDesc == NULL || (validationPort == NULL && !undefinedLinkViolation)) {
		return;
	}

	snprintf(basePortGuidPrint, sizeof(basePortGuidPrint), " PortGUID: "
		FMT_U64", ", upstreamPortp->portData->guid);
	snprintf(foundPortGuidPrint, sizeof(foundPortGuidPrint), " PortGUID: "
		FMT_U64", ", nodeInfo->PortGUID);

	// If we have hit our log threshold, spit out a final message but then stop
	// logging
	if(sm_config.preDefTopo.logMessageThreshold != 0 &&
			topop->preDefLogCounts.totalLogCount == sm_config.preDefTopo.logMessageThreshold) {
		IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: Log message threshold "
			"of %d messages per sweep has been reached. "
			"Suppressing further topology mismatch information.",
			sm_config.preDefTopo.logMessageThreshold);
		topop->preDefLogCounts.totalLogCount++;
	}

	// If we have an undefined link violation, don't log any of the other
	// fields as they would be invalid anyways.
	if (undefinedLinkViolation) {
		nodeDescViolation = 0;
		nodeGuidViolation = 0;
		portGuidViolation = 0;

		if (forceLogAsWarn || sm_config.preDefTopo.fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_WARN) {
			snprintf(undefinedLinkPrint, sizeof(undefinedLinkPrint), " No Link (Warn)");
			topop->preDefLogCounts.undefinedLinkWarn++;
		} else if (sm_config.preDefTopo.fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_ENABLED) {
			snprintf(undefinedLinkPrint, sizeof(undefinedLinkPrint), " No Link (Quarantined)");
			topop->preDefLogCounts.undefinedLinkQuarantined++;
		}
	}

	// NodeDesc Violation Logging
	if (nodeDescViolation && (validationPort != NULL)) {
		if (forceLogAsWarn || sm_config.preDefTopo.fieldEnforcement.nodeDesc == FIELD_ENF_LEVEL_WARN) {
			snprintf(expectedNodeDescPrint, sizeof(expectedNodeDescPrint), " NodeDesc: %s (Warn)", validationPort->NodeDesc);
			topop->preDefLogCounts.nodeDescWarn++;
		} else if (sm_config.preDefTopo.fieldEnforcement.nodeDesc == FIELD_ENF_LEVEL_ENABLED) {
			snprintf(expectedNodeDescPrint, sizeof(expectedNodeDescPrint), " NodeDesc: %s (Quarantined)", validationPort->NodeDesc);
			topop->preDefLogCounts.nodeDescQuarantined++;
		}
	}

	// NodeGUID Violation Logging
	if (nodeGuidViolation) {
		if (forceLogAsWarn || sm_config.preDefTopo.fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_WARN) {
			snprintf(expectedNodeGuidPrint, sizeof(expectedNodeGuidPrint),
				" NodeGUID: "FMT_U64" (Warn)", (validationPort ? validationPort->NodeGUID : 0));
			topop->preDefLogCounts.nodeGuidWarn++;
		} else if (sm_config.preDefTopo.fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_ENABLED) {
			snprintf(expectedNodeGuidPrint, sizeof(expectedNodeGuidPrint),
				" NodeGUID: "FMT_U64" (Quarantined)", (validationPort ? validationPort->NodeGUID : 0));
			topop->preDefLogCounts.nodeGuidQuarantined++;
		}
	}

	// PortGUID Violation Logging
	if (portGuidViolation) {
		if (forceLogAsWarn || sm_config.preDefTopo.fieldEnforcement.portGuid == FIELD_ENF_LEVEL_WARN) {
			snprintf(expectedPortGuidPrint, sizeof(expectedPortGuidPrint),
				" PortGUID: "FMT_U64" (Warn)", (validationPort ? validationPort->PortGUID : 0));
			topop->preDefLogCounts.portGuidWarn++;
		} else if (sm_config.preDefTopo.fieldEnforcement.portGuid == FIELD_ENF_LEVEL_ENABLED) {
			snprintf(expectedPortGuidPrint, sizeof(expectedPortGuidPrint),
				" PortGUID: "FMT_U64" (Quarantined)", (validationPort ? validationPort->PortGUID : 0));
			topop->preDefLogCounts.portGuidQuarantined++;
		}
	}

	if (sm_config.preDefTopo.logMessageThreshold != 0 &&
		topop->preDefLogCounts.totalLogCount > sm_config.preDefTopo.logMessageThreshold) {
		return;
	}

	// If something is getting quarantined, log it as an error, otherwise log
	// it as a warning.
	if (logError) {
		IB_LOG_ERROR_FMT(__func__, "Pre-Defined Topology: Line %"PRIu64
			" Found %s (NodeGUID: "FMT_U64", NodeType: %s,%sPortNum: %d)"
			" connected to %s (NodeGUID: "FMT_U64", NodeType: %s,%sPortNum: %d)"
			", but expected:%s%s"/* */"%s%s"/* */"%s%s",
			(validationPort)?( validationPort->enodep->lineno):0,
			nodeDesc->NodeString, nodeInfo->NodeGUID, StlNodeTypeToText(nodeInfo->NodeType),
			nodeInfo->PortGUID ? foundPortGuidPrint : " ", nodeInfo->u1.s.LocalPortNum,
			sm_nodeDescString(upstreamNodep), upstreamNodep->nodeInfo.NodeGUID, StlNodeTypeToText(upstreamNodep->nodeInfo.NodeType),
			upstreamPortp->portData->guid ? basePortGuidPrint : " ", upstreamPortp->index,
			expectedNodeDescPrint, nodeDescViolation && (nodeGuidViolation || portGuidViolation) ? "," : "",
			expectedNodeGuidPrint, nodeGuidViolation && portGuidViolation ? "," : "",
			expectedPortGuidPrint, undefinedLinkPrint);
	} else {
		IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: Line %"PRIu64
			" Found %s (NodeGUID: "FMT_U64", NodeType: %s,%sPortNum: %d)"
			" connected to %s (NodeGUID: "FMT_U64", NodeType: %s,%sPortNum: %d)"
			", but expected:%s%s"/* */"%s%s"/* */"%s%s",
			(validationPort)?( validationPort->enodep->lineno):0, nodeDesc->NodeString,
			nodeInfo->NodeGUID, StlNodeTypeToText(nodeInfo->NodeType),
			nodeInfo->PortGUID ? foundPortGuidPrint : " ", nodeInfo->u1.s.LocalPortNum,
			sm_nodeDescString(upstreamNodep), upstreamNodep->nodeInfo.NodeGUID,
			StlNodeTypeToText(upstreamNodep->nodeInfo.NodeType),
			upstreamPortp->portData->guid ? basePortGuidPrint : " ", upstreamPortp->index,
			expectedNodeDescPrint, nodeDescViolation && (nodeGuidViolation || portGuidViolation) ? "," : "",
			expectedNodeGuidPrint, nodeGuidViolation && portGuidViolation ? "," : "",
			expectedPortGuidPrint, undefinedLinkPrint);
	}

	topop->preDefLogCounts.totalLogCount++;
}

/*
* Validates the SM NodeGuid and PortGuid.
*
* Checks the ExpectedSM link for matching NodeGuid and PortGuid
*
* @returns if the SM is authentic or not
*/
static int
_validate_predef_sm_field(Topology_t* topop, FabricData_t* pdtop,
	STL_NODE_INFO* nodeInfo, STL_NODE_DESCRIPTION* nodeDesc)
{
	FSTATUS status;
	SmPreDefTopoXmlConfig_t *pdtCfg = &sm_config.preDefTopo;
	int authentic = 1;

	status = FindExpectedSMByNodeGuid(pdtop, nodeInfo->NodeGUID);
	if(status != FSUCCESS) {
		if(pdtCfg->fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_WARN) {
			if(topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold) {
				IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: %s (NodeGUID: " FMT_U64") was not found in input file (Warn).",
							(char*) nodeDesc->NodeString, nodeInfo->NodeGUID);
				topop->preDefLogCounts.totalLogCount++;
			}
			topop->preDefLogCounts.nodeGuidWarn++;
		}
		else if(pdtCfg->fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_ENABLED) {
			if(topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold) {
				IB_LOG_ERROR_FMT(__func__, "Pre-Defined Topology: %s (NodeGUID: " FMT_U64") was not found in input file (Quarantined).",
							(char*) nodeDesc->NodeString, nodeInfo->NodeGUID);
				topop->preDefLogCounts.totalLogCount++;
			}
			topop->preDefLogCounts.nodeGuidQuarantined++;
			authentic = 0;
		}
		// If we have hit our log threshold, spit out a final message but then stop logging
		if(topop->preDefLogCounts.totalLogCount == pdtCfg->logMessageThreshold) {
			IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: Log message threshold of %d messages per sweep has been reached. Suppressing further topology mismatch information.",
							pdtCfg->logMessageThreshold);
			topop->preDefLogCounts.totalLogCount++;
		}
	}

	status = FindExpectedSMByPortGuid(pdtop, nodeInfo->PortGUID);
	if(status != FSUCCESS) {
		if(pdtCfg->fieldEnforcement.portGuid == FIELD_ENF_LEVEL_WARN) {
			if(topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold) {
				IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: %s (PortGUID: "FMT_U64") was not found in input file (Warn).",
							(char*) nodeDesc->NodeString, nodeInfo->PortGUID);
				topop->preDefLogCounts.totalLogCount++;
			}
			topop->preDefLogCounts.portGuidWarn++;
		}
		else if(pdtCfg->fieldEnforcement.portGuid == FIELD_ENF_LEVEL_ENABLED) {
			if(topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold) {
				IB_LOG_ERROR_FMT(__func__, "Pre-Defined Topology: %s (PortGUID: "FMT_U64") was not found in input file (Quarantined).",
							(char*) nodeDesc->NodeString, nodeInfo->PortGUID);
				topop->preDefLogCounts.totalLogCount++;
			}
			topop->preDefLogCounts.portGuidQuarantined++;
			authentic = 0;
		}
		// If we have hit our log threshold, spit out a final message but then stop logging
		if(topop->preDefLogCounts.totalLogCount == pdtCfg->logMessageThreshold) {
			IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: Log message threshold of %d messages per sweep has been reached. Suppressing further topology mismatch information.",
							pdtCfg->logMessageThreshold);
			topop->preDefLogCounts.totalLogCount++;
		}
	}

	return authentic;
}

static int
_portsel_match(PortSelector *ps, const char *nd, int pnum)
{
	return (ps && strncmp(nd, ps->NodeDesc,
		STL_NODE_DESCRIPTION_ARRAY_SIZE) == 0 && pnum == ps->PortNum);
}

/*
 * Find all ExpectedLink elems in @fdp given NodeDesc and PortNum values.
 *
 * Can do partial matches if either @ndesc1 or @ndesc2 is NULL. Writes
 * up to @elSize found links to @elOut. Writes in order discovered so
 * if there are more than @elSize hits, additional matching
 * ExpectedLinks will not be written.
 *
 * @returns number of matches in fdp->ExpectedLinks, even if greater
 * than @elSize. Does not invalidate @elOut; only up to @return value
 * pointers in @elOut are guaranteed to be valid.
 */
static int
_find_exp_links_by_desc_and_port(FabricData_t * fdp,
	const char *ndesc1, int pnum1,
	const char *ndesc2, int pnum2,
	ExpectedLink **elOut, int elSize)
{
	int hits = 0;
	LIST_ITEM *it;

	if (!ndesc1 && !ndesc2)
		return 0;

	for (it = QListHead(&fdp->ExpectedLinks); it != NULL;
		it = QListNext(&fdp->ExpectedLinks, it)) {
		int n1match, n2match; // Matching side
		n1match = n2match = 0;
		ExpectedLink *el = PARENT_STRUCT(it, ExpectedLink, ExpectedLinksEntry);

		if (!el->portselp1 || !el->portselp2)
			continue;

		if (ndesc1) {
			if (_portsel_match(el->portselp1, ndesc1, pnum1))
				n1match = 1;
			else if (_portsel_match(el->portselp2, ndesc1, pnum1))
				n1match = 2;
		}

		if (ndesc2) {
			if (_portsel_match(el->portselp1, ndesc2, pnum2))
				n2match = 1;
			else if (_portsel_match(el->portselp2, ndesc2, pnum2))
				n2match = 2;
		}

		// Both are defined, require complete match
		if (ndesc1 && ndesc2) {
			if (!n1match || !n2match)
				continue;
			if (n1match == n2match)
				continue;
		} else if (ndesc1 && !n1match) {
			continue;
		} else if (ndesc2 && !n2match)
			continue;

		if (hits < elSize)
			elOut[hits] = el;
		++hits;
	}

	return hits;
}

static int
_validate_predef_fields(Topology_t* topop, FabricData_t* pdtop, Node_t * upstreamNodep,
	Port_t * upstreamPortp, STL_NODE_INFO* nodeInfo, STL_NODE_DESCRIPTION* nodeDesc,
	uint32_t* quarantineReasons, STL_EXPECTED_NODE_INFO* expNodeInfo,
	boolean forceLogAsWarn)
{
	int authentic = 1;
	ExpectedLink* validationLink;
	PortSelector* validationPort;
	uint8_t linkSide;
	uint32_t logQuarantineReasons = 0x00000000;
	SmPreDefTopoXmlConfig_t *pdtCfg = &sm_config.preDefTopo;

	// If all the fields enforcements are disabled, no need of further checks,
	// return authentic
	if (pdtCfg->fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_DISABLED &&
		pdtCfg->fieldEnforcement.nodeDesc == FIELD_ENF_LEVEL_DISABLED &&
		pdtCfg->fieldEnforcement.portGuid == FIELD_ENF_LEVEL_DISABLED &&
		pdtCfg->fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_DISABLED) {
		return authentic;
	}

	if(pdtop == NULL || quarantineReasons == NULL || nodeInfo == NULL || nodeDesc == NULL) {
		authentic = 0;
		return authentic;
	}

	// If either of these are null, we're on the SM node looking at our self at
	// the start of a sweep, so just make sure that we show up in the topology.
	// The logging for this is done separately from sm_log_predef_violation as
	// this is a weird edge case
	if(upstreamNodep == NULL || upstreamPortp == NULL) {
		authentic = _validate_predef_sm_field(topop, pdtop, nodeInfo, nodeDesc);
		return authentic;
	}

	// UndefinedLink Validation
	validationLink = FindExpectedLinkByOneSide(pdtop, upstreamNodep->nodeInfo.NodeGUID, upstreamPortp->index, &linkSide);

	// Special case: match by NodeDesc and PortNum
	if(validationLink == NULL &&
		pdtCfg->fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_DISABLED) {

		int hitCnt;
		int forcePrint = 0;

		if(pdtCfg->logMessageThreshold == 0 ||
			topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold) {
			IB_LOG_WARN_FMT(__func__, "Validation link not found using (NodeGUID: "
				FMT_U64", PortNum: %d). Looking up link by (NodeDesc: %s,"
				" PortNum: %d). False matches may occur if node descriptions are"
				" not unique.", upstreamNodep->nodeInfo.NodeGUID, upstreamPortp->index,
				sm_nodeDescString(upstreamNodep), upstreamPortp->index);
			topop->preDefLogCounts.totalLogCount++;
			forcePrint = (topop->preDefLogCounts.totalLogCount >= pdtCfg->logMessageThreshold);
		}

		hitCnt = _find_exp_links_by_desc_and_port(pdtop,
			(char*)upstreamNodep->nodeDesc.NodeString, upstreamPortp->index, NULL, 0,
			&validationLink, 1);

		if (hitCnt > 1 && (pdtCfg->logMessageThreshold == 0 || forcePrint ||
			topop->preDefLogCounts.totalLogCount < pdtCfg->logMessageThreshold)) {
			IB_LOG_WARN_FMT(__func__, "Mutlitple expected links matched by node"
				" desc and port number. NodeGUID: "FMT_U64", NodeDesc: %s,"
				" PortNum: %d", upstreamNodep->nodeInfo.NodeGUID, sm_nodeDescString(upstreamNodep),
				upstreamPortp->index);
			if (!forcePrint)
				topop->preDefLogCounts.totalLogCount++;
		}

		if(topop->preDefLogCounts.totalLogCount == pdtCfg->logMessageThreshold) {
			IB_LOG_WARN_FMT(__func__, "Pre-Defined Topology: Log message threshold of %d messages per sweep has been reached. Suppressing further topology mismatch information.", pdtCfg->logMessageThreshold);
			topop->preDefLogCounts.totalLogCount++;
		}

		if (validationLink) {
			if (strncmp(validationLink->portselp1->NodeDesc,
				(char*) upstreamNodep->nodeDesc.NodeString, STL_NODE_DESCRIPTION_ARRAY_SIZE) == 0)
				linkSide = 1;
			else
				linkSide = 2;
		}
	}

	//Found a potential match, now verify it is connected to correct PortNum on
	//other side
	if(validationLink) {
		validationPort = linkSide == 1 ? validationLink->portselp2 : validationLink->portselp1;
		if(validationPort->PortNum != nodeInfo->u1.s.LocalPortNum)
			validationLink = NULL;
	}

	if (validationLink == NULL) {
		if(pdtCfg->fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_DISABLED)
			return authentic;

		logQuarantineReasons |= STL_QUARANTINE_REASON_TOPO_UNDEFINED_LINK;
		if(pdtCfg->fieldEnforcement.undefinedLink == FIELD_ENF_LEVEL_ENABLED) {
			*quarantineReasons |= STL_QUARANTINE_REASON_TOPO_UNDEFINED_LINK;
			authentic = 0;
		}

		_log_predef_field_violation(topop, logQuarantineReasons, forceLogAsWarn,
			upstreamNodep, upstreamPortp, nodeInfo, nodeDesc, NULL);

		return authentic;
	}


	// NodeGUID Validation
	if(pdtCfg->fieldEnforcement.nodeGuid != FIELD_ENF_LEVEL_DISABLED && nodeInfo->NodeGUID != validationPort->NodeGUID) {
		if(pdtCfg->fieldEnforcement.nodeGuid == FIELD_ENF_LEVEL_ENABLED) {
			if(expNodeInfo)
				expNodeInfo->nodeGUID = validationPort->NodeGUID;

			*quarantineReasons |= STL_QUARANTINE_REASON_TOPO_NODE_GUID;
			authentic = 0;
		}
		logQuarantineReasons |= STL_QUARANTINE_REASON_TOPO_NODE_GUID;
	}

	if(pdtCfg->fieldEnforcement.nodeDesc != FIELD_ENF_LEVEL_DISABLED &&
			(validationPort->NodeDesc == NULL ||
			strncmp((char*) nodeDesc->NodeString, validationPort->NodeDesc, STL_NODE_DESCRIPTION_ARRAY_SIZE))) {
		if(pdtCfg->fieldEnforcement.nodeDesc == FIELD_ENF_LEVEL_ENABLED) {
			if(expNodeInfo){
				if(validationPort->NodeDesc == NULL)
					strncpy((char*) expNodeInfo->nodeDesc.NodeString, "<UNDEFINED>", STL_NODE_DESCRIPTION_ARRAY_SIZE);
				else
					strncpy((char*) expNodeInfo->nodeDesc.NodeString, validationPort->NodeDesc, STL_NODE_DESCRIPTION_ARRAY_SIZE);
			}

			*quarantineReasons |= STL_QUARANTINE_REASON_TOPO_NODE_DESC;
			authentic = 0;
		}
		logQuarantineReasons |= STL_QUARANTINE_REASON_TOPO_NODE_DESC;
	}

	// PortGUID Validation
	// Only valid for HFIs, as Switches only have a single PortGUID for Port 0.
	// If we expected a switch and found an HFI, do a PortGUID comparison as it
	// is technically an invalid PortGUID (we were expecting 0)
	if(pdtCfg->fieldEnforcement.portGuid != FIELD_ENF_LEVEL_DISABLED &&
			((validationPort->NodeType == NI_TYPE_CA && nodeInfo->PortGUID != validationPort->PortGUID) ||
			(validationPort->NodeType == NI_TYPE_SWITCH && nodeInfo->NodeType == NI_TYPE_CA && nodeInfo->PortGUID != validationPort->PortGUID))) {
		if(pdtCfg->fieldEnforcement.portGuid == FIELD_ENF_LEVEL_ENABLED) {
			if(expNodeInfo)
				expNodeInfo->portGUID = validationPort->PortGUID;

			*quarantineReasons |= STL_QUARANTINE_REASON_TOPO_PORT_GUID;
			authentic = 0;
		}
		logQuarantineReasons |= STL_QUARANTINE_REASON_TOPO_PORT_GUID;
	}

	if(logQuarantineReasons) {
		_log_predef_field_violation(topop, logQuarantineReasons, forceLogAsWarn,
			upstreamNodep, upstreamPortp, nodeInfo, nodeDesc, validationPort);
	}

	return authentic;
}

/*
 *	Get SMInfo from the SM at port passed in.
 */
static Status_t
_get_sminfo(Topology_t * topop, Node_t * nodep, Port_t * portp)
{
	STL_SM_INFO smInfo;
	Status_t status;
	STL_SMINFO_RECORD sminforec = { {0}, 0 };

	IB_ENTER(__func__, nodep, portp, 0, 0);

	/*
	 *  Fetch the SMInfo from the other port.
	 */
	SmpAddr_t addr = SMP_ADDR_CREATE_DR(PathToPort(nodep, portp));
	status = SM_Get_SMInfo(fd_topology, 0, &addr, &smInfo);
	if (status != VSTATUS_OK) {
		IB_LOG_INFINI_INFO_FMT(__func__,
							   "failed to get SmInfo from remote SM %s : " FMT_U64,
							   sm_nodeDescString(nodep), portp->portData->guid);
		/* remove the SM from our list */
		(void) sm_dbsync_deleteSm(portp->portData->guid);
		status = sm_popo_port_error(&sm_popo, topop, portp, status);
		IB_EXIT(__func__, status);
		return status;
	} else {
		/*
		 * add the found Sm to the topology
		 */
		sminforec.RID.LID = portp->portData->portInfo.LID;
		sminforec.SMInfo = smInfo;
		(void) sm_dbsync_addSm(nodep, portp, &sminforec);
		if (smInfo.SM_Key != sm_smInfo.SM_Key) {
			IB_LOG_WARN_FMT(__func__,
							"Remote SM %s [" FMT_U64 "] does not have the proper SM_Key["
							FMT_U64 "]", sm_nodeDescString(nodep), portp->portData->guid,
							smInfo.SM_Key);
		}
	}

	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}								/* _get_sminfo */

static Status_t
_get_portinfo_loop(SmMaiHandle_t *fd, Topology_t * topop, Node_t * nodep,
	bitset_t * needPortInfo, STL_PORT_INFO * newPortInfos,
	ParallelSweepContext_t *psc)
{
	Status_t status;
	int i;
	SmpAddr_t addr = SMP_ADDR_CREATE_DR(nodep->path);

	for (i = bitset_find_first_one(needPortInfo); i != -1;
		i = bitset_find_next_one(needPortInfo, i + 1)) {
		psc_unlock(psc);
		status = SM_Get_PortInfo(fd, 1<<24 | STL_SM_CONF_START_ATTR_MOD | (uint32_t)i,
			&addr, newPortInfos + i);
		psc_lock(psc);
		if (status == VSTATUS_OK) {
			bitset_clear(needPortInfo, i);
		} else {
			Port_t * portp = sm_find_node_port(topop, nodep, i);
			if ((status = sm_popo_port_error(&sm_popo, sm_topop, portp,
				status)) == VSTATUS_TIMEOUT_LIMIT) {
				return status;
			}
		}
	}

	return VSTATUS_OK;
}

/*
 * Perform a Get(Aggregate) of PortInfo's for a node.
 *
 * Only request the PortInfo's for the ports select in the @ports
 * bitmask. Resulting PortInfo structures are put in the @res array. Note
 * that @res will be indexed into directly by port number, hence filling only
 * entries that we're actually requesting.
 *
 * Upon completion, @ports bitmask will leave set bits corresponding to ports that
 * had a error in getting their corresponding PortInfo.
 */
static Status_t
_get_portinfo_aggr(SmMaiHandle_t *fd, Topology_t *topop, Node_t *nodep,
	bitset_t *ports, STL_PORT_INFO *res, ParallelSweepContext_t *psc)
{
	const size_t blockSize = sizeof(STL_AGGREGATE) + 8*((sizeof(STL_PORT_INFO) + 7)/8);
	const size_t portsPerAggr = (STL_MAX_PAYLOAD_SMP_DR / blockSize);
	uint8_t buffer[STL_MAX_PAYLOAD_SMP_DR];
	STL_AGGREGATE *aggrHdr = (STL_AGGREGATE*)buffer;
	int port;
	bitset_t local_ports;
	uint8_t i = 0;
	Status_t status = VSTATUS_OK;

	memset(buffer, 0, sizeof(buffer));
	bitset_init(&sm_pool, &local_ports, MAX_STL_PORTS + 1);
	bitset_copy(&local_ports, ports);

	bitset_clear_all(ports);

	port = bitset_find_first_one(&local_ports);

	while (port != -1) {

		aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_PORT_INFO;
		aggrHdr->Result.s.Error = 0;
		aggrHdr->Result.s.Reserved = 0;
		aggrHdr->Result.s.RequestLength = (sizeof(STL_PORT_INFO) + 7)/8;

		aggrHdr->AttributeModifier = (1<<24) | STL_SM_CONF_START_ATTR_MOD | (uint32_t)port;
		*((STL_PORT_INFO*)aggrHdr->Data) = (STL_PORT_INFO){0};

		aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);
		++i;

		if (port == bitset_find_last_one(&local_ports) || i == portsPerAggr) {
			uint32_t madStatus = 0;
			STL_AGGREGATE *lastSeg = NULL;

			psc_unlock(psc);
			status = SM_Get_Aggregate_DR(fd, (STL_AGGREGATE*)buffer, aggrHdr, 0,
				nodep->path, &lastSeg, &madStatus);
			psc_lock(psc);
			status = sm_popo_port_error(&sm_popo, topop,
				sm_get_port(nodep, nodep->nodeInfo.NodeType == NI_TYPE_SWITCH ? 0 : port), status);
			if (status == VSTATUS_TIMEOUT_LIMIT)
				goto exit;

			if (status == VSTATUS_OK && lastSeg) {
				for (i = 0, aggrHdr = (STL_AGGREGATE*)buffer; aggrHdr < lastSeg; aggrHdr = STL_AGGREGATE_NEXT(aggrHdr), ++i) {
					STL_PORT_INFO *pPortInfo = (STL_PORT_INFO*)aggrHdr->Data;
					uint8_t cport;

					// Processing this PortInfo, clear it from the bitset.
					bitset_clear(&local_ports, cport = bitset_find_first_one(&local_ports));

					if (aggrHdr->Result.s.Error) {
						// Let caller know of failed ports.
						// Stop processing, rest of the Aggregate is bunk.
						bitset_set(ports, cport);
						break;
					}

					BSWAP_STL_PORT_INFO(pPortInfo);
					res[cport] = *pPortInfo;
				}
			} else {
				IB_LOG_ERROR_FMT(__func__,
					"Error on Get(Aggregate) DR from NodeGUID "FMT_U64" [%s]; status=%u",
					nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep), status);
				// Reset state on failure
				bitset_copy(ports, &local_ports);
				bitset_free(&local_ports);
				return status == VSTATUS_OK ? VSTATUS_BAD : status;
			}

			i = 0;
		}

		// Reset the port we're processing if we had an error.
		if (aggrHdr->Result.s.Error)
			port = bitset_find_first_one(&local_ports);
		else
			port = bitset_find_next_one(&local_ports, port + 1);

		if (i == 0)
			aggrHdr = (STL_AGGREGATE*)buffer;

	}

exit:
	bitset_free(&local_ports);

	return status;

}

static void
_check_for_new_endnode(STL_NODE_INFO * nodeInfo, Node_t * nodep, Node_t * oldnodep)
{
	Port_t *oldportp;
	int match;

	if (oldnodep == NULL)
		oldnodep = sm_find_guid(&old_topology, nodeInfo->NodeGUID);
	if (!oldnodep) {
		// this is a new node; mark it
		if (sm_mark_new_endnode(nodep) != VSTATUS_OK)
			topology_changed = 1;
	} else {
		// old node did exist; check the ports for a match
		match = 0;
		for_all_ports(oldnodep, oldportp) {
			if (oldportp
				&& oldportp->state > IB_PORT_DOWN
				&& oldportp->index == nodeInfo->u1.s.LocalPortNum) {
				match = 1;
				break;
			}
		}
		if (!match) {
			// found no matches
			// this isn't a new node, but it has a new port, so mark it
			if (sm_mark_new_endnode(nodep) != VSTATUS_OK)
				topology_changed = 1;
		}
	}
}

static inline uint8
_get_trusted_node_type(Port_t *upstreamPort, const STL_NODE_INFO *downstreamNodeInfo) {
	if(sm_valid_port(upstreamPort)) {
		if(!sm_stl_appliance(upstreamPort->portData->portInfo.NeighborNodeGUID)) {
			switch(upstreamPort->portData->portInfo.PortNeighborMode.NeighborNodeType) {
				case STL_NEIGH_NODE_TYPE_HFI:
					return NI_TYPE_CA;
				case STL_NEIGH_NODE_TYPE_SW:
					return NI_TYPE_SWITCH;
			}
		}
	}
	return downstreamNodeInfo->NodeType;
}

// _get_discovery_node_attributes()
//  Retrieves attributes from a new node that are useful during discovery.
//  Currently retreives nodeDesc, PortInfo, and SwitchInfo (in that order). When
//  available, utilizes aggregate MADs.
//
//   Parameters:
//    fd - MAI file descriptor to use. Defaults to fd_topology.
//    path - DR path to the node
//    portNumber - port index of the node we will be interogating
//    nodeDesc - Pointer to where nodeDesc we retrieve should be placed
//    PortInfo - Pointer to where PortInfo we retreive should be placed
//    switchInfo - OPTIONAL pointer to where swInfo we retreive should be
//    placed (set to NULL if this is not a switch)
//    err - Pointer to buffer where we will place a string indicating error
//    should we have one
//
//   Returns:
//
//   Regular status.
//
static Status_t
_get_discovery_node_attributes(SmMaiHandle_t *fd, uint8_t *path, uint8_t portNumber,
	STL_NODE_DESCRIPTION *nodeDesc, STL_PORT_INFO *PortInfo,
	STL_SWITCH_INFO *switchInfo, char *err, uint8_t *discnodestatus)
{
	Status_t status = VSTATUS_OK;
	uint8_t buffer[3 * sizeof(STL_AGGREGATE) + 8*((sizeof(STL_NODE_DESCRIPTION) + 7)/8) + 8*((sizeof(STL_PORT_INFO) + 7)/8)
					+ 8*((sizeof(STL_SWITCH_INFO) + 7)/8)] = {0};
	STL_AGGREGATE *aggrHdr = (STL_AGGREGATE*)buffer;
	STL_AGGREGATE *lastSeg = NULL;
	SmpAddr_t addr = SMP_ADDR_CREATE_DR(path);

	if (sm_config.use_aggregates) {
		//
		// Get the NodeDescription.
		//
		aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_NODE_DESCRIPTION;
		aggrHdr->Result.s.Error = 0;
		aggrHdr->Result.s.Reserved = 0;
		aggrHdr->Result.s.RequestLength = (sizeof(STL_NODE_DESCRIPTION) + 7)/8;
		aggrHdr->AttributeModifier = 0;

		aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);

		// Get the PortInfo.
		aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_PORT_INFO;
		aggrHdr->Result.s.Error = 0;
		aggrHdr->Result.s.Reserved = 0;
		aggrHdr->Result.s.RequestLength = (sizeof(STL_PORT_INFO) + 7)/8;
		aggrHdr->AttributeModifier = (1<<24) | STL_SM_CONF_START_ATTR_MOD | (uint32_t)portNumber;

		aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);

		// If switch, get SwitchInfo too
		if (switchInfo) {
			aggrHdr->AttributeID = STL_MCLASS_ATTRIB_ID_SWITCH_INFO;
			aggrHdr->Result.s.Error = 0;
			aggrHdr->Result.s.Reserved = 0;
			aggrHdr->Result.s.RequestLength = (sizeof(STL_SWITCH_INFO) + 7)/8;
			aggrHdr->AttributeModifier = 0;

			aggrHdr = STL_AGGREGATE_NEXT(aggrHdr);
		}

		status = SM_Get_Aggregate_DR(fd, (STL_AGGREGATE*)buffer, aggrHdr, 0 ,path,
			&lastSeg, NULL);

		aggrHdr = (STL_AGGREGATE*)buffer;
	}

	if (!sm_config.use_aggregates || status != VSTATUS_OK) {
		if (sm_config.use_aggregates)
			err += sprintf(err, "Get(Aggregate) failed with status = %d, attempting fallback Gets()...", status);

		if ((status = SM_Get_NodeDesc(fd, 0, &addr, nodeDesc)) != VSTATUS_OK) {
			sprintf(err, "Get(NodeDesc) failed with status = %d", status);
			*discnodestatus |= NODEDESC_UNAVAILABLE;
			return status;
		}

		if ((status = SM_Get_PortInfo(fd, (1<<24) | STL_SM_CONF_START_ATTR_MOD | (uint32_t)portNumber, &addr, PortInfo)) != VSTATUS_OK) {
			sprintf(err, "Get(PortInfo) failed with status = %d", status);
			*discnodestatus |= PORTINFO_UNAVAILABLE;
			return status;
		}

		if (switchInfo && (status = SM_Get_SwitchInfo(fd, 0, &addr, switchInfo)) != VSTATUS_OK) {
			sprintf(err, "Get(SwitchInfo) failed with status = %d", status);
			*discnodestatus |= SWINFO_UNAVAILABLE;
			return status;

		}

	} else {
		// Aggregate itself was okay, but encapsulated NodeDesc or PortInfo was bad.
		if (status != VSTATUS_OK) {
			switch (lastSeg->AttributeID) {
				case STL_MCLASS_ATTRIB_ID_NODE_DESCRIPTION:
					sprintf(err, "Get(NodeDesc) failed with status = %d", status);
					*discnodestatus |= NODEDESC_UNAVAILABLE;
					return status;

				case STL_MCLASS_ATTRIB_ID_PORT_INFO:
					sprintf(err, "Get(PortInfo) failed with status = %d", status);
					*discnodestatus |= PORTINFO_UNAVAILABLE;
					return status;

				case STL_MCLASS_ATTRIB_ID_SWITCH_INFO:
					sprintf(err, "Get(SwitchInfo) failed with status = %d", status);
					*discnodestatus |= SWINFO_UNAVAILABLE;
					return status;
				default:
					break;
			}
		} else {
			*PortInfo = *((STL_PORT_INFO*)STL_AGGREGATE_NEXT(aggrHdr)->Data);
			BSWAP_STL_PORT_INFO(PortInfo);

			if (switchInfo) {
				*switchInfo = *((STL_SWITCH_INFO*)STL_AGGREGATE_NEXT(STL_AGGREGATE_NEXT(aggrHdr))->Data);
				BSWAP_STL_SWITCH_INFO(switchInfo);
			}
		}

		*nodeDesc = *((STL_NODE_DESCRIPTION*)(aggrHdr)->Data);
		BSWAP_STL_NODE_DESCRIPTION(nodeDesc);
	}

	// Success
	return VSTATUS_OK;
}

// Authenticates node described by neighborInfo and neighborPI, which is being
// discovered through upstreamNodep and upstreamPortp. upstreamNodep and
// upstreamPortp should always be a trusted node.  As called by setup_node,
// upstreamNodep and upstreamPortp will always be either a switch or our SM
// node.  neighborInfo must not be NULL for authentication to succeed
static __inline__ int
_stl_authentic_node(Topology_t *tp, Node_t *upstreamNodep, Port_t *upstreamPortp,
	STL_NODE_INFO *neighborInfo, STL_PORT_INFO *neighborPI,
	STL_SWITCH_INFO *swInfo, uint32 *quarantineReasons,
	uint8_t discnodestatus)
{
	int authentic = 1;
	uint64 expectedPortGuid;

	if (quarantineReasons == NULL) {
		return 0;
	}

	if (neighborInfo == NULL) {
		return 0;
	}

	if (neighborPI && !(discnodestatus & PORTINFO_UNAVAILABLE) &&
		sm_valid_port(upstreamPortp) &&
		!(neighborPI->PortPacketFormats.Supported &
		upstreamPortp->portData->portInfo.PortPacketFormats.Supported)) {

		*quarantineReasons |= STL_QUARANTINE_REASON_BAD_PACKET_FORMATS;
		authentic = 0;
		return (authentic);
	}

	if (swInfo && !(discnodestatus & SWINFO_UNAVAILABLE) &&
		tp->routingModule->funcs.routing_mode() == STL_ROUTE_LINEAR &&
		(swInfo->RoutingMode.Supported & STL_ROUTE_LINEAR) &&
		(swInfo->LinearFDBCap < STL_GET_UNICAST_LID_MAX())) {

		*quarantineReasons |= STL_QUARANTINE_REASON_MAXLID;
		return (authentic = 0);
	}

	if (neighborPI && !(discnodestatus & PORTINFO_UNAVAILABLE)) {
		//Verify node MTU not less than 2048
		if (neighborPI->MTU.Cap < IB_MTU_2048) {
			*quarantineReasons |= STL_QUARANTINE_REASON_SMALL_MTU_SIZE;
			authentic = 0;
			return (authentic);
		}

		// Verify port supports min number required VLs needed for specified
		// configuration This test against LocalPortNum should likely be
		// against NeighborPortNum instead, but as this code is only called
		// when communicating directly across the LocalPortNum link to this
		// node, the point is moot
		if ((neighborInfo->NodeType != NI_TYPE_SWITCH) ||
			((neighborInfo->NodeType == NI_TYPE_SWITCH) &&
			(neighborPI->LocalPortNum!=0))) {
			if (neighborPI->VL.s2.Cap < sm_needed_vls) {
				if (!sm_config.allow_mixed_vls) {
					*quarantineReasons |= STL_QUARANTINE_REASON_VL_COUNT;
					authentic = 0;
					return (authentic);
				} else if (upstreamNodep) {
					IB_LOG_WARN_FMT(__func__, "Admitting node %s NodeGuid "
						FMT_U64" with VL cap %d below required (%d)",
						sm_nodeDescString(upstreamNodep), upstreamNodep->nodeInfo.NodeGUID,
						neighborPI->VL.s2.Cap, sm_needed_vls);
				}
			}
		}
	}

	if (!sm_config.sma_spoofing_check)
		return authentic;

	// If upstreamNodep and upstreamPortp are NULL, then we are at the first
	// node (our own SM) in discovery and have nothing to verify
	if (!(upstreamNodep && upstreamPortp))
		return authentic;

	// If the device is an appliance as configued in opafm.xml, we bypass
	// security checks.
	if (sm_stl_appliance(upstreamPortp->portData->portInfo.NeighborNodeGUID))
		return authentic;

	if (!sm_stl_port(upstreamPortp))
		return authentic;


	// This should never happen, but if upstreamPortp is unenhanced SWP0, then
	// it doesn't support these checks so we can bail out.
	if (upstreamNodep->nodeInfo.NodeType == NI_TYPE_SWITCH && upstreamPortp->index == 0
			&& !upstreamNodep->switchInfo.u2.s.EnhancedPort0)
		return authentic;

	if (upstreamPortp->portData->portInfo.NeighborNodeGUID != neighborInfo->NodeGUID ||
		StlNeighNodeTypeToNodeType(upstreamPortp->portData->portInfo.PortNeighborMode.NeighborNodeType) != neighborInfo->NodeType ||
		(neighborPI && !(discnodestatus & PORTINFO_UNAVAILABLE) &&
		upstreamPortp->portData->portInfo.NeighborPortNum != neighborPI->LocalPortNum) ||
		upstreamPortp->portData->portInfo.NeighborPortNum != neighborInfo->u1.s.LocalPortNum) {

		authentic = 0;
		*quarantineReasons |= STL_QUARANTINE_REASON_SPOOF_GENERIC;
	}

	switch (neighborInfo->NodeType) {
		case NI_TYPE_SWITCH:
			if (sm_config.neighborFWAuthenEnable && upstreamPortp->portData->portInfo.PortNeighborMode.NeighborFWAuthenBypass != 0) {
				authentic = 0;
				*quarantineReasons |= STL_QUARANTINE_REASON_SPOOF_GENERIC;
			}
			break;

		case NI_TYPE_CA:
			expectedPortGuid = NodeGUIDtoPortGUID(upstreamPortp->portData->portInfo.NeighborNodeGUID, upstreamPortp->portData->portInfo.NeighborPortNum);

			// for Gen-1 the HFI is never trusted, no need to check the
			// portInfo.PortNeighborMode.NeighborFWAuthenBypass field
			if (expectedPortGuid != neighborInfo->PortGUID)	 {
				authentic = 0;
				*quarantineReasons |= STL_QUARANTINE_REASON_SPOOF_GENERIC;
			}

			// During discovery, if spoof check security is enabled, and we
			// discover a HFI node with a LID that doesn't match the LID the
			// switch thinks the HFI node has, it indicates that HFI has
			// manually tinkered with its LID.
			//
			// The node should be quarantined because it appears the user is
			// trying to spoof the fabric.
			//
			// This is only possible when the port is active, and security is
			// enabled so the switch knows the neighbor LID.  If the port is
			// down, the switch forgets the neighbor LID.
			//
			// Exception when switchport LID is permissive, and this switchport
			// is not doing SLID checks.
			if (upstreamNodep->nodeInfo.NodeType == NI_TYPE_SWITCH
					&& upstreamPortp->portData->portInfo.PortStates.s.PortState == IB_PORT_ACTIVE
					&& upstreamPortp->portData->portInfo.LID != STL_LID_PERMISSIVE
					&& neighborPI != NULL
					&& !(discnodestatus & PORTINFO_UNAVAILABLE)
					&& upstreamPortp->portData->portInfo.LID != neighborPI->LID) {
				authentic = 0;
				*quarantineReasons |= STL_QUARANTINE_REASON_SPOOF_GENERIC;
			}
			break;

		default:
			authentic = 0;
			*quarantineReasons |= STL_QUARANTINE_REASON_SPOOF_GENERIC;
			break;
	}

	return (authentic);
}

// Log the reasons the current node is being quarantined or bounced.
static void
_stl_quarantine_reasons(Topology_t *tp, Node_t *upstreamNodep,
	Port_t *upstreamPortp, Node_t *qnp,	uint32 quarantineReasons,
	STL_EXPECTED_NODE_INFO* expNodeInfo, const STL_PORT_INFO *pQPI)
{
	if (pQPI) {
		if (quarantineReasons & STL_QUARANTINE_REASON_VL_COUNT) {
			IB_LOG_ERROR_FMT(__func__, "Node:%s guid:"FMT_U64" type:%s port:%d. Supported VLs(%d) too small(needs => %d).",
			sm_nodeDescString(qnp), qnp->nodeInfo.NodeGUID, StlNodeTypeToText(qnp->nodeInfo.NodeType), pQPI->LocalPortNum,
			pQPI->VL.s2.Cap, sm_needed_vls);
			return;
		}
		if (quarantineReasons & STL_QUARANTINE_REASON_SMALL_MTU_SIZE) {
			IB_LOG_ERROR_FMT(__func__, "Node:%s guid:"FMT_U64" type:%s port:%d. Supported MTU(%s) too small(needs => 2048).",
			sm_nodeDescString(qnp), qnp->nodeInfo.NodeGUID, StlNodeTypeToText(qnp->nodeInfo.NodeType), pQPI->LocalPortNum,
			IbMTUToText(pQPI->MTU.Cap));
			return;
		}
		if (quarantineReasons & STL_QUARANTINE_REASON_MAXLID) {
			IB_LOG_ERROR_FMT(__func__, "Node %s guid:"FMT_U64" type%s port:%d. Unable to support MaximumLID of %x.",
			sm_nodeDescString(qnp), qnp->nodeInfo.NodeGUID, StlNodeTypeToText(qnp->nodeInfo.NodeType), pQPI->LocalPortNum,
			STL_GET_UNICAST_LID_MAX() );
			return;
		}
	}

	if (upstreamPortp) {
		IB_LOG_ERROR_FMT(__func__, "Neighbor of %s %s NodeGUID "FMT_U64" port %d could not be authenticated. "
			"Reports: %s %s Guid "FMT_U64" Port %d: Actual: %s Guid "FMT_U64", Port %d",
			StlNodeTypeToText(upstreamNodep->nodeInfo.NodeType),
			sm_nodeDescString(upstreamNodep), upstreamNodep->nodeInfo.NodeGUID, upstreamPortp->index,
			//reports
			StlNodeTypeToText(qnp->nodeInfo.NodeType),
			sm_nodeDescString(qnp), qnp->nodeInfo.NodeGUID,
			qnp->nodeInfo.u1.s.LocalPortNum,
			//actual
			OpaNeighborNodeTypeToText(upstreamPortp->portData->portInfo.PortNeighborMode.NeighborNodeType),
			upstreamPortp->portData->portInfo.NeighborNodeGUID,
			upstreamPortp->portData->portInfo.NeighborPortNum);
		IB_LOG_ERROR_FMT(__func__, "Authentication expected from the neighbor guid "FMT_U64" neighbor node type %s",
			upstreamPortp->portData->portInfo.NeighborNodeGUID,
			OpaNeighborNodeTypeToText(upstreamPortp->portData->portInfo.PortNeighborMode.NeighborNodeType));
	} else {
		// Yes, the SM can fail authentication. (Failure in validation of pre-defined topology).
		IB_LOG_ERROR_FMT(__func__, "SM's port failed authentication.");
	}
}

static void
_stl_quarantine_node(Topology_t *tp, Node_t *upstreamNodep, Port_t *upstreamPortp,
	Node_t *qnp, uint32 quarantineReasons, STL_EXPECTED_NODE_INFO* expNodeInfo,
	const STL_PORT_INFO *pQPI)
{
	QuarantinedNode_t * qnodep;

	// allocate memory for quarantined node list entry
	if (vs_pool_alloc(&sm_pool, sizeof(QuarantinedNode_t), (void *)&qnodep)) {
		IB_LOG_WARN0("_stl_quarantine_node: No memory for quarantined node entry");
	} else {
		// add to SM/SA repository quarantined link list used by the SA
		memset(qnodep, 0, sizeof(QuarantinedNode_t));
		qnodep->authenticNode = upstreamNodep;
		qnodep->authenticNodePort = upstreamPortp;
		qnodep->quarantinedNode = qnp;
		qnodep->quarantineReasons = quarantineReasons;
		memcpy(&qnodep->expNodeInfo, expNodeInfo, sizeof(STL_EXPECTED_NODE_INFO));
		Node_Enqueue(tp, qnodep, quarantined_node_head, quarantined_node_tail);
	}

	if (upstreamPortp)
		upstreamPortp->portData->neighborQuarantined = 1;

	// add to SM/SA repository quarantined map used by the SM
	qnp->index = tp->num_quarantined_nodes++;

	// nodeGUID may be falsified, check with neighbor.
	if (upstreamPortp && cl_qmap_insert(tp->quarantinedNodeMap,
		upstreamPortp->portData->portInfo.NeighborNodeGUID,
		&qnp->mapQuarantinedObj.item) == &qnp->mapQuarantinedObj.item) {
		cl_qmap_set_obj(&qnp->mapQuarantinedObj, qnp);
	}

	_stl_quarantine_reasons(tp, upstreamNodep, upstreamPortp, qnp, quarantineReasons,
		expNodeInfo, pQPI);
}

//
// Re-orders the node and type lists to be in BFS order, rooted at
// topop->node_head.
//
// Not thread safe, and the caller must hold the relevant topology lock.
//
static Status_t
_reorder_node_lists(Topology_t * topop)
{
	Node_t * nodep = NULL;
	Port_t * portp = NULL;

	bitset_t visited;
	if (!bitset_init(&sm_pool, &visited, topop->num_nodes)) {
		IB_LOG_ERROR_FMT(__func__, "failed to initialize visited bitset");
		return VSTATUS_UNRECOVERABLE;
	}

	// while on the BFS queue, node->next points along the queue.

	// define the bfs queue and init it to the head node
	Node_t * qhead = topop->node_head;
	Node_t * qtail = topop->node_head;
	qhead->next = qhead->prev = NULL;

	DEBUG_ASSERT(qhead->index < topop->num_nodes);
	bitset_set(&visited, qhead->index);

	// wipe the topo list; we'll place nodes back onto them during the BFS
	// walk.  keep the type lists valid, as those are used to track
	// unreachable nodes
	topop->node_head = topop->node_tail = NULL;

	// walk the bfs queue and build the bfs node list
	while (qhead) {
		for_all_physical_ports(qhead, portp) {
			if (!sm_valid_port(portp) || portp->nodeno == -1)
				continue;

			Node_t * nnodep = sm_find_node(topop, portp->nodeno);
			DEBUG_ASSERT(nnodep && nnodep->index < topop->num_nodes);

			if (!nnodep || bitset_test(&visited, nnodep->index))
				continue;

			qtail = qtail->next = nnodep;
			nnodep->next = nnodep->prev = NULL;

			bitset_set(&visited, nnodep->index);
		}

		Node_t * qnext = qhead->next;

		Node_Enqueue(topop, qhead, node_head, node_tail);
		if (qhead->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			Node_Dequeue_Type(topop, qhead, switch_head, switch_tail);
		} else {
			Node_Dequeue_Type(topop, qhead, ca_head, ca_tail);
		}

		qhead = qnext;
	}

	bitset_free(&visited);

	// devices left on the type lists are unreachable
	// append them to the end of the node list to ensure they're not dangling
	for_all_switch_nodes(topop, nodep) {
		IB_LOG_WARN_FMT(__func__, "unreachable switch; node %s nodeGuid "FMT_U64,
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
		Node_Enqueue(topop, nodep, node_head, node_tail);
	}
	for_all_ca_nodes(topop, nodep) {
		IB_LOG_WARN_FMT(__func__, "unreachable hfi; node %s nodeGuid "FMT_U64,
			sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID);
		Node_Enqueue(topop, nodep, node_head, node_tail);
	}

	// populate type lists in bfs order from the node list
	topop->switch_head = topop->switch_tail = NULL;
	topop->ca_head = topop->ca_tail = NULL;
	for_all_nodes(topop, nodep) {
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			Node_Enqueue_Type(topop, nodep, switch_head, switch_tail);
		} else {
			Node_Enqueue_Type(topop, nodep, ca_head, ca_tail);
		}
	}

	return VSTATUS_OK;
}

static Status_t
_setup_node(Topology_t * topop, FabricData_t * pdtop, Node_t **nnp,
	ParallelSweepContext_t *psc, DiscoveryWorkItem_t *disc_item)
{
	int i;
	int new_node = 0;
	int end_port;
	int start_port;
	int nextIdx, lastIdx;
	int switchPortChange = 0, authenticNode;
	uint8_t portNumber;
	uint64_t portGuid;
	Node_t *qnodep = NULL, *nodep = NULL, *oldnodep = NULL;
	Node_t *linkednodep = NULL;
	Port_t *portp, *oldportp, *swPortp = NULL;
	int use_cache = 0;
	Node_t *cache_nodep = NULL, *noresp_nodep = NULL;
	Port_t *cache_portp = NULL, *noresp_portp = NULL;
	Status_t status = VSTATUS_OK;
	STL_NODE_DESCRIPTION nodeDesc = {{ 0 }};
	STL_NODE_INFO nodeInfo = { 0 };
	STL_PORT_INFO *portInfo;
	STL_PORT_INFO conPortInfo, *conPIp = NULL;
	STL_SWITCH_INFO switchInfo = { 0 };
	STL_PORT_STATE_INFO *portStateInfo = NULL;
	STL_EXPECTED_NODE_INFO expNodeInfo = {{{ 0 }}};
	Node_t *upstreamNodep = disc_item->upstream_nodep;
	Port_t *upstreamPortp = disc_item->upstream_portp;
	uint8_t *path = disc_item->path;
	boolean local = !upstreamNodep || !upstreamPortp;
	SmpAddr_t addr = SMP_ADDR_CREATE_DR(path);
	MaiPool_t *mpp = NULL;
	SmMaiHandle_t *fd = fd_topology;

	//
	// Note that this routine needs to be called with the topology locks set.
	//

	IB_ENTER(__func__, topop, pdtop, psc, disc_item);

	// validate upstream pointers:
	//   both NULL: discovering local node
	//   both !NULL: discovering neighbor of the specified port
	//   else: invalid
	DEBUG_ASSERT(!upstreamNodep == !upstreamPortp);

	if (psc) {
		mpp = psc_get_mai(psc);
		if (mpp) fd = mpp->fd;
	}

	// Basically, we're going to hold the mutex except for those periods
	// when we're actually waiting for a MAD.
	psc_lock(psc);

	if (local) {
		// local discovery is always new
		new_node = 1;
	} else {
		// Skip quarantined ports.
		if (sm_popo_is_port_quarantined(&sm_popo, upstreamPortp)) {
			IB_LOG_WARN_FMT(__func__, "skipping port due to quarantine: node %s nodeGuid "FMT_U64" port %u",
				sm_nodeDescString(upstreamNodep), upstreamNodep->nodeInfo.NodeGUID, upstreamPortp->index);
			IB_EXIT(__func__, VSTATUS_BAD);
			status = VSTATUS_BAD; goto bail;
		}
		if (sm_popo_is_node_quarantined(&sm_popo, upstreamPortp->portData->portInfo.NeighborNodeGUID)) {
			IB_LOG_WARN_FMT(__func__, "skipping node due to quarantine: source node %s nodeGuid "FMT_U64" port %u; dest nodeGuid "FMT_U64,
				sm_nodeDescString(upstreamNodep), upstreamNodep->nodeInfo.NodeGUID, upstreamPortp->index,
				upstreamPortp->portData->portInfo.NeighborNodeGUID);
			IB_EXIT(__func__, VSTATUS_BAD);
			status = VSTATUS_BAD; goto bail;
		}

		// check for races with other threads discovering the same target node
		uint64_t guid = upstreamPortp->portData->portInfo.NeighborNodeGUID;
		nodep = sm_find_guid(topop, guid);

		if (cl_qmap_get(&disc_item->ctx->dup_map, guid) != cl_qmap_end(&disc_item->ctx->dup_map)) {
			// it's currently in progress; abandon
			// it will enqueue a discovery back this way
			status = VSTATUS_OK;
			goto bail;
		} else if (nodep) {
			// already discovered; continue down the "existing node" path
			new_node = 0;
		} else {
			// new node; it's ours; mark it as in progress and continue
			new_node = 1;
			if (cl_qmap_insert(&disc_item->ctx->dup_map, guid, &disc_item->dup_item) != &disc_item->dup_item) {
				IB_LOG_WARN_FMT(__func__, "failed to update the dup guid map: node %s nodeGuid "FMT_U64" port %u; dup guid "FMT_U64,
					sm_nodeDescString(upstreamNodep), upstreamNodep->nodeInfo.NodeGUID, upstreamPortp->index, guid);
				IB_EXIT(__func__, VSTATUS_BAD);
				return VSTATUS_BAD;
			}
		}
	}

	//
	// Check to see if we have cached info on this node.
	//
	if (topology_passcount && !local && upstreamPortp->state == IB_PORT_ACTIVE) {
		sm_get_nonresp_cache_node_port(upstreamNodep, upstreamPortp, &noresp_nodep, &noresp_portp);
		use_cache = sm_find_cached_neighbor(upstreamNodep, upstreamPortp, &cache_nodep, &cache_portp);
	}

	if (!local && upstreamPortp->portData && nodep
			&& !sm_stl_appliance(upstreamPortp->portData->portInfo.NeighborNodeGUID)) {
		//
		// If we've already seen this node, use our stored copy of the NodeInfo.
		//
		nodeInfo = nodep->nodeInfo;
		nodeInfo.u1.s.LocalPortNum = upstreamPortp->portData->portInfo.NeighborPortNum;
	} else {
		// On SM resweep, first check if HFI nodeInfo can use cached data
		if (sm_config.use_cached_hfi_node_data && use_cache && path[0] > 0
				&& cache_nodep && cache_nodep->nodeInfo.NodeType == NI_TYPE_CA
				&& cache_nodep->nodeInfo.NodeGUID == upstreamPortp->portData->portInfo.NeighborNodeGUID) {
			nodeInfo = cache_nodep->nodeInfo;
			nodeInfo.u1.s.LocalPortNum = upstreamPortp->portData->portInfo.NeighborPortNum;
		} else if (sm_config.use_cached_node_data && use_cache && path[0] > 0
				&& cache_nodep && cache_nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
				&& cache_nodep->nodeInfo.NodeGUID == upstreamPortp->portData->portInfo.NeighborNodeGUID) {
			nodeInfo = cache_nodep->nodeInfo;
			nodeInfo.u1.s.LocalPortNum = upstreamPortp->portData->portInfo.NeighborPortNum;
		} else {
			//
			// New node or no cache. Get the current NodeInfo struct.
			//
			psc_unlock(psc);
			status = SM_Get_NodeInfo(fd, 0, &addr, &nodeInfo);
			psc_lock(psc);
			if (status != VSTATUS_OK) {
				if (local) {
					IB_LOG_ERRORRC("Get NodeInfo failed for local node. rc:",
									status);
					status = VSTATUS_UNRECOVERABLE;
					goto bail;
				} else if (sm_popo_inc_and_use_cache_nonresp(&sm_popo, noresp_nodep, &status)) {
					status = VSTATUS_OK;
					memcpy(&nodeInfo, &noresp_nodep->nodeInfo, sizeof(STL_NODE_INFO));
					nodeInfo.PortGUID = noresp_portp->portData->guid;
					nodeInfo.u1.s.LocalPortNum = noresp_portp->index;
					IB_LOG_INFINI_INFO_FMT(__func__,
						"Get NodeInfo failed for nodeGuid "FMT_U64" port %u, via node %s nodeGuid "FMT_U64" port %u; status=%d; using cached data",
						upstreamPortp->portData ? upstreamPortp->portData->portInfo.NeighborNodeGUID : 0,
						upstreamPortp->portData ? upstreamPortp->portData->portInfo.NeighborPortNum : 0,
						sm_nodeDescString(upstreamNodep), upstreamNodep->nodeInfo.NodeGUID, upstreamPortp->index, status);
				} else {
					IB_LOG_WARN_FMT(__func__,
						"Get NodeInfo failed for nodeGuid "FMT_U64" port %u, via node %s nodeGuid "FMT_U64" port %u; status=%d",
						upstreamPortp->portData ? upstreamPortp->portData->portInfo.NeighborNodeGUID : 0,
						upstreamPortp->portData ? upstreamPortp->portData->portInfo.NeighborPortNum : 0,
						sm_nodeDescString(upstreamNodep), upstreamNodep->nodeInfo.NodeGUID, upstreamPortp->index, status);
					// Set the topology changed flag if the node at other end was there
					// on previous sweep
					Node_t * oldUpstreamNodep;
					Port_t * oldUpstreamPortp;
					if (topology_passcount &&
						(oldUpstreamNodep = sm_find_guid(&old_topology, upstreamNodep->nodeInfo.NodeGUID))) {
						if ((oldUpstreamPortp = sm_get_port(oldUpstreamNodep, upstreamPortp->index)) == NULL) {
							IB_LOG_WARN_FMT(__func__,
								"Failed to get Port %d of Node "FMT_U64":%s, status=%d",
								upstreamPortp->index, upstreamNodep->nodeInfo.NodeGUID,
								sm_nodeDescString(upstreamNodep), status);
						} else if (oldUpstreamPortp->portno) {
							topology_changed = 1;
						}
					}
					status = sm_popo_port_error(&sm_popo, topop, upstreamPortp, status);
					goto bail;
				}
			} else {
				sm_popo_clear_cache_nonresp(&sm_popo, noresp_nodep);
			}
		}
	}

	if (nodeInfo.NodeGUID == 0ull) {
		if (local) {
			IB_LOG_ERROR0("Received zero node guid for local node.");
		} else {
			IB_LOG_ERROR_FMT(__func__,
				 "Received zero node guid from node off Port %d of Node "FMT_U64":%s",
				 upstreamPortp->index, upstreamNodep->nodeInfo.NodeGUID, sm_nodeDescString(upstreamNodep));
		}

		status = VSTATUS_BAD; goto bail;

	} else if (nodeInfo.PortGUID == 0ull && nodeInfo.NodeType == NI_TYPE_CA) {
		if (local) {
			IB_LOG_ERROR("Received zero PORTGUID for local node.", 0);
		} else {
			IB_LOG_ERROR_FMT(__func__,
				 "Received NULL PORTGUID from node off Port %d of Node "FMT_U64":%s",
				 upstreamPortp->index, upstreamNodep->nodeInfo.NodeGUID, sm_nodeDescString(upstreamNodep));
		}
		status = VSTATUS_BAD; goto bail;
	} else if (!local && (qnodep = sm_find_quarantined_guid(topop, upstreamPortp->portData->portInfo.NeighborNodeGUID))) {
		// Quarantine check - don't trust nodeInfo.NodeGUID.
		IB_LOG_ERROR_FMT(__func__,
			 "Ignoring port connections to quarantined node %s guid " FMT_U64 "",
			 sm_nodeDescString(qnodep), upstreamPortp->portData->portInfo.NeighborNodeGUID);
		status = VSTATUS_BAD; goto bail;
	}

	// sanity check port count, mainly to prevent simulator abuse
	if (nodeInfo.NumPorts > MAX_STL_PORTS) {
		IB_LOG_ERROR_FMT(__func__,
			"Node " FMT_U64 " exceeded %d port limit", nodeInfo.NodeGUID, MAX_STL_PORTS);
		status = VSTATUS_BAD; goto bail;
	}

	portGuid = nodeInfo.PortGUID;
	portNumber = nodeInfo.u1.s.LocalPortNum;

	if (local) {
		portNumber = sm_config.port;
	}

	//
	// Branch based on whether we've seen this node already during this
	// discovery.
	//
	if (new_node) {
		uint32 quarantineReasons = 0x00000000;
		uint8_t discnodestatus = 0x00;
		char errStr[256] = { '\0' };

		conPIp = &conPortInfo;

		// On re-sweep, attempt to re-use cached data for HFIs.
		if (!local && sm_config.use_cached_hfi_node_data && use_cache && cache_nodep
				&& cache_nodep->nodeInfo.NodeType == NI_TYPE_CA
				&& upstreamNodep->nodeInfo.NodeType == NI_TYPE_SWITCH
				&& cache_nodep->nodeInfo.NodeGUID == upstreamPortp->portData->portInfo.NeighborNodeGUID
				&& sm_valid_port(cache_portp)) {
			// If the port hasn't changed we can just copy the data from the old topology!
			nodeDesc = cache_nodep->nodeDesc;
			conPortInfo = cache_portp->portData->portInfo;
			conPortInfo.LocalPortNum = upstreamPortp->portData->portInfo.NeighborPortNum;
			// LWD may have changed. Instead of checking manually, fill it in with the connected
			// switch ports LWD value.
			conPortInfo.LinkWidthDowngrade.TxActive = upstreamPortp->portData->portInfo.LinkWidthDowngrade.TxActive;
			conPortInfo.LinkWidthDowngrade.RxActive = upstreamPortp->portData->portInfo.LinkWidthDowngrade.RxActive;
			//LedInfo may have changed, update in case
			conPIp->PortStates.s.LEDEnabled = upstreamNodep->portStateInfo[upstreamPortp->index].PortStates.s.LEDEnabled;
		} else if (!local && sm_config.use_cached_node_data && use_cache && cache_nodep
				&& cache_nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
				&& cache_nodep->nodeInfo.NodeGUID == upstreamPortp->portData->portInfo.NeighborNodeGUID
				&& sm_valid_port(cache_portp)) {
			nodeDesc = cache_nodep->nodeDesc;
			conPortInfo = cache_portp->portData->portInfo;
			conPortInfo.LocalPortNum = upstreamPortp->portData->portInfo.NeighborPortNum;
			switchInfo = cache_nodep->switchInfo;
		} else {
			if (!sm_popo_is_nonresp_this_sweep(&sm_popo, noresp_nodep)) {
				psc_unlock(psc);
				status = _get_discovery_node_attributes(fd, path, portNumber, &nodeDesc, conPIp,
					(_get_trusted_node_type(upstreamPortp, &nodeInfo) == NI_TYPE_SWITCH) ? &switchInfo : NULL,
					errStr, &discnodestatus);
				psc_lock(psc);
			} else {
				// If port was previously nonResp this Sweep, then use cache for NodeDesc/PortInfo
				status = VSTATUS_TIMEOUT;
				discnodestatus = NODEDESC_UNAVAILABLE | PORTINFO_UNAVAILABLE;
				sprintf(errStr, "Nonresponsive, skipping Get NodeDesc and PortInfo");
			}
			// If there was a failure this sweep and we can use the cache, then try.
			if (sm_popo_inc_and_use_cache_nonresp(&sm_popo, noresp_nodep, &status)
				&& discnodestatus != 0)
			{
				if (discnodestatus & NODEDESC_UNAVAILABLE) {
					memcpy(&nodeDesc, &noresp_nodep->nodeDesc, sizeof(STL_NODE_DESCRIPTION));
					discnodestatus &= ~NODEDESC_UNAVAILABLE;
				}
				if (discnodestatus & PORTINFO_UNAVAILABLE) {
					memcpy(conPIp, &noresp_portp->portData->portInfo, sizeof(STL_PORT_INFO));
					discnodestatus &= ~PORTINFO_UNAVAILABLE;
				}
				// if no more errors reset error status
				if (discnodestatus == 0) {
					status = VSTATUS_OK;
					IB_LOG_INFINI_INFO_FMT(__func__, "%s for node off Port %d of Node"FMT_U64":%s; using cached data",
						errStr, upstreamPortp->index, upstreamNodep->nodeInfo.NodeGUID,
						sm_nodeDescString(upstreamNodep));
					errStr[0] = '\0';
				}
			} else if (status == VSTATUS_OK) {
				sm_popo_clear_cache_nonresp(&sm_popo, noresp_nodep);
			}
		}

		if (*errStr) {
			if (local) {
				IB_LOG_WARN_FMT(__func__, "%s for local node", errStr);
			} else {
				IB_LOG_WARN_FMT(__func__, "%s for node off Port %d of Node"FMT_U64":%s",
					errStr, upstreamPortp->index, upstreamNodep->nodeInfo.NodeGUID,
					sm_nodeDescString(upstreamNodep));
			}
			if (status == VSTATUS_UNRECOVERABLE || status == VSTATUS_TIMEOUT_LIMIT)
				goto bail;
		}

		// authenticate the node in order to determine whether to allow the
		// node to be part of the fabric.
		authenticNode = _stl_authentic_node(topop, upstreamNodep, upstreamPortp, &nodeInfo, conPIp,
			(nodeInfo.NodeType == NI_TYPE_SWITCH) ? &switchInfo : NULL,
			&quarantineReasons, discnodestatus);

		if (discnodestatus & NODEDESC_UNAVAILABLE) {
			StringCopy((char *) nodeDesc.NodeString, "Not Available", STL_NODE_DESCRIPTION_ARRAY_SIZE);
		}

		// If the node is authentic and pre-defined topology is enabled, verify
		// the node against the input topology.
		if (authenticNode && sm_config.preDefTopo.enabled) {
			// If the Connected Port is Armed or Active, the port is going to be
			// bounced so, force log only as Warn.
			boolean forceLogAsWarn = (conPIp->PortStates.s.PortState > IB_PORT_INIT) ? TRUE : FALSE;
			authenticNode = _validate_predef_fields(topop, pdtop, upstreamNodep, upstreamPortp,
				&nodeInfo, &nodeDesc, &quarantineReasons, &expNodeInfo, forceLogAsWarn);
		}

		nodep = Node_Create(topop, &nodeInfo, nodeInfo.NodeType, nodeInfo.NumPorts + 1, &nodeDesc,
			authenticNode);
		if (nodep == NULL) {
			IB_LOG_ERROR_FMT(__func__, "cannot create a new node for GUID: " FMT_U64,
							 nodeInfo.NodeGUID);
			status = VSTATUS_EIO; goto bail;
		}

		memcpy((void *) nodep->path, (void *) path, 64);

		// if node fails authentication, then quarantine the attacker node.
		if (!authenticNode) {
			status = VSTATUS_BAD;
			if (!local) {
				// Bounce any ports on node which are Armed or Active
				if (upstreamPortp->portData->portInfo.PortNeighborMode.NeighborNodeType == STL_NEIGH_NODE_TYPE_SW) {
					//node type may be spoofed so set it to correct value:
					nodep->nodeInfo.NodeType = NI_TYPE_SWITCH;
					status = sm_bounce_all_switch_ports(fd, topop, nodep, NULL, path, psc);
					if(status!=VSTATUS_OK) {
						IB_LOG_WARN_FMT(__func__, "Unable to bounce potentially"
							" active ports on quarantined switch %s. Status: %d",
							sm_nodeDescString(nodep), status);
					}

				} else { // HFI
					//node type may be spoofed so set it to correct value:
					nodep->nodeInfo.NodeType = NI_TYPE_CA;
					// if it is a HFI, the max num of ports is 1
					nodep->nodeInfo.NumPorts = 1;
					if(upstreamPortp->portData->portInfo.PortStates.s.PortState > IB_PORT_INIT) {
						status = sm_bounce_link(topop, upstreamNodep, upstreamPortp);
					}
				}
			}
			if (status != VSTATUS_OK) {
				// Quarantine Node only if the bounce did not happen, as the
				// bounce should set the port to DOWN and should be picked up
				// and rechecked for quarantine Next sweep when it is back in
				// INIT
				(void)_stl_quarantine_node(topop, upstreamNodep, upstreamPortp, nodep,
					quarantineReasons, &expNodeInfo, conPIp);
			} else {
				if (!local) {
					_stl_quarantine_reasons(topop, upstreamNodep, upstreamPortp, nodep,
						quarantineReasons, &expNodeInfo, conPIp);
					IB_LOG_WARN_FMT(__func__, "Link was bounced instead of quarantined to disable potentially unsecure traffic.");
				}
			}
			status = VSTATUS_BAD; goto bail;
		}

		if (status != VSTATUS_OK) {
			if (local)
				status = VSTATUS_UNRECOVERABLE;
			else
				status = sm_popo_port_error(&sm_popo, topop, upstreamPortp, status);

			goto bail;
		}

		//Once the node is created, it passed the authentication and we
		//validated switchinfo copy switchinfo.
		if (nodeInfo.NodeType == NI_TYPE_SWITCH) {
			// all switch specific information has been retreived
			nodep->switchInfo = switchInfo;
		}

		nodep->index = topop->num_nodes++;

		// Check to see if the node existed in the previous topology.
		if (cache_nodep) {
			oldnodep = cache_nodep;
		} else if (noresp_nodep) {
			oldnodep = noresp_nodep;
		} else {
			oldnodep = sm_find_guid(&old_topology, nodeInfo.NodeGUID);
		}

		if (oldnodep != NULL) {
			nodep->oldIndex = oldnodep->index;
			nodep->oldExists = 1;
			nodep->old = oldnodep;
			oldnodep->old = NULL;
		}

		/* keep track of how many switch chips in fabric and which are newly added */
		if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			if (oldnodep != NULL) {
				nodep->swIdx = oldnodep->swIdx;
			} else {
				lastIdx = -1;
				nextIdx = bitset_find_first_zero(&old_switchesInUse);
				while ((nextIdx < 0) || bitset_test(&new_switchesInUse, nextIdx)) {
					if (nextIdx == -1) {
						if (!bitset_resize
							(&old_switchesInUse, old_switchesInUse.nbits_m + SM_NODE_NUM)
							|| !bitset_resize(&new_switchesInUse,
							new_switchesInUse.nbits_m + SM_NODE_NUM)) {
							// Can't track deltas, do full route check.
							topology_changed = 1;
							break;
						}
						if (lastIdx == -1) {
							nextIdx = bitset_find_first_zero(&old_switchesInUse);
						} else {
							nextIdx = bitset_find_next_zero(&old_switchesInUse, lastIdx);
						}
					}
					lastIdx = nextIdx;
					nextIdx = bitset_find_next_zero(&old_switchesInUse, nextIdx + 1);
				}
				if (nextIdx < 0) {
					// flag error
					nodep->swIdx = topop->num_sws;
				} else {
					nodep->swIdx = nextIdx;
				}
			}
			topop->num_sws++;
			topop->max_sws = MAX(topop->max_sws, nodep->swIdx + 1);
			bitset_set(&new_switchesInUse, nodep->swIdx);

		}

		// track all new switches and endnodes for LID additions
		if (!oldnodep) {
			if (sm_mark_new_endnode(nodep) != VSTATUS_OK)
				topology_changed = 1;
		} else {
			_check_for_new_endnode(&nodeInfo, nodep, oldnodep);
		}

		nodep->asyncReqsSupported = sm_config.sma_batch_size;


		// Add NODE to sorted topo tree here
		if (cl_qmap_insert
			(sm_newTopology.nodeIdMap, (uint64_t) nodep->index,
			 &nodep->nodeIdMapObj.item) != &nodep->nodeIdMapObj.item) {
			IB_LOG_ERROR_FMT(__func__,
				"Error adding Node Id: 0x%x to tree. Already in tree!",
				nodep->index);
		} else {
			cl_qmap_set_obj(&nodep->nodeIdMapObj, nodep);
		}
		if (cl_qmap_insert
			(sm_newTopology.nodeMap, nodep->nodeInfo.NodeGUID,
			 &nodep->mapObj.item) != &nodep->mapObj.item) {
			IB_LOG_ERROR_FMT(__func__,
				"Error adding Node GUID: " FMT_U64 " to tree. Already in tree!",
				nodep->nodeInfo.NodeGUID);
		} else {
			cl_qmap_set_obj(&nodep->mapObj, nodep);
		}

	} else { // existing node
		sm_popo_update_node(nodep->ponodep);

		uint32 quarantineReasons = 0x00000000;

		// attacker may attempt to use valid node GUID of an existing node, so
		// authenticate the node in order to determine whether to continue to
		// allow the node to be part of the fabric.
		authenticNode = _stl_authentic_node(topop, upstreamNodep, upstreamPortp, &nodeInfo, NULL,
			NULL, &quarantineReasons, 0);
		nodeDesc = nodep->nodeDesc;

		// If the node is authentic and pre-defined topology is enabled, verify
		// the node against the input topology.
		if(authenticNode && sm_config.preDefTopo.enabled) {
			authenticNode = _validate_predef_fields(topop, pdtop, upstreamNodep, upstreamPortp,
				&nodeInfo, &nodeDesc, &quarantineReasons, &expNodeInfo, FALSE);
		}

		if (!authenticNode) {
			Node_t *attackerNodep;

			// get the node description from this attacker node, for it too may
			// have been spoofed
			SmpAddr_t addr = SMP_ADDR_CREATE_DR(path);
			psc_unlock(psc);
			status = SM_Get_NodeDesc(fd, 0, &addr, &nodeDesc);
			psc_lock(psc);
			if (status != VSTATUS_OK) {
				StringCopy((char *) nodeDesc.NodeString, "Not Available",
				STL_NODE_DESCRIPTION_ARRAY_SIZE);
			}

			// create new node as a placeholder for the attacker node
			attackerNodep = Node_Create(topop, &nodeInfo, nodeInfo.NodeType,
						nodeInfo.NumPorts + 1, &nodeDesc, authenticNode);
			if (attackerNodep == NULL) {
				IB_LOG_ERROR_FMT(__func__,
					"cannot create a new node for attacker GUID: " FMT_U64,
					nodeInfo.NodeGUID);
				status = VSTATUS_EIO; goto bail;
			}

			//save path to attacker node
			memcpy((void *) attackerNodep->path, (void *) path, 64);
			// quarantine the attacker node
			(void) _stl_quarantine_node(topop, upstreamNodep, upstreamPortp, attackerNodep,
				quarantineReasons, &expNodeInfo, NULL);
			status = VSTATUS_BAD; goto bail;
		}

		if (!sm_config.loopback_mode && (nodep->nodeInfo.NodeType != nodeInfo.NodeType ||
				((nodep->nodeInfo.NodeType == NI_TYPE_CA && nodeInfo.NodeType == NI_TYPE_CA) &&
				(nodep->nodeInfo.PortGUID == nodeInfo.PortGUID || nodeInfo.u1.s.LocalPortNum == nodep->nodeInfo.u1.s.LocalPortNum)))) {
			char nodeDescStr[STL_NODE_DESCRIPTION_ARRAY_SIZE];
			memcpy(nodeDescStr, nodeDesc.NodeString, STL_NODE_DESCRIPTION_ARRAY_SIZE);
			nodeDescStr[STL_NODE_DESCRIPTION_ARRAY_SIZE - 1] = '\0';
			IB_LOG_ERROR_FMT(__func__,
				"Duplicate NodeGuid for Node %s nodeType[%d] guid " FMT_U64 " and "
				"existing node[%d] nodeType=%d, %s, guid " FMT_U64,
				nodeDescStr, nodeInfo.NodeType, nodeInfo.NodeGUID,
				nodep->index, nodep->nodeInfo.NodeType, sm_nodeDescString(nodep),
				nodep->nodeInfo.NodeGUID);

			status = VSTATUS_BAD; goto bail;
		}

		// potentially new port on existing node; check previous topology
		//
		// only applies to FIs... we can hit a switch multiple times, but the
		// end node is always the same port zero.  hitting a FI always
		// results in a different end port
		if (nodep->nodeInfo.NodeType == NI_TYPE_CA)
			_check_for_new_endnode(&nodeInfo, nodep, NULL);
	}

	//
	// Initialize the port and link this node/port into the topology links.
	//
	if ((portp = sm_get_port(nodep, portNumber)) == NULL) {
		IB_LOG_ERROR_FMT(__func__, "get portNumber %d for node %s failed",
			portNumber, sm_nodeDescString(nodep));
		status = VSTATUS_BAD; goto bail;
	}

	if (sm_dynamic_port_alloc()) {
		/* allocate port record associated with the primary port of the node */
		if ((portp->portData = sm_alloc_port(topop, nodep, portNumber)) == NULL) {
			IB_LOG_ERROR_FMT(__func__, "cannot create primary port %d for node %s",
				portNumber, sm_nodeDescString(nodep));
			status = VSTATUS_BAD; goto bail;
		}
	}

	if (!new_node) {
		// This port may have failed to discover for various reasons (e.g.
		// a failed Get(PortInfo)), or may have been legitimately DOWN at
		// the time of discovery.
		if (portp->state <= IB_PORT_DOWN) {
			IB_LOG_INFINI_INFO_FMT(__func__,
				"Discovering node %s nodeGuid "FMT_U64" via inbound port %u which previously failed discovery.",
				sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID, portNumber);
			status = VSTATUS_BAD; goto bail;
		}

		conPIp = &portp->portData->portInfo;
	}

	// There can be a race condition where the state of local port of the
	// connected node doesn't match its neighbor's because of the time
	// that elapsed between getting the neighbor's info and getting this
	// info. This can lead to inconsistencies programming the fabric.
	// If this happens, mark the link as down, we'll fix it in the next
	// sweep.
	if (!local &&
		((upstreamPortp->state == IB_PORT_ACTIVE && conPIp->PortStates.s.PortState <= IB_PORT_INIT  ) ||
		 (upstreamPortp->state <= IB_PORT_INIT   && conPIp->PortStates.s.PortState == IB_PORT_ACTIVE)))
	{
		IB_LOG_INFINI_INFO_FMT(__func__, "Port states mismatched for"
			" port[%d] of Node "FMT_U64 ":%s (State=%s), connected to port[%d]"
			" of Node "FMT_U64 ":%s (State=%s)",
			upstreamPortp->index, upstreamNodep->nodeInfo.NodeGUID,
			sm_nodeDescString(upstreamNodep), IbPortStateToText(upstreamPortp->state),
			portNumber, nodeInfo.NodeGUID, nodeDesc.NodeString,
			IbPortStateToText(conPIp->PortStates.s.PortState));
		// sweep_discovery() will mark the link down
		sm_request_resweep(0, 0, SM_SWEEP_REASON_ROUTING_FAIL);
		status = VSTATUS_BAD; goto bail;
	}

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		if ((swPortp = sm_get_port(nodep, 0))) {
			if (sm_dynamic_port_alloc()) {
				/* allocate port record associated with the switch port */
				if (!(swPortp->portData = sm_alloc_port(topop, nodep, 0))) {
					IB_LOG_ERROR_FMT(__func__,
						"failed to allocate switch port %d for node %s", 0,
						sm_nodeDescString(nodep));
					status = VSTATUS_BAD; goto bail;
				}
			}
			swPortp->portData->guid = portGuid;
			// Not really necessary, but for consistency.  Default wire depth.
			swPortp->portData->initWireDepth = -1;
		} else {
			IB_LOG_ERROR_FMT(__func__, "failed to get switch port %d for node %s",
				0, sm_nodeDescString(nodep));
			status = VSTATUS_BAD; goto bail;
		}

	} else {
		portp->portData->guid = portGuid;
		// Default wire depth.
		portp->portData->initWireDepth = -1;
	}

	if (new_node != 0) {
		portp->state = IB_PORT_DOWN;
	}

	if (portp->path[0] == 0xff) {
		(void) memcpy((void *) portp->path, (void *) path, 64);
	}

	if ((nodeInfo.NodeType == NI_TYPE_SWITCH) && (swPortp->path[0] == 0xff)) {
		(void) memcpy((void *) swPortp->path, (void *) path, 64);
	}

	if (!local) {
		upstreamPortp->nodeno = nodep->index;
		upstreamPortp->portno = portp->index;

		portp->nodeno = upstreamNodep->index;
		portp->portno = upstreamPortp->index;

		if (upstreamNodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
				if (!portp->portData->isIsl) {
					portp->portData->isIsl = 1;
					nodep->numISLs++;
				}
				if (!upstreamPortp->portData->isIsl) {
					upstreamPortp->portData->isIsl = 1;
					upstreamNodep->numISLs++;
				}
				if (nodep->nodeInfo.SystemImageGUID != upstreamNodep->nodeInfo.SystemImageGUID) {
					upstreamNodep->externalLinks = 1;
					nodep->externalLinks = 1;
				} else if (upstreamNodep != nodep) {
					upstreamNodep->internalLinks = 1;
					nodep->internalLinks = 1;
				}
			} else {
				upstreamNodep->edgeSwitch = 1;
			}
		} else if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			nodep->edgeSwitch = 1;
		}
	}

	// We may have seen this node before, but not had both sides of the link set up
	// properly to evaluate link policy, so check again now.
	if(new_node == 0) {
		if(!sm_valid_port(portp)) {
			status = VSTATUS_BAD; goto bail;
		}

		if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH
			|| (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
			&& (portp->index > 0))) {

			if(sm_verifyPortSpeedAndWidth(topop, nodep, portp) != VSTATUS_OK) {
				sm_mark_link_down(topop, portp);
				status = VSTATUS_BAD; goto bail;
			}
		}
	}


	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		if (!local && upstreamPortp->state == IB_PORT_INIT) {
			if (upstreamNodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
				topology_changed = 1;
			} else if (!topology_changed) {
				// Check for case when new HFI port was linked to an ISL on the
				// last sweep.
				Node_t *oldUpstreamNodep = sm_find_guid(&old_topology, upstreamNodep->nodeInfo.NodeGUID);
				if (oldUpstreamNodep) {
					Port_t *oldUpstreamPortp = sm_get_port(oldUpstreamNodep, upstreamPortp->index);
					if (sm_valid_port(oldUpstreamPortp) &&
						oldUpstreamPortp->state >= IB_PORT_INIT &&
						oldUpstreamPortp->portData->isIsl) {
						topology_changed = 1;
					}
				}
			}
		}
		//
		// If we have already seen this node, just return.
		//
		if (new_node == 0) {
			status = VSTATUS_OK; goto bail;
		}
	}

	//
	// Get the information from all of its ports.  If this node/port is a
	// Master SM, then we do the comparison to see if we should still be
	// the SM.
	//
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {

		start_port = 0;
		end_port = nodep->nodeInfo.NumPorts;

		// if the switch supports adaptive routing, then allocate adaptive
		// routing related structures
		if (switchInfo.CapabilityMask.s.IsAdaptiveRoutingSupported) {
			if (switchInfo.AdaptiveRouting.s.Enable) {
				if (sm_adaptiveRouting.debug) {
					IB_LOG_INFINI_INFO_FMT(__func__,
						"Switchinfo for node %s guid " FMT_U64
						" indicates Adaptive Routing support",
						sm_nodeDescString(nodep),
						nodep->nodeInfo.NodeGUID);
				}
			}
			if (sm_adaptiveRouting.enable) {
				nodep->arSupport = 1;
				// We round up the allocation to a whole block.
				if ((status = vs_pool_alloc(&sm_pool,
					sizeof(STL_PORTMASK) * ROUNDUP(switchInfo.PortGroupCap+1,
					NUM_PGT_ELEMENTS_BLOCK),
					(void *) &nodep->pgt)) == VSTATUS_OK) {
					memset((void *) nodep->pgt, 0,
						sizeof(STL_PORTMASK) * (switchInfo.PortGroupCap));
				}
				if (status != VSTATUS_OK) {
					nodep->arSupport = 0;
					IB_LOG_WARN0("No memory for "
						"adaptive routing support");
				}
			}
		}
		if (nodep->switchInfo.u1.s.PortStateChange) {
			if (bitset_test(&old_switchesInUse, nodep->swIdx)) {
				// Set indicator that need to check for link down to switch.
				switchPortChange = 1;
			}
			if (!oldnodep) {
				oldnodep = sm_find_guid(&old_topology, nodeInfo.NodeGUID);
			}
			if (oldnodep) {
				/* save the portChange value in the old node in case this sweep gets
				aborted */
				oldnodep->switchInfo.u1.s.PortStateChange = 1;
			}
			topology_switch_port_changes = 1;
		}
		if (!topology_switch_port_changes) {
			/* Check if portChange flag is set in the old topology. If it is
			 * still set then the previous sweep had been aborted at some
			 * point. Set the topology_switch_port_changes flag to ensure MFTs
			 * are programmed. */
			if (!oldnodep) {
				oldnodep = sm_find_guid(&old_topology, nodeInfo.NodeGUID);
			}
			if (oldnodep && oldnodep->switchInfo.u1.s.PortStateChange) {
				topology_switch_port_changes = 1;
			}
		}

		// Fill in our PortStateInfo information for this switch
		status = sm_get_node_port_states(fd, topop, nodep, portp, path,
			&portStateInfo, psc);
		if (status != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__, "Unable to get PortStateInfo information"
				" for switch %s. Status: %d", sm_nodeDescString(nodep), status);
			nodep->portStateInfo = NULL;
			if ((status = sm_popo_port_error(&sm_popo, topop, swPortp, status)) == VSTATUS_TIMEOUT_LIMIT) {
				goto bail;
			}
		} else {
			nodep->portStateInfo = portStateInfo;
		}
	} else {
		start_port = portNumber;
		end_port = portNumber;
	}

	STL_PORT_INFO *newPortInfos;
	bitset_t needPortInfo, skipPorts, newlyDownPorts;

	vs_pool_alloc(&sm_pool, sizeof(STL_PORT_INFO) * (end_port + 1), (void *)&newPortInfos );
	bitset_init(&sm_pool, &needPortInfo, end_port + 1);
	bitset_init(&sm_pool, &skipPorts, end_port + 1);
	bitset_init(&sm_pool, &newlyDownPorts, end_port + 1);

	for (i = start_port; i <= end_port; i++) {
		if ((portp = sm_get_port(nodep, i)) == NULL) {
			IB_LOG_ERROR_FMT(__func__, "get port %d for node %s nodeGuid "
				FMT_U64" failed", i, sm_nodeDescString(nodep),
				nodep->nodeInfo.NodeGUID);
			continue;
		}

		if (sm_dynamic_port_alloc()) {
			if ((portp->portData = sm_alloc_port(topop, nodep, i)) == NULL) {
				IB_LOG_ERROR_FMT(__func__, "cannot create port %d for node %s"
					" nodeGuid "FMT_U64, i, sm_nodeDescString(nodep),
					nodep->nodeInfo.NodeGUID);
				continue;
			}
			// Default wire depth.
			portp->portData->initWireDepth = -1;
		}

		/*
		 * Note: Only attempt to put the port GUID in the tree if it is
		 * non-zero (CA ports and switch port 0) Check for FI based SM port
		 * being inserted in the map twice (SM node = tp->node_head) This is to
		 * get around a quickmap abort when a duplicate portGuid is inserted in
		 * map
		 */
		if ((portp->portData->guid && nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) ||
			(portp->portData->guid
			 && (nodep != sm_topop->node_head
				 || !(sm_find_port_guid(&sm_newTopology, portp->portData->guid))))) {
			if (cl_qmap_insert
				(sm_newTopology.portMap, portp->portData->guid,
				 &portp->portData->mapObj.item) != &portp->portData->mapObj.item) {
				IB_LOG_ERROR_FMT(__func__,
					"Error adding Port GUID: " FMT_U64
					" to tree. Already in tree!", portp->portData->guid);
			} else {
				cl_qmap_set_obj(&portp->portData->mapObj, portp);
			}
		} else {
			if (portp->portData->guid) {
				bitset_set(&skipPorts, i);
				continue;		/* already seen this port, don't get its portInfo again */
			}
		}

		if (use_cache) {
			if(nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && cache_nodep) {
				cache_portp = sm_get_port(cache_nodep, i);

				if (sm_valid_port(cache_portp) && nodep->portStateInfo) {
					// If the port hasn't changed we can just copy the data from the old topology!
					if(cache_portp->state == nodep->portStateInfo[i].PortStates.s.PortState) {
						newPortInfos[i] = cache_portp->portData->portInfo;
						//LedInfo may have changed, update in case
						newPortInfos[i].PortStates.s.LEDEnabled = nodep->portStateInfo[i].PortStates.s.LEDEnabled;
						//The Tx/RxActive values may have changed. Update in case.
						newPortInfos[i].LinkWidthDowngrade.TxActive = nodep->portStateInfo[i].LinkWidthDowngradeTxActive;
						newPortInfos[i].LinkWidthDowngrade.RxActive = nodep->portStateInfo[i].LinkWidthDowngradeRxActive;
						continue;
					}
				}
			}
		}

		if (portp->index==portNumber) {
			// Save the extra read since we've already read it.
			// and we know the read was good.
			newPortInfos[i] = *conPIp;
			continue;
		}

		//If this port's state has changed from active->init since last sweep,
		//add it to port quarantine monitor list - it may be a flapping port.
		if (portStateInfo && portStateInfo[i].PortStates.s.PortState != IB_PORT_ACTIVE) {
			oldnodep = sm_find_guid(&old_topology, nodep->nodeInfo.NodeGUID);
			if (oldnodep) {
				oldportp = sm_get_port(oldnodep, i);
				if (sm_valid_port(oldportp)) {
					if(oldportp->portData->portInfo.PortStates.s.PortState >
						portStateInfo[i].PortStates.s.PortState) {
						sm_popo_monitor_port(&sm_popo, portp,
							POPO_LONGTERM_FLAPPING);

						// If PortState dropped back to INIT/DOWN, get the LinkDownReason
						bitset_set(&newlyDownPorts, i);
						bitset_set(&needPortInfo, i);
					}
				}
			}
		}

		// If this port is down, then we don't care about most of it's portData
		// and portInfo contents, so just jump to the part where we clean it up.
		if (portStateInfo && portStateInfo[i].PortStates.s.PortState == IB_PORT_DOWN) {
			sm_mark_link_down(topop, portp);
			continue;
		}

		bitset_set(&needPortInfo, i);
	}

	if (!sm_config.use_aggregates) {
		status = _get_portinfo_loop(fd, topop, nodep, &needPortInfo,
			 newPortInfos, psc);
	} else {
		status = _get_portinfo_aggr(fd, topop, nodep, &needPortInfo,
			newPortInfos, psc);
		if (status != VSTATUS_OK && status != VSTATUS_TIMEOUT_LIMIT) {
			status = _get_portinfo_loop(fd, topop, nodep, &needPortInfo,
				newPortInfos, psc);
		}
	}

	if (status == VSTATUS_TIMEOUT_LIMIT) {
		bitset_free(&needPortInfo);
		bitset_free(&skipPorts);
		bitset_free(&newlyDownPorts);
		vs_pool_free(&sm_pool, newPortInfos);
		goto bail;
	}

	for (i = start_port; i <= end_port; i++) {
		portInfo = &newPortInfos[i];
		if ((portp = sm_get_port(nodep, i)) == NULL) {
			IB_LOG_ERROR_FMT(__func__, "get port %d for node %s failed",
				i, sm_nodeDescString(nodep));
			continue;
		}

		if (bitset_test(&skipPorts, i)) continue;

		if (portStateInfo && portStateInfo[i].PortStates.s.PortState == IB_PORT_DOWN
			&& !bitset_test(&newlyDownPorts, i))
		{
			goto cleanup_down_ports;
		}

		// There was a problem getting this port
		if (bitset_test(&needPortInfo, i)) {
			/* indicate a fabric change has been detected to insure paths-lfts-etc are
			   recalculated */
			if (topology_passcount && nodep->nodeInfo.NodeType == NI_TYPE_SWITCH)
				topology_changed = 1;
			IB_LOG_WARN_FMT(__func__,
							"Failed to get PortInfo from NodeGUID " FMT_U64
							" [%s] Port %d; Ignoring port!", nodep->nodeInfo.NodeGUID,
							sm_nodeDescString(nodep), portp->index);
			sm_mark_link_down(topop, portp);
			goto cleanup_down_ports;
		}

		if (portInfo->DiagCode.s.UniversalDiagCode != 0) {
			IB_LOG_ERROR_FMT(__func__,
							 "DiagCode of node %s index %d port %d: chain %d vendor %d universal %d",
							 sm_nodeDescString(nodep), nodep->index, portp->index,
							 portInfo->DiagCode.s.Chain, portInfo->DiagCode.s.VendorDiagCode,
							 portInfo->DiagCode.s.UniversalDiagCode);

			if (portInfo->DiagCode.s.UniversalDiagCode == DIAG_HARD_ERROR) {
				sm_mark_link_down(topop, portp);
				goto cleanup_down_ports;
			}
		}

		if (nodep->nodeInfo.NodeType == NI_TYPE_CA ||
			(nodep->nodeInfo.NodeType == NI_TYPE_SWITCH && i == 0)) {
			if (portInfo->CapabilityMask3.s.VLSchedulingConfig ==
					VL_SCHED_MODE_VLARB) {
				nodep->vlArb = 1;
			}
		}


		/* set to correct gid prefix in portp structure - we'll do the portInfo later */
		if (portInfo->SubnetPrefix != sm_config.subnet_prefix) {
			portp->portData->gidPrefix = sm_config.subnet_prefix;
		} else {
			portp->portData->gidPrefix = portInfo->SubnetPrefix;
		}
		(void) memcpy((void *) &portp->portData->gid[0], (void *) &portp->portData->gidPrefix,
					  8);
		(void) memcpy((void *) &portp->portData->gid[8], (void *) &portp->portData->guid, 8);
		*(uint64_t *) & portp->portData->gid[0] =
			ntoh64(*(uint64_t *) & portp->portData->gid[0]);
		*(uint64_t *) & portp->portData->gid[8] =
			ntoh64(*(uint64_t *) & portp->portData->gid[8]);
		portp->portData->capmask = portInfo->CapabilityMask.AsReg32;
		portp->portData->capmask3 = portInfo->CapabilityMask3.AsReg16;
		portp->portData->lid = portInfo->LID;
		portp->portData->lmc = portInfo->s1.LMC;
		portp->portData->mtuSupported = portInfo->MTU.Cap;
		portp->state = portInfo->PortStates.s.PortState;

		oldnodep = sm_find_guid(&old_topology, nodep->nodeInfo.NodeGUID);
		if (oldnodep) {
			/* Copy over trap related counters from old topology so that they
 			 * don't get lost with every sweep. Also copy over the
 			 * LinkDownReason(s) if there are any */
			oldportp = sm_get_port(oldnodep, i);
			if (sm_valid_port(oldportp)) {
				portp->portData->numFailedActivate = oldportp->portData->numFailedActivate;
				portp->portData->trapWindowStartTime = oldportp->portData->trapWindowStartTime;
				portp->portData->trapWindowCount = oldportp->portData->trapWindowCount;
				portp->portData->lastTrapTime = oldportp->portData->lastTrapTime;
				portp->portData->suppressTrapLog = oldportp->portData->suppressTrapLog;
				portp->portData->logSuppressedTrapCount =
					oldportp->portData->logSuppressedTrapCount;
				portp->portData->logTrapSummaryThreshold =
					oldportp->portData->logTrapSummaryThreshold;
				portp->portData->linkPolicyViolation  =
					oldportp->portData->linkPolicyViolation;

				// Copy wire depth from old port data.
				portp->portData->initWireDepth = oldportp->portData->initWireDepth;

				if(nodep->nodeInfo.NodeType == NI_TYPE_CA &&
					portp->portData->lid != oldportp->portData->lid) {
					/* Don't warn about ports that merely bounced since the last sweep. */
					if (portp->state > IB_PORT_INIT || portp->portData->lid != 0) {
						IB_LOG_WARN_FMT(__func__,
							"Node %s (GUID: " FMT_U64 ") attempted to change"
							" LID on port %d from 0x%x to 0x%x."
							" Resetting to previous LID.",
							sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
							portp->index, oldportp->portData->lid, portp->portData->lid);
					}
					portp->portData->lid = oldportp->portData->lid;
					portp->portData->dirty.portInfo=1;
				}
			}
		}

		/* save the portInfo and LinkDownReason*/
		portp->portData->portInfo = newPortInfos[i];
		sm_popo_update_port_state_with_ldr(&sm_popo, portp,
			&portp->portData->portInfo.PortStates,
			portp->portData->portInfo.LinkDownReason,
			portp->portData->portInfo.NeighborLinkDownReason);

		if (portStateInfo && portStateInfo[i].PortStates.s.PortState == IB_PORT_DOWN
			&& bitset_test(&newlyDownPorts, i))
		{
			goto cleanup_down_ports;
		}


		/***** applicable to all non management ports *****/
		if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH
			|| (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
			&& (portp->index > 0))) {

			if(sm_verifyPortSpeedAndWidth(topop, nodep, portp) != VSTATUS_OK) {
				sm_mark_link_down(topop, portp);
				goto cleanup_down_ports;
			}

		}

		if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH &&
			portp->portData->portInfo.PortStates.s.PortState != IB_PORT_INIT &&
			portp->portData->portInfo.CapabilityMask3.s.IsMAXLIDSupported &&
			portp->portData->portInfo.MaxLID > 0 && portp->portData->portInfo.MaxLID < STL_GET_UNICAST_LID_MAX() ) {

			sm_bounce_link(topop, nodep, portp);
			sm_mark_link_down(topop, portp);
			goto cleanup_down_ports;
		}


		if (portInfo->PortStates.s.PortState == IB_PORT_INIT) {
			if (nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
				if (sm_mark_new_endnode(nodep) != VSTATUS_OK)
					topology_changed = 1;
			}
		}

		portp->portData->vl0 = portInfo->VL.s2.Cap;
		/* we will only support one block of 8 guids for now */
		portp->portData->guidCap = PORT_GUIDINFO_DEFAULT_RECORDS;	/* portInfo.GUIDCap; */
		portp->portData->rate = linkWidthToRate(portp->portData);

		portp->portData->portSpeed = sm_GetSpeed(portp->portData);

		if (i == start_port || (nodep->index == 0 && i == sm_config.port)) {
			int newLidCount;
			Node_t *neighbor = sm_find_node(topop, portp->nodeno);
			sm_update_or_assign_lid(portp, 0, &newLidCount, neighbor);

			if (sm_config.sm_debug_lid_assign) {
				IB_LOG_INFINI_INFO_FMT(__func__,
				   "assign lid to port " FMT_U64 ": lid 0x%x, lmc 0x%x",
				   portGuid, portp->portData->lid, portp->portData->lmc);
			}

			if (newLidCount) {
				if (sm_mark_new_endnode(nodep) != VSTATUS_OK)
					topology_changed = 1;
			}
		}

		if ((i > 0) && (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH)) {
			portp->portData->lid = STL_LID_RESERVED;
		}

		if ((portp->portData->vl0 < 1) || (portp->portData->vl0 > STL_MAX_VLS)) {
			portp->portData->vl0 = 1;
		}

		if ((nodep->index != 0) || (portp->index != sm_config.port)) {
			if ((portp->portData->capmask & PI_CM_IS_SM) != 0) {
				/* get and save the SmInfo of the SM node for later processing */
				/* Note: If we encountered a temporarily non-responding node, skip this step */
				status = _get_sminfo(topop, nodep, portp);
				if (status == VSTATUS_TIMEOUT_LIMIT) {
					bitset_free(&needPortInfo);
					bitset_free(&skipPorts);
					bitset_free(&newlyDownPorts);
					vs_pool_free(&sm_pool, newPortInfos);
					goto bail;
				}
			}
		}

	  cleanup_down_ports:
		/* keep track of live ports */
		if (portp->state >= IB_PORT_INIT) {
			bitset_set(&nodep->activePorts, portp->index);
			INCR_PORT_COUNT(topop, nodep);

			if (portp->state == IB_PORT_INIT) {
				bitset_set(&nodep->initPorts, portp->index);
				nodep->portsInInit = nodep->initPorts.nset_m;
			}

			// if applicable, take it out of the list of removed ports
			(void) sm_removedEntities_clearPort(nodep, portp);
		} else if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
			if (sm_dynamic_port_alloc() && !portp->portData->linkPolicyViolation) {
				/* free port record associated with the DOWN ports of switches */
				/* except when port is only down administatively */
				sm_free_port(topop, portp);
			}
			if (!topology_changed) {
				if (!oldnodep) {
					oldnodep = sm_find_guid(&old_topology, nodeInfo.NodeGUID);
				}
				if (oldnodep) {
					if (switchPortChange || oldnodep->switchInfo.u1.s.PortStateChange) {
						oldportp = sm_get_port(oldnodep, portp->index);
						if (oldportp && (oldportp->state >= IB_PORT_INIT)) {
							linkednodep = sm_find_node(&old_topology, oldportp->nodeno);
							if (linkednodep
								&& (linkednodep->nodeInfo.NodeType == NI_TYPE_SWITCH)) {
								// Was up and connected to a switch.  Flag topology change.
								topology_changed = 1;
								/*
								   IB_LOG_INFINI_INFO_FMT(__func__, "Switch reporting
								   port change, node %s guid "FMT_U64" link down port=%d",
								   sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
								   portp->index); */
							}
						}
					}
				}
			}
		}
	}

	bitset_free(&needPortInfo);
	bitset_free(&skipPorts);
	bitset_free(&newlyDownPorts);
	vs_pool_free(&sm_pool, newPortInfos);

	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		start_port = 1;
	}

	if (new_node == 1) {
		// initialize DGs for the node
		smSetupNodeDGs(nodep);

		// initialize VFs for the node
		smSetupNodeVFs(nodep);

	}

	nodep->aggregateEnable = sm_config.use_aggregates;
	if (oldnodep)
		nodep->aggregateEnable &= oldnodep->aggregateEnable;

bail:
	if (psc) {
		psc_unlock(psc);
	}

	if (fd != fd_topology) {
		psc_free_mai(psc,mpp);
	}

	if (nnp && new_node) *nnp = nodep;
	IB_EXIT(__func__, status);
	return status;
}

// Special case of _setup_node that is invoked on the local port.
// No support for parallelism or addressing.
//
Status_t
sm_setup_local_node(Topology_t * topop, FabricData_t * pdtop)
{
	DiscoveryWorkItem_t wi = {{{0}}};
	return _setup_node(topop, pdtop, NULL, NULL, &wi);
}

// Allocates a new work queue item.
static DiscoveryWorkItem_t *
_discover_workitem_alloc(DiscoveryContext_t *disc_ctx, Node_t *nodep,
	Port_t *portp, PsWorker_t workfunc)
{
	DiscoveryWorkItem_t *disc_workitem = NULL;
	if (vs_pool_alloc(&sm_pool, sizeof(DiscoveryWorkItem_t),
		(void**)&disc_workitem) != VSTATUS_OK) {
		return NULL;
	}
	memset(disc_workitem, 0, sizeof(DiscoveryWorkItem_t));

	disc_workitem->upstream_nodep = nodep;
	disc_workitem->upstream_portp = portp;
	disc_workitem->item.workfunc = workfunc;
	disc_workitem->ctx = disc_ctx;

	return disc_workitem;
}

// Assumes the PDI has already been removed from the discovery queue.
// Assumes the PSC lock is held (to cover shared access to the dup map).
//
static void
_discover_workitem_free(DiscoveryWorkItem_t *disc_workitem)
{
	// If this item was added to the duplicate check map, remove it.
	if (disc_workitem->dup_item.key != 0) {
		cl_qmap_remove_item(&disc_workitem->ctx->dup_map, &disc_workitem->dup_item);
	}

	vs_pool_free(&sm_pool, disc_workitem);
}

// When a node is discovered, the path to discovering it is saved as the DR
// path to that node. If we allow discovery to run in a fully parallel manner
// we can end up with non-optimial DR paths. In the worst case this could lead
// to nodes appearing to be more than 64 hops away when they are not.
//
// To prevent this, we use barriers to ensure that all nodes of "N" hops
// from the SM are discovered before discovering nodes that are "N+1" hops
// away. Barriers can do that quite elegantly, since we know the very first
// node discovered is distance 0, if we put a barrier after it, it will be
// discovered before nodes of distance 1. Once PSC begins discovering distance
// 1 nodes, it appends a new barrier before queueing any distance 2 nodes,
// and so on.
//
// This can actually be done neatly by having the barrier worker check to see
// if new nodes have been added to the work queue since the last barrier was
// invoked and, if so, enqueueing a new barrer at the current end of the work
// queue.
//
static void
_barrier_worker(ParallelSweepContext_t* psc, ParallelWorkItem_t* pwi)
{
	DiscoveryWorkItem_t *disc_item = PARENT_STRUCT(pwi, DiscoveryWorkItem_t, item);
	if (disc_item->ctx->new_level_found) {
		// It would be cool if we could just re-use the old barrier item
		// but we can't because it's still in the work queue, acting as a barrier.
		// So, we have to create a new one.
    	DiscoveryWorkItem_t *barrier = _discover_workitem_alloc(disc_item->ctx,
        	NULL, NULL, _barrier_worker);
    	if (barrier == NULL) {
        	IB_LOG_ERROR_FMT(__func__, "Failed to allocate discovery work item.");
        	psc_set_status(psc, VSTATUS_NOMEM);
			psc_stop(psc);
    	} else {
			disc_item->ctx->new_level_found = 0;
			psc_add_barrier(psc, &barrier->item);
		}
	}
}

static void
_discover_worker(ParallelSweepContext_t* psc, ParallelWorkItem_t* pwi)
{
	Status_t status = VSTATUS_OK;
	Node_t *new_np = NULL;
	Port_t *new_pp = NULL;
	unsigned start_port, end_port;
	DiscoveryWorkItem_t *disc_item = PARENT_STRUCT(pwi, DiscoveryWorkItem_t, item);

	psc_lock(psc);
	if (disc_item->path[0] == 0) {
		// Special case - the local node was discovered in sweep_initialize.
		new_np = sm_topop->node_head;
	} else {
		psc_unlock(psc);
		// setup_node is responsible for managing the topology mutex.
		status = _setup_node(sm_topop, &preDefTopology, &new_np, psc, disc_item);
		psc_lock(psc);
		if (status == VSTATUS_NOT_MASTER) {
			IB_LOG_INFO("now running as a STANDBY SM", sm_state);
			status = sm_topop->routingModule->funcs.post_process_discovery(
				sm_topop, VSTATUS_NOT_MASTER, disc_item->ctx->routing_context);
			goto bail;
		} else if (status != VSTATUS_OK) {
			IB_LOG_WARN_FMT(__func__,
				" unable to setup port[%d] of node %s, nodeGuid "FMT_U64
				", ignoring port!", disc_item->upstream_portp->index,
				sm_nodeDescString(disc_item->upstream_nodep),
				disc_item->upstream_nodep->nodeInfo.NodeGUID);
			sm_mark_link_down(sm_topop, disc_item->upstream_portp);
			topology_changed = 1;	/* indicates a fabric change has been detected */

#ifndef __VXWORKS__
			// Self-quarantining not implementing for ESM due to ESM
			// presumably having multiple ports to the fabric
			Node_t *sm_nodep = sm_topop->node_head;
			Port_t *sm_portp = sm_get_port(sm_nodep, sm_config.port);
			if (disc_item->upstream_nodep == sm_nodep && disc_item->upstream_portp == sm_portp &&
					sm_portp && sm_portp->portData->neighborQuarantined) {
				sm_peer_quarantined = 1;
				IB_LOG_ERROR_FMT(__func__, "SM neighbor is quarantined."
					" Cannot manage fabric. Transitioning to NOTACTIVE.");
				sm_transition(SM_STATE_NOTACTIVE);
				psc_stop(psc);
				goto bail;
			}
#endif

			if (status == VSTATUS_TIMEOUT_LIMIT) {
				IB_LOG_ERROR_FMT(__func__, "Cumulative timeout limit"
					" exceeded: abandoning discovery");
				psc_stop(psc);
				goto bail;
			}
		}
	}

	if ((!new_np) || ((new_np != sm_topop->node_head) && (new_np->nodeInfo.NodeType != NI_TYPE_SWITCH))) {
		// Our work here is done...
		goto bail;
	}

	// At this point the node is either the local HSM node or a switch.
	// If we exploring a switch, we have a lot of egress ports to examine.
	if (new_np->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		start_port = 1;
		end_port = new_np->nodeInfo.NumPorts;
	} else {
		start_port = sm_config.port;
		end_port = sm_config.port;
	}

	if (disc_item->path[0] >= 62) {
		/* This means the end node is 63 hops away. The SM must be within 62
		 * hops (allowing SM to look 1 node beyond end of fabric).
		 */
		IB_LOG_ERROR_FMT(__func__,
			"NodeGuid "FMT_U64" [%s] is 63 hops away from the SM. "
			"Maximum allowed hops is 62.", disc_item->upstream_nodep->nodeInfo.NodeGUID,
			sm_nodeDescString(disc_item->upstream_nodep));
		goto bail;
	}

	status = sm_topop->routingModule->funcs.discover_node(sm_topop,
		new_np, disc_item->ctx->routing_context);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to process 'discover node' routing hook; rc:", status);
		psc_stop(psc);
		goto bail;
	}

	// Create work items for the neighbors of this node.  Do this as the last
	// step, but retain the lock so that this is atomic w.r.t. removing the
	// GUID from dup_map, providing the invariant that new work items represent
	// a completed (i.e. "not in progress") upstream node.
	int p;
	for (p = start_port; p <= end_port && psc_is_running(psc); p++) {
		new_pp = sm_get_port(new_np, p);

		if (!new_pp || new_pp->state < IB_PORT_INIT) {
			// Link is down.
			continue;
		} else if (new_pp->nodeno != -1 && new_pp->nodeno != -1) {
			// Neighbor has already been set up.
			continue;
		}

		DiscoveryWorkItem_t *disc_workitem = _discover_workitem_alloc(
			disc_item->ctx, new_np, new_pp, _discover_worker);
		if (disc_workitem == NULL) {
			continue;
		}

		memcpy(disc_workitem->path, disc_item->path, 64);
		disc_workitem->path[0]++;
		disc_workitem->path[disc_workitem->path[0]] = new_pp->index;
		disc_workitem->ctx->new_level_found = 1;

		psc_add_work_item(psc, &disc_workitem->item);
	}

bail:
	psc_set_status(psc, status);
	_discover_workitem_free(disc_item);

	// Note the assumption that we hold the lock when we get here.
	psc_unlock(psc);
}

Status_t
sweep_discovery(SweepContext_t *sweep_context)
{
	Node_t		*nodep;
	Port_t		*portp;
	Status_t	status;
	STL_SMINFO_RECORD sminforec={{0}, 0};
	DiscoveryContext_t disc_ctx;

	IB_ENTER(__func__, 0, 0, 0, 0);

	if (sm_topop->routingModule == NULL) {
		status = vs_wrlock(&old_topology_lock);
		if (status != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Failed to acquire old_topology_lock");
			return status;
		}

		status = sm_routing_makeCopy(&sm_topop->routingModule,
			old_topology.routingModule);

		if (status != VSTATUS_OK)
			IB_LOG_ERROR_FMT(__func__, "Failed to copy prior routing module data");

		Status_t unlockStatus = vs_rwunlock(&old_topology_lock);
		if (unlockStatus != VSTATUS_OK) {
			IB_LOG_ERROR_FMT(__func__, "Error on unlocking old_topology_lock");
			return unlockStatus;
		}

		if (status != VSTATUS_OK)
			return status;
	}

	// Initialize parallel sweep context
	cl_qmap_init(&disc_ctx.dup_map, NULL);
	disc_ctx.new_level_found=0;

	status = sm_topop->routingModule->funcs.pre_process_discovery(sm_topop,
		&disc_ctx.routing_context);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to process 'pre-discovery' routing hook; rc:", status);
		goto bail;
	}

	//
	// Find out about the Node that I am on.
	//
	nodep = sm_topop->node_head;
	if (!nodep) {
		IB_LOG_ERROR_FMT(__func__,"Local node has not been initialized.");
		status = VSTATUS_BAD;
		goto bail;
	}
	if (!sm_valid_port((portp = sm_get_port(nodep,sm_config.port)))) {
		IB_LOG_ERROR0("failed to get SM port");
		sm_topop->routingModule->funcs.post_process_discovery(sm_topop, VSTATUS_BAD, disc_ctx.routing_context);
		status = VSTATUS_BAD;
		goto bail;
	}

	vs_time_get(&sm_newTopology.sweepStartTime);
	sm_popo_begin_sweep(&sm_popo, sm_newTopology.sweepStartTime/VTIMER_1S);

	//
	//	Start a directed route exploration of the fabric.
	//

	// Normally this is the guid of the neighbor of portp but
	// the local node is a special case. In that case we are discovering
	// ourselves.
	DiscoveryWorkItem_t *self = _discover_workitem_alloc(&disc_ctx,
		NULL, NULL, _discover_worker);
	if (self == NULL) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate discovery work item.");
		status = VSTATUS_NOMEM;
		goto bail;
	}
	DiscoveryWorkItem_t *barrier = _discover_workitem_alloc(&disc_ctx,
		NULL, NULL, _barrier_worker);
	if (barrier == NULL) {
		IB_LOG_ERROR_FMT(__func__, "Failed to allocate discovery work item.");
		status = VSTATUS_NOMEM;
		goto bail;
	}


	// Enqueues the local port, without triggering the workers to being
	// discovery.  We have to add the first work item without triggering,
	// then add the barrier, then trigger the worker threads. This is only
	// needed for the first barrier.
	
	psc_stop(sweep_context->psc);
	psc_add_work_item_no_trigger(sweep_context->psc, &self->item);
	psc_add_barrier(sweep_context->psc, &barrier->item);
	psc_go(sweep_context->psc);
	psc_trigger(sweep_context->psc);

	//
	// Wait for parallel discovery to finish.
	//
	status = psc_wait(sweep_context->psc);

	// Drain the work queue (in case discovery terminated early).
	// Because the duplicate check map requires special handling
	// we don't use psc_drain_work_queue() here.
	psc_stop(sweep_context->psc);
	while(!QListIsEmpty(&sweep_context->psc->work_queue)) {
		LIST_ITEM *li = QListRemoveHead(&sweep_context->psc->work_queue);
		if (li) {
			DiscoveryWorkItem_t *dwi = PARENT_STRUCT(li, DiscoveryWorkItem_t, item);
			_discover_workitem_free(dwi);
		}
	}

	if (topology_main_exit) {
		status = VSTATUS_OK;
		goto bail;
	}

	// topology graph is complete but node list order will vary when multiple
	// discovery threads are in use.  re-order node lists according to BFS
	if ((status = _reorder_node_lists(sm_topop)) != VSTATUS_OK) {
		IB_LOG_ERROR_FMT(__func__, "Failed to enforce BFS node order: status %u", status);
		IB_EXIT(__func__, status);
		return status;
	}

	// compute maximums and invoke routing hooks (after bfs reordering to
	// prevent routing engines from seeing nondeterministic orders)
	for_all_nodes(sm_topop, nodep) {
		for_all_physical_ports(nodep, portp) {
			if (sm_valid_port(portp) && portp->state >= IB_PORT_INIT) {
				Node_t *nnodep = sm_find_node(sm_topop, portp->nodeno);
				if (!nnodep) continue;

				Port_t *nportp = sm_get_port(nnodep, portp->portno);

				// update fabric MTU & rate for use in multicast group
				// creation/join requests, and the maximum ISL MTU and rate we
				// have seen so far we only care about ISL's so that's all we're
				// checking.
				if (sm_valid_port(nportp)
					&& nodep->nodeInfo.NodeType == NI_TYPE_SWITCH
					&& nnodep->nodeInfo.NodeType == NI_TYPE_SWITCH ) {
					uint32_t rate = portp->portData->rate;

					if (sm_mc_config.disable_mcast_check == McGroupBehaviorStrict) {
						// First time through, the maxVlMtu is not setup yet.
						// So don't check it against the maxMcastMtu until the port
						// maxVlMtu is non-zero.
						if ((portp->portData->maxVlMtu !=0) &&
							(portp->portData->maxVlMtu < sm_topop->maxMcastMtu)) {
							sm_topop->maxMcastMtu = portp->portData->maxVlMtu;
						}

						if (linkrate_lt(rate, sm_topop->maxMcastRate)) {
							sm_topop->maxMcastRate = rate;
						}
					}

					if (portp->portData->maxVlMtu > sm_topop->maxISLMtu) {
						sm_topop->maxISLMtu = portp->portData->maxVlMtu;
					}

					if (linkrate_gt(rate, sm_topop->maxISLRate)) {
						sm_topop->maxISLRate = rate;
					}
				}

				// Let the routing module see this port.
				if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
					status = sm_topop->routingModule->funcs.discover_node_port(sm_topop,
						nodep, portp, disc_ctx.routing_context);
					if (status != VSTATUS_OK) {
						IB_LOG_ERRORRC("Failed to process 'discover node/port'"
							" routing hook; rc:", status);
					}
				}
			}
		}
	}

	status = sm_topop->routingModule->funcs.post_process_discovery(sm_topop,
		psc_get_status(sweep_context->psc), disc_ctx.routing_context);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("Failed to process 'post-discovery' routing hook; rc:", status);
		goto bail;
	}

	// Do not become master of a fabric where our own port is down
	// This prevents an odd situation where a SM with an unstable link becomes
	// master when its link is down, and later when its link is up, it has an
	// elevated priority and becomes master for the whole fabric.  Which would
	// be contrary to the goals of sticky failover.
	// Note that port DOWN won't happen for SWE0
	if (sm_topop->node_head->nodeInfo.NodeType != NI_TYPE_SWITCH) {
		portp = sm_get_port(sm_topop->node_head, sm_config.port);
		if (  ! sm_valid_port(portp) || portp->state <= IB_PORT_DOWN) {
			if (sm_state == SM_STATE_MASTER)
				(void)sm_transition(SM_STATE_STANDBY);
			status = VSTATUS_NOT_MASTER;
			goto bail;
		}
	}

	if (sm_config.sm_debug_vf) {
 		if (topology_passcount <= 2) smLogVFs();
	}

	// did we remove any ports?  if so, abort sweep
	if (sm_topop->numRemovedPorts) {
		IB_LOG_WARN_FMT(__func__,
		       "Removed %d port(s) from the fabric; will initiate re-sweep",
		       sm_topop->numRemovedPorts);
		status = VSTATUS_BAD;
		goto bail;
	}

	// If SM port LID was not registered before, find a LID for it now.
	// SM port LID assignment may evict an already-registered LID range
	// if necessary to avoid the SM getting "locked out" of the fabric.
	portp = sm_get_port(sm_topop->node_head,sm_config.port);
	if (sm_update_or_assign_lid(portp, 1, NULL, NULL) != VSTATUS_OK) {
		IB_LOG_ERROR0("failed to find/register LID for SM port");
		sm_topop->routingModule->funcs.post_process_discovery(sm_topop,
			VSTATUS_BAD, disc_ctx.routing_context);
		status = VSTATUS_BAD;
		goto bail;
	}

	/*
	 * add ourselves to the SM list during first sweep
	 */
	if (!topology_passcount) {
		sminforec.RID.LID = sm_lid;
		memcpy((void *)&sminforec.SMInfo, (void *)&sm_smInfo, sizeof(sminforec.SMInfo));
		if (!sm_valid_port((portp = sm_get_port(sm_topop->node_head,sm_config.port)))) {
			IB_LOG_ERROR0("failed to get SM port");
			status = VSTATUS_BAD;
			goto bail;
		}
		(void) sm_dbsync_addSm(sm_topop->node_head, portp, &sminforec);
	}

	// discovery went well, build the node array
	status = sm_build_node_array(sm_topop);
	if (status != VSTATUS_OK)
		IB_LOG_INFINI_INFORC("failed to build node array: node lookup "
			"optimization is disabled for this sweep. rc:", status);

	// This was moved out of the above loop because that loop only considers
	// switch nodes.  This can be moved back in if that loop doesn't skip
	// non-switch nodes
	if (sm_config.cableInfoPolicy > CIP_NONE) {
		for_all_nodes(sm_topop, nodep) {
			for_all_ports(nodep, portp) {
				if (sm_Port_t_IsCableInfoSupported(portp) && portp->state == IB_PORT_ACTIVE) {

					// CableInfo_t is reference counted, so acquire mutex
					// before copying May not be necessary if all other threads
					// are read only and don't care about the reference count
					// since this process will never result in a CableInfo_t
					// being freed
					if (vs_wrlock(&old_topology_lock) == VSTATUS_OK) {
						Node_t * oldNode = sm_find_guid(&old_topology, nodep->nodeInfo.NodeGUID);

						Port_t * oldPort = NULL;

						if (oldNode)
							oldPort = sm_get_port(oldNode, portp->index);

						if (sm_valid_port(oldPort) && oldPort->portData->cableInfo) {
							portp->portData->cableInfo = sm_CableInfo_copy(oldPort->portData->cableInfo);

							if (!portp->portData->cableInfo) {
								IB_LOG_ERROR_FMT(__func__, "Failed to copy cable "
									"data from old_topology to current topology"
									".  Node %s node GUID "FMT_U64" port %d",
									sm_nodeDescString(nodep), nodep->nodeInfo.NodeGUID,
									portp->index);
							}
						}

						vs_rwunlock(&old_topology_lock);
					}
				}
			}
		}
	}

bail:
	IB_EXIT(__func__, VSTATUS_OK);
	return status;
}
