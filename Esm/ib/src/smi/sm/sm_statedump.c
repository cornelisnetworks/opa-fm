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
//===========================================================================//
//
// FILE NAME
//    sm_counters.c
//
// DESCRIPTION
//
// DATA STRUCTURES
//    None
//
// FUNCTIONS
//
// DEPENDENCIES
//
//
//===========================================================================//

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include "sm_l.h"

#define PATH_BUF_SZ 1024

#define TOPO_FNAME "topology"
#define NODE_FNAME "nodes"
#define PORT_FNAME "ports"
#define LIDMAP_FNAME "lidMap"
#define MCGRP_FNAME "mcGroups"
#define MCMEMBER_FNAME "mcMembers"
#define MCCLS_FNAME "mcClasses"
#define MCTREE_FNAME "mcSpanningTrees"

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

static char pathBuffer[PATH_BUF_SZ];

#define MAPADDR_FILENAME_LEN 32
typedef struct _MapAddress
{
	void * memAddr;
	char file[MAPADDR_FILENAME_LEN];
	uint64_t fileOffset;
} MapAddress;

typedef Status_t (*dumpfunc_t)(const char * dumpDir, FILE * mapFile);

Status_t dumpOldTopology(const char * dumpDir, FILE * mapFile);
Status_t dumpMcGroups(const char * dumpDir, FILE * mapFile);

dumpfunc_t dumpFunctions[] = {
	dumpOldTopology,
	dumpMcGroups,
	NULL
};

//
//
//
static Status_t dumpStructure(void * ptr, size_t size, FILE * file,
                              const char * fileName, FILE * mapFile)
{
	MapAddress addr;
	long pos = 0;

	if (ptr != NULL)
	{
		pos = ftell(file);

		// Allign on a 64 bit boundary
		if (pos % 8)
			pos += (8 - (pos % 8));

		if (fseek(file, pos, SEEK_SET))
			return VSTATUS_BAD;

		addr.memAddr = ptr;
		strncpy(addr.file, fileName, sizeof(addr.file)-1);
		addr.file[sizeof(addr.file)-1]=0;
		addr.fileOffset = pos;

		if (fwrite(ptr, size, 1, file) != 1)
		{
			IB_LOG_ERROR_FMT(__func__, "Write of structure at address %p "
			       " (size = %"PRISZT" failed: %s", ptr, size, strerror(errno));
			return VSTATUS_BAD;
		}

		if (fwrite(&addr, sizeof(addr), 1, mapFile) != 1)
		{
			IB_LOG_ERROR_FMT(__func__, "Write of address failed: %s",
			       strerror(errno));
			return VSTATUS_BAD;
		}
	}

	return VSTATUS_OK;
	
}

//
//
//
Status_t dumpTopologyStruct(const char * dumpDir, FILE * mapFile)
{
	Status_t rc = VSTATUS_OK;
	FILE * topoFile = NULL, * lidFile = NULL;

	
	snprintf(pathBuffer, PATH_BUF_SZ, "%s/%s", dumpDir, TOPO_FNAME);
	if ((topoFile = fopen(pathBuffer, "a")) == NULL)
		return VSTATUS_BAD;

	snprintf(pathBuffer, PATH_BUF_SZ, "%s/%s", dumpDir, LIDMAP_FNAME);
	if ((lidFile = fopen(pathBuffer, "a")) == NULL)
	{
		fclose(topoFile);
		return VSTATUS_BAD;
	}

	printf("Dumping topology\n");
	if (VSTATUS_OK != (rc = dumpStructure(&old_topology, sizeof(old_topology),
	                                      topoFile, TOPO_FNAME, mapFile)))
		goto bail;

	printf("Dumping topology.cost\n");
	if (VSTATUS_OK != (rc = dumpStructure(old_topology.cost,
	                                      old_topology.bytesCost, topoFile,
	                                      TOPO_FNAME, mapFile)))
		goto bail;

	printf("Dumping topology.path\n");
	if (VSTATUS_OK != (rc = dumpStructure(old_topology.path,
	                                      old_topology.bytesPath, topoFile,
	                                      TOPO_FNAME, mapFile)))
		goto bail;

	printf("Dumping topology.nodeArray\n");
	if (VSTATUS_OK != (rc = dumpStructure(old_topology.nodeArray,
	                                      old_topology.num_nodes,
	                                      topoFile, TOPO_FNAME, mapFile)))
		goto bail;

	// now we dump the lidmap

	printf("Dumping lidmap\n");
	if (VSTATUS_OK != (rc = dumpStructure(lidmap, sizeof(LidMap_t) *
	                                              (UNICAST_LID_MAX + 1),
	                                      lidFile, LIDMAP_FNAME, mapFile)))
		goto bail;

 bail:
	fclose(lidFile);
	fclose(topoFile);
	return rc;
}

//
//
//
Status_t dumpTopologyNodes(const char * dumpDir, FILE * mapFile)
{
	Status_t rc = VSTATUS_OK;
	FILE * nodeFile = NULL, * portFile = NULL;
	Node_t * node = NULL;
	Port_t * port = NULL;
	size_t len;

	snprintf(pathBuffer, PATH_BUF_SZ, "%s/%s", dumpDir, NODE_FNAME);
	if ((nodeFile = fopen(pathBuffer, "a")) == NULL)
		return VSTATUS_BAD;

	snprintf(pathBuffer, PATH_BUF_SZ, "%s/%s", dumpDir, PORT_FNAME);
	if ((portFile = fopen(pathBuffer, "a")) == NULL)
	{
		fclose(nodeFile);
		return VSTATUS_BAD;
	}

	for_all_nodes(&old_topology, node)
	{
		printf("-> %s\n", (char*) sm_nodeDescString(node));
		printf("Dumping node\n");
		if (VSTATUS_OK != (rc = dumpStructure(node, sizeof(Node_t), nodeFile,
		                                      NODE_FNAME, mapFile)))
			goto bail;

		if (node->nodeInfo.NodeType == NI_TYPE_SWITCH)
		{
			printf("Dumping node.lft\n");
			len = sizeof(uint8_t) * ROUNDUP(node->switchInfo.LinearFDBTop+1, MAX_LFT_ELEMENTS_BLOCK);
			if (VSTATUS_OK != (rc = dumpStructure(node->lft, len, nodeFile,
			                                      NODE_FNAME, mapFile)))
				goto bail;

			printf("Dumping node.mft[]\n");
			len = sizeof(STL_PORTMASK *) * sm_mcast_mlid_table_cap;
			if (VSTATUS_OK != (rc = dumpStructure(node->mft, len, nodeFile,
			                                      NODE_FNAME, mapFile)))
				goto bail;
			
			printf("Dumping node.mft[][]\n");
			len = sizeof(STL_PORTMASK) * sm_mcast_mlid_table_cap * STL_NUM_MFT_POSITIONS_MASK;
			printf("MFTable at %p\n", node->mft[0]);
			if (VSTATUS_OK != (rc = dumpStructure(node->mft[0], len, nodeFile,
			                                      NODE_FNAME, mapFile)))
				goto bail;
		}

		printf("Dumping node.activePorts\n");
		len = node->activePorts.nwords_m * sizeof(uint32_t);
		if (VSTATUS_OK != (rc = dumpStructure(node->activePorts.bits_m, len,
		                                      nodeFile, NODE_FNAME, mapFile)))
			goto bail;

		printf("Dumping node.initPorts\n");
		len = node->initPorts.nwords_m * sizeof(uint32_t);
		if (VSTATUS_OK != (rc = dumpStructure(node->initPorts.bits_m, len,
		                                      nodeFile, NODE_FNAME, mapFile)))
			goto bail;
		
		printf("Dumping node.vfmember\n");
		len = node->vfMember.nwords_m * sizeof(uint32_t);
		if (VSTATUS_OK != (rc = dumpStructure(node->vfMember.bits_m, len,
		                                      nodeFile, NODE_FNAME, mapFile)))
			goto bail;
		
		printf("Dumping node.fullPKeyMember\n");
		len = node->fullPKeyMember.nwords_m * sizeof(uint32_t);
		if (VSTATUS_OK != (rc = dumpStructure(node->fullPKeyMember.bits_m, len,
		                                      nodeFile, NODE_FNAME, mapFile)))
			goto bail;
		
		for_all_ports(node, port)
		{
			printf("Dumping port\n");
			if (VSTATUS_OK !=
			      (rc = dumpStructure(port, sizeof(Port_t), portFile,
			                          PORT_FNAME, mapFile)))
				goto bail;

			if (port->portData)
			{
				printf("Dumping port.portData\n");
				if (VSTATUS_OK !=
				      (rc = dumpStructure(port->portData, sizeof(PortData_t),
				                          portFile, PORT_FNAME, mapFile)))
				goto bail;

				printf("Dumping port.portData.slscMap\n");
				// FIXME: If we ever get more intelligent about allocating
				// sl2vl mappings, this len needs to change
				len = sizeof(STL_SLSCMAP);
				if (VSTATUS_OK != (rc = dumpStructure(&port->portData->slscMap,
				                                      len, portFile,
				                                      PORT_FNAME, mapFile)))
				goto bail;

				printf("Dumping port.portData.vfMember\n");
				len = port->portData->vfMember.nwords_m * sizeof(uint32_t);
				if (VSTATUS_OK !=
				      (rc = dumpStructure(port->portData->vfMember.bits_m,
				                          len, portFile, PORT_FNAME, mapFile)))
				goto bail;
			}
		}
	}

 bail:
	fclose(nodeFile);
	fclose(portFile);
	return rc;
}

//
//
//
Status_t dumpOldTopology(const char * dumpDir, FILE * mapFile)
{
	Status_t rc = VSTATUS_OK;

	if ((rc = vs_rdlock(&old_topology_lock)) == VSTATUS_OK)
	{
		if ((rc = dumpTopologyStruct(dumpDir, mapFile)) != VSTATUS_OK)
			goto bail;

		printf("NODE DUMP\n");

		if ((rc = dumpTopologyNodes(dumpDir, mapFile)) != VSTATUS_OK)
			goto bail;

 bail:
		printf("---> rc = %d\n", rc);
		vs_rwunlock(&old_topology_lock);
	}

	return rc;
}

extern McGroupClass_t mcGroupClasses[];
extern McGroupClass_t defaultMcGroupClass;
extern McSpanningTrees_t spanningTrees[STL_MTU_MAX+1][IB_STATIC_RATE_MAX+1];
extern uint16_t numMcGroupClasses;
//
//
//
Status_t dumpMcGroups(const char * dumpDir, FILE * mapFile)
{
	Status_t rc = VSTATUS_OK;
	McGroup_t * mcgp = NULL;
	McMember_t * mcmb = NULL;
	FILE * groupFile = NULL;
	FILE * memberFile = NULL;
	FILE * classFile = NULL;
	FILE * spanFile = NULL;
	int i = 0, j = 0;
	cl_map_item_t * mi;

	snprintf(pathBuffer, PATH_BUF_SZ, "%s/%s", dumpDir, MCGRP_FNAME);
	if ((groupFile = fopen(pathBuffer, "a")) == NULL)
	{
		return rc;
	}

	snprintf(pathBuffer, PATH_BUF_SZ, "%s/%s", dumpDir, MCMEMBER_FNAME);
	if ((memberFile = fopen(pathBuffer, "a")) == NULL)
	{
		fclose(groupFile);
		return rc;
	}

	snprintf(pathBuffer, PATH_BUF_SZ, "%s/%s", dumpDir, MCCLS_FNAME);
	if ((classFile = fopen(pathBuffer, "a")) == NULL)
	{
		fclose(groupFile);
		fclose(memberFile);
		return rc;
	}

	snprintf(pathBuffer, PATH_BUF_SZ, "%s/%s", dumpDir, MCTREE_FNAME);
	if ((spanFile = fopen(pathBuffer, "a")) == NULL)
	{
		fclose(groupFile);
		fclose(memberFile);
		fclose(classFile);
		return rc;
	}

	if ((rc = vs_lock(&sm_McGroups_lock)) != VSTATUS_OK)
	{
		fclose(groupFile);
		fclose(memberFile);
		fclose(classFile);
		fclose(spanFile);
		return rc;
	}

	for_all_multicast_groups(mcgp)
	{
		if (rc != VSTATUS_OK)
			goto bail;

		printf("Dumping McGroup\n");
		rc = dumpStructure(mcgp, sizeof(*mcgp), groupFile,
		                   MCGRP_FNAME, mapFile);

		for_all_multicast_members(mcgp, mcmb)
		{
			if (rc != VSTATUS_OK)
				goto bail;

			printf("Dumping McMember\n");
			rc = dumpStructure(mcmb, sizeof(*mcmb), memberFile,
			                   MCMEMBER_FNAME, mapFile);
		}
	}

	if (VSTATUS_OK != (rc = dumpStructure(&defaultMcGroupClass,
	                                      sizeof(defaultMcGroupClass),
	                                      classFile, MCCLS_FNAME, mapFile)))
		goto bail;

	for (mi = cl_qmap_head(&defaultMcGroupClass.usageMap);
	     mi != cl_qmap_end(&defaultMcGroupClass.usageMap);
	     mi = cl_qmap_next(mi))
	{
		LidClass_t * lc = PARENT_STRUCT(mi, LidClass_t, mapItem);
		if (VSTATUS_OK != (rc = dumpStructure(lc, sizeof(LidClass_t),
		                                      classFile, MCCLS_FNAME, mapFile)))
			goto bail;

		if (VSTATUS_OK != (rc = dumpStructure(lc->lids,
		                                      sizeof(LidUsage_t) * defaultMcGroupClass.maximumLids,
		                                      classFile, MCCLS_FNAME, mapFile)))
			goto bail;
	}

	for (i = 0; i < numMcGroupClasses; ++i)
	{
		if (VSTATUS_OK != (rc = dumpStructure(&mcGroupClasses[i],
		                                      sizeof(defaultMcGroupClass),
		                                      classFile, MCCLS_FNAME, mapFile)))
			goto bail;

		for (mi = cl_qmap_head(&mcGroupClasses[i].usageMap);
		     mi != cl_qmap_end(&mcGroupClasses[i].usageMap);
		     mi = cl_qmap_next(mi))
		{
			LidClass_t * lc = PARENT_STRUCT(mi, LidClass_t, mapItem);
			if (VSTATUS_OK != (rc = dumpStructure(lc, sizeof(LidClass_t),
			                                      classFile, MCCLS_FNAME, mapFile)))
				goto bail;
			if (VSTATUS_OK != (rc = dumpStructure(lc->lids,
			                                      sizeof(LidUsage_t) * mcGroupClasses[i].maximumLids,
			                                      classFile, MCCLS_FNAME, mapFile)))
				goto bail;
		}
	}

	if (VSTATUS_OK != (rc = dumpStructure(spanningTrees,
	                                      sizeof(McSpanningTrees_t*) * (STL_MTU_MAX + 1) * (IB_STATIC_RATE_MAX + 1),
	                                      spanFile, MCTREE_FNAME, mapFile)))
		goto bail;

	for (i = IB_MTU_256; i <= STL_MTU_MAX; i = getNextMTU(i))
	{
		for (j = IB_STATIC_RATE_MIN; j <= IB_STATIC_RATE_MAX; ++j)
		{
			McSpanningTree_t * t = spanningTrees[i][j].spanningTree;
			if (t != NULL)
			{
				if (VSTATUS_OK != (rc = dumpStructure(t, sizeof(*t), spanFile, MCTREE_FNAME, mapFile)))
					goto bail;

				if (VSTATUS_OK != (rc = dumpStructure(t->nodes, t->num_nodes * sizeof(McNode_t),
				                                      spanFile, MCTREE_FNAME, mapFile)))
					goto bail;
			}
		}
	}
	

bail:
	vs_unlock(&sm_McGroups_lock);

	fclose(groupFile);
	fclose(memberFile);
	fclose(classFile);
	fclose(spanFile);

	return rc;
}

//
//
//
Status_t sm_dump_state(const char * dumpDir)
{
	Status_t status = VSTATUS_OK;
	struct stat st;
	int i = 0;
	FILE * mapFile = NULL;

	if (  (stat(dumpDir, &st) != 0)
	   || (S_ISDIR(st.st_mode) == 0)
	   || (access(dumpDir, R_OK | W_OK | X_OK) != 0) )
	{
		IB_LOG_ERROR_FMT(__func__, "Error, could not dump state to "
		       "directory '%s': %s", dumpDir, strerror(errno));

		status = VSTATUS_BAD;
	}

	snprintf(pathBuffer, PATH_BUF_SZ, "%s/%s", dumpDir, "mapFile");

	if ((mapFile = fopen(pathBuffer, "w")) == NULL)
	{
		status = VSTATUS_BAD;
		IB_LOG_ERROR_FMT(__func__, "Error, could not open file %s: %s",
		       pathBuffer, strerror(errno));
	}

	for (i = 0; (dumpFunctions[i] != NULL) && (status == VSTATUS_OK); ++i)
	{
		status = dumpFunctions[i](dumpDir, mapFile);

		if (status != VSTATUS_OK)
			IB_LOG_ERROR_FMT(__func__, "Error, SM state dump failed at "
			       "stage %d\n", i);
	}

	if (mapFile)
		fclose(mapFile);

	return status;
}
