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

//===========================================================================//
//									     //
// FILE NAME								     //
//    sm_userexits.c							     //
//									     //
// DESCRIPTION								     //
//    This file contains the user exits as outlined in the design doc.	     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    None								     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//									     //
//===========================================================================//

#include "os_g.h"
#include "ib_status.h"
#include "ib_mad.h"
#include "ib_macros.h"
#include "ib_sa.h"
#include "cs_g.h"
#include "mai_g.h"
#include "sm_l.h"

extern	Lock_t		tid_lock;
STL_LID loopPathLidStart = 0;       // will be calculated to next 64 after max lid
STL_LID loopPathLidEnd=0;

int     esmLoopTestOn = 0;
int     esmLoopTestAllowAdaptiveRouting = 0;
int		esmLoopTestFast = 0;
int     esmLoopTestInjectNode = -1;       // set to node index of switch to inject packets for or -1=all switches
int     esmLoopTestNumPkts = 1;
int     esmLoopTestPathLen = DEFAULT_LOOP_PATH_LENGTH;
int     esmLoopTestMinISLRedundancy = 4;	//min. number of different loops in which a single ISL should be included

uint32_t	esmLoopTestTotalPktsInjected = 0;
uint8_t		esmLoopTestInjectEachSweep = DEFAULT_LOOP_INJECT_EACH_SWEEP;
uint8_t		esmLoopTestForceInject = 0;
uint8_t		bkup_loop_path=0;
uint32_t	bkup_path_loopweight=0;

extern void     sm_forceSweep(const char* reason);
extern char * snprintfcat(char * buf, int * len, const char * format, ...);

typedef struct _Switches_Skip_Search {
	uint32_t	*nodes;
	uint32_t	count;
} Switches_Skip_Search_t;

typedef struct _PortLoopUsage {
	uint8_t	count;
	uint8_t	portno;
	Node_t *neighbor;
} PortLoopUsage_t;

//---------------------------------------------------------------------------//

Status_t
sweep_userexit(SweepContext_t *sweep_context) {

	IB_ENTER(__func__, 0, 0, 0, 0);
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
forwarding_userexit(uint16_t *cost, int16_t *path) {

	IB_ENTER(__func__, 0, 0, 0, 0);
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
authorization_userexit(void) {

	IB_ENTER(__func__, 0, 0, 0, 0);
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

Status_t
license_userexit(void) {

	IB_ENTER(__func__, 0, 0, 0, 0);
	IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

static int checkForDupPath(Topology_t *top, uint16_t *nodeIdx, uint8_t *path) {
    int         i,j;
    LoopPath_t  *loopPath;
    int         dup = 0;

    for (loopPath=top->loopPaths; loopPath != NULL && dup==0; loopPath = loopPath->next) {
        if (nodeIdx[0] != loopPath->nodeIdx[0]) continue;
        for (i=1, dup=1; i<=nodeIdx[0] && dup==1; i++) {
            dup = 0;
            for (j=1; j<=loopPath->nodeIdx[0]; j++) {
                if (nodeIdx[i] == loopPath->nodeIdx[j] && path[i] == loopPath->path[j]) {
                    dup = 1;
                    break;
                }
            }
        }
        if (dup) {
            /*IB_LOG_INFO_FMT(__func__,
               "duplicate of loop path lid[0x%x] found.....", loopPath->lid);*/
            return dup;
        }
    }
    return dup;
}

static Status_t buildPathList(Topology_t *top, Node_t *node, Port_t *portp, uint16_t *nodeIdx, uint8_t *path) {
	Status_t	status=VSTATUS_OK;
    LoopPath_t  *loopPath=NULL, *lpLast=NULL, *lpTemp=NULL;

	IB_ENTER(__func__, 0, 0, 0, 0);
 
	if (!esmLoopTestFast) {
	     // check for duplicate paths
    	if (checkForDupPath(top, nodeIdx, path)) return(VSTATUS_OK);
	}
  
    // build up the path list
    top->numLoopPaths++;
    if ((status = vs_pool_alloc(&sm_pool, sizeof(LoopPath_t), (void *)&loopPath)) != VSTATUS_OK) {
        IB_LOG_ERRORLX("can't allocate space for loop Path from node ", node->nodeInfo.NodeGUID);
        return(VSTATUS_BAD);
    }
    memset(loopPath, 0, sizeof(LoopPath_t));
    // move to end of loopPaths to add new one
    if (top->loopPaths == NULL) {
        top->loopPaths = loopPath;
    } else {
        lpTemp = top->loopPaths;
        while (lpTemp != NULL) {
            lpLast = lpTemp;
            lpTemp = lpTemp->next;
        }
        lpLast->next = loopPath;
    }
    loopPathLidEnd = (loopPathLidEnd == 0) ? loopPathLidStart : (loopPathLidEnd+1);
    loopPath->lid = loopPathLidEnd;
	if (esmLoopTestFast) {
		// With the faster loop test version, we do not find all loops exhaustively.
		// In this scenario, to test both send/recv paths on each ISL, we need to 
		// inject packets both for forward (or clock-wise) direction and
		// reverse (or anti-clockwise) direction
		loopPathLidEnd++;	//one more lid for reverse path
	}
    loopPath->startNodeno = node->index;
    // start with path to node from SM node and add loop path
    memcpy(loopPath->path, path, path[0]+1);
    memcpy(loopPath->nodeIdx, nodeIdx, (nodeIdx[0]+1)*sizeof(uint16_t));
    IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

//
// update node lft to include loop paths for loop test
//
void
loopTest_userexit_updateLft(Topology_t *topop, Node_t *cnp) {
    int         i,j,ii=0;
    LoopPath_t  *loopPath;
    Port_t      *uePortp;
    Node_t      *nodep, *nr, *loopNode;
    int         nodeLftUpdatedForLoop;
	Port_t		*outPortp;

    if (!esmLoopTestOn) return;
    //
    // update lft to include loop paths for loop test
    //
	if (smDebugPerf || sm_config.sm_debug_routing) {
		IB_LOG_INFINI_INFO_FMT(__func__,
			   "updating LFT for node[%d:"FMT_U64"], topo->switch_head index=%d.....", 
			   cnp->index, cnp->nodeInfo.NodeGUID, topop->switch_head->index);
	}
    for (loopPath=topop->loopPaths; loopPath != NULL; loopPath = loopPath->next) {
        nodeLftUpdatedForLoop = 0;
        // setup the node's loops first
        for (j=1; j<=loopPath->path[0]; j++) {
            if (cnp->index == loopPath->nodeIdx[j]) {
                nodeLftUpdatedForLoop = 1;
                // update lft for this lid for this switch
                cnp->lft[loopPath->lid] = loopPath->path[j];
            }
			if (esmLoopTestFast) {
				//update lft for the reverse path lid
				outPortp = sm_find_port(topop, loopPath->nodeIdx[j], loopPath->path[j]);
				if (outPortp && (cnp->index == outPortp->nodeno)) {
					cnp->lft[loopPath->lid + 1] = outPortp->portno;
				}
			}
        }

        // see if we have to setup path to the loop's start node from the first switch
		// since its a loop, the start node is the same for the reverse path too
        if (topop->switch_head->index != loopPath->startNodeno && nodeLftUpdatedForLoop == 0) {
            nodep = topop->switch_head;
            // find node record for loop start node
            if ((loopNode = sm_find_node(topop, loopPath->startNodeno)) == NULL) {
                IB_LOG_ERROR_FMT(__func__,
                       "can't find node record for loop path[node index:GUID] %d:"FMT_U64, 
                       nodep->index, nodep->nodeInfo.NodeGUID);
                continue;
            }
            // see if we have to setup path to the node from the first switch
			if (topop->node_head->nodeInfo.NodeType == NI_TYPE_CA) {
				ii = 2;  /* skip first node which is the FI */
			} else {
				ii = 1;  /* first node is switch, star there */
			}
            for (i=ii; i<=loopNode->path[0]; i++) {
                if (nodep->index == cnp->index && nodep->lft[loopPath->lid] == 0xff) {
					if (smDebugPerf) {
						IB_LOG_INFO_FMT(__func__, 
							   "updating node[%d], lft[0x%x] to port %d.....", 
							   nodep->index, loopPath->lid, loopNode->path[i]);
					}
                    nodep->lft[loopPath->lid] = loopNode->path[i];
					if (esmLoopTestFast) {
						nodep->lft[loopPath->lid+1] = loopNode->path[i]; //reverse path
					}
                } 
                if (i < loopNode->path[0]) {
                    // find next node in path
                    if ((uePortp = sm_get_port(nodep,loopNode->path[i])) == NULL) {
                        IB_LOG_ERROR_FMT(__func__,
                               "failed to get port %d at other end of %d:"FMT_U64":%d", 
                               loopNode->path[i], nodep->index, nodep->nodeInfo.NodeGUID, loopNode->path[i]);
                        break;
                    }

                    if ((nr = sm_find_node(topop, uePortp->nodeno)) == NULL) {
                        IB_LOG_ERROR_FMT(__func__,
                               "can't find node record at other end of %d:"FMT_U64":%d (nodeno=%d)", 
                               nodep->index, nodep->nodeInfo.NodeGUID, loopNode->path[i], uePortp->nodeno);
                        break;
                    } else {
                        nodep = nr;
                    }
                }
            }
        }
    }
}

int setLoopTestFastMode(int flag)
{
	if (esmLoopTestOn) {
		IB_LOG_WARN0("Loop Test is already running, please stop loop test before changing fast mode");
		IB_LOG_INFINI_INFO_FMT(__func__,
						   "Current loop test config - FastMode=%d, FastMode MinISLRedundancy=%d, InjectEachSweep=%d, TotalPktsInjected since start=%d",
							esmLoopTestFast, esmLoopTestMinISLRedundancy, esmLoopTestInjectEachSweep,
							esmLoopTestTotalPktsInjected);
		return -1;
	} else {
		if (flag == 1) {
			esmLoopTestFast = 1;
			esmLoopTestInjectEachSweep = 0;	
			esmLoopTestPathLen = 4;
	        IB_LOG_INFINI_INFO("loop test fast mode ENABLED, will NOT inject packets every sweep, path length set to ",
								 esmLoopTestPathLen);
		} else if (flag == 0) {
			esmLoopTestFast = 0;
			esmLoopTestInjectEachSweep = DEFAULT_LOOP_INJECT_EACH_SWEEP;	
			esmLoopTestPathLen = DEFAULT_LOOP_PATH_LENGTH;
	        IB_LOG_INFINI_INFO_FMT(__func__,
							 "loop test fast mode DISABLED, inject packets every sweep = %d, path length set to %d",
							 esmLoopTestInjectEachSweep, esmLoopTestPathLen);
		} else {
			IB_LOG_WARN0("argument should be 1 to turn ON or 0 to turn OFF");
			IB_LOG_INFINI_INFO_FMT(__func__,
						   "Current loop test config - FastMode=%d, FastMode MinISLRedundancy=%d, InjectEachSweep=%d, TotalPktsInjected since start=%d",
							esmLoopTestFast, esmLoopTestMinISLRedundancy, esmLoopTestInjectEachSweep,
							esmLoopTestTotalPktsInjected);
			return -1;
		}
	}

	return 0;
}


void print_LoopCoverage(Topology_t *top)
{
	int arr[256], index, i, printed;
	char buf[512];
	char *p;
	uint32_t count = 0;
	uint32_t loop_count = 0;
	Node_t *node, *n2p;
	Port_t *portp;

	for_all_switch_nodes(top, node) {
		memset(arr, 0, sizeof(arr));
		index = 0;
		for_all_physical_ports(node, portp) {
			if (portp->state <= IB_PORT_DOWN)
				continue;
        	if ((n2p = sm_find_node(top, portp->nodeno)) == NULL) {
                IB_LOG_ERROR("can't find node structure for node index ", portp->nodeno);
            } else if (n2p->nodeInfo.NodeType != NI_TYPE_SWITCH) {
				continue;  /* only interested in switches */
            } else {
				if (!portp->portData->inLoopCount) {
					arr[index] = portp->index;
					index++;
					count++;
				} else {
					loop_count++;
				}
			}
		}
		i = 0;
		p = buf;
		while (arr[i] && i < 50) {
#ifdef __VXWORKS__
			printed = sprintf(p, "%d ", arr[i]);
#else
			printed = snprintf(p, 4, "%d ", arr[i]);
#endif
			p += printed;
			i++;
		}
		if (i) {
			IB_LOG_INFINI_INFO_FMT(__func__, "Switch %s "FMT_U64" %d ISL ports are not part of any loop. Ports : %s", 
								node->nodeDesc.NodeString, node->nodeInfo.NodeGUID, i, buf);
		}
	}

	if (loop_count) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Total %d ISL ports (hence %d ISLs) are in loop paths", 
							loop_count, (loop_count/2));
	}

	if (loop_count && count) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Loop Coverage: Partial - %d ISL ports (hence %d ISLs) are NOT in any loop", 
							count, (count/2));
	} else if (loop_count && !count) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Loop Coverage: Full - all ISL ports covered");
	} else if (!loop_count) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Loop Coverage: ZERO - NO ISL ports covered (loop path length is %d)", esmLoopTestPathLen);
	}

	if (loop_count) {
		IB_LOG_INFINI_INFO_FMT(__func__, "Total %d Loop paths of length <= %d links. MinISLRedundancy is %d.",
								((loopPathLidEnd-loopPathLidStart)+1)/2, esmLoopTestPathLen, esmLoopTestMinISLRedundancy);
		IB_LOG_INFINI_INFO_FMT(__func__, "Total LIDs required for loop paths is %d.",
								(loopPathLidEnd-loopPathLidStart)+1);
	}
}

void free_skip_search(Switches_Skip_Search_t *s)
{
	if (s->nodes) {
		vs_pool_free(&sm_pool, (void *)s->nodes);
		s->nodes = NULL;
	}
}

void free_portList(PortLoopUsage_t **pp)
{
	if (*pp) {
		vs_pool_free(&sm_pool, (void *)(*pp));
		*pp = NULL;
	}
}


void add_to_skip_search(Switches_Skip_Search_t *s, Node_t *parent_node, Node_t *node)
{
	Status_t status;

	if (!s->nodes) {
		if ((status = vs_pool_alloc(&sm_pool, sizeof(uint32_t) * parent_node->nodeInfo.NumPorts, (void *)&(s->nodes))) != VSTATUS_OK) {
			s->nodes = NULL;
			return;
		} else {
    		memset(s->nodes, -1, sizeof(uint32_t) * parent_node->nodeInfo.NumPorts);
		}
		s->count = 0;
	}

	s->nodes[s->count] = node->index;

	s->count++;
}

int switch_needs_search(Switches_Skip_Search_t *s, Node_t *node)
{
	int i;

	if (!s->nodes)
		return 1;

	for (i = 0; i < s->count; i++) {
		if (s->nodes[i] == node->index) {
			return 0;
		}
	}

	return 1;
}


static int compare_port_usage(const void *a, const void *b)
{
	return (((PortLoopUsage_t *)a)->count - ((PortLoopUsage_t *)b)->count);

}

Status_t getISLPortList(Topology_t *top, Node_t *nodep, PortLoopUsage_t **p_arr, int *count)
{
	Port_t *portp;
	PortLoopUsage_t *arr;
	Node_t *neighNode;
	Status_t status;
	int i = 0;

	 if ((status = vs_pool_alloc(&sm_pool, sizeof(PortLoopUsage_t) * nodep->nodeInfo.NumPorts,
				 (void *)&(arr))) != VSTATUS_OK) {
			IB_LOG_ERROR("failed to allocate array for sorting port usage ", status);
			*p_arr = NULL;
			*count = 0;
			return status;
	 } else {
    		memset(arr, 0, sizeof(PortLoopUsage_t) * nodep->nodeInfo.NumPorts);
	 }

	*p_arr = arr;

	for_all_physical_ports(nodep, portp) {
		if (portp->state <= IB_PORT_DOWN)
			continue;

        if ((neighNode = sm_find_node(top, portp->nodeno)) == NULL) {
            IB_LOG_ERROR("can't find node structure for node index ", portp->nodeno);
			continue;
		}

		if (neighNode->nodeInfo.NodeType != NI_TYPE_SWITCH)
			continue;

		arr[i].count = portp->portData->inLoopCount;
		arr[i].portno = portp->index;
		arr[i].neighbor = neighNode;
		i++;
	}

	qsort(arr, i, sizeof(PortLoopUsage_t), compare_port_usage);
	*count = i;

	return VSTATUS_OK;
}

void restore_backup_loop_path(Topology_t *topo, uint16_t *nodeIdx, uint8_t *path, uint16_t *bkup_nodeIdx, uint8_t *bkup_path)
{
	Node_t *neighNode;
	Port_t *portp, *neighPortp;
	int i;
	
	memcpy(path, bkup_path, path[0]+1);
    memcpy(nodeIdx, bkup_nodeIdx, (nodeIdx[0]+1)*sizeof(uint16_t));

	for (i = 1; i < path[0]; i++) {
		portp = sm_find_port(topo, nodeIdx[i], path[i]);
		if (!sm_valid_port(portp))
			continue;
		INC_IN_LOOP_COUNT(portp->portData->inLoopCount);

		neighNode = sm_find_node(topo, portp->nodeno);
		if (!neighNode)
			continue;

		neighPortp = sm_get_port(neighNode, portp->portno);

		if (!sm_valid_port(neighPortp))
			continue;

		INC_IN_LOOP_COUNT(neighPortp->portData->inLoopCount);
	}

	bkup_loop_path = 0;
}

void save_loop_path(Topology_t *topo, uint16_t *bkup_nodeIdx, uint8_t *bkup_path, uint16_t *nodeIdx, uint8_t *path)
{
	Port_t *portp;
	uint8_t	backup = 0;
	uint32_t loop_weight = 0;
	int i;

	for (i = 1; i < path[0]; i++) {
		portp = sm_find_port(topo, nodeIdx[i], path[i]);
		if (!sm_valid_port(portp))
			continue;
		loop_weight += portp->portData->inLoopCount;
	}

	if (bkup_loop_path && loop_weight) {
		if ((loop_weight < bkup_path_loopweight) ||
			 ((loop_weight == bkup_path_loopweight) && (path[0] > bkup_path[0]))) {
			bkup_path_loopweight = loop_weight;
			backup = 1;
		} 
	} else {
		bkup_loop_path = 1;
		bkup_path_loopweight = loop_weight;
		backup = 1;
	}

	if (backup) {
		memcpy(bkup_path, path, path[0]+1);
	    memcpy(bkup_nodeIdx, nodeIdx, (nodeIdx[0]+1)*sizeof(uint16_t));
	}

}

//
//  Quick version - Optimized for time, doesn't try to find all possible
//	loops for each ISL, but limits each ISL to esmLoopTestMinISLRedundancy loops.
//
//  Note - this function is modeled after loopTest_userexit_findPaths and duplicates
//  it a bit.
//  TODO - Since this function is based on loopTest_userexit_findPaths, there is a bit
//		   of repeated code for each hop. This could be re-modeled to make the function
//		   a bit smaller.
//
Status_t
findLoopPaths_quick(Topology_t *top)
{
	uint8_t		path[64], bkup_path[10];
    uint16_t    nodeIdx[64], bkup_nodeIdx[10];
    Node_t      *node;
	Port_t		*portp, *n2portp, *n3portp, *n4portp, *otherPortp;
	Node_t		*n2p, *n3p, *n4p;
	Status_t	status=VSTATUS_OK;
    int         thisNodeIndex=0;
	uint8_t		found = 0, found_from_n3 = 0;
	Switches_Skip_Search_t n1_skip_search, n2_skip_search, n3_skip_search;
	PortLoopUsage_t	*n1Arr, *n2Arr, *n3Arr, *n4Arr;
	int			i, j, k, l;
	int			n1_count = 0, n2_count = 0, n3_count = 0, n4_count = 0;


	IB_ENTER(__func__, 0, 0, 0, 0);

	n1_skip_search.nodes = NULL;
	n2_skip_search.nodes = NULL;
	n3_skip_search.nodes = NULL;

	n1Arr = NULL;
	n2Arr = NULL;
	n3Arr = NULL;
	n4Arr = NULL;

    if (!esmLoopTestOn) return status;
    loopPathLidEnd = 0;             // reset end lid number
    loopPathLidStart = ((sm_topop->maxLid + 64)/64)*64;
    //
    // loop test - find loop paths through the switches
    //
	for_all_switch_nodes(top, node) {
        thisNodeIndex = node->index;
		if (smDebugPerf) {
			IB_LOG_INFO_FMT(__func__,
				   "checking for paths starting from node %d:"FMT_U64,node->index, node->nodeInfo.NodeGUID);
		}
		n1_skip_search.nodes = NULL;
	 	if ((status = getISLPortList(top, node, &n1Arr, &n1_count)) != VSTATUS_OK)
			goto err_exit;

		for (l = 0; l < n1_count; l++) {
			portp = sm_get_port(node, n1Arr[l].portno);
			if (!portp)
				continue;
            if (portp->state <= IB_PORT_DOWN || (portp->portData->inLoopCount >= esmLoopTestMinISLRedundancy))
                continue;
			if (smDebugPerf) {
				IB_LOG_INFO_FMT(__func__,
					   "checking for paths from node %d:"FMT_U64":%d",node->index, node->nodeInfo.NodeGUID, portp->index);
			}
			found = 0;
            if (portp->nodeno == thisNodeIndex) {
                // build up the path list
                nodeIdx[0] = path[0] = 1;           // one link deep
                path[1] = portp->index;       // out port

			    // check for duplicate paths
			    if (checkForDupPath(top, nodeIdx, path)) continue;

				INC_IN_LOOP_COUNT(portp->portData->inLoopCount);
				otherPortp = sm_get_port(node, portp->portno);
				if (sm_valid_port(otherPortp))
					INC_IN_LOOP_COUNT(otherPortp->portData->inLoopCount);
                nodeIdx[1] = node->index;
                if ((status = buildPathList(top, node, portp, nodeIdx, path)) != VSTATUS_OK) {
                    IB_LOG_ERRORLX("can't allocate space for loop Path from node ", node->nodeInfo.NodeGUID);
					status = VSTATUS_BAD;
					goto err_exit;
                }
				found = 1;
            } else {
                if (esmLoopTestPathLen < 2) continue;
                // look beyond this port's node
				n2p = n1Arr[l].neighbor;
                // look for 2 link loops
				if (!switch_needs_search(&n1_skip_search, n2p))
					continue;
		
				  n2_skip_search.nodes = NULL;
			
				  if ((status = getISLPortList(top, n2p, &n2Arr, &n2_count)) != VSTATUS_OK)
						goto err_exit;

				  for (i = 0; i < n2_count; i++) {
						bkup_loop_path = 0;
						bkup_path_loopweight = 0;
						//try and find atleast esmLoopTestMinISLRedundancy loop paths for this ISL between node and n2p
						if (found && (portp->portData->inLoopCount >= esmLoopTestMinISLRedundancy))
							break;
						n2portp = sm_get_port(n2p, n2Arr[i].portno);
						if (!n2portp)
							continue;
                        if (n2portp->state <= IB_PORT_DOWN || portp->portno == n2portp->index)
                            continue;

						if (smDebugPerf) {
							IB_LOG_INFO_FMT(__func__,
								   "checking for 2 link paths from node %d:"FMT_U64":%d through %d:"FMT_U64":%d",
								   node->index, node->nodeInfo.NodeGUID, portp->index, n2p->index, n2p->nodeInfo.NodeGUID, n2portp->index);
						}
                        if (n2portp->nodeno == thisNodeIndex) {
                            nodeIdx[0] = path[0] = 2;
                            path[1] = portp->index;
                            path[2] = n2portp->index;

						    nodeIdx[1] = node->index;
                            nodeIdx[2] = n2p->index;

						    // check for duplicate paths
						    if (checkForDupPath(top, nodeIdx, path)) continue;

							INC_IN_LOOP_COUNT(portp->portData->inLoopCount);
							otherPortp = sm_get_port(n2p, portp->portno);
							if (sm_valid_port(otherPortp))
								INC_IN_LOOP_COUNT(otherPortp->portData->inLoopCount);

							INC_IN_LOOP_COUNT(n2portp->portData->inLoopCount);
							otherPortp = sm_get_port(node, n2portp->portno);
							if (sm_valid_port(otherPortp))
								INC_IN_LOOP_COUNT(otherPortp->portData->inLoopCount);

                    		if (smDebugPerf) {
								IB_LOG_INFO_FMT(__func__,
									   "found 2 link loop through %d:"FMT_U64":%d and %d:"FMT_U64":%d", 
									   node->index, node->nodeInfo.NodeGUID, portp->index, n2p->index, n2p->nodeInfo.NodeGUID, n2portp->index);
							}
                            if ((status = buildPathList(top, node, portp, nodeIdx, path)) != VSTATUS_OK) {
                                IB_LOG_ERRORLX("can't allocate space for loop Path from node ", node->nodeInfo.NodeGUID);
								status = VSTATUS_BAD;
								goto err_exit;
                            }
							found = 1;
                        } else {
                            if (esmLoopTestPathLen < 3) continue;
                            // look beyond this port's node
							n3p = n2Arr[i].neighbor;
						    if (!switch_needs_search(&n2_skip_search, n3p))
								continue;
                                // look for 3 link loops
							  n3_skip_search.nodes = NULL;

							  if ((status = getISLPortList(top, n3p, &n3Arr, &n3_count)) != VSTATUS_OK)
								goto err_exit;

							  found_from_n3 = 0;
							  for (j = 0; j < n3_count; j++) {
									if (found_from_n3)
										break;
									n3portp = sm_get_port(n3p, n3Arr[j].portno);
									if (!n3portp)
										continue;
							        if ((n3portp->state <= IB_PORT_DOWN) || 
                                        (n2p->index == n3p->index && n2portp->index == n3portp->index) ||
                                        (n2portp->portno == n3portp->index)) continue;

									if (smDebugPerf) {
										IB_LOG_INFO_FMT(__func__,
											   "checking for 3 link paths from node %d:"FMT_U64":%d through %d:"FMT_U64":%d and %d:"FMT_U64":%d",
										   node->index, node->nodeInfo.NodeGUID, portp->index, n2p->index, n2p->nodeInfo.NodeGUID, n2portp->index, 
											   n3p->index, n3p->nodeInfo.NodeGUID, n3portp->index);
									}
                                    if (n3portp->nodeno == thisNodeIndex) {
										if (n3portp->portno == portp->index)
											continue; //possible in case of loop back cable cases

                                        nodeIdx[0] = path[0] = 3;
                                        path[1] = portp->index;
                                        path[2] = n2portp->index;
                                        path[3] = n3portp->index;

                                        nodeIdx[1] = node->index;
                                        nodeIdx[2] = n2p->index;
                                        nodeIdx[3] = n3p->index;

									    // check for duplicate paths
									    if (checkForDupPath(top, nodeIdx, path)) continue;

										// If this ISL is already part of a loop, try and see if
										// we can find a better ISL
										if ((n2portp->portData->inLoopCount >= esmLoopTestMinISLRedundancy)
											|| (n3portp->portData->inLoopCount >= esmLoopTestMinISLRedundancy)) {
											save_loop_path(top, bkup_nodeIdx, bkup_path, nodeIdx, path);
											continue;
										} else {
											bkup_loop_path = 0;
										}

										INC_IN_LOOP_COUNT(portp->portData->inLoopCount);
										otherPortp = sm_get_port(n2p, portp->portno);
				            			if (sm_valid_port(otherPortp))
            								INC_IN_LOOP_COUNT(otherPortp->portData->inLoopCount);

										INC_IN_LOOP_COUNT(n2portp->portData->inLoopCount);
	             						otherPortp = sm_get_port(n3p, n2portp->portno);
				            			if (sm_valid_port(otherPortp))
            								INC_IN_LOOP_COUNT(otherPortp->portData->inLoopCount);

										INC_IN_LOOP_COUNT(n3portp->portData->inLoopCount);
				             			otherPortp = sm_get_port(node, n3portp->portno);
				            			if (sm_valid_port(otherPortp))
            								INC_IN_LOOP_COUNT(otherPortp->portData->inLoopCount);

										if (smDebugPerf) {
											IB_LOG_INFO_FMT(__func__,
												   "found 3 link loop through %d:"FMT_U64":%d and %d:"FMT_U64":%d and %d:"FMT_U64":%d", 
												   node->index, node->nodeInfo.NodeGUID, portp->index,
												   n2p->index, n2p->nodeInfo.NodeGUID, n2portp->index, 
												   n3p->index, n3p->nodeInfo.NodeGUID, n3portp->index);
										}
                                        if ((status = buildPathList(top, node, portp, nodeIdx, path)) != VSTATUS_OK) {
                                            IB_LOG_ERRORLX("can't allocate space for loop Path from node ", node->nodeInfo.NodeGUID);
                                            status = VSTATUS_BAD;
											goto err_exit;
                                        }
										found_from_n3 = 1;
										found = 1;
										break;
                                    } else {
                                        if (esmLoopTestPathLen < 4) continue;
                                        // look beyond this port's node
										n4p = n3Arr[j].neighbor;
									    if (!switch_needs_search(&n3_skip_search, n4p))
											continue;
		
										  if ((status = getISLPortList(top, n4p, &n4Arr, &n4_count)) != VSTATUS_OK)
											goto err_exit;

                                            // look for 4 hop loops
										  for (k = 0; k < n4_count; k++) {
												n4portp = sm_get_port(n4p, n4Arr[k].portno);
												if (!n4portp)
													continue;									
                                                if (n4portp->state <= IB_PORT_DOWN ||
                                                    (n3p->index == n4p->index && n3portp->index == n4portp->index) ||
                                                    n4p->index == n2p->index ||  //this case possible when the loop is between 3 switches
                                                    n3portp->portno == n4portp->index) continue;

										
                                                if (n4portp->nodeno == thisNodeIndex) {
													if (n4portp->portno == portp->index)
														continue;	//possible when the loop is between 3 switches.

                                                    nodeIdx[0] = path[0] = 4;
                                                    path[1] = portp->index;
                                                    path[2] = n2portp->index;
                                                    path[3] = n3portp->index;
                                                    path[4] = n4portp->index;

                                                    nodeIdx[1] = node->index;
                                                    nodeIdx[2] = n2p->index;
                                                    nodeIdx[3] = n3p->index;
                                                    nodeIdx[4] = n4p->index;

												    // check for duplicate paths
												    if (checkForDupPath(top, nodeIdx, path)) continue;

													// This ISL is already part of a loop, try and see if
													// we can find a better ISL
													if ((n2portp->portData->inLoopCount >= esmLoopTestMinISLRedundancy) ||
														(n3portp->portData->inLoopCount >= esmLoopTestMinISLRedundancy) ||
														(n4portp->portData->inLoopCount >= esmLoopTestMinISLRedundancy)) {
														save_loop_path(top, bkup_nodeIdx, bkup_path, nodeIdx, path);
														continue;
													} else {
														bkup_loop_path = 0;
													}

													INC_IN_LOOP_COUNT(portp->portData->inLoopCount);
													otherPortp = sm_get_port(n2p, portp->portno);
				        			    			if (sm_valid_port(otherPortp))
			            								INC_IN_LOOP_COUNT(otherPortp->portData->inLoopCount);

													INC_IN_LOOP_COUNT(n2portp->portData->inLoopCount);
	             									otherPortp = sm_get_port(n3p, n2portp->portno);
				            						if (sm_valid_port(otherPortp))
			            								INC_IN_LOOP_COUNT(otherPortp->portData->inLoopCount);

													INC_IN_LOOP_COUNT(n3portp->portData->inLoopCount);
				        			     			otherPortp = sm_get_port(n4p, n3portp->portno);
				            						if (sm_valid_port(otherPortp))
			            								INC_IN_LOOP_COUNT(otherPortp->portData->inLoopCount);
			
						                        	INC_IN_LOOP_COUNT(n4portp->portData->inLoopCount);
            	             						otherPortp = sm_get_port(node, n4portp->portno);
             				            			if (sm_valid_port(otherPortp))
                        								INC_IN_LOOP_COUNT(otherPortp->portData->inLoopCount);


													if (smDebugPerf) {
														IB_LOG_INFO_FMT(__func__,
															   "found 4 link loop through %d:"FMT_U64":%d and %d:"FMT_U64":%d and %d:"FMT_U64":%d and %d:"FMT_U64":%d", 
															   node->index, node->nodeInfo.NodeGUID, portp->index, n2p->index, n2p->nodeInfo.NodeGUID, n2portp->index, 
															   n3p->index, n3p->nodeInfo.NodeGUID, n3portp->index, n4p->index, n4p->nodeInfo.NodeGUID, n4portp->index);
													}
                                                    if ((status = buildPathList(top, node, portp, nodeIdx, path)) != VSTATUS_OK) {
                                                        IB_LOG_ERRORLX("can't allocate space for loop Path from node ", node->nodeInfo.NodeGUID);
                                                        status = VSTATUS_BAD;
														goto err_exit;
                                                    }
													found_from_n3 = 1;
													found = 1;
													break;
                                                } else {
                                                    continue;
                                                }
										 } // end for all physical ports of switch node 4
										 if (!found_from_n3) {
										 	add_to_skip_search(&n3_skip_search, n3p, n4p);
									     }
						 			     if (n4Arr) {
											vs_pool_free(&sm_pool, (void *)n4Arr);
											n4Arr = NULL;
										 }
				                    }
							 } // end for all physical ports of switch node 3
			 			     if (n3Arr) {
								vs_pool_free(&sm_pool, (void *)n3Arr);
								n3Arr = NULL;
							 }
							 if (n3_skip_search.nodes) {
								vs_pool_free(&sm_pool, (void *)n3_skip_search.nodes);
								n3_skip_search.nodes = NULL;
							 }
							 if (!found) {
								add_to_skip_search(&n2_skip_search, n2p, n3p);
							 }
                        }
                  }  // end for all physical ports of switch node 2
				  if (n2Arr) {
					vs_pool_free(&sm_pool, (void *)n2Arr);
					n2Arr = NULL;
				  }
			      if (n2_skip_search.nodes) {
					vs_pool_free(&sm_pool, (void *)n2_skip_search.nodes);
					n2_skip_search.nodes = NULL;
				  }
				  if (!found && !bkup_loop_path) {
					add_to_skip_search(&n1_skip_search, node, n2p);
				  }
				
				  if (!found && bkup_loop_path) {
					restore_backup_loop_path(top, nodeIdx, path, bkup_nodeIdx, bkup_path);
                    if ((status = buildPathList(top, node, portp, nodeIdx, path)) != VSTATUS_OK) {
                        IB_LOG_ERRORLX("can't allocate space for loop Path from node ", node->nodeInfo.NodeGUID);
						status = VSTATUS_BAD;
						goto err_exit;
                    }
					found = 1;
				  }
            } 
        } // end for all physical ports of switch node 1
		if (n1Arr) {
			vs_pool_free(&sm_pool, (void *)n1Arr);
			n1Arr = NULL;
		}
		if (n1_skip_search.nodes) {
			vs_pool_free(&sm_pool, (void *)n1_skip_search.nodes);
			n1_skip_search.nodes = NULL;
		}
    } // end for all switches loop

	print_LoopCoverage(top);

    IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);

err_exit:
	free_skip_search(&n1_skip_search);
	free_skip_search(&n2_skip_search);
	free_skip_search(&n3_skip_search);

	free_portList(&n1Arr);
	free_portList(&n2Arr);
	free_portList(&n3Arr);
	free_portList(&n4Arr);


	return status;
}

//
//	find every path up to 4 links long that start and end at a switch node.
//  Each found path will get a lid assigned to it.  The path to these lids 
//  is a concatenation of the path to the node when first discovered and 
//  the up to 4 link path found
//
Status_t
loopTest_userexit_findPaths(Topology_t *top) {
	uint8_t		path[64];
    uint16_t    nodeIdx[64];
    Node_t      *node;
	Port_t		*portp, *n2portp, *n3portp, *n4portp;
	Node_t		*n2p, *n3p, *n4p;
	Status_t	status=VSTATUS_OK;
    int         thisNodeIndex=0;

	IB_ENTER(__func__, 0, 0, 0, 0);

    if (!esmLoopTestOn) return status;

	if (esmLoopTestFast) {
		 return findLoopPaths_quick(top);
	}

    loopPathLidEnd = 0;             // reset end lid number
    loopPathLidStart = ((sm_topop->maxLid + 64)/64)*64;
    //
    // loop test - find loop paths through the switches
    //
	for_all_switch_nodes(top, node) {
        thisNodeIndex = node->index;
		if (smDebugPerf) {
			IB_LOG_INFO_FMT(__func__,
				   "checking for paths starting from node %d:"FMT_U64,node->index, node->nodeInfo.NodeGUID);
		}
        for_all_physical_ports(node, portp) {
            if (portp->state <= IB_PORT_DOWN)
                continue;
			if (smDebugPerf) {
				IB_LOG_INFO_FMT(__func__,
					   "checking for paths from node %d:"FMT_U64":%d",node->index, node->nodeInfo.NodeGUID, portp->index);
			}
            if (portp->nodeno == thisNodeIndex) {
                // build up the path list
                nodeIdx[0] = path[0] = 1;           // one link deep
                path[1] = portp->index;       // out port
                nodeIdx[1] = node->index;
                if ((status = buildPathList(top, node, portp, nodeIdx, path)) != VSTATUS_OK) {
                    IB_LOG_ERRORLX("can't allocate space for loop Path from node ", node->nodeInfo.NodeGUID);
                    return(VSTATUS_BAD);
                }
            } else {
                if (esmLoopTestPathLen < 2) continue;
                // look beyond this port's node
                if ((n2p = sm_find_node(top, portp->nodeno)) == NULL) {
                    IB_LOG_ERROR("can't find node structure for node index ", portp->nodeno);
                } else if (n2p->nodeInfo.NodeType != NI_TYPE_SWITCH) {
					continue;  /* only interested in switches */
                } else {
                    // look for 2 link loops
                    for_all_physical_ports(n2p, n2portp) {
                        if (n2portp->state <= IB_PORT_DOWN || portp->portno == n2portp->index)
                            continue;
						if (smDebugPerf) {
							IB_LOG_INFO_FMT(__func__,
								   "checking for 2 link paths from node %d:"FMT_U64":%d through %d:"FMT_U64":%d",
								   node->index, node->nodeInfo.NodeGUID, portp->index, n2p->index, n2p->nodeInfo.NodeGUID, n2portp->index);
						}
                        if (n2portp->nodeno == thisNodeIndex) {
                            nodeIdx[0] = path[0] = 2;
                            path[1] = portp->index;
                            path[2] = n2portp->index;
                            nodeIdx[1] = node->index;
                            nodeIdx[2] = n2p->index;
							if (smDebugPerf) {
								IB_LOG_INFO_FMT(__func__,
									   "found 2 link loop through %d:"FMT_U64":%d and %d:"FMT_U64":%d", 
									   node->index, node->nodeInfo.NodeGUID, portp->index, n2p->index, n2p->nodeInfo.NodeGUID, n2portp->index);
							}
                            if ((status = buildPathList(top, node, portp, nodeIdx, path)) != VSTATUS_OK) {
                                IB_LOG_ERRORLX("can't allocate space for loop Path from node ", node->nodeInfo.NodeGUID);
                                return(VSTATUS_BAD);
                            }
                        } else {
                            if (esmLoopTestPathLen < 3) continue;
                            // look beyond this port's node
                            if ((n3p = sm_find_node(top, n2portp->nodeno)) == NULL) {
                                IB_LOG_ERROR("can't find node structure for node index ", n2portp->nodeno);
							} else if (n3p->nodeInfo.NodeType != NI_TYPE_SWITCH) {
								continue;  /* only interested in switches */
							} else {
                                // look for 3 link loops
                                for_all_physical_ports(n3p, n3portp) {
                                    if ((n3portp->state <= IB_PORT_DOWN) || 
                                        (n2p->index == n3p->index && n2portp->index == n3portp->index) ||
                                        (n2portp->portno == n3portp->index)) continue;
									if (smDebugPerf) {
										IB_LOG_INFO_FMT(__func__,
											   "checking for 3 link paths from node %d:"FMT_U64":%d through %d:"FMT_U64":%d and %d:"FMT_U64":%d",
											   node->index, node->nodeInfo.NodeGUID, portp->index, n2p->index, n2p->nodeInfo.NodeGUID, n2portp->index, 
											   n3p->index, n3p->nodeInfo.NodeGUID, n3portp->index);
									}
                                    if (n3portp->nodeno == thisNodeIndex) {
                                        nodeIdx[0] = path[0] = 3;
                                        path[1] = portp->index;
                                        path[2] = n2portp->index;
                                        path[3] = n3portp->index;
                                        nodeIdx[1] = node->index;
                                        nodeIdx[2] = n2p->index;
                                        nodeIdx[3] = n3p->index;
										if (smDebugPerf) {
											IB_LOG_INFO_FMT(__func__,
												   "found 3 link loop through %d:"FMT_U64":%d and %d:"FMT_U64":%d and %d:"FMT_U64":%d", 
												   node->index, node->nodeInfo.NodeGUID, portp->index,
												   n2p->index, n2p->nodeInfo.NodeGUID, n2portp->index, 
												   n3p->index, n3p->nodeInfo.NodeGUID, n3portp->index);
										}
                                        if ((status = buildPathList(top, node, portp, nodeIdx, path)) != VSTATUS_OK) {
                                            IB_LOG_ERRORLX("can't allocate space for loop Path from node ", node->nodeInfo.NodeGUID);
                                            return(VSTATUS_BAD);
                                        }
                                    } else {
                                        if (esmLoopTestPathLen < 4) continue;
                                        // look beyond this port's node
                                        if ((n4p = sm_find_node(top, n3portp->nodeno)) == NULL) {
                                            IB_LOG_ERROR("can't find node structure for node index ", n3portp->nodeno);
										} else if (n4p->nodeInfo.NodeType != NI_TYPE_SWITCH) {
											continue;  /* only interested in switches */
										} else {
                                            // look for 4 hop loops
                                            for_all_physical_ports(n4p, n4portp) {
                                                if (n4portp->state <= IB_PORT_DOWN ||
                                                    (n3p->index == n4p->index && n3portp->index == n4portp->index) ||
                                                    n3portp->portno == n4portp->index) continue;
                                                if (n4portp->nodeno == thisNodeIndex) {
                                                    nodeIdx[0] = path[0] = 4;
                                                    path[1] = portp->index;
                                                    path[2] = n2portp->index;
                                                    path[3] = n3portp->index;
                                                    path[4] = n4portp->index;
                                                    nodeIdx[1] = node->index;
                                                    nodeIdx[2] = n2p->index;
                                                    nodeIdx[3] = n3p->index;
                                                    nodeIdx[4] = n4p->index;
													if (smDebugPerf) {
														IB_LOG_INFO_FMT(__func__,
															   "found 4 link loop through %d:"FMT_U64":%d and %d:"FMT_U64":%d and %d:"FMT_U64":%d and %d:"FMT_U64":%d", 
															   node->index, node->nodeInfo.NodeGUID, portp->index, n2p->index, n2p->nodeInfo.NodeGUID, n2portp->index, 
															   n3p->index, n3p->nodeInfo.NodeGUID, n3portp->index, n4p->index, n4p->nodeInfo.NodeGUID, n4portp->index);
													}
                                                    if ((status = buildPathList(top, node, portp, nodeIdx, path)) != VSTATUS_OK) {
                                                        IB_LOG_ERRORLX("can't allocate space for loop Path from node ", node->nodeInfo.NodeGUID);
                                                        return(VSTATUS_BAD);
                                                    }
                                                } else {
                                                    continue;
                                                }
                                            } // // end for all physical ports of switch node 4
                                        }
                                    }
                                } // end for all physical ports of switch node 3
                            }
                        }
                    } // end for all physical ports of switch node 2 
                }
            } 
        } // end for all physical ports of switch node 1
    } // end for all switches loop
	
    IB_EXIT(__func__, VSTATUS_OK);
	return(VSTATUS_OK);
}

//
// print loop paths
// user is responsible for freeing it using vs_pool_free()
// if buffering is specified
//
char*  printLoopPaths(int nodeIdx, int buffer) {
    int         i,j,cnt=0;
    LoopPath_t  *loopPath;
    Node_t      *nodep = NULL;
    Port_t      *portp = NULL;
	Port_t		*outPortp=NULL;
	char        *buf = NULL;
	int 		len = 1000;

	if (buffer) {
		if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
			IB_FATAL_ERROR_NODUMP("printLoopPaths: CAN'T ALLOCATE SPACE.");
			return NULL;
		}
		buf[0] = '\0';
	}

    if (!esmLoopTestOn) {
        buf = snprintfcat(buf, &len, "Loop Test is not currently running!\n");
    } else {
		(void)vs_rdlock(&old_topology_lock);
		buf = snprintfcat(buf, &len, "\n");
		for_all_switch_nodes(&old_topology, nodep) {
			buf = snprintfcat(buf, &len, "Node Idx: %d, Guid: "FMT_U64" Desc %s\n", 
				(int)nodep->index, nodep->nodeInfo.NodeGUID, nodep->nodeDesc.NodeString);
		}
    	buf = snprintfcat(buf, &len, "\n");
    	buf = snprintfcat(buf, &len, "--------------------------------------------------------------------------\n");
    	buf = snprintfcat(buf, &len, " Node  Node                       Node    Path\n");
    	buf = snprintfcat(buf, &len, " Idx   Lid        NODE GUID      #Ports    LID    PATH[n:p->n:p]\n");
    	buf = snprintfcat(buf, &len, " ---- ------  ------------------  ------  ------  ---------------\n");
    	if (nodeIdx >= 0) {
        	if ( ((nodep = sm_find_node(&old_topology, nodeIdx)) == NULL) || nodep->nodeInfo.NodeType != NI_TYPE_SWITCH) {
            	buf = snprintfcat(buf, &len, "Node index %d is not a valid switch node\n", nodeIdx);
            	(void)vs_rwunlock(&old_topology_lock);
				return buf;
        	}
        	for (loopPath=old_topology.loopPaths; loopPath != NULL; loopPath = loopPath->next) {
            	for (j=1; j<=loopPath->nodeIdx[0]; j++) {
                	if (nodeIdx == loopPath->nodeIdx[j]) {
                    	cnt++;
                    	// print out loop path
                    	portp = sm_get_port(nodep,0);
                    	buf = snprintfcat(buf, &len, " %4d 0x%04x  "FMT_U64"    %2d    0x%04x  ",
                           (int)nodep->index, (sm_valid_port(portp)) ? (int)portp->portData->lid : -1,
						   nodep->nodeInfo.NodeGUID, (int)nodep->nodeInfo.NumPorts, (int)loopPath->lid);
                    	for (i = 1; i <= (int)loopPath->path[0]; i++) {
							outPortp = sm_find_port(&old_topology, loopPath->nodeIdx[i], loopPath->path[i]);
                        	buf = snprintfcat(buf, &len, "%d:%d->%d:%d ", loopPath->nodeIdx[i], loopPath->path[i],
							   (outPortp)?(int)outPortp->nodeno:-1, (outPortp)?(int)outPortp->portno:0xffff);
                    	}
                    	buf = snprintfcat(buf, &len, "\n");
                    	break;
                	}
            	}
        	}
    	} else {
        	// print every loop path
        	for (loopPath=old_topology.loopPaths; loopPath != NULL; loopPath = loopPath->next) {
            	if ((nodep = sm_find_node(&old_topology, loopPath->startNodeno)) != NULL) {
                	cnt++;
                	// print out loop path
                	portp = sm_get_port(nodep,0);
                	buf = snprintfcat(buf, &len, " %4d 0x%04x  "FMT_U64"    %2d    0x%04x  ",
                       (int)nodep->index, (sm_valid_port(portp)) ? (int)portp->portData->lid : -1,
					   nodep->nodeInfo.NodeGUID, (int)nodep->nodeInfo.NumPorts, (int)loopPath->lid);
                	for (i = 1; i <= (int)loopPath->path[0]; i++) {
						outPortp = sm_find_port(&old_topology, loopPath->nodeIdx[i], loopPath->path[i]);
						buf = snprintfcat(buf, &len, "%d:%d->%d:%d ", loopPath->nodeIdx[i], loopPath->path[i],
						   (outPortp)?(int)outPortp->nodeno:0, (outPortp)?(int)outPortp->portno:0);
                	}
                	buf = snprintfcat(buf, &len, "\n");
            	}
        	}
    	}
    	// print out how many paths found
    	buf = snprintfcat(buf, &len, "--------------------------------------------------------------------------\n");
    	buf = snprintfcat(buf, &len, "There are %d total loop paths of <=%d links in length in the fabric!\n", 
			old_topology.numLoopPaths, esmLoopTestPathLen);
		if (esmLoopTestFast) {
	    	buf = snprintfcat(buf, &len, "Two LIDs are used per loop path to inject packets in clockwise and anti-clockwise directions\n");
		}
    	if (nodeIdx >= 0) {
        	portp = sm_get_port(nodep,0);
        	buf = snprintfcat(buf, &len, "Node index=%d, GUID="FMT_U64", LID=0x%x has %d loops\n", nodeIdx, nodep->nodeInfo.NodeGUID,
				(sm_valid_port(portp)) ? (int)portp->portData->lid : -1, cnt);
    	}
    	buf = snprintfcat(buf, &len, "--------------------------------------------------------------------------\n");
	
    	(void)vs_rwunlock(&old_topology_lock);
	}

	return buf;
}

//
// set loop path max length
//
void setLoopPathLength (int len) {
    esmLoopTestPathLen = (len < 5 && len > 1) ? len : 3;
    IB_LOG_INFINI_INFO("Loop path length = ", esmLoopTestPathLen);
    if (esmLoopTestOn) sm_forceSweep("SM Loop Test Path Length Changed");
}

//
//  Set esmLoopTestMinISLRedundancy which determines min number of loops in which each ISL should be included.
//
void setLoopMinISLRedundancy (int num) {
	int new = 0;
     new = (num > 0) ? num : 4;
	if (new != esmLoopTestMinISLRedundancy) {
		esmLoopTestMinISLRedundancy = new;
	    if (esmLoopTestOn) sm_forceSweep("SM Loop Test Minimum ISL Redundancy Changed");
	}
    IB_LOG_INFINI_INFO("Loop MinISLRedundancy = ", esmLoopTestMinISLRedundancy);
}

//
// print switch Linear Forwarding Table
//
char* printSwitchLft(int nodeIdx, int useNew, int haveLock, int buffer) {
    int     i;
    Node_t  *nodep;
    Port_t *portp;
    Topology_t * topop;
    Lock_t * lockp;
	char *buf = NULL;
	int len = 1000;

	if (buffer) {
		if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
			IB_FATAL_ERROR_NODUMP("printSwitchLft: CAN'T ALLOCATE SPACE.");
			return NULL;
		}
		buf[0] = '\0';
	}

    if (useNew) {
        topop  = &sm_newTopology;
        lockp = &new_topology_lock;
    } else {
        topop  = &old_topology;
        lockp = &old_topology_lock;
    }

    if (topology_passcount >= 1 || useNew) {
        if (!haveLock) {
            if (lockp == &old_topology_lock) (void)vs_rdlock(lockp);
            else (void)vs_lock(lockp);
        }
        if (nodeIdx < 0) {
            // print all switches LFTs
            for_all_switch_nodes(topop, nodep) {
                if (nodep != NULL) {
                    portp = sm_get_port(nodep,0);
                    buf = snprintfcat(buf, &len, "Node[%.4d]  LID=0x%.4X  GUID="FMT_U64" [%s] Linear Forwarding Table\n", 
                           (int)nodep->index, (sm_valid_port(portp)) ? (int)portp->portData->lid : -1, nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
                    buf = snprintfcat(buf, &len, "   LID    PORT\n");
                    buf = snprintfcat(buf, &len, " ------   ----\n");
                    for (i = 1; i <= topop->maxLid; i++) {
                        if (nodep->lft[i] != 0xff) 
                            buf = snprintfcat(buf, &len, " 0x%04x   %04d\n", i, nodep->lft[i]);
                    }
                    buf = snprintfcat(buf, &len, " --------------\n");
                    buf = snprintfcat(buf, &len, "\n");
                }
            }
        } else {
            if ((nodep = sm_find_node(topop, nodeIdx)) != NULL && nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
                portp = sm_get_port(nodep,0);
                buf = snprintfcat(buf, &len, "Node[%.4d]    LID=0x%.4X    GUID="FMT_U64" [%s] Linear Forwarding Table\n", 
                       (int)nodep->index, (sm_valid_port(portp)) ? (int)portp->portData->lid : -1, nodep->nodeInfo.NodeGUID, sm_nodeDescString(nodep));
                buf = snprintfcat(buf, &len, "   LID    PORT\n");
                buf = snprintfcat(buf, &len, " ------   ----\n");
                for (i = 1; i <= topop->maxLid; i++) {
                    if (nodep->lft[i] != 0xff) 
                        buf = snprintfcat(buf, &len, " 0x%04x   %04d\n", i, nodep->lft[i]);
                }
            } else {
                buf = snprintfcat(buf, &len, "Node index %d is not a valid switch!\n", nodeIdx);
            }
        }
        if (!haveLock) {
            if (lockp == &old_topology_lock) (void)vs_rwunlock(lockp);
            else (void)vs_unlock(lockp);
        }
    } else {
        buf = snprintfcat(buf, &len, "Sm is not MASTER or is not currently running!\n");
    }
	return buf;
}

// print running Loop Test Configuration to buffer
// user is responsible for freeing it using vs_pool_free()
// if buffering is specified
//
char* printLoopTestConfig(int buffer) {
	char * buf = NULL;
	int len = 500;

	if (buffer) {
		if (vs_pool_alloc(&sm_pool, len, (void*)&buf) != VSTATUS_OK) {
			IB_FATAL_ERROR_NODUMP("printLoopTestConfig: CAN'T ALLOCATE SPACE.");
			return NULL;
		}
		buf[0] = '\0';
	}

    if (esmLoopTestOn) {
		buf = snprintfcat(buf, &len, "Loop Test is running with following parameters:\n");
		buf = snprintfcat(buf, &len, " Max Path Length   #Packets   Inject Point\n");
		buf = snprintfcat(buf, &len, " ---------------   --------   ------------\n");
        if (esmLoopTestInjectNode < 0) {
			buf = snprintfcat(buf, &len, "        %d           %05d        All Nodes\n", esmLoopTestPathLen, esmLoopTestNumPkts);
        } else {
			buf = snprintfcat(buf, &len, "        %d           %05d        Node %d\n", esmLoopTestPathLen, esmLoopTestNumPkts, esmLoopTestInjectNode);
        }
		buf = snprintfcat(buf, &len, "\n");
		buf = snprintfcat(buf, &len, "FastMode=%d, FastMode MinISLRedundancy=%d, InjectEachSweep=%d, TotalPktsInjected since start=%d\n",
			esmLoopTestFast, esmLoopTestMinISLRedundancy, esmLoopTestInjectEachSweep, (int)esmLoopTestTotalPktsInjected);
    } else {
		buf = snprintfcat(buf, &len, "Loop Test is not currently running; enter 'smLooptestStart' or 'smLooptestFastModeStart' to start the Loop Test!\n");
    }
	return buf;
}
