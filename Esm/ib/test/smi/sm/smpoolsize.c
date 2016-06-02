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
//									     //
// FILE NAME								     //
//    smpoolsize.c							     //
//									     //
// DESCRIPTION								     //
//    This program calculates the sm pool size required to support fabric.	 //
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
//    ib_const.h							     //
//									     //
//									     //
//===========================================================================//

#include "os_g.h"
#include "ib_status.h"
#include "ib_types.h"
#include "ib_mad.h"
#include "ib_macros.h"
#include "sm_l.h"
#include "sa_l.h"
#include "ib_sa.h"
#include "mai_g.h"
#include "cs_g.h"
#include "cs_log.h"

extern	int	optind;
extern	int	opterr;
extern	int	optopt;
extern	char	*optarg;

uint32_t	numFIs=0;
uint32_t	numSwChips=0;
uint32_t	numPortsPerSwChip=24;
uint32_t	numSwChips2=0;
uint32_t	numPortsPerSwChip2=0;
uint32_t	numActivPortsPerSwChip2=0;
uint32_t    Lmc=0;
Status_t	status;

#define SA_MAX_DATA_LEN (512*1024)


void
usage(void) {

	fprintf(stderr, "smpoolsize -n numFIs -s numSwType1 -p numPortsSwType1 -S numSwType2 -P numPortsSwType2 -A numActivePortsSwType2 -l lmc\n");
	fprintf(stderr, "           -n, -s, are mandatory.  -p defaults to 24, -S, -P, -A, -l default to 0 if not entered\n");
	exit(1);
}
//
// available on both embeded and HSM
//
int saSubscriberSize(void) {
	int size = sizeof(InformRecord_t);
	sysPrintf("An SA Subscriber is %d bytes\n", size);
	return size;
}

int saServiceRecordSize(void) {
	int size = sizeof(OpaServiceRecord_t);
	sysPrintf("An SA Service Record is %d bytes\n", size);
	return size;
}

int saMcGroupSize(void) {
    int size = sizeof(McGroup_t) + 16;
    sysPrintf("An SA Multicast Group is %d bytes\n", size);
    return size;
}

int saMcMemberSize(void) {
    int size = sizeof(McMember_t) + 16;
    sysPrintf("An SA Multicast Member is %d bytes\n", size);
    return size;
}

int saNodeRecordSize(void) {
    int size = sizeof(STL_NODE_RECORD) + Calculate_Padding(sizeof(STL_NODE_RECORD));
    sysPrintf("An SA Node Record is %d bytes\n", size);
    return size;
}

int saPathRecordSize(void) {
    int size = sizeof(IB_PATH_RECORD) + Calculate_Padding(sizeof(IB_PATH_RECORD));
    sysPrintf("An SA Path Record is %d bytes\n", size);
    return size;
}

int saMaxResponseSize(void) {
	return SA_MAX_DATA_LEN;
}

//
// the following are available on both embebded and HSM
//
int smCASize(void) {
	int size = sizeof(Node_t) + (3 * sizeof(Port_t)) + sizeof(PortData_t);
	sysPrintf("The size of one FI in the topology is %d bytes\n", size);
	return size;
}

int smSwitchSize(int numSwitches, int numPorts, int numActivePorts, int numSwitches2, int numFIs, int lmc, int mlidTableCap) {
	int size;
    int    sizelft;

    sizelft = (numSwitches + numSwitches2 + numFIs) * (1 << lmc) + 64;
	//sysPrintf("The size of a Switch LFT table is %d bytes\n", sizelft);
    size = sizeof(Node_t);
    size += ((numPorts + 1) * (sizeof(Port_t))) + ((numActivePorts+1) * sizeof(PortData_t));
    size += 16 + sizelft + sizeof(uint16_t * ) * mlidTableCap + sizeof(uint16_t) * mlidTableCap * 16;
	sysPrintf("The average size of this type Switch in the topology is %d bytes\n", size);
	return size;
}

int smPortSize(void) {
	int size = sizeof(Port_t) + sizeof(PortData_t);
	sysPrintf("The size of one port in the topology is %d bytes\n", size);
	return size;
}

int smLidmapSize(void) {
	int size = sizeof(LidMap_t) * SIZE_LFT;
	sysPrintf("The size LidMap in the topology is %d bytes\n", size);
	return size;
}

uint64_t smFabricSize(void) {
    int numTopologies = 2;
    int numMcGroups = 4;
    int numCaSubscriptions = 4;
    int numManagers = 4; // PM, BM, FE, PSM
    int numSmSubs = 7; // PM, BM, FE all do GID IN/OUT, FE does PortState
    int numStandbys = 2;
    int numHostServices = 2;

    int      sizelft = (numSwChips + numSwChips2 + numFIs) * (1 << Lmc) + 64;
    uint64_t caSize = smCASize() * numTopologies;
    uint64_t switchSize = smSwitchSize(numSwChips, numPortsPerSwChip, numPortsPerSwChip, numSwChips2, numFIs, Lmc, DEFAULT_SW_MLID_TABLE_CAP) * numTopologies;
    uint64_t switchSize2 = smSwitchSize(numSwChips2, numPortsPerSwChip2, numActivPortsPerSwChip2, numSwChips, numFIs, Lmc, DEFAULT_SW_MLID_TABLE_CAP) * numTopologies;
    uint64_t subscriberSize = saSubscriberSize();
    uint64_t serviceRecordSize = saServiceRecordSize();
    uint64_t groupSize = saMcGroupSize();
    uint64_t memberSize = saMcMemberSize();
    uint64_t pathArraySize = sizeof(uint8_t) * numTopologies;
    uint64_t costArraySize = sizeof(uint8_t) * numTopologies;
    uint64_t maxResponseSize = saMaxResponseSize();
    uint64_t nodeRecordSize = saNodeRecordSize();
    uint64_t lidMapSize = smLidmapSize();
    uint64_t smSize = numSmSubs * subscriberSize + numManagers * serviceRecordSize;
    uint64_t standbySize = smSize * numStandbys;
    uint64_t guidInfoSize = sizeof(GuidInfo_t) * (numFIs+numSwChips+numSwChips2) * numTopologies;
    uint64_t size = 0;
    caSize += numHostServices * serviceRecordSize + numCaSubscriptions * subscriberSize + numMcGroups * memberSize;
    caSize *= numFIs;
 
    switchSize *= numSwChips;
    switchSize2 *= numSwChips2;
    /* now for total switch size */
    switchSize += switchSize2;

    groupSize *= numMcGroups;
    
    costArraySize *= (numSwChips+numSwChips2) * (numSwChips+numSwChips2);
    pathArraySize *= (numSwChips+numSwChips2) * (numSwChips+numSwChips2);

    nodeRecordSize *= numFIs * (numFIs / 4);

    size = caSize + switchSize + smSize + standbySize + groupSize +
           costArraySize + pathArraySize + maxResponseSize +
           nodeRecordSize + lidMapSize + guidInfoSize;

    // add in 10% fudge factor
    size += size/10;

	sysPrintf("The size of a Switch LFT table is %d bytes\n", sizelft);
    sysPrintf("The total for FIs in the fabric is %"CS64"d bytes\n", caSize);
    sysPrintf("The total for switches in the fabric is %"CS64"d bytes\n", switchSize);
    sysPrintf("The total for the master SM is %"CS64"d bytes\n", smSize);
    sysPrintf("The total for %d standby SMs is %"CS64"d bytes\n", numStandbys, standbySize);
    sysPrintf("The total for %d multicast groups is %"CS64"d bytes\n", numMcGroups, groupSize);
    sysPrintf("The total for the cost array is %"CS64"d bytes\n", costArraySize);
    sysPrintf("The total for the path array is %"CS64"d bytes\n", pathArraySize);
    sysPrintf("The total for the LidMap is %"CS64"d bytes\n", lidMapSize);
    sysPrintf("The total for SA max response is %"CS64"d bytes\n", maxResponseSize);
    sysPrintf("The total for sending all node records to 1/4 the nodes at once is %"CS64"d bytes\n", nodeRecordSize);

	sysPrintf("The topology size of a fabric with %d FIs and %d %d port switches and %d %d port (%d active) switches is %"CS64"d bytes\n",
			  numFIs, numSwChips, numPortsPerSwChip, numSwChips2, numPortsPerSwChip2, numActivPortsPerSwChip2, size);
	return size;	
}

//
//  main utility function
//
int main(int argc, char *argv[]) {
	int		    c;

//
//	Get the parameters
//
	while ((c = getopt(argc, argv, "n:s:p:S:P:A:l:")) != -1) {
		switch (c) {
		case 'n':
			sscanf(optarg, "%u", &numFIs);
			break;
		case 's':
			sscanf(optarg, "%u", &numSwChips);
			break;
		case 'p':
			sscanf(optarg, "%u", &numPortsPerSwChip);
			break;
		case 'S':
			sscanf(optarg, "%u", &numSwChips2);
			break;
		case 'P':
			sscanf(optarg, "%u", &numPortsPerSwChip2);
			break;
		case 'A':
			sscanf(optarg, "%u", &numActivPortsPerSwChip2);
			break;
		case 'l':
			sscanf(optarg, "%u", &Lmc);
			break;
		default:
			usage();
			break;
		}
	}

	if (numFIs == 0 || numSwChips == 0) {
		usage();
	}

    // calulate and print the fabric size
    smFabricSize(/*numFIs, numSwChips, numPortsPerSwChip, numSwChips2, numPortsPerSwChip2, numActivPortsPerSwChip2, Lmc, DEFAULT_SW_MLID_TABLE_CAP*/);

	exit(0);
}
