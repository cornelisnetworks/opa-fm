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
// DESCRIPTION								     //
//   Functions to implement Set/Get and programming of buffer control Tables //
//									     //
//===========================================================================//


#include <stdlib.h>
#include <string.h>

#include "ib_status.h"
#include "os_g.h"
#include "iba/stl_sm_priv.h"
#include "sm_l.h"

static Status_t
sm_get_mkey(Node_t *nodep, uint64_t *mkey, uint8_t portNum)
{
	Port_t *portp;
	uint8_t p;
	if (nodep->nodeInfo.NodeType == NI_TYPE_SWITCH) {
		p = 0;
	} else {
		p = portNum;
	}
	if ((portp = sm_get_port(nodep, p)) == NULL) {
		cs_log(VS_LOG_ERROR, "sm_get_mkey",
			"Failed to find mkey for for node %s ; port %u",
			sm_nodeDescString(nodep), p);
		return (VSTATUS_NODEV);
	}
	*mkey = (portp->portData->portInfo.M_Key==0) ? sm_config.mkey : portp->portData->portInfo.M_Key;
	return (VSTATUS_OK);
}

Status_t
sm_get_buffer_control_tables(IBhandle_t fd_topology, Node_t *nodep,
						uint8_t start_port, uint8_t end_port)
{
	int port;
	Status_t status = FERROR;
	uint8_t num_ports;
	STL_BUFFER_CONTROL_TABLE *pbct;
    uint8 maxcount = STL_NUM_BFRCTLTAB_BLOCKS_PER_DRSMP;
	Port_t *portp;

	if (!nodep || !start_port || start_port > end_port) {
		cs_log(VS_LOG_ERROR, "sm_get_buffer_control_table",
				"Invalid Parameters for node %s ; %p, %u, %u",
				nodep?sm_nodeDescString(nodep):"<NULL>", nodep, start_port, end_port);
		return (VSTATUS_ILLPARM);
	}


	num_ports = (end_port - start_port) +1;
	pbct = malloc(num_ports * sizeof(STL_BUFFER_CONTROL_TABLE));
	if (!pbct) {
		cs_log(VS_LOG_ERROR, "sm_get_buffer_control_table",
				"Failed to allocate buffer control table; node %s; no memory",
				sm_nodeDescString(nodep));
		return (VSTATUS_NOMEM);
	}

	port = nodep->nodeInfo.NodeType == NI_TYPE_CA ? start_port : 0;
	portp = sm_get_port(nodep, port);
	if (!sm_valid_port(portp)) {
		cs_log(VS_LOG_VERBOSE, "sm_get_buffer_control_table",
			   "Failed to get Port %d of node %s",
			   port, sm_nodeDescString(nodep));
		free(pbct);
		return VSTATUS_BAD;
	}
	SmpAddr_t addr = SMP_ADDR_CREATE_DR(PathToPort(nodep, portp));

	/*
	 * FIXME Optimization:
	 * this could be updated to query only ports which are LinkUp.
	 * But that makes the block algorithm much more difficult and for most
	 * large fabrics you will only be missing a small number of links which
	 * means it is likely just as efficient to query all ports in large ranges.
	 */
	for (port = start_port; port <= end_port; port += maxcount) {
		uint8_t n = MIN(maxcount, (end_port - port)+1);
		uint32_t am = (n << 24) | port;
		STL_BUFFER_CONTROL_TABLE *buf = &pbct[port-start_port];

		status = SM_Get_BufferControlTable(fd_topology, am, &addr, buf);
		if (status != VSTATUS_OK) {
			cs_log(VS_LOG_ERROR, "sm_get_buffer_control_table",
					"buffer control table query failed; node %s; %s",
					sm_nodeDescString(nodep),
					cs_convert_status (status));
			goto error;
		}
	}

	for (port = start_port; port <= end_port; port++) {
		portp = sm_get_port(nodep, port);
		if (!sm_valid_port(portp)) {
			cs_log(VS_LOG_VERBOSE, "sm_get_buffer_control_tables",
				   "Failed to get port %d of node %s",
				   port, sm_nodeDescString(nodep));
			continue;
		}
		memcpy(&portp->portData->bufCtrlTable, &(pbct[port-start_port]),
			sizeof(portp->portData->bufCtrlTable));
	}

error:
	free(pbct);
	return (status);
}


Status_t
sm_set_buffer_control_tables(IBhandle_t fd_topology, Node_t *nodep,
						uint8_t start_port, uint8_t end_port,
						STL_BUFFER_CONTROL_TABLE bcts[],
						uint8_t uniform_bcts)
{
	Status_t status = VSTATUS_OK;
    uint8 maxcount = STL_NUM_BFRCTLTAB_BLOCKS_PER_LID_SMP;
	uint16_t port;
	uint8_t num_ports;
	uint64_t mkey;
	uint32_t madStatus;
	extern char * sm_getMadStatusText(uint16_t status);
	Port_t *destPortp;
	uint32_t dlid = 0;

	IB_ENTER(__func__, nodep, start_port, end_port, 0);

	if (!nodep || start_port > end_port) {
		cs_log(VS_LOG_ERROR, "sm_set_buffer_control_table",
				"Invalid Parameters for node %s ; %p, ports %u - %u",
				nodep?sm_nodeDescString(nodep):"<NULL>", nodep, start_port, end_port);
		return (VSTATUS_ILLPARM);
	}

	num_ports = (end_port - start_port) +1;

	if (nodep->nodeInfo.NodeType == NI_TYPE_CA && num_ports > 1 ) {
		cs_log(VS_LOG_ERROR, "sm_set_buffer_control_table",
			"Failed to Set(BufferControlTable) (HFI %s) num_ports > 1 (%d)",
			sm_nodeDescString(nodep), num_ports);
		return (VSTATUS_ILLPARM);
	}

	if ((status = sm_get_mkey(nodep, &mkey, start_port)) != VSTATUS_OK)
		return (status);

	port = nodep->nodeInfo.NodeType == NI_TYPE_CA ? start_port : 0;
	destPortp = sm_get_port(nodep, port);
	if (!sm_valid_port(destPortp)) {
		cs_log(VS_LOG_VERBOSE, "sm_set_buffer_control_table",
			   "Failed to get Port %d of node %s",
			   port, sm_nodeDescString(nodep));
		return VSTATUS_BAD;
	}

	dlid = destPortp->portData->lid;

	if (uniform_bcts) {
		// All the ports have identical BCts, so send one block with the all
		// ports bit set.
		uint32_t amod = (1<<24) + (1<<8) + start_port;
		SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);

		status = SM_Set_BufferControlTable(fd_topology, amod, &addr, &bcts[0],
			mkey, &madStatus);
		if (status != VSTATUS_OK) {
			cs_log(VS_LOG_ERROR, "sm_set_buffer_control_table",
				"buffer control table query failed; node %s; %s (madStatus=%s)",
				sm_nodeDescString(nodep),
				cs_convert_status (status), sm_getMadStatusText(madStatus));
			goto error;
		} else {
			/**
			 * If the Set was successful update port data
			 */
			for (port = start_port; port <= end_port; port++) {
				Port_t *portp;
				portp = sm_get_port(nodep, port);
				if (!sm_valid_port(portp)) {
					cs_log(VS_LOG_VERBOSE, "sm_set_buffer_control_table",
									"Failed to get port %d of node %s",
									port, sm_nodeDescString(nodep));
					continue;
				}
				memcpy(&portp->portData->bufCtrlTable, &bcts[0],
								sizeof(portp->portData->bufCtrlTable));
			}
		}
	} else {
		// Send the BCTs in the fewest MADs possible. A single MAD can hold
		// up to maxcount BCTs, but may hold fewer because we need to omit
		// ports that are not up.
		//
		// In this loop, the value of num_ports is the # of BCTs that can be
		// sent in the next MAD, including the current port. We may end up
		// sending less than that if the current port is not valid or isn't up.
		for (num_ports = 1, port = start_port; port <= end_port;
			num_ports++, port ++) {
			Port_t *portp = sm_get_port(nodep, port);

			if (!sm_valid_port(portp)) {
				cs_log(VS_LOG_VERBOSE, "sm_set_buffer_control_table",
					"Failed to get port %d of node %s",
					port, sm_nodeDescString(nodep));
			}

			if (!sm_valid_port(portp) || (portp->state < IB_PORT_INIT)) {
				// We have to skip down ports, so send what we have, not
				// including the bad port.  If this is the only port
				// (num_ports == 1) then skip the send.
				if (num_ports > 1) {
					uint16_t first_port = port - num_ports + 1;
					STL_BUFFER_CONTROL_TABLE *buf = &bcts[first_port-start_port];
					uint32_t amod = ((num_ports-1)<<24) | first_port;
					SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);

					status = SM_Set_BufferControlTable(fd_topology, amod, &addr, buf, mkey, &madStatus);
					if (status != VSTATUS_OK) {
						cs_log(VS_LOG_ERROR, "sm_set_buffer_control_table",
										"buffer control table query failed; node %s; %s (madStatus=%s)",
										sm_nodeDescString(nodep),
										cs_convert_status (status), sm_getMadStatusText(madStatus));
						goto error;
					} else {
						/*
						 * If the Set was successful update port data
						 */
						uint8_t p;
						for (p = first_port; p < port; p++) {
							Port_t *portp2 = sm_get_port(nodep, p);
							if (sm_valid_port(portp2)) {
								memcpy(&portp2->portData->bufCtrlTable, &buf[p-first_port],
												sizeof(portp2->portData->bufCtrlTable));
							}
						}
					}
				}

				// Note that this will get set back to 1 by the for loop.
				num_ports = 0;
			} else if ((num_ports == maxcount) || port == end_port) {
				// Unlike the previous case, here we include the current port.
				uint16_t first_port = port - num_ports + 1;
				STL_BUFFER_CONTROL_TABLE *buf = &bcts[first_port-start_port];
				uint32_t amod = (num_ports<<24) | first_port;
				SmpAddr_t addr = SMP_ADDR_CREATE_LR(sm_lid, dlid);

				status = SM_Set_BufferControlTable(fd_topology, amod, &addr, buf, mkey, &madStatus);
				if (status != VSTATUS_OK) {
					cs_log(VS_LOG_ERROR, "sm_set_buffer_control_table",
						"buffer control table query failed; node %s; %s (madStatus=%s)",
						sm_nodeDescString(nodep),
						cs_convert_status (status), sm_getMadStatusText(madStatus));
					goto error;
				} else {
					/*
					 * If the Set was successful update port data
					 */
					uint8_t p;
					for (p = first_port; p <= port; p++) {
						Port_t *portp2 = sm_get_port(nodep, p);
						if (!portp2) {
							cs_log(VS_LOG_ERROR, "sm_set_buffer_control_table",
								"invalid pointer for node %s, port %u",
								sm_nodeDescString(nodep), p);
							status = VSTATUS_BAD;
							goto error;
						}
						memcpy(&portp2->portData->bufCtrlTable, &buf[p-first_port],
							sizeof(portp2->portData->bufCtrlTable));
					}
				}

				// Note that this will get set back to 1 by the for loop.
				num_ports = 0;
			}
		}
	}

error:
	return (status);
}
