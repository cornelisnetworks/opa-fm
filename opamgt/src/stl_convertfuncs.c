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

#include "iba/stl_mad_priv.h"
#include "stl_convertfuncs.h"

int stl_CopyIbNodeInfo(STL_NODE_INFO * dest, NODE_INFO * src, int cpyVerInfo)
{
	if (cpyVerInfo) {
		dest->BaseVersion = src->BaseVersion;
		dest->ClassVersion = src->ClassVersion;
	}
	else {
		dest->BaseVersion = STL_BASE_VERSION;
		dest->ClassVersion = STL_SM_CLASS_VERSION;
	}

	dest->NodeType = src->NodeType;
	dest->NumPorts = src->NumPorts;

	dest->SystemImageGUID = src->SystemImageGUID;

	dest->NodeGUID = src->NodeGUID;
	dest->PortGUID = src->PortGUID;

	dest->PartitionCap = src->PartitionCap;

	dest->DeviceID = src->DeviceID;

	dest->Revision = src->Revision;	

	dest->u1.AsReg32 = src->u1.AsReg32;

	return 0;
}

int stl_CopyStlNodeInfo(NODE_INFO * dest, STL_NODE_INFO * src, int cpyVerInfo)
{
	if (cpyVerInfo) {
		dest->BaseVersion = src->BaseVersion;
		dest->ClassVersion = src->ClassVersion;
	}
	else {
		dest->BaseVersion = IB_BASE_VERSION;
		dest->ClassVersion = IB_SM_CLASS_VERSION;
	}

	dest->NodeType = src->NodeType;
	dest->NumPorts = src->NumPorts;

	dest->SystemImageGUID = src->SystemImageGUID;

	dest->NodeGUID = src->NodeGUID;
	dest->PortGUID = src->PortGUID;

	dest->PartitionCap = src->PartitionCap;

	dest->DeviceID = src->DeviceID;

	dest->Revision = src->Revision;	

	dest->u1.AsReg32 = src->u1.AsReg32;

	return 0;
}

int IB2STL_NODE_INFO(NODE_INFO *pIb, STL_NODE_INFO *pStl)
{
	if (!pIb || !pStl) return -1;

	// Fields shared with IB (in IB listed order)
	pStl->BaseVersion = pIb->BaseVersion;
	pStl->ClassVersion = pIb->ClassVersion;
	pStl->NodeType = pIb->NodeType;	
	pStl->NumPorts = pIb->NumPorts;
	pStl->SystemImageGUID = pIb->SystemImageGUID;		
	pStl->NodeGUID = pIb->NodeGUID;
	pStl->PortGUID = pIb->PortGUID;
	pStl->PartitionCap = pIb->PartitionCap;
	pStl->DeviceID = pIb->DeviceID;
	pStl->Revision = pIb->Revision;
	pStl->u1.AsReg32 = pIb->u1.AsReg32;

	// STL fields with no IB equiv
	pStl->Reserved = 0;

	return 0;
};

int STL2IB_NODE_INFO(STL_NODE_INFO *pStl, NODE_INFO *pIb)
{
	if (!pIb || !pStl) return -1;

	pIb->BaseVersion = pStl->BaseVersion;
	pIb->ClassVersion = pStl->ClassVersion;
	pIb->NodeType = pStl->NodeType;	
	pIb->NumPorts = pStl->NumPorts;
	pIb->SystemImageGUID = pStl->SystemImageGUID;		
	pIb->NodeGUID = pStl->NodeGUID;
	pIb->PortGUID = pStl->PortGUID;
	pIb->PartitionCap = pStl->PartitionCap;
	pIb->DeviceID = pStl->DeviceID;
	pIb->Revision = pStl->Revision;
	pIb->u1.AsReg32 = pStl->u1.AsReg32;

	return 0;
};

int IB2STL_PORT_INFO(PORT_INFO *pIb, STL_PORT_INFO *pStl)
{
	if (!pIb || !pStl) return -1;

	// Fields shared with IB (in IB listed order)
	pStl->M_Key = pIb->M_Key;
	pStl->SubnetPrefix = pIb->SubnetPrefix;
	pStl->LID = (STL_LID)pIb->LID;
	pStl->MasterSMLID = (STL_LID)pIb->MasterSMLID;
	pStl->CapabilityMask.AsReg32 = pIb->CapabilityMask.AsReg32;
	pStl->DiagCode.AsReg16 = pIb->DiagCode;
	pStl->M_KeyLeasePeriod = pIb->M_KeyLeasePeriod;	
	pStl->LocalPortNum = pIb->LocalPortNum;
	// TODO FIX THIS pStl->LinkWidth, pIb->LinkWidth;
	//struct _LinkWidth {
	//	uint8	Enabled;		/* Enabled link width */
	//	uint8	Supported;		/* Supported link width */
	//	uint8	Active;			/* Currently active link width */
	//} LinkWidth;
#if 0
	pStl->Link;
	struct {
#if CPU_BE      
		uint8	SpeedSupported:			4;	/* Supported link speed */
		uint8	PortState:				4;	/* Port State.  */
#else
		uint8	PortState:	       		4;	/* Port State.  */
		uint8	SpeedSupported:			4;	/* Supported link speed */
#endif
#if CPU_BE      
		uint8	PortPhysicalState:		4;
		uint8	DownDefaultState:		4;
#else
		uint8   DownDefaultState:		4;
		uint8	PortPhysicalState:		4;
#endif
	} Link;
	
	struct {
#if CPU_BE
		uint8	M_KeyProtectBits:	2;	/* see mgmt key usage */
		uint8	Reserved:			3;	/* reserved, shall be zero */
		uint8	LMC:				3;	/* LID mask for multipath support */
#else
		uint8	LMC:				3;	/* LID mask for multipath support */
		uint8	Reserved:			3;	/* reserved, shall be zero */
		uint8	M_KeyProtectBits:	2;	/* see mgmt key usage */
#endif
	} s1;

	struct _LinkSpeed {
#if CPU_BE
		uint8	Active:				4;	/* Currently active link speed */
		uint8	Enabled:			4;	/* Enabled link speed */
#else
		uint8	Enabled:			4;	/* Enabled link speed */
		uint8	Active:				4;	/* Currently active link speed */
#endif
	} LinkSpeed;

	struct {
#if CPU_BE
		uint8	NeighborMTU:		4;
		uint8	MasterSMSL:			4;
#else
		uint8	MasterSMSL:			4;	/* The adminstrative SL of the master  */
										/* SM that is managing this port. */
		uint8	NeighborMTU:		4;	/* MTU of neighbor endnode connected  */
										/* to this port */
#endif
	} s2;
	
	
	struct {
		struct {
#if CPU_BE
			uint8	Cap:			4;	/* Virtual Lanes supported on this port */
			uint8	InitType:		4;	/* IB_PORT_INIT_TYPE */
#else
			uint8	InitType:		4;	/* IB_PORT_INIT_TYPE */
			uint8	Cap:			4;	/* Virtual Lanes supported on this port */
#endif
		} s;

		uint8	HighLimit;			/* Limit of high priority component of  */
										/* VL Arbitration table */
		uint8	ArbitrationHighCap;		
		uint8	ArbitrationLowCap;
	} VL;

	struct {
#if CPU_BE
		uint8	InitTypeReply:	4;	/* IB_PORT_INIT_TYPE */
		uint8	Cap:			4;	/* Maximum MTU supported by this port. */
#else
		uint8	Cap:			4;	/* Maximum MTU supported by this port. */
		uint8	InitTypeReply:	4;	/* IB_PORT_INIT_TYPE */
#endif
	} MTU;

	struct {					/* transmitter Queueing Controls */
#if CPU_BE
		uint8	VLStallCount:	3;
		uint8	HOQLife:		5;
#else
		uint8	HOQLife:		5;	/* Applies to routers & switches only */
		uint8	VLStallCount:	3;	/* Applies to switches only.  */
#endif
	} XmitQ;
	
	struct {
#if CPU_BE
		uint8	OperationalVL:					4;	/* Virtual Lanes  */
		uint8	PartitionEnforcementInbound:	1;
		uint8	PartitionEnforcementOutbound:	1;
		uint8	FilterRawInbound:				1;
		uint8	FilterRawOutbound:				1;
#else
		uint8	FilterRawOutbound:				1;
		uint8	FilterRawInbound:				1;
		uint8	PartitionEnforcementOutbound:	1;
		uint8	PartitionEnforcementInbound:	1;
		uint8	OperationalVL:					4;	/* Virtual Lanes  */
													/* operational on this port */
#endif
	} s3;

	struct _Violations {
		uint16	M_Key;
		uint16	P_Key;
		uint16	Q_Key;
	} Violations;

	pStl->GUIDCap = pIb->GUIDCap;

	struct {
#if CPU_BE
		uint8	ClientReregister:	1;
		uint8	Reserved:	2;
		uint8	Timeout:	5;
#else
		uint8	Timeout:	5;	/* Timer value used for subnet timeout  */
		uint8	Reserved:	2;
		uint8	ClientReregister:	1;
#endif
	} Subnet;


	struct {
#if CPU_BE
		uint8	Reserved:	3;
		uint8	TimeValue:	5;
#else
		uint8	TimeValue:	5;
		uint8	Reserved:		3;
#endif
			
	} Resp;

	struct	_ERRORS {
#if CPU_BE
		uint8	LocalPhys:	4;
		uint8	Overrun:	4;
#else
		uint8	Overrun:	4;
		uint8	LocalPhys:	4;
#endif
	} Errors;
#endif // 0

#if 0
	struct _LATENCY {
#if CPU_BE
		uint32	Reserved:		8;
		uint32	LinkRoundTrip:	24;	/* LinkRoundTripLatency */
#else
		uint32	LinkRoundTrip:	24;
		uint32	Reserved:		8;
#endif
	} Latency;
#endif // 0

#if 0
	struct _LinkSpeedExt {
#if CPU_BE
		uint8	Active:			4;	/* Currently active link speed extended */
		uint8	Supported:		4;	/* Supported link speed extended */
#else
		uint8	Supported:		4;	/* Supported link speed extended */
		uint8	Active:			4;	/* Currently active link speed extended */
#endif
#if CPU_BE
		uint8	Reserved:		3;	/* Reserved */
		uint8	Enabled:		5;	/* Enabled link speed extended */
#else
		uint8	Enabled:		5;	/* Enabled link speed extended */
		uint8	Reserved:		3;	/* Reserved */
#endif
	} LinkSpeedExt;
#endif // 0

	// STL fields with no IB equiv
	// TODO
	return -1;
};

int STL2IB_PORT_INFO(STL_PORT_INFO *pStl, PORT_INFO *pIb)
{
	if (!pIb || !pStl) return -1;

	// TODO
	return -1;
};

int IB2STL_SWITCH_INFO(SWITCH_INFO *pIb, STL_SWITCH_INFO *pStl)
{
	if (!pIb || !pStl) return -1;

	// Fields shared with IB (in IB listed order)
	// STL fields with no IB equiv
	// TODO
	return -1;
};

int STL2IB_SWITCH_INFO(STL_SWITCH_INFO *pStl, SWITCH_INFO *pIb)
{
	if (!pIb || !pStl) return -1;

	// TODO
	return -1;
};
