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

#ifndef _IBA_STL_HELPER_H_
#define _IBA_STL_HELPER_H_

#include <stdio.h>
#include "ib_helper.h"
#include "iba/stl_sm.h"
#if defined(VXWORKS)
#include "private/stdioP.h" // pick up snprintf extern definition
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Convert STL_PORT_STATE to a constant string
 */
static __inline const char *
StlPortStateToText(uint8_t state)
{
	return IbPortStateToText((IB_PORT_STATE)state);
}

/*
 * Convert STL_PORT_PHYS_STATE to a constant string
 */
static __inline const char*
StlPortPhysStateToText( uint8_t state )
{
	switch (state)
	{
		case STL_PORT_PHYS_OFFLINE:
			return "Offline";
		case STL_PORT_PHYS_TEST:
			return "Test";
	}
	return IbPortPhysStateToText((IB_PORT_PHYS_STATE)state);
}

static __inline int IsPortInitialized(STL_PORT_STATES portStates)
{
	return (portStates.s.PortState == IB_PORT_ARMED
			|| portStates.s.PortState == IB_PORT_ACTIVE);
}

static __inline uint32 StlMbpsToStaticRate(uint32 rate_mbps)
{
	/* 1Gb rate is obsolete */
	if (rate_mbps <= 2500)
		return IB_STATIC_RATE_2_5G;
	else if (rate_mbps <= 5000)
		return IB_STATIC_RATE_5G;
	else if (rate_mbps <= 10000)
		return IB_STATIC_RATE_10G;
	else if (rate_mbps <= 12500)
		return IB_STATIC_RATE_14G; // STL_STATIC_RATE_12_5G;
	else if (rate_mbps <= 14062)
		return IB_STATIC_RATE_14G;
	else if (rate_mbps <= 20000)
		return IB_STATIC_RATE_20G;
	else if (rate_mbps <= 25781)
		return IB_STATIC_RATE_25G;
	else if (rate_mbps <= 30000)
		return IB_STATIC_RATE_30G;
	else if (rate_mbps <= 37500)
		return IB_STATIC_RATE_40G; // STL_STATIC_RATE_37_5G;
	else if (rate_mbps <= 40000)
		return IB_STATIC_RATE_40G;
	else if (rate_mbps <= 50000)
		return IB_STATIC_RATE_56G; // STL_STATIC_RATE_50G;
	else if (rate_mbps <= 56250)
		return IB_STATIC_RATE_56G;
	else if (rate_mbps <= 60000)
		return IB_STATIC_RATE_60G;
	else if (rate_mbps <= 75000)
		return IB_STATIC_RATE_80G; // STL_STATIC_RATE_75G;
	else if (rate_mbps <= 80000)
		return IB_STATIC_RATE_80G;
	else if (rate_mbps <= 103125)
		return IB_STATIC_RATE_100G;
	else
		return IB_STATIC_RATE_100G;
#if 0
	// future
	else if (rate_mbps <= 112500)
		return IB_STATIC_RATE_112G;
	else if (rate_mbps <= 120000)
		return IB_STATIC_RATE_120G;
	else if (rate_mbps <= 150000)
		return STL_STATIC_RATE_150G;
	else if (rate_mbps <= 168750)
		return IB_STATIC_RATE_168G;
	else if (rate_mbps <= 206250)
		return IB_STATIC_RATE_200G;
	else if (rate_mbps <= 225000)
		return STL_STATIC_RATE_225G;
	else if (rate_mbps <= 300000)
		return IB_STATIC_RATE_300G;	
	else
		return STL_STATIC_RATE_400G;
#endif
}

/* Convert static rate to text */
static __inline const char*
StlStaticRateToText(uint32 rate)
{
	switch (rate)
	{
		case IB_STATIC_RATE_DONTCARE:
			return "any";
		case IB_STATIC_RATE_1GB:
			return "1g";
		case IB_STATIC_RATE_2_5G:
			return "2.5g";
		case IB_STATIC_RATE_10G:
			return "10g";
		case IB_STATIC_RATE_30G:
			return "30g";
		case IB_STATIC_RATE_5G:
			return "5g";
		case IB_STATIC_RATE_20G:
			return "20g";
		case IB_STATIC_RATE_40G:
			return "37.5g";				// STL_STATIC_RATE_37_5G;
		case IB_STATIC_RATE_60G:
			return "60g";
		case IB_STATIC_RATE_80G:
			return "75g";				// STL_STATIC_RATE_75G;
		case IB_STATIC_RATE_120G:
			return "120g";
		case IB_STATIC_RATE_14G:
			return "12.5g";				// STL_STATIC_RATE_12_5G;
		case IB_STATIC_RATE_25G:
			return "25g";				// 25.78125g
		case IB_STATIC_RATE_56G:
			return "50g";				// STL_STATIC_RATE_50G;
		case IB_STATIC_RATE_100G:
			return "100g";				// 103.125g
		case IB_STATIC_RATE_112G:
			return "112g";				// 112.5g
		case IB_STATIC_RATE_200G:
			return "200g";				// 206.25g
		case IB_STATIC_RATE_168G:
			return "168g";				// 168.75g
		case IB_STATIC_RATE_300G:
			return "300g";				// 309.375g
		default:
			return "???";
	}
}

static __inline uint32 StlLinkSpeedToMbps(uint32 speed)
{
	switch(speed) {
	default:
	case STL_LINK_SPEED_12_5G: return 12500;
	case STL_LINK_SPEED_25G: return 25781;
	}
}

static __inline uint32 StlLinkWidthToInt(uint32 width)
{
	switch(width) {
	default:
	case STL_LINK_WIDTH_1X: return 1;
	case STL_LINK_WIDTH_2X: return 2;
	case STL_LINK_WIDTH_3X: return 3;
	case STL_LINK_WIDTH_4X: return 4;
	}
}

static __inline uint32 StlStaticRateToMbps(IB_STATIC_RATE rate)
{
	switch(rate) {
	case IB_STATIC_RATE_14G: // STL_STATIC_RATE_12_5G
		return 12890;			// half of 25781 (25G)
	default:
	case IB_STATIC_RATE_25G:
		return 25781;
	case IB_STATIC_RATE_40G: // STL_STATIC_RATE_37_5G
		return 37500;
	case IB_STATIC_RATE_56G: // STL_STATIC_RATE_50G
		return 51562;
	case IB_STATIC_RATE_80G: // STL_STATIC_RATE_75G
		return 77343;
	case IB_STATIC_RATE_100G:
		return 103125;
	case IB_STATIC_RATE_200G:
		return 206250;
	case IB_STATIC_RATE_300G:
		return 309375;
	}
}
/* convert data Mbps to wire MBps for stl
 *  speeds use 64b66b wire encoding   mbps wire Megabit/s * 1Byte/8bits * 64 data bits / 66 wire bits = X data MegaByte/s
 */
#define StlmbpsToMBpsExt(mbps) { 			\
	((uint32)((uint64)mbps * 8LL / 66LL));	\
}

/* convert static rate to MByte/sec units, M=10^6 */
static __inline uint32
StlStaticRateToMBps(IB_STATIC_RATE rate)
{
	return (StlmbpsToMBpsExt(StlStaticRateToMbps(rate)));
}

/* return the best link speed between two ports. */
static __inline uint32
StlExpectedLinkSpeed(uint32 a, uint32 b)
{
    if ((STL_LINK_SPEED_25G & a) && (STL_LINK_SPEED_25G & b))
        return STL_LINK_SPEED_25G;
    else if ((IB_LINK_SPEED_25G & a) && (IB_LINK_SPEED_25G & b))
        return IB_LINK_SPEED_25G;
    else if ((IB_LINK_SPEED_14G & a) && (IB_LINK_SPEED_14G & b))
        return IB_LINK_SPEED_14G;
	else if ((STL_LINK_SPEED_12_5G & a) && (STL_LINK_SPEED_12_5G & b))
		return STL_LINK_SPEED_12_5G;
    else if ((IB_LINK_SPEED_10G & a) && (IB_LINK_SPEED_10G & b))
        return IB_LINK_SPEED_10G;
    else if ((IB_LINK_SPEED_5G & a) && (IB_LINK_SPEED_5G & b))
        return IB_LINK_SPEED_5G;
    else if ((IB_LINK_SPEED_2_5G & a) && (IB_LINK_SPEED_2_5G & b))
        return IB_LINK_SPEED_2_5G;
    else
        return 0;    /* no speed? */
}

/* return the best link speed set in the bit mask */
static __inline uint32
StlBestLinkSpeed(uint32 speed)
{
    if (STL_LINK_SPEED_25G & speed)
        return STL_LINK_SPEED_25G;
    else if (IB_LINK_SPEED_25G & speed)
        return IB_LINK_SPEED_25G;
    else if (IB_LINK_SPEED_14G & speed)
        return IB_LINK_SPEED_14G;
	else if (STL_LINK_SPEED_12_5G & speed)
		return STL_LINK_SPEED_12_5G;
    else if (IB_LINK_SPEED_10G & speed)
        return IB_LINK_SPEED_10G;
    else if (IB_LINK_SPEED_5G & speed)
        return IB_LINK_SPEED_5G;
    else if (IB_LINK_SPEED_2_5G & speed)
        return IB_LINK_SPEED_2_5G;
    else
        return 0;    /* no speed? */
}

/* 
 * Return the best possible link width supported by or enabled in a port.
 */
static __inline uint32
StlBestLinkWidth(uint32 a)
{
	if (STL_LINK_WIDTH_4X & a)
		return STL_LINK_WIDTH_4X;
	else if (STL_LINK_WIDTH_3X & a)
		return STL_LINK_WIDTH_3X;
	else if (STL_LINK_WIDTH_2X & a)
		return STL_LINK_WIDTH_2X;
	else if (STL_LINK_WIDTH_1X & a) 
		return STL_LINK_WIDTH_1X;
	else
		return 0;
}

/* compare 2 link widths and report expected operational width
 * can be used to compare Enabled or Supported values in connected ports
 */
static __inline uint16
StlExpectedLinkWidth(uint32 a, uint32 b)
{
	if ((STL_LINK_WIDTH_4X & a) && (STL_LINK_WIDTH_4X & b))
		return STL_LINK_WIDTH_4X;
	else if ((STL_LINK_WIDTH_3X & a) && (STL_LINK_WIDTH_3X & b))
		return STL_LINK_WIDTH_3X;
	else if ((STL_LINK_WIDTH_2X & a) && (STL_LINK_WIDTH_2X & b))
		return STL_LINK_WIDTH_2X;
	else if ((STL_LINK_WIDTH_1X & a) && (STL_LINK_WIDTH_1X & b))
		return STL_LINK_WIDTH_1X;
	else
		return STL_LINK_WIDTH_NOP;    /* link should come up */
}

static __inline uint32 StlLinkSpeedWidthToMbps(uint32 speed, uint32 width)
{
	return StlLinkSpeedToMbps(speed) * StlLinkWidthToInt(width);
}

static __inline uint32 StlLinkSpeedWidthToStaticRate(uint32 speed, uint32 width)
{
	return StlMbpsToStaticRate(StlLinkSpeedWidthToMbps(speed, width));
}

/**
 * This is a kind of nasty macro which "exits" to the label "out" if the
 * snprintf did not have enough room.  It keeps the Print Functions below
 * concise.  But does have side effects of changing "n" and possibly vectoring
 * out of the enclosing block.
 */
#define PRINT_OR_OUT(str, len, val) { \
		i = snprintf(str+n, len-n, val); \
		if (i >= len-n) { \
			DEBUG_ASSERT(0 == "IbPrint: ERROR buffer length short\n"); \
			goto out; \
		} \
		n+=i; \
	}

/* convert link width to text */
static __inline char*
StlLinkWidthToText(uint16_t w, char *buf, size_t len)
{
	int i, j, l;

#define STL_WIDTH_TEXT_LENGTH 16

	static const char *StlWidthText[STL_WIDTH_TEXT_LENGTH] = {
		"1", "2", "3", "4", "?", "?", "?", "?", 
		"?", "?", "?", "?", "?", "?", "?", "?", 
	};
	
	if (w == STL_LINK_WIDTH_NOP) {
		snprintf(buf,len-1,"None");
		buf[len-1]=0;
	} else {
		l=len-4;	// the max we can tack on in 1 pass is 3 bytes.

		// Loop terminates as soon as the width is zero.
		for (i=0, j=0; w!=0 && i<STL_WIDTH_TEXT_LENGTH && j<l; i++, w>>=1) {
			if (w & 1) {
				if (j>0) buf[j++]=',';
				j+=snprintf(buf+j,len-j,"%s", StlWidthText[i]);
			}
		}
	}
	return buf;
}

static __inline const char*
StlLinkSpeedToText(uint16_t speed, char *str, size_t len)
{
	size_t n = 0;
	size_t i = 0;

	if (speed == STL_LINK_SPEED_NOP) {
		PRINT_OR_OUT(str, len, "None");
		goto out;
	}

	str[0] = '\0';

	if ((speed & (STL_LINK_SPEED_12_5G|STL_LINK_SPEED_25G))
	  != speed) {
		i = snprintf(str, len, "Unexpected (0x%04X) ", speed);
		if (i >= len-n) {
			DEBUG_ASSERT(0 == "IbPrint: ERROR buffer length short\n");
			goto out;
		}
		// short-circuit, don't print the rest
		goto out;
		n+=i;
	}

	if (speed & STL_LINK_SPEED_12_5G)
		PRINT_OR_OUT(str, len, "12.5Gb,");
	if (speed & STL_LINK_SPEED_25G)
		PRINT_OR_OUT(str, len, "25Gb,");
    str[n-1] = 0; // Eliminate trailing comma

out:
	return (str);
}


static __inline const char*
StlPortLinkModeToText(uint16_t mode, char *str, size_t len)
{
	size_t n = 0;
	size_t i = 0;

	if (mode == STL_PORT_LINK_MODE_NOP) {
		PRINT_OR_OUT(str, len, "Noop");
		goto out;
	}

	str[0]='\0';

	if (mode & STL_PORT_LINK_MODE_ETH)
		PRINT_OR_OUT(str, len, "ETH,");
	if (mode & STL_PORT_LINK_MODE_STL)
		PRINT_OR_OUT(str, len, "STL,");
    str[n-1] = 0; // Eliminate trailing comma

out:
	return str;
}

static __inline const char*
StlPortLtpCrcModeToText(uint16_t mode, char *str, size_t len)
{
	size_t n = 0;
	size_t i = 0;

	if (mode == STL_PORT_LTP_CRC_MODE_NONE) {
		PRINT_OR_OUT(str, len, "None");
		goto out;
	}

	str[0]='\0';

	if (mode & STL_PORT_LTP_CRC_MODE_14)
		PRINT_OR_OUT(str, len, "14-bit,");
	if (mode & STL_PORT_LTP_CRC_MODE_16)
		PRINT_OR_OUT(str, len, "16-bit,");
	if (mode & STL_PORT_LTP_CRC_MODE_48)
		PRINT_OR_OUT(str, len, "48-bit,");
	if (mode & STL_PORT_LTP_CRC_MODE_12_16_PER_LANE)
		PRINT_OR_OUT(str, len, "12-16/lane,");
    str[n-1] = 0; // Eliminate trailing comma

out:
	return str;
}

static __inline const char*
StlLinkInitReasonToText(uint8 initReason)
{
	// all output strings are 13 characters or less
	switch(initReason) {
		case STL_LINKINIT_REASON_NOP:
			return("None");
		case STL_LINKINIT_REASON_LINKUP:
			return("LinkUp");
		case STL_LINKINIT_REASON_FLAPPING:
			return("Flapping");
		case STL_LINKINIT_OUTSIDE_POLICY:
			return("Out of policy");
		case STL_LINKINIT_QUARANTINED:
			return("Quarantined");
		case STL_LINKINIT_INSUFIC_CAPABILITY:
			return("Insuffic Cap");
		default:
			return(" ???? ");
	}		
}

static __inline const char*
StlLinkDownReasonToText(uint8 downReason)
{
	// all output strings are 13 characters or less
	switch (downReason) {
		case STL_LINKDOWN_REASON_NONE:
			return("None");
		case STL_LINKDOWN_REASON_RCV_ERROR_0:
			return("Rcv Error 0"); 
		case STL_LINKDOWN_REASON_BAD_PKT_LEN:
			return("Bad Pkt Len"); 
		case STL_LINKDOWN_REASON_PKT_TOO_LONG:
			return("Pkt Too Long"); 
		case STL_LINKDOWN_REASON_PKT_TOO_SHORT:
			return("Pkt Too Short"); 
		case STL_LINKDOWN_REASON_BAD_SLID:
			return("Bad slid"); 
		case STL_LINKDOWN_REASON_BAD_DLID:
			return("Bad dlid"); 
		case STL_LINKDOWN_REASON_BAD_L2:
			return("Bad L2"); 
		case STL_LINKDOWN_REASON_BAD_SC:
			return("Bad SC"); 
		case STL_LINKDOWN_REASON_RCV_ERROR_8:
			return("Rcv Error 8"); 
		case STL_LINKDOWN_REASON_BAD_MID_TAIL:
			return("Bad Mid Tail"); 
		case STL_LINKDOWN_REASON_RCV_ERROR_10:
			return("Rcv Error 10"); 
		case STL_LINKDOWN_REASON_PREEMPT_ERROR:
			return("Preempt Error"); 
		case STL_LINKDOWN_REASON_PREEMPT_VL15:
			return("Preempt VL15"); 
		case STL_LINKDOWN_REASON_BAD_VL_MARKER:
			return("Bad VL Marker"); 
		case STL_LINKDOWN_REASON_RCV_ERROR_14:
			return("Rcv Error 14"); 
		case STL_LINKDOWN_REASON_RCV_ERROR_15:
			return("Rcv Error 15"); 
		case STL_LINKDOWN_REASON_BAD_HEAD_DIST:
			return("Bad Head Dist"); 
		case STL_LINKDOWN_REASON_BAD_TAIL_DIST:
			return("Bad Tail Dist"); 
		case STL_LINKDOWN_REASON_BAD_CTRL_DIST:
			return("Bad Ctrl Dist"); 
		case STL_LINKDOWN_REASON_BAD_CREDIT_ACK:
			return("Bad Cred Ack"); 
		case STL_LINKDOWN_REASON_UNSUPPORTED_VL_MARKER:
			return("Unsup VL Mrkr"); 
		case STL_LINKDOWN_REASON_BAD_PREEMPT:
			return("Bad Preempt"); 
		case STL_LINKDOWN_REASON_BAD_CONTROL_FLIT:
			return("Bad Ctrl Flit"); 
		case STL_LINKDOWN_REASON_EXCEED_MULTICAST_LIMIT:
			return("Exc MC Limit"); 
		case STL_LINKDOWN_REASON_RCV_ERROR_24:
			return("Rcv Error 24"); 
		case STL_LINKDOWN_REASON_RCV_ERROR_25:
			return("Rcv Error 25"); 
		case STL_LINKDOWN_REASON_RCV_ERROR_26:
			return("Rcv Error 26"); 
		case STL_LINKDOWN_REASON_RCV_ERROR_27:
			return("Rcv Error 27"); 
		case STL_LINKDOWN_REASON_RCV_ERROR_28:
			return("Rcv Error 28"); 
		case STL_LINKDOWN_REASON_RCV_ERROR_29:
			return("Rcv Error 29"); 
		case STL_LINKDOWN_REASON_RCV_ERROR_30:
			return("Rcv Error 30"); 
		case STL_LINKDOWN_REASON_EXCESSIVE_BUFFER_OVERRUN:
			return("Exc Buf OVR"); 
		case STL_LINKDOWN_REASON_UNKNOWN:
			return("Unknown"); 
		case STL_LINKDOWN_REASON_REBOOT:
			return("Reboot"); 
		case STL_LINKDOWN_REASON_NEIGHBOR_UNKNOWN:
			return("Neigh Unknown"); 
		case STL_LINKDOWN_REASON_FM_BOUNCE:
			return("FM Bounce"); 
		case STL_LINKDOWN_REASON_SPEED_POLICY:
			return("Speed Policy"); 
		case STL_LINKDOWN_REASON_WIDTH_POLICY:
			return("Width Policy"); 
		case STL_LINKDOWN_REASON_DISCONNECTED:
			return("Disconnected"); 
		case STL_LINKDOWN_REASON_LOCAL_MEDIA_NOT_INSTALLED:	
			return("No Loc Media"); 
		case STL_LINKDOWN_REASON_NOT_INSTALLED:	
			return("Not Installed"); 
		case STL_LINKDOWN_REASON_CHASSIS_CONFIG:
			return("Chassis Conf"); 
		case STL_LINKDOWN_REASON_END_TO_END_NOT_INSTALLED:
			return("No End to End"); 
		case STL_LINKDOWN_REASON_POWER_POLICY:
			return("Power Policy"); 
		case STL_LINKDOWN_REASON_LINKSPEED_POLICY:
			return("Speed Policy"); 
		case STL_LINKDOWN_REASON_LINKWIDTH_POLICY:
			return("Width_Policy"); 
		case STL_LINKDOWN_REASON_SWITCH_MGMT:
			return("Switch Mgmt"); 
		case STL_LINKDOWN_REASON_SMA_DISABLED:
			return("SMA Disabled"); 
		case STL_LINKDOWN_REASON_TRANSIENT:	
			return("Transient"); 
		default:
			return " ???? ";
	};
}

static __inline const char*
StlPortOfflineDisabledReasonToText(uint8 offlineReason)
{
	// all output strings are 13 characters or less
	// same text as the relevant LinkDownReason subset
	switch(offlineReason) {
		case STL_OFFDIS_REASON_NONE:
			return "None";
		case STL_OFFDIS_REASON_DISCONNECTED:
			return "Disconnected";
		case STL_OFFDIS_REASON_LOCAL_MEDIA_NOT_INSTALLED:
			return "No Loc Media";
		case STL_OFFDIS_REASON_NOT_INSTALLED:
			return "Not installed";
		case STL_OFFDIS_REASON_CHASSIS_CONFIG:
			return "Chassis Conf";
		case STL_OFFDIS_REASON_END_TO_END_NOT_INSTALLED:
			return "No End to End";
		case STL_OFFDIS_REASON_POWER_POLICY:
			return "Power Policy";
		case STL_OFFDIS_REASON_LINKSPEED_POLICY:
			return "Speed Policy";
		case STL_OFFDIS_REASON_LINKWIDTH_POLICY:
			return "Width Policy";
		case STL_OFFDIS_REASON_SWITCH_MGMT:
			return "Switch Mgmt";
		case STL_OFFDIS_REASON_SMA_DISABLED:
			return "SMA disabled";
		case STL_OFFDIS_REASON_TRANSIENT:
			return "Transient";
		default:
			return " ???? ";
	};
}

static __inline const char*
StlSMStateToText(SM_STATE state)
{
	return ((state == SM_INACTIVE)? "Inactive":
			(state == SM_DISCOVERING)? "Discovering":
			(state == SM_STANDBY)? "Standby":
			(state == SM_MASTER)? "Master": "???");
}

static __inline
void FormatStlCapabilityMask3(char *buf, STL_CAPABILITY_MASK3 cmask, int buflen)
{
	snprintf(buf, buflen, "%s%s%s%s%s%s%s%s",
		cmask.s.IsSnoopSupported?"SN":"",
		cmask.s.IsAsyncSC2VLSupported?"aSC2VL":"",
		cmask.s.IsAddrRangeConfigSupported?"ARC":"",
		cmask.s.IsPassThroughSupported?"PT":"",
		cmask.s.IsSharedSpaceSupported?"SS":"",
		cmask.s.IsVLMarkerSupported?"VLM":"",
		cmask.s.IsVLrSupported?"VLr":"",
		cmask.AsReg16?"":"-");
	buf[buflen-1] = '\0';
}

static __inline
void FormatStlPortErrorAction(char *buf, const STL_PORT_INFO *pPortInfo, int buflen)
{
	snprintf(buf, buflen, "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
		pPortInfo->PortErrorAction.s.ExcessiveBufferOverrun?"EBO":"",
		pPortInfo->PortErrorAction.s.FmConfigErrorExceedMulticastLimit?"CE-EML":"",
		pPortInfo->PortErrorAction.s.FmConfigErrorBadControlFlit?"CE-BCF":"",
		pPortInfo->PortErrorAction.s.FmConfigErrorBadPreempt?"CE-BP":"",
		pPortInfo->PortErrorAction.s.FmConfigErrorUnsupportedVLMarker?"CE-UVLM":"",
		pPortInfo->PortErrorAction.s.FmConfigErrorBadCrdtAck?"CE-BCA":"",
		pPortInfo->PortErrorAction.s.FmConfigErrorBadCtrlDist?"CE-BCD":"",
		pPortInfo->PortErrorAction.s.FmConfigErrorBadTailDist?"CE-BTD":"",
		pPortInfo->PortErrorAction.s.FmConfigErrorBadHeadDist?"CE-BHD":"",
		pPortInfo->PortErrorAction.s.PortRcvErrorBadVLMarker?"R-BVLM":"",
		pPortInfo->PortErrorAction.s.PortRcvErrorPreemptVL15?"R-PV15":"",
		pPortInfo->PortErrorAction.s.PortRcvErrorPreemptError?"R-PE":"",
		pPortInfo->PortErrorAction.s.PortRcvErrorBadMidTail?"R-BMT":"",
		pPortInfo->PortErrorAction.s.PortRcvErrorBadSC?"R-BSC":"",
		pPortInfo->PortErrorAction.s.PortRcvErrorBadL2?"R-BL2":"",
		pPortInfo->PortErrorAction.s.PortRcvErrorBadDLID?"R-BDLID":"",
		pPortInfo->PortErrorAction.s.PortRcvErrorBadSLID?"R-BSLID":"",
		pPortInfo->PortErrorAction.s.PortRcvErrorPktLenTooShort?"R-LTS":"",
		pPortInfo->PortErrorAction.s.PortRcvErrorPktLenTooLong?"R-LTL":"",
		pPortInfo->PortErrorAction.s.PortRcvErrorBadPktLen?"R-BPL":"");
	buf[buflen-1] = '\0';
}

static __inline
void FormatStlVLStalls(char *buf, const STL_PORT_INFO *pPortInfo, int buflen)
{
	// NOTO BENE: This code depends on VLStallCount being less than 8 bits long...
	int i, l;
	for (i = 0, l = 0; (i < STL_MAX_VLS) & (l<(buflen-3)); i++) {
		l+=sprintf(&buf[i*2], "%2x", pPortInfo->XmitQ[i].VLStallCount);
	}
}

/**
	Format the HOQLife information for VLs in the range [start, end) for output.  Sets '\0' at end.  Last VL may be truncated if space is exhausted.

	@return Index of highest VL that was formatted to @c buf.  @c end indicates all were.
*/
static __inline
int FormatStlVLHOQLife(char *buf, const STL_PORT_INFO *pPortInfo, int start, int end, int buflen)
{
	int i, l;
	end = MIN(end, STL_MAX_VLS);
	l = 0;
	for (i = start; i < end; i++) {
		if (i != start) {
			if (l >= buflen) break;
			l += snprintf(&buf[l], buflen - l, " ");
		}

		if (l >= buflen) break;
		l += snprintf(&buf[l], buflen - l, "0x%02x", pPortInfo->XmitQ[i].HOQLife);
	}

	if (l >= buflen)
		buf[buflen - 1] = '\0'; // ran out of space, cap it off
	return i;
}

static __inline const char*
StlQPServiceTypeToText(uint8_t code)
{
	switch (code) {
		case CC_RC_TYPE:
			return "Reliable Connection";
		case CC_UC_TYPE:
			return "Unreliable Connection";
		case CC_RD_TYPE:
			return "Reliable Datagram";
		case CC_UD_TYPE:
			return "Unreliable Datagram";
		default:
			return "Unknown";
	}
}

static __inline
int IsStlCableInfoActiveCable(uint8_t code_xmit)
{
	if ((code_xmit == STL_CIB_STD_TXTECH_CU_UNEQ) ||
			(code_xmit == STL_CIB_STD_TXTECH_CU_PASSIVEQ) ||
			(code_xmit > STL_CIB_STD_TXTECH_MAX))
		return 0;
	else
		return 1;

}	// End of IsStlCableInfoActiveCable()

static __inline
int IsStlCableInfoCableLengthValid(uint8_t code_xmit, uint8_t code_connector)
{
	if ( (code_xmit == STL_CIB_STD_TXTECH_OTHER) ||
			( (code_xmit <= STL_CIB_STD_TXTECH_1490_DFB) &&
			(code_connector != STL_CIB_STD_CONNECTOR_NO_SEP) ) ||
			(code_xmit > STL_CIB_STD_TXTECH_MAX) )
		return 0;
	else
		return 1;

}	// End of IsStlCableInfoCableLengthValid()

static __inline
int IsStlCableInfoCableCertified(uint8_t code_cert)
{
	if (code_cert == STL_CIB_STD_OPA_CERTIFIED_CABLE)
		return 1;
	else
		return 0;

}	// End of IsStlCableInfoCableLengthValid()

static __inline
void StlCableInfoBitRateToText(uint8_t code_low, uint8_t code_high, char *text_out)
{
	if (! text_out)
		return;
	if (code_low == STL_CIB_STD_RATELOW_EXCEED)
		sprintf(text_out, "%u Gb", code_high / 4);
	else
		sprintf(text_out, "%u Gb", code_low / 10);
	return;

}	// End of StlCableInfoBitRateToText()

static __inline
const char * StlCableInfoOpaCertifiedRateToText(uint8_t code_rate)
{
	if (code_rate & STL_CIB_STD_OPACERTRATE_4X25G)
		return "4x25G";
	else
		return "Undefined";

}	// End of StlCableInfoOpaCertifiedRateToText()

static __inline
const char * StlCableInfoPowerClassToText(uint8_t code_low, uint8_t code_high)
{
	switch (code_high) {
	case STL_CIB_STD_PWRHIGH_LEGACY:
		switch (code_low) {
		case STL_CIB_STD_PWRLOW_1_5:
			return "Power Class 1, 1.5W max";
		case STL_CIB_STD_PWRLOW_2_0:
			return "Power Class 2, 2.0W max";
		case STL_CIB_STD_PWRLOW_2_5:
			return "Power Class 3, 2.5W max";
		case STL_CIB_STD_PWRLOW_3_5:
			return "Power Class 4, 3.5W max";
		default:
			return "Undefined";
		}
		break;
	case STL_CIB_STD_PWRHIGH_4_0:
		return "Power Class 5, 4.0W max";
	case STL_CIB_STD_PWRHIGH_4_5:
		return "Power Class 6, 4.5W max";
	case STL_CIB_STD_PWRHIGH_5_0:
		return "Power Class 7, 5.0W max";
	default:
		return "Undefined";
	}

}	// End of StlCableInfoPowerClassToText()

static __inline
void StlCableInfoCableTypeToTextShort(uint8_t code_xmit, uint8_t code_connector, char *text_out)
{
	if (! text_out)
		return;
	if (code_xmit <= STL_CIB_STD_TXTECH_1490_DFB) {
		if (code_xmit == STL_CIB_STD_TXTECH_OTHER) {
			strcpy(text_out, "Other");
			return;
		}
		if (code_connector == STL_CIB_STD_CONNECTOR_NO_SEP) {
			strcpy(text_out, "AOC");
			return;
		}
		strcpy(text_out, "OpticXcvr");
		return;
	}
	if (code_xmit <= STL_CIB_STD_TXTECH_CU_PASSIVEQ) {
		strcpy(text_out, "PassiveCu");
		return;
	}
	if (code_xmit <= STL_CIB_STD_TXTECH_CU_LINACTEQ) {
		strcpy(text_out, "ActiveCu");
		return;
	} else {
		strcpy(text_out, "Undefined");
		return;
	}

}	// End of StlCableInfoCableTypeToTextShort()

static __inline
void StlCableInfoCableTypeToTextLong(uint8_t code_xmit, uint8_t code_connector, char *text_out)
{
	if (! text_out)
		return;
	if ((code_xmit <= STL_CIB_STD_TXTECH_1490_DFB) && (code_xmit != STL_CIB_STD_TXTECH_OTHER)) {
		if (code_connector == STL_CIB_STD_CONNECTOR_NO_SEP)
			strcpy(text_out, "AOC, ");
		else
			strcpy(text_out, "Optical transceiver, ");
	}
	switch (code_xmit) {
	case STL_CIB_STD_TXTECH_850_VCSEL:
		strcat(text_out, "850nm VCSEL");
		break;
	case STL_CIB_STD_TXTECH_1310_VCSEL:
		strcat(text_out, "1310nm VCSEL");
		break;
	case STL_CIB_STD_TXTECH_1550_VCSEL:
		strcat(text_out, "1550nm VCSEL");
		break;
	case STL_CIB_STD_TXTECH_1310_FP:
		strcat(text_out, "1310nm FP");
		break;
	case STL_CIB_STD_TXTECH_1310_DFB:
		strcat(text_out, "1310nm DFB");
		break;
	case STL_CIB_STD_TXTECH_1550_DFB:
		strcat(text_out, "1550nm DFB");
		break;
	case STL_CIB_STD_TXTECH_1310_EML:
		strcat(text_out, "1310nm EML");
		break;
	case STL_CIB_STD_TXTECH_1550_EML:
		strcat(text_out, "1550nm EML");
		break;
	case STL_CIB_STD_TXTECH_OTHER:
		strcpy(text_out, "Other/Undefined");
		break;
	case STL_CIB_STD_TXTECH_1490_DFB:
		strcat(text_out, "1490nm DFB");
		break;
	case STL_CIB_STD_TXTECH_CU_UNEQ:
		strcpy(text_out, "Passive copper cable");
		break;
	case STL_CIB_STD_TXTECH_CU_PASSIVEQ:
		strcpy(text_out, "Equalized passive copper cable");
		break;
	case STL_CIB_STD_TXTECH_CU_NFELIMACTEQ:
		strcpy(text_out, "Active copper cable, TX and RX repeaters");
		break;
	case STL_CIB_STD_TXTECH_CU_FELIMACTEQ:
		strcpy(text_out, "Active copper cable, RX repeaters");
		break;
	case STL_CIB_STD_TXTECH_CU_NELIMACTEQ:
		strcpy(text_out, "Active copper cable, TX repeaters");
		break;
	case STL_CIB_STD_TXTECH_CU_LINACTEQ:
		strcpy(text_out, "Linear active copper cable");
		break;
	default:
		strcpy(text_out, "Undefined");
		break;
	}	// End of switch (code_xmit)

	return;

}	// End of StlCableInfoCableTypeToTextLong()

static __inline
uint32_t StlCableInfoSMFLength(uint8_t code_len)
{
	return code_len;

}	// End of StlCableInfoSMFLength()

static __inline
uint32_t StlCableInfoOM1Length(uint8_t code_len)
{
	return code_len;

}	// End of StlCableInfoOM1Length()

static __inline
uint32_t StlCableInfoOM2Length(uint8_t code_len)
{
	return code_len;

}	// End of StlCableInfoOM2Length()

static __inline
uint32_t StlCableInfoOM3Length(uint8_t code_len)
{
	return code_len * 2;

}	// End of StlCableInfoOM3Length()

static __inline
uint32_t StlCableInfoOM4Length(uint8_t code_len, uint8_t code_valid)
{
	if (code_valid)
		return code_len;
	else
		return code_len * 2;

}	// End of StlCableInfoOM4Length()

static __inline
void StlCableInfoOM4LengthToText(uint8_t code_len, uint8_t code_valid, int max_chars, char *text_out)
{
	if (! text_out)
		return;
	snprintf(text_out, max_chars, "%um", StlCableInfoOM4Length(code_len,code_valid));
}
#if 0
// This macro is based on stl_sma.c/GET_LENGTH() but contains invalid logic
//  for current CableInfo configurations 
static __inline
void StlCableInfoCableLengthToText(uint8_t code_smf, uint8_t code_om1, uint8_t code_om2, uint8_t code_om3, uint8_t code_om4, char *text_out)
{
	if (! text_out)
		return;
	if (code_smf)
		sprintf(text_out, "%3ukm", StlCableInfoSMFLength(code_smf));
	else if (code_om3)
		sprintf(text_out, "%3um", StlCableInfoOM3Length(code_om3));
	else if (code_om2)
		sprintf(text_out, "%3um", StlCableInfoOM2Length(code_om2));
	else if (code_om1)
		sprintf(text_out, "%3um", StlCableInfoOM1Length(code_om1));
	else if (code_om4)
		sprintf(text_out, "%3um", StlCableInfoOM4Length(code_om4));
	else
		strcpy(text_out, "");
	return;

}	// End of StlCableInfoCableLengthToText()
#endif

static __inline
void StlCableInfoValidCableLengthToText(uint8_t code_len, uint8_t code_valid, char *text_out)
{
	if (! text_out)
		return;
	if (code_valid)
		sprintf(text_out, "%3um", code_len);
	else
		strcpy(text_out, "");
	return;

}	// End of StlCableInfoValidCableLengthToText()

static __inline
void StlCableInfoDateCodeToText(uint8_t * code_date, char *text_out)
{
	if (! text_out)
		return;
	// Format date code as: 20YY/MM/DD LL (lot)
	strcpy(text_out, "20xx/xx/xx-xx");
	text_out[2] = code_date[0];
	text_out[3] = code_date[1];
	text_out[5] = code_date[2];
	text_out[6] = code_date[3];
	text_out[8] = code_date[4];
	text_out[9] = code_date[5];
	text_out[11] = code_date[6];
	text_out[12] = code_date[7];
	text_out[13] = '\0';
	return;

}	// End of StlCableInfoOutputDateCodeToText()

static __inline
const char * StlCableInfoCDRToText(uint8_t code_supp, uint8_t code_ctrl)
{
	if (! code_supp)
		return "N/A";
	else if (code_ctrl)
		return "On ";
	else
		return "Off";

}	// End of StlCableInfoCDRToText()

static __inline const char*
StlPortTypeToText(uint8_t code)
{
	switch(code) {
		case STL_PORT_TYPE_DISCONNECTED:
			return "Disconnected";
		case STL_PORT_TYPE_STANDARD:
			return "Standard";
		case STL_PORT_TYPE_FIXED:
			return "Fixed";
		case STL_PORT_TYPE_VARIABLE:
			return "Variable";
		case STL_PORT_TYPE_SI_PHOTONICS:
			return "Silicon Photonics";
		default:
			return "Unknown";
	}
}

static __inline uint32 StlLargePktLimitToBytes(uint8 largepktlimit)
{
	return (512 + (uint32)largepktlimit*512);
}

static __inline uint8 StlBytesToLargePktLimit(uint32 bytes)
{
	bytes /= 512;
	return bytes?bytes-1:0;	// <=512 returned as 0
}

static __inline uint32 StlSmallPktLimitToBytes(uint8 smallpktlimit)
{
	return (32 + (uint32)smallpktlimit*32);
}

static __inline uint8 StlBytesToSmallPktLimit(uint32 bytes)
{
	bytes /= 32;
	return bytes?bytes-1:0;	// <=32 returned as 0
}

static __inline uint32 StlPreemptionLimitToBytes(uint8 limit)
{
	// no ideal return for NONE, so use largest uint8 (can't fo uint32 as it would overflow buf[6]
	if (limit == STL_PORT_PREEMPTION_LIMIT_NONE) return IB_UINT8_MAX;
	return ((uint32)limit*256);
}

static __inline uint8 StlBytesToPreemptionLimit(uint32 bytes)
{
	bytes /= 256;
	return (bytes>=0xff)?STL_PORT_PREEMPTION_LIMIT_NONE:bytes;
}

static __inline
void FormatStlPreemptionLimit(char buf[6], uint8 limit)
{
	if (limit == STL_PORT_PREEMPTION_LIMIT_NONE)
		sprintf(buf, "%5s", "None");
	else
		snprintf(buf, 6, "%5u", (unsigned int)StlPreemptionLimitToBytes(limit));
}

/* convert capability mask into a text representation */
static __inline
void FormatStlCapabilityMask(char buf[80], STL_CAPABILITY_MASK cmask)
{
	sprintf(buf, "%s%s%s%s%s%s%s",
		cmask.s.IsCapabilityMaskNoticeSupported?"CN ": "",
		cmask.s.IsVendorClassSupported?"VDR ": "",
		cmask.s.IsDeviceManagementSupported?"DM ": "",
		cmask.s.IsConnectionManagementSupported?"CM ": "",
		cmask.s.IsAutomaticMigrationSupported?"APM ": "",
		cmask.s.IsSM?"SM ": "",
		cmask.AsReg32?"": "-");
}

/* convert Node Type to text */
static __inline const char*
StlNodeTypeToText(NODE_TYPE type)
{
	return (type == STL_NODE_FI)?"FI":
		(type == STL_NODE_SW)?"SW": "??";
}

static __inline const char*
StlLinkQualToText(uint8 linkQual)
{
	switch (linkQual)
	{
		case STL_LINKQUALITY_NONE:
			return "Down";
		case STL_LINKQUALITY_BAD:
			return "Bad";
		case STL_LINKQUALITY_POOR:
			return "Poor";
		case STL_LINKQUALITY_GOOD:
			return "Good";
		case STL_LINKQUALITY_VERY_GOOD:
			return "VeryGood";
		case STL_LINKQUALITY_EXCELLENT:
			return "Excellent";
		default:
			return "???";
	}
}

/* convert Neighbor node Type to text */
static __inline const char*
OpaNeighborNodeTypeToText(uint8 ntype)
{
	return (ntype == STL_NEIGH_NODE_TYPE_HFI) ? "HFI":
		(ntype == STL_NEIGH_NODE_TYPE_SW) ? "Switch" : "Unknown";
}

static __inline int
IsCableInfoAvailable(STL_PORT_INFO *portInfo)
{
	return (portInfo->PortPhyConfig.s.PortType == STL_PORT_TYPE_STANDARD
			&& ! (portInfo->PortStates.s.PortPhysicalState == STL_PORT_PHYS_OFFLINE
				&& portInfo->PortStates.s.OfflineDisabledReason == STL_OFFDIS_REASON_LOCAL_MEDIA_NOT_INSTALLED));
}

static __inline uint8
StlResolutionToShift(uint32 res, uint8 add) {
// shift = log2(res) - add
	uint8 shift = FloorLog2(res);
	if (shift > 15) return 15; // 15 is the maximum allowed value
	else if (shift > add) return shift-add;
	else return 0;
}

static __inline uint32
StlShiftToResolution(uint8 shift, uint8 add) {
// res = 2^(shift + add)
	if (shift) return (uint32)1<<(shift + add);
	else return 0;
}

static __inline void
FormatStlCounterSelectMask(char buf[128], CounterSelectMask_t mask) {

	sprintf(buf, "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
		(mask.s.PortXmitData                ? "TxD ": ""),
		(mask.s.PortRcvData                 ? "RxD ": ""),
		(mask.s.PortXmitPkts                ? "TxP ": ""),
		(mask.s.PortRcvPkts                 ? "RxP ": ""),
		(mask.s.PortMulticastXmitPkts       ? "MTxP ": ""),
		(mask.s.PortMulticastRcvPkts        ? "MRxP ": ""),

		(mask.s.PortXmitWait                ? "TxW ": ""),
		(mask.s.SwPortCongestion            ? "CD ": ""), /* CongDiscards */
		(mask.s.PortRcvFECN                 ? "RxF ": ""),
		(mask.s.PortRcvBECN                 ? "RxB ": ""),
		(mask.s.PortXmitTimeCong            ? "TxTC ": ""),
		(mask.s.PortXmitWastedBW            ? "WBW ": ""),
		(mask.s.PortXmitWaitData            ? "TxWD ": ""),
		(mask.s.PortRcvBubble               ? "RxBb ": ""),
		(mask.s.PortMarkFECN                ? "MkF ": ""),
		(mask.s.PortRcvConstraintErrors     ? "RxCE ": ""),
		(mask.s.PortRcvSwitchRelayErrors    ? "RxSR ": ""),
		(mask.s.PortXmitDiscards            ? "TxDc ": ""),
		(mask.s.PortXmitConstraintErrors    ? "TxCE ": ""),
		(mask.s.PortRcvRemotePhysicalErrors ? "RxRP ": ""),
		(mask.s.LocalLinkIntegrityErrors    ? "LLI ": ""),
		(mask.s.PortRcvErrors               ? "RxE ": ""),
		(mask.s.ExcessiveBufferOverruns     ? "EBO ": ""),
		(mask.s.FMConfigErrors              ? "FMC ": ""),
		(mask.s.LinkErrorRecovery           ? "LER ": ""),
		(mask.s.LinkDowned                  ? "LD ": ""),
		(mask.s.UncorrectableErrors         ? "Unc": ""));
}

#if !defined(ROUNDUP)
#define ROUNDUP(val, align) ((((uintn)(val)+(uintn)(align)-1)/((uintn)align))*((uintn)(align)))
#endif

#if !defined(ROUNDUP_TYPE)
#define ROUNDUP_TYPE(type, val, align) ((((val)-1) | ((type)((align)-1)))+1)
#endif

/* Simplify accessing PORT(i%MAX_PORTS) data from a Block(i/MAX_PORTS) */
#define STL_LFT_PORT_BLOCK(LftTable, i) \
    LftTable[i/MAX_LFT_ELEMENTS_BLOCK].LftBlock[i%MAX_LFT_ELEMENTS_BLOCK]
#define STL_PGFT_PORT_BLOCK(PgftTable, i) \
    PgftTable[i/NUM_PGFT_ELEMENTS_BLOCK].PgftBlock[i%NUM_PGFT_ELEMENTS_BLOCK]

#ifdef __cplusplus
};
#endif

#endif	/* _IBA_STL_HELPER_H_ */
