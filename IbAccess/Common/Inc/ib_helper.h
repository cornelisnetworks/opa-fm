/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_IB_HELPER_H_
#define _IBA_IB_HELPER_H_

#include "iba/stl_types.h"
#include "iba/stl_sm_types.h"

#include "iba/ib_mad.h"
#include "iba/public/imath.h"

#ifdef __cplusplus
extern "C" {
#endif

/* these helper functions are macros available only within ib_helper.h
 *  to protect them from misuse; they are undefined outside of ib_helper.h
 */

/* convert mbps (10^6 bits/sec) to MBps (2^20 bytes/sec)
 *  speeds through QDR use 8b10b wire encoding ( * 1 / 10 )
 */
#define IbmbpsToMBps(mbps) {\
	((uint32)((((uint64)mbps / 10LL) * 1000000LL) / ( 1LL << 20))); \
}

/* convert mbps (10^6 bits/sec) to MBps (2^20 bytes/sec) for extended speeds
 *  extended speeds use 64b66b wire encoding ( * 8 / 66 )
 */
#define IbmbpsToMBpsExt(mbps) {\
	((uint32)(((uint64)mbps * 8LL * 1000000LL) / (66LL * (1LL << 20)))); \
}

/* these helper functions are simple, so they are done as inlines
 * largely to avoid the need to export all these symbols from a
 * kernel module or a library
 */

/*
 * Convert a IB_PORT_STATE to a constant string
 */
static __inline const char*
IbPortStateToText( IN IB_PORT_STATE state )
{
	switch (state)
	{
		case IB_PORT_NOP:
			return "Noop";
		case IB_PORT_DOWN:
			return "Down";
		case IB_PORT_INIT: 
			return "Init";
		case IB_PORT_ARMED: 
			return "Armed";
		case IB_PORT_ACTIVE: 
			return "Active";
		default:
			return "????";
	}
}

/*
 * Convert a IB_PORT_PHYS_STATE to a constant string
 */
static __inline const char*
IbPortPhysStateToText( IN IB_PORT_PHYS_STATE state )
{
	switch (state)
	{
		case IB_PORT_PHYS_NOP:
			return "Noop";
		case IB_PORT_PHYS_SLEEP:
			return "Sleep";
		case IB_PORT_PHYS_POLLING:
			return "Polling";
		case IB_PORT_PHYS_DISABLED:
			return "Disabled";
		case IB_PORT_PHYS_TRAINING:
			return "Training";
		case IB_PORT_PHYS_LINKUP:
			return "LinkUp";
		case IB_PORT_PHYS_LINK_ERROR_RECOVERY:
			return "Recovery";
		default:
			return "????";
	}
}

/*
 * Convert a IB_PORT_PHYS_STATE for DownDefault to a constant string
 */
static __inline const char*
IbPortDownDefaultToText( IN IB_PORT_PHYS_STATE state )
{
	switch (state)
	{
		case IB_PORT_PHYS_NOP:
			return "Noop";
		case IB_PORT_PHYS_SLEEP:
			return "Sleep";
		case IB_PORT_PHYS_POLLING:
			return "Polling";
		default:
			return "????";
	}
}

/*
 * Convert IB_MTU enumeration to a byte count
 */
static __inline uint16
GetBytesFromMtu(uint8 mtu)
{
	switch (mtu)
	{
		default:
			return 0;
		case IB_MTU_256:
			return 256;
		case IB_MTU_512:
			return 512;
		case IB_MTU_1024:
			return 1024;
		case IB_MTU_2048:
			return 2048;
		case IB_MTU_4096:
			return 4096;
		case STL_MTU_8192:
			return 8192;
		case STL_MTU_10240:
			return 10240;
	}
}
/* Convert byte count to nearest IB_MTU enumeration (rounds up)
 * (eg. returns IB MTU required to support packet sizes of "bytes")
 */
static __inline uint8
GetMtuFromBytes(uint16 bytes)
{
	if (bytes <= 256)
		return IB_MTU_256;
	else if (bytes <= 512)
		return IB_MTU_512;
	else if (bytes <= 1024)
		return IB_MTU_1024;
	else if (bytes <= 2048)
		return IB_MTU_2048;
	else if (bytes <= 4096)
		return IB_MTU_4096;
	else if (bytes <= 8192)
		return STL_MTU_8192;
	else if (bytes <= 10240)
		return STL_MTU_10240;
	else 
		return IB_MTU_4096;
}

/* Convert IB_MTU enumeration to text */
static __inline const char*
IbMTUToText(IB_MTU mtu)
{
	switch ((int)mtu)
	{
		case STL_MTU_0:
		default:
			return "0";
		case IB_MTU_256:
			return "256";
		case IB_MTU_512:
			return "512";
		case IB_MTU_1024:
			return "1024";
		case IB_MTU_2048:
			return "2048";
		case IB_MTU_4096:
			return "4096";
        case STL_MTU_8192:
            return "8192";
        case STL_MTU_10240:
	        return "10240";
	}
}

/* Convert static rate to text */
static inline const char*
IbStaticRateToText(IB_STATIC_RATE rate)
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
			return "40g";
		case IB_STATIC_RATE_60G:
			return "60g";
		case IB_STATIC_RATE_80G:
			return "80g";
		case IB_STATIC_RATE_120G:
			return "120g";
		case IB_STATIC_RATE_14G:
			return "14g";				// 14.0625g
		case IB_STATIC_RATE_25G:
			return "25g";				// 25.78125g
		case IB_STATIC_RATE_56G:
			return "56g";				// 56.25g
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

/* convert static rate to mbps (10^6 bits/sec) units */
static __inline uint32
IbStaticRateToMbps(IB_STATIC_RATE rate)
{
	switch (rate)
	{
		default:
		case IB_STATIC_RATE_DONTCARE:
			return 10000;	/* default to 10G */
							/* In other functions this is present wire speed */
		case IB_STATIC_RATE_1GB:
			return 1000;
		case IB_STATIC_RATE_2_5G:
			return 2500;
		case IB_STATIC_RATE_10G:
			return 10000;
		case IB_STATIC_RATE_30G:
			return 30000;
		case IB_STATIC_RATE_5G:
			return 5000;
		case IB_STATIC_RATE_20G:
			return 20000;
		case IB_STATIC_RATE_40G:
			return 40000;
		case IB_STATIC_RATE_60G:
			return 60000;
		case IB_STATIC_RATE_80G:
			return 80000;
		case IB_STATIC_RATE_120G:
			return 120000;
		case IB_STATIC_RATE_14G:
			return 14062;
		case IB_STATIC_RATE_25G:
			return 25781;
		case IB_STATIC_RATE_56G:
			return 56250;
		case IB_STATIC_RATE_100G:
			return 103125;
		case IB_STATIC_RATE_112G:
			return 112500;
		case IB_STATIC_RATE_200G:
			return 206250;
		case IB_STATIC_RATE_168G:
			return 168750;
		case IB_STATIC_RATE_300G:
			return 309375;
	}
}

/* convert static rate to PMA million ticks/sec units
 * tick = time to send 1 byte on wire
 */
static __inline uint32
IbStaticRateToMTickps(IB_STATIC_RATE rate)
{
	if (rate <= IB_STATIC_RATE_LAST_QDR)
		return (IbStaticRateToMbps(rate)/10);		// 8b10b encoding
	else
		return ((IbStaticRateToMbps(rate)*8)/66);	// 64b66b encoding
}

/* convert static rate to MByte/sec units, M=2^20 */
static __inline uint32
IbStaticRateToMBps(IB_STATIC_RATE rate)
{
	if (rate <= IB_STATIC_RATE_LAST_QDR)
		return (IbmbpsToMBps(IbStaticRateToMbps(rate)));
	else
		return (IbmbpsToMBpsExt(IbStaticRateToMbps(rate)));
}

/* convert mbps (10^6 bits/sec) to static rate */
static __inline IB_STATIC_RATE
IbMbpsToStaticRate(uint32 rate_mbps)
{
	/* 1Gb rate is obsolete */
	if (rate_mbps <= 2500)
		return IB_STATIC_RATE_2_5G;
	else if (rate_mbps <= 5000)
		return IB_STATIC_RATE_5G;
	else if (rate_mbps <= 10000)
		return IB_STATIC_RATE_10G;
	else if (rate_mbps <= 14062)
		return IB_STATIC_RATE_14G;
	else if (rate_mbps <= 20000)
		return IB_STATIC_RATE_20G;
	else if (rate_mbps <= 25781)
		return IB_STATIC_RATE_25G;
	else if (rate_mbps <= 30000)
		return IB_STATIC_RATE_30G;
	else if (rate_mbps <= 40000)
		return IB_STATIC_RATE_40G;
	else if (rate_mbps <= 56250)
		return IB_STATIC_RATE_56G;
	else if (rate_mbps <= 60000)
		return IB_STATIC_RATE_60G;
	else if (rate_mbps <= 80000)
		return IB_STATIC_RATE_80G;
	else if (rate_mbps <= 103125)
		return IB_STATIC_RATE_100G;
	else if (rate_mbps <= 112500)
		return IB_STATIC_RATE_112G;
	else if (rate_mbps <= 120000)
		return IB_STATIC_RATE_120G;
	else if (rate_mbps <= 168750)
		return IB_STATIC_RATE_168G;
	else if (rate_mbps <= 206250)
		return IB_STATIC_RATE_200G;
	else 
		return IB_STATIC_RATE_300G;
}

/* convert link width to text
 * 7 char output for Supported/Enabled Width (multi rate)
 * 4 char output for Active Width (single rate or Noop)
 */
static __inline const char*
IbLinkWidthToText(IB_LINK_WIDTH width)
{
	return (width == IB_LINK_WIDTH_NOP)?"None":
		(width == IB_LINK_WIDTH_1X)?"1x":
		(width == IB_LINK_WIDTH_4X)?"4x":
		(width == IB_LINK_WIDTH_8X)?"8x":
		(width == IB_LINK_WIDTH_12X)?"12x":
		(width == IB_LINK_WIDTH_1X+IB_LINK_WIDTH_4X)?"1-4x":
		(width == IB_LINK_WIDTH_1X+IB_LINK_WIDTH_8X)?"1,8x":
		(width == IB_LINK_WIDTH_1X+IB_LINK_WIDTH_12X)?"1,12x":
		(width == IB_LINK_WIDTH_1X+IB_LINK_WIDTH_4X+IB_LINK_WIDTH_8X)?"1-8x":
		(width == IB_LINK_WIDTH_1X+IB_LINK_WIDTH_4X+IB_LINK_WIDTH_12X)?"1,4,12x":
		(width == IB_LINK_WIDTH_1X+IB_LINK_WIDTH_8X+IB_LINK_WIDTH_12X)?"1,8,12x":
		(width == IB_LINK_WIDTH_1X+IB_LINK_WIDTH_4X+IB_LINK_WIDTH_8X+IB_LINK_WIDTH_12X)?"1-12x":
		(width == IB_LINK_WIDTH_4X+IB_LINK_WIDTH_8X)?"4-8x":
		(width == IB_LINK_WIDTH_4X+IB_LINK_WIDTH_12X)?"4,12x":
		(width == IB_LINK_WIDTH_4X+IB_LINK_WIDTH_8X+IB_LINK_WIDTH_12X)?"4-12x":
		(width == IB_LINK_WIDTH_8X+IB_LINK_WIDTH_12X)?"8-12x":
		(width == IB_LINK_WIDTH_ALL_SUPPORTED)?"all":
		"???";
}

/* determine if link width is a valid value for active link width */
/* Active Link width can only have 1 bit set, Note that Enabled and Supported
 * can have multiple bits set
 */
static __inline boolean
IbLinkWidthIsValidActive(IB_LINK_WIDTH width)
{
	switch (width) {
	default:
		return FALSE;
	case IB_LINK_WIDTH_1X:
	case IB_LINK_WIDTH_4X:
	case IB_LINK_WIDTH_8X:
	case IB_LINK_WIDTH_12X:
		return TRUE;
	}
}

/* convert link width to integer multiplier */
static __inline uint32
IbLinkWidthToInt(IB_LINK_WIDTH width)
{
	switch (width) {
	default:
	case IB_LINK_WIDTH_1X: return 1;
	case IB_LINK_WIDTH_4X: return 4;
	case IB_LINK_WIDTH_8X: return 8;
	case IB_LINK_WIDTH_12X: return 12;
	}
}

/* convert integer multiplier to link width */
static __inline IB_LINK_WIDTH
IbIntToLinkWidth(uint32 width)
{
	if (width <= 1)
		return IB_LINK_WIDTH_1X;
	else if (width <= 4)
		return IB_LINK_WIDTH_4X;
	else if (width <= 8)
		return IB_LINK_WIDTH_8X;
	else 
		return IB_LINK_WIDTH_12X;
}

/* compare 2 link widths and report expected operational width
 * can be used to compare Enabled or Supported values in connected ports
 */
static __inline IB_LINK_WIDTH
IbExpectedLinkWidth(IB_LINK_WIDTH a, IB_LINK_WIDTH b)
{
	if ((IB_LINK_WIDTH_12X & a) && (IB_LINK_WIDTH_12X & b))
		return IB_LINK_WIDTH_12X;
	else if ((IB_LINK_WIDTH_8X & a) && (IB_LINK_WIDTH_8X & b))
		return IB_LINK_WIDTH_8X;
	else if ((IB_LINK_WIDTH_4X & a) && (IB_LINK_WIDTH_4X & b))
		return IB_LINK_WIDTH_4X;
	else if ((IB_LINK_WIDTH_1X & a) && (IB_LINK_WIDTH_1X & b))
		return IB_LINK_WIDTH_1X;
	else
		return (IB_LINK_WIDTH)0;	/* link should come up */
}

/* return the best link width set in the bit mask */
static __inline IB_LINK_WIDTH
IbBestLinkWidth(IB_LINK_WIDTH width)
{
	if (IB_LINK_WIDTH_12X & width)
		return IB_LINK_WIDTH_12X;
	else if (IB_LINK_WIDTH_8X & width)
		return IB_LINK_WIDTH_8X;
	else if (IB_LINK_WIDTH_4X & width)
		return IB_LINK_WIDTH_4X;
	else if (IB_LINK_WIDTH_1X & width)
		return IB_LINK_WIDTH_1X;
	else
		return (IB_LINK_WIDTH)0;	/* no width? */
}

/* convert link speed to 12 character text (most are <= 10) */
static __inline const char*
IbLinkSpeedToText(IB_LINK_SPEED speed)
{
	return (speed == IB_LINK_SPEED_NOP)?"None":
			(speed == IB_LINK_SPEED_2_5G)?"2.5Gb":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_5G)?"2.5-5Gb":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_10G)?"2.5,10Gb":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_5G+IB_LINK_SPEED_10G)?"2.5-10Gb":
			(speed == IB_LINK_SPEED_5G)?"5.0Gb":
			(speed == IB_LINK_SPEED_5G+IB_LINK_SPEED_10G)?"5-10Gb":
			(speed == IB_LINK_SPEED_10G)?"10.0Gb":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_14G)?"2.5,14Gb":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_5G+IB_LINK_SPEED_14G)?"2.5-5,14Gb":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_5G+IB_LINK_SPEED_10G+IB_LINK_SPEED_14G)?"2.5-14Gb":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_10G+IB_LINK_SPEED_14G)?"2.5,10-14G":
			(speed == IB_LINK_SPEED_5G+IB_LINK_SPEED_10G+IB_LINK_SPEED_14G)?"5-14Gb":
			(speed == IB_LINK_SPEED_5G+IB_LINK_SPEED_14G)?"5,14Gb":
			(speed == IB_LINK_SPEED_10G+IB_LINK_SPEED_14G)?"10-14Gb":
			(speed == IB_LINK_SPEED_14G)?"14.0Gb":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_25G)?"2.5,25Gb":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_5G+IB_LINK_SPEED_25G)?"2.5-5,25Gb":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_5G+IB_LINK_SPEED_10G+IB_LINK_SPEED_25G)?"2.5-10,25G":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_5G+IB_LINK_SPEED_10G+IB_LINK_SPEED_14G+IB_LINK_SPEED_25G)?"2.5-25Gb":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_5G+IB_LINK_SPEED_14G+IB_LINK_SPEED_25G)?"2.5-5,14-25G":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_10G+IB_LINK_SPEED_14G+IB_LINK_SPEED_25G)?"2.5,10-25G":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_14G+IB_LINK_SPEED_25G)?"2.5,14-25G":
			(speed == IB_LINK_SPEED_2_5G+IB_LINK_SPEED_10G+IB_LINK_SPEED_25G)?"2.5,10,25G":
			(speed == IB_LINK_SPEED_5G+IB_LINK_SPEED_25G)?"5.0,25Gb":
			(speed == IB_LINK_SPEED_5G+IB_LINK_SPEED_10G+IB_LINK_SPEED_25G)?"5-10,25Gb":
			(speed == IB_LINK_SPEED_5G+IB_LINK_SPEED_10G+IB_LINK_SPEED_14G+IB_LINK_SPEED_25G)?"5-25Gb":
			(speed == IB_LINK_SPEED_5G+IB_LINK_SPEED_14G+IB_LINK_SPEED_25G)?"5,14-25Gb":
			(speed == IB_LINK_SPEED_10G+IB_LINK_SPEED_14G+IB_LINK_SPEED_25G)?"10-25Gb":
			(speed == IB_LINK_SPEED_14G+IB_LINK_SPEED_25G)?"14-25Gb":
			(speed == IB_LINK_SPEED_10G+IB_LINK_SPEED_25G)?"10,25Gb":
			(speed == IB_LINK_SPEED_25G)?"25.7Gb":
			(speed == IB_LINK_SPEED_ALL_SUPPORTED)?"ALL":
			"????";
}

/* determine if link speed is a valid value for active link speed */
/* Active Link speed can only have 1 bit set, Note that Enabled and Supported
 * can have multiple bits set
 */
static __inline boolean
IbLinkSpeedIsValidActive(IB_LINK_SPEED speed)
{
	switch (speed) {
	default:
		return FALSE;
	case IB_LINK_SPEED_2_5G:
	case IB_LINK_SPEED_5G:
	case IB_LINK_SPEED_10G:
	case IB_LINK_SPEED_14G:
	case IB_LINK_SPEED_25G:
		return TRUE;
	}
}

/* convert link speed to mbps (10^6 bits/sec) units */
static __inline uint32
IbLinkSpeedToMbps(IB_LINK_SPEED speed)
{
	switch (speed) {
	default:
	case IB_LINK_SPEED_2_5G: return 2500;
	case IB_LINK_SPEED_5G: return 5000;
	case IB_LINK_SPEED_10G: return 10000;
	case IB_LINK_SPEED_14G: return 14062;
	case IB_LINK_SPEED_25G: return 25781;
	}
}

/* convert mbps (10^6 bits/sec) to link speed */
static __inline IB_LINK_SPEED
IbMbpsToLinkSpeed(uint32 speed)
{
	if (speed <= 2500)
		return IB_LINK_SPEED_2_5G;
	else if (speed <= 5000)
		return IB_LINK_SPEED_5G;
	else if (speed <= 10000)
		return IB_LINK_SPEED_10G;
	else if (speed <= 14062)
		return IB_LINK_SPEED_14G;
	else
		return IB_LINK_SPEED_25G;
}

/* compare 2 link speeds and report expected operational speed
 * can be used to compare Enabled or Supported values in connected ports
 */
static __inline IB_LINK_SPEED
IbExpectedLinkSpeed(IB_LINK_SPEED a, IB_LINK_SPEED b)
{
	if ((IB_LINK_SPEED_25G & a) && (IB_LINK_SPEED_25G & b))
		return IB_LINK_SPEED_25G;
	else if ((IB_LINK_SPEED_14G & a) && (IB_LINK_SPEED_14G & b))
		return IB_LINK_SPEED_14G;
	else if ((IB_LINK_SPEED_10G & a) && (IB_LINK_SPEED_10G & b))
		return IB_LINK_SPEED_10G;
	else if ((IB_LINK_SPEED_5G & a) && (IB_LINK_SPEED_5G & b))
		return IB_LINK_SPEED_5G;
	else if ((IB_LINK_SPEED_2_5G & a) && (IB_LINK_SPEED_2_5G & b))
		return IB_LINK_SPEED_2_5G;
	else
		return (IB_LINK_SPEED)0;	/* link should not come up */
}

/* return the best link speed set in the bit mask */
static __inline IB_LINK_SPEED
IbBestLinkSpeed(IB_LINK_SPEED speed)
{
	if (IB_LINK_SPEED_25G & speed)
		return IB_LINK_SPEED_25G;
	else if (IB_LINK_SPEED_14G & speed)
		return IB_LINK_SPEED_14G;
	else if (IB_LINK_SPEED_10G & speed)
		return IB_LINK_SPEED_10G;
	else if (IB_LINK_SPEED_5G & speed)
		return IB_LINK_SPEED_5G;
	else if (IB_LINK_SPEED_2_5G & speed)
		return IB_LINK_SPEED_2_5G;
	else
		return (IB_LINK_SPEED)0;	/* no speed? */
}

/* convert link speed and width to mbps (10^6 bits/sec) */
static __inline uint32
IbLinkSpeedWidthToMbps(IB_LINK_SPEED speed, IB_LINK_WIDTH width)
{
	return IbLinkSpeedToMbps(speed) * IbLinkWidthToInt(width);
}

/* convert link speed and width to Static Rate */
static __inline IB_STATIC_RATE
IbLinkSpeedWidthToStaticRate(IB_LINK_SPEED speed, IB_LINK_WIDTH width)
{
	return IbMbpsToStaticRate(IbLinkSpeedWidthToMbps(speed, width));
}

/* convert static rate to InterPacketDelay (IPD) */
static __inline uint8
IbStaticRateToIpd(IB_STATIC_RATE rate, IB_LINK_WIDTH linkWidth, IB_LINK_SPEED linkSpeed)
{
	uint32 link_rate_mbps;
	uint32 static_rate_mbps;

	if (rate == IB_STATIC_RATE_DONTCARE)
		return 0;	/* 100% speed */
	link_rate_mbps = IbLinkWidthToInt(linkWidth)*IbLinkSpeedToMbps(linkSpeed);
	static_rate_mbps = IbStaticRateToMbps(rate);

	if (static_rate_mbps >= link_rate_mbps)
		return 0;	/* 100% speed */
	/* round up to handle odd ones like 8x rate over 12x link */
	return ((link_rate_mbps  + (static_rate_mbps-1))/ static_rate_mbps) - 1;
}

/* convert InterPacketDelay (IPD) to static rate */
static __inline IB_STATIC_RATE
IbIpdToStaticRate(uint8 ipd, IB_LINK_WIDTH linkWidth, IB_LINK_SPEED linkSpeed)
{
	uint32 link_rate_mbps = IbLinkWidthToInt(linkWidth)*IbLinkSpeedToMbps(linkSpeed);
	return IbMbpsToStaticRate(link_rate_mbps/(ipd+1));
}

/* convert IB_PORT_ATTRIBUTES.ActiveWidth to IB_LINK_WIDTH */
static __inline IB_LINK_WIDTH
IbPortWidthToLinkWidth(uint8 width)
{
	switch (width) {
	case PORT_WIDTH_1X:
		return IB_LINK_WIDTH_1X;
	default:
	case PORT_WIDTH_4X:
		return IB_LINK_WIDTH_4X;
	case PORT_WIDTH_8X:
		return IB_LINK_WIDTH_8X;
	case PORT_WIDTH_12X:
		return IB_LINK_WIDTH_12X;
	}
}

/* convert IB_PORT_ATTRIBUTES.ActiveSpeed to IB_LINK_SPEED */
static __inline IB_LINK_SPEED
IbPortSpeedToLinkSpeed(uint8 speed)
{
	switch (speed) {
	default:
	case PORT_SPEED_2_5G:
		return IB_LINK_SPEED_2_5G;
	case PORT_SPEED_5G:
		return IB_LINK_SPEED_5G;
	case PORT_SPEED_10G:
		return IB_LINK_SPEED_10G;
	case PORT_SPEED_14G:
		return IB_LINK_SPEED_14G;
	case PORT_SPEED_25G:
		return IB_LINK_SPEED_25G;
	}
}

/* convert IB_PORT_ATTRIBUTES.ActiveSpeed and ActiveWidth to Mbps */
static __inline uint32
IbPortSpeedWidthToMbps(uint8 speed, uint8 width)
{
	return IbLinkSpeedToMbps(IbPortSpeedToLinkSpeed(speed))
			* IbLinkWidthToInt(IbPortWidthToLinkWidth(width));
}

/* convert IB_PORT_ATTRIBUTES.ActiveSpeed and ActiveWidth to Static Rate */
static __inline IB_STATIC_RATE
IbPortSpeedWidthToStaticRate(uint8 speed, uint8 width)
{
	return IbMbpsToStaticRate(IbPortSpeedWidthToMbps(speed, width));
}

// combine LinkSpeedActive and LinkSpeedExtActive
static __inline uint8
PortDataGetLinkSpeedActiveCombined(PORT_INFO *portInfop)
{
	uint8 speed = portInfop->LinkSpeed.Active & 0x0F;
	if (portInfop->CapabilityMask.s.IsExtendedSpeedsSupported)
		speed |= (portInfop->LinkSpeedExt.Active << 4);
	return(speed);
}

// split LinkSpeedActive into LinkSpeedActive and LinkSpeedExtActive
static __inline void
PortDataSetLinkSpeedActiveSplit(uint8 speed, PORT_INFO *portInfop)
{
	portInfop->LinkSpeed.Active = speed & 0x0F;
	portInfop->LinkSpeedExt.Active = speed >> 4;
}

// combine LinkSpeedSupported and LinkSpeedExtSupported
static __inline uint8
PortDataGetLinkSpeedSupportedCombined(PORT_INFO *portInfop)
{
	uint8 speed = portInfop->Link.SpeedSupported & 0x0F;
	if (portInfop->CapabilityMask.s.IsExtendedSpeedsSupported)
		speed |= (portInfop->LinkSpeedExt.Supported << 4);
	return(speed);
}

// split LinkSpeedSupported into LinkSpeedSupported and LinkSpeedExtSupported
static __inline void
PortDataSetLinkSpeedSupportedSplit(uint8 speed, PORT_INFO *portInfop)
{
	portInfop->Link.SpeedSupported = speed & 0x0F;
	portInfop->LinkSpeedExt.Supported = speed >> 4;
}

// combine LinkSpeedEnabled and LinkSpeedExtEnabled
static __inline uint8
PortDataGetLinkSpeedEnabledCombined(PORT_INFO *portInfop)
{
	uint8 speed = portInfop->LinkSpeed.Enabled & 0x0F;
	if (portInfop->CapabilityMask.s.IsExtendedSpeedsSupported)
		speed |= (portInfop->LinkSpeedExt.Enabled << 4);
	return(speed);
}

// split LinkSpeedEnabled into LinkSpeedEnabled and LinkSpeedExtEnabled
static __inline void
PortDataSetLinkSpeedEnabledSplit(uint8 speed, PORT_INFO *portInfop)
{
	portInfop->LinkSpeed.Enabled = speed & 0x0F;
	portInfop->LinkSpeedExt.Enabled = speed >> 4;
}

/* convert Notice Type to text */
static __inline const char*
IbNoticeTypeToText(IB_NOTICE_TYPE type)
{
	return (type == IB_NOTICE_FATAL? "Fatal":
			type == IB_NOTICE_URGENT? "Urgent":
			type == IB_NOTICE_SECURITY? "Security":
			type == IB_NOTICE_SM? "SM":
			type == IB_NOTICE_INFO? "Info":
			type == IB_NOTICE_ALL? "All": "???");
}

/* convert MKey protect selection to text */
static __inline const char*
IbMKeyProtectToText(IB_MKEY_PROTECT protect)
{
	return (protect == IB_MKEY_PROTECT_RO)?"Read-only":
		(protect == IB_MKEY_PROTECT_HIDE)?"Hide":
		(protect == IB_MKEY_PROTECT_SECURE)?"Secure":"????";
}

/* TBD - come back and do these a little more efficiently with a
 * global array indexed by timeout_mult
 */

#define IB_MAX_TIMEOUT_MULT_MS 8796093	/* 2.4 hr */

/* Convert Timeout value from 4.096usec * 2^timeout_mult
 * to ms (which is units for Timers in Public)
 */
static __inline
uint32 TimeoutMultToTimeInMs(uint32 timeout_mult)
{
	/* all values are rounded up, comments reflect exact value */
	switch (timeout_mult)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			return 1;	/* < 1 ms, round up */
		case 8:
			return 2;	/* 1.048 ms */
		case 9:
			return 3;	/* 2.097 ms */
		case 10:
			return 5;	/* 4.197 ms */
		case 11:
			return 9;	/* 8.388 ms */
		case 12:
			return 17;	/* 16.777 ms */
		case 13:
			return 34;	/* 33.554 ms */
		case 14:
			return 68;	/* 67.1 ms */
		case 15:
			return 135;	/* 134.2 ms */
		case 16:
			return 269;	/* 268.4 ms */
		case 17:
			return 537;	/* 536.8 ms */
		case 18:
			return 1074;/* 1.073 s */
		case 19:
			return 2148;/* 2.148 s */
		case 20:
			return 4295;/* 4.294 s */
		case 21:
			return 8590;/* 8.589 s */
		case 22:
			return 17180;/* 17.179 s */
		case 23:
			return 34359;/* 34.359 s */
		case 24:
			return 68720;/* 68.719 s */
		case 25:
			return 137438;/* 2.2 minutes */
		case 26:
			return 274877; /* 4.5 minutes */
		case 27:
			return 549755; /* 9 minutes */
		case 28:
			return 1099511;	/* 18 minutes */
		case 29:
			return 2199023;	/* 0.6 hr */
		case 30:
			return 4398046;	/* 1.2 hr	 */
		default:	/* PktLifeTime allows 6 bits in PathRecord */
					/* but anything beyond 2 hrs is an eternity */
		case 31:
			return 8796093;	/* 2.4 hr */
	}
}

/* Convert Timeout value from 4.096usec * 2^timeout_mult to usec */
static __inline
uint64 TimeoutMultToTimeInUsec(uint32 timeout_mult)
{
	/* all values are rounded up, comments reflect exact value */
	switch (timeout_mult)
	{
		case 0:
			return 5;	/* 4.096 usec */
		case 1:
			return 9;	/* 8.192 usec */
		case 2:
			return 17;	/* 16.384 usec */
		case 3:
			return 33;	/* 32.768 usec */
		case 4:
			return 66;	/* 65.536 usec */
		case 5:
			return 132;	/* 131.072 usec */
		case 6:
			return 263;	/* 262.144 usec */
		case 7:
			return 525;	/* 524.288 usec */
		case 8:
			return 1048;	/* 1.048 ms */
		case 9:
			return 2097;	/* 2.097 ms */
		case 10:
			return 4197;	/* 4.197 ms */
		case 11:
			return 8388;	/* 8.388 ms */
		default:
			/* ms accuracy should be good enough for the rest */
			return ((uint64)TimeoutMultToTimeInMs(timeout_mult)*(uint64)1000);
	}
}

/* convert capability mask into a text representation */
static __inline
void FormatCapabilityMask(char buf[80], IB_CAPABILITY_MASK cmask)
{
	snprintf(buf, 80, "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
		cmask.s.IsClientReregistrationSupported?"CR ": "",
		cmask.s.IsLinkRoundTripLatencySupported?"LR ": "",
		cmask.s.IsBootManagementSupported?"BT ": "",
		cmask.s.IsCapabilityMaskNoticeSupported?"CN ": "",
		cmask.s.IsDRNoticeSupported?"DN ": "",
		cmask.s.IsVendorClassSupported?"VDR ": "",
		cmask.s.IsDeviceManagementSupported?"DM ": "",
		cmask.s.IsReinitSupported?"RI ": "",
		cmask.s.IsSNMPTunnelingSupported?"SNMP ": "",
		cmask.s.IsConnectionManagementSupported?"CM ": "",
		cmask.s.IsPKeySwitchExternalPortTrapSupported?"PST ": "",
		cmask.s.IsSystemImageGuidSupported?"SIG ": "",
		cmask.s.IsSMDisabled?"SMD ": "",
		cmask.s.IsLEDInfoSupported?"LED ": "",
		cmask.s.IsPKeyNVRAM?"PK ": "",
		cmask.s.IsMKeyNVRAM?"MK ": "",
		cmask.s.IsSLMappingSupported?"SL ": "",
		cmask.s.IsAutomaticMigrationSupported?"APM ": "",
		cmask.s.IsTrapSupported?"Trap ": "",
		cmask.s.IsNoticeSupported?"Not ": "",
		cmask.s.IsSM?"SM ": "");
}

#define FORMAT_MULT_THRESHOLD_FRACTION 1000	/* for fractions of a second */
#define FORMAT_MULT_THRESHOLD 180	/* for seconds and minutes */

/* convert 4.096us*2^timeout to a reasonable value
 * and format into a buffer as #### uu
 * where uu is units and timeout is limited to 31
 * buffer will be exactly 7 characters + terminating \0
 */
static __inline
void FormatTimeoutMult(char buf[8], uint8 timeout)
{
	uint64 time64 = 4096 * ((uint64)1<<timeout);	/* units of ns */

#if defined(VXWORKS)
	if (time64 < FORMAT_MULT_THRESHOLD_FRACTION)
		snprintf(buf, 8, "%4u ns", (uint32)time64);
	else if (time64 < FORMAT_MULT_THRESHOLD_FRACTION*1000)
		snprintf(buf, 8, "%4u us", ((uint32)time64)/1000);
	else {
		/* Linux kernel doesn't have 64 bit division support */
		uint32 time32 = 4096 * (((uint32)1<<timeout)/1000000);	/* units of ms */
		if (time64 < 9999000000ULL) {
			/* recompute since /1000000 will yield 0 for timeout < 2.1 sec */
			time32=(4096 * (((uint32)1<<timeout)/10))/100000;
		}
		if (time32 < FORMAT_MULT_THRESHOLD_FRACTION)
			snprintf(buf, 8, "%4u ms", time32);
		else if (time32 < FORMAT_MULT_THRESHOLD*1000)
			snprintf(buf, 8, "%4u s ", time32/1000);
		else if (time32 < FORMAT_MULT_THRESHOLD*1000*60)
			snprintf(buf, 8, "%4u m ", time32/(1000*60));
		else 
			snprintf(buf, 8, "%4u h ", time32/(1000*60*60));
	}
#else
	if (time64 < FORMAT_MULT_THRESHOLD_FRACTION)
		snprintf(buf, 8, "%4u ns", (uint32)time64);
	else if (time64 < FORMAT_MULT_THRESHOLD_FRACTION*1000)
		snprintf(buf, 8, "%4u us", ((uint32)time64)/1000);
	else {
		/* Linux kernel doesn't have 64 bit division support */
		uint32 time32 = 4096 * (((uint32)1<<timeout)/1000000);	/* units of ms */
		if (time64 < 9999000000ULL) {
			/* recompute since /1000000 will yield 0 for timeout < 2.1 sec */
			time32=(4096 * (((uint32)1<<timeout)/10))/100000;
		}
		if (time32 < FORMAT_MULT_THRESHOLD_FRACTION)
			snprintf(buf, 8, "%4u ms", time32);
		else if (time32 < FORMAT_MULT_THRESHOLD*1000)
			snprintf(buf, 8, "%4u s ", time32/1000);
		else if (time32 < FORMAT_MULT_THRESHOLD*1000*60)
			snprintf(buf, 8, "%4u m ", time32/(1000*60));
		else 
			snprintf(buf, 8, "%4u h ", time32/(1000*60*60));
	}
#endif
}

/* Convert Timeout value from usec to
 * timeout_mult where usec = 4.096usec * 2^timeout_mult
 */
static __inline
uint32 TimeoutTimeToMult(uint64 timeout_us)
{
	/* all values are rounded up, comments reflect exact value */
	if (timeout_us <= 4)
		return 0;	/* 4.096 us */
	else if (timeout_us <= 8)
		return 1;	/* 8.192 us */
	else if (timeout_us <= 16)
		return 2;	/* 16.384 us */
	else if (timeout_us <= 32)
		return 3;	/* 32.768 us */
	else if (timeout_us <= 65)
		return 4;	/* 65.536 us */
	else if (timeout_us <= 131)
		return 5;	/* 131.072 us */
	else if (timeout_us <= 262)
		return 6;	/* 262.144 us */
	else if (timeout_us <= 524)
		return 7;	/* 524.288 us */
	else if (timeout_us <= 1048)
		return 8;	/* 1048.576 us */
	else if (timeout_us <= 2097)
		return 9;	/* 2.097 ms */
	else if (timeout_us <= 4194)
		return 10;	/* 4.197 ms */
	else if (timeout_us <= 8388)
		return 11;	/* 8.388 ms */
	else if (timeout_us <= 16777)
		return 12;	/* 16.777 ms */
	else if (timeout_us <= 33554)
		return 13;	/* 33.554 ms */
	else if (timeout_us <= 67108)
		return 14;	/* 67.1 ms */
	else if (timeout_us <= 134217)
		return 15;	/* 134.2 ms */
	else if (timeout_us <= 268435)
		return 16;	/* 268.4 ms */
	else if (timeout_us <= 536870)
		return 17;	/* 536.8 ms */
	else if (timeout_us <= 1073741)
		return 18;/* 1.073 s */
	else if (timeout_us <= 2147483)
		return 19;/* 2.148 s */
	else if (timeout_us <= 4294967)
		return 20;/* 4.294 s */
	else if (timeout_us <= 8589934)
		return 21;/* 8.589 s */
	else if (timeout_us <= 17179869)
		return 22;/* 17.179 s */
	else if (timeout_us <= 34359738)
		return 23;/* 34.359 s */
	else if (timeout_us <= 68719476)
		return 24;/* 68.719 s */
	else if (timeout_us <= 137438953ll)
		return 25;/* 2.2 minutes */
	else if (timeout_us <= 274877906ll)
		return 26; /* 4.5 minutes */
	else if (timeout_us <= 549755813ll)
		return 27; /* 9 minutes */
	else if (timeout_us <= 1099511628ll)
		return 28;	/* 18 minutes */
	else if (timeout_us <= 2199023256ll)
		return 29;	/* 0.6 hr */
	else if (timeout_us <= 4398046511ll)
		return 30;	/* 1.2 hr	 */
	else
		return 31;	/* 2.4 hr */
}

/* Convert Timeout value from ms to
 * timeout_mult where ms = 4.096usec * 2^timeout_mult
 */
static __inline
uint32 TimeoutTimeMsToMult(uint32 timeout_ms)
{
	/* all values are rounded up, comments reflect exact value */
	if (timeout_ms < 1)
		return 0;	/* 4.096 us */
	else if (timeout_ms <= 1)
		return 8;	/* 1048.576 us */
	else if (timeout_ms <= 2)
		return 9;	/* 2.097 ms */
	else if (timeout_ms <= 4)
		return 10;	/* 4.197 ms */
	else if (timeout_ms <= 8)
		return 11;	/* 8.388 ms */
	else if (timeout_ms <= 16)
		return 12;	/* 16.777 ms */
	else if (timeout_ms <= 33)
		return 13;	/* 33.554 ms */
	else if (timeout_ms <= 67)
		return 14;	/* 67.1 ms */
	else if (timeout_ms <= 134)
		return 15;	/* 134.2 ms */
	else if (timeout_ms <= 268)
		return 16;	/* 268.4 ms */
	else if (timeout_ms <= 536)
		return 17;	/* 536.8 ms */
	else if (timeout_ms <= 1073)
		return 18;/* 1.073 s */
	else if (timeout_ms <= 2147)
		return 19;/* 2.148 s */
	else if (timeout_ms <= 4294)
		return 20;/* 4.294 s */
	else if (timeout_ms <= 8589)
		return 21;/* 8.589 s */
	else if (timeout_ms <= 17179)
		return 22;/* 17.179 s */
	else if (timeout_ms <= 34359)
		return 23;/* 34.359 s */
	else if (timeout_ms <= 68719)
		return 24;/* 68.719 s */
	else if (timeout_ms <= 137438)
		return 25;/* 2.2 minutes */
	else if (timeout_ms <= 274877)
		return 26; /* 4.5 minutes */
	else if (timeout_ms <= 549755)
		return 27; /* 9 minutes */
	else if (timeout_ms <= 1099511)
		return 28;	/* 18 minutes */
	else if (timeout_ms <= 2199023)
		return 29;	/* 0.6 hr */
	else if (timeout_ms <= 4398046)
		return 30;	/* 1.2 hr	 */
	else
		return 31;	/* 2.4 hr */
}

/* using the LMC, convert a lid to the Source Path Bits for use in
 * and address vector
 */
static __inline
IB_PATHBITS
LidToPathBits(
	IN IB_LID lid,
	IN IB_LMC lmc
	)
{
	return lid & ( (1<<lmc) -1);
}

/* using the LMC and base lid, convert a lid to the Source Path Bits for use in
 * and address vector
 */
static __inline
IB_LID
PathBitsToLid(
	IN IB_LID baselid,
	IN IB_PATHBITS pathbits,
	IN IB_LMC lmc
	)
{
	IB_LID mask = (1<<lmc)-1;
	return (baselid & ~mask) | (pathbits & mask);
}


/*
 * Returns TRUE if the PKey value is valid for the specified port attributes.
 * Sets the PKeyIndex output value, if any, if a match is found.
 */
static __inline boolean
ValidatePKey(
	IN const IB_PORT_ATTRIBUTES* const pPortAttr,
	IN const IB_P_KEY PKeyValue,
	OUT uint16* const pPKeyIndex OPTIONAL )
{
	uint16	i;

	/* Search for the PKey index for the PKey value provided. */
	for( i = 0; i < pPortAttr->NumPkeys; i++ )
	{
		/* Check for invalid PKey values.  Invalid values have */
		/* the lower 15 bits set to zero. */
		if( !(pPortAttr->PkeyTable[i] & 0x7FFF) )
			continue;

        /* Compare the PKey value in the port's PKey table with */
        /* the value provided. */
		if( ((pPortAttr->PkeyTable[i] & 0xffff) == (PKeyValue & 0xffff)) )
		{
			/* We found a match on the value.  Copy the PKey index. */
			if( pPKeyIndex )
				*pPKeyIndex = i;
			return TRUE;
		}
	}

	return FALSE;
}

/*
 * Returns TRUE if the PKey value is found in the specified port attributes.
 * Sets the PKeyIndex output value, if any, if a match is found.
 */
static __inline boolean
LookupPKey(
	IN const IB_PORT_ATTRIBUTES* const pPortAttr,
	IN const IB_P_KEY PKeyValue,
	IN const boolean masked,
	OUT uint16* const pPKeyIndex OPTIONAL )
{
	uint16	i;
	uint16 mask = masked?0x7fff:0xffff;

	/* Search for the PKey index for the PKey value provided. */
	for( i = 0; i < pPortAttr->NumPkeys; i++ )
	{
		/* Check for invalid PKey values.  Invalid values have */
		/* the lower 15 bits set to zero. */
		if( !(pPortAttr->PkeyTable[i] & 0x7FFF) )
			continue;

		/* Compare the PKey value in the port's PKey table with */
		/* the value provided. */
		if( (pPortAttr->PkeyTable[i] & mask) == (PKeyValue & mask) )
		{
			/* We found a match on the value.  Copy the PKey index. */
			if( pPKeyIndex )
				*pPKeyIndex = i;
			return TRUE;
		}
	}

	return FALSE;
}

/*
 * Return GID index if the GID is valid for the specified port attributes.
 * Return -1 if invalid/not configured
 */
static __inline int32
GetGidIndex( 
	IN const IB_PORT_ATTRIBUTES* const pPortAttr,
	IN const IB_GID* const pGid )
{
	uint8	i;

	/* Compare the port's GIDs to see if the requested GID matches. */
	for( i = 0; i < pPortAttr->NumGIDs; i++ )
	{
		/* workaround SM's which forget to specify SubnetPrefix */
		if (pPortAttr->GIDTable[i].Type.Global.SubnetPrefix == 0)
		{
			if (pPortAttr->GIDTable[i].Type.Global.InterfaceID ==
				pGid->Type.Global.InterfaceID)
			{
				return i;
			}
		} else {
			if( !MemoryCompare( &pPortAttr->GIDTable[i], 
				pGid, sizeof( IB_GID ) ) )
			{
				return i;
			}
		}
	}

	return -1;
}

/* non-Wildcarded compare of 2 64 bit values
 * Return:
 * 	0 : v1 == v2
 * 	-1: v1 < v2
 * 	1 : v1 > v2
 */
static __inline int
CompareU64(uint64 v1, uint64 v2)
{
	if (v1 == v2)
		return 0;
	else if (v1 < v2)
		return -1;
	else
		return 1;
}

/* Wildcarded compare of 2 64 bit values
 * Return:
 * 	0 : v1 == v2
 * 	-1: v1 < v2
 * 	1 : v1 > v2
 * 	if v1 or v2 is 0, they are considered wildcards and match any value
 */
static __inline int
WildcardCompareU64(uint64 v1, uint64 v2)
{
	if (v1 == 0 || v2 == 0 || v1 == v2)
		return 0;
	else if (v1 < v2)
		return -1;
	else
		return 1;
}

/* Compare Gid1 to Gid2 (host byte order)
 * Return:
 * 	0 : Gid1 == Gid2
 * 	-1: Gid1 < Gid2
 * 	1 : Gid1 > Gid2
 * This also allows for Wildcarded compare.
 * A MC Gid with the lower 56 bits all 0, will match any MC gid
 * A SubnetPrefix of 0 will match any top 64 bits of a non-MC gid
 * A InterfaceID of 0 will match any low 64 bits of a non-MC gid
 * Coallating order:
 *  non-MC Subnet Prefix (0 is wildcard and comes first)
 *  non-MC Interface ID (0 is wilcard and comes first)
 * 	MC wildcard
 * 	MC by value of low 56 bits (0 is wildcard and comes first)
 */
static __inline int
WildcardGidCompare(IN const IB_GID* const pGid1, IN const IB_GID* const pGid2 )
{
	if (pGid1->Type.Multicast.s.FormatPrefix == IPV6_MULTICAST_PREFIX
		&& pGid2->Type.Multicast.s.FormatPrefix == IPV6_MULTICAST_PREFIX)
	{
		/* Multicast compare: compare low 120 bits, 120 bits of 0 is wildcard */
		uint64 h1 = pGid1->AsReg64s.H & ~IB_GID_MCAST_FORMAT_MASK_H;
		uint64 h2 = pGid2->AsReg64s.H & ~IB_GID_MCAST_FORMAT_MASK_H;
		/* check for 120 bits of wildcard */
		if ((h1 == 0 && pGid1->AsReg64s.L == 0)
			|| (h2 == 0 && pGid2->AsReg64s.L == 0))
		{
			return 0;
		} else if (h1 < h2) {
			return -1;
		} else if (h1 > h2) {
			return 1;
		} else {
			return CompareU64(pGid1->AsReg64s.L, pGid1->AsReg64s.L);
		}
	} else if (pGid1->Type.Multicast.s.FormatPrefix == IPV6_MULTICAST_PREFIX) {
		/* Gid1 is MC, Gid2 is other, treat MC as > others */
		return 1;
	} else if (pGid2->Type.Multicast.s.FormatPrefix == IPV6_MULTICAST_PREFIX) {
		/* Gid1 is other, Gid2 is MC, treat other as < MC */
		return -1;
	} else {
		/* Non-Multicast compare: compare high 64 bits */
		/* Note all other GID formats are essentially a prefix in upper */
		/* 64 bits and a identifier in the low 64 bits */
		/* so this covers link local, site local, global formats */
		int res = WildcardCompareU64(pGid1->AsReg64s.H, pGid2->AsReg64s.H);
		if (res == 0)
		{
			return WildcardCompareU64(pGid1->AsReg64s.L, pGid2->AsReg64s.L);
		} else {
			return res;
		}
	}
}

/* Compute the size of the port attributes structure in src
 * It is assumed that the GID and PKey tables are after src
 * Typically port attributes lists for a CA should be structured as:
 * 		Port 1 Attributes
 * 		Port 1 GID and PKey Tables
 * 		Port 2 Attributes
 * 		Port 2 GID and PKey Tables
 * 		etc
 */
static __inline uint32
IbPortAttributesSize(IN const IB_PORT_ATTRIBUTES* ptr)
{
	uint32 result, temp;

	/* this is best case size */
	result = sizeof(IB_PORT_ATTRIBUTES)
					+ sizeof(IB_GID) * ptr->NumGIDs
					+ sizeof(IB_P_KEY) * ptr->NumPkeys;
	/* however VPD could add pad bytes for alignment */
	temp = (uint8*)(ptr->GIDTable + ptr->NumGIDs) - (uint8*)ptr;
	result = MAX(result, temp);
	temp = (uint8*)(ptr->PkeyTable + ptr->NumPkeys) - (uint8*)ptr;
	result = MAX(result, temp);
	return result;
}

/* Copy the port attributes structure from src to dest for size bytes
 * size must be computed using a function such as IbPortAttributesSize above.
 * It is assumed that the GID and PKey tables are within size bytes from src.
 * Typically port attributes lists for a CA should be structured as:
 * 		Port 1 Attributes
 * 		Port 1 GID and PKey Tables
 * 		Port 2 Attributes
 * 		Port 2 GID and PKey Tables
 * 		etc
 * 	dest->Next is set to NULL, caller must initialize if there is a Next
 */
static __inline void
IbCopyPortAttributes(
				OUT IB_PORT_ATTRIBUTES *dest,
				IN const IB_PORT_ATTRIBUTES* src,
				IN uint32 size
				)
{
	MemoryCopy(dest, src, size);
	dest->GIDTable = (IB_GID *)((uintn)dest + ((uintn)src->GIDTable - (uintn)src));
	dest->PkeyTable = (IB_P_KEY *)((uintn)dest + ((uintn)src->PkeyTable - (uintn)src));
	/*dest->Next = (IB_PORT_ATTRIBUTES*)((uintn)dest + size);*/
	dest->Next = NULL;
}

#undef IbmbpsToMBps
#undef IbmbpsToMBpsExt

#ifdef __cplusplus
};
#endif

#endif	/* _IBA_IB_HELPER_H_ */
