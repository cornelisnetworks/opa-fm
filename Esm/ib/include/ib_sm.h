/* BEGIN_ICS_COPYRIGHT2 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

//----------------------------------------------------------------------/
//                                                                      /
// FILE NAME                                                            /
//    ib_mad.h                                                          /
//                                                                      /
// DESCRIPTION                                                          /
//    general transport definitions                                     /
//                                                                      /
// DATA STRUCTURES                                                      /
//    Trap64_t		Trap information (port up)			/
//    Trap65_t		Trap information (link down)			/
//    Trap66_t		Trap information (multicast group created)	/
//    Trap67_t		Trap information (multicast group deleted)	/
//    Trap128_t		Trap information (link state change)		/
//    Trap129_t		Trap information (link integrity threshold)	/
//    Trap130_t		Trap information (buffer overrun threshold)	/
//    Trap131_t		Trap information (flow control update timer)	/
//    Trap144_t		Trap information (change to cap mask/other)	/
//    Trap256_t		Trap information (bad M_Key)			/
//    Trap257_t		Trap information (bad P_Key)			/
//    Trap258_t		Trap information (bad Q_Key)			/
//    SMTrap259_t	Trap information (bad P_Key)			/
//    NodeDesc_t	Node description				/
//    GuidInfo_t	IEEE address information			/
//    PKeyBlock_t	Partition Key 					/
//                                                                      /
// FUNCTIONS                                                            /
//    None                                                              /
//                                                                      /
// DEPENDENCIES                                                         /
//    ib_types.h                                                        /
//                                                                      /
//                                                                      /
// HISTORY                                                              /
//                                                                      /
//    NAME      DATE  REMARKS                                           /
//                                                                      /
//----------------------------------------------------------------------/

//----------------------------------------------------------------------//
//                                                                      //
//	WARNING:  These structures do NOT represent the IBTA defined    //
//		  structures as they appear in the Volume1 spec.        //
//                                                                      //
//----------------------------------------------------------------------//

#ifndef	_IB_SM_H_
#define	_IB_SM_H_

#include "ib_types.h"

// IBTA:  Volume 1, Section 14.2.5

//
//	Subnet Management Attributes (MAD_SMA)
//
#define	MAD_SMA_NOTICE		0x0002
#define	MAD_SMA_NODEDESC	0x0010
#define	MAD_SMA_NODEINFO	0x0011
#define	MAD_SMA_SWITCHINFO	0x0012
#define	MAD_SMA_GUIDINFO	0x0014
#define	MAD_SMA_PORTINFO	0x0015
#define	MAD_SMA_PART_TABLE	0x0016
#define	MAD_SMA_VL_ARBITRATION	0x0018
#define	MAD_SMA_LFT		0x0019
#define	MAD_SMA_RFT		0x001a
#define	MAD_SMA_MFT		0x001b
#define	MAD_SMA_SMINFO		0x0020
#define	MAD_SMA_VENDOR_DIAG	0x0030
#define	MAD_SMA_LEDINFO		0x0031

// IBTA:  Volume 1, Section 14.2.5.1

//
//	Subnet Management Traps (MAD_SMT)
//
#define	MAD_SMT_PORT_UP		 64
#define	MAD_SMT_PORT_DOWN	 65
#define	MAD_SMT_MCAST_GRP_CREATED 66
#define	MAD_SMT_MCAST_GRP_DELETED 67
#define	MAD_SMT_UNPATH 68
#define	MAD_SMT_REPATH 69
#define	MAD_SMT_PORT_CHANGE	128
#define	MAD_SMT_LINK_INTEGRITY	129
#define	MAD_SMT_BUF_OVERRUN	130
#define	MAD_SMT_FLOW_CONTROL	131
#define MAD_SMT_CAPABILITYMASK_CHANGE 144
#define MAD_SMT_SYSTEMIMAGEGUID_CHANGE 145
#define	MAD_SMT_BAD_MKEY	256
#define	MAD_SMT_BAD_PKEY	257
#define	MAD_SMT_BAD_QKEY	258
/* optional bad pkey on switch external port */
#define MAD_SMT_BAD_PKEY_ONPORT 259

// IBTA:  Volume 1, Section 14.2.5.2

#define ND_LEN				64

// IBTA:  Volume 1, Section 14.2.5.3

#define NI_LPN_MASK       0xff000000
#define NI_VID_MASK       0x00ffffff

#define NI_TYPE_CA          0x01
#define NI_TYPE_SWITCH      0x02
#define NI_TYPE_ROUTER      0x03

#define NI_TYPE_MASK        0x03


// IBTA:  Volume 1, Section 14.2.5.5

#define GUID_INFO_GUID_COUNT         8

typedef	struct {
	uint64_t	guid[GUID_INFO_GUID_COUNT];	// list of GUID block elements
} GuidInfo_t;


// IBTA:  Table 126 Page 665  Vol1 1.0.a (June 19 2001)

//
// CapabilityMask.
//
// Note for STL capability mask:
// STL provides a bitfield definition for its capability mask. (see stl_sm_types.h)
// Many capability bits that were optional in IB are now mandatory in STL.
// However, as per STL1G1 spec, STL returns these "mandatory" capability
// bits as 0.  Be careful when checking STL capabilities and using 
// bitmask manipulations below instead of the new STL bitfields.
// STL responses for the IB capability bits are annotated below.

#define PI_CM_RESERVED                  0xff00e011   // These are all reserved bits.
#define PI_CM_IS_SM                               0x00000002 /* STL: 0/1 */
//#define PI_CM_IS_NOTICE_SUPPORTED                 0x00000004 /* STL: 0   */
//#define PI_CM_IS_TRAP_SUPPORTED                   0x00000008 /* STL: 0   */
//#define PI_CM_IS_OPTIONAL_IPD_SUPPORTED           0x00000010 /* STL: 0   */
#define PI_CM_IS_AUTOMATIC_MIGRATION_SUPPORTED    0x00000020 /* STL: 0/1 */
//#define PI_CM_IS_SL_MAPPING_SUPPORTED             0x00000040 /* STL: 0   */
//#define PI_CM_IS_MKEY_NVRAM                       0x00000080 /* STL: 0   */
//#define PI_CM_IS_PKEY_NVRAM                       0x00000100 /* STL: 0   */
//#define PI_CM_IS_LEDINFO_SUPPORTED                0x00000200 /* STL: 0   */
//#define PI_CM_IS_SM_DISABLED                      0x00000400 /* STL: 0   */
//#define PI_CM_IS_SYS_IMAGE_SUPPORTED              0x00000800 /* STL: 0   */
//#define PI_CM_IS_PKEY_SWITCH_EXTERNAL_TRAP_SUPPORTED  0x00001000 /* STL: 0   */
//#define PI_CM_IS_EXT_LINK_SPEED_SUPPORTED         0x00004000 /* STL: 0   */
#define PI_CM_IS_COM_MGT_SUPPORTED                0x00010000 /* STL: 0/1 */
//#define PI_CM_IS_SNMP_SUPPORTED                   0x00020000 /* STL: 0   */
//#define PI_CM_IS_REINIT_SUPPORTED                 0x00040000 /* STL: 0   */
#define PI_CM_IS_DEVICE_MGT_SUPPORTED             0x00080000 /* STL: 0/1 */
#define PI_CM_IS_VENDOR_CLASS_SUPPORTED           0x00100000 /* STL: 0/1 */
//#define PI_CM_IS_DRNOTICE_SUPPORTED               0x00200000 /* STL: 0   */
#define PI_CM_IS_CAPMASKNOTICE_SUPPORTED          0x00400000 /* STL: 0/1 */
//#define PI_CM_IS_BOOT_MGT_SUPPORTED               0x00800000 /* STL: 0   */
//#define PI_CM_IS_LINK_ROUNDTRIP_LATENCY_SUPPORTED 0x01000000 /* STL: 0   */
//#define PI_CM_IS_CLIENT_REREGISTER_SUPPORTED      0x02000000 /* STL: 0   */
//#define PI_CM_IS_OTHER_LOCAL_NOTICE_SUPPORTED     0x04000000 /* STL: 0   */
//#define PI_CM_IS_MULTICAST_FDB_TOP_SUPPORTED      0x40000000 /* STL: 0   */

// IBTA:  Volume 1, Section 14.2.5.7

/* Macros for extracting the two pieces of a PKey */
#define PKEY_TYPE(key) ( ((uint16_t) (key)) >> 15 )
#define PKEY_VALUE(key) ( ((uint16_t) (key)) & 0x7FFF )
#define MAKE_PKEY(type, key) \
	((uint16_t) ((((uint16_t)(type)) & 0x01) << 15) | ((key) & 0x7FFF))

typedef PKey_t PKeyBlock_t;

// IBTA:  Volume 1, Section 14.2.5.10

#define LFTABLE_LIST_COUNT 64

#define MFTABLE_LIST_COUNT 32	// entries per MFT Table block
#define MFTABLE_POSITION_COUNT 16	// Positions per portMask

#define	MFT_MAX_INDEX		511		// maximum Block Number value for Get or Set
#define MFT_MAX_POSITION	15		// Maximum PortMask Position value for Get or Set

// IBTA:  Volume 1, Section 14.2.5.13

#define	SM_STATE_NOTACTIVE	0
#define	SM_STATE_DISCOVERING	1
#define	SM_STATE_STANDBY	2
#define	SM_STATE_MASTER		3

#define	SM_AMOD_HANDOVER	1		// SM state transitions
#define	SM_AMOD_ACKNOWLEDGE	2
#define	SM_AMOD_DISABLE		3
#define	SM_AMOD_STANDBY		4
#define	SM_AMOD_DISCOVER	5

#define SM_VERSION		0x01

//
// SM Service Record Capability bits
//
#define SM_CAPABILITY_NONE		0x00000000
#define SM_CAPABILITY_VSWINFO	0x00000001

/*
 * Per SM data
 */ 
typedef struct {
    uint32_t        version;                /* version = 1 */

    uint32_t        fullSyncStatus;         /* 0=un-initialized, 1=in progress, 2=synchronized, 3=failure */
    uint32_t        fullSyncFailCount;      /* count of consecutive full sync attempt failures */
    uint32_t        fullTimeSyncFail;       /* time of last full sync attempt failure */
    uint32_t        fullTimeLastSync;       /* time of last scheduled full sync - 0 if never sync'd */

    uint32_t        informSyncStatus;       /* 0=un-initialized, 1=in progress, 2=synchronized, 3=failure */
    uint32_t        informSyncFailCount;    /* count of consecutive sync attempt failures for INFORM records */
    uint32_t        informTimeSyncFail;     /* time of last INFORM sync attempt failure */
    uint32_t        informTimeLastSync;     /* time of last sync of INFORM records; 0=un-initialized */

    uint32_t        groupSyncStatus;        /* 0=un-initialized, 1=in progress, 2=synchronized, 3=failure */
    uint32_t        groupSyncFailCount;     /* count of consecutive sync attempt failures for MCAST group records */
    uint32_t        groupTimeSyncFail;      /* time of last MCAST group sync attempt failure */
    uint32_t        groupTimeLastSync;      /* time of last sync of mcast groups; 0=un-initialized */

    uint32_t        serviceSyncStatus;      /* 0=un-initialized, 1=in progress, 2=synchronized, 3=failure */
    uint32_t        serviceSyncFailCount;   /* count of consecutive sync attempt failures for SERVICE records */
    uint32_t        serviceTimeSyncFail;    /* time of last SERVICE sync attempt failure */
    uint32_t        serviceTimeLastSync;    /* time of last sync of services; 0=un-initialized */

	uint32_t        mcrootSyncStatus;      /* 0=un-initialized, 1=in progress, 2=synchronized, 3=failure */
    uint32_t        mcrootSyncFailCount;   /* count of consecutive sync attempt failures for MC Root records */
    uint32_t        mcrootTimeSyncFail;    /* time of last MC Root sync attempt failure */
    uint32_t        mcrootTimeLastSync;    /* time of last sync of services; 0=un-initialized */

    uint32_t        datelineGuidSyncStatus;   /*0=un-initialized, 1=in progress, 2=synchronized, 3=failure */
    uint32_t        datelineGuidSyncFailCount;/* count of consecutive sync attempt failures for Dateline Switch GUID records */
    uint32_t        datelineGuidTimeSyncFail; /* time of last Dateline Switch GUID sync attempt failure */
    uint32_t        datelineGuidTimeLastSync; /* time of last sync of services; 0=un-initialized */
} SMDBSync_t;
typedef SMDBSync_t  *SMDBSyncp;         /* SM DBSYNC pointer type */

// FM Protocol Version


// version 1 - initial creation of FM protocol version feature
// version 2 - 10.0.1 releases and before
// version 3 - 10.1 release
// version 4 - 10.2 release
// version 5 - 10.3 release
// version 6 - 10.4 release
// version 7 - 10.5 release
// version 8 - 10.6 release
// version 9 - 10.7 release
// version 10 - 10.8 release
// version 11 - 10.9 release
// version 12 - 10.9.3 release
// version 13 - 10.10.1 release
#define     FM_PROTOCOL_VERSION    13


typedef struct {
    uint32_t        protocolVersion;

    // SM checksums
	uint32_t		smVfChecksum;			/* Virtual Fabric database checksum */
    uint32_t        smConfigChecksum;		/* SM configuration checksum */

    // PM checksums
    uint32_t        pmConfigChecksum;

	// EM checksums
	uint32_t		emConfigChecksum;		/* Ethernet over STL */

	// spare values for future expansion so packet framing does not need to change
	uint32_t		spare1;
	uint32_t		spare2;
	uint32_t		spare3;
	uint32_t		spare4;
	uint32_t		spare5;
	uint32_t		spare6;
	uint32_t		spare7;
	uint32_t                spare8;

} SMDBCCCSync_t;
typedef SMDBCCCSync_t  *SMDBCCCSyncp;         /* SM DBSYNC CCC pointer type */

// File transport structure

#define DBSYNC_FILE_TRANSPORT_VERSION   1

typedef enum {
	DBSYNC_FILE_XML_CONFIG = 1,
	DBSYNC_PM_SWEEP_IMAGE,
	DBSYNC_PM_HIST_IMAGE
} DBSyncFileType_t;

#define SMDBSYNCFILE_NAME_LEN 64
typedef struct {
    uint32_t        version;        /* structure version */
    uint32_t        length;         /* structure length */
    uint32_t        activate;       /* should file be activated */
    uint32_t        type;           /* type of file */
    uint32_t        size;           /* size of file */
    uint32_t        spare1;
    uint32_t        spare2;
    uint32_t        spare3;
    uint32_t        spare4;
    char            name[SMDBSYNCFILE_NAME_LEN];       /* filename */
} SMDBSyncFile_t;
typedef SMDBSyncFile_t *SMDBSyncFilep;  /* pointer to sync file transport structure */

#endif	// _IB_SM_H_

