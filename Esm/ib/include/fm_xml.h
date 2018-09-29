/* BEGIN_ICS_COPYRIGHT2 ****************************************

Copyright (c) 2015-2018, Intel Corporation

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

//===========================================================================//
//
// FILE NAME
//    fm_xml.h
//
// DESCRIPTION
//    This file contains structures used for parsing the fm_config.xml
//    file for configuring SM
//
//===========================================================================//

#ifndef	_FM_XML_H_
#define	_FM_XML_H_

#include <stdio.h>
#include <signal.h>
#include <ixml.h>
#include "cs_log.h"


#ifdef __VXWORKS__
#include "regexp.h"
#else
#include "regex.h"
#endif

#ifndef __VXWORKS__
#define MAX_SUBNET_SIZE						64000		// HSM has no realistic max
#define DEFAULT_SUBNET_SIZE					2560		// XML default subnet size
#else  // __VXWORKS__
#define MAX_SUBNET_SIZE						288			// ESM has a max subnet size of 288
#define DEFAULT_SUBNET_SIZE					100		    // XML default subnet size
#endif // __VXWORKS__

// Key PM config parameter defaults and ranges
#define PM_DEFAULT_TOTAL_IMAGES	10			// for Image[] and history[]
#define PM_MIN_TOTAL_IMAGES	3				// for Image[] and history[]
#ifdef __VXWORKS__
// keep vxWorks limited to save memory and also because pm_compute_pool_size
// is called before reading config file
#define PM_MAX_TOTAL_IMAGES	10				// for Image[] and history[]
#else
#define PM_MAX_TOTAL_IMAGES	100000			// for Image[] and history[]
#endif

#define PM_DEFAULT_FF_IMAGES 5				// for freezeFrame[]
#define PM_MIN_FF_IMAGES 2					// for freezeFrame[]
#ifdef __VXWORKS__
#define PM_MAX_FF_IMAGES (PM_MAX_TOTAL_IMAGES-2)	// for freezeFrame[]
#else
// due to some linear searches, we limit number of FF.  2-3 per client is enough
// and only a handful of clients are expected, 100 should be plenty
#define PM_MAX_FF_IMAGES 100				// for freezeFrame[]
#endif

#define PM_DEFAULT_PA_MAX_CLIENTS 3
#define PM_MIN_MAX_CLIENTS	3
#ifdef __VXWORKS__
// keep vxWorks limited to save memory and also because pm_compute_pool_size
// is called before reading config file
#define PM_MAX_MAX_CLIENTS	4
#else
#define PM_MAX_MAX_CLIENTS	20
#endif

#define PM_DEFAULT_FF_LEASE 60				// in seconds

#define PM_DEFAULT_MAX_ATTEMPTS			3
#define PM_DEFAULT_RESP_TIMEOUT			250
#define PM_DEFAULT_MIN_RESP_TIMEOUT		35
#define PM_DEFAULT_SWEEP_ERRORS_LOG_THRESHOLD	10
#define PM_DEFAULT_MAX_PARALLEL_NODES	10
#define PM_DEFAULT_PMA_BATCH_SIZE		2

#define STL_PM_MAX_DG_PER_PMPG	5		//Maximum number of Monitors allowed in a PmPortGroup
#define STL_PM_GROUPNAMELEN		64
#define STL_PM_MAX_CUSTOM_PORT_GROUPS 8 // HFIs and SWs take the first 2 slots.

#define FE_LISTEN_PORT     	3245    // FAB_EXEC listen port
#define FE_WIN_SIZE			1       // Default Window Size for Reliable
#define FE_MAX_WIN_SIZE    	16      // Maximum window size
#define FE_MIN_WIN_SIZE    	1       // Minimum window size

#define FE_MAX_TRAP_SUBS    20      // Maximum number of TrapSubs

// max number of times to fail to check on master
#define SM_CHECK_MASTER_INTERVAL 5
#define SM_CHECK_MASTER_MAX_COUNT 3

#define SM_TRAP_THRESHOLD_MIN 10
#define SM_TRAP_THRESHOLD_MAX 100
#define SM_TRAP_THRESHOLD_DEFAULT 0 // off

#define SM_TRAP_THRESHOLD_COUNT_DEFAULT 10
#define SM_TRAP_THRESHOLD_COUNT_MIN     2
#define SM_TRAP_THRESHOLD_COUNT_MAX     100

#define STL_CONFIGURABLE_SLS		16


#define NONRESP_TIMEOUT  	600ull
#define NONRESP_MAXRETRY 	3

// Cable Info Policy
#define CIP_NONE 0
#define CIP_LINK 1
#define CIP_PORT 2

// SwitchCascadeActivateEnable setting
#define SCAE_DISABLED 0
#define SCAE_SW_ONLY 1
#define SCAE_ALL 2

// PortBounceLogLimit no limit setting
#define PORT_BOUNCE_LOG_NO_LIMIT 0

// While logging of traps from a port is suppressed, summary trap information
// will be logged for that port after accumulating a certain number of traps.
// This number starts with SM_TRAP_LOG_SUMMARY_THRESHOLD_START.
// Once that number is reached and sa_TrapNeedsLogging() is called, the count
// of traps is reset to zero and the threshold  is increased by
// SM_TRAP_LOG_SUMMARY_THRESHOLD_INCREMENT, with the maximum value of the
// threshold being capped off by  SM_TRAP_LOG_SUMMARY_THRESHOLD_MAX

#define SM_TRAP_LOG_SUMMARY_THRESHOLD_START     5
#define SM_TRAP_LOG_SUMMARY_THRESHOLD_INCREMENT 5
/* Should not be greater than 250 - logSuppressTrapCount is an 8 bit field*/
#define SM_TRAP_LOG_SUMMARY_THRESHOLD_MAX       250

// Below constants are for suppressing trap logging information to reduce
// too much noise in the log

// If two traps are received from the same port with in
// SM_TRAP_LOG_SUPPRESS_TRIGGER_INTERVAL, logging of trap information from
// that port will be suppressed

#define SM_TRAP_LOG_SUPPRESS_TRIGGER_INTERVAL   30      /*in seconds*/

// Preemption Default Values
#define SM_PREEMPT_SMALL_PACKET_MIN 32
#define SM_PREEMPT_SMALL_PACKET_MAX 8192
#define SM_PREEMPT_SMALL_PACKET_DEF 256
#define SM_PREEMPT_SMALL_PACKET_INC 32

#define SM_PREEMPT_LARGE_PACKET_MIN 512
#define SM_PREEMPT_LARGE_PACKET_MAX 8192
#define SM_PREEMPT_LARGE_PACKET_DEF 4096
#define SM_PREEMPT_LARGE_PACKET_INC 512

#define SM_PREEMPT_LIMIT_MIN 256    // Note 0 is a valid value, but effectively turns off preemption
#define SM_PREEMPT_LIMIT_MAX 65024  // This is the max without being infinite
#define SM_PREEMPT_LIMIT_INF 65280  // This is the magical infinite value. SM_PREEMPT_LIMIT_INF / SM_PREEMPT_LIMIT_INC = 0xff.
#define SM_PREEMPT_LIMIT_DEF 4096
#define SM_PREEMPT_LIMIT_INC 256

#define SM_PREEMPT_TAIL_MIN 80
#define SM_PREEMPT_TAIL_DEF 80
#define SM_PREEMPT_TAIL_INC 8

#define SM_PREEMPT_HEAD_MIN 80
#define SM_PREEMPT_HEAD_DEF 80
#define SM_PREEMPT_HEAD_INC 8
#define SM_DEFAULT_MAX_LID  0xBFFF
#define SM_MAX_LID_LIMIT    0xBFFF
#define SM_MAX_9B_LID       0xBFFF

// Note: Multicast and Collective masks are currently
// not configurable.
#define SM_DEFAULT_MULTICAST_MASK 4
#define SM_DEFAULT_COLLECTIVE_MASK 1

// Constants related to our multicast lid collescing scheme
#define MAX_SUPPORTED_MCAST_GRP_CLASSES 32
#define DEFAULT_SW_MLID_TABLE_CAP 1024

#define XML_PARSE_MEMORY_LIMIT				10000000	// limit to 10M per instance of FM

// this define will turn on a series of debug statements for XML parsing
//#define XML_DEBUG

// this define will turn on memory debugging for XML and VF parsing
//#define XML_MEMORY

// Max number on FM instances - ESM only requires one instance and will be
// adjusted for in the parser
#define MAX_INSTANCES 						8

// VF definitions - note that the number of devices is unlimited and has been
// tested to 20,000
#ifdef __VXWORKS__
#define MAX_CONFIGURED_VFABRICS             64
#define MAX_ENABLED_VFABRICS                32
#else /* __VXWORKS__ */
#define MAX_CONFIGURED_VFABRICS           1000
#define MAX_ENABLED_VFABRICS              1000
#endif /* __VXWORKS__ */

#define MAX_VFABRIC_APPS				64
#define MAX_VFABRIC_GROUPS				1024
#define MAX_VFABRIC_MEMBERS_PER_VF			1024
#define MAX_VFABRIC_APPS_PER_VF				64
#define MAX_VFABRIC_APP_SIDS				64
#define MAX_VFABRIC_APP_MGIDS				64
#define MAX_QOS_GROUPS					32
#define MAX_INCLUDED_GROUPS				32
#define MAX_INCLUDED_APPS				32
#define MAX_DEFAULT_GROUPS				64
#define MAX_VFABRIC_DG_MGIDS				32

// Length of VirtualFabric, Group, Application, Fm instance  Names
#define MAX_VFABRIC_NAME					64

// Length of longest compound string in App - typically MGID
#define MAX_VFABRIC_APP_ELEMENT				96

// same quantity as MAX_SUPPORTED_MCAST_GRP_CLASSES in sm_l.h
#define MAX_SUPPORTED_MCAST_GRP_CLASSES_XML 32

// priority range
#define MAX_PRIORITY						15

// VL15CreditRate range
#define MAX_VL15_CREDIT_RATE				21

// Max ports per switch, used in config definitions
#define MAX_SWITCH_PORTS					255

// out of memory error
#define OUT_OF_MEMORY					"Memory limit has been exceeded parsing the XML configuration"
#define OUT_OF_MEMORY_RETURN			"Memory limit has been exceeded parsing the XML configuration\n"
#define PRINT_MEMORY_ERROR				IXmlParserPrintError(state, OUT_OF_MEMORY);

// undefined XML values - used to see if XML config provided value after parsing
#define UNDEFINED_XML8					0xff
#define UNDEFINED_XML16					0xffff
#define UNDEFINED_XML32					0xffffffff
#define UNDEFINED_XML64					0xffffffffffffffffull

// array sizes
#define FILENAME_SIZE					256
#define LOGFILE_SIZE					100
#define STRING_SIZE						32

// config consistency check levels
#define NO_CHECK_CCC_LEVEL				0
#define CHECK_NO_ACTION_CCC_LEVEL		1
#define CHECK_ACTION_CCC_LEVEL			2
#define DEFAULT_CCC_LEVEL				CHECK_ACTION_CCC_LEVEL
#define DEFAULT_CCC_METHOD				MD5_CHECKSUM_METHOD

// values for Sm.PathSelection parameter
#define PATH_MODE_MINIMAL	0	// no more than 1 path per lid
#define PATH_MODE_PAIRWISE	1	// cover every lid on "bigger side" exactly once
#define PATH_MODE_ORDERALL	2	// PAIRWISE, then all src, all dst (skip dups)
#define PATH_MODE_SRCDSTALL	3	// all src, all dst

// Bitmask definitions for [Debug] SM Skip Write
#define SM_SKIP_WRITE_PORTINFO		0x00000001	// Includes Port Info
#define SM_SKIP_WRITE_SMINFO		0x00000002	// Includes Sm Info
#define SM_SKIP_WRITE_GUID			0x00000004	// Includes GUID Info
#define SM_SKIP_WRITE_SWITCHINFO	0x00000008	// Includes Switch Info
#define SM_SKIP_WRITE_VLARB			0x00000020	// Includes VLArb Tables (High / Low / Preempt Matrix)
#define SM_SKIP_WRITE_MAPS			0x00000040	// Includes SL::SC, SC::SL, SC::VL

#define SM_SKIP_WRITE_LFT			0x00000080	// Includes LFT and  MFT

#define SM_SKIP_WRITE_AR			0x00000100	// Includes port group table, portgroup FDB
#define SM_SKIP_WRITE_PKEY			0x00000200
#define SM_SKIP_WRITE_CONG			0x00000400	// Includes HFI congestion / Switch congestion
#define SM_SKIP_WRITE_BFRCTRL		0x00000800
#define SM_SKIP_WRITE_NOTICE		0x00001000
#define SM_SKIP_WRITE_PORTSTATEINFO 0x00002000  // Includes PortStateInfo sets for cascade activation


// This can be used for values such as LogMask's where it is necessary
// to know if a value was supplied.
typedef struct _FmParamU32 {
	uint32_t			value;
	uint8_t				valid;	// is value valid
} FmParamU32_t;

// Structures for parsing of Virtual Fabric XML tags

typedef struct _XmlGuid {
	uint64_t			guid;
	struct _XmlGuid		*next;
} XmlGuid_t;

typedef struct _XmlNode {
	char				node[MAX_VFABRIC_NAME];
	struct _XmlNode		*next;
} XmlNode_t;

typedef struct _XmlIncGroup {
	char group[MAX_VFABRIC_NAME];
	int  dg_index; // Index into dg_config
	struct _XmlIncGroup	*next;
} XmlIncGroup_t;


#define MAX_BRACKETS_SUPPORTED 10
#define MAX_DIGITS_TO_PROCESS 5
#define MAX_NODE_DESC_REG_EXPR 1500
typedef struct _RegexBracketParseInfo{
  int     totalGroups;
  boolean portRangeDefined;
  int     portNum1;
  int     portNum2;
  int     lead0sPort1;
  int     lead0sPort2;

  int numBracketRangesDefined;
  int number1[MAX_BRACKETS_SUPPORTED];
  int numDigitsNum1[MAX_BRACKETS_SUPPORTED];
  int number2[MAX_BRACKETS_SUPPORTED];
  int numDigitsNum2[MAX_BRACKETS_SUPPORTED];
  int leading0sNum1[MAX_BRACKETS_SUPPORTED];
  int leading0sNum2[MAX_BRACKETS_SUPPORTED];
  int bracketGroupNum[MAX_BRACKETS_SUPPORTED];
} RegexBracketParseInfo_t;

typedef struct _RegExp {
	char			     	regexString[MAX_NODE_DESC_REG_EXPR];
	boolean                 isSyntaxValid;
	RegexBracketParseInfo_t regexInfo;

#ifdef __VXWORKS__
	regexp                  *regexpCompiled;
#else
	regmatch_t              groupArray[MAX_BRACKETS_SUPPORTED];
	regex_t                 regexCompiled;
#endif

	struct _RegExp   	    *next;
} RegExp_t;


// Virtual Fabric Group configuration
typedef struct  _DGConfig {
	char            name[MAX_VFABRIC_NAME];
	int             dg_index; // index into dg_config
	uint32_t        number_of_system_image_guids;
	cl_qmap_t       system_image_guid;
	uint32_t        number_of_node_guids;
	cl_qmap_t       node_guid;
	uint32_t        number_of_port_guids;
	cl_qmap_t       port_guid;
	uint32_t        number_of_node_descriptions;
	XmlNode_t      *node_description;
	RegExp_t       *reg_expr; // setup by SM
	uint32_t        number_of_included_groups;
	XmlIncGroup_t  *included_group;

	// Select
	uint8_t         select_all;
	uint8_t         select_self;
	uint8_t         select_swe0;
	uint8_t         select_all_mgmt_allowed;
	uint8_t         select_hfi_direct_connect;
	uint8_t         select_all_tfis;


	// NodeType
	uint8_t         node_type_fi;
	uint8_t         node_type_sw;
} DGConfig_t;

typedef struct _XmlAppMgid {
	char			mgid[MAX_VFABRIC_NAME];
} XmlAppMgid_t;

typedef struct _XmlAppMgidRng {
	char			range[MAX_VFABRIC_APP_ELEMENT];
} XmlAppMgidRng_t;

typedef struct _XmlAppMgidMsk {
	char			masked[MAX_VFABRIC_APP_ELEMENT];
} XmlAppMgidMsk_t;

// Virtual Fabric Application configuration
typedef struct 	_AppConfig {
	char			name[MAX_VFABRIC_NAME]; /* MUST BE FIRST FIELD FOR MAP */
	uint32_t		serviceIdMapSize;
	cl_qmap_t		serviceIdMap;
	uint32_t		serviceIdRangeMapSize;
	cl_qmap_t		serviceIdRangeMap;
	uint32_t		serviceIdMaskedMapSize;
	cl_qmap_t		serviceIdMaskedMap;
	uint32_t		number_of_mgids;
	XmlAppMgid_t	mgid[MAX_VFABRIC_APP_MGIDS];
	uint32_t		number_of_mgid_ranges;
	XmlAppMgidRng_t	mgid_range[MAX_VFABRIC_APP_MGIDS];
	uint32_t		number_of_mgid_maskeds;
	XmlAppMgidMsk_t	mgid_masked[MAX_VFABRIC_APP_MGIDS];
	uint32_t		number_of_included_apps;
	XmlNode_t		included_app[MAX_INCLUDED_APPS];
	uint8_t			select_sa;
	uint8_t			select_unmatched_sid;
	uint8_t			select_unmatched_mgid;
	uint8_t			select_pm;
} AppConfig_t;

// Virtual Fabric QOS Group configuration
typedef struct 	_QosConfig {
	char			name[MAX_VFABRIC_NAME];
	uint8_t			enable;
	uint8_t			qos_enable; /* For legacy support of VFs with QOS=0 */
	uint8_t			private_group;     /* For legacy support of VFs with QOS=0 */
	uint8_t			base_sl;
	uint8_t			base_sl_specified;
	uint8_t			resp_sl;
	uint8_t			resp_sl_specified;
	uint8_t			requires_resp_sl;
	uint8_t			mcast_sl;
	uint8_t			mcast_sl_specified;
	uint8_t			contains_mcast;
	uint8_t			flowControlDisable;
	uint8_t			percent_bandwidth;
	uint8_t			priority;
	uint8_t			pkt_lifetime_mult; /* Converted to power of 2 */
    uint32_t		hoqlife_qos;
    uint8_t			preempt_rank;
	uint32_t		num_implicit_vfs;
	uint32_t		num_vfs;
	uint8_t			pkt_lifetime_specified;
	uint8_t			hoqlife_specified;
} QosConfig_t;

// Application configuration
typedef struct _XMLApp {
	char            application[MAX_VFABRIC_NAME];
} XMLApp_t;

typedef struct _XMLMember {
	char member[MAX_VFABRIC_NAME];
	int  dg_index; // Index into dg_config;
} XMLMember_t;

// Virtual Fabric configuration
typedef struct 	_VFConfig {
	char			name[MAX_VFABRIC_NAME]; /* MUST BE FIRST FIELD FOR MAP */
	uint32_t		enable;
	uint32_t		standby;
	uint32_t		pkey;
	uint32_t		security;
	uint32_t		qos_index;
	char			qos_group[MAX_VFABRIC_NAME];
	uint32_t		number_of_full_members;
	XMLMember_t		full_member[MAX_VFABRIC_MEMBERS_PER_VF];
	uint32_t		number_of_limited_members;
	XMLMember_t		limited_member[MAX_VFABRIC_MEMBERS_PER_VF];
	uint32_t		number_of_applications;
	XMLApp_t 		application[MAX_VFABRIC_APPS_PER_VF];
	uint8_t			max_mtu_int;
	uint8_t			max_rate_int;
	uint8_t			qos_implicit;
	// Deprecated configuration items (see QOSConfig_t for new fields)
	uint8_t			qos_enable;
	uint8_t			base_sl;
	uint8_t			resp_sl;
	uint8_t			mcast_sl;
	uint8_t			flowControlDisable;
	uint8_t			percent_bandwidth;
	uint8_t			priority;
	uint8_t			pkt_lifetime_mult;
    uint8_t			preempt_rank;
    uint32_t		hoqlife_vf;
} VFConfig_t;

// Application Database Composite per FM
typedef struct 	_AppXmlConfig {
	uint32_t			appMapSize;
	cl_qmap_t			appMap;					// binary tree of apps by name
} AppXmlConfig_t;

// DeviceGroup Database Composite per FM
typedef struct 	_DGXmlConfig {
	uint32_t			number_of_dgs;
	DGConfig_t			*dg[MAX_VFABRIC_GROUPS];
} DGXmlConfig_t;

// QOSGroup Database Composite per FM
typedef struct 	_QosXmlConfig {
	uint32_t			number_of_qosgroups;
	QosConfig_t			*qosgroups[MAX_QOS_GROUPS];
} QosXmlConfig_t;

// VirtualFabrics Database Composite per FM
typedef struct 	_VFXmlConfig {
	uint32_t			number_of_vfs;
	VFConfig_t			*vf[MAX_CONFIGURED_VFABRICS];
	uint8_t				securityEnabled;				// Composite of all vf's
	uint8_t				qosEnabled;						// Composite of all vf's
} VFXmlConfig_t;

// Internal structures for internal application use for Virtual Fabric Configuration

// Internal Service ID's
typedef struct VFAppSid_t {
	uint64_t        	service_id;					// service_id or first service_id in range
													// if 0xfffffffffffffff then all apps
	uint64_t        	service_id_last;			// if 0 then then no range
	uint64_t        	service_id_mask;			// if 0xfffffffffffffff then no mask
} VFAppSid_t;

// Internal Multicast Group ID's
typedef struct _VFAppMgid {
	uint64_t			mgid[2];					// 128 bit MGID - upper word at index 0
	uint64_t			mgid_last[2];				// 128 bit MGID upper range - if 0 then
													// no range
	uint64_t			mgid_mask[2];				// 128 bit MGID mask
													// if 0xffffffffffffffffffffffffffffff
													// then no mask
} VFAppMgid_t;

// Internal Virtual Fabric Applications
typedef struct _VFApp {
	// Service ID list
	uint32_t			sidMapSize;
	cl_qmap_t			sidMap;						// Map of VFAppSid_t
	// Multicast Group list
	uint32_t			mgidMapSize;
	cl_qmap_t			mgidMap;					// Map of VFAppMgid_t

	// aggregate settings from application and included applications
	uint8_t				select_sa;					// boolean select SA - defaults to 0
	uint8_t				select_unmatched_sid;		// boolean select unmatched service id
													// defaults to 0
	uint8_t				select_unmatched_mgid;		// boolean select unmatched MGID
	uint8_t				select_pm;					// boolean select PM - defaults to 0


} VFApp_t;

// Internal rendering of Virtual Fabric Members
typedef struct _VFMem {
	// System Image GUID's
	uint32_t		sysGuidMapSize;					// number of System Image GUID entries
	cl_qmap_t		sysGuidMap;						// binary tree of System Image GUID's

	// Node GUID entries
	uint32_t		nodeGuidMapSize;				// number of Node GUID entries
	cl_qmap_t		nodeGuidMap;					// binary tree of Node GUID's

	// Port GUID entries
	uint32_t        portGuidMapSize;    			// number of Port GUID entries
	cl_qmap_t		portGuidMap;					// binary tree of Port GUID's

	// Node Description entries
	uint32_t        nodeDescMapSize;				// number of Node descriptions
	cl_qmap_t		nodeDescMap;					// binary tree of Node Description pointers

	// aggregate settings from group and included groups
	uint8_t			select_all;						// boolean select all - defaults to 0
	uint8_t			select_self;					// boolean select self - defaults to 0
	uint8_t			select_hfi_direct_connect;		// boolean select all nodes in b2b (hfi direct connect)
	uint8_t			select_swe0;					// boolean select SWE0 - defaults to 0
	uint8_t			select_all_mgmt_allowed; 		// boolean select all mgmt allowed - defaults to 0
	uint8_t			select_all_tfis; 	         	// boolean select all TFIs - defaults to 0

	// aggregate settings from group and included groups
	uint8_t			node_type_fi;					// boolean FI type - defaults to 0
	uint8_t			node_type_sw;					// boolean SW type - defaults to 0
} VFMem_t;

// Internal rendering of Default Groups
typedef struct _VFDg {
	uint8_t				def_mc_create;				// if undefined will default to 1
	uint8_t				prejoin_allowed;			// 0 = false, non-zero = true.
													// if undefined will default to 0
	uint32_t			def_mc_pkey;			 	// if undefined will default to
													// STL_DEFAULT_APP_PKEY
	uint8_t				def_mc_mtu_int;				// if undefined will default to 0xff
	uint8_t				def_mc_rate_int;			// if undefined will default to 0xff
	uint8_t				def_mc_sl;					// if undefined will default to 0xff
	uint32_t			def_mc_qkey;				// if undefined will default to 0x0
	uint32_t			def_mc_fl;					// if undefined will default to 0x0
	uint32_t			def_mc_tc;					// if undefined will default to 0x0

	uint32_t			mgidMapSize;
	cl_qmap_t			mgidMap;					// Map of VFAppMgid_t
	struct _VFDg		*next_default_group;		// pointer to next group
} VFDg_t;

// Internal rendering of Virtual Fabric configuration - if no Vitual Fabrics
// are configured or all Virtual Fabrics are Disabled, a default single
// Virtual Fabric will be created. Disabled Virtual Fabrics will not show
// up as a defined VirtualFabric within this structure.
typedef struct _VFabric {
	char			name[MAX_VFABRIC_NAME];		    // defaults to "Default"
	uint32_t		index;							// one will be uniquely assigned
	uint32_t		pkey;							// if 0xffffffff use default pkey
	uint32_t		qos_index;
	uint8_t			qos_implicit;
	uint8_t			added;							// For use on reconfiguration
	uint8_t			removed;						// For use on reconfiguration
	uint8_t			standby;
	uint8_t			security;						// defaults to 0 or false
	uint8_t			max_mtu_int;					// if 0xff then unlimited
	uint8_t			max_mtu_specified;
	uint8_t			max_rate_int;					// if 0xff then unlimited
	uint8_t			max_rate_specified;				// set by SM for use in queries
	VFApp_t			apps;							// application SID's and MGID's etc...
	uint32_t		number_of_full_members;
	XMLMember_t		full_member[MAX_VFABRIC_MEMBERS_PER_VF];
	uint32_t		number_of_limited_members;
	XMLMember_t		limited_member[MAX_VFABRIC_MEMBERS_PER_VF];
	uint32_t		number_of_default_groups;		// number of default multicast groups
	VFDg_t			*default_group;					// default group configuration linked list
	uint32_t		consistency_checksum;
	uint32_t		disruptive_checksum;
	uint32_t		overall_checksum;
} VF_t;

// Internal rendering of Composite Virtual Fabric configuration
typedef struct _SMVirtualFabricsInternal {
	uint32_t		number_of_vfs_all;
	uint32_t		number_of_qos_all;
	uint8_t			securityEnabled;				// setup by SM
	uint8_t			qosEnabled;						// setup by SM
	VF_t			v_fabric_all[MAX_ENABLED_VFABRICS];
	QosConfig_t		qos_all[MAX_QOS_GROUPS];
	DGXmlConfig_t	dg_config;
	uint32_t		consistency_checksum;
	uint32_t		disruptive_checksum;
	uint32_t		overall_checksum;
} VirtualFabrics_t;

// SM Dynamic Packet Lifetime configuration
typedef struct _SMDPLXmlConfig {
    uint32_t    dp_lifetime[10];
} SMDPLXmlConfig_t;

// SM Multicast DefaultGroup configuration
typedef struct _SMMcastDefGrp {
	uint32_t		def_mc_create;
	uint32_t		prejoin_allowed;
	char			virtual_fabric[MAX_VFABRIC_NAME];
	uint32_t		def_mc_pkey;
	uint8_t			def_mc_mtu_int;
	uint8_t			def_mc_rate_int;
	uint8_t			def_mc_sl;
	uint32_t		def_mc_qkey;
	uint32_t		def_mc_fl;
	uint32_t		def_mc_tc;
	uint32_t		number_of_mgids;
	XmlAppMgid_t	mgid[MAX_VFABRIC_DG_MGIDS];
	uint32_t		number_of_mgid_ranges;
	XmlAppMgidRng_t	mgid_range[MAX_VFABRIC_DG_MGIDS];  // May implement in future
	uint32_t		number_of_mgid_maskeds;
	XmlAppMgidMsk_t	mgid_masked[MAX_VFABRIC_DG_MGIDS];  // May implement in future
} SMMcastDefGrp_t;

// SM Multicast DefaultGroups
typedef struct _SMMcastDefGrpCfg {
	uint32_t			number_of_groups;
	SMMcastDefGrp_t		group[MAX_DEFAULT_GROUPS];	// list of default groups
} SMMcastDefGrpCfg_t;

// SM configuration for McastGrpMGidLimitMask
typedef struct _mcastGrpMGidLimitMask {
    char    	value[40];
} mcastGrpMGidLimitMask_t;

// SM configuration for McastGrpMGidLimitValue
typedef struct _mcastGrpMGidLimitValue {
    char    	value[40];
} mcastGrpMGidLimitValue_t;

// SM configuration of MLIDShared
typedef struct _SmMcastMlidShared {
	uint32_t					enable;
	mcastGrpMGidLimitMask_t		mcastGrpMGidLimitMaskConvert;
	mcastGrpMGidLimitValue_t	mcastGrpMGidLimitValueConvert;
	uint32_t					mcastGrpMGidLimitMax;
	uint32_t					mcastGrpMGidperPkeyMax;
} SmMcastMlidShared_t;

// SM MLIDShared Instances
typedef struct _SmMcastMlidShare {
	uint32_t					number_of_shared;
	SmMcastMlidShared_t			mcastMlid[MAX_SUPPORTED_MCAST_GRP_CLASSES_XML];
} SmMcastMlidShare_t;

// SM configuration for Multicast configuration
typedef struct _SMMcastConfig {
    uint32_t 	disable_mcast_check;
    uint32_t   	enable_pruning;
    uint32_t   	mcast_mlid_table_cap;
	char		mcroot_select_algorithm[STRING_SIZE];
	char		mcroot_min_cost_improvement[STRING_SIZE];
} SMMcastConfig_t;

// SM configuration for Link Policy
typedef struct _SMPolicyConfig {
	uint8_t	    	enabled;
	uint16_t		policy;
} SMPolicyConfig_t;

typedef struct _SMLinkPolicyXmlConfig {
    uint16_t				link_max_downgrade;
    SMPolicyConfig_t		width_policy;
    SMPolicyConfig_t		speed_policy;
} SMLinkPolicyXmlConfig_t;

// SM configuration for Preemption
typedef struct _SMPreemptionXmlConfig {
    uint32_t    small_packet;
    uint32_t    large_packet;
    uint32_t    preempt_limit;
} SMPreemptionXmlConfig_t;


// uniform switch congestion control settings
typedef struct _SmSwCongestionXmlConfig {
	uint8_t  victim_marking_enable;
	uint8_t  threshold;
	uint8_t  packet_size;
	uint8_t  cs_threshold;
	uint16_t cs_return_delay;
	uint32_t marking_rate;
} SmSwCongestionXmlConfig_t;

// uniform congestion control settings
typedef struct _SmCaCongestionXmlConfig {
	uint8_t  sl_based;
	uint8_t  increase;
	uint16_t timer;
	uint8_t  threshold;
	uint8_t  min;
	uint16_t  limit;
	uint32_t desired_max_delay;
} SmCaCongestionXmlConfig_t;

// uniform congestion control settings
typedef struct _SmCongestionXmlConfig {
	uint8_t enable;
	uint8_t debug;
	SmSwCongestionXmlConfig_t sw;
	SmCaCongestionXmlConfig_t ca;
} SmCongestionXmlConfig_t;


// Adaptive Routing control settings
typedef struct _SmAdaptiveRoutingXmlConfig {
	uint8_t enable;
	uint8_t debug;
	uint8_t lostRouteOnly;
	uint8_t algorithm;
	uint8_t	arFrequency;
	uint8_t threshold;
} SmAdaptiveRoutingXmlConfig_t;

#define MAX_TIER	10
typedef struct _SmFtreeRouting_t {
	uint8_t 	debug;
	uint8_t 	passthru;
	uint8_t 	converge;
	uint8_t		tierCount;				// height of the fat tree. edges are rank 0.
	uint8_t		fis_on_same_tier;		// indicates that all end nodes are at the bottom of the tree.
	XMLMember_t	coreSwitches;			// device group indicating core switches
	XMLMember_t	routeLast;				// device group indicating HFIs that should be routed last.
} SmFtreeRouting_t;


/*
 * Structure for Device Group Min Hop routing.
 *
 * MAX_DGROUTING_ORDER is arbitrarily capped at 8. This could be higher,
 * but raising it might cause a performance hit.
 */
#define MAX_DGROUTING_ORDER 8
typedef struct _SmDGRouting_t {
	uint8_t 	dgCount;
	XMLMember_t	dg[MAX_DGROUTING_ORDER];
} SmDGRouting_t;

/*
 * Structures for Enhanced Routing Control for Hypercube Routing, per switch.
 */
typedef uint32_t portMap_t;

static __inline__ void portMapSet(portMap_t *portMap, int port)
{
	portMap[port/(sizeof(portMap_t)*8)] |= (1u << port%(sizeof(portMap_t)*8));
}

static __inline__ portMap_t portMapTest(portMap_t *portMap, int port)
{
	return portMap[port/(sizeof(portMap_t)*8)] & (1u << port%(sizeof(portMap_t)*8));
}

typedef struct _SmSPRoutingPort {
	uint8_t			pport;
	uint8_t			vport;
	uint16_t		cost;
} SmSPRoutingPort_t;

typedef struct _SmSPRoutingCtrl {
	struct _SmSPRoutingCtrl	*next;
	XMLMember_t		switches;			// one or more switches
	portMap_t		pportMap[MAX_SWITCH_PORTS/(sizeof(portMap_t)*8) + 1];
	portMap_t		vportMap[MAX_SWITCH_PORTS/(sizeof(portMap_t)*8) + 1];
	SmSPRoutingPort_t *ports;
	uint16_t		portCount;
} SmSPRoutingCtrl_t;

typedef struct _SmHypercubeRouting_t {
	uint8_t				debug;
	XMLMember_t			routeLast;		// device group indicating HFIs that should be routed last.
	SmSPRoutingCtrl_t	*enhancedRoutingCtrl;
} SmHypercubeRouting_t;

#define MAX_TOROIDAL_DIMENSIONS 6
#define MAX_DOR_DIMENSIONS 20
#define DEFAULT_DOR_PORT_PAIR_WARN_THRESHOLD	5
#define DEFAULT_UPDN_MC_SAME_SPANNING_TREE	1
#define DEFAULT_ESCAPE_VLS_IN_USE			1
#define DEFAULT_FAULT_REGIONS_IN_USE		1

typedef enum {
	DOR_MESH,
	DOR_TORUS,
	DOR_PARTIAL_TORUS,
} DorTop_t;

typedef struct _SmPortPair {
	uint8_t				port1;
	uint8_t				port2;
} SmPortPair_t;

typedef struct _SmDimension {
	uint8_t				toroidal;
	uint8_t				length;
	uint8_t				portCount;
	uint8_t				created;
	SmPortPair_t		portPair[MAX_SWITCH_PORTS];
} SmDimension_t;

typedef struct _SmDorRouting {
	uint8_t				debug;
	uint8_t				overlayMCast;
	uint8_t				dimensionCount;
	uint8_t				numToroidal;
	uint8_t				routingSCs;
	uint32_t			warn_threshold;
	SmDimension_t		dimension[MAX_DOR_DIMENSIONS];
	DorTop_t			topology;
	uint8_t				escapeVLs;
	uint8_t				faultRegions;
	XMLMember_t			routeLast;				// device group indicating HFIs that should be routed last.
} SmDorRouting_t;

#define MAX_SM_APPLIANCES       5
typedef struct _SmAppliancesXmlConfig {
	uint8_t             enable;
    uint64_t            guids[MAX_SM_APPLIANCES];
} SmAppliancesXmlConfig_t;

typedef enum {
	FIELD_ENF_LEVEL_DISABLED 	= 0,
	FIELD_ENF_LEVEL_WARN 		= 1,
	FIELD_ENF_LEVEL_ENABLED 	= 2,
} FieldEnforcementLevel_t;

typedef struct _SmPreDefTopoFieldEnfXmlConfig{
	FieldEnforcementLevel_t nodeGuid;
	FieldEnforcementLevel_t nodeDesc;
	FieldEnforcementLevel_t portGuid;
	FieldEnforcementLevel_t undefinedLink;
} SmPreDefTopoFieldEnfXmlConfig_t;

typedef struct _SmPreDefTopoXmlConfig {
	uint8_t 	enabled;
	char 		topologyFilename[FILENAME_SIZE];
	uint32_t 	logMessageThreshold;
	SmPreDefTopoFieldEnfXmlConfig_t fieldEnforcement;
} SmPreDefTopoXmlConfig_t;

extern const char* SmPreDefFieldEnfToText(FieldEnforcementLevel_t fieldEnfLevel);

typedef enum {
	LID_STRATEGY_SERIAL,
	LID_STRATEGY_TOPOLOGY
} LidStrategy_t;


// SM configuration
typedef struct _SMXmlConfig {
	uint64_t	subnet_prefix;
	uint32_t	subnet_size;
	uint32_t	config_consistency_check_level;

	uint32_t	startup_retries;
	uint32_t	startup_stable_wait;
    uint64_t    sm_key;
    uint64_t    mkey;
    uint64_t    timer;
    uint32_t    IgnoreTraps;
    uint32_t    trap_hold_down;
    uint32_t    max_retries;
    uint32_t    rcv_wait_msec;
    uint32_t    min_rcv_wait_msec;
    uint32_t    master_ping_interval;
    uint32_t    master_ping_max_fail;
    uint32_t    topo_errors_threshold;
    uint32_t    topo_abandon_threshold;
    uint32_t    switch_lifetime_n2;
    uint32_t    hoqlife_n2;
    uint32_t    vl15FlowControlDisable;
	uint32_t	vl15_credit_rate;
    uint32_t    sa_resp_time_n2;
    uint32_t    sa_packet_lifetime_n2;
    uint32_t    vlstall;
    uint32_t    db_sync_interval;
	uint32_t    mc_dos_threshold;
	uint32_t    mc_dos_action;
	uint32_t    mc_dos_interval;
    uint32_t    trap_threshold;
	uint32_t	trap_threshold_min_count;
    uint32_t    node_appearance_msg_thresh;
    uint32_t    spine_first_routing;
    uint32_t    shortestPathBalanced;
    uint32_t    lmc;
    uint32_t    lmc_e0;
	char		routing_algorithm[STRING_SIZE];
	uint32_t	path_selection;
	uint32_t	queryValidation;
	uint32_t	enforceVFPathRecs;	// Default to Enable to limit pathrecord scope to VFs
									// otherwise, use PKEY as scope limit.
	uint32_t	sma_batch_size;
	uint32_t	max_parallel_reqs;
 	uint32_t	check_mft_responses;
	uint32_t	min_supported_vls;
	uint64_t	cumulative_timeout_limit;
	uint32_t	max_fixed_vls;

	uint64_t	non_resp_tsec;
	uint32_t	non_resp_max_count;

	uint32_t	monitor_standby_enable;
	uint32_t	loopback_mode; 		// disable duplicate portguid checking, allowing loopback fabrics
	uint32_t	max_supported_lid;
	uint32_t	force_rebalance;
	uint32_t	use_cached_node_data;
	uint32_t	use_cached_hfi_node_data;

    SMLinkPolicyXmlConfig_t hfi_link_policy;
    SMLinkPolicyXmlConfig_t isl_link_policy;
    SMPreemptionXmlConfig_t preemption;

	SmCongestionXmlConfig_t congestion;


	SmDorRouting_t smDorRouting;

	SmAdaptiveRoutingXmlConfig_t adaptiveRouting;

	uint32_t	sma_spoofing_check;	// used to enable/disable usage of SMA security checking
	uint32_t	NoReplyIfBusy;				// Normally when an SA query cannot be processed because
											// a context is temporarily not available or an error is
											// detected which may be resolved when an im progress sweep
											// finishes, a busy status will be returned to the requester.
											//
											// If the new configuration option NoReplyIfBusy is set to 1,
											// rather than returning the busy status, no status will
											// be returned.  The behaviour will be as if the MAD request
											// were lost.
											//
											// If NoReplyIfBusy is set to 0 or is not in the configuration
											// file, the behavior of SM will be the same as if this change
											// had not been made.

	/* STL EXTENSIONS */
	uint32_t	lft_multi_block;			// # of LFT blocks per MAD. Valid range is 1-31.
	uint32_t	use_aggregates;				// 0/1 - if true, combine MADs where possible.
	uint32_t    optimized_buffer_control;	// 0/1 - if true, use multi-port and uniform buffer ctrl
											// MADs to program switch ports.
    SmAppliancesXmlConfig_t appliances;     // List of node GUIDs associated with appliance nodes
	SmPreDefTopoXmlConfig_t preDefTopo; 	// Pre-defined topology verification options and field enforcement

	uint32_t	minSharedVLMem;				// A specification (percentage) for minimum VL shared memory.
	uint32_t	dedicatedVLMemMulti;		// A multiplier increasing the dedicated memory per VL
	int32_t		wireDepthOverride;			// Value (bytes) to override the wire depth,
											//     -1 means use from Portinfo, 0 means no wire depth
	int32_t		replayDepthOverride;		// Value (bytes) to override the replay buffer depth, -1 means use from Portinfo
											//     -1 means use from Portinfo, 0 means no replay depth

	uint8_t		cableInfoPolicy;			// 0 means no CI cache, 1 means assume CI is identical for both ends of a cable,
											// 2 means do not assume CI is identical for both ends.
	uint32_t 	forceAttributeRewrite; 		// Used to force the SM to rewrite all attributes upon resweep
											// 0 is disabled (default), 1 is enabled
    uint32_t    timerScalingEnable;         // 0 is disabled (default), when enabled - HOQ and SLL are potentially modified.
	uint32_t 	defaultPortErrorAction; 	// Bitfield representing default PortInfo PortErrorAction(s) to set for each port
	uint32_t 	skipAttributeWrite; 		// Bitfield indicating attributes to be skipped for updating (debug)
	uint32_t 	switchCascadeActivateEnable;// 0 - Disabled, 1 - Enable cascade activation using IsActiveOptimizeEnable
	uint32_t 	neighborNormalRetries; 		// Number of retries when a port fails to go ACTIVE based on neighbor state (NeighborNormal flag)
	uint8_t		terminateAfter;				// Undocumented setting. 0 - Disabled, #>0 - Terminate after # sweeps.
	uint32_t 	portBounceLogLimit; 		// Number of port bounce log messages to show before suppressing the rest.
	uint32_t	neighborFWAuthenEnable;     // used to enable/disable usage of NeighborFWAuthenBypass checking
    uint32_t    SslSecurityEnabled;
	char		SslSecurityDir[FILENAME_SIZE];
	char		SslSecurityFmCertificate[FILENAME_SIZE];
	char		SslSecurityFmPrivateKey[FILENAME_SIZE];
	char		SslSecurityFmCaCertificate[FILENAME_SIZE];
    uint32_t    SslSecurityFmCertChainDepth;
	char		SslSecurityFmDHParameters[FILENAME_SIZE];
    uint32_t    SslSecurityFmCaCRLEnabled;
    char        SslSecurityFmCaCRL[FILENAME_SIZE];

	uint32_t	consistency_checksum;		// used for checking SM consistency in fabric
	uint32_t	overall_checksum;			// used to determine if SM config needs to be re-read
	uint32_t	disruptive_checksum;		// checksum of parameters that are disruptive to change

	SmFtreeRouting_t ftreeRouting;
	SmDGRouting_t dgRouting;
	SmHypercubeRouting_t hypercubeRouting;

	char        name[MAX_VFABRIC_NAME];
	uint32_t	start;
    uint32_t    hca;
    uint32_t	port;
    uint64_t    port_guid;

    uint32_t    lid;
	uint32_t	sa_rmpp_checksum;
	uint32_t	dynamic_port_alloc;

	uint32_t	sm_debug_perf;
	uint32_t	sa_debug_perf;
	uint32_t	sm_debug_vf;
	uint32_t	sm_debug_routing;
 	uint32_t	sm_debug_lid_assign;
	uint32_t	trap_log_suppress_trigger_interval;
	uint32_t	debug_jm;

    uint32_t	priority;
    uint32_t	elevated_priority;
	char		CoreDumpLimit[STRING_SIZE]; // inherited from Common FM setting
	char		CoreDumpDir[FILENAME_SIZE]; // inherited from Common FM setting
    uint32_t	debug;
    uint32_t	debug_rmpp;
    uint32_t    log_level;
	char		log_file[LOGFILE_SIZE];
	uint32_t	syslog_mode;
	char		syslog_facility[STRING_SIZE];
    FmParamU32_t    log_masks[VIEO_LAST_MOD_ID+1];

	uint32_t	loop_test_on;
	uint32_t	loop_test_fast_mode;
	uint32_t	loop_test_packets;

	char		dumpCounters[LOGFILE_SIZE];	// Undocumented setting. Null - Disabled. Not Null - dump performance counters after each sweep.
	uint32_t	lid_strategy;
	uint32_t	P_Key_8B;
	uint32_t	P_Key_10B;
} SMXmlConfig_t;

typedef struct _XMLMonitor {
	char			monitor[MAX_VFABRIC_NAME];
	uint16_t		dg_Index;
} XMLMonitor_t;
//Pm PortGroups
typedef struct PmPortGroupXmlConfig {
	uint8_t 		Enabled;
	char			Name[STL_PM_GROUPNAMELEN+1];
	XMLMonitor_t	Monitors[STL_PM_MAX_DG_PER_PMPG];
} PmPortGroupXmlConfig_t;

// Pm Thresholds
typedef struct _PmThresholdsXmlConfig {
	uint32_t Integrity;
	uint32_t Congestion;
	uint32_t SmaCongestion;
	uint32_t Bubble;
	uint32_t Security;
	uint32_t Routing;
} PmThresholdsXmlConfig_t;

// Pm ThresholdsExceededMsgLimit
typedef struct _PmThresholdsExceededMsgLimitXmlConfig {
	uint32_t Integrity;
	uint32_t Congestion;
	uint32_t SmaCongestion;
	uint32_t Bubble;
	uint32_t Security;
	uint32_t Routing;
} PmThresholdsExceededMsgLimitXmlConfig_t;

// Pm IntegrityWeights
typedef struct _PmIntegrityWeightsXmlConfig {
	uint8_t LocalLinkIntegrityErrors;
	uint8_t PortRcvErrors;
	uint8_t ExcessiveBufferOverruns;
	uint8_t LinkErrorRecovery;
	uint8_t LinkDowned;
	uint8_t UncorrectableErrors;
	uint8_t FMConfigErrors;
	uint8_t LinkQualityIndicator;
	uint8_t LinkWidthDowngrade;
} PmIntegrityWeightsXmlConfig_t;

// Pm CongestionWeights
typedef struct _PmCongestionWeightsConfig {
	uint8_t PortXmitWait;
	uint8_t SwPortCongestion;
	uint8_t PortRcvFECN;
	uint8_t PortRcvBECN;
	uint8_t PortXmitTimeCong;
	uint8_t PortMarkFECN;
} PmCongestionWeightsXmlConfig_t;

typedef struct _PmResolutionXmlConfig {
	uint64_t LocalLinkIntegrity;
	uint32_t LinkErrorRecovery;
} PmResolutionXmlConfig_t;

typedef struct _PmShortTermHistoryXmlConfig {
	uint8_t		enable;
	char		StorageLocation[FILENAME_SIZE];
	uint32_t	totalHistory;
	uint32_t	imagesPerComposite;
	uint64_t	maxDiskSpace;
	uint8_t		compressionDivisions;
} PmShortTermHistoryXmlConfig_t;

// PM configuration
typedef struct _PMXmlConfig {
	uint32_t	subnet_size;
	uint32_t	config_consistency_check_level;

    uint16_t    sweep_interval;
    uint32_t    timer;
    uint8_t	    ErrorClear;
    uint8_t	    ClearDataXfer;
    uint8_t	    Clear64bit;
    uint8_t	    Clear32bit;
    uint8_t	    Clear8bit;
	uint8_t 	process_hfi_counters;
    uint8_t	    process_vl_counters;
    uint32_t	MaxRetries;
    uint32_t	RcvWaitInterval;
    uint32_t	MinRcvWaitInterval;
    uint32_t	SweepErrorsLogThreshold;
    uint32_t	MaxParallelNodes;
    uint32_t	PmaBatchSize;
    uint32_t    freeze_frame_lease;
    uint32_t    total_images;
    uint32_t    freeze_frame_images;
    uint32_t    max_clients;
    uint32_t    history_file_update_rate;
    uint16_t    image_update_interval;
	PmThresholdsXmlConfig_t thresholds;
	PmIntegrityWeightsXmlConfig_t integrityWeights;
	PmCongestionWeightsXmlConfig_t congestionWeights;
	PmResolutionXmlConfig_t resolution;
	uint8_t		number_of_pm_groups;
	PmPortGroupXmlConfig_t pm_portgroups[STL_PM_MAX_CUSTOM_PORT_GROUPS];
	PmShortTermHistoryXmlConfig_t shortTermHistory;
    uint32_t    SslSecurityEnabled;
	char		SslSecurityDir[FILENAME_SIZE];
	char		SslSecurityFmCertificate[FILENAME_SIZE];
	char		SslSecurityFmPrivateKey[FILENAME_SIZE];
	char		SslSecurityFmCaCertificate[FILENAME_SIZE];
    uint32_t    SslSecurityFmCertChainDepth;
	char		SslSecurityFmDHParameters[FILENAME_SIZE];
    uint32_t    SslSecurityFmCaCRLEnabled;
    char        SslSecurityFmCaCRL[FILENAME_SIZE];

	uint32_t	consistency_checksum;
	uint32_t	overall_checksum;
	uint32_t	disruptive_checksum;		// checksum of parameters that are disruptive to change

	char        name[MAX_VFABRIC_NAME];
	uint32_t	start;
    uint32_t    hca;
    uint32_t	port;
    uint64_t    port_guid;

    uint32_t   	priority;
    uint32_t   	elevated_priority;
    uint32_t   	debug;
    uint32_t	debug_rmpp;
    uint32_t    log_level;
	char		log_file[LOGFILE_SIZE];
	uint32_t	syslog_mode;
	char		syslog_facility[STRING_SIZE];
    FmParamU32_t log_masks[VIEO_LAST_MOD_ID+1];

	PmThresholdsExceededMsgLimitXmlConfig_t thresholdsExceededMsgLimit;

} PMXmlConfig_t;

// FE configuration
typedef struct _FEXmlConfig {
	uint32_t	subnet_size;
	uint32_t	startup_retries;
	uint32_t	startup_stable_wait;

	uint32_t 	manager_check_rate;
    uint32_t   	login;
	uint32_t	window;
    uint32_t    SslSecurityEnabled;
	char		SslSecurityDir[FILENAME_SIZE];
	char		SslSecurityFmCertificate[FILENAME_SIZE];
	char		SslSecurityFmPrivateKey[FILENAME_SIZE];
	char		SslSecurityFmCaCertificate[FILENAME_SIZE];
    uint32_t    SslSecurityFmCertChainDepth;
	char		SslSecurityFmDHParameters[FILENAME_SIZE];
    uint32_t    SslSecurityFmCaCRLEnabled;
    char        SslSecurityFmCaCRL[FILENAME_SIZE];

	char        name[MAX_VFABRIC_NAME + 4]; // Padded for the "_fe" suffix.
	uint32_t	start;
    uint32_t    hca;
    uint32_t	port;
    uint64_t    port_guid;

    uint32_t    listen;

	char		CoreDumpLimit[STRING_SIZE];
	char		CoreDumpDir[FILENAME_SIZE];
    uint32_t   	debug;
    uint32_t	debug_rmpp;
    uint32_t   	log_level;
	char		log_file[LOGFILE_SIZE];
	uint32_t	syslog_mode;
	char		syslog_facility[STRING_SIZE];
    FmParamU32_t log_masks[VIEO_LAST_MOD_ID+1];

	uint32_t    trap_count;
	uint16_t    trap_nums[FE_MAX_TRAP_SUBS];
} FEXmlConfig_t;

// FM configuration (Shared)
typedef struct _FMXmlConfig {
	uint64_t	subnet_prefix;
    uint32_t	subnet_size;
	uint32_t	startup_retries;
	uint32_t	startup_stable_wait;
	uint32_t	config_consistency_check_level;
    uint32_t    SslSecurityEnabled;
	char		SslSecurityDir[FILENAME_SIZE];
	char		SslSecurityFmCertificate[FILENAME_SIZE];
	char		SslSecurityFmPrivateKey[FILENAME_SIZE];
	char		SslSecurityFmCaCertificate[FILENAME_SIZE];
    uint32_t    SslSecurityFmCertChainDepth;
	char		SslSecurityFmDHParameters[FILENAME_SIZE];
    uint32_t    SslSecurityFmCaCRLEnabled;
    char        SslSecurityFmCaCRL[FILENAME_SIZE];

	char		fm_name[MAX_VFABRIC_NAME];
	uint32_t	start;
    uint32_t    hca;
    uint32_t	port;
    uint64_t    port_guid;

    uint32_t   	priority;
    uint32_t   	elevated_priority;
	char		CoreDumpLimit[STRING_SIZE];
	char		CoreDumpDir[FILENAME_SIZE];
    uint32_t   	debug;
    uint32_t	debug_rmpp;
    uint32_t   	log_level;
	char		log_file[LOGFILE_SIZE];
	uint32_t	syslog_mode;
	char		syslog_facility[STRING_SIZE];
    FmParamU32_t log_masks[VIEO_LAST_MOD_ID+1];
	// FM config doesn't have checksums because all the data is contained in SM, PM, or FE configs
} FMXmlConfig_t;




// FM instance
typedef struct _FMXmlInstance {
	FMXmlConfig_t			fm_config;
	SMXmlConfig_t			sm_config;
	SMDPLXmlConfig_t		sm_dpl_config;
	SMMcastConfig_t			sm_mc_config;
	SmMcastMlidShare_t		sm_mls_config;
	SMMcastDefGrpCfg_t      sm_mdg_config;
	VFXmlConfig_t			vf_config;
	QosXmlConfig_t			qos_config;
	DGXmlConfig_t			dg_config;
	AppXmlConfig_t			app_config;
	PMXmlConfig_t			pm_config;
	FEXmlConfig_t			fe_config;


} FMXmlInstance_t;

// XML debug
typedef struct _XmlDebug {
	uint32_t	xml_all_debug;
	uint32_t	xml_vf_debug;
	uint32_t	xml_sm_debug;
	uint32_t	xml_fe_debug;
	uint32_t	xml_pm_debug;
	uint32_t	xml_parse_debug;
} XmlDebug_t;

// Composite FM configuration
typedef struct _FMXmlCompositeConfig {
	FMXmlInstance_t			*fm_instance_common;
	FMXmlInstance_t			*fm_instance[MAX_INSTANCES];
	int						num_instances;
	XmlDebug_t				xmlDebug;
} FMXmlCompositeConfig_t;

// parseFmConfig:
// fm - instance to parse
// full - Parse all the instances in the config file (ignore the fm parameter)
// preverify - run renderVirtualFabics on config to verify VFs
//             (not necessary if renderVirtualFabrics will be called)
extern FMXmlCompositeConfig_t* parseFmConfig(char *filename, uint32_t flags, uint32_t fm, uint32_t full, uint32_t preverify, uint32_t embedded);
extern void releaseXmlConfig(FMXmlCompositeConfig_t *config, uint32_t full);
extern VirtualFabrics_t* renderVirtualFabricsConfig(uint32_t fm, FMXmlCompositeConfig_t *config, 
	IXmlParserPrintMessage printError, IXmlParserPrintMessage printWarning);
extern VirtualFabrics_t* reRenderVirtualFabricsConfig(uint32_t fm, VirtualFabrics_t *oldVfsip, FMXmlCompositeConfig_t *config,
	IXmlParserPrintMessage printError, IXmlParserPrintMessage printWarning);


extern void releaseVirtualFabricsConfig(VirtualFabrics_t *vfsip);
extern void initXmlPoolGetCallback(void *function);
extern void initXmlPoolFreeCallback(void *function);
extern uint32_t xml_compute_pool_size(uint8_t full);
extern uint8_t verifyFmConfig(char* filename, uint32_t flags);
int generateDefaultXmlConfig(uint8_t cli);
extern VF_t* findVfPointer(VirtualFabrics_t*, char*);
extern void fmClearConfig(FMXmlConfig_t *fmp);
extern void pmClearConfig(PMXmlConfig_t *pmp);
extern void feClearConfig(FEXmlConfig_t *fep);
extern void smScrubConfig(SMXmlConfig_t *smp);
extern boolean smCopyConfig(SMXmlConfig_t *dst,SMXmlConfig_t *src);
extern void smClearConfig(SMXmlConfig_t *smp);
#ifndef __VXWORKS__
extern int getFacility(char* name, uint8_t test);
#endif
extern void smShowConfig(SMXmlConfig_t *smp, SMDPLXmlConfig_t *dplp, SMMcastConfig_t *mcp, SmMcastMlidShare_t *mlsp);
extern void pmShowConfig(PMXmlConfig_t *pmp);
extern void feShowConfig(FEXmlConfig_t *fep);
extern int getXMLConfigData(uint8_t *buffer, uint32_t bufflen, uint32_t *filelen);
extern int putXMLConfigData(uint8_t *buffer, uint32_t filelen);
extern int copyCompressXMLConfigFile(char *src, char *dst);
extern void freeDgInfo(DGXmlConfig_t *dg);
extern FSTATUS MatchImplicitMGIDtoVF(SMMcastDefGrp_t *mdgp, VirtualFabrics_t *vf_config);
extern FSTATUS MatchExplicitMGIDtoVF(SMMcastDefGrp_t *mdgp, VirtualFabrics_t *vf_config, boolean update_active_vfabrics, int enforceVFPathRecs);

// Export XML Memory mapping functions for the SM to use with the Topology lib on ESM
#ifdef __VXWORKS__
extern void* getParserMemory(size_t size);
extern void* reallocParserMemory(void* ptr, size_t size);
extern void freeParserMemory(void* ptr);
#endif

// Macro for proper typecasting of pointers into the cl_map_item key
#ifdef __VXWORKS__
#define XML_QMAP_CHAR_CAST	(char*)(uint32)
#define XML_QMAP_U64_CAST	(uint64)(uint32)
#define XML_QMAP_VOID_CAST	(void*)(uint32)
#define XML_QMAP_U8_CAST	(uint8_t*)(uint32)
#else
#define XML_QMAP_CHAR_CAST	(char*)
#define XML_QMAP_U64_CAST	(uint64)
#define XML_QMAP_VOID_CAST	(void*)
#define XML_QMAP_U8_CAST	(uint8_t*)
#endif

#define for_all_qmap_item(map, item) \
	 for(item = cl_qmap_head(map); \
	     item != cl_qmap_end(map); \
		 item = cl_qmap_next(item))

#define for_all_qmap_type(map, item, obj, type) \
	 for(item = cl_qmap_head(map), obj = type cl_qmap_key(item); \
	     item != cl_qmap_end(map); \
		 item = cl_qmap_next(item), obj = type cl_qmap_key(item))

#define for_all_qmap_u64(map, item, obj) for_all_qmap_type(map, item, obj, XML_QMAP_U64_CAST)
#define for_all_qmap_ptr(map, item, obj) for_all_qmap_type(map, item, obj, XML_QMAP_VOID_CAST)

#endif // _FM_XML_H_
