/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#if defined(CHECK_HEADERS)

#ifndef __STL_SD_H__
#warning FIX ME!!! Your includes should use the stl_sd.h header and not the ib_sd.h header for STL builds
#endif

#endif

#ifndef _IBA_IB_SD_H_
#define _IBA_IB_SD_H_

/* IB Subnet Data Interface
 * The subnet data interface provides a simplied interface to the SM/SA and
 * automates queries, sets and deletes for commonly required information
 * from the SM/SA.
 *
 * The subnet data interface automatically handles the following:
 * - retries (client configurable)
 * - timeouts (client configurable)
 * - RMPP response coallessing and inter-record padding
 * - extraction of selected fields from a query to provide a concise response
 * 		to client
 * - multi-tiered queries (get paths to a node, etc)
 *
 * New drivers/applications should use the iba_* functions.
 * They can now be called directly.
 */

#include <iba/public/datatypes.h>
#include <iba/stl_types.h>
#include <iba/stl_mad.h>
#ifdef IB_STACK_OPENIB
#include <iba/stl_sa.h>
#else

#if defined(VXWORKS)
#include <iba/ib_sa_records.h>
#ifdef BUILD_CM
#include <iba/ib_cm.h>
#endif

#else
#include <iba/ib_cm.h>
#endif

#endif

#ifdef __cplusplus
extern "C" {
#endif

#define TIMEOUT_INFINITE            0
#define RETRYCOUNT_INVALID			0xffffffff	/* used internally to flag */
												/* lack of control parameters */

/* default command control parameters, used if not specified by client for
 * open session nor for query/fabric operation calls
 */
#define DEFAULT_SD_TIMEOUT			20000	/* 20 Seconds */
#define DEFAULT_SD_RETRY_COUNT		3

/* ClientDebug Flags, affects level of output to debug log for given client */
#define SD_DBG_SILENT				0x00000000
#define SD_DBG_ERROR				0x00000001
#define SD_DBG_INFO					0x00000002
#define SD_DBG_TRACE				0x00000004
#define DBG_SILENT					SD_DBG_SILENT/* DBG_SILENT is depricated */
#define DBG_ERROR					SD_DBG_ERROR /* DBG_ERROR is depricated*/
#define DBG_INFO					SD_DBG_INFO	 /* DBG_INFO is depricated */
#define DBG_TRACE					SD_DBG_TRACE /* DBG_TRACE is depricated*/
	
typedef void *CLIENT_HANDLE;

/* parameters for query and fabric operation error handling */
typedef struct _COMMAND_CONTROL_PARAMETERS  {
    uint32 RetryCount;	/* 0=no retries, >=1 # retries if initial attempt fails */
						/* retries are only performed upon timeout */
						/* requests with a failed response are not retried */
    uint32 Timeout;		/* in milliseconds */
} COMMAND_CONTROL_PARAMETERS, *PCOMMAND_CONTROL_PARAMETERS;

#define SD_OPTION_CACHED	0x00000001 /* No new query if info already available */

/* per client selection of default parameters for all queries */
typedef struct _CLIENT_CONTROL_PARAMETERS {
    COMMAND_CONTROL_PARAMETERS ControlParameters;
    uint32 ClientDebugFlags;        
    uint32 OptionFlags;
} CLIENT_CONTROL_PARAMETERS, *PCLIENT_CONTROL_PARAMETERS;

/* multicast flag values */
#define MC_FLAG_NONE             0x0000
#define MC_FLAG_WANT_UNAVAILABLE 0x0001 /* want callback when group becomes unavailable */

/* multicast group state values */
typedef enum
{
	MC_GROUP_STATE_REQUEST_JOIN,
	MC_GROUP_STATE_REQUEST_LEAVE,
	MC_GROUP_STATE_JOIN_FAILED,
	MC_GROUP_STATE_AVAILABLE,
	MC_GROUP_STATE_UNAVAILABLE
} MC_GROUP_STATE;

/* ----------------------------------------------------------------------------
 * QueryFabricInformation
 * Data structures used to perform Queries of the SA/SM via one or more
 * Get or GetTable RMPP sequences
 */

/* QueryFabricInformation combinations Supported:
 *
 * InputType		OutputType		Query performed
 * ---------        ----------      ---------------
 * NoInput			Lids			SA GetTable all NodeRecords
 * NoInput			SystemImageGuid	SA GetTable all NodeRecords
 * NoInput			NodeGuids		SA GetTable all NodeRecords
 * NoInput			PortGuids		SA GetTable all NodeRecords
 * NoInput			NodeDesc		SA GetTable all NodeRecords
 * NoInput			NodeRecord		SA GetTable all NodeRecords
 * NoInput			PortInfoRecord	SA GetTable all PortInfoRecords
 * NoInput			SMInfoRecord	SA GetTable all SMInfoRecords
 * NoInput			LinkRecord		SA GetTable all LinkRecords
 * NoInput			ServiceRecord	SA GetTable all ServiceRecords
 * NoInput			McMemberRecord	SA GetTable all McMemberRecords
 * NoInput			InformInfoRecord SA GetTable all InformInfoRecords
 *
 * NoInput			PathRecord		SA GetTable all NodeRecords
 * 									followed by a SA GetTable PathRecords
 * 									from the local PortGuid and each
 * 									Remote PortGuid reported in NodeRecords
 *
 * NoInput			SwitchInfoRecord SA GetTable all SwitchInfoRecords
 * NoInput			LinearFDBRecord	SA GetTable all LinearFDBRecords
 * NoInput			RandomFDBRecord	SA GetTable all RandomFDBRecords
 * NoInput			MCastFDBRecord	SA GetTable all MCastFDBRecords
 * NoInput			VLArbTableRecord SA GetTable all VLArbTableRecords
 * NoInput			PKeyTableRecord SA GetTable all PKeyTableRecords
 * NoInput			GuidInfoRecord	SA GetTable all GuidInfoRecords
 *
 * NodeType			Lids			SA GetTable NodeRecords of NodeType
 * NodeType			SystemImageGuid	SA GetTable NodeRecords of NodeType
 * NodeType			NodeGuids		SA GetTable NodeRecords of NodeType
 * NodeType			PortGuids		SA GetTable NodeRecords of NodeType
 * NodeType			NodeDesc		SA GetTable NodeRecords of NodeType
 * NodeType			NodeRecord		SA GetTable NodeRecords of NodeType
 *
 * NodeType			PathRecord		SA GetTable NodeRecords of NodeType
 * 									followed by a SA GetTable PathRecords
 * 									from the local PortGuid and each
 * 									Remote PortGuid reported in NodeRecords
 *
 * SystemImageGuid	Lids			SA GetTable NodeRecords with SystemImageGuid
 * SystemImageGuid	SystemImageGuid	SA GetTable NodeRecords with SystemImageGuid
 * SystemImageGuid	NodeGuids		SA GetTable NodeRecords with SystemImageGuid
 * SystemImageGuid	PortGuids		SA GetTable NodeRecords with SystemImageGuid
 * SystemImageGuid	NodeDesc		SA GetTable NodeRecords with SystemImageGuid
 * SystemImageGuid	NodeRecord		SA GetTable NodeRecords with SystemImageGuid
 *
 * SystemImageGuid	PathRecord		SA GetTable NodeRecords with SystemImageGuid
 * 									followed by a SA GetTable PathRecords
 * 									from the local PortGuid and each
 * 									Remote PortGuid reported in NodeRecords
 *
 * NodeGuid			Lids			SA GetTable NodeRecords with NodeGuid		
 * NodeGuid			SystemImageGuid	SA GetTable NodeRecords with NodeGuid		
 * NodeGuid			NodeGuids		SA GetTable NodeRecords with NodeGuid		
 * NodeGuid			PortGuids		SA GetTable NodeRecords with NodeGuid		
 * NodeGuid			NodeDesc		SA GetTable NodeRecords with NodeGuid		
 * NodeGuid			NodeRecord		SA GetTable NodeRecords with NodeGuid		
 *
 * NodeGuid			PathRecord		SA GetTable NodeRecords with NodeGuid		
 * 									followed by a SA GetTable PathRecords
 * 									from the local PortGuid and each
 * 									Remote PortGuid reported in NodeRecords
 *
 * PortGuid			Lids			SA GetTable NodeRecords with PortGuid		
 * PortGuid			SystemImageGuid	SA GetTable NodeRecords with PortGuid		
 * PortGuid			NodeGuids		SA GetTable NodeRecords with PortGuid		
 * PortGuid			PortGuids		SA GetTable NodeRecords with PortGuid		
 * PortGuid			NodeDesc		SA GetTable NodeRecords with PortGuid		
 *
 * PortGuid			NodeRecord		SA GetTable NodeRecords with PortGuid		
 * 									followed by a SA GetTable PathRecords
 * 									from the local PortGuid and each
 * 									Remote PortGuid reported in NodeRecords
 *
 * PortGuid			PathRecord		SA GetTable PathRecords
 * 									from the local PortGuid to the
 * 									given Remote PortGuid
 *
 * PortGuid			ServiceRecord	SA GetTable ServiceRecords
 * 									provided by given PortGuid
 *
 * PortGuid			McMemberRecord	SA GetTable McMemberRecords
 * 									for the given PortGuid
 *
 * PortGuid			TraceRecord		SA GetTraceTable TraceRecords
 * 									from the local PortGuid to the
 * 									given Remote PortGuid
 *
 * PortGuid			InformInfoRecord SA GetTable InformInfoRecords
 * 									for the given PortGuid
 *
 * PortGid			PathRecord		SA GetTable PathRecords
 * 									from the local PortGuid to the
 * 									given Remote PortGid
 *
 * PortGid			ServiceRecord	SA GetTable ServiceRecords
 * 									provided by given PortGid
 *
 * PortGid			McMemberRecord	SA GetTable McMemberRecords
 * 									for the given PortGid
 *
 * PortGid			InformInfoRecord SA GetTable InformInfoRecords
 * 									for the given PortGid
 *
 * PortGid			TraceRecord		SA GetTraceTable TraceRecords
 * 									from the local PortGuid to the
 * 									given Remote PortGid
 *
 * McGid			McMemberRecord	SA GetTable McMemberRecords
 * 									for the given Multicast Gid
 *
 * PortGuidPair		PathRecord		SA GetTable PathRecords
 * 									from the given source PortGuid to the
 * 									given destination PortGuid
 *
 * PortGuidPair		TraceRecord		SA GetTraceTable TraceRecords
 * 									from the given source PortGuid to the
 * 									given destination PortGuid
 *
 * GidPair			PathRecord		SA GetTable PathRecords
 * 									from the given source Gid to the
 * 									given destination Gid
 *
 * GidPair			TraceRecord		SA GetTraceTable TraceRecords
 * 									from the given source Gid to the
 * 									given destination Gid
 *
 * PathRecord		PathRecord		SA GetTable PathRecords with PathRecord
 * 									and component mask as given
 *
 * PathRecord		TraceRecord		SA GetTraceTable TraceRecords with PathRecord
 * 									and component mask as given
 *
 * PortGuidList		PathRecord		SA GetTable MultiPathRecords
 * 									from the given source PortGuids to the
 * 									given destination PortGuids
 *
 * GidList			PathRecord		SA GetTable MultiPathRecords
 * 									from the given source Gids to the
 * 									given destination Gids
 *
 * MultiPathRecord	PathRecord		SA GetTable MultiPathRecords with
 * 									MultiPathRecord and component mask as given
 * 									
 * Lid				Lids			SA GetTable NodeRecords with Lid		
 * Lid				SystemImageGuid	SA GetTable NodeRecords with Lid		
 * Lid				NodeGuids		SA GetTable NodeRecords with Lid		
 * Lid				PortGuids		SA GetTable NodeRecords with Lid		
 * Lid				NodeDesc		SA GetTable NodeRecords with Lid		
 * Lid				NodeRecord		SA GetTable NodeRecords with Lid		
 *
 * Lid				PathRecord		SA GetTable NodeRecords with Lid		
 * 									followed by a SA GetTable PathRecords
 * 									from the local PortGuid and each
 * 									Remote PortGuid reported in NodeRecords
 *
 * Lid				McMemberRecord	SA GetTable McRecords with given MLid		
 * Lid				SwitchInfoRecord SA GetTable SwitchInfoRecords with Lid		
 * Lid				LinearFDBRecord	SA GetTable LinearFDBRecords for switch Lid
 * Lid				RandomFDBRecord	SA GetTable RandomFDBRecords for switch Lid
 * Lid				MCastFDBRecord	SA GetTable MCastFDBRecords for switch Lid
 * Lid				VLArbTableRecord SA GetTable VLArbTableRecords with Lid
 * Lid				PKeyTableRecord SA GetTable PKeyTableRecords with Lid
 * Lid				GuidInfoRecord	SA GetTable GuidInfoRecords with Lid
 *
 * NodeDesc			Lids			SA GetTable NodeRecords with NodeDesc		
 * NodeDesc			SystemImageGuid	SA GetTable NodeRecords with NodeDesc		
 * NodeDesc			NodeGuids		SA GetTable NodeRecords with NodeDesc		
 * NodeDesc			PortGuids		SA GetTable NodeRecords with NodeDesc		
 * NodeDesc			NodeDesc		SA GetTable NodeRecords with NodeDesc		
 * NodeDesc			NodeRecord		SA GetTable NodeRecords with NodeDesc		
 *
 * NodeDesc			PathRecord		SA GetTable NodeRecords with NodeDesc		
 * 									followed by a SA GetTable PathRecords
 * 									from the local PortGuid and each
 * 									Remote PortGuid reported in NodeRecords
 *
 * ServiceRecord	ServiceRecord	SA GetTable ServiceRecords with
 * 									ServiceRecord and component mask given
 *
 * McMemberRecord	McMemberRecord	SA GetTable McMemberRecords with
 * 									McMemberRecord and component mask given
 */

/* record query selector(s) for QueryFabricInformation
 * next to each is listed the corresponding field in the QUERY_INPUT_VALUE
 * union below which will be used
 */
typedef enum  _QUERY_INPUT_TYPE
{
    InputTypeNoInput = 0,          	/* No input. returns all records */
    InputTypeNodeType,          	/* NodeType */
	InputTypeSystemImageGuid,		/* Guid - a system Image guid */
    InputTypeNodeGuid,              /* Guid - a node guid */
    InputTypePortGuid,              /* Guid - a port guid */
    InputTypePortGid,              	/* Gid - a gid associated with a port */
    InputTypeMcGid,              	/* Gid - a multicast gid */
    InputTypePortGuidPair,			/* GuidPair - a pair of port guids */
    InputTypeGidPair,				/* GidPair - a pair of gids */
    InputTypePathRecord,            /* PathRecord */
#ifdef IB_STACK_OPENIB
    InputTypePathRecordNetworkOrder,/* PathRecord in network byte order */
#endif
	InputTypeLid,					/* Lid - a lid in the local subnet */
	InputTypePKey,					/* PKey - a pkey */
	InputTypeSL,					/* SL - a service level */
	InputTypeIndex,					/* Index - an index associated with a VF */
	InputTypeServiceId,				/* ServiceId */
	InputTypeNodeDesc,				/* NodeDesc - a node description/name */
	InputTypeServiceRecord,			/* ServiceRecordValue - complete SA SERVICE_RECORD and component mask */
	InputTypeMcMemberRecord,		/* McMemberRecordValue - complete SA MCMEMBER_RECORD and component mask */
    InputTypePortGuidList,			/* GuidList - a list of port guids */
    InputTypeGidList,				/* GidList - a list of gids */
    InputTypeMultiPathRecord,       /* MultiPathRecord */
	InputTypeSourceGid,             /* SourceGid */

} QUERY_INPUT_TYPE, *PQUERY_INPUT_TYPE;

/* convert QUERY_INPUT_TYPE to a string */
IBA_API const char* iba_sd_query_input_type_msg(QUERY_INPUT_TYPE code);

/* output type requested for QueryFabricInformation
 * next to each is listed the corresponding structure below which will be used
 * for output
 */
typedef enum  _QUERY_RESULT_TYPE
{
	/* SA query results */
    OutputTypeSystemImageGuid = 0,  /* GUID_RESULTS a set of system image GUIDs */
    OutputTypeNodeGuid,            	/* GUID_RESULTS a set of node GUIDs */
    OutputTypePortGuid,            	/* GUID_RESULTS a set of port GUIDs */
    OutputTypeLid,                 	/* LID_RESULTS is a set of LIDs */
    OutputTypeGid,                 	/* GID_RESULTS is a set of GIDs */
    OutputTypeNodeDesc,		        /* NODEDESC_RESULTS set of node descr/names */
    OutputTypePathRecord,           /* PATH_RESULTS is set of path records */
#ifdef IB_STACK_OPENIB
    OutputTypePathRecordNetworkOrder,/* PATH_RESULTS is set of path records in network byte order*/
#endif
	OutputTypeNodeRecord,			/* NODE_RECORD_RESULTS complete SA NodeRecords */
	OutputTypePortInfoRecord,		/* PORTINFO_RECORD_RESULTS complete SA PortInfoRecords */
	OutputTypeSMInfoRecord,			/* SMINFO_RECORD_RESULTS complete SA SMInfoRecords */
	OutputTypeLinkRecord,			/* LINK_RECORD_RESULTS complete SA LinkRecords */
	OutputTypeServiceRecord,		/* SERVICE_RECORD_RESULTS complete SA IB_SERVICE_RECORD */
	OutputTypeMcMemberRecord,		/* MCMEMBER_RECORD_RESULTS complete SA IB_MCMEMBER_RECORD */
	OutputTypeInformInfoRecord,		/* INFORM_INFO_RECORD_RESULTS complete SA IB_INFORM_INFO_RECORD */
    OutputTypeTraceRecord,          /* TRACE_RECORD_RESULTS is set of trace records */
	OutputTypeSwitchInfoRecord,		/* SWITCHINFO_RECORD_RESULTS is set of switch info records */
	OutputTypeLinearFDBRecord,		/* LINEAR_FDB_RECORD_RESULTS is set of linear FDB records */
	OutputTypeRandomFDBRecord,		/* RANDOM_FDB_RECORD_RESULTS is set of random FDB records */
	OutputTypeMCastFDBRecord,		/* MCAST_FDB_RECORD_RESULTS is set of multicast FDB records */
	OutputTypeVLArbTableRecord,		/* VLARBTABLE_RECORD_RESULTS is set of VL Arbitration records */
	OutputTypePKeyTableRecord,		/* PKEYTABLE_RECORD_RESULTS is set of VL Arbitration records */
	OutputTypeVfInfoRecord,			/* VF_RECORD_RESULTS is set of VF info records */
	OutputTypeClassPortInfo,		

    /* PA query results */
	OutputTypePaRecord,             /* PA_PACKET_RESULTS complete PA SinglePacketRespRecords */
	OutputTypePaTableRecord,        /* PA_TABLE_PACKET_RESULTS complete PA MultiPacketRespRecords */

} QUERY_RESULT_TYPE, *PQUERY_RESULT_TYPE;

/* convert QUERY_RESULT_TYPE to a string */
IBA_API const char* iba_sd_query_result_type_msg(QUERY_RESULT_TYPE code);

/* limit of 8 allows Multipath request to fit in a single output packet
 * also allows QUERY_INPUT_VALUE to stay same overall size for backward
 * compatibility
 */
#define MULTIPATH_GID_LIMIT	8

/* input value for QueryFabricInformation, InputType selects field in union */
typedef union _QUERY_INPUT_VALUE
{
    NODE_TYPE TypeOfNode;       /* Query input is a node type */
	EUI64 Guid;                 /* Query input is a GUID */
	IB_GID Gid;                 /* Query input is a GID */
	struct						/* Query input is a GUID pair  */
	{							
		EUI64 SourcePortGuid;   /* Query Input source port GUID */
		EUI64 DestPortGuid;		/* Query Input destination port GUID */
	} PortGuidPair;
	struct						/* Query input is a GUID pair  */
	{							
		IB_GID SourceGid;   	/* Query Input source GID */
		IB_GID DestGid;			/* Query Input destination GID */
	} GidPair;
	struct						/* Query input is a GUID list  */
	{							
		uint8 SourceGuidCount;	/* number of Source GUIDs in GuidList */
		uint8 DestGuidCount;	/* number of Dest GUIDs in GuidList */
		EUI64 GuidList[MULTIPATH_GID_LIMIT];/* Src GUIDs, followed by Dest GUIDs */
	} PortGuidList;
	struct						/* Query input is a GID list  */
	{							
		uint8 SourceGidCount;	/* number of Source GIDs in GidList */
		uint8 DestGidCount;		/* number of Dest GIDs in GidList */
		IB_GID GidList[MULTIPATH_GID_LIMIT];/* Src GIDs, followed by Dest GIDs */
	} GidList;

	IB_LID Lid;					/* Query input is a LID */
	uint16 PKey;				/* Query input is a pkey */
	uint8  SL;					/* Query input is a SL */
	uint16 vfIndex;				/* Query input is a vf index */
	uint64 ServiceId;			/* Query input is a ServiceID */
	struct
	{
		uint64 ComponentMask;
		IB_PATH_RECORD PathRecord;
	} PathRecordValue;
	struct
	{
		uint64 ComponentMask;
		IB_MULTIPATH_RECORD MultiPathRecord;
		/* Gids below allows up to 8 SGID and/or DGID in MultiPathRecord.GIDList */
		/* do not use Gids field directly, instead use */
		/* MultiPathRecord.GIDList[0-7] */
		IB_GID Gids[MULTIPATH_GID_LIMIT-1];
	} MultiPathRecordValue;
	struct
	{
		uint32 NameLength;	/* actual characters, no \0 terminator */
		uint8 Name[NODE_DESCRIPTION_ARRAY_SIZE];	/* not \0 terminated */
	} NodeDesc;
	struct
	{
		uint64 ComponentMask;
		IB_SERVICE_RECORD ServiceRecord;
	} ServiceRecordValue;		/* Use InputTypeServiceRecord */
	struct
	{
		uint64 ComponentMask;
		IB_MCMEMBER_RECORD McMemberRecord;
	} McMemberRecordValue;		/* Use InputTypeMcMemberRecord */
} QUERY_INPUT_VALUE, *PQUERY_INPUT_VALUE;

/* description of a query for QueryFabricInformation */
typedef struct _QUERY  {
    QUERY_INPUT_TYPE InputType;     /* Type of input (i.e. query based on) */
    QUERY_RESULT_TYPE OutputType;   /* Type of output (i.e. info requested) */
    QUERY_INPUT_VALUE InputValue;   /* input record selection value input query */
} QUERY, *PQUERY;

/* value returned from a query
 * Status is primary status
 * When status is FERROR, MadStatus indicates 16 Status and 16 bit
 * Reserved fields from MAD response (see MAD_COMMON.u.NS in ib_mad.h)
 * Status is in least significant 16 bits, Reserved in upper 16 bits
 */
typedef struct _QUERY_RESULT_VALUES  {
    FSTATUS Status;			/* overall result of query */
    uint32	MadStatus;		/* for FSUCCESS or FERROR Status: */
							/* manager's Mad Status code from response */
    uint32  ResultDataSize;	/* number of bytes in QueryResult */
	uint32	reserved;		/* to force 64 bit alignment of QueryResult */
    uint8   QueryResult[1];	/* one of the RESULTS types below as selected by */
							/* OutputType in query */
} QUERY_RESULT_VALUES, *PQUERY_RESULT_VALUES;


/* convert SD MadStatus (uint32 reported by SD) to a string
 * The string will only contain the message corresponding to the defined values
 * to be safe, callers should also display the hex value in the event a
 * reserved or undefined value is returned by the manager
 */
IBA_API const char* iba_sd_mad_status_msg(uint32 code);

/* --------------------------------------------------------------------------
 * Structures for Result types for QueryFabricInformation
 * OutputType in query selects which record format will be returned
 * in QueryResult
 */

typedef struct _GUID_RESULTS  {
    uint32 NumGuids;                /* Number of GUIDs returned */
	EUI64 Guids[1];                 /* Start of the list of GUIDs returned */
} GUID_RESULTS, *PGUID_RESULTS;

typedef struct _LID_RESULTS  {
    uint32 NumLids;                 /* Number of LIDs returned */
    IB_LID Lids[1];                 /* list of LIDs returned */
} LID_RESULTS, *PLID_RESULTS;

typedef struct _GID_RESULTS  {
    uint32 NumGids;                 /* Number of GIDs returned */
	IB_GID Gids[1];                 /* list of GIDs returned */
} GID_RESULTS, *PGID_RESULTS;

typedef struct _PATH_RESULTS  {
    uint32 			NumPathRecords;  /* Number of PathRecords returned */
	IB_PATH_RECORD 	PathRecords[1];   /* list of path records returned */
} PATH_RESULTS, *PPATH_RESULTS;

typedef struct _NODE_RECORD_RESULTS  {
    uint32 				NumNodeRecords;     /* Number of NodeRecords returned */
    IB_NODE_RECORD 		NodeRecords[1];		/* list of Node records returned */
} NODE_RECORD_RESULTS, *PNODE_RECORD_RESULTS;

typedef struct _PORTINFO_RECORD_RESULTS  {
    uint32 				NumPortInfoRecords;	/* Number of PortInfoRecords returned */
    IB_PORTINFO_RECORD 	PortInfoRecords[1];  /* list of PortInfo records returned */
} PORTINFO_RECORD_RESULTS, *PPORTINFO_RECORD_RESULTS;

typedef struct _SMINFO_RECORD_RESULTS  {
    uint32              NumSMInfoRecords;	/* Number of SmInfoRecords returned */
    IB_SMINFO_RECORD  	SMInfoRecords[1];   	/* list of SMInfo records returned */
} SMINFO_RECORD_RESULTS, *PSMINFO_RECORD_RESULTS;

typedef struct _LINK_RECORD_RESULTS  {
    uint32              NumLinkRecords;		/* Number of SmInfoRecords returned */
    IB_LINK_RECORD    	LinkRecords[1];		/* list of SMInfo records returned */
} LINK_RECORD_RESULTS, *PLINK_RECORD_RESULTS;

typedef struct _NODEDESC_RESULTS {
    uint32 			 NumDescs;               /* Number of NodeDescs returned */
	NODE_DESCRIPTION NodeDescs[1];			/* NodeDesc, not \0 terminated */
} NODEDESC_RESULTS, *PNODEDESC_RESULTS;

typedef struct _SERVICE_RECORD_RESULTS  {
    uint32              NumServiceRecords;	/* Number of records returned */
    IB_SERVICE_RECORD   ServiceRecords[1];	/* list of records returned */
} SERVICE_RECORD_RESULTS, *PSERVICE_RECORD_RESULTS;

typedef struct _MCMEMBER_RECORD_RESULTS  {
    uint32              NumMcMemberRecords;	/* Number of records returned */
    IB_MCMEMBER_RECORD  McMemberRecords[1];	/* list of records returned */
} MCMEMBER_RECORD_RESULTS, *PMCMEMBER_RECORD_RESULTS;

typedef struct _INFORM_INFO_RECORD_RESULTS  {
    uint32              NumInformInfoRecords;	/* Number of records returned */
    IB_INFORM_INFO_RECORD  InformInfoRecords[1];/* list of records returned */
} INFORM_INFO_RECORD_RESULTS, *PINFORM_INFO_RECORD_RESULTS;

typedef struct _TRACE_RECORD_RESULTS  {
    uint32 			    NumTraceRecords;   /* Number of TraceRecords returned */
	IB_TRACE_RECORD 	TraceRecords[1];   /* list of trace records returned */
} TRACE_RECORD_RESULTS, *PTRACE_RECORD_RESULTS;

typedef struct _SWITCHINFO_RECORD_RESULTS  {
    uint32 			    NumSwitchInfoRecords;   /* Number of SwitchInfoRecords returned */
	IB_SWITCHINFO_RECORD 	SwitchInfoRecords[1];   /* list of Switch Info records returned */
} SWITCHINFO_RECORD_RESULTS, *PSWITCHINFO_RECORD_RESULTS;

typedef struct _LINEAR_FDB_RECORD_RESULTS  {
    uint32 			    NumLinearFDBRecords;   /* Number of LinearFDBRecords returned */
	IB_LINEAR_FDB_RECORD 	LinearFDBRecords[1];   /* list of Linear FDB records returned */
} LINEAR_FDB_RECORD_RESULTS, *PLINEAR_FDB_RECORD_RESULTS;

typedef struct _RANDOM_FDB_RECORD_RESULTS  {
    uint32 			    NumRandomFDBRecords;   /* Number of RandomFDBRecords returned */
	IB_RANDOM_FDB_RECORD 	RandomFDBRecords[1];   /* list of Random FDB records returned */
} RANDOM_FDB_RECORD_RESULTS, *PRANDOM_FDB_RECORD_RESULTS;

typedef struct _MCAST_FDB_RECORD_RESULTS  {
    uint32 			    NumMCastFDBRecords;   /* Number of MCastFDBRecords returned */
	IB_MCAST_FDB_RECORD 	MCastFDBRecords[1];   /* list of multicast FDB records returned */
} MCAST_FDB_RECORD_RESULTS, *PMCAST_FDB_RECORD_RESULTS;

typedef struct _VLARBTABLE_RECORD_RESULTS  {
    uint32 			    NumVLArbTableRecords;   /* Number of VLArbTableRecords returned */
	IB_VLARBTABLE_RECORD 	VLArbTableRecords[1];   /* list of VL Arbitration table records returned */
} VLARBTABLE_RECORD_RESULTS, *PVLARBTABLE_RECORD_RESULTS;

typedef struct _PKEYTABLE_RECORD_RESULTS  {
    uint32 			    NumPKeyTableRecords;   /* Number of PKeyTableRecords returned */
	IB_P_KEY_TABLE_RECORD 	PKeyTableRecords[1];   /* list of P-Key table records returned */
} PKEYTABLE_RECORD_RESULTS, *PPKEYTABLE_RECORD_RESULTS;

typedef struct _VF_RECORD_RESULTS  {
    uint32 			    NumVfInfoRecords;   /* Number of VfInfoRecords returned */
	VEND_VFINFO_RECORD	VfInfoRecords[1];   /* list of VF Info records returned */
} VF_RECORD_RESULTS, *PVF_RECORD_RESULTS;

typedef struct _IB_CLASS_PORT_INFO_RESULTS  {
    uint32 			    NumClassPortInfo;   /* Number of records returned */
	IB_CLASS_PORT_INFO	ClassPortInfo[1];   /* list of records returned */
} IB_CLASS_PORT_INFO_RESULTS, *PIB_CLASS_PORT_INFO_RESULTS;

/* ===========================================================================
 * FabricOperation
 * Data structures used to manage records in the SA/SM via Set or Delete
 * for a fabric operation a single record is provided in conjunction with
 * an operation to perform.  In response a single record is returned.
 * For some Set operations the SA may augment or modify the original record
 * provided in which case the returned record is interesting.
 * However for delete operations the record is generally uninteresting.
 */

/* FabricOperations Supported:
 *
 * OperationType		Operation performed
 * -------------        -------------------
 * SetServiceRecord		SA Set ServiceRecord with service record given
 * 						creates a service record to register this node/port
 * 						as providing the given service
 *
 * DeleteServiceRecord	SA Delete ServiceRecord with service record given
 * 						deletes a service record to indicate this node/port
 * 						no longer provides the given service
 *
 * SetMcMemberRecord	SA Set McMemberRecord with McMemberRecord given
 * 						creates and/or joins a multicast group
 *
 * JoinMcGroup			SA Set McMemberRecord with MGID and JoinState given
 * 						joins an existing multicast group
 *						input is McJoinLeave, output is McMemberRecord
 *
 * LeaveMcGroup			SA Delete McMemberRecord with MGID and JoinState given
 * 						Leave an existing multicast group
 *						input is McJoinLeave, output is McMemberRecord
 *
 * DeleteMcMemberRecord	SA Delete McMemberRecord with McMemberRecord given
 * 						leaves and/or deletes a multicast group
 *
 * SetInfomInfo			SA Set InformInfo with InformInfo given
 * 						Subscribe for Traps/Notices to be sent to this port
 */

/* Fabric Operation to perform */
typedef enum  _FABRIC_OPERATION_TYPE
{
	/* SA operations */
	FabOpSetServiceRecord =0,		/* Set a service record */
	FabOpDeleteServiceRecord,		/* Delete a service record */
	FabOpSetMcMemberRecord,			/* Set a Multicast Member record */
	FabOpJoinMcGroup,				/* Join a Multicast Group */
	FabOpLeaveMcGroup,				/* Leave a Multicast Group */
	FabOpDeleteMcMemberRecord,		/* Delete a Multicast Member record */
	FabOpSetInformInfo,				/* Set InformInfo to register for notices */
} FABRIC_OPERATION_TYPE;

/* convert FABRIC_OPERATION_TYPE to a string */
IBA_API const char* iba_sd_fabric_op_type_msg(FABRIC_OPERATION_TYPE code);

/* value for a FabricOperation, used as input and output */
typedef	union	_FABRIC_OPERATION_VALUE
{
	/* Data for SA operations */
	struct
	{
		uint64 ComponentMask;	/* input only, only for DeleteServiceRecord */
		IB_SERVICE_RECORD ServiceRecord;
	} ServiceRecordValue;
	struct
	{
		uint64 ComponentMask;	/* input only */
		IB_MCMEMBER_RECORD McMemberRecord;
	} McMemberRecordValue;
	struct {
		IB_GID	MGID;
		uint8	JoinFullMember:1;
		uint8	JoinNonMember:1;
		uint8	JoinSendOnlyMember:1;
		uint8	Reserved:5;
	} McJoinLeave;
	IB_INFORM_INFO				InformInfo;
} FABRIC_OPERATION_VALUE;

/* Data for a FabricOperation, also returned as result */
typedef	struct	_FABRIC_OPERATION_DATA
{
	FABRIC_OPERATION_TYPE	Type;
	FABRIC_OPERATION_VALUE	Value;
} FABRIC_OPERATION_DATA, *PFABRIC_OPERATION_DATA;

/* ===========================================================================
 * common (kernel/user) mode function interface
 */

/* Register as a client with the Subnet Data Interface
 * handle is returned which must be used in all subsequent calls on behalf
 * of this registration
 * pass a NULL to use default ClientParameters
 * this does not preempt
 * data pointed to only used during duration of call
 */
typedef  FSTATUS
		(SD_REGISTER)(
		 IN OUT CLIENT_HANDLE				*ClientHandle,
   IN PCLIENT_CONTROL_PARAMETERS		pClientParameters OPTIONAL
					 );
IBA_API SD_REGISTER iba_sd_register;

/* DeRegister as a client with the Subnet Data Interface
 * this does not preempt
 */
typedef  FSTATUS
		(SD_DEREGISTER)(
		 IN CLIENT_HANDLE					ClientHandle
					   );
IBA_API SD_DEREGISTER iba_sd_deregister;

/* get present parameters to control queries/operations by client
 * this does not preempt
 * data pointed to only used during duration of call
 */
typedef FSTATUS
		(SD_GET_CLIENT_CONTROL_PARAMETERS)(
		 IN CLIENT_HANDLE					ClientHandle,
   IN OUT PCLIENT_CONTROL_PARAMETERS	pClientControlParameters
										  );
IBA_API SD_GET_CLIENT_CONTROL_PARAMETERS iba_sd_get_client_control_parameters;

/* set parameters to control queries/operations by client
 * this does not preempt
 * data pointed to only used during duration of call
 */
typedef FSTATUS
		(SD_SET_CLIENT_CONTROL_PARAMETERS)(
		 IN CLIENT_HANDLE					ClientHandle,
   IN PCLIENT_CONTROL_PARAMETERS		pClientControlParameters
										  );
IBA_API SD_SET_CLIENT_CONTROL_PARAMETERS iba_sd_set_client_control_parameters;

/* Callback made when a Report Request is received
 * this callback occurs in a thread context, however it should not preempt
 * for long durations
 * data pointed to only valid during duration of call
 */
typedef
		void (SD_REPORT_NOTICE_CALLBACK)(
			  IN void   *pContext, /* as supplied at subscription time */
	 IN IB_NOTICE *pNotice,  /* IB_NOTICE record returned in the Report Request */
  IN EUI64   PortGuid  /* Port Guid from which Report Request was received */
										);
typedef SD_REPORT_NOTICE_CALLBACK *PREPORT_NOTICE_CALLBACK;/* depricated */

/* Callback made in kernel mode when a multicast group's state changes
 * this callback occurs in a thread context, however it should not preempt
 * for long durations
 * data pointed to only valid during duration of call
 *
 * There are three basic combinations of callback:
 * State                        Status       pMcMemberRecord
 * MC_GROUP_STATE_JOIN_FAILED   failure code Only fields from request are valid
 * MC_GROUP_STATE_AVAILABLE     FSUCCESS     All fields valid, response from SA
 * MC_GROUP_STATE_UNAVAILABLE   N/A          Only fields from request are valid
 */
typedef 
		void (SD_MULTICAST_CALLBACK)(
			  IN void               *pContext,       /* as supplied at group join time */
	 IN FSTATUS            Status,          /* reason code */
  IN MC_GROUP_STATE     State,           /* current state of multicast group */
  IN IB_MCMEMBER_RECORD *pMcMemberRecord /* current member record settings */
									);
typedef SD_MULTICAST_CALLBACK *PMULTICAST_CALLBACK;/* depricated */


/* ===========================================================================
 * kernel mode function interface
 */
#if defined(VXWORKS)

/* return a list of the PortGuids for presently active ports
 * caller must MemoryDeallocate(*ppLocalPortGuidsList) when done using it
 * this does not preempt
 * if no ports are found, can return FSUCCESS but with both outputs set to 0
 * data pointed to only used during duration of call
 */
typedef FSTATUS
		(SDK_GET_LOCAL_PORT_GUIDS)(
		 IN OUT uint64						**ppLocalPortGuidsList,
   IN OUT uint32						*LocalPortGuidsCount
								  );
IBA_API SDK_GET_LOCAL_PORT_GUIDS iba_sd_get_local_port_guids_alloc;

/* return a list of the PortGuids and SubnetPrefixes for presently active ports
 * caller must MemoryDeallocate(*ppLocalPortGuidsList) and
 * MemoryDeallocate(*ppLocalPortSubnetPrefixList) when done using it
 * this does not preempt
 * if no ports are found, can return FSUCCESS but with both outputs set to 0
 * data pointed to only used during duration of call
 */
typedef FSTATUS
		(SDK_GET_LOCAL_PORT_GUIDS2)(
		 IN OUT uint64						**ppLocalPortGuidsList,
   IN OUT uint64						**ppLocalPortSubnetPrefixList,
   IN OUT uint32						*LocalPortGuidsCount
								   );
IBA_API SDK_GET_LOCAL_PORT_GUIDS2 iba_sd_get_local_port_guids_alloc2;

/* Callback made in kernel mode when a Query[Port]FabricInformation completes
 * this callback occurs in a thread context, however it should not preempt
 * for long durations
 * data pointed to only valid during duration of call
 */
typedef 
		void (SDK_QUERY_CALLBACK)(
			  IN void					*Context,		/* as supplied in Query */
	 IN PQUERY				pInputQuery,	/* input request */
  IN PQUERY_RESULT_VALUES pQueryResults	/* results returned from Manager */
								 );
typedef SDK_QUERY_CALLBACK *PQUERY_CALLBACK;/* depricated */

/* perform a query of a manager (SA, etc)
 * using 1st active port
 * if pQueryControlParameters is NULL, values established for client session
 * will be used
 * this function does not preempt.  When the operation completes a callback
 * will occur with the result data.
 * data pointed to only used during duration of call
 */
typedef  FSTATUS
		(SDK_QUERY_FABRIC_INFO)(
		 IN CLIENT_HANDLE				ClientHandle,
   IN PQUERY						pQuery,		/* query to make */
   IN SDK_QUERY_CALLBACK			*Callback,	/* called on completion */
   IN COMMAND_CONTROL_PARAMETERS	*pQueryControlParameters OPTIONAL,
   IN void							*Context OPTIONAL/* context for callback */
							   );
IBA_API SDK_QUERY_FABRIC_INFO iba_sd_query_fabric_info;

/* Same as QueryFabricInformation, except issued against a specific port */
typedef FSTATUS
		(SDK_QUERY_PORT_FABRIC_INFO)(
		 IN CLIENT_HANDLE				ClientHandle,
   IN EUI64						PortGuid,
   IN PQUERY						pQuery,
   IN SDK_QUERY_CALLBACK			*Callback,
   IN COMMAND_CONTROL_PARAMETERS	*pQueryControlParameters OPTIONAL,
   IN void							*Context OPTIONAL
									);
IBA_API SDK_QUERY_PORT_FABRIC_INFO iba_sd_query_port_fabric_info;

/* Callback made in kernel mode when a [Port]FabricOperation completes
 * this callback occurs in a thread context, however it should not preempt
 * for long durations
 * data pointed to only valid during duration of call
 */
typedef 
		void (SDK_FABRIC_OPERATION_CALLBACK)(
			  IN void* 					Context,/* as supplied in FabricOperation */
	 IN FABRIC_OPERATION_DATA*	pFabOp,	/* contains response value */
  IN FSTATUS 					Status,	/* overall result of query */
  IN uint32					MadStatus	/* for FSUCCESS or FERROR Status: */
		  /* manager's Mad Status code from resp */
											);
typedef SDK_FABRIC_OPERATION_CALLBACK *PFABRIC_OPERATION_CALLBACK;/* depricated */

/* perform a fabric operation (eg. set/delete, etc) against a manager
 * using 1st active port
 * if pCmdControlParameters is NULL, values established for client session
 * will be used
 * this function does not preempt.  When the operation completes a callback
 * will occur with the result data.
 * data pointed to only used during duration of call
 */
typedef  FSTATUS
		(SDK_FABRIC_OPERATION)(
		 IN CLIENT_HANDLE 				ClientHandle,
   IN FABRIC_OPERATION_DATA* 		pFabOp,	/* operation to perform */
   IN SDK_FABRIC_OPERATION_CALLBACK 	*Callback,
   IN COMMAND_CONTROL_PARAMETERS* 	pCmdControlParameters OPTIONAL,
   IN void*						Context OPTIONAL
							  );
IBA_API SDK_FABRIC_OPERATION iba_sd_fabric_operation;

/* Same as FabricOperation, except issued against a specific port */
typedef FSTATUS
		(SDK_PORT_FABRIC_OPERATION)(
		 IN CLIENT_HANDLE 				ClientHandle,
   IN EUI64 						PortGuid,
   IN FABRIC_OPERATION_DATA*		pFabOp,	/* operation to perform */
   IN SDK_FABRIC_OPERATION_CALLBACK	*Callback,
   IN COMMAND_CONTROL_PARAMETERS* 	pCmdControlParameters OPTIONAL,
   IN void* 						Context OPTIONAL
								   );
IBA_API SDK_PORT_FABRIC_OPERATION iba_sd_port_fabric_operation;

/* subscribe for an SA Trap/Notice
 * client will receive a callback when the given Trap/Notice is received
 */
typedef FSTATUS
		(SDK_TRAP_NOTICE_SUBSCRIBE)(
		 IN CLIENT_HANDLE ClientHandle,
   IN uint16 TrapNumber,
   IN EUI64 PortGuid,
   IN void *pContext,
   IN SD_REPORT_NOTICE_CALLBACK *pReportNoticeCallback
								   );
IBA_API SDK_TRAP_NOTICE_SUBSCRIBE iba_sd_trap_notice_subscribe;

/* unsubscribe for an SA Trap/Notice
 * this does not preempt
 */
typedef FSTATUS
		(SDK_TRAP_NOTICE_UNSUBSCRIBE)(
		 IN CLIENT_HANDLE ClientHandle,
   IN uint16 TrapNumber,
   IN EUI64 PortGuid
									 );
IBA_API SDK_TRAP_NOTICE_UNSUBSCRIBE iba_sd_trap_notice_unsubscribe;

/* have any of the client's traps been successfully subscribed for
 * this does not preempt
 */
typedef boolean
		(SDK_PORT_TRAPS_SUBSCRIBED)(
		 IN EUI64 PortGuid
								   );
IBA_API SDK_PORT_TRAPS_SUBSCRIBED iba_sd_port_traps_subscribed;

/* Request join/create of a multicast group
 *
 * This version of the call uses the MGID as the group identifier
 *
 * SdClientHandle - handle from iba_sd_register
 * McFlags - MC_FLAGS_* to control when get callbacks
 * ComponentMask - indicate fields in pMcMemberRecord which are set
 * pContext - user defined context pointer supplied to callback
 * pMulticastCallback - callback function invoked when join state changes
 *
 * Since joins are per HCA port, this call handles multiple applications
 * joining the same MC group and will only issue joins to the SM on the
 * 1st application joining.  Similarly iba_sd_leave_mcgroup will only issue
 * the leave to the SM on last application leaving.
 *
 * Internally this call maintains the joined state and will rejoin when
 * fabric events warrent, such as SM restart, port down and up, etc.
 *
 * this does not preempt
 * data pointed to only used during duration of call
 */
typedef FSTATUS
		(SDK_JOIN_MULTICAST_GROUP)(
		 IN CLIENT_HANDLE SdClientHandle,
   IN uint16 McFlags,
   IN uint64 ComponentMask, 
   IN IB_MCMEMBER_RECORD *pMcMemberRecord,
   IN void *pContext,
   IN SD_MULTICAST_CALLBACK *pMulticastCallback
								  );
IBA_API SDK_JOIN_MULTICAST_GROUP iba_sd_join_mcgroup;

/* Request join/create of a multicast group
 *
 * This version of the call uses the RID as the group identifier
 *
 * SdClientHandle - handle from iba_sd_register
 * McFlags - MC_FLAGS_* to control when get callbacks
 * ComponentMask - indicate fields in pMcMemberRecord which are set
 * pContext - user defined context pointer supplied to callback
 * pMulticastCallback - callback function invoked when join state changes
 *
 * Since joins are per HCA port, this call handles multiple applications
 * joining the same MC group and will only issue joins to the SM on the
 * 1st application joining.  Similarly iba_sd_leave_mcgroup2 will only issue
 * the leave to the SM on last application leaving.
 *
 * Internally this call maintains the joined state and will rejoin when
 * fabric events warrent, such as SM restart, port down and up, etc.
 *
 * this does not preempt
 * data pointed to only used during duration of call
 */
typedef FSTATUS
		(SDK_JOIN_MULTICAST_GROUP2)(
		 IN CLIENT_HANDLE SdClientHandle,
   IN uint16 McFlags,
   IN uint64 ComponentMask, 
   IN IB_MCMEMBER_RECORD *pMcMemberRecord,
   IN EUI64 EgressPortGUID,
   IN void *pContext,
   IN SD_MULTICAST_CALLBACK *pMulticastCallback
								   );
IBA_API SDK_JOIN_MULTICAST_GROUP2 iba_sd_join_mcgroup2;

/* Request leave of a multicast group which was previously joined via
 * iba_sd_join_mcgroup
 *
 * Once this returns, no more callbacks will be issued for the given join.
 * This call does not trigger a callback.
 *
 * This will only issue the leave to the SM on last application leaving the
 * group.
 *
 * this does not preempt
 */
typedef FSTATUS
		(SDK_LEAVE_MULTICAST_GROUP)(
		 IN CLIENT_HANDLE SdClientHandle,
   IN IB_GID *pMGID
								   );
IBA_API SDK_LEAVE_MULTICAST_GROUP iba_sd_leave_mcgroup;

/* Request leave of a multicast group which was previously joined via
 * iba_sd_join_mcgroup2
 *
 * Once this returns, no more callbacks will be issued for the given join.
 * This call does not trigger a callback.
 *
 * This will only issue the leave to the SM on last application leaving the
 * group.
 *
 * this does not preempt
 */
typedef FSTATUS
		(SDK_LEAVE_MULTICAST_GROUP2)(
		 IN CLIENT_HANDLE SdClientHandle,
   IN IB_GID *pMGID,
   IN IB_GID *pPortGID,
   IN EUI64  EgressPortGUID
									);
IBA_API SDK_LEAVE_MULTICAST_GROUP2 iba_sd_leave_mcgroup2;

#endif /* defined(VXWORKS) */

#ifdef __cplusplus
};
#endif

#endif  /* _IBA_IB_SD_H_ */
