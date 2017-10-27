/* BEGIN_ICS_COPYRIGHT3 ****************************************

Copyright (c) 2015-17, Intel Corporation

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

#ifndef _IBA_IB_SD_PRIV_H_
#define _IBA_IB_SD_PRIV_H_

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

#include <iba/ib_sd.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _SMINFO_RECORD_RESULTS  {
    uint32              NumSMInfoRecords;	/* Number of SmInfoRecords returned */
    IB_SMINFO_RECORD  	SMInfoRecords[1];   	/* list of SMInfo records returned */
} SMINFO_RECORD_RESULTS, *PSMINFO_RECORD_RESULTS;

typedef struct _LINK_RECORD_RESULTS  {
    uint32              NumLinkRecords;		/* Number of SmInfoRecords returned */
    IB_LINK_RECORD    	LinkRecords[1];		/* list of SMInfo records returned */
} LINK_RECORD_RESULTS, *PLINK_RECORD_RESULTS;

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

#endif  /* _IBA_IB_SD_PRIV_H_ */
