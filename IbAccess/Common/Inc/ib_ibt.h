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

#ifndef __IBA_IB_IBT_H__
#define __IBA_IB_IBT_H__	(1)

#include "iba/stl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * IB notification routines
 */

/*
 * Bit masks for event types subscribed and generated
 */

typedef	uintn	IB_NOTIFY_TYPE;

#define		IB_NOTIFY_CA_ADD		0x00000001	/* A new adapter was added */
#define		IB_NOTIFY_CA_REMOVE		0x00000002	/* An adapter is being removed */
#define		IB_NOTIFY_PORT_DOWN		0x00000004	/* Link went down, sync loss */
#define		IB_NOTIFY_LID_EVENT		0x00000008	/* The specified port had a */
												/* LID/state/LMC change */
#define		IB_NOTIFY_SM_EVENT		0x00000010	/* A Subnet Manager has  */
												/* programmed specified port */
#define		IB_NOTIFY_PKEY_EVENT	0x00000020	/* The Pkey for the specified */
												/* port has changed */
#define		IB_NOTIFY_PORT_UP		0x00000040	/* Link went up */
#define		IB_NOTIFY_ON_REGISTER	0x80000000	/* Notify immediately during */
												/* Registration of any */
												/* present state (eg. CA_ADD */
												/* for all active CAs and */
												/* LID_EVENT for all active */
												/* ports) */
#define		IB_NOTIFY_ANY			0x7FFFFFFF	/* Receive notification for  */
												/* all event types */
												/* note IB_NOTIFY_ON_REGISTER */
												/* is not included in this flag */

/* Notification callback style
 * async - no locks are held during callback:
 *	- callback can deregister or do any other operations
 *	- callback could occur slightly after deregister, caller must handle
 * sync - a lock is held during callback:
 *	- callback cannot deregister or do operations which may cause a notification
 *		(eg. send packets to local SMA)
 *	- callbacks will not occur after deregister
 */
typedef enum {
	IB_NOTIFY_ASYNC,
	IB_NOTIFY_SYNC
} IB_NOTIFY_LOCKING;


/*
 * Notification record passed during a callback
 */

typedef struct {
	IB_NOTIFY_TYPE		EventType;	/* Type of event being reported */
	EUI64				Guid;		/* Port/Node GUID, depends on event type */
	void				*Context;	/* User Context that was supplied  */
									/* during the registration */
} IB_NOTIFY_RECORD;


typedef 
void
( *IB_NOTIFY_CALLBACK ) (
	IN	IB_NOTIFY_RECORD		NotifyRecord
	);

/*
 * iba_register_notify/RegisterNotify
 *
 *	register for notifications of IB type events
 *
 * INPUTS:
 *
 *	Callback		-Callback routine to be registered for notifications
 *	Context			-User specified context to return in callbacks
 *	EventMask		-Mask to subscribe to event types
 *	Locking			-Locking/Synchronization model
 *
 * OUTPUTS:
 *
 *	NotifyHandle	-Notification handle on successful registration
 *
 * RETURNS:
 *
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY
 */

typedef
FSTATUS
(IBT_REGISTER_NOTIFY) (
	IN	IB_NOTIFY_CALLBACK	Callback,
	IN	void				*Context,
	IN	IB_NOTIFY_TYPE		EventMask,
	IN	IB_NOTIFY_LOCKING	Locking,
	OUT	IB_HANDLE			*NotifyHandle
	);
IBA_API IBT_REGISTER_NOTIFY iba_register_notify;


/*
 * iba_register_guid_notify/RegisterGuidNotify
 *
 *	register for notifications of IB type events against a specific Node/Port
 *	Guid
 *
 *
 * INPUTS:
 *
 *	Guid			-Guid to limit events to
 * 						(Node Guid for CA events, Port Guid for other events)
 *	Callback		-Callback routine to be registered for notifications
 *	Context			-User specified context to return in callbacks
 *	EventMask		-Mask to subscribe to event types
 *	Locking			-Locking/Synchronization model
 *
 * OUTPUTS:
 *
 *	NotifyHandle	-Notification handle on successful registration
 *
 * RETURNS:
 *
 *	FSUCCESS
 *	FINSUFFICIENT_MEMORY
 *
 */

typedef
FSTATUS
(IBT_REGISTER_GUID_NOTIFY) (
	IN	EUI64				Guid,
	IN	IB_NOTIFY_CALLBACK	Callback,
	IN	void				*Context,
	IN	IB_NOTIFY_TYPE		EventMask,
	IN	IB_NOTIFY_LOCKING	Locking,
	OUT	IB_HANDLE			*NotifyHandle
	);
IBA_API IBT_REGISTER_GUID_NOTIFY iba_register_guid_notify;

/*
 * iba_deregister_notify/DeregisterNotify 
 *
 *	Deregister previously registered notifications
 *
 * INPUTS:
 *
 *	NotifyHandle		-Notification handle returned on successful registration
 *
 * OUTPUTS:
 *
 *
 * RETURNS:
 *
 *	FSUCCESS
 *	FINVALID_PARAMETER
 *	FINSUFFICIENT_MEMORY
 */

typedef
FSTATUS
(IBT_DEREGISTER_NOTIFY) (
	IN 	IB_HANDLE			NotifyHandle
	);
IBA_API IBT_DEREGISTER_NOTIFY iba_deregister_notify;

/*
 * iba_get_caguids/GetCaGuids 
 *
 *	Get the list of CA's presently in the system
 *
 * INPUTS:
 *	CaCount	- size of CaGuidList
 *			  data pointed to only used during duration of call
 *
 * OUTPUTS:
 *	CaCount	- total CAs in system (always returned, even if error status)
 *	CaGuidList - where to save CA Guids
 *
 * RETURNS:
 *	FINSUFFICIENT_MEMORY - CaGuidList too small for # of CAs in system
 *	FINVALID_PARAMETER - CaGuidList is NULL and CaCount was not 0
 */

typedef
FSTATUS
(IBT_GET_CA_GUIDS)(
	IN OUT uint32		*CaCount,
	OUT EUI64			*CaGuidList
	);
IBA_API IBT_GET_CA_GUIDS iba_get_caguids;

/*
 * iba_get_caguids_alloc/GetCaGuidsAlloc
 *
 *	Get the list of CA's presently in the system and allocate *CaGuidList
 *	caller must MemoryDeallocate *CaGuidList
 *
 * INPUTS:
 *
 * OUTPUTS:
 *	CaCount	- total CAs in system (always returned, even if error status)
 *	CaGuidList - where to save CA Guids
 *
 * RETURNS:
 *	FINSUFFICIENT_RESOURCES - can't allocate memory for list
 *	FINVALID_PARAMETER - CaCount or CaGuidList is NULL
 */

typedef
FSTATUS
(IBT_GET_CA_GUIDS_ALLOC)(
	OUT uint32			*CaCount,
	OUT EUI64			**CaGuidList
	);
IBA_API IBT_GET_CA_GUIDS_ALLOC iba_get_caguids_alloc;

/*
 * iba_query_ca_by_guid/QueryCaByGuid 
 *
 *	Get CA attributes for CA with given Guid
 *
 * INPUTS:
 *	CaGuid - CA to query
 *
 * OUTPUTS:
 *	CaAttributes
 *
 * RETURNS:
 *	FINVALID_PARAMETER - CaAttributes is NULL
 *	FNOT_FOUND - no such CA
 *	otherwise status of QueryCA is returned
 */

typedef
FSTATUS
(IBT_QUERY_CA_BY_GUID)(
	IN  EUI64				CaGuid,
	OUT	IB_CA_ATTRIBUTES	*CaAttributes
	);
IBA_API IBT_QUERY_CA_BY_GUID iba_query_ca_by_guid;

/*
 * iba_query_ca_by_guid_alloc/QueryCaByGuidAlloc
 *
 *	Get CA attributes for CA with given Guid and allocate
 *	CaAttributes->PortAttributesList
 *	caller must MemoryDeallocate CaAttributes->PortAttributesList
 *
 * INPUTS:
 *	CaGuid - CA to query
 *
 * OUTPUTS:
 *	CaAttributes
 *
 * RETURNS:
 *	FINVALID_PARAMETER - CaAttributes is NULL
 *	FNOT_FOUND - no such CA
 *	otherwise status of QueryCA is returned:
 * 	FINSUFFICIENT_MEMORY - PortAttributesListSize too small
 * 					ca attributes returned, but not port attributes
 * 	FSUCCESS - ca and port attributes returned
 */

typedef
FSTATUS
(IBT_QUERY_CA_BY_GUID_ALLOC)(
	IN  EUI64				CaGuid,
	OUT	IB_CA_ATTRIBUTES	*CaAttributes
	);
IBA_API IBT_QUERY_CA_BY_GUID_ALLOC iba_query_ca_by_guid_alloc;

/*
 * iba_query_port_by_guid/QueryPortByGuid 
 *
 *	Get Port attributes for CA Port with given Guid
 *
 * INPUTS:
 *	PortGuid - CA Port to query
 *	ByteCount - size of PortAttributes
 *
 * OUTPUTS:
 *	PortAttributes
 *	ByteCount - space needed to hold PortAttributes
 *
 * RETURNS:
 *	FINSUFFICIENT_MEMORY - ByteCount insufficient
 *	FINSUFFICIENT_RESOURCES - unable to allocate internal memory needed
 *	FINVALID_PARAMETER - PortAttributes is NULL
 *	FNOT_FOUND - no such CA Port
 *	otherwise status of QueryCA is returned
 */

typedef
FSTATUS
(IBT_QUERY_PORT_BY_GUID)(
	IN  EUI64				PortGuid,
	OUT IB_PORT_ATTRIBUTES	*PortAttributes,
	IN OUT uint32			*ByteCount
	);
IBA_API IBT_QUERY_PORT_BY_GUID iba_query_port_by_guid;

/*
 * iba_query_port_by_guid/QueryPortByGuid 
 *
 *	Get Port attributes for CA Port with given Guid and allocate
 *	*PortAttributes
 *	caller must MemoryDeallocate *PortAttributes
 *
 * INPUTS:
 *	PortGuid - CA Port to query
 *
 * OUTPUTS:
 *	*PortAttributes
 *
 * RETURNS:
 *	FINSUFFICIENT_MEMORY - ByteCount insufficient
 *	FINSUFFICIENT_RESOURCES - unable to allocate internal memory needed
 *	FINVALID_PARAMETER - PortAttributes is NULL
 *	FNOT_FOUND - no such CA Port
 *	otherwise status of QueryCA is returned
 */

typedef
FSTATUS
(IBT_QUERY_PORT_BY_GUID_ALLOC)(
	IN  EUI64				PortGuid,
	OUT IB_PORT_ATTRIBUTES	**PortAttributes
	);
IBA_API IBT_QUERY_PORT_BY_GUID_ALLOC iba_query_port_by_guid_alloc;

/* translate port number into a port Guid
 *
 * INPUTS:
 * 	caGuid - CA to check against
 * 	port - 1-n, port within CA
 * 			if 0, 1st active port
 *
 * OUTPUTS:
 * 	*pPortGuid - port guid for specified port
 * 	*pCaAttributes - attributes for CA,
 * 					caller must MemoryDeallocate pCaAttributes->PortAttributesList
 * 	*ppPortAtributes - attributes for port, caller must MemoryDeallocate
 * 	*pPortCount - number of ports in CA
 *
 * RETURNS:
 * 	FNOT_FOUND - *pPortCount still output
 */
typedef
FSTATUS
(IBT_GET_CA_PORTGUID)(
		IN EUI64 caGuid,
		IN uint32 port,
		OUT EUI64 *pPortGuid OPTIONAL,
		OUT IB_CA_ATTRIBUTES		*pCaAttributes OPTIONAL,
		OUT IB_PORT_ATTRIBUTES		**ppPortAttributes OPTIONAL,
		OUT uint32 *pPortCount OPTIONAL
		);
IBA_API IBT_GET_CA_PORTGUID iba_get_ca_portguid;

/* translate ca/port number into a port Guid
 *
 * INPUTS:
 * 	ca - system wide CA number 1-n, if 0 port is a system wide port #
 * 	port - 1-n, if ca is 0, system wide port number, otherwise port within CA
 * 			if 0, 1st active port
 *
 * OUTPUTS:
 * 	*pCaGuid - ca guid for specified port
 * 	*pPortGuid - port guid for specified port
 * 	*pCaAttributes - attributes for CA,
 * 					caller must MemoryDeallocate pCaAttributes->PortAttributesList
 * 	*ppPortAtributes - attributes for port, caller must MemoryDeallocate
 * 	*pCaCount - number of CA in system
 * 	*pPortCount - number of ports in system or CA (depends on ca input)
 *
 * RETURNS:
 * 	FNOT_FOUND - *pCaCount and *pPortCount still output
 * 				if ca == 0, *pPortCount = number of ports in system
 * 				if ca < *pCaCount, *pPortCount = number of ports in CA
 * 									otherwise *pPortCount will be 0
 */
typedef
FSTATUS
IBT_GET_PORTGUID(
		IN uint32 ca,
		IN uint32 port,
		OUT EUI64 *pCaGuid OPTIONAL,
		OUT EUI64 *pPortGuid OPTIONAL,
		OUT IB_CA_ATTRIBUTES		*pCaAttributes OPTIONAL,
		OUT IB_PORT_ATTRIBUTES		**ppPortAttributes OPTIONAL,
		OUT uint32 *pCaCount OPTIONAL,
		OUT uint32 *pPortCount OPTIONAL
		);
IBA_API IBT_GET_PORTGUID iba_get_portguid;

/* format an error message when iba_get_portguid returns FNOT_FOUND
 *
 * INPUTS:
 * 	ca, port - inputs provided to failed call to iba_get_portguid
 *	CaCount, PortCount - outputs returned by failed call to iba_get_portguid
 *
 * RETURNS:
 * 	string formatted with error message.  This is a pointer into a static
 *  string which will be invalidated upon next call to this function.
 */
typedef
const char*
IBT_FORMAT_GET_PORTGUID_ERROR(
		IN uint32 ca,
		IN uint32 port,
		IN uint32 CaCount,
		IN uint32 PortCount
		);
IBA_API IBT_FORMAT_GET_PORTGUID_ERROR iba_format_get_portguid_error;

/*
 * iba_find_ca_gid/FindCaGid 
 *
 *	search Ports of a given CA to find the Port with the matching GID
 *
 * INPUTS:
 *	pGid	- GID to search for
 *			  data pointed to only used during duration of call
 *	CaGuid	- CA to search
 *
 * OUTPUTS:
 *	pPortGuid - port which GID was found on
 *	pGidIndex - index of GID within ports GID Table
 *	PortAttributes - attributes of port (caller must MemoryDeallocate)
 *	LocalCaAckDelay - corresponding field from CaAttributes
 *
 * RETURNS:
 *	FINSUFFICIENT_MEMORY - unable to allocate internal memory needed
 *	FINSUFFICIENT_RESOURCES - unable to allocate internal memory needed
 *	FNOT_FOUND - no such CA Port or no such CA
 *	otherwise status of QueryCA is returned
 */

typedef
FSTATUS
(IBT_FIND_CA_GID)(
	IN const IB_GID *pGid,
	IN EUI64 CaGuid,
	OUT EUI64 *pPortGuid OPTIONAL,
	OUT uint8 *pGidIndex OPTIONAL,
	OUT IB_PORT_ATTRIBUTES	**PortAttributes OPTIONAL,
	OUT uint8 *pLocalCaAckDelay OPTIONAL
	);
IBA_API IBT_FIND_CA_GID iba_find_ca_gid;

/*
 * iba_find_ca_gid2/FindCaGid2
 *
 *	search Ports of a given CA to find the Port with the matching GID
 *
 * INPUTS:
 *	pGid	- GID to search for
 *			  data pointed to only used during duration of call
 *	CaGuid	- CA to search
 *
 * OUTPUTS:
 *	pPortGuid - port which GID was found on
 *	pGidIndex - index of GID within ports GID Table
 *	PortAttributes - attributes of port (caller must MemoryDeallocate)
 *	CaAttributes - CaAttributes (caller must MemoryDeallocate)
 *
 * RETURNS:
 *	FINSUFFICIENT_MEMORY - unable to allocate internal memory needed
 *	FINSUFFICIENT_RESOURCES - unable to allocate internal memory needed
 *	FNOT_FOUND - no such CA Port or no such CA
 *	otherwise status of QueryCA is returned
 */

typedef
FSTATUS
(IBT_FIND_CA_GID2)(
	IN const IB_GID *pGid,
	IN EUI64 CaGuid,
	OUT EUI64 *pPortGuid OPTIONAL,
	OUT uint8 *pGidIndex OPTIONAL,
	OUT IB_PORT_ATTRIBUTES	**PortAttributes OPTIONAL,
	OUT IB_CA_ATTRIBUTES **CaAttributes OPTIONAL
	);
IBA_API IBT_FIND_CA_GID2 iba_find_ca_gid2;

/*
 * iba_find_gid
 *
 *	search Ports of all CAs to find the CA/Port with the matching GID
 *
 * INPUTS:
 *	pGid	- GID to search for
 *			  data pointed to only used during duration of call
 *
 * OUTPUTS:
 *	pCaGuid	- CA which GID was found on
 *	pPortGuid - port which GID was found on
 *	pGidIndex - index of GID within ports GID Table
 *	PortAttributes - attributes of port (caller must MemoryDeallocate)
 *	CaAttributes - CaAttributes (caller must MemoryDeallocate)
 *
 * RETURNS:
 *	FINSUFFICIENT_MEMORY - unable to allocate internal memory needed
 *	FINSUFFICIENT_RESOURCES - unable to allocate internal memory needed
 *	FNOT_FOUND - no such CA Port
 *	otherwise status of QueryCA is returned
 */

typedef
FSTATUS
(IBT_FIND_GID)(
	IN const IB_GID *pGid,
	OUT EUI64 *pCaGuid OPTIONAL,
	OUT EUI64 *pPortGuid OPTIONAL,
	OUT uint8 *pGidIndex OPTIONAL,
	OUT IB_PORT_ATTRIBUTES	**PortAttributes OPTIONAL,
	OUT IB_CA_ATTRIBUTES **CaAttributes OPTIONAL
	);
IBA_API IBT_FIND_GID iba_find_gid;

/*
 * iba_get_ca_for_port/GetCaForPort
 *
 *	Get CA Guid for given Port Guid
 *
 * INPUTS:
 *	PortGuid - Port to find CA for
 *
 * OUTPUTS:
 *	CaGuid - CA Guid for given port
 *
 * RETURNS:
 *	FINSUFFICIENT_RESOURCES - unable to allocate internal memory needed
 *	FNOT_FOUND - no such CA Port
 *	otherwise status of QueryCA is returned
 */

typedef
FSTATUS
(IBT_GET_CA_FOR_PORT)(
	IN  EUI64				PortGuid,
	OUT EUI64				*CaGuid
	);
IBA_API IBT_GET_CA_FOR_PORT iba_get_ca_for_port;

/*
 * iba_select_ca
 *
 *	Track CA/ports being heavily used and aid in selection of least used one
 *	Usage models:
 *		pCaGuid = NULL   invalid call
 *		*pCaGuid !=0     caller has specified a CA, API increases its count
 *		*pCaGuid ==0     API will recommend a CA and increase its count
 *
 * INPUTS:
 *	*pCaGuid - CA caller plans to use
 *
 * OUTPUTS:
 *	*pCaGuid - CA recommended for use (matches supplied *pCaGuid if input != 0)
 *
 * RETURNS:
 *	FNOT_FOUND - no such CA
 *	FINVALID_PARAMETER - invalid combination of arguments
 *	FINSUFFICIENT_RESOURCES - no CAs in system with active ports
 *	FERROR - other errors (unable to query CA attributes, etc)
 */

typedef
FSTATUS
(IBT_SELECT_CA)(
	IN OUT	EUI64				*pCaGuid
	);
IBA_API IBT_SELECT_CA iba_select_ca;

/*
 * iba_deselect_ca
 *
 * undo effects of iba_select_ca
 * call should match the CaGUID from a previous iba_select_ca call
 *
 * INPUTS:
 *	CaGuid - CA output/selected via iba_select_ca
 *
 * OUTPUTS:
 *	None
 *
 * RETURNS:
 *	FNOT_FOUND - no such CA or Port, or PortGuid not within specified CA
 *	FINVALID_OPERATION - would result in negative use count
 */

typedef
FSTATUS
(IBT_DESELECT_CA)(
	IN EUI64				CaGuid
	);
IBA_API IBT_DESELECT_CA iba_deselect_ca;

#ifdef __cplusplus
};
#endif

#endif	/* __IBA_IB_IBT_H__ */
