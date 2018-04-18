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

/* Suppress duplicate loading of this file */
#ifndef _IBA_VPI_EXPORT_H_
#define _IBA_VPI_EXPORT_H_

#include "iba/vpi.h"
#include "stl_types.h"
#include "stl_sm_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Verbs Provider interface.
 * This is the interface provided by the access layer for use by kernel
 * mode applications.
 */

/* New drivers/applications should use the iba_* functions.
 * They can now be called directly.
 * In some cases there are not iba_* versions of the version 1 API
 * in these cases the version 2 API for the given interface should be used.
 */


/*
 * OpenCA/iba_open_ca
 *
 * Returns a handle to an open instance of a CA. The consumer may call this
 * to retrieve a new open instance, and the number of open instances allowed
 * is not fixed but is subject to available system resources.
 *
 * INPUTS:
 *	CaGUID - The Channel Adapter's EUI64 GUID identifier.
 *	CompletionCallback - Completion handler. One per open instance.
 *	AsyncEventCallback - Asynchronous Error handler. One per open instance.
 *	Context - Consumer supplied value to be returned on calls to the handlers
 *		to identify the associated CA to the consumer.
 *
 * OUTPUTS:
 *	CaHandle - Handle to the newly open instance of the CA.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_OPENCA)(
	IN  EUI64					CaGUID,
	IN	IB_COMPLETION_CALLBACK	CompletionCallback,
	IN	IB_ASYNC_EVENT_CALLBACK	AsyncEventCallback,
	IN	void					*Context,
	OUT	IB_HANDLE				*CaHandle
	);
IBA_API VPI_OPENCA iba_open_ca;

/*
 * QueryCA/iba_query_ca
 *
 * Returns the capabilities of the CA and the Ports in the CA.
 *
 * INPUTS:
 *	CaHandle - Handle from a previous OpenCA
 *
 * OUTPUTS:
 *	CaAttributes - Where to put the attributes of the CA.
 *			The CaAttributes->PortAttributesList is where to store
 *			the port attributes.  If NULL, this call will fail and
 *			will return the proper size needed in PortAttributesListSize.
 *			CaAttributes->PortAttributesListSize must indicate the
 *			size available at PortAttributesList.  If too small, the call
 *			will fail and indicate the size needed via this field.
 *	Context - Consumer supplied context value
 *
 * RETURNS:
 * 	FINSUFFICIENT_MEMORY - PortAttributesListSize too small
 * 					ca attributes returned, but not port attributes
 * 	FINVALID_PARAMETER - invalid argument
 * 	FSUCCESS - ca and port attributes returned
 *
 */

typedef FSTATUS (VPI_QUERYCA)(
	IN  IB_HANDLE			CaHandle,
	OUT IB_CA_ATTRIBUTES	*CaAttributes,
	OUT	void				**Context
	);
IBA_API VPI_QUERYCA iba_query_ca;

/*
 * same as VPI_QUERYCA, but returns version #2 of the CA attributes.
 */
typedef FSTATUS (VPI_QUERYCA2)(
	IN  IB_HANDLE			CaHandle,
	OUT IB_CA_ATTRIBUTES2	*CaAttributes,
	OUT	void				**Context
	);
IBA_API VPI_QUERYCA2 iba_query_ca2;

/*
 * SetCompletionHandler/iba_set_compl_handler
 *
 * Change the consumer-supplied completion handler for an open
 * instance of the CA.
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of a CA.
 *	CompletionCallback - Completion handler.
 *
 * OUTPUTS:
 *	None.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_SETCOMPLETIONHANDLER)(
	IN  IB_HANDLE				CaHandle,
	IN  IB_COMPLETION_CALLBACK	CompletionCallback
	);
IBA_API VPI_SETCOMPLETIONHANDLER iba_set_compl_handler;

/*
 * SetAsyncEventHandler/iba_set_async_event_handler
 *
 * Change the consumer-supplied asynchronous error handler for an open
 * instance of the CA.
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of a CA.
 *	AsyncEventCallback - Async Error handler.
 *
 * OUTPUTS:
 *	None.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_SETASYNCEVENTHANDLER)(
	IN  IB_HANDLE				CaHandle,
	IN  IB_ASYNC_EVENT_CALLBACK	AsyncEventCallback
	);
IBA_API VPI_SETASYNCEVENTHANDLER iba_set_async_event_handler;

/*
 * SetCAContext/iba_set_ca_context
 *
 * Change the consumer-supplied context value for an open instance of a CA.
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of a CA.
 *	Context - New context value.
 *
 * OUTPUTS:
 *	None.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_SETCACONTEXT)(
	IN	IB_HANDLE	CaHandle,
	IN	void		*Context
	);
IBA_API VPI_SETCACONTEXT iba_set_ca_context;

/*SetCAInternal
 * 
 *mark a CA open instance as for internal stack use
 *
 * INPUTS:
 *	.		CaHandle			handle of the channel adapter 
 *
 * OUTPUTS:
 *			None
 *
 * RETURNS:
 *			Status 'FSUCCESS' if instance updated appropriately
 *			else a relevant error status.
 * 
 */

typedef FSTATUS (VPI_SETCAINTERNAL)(
	IN  IB_HANDLE	CaHandle
	);
VPI_SETCAINTERNAL SetCAInternal;	/* internal use only */

/*
 * ModifyCA/iba_modify_ca
 *
 * Modifies CA attributes.
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of the CA.
 *	CaAttributes - new attribute settings
 *				data pointed to only used during duration of call
 *
 * OUTPUTS:
 *	None.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_MODIFYCA)(
	IN  IB_HANDLE			CaHandle,
	IN	IB_CA_ATTRIBUTES	*CaAttributes
	);
IBA_API VPI_MODIFYCA iba_modify_ca;

/*CloseCA/iba_close_ca
 * 
 *Close the previously opened channel adapter.
 *
 * INPUTS:
 *			CaHandle			handle of the channel adapter 
 *
 * OUTPUTS:
 *			None		
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the specified channel adapter was
 *			closed successfully else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_CLOSECA)(
	IN  IB_HANDLE	CaHandle
	);
IBA_API VPI_CLOSECA iba_close_ca;

/*AllocatePD2/iba_alloc_pd
 * 
 *create a Protection Domain.
 *
 * INPUTS:
 *	.		CaHandle			handle of the channel adapter 
 *			MaxAVs				number of AV's user would like to be able to
 *								allocate within this PD
 *
 * OUTPUTS:
 *			PdHandle			Pointer to the PD handle returned by this
 *								function.
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the PD was created successfully 
 *			else a relevant error status.
 * 
 */

/* depricated, use ALLOCATEPD2 */
typedef FSTATUS (VPI_ALLOCATEPD)(
	IN  IB_HANDLE	CaHandle,
	OUT IB_HANDLE	*PdHandle
	);
VPI_ALLOCATEPD AllocatePD;	/* internal use only, depricated */

/* Version2 of API, supports control on limit of Address Vectors in PD */
typedef FSTATUS (VPI_ALLOCATEPD2)(
	IN  IB_HANDLE	CaHandle,
	IN  uint32		MaxAVs,
	OUT IB_HANDLE	*PdHandle
	);
IBA_API VPI_ALLOCATEPD2 iba_alloc_pd;

/* iba_alloc_sqp_pd
 * 
 *create a Protection Domain for use by Special QPs.
 *This PD will only be allowed to have Special QPs (no normal QPs)
 *as such the hardware driver may be able to optimize handling of
 *AVs and other resources
 *
 * INPUTS:
 *	.		CaHandle			handle of the channel adapter 
 *			MaxAVs				number of AV's user would like to be able to
 *								allocate within this PD
 *
 * OUTPUTS:
 *			PdHandle			Pointer to the PD handle returned by this
 *								function.
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the PD was created successfully 
 *			else a relevant error status.
 * 
 */

typedef FSTATUS (VPI_ALLOCATESQPPD)(
	IN  IB_HANDLE	CaHandle,
	IN  uint32		MaxAVs,
	OUT IB_HANDLE	*PdHandle
	);
IBA_API VPI_ALLOCATESQPPD iba_alloc_sqp_pd;

/*FreePD/iba_free_pd
 * 
 *Destroy a previously created Protection Domain.
 *
 *	INPUTS:
 *			PdHandle			PD handle 
 *
 *	OUTPUTS:
 *			none
 *	RETURNS:
 *			Status 'FSUCCESS' if the PD was destroyed successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_FREEPD)(
	IN  IB_HANDLE	PdHandle
	);
IBA_API VPI_FREEPD iba_free_pd;

/*
 * AllocateRDD/iba_alloc_rdd
 *
 * Allocates a Reliable Datagram Domain.
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of a CA.
 *
 * OUTPUTS:
 *	RddHandle - Handle to allocated RDD.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_ALLOCATERDD)(
	IN  IB_HANDLE	CaHandle,
	OUT IB_HANDLE	*RddHandle
	);
IBA_API VPI_ALLOCATERDD iba_alloc_rdd;

/*FreeRDD/iba_free_rdd
 * 
 *free a previously created RDD.
 *
 * INPUTS:
 *			RddHandle			handle of the RDD 
 *
 * OUTPUTS:
 *			None
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the RDD was freed successfully 
 *			else a relevant error status.
 * 
 */

typedef FSTATUS (VPI_FREERDD)(
	IN  IB_HANDLE	RddHandle
	);
IBA_API VPI_FREERDD iba_free_rdd;

/*CreateAV/iba_create_av
 * 
 *create an Address Vector.
 *
 * INPUTS:
 *	.		CaHandle			handle of the channel adapter 
 *			PdHandle			PD handle
 *			AddressVector		pointer to the address vector structure.
 *								data pointed to only used during duration of call
 *
 * OUTPUTS:
 *			AvHandle			Pointer to the AV handle returned by this
 *								function.
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the Address Vector was created successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_CREATEAV)(
	IN  IB_HANDLE			CaHandle,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ADDRESS_VECTOR	*AddressVector,
	OUT IB_HANDLE			*AvHandle
	);
IBA_API VPI_CREATEAV iba_create_av;

/*ModifyAV/iba_modify_av
 * 
 *modify a previously created Address Vector.
 *
 * INPUTS:
 *			AvHandle			AV handle.
 *			AddressVector		pointer to the address vector	
 *								structure
 *								data pointed to only used during duration of call
 *
 * OUTPUTS:	none
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the Address Vector was modified successfully 
 *			else a relevant error status.
 * 
 */

typedef FSTATUS (VPI_MODIFYAV)(
	IN  IB_HANDLE			AvHandle,
	IN  IB_ADDRESS_VECTOR	*AddressVector
	);
IBA_API VPI_MODIFYAV iba_modify_av;


/*QueryAV/iba_query_av
 * 
 *query the attributes of a previously Address Vector.
 *
 *	INPUTS:
 *				AvHandle			AV handle.
 *	OUTPUTS:
 *				AddressVector		pointer to the address vector
 *									structure
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the Address Vector was queried successfully 
 *			else a relevant error status.
 * 
 */

typedef FSTATUS (VPI_QUERYAV)(
	IN  IB_HANDLE			AvHandle,
	OUT IB_ADDRESS_VECTOR	*AddressVector,
	OUT IB_HANDLE			*PdHAndle
	);
IBA_API VPI_QUERYAV iba_query_av;

/*DestroyAV/iba_destroy_av
 * 
 *destroy a previously created Address Vector.
 *
 * INPUTS:
 *	.		RddHandle			handle of the RDD 
 *
 * OUTPUTS:
 *			None
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the Address vector was destroyed successfully 
 *			else a relevant error status.
 * 
 */

typedef FSTATUS (VPI_DESTROYAV)(
	IN  IB_HANDLE	AvHandle
	);
IBA_API VPI_DESTROYAV iba_destroy_av;

/*
 * CreateQP/iba_create_qp
 *
 * Creates a Queue Pair.
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of the CA.
 *	QpCreateAttributes - Contains the values in the Create member of the
 *		attributes union to be used to configure the QP.
 *		data pointed to only used during duration of call
 *	Context - Consumer supplied value to be returned on when the asynchronous
 *		error callback is made for informational or error events on this QP.
 *
 * OUTPUTS:
 *	QpHandle - Handle to the QP returned.
 *	QpAttributes - Attributes upon completion of the operation residing in the
 *		the Query member of the attributes union.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_CREATEQP)(
	IN  IB_HANDLE				CaHandle,
	IN  IB_QP_ATTRIBUTES_CREATE	*QpCreateAttributes,
	IN	void					*Context,
	OUT IB_HANDLE				*QpHandle,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes
	);
IBA_API VPI_CREATEQP iba_create_qp;

/*ModifyQP/iba_modify_qp
 * 
 *modify the attributes of a  previously created queue pair.
 *
 * INPUTS:
 *			QpHandle			queue pair handle.
 *			QpModifyAttributes	Attributes to Modify
 *								data pointed to only used during duration of call
 * OUTPUTS:
 *			QpAttributes		Resulting attributes
 * RETURNS:
 *			Status 'FSUCCESS' if the queue pair was modified successfully 
 *			else a relevant error status.
 * 
 */

typedef FSTATUS (VPI_MODIFYQP)(
	IN  IB_HANDLE						QpHandle,
	IN  IB_QP_ATTRIBUTES_MODIFY			*QpModifyAttributes,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes OPTIONAL
	);
IBA_API VPI_MODIFYQP iba_modify_qp;

/*ResetQp/iba_reset_qp
 * 
 *reset a previously created queue pair.
 *This is a simple wrapper for ib_modify_qp which sets up the QpAttributes
 *to move the QP to the QPStateReset state
 *
 * INPUTS:
 *			QpHandle			queue pair handle.
 * OUTPUTS:
 *			QpAttributes		Resulting attributes
 * RETURNS:
 *			Status 'FSUCCESS' if the queue pair was modified successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_RESETQP)(
	IN  IB_HANDLE						QpHandle,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes OPTIONAL
	);
IBA_API VPI_RESETQP iba_reset_qp;

/*InitQp/iba_init_qp
 * 
 *initialize a previously created RC/UC/RD queue pair.
 *This is a simple wrapper for ib_modify_qp which sets up the QpAttributes
 *to move the QP to the QPStateInit state
 *
 * INPUTS:
 *			QpHandle			queue pair handle.
 *			PortGuid			port to use for QP
 *			PkeyIndex			PKey Index
 *			AccessControl		Access Control settings for QP
 *			Qkey				RD QKey, set to 0 for RC/UC QPs
 * OUTPUTS:
 *			QpAttributes		Resulting attributes
 * RETURNS:
 *			Status 'FSUCCESS' if the queue pair was modified successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_INITQP)(
	IN  IB_HANDLE						QpHandle,
	IN	EUI64							PortGuid,
	IN	uint16							PkeyIndex,
	IN	IB_ACCESS_CONTROL				AccessControl,
	IN	IB_Q_KEY						Qkey OPTIONAL,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes OPTIONAL
	);
IBA_API VPI_INITQP iba_init_qp;

/*ReadyUdQp/iba_ready_ud_qp
 * 
 *initialize a previously created UD queue pair.
 *This is a simple wrapper for ib_modify_qp which sets up the QpAttributes
 *to move the QP through the QPStateInit, RTR and RTS states
 *
 * INPUTS:
 *			QpHandle			queue pair handle.
 *			PortGuid			port to use for QP
 *			PkeyIndex			PKey Index
 *			Qkey				QKey
 * OUTPUTS:
 *			QpAttributes		Resulting attributes
 * RETURNS:
 *			Status 'FSUCCESS' if the queue pair was modified successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_READYUDQP)(
	IN  IB_HANDLE						QpHandle,
	IN	EUI64							PortGuid,
	IN	uint16							PkeyIndex,
	IN	IB_Q_KEY						Qkey,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes OPTIONAL
	);
IBA_API VPI_READYUDQP iba_ready_ud_qp;

/*ReInitQp/iba_reinit_qp
 * 
 *reset and re-initialize a previously created queue pair.
 *This is a simple wrapper for ib_modify_qp which sets up the QpAttributes
 *to move the QP to QPStateInit state via QPStateReset
 *The present AccessControl, PortGuid, PKeyIndex and QKey settings
 *are retained for the QP
 *
 * INPUTS:
 *			QpHandle			queue pair handle.
 * OUTPUTS:
 *			QpAttributes		Resulting attributes
 * RETURNS:
 *			Status 'FSUCCESS' if the queue pair was modified successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_REINITQP)(
	IN  IB_HANDLE						QpHandle,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes OPTIONAL
	);
IBA_API VPI_REINITQP iba_reinit_qp;

/* 
 *ErrorQp/iba_error_qp
 * 
 *move a previously created queue pair to QPStateError
 *This is a simple wrapper for ib_modify_qp which sets up the QpAttributes
 *to move the QP to the QPStateError state
 *
 * INPUTS:
 *			QpHandle			queue pair handle.
 * OUTPUTS:
 *			QpAttributes		Resulting attributes
 * RETURNS:
 *			Status 'FSUCCESS' if the queue pair was modified successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_ERRORQP)(
	IN  IB_HANDLE						QpHandle,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes OPTIONAL
	);
IBA_API VPI_ERRORQP iba_error_qp;

/*SendQDrainQp/iba_sendq_drain_qp
 * 
 *move a previously created queue pair to the QPStateSendQDrain state
 *This is a simple wrapper for ib_modify_qp which sets up the QpAttributes
 *to move the QP to the QPStateSendQDrain state
 *
 * INPUTS:
 *			QpHandle			queue pair handle.
 *			EnableSQDAsyncEvent	should async event be generated when Q has
 *								completed draining
 * OUTPUTS:
 *			QpAttributes		Resulting attributes
 * RETURNS:
 *			Status 'FSUCCESS' if the queue pair was modified successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_SENDQDRAINQP)(
	IN  IB_HANDLE						QpHandle,
	IN  boolean							EnableSQDAsyncEvent,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes OPTIONAL
	);
IBA_API VPI_SENDQDRAINQP iba_sendq_drain_qp;

/*
 * SetQPContext/iba_set_qp_context
 *
 * Changes the consumer-supplied context value for a QP to the supplied value.
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of a CA.
 *	Context - New context value.
 *
 * OUTPUTS:
 *	None.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_SETQPCONTEXT)(
	IN  IB_HANDLE	QpHandle,
	IN  void		*Context
	);
IBA_API VPI_SETQPCONTEXT iba_set_qp_context;


/*QueryQP/iba_query_qp
 * 
 *query the attributes of a specific queue pair.
 *
 * INPUTS:
 *			QpHandle			queue pair handle.
 *
 * OUTPUTS:
 *			QpAttributes		pointer to the queue pair attribute
 *								structure
 *			Context				pointer to the pointer of the  caller's context
 *								associated with the queue pair. The returned 
 *								context value should be equal to the last value
 *								set by the user.
 * RETURNS:
 *			Status 'FSUCCESS' if the queue pair attributes were retrieved
 *			successfully else a relevant error status.
 * 
 */

typedef FSTATUS (VPI_QUERYQP)(
	IN  IB_HANDLE				QpHandle,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes,
	OUT	void					**Context
	);
IBA_API VPI_QUERYQP iba_query_qp;

/*DestroyQP/iba_destroy_qp
 * 
 *destroy a previously created queue pair.
 *
 * INPUTS:
 *			QpHandle			handle of the queue pair 
 *
 * OUTPUTS:
 *			None
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the queue pair was destroyed successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_DESTROYQP)(
	IN  IB_HANDLE	QpHandle
	);
IBA_API VPI_DESTROYQP iba_destroy_qp;

/*
 * CreateSpecialQP/iba_create_special_qp
 *
 * Creates a special Queue Pair.
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of the CA.
 *	PortGUID - For SMI or GSI QPs the port must be given. This
 *		parameter is ignored for other Special QP types.
 *	QpCreateAttributes - Contains the values in the Create member of the
 *		attributes union to be used to configure the QP.
 *		data pointed to only used during duration of call
 *	Context - Consumer supplied value to be returned on when the asynchronous
 *		error callback is made for informational or error events on this QP.
 *
 * OUTPUTS:
 *	QpHandle - Handle to the QP returned.
 *	QpAttributes - Attributes upon completion of the operation residing in the
 *		the Query member of the attributes union.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_CREATESPECIALQP)(
	IN  IB_HANDLE				CaHandle,
	IN  EUI64					PortGUID,
	IN  IB_QP_ATTRIBUTES_CREATE	*QpCreateAttributes,
	IN	void					*Context,
	OUT IB_HANDLE				*QpHandle,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes
	);
IBA_API VPI_CREATESPECIALQP iba_create_special_qp;

/*
 * AttachQPToMulticastGroup/iba_attach_qp_to_mcgroup
 *
 * Attach a UD QP the specified Multicast group, hence enabling receipt of
 * multicast packets for the given group delivered to the CA to that QP.
 *
 * INPUTS:
 *	QpHandle - Handle to a UD QP
 *	McgDestLid - Multicast Group's DLID
 *	McgGID - Multicast Group's GID
 *
 * OUTPUTS:
 *	None.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_ATTACHQPTOMULTICASTGROUP2)(
	IN IB_HANDLE QpHandle,
#if INCLUDE_16B
	IN STL_LID McgDestLid,
#else
	IN IB_LID McgDestLid,
#endif
	IN IB_GID McgGID
	);
IBA_API VPI_ATTACHQPTOMULTICASTGROUP2 iba_attach_qp_to_mcgroup;

/*
 * DetachQPFromMulticastGroup/iba_detach_qp_from_mcgroup
 *
 * Detach a UD QP from the specified Multicast group, hence disabling receipt of
 * multicast packets for the given group delivered to the CA to that QP.
 *
 * INPUTS:
 *	QpHandle - Handle to a UD QP
 *	McgDestLid - Multicast Group's DLID
 *	McgGID - Multicast Group's GID
 *
 * OUTPUTS:
 *	None.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_DETACHQPFROMMULTICASTGROUP2)(
	IN IB_HANDLE QpHandle,
#if INCLUDE_16B
	IN STL_LID McgDestLid,
#else
	IN IB_LID McgDestLid,
#endif
	IN IB_GID McgGID
	);
IBA_API VPI_DETACHQPFROMMULTICASTGROUP2 iba_detach_qp_from_mcgroup;

/*CreateCQ/iba_create_cq
 * 
 *create a completion queue.
 *
 * INPUTS:
 *		CaHandle				handle of the channel adapter 
 *		CqReqSize				Required size of the CQ
 *		Context					pointer to the caller's context.
 *
 * OUTPUTS:
 *		CqHandle			pointer to the CQ handle returned by this
 *							function.
 *		CqSize				pointer to the actual CQ size  returned
 *							by this function.
 * RETURNS:
 *			Status 'FSUCCESS' if the CQ was created successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_CREATECQ)(
	IN  IB_HANDLE	CaHandle,
	IN  uint32		CqReqSize,
	IN  void		*Context,
	OUT IB_HANDLE	*CqHandle,
	OUT	uint32		*CqSize
	);
IBA_API VPI_CREATECQ iba_create_cq;

/*ResizeCQ/iba_resize_cq
 * 
 * re size the CQ.
 *
 * INPUTS:
 *	.	CqHandle				handle of the completion queue 
 *		CqReqSize				required size of the CQ
 *
 * OUTPUTS:
 *		Cqsize					pointer to the 	actual size returned by this
 *								function.		
 *
 * RETURNS:
 *		Status 'FSUCCESS' if the QP was created successfully 
 *		else a relevant error status.
 *
 *		If FOVERRUN is returned, the new CQ size was not adequate to hold
 *		all the presently queued completions.  In which case the CQ
 *		has been resized to the new size, however some completions
 *		have been discarded.
 * 
 */
typedef FSTATUS (VPI_RESIZECQ)(
	IN  IB_HANDLE	CqHandle,
	IN  uint32		CqReqSize,
	OUT	uint32		*Cqsize
	);
IBA_API VPI_RESIZECQ iba_resize_cq;

/*
 * SetCQContext/iba_set_cq_context
 *
 * Changes the consumer-supplied context value for a CQ to the supplied value.
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of a CA.
 *	Context - New context value.
 *
 * OUTPUTS:
 *
 *	None.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_SETCQCONTEXT)(
	IN  IB_HANDLE	CqHandle,
	IN  void		*Context
	);
IBA_API VPI_SETCQCONTEXT iba_set_cq_context;

/*
 * iba_set_cq_compl_handler
 *
 * Changes the consumer-supplied completion handler for a CQ
 * If a CQ completion handler is provided, the CA level completion handler
 * for the given CQ will not be called.  If a CQ handler is removed
 * (set to NULL), the CA level completion handler will take effect again.
 *
 * INPUTS:
 *	CqHandle - Handle to a CQ
 *	CompletionCallback - Completion handler.
 *
 * OUTPUTS:
 *	None.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_SETCQCOMPLETIONHANDLER)(
	IN  IB_HANDLE				CqHandle,
	IN  IB_COMPLETION_CALLBACK	CompletionCallback
	);
IBA_API VPI_SETCQCOMPLETIONHANDLER iba_set_cq_compl_handler;

#if defined(VXWORKS)
/* 
 * Set the completion handler and context for the kernel CQ given a handle
 * previously provided by iba_get_kernel_cq
 * this function is depricated, it is only provided for backward compatibility
 * with uDAPL
 *
 * INPUTS:
 *		CqHndl		handle previously provided by iba_get_kernel_cq
 *		CompletionCallback	kernel level callback function
 *		Context		kernel level context value
 *
 * OUTPUTS:
 *			
 * RETURNS:
 *			FSUCCESS if CqHndl is valid
 *			FINVALID_PARAMETER otherwise
 */

typedef FSTATUS (VPI_SETKCQCOMPLETIONHANDLER) (
						IN	uint32			CqHndl,
						IN IB_COMPLETION_CALLBACK	CompletionCallback,
						IN void				*Context
				   );
IBA_API VPI_SETKCQCOMPLETIONHANDLER iba_set_k_cq_comp_handler;
#endif /* defined(VXWORKS) */

/*QueryCQ/iba_query_cq
 * 
 *query the attributes of a specific completion queue.
 *
 * INPUTS:
 *			CqHandle			CQ handle.
 *
 * OUTPUTS:
 *			CqSize				pointer to the size of the CQ returned
 *								by this function.
 *			Context				pointer to the pointer of the  caller's context
 *								returned by this function. The context value
 *								should match the last context value set by the 
 *								caller.
 * RETURNS:
 *			Status 'FSUCCESS' if the queue pair attributes were retrieved
 *			successfully else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_QUERYCQ)(
	IN  IB_HANDLE	CqHandle,
	OUT uint32		*Cqsize,
	OUT	void		**Context
	);
IBA_API VPI_QUERYCQ iba_query_cq;

/*DestroyCQ/iba_destroy_cq
 * 
 *destroy a previously created completion queue.
 *
 * INPUTS:
 *	.		CqHandle			handle of the completion queue 
 *
 * OUTPUTS:
 *			None
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the completion queue was destroyed 
 *			successfully else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_DESTROYCQ)(
	IN  IB_HANDLE	CqHandle
	);
IBA_API VPI_DESTROYCQ iba_destroy_cq;

/*RegisterMemRegion/iba_register_mr
 * 
 *register a memory region with the hardware.
 *
 * INPUTS:
 *	.		CaHandle		handle of the channel adapter 
 *			VirtualAddress	virtual address of the memory region.
 *			Length			length of the memory region
 *			PdHandle		PD handle
 *			AccessControl	Access Control 	
 *
 * OUTPUTS:
 *			MrHandle		handle associated with the memory region.
 *			Lkey			pointer to the local key returned by the
 *							function
 *			Rkey			pointer to the remote key returned by the
 *							function.
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the memory region was registered successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_REGISTERMEMREGION)(
	IN  IB_HANDLE			CaHandle,
	IN  void				*VirtualAddress,
	IN  uintn				Length,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ACCESS_CONTROL	AccessControl,
	OUT IB_HANDLE			*MrHandle,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
IBA_API VPI_REGISTERMEMREGION iba_register_mr;


/*
 * RegisterPhysMemRegion/iba_register_pmr
 *
 * Registers a physical memory region. The physical memory region is defined
 * by a list of physical buffers. The physical buffers must begin and end
 * on an HFI page size boundary.
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of a Channel Adapter.
 *	ReqIoVirtualAddress - the virtual address the consumer request's be used to
 *		address the first byte of the registered memory region.
 *	PhysBufferList - array of the addresses of the physical memory buffers.
 *		Each buffer is a single physical CA page of memory
 *		(eg. using CA Page Size specified in IB_CA_ATTRIBUTES).
 *	NumPhysBuffers - Number of elements in the PhysBufferList array.
 *	IoVaOffset - Offset into the first buffer of the region's IoVirtualAddress.
 *	PdHandle - Handle of the Protection Domain to be used to register
 *		the memory.
 *	AccessControl - Selectors indicating the access control attributes for the
 *		Memory Region.
 *
 * OUTPUTS:
 *	MrHandle - The handle to the registered memory region.
 *	IoVirtualAddress - the virtual address the Verbs Provider has assigned to
 *		the first bytes of the registered memory region.
 *	Lkey - The local protection key for the memory region.
 *	Rkey - The remote protection key for the memory region. Only valid if RdmaWrite,
 *		RdmaRead, or AtomicOp access has been requested for the memory region.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_REGISTERPHYSMEMREGION)(
	IN  IB_HANDLE			CaHandle,
	IN	void				*ReqIoVirtualAddress,
	IN  void				*PhysBufferList,
	IN  uint64				NumPhysBuffers,
	IN  uint32				IoVaOffset,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ACCESS_CONTROL	AccessControl,
	OUT IB_HANDLE			*MrHandle,
	OUT void				**IoVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
VPI_REGISTERPHYSMEMREGION RegisterPhysMemRegion;	/* internal use only, depricated */

/* Version2 of API, supports 64 bit IO virtual addresses */
typedef FSTATUS (VPI_REGISTERPHYSMEMREGION2)(
	IN  IB_HANDLE			CaHandle,
	IN	IB_VIRT_ADDR		ReqIoVirtualAddress,
	IN  void				*PhysBufferList,
	IN  uint64				NumPhysBuffers,
	IN	uint32				IoVaOffset,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ACCESS_CONTROL	AccessControl,
	OUT IB_HANDLE			*MrHandle,
	OUT IB_VIRT_ADDR		*IoVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
IBA_API VPI_REGISTERPHYSMEMREGION2 iba_register_pmr;

/*
 * RegisterContigPhysMemRegion/iba_register_contig_pmr
 *
 * Registers a physical memory region. The physical memory region is defined
 * by a list of physically contiguous buffers.
 * Each physical buffer must begin and end on an HFI page size boundary.
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of a Channel Adapter.
 *	ReqIoVirtualAddress - the virtual address the consumer request's be used to
 *		address the first byte of the registered memory region.
 *	PhysBufferList - array of buffer specifications for the physical memory
 *		buffers.  Each buffer must start and end on a CA Physical Page boundary
 *	NumPhysBuffers - Number of elements in the PhysBufferList array.
 *	IoVaOffset - Offset into the first buffer of the region's IoVirtualAddress.
 *	PdHandle - Handle of the Protection Domain to be used to register
 *		the memory.
 *	AccessControl - Selectors indicating the access control attributes for the
 *		Memory Region.
 *
 * OUTPUTS:
 *	MrHandle - The handle to the registered memory region.
 *	IoVirtualAddress - the virtual address the Verbs Provider has assigned to
 *		the first bytes of the registered memory region.
 *	Lkey - The local protection key for the memory region.
 *	Rkey - The remote protection key for the memory region. Only valid if RdmaWrite,
 *		RdmaRead, or AtomicOp access has been requested for the memory region.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_REGISTERCONTIGPHYSMEMREGION2)(
	IN  IB_HANDLE			CaHandle,
	IN	IB_VIRT_ADDR		ReqIoVirtualAddress,
	IN  IB_MR_PHYS_BUFFER*	PhysBufferList,
	IN  uint64				NumPhysBuffers,
	IN	uint32				IoVaOffset,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ACCESS_CONTROL	AccessControl,
	OUT IB_HANDLE			*MrHandle,
	OUT IB_VIRT_ADDR		*IoVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
IBA_API VPI_REGISTERCONTIGPHYSMEMREGION2 iba_register_contig_pmr;

/*QueryMemRegion/iba_query_mr
 * 
 *query the attributes of a previously registered memory region.
 *
 * INPUTS:
 *			MrHandle			memory region handle.
 *
 * OUTPUTS:
 *			VirtualAddress		pointer to starting virtual address of the
 *								memory region.
 *			Length				length of the memory region
 *			PdHandle			pointer to the PD handle
 *			AccessControl		pointer to the access control word
 *			Lkey				pointer to the local key
 *			Rkey				pointer to the remote key	
 * RETURNS:
 *			Status 'FSUCCESS' if the memory region attributes were retrieved
 *			successfully else a relevant error status.
 * 
 */

typedef FSTATUS (VPI_QUERYMEMREGION)(
	IN  IB_HANDLE			MrHandle,
	OUT	void				**VirtualAddress,
	OUT	uintn				*Length,
	OUT IB_HANDLE			*PdHandle,
	OUT IB_ACCESS_CONTROL	*AccessControl,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
VPI_QUERYMEMREGION QueryMemRegion;	/* internal use only, depricated */

/* Version2 of API, supports 64 bit IO virtual addresses */
typedef FSTATUS (VPI_QUERYMEMREGION2)(
	IN  IB_HANDLE			MrHandle,
	OUT	IB_VIRT_ADDR		*VirtualAddress,
	OUT	uint64				*Length,
	OUT IB_HANDLE			*PdHandle,
	OUT IB_ACCESS_CONTROL	*AccessControl,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
IBA_API VPI_QUERYMEMREGION2 iba_query_mr;

/*DeregisterMemRegion/iba_deregister_mr
 *
 *de-register the previously registered memory region.
 * 
 * INPUTS:
 *			MrHandle		handle of the memory region
 *
 * OUTPUTS:
 *	.		None	
 *
 * RETURNS:
 *	Status 'FSUCCESS' if the memory region was de-registered
 *	successfully else a relevant error status.
 *  
 *					
 */
typedef FSTATUS (VPI_DEREGISTERMEMREGION)(
	IN  IB_HANDLE	MrHandle
	);
IBA_API VPI_DEREGISTERMEMREGION iba_deregister_mr;

/*
 * ModifyMemRegion/iba_modify_mr
 *
 * Modifies the attributes of an existing Memory Region. Any combination of
 * attributes may be modified by the call. The attributes are Virtual Address
 * with Length, Protection Domain, and Access Control selectors.
 *
 * If the Protection Domain handle is null for a modify Protection Domain
 * request, the effect is to make the Memory Region inaccessible for all
 * work request operations until a Protection Domain handle is provided on
 * a subsequent ModifyMemRegion call.
 *
 * INPUTS:
 *	MrHandle - Handle to an Memory Region. Upon completion of this call this
 *		handle is no longer valid, regardless of the return status of the call.
 *	ModifyReqType - Indicates an attribute or a combination of attributes to
 *		modify.
 *	VirtualAddress - Virtual address of the first byte of the memory area
 *		to register.
 *	Length - With the Virtual Address, defines the memory area to register.
 *	ProtectionDomain - Handle of the Protection Domain to be used to register
 *		the memory.
 *	AccessControl - Selectors indicating the access control attributes for the
 *		Memory Region.
 *
 * OUTPUTS:
 *	NewMrHandle - The handle to the modified Memory Region. This handle must be
 *		used on future references to the Memory Region. On error this may be
 *		null, indicating no Memory Region was successfully returned and the
 *		previous Memory Region is no longer registered.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_MODIFYMEMREGION)(
	IN  IB_HANDLE			MrHandle,
	IN  IB_MR_MODIFY_REQ	ModifyReqType,
	IN  void				*VirtualAddress OPTIONAL,
	IN  uintn				Length OPTIONAL,
	IN  IB_HANDLE			PdHandle OPTIONAL,
	IN  IB_ACCESS_CONTROL	AccessControl OPTIONAL,
	OUT IB_HANDLE			*NewMrHandle,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
IBA_API VPI_MODIFYMEMREGION iba_modify_mr;

/*
 * ModifyPhysMemRegion/iba_modify_pmr
 *
 * Modifies the attributes of an existing Memory Region previously created
 * by RegisterPhysMemRegion or RegisterContigPhysMemRegion.
 * Any combination attributes may be modified by
 * the call. The attributes are the physical buffer list, Protection Domain,
 * and Access Control selectors.
 *
 * If the Protection Domain handle is null for a modify Protection Domain
 * request, the effect is to make the Memory Region inaccessible for all
 * work request operations until a Protection Domain handle is provided on
 * a subsequent ModifyPhysMemRegion call.
 *
 * INPUTS:
 *	MrHandle - Handle to an Memory Region. Upon completion of this call this
 *		handle is no longer valid, regardless of the return status of the call.
 *	ModifyReqType - Indicates an attribute or a combination of attributes to
 *		modify.
 *	ReqIoVirtualAddress - Virtual address of the first byte of the memory
 *		area to register.
 *	NumPhysBuffers - With the Virtual Address, defines the memory area to register.
 *	IoVaOffset - Offset into the first buffer of the region's IoVirtualAddress.
 *	ProtectionDomain - Handle of the Protection Domain to be used to register
 *		the memory.
 *	AccessControl - Selectors indicating the access control attributes for the
 *		Memory Region.
 *
 * OUTPUTS:
 *	NewMrHandle - The handle to the modified Memory Region. This handle must be
 *		used on future references to the Memory Region. On error this may be
 *		null, indicating no Memory Region was successfully returned and the
 *		previous Memory Region is no longer registered.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_MODIFYPHYSMEMREGION)(
	IN  IB_HANDLE			MrHandle,
	IN  IB_MR_MODIFY_REQ	ModifyReqType,
	IN	void				*ReqIoVirtualAddress OPTIONAL,
	IN  void				*PhysBufferList OPTIONAL,
	IN  uint64				NumPhysBuffers OPTIONAL,
	IN  uint32				IoVaOffset OPTIONAL,
	IN  IB_HANDLE			PdHandle OPTIONAL,
	IN  IB_ACCESS_CONTROL	AccessControl OPTIONAL,
	OUT IB_HANDLE			*NewMrHandle,
	OUT void				**IoVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
VPI_MODIFYPHYSMEMREGION ModifyPhysMemRegion;	/* internal use only, depricated */

/* Version2 of API, supports 64 bit IO virtual addresses */
typedef FSTATUS (VPI_MODIFYPHYSMEMREGION2)(
	IN  IB_HANDLE			MrHandle,
	IN  IB_MR_MODIFY_REQ	ModifyReqType,
	IN	IB_VIRT_ADDR		ReqIoVirtualAddress OPTIONAL,
	IN  void				*PhysBufferList OPTIONAL,
	IN  uint64				NumPhysBuffers OPTIONAL,
	IN	uint32				IoVaOffset OPTIONAL,
	IN  IB_HANDLE			PdHandle OPTIONAL,
	IN  IB_ACCESS_CONTROL	AccessControl OPTIONAL,
	OUT IB_HANDLE			*NewMrHandle,
	OUT IB_VIRT_ADDR		*IoVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
IBA_API VPI_MODIFYPHYSMEMREGION2 iba_modify_pmr;

/*
 * ModifyContigPhysMemRegion/iba_modify_contig_pmr
 *
 * Modifies the attributes of an existing Memory Region previously created
 * by RegisterPhysMemRegionr RegisterContigPhysMemRegion.
 * Any combination attributes may be modified by
 * the call. The attributes are the physical buffer list, Protection Domain,
 * and Access Control selectors.
 *
 * If the Protection Domain handle is null for a modify Protection Domain
 * request, the effect is to make the Memory Region inaccessible for all
 * work request operations until a Protection Domain handle is provided on
 * a subsequent ModifyPhysMemRegion call.
 *
 * INPUTS:
 *	MrHandle - Handle to an Memory Region. Upon completion of this call this
 *		handle is no longer valid, regardless of the return status of the call.
 *	ModifyReqType - Indicates an attribute or a combination of attributes to
 *		modify.
 *	ReqIoVirtualAddress - Virtual address of the first byte of the memory
 *		area to register.
 *	NumPhysBuffers - With the Virtual Address, defines the memory area to register.
 *	IoVaOffset - Offset into the first buffer of the region's IoVirtualAddress.
 *	ProtectionDomain - Handle of the Protection Domain to be used to register
 *		the memory.
 *	AccessControl - Selectors indicating the access control attributes for the
 *		Memory Region.
 *
 * OUTPUTS:
 *	NewMrHandle - The handle to the modified Memory Region. This handle must be
 *		used on future references to the Memory Region. On error this may be
 *		null, indicating no Memory Region was successfully returned and the
 *		previous Memory Region is no longer registered.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_MODIFYCONTIGPHYSMEMREGION2)(
	IN  IB_HANDLE			MrHandle,
	IN  IB_MR_MODIFY_REQ	ModifyReqType,
	IN	IB_VIRT_ADDR		ReqIoVirtualAddress OPTIONAL,
	IN  IB_MR_PHYS_BUFFER	*PhysBufferList OPTIONAL,
	IN  uint64				NumPhysBuffers OPTIONAL,
	IN  uint32				IoVaOffset OPTIONAL,
	IN  IB_HANDLE			PdHandle OPTIONAL,
	IN  IB_ACCESS_CONTROL	AccessControl OPTIONAL,
	OUT IB_HANDLE			*NewMrHandle,
	OUT IB_VIRT_ADDR		*IoVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
IBA_API VPI_MODIFYCONTIGPHYSMEMREGION2 iba_modify_contig_pmr;

/*RegisterSharedMemRegion/iba_register_smr
 * 
 *create a new IBA memory region mapping to physical pages representing
 *the original memory region (with either same or different access
 * rights based on the specified access control word).
 * For this one special API, the MrHandle given does not need to be from
 * the same process nor protection domain as the PdHandle for which the
 * new shared memory region will be created.  User space MR handles
 * can be interchanged between user processes and Kernel space MR handles
 * can be interchanges between drivers/PDs/CA instances.  However user space
 * handles cannot be interchanged with kernel space MR handles.
 *
 * INPUTS:
 *	.		MrHandle		handle of the original memory region
 *			VirtualAddress	virtual address of the shared memory region.
 *			PdHandle		PD handle
 *			AccessControl	Access Control word 
 *
 * OUTPUTS:
 *			SharedMrHandle	handle associated with the shared memory region.
 *			Lkey			pointer to the local key returned by the
 *							function
 *			Rkey			pointer to the remote key returned by the
 *							function.
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the shared memory region was created 
 *			successfully else a relevant error status.
 */
typedef FSTATUS (VPI_REGISTERSHAREDMEMREGION)(
	IN  IB_HANDLE			MrHandle,
	IN  void				*VirtualAddress,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ACCESS_CONTROL	AccessControl,
	OUT IB_HANDLE			*SharedMrHandle,
	OUT void				**SharedVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
VPI_REGISTERSHAREDMEMREGION RegisterSharedMemRegion;	/* internal use only, depricated */

/* Version2 of API, supports 64 bit IO virtual addresses */
typedef FSTATUS (VPI_REGISTERSHAREDMEMREGION2)(
	IN  IB_HANDLE			MrHandle,
	IN  IB_VIRT_ADDR		VirtualAddress,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ACCESS_CONTROL	AccessControl,
	OUT IB_HANDLE			*SharedMrHandle,
	OUT IB_VIRT_ADDR		*SharedVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	);
IBA_API VPI_REGISTERSHAREDMEMREGION2 iba_register_smr;

/*CreateMemWindow/iba_create_mw
 * 
 *create a memory window, that can be bound to a 'registered' memory region.
 *
 * INPUTS:
 *	.		CaHandle		handle of the channel adapter 
 *			PdHandle		PD handle
 *
 * OUTPUTS:
 *			MwHandle		pointer to the handle of the memory	window 
 *			Rkey			pointer to the remote key returned by the
 *							function.
 *
 * RETURNS:
 *			Status 'FSUCCESS' if the memory window was created successfully 
 *			else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_CREATEMEMWINDOW)(
	IN  IB_HANDLE	CaHandle,
	IN  IB_HANDLE	PdHandle,
	OUT IB_HANDLE	*MwHandle,
	OUT IB_R_KEY	*Rkey
	);
IBA_API VPI_CREATEMEMWINDOW iba_create_mw;

/*QueryMemWindow/iba_query_mw
 * 
 *query the attributes of a previously created  memory window.
 *
 * INPUTS:
 *			MwHandle			memory window handle.
 *
 * OUTPUTS:
 *			PdHandle			pointer to the PD handle
 *			Rkey				pointer to the remote key	
 * RETURNS:
 *			Status 'FSUCCESS' if the memory window attributes were retrieved
 *			successfuly else a relevant error status.
 * 
 */
typedef FSTATUS (VPI_QUERYMEMWINDOW)(
	IN  IB_HANDLE	MwHandle,
	OUT IB_HANDLE	*PdHandle,
	OUT IB_R_KEY	*Rkey
	);
IBA_API VPI_QUERYMEMWINDOW iba_query_mw;



/*
 * PostMemWindowBind/iba_post_bind_mw
 *
 * Posts a Memory Window Bind work request to the specified QP. This operation
 * will consume a Work Queue Element (WQE) from the Send Q and thus is
 * equivalent to a PostSend request in consuming a WQE. The work
 * completion for the work request is returned through PollCQ.
 *
 * INPUTS:
 *	QpHandle - The handle of the QP. The bind work request is posted to the
 *		Send Q of the QP.
 *	WorkReqId - Identifier supplied by the consumer for the consumer's use in
 *		associating a subsequent work completion with this work request. Not
 *		used by the Verbs Provider.
 *	MwHandle - The handle of the Memory Window.
 *	CurrentRKey - The current Rkey for the Memory Window. This Rkey was
 *		returned from the previous bind request if this is a subsequent bind
 *		request for the Memory Window, or from the Create Memory Window
 *		operation if this is the first bind on this Memory Window.
 *	MrHandle - The handle of the Memory Region the Memory Window is to be
 *		bound to. (passed through but generally ignored)
 *	MrLkey - Lkey of the Memory Region.
 *	VirtualAddress - Virtual Address of base of memory window region.
 *	Length - Length (in bytes) of the window.
 *	AccessControl - Selects RdmaWrite and/or RdmaRead enable bits.
 *	Options - Allows selection of SignaledCompletion and Fence bits. Fence bit
 *		only applies on an Unreliable Connected QP.
 *
 * OUTPUTS:
 *	NewRKey - The new Rkey for the bound Memory Window to be used in a
 *		subsequent send or memory window bind work request referencing this
 *		memory window.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_POSTMEMWINDOWBIND)(
	IN  IB_HANDLE			QpHandle,
	IN  uint64				WorkReqId,
	IN  IB_HANDLE			MwHandle,
	IN  IB_R_KEY			CurrentRkey,
	IN  IB_HANDLE			MrHandle,
	IN  IB_L_KEY			MrLkey,
	IN  void				*VirtualAddress,
	IN  uintn				Length,
	IN  IB_ACCESS_CONTROL	AccessControl,
	IN  IB_MW_BIND_OPTIONS	Options,
	OUT IB_R_KEY			*NewRkey
	);
VPI_POSTMEMWINDOWBIND PostMemWindowBind;	/* internal use only, depricated */


/* Version2 of API, supports 64 bit IO virtual addresses */
typedef FSTATUS (VPI_POSTMEMWINDOWBIND2)(
	IN  IB_HANDLE			QpHandle,
	IN  uint64				WorkReqId,
	IN  IB_HANDLE			MwHandle,
	IN  IB_R_KEY			CurrentRkey,
	IN  IB_HANDLE			MrHandle,
	IN  IB_L_KEY			MrLkey,
	IN  IB_VIRT_ADDR		VirtualAddress,
	IN  uint64				Length,
	IN  IB_ACCESS_CONTROL	AccessControl,
	IN  IB_MW_BIND_OPTIONS	Options,
	OUT IB_R_KEY			*NewRkey
	);
IBA_API VPI_POSTMEMWINDOWBIND2 iba_post_bind_mw;


/*DestroyMemWindow/iba_destroy_mw
 *
 *destroy the previously registered memory window.
 *
 * INPUTS:
 *			MwHandle		handle of the memory window			 
 *
 * OUTPUTS:
 *	.		None	
 *
 * RETURNS:
 *	Status 'FSUCCESS' if the memory window was destroyed
 *	successfully else a relevant error status.
 */
typedef FSTATUS (VPI_DESTROYMEMWINDOW)(
	IN  IB_HANDLE	MwHandle
	);
IBA_API VPI_DESTROYMEMWINDOW iba_destroy_mw;

/*PostSend/iba_post_send
 *post a work request to the 'send' queue of a specific queue pair.
 * this is depricated, use iba_post_send2
 *
 * INPUTS:
 *		QpHandle		Handle of the queue pair
 *		WorkRequest		Pointer to the work request structure				
 *						data pointed to only used during duration of call
 *
 * OUTPUTS:
 *			None		
 *
 * RETURNS:
 *		Status 'FSUCCESS' if the request was posted successfully else
 *		a relevant error status.
 */
typedef FSTATUS (VPI_POSTSEND)(
	IN  IB_HANDLE	QpHandle,
	IN  IB_WORK_REQ	*WorkRequest
	);
IBA_API VPI_POSTSEND iba_post_send;

/*iba_post_send2
 *post a list of work requests to the 'send' queue of a specific queue pair.
 *
 *
 * INPUTS:
 *		QpHandle		Handle of the queue pair
 *		WorkRequestList	Pointer to the linked list of work request structures
 *						data pointed to only used during duration of call
 *
 * OUTPUTS:
 *		FailedWorkRequest Pointer to 1st WorkRequest which failed
 *
 * RETURNS:
 *		Status 'FSUCCESS' if the request was posted successfully else
 *		a relevant error status.  On failure FailedWorkRequest is set to point
 *		to the WorkRequest which failed. On success FailedWorkRequest is not
 *		altered
 */
typedef FSTATUS (VPI_POSTSEND2)(
	IN  IB_HANDLE	QpHandle,
	IN  IB_WORK_REQ2	*WorkRequestList,
	OUT IB_WORK_REQ2	**FailedWorkRequest OPTIONAL
	);
IBA_API VPI_POSTSEND2 iba_post_send2;

/*PostRecv/iba_post_recv
 *
 *post a work request to the 'receive' queue of a specific queue pair.
 * this is depricated, use iba_post_recv2
 *
 * INPUTS:
 *		QpHandle		Handle of the queue pair
 *		WorkRequest		Pointer to the work request structure				
 *						data pointed to only used during duration of call
 *
 * OUTPUTS:
 *			None		
 *
 * RETURNS:
 *		Status 'FSUCCESS' if the  request was posted successfully else
 *		a relevant error status.
 */
typedef FSTATUS (VPI_POSTRECV)(
	IN  IB_HANDLE	QpHandle,
	IN  IB_WORK_REQ	*WorkRequest
	);
IBA_API VPI_POSTRECV iba_post_recv;


/*iba_post_recv2
 *post a list of work requests to the 'recv' queue of a specific queue pair.
 *
 *
 * INPUTS:
 *		QpHandle		Handle of the queue pair
 *		WorkRequestList	Pointer to the linked list of work request structures
 *						data pointed to only used during duration of call
 *
 * OUTPUTS:
 *		FailedWorkRequest Pointer to 1st WorkRequest which failed
 *
 * RETURNS:
 *		Status 'FSUCCESS' if the request was posted successfully else
 *		a relevant error status.  On failure FailedWorkRequest is set to point
 *		to the WorkRequest which failed. On success FailedWorkRequest is not
 *		altered
 */
typedef FSTATUS (VPI_POSTRECV2)(
	IN  IB_HANDLE	QpHandle,
	IN  IB_WORK_REQ2	*WorkRequestList,
	OUT IB_WORK_REQ2	**FailedWorkRequest OPTIONAL
	);
IBA_API VPI_POSTRECV2 iba_post_recv2;

/*
 * PollCQ/iba_poll_cq
 *
 * Checks for a work completion on the given Completion Queue. If one is
 * present the WorkCompletion structure passed to the call is filled in with
 * completion information for the completed request.
 *
 * INPUTS:
 *	CqHandle - Handle to a Completion Queue.
 *
 * OUTPUTS:
 *	WorkCompletion - The caller passes a pointer to a work completion structure.
 *		If the function returns FSUCCESS the structure contains information for
 *		a completed work request.
 *
 * RETURNS:
 *	FSUCCESS - A work completion is returned.
 *	FNOT_DONE - There is no work completion.
 *	FINVALID_PARAMETER - One of the parameters was invalid.
 *
 * Does not prempt
 */

typedef FSTATUS (VPI_POLLCQ)(
	IN  IB_HANDLE			CqHandle,
	OUT IB_WORK_COMPLETION	*WorkCompletion
	);
IBA_API VPI_POLLCQ iba_poll_cq;


/*
 * PeekCQ/iba_peek_cq
 *
 * Determines if the given Completion Queue has at least NumWC completions
 * queued to it.
 *
 * INPUTS:
 *	CqHandle - Handle to a Completion Queue.
 *	NumWC - number of CQEs to check for
 *
 * OUTPUTS:
 *
 * RETURNS:
 *	FSUCCESS - >= NumWC on completion Queue
 *	FNOT_DONE - < NumWC on completion Queue
 *	FINSUFFICIENT_RESOURCES - NumWC > Size of Completion Queue
 *	FINVALID_PARAMETER - One of the parameters was invalid.
 *
 * Does not prempt
 */

typedef FSTATUS (VPI_PEEKCQ)(
	IN  IB_HANDLE			CqHandle,
	IN	uint32				NumWC
	);
IBA_API VPI_PEEKCQ iba_peek_cq;

/*
 * PollCQAndRearmCQ/iba_poll_and_rearm_cq
 *
 * Optimized combination of PollCQ and RearmCQ for use in completion callbacks.
 * Checks for a work completion on the given Completion Queue. If one is
 * present the WorkCompletion structure passed to the call is filled in with
 * completion information for the completed request. Also Rearms the CQ
 * after processing last entry on CQ
 *
 * INPUTS:
 *	CqHandle - Handle to a Completion Queue.
 *	EventSelect - Events to rearm CQ for
 *
 * OUTPUTS:
 *	WorkCompletion - The caller passes a pointer to a work completion structure.
 *		If the function returns FSUCCESS, FCOMPLETED or FPOLL_NEEDED
 *		the structure contains information for
 *		a completed work request.
 *
 * RETURNS:
 *	FSUCCESS - A work completion is returned.  Caller should call
 *				PollAndRearmCQ again.  CQ has not been rearmed.
 *	FCOMPLETED - A work completion is returned.  All CQEs have been
 *				processed, caller need not call PollCQ nor PollAndRearmCQ
 *				CQ has been rearmed.
 *	FPOLL_NEEDED - A work completion is returned.  The CQ has been Rearmed,
 *				caller must loop on PollCQ to drain rest of CQ.
 *	FNOT_DONE - There is no work completion. CQ has been rearmed.
 *	FINVALID_PARAMETER - One of the parameters was invalid.
 *
 * Does not prempt
 */

typedef FSTATUS (VPI_POLLANDREARMCQ2)(
	IN  IB_HANDLE			CqHandle,
	IN	IB_CQ_EVENT_SELECT	EventSelect,
	OUT IB_WORK_COMPLETION	*WorkCompletion
	);
IBA_API VPI_POLLANDREARMCQ2 iba_poll_and_rearm_cq;


/*RearmCQ/iba_rearm_cq
 *
 *arm a specific CQ so that a callback will occur the next time
 *the hardware adds a work completion to the CQ
 *
 * INPUTS:
 *		CqHandle			Handle of the CQ
 *		EventSelect			specifies a filter for the types of events 
 *							that would be allowed to generate callbacks  
 *
 * OUTPUTS:
 *
 *
 * RETURNS:
 *		status 'FSUCCESS' if the CQ was armed successfully
 *		else a relevant error status.
 *		
 * Does not prempt
 */
typedef FSTATUS (VPI_REARMCQ)(
	IN  IB_HANDLE			CqHandle,
	IN	IB_CQ_EVENT_SELECT	EventSelect
	);
IBA_API VPI_REARMCQ iba_rearm_cq;

/*RearmNCQ/iba_rearm_n_cq
 *
 *arm a specific CQ so that a callback will occur once the hardware has
 *added NumWC work completions to the CQ.
 *If the CQ already has NumWC completions, a callback may be generated
 *however this behaviour can be hardware specific
 *
 * INPUTS:
 *		CqHandle			Handle of the CQ
 *		NumWC				How many Work completions to wait for before
 *							generating a callback.
 *
 * OUTPUTS:
 *
 * RETURNS:
 *		status 'FSUCCESS' if the CQ was armed successfully
 *		else a relevant error status.
 *		
 * Does not prempt
 */
typedef FSTATUS (VPI_REARMNCQ)(
	IN  IB_HANDLE			CqHandle,
	IN	uint16				NumWC
	);
IBA_API VPI_REARMNCQ iba_rearm_n_cq;

#if defined(VXWORKS)
/*
 * Private interfaces for use by the SMA and CM
 */

/* Process a MAD Packet for the SMA or PMA.
 * The SMA and PMA in IbAccess use this function to process all the
 * MAD packets they receive.
 *
 * Arguments:
 * 	qp_handle - QP packet arrived on (QP0/QP1), implies HFI
 * 	port_number - Port number packet arrived on (1 to N)
 * 	SLID - LID who sent us packet, special value of 0 indicates our own firmware
 *		sent it to us for us to send out on QP0
 * 	MadInOut - Mad packet, the response should be composed here
 *			data pointed to only used during duration of call
 *
 * Returns:
 * 	FSUCCESS - MadArg contains an appropriate response packet
 *	other - packet could not be processed and no response should be sent
 */
typedef FSTATUS (VPI_GETSETMAD)(
	IN  IB_HANDLE			QpHandle,
	IN  uint8				PortNumber,
	IN STL_LID_32			SLID,
	IN STL_SMP				*SmpInOut,
	IN uint32				*SmpLength
	);
VPI_GETSETMAD iba_get_set_mad;


/*
 * RegisterConnMgr
 *
 * Registers the Connection Manager for the given CA.
 * Async Events appropriate to CM will be sent to this
 * open CA instance even for QPs not part of the CA instance itself
 *
 * INPUTS:
 *	CaHandle - Handle to an open instance of a CA.
 *
 * OUTPUTS:
 *	None.
 *
 * RETURNS:
 *
 */

typedef FSTATUS (VPI_REGISTERCONNMGR)(
	IN IB_HANDLE				CaHandle
	);
VPI_REGISTERCONNMGR iba_register_cm;
#endif /* defined(VXWORKS) */

/*
 * Private interface to support the user-mode Verbs Provider component
 * Each HFI's UVP and its corresponding kernel VPD can define the
 * private info they need to handshake such that kernel bypass operations
 * can be performed without calling the kernel VPD
 * The private info can be input and/or output data.  It is up to the
 * HFI specific UVP as to how and when these calls are made
 *
 * These calls are not for end user applications
 */

/*QueryCAPrivateInfo/iba_query_ca_private_info
 *This function is exported by the UVCA to let the caller
 *(specifically UVP components) extract the hardware specific information 
 *of the specified channel adapter.This functions provides a means for 
 *the UVP module to privately communicate with its kernel mode 
 *counter part the VPD. The intermediate components including the UVCA act
 *the carriers of data communicated between these two modules
 *
 * INPUTS:
 *			CaHandle		UVCA specific handle of the channel adapter.	 
 *			OutBufSize		pointer to the size of the buffer allocated by
 *							the caller (UVP).
 *
 * OUTPUTS:
 *			OutBufSize	.	pointer to the number of bytes returned by the VPD.
 *			OutBuf			pointer the data buffer.On successful return from 
 *							this function, the buffer will be filled with 
 *							hardware specific information of the specified 
 *							channel adapter.		
 *
 * RETURNS:
 *		status 'FSUCCESS' if the  channel adapter specific information was 
 *		successfully retrieved else a relevant error status.
 * 
 *					
 */
typedef FSTATUS
(VPI_QUERYCA_PRIVATE_INFO)(
	IN  IB_HANDLE	CaHandle,
	IN OUT uint32	*BufferSize,
	OUT void		*Buffer
	);
IBA_API VPI_QUERYCA_PRIVATE_INFO iba_query_ca_private_info;

/*QueryPDPrivateInfo/iba_query_pd_private_info
 *This function is exported by the UVCA to let the caller
 *(specifically UVP components) extract the hardware specific information 
 *of the specified PD. This functions provides a means for 
 *the UVP module to privately communicate with its kernel mode 
 *counter part the VPD. The intermediate components including the UVCA act
 *the carriers of data communicated between these two modules
 *
 * INPUTS:
 *			PdHandle		UVCA specific handle of the PD.	 
 *			OutBufSize		pointer to the size of the buffer allocated by
 *							the caller (UVP).
 *
 * OUTPUTS:
 *			OutBufSize	.	pointer to the number of bytes returned by the VPD.
 *			OutBuf			pointer the data buffer.On successful return from 
 *							this function, the buffer will be filled with 
 *							hardware specific information of the specified 
 *							PD.		
 *
 * RETURNS:
 *		status 'FSUCCESS' if the PD specific information was 
 */
typedef FSTATUS
(VPI_QUERYPD_PRIVATE_INFO)(
	IN  IB_HANDLE	PdHandle,
	IN OUT uint32	*BufferSize,
	OUT void		*Buffer
	);
IBA_API VPI_QUERYPD_PRIVATE_INFO iba_query_pd_private_info;

/*QueryQPPrivateInfo/iba_query_qp_private_info
 *This function is exported by the UVCA to let the caller
 *(specifically UVP components) extract the hardware specific information 
 *of the specified queue pair. This functions provides a means for 
 *the UVP module to privately communicate with its kernel mode 
 *counter part the VPD. The intermediate components including the UVCA act
 *the carriers of data communicated between these two modules
 *
 * INPUTS:
 *			QpHandle		UVCA specific handle of the queue pair.	 
 *			OutBufSize		pointer to the size of the buffer allocated by
 *							the caller (UVP).
 *
 * OUTPUTS:
 *			OutBufSize	.	pointer to the number of bytes returned by the VPD.
 *			OutBuf			pointer the data buffer.On successful return from 
 *							this function, the buffer will be filled with 
 *							hardware specific information of the specified 
 *							queue pair.		
 *
 * RETURNS:
 *		status 'FSUCCESS' if the queue pair specific information was 
 */
typedef FSTATUS
(VPI_QUERYQP_PRIVATE_INFO)(
	IN  IB_HANDLE	QpHandle,
	IN OUT uint32	*BufferSize,
	OUT void		*Buffer
	);
IBA_API VPI_QUERYQP_PRIVATE_INFO iba_query_qp_private_info;

/*QueryCQPrivateInfo/iba_query_cq_private_info
 *This function is exported by the UVCA to let the caller
 *(specifically UVP components) extract the hardware specific information 
 *of the specified completion queue. This functions provides a means for 
 *the UVP module to privately communicate with its kernel mode 
 *counter part the VPD. The intermediate components including the UVCA act
 *the carriers of data communicated between these two modules
 *
 * INPUTS:
 *			CqHandle		UVCA specific handle of the completion queue.	 
 *			OutBufSize		pointer to the size of the buffer allocated by
 *							the caller (UVP).
 *
 * OUTPUTS:
 *			OutBufSize	.	pointer to the number of bytes returned by the VPD.
 *			OutBuf			pointer the data buffer.On successful return from 
 *							this function, the buffer will be filled with 
 *							hardware specific information of the specified 
 *							completion queue.		
 *
 * RETURNS:
 *		status 'FSUCCESS' if the  completion queue specific information was 
 *		successfully retrieved else a relevant error status.
 * 
 */
typedef FSTATUS
(VPI_QUERYCQ_PRIVATE_INFO)(
	IN  IB_HANDLE	CqHandle,
	IN OUT uint32	*BufferSize,
	OUT	void		*Buffer
	);
IBA_API VPI_QUERYCQ_PRIVATE_INFO iba_query_cq_private_info;

/*QueryAVPrivateInfo/iba_query_av_private_info
 *This function is exported by the UVCA to let the caller
 *(specifically UVP components) extract the hardware specific information 
 *of the specified Address Vector. This functions provides a means for 
 *the UVP module to privately communicate with its kernel mode 
 *counter part the VPD. The intermediate components including the UVCA act
 *the carriers of data communicated between these two modules
 *
 * INPUTS:
 *			AvHandle		UVCA specific handle of the adress vector.	 
 *			OutBufSize		pointer to the size of the buffer allocated by
 *							the caller (UVP).
 *
 * OUTPUTS:
 *			OutBufSize	.	pointer to the number of bytes returned by the VPD.
 *			OutBuf			pointer the data buffer.On successful return from 
 *							this function, the buffer will be filled with 
 *							hardware specific information of the specified 
 *							address vector.		
 *
 * RETURNS:
 *		status 'FSUCCESS' if the  address vector specific information was 
 *		successfully retrieved else a relevant error status.
 * 
 *					
 */
typedef FSTATUS
(VPI_QUERYAV_PRIVATE_INFO)(
	IN  IB_HANDLE	AvHandle,
	IN OUT uint32	*BufferSize,
	OUT void		*Buffer
	);
IBA_API VPI_QUERYAV_PRIVATE_INFO iba_query_av_private_info;

#ifdef __cplusplus
};
#endif


#endif /* _IBA_VPI_EXPORT_H_ */
