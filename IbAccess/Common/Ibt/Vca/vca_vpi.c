/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/


#include "precomp.h"

FSTATUS
iba_open_ca(
	IN  EUI64					CaGUID,
	IN	IB_COMPLETION_CALLBACK	CompletionCallback,
	IN	IB_ASYNC_EVENT_CALLBACK	AsyncEventCallback,
	IN	void					*Context,
	OUT	IB_HANDLE				*CaHandle
	)
{
	DEVICE_INFO	*DevInfo;

	DevInfo = vcaFindCA(CaGUID);

	if ( DevInfo == NULL )
		return FINVALID_PARAMETER;
	return vcaOpenCa(DevInfo, CompletionCallback, AsyncEventCallback,
					Context, CaHandle);
}

// internal function so we can open a CA and start using it before
// adding DevInfo to the global list
FSTATUS
vcaOpenCa(
	IN  DEVICE_INFO				*DevInfo,
	IN	IB_COMPLETION_CALLBACK	CompletionCallback,
	IN	IB_ASYNC_EVENT_CALLBACK	AsyncEventCallback,
	IN	void					*Context,
	OUT	IB_HANDLE				*CaHandle
	)
{
	VCA_CA_CONTEXT	*vcaCaContext;
	FSTATUS	status;

	if ( (vcaCaContext = (VCA_CA_CONTEXT*)MemoryAllocate2(sizeof(VCA_CA_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	MemoryClear(vcaCaContext, sizeof(VCA_CA_CONTEXT));

	status = DevInfo->VpInterface.OpenCA(DevInfo->VpInterface.CaGUID,
									CompletionCallback,
									AsyncEventCallback,
									Context,
									&vcaCaContext->VpHandle);

	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle
		vcaCaContext->VpInterface = &DevInfo->VpInterface;
		*CaHandle = (IB_HANDLE)vcaCaContext;
	} else {
		MemoryDeallocate(vcaCaContext);
	}

	return status;
}

FSTATUS
iba_query_ca(
	IN  IB_HANDLE			CaHandle,
	OUT IB_CA_ATTRIBUTES	*CaAttributes,
	OUT	void				**Context
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;

	return vcaCaHandle->VpInterface->QueryCA(vcaCaHandle->VpHandle,
									CaAttributes,
									Context);
}

FSTATUS
iba_query_ca2(
	IN  IB_HANDLE			CaHandle,
	OUT IB_CA_ATTRIBUTES2	*CaAttributes,
	OUT	void				**Context
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;

	return vcaCaHandle->VpInterface->QueryCA2(vcaCaHandle->VpHandle,
									CaAttributes,
									Context);
}

FSTATUS
iba_set_compl_handler(
	IN  IB_HANDLE				CaHandle,
	IN  IB_COMPLETION_CALLBACK	CompletionCallback
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;

	return vcaCaHandle->VpInterface->SetCompletionHandler(vcaCaHandle->VpHandle,
									CompletionCallback);
}


FSTATUS
iba_set_async_event_handler(
	IN  IB_HANDLE				CaHandle,
	IN  IB_ASYNC_EVENT_CALLBACK	AsyncEventCallback
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;

	return vcaCaHandle->VpInterface->SetAsyncEventHandler(
									vcaCaHandle->VpHandle,
									AsyncEventCallback);
}

FSTATUS
iba_set_ca_context(
	IN	IB_HANDLE	CaHandle,
	IN	void		*Context
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;

	return vcaCaHandle->VpInterface->SetCAContext(
									vcaCaHandle->VpHandle,
									Context);
}

FSTATUS
SetCAInternal(
	IN	IB_HANDLE	CaHandle
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;

	return vcaCaHandle->VpInterface->SetCAInternal(vcaCaHandle->VpHandle);
}

FSTATUS
iba_modify_ca(
	IN  IB_HANDLE			CaHandle,
	IN	IB_CA_ATTRIBUTES	*CaAttributes
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;

	return vcaCaHandle->VpInterface->ModifyCA(vcaCaHandle->VpHandle, CaAttributes);
}

FSTATUS
iba_close_ca(
	IN  IB_HANDLE	CaHandle
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;
	FSTATUS	status;

	status = vcaCaHandle->VpInterface->CloseCA(vcaCaHandle->VpHandle);

	if ( status == FSUCCESS )
	{
		// Clear context so that future references fail
		MemoryClear(vcaCaHandle, sizeof(VCA_CA_CONTEXT));
		MemoryDeallocate(vcaCaHandle);
	}

	return status;
}

// depricated interface
FSTATUS
AllocatePD(
	IN  IB_HANDLE	CaHandle,
	OUT IB_HANDLE	*PdHandle
	)
{
	return iba_alloc_pd(CaHandle, DEFAULT_MAX_AVS_PER_PD, PdHandle);
}

FSTATUS
iba_alloc_pd(
	IN  IB_HANDLE	CaHandle,
	IN	uint32		MaxAVs,
	OUT IB_HANDLE	*PdHandle
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;
	VCA_PD_CONTEXT	*vcaPdContext;
	FSTATUS	status;

	if ( (vcaPdContext = (VCA_PD_CONTEXT*)MemoryAllocate2(sizeof(VCA_PD_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	MemoryClear(vcaPdContext, sizeof(VCA_PD_CONTEXT));

	status = vcaCaHandle->VpInterface->AllocatePD(vcaCaHandle->VpHandle,
									MaxAVs,
									&vcaPdContext->VpHandle);

	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle
		vcaPdContext->VpInterface = vcaCaHandle->VpInterface;
		*PdHandle = (IB_HANDLE)vcaPdContext;
	} else {
		MemoryDeallocate(vcaPdContext);
	}

	return status;
}


FSTATUS
iba_alloc_sqp_pd(
	IN  IB_HANDLE	CaHandle,
	IN	uint32		MaxAVs,
	OUT IB_HANDLE	*PdHandle
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;
	VCA_PD_CONTEXT	*vcaPdContext;
	FSTATUS	status;

	if ( (vcaPdContext = (VCA_PD_CONTEXT*)MemoryAllocate2(sizeof(VCA_PD_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	MemoryClear(vcaPdContext, sizeof(VCA_PD_CONTEXT));

	if (! vcaCaHandle->VpInterface->AllocateSqpPD)
	{
		status = vcaCaHandle->VpInterface->AllocatePD(vcaCaHandle->VpHandle,
									MaxAVs,
									&vcaPdContext->VpHandle);
	} else {
		status = vcaCaHandle->VpInterface->AllocateSqpPD(vcaCaHandle->VpHandle,
									MaxAVs,
									&vcaPdContext->VpHandle);
	}

	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle
		vcaPdContext->VpInterface = vcaCaHandle->VpInterface;
		*PdHandle = (IB_HANDLE)vcaPdContext;
	} else {
		MemoryDeallocate(vcaPdContext);
	}

	return status;
}


FSTATUS
iba_free_pd(
	IN  IB_HANDLE	PdHandle
	)
{
	VCA_PD_CONTEXT	*vcaPdHandle = (VCA_PD_CONTEXT *)PdHandle;
	FSTATUS	status;

	status = vcaPdHandle->VpInterface->FreePD(vcaPdHandle->VpHandle);

	if (status == FSUCCESS)
	{
		// Clear context so that future references fail
		MemoryClear(vcaPdHandle, sizeof(VCA_PD_CONTEXT));
		MemoryDeallocate(vcaPdHandle);
	}
	return status;
}

FSTATUS
iba_alloc_rdd(
	IN  IB_HANDLE	CaHandle,
	OUT IB_HANDLE	*RddHandle
	)
{
	return FERROR;
}

FSTATUS
iba_free_rdd(
	IN  IB_HANDLE	RddHandle
	)
{
	return FERROR;
}

FSTATUS
iba_create_av(
	IN  IB_HANDLE			CaHandle,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ADDRESS_VECTOR	*AddressVector,
	OUT IB_HANDLE			*AvHandle
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;
	VCA_PD_CONTEXT	*vcaPdHandle = (VCA_PD_CONTEXT *)PdHandle;
	VCA_AV_CONTEXT	*vcaAvContext;
	FSTATUS	status;

	if ( (vcaAvContext = (VCA_AV_CONTEXT*)MemoryAllocate2(sizeof(VCA_AV_CONTEXT), IBA_MEM_FLAG_NONE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	MemoryClear(vcaAvContext, sizeof(VCA_AV_CONTEXT));

	status = vcaCaHandle->VpInterface->CreateAV(
									vcaCaHandle->VpHandle,
									vcaPdHandle->VpHandle,
									AddressVector,
									&vcaAvContext->VpHandle);

	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle
		vcaAvContext->VpInterface	= vcaCaHandle->VpInterface;
		vcaAvContext->VcaPdHandle	= vcaPdHandle;
		*AvHandle = (IB_HANDLE)vcaAvContext;
	} else {
		MemoryDeallocate(vcaAvContext);
	}

	return status;
}

FSTATUS
iba_modify_av(
	IN  IB_HANDLE			AvHandle,
	IN  IB_ADDRESS_VECTOR	*AddressVector
	)
{
	VCA_AV_CONTEXT	*vcaAvHandle = (VCA_AV_CONTEXT *)AvHandle;

	return vcaAvHandle->VpInterface->ModifyAV(vcaAvHandle->VpHandle,
										AddressVector);
}


FSTATUS
iba_query_av(
	IN  IB_HANDLE			AvHandle,
	OUT IB_ADDRESS_VECTOR	*AddressVector,
	OUT IB_HANDLE			*PdHandle
	)
{
	FSTATUS status;
	VCA_AV_CONTEXT	*vcaAvHandle = (VCA_AV_CONTEXT *)AvHandle;

	if (	 (status = vcaAvHandle->VpInterface->QueryAV(
										vcaAvHandle->VpHandle,
										AddressVector,
										PdHandle)) == FSUCCESS
			 	
		)
	{
		*PdHandle = vcaAvHandle->VcaPdHandle;
	}
	return status;
}


FSTATUS
iba_destroy_av(
	IN  IB_HANDLE	AvHandle
	)
{
	VCA_AV_CONTEXT	*vcaAvHandle = (VCA_AV_CONTEXT *)AvHandle;
	FSTATUS	status;

	status = vcaAvHandle->VpInterface->DestroyAV(vcaAvHandle->VpHandle);

	if (status == FSUCCESS)
	{
		// Clear context so that future references fail
		MemoryClear(vcaAvHandle, sizeof(VCA_AV_CONTEXT));
		MemoryDeallocate(vcaAvHandle);
	}

	return status;
}

FSTATUS
iba_create_qp(
	IN  IB_HANDLE				CaHandle,
	IN  IB_QP_ATTRIBUTES_CREATE	*QpCreateAttributes,
	IN	void					*Context,
	OUT IB_HANDLE				*QpHandle,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;
	VCA_QP_CONTEXT	*vcaQpContext;
	VCA_PD_CONTEXT	*vcaPdHandle;
	FSTATUS	status;

	if ( (vcaQpContext = (VCA_QP_CONTEXT*)MemoryAllocate2AndClear(sizeof(VCA_QP_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	// Translate PD handle to VP's
	vcaPdHandle = (VCA_PD_CONTEXT *)QpCreateAttributes->PDHandle;
	QpCreateAttributes->PDHandle = vcaPdHandle->VpHandle;

	// Translate the CQ handles to VP's
	vcaQpContext->VcaSendCQHandle =
			(VCA_CQ_CONTEXT *)QpCreateAttributes->SendCQHandle;
	vcaQpContext->VcaRecvCQHandle =
			(VCA_CQ_CONTEXT *)QpCreateAttributes->RecvCQHandle;
	QpCreateAttributes->SendCQHandle = vcaQpContext->VcaSendCQHandle->VpHandle;
	QpCreateAttributes->RecvCQHandle = vcaQpContext->VcaRecvCQHandle->VpHandle;

	status = vcaCaHandle->VpInterface->CreateQP(vcaCaHandle->VpHandle,
									QpCreateAttributes,
									Context,
									&vcaQpContext->VpHandle,
									QpAttributes);

	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle
		vcaQpContext->VpInterface = vcaCaHandle->VpInterface;
		*QpHandle = (IB_HANDLE)vcaQpContext;	// Handle returned is ptr to VCA QP context

		// Save the VCA's PD handle in the VCA's QP context and return it in
		// the QP attributes structure
		vcaQpContext->VcaPdHandle = vcaPdHandle;
		QpAttributes->PDHandle = (IB_HANDLE)vcaPdHandle;

		// Replace the PD handle in the create attributes structure
		QpCreateAttributes->PDHandle = (IB_HANDLE)vcaPdHandle;

		// Replace the CQ handles
		QpCreateAttributes->SendCQHandle = vcaQpContext->VcaSendCQHandle;
		QpCreateAttributes->RecvCQHandle = vcaQpContext->VcaRecvCQHandle;

		// Copy the VCA CQ handles to the Query member of the QP Attributes
		// structure.
		QpAttributes->SendCQHandle = vcaQpContext->VcaSendCQHandle;
		QpAttributes->RecvCQHandle = vcaQpContext->VcaRecvCQHandle;

		// Save the QP type
		vcaQpContext->Type = QpAttributes->Type;
	} else {
		MemoryDeallocate(vcaQpContext);
	}

	return status;
}


FSTATUS
iba_modify_qp(
	IN  IB_HANDLE						QpHandle,
	IN  IB_QP_ATTRIBUTES_MODIFY			*QpModifyAttributes,
	OUT OPTIONAL IB_QP_ATTRIBUTES_QUERY	*QpAttributes
	)
{
	VCA_QP_CONTEXT	*vcaQpHandle = (VCA_QP_CONTEXT *)QpHandle;
	FSTATUS status;

	status = vcaQpHandle->VpInterface->ModifyQP(vcaQpHandle->VpHandle,
										QpModifyAttributes, QpAttributes);
	if ( status == FSUCCESS && QpAttributes)
	{
		// Fill in the translated handles
		QpAttributes->SendCQHandle = vcaQpHandle->VcaSendCQHandle;
		QpAttributes->RecvCQHandle = vcaQpHandle->VcaRecvCQHandle;
		QpAttributes->PDHandle = vcaQpHandle->VcaPdHandle;
	}
	return status;
}


FSTATUS
iba_set_qp_context(
	IN  IB_HANDLE	QpHandle,
	IN  void		*Context
	)
{
	return ((VCA_QP_CONTEXT*)QpHandle)->VpInterface->SetQPContext(
										((VCA_QP_CONTEXT*)QpHandle)->VpHandle,
										Context);
}

FSTATUS
iba_query_qp(
	IN  IB_HANDLE				QpHandle,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes,
	OUT	void					**Context
	)
{
	VCA_QP_CONTEXT	*vcaQpHandle = (VCA_QP_CONTEXT *)QpHandle;
	FSTATUS	status = FSUCCESS;
	
	status = vcaQpHandle->VpInterface->QueryQP(vcaQpHandle->VpHandle,
									QpAttributes,
									Context);

	if ( status == FSUCCESS )
	{
		// Fill in the translated handles
		QpAttributes->SendCQHandle = vcaQpHandle->VcaSendCQHandle;
		QpAttributes->RecvCQHandle = vcaQpHandle->VcaRecvCQHandle;
		QpAttributes->PDHandle = vcaQpHandle->VcaPdHandle;
	}

	return status;
}

FSTATUS
iba_destroy_qp(
	IN  IB_HANDLE	QpHandle
	)
{
	VCA_QP_CONTEXT	*vcaQpHandle = (VCA_QP_CONTEXT *)QpHandle;
	FSTATUS	status;

	status = vcaQpHandle->VpInterface->DestroyQP(vcaQpHandle->VpHandle);

	if (status == FSUCCESS)
	{
		// Clear context so that future references fail
		MemoryClear(vcaQpHandle, sizeof(VCA_QP_CONTEXT));
		MemoryDeallocate(vcaQpHandle);
	}

	return status;
}

FSTATUS
iba_create_special_qp(
	IN  IB_HANDLE				CaHandle,
	IN  EUI64					PortGUID,
	IN  IB_QP_ATTRIBUTES_CREATE	*QpCreateAttributes,
	IN	void					*Context,
	OUT IB_HANDLE				*QpHandle,
	OUT IB_QP_ATTRIBUTES_QUERY	*QpAttributes
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;
	VCA_QP_CONTEXT	*vcaQpContext;
	VCA_PD_CONTEXT	*vcaPdHandle;
	FSTATUS	status;

	if ( (vcaQpContext = (VCA_QP_CONTEXT*)MemoryAllocate2AndClear(sizeof(VCA_QP_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	// Translate PD handle to VP's
	vcaPdHandle = (VCA_PD_CONTEXT *)QpCreateAttributes->PDHandle;
	QpCreateAttributes->PDHandle = vcaPdHandle->VpHandle;

	// Translate the CQ handles to VP's
	vcaQpContext->VcaSendCQHandle =
			(VCA_CQ_CONTEXT *)QpCreateAttributes->SendCQHandle;
	vcaQpContext->VcaRecvCQHandle =
			(VCA_CQ_CONTEXT *)QpCreateAttributes->RecvCQHandle;
	QpCreateAttributes->SendCQHandle = vcaQpContext->VcaSendCQHandle->VpHandle;
	QpCreateAttributes->RecvCQHandle = vcaQpContext->VcaRecvCQHandle->VpHandle;

	status = vcaCaHandle->VpInterface->CreateSpecialQP(vcaCaHandle->VpHandle,
									PortGUID,
									QpCreateAttributes,
									Context,
									&vcaQpContext->VpHandle,
									QpAttributes);

	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle
		vcaQpContext->VpInterface = vcaCaHandle->VpInterface;
		*QpHandle = (IB_HANDLE)vcaQpContext;

		// Save the VCA's PD handle in the VCA's QP context and return it in
		// the QP attributes structure.
		vcaQpContext->VcaPdHandle = vcaPdHandle;
		QpAttributes->PDHandle = (IB_HANDLE)vcaPdHandle;

		// Replace the CQ handles
		QpCreateAttributes->SendCQHandle = vcaQpContext->VcaSendCQHandle;
		QpCreateAttributes->RecvCQHandle = vcaQpContext->VcaRecvCQHandle;

		// Save the QP type
		vcaQpContext->Type = QpAttributes->Type;
	} else {
		MemoryDeallocate(vcaQpContext);
	}
	return status;
}

FSTATUS
iba_attach_qp_to_mcgroup(
	IN IB_HANDLE QpHandle,
#if INCLUDE_16B
	IN STL_LID McgDestLid,
#else
	IN IB_LID McgDestLid,
#endif
	IN IB_GID McgGID
	)
{
	return ((VCA_QP_CONTEXT*)QpHandle)->VpInterface->AttachQPToMulticastGroup(
								((VCA_QP_CONTEXT*)QpHandle)->VpHandle,
								McgDestLid, McgGID);
}

FSTATUS
iba_detach_qp_from_mcgroup(
	IN IB_HANDLE QpHandle,
#if INCLUDE_16B
	IN STL_LID McgDestLid,
#else
	IN IB_LID McgDestLid,
#endif
	IN IB_GID McgGID
	)
{
	return ((VCA_QP_CONTEXT*)QpHandle)->VpInterface->DetachQPFromMulticastGroup(
								((VCA_QP_CONTEXT*)QpHandle)->VpHandle,
								McgDestLid, McgGID);
}


FSTATUS
iba_create_cq(
	IN  IB_HANDLE	CaHandle,
	IN  uint32		CqReqSize,
	IN  void		*Context,
	OUT IB_HANDLE	*CqHandle,
	OUT	uint32		*CqSize
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;
	VCA_CQ_CONTEXT	*vcaCqContext;
	FSTATUS	status;

	if ( (vcaCqContext = (VCA_CQ_CONTEXT*)MemoryAllocate2(sizeof(VCA_CQ_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	MemoryClear(vcaCqContext, sizeof(VCA_CQ_CONTEXT));

	status = vcaCaHandle->VpInterface->CreateCQ(vcaCaHandle->VpHandle,
									CqReqSize, Context,
									&vcaCqContext->VpHandle, CqSize);
	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle
		vcaCqContext->VpInterface = vcaCaHandle->VpInterface;
		*CqHandle = (IB_HANDLE)vcaCqContext;
	} else {
		MemoryDeallocate(vcaCqContext);
	}

	return status;
}


FSTATUS
iba_resize_cq(
	IN  IB_HANDLE	CqHandle,
	IN  uint32		CqReqSize,
	OUT	uint32		*CqSize
	)
{
	return ((VCA_CQ_CONTEXT*)CqHandle)->VpInterface->ResizeCQ(
									((VCA_CQ_CONTEXT*)CqHandle)->VpHandle,
									CqReqSize, CqSize);
}


FSTATUS
iba_set_cq_context(
	IN  IB_HANDLE	CqHandle,
	IN  void		*Context
	)
{
	return ((VCA_CQ_CONTEXT*)CqHandle)->VpInterface->SetCQContext(
									((VCA_CQ_CONTEXT*)CqHandle)->VpHandle,
									Context);
}


FSTATUS
iba_set_cq_compl_handler(
	IN  IB_HANDLE	CqHandle,
	IN  IB_COMPLETION_CALLBACK CompletionCallback
	)
{
	return ((VCA_CQ_CONTEXT*)CqHandle)->VpInterface->SetCQCompletionHandler(
									((VCA_CQ_CONTEXT*)CqHandle)->VpHandle,
									CompletionCallback);
}


FSTATUS
iba_query_cq(
	IN  IB_HANDLE	CqHandle,
	OUT uint32		*CqSize,
	OUT	void		**Context
	)
{
	return ((VCA_CQ_CONTEXT*)CqHandle)->VpInterface->QueryCQ(
									((VCA_CQ_CONTEXT*)CqHandle)->VpHandle,
									CqSize, Context);
}


FSTATUS
iba_destroy_cq(
	IN  IB_HANDLE	CqHandle
	)
{
	VCA_CQ_CONTEXT	*vcaCqHandle = (VCA_CQ_CONTEXT *)CqHandle;
	FSTATUS	status;

	status = vcaCqHandle->VpInterface->DestroyCQ(vcaCqHandle->VpHandle);

	if (status == FSUCCESS)
	{
		// Clear context so that future references fail
		MemoryClear(vcaCqHandle, sizeof(VCA_CQ_CONTEXT));
		MemoryDeallocate(vcaCqHandle);
	}

	return status;
}


FSTATUS
iba_register_mr(
	IN  IB_HANDLE			CaHandle,
	IN  void				*VirtualAddress,
	IN  uintn				Length,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ACCESS_CONTROL	AccessControl,
	OUT IB_HANDLE			*MrHandle,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;
	VCA_PD_CONTEXT	*vcaPdHandle = (VCA_PD_CONTEXT *)PdHandle;
	VCA_MR_CONTEXT	*vcaMrContext;
	FSTATUS	status;

	if ( (vcaMrContext = (VCA_MR_CONTEXT*)MemoryAllocate2(sizeof(VCA_MR_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	MemoryClear(vcaMrContext, sizeof(VCA_MR_CONTEXT));

	status = vcaCaHandle->VpInterface->RegisterMemRegion(vcaCaHandle->VpHandle,
									VirtualAddress,
									Length,
									vcaPdHandle->VpHandle,
									AccessControl,
									&vcaMrContext->VpHandle,
									Lkey,
									Rkey);

	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle. Save
		// the PD handle in the MR handle for the modify and query mem
		// region functions.
		vcaMrContext->VpInterface = vcaCaHandle->VpInterface;
		vcaMrContext->VcaPdHandle = vcaPdHandle;
		*MrHandle = (IB_HANDLE)vcaMrContext;
	} else {
		MemoryDeallocate(vcaMrContext);
	}

	return status;
}

// depricated
FSTATUS
RegisterPhysMemRegion(
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
	)
{
	IB_VIRT_ADDR reqVa = (IB_VIRT_ADDR)(uintn)ReqIoVirtualAddress;
	IB_VIRT_ADDR va;
	FSTATUS	status;

	status = iba_register_pmr(CaHandle, reqVa,
				PhysBufferList, NumPhysBuffers, IoVaOffset, PdHandle,
				AccessControl, MrHandle, &va, Lkey, Rkey);
	if (status == FSUCCESS)
	{
		*IoVirtualAddress = (void*)(uintn)va;
		if ((IB_VIRT_ADDR)(uintn)*IoVirtualAddress != va)
		{
			// we cannot fit the resulting va in a void*
			(void)iba_deregister_mr(*MrHandle);
			status = FERROR;
		}
	}
	return status;
}

FSTATUS
iba_register_pmr(
	IN  IB_HANDLE			CaHandle,
	IN	IB_VIRT_ADDR		ReqIoVirtualAddress,
	IN  void				*PhysBufferList,
	IN  uint64				NumPhysBuffers,
	IN  uint32				IoVaOffset,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ACCESS_CONTROL	AccessControl,
	OUT IB_HANDLE			*MrHandle,
	OUT IB_VIRT_ADDR		*IoVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;
	VCA_PD_CONTEXT	*vcaPdHandle = (VCA_PD_CONTEXT *)PdHandle;
	VCA_MR_CONTEXT	*vcaMrContext;
	FSTATUS	status;

	if ( (vcaMrContext = (VCA_MR_CONTEXT*)MemoryAllocate2(sizeof(VCA_MR_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	MemoryClear(vcaMrContext, sizeof(VCA_MR_CONTEXT));

	status = vcaCaHandle->VpInterface->RegisterPhysMemRegion(vcaCaHandle->VpHandle,
										ReqIoVirtualAddress,
										PhysBufferList,
										NumPhysBuffers,
										IoVaOffset,
										vcaPdHandle->VpHandle,
										AccessControl,
										&vcaMrContext->VpHandle,
										IoVirtualAddress,
										Lkey,
										Rkey);

	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle
		vcaMrContext->VpInterface = vcaCaHandle->VpInterface;
		vcaMrContext->VcaPdHandle = vcaPdHandle;
		*MrHandle = (IB_HANDLE)vcaMrContext;
	} else {
		MemoryDeallocate(vcaMrContext);
	}

	return status;
}

FSTATUS
iba_register_contig_pmr(
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
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;
	VCA_PD_CONTEXT	*vcaPdHandle = (VCA_PD_CONTEXT *)PdHandle;
	VCA_MR_CONTEXT	*vcaMrContext;
	FSTATUS	status;

	if ( (vcaMrContext = (VCA_MR_CONTEXT*)MemoryAllocate2(sizeof(VCA_MR_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	MemoryClear(vcaMrContext, sizeof(VCA_MR_CONTEXT));

	status = vcaCaHandle->VpInterface->RegisterContigPhysMemRegion(vcaCaHandle->VpHandle,
										ReqIoVirtualAddress,
										PhysBufferList,
										NumPhysBuffers,
										IoVaOffset,
										vcaPdHandle->VpHandle,
										AccessControl,
										&vcaMrContext->VpHandle,
										IoVirtualAddress,
										Lkey,
										Rkey);

	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle
		vcaMrContext->VpInterface = vcaCaHandle->VpInterface;
		vcaMrContext->VcaPdHandle = vcaPdHandle;
		*MrHandle = (IB_HANDLE)vcaMrContext;
	} else {
		MemoryDeallocate(vcaMrContext);
	}

	return status;
}

// depricated
FSTATUS
QueryMemRegion(
	IN  IB_HANDLE			MrHandle,
	OUT	void				**VirtualAddress,
	OUT	uintn				*Length,
	OUT IB_HANDLE			*PdHandle,
	OUT IB_ACCESS_CONTROL	*AccessControl,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	)
{
	IB_VIRT_ADDR va;
	uint64 length;
	FSTATUS	status;

	status = iba_query_mr(MrHandle, &va, &length, PdHandle, AccessControl,
							Lkey, Rkey);
	if (status == FSUCCESS)
	{
		// assume Va & Length fits, since must have registered successfully
		*VirtualAddress = (void*)(uintn)va;
		*Length = (uintn)length;
	}
	return status;
}

FSTATUS
iba_query_mr(
	IN  IB_HANDLE			MrHandle,
	OUT	IB_VIRT_ADDR		*VirtualAddress,
	OUT	uint64				*Length,
	OUT IB_HANDLE			*PdHandle,
	OUT IB_ACCESS_CONTROL	*AccessControl,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	)
{
	VCA_MR_CONTEXT	*vcaMrHandle = (VCA_MR_CONTEXT *)MrHandle;
	FSTATUS	status;
	
	status = vcaMrHandle->VpInterface->QueryMemRegion(vcaMrHandle->VpHandle,
										VirtualAddress,
										Length,
										PdHandle,
										AccessControl,
										Lkey,
										Rkey);

	// Set PD handle to the VCA's PD handle
	*PdHandle = (IB_HANDLE)vcaMrHandle->VcaPdHandle;

	return status;
}


FSTATUS
iba_deregister_mr(
	IN  IB_HANDLE	MrHandle
	)
{
	VCA_MR_CONTEXT	*vcaMrHandle = (VCA_MR_CONTEXT *)MrHandle;
	FSTATUS	status;
	
	status = vcaMrHandle->VpInterface->DeregisterMemRegion(vcaMrHandle->VpHandle);

	if (status == FSUCCESS)
	{
		// Clear context so that future references fail
		MemoryClear(vcaMrHandle, sizeof(VCA_MR_CONTEXT));
		MemoryDeallocate(vcaMrHandle);
	}

	return status;
}

FSTATUS
iba_modify_mr(
	IN  IB_HANDLE			MrHandle,
	IN  IB_MR_MODIFY_REQ	ModifyReqType,
	IN  void				*VirtualAddress OPTIONAL,
	IN  uintn				Length OPTIONAL,
	IN  IB_HANDLE			PdHandle OPTIONAL,
	IN  IB_ACCESS_CONTROL	AccessControl OPTIONAL,
	OUT IB_HANDLE			*NewMrHandle,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	)
{
	VCA_MR_CONTEXT	*vcaMrHandle = (VCA_MR_CONTEXT *)MrHandle;
	VCA_PD_CONTEXT	*vcaPdHandle;
	IB_HANDLE		vpPdHandle = NULL;
	FSTATUS	status;

	if ( ModifyReqType.s.ProtectionDomain )
	{
		vcaPdHandle = (VCA_PD_CONTEXT *)PdHandle;
		vpPdHandle = vcaPdHandle->VpHandle;

		// Replace the current PdHandle in the MR context with the new
		// PdHandle. Don't need to worry that the call may fail in the
		// VP as if it does the MR is destroyed below.
		vcaMrHandle->VcaPdHandle = vcaPdHandle;
	}

	status = vcaMrHandle->VpInterface->ModifyMemRegion(vcaMrHandle->VpHandle,
										ModifyReqType,
										VirtualAddress,
										Length,
										vpPdHandle,
										AccessControl,
										&vcaMrHandle->VpHandle,
										Lkey,
										Rkey);

	if ( status == FSUCCESS )
	{
		*NewMrHandle = MrHandle;
	} else {
		// On failure the existing MR is destroyed by the VP, so do the same
		// here.
		MemoryClear(vcaMrHandle, sizeof(VCA_MR_CONTEXT));
		MemoryDeallocate(vcaMrHandle);
		*NewMrHandle = (IB_HANDLE)NULL;
	}

	return status;
}

// depricated
FSTATUS
ModifyPhysMemRegion(
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
	)
{
	IB_VIRT_ADDR reqVa = (IB_VIRT_ADDR)(uintn)ReqIoVirtualAddress;
	IB_VIRT_ADDR va;
	FSTATUS	status;

	status = iba_modify_pmr(MrHandle, ModifyReqType, reqVa,
						PhysBufferList, NumPhysBuffers, IoVaOffset, PdHandle,
						AccessControl, NewMrHandle, &va, Lkey,
						Rkey);
	if (status == FSUCCESS)
	{
		*IoVirtualAddress = (void*)(uintn)va;
		if ((IB_VIRT_ADDR)(uintn)*IoVirtualAddress != va)
		{
			// we cannot fit the resulting va in a void*
			//
			// On failure the existing MR is destroyed by the VP, so do the same
			// here.
			(void)iba_deregister_mr(*NewMrHandle);
			status = FERROR;
		}
	}
	return status;
}

FSTATUS
iba_modify_pmr(
	IN  IB_HANDLE			MrHandle,
	IN  IB_MR_MODIFY_REQ	ModifyReqType,
	IN	IB_VIRT_ADDR		ReqIoVirtualAddress OPTIONAL,
	IN  void				*PhysBufferList OPTIONAL,
	IN  uint64				NumPhysBuffers OPTIONAL,
	IN  uint32				IoVaOffset OPTIONAL,
	IN  IB_HANDLE			PdHandle OPTIONAL,
	IN  IB_ACCESS_CONTROL	AccessControl OPTIONAL,
	OUT IB_HANDLE			*NewMrHandle,
	OUT IB_VIRT_ADDR		*IoVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	)
{
	VCA_MR_CONTEXT	*vcaMrHandle = (VCA_MR_CONTEXT *)MrHandle;
	VCA_PD_CONTEXT	*vcaPdHandle;
	IB_HANDLE		vpPdHandle = NULL;
	FSTATUS	status;

	if ( ModifyReqType.s.ProtectionDomain )
	{
		vcaPdHandle = (VCA_PD_CONTEXT *)PdHandle;
		vpPdHandle = vcaPdHandle->VpHandle;

		// Replace the current PdHandle in the MR context with the new
		// PdHandle. Don't need to worry that the call may fail in the
		// VP as if it does the MR is destroyed below.
		vcaMrHandle->VcaPdHandle = vcaPdHandle;
	}

	status = vcaMrHandle->VpInterface->ModifyPhysMemRegion(vcaMrHandle->VpHandle,
										ModifyReqType,
										ReqIoVirtualAddress,
										PhysBufferList,
										NumPhysBuffers,
										IoVaOffset,
										vpPdHandle,
										AccessControl,
										&vcaMrHandle->VpHandle,
										IoVirtualAddress,
										Lkey,
										Rkey);

	if ( status == FSUCCESS )
	{
		*NewMrHandle = MrHandle;
	} else {
		// On failure the existing MR is destroyed by the VP, so do the same
		// here.
		MemoryClear(vcaMrHandle, sizeof(VCA_MR_CONTEXT));
		MemoryDeallocate(vcaMrHandle);
		*NewMrHandle = (IB_HANDLE)NULL;
	}

	return status;
}


FSTATUS
iba_modify_contig_pmr(
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
	)
{
	VCA_MR_CONTEXT	*vcaMrHandle = (VCA_MR_CONTEXT *)MrHandle;
	VCA_PD_CONTEXT	*vcaPdHandle;
	IB_HANDLE		vpPdHandle = NULL;
	FSTATUS	status;

	if ( ModifyReqType.s.ProtectionDomain )
	{
		vcaPdHandle = (VCA_PD_CONTEXT *)PdHandle;
		vpPdHandle = vcaPdHandle->VpHandle;

		// Replace the current PdHandle in the MR context with the new
		// PdHandle. Don't need to worry that the call may fail in the
		// VP as if it does the MR is destroyed below.
		vcaMrHandle->VcaPdHandle = vcaPdHandle;
	}

	status = vcaMrHandle->VpInterface->ModifyContigPhysMemRegion(vcaMrHandle->VpHandle,
										ModifyReqType,
										ReqIoVirtualAddress,
										PhysBufferList,
										NumPhysBuffers,
										IoVaOffset,
										vpPdHandle,
										AccessControl,
										&vcaMrHandle->VpHandle,
										IoVirtualAddress,
										Lkey,
										Rkey);

	if ( status == FSUCCESS )
	{
		*NewMrHandle = MrHandle;
	} else {
		// On failure the existing MR is destroyed by the VP, so do the same
		// here.
		MemoryClear(vcaMrHandle, sizeof(VCA_MR_CONTEXT));
		MemoryDeallocate(vcaMrHandle);
		*NewMrHandle = (IB_HANDLE)NULL;
	}

	return status;
}


// depricated
FSTATUS
RegisterSharedMemRegion(
	IN  IB_HANDLE			MrHandle,
	IN  void				*VirtualAddress,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ACCESS_CONTROL	AccessControl,
	OUT IB_HANDLE			*SharedMrHandle,
	OUT void				**SharedVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	)
{
	IB_VIRT_ADDR reqVa = (IB_VIRT_ADDR)(uintn)VirtualAddress;
	IB_VIRT_ADDR va;
	FSTATUS	status;

	status = iba_register_smr(MrHandle, reqVa, PdHandle, AccessControl,
						SharedMrHandle, &va, Lkey, Rkey);
	if (status == FSUCCESS)
	{
		*SharedVirtualAddress = (void*)(uintn)va;
		if ((IB_VIRT_ADDR)(uintn)*SharedVirtualAddress != va)
		{
			// we cannot fit the resulting va in a void*
			//
			// On failure the existing MR is destroyed by the VP, so do the same
			// here.
			(void)iba_deregister_mr(*SharedMrHandle);
			status = FERROR;
		}
	}
	return status;
}

FSTATUS
iba_register_smr(
	IN  IB_HANDLE			MrHandle,
	IN  IB_VIRT_ADDR		VirtualAddress,
	IN  IB_HANDLE			PdHandle,
	IN  IB_ACCESS_CONTROL	AccessControl,
	OUT IB_HANDLE			*SharedMrHandle,
	OUT IB_VIRT_ADDR		*SharedVirtualAddress,
	OUT IB_L_KEY			*Lkey,
	OUT IB_R_KEY			*Rkey
	)
{
	VCA_MR_CONTEXT	*vcaMrHandle = (VCA_MR_CONTEXT *)MrHandle;
	VCA_PD_CONTEXT	*vcaPdHandle = (VCA_PD_CONTEXT *)PdHandle;
	VCA_MR_CONTEXT	*vcaSharedMrContext;
	FSTATUS	status;

	// PD and MR must be on same HCA, we can at least make sure its
	// same verbs driver here, verbs driver will need to confirm its same HCA
	if (vcaMrHandle->VpInterface != vcaPdHandle->VpInterface)
		return FINVALID_PARAMETER;

	if ( (vcaSharedMrContext = (VCA_MR_CONTEXT*)MemoryAllocate2(sizeof(VCA_MR_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	MemoryClear(vcaSharedMrContext, sizeof(VCA_MR_CONTEXT));

	status = vcaMrHandle->VpInterface->RegisterSharedMemRegion(vcaMrHandle->VpHandle,
									VirtualAddress,
									vcaPdHandle->VpHandle,
									AccessControl,
									&vcaSharedMrContext->VpHandle,
									SharedVirtualAddress,
									Lkey,
									Rkey);

	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle
		vcaSharedMrContext->VpInterface = vcaMrHandle->VpInterface;
		vcaSharedMrContext->VcaPdHandle = vcaPdHandle;
		*SharedMrHandle = (IB_HANDLE)vcaSharedMrContext;
	} else {
		MemoryDeallocate(vcaSharedMrContext);
	}

	return status;
}


FSTATUS
iba_create_mw(
	IN  IB_HANDLE	CaHandle,
	IN  IB_HANDLE	PdHandle,
	OUT IB_HANDLE	*MwHandle,
	OUT IB_R_KEY	*Rkey
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;
	VCA_PD_CONTEXT	*vcaPdHandle = (VCA_PD_CONTEXT *)PdHandle;
	VCA_MW_CONTEXT	*vcaMwContext;
	FSTATUS	status;

	if ( (vcaMwContext = (VCA_MW_CONTEXT*)MemoryAllocate2(sizeof(VCA_MW_CONTEXT), IBA_MEM_FLAG_PREMPTABLE, VCA_MEM_TAG)) == NULL )
	{
		return FINSUFFICIENT_RESOURCES;
	}

	MemoryClear(vcaMwContext, sizeof(VCA_MW_CONTEXT));

	status = vcaCaHandle->VpInterface->CreateMemWindow(vcaCaHandle->VpHandle,
									vcaPdHandle->VpHandle,
									&vcaMwContext->VpHandle,
									Rkey);

	if (status == FSUCCESS)
	{
		// Save VP I/F so we can make direct calls from the handle
		vcaMwContext->VpInterface = vcaCaHandle->VpInterface;
		vcaMwContext->VcaPdHandle = vcaPdHandle;
		*MwHandle = (IB_HANDLE)vcaMwContext;
	} else {
		MemoryDeallocate(vcaMwContext);
	}

	return status;
}

FSTATUS
iba_query_mw(
	IN  IB_HANDLE	MwHandle,
	OUT IB_HANDLE	*PdHandle,
	OUT IB_R_KEY	*Rkey
	)
{
	VCA_MW_CONTEXT	*vcaMwHandle = (VCA_MW_CONTEXT *)MwHandle;
	FSTATUS	status;

	status = vcaMwHandle->VpInterface->QueryMemWindow(vcaMwHandle->VpHandle,
									PdHandle,
									Rkey);

	// Set the PD handle to the VCA's PD handle
	*PdHandle = (IB_HANDLE)vcaMwHandle->VcaPdHandle;

	return status;
}


// depricated
FSTATUS
PostMemWindowBind(
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
	)
{
	return iba_post_bind_mw(QpHandle, WorkReqId, MwHandle, CurrentRkey,
				MrHandle, MrLkey, (IB_VIRT_ADDR)(uintn)VirtualAddress,
				(uint64)Length, AccessControl, Options, NewRkey);
}

FSTATUS
iba_post_bind_mw(
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
	)
{
	VCA_MW_CONTEXT	*vcaMwHandle = (VCA_MW_CONTEXT *)MwHandle;
	VCA_MR_CONTEXT	*vcaMrHandle = (VCA_MR_CONTEXT *)MrHandle;

	return ((VCA_QP_CONTEXT*)QpHandle)->VpInterface->PostMemWindowBind(
									((VCA_QP_CONTEXT*)QpHandle)->VpHandle,
									WorkReqId,
									vcaMwHandle->VpHandle,
									CurrentRkey,
									vcaMrHandle->VpHandle,
									MrLkey,
									VirtualAddress,
									Length,
									AccessControl,
									Options,
									NewRkey);
}

FSTATUS
iba_destroy_mw(
	IN  IB_HANDLE	MwHandle
	)
{
	VCA_MW_CONTEXT	*vcaMwHandle = (VCA_MW_CONTEXT *)MwHandle;
	FSTATUS	status;

	status = vcaMwHandle->VpInterface->DestroyMemWindow(vcaMwHandle->VpHandle);

	if ( status == FSUCCESS )
	{
		// Clear context so that future references fail
		MemoryClear(vcaMwHandle, sizeof(VCA_MW_CONTEXT));
		MemoryDeallocate(vcaMwHandle);
	}

	return status;
}


FSTATUS
iba_post_send(
	IN  IB_HANDLE	QpHandle,
	IN  IB_WORK_REQ	*WorkRequest
	)
{
	IB_WORK_REQ2 WorkRequest2;

	// for speed in data path we don't validate handles
	WorkRequest2.Next = NULL;
	WorkRequest2.DSList = WorkRequest->DSList;
	WorkRequest2.WorkReqId = WorkRequest->WorkReqId;
	WorkRequest2.MessageLen = WorkRequest->MessageLen;
	WorkRequest2.DSListDepth = WorkRequest->DSListDepth;
	WorkRequest2.Operation = WorkRequest->Operation;
	switch ( ((VCA_QP_CONTEXT*)QpHandle)->Type )
	{
		case QPTypeReliableConnected:
		case QPTypeUnreliableConnected:
			switch (WorkRequest->Operation)
			{
				case WROpRdmaWrite:
				case WROpRdmaRead:
					WorkRequest2.Req.SendRC.RemoteDS = WorkRequest->Req.SendRC.RemoteDS;
				case WROpSend:
					WorkRequest2.Req.SendRC.Options = WorkRequest->Req.SendRC.Options;
					WorkRequest2.Req.SendRC.ImmediateData = WorkRequest->Req.SendRC.ImmediateData;
					break;
				case WROpFetchAdd:
				case WROpCompareSwap:
					WorkRequest2.Req.AtomicRC.Options = WorkRequest->Req.AtomicRC.Options;
					WorkRequest2.Req.AtomicRC.RemoteDS = WorkRequest->Req.AtomicRC.RemoteDS;
					WorkRequest2.Req.AtomicRC.SwapAddOperand = WorkRequest->Req.AtomicRC.SwapAddOperand;
					WorkRequest2.Req.AtomicRC.CompareOperand = WorkRequest->Req.AtomicRC.CompareOperand;
					break;
				default:
					break;
			}
			break;
		case QPTypeReliableDatagram:
			return FINVALID_OPERATION;	/* not supported */
			break;
		case QPTypeUnreliableDatagram:
		case QPTypeSMI:
		case QPTypeGSI:
			WorkRequest2.Req.SendUD.Options = WorkRequest->Req.SendUD.Options;
			WorkRequest2.Req.SendUD.PkeyIndex = WorkRequest->Req.SendUD.PkeyIndex;
			WorkRequest2.Req.SendUD.ImmediateData = WorkRequest->Req.SendUD.ImmediateData;
			WorkRequest2.Req.SendUD.QPNumber = WorkRequest->Req.SendUD.QPNumber;
			WorkRequest2.Req.SendUD.Qkey = WorkRequest->Req.SendUD.Qkey;
			WorkRequest2.Req.SendUD.AVHandle = WorkRequest->Req.SendUD.AVHandle;
			break;
#if defined(INCLUDE_STLEEP)
		case QPTypeSTLEEP:
			/* This should never happen as there are no send options defined for an IB_WORK_REQ */
			/* Must use IB_WORK_REQ2 and ib_post_send2 */
			sysPrintf("%s: Sends for QPTypeSTLEEP must be issued through ib_post_send2.\n", __func__);
			return FERROR;
			break;
#endif
		case QPTypeRawDatagram:
		case QPTypeRawIPv6:
		case QPTypeRawEthertype:
			WorkRequest2.Req.SendRawD.QPNumber = WorkRequest->Req.SendRawD.QPNumber;
			WorkRequest2.Req.SendRawD.DestLID = WorkRequest->Req.SendRawD.DestLID;
			WorkRequest2.Req.SendRawD.PathBits = WorkRequest->Req.SendRawD.PathBits;
			WorkRequest2.Req.SendRawD.ServiceLevel = WorkRequest->Req.SendRawD.ServiceLevel;
			WorkRequest2.Req.SendRawD.SignaledCompletion = WorkRequest->Req.SendRawD.SignaledCompletion;
			WorkRequest2.Req.SendRawD.SolicitedEvent = WorkRequest->Req.SendRawD.SolicitedEvent;
			WorkRequest2.Req.SendRawD.StaticRate = WorkRequest->Req.SendRawD.StaticRate;
			WorkRequest2.Req.SendRawD.EtherType = WorkRequest->Req.SendRawD.EtherType;
			break;
	}
	return ((VCA_QP_CONTEXT*)QpHandle)->VpInterface->PostSend(
					((VCA_QP_CONTEXT*)QpHandle)->VpHandle, &WorkRequest2, NULL);
}

FSTATUS
iba_post_recv(
	IN  IB_HANDLE	QpHandle,
	IN  IB_WORK_REQ	*WorkRequest
	)
{
	IB_WORK_REQ2_RECV WorkRequest2;

	// for speed in data path we don't validate handles
	WorkRequest2.Next = NULL;
	WorkRequest2.DSList = WorkRequest->DSList;
	WorkRequest2.WorkReqId = WorkRequest->WorkReqId;
	WorkRequest2.MessageLen = WorkRequest->MessageLen;
	WorkRequest2.DSListDepth = WorkRequest->DSListDepth;
	WorkRequest2.Operation = WorkRequest->Operation;
	/* no Req fields for Recv */
	return ((VCA_QP_CONTEXT*)QpHandle)->VpInterface->PostRecv(
										((VCA_QP_CONTEXT*)QpHandle)->VpHandle,
										(IB_WORK_REQ2*)&WorkRequest2, NULL);
}

FSTATUS
iba_post_send2(
	IN  IB_HANDLE	QpHandle,
	IN  IB_WORK_REQ2*WorkRequestList,
	OUT IB_WORK_REQ2	**FailedWorkRequest OPTIONAL
	)
{
	CpuPrefetch(WorkRequestList);
	return ((VCA_QP_CONTEXT*)QpHandle)->VpInterface->PostSend(
							((VCA_QP_CONTEXT*)QpHandle)->VpHandle,
							WorkRequestList, FailedWorkRequest);
}

/* fetch a VPD AV Handle given a VCA AV handle in a IB_WORK_REQ2 */
IB_HANDLE
GetVpdAvHandle(IN IB_HANDLE IbaAvHandle)
{
	return ((VCA_AV_CONTEXT	*)IbaAvHandle)->VpHandle;
}

FSTATUS
iba_post_recv2(
	IN  IB_HANDLE	QpHandle,
	IN  IB_WORK_REQ2*WorkRequestList,
	OUT IB_WORK_REQ2	**FailedWorkRequest OPTIONAL
	)
{
	CpuPrefetch(WorkRequestList);
	return ((VCA_QP_CONTEXT*)QpHandle)->VpInterface->PostRecv(
										((VCA_QP_CONTEXT*)QpHandle)->VpHandle,
										WorkRequestList, FailedWorkRequest);
}

FSTATUS
iba_poll_cq(
	IN  IB_HANDLE			CqHandle,
	OUT IB_WORK_COMPLETION	*WorkCompletion
	)
{
	CpuPrefetch(WorkCompletion);
	return ((VCA_CQ_CONTEXT*)CqHandle)->VpInterface->PollCQ(
										((VCA_CQ_CONTEXT*)CqHandle)->VpHandle,
										WorkCompletion);
}

FSTATUS
iba_peek_cq(
	IN  IB_HANDLE			CqHandle,
	IN	uint32				NumWC
	)
{
	return ((VCA_CQ_CONTEXT*)CqHandle)->VpInterface->PeekCQ(
								((VCA_CQ_CONTEXT*)CqHandle)->VpHandle, NumWC);
}

FSTATUS
iba_poll_and_rearm_cq(
	IN  IB_HANDLE			CqHandle,
	IN	IB_CQ_EVENT_SELECT	EventSelect,
	OUT IB_WORK_COMPLETION	*WorkCompletion
	)
{
	return ((VCA_CQ_CONTEXT*)CqHandle)->VpInterface->PollAndRearmCQ(
										((VCA_CQ_CONTEXT*)CqHandle)->VpHandle,
										EventSelect, WorkCompletion);
}

FSTATUS
iba_rearm_cq(
	IN  IB_HANDLE			CqHandle,
	IN	IB_CQ_EVENT_SELECT	EventSelect
	)
{
	return ((VCA_CQ_CONTEXT*)CqHandle)->VpInterface->RearmCQ(
										((VCA_CQ_CONTEXT*)CqHandle)->VpHandle,
										EventSelect);
}


FSTATUS
iba_rearm_n_cq(
	IN  IB_HANDLE			CqHandle,
	IN	uint16				NumWC
	)
{
	return ((VCA_CQ_CONTEXT*)CqHandle)->VpInterface->RearmNCQ(
								((VCA_CQ_CONTEXT*)CqHandle)->VpHandle, NumWC);
}


FSTATUS
default_poll_and_rearm_cq(
	IN  IB_HANDLE			CqHandle,
	IN	IB_CQ_EVENT_SELECT	EventSelect,
	OUT IB_WORK_COMPLETION	*WorkCompletion
	)
{
	// simplistic implementation, not optimized
	// a more optimized version would be:
	//		validate all handles and arguments - error returns as needed
	//		get entry from CQ (if any)
	//		if CQ has more entries
	//			return FSUCCESS
	//		RearmCQ
	//		if CQ is now empty (must check again after Rearm)
	//			if got a CQE above
	//				return FCOMPLETED
	//			else
	//				return FNOT_DONE
	//		else
	//			return FPOLL_NEEDED

	FSTATUS		status;

	status = iba_poll_cq(CqHandle, WorkCompletion);
	if (FNOT_DONE == status)
	{
		// CQ now empty
		status = iba_rearm_cq(CqHandle, EventSelect);
		if (FSUCCESS == status)
		{
			status = iba_poll_cq(CqHandle, WorkCompletion);
			if (FSUCCESS == status)
				status = FPOLL_NEEDED;
		}
	}

	return status;
}

//
// Private interfaces for the SMA and CM
//

FSTATUS
iba_get_set_mad(
	IN IB_HANDLE	QpHandle,
	IN uint8		PortNumber,
	IN STL_LID	SLID,
	IN STL_SMP		*SmpInOut,
	IN uint32		*SmpLength
	)
{
	FSTATUS rc;

	BSWAP_STL_SMP_HEADER(SmpInOut);
	rc = ((VCA_QP_CONTEXT*)QpHandle)->VpInterface->GetSetMad(
										((VCA_QP_CONTEXT*)QpHandle)->VpHandle,
										PortNumber,
										SLID,
		  								(void*)SmpInOut,
		  								SmpLength);
	BSWAP_STL_SMP_HEADER(SmpInOut);

	return rc;
}


FSTATUS
iba_register_cm(
	IN IB_HANDLE				CaHandle
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;

	return vcaCaHandle->VpInterface->RegisterConnMgr(vcaCaHandle->VpHandle);
}


//
// Private interfaces to the user-mode VP component
//


FSTATUS
iba_query_ca_private_info(
	IN  IB_HANDLE	CaHandle,
	IN OUT uint32	*BufferSize,
	OUT void		*Buffer
	)
{
	VCA_CA_CONTEXT	*vcaCaHandle = (VCA_CA_CONTEXT *)CaHandle;

	if (! vcaCaHandle->VpInterface->QueryCAPrivateInfo)
		return FINVALID_OPERATION;
	return vcaCaHandle->VpInterface->QueryCAPrivateInfo(vcaCaHandle->VpHandle,
									BufferSize,
									Buffer );
}


FSTATUS
iba_query_pd_private_info(
	IN  IB_HANDLE	PdHandle,
	IN OUT uint32	*BufferSize,
	OUT void		*Buffer
	)
{
	if (! ((VCA_PD_CONTEXT*)PdHandle)->VpInterface->QueryPDPrivateInfo)
		return FINVALID_OPERATION;
	return ((VCA_PD_CONTEXT*)PdHandle)->VpInterface->QueryPDPrivateInfo(
					((VCA_PD_CONTEXT*)PdHandle)->VpHandle,
					BufferSize, Buffer );
}

FSTATUS
iba_query_qp_private_info(
	IN  IB_HANDLE	QpHandle,
	IN OUT uint32	*BufferSize,
	OUT void		*Buffer
	)
{
	if (! ((VCA_QP_CONTEXT*)QpHandle)->VpInterface->QueryQPPrivateInfo)
		return FINVALID_OPERATION;
	return ((VCA_QP_CONTEXT*)QpHandle)->VpInterface->QueryQPPrivateInfo(
					((VCA_QP_CONTEXT*)QpHandle)->VpHandle,
					BufferSize, Buffer );
}


FSTATUS
iba_query_cq_private_info(
	IN  IB_HANDLE	CqHandle,
	IN OUT uint32	*BufferSize,
	OUT	void		*Buffer
	)
{
	if ( ! ((VCA_CQ_CONTEXT*)CqHandle)->VpInterface->QueryCQPrivateInfo)
		return FINVALID_OPERATION;
	return ((VCA_CQ_CONTEXT*)CqHandle)->VpInterface->QueryCQPrivateInfo(
							((VCA_CQ_CONTEXT*)CqHandle)->VpHandle,
							BufferSize, Buffer );
}


FSTATUS
iba_query_av_private_info(
	IN  IB_HANDLE	AvHandle,
	IN OUT uint32	*BufferSize,
	OUT void		*Buffer
	)
{
	VCA_AV_CONTEXT	*vcaAvHandle = (VCA_AV_CONTEXT *)AvHandle;

	if (! vcaAvHandle->VpInterface->QueryAVPrivateInfo)
		return FINVALID_OPERATION;
	return vcaAvHandle->VpInterface->QueryAVPrivateInfo(vcaAvHandle->VpHandle,
						BufferSize,
						Buffer );
}
