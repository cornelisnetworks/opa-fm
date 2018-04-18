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

#include <datatypes.h>
#include <imemory.h>
#include <ilist.h>
#include <ievent.h>
#include <imutex.h>
#include <stl_sd.h>
#include <sdi.h>
#include <ib_generalServices.h>


#if 0 // keep this around in case we need it for debugging again
void SaShowPacket(char *msg, unsigned char *packet, int length) {
	_DBG_DUMP_MEMORY(_DBG_LVL_? ("%s", msg), (void*)packet, length);
}
#endif

FSTATUS
SubnetAdmInit(
    void
    )
{
	uint64 *pLocalPortGuidsList = NULL;
	uint32 LocalPortGuidsCount;
	FSTATUS Fstatus;
	unsigned ii;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SubnetAdmInit);

	Fstatus = iba_sd_get_local_port_guids_alloc(&pLocalPortGuidsList, &LocalPortGuidsCount);
	if (Fstatus != FSUCCESS || LocalPortGuidsCount == 0)
	{
		if (Fstatus == FSUCCESS)
			Fstatus = FNOT_DONE;
		if (pLocalPortGuidsList != NULL)
		{
			MemoryDeallocate(pLocalPortGuidsList);
			pLocalPortGuidsList = NULL;
		}
		_DBG_INFO(("No Active LocalPortGuids found\n"));
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
        return(Fstatus);
	}

	for (ii = 0; ii < LocalPortGuidsCount; ii++)
	{
		MAD *mad;
		PQueryDetails pQueryElement;

		pQueryElement = (QueryDetails*)MemoryAllocateAndClear(sizeof(QueryDetails),
									FALSE, SUBNET_DRIVER_TAG);
	
		if (pQueryElement == NULL)  
		{
			_DBG_ERROR(("Cannot allocate memory for QueryElement in SubnetAdmInit\n"));
			_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
        	return(FINSUFFICIENT_MEMORY);
    	}

		QListSetObj(&pQueryElement->QueryListItem, (void *)pQueryElement);
		pQueryElement->IsSelfCommand = TRUE;
		pQueryElement->SdSentCmd = GetClassportInfo;
		pQueryElement->ControlParameters.RetryCount = g_SdParams.DefaultRetryCount;
		pQueryElement->ControlParameters.Timeout = g_SdParams.DefaultRetryInterval;
	
    	pQueryElement->u.pSaMad = (SA_MAD*)MemoryAllocateAndClear(sizeof(SA_MAD),
                                                FALSE,
                                                SUBNET_DRIVER_TAG);
    	if (pQueryElement->u.pSaMad == NULL)
    	{
			MemoryDeallocate(pQueryElement);
        	_DBG_ERROR(("Cannot allocate memory for QueryElement->u.pSaMad in SubnetAdmInit\n"));
        	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
        	return(FINSUFFICIENT_MEMORY);
    	} 

		mad = (MAD *)pQueryElement->u.pSaMad;
	
		FillBaseGmpValues(pQueryElement);

		mad->common.mr.s.R = 0; //since this is not a response, set it to 0;
		mad->common.mr.s.Method = MMTHD_GET; 

		MAD_SET_ATTRIB_ID(mad, MCLASS_ATTRIB_ID_CLASS_PORT_INFO);
		MAD_SET_ATTRIB_MOD(mad, 0); 

		// fields were initialized to 0 above
		// which is what we want for a single packet RMPP request in 1.0a or 1.1
	pQueryElement->TotalBytesInGmp = (mad->common.BaseVersion == IB_BASE_VERSION)
		? IB_MAD_BLOCK_SIZE
		: (sizeof(SA_MAD) - STL_SUBN_ADM_DATASIZE) + sizeof(STL_CLASS_PORT_INFO);

		pQueryElement->PortGuid = pLocalPortGuidsList[ii];

#if defined( DBG) || defined(IB_DEBUG)
		{
			CA_PORT *pCaPort;

			SpinLockAcquire(pCaPortListLock);
			pCaPort = GetCaPort(pLocalPortGuidsList[ii]);
			ASSERT(pCaPort != NULL);
			ASSERT(pQueryElement->PortGuid == pCaPort->PortGuid);
			SpinLockRelease(pCaPortListLock);
		}
#endif

		SendQueryElement(pQueryElement, TRUE /*FirstAttempt*/);
	}

	MemoryDeallocate(pLocalPortGuidsList);

	// Trigger the timer handler
	StartTimerHandler(pTimerObject, g_SdParams.TimeInterval);

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return(FSUCCESS);
}

// for a given SA attribute, indicate the size of the actual
// unpadded record returned
uint32
SubnetAdmAttrToRecordSize(uint8 attribute)
{
	switch (attribute)
	{
	case SA_ATTRIB_INFORM_INFO:
			return(sizeof(IB_INFORM_INFO));
	case SA_ATTRIB_NODE_RECORD:
			return(sizeof(IB_NODE_RECORD));
	case SA_ATTRIB_PORTINFO_RECORD:
			return(sizeof(IB_PORTINFO_RECORD));
	case SA_ATTRIB_SWITCHINFO_RECORD:
			return(sizeof(IB_SWITCHINFO_RECORD));
	case SA_ATTRIB_LINEAR_FWDTBL_RECORD:
			return(sizeof(IB_LINEAR_FDB_RECORD));
	case SA_ATTRIB_RANDOM_FWDTBL_RECORD:
			return(sizeof(IB_RANDOM_FDB_RECORD));
	case SA_ATTRIB_MCAST_FWDTBL_RECORD:
			return(sizeof(IB_MCAST_FDB_RECORD));
	case SA_ATTRIB_SMINFO_RECORD:
			return(sizeof(IB_SMINFO_RECORD));
	case SA_ATTRIB_INFORM_INFO_RECORD:
			return(sizeof(IB_INFORM_INFO_RECORD));
	case SA_ATTRIB_LINK_RECORD:
			return(sizeof(IB_LINK_RECORD));
	case SA_ATTRIB_SERVICE_RECORD:
			return(sizeof(IB_SERVICE_RECORD));
	case SA_ATTRIB_P_KEY_TABLE_RECORD:
			return(sizeof(IB_P_KEY_TABLE_RECORD));
	case SA_ATTRIB_PATH_RECORD:
			return(sizeof(IB_PATH_RECORD));
	case SA_ATTRIB_VLARBTABLE_RECORD:
			return(sizeof(IB_VLARBTABLE_RECORD));
	case SA_ATTRIB_MCMEMBER_RECORD:
			return(sizeof(IB_MCMEMBER_RECORD));
	case SA_ATTRIB_TRACE_RECORD:
			return(sizeof(IB_TRACE_RECORD));
	case SA_ATTRIB_MULTIPATH_RECORD:
			return(sizeof(IB_MULTIPATH_RECORD));
	case SA_ATTRIB_SERVICEASSOCIATION_RECORD:
			return(sizeof(IB_SERVICEASSOCIATION_RECORD));
	default:
			return 0;
	}
}

// look at payload len in SA reponse and compute the size of
// each record and the number of records
// also validates payload does not indicate truncation of last record
// if last was truncated, it reduces NumRecords with a warning
FSTATUS
SdComputeResponseSizes(
	SA_MAD 				*pSaMad,
	uint32				messageSize,	// Size of Coalesced message
	uint32				*pNumRecords,
	uint32				*pPaddedRecordSize,
	uint32				*pRecordSize
	)
{
	uint32				saPayload;
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SdComputeResponseSizes);

	*pRecordSize = SubnetAdmAttrToRecordSize(pSaMad->common.AttributeID);
	if (pSaMad->common.mr.AsReg8 == SUBN_ADM_GET_RESP)
	{
		// single record response
		*pPaddedRecordSize = *pRecordSize;
		*pNumRecords = 1;
		return FSUCCESS;
	}
	// round up just in case SA omits pad bytes at end of last record
	*pPaddedRecordSize = pSaMad->SaHdr.AttributeOffset*sizeof(uint64) ;
	if (*pPaddedRecordSize == 0) {
		_DBG_WARN(("Invalid AttributeOffset!\n"
					"\tAttributeOffset........%d\n"
					"\tPayloadLen.............%d\n"
					"\tRecordSize.............%d\n",
					pSaMad->SaHdr.AttributeOffset,
					pSaMad->RmppHdr.u2.PayloadLen,
					*pRecordSize));
		*pNumRecords = 0;	// TBD ignore or fail request????
		_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
		return FSUCCESS;
	}
	// GsiCoalesceDgrm already removed extra SA_HDRs and adjusted
	// messageSize to account for their removal
	saPayload = messageSize - (sizeof(MAD_COMMON) + sizeof(RMPP_HEADER) + sizeof(SA_HDR));
	*pNumRecords = (saPayload + (*pPaddedRecordSize)-1) / (*pPaddedRecordSize);

	// see if last Record in list is complete
	if (*pNumRecords 
		&& (*pNumRecords-1)*(*pPaddedRecordSize)+(*pRecordSize) > saPayload)
	{
		// last record was truncated
		_DBG_WARN(("Final Record of SA Response is truncated!\n"
					"\tAttributeOffset........%d\n"
					"\tPayloadLen.............%d\n"
					"\tRecordSize.............%d\n",
					pSaMad->SaHdr.AttributeOffset,
					pSaMad->RmppHdr.u2.PayloadLen,
					*pRecordSize));
		--(*pNumRecords);	// TBD ignore or fail request????
	}
	_DBG_INFO(("SA Response:\n"
				"\tAttributeOffset........%d (%d)\n"
				"\tPayloadLen.............%d\n"
				"\tRecordSize.............%d\n"
				"\tNumRecords.............%d\n",
				pSaMad->SaHdr.AttributeOffset,
				*pPaddedRecordSize,
				pSaMad->RmppHdr.u2.PayloadLen,
				*pRecordSize, *pNumRecords));

	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return FSUCCESS;
}

// called with pQueryList->m_Lock held
// called with buffer==NULL and messageSize == 0 if query has timed out
// pQueryElement is in WaitingForResult, still on list
FSTATUS 
SubnetAdmRecv(
	PQueryDetails pQueryElement,
	void *buffer,			// caller removed GRH already
	uint32 messageSize		// caller removed GRH already
	,IBT_DGRM_ELEMENT		*pDgrmList
    )
{
	QUERY_RESULT_VALUES * pClientResult = NULL;
	SA_MAD *pSaMad = (SA_MAD*)buffer;
	uint16	MadAttributeID;
	QUERY_RESULT_TYPE	QueryOutputType;
	uint32				NumRecords;
	uint32				RecordSize;	// before padding
	uint32				PaddedRecordSize;	// after padding
	uint8				*p;	// start of next record

	uint32	i;
    
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SubnetAdmRecv);

	if(pQueryElement->pParentQuery != NULL)
	{
		return SdHandleChildQuery(pQueryElement, buffer, messageSize);
	}
	// Handle the case if buffer == NULL;

	if (buffer == NULL) //Timeout 
	{
		_DBG_ERROR(("SA Query Timed Out <%d ms, %d retries>\n",
			pQueryElement->ControlParameters.Timeout,
			pQueryElement->ControlParameters.RetryCount));

		DEBUG_ASSERT(pQueryElement->SdSentCmd != UserQueryFabricInformation
					|| pQueryElement->cmd.query.pClientResult == NULL);
		pQueryElement->Status = FTIMEOUT;
		pQueryElement->MadStatus = 0;	// unknown, no response
		goto done;
	}

	// Include MAD Status and reserved field
	pQueryElement->MadStatus = (pSaMad->common.u.NS.Reserved1 << 16|
									pSaMad->common.u.NS.Status.AsReg16);

	if (pSaMad->common.u.NS.Status.AsReg16 == MAD_STATUS_SUCCESS) 
	{
		_DBG_INFO(("Successful MAD completion received\n"));
		pQueryElement->Status = FSUCCESS; 
	} else if (pQueryElement->SdSentCmd == UserQueryFabricInformation
			&& pSaMad->common.u.NS.Status.AsReg16 == MAD_STATUS_SA_NO_RECORDS)
	{
		_DBG_INFO(("No Records MAD completion received\n"));
		pQueryElement->Status = FSUCCESS; 
	} else {
		_DBG_ERROR(("Failed MAD completion received, Status <0x%x: %s>, Port 0x%016"PRIx64"\n",
			pSaMad->common.u.NS.Status.AsReg16,
			_DBG_PTR(iba_sd_mad_status_msg(pQueryElement->MadStatus)),
			pQueryElement->PortGuid));
		pQueryElement->Status = FERROR; 
		DEBUG_ASSERT(pQueryElement->SdSentCmd != UserQueryFabricInformation
					|| pQueryElement->cmd.query.pClientResult == NULL);
		goto done;
	}

	//
	// Based on the attribute ID figure out how to process the response
	//
	MadAttributeID = pSaMad->common.AttributeID;
	BSWAP_SA_HDR(&pSaMad->SaHdr);
	if (pQueryElement->SdSentCmd == UserFabricOperation)
	{
		// Fabric Operation Response
		switch (MadAttributeID)
		{
			case SA_ATTRIB_SERVICE_RECORD:
				{
            	IB_SERVICE_RECORD          	*pServiceRecord;
 	
            	pServiceRecord = (IB_SERVICE_RECORD*)pSaMad->Data;
                BSWAP_IB_SERVICE_RECORD(pServiceRecord);
            	_DBG_INFO(("Processing SERVICE_RECORD..... NumRecords[1]\n"));
				pQueryElement->cmd.op.FabOp.Value.ServiceRecordValue.ServiceRecord =
							*pServiceRecord;
				}
				break;
			case SA_ATTRIB_MCMEMBER_RECORD:
				{
            	IB_MCMEMBER_RECORD          	*pMcMemberRecord;
 	
            	pMcMemberRecord = (IB_MCMEMBER_RECORD*)pSaMad->Data;
                BSWAP_IB_MCMEMBER_RECORD(pMcMemberRecord);
				if (g_SdParams.MinPktLifeTime) {
					uint32 PktLifeTime = TimeoutTimeMsToMult(g_SdParams.MinPktLifeTime);
					if (PktLifeTime > pMcMemberRecord->PktLifeTime)
						pMcMemberRecord->PktLifeTime = PktLifeTime;
				}
            	_DBG_INFO(("Processing MCMEMBER_RECORD..... NumRecords[1]\n"));
				pQueryElement->cmd.op.FabOp.Value.McMemberRecordValue.McMemberRecord =
							*pMcMemberRecord;
				}
				break;
			case SA_ATTRIB_INFORM_INFO:
				{
            	IB_INFORM_INFO          	*pInformInfo;
 	
            	pInformInfo = (IB_INFORM_INFO*)pSaMad->Data;
                BSWAP_INFORM_INFO(pInformInfo);
            	_DBG_INFO(("Processing IB_INFORM_INFO..... NumRecords[1]\n"));
				pQueryElement->cmd.op.FabOp.Value.InformInfo =
							*pInformInfo;
				}
			default:
				break;
		}
		goto done;
	}

	if (pQueryElement->SdSentCmd != UserQueryFabricInformation)
	{
		DEBUG_ASSERT(0);
		goto done;
	}

	// Fabric Query
	QueryOutputType = pQueryElement->cmd.query.UserQuery.OutputType;
	pQueryElement->QState = ProcessingResponse; // holds our reference
	// unlock so we can prempt during memory allocates below
	SpinLockRelease(&pQueryList->m_Lock);

	if (pSaMad->common.u.NS.Status.AsReg16 == MAD_STATUS_SA_NO_RECORDS) {
		NumRecords = 0;
	} else {
		(void)SdComputeResponseSizes( pSaMad, messageSize, &NumRecords,
								&PaddedRecordSize, &RecordSize);
	}

	switch (MadAttributeID)
	{
		case SA_ATTRIB_NODE_RECORD:
		{
			if (! g_SdParams.ValidateNodeRecord)
			{
				for (i = 0, p = pSaMad->Data;
						i < NumRecords; i++, p+=PaddedRecordSize)
            	{
					BSWAP_IB_NODE_RECORD((IB_NODE_RECORD*)p);
            	}
			} else {
				int dodump = 0;

				/* while we byte swap, also validate
				 * we are checking for 0 Node or Port GUIDs.  This generally
				 * indicates an RMPP or SA problem where records get skewed
				 */
				for (i = 0, p = pSaMad->Data;
						i < NumRecords; i++, p+=PaddedRecordSize)
            	{
//#define VALID_GUID(guid) ((((guid) >> 40) == 0x66a) || (((guid) >> 40) == 0x2c9))
#define VALID_GUID(guid) ((guid) != 0)
					BSWAP_IB_NODE_RECORD((IB_NODE_RECORD*)p);
					if ( ! VALID_GUID(((IB_NODE_RECORD*)p)->NodeInfoData.NodeGUID)
						|| ! VALID_GUID(((IB_NODE_RECORD*)p)->NodeInfoData.PortGUID))
#undef VALID_GUID
					{
						dodump=1;
#ifdef ICS_LOGGING
						_DBG_ERROR(("Corrupted Node Record %d of %d offset 0x%x", i+1, NumRecords, (unsigned)(p - pSaMad->Data)));
						_DBG_DUMP_MEMORY(_DBG_LVL_ERROR, ("Corrupted Node Record"), (void*)p, sizeof(IB_NODE_RECORD));
#else
						_DBG_DUMP_MEMORY(_DBG_LVL_ERROR,
							("Corrupted Node Record %d of %d offset 0x%x", i+1, NumRecords, (unsigned)(p - pSaMad->Data)),
							(void*)p, sizeof(IB_NODE_RECORD));
#endif
					}
				}
				if (dodump && g_SdParams.DumpInvalidNodeRecord)
				{
					IBT_ELEMENT *pElement;

					_DBG_DUMP_MEMORY(_DBG_LVL_ERROR, ("Coalesed"), (void*)buffer, messageSize);

					for (i=0,pElement = &pDgrmList->Element; pElement; i++,pElement=pElement->pNextElement)
					{
						IBT_BUFFER *pBufferHdr = pElement->pBufferList->pNextBuffer;
#ifdef ICS_LOGGING
						_DBG_ERROR(("Dgrm %d", i));
						_DBG_DUMP_MEMORY(_DBG_LVL_ERROR, ("Dgrm"), (void*)pBufferHdr->pData, pBufferHdr->ByteCount);
#else
						_DBG_DUMP_MEMORY(_DBG_LVL_ERROR, ("Dgrm %d", i), (void*)pBufferHdr->pData, pBufferHdr->ByteCount);
#endif
					}
				}
			}
			_DBG_INFO(("Processing IB_NODE_RECORD..... NumRecords[%d]\n",
										NumRecords));
			switch(QueryOutputType)
			{
            	case (OutputTypeLid):
				{
    				PLID_RESULTS 			pLidResults;

					_DBG_INFO(("Extracting LIDs from NodeRecord....\n"));

               		pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
										+ (NumRecords * sizeof(LID_RESULTS)),
                    					IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
               		if (pClientResult == NULL)
						goto failalloc;
               		pClientResult->ResultDataSize = NumRecords * sizeof(LID_RESULTS);
					
					if (NumRecords > 0 )
					{
               			pLidResults = (PLID_RESULTS)pClientResult->QueryResult;
               			pLidResults->NumLids = NumRecords;
						for (i = 0, p = pSaMad->Data;
							i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
               	    		pLidResults->Lids[i] = 
									((IB_NODE_RECORD*)p)->RID.s.LID;
                    	}
					}
					break;
				}

               	case (OutputTypeSystemImageGuid):
				case (OutputTypeNodeGuid):
                case (OutputTypePortGuid):
				{
   					PGUID_RESULTS 			pGuidResults;

                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
										+ (NumRecords * sizeof(GUID_RESULTS)),
										IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(GUID_RESULTS);
					switch (QueryOutputType)
					{
                    	case (OutputTypeSystemImageGuid):

							_DBG_INFO(("Extracting SystemImageGUIDs from NodeRecord - NumRecords[%d]\n",
															NumRecords));
                   			if (NumRecords > 0 )
                    		{
                    			pGuidResults = (PGUID_RESULTS)pClientResult->QueryResult;
                    			pGuidResults->NumGuids = NumRecords;
								for (i = 0, p = pSaMad->Data;
									i < NumRecords; i++, p+=PaddedRecordSize)
                       			{
									pGuidResults->Guids[i] = 
										((IB_NODE_RECORD*)p)->NodeInfoData.SystemImageGUID;
                       			}
							}
							break;
						case (OutputTypeNodeGuid):

							_DBG_INFO(("Extracting NodeGUIDs from NodeRecord - NumRecords[%d]\n",
															NumRecords));
                           	if (NumRecords > 0 )
                            {
                                pGuidResults = (PGUID_RESULTS)pClientResult->QueryResult;
                                pGuidResults->NumGuids = NumRecords;
								for (i = 0, p = pSaMad->Data;
									i < NumRecords; i++, p+=PaddedRecordSize)
								{

									pGuidResults->Guids[i] = 
										((IB_NODE_RECORD*)p)->NodeInfoData.NodeGUID;
								}
							}
							break;
                    	case (OutputTypePortGuid):

							_DBG_INFO(("Extracting PortGUIDS from NodeRecord - NumRecords[%d]\n",
															NumRecords));
                           	if (NumRecords > 0 )
                            {
                                pGuidResults = (PGUID_RESULTS)pClientResult->QueryResult;
                                pGuidResults->NumGuids = NumRecords;
								for (i = 0, p = pSaMad->Data;
									i < NumRecords; i++, p+=PaddedRecordSize)
                        		{
									pGuidResults->Guids[i] = 
										((IB_NODE_RECORD*)p)->NodeInfoData.PortGUID;

                        		}
							}
							break;
						default:
							break;
					}
					break;
				}
				case (OutputTypeNodeDesc):
				{
    				PNODEDESC_RESULTS 		pNodeDescResults;


                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
									+ (NumRecords * sizeof(NODEDESC_RESULTS)),
									IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(NODEDESC_RESULTS);

					_DBG_INFO(("Extracting NodeDescription from NodeRecord - NumRecords[%d]\n",
														NumRecords));
					if ( NumRecords > 0 )	
					{
                    	pNodeDescResults = (PNODEDESC_RESULTS)pClientResult->QueryResult;
						pNodeDescResults->NumDescs = NumRecords;
						for (i = 0, p = pSaMad->Data;
							i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                    		pNodeDescResults->NodeDescs[i] = 
									((IB_NODE_RECORD*)p)->NodeDescData;
                    	}
					}
                    break;
				}

                case (OutputTypeNodeRecord):
				{
    				PNODE_RECORD_RESULTS	pNodeInfoResults;


                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(NODE_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(NODE_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete NodeRecord - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0 )
                    {
                    	pNodeInfoResults = (PNODE_RECORD_RESULTS)pClientResult->QueryResult;
                    	pNodeInfoResults->NumNodeRecords = NumRecords;

						for (i = 0, p = pSaMad->Data;
							i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                        	pNodeInfoResults->NodeRecords[i] = 
									*((IB_NODE_RECORD*)p);
                    	}
					}
                    break;
				}
                case (OutputTypePathRecord):
                {
                	uint64 *pDestPortGuidsList;
                	uint32 DestPortGuidsCount;
					QueryDetails **pChildQueryElements;

                	extern uint32 NumTotalPorts;
 
                	if(NumTotalPorts == 0)
                	{
                    	_DBG_INFO(("Total Number of Ports turns out to be zero\n"));
                    	break;
                	}

                    if ( NumRecords == 0)
                    {
                    	pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES),
                                                                IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    	if (pClientResult == NULL)
							goto failalloc;
                    	pClientResult->ResultDataSize = 0;
						break;
					}

                    pDestPortGuidsList = (EUI64*)MemoryAllocate2AndClear((sizeof(EUI64)*NumRecords),
							IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION,
							SUBNET_DRIVER_TAG);

					DestPortGuidsCount = NumRecords;
                    if (pDestPortGuidsList == NULL)
                    {
                        _DBG_ERROR(("Cannot allocate memory for DestPortGuids\n"));
                        goto failalloc2;
                    }
					 
					for (i = 0, p = pSaMad->Data;
							i < NumRecords; i++, p+=PaddedRecordSize)
                    {
                        pDestPortGuidsList[i] = 
							((IB_NODE_RECORD*)p)->NodeInfoData.PortGUID;
                    } 
 
                	// Multistaged query
                	pChildQueryElements = BuildSAChildQueriesForPathRecord(
														pQueryElement,
                    									pDestPortGuidsList, 
														DestPortGuidsCount);
                	MemoryDeallocate(pDestPortGuidsList);
					if(! pChildQueryElements)
                	{
                        goto failalloc2;
                	}
					SpinLockAcquire(&pQueryList->m_Lock);
					if (FreeCancelledQueryElement(pQueryElement))
					{
						SpinLockRelease(&pQueryList->m_Lock);
						for(i = 0; i<DestPortGuidsCount; i++)
						{
							FreeQueryElement(pChildQueryElements[i]);
						}
						MemoryDeallocate(pChildQueryElements);
						SpinLockAcquire(&pQueryList->m_Lock);
						goto ret;
					}
                	pQueryElement->QState = WaitingForChildToComplete;
                	//Leave pQueryElement on pQueryList
					DEBUG_ASSERT(pQueryElement->SdSentCmd == UserQueryFabricInformation);
					DEBUG_ASSERT(pQueryElement->cmd.query.pClientResult == NULL);
					// start the child queries
					pQueryElement->NoOfChildQuery = DestPortGuidsCount;
					for(i = 0; i<DestPortGuidsCount; i++)
					{
						// Put this on to the child query List
						LQListInsertHead(pChildQueryList, &(pChildQueryElements[i])->QueryListItem);
					}
					SpinLockRelease(&pQueryList->m_Lock);
					MemoryDeallocate(pChildQueryElements);
					SpinLockAcquire(&pQueryList->m_Lock);
					// completion processing will processes pChildQueryList
					ScheduleCompletionProcessing();
					goto ret;
                }
				default:
					break;
			}
			break;
		}

		case	SA_ATTRIB_PORTINFO_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_PORTINFO_RECORD((IB_PORTINFO_RECORD*)p, TRUE);
            }
			_DBG_INFO(("Processing IB_PORTINFO_RECORD..... NumRecords[%d]\n",
										NumRecords));
 
            switch(QueryOutputType)
            {

                case (OutputTypePortInfoRecord):
				{
					PPORTINFO_RECORD_RESULTS	pPortInfoResults;

                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
                                                                + (NumRecords * sizeof(PORTINFO_RECORD_RESULTS)),
                                                                IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(PORTINFO_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete PortInfoRecords - NumRecords[%d]\n",
														NumRecords));
					
					if ( NumRecords > 0)
					{
                    	pPortInfoResults = (PPORTINFO_RECORD_RESULTS)pClientResult->QueryResult;
                    	pPortInfoResults->NumPortInfoRecords = NumRecords;

						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                        	pPortInfoResults->PortInfoRecords[i] = 
								*((IB_PORTINFO_RECORD*)p);
                    	}
					}
                    break;
				}
                default:
                    break;
			}
			break;
		}
		case	SA_ATTRIB_SMINFO_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_SMINFO_RECORD((IB_SMINFO_RECORD*)p);
            }
 
			_DBG_INFO(("Processing IB_SMINFO_RECORD..... NumRecords[%d]\n",
										NumRecords));
            switch(QueryOutputType)
            {
 
                case (OutputTypeSMInfoRecord):
                {
                    PSMINFO_RECORD_RESULTS    pSMInfoResults;

                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(SMINFO_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(SMINFO_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete SMInfoRecords - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
                    	pSMInfoResults = (PSMINFO_RECORD_RESULTS)pClientResult->QueryResult;
                    	pSMInfoResults->NumSMInfoRecords = NumRecords;

						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                        	pSMInfoResults->SMInfoRecords[i] = 
										*((IB_SMINFO_RECORD*)p);
                    	}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
		case	SA_ATTRIB_LINK_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_LINK_RECORD((IB_LINK_RECORD*)p);
            }
            _DBG_INFO(("Processing IB_LINK_RECORD..... NumRecords[%d]\n",
                                        NumRecords));
            switch(QueryOutputType)
            {
                case (OutputTypeLinkRecord):
                {
                    PLINK_RECORD_RESULTS    pLinkResults;
 
                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(LINK_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(LINK_RECORD_RESULTS);

                    _DBG_INFO(("Extracting complete LinkRecords - NumRecords[%d]\n",
                                                        NumRecords));
                    if ( NumRecords > 0)
                    {
                    	pLinkResults = (PLINK_RECORD_RESULTS)pClientResult->QueryResult;
                    	pLinkResults->NumLinkRecords = NumRecords;
 
						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                        	pLinkResults->LinkRecords[i] = *((IB_LINK_RECORD*)p);
                    	}
					}
                    break;
                }
                default:
                    break;
            }
			break;
        }
		case	SA_ATTRIB_PATH_RECORD:
		{
			uint32 PktLifeTime = 0;
			if (g_SdParams.MinPktLifeTime)
				PktLifeTime = TimeoutTimeMsToMult(g_SdParams.MinPktLifeTime);
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_PATH_RECORD((IB_PATH_RECORD*)p);
				if (PktLifeTime > ((IB_PATH_RECORD*)p)->PktLifeTime)
					((IB_PATH_RECORD*)p)->PktLifeTime = PktLifeTime;
            }
			_DBG_INFO(("Processing IB_PATH_RECORD..... NumRecords[%d]\n",
										NumRecords));
            switch(QueryOutputType)
            {
 
                case (OutputTypePathRecord):
                {
                    PPATH_RESULTS    pPathRecordResults;

                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(PATH_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(PATH_RESULTS);

					_DBG_INFO(("Extracting complete PathRecord - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
                    	pPathRecordResults = (PPATH_RESULTS)pClientResult->QueryResult;
                    	pPathRecordResults->NumPathRecords = NumRecords;

						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                        	pPathRecordResults->PathRecords[i] = 
										*((IB_PATH_RECORD*)p);
                    	}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
		case	SA_ATTRIB_SERVICE_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_SERVICE_RECORD((IB_SERVICE_RECORD*)p);
            }
			_DBG_INFO(("Processing IB_SERVICE_RECORD..... NumRecords[%d]\n",
						NumRecords));

            switch(QueryOutputType)
            {
 
                case (OutputTypeServiceRecord):
                {
                    PSERVICE_RECORD_RESULTS    pServiceResults;
 
                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(SERVICE_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(SERVICE_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete ServiceRecords - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
						pServiceResults = (PSERVICE_RECORD_RESULTS)pClientResult->QueryResult;
                   		pServiceResults->NumServiceRecords = NumRecords;
						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                   			pServiceResults->ServiceRecords[i] = *((IB_SERVICE_RECORD*)p);
						}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
		case	SA_ATTRIB_MCMEMBER_RECORD:
		{
			uint32 PktLifeTime = 0;
			if (g_SdParams.MinPktLifeTime)
				PktLifeTime = TimeoutTimeMsToMult(g_SdParams.MinPktLifeTime);
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_MCMEMBER_RECORD((IB_MCMEMBER_RECORD*)p);
				if (PktLifeTime > ((IB_MCMEMBER_RECORD*)p)->PktLifeTime)
					((IB_MCMEMBER_RECORD*)p)->PktLifeTime = PktLifeTime;
            }
			_DBG_INFO(("Processing IB_MCMEMBER_RECORD..... NumRecords[%d]\n",
						NumRecords));

            switch(QueryOutputType)
            {
 
                case (OutputTypeMcMemberRecord):
                {
                    PMCMEMBER_RECORD_RESULTS    pMcMemberResults;
 
                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(MCMEMBER_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(MCMEMBER_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete McMemberRecords - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
						pMcMemberResults = (PMCMEMBER_RECORD_RESULTS)pClientResult->QueryResult;
                   		pMcMemberResults->NumMcMemberRecords = NumRecords;
						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                   			pMcMemberResults->McMemberRecords[i] = *((IB_MCMEMBER_RECORD*)p);
						}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
		case	SA_ATTRIB_INFORM_INFO_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_INFORM_INFO_RECORD((IB_INFORM_INFO_RECORD*)p);
            }
			_DBG_INFO(("Processing IB_INFORM_INFO_RECORD..... NumRecords[%d]\n",
						NumRecords));

            switch(QueryOutputType)
            {
 
                case (OutputTypeInformInfoRecord):
                {
                    PINFORM_INFO_RECORD_RESULTS    pInformInfoResults;
 
                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(INFORM_INFO_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(INFORM_INFO_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete InformInfoRecords - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
						pInformInfoResults = (PINFORM_INFO_RECORD_RESULTS)pClientResult->QueryResult;
                   		pInformInfoResults->NumInformInfoRecords = NumRecords;
						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                   			pInformInfoResults->InformInfoRecords[i] = *((IB_INFORM_INFO_RECORD*)p);
						}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
        case	SA_ATTRIB_TRACE_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_TRACE_RECORD((IB_TRACE_RECORD*)p);
            }
			_DBG_INFO(("Processing IB_TRACE_RECORD..... NumRecords[%d]\n",
										NumRecords));

            switch(QueryOutputType)
            {
 
                case (OutputTypeTraceRecord):
                {
                    PTRACE_RECORD_RESULTS    pTraceRecordResults;

                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(TRACE_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(TRACE_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete TraceRecord - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
                    	pTraceRecordResults = (PTRACE_RECORD_RESULTS)pClientResult->QueryResult;
                    	pTraceRecordResults->NumTraceRecords = NumRecords;

						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                        	pTraceRecordResults->TraceRecords[i] = 
										*((IB_TRACE_RECORD*)p);
                            // convert encrypted fields
                            pTraceRecordResults->TraceRecords[i].GIDPrefix ^= IB_TRACE_RECORD_COMP_ENCRYPT_MASK;
                            pTraceRecordResults->TraceRecords[i].NodeID ^= IB_TRACE_RECORD_COMP_ENCRYPT_MASK;
                            pTraceRecordResults->TraceRecords[i].ChassisID ^= IB_TRACE_RECORD_COMP_ENCRYPT_MASK;
                            pTraceRecordResults->TraceRecords[i].EntryPortID ^= IB_TRACE_RECORD_COMP_ENCRYPT_MASK;
                            pTraceRecordResults->TraceRecords[i].ExitPortID ^= IB_TRACE_RECORD_COMP_ENCRYPT_MASK;
                    	}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
		case	SA_ATTRIB_SWITCHINFO_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_SWITCHINFO_RECORD((IB_SWITCHINFO_RECORD*)p);
            }
			_DBG_INFO(("Processing IB_SWITCHINFO_RECORD..... NumRecords[%d]\n",
						NumRecords));

            switch(QueryOutputType)
            {
 
                case (OutputTypeSwitchInfoRecord):
                {
                    PSWITCHINFO_RECORD_RESULTS    pSwitchInfoResults;
 
                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(SWITCHINFO_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(SWITCHINFO_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete SwitchInfoRecords - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
						pSwitchInfoResults = (PSWITCHINFO_RECORD_RESULTS)pClientResult->QueryResult;
                   		pSwitchInfoResults->NumSwitchInfoRecords = NumRecords;
						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                   			pSwitchInfoResults->SwitchInfoRecords[i] = *((IB_SWITCHINFO_RECORD*)p);
						}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
		case	SA_ATTRIB_LINEAR_FWDTBL_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_LINEAR_FDB_RECORD((IB_LINEAR_FDB_RECORD*)p);
            }
			_DBG_INFO(("Processing IB_LINEAR_FDB_RECORD..... NumRecords[%d]\n",
						NumRecords));

            switch(QueryOutputType)
            {
 
                case (OutputTypeLinearFDBRecord):
                {
                    PLINEAR_FDB_RECORD_RESULTS    pLinearFDBResults;
 
                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(LINEAR_FDB_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(LINEAR_FDB_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete LinearFDBRecords - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
						pLinearFDBResults = (PLINEAR_FDB_RECORD_RESULTS)pClientResult->QueryResult;
                   		pLinearFDBResults->NumLinearFDBRecords = NumRecords;
						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                   			pLinearFDBResults->LinearFDBRecords[i] = *((IB_LINEAR_FDB_RECORD*)p);
						}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
		case	SA_ATTRIB_RANDOM_FWDTBL_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_RANDOM_FDB_RECORD((IB_RANDOM_FDB_RECORD*)p);
            }
			_DBG_INFO(("Processing IB_RANDOM_FDB_RECORD..... NumRecords[%d]\n",
						NumRecords));

            switch(QueryOutputType)
            {
 
                case (OutputTypeRandomFDBRecord):
                {
                    PRANDOM_FDB_RECORD_RESULTS    pRandomFDBResults;
 
                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(RANDOM_FDB_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(RANDOM_FDB_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete RandomFDBRecords - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
						pRandomFDBResults = (PRANDOM_FDB_RECORD_RESULTS)pClientResult->QueryResult;
                   		pRandomFDBResults->NumRandomFDBRecords = NumRecords;
						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                   			pRandomFDBResults->RandomFDBRecords[i] = *((IB_RANDOM_FDB_RECORD*)p);
						}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
		case	SA_ATTRIB_MCAST_FWDTBL_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_MCAST_FDB_RECORD((IB_MCAST_FDB_RECORD*)p);
            }
			_DBG_INFO(("Processing IB_MCAST_FDB_RECORD..... NumRecords[%d]\n",
						NumRecords));

            switch(QueryOutputType)
            {
 
                case (OutputTypeMCastFDBRecord):
                {
                    PMCAST_FDB_RECORD_RESULTS    pMCastFDBResults;
 
                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(MCAST_FDB_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(MCAST_FDB_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete MCastFDBRecords - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
						pMCastFDBResults = (PMCAST_FDB_RECORD_RESULTS)pClientResult->QueryResult;
                   		pMCastFDBResults->NumMCastFDBRecords = NumRecords;
						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                   			pMCastFDBResults->MCastFDBRecords[i] = *((IB_MCAST_FDB_RECORD*)p);
						}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
		case	SA_ATTRIB_VLARBTABLE_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_VLARBTABLE_RECORD((IB_VLARBTABLE_RECORD*)p);
            }
			_DBG_INFO(("Processing IB_VLARBTABLE_RECORD..... NumRecords[%d]\n",
						NumRecords));

            switch(QueryOutputType)
            {
 
                case (OutputTypeVLArbTableRecord):
                {
                    PVLARBTABLE_RECORD_RESULTS    pVLArbTableResults;
 
                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(VLARBTABLE_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(VLARBTABLE_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete VLArbTableRecords - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
						pVLArbTableResults = (PVLARBTABLE_RECORD_RESULTS)pClientResult->QueryResult;
                   		pVLArbTableResults->NumVLArbTableRecords = NumRecords;
						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                   			pVLArbTableResults->VLArbTableRecords[i] = *((IB_VLARBTABLE_RECORD*)p);
						}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
		case	SA_ATTRIB_P_KEY_TABLE_RECORD:
		{
			for (i = 0, p = pSaMad->Data;
					i < NumRecords; i++, p+=PaddedRecordSize)
            {
				BSWAP_IB_P_KEY_TABLE_RECORD((IB_P_KEY_TABLE_RECORD*)p);
            }
			_DBG_INFO(("Processing IB_P_KEY_TABLE_RECORD..... NumRecords[%d]\n",
						NumRecords));

            switch(QueryOutputType)
            {
 
                case (OutputTypePKeyTableRecord):
                {
                    PPKEYTABLE_RECORD_RESULTS    pPKeyTableResults;
 
                    pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(sizeof(QUERY_RESULT_VALUES)
								+ (NumRecords * sizeof(PKEYTABLE_RECORD_RESULTS)),
								IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION, SUBNET_DRIVER_TAG);
                    if (pClientResult == NULL)
						goto failalloc;
                    pClientResult->ResultDataSize = NumRecords * sizeof(PKEYTABLE_RECORD_RESULTS);

					_DBG_INFO(("Extracting complete PKeyTableRecords - NumRecords[%d]\n",
														NumRecords));
                    if ( NumRecords > 0)
                    {
						pPKeyTableResults = (PPKEYTABLE_RECORD_RESULTS)pClientResult->QueryResult;
                   		pPKeyTableResults->NumPKeyTableRecords = NumRecords;
						for (i = 0, p = pSaMad->Data;
								i < NumRecords; i++, p+=PaddedRecordSize)
                    	{
                   			pPKeyTableResults->PKeyTableRecords[i] = *((IB_P_KEY_TABLE_RECORD*)p);
						}
					}
                    break;
                }
                default:
                    break;
            }
			break;
		}
		default:
			break;
	}
	SpinLockAcquire(&pQueryList->m_Lock);
	if (FreeCancelledQueryElement(pQueryElement))
		goto ret;
	DEBUG_ASSERT(pQueryElement->SdSentCmd == UserQueryFabricInformation);
	DEBUG_ASSERT(pQueryElement->cmd.query.pClientResult == NULL);
	pQueryElement->cmd.query.pClientResult = pClientResult;
	if (NULL != pClientResult)
	{
		pClientResult->Status = pQueryElement->Status;
		pClientResult->MadStatus = pQueryElement->MadStatus;
	}

done:
	// Insert the Queryelement into the CompletionList
	QListRemoveItem( &pQueryList->m_List, &pQueryElement->QueryListItem);
	pQueryElement->QState = QueryComplete;
	LQListInsertHead(pCompletionList, &pQueryElement->QueryListItem);  
	ScheduleCompletionProcessing();
ret:
	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
	return (FSUCCESS);

failalloc:
	_DBG_ERROR(("Cannot allocate memory for SA Response\n"));
failalloc2:
	SpinLockAcquire(&pQueryList->m_Lock);
	if (FreeCancelledQueryElement(pQueryElement))
		goto ret;
	DEBUG_ASSERT(pQueryElement->SdSentCmd == UserQueryFabricInformation);
	DEBUG_ASSERT(pQueryElement->cmd.query.pClientResult == NULL);
	pQueryElement->Status = FINSUFFICIENT_MEMORY;
	pQueryElement->MadStatus = 0;
	goto done;
}


// called with pQueryList->m_Lock already held
// This handles the 2nd tier of a 2 tiered query
// currently only 2nd tier is a GetTable for PathRecord
// called with buffer==NULL and messageSize == 0 if query has timed out
// pQueryElement is in WaitingForResult, still on list
FSTATUS 
SdHandleChildQuery(
	PQueryDetails pQueryElement,
	void *buffer,			// caller removed GRH already
	uint32 messageSize		// caller removed GRH already
	)
{
	//FSTATUS Fstatus = FERROR;
	QUERY_RESULT_VALUES * pClientResult = NULL;
	uint32 NumRecords;
	uint32 RecordSize;
	uint32 PaddedRecordSize;
	uint32 PreviousNumPathRecords = 0;
	uint32 PrevDataSize = 0;
	SA_MAD *pSaMad = (SA_MAD*)buffer;
	char *temp;
	PPATH_RESULTS pPathResults = NULL;
	PQueryDetails pRootQueryElement = pQueryElement->pParentQuery;
    
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, SdHandleChildQuery);
	//Check for the Parent Query
	// pQueryList->m_Lock already held
	// this does not dereference pParentQuery
	if (! QListIsItemInList(&pQueryList->m_List,&pRootQueryElement->QueryListItem))
	{
		_DBG_INFO(("Could not find the root queryelement\n"));
		goto childdone;
	}

	if (buffer == NULL) //Timeout 
	{
		_DBG_ERROR(("SA Query Timed Out <%d ms, %d retries>\n",
			pQueryElement->ControlParameters.Timeout,
			pQueryElement->ControlParameters.RetryCount));
		pQueryElement->Status = FTIMEOUT;
		pQueryElement->MadStatus = 0;	// unknown, no response
		goto done;
	}
	// Include MAD Status and reserved field
	pQueryElement->MadStatus = (pSaMad->common.u.NS.Reserved1 << 16|
									pSaMad->common.u.NS.Status.AsReg16);
	if (pSaMad->common.u.NS.Status.AsReg16 == MAD_STATUS_SA_NO_RECORDS)
	{
		_DBG_INFO(("No Records MAD completion received\n"));
	} else if (pSaMad->common.u.NS.Status.AsReg16 != MAD_STATUS_SUCCESS) {
		_DBG_ERROR(("Failed MAD completion received, Status <0x%x: %s>, Port 0x%016"PRIx64"\n",
			pSaMad->common.u.NS.Status.AsReg16,
			_DBG_PTR(iba_sd_mad_status_msg(pQueryElement->MadStatus)),
			pQueryElement->PortGuid));
		pQueryElement->Status = FERROR; 
		goto done;
	}

	pQueryElement->Status = FSUCCESS; 
	_DBG_INFO(("Successful MAD completion received\n"));
	BSWAP_SA_HDR(&pSaMad->SaHdr);
	
	DEBUG_ASSERT(pRootQueryElement->SdSentCmd == UserQueryFabricInformation);
	pRootQueryElement->ChildProcessingResponseCount++;	// holds ref to parent
	pQueryElement->QState = ProcessingResponse;	// prevent timeout of child
	SpinLockRelease(&pQueryList->m_Lock);

	// This mutex prevents races between multiple child processes updating
	// their parents pClientResult at the same time.  In reality, since there
	// is one thread for SD GSA callbacks, there will be no contention for
	// this mutex, hence there is no advantage to putting it into the
	// parent query element.  Also note that for parent queries, the
	// pClientResult of the parent is not modified until all children are done
	MutexAcquire(pChildGrowParentMutex);
	if(pRootQueryElement->cmd.query.pClientResult)
	{
		PrevDataSize = pRootQueryElement->cmd.query.pClientResult->ResultDataSize;
		pPathResults =(PPATH_RESULTS)pRootQueryElement->cmd.query.pClientResult->QueryResult;
		PreviousNumPathRecords =  pPathResults->NumPathRecords;
	}else
	{
		PrevDataSize = sizeof(PATH_RESULTS) - sizeof(IB_PATH_RECORD);
		PreviousNumPathRecords = 0;
	}
	
	if (pSaMad->common.u.NS.Status.AsReg16 == MAD_STATUS_SA_NO_RECORDS)
	{
		NumRecords = 0;
	} else {
		(void)SdComputeResponseSizes(pSaMad, messageSize, &NumRecords,
								&PaddedRecordSize, &RecordSize);
	}
	if(NumRecords)
	{
		uint32 ResBuffSize = 0;
		uint32 PathRecSize = NumRecords*RecordSize;
		
		ResBuffSize = sizeof(QUERY_RESULT_VALUES) 
				+ PathRecSize + PrevDataSize;
		
		pClientResult = (QUERY_RESULT_VALUES*)MemoryAllocate2AndClear(ResBuffSize,
							IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION,
							SUBNET_DRIVER_TAG);
		if (pClientResult != NULL)  
		{
			uint8*	p;
			uint32 i;
			IB_PATH_RECORD *pIbPathRecDest;
			uint32 PktLifeTime = 0;
			
			pClientResult->Status = FSUCCESS;
			pClientResult->MadStatus = pQueryElement->MadStatus;
			pClientResult->ResultDataSize = PathRecSize + PrevDataSize;
			pPathResults = (PPATH_RESULTS)pClientResult->QueryResult;
			if (g_SdParams.MinPktLifeTime) {
				PktLifeTime = TimeoutTimeMsToMult(g_SdParams.MinPktLifeTime);
			}

			pIbPathRecDest = pPathResults->PathRecords;
			for (i = 0, p = pSaMad->Data;
					i < NumRecords;
					i++, p+=PaddedRecordSize, pIbPathRecDest++)
			{
				BSWAP_IB_PATH_RECORD((IB_PATH_RECORD*)p);
				MemoryCopy(pIbPathRecDest, p, sizeof(IB_PATH_RECORD));
				if (PktLifeTime > pIbPathRecDest->PktLifeTime)
                	pIbPathRecDest->PktLifeTime = PktLifeTime;
			}
			if(PreviousNumPathRecords>0)
			{
				//Append the Previous Data
				temp = (char *)pPathResults->PathRecords;
				temp = temp + PathRecSize;
				pPathResults = 
					(PPATH_RESULTS)pRootQueryElement->cmd.query.pClientResult->QueryResult;
				MemoryCopy(temp, 
					pPathResults->PathRecords,
					PreviousNumPathRecords * RecordSize);
				MemoryDeallocate(pRootQueryElement->cmd.query.pClientResult);
				
			}
			pPathResults = (PPATH_RESULTS)pClientResult->QueryResult;
			pPathResults->NumPathRecords = PreviousNumPathRecords + NumRecords;
			pRootQueryElement->cmd.query.pClientResult = pClientResult;
		}else {
			_DBG_ERROR( ("Cannot allocate memory for QueryElement\n"));
		}
	}
	MutexRelease(pChildGrowParentMutex);
	SpinLockAcquire(&pQueryList->m_Lock);
	pRootQueryElement->ChildProcessingResponseCount--;
	// children can't be cancelled directly
	DEBUG_ASSERT(pQueryElement->QState != QueryDestroy);
	if (FreeCancelledQueryElement(pRootQueryElement))
	{
		// parent has been cancelled, child is done but no parent
		goto childdone;
	}

done:
	// Associate this child to the parent
	(pRootQueryElement->NoOfChildQuery)--;
	if(pRootQueryElement->NoOfChildQuery == 0)
	{
		ASSERT(pRootQueryElement->QState == WaitingForChildToComplete);
		//Remove Parent from Qlist and insert it into CompletionList //NOTENOTE
		// pQueryList->m_Lock already held
		QListRemoveItem( &pQueryList->m_List, (LIST_ITEM *)pRootQueryElement );
			
		//Update the status from WaitingForChild to QueryComplete
		
		pRootQueryElement->QState = QueryComplete;
		
		// Insert the parent QueryElement into the CompletionList
		LQListInsertHead(pCompletionList, &pRootQueryElement->QueryListItem);  
	}
childdone:
	AtomicDecrementVoid(&ChildQueryActive);
	QListRemoveItem( &pQueryList->m_List, &pQueryElement->QueryListItem);
	if (buffer == NULL)
	{
		// called within TimerHandler, defer destroy to premptable context
		pQueryElement->QState = QueryDestroy;
		LQListInsertHead(pCompletionList, &pQueryElement->QueryListItem);  
	} else {
		// call free in a premptable context
		SpinLockRelease(&pQueryList->m_Lock);
		FreeQueryElement(pQueryElement);
		SpinLockAcquire(&pQueryList->m_Lock);
	}
	// run CompletionQueue to queue next child queries or complete
	// parent as appropriate
	ScheduleCompletionProcessing();
	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
	return(FSUCCESS);
}
 
//////////////////////////////////////////////////////
// called without pQueryList->m_Lock held
// pRootQueryElement is in ProcessingResponse state, so we can access it
// returns a list of Child queries which have been built and initialized
// but have not yet been places on ChildQueryList, so they have not yet
// been started
QueryDetails **
BuildSAChildQueriesForPathRecord(
	PQueryDetails pRootQueryElement, 
	uint64 *pDestPortGuidsList,
	uint32 DestPortGuidsCount
	)
{
	uint32 i;
	QueryDetails *pQueryElement;
	QueryDetails **pChildQueryElements;
	FSTATUS status;

	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, BuildSAChildQueriesForPathRecord);

	pChildQueryElements = (QueryDetails**)MemoryAllocate2AndClear(
							sizeof(QueryDetails*)*DestPortGuidsCount,
							IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION,
							SUBNET_DRIVER_TAG);
	if (NULL == pChildQueryElements)
	{
		_DBG_ERROR(("Cannot allocate memory for list of Child SA Queries\n"));
		goto done;
	}

	for(i = 0; i<DestPortGuidsCount; i++)
	{
		pQueryElement = (QueryDetails*)MemoryAllocate2AndClear(sizeof(QueryDetails),
							IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION,
							SUBNET_DRIVER_TAG);
		if (pQueryElement == NULL)  
		{
			_DBG_ERROR(("Cannot allocate memory for Child SA Query\n"));
			goto failalloc;
		}
		QListSetObj(&pQueryElement->QueryListItem, (void *)pQueryElement);

		pQueryElement->ControlParameters = pRootQueryElement->ControlParameters;

		pQueryElement->SdSentCmd = UserQueryFabricInformation;
		pQueryElement->cmd.query.UserQuery.InputType = InputTypePortGuidPair;
		pQueryElement->cmd.query.UserQuery.OutputType = OutputTypePathRecord;
		pQueryElement->cmd.query.UserQuery.InputValue.PortGuidPair.DestPortGuid=
						pDestPortGuidsList[i];
		pQueryElement->cmd.query.UserQuery.InputValue.PortGuidPair.SourcePortGuid = 
						pRootQueryElement->PortGuid;
		pQueryElement->cmd.query.pClientResult = NULL;
		pQueryElement->PortGuid = pRootQueryElement->PortGuid;
		// do not associate directly to client, we let destruction of our parent
		// be our control.
		pQueryElement->ClientHandle = NULL;

       	status = ValidateAndFillSDQuery(pQueryElement,
						IBA_MEM_FLAG_PREMPTABLE|IBA_MEM_FLAG_SHORT_DURATION);   
		if (FSUCCESS != status)
		{
			_DBG_ERROR(("Unable to allocate Mad for Child SA Query\n"));
			MemoryDeallocate(pQueryElement);
			goto failalloc;
		}

		//Update the parent pointer
		pQueryElement->pParentQuery = pRootQueryElement;
		pQueryElement->QState = ReadyToSend;

		pChildQueryElements[i] = pQueryElement;
	}

done:
	_DBG_LEAVE_EXT(_DBG_LVL_FUNC_TRACE);
	return pChildQueryElements;

failalloc:
	// we failed at i, free i-1 to 0
	while (i > 0)
		FreeQueryElement(pChildQueryElements[--i]);
	MemoryDeallocate(pChildQueryElements);
	pChildQueryElements = NULL;
	goto done;
}
