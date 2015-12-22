/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/


#include "gsamain.h"

//
// GsiCreateDgrmPool
//
//
//
// INPUTS:
//
//	ServiceHandle	- Handle returned upon registration
//
//	ElementCount	- Number of elements in pool
//
//	BuffersPerElement- Number of buffers in each pool element
//
//	BufferSizeArray[]- An array of buffers sizes. The length of this array must
//					  be equal to BuffersPerElement.
//
//	ContextSize		- Size of user context in each element
//	
//	
//
// OUTPUTS:
//
//	DgrmPoolHandle	- Opaque handle returned to identify datagram pool created
//
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
//
FSTATUS
iba_gsi_create_dgrm_pool(
	IN	IB_HANDLE				ServiceHandle,
	IN	uint32					ElementCount,
	IN	uint32                  BuffersPerElement,
	IN	uint32					BufferSizeArray[],
	IN	uint32                  ContextSize,
	OUT IB_HANDLE				*DgrmPoolHandle
	)
{
	FSTATUS						status;

	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_gsi_create_dgrm_pool);

	status = CreateDgrmPool(
						ServiceHandle,
						ElementCount,
						BuffersPerElement,
						BufferSizeArray,
						ContextSize,
						DgrmPoolHandle );

#if defined(DBG) || defined(IB_DEBUG)
	if ( FSUCCESS == status )
		((IBT_MEM_POOL *)*DgrmPoolHandle)->SigInfo = GSA_DGRM_SIG;
#endif

	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
    return status;
}


//
// GsiDestroyDgrmPool
//
//
//
// INPUTS:
//
//	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
//
//	
//
// OUTPUTS:
//
//	None
//
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
//
FSTATUS
iba_gsi_destroy_dgrm_pool(
	IN	IB_HANDLE				DgrmPoolHandle
	)
{
	FSTATUS						status;

	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_gsi_destroy_dgrm_pool);

#if defined(DBG) || defined(IB_DEBUG)
	if (GSA_DGRM_SIG != ((IBT_MEM_POOL *)DgrmPoolHandle)->SigInfo )
	{
		_DBG_ERROR(("Bad DgrmPool Handle passed by client!!!\n"));
		_DBG_BREAK;
	}
#else
#endif

	status = DestroyDgrmPool(DgrmPoolHandle);

	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
    return status;
}



//
// GsiDgrmPoolGet
//
//
//
// INPUTS:
//
//	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
//
//	ElementCount	- Number of elements to fetch from pool
//
//
//	
//
// OUTPUTS:
//
//	ElementCount	- Number of elements fetched from pool
//
//	DgrmList		- Pointer holds the datagram list if successful
//
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
//
FSTATUS
iba_gsi_dgrm_pool_get(
	IN	IB_HANDLE				DgrmPoolHandle,
	IN OUT	uint32				*ElementCount,
	OUT	IBT_DGRM_ELEMENT		**DgrmList
	)
{
	FSTATUS						status;

	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_gsi_dgrm_pool_get);

#if defined(DBG) || defined(IB_DEBUG)
	if (GSA_DGRM_SIG != ((IBT_MEM_POOL *)DgrmPoolHandle)->SigInfo )
	{
		_DBG_ERROR(("Bad DgrmPool Handle passed by client!!!\n"));
		_DBG_BREAK;
	}
#else
#endif

	status = DgrmPoolGet(
					DgrmPoolHandle,
					ElementCount,
					DgrmList );

	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
    return status;
}

//
// GsiDgrmPoolCount
//
//
//
// INPUTS:
//
//	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
//
//	
//
// OUTPUTS:
//
//
// RETURNS:
//
//	Number of datagrams in the pool
//
uint32
iba_gsi_dgrm_pool_count(
	IN	IB_HANDLE				DgrmPoolHandle
	)
{
	uint32 rval;
	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_gsi_dgrm_pool_count);
	rval = DgrmPoolCount(DgrmPoolHandle);
	_DBG_RETURN_LVL(_DBG_LVL_POOL, rval);
	return rval;
}

//
// GsiDgrmPoolTotal
//
//
//
// INPUTS:
//
//	DgrmPoolHandle	- Handle returned upon successful creation of Datagram pool
//
//	
//
// OUTPUTS:
//
//
// RETURNS:
//
//	Total Number of datagrams allocated in the pool
//
uint32
iba_gsi_dgrm_pool_total(
	IN	IB_HANDLE				DgrmPoolHandle
	)
{
	uint32 rval;
	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_gsi_dgrm_pool_total);
	rval = DgrmPoolTotal(DgrmPoolHandle);
	_DBG_RETURN_LVL(_DBG_LVL_POOL, rval);
	return rval;
}

//
// GsiDgrmPoolPut
//
//
//
// INPUTS:
//
//	DgrmList		- Datagram elements to return back to pool
//
//	
//
// OUTPUTS:
//
//	None
//
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
//
FSTATUS
iba_gsi_dgrm_pool_put(
	IN	IBT_DGRM_ELEMENT		*DgrmList
	)
{
	FSTATUS						status;

	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_gsi_dgrm_pool_put);


	status = DgrmPoolPut( DgrmList );

	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
    return status;
}

//
// CreateGrowableDgrmPool
//
//
//
// INPUTS:
//
//	ServiceHandle	- Handle returned upon registration
//
//	ElementCount	- Initial Number of elements in pool (can be 0)
//
//	BuffersPerElement- Number of buffers in each pool element (required)
//
//	BufferSizeArray[]- An array of buffers sizes. The length of this array must
//					  be equal to BuffersPerElement. (required)
//
//	ContextSize		- Size of user context in each element (required)
//	
//	
//
// OUTPUTS:
//
//	DgrmPoolHandle	- Opaque handle returned to identify datagram pool created
//
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
//

FSTATUS
iba_gsi_create_growable_dgrm_pool(
	IN	IB_HANDLE				ServiceHandle,
	IN	uint32					ElementCount,
	IN	uint32                  BuffersPerElement,
	IN	uint32					BufferSizeArray[],
	IN	uint32                  ContextSize,
	OUT IB_HANDLE				*DgrmPoolHandle
	)
{
	FSTATUS						status;

	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_gsi_create_growable_dgrm_pool);

	status = CreateGrowableDgrmPool(
						ServiceHandle,
						ElementCount,
						BuffersPerElement,
						BufferSizeArray,
						ContextSize,
						DgrmPoolHandle );

#if defined(DBG) || defined(IB_DEBUG)
	if ( FSUCCESS == status )
		((IBT_MEM_POOL *)*DgrmPoolHandle)->SigInfo = GSA_DGRM_SIG;
#endif

	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
    return status;
}

//
// DgrmPoolGrow
// Grow the size of a Growable Dgrm Pool
// This routine may preempt.
//
//
// INPUTS:
//	DgrmPoolHandle	- Opaque handle to datagram pool
//
//	ELementCount	- number of elements to add to pool
//
//	
//
// OUTPUTS:
//
//	None
//
//
// RETURNS:
//
//	FSUCCESS
//	FINVALID_PARAMETER
//	FINSUFFICIENT_RESOURCES
//
FSTATUS
iba_gsi_dgrm_pool_grow(
	IN	IB_HANDLE				DgrmPoolHandle,
	IN	uint32					ElementCount
	)
{
	FSTATUS						status;

	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_gsi_dgrm_pool_grow);

#if defined(DBG) || defined(IB_DEBUG)
	if (GSA_DGRM_SIG != ((IBT_MEM_POOL *)DgrmPoolHandle)->SigInfo )
	{
		_DBG_ERROR(("Bad DgrmPool Handle passed by client!!!\n"));
		_DBG_BREAK;
	}
#else
#endif

	status = DgrmPoolGrow(
					DgrmPoolHandle,
					ElementCount);

	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
    return status;
}

//
// DgrmPoolGrowAsNeeded
// Checks Pool size and schedules a system callback to grow it as needed
// This routine does not preempt.
//
//
// INPUTS:
//	DgrmPoolHandle	- Opaque handle to datagram pool
//
//	lowWater		- if Dgrm Pool available < this, we will grow it
//
//	maxElements		- limit on size of Dgrm Pool, will not grow beyond this
//
//	growIncrement	- amount to grow by if we are growing
//
//	
//
// OUTPUTS:
//
//	None
//
//
// RETURNS:
//
//	None
//
void
iba_gsi_dgrm_pool_grow_as_needed(
	IN	IB_HANDLE				DgrmPoolHandle,
	IN	uint32					lowWater,
	IN	uint32					maxElements,
	IN	uint32					growIncrement
	)
{
	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_gsi_dgrm_pool_grow_as_needed);

#if defined(DBG) || defined(IB_DEBUG)
	if (GSA_DGRM_SIG != ((IBT_MEM_POOL *)DgrmPoolHandle)->SigInfo )
	{
		_DBG_ERROR(("Bad DgrmPool Handle passed by client!!!\n"));
		_DBG_BREAK;
	}
#else
#endif

	DgrmPoolGrowAsNeeded( DgrmPoolHandle, lowWater, maxElements,
								growIncrement);

	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
}

//
// iba_gsi_dgrm_len
//
//  This reports the length of a Dgrm sequence as total of:
//		GRH
//		MAD_COMMON
//		RMPP_HEADER
//		class Header (SA, etc)
//		RMPP_DATA (possibly large)
//	returned messageSize will indicate the total size of the output.
//	RmppHdr.PayloadLen will not be adjusted, it will still reflect the
//	total RMPP_DATA which will account for a class Header in each
//	packet.
//
// INPUTS:
//	DgrmElement		- A list of Datagrams that need to be analyzed
//	ClassHeaderSize	- size of class header
//						this is expected to be in every RMPP packet
//						immediately after the RMPP_HEADER
//						size will account for one copy of the Class header
//						from the 1st RMPP DATA of the message
//
// OUTPUTS:
//	None
//
// RETURNS:
//	Message Size.
//	if invalid RMPP sequence, returns 0
//
uint32									
iba_gsi_dgrm_len(
	IN IBT_DGRM_ELEMENT			*DgrmElement,
	IN uint32					ClassHeaderSize
	)
{
	uint32						messageSize = 0;
	IBT_BUFFER					*pBufferHdr;
	MAD_RMPP					*pMadRmpp;
	uint32						rmppGsDataSize;

	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_gsi_dgrm_len);

	// Calculate total bytes. 
	// Each buffer has a GRH+Mad Common Header+RMPP Header+ClassHeader+RMPP Data
	//
	// We report only one copy of the headers and coalesce the data

	pBufferHdr	= DgrmElement->Element.pBufferList;
	pMadRmpp	= (MAD_RMPP*)pBufferHdr->pNextBuffer->pData;// 2nd buffer is MAD

	rmppGsDataSize = (pMadRmpp->common.BaseVersion == IB_BASE_VERSION)
		? IBA_RMPP_GS_DATASIZE : STL_RMPP_GS_DATASIZE;
	
	// Check overloaded field 'base' if the recvd data is SAR

	// indicates receive processing identified a multi-packet RMPP message
	if ( ((IBT_DGRM_ELEMENT_PRIV *)DgrmElement)->Base )
	{
		uint32 segs;
		segs = (pMadRmpp->RmppHdr.u2.PayloadLen + rmppGsDataSize-1)/rmppGsDataSize;
			// assumes last packet includes a full ClassHeader
		// we saved total Data length into PayloadLen of first
		messageSize = sizeof(IB_GRH) + sizeof(MAD_COMMON) + 
							sizeof(RMPP_HEADER) + 
							ClassHeaderSize + 
							pMadRmpp->RmppHdr.u2.PayloadLen - segs*ClassHeaderSize;
		if ((pMadRmpp->RmppHdr.u2.PayloadLen -(segs-1)*rmppGsDataSize < ClassHeaderSize)
			|| (pMadRmpp->RmppHdr.u2.PayloadLen < ClassHeaderSize))
		{
			_DBG_ERROR(("Bad RMPP Packet: Payloadlen does not allow for complete class header in last packet: segs=%u datasize=%u Payload=%u ClassHeader=%u\n",
				segs, (unsigned)rmppGsDataSize, pMadRmpp->RmppHdr.u2.PayloadLen, ClassHeaderSize));
			goto fail;
#if 0
			// workaround, length doesn't allow for complete class
			// header in final packet, so drop final packet
			segs--;
			messageSize = sizeof(IB_GRH) + sizeof(MAD_COMMON) + 
							sizeof(RMPP_HEADER) + 
							ClassHeaderSize + 
							TBD - downgrade PayloadLen by the fluff
							pMadRmpp->RmppHdr.u2.PayloadLen - segs*ClassHeaderSize;
			if (segs == 0)
			{
				_DBG_ERROR(("RMPP Bug: Payloadlen does not allow for any class headers: segs=%u datasize=%u Payload=%u ClassHeader=%u\n",
				segs, IBA_RMPP_GS_DATASIZE, pMadRmpp->RmppHdr.u2.PayloadLen, ClassHeaderSize));
				goto fail;
			} else {
				ASSERT(! (pMadRmpp->RmppHdr.u2.PayloadLen -(segs-1)*IBA_RMPP_GS_DATASIZE < ClassHeaderSize));
			}
#endif
		}
	} else {
		messageSize = sizeof(IB_GRH) + sizeof(MAD);
	}

	_DBG_PRINT(_DBG_LVL_POOL,("Total bytes: %d\n", messageSize));

	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
	return messageSize;

fail:
	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
	return 0;
}

//
// GsiCoalesceDgrm
//
//	The Coalesce function will create an output as follows:
//		GRH
//		MAD_COMMON
//		RMPP_HEADER
//		class Header (SA, etc)
//		RMPP_DATA (possibly large)
//	returned messageSize will indicate the total size of the output
//	RmppHdr.PayloadLen will not be adjusted, it will reflect the
//	total RMPP_DATA which will account for a class Header in each
//	packet.
//
// INPUTS:
//	DgrmElement		- A list of Datagrams that need to be coalesced
//	ppBuffer		- *ppBuffer is Client buffer to coalesce to
//						if *ppBuffer is NULL, this routine will
//						compute the necessary size of the buffer and
//						allocate it on behalf of the caller
//	ClassHeaderSize	- size of class header
//						this is expected to be in every RMPP packet
//						immediately after the RMPP_HEADER
//						a copy of the Class header from the 1st RMPP
//						DATA of the message will be included in the
//						Coalesced output buffer
//	MemAllocFlags		- flags for memory allocate of buffer
//						for larger responses, it is recommended caller
//						be structured such that IBA_MEM_FLAG_PREEMPTABLE
//						can be used
//
// OUTPUTS:
//	None
//
// RETURNS:
//	Message Size.
//	if unable to allocate buffer, *ppBuffer is left as NULL
//	if invalid RMPP sequence, returns 0. In which case *ppBuffer will be NULL.
//
uint32									
iba_gsi_coalesce_dgrm(
	IN IBT_DGRM_ELEMENT			*DgrmElement,
	IN uint8					**ppBuffer,
	IN uint32					ClassHeaderSize
	)
{
	return iba_gsi_coalesce_dgrm2(DgrmElement, ppBuffer, ClassHeaderSize,
									IBA_MEM_FLAG_NONE);
}

void
DumpDgrmElement(
	IN IBT_DGRM_ELEMENT			*DgrmElement
	)
{
	IBT_ELEMENT					*pElement;
	IBT_BUFFER					*pBufferHdr;
	MAD_RMPP					*pMadRmpp;
	uint32						packet = 0;

	_DBG_ENTER_LVL(_DBG_LVL_POOL, DumpDgrmElement);

	_DBG_ERROR(("DgrmElement %p DgrmElement->RecvByteCount %d\n",
			_DBG_PTR(DgrmElement), DgrmElement->TotalRecvSize));

	pElement	= &DgrmElement->Element;
	pBufferHdr	= pElement->pBufferList;
	pMadRmpp	= (MAD_RMPP*)GsiDgrmGetRecvMad(DgrmElement);
	while (pElement)
	{
		uint32 Length;

		_DBG_ERROR(("pElement %p packet %d Element RecvByteCount %d pMadRmpp %p\n",
			_DBG_PTR(pElement), packet, pElement->RecvByteCount, _DBG_PTR(pMadRmpp)));
		packet++;

		Length = pElement->RecvByteCount;
		while (Length && pBufferHdr)
		{
			if (pBufferHdr->pData == pMadRmpp)
			{
				/* MAD fields */
				_DBG_ERROR(("Mad MgmtClass %d mr %x Status %d tid %"PRIx64" AttributeID %d\n",
					pMadRmpp->common.MgmtClass,
					pMadRmpp->common.mr.AsReg8,
					pMadRmpp->common.u.NS.Status.AsReg16,
					pMadRmpp->common.TransactionID,
					pMadRmpp->common.AttributeID));
					
				/* RMPP fields */
				_DBG_ERROR(("RmppType %u RmppFlags %x RmppStatus %u u1 %u u2 %u\n",
					pMadRmpp->RmppHdr.RmppType,
					pMadRmpp->RmppHdr.RmppFlags.AsReg8,
					pMadRmpp->RmppHdr.RmppStatus,
					pMadRmpp->RmppHdr.u1.AsReg32,
					pMadRmpp->RmppHdr.u2.AsReg32));
				_DBG_ERROR(("pBufferHdr->pData %p ByteCount %d Length %d\n",
					_DBG_PTR(pBufferHdr->pData), pBufferHdr->ByteCount, Length));
				if (g_GsaSettings.FullPacketDump)
				{
					BSWAP_RMPP_HEADER (&pMadRmpp->RmppHdr);
				}
			}
			else
			{
				if (g_GsaSettings.FullPacketDump)
				{
					_DBG_ERROR(("pBufferHdr->pData %p ByteCount %d Length %d\n",
						_DBG_PTR(pBufferHdr->pData), pBufferHdr->ByteCount, Length));
				}
			}

			if (g_GsaSettings.FullPacketDump)
			{
				BSWAP_MAD_HEADER((MAD *)&pMadRmpp->common);

				_DBG_DUMP_MEMORY(_DBG_LVL_ERROR, ("Recv GSI Packet Buffer"),
                                pBufferHdr->pData,
                                MIN(pBufferHdr->ByteCount, Length));

				BSWAP_MAD_HEADER((MAD *)&pMadRmpp->common);
			}

			if (pBufferHdr->pData == pMadRmpp)
			{
				if (g_GsaSettings.FullPacketDump)
					BSWAP_RMPP_HEADER (&pMadRmpp->RmppHdr);
			}

			Length -= MIN(pBufferHdr->ByteCount,Length);
			pBufferHdr = pBufferHdr->pNextBuffer;
		}

		pElement = pElement->pNextElement;
		if (pElement)
		{
			pBufferHdr	= pElement->pBufferList;
			pMadRmpp	= (MAD_RMPP*)GsiDgrmGetRecvMad((IBT_DGRM_ELEMENT *)pElement);
		}
	}
	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
}

uint32									
iba_gsi_coalesce_dgrm2(
	IN IBT_DGRM_ELEMENT			*DgrmElement,
	IN uint8					**ppBuffer,
	IN uint32					ClassHeaderSize,
	IN uint32					MemAllocFlags
	)
{
	uint32						messageSize = 0;
	uint32						byteCount, totalBytesToCopy;
	IBT_ELEMENT					*pElement;
	IBT_BUFFER					*pBufferHdr;
	uint8						*pBuffer;
	MAD_RMPP					*pMadRmpp;
#ifdef TEST_DUMP
	static int counter;
#endif

	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_gsi_coalesce_dgrm);

	pElement	= &DgrmElement->Element;
	pBufferHdr	= pElement->pBufferList;
	pMadRmpp	= (MAD_RMPP*)pBufferHdr->pNextBuffer->pData;// 2nd buffer is MAD
	
	// Calculate total bytes to copy. 
	// Each buffer has a GRH+Mad Common Header+RMPP Header+ClassHeader+RMPP Data
	//
	// We copy only one copy of the headers and coalesce the data
	messageSize = totalBytesToCopy = iba_gsi_dgrm_len(DgrmElement, ClassHeaderSize);
	if (! messageSize)
		goto fail;
	_DBG_PRINT(_DBG_LVL_POOL,("Total bytes to copy: %d\n", totalBytesToCopy));

	if (*ppBuffer == NULL)
	{
		*ppBuffer = (uint8*)MemoryAllocate2(totalBytesToCopy, MemAllocFlags, GSA_MEM_TAG);
		if (*ppBuffer == NULL)
			goto fail;
	}
	pBuffer		= *ppBuffer;

	// Copy GRH
	if (pBufferHdr->ByteCount != sizeof(IB_GRH)) {
		_DBG_ERROR(("Bad buffer header size: %u expected %u\n", pBufferHdr->ByteCount, (unsigned)sizeof(IB_GRH)));
		messageSize = 0;	// indicate invalid packets
		goto fail;
	}
	byteCount = pBufferHdr->ByteCount;
	MemoryCopy( pBuffer, pBufferHdr->pData, byteCount ); 
	pBuffer += byteCount;
	totalBytesToCopy -= byteCount; // keep count
	pBufferHdr = pBufferHdr->pNextBuffer;
	
	// Copy Mad Common + RMPP Header + class header + first data area
	byteCount = (totalBytesToCopy > pBufferHdr->ByteCount) ? \
					pBufferHdr->ByteCount:totalBytesToCopy;
	MemoryCopy( pBuffer, pBufferHdr->pData, byteCount );
	pBuffer += byteCount;
	totalBytesToCopy -= byteCount; // keep count
	
	// Walk all elements of RMPP list
	while ( totalBytesToCopy )
	{
		pElement = pElement->pNextElement;

		if ( NULL == pElement )
		{
			_DBG_ERROR(("LIST ERROR DgrmElement %p ClassHeaderSize %d messageSize %d BytesLeft %d\n", 
					_DBG_PTR(DgrmElement), ClassHeaderSize, messageSize, totalBytesToCopy));
			DumpDgrmElement(DgrmElement);
			messageSize = 0;	// indicate invalid packets/buffers
			goto fail;
		}
		else
		{
#ifdef TEST_DUMP
			if ((++counter%50) == 0)
			{
				_DBG_ERROR(("LIST ERROR DgrmElement %p ClassHeaderSize %d messageSize %d BytesLeft %d\n", 
					_DBG_PTR(DgrmElement), ClassHeaderSize, messageSize, totalBytesToCopy));
				DumpDgrmElement(DgrmElement);
				messageSize = 0;	// indicate invalid packets/buffers
				goto fail;
			}
#endif
		}

		// skip GRH
		pBufferHdr = pElement->pBufferList->pNextBuffer;
		
		pMadRmpp = (MAD_RMPP *)pBufferHdr->pData;

		// copy as much class data as is left in packet
		// subtract common headers, if packet too small, don't copy anything
		byteCount = (sizeof(MAD_COMMON) + sizeof(RMPP_HEADER) + ClassHeaderSize);
		byteCount = (byteCount > pBufferHdr->ByteCount)? \
								0:(pBufferHdr->ByteCount - byteCount);

		byteCount = (totalBytesToCopy > byteCount)?byteCount:totalBytesToCopy;
		if (byteCount)
		{
			MemoryCopy( pBuffer, &pMadRmpp->Data[ClassHeaderSize], byteCount); 
			pBuffer += byteCount;
			totalBytesToCopy -= byteCount; // keep count
		}
	}
	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
	return messageSize;

fail:
	if (*ppBuffer)
	{
		MemoryDeallocate(*ppBuffer);
		*ppBuffer = NULL;
	}
	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
	return messageSize;
}
