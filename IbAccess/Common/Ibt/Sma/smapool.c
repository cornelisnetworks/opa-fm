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

#include "smamain.h"


//
// Forward declarations
//
	



//
// Common buffer calls for Smp's on requested Hca
//
//
//
//	Usage: 
//
//	The SMA allocates all SMP buffers for sends and receives. Since these 
//	buffers are of a constant size (MAD size), allocation size is known at 
//	all times and hence not specified in calls.
//
//	The following rules apply for all SMP buffer usage:
//		* Cut down the requirements on system memory is reduced by half as 
//		  there is no copy administered between Sends and Receives. The same 
//		  SMP buffers are passed to/from the hardware. All SMP buffers come 
//		  from registered memory.
//		* The buffers are not tied down to a particular Channel Adapter. Hence 
//		  they can be sent to or received from any CA. The memory registration 
//		  is taken care of by the SMA.
//		* The buffers are not required to be maintained by the Subnet Manager. 
//		  All tracking is done by the SMA. 
//		* Once a buffer is submitted by a SmaPost or SmaSetLocal call to the 
//		  SMA, it will be not returned back.
//		* There is no rule maintained as to the order of these buffers. The 
//		  Subnet Manager cannot except the same buffers to be passed in 
//		  by the SMA.
//		* All buffers allocated and returned through any send/receive calls 
//		  hold the same size and is equal to the size of a MAD structure of 
//		  256 bytes.
//		* A buffer once allocated from pool can be rturned back to pool by 
//		  SmaPost() and SmaReturnToPool().
//
//		


//
// SmaGrabFromPool
//
//	The caller
//
//
// INPUTS:
//
//	SmObject		-Object returned in the Open call.
//
//	NumSmps			-Number of blocks of memory that the caller wants in 
//					 SMP sizes.
//
//	SmpPool			-An array to hold the SMPs requested.
//
//
// OUTPUTS:
//
//	SmpPool			-Array filled with SMPs with total number allocated.
//
//
// RETURNS:
//
//
//
FSTATUS
iba_smi_pool_get(
	IN	SMA_OBJECT		*SmObject,
	IN OUT	uint32		*NumSmps,		
	OUT SMP_BLOCK		**SmpBlockList
	)
{
	FSTATUS				status = FSUCCESS;
	POOL_DATA_BLOCK		*pMem, *pMemNext;
	uint32				i;
	
	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_smi_pool_get);

	//
	// Do param checks
	//
	if ( !*NumSmps )
	{
		_DBG_LEAVE_LVL(_DBG_LVL_POOL);
		return FERROR;
		//*NumSmps = 1;		// set minimal 1 to make SMPT work!!
	}
	
	//
	// look up bin
	//

	//
	// lock
	//

	SpinLockAcquire( &g_Sma->Bin.Lock );


	//
	// If we cannot satisfy request, try allocating from system 
	//


	if ( g_Sma->Bin.NumBlocks < *NumSmps )
	{
		// this is slightly complex however it avoids the need to get a
		// semaphore for non-growth allocates
		// the heirarchy requires we have the semaphore before the Lock
		// and we need the lock to test NumBlocks
		// we loop in case between Alloc and Lock someone takes all
		// our new buffers (unlikely).  Ultimately if the Alloc fails
		// we must bail out
		SpinLockRelease( &g_Sma->Bin.Lock );
		MutexAcquire( &g_Sma->Bin.Mutex );
		SpinLockAcquire( &g_Sma->Bin.Lock );
		while ( FSUCCESS == status && g_Sma->Bin.NumBlocks < *NumSmps )
		{
			SpinLockRelease( &g_Sma->Bin.Lock );
			status = SmaAllocToBin( *NumSmps, TRUE );
			SpinLockAcquire( &g_Sma->Bin.Lock );
		}
		MutexRelease( &g_Sma->Bin.Mutex );
	}

	// now have lock, don't have sema

	if ( FSUCCESS == status )
	{
		//
		// else return from bin
		//

		pMem = g_Sma->Bin.Head;


		//
		// more than NumSmps in bin
		//

		if ( g_Sma->Bin.NumBlocks > *NumSmps )
		{
			pMemNext = pMem;
				
			for ( i=0; i< *NumSmps; i++ )
			{
				pMemNext = (POOL_DATA_BLOCK*)pMemNext->Block.Next;
			}


			//
			// update links
			//

			//
			// is it first link?
			//

			ASSERT ((POOL_DATA_BLOCK *)pMemNext->Block.Prev);

			//
			// No?, then break link and update ends
			//

			((POOL_DATA_BLOCK *)(pMemNext->Block.Prev))->Block.Next = NULL;
			pMemNext->Block.Prev = NULL;		// start of bin
			

			//
			// update bin
			//

			g_Sma->Bin.Head = pMemNext;
			g_Sma->Bin.NumBlocks -= *NumSmps;

		}
		else
		{
			//
			// <= Smps in bin
			//

			*NumSmps = g_Sma->Bin.NumBlocks;


			//
			// update bin
			//

			g_Sma->Bin.Head = g_Sma->Bin.Tail = NULL;
			g_Sma->Bin.NumBlocks = 0;
		}

		if ( 0 != *NumSmps )
		{
			SMP_BLOCK *SmpBlock = (SMP_BLOCK *)pMem;

			for ( i=0; i< *NumSmps; i++ )
			{
				SmpBlock->SmpByteCount = 0;
				SmpBlock->RecvByteCount = 0;
				SmpBlock = SmpBlock->Next;
			}

			*SmpBlockList = (SMP_BLOCK *)pMem;

			_DBG_PRINT(_DBG_LVL_POOL,(
				"pMem(x%p) Next(x%p) Prev(x%p)\n",
				_DBG_PTR(pMem), _DBG_PTR(pMem->Block.Next),
				_DBG_PTR(pMem->Block.Prev)));

		}
	}	// status

	SpinLockRelease( &g_Sma->Bin.Lock );

	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
    
	return status;
}



//
// SmaReturnToPool
//
//	The caller
//
//
// INPUTS:
//
//	SmObject		-Object returned in the Open call.
//
//	SmpPool			-An array of SMPs to be freed.
//
//
// OUTPUTS:
//
//
// RETURNS:
//
//
//

FSTATUS
iba_smi_pool_put(
	IN	SMA_OBJECT		*SmObject,
	IN	SMP_BLOCK		*SmpBlockList
	)
{
	FSTATUS				status=FSUCCESS;
	
	_DBG_ENTER_LVL(_DBG_LVL_POOL, iba_smi_pool_put);
	

	//
	// Do param checks
	//


	SmaReturnToBin((POOL_DATA_BLOCK *)SmpBlockList);
	
	_DBG_LEAVE_LVL(_DBG_LVL_POOL);
    
	return status;
}
