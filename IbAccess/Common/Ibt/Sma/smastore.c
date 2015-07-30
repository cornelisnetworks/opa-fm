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

#include "smamain.h"




//
// RegisterMemGlobal
//
//	Registers a given virtual address with all known Channel Adapters
//	to the SMA
//
// Note: If we tweak the PdHandle to be unique for each client
//		 we can expose this function to clients.
//
//
//
// INPUTS:
//
//	VirtualAddress	-Address to register
//
//	Length			-amount of bytes to register in Length.
//
//	AccessControl	-Type of memory access.
//
//
//
// OUTPUTS:
//
//	CaMemIndex		-Global memory index that uniquely identifies this memory
//					 on all Channel Adapters
//
//
// RETURNS:
//
//
//
// caller cannot hold g_Sma->Bin.Lock, but may hold Mutex

FSTATUS
RegisterMemGlobal(
	IN  void				*VirtualAddress,
	IN  size_t				Length,
	IN  IB_ACCESS_CONTROL	AccessControl,
	OUT uint32				*CaMemIndex
	 )
{
	FSTATUS					status = FSUCCESS;
	SMA_CA_OBJ_PRIVATE		*pCaObj;
	CA_MEM_LIST				caMemList;
	boolean					haveIndex = FALSE;


	_DBG_ENTER_LVL(_DBG_LVL_STORE, RegisterMemGlobal);


	// Register memory with all CA's

	SpinLockAcquire(&g_Sma->CaListLock);
	for (pCaObj = g_Sma->CaObj; pCaObj; pCaObj = pCaObj->Next)
	{
		pCaObj->UseCount++;
		SpinLockRelease(&g_Sma->CaListLock);

		if (! haveIndex)
		{
			// get an index for memory
			SmaCreateCaMemIndex( pCaObj, CaMemIndex );
			haveIndex = TRUE;
		}

		status = iba_register_mr( pCaObj->CaHandle, VirtualAddress, Length,
						pCaObj->PdHandle, AccessControl, &caMemList.MrHandle,
						&caMemList.LKey, &caMemList.RKey);

		// Check for return values
		if ( FSUCCESS != status )
		{
			_DBG_ERROR(("Could not register Memory!\n"));
			goto failmr;
		}

		status = SmaAddToCaMemTable( pCaObj, &caMemList, *CaMemIndex);
		if ( FSUCCESS != status )
		{
			_DBG_ERROR(("Could not add to Ca Mem Table!\n"));
			goto failtbl;
		}

		SpinLockAcquire(&g_Sma->CaListLock);
		pCaObj->UseCount--;

	}	// for I loop
	SpinLockRelease(&g_Sma->CaListLock);

	if (! haveIndex)
	{
		// get an index for memory
		SmaCreateCaMemIndex( NULL, CaMemIndex );
	}

done:
	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
	return status;

failtbl:
	(void)iba_deregister_mr(caMemList.MrHandle);

failmr:
	SpinLockAcquire(&g_Sma->CaListLock);
	pCaObj->UseCount--;
	// unwind other CA's
	while (NULL != (pCaObj = pCaObj->Prev))
	{
		pCaObj->UseCount++;
		SpinLockRelease(&g_Sma->CaListLock);

		GetMemInfoFromCaMemTable( pCaObj, *CaMemIndex, &caMemList);

		(void)iba_deregister_mr(caMemList.MrHandle);

		DeleteFromCaMemTable( pCaObj, *CaMemIndex);

		SpinLockAcquire(&g_Sma->CaListLock);
		pCaObj->UseCount--;
	}
	SpinLockRelease(&g_Sma->CaListLock);

	SmaDeleteCaMemIndex( *CaMemIndex );

	goto done;
}


//
// DeregisterMemGlobal
//
//	DeRegisters a given virtual address with all known Channel Adapters
//	to the SMA. The address is defined by the Index it references.
//
//
//
// INPUTS:
//
//	CaMemIndex		-Global memory index that uniquely identifies this memory
//					 on all Channel Adapters
//
//
//
// OUTPUTS:
//
//
//
// RETURNS:
//
//
//

FSTATUS
DeregisterMemGlobal(
	IN	uint32				CaMemIndex
	 )
{
	FSTATUS					status = FSUCCESS;
	SMA_CA_OBJ_PRIVATE		*pCaObj;
	CA_MEM_LIST				caMemList = { 0 };	/* keep compiler happy */


	_DBG_ENTER_LVL(_DBG_LVL_STORE, DeregisterMemGlobal);


	// DeRegister memory with all CA's


	// Loop through all CAs
	SpinLockAcquire(&g_Sma->CaListLock);
	for (pCaObj = g_Sma->CaObj; pCaObj; pCaObj = pCaObj->Next)
	{
		pCaObj->UseCount++;
		SpinLockRelease(&g_Sma->CaListLock);

		GetMemInfoFromCaMemTable( pCaObj, CaMemIndex, &caMemList);

		iba_deregister_mr(caMemList.MrHandle);

		DeleteFromCaMemTable( pCaObj, CaMemIndex);

		SpinLockAcquire(&g_Sma->CaListLock);
		pCaObj->UseCount--;
	}
	SpinLockRelease(&g_Sma->CaListLock);

	SmaDeleteCaMemIndex( CaMemIndex );

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
	return status;

}


void
AcquireMemListMutex(void)
{
	MutexAcquire( &g_Sma->Bin.Mutex );
}

void
ReleaseMemListMutex(void)
{
	MutexRelease( &g_Sma->Bin.Mutex );
}

//
// CreateGlobalMemList
//
//	Registers a given virtual address with all known Channel Adapters
//	to the SMA.
//
// Note: If we tweak the PdHandle to be unique for each client
//		 we can expose this function to clients.
//
//
// INPUTS:
//
//	VirtualAddress	-Address to register
//
//	Length			-amount of bytes to register in Length.
//
//	HeaderAddress	-Address to block headers that hold registered mem
//
//	HoldingMutex		-is caller already holding g_Sma->Bin.Mutex (eg. MemListMutex)
//
//
//
// OUTPUTS:
//
//	GlobalMemList	-Global memory List that identifies memory across CA's
//
//
// RETURNS:
//
//
//

GLOBAL_MEM_LIST *
CreateGlobalMemList(
	void				*VirtualAddr,		
	uintn				Length,
	void				*HdrAddr,
	boolean				HoldingMutex
	)
{
	GLOBAL_MEM_LIST		*pMemList;

	//
	// Create global memory block in memory list
	//

	pMemList = (GLOBAL_MEM_LIST*)MemoryAllocateAndClear(sizeof(GLOBAL_MEM_LIST), FALSE, SMA_MEM_TAG);
	if ( NULL != pMemList )
	{
		pMemList->VirtualAddr = VirtualAddr;
		pMemList->Length = Length;
		pMemList->HdrAddr = HdrAddr;

		pMemList->Prev = NULL;

		if (! HoldingMutex)
			MutexAcquire( &g_Sma->Bin.Mutex );
		pMemList->Next = g_Sma->Bin.MemList;
		if (g_Sma->Bin.MemList)
			g_Sma->Bin.MemList->Prev = pMemList;
		g_Sma->Bin.MemList = pMemList;
		if (! HoldingMutex)
			MutexRelease( &g_Sma->Bin.Mutex );
	}	// pMemList

	return pMemList;
}

//	caller must not be holding g_Sma->Bin.Mutex
FSTATUS
DeleteGlobalMemList(
	GLOBAL_MEM_LIST				*MemList		
	)
{
	FSTATUS						status=FSUCCESS;

	MutexAcquire( &g_Sma->Bin.Mutex );

	//
	// Delete global memory block in memory list
	//

	//
	// delink from list first
	//

	if ( NULL != MemList->Prev )
	{
		((GLOBAL_MEM_LIST *)MemList->Prev)->Next = MemList->Next;
	}
	
	if ( NULL != MemList->Next )
	{
		((GLOBAL_MEM_LIST *)MemList->Next)->Prev = MemList->Prev;
	}


	//
	// fix head of list if needed
	//

	if ( MemList == g_Sma->Bin.MemList )
	{
		g_Sma->Bin.MemList = (GLOBAL_MEM_LIST*)MemList->Next;
	}

	MutexRelease( &g_Sma->Bin.Mutex );
	
	//
	// Free this block of mem
	//

	MemoryDeallocate( MemList );

	return status;
}



//
// SmaGrabFromBin
//
//	Grabs a specified number of MADs from pool.
//	If the specified number of MADs are not available in pool, it is made
//	available by more allocations.
//
//
//
// INPUTS:
//
//	NumMad				-Number of buffers requested in MAD sizes 
//
//	MadPool				-Pointer to hold list of alloacted MAD buffers
//
//
// OUTPUTS:
//
//	NumMad				-Number of buffers allocated
//
//	MadPool				-Pointer holds list of alloacted MAD buffers
//
//
// RETURNS:
//
//
//

#if 0 // not yet implemented, if'def out so get errors if anyone tried to use
FSTATUS
SmaGrabFromBin(
	IN OUT	uint32		*NumMad,
	IN OUT	SMP_BLOCK	*MadPool
	)
{
	FSTATUS				status = FSUCCESS;
	
	_DBG_ENTER_LVL(_DBG_LVL_STORE, SmaGrabFromBin);
	

	SpinLockAcquire( &g_Sma->Bin.Lock );

	SpinLockRelease( &g_Sma->Bin.Lock );

	
	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
	return status;
}
#endif





//
// SmaReturnToBin
//
//	Grabs a specified number of MADs from pool.
//	If the specified number of MADs are not available in pool, it is made
//	available by more allocations.
//
//
//
// INPUTS:
//
//	NumMad				-Number of buffers to free (in MAD sizes) 
//
//	MadPool				-Pointer to list of MAD buffers to free
//
//
// OUTPUTS:
//
//
// RETURNS:
//
//
//
//

FSTATUS
SmaReturnToBin(
	IN	POOL_DATA_BLOCK		*SmpBlockList
	)
{
	FSTATUS					status = FSUCCESS;
	POOL_DATA_BLOCK			*pMem, *pMemNext;
	uint32					NumSmps;

	_DBG_ENTER_LVL(_DBG_LVL_STORE, SmaReturnToBin);
	

	//
	// lock
	//
	
	SpinLockAcquire( &g_Sma->Bin.Lock );


	//
	// return to bin
	//

	pMemNext = g_Sma->Bin.Tail;					// get system list
	pMem = SmpBlockList;						// load user list

	_DBG_PRINT(_DBG_LVL_STORE,(
		"(FREE)pMem(x%p) Next(x%p) Prev(x%p)\n",
		_DBG_PTR(pMem), _DBG_PTR(pMem->Block.Next), _DBG_PTR(pMem->Block.Prev)));

	
	//
	// add to system list
	//

	pMem->Block.Prev = pMemNext;
	if ( NULL == pMemNext )
	{
		g_Sma->Bin.Head = pMem;
	}
	else
	{
		_DBG_PRINT(_DBG_LVL_STORE,(
			"(TAIL)pMemNext(x%p) Next(x%p) Prev(x%p)\n",
			_DBG_PTR(pMemNext), _DBG_PTR(pMemNext->Block.Next), _DBG_PTR(pMemNext->Block.Prev)));


		pMemNext->Block.Next = pMem;
	}


	//
	// count the SMPs returned
	//

	NumSmps = 1;								// start from here
	pMem->Block.PathBits = 0;
	pMem->Block.ServiceLevel = 0;
	pMem->Block.StaticRate = 0;
	pMem->Block.pSendContext = (void *)-1;    //reset the sendcontext

	while ( NULL != pMem->Block.Next )
	{
		NumSmps++;								// keep counting
		pMem = (POOL_DATA_BLOCK*)pMem->Block.Next;						// loop

		pMem->Block.PathBits = 0;
		pMem->Block.ServiceLevel = 0;
		pMem->Block.StaticRate = 0;
		pMem->Block.pSendContext = (void *)-1;    //reset the sendcontext
	}


	//
	// update system list
	//

	g_Sma->Bin.Tail = pMem;
	g_Sma->Bin.NumBlocks += NumSmps;

	_DBG_PRINT(_DBG_LVL_STORE,(
		"x%x Free Smps in POOL\n", g_Sma->Bin.NumBlocks));

	//
	// Do house keeping
	// TBD: Do we need to free memory not used?
	//

	
	//
	// unlock
	//

	SpinLockRelease( &g_Sma->Bin.Lock );

	
	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
	return status;

}




//
// SmaAllocToBin
//
//	Creates new entries in MAD pool. The number of new entries is specified
//	as input.
//	The functions adds the an average of the total number of MADs specified.
//	The average is derived from a stored value that may be set at install
//	time.
//
//	In addition to the MAD buffer that is created in pool, a header is also
//	allocated that will hold the transport header like the GRH and the 
//	data transfer values for send/recevies.
//
// Note:
//	Lock taken by caller
//
//
//
// INPUTS:
//
//	NumMads				-Number of MADs buffers to create(in MAD sizes) 
//
//	HoldingMutex			-is caller already holding g_Sma->Bin.Mutex
//
//  Note: Caller must not be holding g_Sma->Bin.Lock
//
// OUTPUTS:
//
//
// RETURNS:
//
//
//
//

FSTATUS
SmaAllocToBin(
	IN uint32			NumMads,
	IN boolean			HoldingMutex
	)
{
	FSTATUS				status = FSUCCESS;
	UD_TOTAL_BLOCK		*pUdBlock=NULL;
	POOL_DATA_BLOCK		*pSmpBlock=NULL;
	GLOBAL_MEM_LIST		*pMemList=NULL;

	POOL_DATA_BLOCK		*pSmpBlockCurr;
	POOL_DATA_BLOCK		*pSmpBlockPrev, *pSmpBlockNext;
	uint32				i, totalSmps;
	
	_DBG_ENTER_LVL(_DBG_LVL_STORE, SmaAllocToBin);
	

	//
	// lookup threshold level for smps in bin
	//
	
	totalSmps = ( NumMads <= g_Settings.MinSMPsPerAllocate ? \
					g_Settings.MinSMPsPerAllocate: \
				 ( NumMads + g_Settings.MinSMPsPerAllocate - \
				   ( NumMads % g_Settings.MinSMPsPerAllocate )));

	if (! HoldingMutex)
		MutexAcquire( &g_Sma->Bin.Mutex );

	if ( g_Settings.MaxSMPs < g_Sma->Bin.TotalAllocated + totalSmps )
	{
		_DBG_ERROR((
			"MaxSMP allocation threshold (x%x) level reached\n",
			g_Settings.MaxSMPs));

		totalSmps = g_Settings.MaxSMPs - g_Sma->Bin.TotalAllocated;
	}

	
	//
	// Allocate memory for SMPs
	//

	//
	// BUGBUG: We should allocate paged mem here for Win2K and lock it
	//
	
	if ( totalSmps )
	{
		pUdBlock = (UD_TOTAL_BLOCK*)MemoryAllocateAndClear(
						 totalSmps * sizeof(MAD), FALSE, SMA_MEM_TAG);
	}

	if ( NULL != pUdBlock )
	{

		//
		// alloc memory for POOL_DATA_BLOCKs
		//

		pSmpBlock = (POOL_DATA_BLOCK*)MemoryAllocateAndClear(
					totalSmps * sizeof(POOL_DATA_BLOCK), FALSE, SMA_MEM_TAG);
		if ( NULL != pSmpBlock )
		{

			//
			// Create global memory block in memory list
			//

			pMemList = CreateGlobalMemList( 
									pUdBlock, 
									( totalSmps * sizeof(MAD)),
									pSmpBlock,
									TRUE );
		}		// pSmpBlock
	}			// pUdBlock

	if ( ( NULL != pUdBlock ) && \
		 ( NULL != pSmpBlock ) && \
		 ( NULL != pMemList ) )
	{
		_DBG_PRINT(_DBG_LVL_STORE,
			("Total : requested(x%x), allocated(x%x)\n", \
			NumMads, totalSmps));
		_DBG_PRINT(_DBG_LVL_STORE,
			("Memory: pUdBlock(x%p), pSmpBlock(x%p)\n", \
			_DBG_PTR(pUdBlock), _DBG_PTR(pSmpBlock)));

		//
		// set access level for memory region
		//

		pMemList->AccessControl.AsUINT16 = 0;
		pMemList->AccessControl.s.LocalWrite = 1;

		status = RegisterMemGlobal(
								pMemList->VirtualAddr,
								pMemList->Length,
								pMemList->AccessControl,
								&pMemList->CaMemIndex
								);

		if ( FSUCCESS == status )
		{

			//
			// get pointers
			//

			pSmpBlockCurr = pSmpBlock;
			pSmpBlockPrev = NULL;	// fill in g_Sma->Bin.Tail at end of loop
			

			//
			// update next value
			//

			pSmpBlockNext = pSmpBlockCurr;
			pSmpBlockNext++;

			for ( i=0; i< totalSmps; i++ )
			{

				//
				// update current pointer values
				//

				pSmpBlockCurr->Block.Smp = &pUdBlock->Mad;
				//pSmpBlockCurr->Block.Grh = &pUdBlock->Grh;
				//pSmpBlockCurr->Block.Lrh = &pUdBlock->Lrh;

				pSmpBlockCurr->Block.Prev = pSmpBlockPrev;
				pSmpBlockCurr->Block.Next = pSmpBlockNext;


				//
				// Add Index for mem keys
				//

				pSmpBlockCurr->CaMemIndex = pMemList->CaMemIndex;
		

				//
				// increment pointers
				//

				pSmpBlockPrev = pSmpBlockCurr;
				pSmpBlockCurr = pSmpBlockNext;
				pSmpBlockNext++;
				pUdBlock++;

			}	// i loop


			//
			// Adjust and add to bin
			//

			SpinLockAcquire( &g_Sma->Bin.Lock );

			pSmpBlock->Block.Prev = g_Sma->Bin.Tail;	// first of new
			pSmpBlockPrev->Block.Next = NULL;			// last of new

			if ( NULL == g_Sma->Bin.Head )
			{
				g_Sma->Bin.Head = pSmpBlock;
			}
			else
			{
				g_Sma->Bin.Tail->Block.Next = pSmpBlock;
			}

			g_Sma->Bin.Tail = pSmpBlockPrev;
			g_Sma->Bin.NumBlocks  += totalSmps;
			SpinLockRelease( &g_Sma->Bin.Lock );

			g_Sma->Bin.TotalAllocated += totalSmps;
		}
	}
	else
	{
		
		//
		// errors in allocations.
		//

		_DBG_ERROR((
			"Could not allocate memory for x%x SMPs; tried x%x\n\n", \
			NumMads, totalSmps));
		
		//
		// return module specific errors
		//

		status = FINSUFFICIENT_MEMORY;
	}

	if (status != FSUCCESS)
	{

		//
		// unwind
		//

		if ( NULL != pMemList )
		{
			g_Sma->Bin.MemList = (GLOBAL_MEM_LIST*)pMemList->Next;
			MemoryDeallocate( pMemList );
		}
		if ( NULL != pSmpBlock )
			MemoryDeallocate( pSmpBlock );
		if ( NULL != pUdBlock )
			MemoryDeallocate( pUdBlock );

	}
	
	if (! HoldingMutex)
		MutexRelease( &g_Sma->Bin.Mutex );
	
	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
	return status;

}




//
// SmaRemoveFromBin
//
// Note:
//	Lock taken by caller
//
//
//
// INPUTS:
//
//
//
// OUTPUTS:
//
//
// RETURNS:
//
//
//
//

#if 0 // not yet implemented, if'def out so get errors if anyone tried to use
FSTATUS
SmaRemoveFromBin(void)
{
	FSTATUS				status = FSUCCESS;
	
	_DBG_ENTER_LVL(_DBG_LVL_STORE, SmaRemoveFromBin);
	

	//
	// SpinLockDestroy( g_Sma->Lock);
	//

	
	
	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
	return status;

}
#endif




//
// SmaCreateCaMemIndex
//
//	Creates new entries in MAD pool. The number of new entries is specified
//	as input.
//	The functions adds the an average of the total number of MADs specified.
//	The average is derived from a stored value that may be set at install
//	time.
//
//	In addition to the MAD buffer that is created in pool, a header is also
//	allocated that will hold the transport header like the GRH and the 
//	data transfer values for send/recevies.
//
// Note:
//	Lock taken by caller
//
//
//
// INPUTS:
//
//	NumMads				-Number of MADs buffers to create(in MAD sizes) 
//
//
// OUTPUTS:
//
//
// RETURNS:
//
//
//
//
//
// Memory functions to Store, Get and delete registered memory info
//
// caller cannot hold g_Sma->Bin.Lock
void
SmaCreateCaMemIndex(
	IN	SMA_CA_OBJ_PRIVATE	*CaObj,
	OUT	uint32				*CaMemIndex
	)
{	
	CA_MEM_TABLE			*pCaMemTbl;


	_DBG_ENTER_LVL(_DBG_LVL_STORE, SmaCreateCaMemIndex);
	
	if (CaObj)
		pCaMemTbl = CaObj->CaMemTbl;
	else
		pCaMemTbl = NULL;


	SpinLockAcquire( &g_Sma->Bin.Lock );

	//
	// Allocate Index from global entry
	//

	*CaMemIndex = g_Sma->Bin.CurrentIndex++;
	
	//_DBG_PRINT(_DBG_LVL_STORE,("MemIndex = x%x\n",*CaMemIndex));

	//
	// Is an Index available in table ?
	//

	if ( NULL !=	pCaMemTbl )
	{
		
		//
		// If index is from the middle, scan for new index
		//

		while (0 != pCaMemTbl->MemBlock[g_Sma->Bin.CurrentIndex].LKey)
		{
			g_Sma->Bin.CurrentIndex++;

			//
			// TBD:need logic here to scan from start
			//

			//
			// if the index is greater than what it can hold, a table 
			// will subsiquently be created
			//
			// if we have reached out, break
			//

			if ( g_Sma->Bin.CurrentIndex > pCaMemTbl->Total )	
				break;
		}
	}
	SpinLockRelease( &g_Sma->Bin.Lock );

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
}

// caller cannot hold g_Sma->Bin.Lock
void
SmaDeleteCaMemIndex( 
	IN	uint32					CaMemIndex 
	)
{
	SpinLockAcquire( &g_Sma->Bin.Lock );

	if ( g_Sma->Bin.CurrentIndex > CaMemIndex )
		g_Sma->Bin.CurrentIndex = CaMemIndex;	

	SpinLockRelease( &g_Sma->Bin.Lock );
}


FSTATUS
SmaAddToCaMemTable(
	IN	SMA_CA_OBJ_PRIVATE	*CaObj,
	IN	CA_MEM_LIST			*CaMemList,
	IN	uint32				CaMemIndex
	)
{	
	FSTATUS					status = FSUCCESS;
	CA_MEM_TABLE			*pCaMemTbl, *pNewCaMemTbl;
	uint32					memSize;
	boolean					newTable=FALSE;


	_DBG_ENTER_LVL(_DBG_LVL_STORE, SmaAddToCaMemTable);
	
	pCaMemTbl = CaObj->CaMemTbl;

	_DBG_PRINT(_DBG_LVL_STORE,(\
		"Index = x%x \n",
		CaMemIndex));

	//
	// do we need a new table ?
	//

	if (( NULL != pCaMemTbl ) && ( CaMemIndex > pCaMemTbl->Total ))
		newTable = TRUE;


	//
	// Allocate table if it does not exist or if too small
	//

	if (( NULL == pCaMemTbl ) || ( TRUE == newTable ))
	{

		//
		// calculate amount of memory needed
		//

		memSize = g_Settings.MemTableSize;
		if (TRUE == newTable)
		{
			memSize = (CaMemIndex + 1) * sizeof(CA_MEM_BLOCK);

			//
			// adjust in page sizes set in global settings
			//

			memSize += ((CaMemIndex + 1) * sizeof(CA_MEM_BLOCK) % \
				g_Settings.MemTableSize );
		}

		pNewCaMemTbl = (CA_MEM_TABLE*)MemoryAllocateAndClear(memSize, FALSE, SMA_MEM_TAG);
		if ( NULL != pNewCaMemTbl )
		{
			if (TRUE == newTable)
			{

				//
				// copy old entries
				//

				MemoryCopy(
					&pNewCaMemTbl->MemBlock[0], 
					&pCaMemTbl->MemBlock[0], 
					pCaMemTbl->Total*sizeof(CA_MEM_BLOCK) 
					);
				

				//
				// free old table
				//

				MemoryDeallocate( pCaMemTbl );
			}


			//
			// write down total entries
			//

			pNewCaMemTbl->Total = (memSize)/sizeof(CA_MEM_BLOCK) - 1;

			
			//
			// reassign pointers
			//

			pCaMemTbl = pNewCaMemTbl;		
			CaObj->CaMemTbl = pCaMemTbl;
		}
		else
		{
			status = FERROR;
		}
	}
	
	if ( FSUCCESS == status )
	{

		//
		// ASSERT on out of table experience
		//

		pCaMemTbl->MemBlock[CaMemIndex].MrHandle = CaMemList->MrHandle;
		pCaMemTbl->MemBlock[CaMemIndex].LKey = CaMemList->LKey;
		//pCaMemTbl->MemBlock[CaMemIndex].RKey = CaMemList->RKey;
	}

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
	return status;
}




FSTATUS
GetMemInfoFromCaMemTable(
	IN	SMA_CA_OBJ_PRIVATE	*CaObj,
	IN	uint32				CaMemIndex,
	OUT	CA_MEM_LIST			*CaMemList
	)
{	
	FSTATUS					status = FSUCCESS;
	CA_MEM_TABLE			*pCaMemTbl;


	_DBG_ENTER_LVL(_DBG_LVL_STORE, GetMemInfoFromCaMemTable);
	
	pCaMemTbl = CaObj->CaMemTbl;

	_DBG_PRINT(_DBG_LVL_STORE,(\
		"Index = x%x\n",
		CaMemIndex));

	//
	// Allocate table if it does not exist
	//

	if ( NULL !=	pCaMemTbl )
	{
		CaMemList->MrHandle = pCaMemTbl->MemBlock[CaMemIndex].MrHandle;
		CaMemList->LKey = pCaMemTbl->MemBlock[CaMemIndex].LKey;
//		CaMemList->RKey = pCaMemTbl->MemBlock[CaMemIndex].RKey;
	}
	else
	{
		status = FERROR;
	}

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
	return status;
}

FSTATUS
DeleteFromCaMemTable(
	IN	SMA_CA_OBJ_PRIVATE	*CaObj,
	IN	uint32				CaMemIndex
   )
{	
	FSTATUS					status = FSUCCESS;
	CA_MEM_TABLE			*pCaMemTbl;


	_DBG_ENTER_LVL(_DBG_LVL_STORE, DeleteFromCaMemTable);
	
	pCaMemTbl = CaObj->CaMemTbl;

	//
	// Allocate table if it does not exist
	//
	if ( NULL !=	pCaMemTbl )
	{
		pCaMemTbl->MemBlock[CaMemIndex].MrHandle = NULL;
		pCaMemTbl->MemBlock[CaMemIndex].LKey = 0;
//		pCaMemTbl->MemBlock[CaMemIndex].RKey = 0;
	}
	else
	{
		status = FERROR;
	}

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
	return status;
}


//
// Register any registered memory with this CA
// caller cannot hold g_Sma->Bin.Lock
// caller must hold g_Sma->Bin.Mutex
FSTATUS
SmaBindGlobalMem(
	IN	SMA_CA_OBJ_PRIVATE	*CaObj
	)
{
	FSTATUS					status = FSUCCESS;
	GLOBAL_MEM_LIST			*pMemList;
	CA_MEM_LIST				caMemList;


	_DBG_ENTER_LVL(_DBG_LVL_STORE, SmaBindGlobalMem);

	pMemList = g_Sma->Bin.MemList;

	while ( NULL != pMemList )
	{

		status = iba_register_mr( CaObj->CaHandle, pMemList->VirtualAddr,
						pMemList->Length, CaObj->PdHandle,
						pMemList->AccessControl, &caMemList.MrHandle,
						&caMemList.LKey, &caMemList.RKey); 

		// Check returns
		if ( FSUCCESS != status )
		{
			_DBG_ERROR(("Could not register Memory x%p!\n", 
						_DBG_PTR(pMemList->VirtualAddr)));

			//
			// unwind
			//

			while ( NULL != pMemList->Prev )
			{
				pMemList = (GLOBAL_MEM_LIST*)pMemList->Prev;

				GetMemInfoFromCaMemTable(
						CaObj,
						pMemList->CaMemIndex,
						&caMemList
						);

				iba_deregister_mr(caMemList.MrHandle);

			}


			//
			// remove the memlink
			//

			if ( NULL != CaObj->CaMemTbl )
				MemoryDeallocate( CaObj->CaMemTbl );

			break;
		}
		else
		{
			status = SmaAddToCaMemTable(
							CaObj,
							&caMemList,
							pMemList->CaMemIndex
							);
		}


		//
		// loop thru' all lists
		//

		pMemList = (GLOBAL_MEM_LIST*)pMemList->Next;

	}	// while loop

	_DBG_LEAVE_LVL(_DBG_LVL_STORE);
	return status;

}
