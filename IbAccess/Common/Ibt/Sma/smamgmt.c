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


extern IB_DBG_PARAMS	Gsa_debug_params;


//
// SmaRemoveCaObj
//
//	This routine frees resources allocated to a CaObject. 
//
// INPUTS:
//
//	Pointer to CaObj
//
// OUTPUTS:
//
//	None.
//
// RETURNS:
//
//	FSUCCESS - Successful freeing of Ca resources.
//
// NOTES:
//
//	This routine can be called from SmaRemoveDevice() or SmaUnload()
//  called with pCaObj UseCount held
//
FSTATUS
SmaRemoveCaObj(
	IN	SMA_CA_OBJ_PRIVATE		*pCaObj
	)
{
	FSTATUS						status = FSUCCESS;
	CA_MEM_TABLE				*caMemTbl;
	SMA_PORT_TABLE_PRIV			*pPortTbl;
	uint32						i;
	IB_QP_ATTRIBUTES_MODIFY		qpModAttributes;
	
	_DBG_ENTER_LVL(_DBG_LVL_MGMT, SmaRemoveCaObj);

	pPortTbl = (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;

	// Before destorying the Qps first flush them
	for ( i=0; i<pCaObj->CaPublic.Ports; i++ )
	{		
		// Before destroying the QP, set it to error state
		// This should flush all pending completions in error
		// Also, set internal state to destroy so that we
		// do not post recv buffers
		pPortTbl->PortBlock[i].Qp0Control = QP_DESTROY;

		qpModAttributes.RequestState = QPStateError;
		qpModAttributes.Attrs = 0;
		status =iba_modify_qp(pPortTbl->PortBlock[i].QpHandle,&qpModAttributes,NULL);
		if ( FSUCCESS != status )
		{
			_DBG_ERROR((
				"Could not ModifyQp (Qp0 on Port %d) to error state!!!\n",
				i+1));
		}

		pPortTbl->PortBlock[i].Qp1Control = QP_DESTROY;

		qpModAttributes.RequestState = QPStateError;
		qpModAttributes.Attrs = 0;
		status=iba_modify_qp(pPortTbl->PortBlock[i].Qp1Handle,&qpModAttributes,NULL);

		if ( FSUCCESS != status )
		{
			_DBG_ERROR((
				"Could not ModifyQp (Qp1 on Port %d) to error state!!!\n",
				i+1));
		}
	}

	// loop a few times to make sure all CQEs have come back
	for (i=0; i<10; ++i)
	{
		// Wait for any interrupts to be processed
		ThreadSuspend( 100 );

		// Forcebly poll for any pending completions
		SmaCompletionCallback( pCaObj );
		GsaCompletionCallback( pCaObj, pCaObj->Qp1CqHandle );
	}

	EventThreadStop(&pCaObj->GsaEventThread);
	EventThreadStop(&pCaObj->SmaEventThread);

	// update global info
	SpinLockAcquire(&g_Sma->CaListLock);
	if ( NULL != pCaObj->Prev )
		pCaObj->Prev->Next = pCaObj->Next;
	if ( NULL != pCaObj->Next )
		pCaObj->Next->Prev = pCaObj->Prev;
	if ( g_Sma->CaObj == pCaObj )
		g_Sma->CaObj = g_Sma->CaObj->Next;
	g_Sma->NumCa--;
	pCaObj->UseCount--;
	SpinLockRelease(&g_Sma->CaListLock);

	// wait for outstanding Uses to be done
	// pCaObj->Next is still valid
	while (1)
	{
		SpinLockAcquire(&g_Sma->CaListLock);
		if (0 == pCaObj->UseCount)
			break;
		SpinLockRelease(&g_Sma->CaListLock);
		ThreadSuspend(10);
	}
	SpinLockRelease(&g_Sma->CaListLock);

	// deregister all memory registered for this CA
	caMemTbl = pCaObj->CaMemTbl;
	if ( NULL != caMemTbl )
	{	
		for ( i=0; i < caMemTbl->Total; ++i )
		{
			if ( NULL != caMemTbl->MemBlock[i].MrHandle )
			{
				status = iba_deregister_mr( caMemTbl->MemBlock[i].MrHandle );

				if ( FSUCCESS != status )
				{
					_DBG_ERROR(("Could not Deregister Memory!\n"));
				}
			}
		}	// i loop
	}

	// remove mem list
	if ( pCaObj->CaMemTbl )
		MemoryDeallocate( pCaObj->CaMemTbl );

	// destroy Qps
	for ( i=0; i<pCaObj->CaPublic.Ports; i++ )
	{		

		iba_destroy_qp( pPortTbl->PortBlock[i].QpHandle );	// QP0
		iba_destroy_qp( pPortTbl->PortBlock[i].Qp1Handle );	// QP1
//		DestroyAV( pPortTbl->PortBlock[i].AvHandle );	// Permissive LID

		// Spinlock for Qp1
		SpinLockDestroy(&pPortTbl->PortBlock[i].QpLock);
		SpinLockDestroy(&pPortTbl->PortBlock[i].Qp0Lock);
	}

	// destroy Cqs
	iba_destroy_cq( pCaObj->CqHandle );
	iba_destroy_cq( pCaObj->Qp1CqHandle );

	//BUGBUG:
	// destroy AVs if in Table Cache

	// destroy PDs
	if ( pCaObj->PdHandle )
		iba_free_pd(pCaObj->PdHandle );

	// close Ca
	iba_close_ca( pCaObj->CaHandle );

	EventThreadDestroy(&pCaObj->GsaEventThread);
	EventThreadDestroy(&pCaObj->SmaEventThread);

	// remove sublists likePort info and tag other requirements
	
	// destroy Port tables
	MemoryDeallocate( pCaObj->CaPublic.PortTbl );


	// de-allocate memory for CA_OBJ
	MemoryDeallocate( pCaObj );

   _DBG_LEAVE_LVL(_DBG_LVL_MGMT);
    
   return status;
}



//
// SmaCreateSmaObj
//
//	This routine creates a new instance for a new user on SmaOpen()
//
// INPUTS:
//
//	Pointer to User Sma Object pointer
//
// OUTPUTS:
//
//	Allocated SmaObj for user
//
//
// RETURNS:
//
//	FSUCCESS - Successful freeing of Ca resources.
//
// NOTES:
//
//	This routine can be called from SmaOpen() and SmaUpdateInfo()
//
FSTATUS
SmaCreateSmaObj( 
	IN	OUT	SMA_OBJECT			*SmaObj
	)
{
	FSTATUS						status = FSUCCESS;
	SMA_CA_OBJ_PRIVATE			*pCaObj;
	size_t						i, j, sizeSmObj=0;
	SMA_OBJECT					*pSmaObj;
	SMA_CA_TABLE				*pCaTblUsr;
	SMA_PORT_TABLE				*pCaPortTbl;
	SMA_PORT_TABLE_PRIV			*pPortTbl;

	_DBG_ENTER_LVL(_DBG_LVL_MGMT, SmaCreateSmaObj);

	// walk caObj list and calculate memory required
	SpinLockAcquire(&g_Sma->CaListLock);
	if (g_Sma->NumCa)
	{
		for (pCaObj = g_Sma->CaObj; pCaObj; pCaObj = pCaObj->Next)
		{
			sizeSmObj += ( sizeof( SMA_CA_OBJECT ) + 
						   pCaObj->CaPublic.Ports * sizeof( SMA_PORT_BLOCK ));
		}

		// Map this to our internal type
		pSmaObj = SmaObj;
		
		// allocate memory
		pSmaObj->CaTbl = (SMA_CA_TABLE*)MemoryAllocateAndClear(sizeSmObj, FALSE, SMA_MEM_TAG);
		if ( NULL != pSmaObj->CaTbl )
		{
			// create pointers
			pCaTblUsr = pSmaObj->CaTbl;	
			pCaPortTbl = (SMA_PORT_TABLE *) &pSmaObj->CaTbl[g_Sma->NumCa];
			
			// Fill in info

			// SmaObj
			pSmaObj->NumCa = g_Sma->NumCa;
			
			// now the Ca's
			for (i=0, pCaObj = g_Sma->CaObj; pCaObj; i++, pCaObj = pCaObj->Next)
			{
				// Get private port table
				pPortTbl = (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;
			
				// CaInfo
				*pCaTblUsr[i].CaObj = pCaObj->CaPublic;

				// Port info
				pCaTblUsr[i].CaObj->PortTbl = pCaPortTbl;
				for ( j=0; j<pCaObj->CaPublic.Ports; j++ )
				{
					MemoryCopy(
						&pCaTblUsr[i].CaObj->PortTbl[j], 
						&pPortTbl[j].PortBlock->Public, 
						sizeof( pPortTbl[j].PortBlock->Public ));
				}

				// advance pointers
				pCaPortTbl = &pCaPortTbl[pCaObj->CaPublic.Ports];
			}
		} else {
			_DBG_ERROR(("Could not Allocate Memory for SmaObj!\n"));
			status = FERROR;
		}
	}	// NumCa
	SpinLockRelease(&g_Sma->CaListLock);

	
   _DBG_LEAVE_LVL(_DBG_LVL_MGMT);
   	return status;
}
