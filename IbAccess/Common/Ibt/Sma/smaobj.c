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
#include "imath.h"


//
// Forward declarations
//




//
// SmaOpen
//
//	Returns a pointer to an Object that that refereces in the SMA structures. 
//	The structures should be referenced as read-only.
//	The SM can use the handles of CA/QP/CQ's for its own usage if required.
//	No tracking of resources will be done if Handles are used.
//
//
// INPUTS:
//
//	RcvCallback		-Callback to be invoked if SMP's are received.
//
//	EventCallback	-Event callback for all events.
//
//	Context			-Global context
//
//
// OUTPUTS:
//
//	SmObject		-A pointer to an Object that references into the SMA 
//					 structures.
//
//
// RETURNS:
//
//
//
FSTATUS
iba_smi_open(
	IN	SM_RCV_CALLBACK		*RcvCallback,
	IN	SM_EVENT_CALLBACK	*EventCallback,
	IN	void				*Context,		
	OUT	SMA_OBJECT			**SmObject
	)
{
	FSTATUS					status = FSUCCESS;
	SMA_OBJECT_PRIVATE		*pSmaObj;

	
	_DBG_ENTER_LVL(_DBG_LVL_OBJ, iba_smi_open);

	//
	// Param check
	//


	//
	// lock
	//

	if ( NULL == g_Sma->SmUserTbl )
	{
		// This value should come from registry
		g_Sma->SmUserTbl = (SMA_USER_TABLE*)MemoryAllocateAndClear(
			sizeof( *(g_Sma->SmUserTbl) )
					+ sizeof(*(g_Sma->SmUserTbl->SmUser))* g_Settings.MaxUsers
					+ sizeof(uint64),
			FALSE, SMA_MEM_TAG);
		
		if ( NULL != g_Sma->SmUserTbl )
		{
			//g_Sma->SmUserTbl->Current = 0;
			g_Sma->SmUserTbl->Total = g_Settings.MaxUsers;
			g_Sma->SmUserTbl->SmUser = (SMA_OBJECT_PRIVATE *) \
				((uint8*)(g_Sma->SmUserTbl)
					+ ROUNDUPP2(sizeof( SMA_USER_TABLE ), sizeof(uint64)));
		}
	}

	if ( NULL != g_Sma->SmUserTbl )
	{
		if (g_Sma->SmUserTbl->Current == g_Sma->SmUserTbl->Total )
		{
			_DBG_ERROR(("Maximum users reached!\n"));
			status = FERROR;
		}
		else
		{
			pSmaObj = &g_Sma->SmUserTbl->SmUser[g_Sma->SmUserTbl->Current];
			pSmaObj->u.UserNo = g_Sma->SmUserTbl->Current;
			g_Sma->SmUserTbl->Current++;

			status = SmaCreateSmaObj( &pSmaObj->SmaObj );
			if ( FSUCCESS == status )
			{

				//
				// Update callback info
				//

				pSmaObj->u.RcvCallback = RcvCallback;
//				pSmaObj->u.PostCallback = PostCallback;
				pSmaObj->u.EventCallback = EventCallback;
				pSmaObj->u.Context = Context;

				if ( RcvCallback )
				{

					//
					// PS: There can only be one receive callback
					// Set default values fr callback
					//

					g_Sma->SmUserTbl->DefRecv.RcvCallback = RcvCallback;
					g_Sma->SmUserTbl->DefRecv.Context = Context;
					g_Sma->SmUserTbl->DefRecv.EventCallback = EventCallback;
				}

				*SmObject = &pSmaObj->SmaObj;
				g_Sma->NumUser++;			// increment no. of user count
			}
		}
	}
	else
	{
		_DBG_ERROR(("MemAlloc failed for iba_smi_open!\n"));
	}


	//
	// unlock
	//

	
    _DBG_LEAVE_LVL(_DBG_LVL_OBJ);
    
    return status;
}



//
// SmaClose
//
//	Dereferences usage of the SMA used previously by iba_smi_open().
//	No tracking of resources will be done if Handles are used.
//
//
// INPUTS:
//
//	SmObject		-Object returned in the Open call.
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
iba_smi_close(
	IN	SMA_OBJECT			*SmObject
	)
{
	FSTATUS					status = FSUCCESS;
	SMA_OBJECT_PRIVATE		*pSmaObj;

	
	_DBG_ENTER_LVL(_DBG_LVL_OBJ, iba_smi_close);


	//
	// Param check
	//


	//
	// lock
	//

	if ( NULL != g_Sma->SmUserTbl )
	{
		pSmaObj = (SMA_OBJECT_PRIVATE *)SmObject;

		pSmaObj->u.UserNo = 0;
		g_Sma->SmUserTbl->Current--;


		// Update callback info
		if ( pSmaObj->u.RcvCallback == \
			g_Sma->SmUserTbl->DefRecv.RcvCallback )
		{
			// Deregister default callbacks
			g_Sma->SmUserTbl->DefRecv.RcvCallback = NULL;
			g_Sma->SmUserTbl->DefRecv.Context = NULL;
			g_Sma->SmUserTbl->DefRecv.EventCallback = NULL;
		}

		g_Sma->NumUser--;			// Decrement no. of user count


		// free user CA/port table
		if ( pSmaObj->SmaObj.CaTbl )
			MemoryDeallocate( pSmaObj->SmaObj.CaTbl );
	} else {
		_DBG_ERROR(("Could not locate userInfo!\n"));
	}

	// unlock
	
    _DBG_LEAVE_LVL(_DBG_LVL_OBJ);
    
    return status;
}



//
// SmaUpdateInfo
//
//	The caller uses this function to update the SMA_OBJECT to reflect any
//	changes to Channel adapters and their states. The call is done 
//	immedeatly after an state change event is received by the caller.
//
//
// INPUTS:
//
//	SmObject		-Object returned in the Open call.
//
//
// OUTPUTS:
//
//	SmObject		-Updated Object information.
//
//	Event			-Update information based on this event.
//
// RETURNS:
//
//
//
FSTATUS
iba_smi_update_info(
	IN OUT	SMA_OBJECT		*SmObject
	//IN	SMA_EVENT_TYPE	Event
	)
{
	FSTATUS					status = FSUCCESS;
	SMA_OBJECT				*pSmaObj;

	
	_DBG_ENTER_LVL(_DBG_LVL_OBJ, iba_smi_update_info);


	//
	// Param check
	//


	//
	// lock
	//


	//
	// BUGBUG:
	// Lookup type of event
	//

	//
	// If event is Add device/ Remove device
	//

	pSmaObj = SmObject;


	//
	// first free old table
	//

	if( pSmaObj->CaTbl ) {
		MemoryDeallocate( pSmaObj->CaTbl );
		pSmaObj->CaTbl  = NULL;
		pSmaObj->NumCa  = 0;
	}


	//
	// create new table
	//

	status = SmaCreateSmaObj( pSmaObj );
	

	//
	// unlock
	//

	
    _DBG_LEAVE_LVL(_DBG_LVL_OBJ);
    
    return status;
}

