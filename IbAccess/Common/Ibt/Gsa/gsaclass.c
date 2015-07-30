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


// Manage list of Registered Gsa Class Clients/Managers

//
// Function prototype.
//
boolean
FindMatchingClassMgr( 
	IN LIST_ITEM *pListItem, 
	IN void *Context );

FSTATUS
GetClientId(
	IN	OUT	uint8	*pClientId
	);

void
PutClientId(
	IN	uint8		ClientId
	);


//
// GsiRegisterClass
//
//	GsiRegisterClass registers a management class with GSI.
//	A class is identified by a unique id, a version. The class owner
//  can also register a callback that can be used to indicate
//  incoming packets targeted for this class and a callback to 
//	indicate post send completions.
//
//
//	This function registers an agent defined by a management class. Any number
//	of agents can register using the same management class. In addition, 
//	only one agent at the most may register as a class manager for the class 
//	by indicating Server in Flags. 
//
//
//
//
// INPUTS:
//
//	MgmtClass		- Management class to be registered
//					  An 8-bit value indicating the management class. 
//					  In debug mode, this value has to match one of the known 
//					  values, else we print out an error and then return an 
//					  error code. 
//					  In release mode, we do not check these values. 
//					  This is only to ensure that well-known agents 
//					  register with us.
//
//	MgmtClassVersion- Will be 1 for this version.
//
//	Flags			- Indicates if Server or client. If a server is already
//					  registered, this call will fail.
//					  If TRUE, indicates the registering agent is a class 
//					  manager. When this is set to TRUE,
//
//					  GSI checks to see that there is no existing class manager
//					  for this class. If there is no existing class manager, 
//					  this function proceeds else returns failure.
//
//
//					- Indicates if client wishes to get unsolicited messages
//
//	SARClient		- TRUE indicates client needs SAR capability.
//
//	ServiceContext	- Caller defined context passed back in callbacks.
//
//	SendCompletionCallback- Callback for all datagrams completions posted 
//					  in send.
//
//	ReceiveCallback	- Callback for all datagrams received.
//
//	
//
// OUTPUTS:
//
//	ServiceHandle	- Client unique handle returned to caller on successful
//					  registration.
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
iba_gsi_register_class(
	IN	uint8						MgmtClass,
	IN	uint8						MgmtClassVersion,
	IN	GSI_REGISTRATION_FLAGS		RegistrationFlags,
	IN	boolean						SARClient,
	IN	void						*ServiceContext,
	IN	GSI_SEND_COMPLETION_CALLBACK *SendCompletionCallback,
	IN	GSI_RECEIVE_CALLBACK		*ReceiveCallback,
	OUT IB_HANDLE					*ServiceHandle
	)
{
	FSTATUS					status;
    GSA_SERVICE_CLASS_INFO	*newclass = NULL;

	_DBG_ENTER_LVL (_DBG_LVL_CLASS, iba_gsi_register_class);
	
	//
	//	Grab registration lock. This is a lock that is global to the whole 
	//	of GSI and guards the registered class table or list. 

	//	Allocate a class registration element. This element contains the 
	//	following.
	//
	//	Link element to link into a linked list.
	//		<class,version>
	//		list of registered send buffers for this client.
	//
	//	At any time, this list includes all the buffers that have been posted 
	//	( either during <registerbuffer> or during <send> ) but not yet been 
	//	processed. A buffer is considered processed once the CA to which the 
	//	buffer has been posted is used for sending data and indicated back to 
	//	the entity posting the buffer.
	//
	//	list of registered receive buffers for this client.
	//	GSI provides facilities to allocate memory on behalf of a client and 
	//	post WQE mapping that memory to a CA's QP1. In reality, the service 
	//	agent ( "client" ) does not own this memory. The client only
	//	informs GSI to allocate some memory of certain length. This is useful 
	//	when the client knows it is	about to receive data and wants to ensure 
	//	that enough buffers have been posted to receive data.
	//	There is no guarantee that these buffers are used to receive the data 
	//	meant for a client. It is possible that data for another client was 
	//	received in these buffers . Under these conditions, it is possible
	//	for the original client to lose the data since there was not enough 
	//	buffer space. Since GSI is just providing access to QP1 
	//	( for a given port on a given CA ), it is not possible for GSI to 
	//	properly know the traffic patterns and actively post buffers.
	//
	//	Client's context for this registration 
	//		( the "ServiceAgentContext" parameter ).
	//	Client's send completion and receive callbacks.
	//
	

	newclass = (GSA_SERVICE_CLASS_INFO*)MemoryAllocateAndClear(sizeof(GSA_SERVICE_CLASS_INFO), FALSE, GSA_MEM_TAG);
    if ( NULL == newclass )
    {
        status = FINSUFFICIENT_RESOURCES;
		_DBG_ERROR (("FINSUFFICIENT_RESOURCES\n"));
    }
    else
    {
		status = FSUCCESS;

        newclass->MgmtClass = MgmtClass;
        newclass->MgmtClassVersion = MgmtClassVersion;
		newclass->RegistrationFlags = RegistrationFlags;
		newclass->bSARRequired = SARClient;

        newclass->ServiceClassContext = ServiceContext;
        newclass->ReceiveCompletionCallback = ReceiveCallback;
        newclass->SendCompletionCallback = SendCompletionCallback;

        newclass->pGlobalInfo = g_GsaGlobalInfo;

#if defined(DBG) || defined(IB_DEBUG)
		newclass->SigInfo = GSA_CLASS_SIG;
#endif
		//
		// Insert this class onto the global list of service classes 
		//
		SpinLockAcquire( &g_GsaGlobalInfo->ServiceClassListLock );

		if (FSUCCESS != GetClientId( &newclass->ClientId ))
		{
			SpinLockRelease( &g_GsaGlobalInfo->ServiceClassListLock );
			MemoryDeallocate( newclass );

	        status = FINSUFFICIENT_RESOURCES;
			_DBG_ERROR (("FINSUFFICIENT_RESOURCES\n"));
		}
		else
		{
			QListInsertHead( &g_GsaGlobalInfo->ServiceClassList, 
				&newclass->ListItem );
			SpinLockRelease( &g_GsaGlobalInfo->ServiceClassListLock );

			_DBG_PRINT (_DBG_LVL_CLASS,(
				"Registered Info...:\n"
				"\tClass...........:x%x\n"
				"\tClientID........:x%x\n"
				"\tClassContext....:x%p\n"
				"\tSAR capable.....:%s\n",
				newclass->MgmtClass,
				newclass->ClientId,
				_DBG_PTR(newclass->ServiceClassContext),
				_DBG_PTR(newclass->bSARRequired ? "TRUE":"FALSE" )));

			_DBG_PRINT (_DBG_LVL_CLASS,(
				"\tResponder.......:%s\n"
				"\tTrapProcessor...:%s\n"
				"\tReportProcessor.:%s\n",
				_DBG_PTR(GSI_IS_RESPONDER(newclass->RegistrationFlags)
					? "TRUE":"FALSE"),
				_DBG_PTR(GSI_IS_TRAP_PROCESSOR(newclass->RegistrationFlags)
					? "TRUE":"FALSE"),
				_DBG_PTR(GSI_IS_REPORT_PROCESSOR(newclass->RegistrationFlags)
					? "TRUE":"FALSE")));

			//
			// Set capabilities in PortInfo through SMA.  Enable class
			// management support if not already enabled.
			//
			if( GSI_IS_RESPONDER(newclass->RegistrationFlags)  )
				SetPortCapabilityMask( newclass->MgmtClass, TRUE );

			*ServiceHandle = ( IB_HANDLE )newclass;
		}
    }

	_DBG_LEAVE_LVL (_DBG_LVL_CLASS);
	return status;
}




//
// GsiDeregisterClass
//
//
//
// INPUTS:
//
//	ServiceHandle	- Handle returned upon a successful registration.
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
iba_gsi_deregister_class(
	IN	IB_HANDLE					ServiceHandle
	)
{
	GSA_SERVICE_CLASS_INFO		*pClassInfo;
	LIST_ITEM					*pListItem;
    
	_DBG_ENTER_LVL (_DBG_LVL_CLASS, iba_gsi_deregister_class);

	pClassInfo = (GSA_SERVICE_CLASS_INFO*)ServiceHandle;
	ASSERT( pClassInfo );

	_DBG_PRINT (_DBG_LVL_CLASS,(
		"Registered Info...:\n"
		"\tClass...........:x%x\n"
		"\tClientID........:x%x\n"
		"\tClassContext....:x%p\n"
		"\tSAR capable.....:%s\n",
		pClassInfo->MgmtClass,
		pClassInfo->ClientId,
		_DBG_PTR(pClassInfo->ServiceClassContext),
		_DBG_PTR(pClassInfo->bSARRequired ? "TRUE":"FALSE") ));
		
	_DBG_PRINT (_DBG_LVL_CLASS,(
		"\tResponder.......:%s\n"
		"\tTrapProcessor...:%s\n"
		"\tReportProcessor.:%s\n",
		_DBG_PTR(GSI_IS_RESPONDER(pClassInfo->RegistrationFlags)
			? "TRUE":"FALSE"),
		_DBG_PTR(GSI_IS_TRAP_PROCESSOR(pClassInfo->RegistrationFlags)
			? "TRUE":"FALSE"),
		_DBG_PTR(GSI_IS_REPORT_PROCESSOR(pClassInfo->RegistrationFlags)
			? "TRUE":"FALSE")));
	
#if defined(DBG) || defined(IB_DEBUG)
	if (pClassInfo->SigInfo != GSA_CLASS_SIG)
	{
		_DBG_ERROR (("Bad handle passed!\n"));
		ASSERT(0);
	}
#endif

	// Remove this class from the global list of service classes 
	SpinLockAcquire( &g_GsaGlobalInfo->ServiceClassListLock );

	ASSERT( QListIsItemInList( &g_GsaGlobalInfo->ServiceClassList, 
		&pClassInfo->ListItem ) );
	QListRemoveItem( &g_GsaGlobalInfo->ServiceClassList, 
		&pClassInfo->ListItem );

	PutClientId (pClassInfo->ClientId);

	SpinLockRelease( &g_GsaGlobalInfo->ServiceClassListLock );

	// If this client was the last manager for this class, disable the
	// port attributes for it.
	if( GSI_IS_RESPONDER(pClassInfo->RegistrationFlags))
	{
		// Client was a class manager.  See if there is another one.
		SpinLockAcquire( &g_GsaGlobalInfo->ServiceClassListLock );

		pListItem = QListFindFromHead( &g_GsaGlobalInfo->ServiceClassList,
			FindMatchingClassMgr, (void*)(uintn)pClassInfo->MgmtClass );

		SpinLockRelease(&g_GsaGlobalInfo->ServiceClassListLock);

		// If we didn't find another class manager, disable the class on
		// the port attributes.
		if( !pListItem )
			SetPortCapabilityMask( pClassInfo->MgmtClass, FALSE );
	}

#if defined(DBG) || defined(IB_DEBUG)
	pClassInfo->SigInfo = 0;
#endif

	MemoryDeallocate( pClassInfo );

	_DBG_LEAVE_LVL (_DBG_LVL_CLASS);
	return FSUCCESS;
}



//
// Return TRUE if the class information specified by pListItem is a class
// manager for the class specified by Context.
//
boolean
FindMatchingClassMgr( 
	IN LIST_ITEM *pListItem, 
	IN void *Context )
{
	GSA_SERVICE_CLASS_INFO		*pClassInfo;
	
	pClassInfo = (GSA_SERVICE_CLASS_INFO*)pListItem;
	// we cast the generic void* supplied by the list search routines into
	// the IBTA GSA Class Number
	return( GSI_IS_RESPONDER(pClassInfo->RegistrationFlags) && 
		(pClassInfo->MgmtClass == (uint8)(uintn)Context) );
}

//
// Return FSUCCESS if an ID is found in list
//
FSTATUS
GetClientId(
	IN	OUT	uint8	*pClientId
	)
{
	FSTATUS			status = FERROR;
	uint8			*pIdList	= &g_GsaGlobalInfo->ClientIdArray[0];
	uint8			i;

	for ( i=0; i<MAX_CLIENT_ID; i++ )
	{
		if	(!pIdList[i])
		{
			pIdList[i] = 1;
			*pClientId = i;
			status = FSUCCESS;
			break;
		}
	}

	return (status);
}


//
// Return Old Id back to list for reuse
//
void
PutClientId(
	IN	uint8		ClientId
	)
{
	uint8			*pIdList	= &g_GsaGlobalInfo->ClientIdArray[0];

	pIdList[ClientId]=0;
}

//
// Return TRUE if the ClientId/MgmtClass wants GSI do to RMPP SAR
// ClientId - for inbound request packets, should be MAX_CLIENT_ID
//		in which case if any manager has requested GSI to do SAR we return TRUE
//	- for inbound response packets, should be client ID field out of TID
//		in which case we look at the specific registration of the client
//
boolean
IsSarClient(
	IN	uint8				ClientId,
	IN	uint8				MgmtClass )
{
    GSA_SERVICE_CLASS_INFO	*pClassInfo;
	LIST_ITEM				*pListNext;
    boolean                 bSARRequired = FALSE;
    
	_DBG_ENTER_LVL( _DBG_LVL_CALLBACK, GsaClientNeedsSAR );

	SpinLockAcquire( &g_GsaGlobalInfo->ServiceClassListLock );

	for( pListNext = QListHead( &g_GsaGlobalInfo->ServiceClassList );
		 NULL != pListNext;
		 pListNext = QListNext( &g_GsaGlobalInfo->ServiceClassList, pListNext) )
	{
		pClassInfo = PARENT_STRUCT( pListNext,
								GSA_SERVICE_CLASS_INFO, ListItem );
		if (ClientId != MAX_CLIENT_ID)
		{
			// See if initiating client has requested GSI to do SAR
			if( pClassInfo->ClientId == ClientId )
			{
				bSARRequired = pClassInfo->bSARRequired;
				break;
			}
		} else {
			// See if class manager has requested GSI to do SAR
			if ((MgmtClass == pClassInfo->MgmtClass) &&
				(GSI_IS_RESPONDER(pClassInfo->RegistrationFlags) && 
				pClassInfo->bSARRequired))
			{
				bSARRequired = TRUE;
				break;
			}
		}
	}		// class for loop

	SpinLockRelease( &g_GsaGlobalInfo->ServiceClassListLock );

	_DBG_LEAVE_LVL( _DBG_LVL_CALLBACK );
	return bSARRequired;
}


//
// Call the clients of the GSA back with received messages.
//
FSTATUS
GsaNotifyClients(
	IN	uint8				ClientId,
	IN	MAD_COMMON			*pMadHdr,
	IN	IBT_DGRM_ELEMENT	*pDgrm )
{
	FSTATUS						status = FNOT_FOUND;
    GSA_SERVICE_CLASS_INFO		*pClassInfo;
	LIST_ITEM					*pListNext;
	boolean 					isInterested;
#if defined(IB_DEBUG) || defined(DBG)
	uint32 count = 0;
#endif
    
	_DBG_ENTER_LVL( _DBG_LVL_CALLBACK, GsaNotifyClients );
	ASSERT( pMadHdr && pDgrm );
    
	// To do: coordinate callbacks with a client deregistering.  This
	// will require changes to the destruction model to be asynchronous.
	SpinLockAcquire( &g_GsaGlobalInfo->ServiceClassListLock );
	for( pListNext = QListTail( &g_GsaGlobalInfo->ServiceClassList );
		 NULL != pListNext;
         pListNext = QListPrev( &g_GsaGlobalInfo->ServiceClassList, pListNext) )
	{
		pClassInfo =PARENT_STRUCT(pListNext, GSA_SERVICE_CLASS_INFO, ListItem );

		// See if this is a request or a response.
		if (MAD_IS_RESPONSE((MAD*)pMadHdr))
		{
			// Responses are only returned to the initiating client.
			if( pClassInfo->ClientId == ClientId )
			{
				SpinLockRelease( &g_GsaGlobalInfo->ServiceClassListLock );
				status = (pClassInfo->ReceiveCompletionCallback)(
									pClassInfo->ServiceClassContext, pDgrm );
				break;
			}
		} else {
			// New requests are directed to the responder or anyone who
			// requires access to all unsolicited messages.
			
			isInterested = FALSE;
			
			if ( pMadHdr->MgmtClass == pClassInfo->MgmtClass ) {
				boolean fanout;	// distribute dgrm to all interested parties

				switch ( pMadHdr->mr.AsReg8 ) {
					case MMTHD_REPORT:
						isInterested = GSI_IS_REPORT_PROCESSOR( pClassInfo->RegistrationFlags );
						fanout = TRUE;
						break;
						
					case MMTHD_TRAP:
						isInterested = GSI_IS_TRAP_PROCESSOR( pClassInfo->RegistrationFlags );
						fanout = TRUE;
						break;
						
					default:
						isInterested = GSI_IS_RESPONDER( pClassInfo->RegistrationFlags );
						fanout = FALSE;
						break;
				}
				
				if ( isInterested ) {
					FSTATUS ret;

					SpinLockRelease( &g_GsaGlobalInfo->ServiceClassListLock );
#if defined(IB_DEBUG) ||defined( DBG)
					if (fanout)
						count++;
#endif
					ret = ( pClassInfo->ReceiveCompletionCallback )(
										pClassInfo->ServiceClassContext, pDgrm );

					// If we are fanning out, we will provide it to all
					// interested parties.  However if someone keeps a reference
					// (FPENDING), we can't give it to anyone else.  Typically
					// for traps and notices FCOMPLETED is returned which allows
					// fanout to work.
					//
					// If we are not fanning out, only the first who reports
					// FCOMPLETED or FPENDING will get it.
					//
					// For other cases we update final return status to reflect
					// our best case result.  (eg. we don't want to report
					// NOT_FOUND if we found someone who FREJECT'ed it, etc).
					if ( ret == FPENDING) {
						status = FPENDING;
						_DBG_PRINT (_DBG_LVL_CLASS,("FPENDING in GsaNotifyClients\n"));
						break; // they grabbed reference, must stop
					} else if (ret == FCOMPLETED) {
						status = FCOMPLETED;
						if (! fanout)
							break;	// only give to 1st who wants it
					} else if (status == FNOT_FOUND) {
						status = ret;	// upgrade our return for FREJECT, etc.
					}

					SpinLockAcquire( &g_GsaGlobalInfo->ServiceClassListLock );
				}
			}
		}
	}

#if defined(IB_DEBUG) || defined(DBG)
	if (count)
		_DBG_PRINT (_DBG_LVL_CLASS,("GsaNotifyClients: %d clients\n", count));
#endif
	// If we didn't find a client to hand the receive to, we're still holding
	// the spinlock.
	if( !pListNext )
		SpinLockRelease( &g_GsaGlobalInfo->ServiceClassListLock );

	_DBG_LEAVE_LVL( _DBG_LVL_CALLBACK );
	return status;
}
