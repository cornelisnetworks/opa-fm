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
// SmaProcessLocalToSma
//
//	Process Local SMP activity to SMA.
//	Does a local Operation and sets internal states
//
//
//
// INPUTS:
//
//	SmpBlock		- SMP Block received
//
// OUTPUTS:
//
//	TRUE/FALSE		- TRUE: SMP processed and used
//					- FALSE: Recycle SMP
//
// RETURNS:
//
//	FSTATUS.
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL.
//

static boolean
SmaProcessLocalToSma(
	IN	POOL_DATA_BLOCK		*SmpBlock
	)
{
	FSTATUS					status = FSUCCESS;
	STL_SMP					*pSmp = (STL_SMP*)SmpBlock->Block.Smp;
	SMA_CA_OBJ_PRIVATE		*pCaObj;
	SMA_PORT_TABLE_PRIV		*pPortTbl;
	SMA_PORT_BLOCK_PRIV		*pPortBlock;
	IB_PORT_STATE			oldPortState;
	boolean					bStatus=TRUE;		// default processed
	uint8					portNo,oldLMC;
	IB_LID					oldLID, oldSmLID;
	uint8					method;
	boolean					zero_lid;	

	_DBG_ENTER_LVL( _DBG_LVL_CALLBACK, SmaProcessLocalToSma );

	pCaObj = (SMA_CA_OBJ_PRIVATE *)SmpBlock->Block.SmaCaContext;
	pPortTbl = (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;

	/* special case, suppress INVALID_ATTRIB error message on Set(PortInfo)
	 * with 0 LID.  This can occur for host SM when 1st starting only SM
	 * on fabric.  What happens is SM sets capability mask using present LID
	 * which is 0 when HFI first comes up.
	 */
	zero_lid = (pSmp->common.mr.AsReg8 == MMTHD_SET
		&& pSmp->common.AttributeID == STL_MCLASS_ATTRIB_ID_PORT_INFO
		&& 0 == ((STL_PORT_INFO *)&pSmp->SmpExt.DirectedRoute.SMPData)->LID);

	// save method
	method = pSmp->common.mr.AsReg8;
	// clear Status
	pSmp->common.u.DR.s.Status = MAD_STATUS_SUCCESS;

	if (pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0)
	{
		if (SmpBlock->Block.PortNumber != 0)
		{
			_DBG_ERROR(("SmpBlock->Block.PortNumber == %d, should be 0!\n", SmpBlock->Block.PortNumber));
			status = FERROR;
		}
		portNo = 0;
	} else {
		portNo = SmpBlock->Block.PortNumber - 1;
	}

	// call verb
	if (FSUCCESS == status)
	{
		status = iba_get_set_mad(
					pPortTbl->PortBlock[portNo].QpHandle,
					SmpBlock->Block.PortNumber,
					SmpBlock->Block.SLID,
					pSmp,
					&SmpBlock->Block.SmpByteCount);
	}

	if ( FSUCCESS != status )
	{
		// FNOT_DONE indicates an internal MAD which we chose to quietly discard
		if (FNOT_DONE != status)
		{
			_DBG_ERROR(("GetSetMad() failed with x%x!\n", status));
		}

		// return VPD reported status in status
		SmpBlock->Block.Status = status;
		bStatus = FALSE;
	}
	// we need to update port state view even if Mad returns
	// INVALID_ATTRIB valid because it is reporting the new state
	else if (pSmp->common.u.DR.s.Status == MAD_STATUS_SUCCESS
		|| pSmp->common.u.DR.s.Status == MAD_STATUS_INVALID_ATTRIB)
	{
		// Update internal states 
		switch ( pSmp->common.AttributeID )
		{
		default:
			{
				break;
			}

		case	MCLASS_ATTRIB_ID_PORT_INFO:
			{
				// Update internal port state: LID, LMC, State
				if (pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0)
				{
					if (pSmp->common.AttributeModifier != 0)
					{
						// Nothing to cache for switch ports other than 0
						break;
					}
					portNo = 0;	// Switch port 0 is treated as first 0 in list
				} else {
					portNo = (uint8)(pSmp->common.AttributeModifier & 0x000000FF);
					if (0 == portNo)	// Is it the default port?
					{
						portNo = SmpBlock->Block.PortNumber - 1;	// use in port
					} else {
						portNo--;		// get zero based index into port tables
	
						// Check for invalid port number, GetSetMad should have
						// already checked, so we do not expect the test below
						// to fail.
						if ( portNo >= pCaObj->CaPublic.Ports )
						{
							_DBG_ERROR(("Invalid Port: %d\n", portNo+1));
							bStatus = FALSE;		// SMP not processed
						}
					}
				}

				if (bStatus)
				{
					STL_PORT_INFO *pPortInfo =
						(STL_PORT_INFO *)&pSmp->SmpExt.DirectedRoute.SMPData;
					BSWAP_STL_PORT_INFO(pPortInfo);
					ZERO_RSVD_STL_PORT_INFO(pPortInfo);
					pPortBlock = &pPortTbl->PortBlock[portNo];
					
					// Before we update our internal state save 
					// old state for callbacks
					oldPortState = pPortBlock->Public.State;
					oldLID = pPortBlock->Public.Address.BaseLID;
					oldLMC = pPortBlock->Public.Address.LMC;
					oldSmLID = pPortBlock->SMLID;

					// Update internal state
					pPortBlock->Public.Address.BaseLID = pPortInfo->LID;
					pPortBlock->Public.Address.LMC = pPortInfo->s1.LMC;
					pPortBlock->Public.State = (IB_PORT_STATE) pPortInfo->PortStates.s.PortState;
					pPortBlock->SMLID = pPortInfo->MasterSMLID;

					// Notify users of a Port event
					if ( oldPortState != pPortBlock->Public.State )
					{
						if ( IB_PORT_ACTIVE == pPortBlock->Public.State )
						{
							IbtNotifyGroup( pPortBlock->Public.GUID,
												IB_NOTIFY_LID_EVENT );	
						}

						// Here any other state will be a port down event
						// An SM could change a port status from Active to 
						// Armed to reconfigure it. We take care of a case
						// where connections were active and go down on an
						// incoming ARM Smp that needs to report to Channel
						// drivers that the port is down.
						if (( IB_PORT_DOWN == pPortBlock->Public.State ) ||\
							( IB_PORT_ACTIVE == oldPortState ))
						{
							IbtNotifyGroup( pPortBlock->Public.GUID,
												IB_NOTIFY_PORT_DOWN );	
						}
					} else {
						// Notify changes to Active LID/LMC when multipathing
						// kicks in or on a SM takeover.
						if ( IB_PORT_ACTIVE == pPortBlock->Public.State ) 
						{
							if (( oldLID != pPortInfo->LID ) || \
								( oldLMC != pPortInfo->s1.LMC ))
							{
								IbtNotifyGroup( pPortBlock->Public.GUID,
												IB_NOTIFY_LID_EVENT );	
							} else if ( oldSmLID != pPortInfo->MasterSMLID 
								 || (method == MMTHD_SET && pPortInfo->Subnet.ClientReregister) )
							{
								IbtNotifyGroup( pPortBlock->Public.GUID,
												IB_NOTIFY_SM_EVENT );	
							}
						}
					}
					BSWAP_STL_PORT_INFO(pPortInfo);
				} // valid portNo

				break;
			}

		case	STL_MCLASS_ATTRIB_ID_PART_TABLE:
			if (bStatus && method == MMTHD_SET)
			{
				// Notify users of PKey event
				if (pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0)
				{
					portNo = (pSmp->common.AttributeModifier >> 16) & 0x00FF;
					if (portNo != 0)
					{
						break;
					}
				} else {
					portNo = SmpBlock->Block.PortNumber - 1;	// use in port
				}
				
				pPortBlock = &pPortTbl->PortBlock[portNo];

				// BUGBUG TBD compare PKeys and only notify if they changed
				IbtNotifyGroup( pPortBlock->Public.GUID, IB_NOTIFY_PKEY_EVENT);
			}

			break;

		}	// switch

		if (pSmp->common.u.DR.s.Status == MAD_STATUS_INVALID_ATTRIB
			&& ! zero_lid)
		{
			_DBG_ERROR(("GetSetMad() returned MAD with Status x%x!\n", 
					pSmp->common.u.DR.s.Status));
		}
	} else {
		_DBG_ERROR(("GetSetMad() returned MAD with Status x%x!\n", 
					pSmp->common.u.DR.s.Status));
	}		// GetSetMad status

	_DBG_RETURN_LVL(_DBG_LVL_CALLBACK, bStatus);

	return (bStatus);
}

//
// SmaProcessSmaSmpRcvd
//
//	Processes SMA SMP received.
//	A check is done to verify the validity of the SMP.
//	If the SMP needs further processing like a reply, this returns TRUE
//	and the caller must ProcessSmiSmp and if appropriate
//	post a reply on the SendQ of this port's QP.
//
//
// INPUTS:
//
//	SmpBlock		- SMP Block received
//
// OUTPUTS:
//
//	TRUE/FALSE		- TRUE: SMP processed and used
//					- FALSE: Recycle SMP
//
// RETURNS:
//
//	FSTATUS.
//
// IRQL:
//
//	This routine is called at IRQL_DISPATCH_LEVEL.
//

boolean
SmaProcessSmaSmpRcvd(
	IN	POOL_DATA_BLOCK		*SmpBlock
	)
{
	STL_SMP					*pSmp = (STL_SMP*)SmpBlock->Block.Smp;
	SMA_CA_OBJ_PRIVATE		*pCaObj;
	SMA_PORT_TABLE_PRIV		*pPortTbl;
	boolean					bProcess=TRUE;


	_DBG_ENTER_LVL(_DBG_LVL_CALLBACK, SmaProcessSmaSmpRcvd);

	pCaObj = (SMA_CA_OBJ_PRIVATE *)SmpBlock->Block.SmaCaContext;
	pPortTbl = (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;


	// M_Key check is expected to be done by SmaProcessLocalToSma
	// it should keep track of the M_Key for each port and validate them

	bProcess = SmaProcessLocalToSma( SmpBlock );
	if ( TRUE == bProcess )
	{

		//
		// Return info :
		// Set HopPointer HopCount info
		//
		
		//
		// DLID = SLID of request
		//

		if (SmpBlock->Block.SLID == 0)
		{
			// special case for locally generated Traps provided to us on
			// QP0 by firmware.  We send to SM.
			if ( MCLASS_SM_DIRECTED_ROUTE == pSmp->common.MgmtClass )
			{
				SmpBlock->Block.SLID = STL_LID_PERMISSIVE;
				SmpBlock->Block.DLID = STL_LID_PERMISSIVE;
				SmpBlock->Block.PathBits = 0;
			} else {
				SmpBlock->Block.DLID = pPortTbl[SmpBlock->Block.PortNumber - 1].PortBlock->SMLID;
				SmpBlock->Block.PathBits = 0;	// use our first lid
				SmpBlock->Block.SLID = 0;		// will fill in our lid below
			}
		} else {
			// IBTA 1.1 14.2.2.3 C14-10
			// GetResp is set by GetSetMad for SMA packets
			// D bit is set by GetSetMad for Directed Route SMA packets
			// GetSetMad uses original SMP buffer, so other parts of Route
			// are copied as is
			// C14-11 2nd bullet
			SmpBlock->Block.DLID = SmpBlock->Block.SLID;
			SmpBlock->Block.SLID = 0;		// will fill in our lid below
			// PathBits of SLID will be from DestPathBits in request
		}

		if ( MCLASS_SM_DIRECTED_ROUTE == pSmp->common.MgmtClass )
		{
			if ( STL_LID_PERMISSIVE == pSmp->SmpExt.DirectedRoute.DrDLID) {
				// IBTA 1.1 Sec 14.2.2.3 C14-11
				// the directed route part starts from the respondor node (us)
				// LRH_SLID shall be the permissive lid or a lid of this port
				// safest to use permissive lid, since we may not have a lid yet
				SmpBlock->Block.SLID = STL_LID_PERMISSIVE;
			}
		}

		//
		// If ( SLID != LID_PERMISSIVE )
		//		SLID = Responder's LID
		//

		if ( STL_LID_PERMISSIVE != SmpBlock->Block.SLID )
		{
			uint8 lmc = pPortTbl[SmpBlock->Block.PortNumber - 1].PortBlock->Public.Address.LMC;
			SmpBlock->Block.SLID = 
				PathBitsToLid(pPortTbl[SmpBlock->Block.PortNumber - 1].PortBlock->Public.Address.BaseLID,
						SmpBlock->Block.PathBits, lmc);
			_DBG_PRINT (_DBG_LVL_CALLBACK, (
					"Response lmc: %d\n"
					"Pathbits: %d\n"
					"SLID: %x\n",
					lmc,
					SmpBlock->Block.PathBits,
					SmpBlock->Block.SLID));
		}
	}

	_DBG_RETURN_LVL(_DBG_LVL_CALLBACK, bProcess);

	return (bProcess);
 
}
	
// Per SMI packet processing as defined in section 13.5.3.3 and 14.2.2
// of IBTA 1.1
// This should be used to process all SMP packets arriving at the SMI
// both from QP0, responses from the local SMA, and packets from SmaPostSend
// 
// INPUTS:
//
//	pCaObj			- CA receiving block
//	SmpBlock		- SMP Block delivered to Smi
//	FromWire		- was SMP received on QP0 or delivered locally to SMI
//						affects LID routed identification
//
// RETURNS:
// 	SMI_DISCARD - SMP should be discarded
// 	SMI_SEND - SMP should be sent on the wire via PortNumber in pSmpBlock
// 	SMI_TO_LOCAL_SM - SMP should be delivered to local SM
// 	SMI_TO_LOCAL - SMP should be delivered to local SM or SMA
// 				see SmaProcessLocalSmp
// 	SMI_LID_ROUTED - LID routed SMP, destination will depend on context of call
SMI_ACTION
SmaProcessSmiSmp(
	IN	SMA_CA_OBJ_PRIVATE	*pCaObj,
	IN OUT POOL_DATA_BLOCK	*pSmpBlock,
	IN	boolean				FromWire
	)
{
	IN STL_SMP			*pSmp     = (STL_SMP*)pSmpBlock->Block.Smp;
	SMA_PORT_TABLE_PRIV	*pPortTbl = (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;

	_DBG_ENTER_LVL(_DBG_LVL_CALLBACK, SmaProcessSmiSmp);

	_DBG_PRINT (_DBG_LVL_CALLBACK, (
			"SMI Processing for:\n"
			"\tClass........:x%x\n"
			"\tAttribID.....:x%X\n"
			"\tAttribModfier:x%X\n"
			"\tMethod.......:x%X\n"
			"\tResponseBit..:x%X\n"
			"\tHopCount.....:x%X\n",
			pSmp->common.MgmtClass,
			pSmp->common.AttributeID,
			pSmp->common.AttributeModifier,
			pSmp->common.mr.s.Method,
			pSmp->common.mr.s.R,
			pSmp->common.u.DR.HopCount
			));
	_DBG_PRINT (_DBG_LVL_CALLBACK, (
			"SMI Processing for (cont):\n"
			"\tHopPointer...:x%X\n"
			"\tDrSLID.......:x%X\n"
			"\tDrDLID.......:x%X\n"
			"\tTID..........:x%"PRIx64"\n"
			"\tSLID.........:x%X\n"
			"\tDLID.........:x%X\n",
			pSmp->common.u.DR.HopPointer,
			pSmp->SmpExt.DirectedRoute.DrSLID,
			pSmp->SmpExt.DirectedRoute.DrDLID,
			pSmp->common.TransactionID,
			pSmpBlock->Block.SLID,
			pSmpBlock->Block.DLID
			));

	// Algorithm for SMP handling per IBTA 1.1 Section 13.5.3.3
	// Rather than be elegant, the flow charts are followed fairly
	// directly, allowing for easier code review and support.
	// However, there are cases where the flow charts are in error
	// in which case the text in 14.2.2 takes precedence.
	// Assumes LRH, BTH and GRH checks are done in the Verbs
	// Provider Driver
				
	// Figure 157, SMP Check 1
	if (pSmp->common.BaseVersion != STL_BASE_VERSION
		|| pSmp->common.ClassVersion != STL_SM_CLASS_VERSION)
	{
		// can optionally send a GetResp(MAD_STATUS_UNSUPPORTED_VER)
		// however since we don't understand the header we could
		// not effectively respond to directed route
		// so better to discard
        _DBG_PRINT (_DBG_LVL_CALLBACK,(
                    "Unsupported Base or Class Version [packet discarded]\n"));
		goto discard;
	}
	if( pSmp->common.MgmtClass != MCLASS_SM_DIRECTED_ROUTE
		&& pSmp->common.MgmtClass != MCLASS_SM_LID_ROUTED ) 
	{
		// drop packet, unknown class
        _DBG_PRINT (_DBG_LVL_CALLBACK,(
                    "Invalid Mgmt Class on SMI [packet discarded]\n"));
		goto discard;
	}

	if( pSmp->common.MgmtClass != MCLASS_SM_DIRECTED_ROUTE ) 
	{
		// LID routed, next check is MKey which is done in Verbs Driver/SMA
        _DBG_PRINT (_DBG_LVL_CALLBACK,("LID routed SMP\n"));
		goto lidsmp;
	}

	// not in spec, but a necessary check so indexing
	// based on HopPointer below doesn't overflow
	if (pSmp->common.u.DR.HopCount >
			sizeof(pSmp->SmpExt.DirectedRoute.InitPath)
		|| pSmp->common.u.DR.HopPointer > 
			sizeof(pSmp->SmpExt.DirectedRoute.InitPath))
	{
		_DBG_ERROR((
			"HopPointer(%d) or HopCount(%d) exceeds array bounds!\n"
			"Check your SMP values [packet discarded]\n",
			pSmp->common.u.DR.HopPointer,
			pSmp->common.u.DR.HopCount
			));
    
		goto discard;
	}

	// Figure 161, SMP Direct Route Check 1
	if ( 0 == pSmp->common.u.DR.s.D)
	{
		if ( ! FromWire && STL_LID_PERMISSIVE != pSmp->SmpExt.DirectedRoute.DrSLID)
		{
			// IBTA 1.1 sec 14.2.2.1
			// For packets originated by a local SM
			// if Directed Route Part does not start from requestor node
			// process as LID routed
			goto sendsmp;
		}
		// 14.2.2.2, C14-8, C14-9
		if (0 != pSmp->common.u.DR.HopCount
			&& 0 == pSmp->common.u.DR.HopPointer)
		{
			// SMP is at beginning of directed route
			// C14-9 clause 1)
			++(pSmp->common.u.DR.HopPointer);
			pSmpBlock->Block.SLID = STL_LID_PERMISSIVE;
			// flow chart Figure 161 disagrees with text, Text has priority
			//pSmpBlock->Block.DLID = pSmp->SmpExt.DirectedRoute.DrDLID;
			pSmpBlock->Block.DLID = STL_LID_PERMISSIVE;
			pSmpBlock->Block.PortNumber = pSmp->SmpExt.DirectedRoute.InitPath[pSmp->common.u.DR.HopPointer];
			goto sendsmp;
		}
		if (0 != pSmp->common.u.DR.HopCount
			&& 1 <= pSmp->common.u.DR.HopPointer
			&& pSmp->common.u.DR.HopPointer <
			pSmp->common.u.DR.HopCount
			)
		{
			// SMP is at intermediate hop in directed route portion of path
			// C14-9 clause 2)
			if (pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0)
			{
				// Switch code here
				pSmp->SmpExt.DirectedRoute.RetPath[pSmp->common.u.DR.HopPointer] = pSmpBlock->Block.PortNumber;
				++pSmp->common.u.DR.HopPointer;
				pSmpBlock->Block.SLID = STL_LID_PERMISSIVE;
				pSmpBlock->Block.DLID = STL_LID_PERMISSIVE;
				pSmpBlock->Block.PortNumber = pSmp->SmpExt.DirectedRoute.InitPath[pSmp->common.u.DR.HopPointer];
				goto sendsmp;
			}
			else
			{
				// we are not a switch so we drop
				_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"Invalid DR [packet discarded per Check 1]\n"));
				goto discard;
			}
		}
		// Figure 162 SMP Direct Route Check 2
		if (pSmp->common.u.DR.HopCount
			== pSmp->common.u.DR.HopPointer)
		{
			// SMP is at end of directed route portion of path
			// C14-9 clause 3)
			if (0 != pSmp->common.u.DR.HopCount)
			{
				pSmp->SmpExt.DirectedRoute.RetPath[pSmp->common.u.DR.HopPointer] = pSmpBlock->Block.PortNumber;
			}
			++(pSmp->common.u.DR.HopPointer);

			if (pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0)
			{
				if (STL_LID_PERMISSIVE == pSmp->SmpExt.DirectedRoute.DrDLID)
				{
					pSmpBlock->Block.SLID = STL_LID_PERMISSIVE;
				}
				else
				{
					pSmpBlock->Block.SLID = pPortTbl[0].PortBlock->Public.Address.BaseLID |
											pSmpBlock->Block.PathBits;
				}
			}
			else
			{
				// not a switch
				if ( STL_LID_PERMISSIVE != pSmp->SmpExt.DirectedRoute.DrDLID)
				{
					_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"Invalid DR [packet discarded per Check 2]\n"));
					goto discard;
				}
				//pSmpBlock->Block.SLID = LID_PERMISSIVE;	// optional in spec
			}

			pSmpBlock->Block.DLID = pSmp->SmpExt.DirectedRoute.DrDLID;
			// Text in clause 3 has this additional test which is
			// redundant with test above due to DLID assignment above
			// for a CA DLID will be PERMISSIVE if get here
			if ( STL_LID_PERMISSIVE == pSmpBlock->Block.DLID )
			{
				// per path above, HopPointer is now HopCount+1
#ifdef  IB_DEBUG
				ASSERT(pSmp->common.u.DR.HopCount+1 == pSmp->common.u.DR.HopPointer);
#endif
				goto localsmp;
			} else {
#ifdef  IB_DEBUG
				if (!(pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0))
				{
					// can't get here given sequence of tests above for a CA
					ASSERT(0);
				}
#endif
				goto sendsmp;
			}
		} else if ( pSmp->common.u.DR.HopPointer == pSmp->common.u.DR.HopCount+1 )
		{
			// SMP is a local request to SM or SMA
			// C14-9 clause 4)
			goto localsmp;
		} else {
			// HopPointer is in range HopCount+1 <HopPointer <=255
			// C14-9 clause 5)
        	_DBG_PRINT (_DBG_LVL_CALLBACK,(
                    "Invalid DR [packet discarded per Check 3]\n"));
			goto discard;
		}
	} else { // D bit == 1 case of below
		if ( ! FromWire && (STL_LID_PERMISSIVE != pSmp->SmpExt.DirectedRoute.DrDLID))
		{
			// IBTA 1.1 sec 14.2.2.3
			// For packets originated by a local SMA or SM
			// if Directed Route Part does not start from responder node
			// process as LID routed
			goto sendsmp;
		}
		// Figure 164 SMP Direct Route Check 4
		if (0 != pSmp->common.u.DR.HopCount
			&& pSmp->common.u.DR.HopPointer ==
				pSmp->common.u.DR.HopCount+1
			)
		{
			// SMP is at beginning of directed route
			// C14-13 clause 1)
			--(pSmp->common.u.DR.HopPointer);
			pSmpBlock->Block.SLID = STL_LID_PERMISSIVE;
			pSmpBlock->Block.DLID = pSmp->SmpExt.DirectedRoute.DrDLID;
			pSmpBlock->Block.PortNumber = pSmp->SmpExt.DirectedRoute.RetPath[pSmp->common.u.DR.HopPointer];
			goto sendsmp;
		} else if (0 != pSmp->common.u.DR.HopCount
					&& 2 <= pSmp->common.u.DR.HopPointer
					&& pSmp->common.u.DR.HopPointer <=
							pSmp->common.u.DR.HopCount
					)
		{
			// SMP is at an intermediate hop of directed route
			// C14-13 clause 2)
			if (pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0)
			{
				--pSmp->common.u.DR.HopPointer;
				pSmpBlock->Block.SLID = STL_LID_PERMISSIVE;
				pSmpBlock->Block.DLID = STL_LID_PERMISSIVE;
				pSmpBlock->Block.PortNumber = pSmp->SmpExt.DirectedRoute.RetPath[pSmp->common.u.DR.HopPointer];
			}
			else
			{
				// we are not a switch so we drop
				_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"Invalid DR [packet discarded per Check 4]\n"));
				goto discard;
			}
		}
		// Figure 165 SMP Direct Route Check 5
		if ( 1 == pSmp->common.u.DR.HopPointer)
		{
			// SMP is at end of directed route part of path
			// C14-13 clause 3)
			--(pSmp->common.u.DR.HopPointer);
			if (pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0)
			{
				if (STL_LID_PERMISSIVE == pSmp->SmpExt.DirectedRoute.DrSLID)
				{
					pSmpBlock->Block.SLID = STL_LID_PERMISSIVE;
				}
				else
				{
					pSmpBlock->Block.SLID = pPortTbl[0].PortBlock->Public.Address.BaseLID |
											pSmpBlock->Block.PathBits;
				}
			}
			else
			{
				// we are not a switch
				if ( STL_LID_PERMISSIVE != pSmp->SmpExt.DirectedRoute.DrSLID)
				{
					_DBG_PRINT (_DBG_LVL_CALLBACK,(
						"Invalid DR [packet discarded per Check 5]\n"));
					goto discard;
				}
				pSmpBlock->Block.SLID = STL_LID_PERMISSIVE;
			}

			pSmpBlock->Block.DLID = pSmp->SmpExt.DirectedRoute.DrSLID;
			// Text in clause 3 has this additional test which is
			// redundant with test above due to DLID assignment above
			// for a CA DLID will be PERMISSIVE if get here
			if ( STL_LID_PERMISSIVE == pSmpBlock->Block.DLID )
			{
				// per path above, HopPointer is now HopCount+1
#ifdef IB_DEBUG
				ASSERT(pSmp->common.u.DR.HopPointer == 0);
#endif
				goto localsm;
			} else {
#ifdef  IB_DEBUG
				if (!(pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0))
				{
					// can't get here given sequence of tests above for a CA
					ASSERT(0);
				}
#endif
				goto sendsmp;
			}
			// Figure 166, SMP - Direct Route Check 6
		} else if ( 0 == pSmp->common.u.DR.HopPointer)
		{
			// SMP is to a local SM
			// C14-13 clause 4)
			goto localsm;
		} else {
			// means HopCount+2 <= HopPointer <= 255
        	_DBG_PRINT (_DBG_LVL_CALLBACK,(
                   "Invalid DR [packet discarded per Check 6]\n"));
			goto discard;
		}
	}

lidsmp:
    _DBG_RETURN_LVL(_DBG_LVL_CALLBACK, SMI_LID_ROUTED);
	return SMI_LID_ROUTED;

localsmp:
    _DBG_RETURN_LVL(_DBG_LVL_CALLBACK, SMI_TO_LOCAL);
	return SMI_TO_LOCAL;

localsm:
    _DBG_RETURN_LVL(_DBG_LVL_CALLBACK, SMI_TO_LOCAL_SM);
	return SMI_TO_LOCAL_SM;

sendsmp:
	// validate output port number
	// With the switch's built in SMA, we leave validation to the switch firmware
	// On the host we handle it here
	if (!(pCaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0) &&
		(pSmpBlock->Block.PortNumber < 1 || pSmpBlock->Block.PortNumber > pCaObj->CaPublic.Ports))
	{
		_DBG_PRINT (_DBG_LVL_CALLBACK,(
					"Invalid DR Port Number [packet discarded]\n"));
		goto discard;
	}

	_DBG_RETURN_LVL(_DBG_LVL_CALLBACK, SMI_SEND);
	return SMI_SEND;

discard:
    _DBG_RETURN_LVL(_DBG_LVL_CALLBACK, SMI_DISCARD);
	return SMI_DISCARD;
}

// Process an SMP which is routed to a local SMA or SM
// returns action to be performed with SmpBlock
// SMI_TO_LOCAL_SM - pass on to SM
// SMI_DISCARD - discard packet
// SMI_SEND - Smp should be sent on the wire
// SMI_TO_LOCAL - never returned by this function
// SMI_LID_ROUTED - LID routed SMP to SMA, send response back to source
SMI_ACTION
SmaProcessLocalSmp(
	IN	SMA_CA_OBJ_PRIVATE	*pCaObj,
	IN	POOL_DATA_BLOCK		*pSmpBlock
	)
{
	STL_SMP			*pSmp = (STL_SMP*)pSmpBlock->Block.Smp;

	_DBG_ENTER_LVL(_DBG_LVL_CALLBACK, SmaProcessLocalSmp);
	// M_Key checks are done in GetSetMad, so we defer
	// IBTA 1.1 Figure 158 SMP M_Key Check until we give to SMA
				
	// Sma Processes local Smps for the following:
	//	1) It a a request and not a response
	//	2) It is not a Trap (Trap does not have R bit set)
	//	3) It is not a SM_INFO

	// This is based on IBTA 1.1 Figure 159 SMP Check 2
	// with some minor changes to the flow

	if (( MMTHD_TRAP == pSmp->common.mr.s.Method)
		|| ( MCLASS_ATTRIB_ID_SM_INFO == pSmp->common.AttributeID )
		|| ( 0 != pSmp->common.mr.s.R ) 
		)
	{
		// Traps, SM_INFO and Responses are destined to SM
		// not destined to SMA, just pass to SM if any
        _DBG_PRINT (_DBG_LVL_CALLBACK,(
                    "Trap, SMInfo or Response to SM\n"));
    	_DBG_RETURN_LVL(_DBG_LVL_CALLBACK, SMI_TO_LOCAL_SM);
		return SMI_TO_LOCAL_SM;
	}

	if ((MMTHD_GET == pSmp->common.mr.s.Method)
		|| (MMTHD_SET == pSmp->common.mr.s.Method)
		|| (MMTHD_TRAP_REPRESS == pSmp->common.mr.s.Method))
	{
		SMI_ACTION SmiAction;

		// destined to SMA
		if ( FALSE == SmaProcessSmaSmpRcvd(pSmpBlock))
		{
			return SMI_DISCARD;
		}
		SmiAction = SmaProcessSmiSmp(pCaObj, pSmpBlock, FALSE);

		if (SMI_TO_LOCAL == SmiAction)
		{
			return SMI_TO_LOCAL_SM;
		}
    	_DBG_RETURN_LVL(_DBG_LVL_CALLBACK, SmiAction);
		return SmiAction;
	} else {
		// unexpected method - discard
        _DBG_PRINT (_DBG_LVL_CALLBACK,(
                    "Unexpected Method [packet discarded]\n"));
    	_DBG_RETURN_LVL(_DBG_LVL_CALLBACK, SMI_DISCARD);
		return SMI_DISCARD;
	}
}

// Add to list of completed Sends to be reported to SM
void
SmaAddToSendQ(
	IN	POOL_DATA_BLOCK		*SmpBlock
	)
{
	FSTATUS					status;
	SMP_LIST				*pSmpList;
	STL_SMP					*pSmp;
	
	//
	// first free AV handle
	//
	if( NULL != SmpBlock->AvHandle )
	{
		status = PutAV(
					(SMA_CA_OBJ_PRIVATE*)SmpBlock->Block.SmaCaContext,
					SmpBlock->AvHandle);
		SmpBlock->AvHandle = 0;
	}

	pSmpList = (SMP_LIST*)SmpBlock->Base;

	if ( NULL != pSmpList )
	{
		//
		// Increment recvd counter
		//
		AtomicIncrementVoid( &pSmpList->SmpsOut );


		// set error indicator.
		if ( FSUCCESS != SmpBlock->Block.Status )
			pSmpList->SmpsInError = 1;

		//
		// Byte order for user
		//
		BSWAP_MAD_HEADER( (MAD*)SmpBlock->Block.Smp);

		pSmp = (STL_SMP*)SmpBlock->Block.Smp;

#ifdef  ICS_LOGGING
		_DBG_PRINT (_DBG_LVL_CALLBACK,(
			"Completed Send Info:\n"
			"\tClass........:x%x\n"
			"\tMethod.......:x%X\n"
			"\tResponseBit..:x%X\n"	
			"\tAttribID.....:x%X\n"
			"\tAttribModfier;x%X\n"
			"\tTID..........:x%"PRIx64"\n",
			pSmp->common.MgmtClass,
			pSmp->common.mr.s.Method,
			pSmp->common.mr.s.R,
			pSmp->common.AttributeID,
			pSmp->common.AttributeModifier,
			pSmp->common.TransactionID
			));
		_DBG_PRINT (_DBG_LVL_CALLBACK,(
			"Completed Send Info:\n"
			"\tClass........:x%x\n"
			"\tMethod.......:x%X\n"
			"\tResponseBit..:x%X\n"	
			"\tAttribID.....:x%X\n"
			"\tHopCount.....:x%X\n"
			"\tHopPointer...:x%X\n",
			pSmp->common.MgmtClass,
			pSmp->common.mr.s.Method,
			pSmp->common.mr.s.R,
			pSmp->common.AttributeID,
			pSmp->common.u.DR.HopCount,
			pSmp->common.u.DR.HopPointer
			));
#else
		_DBG_PRINT (_DBG_LVL_CALLBACK,(
			"Completed Send Info:\n"
			"\tClass........:x%x\n"
			"\tMethod.......:x%X\n"
			"\tResponseBit..:x%X\n"	
			"\tAttribID.....:x%X\n"
			"\tAttribModfier;x%X\n"
			"\tTID..........:x%"PRIx64"\n"
			"\tHopCount.....:x%X\n"
			"\tHopPointer...:x%X\n",
			pSmp->common.MgmtClass,
			pSmp->common.mr.s.Method,
			pSmp->common.mr.s.R,
			pSmp->common.AttributeID,
			pSmp->common.AttributeModifier,
			pSmp->common.TransactionID,
			pSmp->common.u.DR.HopCount,
			pSmp->common.u.DR.HopPointer
			));
#endif	
		if ( pSmpList->SmpsIn == AtomicRead(&pSmpList->SmpsOut) )
		{

			//
			// signal caller?
			//

			if ( pSmpList->PostCallback )	// callback user
			{

				//
				// If caller does not want the status, add to bin
				//

				(pSmpList->PostCallback)(
								pSmpList->Context, 
								pSmpList->SmpBlock);
			
				//
				// Free list
				//

				MemoryDeallocate( pSmpList );
			}
			else
			{

				//
				// Do we have errors?
				//

				if ( 0 != pSmpList->SmpsInError )
				{
					if ( g_Sma->SmUserTbl )
					{
						//
						// we have an error ; call user
						//

						if ( g_Sma->SmUserTbl->DefRecv.EventCallback )
						{
							(g_Sma->SmUserTbl->DefRecv.EventCallback)(
								EVENT_SEND_ERROR,
								pSmpList->SmpBlock,
								pSmpList->Context);

							//
							// Free list
							//

							MemoryDeallocate( pSmpList );
							pSmpList = NULL;
						}
					}
				}

				//
				// No errors, pass buffer to Bin
				//

				if ( NULL != pSmpList )
				{
					status = iba_smi_pool_put( NULL, pSmpList->SmpBlock);

					//
					// Free list
					//

					MemoryDeallocate( pSmpList );
				}
			}		// callback
		}			// In == Out
	}
	else
	{
		//
		// Silently return/recycle SMP to pool
		//
		// We land here if it was an internal send or only one send
		//
		status = iba_smi_pool_put( NULL, (SMP_BLOCK *)SmpBlock);
	}
}


void
SmaAddToRecvQ(
	IN	POOL_DATA_BLOCK		*SmpBlock
	)
{

	SpinLockAcquire( &g_Sma->RQ.Lock );

	//
	// Handle common recv completion
	//

	if ( NULL != g_Sma->RQ.s.RECV.Tail )
	{
		SmpBlock->Block.Prev = g_Sma->RQ.s.RECV.Tail;
		g_Sma->RQ.s.RECV.Tail->Block.Next = SmpBlock;
		g_Sma->RQ.s.RECV.Tail = SmpBlock;
	}
	else
	{
		g_Sma->RQ.s.RECV.Head = g_Sma->RQ.s.RECV.Tail = SmpBlock;
	}

	g_Sma->RQ.Count++;

	SpinLockRelease( &g_Sma->RQ.Lock );
}


void
SmaProcessCQ(
	IN		SMA_CA_OBJ_PRIVATE	*CaObj,
	IN	OUT	RECV_BLOCK			*RecvBlock
)
{
	boolean						bRearmed		= FALSE;
	FSTATUS						status;
	POOL_DATA_BLOCK				*pSmpBlock;
	STL_SMP							*pSmp;
	IB_WORK_COMPLETION			workCompletion;
	SMA_CA_OBJ_PRIVATE			*pCaObj;
	SMA_PORT_TABLE_PRIV			*pCaPortTbl;
	uint8						PortNumber;
	SMI_ACTION					SmiAction = SMI_TO_LOCAL_SM;
	

	_DBG_ENTER_LVL(_DBG_LVL_CALLBACK, SmaProcessCQ);

	//
	// Processing completions on the CQ is done as follows:
	//	1. Repeatedly Poll till all entries are done. 
	//	2. Rearm CQ and Poll again. If there are no more completions bail out.
	//	3. If there are completions go back to step 1.
	//
	//	We follow these steps to optimize on hardware events and race 
	//	conditions.
	//
	
	while (1)
	{
		//
		// Loop and pull out all sends and receives
		//
		while ( FSUCCESS == (status = iba_poll_cq( CaObj->CqHandle, &workCompletion )) )
		{
			//
			// fill in common values
			//

			pSmpBlock = (POOL_DATA_BLOCK *)(uintn)MASK_WORKREQID(workCompletion.WorkReqId);
			
			ASSERT (pSmpBlock);

			if ( !pSmpBlock )
			{
				_DBG_ERROR (("Spurious CQ Event!!!\n"
							"\tCompleted WQE.....:(%s)\n"
							"\tSTATUS............:x%x\n"
							"\tLength............:x%x\n",
							_DBG_PTR(((WCTypeSend == workCompletion.WcType) ? "Send":"Recv")),
							workCompletion.Status,
							workCompletion.Length ));

				if ( WCTypeRecv == workCompletion.WcType )
				{
					_DBG_ERROR (("\n"
							"\tQP Number.........:x%x\n"
							"\tSourceLID.........:x%x\n",
							workCompletion.Req.RecvUD.SrcQPNumber,
							workCompletion.Req.RecvUD.SrcLID ));
				}

				_DBG_BREAK;
				break;
			}	

			pSmpBlock->Block.Status = workCompletion.Status;
			PortNumber = pSmpBlock->Block.PortNumber;
			

			_DBG_PRINT (_DBG_LVL_CALLBACK, (
				"Completed %s WQE (%p):\n"
				"\tSTATUS.......:x%X (%s)\n"
				"\tPort Number..:%d\n",
				_DBG_PTR((IS_SQ_WORKREQID(workCompletion.WorkReqId) ? "Send":"Recv")),
				_DBG_PTR(pSmpBlock),
				pSmpBlock->Block.Status, 
				_DBG_PTR(iba_fstatus_msg(pSmpBlock->Block.Status)),
				pSmpBlock->Block.PortNumber ));
		
			//
			// Is it Send?
			//
			if (IS_SQ_WORKREQID(workCompletion.WorkReqId))
			{
				//
				// Handle common send completion
				//
				SmaAddToSendQ(pSmpBlock);
			
			}
			else
			{
				boolean QpDestroy;

				//
				// Its a receive!
				//

				pSmpBlock->Block.SmpByteCount = workCompletion.Length;

				//
				// First update receive Q depth
				//
				pCaObj = (SMA_CA_OBJ_PRIVATE *)pSmpBlock->Block.SmaCaContext;
				pCaPortTbl = (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;
				if (CaObj->CaPublic.Capabilities & CA_IS_ENHANCED_SWITCH_PORT0)
				{
					if (pSmpBlock->Block.PortNumber != 1)
					{
#ifdef  ICS_LOGGING
						_DBG_WARNING(( "RecvQ Error: Bad Port Number of %d, should be 1\n",
									   pSmpBlock->Block.PortNumber));
#else
						_DBG_WARN(( "RecvQ Error: Bad Port Number of %d, should be 1\n",
									pSmpBlock->Block.PortNumber));
#endif
						pSmpBlock->Block.PortNumber = 1;
					}
				}
				QpDestroy = ( QP_DESTROY == pCaPortTbl[pSmpBlock->Block.PortNumber-1].PortBlock->Qp0Control );
				
				AtomicIncrementVoid(&pCaPortTbl[pSmpBlock->Block.PortNumber-1].PortBlock->Qp0RecvQTop);

				if (pSmpBlock->Block.Status != FSUCCESS)
				{
					// In destroy state we expect to get back CQEs with Flush Status
					if ( ! QpDestroy )
					{
#ifdef  ICS_LOGGING
						_DBG_WARNING(( "RecvQ Error: CQE Status %d\n",
							pSmpBlock->Block.Status));
#else
						_DBG_WARN(( "RecvQ Error: CQE Status %d\n",
							pSmpBlock->Block.Status));
#endif		
					}
    
					SmiAction = SMI_DISCARD;
					goto DoAction;
				}

				//
				// Fill in receive details
				//

				//
				// Port Number and Q pair are filled-up during the iba_smi_post_recv()
				//
				pSmpBlock->Block.SLID = workCompletion.Req.RecvUD.SrcLID;
				if (pSmpBlock->Block.SLID == LID_PERMISSIVE)
					pSmpBlock->Block.SLID = STL_LID_PERMISSIVE;
				pSmpBlock->Block.PathBits = workCompletion.Req.RecvUD.DestPathBits;
				pSmpBlock->Block.ServiceLevel = workCompletion.Req.RecvUD.ServiceLevel;
				// BUGBUG - need to get GRH header out of workCompletion

				// IB does not have Static rate in UD LRH
				// so we can't be sure what rate of remote port is
				// we use a constant value for SMI QPs
				pSmpBlock->Block.StaticRate = IB_STATIC_RATE_SMI;
				_DBG_PRINT (_DBG_LVL_CALLBACK, (
					"Inbound PathBits: %d\n",
					pSmpBlock->Block.PathBits));
				
				pSmp = (STL_SMP*)pSmpBlock->Block.Smp;
				//
				// Network Byte order block, make Host Byte Order
				//
				BSWAP_STL_SMP_HEADER(pSmp);



				_DBG_PRINT (_DBG_LVL_CALLBACK, (
					"Receive Info:\n"
					"\tClass........:x%x\n"
					"\tAttribID.....:x%X\n"
					"\tAttribModfier:x%X\n"
					"\tMethod.......:x%X\n"
					"\tResponseBit..:x%X\n"
					"\tHopCount.....:x%X\n",
					pSmp->common.MgmtClass,
					pSmp->common.AttributeID,
					pSmp->common.AttributeModifier,
					pSmp->common.mr.s.Method,
					pSmp->common.mr.s.R,
					pSmp->common.u.DR.HopCount
					));
				_DBG_PRINT (_DBG_LVL_CALLBACK, (
					"Receive Info (cont):\n"
					"\tHopPointer...:x%X\n"
					"\tTID..........:x%"PRIx64"\n"
					"\tSLID.........:x%X\n",
					pSmp->common.u.DR.HopPointer,
					pSmp->common.TransactionID,
					pSmpBlock->Block.SLID
					));

				// Trap with SLID 0 is a special case
				// Firmware has provided a trap to us on QP0
				// which we need to send out
				// use SmaProcessSmaSmpRcvd to allow the driver to complete
				// setup of the mad.  If driver does not want to send it
				// FNOT_DONE is returned
				if ( MMTHD_TRAP == pSmp->common.mr.s.Method
					  && 0 == pSmpBlock->Block.SLID)
				{
					// Let Driver fixup mad if needed
					if (FALSE == SmaProcessSmaSmpRcvd(pSmpBlock))
					{
						SmiAction = SMI_DISCARD;
					} else {
						// output on wire, assume its LID routed and valid
						// so no need to do SmaProcessSmiSmp DR algorithms
						// if its to our own LID (local SM), it will reappear
						// on our recv queue and get to SM that way
						SmiAction = SMI_SEND;
					}
				} else {
					// Validate incoming Smp per common Header
					// and Directed Route Checks
					// LID routed SMPs are destined to local SMA/SM
					SmiAction = SmaProcessSmiSmp(pCaObj, pSmpBlock, TRUE);

					if (SMI_TO_LOCAL == SmiAction
						|| SMI_LID_ROUTED == SmiAction)
					{
						SmiAction = SmaProcessLocalSmp(pCaObj, pSmpBlock);
						if (SMI_LID_ROUTED == SmiAction)
						{
							// LID routed responses sent back to requestor
							SmiAction = SMI_SEND;
						}
					}
				}

DoAction:
				if (SmiAction == SMI_SEND)
				{
					// output on wire, DR processing already completed
					pSmpBlock->Base = NULL;
					pSmpBlock->CQTrigger = FALSE;// Do not ask for interrupt. 
												// Flush SMP in next cycle

					if (FSUCCESS != SmaPostSmp( pCaObj, pSmpBlock ))
					{
						// unable to send
						SmiAction = SMI_DISCARD;
					}
				}

				if ( SMI_TO_LOCAL_SM == SmiAction )
				{
					// Add to recv Q for SM, else (no SM) return to pool
					if (( NULL != g_Sma->SmUserTbl )
						&& ( NULL != g_Sma->SmUserTbl->DefRecv.RcvCallback ) )
					{
						_DBG_PRINT (_DBG_LVL_CALLBACK,(
							"Adding SMP to RQ\n" ));
											
						if ( NULL == RecvBlock->Head )
						{
							RecvBlock->Head = RecvBlock->Tail = pSmpBlock;
						}
						else
						{
							RecvBlock->Tail->Block.Next = pSmpBlock;
							pSmpBlock->Block.Prev		= RecvBlock->Tail;
							RecvBlock->Tail				= pSmpBlock;
						}

						RecvBlock->Count++;		// keep count for quick q insertion
					}	
					else
					{
						SmiAction = SMI_DISCARD;
					}
				}

				if (SMI_DISCARD == SmiAction)
				{
					status = iba_smi_pool_put( NULL, (SMP_BLOCK *)pSmpBlock);
				}

				//
				// Post a SMP to recvQ to recv new msgs
				//
				// only post if not in process of destroying QP
				if ( workCompletion.Status != WRStatusWRFlushed && ! QpDestroy )
				{
					status = iba_smi_post_recv( NULL, pCaObj, PortNumber, 1);
					if ( FSUCCESS != status )
					{
						_DBG_ERROR (( "Could not post to RecvQ!!!\n" ));
					}
				}
			}			// recv

			SmiAction = SMI_TO_LOCAL_SM;

		}				// iba_poll_cq while loop


		if ( FNOT_DONE != status )
		{
			_DBG_ERROR (("Got %s Status!!!\n", _DBG_PTR(FSTATUS_MSG(status)) ));
		}
		
		//
		// Rearm CQ for race conditions
		//
		if ( FALSE == bRearmed )
		{
			status = iba_rearm_cq( CaObj->CqHandle, CQEventSelNextWC);
			bRearmed = TRUE;
		}
		else
		{
			break;
		}
	}			// while 1 for ReArm

    _DBG_LEAVE_LVL(_DBG_LVL_CALLBACK);

}

// When CaContext != NULL, we are in a thread context, as such we can preempt
// note that SmaProcessCQ can call GetSetMad in the VPD which might prempt
void 
SmaCompletionCallback(
	IN	void *CaContext
	)
{
	SMA_CA_OBJ_PRIVATE	*pCaObj;
	RECV_BLOCK			recvBlock;
	
	_DBG_ENTER_LVL(_DBG_LVL_CALLBACK, SmaCompletionCallback);

	pCaObj = (SMA_CA_OBJ_PRIVATE*)CaContext;

	// Init Recv context
	recvBlock.Head = recvBlock.Tail = NULL;
	recvBlock.Count = 0;

	//lookup request and fill in appropriate post set

	if ( NULL != pCaObj )
		SmaProcessCQ(pCaObj, &recvBlock);

	// We optimize on Lock usage by taking from RQ
	// then looking up client definitions

	SpinLockAcquire( &g_Sma->RQ.Lock );

	// Move contents of RQ to recvBlock
	if ( 0 != g_Sma->RQ.Count ) 
	{
		if (recvBlock.Count)
		{
			// add recvBlock contents at end of RQ
			g_Sma->RQ.s.RECV.Tail->Block.Next = recvBlock.Head;
			recvBlock.Head->Block.Prev = g_Sma->RQ.s.RECV.Tail;
			g_Sma->RQ.s.RECV.Tail = recvBlock.Tail;
		}

		// recvBlock gets all of combined RQ
		recvBlock.Head = g_Sma->RQ.s.RECV.Head;
		recvBlock.Tail = g_Sma->RQ.s.RECV.Tail;
		
		g_Sma->RQ.s.RECV.Head = g_Sma->RQ.s.RECV.Tail = NULL;

		recvBlock.Count += g_Sma->RQ.Count;
		g_Sma->RQ.Count = 0;
	}

	SpinLockRelease( &g_Sma->RQ.Lock );

	// Look up if we have receives and client defined
	// Else recycle buffers
	//

	// Process recvs, Distribute through User Callbacks

	if ( 0 != recvBlock.Count )
	{
		if ( ( NULL != g_Sma->SmUserTbl ) && \
			 ( NULL != g_Sma->SmUserTbl->DefRecv.RcvCallback ) )
		{
			_DBG_PRINT(_DBG_LVL_CALLBACK,
				("Received[x%x] for Callback[x%p] Context[x%p]\n",
				 recvBlock.Count,
				 _DBG_PTR(g_Sma->SmUserTbl->DefRecv.RcvCallback),
				 _DBG_PTR(g_Sma->SmUserTbl->DefRecv.Context)
				 ));
		
			// call user callback
			( g_Sma->SmUserTbl->DefRecv.RcvCallback )(
				g_Sma->SmUserTbl->DefRecv.Context,
				(SMP_BLOCK *)recvBlock.Head);
		} else {
			// recycle buffers
			SmaReturnToBin(recvBlock.Head);
		}
	}

    _DBG_LEAVE_LVL(_DBG_LVL_CALLBACK);
}




//
// The common completion event handler provided to the verbs consumer
//

void 
SmaGsaCompletionCallback(
	IN	void *CaContext, 
	IN	void *CqContext
	)
{
	SMA_CA_OBJ_PRIVATE	*pCaObj = (SMA_CA_OBJ_PRIVATE*)CaContext;
	
	_DBG_ENTER_LVL(_DBG_LVL_CALLBACK, SmaGsaCompletionCallback);

	if ( (void *)QPTypeSMI == CqContext )
	{
		EventThreadSignal(&pCaObj->SmaEventThread);
	} else if ( (void *)QPTypeGSI == CqContext ) {
		EventThreadSignal(&pCaObj->GsaEventThread);
	}
    _DBG_LEAVE_LVL(_DBG_LVL_CALLBACK);
}



//
// The asynchronous error handler provided to the verbs consumer
//

void
SmaAsyncEventCallback(
	IN	void				*CaContext, 
	IN IB_EVENT_RECORD		*EventRecord
	)
{
	SMA_CA_OBJ_PRIVATE		*pCaObj;
	SMA_PORT_TABLE_PRIV		*pPortTbl;

	_DBG_ENTER_LVL(_DBG_LVL_CALLBACK, SmaAsyncEventCallback);


	_DBG_PRINT(_DBG_LVL_CALLBACK,(\
		"Received Async Event:\n"
		"\tUserCaContext..: x%p\n"
		"\tContext........: x%p\n"
		"\tEventType......: x%x\n"
		"\tEventCode......: x%x\n",
		_DBG_PTR(CaContext),
		_DBG_PTR(EventRecord->Context),
		EventRecord->EventType,
		EventRecord->EventCode));


	pCaObj = (SMA_CA_OBJ_PRIVATE*)CaContext;
	pPortTbl = (SMA_PORT_TABLE_PRIV *)pCaObj->CaPublic.PortTbl;

	switch ( EventRecord->EventType )
	{
	default:
		break;
	
	case	AsyncEventQP:
		{
			_DBG_PRINT(_DBG_LVL_CALLBACK,("AsyncEventQP\n"));
			break;
		}

	case	AsyncEventSendQ:
		{
			_DBG_PRINT(_DBG_LVL_CALLBACK,("AsyncEventSendQ\n"));
			break;
		}

	case	AsyncEventRecvQ:
		{
			_DBG_PRINT(_DBG_LVL_CALLBACK,("AsyncEventRecvQ\n"));
			break;
		}

	case	AsyncEventCQ:
		{
			_DBG_PRINT(_DBG_LVL_CALLBACK,("AsyncEventCQ\n"));
			break;
		}

	case	AsyncEventCA:
		{
			_DBG_PRINT(_DBG_LVL_CALLBACK,("AsyncEventCA\n"));
			break;
		}

	case	AsyncEventPort:
		{
			uint8					portNo;
			IB_PORT_STATE			portState, oldPortState;
			SMA_PORT_BLOCK_PRIV		*pPortBlock;

			_DBG_PRINT(_DBG_LVL_CALLBACK,("AsyncEventPort\n"));

			//
			// context field = 1 based port number 
			// event type = port event
			// event code = port state
			//

			portNo = (uint8)((uintn)EventRecord->Context);
			portState = (IB_PORT_STATE)EventRecord->EventCode;

			pPortBlock = &pPortTbl->PortBlock[portNo-1];
			oldPortState = pPortBlock->Public.State;
			pPortBlock->Public.State = portState;

			/* ignore port state Async events while shutting down CA */
			if ( pPortBlock->Qp0Control != QP_DESTROY
				&& oldPortState != portState )
			{
				if ( IB_PORT_DOWN == portState )
				{
					IbtNotifyGroup(
						pPortBlock->Public.GUID,
						IB_NOTIFY_PORT_DOWN );	
				}

				if ( IB_PORT_ACTIVE == portState )
				{
					IbtNotifyGroup(
						pPortBlock->Public.GUID,
						IB_NOTIFY_LID_EVENT );	
				}
			}

			break;
		}

	case	AsyncEventPathMigrated:
		{
			_DBG_PRINT(_DBG_LVL_CALLBACK,("AsyncEventPathMigrated\n"));
			break;
		}
	}

    _DBG_LEAVE_LVL(_DBG_LVL_CALLBACK);
}

