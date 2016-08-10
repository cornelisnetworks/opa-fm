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
/*!

  \file sdrequest.h

  $Revision$
  $Date$

  \brief Routines for handling requests forwarded by the subnet manager
*/

#include <sdrequest.h>
#include <ib_generalServices.h>
#include <dbg.h>
#include <ib_debug.h>
#include <report.h>

/*!
  \brief Attempts to process a Request received by the GSA
*/
FSTATUS 
ProcessRequest( void *ServiceContext, IBT_DGRM_ELEMENT *pDgrmList )
{
	FSTATUS Status = FSUCCESS;
	SA_MAD  *pMad = (SA_MAD *)GsiDgrmGetRecvMad(pDgrmList);
        
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE, ProcessRequest);

	switch ( pMad->common.MgmtClass )
	{
		case MCLASS_SUBN_ADM:
			switch ( pMad->common.mr.AsReg8 )
			{
				case MMTHD_REPORT:
					Status = ProcessReport( ServiceContext, pDgrmList );
					break;

				default:
				case MMTHD_GET:
				case MMTHD_SET:
				case MMTHD_SEND:
				case MMTHD_TRAP:
				case MMTHD_TRAP_REPRESS:
				case MMTHD_GET_RESP:
				case MMTHD_REPORT_RESP:
					break;
			}
			break;

		default:
		case MCLASS_SM_LID_ROUTED:
		case MCLASS_SM_DIRECTED_ROUTE:
		case MCLASS_PERF:
		case MCLASS_BM:
		case MCLASS_DEV_MGT:
		case MCLASS_COMM_MGT:
		case MCLASS_SNMP:
		case MCLASS_DEV_CONF_MGT:
		case MCLASS_DTA:
			break;
	}
	
	_DBG_LEAVE_LVL( _DBG_LVL_FUNC_TRACE );
	return Status;
}
