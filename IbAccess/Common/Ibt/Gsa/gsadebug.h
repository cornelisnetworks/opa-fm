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
/* [ICS VERSION STRING: unknown] */

// Suppress duplicate loading of this file
#ifndef _IBA_GSA_DEBUG_H_
#define _IBA_GSA_DEBUG_H_

#undef _ib_dbg_params
#define _ib_dbg_params Gsa_debug_params
#ifndef Gsa_debug_params
	extern IB_DBG_PARAMS	Gsa_debug_params;
#endif

#define GSA_MEM_TAG MAKE_MEM_TAG(G,s,a,M)

//
// Debug Levels
//
#define	_DBG_LVL_MAIN			0x00000001
#define	_DBG_LVL_CLASS			0x00000002
#define	_DBG_LVL_MGMT			0x00000004
#define	_DBG_LVL_OSD			0x00000008
#define	_DBG_LVL_DEVICE			0x00000010

#define	_DBG_LVL_POOL			0x00000020
#define	_DBG_LVL_STORE			0x00000040

#define	_DBG_LVL_SEND			0x00000080
#define	_DBG_LVL_RECV			0x00000100
#define	_DBG_LVL_LOCAL			0x00000200
#define	_DBG_LVL_CALLBACK		0x00000400
#define	_DBG_LVL_PKTDUMP		0x00000800

#define	_DBG_LVL_SAR_INFO		0x00004000


#define	_DBG_LVL_TEST			0x00010000

#ifdef IB_TRACE

#define	_TRC_LVL_STORE			0x00000040

#define _TRC_STORE(string) \
		_TRC_PRINT(_TRC_LVL_STORE, string)

#else

#define _TRC_STORE(string)

#endif

#endif // _IBA_GSA_DEBUG_H_
