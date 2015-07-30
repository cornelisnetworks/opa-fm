/* BEGIN_ICS_COPYRIGHT6 ****************************************

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

** END_ICS_COPYRIGHT6   ****************************************/
/* [ICS VERSION STRING: unknown] */

// Suppress duplicate loading of this file
#ifndef _INTEL_VP_DEBUG_H_
#define _INTEL_VP_DEBUG_H_


#define	_DBG_PRINT_EPILOG(LEVEL,STRING)
#define	_DBG_PRINT_PROLOG(LEVEL,STRING)
#define	_DBG_ERROR_EPILOG(LEVEL,STRING)
#define	_DBG_ERROR_PROLOG(LEVEL,STRING)
#define	_DBG_FATAL_EPILOG(LEVEL,STRING)
#define	_DBG_FATAL_PROLOG(LEVEL,STRING)

#include "ib_debug_osd.h"

#if defined(IB_DEBUG) ||defined( DBG)	// Define file contents only on checked builds

//
// Range reserved for component specific values is 0x00000001 - 0x00FFFFFF
//
#define _DBG_LVL_CALL			((uint32)0x00000001)	// exported call interface functions
#define _DBG_LVL_EVENT			((uint32)0x00000002)	// Event Queue handling control path
#define _DBG_LVL_INTR			((uint32)0x00000004)	// Interrupt handler
#define _DBG_LVL_TPT			((uint32)0x00000008)	// TPT management
//
#define	_DBG_LVL_MEM_ALLOC		((uint32)0x00000010)
#define	_DBG_LVL_TIMERS			((uint32)0x00000020)
#define	_DBG_LVL_SMP			((uint32)0x00000040)	// SMP processing
#define	_DBG_LVL_MAD			((uint32)0x00000080)	// MAD processing
//
#define _DBG_LVL_PNP			((uint32)0x00000100)	// Plug and Play decisions
#define _DBG_LVL_POWER			((uint32)0x00000200)	// Power management decisions
#define _DBG_LVL_SYSTEM_CONTROL	((uint32)0x00000400)	// System control decisions

#define _DBG_ENTER	_DBG_ENTER_EXT
#define _DBG_LEAVE	_DBG_LEAVE_EXT

#define _DBG_CALL(STRING) \
	_DBG_PRINT (_DBG_LVL_CALL, STRING);

#define _DBG_EVENT(STRING) \
	_DBG_PRINT (_DBG_LVL_EVENT, STRING);

#define _DBG_INTR(STRING) \
	_DBG_PRINT (_DBG_LVL_INTR, STRING);

#define _DBG_TPT(STRING) \
	_DBG_PRINT (_DBG_LVL_TPT, STRING);

#define _DBG_MEM_ALLOC(STRING) \
	_DBG_PRINT (_DBG_LVL_MEM_ALLOC, STRING);

#define _DBG_PNP(STRING) \
	_DBG_PRINT (_DBG_LVL_PNP, STRING);

#define _DBG_POWER(STRING) \
	_DBG_PRINT (_DBG_LVL_POWER, STRING);

#define _DBG_SYSCTL(STRING) \
	_DBG_PRINT (_DBG_LVL_SYSTEM_CONTROL, STRING);

#else // _DEBUG || DBG

#include "ib_debug.h"

#define _DBG_LVL_CALL 0
#define _DBG_LVL_EVENT 0
#define _DBG_LVL_INTR 0
#define _DBG_LVL_TPT 0
#define	_DBG_LVL_MEM_ALLOC 0
#define	_DBG_LVL_TIMERS 0
#define	_DBG_LVL_SMP 0
#define	_DBG_LVL_MAD 0
#define _DBG_LVL_PNP 0
#define _DBG_LVL_POWER 0
#define _DBG_LVL_SYSTEM_CONTROL 0

#define _DBG_PNP(STRING)
#define _DBG_POWER(STRING)
#define _DBG_SYSCTL(STRING)
#define _DBG_CALL(STRING)
#define _DBG_EVENT(STRING)
#define _DBG_INTR(STRING)
#define _DBG_TPT(STRING)

#endif // _DEBUG || DBG


#endif // _INTEL_VP_DEBUG_H_
