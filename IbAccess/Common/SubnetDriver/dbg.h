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

#ifndef _IBA_SD_DBG_H_
#define _IBA_SD_DBG_H_

#include <idebug.h>
#include <datatypes.h>
#include <stl_sd.h>

// define boolean 
#define TRUE	1
#define FALSE	0

#ifndef Ib_SubnDrv_Dbg
#undef _ib_dbg_params
#define _ib_dbg_params Ib_SubnDrv_Dbg
#endif

// The following debug macros must be defined before including the "ib_debug.h"
#define	_DBG_PRINT_EPILOG(LEVEL,STRING)
#define	_DBG_PRINT_PROLOG(LEVEL,STRING)
#define	_DBG_ERROR_EPILOG(LEVEL,STRING)
#define	_DBG_ERROR_PROLOG(LEVEL,STRING)
#define	_DBG_FATAL_EPILOG(LEVEL,STRING)
#define	_DBG_FATAL_PROLOG(LEVEL,STRING)


#define _DBG_CHK_IRQL(IRQL)

#include "ib_debug.h"

#if defined(DBG) ||defined( IB_DEBUG)

//
// We want to be able to control debug output based on global flags as well
// as per client debug flags that over-ride the global debug flags. This is
// why we need to take debug flags as input to this macro
//
#define DebugPrint(_DbgFlags_,_l_,_x_) \
            if (_DbgFlags_ & (_l_)) { \
               DbgOut("IBSd: "); \
               DbgOut _x_; \
            }

//
// Debugging Output Levels
//

#define _SD_ERROR(_DbgFlags_,_x_)								\
	if ( _DbgFlags_ & SD_DBG_ERROR) 								\
	{															\
		DbgOut("%s%s() !ERROR! ", _DBG_PTR(__MOD_NAME__),_DBG_PTR(__FUNC__));		\
		DbgOut _x_;												\
	}


#define _SD_INFO(_DbgFlags_,_x_)								\
	if ( _DbgFlags_ & SD_DBG_INFO) 								\
	{															\
		DbgOut("%s%s() !INFO! ", _DBG_PTR(__MOD_NAME__),_DBG_PTR(__FUNC__));		\
		DbgOut _x_;												\
	}		

#define _SD_TRACE(_DbgFlags_,_x_)								\
	if ( _DbgFlags_ & SD_DBG_TRACE) 								\
	{															\
		DbgOut("%s%s() !TRACE! ", _DBG_PTR(__MOD_NAME__),_DBG_PTR(__FUNC__));		\
		DbgOut _x_;												\
	}		

#else // !((_DEBUG) || (DBG))

#define DebugPrint(_DbgFlags_,_l_,_x_)
#define _SD_ERROR(_DbgFlags_,_x_)
#define _SD_INFO(_DbgFlags_,_x_)
#define _SD_TRACE(_DbgFlags_,_x_)

#endif  // (_DEBUG) || (DBG)

#endif // _IBA_SD_DBG_H_

