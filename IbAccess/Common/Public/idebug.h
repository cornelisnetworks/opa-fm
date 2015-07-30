/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_PUBLIC_IDEBUG_H_
#define _IBA_PUBLIC_IDEBUG_H_

#include "iba/public/datatypes.h"
#include "iba/public/idebug_osd.h"


#ifdef __cplusplus
extern "C"
{
#endif

typedef enum { 
	_DBG_DUMP_FORMAT_BYTES,	/* dump bytes */
	_DBG_DUMP_FORMAT_U64,	/* dump 64 bit words, Host Byte Order */
	_DBG_DUMP_FORMAT_U32,	/* dump 32 bit words, Host Byte Order */
	_DBG_DUMP_FORMAT_U16,	/* dump 16 bit words, Host Byte Order */
	_DBG_DUMP_FORMAT_LE_U64,/* dump 64 bit words, Little Endian */
	_DBG_DUMP_FORMAT_LE_U32,/* dump 32 bit words, Little Endian */
	_DBG_DUMP_FORMAT_LE_U16,/* dump 16 bit words, Little Endian */
	_DBG_DUMP_FORMAT_BE_U64,/* dump 64 bit words, Big Endian */
	_DBG_DUMP_FORMAT_BE_U32,/* dump 32 bit words, Big Endian */
	_DBG_DUMP_FORMAT_BE_U16	/* dump 16 bit words, Big Endian */
} dbgDumpFormat_t;

#if defined( IB_DEBUG )
#ifndef DbgOut
#define DbgOut DbgOutRoutine
#endif
#else
#undef DbgOut
#define DbgOut(ARGLIST...) (void)(0)
#endif	/* defined( IB_DEBUG ) */

#ifndef MsgOut
extern void 
MsgOut( 
	IN char *Message, 
	IN ... );
#endif

void PrintDump( void* addr, uint32 len, dbgDumpFormat_t dumpFormat);
void PrintDump2( uint32 level, void* addr, uint32 len, dbgDumpFormat_t dumpFormat);

void DumpStack(void);

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IBA_PUBLIC_DEBUG_H_ */
