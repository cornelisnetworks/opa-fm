/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */
#ifndef OSA_MESSAGES_H_INCLUDED
#define OSA_MESSAGES_H_INCLUDED

/*!
 @file    Osa/Osa_Messages.h
 @brief   Nationalizable Strings and Messages for Osa Package generated from Osa.msg
 */

#include "Gen_Arch.h"
#include "Gen_Macros.h"
#include "Log.h"

/* Messages */
#define OSA_MSG_ASSERT_FAILED             LOG_BUILD_MSGID(MOD_OSA, 0)

/* Logging Macros */
/* OSA_MSG_ASSERT_FAILED
 * Arguments: 
 *	expr: text for failed expression
 */
#define OSA_LOG_FINAL_ASSERT_FAILED(expr)\
	LOG_FATAL(MOD_OSA, OSA_MSG_ASSERT_FAILED, expr, 0, 0, 0, 0, 0)

/* Constant Strings for use as substitutionals in Messages */
#define OSA_STR_MODNAME                   LOG_BUILD_STRID(MOD_OSA, 0)

GEN_EXTERNC(extern void Osa_AddMessages();)

#endif /* OSA_MESSAGES_H_INCLUDED */
