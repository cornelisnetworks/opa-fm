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
#ifndef LOG_MESSAGES_H_INCLUDED
#define LOG_MESSAGES_H_INCLUDED

/*!
 @file    Log/Log_Messages.h
 @brief   Nationalizable Strings and Messages for Log Package generated from Log.msg
 */

#include "Gen_Arch.h"
#include "Gen_Macros.h"
#include "Log.h"

/* Messages */
#define LOG_MSG_TRUNCATED                 LOG_BUILD_MSGID(MOD_LOG, 0)
#define LOG_MSG_TRAP_FAILED               LOG_BUILD_MSGID(MOD_LOG, 1)

/* Logging Macros */
/* LOG_MSG_TRUNCATED */
#define LOG_LOG_FINAL_TRUNCATED()\
	LOG_WARNING(MOD_LOG, LOG_MSG_TRUNCATED, 0, 0, 0, 0, 0, 0)
/* LOG_MSG_TRAP_FAILED
 * Arguments: 
 *	erc
 */
#define LOG_LOG_FINAL_TRAP_FAILED(erc)\
	LOG_WARNING(MOD_LOG, LOG_MSG_TRAP_FAILED, erc, 0, 0, 0, 0, 0)

/* Constant Strings for use as substitutionals in Messages */
#define LOG_STR_MODNAME                   LOG_BUILD_STRID(MOD_LOG, 0)
#define LOG_STR_RESPONSE                  LOG_BUILD_STRID(MOD_LOG, 1)
#define LOG_STR_CORRECTION                LOG_BUILD_STRID(MOD_LOG, 2)
#define LOG_STR_TASK                      LOG_BUILD_STRID(MOD_LOG, 3)
#define LOG_STR_FILENAME                  LOG_BUILD_STRID(MOD_LOG, 4)
#define LOG_STR_LINE                      LOG_BUILD_STRID(MOD_LOG, 5)
#define LOG_STR_NULL                      LOG_BUILD_STRID(MOD_LOG, 6)
#define LOG_STR_NOMEMORY                  LOG_BUILD_STRID(MOD_LOG, 7)
#define LOG_STR_SEV_DUMP                  LOG_BUILD_STRID(MOD_LOG, 8)
#define LOG_STR_SEV_FATAL                 LOG_BUILD_STRID(MOD_LOG, 9)
#define LOG_STR_SEV_ERROR                 LOG_BUILD_STRID(MOD_LOG, 10)
#define LOG_STR_SEV_ALARM                 LOG_BUILD_STRID(MOD_LOG, 11)
#define LOG_STR_SEV_WARNING               LOG_BUILD_STRID(MOD_LOG, 12)
#define LOG_STR_SEV_PARTIAL               LOG_BUILD_STRID(MOD_LOG, 13)
#define LOG_STR_SEV_CONFIG                LOG_BUILD_STRID(MOD_LOG, 14)
#define LOG_STR_SEV_PROGRAM_INFO          LOG_BUILD_STRID(MOD_LOG, 15)
#define LOG_STR_SEV_PERIODIC_INFO         LOG_BUILD_STRID(MOD_LOG, 16)
#define LOG_STR_SEV_DEBUG1                LOG_BUILD_STRID(MOD_LOG, 17)
#define LOG_STR_SEV_DEBUG2                LOG_BUILD_STRID(MOD_LOG, 18)
#define LOG_STR_SEV_DEBUG3                LOG_BUILD_STRID(MOD_LOG, 19)
#define LOG_STR_SEV_DEBUG4                LOG_BUILD_STRID(MOD_LOG, 20)
#define LOG_STR_SEV_DEBUG5                LOG_BUILD_STRID(MOD_LOG, 21)
#define LOG_STR_SEV_PURGE                 LOG_BUILD_STRID(MOD_LOG, 22)

GEN_EXTERNC(extern void Log_AddMessages();)

#endif /* LOG_MESSAGES_H_INCLUDED */
