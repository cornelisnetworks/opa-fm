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
#ifndef ERR_MESSAGES_H_INCLUDED
#define ERR_MESSAGES_H_INCLUDED

/*!
 @file    Err/Err_Messages.h
 @brief   Nationalizable Strings and Messages for Err Package generated from Err.msg
 */

#include "Gen_Arch.h"
#include "Gen_Macros.h"
#include "Log.h"


/* Constant Strings for use as substitutionals in Messages */
#define ERR_STR_MODNAME                   LOG_BUILD_STRID(MOD_ERR, 0)
#define ERR_STR_OK                        LOG_BUILD_STRID(MOD_ERR, 1)
#define ERR_STR_FAILED_CONSTRUCTOR        LOG_BUILD_STRID(MOD_ERR, 2)
#define ERR_STR_END_OF_DATA               LOG_BUILD_STRID(MOD_ERR, 3)
#define ERR_STR_IO_ERROR                  LOG_BUILD_STRID(MOD_ERR, 4)
#define ERR_STR_INVALID                   LOG_BUILD_STRID(MOD_ERR, 5)
#define ERR_STR_MISSING                   LOG_BUILD_STRID(MOD_ERR, 6)
#define ERR_STR_DUPLICATE                 LOG_BUILD_STRID(MOD_ERR, 7)
#define ERR_STR_OVERFLOW                  LOG_BUILD_STRID(MOD_ERR, 8)
#define ERR_STR_TIMEOUT                   LOG_BUILD_STRID(MOD_ERR, 9)
#define ERR_STR_UNDERFLOW                 LOG_BUILD_STRID(MOD_ERR, 10)
#define ERR_STR_OUT_OF_RESOURCES          LOG_BUILD_STRID(MOD_ERR, 11)
#define ERR_STR_UNKNOWN                   LOG_BUILD_STRID(MOD_ERR, 12)
#define ERR_STR_BAD                       LOG_BUILD_STRID(MOD_ERR, 13)

GEN_EXTERNC(extern void Err_AddMessages();)

#endif /* ERR_MESSAGES_H_INCLUDED */
