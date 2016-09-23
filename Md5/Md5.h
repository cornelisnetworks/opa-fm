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

#ifndef MD5_H_INCLUDED
#define MD5_H_INCLUDED
/*!
 @file    $Source$
 $Name$
 $Revision$
 $Date$
 @brief   Md5 API
 */

#ifdef __linux__
#include <stdint.h>
#define U8_t  uint8_t
#define U32_t uint32_t
#else
#include <Gen_Types.h>
#endif

#ifndef GEN_EXTERNC
#ifdef __cplusplus
#define GEN_EXTERNC(decl) extern "C" { decl }
#else
#define GEN_EXTERNC(decl) decl
#endif
#endif

#define MD5_USE_DEUTSCH_IMPLEMENTATION


#if defined(MD5_USE_RIVEST_IMPLEMENTATION)

#include "Md5_Rivest.h"

typedef MD5_CTX Md5_Context_t;


#elif defined(MD5_USE_DEUTSCH_IMPLEMENTATION)


#include "Md5_Deutsch.h"

typedef md5_state_t Md5_Context_t;


#endif /* MD5_USE_XXX_IMPLEMENTATION */


GEN_EXTERNC( extern void Md5 (U8_t *buffer, U32_t bufferLength, U8_t digest[16]); );
GEN_EXTERNC( extern void Md5_Start (Md5_Context_t *md5Context); );
GEN_EXTERNC( extern void Md5_Finish (Md5_Context_t *md5Context, U8_t digest[16]); );
GEN_EXTERNC( extern void Md5_Update (Md5_Context_t *md5Context, U8_t *buffer, U32_t bufferLength); );


#endif /* MD5_H_INCLUDED */
