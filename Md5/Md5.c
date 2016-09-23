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

/*!
 @file    $Source$
 $Name$
 $Revision$
 $Date$
 @brief   Md5 API
 */



#include "Md5.h"


void Md5_Start (Md5_Context_t *md5Context) {

#if defined(MD5_USE_RIVEST_IMPLEMENTATION)

  MD5Init(md5Context);

#elif defined(MD5_USE_DEUTSCH_IMPLEMENTATION)

  md5_init(md5Context);

#endif

}



void Md5_Update (Md5_Context_t *md5Context, U8_t *buffer, U32_t bufferLength) {

#if defined(MD5_USE_RIVEST_IMPLEMENTATION)

  MD5Update(md5Context, buffer, bufferLength);

#elif defined(MD5_USE_DEUTSCH_IMPLEMENTATION)

  md5_append(md5Context, buffer, bufferLength);

#endif

}



void Md5_Finish (Md5_Context_t *md5Context, U8_t digest[16]) {

#if defined(MD5_USE_RIVEST_IMPLEMENTATION)
  int i;

  MD5Final(md5Context);

  for(i=0; i<16; i++) {
	digest[i] = md5Context->digest[i];
  }

#elif defined(MD5_USE_DEUTSCH_IMPLEMENTATION)

  md5_finish(md5Context, digest);

#endif

}



void Md5 (U8_t *buffer, U32_t bufferLength, U8_t digest[16]) {

  Md5_Context_t ctx;

  Md5_Start(&ctx);
  Md5_Update(&ctx, buffer, bufferLength);
  Md5_Finish(&ctx, digest);

}
