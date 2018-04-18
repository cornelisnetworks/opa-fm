/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2017, Intel Corporation

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

#ifndef __OMGT_UTIL_DUMP_MAD__
#define __OMGT_UTIL_DUMP_MAD__

#include <stdio.h>
#include <stdint.h>

/**
 * dump MAD HEX data with minimal header decoding
 */
void omgt_dump_mad(FILE * file, uint8_t *buf, size_t size, const char *msg, ...);

/**
 * dump RAW HEX data
 */
void omgt_xdump(FILE *file, uint8_t *p, size_t size, int width);


/* Extends umad_class_str().
 * Extends umad_method_str(). 
 * Extends umad_attribute_str(). 
 *  
 * Checks BaseVersion == 128 (STL) else IB
 */
const char *stl_class_str(uint8_t BaseVersion, uint8_t class);
const char *stl_method_str(uint8_t BaseVersion, uint8_t class, uint8_t method);
const char *stl_mad_status_str(uint8_t BaseVersion, uint8_t class, uint16_t Status);
const char *stl_attribute_str(uint8_t BaseVersion, uint8_t class, uint16_t attr);
#endif /* __OMGT_UTIL_DUMP_MAD__ */
