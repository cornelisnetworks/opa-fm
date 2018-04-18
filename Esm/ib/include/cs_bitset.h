/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

//===========================================================================//
//
// FILE NAME							
//    cs_bitset.h					
//
// DESCRIPTION					
//    Implement a generic bitset structure.
//				
// DATA STRUCTURES
//    None
//
// FUNCTIONS
//    None
//
// DEPENDENCIES
//    None
//
//									  
//===========================================================================//

#ifndef	_CS_BITSET_H_
#define	_CS_BITSET_H_

#include <stdio.h>
#include <signal.h>
#include "vs_g.h"
#include "ib_const.h"
#include "ib_types.h"

typedef struct {
	size_t nwords_m;
	size_t nbits_m;
	uint32_t *bits_m;
	size_t nset_m;
	Pool_t* pool_m;
} bitset_t;

/* Initialize a bitset struct large enough to hold nbits. */
int bitset_init(Pool_t *, bitset_t *, size_t nbits);

/* Like bitset_init() only the bitset_t is assumed to be valid when
   called, so the existing memory used by the bitset if freed after
   the bitset is resized */
int bitset_resize(bitset_t *, size_t nbits);

/* Destroy a bitset. */
void bitset_free(bitset_t *);

/* Clear all bits (default) */
void bitset_clear_all(bitset_t *);

/* Set all bits */
void bitset_set_all(bitset_t *);

/* Copy one bitset to the other */
void bitset_copy(bitset_t *, bitset_t *);

/* Clear a specific bit (0..nbits-1). Returns -1 on error */
int bitset_clear(bitset_t *, unsigned bit);

/* Set a specific bit, Returns -1 on error */
int bitset_set(bitset_t *, unsigned bit);

/* Test a specific bit. Result is undefined if bit >= nbits() */
int bitset_equal(bitset_t*, bitset_t*);

/* Test a specific bit. Result is undefined if bit >= nbits() */
int bitset_test(bitset_t *, unsigned bit);

/* Find the first one bit. Returns -1 if not found */
int bitset_find_first_one(bitset_t *);

/* Find the next one bit. */
int bitset_find_next_one(bitset_t *, unsigned bit);

/* Find the last one bit. */
int bitset_find_last_one(bitset_t *);

/* Find the first zero bit. Returns -1 if not found */
int bitset_find_first_zero(bitset_t *);

/* Find the next zero bit past the given bit */
int bitset_find_next_zero(bitset_t *, unsigned bit);

/* Returns -1 if not found */
int bitset_find_last_zero(bitset_t *);

/* Return the number of bits in the bitset */
size_t bitset_nbits(bitset_t *);

/* Return the number of one bits in the bitset. */
size_t bitset_nset(bitset_t *); 

/* Returns 1 if logical and of the bitsets has at least one bit set,
 * returns 0 if bitsets are different sizes or logical and is 0 */
int bitset_test_intersection(bitset_t * a, bitset_t * b);

/* Sets result to the bitwise and of bitsets a and b
 * Returns the number of bits set in the result
 * or -1 if the bitsets aren't allocated or differ in size */
int bitset_set_intersection(bitset_t * a, bitset_t * b, bitset_t * result);

/* Display a human readable representation of the bitset.
   This uses log level INFINI_INFO and should probably only
   be called under debug mode. */
void bitset_info_log(bitset_t*, char* prelude);

#endif	// _CS_BITSET_H_


