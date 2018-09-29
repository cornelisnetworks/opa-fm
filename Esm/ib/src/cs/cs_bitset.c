/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2018, Intel Corporation

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

//===========================================================================//
//									                                         //
// FILE NAME								                                 //
//    cs_bitset.c							                                 //
//									                                         //
// DESCRIPTION								                                 //
//    Bitset handling methods.                                               //
//									                                         //
// DATA STRUCTURES							                                 //
//    None								                                     //
//                                                                           //
// DEPENDENCIES								                                 //
//    cs_bitset.h							                                 //
//									                                         //
//									                                         //
//===========================================================================//	   

#include "os_g.h"
#include "cs_log.h"
#include "ib_status.h"
#include "cs_bitset.h"

size_t bitset_nset(bitset_t *bitset) {
	return bitset->nset_m;
}

static void _bitset_mask_off_unused_bits(bitset_t *bitset) {

	uint32_t bits_left = bitset->nbits_m % 32;
	if (bits_left) {
		// Mask off the unused bits
		uint32_t mask = (1 << bits_left) - 1;
		bitset->bits_m[bitset->nwords_m-1] &= mask;
	}
}

int bitset_init(Pool_t* pool, bitset_t *bitset, size_t nbits) {
    Status_t	status;

	bitset->pool_m = pool;
	bitset->bits_m = NULL;
	bitset->nset_m = 0;
	bitset->nbits_m = nbits;
	bitset->nwords_m = (nbits/32);
	if (nbits%32) {
		bitset->nwords_m++;
	}
	status = vs_pool_alloc(pool, sizeof(uint32_t)*bitset->nwords_m, (void *)&bitset->bits_m);
	if (status != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't allocate space for bitset, rc:", status);
		memset(bitset, 0, sizeof(bitset_t));
		return 0;
	}
	memset(bitset->bits_m, 0, sizeof(uint32_t)*bitset->nwords_m);
	return 1;
}

size_t count_nset(bitset_t* bitset) {
	size_t bits_left = bitset->nbits_m;
	size_t i = 0;
	size_t nset = 0;

	if (!bitset->bits_m) {
		IB_LOG_INFINI_INFO0("bad bits");
		return 0;
	}
	for (i = 0; bits_left >= 32; i++, bits_left -= 32) {
		uint32_t word = bitset->bits_m[i];
		nset += __builtin_popcount(word);
	}
	if (bits_left > 0) {
		uint32_t mask = (1 << bits_left) - 1;
		nset += __builtin_popcount(bitset->bits_m[i] & mask);
		//bits_left = 0;
	}
	return nset;
}


int bitset_resize(bitset_t *bitset, size_t n) {
    Status_t	status;
	size_t orig_nbits = bitset->nbits_m;
	size_t orig_nset = bitset->nset_m;
	size_t orig_nwords = bitset->nwords_m;
	uint32_t *orig_bits = bitset->bits_m;

	if (n == bitset->nbits_m) return 1;

	bitset->bits_m = NULL;

	if (!bitset_init(bitset->pool_m, bitset, n)) {
		bitset->nbits_m = orig_nbits;
		bitset->nset_m = orig_nset;
		bitset->nwords_m = orig_nwords;
		bitset->bits_m = orig_bits;
		return 0;
	}

	if (!orig_bits) return 1;

	if (n > orig_nbits) {
		memcpy(bitset->bits_m, orig_bits, orig_nwords*sizeof(uint32_t));
		bitset->nset_m = orig_nset;
	} else {
		memcpy(bitset->bits_m, orig_bits, bitset->nwords_m*sizeof(uint32_t));
		_bitset_mask_off_unused_bits(bitset);
		bitset->nset_m = count_nset(bitset);
	}

	if ((status = vs_pool_free(bitset->pool_m, orig_bits)) != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't free allocated space for bitset, rc:", status);
	}
	return 1;
}

void bitset_free(bitset_t *bitset) {
    Status_t	status;

	if (bitset->bits_m) {
		if ((status = vs_pool_free(bitset->pool_m, bitset->bits_m)) != VSTATUS_OK) {
			IB_LOG_ERRORRC("can't free allocated space for bitset, rc:", status);
		}
	}
    memset(bitset, 0, sizeof(bitset_t));
}

void bitset_clear_all(bitset_t *bitset) {
	if (bitset->bits_m) {
		memset(bitset->bits_m, 0, sizeof(uint32_t)*bitset->nwords_m);
	}
	bitset->nset_m = 0;
}

void bitset_set_all(bitset_t *bitset) {
	if (bitset->bits_m) {
		memset(bitset->bits_m, 0xff, sizeof(uint32_t)*bitset->nwords_m);
		bitset->nset_m = bitset->nbits_m;
		_bitset_mask_off_unused_bits(bitset);
	}
}

void bitset_copy(bitset_t *bitset1, bitset_t *bitset2) {

	if (!bitset1->bits_m) return;
	if (!bitset2->bits_m) return;

	if (bitset1->nbits_m != bitset2->nbits_m) {
		bitset_resize(bitset1, bitset2->nbits_m);
	}

	if (bitset1->nbits_m != bitset2->nbits_m) return;

	memcpy(bitset1->bits_m, bitset2->bits_m, sizeof(uint32_t)*bitset1->nwords_m);
	bitset1->nset_m = bitset2->nset_m;
}

int bitset_clear(bitset_t *bitset, unsigned bit) {
	int rc = 0;

	if (bitset->bits_m && (bit < bitset->nbits_m)) {
		if ((bitset->bits_m[bit/32] & (1<<bit%32)) != 0) {
			bitset->bits_m[bit/32] ^= (1<<bit%32);
			bitset->nset_m--;
		}
		rc = 1;
	}
	return rc;
}

int bitset_set(bitset_t *bitset, unsigned bit) {
	int rc = 0;

	if (bitset->bits_m && (bit < bitset->nbits_m)) {
		if ((bitset->bits_m[bit/32] & (1<<bit%32)) == 0) {
			bitset->bits_m[bit/32] |= (1<<bit%32);
			bitset->nset_m++;
		}
		rc = 1;
	}
	return rc;
}

int bitset_test(bitset_t *bitset, unsigned bit) {
	int rc = 0;

	if (bitset->bits_m && (bit < bitset->nbits_m)) {
		rc = (bitset->bits_m[bit/32] & (1<<bit%32)) != 0;
	}
	return rc;
}

int bitset_equal(bitset_t* a, bitset_t* b) {
	int word;
	if ((a->nbits_m != b->nbits_m) ||
		(a->nset_m != b->nset_m) ||
		(a->nwords_m != b->nwords_m) ||
		!a->bits_m ||
		!b->bits_m) {
		return 0;
	}

	for (word = 0; word < a->nwords_m; word++) {
		if (a->bits_m[word] != b->bits_m[word]) return 0;
	}
	return 1;
}


int bitset_find_first_one(bitset_t *bitset) {
	return bitset_find_next_one(bitset, 0);
}

int bitset_find_next_one(bitset_t *bitset, unsigned bit) {

	if (bitset && bitset->bits_m && (bit < bitset->nbits_m)) {
		unsigned i_word;
		uint32_t mask = (1 << (bit % 32)) - 1;
		unsigned bits = bitset->nbits_m % 32;
		uint32_t word;
		for (i_word = bit / 32; i_word < bitset->nwords_m; i_word++) {
			if (i_word == (bitset->nwords_m - 1) && bits != 0) {
				/* Handle last Word if not 32bits */
				mask |= ~((1 << bits) -1);
			}
			word = bitset->bits_m[i_word] & ~mask;
			if (word != 0) {
				return (i_word * 32) + __builtin_ctz(word);
			}
			mask = 0;
		}
	}
	return -1;
}

int bitset_find_last_one(bitset_t *bitset) {

	if (bitset && bitset->bits_m) {
		unsigned i_word;
		unsigned bits = bitset->nbits_m % 32;
		uint32_t mask = (uint32_t)(-1);
		if (bits) mask = (1 << bits) - 1;
		for (i_word = bitset->nwords_m-1; i_word >= 0; i_word--) {
			uint32_t word = bitset->bits_m[i_word] & mask;
			if (word) {
				return (i_word * 32) + (32 - 1 - __builtin_clz(word));
			}
			mask = (uint32_t)(-1);
		}
	}
	return -1;
}

int bitset_find_first_zero(bitset_t *bitset) {
	return bitset_find_next_zero(bitset, 0);
}

int bitset_find_next_zero(bitset_t *bitset, unsigned bit) {

	if (bitset && bitset->bits_m && (bit < bitset->nbits_m)) {
		unsigned i_word;
		uint32_t mask = (1 << (bit % 32)) - 1;
		unsigned bits = bitset->nbits_m % 32;
		uint32_t word;
		for (i_word = bit/32; i_word < bitset->nwords_m; i_word++) {
			if (i_word == (bitset->nwords_m - 1) && bits != 0) {
				/* Handle last Word if not 32bits */
				mask |= ~((1 << bits) -1);
			}
			word = bitset->bits_m[i_word] | mask;
			if (word != (uint32_t)(-1)) {
				return (i_word * 32) + __builtin_ctz(~word);
			}
			mask = 0;
		}
	}
	return -1;
}

int bitset_find_last_zero(bitset_t *bitset) {

	if (bitset && bitset->bits_m) {
		unsigned i_word;
		unsigned bits = bitset->nbits_m % 32;
		uint32_t mask = 0;
		if (bits) mask = (1 << bits) - 1;
		else bits = 32;
		for (i_word = bitset->nwords_m - 1; i_word >= 0; i_word--) {
			uint32_t word = bitset->bits_m[i_word] | mask;
			if (word != (uint32_t)(-1)) {
				return (i_word * 32) + (bits - 1 - __builtin_clz(~word));
			}
			bits = 32;
			mask = 0;
		}
	}
	return -1;
}

size_t bitset_nbits(bitset_t *bitset) {
	return bitset->nbits_m;
}

//
//  This uses log level INFINI_INFO and should probably only
//  be called under debug mode.
//
void bitset_info_log(bitset_t* bitset, char* prelude) {
	char*	string = NULL;
	int		first = 1;
	int		range = 0;
	int		range_start = -1;
	int		prev = -1;
	int		bit = -1;
	size_t	max_str_len = bitset->nset_m*5+1;
	size_t	pos = 0;
	int		res = 0;
    Status_t	status;
	
	if (!bitset) return;

	if (bitset->bits_m == NULL) {
		IB_LOG_INFINI_INFO_FMT( __func__, "NOBITS");
		return;
	}

	if (bitset->nset_m == 0) {
		if (prelude) {
			IB_LOG_INFINI_INFO_FMT(__func__, "%s <nil>", prelude);
		} else {
			IB_LOG_INFINI_INFO_FMT(__func__, "<nil>");
		}
		return;

	} else if (!bitset->pool_m || (bitset->nset_m>500)) {
		if (prelude) {
			IB_LOG_INFINI_INFO_FMT(__func__, "%s, nset= %d", prelude, (int)bitset->nset_m);
		} else {
			IB_LOG_INFINI_INFO_FMT(__func__, "nset= %d", (int)bitset->nset_m);
		}
		return;
	}

	status = vs_pool_alloc(bitset->pool_m, max_str_len, (void *)&string);
	if (status != VSTATUS_OK) {
		if (prelude) {
			IB_LOG_INFINI_INFO_FMT(__func__, "%s, nset= %d", prelude, (int)bitset->nset_m);
		} else {
			IB_LOG_INFINI_INFO_FMT(__func__, "nset= %d", (int)bitset->nset_m);
		}
		return;
	}
	string[0] = '\0';

	bit = bitset_find_first_one(bitset);

	while (bit != -1) {
		if (first) {
			res = snprintf(string + pos, max_str_len - pos, "%d", bit);
			if (res > 0){
				pos += res;
			} else {
				if (res == 0)
					break;
				else
					goto bail;
			}
			first = 0;
		} else {
			if (range && (prev != bit-1)) {
				range = 0;
				if ((prev - range_start) > 1) {
					res = snprintf(string + pos, max_str_len - pos, "-%d,%d", prev, bit);
				} else {
					res = snprintf(string + pos, max_str_len - pos, ",%d,%d", prev, bit);
				}
				if (res > 0){
					pos += res;
				} else {
					if (res == 0)
						break;
					else
						goto bail;
				}
				prev = -1;
				range_start = -1;
			} else if (!range && (prev == bit-1)) {
				range_start = prev;
				range = 1;
			} else if (!range) {
				res = snprintf(string + pos, max_str_len - pos, ",%d", bit);
				if (res > 0){
					pos += res;
				} else {
					if (res == 0)
						break;
					else
						goto bail;
				}
			}
		}
		prev = bit;
		bit = bitset_find_next_one(bitset, bit+1);
	}

	if (range && (prev != -1)) {
		if ((prev - range_start) > 1) {
			res = snprintf(string + pos, max_str_len - pos, "-%d", prev);
		} else {
			res = snprintf(string + pos, max_str_len - pos, ",%d", prev);
		}
		if (res > 0){
			pos += res;
		}
	}

bail:  
	if (prelude) {
		IB_LOG_INFINI_INFO_FMT(__func__, "%s %s", prelude, string);
	} else {
		IB_LOG_INFINI_INFO_FMT(__func__, "%s", string);
	}

	if ((status = vs_pool_free(bitset->pool_m, string)) != VSTATUS_OK) {
		IB_LOG_ERRORRC("can't free allocated space for bitset log, rc:", status);
	}
}

int bitset_test_intersection(bitset_t * a, bitset_t * b) {
	int i = 0;

	if (a->nwords_m != b->nwords_m)
		return 0;

	for (i = 0; i < a->nwords_m; ++i) {
		if (a->bits_m[i] & b->bits_m[i])
			return 1;
	}

	return 0;
}

int bitset_set_intersection(bitset_t * a, bitset_t * b, bitset_t * result) {
	int i;

	if (!a->bits_m || !b->bits_m || !result->bits_m) return -1;
	if (a->nwords_m != b->nwords_m || a->nwords_m != result->nwords_m) return -1;

	for (i = 0; i < a->nwords_m; ++i) {
		result->bits_m[i] = a->bits_m[i] & b->bits_m[i];
	}

	result->nset_m = count_nset(result);

	return result->nset_m;
}
