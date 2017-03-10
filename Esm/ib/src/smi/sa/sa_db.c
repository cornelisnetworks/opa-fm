/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/* [ICS VERSION STRING: unknown] */

//===========================================================================//
//									     //
// FILE NAME								     //
//    sa_db.c								     //
//									     //
// DESCRIPTION								     //
//    This file contains the routines to process the SA database requests    //
//    These routines are the interface to the PSM.			     //
//									     //
// DATA STRUCTURES							     //
//    None								     //
//									     //
// FUNCTIONS								     //
//    sa_db.c								     //
//									     //
// DEPENDENCIES								     //
//    ib_mad.h								     //
//    ib_status.h							     //
//									     //
//									     //
//===========================================================================//

#include "os_g.h"
#include "ib_status.h" 
#include "ib_types.h"
#include "ib_mad.h"
#include "ib_sm.h"
#include "ib_sa.h"
#include "mai_g.h"
#include "cs_g.h"
#include "sm_l.h"
#include "sa_l.h"

extern uint8_t		*sa_data;
extern uint32_t		sa_data_length;


// -------------------------------------------------------------------------- //

Status_t
sa_template_test_mask(uint64_t componentMask, uint8_t *src, uint8_t ** dstp, uint32_t length, uint32_t bytes, uint32_t * records) {
	Status_t status = VSTATUS_OK;

	IB_ENTER("sa_template_test_mask", src, dstp, length, bytes);

	if (! componentMask || (status = sa_template_test_noinc(src, *dstp, length)) == VSTATUS_OK) {
		sa_increment_and_pad(dstp, length, bytes, records);
	}
	
	IB_EXIT("sa_template_test_mask", status);
	return(status);
}

Status_t
sa_template_test(uint8_t *src, uint8_t ** dstp, uint32_t length, uint32_t bytes, uint32_t * records) {
	Status_t status;

	IB_ENTER("sa_template_test", src, dstp, length, bytes);

	if ((status = sa_template_test_noinc(src, *dstp, length)) == VSTATUS_OK) {
		sa_increment_and_pad(dstp, length, bytes, records);
	}
	
	IB_EXIT("sa_template_test", status);
	return(status);
}

Status_t
sa_template_test_noinc(uint8_t *src, uint8_t * dst, uint32_t length) {
	int i;
	for (i = 0; i < (int)length; i++) {
		if (((src[i] ^ dst[i]) & template_mask[i]) != 0) {
			return(VSTATUS_BAD);
		}
	}

	return(VSTATUS_OK);
}

void
sa_increment_and_pad(uint8_t ** dstp, uint32_t length, uint32_t bytes, uint32_t * records) {

	*dstp += length;
	if (bytes) {
		memset(*dstp, 0, bytes);
		*dstp += bytes;
	}
	
	++(*records);
}

Status_t
sa_check_len(uint8_t *dst, uint32_t length, uint32_t bytes) {
	Status_t status;

	IB_ENTER("sa_check_len", dst, length, bytes, 0);

	if (dst < sa_data ||
		dst + length + bytes > sa_data + sa_data_length) {

		status = VSTATUS_NOMEM;
	} else {
		status = VSTATUS_OK;
	}

	IB_EXIT("sa_check_len", status);
	return(status);
}

// --------------------------------------------------------------------------- /

Status_t
sa_create_template_mask(uint16_t type, uint64_t componentMask) {
	uint8_t		bits;
	int16_t		offset;
	int16_t		length;

	IB_ENTER("sa_create_template_mask", type, componentMask, template_fieldp, 0);

//
//	Zero out the global masking array.
//
	(void)memset((void *)template_mask, 0, sizeof(template_mask));

//
//	The template global variables have been set by a previous call.  So
//	we can use them here.  (This assumes that SA remains single threaded.)
//
	if (template_fieldp == NULL) {
		IB_EXIT("sa_create_template_mask", VSTATUS_OK);
		return(VSTATUS_OK);
	}

//
//	Loop through the component mask and update the global 'template_mask'
//
	while ((componentMask != 0ull) && (template_fieldp->length != 0)) {
		if ((componentMask & 1ull) != 0) {
			offset = template_fieldp->offset;
			length = template_fieldp->length;

			//
			//	Fill out partial first byte.
			//
			if ((offset%8) != 0) {
				// number of masked bits in the first byte
				uint8_t len = MIN(8 - offset%8, length);
				
				// mask of bits in first byte
				// shift right to reduce to len bits, then left to align to field offset
				bits = (0xff >> (8 - len)) << (8 - len - offset%8);

				if ((offset/8) < sizeof(template_mask)) template_mask[offset/8] |= bits;

				length -= len;
				offset += len;
			}

			//
			//	Fill out full bytes.
			//
			while (length >= 8) {
				if ((offset/8) < sizeof(template_mask)) template_mask[offset/8] |= 0xff;
				offset += 8;
				length -= 8;
			}

			//
			//	Fill out partial trailing byte.
			//
			if (length > 0) {
				bits = (length < 8) ? 0xff << (8-length) : 0xff;
				if ((offset/8) < sizeof(template_mask)) template_mask[offset/8] |= bits;
			}
		}

		template_fieldp++;
		componentMask >>= 1;
	}

	IB_EXIT("sa_create_template_mask", VSTATUS_OK);
	return(VSTATUS_OK);
}
