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

 * ** END_ICS_COPYRIGHT5   ****************************************/
/* [ICS VERSION STRING: unknown] */

#if HAVE_CONFIG_H
#  include <config.h>
#endif /* HAVE_CONFIG_H */

#include <stdlib.h>
#include <string.h>
#include <byteswap.h>

#include <netinet/in.h>

#include "ib_notice_net.h"

uint32_t ibv_sa_get_field(void *data, int offset, int size)
{
	uint32_t value, left_offset;

	left_offset = offset & 0x07;
	if (size <= 8) {
		value = ((uint8_t *) data)[offset / 8];
		value = ((value << left_offset) & 0xFF) >> (8 - size);
	} else if (size <= 16) {
		value = ntohs(((uint16_t *) data)[offset / 16]);
		value = ((value << left_offset) & 0xFFFF) >> (16 - size);
	} else {
		value = ntohl(((uint32_t *) data)[offset / 32]);
		value = (value << left_offset) >> (32 - size);
	}
	return value;
}

void ibv_sa_set_field(void *data, uint32_t value, int offset, int size)
{
	uint32_t left_value, right_value;
	uint32_t left_offset, right_offset;
	uint32_t field_size;

	if (size <= 8)
		field_size = 8;
	else if (size <= 16)
		field_size = 16;
	else
		field_size = 32;

	left_offset = offset & 0x07;
	right_offset = field_size - left_offset - size;

	left_value = left_offset ? ibv_sa_get_field(data, offset - left_offset,
						    left_offset) : 0;
	right_value = right_offset ? ibv_sa_get_field(data, offset + size,
						      right_offset) : 0;

	value = (left_value << (size + right_offset)) |
		(value << right_offset) | right_value;

	if (field_size == 8)
		((uint8_t *) data)[offset / 8] = (uint8_t) value;
	else if (field_size == 16)
		((uint16_t *) data)[offset / 16] = htons((uint16_t) value);
	else
		((uint32_t *) data)[offset / 32] = htonl((uint32_t) value);
}
