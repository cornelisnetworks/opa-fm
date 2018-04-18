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

#ifndef __STL_RMPP_H__
#define __STL_RMPP_H__

#include "iba/public/ipackon.h"
#include "ib_mad.h"

#if defined (__cplusplus)
extern "C" {
#endif

/*
 * Base RMPP header
 */
typedef	struct _STL_RMPP_DATA_HEADER {
	uint8_t     rmppVersion;
	uint8_t     rmppType;
	union {
		struct {
			#if CPU_BE
				uint8_t  rmppRespTime : 5;
				uint8_t  rmppFlags : 3;
			#else
				uint8_t  rmppFlags : 3;
				uint8_t  rmppRespTime : 5;
			#endif
		} tf;
		uint8_t timeFlag;
	} u;
	uint8_t     rmppStatus;
	uint32_t    segNum;
	uint32_t    length;
} STL_RMPP_DATA_HEADER ;

typedef	struct _STL_RMPP_ACK_HEADER {
	uint8_t     rmppVersion;
	uint8_t     rmppType;
	union {
		struct {
			#if CPU_BE
				uint8_t  rmppRespTime : 5;
				uint8_t  rmppFlags : 3;
			#else
				uint8_t  rmppFlags : 3;
				uint8_t  rmppRespTime : 5;
			#endif
		} tf;
		uint8_t timeFlag;
	} u;
	uint8_t     rmppStatus;
	uint32_t    segNum;
	uint32_t    newwindowlast;
} STL_RMPP_ACK_HEADER ;

/*
 * Vendor RMPP Header
 */
typedef	struct _STL_RMPP_VENDOR_HEADER {
	uint8_t     rmppVersion;
	uint8_t     rmppType;
	union {
		struct {
			#if CPU_BE
				uint8_t  rmppRespTime : 5;
				uint8_t  rmppFlags : 3;
			#else
				uint8_t  rmppFlags : 3;
				uint8_t  rmppRespTime : 5;
			#endif
		} tf;
		uint8_t timeFlag;
	} u;
	uint8_t     rmppStatus;
	uint32_t    segNum;
	uint32_t    length;
	uint8_t     reserved;
	uint8_t     oui[3];
} PACK_SUFFIX STL_RMPP_VENDOR_HEADER;

#define STL_RMPP_VENDOR_DATA_LEN	(STL_MAD_BLOCK_SIZE - \
				 sizeof(STL_RMPP_VENDOR_HEADER) - \
				 sizeof(MAD_COMMON))
#define STL_RMPP_SIZEOF_OUI 4

typedef struct _STL_RMPP_VENDOR_MAD {
	MAD_COMMON              common;
	STL_RMPP_VENDOR_HEADER	header;
	uint8_t		data[STL_RMPP_VENDOR_DATA_LEN];
} PACK_SUFFIX STL_RMPP_VENDOR_MAD;

/*
 * Contains the vendor header (RMPP + OUI) and the data portion
 */
typedef struct _STL_RMPP_VENDOR_PACKET {
	STL_RMPP_VENDOR_HEADER header;
	uint8_t             data[STL_RMPP_VENDOR_DATA_LEN];
} PACK_SUFFIX STL_RMPP_VENDOR_PACKET;


static __inline
void
BSWAPCOPY_STL_RMPP_VENDOR_HEADER(STL_RMPP_VENDOR_HEADER *src,
			      STL_RMPP_VENDOR_HEADER *dst)
{
	dst->rmppVersion = src->rmppVersion;
	dst->rmppType = src->rmppType;
	dst->rmppStatus = src->rmppStatus;
	dst->u.timeFlag = src->u.timeFlag;

	dst->segNum = ntoh32(src->segNum);
	dst->length = ntoh32(src->length);
}

static __inline
void
BSWAPCOPY_STL_RMPP_VENDOR_PACKET(STL_RMPP_VENDOR_PACKET *src,
		       STL_RMPP_VENDOR_PACKET  *dst, size_t len)
{
	BSWAPCOPY_STL_RMPP_VENDOR_HEADER(&src->header,&dst->header);
	memcpy(&dst->data, &src->data, len);
}

#if defined (__cplusplus)
}
#endif

#endif
