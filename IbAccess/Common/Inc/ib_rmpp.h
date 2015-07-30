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

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_IB_RMPP_H_
#define _IBA_IB_RMPP_H_ (1) /* suppress duplicate loading of this file */

#include "iba/public/datatypes.h"
#include "iba/public/ibyteswap.h"
#include "iba/ib_generalServices.h"

#ifdef __cplusplus
extern "C" {
#endif



/* RMPP Protocol  methods */
#define RMPP_CMD_GET                (0x01)
#define RMPP_CMD_GET_RESP           (0x81)
#define	RMPP_CMD_SET		        (0x02)
#define RMPP_CMD_GETTABLE           (0x12)
#define RMPP_CMD_GETTRACETABLE      (0x13)
#define RMPP_CMD_GETTABLE_RESP      (0x92)
#define	RMPP_CMD_DELETE		        (0x15)	// Request to delete an attribute
#define	RMPP_CMD_DELETE_RESP	    (0x95)	// Response to delete an attribute

#define RMPP_CMD_FE_MNGR_PROBE_CMD  (0x21)
#define RMPP_CMD_FE_MNGR_CLOSE_CMD  (0x22)
#define RMPP_CMD_FE_SEND            (0x24)
#define RMPP_CMD_FE_RESP            (0x25)

#define	MAD_RMPP_REPLY              0x80		// Reply bit for methods

#define RMPP_DATA_OFFSET	32

#define RMPP_CLASS_VERSION	0x01


/* RMPP Protocol Response Structures */
#define RMPP_REC_DESC_LEN						64
#define RMPP_TABLE_REC_DATA_LEN				512
#define RMPP_RECORD_NSIZE					sizeof(rmppRecord_t)
#define RMPP_TABLE_RECORD_NSIZE				sizeof(rmppTableRecord_t)

typedef struct rmppRecord_s {
  uint16_t fieldUint16;  
  uint16_t fieldFiller;  
  uint32_t fieldUint32;
  uint64_t fieldUint64;             
  uint8_t desc[RMPP_REC_DESC_LEN];
} rmppRecord_t;

typedef struct rmppTableRecord_s {
    uint16_t fieldUint16;  
    uint16_t fieldFiller;  
    uint32_t fieldUint32;
    uint64_t fieldUint64;             
  uint8_t desc[RMPP_REC_DESC_LEN];
  uint8_t data[RMPP_TABLE_REC_DATA_LEN];
} rmppTableRecord_t;

typedef struct _RMPP_TABLE_RECORD_RESULTS  {
    uint32 NumrmppRecords;                /* Number of PA Records returned */
    rmppTableRecord_t rmppRecords[1];		/* list of PA records returned */
} RMPP_TABLE_RECORD_RESULTS, *PRMPP_TABLE_RECORD_RESULTS;

static __inline void
BSWAP_RMPP_RECORD(rmppRecord_t  *pRecord)
{
#if CPU_LE
	pRecord->fieldUint16 = ntoh16(pRecord->fieldUint16);
	pRecord->fieldUint32 = ntoh32(pRecord->fieldUint32);
	pRecord->fieldUint64 = ntoh64(pRecord->fieldUint64);
#endif /* CPU_LE */
}

static __inline void
BSWAP_RMPP_TABLE_RECORD(rmppTableRecord_t  *pRecord)
{
#if CPU_LE
	pRecord->fieldUint16 = ntoh16(pRecord->fieldUint16);
	pRecord->fieldUint32 = ntoh32(pRecord->fieldUint32);
	pRecord->fieldUint64 = ntoh64(pRecord->fieldUint64);
#endif /* CPU_LE */
}

#ifdef __cplusplus
}
#endif

#endif /* _IBA_IB_RMPP_H_ */
