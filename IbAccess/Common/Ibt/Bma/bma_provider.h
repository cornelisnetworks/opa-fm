/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/
/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_IB_BMA_SHIM_H_
#define _IBA_IB_BMA_SHIM_H_

#include "bmamain.h"
#include "ib_bm.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if 0
typedef enum {
	Ok		 	    = 0x00,	  /* Ok */
	UnspErr			= 0x01,	  /* Unspecified Error */
	CmeBusy			= 0x02,	  /* CME Busy */
    MmeBusy			= 0x03,	  /* MME Busy */
    ComNotSupported	= 0x04,	  /* Command not supported */
    IllReqParam		= 0x05,	  /* Illegal Request Parameter */
	GenCompLow	    = 0x06,	  /* Generic Completion status range low  */
	GenCompHigh		= 0xbf,   /* Generic Completion status range high */
	WriteProtect	= 0x40,	  /* Write protected */
	NACKed			= 0x41,	  /* NACK'd */
	BusErr			= 0x42,	  /* BUS Error */
	Busy			= 0x43,	  /* Busy */
	InvVPDDevSel	= 0x44,	  /* Invalid VPD Device Selector */
	IllOffset		= 0x45,	  /* Illegal Offset */
	IllByteCount	= 0x46,	  /* Illegal Byte Count */
	VendSpecLow   	= 0x47,	  /* Vendor Specified range low (vendor ID as in Module info */
	VendSpecHigh   	= 0xff    /* Vendor Specified range high */
} Bma_CompStatus_t;

#endif

uint8 
BmaProvider_WriteVPD
(
	uint8  device, 			/* Device Selector / Address */
	uint16 data_size, 		/* Size of the data to write */
	uint16 data_offset,		/* Offset into the SEEPROM to write the data? */
	uint8 *data				/* Pointer to the data buffer */
);

uint8 
BmaProvider_ReadVPD
(
	uint8  device, 			/* Device Selector / Address */
	uint16 data_size, 		/* Size of the data to read */
	uint16 data_offset,		/* Offset into the SEEPROM to write the data? */
	uint8 *data				/* Pointer to the data buffer */
);

uint8 
BmaProvider_GetModuleStatus
(
	uint8 *data				/* Pointer to the data buffer */
);

#if 0
// this is a callback from the shim to the agent to generate a trap
// not currently supported
extern uint8
BmaProvider_SendTrap
(
	/* Note: See Table 211 in section: 16.2.3.2 in vol 1-1 for details of valid values in these fields. */
	uint8  data_length,		/* Size of the data buffer */
	uint8  trap_type,
	uint32 type_modifier,   /* Note: only bottom 24 bits are actually used.  */
	uint8 *data				/* Pointer to the data buffer */
);

// not currently supported
uint8 
BmaProvider_Ib2IbML
(
	uint8  addr,            /* Device Selector / Address                    */
	uint16 writeLength,     /* Length of the data to write                  */
	uint8 * pWriteBuffer,	/* Pointer to the write data buffer             */
	uint16 * pReadLength,   /* Pointer to the length of the data to read    */
	uint8 * pReadBuffer		/* Pointer to the read data buffer              */
);

// not currently supported
uint8
BmaProvider_GetModulePMControl
(
    uint8 *data				/* Pointer to the data buffer   */
);

// not currently supported
uint8
BmaProvider_SetModuleState
(
    uint8 removalControl    /* Removal Control request byte */
);

// not currently supported
uint8
BmaProvider_SetModuleAttention
(
    uint8 moduleAttention   /* Module Attention request byte */
);

// not currently supported
uint8
BmaProvider_SetModulePMControl
(
    uint8 moduleControl     /* Module Control request byte  */
);
#endif

uint8 
BmaProvider_OemCmd
(
    uint32		 vendorId,		/* IN:  OUI of the vendor */
	uint8		 in_length,		/* IN:  number of bytes in input buffer */
	const uint8* in_data,		/* IN:  input data */
	uint8*		 out_length,	/* OUT: number of bytes in output buffer */
	uint8*		 out_data		/* OUT: output data (up to BM_OEM_RSP_MAX_BYTES bytes) */
);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _IBA_IB_BMA_SHIM_H_ */
