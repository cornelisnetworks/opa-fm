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

//=======================================================================
//									/
// FILE NAME								/
//    ib_macros.h							/
//									/
// DESCRIPTION								/
//    Definition of some convenience macros for MAD processing.		/
//									/
// DATA STRUCTURES							/
//    None								/
//									/
// FUNCTIONS								/
//    None								/
//									/
// DEPENDENCIES								/
//    ib_mad.h								/
//									/
//									/
//=======================================================================

#ifndef	_IB_MACROS_H_
#define	_IB_MACROS_H_

#include "ib_mad.h"

// -------------------------------------------------------------------- //
//                                                                      //
//      These macros are for the use of Infiniband managers.  They are  //
//      convenience macros only.                                        //
//                                                                      //
// -------------------------------------------------------------------- //

//
//	Setup the MAI.
//
#define	Mai_Init(MAIP) {							\
	(void)memset((void *)MAIP, 0, sizeof(*MAIP));				\
										\
	(MAIP)->type = MAI_TYPE_EXTERNAL;						\
	(MAIP)->active = MAI_ACT_TYPE;						\
}

//
//	Setup the addrInfo.
//
#define	AddrInfo_Init(MAIP,SLID,DLID,SL,PKEY,SRCQP,DSTQP,QKEY) {	\
	(MAIP)->active |= MAI_ACT_ADDRINFO;						\
	(MAIP)->addrInfo.slid = SLID;						\
	(MAIP)->addrInfo.dlid = DLID;						\
	(MAIP)->addrInfo.sl = SL;							\
	(MAIP)->addrInfo.pkey = PKEY;						\
	(MAIP)->addrInfo.srcqp = SRCQP;						\
	(MAIP)->addrInfo.destqp = DSTQP;						\
	(MAIP)->addrInfo.qkey = QKEY;						\
}

//
//	Setup the LR_Mad
//
#define	LRMad_Init(MAIP,MCLASS,METHOD,TID,AID,AMOD,MKEY) {			\
	LRSmp_t	*lrp;								\
										\
	(void)memset((void *)&(MAIP)->base, 0, sizeof((MAIP)->base));		\
										\
	(MAIP)->active |= (MAI_ACT_BASE | MAI_ACT_DATA);			\
	(MAIP)->base.bversion = MAD_BVERSION;					\
	(MAIP)->base.mclass = MCLASS;						\
	(MAIP)->base.cversion = MCLASS == MAD_CV_SUBN_ADM ? SA_MAD_CVERSION : MAD_CVERSION;					\
	(MAIP)->base.method = METHOD;						\
	(MAIP)->base.status = 0;						\
	(MAIP)->base.hopPointer = 0;						\
	(MAIP)->base.hopCount = 0;						\
	(MAIP)->base.tid = TID;							\
	(MAIP)->base.aid = AID;							\
	(MAIP)->base.rsvd3 = 0;							\
	(MAIP)->base.amod = AMOD;						\
										\
	lrp = (LRSmp_t *)(MAIP)->data;						\
	(void)memset((void *)lrp, 0, sizeof(*lrp));				\
										\
	lrp->mkey = MKEY;							\
}

//
//	Setup the LR_Mad data.
//
#define	LRData_Init(MAIP,DATA,BYTES) {						\
	LRSmp_t	*lrp;								\
										\
	(MAIP)->active |= MAI_ACT_DATA;						\
										\
	lrp = (LRSmp_t *)(MAIP)->data;						\
	(void)memcpy((void *)lrp->smpdata, (void *)(DATA), BYTES);		\
}

//
//	Setup the Filter.
//
#define	Filter_Init(FILTERP,ACTIVE,TYPE) {					\
	(void)memset((void *)FILTERP, 0, sizeof(*FILTERP));			\
										\
	(FILTERP)->active = ACTIVE;						\
	(FILTERP)->type = TYPE;							\
	(FILTERP)->dev = MAI_FILTER_ANY;					\
	(FILTERP)->port = MAI_FILTER_ANY;					\
	(FILTERP)->qp = MAI_FILTER_ANY;						\
}

#endif	// _IB_MACROS_H_

