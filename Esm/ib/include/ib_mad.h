/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

// -------------------------------------------------------------------- //
//                                                                      //
// FILE NAME                                                            //
//    ib_mad.h                                                          //
//                                                                      //
// DESCRIPTION                                                          //
//    General transport definitions                                     //
//                                                                      //
// DATA STRUCTURES                                                      //
//    Mad_t		Common MAD header				//
//    DRSmp_t	Directed Route MAD data					//
//    LRSmp_t	LID Route MAD data					//
//                                                                      //
// FUNCTIONS                                                            //
//    None                                                              //
//                                                                      //
// DEPENDENCIES                                                         //
//    ib_types.h                                                        //
//                                                                      //
//                                                                      //
// -------------------------------------------------------------------- //

// -------------------------------------------------------------------- //
//                                                                      //
//	WARNING:  These structures do NOT represent the IBTA defined	//
//		  structures as they appear in the Volume1 spec.	//
//                                                                      //
// -------------------------------------------------------------------- //

#ifndef	_IB_MAD_H_
#define	_IB_MAD_H_

#include "ib_types.h"
#include "ib_const.h"

// IBTA:  Volume 1, Section 13.4.2

//
//	Base MAD format
//
// NOTE:
//   1. Changes here require changes to mai/maic.c
//
typedef	struct {
	uint8_t		bversion;		// base version - 1
	uint8_t		mclass;			// management class
	uint8_t		cversion;		// class version - 1
	uint8_t		method;			// method and/or response
	uint16_t	status;			// status - upper bit is 'D' for DR
	uint8_t		hopPointer;		// DR only
	uint8_t		hopCount;		// DR only
	uint64_t	tid;			// transaction ID
	uint16_t	aid;			// attribute ID
	uint16_t	rsvd3;			// reserved - 0
	uint32_t	amod;			// attribute modifier
} Mad_t;

// IBTA:  Volume 1, Section 13.4.3

#define MAD_BVERSION	   1
#define MAD_CVERSION	   1
#define SA_MAD_CVERSION     2       /* IB 1.1 compliance */
#define CC_MAD_CVERSION     2
#define PA_MAD_CVERSION     1
#define RMPP_MAD_CVERSION   2
#define RMPP_MAD_BVERSION   1
#define	MAD_STATUS_MASK		0x7fff

// IBTA:  Volume 1, Section 13.4.4

//
//	Mad Class Values (MAD_CV)
//
#define	MAD_CV_SUBN_LR		0x01
#define	MAD_CV_SUBN_DR		0x81
#define	MAD_CV_SUBN_ADM		0x03
#define	MAD_CV_PERF		0x04
#define	MAD_CV_BM		0x05
#define	MAD_CV_DEV_MGT		0x06
#define	MAD_CV_COMM_MGT		0x07
#define	MAD_CV_SNMP		0x08
#define	MAD_CV_VENDOR_0		0x09
#define	MAD_CV_VENDOR_CM	0x09		// reserved by CM
//#define	MAD_CV_VENDOR_FE	0x0a		// reserved by FabricExec(tm)
//#define	MAD_CV_INTEL_NS		0x0b		// JSY - temp for Intel testing
//#define	MAD_CV_VENDOR_DBSYNC 0x0b		// SM db sync
#define	MAD_CV_VENDOR_LOG	0x0c		// reserved by CS for Logging
//#define	MAD_CV_VFI_PM		0x0d		// PM VFI mclass value
//#define	MAD_CV_VFI_BM		0x0e		// BM VFI mclass value
#define	MAD_CV_VENDOR_1		0x0f		// use by PMA to get 64bit ctrs
#define	MAD_CV_APP_0		0x10
#define	MAD_CV_APP_1		0x1f
#define MAD_CV_CC			0x21		// CCA
#define	MAD_CV_VENDOR_FE	0x30		// reserved by FabricExec(tm)
#define	MAD_CV_VENDOR_DBSYNC 0x31		// SM db sync
#define	MAD_CV_VFI_PM		0x32		// PM VFI mclass value
#define	MAD_CV_VFI_BM		0x33		// BM VFI mclass value

// VENDOR MAD RANGE
#define MAD_VENDOR_CLASS_MIN    0x30
#define MAD_VENDOR_CLASS_MAX    0x4f

// IBTA:  Volume 1, Section 13.4.5

//
//	Mad Class Methods (MAD_CM)
//
#define	MAD_CM_GET		0x01
#define	MAD_CM_SET		0x02
#define	MAD_CM_GET_RESP		0x81
#define	MAD_CM_SEND		0x03
#define	MAD_CM_TRAP		0x05
#define	MAD_CM_REPORT		0x06
#define	MAD_CM_REPORT_RESP	0x86
#define	MAD_CM_TRAP_REPRESS	0x07
#define	MAD_CM_VIEO_REQ		0x98		// VIEO specific
#define	MAD_CM_VIEO_REP		0x99		// VIEO specific

#define	MAD_CM_REPLY		0x80		// Reply bit for methods

// IBTA:  Volume 1, Section 13.4.7

//
//	Common MAD status bits
//

#define	MAD_STATUS_OK			0x0000
#define	MAD_STATUS_BUSY			0x0001
#define	MAD_STATUS_REDIRECT		0x0002
#define	MAD_STATUS_BAD_CLASS	0x0004
#define	MAD_STATUS_BAD_METHOD	0x0008
#define	MAD_STATUS_BAD_ATTR		0x000c
#define	MAD_STATUS_RSVD1		0x0010
#define	MAD_STATUS_RSVD2		0x0014
#define	MAD_STATUS_RSVD3		0x0018
#define	MAD_STATUS_BAD_FIELD	0x001c
#define MAD_STATUS_D_BIT        0x8000      // set by SM if response to directed route SMInfo

// IBTA:  Volume 1, Section 13.4.8.1

//
//	ClassPortInfo
//
typedef	struct {
	uint8_t		bversion;		// base version
	uint8_t		cversion;		// class version
	uint16_t	capabilityMask;		// capability mask (2)
	uint32_t	capabilityMask2;	// capability mask2 (6)
	uint8_t		respTimeValue;		//
	uint8_t		redirectGid[16];	// GID for GRH
	uint8_t		redirectTC;		// TC
	uint8_t		redirectSL;		// SL
	uint32_t	redirectFL;		// FL
	uint16_t	redirectLid;		// DLID to access the class services
	uint16_t	redirectPKey;		// P_Key to use
	uint8_t		rsvd2;			// (8)
	uint32_t	redirectQP;		// QP to use (can't be zero) (24)
	uint32_t	redirectQKey;		// Q_Key associated with redirect QP
	uint8_t		trapGid[16];		// GID for GRH trap messages
	uint8_t		trapTC;			// TC (8)
	uint8_t		trapSL;			// SL (4)
	uint32_t	trapFL;			// FL (20)
	uint16_t	trapLid;		// LID for traps
	uint16_t	trapPKey;		// P_Key for traps
	uint8_t		trapHL;			// HL
	uint32_t	trapQP;			// QP
	uint32_t	trapQKey;		// Q_Key associated with trap QP
} ClassPortInfo_t;


// IBTA:  Volume 1, Section 13.4.8.3

typedef	struct {
	uint8_t		gid[16];		// GID (or zero)
	Lid_t	    startLid;		// starting Lid range value
	Lid_t	    endLid;			// ending Lid range value
	uint16_t	reserved1;		// reserved
	uint8_t		isGeneric;		// 1 -> forward generic traps
	uint8_t		subscribe;		// 1 -> subscribe
	uint16_t	type;		    // class of trap/notice
	uint16_t	trapNumber;		// trap number (or device ID)
    uint32_t    qpn;            // Queue pair number - 24 bits
    uint8_t     reserv3bits;    // 3 bit reserved field
	uint8_t     respTimeValue;  // time to respond to requests - 5 bit field
	uint8_t		reserved2;		// reserved
	uint32_t	nodeType;		// node type (or vendor ID) - 24 bit field
} InformInfo_t;

#define	TRAP_ALL	0xffff
#define NODE_TYPE_ALL 0xffffff

//
// InformInfo Producer Type
//
#define	INFORMINFO_PRODUCERTYPE_CA		1
#define	INFORMINFO_PRODUCERTYPE_SWITCH	2
#define	INFORMINFO_PRODUCERTYPE_ROUTER	3
#define INFORMINFO_PRODUCERTYPE_SM		4

// IBTA:  Volume 1, Section 14.2.1.1

#define LRSMP_RSVD3_COUNT	32
#define LRSMP_SMPDATA_COUNT	64
#define LRSMP_RSVD4_COUNT	128

typedef	struct {
	uint64_t	mkey;				// management key
	uint8_t		rsvd3[LRSMP_RSVD3_COUNT];	// reserved - 0
	uint8_t		smpdata[LRSMP_SMPDATA_COUNT];	// SMP data
	uint8_t		rsvd4[LRSMP_RSVD4_COUNT];	// reserved - 0
} LRSmp_t;

// IBTA:  Volume 1, Section 14.2.1.2

#define DRSMP_RSVD2_COUNT	28
#define DRSMP_SMPDATA_COUNT	64
#define DRSMP_IPATH_COUNT	64
#define DRSMP_RPATH_COUNT	64

typedef	struct {
	uint64_t	mkey;				// management key
	uint16_t	drslid;				// source LID
	uint16_t	drdlid;				// destination LID
	uint8_t		rsvd2[DRSMP_RSVD2_COUNT];	// reserved - 0
	uint8_t		smpdata[DRSMP_SMPDATA_COUNT];	// SMP data
	uint8_t		ipath[DRSMP_IPATH_COUNT];	// initial path
	uint8_t		rpath[DRSMP_RPATH_COUNT];	// return path
} DRSmp_t;

#define	MAD_DR_INITIAL		0x0000
#define	MAD_DR_RETURN		0x8000

// IBTA:  Volume 1, Section 13.4.8.2

//
//	Generic Notice structure.
//

#define NOTICE_DATA_COUNT	54

typedef	struct {
	uint8_t		generic;			// generic
	uint8_t		noticeType;			// noticeType
	uint32_t	nodeType;			// nodeType
	uint16_t	trapNumber;			// class-defined (or device ID)
	uint16_t	issuerLid;			// LID of issuer
	uint8_t		noticeToggle;			// noticeToggle
	uint16_t	noticeCount;			// noticeCount
	uint8_t		data[NOTICE_DATA_COUNT];	// class specific data
    Gid_t       issuerGid;          // Issuer Gid
} Notice_t;
typedef Notice_t * Noticep;  /* pointer to a notice structure */

#define	NOTICE_TYPE_FATAL		0
#define	NOTICE_TYPE_URGENT		1
#define	NOTICE_TYPE_SECURITY		2
#define	NOTICE_TYPE_SM			3
#define	NOTICE_TYPE_INFO		4
#define	NOTICE_TYPE_EMPTY		0x7f

#define	NOTICE_PRODUCERTYPE_CA           1
#define	NOTICE_PRODUCERTYPE_SWITCH       2
#define	NOTICE_PRODUCERTYPE_ROUTER       3
#define NOTICE_PRODUCERTYPE_CLASSMANAGER 4

#include "ib_sm.h"

#endif	// _IB_MAD_H_

