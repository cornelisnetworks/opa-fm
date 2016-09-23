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

#ifndef _IB_CONST_H_
#define _IB_CONST_H_

// ******************************************************************** //
//                                                                      //
// FILE NAME                                                            //
//    ib_const.h                                                        //
//                                                                      //
// DESCRIPTION                                                          //
//    Constant values for iba                                           //
//                                                                      //
//                                                                      //
// FUNCTIONS                                                            //
//    None                                                              //
//                                                                      //
// DEPENDENCIES                                                         //
//                                                                      //
//                                                                      //
//                                                                      //
// HISTORY                                                              //
//                                                                      //
//    NAME      DATE  REMARKS                                           //
//     pwalker  12/21/00  Initial creation of file.                     //
//     jsy      02/09/01  Added default GID prefix.			//
//                                                                      //
// ******************************************************************** //

#define OUI_TRUESCALE_0 (0x00) /* 3-byte InfiniCon (now Intel) OUI */
#define OUI_TRUESCALE_1 (0x06)
#define OUI_TRUESCALE_2 (0x6a)

#define GSI_WELLKNOWN_QKEY		(0x80010000)  /* QKEY for GSI            */
#define UNICAST_LID_MAX			(0xBFFF)      /* Unicast LID upper range */
#define UNICAST_LID_MIN			(0x0001)      /* Lowest valid Unicast Lid*/
#define MULTICAST_LID_MAX		(0xFFFE)      /* MAX MC lid value        */
#define MULTICAST_LID_MIN		(0xC000)      /* Min MC lid value        */
#define PERMISSIVE_LID			(0xFFFF)      /* permissive lid for SMP  */
#define RESERVED_LID			(0)           /* Reserved lid in IB space*/

#define MAX_LMC_BITS			(8)           /* Max number of LMC bits   */
#define	DEFAULT_GID_PREFIX		0xFE80000000000000ull /* Default GID      */

#define LINK_LOCAL_ADDR_MASK		(0xFE80)      /* mask for link local addr */
#define SITE_LOCAL_ADDR_MASK		(0xFEC0)      /* mask for site local      */
#define MUTLICAST_ADDR_MASK		(0xFF00)      /* multicast address        */
#define MULTICAST_GID_FLAGSCOPE_MASK	(0x00FF)      /* mask for  scope bits     */

#define IBA_MAX_HDR                    (128)          /* Maximum pkt hdr size    */ 
#define IBA_MAX_PATHSIZE               (63)           /* mximum DR hops          */

#define IBA_VENDOR_RANGE1_START (0x09) /* range for standard vendor classes */
#define IBA_VENDOR_RANGE1_END   (0x0f)
#define IBA_VENDOR_RANGE2_START (0x30) /* range for RMPP/OUI vendor classes */
#define IBA_VENDOR_RANGE2_END   (0x4f)

#define EUI64_COMPANYID_LEN     (3)         /* length in bytes                    */
#define EUI64_EXTENSION_LEN     (5)         /* length in bytes                    */
#define EUI64_LEN               (8)         /* length in bytes                    */

#define SMP_VIRTUAL_LANE       (0xf)        /* use by SMP MADs only               */
#define MAX_VIRTUAL_LANES      (16)         /* Max number of VLs 0 .. 15          */
#define MAX_SLS			       (16)         /* Max number of SLs 0 .. 15          */


#define MAX_LFT_SIZE            (48*1024)   /* for all UNICAST LIDs               */
#define MAX_RFT_SIZE            (48*1024)   /* worst case for  RFT                */
#define MAX_MCFT_SIZE           (16*1024)   /* for all Multicast LIDS             */

//
//	SM Mad retries and default Receive wait interval
//
#define LOG_LEVEL_MIN		0
#define LOG_LEVEL_MAX		5
#define	MAD_RETRIES		    3
#define MAD_RCV_WAIT_MSEC   250
#define MAD_MIN_RCV_WAIT_MSEC   35	/* PR 110945 - when using stepped retries, default minimum timeout
									 * with which retries are started. Subsequent retries will be
									 * multiples of this timeout value.
									 */

#define DYNAMIC_PACKET_LIFETIME_ARRAY_SIZE 10

#define DYNAMIC_PLT_MIN 0
#define DYNAMIC_PLT_MAX 0x3F

#define	SM_MAX_PRIORITY		15	/* 4-bit field in SMInfo - table 45 vol 1.1 */

#define SM_NO_SWEEP         0   /* do not allow a sweep */
#define	SM_MIN_SWEEPRATE	3	/* 3 Seconds */
#define	SM_MAX_SWEEPRATE	86400	/* 24 Hours */

#define BM_NO_SWEEP         0   /* do not allow a sweep */
#define	BM_MIN_SWEEPRATE	30	/* 30 Seconds */
#define	BM_MAX_SWEEPRATE	86400	/* 24 Hours */

#define	PM_MIN_SWEEPRATE    1       /* 1 Second */
#define	PM_MAX_SWEEPRATE    43200   /* 12 Hours */

/* the multi path defs are from sm_l.h */
// FIXME
#define SM_NO_MULTI_PATH	0
#define SM_SOURCE_ROUTE_MULTIPATH 1

#define SM_MIN_SWITCH_LIFETIME	0
#define SM_MAX_SWITCH_LIFETIME	31

#define SM_MIN_HOQ_LIFE		0
#define SM_MAX_HOQ_LIFE		31

#define SM_MIN_VL_STALL		1
#define SM_MAX_VL_STALL		7

#define MAX_PKEYS 32

#define PKEY_TABLE_LIST_COUNT                   32

/* number of endports suppported by fabric related defines */
#ifdef __VXWORKS__
#define     MIN_SUPPORTED_ENDPORTS  40      /* max for 9020 VIO card */ 
#define     MAX_SUPPORTED_ENDPORTS  512
#define     MAX_SUPPORTED_ENDPORTS_9020  MIN_SUPPORTED_ENDPORTS
#else
#define     MIN_SUPPORTED_ENDPORTS  40 
#endif

#endif	// _IB_CONST_H_









