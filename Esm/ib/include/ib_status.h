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

// ******************************************************************** //
//                                                                      //
// FILE NAME                                                            //
//    ib_status.h                                                       //
//                                                                      //
// DESCRIPTION                                                          //
//    lists all of the possible status codes which are returned by      //
//    routines of type Status_t                                         //
//                                                                      //
// DATA STRUCTURES                                                      //
//    Status_t          VIEO specifiec status                           //
//                                                                      //
// FUNCTIONS                                                            //
//    None                                                              //
//                                                                      //
// DEPENDENCIES                                                         //
//    ib_types.h                                                        //
//                                                                      //
//                                                                      //
// HISTORY                                                              //
//                                                                      //
//    NAME      DATE  REMARKS                                           //
//     jsy  01/15/01  Initial creation of file                          //
//     jsy  01/15/01  Fix typo                                          //
//     jmm  01/21/01  Added some new mai required status codes          //
//     jsy  01/29/01  Added conditional status                          //
//     trp  02/05/01  Added common transport services error codes	//
//     jsy  02/05/01  Added VSTATUS_EIO					//
//     jsy  02/08/01  Added VSTATUS_KNOWN				//
//     jsy  02/08/01  Added VSTATUS_NOT_MASTER				//
//     jsy  02/09/01  Added VSTATUS_BAD in second place			//
//     jsy  02/12/01  Added VSTATUS_QFULL and EMPTY			//
//     sfw  02/13/01  Added VSTATUS_AGAIN                               //
//     trp  02/14/01  Added VSTATUS_UNINIT                              //
//     jmm  02/18/01  Added more descriptive mai_* error codes          //
//     dkj  03/08/01  Added VSTATUS_NOSUPPORT                           //
//     jak  03/21/01  Added VSTATUS_IGNORE                              //
//     dkj  03/29/01  Added VSTATUS_INUSE                               //
//     pjg  04/02/01  Added VSTATUS_INVALID_PORT through                /
//                      VSTATUS_INVALID_NODE                            //
//     trp  04/05/01  Added INTERNAL and MAD_FULL for MAI			//
//     jak  04/20/01  Added VSTATUS_SIGNAL				//
//     trp  06/10/01  Cleaned up comments				//
//     trp  07/31/01  Added MAI FILTER and RAW				//
//     sfw  08/14/01  Added VSTATUS_EVENT_CONSUMED                      //
//     ram  11/13/01  Fix type (VSTATUS_MAD_OERFLOW)                    //
//     dkj  01/09/02  Added VSTATUS_INVALID_GID_INDEX                   //
//     dkj  03/21/02  Added VSTATUS_BADPAGESIZE                         //
//     dkj  04/16/02  Added VSTATUS_INVALID_CQ_HANDLE                   //
//     ram  04/22/02  Added VSTATUS_INVALID_FORMAT                      //
// ******************************************************************** //

#ifndef	_IB_STATUS_H_
#define	_IB_STATUS_H_

#include "ib_types.h"

typedef	uint32_t	Status_t;

#define	VSTATUS_OK		 (0)		// Good status
#define	VSTATUS_BAD		 (1)		// Bad status
#define	VSTATUS_MISMATCH	 (2)		// Key mismatch
#define	VSTATUS_DROP		 (3)		// Drop this packet
#define	VSTATUS_FORWARD		 (4)		// Forward this packet
#define VSTATUS_ILLPARM		 (5)		// Invalid parameter
#define VSTATUS_NOMEM		 (6)		// Out of memory
#define VSTATUS_TIMEOUT		 (7)		// Request timed out
#define VSTATUS_NOPRIV		 (8)		// Not enough privs
#define VSTATUS_BUSY		 (9)		// Can't do it - busy
#define VSTATUS_NODEV		(10)		// No object available
#define VSTATUS_NXIO		(11)		// Invalid object
#define VSTATUS_PARTIAL_PRIVS	(12)		// Some privs are avail
#define VSTATUS_CONDITIONAL     (13)		// Conditionally good
#define VSTATUS_NOPORT          (14)		// Port does not exist

#define	VSTATUS_INVALID_HANDL	(15)		// invalid handle
#define	VSTATUS_INVALID_TYPE	(16)		// invalid type (ib_attr)
#define	VSTATUS_INVALID_ATTR	(17)		// invalid attribute (ib_attr)
#define	VSTATUS_INVALID_PROTO	(18)		// invalid protocol (RC,RD,UC,UD,RAW)
#define	VSTATUS_INVALID_STATE	(19)		// invalid CEP or QP state
#define	VSTATUS_NOT_FOUND	(20)		// item not found
#define	VSTATUS_TOO_LARGE	(21)		// value too large
#define	VSTATUS_CONNECT_FAILED	(22)		// attempt to connect failed
#define	VSTATUS_CONNECT_GONE	(23)		// connection torn down
#define	VSTATUS_NOHANDLE	(24)		// no more handles available
#define	VSTATUS_NOCONNECT	(25)		// no more connections available

#define	VSTATUS_EIO		(26)		// generic IO error
#define	VSTATUS_KNOWN		(27)		// info already known
#define	VSTATUS_NOT_MASTER	(28)		// not the Master SM

#define	VSTATUS_INVALID_MAD	(29)		// invalid MAD
#define	VSTATUS_QFULL		(30)		// Queue is full
#define	VSTATUS_QEMPTY		(31)		// Queue is full

#define VSTATUS_AGAIN           (32)            // Data not available

#define VSTATUS_BAD_VERSION     (33)		// Version mismatch

#define VSTATUS_UNINIT	        (34)		// Library not initialized

#define VSTATUS_NOT_OWNER       (35)		// Not resource owner
#define VSTATUS_INVALID_MADT    (36)		// Malformed Mai_t
#define VSTATUS_INVALID_METHOD  (37)		// MAD method invalid
#define VSTATUS_INVALID_HOPCNT  (38)		// HopCount not in [0..63]
#define VSTATUS_MISSING_ADDRINFO     (39)		// Mai_t has no addrInfo
#define VSTATUS_INVALID_LID     (40)		// LID=RESERVED_LID
#define VSTATUS_INVALID_ADDRINFO     (41)		// invalid addrInfo in Mai_t
#define VSTATUS_MISSING_QP      (42)		// QP not in Mai_t
#define VSTATUS_INVALID_QP      (43)		// MAI QP not SMI(0) or GSI(1)
#define VSTATUS_INVALID_MCLASS	(48)		// Reserved mclass used
#define VSTATUS_INVALID_QKEY	(51)		// BTH qkey field for GSI invalid
#define VSTATUS_INVALID_MADLEN	(53)		// MAD datasize invalid/not determinable
#define VSTATUS_NOSUPPORT       (55)            // Function not supported
#define VSTATUS_IGNORE          (56)            // Ignorable condition
#define VSTATUS_INUSE           (57)            // Resource already in use

#define VSTATUS_INVALID_PORT           (58)     // Port invalid
#define VSTATUS_ATOMICS_NOTSUP         (59)     // Atomic operations not
                                                //   supported by hardware
#define VSTATUS_INVALID_ACCESSCTL      (60)     // Access definition invalid
#define VSTATUS_INVALID_ADDR_HANDLE    (61)     // Address handle invalid
#define VSTATUS_INVALID_ADDR           (62)     // Address invalid
#define VSTATUS_INVALID_ARG            (63)     // Arguments invalid
#define VSTATUS_INVALID_HCA_HANDLE     (64)     // Channel adapter handle invalid
#define VSTATUS_INVALID_KEY_VALUE      (65)     // Partition key invalid
#define VSTATUS_INVALID_MEMADDR        (66)     // Memory not accessible
#define VSTATUS_INVALID_MEMSIZE        (67)     // Memory region size not
                                                //   supported by hardware
#define VSTATUS_INVALID_MIGSTATE       (68)     // Migration state requested invalid
#define VSTATUS_INVALID_NOTICE         (69)     // Completion notice requested invalid
#define VSTATUS_INVALID_OFFSET         (70)     // Offset specified invalid
#define VSTATUS_INVALID_PD             (71)     // Protection domain invalid
#define VSTATUS_INVALID_PKEY_IDX       (72)     // Partition key index invalid
#define VSTATUS_INVALID_RC_TIMER       (73)     // Reliable connection timeout invalid
#define VSTATUS_INVALID_RDD            (74)     // Reliable datagram domain invalid
#define VSTATUS_INVALID_REQUEST        (75)     // Request is invalid for this type of hardware
#define VSTATUS_INVALID_RNR_CNT        (76)     // Receiver not ready count invalid.
#define VSTATUS_INVALID_RNR_TIMER      (77)     // Receiver not ready timer invalid.
#define VSTATUS_INVALID_TRANSPORT      (78)     // Transport type invalid or not supported
#define VSTATUS_INVALID_WORKREQ        (79)     // Work request invalid for transport service
#define VSTATUS_MCAST_NOTSUP           (80)     // Multicast not supported
#define VSTATUS_MCGRP_QPEXCEEDED       (81)     // Number of Queue Pairs in the
                                                //   multicast group exceeded
                                                //   the number supported
#define VSTATUS_MEMRGN_NOTSUP          (82)     // Memory regions not supported
#define VSTATUS_MEMWIN_NOTSUP          (83)     // Memory windows not supported
#define VSTATUS_NORESOURCE             (84)     // Out of hardware resources
#define VSTATUS_OPERATION_DENIED       (85)     // Operation cannot be performed
#define VSTATUS_QP_BUSY                (86)     // Queue Pair is already opened
#define VSTATUS_RDMA_NOTSUP            (87)     // Remote DMA operations not supported
#define VSTATUS_RD_NOTSUP              (88)     // Reliable Datagram transport not supported
#define VSTATUS_SG_TOOMANY             (89)     // Number of scatter/gather entries
                                                //   exceeds supported capability.
#define VSTATUS_WQ_RESIZE_NOTSUP       (90)     // Work queue resizing not supported
#define VSTATUS_WR_TOOMANY             (91)     // Work queue size not supported
#define VSTATUS_INVALID_NODE           (92)     // Node index invalid
#define	VSTATUS_MAI_INTERNAL		   (93)	// MAI internal loopback message
#define	VSTATUS_MAD_OVERFLOW	       (94)	// MAD buffer overflow
#define	VSTATUS_SIGNAL		       (95)	// Signal received
#define	VSTATUS_FILTER		       (96)	// Filter take over notification received
#define	VSTATUS_EXPIRED		       (98)	// time period has expired
#define	VSTATUS_EVENT_CONSUMED	       (99)	// handler consumed event
#define	VSTATUS_INVALID_PATH	      (100)	// path handle invalid
#define VSTATUS_INVALID_GID_INDEX     (101)     // GID index invalid
#define VSTATUS_INVALID_DEVICE        (102)     // Device ordinal out of range
#define VSTATUS_INVALID_RESOP         (103)     // resolver operation invalid
#define VSTATUS_INVALID_RESCMD	      (104)     // resolver command invalid
#define VSTATUS_INVALID_ITERATOR      (105)     // resolver iterator invalid
#define VSTATUS_INVALID_MAGIC         (106)     // magic number not correct for structure
#define VSTATUS_BADPAGESIZE           (107)     // invalid memory page size specified
#define VSTATUS_UNRECOVERABLE         (108)     // a variant of BAD that indicates to the
                                                //   caller that recovery (within a context)
                                                //   should not be attempted
#define VSTATUS_ITERATOR_OUT_OF_DATE     (119)   // data referred by
                                                // iterator updated or deleted.
#define VSTATUS_INSUFFICIENT_PERMISSION  (120)  // client has insufficient
                                                // privillages for action
#define VSTATUS_INVALID_CQ_HANDLE        (126)  // CQ handle is invalid
#define VSTATUS_INVALID_FORMAT           (127)  // Data format is invalid
#define VSTATUS_REJECT                   (128)  // Request rejected

// WHEN ADDING STATUS CODES, ALSO UPDATE cs_convert_status() in cs_utility.c

#endif	// _IB_STATUS_H_
