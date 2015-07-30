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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _FE_ERROR_TYPES_H_
#define _FE_ERROR_TYPES_H_

/* Generic error types */
#define SUCCESS                0x00000000
#define NOT_IMPLEMENTED        0x00000001
#define NO_COMPLETE            0x00000002
#define TIMEOUT                0x00000004
#define NO_STORE               0x00000008
#define NO_RETRIEVE            0x00000010
#define NULL_MALLOC            0x00000020

/* Generic invalid parameter error types */
#define FV_GUID_INVALID        0x00000040
#define PORT_INDEX_INVALID     0x00000080
#define INVALID_PARAMETER      0x00000100
#define MODE_INVALID           0x00000200
#define PORT_INVALID           0x00000400

/* Configuration invalid parameter error types */
#define HOST_NAME              0x00001000
#define PORT_NAME              0x00002000
#define CONNECTION             0x00004000
#define COMBINATION            0x00008000
#define NOT_LOGGED_IN          0x00010000
#define USER_NAME              0x00020000
#define PASSWORD               0x00040000
#define FUNCTION               0x00080000
#define UNSYNCHRONIZED_VERSION 0x00200000

/* Subnet error types */
#define SYNCHRONIZED           0x00001000

/* Performance error types */
#define START_INVALID          0x00001000
#define INTERVAL_INVALID       0x00002000
#define PORTS_INVALID          0x00004000

/* Device error types */
#define TESTS_NOT_RUN          0x00001000

/* Device invalid parameter error types */
#define NUM_INVALID            0x00002000
#define IOC_INDEX_INVALID      0x00004000

/* Baseboard invalid parameter error types */
#define SLOT_INDEX_INVALID     0x00001000
#define INVALID_CHASSIS_CHILD  0x00002000
#define INVALID_MODULE_CHILD   0x00004000
#define INVALID_SLOT_CHILD     0x00008000
#define INVALID_FRU_CHILD      0x00010000

/* Unsolicited error types */
#define AVAILABLE              0x00001000
#define TEMP_UNAVAILABLE       0x00002000
#define UNAVAILABLE            0x00004000

#endif /* _FE_ERROR_TYPES_H_ */

