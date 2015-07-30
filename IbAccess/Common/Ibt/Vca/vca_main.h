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

#ifndef _IBA_VCA_MAIN_H_
#define _IBA_VCA_MAIN_H_ (1) // suppress duplicate loading of this file


//
// External declarations of global variables
//

extern struct _VCA_GLOBAL	VcaGlobal;


//
// Constant definitions
//

#define VCA_MEM_TAG	MAKE_MEM_TAG(V,c,a,M)


//
// Forward declarations
//

FSTATUS
vcaLoad(EUI64 SystemImageGUID);

void
vcaUnload(void);

FSTATUS
vcaInitCA(
	IN DEVICE_INFO	*DevInfo
	);

FSTATUS
vcaAddCA(
	IN DEVICE_INFO	*DevInfo
	);

void
vcaRemoveCA(
	IN DEVICE_INFO	*DevInfo
	);

FSTATUS
vcaFreeCA(
	IN DEVICE_INFO	*DevInfo
	);

DEVICE_INFO *
vcaFindCA(
	IN EUI64 CaGuid
	);

FSTATUS
default_poll_and_rearm_cq(
	IN  IB_HANDLE			CqHandle,
	IN	IB_CQ_EVENT_SELECT	EventSelect,
	OUT IB_WORK_COMPLETION	*WorkCompletion
	);

FSTATUS
vcaOpenCa(
	IN  DEVICE_INFO				*DevInfo,
	IN	IB_COMPLETION_CALLBACK	CompletionCallback,
	IN	IB_ASYNC_EVENT_CALLBACK	AsyncEventCallback,
	IN	void					*Context,
	OUT	IB_HANDLE				*CaHandle
	);

//
// VCA global info
//

typedef struct _VCA_GLOBAL {
	DEVICE_INFO	*DeviceList;	// List of all known devices
	SPIN_LOCK	Lock;			// Lock on device list
	EUI64		SystemImageGUID;	// global for platform
} VCA_GLOBAL;

#endif // _IBA_VCA_MAIN_H_
