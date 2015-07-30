/* BEGIN_ICS_COPYRIGHT6 ****************************************

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

** END_ICS_COPYRIGHT6   ****************************************/

#include "statustext.h"

// Status values above converted to text for easier printing.
const char* FStatusText[] =
{
	"FSUCCESS",
	"FERROR",
	"FINVALID_STATE",
	"FINVALID_OPERATION",
	"FINVALID_SETTING",
	"FINVALID_PARAMETER",
	"FINSUFFICIENT_RESOURCES",
	"FINSUFFICIENT_MEMORY",
	"FCOMPLETED",
	"FNOT_DONE",
	"FPENDING",
	"FTIMEOUT",
	"FCANCELED",
	"FREJECT",
	"FOVERRUN",
	"FPROTECTION",
	"FNOT_FOUND",
	"FUNAVAILABLE",
	"FBUSY",
	"FDISCONNECT",
	"FDUPLICATE",
	"FPOLL_NEEDED"
};

const char* FCmStatusText[] =
{
	"FCM0",
	"FCM_NOT_INITIALIZED",
	"FCM_INVALID_HANDLE",
	"FCM_ADDR_INUSE",
	"FCM_INVALID_EVENT",
	"FCM_ALREADY_DISCONNECTING",
	"FCM_CONNECT_DESTROY",
	"FCM7",
	"FCM8",
	"FCM9",
	"FCM_CONNECT_TIMEOUT",
	"FSIDR_REQUEST_TIMEOUT",
	"FCM12",
	"FCM_CONNECT_REJECT",
	"FCM_CONNECT_CANCEL",
	"FCM15",
	"FCM16",
	"FCM17",
	"FCM18",
	"FCM19",
	"FCM_CONNECT_REPLY",
	"FCM_DISCONNECT_REQUEST",
	"FCM_DISCONNECT_REPLY",
	"FCM_CONNECT_REQUEST",
	"FCM_CONNECT_ESTABLISHED",
	"FCM_DISCONNECTED",
	"FSIDR_REQUEST",
	"FSIDR_RESPONSE",
	"FSIDR_RESPONSE_ERR",
	"FCM_CA_REMOVED",
	"FCM_ALTPATH_REQUEST",
	"FCM_ALTPATH_REPLY",
	"FCM_ALTPATH_REJECT",
	"FCM_ALTPATH_TIMEOUT",
	"FCM34",
	"FCM35",
	"FCM36",
	"FCM37",
	"FCM38",
	"FCM39",
	"FCM40",
	"FCM41",
	"FCM42",
	"FCM43",
	"FCM44",
	"FCM45",
	"FCM46",
	"FCM47",
	"FCM48",
	"FCM49",
	"FCM50",
	"FCM51",
	"FCM52",
	"FCM53",
	"FCM54",
	"FCM55",
	"FCM56",
	"FCM57",
	"FCM58",
	"FCM59",
	"FCM60",
	"FCM61",
	"FCM62",
	"FCM63",
	"FCM_WAIT_OBJECT0",
	"FCM_WAIT_OBJECT1",
	"FCM_WAIT_OBJECT2",
	"FCM_WAIT_OBJECT3",
	"FCM_WAIT_OBJECT4",
	"FCM_WAIT_OBJECT5",
	"FCM_WAIT_OBJECT6",
	"FCM_WAIT_OBJECT7",
	"FCM_WAIT_OBJECT8",
	"FCM_WAIT_OBJECT9"
};

const char* iba_fstatus_msg(FSTATUS status)
{
	return FSTATUS_MSG(status);
}
