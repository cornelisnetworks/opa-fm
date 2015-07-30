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
// this file has MAC OS specific defines/config settings

// TBD we may not need these
#define __SMP__
#define CONFIG_SMP 1

#define UTS_RELEASE "10.3"

// enable /proc simulation
#define CONFIG_PROC_FS 1

#define HAS_KALLOC 				1
#define HAS_CPU_NUMBER				1	//	For cpu_number()
#define HAS_KERNEL_FUNNEL 			1	//	For CNetworkInterface
#define HAS_OPAQUE_PROCPTR 		0	
#define HAS_KERNEL_SOCKETS 			1	//	For CNetworkInterface
#define HAS_ENABLE_PREEMPTION	1
#define HAS_ABSOLUTETIME		1	// 	All the clock calls in Tiger use uint64
#define HAS_THREAD_DEADLINE		1	//  Has routines like thread_set_timer_deadline()...
#define HAS_NETWORK_LOGGING		1	//	Do we use CNetworkInterface
#define HAS_PANTHER_DEBUGGING	1	//	Supports CTrace::BackTrace(), CTrace::PointerCheck()
#define HAS_HOSTNAME			1	//	For the hostname variable in the kernel
#define HAS_MEMORY_GLOBALS		1
#define NEEDS_VSNPRINTF_EXTERNED	1
#define NEEDS_REPORT_LUNS_DEFS	1
#define HAS_MBUF_FUNCS			0
#define HAS_IFNET_T			0
