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

/* Main header for OS Abstraction layer */

#ifndef _IBA_IPUBLIC_H_
#define _IBA_IPUBLIC_H_

#include "iba/public/datatypes.h"
#include "iba/public/ibyteswap.h"
#ifndef OPEN_SOURCE_12200
#include "iba/public/iarray.h"
#include "iba/public/ibitvector.h"
#include "iba/public/icmdthread.h"
#include "iba/public/idebug.h"
#include "iba/public/ievent.h"
#include "iba/public/ieventthread.h"
#include "iba/public/iheapmanager.h"
#include "iba/public/imutex.h"
#include "iba/public/iobjmgr.h"
#include "iba/public/isemaphore.h"
#include "iba/public/ithread.h"
#endif
#ifndef IB_STACK_OPENIB
#include "iba/public/igrowpool.h"
#endif
#include "iba/public/ilist.h"
#ifndef IB_STACK_OPENIB
#include "iba/public/ihandletrans.h"
#endif
#if defined(VXWORKS)
#include "iba/public/iintrservicethread.h"
#endif
#ifndef IB_STACK_OPENIB
#include "iba/public/imap.h"
#endif
#include "iba/public/imath.h"
#include "iba/public/imemory.h"
#include "iba/public/imemtrack.h"
#ifndef IB_STACK_OPENIB
#include "iba/public/ipci.h"
#endif
#include "iba/public/iquickmap.h"
#if defined(VXWORKS)
#include "iba/public/ireaper.h"
#endif
#ifndef IB_STACK_OPENIB
#include "iba/public/ireqmgr.h"
#include "iba/public/iresmap.h"
#include "iba/public/iresmgr.h"
#endif
#include "iba/public/ispinlock.h"
#ifndef IB_STACK_OPENIB
#include "iba/public/isyscallback.h"
#endif
#if defined(VXWORKS)
#include "iba/public/isysutil.h"
#endif
#ifndef IB_STACK_OPENIB
#include "iba/public/ithreadpool.h"
#endif
#include "iba/public/itimer.h"
#include "iba/public/statustext.h"

#endif /* _IBA_IPUBLIC_H_ */
