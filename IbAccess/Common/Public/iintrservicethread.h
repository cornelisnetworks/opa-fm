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


#ifndef _IINTR_SERVICE_THREAD_H_
#define _IINTR_SERVICE_THREAD_H_

#include "iba/public/ithread.h"
#include "iba/public/ipci.h"
#include "iba/public/iintrservicethread_osd.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Interrupt Service Thread.
 * This allows the "slower" part of servicing of an interrupt to be
 * done in a higher level task.  On most OS's this is still a restricted
 * task which cannot sleep/wait or otherwise preempt itself
 *
 * Each instance could potentially be a separate thread, hence allowing
 * concurrent processing of different types of Interrupts without having
 * serious interrupts need to queue behind less severe interrupts.
 *
 * The interrupt handler should manage the low level hardware registers
 * to identify the cause of the interrupt then Signal the appropriate
 * IntrServiceThread.  The interrupt can then return (releasing the interrupt
 * lockout) and the IntrServiceThread callback will be run in a higher level
 * context to complete the processing of the interrupt.
 */

void	
IntrServiceThreadInitState( 
	IN INTR_SERVICE_THREAD *pIntrServiceThread );

boolean 
IntrServiceThreadCreate( 
	IN INTR_SERVICE_THREAD *pIntrServiceThread, 
	IN PCI_DEVICE *pDev,
	IN char* Name,
	IN THREAD_CALLBACK Callback, 
	IN void *Context );

/* Destroy must be called at passive level. */
void	
IntrServiceThreadDestroy( 
	IN INTR_SERVICE_THREAD *pIntrServiceThread );

/* This is Interrupt callable, it will run the callback on the
 * Interrupt Service Thread
 */
static _inline void	
IntrServiceThreadSignal( 
	IN INTR_SERVICE_THREAD *pIntrServiceThread );

#ifdef __cplusplus
}	/* extern "C" */
#endif

#endif /* _IINTR_SERVICE_THREAD_H_ */
