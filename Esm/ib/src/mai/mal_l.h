/* BEGIN_ICS_COPYRIGHT2 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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

/****************************************************************************
 *                                                                          *
 * FILE NAME                                               VERSION          *
 *      mal_l.h                                            MAPI 0.05        *
 *                                                                          *
 * DESCRIPTION                                                              *
 *      This file contains  function prototypes used by the MIA adaptation  *
 *      layer code (MAL)                                                    *
 *                                                                          *
 *                                                                          *
 *                                                                          *
 * FUNCTIONS                                                                *
 *                                                                          *
 * DEPENDENCIES                                                             *
 *                                                                          * 
 *                                                                          *
 ****************************************************************************/

#ifndef _MAL_L_H_
#define _MAL_L_H_

#ifdef IB_STACK_OPENIB
#include <mal_g.h>
#endif

/*
 *- - - - - - - - - - - - - - - - - - - - - - - - - - -
 * Function Prototypes
 *- - - - - - - - - - - - - - - - - - - - - - - - - - -
 */

/*
 * ib_attach_sma
 *   Create a channel to send and receive 256 byte SMDs with the SMI or
 * GSI.
 *
 * INPUTS
 *      dev         ib device ordinal
 *      port        the port on the device
 *      qpn         Must be 0 (SMI) or 1 (GSI)
 *      handlep     Pointer to location to store the handle
 *
 * RETURNS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *      VSTATUS_NOPRIV
 *      VSTATUS_NODEV
 *      VSTATUS_NOPORT
 */
Status_t        ib_attach_sma(int16_t dev, int8_t port, uint32_t qpn,
			      IBhandle_t * handlep, uint8_t * nodeTypep);

/*
 * ib_detach_sma
 *   Close down a channel to send/receive SMAs.
 *
 * INPUTS
 *      handle      Handle returned by an ib_attach_sma()
 *
 * RETURNS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 */
Status_t        ib_detach_sma(IBhandle_t handle);

/*
 * ib_recv_sma
 *   Receive an SMD from an open (ib_attach_sma) channel.
 *
 * INPUTS
 *      handle      Handle returned by an ib_attach_sma()
 *      mad         Pointer to a fully loaded MAD
 *      timeout     Number of uSecs to wait for the event
 *
 * RETURNS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *      VSTATUS_TIMEOUT
 */
Status_t        ib_recv_sma(IBhandle_t handle, Mai_t * mad,
			    uint64_t timeout);

/*
 * ib_send_sma
 *   Send an SMD on an open (ib_attach_sma) channel.
 *
 * INPUTS
 *      handle      Handle returned by an ib_attach_sma()
 *      mad         Pointer to a fully loaded MAD
 *      timeout     Send timeout to pass down to kernel
 *
 * RETURNS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *      VSTATUS_NOMEM
 *
 * NOTES
 *   1. The lowest level ib_send_sma() function, the one that actually
 *      puts the MAD out on the IB hardware instead of piping it to a
 *      lower level mai_send() function, must wrap INTERNAL mads back into
 *      the incoming MAD stream.  This assures that messages can be
 *      sent to managers/agents at any level in the pipeline.
 */
Status_t        stl_send_sma(IBhandle_t handle, Mai_t * mad, uint64_t timeout);

/*
 *-----------------------------------------------------------------
 * ib_init_sma
 *-----------------------------------------------------------------
 *
 * INPUTS
 *      numMadBuffers   The number of MAD buffers to allocate
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_BAD
 *-----------------------------------------------------------------
 */
Status_t        ib_init_sma(uint32_t maxMadBuffers);

/*
 *-----------------------------------------------------------------
 * ib_shutdown_all
 *-----------------------------------------------------------------
 *
 * INPUTS
 *
 * OUTPUTS
 *-----------------------------------------------------------------
 */
void        ib_shutdown_all(void);
#endif				/* _MAL_L_H_ */
