/* BEGIN_ICS_COPYRIGHT2 ****************************************

Copyright (c) 2015-2017, Intel Corporation

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

#ifndef __OPAMGT_NOTICE_H__
#define __OPAMGT_NOTICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <opamgt.h>
#include <opamgt_sa.h>

/**
 * @brief Initiates a registration for the specified trap.
 *
 * @param port      Previously initialized port object for an in-band
 *  				connection.
 * @param trap_num  Trap Number to resgister for.
 * @param context   optional opaque info to be retruned when trap is recieved.
 *
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_sa_register_trap(struct omgt_port *port, uint16_t trap_num,
	void *context);

/**
 * @brief Unregisters for the specified trap.
 *
 * @param port      Previously initialized port object for an in-band
 *  				connection.
 * @param trap_num  Trap Number to unresgister.
 *
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_sa_unregister_trap(struct omgt_port *port, uint16_t trap_num);

/**
 * @brief Gets the Notice Report forwarded from the SM
 *
 * @param port             Previously initialized port object for an in-band
 *  					   connection.
 * @param notice           Pointer to Notice structure that is allocate and
 *  					   returned. Must be freed by user.
 * @param notice_len       pointer to length of Notice structure to be returned.
 *  					   Length should always be greater than or equal to the
 *  					   sizeof(STL_NOTICE). All bytes that exist greather
 *  					   than sizeof(STL_NOTICE) are for ClassData (i.e.
 *  					   ClassData Length = notice_len - sizeof(STL_NOTICE)).
 * @param context          pointer to registration context value returned.
 * @param poll_timeout_ms  Length of time this function will poll (wait) for a
 *  					   Notice Report to be received in milliseconds (-1 will
 *  					   block indefinitely, 0 will not block, and X > 0 will
 *  					   block for X).
 *
 * @return OMGT_STATUS_T
 */
OMGT_STATUS_T omgt_sa_get_notice_report(struct omgt_port *port, STL_NOTICE **notice,
	size_t *notice_len, void **context, int poll_timeout_ms);

#ifdef __cplusplus
}
#endif
#endif /* __OPAMGT_NOTICE_H__ */
