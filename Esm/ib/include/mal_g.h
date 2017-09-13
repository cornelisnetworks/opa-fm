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

/*
 * MAL - MAI Abstraction Layer
 *
 * These are the public interfaces to the MAL layer.  Historically, this
 * layer represented the interface between MAI and the next layer down (CAL).
 *
 * As of the port to OFED, CAL has been eliminated, and the MAL layer now
 * interfaces with the IB stack directly.
 */

#ifndef _IB_MAL_G_H_
#define _IB_MAL_G_H_

#include <ib_types.h>

#ifndef __VXWORKS__
#include "opamgt_priv.h"
#endif

extern int ib_instrumentJmMads;


#ifndef __VXWORKS__
/*
 * ib_init_devport
 *
 * Initializes stack interface and binds to the specified
 * device and port.
 * If *Guidp is 0, *devp and *portp are used to select the device and port
 *      and *Guidp is returned.
 * If *Guidp is not 0, *Guid is used to select the device and port
 *		and *devp and *portp are returned.
 * Guidp may be NULL and/or devp and portp may be NULL
 */
Status_t ib_init_devport(uint32_t *dev, uint32_t *port, uint64_t *Guid, struct omgt_params *session_params);

/*
 * ib_get_devport
 *
 * Returns current porthandle
 */
struct omgt_port * ib_get_devport(void);
#else
/*
 * ib_init_devport
 *
 * Initializes stack interface and binds to the specified
 * device and port.
 * If *Guidp is 0, *devp and *portp are used to select the device and port
 *      and *Guidp is returned.
 * If *Guidp is not 0, *Guid is used to select the device and port
 *		and *devp and *portp are returned.
 * Guidp may be NULL and/or devp and portp may be NULL
 */
Status_t ib_init_devport(uint32_t *dev, uint32_t *port, uint64_t *Guid);

#endif

/*
 * ib_refresh_devport
 *
 * refreshes pkeys for stack interface
 */
Status_t ib_refresh_devport(void);

/*
 * ib_shutdown
 *
 * De-initializes stack interface (closes OFED port and releases 'issm' device)
 */
Status_t ib_shutdown(void);

/*
 * ib_register_sm
 *
 * Registers the SM with the MAL layer.
 */
Status_t ib_register_sm(int queue_size);

/*
 * ib_register_fe
 *
 * Registers the FE with the MAL layer.
 */
Status_t ib_register_fe(int queue_size, uint8_t thread);

/*
 * ib_register_pm
 *
 * Registers the PM with the MAL layer.
 */
Status_t ib_register_pm(int queue_size);

/*
 * ib_disable_is_sm
 *
 * Enables the IS_SM capability flag for the currently bound port.
 */
Status_t ib_enable_is_sm(void);

/*
 * ib_disable_is_sm
 *
 * Disables the IS_SM capability flag on the currently active port.
 */
Status_t ib_disable_is_sm(void);

#endif // _IB_MAL_G_H_ 
