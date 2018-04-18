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

#ifndef __OPAMGT_PRIV_EA_H__
#define __OPAMGT_PRIV_EA_H__

#include "stl_mad_priv.h"
#include <opamgt_priv.h>
#include <iba/ipublic.h>
#include <iba/stl_sd.h>
#include <iba/stl_ea_eostl.h>

/** =========================================================================
 * OPAMGT EA interface
 */

/**
 * @brief Initialize EA connection on existing omgt_port session
 *
 * @param port                      omgt_port session to start EA interface on
 *
 * @return
 *  OMGT_SERVICE_STATE_OPERATIONAL - initialization successful
 *  	   OMGT_SERVICE_STATE_DOWN - initialization not successful/EaServer not
 *  								 available
 *  OMGT_SERVICE_STATE_UNAVAILABLE - ES Service not found
 */
int
omgt_ea_service_connect(struct omgt_port *port);

/**
 * @brief Get LID and SL of master EM service
 *
 * This function will query the SA service records to get the EM's GID. It will
 * then construct a PATH record query (using the ServiceGID as the DGID for the
 * request) to get the EM's LID and SL for sending PA queries.
 *
 * @param port         port object to to store master EM LID and SL.
 *
 * @return FSTATUS
 */
FSTATUS
iba_ea_query_master_em_lid(struct omgt_port  *port);

/**
 * Pose a query to the fabric, expect a response.
 *
 * @param port - port opened by omgt_open_port_*
 * @param pQuery - pointer to the query structure
 * @param ppQueryResult - pointer where the response will go
 *
 * @return - 0 if success, else error code
 */
FSTATUS omgt_query_ea(struct omgt_port *port, OMGT_QUERY *pQuery, struct _QUERY_RESULT_VALUES **ppQueryResult);

#endif /* __OPAMGT_PRIV_EA_H__ */
