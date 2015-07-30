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

/****************************************************************************
 *                                                                          *
 * FILE NAME                                               VERSION          *
 *      mai_info.h                                         MAPI 0.05        *
 *                                                                          *
 * DESCRIPTION                                                              *
 *      This file contains global function prototypes and structure         *
 *      definitions for querying the management API.                        *
 *                                                                          *
 *                                                                          *
 *                                                                          *
 ****************************************************************************/

#ifndef _MAI_INFO_H_
#define _MAI_INFO_H_

#include <stdio.h>
#include <string.h>
#include <ib_types.h>
#include <ib_status.h>
#include <cs_g.h>		/* Global common services functions */
#include <cs_log.h>



/*
 * The current version the modules are linked against 
 */
#define MAI_STATS_VERSION (1)

/*
 * mai_stats_t This stucture holds the global statistics info about the
 * resource useage in a given layer of MAI. 
 */

typedef struct {
    int             filt_free;	/* Number of free mai_filter */
    int             filt_inuse;	/* mia_filter in use */
    int             filt_max;	/* Maximum number of filters */

    int             data_free;	/* mai_data available for mads etc */
    int             data_inuse;	/* in use mai_data */
    int             data_max;	/* Max number of data avail */

    int             dc_free;	/* available hardware (down) channels */
    int             dc_inuse;	/* inuse hardware (down) channels */
    int             dc_max;	/* Max dc available.  */

    int             no_resource;	/* Number of times no resource
					 * occured */
    int             max_queue;	/* max message that can be queued */

    int             upchan_max;	/* Maximum number of upchannels */
    int             smi;	/* number of up channels on SMI */
    int             gsi;	/* number of up channels on GSI */
} mai_stats_t;

typedef struct {
    int             tx;		/* number of transmissions */
    int             rx;		/* number of receptions */
} mai_traffic_t;

/*
 * mai_dc_stats_t This structure holds the counters monitored by a given
 * hardware (down) channel 
 */

typedef struct {
    mai_traffic_t   external;	/* Number of EXTERNAL mads sent/recv on this dc */
    mai_traffic_t   internal;	/* Number of INTERNAL mads sent/recv on this dc */
} mai_dc_stats_t;

/*
 * mai_fd_stats_t This structure holds the counts monitored on given
 * handle associated with an mai_open call. 
 */
typedef struct {
    int             filtcnt;	/* Number on filters chained to fd */
    int             madcnt;	/* Mads chained to data for collection */
    mai_traffic_t   external;	/* Number of EXTERNAL mads sent/recv on this fd */
    mai_traffic_t   internal;	/* Number of INTERNAL mads sent/recv on this fd */
} mai_fd_stats_t;

#endif
