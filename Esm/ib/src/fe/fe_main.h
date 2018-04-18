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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/************************************************************************
* 
* FILE NAME
*       fe_main.h
*
* DESCRIPTION
*       Header file for Fabric Executive
*
* DEPENDENCIES
*
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   01/17/2000      Initial file for checkin to source control
* jrw   12/04/2001      Changes from code review 
*
***********************************************************************/

#ifndef FEMAIN_H
#define FEMAIN_H

/* Standard include libs                */
#ifdef __LINUX__
#include <stdlib.h>
#include <getopt.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#endif
#ifdef __VXWORKS__
#include <stdlib.h>
#include <time.h>
#include <types.h>
#include <errno.h>
#include <sys/stat.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>

#endif

#include <stdio.h>
#include "ib_types.h"
#include "ib_status.h"
#include "net/libnet.h"
#include "fe_cmds.h"
#include "mai_g.h"
#include "if3.h"
#include <vs_g.h>
#include <cs_log.h>
#include <fm_xml.h>

/* Change the definition of module id to that of FE */
#undef  LOCAL_MOD_ID
#define LOCAL_MOD_ID VIEO_FE_MOD_ID

#define SUCCESS   0
#define FAILED    1
#define LOGINVAL  2

#define FE_BUSY_RETRIES 4       /* How many times to retry              */
#define ACTIVE          0x01    /* Conn/Seq is active                   */
#define PENDING         0x02    /* Pending sequence                     */
#define COMPLETE        0x04    /* Completed sequence                   */
#define UNLOGGED        0x08    /* Connected but not logged in          */
#define BAD_CONN        0x10    /* Connection is bad or doesn't exist   */
#define CON_DISC        0x20    /* Schedule a port to disconnect        */
#define CONN_UNCHANGED  0   /* Connection is unchanged      */
#define CONN_LOST       1   /* Connection was lost          */
#define CONN_ESTABLISH  2   /* Connection re-established        */ 

#define BLOCK           1       /* Defines if the net code should block */
#define SLEEP_TIME      100     /* Amount of milliseconds to sleep      */
#define INVALID_HANDLE  (-1)

/* externs */
extern STL_LID         pm_lid;
extern STL_LID         ea_lid;
extern STL_LID         dm_lid;
extern STL_LID         bm_lid;
extern int32_t         ConnDisconnect;
extern int32_t         Shutdown;
extern int32_t         fe_nodaemon;
extern uint32_t        fe_log_to_console;
extern  char           fe_config_filename[256];
extern uint32_t        fe_log_level_override;
extern uint32_t        fe_log_level_arg;
extern uint32_t        fe_log_masks[VIEO_LAST_MOD_ID+1];
extern char            fe_env_str[32];
extern uint32_t        fe_in_buff_size;
extern uint32_t        fe_prts_size;
extern uint32_t        fe_lnks_size;
extern int             fe_rmpp_usrid;
extern Pool_t fe_pool; /* Pointer to Fab_exec memory pool */
extern IBhandle_t fdsa; /* Handle used in speaking to SA */ 

#define STL_BUF_RECV_SIZE       (fe_in_buff_size + fe_prts_size + fe_lnks_size)
#define STL_BUF_OOB_SEND_SIZE   (fe_in_buff_size + fe_prts_size + fe_lnks_size)

/* Connection list Structure    */
typedef struct _FE_ConnList {
    NetConnection   *conn;
    uint32_t         state;
    struct _FE_ConnList *prev;
    struct _FE_ConnList *next;
} FE_ConnList;

/* Error checking macros for pack func  */
#define PCHK(s,r,t)                                     \
{                                                       \
    if(s)                                               \
    {                                                   \
         IB_LOG_ERROR("Error Pack Failure",r);          \
         IB_EXIT(#t,r);                                 \
         return(r);                                     \
    }                                                   \
}
#define UPCHK(s,r,t,c)                                  \
{                                                       \
    if(s)                                               \
    {                                                   \
         IB_LOG_ERROR("Error Unpack Failure:",r);        \
         IB_EXIT(#t,r);                                 \
         fe_callBack(c);                                \
         return(r);                                     \
    }                                                   \
}

#define FE_VALID_CACHE(c) (c && c->fe_in_buf_timestamp && c->fe_in_buff && c->fe_nodes_len)

/* debug trace control for embeded FE */
#define _FE_TRACE(l) (l == 0) ? VS_LOG_VERBOSE : VS_LOG_INFINI_INFO

/*
 * FE debug flag for embeded 
 * re-routes all verbose messages to info level for embeded
 */
void feDebugOn(void);
void feDebugOff(void);
uint32_t feDebugGet(void);

/* Function prototypes  */
uint32_t  fe_init(void);
void fe_callBack(NetConnection *);
int fe_main(void);

/* External Functions   */
extern uint32_t pack_64(uint64_t data, uint8_t *buf, uint32_t blen, int32_t *len);
extern uint32_t pack_32(uint32_t data, uint8_t *buf, uint32_t blen, int32_t *len);
extern uint32_t pack_16(uint16_t data, uint8_t *buf, uint32_t blen, int32_t *len);
extern uint32_t pack_8(uint8_t data, uint8_t *buf, uint32_t blen, int32_t *len);
extern uint32_t pack_string(uint8_t *str, uint8_t *buf, uint32_t blen, int32_t *len);
extern uint32_t pack_data(uint8_t *data, uint8_t *buf, uint32_t blen, uint32_t dlen, int32_t *len);
extern uint32_t unpack_64(uint64_t *data, uint8_t *buf, int32_t *len, uint32_t plen);
extern uint32_t unpack_32(uint32_t *data, uint8_t *buf, int32_t *len, uint32_t plen);
extern uint32_t unpack_16(uint16_t *data, uint8_t *buf, int32_t *len, uint32_t plen);
extern uint32_t unpack_8(uint8_t *data, uint8_t *buf, int32_t *len, uint32_t plen);
extern uint32_t unpack_string(uint8_t *str, uint8_t *buf, int32_t *len, uint32_t plen);
extern uint32_t unpack_data(uint8_t *data, uint8_t *buf, uint32_t dlen, int32_t *len, uint32_t plen);
extern uint32_t getPayloadLength(uint8_t *);
extern uint32_t fe_process_fec_command(uint8_t *,FE_ConnList*);
extern uint32_t fe_vieo_init(uint8_t *);
extern void fe_shutdown(void);
extern void FE_db_init(void);

#endif
