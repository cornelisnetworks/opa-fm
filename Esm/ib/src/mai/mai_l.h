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
 *      mai_l.h                                            MAPI 0.05        *
 *                                                                          *
 * DESCRIPTION                                                              *
 *      This file contains global function prototypes and structure         *
 *      definitions for the management API.                                 *
 *                                                                          *
 *                                                                          *
 * DATA STRUCTURES                                                          *
 *      mai_data                        The mbufs of the MAD world          *
 *      mai_fd                          Context for an up-channel           *
 *      mai_filter                      Filter description                  *
 *      mai_stats                       global stats description            *
 *      MAI_MAX_CHANNELS                Maximum channles we support         *
 *      MAI_MAX_FILTERS                 Maximum total filters we handle     *
 *      MAI_MAX_DATA                    Total number of queued MADs         *
 *      MAI_MAX_QUEUED                  Maximum queued MADs on one channel  *
 *                                                                          *
 * FUNCTIONS                                                                *
 *                                                                          *
 * DEPENDENCIES                                                             *
 *                                                                          *
 *                                                                          *
 * HISTORY                                                                  *
 *          NAME             DATE                REMARKS                    *
 *       Jim Mott          01-15-2001            Entry from prototype code  *
 *                                                                          *
 ****************************************************************************/

#ifndef _MAI_L_H_
#define _MAI_L_H_

#include <stdio.h>
#include <string.h>
#ifdef __VXWORKS__
#include "taskLib.h"
#endif
#include <ib_types.h>
#include <ib_status.h>
#include <cs_g.h>		/* Global common services functions */
#include <mai_g.h>		/* Global mai function definitions */
#include <cs_log.h>

#include "mai_info.h"		/* Statistics structures */
#include "vmai_stream.h"	/* Streams conversion calls */


#ifndef NULL
#define NULL (0)
#endif

/*
 * Change the definition of module id to that of MAI 
 */
#undef LOCAL_MOD_ID
#define LOCAL_MOD_ID VIEO_MAI_MOD_ID

/*
 * Local definitions
 *   MAI_FREE state for mai_data, mai_filter, mai_fd
 *   MAI_BUSY state for mai_data, mai_filter, mai_fd
 *   MAI_WAIT state for mai_fd
 */
#define MAI_FREE   (0)		/* Structure is unallocated */
#define MAI_BUSY   (1)		/* Structure is in use */
#define MAI_WAIT   (2)		/* Post to mai_fd.ecb to wake up */

/*
 * MAI_MAX_CHANNELS
 *   If not sent by the build environment, we set the maximum number of
 * upstream clients equal to this number.  The default depends on if we
 * are building explicitly un-threaded to save space.
 */
#ifndef MAI_MAX_CHANNELS
#define MAI_MAX_CHANNELS (128)	/* If threaded - then default */
#endif				/* MAI_MAX_CHANNELS */

/*
 * MAI_MAXDC_CHANNELS
 *   If not sent by the build environment, we set the maximum number of
 * downstream interfaces.  
 */
#ifndef MAI_MAXDC_CHANNELS
#define MAI_MAXDC_CHANNELS (1)	/* If threaded - then default */
#endif				/* MAI_MAX_CHANNELS */

/*
 * MAI_MAX_FILTERS
 *   The managment API handles a fixed maximum number of filters.  If the
 * build environment does not provide one, we set a reasonable default
 * here.
 */

#ifndef MAI_MAX_FILTERS
#define MAI_MAX_FILTERS (MAI_MAX_CHANNELS * 4)
#endif				/* MAI_MAX_FILTERS */

/*
 * MAI_MAX_EVENTS
 *   The number of event sets we will need for the handles.
 */

#ifndef MAI_MAX_EVENTS
#define MAI_MAX_EVENTS ((MAI_MAX_CHANNELS/VEVENT_NUM_EVENTS) + 1 )
#endif				/* MAI_MAX_EVENTS */


/*
 * Set the low water marks for resources .. so warnings 
 * can be generated. Use 10 percent level.
 */

#define MAI_FILTER_LOWWM (MAI_MAX_FILTERS/10)
#define MAI_HNDL_LOWWM   (MAI_MAX_CHANNELS/10)

/* note that this is now adjustable at init
   via mai_set_num_end_ports() */
#define MAI_MADS_LOWWM_DEFAULT (MAI_MAX_DATA_DEFAULT/10)

/*
 * Use the compile flag to create a dedicated Down call DC thread 
 */
#ifdef DEDICATED_DCTHREAD
#define MAI_DCTHREAD_OPTION 1
#else
#define MAI_DCTHREAD_OPTION 0
#endif

/*
 * mai_data
 *   This internal data structure describes exactly 256 bytes of a MAD.
 * A single MAD can not exceed 256 bytes, but there are multi-MAD 
 * management datagrams.  These are handled as well.
 */
struct mai_data {
    struct mai_data *next;	/* Next MAD */
    int             state;	/* Current state - FREE,BUSY */
    Mai_t           mad;	/* Actual data when BUSY or READY */
};

/*
 * mai_filter
 *   This internal data structure describes one filter that has been
 * applied to a channel.
 */
typedef struct mai_filter {
    struct mai_filter *next;	/* Used for other filters on chan */
    struct mai_filter *prev;	/* Prev of chain that filter is on */

    struct mai_fd  *owner;	/* Channel owning this filter */
    int             state;	/* State: FREE, BUSY */
    int             qp;		/* QP (smi/gsi) we are applying to */
    Filter_t        filter;	/* Actual filter specification */
    int             flags;	/* Flags used in filter spec */
    int             once;	/* True if this is one shot filter */
    IBhandle_t      hndl;	/* Filter handle */
#ifdef MAI_STATS
    int             matchcnt;	/* Number of matches on this filter */
#endif

} MaiFilterBuff_t;

#ifdef MAI_STATS
#define MSTATS_FMATCH_INCR(fm) fm->matchcnt++
#define MSTATS_FMATCH_CLR(fm)  fm->matchcnt = 0
#else
#define MSTATS_FMATCH_INCR(fm)
#define MSTATS_FMATCH_CLR(fm)
#endif

/*
 * Global statistics info 
 */

#ifdef MAI_STATS

#define MSTATS_FILT_FREE() gMAI_STATS.filt_free++;gMAI_STATS.filt_inuse--
#define MSTATS_FILT_USE()  gMAI_STATS.filt_free--;gMAI_STATS.filt_inuse++

#define MSTATS_DC_FREE() gMAI_STATS.dc_free++;gMAI_STATS.dc_inuse--
#define MSTATS_DC_USE()  gMAI_STATS.dc_free--;gMAI_STATS.dc_inuse++

#define MSTATS_DATA_FREE() gMAI_STATS.data_free++;gMAI_STATS.data_inuse--
#define MSTATS_DATA_USE()  gMAI_STATS.data_free--;gMAI_STATS.data_inuse++

#define MSTATS_UPCHAN_FREE(qp)  if(qp)gMAI_STATS.gsi--;else gMAI_STATS.smi--
#define MSTATS_UPCHAN_USE(qp)  if(qp)gMAI_STATS.gsi++;else gMAI_STATS.smi++

#define MSTATS_NORESOURCE_INCR() gMAI_STATS.no_resource++

#define MSTATS_INIT(){\
  gMAI_STATS.filt_max = gMAI_STATS.filt_inuse = MAI_MAX_FILTERS;\
  gMAI_STATS.data_max = gMAI_STATS.data_inuse = gMAI_MAX_DATA;   \
  gMAI_STATS.dc_max   = gMAI_STATS.dc_free    = MAI_MAXDC_CHANNELS;\
  gMAI_STATS.max_queue = gMAI_MAX_QUEUED;\
  gMAI_STATS.upchan_max= MAI_MAX_CHANNELS;\
}

#else
#define MSTATS_INIT()

#define MSTATS_FILT_FREE()
#define MSTATS_FILT_USE()

#define MSTATS_DC_FREE()
#define MSTATS_DC_USE()

#define MSTATS_NORESOURCE_INCR()

#define MSTATS_DATA_FREE()
#define MSTATS_DATA_USE()

#define MSTATS_UPCHAN_FREE(chan)
#define MSTATS_UPCHAN_FUSE(chan)

#endif

typedef struct {
    int             locked;	/* True if lock is held */
    Lock_t          lock;	/* The lock itself */
} MLock_t;

/*
 * mai_dc
 *   This internal data structure fully describes one open IB down 
 * channel.  Each combination CA, PORT, QP uses requires a seperate
 * hardware (down) channel.
 */
typedef struct mai_dc {
    struct mai_dc  *next;	/* Pointer to the next active one */
    int             state;	/* hardware (down) channel state: FREE/BUSY */
    int             hndl;	/* FD for down calls */
    int             ref;	/* Reference count */
    Threadname_t    readt;	/* Holds context for reading thread */
    int             readt_state;	/* Current state for this thread */
    int             waiters;	/* number of threads waiting on downcall */
    int             active;	/* true if thread is down there waiting */
    unsigned int    incarn;	/* the open incarnation number */

    MLock_t         lock;	/* DC handle lock */
#ifdef MAI_STATS
    mai_dc_stats_t  stats;	/* statistics of activity on this dc */
#endif
} MaiDc_t;

#define QPS_DEAD     (0)	/* Read process has died */
#define QPS_STARTED  (1)	/* Read process started */
#define QPS_RUNNING  (2)	/* Read process running */
#define QPS_DIE      (3)	/* Close process asked thread to die */
#define QPS_ERROR    (4)	/* Read process has errored out */
#define QPS_RESTART  (5)	/* Try to restart read proc to clr error */

#ifdef MAI_STATS
#define MSTATS_DC_TX(dc,type){                         \
    if(type == MAI_TYPE_EXTERNAL)dc->stats.external.tx++;        \
    else                           dc->stats.internal.tx++; \
}

#define MSTATS_DC_RX(dc,type){                         \
    if(type == MAI_TYPE_EXTERNAL)dc->stats.external.rx++;        \
    else                           dc->stats.internal.rx++; \
}

#define MSTATS_DC_CLR(dc) memset(&dc->stats,0,sizeof(dc->stats))
#else
#define MSTATS_DC_TX(dc,type)
#define MSTATS_DC_RX(dc,type)
#define MSTATS_DC_CLR(dc)
#endif

/*
 * mai_fd
 *   This internal data structure fully describes one open IB channel.  It
 * also points to 'ready to receive' data blocks.
 */
typedef struct mai_fd {
    struct mai_fd  *next;	/* Linked list of active channels */
    struct mai_fd  *prev;	/* Linked list of active channels */

    int             sfilt_cnt;	/* number of shared filters on handle */
    struct mai_filter *sfilters;	/* 
					 * Linked list of active shared filters
					 * owned by the handle
					 */

    struct mai_data *mad_hqueue;	/* Head Data ready to consume */
    struct mai_data *mad_tqueue;	/* Tail Data ready to consume */

    int             mad_cnt;	/* number of mads to be pick up */

    MLock_t         lock;	/* Lock for handle */

    int             state;	/* Current channel state -FREE/BUSY/WAIT */
    int             up_fd;	/* Our current FD */

    int             dev;	/* Device [0..n-1] where n adapters */
    int             port;	/* port [0..p-1] where p ports/adapter */
    int             qp;		/* QP [0,1] or SMI/GSI */
    uint8_t         nodeType;	/* Type of device below associated hdl */
    int             waiters;	/* number of callers waiting on handle */
    int             overflow;	/* number of mads dropped .. no place */

    struct mai_dc  *down_fd;	/* Channel to send and receive on */
    uint64_t        wakeup;	/* Absolute time to return by */

    Evt_handle_t    hdl_ehdl;	/* Used in threaded implementations */
    Eventset_t      hdl_emask;   /* Event mask to wait on/post         */

    unsigned int    incarn;	/* the open incarnation number */
#ifdef MAI_STATS
    mai_fd_stats_t  stats;	/* statistics on fd */
#endif
} mai_fd_t;

#ifdef MAI_STATS
#define MSTATS_FD_TX MSTATS_DC_TX
#define MSTATS_FD_RX MSTATS_DC_RX

#define MSTATS_FD_FILTADD(fd)  fd->stats.filtcnt++
#define MSTATS_FD_FILTREM(fd)  fd->stats.filtcnt--

#define MSTATS_FD_MADINCR(fd) fd->stats.madcnt++;
#define MSTATS_FD_MADDECR(fd) fd->stats.madcnt--;

#define MSTATS_FD_OVERFLOW(fd) fd->overflow++;

#define MSTATS_FD_CLR(fd) memset(&fd->stats,0,sizeof(fd->stats))
#else

#define MSTATS_FD_TX(fd,type)
#define MSTATS_FD_RX(fd,type)

#define MSTATS_FD_FILTADD(fd)
#define MSTATS_FD_FILTREM(fd)

#define MSTATS_FD_MADINCR(fd)
#define MSTATS_FD_MADDECR(fd)
#define MSTATS_FD_OVERFLOW(fd)
#define MSTATS_FD_CLR(fd)
#endif

/*
 * Values defining the level of equality between filters 
 */

#define MAI_FILTER_EQUAL     (1)	/* test filter same as reference
					 * filt */
#define MAI_FILTER_SUBSET   (-1)	/* test filter subset of ref filt */
#define MAI_FILTER_SUPSET   (-2)	/* test filter superset of ref
					 * filt */
#define MAI_FILTER_FUZZY    (-3)	/* filters have intertwined
					 * sub/supset */
#define MAI_FILTER_DISTINCT  (0)	/* test filter no relation to ref
					 * filt */

/*
 * Local function prototypes
 *
 * maif_init            Filter management subsystem initialization
 * maif_match           Compare MAD against a filter
 * maif_reduce          Compare 2 filters 
 * maif_special         Extract filter from special MAD
 * maif_make_special    Create special MAD from filter
 */
void            maif_init(void);
int             maif_match(Mai_t * data, Filter_t * filter);
int             maif_reduce(Filter_t * filt1, Filter_t * filt2);

/*
 * Static global data
 *   These data areas support the implementation of the API.
 *
 * gMAI_DOWN_CHANNELS[]
 *   Pointer to linked list of active (open) hardware (down) channel descriptors
 * used to send and receive SMDs.
 *
 * gMAI_DOWN_CL[]
 *   Array of MAI_MAX_CHANNEL channel descriptors available for down
 * channel info.
 *
 * gMAI_UP_CHANNELS[]
 *   Pointer to linked list of up channel open connections for both QP0
 * and QP1.  
 *
 * gMAI_CHANNELS[] 
 *   Array of MAI_MAX_CHANNEL channel descriptors.  These are either 
 * available or used a the rate of one per-connection.
 * 
 * gMAI_MAD_BUFFS[]
 *   Array of gMAI_MAX_DATA data blocks.  Messages received from below
 * are copied into these guys and chained of an upper channel descriptor.
 *
 * gMAI_FILTERS[]
 *   Array of MAI_MAX_FILTERS filter descriptors.  These are chained off
 * channel control blocks (mai_fd.root through mai_filter.next) and off
 * filter_absorb (filter_absorb through mai_filter.only).
 *
 * gmai_lock 
 *   Contains the only global lock in this function.  Used to serialize
 * access to all local resources.  A spin lock.
 *
 * gMAI_DATA_FREE
 *   Used to chain together all the currently available elements in
 * gMAI_MAD_BUFFS[] (through mai_data.next).
 *
 * gMAI_INITIALIZED
 *   Checks for recursive initialization calls and allows functions to
 * verify that initialization has taken place.
 */

extern struct mai_dc *gMAI_DOWN_CHANNELS;
extern struct mai_fd *gMAI_UP_CHANNELS;

extern struct mai_data *gMAI_DATA_FREE;
extern struct mai_fd *gMAI_HANDLE_FREE;
extern struct mai_filter *gMAI_FILT_FREE;

extern struct mai_fd gMAI_CHANNELS[MAI_MAX_CHANNELS];
extern struct mai_data * gMAI_MAD_BUFFS;
extern struct mai_filter gMAI_FILTERS[MAI_MAX_FILTERS];
extern struct mai_dc gMAI_DOWN_CL[MAI_MAXDC_CHANNELS];

extern int      gMAI_INITIALIZED,
                gMAI_FILT_CNT,
                gMAI_MAD_CNT,
                gMAI_HDL_CNT;

extern MLock_t  gmai_uplock,
                gmai_tidlock;
extern MLock_t  gmai_hlock,
                gmai_flock,
                gmai_mlock;
extern MLock_t  gmai_dcthr_lock;

#ifdef MAI_STATS
extern mai_stats_t gMAI_STATS;
#endif

extern Sema_t   gMAI_DCTHREAD_SEMA;
extern int      gMAI_USE_DEDICATED_DCTHREAD;
extern int      gMAI_DCTHREAD_HANDLE;

extern uint32_t gMAI_MAX_QUEUED;
extern uint32_t gMAI_MAX_DATA;

extern uint32_t gMAI_MADS_LOWWM;

#define MAI_INVALID (-1)

/*
 * Event post mask
 */

#define MAD_READY_EMASK  (0x001)
#define MAI_DC_EMASK     (0x002)
#define MAI_ANY_EMASK    (MAD_READY_EMASK |  MAI_DC_EMASK)

#ifndef MAI_FUNC
#define MAI_FUNC "mai_func:"
#endif

/*
 * This should be turned off once we are satisfied everything is Ok 
 */

#define MAI_ASSERT_TRUE(val) do{                               \
         if(!(val)){                                      \
            IB_LOG_ERROR0("Condition check failed");\
            IB_LOG_ERROR(MAI_FUNC,__LINE__);              \
            IB_FATAL_ERROR(__FILE__);                     \
           }}while(0)

#define MAI_ASSERT_LOCK_HELD(v) MAI_ASSERT_TRUE((v->lock.locked))

#if 0
#define LOCK_VERBOSE
#endif

#ifdef LOCK_VERBOSE
#define TRACK_LOCK(_p) IB_LOG_VERBOSELX("unlocked",_p);
#define TRACK_UNLOCK(_p) IB_LOG_VERBOSELX("locked",_p)
#else
#define TRACK_LOCK(_p)
#define TRACK_UNLOCK(_p)
#endif

#define MAI_LOCK(_p)do{                                          \
    int rc2;                                                   \
    if(((rc2=vs_lock(&_p->lock))!= VSTATUS_OK)){                 \
       IB_LOG_ERRORRC("LOCK acquire fail rc:",rc2);                   \
       IB_LOG_ERROR(MAI_FUNC,__LINE__);                          \
       mai_shut_down(); }                                        \
       _p->locked = 1;                                           \
       TRACK_LOCK((_p));                                         \
     }while(0)

#define MAI_UNLOCK(_p)do{                         \
    int rc2;                                      \
     MAI_ASSERT_TRUE(_p->locked);                      \
    _p->locked = 0;                               \
    TRACK_UNLOCK((_p));                           \
    if((rc2=vs_unlock(&_p->lock))!= VSTATUS_OK){  \
       IB_LOG_ERRORRC("LOCK free failed rc:",rc2);     \
       IB_LOG_ERROR(MAI_FUNC,__LINE__);           \
       mai_shut_down();}                          \
     }while(0)

/*
 * Locks in the MAI should be acquired in the following hierarchy -
 * 1. MAI_UPCHANNELS_LOCK
 * 2. MAI_HANDLE_LOCK and others.
 * Violation of this acquisition order will cause problems.
 */

#define MAI_HANDLE_LOCK(hdl)   do{ MAI_LOCK((&hdl->lock)); }while(0)
#define MAI_HANDLE_UNLOCK(hdl) do{ MAI_UNLOCK((&hdl->lock));}while(0)

#define MAI_UPCHANNELS_LOCK()   MAI_LOCK((&gmai_uplock))
#define MAI_UPCHANNELS_UNLOCK() MAI_UNLOCK((&gmai_uplock))

#define MAI_DC_LOCK(hdl)        MAI_LOCK((&hdl->lock))
#define MAI_DC_UNLOCK(hdl)      MAI_UNLOCK((&hdl->lock))

#define MAI_FILT_LOCK()         MAI_LOCK((&gmai_flock))
#define MAI_FILT_UNLOCK()       MAI_UNLOCK((&gmai_flock))

#define MAI_MBUFFS_LOCK()       MAI_LOCK((&gmai_mlock))
#define MAI_MBUFFS_UNLOCK()     MAI_UNLOCK((&gmai_mlock))

#define MAI_GHANDLES_LOCK()     MAI_LOCK((&gmai_hlock))
#define MAI_GHANDLES_UNLOCK()   MAI_UNLOCK((&gmai_hlock))

#define MAI_TID_LOCK()          MAI_LOCK((&gmai_tidlock))
#define MAI_TID_UNLOCK()        MAI_UNLOCK((&gmai_tidlock))

#define MAI_DCTHR_LOCK()          MAI_LOCK((&gmai_dcthr_lock))
#define MAI_DCTHR_UNLOCK()        MAI_UNLOCK((&gmai_dcthr_lock))

int             mai_get_dc(int dev, int port, int qp, struct mai_dc **out,
			   uint8_t * nodeType);
void            mai_free_dc(struct mai_dc *dc);

int             mai_validate_filter(Filter_t * filter);

struct mai_data *mai_alloc_mbuff(Mai_t * rawmad);
void            mai_free_mbuff(struct mai_data *fmad);

int             mai_enqueue_mbuff(struct mai_data *mad,
				  struct mai_fd *chan);
struct mai_data *mai_dequeue_mbuff(struct mai_fd *chan);

struct mai_fd  *mai_alloc_handle(void);
Status_t        mai_free_handle(struct mai_fd *fd);

void            mai_enqueue_upchannel(struct mai_fd *new_fd);
void            mai_dequeue_upchannel(struct mai_fd *free_fd);

int             mai_mad_process(Mai_t * mad, int *filterMatch);

int             mai_getqp(struct mai_fd *act, uint64_t wakeup);

void            mai_filter_purge(struct mai_fd *act, Filter_t * ft);

struct mai_filter *mai_alloc_filter(void);
void            mai_free_filter(struct mai_filter *filt);

Status_t        mai_load_filter(Filter_t * in, struct mai_filter **pout);

struct mai_filter *mai_filter_dequeue(struct mai_fd *chanp,
				      struct mai_filter *filt);
void            mai_filter_enqueue(struct mai_fd *chanp,
				   struct mai_filter *filt);

Status_t        mai_filter_find(struct mai_fd *act,
				Filter_t * ref,
				int flag,
				struct mai_filter **match,
				struct mai_filter **prev);

void            mai_shut_down(void);

void mai_close_portinfo(void);

#endif				/* _MAI_L_H_ */
