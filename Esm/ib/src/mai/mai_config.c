/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/****************************************************************************
 *                                                                          *
 * FILE NAME                                               VERSION          *
 *      mai_config.c                                       MAPI 0.05        *
 *                                                                          *
 * DESCRIPTION                                                              *
 *      This file contains the code for initializing the mai subsystem      *
 *                                                                          *
 *                                                                          *
 * DATA STRUCTURES                                                          *
 *      None                                                                *
 *                                                                          *
 ****************************************************************************/

#include <stdio.h>
#include <string.h>

#include <ib_types.h>
#include <ib_status.h>
#include <mai_g.h>		/* Global mai function definitions */
#include <cs_g.h>		/* Global common services functions */
#include "mai_info.h"		/* Statistics structures */
#include "mai_l.h"		/* Local mai function definitions */
#include "vmai_stream.h"	/* Streams conversion calls */

struct mai_dc  *gMAI_DOWN_CHANNELS;
struct mai_fd  *gMAI_UP_CHANNELS;
struct mai_data *gMAI_DATA_FREE;
struct mai_filter *gMAI_FILT_FREE;
struct mai_fd  *gMAI_HANDLE_FREE;

struct mai_fd   gMAI_CHANNELS[MAI_MAX_CHANNELS];
struct mai_data * gMAI_MAD_BUFFS = NULL;
struct mai_filter gMAI_FILTERS[MAI_MAX_FILTERS];
struct mai_dc   gMAI_DOWN_CL[MAI_MAXDC_CHANNELS];
int             gMAI_INITIALIZED = 0;

int             gMAI_FILT_CNT,
                gMAI_MAD_CNT,
                gMAI_HDL_CNT;

MLock_t         gmai_uplock,
                gmai_tidlock;
MLock_t         gmai_hlock,
                gmai_mlock,
                gmai_flock;
MLock_t			gmai_dcthr_lock;

/* used to sync start of dedicated dc router thread */ 
Sema_t			gMAI_DCTHREAD_SEMA;	 
int             gMAI_USE_DEDICATED_DCTHREAD = MAI_DCTHREAD_OPTION;
int             gMAI_DCTHREAD_HANDLE;

uint32_t        gMAI_MAX_QUEUED = MAI_MAX_QUEUED_DEFAULT;
uint32_t        gMAI_MAX_DATA = MAI_MAX_DATA_DEFAULT;

uint32_t        gMAI_MADS_LOWWM = MAI_MADS_LOWWM_DEFAULT;

#ifdef MAI_STATS
mai_stats_t     gMAI_STATS;
#endif

static Event_t ev_array[MAI_MAX_EVENTS];

extern int         mai_dc_read_exit;    // tells dc thread to shutdown


/*
 * FUNCTION
 *      mai_set_num_end_ports
 *
 * DESCRIPTION
 *      This function can be called prior to calling mai_init.  It will
 *      adjust the MAD buffer size and queue depths accordingly.  If not
 *      called, they will be given reasonable defaults.
 *
 * 
 * INPUTS
 *      num_end_ports    The required number of fabric end ports to scale to.
 *
 * OUTPUTS
 *
 */
void mai_set_num_end_ports(uint32_t num_end_ports)
{
    IB_ENTER(__func__, num_end_ports, 0, 0, 0);
	
	if (gMAI_INITIALIZED)
	{
		IB_LOG_INFO("MAI already initialized; skipping...", 0);
		return;
	}
		
	gMAI_MAX_QUEUED = num_end_ports;
	gMAI_MAX_DATA = 5 * gMAI_MAX_QUEUED;
	
	// see mai_l.h for other non-adjustable low watermarks
	gMAI_MADS_LOWWM = gMAI_MAX_DATA / 10;
	
	IB_EXIT(__func__, 0);
}


/*
 * FUNCTION
 *      mai_init
 *
 * DESCRIPTION
 *      This function initializes the internal data areas used by the VIEO
 *      management API.  It must be called exactly once, prior to using any
 *      other management API functions.
 *
 * 
 * INPUTS
 *      None
 *
 * OUTPUTS
 *
 * HISTORY
 *      NAME      DATE          REMARKS
 *      JMM     01/16/01        Initial entry
 */
static Threadname_t mai_init_pid=0;

void
mai_init()
{
    int             rc;
    int             i;
    Threadname_t    my_pid;
    uint8_t         name[16];

    IB_ENTER(__func__, 0, 0, 0, 0);

    /*
     *   Check to see if we have been gMAI_INITIALIZED yet.  Since there can be a race
     *   condition when creating a lock, we can't do that here.  What we do instead
     *   is have all 'N' contending processes write to a static variable and then go
     *   to sleep.  After they wake up, only one will have the variable.  That
     *   thread goes on.
     *
     *   If the gMAI_INITIALIZED variable is zero, then there can be a thread in the
     *   process of initialization.  In this case, we just wait here until the other
     *   thread finishes.
     */

   (void) vs_thread_name(&my_pid);

    if (gMAI_INITIALIZED == 0)
      {
	  if (mai_init_pid == 0ull)
	    {
		mai_init_pid = my_pid;
		(void)vs_thread_sleep(VTIMER_1S);
	    }

	  i = 0;

	  if (mai_init_pid != my_pid)
	    {
		while (gMAI_INITIALIZED == 0 && i < 30)
		  {
		      (void)vs_thread_sleep(VTIMER_1S);
		      i++;
		  }

		if (gMAI_INITIALIZED == 0)
		  {
		      IB_LOG_ERROR0("failed to initialized");
		  }
		return;
	    }
      }
    else
      {
	  IB_LOG_INFO("Already initialized", 0);
	  return;
      }

    /*
     * Make sure we belong here 
     */
    if (gMAI_INITIALIZED)
      {
	  IB_LOG_INFO("Already initialized", 0);
	  return;
      }

    /*
     * Reset these values 
     */

    gMAI_FILT_CNT = gMAI_MAD_CNT = gMAI_HDL_CNT = 0;

    MSTATS_INIT();

    /*
     * Assume threads will not work - Unless changed later on 
     */

    /*
     * Basic initialization 
     */
    gMAI_UP_CHANNELS = NULL;
    gMAI_DATA_FREE = NULL;
    gMAI_HANDLE_FREE = NULL;
    gMAI_FILT_FREE = NULL;

    gMAI_DCTHREAD_HANDLE = MAI_INVALID;
	// initialize read semaphore
	cs_sema_create (&gMAI_DCTHREAD_SEMA, 0);

    mai_dc_read_exit = 0;

    /*
     * Initialize the one and only lock 
     */
    (void)memset(&gmai_uplock, 0, sizeof(MLock_t));
    (void)memset(&gmai_hlock, 0, sizeof(MLock_t));
    (void)memset(&gmai_flock, 0, sizeof(MLock_t));
    (void)memset(&gmai_mlock, 0, sizeof(MLock_t));
    (void)memset(&gmai_tidlock, 0, sizeof(MLock_t));
    (void)memset(&gmai_dcthr_lock, 0, sizeof(MLock_t));

    rc = vs_lock_init(&gmai_uplock.lock, VLOCK_FREE, VLOCK_THREAD);
    if (rc)
      {
	  IB_LOG_ERRORRC("vs_lock_init rc:", rc);
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return;
      }

    rc = vs_lock_init(&gmai_hlock.lock, VLOCK_FREE, VLOCK_THREAD);
    if (rc)
      {
	  IB_LOG_ERRORRC("vs_lock_init rc:", rc);
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return;
      }

    rc = vs_lock_init(&gmai_flock.lock, VLOCK_FREE, VLOCK_THREAD);
    if (rc)
      {
	  IB_LOG_ERRORRC("vs_lock_init rc:", rc);
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return;
      }

    rc = vs_lock_init(&gmai_mlock.lock, VLOCK_FREE, VLOCK_THREAD);
    if (rc)
      {
	  IB_LOG_ERRORRC("vs_lock_init rc:", rc);
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return;
      }

    rc = vs_lock_init(&gmai_tidlock.lock, VLOCK_FREE, VLOCK_THREAD);
    if (rc)
      {
	  IB_LOG_ERRORRC("vs_lock_init rc:", rc);
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return;
      }

    rc = vs_lock_init(&gmai_dcthr_lock.lock, VLOCK_FREE, VLOCK_THREAD);
    if (rc)
      {
	  IB_LOG_ERRORRC("gmai_dcthr_lock vs_lock_init rc:", rc);
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return;
      }

    (void)memset(gMAI_DOWN_CL, 0, sizeof(gMAI_DOWN_CL));

    /*
     * Initialize the hardware channel control blocks 
     */
    for (i = 0; i < MAI_MAXDC_CHANNELS; i++)
      {
	  struct mai_dc  *p;	/* To ease typing */

	  p = &gMAI_DOWN_CL[i];

	  p->next = NULL;
	  p->state = MAI_FREE;
	  p->ref = 0;
	  p->readt_state = QPS_DEAD;

	  rc = vs_lock_init(&p->lock.lock, VLOCK_FREE, VLOCK_THREAD);
	  if (rc)
	    {
		IB_LOG_ERRORRC("vs_lock_init rc:", rc);
		IB_EXIT(__func__, VSTATUS_BAD);
		return;
	    }
      }


    (void)memset(ev_array, 0, sizeof(ev_array));

    /*
     * Initialize the events sets
     */
    for (i = 0; i < MAI_MAX_EVENTS; i++)
      {

	sprintf((char *) name, "EVT_chan%d", i);
	  rc = vs_event_create(&ev_array[i], name, (Eventset_t) 0x00U);

	  if (rc)
	    {
		IB_LOG_ERRORRC("vs_event_create failed rc:", rc);
		IB_EXIT(__func__, VSTATUS_BAD);
		return;
	    }
	  else
	    {

	      IB_LOG_VERBOSE("vs_event_created event ", i);
	    }

      }



    (void)memset(gMAI_CHANNELS, 0, sizeof(gMAI_CHANNELS));

    /*
     * Initialize all the channel control blocks 
     */
    for (i = 0; i < MAI_MAX_CHANNELS; i++)
      {
	  struct mai_fd  *p;


	  p = &gMAI_CHANNELS[i];	/* To ease typing */
	  p->state = MAI_FREE;
	  p->up_fd = i;
	  p->dev   = MAI_INVALID;
	  p->port  = MAI_INVALID;
	  p->qp    = MAI_INVALID;

	  /*
	   * If threaded we will sleep on timeout or data arrived events.
	   *  The event to wait on and the mask to signal with
	   */ 
	  p->hdl_ehdl  = ev_array[i/VEVENT_NUM_EVENTS].event_handle;
	  p->hdl_emask = (Eventset_t) (1 << (i%VEVENT_NUM_EVENTS));
	  

	  rc = vs_lock_init(&p->lock.lock, VLOCK_FREE, VLOCK_THREAD);
	  if (rc)
	    {
		IB_LOG_ERRORRC("vs_lock_init rc:", rc);
		IB_EXIT(__func__, VSTATUS_BAD);
		return;
	    }

	  /*
	   * So that free will work .. set stat to busy 
	   */
	  p->state = MAI_BUSY;

	  (void)mai_free_handle(p);

      }

    (void)memset(gMAI_FILTERS, 0, sizeof(gMAI_FILTERS));

    /*
     * Initialize all the filters 
     */
    for (i = 0; i < MAI_MAX_FILTERS; i++)
      {
	  struct mai_filter *p;

	  p = &gMAI_FILTERS[i];
	  p->next = NULL;
	  p->next = NULL;
	  p->owner = NULL;
	  p->state = MAI_FREE;
	  p->qp = MAI_INVALID;
	  p->flags = 0;
	  p->hndl = i;
	  mai_free_filter(p);

      }

    IB_LOG_INFO("Allocating MAI MAD buffers", gMAI_MAX_DATA);
    IB_LOG_INFO("Setting max queue depth to", gMAI_MAX_QUEUED);
    IB_LOG_INFO("Total memory required is", gMAI_MAX_DATA * sizeof(struct mai_data));
    gMAI_MAD_BUFFS = vs_malloc(gMAI_MAX_DATA * sizeof(struct mai_data));
    if (!gMAI_MAD_BUFFS)
    {
        IB_LOG_ERROR("Failed to allocate MAI MAD buffers:", gMAI_MAX_DATA);
        IB_LOG_ERROR("Total memory required was", gMAI_MAX_DATA * sizeof(struct mai_data));
        IB_EXIT(__func__, VSTATUS_NOMEM);
        return;
    }
    (void)memset(gMAI_MAD_BUFFS, 0, gMAI_MAX_DATA * sizeof(struct mai_data));

    /*
     * Initialize all the MAD mbufs 
     */
    for (i = 0; i < gMAI_MAX_DATA; i++)
      {
	  struct mai_data *p;

	  p = &gMAI_MAD_BUFFS[i];
	  mai_free_mbuff(p);
      }

    /*
     * Initialize filter subsysem 
     */
    maif_init();

    rc = ib_init_sma(gMAI_MAX_DATA);
    if (rc != VSTATUS_OK)
      {
	  IB_LOG_ERRORRC("can't initialize sma rc:", rc);
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return;
      }

    /*
     * We are done and everything worked 
     */
    gMAI_INITIALIZED = 1;

    IB_EXIT(__func__, 0);
    return;
}

/*
 * FUNCTION
 *      mai_shutdown
 *
 * DESCRIPTION
 *      This function marks the MAI library as un-initialized, forcing all 
 * applications fail and return from the library.
 * 
 * INPUTS
 *      None
 *
 * OUTPUTS
 *
 */

void
mai_shut_down(void)
{
    int             rc,i;

    IB_ENTER(__func__, 0, 0, 0, 0);

	mai_close_portinfo();

    /*
     * If we are using a dedicated down call thread, then
     * terminate the send a message to wake the thread.
     */

    if (gMAI_USE_DEDICATED_DCTHREAD)
      {

	  if (gMAI_DCTHREAD_HANDLE != MAI_INVALID)
	    {
		Mai_t           mad;
		struct mai_dc  *p;

		mad.type = MAI_TYPE_INTERNAL;
		mad.active = MAI_ACT_TYPE;

		p = &gMAI_DOWN_CL[0];
		mai_dc_read_exit = 1;
		rc = ib_send_sma(p->hndl, &mad, MAI_DEFAULT_SEND_TIMEOUT);

		if (rc)
		  {
		      IB_LOG_ERRORRC("sending shutdown mad rc:", rc);
		  }
		else
		  {
		      /*
		       * Sleep 1 second so that the MAI down call thread has time to run and 
		       * terminate.
		       */

		      (void)vs_thread_sleep(VTIMER_1S);
		  }
	    }
      }
	ib_shutdown_all();

    /* mark MAI as not initialized to stop use */
    gMAI_INITIALIZED = 0;

    /*
     *  Free the events
     */ 

    for (i = 0; i < MAI_MAX_EVENTS; i++)
      {


	  rc = vs_event_delete(&ev_array[i]);

	  if (rc)
	    {
	      IB_LOG_ERRORRC("mai_shutdown: vs_event_create failed rc:", rc);
	    }

      }

    vs_free(gMAI_MAD_BUFFS);

    IB_EXIT(__func__, 0);
}

#ifdef __VXWORKS__
extern Status_t ib_term_sma (void); /* to avoid implicit decl warning */
void 
mai_deinit(){
	int i=0;
	vs_lock_delete(&gmai_uplock.lock);
	vs_lock_delete(&gmai_tidlock.lock);
	vs_lock_delete(&gmai_hlock.lock);
	vs_lock_delete(&gmai_mlock.lock);
	vs_lock_delete(&gmai_flock.lock);
	vs_lock_delete(&gmai_dcthr_lock.lock);

    for (i = 0; i < MAI_MAXDC_CHANNELS; i++)
    {
	  struct mai_dc  *p;	/* To ease typing */
	  p = &gMAI_DOWN_CL[i];
	  vs_lock_delete(&p->lock.lock);
	}

    for (i = 0; i < MAI_MAX_CHANNELS; i++)
    {
	  struct mai_fd  *p;
	  p = &gMAI_CHANNELS[i];	/* To ease typing */
	  vs_lock_delete(&p->lock.lock);
    }

	ib_term_sma();
	gMAI_INITIALIZED = 0;
	mai_init_pid = 0;

}

int maiBufferSize()
{
    size_t size = sizeof(struct mai_data);
    sysPrintf("An MAI buffer is %d bytes\n", size);
    return size;
}

int maiBufferTotals()
{
    size_t size = maiBufferSize();
    size *= gMAI_MAX_DATA;
    sysPrintf("There are %d MAI buffers that use %d bytes total\n", (unsigned)gMAI_MAX_DATA, size);
    return size;
}

#endif
