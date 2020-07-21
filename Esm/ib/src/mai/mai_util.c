/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

** END_ICS_COPYRIGHT5   ****************************************/

/****************************************************************************
 *                                                                          *
 * FILE NAME                                               VERSION          *
 *      mai_util.c                                         MAPI 0.05        *
 *                                                                          *
 * DESCRIPTION                                                              *
 *      This file contains the utility functions used by  MAI subsystem     *
 *                                                                          *
 *                                                                          *
 * DATA STRUCTURES                                                          *
 *      None                                                                *
 *                                                                          *
 ****************************************************************************/

#include "mai_l.h"		/* Local mai function definitions */
#include "cs_log.h"
#include "vs_g.h"

#define MAX_USECS_BETWEEN_OVERUNDERFLOW_MESSAGES 5000000ull    // 5 seconds apart max
static uint64_t     time_last_overflow_logged=0;
static uint64_t     time_last_underflow_logged=0;
static uint32_t     num_times_since_logged=0;

int             mai_spawn_dc_reader(int qp, int dev, int port);

#define mai_dcthread_handle(x) (x->up_fd ==gMAI_DCTHREAD_HANDLE)

extern uint32_t    smDebugPerf;    // controls output of SM/SA performance messages
extern uint32_t    sm_debug; 
extern int         mai_dc_read_exit;    // tells dc thread to shutdown

/*
 * FUNCTION
 *   mai_dcthread_iam
 * 
 * DESCRIPTION
 *   Checks to see if the calling thread is a the down call thread.
 *   If the MAI library is not built to support automatically spawned
 *   DC thread then the calling thread should believe it is the DC 
 *   thread. Otherwise True is returned only if this is indeed the
 *   dc thread.
 *
 * INPUTS
 *      act  Pointer to the MAI handle owned by the calling thread.
 *
 * OUTPUTS
 *     1 - the caller is allowed to act the role of DC thread
 *     0 - the caller is not allowed.
 */
int
mai_dcthread_iam(struct mai_fd *act)
{
	int isDcthr=1;

    IB_ENTER(__func__, act, 0, 0, 0);

	MAI_DCTHR_LOCK();
	if (gMAI_USE_DEDICATED_DCTHREAD) {
		if (gMAI_DCTHREAD_HANDLE == MAI_INVALID) {
			int rc=0;
		/*
		 * Startup the dc thread 
		 */
		rc = mai_spawn_dc_reader(act->qp, act->dev, act->port);
			if (rc) {
				IB_LOG_ERRORRC("failed to start DC thread, rc:", rc);
			}
			isDcthr = 0;
		} else if (act->up_fd == gMAI_DCTHREAD_HANDLE) {
			isDcthr = 1;
		} else {
	  /*
		 	 * Always return false since a dedicated DC thread is suppose to run
	   */
			isDcthr = 0;
		}
	} else {
    /*
     * Always return TRUE when dedicated DC thread is not used 
     */
		isDcthr = 1;
	}
	MAI_DCTHR_UNLOCK();
	IB_EXIT(__func__, isDcthr);
	return isDcthr;
}


/*
 * FUNCTION
 *   mai_alloc_mad_buff
 *
 * DESCRIPTION
 *    This function is called with a pointer to the raw MAD that came
 *    up from below.  It will allocate a free mai_data element from the 
 *    free list (gMAI_DATA_FREE), and copy in the stuff.
 *
 * INPUTS
 *     The MAD message to load in the buffer allocated
 *
 * RETURNS
 *   NULL - No data blocks available
 *  !NULL - Pointer to the loaded  mad buffer
 *
 */

struct mai_data *
mai_alloc_mbuff(Mai_t * rawmad)
{
    struct mai_data *newmad;

    IB_ENTER(__func__, rawmad, 0, 0, 0);

    MAI_MBUFFS_LOCK();

    /*
     * Get a free one 
     */
    newmad = gMAI_DATA_FREE;

    if (!newmad)
      {
	  MAI_MBUFFS_UNLOCK();

	  /* Caller will log error */
	  IB_EXIT(__func__, 0);

	  MSTATS_NORESOURCE_INCR();
	  return (NULL);
      }

    gMAI_DATA_FREE = newmad->next;

    /*
     * Count free mbuffs 
     */

    gMAI_MAD_CNT--;

    MAI_ASSERT_TRUE((gMAI_MAD_CNT >= 0));

    if (gMAI_MAD_CNT <= gMAI_MADS_LOWWM)
      {
	  if (smDebugPerf) IB_LOG_INFINI_INFO("Running low on free mad buffers cnt:", gMAI_MAD_CNT);
      }

    MAI_MBUFFS_UNLOCK();

    /*
     * Clear the new structure and fill in the data they passed 
     */
    newmad->next = NULL;
    newmad->state = MAI_BUSY;
    memcpy(&newmad->mad, rawmad, sizeof(newmad->mad));

    MSTATS_DATA_USE();

    IB_EXIT(__func__, newmad);
    return (newmad);
}

/*
 * FUNCTION
 *   mai_free_mad_buff
 *
 *  DESCRIPTION 
 *   This function requeues  a MAD buffer to the free list
 *
 * INPUTS
 *   fmad    the MAD buffer being freed.
 *
 * RETURNS
 *
 */
void
mai_free_mbuff(struct mai_data *fmad)
{

    IB_ENTER(__func__, fmad, 0, 0, 0);

    fmad->state = MAI_FREE;

    MAI_MBUFFS_LOCK();

    MSTATS_DATA_FREE();

    /*
     * Count free mbuffs 
     */
    gMAI_MAD_CNT++;

    fmad->next = gMAI_DATA_FREE;
    gMAI_DATA_FREE = fmad;

    MAI_MBUFFS_UNLOCK();

    IB_EXIT(__func__, 0);
    return;
}

/*
 * FUNCTION
 *  mai_enqueue_mbuff
 *
 * DESCRIPTION
 *   This function adds the mai_data element passed to the end of the receive
 *   queue for the channel.  It does basic error checking.  Errors are logged
 *   and the code returns non-zero.
 *
 * INPUTS
 *    mad    pointer the mad buffer to queue to chan FIFO queue
 *    chan   pointer to channel
 *
 * RETURNS
 *   0   It worked
 *  !0   Too many already queued on this pipe
 *
 * SPECIAL CONDITIONS
 *   1. This code MUST be called holding the lock of the channel
 */
int
mai_enqueue_mbuff(struct mai_data *mad, struct mai_fd *chan)
{
	int rc,over_flow=0;
	
	IB_ENTER(__func__, mad, chan, 0, 0);
	
	MAI_ASSERT_LOCK_HELD((chan));
	

    if (chan->mad_cnt > gMAI_MAX_QUEUED)
    {
        MSTATS_FD_OVERFLOW(chan);
        /* Caller will log error */
        
        /* Signal the waiter again */
        over_flow = 1;
    }
	else
	{
		if (chan->mad_hqueue)
		{
			chan->mad_tqueue->next = mad;
			chan->mad_tqueue = mad;
		}
		else
		{
			chan->mad_hqueue = mad;
			chan->mad_tqueue = mad;
			
			MAI_ASSERT_TRUE((chan->mad_cnt == 0));
		}
		
		mad->next = NULL;
		
		chan->mad_cnt++;

		/* log SA mad count if greater than 80% of queue depth */
		if (smDebugPerf) {
			if (mad->mad.base.mclass == MAD_CV_SUBN_ADM && mad->mad.base.cversion == SA_MAD_CVERSION && chan->mad_cnt > (gMAI_MAX_QUEUED*4/5)) {
                if (++num_times_since_logged >= 20) {
                    num_times_since_logged = 0;
                    IB_LOG_INFINI_INFO("Number of entries on SA queue now", chan->mad_cnt);
                }
			} else {
                num_times_since_logged = 0;
            }
		}
		MSTATS_FD_MADINCR(chan);
	}
	
	/*
	* Add this buffer to the end as requested 
	*/
	
	if (chan->state == MAI_WAIT || over_flow == 1)
	{		
		/* Are they interested in knowning? */
		rc = vs_event_post(chan->hdl_ehdl, 
						   VEVENT_WAKE_ONE, 
						   chan->hdl_emask);
		
		if(rc)
		{
			IB_LOG_ERRORRC(" error posting to handle event rc:",rc);
			IB_EXIT(__func__, over_flow);
			return over_flow;
		}
		chan->state = MAI_BUSY;	/* Don't post another one */
	}
	
	IB_EXIT(__func__, over_flow);
	return (over_flow);
}

/*
 * FUNCTION
 *   mai_dequeue_mbuff
 *
 * DESCRIPTION
 *   This function removes the first mai_data element  from the receive
 *   queue for the channel. 
 *
 * INPUTS
 *    chan   Pointer to channel to FIFO remove the buffer.
 *
 * RETURNS
 *   pointer to mad buffer dequeued
 *
 * SPECIAL CONDITIONS
 *   1. This code MUST be called holding the lock of the channel
 */
struct mai_data *
mai_dequeue_mbuff(struct mai_fd *chan)
{
    struct mai_data *p;		/* Used to find the end of the list */

    IB_ENTER(__func__, chan, 0, 0, 0);

    MAI_ASSERT_LOCK_HELD((chan));

    if (chan->mad_cnt == 0)
      {

	  MAI_ASSERT_TRUE((chan->mad_tqueue == NULL));
	  MAI_ASSERT_TRUE((chan->mad_hqueue == NULL));

	  IB_EXIT(__func__, 0);
	  return (NULL);
      }

    /*
     * Track the statistics of MADs removed 
     */
    MSTATS_FD_MADDECR(chan);

    p = chan->mad_hqueue;	/* pick off the head of list */
    chan->mad_hqueue = p->next;	/* move the next one up to the head of
				 * list */

    p->next = NULL;		/* dissociate from list */
    chan->mad_cnt--;

    if (chan->mad_hqueue == NULL)
      {
	  /*
	   * If the head is null then p must be the last one on the list
	   * also 
	   */
	  /*
	   * Similarly, the count of pending mads must be zero 
	   */
	  MAI_ASSERT_TRUE((chan->mad_tqueue == p));
	  MAI_ASSERT_TRUE((chan->mad_cnt == 0));

	  chan->mad_tqueue = NULL;
      }

    IB_EXIT(__func__, p);
    return (p);
}

/*
 * FUNCTION
 *      mai_alloc_handle.
 *
 * DESCRIPTION
 *     This call allocates handle to associate with an open mai open channel 
 *
 * 
 * INPUTS
 *      None
 *
 * OUTPUTS
 *   NULL - No handles  available
 *  !NULL - Pointer to the handle
 */

static unsigned int mai_incarnation_ctr;

struct mai_fd  *
mai_alloc_handle()
{

    struct mai_fd  *fd;

    IB_ENTER(__func__, 0, 0, 0, 0);

    MAI_GHANDLES_LOCK();

    /*
     * Get a free one 
     */
    fd = gMAI_HANDLE_FREE;

    if (fd == NULL)
      {

	  MAI_GHANDLES_UNLOCK();

	  IB_LOG_ERROR0("No free handles");
	  IB_EXIT(__func__, 0);

	  /*
	   * Count the number of times are out of resources 
	   */
	  MSTATS_NORESOURCE_INCR();

	  return (NULL);
      }

    gMAI_HANDLE_FREE = fd->next;

    gMAI_HDL_CNT--;

    MAI_ASSERT_TRUE((gMAI_HDL_CNT >= 0));

    if (gMAI_HDL_CNT <= MAI_HNDL_LOWWM)
      {
	  if (smDebugPerf) IB_LOG_INFINI_INFO("running low on handles cnt:", gMAI_HDL_CNT);
      }

    MAI_GHANDLES_UNLOCK();

    /*
     * Should only have been on the free list if it was free 
     */
    MAI_ASSERT_TRUE((fd->state == MAI_FREE));

    fd->next = NULL;
    fd->state = MAI_BUSY;
    /*
     * Update the incarnation number 
     */
    fd->incarn = mai_incarnation_ctr++;

    /*
     * Clear the statistics on the handle 
     */
    MSTATS_FD_CLR(fd);

    IB_EXIT(__func__, 0);

    return fd;
}

/*
 * mai_free_handle Free a previously allocated handle. 
 */

/*
 * FUNCTION
 *      mai_free_handle
 *
 * DESCRIPTION
 *      Free a previously allocated handle. 
 *
 * 
 * INPUTS
 *     fd  - Pointer to the handlee being freed.
 *
 * OUTPUTS
 *    0
 */

Status_t
mai_free_handle(struct mai_fd * fd)
{

    IB_ENTER(__func__, fd, 0, 0, 0);

    MAI_ASSERT_TRUE((fd->state == MAI_BUSY));
    MAI_ASSERT_TRUE((fd->sfilt_cnt == 0));
    MAI_ASSERT_TRUE((fd->sfilt_cnt == 0));

    fd->state = MAI_FREE;
    fd->down_fd = NULL;
    fd->dev = MAI_INVALID;
    fd->port = MAI_INVALID;
    fd->qp = MAI_INVALID;
    fd->sfilters = NULL;

    fd->mad_hqueue = NULL;
    fd->mad_tqueue = NULL;

    MAI_GHANDLES_LOCK();

    gMAI_HDL_CNT++;

    fd->next = gMAI_HANDLE_FREE;
    gMAI_HANDLE_FREE = fd;

    MAI_GHANDLES_UNLOCK();

    IB_EXIT(__func__, 0);
    return 0;
}

/*
 * FUNCTION
 *   mai_enqueue _upchannel
 *
 * DESCRIPTION
 *    Add a newly opened handle to global queue. 
 *
 * INPUTS
 *    new_fd   Pointer to channel to add to global queue
 *
 * RETURNS
 *
 */

void
mai_enqueue_upchannel(struct mai_fd *new_fd)
{

    IB_ENTER(__func__, new_fd, 0, 0, 0);

    MAI_ASSERT_TRUE((new_fd->state == MAI_BUSY));

    MAI_UPCHANNELS_LOCK();

    new_fd->next = gMAI_UP_CHANNELS;

    if (gMAI_UP_CHANNELS)
      {
	  gMAI_UP_CHANNELS->prev = new_fd;
      }

    gMAI_UP_CHANNELS = new_fd;

    /*
     * Count the number of channels in use 
     */
    MSTATS_UPCHAN_USE(new_fd->qp);

    MAI_UPCHANNELS_UNLOCK();

    IB_EXIT(__func__, 0);
}

/*
 * FUNCTION
 *   mai_dequeue_upchannel
 *
 * DESCRIPTION
 *    Remove a closed handle from the  global queue. 
 *
 * INPUTS
 *    new_fd   Pointer to channel to remove from  global queue
 *
 * RETURNS
 
 */

void
mai_dequeue_upchannel(struct mai_fd *free_fd)
{

    IB_ENTER(__func__, free_fd, 0, 0, 0);

    MAI_ASSERT_TRUE((free_fd->state != MAI_FREE));

    MAI_UPCHANNELS_LOCK();

    if (free_fd->prev)
      {
	  free_fd->prev->next = free_fd->next;
      }

    if (free_fd->next)
      {
	  free_fd->next->prev = free_fd->prev;
      }

    /*
     * If this guy was the head of the list .. update the list head 
     */
    if (free_fd == gMAI_UP_CHANNELS)
      {
	  gMAI_UP_CHANNELS = free_fd->next;

	  MAI_ASSERT_TRUE(((gMAI_UP_CHANNELS && gMAI_UP_CHANNELS->prev == NULL)
		      || (!gMAI_UP_CHANNELS)));
      }

    /*
     * this guy is now out of the loop 
     */
    free_fd->next = free_fd->prev = NULL;

    /*
     * Count the number of channels in use 
     */
    MSTATS_UPCHAN_FREE(free_fd->qp);

    MAI_UPCHANNELS_UNLOCK();

    IB_EXIT(__func__, 0);

}




/*
 * mai_mad_process
 *   This function is called with a MAD and the QP it was received on.
 * We scan all outstanding regular filters (gMAI_UP_CHANNELS[qp]) duplicating
 * the MAD on the input queues of any channel with a matching filter.
 *
 * The function returns the total number of filter matches.
 *
 * SPECIAL CASES
 *   1. When we don't have resource to chain a MAD we just log a resource
 *      error and drop the mad.
 */
int
mai_mad_process(Mai_t * mad, int *filterMatch)
{
    struct mai_fd  *chan;	/* Loops over all channels */
    struct mai_filter *filt;	/* Loops over all filters on a channel */
    struct mai_data *data;	/* Points to the MAD */
    int             handled;	/* Total number of duplicates created */
    int             limit;	/* Used to avoid linked list bugs */
    int             limit2;	/* Ditto */
    int             rc;
    uint64_t        timeNow=0;

    IB_ENTER(__func__, mad, 0, 0, 0);
    handled = 0;		/* Start from scratch */

    /*
     * Scan the filter lists for all open channels looking for matches 
     */

    MAI_UPCHANNELS_LOCK();

    limit = 0;

    for (chan = gMAI_UP_CHANNELS;
	 (chan != NULL) && (gMAI_INITIALIZED != 0); chan = chan->next)
      {
	  if (limit++ > MAI_MAX_CHANNELS)
	    {
		MAI_UPCHANNELS_UNLOCK();
		IB_LOG_ERROR("Channel list corrupt limit:",
			     limit);
		mai_shut_down();
		return 0;
	    }

	  limit2 = 0;

	  MAI_HANDLE_LOCK(chan);

	  for (filt = chan->sfilters; filt; filt = filt->next)
	    {
		if (limit2++ > MAI_MAX_FILTERS)
		  {

		      MAI_HANDLE_UNLOCK(chan);
		      MAI_UPCHANNELS_UNLOCK();

		      IB_LOG_ERROR("Filter list corrupt limit:",
				   limit2);
		      mai_shut_down();
		      return 0;
		  }

		/*
		 * See if this MAD matches this filter 
		 */
		rc = maif_match(mad, &filt->filter);

		if (!rc)
		    continue;	/* No match, try the next one */

		MSTATS_FMATCH_INCR(filt);

		/*
		 * There is a match.  Create a new MAD and add to this guy 
		 */
        *filterMatch = 1;
		data = mai_alloc_mbuff(mad);

		if (!data)
		  {
		      /*
		       * If we ran out of buffers, drop it 
		       */
		      MAI_HANDLE_UNLOCK(chan);
		      MAI_UPCHANNELS_UNLOCK();

              vs_time_get(&timeNow);
              if ((timeNow - time_last_underflow_logged) > MAX_USECS_BETWEEN_OVERUNDERFLOW_MESSAGES)
                {
                    time_last_underflow_logged = timeNow;
                    IB_LOG_INFINI_INFO_FMT(__func__,
                           "Out of mad buffers, %s[%s] for filter %s not handled from LID [0x%x], TID=0x%.16"CS64"X, total since start=%d",
                           cs_getMethodText((int)mad->base.method), cs_getAidName((int)mad->base.mclass, (int)mad->base.aid), filt->filter.fname, 
                           mad->addrInfo.slid, mad->base.tid, gMAI_STATS.no_resource);
                }

		      IB_EXIT(__func__, handled);
		      return (handled);
		  }

		/*
		 * Chain it to the end of this channels input queue 
		 */
		rc = mai_enqueue_mbuff(data, filt->owner);

		if (rc)
		  {
		      /*
		       * NO - put the MAD back on free list 
		       */
		      mai_free_mbuff(data);

              vs_time_get(&timeNow);
              if ((timeNow - time_last_overflow_logged) > MAX_USECS_BETWEEN_OVERUNDERFLOW_MESSAGES)
			    {
                    time_last_overflow_logged = timeNow;
                    IB_LOG_INFINI_INFO_FMT(__func__,
                           "Cannot enqueue %s[%s] for filter %s, from LID [0x%x], TID=0x%.16"CS64"X, num mads dropped since start=%d",
                           cs_getMethodText((int)mad->base.method), cs_getAidName((int)mad->base.mclass, (int)mad->base.aid), filt->filter.fname, 
                           mad->addrInfo.slid, mad->base.tid, filt->owner->overflow);
			    }
		  }
		else
		  {
		      //IB_LOG_INFO("mai_mad_process added mad to up channel handle", chan->up_fd);
		      /*
		       * If the filter was a one shot filter then remove
		       * it 
		       */
		      if (filt->once)
			{
			    MAI_HANDLE_UNLOCK(chan);

			    rc = mai_filter_hdelete(filt->owner->up_fd,
						    filt->hndl);
			    if (rc != VSTATUS_OK)
			      {
				  IB_LOG_ERROR
				      ("Deleting one shot filter rc:",
				       rc);
			      }

			    MAI_HANDLE_LOCK(chan);
			}

		      handled++;
		      break;	// each channel gets a single notice of match
		  }
	    }

	  /*
	   * Release the lock to the handle 
	   */
	  MAI_HANDLE_UNLOCK(chan);

      }

    MAI_UPCHANNELS_UNLOCK();

    IB_EXIT(__func__, handled);
    return (handled);
}

/*
 * FUNCTION
 *    mai_free_dc
 *
 * DESCRIPTION
 *    This functino will release a hardware (down) channel structure.  It does
 *    this by dropping the reference count until there are no consumers
 *    of the channel at which point the channel is closed.
 *
 * 
 * INPUTS
 *      None
 *
 * OUTPUTS
 *
 */

void
mai_free_dc(struct mai_dc *dc)
{
    int             rc;

    IB_ENTER(__func__, dc, 0, 0, 0);

    /*
     * Some simple parameter checking 
     */
    if (dc == NULL)
      {
	  IB_LOG_ERROR0("NULL pointer");
	  return;
      }

    MAI_ASSERT_TRUE((dc->ref > 0));

    MAI_DC_LOCK(dc);

    /*
     * Drop the reference count and return if people still hanging 
     */
    dc->ref--;

    /*
     * Simple case first - If anyone is still using it, just return 
     */
    if (dc->ref > 0)
      {
	  MAI_DC_UNLOCK(dc);
	  IB_EXIT(__func__, dc->ref);
	  return;
      }

    dc->readt_state = QPS_DEAD;

    IB_LOG_INFO("detaching handle ", dc->hndl);

    rc = ib_detach_sma(dc->hndl);

    if (rc)
      {
	  IB_LOG_ERRORRC("detach sma failed - mai_free_dc rc:", rc);
      }
    else
      {
	  IB_LOG_INFORC("detached completed successfully rc:", rc);
      }

    /*
     * Count the number free DCown channels 
     */
    MSTATS_DC_FREE();

    /*
     * remove it from the list of hardware (down) channels.  
     */
    dc->state = MAI_FREE;

    MAI_DC_UNLOCK(dc);

    IB_EXIT(__func__, VSTATUS_OK);
    return;
}

/*
 * FUNCTION
 *    mai_get_dc 
 *
 * DESCRIPTION
 *     This function will return a pointer to the hardware (down) channel structure
 *     established for the dev/port/qp context.  If there is already an 
 *     open hardware (down) channel, we increment the reference count and return
 *     a pointer to the structure.  If not, we open one and attempt to
 *     open the channel.
 *
 * 
 * INPUTS
 *      dev      device ordinal
 *      port     port on the device
 *      qp       queue pair
 *      out      pointer of location to place the dc returned.
 *      nodeType pointer of location to place node type
 *
 * Returns
 *    VSTATUS_OK
 *
 */

int
mai_get_dc(int dev, int port, int qp,
	   struct mai_dc **out, uint8_t * nodeType)
{

    struct mai_dc  *p;
    int             rc;
    IBhandle_t      hdl;

    IB_ENTER(__func__, dev, port, qp, 0);


    /*
     * We only use 1 DC 
     */
    p = &gMAI_DOWN_CL[0];

    /*
     * Since we already have a down call instance open we only
     * need to do the attach call to see whether the device
     * port,qp tuple does exist below us. If it exist, then
     * we are fine and we  detach immediately. The single 
     * down call handle is all we need to keep open
     * as it will feed all associated traffic up to us.
     *
     * If the call fails, then we know that the user is asking
     * for a tuple that doesn't exist and so we simple return
     * the  failure status. 
     * So go ahead and attempt to open a hardware (down) channel.
     */

    rc = ib_attach_sma(dev, port, qp, &hdl, nodeType);

    if (rc)
      {
        IB_LOG_ERRORRC("ib_attach_sma returns error rc:", rc);
	  IB_EXIT(__func__, rc);
	  return (rc);
      }

    //IB_LOG_VERBOSEX("ib_attach_sma returns", hdl);

    /*
     * Things are ok down there so just dettach 
     */

    MAI_DC_LOCK(p);

    if (p->ref)
      {
	  /*
	   * Detach if this is not the first open on the hardware (down) channel. We
	   * only need to ensure that one open is maintained on the down
	   * channel so that only a single detach is called when the last
	   * reference is removed. 
	   */

	  rc = ib_detach_sma(hdl);

	  p->ref++;

	  if (rc)
	    {
		MAI_DC_UNLOCK(p);
        IB_LOG_ERRORRC("detach_sma returns error rc:", rc);
		IB_EXIT(__func__, rc);
		return (rc);
	    }
      }
    else
      {
	  gMAI_DOWN_CHANNELS = p;
	  MSTATS_DC_USE();
	  MSTATS_DC_CLR(p);

	  /*
	   * We are reopening the channel. Take the new handle passed up 
	   */
	  p->hndl = hdl;

	  /*
	   * Initialize the constant fields 
	   */
	  p->state = MAI_BUSY;
	  p->readt_state = QPS_STARTED;
      }

    /*
     * If somebody scheduled this guy to die, unschedule it 
     */
    if (p->readt_state == QPS_DIE)
      {
	  p->incarn++;
	  p->readt_state = QPS_STARTED;
      }
    p->ref++;

    MAI_DC_UNLOCK(p);

    *out = p;

    IB_EXIT(__func__, VSTATUS_OK);
    return (VSTATUS_OK);

}

/*
 * FUNCTION
 *   mai_getqp
 *
 * DESCRIPTION
 *
 *   This function hides all the tricky timeout logic needed
 * to juggle implementations that support multiple threads/processes.
 *
 *   The basic purpose of this function is to wait for data to 
 * arrive from the hardware (down) channel or for a timeout.  
 *   The first caller to this function will pass through to ib_recv_sma
 * function below.
 * 
 *   2. Single process
 *        In this case we will call directly through the common services
 *      smi/gsi function to receive a block of data.  Our callers timeout
 *      is given to that code.
 *
 *   When this funciton returns, the caller must check for data on the
 *  channel receive queue. 
 * 
 * INPUT
 *    act    The channel of handle being waited on
 *    wakeup when to return from this call
 * RETURN
 *
 *
 */

int
mai_getqp(struct mai_fd *act, uint64_t wakeup)
{
    uint64_t        timenow,
                    timeout;
    int             rc,
                    is_dc;
    struct mai_dc  *dc;
    unsigned int    incarn;
    Eventset_t      events = (Eventset_t) 0U;
    /*
     * Standard logging stuff 
     */

    IB_ENTER(__func__, act, 0, 0, 0);

    dc = act->down_fd;

    /*
     * Note the incarnation number 
     */
    incarn = dc->incarn;

  retry:

    if (wakeup == 0)		/* No timeout needed? */
      {
	  timeout = 0;
      }
    else
      {
	 (void) vs_time_get(&timenow);

	  if (timenow >= wakeup)
	    {
		rc = VSTATUS_TIMEOUT;
		IB_EXIT(__func__, rc);
		return rc;
	    }
	  else
	    {
		timeout = wakeup - timenow;
	    }
      }

    if ((is_dc = mai_dcthread_iam(act)))
      {

	  MAI_DC_LOCK(dc);
	  /*
	   * Its OK for me to work as the dc thread 
	   */

	  if (dc->readt_state != QPS_STARTED &&
	      dc->readt_state != QPS_RUNNING)
	    {

		MAI_DC_UNLOCK(dc);
		IB_EXIT(__func__, dc->readt_state);
		return (VSTATUS_ILLPARM);
	    }

	  /*
	   * Handle the single threaded (easy) case first 
	   */
	  if (dc->active == 0 && dc->readt_state == QPS_STARTED)
	    {
		Mai_t           mad;	/* Holds data read from downstream 
					 */
		int             chan;


		/*
		 * Mark the down call as running to stopp anyone else 
		 * from entering.
		 */

		dc->readt_state = QPS_RUNNING;
		dc->active++;
		(void) vs_thread_name(&dc->readt);
		/*
		 * Now that we have marked it as active, no one else will come
		 * through the gate so we can release the lock.
		 */

		MAI_DC_UNLOCK(dc);

		/*
		 * Call common services to read the data 
		 */

		chan = dc->hndl;

		rc = ib_recv_sma(chan, &mad, timeout);

		IB_LOG_VERBOSERC("ib_recv_sma returned rc:", rc);

		if (gMAI_INITIALIZED)
		  {
		      /*
		       * If MAI is still up and running 
		       */

		      switch (rc)
			{
			case VSTATUS_MAI_INTERNAL:
			case VSTATUS_OK:
			case VSTATUS_FILTER:
			    {
				MSTATS_DC_RX(dc, mad.type);
				rc = 0;
				switch (mad.type)
				  {
				  case MAI_TYPE_EXTERNAL:
				  case MAI_TYPE_INTERNAL:
				  case MAI_TYPE_ERROR:
				      {
                      int filterMatch=0;
					  int handled = mai_mad_process(&mad, &filterMatch);
					  if (!handled)
					  {
						  STL_LID  tempLid;
						  uint32_t tempQP;
						  Status_t status;

						/* log unhandled requests/responses if smDebugPerf is turned on */
						if (smDebugPerf && !filterMatch && mad.type != MAI_TYPE_ERROR) {
							/* Not handled, log and possibly reply to sender */
							IB_LOG_INFINI_INFO_FMT(__func__,
								   "MAD class=0x%x, Method=0x%x, AID=0x%x, AMOD=0x%x not handled "
								   "from LID [0x%x], TID=0x%.16"CS64"X, mad status=0x%x",
								   mad.base.mclass, mad.base.method, mad.base.aid, mad.base.amod,
								   mad.addrInfo.slid, mad.base.tid, mad.base.status);
						/* also display the MAI_TYPE_ERROR mad type message only if debug is turned on */
						/* this was done for PR 115443 to supress messages that had timed out */
						} else if (smDebugPerf && !filterMatch && mad.type == MAI_TYPE_ERROR && sm_debug) {
							IB_LOG_INFINI_INFO_FMT(__func__,
								   "MAD class=0x%x, Method=0x%x, AID=0x%x, AMOD=0x%x not handled due to MAI_TYPE_ERROR "
								   "from LID [0x%x], TID=0x%.16"CS64"X, mad status=0x%x",
								   mad.base.mclass, mad.base.method, mad.base.aid, mad.base.amod,
								   mad.addrInfo.slid, mad.base.tid, mad.base.status);
						} else if (smDebugPerf && mad.type == MAI_TYPE_DROP) {
							IB_LOG_INFINI_INFO_FMT(__func__,
								   "MAD class=0x%x, Method=0x%x, AID=0x%x, AMOD=0x%x not handled due to MAI_TYPE_DROP "
								   "from LID [0x%x], TID=0x%.16"CS64"X, mad status=0x%x",
								   mad.base.mclass, mad.base.method, mad.base.aid, mad.base.amod,
								   mad.addrInfo.slid, mad.base.tid, mad.base.status);
						}
						
						/*
						 * Now send back with error status if Device Management
						 * or Communication Management ignore and drop
						 * otherwise
						 */
						if (mad.type != MAI_TYPE_DROP && (mad.base.mclass == MAD_CV_DEV_MGT || mad.base.mclass == MAD_CV_COMM_MGT)) {
							/* Respond only to Get/Set/Send */
							mad.base.status = MAD_STATUS_BAD_ATTR;
							if (mad.base.method == MAD_CM_GET || mad.base.method == MAD_CM_SET) {
								mad.base.method = MAD_CM_GET_RESP;
							} else if (mad.base.method == MAD_CM_SEND) {
								mad.base.method = MAD_CM_SEND;
							} else {
								break;
							}
						} else {
							break;  /* ignore packet */
						}

						/* First swap lids */
						tempLid = mad.addrInfo.dlid;
						mad.addrInfo.dlid = mad.addrInfo.slid;
						mad.addrInfo.slid = tempLid;

						/* Next swap QPs */
						tempQP = mad.addrInfo.destqp;
						mad.addrInfo.destqp = mad.addrInfo.srcqp;
						mad.addrInfo.srcqp = tempQP;

						status = stl_send_sma(chan, &mad, MAI_DEFAULT_SEND_TIMEOUT);
						if (status != VSTATUS_OK) {
							IB_LOG_WARNRC("failed to send error response rc:", status);
						}

					  }
					  break;
				      }
				  default:
				      IB_LOG_ERROR ("ib_recv_sma type:", mad.type);
				      break;
				  }

			    }
			    break;
			case VSTATUS_TIMEOUT:
			    break;

			case VSTATUS_ILLPARM:
			case VSTATUS_BAD:
			    IB_LOG_ERRORRC("ib_recv_sma rc:", rc);
			    break;

			default:

			    IB_LOG_ERROR
				("mai_getqp: Unexpected value returned from ib_recv_sma rc:",
				 rc);
			    rc = VSTATUS_BAD;
			    break;
			}
		  }

		/*
		 * Restore the state of the down call. First retake the
		 * lock 
		 */
		MAI_DC_LOCK(dc);

		/*
		 * clear the active count flag 
		 */
		dc->active--;
		dc->readt = 0;
		/*
		 * if the state is the same as I set it then we assume we are ok. 
		 * It is possible that  user could use via thread could have close 
		 * the handle we were waiting on, resulting the state down call moving 
		 * to free. This happens when all the handles in the layer has been 
		 * closed.
		 */

		if (dc->readt_state == QPS_RUNNING && dc->incarn == incarn)
		  {
		      /*
		       * mark the down call free to allow others to use it 
		       */
		      dc->readt_state = QPS_STARTED;
		  }
		else
		  {
		      /*
		       * If the state is not what we expected then the ref count should
		       * be zero.
		       */
		      if (dc->incarn != incarn)
			{
				IB_LOG_ERROR_FMT(__func__,
					"incarnation number changed %u != %u", incarn, dc->incarn);
			}

		      if (dc->ref != 0)
			{
			    MAI_DC_UNLOCK(dc);

			    IB_LOG_ERROR
				("unexpected down call state:",
				 dc->readt_state);
			    IB_EXIT(__func__, VSTATUS_BAD);
			    return VSTATUS_BAD;
			}

		      /*
		       * Return a status of bad here so that the mai_recv call will
		       * return fail.
		       */

		      MAI_DC_UNLOCK(dc);

		      IB_EXIT(__func__,
			      VSTATUS_BAD);
		      return (VSTATUS_BAD);
		  }

		if (mai_dc_read_exit)
		  {
		  MAI_DC_UNLOCK(dc);
		  //rc = VSTATUS_BAD;
          rc = VSTATUS_CONNECT_GONE;
		  IB_LOG_VERBOSE("Mai told to shut down, returning VSTATUS_CONNECT_GONE=", rc);
		  IB_EXIT(__func__, rc);
		  return rc;
		  }

		if (((mai_dcthread_handle(act)) == 0) ||
		    (gMAI_INITIALIZED == 0))
		  {
		      /*
		       * I am a regular thread so I can't stay here for
		       * ever. Wake some else to take over the job 
		       */

		      /*
		       * Wake the waiters only if thee state of the handle allows for 
		       * it.
		       * NOTE: When  dedicated DC threads  skip this search.
		       */

		      if (dc->readt_state == QPS_STARTED && dc->waiters)
			{
			    int             i;
			    struct mai_fd  *up_fd;
			    /*
			     * Find and wake just one other thread that does not have 
			     * any data posted on his handle so that it can do the 
			     * down_call to wait for data.
			     */

			    MAI_UPCHANNELS_LOCK();

			    for (i = 0, up_fd = gMAI_UP_CHANNELS;
				 ((i < MAI_MAX_CHANNELS)
				  && (up_fd != NULL));
				 i++, up_fd = up_fd->next)
			      {

				  /*
				   * Skip my handle 
				   */
				  if (gMAI_INITIALIZED)
				    {
					if (up_fd == act)
					    continue;
				    }

				  /*
				   * Wake someone who received nothing so that they can continue
				   * to do the wait. 
				   */

				  if ((gMAI_INITIALIZED == 0) ||
				      (up_fd->state == MAI_WAIT
				       && up_fd->mad_hqueue == NULL))
				    {
				         rc = vs_event_post(up_fd->hdl_ehdl,
							    VEVENT_WAKE_ONE,
							    up_fd->hdl_emask);
				       if(rc)
					 {
					   IB_LOG_ERRORRC("error e-posting up_fd rc:",
							rc);
					   
					 }
					break;
				    }
			      }

			    MAI_UPCHANNELS_UNLOCK();
			}

		  }

		/*
		 * If the mai_wait_handle channel has waiters, then the
		 * waiters need to be notified. The wake up call simply
		 * make them recheck the handles passed in by the user to
		 * see anything has been posted to it. 
		 */

		MAI_DC_UNLOCK(dc);

		IB_EXIT(__func__, rc);
		return rc;

	    }			/* if */
	  else
	    {
		/*
		 * Some one got here before ... so just let them know to wake be 
		 * before they leave.
		 */

		dc->waiters++;
	    }

	  MAI_DC_UNLOCK(dc);
      }

    /*
     * Now wait for the event or post 
     */
    
    do{
      rc = vs_event_wait(act->hdl_ehdl, timeout,act->hdl_emask,&events); 
    }while(rc == VSTATUS_AGAIN);
	
    if (is_dc != 0)
      {
	  /*
	   * Retake the lock 
	   */
	  MAI_DC_LOCK(dc);

	  /*
	   * Decrement the waiters count 
	   */
	  dc->waiters--;

	  MAI_DC_UNLOCK(dc);
      }

    if ((rc != VSTATUS_OK) &&
	(rc != VSTATUS_TIMEOUT) && (rc != VSTATUS_SIGNAL))
      {
	  IB_LOG_ERRORRC("vs_event_wait rc:", rc);
	  IB_EXIT(__func__, rc);
	  return rc;
      }

    if (gMAI_INITIALIZED == 0)
      {
	  /*
	   * If MAI has shutdown 
	   */

	  rc = VSTATUS_BAD;
	  IB_LOG_ERROR0("MAI shutdown ");
	  IB_EXIT(__func__, rc);
	  return rc;

      }

    if (rc == VSTATUS_OK && act->mad_hqueue == NULL)
      {
	  /*
	   * Keep waiting for data.
	   */
	  if (act->state == MAI_FREE)
	    {
		/*
		 *The channel was closed. The user had multiple threads using it 
		 * closed in one context while someone was listening on it.
		 */
		IB_EXIT(__func__,
			VSTATUS_BAD);
		return VSTATUS_BAD;
	    }

	  /*
	   * If we were awaken but there is no data posted then the 
	   * thread that was doing the down call has returned and so we must 
	   * now proceed to do the down call ourself.
	   */
	  goto retry;

      }

    /*
     * Our job is done.  
     */
    IB_EXIT(__func__, rc);
    return rc;

}
