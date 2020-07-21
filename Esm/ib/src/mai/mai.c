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
 *      mai.c                                              MAPI 0.05        *
 *                                                                          *
 * DESCRIPTION                                                              *
 *      This file contains entry points for  the management API open,       *
 *      close, send and receive functions                                   *
 *      functions.                                                          *
 *                                                                          *
 *                                                                          *
 * DATA STRUCTURES                                                          *
 *      None                                                                *
 *                                                                          *
 ****************************************************************************/

/*
 * Local definitions:
 *
 *
 */

#include "mai_l.h"		/* Local mai function definitions */
#include "ib_macros.h"
#include "iba/stl_sm_priv.h"
#include "iba/stl_sa_priv.h"

#ifdef __VXWORKS__
extern uint16_t getDefaultPKey(void);
#endif

#define	Min(X,Y)		((X) < (Y)) ? (X) : (Y)

uint16_t maiDefPKey = STL_DEFAULT_FM_PKEY;

uint32_t            smDebugPerf=0;  // control SM performance messages; default is off
uint32_t            saDebugPerf=0;  // control SA performance messages; default is off
uint32_t            sm_debug=0;  	// SM debug; default is off
uint8_t				smTerminateAfter=0; // Undocumented. Used to terminate the SM early, for controlled testing.
char*				smDumpCounters=NULL; // Undocumented. Used to dump counters after each sweep.

/*
 * mai_get_max_filters
 *   return maximum filters available in MAI
 *	provided for use by MAI unit tests since MAI_MAX_FILTERS is
 *	a local constant
 *
 * INPUTS
 *      None
 *
 * RETURNS
 *	unsigned
 */
unsigned mai_get_max_filters(void)
{
	return MAI_MAX_FILTERS;
}

/*
 * FUNCTION
 *      mai_open
 *
 * DESCRIPTION
 *      This function is called to create a new SMA (QP0 or QP1) channel
 *      for reading and writing MADs.  It is an exported interface for the
 *      VIEO management API.
 *
 *
 * INPUTS
 *       qp    The SMB QP you need to receive on.  Must be 0 or 1
 *       dev   Device number to open.  (Which FI in multi-FI systems)
 *       port  Port on FI for multi-ported channel adapters
 *      *fd    Pointer to where to store the new channel number 
 *
 * OUTPUTS
 *       VSTATUS_OK
 *       VSTATUS_UNINIT
 *       VSTATUS_ILLPARM
 *       VSTATUS_NOMEM
 *       VSTATUS_NOPRIV
 *
 * NOTES
 *   1.  When a system contains multiple channel adapters, they are
 *       numbered consecuatively starting at 0.  One way to determine
 *       the number of FIs on this host is to attempt to open them and
 *       look for VSTATUS_NODEV.
 *   2.  When a FI supports multiple ports, they are numbered consecuatively
 *       starting at 0.  One way to determine the number of ports on a FI
 *       is to attempt to open them and look for VSTATUS_NOPORT
 *
 */
Status_t
mai_open(uint32_t qp, uint32_t dev, uint32_t port, IBhandle_t *fd)
{
    struct mai_fd  *new_fd;
    struct mai_dc  *dc;
    int             i;
    int             rc;
    uint8_t         nodeType;

    /*
     * Standard entry stuff and parameter checking 
     */
    IB_ENTER(__func__, qp, dev, port, fd);

    if (!gMAI_INITIALIZED)
      {
	  rc = VSTATUS_UNINIT;
	  IB_LOG_ERROR0("MAPI library not initialized");
	  return rc;
      }

    if ((qp != 0) && (qp != 1))
      {
	  IB_LOG_ERROR("Invalid QP:", qp);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    if (fd == NULL)
      {
	  IB_LOG_ERROR0("NULL fd passed");
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    /*
     * Open the hardware channel 
     */

    rc = mai_get_dc(dev, port, qp, &dc, &nodeType);

    if (rc != VSTATUS_OK)
      {
	  IB_EXIT(__func__, rc);
	  return (rc);
      }

    if (nodeType != NI_TYPE_SWITCH &&
	nodeType != NI_TYPE_CA)
      {
	  IB_EXIT(__func__, nodeType);
	  return VSTATUS_BAD;
      }

    /*
     * Scan looking for a free upchannel 
     */
    new_fd = mai_alloc_handle();

    /*
     * If no free channels, log it and return an error 
     */
    if (new_fd == NULL)
      {
	  mai_free_dc(dc);

	  IB_LOG_ERROR0("No free channels");
	  IB_EXIT(__func__, VSTATUS_NOMEM);
	  return (VSTATUS_NOMEM);
      }

    /*
     * take off the index 
     */
    i = new_fd->up_fd;

    /*
     * Found one.  Grab it, link it into active channel list, and unlock 
     */

    new_fd->qp = qp;
    new_fd->dev = dev;
    new_fd->port = port;
    new_fd->nodeType = nodeType;

    /*
     * Initialize the new channel 
     */
    new_fd->down_fd = dc;
    new_fd->wakeup = 0;

    /*
     * Channel has been initialized.  Return the data 
     */
    *fd = new_fd->up_fd;

    /*
     * Put it on the list do it can start receiving MADs 
     */
    mai_enqueue_upchannel(new_fd);

    IB_LOG_INFO("mai_open opens channel", i);
    IB_EXIT(__func__, VSTATUS_OK);
    return (VSTATUS_OK);
}

/*
 * FUNCTION
 *      mai_close
 *
 * DESCRIPTION
 *      Close an open channel.  Release any assigned resources. 
 *
 *
 * INPUTS
 *      fd      The channel to close
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_UNINIT
 *      VSTATUS_ILLPARM
 *
 */
Status_t
mai_close(IBhandle_t fd)
{
    struct mai_fd  *chanp;
    struct mai_data *data;
    struct mai_filter *filterp;
    int             rc;

    /*
     * Standard entry stuff and parameter checking 
     */
    IB_ENTER(__func__, fd, 0, 0, 0);

    if (!gMAI_INITIALIZED)
      {
	  rc = VSTATUS_UNINIT;
	  IB_LOG_VERBOSE0("MAPI library not initialized");
      return VSTATUS_OK;
      }

    if ((fd < 0) || (fd >= MAI_MAX_CHANNELS))
      {
	  IB_LOG_ERROR("Invalid fd:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    /*
     * The cleanup must be done under the lock 
     */

    chanp = &gMAI_CHANNELS[fd];

    MAI_HANDLE_LOCK(chanp);

    if (chanp->state == MAI_FREE)
      {
	  MAI_HANDLE_UNLOCK(chanp);
	  IB_LOG_ERROR("Channel not active:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    /*
     * Verify that we are queued on a valid SMI/GSI chain 
     */
    if ((chanp->down_fd == NULL))
      {
	  MAI_HANDLE_UNLOCK(chanp);
	  IB_LOG_ERROR0("hardware channel fd is null");
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return VSTATUS_BAD;
      }

    /*
     * Wake any thread that may be waiting on the handle 
     */
    if ((chanp->waiters))
      {
	 rc = vs_event_post(chanp->hdl_ehdl,
			     VEVENT_WAKE_ALL,
			     chanp->hdl_emask);

	  if(rc)
	    {
	      IB_LOG_ERRORRC("event_post error rc:", rc);
	    }
      }

    
    /*
     * Release the lock so that we can aquire locks in proper order 
     */
    MAI_HANDLE_UNLOCK(chanp);

    /*
     * Remove the up channel from the list to stop receiving MADs 
     */
    mai_dequeue_upchannel(chanp);

    /*
     * Mark as free to prevent operations on handle 
     */
    chanp->state = MAI_FREE;

    /*
     * Release all data currently chained to this channel 
     */
    MAI_HANDLE_LOCK(chanp);

    while (chanp->mad_cnt)
      {
	  data = mai_dequeue_mbuff(chanp);
	  if (data) mai_free_mbuff(data);
      }

    MAI_ASSERT_TRUE((chanp->mad_cnt == 0));
    MAI_ASSERT_TRUE((chanp->mad_hqueue == NULL));
    MAI_ASSERT_TRUE((chanp->mad_tqueue == NULL));

    /*
     * Scan the list of normal filters and clean them up as well 
     */
    while (chanp->sfilt_cnt)
      {
	  /*
	   * First remove the filter from handle.  This removes it from the 
	   * exclusive filter list as well.
	   */

	  filterp = mai_filter_dequeue(chanp, chanp->sfilters);

	  mai_free_filter(filterp);

      }

    MAI_ASSERT_TRUE((chanp->sfilt_cnt == 0));
    MAI_ASSERT_TRUE((chanp->sfilters == NULL));

    /*
     * Now free the handle 
     */
    /*
     * First restore state to busy so free logic works 
     */
    chanp->state = MAI_BUSY;

    mai_free_dc(chanp->down_fd);
    rc = mai_free_handle(chanp);
    if (rc)
      {
	IB_LOG_ERRORRC("Error freeing handle rc:",
		     rc);
      }
    MAI_HANDLE_UNLOCK(chanp);

    IB_EXIT(__func__, VSTATUS_OK);
    return (VSTATUS_OK);
}

/*
 * FUNCTION
 *      mai_recv_handles
 *
 * DESCRIPTION
 *      This function will wait for a message to be posted to any of the handles 
 *      specified in argument.  It returns the index of the lowest handle
 *      that has a message on it, if a message is received before timeout.
 *      This index is posted to the location the user passed in, pointed
 *      by the location pfirst.
 * 
 * INPUTS
 *      ha      Array of handles returned from mai_open
 *      count   Number of handles in array.
 *      timout  How long to wait. 0 means return immediately
 *      pfirst  Where to post the index handle with message on in 
 *      maip    Where to post the MAD received on the handle
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *      VSTATUS_TIMEOUT
 *
 * HISTORY
 *      NAME      DATE          REMARKS
 *      JMS     02/16/07        Initial entry
 */
Status_t mai_recv_handles(IBhandle_t *ha, uint32_t hdlCnt, 
                          uint64_t timeout, uint32_t *pfirst, Mai_t *maip)
{
    uint64_t          timenow;
    uint64_t          wakeup;
    uint32_t          rc=VSTATUS_OK,i;

    IB_ENTER(__func__, ha, hdlCnt, timeout, 0);
    if (ha == NULL || pfirst == NULL ||
        hdlCnt <=0  || maip   == NULL) {
        rc = VSTATUS_ILLPARM;
        return(rc); 
    }
    /* Update the absolute timeout value to wait for. */
    if (timeout == MAI_RECV_NOWAIT) {
        wakeup  = 0;
    } else {
        (void)vs_time_get(&timenow);
        wakeup = timenow + timeout;
    }
    /* 
     * The handles are valid. Now Try and see if there is data
     * already waiting on any.
     */
    *pfirst = hdlCnt + 1;  /* set to greater than handle count */
    for (i=0; i<hdlCnt; i++) {
        rc = mai_recv(ha[i], maip, MAI_RECV_NOWAIT);
        if (rc == VSTATUS_OK) {
            *pfirst = i;  /* set handle that had data */
            break;
        }
        if (rc != VSTATUS_TIMEOUT) {
            /* An error has occurred  */
            IB_LOG_ERROR("recv err on handle:",i);
            break;
        }
    }
    /* 
     * If the handles did not have data on them, let's 
     * proceed to the next stage of waiting.
     * See if we have reached (or exceeded timeout)
     */
    if (rc == VSTATUS_TIMEOUT) {
        rc = VSTATUS_OK;
        if (wakeup == 0) {
            rc = VSTATUS_TIMEOUT;
        } else {
            (void) vs_time_get(&timenow);
            if (timenow > wakeup) {
                rc = VSTATUS_TIMEOUT;
            }
        }
        if (rc == VSTATUS_OK) {
            /* wait for remainder of requested time on first handle */
            rc = mai_recv(ha[0], maip, timeout);
            if (rc == VSTATUS_OK) {
                *pfirst = 0;
            }
        }
    }

    IB_EXIT(__func__,rc);
    return(rc); 
}

/*
 * FUNCTION
 *      mai_recv
 *
 * DESCRIPTION
 *      Return the next SMD that passes the registered filters for the
 *      channel requested.  Also can return if a timeout occurs.
 * 
 *
 * INPUTS
 *      fd      The channel to receive data from
 *      buf     Location to copy the SMD to (256 byte max)
 *      timeout The number of micro-seconds to wait for data
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *      VSTATUS_TIMEOUT
 *      VSTATUS_UNINIT
 *      VSTATUS_BAD
 *
 * HISTORY
 *      NAME      DATE          REMARKS
 *      JMM     01/21/01        Initial entry
 */

Status_t
mai_recv(IBhandle_t fd, Mai_t * buf, uint64_t timeout)
{
    struct mai_fd  *act;
    uint64_t        timenow;
    uint64_t        wakeup;
    int             rc;
    unsigned int    incarn;

    /*
     * Standard entry stuff 
     */
    IB_ENTER(__func__, fd, buf, timeout, 0);

    if (!gMAI_INITIALIZED)
      {
	  rc = VSTATUS_UNINIT;
	  IB_LOG_ERROR0("MAPI library not initialized");
	  return VSTATUS_UNINIT;
      }

    /*
     * Validate parameters 
     */
    if ((fd < 0) || (fd >= MAI_MAX_CHANNELS))
      {
	  IB_LOG_ERROR("Invalid fd:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }
    if (buf == NULL)
      {
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    act = &gMAI_CHANNELS[fd];

    if (act->state == MAI_FREE)
      {
	  IB_LOG_ERROR("Channel not active:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    /*
     * Remember the incarnation number 
     */
    incarn = act->incarn;

    /*
     * Update the absolute timeout value for this channel 
     */
    if (timeout == 0)
	wakeup = 0;
    else
      {
	  (void)vs_time_get(&timenow);
	  wakeup = timenow + timeout;
      }

    /*
     * Check for data waiting - Must be done with lock WRT producer 
     */
    MAI_HANDLE_LOCK(act);

  mai_recv_retry:

    if (act->mad_cnt)
      {
	  struct mai_data *md;

	  md = mai_dequeue_mbuff(act);

	  if (md) {
	  	memcpy((void *) buf, (void *) &md->mad, sizeof(md->mad));
	  	mai_free_mbuff(md);
	  } 

	  MSTATS_FD_RX(act, buf->type);

	  /*
	   * Decrement waiters count 
	   */

	  MAI_HANDLE_UNLOCK(act);
	  
	  rc = VSTATUS_OK;

	  
	  switch (buf->type)
	  {
	     case  MAI_TYPE_EXTERNAL:
	       {
		 IB_EXIT(__func__, VSTATUS_OK);
	       }
	       break;
	     case MAI_TYPE_INTERNAL:
	       {
		 IB_EXIT(__func__, VSTATUS_MAI_INTERNAL);
	       }
	       break;
	     case  MAI_TYPE_ERROR:
	       {
		 IB_EXIT(__func__, VSTATUS_OK);
	       }
	       break;
	     default:
	       {
		 IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	       }
	  }
	  IB_EXIT(__func__, rc);
	  return rc;
      }

    if (timeout == MAI_RECV_NOWAIT)
      {
	  MAI_HANDLE_UNLOCK(act);
	  IB_EXIT(__func__, VSTATUS_TIMEOUT);
	  return (VSTATUS_TIMEOUT);
      }

    /*
     * See if we have reached (or exceeded timeout) 
     */
    if (wakeup != 0)
      {
	rc = vs_time_get(&timenow);
	  if (timenow > wakeup)
	    {
		MAI_HANDLE_UNLOCK(act);
		IB_EXIT(__func__, VSTATUS_TIMEOUT);
		return (VSTATUS_TIMEOUT);
	    }
      }

    /*
     * Let common mode receive code get the next one 
     */

    /*
     * Increment waiters count 
     */
    act->waiters++;
    act->state = MAI_WAIT;

    MAI_HANDLE_UNLOCK(act);

    rc = mai_getqp(act, wakeup);

    MAI_HANDLE_LOCK(act);

    act->waiters--;

    /*
     * check if someone closed the handle underneath me 
     */
    if (act->state == MAI_FREE || act->incarn != incarn)
      {
	  IB_LOG_ERROR("closed unexpectedly:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  MAI_HANDLE_UNLOCK(act);
	  return (VSTATUS_ILLPARM);
      }

    /*
     * If the state is what I expect then just mark it as busy.
     * This assumption could be violated if the handle was closed while
     * I was waiting. Also it there are other waiters, we have preserve
     * the state until the last one leaves.
     */

    if (act->waiters == 0 && act->state == MAI_WAIT)
      {
	  act->state = MAI_BUSY;
      }

    if (rc == VSTATUS_OK || rc == VSTATUS_TIMEOUT)
	goto mai_recv_retry;	/* And try it again */

    MAI_HANDLE_UNLOCK(act);

    IB_EXIT(__func__, rc);
    return (rc);
}

/*
 * FUNCTION
 *      mai_send_timeout
 * 
 * DESCRIPTION
 *      Send an SMD to the hardware channel (QP0/QP1) interface.  This is a
 *      synchronous call.  It only returns when the next layer down has
 *      returned from the send.
 *
 
 * INPUTS
 *      fd     The channel to send data to
 *      buf    Pointer to SMD (256 bytes) to send
 *      timeout     Send timeout to pass down to the kernel
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_INVALID_HANDL
 *      VSTATUS_NOT_OWNER
 *      VSTATUS_INVALID_MADT
 *      VSTATUS_INVALID_TYPE
 *      VSTATUS_INVALID_METHOD
 *      VSTATUS_INVALID_HOPCNT
 *      VSTATUS_INVALID_LID
 *      VSTATUS_MISSING_ADDRINFO
 *      VSTATUS_MISSING_QP
 *      VSTATUS_INVALID_QP
 *      VSTATUS_ILLPARM
 *      VSTATUS_NOPRIV
 *      ib_send_sma() return codes
 *
 * NOTES
 *      This code currently holds the managment API global lock for the
 *      duration of the ib_send() call.  This means that the hardware channel
 *      can not be modified by do_smi/do_gsi error recovery while a send
 *      is outstanding.  If this turns out to be unacceptable, we can 
 *      consider dropping the locking requirement.
 *
 *      The exact fields that must be validate in the Mai_t structure
 *      have yet to be nailed down.
 *
 * HISTORY
 *      NAME      DATE          REMARKS
 *      JMM     01/21/01        Initial entry
 *      JMM     02/01/01        Added priv check for headers
 *      JMM     03/01/01        Added default values
 */
Status_t
mai_send_timeout(IBhandle_t fd, Mai_t * inbuf, uint64_t timeout)
{
    struct mai_fd  *act;
    struct mai_dc  *dc;

    int             rc;
    uint32_t        mask;
    Mai_t          *buf;
    Mai_t           buffer;

    if (inbuf == NULL)
      {
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    if (inbuf->base.bversion == STL_BASE_VERSION) 
      {
	  uint32_t datalen = inbuf->datasize ? inbuf->datasize : STL_MAD_PAYLOAD_SIZE;
	  return mai_send_stl_timeout(fd, inbuf, &datalen, timeout);
      }

    /*
     * Standard entry stuff 
     */
    IB_ENTER(__func__, fd, inbuf, 0, 0);

    if (!gMAI_INITIALIZED)
      {
	  rc = VSTATUS_UNINIT;
	  IB_LOG_ERROR0("mai_send: MAPI library not initialized");
	  return rc;
      }
    
    memcpy(&buffer, inbuf, sizeof(buffer));
    buf = &buffer;

    /*
     * Validate parameters 
     */

    if ((fd < 0) || (fd >= MAI_MAX_CHANNELS))
      {
	  IB_LOG_ERROR("mai_send: Invalid fd:", fd);
	  IB_EXIT(__func__, VSTATUS_INVALID_HANDL);
	  return (VSTATUS_INVALID_HANDL);
      }

    act = &gMAI_CHANNELS[fd];

    if (act->state == MAI_FREE)
      {
	  IB_LOG_ERROR("mai_send: Channel not active:", fd);
	  IB_EXIT(__func__, VSTATUS_INVALID_HANDL);
	  return (VSTATUS_INVALID_HANDL);
      }

    /*
     * Make sure we have a hardware channel, and that hardware channel is open 
     */
    if ((act->qp != 0) && (act->qp != 1))
      {
	  IB_LOG_ERROR("mai_send: Invalid QP (not 0/1) on handle qp:",
		       act->qp);
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return VSTATUS_BAD;
      }

    MAI_HANDLE_LOCK(act);

    /*
     * Take the pointer to the down call handle 
     */
    dc = act->down_fd;

    if (dc == NULL)
      {
	  MAI_HANDLE_UNLOCK(act);

	  IB_LOG_ERROR0("mai_send: Invalid hardware channel");
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return VSTATUS_BAD;
      }

    /*
     * If the user does not have an active dev,port,qp set, then we use the on
     * asociated with the handle.
     */

    if (!(buf->active & MAI_ACT_QP))
      {
	  buf->active |= MAI_ACT_QP;
	  buf->qp = act->qp;
      }
/*
 * Since QP redirection is supported, this check is no longer valid.
 */
#if 0
    else
      {
	  if (buf->qp != 0 && buf->qp != 1)
	    {
		MAI_HANDLE_UNLOCK(act);
		IB_LOG_ERROR("mai_send: Invalid QP:", buf->qp);
		return (VSTATUS_INVALID_MAD);
	    }
      }
#endif

    if (!(buf->active & MAI_ACT_PORT))
      {
	  buf->active |= MAI_ACT_PORT;
	  buf->port = act->port;
      }
    else
      {
	  if (buf->port < 0 || buf->port > MAI_MAX_PORT)
	    {
		MAI_HANDLE_UNLOCK(act);
		IB_LOG_ERROR("mai_send: Invalid port:", buf->port);
		return (VSTATUS_INVALID_MAD);
	    }
      }

    if (!(buf->active & MAI_ACT_DEV))
      {
	  buf->active |= MAI_ACT_DEV;
	  buf->dev = act->dev;
      }
    else
      {
	  if (buf->dev < 0 || buf->dev > MAI_MAX_DEV)
	    {
		MAI_HANDLE_UNLOCK(act);
		IB_LOG_ERROR("mai_send: Invalid dev:", buf->dev);
		return (VSTATUS_INVALID_MAD);
	    }
      }

    /*
     * No need to hold the handle any longer. We have taken all we need
     * from it 
     */
    MAI_HANDLE_UNLOCK(act);

    IB_LOG_VERBOSE("Send  dev :", buf->dev);
    IB_LOG_VERBOSE("Send  port:", buf->port);
    IB_LOG_VERBOSE("Send  qp  :", buf->qp);

    /*
     * Make sure they gave us enough fields in the MAD to send something 
     */
    mask = MAI_ACT_TYPE;
    if ((buf->active & mask) != mask)
      {
	mask_error:
	  IB_LOG_ERROR("mai_send: active mask error:", buf->active);
	  IB_EXIT(__func__, VSTATUS_INVALID_MADT);
	  return (VSTATUS_INVALID_MADT);
      }

    /*
     * Validate the type code 
     */
    switch (buf->type)
      {
      case MAI_TYPE_EXTERNAL:
	  {
	      if ((buf->active & MAI_ACT_BASE) != MAI_ACT_BASE)
		  goto mask_error;
	  }
	  break;
      case MAI_TYPE_INTERNAL:
	  break;

      default:
	  IB_LOG_ERROR("mai_send: Invalid mai_mad type:", buf->type);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    /*
     * Manage setting base fields of the various headers 
     */
    if (buf->type == MAI_TYPE_EXTERNAL) {
        if (buf->base.bversion != MAD_BVERSION) {
            IB_LOG_ERROR("mai_send: Invalid MAD base version:", buf->base.bversion);
            IB_EXIT(__func__, VSTATUS_INVALID_MAD);
            return VSTATUS_INVALID_MAD;
        }
        switch (buf->base.mclass) {
        case MAD_CV_SUBN_ADM:
            if (buf->base.cversion != SA_MAD_CVERSION) {
                IB_LOG_ERROR("mai_send: Invalid SA MAD class version:", buf->base.cversion);
                IB_EXIT(__func__, VSTATUS_INVALID_MAD);
                return VSTATUS_INVALID_MAD;
            }
            break;
        case MAD_CV_VFI_PM:
            if (buf->base.cversion != MAD_CVERSION && buf->base.cversion != RMPP_MAD_CVERSION) {
                IB_LOG_ERROR("mai_send: Invalid PM MAD class version:", buf->base.cversion);
                IB_EXIT(__func__, VSTATUS_INVALID_MAD);
                return VSTATUS_INVALID_MAD;
            }
            break;
        default:
            if (buf->base.cversion != MAD_CVERSION) {
                IB_LOG_ERROR("mai_send: Invalid MAD class version:", buf->base.cversion);
                IB_EXIT(__func__, VSTATUS_INVALID_MAD);
                return VSTATUS_INVALID_MAD;
            }
        }

	  buf->base.rsvd3 = 0;


	  /*
	   * Addr Info
       * At this point we have the information needed by lower layers to
       * build a valid packet.  
	   * The fields break down like this:
	   *
	   *     FIELD             DEFAULT                  REQUIRED
	   *   Src LID                                          X (Note 4)
	   *   Dest LID                                         X
	   *   Service Level                   15
	   *   P_KEY                     (Note 1)              X
	   *   SrcQP                       (Note 1)            X
	   *   DestQP                     buf->qp
	   *   Q_Key                       (Note 1)            X
	   *
	   * Note 4:
	   *   The lowest level driver is allowed to override the Src LID with our
	   * valid source LID before sending out the MAD.  The management datagram
	   * subsystem does not (currently) do this.
	   */

	  if ((buf->active & MAI_ACT_ADDRINFO) == 0)
	    {
		IB_EXIT(__func__, VSTATUS_MISSING_ADDRINFO);
		return (VSTATUS_MISSING_ADDRINFO);
	    }

	  /*
	   * Make sure that the slid and the dlid are valid 
	   */
/* Note: For linux/Ibaccess builds we can ignore the slid, since that 
		 is handled within Umadt */
#ifdef CAL_IBACCESS
	  if (buf->addrInfo.dlid == STL_LID_RESERVED)
	    {
		IB_EXIT(__func__, VSTATUS_INVALID_LID);
		IB_LOG_ERRORX("mai_send, invalid DLID:", buf->addrInfo.dlid);
		return (VSTATUS_INVALID_LID);
	    }
#else
	  if ((buf->addrInfo.slid == STL_LID_RESERVED)
	      || (buf->addrInfo.dlid == STL_LID_RESERVED))
	    {
		IB_EXIT(__func__, VSTATUS_INVALID_LID);
		return (VSTATUS_INVALID_LID);
	    }
#endif

	  if (buf->base.method & MAD_CM_REPLY) /* Else if a packet is a reply, flip the source and dest QP */
	    {
		/* We can use the src qp in the packet since it has not been overwritten yet */
		buf->addrInfo.destqp = buf->addrInfo.srcqp;
		buf->addrInfo.srcqp = buf->qp;
	    }
	}

    /*
     * We have done all the validation we can - Pass it on 
     */

    
    MAI_HANDLE_LOCK(act);

    MSTATS_FD_TX(act, buf->type);
    MSTATS_DC_TX(dc, buf->type);

    MAI_HANDLE_UNLOCK(act);


    /*
     * All checking has been done, now we send to common services 
     */
    rc = stl_send_sma(dc->hndl, buf, timeout);
    if (rc)
      {
	  IB_LOG_INFO("mai_send: Error on hardware channel hndl:", dc->hndl);
      }

    IB_EXIT(__func__, rc);
    return (rc);
}

/*
 * FUNCTION
 *      mai_send_stl_timeout
 * 
 * DESCRIPTION
 *      Send an SMD to the hardware channel (QP0/QP1) interface.  This is a
 *      synchronous call.  It only returns when the next layer down has
 *      returned from the send.
 *
 
 * INPUTS
 *      fd     The channel to send data to
 *      buf    Pointer to SMD (256 bytes) to send
 *      timeout     Send timeout to pass down to the kernel
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_INVALID_HANDL
 *      VSTATUS_NOT_OWNER
 *      VSTATUS_INVALID_MADT
 *      VSTATUS_INVALID_TYPE
 *      VSTATUS_INVALID_METHOD
 *      VSTATUS_INVALID_HOPCNT
 *      VSTATUS_INVALID_LID
 *      VSTATUS_MISSING_ADDRINFO
 *      VSTATUS_MISSING_QP
 *      VSTATUS_INVALID_QP
 *      VSTATUS_ILLPARM
 *      VSTATUS_NOPRIV
 *      ib_send_sma() return codes
 *
 * NOTES
 *      This code currently holds the managment API global lock for the
 *      duration of the ib_send() call.  This means that the hardware channel
 *      can not be modified by do_smi/do_gsi error recovery while a send
 *      is outstanding.  If this turns out to be unacceptable, we can 
 *      consider dropping the locking requirement.
 *
 *      The exact fields that must be validate in the Mai_t structure
 *      have yet to be nailed down.
 *
 * HISTORY
 *      NAME      DATE          REMARKS
 *      JMM     01/21/01        Initial entry
 *      JMM     02/01/01        Added priv check for headers
 *      JMM     03/01/01        Added default values
 */
Status_t
mai_send_stl_timeout(IBhandle_t fd, Mai_t *inbuf, uint32_t *datalen, uint64_t timeout) 
{ 
   struct mai_fd  *act; 
   struct mai_dc  *dc; 
   
   int             rc; 
   uint32_t        mask; 
   Mai_t          *buf; 
   Mai_t           buffer; 
   
   /*
    * Standard entry stuff 
    */
   IB_ENTER(__func__, fd, inbuf, datalen, 0); 
   
   if (!gMAI_INITIALIZED) {
      rc = VSTATUS_UNINIT; 
      IB_LOG_ERROR0("mai_send_stl: MAPI library not initialized"); 
      return rc;
   }
   
   
   if (inbuf == NULL || datalen == NULL) {
      IB_EXIT(__func__, VSTATUS_ILLPARM); 
      return (VSTATUS_ILLPARM);
   } else {
      memcpy(&buffer, inbuf, sizeof(buffer)); 
      buf = &buffer;
      buf->active |= MAI_ACT_DATASIZE;
      buf->datasize = Min(*datalen, STL_MAD_PAYLOAD_SIZE);
   }
   
   /*
    * Validate parameters 
    */
   
   if ((fd < 0) || (fd >= MAI_MAX_CHANNELS)) {
      IB_LOG_ERROR("mai_send_stl: Invalid fd:", fd); 
      IB_EXIT(__func__, VSTATUS_INVALID_HANDL); 
      return (VSTATUS_INVALID_HANDL);
   }
   
   act = &gMAI_CHANNELS[fd]; 
   
   if (act->state == MAI_FREE) {
      IB_LOG_ERROR("mai_send_stl: Channel not active:", fd); 
      IB_EXIT(__func__, VSTATUS_INVALID_HANDL); 
      return (VSTATUS_INVALID_HANDL);
   }
   
   /*
    * Make sure we have a hardware channel, and that hardware channel is open 
    */
   if ((act->qp != 0) && (act->qp != 1)) {
      IB_LOG_ERROR("mai_send_stl: Invalid QP (not 0/1) on handle qp:", 
                   act->qp); 
      IB_EXIT(__func__, VSTATUS_BAD); 
      return VSTATUS_BAD;
   }
   
   MAI_HANDLE_LOCK(act); 
   
   /*
    * Take the pointer to the down call handle 
    */
   dc = act->down_fd; 
   
   if (dc == NULL) {
      MAI_HANDLE_UNLOCK(act); 
      
      IB_LOG_ERROR0("mai_send_stl: Invalid hardware channel"); 
      IB_EXIT(__func__, VSTATUS_BAD); 
      return VSTATUS_BAD;
   }
   
   /*
    * If the user does not have an active dev,port,qp set, then we use the on
    * asociated with the handle.
    */
   
   if (!(buf->active & MAI_ACT_QP)) {
      buf->active |= MAI_ACT_QP; 
      buf->qp = act->qp;
   }
   
   if (!(buf->active & MAI_ACT_PORT)) {
      buf->active |= MAI_ACT_PORT; 
      buf->port = act->port;
   } else {
      if (buf->port < 0 || buf->port > MAI_MAX_PORT) {
         MAI_HANDLE_UNLOCK(act); 
         IB_LOG_ERROR("mai_send_stl: Invalid port:", buf->port); 
         return (VSTATUS_INVALID_MAD);
      }
   }
   
   if (!(buf->active & MAI_ACT_DEV)) {
      buf->active |= MAI_ACT_DEV; 
      buf->dev = act->dev;
   } else {
      if (buf->dev < 0 || buf->dev > MAI_MAX_DEV) {
         MAI_HANDLE_UNLOCK(act); 
         IB_LOG_ERROR("mai_send_stl: Invalid dev:", buf->dev); 
         return (VSTATUS_INVALID_MAD);
      }
   }
   
   /*
    * No need to hold the handle any longer. We have taken all we need
    * from it 
    */
   MAI_HANDLE_UNLOCK(act); 
   
   IB_LOG_VERBOSE("Send  dev :", buf->dev); 
   IB_LOG_VERBOSE("Send  port:", buf->port); 
   IB_LOG_VERBOSE("Send  qp  :", buf->qp); 
   
   /*
    * Make sure they gave us enough fields in the MAD to send something 
    */
   mask = MAI_ACT_TYPE; 
   if ((buf->active & mask) != mask) {
   mask_error:
      IB_LOG_ERROR("mai_send_stl: active mask error:", buf->active); 
      IB_EXIT(__func__, VSTATUS_INVALID_MADT); 
      return (VSTATUS_INVALID_MADT);
   }
   
   /*
    * Validate the type code 
    */
   switch (buf->type) {
   case MAI_TYPE_EXTERNAL:
      {
         if ((buf->active & MAI_ACT_BASE) != MAI_ACT_BASE) 
            goto mask_error;
      }
      break; 
   case MAI_TYPE_INTERNAL:
      break; 
      
   default:
      IB_LOG_ERROR("mai_send_stl: Invalid mai_mad type:", buf->type); 
      IB_EXIT(__func__, VSTATUS_ILLPARM); 
      return (VSTATUS_ILLPARM);
   }
   
   /*
    * Manage setting base fields of the various headers 
    */
   if (buf->type == MAI_TYPE_EXTERNAL) {
      if (buf->base.bversion != STL_BASE_VERSION) {
         IB_LOG_ERROR("mai_send_stl: Invalid MAD base version:", buf->base.bversion); 
         IB_EXIT(__func__, VSTATUS_INVALID_MAD); 
         return VSTATUS_INVALID_MAD;
      }
      switch (buf->base.mclass) {
      case MAD_CV_SUBN_ADM:
         if (buf->base.cversion != STL_SA_CLASS_VERSION) {
            IB_LOG_ERROR("mai_send_stl: Invalid SA MAD class version:", buf->base.cversion); 
            IB_EXIT(__func__, VSTATUS_INVALID_MAD); 
            return VSTATUS_INVALID_MAD;
         }
         break; 
      case MAD_CV_VFI_PM:
         if (buf->base.cversion != STL_SA_CLASS_VERSION) {
            IB_LOG_ERROR("mai_send_stl: Invalid PM MAD class version:", buf->base.cversion); 
            IB_EXIT(__func__, VSTATUS_INVALID_MAD); 
            return VSTATUS_INVALID_MAD;
         }
         break; 
      default:
         if (buf->base.cversion != STL_SM_CLASS_VERSION) {
            IB_LOG_ERROR("mai_send_stl: Invalid MAD class version:", buf->base.cversion); 
            IB_EXIT(__func__, VSTATUS_INVALID_MAD); 
            return VSTATUS_INVALID_MAD;
         }
      }
      
      buf->base.rsvd3 = 0; 
      
      
      /*
       * Address Info 
	   *   At this point we have the information needed by lower layers to
       * build a valid packet.
       * The fields break down like this:
       *
       *     FIELD             DEFAULT                  REQUIRED
       *   Src LID                                          X (Note 4)
       *   Dest LID                                         X
       *   Service Level                   15
       *   P_KEY                     (Note 1)              X
       *   DestQP                     buf->qp
       *
       * Note 4:
       *   The lowest level driver is allowed to override the Src LID with our
       * valid source LID before sending out the MAD.  The management datagram
       * subsystem does not (currently) do this.
       */
      
      if ((buf->active & MAI_ACT_ADDRINFO) == 0) {
         /*
          * We cannot do this either .. the mad could be intended
          * to have lid routed part of the way and then DR routed
          * the other. So if addrInfo is not defined we don't know what
          * to do with it. PAW 
          */
         
         IB_EXIT(__func__, VSTATUS_MISSING_ADDRINFO); 
         return (VSTATUS_MISSING_ADDRINFO);
         
      }
      
      /*
       * Make sure that the slid and the dlid are valid 
       */
/* Note: For linux/Ibaccess builds we can ignore the slid, since that 
         is handled within Umadt */
#ifdef CAL_IBACCESS
      if (buf->addrInfo.dlid == STL_LID_RESERVED) {
         IB_EXIT(__func__, VSTATUS_INVALID_LID); 
         IB_LOG_ERRORX("mai_send_stl, invalid DLID:", buf->addrInfo.dlid); 
         return (VSTATUS_INVALID_LID);
      }
#else
      if ((buf->addrInfo.slid == STL_LID_RESERVED)
          || (buf->addrInfo.dlid == STL_LID_RESERVED)) {
         IB_EXIT(__func__, VSTATUS_INVALID_LID); 
         return (VSTATUS_INVALID_LID);
      }
#endif
      
      if (buf->base.method & MAD_CM_REPLY) { /* Else if a packet is a reply, flip the source and dest QP */
         /* We can use the src qp in the packet since it has not been overwritten yet */
         buf->addrInfo.destqp = buf->addrInfo.srcqp;
         /* We can't use the dst qp in the packet since it has been overwritten.
          * We can use buf->qp since we know that is always the correct source
          */
         buf->addrInfo.srcqp = buf->qp;
      }
   }
   
   
   /*
    * We have done all the validation we can - Pass it on 
    */
   
   
   MAI_HANDLE_LOCK(act); 
   
   MSTATS_FD_TX(act, buf->type); 
   MSTATS_DC_TX(dc, buf->type); 
   
   MAI_HANDLE_UNLOCK(act); 
   
   
   /*
    * All checking has been done, now we send to common services 
    */
   rc = stl_send_sma(dc->hndl, buf, timeout); 
   if (rc) {
      IB_LOG_INFO("mai_send_stl: Error on hardware channel hndl:", dc->hndl);
   }
   
   IB_EXIT(__func__, rc);
    return (rc);
}

//==============================================================================
// mai_init_portinfo
//==============================================================================
// This is the up_fd.
static IBhandle_t mai_portinfo_fd = -1;

static
Status_t
mai_init_portinfo(uint32_t dev, uint32_t port)
{
	if (mai_portinfo_fd > -1)
		return VSTATUS_OK;
	return mai_open(MAI_GSI_QP, dev, port, &mai_portinfo_fd);
}

void
mai_close_portinfo(void)
{
	if (mai_portinfo_fd > -1) {
		mai_close(mai_portinfo_fd);
		mai_portinfo_fd = -1;
	}
}

/*
 * stl_get_portinfo
 *   Get the STL_PORT_INFO on a given port.  
 *
 * INPUTS
 *      handle      ib device handle
 *      *pip        Pointer to PORT_INFO
 *      user_port   Port to get the PORT_INFO for
 *      dev         The device number associated with handle
 *      port        The port associated with handle 
 *      qpn         The qp associated with the handle

 *
 * RETURNS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *      VSTATUS_NOPRIV
 */

static
Status_t
stl_get_portinfo(IBhandle_t handle, STL_PORT_INFO * pip, uint8_t user_port,
                int16_t dev, uint8_t port, uint32_t qpn)
{
	Status_t rc, saved_rc;
	Filter_t filter;
	int i;
	uint64_t tid;
    uint32_t datalen;
	uint8_t  path[2];
    DRStlSmp_t *drp;
	Mai_t out_mad, in_mad;
	
	IB_ENTER(__func__, handle, pip, user_port, 0);

	rc = mai_init_portinfo(dev, port);
	if (rc != VSTATUS_OK) {
		IB_LOG_WARNRC("Error in mai_init_portinfo; rc:", rc);
		return(VSTATUS_BAD);
	}
	
	memset(path, 0, sizeof(path));
	mai_alloc_tid(mai_portinfo_fd, MAD_CV_SUBN_DR, &tid);
	
	// create MAD
	Mai_Init(&out_mad);
	out_mad.dev     = dev;
	out_mad.port    = port;
	out_mad.qp      = 0;
	out_mad.active |= (MAI_ACT_DEV | MAI_ACT_PORT | MAI_ACT_QP);
	
	AddrInfo_Init(&out_mad, STL_LID_PERMISSIVE, STL_LID_PERMISSIVE, SMI_MAD_SL, mai_get_default_pkey(), MAI_SMI_QP, MAI_SMI_QP, 0);
	DRStlMad_Init(&out_mad, MAD_CM_GET, tid, STL_MCLASS_ATTRIB_ID_PORT_INFO, 0x01000000 | user_port, 0, path);
    datalen = STL_SMP_DR_HDR_LEN;
	
	// create MAI filter
	Filter_Init(&filter, 0, 0);
	
	filter.active |= MAI_ACT_ADDRINFO;
	filter.active |= MAI_ACT_BASE;
	filter.active |= MAI_ACT_TYPE;
	filter.active |= MAI_ACT_DATA;
	filter.active |= MAI_ACT_DEV;
	filter.active |= MAI_ACT_PORT;
	filter.active |= MAI_ACT_QP;
	filter.active |= MAI_ACT_FMASK;
	filter.type    = MAI_TYPE_EXTERNAL;
	filter.dev     = dev;
	filter.port    = (port == 0) ? MAI_TYPE_ANY : port;
	filter.qp      = 0;

	filter.value.bversion = STL_BASE_VERSION;
	filter.value.cversion = STL_SM_CLASS_VERSION;
	filter.value.mclass   = MAD_CV_SUBN_DR;
	filter.value.method   = MAD_CM_GET_RESP;
	filter.value.aid      = STL_MCLASS_ATTRIB_ID_PORT_INFO;
	filter.value.amod     = 0x01000000 | user_port;
	filter.value.tid      = tid;

	filter.mask.bversion  = 0xff;
	filter.mask.cversion  = 0xff;
	filter.mask.mclass	  = 0xff;
	filter.mask.method    = 0xff;
	filter.mask.aid       = 0xffff;
	filter.mask.amod      = 0xffffffff;
	filter.mask.tid       = 0xffffffffffffffffull;
	
	MAI_SET_FILTER_NAME(&filter, "stl_get_portinfo");
	
	rc = mai_filter_create(mai_portinfo_fd, &filter, VFILTER_SHARE | VFILTER_PURGE);
	if (rc != VSTATUS_OK) {
		IB_LOG_WARNRC("Error creating MAI filter; rc:", rc);
		return VSTATUS_BAD;
	}
	
	// do send/recv
	for (i = 0; i < MAD_RETRIES; i++) {
		rc = mai_send_stl_timeout(mai_portinfo_fd, &out_mad, &datalen, VTIMER_1S/4);
		if (rc != VSTATUS_OK) {
			IB_LOG_WARNRC("Error on send; rc:", rc);
			break;
		}
		rc = mai_recv(mai_portinfo_fd, &in_mad, VTIMER_1S/4);
		if (rc == VSTATUS_OK) {
			break;
		} else if (rc != VSTATUS_TIMEOUT) {
			IB_LOG_WARNRC("Error on receive; rc:", rc);
			break;
		}
	}
	
	// remove the MAI filter
	saved_rc = rc;
	rc = mai_filter_delete(mai_portinfo_fd, &filter, VFILTER_SHARE | VFILTER_PURGE);
	if (rc != VSTATUS_OK) {
		IB_LOG_WARNRC("Error deleting MAI filter; rc:", rc);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}
	
	// did send/receive work?
	if (saved_rc != VSTATUS_OK) {
		IB_EXIT(__func__, saved_rc);
		return saved_rc;
	}
	
	// confirm MAD status
	if ((in_mad.base.status & MAD_STATUS_MASK) != 0) {
		IB_LOG_WARN("Bad MAD status:", in_mad.base.status);
		IB_EXIT(__func__, VSTATUS_BAD);
		return VSTATUS_BAD;
	}
	
	// extract the PortInfo record
     drp = (DRStlSmp_t *)in_mad.data;
     (void)memcpy((void *)pip, (void *)drp->SMPData, sizeof(STL_PORT_INFO)); 
     (void)BSWAP_STL_PORT_INFO(pip); 
	IB_EXIT(__func__, VSTATUS_OK);
	return VSTATUS_OK;
}

/*
 *  mai_get_stl_portinfo
 *
 * DESCRIPTION
 *      This function allows the local user to retrieve the portinfo
 *      of the local port.  
 *
 *
 * INPUTS
 *      fd      mai handle
 *      pinfop  Pointer to STL_PORT_INFO
 *      port    The port to get
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *
 */

Status_t
mai_get_stl_portinfo(IBhandle_t fd, STL_PORT_INFO * pinfop, uint8_t port)
{
    Status_t        status;
    struct mai_fd  *act;

    IB_ENTER(__func__, fd, pinfop, port, 0);

    if (!gMAI_INITIALIZED)
      {
	  IB_LOG_ERROR("not initialized port:", port);
	  IB_EXIT(__func__, VSTATUS_INVALID_HANDL);
	  return (VSTATUS_BAD);
      }

    /*
     * Validate parameters 
     */
    if ((fd < 0) || (fd >= MAI_MAX_CHANNELS))
      {
	  IB_LOG_ERROR("Invalid fd:", fd);
	  IB_EXIT(__func__, VSTATUS_INVALID_HANDL);
	  return (VSTATUS_INVALID_HANDL);
      }
    if (pinfop == NULL)
      {
	  IB_LOG_ERROR("NULL pinfop fd:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    act = &gMAI_CHANNELS[fd];

    if (act->state == MAI_FREE)
      {
	  IB_LOG_ERROR("Channel not active fd:", fd);
	  IB_EXIT(__func__, VSTATUS_INVALID_HANDL);
	  return (VSTATUS_INVALID_HANDL);
      }

    status = stl_get_portinfo(fd, pinfop, port, act->dev, act->port, act->qp);

    IB_EXIT(__func__, status);
    return (status);
}


void mai_set_default_pkey(uint16_t pKey)
{
	maiDefPKey = pKey;
}



uint16_t mai_get_default_pkey(void)
{
#ifdef __VXWORKS__
	return getDefaultPKey();
#else
	return maiDefPKey;
#endif
}

