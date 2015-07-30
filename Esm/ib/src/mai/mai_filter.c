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
 *      mai_filter.c                                       MAPI 0.05        *
 *                                                                          *
 * DESCRIPTION                                                              *
 *      This file contains all the routines that understand the semantic    *
 *      content of filters.  
 *                                                                          *
 *                                                                          *
 *                                                                          *
 ****************************************************************************/
#include "mai_l.h"		/* Local mai function definitions */

/*
 * FUNCTION
 *      mai_filter_hcreate
 *
 * DESCRIPTION
 *      Add a filter to an open channel.  This function will add both 
 *      absorbing filters and copy filters.  In the case where a new filter
 *      has been added to absorb SMDs, this function will notify all
 *      other channels that would have received that SMD. It returns a handle
 *      to the created filter.
 *
 *
 *
 * INPUTS
 *      fd      Channel number to add the filter to
 *      filter  Pointer to the filter to add to the channel
 *      flags   Flags to modify filter (absorb or copy)
 *      fh      Pointer to the filter handle location
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *      VSTATUS_NOPRIV
 *
 */
Status_t
mai_filter_hcreate(IBhandle_t fd, Filter_t * filter,
		   uint32_t flags, IBhandle_t * fh)
{
    struct mai_fd  *act;
    struct mai_filter *filt,
                   *q;
    int             rc = VSTATUS_OK;
    Filter_t       *ft,
                    f;

    /*
     * Standard entry header 
     */
    IB_ENTER(__func__, fd, filter, flags, fh);

    if (!gMAI_INITIALIZED)
      {
	  rc = VSTATUS_UNINIT;
	  IB_LOG_ERROR0("MAPI library not initialized");
	  return rc;
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

    act = &gMAI_CHANNELS[fd];

    if (act->state == MAI_FREE)
      {
	  IB_LOG_ERROR("Channel not active fd:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    
    /*
     * Verify a valid and open hardware channel[] 
     */
    if (act->down_fd == NULL)
      {
	  IB_LOG_ERROR0("hardware channel invalid");
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return (VSTATUS_BAD);
      }

#ifdef IB_STACK_OPENIB	  
	  /*
	   * Make sure the upper 32 bits are not masked on OFED
	   */
	  filter->mask.tid &= 0x00000000ffffffffull;
#endif

    /*
     * Validate the Filter_t structure 
     */
    rc = mai_validate_filter(filter);
    if (!rc)
      {
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    /*
     * Copyin the filter- so we chan change the local copy 
     */
    memcpy(&f, filter, sizeof(f));
    ft = &f;

    /*
     * Lock the handle so that no one else does fishy things behind our
     * back 
     */
    MAI_HANDLE_LOCK(act);

    /*
     * If no association set with the filter on dev,port,qp then do so 
     */
    if (!(ft->active & MAI_ACT_QP))
      {
	  ft->active |= MAI_ACT_QP;
	  ft->qp = act->qp;
      }

    if (!(ft->active & MAI_ACT_DEV))
      {
	  ft->active |= MAI_ACT_DEV;
	  ft->dev = act->dev;
      }

//FIXME: Todd - come back to this, do we need port checks in filters at all
// if we don't need port checks, we do not need nodeType in mai_fd_t
    if (!(ft->active & MAI_ACT_PORT))
      {
	  if (act->nodeType != NI_TYPE_SWITCH)
	    {
		/*
		 * Accept MADs only from the port we live on 
		 */
		ft->port = act->port;

	    }
	  else
	    {
		/*
		 * accept MADs from any port on the switch
		 */
		ft->port = MAI_TYPE_ANY;
	    }
	  ft->active |= MAI_ACT_PORT;
      }

    /*
     * First try and see if this filter exist on this handle 
     */
    rc = mai_filter_find(act, ft, flags, &filt, &q);

    if (!rc)
      {
	  Filter_t       *fp = &filt->filter;

	  /*
	   * We have a match. Just increment the reference count and
	   * return
	   */
	  fp->refcnt++;
	  /*
	   * Return a pointer to the filter handle 
	   */
	  *fh = filt->hndl;

	  MAI_HANDLE_UNLOCK(act);

	  IB_EXIT(__func__, VSTATUS_OK);
	  return (VSTATUS_OK);
      }

    /*
     * Move the Filter_t into one of our filter structures 
     */

    rc = mai_load_filter(ft, &filt);

    if (rc != VSTATUS_OK)
      {
	  MAI_HANDLE_UNLOCK(act);
	  IB_EXIT(__func__, rc);
	  return (rc);
      }

    filt->qp = act->qp;

    /*
     * Set the refrence count 
     */
    filt->filter.refcnt = 1;

    /*
     * remember the flags value 
     */
    filt->flags = flags;

    /*
     * pass the full flags value down 
     */
    filt->filter.handle = (void *) ((unint)flags);

    IB_LOG_INFO("creating shared filter, flags =",
		flags);

	/*
	 * Put the filter on the list 
	 */
	mai_filter_enqueue(act, filt);

	if (flags & VFILTER_ONCE)
	  {
		/*
		 * Now set the one shot flag.
		 */
		filt->once = 1;
	  }

	/*
	 * Return  the filter handle 
	 */
	*fh = filt->hndl;

    MAI_HANDLE_UNLOCK(act);

    IB_EXIT(__func__, rc);
    return (rc);
}

/*
 * FUNCTION
 *      mai_filter_delete
 *
 * DESCRIPTION
 *      Removes a filter from a channel.  It will remove the first instance
 *      of a matching filter from the channel.
 *
 *
 * INPUTS
 *      fd      Channel number to add the filter to
 *      filter  Pointer to the filter to add to the channel
 *      flags   Flags to modify filter (absorb or copy)
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *
 */
Status_t
mai_filter_delete(IBhandle_t fd, Filter_t * infilter, uint32_t flags)
{
    struct mai_fd  *act;
    struct mai_filter *p,
                   *q;
    int             rc;
    Filter_t        *ft,
                    fbuf,
                   *f;

    /*
     * Standard entry conventions 
     */
    IB_ENTER(__func__, fd, infilter, flags, 0);

    if (!gMAI_INITIALIZED)
      {
	  rc = VSTATUS_UNINIT;
	  IB_LOG_ERROR0("MAPI library not initialized");
	  return rc;
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

    act = &gMAI_CHANNELS[fd];

    if (act->state == MAI_FREE)
      {
	  IB_LOG_ERROR("Channel not active fd:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    if (infilter == NULL)
      {
	  IB_LOG_ERROR("invalid filter param fd:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    /*
     * Verify a valid and open hardware channel[] 
     */

    if (act->down_fd == NULL)
      {
	  IB_LOG_ERROR0("hardware channel invalid");
	  IB_EXIT(__func__, VSTATUS_BAD);
	  return (VSTATUS_BAD);
      }

    /*
     * Copy in the filter so we can modify it if we need to 
     */
    memcpy(&fbuf, infilter, sizeof(fbuf));
    ft = &fbuf;

    /*
     * If no association set with the filter on dev,port,qp then do so now 
     */

    if (!(ft->active & MAI_ACT_QP))
      {
	  ft->active |= MAI_ACT_QP;
	  ft->qp = act->qp;
      }

    if (!(ft->active & MAI_ACT_DEV))
      {
	  ft->active |= MAI_ACT_DEV;
	  ft->dev = act->dev;
      }


//FIXME: Todd - come back to this, do we need port checks in filters at all
// if we don't need port checks, we do not need nodeType in mai_fd_t
    if (!(ft->active & MAI_ACT_PORT))
      {
	  if (act->nodeType != NI_TYPE_SWITCH)
	    {
		/*
		 * Accept MADs only from the port we live on 
		 */
		ft->port = act->port;

	    }
	  else
	    {
		/*
		 * accept MADs from any port on the switch
		 */
		ft->port = MAI_TYPE_ANY;
	    }
	  ft->active |= MAI_ACT_PORT;
      }


    /*
     * We now have everything we need to use the filter match tools 
     */

    /*
     * Lock the handle so that no one else does fishy things behind our
     * back 
     */
    MAI_HANDLE_LOCK(act);

    /*
     * A normal filter.  Search it out 
     */
    rc = mai_filter_find(act, ft, flags, &p, &q);

    if (rc)
      {
	  /*
	   * If we didn't find it, let them know 
	   */
	  MAI_HANDLE_UNLOCK(act);

	  IB_EXIT(__func__, VSTATUS_NXIO);
	  return (VSTATUS_NXIO);
      }

    f = &p->filter;
    f->refcnt--;

    /*
     * If the reference count on the filter is not zero keep it 
     */
    if (f->refcnt > 0)
      {
	  rc = VSTATUS_OK;

	  MAI_HANDLE_UNLOCK(act);

	  IB_EXIT(__func__, rc);
	  return (rc);
      }

    /*
     * filters are trully identical 
     */
    (void ) mai_filter_dequeue(act, p);

    if (flags & VFILTER_PURGE)
      {
	  mai_filter_purge(act, ft);
      }

    MAI_HANDLE_UNLOCK(act);

    mai_free_filter(p);

    IB_EXIT(__func__, rc);
    return (rc);
}

/*
 * FUNCTION
 *      mai_filter_handle
 *
 * DESCRIPTION
 *       Returns the handle to previously chreated filter.
 *
 * CALLED BY
 *
 * CALLS
 *      maif_filter_handle
 *
 * INPUTS
 *      devh  - device handle returned by mai_open and used in filter create.
 *      flags - a set of characteristics to determine behavior. 
 *      infilter - the filter definition.
 *      fh    - pointer to filter handle returned by call.
 *
 * OUTPUTS
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *
 */
Status_t
mai_filter_handle(IBhandle_t fd,
		  Filter_t * infilter, uint32_t flags, IBhandle_t * fh)
{

    struct mai_fd  *act;
    struct mai_filter *p,
                   *q;
    int             rc;
    Filter_t        *ft,
                    fbuf;

    /*
     * Standard entry conventions 
     */
    IB_ENTER(__func__, fd, infilter, flags, fh);

    if (!gMAI_INITIALIZED)
      {
	  rc = VSTATUS_UNINIT;
	  IB_LOG_ERROR0("MAPI library not initialized");
	  return rc;

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

    act = &gMAI_CHANNELS[fd];

    if (act->state == MAI_FREE)
      {
	  IB_LOG_ERROR("Channel not active fd:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    if (infilter == NULL || fh == NULL)
      {
	  IB_LOG_ERROR("invalid filter param fd:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    /*
     * Validate the Filter_t structure 
     */
    rc = mai_validate_filter(infilter);
    if (!rc)
      {
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    memcpy(&fbuf, infilter, sizeof(fbuf));
    ft =  &fbuf;

    /*
     * If no association set with the filter on dev,port,qp then do so now 
     */

    MAI_HANDLE_LOCK(act);

    if (!(ft->active & MAI_ACT_QP))
      {
	  ft->active |= MAI_ACT_QP;
	  ft->qp = act->qp;
      }

    if (!(ft->active & MAI_ACT_DEV))
      {
	  ft->active |= MAI_ACT_DEV;
	  ft->dev = act->dev;
      }

//FIXME: Todd - come back to this, do we need port checks in filters at all
// if we don't need port checks, we do not need nodeType in mai_fd_t
    if (!(ft->active & MAI_ACT_PORT))
      {
	  if (act->nodeType != NI_TYPE_SWITCH)
	    {
		/*
		 * Accept MADs only from the port we live on 
		 */
		ft->port = act->port;

	    }
	  else
	    {
		/*
		 * accept MADs from any port on the switch
		 */
		ft->port = MAI_TYPE_ANY;
	    }
	  ft->active |= MAI_ACT_PORT;
      }

    /*
     * We now have everything we need to use the filter match tools 
     */

    /*
     * Find the filter 
     */
    rc = mai_filter_find(act, ft, flags, &p, &q);

    MAI_HANDLE_UNLOCK(act);

    if (rc)
      {
	  /*
	   * If we didn't find it, let them know 
	   */
	  IB_EXIT(__func__, VSTATUS_NXIO);
	  return (VSTATUS_NXIO);
      }

    /*
     * Return a pointer to the filter handle 
     */
    *fh = p->hndl;

    IB_EXIT(__func__, rc);
    return (rc);

}

/*
 * FUNCTION
 *      mai_filter_hdelete
 *
 * DESCRIPTION
 *      Removes a filter from a channel.  It will remove the first instance
 *      of a matching filter from the channel.
 *
 *
 * INPUTS
 *      fd    handle returned by mai_open and used in filter create.
 *      fh    filter handle
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_ILLPARM
 *      VSTATUS_NXIO
 *
 */
Status_t
mai_filter_hdelete(IBhandle_t fd, IBhandle_t fh)
{
    struct mai_fd  *act;
    int             rc,
                    flags;
    Filter_t       *ft,
                    fbuf;
    struct mai_filter *p;

    /*
     * Standard entry conventions 
     */
    IB_ENTER(__func__, fd, fh, 0, 0);

    if (!gMAI_INITIALIZED)
      {
	  rc = VSTATUS_UNINIT;
	  IB_LOG_ERROR0("MAPI library not initialized");
	  return rc;
      }

    /*
     * Convert handle to values 
     */

    if (fh < 0 || fh >= MAI_MAX_FILTERS)
      {

	  IB_LOG_ERROR("Invalid handle fh:", fh);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    p = &gMAI_FILTERS[fh];

    act = p->owner;

    if (act == NULL)
      {

	  IB_LOG_ERROR("Invalid handle- not owned fh:",
		       fh);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    /*
     * Find out the handle on the filter 
     */
    if (fd != act->up_fd)
      {
	  IB_LOG_ERROR("Invalid upchannel fd:", fd);
	  IB_EXIT(__func__, VSTATUS_ILLPARM);
	  return (VSTATUS_ILLPARM);
      }

    memcpy(&fbuf, &(p->filter), sizeof(fbuf));

    ft = &fbuf;
    /*
     * Recall the flag stored when the filter was created 
     */
    flags = p->flags;

    rc = mai_filter_delete(fd, ft, flags);

    IB_EXIT(__func__, rc);
    return (rc);
}
