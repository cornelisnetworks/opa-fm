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
 *      mai_futil.c                                        MAPI 0.05        *
 *                                                                          *
 * DESCRIPTION                                                              *
 *      This file contains the utility functions used by MAI filter         *
 *      subsystem                                                           *
 *                                                                          *
 *                                                                          *
 * DATA STRUCTURES                                                          *
 *      None                                                                *
 *                                                                          *
 ****************************************************************************/

#include "mai_l.h"		/* Local mai function definitions */

/*
 * FUNCTION
 *   mai_filter_purge
 *
 * DESCRIPTION
 *   Removes any matching data from a handle that exclusively matches this 
 *   filter. 
 *
 * INPUTS
 *      act  Pointer to the MAI handle being purged.
 *      ft   the filter being removed and whose data is to be removed
 *
 * RETURNS
 *    The number of mads removed from the handle.
 */

void
mai_filter_purge(struct mai_fd *act, Filter_t * ft)
{
    Mai_t          *madp;
    struct mai_data *md,
                   *prev,
                   *del;
    struct mai_filter *q;	/* Loops over all filters on a channel */
    int             limit,
                    limit1,
                    purge,
                    rc;

    IB_ENTER(__func__, act, ft, 0, 0);

    /*
     * First determine if there are pending MADS on the handle 
     */
    if (act->mad_hqueue == NULL)
      {
	  /*
	   * Nothing to do ... just return 
	   */
	  IB_EXIT(__func__, 0);
	  return;
      }

    /*
     * Next check to determine if there is are
     * any MADs on this handle that the filter passes.
     */

    limit1 = 0;
    prev = NULL;
    md = act->mad_hqueue;
    act->mad_hqueue = NULL;	/* empty the list */
    act->mad_tqueue = NULL;	/* and the tail too */

    purge = 0;

    for (; md;)
      {

	  if (limit1++ > act->mad_cnt)
	    {
		IB_LOG_ERROR("mads list corrupt limit1:", limit1);
		mai_shut_down();
		IB_EXIT(__func__, VSTATUS_BAD);
		return;
	    }

	  rc = maif_match(&md->mad, ft);

	  if (!rc)
	    {
	      next_mad:
		/*
		 * No match here .. just move to next one 
		 */
		if (prev)
		  {
		      prev->next = md;

		  }
		else
		  {
		      act->mad_hqueue = prev = md;
		  }

		act->mad_tqueue = md;
		prev = md;	/* Remember this as last one queued */
		md = md->next;	/* Move to the next one */
		prev->next = NULL;	/* Make sure we terminate the list 
					 */

		continue;
	    }

	  /*
	   * We have a data match. Check to see if there are other filters 
	   * that accept this data as well 
	   */

	  limit = 0;

	  for (q = act->sfilters; q; q = q->next)
	    {
		if (limit++ > act->sfilt_cnt)
		  {
		      IB_LOG_ERROR("Filter list corrupt limit:",
				   limit);
		      mai_shut_down();
		      return;
		  }

		/*
		 * See if this MAD matches this filter 
		 */
		rc = maif_match(&md->mad, &q->filter);

		if (rc)
		  {

		      /*
		       * This filter matches the data so we can't purge this data.
		       * So skip this data element and continue.
		       */

		      goto next_mad;
		  }

	    }

	  /*
	   * When we get here we are sure that no other filter associated 
	   * with this handle will pass this MAD so we can proceed to
	   * purge it from the handle.
	   */

	  del = md;
	  /*
	   * Move to next MAD 
	   */
	  md = md->next;
	  madp = &del->mad;

	  MSTATS_FD_MADDECR(act);
	  MSTATS_FD_RX(act, madp->type);

#if 0
	  if (IB_LOG_IS_INTERESTED(VS_LOG_INFO))
	  {
	      IB_LOG_INFO("deleted MAD ", madp->type);
	      dump_mad(madp);
	      dump_filter(ft);
	  }
#endif

	  /*
	   * Now store the free on back on the free list 
	   */

	  mai_free_mbuff(del);
	  purge++;
      }

    /*
     * We are done.. 
     */

    /*
     * Update the count of MADs on the list 
     */
    act->mad_cnt -= purge;

    MAI_ASSERT_TRUE((act->mad_cnt >= 0) &&
	       ((act->mad_cnt != 0 &&
		 act->mad_hqueue != NULL &&
		 act->mad_tqueue != NULL) ||
		(act->mad_cnt == 0 &&
		 act->mad_hqueue == NULL && act->mad_tqueue == NULL)));

    IB_EXIT(__func__, purge);
    return;
}

/*
 * FUNCTION
 *  mai_alloc_filter
 *
 * DESCRIPTION
 *   Allocates a filter from the free list.
 *
 * INPUTS
 *  
 *
 * RETURNS
 *   NULL  No filters available
 *  !NULL  Pointer to an available filter
 *
 */
struct mai_filter *
mai_alloc_filter(void)
{

    struct mai_filter *filt;

    IB_ENTER(__func__, 0, 0, 0, 0);

    MAI_FILT_LOCK();

    /*
     * Get a free one 
     */
    filt = gMAI_FILT_FREE;
    if (filt == NULL)
      {
	  MAI_FILT_UNLOCK();

	  IB_LOG_ERROR0("No free filters");
	  IB_EXIT(__func__, 0);

	  MSTATS_NORESOURCE_INCR();

	  return (NULL);
      }

    gMAI_FILT_FREE = filt->next;

    MSTATS_FILT_USE();

    gMAI_FILT_CNT--;

    MAI_ASSERT_TRUE((gMAI_FILT_CNT >= 0));

    if (gMAI_FILT_CNT <= MAI_FILTER_LOWWM)
      {
	  IB_LOG_WARN("Running low on free filters count:",
		      gMAI_FILT_CNT);
      }

    MAI_FILT_UNLOCK();

    MAI_ASSERT_TRUE((filt->state == MAI_FREE));

    filt->state = MAI_BUSY;
    filt->next = NULL;
    filt->prev = NULL;
    filt->owner = NULL;

    MSTATS_FMATCH_CLR(filt);

    IB_EXIT(__func__, filt);
    return (filt);
}

/*
 * FUNCTION
 *  mai_free_filter
 *
 * DESCRIPTION
 *   Places a filter back on the free list.
 *
 * INPUTS
 *   filt   Pointer the filter being freed.
 *
 * RETURNS
 *
 */

void
mai_free_filter(struct mai_filter *filt)
{

    IB_ENTER(__func__, filt, 0, 0, 0);

    /*
     * Clear the new structure and fill in the data they passed 
     */

    MAI_ASSERT_TRUE((filt->next == NULL));
    MAI_ASSERT_TRUE((filt->prev == NULL));
    MAI_ASSERT_TRUE((filt->owner == NULL));

    filt->state = MAI_FREE;
    filt->once = 0;

    MAI_FILT_LOCK();

    MSTATS_FILT_FREE();

    /*
     * Count the number of filters 
     */

    gMAI_FILT_CNT++;

    filt->next = gMAI_FILT_FREE;
    gMAI_FILT_FREE = filt;

    MAI_FILT_UNLOCK();

    IB_EXIT(__func__, 0);
    return;
}

/*
 * FUNCTION
 *   mai_load_filter
 *
 * DESCRIPTION
 *   Given a Filter_t, allocate one of our internal filters and fill
 *   it in with the data.  Translate flags as required.
 *
 * INPUTS
 *    in      pointer to the  filter being copied
 *    pout    pointer to location to return pointer
 *            of loaded filter buffer.
 *
 * RETURNS
 *  VSTATUS_OK      - success
 *  VSTATUS_NOMEM   - out of filters
 *  VSTATUS_ILLPARM - bad parameters on filter
 *
 */
Status_t
mai_load_filter(Filter_t * in, struct mai_filter ** pout)
{
    struct mai_filter *out;
    int             tmp,
                    rc = VSTATUS_OK;

    IB_ENTER(__func__, in, rc, 0, 0);

    /*
     * Get a new filter structure 
     */
    out = mai_alloc_filter();

    if (out == NULL)
      {
	  rc = VSTATUS_NOMEM;
	  IB_EXIT(__func__, rc);
	  return (rc);
      }

    /*
     * Store flags 
     */
    out->flags = in->flags;

    tmp = mai_validate_filter(in);

    if (tmp == 0)
      {
	  mai_free_filter(out);
	  rc = VSTATUS_ILLPARM;
	  IB_EXIT(__func__, rc);
	  return (rc);
      }

    /*
     * Copy in the actual filter contents 
     */
    memcpy((void *) &out->filter, in, sizeof(Filter_t));
    *pout = out;
    IB_EXIT(__func__, rc);
    return (rc);
}

/*
 * FUNCTION
 *    mai_filter_dequeue
 *
 * DESCRIPTION
 *    Remove a filter buffer from a handle. If the filter is an 
 *    exclusive filter also remove it from the exlusive filter queue. 
 *
 * INPUTS
 *    chanp    the handle from which the filter is being removed
 *    filt     pointed to the filter buffer being removed 
 * RETURN
 *     filt    the pointer to filter buffer removed
 *
 * SPECIAL:
 *  This function should be called holding the lock to handle
 */

struct mai_filter *
mai_filter_dequeue(struct mai_fd *chanp, struct mai_filter *filt)
{

    IB_ENTER(__func__, chanp, filt, 0, 0);

    MAI_ASSERT_TRUE((chanp == filt->owner));
    MAI_ASSERT_LOCK_HELD((chanp));

	/*
	 * The filter is a shared filter so remove it from the list 
	 */
	if (chanp->sfilters == filt)
	  {
		/*
		 * It is at the head of the list 
		 */
		chanp->sfilters = filt->next;

		/*
		 * make sure the prev of the head points to 
		 * nowhere now.
		 */
		if (chanp->sfilters)
		  {
		      chanp->sfilters->prev = NULL;
		  }
	    }
	  else
	    {
		struct mai_filter *tmp;
		/*
		 * This is not the head of filters on the handle so
		 * remove it from wherever it is.
		 */

		MAI_ASSERT_TRUE((filt->prev != NULL));

		tmp = filt->next;
		filt->prev->next = tmp;

		/*
		 * tmp could be NULL if this is the tail of the list 
		 */

		if (tmp)
		  {
		      tmp->prev = filt->prev;
		  }
	    }

	  /*
	   * Decrement the count of filters on the handle 
	   */
	  chanp->sfilt_cnt--;

    /*
     * The filter is now floating so set the paramters accordingly 
     */
    filt->next = NULL;
    filt->prev = NULL;
    filt->owner = NULL;

    MSTATS_FD_FILTREM(chanp);

    IB_EXIT(__func__, filt);
    return filt;

}

/*
 * FUNCTION
 *     mai_filter_queue
 *
 * DESCRIPTION
 *    Adds a filter buffer to  a handle. If the filter in the buffer 
 *    is an exclusive filter the buffer is  also added to the head of
 *    the exlusive filter buffer queue.
 *
 * INPUTS
 *    chanp    the handle to which the filter is being added
 *    filt     pointed to the filter buffer being queued 
 *
 *  RETURN
 *
 * SPECIAL:
 *  This function should be called holding the lock to handle
 */

void
mai_filter_enqueue(struct mai_fd *chanp, struct mai_filter *filt)
{
    IB_ENTER(__func__, chanp, filt, 0, 0);

    MAI_ASSERT_LOCK_HELD((chanp));

	/*
	 * Tack it to the head of the shared filter list 
	 */

	filt->next = chanp->sfilters;
	if (chanp->sfilters)
	  {
		chanp->sfilters->prev = filt;
	  }

	filt->prev = NULL;
	chanp->sfilters = filt;

	/*
	 * Count the number of filters on the handle 
	 */
	chanp->sfilt_cnt++;

    filt->owner = chanp;

    MSTATS_FD_FILTADD(chanp);

    IB_EXIT(__func__, 0);
    return;
}

/*
 * FUNCTION
 *   mai_filter_find
 *
 * DESCRIPTION
 *   This function finds the filter buffer on a  handle that contains a 
 *   filter matching the reference filter passed. A pointer to filter
 *   buffer is returned if one is found.
 *
 * INPUTS
 *   act    The channel pointer
 *   ref    Points to the reference filter to compare.
 *   flag   The filter flags
 *   match  Location to return the matching filter
 *   prev   The filter before the matching one.
 *
 * 
 * RETURNS
 *   VSTATUS_OK - when match is found
 *   VSTATUS_NODEV - no matching filter found. 
 *   VSTATUS_BAD - error.
 *
 * SPECIAL CONDITIONS
 *   1. This code MUST be called holding the lock of the handle
 */
Status_t
mai_filter_find(struct mai_fd * act,
		Filter_t * ref,
		int flag,
		struct mai_filter ** match, struct mai_filter ** prev)
{
    int             rc,
                    limit;
    struct mai_filter *q,
                   *p = NULL;

    IB_ENTER(__func__, act, ref, flag, match);

    /*
     * Scan the handle filter lists for the channel looking for a match
     */
    limit = 0;
    for (q = act->sfilters; q; q = q->next)
      {
	  if (limit++ > act->sfilt_cnt)
	    {
		IB_LOG_ERROR("Filter list corrupt limit:",
			     limit);
		mai_shut_down();
		rc = VSTATUS_BAD;
		IB_EXIT(__func__, rc);
		return rc;
	    }

	  /*
	   * Make sure that flags are completely equal 
	   */
	  if (flag != q->flags)
	    {
		/*
		 * Keep track of the last one 
		 */
		p = q;
		continue;
	    }

	  rc = maif_reduce(ref, &q->filter);

	  if (rc != MAI_FILTER_EQUAL)
	    {
		/*
		 * Keep track of the last one 
		 */
		p = q;
		continue;	/* No relationship */
	    }

	  /*
	   * We have a functional match. Now see if th name flag is set and 
	   * make sure the names match if the name is indicated as
	   * between the two.
	   */

	  if (ref->active & MAI_ACT_FNAME)
	    {
		Filter_t       *f = &q->filter;

		if (f->active & MAI_ACT_FNAME)
		  {
		      if (memcmp(f->fname, ref->fname, sizeof(f->fname)))
			{
			    IB_LOG_INFO
				("Shfilter EQUIV with name mismatch ",
				 q);
			    /*
			     * Keep track of the last one 
			     */
			    p = q;
			    continue;
			}
		  }
	    }
	  else
	    {
		/*
		 * If his fname field is set then move on..  
		 */
		Filter_t       *f = &q->filter;

		if (f->active & MAI_ACT_FNAME)
		  {
		      IB_LOG_INFO
			  ("Shfilter EQUIV with name mismatch ",
			   q);
		      /*
		       * Keep track of the last one 
		       */
		      p = q;
		      continue;
		  }
	    }

	  /*
	   * When we get here we know we have completely matching filters 
	   */

	  *match = q;
	  *prev = p;
	  rc = VSTATUS_OK;
	  /*
	   * Done - Document and out 
	   */
	  IB_EXIT(__func__, rc);
	  return rc;
      }

    /*
     * When we get here we have found not matching exclusive filter.
     * Done - Document and out                                             
     */

    rc = VSTATUS_NODEV;
    IB_EXIT(__func__, rc);
    return rc;

}
