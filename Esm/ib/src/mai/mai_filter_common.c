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
 *      mai_filt_common.c                                  MAPI 0.05        *
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
 *      mai_filter_create
 *
 * DESCRIPTION
 *      Add a filter to an open channel.  This function will add both 
 *      absorbing filters and copy filters.  In the case where a new filter
 *      has been added to absorb SMDs, this function will notify all
 *      other channels that would have received that SMD.
 *
 * CALLED BY
 *
 * CALLS
 *      mai_filter_hcreate
 *
 * INPUTS
 *      fd      Channel number to add the filter to
 *      filter  Pointer to the filter to add to the channel
 *      flags   Flags to modify filter (absorb or copy)
 *
 * OUTPUTS
 *
 */
Status_t
mai_filter_create(IBhandle_t fd, Filter_t * filter, uint32_t flags)
{
    IBhandle_t      fh;
    int             rc;

    /*
     * Standard entry header 
     */
    IB_ENTER(__func__, fd, filter, flags, 0);
    rc = mai_filter_hcreate(fd, filter, flags, &fh);
    IB_EXIT(__func__, rc);
    return (rc);
}

/*
 * FUNCTION
 *      mai_filter_mclass
 *
 * DESCRIPTION
 *      Create a filter that accepts only MADs of a specified mclass.
 *
 * CALLED BY
 *
 * CALLS
 *      maif_filter_hcreate
 *
 * INPUTS
 *      devh  - device handle returned by mai_open
 *      flags - a set of characteristics to determine behavior. 
 *      ftype - filter type
 *      filterh - filter handle returned by call.
 *      mclass - the class to filter (see ib_mad.h for a list of classes).
 *
 * OUTPUTS
 *      Status return by mai_filter_hcreate.
 *
 */
Status_t
mai_filter_mclass(IBhandle_t devh, uint32_t flags, int ftype,
		  IBhandle_t * filterh, uint8_t mclass)
{
    Filter_t        f;
    int             rc;

    IB_ENTER(__func__, devh, flags, filterh, mclass);

    memset(&f, 0, sizeof(f));

    f.value.mclass = mclass;
    f.mask.mclass = MAI_FMASK_ALL;
    f.type = ftype;
    f.active = MAI_ACT_FMASK | MAI_ACT_TYPE;
    MAI_SET_FILTER_NAME((&f), "mclass");
    /*
     * Now call mai_filter create. The qp association is that 
     * of already associated with devh
     */
    rc = mai_filter_hcreate(devh, &f, flags, filterh);
    IB_EXIT(__func__, rc);
    return (rc);

}

/*
 * FUNCTION
 *      mai_filter_method
 *
 * DESCRIPTION
 *      Create a filter that accepts only MADs of a specified mclass,
 *       method.
 *
 * CALLED BY
 *
 * CALLS
 *      maif_filter_hcreate
 *
 * INPUTS
 *      devh  - device handle returned by mai_open
 *      flags - a set of characteristics to determine behavior. 
 *      ftype - filter type
 *      filterh - filter handle returned by call.
 *      mclass - the class to filter (see ib_mad.h for a list of classes).
 *      method - the method within the class.
 *
 * OUTPUTS
 *      Status return by mai_filter_hcreate.
 *
 */
Status_t
mai_filter_method(IBhandle_t devh, uint32_t flags, int ftype,
		  IBhandle_t * filterh, uint8_t mclass, uint8_t method)
{
    Filter_t        f;
    int             rc;

    IB_ENTER(__func__, devh, flags, filterh, mclass);

    memset(&f, 0, sizeof(f));
    
    f.value.mclass = mclass;
    f.value.method = method;

    f.mask.mclass = MAI_FMASK_ALL;
    f.mask.method = MAI_FMASK_ALL;

    f.type        = ftype;
    f.active      = MAI_ACT_FMASK | MAI_ACT_TYPE;

    MAI_SET_FILTER_NAME((&f), "method");

    /*
     * Now call mai_filter create. The qp association is that 
     * of already associated with devh
     */
    rc = mai_filter_hcreate(devh, &f, flags, filterh);
    IB_EXIT(__func__, rc);
    return rc;
}

/*
 * FUNCTION
 *      mai_filter_aid
 *
 * DESCRIPTION
 *      Create a filter that accepts only MADs of a specified mclass,
 *       method, aid.
 *
 * CALLED BY
 *
 * CALLS
 *      maif_filter_hcreate
 *
 * INPUTS
 *      devh  - device handle returned by mai_open
 *      flags - a set of characteristics to determine behavior. 
 *      ftype - filter type
 *      filterh - filter handle returned by call.
 *      mclass - the class to filter (see ib_mad.h for a list of classes).
 *      method - the method within the class.
 *      aid - the attribute id within the method.
 *      amod - the attribute modifier within the aid
 *
 * OUTPUTS
 *      Status return by mai_filter_hcreate.
 *
 */
Status_t
mai_filter_aid(IBhandle_t devh, uint32_t flags, int ftype,
	       IBhandle_t * filterh,
	       uint8_t mclass, uint8_t method, uint16_t aid)
{
    Filter_t        f;
    int             rc;

    IB_ENTER(__func__, flags, mclass, method, aid);

    memset(&f, 0, sizeof(f));

    f.value.mclass = mclass;
    f.value.method = method;
    f.value.aid = aid;

    f.mask.mclass = MAI_FMASK_ALL;
    f.mask.method = MAI_FMASK_ALL;
    f.mask.aid    = MAI_FMASK_ALL;

    f.type     = MAI_TYPE_ANY;
    f.active   = MAI_ACT_FMASK | MAI_ACT_TYPE;

    MAI_SET_FILTER_NAME((&f), "AID");

    /*
     * Now call mai_filter create. The qp association is that 
     * of already associated with devh
     */
    rc = mai_filter_hcreate(devh, &f, flags, filterh);
    IB_EXIT(__func__, rc);
    return rc;
}

/*
 * FUNCTION
 *      mai_filter_amod
 *
 * DESCRIPTION
 *      Create a filter that accepts only MADs of a specified mclass,
 *       method, aid, and amod.
 *
 * CALLED BY
 *
 * CALLS
 *      maif_filter_hcreate
 *
 * INPUTS
 *      devh  - device handle returned by mai_open
 *      flags - a set of characteristics to determine behavior. 
 *      ftype - filter type
 *      filterh - filter handle returned by call.
 *      mclass - the class to filter (see ib_mad.h for a list of classes).
 *      method - the method within the class.
 *      aid - the attribute id within the method.
 *      amod - the attribute modifier within the aid
 *
 * OUTPUTS
 *      Status return by mai_filter_hcreate.
 *
 */
Status_t
mai_filter_amod(IBhandle_t devh, uint32_t flags, int ftype,
		IBhandle_t * filterh,
		uint8_t mclass,
		uint8_t method, uint16_t aid, uint32_t amod)
{
    Filter_t        f;
    int             rc;

    IB_ENTER(__func__, mclass, method, aid, amod);
    memset(&f, 0, sizeof(f));

    f.value.mclass = mclass;
    f.value.method = method;
    f.value.aid = aid;
    f.value.amod = amod;

    f.mask.mclass = MAI_FMASK_ALL;
    f.mask.method = MAI_FMASK_ALL;
    f.mask.aid = MAI_FMASK_ALL;
    f.mask.amod = MAI_FMASK_ALL;
    
    f.type        = ftype;
    f.active      = MAI_ACT_FMASK | MAI_ACT_TYPE;

    MAI_SET_FILTER_NAME((&f), "AMOD");

    /*
     * Now call mai_filter create. The qp association is that 
     * of already associated with devh
     */
    rc = mai_filter_hcreate(devh, &f, flags, filterh);
    IB_EXIT(__func__, rc);
    return rc;

}

/*
 * FUNCTION
 *      mai_filter_tid
 *
 * DESCRIPTION
 *      Create a filter that accepts only MADs of a specified mclass,
 *       method, tid.
 *
 * CALLED BY
 *
 * CALLS
 *      maif_filter_hcreate
 *
 * INPUTS
 *      devh  - device handle returned by mai_open
 *      flags - a set of characteristics to determine behavior. 
 *      ftype - filter type
 *      filterh - filter handle returned by call.
 *      mclass - the class to filter (see ib_mad.h for a list of classes).
 *      tid - the attribute id within the method.
 *
 * OUTPUTS
 *      Status return by mai_filter_hcreate.
 *
 */
Status_t
mai_filter_tid(IBhandle_t devh, uint32_t flags, int ftype,
	       IBhandle_t * filterh, uint8_t mclass, uint64_t tid)
{
    Filter_t        f;
    int             rc;

    IB_ENTER(__func__, flags, mclass, tid, 0);

    memset(&f, 0, sizeof(f));

    f.value.mclass = mclass;
    f.value.tid = tid;

    f.mask.mclass = MAI_FMASK_ALL;
    f.mask.tid    = MAI_FMASK_ALL;

    f.type        = MAI_TYPE_ANY;
    f.active      = MAI_ACT_FMASK | MAI_ACT_TYPE;

    MAI_SET_FILTER_NAME((&f), "TID");

    /*
     * Now call mai_filter create. The qp association is that 
     * of already associated with devh
     */
    rc = mai_filter_hcreate(devh, &f, flags, filterh);
    IB_EXIT(__func__, rc);
    return rc;
}

/*
 * FUNCTION
 *      mai_filter_once
 *
 * DESCRIPTION
 *      This routine will create a filter which will accept messages 
 *      for a specified mclass, and tid. This routine allows a manager
 *      to issue a request and wait for the reply by filtering on just 
 *      the tid and mclass. After a message is received which passes the filter,
 *      the filter is automatically deleted.
 *
 * CALLED BY
 *
 * CALLS
 *      maif_filter_hcreate
 *
 * INPUTS
 *      devh  - device handle returned by mai_open
 *      flags - a set of characteristics to determine behavior. 
 *      ftype - filter type
 *      filterh - filter handle returned by call.
 *      mclass - the class to filter (see ib_mad.h for a list of classes).
 *      tid    - the Transaction ID to use.
 *
 * OUTPUTS
 *      Status return by mai_filter_hcreate.
 *
 */

Status_t
mai_filter_once(IBhandle_t devh, uint32_t flags, int ftype,
		IBhandle_t * filterh, uint8_t mclass, uint64_t tid)
{
    Filter_t        f;
    int             rc;

    /*
     * Standard entry stuff 
     */
    IB_ENTER(__func__, devh, mclass, tid, 0);

    memset(&f, 0, sizeof(f));

    f.value.mclass = mclass;
    f.value.tid = tid;

    f.mask.mclass = MAI_FMASK_ALL;
    f.mask.tid    = MAI_FMASK_ALL;

    f.type        = ftype;
    f.active      = MAI_ACT_FMASK | MAI_ACT_TYPE;

    MAI_SET_FILTER_NAME((&f), "once");

    /*
     * Now call mai_filter create. The qp association is that 
     * of already associated with devh
     */

    rc = mai_filter_hcreate(devh, &f, flags | VFILTER_ONCE, filterh);

    IB_EXIT(__func__, rc);
    return (rc);
}
