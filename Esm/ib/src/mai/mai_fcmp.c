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
 *      mai_fcmp.c                                         MAPI 0.05        *
 *                                                                          *
 * DESCRIPTION                                                              *
 *      This file contains all the routines that understand the semantic    *
 *      content of filters.  These are only used by other management API    *
 *      functions and are not exported.                                     *
 *                                                                          *
 *                                                                          *
 * DATA STRUCTURES                                                          *
 *      None                                                                *
 *                                                                          *
 * FUNCTIONS                                                                *
 *      maif_init                       Filter management initialization    *
 *      maif_match                      Compare MAD and filter; signal match*
 *      maif_reduce                     Compare 2 filters and signal match  *
 *      maif_special                    Extracts filter from 'special' MAD  *
 *      maif_make_special               Create 'special' MAD from filter    *
 *                                                                          *
 * DEPENDENCIES                                                             *
 *      ib_types.h                      Include file                        *
 *                                                                          *
 *                                                                          *
 * HISTORY                                                                  *
 *          NAME      DATE  REMARKS                                         *
 *      Jim Mott  01/15/01  Entry from prototype code                       *
 *           jsy  02/14/01  Fix compiler warnings                           *
 *                                                                          *
 * OUTSTANDING ISSUES                                                       *
 *   1. Must fill in the 'real' filter logic here                           *
 *                                                                          *
 ****************************************************************************/

#include "mai_l.h"		/* Local mai function definitions */

#if 0
/*
 * filter_test
 *   This utility routine will compare the mask/value pair from one filter
 * with the same stuff on another filter.  At a high level the objective 
 * is to see if the new filter (p1/p2) just added as a consuming filter
 * can 'consume' any (or all) of the MADs that somebody with a different
 * filter registered could get.
 *
 *   The only place where you might not eat somebody elses MADs is where
 * the masks are equal, and the values are different.
 *
 * INPUTS
 *     size   The number of bytes to compare
 *     p1     Filter 1 value
 *     p2     Filter 1 mask
 *     p3     Filter 2 value
 *     p4     Filter 2 mask
 *
 * It returns:
 *    0 value1/mask1 is not a match with value2/mask2
 *    1 They do match
 *
 * TODO:
 *   1. Implement logic
 */
static int
filter_test(int size, void *p1, void *p2, void *p3, void *p4)
{
    uint8_t         p1_8,
                    p2_8,
                    p3_8,
                    p4_8;
    uint16_t        p1_16,
                    p2_16,
                    p3_16,
                    p4_16;
    uint32_t        p1_32,
                    p2_32,
                    p3_32,
                    p4_32;
    uint32_t       *p1p,
                   *p2p,
                   *p3p,
                   *p4p;
    int             rc;

    IB_ENTER(__func__, size, p1, p2, p3);

    rc = 1;			/* Assume a match is possible */
    switch (size)
      {
      case 1:
	  p1_8 = *(uint8_t *) p1;
	  p2_8 = *(uint8_t *) p2;
	  p3_8 = *(uint8_t *) p3;
	  p4_8 = *(uint8_t *) p4;

	  p2_8 = (p2_8 & p4_8);	/* See if any bits in common */
	  if (p2_8 == 0)	/* If no mask bits in common */
	      break;		/* they might overlap */
	  p1_8 = p1_8 & p2_8;	/* Calculate value under mask */
	  p3_8 = p3_8 & p2_8;	/* and value2 under mask */
	  if (p1_8 == p3_8)	/* If masked section identical */
	      break;		/* they might overlap */
	  rc = 0;
	  break;

      case 2:
	  p1_16 = *(uint16_t *) p1;
	  p2_16 = *(uint16_t *) p2;
	  p3_16 = *(uint16_t *) p3;
	  p4_16 = *(uint16_t *) p4;

	  p2_16 = p2_16 & p4_16;
	  if (p2_16 == 0)
	      break;
	  p1_16 = p1_16 & p2_16;
	  p3_16 = p3_16 & p2_16;
	  if (p1_16 == p3_16)
	      break;
	  rc = 0;
	  break;

      case 4:
	  p1_32 = *(uint32_t *) p1;
	  p2_32 = *(uint32_t *) p2;
	  p3_32 = *(uint32_t *) p3;
	  p4_32 = *(uint32_t *) p4;

	  p2_32 = p2_32 & p4_32;
	  if (p2_32 == 0)
	      break;
	  p1_32 = p1_32 & p2_32;
	  p3_32 = p3_32 & p2_32;
	  if (p1_32 == p3_32)
	      break;
	  rc = 0;
	  break;

      case 8:
	  p1p = (uint32_t *) p1;
	  p2p = (uint32_t *) p2;
	  p3p = (uint32_t *) p3;
	  p4p = (uint32_t *) p4;

	  p2p[0] = p2p[0] & p4p[0];
	  p2p[1] = p2p[1] & p4p[1];
	  if ((p2p[0] == 0) && (p2p[1] == 0))
	      break;
	  p1p[0] = p1p[0] & p2p[0];
	  p1p[1] = p1p[1] & p2p[1];
	  p3p[0] = p3p[0] & p2p[0];
	  p3p[1] = p3p[1] & p2p[1];
	  if ((p1p[0] == p3p[0]) && (p1p[1] == p3p[1]))
	      break;
	  rc = 0;
	  break;

      default:
	  IB_FATAL_ERROR("filter_test: Invalid size parameter");
	  /*
	   * NEVER RETURNS 
	   */
      }

    /*
     * Done with the easy stuff 
     */
    IB_EXIT(__func__, rc);
    return (rc);
}
#define FILT_TEST(p1) filter_test(sizeof(filt1->value.p1),    \
                                  (void *)&filt1->value.p1,   \
                                  (void *)&filt1->mask.p1,    \
                                  (void *)&filt2->value.p1,   \
                                  (void *)&filt2->mask.p1)

/*
 * mad_test
 *   This utility routine will compare the mask value passed with the
 * compare value passed against the target value passed and return a
 * value indicating what happened.  The logic for this is:
 *
 * if p1 is 0:
 *   RC = (p2 == 0) ? 0 : 1
 *
 * if p1 is non-0:
 *   RC = (((p1 & p2) ^ p3) == 0) ? 1 : 0
 *
 * Where p1 is the mask, p2 is the compare value, and p3 is the target
 * to compare with. 
 *
 * In English:
 *   If you set the mask field to 0, then the value field is used to
 * decide if you want to ignore this field for purposes of a filter
 * match (value=0), or if you want to match any value in this field
 * (value != 0).
 *   If the mask field is non-zero, then we and the mask with the target
 * to compare with, and compare the result with the value passed.
 *
 * It returns:
 *    0 No match
 *    1 Match
 *
 */
static int
mad_test(int size, void *p1, void *p2, void *p3)
{
    uint8_t         p1_8,
                    p2_8,
                    p3_8;
    uint16_t        p1_16,
                    p2_16,
                    p3_16;
    uint32_t        p1_32,
                    p2_32,
                    p3_32;
    uint32_t       *p1p,
                   *p2p,
                   *p3p;
    int             rc;

    IB_ENTER(__func__, size, p1, p2, p3);

    /*
     * Cast everybody and get ready to go 
     */
    rc = 0;			/* Start assuming no match */
    switch (size)
      {
      case 1:
	  p1_8 = *(uint8_t *) p1;
	  p2_8 = *(uint8_t *) p2;
	  p3_8 = *(uint8_t *) p3;
	  if (!p1_8)
	    {
		if (p2_8 != 0)
		    rc = 1;
		break;
	    }
	  if ((p1_8 & p3_8) == p2_8)
	      rc = 1;
	  break;

      case 2:
	  p1_16 = *(uint16_t *) p1;
	  p2_16 = *(uint16_t *) p2;
	  p3_16 = *(uint16_t *) p3;
	  if (!p1_16)
	    {
		if (p2_16 != 0)
		    rc = 1;
		break;
	    }
	  if ((p1_16 & p3_16) == p2_16)
	      rc = 1;
	  break;

      case 4:
	  p1_32 = *(uint32_t *) p1;
	  p2_32 = *(uint32_t *) p2;
	  p3_32 = *(uint32_t *) p3;
	  if (!p1_32)
	    {
		if (p2_32 != 0)
		    rc = 1;
		break;
	    }
	  if ((p1_32 & p3_32) == p2_32)
	      rc = 1;
	  break;

      case 8:
	  p1p = (uint32_t *) p1;
	  p2p = (uint32_t *) p2;
	  p3p = (uint32_t *) p3;
	  if ((p1p[0] == 0) && (p1p[1] == 0))
	    {
		if ((p2p[0] != 0) || (p2p[1] != 0))
		    rc = 1;
		break;
	    }
	  p1_32 = p1p[0] & p3p[0];
	  p2_32 = p1p[1] & p3p[1];
	  if ((p1_32 == p2p[0]) && (p2_32 == p2p[1]))
	      rc = 1;
	  break;

      default:
	  IB_FATAL_ERROR("mad_test: Invalid size parameter");
	  /*
	   * NEVER RETURNS 
	   */
      }

    /*
     * Done with the easy stuff 
     */
    IB_EXIT(__func__, rc);
    return (rc);
}
#define MAD_TEST(p1) mad_test(sizeof(mad2->p1),  \
                              (void *)&mad2->p1, \
                              (void *)&mad1->p1, \
                              (void *)&mad3->p1)

#endif

/*
 * Functions used to dump the contents of filters.
 */

#if defined(VIEO_DEBUG) || defined(DEBUG)
static char     pbuff[32];

static char    *
ptype(int p)
{
    if (p == MAI_FILTER_ANY)
      {
	  sprintf(pbuff, "ANY");
      }
    else
      {
	  sprintf(pbuff, "%d", p);
      }
    return pbuff;
}

void
dump_maibase_info(Mai_t *info)
{
    char            data[4096];
    char           *ptr;


	if (! IB_LOG_IS_INTERESTED(VS_LOG_INFO))
		return;

    ptr = data;

    ptr +=
		sprintf(ptr,
			"\n\t\t\t ********* MAI BASE INFO ****************\n\n");
    ptr +=
		sprintf(ptr,
			"\t\t\t AID:       0x%04x\n",info->base.aid);
    ptr +=
		sprintf(ptr,
			"\t\t\t AMOD:      0x%04x\n",(unsigned)info->base.amod);
    ptr +=
		sprintf(ptr,
			"\t\t\t BVERSION:  0x%02x\n",info->base.bversion);
    ptr +=
		sprintf(ptr,
			"\t\t\t CVERSION:  0x%02x\n",info->base.cversion);
    ptr +=
		sprintf(ptr,
			"\t\t\t HOP COUNT: 0x%02x\n",info->base.hopCount);
    ptr +=
		sprintf(ptr,
			"\t\t\t HOP PTR:   0x%02x\n",info->base.hopPointer);
    ptr +=
		sprintf(ptr,
			"\t\t\t MCLASS:    %d\n",info->base.mclass);
    ptr +=
		sprintf(ptr,
			"\t\t\t METHOD:    0x%02x\n",info->base.method);
    ptr +=
		sprintf(ptr,
			"\t\t\t STATUS:    0x%04x\n",info->base.status);
    ptr +=
		sprintf(ptr,
			"\t\t\t TID:       0x%"CS64"x\n",info->base.tid);

    IB_LOG_INFO0(Log_StrDup(data));

}

void
dump_maifilter_info(int i, Filter_t * ft)
{
    char            buff[MAI_FNAME_LEN];
    char            data[4096];
    char           *ptr;

	if (! IB_LOG_IS_INTERESTED(VS_LOG_INFO))
		return;

    ptr = data;

    ptr +=
	sprintf(ptr,
		"\n\t\t\t %d:  ********** FILTER INFO *************\n\n",
		i);
    ptr +=
	sprintf(ptr, "\t\t\t  Type               :  Normal\n");

    if (ft->active & MAI_ACT_FNAME)
      {
	  int             i;
	  for (i = 0; i < MAI_FNAME_LEN - 1; i++)
	      buff[i] = ft->fname[i];
	  buff[i] = 0;
	  ptr += sprintf(ptr, "\t\t\t  Name               :  %s\n", buff);
      }

    ptr +=
	sprintf(ptr, "\t\t\t  Active flag        :  0x%x\n", (unsigned)ft->active);

    if (ft->active & MAI_ACT_TYPE)
	ptr +=
	    sprintf(ptr, "\t\t\t  Ftype              :  %s\n",
		    ptype(ft->type));

    if (ft->active & MAI_ACT_DEV)
	ptr +=
	    sprintf(ptr, "\t\t\t  Device             :  %s\n",
		    ptype(ft->dev));

    if (ft->active & MAI_ACT_PORT)
	ptr +=
	    sprintf(ptr, "\t\t\t  Port               :  %s\n",
		    ptype(ft->port));

    if (ft->active & MAI_ACT_QP)
	ptr +=
	    sprintf(ptr, "\t\t\t  QP                 :  %s\n",
		    ptype(ft->qp));

    if (ft->active & MAI_ACT_FMASK)
      {
	  Mad_t          *md;
	  int             i;

	  for (i = 0; i < 2; i++)
	    {
		char           *set;
		// ptr += sprintf(ptr,"\n");
		if (i == 0)
		  {
		      md = &ft->value;
		      ptr +=
			  sprintf(ptr,
				  "\t\t\t _________ Value MAD _______\n");
		      set = ptr;
		  }
		else
		  {
		      md = &ft->mask;
		      ptr +=
			  sprintf(ptr,
				  "\t\t\t _________ MASK MAD  ________\n");
		      set = ptr;
		  }

		if (md->bversion)
		    ptr +=
			sprintf(ptr,
				"\t\t\t  Version            :  0x%02x\n",
				md->bversion);
		if (md->mclass)
		    ptr +=
			sprintf(ptr,
				"\t\t\t  Mclass             :  0x%02x\n",
				md->mclass);
		if (md->method)
		    ptr +=
			sprintf(ptr,
				"\t\t\t  Method             :  0x%02x\n",
				md->method);
		if (md->status)
		    ptr +=
			sprintf(ptr,
				"\t\t\t  Status             :  0x%04x\n",
				md->status);
		if (md->hopPointer)
		    ptr +=
			sprintf(ptr,
				"\t\t\t  HopPointer         :  0x%02x\n",
				md->hopPointer);
		if (md->hopCount)
		    ptr +=
			sprintf(ptr,
				"\t\t\t  HopCount           :  0x%02x\n",
				md->hopCount);
		if (md->tid)
		    ptr +=
			sprintf(ptr,
				"\t\t\t  TID                :  0x%"CS64"x\n",
				md->tid);
		if (md->aid)
		    ptr +=
			sprintf(ptr,
				"\t\t\t  AID                :  0x%04x\n",
				md->aid);
		if (md->rsvd3)
		    ptr +=
			sprintf(ptr,
				"\t\t\t  Revd3              :  0x%04x\n",
				md->rsvd3);
		if (md->rsvd3)
		    ptr +=
			sprintf(ptr,
				"\t\t\t  AMOD               :  0x%08x\n",
				(unsigned)md->amod);

		if (set == ptr)
		  {
		      ptr +=
			  sprintf(ptr,
				  "\t\t\t  ALL VALUES          :  0x%08x\n",
				  0);
		  }
	    }
      }

    IB_LOG_INFO0(Log_StrDup(data));
}

#define DUMP_FILTER(ft,i) dump_maifilter_info(i,ft)
#define DUMP_MAI_BASEINFO(info) dump_maibase_info(info)
#else
#define DUMP_FILTER(ft,i)
#define DUMP_MAI_BASEINFO(info)
#endif

/*
 * mai_validate_filter
 *   This routine will look at a Fiter_t and decide if it is valid or
 * just junk.  It returns:
 *    0 if the thing is not valid
 *    1 if it is valid
 */
int
mai_validate_filter(Filter_t * filter)
{
    int             rc;
    uint32_t        mask;

    IB_ENTER(__func__, filter, 0, 0, 0);

    /*
     * Assume this filter is valid 
     */
    rc = 1;			/* For now, it must be valid */

    /*
     * Validate the type field if they say it is active 
     */
    mask = MAI_ACT_TYPE | MAI_ACT_DEV | MAI_ACT_PORT |
	MAI_ACT_QP | MAI_ACT_FMASK;

    /*
     * mask ^= 0xFFFFFFFF; Turn off the only valid bits 
     */

    if ((mask & filter->active) == 0)
      {
	  IB_LOG_ERRORX("Invalid mask:", filter->active);
	  IB_EXIT(__func__, 0);
	  return (0);
      }

    /*
     * If they are looking for a specific type, make sure it is valid 
     */
    if (filter->active & MAI_ACT_TYPE)
      {
	  switch (filter->type)
	    {
	    case MAI_TYPE_EXTERNAL:
	    case MAI_TYPE_INTERNAL:
	    case MAI_TYPE_ERROR:
		break;		/* Anybody can use these filters */

	    case MAI_TYPE_ANY:
		break;

	    default:
		IB_LOG_ERROR("Invalid type:", filter->type);
		IB_EXIT(__func__, 0);
		return (0);
	    }
      }
    /*
     * If they are looking at a specific QP, make sure it is valid 
     */
    if (filter->active & MAI_ACT_QP)
      {
	  switch (filter->qp)
	    {
	    case MAI_FILTER_ANY:
		break;

	    case 0:		/* SMI */
		break;

	    case 1:		/* GSI */
		break;

	    default:
		IB_LOG_ERROR("Invalid QP:", filter->qp);
		IB_EXIT(__func__, 0);
		return (0);
	    }
      }
    /*
     * If they are looking at a specific port, make sure it is valid 
     */
    if (filter->active & MAI_ACT_PORT)
      {
	  switch (filter->port)
	    {
	    case MAI_FILTER_ANY:
		break;

	    default:
		{
		    if (filter->port < 0 || filter->port > MAI_MAX_PORT)
		      {

			  IB_LOG_ERROR("Invalid PORT:", filter->port);
			  IB_EXIT(__func__, 0);
			  return (0);
		      }
		}
	    }
      }
    /*
     * If they are looking at a specific device, make sure it is valid 
     */
    if (filter->active & MAI_ACT_DEV)
      {
	  switch (filter->dev)
	    {
	    case MAI_FILTER_ANY:
		break;

	    default:
		{
		    if (filter->dev < 0 || filter->dev > MAI_MAX_DEV)
		      {

			  IB_LOG_ERROR ("Invalid device:", filter->dev);
			  IB_EXIT(__func__, 0);
			  return (0);
		      }
		}
	    }
      }
    IB_EXIT(__func__, rc);
    return (rc);
}

/*
 * maif_init
 *   Called to initialize internal static data areas.
 */
void
maif_init(void)
{
    IB_ENTER(__func__, 0, 0, 0, 0);

    IB_EXIT(__func__, 0);
}

/*
 * mai_maskequate_filter
 * The logic below implements a matching logic that correspond to
 * equating the masked values of incoming mad and the filter value mad.
 * As the filter spec is refined we will add other options.
 * returns:
 *   0  MAD is not described by the filter
 *   1  MAD is described by the filter
 */
int
maif_maskequate_filter(void *maddp, Filter_t * filter)
{
    unsigned int   *dp = (unsigned int *) maddp;	// pointer to mad
    // data to check 
    unsigned int   *mp,
                   *fp;
    int             len;
    int             rc;

    IB_ENTER(__func__, maddp, filter, 0, 0);

    fp = (unsigned int *) &filter->value;
    mp = (unsigned int *) &filter->mask;
    len = sizeof(filter->value);

    for (; len > 0; len -= sizeof(int))
      {
	  if ((*fp++ ^ *dp++) & *mp++)
	    {
		/*
		 * failed this filter 
		 */
		IB_EXIT(__func__, 0);
		return 0;
	    }
      }

    if (len < 0)
      {
	  // (handle filters of arbitrary length, just in case)
	  uint8_t        *cmp = (uint8_t *) mp;
	  uint8_t        *cfp = (uint8_t *) fp;
	  uint8_t        *cdp = (uint8_t *) dp;

	  len += sizeof(int);
	  for (; len > 0; len--)
	    {
		if ((*cfp++ ^ *cdp++) & *cmp++)
		  {
		      /*
		       * failed this filter 
		       */
		      IB_EXIT(__func__, 0);
		      return 0;
		  }
	    }
      }

    rc = (len == 0);
    IB_EXIT(__func__, rc);
    return rc;

}

/*
 * maif_match
 *   This function is called with a pointer to a MAD as received from a
 * down stream process and a filter to check against that MAD.  It 
 * returns:
 *   0  MAD is not described by the filter
 *   1  MAD is described by the filter
 */
int
maif_match(Mai_t * data, Filter_t * filter)
{
    int             rc,
                    rc2;
    int             diff;

    IB_ENTER(__func__, data, filter, 0, 0);

    /*
     * Assume they are not a matching MAD 
     */
    rc = 0;
    rc2 = 0;
    diff = 0;

#if 0				// PAW
    {
	Mad_t          *mad1,
	               *mad2,
	               *mad3;
	/*
	 * Now start checking specific fields 
	 */
	mad1 = &filter->value;
	mad2 = &filter->mask;
	mad3 = &data->base;

	rc += MAD_TEST(bversion);
	rc += MAD_TEST(mclass);
	rc += MAD_TEST(cversion);
	rc += MAD_TEST(method);
	rc += MAD_TEST(status);
	rc += MAD_TEST(hopPointer);
	rc += MAD_TEST(hopCount);
	rc += MAD_TEST(tid);
	rc += MAD_TEST(aid);
	rc += MAD_TEST(amod);
    }
#else

     DUMP_FILTER(filter,0) ;
	 DUMP_MAI_BASEINFO(data);
    

    IB_LOG_VERBOSE_FMT(__func__, "filter dev: %d port: %d qp: %d : act",
		     filter->dev,filter->port,(int)filter->qp);
    
    IB_LOG_VERBOSE("filter value & mask",filter);     
    //IB_LOG_DATA("fliler value",&filter->value,sizeof(filter->value));    
    //IB_LOG_DATA("fliler mask",&filter->mask,sizeof(filter->mask));

    if (filter->active & MAI_ACT_FMASK)
      {
	  rc = maif_maskequate_filter((void *) (&data->base), filter);
	  if (rc == 0)
	    {
		/*
		 * match failed .. we are done 
		 */
		IB_EXIT(__func__, rc);
		return (rc);
	    }
      }
#endif

    rc2 = 1;

    /*
     * Do they want to match on the type field (EXTERNAL, INTERNAL) 
     */
	/*
	 * We treat MAI_TYPE_ERROR specially so that unsuspecting clients
	 * don't get this special type of MAD.  Later we could enhance the
	 * clients to properly set up their filters to avoid this MAD unless
	 * truely desired
	 */
    if (data->type == MAI_TYPE_ERROR)
      {
      if ((filter->active & MAI_ACT_TYPE)
          && (filter->type == MAI_TYPE_ERROR) )
	    rc2++;
	  else
        {
        diff++;
            IB_LOG_VERBOSE("FType differs ",filter->type);
	    }
	  }
	else if (filter->active & MAI_ACT_TYPE)
      {
	  if ((filter->type == MAI_FILTER_ANY)
	      || (filter->type == data->type))
	      rc2++;
	  else
	    {
	      diff++;
              IB_LOG_VERBOSE("FType differs ",filter->type);
	    }
      }

    /*
     * See if they care which device the data comes from 
     */
    if (!diff && filter->active & MAI_ACT_DEV)
      {
	  if ((filter->dev == MAI_FILTER_ANY)
	      || (filter->dev == data->dev))
	      rc2++;
	  else
	    {
	      diff++;
	      IB_LOG_VERBOSE("MAD Dev differs ",data->dev);
	    }
      }

    /*
     * See if they care which port it comes from 
     */
    if (!diff && filter->active & MAI_ACT_PORT)
      {
	  if ((filter->port == MAI_FILTER_ANY)
	      || (filter->port == data->port))
	      rc2++;
	  else
	    {
	      diff++;
	      IB_LOG_VERBOSE("MAD Port differs ",data->port);
	    }
      }

    /*
     * See if they care which QP it comes from 
     */
    if (!diff && filter->active & MAI_ACT_QP)
      {
	  if ((filter->qp == MAI_FILTER_ANY) || (filter->qp == data->qp))
	      rc2++;
	  else{
	      diff++;
	      IB_LOG_VERBOSE("MAD QP differs ",data->qp);
	    }
      }

	if (!diff && filter->mai_filter_check_packet)
	  {
		diff += filter->mai_filter_check_packet(data);
	  }

    if (diff)
      {
	  /*
	   * if any differnce exits in header fail the match 
	   */
	  IB_EXIT(__func__, 0);
	  return (0);
      }

    /*
     * Else we have match 
     */
    IB_EXIT(__func__, rc2);
    return (rc2);
}

/*
 * maif_reduce
 *   Called with pointers to two filters.  It compares them and returns
 * information on how they compare.  Assume parameters Filt1 and Filt2,
 * this code returns:
 *
 *   MAI_FILTER_SUBSET  - Filt1 is a subset of Filt2 
 *   MAI_FILTER_SUPSET  - Filt1 is a superset of Filt2 
 *   MAI_FILTER_DISTINCT- Filt1 has nothing to do with Filt2 
 *                        (or at least 1 is not a filter)
 *   MAI_FILTER_EQUAL   - Filt1 is idential to Filt2
 *
 * TODO:
 *   1. Must be updated once filters settle down.
 */

#define FMATCH_TEST(var,mask){						\
	if (filt1->active & mask) {					\
		if (filt1->var == MAI_FILTER_ANY) {			\
			any++;						\
			if ((filt2->active & mask)) {			\
				rc++;					\
				if (filt2->var != MAI_FILTER_ANY) {	\
					supset++;			\
				}					\
			} else {					\
				mismatch++;				\
			}						\
		} else {						\
			if ((filt2->active & mask)) {			\
				if (filt2->var == MAI_FILTER_ANY) {	\
					rc++;				\
					subset++;			\
				} else if (filt1->var == filt2->var) {	\
					rc++;				\
				} else {				\
					mismatch++;			\
				}					\
			} else {					\
				mismatch++;				\
			}						\
		}							\
	}								\
}

int
maif_reduce(Filter_t * filt1, Filter_t * filt2)
{
    int             rc;
    int             mismatch = 0; /* > 0 if filters are different in 
				  * anyway 
				  */
    int             supset;	/* > 0 if test filter is subset of
				 * reference filter */
    int             subset;	/* > 0 if ref filter is subset of test
				 * filter */
    int             any;

    IB_ENTER(__func__, filt1, filt2, 0, 0);

    /*
     * Validate the Filter_t structure 
     */
    rc = mai_validate_filter(filt1);
    if (!rc)
      {
	  IB_LOG_ERRORRC("Invalid(1) filter rc:", rc);
	  IB_EXIT(__func__, 0);
	  return (0);
      }
    rc = mai_validate_filter(filt2);
    if (!rc)
      {
	  IB_LOG_ERRORRC("Invalid(2) filter rc:", rc);
	  IB_EXIT(__func__, 0);
	  return (0);
      }

#if 0
    DUMP_FILTER(filt1, 1);
    DUMP_FILTER(filt2, 2);
#endif

    /*
     * Strip off and handle the identical case 
     */
    rc = memcmp(filt1, filt2, sizeof(Filter_t));
    if (rc == 0)
      {
	  IB_EXIT(__func__, MAI_FILTER_EQUAL);
	  return (MAI_FILTER_EQUAL);
      }

    /*
     * Do a comparison of the fields directly 
     */
    if (filt1->active == filt2->active &&
	filt1->type == filt2->type &&
	filt1->dev == filt2->dev &&
	filt1->port == filt2->port &&
	filt1->flags == filt2->flags &&
	filt1->qp == filt2->qp &&
	!memcmp((void *) (&filt1->value), (void *) (&filt2->value),
		sizeof(filt2->value)) &&
	!memcmp((void *) (&filt1->mask), (void *) (&filt2->mask),
		sizeof(filt2->mask)))
      {
	  IB_EXIT(__func__, MAI_FILTER_EQUAL);
	  return (MAI_FILTER_EQUAL);
      }

    /*
     * Assume they do not match at all 
     */
    rc = 0;

#ifdef PAW
    /*
     * Handle the nasty match logic for mask+value stuff 
     */
    rc += FILT_TEST(bversion);
    rc += FILT_TEST(mclass);
    rc += FILT_TEST(cversion);
    rc += FILT_TEST(method);
    rc += FILT_TEST(status);
    rc += FILT_TEST(hopPointer);
    rc += FILT_TEST(hopCount);
    rc += FILT_TEST(tid);
    rc += FILT_TEST(aid);
    rc += FILT_TEST(amod);

    IB_LOG_INFO("Mask/Value testing rc", rc);
    if (rc != 0)
	rc = -1;
#endif

    /*
     * Handle the match any guys 
     */
    rc = 0;
    mismatch = 0;
    subset = 0;
    supset = 0;
    any   = 0;
    FMATCH_TEST(type, MAI_ACT_TYPE);
    FMATCH_TEST(dev, MAI_ACT_DEV);
    FMATCH_TEST(port, MAI_ACT_PORT);
    FMATCH_TEST(qp, MAI_ACT_QP);

    IB_LOG_VERBOSE("Any count", any);

    /*
     * Some notes: If There is mismatch in the header. The possiblilties
     * are - 1. filt2 has some MAI_FILTER_ANY values for values that filt1 
     * hold as distinct.In this case subset indicates the number of
     * incidence of this type (that filt1 value is is a subset of range
     * covered by filt2). 2. The inverse of - filt1 has MAI_FILTER_ANY on
     * some values that filt2 has set as distinct - supset indicates the
     * number incidenct of this type (filt1 value is superset of the value 
     * covered by filt2). 
     */


    /*
     * Now compare the the values and mask to determine complete
     * equivalence 
     */
    if (mismatch == 0 && rc)
      {
	  /*
	   * If filt1  does not have an active MAD mask then 
	   * the equivalence is determined solely by above match above
	   */

	  if ((filt1->active & MAI_ACT_FMASK) == 0)
	    {
		if ((filt2->active & MAI_ACT_FMASK))
		  {
		      /*
		       * We have mismatch on the FMASK only. 
		       * The MAD specification ofn filt2 narrows it
		       * scope further thereby making it subset of filt1.
		       * We use the subset/supset values determine 
		       * if we should return explicit sets or fuzzy.
		       */

		      if (subset == 0)
			{
			    /*
			     * filt1 is not covered by any MAI_FILTER_ANY
			     * in filt2.
			     */

			    IB_LOG_INFO(" Filter SUPSET ", MAI_FILTER_SUPSET);
			    IB_EXIT(__func__, MAI_FILTER_SUPSET);
			    return (MAI_FILTER_SUPSET);
			}
		      else
			{
			    /*
			     * filt2 covers filt1 by an MAI_FILTER_ANY
			     * but the range of filt2 is contracted by
			     * the presence of mask,value pair that 
			     * filt1 does not have. So we return
			     * fuzzy here.
			     */

			    IB_LOG_INFO(" Filter FUZZY ", MAI_FILTER_FUZZY);
			    IB_EXIT(__func__, MAI_FILTER_FUZZY);
			    return (MAI_FILTER_FUZZY);

			}
		  }
		else
		  {
		      /*
		       * Neither filters have active mask so they are equivalent,
		       * subject to subset/supset incidents.           
		       */

		    fuzzy:
		      if (subset == 0 && supset == 0)
			{
			    /*
			     * Filters are equal in every way 
			     */
			    IB_LOG_INFO("Filter EQUAL", MAI_FILTER_EQUAL);
			    IB_EXIT(__func__, MAI_FILTER_EQUAL);
			    return (MAI_FILTER_EQUAL);
			}
		      else if (subset && supset == 0)
			{
			    /*
			     * filt1 is a subset of filt2 on some values 
			     */

			    IB_LOG_INFO(" Filter SUBSET ", MAI_FILTER_SUBSET);
			    IB_EXIT(__func__, MAI_FILTER_SUBSET);
			    return (MAI_FILTER_SUBSET);
			}
		      else if (subset == 0 && supset)
			{
			    /*
			     * filt1 is a superset of filt2 - it cover wider range on
			     * some values.
			     */

			    IB_LOG_INFO(" Filter SUPSET ", MAI_FILTER_SUPSET);
			    IB_EXIT(__func__, MAI_FILTER_SUPSET);
			    return (MAI_FILTER_SUPSET);
			}
		      else
			{
			    /*
			     * We have a combination of subset and supsets
			     * return fuzzy.
			     */
			    IB_LOG_INFO(" Filter FUZZY ", MAI_FILTER_FUZZY);
			    IB_EXIT(__func__, MAI_FILTER_FUZZY);
			    return (MAI_FILTER_FUZZY);

			}
		  }

	    }
	  else
	    {
		/*
		 * filt1 does  have a MAI_ACT_FMASK.
		 * See if  mask is active on filt2 and check.                   
		 */

		if ((filt2->active & MAI_ACT_FMASK) == 0)
		  {
		      /*
		       * We have mismatch on the FMASK only. 
		       * The MAD specification on filt1 narrows its
		       * scope further thereby making it subset of filt2. Use
		       * the subset/supset to resolve coverage
		       */

		      if (supset == 0)
			{
			    /*
			     * filt1 has no range advantage over filt2 so it is 
			     * a proper subset of filt2
			     */

			    IB_LOG_INFO(" Filter SUBSET ", MAI_FILTER_SUBSET);
			    IB_EXIT(__func__, MAI_FILTER_SUBSET);
			    return (MAI_FILTER_SUBSET);
			}
		      else
			{
			    /*
			     * We have contraction of the range on filt1 by
			     * the presence of the FMASK. We therefore have
			     * a fuzzy condition.
			     */

			    IB_LOG_INFO(" Filter FUZZY ", MAI_FILTER_FUZZY);
			    IB_EXIT(__func__, MAI_FILTER_FUZZY);
			    return (MAI_FILTER_FUZZY);

			}
		  }

		/*
		 * Both filters  have active mask; Four conditions can occur.
		 * 1. The masks,value pairs are equivalent.
		 * 2. They have nothing in common.
		 * 3. Filter 1 mask,value is superset of Filter 2
		 * 4. Filter 1 mask,value is subset   of Filter 2
		 */

	      mask_cmp:

		/*
		 * First check for equivalence 
		 */
		if (!memcmp
		    ((void *) (&filt1->value), (void *) (&filt2->value),
		     sizeof(filt2->value))
		    && !memcmp((void *) (&filt1->mask),
			       (void *) (&filt2->mask),
			       sizeof(filt2->mask)))
		  {
		      /*
		       * The pairs equate. Use subset supset to resolve
		       * set relations.
		       */

		      goto fuzzy;
		  }

		/*
		 * Now check if filt2 will match the value specified in filt1.
		 * First apply the filt1->mask and pass the result to 
		 * Test with filt2->mask. The logic of this test is that
		 * if filt1 mask for a given value covers a wider range than
		 * filt2 mask on the same value then filt2 will cover 
		 * a subset of filt1 value and will therefore have greater
		 * range of hits. Similarly if filt1 mask covers a shorter
		 * range that filt2 it will have the greater range.
		 * If the mad = filt1->mask & filt1->value, is under
		 * filt2 range we have a collision. IF not then there is
		 * no overlap here.
		 */

		{
		    Mad_t           mad;
		    uint8_t        *value,
		                   *mask,
		                   *result;
		    int             rc_tmp;
		    uint32_t        i;

		    value = (uint8_t *) (&filt1->value);
		    mask = (uint8_t *) (&filt1->mask);
		    result = (uint8_t *) & mad;

		    for (i = 0; i < sizeof(mad); i++)
			result[i] = value[i] & mask[i];

		    rc_tmp = maif_maskequate_filter((void *) &mad, filt2);

		    if (rc_tmp)
		      {
			  /*
			   * There is intersection. The mask,value pairs of the
			   * filters intersect. There are two possible
			   * interpretations here:
			   * 1. filt1 mask,value is a subset of filt2 mask,value pair
			   * 2. filt1 mask,value is equal to that of filts2.
			   * The equivalence test above rules out option 2. So we
			   * return filt1 is a subset of filt2 if no fuzziness exist.
			   */

			  if (supset == 0)
			    {
				IB_LOG_INFO(" Filter SUBSET ", MAI_FILTER_SUBSET);
				IB_EXIT(__func__, MAI_FILTER_SUBSET);
				return (MAI_FILTER_SUBSET);
			    }
			  else
			    {
				/*
				 * filt1 is mask,value range is subset of
				 * filt2 but its header has wider range.
				 * We therefore have a fuzzy situation.
				 */

				IB_LOG_INFO(" Filter FUZZY ", MAI_FILTER_FUZZY);
				IB_EXIT(__func__, MAI_FILTER_FUZZY);
				return (MAI_FILTER_FUZZY);
			    }
		      }

		    /*
		     * Now we see if filt1 is superset of filt2 mask,value 
		     * pair 
		     */
		    value = (uint8_t *) (&filt2->value);
		    mask = (uint8_t *) (&filt2->mask);
		    result = (uint8_t *) & mad;

		    for (i = 0; i < sizeof(mad); i++)
			result[i] = value[i] & mask[i];

		    rc_tmp = maif_maskequate_filter((void *) &mad, filt1);

		    if (rc_tmp)
		      {

			  if (subset == 0)
			    {
				/*
				 * Filt1 is not a subset of filt2 in any
				 * way so it is exclusively a superset of 
				 * filt2. 
				 */
				IB_LOG_INFO(" Filter SUPSET ", MAI_FILTER_SUPSET);
				IB_EXIT(__func__, MAI_FILTER_SUPSET);
				return (MAI_FILTER_SUPSET);
			    }
			  else
			    {
				/*
				 * * filt2 is mask,value range is subset
				 * of * filt1 but its header has wider
				 * range. * We therefore have a fuzzy
				 * situation. 
				 */

				IB_LOG_INFO(" Filter FUZZY ", MAI_FILTER_FUZZY);
				IB_EXIT(__func__, MAI_FILTER_FUZZY);
				return (MAI_FILTER_FUZZY);
			    }

		      }

		    /*
		     * The mask value pairs have nothing in common and so cancels
		     * equivalence in the filters as suggested by the filter headers
		     */

		    IB_LOG_INFO(" Filter DISTINCT", MAI_FILTER_DISTINCT);
		    IB_EXIT(__func__, MAI_FILTER_DISTINCT);
		    return (MAI_FILTER_DISTINCT);

		}
	    }
      }
    else if (mismatch == 0 && rc == 0)
      {
	  /*
	   * None of the tested flags were active in the filter. Now test the
	   * MAI_ACT_FMASK to see if we have some action.
	   */

	  /*
	   * Can't  have all the flags on a filter set to zero..
	   */

	  MAI_ASSERT_TRUE(((filt1->active & MAI_ACT_FMASK) != 0));

	  if ((filt2->active & MAI_ACT_FMASK) == 0)
	    {
		/*
		 * filt2 has FMASK set; The return value
		 * is determined soley by the values on the 
		 * mask,value pairs.
		 */
		goto mask_cmp;
	    }

      }

    /*
     * If we get here then the filters have nothing in common
     */

    IB_LOG_INFO(" Filter DISTINCT", MAI_FILTER_DISTINCT);
    IB_EXIT(__func__, MAI_FILTER_DISTINCT);
    return (MAI_FILTER_DISTINCT);

}
