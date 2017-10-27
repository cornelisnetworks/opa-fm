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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/****************************************************************************/
/*                                                                          */
/* FILE NAME                                               VERSION          */
/*      mai_g.h                                            MAPI 0.05        */
/*                                                                          */
/* DESCRIPTION                                                              */
/*      This file contains global function prototypes and structure         */
/*      definitions for the Management API.                                 */
/*                                                                          */
/*                                                                          */
/* DATA STRUCTURES                                                          */
/*   	Filter_t	        Holds filter description		    */
/*   	Mai_t		        Used to exchange MADs with API		    */
/*                                                                          */
/* FUNCTIONS                                                                */
/*      max_get_max_filters     return MAI_MAX_FILTERS for unit test use    */
/*      mai_open                Create a channel to send/recv MADs  	    */
/*      mai_close               Tear down an unneeded channel      	    */
/*      mai_filter_create       Add a filter to an open channel    	    */
/*      mai_filter_delete       Remove a channel                    	    */
/*      mai_recv                Receive one MAD from a channel      	    */
/*      mai_send                Send a MAD to an open channel       	    */
/*      mai_init                Initialize MAD API layer            	    */
/*	mai_filter_mclass	Create filter that accepts only MADs 	    */
/*				of a specified mclass			    */
/*	mai_filter_method	Create a filter that accepts only MADs 	    */
/*				of a specified mclass and method	    */
/*	mai_filter_aid		Create a filter that accepts only MADs 	    */
/*				of a specified mclass, method, and aid	    */
/*	mai_filter_amod		Create a filter that accepts only MADs of   */
/*				a specified mclass, method, aid, and amod   */
/*	mai_filter_once		Create a one-shot’ filter with a unique     */
/*				TID and associate it with the filter 	    */
/*	mai_filter_tid		Create a filter that accepts only MADs of   */
/*				a specified mclass and tid	  	    */
/*	mai_alloc_tid		Allocate a unique TID for this class	    */
/*	mai_filter_handle	Returns the handle associated with a 	    */
/*				previously created filter		    */
/*	mai_filter_hcreate	Create a filter				    */
/*	mai_filter_hdelete	Delete the specified filter		    */
/*	mai_get_stl_portinfo	Receive port information for the local port */
/*	mai_shutdown		Mark the MAI library as uninitialized	    */
/*                                                                          */
/* DEPENDENCIES                                                             */
/*      VIEO_THREADED                   Global definition                   */
/*                                                                          */
/*                                                                          */
/* HISTORY                                                                  */
/*          NAME             DATE                REMARKS                    */
/*       Jim Mott          01-15-2001            Entry from prototype code  */
/*     Jeff Young          02-08-2001            Fixed Filter_t def	    */
/*     Todd Pisek          02-12-2001            Moved grh b4 lrh in mai_mad*/
/*       Jim Mott          02-18-2001            Added base+extra lengths   */
/*                                               to Mai_t.                  */
/*    Peter Walker         03-26-2001            remove extra in Mai_t      */
/*                                                                          */
/* OUTSTANDING ISSUES                                                       */
/*                                                                          */
/****************************************************************************/


#ifndef _IB_MAI_G_H_
#define _IB_MAI_G_H_

#include <ib_status.h>
#include <ib_mad.h>
#include <vs_g.h>
#include "iba/stl_sm_priv.h"


#define MAD_BASEHDR_SIZE (24)

#define IB_MAX_MAD_DATA       (256)
#define IB_MAD_PAYLOAD_SIZE   (IB_MAX_MAD_DATA - MAD_BASEHDR_SIZE)

#define STL_MAX_MAD_DATA      (2048)
#define STL_MAD_PAYLOAD_SIZE  (STL_MAX_MAD_DATA - MAD_BASEHDR_SIZE)

#define MAI_FNAME_LEN    (16)
/*
 * Structure definitions
 */

/* MaiAddrInfo_t
 *   This striucture is used within Mai_t to describe the source and
 * destination addressing for a MAD packet.
 */
typedef struct _MaiAddrInfo_t {
	uint16_t	slid;		// source LID (16)
	uint16_t	dlid;		// destination LID (16)
	uint8_t		sl;		    // service level (4)
	uint16_t	pkey;		// partition key (16)
	uint32_t	srcqp;	    // WQP number at the source (24)
	uint32_t	destqp;		// destination QP (24)
	uint32_t	qkey;	    // queue pair key (32)
} MaiAddrInfo_t;

#define	SMI_MAD_SL	 0		// C7-25

/*
 * Mai_t
 *   This structure is the primary interchange for the
 * Management API and all the glue layers. It is created by managers
 * and agents on one side of the pipe and IB hardware on the other.
 *
 * active
 *   This bit field describes which parts of Mai_t contain
 * real information. Manager and agents use it to communicate with
 * hardware on wire level protocol.  The receive side
 * code+hardware use it to communicate with agents and managers how
 * much of the data is valid.
 *
 * type
 *   There are 3 types of mai_mad data objects:
 *     EXTERNAL    send + receive	Contains a MAD for the wire
 *     INTERNAL    send + receive	Used for inter-agent communication
 *   When an agent/manager sends a MAD of type INTERNAL, that data chunk
 * is transported to the very bottom of the stream and queued to
 * the incoming MAD chain. As it passes back up the stack, the
 * various fields are matched against every channels filters and
 * the MAD is duplicated on the input queue of each of them.
 *
 * dev
 *   When there are multiple FIs in a system, the dev parameter is
 * used to select them. FIs are numbered consecutively from 0 to
 * n-1 where there are n adapters. The dev fields must be >0 except
 * when constructing filters. dev==-1 --> MAI_FILTER_ANY.
 *
 * port
 *   Switches use the port parameter to select the correct output
 * port for PERMISSIVE MADs.  Multi-port channel adapters use the
 * port to indicate which port in the FI should be used.  The port
 * fields must be >0 except when constructing filters.
 * ports==-1 --> MAI_FILTER_ANY
 *
 * addrInfo
 *  addressing information for the MAD packet
 *
 * base
 *   Contains the standard MAD header
 *
 * datasize
 *   Contains the number of active bytes in data.
 *
 * data
 *   Optional fields at the end of the standard MAD header.
 *
 *
 *
 *
 * NOTES:
 *   1. You must always preserve the order of the following fields
 *        - base
 *        - data
 *   2. Changes to the active masks (MAI_ACT_*) require updates to the
 *      filter-checking logic in maif.c
 */

typedef struct _Mai_t
  {
  uint32_t active;		/* Active field bit array			*/
  int16_t  datasize;		/* Amount of valid data in 'data'; Default all.	*/
  int16_t  type;		/* Type of mad: EXTERNAL, INTERNAL		*/
  int16_t  dev;			/* Device that received the MAD			*/
  int16_t  port;		/* Port that the MAD is sent or received on	*/
  int32_t  qp;			/* QP for data					*/
  MaiAddrInfo_t addrInfo; /* address information */
  Mad_t    base;		/* Base MAD  					*/
  uint64_t intime;      /* Time data arrived at the bottom of the stack */
  uint8_t  data[STL_MAX_MAD_DATA];  /* The rest of basic MAD */
  /* DO NOT PLACE ANY FIELDS BELOW data */
  } Mai_t;


/* Function for filter callback */
typedef int (mai_filter_check_packet_t)(Mai_t * data);

/*
 * Filter_t
 *   This structure is used to pass filter requests around.
 *
 * NOTES:
 *   1. If MAI_FILTER_ANY is set for any value, a match is
 *      is considered to have occured only if:
 *      A. active has MAI_ACT_FMASK set and there is a corresponding
 *         match on the MAD data as set by the tuple (mask,value)
 *      B. active does not have MAI_ACT_FMASK set.
 */
typedef struct _Filter_t
  {
  uint32_t   active;			/* Active field mask: MAI_ACT_*		*/
  int16_t    type;			/* Type of MAD: EXTERNAL, INTERNAL	*/
  int16_t    dev;			/* Device that received the MAD		*/
  int16_t    port;			/* Port the MAD sent or received on	*/
  int32_t    qp;			/* Zero or one				*/
  uint8_t    unused;			/* Alignment				*/
  Mad_t      value;			/* Value to match			*/
  Mad_t      mask;			/* Mask to match			*/
  char       fname[MAI_FNAME_LEN+1];    /* To identify the filter        	*/
  uint32_t   refcnt;       		/* Instances of references to this filter */

  uint8_t    flags;			/* State flags: Opaque to user		*/
  void       *handle;			/* lower-level handle for this filter	*/
  mai_filter_check_packet_t * mai_filter_check_packet;
} Filter_t;
#define MAI_FILTER_ANY     (-1)		/* Match any type,dev,port,qp		*/




#define MAI_TYPE_ANY    (-1)	/* MAI_FILTER_ANY place holder		*/
#define MAI_TYPE_EXTERNAL     (1)	/* Normal MAD packet			*/
#define MAI_TYPE_INTERNAL     (3)	/* Mai_t payload is an internal message */
								/* sent between handles in the MAI stack */
#define MAI_TYPE_ERROR   (8)	/* Indicates error, mad was never sent, returning original mad */

#define MAI_TYPE_DROP    (9)	/* Indicates security violation error, mad was never processed, dropping the mad */

#define MAI_ACT_TYPE     (0x00000001)	/* Type field is valid		*/
#define MAI_ACT_ADDRINFO (0x00000004)	/* addrInfo contains valid data	*/
#define MAI_ACT_BASE     (0x00000040)	/* base contains valid data	*/
#define MAI_ACT_DATA     (0x00000080)	/* data contains valid data	*/
#define MAI_ACT_DEV      (0x00000100)	/* Device field is active	*/
#define MAI_ACT_PORT     (0x00000200)	/* Port field is active		*/
#define MAI_ACT_QP       (0x00000400)	/* qp field is active		*/
#define MAI_ACT_FMASK    (0x00000800)	/* Filter mask is active        */
#define MAI_ACT_FNAME    (0x00001000)	/* Filter name is active        */
#define MAI_ACT_TSTAMP   (0x00002000)	/* The Time stamp is active     */
#define MAI_ACT_DATASIZE (0x00004000)	/* datasize field valid data    */



#define MAI_MAX_PORT (255)
#define MAI_MAX_DEV  (255)

/* Used to set the mask on filters.     */
#define MAI_FMASK_ALL (~0)

/* Declaration on constants 		*/

#define MAI_SMI_QP         (0)      /* Subnet Management QP	*/
#define MAI_GSI_QP         (1)      /* General Service   QP	*/


/*
 * MAI_MAX_QUEUED_DEFAULT
 *   To prevent one consumer from shutting down all the others by not
 * servicing his queue, we will limit the maximum number of outstanding
 * MADs allowed on a single channel.
 *
 *   Note that this default may be overriden prior to initializing MAI.
 */
#ifndef MAI_MAX_QUEUED_DEFAULT
#ifdef __VXWORKS__
#define MAI_MAX_QUEUED_DEFAULT  256
#else
#define MAI_MAX_QUEUED_DEFAULT  512
#endif
#endif				/* MAI_MAX_QUEUED_DEFAULT */


/*
 * MAI_MAX_DATA_DEFAULT
 *   The size of the buffer pool that stores MADs flowing up from the
 * bottom.  In cases where we are single threaded, there are not many
 * of these.
 * It is possible for 2 SA threads to consume 2 X MAI_MAX_QUEUED by 
 * themselves so alloc 3X buffers to support SM/PM/BM/FE
 *
 *   Note that this default may be overriden prior to initializing MAI.
 */
#ifndef MAI_MAX_DATA_DEFAULT
#define MAI_MAX_DATA_DEFAULT  (5 * MAI_MAX_QUEUED_DEFAULT)    
#endif				/* MAI_MAX_DATA_DEFAULT */


/* Macro to set the name of a filter. Particularly useful during debug	*/

#define MAI_SET_FILTER_NAME(ft, szname) do{             \
  int _i=0; char *_p = (char *)(szname);                \
  int _l=(sizeof(szname)<(MAI_FNAME_LEN-1)?sizeof(szname):(MAI_FNAME_LEN-1)); \
   for(_i=0;_i<_l;_i++)                    \
     { (ft)->fname[_i]=_p[_i]; if (_p[_i]==0)break;}    \
    (ft)->fname[_i]=0; (ft)->active |= MAI_ACT_FNAME;   \
}while(0)

/*
 * Masks the bits of the TID that are reserved for the IB stacks use.  In
 * particular, OFED places no guarantee on preserving the integrity of the
 * upper 32 bits.
 *
 * To be used when comparing TID coming up from the stack against expected
 * values.
 */
#ifdef IB_STACK_OPENIB
#define MAI_MASK_TID(tid) (tid & 0xffffffffull)
#else
#define MAI_MASK_TID(tid) (tid)
#endif

/*
 * mai_set_num_endports
 *   This function can be called prior to calling mai_init.  It will adjust
 * the MAD buffer size and queue depths accordingly.  If not called, they will
 * be given reasonable defaults.
 *
 * INPUTS
 *      num_end_ports    The required number of fabric end ports to scale to.
 *
 * RETURNS
 *      Nothing.
 */
void mai_set_num_end_ports(uint32_t num_end_ports);


/*
 * mai_init
 *   This function must be called prior to calling any of the other management
 * API function. It can only be called once.
 *
 * INPUTS
 *      None
 *
 * RETURNS
 *      Nothing.  Errors are fatal.
 */
void mai_init(void);

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
unsigned mai_get_max_filters(void);

/*
 * mai_open
 *   This function will create a new channel to the SMI or GSI.
 * Initially there are no filters active on the channel so you will receive nothing
 * from it. You can always send MADs to a channel. The qp, dev, port parameters are
 * the default values associated with filters that are created on the handle.
 * This function also calls through the hardware to validate the values passed in.
 * If the values in the message have not been set by the caller, they will be the
 * default values assigned to Mai_t messages on mai_send.
 *
 * INPUTS
 *      qp          The QP to open.  SMI=QP0, GSI=QP1.  Nothing else is valid
 *      dev         Device number [0..n-1] when n FIs in a system
 *      port        FI port for multi-ported FIs
 *      fd          Pointer to a place to store the channel number
 *
 * RETURNS
 *      Status_t 	See ib_status.h
 */
Status_t mai_open(uint32_t qp, uint32_t dev, uint32_t port, IBhandle_t *fd);




/*
 * mai_close
 *      Close a previously opened channel. Release all resources (filters,
 *      queued read data) to the handle.
 *
 * INPUTS
 *      fd          An open channel (mai_open) to close
 *
 * RETURNS
 *      Status_t 	See ib_status.h
 */
Status_t mai_close(IBhandle_t fd);


#define VFILTER_SHARE      (0x000)  /* Filter will share MADs		*/
#define VFILTER_PURGE      (0x002)  /* Purge filter MADs on delete  	*/
#define VFILTER_ONCE       (0x008)  /* Filter deleted after first match	*/


/*
 * mai_filter_create
 *      Add a filter to an open channel. This function will add both
 *      absorbing filters and copy filters. In the case where a new filter
 *      has been added to absorb Subnet Manager Datatgrams (SMDs), this
 *	function will notify all other channels that would have received
 *	that SMD. If the dev, port, qp are not marked active in the filter,
 *	the default values associated with the handle are assigned to filter.
 *
 * INPUTS
 *      fd          An open channel (mai_open) to close
 *      filter      The filter to add to the open channel
 *      flags       Flags that modify the filter
 *
 * RETURNS
 *      Status_t 	See ib_status.h
 */
Status_t mai_filter_create(IBhandle_t fd, Filter_t *filter, uint32_t flags);

/*
 * mai_filter_delete
 *      This functions removes an active filter from an open channel. If
 *      there are multiple instances of the filter, it removes only the
 *      first one.
 *
 * INPUTS
 *      fd          An open channel (mai_open) to use
 *      filter      The filter to remove from the open channel
 *      flags       Flags that modify the filter
 *
 * RETURNS
 *      Status_t 	See ib_status.h
 */
Status_t mai_filter_delete(IBhandle_t fd, Filter_t *filter, uint32_t flags);


/*
 * mai_send_timeout
 *   Send an SMD to the hardware channel (QP0/QP1) interface associated with
 * the fd (mai_open). This is a synchronous call. The dev, port, qp that the message gets
 * sent on is determined by the values in the Mai_t argument. If the these
 * are not marked as active the values associated with the handle are assigned to it.
 *
 * Note that on OFED, the timeout will be passed down to the kernel.  This
 * does not make the call block, but instead just lets the kernel know how
 * long to keep the MAD around on the send list while waiting for an ACK.
 * On OFED, when the MAD expires from the kernel send list, incoming responses
 * with the same TID will be dropped.
 *
 * INPUTS
 *      fd          An open channel (mai_open) to send on
 *      buffer      Pointer to MAD to send down
 *      timeout     Send timeout to pass down to the kernel (OFED only)
 *
 * RETURNS
 *      Status_t 	See ib_status.h
 */
Status_t mai_send_timeout(IBhandle_t fd, Mai_t *buffer, uint64_t timeout);
Status_t mai_send_stl_timeout(IBhandle_t fd, Mai_t *buffer, uint32_t *datalen, uint64_t timeout);

#define MAI_DEFAULT_SEND_TIMEOUT (1ull * VTIMER_1S)
#define mai_send(fd, buffer) mai_send_timeout(fd, buffer, MAI_DEFAULT_SEND_TIMEOUT)
#define mai_stl_send(fd, buffer, datalen) mai_send_stl_timeout(fd, buffer, datalen, MAI_DEFAULT_SEND_TIMEOUT)

/*
 * mai_recv
 *   Receive the next SMD on the channel opened (mai_open) earlier.  The
 * MAD must pass the filters (mai_filter_create) added to the open channel.
 * A single MAD will be returned no matter how many filters on this channel
 * happen to describe that MAD. The return status indicates the normal MAD was
 * received or otherwise. MADs coming from the wire have status VSTATUS_OK.
 * Otherwise the following values are possible
 *   VSTATUS_MAI_INTERNAL - an internal message was received
 *   VSTATUS_FILTER - a filter take-over notice was received.
 *
 * INPUTS
 *      fd          An open channel (mai_open) to receive on
 *      buffer      Pointer to space to store the received MAD
 *      timeout     The number of microseconds to wait for a MAD;
 *                  	a timeout value of 0 means to wait forever
 *
 * RETURNS
 *      Status_t 	See ib_status.h
 */
Status_t mai_recv(IBhandle_t fd, Mai_t *buffer, uint64_t timeout);


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
Status_t mai_recv_handles(IBhandle_t *ha, uint32_t count, 
                          uint64_t timeout, uint32_t *pfirst, Mai_t *maip);

#define MAI_RECV_NOWAIT   (0ull)    /* Implies mai_recv should not block*/

/*
 * mai_alloc_tid
 *   Allocates a unique TID to be set in filters and MADs.
 *
 * INPUTS
 *      fd          An open channel (mai_open) to use
 *      mclass      The class associated with the TID.
 *      buf         Pointer to place the TID.
 *
 * RETURNS
 *      Status_t 	See ib_status.h
 */
Status_t mai_alloc_tid(IBhandle_t fd, uint8_t mclass, uint64_t  *buf);

/*
 * mai_increment_tid
 *   Given a tid allocated by mai_alloc_tid, computes a new unique TID
 *
 * INPUTS
 *      the previous tid
 *
 * RETURNS
 *      new tid
 */
uint64_t mai_increment_tid(uint64_t  tid);

/*
 * mai_filter_hcreate
 *
 * DESCRIPTION
 *      Add a filter to an open channel.  This function will add both
 *      absorbing filters and copy filters.  In the case where a new filter
 *      has been added to absorb SMDs, this function will notify all
 *      other channels that would have received that SMD. It returns a handle
 *      to the created filter.
 *
 *
 * INPUTS
 *      fd      Channel number to add the filter to
 *      filter  Pointer to the filter to add to the channel
 *      flags   Flags to modify filter (absorb or copy)
 *      fh      Pointer to the filter handle location
 *
 * OUTPUTS
 *      Status_t 	See ib_status.h
 */
Status_t mai_filter_hcreate(IBhandle_t fd, Filter_t *filter,
			    uint32_t flags, IBhandle_t *fh);


/*
 *mai_filter_mclass
 *
 * DESCRIPTION
 *      Create a filter that accepts only MADs of a specified mclass.
 *
 * INPUTS
 *      devh	Device handle returned by mai_open
 *      flags	A set of characteristics to determine behavior.
 *            	- VFILTER_SHARE: Filter shared MAD with other filters
 *      ftype 	Filter type
 *      filterh Filter handle returned by call.
 *      mclass 	The class to filter (see ib_mad.h for a list of classes).
 *
 * OUTPUTS
 *      Status_t 	See ib_status.h
 */
Status_t mai_filter_mclass(IBhandle_t devh, uint32_t flags, int ftype,
			   IBhandle_t *filterh, uint8_t mclass);

/*
 * mai_filter_method
 *
 * DESCRIPTION
 *      Create a filter that accepts only MADs of a specified mclass,
 *		method.
 *
 *
 * INPUTS
 *      devh	Device handle returned by mai_open
 *      flags	A set of characteristics to determine behavior.
 *            	- VFILTER_SHARE: Filter shared MAD with other filters
 *      ftype 	Filter type
 *      filterh	Filter handle returned by call.
 *      mclass	The class to filter (see ib_mad.h for a list of classes).
 *      method	The method within the class.
 *
 * OUTPUTS
 *      Status_t 	See ib_status.h
 *
 */

Status_t mai_filter_method(IBhandle_t devh, uint32_t flags, int ftype,
			   IBhandle_t *filterh, uint8_t mclass, uint8_t method);


/*
 * mai_filter_aid
 *
 * DESCRIPTION
 *      Create a filter that accepts only MADs of a specified mclass,
 *       method, aid.
 *
 * INPUTS
 *      devh	Device handle returned by mai_open
 *      flags	A set of characteristics to determine behavior.
 *            	- VFILTER_SHARE: Filter shared MAD with other filters
 *      ftype 	Filter type
 *      filterh	Filter handle returned by call.
 *      mclass	The class to filter (see ib_mad.h for a list of classes).
 *      method	The method within the class.
 *      aid		The attribute ID within the method.
 *
 * OUTPUTS
 *      Status_t 	See ib_status.h
 *
 */
Status_t mai_filter_aid(IBhandle_t devh, uint32_t flags, int ftype,
			IBhandle_t *filterh,
			uint8_t mclass,
			uint8_t method,
			uint16_t aid);



/*
 * mai_filter_amod
 *
 * DESCRIPTION
 *      Create a filter that accepts only MADs of a specified mclass,
 *      method, aid, and amod.
 *
 * INPUTS
 *      devh	Device handle returned by mai_open
 *      flags	A set of characteristics to determine behavior.
 *            	- VFILTER_SHARE -   Filter shared    MAD  with other filters
 *      ftype	Filter type
 *      filterh	Filter handle returned by call.
 *      mclass	The class to filter (see ib_mad.h for a list of classes).
 *      method	The method within the class.
 *      aid		The attribute ID within the method.
 *      amod	The attribute modifier within the aid
 *
 * OUTPUTS
 *      Status_t 	See ib_status.h
 */
Status_t mai_filter_amod(IBhandle_t devh, uint32_t flags, int ftype,
			 IBhandle_t *filterh,
			 uint8_t mclass,
			 uint8_t method,
			 uint16_t aid,
			 uint32_t amod);



/*
 * mai_filter_handle
 *
 * DESCRIPTION
 *       Returns the handle to previously created filter.
 *
 * INPUTS
 *      devh	Device handle returned by mai_open and used in filter create.
 *      filter	The filter's definition.
 *      flags	A set of characteristics to determine behavior.
 *             	- VFILTER_SHARE -   Filter shared MAD  with other filters
 *      fh		Pointer to filter handle returned by call.
 *
 * OUTPUTS
 *      Status_t 	See ib_status.h
 *
 */

Status_t mai_filter_handle(IBhandle_t devh,
			   Filter_t *filter,
			   uint32_t flags,
			   IBhandle_t *fh);

/*
 * mai_filter_hdelete
 *
 * DESCRIPTION
 *      Removes a filter from a channel.
 *
 * INPUTS
 *      devh	Device handle returned by mai_open and used in filter create.
 *      fh		Filter handle
 *
 * OUTPUTS
 *      Status_t 	See ib_status.h
 *
 */
Status_t mai_filter_hdelete(IBhandle_t devh, IBhandle_t fh);


/*
 *  mai_filter_once
 *
 * DESCRIPTION
 *      This routine will create a filter which will accept messages
 *      for a specified mclass and TID. This routine allows an application
 *      to issue a request and wait for the reply by filtering on just
 *      the TID and mclass. After a messAge is received which passes the filter,
 *      the filter is automatically deleted.
 *
 *
 * INPUTS
 *      devh	Device handle returned by mai_open
 *      flags	A set of characteristics to determine behavior.
 *              - VFILTER_SHARE: Filter shared MAD with other filters
 *      ftype 	Filter type
 *      filterh	Filter handle returned by call.
 *      mclass	The class to filter (see ib_mad.h for a list of classes).
 *      tid		The transaction ID to use.
 *
 * OUTPUTS
 *      Status_t 	See ib_status.h
 *
 */
Status_t mai_filter_once(IBhandle_t devh, uint32_t flags, int ftype,
			 IBhandle_t *filterh,
			 uint8_t mclass,
			 uint64_t tid);



/*
 *  mai_filter_tid
 *
 * DESCRIPTION
 *      This routine will create a filter which will accept messages
 *      for a specified mclass and tid.
 *
 *
 * INPUTS
 *      devh	Device handle returned by mai_open
 *      flags	A set of characteristics to determine behavior.
 *              - VFILTER_SHARE: Filter shared MAD with other filters
 *      ftype	Filter type
 *      filterh	Filter handle returned by call.
 *      mclass	The class to filter (see ib_mad.h for a list of classes).
 *      tid		The transaction ID to use.
 *
 * OUTPUTS
 *      Status_t 	See ib_status.h
 *
 */
Status_t mai_filter_tid(IBhandle_t devh, uint32_t flags, int ftype,
			IBhandle_t *filterh,
			uint8_t mclass,
			uint64_t tid);



/*
 *  mai_get_stl_portinfo
 *
 * DESCRIPTION
 *      This function allows the local user to retrieve the portinfo
 *      of the local port.
 *
 *
 * INPUTS
 *      fd      MAI handle
 *      pinfop  Pointer to STL_PORT_INFO
 *      port    The port to get
 *
 * OUTPUTS
 *      Status_t 	See ib_status.h
 *
 */
Status_t mai_get_stl_portinfo(IBhandle_t fd, STL_PORT_INFO *pinfop, uint8_t port);


/*
 * FUNCTION
 *      mai_shutdown
 *
 * DESCRIPTION
 *      This function marks the MAI library (at the calling layer) as uninitialized.
 * 		This forces all users of the interface to fail and return.
 *
 * INPUTS
 *      None
 *
 * OUTPUTS
 *      None
 */

void mai_shut_down(void);

/*
 * FUNCTION
 *      mai_get_default_pkey
 *
 * DESCRIPTION
 *  	This function retrieves the default PKey (index 0) for the adapter.    
 * 		
 *
 * INPUTS
 *      None
 *
 * OUTPUTS
 */ 

uint16_t mai_get_default_pkey(void);

#endif /* IB_MAI_G_H */









