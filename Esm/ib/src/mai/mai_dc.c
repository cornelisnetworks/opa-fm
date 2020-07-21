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
 *      mai_dc.c                                           MAPI 0.05        *
 *                                                                          *
 * DESCRIPTION                                                              *
 *      This file contains the interfaces the down call interface to       *
 *      the MAI subsystem                                                   *
 *                                                                          *
 * DATA STRUCTURES                                                          *
 *      None                                                                *
 *                                                                          *
 ****************************************************************************/

#include "mai_l.h"		/* Local mai function definitions */

 /*
  * FUNCTION
  *      mai_dc_read
  *
  * DESCRIPTION
  *      The caller acts as the DC thread of MAI. It purpose is  to
  *      ensure that a thread is to do  a down call receive, 
  *      pulling mads up into MAI.
  *      It is in no way different from a normal application doing 
  *      a mai_recv call, except that no filters should be  associated with
  *      the handle owned by this thread. No filters .. means it never
  *      receives anything.
  * 
  *
  * INPUTS
  *      fd      An open MAI handle.
  *
  * OUTPUTS
  *      VSTATUS_OK
  *      VSTATUS_ILLPARM
  *
  * HISTORY
  *      NAME      DATE          REMARKS
  *      PAW     03/19/02 Initial entry
  */

int mai_dc_read_exit = 0;

#define DC_THREAD_TIMEOUT (15000000)
static Mai_t           mad;

int
mai_dc_read(IBhandle_t fd)
{
    int             rc;

    IB_ENTER(__func__, fd, 0, 0, 0);

    if (gMAI_USE_DEDICATED_DCTHREAD) {

        if (gMAI_DCTHREAD_HANDLE != MAI_INVALID) {
            IB_LOG_WARNX("down call thread handle is busy handle:",
                        gMAI_DCTHREAD_HANDLE);
            IB_EXIT(__func__, VSTATUS_BUSY);
            return VSTATUS_BUSY;
        }

        /*
         * Remember the handle associated with the thread 
         */
        gMAI_DCTHREAD_HANDLE = fd;
        /*  release the waiting initiator thread */
		cs_vsema(&gMAI_DCTHREAD_SEMA);
    }
    mai_dc_read_exit = 0;
    if (mai_dc_read_exit == 1) {
        IB_LOG_VERBOSE0("Mai DC Task exiting OK.");
        return VSTATUS_OK;
    }
    do {
        rc = mai_recv(fd, &mad, DC_THREAD_TIMEOUT);
    } while (rc == VSTATUS_TIMEOUT);

    if (rc != VSTATUS_TIMEOUT) {
        if (rc == VSTATUS_CONNECT_GONE) {
            IB_LOG_VERBOSERC("mai is shutting down rc:", rc);
        } else {
            IB_LOG_ERRORRC("bad status rc:", rc);
        }
    }

    if (gMAI_USE_DEDICATED_DCTHREAD) {
        /*
         * Reset the handle associated with the DC thread 
         */
        gMAI_DCTHREAD_HANDLE = MAI_INVALID;
    }

    IB_EXIT(__func__, rc);
    return rc;

}

 /*
  * FUNCTION
  *      mai_dc_reader
  *
  * DESCRIPTION
  *      Perpetually loops doing receives. 
  *      It serves only to pull ensure that a thread is always 
  *      doing a down call receive, pulling mads up into MAI.
  *      It is in no way different from a normal application doing 
  *      a mai_recv call, except that no filters are associated with
  *      the handle owned by this thread. No filters .. means it never
  *      receives anything.
  *      Note that although a device, port, qp argument is passed in,
  *      ib_sma_recv that mai_rcv calls, always return the mads 
  *      from any source that has been received. It is the filters 
  *      in MAI that determines who gets what, not the open args.
  * 
  *
  * INPUTS
  *      dev      A device to open.
  *      port     A port on the device
  *      qp       A qp on the port.
  *
  * OUTPUTS
  *      VSTATUS_OK
  *      VSTATUS_ILLPARM
  *
  * HISTORY
  *      NAME      DATE          REMARKS
  *      PAW     06/4/01 Initial entry
  */

int
mai_dc_reader(int qp, int dev, int port)
{
    int             rc;
	IBhandle_t	    fd;

    IB_ENTER(__func__, qp, dev, port, 0);

    rc = mai_open(qp, dev, port, &fd);

    if (rc != VSTATUS_OK)
	return rc;

    (void) mai_dc_read(fd);

    IB_LOG_INFINI_INFO0("MAI DC Reader shutting down..");
    (void) mai_close(fd);
    IB_EXIT(__func__, 0);
    return 0;

}

/*
 * FUNCTION
 *      mai_spawn_dc_reader
 *
 * DESCRIPTION
 *      Spawn a thread that calls the mai_dc_read function.
 * 
 * CALLED BY
 *
 * CALLS
 
 *       mai_dc_reader
 *
 * INPUTS
 *
 * OUTPUTS
 *
 * HISTORY
 *      NAME      DATE          REMARKS
 *      PAW     06/4/01 Initial entry
 */
struct {
    int             dev;
    int             qp;
    int             port;
} dc_reader_args;

static void
dc_reader_start(void)
{
    int             rc;
    IB_ENTER(__func__,
	     dc_reader_args.qp,
	     dc_reader_args.dev, dc_reader_args.port, 0);

    rc = mai_dc_reader(dc_reader_args.qp,
		       dc_reader_args.dev, dc_reader_args.port);
    IB_EXIT(__func__, rc);
    (void)(rc); // fix "unused" warning
}

/*
 * FUNCTION
 *      mai_spawn_dc_reader
 *
 * DESCRIPTION
 *      Spawn a thread that calls the mai_dc_read function.
 * 
 * CALLED BY
 *
 * CALLS
 
 *       vs_thread_create
 *
 * INPUTS
 *      dev      A device to open.
 *      port     A port on the device
 *      qp       A qp on the port.
 *
 * OUTPUTS
 *      VSTATUS_OK
 *      VSTATUS_BAD
 *
 * HISTORY
 *      NAME      DATE          REMARKS
 *      PAW     06/4/01 Initial entry
 */

#define MAI_DC_STACK_SIZE (8*1024)
static Thread_t dc_thandle;

int
mai_spawn_dc_reader(int qp, int dev, int port)
{
    int             rc;
    unsigned char name[VS_NAME_MAX];
 
    IB_ENTER(__func__, qp, dev, port, 0);

    dc_reader_args.qp = qp;
    dc_reader_args.dev = dev;
    dc_reader_args.port = port;

    /*
     * start the workert thread 
     */
    (void) memset (name, 8, sizeof(name));
    (void) sprintf((void *)name,"mai_dc");
    rc  =   vs_thread_create (&dc_thandle, 
			      name, 
			      (void (*)(uint32_t, uint8_t **))dc_reader_start,
			      0,NULL ,
			      MAI_DC_STACK_SIZE);
    if (rc) {
			IB_LOG_ERRORRC("failed to create MAI layer server thread rc:", rc);
	IB_EXIT(__func__, rc);
	  return rc;
    } else {
        /* need to wait (up to 2 seconds) for the dedicated dcthread to initialize here */
		return(cs_psema_wait(&gMAI_DCTHREAD_SEMA, 2));
	}

    IB_LOG_INFO("INFO:  MAI layer server thread created ", rc);
    IB_EXIT(__func__, 0);
    return VSTATUS_OK;
}

#ifdef __VXWORKS__
void 
mai_dc_read_kill(){
	mai_dc_read_exit = 1;
}
#endif

