/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015-2017, Intel Corporation

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifdef IB_STACK_OPENIB


#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <assert.h>
#include <ctype.h>
#include <string.h>

#define OPAMGT_PRIVATE 1
#include "ibt.h"

#include "ib_utils_openib.h"

static FSTATUS unimplemented_fn (const char * fn)
{
    return(FSUCCESS);
}

FSTATUS iba_init(void)
{
    return (unimplemented_fn (__func__));
}

FSTATUS iba_umadt_deregister( MADT_HANDLE   serviceHandle)
{
    return (unimplemented_fn (__func__));
}

FSTATUS iba_umadt_get_sendmad(MADT_HANDLE serviceHandle, uint32 *madCount, MadtStruct** mad)
{
    return (unimplemented_fn (__func__));
}

FSTATUS iba_umadt_release_sendmad(MADT_HANDLE serviceHandle, MadtStruct *mad)
{
    //return (unimplemented_fn (__func__)); // Removed due to too many prints
    return FSUCCESS;
}

FSTATUS iba_umadt_release_recvmad(MADT_HANDLE serviceHandle, MadtStruct *mad)
{
    return (unimplemented_fn (__func__));
}

FSTATUS iba_umadt_post_send(MADT_HANDLE serviceHandle, MadtStruct *mad, MadAddrStruct *DestAddr)
{
    return (unimplemented_fn (__func__));
}

FSTATUS iba_umadt_wait_any_compl(MADT_HANDLE serviceHandle, uint32 completionType, uint32 timeout)
{
    return (unimplemented_fn (__func__));
}

FSTATUS iba_umadt_poll_recv_compl(MADT_HANDLE serviceHandle, MadtStruct **mad, MadWorkCompletion **ppWorkCmpletion)
{
    return (unimplemented_fn (__func__));
}

FSTATUS iba_umadt_poll_send_compl(MADT_HANDLE serviceHandle, MadtStruct **mad, MadWorkCompletion **ppWorkCmpletion)
{
    //return (unimplemented_fn (__func__)); // Removed due to too many prints
    return 0;
}

void flush_rcv(MADT_HANDLE umadtHandle, int timeout)
{
    unimplemented_fn (__func__);
}

///////////////////////////////////////////////////////////
//                                                       //
//    IMPLEMENTED FUNCTIONS WITH OFED EQUIVALENTS        //
//                                                       //
///////////////////////////////////////////////////////////

FSTATUS iba_get_caguids_alloc(uint32 *CaCount,EUI64 **CaGuidList)
{
    // Call OFED equivalent
    return (omgt_get_caguids(CaCount, CaGuidList));
}

FSTATUS iba_query_ca_by_guid_alloc(IN  EUI64 CaGuid,OUT IB_CA_ATTRIBUTES *CaAttributes )
{
    // Call OFED equivalent
    return (omgt_query_ca_by_guid_alloc(CaGuid, CaAttributes));
}

FSTATUS iba_query_ca_by_guid(IN  EUI64 CaGuid,OUT IB_CA_ATTRIBUTES *CaAttributes )
{
    // Call OFED equivalent
    return (omgt_query_ca_by_guid(CaGuid, CaAttributes));
}


FSTATUS iba_get_portguid(
        IN uint32 ca,
        IN uint32 port,
        OUT EUI64 *pCaGuid OPTIONAL,
        OUT EUI64 *pPortGuid OPTIONAL,
        OUT IB_CA_ATTRIBUTES    *pCaAttributes OPTIONAL,
        OUT IB_PORT_ATTRIBUTES      **ppPortAttributes OPTIONAL,
        OUT uint32 *pCaCount OPTIONAL,
        OUT uint32 *pPortCount OPTIONAL
        )
{
    return (omgt_get_portguid(ca, port, NULL, NULL,  pCaGuid, pPortGuid, pCaAttributes, ppPortAttributes,
                                    pCaCount, pPortCount, NULL, NULL, NULL));

}

/* format an error message when iba_get_portguid returns FNOT_FOUND
 *
 * INPUTS:
 * 	ca, port - inputs provided to failed call to iba_get_portguid
 *	CaCount, PortCount - outputs returned by failed call to iba_get_portguid
 *
 * RETURNS:
 * 	string formatted with error message.  This is a pointer into a static
 *  string which will be invalidated upon next call to this function.
 */
const char*
iba_format_get_portguid_error(
		IN uint32 ca,
		IN uint32 port,
		IN uint32 caCount,
		IN uint32 portCount
		)
{
	static char errstr[80];

	if (ca) {
		if (portCount) {
			if (port) {
				sprintf(errstr, "Invalid port number: %u, FI %u only has %u Ports",
						port, ca, portCount);
			} else {
				sprintf(errstr, "No Active Ports found on FI %u", ca);
			}
		} else {
			sprintf(errstr,"Invalid FI number: %u, only have %u FIs",
						ca, caCount);
		}
	} else {
		if (portCount) {
			if (port) {
				sprintf(errstr, "Invalid port number: %u, System only has %u Ports",
						port, portCount);
			} else {
				sprintf(errstr, "No Active ports found in System");
			}
		} else {
			sprintf(errstr,"No FIs found in System");
		}
	}
	return errstr;
}
#endif
