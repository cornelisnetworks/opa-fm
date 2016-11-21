/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifdef IB_STACK_OPENIB


#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <assert.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>

#include <infiniband/umad_types.h>
#include <infiniband/verbs.h>
#include <infiniband/umad.h>
#include <infiniband/umad_sa.h>
#include <infiniband/umad_sm.h>

#define OIB_UTILS_PRIVATE 1

// Includes / Defines from IBACCESS
#include "ibt.h"
#include "oib_utils.h"
#include "ib_utils_openib.h"
#include "ib_pm.h"
#include "stl_pa.h"
#include "ib_mad.h"
#include "oib_utils_dump_mad.h"
#include "ib_sd.h" 
#include "stl_pm.h" 
#include "oib_utils_pa.h"
#include "oib_utils_sa.h"

/************************************** 
 * Defines and global variables 
 **************************************/

#define IB_SA_DATA_SIZE         200 // PA_REQ_HEADER_SIZE + sizeof()     /* DEBUG_CODE: Patch - cjking: Temporary patch, in order to support reception/trasmission of MADs */
#define IB_SA_DATA_OFFS         56
#define DEF_BUF_DATA_SIZE       (1024 * 10)

#define MAX_PORTS_LIST          300000  // Max number of ports in port list

#define OIB_UTILS_MAD_SIZE      256
#define OIB_UTILS_DEF_RETRIES   3
#define MAD_STATUS_PA_MINIMUM   0x0A00

// from ib_pa.h
#define MAD_STATUS_PA_NO_GROUP  0x0B00  // No such group
#define MAD_STATUS_PA_NO_PORT   0x0C00  // Port not found

#define RMPP_MAD_TAG_HDR_LEN            16
#define RMPP_MAD_TAG_HDR_BASE_OFFSET    1992
#define RMPP_MAD_TAG_HDR_OFFSET_INC     (RMPP_MAD_TAG_HDR_BASE_OFFSET + RMPP_MAD_TAG_HDR_LEN)

#define	IBA_SUBN_ADM_HDRSIZE    56  /* common + class specific header */

#define OIB_WARN(fmt, ...) fprintf(stderr, "oib_util_warn: [%d] %s: " fmt "\n", \
            (int)getpid(), __func__, ## __VA_ARGS__)

// from ib_sa.h
#define	PR_COMPONENTMASK_SRV_ID	0x0000000000000001ull
#define	PR_COMPONENTMASK_DGID	0x0000000000000004ull
#define	PR_COMPONENTMASK_SGID	0x0000000000000008ull

//from ib_sa_records.h
#define IB_MULTIPATH_RECORD_COMP_NUMBPATH	0x00000040


struct dest_portid 
{
    int lid;    /* lid or 0 if directed route */
    struct 
    {
        int         cnt;
        uint8_t     p[UMAD_SMP_MAX_HOPS];
        uint16_t    drslid;
        uint16_t    drdlid;
    } dr_path;
    int         grh_present;    /* flag */
    union       ibv_gid gid;
    uint32_t    qp;
    uint32_t    qkey;
    uint8_t     sl;
    unsigned    pkey_idx;
};

struct oib_rmpp_md 
{
    uint8_t     mgtclass;
    uint8_t     mgtclass_ver;
    uint8_t     method;
    uint16_t    attr_id;
    uint32_t    attr_mod;
    uint16_t    rstatus;    /* return status */
    int         dataoffs;
    int         data_size;
    uint64_t    trid;       /* used for out mad if nonzero, return real val */
    int         timeout;
    uint16_t    attr_offset;
    uint8_t     oui[3];
};

static uint8_t ib_truescale_oui[] = {OUI_TRUESCALE_0, OUI_TRUESCALE_1, OUI_TRUESCALE_2};

static const char* const pa_query_input_type_text[] = 
{
    "InputTypeNoInput",
};

static const char* const pa_query_result_type_text[] = 
{
    "OutputTypePaTableRecord",        /* PA_TABLE_PACKET_RESULTS complete PA MultiPacketRespRecords */
};

static const char* const pa_sa_status_text[] =
{
    "Success",  // not used by code below
	"PA: Insufficient Resources",             //  0x0100 NO_RESOURCES
	"PA: Invalid Request",                    //  0x0200 REQ_INVALID
	"PA: No Records",                         //  0x0300 NO_RECORDS
	"PA: Too Many Records",                   //  0x0400 TOO_MANY_RECORDS
	"PA: Invalid GID in Request",             //  0x0500 REQ_INVALID_GID
	"PA: Insufficient Components in Request", //  0x0600 REQ_INSUFFICIENT_COMPONENT
};
static const char* const pa_status_text[] =
{
    "Success",  // not used by code below
    "PA: PM Engine Unavailable",			//0x0A00 - Engine unavailable
    "PA: Group Not Found",  				//0x0B00 - No such group
    "PA: Port Not Found", 					//0x0C00 - Port not found
    "PA: Virtual Fabric Not Found", 		//0x0D00 - VF not found
    "PA: Invalid Parameter",  				//0x0E00 - Invalid parameter
    "PA: Image Not Found",  				//0x0F00 - Image not found
};

/**************************************** 
 * Prototypes
 ****************************************/


/******************************************************** 
 * Local file functions
 *******************************************************/
/** 
 *  Set RMPP port id structure.
 * 
 * @param port              The port from which we access the fabric.
 * @param portid            The destination port id structure to fill.
 *
 * @return 
 *   FSUCCESS - Query successful
 */
static FSTATUS 
oib_set_rmpp_port_id(
    struct oib_port     *port,
    struct dest_portid  *portid
    )
{
    uint64_t guid = oib_get_port_guid(port);
    uint64_t prefix = oib_get_port_prefix(port);

    if (portid->gid.global.subnet_prefix == 0ULL)
        portid->gid.global.subnet_prefix = htonll(prefix);
    if (guid)
        portid->gid.global.interface_id = htonll(guid);

    portid->lid = port->primary_pm_lid;
    portid->sl = port->primary_pm_sl;
    int idx = oib_find_pkey(port, 0xffff);
    if (idx<0) return FPROTECTION; 
    portid->pkey_idx = idx;
    return FSUCCESS;
}

/** 
 *  Get the data part of a SA response, not a generic RMPP packet data.
 * 
 * @param buffer            A SA umad Response data (umad header + std MAD header + 
 *                          RMPP header + SA hdr (SM_Key + AttributeOffset +
 *                          + reserved + ComponentMask) + data). 
 *
 * @return 
 *   None.
 */
static uint8_t * 
oib_get_rmpp_mad_data(
    uint8_t     *buffer
    )
{
    if (buffer) 
    {
        uint8_t *mad = NULL;

        mad = umad_get_mad(buffer);
        return (mad) ? mad + offsetof(struct umad_sa_packet, data) : NULL;
    }

    return NULL;
}

/** 
 *  Dump the umad header of a response to the screen.
 * 
 * @param resMad            MAD Response data (umad header + MAD header + data). 
 *
 * @return 
 *   None.
 */
static void 
oib_dump_rmpp_response(
    uint8_t     *rspMad
    )
{
    struct umad_hdr *mad;

    mad = (struct umad_hdr *)umad_get_mad(rspMad);
    DBGPRINT("Response MAD \n");
    DBGPRINT("\tmclass: 0x%x\n", mad->mgmt_class);
    DBGPRINT("\tmethod: 0x%x\n", mad->method);
    DBGPRINT("\taid: 0x%x\n", ntohs(mad->attr_id));
    DBGPRINT("\tamod: 0x%x\n", ntohl(mad->attr_mod));
}

/** 
 *  Send a MAD RMPP packet.
 * 
 * @param port_id              The port from which we access the
 *                           fabric.
 * @param portid            The destination port id structure to fill.
 *
 * @return 
 *   FSUCCESS - Query successful
 */
static int
oib_do_mad_rmpp(
    struct oib_port     *port,
    void                *sndbuf,
    int                 snddatalen,
    void                **rcvbuf,
    int                 rcvbuflen,
    int                 agentid,
    int                 timeout
    )
{
    uint32_t    trid; // only low 32 bits
    int         retries;
    int         length, status;

    if (!timeout)
        timeout = OIB_UTILS_DEF_TIMEOUT_MS;

    if (port->pa_verbose) 
    {
        DBGPRINT(">>> sending: agentid=%d len=%d pktsz=%zu\n", agentid, snddatalen, umad_size() + snddatalen);
        umad_dump(sndbuf);
        oib_dump_mad(port->verbose_file, umad_get_mad(sndbuf), snddatalen, "send mad\n");
    }

    trid = ntohll(((struct umad_hdr*)umad_get_mad(sndbuf))->tid);

    for (retries = 0; retries < (OIB_UTILS_DEF_RETRIES * 3); retries++) 
    {
        // Extend retries x3
        if (retries) 
        {
            if (port->pa_verbose) 
                fprintf(port->verbose_file, "Error, retry %d (timeout %d ms)\n", retries, timeout);
        }

        if (umad_send(port->umad_fd, agentid, sndbuf, snddatalen, timeout, 0) < 0) 
        {
            if (port->pa_verbose) OIB_WARN("send failed; %s", strerror(errno));
            return -1;
        }

        // use same timeout on receive side just in case
        // send packet is lost somewhere.
        do 
        {
            length = rcvbuflen;
            if (umad_recv(port->umad_fd, *rcvbuf, &length, timeout) < 0) 
            {
                if (length <= OIB_UTILS_MAD_SIZE) 
                {
                    // no MAD returned.  None available.
                    if (port->pa_verbose) 
                        fprintf(port->verbose_file,"oib_do_mad_rmpp: recv error on MAD sized umad\n");
                    return (-1);
                } 
                else 
                {
                    /* Need a larger buffer for RMPP */
                    umad_free(*rcvbuf);
                    *rcvbuf = umad_alloc(1, length + umad_size());
                    if (*rcvbuf == NULL) 
                    {
                        if (port->pa_verbose) 
                            fprintf(port->verbose_file, "Error, unable to reallocate memory for RMPP receive buffer!\n");
                        return (-1);
                    }
                    if (umad_recv(port->umad_fd, *rcvbuf, &length, timeout) < 0) 
                    {
                        if (port->pa_verbose) 
                            fprintf(port->verbose_file,"Error, receive error on RMPP receive retry!\n");
                        return (-1);
                    }
                }
            } // End of do

            if (port->pa_verbose) 
            {
                OIB_WARN("rcv buf:");
                umad_dump(*rcvbuf);
                oib_dump_mad(port->verbose_file, umad_get_mad(*rcvbuf), length, "rcv mad\n");
            }
        } while ((uint32_t)ntohll(((struct umad_hdr*)umad_get_mad(*rcvbuf))->tid) != trid);

        status = umad_status(*rcvbuf);
        if (!status)
            return length;      // done
        if (status == ENOMEM)
            return length;
    }

    if (port->pa_verbose) OIB_WARN("timeout after %d retries, %d ms", retries, timeout * retries);
    return -1;

}	// End of oib_do_mad_rmpp()


#define GET_IB_USERLAND_TID(tid)    (tid & 0x00000000ffffffff)

/** 
 * Generate the 64 bit MAD transaction ID. The upper 32 bits are reserved for
 * use by the kernel. We clear the upper 32 bits here, but MADs received from
 * the kernel may contain kernel specific data in these bits, consequently
 * userland TID matching should only be done on the lower 32 bits.
 * 
 * @param pkt               Pointer to the umad vendor packet start.
 * @param rmpp_md           The pointer to the rmpp_md structure that containing the sending info.  
 * @return 
 *   None.
 */
static uint64_t gen_trid(void)
{
    static uint64_t trid;
    uint64_t        next;
    
    if (!trid) 
    {
        srandom((int)time(0) * getpid());
        trid = random();
    }
    next = ++trid;
    next = GET_IB_USERLAND_TID(next);
    return next;
}

/** 
 *  Build the umad vendor rmpp packet.
 * 
 * @param pkt               Pointer to the umad vendor packet start.
 * @param rmpp_md           The pointer to the rmpp_md structure that containing the sending info.  
 * @return 
 *   None.
 */
static void 
encode_vendor_rmpp_mad(
    struct umad_vendor_packet   *pkt, 
    struct oib_rmpp_md *        rmpp_md, 
    void                        *data
    )
{
    pkt->mad_hdr.base_version = STL_BASE_VERSION;
    pkt->mad_hdr.mgmt_class = rmpp_md->mgtclass;
    pkt->mad_hdr.class_version = rmpp_md->mgtclass_ver;
    pkt->mad_hdr.method = rmpp_md->method;
    
    pkt->mad_hdr.status = htons(rmpp_md->rstatus);
    if (rmpp_md->trid)
        pkt->mad_hdr.tid = htonll(rmpp_md->trid);
    else
        pkt->mad_hdr.tid = htonll(gen_trid());
    
    pkt->mad_hdr.attr_id = htons(rmpp_md->attr_id);
    pkt->mad_hdr.attr_mod = htonl(rmpp_md->attr_mod);
    
    if (data)
        memcpy((uint8_t *)pkt + rmpp_md->dataoffs, data, rmpp_md->data_size);
    
    /* vendor mads range 2 */
    if (rmpp_md->mgtclass >= 0x30 && rmpp_md->mgtclass <= 0x4f)
        memcpy(&pkt->oui, &rmpp_md->oui, 3);
    
    pkt->rmpp_hdr.paylen_newwin = htonl(rmpp_md->data_size);
}

/** 
 *  Build the umad header.
 * 
 * @param umad              Pointer to the umad header.
 * @param dport             The pointer to dest_portid that contains the destination port info. 
 * @return 
 *   None.
 */
static void 
oib_build_rmpp_umad_md(
    void                    *umad, 
    struct dest_portid *    dport
    )
{
    umad_set_addr(umad, dport->lid, dport->qp, dport->sl, dport->qkey);
    if (dport->grh_present) 
    {
        struct ib_mad_addr addr;
        addr.grh_present = 1;
        memcpy(addr.gid, dport->gid.raw, 16);
        addr.hop_limit = 0xff;
        addr.traffic_class = 0;
        addr.flow_label = 0;
        umad_set_grh(umad, &addr);
    } 
    else
        umad_set_grh(umad, 0);
    umad_set_pkey(umad, dport->pkey_idx);
}

/** 
 *  Send a PA MAD request and receive response if applicable.
 * 
 * @param port              The port from which we access the fabric.
 * @param rmpp_md           The pointer to the rmpp_md structure that containing the sending info. 
 * @param dport             The pointer to dest_portid that contains the destination port info. 
 * @param snddata           The data to send.  
 * @param sndbuflen         The send buffer length. 
 * @param rcvbuflen         The address of the length of the receiving buffer. Upon successful 
 *                          return, it should be set with the lenght of received data.
 * @param ppRspMad          The Address of the variable to receive the response buffer. It should be 
 *                          freed by the caller after successful return.
 * @return 
 *   FSUCCESS   - Query successful
 *   Other      - Query failed.
 */
static FSTATUS
oib_vendor_rmpp(
    struct oib_port     *port,
    struct oib_rmpp_md  *rmpp_md,
    struct dest_portid  *dport,
    void                *snddata,
    int                 sndbuflen,
    int                 *rcvbuflen,
    uint8_t             **ppRspMad
    )
{
    FSTATUS     fstatus = FSUCCESS;
    int         status, len;
    int         originalRcvbuflen = 0;
    uint8_t     *sndbuf = NULL, *rcvbuf = NULL, *mad = NULL, *tmpbuf = NULL;
    uint8_t     **rcvbufP;

    rcvbufP = &rcvbuf;

    // allocate buffers used to handle RMPP
    if ((sndbuf = umad_alloc(1, sndbuflen + umad_size())) == NULL) 
    {
        if (port->pa_verbose)
            fprintf(port->verbose_file, "Error, unable to allocate memory for RMPP send buffer!\n");
        return FERROR;
    } else {
        memset(sndbuf, 0, umad_size() + sndbuflen);
    }

    if (*rcvbuflen) 
    {
        if ((rcvbuf = umad_alloc(1, *rcvbuflen + umad_size())) == NULL) 
        {
            if (port->pa_verbose)
                fprintf(port->verbose_file, "Error, unable to allocate memory for RMPP receive buffer!\n");
            fstatus = FERROR;
        } else {
        	memset(rcvbuf, 0, umad_size() + *rcvbuflen);
		}

        if ((tmpbuf = umad_alloc(1, *rcvbuflen + umad_size())) == NULL) 
        {
            if (port->pa_verbose)
                fprintf(port->verbose_file, "Error, unable to allocate memory for RMPP receive work buffer!\n");
            fstatus = FERROR;
        } else {
        	memset(tmpbuf, 0, umad_size() + *rcvbuflen);
		}
        originalRcvbuflen = *rcvbuflen;
    }

    if (!fstatus) 
    {

        // initialize packet to send
        oib_build_rmpp_umad_md(sndbuf, dport);
        encode_vendor_rmpp_mad(umad_get_mad(sndbuf), rmpp_md, snddata);

        // process RMPP request and response
        if ((len = oib_do_mad_rmpp(port, sndbuf, rmpp_md->data_size, (void **)rcvbufP, *rcvbuflen,
                              port->umad_agents[rmpp_md->mgtclass_ver][rmpp_md->mgtclass], rmpp_md->timeout)) < 0)
        {
            fstatus = FERROR;
        }

        // check response packet
        if (fstatus == FSUCCESS) 
        {
            // Check the response MAD header beyond the umad header
            mad = umad_get_mad(rcvbuf);
            if ((status = ntohs(((struct umad_hdr *)mad)->status)) != 0) 
            {
                // Remember the MAD status for later query
                port->mad_status = status;

#ifdef OPEN_SOURCE_12200
				DBGPRINT("Error, MAD completed with error status 0x%x: %s!\n", status, "unexpected PA status");
				fstatus = FERROR;
#else
                    DBGPRINT("Error, MAD completed with error status 0x%x: %s!\n", status, iba_pa_mad_status_msg(port));

                    switch (status) 
                    {
					case STL_MAD_STATUS_STL_PA_NO_GROUP:
					case STL_MAD_STATUS_STL_PA_NO_PORT:
					case STL_MAD_STATUS_STL_PA_NO_VF:
						fstatus = FNOT_FOUND;
						break;
					case STL_MAD_STATUS_STL_PA_INVALID_PARAMETER:
						fstatus = FINVALID_PARAMETER;
						break;
					case STL_MAD_STATUS_STL_PA_UNAVAILABLE:
						fstatus = FINVALID_STATE;
					default:
						fstatus = FERROR;
						break;
                    }
#endif
            }  // End of if the response MAD status is error (not 0).
            else 
            {
                // Now we got the correct response
                uint8_t                 version;
                struct umad_rmpp_packet *rmpp_pkt = (struct umad_rmpp_packet *)mad;
                if ((rmpp_pkt->rmpp_hdr.rmpp_rtime_flags & 0x03) &&
                    (version = rmpp_pkt->rmpp_hdr.rmpp_version) != 1) 
                {
                    if (port->pa_verbose)
                        fprintf(port->verbose_file, "Warning, bad rmpp version %d!\n", version);
                    // for now, ignore differences between OFED RMPP versions 
                    //fstatus = FERROR;
                }

                // set pointer to response MAD packet
                if (!fstatus && rcvbuf) 
                {
                    uint8_t *ptr, *ptrEnd;
                    int     dataIx = 0, madDataIx = 0;
                    
                    *ppRspMad = rcvbuf;
                    // for large data respones that required multiple MAD packets, OFED
                    // inserts a 16 byte tag header between each MAD packet.  we must
                    // stripe out this header and re-package the data.
                    
                    // get pointer to payload of the MAD packet
                    ptr = oib_get_rmpp_mad_data(rcvbuf);
                    // get pointer to the end of the MAD packet
                    ptrEnd = rcvbuf + (umad_size() + len);

                    if (len > originalRcvbuflen) 
                    {
                        umad_free(tmpbuf);
                        tmpbuf = umad_alloc(1, len + umad_size());
                        if (tmpbuf == NULL) 
                        {
                            if (port->pa_verbose)
                                fprintf(port->verbose_file, "Error, unable to reallocate memory for RMPP receive work buffer!\n");
                            // Free the resources
                            fstatus = FERROR;
                            goto VRMPP_EXIT;
                        }
                    }

                    if (tmpbuf == NULL) 
                    {
                        if (port->pa_verbose)
                            fprintf(port->verbose_file, "Error, unable to reallocate memory for RMPP receive work buffer!\n");
                        // Free the resources
                        fstatus = FERROR;
                        goto VRMPP_EXIT;
                    } else {
                    	for (; ptr < ptrEnd; ptr++, madDataIx++)
						{
                        	//DBGPRINT("Mad byte[%d] = 0x%x\n", madDataIx, *ptr);
                        	// The tag does not have a constant value, but is at offsets 1992, 4000, 6008, 8016, ...
                        	if ( (madDataIx == RMPP_MAD_TAG_HDR_BASE_OFFSET) || (((madDataIx - RMPP_MAD_TAG_HDR_BASE_OFFSET) % RMPP_MAD_TAG_HDR_OFFSET_INC) == 0)  ) 
                        	{
                            	//DBGPRINT("\tFound tag header byte[%d] = 0x%x\n", madDataIx, *ptr);
                            	ptr += (RMPP_MAD_TAG_HDR_LEN - 1);
                            	madDataIx += (RMPP_MAD_TAG_HDR_LEN - 1);
                        	} 
                        	else 
                        	{
                            	tmpbuf[dataIx++] = *ptr;
                            	//DBGPRINT("\tRec byte[%d] = 0x%x\n", dataIx - 1, tmpbuf[dataIx -1]);
                        	}
                    	}
                    }

                    // update the payload of the MAD packet
                    ptr = oib_get_rmpp_mad_data(rcvbuf);
					len = dataIx + IBA_SUBN_ADM_HDRSIZE;
                    memcpy(ptr, tmpbuf, dataIx);
                }

                // set length related fields to be returned
                *rcvbuflen = len;
                rmpp_md->attr_offset = ntohs(((struct umad_sa_packet *)rmpp_pkt)->attr_offset);
            } // End of else the response MAD status is success (0).
        } // End of if send the request and receive response correctly
    } // End of if we allocate send/rcv buffers successfully

VRMPP_EXIT:
    // free buffers if allocated.
    if (sndbuf != NULL)
        umad_free((void *)sndbuf);
    if (fstatus && rcvbuf != NULL)
        umad_free(rcvbuf);
    if (tmpbuf != NULL)
        umad_free((void *)tmpbuf);

    return fstatus;

}	// End of oib_vendor_rmpp()

/** 
 *  Send a PA MAD request and receive response if applicable.
 * 
 * @param port              The port from which we access the fabric.
 * @param method            The MAD method. 
 * @param attrId            The MAD atribute ID. 
 * @param attrMod           The MAD attribute modifier. 
 * @param snddata           The data to send. 
 * @param snddatalen        The length of the data to send. 
 * @param sndbuflen         The send buffer length. 
 * @param rcvbuflen         The address of the length of the receiving buffer. Upon successful 
 *                          return, it should be set with the lenght of received data.
 * @param ppRspMad          The Address of the variable to receive the response buffer. It should be 
 *                          freed by the caller after successful return.
 * @param recsz             Address of the variable to receive the record size in 
 *                          the data part of the response upon successful return. 
 * @param reccnt            Address of the variable to receive the number of records in 
 *                          the data part of the response upon successful return. 
 *
 * @return 
 *   FSUCCESS   - Query successful
 *   Other      - Query failed.
 */
static FSTATUS
oib_send_pa_mad(
    struct oib_port     *port,
    uint8_t             method,
    uint32_t            attrId,
    uint32_t            attrMod,
    void                *snddata,
    int                 snddatalen,
    int                 sndbuflen,
    int                 *rcvbuflen,
    uint8_t             **ppRspMad,
    uint32_t            *recsz,
    uint32_t            *reccnt
    )
{
    FSTATUS             fstatus = FINVALID_OPERATION;
    struct oib_rmpp_md  rmpp_md = {0};
    struct dest_portid  portid = {0};
    int                 respExpected = (*rcvbuflen) ? 1 : 0;

    // initialize PortId structure
    memset(&portid, 0, sizeof(portid));
    if ((fstatus = oib_set_rmpp_port_id(port, &portid))!=FSUCCESS)
        return fstatus;
    portid.qp = 1;
    if (!portid.qkey)
        portid.qkey = QP1_WELL_KNOWN_Q_KEY;

    DBGPRINT("LID %u pkeyidx %d snddata %p, sndbuflen=%d, rcvbuflen=%d\n",
             portid.lid, portid.pkey_idx, snddata, sndbuflen, *rcvbuflen);
    if (portid.lid <= 0)
        return fstatus; // no direct SMI

    // initialize RPC structure
    memset(&rmpp_md, 0, sizeof(rmpp_md));
    rmpp_md.mgtclass = MCLASS_VFI_PM;
    rmpp_md.mgtclass_ver = STL_PM_CLASS_VERSION;
    rmpp_md.method = method;
    rmpp_md.attr_id = attrId;
    rmpp_md.attr_mod = attrMod;
    rmpp_md.timeout = 0;
    rmpp_md.data_size = snddatalen;
    /* STL PA class (VFI PM) uses same data offset as STL SA class */
    rmpp_md.dataoffs = offsetof(struct umad_sa_packet, data);
    memcpy(&rmpp_md.oui, &ib_truescale_oui, 3);

    DBGPRINT("class 0x%x method 0x%x attr 0x%x mod 0x%x data_size %d off %d res_ex %d\n",
             rmpp_md.mgtclass, rmpp_md.method, rmpp_md.attr_id, rmpp_md.attr_mod,
             rmpp_md.data_size, rmpp_md.dataoffs, respExpected);

    if (respExpected) 
    {
        // send packet and process RMPP response packet
        fstatus = oib_vendor_rmpp(port, &rmpp_md, &portid, snddata, sndbuflen, rcvbuflen, ppRspMad);
        DBGPRINT("Completed oib_vendor_rmpp(): st=%u\n", (unsigned int)fstatus);

        // if no records IBTA 1.2.1 says AttributeOffset should be 0
        // Assume we never get here for a non-RMPP (eg. non-GetTable) response
        if (rmpp_md.attr_offset) 
        {
			*recsz = rmpp_md.attr_offset * sizeof(uint64);
            *reccnt = (int)((*rcvbuflen - IBA_SUBN_ADM_HDRSIZE) / *recsz);
        } 
        else 
        {
            *recsz = 0;
            *reccnt = 0;
        }
        return fstatus;
    }

    return(FSUCCESS);

}	// End of oib_send_pa_mad()

/** 
 *  Send a PA query and get the result.
 * 
 * @param port              The port from which we access the fabric.
 * @param method            PA method identifier.
 * @param attr_id           PA attribute identifier
 * @param attr_mod          PA attribute modifier
 * @param snd_data          Outbound request data.
 * @param snd_data_len      Outbound request data length.
 * @param rcv_buf_len       Max rcv buffer length (required for OFED driver)
 * @param snd_buf_len       Max xmt buffer length
 * @param rsp_mad           Response MAD packet. The caller should free it after use.
 * @param query_result      Query return result (pointer to pointer to the query result). Allocated
 *                          here if successful and the caller must free it.
 *
 * @return 
 *   FSUCCESS - Query successful
 *     FERROR - Error
 */
static FSTATUS 
pa_query_common (
    struct oib_port         *port,
    uint8_t                 method,
    uint32_t                attr_id,
    uint32_t                attr_mod,
    void                    *snd_data,
    int                     snd_data_len,
    int                     *rcv_buf_len,
    int                     snd_buf_len,
    uint8_t                 **rsp_mad,
    PQUERY_RESULT_VALUES    *query_result 
    )
{
    FSTATUS    fstatus = FSUCCESS;
    uint32_t   rec_sz = 0;
    uint32_t   rec_cnt = 0;
    uint32_t   mem_size;

    // do some common stuff for each command / query.
    DBG_ENTER_FUNC;

    DBGPRINT("Request MAD method: 0x%x\n", method);
    DBGPRINT("\taid: 0x%x\n", attr_id);
    DBGPRINT("\tamod: 0x%x\n", attr_mod);

    // default the parameters to failed state.
    *query_result = NULL;
    *rsp_mad= NULL;

    // submit RMPP MAD request
    fstatus = oib_send_pa_mad(port, method, attr_id, attr_mod, snd_data, snd_data_len, snd_buf_len, rcv_buf_len, rsp_mad, &rec_sz, &rec_cnt); 
    if (fstatus != FSUCCESS) 
    {
        DBGPRINT("Query PA failed to send: %u\n",  (unsigned int)fstatus);
        if (!port->mad_status)
			port->mad_status = MAD_STATUS_INVALID_ATTRIB;
        if (fstatus==FPROTECTION) {
            // PKEY lookup error. 
            OUTPUT_ERROR("Query PA failed: requires full management node. Status:(%u)\n",  (unsigned int)fstatus);
        }
        goto done;
    }

    DBGPRINT("Rcv buffer length is %d\n", *rcv_buf_len);
    DBGPRINT("Result count is %d\n", rec_cnt);

    // query result is the size of one of the IBACCESS expected data types.
    // Multiply that size by the count of records we got back.
    // Add to that size, the size of the size of the query response struct.
    mem_size  = rec_sz * rec_cnt;
    mem_size += sizeof (uint32_t); 
    mem_size += sizeof (QUERY_RESULT_VALUES);

    // resultDataSize should be 0 when status is not successful and no data is returned
    *query_result = MemoryAllocate2AndClear(mem_size, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (*query_result == NULL) 
    {
        OUTPUT_ERROR("error allocating query result buffer\n");
        fstatus = FINSUFFICIENT_MEMORY;
        goto done;
    }
    (*query_result)->Status = fstatus;
    (*query_result)->MadStatus = port->mad_status;
    (*query_result)->ResultDataSize = rec_sz * rec_cnt;   
    *((uint32_t*)((*query_result)->QueryResult)) = rec_cnt;

done:
    if (fstatus != FSUCCESS) 
    {
        if (*rsp_mad != NULL) 
        {
            free (*rsp_mad);
            *rsp_mad = NULL;
        }
    }

    DBG_EXIT_FUNC;
    
    return (fstatus);

}	// End of pa_query_common()

static const char* 
iba_pa_query_input_type_msg(
    QUERY_INPUT_TYPE    code
    )
{
    if (code < 0 || code >= (int)(sizeof(pa_query_input_type_text)/sizeof(char*)))
        return "Unknown SD Query Input Type";
    else
        return pa_query_input_type_text[code];
}

const const char* 
iba_pa_query_result_type_msg(
    QUERY_RESULT_TYPE   code
    )
{
    if (code < 0 || code >= (int)(sizeof(pa_query_result_type_text)/sizeof(char*)))
        return "Unknown SD Query Result Type";
    else
        return pa_query_result_type_text[code];
}


/******************************************************* 
 * Public functions
 *******************************************************/
/**
 *   Enable debugging output in the oib_utils library.
 *
 * @param   port    The local port to operate on.
 *  
 * @return 
 *   None
 */
void 
set_opapaquery_debug (
    IN struct oib_port  *port
    )
{
    port->pa_verbose = 1;

    // Should we pass along the debug info
    //_openib_debug_ = 1;     // enable general debugging

    return;
}

/**
 *  Get the classportinfo from the given port.
 *
 * @param port                  Local port to operate on.  
 *  
 * @return 
 *   Valid pointer  -   A pointer to the ClassPortInfo. The caller must free it after use.
 *   NULL           -   Error.
 */
STL_CLASS_PORT_INFO *
iba_pa_classportinfo_response_query(
    IN struct oib_port  *port
    )
{
    FSTATUS                 fstatus = FSUCCESS;
    QUERY_RESULT_VALUES     *query_result = NULL;
    STL_CLASS_PORT_INFO     *response = NULL;
    uint8_t                 *rsp_mad, *data;
    int                     rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                     snd_buf_len = DEF_BUF_DATA_SIZE;
    char                    request_data[PA_REQ_HEADER_SIZE] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_GET, STL_PA_ATTRID_GET_CLASSPORTINFO, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
		oib_free_query_result_buffer(query_result);
        return response;
    } 
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);

    response = MemoryAllocate2AndClear(sizeof(STL_CLASS_PORT_INFO), IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
		oib_free_query_result_buffer(query_result);
        return(NULL);
    }
    memcpy((uint8 *)response, data, min(sizeof(STL_CLASS_PORT_INFO), rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_CLASS_PORT_INFO(response);

    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;

}	// End of iba_pa_classportinfo_response_query()

/**
 *  Get port statistics (counters)
 *
 * @param port                  Local port to operate on. 
 * @param node_lid              Remote node LID.
 * @param port_num              Remote port number. 
 * @param port_counters         Pointer to port counters to fill.
 * @param delta_flag            1 for delta counters, 0 for raw image counters.
 * @param user_ctrs_flag        1 for running counters, 0 for image counters. (delta must be 0)
 * @param image_id              Pointer to image ID of port counters to get. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the counters data. The caller must free it.
 *     NULL         - Error
 */
STL_PORT_COUNTERS_DATA *
iba_pa_single_mad_port_counters_response_query(
    IN struct oib_port  *port,
    IN uint32_t          node_lid,
    IN uint8_t           port_number,
    IN uint32_t          delta_flag,
    IN uint32_t          user_cntrs_flag,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    )
{
    FSTATUS                 fstatus = FSUCCESS;
    QUERY_RESULT_VALUES     *query_result = NULL;
    STL_PORT_COUNTERS_DATA  *response = NULL;
    STL_PORT_COUNTERS_DATA  *p;
    uint8_t                 *rsp_mad, *data;
    int                     rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                     snd_buf_len = DEF_BUF_DATA_SIZE;
    char                    request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PORT_COUNTERS_DATA)] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    p = (STL_PORT_COUNTERS_DATA *)request_data;
    p->nodeLid = node_lid;
    p->portNumber = port_number;
    p->flags = (delta_flag ? STL_PA_PC_FLAG_DELTA : 0) |
			   (user_cntrs_flag ? STL_PA_PC_FLAG_USER_COUNTERS : 0);

    p->imageId.imageNumber = image_id->imageNumber;
    p->imageId.imageOffset = image_id->imageOffset;
	p->imageId.imageTime.absoluteTime = image_id->imageTime.absoluteTime;
	memset(p->reserved, 0, sizeof(p->reserved));
	memset(p->reserved2, 0, sizeof(p->reserved2));
	p->lq.s.reserved = 0;
    BSWAP_STL_PA_PORT_COUNTERS(p);

    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_GET, STL_PA_ATTRID_GET_PORT_CTRS, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
	oib_free_query_result_buffer(query_result);
        return response;
    } 
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);
    
    response = MemoryAllocate2AndClear(STL_PA_PORT_COUNTERS_NSIZE, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
        oib_free_query_result_buffer(query_result);
        return (NULL);
    }
    memcpy((uint8 *)response, data, min(STL_PA_PORT_COUNTERS_NSIZE, rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_PA_PORT_COUNTERS(response);

    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;
}

/**
 *  Clear port statistics (counters)
 *
 * @param port                  Local Port to operate on. 
 * @param node_lid              Remote node LID.
 * @param port_num              Remote port number. 
 * @param select                Port's counters to clear. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the clear port counters data (containing the
 *                    info as which counters have been cleared).  The caller must free it.
 *     NULL         - Error
 */
STL_CLR_PORT_COUNTERS_DATA *
iba_pa_single_mad_clr_port_counters_response_query(
    IN struct oib_port  *port,
    IN uint32_t          node_lid,
    IN uint8_t           port_number,
    IN uint32_t          select
    )
{
    FSTATUS                 fstatus = FSUCCESS;
    QUERY_RESULT_VALUES     *query_result = NULL;
    STL_CLR_PORT_COUNTERS_DATA  *response = NULL;
    STL_CLR_PORT_COUNTERS_DATA  *p;
    uint8_t                 *rsp_mad, *data;
    int                     rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                     snd_buf_len = DEF_BUF_DATA_SIZE;
    char                    request_data[PA_REQ_HEADER_SIZE + sizeof(STL_CLR_PORT_COUNTERS_DATA)] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    p = (STL_CLR_PORT_COUNTERS_DATA *)request_data;
    p->NodeLid = node_lid;
    p->PortNumber = port_number;
    p->CounterSelectMask.AsReg32 = select;
	p->CounterSelectMask.s.Reserved = 0;
	memset(p->Reserved, 0, sizeof(p->Reserved));
    BSWAP_STL_PA_CLR_PORT_COUNTERS(p);

    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_CLR_PORT_CTRS, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
	oib_free_query_result_buffer(query_result);
        return response;
    } 
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);
    
    response = MemoryAllocate2AndClear(STL_PA_CLR_PORT_COUNTERS_NSIZE, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
        oib_free_query_result_buffer(query_result);
        return (NULL);
    }
    memcpy((uint8 *)response, data, min(STL_PA_CLR_PORT_COUNTERS_NSIZE, rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_PA_CLR_PORT_COUNTERS(response);


    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;
}

/**
 *  Clear all ports, statistics (counters)
 *
 * @param port                  Local port to operate on. 
 * @param select                Port's counters to clear. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the clear all port counters data (containing the
 *                    info as which counters have been cleared).  The caller must free it.
 *     NULL         - Error
 */
STL_CLR_ALL_PORT_COUNTERS_DATA *
iba_pa_single_mad_clr_all_port_counters_response_query(
    IN struct oib_port  *port,
    IN uint32_t          select
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_CLR_ALL_PORT_COUNTERS_DATA  *response = NULL;
    STL_CLR_ALL_PORT_COUNTERS_DATA  *p;
    uint8_t                     *rsp_mad, *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
    char                        request_data[PA_REQ_HEADER_SIZE + sizeof(STL_CLR_ALL_PORT_COUNTERS_DATA)] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    p = (STL_CLR_ALL_PORT_COUNTERS_DATA *)request_data;
    p->CounterSelectMask.AsReg32 = select;
	p->CounterSelectMask.s.Reserved = 0;
	BSWAP_STL_PA_CLR_ALL_PORT_COUNTERS(p);

    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_CLR_ALL_PORT_CTRS, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
	oib_free_query_result_buffer(query_result);
        return response;
    } 
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);
    
    response = MemoryAllocate2AndClear(STL_PA_CLR_ALL_PORT_COUNTERS_NSIZE, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
        oib_free_query_result_buffer(query_result);
        return (NULL);
    }
    memcpy((uint8 *)response, data, min(STL_PA_CLR_ALL_PORT_COUNTERS_NSIZE, rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_PA_CLR_ALL_PORT_COUNTERS(response);

    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;
}

/**
 *  Get the PM config data.
 *
 * @param port                  Local port to operate on. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for pm config data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_PM_CFG_DATA *
iba_pa_single_mad_get_pm_config_response_query(
    IN struct oib_port  *port
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_PA_PM_CFG_DATA       	*response = NULL;
    uint8_t                     *rsp_mad, *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
    char                        request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_PM_CFG_DATA)] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_GET, STL_PA_ATTRID_GET_PM_CONFIG, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
	oib_free_query_result_buffer(query_result);
        return response;
    } 
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);
    
    response = MemoryAllocate2AndClear(STL_PA_PM_CONFIG_NSIZE, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
        oib_free_query_result_buffer(query_result);
        return (NULL);
    }
    memcpy((uint8 *)response, data, min(STL_PA_PM_CONFIG_NSIZE, rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_PA_PM_CFG(response);

    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;
}

/**
 *  Free a sweep image.
 *
 * @param port                  Local port to operate on. 
 * @param image_id              ID of image to freeze. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the frozen image id data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_ID_DATA *
iba_pa_single_mad_freeze_image_response_query(
    IN struct oib_port  *port,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_PA_IMAGE_ID_DATA        *response = NULL;
    STL_PA_IMAGE_ID_DATA        *p;
    uint8_t                     *rsp_mad, *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
    char                        request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_IMAGE_ID_DATA)] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    p = (STL_PA_IMAGE_ID_DATA *)request_data;
    p->imageNumber = hton64(image_id->imageNumber);
    p->imageOffset = hton32(image_id->imageOffset);
	p->imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);
    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_FREEZE_IMAGE, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
	oib_free_query_result_buffer(query_result);
        return response;
    } 
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);

    response = MemoryAllocate2AndClear(STL_PA_IMAGE_ID_NSIZE, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
        oib_free_query_result_buffer(query_result);
        return (NULL);
    }
    memcpy((uint8 *)response, data, min(STL_PA_IMAGE_ID_NSIZE, rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_PA_IMAGE_ID(response);

    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;
}

/**
 *  Release a sweep image.
 *
 * @param port                  Local port to operate on. 
 * @param image_id              Ponter to id of image to release. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the released image id data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_ID_DATA *
iba_pa_single_mad_release_image_response_query(
    IN struct oib_port  *port,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_PA_IMAGE_ID_DATA        *response = NULL;
    STL_PA_IMAGE_ID_DATA        *p;
    uint8_t                     *rsp_mad, *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
    char                        request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_IMAGE_ID_DATA)] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    p = (STL_PA_IMAGE_ID_DATA *)request_data;
    p->imageNumber = hton64(image_id->imageNumber);
    p->imageOffset = hton32(image_id->imageOffset);
	p->imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);

    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_RELEASE_IMAGE, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
	oib_free_query_result_buffer(query_result);
        return response;
    } 
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);

    response = MemoryAllocate2AndClear(STL_PA_IMAGE_ID_NSIZE, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
        oib_free_query_result_buffer(query_result);
        return (NULL);
    }
    memcpy((uint8 *)response, data, min(STL_PA_IMAGE_ID_NSIZE, rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_PA_IMAGE_ID(response);

    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;
}

/**
 *  Renew a sweep image.
 *
 * @param port                  Local port to operate on. 
 * @param image_id              Ponter to id of image to renew. 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the renewed image id data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_ID_DATA *
iba_pa_single_mad_renew_image_response_query(
    IN struct oib_port  *port,
    IN STL_PA_IMAGE_ID_DATA    *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_PA_IMAGE_ID_DATA        *response = NULL;
    STL_PA_IMAGE_ID_DATA        *p;
    uint8_t                     *rsp_mad, *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
    char                        request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_IMAGE_ID_DATA)] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    p = (STL_PA_IMAGE_ID_DATA *)request_data;
    p->imageNumber = hton64(image_id->imageNumber);
    p->imageOffset = hton32(image_id->imageOffset);
	p->imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);

    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_RENEW_IMAGE, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
	oib_free_query_result_buffer(query_result);
        return response;
    } 
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);

    response = MemoryAllocate2AndClear(STL_PA_IMAGE_ID_NSIZE, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
        oib_free_query_result_buffer(query_result);
        return (NULL);
    }
    memcpy((uint8 *)response, data, min(STL_PA_IMAGE_ID_NSIZE, rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_PA_IMAGE_ID(response);

    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;
}

/**
 *  Move a frozen image 1 to image 2.
 *
 * @param port                  Local port to operate on. 
 * @param move_info             Ponter to move info (src and dest image ID). 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the image  move info.  The caller must free it.
 *     NULL         - Error
 */
STL_MOVE_FREEZE_DATA *
iba_pa_single_mad_move_freeze_response_query(
    IN struct oib_port  *port,
    IN STL_MOVE_FREEZE_DATA *move_info
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_MOVE_FREEZE_DATA        *response = NULL;
    STL_MOVE_FREEZE_DATA        *p;
    uint8_t                     *rsp_mad, *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
    char                        request_data[PA_REQ_HEADER_SIZE + sizeof(STL_MOVE_FREEZE_DATA)] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    p = (STL_MOVE_FREEZE_DATA *)request_data;
    p->oldFreezeImage.imageNumber = hton64(move_info->oldFreezeImage.imageNumber);
    p->oldFreezeImage.imageOffset = hton32(move_info->oldFreezeImage.imageOffset);
    p->newFreezeImage.imageNumber = hton64(move_info->newFreezeImage.imageNumber);
    p->newFreezeImage.imageOffset = hton32(move_info->newFreezeImage.imageOffset);

    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_MOVE_FREEZE_FRAME, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
	oib_free_query_result_buffer(query_result);
        return response;
    } 
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);

    response = MemoryAllocate2AndClear(STL_PA_MOVE_FREEZE_NSIZE, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
        oib_free_query_result_buffer(query_result);
        return (NULL);
    }
    memcpy((uint8 *)response, data, min(STL_PA_MOVE_FREEZE_NSIZE, rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_PA_MOVE_FREEZE(response);

    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;
}

/**
 *  Get image info
 *
 * @param port                  Local port to operate on. 
 * @param image_info            Ponter to image info (containing valid image ID). 
 *
 * @return 
 *   Valid Pointer  - A pointer to the memory for the image info data.  The caller must free it.
 *     NULL         - Error
 */
STL_PA_IMAGE_INFO_DATA *
iba_pa_multi_mad_get_image_info_response_query (
    IN struct oib_port  *port,
    IN STL_PA_IMAGE_INFO_DATA  *image_info
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    STL_PA_IMAGE_INFO_DATA      *response = NULL;
    STL_PA_IMAGE_INFO_DATA      *p;
    uint8_t                     *rsp_mad, *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
    char                        request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_IMAGE_INFO_DATA)] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    p = (STL_PA_IMAGE_INFO_DATA *)request_data;
    p->imageId.imageNumber = hton64(image_info->imageId.imageNumber);
    p->imageId.imageOffset = hton32(image_info->imageId.imageOffset);
	p->imageId.imageTime.absoluteTime = hton32(image_info->imageId.imageTime.absoluteTime);
	p->reserved = 0;

    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_IMAGE_INFO, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
	// TODO JW WHAT THIS TEST? IT IS CAUSING FAILURE 
#if 0
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
	oib_free_query_result_buffer(query_result);
        return response;
    } 
#endif
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);

    response = MemoryAllocate2AndClear(STL_PA_IMAGE_INFO_NSIZE, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
        oib_free_query_result_buffer(query_result);
        return (NULL);
    }
    memcpy((uint8 *)response, data, min(STL_PA_IMAGE_INFO_NSIZE, rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_PA_IMAGE_INFO(response);

    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;
}

/**
 *  Get multi-record response for pa group data (group list).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param pquery_result             Pointer to query result to be filled. The caller 
 *                                  has to free the buffer. 
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_group_list_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    uint8_t                     *rsp_mad = NULL;
    uint8_t                     *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
    char                        request_data[PA_REQ_HEADER_SIZE] = {0};
    STL_PA_GROUP_LIST     		*pa_data;
    STL_PA_GROUP_LIST_RESULTS   *pa_result;

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return (FERROR);
    }

    DBGPRINT("Query: Port=0x%"PRIx64" Input=%s, Output=%s\n",
             oib_get_port_guid(port),
             iba_pa_query_input_type_msg(query->InputType),
             iba_pa_query_result_type_msg(query->OutputType));

    // optional query parameters are not supported in Open IB.
    if (query_control_parameters != NULL) 
    {
        printf("Optional Query Control paramerers are not supported in OPENIB\n");
        return (FERROR);
    }

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return (FERROR);
    }
    
    // process the command.
    switch (query->OutputType) 
    {
        case OutputTypePaTableRecord:       
        {
            DBGPRINT("Sending Get Multi Record request\n");

            // submit request
            fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_GRP_LIST, 0,
                                      request_data, sizeof(request_data),
                                      &rcv_buf_len, snd_buf_len,
                                      &rsp_mad, &query_result); 
            if (fstatus == FSUCCESS) 
            {
                if (port->pa_verbose) 
                {
                    DBGPRINT("Completed request: OK\n");
                    oib_dump_rmpp_response(rsp_mad);
                }
                data = oib_get_rmpp_mad_data(rsp_mad);

                // translate the data.
				pa_result = (STL_PA_GROUP_LIST_RESULTS *)query_result->QueryResult;
				pa_data = pa_result->GroupListRecords;
				if (pa_result->NumGroupListRecords) {
					memcpy(pa_data, data, rcv_buf_len - IB_SA_DATA_OFFS);
				}
			} else {
                if (port->pa_verbose)
                    DBGPRINT("Completed request: FAILED\n");
            }
        }
        break;

    default:
        OUTPUT_ERROR("Query Not supported in OPEN IB: Input=%s, Output=%s\n",
                iba_pa_query_input_type_msg(query->InputType),
                iba_pa_query_result_type_msg(query->OutputType));
        fstatus = FERROR; 
        goto done;
        break;
    }

done:
    if (rsp_mad!=NULL) 
    {
        free (rsp_mad);
        rsp_mad = NULL;
    }
    
    *pquery_result = query_result;

    DBG_EXIT_FUNC;
    return fstatus;
}

/**
 *  Get multi-record response for pa group info (stats).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param group_name                Group name 
 * @param pquery_result             Pointer to query result to be filled. The caller has to 
 *                                  to free the buffer. 
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 * @param image_id                  Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_group_stats_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *group_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
	IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    uint8_t                     *rsp_mad = NULL;
    uint8_t                     *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
    char                        request_data[PA_REQ_HEADER_SIZE + 80] = {0}; // 80 = GroupName + ImageId (Request)
    STL_PA_PM_GROUP_INFO_DATA   *pa_data;
    STL_PA_PM_GROUP_INFO_DATA   *p;
    STL_PA_GROUP_INFO_RESULTS   *pa_result;

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return (FERROR);
    }

    DBGPRINT("Query: Port=0x%"PRIx64" Input=%s, Output=%s\n",
             oib_get_port_guid(port),
             iba_pa_query_input_type_msg(query->InputType),
             iba_pa_query_result_type_msg(query->OutputType));

    // optional query parameters are not supported in Open IB.
    if (query_control_parameters != NULL) 
    {
        printf("Optional Query Control paramerers are not supported in OPENIB\n");
        return (FERROR);
    }

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return (FERROR);
    }
    
    p = (STL_PA_PM_GROUP_INFO_DATA *)request_data;
    snprintf(p->groupName, sizeof(p->groupName), "%s", group_name);
    p->imageId.imageNumber = image_id->imageNumber;
    p->imageId.imageOffset = image_id->imageOffset;
	p->imageId.imageTime.absoluteTime = image_id->imageTime.absoluteTime;
	BSWAP_STL_PA_PM_GROUP_INFO(p, 1);
    
    // process the command.
    switch (query->OutputType) 
    {
        case OutputTypePaTableRecord:       
        {
            DBGPRINT("Sending Get Multi Record request\n");

            // submit request
            fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_GRP_INFO, 0,
                                      request_data, sizeof(request_data),
                                      &rcv_buf_len, snd_buf_len,
                                      &rsp_mad, &query_result); 
            if (fstatus == FSUCCESS) 
            {
                if (port->pa_verbose) 
                {
                    DBGPRINT("Completed request: OK\n");
                    oib_dump_rmpp_response(rsp_mad);
                }
                data = oib_get_rmpp_mad_data(rsp_mad);
                if (port->pa_verbose) 
                {
                    DBGPRINT("Response Data:\n");
                    oib_xdump(stderr, data, rcv_buf_len - IB_SA_DATA_OFFS, 8);
                }

                // translate the data.
                pa_result = (STL_PA_GROUP_INFO_RESULTS*)query_result->QueryResult;
                pa_data = pa_result->GroupInfoRecords;
                memcpy(pa_data, data, min(STL_PA_GROUP_INFO_NSIZE * pa_result->NumGroupInfoRecords, rcv_buf_len - IB_SA_DATA_OFFS));
                BSWAP_STL_PA_PM_GROUP_INFO(pa_data, 0);
            } 
            else 
            {
                if (port->pa_verbose)
                    DBGPRINT("Completed request: FAILED\n");
            }
        }
        break;

    default:
        OUTPUT_ERROR("Query Not supported in OPEN IB: Input=%s, Output=%s\n",
                iba_pa_query_input_type_msg(query->InputType),
                iba_pa_query_result_type_msg(query->OutputType));
        fstatus = FERROR; 
        goto done;
        break;
    }

done:
    if (rsp_mad!=NULL) 
    {
        free (rsp_mad);
        rsp_mad = NULL;
    }
    
    *pquery_result = query_result;

    DBG_EXIT_FUNC;
    return fstatus;
}

/**
 *  Get multi-record response for pa group config.
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param group_name                Group name 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to free the buffer. 
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 * @param imageID                   Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_group_config_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *group_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    uint8_t                     *rsp_mad = NULL;
    uint8_t                     *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
	int							i;
    char                        request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_PM_GROUP_CFG_REQ)] = {0};
    STL_PA_PM_GROUP_CFG_RSP     *pa_data;
    STL_PA_PM_GROUP_CFG_REQ     *p;
    STL_PA_GROUP_CONFIG_RESULTS *pa_result;

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return (FERROR);
    }

    DBGPRINT("Query: Port=0x%"PRIx64" Input=%s, Output=%s\n",
             oib_get_port_guid(port),
             iba_pa_query_input_type_msg(query->InputType),
             iba_pa_query_result_type_msg(query->OutputType));

    // optional query parameters are not supported in Open IB.
    if (query_control_parameters != NULL) 
    {
        printf("Optional Query Control paramerers are not supported in OPENIB\n");
        return (FERROR);
    }

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return (FERROR);
    }
    
    p = (STL_PA_PM_GROUP_CFG_REQ *)request_data;
    snprintf(p->groupName, sizeof(p->groupName), "%s", group_name);

    p->imageId.imageNumber = hton64(image_id->imageNumber);
    p->imageId.imageOffset = hton32(image_id->imageOffset);
	p->imageId.imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);
    
    // process the command.
    switch (query->OutputType) 
    {
        case OutputTypePaTableRecord:       
        {
            DBGPRINT("Sending Get Multi Record request\n");

            // submit request
            fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_GRP_CFG, 0,
                                      request_data, sizeof(request_data),
                                      &rcv_buf_len, snd_buf_len,
                                      &rsp_mad, &query_result); 
            if (fstatus == FSUCCESS) 
            {
                if (port->pa_verbose) 
                {
                    DBGPRINT("Completed request: OK\n");
                    oib_dump_rmpp_response(rsp_mad);
                }
                data = oib_get_rmpp_mad_data(rsp_mad);

                // translate the data.
                pa_result = (STL_PA_GROUP_CONFIG_RESULTS*)query_result->QueryResult;
                pa_data = pa_result->GroupConfigRecords;
                if (pa_result->NumGroupConfigRecords) 
                {
					memcpy(pa_data, data, rcv_buf_len - IB_SA_DATA_OFFS);
					for (i = 0; i < pa_result->NumGroupConfigRecords; i++)
                    	BSWAP_STL_PA_GROUP_CONFIG_RSP(&pa_data[i]);
                }
            } 
            else 
            {
                if (port->pa_verbose)
                    DBGPRINT("Completed request: FAILED\n");
            }
        }
        break;

    default:
        OUTPUT_ERROR("Query Not supported in OPEN IB: Input=%s, Output=%s\n",
                iba_pa_query_input_type_msg(query->InputType),
                iba_pa_query_result_type_msg(query->OutputType));
        fstatus = FERROR; 
        goto done;
        break;
    }

done:
    if (rsp_mad!=NULL) 
    {
        free (rsp_mad);
        rsp_mad = NULL;
    }
    
    *pquery_result = query_result;

    DBG_EXIT_FUNC;
    return fstatus;

}	// End of iba_pa_multi_mad_group_config_response_query()

/**
 *  Get multi-record response for pa group focus portlist.
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param group_name                Group name 
 * @param select                    Select value for focus portlist. 
 * @param start                     Start index value of portlist 
 * @param range                     Index range of portlist. 
 * @param pquery_result             Pointer to query result to be filled. The caller has 
 *                                  to fill the buffer. 
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 * @param imageID                   Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_focus_ports_response_query (
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *group_name,
    IN uint32                       select,
    IN uint32                       start,
    IN uint32                       range,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    uint8_t                     *rsp_mad = NULL;
    uint8_t                     *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
    int                         i;
    char                        request_data[PA_REQ_HEADER_SIZE + sizeof(STL_FOCUS_PORTS_REQ)] = {0};
    STL_FOCUS_PORTS_RSP         *pa_data;
    STL_FOCUS_PORTS_REQ         *p;
    STL_PA_FOCUS_PORTS_RESULTS  *pa_result;

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return (FERROR);
    }

    DBGPRINT("Query: Port=0x%"PRIx64" Input=%s, Output=%s\n",
             oib_get_port_guid(port),
             iba_pa_query_input_type_msg(query->InputType),
             iba_pa_query_result_type_msg(query->OutputType));

    // optional query parameters are not supported in Open IB.
    if (query_control_parameters != NULL) 
    {
        printf("Optional Query Control paramerers are not supported in OPENIB\n");
        return (FERROR);
    }

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return (FERROR);
    }
    
    p = (STL_FOCUS_PORTS_REQ *)request_data;
    snprintf(p->groupName, sizeof(p->groupName), "%s", group_name);
    p->select = hton32(select);
    p->start = hton32(start);
    p->range = hton32(range);

    p->imageId.imageNumber = hton64(image_id->imageNumber);
    p->imageId.imageOffset = hton32(image_id->imageOffset);
	p->imageId.imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);

    // process the command.
    switch (query->OutputType) 
    {
        case OutputTypePaTableRecord:       
        {
            DBGPRINT("Sending Get Multi Record request\n");

            // submit request
            fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_FOCUS_PORTS, 0,
                                      request_data, sizeof(request_data),
                                      &rcv_buf_len, snd_buf_len,
                                      &rsp_mad, &query_result); 
            if (fstatus == FSUCCESS) 
            {
                if (port->pa_verbose) 
                {
                    DBGPRINT("Completed request: OK\n");
                    oib_dump_rmpp_response(rsp_mad);
                }
                data = oib_get_rmpp_mad_data(rsp_mad);
                // translate the data.
                pa_result = (STL_PA_FOCUS_PORTS_RESULTS *)query_result->QueryResult;
                pa_data = pa_result->FocusPortsRecords;
                if (pa_result->NumFocusPortsRecords) 
                {
					memcpy(pa_data, data, rcv_buf_len - IB_SA_DATA_OFFS);
					for (i = 0; i < pa_result->NumFocusPortsRecords; i++)
                    	BSWAP_STL_PA_FOCUS_PORTS_RSP(&pa_data[i]);
                }
            } 
            else 
            {
                if (port->pa_verbose)
                    DBGPRINT("Completed request: FAILED\n");
            }
        }
        break;

    default:
        OUTPUT_ERROR("Query Not supported in OPEN IB: Input=%s, Output=%s\n",
                iba_pa_query_input_type_msg(query->InputType),
                iba_pa_query_result_type_msg(query->OutputType));
        fstatus = FERROR; 
        goto done;
        break;
    }

done:
    if (rsp_mad!=NULL) 
    {
        free (rsp_mad);
        rsp_mad = NULL;
    }
    
    *pquery_result = query_result;

    DBG_EXIT_FUNC;
    return fstatus;
}

/**
 *  Get multi-record response for pa vf data (vf list).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param pquery_result             Pointer to query result to be filled. The caller 
 *                                  has to free the buffer. 
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_vf_list_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    uint8_t                     *rsp_mad = NULL;
    uint8_t                     *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
	char                        request_data[PA_REQ_HEADER_SIZE] = {0};
    STL_PA_VF_LIST	        	*pa_data;
    STL_PA_VF_LIST_RESULTS		*pa_result;

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return (FERROR);
    }

    DBGPRINT("Query: Port=0x%"PRIx64" Input=%s, Output=%s\n",
             oib_get_port_guid(port),
             iba_pa_query_input_type_msg(query->InputType),
             iba_pa_query_result_type_msg(query->OutputType));

    // optional query parameters are not supported in Open IB.
    if (query_control_parameters != NULL) 
    {
        printf("Optional Query Control paramerers are not supported in OPENIB\n");
        return (FERROR);
    }

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return (FERROR);
    }
    
    // process the command.
    switch (query->OutputType) 
    {
        case OutputTypePaTableRecord:       
        {
            DBGPRINT("Sending Get Multi Record request\n");

            // submit request
            fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_VF_LIST, 0,
                                      request_data, sizeof(request_data),
                                      &rcv_buf_len, snd_buf_len,
                                      &rsp_mad, &query_result); 
            if (fstatus == FSUCCESS) 
            {
                if (port->pa_verbose) 
                {
                    DBGPRINT("Completed request: OK\n");
                    oib_dump_rmpp_response(rsp_mad);
                }
                data = oib_get_rmpp_mad_data(rsp_mad);

                // translate the data.
				pa_result = (STL_PA_VF_LIST_RESULTS *)query_result->QueryResult;
				pa_data = pa_result->VFListRecords;
				if (pa_result->NumVFListRecords) {
					memcpy(pa_data, data, rcv_buf_len - IB_SA_DATA_OFFS);
				}
            } 
            else 
            {
                if (port->pa_verbose)
                    DBGPRINT("Completed request: FAILED\n");
            }
        }
        break;

    default:
        OUTPUT_ERROR("Query Not supported in OPEN IB: Input=%s, Output=%s\n",
                iba_pa_query_input_type_msg(query->InputType),
                iba_pa_query_result_type_msg(query->OutputType));
        fstatus = FERROR; 
        goto done;
        break;
    }

done:
    if (rsp_mad!=NULL) 
    {
        free (rsp_mad);
        rsp_mad = NULL;
    }
    
    *pquery_result = query_result;

    DBG_EXIT_FUNC;
    return fstatus;
}

/**
 *  Get multi-record response for pa vf info (stats).
 *
 * @param port                      Local port to operate on. 
 * @param query                     Pointer to the query 
 * @param vf_name                	VF name 
 * @param pquery_result             Pointer to query result to be filled. The caller has to 
 *                                  to free the buffer.
 * @param query_control_parameters  Optional query control parameters (retry, timeout). 
 * @param image_id                  Pointer to the image ID. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_multi_mad_vf_info_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *vf_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    uint8_t                     *rsp_mad = NULL;
    uint8_t                     *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
    char                        request_data[PA_REQ_HEADER_SIZE + 88] = {0}; //80 = VFName + reserved + ImageID
    STL_PA_VF_INFO_DATA         *pa_data;
    STL_PA_VF_INFO_DATA         *p;
    STL_PA_VF_INFO_RESULTS      *pa_result;

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return (FERROR);
    }

    DBGPRINT("Query: Port=0x%"PRIx64" Input=%s, Output=%s\n",
             oib_get_port_guid(port),
             iba_pa_query_input_type_msg(query->InputType),
             iba_pa_query_result_type_msg(query->OutputType));

    // optional query parameters are not supported in Open IB.
    if (query_control_parameters != NULL) 
    {
        printf("Optional Query Control paramerers are not supported in OPENIB\n");
        return (FERROR);
    }

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return (FERROR);
    }
    
    p = (STL_PA_VF_INFO_DATA *)request_data;
    snprintf(p->vfName, sizeof(p->vfName), "%s", vf_name);
    p->imageId.imageNumber = image_id->imageNumber;
    p->imageId.imageOffset = image_id->imageOffset;
	p->imageId.imageTime.absoluteTime = image_id->imageTime.absoluteTime;
	BSWAP_STL_PA_VF_INFO(p, 1);
    
    // process the command.
    switch (query->OutputType) 
    {
        case OutputTypePaTableRecord:       
        {
            DBGPRINT("Sending Get Multi Record request\n");

            // submit request
            fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_VF_INFO, 0,
                                      request_data, sizeof(request_data),
                                      &rcv_buf_len, snd_buf_len,
                                      &rsp_mad, &query_result); 
            if (fstatus == FSUCCESS) 
            {
                if (port->pa_verbose) 
                {
                    DBGPRINT("Completed request: OK\n");
                    oib_dump_rmpp_response(rsp_mad);
                }
                data = oib_get_rmpp_mad_data(rsp_mad);
                if (port->pa_verbose) 
                {
                    DBGPRINT("Response Data:\n");
                    oib_xdump(stderr, data, rcv_buf_len - IB_SA_DATA_OFFS, 8);
                }

                // translate the data.
                pa_result = (STL_PA_VF_INFO_RESULTS*)query_result->QueryResult;
                pa_data = pa_result->VFInfoRecords;
                memcpy(pa_data, data, min(STL_PA_VF_INFO_NSIZE * pa_result->NumVFInfoRecords, rcv_buf_len - IB_SA_DATA_OFFS));
                BSWAP_STL_PA_VF_INFO(pa_data, 0);
            } 
            else 
            {
                if (port->pa_verbose)
                    DBGPRINT("Completed request: FAILED\n");
            }
        }
        break;

    default:
        OUTPUT_ERROR("Query Not supported in OPEN IB: Input=%s, Output=%s\n",
                iba_pa_query_input_type_msg(query->InputType),
                iba_pa_query_result_type_msg(query->OutputType));
        fstatus = FERROR; 
        goto done;
        break;
    }

done:
    if (rsp_mad!=NULL) 
    {
        free (rsp_mad);
        rsp_mad = NULL;
    }
    
    *pquery_result = query_result;

    DBG_EXIT_FUNC;
    return fstatus;
}
FSTATUS 
iba_pa_multi_mad_vf_config_response_query(
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *vf_name,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         *query_result = NULL;
    uint8_t                     *rsp_mad = NULL;
    uint8_t                     *data;
    int                         rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                         snd_buf_len = DEF_BUF_DATA_SIZE;
	int							i;
    char                        request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_VF_CFG_REQ)] = {0};
    STL_PA_VF_CFG_RSP           *pa_data;
    STL_PA_VF_CFG_REQ           *p;
    STL_PA_VF_CONFIG_RESULTS    *pa_result;

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return (FERROR);
    }

    DBGPRINT("Query: Port=0x%"PRIx64" Input=%s, Output=%s\n",
             oib_get_port_guid(port),
             iba_pa_query_input_type_msg(query->InputType),
             iba_pa_query_result_type_msg(query->OutputType));

    // optional query parameters are not supported in Open IB.
    if (query_control_parameters != NULL) 
    {
        printf("Optional Query Control paramerers are not supported in OPENIB\n");
        return (FERROR);
    }

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return (FERROR);
    }
    
    p = (STL_PA_VF_CFG_REQ *)request_data;
    snprintf(p->vfName, sizeof(p->vfName), "%s", vf_name);

    p->imageId.imageNumber = hton64(image_id->imageNumber);
    p->imageId.imageOffset = hton32(image_id->imageOffset);
    p->imageId.imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);

    // process the command.
    switch (query->OutputType) 
    {
        case OutputTypePaTableRecord:       
        {
            DBGPRINT("Sending Get Multi Record request\n");

            // submit request
            fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_VF_CONFIG, 0,
                                      request_data, sizeof(request_data),
                                      &rcv_buf_len, snd_buf_len,
                                      &rsp_mad, &query_result); 
            if (fstatus == FSUCCESS) 
            {
                if (port->pa_verbose) 
                {
                    DBGPRINT("Completed request: OK\n");
                    oib_dump_rmpp_response(rsp_mad);
                }
                data = oib_get_rmpp_mad_data(rsp_mad);

                // translate the data.
                pa_result = (STL_PA_VF_CONFIG_RESULTS*)query_result->QueryResult;
                pa_data = pa_result->VFConfigRecords;
                if (pa_result->NumVFConfigRecords) 
                {
					memcpy(pa_data, data, rcv_buf_len - IB_SA_DATA_OFFS);
					for (i = 0; i < pa_result->NumVFConfigRecords; i++)
                    	BSWAP_STL_PA_VF_CFG_RSP(&pa_data[i]);
                }
            } 
            else 
            {
                if (port->pa_verbose)
                    DBGPRINT("Completed request: FAILED\n");
            }
        }
        break;

    default:
        OUTPUT_ERROR("Query Not supported in OPEN IB: Input=%s, Output=%s\n",
                iba_pa_query_input_type_msg(query->InputType),
                iba_pa_query_result_type_msg(query->OutputType));
        fstatus = FERROR; 
        goto done;
        break;
    }

done:
    if (rsp_mad!=NULL) 
    {
        free (rsp_mad);
        rsp_mad = NULL;
    }
    
    *pquery_result = query_result;

    DBG_EXIT_FUNC;
    return fstatus;
}
STL_PA_VF_PORT_COUNTERS_DATA *
iba_pa_single_mad_vf_port_counters_response_query(
    IN struct oib_port      *port,
    IN uint32_t              node_lid,
    IN uint8_t               port_number,
    IN uint32_t              delta_flag,
    IN uint32_t              user_cntrs_flag,
    IN char                 *vfName,
    IN STL_PA_IMAGE_ID_DATA *image_id
    )
{
    FSTATUS                 fstatus = FSUCCESS;
    QUERY_RESULT_VALUES     *query_result = NULL;
    STL_PA_VF_PORT_COUNTERS_DATA      *response = NULL;
    STL_PA_VF_PORT_COUNTERS_DATA      *p;
    uint8_t                 *rsp_mad = NULL;
    uint8_t                 *data;
    int                     rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                     snd_buf_len = DEF_BUF_DATA_SIZE;
    char                    request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_VF_PORT_COUNTERS_DATA)] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    p = (STL_PA_VF_PORT_COUNTERS_DATA *)request_data;
    p->nodeLid = node_lid;
    p->portNumber = port_number;
    p->flags =  (delta_flag ? STL_PA_PC_FLAG_DELTA : 0) |
                (user_cntrs_flag ? STL_PA_PC_FLAG_USER_COUNTERS : 0);
	snprintf(p->vfName, sizeof(p->vfName), "%s", vfName);

    p->imageId.imageNumber = image_id->imageNumber;
    p->imageId.imageOffset = image_id->imageOffset;
	p->imageId.imageTime.absoluteTime = image_id->imageTime.absoluteTime;
	memset(p->reserved, 0, sizeof(p->reserved));
	p->reserved1 = 0;
    BSWAP_STL_PA_VF_PORT_COUNTERS(p);

    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_GET, STL_PA_ATTRID_GET_VF_PORT_CTRS, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
        oib_free_query_result_buffer(query_result);
        return response;
    } 
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);

    response = MemoryAllocate2AndClear(STL_PA_VF_PORT_COUNTERS_NSIZE, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
        oib_free_query_result_buffer(query_result);
        return (NULL);
    }
    memcpy((uint8 *)response, data, min(STL_PA_VF_PORT_COUNTERS_NSIZE, rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_PA_VF_PORT_COUNTERS(response);

    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;
}

STL_PA_CLEAR_VF_PORT_COUNTERS_DATA *
iba_pa_single_mad_clr_vf_port_counters_response_query(
    IN struct oib_port  *port,
    IN uint32_t         node_lid,
    IN uint8_t         port_number,
    IN uint32_t         select,
	IN char				*vfName
    )
{
    FSTATUS                 fstatus = FSUCCESS;
    QUERY_RESULT_VALUES     *query_result = NULL;
    STL_PA_CLEAR_VF_PORT_COUNTERS_DATA  *response = NULL;
    STL_PA_CLEAR_VF_PORT_COUNTERS_DATA  *p;
    uint8_t                 *rsp_mad = NULL;
    uint8_t                 *data;
    int                     rcv_buf_len = DEF_BUF_DATA_SIZE; 
    int                     snd_buf_len = DEF_BUF_DATA_SIZE;
    char                    request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_CLEAR_VF_PORT_COUNTERS_DATA)] = {0};

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return response;
    }
    DBGPRINT("Query: Port=0x%"PRIx64"\n", oib_get_port_guid(port));

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return response;
    }

    p = (STL_PA_CLEAR_VF_PORT_COUNTERS_DATA *)request_data;
    p->nodeLid = hton32(node_lid);
    p->portNumber = port_number;
    p->vfCounterSelectMask.AsReg32 = hton32(select);
	snprintf(p->vfName, sizeof(p->vfName), "%s", vfName);
	memset(p->reserved, 0, sizeof(p->reserved));
	p->reserved2 = 0;

    DBGPRINT("Sending Get Single Record request\n");

    // submit request
    fstatus = pa_query_common(port, STL_PA_CMD_SET, STL_PA_ATTRID_CLR_VF_PORT_CTRS, 0,
                              request_data, sizeof(request_data),
                              &rcv_buf_len, snd_buf_len,
                              &rsp_mad, &query_result);
    if (fstatus != FSUCCESS) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, requeset failed: status=%u\n",  (unsigned int)fstatus);
        return response;
    } 
    else if (query_result->ResultDataSize) 
    {
        if (port->pa_verbose)
            OUTPUT_ERROR("Error, unexpected multiple MAD response\n");
	oib_free_query_result_buffer(query_result);
        return response;
    } 
    else 
    {
        if (port->pa_verbose) 
        {
            DBGPRINT("Completed request: OK\n");
            oib_dump_rmpp_response(rsp_mad);
        }
    }

    data = oib_get_rmpp_mad_data(rsp_mad);

    response = MemoryAllocate2AndClear(STL_PA_CLR_VF_PORT_COUNTERS_NSIZE, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG);
    if (response == NULL) 
    {
        OUTPUT_ERROR("error allocating response buffer\n");
        oib_free_query_result_buffer(query_result);
        return (NULL);
    }
    memcpy((uint8 *)response, data, min(STL_PA_CLR_VF_PORT_COUNTERS_NSIZE, rcv_buf_len - IB_SA_DATA_OFFS));

    // translate the data.
    BSWAP_STL_PA_CLEAR_VF_PORT_COUNTERS(response);

    oib_free_query_result_buffer(query_result);
    if (rsp_mad)
        free(rsp_mad);

    DBG_EXIT_FUNC;
    return response;
}

FSTATUS 
iba_pa_multi_mad_vf_focus_ports_response_query (
    IN struct oib_port              *port,
    IN PQUERY                       query,
    IN char                         *vf_name,
    IN uint32                       select,
    IN uint32                       start,
    IN uint32                       range,
    OUT PQUERY_RESULT_VALUES        *pquery_result,
    IN COMMAND_CONTROL_PARAMETERS   *query_control_parameters OPTIONAL,
    IN STL_PA_IMAGE_ID_DATA         *image_id
    )
{
    FSTATUS                     	fstatus = FSUCCESS;
    QUERY_RESULT_VALUES         	*query_result = NULL;
    uint8_t                     	*rsp_mad = NULL;
    uint8_t                     	*data;
    int                         	rcv_buf_len = DEF_BUF_DATA_SIZE;
    int                         	snd_buf_len = DEF_BUF_DATA_SIZE;
    int                         	i;
    char                        	request_data[PA_REQ_HEADER_SIZE + sizeof(STL_PA_VF_FOCUS_PORTS_REQ)] = {0};
    STL_PA_VF_FOCUS_PORTS_RSP     	*pa_data;
    STL_PA_VF_FOCUS_PORTS_REQ     	*p;
    STL_PA_VF_FOCUS_PORTS_RESULTS 	*pa_result;

    DBG_ENTER_FUNC;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return (FERROR);
    }

    DBGPRINT("Query: Port=0x%"PRIx64" Input=%s, Output=%s\n",
             oib_get_port_guid(port),
             iba_pa_query_input_type_msg(query->InputType),
             iba_pa_query_result_type_msg(query->OutputType));

    // optional query parameters are not supported in Open IB.
    if (query_control_parameters != NULL) 
    {
        printf("Optional Query Control paramerers are not supported in OPENIB\n");
        return (FERROR);
    }

    // if port is not active we cannot trust its sm_lid, so fail immediately
    if (oib_get_port_state(port) != PortStateActive)
    {
        OUTPUT_ERROR("Error, port is not active!\n");
        return (FERROR);
    }
    
    p = (STL_PA_VF_FOCUS_PORTS_REQ *)request_data;
    snprintf(p->vfName, sizeof(p->vfName), "%s", vf_name);
    p->select = hton32(select);
    p->start = hton32(start);
    p->range = hton32(range);

    p->imageId.imageNumber = hton64(image_id->imageNumber);
    p->imageId.imageOffset = hton32(image_id->imageOffset);
	p->imageId.imageTime.absoluteTime = hton32(image_id->imageTime.absoluteTime);
    // process the command.
    switch (query->OutputType) 
    {
        case OutputTypePaTableRecord:       
        {
            DBGPRINT("Sending Get Multi Record request\n");

            // submit request
            fstatus = pa_query_common(port, STL_PA_CMD_GETTABLE, STL_PA_ATTRID_GET_VF_FOCUS_PORTS, 0,
                                      request_data, sizeof(request_data),
                                      &rcv_buf_len, snd_buf_len,
                                      &rsp_mad, &query_result); 
            if (fstatus == FSUCCESS) 
            {
                if (port->pa_verbose) 
                {
                    DBGPRINT("Completed request: OK\n");
                    oib_dump_rmpp_response(rsp_mad);
                }
                data = oib_get_rmpp_mad_data(rsp_mad);
                // translate the data.
                pa_result = (STL_PA_VF_FOCUS_PORTS_RESULTS *)query_result->QueryResult;
                pa_data = pa_result->FocusPortsRecords;
                if (pa_result->NumVFFocusPortsRecords) 
                {
					memcpy(pa_data, data, rcv_buf_len - IB_SA_DATA_OFFS);
					for (i = 0; i < pa_result->NumVFFocusPortsRecords; i++)
                    	BSWAP_STL_PA_VF_FOCUS_PORTS_RSP(&pa_data[i]);
                }
            } 
            else 
            {
                if (port->pa_verbose)
                    DBGPRINT("Completed request: FAILED\n");
            }
        }
        break;

    default:
        OUTPUT_ERROR("Query Not supported in OPEN IB: Input=%s, Output=%s\n",
                iba_pa_query_input_type_msg(query->InputType),
                iba_pa_query_result_type_msg(query->OutputType));
        fstatus = FERROR; 
        goto done;
        break;
    }

done:
    if (rsp_mad!=NULL) 
    {
        free (rsp_mad);
        rsp_mad = NULL;
    }
    
    *pquery_result = query_result;

    DBG_EXIT_FUNC;
    return fstatus;
}

/**
 *  Get master pm LID
 *
 * @param port                  Local port to operate on. 
 * @param primary_pm_lid        Pointer to master pm lid to be filled. 
 *
 * @return 
 *   FSUCCESS       - Get successfully
 *     FERROR       - Error
 */
FSTATUS 
iba_pa_query_master_pm_lid(
    IN struct oib_port  *port,
    OUT uint16_t        *primary_pm_lid
    )
{
    FSTATUS                 status;
    QUERY                   query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    PQUERY_RESULT_VALUES    query_path_results = NULL;
    SERVICE_RECORD_RESULTS  *service_record_results;
    PATH_RESULTS            *path_record_results;

    // Check the incoming port parameter
    if (NULL == port) 
    {
        OUTPUT_ERROR("Error, port is null!\n");
        return (FERROR);
    }
    if (primary_pm_lid == NULL)
        return (FERROR);

    // query SA for Service Records to all PMs within the fabric
    memset(&query, 0, sizeof(query));
    query.InputType = InputTypeNoInput;
    query.OutputType = OutputTypeServiceRecord;

    status = oib_query_sa(port, &query, &query_results);
    if (status != FSUCCESS) 
    {
        OUTPUT_ERROR("Error, failed to get service records (status=0x%x) query_results=%p: %s\n", 
		     (unsigned int)status, (void *)query_results, FSTATUS_MSG(status));
    } 
    else if ((query_results == NULL) || (query_results->ResultDataSize == 0)) 
    {
        status = FUNAVAILABLE;
    } 
    else 
    {
        int i, ix, pmCount=0;

        status=FUNAVAILABLE;
        service_record_results = (SERVICE_RECORD_RESULTS *)query_results->QueryResult;
        if (service_record_results->NumServiceRecords) 
        {
            DBGPRINT("Got Service Records: records=%d\n", service_record_results->NumServiceRecords);
            for (i = 0; i < service_record_results->NumServiceRecords; ++i) 
            {
                if (PM_SERVICE_ID == service_record_results->ServiceRecords[i].RID.ServiceID) 
                {
                    pmCount++;
                    if ((service_record_results->ServiceRecords[i].ServiceData8[0] >= PM_VERSION) &&
                        (service_record_results->ServiceRecords[i].ServiceData8[1] == PM_MASTER))
                    {
                        DBGPRINT("This is the Primary PM.\n");

                        //
                        // query SA for Path Record to the primary PM
                        memset(&query, 0, sizeof(query));
						query.InputType = InputTypePathRecord;

                        query.OutputType = OutputTypePathRecord;

						query.InputValue.PathRecordValue.PathRecord.DGID = 
							service_record_results->ServiceRecords[i].RID.ServiceGID;
						query.InputValue.PathRecordValue.PathRecord.SGID = port->local_gid;
						query.InputValue.PathRecordValue.PathRecord.ServiceID = PM_SERVICE_ID;
						query.InputValue.PathRecordValue.ComponentMask = IB_MULTIPATH_RECORD_COMP_NUMBPATH |
																		 PR_COMPONENTMASK_SRV_ID |
																		 PR_COMPONENTMASK_DGID |
																		 PR_COMPONENTMASK_SGID;
						query.InputValue.PathRecordValue.PathRecord.NumbPath = 32;

                        status = oib_query_sa(port, &query, &query_path_results);
                        if ((status != FSUCCESS) || (query_path_results == NULL) || (query_path_results->ResultDataSize == 0)) 
                        {
                            status = FERROR;
                            OUTPUT_ERROR("Error, failed to get path record (status=0x%x) query_path_results=%p: %s\n",
                                    (unsigned int)status, (void *)query_path_results, FSTATUS_MSG(status));
                            break;
                        } 
                        else 
                        {
                            path_record_results = (PATH_RESULTS *)query_path_results->QueryResult;
                            DBGPRINT("Got Path Records: records=%d\n", path_record_results->NumPathRecords);
                            if (path_record_results->NumPathRecords >= 1) 
                            {
                                // Find PathRecord with default P_Key or use PathRecords[0]
                                for (ix = path_record_results->NumPathRecords - 1; ix > 0; ix--)
                                    if ( (path_record_results->PathRecords[ix].P_Key & 0x7FFF) ==
                                            (DEFAULT_P_KEY & 0x7FFF) )
                                        break;
                                *primary_pm_lid = path_record_results->PathRecords[ix].DLID;
                                // Remember that for our own use
                                port->primary_pm_lid = *primary_pm_lid;
                                port->primary_pm_sl = path_record_results->PathRecords[ix].u2.s.SL;
                                DBGPRINT("Found Master PM LID 0x%x\n", path_record_results->PathRecords[ix].DLID);
                            } 
                            else 
                            {
                                status = FERROR;
                                OUTPUT_ERROR("Error, received no path records\n");
                                        
                                break;
                            }
                        }
                    }
                }
            }
            DBGPRINT("Number of PMs found %d\n", pmCount);
        }
    }
    // deallocate results buffers
    if (query_results)
        oib_free_query_result_buffer(query_results);
    if (query_path_results)
        oib_free_query_result_buffer(query_path_results);

    return status;
}

/**
 *  Get and Convert a MAD status code into string.
 *
 * @param port                    Local port to operate on. 
 * 
 * @return 
 *   The corresponding status string.
 */
const char* 
iba_pa_mad_status_msg(
    IN struct oib_port              *port
    )
{
    // this is a little more complex than most due to bitfields and reserved
    // values
    MAD_STATUS  mad_status;
    uint16_t    code = port->mad_status;

    mad_status.AsReg16 = port->mad_status; // ignore reserved bits in mad_status
    if (code == MAD_STATUS_SUCCESS || (code & 0xff))
        return iba_mad_status_msg(mad_status);   // standard mad status fields
    else 
    {
		// SA specific status code field 0-6
		code = mad_status.S.ClassSpecific;
		if (code >= (unsigned)(sizeof(pa_sa_status_text)/sizeof(char*))) {
			// PA specific status code field - codes start at 'A' (10)
			code = mad_status.S.ClassSpecific - 9;
			if (code >= (unsigned)(sizeof(pa_status_text)/sizeof(char*)))
				return "Unknown PA Mad Status";
			else
				return pa_status_text[code];
		} else
			return pa_sa_status_text[code];
    }
}



/**
 * initialize Pa Client
 *
 * @param port          Pointer to the port to operate on. Set upon successful exit. 
 * @param hfi_num       The host channel adaptor number to use to access fabric.
 * @param port_num      The port number to use to access fabric.
 * @param verbose_file  Pointer to FILE for verbose output
 *
 * @return 
 *  PACLIENT_OPERATIONAL - initialization successful
 *         PACLIENT_DOWN - initialization not successful/PaServer not available 
 */
int 
oib_pa_client_init(
    struct oib_port     **port, 
    int                 hfi_num, 
    uint8_t             port_num,
    FILE                *verbose_file
    )
{
    FSTATUS                 fstatus;
    int                     mclass = MCLASS_VFI_PM;
    struct oib_class_args   mgmt_class[2];
    int                     err = 0;
 
    // Validate the incoming parameters
    if (NULL == port) 
    {
        fprintf (stderr, "port is NULL!\n");
        return PACLIENT_INVALID_PARAM;
    }
    // Open a port
    if (0 != oib_open_port_by_num(port, hfi_num, port_num))
    {
        fprintf (stderr, "Can't open the port for hfi %d port %d\n", hfi_num, port_num);
        return PACLIENT_DOWN;
    }

    (*port)->verbose_file = verbose_file;

    if (verbose_file != NULL)
        set_opapaquery_debug(*port);

    if ( ( fstatus = iba_pa_query_master_pm_lid( *port, & ((*port)->primary_pm_lid) ) ) != FSUCCESS )
    {
        fprintf (stderr, "Can't query primary PM LID!\n");
        oib_close_port(*port);
        *port = NULL;
        return (PACLIENT_DOWN);
    }

    // register PA methods with OFED
    memset(mgmt_class, 0, sizeof(mgmt_class));
    mgmt_class[0].base_version = STL_BASE_VERSION;
    mgmt_class[0].mgmt_class = mclass;
    mgmt_class[0].class_version = STL_PM_CLASS_VERSION;
    mgmt_class[0].is_responding_client = 0;
    mgmt_class[0].is_trap_client = 0;
    mgmt_class[0].is_report_client = 0;
    mgmt_class[0].kernel_rmpp = 1; // TODO: determine if this should be 0
    mgmt_class[0].use_methods = 0;
    mgmt_class[0].oui = ib_truescale_oui;

    if ((err = oib_bind_classes(*port, mgmt_class)) != 0) 
    {
        fprintf (stderr, "Failed to  register management class 0x%02x: %s\n",
                 mclass, strerror(err));
        oib_close_port(*port);
        *port = NULL;
        return (PACLIENT_DOWN);
    }

    return ((*port)->pa_client_state = PACLIENT_OPERATIONAL);
}

/**
 * initialize Pa Client
 *
 * @param port          Pointer to the port to operate on. Set upon successful exit. 
 * @param port_guid     The port guid to use to access fabric.
 * @param verbose_file  Pointer to FILE for verbose output
 *
 * @return 
 *  PACLIENT_OPERATIONAL - initialization successful
 *         PACLIENT_DOWN - initialization not successful/PaServer not available 
 */
int 
oib_pa_client_init_by_guid(
    struct oib_port     **port, 
    uint64_t            port_guid,
    FILE                *verbose_file
    )
{
    FSTATUS                 fstatus;
    int                     mclass = MCLASS_VFI_PM;
    struct oib_class_args   mgmt_class[2];
    int                     err = 0;
 
    // Validate the incoming parameters
    if (NULL == port) 
    {
        fprintf (stderr, "port is NULL!\n");
        return PACLIENT_INVALID_PARAM;
    }
    // Open a port
    if (0 != oib_open_port_by_guid(port, port_guid))
    {
        fprintf (stderr, "Can't open the port for guid %lx\n", port_guid);
        return PACLIENT_DOWN;
    }

    (*port)->verbose_file = verbose_file;

    if (verbose_file != NULL)
        set_opapaquery_debug(*port);

    if ( ( fstatus = iba_pa_query_master_pm_lid( *port, & ((*port)->primary_pm_lid) ) ) != FSUCCESS )
    {
        fprintf (stderr, "Can't query primary PM LID!\n");
        oib_close_port(*port);
        *port = NULL;
        return (PACLIENT_DOWN);
    }

    // register PA methods with OFED
    memset(mgmt_class, 0, sizeof(mgmt_class));
    mgmt_class[0].base_version = STL_BASE_VERSION;
    mgmt_class[0].mgmt_class = mclass;
    mgmt_class[0].class_version = STL_PM_CLASS_VERSION;
    mgmt_class[0].is_responding_client = 0;
    mgmt_class[0].is_trap_client = 0;
    mgmt_class[0].is_report_client = 0;
    mgmt_class[0].kernel_rmpp = 1;
    mgmt_class[0].use_methods = 0;
    mgmt_class[0].oui = ib_truescale_oui;

    if ((err = oib_bind_classes(*port, mgmt_class)) != 0) 
    {
        fprintf (stderr, "Failed to  register management class 0x%02x: %s\n",
                 mclass, strerror(err));
        oib_close_port(*port);
        *port = NULL;
        return (PACLIENT_DOWN);
    }

    return ((*port)->pa_client_state = PACLIENT_OPERATIONAL);
 }

/** 
 *  Get PM configuration data
 *
 * @param port          Port to operate on. 
 * @param pm_config     Pointer to PM config data to fill
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_get_pm_config(
    struct oib_port     *port, 
    STL_PA_PM_CFG_DATA      *pm_config
    )
{
    FSTATUS         fstatus = FERROR;
	STL_PA_PM_CFG_DATA * response;

    // Validate parameters
    if (NULL == port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) || 
        !pm_config)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    DBGPRINT("Getting PM Configuration...\n");

    if ( ( response =
           iba_pa_single_mad_get_pm_config_response_query(port) ) != NULL )
    {
#if 0
TODO STL COMPILE!
        // TBD remove some of these after additional MAD testing
        DBGPRINT("Interval = %u\n", response->sweepInterval);
        DBGPRINT("Max Clients = %u\n", response->MaxClients);
        DBGPRINT("History Size = %u\n", response->sizeHistory);
        DBGPRINT("Freeze Size = %u\n", response->sizeFreeze);
        DBGPRINT("Freeze Lease = %u\n", response->lease);
        DBGPRINT("Flags = 0x%X\n", response->pmFlags);
        DBGPRINT( "Memory Footprint = 0x%" PRIX64 "\n",
                  response->memoryFootprint );
#endif
        memcpy(pm_config, response, sizeof(*pm_config));

        MemoryDeallocate(response);
        fstatus = FSUCCESS;
    }
    else
        DBGPRINT("Got NULL response - FAILED\n");

    return (fstatus);
}

/** 
 *  Get image info
 *
 * @param port              Port to operate on. 
 * @param pm_image_Id       Image ID of image info to get 
 * @param pm_image_info     Pointer to image info to fill 
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_get_image_info(
    struct oib_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id,
    STL_PA_IMAGE_INFO_DATA     *pm_image_info 
    )
{
    FSTATUS         fstatus = FERROR;
    uint32          ix;
    STL_PA_IMAGE_INFO_DATA query = {{0}};
    STL_PA_IMAGE_INFO_DATA *response;;

    if (NULL == port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) || 
        !pm_image_info)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    query.imageId = pm_image_id;

    DBGPRINT("Getting Image Info...\n");

    if ( ( response = iba_pa_multi_mad_get_image_info_response_query( 
       port, &query ) ) != NULL )
    {
#if 0        // TBD remove some of these after additional MAD testing
        DBGPRINT( "RespImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->imageId.imageNumber, response->imageId.imageOffset );
        DBGPRINT( "sweepStart = 0x%" PRIX64 "\n",
                  response->sweepStart );
        DBGPRINT( "sweepDuration = %u\n",
                  response->sweepDuration );
        DBGPRINT( "numHFIPorts = %u\n",
                  response->numHFIPorts );
        DBGPRINT( "numSwNodes = %u\n",
                  response->numSwitchNodes );
        DBGPRINT( "numSwPorts = %u\n",
                  response->numSwitchPorts );
        DBGPRINT( "numLinks = %u\n",
                  response->numLinks );
        DBGPRINT( "numSMs = %u\n",
                  response->numSMs );
        DBGPRINT( "numFailedNodes = %u\n",
                  response->numFailedNodes );
        DBGPRINT( "numFailedPorts = %u\n",
                  response->numFailedPorts );
        DBGPRINT( "numSkippedPorts = %u\n",
                  response->numSkippedPorts );
        DBGPRINT( "numUnexpectedClearPorts = %u\n",
                  response->numUnexpectedClearPorts );
#endif
        memcpy(pm_image_info, response, sizeof(*pm_image_info));

        for (ix = pm_image_info->numSMs; ix < 2; ix++)
            memset(&pm_image_info->SMInfo[ix], 0, sizeof(SMINFO_DATA));

        MemoryDeallocate(response);
        fstatus = FSUCCESS;
    }
    else
        DBGPRINT("Got NULL response - FAILED\n");

    return (fstatus);
}

/** 
 *  Get list of group names
 *
 * @param port              Port to operate on. 
 * @param pm_group_list     Pointer to group list to fill 
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS
pa_client_get_group_list(
    struct oib_port          *port,
	uint32					 *pNum_Groups,
    STL_PA_GROUP_LIST  		**pm_group_list
    )
{
    FSTATUS						fstatus = FERROR;
    uint32						size_group_list;
    QUERY						query;
    PQUERY_RESULT_VALUES		query_results = NULL;
    STL_PA_GROUP_LIST_RESULTS	*p;

    // Validate the parameters and state
    if (!port ||
        (port->pa_client_state != PACLIENT_OPERATIONAL) || 
		!pNum_Groups || !pm_group_list || *pm_group_list)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));   // initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    DBGPRINT("Getting Multi Record Response For Group Data...\n");
    DBGPRINT("Query: Input=%s, Output=%s\n",
              iba_pa_query_input_type_msg(query.InputType),
              iba_pa_query_result_type_msg(query.OutputType) );

    fstatus = iba_pa_multi_mad_group_list_response_query( port, &query, &query_results, NULL );

    if (!query_results)
    {
        DBGPRINT( "PA Group List query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        DBGPRINT( "PA Group List query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status),
                  port->mad_status, iba_pa_mad_status_msg(port) );
        goto fail;
    }

    else if (query_results->ResultDataSize == 0)
    {
        DBGPRINT("No Records Returned\n");
        goto fail;
    }
    else
    {
		p = (STL_PA_GROUP_LIST_RESULTS*)query_results->QueryResult;
        
        // TBD remove some of these after additional MAD testing
        DBGPRINT( "MadStatus 0x%X: %s\n", port->mad_status,
                  iba_pa_mad_status_msg(port));
        DBGPRINT("%d Bytes Returned\n", query_results->ResultDataSize);
        DBGPRINT("PA Multiple MAD Response for Group Data:\n");
		DBGPRINT("NumGroupListRecords = %d\n",
				 (int)p->NumGroupListRecords );

		*pNum_Groups = p->NumGroupListRecords;
		size_group_list = sizeof(STL_PA_GROUP_LIST) * *pNum_Groups;

		if ( !( *pm_group_list = MemoryAllocate2AndClear( size_group_list, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG ) ) )
        {
            OUTPUT_ERROR("can not allocate memory\n");
            goto fail;
        }

		memcpy(*pm_group_list, p->GroupListRecords, size_group_list);
        fstatus = FSUCCESS;
    }

done:
    // oib_query_port_fabric will have allocated a result buffer
    // we must free the buffer when we are done with it
    if (query_results)
        oib_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = FERROR;
    goto done;

}

void
pa_client_release_group_list(
    struct oib_port          *port,
    STL_PA_GROUP_LIST	    **pm_group_list
    )
{
    if (pm_group_list && *pm_group_list)
    {
        MemoryDeallocate(*pm_group_list);
        *pm_group_list = NULL;
    }
}
/**
 *  Get list of vf names
 *
 * @param port              Port to operate on.
 * @param pm_vf_list     Pointer to vf list to fill
 *
 * @return
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS
pa_client_get_vf_list(
    struct oib_port          *port,
	uint32					 *pNum_VFs,
    STL_PA_VF_LIST  		**pm_vf_list
    )
{
    FSTATUS						fstatus = FERROR;
    uint32						size_vf_list;
    QUERY						query;
    PQUERY_RESULT_VALUES		query_results = NULL;
    STL_PA_VF_LIST_RESULTS		*p;

    // Validate the parameters and state
    if (!port ||
        (port->pa_client_state != PACLIENT_OPERATIONAL) ||
		!pNum_VFs || !pm_vf_list || *pm_vf_list)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));   // initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    DBGPRINT("Getting Multi Record Response For VF Data...\n");
    DBGPRINT("Query: Input=%s, Output=%s\n",
              iba_pa_query_input_type_msg(query.InputType),
              iba_pa_query_result_type_msg(query.OutputType) );

    fstatus = iba_pa_multi_mad_vf_list_response_query( port, &query, &query_results, NULL );

    if (!query_results)
    {
        DBGPRINT( "PA VF List query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        DBGPRINT( "PA VF List query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status),
                  port->mad_status, iba_pa_mad_status_msg(port) );
        goto fail;
    }

    else if (query_results->ResultDataSize == 0)
    {
        DBGPRINT("No Records Returned\n");
        goto fail;
    }
    else
    {
		p = (STL_PA_VF_LIST_RESULTS*)query_results->QueryResult;

        // TBD remove some of these after additional MAD testing
        DBGPRINT( "MadStatus 0x%X: %s\n", port->mad_status,
                  iba_pa_mad_status_msg(port));
        DBGPRINT("%d Bytes Returned\n", query_results->ResultDataSize);
        DBGPRINT("PA Multiple MAD Response for VF Data:\n");
		DBGPRINT("NumVFListRecords = %d\n",
				 (int)p->NumVFListRecords );

		*pNum_VFs = p->NumVFListRecords;
		size_vf_list = sizeof(STL_PA_VF_LIST) * *pNum_VFs;

		if ( !( *pm_vf_list = MemoryAllocate2AndClear( size_vf_list, IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG ) ) )
        {
            OUTPUT_ERROR("can not allocate memory\n");
            goto fail;
        }

		memcpy(*pm_vf_list, p->VFListRecords, size_vf_list);
        fstatus = FSUCCESS;
    }

done:
    // oib_query_port_fabric will have allocated a result buffer
    // we must free the buffer when we are done with it
    if (query_results)
        oib_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = FERROR;
    goto done;

}

void
pa_client_release_vf_list(
    struct oib_port      *port,
    STL_PA_VF_LIST	    **pm_vf_list
    )
{
    if (pm_vf_list && *pm_vf_list)
    {
        MemoryDeallocate(*pm_vf_list);
        *pm_vf_list = NULL;
    }
}
/** 
 *  Get group info
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of group info to get. 
 * @param group_name            Pointer to group name 
 * @param pm_image_id_resp      Pointer to image ID of group info returned. 
 * @param pm_group_info         Pointer to group info to fill. 
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_get_group_info( 
    struct oib_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                *group_name, 
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp, 
    STL_PA_PM_GROUP_INFO_DATA     *pm_group_info
    )
{
    FSTATUS                 fstatus = FERROR;
    STL_PA_IMAGE_ID_DATA           image_id = pm_image_id_query;
    QUERY                   query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_GROUP_INFO_RESULTS   *p;

    // Validate the parameters and state
    if (!port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) || 
        !group_name || !pm_group_info)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));	// initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    DBGPRINT("Getting Multi Record Response For Group Info...\n");
    DBGPRINT("Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

    fstatus = iba_pa_multi_mad_group_stats_response_query( 
       port, &query, group_name, &query_results, NULL, &image_id );

    if (!query_results)
    {
        DBGPRINT( "PA GroupInfo query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        DBGPRINT( "PA GroupInfo query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        DBGPRINT("No Records Returned\n");
        goto fail;
    }
    else
    {
        p = (STL_PA_GROUP_INFO_RESULTS*)query_results->QueryResult;
#if 0
        // TBD remove some of these after additional MAD testing
        DBGPRINT( "MadStatus 0x%X: %s\n", port->mad_status,
        	iba_pa_mad_status_msg(port->mad_status );
        DBGPRINT("%d Bytes Returned\n", query_results->ResultDataSize);
        DBGPRINT("PA Multiple MAD Response for Group Info group %s:\n",
                  group_name );
        DBGPRINT("NumGroupInfoRecords = %d\n",
                  (int)p->NumGroupInfoRecords );
        
        // TBD should this allow multiple records of group info ?
        // TBD remove some of these after additional MAD testing
        DBGPRINT("\tgroup name: %s\n", p->GroupInfoRecords[0].groupName);
        DBGPRINT("\tinternal ports: %d\n", p->GroupInfoRecords[0].numInternalPorts);
        DBGPRINT("\tinternal rate min: %d\n", p->GroupInfoRecords[0].minInternalRate);
        DBGPRINT("\tinternal rate max: %d\n", p->GroupInfoRecords[0].maxInternalRate);
        DBGPRINT("\tinternal MBps max: %d\n", p->GroupInfoRecords[0].maxInternalMBps);
        DBGPRINT("\texternal ports: %d\n", p->GroupInfoRecords[0].numExternalPorts);
        DBGPRINT("\texternal rate min: %d\n", p->GroupInfoRecords[0].minExternalRate);
        DBGPRINT("\texternal rate max: %d\n", p->GroupInfoRecords[0].maxExternalRate);
        DBGPRINT("\texternal MBps max: %d\n", p->GroupInfoRecords[0].maxExternalMBps);
        DBGPRINT( "\tinternal util totalMBps: %" PRIu64 "\n",
                  p->GroupInfoRecords[0].internalUtilStats.totalMBps );
        DBGPRINT( "\tinternal util avgMBps: %u\n",
                  p->GroupInfoRecords[0].internalUtilStats.avgMBps );
        DBGPRINT( "\tinternal util maxMBps: %u\n",
                  p->GroupInfoRecords[0].internalUtilStats.maxMBps );
        DBGPRINT( "\tinternal util totalKPps: %" PRIu64 "\n",
                  p->GroupInfoRecords[0].internalUtilStats.totalKPps );
        DBGPRINT( "\tinternal util avgKPps: %u\n",
                  p->GroupInfoRecords[0].internalUtilStats.avgKPps );
        DBGPRINT( "\tinternal util maxKPps: %u\n",
                  p->GroupInfoRecords[0].internalUtilStats.maxKPps );
        DBGPRINT( "\tsend util stats: %" PRIu64 "\n",
                  p->GroupInfoRecords[0].sendUtilStats.totalMBps );
        DBGPRINT( "\trecv util stats: %" PRIu64 "\n",
                  p->GroupInfoRecords[0].recvUtilStats.totalMBps );
        DBGPRINT( "\tinternal max integrity errors: %u\n",
                  p->GroupInfoRecords[0].internalErrors.errorMaximums.integrityErrors );
        DBGPRINT( "\texternal max integrity errors: %u\n",
                  p->GroupInfoRecords[0].externalErrors.errorMaximums.integrityErrors );
        DBGPRINT( "\tinternal integrity bucket[4]: %u\n",
                  p->GroupInfoRecords[0].internalErrors.ports[4].integrityErrors );
        DBGPRINT( "\texternal integrity bucket[4]: %u\n",
                  p->GroupInfoRecords[0].externalErrors.ports[4].integrityErrors );
#endif
        memcpy(pm_group_info, p->GroupInfoRecords, sizeof(*pm_group_info));
        if (pm_image_id_resp)
            *pm_image_id_resp = p->GroupInfoRecords[0].imageId;

        fstatus = FSUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    if (query_results)
        oib_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = FERROR;
    goto done;
}

/** 
 *  Get group config info
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of group config to get. 
 * @param group_name            Pointer to group name 
 * @param pm_image_id_resp      Pointer to image ID of group info returned. 
 * @param pm_group_config       Pointer to group config to fill. Upon successful return, a memory to 
 *                              contain the group config is allocated. The caller must call
 *                              pa_client_release_group_config to free the memory later. 
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_get_group_config( 
    struct oib_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                *group_name, 
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp, 
	uint32						*pNum_ports,
    STL_PA_PM_GROUP_CFG_RSP   **pm_group_config 
    )
{
    FSTATUS                 fstatus = FERROR;
    uint32                  num_ports;
    uint32                  size_group_config;
    STL_PA_IMAGE_ID_DATA           image_id = pm_image_id_query;
    QUERY                   query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_GROUP_CONFIG_RESULTS *p;

    // Validate the parameters and state
    if (!port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) || 
        !group_name || !pm_group_config || *pm_group_config)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));	// initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    DBGPRINT("Getting Group Config...\n");
    DBGPRINT("Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

    fstatus = iba_pa_multi_mad_group_config_response_query( 
       port, &query, group_name, &query_results, NULL, &image_id );

    if (!query_results)
    {
        DBGPRINT( "PA GroupConfig query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        DBGPRINT( "PA GroupConfig query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        DBGPRINT("No Records Returned\n");
        goto fail;
    }
    else
    {
        p = (STL_PA_GROUP_CONFIG_RESULTS*)query_results->QueryResult;
        
        // TBD remove some of these after additional MAD testing
        DBGPRINT("MadStatus 0x%X: %s\n", port->mad_status,
                  iba_pa_mad_status_msg(port) );
        DBGPRINT("%d Bytes Returned\n", query_results->ResultDataSize);
        DBGPRINT("PA Multiple MAD Response for GroupConfig group %s:\n",
                  group_name );
        DBGPRINT("NumGroupConfigRecords = %d\n",
                  (int)p->NumGroupConfigRecords );
        
        num_ports = p->NumGroupConfigRecords < MAX_PORTS_LIST ?
            p->NumGroupConfigRecords : MAX_PORTS_LIST;
		*pNum_ports = num_ports;
        size_group_config = sizeof(STL_PA_PM_GROUP_CFG_RSP) * num_ports;
        
        if ( !( *pm_group_config = MemoryAllocate2AndClear( size_group_config,
                                                            IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG ) ) )
        {
            OUTPUT_ERROR("can not allocate memory\n");
            goto fail;
        }
        
        DBGPRINT( "\tname:%s, ports:%u\n",
                  group_name,
                  (unsigned int)num_ports );
        
        memcpy(*pm_group_config, p->GroupConfigRecords, size_group_config); 
        fstatus = FSUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    oib_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = FERROR;
    goto done;
}

/** 
 *  Release group config info
 *
 * @param port                  Port to operate on. 
 * @param pm_group_config       Pointer to pointer to the group config to free. 
 *
 * @return 
 *   None
 */
void 
pa_client_release_group_config(
    struct oib_port     *port, 
    STL_PA_PM_GROUP_CFG_RSP   **pm_group_config
    )
{
    if (pm_group_config && *pm_group_config)
    {
        MemoryDeallocate(*pm_group_config);
        *pm_group_config = NULL;
    }

}


/* Get VF config info
 *
 * @param port					Port to operate on
 * @param pm_image_id_query		Image ID of VF config to get.
 * @param vf_name				Pointer to VF name.
 * @param pm_image_id_resp		Pointer to image ID of VF info returned.
 * @param pm_vf_config			Pointer to VF config to fill. Upon successful return, a memory to
 * 								contain the VF config is allocated. The caller must call
 * 								pa_client_release_vf_config to free the memory later.
 *
 * 	@return
 * 	  FSUCCESS - Get successful
 * 	    FERROR - Error
 */
FSTATUS
pa_client_get_vf_config(
	struct oib_port				*port,
	STL_PA_IMAGE_ID_DATA		pm_image_id_query,
	char						*vf_name,
	STL_PA_IMAGE_ID_DATA		*pm_image_id_resp,
	uint32						*pNum_ports,
	STL_PA_VF_CFG_RSP		**pm_vf_config
	)
{
    FSTATUS                 fstatus = FERROR;
    uint32                  num_ports;
    uint32                  size_vf_config;
    STL_PA_IMAGE_ID_DATA    image_id = pm_image_id_query;
    QUERY                   query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_VF_CONFIG_RESULTS *p;

    // Validate the parameters and state
    if (!port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) || 
        !vf_name || !pm_vf_config || *pm_vf_config)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));	// initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    DBGPRINT("Getting Group Config...\n");
    DBGPRINT("Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

    fstatus = iba_pa_multi_mad_vf_config_response_query( 
       port, &query, vf_name, &query_results, NULL, &image_id );

    if (!query_results)
    {
        DBGPRINT( "PA VFConfig query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        DBGPRINT( "PA VFConfig query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        DBGPRINT("No Records Returned\n");
        goto fail;
    }
    else
    {
        p = (STL_PA_VF_CONFIG_RESULTS*)query_results->QueryResult;
        
        // TBD remove some of these after additional MAD testing
        DBGPRINT("MadStatus 0x%X: %s\n", port->mad_status,
                  iba_pa_mad_status_msg(port) );
        DBGPRINT("%d Bytes Returned\n", query_results->ResultDataSize);
        DBGPRINT("PA Multiple MAD Response for VFConfig group %s:\n",
                  vf_name );
        DBGPRINT("NumVFConfigRecords = %d\n",
                  (int)p->NumVFConfigRecords );
        
        num_ports = p->NumVFConfigRecords < MAX_PORTS_LIST ?
            p->NumVFConfigRecords : MAX_PORTS_LIST;
		*pNum_ports = num_ports;
        size_vf_config = sizeof(STL_PA_VF_CFG_RSP) * num_ports;
        
        if ( !( *pm_vf_config = MemoryAllocate2AndClear( size_vf_config,
                                                            IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG ) ) )
        {
            OUTPUT_ERROR("can not allocate memory\n");
            goto fail;
        }
        
        DBGPRINT("\tname:%s, ports:%u\n",
                  vf_name,
                  (unsigned int)num_ports );
        
        memcpy(*pm_vf_config, p->VFConfigRecords, size_vf_config); 
        fstatus = FSUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    oib_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = FERROR;
    goto done;
}


/* Release VF config info
 *
 * @param port					Port to operate on.
 * @param pm_vf_config			Pointer to pointer to the VF config to free
 *
 * @return
 *   None
 */
void
pa_client_release_vf_config(
	struct oib_port				*port,
	STL_PA_VF_CFG_RSP		**pm_vf_config
	)
{
    if (pm_vf_config && *pm_vf_config)
    {
        MemoryDeallocate(*pm_vf_config);
        *pm_vf_config = NULL;
    }
}

/**
 *  Get group focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of group focus portlist to get. 
 * @param group_name            Pointer to group name.
 * @param select                Select value for focus portlist. 
 * @param start                 Start index value of portlist 
 * @param range                 Index range of portlist. 
 * @param pm_image_id_resp      Pointer to image ID of group focus portlist returned. 
 * @param pm_group_focus        Pointer to pointer to focus portlist to fill. Upon 
 *                              successful return, a memory to contain the group focus
 *                              portlist is allocated. The caller must call
 *                              pa_client_release_group_focus to free the memory later.
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */

FSTATUS 
pa_client_get_group_focus( 
    struct oib_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                *group_name, 
    uint32              select, 
    uint32              start, 
    uint32              range,
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp, 
	uint32						*pNum_ports,
    STL_FOCUS_PORTS_RSP    **pm_group_focus 
    )
{
    FSTATUS                 fstatus = FERROR;
    uint32                  num_ports;
    uint32                  size_group_focus;
    STL_PA_IMAGE_ID_DATA           image_id = pm_image_id_query;
    QUERY                   query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_FOCUS_PORTS_RESULTS  *p;

    // Validate the parameters and state
    if (!port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) || 
        !group_name || 
        (range > MAX_PORTS_LIST) ||
        !pm_group_focus || *pm_group_focus)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));   // initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    DBGPRINT("Getting Group Focus...\n");
    DBGPRINT("Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

    fstatus = iba_pa_multi_mad_focus_ports_response_query(port, &query,
		group_name, select, start, range, &query_results, NULL, &image_id );

    if (!query_results)
    {
        DBGPRINT("PA Group Focus query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        DBGPRINT("PA Group Focus query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        DBGPRINT("No Records Returned\n");
        goto fail;
    }
    else
    {
        p = (STL_PA_FOCUS_PORTS_RESULTS*)query_results->QueryResult;
        
        // TBD remove some of these after additional MAD testing
        DBGPRINT("MadStatus 0x%X: %s\n", port->mad_status,
                  iba_pa_mad_status_msg(port) );
        DBGPRINT("%d Bytes Returned\n", query_results->ResultDataSize);
        DBGPRINT("PA Multiple MAD Response for Focus portlist group %s:\n",
                 group_name );
        DBGPRINT("NumFocusPortsRecords = %d\n",
                  (int)p->NumFocusPortsRecords );
        
        num_ports = p->NumFocusPortsRecords < range ?
            p->NumFocusPortsRecords : range;

		*pNum_ports = num_ports;
        size_group_focus = sizeof(STL_FOCUS_PORTS_RSP) * num_ports;
        
        if ( !( *pm_group_focus = MemoryAllocate2AndClear( size_group_focus,
                                                         IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG ) ) )
        {
            OUTPUT_ERROR("can not allocate memory\n");
            goto fail;
        }
        
        DBGPRINT( "\tname:%s, ports:%u\n",
                  group_name,
                  (unsigned int)num_ports );
        
        memcpy(*pm_group_focus, p->FocusPortsRecords, size_group_focus); 
        fstatus = FSUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    oib_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = FERROR;
    goto done;
}

/** 
 *  Release group focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_group_config       Pointer to pointer to the group focus portlist to free. 
 *
 * @return 
 *   None
 */
void 
pa_client_release_group_focus(
    struct oib_port     *port, 
    STL_FOCUS_PORTS_RSP     **pm_group_focus
   )
{
    if (pm_group_focus && *pm_group_focus)
    {
        MemoryDeallocate(*pm_group_focus);
        *pm_group_focus = NULL;
    }
}


/*
 *  Get VF focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of vf focus portlist to get. 
 * @param vf_name            Pointer to vf name.
 * @param select                Select value for focus portlist. 
 * @param start                 Start index value of portlist 
 * @param range                 Index range of portlist. 
 * @param pm_image_id_resp      Pointer to image ID of group focus portlist returned. 
 * @param pm_vf_focus        Pointer to pointer to focus portlist to fill. Upon 
 *                              successful return, a memory to contain the group focus
 *                              portlist is allocated. The caller must call
 *                              pa_client_release_vf_focus to free the memory later.
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */

FSTATUS 
pa_client_get_vf_focus( 
    struct oib_port           *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    char                      *vf_name, 
    uint32                     select, 
    uint32                     start, 
    uint32                     range,
    STL_PA_IMAGE_ID_DATA      *pm_image_id_resp, 
	uint32						*pNum_ports,
    STL_PA_VF_FOCUS_PORTS_RSP     **pm_vf_focus 
    )
{
    FSTATUS                 fstatus = FERROR;
    uint32                  num_ports;
    uint32                  size_vf_focus;
    STL_PA_IMAGE_ID_DATA           image_id = pm_image_id_query;
    QUERY                   query;
    PQUERY_RESULT_VALUES    query_results = NULL;
    STL_PA_VF_FOCUS_PORTS_RESULTS  *p;

    // Validate the parameters and state
    if (!port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) || 
        !vf_name || 
        (range > MAX_PORTS_LIST) ||
        !pm_vf_focus || *pm_vf_focus)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    memset(&query, 0, sizeof(query));   // initialize reserved fields
    query.InputType     = InputTypeNoInput;
    query.OutputType    = OutputTypePaTableRecord;

    DBGPRINT("Getting VF Focus...\n");
    DBGPRINT("Query: Input=%s, Output=%s\n",
             iba_pa_query_input_type_msg(query.InputType),
             iba_pa_query_result_type_msg(query.OutputType));

	fstatus = iba_pa_multi_mad_vf_focus_ports_response_query(port, &query,
		vf_name, select, start, range, &query_results, NULL, &image_id);

    if (!query_results)
    {
        DBGPRINT("PA VF Focus query Failed: %s\n",
                  iba_fstatus_msg(fstatus) );
        goto fail;
    }
    else if (query_results->Status != FSUCCESS)
    {
        DBGPRINT( "PA VF Focus query Failed: %s MadStatus 0x%X: %s\n",
                  iba_fstatus_msg(query_results->Status), port->mad_status,
                  iba_pa_mad_status_msg(port) );
        goto fail;
    }
    else if (query_results->ResultDataSize == 0)
    {
        DBGPRINT("No Records Returned\n");
        goto fail;
    }
    else
    {
        p = (STL_PA_VF_FOCUS_PORTS_RESULTS*)query_results->QueryResult;
        
        // TBD remove some of these after additional MAD testing
        DBGPRINT(" MadStatus 0x%X: %s\n", port->mad_status,
                  iba_pa_mad_status_msg(port) );
        DBGPRINT("%d Bytes Returned\n", query_results->ResultDataSize);
        DBGPRINT("PA Multiple MAD Response for Focus portlist vf %s:\n",
                 vf_name );
        DBGPRINT("NumVFFocusPortsRecords = %d\n",
                  (int)p->NumVFFocusPortsRecords );
        
        num_ports = p->NumVFFocusPortsRecords < range ?
            p->NumVFFocusPortsRecords : range;
		*pNum_ports = num_ports;
        size_vf_focus = sizeof(STL_PA_VF_FOCUS_PORTS_RSP) * num_ports;
        
        if ( !( *pm_vf_focus = MemoryAllocate2AndClear( size_vf_focus,
                                                         IBA_MEM_FLAG_PREMPTABLE, OIB_MEMORY_TAG ) ) )
        {
            OUTPUT_ERROR("can not allocate memory\n");
            goto fail;
        }
        
        DBGPRINT( "\tname:%s, ports:%u\n",
                  vf_name,
                  (unsigned int)num_ports );
        
        memcpy(*pm_vf_focus, p->FocusPortsRecords, size_vf_focus); 
        fstatus = FSUCCESS;
    }

done:
    // iba_pa_query_port_fabric_info will have allocated a result buffer
    // we must free the buffer when we are done with it
    oib_free_query_result_buffer(query_results);
    return (fstatus);

fail:
    fstatus = FERROR;
    goto done;
}


/* 
 *  Release vf focus portlist
 *
 * @param port                  Port to operate on. 
 * @param pm_vf_focus       Pointer to pointer to the vf focus portlist to free. 
 *
 * @return 
 *   None
 */
void 
pa_client_release_vf_focus(
    struct oib_port           *port, 
    STL_PA_VF_FOCUS_PORTS_RSP     **pm_vf_focus
   )
{
    if (pm_vf_focus && *pm_vf_focus)
    {
        MemoryDeallocate(*pm_vf_focus);
        *pm_vf_focus = NULL;
    }
}

/**
 *  Get port statistics (counters)
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of port counters to get. 
 * @param lid                   LID of node.
 * @param port_num              Port number. 
 * @param pm_image_id_resp      Pointer to image ID of port counters returned. 
 * @param port_counters         Pointer to port counters to fill. 
 * @param flags                 Pointer to flags
 * @param delta                 1 for delta counters, 0 for raw image counters.
 * @param user_cntrs            1 for running counters, 0 for image counters. (delta must be 0)
 *
 * @return 
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_get_port_stats( 
    struct oib_port         *port,
    STL_PA_IMAGE_ID_DATA     pm_image_id_query,
    uint16                   lid,
    uint8                    port_num,
    STL_PA_IMAGE_ID_DATA    *pm_image_id_resp,
    STL_PORT_COUNTERS_DATA  *port_counters, 
    uint32                  *flags,
    uint32                   delta,
	uint32                   user_cntrs
    )
{
    FSTATUS                  fstatus = FERROR;
    STL_PA_IMAGE_ID_DATA     image_id = pm_image_id_query;
    STL_PORT_COUNTERS_DATA  *response;

    // Validate the parameters and state
    if (!port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) || 
        !port_counters)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    DBGPRINT("Getting Port Counters...\n");

    if ( ( response = iba_pa_single_mad_port_counters_response_query( 
       port, lid, port_num, delta, user_cntrs, &image_id ) ) != NULL )
    {
        // TBD remove some of these after additional MAD testing
        DBGPRINT("%s Controlled Port Counters (%s) Response for nodeLid 0x%X portNumber %d%s:\n",
				 (user_cntrs?"User":"PM"), (delta?"Delta":"Total"), lid, port_num,
				 response->flags & STL_PA_PC_FLAG_UNEXPECTED_CLEAR?" (Unexpected Clear)":"" );
        DBGPRINT("\tXmitData = %" PRIu64 "\n", response->portXmitData);
        DBGPRINT("\tRcvData = %" PRIu64 "\n", response->portRcvData);
        DBGPRINT("\tXmitPkts = %" PRIu64 "\n", response->portXmitPkts);
        DBGPRINT("\tRcvPkts = %" PRIu64 "\n", response->portRcvPkts);
        DBGPRINT("\tMulticastXmitPkts = %" PRIu64 "\n", response->portMulticastXmitPkts);
        DBGPRINT("\tMulticastRcvPkts = %" PRIu64 "\n", response->portMulticastRcvPkts);

        DBGPRINT("\tLinkQualityIndicator = %u\n", response->lq.s.linkQualityIndicator);
        DBGPRINT("\tUncorrectableErrors = %u\n", response->uncorrectableErrors); // 8 bit
        DBGPRINT("\tLinkDowned = %u\n", response->linkDowned);	// 32 bit
        DBGPRINT("\tNumLanesDown = %u\n", response->lq.s.numLanesDown);
        DBGPRINT("\tRcvErrors = %"PRIu64"\n", response->portRcvErrors);
        DBGPRINT("\tExcessiveBufferOverruns = %"PRIu64"\n",
                response->excessiveBufferOverruns );
        DBGPRINT("\tFMConfigErrors = %"PRIu64"\n",
                  response->fmConfigErrors );
        DBGPRINT( "\tLinkErrorRecovery = %u\n", response->linkErrorRecovery);	// 32 bit
        DBGPRINT("\tLocalLinkIntegrityErrors = %"PRIu64"\n",
                  response->localLinkIntegrityErrors );
        DBGPRINT("\tRcvRemotePhysicalErrors = %"PRIu64"\n",
                  response->portRcvRemotePhysicalErrors);
        DBGPRINT("\tXmitConstraintErrors = %"PRIu64"\n",
                  response->portXmitConstraintErrors );
        DBGPRINT("\tRcvConstraintErrors = %"PRIu64"\n",
                  response->portRcvConstraintErrors );
        DBGPRINT("\tRcvSwitchRelayErrors = %"PRIu64"\n",
                  response->portRcvSwitchRelayErrors);
        DBGPRINT("\tXmitDiscards = %"PRIu64"\n", response->portXmitDiscards);
        DBGPRINT("\tCongDiscards = %"PRIu64"\n",
                  response->swPortCongestion);
        DBGPRINT("\tRcvFECN = %"PRIu64"\n", response->portRcvFECN);
        DBGPRINT("\tRcvBECN = %"PRIu64"\n", response->portRcvBECN);
        DBGPRINT("\tMarkFECN = %"PRIu64"\n", response->portMarkFECN);
        DBGPRINT("\tXmitTimeCong = %"PRIu64"\n", response->portXmitTimeCong);
        DBGPRINT("\tXmitWait = %"PRIu64"\n", response->portXmitWait);
        DBGPRINT("\tXmitWastedBW = %"PRIu64"\n", response->portXmitWastedBW);
        DBGPRINT("\tXmitWaitData = %"PRIu64"\n", response->portXmitWaitData);
        DBGPRINT("\tRcvBubble = %"PRIu64"\n", response->portRcvBubble);
        if (pm_image_id_resp)
            *pm_image_id_resp = response->imageId;
        if (flags)
            *flags = response->flags;
        
        memcpy(port_counters, response, sizeof(*port_counters));
        MemoryDeallocate(response);
        fstatus = FSUCCESS;
    }
    else
        DBGPRINT("Got NULL response - FAILED\n");

    return (fstatus);
}

/**
 *  Clear specified port counters for specified port
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of port counters to clear. 
 * @param lid                   LID of port.
 * @param port_num              Port number. 
 * @param selct_flags           Port's counters to clear. 
 *
 * @return 
 *   FSUCCESS - Clear successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_clr_port_counters( 
    struct oib_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query, 
    uint16              lid,
    uint8               port_num, 
    uint32              select_flag 
    )
{
    FSTATUS                 fstatus = FERROR;
    STL_CLR_PORT_COUNTERS_DATA  *response;
    
    // Validate the parameters and state
    if (!port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) )
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }
  
    if ( ( response =
           iba_pa_single_mad_clr_port_counters_response_query(
               port, lid, port_num, select_flag ) ) )
    {
        MemoryDeallocate(response);
        fstatus = FSUCCESS;
    }

    return (fstatus);
}
/**
 *  Get vf port statistics (counters)
 *
 * @param port                  Port to operate on.
 * @param pm_image_id_query     Image ID of port counters to get.
 * @param vf_name				Pointer to VF name.
 * @param lid                   LID of node.
 * @param port_num              Port number.
 * @param pm_image_id_resp      Pointer to image ID of port counters returned.
 * @param vf_port_counters      Pointer to vf port counters to fill.
 * @param flags                 Pointer to flags
 * @param delta                 1 for delta counters, 0 for raw image counters.
 * @param user_cntrs            1 for running counters, 0 for image counters. (delta must be 0)
 *
 * @return
 *   FSUCCESS - Get successful
 *     FERROR - Error
 */
FSTATUS
pa_client_get_vf_port_stats(
    struct oib_port              *port,
    STL_PA_IMAGE_ID_DATA          pm_image_id_query,
	char				         *vf_name,
    uint16                        lid,
    uint8                         port_num,
    STL_PA_IMAGE_ID_DATA         *pm_image_id_resp,
    STL_PA_VF_PORT_COUNTERS_DATA *vf_port_counters,
    uint32                       *flags,
    uint32                        delta,
	uint32                        user_cntrs
    )
{
	FSTATUS                       fstatus = FERROR;
    STL_PA_IMAGE_ID_DATA          image_id = pm_image_id_query;
    STL_PA_VF_PORT_COUNTERS_DATA *response;

    // Validate the parameters and state
    if (!port || (port->pa_client_state != PACLIENT_OPERATIONAL) || !vf_port_counters)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    DBGPRINT("Getting Port Counters...\n");

    if ( ( response = iba_pa_single_mad_vf_port_counters_response_query(
       port, lid, port_num, delta, user_cntrs, vf_name, &image_id ) ) != NULL )
    {
        // TBD remove some of these after additional MAD testing
        DBGPRINT("%s Controlled VF Port Counters (%s) Response for nodeLid 0x%X portNumber %d%s:\n",
				 (user_cntrs?"User":"PM"), (delta?"Delta":"Total"), lid, port_num,
				 response->flags & STL_PA_PC_FLAG_UNEXPECTED_CLEAR?" (Unexpected Clear)":"" );
        DBGPRINT("\tvfName = %s\n", response->vfName);
        DBGPRINT("\tXmitData = %" PRIu64 "\n", response->portVFXmitData);
        DBGPRINT("\tRcvData = %" PRIu64 "\n", response->portVFRcvData);
        DBGPRINT("\tXmitPkts = %" PRIu64 "\n", response->portVFXmitPkts);
        DBGPRINT("\tRcvPkts = %" PRIu64 "\n", response->portVFRcvPkts);
        DBGPRINT("\tXmitDiscards = %"PRIu64"\n", response->portVFXmitDiscards);
        DBGPRINT("\tCongDiscards = %"PRIu64"\n", response->swPortVFCongestion);
        DBGPRINT("\tRcvFECN = %"PRIu64"\n", response->portVFRcvFECN);
        DBGPRINT("\tRcvBECN = %"PRIu64"\n", response->portVFRcvBECN);
        DBGPRINT("\tMarkFECN = %"PRIu64"\n", response->portVFMarkFECN);
        DBGPRINT("\tXmitTimeCong = %"PRIu64"\n", response->portVFXmitTimeCong);
        DBGPRINT("\tXmitWait = %"PRIu64"\n", response->portVFXmitWait);
        DBGPRINT("\tXmitWastedBW = %"PRIu64"\n", response->portVFXmitWastedBW);
        DBGPRINT("\tXmitWaitData = %"PRIu64"\n", response->portVFXmitWaitData);
        DBGPRINT("\tRcvBubble = %"PRIu64"\n", response->portVFRcvBubble);
        if (pm_image_id_resp)
            *pm_image_id_resp = response->imageId;
        if (flags)
            *flags = response->flags;

        memcpy(vf_port_counters, response, sizeof(*vf_port_counters));
        MemoryDeallocate(response);
        fstatus = FSUCCESS;
    }
    else
        DBGPRINT("Got NULL response - FAILED\n");

    return (fstatus);
}

/**
 *  Freeze specified image
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to free. 
 * @param pm_image_id_resp      Pointer to image ID of image frozen. 
 *
 * @return 
 *   FSUCCESS - Freeze successful
 *     FERROR - Error
 */
FSTATUS 
pa_client_freeze_image( 
    struct oib_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query,
    STL_PA_IMAGE_ID_DATA       *pm_image_id_resp 
    )
{
    FSTATUS         fstatus = FERROR;
    STL_PA_IMAGE_ID_DATA   image_id = pm_image_id_query;
    STL_PA_IMAGE_ID_DATA   *response;

    // Validate the parameters and state
    if (!port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) )
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    DBGPRINT("QueryImageNum = 0x%" PRIX64 " Offset = %d\n",
              pm_image_id_query.imageNumber, (int)pm_image_id_query.imageOffset );

    if ( ( response =
           iba_pa_single_mad_freeze_image_response_query(
              port, &image_id ) ) != NULL )
    {
        DBGPRINT("RespImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->imageNumber, (int)response->imageOffset );
        
        if (pm_image_id_resp)
            *pm_image_id_resp = *response;
        
        MemoryDeallocate(response);
        fstatus = FSUCCESS;
    }
    else
        DBGPRINT("Got NULL response - FAILED\n");

    return (fstatus);
}

/**
 *   Move freeze of image 1 to image 2
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of frozen image 1. 
 * @param pm_image_id_resp      Pointer to image ID of image2. 
 *  
 * @return 
 *   FSUCCESS       - Move image Freeze successful
 *   FUNAVAILABLE   - Image 2 unavailable freeze
 *     FERROR       - Error
 */
FSTATUS 
pa_client_move_image_freeze(
    struct oib_port     *port, 
    STL_PA_IMAGE_ID_DATA       pm_image_id1,
    STL_PA_IMAGE_ID_DATA       *pm_image_id2 
    )
{
    FSTATUS fstatus     = FERROR;
    STL_MOVE_FREEZE_DATA    request;
    STL_MOVE_FREEZE_DATA    *response;

    // Validate the parameters and state
    if (!port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL)  ||
        !pm_image_id2)
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    DBGPRINT("Img1ImageNum = 0x%" PRIX64 " Offset = %d\n",
              pm_image_id1.imageNumber, (int)pm_image_id1.imageOffset );
    DBGPRINT("Img2ImageNum = 0x%" PRIX64 " Offset = %d\n",
              pm_image_id2->imageNumber, (int)pm_image_id2->imageOffset );
    request.oldFreezeImage = pm_image_id1;
    request.newFreezeImage = *pm_image_id2;

    if ( ( response =
           iba_pa_single_mad_move_freeze_response_query( 
              port, &request ) ) != NULL )
    {
        DBGPRINT("RespOldImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->oldFreezeImage.imageNumber, response->oldFreezeImage.imageOffset );
        DBGPRINT("RespNewImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->newFreezeImage.imageNumber, response->newFreezeImage.imageOffset );
         
        *pm_image_id2 = response->newFreezeImage;
        fstatus = FSUCCESS;
        MemoryDeallocate(response);
    }
    else
    {
        DBGPRINT("Got NULL response - UNAVAILABLE\n");
        fstatus = FUNAVAILABLE;
    }

    return (fstatus);
}

/**
 *   Release specified image.
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to release. 
 *  
 * @return 
 *   FSUCCESS       - Release successful
 *     FERROR       - Error
 */
FSTATUS 
pa_client_release_image(
    struct oib_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query
    )
{
    FSTATUS         fstatus = FERROR;
    STL_PA_IMAGE_ID_DATA   image_id = pm_image_id_query;
    STL_PA_IMAGE_ID_DATA   *response;

    // Validate the parameters and state
    if (!port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) )
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    if ( ( response = iba_pa_single_mad_release_image_response_query( 
               port, &image_id ) ) != NULL )
    {
        // TBD remove some of these after additional MAD testing
        DBGPRINT("QueryImageNum = 0x%" PRIX64 " Offset = %d\n",
                  pm_image_id_query.imageNumber, (int)pm_image_id_query.imageOffset );
        DBGPRINT("RespImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->imageNumber, (int)response->imageOffset );
        
        MemoryDeallocate(response);
        fstatus = FSUCCESS;
    }
    else
        DBGPRINT("Got NULL response - FAILED\n");

    return (fstatus);
}

/**
 *   Renew lease of specified image.
 *
 * @param port                  Port to operate on. 
 * @param pm_image_id_query     Image ID of image to renew. 
 *  
 * @return 
 *   FSUCCESS       - Renew successful
 *     FERROR       - Error
 */
FSTATUS 
pa_client_renew_image(
    struct oib_port     *port,
    STL_PA_IMAGE_ID_DATA       pm_image_id_query
    )
{
    FSTATUS         fstatus = FERROR;
    STL_PA_IMAGE_ID_DATA   image_id = pm_image_id_query;
    STL_PA_IMAGE_ID_DATA   *response;

    // Validate the parameters and state
    if (!port || 
        (port->pa_client_state != PACLIENT_OPERATIONAL) )
    {
        OUTPUT_ERROR("invalid params or state\n");
        return (fstatus);
    }

    if ( ( response = iba_pa_single_mad_renew_image_response_query( 
       port, &image_id ) ) != NULL )
    {
        // TBD remove some of these after additional MAD testing
        DBGPRINT("QueryImageNum = 0x%" PRIX64 " Offset = %d\n",
                  pm_image_id_query.imageNumber, (int)pm_image_id_query.imageOffset );
        DBGPRINT("RespImageNum = 0x%" PRIX64 " Offset = %d\n",
                  response->imageNumber, (int)response->imageOffset );
        
        MemoryDeallocate(response);
        fstatus = FSUCCESS;
    }
    else
        DBGPRINT("Got NULL response - FAILED\n");

    return (fstatus);
}

#endif
