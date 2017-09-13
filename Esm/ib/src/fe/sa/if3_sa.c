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

 * ** END_ICS_COPYRIGHT5   ****************************************/

/***********************************************************************
* 
* FILE NAME
*   if3_sa.c
*
* DESCRIPTION
*   Interface 3 functions which commuincate with the SA 
*
*
***********************************************************************/
#include "if3_sa.h"
#include "fe_cmds.h"
#include <fe_main.h>
#include "if3.h"
#include "ib_sa.h"
#include <fm_xml.h>

#ifdef IB_STACK_OPENIB
#include "opamgt_sa_priv.h"
#endif

extern FEXmlConfig_t fe_config;
#ifdef IB_STACK_OPENIB
extern struct omgt_port *fe_omgt_session;
#endif

IBhandle_t filterh = IB_NULLH;
uint32_t fe_in_buff_size = 0;
uint32_t fe_prts_size = 0;
uint32_t fe_lnks_size = 0;

// The FE_SA_RECV_BUF_SIZE is only used for small queries:
//		1 HFIs worth of NodeRecords
//		1 SwitchInfoRecord
//		1 Switch's LFT or MFT (whole table)
//		1 port's PKey, VL2VL or VLArb (whole table)
// Of the above for a large fabric, the biggest would be the LFT at 48K or
// MFT at 15K (36 port 1024 entries).  For a small fabric LFT would be
// much smaller (6K at 8 lids per 800 nodes) but MFT could be big if lots
// of multicast.  So we size with some headroom on VxWorks but just make it
// safely fat on Host SM
#ifdef __VXWORKS__
#define FE_SA_RECV_BUF_SIZE (16*1024)
#else
#define FE_SA_RECV_BUF_SIZE (1024*1024) // more than we should ever need
#endif

static uint8_t fe_sa_recv_buf[FE_SA_RECV_BUF_SIZE];
static uint8_t fe_sa_send_buf[2*SAMAD_DATA_COUNT];


uint32_t
fe_if3_getInfo(Packet_t* pack)
{
  SA_MAD mad;
  uint32_t rc;

  IB_ENTER(__func__,pack->aid,pack->amod,pack->method,pack->mask);

  // initialize the commom mad header fields of the SA MAD
  memset((void *)(&mad),0,sizeof(mad)); 

  MAD_SET_METHOD_TYPE(&mad, pack->method;); 
  MAD_SET_VERSION_INFO(&mad, STL_BASE_VERSION, MCLASS_SUBN_ADM, STL_SA_CLASS_VERSION); 
  MAD_SET_ATTRIB_ID(&mad,pack->aid); 
  MAD_SET_ATTRIB_MOD(&mad, pack->amod);

  // initialize SA MAD header fields    
  SA_MAD_SET_HEADER(&mad, SM_KEY, pack->mask); 

  // initialize SA MAD data field
  if (pack->recsize>0) memcpy(mad.Data, pack->rec, pack->recsize);

  // send query to SA and wait for result
  rc = if3_mngr_send_mad(fdsa,&mad,pack->recsize, pack->data,&pack->sz,&pack->status, NULL, NULL); 
  
  IB_EXIT(__func__,rc);
  return rc;
}

uint32_t
fe_if3_get_node_by_port_guid(uint64_t portguid, STL_NODE_RECORD* nr)
{
  Packet_t pack;
  uint32_t rc, len;  
  uint64_t pguid = 0;
  STL_NODE_RECORD nrec;  
  uint8_t* ptr;

  IB_ENTER(__func__,portguid,0,0,0);
  
  memset(&nrec,0,sizeof(nrec));

  /* Getting  all nodes */
  pack.aid = SA_NODE_RECORD;
  pack.amod = 0;
  /* match on node Guid */
  nrec.NodeInfo.PortGUID = portguid;
  BSWAPCOPY_STL_NODE_RECORD(&nrec, (STL_NODE_RECORD *)fe_sa_send_buf);

  pack.method = SA_CM_GETTABLE;
  pack.mask = NR_COMPONENTMASK_PORTGUID;     // match on portGuid
  pack.recsize = sizeof(STL_NODE_RECORD);
  pack.rec = fe_sa_send_buf;
  pack.data = fe_sa_recv_buf;
  memset(pack.data,0,FE_SA_RECV_BUF_SIZE);
  pack.sz = FE_SA_RECV_BUF_SIZE;
  rc = fe_if3_getInfo(&pack);

  if (rc != VSTATUS_OK) {
      IB_LOG_ERRORRC("Query Failed rc:",rc);
      rc = FE_NO_RETRIEVE;
      IB_EXIT(__func__,rc);
      return rc;
  } else if (pack.sz == 0) {
      rc = FE_NO_RETRIEVE;
      IB_EXIT(__func__,rc);
      return rc;
  } else if (pack.status != 0) {
      rc = FE_NO_RETRIEVE;
      IB_LOG_ERROR("Invalid status getting nodes status:",pack.status);
      IB_EXIT(__func__,rc);
      return rc;
  }

  ptr = pack.data;
  len = 0;

  /* Finding requested node */
  while (len < pack.sz) {
      BSWAPCOPY_STL_NODE_RECORD((STL_NODE_RECORD *)ptr, nr);

      ptr += sizeof(STL_NODE_RECORD) + Calculate_Padding(sizeof(STL_NODE_RECORD));
      len += sizeof(STL_NODE_RECORD) + Calculate_Padding(sizeof(STL_NODE_RECORD));
      pguid = nr->NodeInfo.PortGUID;
      if (pguid == portguid) break;
  }  

  if (len >= pack.sz) {
      if (pguid != portguid) {
          rc = FE_GUID_INVALID;
          IB_LOG_INFINI_INFO("SA does not have port GUID", portguid);
          IB_EXIT(__func__,rc);
          return rc;
      }
  }

  rc = FE_SUCCESS;
  
  IB_EXIT(__func__,rc);
  return rc;
} // end fe_if3_get_node_by_port_guid

uint32_t
fe_if3_sa_check()
{ 
    Packet_t pack; 
    ClassPortInfo_t cpinfo; 
    uint32_t rc = FE_SUCCESS;  
    
    IB_ENTER(__func__, 0, 0, 0, 0);
     
    memset(&cpinfo, 0, sizeof(cpinfo)); 
    
    // initialize packet for ClassPortInfo query
    pack.aid = SA_CLASSPORTINFO; 
    pack.amod = 0; 
    pack.method = SA_CM_GET; 
    //pack.endRid = 0xFFFFFFFF;
    pack.mask = 0; 
    pack.rec = (uint8_t *)&cpinfo; 
    pack.recsize = 0; 
    pack.data = fe_sa_recv_buf; 
    memset(pack.data, 0, FE_SA_RECV_BUF_SIZE); 
    pack.sz = FE_SA_RECV_BUF_SIZE; 
    
    rc = fe_if3_getInfo(&pack);     
    if (rc != VSTATUS_OK) {
        cs_log(_FE_TRACE(fe_config.debug), __func__, 
               "check of FE communications channel to SA has Failed, status=%d", rc); 
        rc = FE_NO_RETRIEVE;
    }

    IB_EXIT(__func__, rc); 
    return rc;
}

uint32_t
fe_if3_subscribe_sa()
{ 
#ifdef IB_STACK_OPENIB
  FSTATUS status;
  
  IB_ENTER(__func__,0,0,0,0);
  
  status = omgt_sa_register_trap(fe_omgt_session,MAD_SMT_PORT_UP,NULL);
  if (status != FSUCCESS) {
    IB_LOG_INFINI_INFO("Failed to register for PORT UP notices, status:", status);
    IB_EXIT(__func__, FE_NO_RETRIEVE);
    return FE_NO_RETRIEVE;
  }
  
  status = omgt_sa_register_trap(fe_omgt_session,MAD_SMT_PORT_DOWN,NULL);
  if (status != FSUCCESS) {
    IB_LOG_INFINI_INFO("Failed to register for PORT DOWN notices, status:", status);
    IB_EXIT(__func__, FE_NO_RETRIEVE);
    return FE_NO_RETRIEVE;
  }
  
#else

  // TBD TODD - replace with calls to IbAccess sd to register for notices
    SA_MAD mad;
  uint32_t rc;
  uint32_t sz = FE_SA_RECV_BUF_SIZE, madRc = 0;
  STL_INFORM_INFO info;

  IB_ENTER(__func__,0,0,0,0);
  memset(&info,0,sizeof(info)); 
    
    // subscibe for Port Down 65
    info.LIDRangeBegin = STL_LID_PERMISSIVE;
  info.IsGeneric = 1;
  info.Subscribe = 1;
  info.Type = TRAP_ALL;
  info.u.Generic.TrapNumber = MAD_SMT_PORT_DOWN;
  info.u.Generic.u1.s.RespTimeValue = 18;
  info.u.Generic.u2.s.ProducerType = INFORMINFO_PRODUCERTYPE_SM; 
    
    memset((void *)(&mad), 0, sizeof(mad)); 
    
    // initialize SA MAD header fields    
    SA_MAD_SET_HEADER(&mad, SM_KEY, 0); 
    
    // initialize the commom mad header fields of the SA MAD
    MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_SET); 
    MAD_SET_VERSION_INFO(&mad, STL_BASE_VERSION, MCLASS_SUBN_ADM, STL_SA_CLASS_VERSION); 
    MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_INFORM_INFO); 
    
    // initialize SA MAD payload
    BSWAPCOPY_STL_INFORM_INFO(&info, (STL_INFORM_INFO *)&mad.Data); 
    
    // send command to the SA
    rc = if3_mngr_send_mad(fdsa,&mad, sizeof(STL_INFORM_INFO),fe_sa_recv_buf,&sz,&madRc, NULL, NULL);
  if (rc != VSTATUS_OK || madRc != 0) {
      if (rc != VSTATUS_OK) {
          IB_LOG_INFINI_INFORC("Failed when trying to subscribe, rc:", rc);
      } else {
          IB_LOG_INFINI_INFO("Invalid status when subscribing, status=", madRc);
      }
      rc = FE_NO_RETRIEVE;
      IB_EXIT(__func__,rc);
      return rc;
  } else {
        // subscibe for Port Up 64
        memset(&info,0,sizeof(info));
      info.LIDRangeBegin = STL_LID_PERMISSIVE;
      info.IsGeneric = 1;
      info.Subscribe = 1;
      info.Type = TRAP_ALL;
      info.u.Generic.TrapNumber = MAD_SMT_PORT_UP;
      info.u.Generic.u1.s.RespTimeValue = 18;
      info.u.Generic.u2.s.ProducerType = INFORMINFO_PRODUCERTYPE_SM; 
        
        memset((void *)(&mad), 0, sizeof(mad)); 
        
        // initialize SA MAD header fields    
        SA_MAD_SET_HEADER(&mad, SM_KEY, 0); 
        
        // initialize the commom mad header fields of the SA MAD
        MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_SET); 
        MAD_SET_VERSION_INFO(&mad, STL_BASE_VERSION, MCLASS_SUBN_ADM, STL_SA_CLASS_VERSION); 
        MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_INFORM_INFO); 
        
        // initialize SA MAD payload
        BSWAPCOPY_STL_INFORM_INFO(&info, (STL_INFORM_INFO *)&mad.Data); 
        
        // send command to the SA
        rc = if3_mngr_send_mad(fdsa,&mad, sizeof(STL_INFORM_INFO),fe_sa_recv_buf,&sz,&madRc, NULL, NULL);
      if (rc != VSTATUS_OK || madRc != 0) {
          if (rc != VSTATUS_OK) {
              IB_LOG_INFINI_INFORC("Failed when trying to subscribe, rc:", rc);
          } else {
              IB_LOG_INFINI_INFO("Invalid status when subscribing, status=", madRc);
          }
          rc = FE_NO_RETRIEVE;
          IB_EXIT(__func__,rc);
          return rc;
      }
  }
    
    // create filters to listen for traps now
    rc = mai_filter_method(fdsa,VFILTER_SHARE,MAI_TYPE_ANY,&filterh,MAD_CV_SUBN_ADM,MAD_CM_REPORT);
  if (rc != VSTATUS_OK)
    {
      IB_LOG_ERRORRC("Failed to create filter to receive traps rc:",rc);
      rc = FE_NO_RETRIEVE;
      IB_EXIT(__func__,rc);
      return rc;
    }

#endif

  IB_EXIT(__func__,FE_SUCCESS);
  return FE_SUCCESS;
}

uint32_t
fe_if3_unsubscribe_sa(int doUnsubscribe)
{ 
#ifdef IB_STACK_OPENIB
    FSTATUS status; 
    
    IB_ENTER(__func__, 0, 0, 0, 0); 
    
    status = omgt_sa_unregister_trap(fe_omgt_session, MAD_SMT_PORT_UP); 
    if (status != FSUCCESS) {
        IB_LOG_INFINI_INFO("Failed to unregister for PORT UP notices, status:", status); 
        IB_EXIT(__func__, FE_NO_RETRIEVE); 
        return FE_NO_RETRIEVE;
    }
    
    status = omgt_sa_unregister_trap(fe_omgt_session, MAD_SMT_PORT_DOWN); 
    if (status != FSUCCESS) {
        IB_LOG_INFINI_INFO("Failed to unregister for PORT DOWN notices, status:", status); 
        IB_EXIT(__func__, FE_NO_RETRIEVE); 
        return FE_NO_RETRIEVE;
    }
    
#else
    
    SA_MAD mad; 
    uint32_t rc, sz = FE_SA_RECV_BUF_SIZE, madRc = 0; 
    STL_INFORM_INFO info; 
    
    IB_ENTER(__func__, 0, 0, 0, 0); 
    
    if (fdsa == INVALID_HANDLE) {
        rc = FE_NO_RETRIEVE; 
        IB_LOG_WARN0("fdsa IBHandle is not valid"); 
        IB_EXIT(__func__, rc); 
        return rc;
    }
    
    if (doUnsubscribe) {
        // unsubscribe for port down traps
        memset(&info, 0, sizeof(info)); 
        info.LIDRangeBegin = STL_LID_PERMISSIVE; 
        info.IsGeneric = 1; 
        info.Subscribe = 0; 
        info.Type = TRAP_ALL; 
        info.u.Generic.TrapNumber = MAD_SMT_PORT_DOWN; 
        info.u.Generic.u1.s.QPNumber = 1;     /* FE always uses qp 1 */
        info.u.Generic.u1.s.RespTimeValue = 18; 
        info.u.Generic.u2.s.ProducerType = INFORMINFO_PRODUCERTYPE_SM; 
        
        memset((void *)(&mad), 0, sizeof(mad)); 
        
        // initialize SA MAD header fields    
        SA_MAD_SET_HEADER(&mad, SM_KEY, 0); 
        
        // initialize the commom mad header fields of the SA MAD
        MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_SET); 
        MAD_SET_VERSION_INFO(&mad, STL_BASE_VERSION, MCLASS_SUBN_ADM, STL_SA_CLASS_VERSION); 
        MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_INFORM_INFO); 
        
        // initialize SA MAD payload
        BSWAPCOPY_STL_INFORM_INFO(&info, (STL_INFORM_INFO *)&mad.Data); 
        
        // send command to the SA
        rc = if3_mngr_send_mad(fdsa, &mad, sizeof(STL_INFORM_INFO), fe_sa_recv_buf, &sz, &madRc, NULL, NULL); 
        if (rc != VSTATUS_OK) {
            IB_LOG_INFINI_INFORC("Failed when trying to unsubscribe for MAD_SMT_PORT_DOWN rc:", rc);
        } else if (rc != VSTATUS_OK && madRc != 0x300) { // ignore status code record not found, just means it was already removed
            IB_LOG_INFINI_INFO("Invalid status: unsubscribing for MAD_SMT_PORT_DOWN status:", madRc);
        }
        
        //
        // unsubscribe for port up traps
        sz = FE_SA_RECV_BUF_SIZE; 
        memset(&info, 0, sizeof(info)); 
        info.LIDRangeBegin = STL_LID_PERMISSIVE; 
        info.IsGeneric = 1; 
        info.Subscribe = 0; 
        info.Type = TRAP_ALL; 
        info.u.Generic.TrapNumber = MAD_SMT_PORT_UP; 
        info.u.Generic.u1.s.QPNumber = 1;     /* FE always uses qp 1 */
        info.u.Generic.u1.s.RespTimeValue = 18; 
        info.u.Generic.u2.s.ProducerType = INFORMINFO_PRODUCERTYPE_SM; 
        
        memset((void *)(&mad), 0, sizeof(mad)); 
        
        // initialize SA MAD header fields    
        SA_MAD_SET_HEADER(&mad, SM_KEY, 0); 
        
        // initialize the commom mad header fields of the SA MAD
        MAD_SET_METHOD_TYPE(&mad, SUBN_ADM_SET); 
        MAD_SET_VERSION_INFO(&mad, STL_BASE_VERSION, MCLASS_SUBN_ADM, STL_SA_CLASS_VERSION); 
        MAD_SET_ATTRIB_ID(&mad, SA_ATTRIB_INFORM_INFO); 
        
        // initialize SA MAD payload
        BSWAPCOPY_STL_INFORM_INFO(&info, (STL_INFORM_INFO *)&mad.Data); 
        
        // send command to the SA
        rc = if3_mngr_send_mad(fdsa, &mad, sizeof(STL_INFORM_INFO), fe_sa_recv_buf, &sz, &madRc, NULL, NULL); 
        if (rc != VSTATUS_OK) {
            IB_LOG_INFINI_INFORC("Failed when trying to unsubscribe for MAD_SMT_PORT_UP rc:", rc);
        } else if (rc != VSTATUS_OK && madRc != 0x300) { // ignore status code record not found, just means it was already removed
            IB_LOG_INFINI_INFO("Invalid status: unsubscribing for MAD_SMT_PORT_UP status:", madRc);
        }
    }
    
    // delete filters to listen for traps now
    if (filterh != IB_NULLH) {
        rc = mai_filter_hdelete(fdsa, filterh); 
        if (rc != VSTATUS_OK) {
            IB_LOG_INFINI_INFORC("Failed when trying to delete filter rc:", rc); 
            rc = FE_NO_RETRIEVE; 
            IB_EXIT(__func__, rc); 
            return rc;
        }
        filterh = IB_NULLH;
    }
#endif
    
    IB_EXIT(__func__, FE_SUCCESS); 
    return FE_SUCCESS;
}

static uint64_t   prevTid=0;
static uint64_t   prevGuid=0;
static uint16_t   prevTrapNum=0;

void fe_if3_ib_notice_to_fe_trap(STL_NOTICE *notice, FE_Trap_t *current, FE_Trap_Processing_State_t *state)
{ 
    uint64_t guid; 
    
    switch (notice->Attributes.Generic.TrapNumber) {
    case STL_TRAP_GID_NOW_IN_SERVICE:   // trap 64
        {
            STL_NOTICE trap64; 
            STL_NODE_RECORD nr = {{0}}; 
            STL_TRAP_GID * trapDataDetails = (STL_TRAP_GID *)notice->Data;
            
            memcpy(&trap64, notice, sizeof(trap64)); 
            state->smlid = notice->IssuerLID; 
            memcpy((void *)&guid, &trapDataDetails->Gid.AsReg64s.H, sizeof(guid)); 
            if (guid != prevGuid || notice->Attributes.Generic.TrapNumber != prevTrapNum) {
                current->trapType = SUBNET_PORT_ACTIVE; 
                if (fe_if3_get_node_by_port_guid(guid, &nr) == FE_SUCCESS) {
                    current->lidAddr = nr.RID.LID; 
                    current->portNum = nr.NodeInfo.u1.s.LocalPortNum; 
                    cs_log(_FE_TRACE(fe_config.debug), "fe_if3_get_traps", 
                           "portGuid 0x%"CS64"X, portNum %d, lid 0x%X has become ACTIVE", 
                           guid, current->portNum, current->lidAddr);
                } else {
                    current->lidAddr = trap64.IssuerLID; // BUGBUG Was trap.lid but this field doesn't exist anymore
                    current->portNum = 0; // BUGBUG Was trap.portno but this field doesn't exist anymore
                    cs_log(_FE_TRACE(fe_config.debug), "fe_if3_get_traps", 
                           "portGuid 0x%"CS64"X has become ACTIVE; no further info", guid);
                }
                ++state->found; 
                state->toBeProcessed = 1; 
                prevGuid = guid; 
                prevTrapNum = notice->Attributes.Generic.TrapNumber;
            } else {
                cs_log(_FE_TRACE(fe_config.debug), "fe_if3_get_traps", 
                       "Duplicate Trap 64 (port active) on portGuid 0x%"CS64"X", guid);
            }
        }
        break; 
        
    case STL_TRAP_GID_OUT_OF_SERVICE:   // trap 65
        {
            STL_NOTICE trap65; 
            STL_NODE_RECORD nr; 
            STL_TRAP_GID * trapDataDetails = (STL_TRAP_GID *)notice->Data;
            
            memcpy(&trap65, notice, sizeof(trap65)); 
            state->smlid = notice->IssuerLID; 
            memcpy((void *)&guid, &trapDataDetails->Gid.AsReg64s.H, sizeof(guid)); 
            if (guid != prevGuid || notice->Attributes.Generic.TrapNumber != prevTrapNum) {
                current->trapType = SUBNET_PORT_INACTIVE; 
                if (fe_if3_get_node_by_port_guid(guid, &nr) == FE_SUCCESS) {
                    current->lidAddr = nr.RID.LID; 
                    current->portNum = nr.NodeInfo.u1.s.LocalPortNum; 
                    current->trapType = SUBNET_PORT_INACTIVE; 
                    cs_log(_FE_TRACE(fe_config.debug), "fe_if3_get_traps", 
                           "portGuid 0x%"CS64"X, portNum %d, lid 0x%X has become INACTIVE", 
                           guid, current->portNum, current->lidAddr);
                } else {
                    current->lidAddr = trap65.IssuerLID; // BUGBUG Was trap.lid but this field doesn't exist anymore
                    current->portNum = 0; // BUGBUG Was trap.portno but this field doesn't exist anymore
                    cs_log(_FE_TRACE(fe_config.debug), "fe_if3_get_traps", 
                           "portGuid 0x%"CS64"X has become INACTIVE; no further info", guid);
                }
                ++state->found; 
                state->toBeProcessed = 1; 
                prevGuid = guid; 
                prevTrapNum = notice->Attributes.Generic.TrapNumber;
            } else {
                cs_log(_FE_TRACE(fe_config.debug), "fe_if3_get_traps", 
                       "Duplicate Trap 65 (port inactive) on portGuid 0x%"CS64"X", guid);
            }
        }
        break; 
        
    case STL_TRAP_ADD_MULTICAST_GROUP:  // trap 66
        {
            STL_NOTICE trap66; 
            
            memcpy(&trap66, notice, sizeof(trap66)); 
            IB_LOG_INFO0("Trap 66");
        }
        break; 
        
    case STL_TRAP_DEL_MULTICAST_GROUP:  // trap 67
        {
            STL_NOTICE trap67; 
            memcpy(&trap67, notice, sizeof(trap67)); 
            IB_LOG_INFO0("Trap 67");
        }
        break; 

    case STL_TRAP_LINK_INTEGRITY:       // trap 129
    case STL_TRAP_BUFFER_OVERRUN:       // trap 130
    case STL_TRAP_FLOW_WATCHDOG:        // trap 131
        {
            STL_NOTICE trap; 
            
            memcpy(&trap, notice, sizeof(trap)); 
            IB_LOG_INFO0("Trap 129");
        }
        break; 
        
    case STL_TRAP_LINK_PORT_CHANGE_STATE: // trap 128
        {
            STL_NOTICE trap128; 
            STL_TRAP_PORT_CHANGE_STATE_DATA *trapDatap = (STL_TRAP_PORT_CHANGE_STATE_DATA *)trap128.Data; 
            
            memcpy(&trap128, notice, sizeof(trap128)); 
            (void)BSWAP_STL_TRAP_PORT_CHANGE_STATE_DATA((STL_TRAP_PORT_CHANGE_STATE_DATA *)trap128.Data); 
            
            current->trapType = UNSOLICITED_TOPO_CHANGE; 
            current->lidAddr = trapDatap->Lid; 
            current->portNum = 0; 
            ++state->found; 
            state->toBeProcessed = 1; 
            state->trap128Received = 1; 
            
            cs_log(_FE_TRACE(fe_config.debug), "fe_if3_get_traps", 
                   "Trap 128 (topology change) from lid 0x%X about lid 0x%X", trap128.IssuerLID, trapDatap->Lid);
        }
        break; 
        
    case STL_TRAP_BAD_M_KEY:                // trap 256
        {
            Trap256_t trap; 
            memcpy(&trap, notice, sizeof(trap)); 
            IB_LOG_INFO0("Trap 256");
        }
        break; 
        
    case STL_TRAP_BAD_P_KEY:                // trap 257
        {
            Trap257_t trap; 
            memcpy(&trap, notice, sizeof(trap)); 
            IB_LOG_INFO0("Trap 257");
        }
        break;
    }
}

uint32_t
fe_if3_get_traps(FE_Trap_t* trapinfo,uint32_t* found)
{
  uint32_t rc,index = 0;
  Mai_t mai;
  Mai_t *maip = &mai;
  uint32_t count = 0;
  IBhandle_t handles[2];
  STL_NOTICE notice;
  FE_Trap_t *temp,*top;
  FE_Trap_t current={0};
  Lid_t tempLid;
  uint64_t  timeout=50000;  // 50 millisecs
  FE_Trap_Processing_State_t state;
  SA_MAD_HDR *sa_hdr;

  IB_ENTER(__func__,0,0,0,0);

  if (!trapinfo) {
      rc = FE_FATAL;
      IB_LOG_ERRORRC("trap info is NULL:", rc);
      return rc;
  }

  trapinfo->next = NULL;
  temp = trapinfo;
  top = trapinfo;
  *found = 0;
  memset(&state, 0, sizeof(state));

  if (fdsa == INVALID_HANDLE) {
      rc = FE_NO_RETRIEVE;
      IB_LOG_INFINI_INFO0("no handle to SA");
      IB_EXIT(__func__,rc);
      return rc;
  }

  handles[count] = fdsa;

  while (1)
    {
      state.toBeProcessed = 0;
      rc = MngrWaitHandle(handles,1,timeout,&index,&mai);

      if (rc != VSTATUS_OK)
        {
          if (*found) {
              if (!state.trap128Received) {
                  /* add a subnet topology change event at end to make FV ask for changes */
                  if (temp == NULL)
                  {
                      rc = vs_pool_alloc(&fe_pool,sizeof(FE_Trap_t),(void*)&temp);
                      if (rc != VSTATUS_OK)
                      {
                          IB_LOG_ERRORRC("Cannot allocate memory rc:", rc);
                          rc = FE_NO_RETRIEVE;
                          IB_EXIT(__func__,rc);
                          return rc;
                      }
                      temp->next = NULL;
                      top->next = temp;
                      top = temp;
                  }
                  temp->trapType = UNSOLICITED_TOPO_CHANGE;
                  temp->lidAddr = state.smlid;
                  temp->portNum = 0;
                  // this is a trap initiated by the FE, so just set the trap number
                  // field of the raw notice data to assist the FEC 
                  memset(&temp->notice, 0, sizeof(temp->notice));
                  temp->notice.Attributes.Generic.TrapNumber = STL_TRAP_LINK_PORT_CHANGE_STATE;
                  temp = temp->next;
                  *found = *found+1;
              }
              rc = FE_SUCCESS;
          } else {
              rc = FE_NO_RETRIEVE;
          }

          IB_EXIT(__func__,rc);
          return rc;
        }


      if (index > count)
        {
          //IB_LOG_INFINI_INFO0("NO notices found");
          rc = FE_NO_RETRIEVE;
          IB_EXIT(__func__,rc);
          return rc;
        }

      switch (mai.base.method)
        {
      case MAD_CM_REPORT:
          sa_hdr = (SA_MAD_HDR*)maip->data;
          sa_hdr = sa_hdr+1;
          (void)BSWAPCOPY_STL_NOTICE((STL_NOTICE *)sa_hdr, &notice);

          /* Prepare to send reply */
          /* Swap QPs, except we always send from QP1 */
          mai.addrInfo.destqp = mai.addrInfo.srcqp;
          mai.addrInfo.srcqp = 1;

          /* Swap LIDs */
          tempLid = mai.addrInfo.slid;
          mai.addrInfo.slid = mai.addrInfo.dlid;
          mai.addrInfo.dlid = tempLid;

          /* Set Reply bit */
          mai.base.method = MAD_CM_REPORT_RESP;
          mai.active |= MAI_ACT_DATA;
          /* no need to ack on HFM - stack does it for us */
          #ifdef __VXWORKS__
          rc = mai_send(fdsa, &mai);
          if (rc != VSTATUS_OK) {
              IB_LOG_ERRORX("Failed to send MAD_CM_REPORT_RESP to lid:", mai.addrInfo.dlid);
              IB_EXIT(__func__,rc);
              return rc;
          }
          #endif
          /*
           * look for duplicate TID and multiple trap 64/65 on same GID
           */
          if (MAI_MASK_TID(prevTid) == MAI_MASK_TID(mai.base.tid)) {
              //IB_LOG_INFINI_INFOLX("CM Report duplicate TID received ", prevTid);
              break;
          } else 
              prevTid = mai.base.tid;
          //*found = *found +1;
		  fe_if3_ib_notice_to_fe_trap(&notice, &current, &state);
		  *found = state.found;
          /* save previous trap number value */
          prevTrapNum = notice.Attributes.Generic.TrapNumber;
          break;

        default:
          IB_LOG_ERROR("Unexpected method:",mai.base.method);
          rc = FE_NO_RETRIEVE;
          IB_EXIT(__func__,rc);
          return rc;
          break;
        }
      /*
       * we will only process traps 64, 65, and 128 for now
       */
      if (state.toBeProcessed) {
          if (temp == NULL)
          {
              rc = vs_pool_alloc(&fe_pool,sizeof(FE_Trap_t),(void*)&temp);
              if (rc != VSTATUS_OK)
              {
                  IB_LOG_ERRORRC("Cannot allocate memory rc:", rc);
                  rc = FE_NO_RETRIEVE;
                  IB_EXIT(__func__,rc);
                  return rc;
              }
              temp->next = NULL;
              top->next = temp;
              top = temp;
          }
          temp->trapType = current.trapType;
          temp->lidAddr = current.lidAddr;
          temp->portNum = current.portNum;
          temp = temp->next;
      }
    }

  rc = FE_SUCCESS;

  IB_EXIT(__func__, rc);
        return rc;
}
