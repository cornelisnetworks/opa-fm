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

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <infiniband/umad_types.h>
#include <infiniband/umad_str.h>
#include <stl_sm_priv.h>
#include <stl_sa_priv.h>
#include <stl_pm.h>
#include <stl_pa_priv.h>


#include <opamgt_priv.h>
#include <syslog.h>

#define OMGT_STACK_BUF_SIZE 128

/*
 * Extends umad_class_str().
 */
const char *stl_class_str(uint8_t BaseVersion, uint8_t class)
{
    if (BaseVersion != STL_BASE_VERSION) {
        return umad_class_str(class);
    }
    switch (class) {
    case MCLASS_VFI_PM:	        	return("PerfAdm");
    }
    return umad_class_str(class);
}
/*
 * Extends umad_method_str(). 
 *  
 * Added All Methods defined in IB spec which STL spec links to.
 * Numbers were used because There is no central definition for all methods used in STL or IB code that makes sense.
 */
const char *stl_method_str(uint8_t BaseVersion, uint8_t class, uint8_t method)
{
    if (BaseVersion != STL_BASE_VERSION) {
        return umad_method_str(class, method);
    }
    switch (method) {
    case 0x01:              return("Get");
    case 0x02:              return("Set");
    case 0x03:              return("Send");
    case 0x05:              return("Trap");
    case 0x06:              return("Report");
    case 0x07:              return("TrapRepress");
    case 0x12:              return("GetTable");
    case 0x13:              return("GetTraceTable");
    case 0x14:              return("GetMulti");
    case 0x15:              return("Delete");
    case 0x81:              return("GetResp");
    case 0x86:              return("ReportResp");
    case 0x92:              return("GetTableResp");
    case 0x94:              return("GetMultiResp");
    case 0x95:              return("DeleteResp");
    }
    return umad_method_str(class, method);
}
/*
 * Extends umad_common_mad_status_str(). 
 *  
 * Added all common status options because they have higher priority over class-specific statuses. 
 * Could not include definitions from Esm/ib/include/ib_mad.h
 */
const char *stl_mad_status_str(uint8_t BaseVersion, uint8_t class, uint16_t Status)
{
    if  (BaseVersion != STL_BASE_VERSION) 
		return umad_common_mad_status_str(Status);

    uint16 status = ntohs(Status);
    if (!(status & 0x7FFF)) return("Success");
	if (status & 0x0001)    return("Busy");                                         // MAD_STATUS_BUSY
	if (status & 0x0002)    return("Redirection required");	                        // MAD_STATUS_REDIRECT
	if (status & 0x001C) {                                                          //7 = 0x001C = 0000 0000 0001 1100 = MAD_STATUS_BAD_FIELD
        switch (status) {
        case 0x0004:        return("Bad Class and/or Base Version");                //1 = 0x0004 = 0000 0000 0000 0100 = MAD_STATUS_BAD_CLASS
        case 0x0008:        return("Method not supported");                         //2 = 0x0008 = 0000 0000 0000 1000 = MAD_STATUS_BAD_METHOD
        case 0x000C:        return("Method/Attribute combination not supported");   //3 = 0x000C = 0000 0000 0000 1100 = MAD_STATUS_BAD_ATTR
        case 0x0010:                                                                //4 = 0x0010 = 0000 0000 0001 0000 = MAD_STATUS_RSVD1
        case 0x0014:                                                                //5 = 0x0014 = 0000 0000 0001 0100 = MAD_STATUS_RSVD2
        case 0x0018:                                                                //6 = 0x0018 = 0000 0000 0001 1000 = MAD_STATUS_RSVD3
        default:            return("Invalid Attribute/Modifier");                   
        }
    }
    // Check Class Specific Status Bits 8-14 = 0x7F00 
    if (status & 0x7F00) {
        switch (class) {
    	case MCLASS_SM_LID_ROUTED:     //SM
        case MCLASS_SM_DIRECTED_ROUTE:
            break;
        case MCLASS_SUBN_ADM:                                       return umad_sa_mad_status_str(Status);
		case MCLASS_PERF:              //PM
			switch (status) {
			case STL_PM_STATUS_REQUEST_TOO_LARGE:           return("Request too large");
			case STL_PM_STATUS_NUMBLOCKS_INCONSISTENT:      return("Request too large");
			case STL_PM_STATUS_OPERATION_FAILED:            return("Request too large");
			}
            break;
		case MCLASS_VFI_PM:            //PA
			switch (status) {
			case STL_MAD_STATUS_STL_PA_UNAVAILABLE:         return("Engine unavailable");
			case STL_MAD_STATUS_STL_PA_NO_GROUP:            return("No such group");
			case STL_MAD_STATUS_STL_PA_NO_PORT:             return("Port not found");
			case STL_MAD_STATUS_STL_PA_NO_VF:               return("VF not found");
			case STL_MAD_STATUS_STL_PA_INVALID_PARAMETER:   return("Invalid parameter");
			case STL_MAD_STATUS_STL_PA_NO_IMAGE:            return("Image not found");
			case STL_MAD_STATUS_STL_PA_NO_DATA:             return("No Counter Data");
			case STL_MAD_STATUS_STL_PA_BAD_DATA:            return("Bad Counter Data");
			default:                                        return umad_sa_mad_status_str(Status);
			}
            break;
        }
    }
	return umad_common_mad_status_str(Status);
}
/*
 * Extends umad_attribute_str(). 
 *  
 * Added All AttrIDs because they were redefined with STL_... 
 */
const char *stl_attribute_str(uint8_t BaseVersion, uint8_t class, uint16_t attr)
{
	if  (BaseVersion != STL_BASE_VERSION) 
		return umad_attribute_str(class, attr);

	switch (class) {
    case MCLASS_SM_LID_ROUTED: //SM //0x01
    case MCLASS_SM_DIRECTED_ROUTE:  //0x81
        switch (ntoh16(attr)) {
        case STL_MCLASS_ATTRIB_ID_NODE_DESCRIPTION:                 return("NodeDesc");
		case STL_MCLASS_ATTRIB_ID_NODE_INFO:                        return("NodeInfo");
		case STL_MCLASS_ATTRIB_ID_SWITCH_INFO:                      return("SwitchInfo");
		case STL_MCLASS_ATTRIB_ID_PORT_INFO:                        return("PortInfo");
		case STL_MCLASS_ATTRIB_ID_PART_TABLE:                       return("PKey");
		case STL_MCLASS_ATTRIB_ID_SL_SC_MAPPING_TABLE:              return("SLtoSC");
		case STL_MCLASS_ATTRIB_ID_SC_SC_MULTI_SET:                  return("SLtoSCMulti");
		case STL_MCLASS_ATTRIB_ID_VL_ARBITRATION:                   return("VLArb");
		case STL_MCLASS_ATTRIB_ID_LINEAR_FWD_TABLE:                 return("LinerFwdTable");
		case STL_MCLASS_ATTRIB_ID_MCAST_FWD_TABLE:                  return("MulticastFwdTable");
		case STL_MCLASS_ATTRIB_ID_SM_INFO:                          return("SMInfo");
		case STL_MCLASS_ATTRIB_ID_LED_INFO:                         return("LEDInfo");
		case STL_MCLASS_ATTRIB_ID_CABLE_INFO:                       return("CableInfo");
		case STL_MCLASS_ATTRIB_ID_AGGREGATE:                        return("Aggregate");
		case STL_MCLASS_ATTRIB_ID_SC_SC_MAPPING_TABLE:              return("SCtoSC");
		case STL_MCLASS_ATTRIB_ID_SC_SL_MAPPING_TABLE:              return("SCtoSL");
		case STL_MCLASS_ATTRIB_ID_SC_VLR_MAPPING_TABLE:             return("SCtoVLr");
		case STL_MCLASS_ATTRIB_ID_SC_VLT_MAPPING_TABLE:             return("SCtoVLt");
		case STL_MCLASS_ATTRIB_ID_SC_VLNT_MAPPING_TABLE:            return("SCtoVLnt");
		case STL_MCLASS_ATTRIB_ID_PORT_STATE_INFO:                  return("PortStateInfo");
		case STL_MCLASS_ATTRIB_ID_PORT_GROUP_FWD_TABLE:             return("PortGroupFwdTable");
		case STL_MCLASS_ATTRIB_ID_PORT_GROUP_TABLE:                 return("PortGroupTable");
		case STL_MCLASS_ATTRIB_ID_BUFFER_CONTROL_TABLE:             return("BufferCtrlTable");
		case STL_MCLASS_ATTRIB_ID_CONGESTION_INFO:                  return("CongestionInfo");
		case STL_MCLASS_ATTRIB_ID_SWITCH_CONGESTION_LOG:            return("SwitchCongLog");
		case STL_MCLASS_ATTRIB_ID_SWITCH_CONGESTION_SETTING:        return("SwitchCongSetting");
		case STL_MCLASS_ATTRIB_ID_SWITCH_PORT_CONGESTION_SETTING:   return("SwitchPortCongSetting");
		case STL_MCLASS_ATTRIB_ID_HFI_CONGESTION_LOG:               return("HFICongLog");
		case STL_MCLASS_ATTRIB_ID_HFI_CONGESTION_SETTING:           return("HFICongSetting");
		case STL_MCLASS_ATTRIB_ID_HFI_CONGESTION_CONTROL_TABLE:     return("HFICongCtrlTable");
        }
        break;
    case MCLASS_SUBN_ADM: //SA  //0x03
        switch (ntoh16(attr)) {
		case STL_SA_ATTR_CLASS_PORT_INFO:				return("ClassPortInfo");
		case STL_SA_ATTR_NOTICE:						return("Notice");
		case STL_SA_ATTR_INFORM_INFO:					return("InformInfo");
		case STL_SA_ATTR_NODE_RECORD:					return("NodeRecord");
		case STL_SA_ATTR_PORTINFO_RECORD:				return("PortInfoRecord");
		case STL_SA_ATTR_SC_MAPTBL_RECORD:				return("SCMappingTableRecord");
		case STL_SA_ATTR_SWITCHINFO_RECORD:				return("SwitchInfoRecord");
		case STL_SA_ATTR_LINEAR_FWDTBL_RECORD:			return("LinerFwdTableRecord");
		case STL_SA_ATTR_MCAST_FWDTBL_RECORD:			return("MulticastFwdTableRecord");
		case STL_SA_ATTR_SMINFO_RECORD:					return("SMInfoRecord");
		case STL_SA_ATTR_LINK_SPD_WDTH_PAIRS_RECORD:    return("LinkSpeedWithPairsRecord");
		case STL_SA_ATTR_LINK_RECORD:					return("LinkRecord");
		case STL_SA_ATTR_SERVICE_RECORD:				return("ServiceRecord");
		case STL_SA_ATTR_P_KEY_TABLE_RECORD:			return("PKeyTableRecord");
		case STL_SA_ATTR_PATH_RECORD:					return("PathRecord");
		case STL_SA_ATTR_VLARBTABLE_RECORD:				return("VLArbitrationRecord");
		case STL_SA_ATTR_MCMEMBER_RECORD:				return("MulticastMemberRecord");
		case STL_SA_ATTR_TRACE_RECORD:					return("TraceRecord");
		case STL_SA_ATTR_MULTIPATH_GID_RECORD:			return("MultipathGIDRecord");
		case STL_SA_ATTR_SERVICEASSOCIATION_RECORD:		return("ServiceAssociationRecord");
		case STL_SA_ATTR_INFORM_INFO_RECORD:			return("InformInfoRecord");
		case STL_SA_ATTR_SC2SL_MAPTBL_RECORD:			return("SCtoSLMappingTableRecord");
		case STL_SA_ATTR_SC2VL_NT_MAPTBL_RECORD:		return("SCtoVLntMappingTableRecord");
		case STL_SA_ATTR_SC2VL_T_MAPTBL_RECORD:			return("SCtoVLtMappingTableRecord");
		case STL_SA_ATTR_SC2VL_R_MAPTBL_RECORD:			return("SCtoVLrMappingTableRecord");
		case STL_SA_ATTR_PGROUP_FWDTBL_RECORD:			return("PortGroupFwdTableRecord");
		case STL_SA_ATTR_MULTIPATH_GUID_RECORD:			return("MultipathGUIDRecord");
		case STL_SA_ATTR_MULTIPATH_LID_RECORD:			return("MultipathLIDRecord");
		case STL_SA_ATTR_CABLE_INFO_RECORD:				return("CableInfoRecord");
		case STL_SA_ATTR_VF_INFO_RECORD:				return("VFInfoRecord");
		case STL_SA_ATTR_PORT_STATE_INFO_RECORD:		return("PortStateInfoRecord");
		case STL_SA_ATTR_PORTGROUP_TABLE_RECORD:		return("PortGroupTableRecord");
		case STL_SA_ATTR_BUFF_CTRL_TAB_RECORD:			return("BufferCtrlTableRecord");
		case STL_SA_ATTR_FABRICINFO_RECORD:				return("FabricInfoRecord");
		case STL_SA_ATTR_QUARANTINED_NODE_RECORD:		return("QuarantinedNodeRecord");
		case STL_SA_ATTR_CONGESTION_INFO_RECORD:		return("CongestionInfoRecord");
		case STL_SA_ATTR_SWITCH_CONG_RECORD:			return("SwitchCongestionRecord");
		case STL_SA_ATTR_SWITCH_PORT_CONG_RECORD:		return("SwitchPortCongestionRecord");
		case STL_SA_ATTR_HFI_CONG_RECORD:				return("HFICongestionRecord");
		case STL_SA_ATTR_HFI_CONG_CTRL_RECORD:			return("HFICongestionCtrlRecord");
		case STL_SA_ATTR_SWITCH_COST_RECORD:			return("SwitchCostRecord");
        }
        break;
    case MCLASS_PERF: //PM      //0x04
        switch (ntoh16(attr)) {
		case STL_PM_ATTRIB_ID_CLASS_PORTINFO:		return("ClassPortInfo");
		case STL_PM_ATTRIB_ID_PORT_STATUS:          return("PortStatus");
		case STL_PM_ATTRIB_ID_CLEAR_PORT_STATUS:    return("ClearPortStatus");
		case STL_PM_ATTRIB_ID_DATA_PORT_COUNTERS:   return("DataPortCounters");
		case STL_PM_ATTRIB_ID_ERROR_PORT_COUNTERS:  return("ErrorPortCounters");
		case STL_PM_ATTRIB_ID_ERROR_INFO:           return("ErrorInfo");
        }
        break;
    case MCLASS_VFI_PM: //PA    //0x32
        switch (ntoh16(attr)) {
        case STL_PA_ATTRID_GET_CLASSPORTINFO:       return("ClassPortInfo");
		case STL_PA_ATTRID_GET_GRP_LIST:            return("GroupList");
		case STL_PA_ATTRID_GET_GRP_INFO:            return("GroupInfo");
		case STL_PA_ATTRID_GET_GRP_CFG:             return("GroupConfig");
		case STL_PA_ATTRID_GET_PORT_CTRS:           return("PortCounters");
		case STL_PA_ATTRID_CLR_PORT_CTRS:           return("ClearPortCounters");
		case STL_PA_ATTRID_CLR_ALL_PORT_CTRS:       return("ClearAllPortCounters");
		case STL_PA_ATTRID_GET_PM_CONFIG:           return("PmConfig");
        case STL_PA_ATTRID_FREEZE_IMAGE:            return("FreezeImage");
		case STL_PA_ATTRID_RELEASE_IMAGE:           return("ReleaseImage");
		case STL_PA_ATTRID_RENEW_IMAGE:             return("RenewImage");
		case STL_PA_ATTRID_GET_FOCUS_PORTS:         return("FocusPorts");
        case STL_PA_ATTRID_GET_IMAGE_INFO:          return("ImageInfo");
		case STL_PA_ATTRID_MOVE_FREEZE_FRAME:       return("MoveFreezeFrame");
		case STL_PA_ATTRID_GET_VF_LIST:             return("VFList");
		case STL_PA_ATTRID_GET_VF_INFO:             return("VFInfo");
		case STL_PA_ATTRID_GET_VF_CONFIG:           return("VFConfig");
		case STL_PA_ATTRID_GET_VF_PORT_CTRS:        return("VFPortCounters");
		case STL_PA_ATTRID_CLR_VF_PORT_CTRS:        return("ClearVFPortCounters");
		case STL_PA_ATTRID_GET_VF_FOCUS_PORTS:      return("VFFocusPorts");
		case STL_PA_ATTRID_GET_FOCUS_PORTS_MULTISELECT: return("MultiSelectFocusPorts");
		case STL_PA_ATTRID_GET_GRP_NODE_INFO:       return("GroupNodeInfo");
		case STL_PA_ATTRID_GET_GRP_LINK_INFO:       return("GroupLinkInfo");
        }
        break;

	}

	return umad_attribute_str(class, attr);
}

static void print_mad_hdr_syslog(uint8_t *buf, size_t size)
{
	struct umad_hdr * umad_hdr = (struct umad_hdr *)buf;
	int i;

	int n = 0;
	char tmp[OMGT_STACK_BUF_SIZE];

	n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "%06d: ", 0);
	for (i = 0; i < size;) {
		n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "%02x", buf[0]);
		if (++i >= size)
			break;
		n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "%02x", buf[1]);
		if ((++i) % 4)
			n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, " ");
		else {
			switch(i) {
				case 4:
					n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "  %02d | %s | %02d | %s",
						umad_hdr->base_version,
						stl_class_str(umad_hdr->base_version, umad_hdr->mgmt_class),
						umad_hdr->class_version,
                        stl_method_str(umad_hdr->base_version, umad_hdr->mgmt_class, umad_hdr->method));
					break;
				case 8:
					n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "  %s",
                        stl_mad_status_str(umad_hdr->base_version, umad_hdr->mgmt_class, umad_hdr->status));
					break;
				case 20:
					n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "  %s | ",
                        stl_attribute_str(umad_hdr->base_version, umad_hdr->mgmt_class, umad_hdr->attr_id));
					break;
				case 24:
					n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "  (AttributeModifier)");
					break;
				default:
					break;
			}
			syslog(LOG_DEBUG, "%s", tmp);
			n = 0;
			n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "%06d: ", i);
		}
		buf += 2;
	}
	syslog(LOG_DEBUG, "%s", tmp);
}

static void print_mad_hdr(FILE *file, uint8_t *buf, size_t size)
{
	struct umad_hdr * umad_hdr = (struct umad_hdr *)buf;
	int i;

	if (file == OMGT_DBG_FILE_SYSLOG) {
		print_mad_hdr_syslog(buf, size);
		return;
	}

	fprintf(file, "%06d: ", 0);
	for (i = 0; i < size;) {
		fprintf(file, "%02x", buf[0]);
		if (++i >= size)
			break;
		fprintf(file, "%02x", buf[1]);
		if ((++i) % 4)
			fprintf(file, " ");
		else {
			switch(i) {
				case 4:
					fprintf(file, "  %02d | %s | %02d | %s",
						umad_hdr->base_version,
						stl_class_str(umad_hdr->base_version, umad_hdr->mgmt_class),
						umad_hdr->class_version,
                        stl_method_str(umad_hdr->base_version, umad_hdr->mgmt_class, umad_hdr->method));
					break;
				case 8:
					fprintf(file, "  %s",
                        stl_mad_status_str(umad_hdr->base_version, umad_hdr->mgmt_class, umad_hdr->status));
					break;
				case 20:
					fprintf(file, "  %s | ",
                        stl_attribute_str(umad_hdr->base_version, umad_hdr->mgmt_class, umad_hdr->attr_id));
					break;
				case 24:
					fprintf(file, "  (AttributeModifier)");
					break;
				default:
					break;
			}
			fprintf(file, "\n%06d: ", i);
		}
		buf += 2;
	}
	fprintf(file, "\n");
}

static void omgt_xdump_syslog(uint8_t *buf, size_t size, int width)
{
	int i;
	int n = 0;
	char tmp[OMGT_STACK_BUF_SIZE];

	n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "%06d: ", 0);
	for (i = 0; i < size;) {
		n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "%02x", buf[0]);
		if (++i >= size)
			break;
		n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "%02x", buf[1]);
		if ((++i) % width)
			n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, " ");
		else {
			syslog(LOG_DEBUG, "%s", tmp);
			n = 0;
			n += snprintf(tmp+n, OMGT_STACK_BUF_SIZE - n, "%06d: ", i);
		}
		buf += 2;
	}
	syslog(LOG_DEBUG, "%s", tmp);
}

void omgt_xdump(FILE *file, uint8_t *buf, size_t size, int width)
{
	int i;

	if (file == OMGT_DBG_FILE_SYSLOG) {
		omgt_xdump_syslog(buf, size, width);
		return;
	}

	fprintf(file, "%06d: ", 0);
	for (i = 0; i < size;) {
		fprintf(file, "%02x", buf[0]);
		if (++i >= size)
			break;
		fprintf(file, "%02x", buf[1]);
		if ((++i) % width)
			fprintf(file, " ");
		else {
			fprintf(file, "\n");
			fprintf(file, "%06d: ", i);
		}
		buf += 2;
	}
	fprintf(file, "\n");
}

static size_t get_data_offset(uint8_t mgmt_class)
{
	switch (mgmt_class) {
    case MCLASS_SM_LID_ROUTED:		return(24+8);               //MAD+MKey
	case MCLASS_SM_DIRECTED_ROUTE:	return(24+8+4+4+64+64+8);   //MAD+Mkey+SLID+DLID+InitPath+RetPath+Rsvd2
	case MCLASS_SUBN_ADM:		    return(24+12+8+2+2+8);      //MAD+RMPP+SMKey+AttrOff+Rsvd+CompMask
	case MCLASS_PERF:		        return(24);                 //MAD
	case MCLASS_VFI_PM:		        return(24+12+8+2+2+8);      //MAD+RMPP+SMKey+AttrOff+Rsvd+CompMask
	case MCLASS_BM:
	case MCLASS_DEV_MGT:
	case MCLASS_COMM_MGT:
	case MCLASS_SNMP:
	case MCLASS_DEV_CONF_MGT:
	case MCLASS_DTA:
	case MCLASS_CC:
	default:                        return(24);
	}
}

void omgt_dump_mad(FILE * file, uint8_t *buf, size_t size, const char *msg, ...)
{
	char b[512];
	va_list va;
	size_t hdr_size;
	uint8_t class = buf[1];
	size_t data_off = 24;

	if (msg) {
		va_start(va, msg);
		vsnprintf(b, 511, msg, va);
		va_end(va);
		b[511] = '\0';
		if (file == OMGT_DBG_FILE_SYSLOG)
			syslog(LOG_DEBUG, "%s", b);
		else
			fputs(b, file);
	}

	data_off = get_data_offset(class);
	hdr_size = (size < data_off) ? size : data_off;

	print_mad_hdr(file, buf, hdr_size);

	if (file == OMGT_DBG_FILE_SYSLOG)
		syslog(LOG_DEBUG, "Data:\n");
	else
		fprintf(file, "Data:\n");

	if ((size - hdr_size) > 0) {
		omgt_xdump(file, buf+hdr_size, size - hdr_size, 8);
	}
}
