/* A query application for OPA SA records similar to opasaquery
 * bundled in IFS distribution
 *
 */

// core API
#include <opamgt/opamgt.h>
// extensions for SA queries
#include <opamgt/opamgt_sa.h>

#include <getopt.h>

void Usage()
{
	fprintf(stdout, "Usage: saquery [-v] [-h hfi] [-p port] [-t ms] [-b oob_host] [-l lid] -o type \n");
	fprintf(stdout, "              or\n");
	fprintf(stdout, "       saquery --help\n");
	fprintf(stdout, "    --help - produce full help text\n");
	fprintf(stdout, "\n");
	fprintf(stderr, "    -v/--verbose       - verbose output\n");
	fprintf(stderr, "    -h/--hfi hfi       - hfi, numbered 1..n, 0= -p port will be a system wide\n");
	fprintf(stderr, "                         port num (default is 0)\n");
	fprintf(stderr, "    -p/--port port     - port, numbered 1..n, 0=1st active (default is 1st\n");
	fprintf(stderr, "                         active)\n");
	fprintf(stderr, "    -t/--timeout       - timeout in ms\n");
	fprintf(stderr, "    -l/--lid           - filter query results for specific lid\n");
	fprintf(stderr, "    -g/--guid          - filter query results for specific guid\n");
	fprintf(stderr, "    -b  oob_host       - perform out of band query. For this example, oob_host\n");
	fprintf(stderr, "                         should have format hostname[:port] or a.b.c.d[:port]\n");
	fprintf(stderr, "    -o/--output type   - output type\n");
	fprintf(stderr, "    -x local_gid       - local gid for out-of-band queryies that need a local gid\n");
	fprintf(stdout, "type: (default is node)\n");
	fprintf(stdout, "    classportinfo - classportinfo of the SA\n");
	fprintf(stdout, "    systemguid    - list of system image guids\n");
	fprintf(stdout, "    nodeguid      - list of node guids\n");
	fprintf(stdout, "    portguid      - list of port guids\n");
	fprintf(stdout, "    lid           - list of lids\n");
	fprintf(stdout, "    desc          - list of node descriptions/names\n");
	fprintf(stdout, "    path          - list of path records\n");
	fprintf(stdout, "    node          - list of node records\n");
	fprintf(stdout, "    portinfo      - list of port info records\n");
	fprintf(stdout, "    sminfo        - list of SM info records\n");
	fprintf(stdout, "    swinfo        - list of switch info records\n");
	fprintf(stdout, "    link          - list of link records\n");
	fprintf(stdout, "    scsc          - list of SC to SC mapping table records\n");
	fprintf(stdout, "    slsc          - list of SL to SC mapping table records\n");
	fprintf(stdout, "    scsl          - list of SC to SL mapping table records\n");
	fprintf(stdout, "    scvlt         - list of SC to VLt table records\n");
	fprintf(stdout, "    scvlnt        - list of SC to VLnt table records\n");
	fprintf(stdout, "    vlarb         - list of VL arbitration table records\n");
	fprintf(stdout, "    pkey          - list of P-Key table records\n");
	fprintf(stdout, "    service       - list of service records\n");
	fprintf(stdout, "    mcmember      - list of multicast member records\n");
	fprintf(stdout, "    inform        - list of inform info records\n");
	fprintf(stdout, "    linfdb        - list of switch linear FDB records\n");
	fprintf(stdout, "    mcfdb         - list of switch multicast FDB records\n");
	fprintf(stdout, "    trace         - list of trace records\n");
	fprintf(stdout, "    vfinfo        - list of vFabrics\n");
	fprintf(stdout, "    fabricinfo    - summary of fabric devices\n");
	fprintf(stdout, "    quarantine    - list of quarantined nodes\n");
	fprintf(stdout, "    conginfo      - list of Congestion Info Records\n");
	fprintf(stdout, "    swcongset     - list of Switch Congestion Settings\n");
	fprintf(stdout, "    swportcong    - list of Switch Port Congestion Settings\n");
	fprintf(stdout, "    hficongset    - list of HFI Congestion Settings\n");
	fprintf(stdout, "    hficongcon    - list of HFI Congesting Control Settings\n");
	fprintf(stdout, "    bfrctrl       - list of buffer control tables\n");
	fprintf(stdout ,"    cableinfo     - list of Cable Info records\n");
	fprintf(stdout ,"    portgroup     - list of AR Port Group records\n");
	fprintf(stdout ,"    portgroupfdb  - list of AR Port Group FWD records\n");
	fprintf(stdout, "\n");
	fprintf(stdout, "Usage examples:\n");
	fprintf(stdout, "    saquery -o desc\n");
	fprintf(stdout, "    saquery -o portinfo\n");
	fprintf(stdout, "    saquery -o sminfo\n");
	fprintf(stdout, "    saquery -o pkey\n");

	exit(0);
}

// command line options
struct option options[] = {
	{ "verbose", required_argument, NULL, 'v' },
	{ "hfi", required_argument, NULL, 'h' },
	{ "port", required_argument, NULL, 'p' },
	{ "timeout", required_argument, NULL, 't' },

	// input types (small subset)
	{ "lid", required_argument, NULL, 'l' },
	{ "guid", required_argument, NULL, 'g' },

	// output type
	{ "output", required_argument, NULL, 'o' },
	{ "help", no_argument, NULL, '$' },	// use an invalid option character
	{ 0 }
};

int main(int argc, char ** argv)
{

	OMGT_STATUS_T status = OMGT_STATUS_SUCCESS;
	int exitcode = 0;
	struct omgt_port * port = NULL;
	int num_records = 0;
	int c, index, debug = 0;
	int sa_service_state = OMGT_SERVICE_STATE_UNKNOWN;
	void *sa_records = NULL;
	omgt_sa_selector_t selector;

	char * type = "node";
	int hfi_num = 1;
	int port_num = 1;
	int ms_timeout = OMGT_DEF_TIMEOUT_MS;
	uint32_t lid = 0;
	uint64_t guid = 0;
	uint64_t local_prefix = 0;
	char *tmp = NULL, *gid = NULL;
	uint64_t local_guid = 0;

	char * oob_addr = NULL;

	//default to requesting all records
	selector.InputType = InputTypeNoInput;

	while (-1 != (c = getopt_long(argc, argv, "h:p:t:o:l:b:g:x:v$", options, &index))){
		switch (c)
		{
			case '$':
				Usage();
				break;
			case 'h':	// hfi to issue query from
				hfi_num = strtol(optarg, NULL, 0);
				break;
			case 'p':	// port to issue query from
				port_num = strtol(optarg, NULL, 0);
				break;
			case 't':
				ms_timeout = strtol(optarg, NULL, 0);
				break;
			case 'o':	// select output record desired
				type = optarg;
				break;
			case 'l':
				selector.InputType = InputTypeLid;
				lid = strtoul(optarg, NULL, 0);
				break;
			case 'g':
				selector.InputType = InputTypePortGuid;
				guid = strtoul(optarg, NULL, 0);
				break;
			case 'x':
				gid = optarg;
				if (gid[0] == '0' && gid[1] == 'x') gid += 2;
				local_prefix = strtoul(gid, &tmp, 16);
				local_guid = strtoul(++tmp, NULL, 16);
				break;
			case 'b':
				oob_addr = optarg;
				break;
			case 'v':
				debug = 1;
				break;
			default:
				fprintf(stderr, "opasaquery: Invalid option -%c\n", c);
				Usage();
				break;
		}
	}

	// create a session
	struct omgt_params session_params = {
		.debug_file = debug ? stderr : NULL,
		.error_file = stderr
	};
	if (oob_addr){
		tmp = strchr(oob_addr, ':');
		if (tmp) *tmp = 0;
		struct omgt_oob_input oob_input = {
			/* host can be specified by name or as either IPv4 of IPv6
			 * but this example only considers IPv4 for simplicity*/
			.host = oob_addr,
			.port = (tmp ? strtol(++tmp, NULL, 0) : 3245)
		};

		status = omgt_oob_connect(&port, &oob_input, &session_params);
	} else {
		status = omgt_open_port_by_num(&port, hfi_num, port_num, &session_params);
		(void)omgt_port_get_port_guid(port, &local_guid);
		(void)omgt_port_get_port_prefix(port, &local_prefix);

		/* (Optional) Check if the SA Service is Operational
		 * All SA queries will do an initial sa service state check if not already
		 * operational, so this is more of a verbose sanity check.
		 */
		if (status == OMGT_STATUS_SUCCESS) {
			status = omgt_port_get_sa_service_state(port, &sa_service_state, OMGT_REFRESH_SERVICE_ANY_STATE);
			if (status == OMGT_STATUS_SUCCESS) {
				if (sa_service_state != OMGT_SERVICE_STATE_OPERATIONAL) {
					fprintf(stderr, "failed to connect, SA service state is Not Operational: %s (%d)\n",
						omgt_service_state_totext(sa_service_state), sa_service_state);
					exitcode = 1;
					goto fail2;
				}
			} else {
				fprintf(stderr, "failed to get and refresh SA service state: %s (%u)\n",
					omgt_status_totext(status), status);
				exitcode = 1;
				goto fail2;
			}
		}
	}

	if (OMGT_STATUS_SUCCESS != status){
		fprintf(stderr, "failed to open port: %s (%u)\n",
			omgt_status_totext(status), status);
		exitcode=1;
		goto fail1;
	}

	/* Set timeout for SA operation */
	omgt_set_timeout(port, ms_timeout);

	/* Perform the requested operation.
	 * Some records are are not always supported,
	 * and in those cases an additional omgt_sa_get_classportinfo
	 * call must be performed to determine the management capabilities
	 */
	if (!strcmp(type, "node")){
		/* Valid Input Types:
		 * 	NoInput, Lid, PortGuid, NodeGuid, SystemImageGuid, NodeType, NodeDesc
		 */
		if (selector.InputType == InputTypePortGuid) selector.InputValue.NodeRecord.NodeGUID = guid;
		if (selector.InputType == InputTypeLid) selector.InputValue.NodeRecord.Lid = lid;
		status = omgt_sa_get_node_records(port, &selector, &num_records, (STL_NODE_RECORD **)&sa_records);
	} else if (!strcmp(type, "portinfo")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.PortInfoRecord.Lid = lid;
		status = omgt_sa_get_portinfo_records(port, &selector, &num_records, (STL_PORTINFO_RECORD **)&sa_records);
	} else if (!strcmp(type, "systemguid")){
		/* Valid Input Types:
		 * 	NoInput, NodeType, SystemImageGuid, NodeGuid, PortGuid, Lid, NodeDesc
		 */
		if (selector.InputType == InputTypePortGuid) selector.InputValue.NodeRecord.NodeGUID = guid;
		if (selector.InputType == InputTypeLid) selector.InputValue.NodeRecord.Lid = lid;
		status = omgt_sa_get_sysimageguid_records(port, &selector, &num_records, (uint64_t **)&sa_records);
	} else if (!strcmp(type, "nodeguid")){
		/* Valid Input Types:
		 * 	NoInput, NodeType, SystemImageGuid, NodeGuid, PortGuid, Lid, NodeDesc
		 */
		if (selector.InputType == InputTypePortGuid) selector.InputValue.NodeRecord.NodeGUID = guid;
		if (selector.InputType == InputTypeLid) selector.InputValue.NodeRecord.Lid = lid;
		status = omgt_sa_get_nodeguid_records(port, &selector, &num_records, (uint64_t **)&sa_records);
	} else if (!strcmp(type, "portguid")){
		/* Valid Input Types:
		 * 	NoInput, NodeType, SystemImageGuid, NodeGuid, PortGuid, Lid, NodeDesc
		 */
		if (selector.InputType == InputTypePortGuid) selector.InputValue.NodeRecord.NodeGUID = guid;
		if (selector.InputType == InputTypeLid) selector.InputValue.NodeRecord.Lid = lid;
		status = omgt_sa_get_portguid_records(port, &selector, &num_records,  (uint64_t **)&sa_records);
	} else if (!strcmp(type, "classportinfo")){
		/*Valid Input Types: NoInput*/
		status = omgt_sa_get_classportinfo_records(port, &selector, &num_records, (STL_CLASS_PORT_INFO **)&sa_records);
	} else if (!strcmp(type, "lid")){
		/* Valid Input Types:
		 * 	NoInput, NodeType, SystemImageGuid, NodeGuid, PortGuid, Lid, NodeDesc
		 */
		if (selector.InputType == InputTypePortGuid) selector.InputValue.NodeRecord.NodeGUID = guid;
		if (selector.InputType == InputTypeLid) selector.InputValue.NodeRecord.Lid = lid;
		status = omgt_sa_get_lid_records(port, &selector, &num_records, (uint32 **)&sa_records);
	} else if (!strcmp(type, "desc")){
		/* Valid Input Types:
		 * 	NoInput, NodeType, SystemImageGuid, NodeGuid, PortGuid, Lid, NodeDesc
		 */
		if (selector.InputType == InputTypePortGuid) selector.InputValue.NodeRecord.NodeGUID = guid;
		if (selector.InputType == InputTypeLid) selector.InputValue.NodeRecord.Lid = lid;
		status = omgt_sa_get_nodedesc_records(port, &selector, &num_records, (STL_NODE_DESCRIPTION **)&sa_records);
	} else if (!strcmp(type, "path")){
		/* Valid Input Types:
		 * 	NoInput, PortGuid, PortGid, PortGuidPair, GidPair, PathRecord,
		 * 	Lid, PKey, SL, ServiceId
		 *
		 * 	SourceGid is not treated as an InputType but is ALWAYS required when querying path/trace records
		 */
		if (selector.InputType == InputTypePortGuid) {
			selector.InputValue.IbPathRecord.PortGuid.DestPortGuid = guid;
			selector.InputValue.IbPathRecord.PortGuid.SourcePortGuid = local_guid;
			selector.InputValue.IbPathRecord.PortGuid.SharedSubnetPrefix = local_prefix;
		} else if (selector.InputType == InputTypeLid) {
			selector.InputValue.IbPathRecord.Lid.DLid = lid;
			selector.InputValue.IbPathRecord.Lid.SourceGid.Type.Global.InterfaceID = local_guid;
			selector.InputValue.IbPathRecord.Lid.SourceGid.Type.Global.SubnetPrefix = local_prefix;
		} else {
			selector.InputValue.IbPathRecord.SourceGid.Type.Global.InterfaceID = local_guid;
			selector.InputValue.IbPathRecord.SourceGid.Type.Global.SubnetPrefix = local_prefix;
		}
		status = omgt_sa_get_ib_path_records(port, &selector, &num_records, (IB_PATH_RECORD **)&sa_records);
	} else if (!strcmp(type, "sminfo")){
		/* Valid Input Types: NoInput */
		status = omgt_sa_get_sminfo_records(port, &selector, &num_records, (STL_SMINFO_RECORD **)&sa_records);
	} else if (!strcmp(type, "link")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.LinkRecord.Lid = lid;
		status = omgt_sa_get_link_records(port, &selector, &num_records, (STL_LINK_RECORD **)&sa_records);
	} else if (!strcmp(type, "service")){
		/* Valid Input Types: NoInput, PortGid */
		status = omgt_sa_get_service_records(port, &selector, &num_records, (IB_SERVICE_RECORD **)&sa_records);
	} else if (!strcmp(type, "mcmember")){
		// must check if SA supports these records
		omgt_sa_selector_t pre_select = {.InputType = InputTypeNoInput};
		status = omgt_sa_get_classportinfo_records(port, &pre_select, &num_records, (STL_CLASS_PORT_INFO **)&sa_records);

		if (OMGT_STATUS_SUCCESS != status || num_records != 1){
			fprintf(stderr, "Failed to determine SA capabilities for mcmember query\n");
			goto fail2;
		}else if (! (((STL_CLASS_PORT_INFO **)&sa_records)[0]->CapMask & STL_SA_CAPABILITY_MULTICAST_SUPPORT )){
			fprintf(stderr, "mcmember records not supported by SA\n");
			goto fail2;
		}

		if (selector.InputType == InputTypeLid) selector.InputValue.IbMcMemberRecord.Lid = lid;
		status = omgt_sa_get_ib_mcmember_records(port, &selector, &num_records, (IB_MCMEMBER_RECORD **)&sa_records);
	} else if (!strcmp(type, "inform")){
		/* Valid Input Types: NoInput, PortGid */
		if (selector.InputType == InputTypeLid) selector.InputValue.StlInformInfoRecord.SubscriberLID = lid;
		status = omgt_sa_get_informinfo_records(port, &selector, &num_records, (STL_INFORM_INFO_RECORD **)&sa_records);
	} else if (!strcmp(type, "trace")){
		/* Valid Input Types: PathRecord, PortGuid, GidPair, PortGid */
		/* 	SourceGid is not treated as an InputType but is ALWAYS required when querying path/trace records */
		if (selector.InputType == InputTypePortGuid) {
			selector.InputValue.TraceRecord.PortGuid.DestPortGuid = guid;
			selector.InputValue.TraceRecord.PortGuid.SourcePortGuid = local_guid;
			selector.InputValue.TraceRecord.PortGuid.SharedSubnetPrefix = local_prefix;
		} else if (selector.InputType == InputTypeLid) {
			selector.InputValue.TraceRecord.Lid.DLid = lid;
			selector.InputValue.TraceRecord.Lid.SourceGid.Type.Global.InterfaceID = local_guid;
			selector.InputValue.TraceRecord.Lid.SourceGid.Type.Global.SubnetPrefix = local_prefix;
		}
		status = omgt_sa_get_trace_records(port, &selector, &num_records, (STL_TRACE_RECORD **)&sa_records);
	} else if (!strcmp(type, "scsc")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.ScScTableRecord.Lid = lid;
		status = omgt_sa_get_scsc_table_records(port, &selector, &num_records, (STL_SC_MAPPING_TABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "slsc")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.SlScTableRecord.Lid = lid;
		status = omgt_sa_get_slsc_table_records(port, &selector, &num_records, (STL_SL2SC_MAPPING_TABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "scsl")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.ScSlTableRecord.Lid = lid;
		status = omgt_sa_get_scsl_table_records(port, &selector, &num_records, (STL_SC2SL_MAPPING_TABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "scvlt")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.ScVlxTableRecord.Lid = lid;
		status = omgt_sa_get_scvlt_table_records(port, &selector, &num_records, (STL_SC2PVL_T_MAPPING_TABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "scvlnt")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.ScVlxTableRecord.Lid = lid;
		status = omgt_sa_get_scvlnt_table_records(port, &selector, &num_records, (STL_SC2PVL_NT_MAPPING_TABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "swinfo")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.SwitchInfoRecord.Lid = lid;
		status = omgt_sa_get_switchinfo_records(port, &selector, &num_records, (STL_SWITCHINFO_RECORD **)&sa_records);
	} else if (!strcmp(type, "linfdb")){
		/* Valid Input Types: NoInput, Lid*/
		if (selector.InputType == InputTypeLid) selector.InputValue.LinFdbTableRecord.Lid = lid;
		status = omgt_sa_get_lfdb_records(port, &selector, &num_records, (STL_LINEAR_FORWARDING_TABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "mcfdb")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.McFdbTableRecord.Lid = lid;
		status = omgt_sa_get_mcfdb_records(port, &selector, &num_records, (STL_MULTICAST_FORWARDING_TABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "vlarb")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.VlArbTableRecord.Lid = lid;
		status = omgt_sa_get_vlarb_records(port, &selector, &num_records, (STL_VLARBTABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "pkey")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.PKeyTableRecord.Lid = lid;
		status = omgt_sa_get_pkey_table_records(port, &selector, &num_records, (STL_P_KEY_TABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "vfinfo")){
		/* Valid Input Types:
		 * 	NoInput, PKey, SL, ServiceID, McGid, Index, NodeDesc
		 */
		status = omgt_sa_get_vfinfo_records(port, &selector, &num_records, (STL_VFINFO_RECORD **)&sa_records);
	} else if (!strcmp(type, "fabricinfo")){
		/* Valid Input Types: NoInput */
		status = omgt_sa_get_fabric_info_records(port, &selector, &num_records, (STL_FABRICINFO_RECORD **)&sa_records);
	} else if (!strcmp(type, "quarantine")){
		/* Valid Input Types: NoInput */
		status = omgt_sa_get_quarantinenode_records(port, &selector, &num_records, (STL_QUARANTINED_NODE_RECORD **)&sa_records);
	} else if (!strcmp(type, "conginfo")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.CongInfoRecord.Lid = lid;
		status = omgt_sa_get_conginfo_records(port, &selector, &num_records, (STL_CONGESTION_INFO_RECORD **)&sa_records);
	} else if (!strcmp(type, "swcongset")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.SwCongRecord.Lid = lid;
		status = omgt_sa_get_swcong_records(port, &selector, &num_records, (STL_SWITCH_CONGESTION_SETTING_RECORD **)&sa_records);
	} else if (!strcmp(type, "swportcong")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.SwPortCongRecord.Lid = lid;
		status = omgt_sa_get_swportcong_records(port, &selector, &num_records, (STL_SWITCH_PORT_CONGESTION_SETTING_RECORD **)&sa_records);
	} else if (!strcmp(type, "hficongset")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.HFICongRecord.Lid = lid;
		status = omgt_sa_get_hficong_records(port, &selector, &num_records, (STL_HFI_CONGESTION_SETTING_RECORD **)&sa_records);
	} else if (!strcmp(type, "hficongcon")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.HFICongCtrlRecord.Lid = lid;
		status = omgt_sa_get_hficongctrl_records(port, &selector, &num_records, (STL_HFI_CONGESTION_CONTROL_TABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "bfrctrl")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.BufCtrlTableRecord.Lid = lid;
		status = omgt_sa_get_buffctrl_records(port, &selector, &num_records, (STL_BUFFER_CONTROL_TABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "cableinfo")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.CableInfoRecord.Lid = lid;
		status = omgt_sa_get_cableinfo_records(port, &selector, &num_records, (STL_CABLE_INFO_RECORD **)&sa_records);
	} else if (!strcmp(type, "portgroup")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.PortGroupRecord.Lid = lid;
		status = omgt_sa_get_portgroup_records(port, &selector, &num_records, (STL_PORT_GROUP_TABLE_RECORD **)&sa_records);
	} else if (!strcmp(type, "portgroupfdb")){
		/* Valid Input Types: NoInput, Lid */
		if (selector.InputType == InputTypeLid) selector.InputValue.PortGroupFwdRecord.Lid = lid;
		status = omgt_sa_get_portgroupfwd_records(port, &selector, &num_records, (STL_PORT_GROUP_FORWARDING_TABLE_RECORD **)&sa_records);
	}


	if (OMGT_STATUS_SUCCESS != status) {
		fprintf(stderr, "failed to execute query. MadStatus=0x%x\n", omgt_get_sa_mad_status(port));
		exitcode = 2;

		/* Not used in oob mode */
		if (!oob_addr) {
			/* Check if failure was related to SA service state */
			status = omgt_port_get_sa_service_state(port, &sa_service_state, OMGT_REFRESH_SERVICE_NOP);
			if (status == OMGT_STATUS_SUCCESS) {
				if (sa_service_state != OMGT_SERVICE_STATE_OPERATIONAL) {
					fprintf(stderr, "SA service state is Not Operational: %s (%d)\n",
						omgt_service_state_totext(sa_service_state), sa_service_state);
					exitcode = 1;
					goto fail2;
				}
			} else {
				fprintf(stderr, "failed to get SA service state: %s (%u)\n",
					omgt_status_totext(status), status);
				exitcode = 1;
				goto fail2;
			}
		}
	} else {
		printf("Number of records returned: %d\n", num_records);
	}

fail2:
	// free our result buffer...
	if (sa_records) omgt_sa_free_records(sa_records);
	// ...and close our session
	omgt_close_port(port);
fail1:
	return exitcode;
}
