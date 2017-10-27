/* This sample demonstrates the use of the SA swcost query
 * to select a set of nodes/switches for a job
 *
 */
// core API
#include <opamgt.h>
// swcost query
#include <opamgt_sa.h>

// "Edge" Switch
typedef struct {
	STL_LID lid; // identify switches by LID. Easiest when using cost records
	STL_NODE_RECORD *nodes [48]; // list of hosts attached to this switch
	int last_host_index; // index in previous list of last host
} opa_switch;


/* Helper function to retrieve the LID
 * of the neighbor of the supplied node
 * Used to determine edge switches, those with hosts attached
 */
STL_LID get_neighbor (STL_NODE_RECORD *node_record, STL_LINK_RECORD *link_records, int num_link_records)
{
	STL_LID lid1, lid2;
	int i;
	for (i = 0; i < num_link_records; ++i) {
		STL_LINK_RECORD *link_record = &link_records[i];
		lid1 = link_record->RID.FromLID;
		lid2 = link_record->ToLID;

		if (node_record->RID.LID == lid1) {
			return lid2;
		} else if (node_record->RID.LID == lid2) {
			return lid1;
		}
	}

	return -1;
}

/* Helper function to determine if list of lids contains
 * a specific lid
 * */
int contains (STL_LID * array, STL_LID lid, int array_size)
{
	int i;
	for (i = 0; i < array_size; ++i){
		if (array[i] == lid) return 1;
	}
	return 0;
}

int main(int argc, char **argv)
{
	OMGT_STATUS_T status = OMGT_STATUS_SUCCESS;
	int exitcode = 0;

	struct omgt_port *port = NULL;
	int num_host_records, num_link_records, num_cost_records, num_fabricinfo_records, num_classportinfo_records;

	STL_LINK_RECORD *link_records;
	STL_NODE_RECORD *host_records;
	STL_FABRICINFO_RECORD *fabricinfo_records;
	STL_SWITCH_COST_RECORD *cost_records = NULL;
	STL_CLASS_PORT_INFO *classportinfo_records = NULL;

	opa_switch *edge_switches;
	opa_switch **used_switches;
	STL_LID *visited_switch_lids;


	int last_switch_index = 0;
	int i, j;
	int requested_hosts = 0, hosts_needed = 0;
	int DEBUG = 0; //set to turn on some additional output

	if (argc > 1) {
		requested_hosts = strtoul(argv[1], NULL, 0);
		if (DEBUG) printf("Requested Hosts: %u\n", requested_hosts);
	} else {
		fprintf(stderr, "Usage: %s <num_hosts>\n", argv[0]);
		exitcode = 1;
		goto done;
	}

	// create a session
	status = omgt_open_port_by_num(&port, 1, 1, NULL);
	if (OMGT_STATUS_SUCCESS != status) {
		fprintf(stderr, "failed to open port\n");
		exitcode = 1;
		goto done;
	}

	omgt_sa_selector_t selector;
	selector.InputType = InputTypeNoInput;

	/* Initial set-up, request topology information from SA
	 *
	 * We request FabricInfo, Link, and (HFI) Node records.
	 */
	status = omgt_sa_get_fabric_info_records(port, &selector, &num_fabricinfo_records, &fabricinfo_records);
	if (OMGT_STATUS_SUCCESS != status || num_fabricinfo_records < 1) {
		fprintf(stderr, "failed to get fabricinfo. MADStatus=0x%x\n", omgt_get_sa_mad_status(port));
		exitcode = 1;
		goto cleanup;
	}

	edge_switches = malloc(sizeof(opa_switch) * fabricinfo_records[0].NumSwitches); // list of switches with hosts
	used_switches = malloc(sizeof(opa_switch *) * fabricinfo_records[0].NumSwitches); // list of switches used in job
	visited_switch_lids = malloc(sizeof(STL_LID) * fabricinfo_records[0].NumSwitches); // list of switch lids visited during cost decisions

	if (!edge_switches || !used_switches || ! visited_switch_lids){
		fprintf(stderr, "failed to allocate memory.\n");
		exitcode = 1;
		goto cleanup;
	}

	status = omgt_sa_get_link_records(port, &selector, &num_link_records, &link_records);
	if (OMGT_STATUS_SUCCESS != status) {
		fprintf(stderr, "failed to execute link record query. MADStatus=0x%x\n", omgt_get_sa_mad_status(port));
		exitcode = 1;
		goto cleanup;
	}

	selector.InputType = InputTypeNodeType; // select records by type
	selector.InputValue.NodeRecord.NodeType = IBA_NODE_CHANNEL_ADAPTER; // select only HFIs (Channel Adapter)
	status = omgt_sa_get_node_records(port, &selector, &num_host_records, &host_records);
	if (OMGT_STATUS_SUCCESS != status) {
		fprintf(stderr, "failed to execute node record query. MADStatus=0x%x\n", omgt_get_sa_mad_status(port));
		exitcode = 1;
		goto cleanup;
	}

	if (DEBUG) printf("hosts: %u sws: %u\n", fabricinfo_records[0].NumHFIs, fabricinfo_records[0].NumSwitches);
	if (requested_hosts > fabricinfo_records[0].NumHFIs) {
		fprintf(stderr, "Error: Requested number of hosts exceeds number of hosts available\n");
		exitcode = 1;
		goto cleanup;
	}

	/* Determine the edge switches in the fabric along with associated hosts*/
	for (i = 0; i < num_host_records; ++i) {
		STL_NODE_RECORD *host_record = &host_records[i];

		STL_LID neighbor_lid = get_neighbor(host_record, link_records, num_link_records);
		if (-1 == neighbor_lid){
			fprintf(stderr, "Node missing neighbor\n");
			exitcode = 1;
			goto cleanup;
		}
		int in_list = 0;
		int swIndex = 0;
		//check if switch in list already
		for (j = 0; j < last_switch_index; ++j) {
			if (neighbor_lid == edge_switches[j].lid) {
				in_list = 1;
				swIndex = j;
				break;
			}
		}
		if (!in_list) {
			edge_switches[last_switch_index++].lid = neighbor_lid;
			edge_switches[last_switch_index - 1].nodes[0] = host_record;

		} else {
			//add this host to switch's host list
			edge_switches[swIndex].nodes[++edge_switches[swIndex].last_host_index] = host_record;
		}
	}

	if (DEBUG) {
		printf("Edge Switches:\n");
		for (i = 0; i < last_switch_index; ++i) {
			printf("0x%x\n", edge_switches[i].lid);
			for (j = 0; j <= edge_switches[i].last_host_index; ++j) {
				printf("---Host %s\n", edge_switches[i].nodes[j]->NodeDesc.NodeString);
			}
		}
		printf("\n\n");
	}


	/* Determine which hosts to use, attempt to use hosts on a single switch*/
	int max_hosts_switch_index = 0, max_hosts = 0;
	for (i = 0; i < last_switch_index; ++i) {
		// Save the switch with the max number of hosts for later
		// use if we need hosts on multiple switches
		if (edge_switches[i].last_host_index > max_hosts) {
			max_hosts = edge_switches[i].last_host_index;
			max_hosts_switch_index = i;
		}
		if (edge_switches[i].last_host_index >= requested_hosts - 1) {
			printf("Job Plan: \nSwitch %u\n", edge_switches[i].lid);
			printf("-----------------------\n");
			for (j = 0; j < requested_hosts; ++j) {
				printf("---Host %s\n", edge_switches[i].nodes[j]->NodeDesc.NodeString);
			}
			goto cleanup;
		}
	}

	//===================PATH COSTS============================
	/* There were no switches with the requisite number of hosts,
	 * so use Switch Cost Records to use hosts on least cost
	 * paths. We must check the capability of the SA before using
	 * this feature
	 *
	 *
	 * We can request the "full" cost matrix which includes costs
	 * among all switches (actually only the upper half of the matrix)
	 * OR
	 * We can request one row of the cost matrix. In the following we
	 * request the row corresponding to our previously saved switch. This
	 * gets us the costs from the saved switch to every other switch
	 */

	selector.InputType = InputTypeNoInput;
	status = omgt_sa_get_classportinfo_records(port, &selector, &num_classportinfo_records, &classportinfo_records);
	if (OMGT_STATUS_SUCCESS != status || num_classportinfo_records != 1) {
		fprintf(stderr, "failed to execute classportinfo record query. MADStatus=0x%x\n", omgt_get_sa_mad_status(port));
		exitcode = 1;
		goto cleanup;
	}

	if (! (classportinfo_records[0].CapMask && STL_SA_CAPABILITY2_SWCOSTRECORD_SUPPORT)) {
		fprintf(stderr, "SA does not support switchcost records\n");
		exitcode = 1;
		goto cleanup;
	}

	selector.InputType = InputTypeLid; // Costs can be looked up by LID
	selector.InputValue.SwitchCostRecord.Lid = edge_switches[max_hosts_switch_index].lid;

	status = omgt_sa_get_switchcost_records(port, &selector, &num_cost_records, &cost_records);
	if (OMGT_STATUS_SUCCESS != status) {
		fprintf(stderr, "failed to execute cost record query. MADStatus=0x%x\n", omgt_get_sa_mad_status(port));
		exitcode = 1;
		goto cleanup;
	}

	hosts_needed = requested_hosts - (max_hosts + 1);
	used_switches[0] = &edge_switches[max_hosts_switch_index]; // keep track of edge switches we're using
	STL_LID switch_lid;
	visited_switch_lids[0] = used_switches[0]->lid;  // keep track of switches we've already considered
	int num_switches = 1, num_lids = 1;
	while(hosts_needed > 0 && num_switches < last_switch_index) {
		int min_cost = -1;
		for (i = 0; i < num_cost_records; ++i) {
			for (j = 0; j < STL_SWITCH_COST_NUM_ENTRIES; ++j) {
				if (((min_cost < 0) || (cost_records[i].Cost[j].value < min_cost)) &&
						!contains(visited_switch_lids, cost_records[i].Cost[j].DLID, num_lids) &&
						(cost_records[i].Cost[j].DLID > 0)) {
					//we should also check that this is an edge switch, this is done below
					min_cost = cost_records[i].Cost[j].value;
					switch_lid = cost_records[i].Cost[j].DLID;
				}
			}
		}

		if (min_cost > 0){
			// look up next switch, only include edge switches
			for (i = 0; i < last_switch_index; ++i) {
				if (edge_switches[i].lid == switch_lid) {
					used_switches[num_switches] = &edge_switches[i];
					num_switches++;

					hosts_needed -= (edge_switches[i].last_host_index + 1);
					break;
				}
			}
			visited_switch_lids[num_lids] = switch_lid; //mark this switch as visited
			num_lids++;
		}
	}

	if (hosts_needed > 0) {
		fprintf(stderr, "Error: Could not allocate job hosts\n");
		exitcode = 1;
		goto cleanup;
	}

	hosts_needed = requested_hosts;
	printf("Job Plan: \n");
	printf("-----------------------\n");
	for (i = 0; i < num_switches; ++i){
		printf("Switch %u\n", used_switches[i]->lid);
		for (j = 0; j <= used_switches[i]->last_host_index; ++j) {
			printf("---Host %s\n", used_switches[i]->nodes[j]->NodeDesc.NodeString);
			if (--hosts_needed <= 0) goto cleanup;
		}
	}


cleanup:
	if (edge_switches) free(edge_switches);
	if (used_switches) free(used_switches);
	if (visited_switch_lids) free(visited_switch_lids);

	// free our result buffers
	if (fabricinfo_records) omgt_sa_free_records(fabricinfo_records);
	if (classportinfo_records) omgt_sa_free_records(classportinfo_records);
	if (cost_records) omgt_sa_free_records(cost_records);
	if (host_records) omgt_sa_free_records(host_records);

	// close our session
	omgt_close_port(port);

done:
	return exitcode;
}
