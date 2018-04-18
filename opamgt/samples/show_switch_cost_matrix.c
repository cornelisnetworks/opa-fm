/* Query SA for Switch Cost  Matrix data
 * and print it to a file. Monitor for any
 * changes and update.
 *
 */
// core API
#include <opamgt.h>
// swcost query
#include <opamgt_sa.h>
// fabric change notice
#include <opamgt_sa_notice.h>

#include <stdio.h>

typedef struct opa_switch{
	STL_LID lid;
	const char * name;
	struct opa_switch *next;
}opa_switch;

FILE * matrix_file;

void free_cost(uint16_t ***cost, int num_rows){
	int i;

	if (!*cost) return;

	for(i = 0; i < num_rows; ++i) {
		if ((*cost)[i]) {
			free((*cost)[i]);
			(*cost)[i] = NULL;
		}
	}
	free(*cost);
	*cost = NULL;
}

void free_switch_list(opa_switch **switchlist_head)
{
	if (!*switchlist_head) return;
	opa_switch * temp = *switchlist_head, *temp2;
	while(temp){
		temp2 = temp->next;
		free(temp);
		temp = temp2;
	}
	*switchlist_head = NULL;
}

/* Helper function to get the associated name for a particular lid
 */
const char * get_name(STL_LID lid, STL_NODE_RECORD *node_records, int num_node_records)
{
	int  i;
	for (i = 0; i < num_node_records; ++i) {
		if (node_records[i].RID.LID == lid)
			return (const char*) node_records[i].NodeDesc.NodeString;
	}
	return "";
}

/* Print Cost Matrix using the Switch Names
 */
void print_matrix(uint16_t **cost, opa_switch *switchlist_head)
{
	fprintf(matrix_file, "%64s", "");
	opa_switch * iter = switchlist_head;
	while(iter){
		// Node Descriptions are up to 64 bytes
		fprintf(matrix_file, "%64s", iter->name);
		iter = iter->next;
	}
	fprintf(matrix_file, "\n");

	iter = switchlist_head;
	opa_switch * iter2;
	while(iter) {
		fprintf(matrix_file, "%-64s", iter->name);
		iter2 = switchlist_head;
		while(iter2) {
			if(cost[iter->lid])
				fprintf(matrix_file, "%64d" , cost[iter->lid][iter2->lid]);
			iter2 = iter2->next;
		}
		iter = iter->next;
		fprintf(matrix_file, "\n");
	}
	fprintf(matrix_file, "\n\n\n");
}


int main(int argc, char **argv)
{
	OMGT_STATUS_T status = OMGT_STATUS_SUCCESS;
	int exitcode = 0;

	struct omgt_port *port = NULL;
	omgt_sa_selector_t selector;

	int num_classportinfo_records, num_fabricinfo_records, num_cost_records, num_switch_records;
	int num_nodes = 0;

	STL_CLASS_PORT_INFO *classportinfo_records = NULL;
	STL_FABRICINFO_RECORD *fabricinfo_records = NULL;
	STL_SWITCH_COST_RECORD *cost_records = NULL;
	STL_NODE_RECORD *switch_records = NULL;

	STL_NOTICE *notice = NULL;
	size_t notice_len = 0;
	struct omgt_port *context = NULL;

	uint16_t **cost = NULL;
	int i,j;

	if (argc < 2) {
		fprintf(stderr, "Usage: %s <output_file>\n", argv[0]);
		exitcode = 1;
		goto done;
	}

	matrix_file = fopen(argv[1], "w");
	if (!matrix_file) {
		fprintf(stderr, "could not open output file for writing\n");
		exitcode = 1;
		goto done;
	}

	// create a session
	status = omgt_open_port_by_num(&port, 1, 1, NULL);
	if (OMGT_STATUS_SUCCESS != status) {
		fprintf(stderr, "failed to open port\n");
		exitcode = 1;
		goto close_file;
	}

	// Determine if SA has Switch Cost Record capabiility
	selector.InputType = InputTypeNoInput;
	status = omgt_sa_get_classportinfo_records(port, &selector, &num_classportinfo_records, &classportinfo_records);
	if (OMGT_STATUS_SUCCESS != status || num_classportinfo_records != 1) {
		fprintf(stderr, "failed to execute classportinfo record query. MADStatus=0x%x\n", omgt_get_sa_mad_status(port));
		exitcode = 1;
		goto close_port;
	}

	if (! (classportinfo_records[0].CapMask && STL_SA_CAPABILITY2_SWCOSTRECORD_SUPPORT)) {
		fprintf(stderr, "SA does not support switchcost records\n");
		exitcode = 1;
		goto close_port;
	}

	// Register for Cost Matrix Change Trap
	if ((status = omgt_sa_register_trap(port, STL_TRAP_COST_MATRIX_CHANGE, port)) != OMGT_STATUS_SUCCESS) {
		fprintf(stderr, " Error: Could not register for Trap %u: %s (%u)\n",
				STL_TRAP_COST_MATRIX_CHANGE, omgt_status_totext(status), status);
		exitcode = 1;
		goto close_port;
	}

	opa_switch *head_switch = NULL, *temp_switch = NULL;
	while(!exitcode){
		selector.InputType = InputTypeNoInput;

		/*	Query SA for fabric information to determine how big to make
		 *	our own cost matrix storage
		 */
		status = omgt_sa_get_fabric_info_records(port, &selector, &num_fabricinfo_records, &fabricinfo_records);
		if (OMGT_STATUS_SUCCESS != status || num_fabricinfo_records < 1) {
			fprintf(stderr, "failed to get fabricinfo. MADStatus=0x%x\n", omgt_get_sa_mad_status(port));
			exitcode = 1;
			goto cleanup;
		}

		/* Using total number of nodes will allocate more space than necessary,
		 * but allows simpler processing of cost data
		 */
		num_nodes = fabricinfo_records[0].NumSwitches + fabricinfo_records[0].NumHFIs;
		if ((cost = calloc(1, num_nodes * sizeof(uint16_t*))) == NULL) {
			fprintf(stderr, "failed to allocate memory\n");
			exitcode = 1;
			goto cleanup;
		}

		/*	Query SA for switch records so we can have the names of the switches
		 *	as the cost records are defined with LIDs
		 */
		selector.InputType = InputTypeNodeType; // select records by type
		selector.InputValue.NodeRecord.NodeType = IBA_NODE_SWITCH; // select only Switches
		status = omgt_sa_get_node_records(port, &selector, &num_switch_records, &switch_records);
		if (OMGT_STATUS_SUCCESS != status) {
			fprintf(stderr, "failed to execute node record query. MADStatus=0x%x\n", omgt_get_sa_mad_status(port));
			exitcode = 1;
			goto cleanup;
		}

		/* Create a linked list of switches in this fabric
		 */
		for (i = 0; i < num_switch_records; ++i) {
			opa_switch * next_switch;
			if ((next_switch = malloc(sizeof(opa_switch))) == NULL) {
				fprintf(stderr, "failed to allocate memory\n");
				exitcode = 1;
				goto cleanup;
			}

			next_switch->lid = switch_records[i].RID.LID;
			next_switch->name = get_name(switch_records[i].RID.LID, switch_records, num_switch_records);
			next_switch->next = NULL;

			if (temp_switch)
				temp_switch->next = next_switch;
			else
				head_switch = next_switch;

			temp_switch = next_switch;
		}
		temp_switch = NULL;

		// reset selector to appropriate type for cost query
		selector.InputType = InputTypeNoInput;
		status = omgt_sa_get_switchcost_records(port, &selector, &num_cost_records, &cost_records);
		if (OMGT_STATUS_SUCCESS != status) {
			fprintf(stderr, "failed to execute cost record query. MADStatus=0x%x\n", omgt_get_sa_mad_status(port));
			exitcode = 1;
			goto cleanup;
		}

		/* Copy returned Cost matrix data into our own format
		 * When requesting the entire matrix, only the lower
		 * diagonal is returned as it is symmetric
		 */
		int slid, dlid;
		STL_SWITCH_COST_RECORD cost_record;
		for (i = 0; i < num_cost_records; ++i) {
			cost_record = cost_records[i];
			slid = cost_record.SLID;
			/* Cost records contain 64 entries, but all may not be filled
			 * Checking the DLID is > 0 ensures the entry is valid
			 */
			for (j = 0; j < STL_SWITCH_COST_NUM_ENTRIES && cost_record.Cost[j].DLID > 0; ++j) {
				dlid = cost_record.Cost[j].DLID;

				// Initialize all entries in cost to zero.
				// The entries of interest will be overwritten with
				// actual cost values from the SA
				if (!cost[slid]) {
					if ((cost[slid] = calloc(1, num_nodes * sizeof(uint16_t))) == NULL) {
						fprintf(stderr, "failed to allocate memory\n");
						exitcode = 1;
						goto cleanup;
					}
				}
				if (!cost[dlid]) {
					if ((cost[dlid] = calloc(1, num_nodes * sizeof(uint16_t))) == NULL) {
						fprintf(stderr, "failed to allocate memory\n");
						exitcode = 1;
						goto cleanup;
					}
				}
				// add this cost to matrix for slid/dlid
				cost[slid][dlid] = cost[dlid][slid] = cost_record.Cost[j].value;
			}
		}

		print_matrix(cost, head_switch);

		printf("\nMonitoring for any changes...\n");
		fflush(matrix_file);

		// monitor cluster for changes, -1 = indefinite wait time
		if ((status = omgt_sa_get_notice_report(port, &notice, &notice_len, (void **)&context, -1)) != OMGT_STATUS_SUCCESS) {
			fprintf(stderr, "Error: Could not wait for Notice: %s (%u)\n",
					omgt_status_totext(status), status);
			exitcode = 1;
			goto cleanup;
		}

		// we only registered for Cost Matrix Change notice, if we got something else it's an error
		if (notice->Attributes.Generic.TrapNumber != STL_TRAP_COST_MATRIX_CHANGE) {
			fprintf(stderr, "Unhandled Trap Received: %u\n", notice->Attributes.Generic.TrapNumber);
			exitcode = 1;
			goto cleanup;
		}

cleanup:
		if (notice) {
			free(notice);
			notice = NULL;
		}

		if (fabricinfo_records) {
			omgt_sa_free_records(fabricinfo_records);
			fabricinfo_records = NULL;
		}
		if (cost_records) {
			omgt_sa_free_records(cost_records);
			cost_records = NULL;
		}
		if (switch_records) {
			omgt_sa_free_records(switch_records);
			switch_records = NULL;
		}

		free_switch_list(&head_switch);
		free_cost(&cost, num_nodes);

	} //end while


	if ((status = omgt_sa_unregister_trap(port, STL_TRAP_COST_MATRIX_CHANGE)) != OMGT_STATUS_SUCCESS) {
		fprintf(stderr, "Error: Could not unregister for Trap %u: %s (%u)\n",
				STL_TRAP_COST_MATRIX_CHANGE, omgt_status_totext(status), status);
		if (!exitcode) exitcode = 1;
	}

close_port:
	if (classportinfo_records) omgt_sa_free_records(classportinfo_records);

	//close our session
	omgt_close_port(port);

close_file:
	fclose(matrix_file);

done:
	return exitcode;
}
