// core API
#include <opamgt/opamgt.h>
// extensions for SA queries
#include <opamgt/opamgt_sa.h>

int main(int argc, char ** argv)
{
	OMGT_STATUS_T status = OMGT_STATUS_SUCCESS;
	int exitcode = 0;
	int i;
	struct omgt_port * port = NULL;
	int num_records;
	STL_PORTINFO_RECORD *pi_records = NULL;


	// create a session
	status = omgt_open_port_by_num(&port, 1 /* hfi */, 1 /* port */, NULL);
	if (OMGT_STATUS_SUCCESS != status) {
		fprintf(stderr, "failed to open port\n");
		exitcode=1;
		goto fail1;
	}


	// specify how and what we want to query by
	omgt_sa_selector_t selector;
	selector.InputType = InputTypeLid;
	selector.InputValue.PortInfoRecord.Lid = 1;

	// execute query synchronously
	status = omgt_sa_get_portinfo_records(port, &selector, &num_records, &pi_records);
	if (status != OMGT_STATUS_SUCCESS) {
		exitcode=1;
		fprintf(stderr, "failed to execute query. MadStatus=0x%x\n", omgt_get_sa_mad_status(port));
		goto fail2;
	}

	if (!num_records) {
		// we can check result count independent of result type
		printf("No records found.\n");
	} else {
		for (i = 0; i < num_records; ++i) {
			// the result is a set of SA records, which often follow a pattern
			// of including a RID section containing top-level identification of
			// the record, and an encapsulated SM payload.
			//
			// in this case:
			//   r->RID: contains the LID and port number as record identifiers
			//   r->PortInfo: the encapsulated subnet management structure (STL_PORT_INFO)
			STL_PORTINFO_RECORD * r = &pi_records[i]; // sa
			printf("PortNum: %2u   PortLID: 0x%08x\n", r->RID.PortNum, r->RID.EndPortLID);
		}
	}

fail2:
	// free our result buffer...
	if (pi_records) omgt_sa_free_records(pi_records);
	// ...and close our session
	omgt_close_port(port);
fail1:
	return exitcode;
}

