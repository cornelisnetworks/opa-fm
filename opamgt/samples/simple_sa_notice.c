// core API
#include <opamgt/opamgt.h>
// extensions for SA Notices
#include <opamgt/opamgt_sa_notice.h>

// extension to print gid using inet_ntop()
#include <arpa/inet.h>


int main(void)
{
	int exit_code = 0;
	struct omgt_port *port = NULL;
	OMGT_STATUS_T status;

	STL_NOTICE *notice = NULL;
	size_t notice_len = 0;
	struct omgt_port *context = NULL;

	/* Set Init Params */
	struct omgt_params params = {0};
	params.error_file = stderr;
	params.debug_file = NULL;
	/* Open in-band port */
	if ((status = omgt_open_port(&port, "hfi1_0", 1, &params)) != OMGT_STATUS_SUCCESS) {
		fprintf(stderr, "Error: Could not open port: %s (%u)\n",
			omgt_status_totext(status), status);
		return 1;
	}

	/* Register for Traps (Node Appear | Node Disappear) */
	if ((status = omgt_sa_register_trap(port, STL_TRAP_GID_NOW_IN_SERVICE, port)) != OMGT_STATUS_SUCCESS) {
		fprintf(stderr, "Error: Could not register for Trap %u: %s (%u)\n",
			STL_TRAP_GID_NOW_IN_SERVICE, omgt_status_totext(status), status);
		exit_code = 1;
		goto close_port;
	}
	if ((status = omgt_sa_register_trap(port, STL_TRAP_GID_OUT_OF_SERVICE, port)) != OMGT_STATUS_SUCCESS) {
		fprintf(stderr, "Error: Could not register for Trap %u: %s (%u)\n",
			STL_TRAP_GID_OUT_OF_SERVICE, omgt_status_totext(status), status);
		exit_code = 1;
		goto unreg_trap1;
	}

	/* Wait for Trap (-1 = indefinite wait time) */
	if ((status = omgt_sa_get_notice_report(port, &notice, &notice_len, (void **)&context, -1)) != OMGT_STATUS_SUCCESS) {
		fprintf(stderr, "Error: Could not wait for Notice: %s (%u)\n",
			omgt_status_totext(status), status);
		exit_code = 1;
		goto unreg_trap2;
	}
	if (notice_len < sizeof(STL_NOTICE)) {
		fprintf(stderr, "Error: Could not get Notice: Returned Length is less than expected: %zu < %zu\n",
			notice_len, sizeof(STL_NOTICE));
		exit_code = 2;
		goto free_notice;
	}
	if (port != context) {
		fprintf(stderr, "Error: Could not get Notice: Returned context (%p) is not port (%p)\n",
			context, port);
		exit_code = 2;
		goto free_notice;
	}

	/* Check Trap Number */
	switch (notice->Attributes.Generic.TrapNumber) {
	case STL_TRAP_GID_NOW_IN_SERVICE:
		/* New Node Appears */
		{
			char gid_buf[46] = {0};
			STL_TRAP_GID *new_gid = (STL_TRAP_GID *)&notice->Data[0];
			fprintf(stderr, "New Node: %s\n",
				inet_ntop(AF_INET6, new_gid, gid_buf, sizeof(gid_buf)));
		}
		break;
	case STL_TRAP_GID_OUT_OF_SERVICE:
		/* Node Disapears */
		{
			char gid_buf[46] = {0};
			STL_TRAP_GID *new_gid = (STL_TRAP_GID *)&notice->Data[0];
			fprintf(stderr, "Node Disappears: %s\n",
				inet_ntop(AF_INET6, new_gid, gid_buf, sizeof(gid_buf)));
		}
		break;
	default:
		fprintf(stderr, "Unhandled Trap Received: %u\n", notice->Attributes.Generic.TrapNumber);
		exit_code = 3;
	}

	/* Clean Up Notice */
free_notice:
	if (notice) {
		free(notice);
		notice = NULL;
	}

	/* Unregister Traps */
unreg_trap2:
	if ((status = omgt_sa_unregister_trap(port, STL_TRAP_GID_OUT_OF_SERVICE)) != OMGT_STATUS_SUCCESS) {
		fprintf(stderr, "Error: Could not unregister for Trap %u: %s (%u)\n",
			STL_TRAP_GID_OUT_OF_SERVICE, omgt_status_totext(status), status);
		if (!exit_code) exit_code = 1;
	}
unreg_trap1:
	if ((status = omgt_sa_unregister_trap(port, STL_TRAP_GID_NOW_IN_SERVICE)) != OMGT_STATUS_SUCCESS) {
		fprintf(stderr, "Error: Could not unregister for Trap %u: %s (%u)\n",
			STL_TRAP_GID_NOW_IN_SERVICE, omgt_status_totext(status), status);
		if (!exit_code) exit_code = 1;
	}

	/* Close port */
close_port:
	omgt_close_port(port);
	port = NULL;

	return exit_code;
}
