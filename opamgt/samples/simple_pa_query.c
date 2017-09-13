/* This file shows a simple example of requesting and printing OPA
 * port counters data
 *
 */

//core API
#include <opamgt/opamgt.h>
//extensions for PA queries
#include <opamgt/opamgt_pa.h>
#include <inttypes.h>


int main()
{
	struct omgt_port * port = NULL;
	OMGT_STATUS_T status;
	int exitcode = 0;

	STL_PA_IMAGE_ID_DATA image_ID = {0};
	STL_PA_IMAGE_INFO_DATA image_info;

	status = omgt_open_port_by_num(&port, 1, 1, NULL);
	if(OMGT_STATUS_SUCCESS != status) {
		fprintf(stderr, "Failed to open port or initialize PA connection\n");
		exitcode=1;
		goto fail1;
	}

	if (omgt_pa_get_image_info(port, image_ID, &image_info)){
		fprintf(stderr, "Failed to get PA image\n");
		exitcode=1;
		goto fail2;
	}

	printf("Sweep start: %s", ctime((time_t *)&image_info.sweepStart));

	STL_PORT_COUNTERS_DATA port_counters;
	/*loop here, collecting counters and storing or displaying*/
	/*maybe show capability to get specific images*/
	if (omgt_pa_get_port_stats(port, image_ID, 1, 1, &image_ID, &port_counters, NULL, 0, 1)){
		fprintf(stderr, "Failed to get port counters\n");
		exitcode=1;
		goto fail2;
	}

	printf("Port Counters Data:\n");
	printf("portXmitData:               %"PRIu64"\n", port_counters.portXmitData);
	printf("portRcvData:                %"PRIu64"\n", port_counters.portRcvData);
	printf("portXmitPkts:               %"PRIu64"\n", port_counters.portXmitPkts);
	printf("portRcvPkts:                %"PRIu64"\n", port_counters.portRcvPkts);
	printf("localLinkIntegrityErrors:   %"PRIu64"\n", port_counters.localLinkIntegrityErrors);
	printf("linkDowned:                 %u\n", port_counters.linkDowned);

fail2:
	omgt_close_port(port);
fail1:
	return exitcode;
}
