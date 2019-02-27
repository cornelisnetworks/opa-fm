/*
 * Copyright (c) 2018, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Intel Corporation nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cmd_query.h"

#include "utils.h"
#include <getopt.h>
#include <opamgt_sa.h>


const char* cmd_exist_help =
	"Usage:\n"
	"    opafmvf exist vfname\n"
	"    opafmvf exist --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"    vfname                    - virtual fabric name\n"
	"\n"
	"Query if given virtual fabric exists in the fabric.\n"
	"\n"
	"Return:\n"
	"    0 - given virtual fabric exists in the fabric\n"
	"    1 - given virtual fabric does not exist in the fabric\n"
	"    2 - invalid arguments were provided or other error occurred\n";

const char* cmd_ismember_help =
	"Usage:\n"
	"    opafmvf ismember vfname portguid...\n"
	"    opafmvf ismember --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"    vfname                    - virtual fabric name\n"
	"    portguid...               - list of port GUIDs (ex. 0x00117501a0000380)\n"
	"\n"
	"Query if specified ports are member of given virtual fabric.\n"
	"\n"
	"Return:\n"
	"    0 - all ports are member of given virtual fabric\n"
	"    1 - some ports are still not member of given virtual fabric\n"
	"    2 - invalid arguments were provided or other error occurred\n";

const char* cmd_isnotmember_help =
	"Usage:\n"
	"    opafmvf isnotmember vfname portguid...\n"
	"    opafmvf isnotmember --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"    vfname                    - virtual fabric name\n"
	"    portguid...               - list of port GUIDs (ex. 0x00117501a0000380)\n"
	"\n"
	"Query if specified ports are not member of given virtual fabric.\n"
	"\n"
	"Return:\n"
	"    0 - none of ports is member of given virtual fabric\n"
	"    1 - some ports are still member of given virtual fabric\n"
	"    2 - invalid arguments were provided or other error occurred\n";

const char* cmd_exist_usage =
	"Usage:\n"
	"    opafmvf exist vfname\n"
	"    opafmvf exist --help\n";

const char* cmd_ismember_usage =
	"Usage:\n"
	"    opafmvf ismember vfname portguid...\n"
	"    opafmvf ismember --help\n";

const char* cmd_isnotmember_usage =
	"Usage:\n"
	"    opafmvf isnotmember vfname portguid...\n"
	"    opafmvf isnotmember --help\n";


static CmdStatus
get_pkey_by_vf_name(const char* vf_name, struct omgt_port* port, uint16* pkey)
{
	OMGT_STATUS_T status = OMGT_STATUS_SUCCESS;
	CmdStatus cmd_status = CMD_STATUS_ERROR;
	int num_records = 0;
	STL_VFINFO_RECORD* vf_records = NULL;
	omgt_sa_selector_t selector;
	memset(&selector, 0, sizeof(selector));
	selector.InputType = InputTypeNoInput;
	status = omgt_sa_get_vfinfo_records(port, &selector, &num_records, &vf_records);
	if (status != OMGT_STATUS_SUCCESS) {
		log_error("Cannot query FM");
		cmd_status = CMD_STATUS_ERROR;
		goto error;
	}

	cmd_status = CMD_STATUS_NOT_EXIST;

	for (int i = 0; i < num_records; i++) {
		STL_VFINFO_RECORD* record = &vf_records[i];
		if (!strcmp((char*)record->vfName, vf_name)) {
			*pkey = record->pKey;
			cmd_status = CMD_STATUS_EXIST;
			break;
		}
	}

error:
	omgt_sa_free_records(vf_records);
	return cmd_status;

}

static CmdStatus
check_membership(const char* vf_name, const uint64_t guids[], size_t guids_count, bool check_are_members)
{
	int found_guids = 0;
	struct omgt_port* port = NULL;
	OMGT_STATUS_T status = OMGT_STATUS_SUCCESS;

	status = omgt_open_port_by_num(&port, 0, 0, NULL);
	if (status != OMGT_STATUS_SUCCESS) {
		log_error("Cannot open HFI port");
		return CMD_STATUS_ERROR;
	}

	uint16 vf_pkey = 0;
	CmdStatus cmd_status = get_pkey_by_vf_name(vf_name, port, &vf_pkey);
	if (cmd_status == CMD_STATUS_NOT_EXIST)
		goto end;
	else if (cmd_status != CMD_STATUS_EXIST)
		goto error;

	uint16 vf_pkey_base = vf_pkey & 0x7FFF;
	int num_pkey_records = 0;
	int num_lid_records = 0;
	bool found = false;
	omgt_sa_selector_t selector;
	memset(&selector, 0, sizeof(selector));

	// TODO: discuss - later (second phase or somewhen) FM should keep mapping and be able
	//					to give membership information directly
	for (int gidx = 0; gidx < guids_count; gidx++) {
		uint64_t guid = guids[gidx];
		selector.InputType = InputTypePortGuid;
		selector.InputValue.NodeRecord.PortGUID = guid;
		uint32* lids = NULL;
		uint32 lid = UINT32_MAX;
		// we can't get pkey_table_records from guid, we need lid to be able to do it
		status = omgt_sa_get_lid_records(port, &selector, &num_lid_records, &lids);
		if (status != OMGT_STATUS_SUCCESS) {
			omgt_sa_free_records(lids);		// don't trust lib function in case of error
			log_error("Cannot query FM, status: %d", status);
			cmd_status = CMD_STATUS_ERROR;
			goto error;
		}
		if (num_lid_records > 0) {
			// We need lid for given guid - if mapping is not 1:1, first one is enough
			lid = lids[0];
		}
		omgt_sa_free_records(lids);
		if (num_lid_records == 0) {
			continue;
		}

		STL_P_KEY_TABLE_RECORD* pkey_records = NULL;
		selector.InputType = InputTypeLid;
		selector.InputValue.NodeRecord.Lid = lid;
		status = omgt_sa_get_pkey_table_records(port, &selector, &num_pkey_records, &pkey_records );
		if (status != OMGT_STATUS_SUCCESS) {
			omgt_sa_free_records(pkey_records);		// don't trust lib function in case of error
			log_error("Cannot query FM, status: %d", status);
			cmd_status = CMD_STATUS_ERROR;
			goto error;
		}
		found = false;
		for (int j = 0; j < num_pkey_records; j++) {
			STL_P_KEY_TABLE_RECORD* pkey_record = &pkey_records[j];
			for (int k = 0; k < NUM_PKEY_ELEMENTS_BLOCK; k++) {
				STL_PKEY_ELEMENT pkey_element = pkey_record->PKeyTblData.PartitionTableBlock[k];
				if (pkey_element.s.P_KeyBase == vf_pkey_base) {
					found_guids++;
					if (!check_are_members) {
						omgt_sa_free_records(pkey_records);
						goto end;
					}
					found = true;
					break;
				}
			}
			if (found) {
				break;
			}
		}
		omgt_sa_free_records(pkey_records);
	}

end:
	if (check_are_members)
		cmd_status = found_guids == guids_count ? CMD_STATUS_ALL_MEMBER : CMD_STATUS_SOME_MEMBER;
	else
		cmd_status = found_guids == 0 ? CMD_STATUS_NONE_MEMBER : CMD_STATUS_SOME_MEMBER;

error:
	omgt_close_port(port);
	return cmd_status;
}

CmdStatus
cmd_exist(int argc, char* argv[])
{
	const char*	vf_name = NULL;

	/* parse options */
	static struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{0, 0, 0, 0}
	};

	optind = 0;
	while (true) {
		int c = getopt_long(argc, argv, "", long_options, NULL);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(cmd_exist_help);
			return CMD_STATUS_OK;
		default:
			print_usage(cmd_exist_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* get virtual fabric name */
	if (!extract_vf_name(argc, argv, optind++, &vf_name)) {
		print_usage(cmd_exist_usage);
		return CMD_STATUS_INVARG;
	}

	/* check not parsed arguments */
	if (optind < argc) {
		log_error("%s: too many arguments", argv[0]);
		print_usage(cmd_exist_usage);
		return CMD_STATUS_INVARG;
	}

	/* this command can be executed from many processes in the same time */
	/* therefore it is not needed to acquire a program lock */

	/* check whether virtual fabric exist */
	struct omgt_port* port = NULL;

	if (omgt_open_port_by_num(&port, 0, 0, NULL) != OMGT_STATUS_SUCCESS) {
		log_error("Cannot open HFI port");
		return CMD_STATUS_ERROR;
	}

	uint16 vf_pkey = 0;
	CmdStatus status = get_pkey_by_vf_name(vf_name, port, &vf_pkey);

	if (status == CMD_STATUS_EXIST)
		log_info("Virtual fabric '%s' exists", vf_name);
	else if (status == CMD_STATUS_NOT_EXIST)
		log_info("Virtual fabric '%s' does not exist", vf_name);

	omgt_close_port(port);

	return status;
}

CmdStatus
cmd_ismember(int argc, char* argv[])
{
	const char*	vf_name = NULL;
	uint64_t*	guids = NULL;
	size_t		guids_count = 0;

	/* parse options */
	static struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{0, 0, 0, 0}
	};

	optind = 0;
	while (true) {
		int c = getopt_long(argc, argv, "", long_options, NULL);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(cmd_ismember_help);
			return CMD_STATUS_OK;
		default:
			print_usage(cmd_ismember_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* get virtual fabric name */
	if (!extract_vf_name(argc, argv, optind++, &vf_name)) {
		print_usage(cmd_ismember_usage);
		return CMD_STATUS_INVARG;
	}

	/* get port guids */
	if (!extract_port_guids(argc, argv, optind, &guids, &guids_count)) {
		print_usage(cmd_ismember_usage);
		return CMD_STATUS_INVARG;
	}

	/* this command can be executed from many processes in the same time */
	/* therefore it is not needed to acquire a program lock */

	/* check membership */
	CmdStatus status = check_membership(vf_name, guids, guids_count, true);

	if (status == CMD_STATUS_ALL_MEMBER)
		log_info("All ports are member of virtual fabric '%s'", vf_name);
	else if (status == CMD_STATUS_SOME_MEMBER)
		log_info("Some ports are still not member of virtual fabric '%s'", vf_name);

	/* release guids */
	free(guids);

	return status;
}

CmdStatus
cmd_isnotmember(int argc, char* argv[])
{
	const char*	vf_name = NULL;
	uint64_t*	guids = NULL;
	size_t		guids_count = 0;

	/* parse options */
	static struct option long_options[] = {
		{"help", no_argument, 0, 'h'},
		{0, 0, 0, 0}
	};

	optind = 0;
	while (true) {
		int c = getopt_long(argc, argv, "", long_options, NULL);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(cmd_isnotmember_help);
			return CMD_STATUS_OK;
		default:
			print_usage(cmd_isnotmember_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* get virtual fabric name */
	if (!extract_vf_name(argc, argv, optind++, &vf_name)) {
		print_usage(cmd_isnotmember_usage);
		return CMD_STATUS_INVARG;
	}

	/* get port guids */
	if (!extract_port_guids(argc, argv, optind, &guids, &guids_count)) {
		print_usage(cmd_isnotmember_usage);
		return CMD_STATUS_INVARG;
	}

	/* this command can be executed from many processes in the same time */
	/* therefore it is not needed to acquire a program lock */

	/* check membership */
	CmdStatus status = check_membership(vf_name, guids, guids_count, false);

	if (status == CMD_STATUS_NONE_MEMBER)
		log_info("None of ports is member of virtual fabric '%s'", vf_name);
	else if (status == CMD_STATUS_SOME_MEMBER)
		log_info("Some ports are still member of virtual fabric '%s'", vf_name);

	/* release guids */
	free(guids);

	return status;
}
