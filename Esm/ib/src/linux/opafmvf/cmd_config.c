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

#include "cmd_config.h"

#include "tenant.h"
#include "utils.h"
#include <getopt.h>


const char* cmd_create_help =
	"Usage:\n"
	"    opafmvf create [-f] --pkey=value vfname\n"
	"    opafmvf create --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"    -f/--force                - if virtual fabric configuration exists, remove\n"
	"                                and retry\n"
	"    -k/--pkey value           - mandatory PKey for virtual fabric\n"
	"                                16 bit value in range [0x0000, 0x7FFF)\n"
	"    vfname                    - virtual fabric name\n"
	"\n"
	"Create empty virtual fabric configuration with given unique name.\n"
	"\n"
	"Return:\n"
	"    0 - if virtual fabric configuration has been properly created\n"
	"    1 - if virtual fabric configuration already exists and -f/--force was not\n"
	"        provided\n"
	"    2 - invalid arguments were provided or other error occurred\n";

const char* cmd_delete_help =
	"Usage:\n"
	"    opafmvf delete [-f] vfname\n"
	"    opafmvf delete --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"    -f/--force                - delete virtual fabric configuration even if\n"
	"                                not empty, corrupted or already deleted\n"
	"    vfname                    - virtual fabric name\n"
	"\n"
	"Delete given virtual fabric configuration.\n"
	"\n"
	"Return:\n"
	"    0 - if virtual fabric configuration has been properly deleted\n"
	"    1 - if virtual fabric configuration is not empty or is corrupted\n"
	"        or is already deleted and -f/--force was not provided\n"
	"    2 - invalid arguments were provided or other error occurred\n";

const char* cmd_add_help =
	"Usage:\n"
	"    opafmvf add [-f] vfname portguid...\n"
	"    opafmvf add --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"    -f/--force                - ignore already added port GUIDs\n"
	"    vfname                    - virtual fabric name\n"
	"    portguid...               - list of port GUIDs (ex. 0x00117501a0000380)\n"
	"\n"
	"Add ports to given virtual fabric configuration.\n"
	"\n"
	"Return:\n"
	"    0 - if all ports have been properly added\n"
	"    1 - if some ports are already added and -f/--force was not provided\n"
	"    2 - invalid arguments were provided or other error occurred\n";

const char* cmd_remove_help =
	"Usage:\n"
	"    opafmvf remove [-f] vfname portguid...\n"
	"    opafmvf remove vfname --all\n"
	"    opafmvf remove --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"    -f/--force                - ignore already removed port GUIDs\n"
	"    --all                     - remove all port GUIDs\n"
	"    vfname                    - virtual fabric name\n"
	"    portguid...               - list of port GUIDs (ex. 0x00117501a0000380)\n"
	"\n"
	"Remove ports from given virtual fabric configuration.\n"
	"\n"
	"Return:\n"
	"    0 - all ports have been properly removed\n"
	"    1 - some ports are already removed and -f/--force was not provided\n"
	"    2 - invalid arguments were provided or other error occurred\n";

const char* cmd_create_usage =
	"Usage:\n"
	"    opafmvf create [-f] --pkey=value vfname\n"
	"    opafmvf create --help\n";

const char* cmd_delete_usage =
	"Usage:\n"
	"    opafmvf delete [-f] vfname\n"
	"    opafmvf delete --help\n";

const char* cmd_add_usage =
	"Usage:\n"
	"    opafmvf add [-f] vfname portguid...\n"
	"    opafmvf add --help\n";

const char* cmd_remove_usage =
	"Usage:\n"
	"    opafmvf remove [-f] vfname portguid...\n"
	"    opafmvf remove vfname --all\n"
	"    opafmvf remove --help\n";


CmdStatus
cmd_create(int argc, char* argv[])
{
	bool force = false;
	uint16_t pkey = 0xffff;
	const char*	vf_name = NULL;

	/* parse options */
	int long_index = -1;
	static struct option long_options[] = {
		{"pkey",  required_argument, 0, 'k'},
		{"force", no_argument,       0, 'f'},
		{"help",  no_argument,       0, 'h'},
		{0, 0, 0, 0}
	};

	optind = 0;
	while (true) {
		int c = getopt_long(argc, argv, "fk:", long_options, &long_index);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(cmd_create_help);
			return CMD_STATUS_OK;
		case 'k':
			if (!string_to_pkey(&pkey, optarg)) {
				log_error("%s: invalid pkey value '%s'", argv[0], optarg);
				print_usage(cmd_create_usage);
				return CMD_STATUS_INVARG;
			}
			break;
		case 'f':
			force = true;
			break;
		default:
			print_usage(cmd_create_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* since above string_to_pkey does not accept 0xffff value */
	/* we can use it and check whether pkey option was provided */
	if (pkey == 0xffff) {
		log_error("%s: missing option '--pkey'", argv[0]);
		print_usage(cmd_create_usage);
		return CMD_STATUS_INVARG;
	}

	/* get virtual fabric name */
	if (!extract_vf_name(argc, argv, optind++, &vf_name)) {
		print_usage(cmd_create_usage);
		return CMD_STATUS_INVARG;
	}

	/* check not parsed arguments */
	if (optind < argc) {
		log_error("%s: too many arguments", argv[0]);
		print_usage(cmd_create_usage);
		return CMD_STATUS_INVARG;
	}

	/* create virtual fabric configuration */
	CmdStatus status = CMD_STATUS_OK;

	Tenant* tenant = tenant_new(vf_name);
	if (!tenant) {
		log_error("Cannot initiate virtual fabric configuration '%s'", vf_name);
		return CMD_STATUS_ERROR;
	}

	/* before proceed ensure only one instance is running */
	int lock = program_lock_acquire();
	if (lock == -1) {
		log_error("Another instance of opafmvf is running");
		status = CMD_STATUS_ANOTHER_INSTANCE_RUNNING;
		goto error;
	}

	log_debug("Creating virtual fabric configuration '%s'", vf_name);

	/* check whether tenant exist, but only if force is not set */
	if (!force && tenant_exist(tenant)) {
		log_error("Virtual fabric configuration '%s' already exists", vf_name);
		status = CMD_STATUS_CANNOT_PERFORM;
		goto error;
	}

	/* fill tenant fields with default values */
	tenant_set_default(tenant);

	/* set new pkey */
	if (!tenant_set_pkey(tenant, pkey)) {
		log_error("Cannot set pkey %x for virtual fabric configuration '%s'", pkey, vf_name);
		status = CMD_STATUS_ERROR;
		goto error;
	}

	/* save tenant on the disk */
	if (!tenant_save(tenant, force)) {
		log_error("Saving virtual fabric configuration '%s' failed", vf_name);
		status = CMD_STATUS_ERROR;
		goto error;
	}

	log_info("Created virtual fabric configuration '%s'", vf_name);

error:
	program_lock_release(lock);
	tenant_free(tenant);

	return status;
}

CmdStatus
cmd_delete(int argc, char* argv[])
{
	bool force = false;
	const char*	vf_name = NULL;

	/* parse options */
	static struct option long_options[] = {
		{"force", no_argument, 0, 'f'},
		{"help",  no_argument, 0, 'h'},
		{0, 0, 0, 0}
	};

	optind = 0;
	while (true) {
		int c = getopt_long(argc, argv, "f", long_options, NULL);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(cmd_delete_help);
			return CMD_STATUS_OK;
		case 'f':
			force = true;
			break;
		default:
			print_usage(cmd_delete_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* get virtual fabric name */
	if (!extract_vf_name(argc, argv, optind++, &vf_name)) {
		print_usage(cmd_delete_usage);
		return CMD_STATUS_INVARG;
	}

	/* check not parsed arguments */
	if (optind < argc) {
		log_error("%s: too many arguments", argv[0]);
		print_usage(cmd_delete_usage);
		return CMD_STATUS_INVARG;
	}

	/* delete virtual fabric configuration */
	CmdStatus status = CMD_STATUS_OK;

	Tenant* tenant = tenant_new(vf_name);
	if (!tenant) {
		log_error("Cannot initiate virtual fabric configuration '%s'", vf_name);
		return CMD_STATUS_ERROR;
	}

	/* before proceed ensure only one instance is running */
	int lock = program_lock_acquire();
	if (lock == -1) {
		log_error("Another instance of opafmvf is running");
		status = CMD_STATUS_ANOTHER_INSTANCE_RUNNING;
		goto error;
	}

	log_debug("Deleting virtual fabric configuration '%s'", vf_name);

	/* if force is not set then check whether tenant exists and is empty */
	if (!force) {
		if (!tenant_exist(tenant)) {
			log_error("Virtual fabric configuration '%s' does not exist", vf_name);
			status = CMD_STATUS_CANNOT_PERFORM;
			goto error;
		}

		if (!tenant_load(tenant)) {
			log_error("Cannot load virtual fabric configuration '%s'", vf_name);
			status = CMD_STATUS_CANNOT_PERFORM;
			goto error;
		}

		if (!tenant_is_valid(tenant)) {
			log_error("Loaded virtual fabric configuration '%s' is corrupted", vf_name);
			status = CMD_STATUS_CANNOT_PERFORM;
			goto error;
		}

		if (tenant->dg_config.port_guid_map.count > 0) {
			log_error("Virtual fabric configuration '%s' contains ports", vf_name);
			status = CMD_STATUS_CANNOT_PERFORM;
			goto error;
		}
	}

	if (!tenant_delete(tenant)) {
		log_error("Cannot delete virtual fabric configuration '%s'", vf_name);
		status = CMD_STATUS_ERROR;
		goto error;
	}

	log_info("Deleted virtual fabric configuration '%s'", vf_name);

error:
	program_lock_release(lock);
	tenant_free(tenant);

	return status;
}

CmdStatus
cmd_add(int argc, char* argv[])
{
	bool force = false;
	const char*	vf_name = NULL;
	uint64_t*	guids = NULL;
	size_t		guids_count = 0;

	/* parse options */
	static struct option long_options[] = {
		{"force", no_argument, 0, 'f'},
		{"help",  no_argument, 0, 'h'},
		{0, 0, 0, 0}
	};

	optind = 0;
	while (true) {
		int c = getopt_long(argc, argv, "f", long_options, NULL);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(cmd_add_help);
			return CMD_STATUS_OK;
		case 'f':
			force = true;
			break;
		default:
			print_usage(cmd_add_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* get virtual fabric name */
	if (!extract_vf_name(argc, argv, optind++, &vf_name)) {
		print_usage(cmd_add_usage);
		return CMD_STATUS_INVARG;
	}

	/* get port guids */
	if (!extract_port_guids(argc, argv, optind, &guids, &guids_count)) {
		print_usage(cmd_add_usage);
		return CMD_STATUS_INVARG;
	}

	/* add port guids to virtual fabric configuration */
	CmdStatus status = CMD_STATUS_OK;

	Tenant* tenant = tenant_new(vf_name);
	if (!tenant) {
		log_error("Cannot initiate virtual fabric configuration '%s'", vf_name);
		free(guids);
		return CMD_STATUS_ERROR;
	}

	/* before proceed ensure only one instance is running */
	int lock = program_lock_acquire();
	if (lock == -1) {
		log_error("Another instance of opafmvf is running");
		status = CMD_STATUS_ANOTHER_INSTANCE_RUNNING;
		goto error;
	}

	if (!tenant_load(tenant)) {
		log_error("Cannot load virtual fabric configuration '%s'", vf_name);
		status = CMD_STATUS_ERROR;
		goto error;
	}

	if (!tenant_is_valid(tenant)) {
		log_error("Loaded virtual fabric configuration '%s' is corrupted", vf_name);
		status = CMD_STATUS_ERROR;
		goto error;
	}

	for (size_t i = 0; i < guids_count; ++i) {
		log_debug("Adding port 0x%016lx to virtual fabric configuration '%s'", guids[i], vf_name);

		if (tenant_add_port_guid(tenant, guids[i]))
			continue;

		if (force)
			continue;

		log_error("Virtual fabric configuration '%s' already contains port 0x%016lx", vf_name, guids[i]);
		status = CMD_STATUS_CANNOT_PERFORM;
		goto error;
	}

	if (!tenant_save(tenant, true)) {
		log_error("Saving virtual fabric configuration '%s' failed", vf_name);
		status = CMD_STATUS_ERROR;
		goto error;
	}

	log_info("Added ports to virtual fabric configuration '%s'", vf_name);

error:
	program_lock_release(lock);
	tenant_free(tenant);
	free(guids);

	return status;
}

CmdStatus
cmd_remove(int argc, char* argv[])
{
	bool all = false;
	bool force = false;
	const char*	vf_name = NULL;
	uint64_t*	guids = NULL;
	size_t		guids_count = 0;

	/* parse options */
	static struct option long_options[] = {
		{"all",   no_argument, 0, 'a'},
		{"force", no_argument, 0, 'f'},
		{"help",  no_argument, 0, 'h'},
		{0, 0, 0, 0}
	};

	optind = 0;
	while (true) {
		int c = getopt_long(argc, argv, "af", long_options, NULL);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(cmd_remove_help);
			return CMD_STATUS_OK;
		case 'a':
			all = true;
			break;
		case 'f':
			force = true;
			break;
		default:
			print_usage(cmd_remove_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* get virtual fabric name */
	if (!extract_vf_name(argc, argv, optind++, &vf_name)) {
		print_usage(cmd_remove_usage);
		return CMD_STATUS_INVARG;
	}

	/* get port guids */
	if (all) {
		/* check not parsed arguments */
		if (optind < argc) {
			log_error("%s: too many arguments", argv[0]);
			print_usage(cmd_remove_usage);
			return CMD_STATUS_INVARG;
		}
	}
	else {
		if (!extract_port_guids(argc, argv, optind, &guids, &guids_count)) {
			print_usage(cmd_remove_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* remove port guids from virtual fabric configuration */
	CmdStatus status = CMD_STATUS_OK;

	Tenant* tenant = tenant_new(vf_name);
	if (!tenant) {
		log_error("Cannot initiate virtual fabric configuration '%s'", vf_name);
		free(guids);
		return CMD_STATUS_ERROR;
	}

	/* before proceed ensure only one instance is running */
	int lock = program_lock_acquire();
	if (lock == -1) {
		log_error("Another instance of opafmvf is running");
		status = CMD_STATUS_ANOTHER_INSTANCE_RUNNING;
		goto error;
	}

	if (!tenant_load(tenant)) {
		log_error("Cannot load virtual fabric configuration '%s'", vf_name);
		status = CMD_STATUS_ERROR;
		goto error;
	}

	if (!tenant_is_valid(tenant)) {
		log_error("Loaded virtual fabric configuration '%s' is corrupted", vf_name);
		status = CMD_STATUS_ERROR;
		goto error;
	}

	if (all) {
		log_debug("Removing all ports from virtual fabric configuration '%s'", vf_name);
		tenant_remove_all_port_guids(tenant);
		goto complete;
	}

	for (size_t i = 0; i < guids_count; ++i) {
		log_debug("Removing port 0x%016lx from virtual fabric configuration '%s'", guids[i], vf_name);

		if (tenant_remove_port_guid(tenant, guids[i]))
			continue;

		if (force)
			continue;

		log_error("Virtual fabric configuration '%s' does not contain port 0x%016lx", vf_name, guids[i]);
		status = CMD_STATUS_CANNOT_PERFORM;
		goto error;
	}

complete:
	if (!tenant_save(tenant, true)) {
		log_error("Saving virtual fabric configuration '%s' failed", vf_name);
		status = CMD_STATUS_ERROR;
		goto error;
	}

	log_info("Removed ports from virtual fabric configuration '%s'", vf_name);

error:
	program_lock_release(lock);
	tenant_free(tenant);
	free(guids);

	return status;
}
