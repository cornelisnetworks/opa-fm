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
#include "cmd_mgmt.h"
#include "cmd_query.h"
#include "utils.h"
#include <getopt.h>
#include <string.h>


const char* cmd_main_help =
	"Usage:\n"
	"    opafmvf [-v] command [arg...]\n"
	"    opafmvf --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"    -v/--verbose              - verbose output\n"
	"\n"
	"Commands:\n"
	"    create                    - create empty virtual fabric configuration\n"
	"    delete                    - delete virtual fabric configuration\n"
	"    add                       - add ports to virtual fabric configuration\n"
	"    remove                    - remove ports from virtual fabric configuration\n"
	"    reset                     - delete all auxiliary opafmvf configuration files\n"
	"    commit                    - generate new configuration for OPA FM\n"
	"    reload                    - inform OPA FM to read new configuration\n"
	"    restart                   - stop and then start OPA FM\n"
	"    exist                     - query if virtual fabric exists in the fabric\n"
	"    ismember                  - query if ports are members of virtual fabric\n"
	"    isnotmember               - query if ports are not members of virtual fabric\n";

const char* cmd_main_usage =
	"Usage:\n"
	"    opafmvf [-v] command [arg...]\n"
	"    opafmvf --help\n";


typedef struct _Command {
	const char* name;
	CmdStatus (*func)(int argc, char* argv[]);
} Command;

static Command commands[] = {
	{ "create", cmd_create },
	{ "delete", cmd_delete },
	{ "add", cmd_add },
	{ "remove", cmd_remove },
	{ "reset", cmd_reset },
	{ "commit", cmd_commit },
	{ "reload", cmd_reload },
	{ "restart", cmd_restart },
	{ "exist", cmd_exist },
	{ "ismember", cmd_ismember },
	{ "isnotmember", cmd_isnotmember },
};

static Command*
get_command(const char* name)
{
	for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
		Command* command = &commands[i];
		if (!strcmp(name, command->name))
			return command;
	}

	return NULL;
}

int main(int argc, char* argv[])
{
	/* parse options */
	static struct option long_options[] = {
		{"help",    no_argument, 0, 'h'},
		{"verbose", no_argument, 0, 'v'},
		{0, 0, 0, 0}
	};

	while (true) {
		int c = getopt_long(argc, argv, "+v", long_options, NULL);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(cmd_main_help);
			return CMD_STATUS_OK;
		case 'v':
			log_verbose = true;
			break;
		default:
			print_usage(cmd_main_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* get command name */
	if (optind >= argc) {
		log_error("%s: missing argument '<command>'", argv[0]);
		print_usage(cmd_main_usage);
		return CMD_STATUS_INVARG;
	}

	Command* command = get_command(argv[optind]);
	if (!command) {
		log_error("%s: unrecognized command '%s'", argv[0], argv[optind]);
		print_usage(cmd_main_usage);
		return CMD_STATUS_INVARG;
	}

	/* execute function assigned to the command */
	if (command->func)
		return command->func(argc - optind, &argv[optind]);

	return CMD_STATUS_OK;
}
