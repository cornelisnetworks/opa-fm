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

#include "cmd_mgmt.h"

#include "utils.h"
#include <dirent.h>
#include <errno.h>
#include <getopt.h>
#include <iba/public/imemory.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>


extern const char* default_dg_dir;
extern const char* default_vf_dir;

const char* cmd_reset_help =
	"Usage:\n"
	"    opafmvf reset [-f]\n"
	"    opafmvf reset --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"    -f/--force                - do not prompt\n"
	"\n"
	"Delete all auxiliary opafmvf configuration files from /etc/opa-fm/vfs\n"
	"and /etc/opa-fm/dgs directories.\n"
	"\n"
	"Return:\n"
	"    0 - all configuration files have been deleted\n"
	"    1 - operation was canceled\n"
	"    2 - invalid arguments were provided or other error occurred\n";

const char* cmd_commit_help =
	"Usage:\n"
	"    opafmvf commit [-p path] [-o path] [-b] [-f]\n"
	"    opafmvf commit --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"    -p/--ppfile path          - file to preprocess\n"
	"                                default: /etc/opa-fm/opafm_pp.xml\n"
	"    -o/--output path          - output XML file\n"
	"                                default: /etc/opa-fm/opafm.xml\n"
	"    -b/--backup               - backup old output XML file if exists\n"
	"    -f/--force                - do not prompt\n"
	"\n"
	"Generate new configuration for OPA FM basing on the preprocess file and\n"
	"configuration files from /etc/opa-fm/vfs and /etc/opa-fm/dgs directories.\n"
	"\n"
	"Return:\n"
	"    0 - a new configuration for OPA FM has been generated properly\n"
	"    1 - operation was canceled or error occurred during generation\n"
	"        new configuration for OPA FM\n"
	"    2 - invalid arguments were provided or other error occurred\n";

const char* cmd_reload_help =
	"Usage:\n"
	"    opafmvf reload\n"
	"    opafmvf reload --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"\n"
	"Inform OPA FM to read new configuration and propagate it over the fabric.\n"
	"\n"
	"Return:\n"
	"    0 - OPA FM reloaded configuration properly\n"
	"    1 - OPA FM could not reload configuration (ex. OPA FM is not running)\n"
	"    2 - invalid arguments were provided or other error occurred\n";

const char* cmd_restart_help =
	"Usage:\n"
	"    opafmvf restart\n"
	"    opafmvf restart --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"\n"
	"Stop and then start OPA FM. If OPA FM is not running yet, it will be started.\n"
	"\n"
	"Return:\n"
	"    0 - OPA FM has been restarted properly\n"
	"    1 - some error occurred during OPA FM restart\n"
	"    2 - invalid arguments were provided or other error occurred\n";

const char* cmd_reset_usage =
	"Usage:\n"
	"    opafmvf reset [-f]\n"
	"    opafmvf reset --help\n";

const char* cmd_commit_usage =
	"Usage:\n"
	"    opafmvf commit [-p path] [-o path] [-b] [-f]\n"
	"    opafmvf commit --help\n";

const char* cmd_reload_usage =
	"Usage:\n"
	"    opafmvf reload\n"
	"    opafmvf reload --help\n";

const char* cmd_restart_usage =
	"Usage:\n"
	"    opafmvf restart\n"
	"    opafmvf restart --help\n";


const char* default_opafm_config = "/etc/opa-fm/opafm.xml";


static int
exec_cmd(const char* cmd)
{
	int ret = system(cmd);

	if (!WIFEXITED(ret)) {
		if (WIFSIGNALED(ret))
			log_error("Command '%s' terminated with the signal %d", cmd, WTERMSIG(ret));
		else
			log_error("Command '%s' terminated abnormally", cmd);
		return -1;
	}

	return WEXITSTATUS(ret);
}

static bool
prompt_user(void)
{
	int c = getchar();

	bool yes = (c == 'y' || c == 'Y');
	while (c != '\n' && c != EOF)
		c = getchar();
	return yes;
}

static bool
remove_files_by_ext(const char* dir_name, const char* ext)
{
	DIR *dir = opendir(dir_name);
	if (!dir) {
		log_error("Cannot open dir %s", dir_name);
		return false;
	}

	bool result = true;
	struct dirent *entry;
	while((entry = readdir(dir)) != NULL) {
		char *file_ext = strrchr(entry->d_name, '.');
		if (entry->d_type == DT_REG && file_ext && !strcmp(ext, file_ext + 1)) {
			log_debug("Deleting: %s/%s", dir_name, entry->d_name);
			if (unlinkat(dirfd(dir), entry->d_name, 0)) {
				log_error("Cannot remove file %s/%s", dir_name, entry->d_name);
				result = false;
				break;
			}
		}
	}
	closedir(dir);
	return result;

}

CmdStatus
cmd_reset(int argc, char* argv[])
{
	bool force = false;

	/* parse options */
	static struct option long_options[] = {
		{"help",  no_argument, 0, 'h'},
		{"force", no_argument, 0, 'f'},
		{0, 0, 0, 0}
	};

	optind = 0;
	while (true) {
		int c = getopt_long(argc, argv, "f", long_options, NULL);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(cmd_reset_help);
			return CMD_STATUS_OK;
		case 'f':
			force = true;
			break;
		default:
			print_usage(cmd_reset_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* check not parsed arguments */
	if (optind < argc) {
		log_error("%s: too many arguments", argv[0]);
		print_usage(cmd_reset_usage);
		return CMD_STATUS_INVARG;
	}

	/* remove all created virtual fabrics files */
	CmdStatus status = CMD_STATUS_OK;

	/* before proceed ensure only one instance is running */
	int lock = program_lock_acquire();
	if (lock == -1) {
		log_error("Another instance of opafmvf is running");
		status = CMD_STATUS_ANOTHER_INSTANCE_RUNNING;
		goto error;
	}

	/* prompt user if force flag is not set */
	if (!force) {
		fprintf(stdout, "All virtual fabrics configurations will be deleted!\n");
		fprintf(stdout, "Do you want to continue? [y/N] ");
		fflush(stdout);
		if (!prompt_user()) {
			status = CMD_STATUS_CANNOT_PERFORM;
			goto error;
		}
	}

	bool result = true;
	result = result && remove_files_by_ext(default_vf_dir, "xml");
	result = result && remove_files_by_ext(default_dg_dir, "xml");

	if (!result) {
		status = CMD_STATUS_ERROR;
		goto error;
	}

	log_info("Deleted all virtual fabrics configurations");

error:
	program_lock_release(lock);

	return status;
}

CmdStatus
cmd_commit(int argc, char* argv[])
{
	const char* ppfile = NULL;
	const char* output = default_opafm_config;
	bool backup = false;
	bool force = false;

	/* parse options */
	static struct option long_options[] = {
		{"ppfile", required_argument, 0, 'p'},
		{"output", required_argument, 0, 'o'},
		{"backup", no_argument,       0, 'b'},
		{"force",  no_argument,       0, 'f'},
		{"help",   no_argument,       0, 'h'},
		{0, 0, 0, 0}
	};

	optind = 0;
	while (true) {
		int c = getopt_long(argc, argv, "p:o:bf", long_options, NULL);
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			print_help(cmd_commit_help);
			return CMD_STATUS_OK;
		case 'p':
			ppfile = optarg;
			break;
		case 'o':
			output = optarg;
			break;
		case 'b':
			backup = true;
			break;
		case 'f':
			force = true;
			break;
		default:
			print_usage(cmd_commit_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* check not parsed arguments */
	if (optind < argc) {
		log_error("%s: too many arguments", argv[0]);
		print_usage(cmd_commit_usage);
		return CMD_STATUS_INVARG;
	}

	/* generate a new configuration file */
	CmdStatus status = CMD_STATUS_OK;

	/* before proceed ensure only one instance is running */
	int lock = program_lock_acquire();
	if (lock == -1) {
		log_error("Another instance of opafmvf is running");
		status = CMD_STATUS_ANOTHER_INSTANCE_RUNNING;
		goto error;
	}

	/* check output file exists, if so prompt user */
	if (!force) {
		if (access(output, F_OK) == 0) {
			fprintf(stdout, "%s will be overwritten!\n", output);
			fprintf(stdout, "Do you want to continue? [y/N] ");
			fflush(stdout);
			if (!prompt_user()) {
				status = CMD_STATUS_CANNOT_PERFORM;
				goto error;
			}
		}
	}

	/* construct command line for opafmconfigpp */
	const char* main_cmd = "opafmconfigpp -f";
	const char* opt_1 = (ppfile) ? " -p" : "";
	const char* opt_1_val = (ppfile) ? ppfile : "";
	const char* opt_2 = (output) ? " -o" : "";
	const char* opt_2_val = (output) ? output : "";
	const char* opt_3 = (backup) ? " -b" : "";

	char* cmd = StringConcat(main_cmd, opt_1, opt_1_val, opt_2, opt_2_val, opt_3, NULL);
	if (!cmd)
		die("Out of memory");

	/* execute opafmconfigpp */
	int cmd_status = exec_cmd(cmd);

	free(cmd);

	if (cmd_status) {
		log_fatal("Could not generate a new configuration for fabric manager");
		status = CMD_STATUS_CANNOT_PERFORM;
		goto error;
	}

	log_info("Generated new configuration for fabric manager");

error:
	program_lock_release(lock);

	return status;
}

CmdStatus
cmd_reload(int argc, char* argv[])
{
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
			print_help(cmd_reload_help);
			return CMD_STATUS_OK;
		default:
			print_usage(cmd_reload_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* check not parsed arguments */
	if (optind < argc) {
		log_error("%s: too many arguments", argv[0]);
		print_usage(cmd_reload_usage);
		return CMD_STATUS_INVARG;
	}

	/* this command can be executed from many processes in the same time */
	/* therefore it is not needed to acquire a program lock */

	/* reload fabric manager */
	if (exec_cmd("systemctl reload opafm")) {
		log_fatal("Could not reload fabric manager");
		return CMD_STATUS_CANNOT_PERFORM;
	}

	log_info("Reloaded fabric manager");

	return CMD_STATUS_OK;
}

CmdStatus
cmd_restart(int argc, char* argv[])
{
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
			print_help(cmd_restart_help);
			return CMD_STATUS_OK;
		default:
			print_usage(cmd_restart_usage);
			return CMD_STATUS_INVARG;
		}
	}

	/* check not parsed arguments */
	if (optind < argc) {
		log_error("%s: too many arguments", argv[0]);
		print_usage(cmd_restart_usage);
		return CMD_STATUS_INVARG;
	}

	/* this command can be executed from many processes in the same time */
	/* therefore it is not needed to acquire a program lock */

	/* restart fabric manager */
	if (exec_cmd("systemctl restart opafm")) {
		log_fatal("Could not restart fabric manager");
		return CMD_STATUS_CANNOT_PERFORM;
	}

	log_info("Restarted fabric manager");

	return CMD_STATUS_OK;
}
