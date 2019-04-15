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
#include <stdio.h>
#include <regex.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <limits.h>

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
	"    opafmvf commit [-p path] [-o path] [-b] [-f] [-s]\n"
	"    opafmvf commit --help\n"
	"\n"
	"    --help                    - produce full help text\n"
	"    -p/--ppfile path          - file to preprocess\n"
	"                                default: /etc/opa-fm/opafm_pp.xml\n"
	"    -o/--output path          - output XML file\n"
	"                                default: /etc/opa-fm/opafm.xml\n"
	"    -b/--backup               - backup old output XML file if exists\n"
	"    -f/--force                - do not prompt\n"
	"    -s/--skip-check           - Skip validation check on output. Speeds up command considerably\n"
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
const char* default_ppfile = "/etc/opa-fm/opafm_pp.xml";


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


static int dump_vfdg_files(FILE *out_file, char *dir_name)
{
	struct dirent *dp;
	DIR *dir = opendir(dir_name);
	if(dir == NULL){
		log_error("Failed to open input directory %s: %s\n", dir_name, strerror(errno));
		return 1;
	}

	while ((dp=readdir(dir)))
	{
		char* file_name = StringConcat(dir_name, "/",dp->d_name, NULL);
		if (!file_name)
			die("Out of memory");

		if (dp->d_type == DT_REG) {
			FILE *in_file = fopen(file_name, "r");
			if(!in_file){
				log_error("Failed to open input file %s: %s\n", file_name, strerror(errno));
				free(file_name);
				closedir(dir);
				return 1;
			}

			char *line = NULL;
			size_t len= 0;
			ssize_t n_read = 0;
			while((n_read = getline(&line, &len, in_file)) != -1) {
				fwrite(line, n_read, 1, out_file);
			}
			fclose(in_file);
		}
		free(file_name);
	}
	closedir(dir);
	return 0;
}


static int commit_file(FILE *in_file, FILE *out_file)
{
	char *line = NULL;
	size_t len = 0;
	ssize_t n_read = 0;

	/*keep first line at top*/
	n_read = getline(&line, &len, in_file);
	fwrite(line, n_read, 1, out_file);

	/*write metadata to file*/
	fprintf(out_file,"<!-- Generated file. Do not edit. -->\n");
	time_t epoch_time;
	char date[26];
	struct tm* tm_info;
	time(&epoch_time);
	tm_info = localtime(&epoch_time);
	if(tm_info) {
		strftime(date, 26, "%Y-%m-%d %H:%M:%S", tm_info);
		fprintf(out_file,"<!-- Generated Date: %s -->\n", date);
	}

	/* Process input file and insert VF/DG includes */
	char *pattern = "^[[:space:]]*<[!]--[[:space:]]*INCLUDE:.._DIR=\\(.*\\)[[:space:]]*-->.*";
	regex_t regex;
	regmatch_t regmatch[2];
	regcomp(&regex, pattern, 0);

	while((n_read = getline(&line, &len, in_file)) != -1) {

		if(regexec(&regex, line, 2, regmatch, 0) == 0){
			char dir_name[PATH_MAX];
			if(regmatch[1].rm_eo - regmatch[1].rm_so >= PATH_MAX){
				log_error("Path name is too long\n");
				free(line);
				regfree(&regex);
				return 1;
			}

			StringCopy(dir_name, &line[regmatch[1].rm_so], regmatch[1].rm_eo - regmatch[1].rm_so);
			printf("Processing files in  %s\n", dir_name);
			if(dump_vfdg_files(out_file, dir_name)){
				regfree(&regex);
				free(line);
				return 1;
			}
		} else {
			fwrite(line, n_read, 1, out_file);
		}
	}

	regfree(&regex);
	free(line);
	return 0;
}

static int commit_check(char *output_temp, const char *output)
{
	size_t len = 0;
	ssize_t n_read = 0;
	char *line = NULL;
	char* cmd = StringConcat("/usr/lib/opa-fm/bin/config_check -c ", output_temp, " -s 2>&1",NULL);
	if (!cmd)
		die("Out of memory");
	fflush(stdout);
	FILE *config_check_pipe = popen(cmd, "r");
	free(cmd);
	if(!config_check_pipe){
		log_error("Failed to open pipe to Config Check utility\n");
		return 1;
	}
	char *output_check = StringConcat(output,".check",NULL);
	if (!output_check)
		die("Out of memory");
	FILE *config_check_out_file = fopen(output_check,"w");
	if(!config_check_out_file){
		log_error("Failed to open output temp file %s: %s\n", output_check, strerror(errno));
		free(output_check);
		pclose(config_check_pipe);
		return 1;
	}
	free(output_check);
	while((n_read = getline(&line, &len, config_check_pipe)) != -1) {
		fwrite(line, n_read, 1, config_check_out_file);
		fprintf(stderr, "%s",line);
	}
	free(line);
	fclose(config_check_out_file);
	int cmd_status = pclose(config_check_pipe);
	if(cmd_status){
		log_error("Error: Config Check Failed: Review %s.check\n", output);
		return 1;
	}
	log_info("Config Check Passed!\n");

	return 0;
}


CmdStatus
cmd_commit(int argc, char* argv[])
{
	const char* ppfile = default_ppfile;
	const char* output = default_opafm_config;
	bool backup = false;
	bool force = false;
	bool skip_check = false;

	/* parse options */
	static struct option long_options[] = {
		{"ppfile", required_argument, 0, 'p'},
		{"output", required_argument, 0, 'o'},
		{"backup", no_argument,       0, 'b'},
		{"force",  no_argument,       0, 'f'},
		{"skip-check",   no_argument, 0, 's'},
		{"help",   no_argument,       0, 'h'},
		{0, 0, 0, 0}
	};

	optind = 0;
	while (true) {
		int c = getopt_long(argc, argv, "p:o:bfs", long_options, NULL);
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
		case 's':
			skip_check = true;
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

	/* check if output file exists, if so prompt user */
	if (access(output, F_OK) == 0) {
		if(!force) {
			fprintf(stdout, "%s will be overwritten!\n", output);
			fprintf(stdout, "Do you want to continue? [y/N] ");
			fflush(stdout);
			fflush(stderr);
			if (!prompt_user()) {
				status = CMD_STATUS_CANNOT_PERFORM;
				goto error;
			}
		}
	}

	FILE *in_file = fopen(ppfile, "r");
	if(!in_file){
		log_error("Failed to open input file %s: %s\n", ppfile, strerror(errno));
		status = CMD_STATUS_CANNOT_PERFORM;
		goto error;
	}
	char *output_temp = StringConcat(output,".tmp",NULL);
	if (!output_temp)
		die("Out of memory");
	FILE *out_file = fopen(output_temp, "w");
	if(!out_file){
		log_error("Failed to open output file %s: %s\n", output, strerror(errno));
		fclose(in_file);
		status = CMD_STATUS_CANNOT_PERFORM;
		free(output_temp);
		goto error;
	}

	if(commit_file(in_file, out_file)){
		fclose(in_file);
		fclose(out_file);
		status = CMD_STATUS_CANNOT_PERFORM;
		free(output_temp);
		goto error;
	}

	fclose(in_file);
	fclose(out_file);

	if(!skip_check){
		if(commit_check(output_temp, output)){
			status = CMD_STATUS_CANNOT_PERFORM;
			free(output_temp);
			goto error;
		}
	}


	/* backup old config file if it exists */
	if (access(output, F_OK) == 0 && backup) {
		char* backup_name = StringConcat(output, ".bak", NULL);
		if (!backup_name)
			die("out of memory");
		if(rename(output, backup_name) != 0){
			free(output_temp);
			free(backup_name);
			log_error("Failed to backup file: %s\n", strerror(errno));
			status = CMD_STATUS_CANNOT_PERFORM;
			goto error;
		}
		free(backup_name);
		log_info("Generated backup file %s.bak", output);
	}


	/* Make temp file permanent */
	if(rename(output_temp, output) != 0){
		free(output_temp);
		log_error("Failed to rename temp file: %s\n", strerror(errno));
		status = CMD_STATUS_CANNOT_PERFORM;
		goto error;
	}

	free(output_temp);
	log_info("Generated new configuration for fabric manager");

error:

	if(status != CMD_STATUS_OK)
		log_fatal("Could not generate a new configuration for fabric manager");
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
