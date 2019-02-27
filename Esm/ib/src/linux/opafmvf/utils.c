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

#include "utils.h"

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <iba/public/imemory.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>


bool log_verbose = false;


void
die(const char* format, ...)
{
	char msg[1024];
	va_list args;
	va_start(args, format);
	vsnprintf(msg, sizeof(msg), format, args);
	va_end(args);

	fprintf(stderr, "\033[31m[FATAL]\033[0m %s\n", msg);

	/* make sure we get an entry to syslog, */
	/* just in case logging not fully initialized */
	openlog("FATAL:", (LOG_NDELAY | LOG_PID), LOG_USER);
	syslog(LOG_CRIT, "%s", msg);
	closelog();

	exit(128);
}

int
program_lock_acquire(void) {
	int sock = socket(AF_UNIX, SOCK_STREAM, 0);
	if (sock == -1) {
		log_error("Cannot create a socket (%s)", strerror(errno));
		return -1;
	}

	struct sockaddr_un sun;
	memset(&sun, 0, sizeof(sun));
	sun.sun_family = AF_UNIX;
	StringCopy(sun.sun_path + 1, "opafmvf.lock", sizeof(sun.sun_path) - 1);
	if (bind(sock, (struct sockaddr *)&sun, sizeof(sun))) {
		close(sock);
		return -1;
	}

	return sock;
}

void
program_lock_release(int lock)
{
	if (lock != -1)
		close(lock);
}

bool
file_save_atomic(const char* dir, const char* file_name, char* ptr, size_t size, bool force)
{
	bool result = false;
	char* path_tmp = NULL;

	char* path = StringConcat(dir, "/", file_name, NULL);
	if (!path)
		die("Out of memory");

	/* do nothing if file exists and force is not set */
	bool exist = (access(path, F_OK) == 0);
	if (exist && !force) {
		errno = EEXIST;
		goto error_open;
	}

#ifdef O_TMPFILE
	/* create a hidden temporary file, not visible by other programs */
	int fd = open(dir, O_TMPFILE | O_WRONLY | O_SYNC, S_IRUSR | S_IWUSR);
	if (fd == -1)
		goto error_open;
#else
	/* create a temporary file, not displayed in directory listings by default */
	path_tmp = StringConcat(dir, "/.tmp_", file_name, NULL);
	if (!path_tmp)
		die("Out of memory");

	int fd = open(path_tmp, O_CREAT | O_EXCL | O_WRONLY | O_SYNC, S_IRUSR | S_IWUSR);
	if (fd == -1)
		goto error_open;
#endif

	/* write all data to this file */
	while (size) {
		ssize_t bytes = write(fd, ptr, size);
		if (bytes == -1)
			goto error_write;
		size -= bytes;
		ptr += bytes;
	}

	/* flush all data to the disk device */
	if (fsync(fd) == -1)
		goto error_write;

#ifdef O_TMPFILE
	if (exist)
		unlink(path);

	/* create a new hard link to this file associated with a given file */
	char path_proc[64];
	snprintf(path_proc, sizeof(path_proc),  "/proc/self/fd/%d", fd);
	if (linkat(AT_FDCWD, path_proc, AT_FDCWD, path, AT_SYMLINK_FOLLOW) == -1)
		goto error_write;
#else
	/* rename a temporary file to its target name */
	if (rename(path_tmp, path))
		goto error_write;
#endif

	result = true;

error_write:
	close(fd);

error_open:
	free(path);
	free(path_tmp);

	return result;
}
