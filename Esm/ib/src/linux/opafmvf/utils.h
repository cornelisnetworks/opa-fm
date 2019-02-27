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

#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

extern bool log_verbose;

#define log_fatal(fmt, ...) do { fprintf(stderr, fmt "\n", ##__VA_ARGS__); syslog(LOG_MAKEPRI(LOG_USER, LOG_CRIT), fmt, ##__VA_ARGS__); } while (0)
#define log_error(fmt, ...) do { fprintf(stderr, fmt "\n", ##__VA_ARGS__); } while (0)
#define log_info(fmt, ...) do { fprintf(stdout, fmt "\n", ##__VA_ARGS__); } while (0)
#define log_debug(fmt, ...) do { if (log_verbose) fprintf(stdout, fmt "\n", ##__VA_ARGS__); } while (0)

void die(const char* message, ...) __attribute__ ((noreturn, format(printf, 1, 2)));

int program_lock_acquire(void);
void program_lock_release(int lock);

/**
 * Save provided memory buffer pointed by @a ptr to a file named @a file_name
 * in directory @a dir. Write operation of the file will complete according to
 * the requirements of synchronized I/O file integrity completion. By the time
 * function returns, the file content (provided memory buffer) and associated
 * file metadata have been transferred to the underlying hardware. If that is
 * not possible no file will be created.
 * @param dir			path to directory where file shall be saved
 * @param file_name		file name
 * @param ptr			pointer to the memory buffer
 * @param size			size of the memory buffer
 * @param force			override existing file named @a file_name if it exists
 * @return	true if file has been created, or false if an error occurred (in which
 *			case, errno is set appropriately)
 */
bool file_save_atomic(const char* dir, const char* file_name, char* ptr, size_t size, bool force);

#endif /* _UTILS_H_ */
