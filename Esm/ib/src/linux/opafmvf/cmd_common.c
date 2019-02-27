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

#include "cmd_common.h"

#include "tenant.h"
#include "utils.h"
#include <iba/public/imemory.h>


bool
extract_vf_name(int argc, char* argv[], int idx, const char** name)
{
	if (idx >= argc) {
		log_error("%s: missing argument '<vf_name>'", argv[0]);
		return false;
	}

	if (strlen(argv[idx]) >= MAX_NAME_LENGTH) {
		log_error("%s: invalid virtual fabric name '%s' (max length: %d)", argv[0], argv[idx], MAX_NAME_LENGTH - 1);
		return false;
	}

	/*
	 * check whether name contains only (A-Z)(a-z)(0-9) "-" "," "=" "." "_"
	 * otherwise it could create an issue when creating file or parsing XML
	 */
	for (char* c = argv[idx]; *c != '\0'; ++c) {
		if (*c >= 'a' && *c <= 'z')
			continue;
		if (*c >= 'A' && *c <= 'Z')
			continue;
		if (*c >= '0' && *c <= '9')
			continue;
		if (*c == '-' || *c == ',' || *c == '=' || *c == '.' || *c == '_')
			continue;

		log_error("%s: invalid virtual fabric name '%s' (illegal char '%c')", argv[0], argv[idx], *c);
		return false;
	}

	*name = argv[idx];

	return true;
}

bool
extract_port_guids(int argc, char* argv[], int idx, uint64_t** guids, size_t* guids_count)
{
	if (idx >= argc) {
		log_error("%s: missing argument '<port_guid>'", argv[0]);
		return false;
	}

	uint64_t* guids_tmp = (uint64_t*)calloc(argc - idx, sizeof(uint64_t));
	if (!guids_tmp)
		die("Out of memory");

	for (int i = idx; i < argc; ++i) {
		if (FSUCCESS != StringToUint64(&guids_tmp[i - idx], argv[i], NULL, 0, true)) {
			log_error("%s: invalid port guid '%s'", argv[0], argv[i]);
			goto error;
		}
	}

	*guids = guids_tmp;
	*guids_count = argc - idx;

	return true;

error:
	free(guids_tmp);
	return false;
}

bool
string_to_pkey(uint16_t* pkey, const char* str)
{
	uint16_t pkey_tmp;
	if (FSUCCESS != StringToUint16(&pkey_tmp, str, NULL, 0, true))
		return false;

	if (pkey_tmp >= 0x7fff)
		return false;

	*pkey = pkey_tmp;

	return true;
}

void
print_help(const char* msg)
{
	fprintf(stdout, "%s", msg);
}

void
print_usage(const char* msg)
{
	fprintf(stderr, "%s", msg);
}
