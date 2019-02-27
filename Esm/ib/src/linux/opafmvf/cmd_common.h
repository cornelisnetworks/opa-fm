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

#ifndef _CMD_COMMON_H_
#define _CMD_COMMON_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Command status code, returned by opafmvf to the system. Some of them are
 * dedicated to specific sub-command (like create, ismember, ...), thus it
 * is perfectly fine they reuse same values.
 */
typedef enum _CmdStatus {
	CMD_STATUS_OK				= 0,			/**< VF created, node added, node removed and similar. */
	CMD_STATUS_EXIST			= 0,			/**< VF exists. */
	CMD_STATUS_NOT_EXIST		= 1,			/**< VF doesn't exist. */
	CMD_STATUS_ALL_MEMBER		= 0,			/**< All PortGUIDs are members of VF. */
	CMD_STATUS_NONE_MEMBER		= 0,			/**< None of PortGUIDs is member of VF. */
	CMD_STATUS_SOME_MEMBER		= 1,			/**< Some of PortGUIDS are members of VF. */
	CMD_STATUS_CANNOT_PERFORM 	= 1,			/**< Can't create VF because a VF with this name exists,
													 can't delete VF because it's not empty,
													 can't add node to VF because it's already added to it
													 and similar. */
	CMD_STATUS_INVARG					= 2,	/**< Invalid PortGUID and similar. */
	CMD_STATUS_ERROR					= 2,	/**< Can't connect to FM or similar. */
	CMD_STATUS_ANOTHER_INSTANCE_RUNNING	= 2,	/**< Another instance of opafmvf is running. */
} CmdStatus;

bool extract_vf_name(int argc, char* argv[], int idx, const char** name);
bool extract_port_guids(int argc, char* argv[], int idx, uint64_t** guids, size_t* guids_count);

bool string_to_pkey(uint16_t* pkey, const char* str);

void print_help(const char* msg);
void print_usage(const char* msg);

#endif /* _CMD_COMMON_H_ */
