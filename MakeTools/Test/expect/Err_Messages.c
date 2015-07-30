/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2015, Intel Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

** END_ICS_COPYRIGHT7   ****************************************/

/* [ICS VERSION STRING: unknown] */
/*!
 @file    Err/Err_Messages.c
 @brief   Nationalizable Strings and Messages for Err Package generated from Err.msg
 */

#include "Err_Messages.h"


/*!
Module specific string table, referenced by Log_StringId_t.
*/

static Log_StringEntry_t err_stringTable[] =
{
	{	ERR_STR_MODNAME,	/* first entry must be 1-6 character package name */
		{ /* LOG_LANG_ENGLISH */ "Err",
		},
	},
	{	ERR_STR_OK,	/* ERR_OK text */
		{ /* LOG_LANG_ENGLISH */ "Success",
		},
	},
	{	ERR_STR_FAILED_CONSTRUCTOR,	/* ERR_FAILED_CONSTRUCTOR and ERR_KIND_FAILED_CONSTRUCTORtext */
		{ /* LOG_LANG_ENGLISH */ "Failed Constructor",
		},
	},
	{	ERR_STR_END_OF_DATA,	/* ERR_END_OF_DATA and ERR_KIND_END_OF_DATA text */
		{ /* LOG_LANG_ENGLISH */ "End of Data",
		},
	},
	{	ERR_STR_IO_ERROR,	/* ERR_IO_ERROR and ERR_KIND_IO_ERROR text */
		{ /* LOG_LANG_ENGLISH */ "IO Error",
		},
	},
	{	ERR_STR_INVALID,	/* ERR_INVALID and ERR_KIND_INVALID text */
		{ /* LOG_LANG_ENGLISH */ "Invalid",
		},
	},
	{	ERR_STR_MISSING,	/* ERR_MISSING and ERR_KIND_MISSING text */
		{ /* LOG_LANG_ENGLISH */ "Missing",
		},
	},
	{	ERR_STR_DUPLICATE,	/* ERR_DUPLICATE and ERR_KIND_DUPLICATE text */
		{ /* LOG_LANG_ENGLISH */ "Duplicate",
		},
	},
	{	ERR_STR_OVERFLOW,	/* ERR_OVERFLOW and ERR_KIND_OVERFLOW text */
		{ /* LOG_LANG_ENGLISH */ "Overflow",
		},
	},
	{	ERR_STR_TIMEOUT,	/* ERR_TIMEOUT and ERR_KIND_TIMEOUT text */
		{ /* LOG_LANG_ENGLISH */ "Timeout",
		},
	},
	{	ERR_STR_UNDERFLOW,	/* ERR_UNDERFLOW and ERR_KIND_UNDERFLOW text */
		{ /* LOG_LANG_ENGLISH */ "Underflow",
		},
	},
	{	ERR_STR_OUT_OF_RESOURCES,	/* ERR_OUT_OF_RESOURCES and ERR_KIND_OUT_OF_RESOURCES text */
		{ /* LOG_LANG_ENGLISH */ "Out of Resources",
		},
	},
	{	ERR_STR_UNKNOWN,	/* ERR_KIND_UNKNOWN text */
		{ /* LOG_LANG_ENGLISH */ "Unknown",
		},
	},
	{	ERR_STR_BAD,	/* invalid Err_Code_t value for Err module defined errors */
		{ /* LOG_LANG_ENGLISH */ "Bad",
		},
	},
};

/*!
Add String table to logging subsystem's tables.

This must be run early in the boot to initialize tables prior to
any logging functions being available.

@return None

@heading Concurrency Considerations
    Must be run once, early in boot

@heading Special Cases and Error Handling
	None

@see Log_AddStringTable
*/

void
Err_AddMessages()
{
	Log_AddStringTable(MOD_ERR, err_stringTable,
						GEN_NUMENTRIES(err_stringTable));
}
