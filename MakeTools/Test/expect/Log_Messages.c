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
 @file    Log/Log_Messages.c
 @brief   Nationalizable Strings and Messages for Log Package generated from Log.msg
 */

#include "Log_Messages.h"


/*!
Module specific message table, referenced by Log_MessageId_t.

The entries in the table must provide equivalent argument usage for
all languages.  Argument usage in format strings is used to determine
which arguments may need to be freed
*/

static Log_MessageEntry_t log_messageTable[] =
{
	{	LOG_MSG_TRUNCATED,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: None */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Message truncated, out of Log Resources",
				/* response */    "Some log text has been discarded",
				/* correction */  "To avoid the loss of log text, reduce the logging options enabled",
			},
		},
	},
	{	LOG_MSG_TRAP_FAILED,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> erc
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Unable to Send Trap: %C",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
};


/*!
Module specific string table, referenced by Log_StringId_t.
*/

static Log_StringEntry_t log_stringTable[] =
{
	{	LOG_STR_MODNAME,	/* first entry must be 1-6 character package name */
		{ /* LOG_LANG_ENGLISH */ "Log",
		},
	},
	{	LOG_STR_RESPONSE,	/* Prefix for Response part of message */
		{ /* LOG_LANG_ENGLISH */ "Response: ",
		},
	},
	{	LOG_STR_CORRECTION,	/* Prefix for Correction part of message */
		{ /* LOG_LANG_ENGLISH */ "Correction: ",
		},
	},
	{	LOG_STR_TASK,	/* Task Id reporting message */
		{ /* LOG_LANG_ENGLISH */ "Task",
		},
	},
	{	LOG_STR_FILENAME,	/* Source file generating message */
		{ /* LOG_LANG_ENGLISH */ "Filename",
		},
	},
	{	LOG_STR_LINE,	/* Source file line number generating message */
		{ /* LOG_LANG_ENGLISH */ "Line",
		},
	},
	{	LOG_STR_NULL,	/* marker to indicate a NULL %s or %F argument was passed */
		{ /* LOG_LANG_ENGLISH */ "(NULL)",
		},
	},
	{	LOG_STR_NOMEMORY,	/* marker to indicate a LOG_ALLOC_ERROR argument was passed */
		{ /* LOG_LANG_ENGLISH */ "(No Memory)",
		},
	},
	{	LOG_STR_SEV_DUMP,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "D",
		},
	},
	{	LOG_STR_SEV_FATAL,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "F",
		},
	},
	{	LOG_STR_SEV_ERROR,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "E",
		},
	},
	{	LOG_STR_SEV_ALARM,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "A",
		},
	},
	{	LOG_STR_SEV_WARNING,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "W",
		},
	},
	{	LOG_STR_SEV_PARTIAL,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "p",
		},
	},
	{	LOG_STR_SEV_CONFIG,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "C",
		},
	},
	{	LOG_STR_SEV_PROGRAM_INFO,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "I",
		},
	},
	{	LOG_STR_SEV_PERIODIC_INFO,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "P",
		},
	},
	{	LOG_STR_SEV_DEBUG1,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "1",
		},
	},
	{	LOG_STR_SEV_DEBUG2,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "2",
		},
	},
	{	LOG_STR_SEV_DEBUG3,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "3",
		},
	},
	{	LOG_STR_SEV_DEBUG4,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "4",
		},
	},
	{	LOG_STR_SEV_DEBUG5,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "5",
		},
	},
	{	LOG_STR_SEV_PURGE,	/* 1 character indication of severity */
		{ /* LOG_LANG_ENGLISH */ "X",
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
Log_AddMessages()
{
	Log_AddStringTable(MOD_LOG, log_stringTable,
						GEN_NUMENTRIES(log_stringTable));
	Log_AddMessageTable(MOD_LOG, log_messageTable,
						GEN_NUMENTRIES(log_messageTable));
}
