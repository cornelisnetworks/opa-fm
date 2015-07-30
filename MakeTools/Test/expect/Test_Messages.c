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
 @file    Osa/Test_Messages.c
 @brief   Nationalizable Strings and Messages for Osa Package generated from Test.msg
 */

#include "Test_Messages.h"


/*!
Module specific message table, referenced by Log_MessageId_t.

The entries in the table must provide equivalent argument usage for
all languages.  Argument usage in format strings is used to determine
which arguments may need to be freed
*/

static Log_MessageEntry_t osa_messageTable[] =
{
	{	OSA_MSG_ASSERT_FAILED,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> expr: text for failed expression
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Assertion Failed: %s",
				/* response */    "Firmware will dump and reboot",
				/* correction */  "Save the Dump and send it to support",
			},
		},
	},
	{	OSA_MSG_TEST1,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> obj1
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "%F",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	OSA_MSG_TEST2,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> arg1
		 *	arg2 -> arg2
		 *	arg3 -> obj1
		 *	arg4 -> obj2
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "%3$F %4$F",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	OSA_MSG_TEST3,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> obj1
		 *	arg2 -> obj2
		 *	arg3 -> width1
		 *	arg4 -> width2
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "%1$*3$F %2$*4$F",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	OSA_MSG_TEST4,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> obj1
		 *	arg2 -> obj2
		 *	arg3 -> width1
		 *	arg4 -> width2
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "%1$.*3$F %2$.*4$F",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	OSA_MSG_TEST5,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> width1
		 *	arg2 -> obj1
		 *	arg3 -> width2
		 *	arg4 -> obj2
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "%*F %*F",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	OSA_MSG_TEST6,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> width1
		 *	arg2 -> obj1
		 *	arg3 -> width2
		 *	arg4 -> obj2
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "%.*F %.*F",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	OSA_MSG_TEST7,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> width1a
		 *	arg2 -> width1b
		 *	arg3 -> obj1
		 *	arg4 -> width2a
		 *	arg5 -> width2b
		 *	arg6 -> obj2
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "%*.*F %*.*F",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	OSA_MSG_TEST8,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> dec
		 *	arg2 -> obj2
		 *	arg3 -> width1
		 *	arg4 -> width2
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "%*3$2$F %*4$1$d",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	OSA_MSG_TEST9,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: None */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "#",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	OSA_MSG_TEST10,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: None */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "\"#",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	OSA_MSG_TEST11,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: None */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "#\"",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	OSA_MSG_TEST12,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: None */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "\" #",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
};


/*!
Module specific string table, referenced by Log_StringId_t.
*/

static Log_StringEntry_t osa_stringTable[] =
{
	{	OSA_STR_MODNAME,	/* first entry must be 1-6 character package name */
		{ /* LOG_LANG_ENGLISH */ "Osa",
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
Osa_AddMessages()
{
	Log_AddStringTable(MOD_OSA, osa_stringTable,
						GEN_NUMENTRIES(osa_stringTable));
	Log_AddMessageTable(MOD_OSA, osa_messageTable,
						GEN_NUMENTRIES(osa_messageTable));
}
