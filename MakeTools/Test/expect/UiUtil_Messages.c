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
 @file    UiUtil/UiUtil_Messages.c
 @brief   Nationalizable Strings and Messages for UiUtil Package generated from UiUtil.msg
 */

#include "UiUtil_Messages.h"


/*!
Module specific message table, referenced by Log_MessageId_t.

The entries in the table must provide equivalent argument usage for
all languages.  Argument usage in format strings is used to determine
which arguments may need to be freed
*/

static Log_MessageEntry_t uiUtil_messageTable[] =
{
	{	UIUTIL_MSG_INVALID_UTC_OFFSET,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> hour: invalid hour
		 *	arg2 -> minute: invalid minutes
		 *	arg3 -> minHour: low hours
		 *	arg4 -> maxHour: high hours
		 *	arg5 -> minMinute: low minutes
		 *	arg6 -> maxMinute: high minutes
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid UTC Offset: %d:%2.2d",
				/* response */    NULL,
				/* correction */  "UTC Offset must be %3$d:%4$2.2d to %5$d:%6$2.2d",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_DST_OFFSET,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> offset: invalid value
		 *	arg2 -> min: range low
		 *	arg3 -> max: range high
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid Daylight Savings Time Offset: %d",
				/* response */    NULL,
				/* correction */  "Daylight Savings Time Offset must be %2$d to %3$d",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_ZERO_DSTEND,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: None */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Daylight Savings Start specified without Daylight Savings End",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	UIUTIL_MSG_INVALID_DSTSTART_MON,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> month: start month name
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid Daylight Savings Start: %S",
				/* response */    NULL,
				/* correction */  "Cannot be Jan nor Feb",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_DSTEND_MON,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> month: end month name
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid Daylight Savings End: %S",
				/* response */    NULL,
				/* correction */  "Cannot be Jan, Feb, nor Dec",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_DSTRANGE,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> start: month name
		 *	arg2 -> end: month name
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid Daylight Savings Range (%S-%S)",
				/* response */    NULL,
				/* correction */  "Start must be a month prior to End",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_DSTEND_HOUR,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> hour: end hour
		 *	arg2 -> offset: dst Offset
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid Daylight Savings End Hour: %d",
				/* response */    NULL,
				/* correction */  "Cannot be within %2$d hours of start nor end of day",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_ZERO_DSTSTART,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: None */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Daylight Savings End specified without Daylight Savings Start",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	UIUTIL_MSG_INVALID_NONZERO_DSTOFFSET,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: None */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Daylight Savings Offset specified without Daylight Savings Start/End",
				/* response */    NULL,
				/* correction */  NULL,
			},
		},
	},
	{	UIUTIL_MSG_INVALID_TIMEZONE_INFO,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: None */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid TimeZone Information",
				/* response */    "The TimeZone Information will not be changed",
				/* correction */  "Correct the Information and re-enter as needed",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_DST_MON,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> month: month name
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid Daylight Savings Transition Month: %S",
				/* response */    NULL,
				/* correction */  "Cannot be Feb",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_DST_MONNUM,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> month: month number
		 *	arg2 -> min: mon low
		 *	arg3 -> max: mon high
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid Daylight Savings Month: %d",
				/* response */    NULL,
				/* correction */  "Must be %2$d to %3$d",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_DST_DAYNUM,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> day: day number
		 *	arg2 -> min: day low
		 *	arg3 -> max: day high
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid Daylight Savings Day: %d",
				/* response */    NULL,
				/* correction */  "Must be %2$d to %3$d",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_DST_WEEKNUM,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> week: week number
		 *	arg2 -> min: week low
		 *	arg3 -> max: week high
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid Daylight Savings Week: %d",
				/* response */    NULL,
				/* correction */  "Must be %2$d to %3$d",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_DST_WDAYNUM,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> wday: wday number
		 *	arg2 -> min: wday low
		 *	arg3 -> max: wday high
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid Daylight Savings Weekday: %d",
				/* response */    NULL,
				/* correction */  "Must be %2$d to %3$d",
			},
		},
	},
	{	UIUTIL_MSG_INVALID_DST_HOURNUM,
		0,	/* argument number which holds unit number, 0 -> none */
		LOG_TRAP_NONE,	/* trap func associated with message */
		/* FUTURE - Binary Data */
		/* Arguments: 
		 *	arg1 -> hour: hour number
		 *	arg2 -> min: hour low
		 *	arg3 -> max: hour high
		 */
		{ /* Language Specific Entries */
			{ /* LOG_LANG_ENGLISH */
				/* description */ "Invalid Daylight Savings Hour: %d",
				/* response */    NULL,
				/* correction */  "Must be %2$d to %3$d",
			},
		},
	},
};


/*!
Module specific string table, referenced by Log_StringId_t.
*/

static Log_StringEntry_t uiUtil_stringTable[] =
{
	{	UIUTIL_STR_MODNAME,	/* first entry must be 1-6 character package name */
		{ /* LOG_LANG_ENGLISH */ "UiUtil",
		},
	},
	{	UIUTIL_STR_DATEFORMAT,	/* format to use for real Dates */
		/* Arguments: 
		 *	arg1 -> year: (4 digit)
		 *	arg2 -> mon
		 *	arg3 -> day
		 */
		{ /* LOG_LANG_ENGLISH */ "%4.4d/%2.2d/%2.2d",
		},
	},
	{	UIUTIL_STR_BOOTDELTAFORMAT,	/* format to use days since boot */
		/* Arguments: 
		 *	arg1 -> daysUp: days since boot (up to 30 years = 10957)
		 */
		{ /* LOG_LANG_ENGLISH */ "%5d",
		},
	},
	{	UIUTIL_STR_TIMEFORMAT,	/* format to use for Time */
		/* Arguments: 
		 *	arg1 -> hour
		 *	arg2 -> min
		 *	arg3 -> sec
		 *	arg4 -> msec
		 *	arg5 -> usec
		 *	arg6 -> DstFlag
		 */
		{ /* LOG_LANG_ENGLISH */ "%2.2d:%2.2d:%2.2d.%3.3d%6$1.1S",
		},
	},
	{	UIUTIL_STR_DSTFLAG_BOOT,	/* 1 character, mark time as since boot */
		{ /* LOG_LANG_ENGLISH */ "B",
		},
	},
	{	UIUTIL_STR_DSTFLAG_UTC,	/* 1 character, mark time as UTC */
		{ /* LOG_LANG_ENGLISH */ "U",
		},
	},
	{	UIUTIL_STR_DSTFLAG_STD,	/* 1 character, mark time as standard */
		{ /* LOG_LANG_ENGLISH */ "S",
		},
	},
	{	UIUTIL_STR_DSTFLAG_DST,	/* 1 character, mark time as Daylight Savings */
		{ /* LOG_LANG_ENGLISH */ "D",
		},
	},
	{	UIUTIL_STR_JAN,	/* January abbreviation */
		{ /* LOG_LANG_ENGLISH */ "Jan",
		},
	},
	{	UIUTIL_STR_FEB,	/* February abbreviation */
		{ /* LOG_LANG_ENGLISH */ "Feb",
		},
	},
	{	UIUTIL_STR_MAR,	/* March abbreviation */
		{ /* LOG_LANG_ENGLISH */ "Mar",
		},
	},
	{	UIUTIL_STR_APR,	/* April abbreviation */
		{ /* LOG_LANG_ENGLISH */ "Apr",
		},
	},
	{	UIUTIL_STR_MAY,	/* May abbreviation */
		{ /* LOG_LANG_ENGLISH */ "May",
		},
	},
	{	UIUTIL_STR_JUN,	/* June abbreviation */
		{ /* LOG_LANG_ENGLISH */ "Jun",
		},
	},
	{	UIUTIL_STR_JUL,	/* July abbreviation */
		{ /* LOG_LANG_ENGLISH */ "Jul",
		},
	},
	{	UIUTIL_STR_AUG,	/* August abbreviation */
		{ /* LOG_LANG_ENGLISH */ "Aug",
		},
	},
	{	UIUTIL_STR_SEP,	/* September abbreviation */
		{ /* LOG_LANG_ENGLISH */ "Sep",
		},
	},
	{	UIUTIL_STR_OCT,	/* October abbreviation */
		{ /* LOG_LANG_ENGLISH */ "Oct",
		},
	},
	{	UIUTIL_STR_NOV,	/* November abbreviation */
		{ /* LOG_LANG_ENGLISH */ "Nov",
		},
	},
	{	UIUTIL_STR_DEC,	/* December abbreviation */
		{ /* LOG_LANG_ENGLISH */ "Dec",
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
UiUtil_AddMessages()
{
	Log_AddStringTable(MOD_UIUTIL, uiUtil_stringTable,
						GEN_NUMENTRIES(uiUtil_stringTable));
	Log_AddMessageTable(MOD_UIUTIL, uiUtil_messageTable,
						GEN_NUMENTRIES(uiUtil_messageTable));
}
