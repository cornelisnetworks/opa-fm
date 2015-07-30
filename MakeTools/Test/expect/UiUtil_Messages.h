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
#ifndef UIUTIL_MESSAGES_H_INCLUDED
#define UIUTIL_MESSAGES_H_INCLUDED

/*!
 @file    UiUtil/UiUtil_Messages.h
 @brief   Nationalizable Strings and Messages for UiUtil Package generated from UiUtil.msg
 */

#include "Gen_Arch.h"
#include "Gen_Macros.h"
#include "Log.h"

/* Messages */
#define UIUTIL_MSG_INVALID_UTC_OFFSET        LOG_BUILD_MSGID(MOD_UIUTIL, 0)
#define UIUTIL_MSG_INVALID_DST_OFFSET        LOG_BUILD_MSGID(MOD_UIUTIL, 1)
#define UIUTIL_MSG_INVALID_ZERO_DSTEND       LOG_BUILD_MSGID(MOD_UIUTIL, 2)
#define UIUTIL_MSG_INVALID_DSTSTART_MON      LOG_BUILD_MSGID(MOD_UIUTIL, 3)
#define UIUTIL_MSG_INVALID_DSTEND_MON        LOG_BUILD_MSGID(MOD_UIUTIL, 4)
#define UIUTIL_MSG_INVALID_DSTRANGE          LOG_BUILD_MSGID(MOD_UIUTIL, 5)
#define UIUTIL_MSG_INVALID_DSTEND_HOUR       LOG_BUILD_MSGID(MOD_UIUTIL, 6)
#define UIUTIL_MSG_INVALID_ZERO_DSTSTART     LOG_BUILD_MSGID(MOD_UIUTIL, 7)
#define UIUTIL_MSG_INVALID_NONZERO_DSTOFFSET LOG_BUILD_MSGID(MOD_UIUTIL, 8)
#define UIUTIL_MSG_INVALID_TIMEZONE_INFO     LOG_BUILD_MSGID(MOD_UIUTIL, 9)
#define UIUTIL_MSG_INVALID_DST_MON           LOG_BUILD_MSGID(MOD_UIUTIL, 10)
#define UIUTIL_MSG_INVALID_DST_MONNUM        LOG_BUILD_MSGID(MOD_UIUTIL, 11)
#define UIUTIL_MSG_INVALID_DST_DAYNUM        LOG_BUILD_MSGID(MOD_UIUTIL, 12)
#define UIUTIL_MSG_INVALID_DST_WEEKNUM       LOG_BUILD_MSGID(MOD_UIUTIL, 13)
#define UIUTIL_MSG_INVALID_DST_WDAYNUM       LOG_BUILD_MSGID(MOD_UIUTIL, 14)
#define UIUTIL_MSG_INVALID_DST_HOURNUM       LOG_BUILD_MSGID(MOD_UIUTIL, 15)

/* Logging Macros */
/* UIUTIL_MSG_INVALID_UTC_OFFSET
 * Arguments: 
 *	hour: invalid hour
 *	minute: invalid minutes
 *	minHour: low hours
 *	maxHour: high hours
 *	minMinute: low minutes
 *	maxMinute: high minutes
 */
#define UIUTIL_LOG_PART_INVALID_UTC_OFFSET(hour, minute, minHour, maxHour, minMinute, maxMinute)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_UTC_OFFSET, hour, minute, minHour, maxHour, minMinute, maxMinute)
/* UIUTIL_MSG_INVALID_DST_OFFSET
 * Arguments: 
 *	offset: invalid value
 *	min: range low
 *	max: range high
 */
#define UIUTIL_LOG_PART_INVALID_DST_OFFSET(offset, min, max)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_DST_OFFSET, offset, min, max, 0, 0, 0)
/* UIUTIL_MSG_INVALID_ZERO_DSTEND */
#define UIUTIL_LOG_PART_INVALID_ZERO_DSTEND()\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_ZERO_DSTEND, 0, 0, 0, 0, 0, 0)
/* UIUTIL_MSG_INVALID_DSTSTART_MON
 * Arguments: 
 *	month: start month name
 */
#define UIUTIL_LOG_PART_INVALID_DSTSTART_MON(month)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_DSTSTART_MON, month, 0, 0, 0, 0, 0)
/* UIUTIL_MSG_INVALID_DSTEND_MON
 * Arguments: 
 *	month: end month name
 */
#define UIUTIL_LOG_PART_INVALID_DSTEND_MON(month)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_DSTEND_MON, month, 0, 0, 0, 0, 0)
/* UIUTIL_MSG_INVALID_DSTRANGE
 * Arguments: 
 *	start: month name
 *	end: month name
 */
#define UIUTIL_LOG_PART_INVALID_DSTRANGE(start, end)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_DSTRANGE, start, end, 0, 0, 0, 0)
/* UIUTIL_MSG_INVALID_DSTEND_HOUR
 * Arguments: 
 *	hour: end hour
 *	offset: dst Offset
 */
#define UIUTIL_LOG_PART_INVALID_DSTEND_HOUR(hour, offset)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_DSTEND_HOUR, hour, offset, 0, 0, 0, 0)
/* UIUTIL_MSG_INVALID_ZERO_DSTSTART */
#define UIUTIL_LOG_PART_INVALID_ZERO_DSTSTART()\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_ZERO_DSTSTART, 0, 0, 0, 0, 0, 0)
/* UIUTIL_MSG_INVALID_NONZERO_DSTOFFSET */
#define UIUTIL_LOG_PART_INVALID_NONZERO_DSTOFFSET()\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_NONZERO_DSTOFFSET, 0, 0, 0, 0, 0, 0)
/* UIUTIL_MSG_INVALID_TIMEZONE_INFO */
#define UIUTIL_LOG_PART_INVALID_TIMEZONE_INFO()\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_TIMEZONE_INFO, 0, 0, 0, 0, 0, 0)
/* UIUTIL_MSG_INVALID_DST_MON
 * Arguments: 
 *	month: month name
 */
#define UIUTIL_LOG_PART_INVALID_DST_MON(month)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_DST_MON, month, 0, 0, 0, 0, 0)
/* UIUTIL_MSG_INVALID_DST_MONNUM
 * Arguments: 
 *	month: month number
 *	min: mon low
 *	max: mon high
 */
#define UIUTIL_LOG_PART_INVALID_DST_MONNUM(month, min, max)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_DST_MONNUM, month, min, max, 0, 0, 0)
/* UIUTIL_MSG_INVALID_DST_DAYNUM
 * Arguments: 
 *	day: day number
 *	min: day low
 *	max: day high
 */
#define UIUTIL_LOG_PART_INVALID_DST_DAYNUM(day, min, max)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_DST_DAYNUM, day, min, max, 0, 0, 0)
/* UIUTIL_MSG_INVALID_DST_WEEKNUM
 * Arguments: 
 *	week: week number
 *	min: week low
 *	max: week high
 */
#define UIUTIL_LOG_PART_INVALID_DST_WEEKNUM(week, min, max)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_DST_WEEKNUM, week, min, max, 0, 0, 0)
/* UIUTIL_MSG_INVALID_DST_WDAYNUM
 * Arguments: 
 *	wday: wday number
 *	min: wday low
 *	max: wday high
 */
#define UIUTIL_LOG_PART_INVALID_DST_WDAYNUM(wday, min, max)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_DST_WDAYNUM, wday, min, max, 0, 0, 0)
/* UIUTIL_MSG_INVALID_DST_HOURNUM
 * Arguments: 
 *	hour: hour number
 *	min: hour low
 *	max: hour high
 */
#define UIUTIL_LOG_PART_INVALID_DST_HOURNUM(hour, min, max)\
	LOG_ADD_PART(MOD_UIUTIL, UIUTIL_MSG_INVALID_DST_HOURNUM, hour, min, max, 0, 0, 0)

/* Constant Strings for use as substitutionals in Messages */
#define UIUTIL_STR_MODNAME                   LOG_BUILD_STRID(MOD_UIUTIL, 0)
#define UIUTIL_STR_DATEFORMAT                LOG_BUILD_STRID(MOD_UIUTIL, 1)
#define UIUTIL_STR_BOOTDELTAFORMAT           LOG_BUILD_STRID(MOD_UIUTIL, 2)
#define UIUTIL_STR_TIMEFORMAT                LOG_BUILD_STRID(MOD_UIUTIL, 3)
#define UIUTIL_STR_DSTFLAG_BOOT              LOG_BUILD_STRID(MOD_UIUTIL, 4)
#define UIUTIL_STR_DSTFLAG_UTC               LOG_BUILD_STRID(MOD_UIUTIL, 5)
#define UIUTIL_STR_DSTFLAG_STD               LOG_BUILD_STRID(MOD_UIUTIL, 6)
#define UIUTIL_STR_DSTFLAG_DST               LOG_BUILD_STRID(MOD_UIUTIL, 7)
#define UIUTIL_STR_JAN                       LOG_BUILD_STRID(MOD_UIUTIL, 8)
#define UIUTIL_STR_FEB                       LOG_BUILD_STRID(MOD_UIUTIL, 9)
#define UIUTIL_STR_MAR                       LOG_BUILD_STRID(MOD_UIUTIL, 10)
#define UIUTIL_STR_APR                       LOG_BUILD_STRID(MOD_UIUTIL, 11)
#define UIUTIL_STR_MAY                       LOG_BUILD_STRID(MOD_UIUTIL, 12)
#define UIUTIL_STR_JUN                       LOG_BUILD_STRID(MOD_UIUTIL, 13)
#define UIUTIL_STR_JUL                       LOG_BUILD_STRID(MOD_UIUTIL, 14)
#define UIUTIL_STR_AUG                       LOG_BUILD_STRID(MOD_UIUTIL, 15)
#define UIUTIL_STR_SEP                       LOG_BUILD_STRID(MOD_UIUTIL, 16)
#define UIUTIL_STR_OCT                       LOG_BUILD_STRID(MOD_UIUTIL, 17)
#define UIUTIL_STR_NOV                       LOG_BUILD_STRID(MOD_UIUTIL, 18)
#define UIUTIL_STR_DEC                       LOG_BUILD_STRID(MOD_UIUTIL, 19)

/* String Formatting Macros */
/* UIUTIL_STR_DATEFORMAT: format to use for real Dates
 * Arguments: 
 *	year: (4 digit)
 *	mon
 *	day
 */
#define UIUTIL_FMT_DATEFORMAT(buffer, year, mon, day)\
	LOG_FORMATBUF(buffer, Log_GetString(UIUTIL_STR_DATEFORMAT, (buffer).GetLanguage()), year, mon, day, 0, 0, 0)
/* UIUTIL_STR_BOOTDELTAFORMAT: format to use days since boot
 * Arguments: 
 *	daysUp: days since boot (up to 30 years = 10957)
 */
#define UIUTIL_FMT_BOOTDELTAFORMAT(buffer, daysUp)\
	LOG_FORMATBUF(buffer, Log_GetString(UIUTIL_STR_BOOTDELTAFORMAT, (buffer).GetLanguage()), daysUp, 0, 0, 0, 0, 0)
/* UIUTIL_STR_TIMEFORMAT: format to use for Time
 * Arguments: 
 *	hour
 *	min
 *	sec
 *	msec
 *	usec
 *	DstFlag
 */
#define UIUTIL_FMT_TIMEFORMAT(buffer, hour, min, sec, msec, usec, DstFlag)\
	LOG_FORMATBUF(buffer, Log_GetString(UIUTIL_STR_TIMEFORMAT, (buffer).GetLanguage()), hour, min, sec, msec, usec, DstFlag)

GEN_EXTERNC(extern void UiUtil_AddMessages();)

#endif /* UIUTIL_MESSAGES_H_INCLUDED */
