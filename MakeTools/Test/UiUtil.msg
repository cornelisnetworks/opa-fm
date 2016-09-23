# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015, Intel Corporation
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor the names of its contributors
#       may be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# END_ICS_COPYRIGHT8   ****************************************

#[ICS VERSION STRING: unknown]
#
# $Source: /cvs/ics/MakeTools/Test/UiUtil.msg,v $
# $Name:  $
# $Revision: 1.3 $
# $Date: 2015/01/28 20:18:11 $
#

# 
# Message Definitions for UiUtil module
#

[Module]
Name=UiUtil

[Message]
Name=INVALID_UTC_OFFSET
Severity=Partial
Arglist=hour: invalid hour, minute: invalid minutes, minHour: low hours,
		maxHour: high hours, minMinute: low minutes, maxMinute: high minutes
Description.English="Invalid UTC Offset: %d:%2.2d"
Correction.English="UTC Offset must be %3$d:%4$2.2d to %5$d:%6$2.2d"

[Message]
Name=INVALID_DST_OFFSET
Severity=Partial
Arglist=offset: invalid value, min: range low, max: range high
Description.English="Invalid Daylight Savings Time Offset: %d"
Correction.English="Daylight Savings Time Offset must be %2$d to %3$d"

[Message]
Name=INVALID_ZERO_DSTEND
Severity=Partial
Description.English="Daylight Savings Start specified without Daylight Savings End"

[Message]
Name=INVALID_DSTSTART_MON
Severity=Partial
Arglist=month: start month name
Description.English="Invalid Daylight Savings Start: %S"
Correction.English="Cannot be Jan nor Feb"

[Message]
Name=INVALID_DSTEND_MON
Severity=Partial
Arglist=month: end month name
Description.English="Invalid Daylight Savings End: %S"
Correction.English="Cannot be Jan, Feb, nor Dec"

[Message]
Name=INVALID_DSTRANGE
Severity=Partial
Arglist=start: month name, end: month name
Description.English="Invalid Daylight Savings Range (%S-%S)"
Correction.English="Start must be a month prior to End"

[Message]
Name=INVALID_DSTEND_HOUR
Severity=Partial
Arglist=hour: end hour, offset: dst Offset
Description.English="Invalid Daylight Savings End Hour: %d"
Correction.English="Cannot be within %2$d hours of start nor end of day"

[Message]
Name=INVALID_ZERO_DSTSTART
Severity=Partial
Description.English="Daylight Savings End specified without Daylight Savings Start"

[Message]
Name=INVALID_NONZERO_DSTOFFSET
Severity=Partial
Description.English="Daylight Savings Offset specified without Daylight Savings Start/End"

[Message]
Name=INVALID_TIMEZONE_INFO
Severity=Partial
Description.English="Invalid TimeZone Information"
Response.English="The TimeZone Information will not be changed"
Correction.English="Correct the Information and re-enter as needed"

[Message]
Name=INVALID_DST_MON
Severity=Partial
Arglist=month: month name
Description.English="Invalid Daylight Savings Transition Month: %S"
Correction.English="Cannot be Feb"

[Message]
Name=INVALID_DST_MONNUM
Severity=Partial
Arglist=month: month number, min: mon low, max: mon high
Description.English="Invalid Daylight Savings Month: %d"
Correction.English="Must be %2$d to %3$d"

[Message]
Name=INVALID_DST_DAYNUM
Severity=Partial
Arglist=day: day number, min: day low, max: day high
Description.English="Invalid Daylight Savings Day: %d"
Correction.English="Must be %2$d to %3$d"

[Message]
Name=INVALID_DST_WEEKNUM
Severity=Partial
Arglist=week: week number, min: week low, max: week high
Description.English="Invalid Daylight Savings Week: %d"
Correction.English="Must be %2$d to %3$d"

[Message]
Name=INVALID_DST_WDAYNUM
Severity=Partial
Arglist=wday: wday number, min: wday low, max: wday high
Description.English="Invalid Daylight Savings Weekday: %d"
Correction.English="Must be %2$d to %3$d"

[Message]
Name=INVALID_DST_HOURNUM
Severity=Partial
Arglist=hour: hour number, min: hour low, max: hour high
Description.English="Invalid Daylight Savings Hour: %d"
Correction.English="Must be %2$d to %3$d"

[String]
Name=DATEFORMAT
Comment=format to use for real Dates
Arglist=year: (4 digit), mon, day
String.English="%4.4d/%2.2d/%2.2d"
# US Style would be: "%2$2.2d/%3$2.2d/%1$4.4d"

[String]
Name=BOOTDELTAFORMAT
Comment=format to use days since boot
Arglist=daysUp: days since boot (up to 30 years = 10957)
String.English="%5d"

[String]
Name=TIMEFORMAT
Comment=format to use for Time
Arglist=hour, min, sec, msec, usec, DstFlag
String.English="%2.2d:%2.2d:%2.2d.%3.3d%6$1.1S"

[String]
Name=DSTFLAG_BOOT
Comment=1 character, mark time as since boot
String.English="B"

[String]
Name=DSTFLAG_UTC
Comment=1 character, mark time as UTC
String.English="U"

[String]
Name=DSTFLAG_STD
Comment=1 character, mark time as standard
String.English="S"

[String]
Name=DSTFLAG_DST
Comment=1 character, mark time as Daylight Savings
String.English="D"

[String]
Name=JAN
Comment=January abbreviation
String.English="Jan"

[String]
Name=FEB
Comment=February abbreviation
String.English="Feb"

[String]
Name=MAR
Comment=March abbreviation
String.English="Mar"

[String]
Name=APR
Comment=April abbreviation
String.English="Apr"

[String]
Name=MAY
Comment=May abbreviation
String.English="May"

[String]
Name=JUN
Comment=June abbreviation
String.English="Jun"

[String]
Name=JUL
Comment=July abbreviation
String.English="Jul"

[String]
Name=AUG
Comment=August abbreviation
String.English="Aug"

[String]
Name=SEP
Comment=September abbreviation
String.English="Sep"

[String]
Name=OCT
Comment=October abbreviation
String.English="Oct"

[String]
Name=NOV
Comment=November abbreviation
String.English="Nov"

[String]
Name=DEC
Comment=December abbreviation
String.English="Dec"
