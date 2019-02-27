/* BEGIN_ICS_COPYRIGHT6 ****************************************

Copyright (c) 2015-2017, Intel Corporation

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

** END_ICS_COPYRIGHT6   ****************************************/

#if !defined(VXWORKS)
#define _GNU_SOURCE
#endif

#include "imemory.h"
#include "idebug.h"
#include "itimer.h"
#include <stdarg.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>

#define MAX_VFABRIC_NAME	64	// from fm_xml.h

// from stl_pa.h
#define STL_PA_SELECT_UTIL_HIGH			0x00020001
#define STL_PA_SELECT_UTIL_MC_HIGH		0x00020081
#define STL_PA_SELECT_UTIL_PKTS_HIGH		0x00020082
#define STL_PA_SELECT_CATEGORY_INTEG			0x00030001
#define STL_PA_SELECT_CATEGORY_CONG			0x00030002
#define STL_PA_SELECT_CATEGORY_SMA_CONG		0x00030003
#define STL_PA_SELECT_CATEGORY_BUBBLE		0x00030004
#define STL_PA_SELECT_CATEGORY_SEC			0x00030005
#define STL_PA_SELECT_CATEGORY_ROUT			0x00030006
#define FOCUS_PORTS_COMPARATOR_INVALID			0
#define FOCUS_PORTS_COMPARATOR_GREATER_THAN		1
#define FOCUS_PORTS_COMPARATOR_LESS_THAN		2
#define FOCUS_PORTS_COMPARATOR_GREATER_THAN_OR_EQUAL	3
#define FOCUS_PORTS_COMPARATOR_LESS_THAN_OR_EQUAL	4

#if defined(MEM_TRACK_ON)
#if defined(VXWORKS)
#include "tickLib.h"
#define TICK tickGet()
#else
#define TICK 0 // TBD need to specify value for Linux and MAC
#endif
#define MEM_TRACK_FTR
#include "imemtrack.h"

// Default number of headers to allocate at a time.
#define MEM_HDR_ALLOC_SIZE	50

#if defined(MEM_TRACK_ON)
#define UNTRACKED_COUNT 12000
void *unTAddr[UNTRACKED_COUNT];
int unTindex=0;
int unTmissed=0;
#endif

MEM_TRACKER	*pMemTracker = NULL;
static uint32 last_reported_allocations;
static uint32 total_allocations;
static uint32 last_reported_secs;
static uint32 current_allocations;
static uint32 current_allocated;
static uint32 max_allocations;
static uint32 max_allocated;

static void MemoryTrackerDereference(MemoryTrackerFileName_t *trk);
static MemoryTrackerFileName_t *MemoryTrackerBuckets[MEMORY_TRACKER_BUCKETS];
#endif	// MEM_TRACK_ON

#ifdef VXWORKS
extern unsigned long long strtoull (const char *__s, char **__endptr, int __base);
extern long long strtoll (const char *__s, char **__endptr, int __base);
#endif

char*
StringConcat(const char* str1, ...)
{
#ifdef VXWORKS
#define stpcpy(s1, s2) (strcpy(s1, s2) + strlen(s2))
#endif

	if (!str1)
		return NULL;

	/* calculate length first */
	size_t strings_length = 1 + strlen(str1);

	va_list args;
	va_start(args, str1);
	char* str = va_arg(args, char*);
	while (str) {
		strings_length += strlen(str);
		str = va_arg(args, char*);
	}
	va_end(args);

	/* next, allocate a memory */
	char* string = (char*)malloc(strings_length);
	if (!string)
		return NULL;

	/* finally, create the string */
	char* string_tmp = stpcpy(string, str1);
	va_start(args, str1);
	str = va_arg(args, char*);
	while (str) {
		string_tmp = stpcpy(string_tmp, str);
		str = va_arg(args, char*);
	}
	va_end(args);

	return string;
}

// convert a string to a uint64.  Very similar to strtoull except that
// base=0 implies base 10 or 16, but excludes base 8
// hence allowing leading 0's for base 10.
//
// Also provides easier to use status returns and error checking.
//
// Can also optionally skip trailing whitespace, when skip_trail_whietspace is
// FALSE, trailing whitespace is treated as a FERROR
//
// When endptr is NULL, trailing characters (after optional whitespace) are 
// considered an FERROR.  When endptr is non-NULL, for a FSUCCESS conversion,
// it points to the characters after the optional trailing whitespace.
// Errors:
//	FSUCCESS - successful conversion, *endptr points to 1st char after value
//	FERROR - invalid contents, non-numeric
//	FINVALID_SETTING - value out of range
//	FINVALID_PARAMETER - invalid function arguments (NULL value or str)
FSTATUS StringToUint64(uint64 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace)
{
	char *end = NULL;
	uint64 temp;

	if (! str || ! value)
		return FINVALID_PARAMETER;
	errno = 0;
	temp = strtoull(str, &end, base?base:10);
	if ( ! base && ! (temp == IB_UINT64_MAX && errno)
		&& (end && temp == 0 && *end == 'x' && end != str)) {
		// try again as base 16
		temp = strtoull(str, &end, 16);
	}
	if ((temp == IB_UINT64_MAX && errno)
		|| (end && end == str)) {
		if (errno == ERANGE)
			return FINVALID_SETTING;
		else
			return FERROR;
	}

	// skip whitespace
	if (end && skip_trail_whitespace) {
		while (isspace(*end)) {
			end++;
		}
	}
	if (endptr)
		*endptr = end;
	else if (end && *end != '\0')
		return FERROR;
	*value = temp;
	return FSUCCESS;
}

FSTATUS StringToUint32(uint32 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace)
{
	uint64 temp;
	FSTATUS status;

	status = StringToUint64(&temp, str, endptr, base, skip_trail_whitespace);
	if (status != FSUCCESS)
		return status;
	if (temp > IB_UINT32_MAX)
		return FINVALID_SETTING;
	*value = (uint32)temp;
	return FSUCCESS;
}

FSTATUS StringToUint16(uint16 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace)
{
	uint64 temp;
	FSTATUS status;

	status = StringToUint64(&temp, str, endptr, base, skip_trail_whitespace);
	if (status != FSUCCESS)
		return status;
	if (temp > IB_UINT16_MAX)
		return FINVALID_SETTING;
	*value = (uint16)temp;
	return FSUCCESS;
}

FSTATUS StringToUint8(uint8 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace)
{
	uint64 temp;
	FSTATUS status;

	status = StringToUint64(&temp, str, endptr, base, skip_trail_whitespace);
	if (status != FSUCCESS)
		return status;
	if (temp > IB_UINT8_MAX)
		return FINVALID_SETTING;
	*value = (uint8)temp;
	return FSUCCESS;
}

// convert a string to a int64.  Very similar to strtoull except that
// base=0 implies base 10 or 16, but excludes base 8
// hence allowing leading 0's for base 10.
//
// Also provides easier to use status returns and error checking.
//
// Can also optionally skip trailing whitespace, when skip_trail_whitespace is
// FALSE, trailing whitespace is treated as a FERROR
//
// When endptr is NULL, trailing characters (after optional whitespace) are 
// considered an FERROR.  When endptr is non-NULL, for a FSUCCESS conversion,
// it points to the characters after the optional trailing whitespace.
// Errors:
//	FSUCCESS - successful conversion, *endptr points to 1st char after value
//	FERROR - invalid contents, non-numeric
//	FINVALID_SETTING - value out of range
//	FINVALID_PARAMETER - invalid function arguments (NULL value or str)
FSTATUS StringToInt64(int64 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace)
{
	char *end = NULL;
	int64 temp;

	if (! str || ! value)
		return FINVALID_PARAMETER;
	errno = 0;
	temp = strtoll(str, &end, base?base:10);
	if ( ! base && ! ((temp == IB_INT64_MAX || temp == IB_INT64_MIN) && errno)
		&& (end && temp == 0 && *end == 'x' && end != str)) {
		// try again as base 16
		temp = strtoll(str, &end, 16);
	}
	if (((temp == IB_INT64_MAX || temp == IB_INT64_MIN) && errno)
		|| (end && end == str)) {
		if (errno == ERANGE)
			return FINVALID_SETTING;
		else
			return FERROR;
	}

	// skip whitespace
	if (end && skip_trail_whitespace) {
		while (isspace(*end)) {
			end++;
		}
	}
	if (endptr)
		*endptr = end;
	else if (end && *end != '\0')
		return FERROR;
	*value = temp;
	return FSUCCESS;
}

FSTATUS StringToInt32(int32 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace)
{
	int64 temp;
	FSTATUS status;

	status = StringToInt64(&temp, str, endptr, base, skip_trail_whitespace);
	if (status != FSUCCESS)
		return status;
	if (temp < IB_INT32_MIN || temp > IB_INT32_MAX)
		return FINVALID_SETTING;
	*value = (int32)temp;
	return FSUCCESS;
}

FSTATUS StringToInt16(int16 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace)
{
	int64 temp;
	FSTATUS status;

	status = StringToInt64(&temp, str, endptr, base, skip_trail_whitespace);
	if (status != FSUCCESS)
		return status;
	if (temp < IB_INT16_MIN || temp > IB_INT16_MAX)
		return FINVALID_SETTING;
	*value = (int16)temp;
	return FSUCCESS;
}

FSTATUS StringToInt8(int8 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace)
{
	int64 temp;
	FSTATUS status;

	status = StringToInt64(&temp, str, endptr, base, skip_trail_whitespace);
	if (status != FSUCCESS)
		return status;
	if (temp < IB_INT8_MIN || temp > IB_INT8_MAX)
		return FINVALID_SETTING;
	*value = (int8)temp;
	return FSUCCESS;
}

// GID of form hValue:lValue
// values must be base 16, as such 0x prefix is optional for both hValue and
// lValue
// whitespace is permitted before and after :
FSTATUS StringToGid(uint64 *hValue, uint64 *lValue, const char* str, char **endptr, boolean skip_trail_whitespace)
{
	FSTATUS status;
	char *end;

	status = StringToUint64(hValue, str, &end, 16, TRUE);
	if (status != FSUCCESS)
		return status;
	if (end == NULL  || *end != ':')
		return FERROR;
	end++;
	return StringToUint64(lValue, end, endptr, 16, skip_trail_whitespace);
}

// VESWPort of form guid:port:index or desc:port:index
// guid must be base 16, as such 0x prefix is optional.
// desc max size is MAX_VFABRIC_NAME (64)
// port and index are decimal
// whitespace is permitted before and after :
// byname = 0: the format is guid:port:index
// byname = 1: the format is desc:port:index
FSTATUS StringToVeswPort(uint64 *guid, char *desc, uint32 *port, uint32 *index,
	const char* str, char **endptr, boolean skip_trail_whitespace,
	boolean byname)
{
	FSTATUS status = FSUCCESS;
	char *end, *string = NULL;
	char *name;

	if (byname) {
		string = strdup(str);

		name = strtok_r((char *)string, ":", &end);
		if (name != NULL && strlen(name) <= MAX_VFABRIC_NAME && end != NULL) {
			strcpy(desc, name);
		} else {
			status = FERROR;
			goto out;
		}
	} else {
		status = StringToUint64(guid, str, &end, 16, TRUE);
		if (status != FSUCCESS || end == NULL || *end != ':') {
			status = FERROR;
			goto out;
		}
		end++;
	}

	status = StringToUint32(port, end, &end, 10, TRUE);
	if (status != FSUCCESS || end == NULL || *end != ':') {
		status = FERROR;
		goto out;
	}
	end++;
	status = StringToUint32(index, end, endptr, 10, skip_trail_whitespace);
out:
	if (string != NULL) {
		free(string);
	}
	return status;
}


// MAC Address of form %02x:%02x:%02x:%02x:%02x:%02x
// values must be base16, 0x prefix is optional
FSTATUS StringToMAC(uint8_t *MAC,const char *str, char **endptr,
					boolean skip_trail_whitespace)
{
	FSTATUS status;
	char *end = NULL;

	status = StringToUint8(&MAC[0], str, &end, 16, FALSE);
	if (status != FSUCCESS)
		return status;
	if ((end == NULL) || (*end != ':'))
		return FERROR;
	end++;

	status = StringToUint8(&MAC[1], end, &end, 16, FALSE);
	if (status != FSUCCESS)
		return status;
	if ((end == NULL) || (*end != ':'))
		return FERROR;
	end++;

	status = StringToUint8(&MAC[2], end, &end, 16, FALSE);
	if (status != FSUCCESS)
		return status;
	if ((end == NULL) || (*end != ':'))
		return FERROR;
	end++;

	status = StringToUint8(&MAC[3], end, &end, 16, FALSE);
	if (status != FSUCCESS)
		return status;
	if ((end == NULL) || (*end != ':'))
		return FERROR;
	end++;

	status = StringToUint8(&MAC[4], end, &end, 16, FALSE);
	if (status != FSUCCESS)
		return status;
	if ((end == NULL) || (*end != ':'))
		return FERROR;
	end++;

	status = StringToUint8(&MAC[5], end, endptr, 16, FALSE);

	return status;
}


// Byte Count as an integer followed by an optional suffix of:
// K, KB, M, MB, G or GB
// (K==KB, etc)
// converted to an absolute number of bytes
FSTATUS StringToUint64Bytes(uint64 *value, const char* str, char **endptr, int base, boolean skip_trail_whitespace)
{
	char *end;
	FSTATUS status;
	uint64 temp;

	status = StringToUint64(&temp, str, &end, base, skip_trail_whitespace);
	if (status != FSUCCESS)
		return status;
	if (end) {
		char *units = end;
		// skip whitespace
		while (isspace(*units)) {
			units++;
		}
		// parse optional units
		if (strncmp(units, "KB",2) == 0) {
			temp *= 1024;
			end = units+2;
		} else if (strncmp(units, "K",1) == 0) {
			temp *= 1024;
			end = units+1;
		} else if (strncmp(units, "MB",2) == 0) {
			temp *= 1024*1024;
			end = units+2;
		} else if (strncmp(units, "M",1) == 0) {
			temp *= 1024*1024;
			end = units+1;
		} else if (strncmp(units, "GB",2) == 0) {
			temp *= (1024*1024*1024ULL);
			end = units+2;
		} else if (strncmp(units, "G",1) == 0) {
			temp *= (1024*1024*1024ULL);
			end = units+1;
		}
	}
	
	// skip whitespace
	if (end && skip_trail_whitespace) {
		while (isspace(*end)) {
			end++;
		}
	}
	if (endptr)
		*endptr = end;
	else if (end && *end != '\0')
		return FERROR;
	*value = temp;
	return FSUCCESS;
}

#if !defined(VXWORKS)
#define RELATIVE_TIME_STR_LEN 3
                                                     // June 6th, 2015 11:59:59 PM
													 // as represented by the format
static const char * date_formats[] = {"%Y-%m-%d %T", // 2015-06-30 23:59:59
                                      "%Y-%m-%d %R", // 2015-06-30 23:59
                                      "%Y-%m-%d",    // 2015-06-30
                                      "%m/%d/%Y %T", // 06/30/2015 23:59:59
                                      "%m/%d/%Y %R", // 06/30/2015 23:59
                                      "%m/%d/%Y",    // 06/30/2015
                                      "%d.%m.%Y %T", // 30.06.2015 23:59:59
                                      "%d.%m.%Y %R", // 30.06.2015 23:59
                                      "%d.%m.%Y",    // 30.06.2015
                                                     // %x, %X are locale specific
                                      "%x %X",       // e.g. 30/06/15 23:59:59 or 30.06.2015 23.59.59, etc.
                                      "%x",          // e.g. 30/06/15, 06/30/15, etc.
                                      "%X",          // e.g. 23:59:59, 23.59.59, etc.
                                      "%R"};         // 23:59
static const char * time_units[] = {"seconds", "minutes", "hours", "days"};

/**
 * StringToDateTime - parse a string to return a date/time
 *
 * @param value  - will contain date as # of seconds since epoch if parsing successful
 * @param str    - the string representation of the date to be parsed
 *
 * @return - FSTATUS indicating success or failure of parsing:
 * 			 FSUCCESS - successful parsing
 * 			 FINVALID_PARAMETER - str not valid date/time or not in valid format
 * 			 FINVALID_SETTING - relative value out of range
 * 			 FINSUFFICIENT_MEMORY - memory could not be allocated to parse string
 * 			 FERROR - other error parsing string
 */
FSTATUS StringToDateTime(uint32 *value, const char* str){
	int i, len;
	char * return_str;
	struct tm tm = {0};
	uint32 seconds;
	FSTATUS status;

	//try to match input to one of known formats from above
	len = sizeof(date_formats) / sizeof(const char *);
	for (i = 0; i < len; ++i){
		memset(&tm, 0, sizeof(struct tm));
		return_str = strptime(str, date_formats[i], &tm);
		// want to match an exact format, meaning strptime returns null byte
		if (return_str != NULL && *return_str == '\0'){
			break;
		}
	}

	if (return_str != NULL && *return_str== '\0'){ // possible valid date entered
		struct tm tm2;
		time_t timer;
		if (i >= len - 2){
			// strptime matched against %X or %R, meaning we have a time
			// but no information on the date. Fill in tm with today's date
			time(&timer);   //get current time
			struct tm * tmp;
			tmp = localtime(&timer);
			if (!tmp){
				return FERROR;
			}
			tm.tm_year = tmp->tm_year;
			tm.tm_mon = tmp->tm_mon;
			tm.tm_mday = tmp->tm_mday;
			tm.tm_wday = tmp->tm_wday;
			tm.tm_yday = tmp->tm_yday;
		}
		// mktime will normalize fields of tm if they are outside their valid range,
		// so make a copy before the mktime call and compare before and after. If the
		// days of the month are not equal then the date enetered was not valid
		memcpy(&tm2, &tm, sizeof(struct tm));
		tm.tm_isdst = -1; //signal mktime to determine if daylight saving in effect
		timer = mktime(&tm);
		if (timer != -1){
			//check if input time was a valid date, e.g. not 02/30/YYYY
			if (tm.tm_mday != tm2.tm_mday){
				return FINVALID_PARAMETER;
			}else{
				*value = (uint32) timer;
				return FSUCCESS;
			}
		}
	}else{
		// determine if time entered in the form
		// "<x> <units> ago" where units is a member of time_units from above
		char *string, *saveptr, *token, *tokens[RELATIVE_TIME_STR_LEN];

		string = strdup(str);
		if (!string){
			return FINSUFFICIENT_MEMORY;
		}

		//split string by space (" ")
		token = strtok_r(string, " ", &saveptr);
		i = 0;
		while (token != NULL){
			if (i >= RELATIVE_TIME_STR_LEN){
				//too many tokens
				free(string);
				return FERROR;
			}
			tokens[i] = token;
			++i;
			token = strtok_r(NULL, " ", &saveptr);
		}

		//check if proper number of tokens
		if (i != RELATIVE_TIME_STR_LEN){
			free(string);
			return FINVALID_PARAMETER;
		}

		//check if first token is a valid base 10 number
		status = StringToUint32(&seconds, tokens[0], NULL, 10, TRUE);
		if (FSUCCESS != status){
			free(string);
			return status;
		}

		//check if second token in accepted units of time
		len = sizeof(time_units) / sizeof (const char *);
		for (i = 0; i < len; ++i){
			if (strcasecmp(time_units[i], tokens[1]) == 0 || strncasecmp(time_units[i], tokens[1], strlen(time_units[i]) - 1) == 0){
				break;
			}
		}

		//convert first token to seconds if necessary
		if (i >= len){
			// could not determine units
			free(string);
			return FERROR;
		}

		switch (i){
		case 0:
			break;
		case 1: //convert from minutes to seconds
			seconds = seconds * 60;
			break;
		case 2: //convert from hours to seconds
			seconds = seconds * 60 * 60;
			break;
		case 3: //convert from days to seconds
			seconds = seconds * 24 * 60 * 60;
		}

		//calculate absolute time to query for
		time_t timer;
		time(&timer);
		*value = (uint32)timer - seconds;

		free(string);
		return FSUCCESS;
	}

	return FERROR;
}

// Tuple of form select:comparator:argument
// select must be one of "utilization", "pktrate", "integrity", "congestion", "smacongestion", "bubbles", "security", or "routing".
// comparator must be one of "GT", "LT", "GE", "LE".
// argument may be any 64-bit value
// string inputs are case insensitive.
FSTATUS StringToTuple(uint32 *select, uint8 *comparator, uint64 *argument, char* str, char **endptr)
{
	FSTATUS status = FSUCCESS;
	char *end;
	char *selectstr;
	char *comparatorstr;
	char *argumentstr;

	if ((select == NULL) || (comparator == NULL) || (argument == NULL) || (str == NULL))
		return FERROR;

	selectstr = strtok_r(str, ":", &end);

	if (selectstr == NULL)
		return FERROR;

	if (strcasecmp(selectstr, "utilization") == 0)
		*select = STL_PA_SELECT_UTIL_HIGH;
	else if (strcasecmp(selectstr, "pktrate") == 0)
		*select = STL_PA_SELECT_UTIL_PKTS_HIGH;
	else if (strcasecmp(selectstr, "integrity") == 0)
		*select = STL_PA_SELECT_CATEGORY_INTEG;
	else if (strcasecmp(selectstr, "congestion") == 0)
		*select = STL_PA_SELECT_CATEGORY_CONG;
	else if (strcasecmp(selectstr, "smacongestion") == 0)
		*select = STL_PA_SELECT_CATEGORY_SMA_CONG;
	else if (strcasecmp(selectstr, "bubbles") == 0)
		*select = STL_PA_SELECT_CATEGORY_BUBBLE;
	else if (strcasecmp(selectstr, "security") == 0)
		*select = STL_PA_SELECT_CATEGORY_SEC;
	else if (strcasecmp(selectstr, "routing") == 0)
		*select = STL_PA_SELECT_CATEGORY_ROUT;
	else
		return FERROR;

	comparatorstr = strtok_r((char *)NULL, ":", &end);

	if (comparatorstr == NULL)
		return FERROR;

	if (strcasecmp(comparatorstr, "GT") == 0)
		*comparator = FOCUS_PORTS_COMPARATOR_GREATER_THAN;
	else if (strcasecmp(comparatorstr, "LT") == 0)
		*comparator = FOCUS_PORTS_COMPARATOR_LESS_THAN;
	else if (strcasecmp(comparatorstr, "GE") == 0)
		*comparator = FOCUS_PORTS_COMPARATOR_GREATER_THAN_OR_EQUAL;
	else if (strcasecmp(comparatorstr, "LE") == 0)
		*comparator = FOCUS_PORTS_COMPARATOR_LESS_THAN_OR_EQUAL;
	else
		return FERROR;

	argumentstr = strtok_r((char *)NULL, ":", &end);

	if (argumentstr) {
		status = StringToUint64(argument, argumentstr, NULL, 0, TRUE);
		return status;
	}

	return FERROR;
}

#endif

#if !defined(VXWORKS)
#define MEMORY_ALLOCATE_PRIV(size, flags, tag) MemoryAllocatePriv(size, flags, tag)
#define MEMORY_ALLOCATE_PHYS_CONT_PRIV(size) MemoryAllocatePhysContPriv(size)
#define MEMORY_DEALLOCATE_PHYS_CONT_PRIV(size) MemoryDeallocatePhysContPriv(size)
#define MEMORY_DEALLOCATE_PRIV(ptr) MemoryDeallocatePriv(ptr)
#else
#define MEMORY_ALLOCATE_PRIV(size, flags, tag) MemoryAllocatePriv(size, __builtin_return_address(0))
#define MEMORY_ALLOCATE_PHYS_CONT_PRIV(size) MemoryAllocatePhysContPriv(size, __builtin_return_address(0))
#define MEMORY_DEALLOCATE_PHYS_CONT_PRIV(size) MemoryDeallocatePhysContPriv(size, __builtin_return_address(0))
#define MEMORY_DEALLOCATE_PRIV(ptr) MemoryDeallocatePriv(ptr, __builtin_return_address(0))
#endif

#if defined(MEM_TRACK_ON)
//
// Destroy the memory tracker object.
//
static __inline void
DestroyMemTracker( void )
{
	MEM_TRACKER *tmp;
	if( !pMemTracker )
		return;

	tmp = pMemTracker;
	pMemTracker = NULL; /* so no one uses it while we're destroying it */

	// Destory all objects in the memory tracker object.
	QListDestroy( &tmp->FreeHrdList );
	SpinLockDestroy( &tmp->Lock );
	QListDestroy( &tmp->AllocList );

	// Free the memory allocated for the memory tracker object.
	MEMORY_DEALLOCATE_PRIV( tmp );
}

//
// Allocate and initialize the memory tracker object.
//
static __inline boolean
CreateMemTracker( void )
{
	MEM_TRACKER *tmp;

	if( pMemTracker )
		return TRUE;

	// Allocate the memory tracker object. Don't update global until we're done
	tmp = (MEM_TRACKER*)MEMORY_ALLOCATE_PRIV( sizeof(MEM_TRACKER), IBA_MEM_FLAG_LEGACY, TRK_TAG );

	if( !tmp )
		return FALSE;

	// Pre-initialize all objects in the memory tracker object.
	QListInitState( &tmp->AllocList );
	SpinLockInitState( &tmp->Lock );
	QListInitState( &tmp->FreeHrdList );

	// Initialize the list.
	if( !QListInit( &tmp->AllocList ) )
	{
		/* global isn't initialize, don't call Destroy func; do the clean up */
		MEMORY_DEALLOCATE_PRIV( tmp );
		return FALSE;
	}

	// Initialize the spin lock to protect list operations.
	if( !SpinLockInit( &tmp->Lock ) )
	{
		/* global isn't initialize, don't call Destroy func; do the clean up */
		QListDestroy( &tmp->AllocList );
		SpinLockDestroy( &tmp->Lock );
		MEMORY_DEALLOCATE_PRIV( tmp );
		return FALSE;
	}

	// Initialize the free list.
	if( !QListInit( &tmp->FreeHrdList ) )
	{
		/* global isn't initialize, don't call Destroy func; do the clean up */
		QListDestroy( &tmp->AllocList );
		SpinLockDestroy( &tmp->Lock );
		MEMORY_DEALLOCATE_PRIV( tmp );
		return FALSE;
	}

//	MsgOut( "\n\n\n*** Memory tracker object address = %p ***\n\n\n", tmp );
	MsgOut( "\n*** Memory tracker enabled ***\n" );

	/* NOW update the global */
	pMemTracker = tmp;

	return TRUE;
}
#endif

//
// Enables memory allocation tracking.
//
static __inline void
MemoryTrackStart( void )
{
#if defined(MEM_TRACK_ON)
	if( pMemTracker )
		return;

	CreateMemTracker();
#endif	// MEM_TRACK_ON
}


//
// Clean up memory tracking.
//
static __inline void
MemoryTrackStop( void )
{
#if defined(MEM_TRACK_ON)
	LIST_ITEM	*pListItem;

	if( !pMemTracker )
		return;

	if( QListCount( &pMemTracker->AllocList ) )
	{
		// There are still items in the list.  Print them out.
		MemoryDisplayUsage(1, 0, 0);
	} else {
		MsgOut( "\n*** Memory tracker stopped, no leaks detected ***\n" );
		MsgOut("IbAccess max allocations=%u bytes=%u\n",
						max_allocations, max_allocated);
	}

	// Free all allocated headers.
	SpinLockAcquire( &pMemTracker->Lock );
	while( (pListItem = QListRemoveHead( &pMemTracker->AllocList )) != NULL )
	{
		SpinLockRelease( &pMemTracker->Lock );
		MEMORY_DEALLOCATE_PRIV( PARENT_STRUCT( pListItem, MEM_ALLOC_HDR, ListItem ) );
		SpinLockAcquire( &pMemTracker->Lock );
	}
	while( (pListItem = QListRemoveHead( &pMemTracker->FreeHrdList )) != NULL )
	{
		SpinLockRelease( &pMemTracker->Lock );
		MEMORY_DEALLOCATE_PRIV( PARENT_STRUCT( pListItem, MEM_ALLOC_HDR, ListItem ) );
		SpinLockAcquire( &pMemTracker->Lock );
	}
	SpinLockRelease( &pMemTracker->Lock );

	DestroyMemTracker();
#endif	// MEM_TRACK_ON
}


//
// Enables memory allocation tracking.
//
void
MemoryTrackUsage( 
	IN boolean Start )
{
	if( Start )
		MemoryTrackStart();
	else
		MemoryTrackStop();
}

#if defined(MEM_TRACK_ON)
static void
MemoryTrackerShow(
		IN char *prefix, 
		IN MEM_ALLOC_HDR	*pHdr,
		IN char *suffix)
{
#if defined(VXWORKS)
	if ((int)pHdr->LineNum >= 0x00408000) {
		MsgOut( "%s%p(%u) %s ra=%p tick=%u%s\n", prefix,
				pHdr->ListItem.pObject, pHdr->Bytes, pHdr->trk->filename,
				(void *)pHdr->LineNum, pHdr->tick, suffix );
	} else 
		MsgOut( "%s%p(%u) in file %s line %d tick=%u%s\n", prefix,
				pHdr->ListItem.pObject, pHdr->Bytes, pHdr->trk->filename,
				pHdr->LineNum, pHdr->tick, suffix );
#else
		MsgOut( "%s%p(%u) in file %s line %d%s\n", prefix,
				pHdr->ListItem.pObject, pHdr->Bytes, pHdr->trk->filename,
				pHdr->LineNum, suffix );
#endif
}

static void
MemoryTrackerCheckOverrun(
		IN MEM_ALLOC_HDR	*pHdr )
{
#ifdef MEM_TRACK_FTR
	// Check that the user did not overrun his memory allocation.
	if( (pHdr->pFtr != NULL) && (pHdr->pFtr->OutOfBound != TRK_TAG) )
	{
		MemoryTrackerShow("*** User overrun detected ", pHdr, "");
	}
#endif
}

/* unlink a header from the allocated list */
/* must be called with pMemTracker->Lock held */
static void
MemoryTrackerUnlink(
		IN MEM_ALLOC_HDR	*pHdr )
{
	// Remove the item from the list.
	QListRemoveItem( &pMemTracker->AllocList, &pHdr->ListItem );

	--current_allocations;
	current_allocated -= pHdr->Bytes;
	if (pHdr->reported)
		MemoryTrackerShow("", pHdr, " FREED");
	MemoryTrackerDereference(pHdr->trk);
	// Return the header to the free header list.
	QListInsertHead( &pMemTracker->FreeHrdList, &pHdr->ListItem );
}
#endif	// MEM_TRACK_ON

//
// Display memory usage.
//
void
MemoryDisplayUsage( int method, uint32 minSize, uint32 minTick )
{
#if defined(MEM_TRACK_ON)
	uint32 allocated = 0;
	uint32 allocations = 0;
	MEM_ALLOC_HDR	*pHdr;
	LIST_ITEM *item, *next, *tail, *head;
	unsigned int allocations_per_sec = 0;
	uint32 currentTime;
	boolean all = (method == 1);

	if( !pMemTracker ) {
		MsgOut( "*** IbAccess Memory Tracking is disabled ***\n" );
		return;
	}

	/* "lock" present allocations by setting
	 * displaying flag, so other allocates/frees will not affect them
	 * This gives us a snapshot while permitting the system to run
	 * while we perform the output (the output itself may use memory allocate)
	 * However, our report loop below must stay within head/tail
	 */
	SpinLockAcquire( &pMemTracker->Lock );
	tail = QListTail(&pMemTracker->AllocList);
	head = QListHead(&pMemTracker->AllocList);
	for(item = head; item != NULL; item = QListNext(&pMemTracker->AllocList, item)) {
		pHdr = PARENT_STRUCT( item, MEM_ALLOC_HDR, ListItem );
		pHdr->displaying = TRUE;
	}
	SpinLockRelease (&pMemTracker->Lock);
	
	MsgOut( "*** IbAccess Memory Usage %s minSize=%d minTick=%d ***\n", all?"All":"Unreported", minSize, minTick );

	if (head && tail) {
		item = head;
		do {
			next = QListNext(&pMemTracker->AllocList, item);
			pHdr = PARENT_STRUCT( item, MEM_ALLOC_HDR, ListItem );
	
	#ifdef MEM_TRACK_FTR
			// Check that the user did not overrun his memory allocation.
			if (pHdr->deallocate == FALSE) {
				MemoryTrackerCheckOverrun(pHdr);
			}
	#endif	// MEM_TRACK_FTR
			if ((pHdr->Bytes >= minSize) && (pHdr->tick >= minTick) && (all || (pHdr->reported == 0))) {
				// method 2 just marks all current allocations as reported, without actually reporting them
				// method 3 displays the items without changing their reported state (allows us to avoid the FREED messages)
				if (method != 2)
					MemoryTrackerShow("", pHdr, "");
				if (method != 3)
					pHdr->reported = 1;
			}
			allocated += pHdr->Bytes;
			++allocations;
			SpinLockAcquire( &pMemTracker->Lock );
			pHdr->displaying = FALSE;
			if (pHdr->deallocate) {
				MemoryTrackerUnlink(pHdr);
			}
			SpinLockRelease (&pMemTracker->Lock);
			item = next;
		} while (&pHdr->ListItem != tail && item != NULL);
	}
	currentTime = GetTimeStampSec();
	if (last_reported_secs && currentTime != last_reported_secs) {
		allocations_per_sec = (total_allocations - last_reported_allocations) / (currentTime - last_reported_secs);
	}
	last_reported_secs = currentTime;
	last_reported_allocations = total_allocations;
	MsgOut("IbAccess current allocations=%u bytes=%u max allocations=%u bytes=%u p/s=%d\n",
			allocations, allocated, max_allocations, max_allocated, allocations_per_sec);
#endif	// MEM_TRACK_ON
}


#if defined(MEM_TRACK_ON)
unsigned int hashValue(const char *key) {
	unsigned int  nHash = 0;
	while (*key)
		nHash = (nHash<<5) + nHash + *key++;
	return nHash;
}

static MemoryTrackerFileName_t *MemoryTrackerFileNameLookup(const char *filename, unsigned int *hash) {
	unsigned int hashVal;
	MemoryTrackerFileName_t *trk;
	int len = strlen(filename);

	hashVal = hashValue(filename) % MEMORY_TRACKER_BUCKETS;
	*hash = hashVal;

	for(trk = MemoryTrackerBuckets[hashVal]; trk != NULL; trk = trk->next) {
		if (trk->filenameLen == len) {
			if (memcmp(&trk->filename[0], filename, len) == 0) {
				return trk;
			}
		}
	}
	return NULL;
}

static MemoryTrackerFileName_t *MemoryTrackerFileNameAlloc(const char *filename, int filenameLen, unsigned int hash) {
	MemoryTrackerFileName_t *trk;

	trk = (MemoryTrackerFileName_t*)MEMORY_ALLOCATE_PRIV(
					 	sizeof( MemoryTrackerFileName_t ) + filenameLen + 1,
						IBA_MEM_FLAG_LEGACY, TRK_TAG );
	if (trk != NULL) {
		trk->referenceCount = 1;
		trk->filenameLen = filenameLen;
		memcpy(&trk->filename, filename, filenameLen + 1);
		trk->next = MemoryTrackerBuckets[hash];
		MemoryTrackerBuckets[hash] = trk;
		// MsgOut("Added len=%d name=(%p)%s\n", filenameLen, trk->filename, trk->filename);
	}
	return trk;
}

static MemoryTrackerFileName_t *MemoryTrackerReference(const char *filename) {
	MemoryTrackerFileName_t *trk;
	int len = strlen(filename);
	unsigned int hash;

	trk = MemoryTrackerFileNameLookup(filename, &hash);
	if (trk == NULL) {
		trk = MemoryTrackerFileNameAlloc(filename, len, hash);
		if (trk == NULL)
			return NULL;
	} else {
		++trk->referenceCount;
	}
	return trk;
}

static void MemoryTrackerDereference(MemoryTrackerFileName_t *trk) {
	if (trk == NULL) {
		MsgOut("Could not find reference to trk=%p\n", trk);
	} else {
		--trk->referenceCount;
	}
}

static void
MemoryTrackerTrackAllocation(
	IN const char *pFileName, 
	IN int32 nLine, 
	IN uint32 Bytes, 
	IN uint32 flags,
	IN void *pMem,
	IN MEM_ALLOC_FTR *pFtr)
{
	MEM_ALLOC_HDR	*pHdr;
	LIST_ITEM		*pListItem;

#ifdef MEM_TRACK_FTR
	if (pFtr)
		pFtr->OutOfBound = TRK_TAG;
#endif  // MEM_TRACK_FTR

	if( !pMemTracker ) {
		if (unTindex < UNTRACKED_COUNT)
			unTAddr[unTindex++] = pMem;
		else {
			static int firsttime=1;
			if (firsttime) {
				MsgOut("***** Untracked memory allocation array LIMIT:%d REACHED; check unTmissed value to calculate new size\n", UNTRACKED_COUNT);
				firsttime = 0;
			}
			unTmissed++; /* so we know how much more to expand the array */
		}
		return;
	}

	// Get a header from the free header list.
	SpinLockAcquire( &pMemTracker->Lock );
	pListItem = QListRemoveHead( &pMemTracker->FreeHrdList );
	SpinLockRelease( &pMemTracker->Lock );

	if( pListItem )
	{
		// Set the header pointer to the header retrieved from the list.
		pHdr = PARENT_STRUCT( pListItem, MEM_ALLOC_HDR, ListItem );
	}
	else
	{
		// We failed to get a free header.  Allocate one.
		// we can prempt if caller allows it, however we do not want
		// pageable, nor short duration memory
		pHdr = (MEM_ALLOC_HDR*)MEMORY_ALLOCATE_PRIV( sizeof( MEM_ALLOC_HDR ),
								flags & IBA_MEM_FLAG_PREMPTABLE, TRK_TAG );
		if( !pHdr )
		{
			// We failed to allocate the header, don't track this allocate
			return;
		}
	}
	pHdr->LineNum = nLine;
	pHdr->tick = TICK;
	pHdr->Bytes = Bytes;
	pHdr->reported = 0;
	pHdr->displaying = FALSE;
	pHdr->deallocate = FALSE;
	// We store the pointer to the memory returned to the user.  This allows
	// searching the list of allocated memory even if the buffer allocated is 
	// not in the list without dereferencing memory we do not own.
	pHdr->ListItem.pObject = pMem;

#ifdef MEM_TRACK_FTR
	pHdr->pFtr = pFtr;
#else
	pHdr->pFtr = NULL;
#endif  // MEM_TRACK_FTR

	SpinLockAcquire( &pMemTracker->Lock );
	pHdr->trk = MemoryTrackerReference(pFileName);
	++total_allocations;
	if (++current_allocations > max_allocations)
		max_allocations = current_allocations;
	if ((current_allocated += pHdr->Bytes) > max_allocated)
		max_allocated = current_allocated;

	// Insert the header structure into our allocation list.
	QListInsertTail( &pMemTracker->AllocList, &pHdr->ListItem );
	SpinLockRelease( &pMemTracker->Lock );

	return;
}

int
MemoryTrackerTrackDeallocate( 
	IN void *pMemory )
{
	MEM_ALLOC_HDR	*pHdr;
	LIST_ITEM		*pListItem;
	int				result = 0;

	if( pMemTracker )
	{
		SpinLockAcquire( &pMemTracker->Lock );

		// Removes an item from the allocation tracking list given a pointer
		// To the user's data and returns the pointer to header referencing the
		// allocated memory block.
		pListItem = 
			QListFindFromTail( &pMemTracker->AllocList, NULL, pMemory );

		if( pListItem )
		{
			// Get the pointer to the header.
			pHdr = PARENT_STRUCT( pListItem, MEM_ALLOC_HDR, ListItem );
#ifdef MEM_TRACK_FTR
			MemoryTrackerCheckOverrun(pHdr);
#endif	// MEM_TRACK_FTR

			if (pHdr->displaying) {
				pHdr->deallocate = TRUE;
			} else {
				// Remove the item from the list.
				MemoryTrackerUnlink(pHdr);
			}
		} else {
			int ii, found; 
			for (ii=0, found=0; ii<unTindex; ii++) {
				if (unTAddr[ii] == pMemory) {
					int nextii = ii+1;
					/* move up the remaining entries */
					if (nextii < unTindex) {
						memcpy(&unTAddr[ii], &unTAddr[nextii], 
							(unTindex - nextii)*sizeof(void));
					}
					/* decrease entry count */
					unTindex--;
					/* zero out the new free location */
					unTAddr[unTindex] = 0;
					found = 1;
					break;
				}
			}
			if (!found) {
				result = 1;
#if defined(VXWORKS)
				MsgOut( "UNMATCHED FREE %p ra=%p %p %p\n", pMemory, __builtin_return_address(0), __builtin_return_address(1), __builtin_return_address(2));
				/* DumpStack fails in to find Osa_SysThread object in Osa_SysThread::FindThread. Until that is fixed skip dumping the stack. */
#else
				MsgOut( "BAD FREE %p\n", pMemory);
				DumpStack();
#endif
			}
		}
		SpinLockRelease( &pMemTracker->Lock );
	}
	return result;
}

//
// Allocates memory and stores information about the allocation in a list.
// The contents of the list can be printed out by calling the function
// "MemoryReportUsage".  Memory allocation will succeed even if the list 
// cannot be created.
//
void*
MemoryAllocateDbg(
	IN const char *pFileName, 
	IN int32 nLine, 
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag )
{
	return MemoryAllocate2Dbg(pFileName, nLine, Bytes,
					(IsPageable?IBA_MEM_FLAG_PAGEABLE:IBA_MEM_FLAG_NONE)
					|IBA_MEM_FLAG_LEGACY, Tag);
}

#if defined(VXWORKS)
void
MemoryAllocateVxWorksTrack(
	IN void *result,
	IN uint32 Bytes,
	IN char *reason,
	IN void *caller) 
{
	MemoryTrackerTrackAllocation(reason, (int)caller, Bytes, IBA_MEM_FLAG_NONE, result, NULL);
}
#endif

void*
MemoryAllocate2Dbg(
	IN const char *pFileName, 
	IN int32 nLine, 
	IN uint32 Bytes, 
	IN uint32 flags,
	IN uint32 Tag )
{
	void			*pMem;

#ifdef MEM_TRACK_FTR
	// Increase the size of our allocation to account for the footer.
	Bytes += sizeof( MEM_ALLOC_FTR );
	Bytes = (Bytes + 3) >> 2 << 2;
#endif  // MEM_TRACK_FTR

	// Allocate the memory first, so that we give the user's allocation 
	// priority over the the header allocation.
	pMem = MEMORY_ALLOCATE_PRIV( Bytes, flags, Tag );

	if( !pMem )
		return NULL;

	MemoryTrackerTrackAllocation(pFileName, nLine, Bytes, flags, pMem,
#ifdef MEM_TRACK_FTR
			(MEM_ALLOC_FTR*)((uchar*)pMem + Bytes - sizeof( MEM_ALLOC_FTR ))
#else
			NULL
#endif
			);

	return pMem;
}

void*
MemoryAllocateRel(
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag )
{
	return MemoryAllocateDbg( "Unknown", 0, Bytes, IsPageable, Tag );
}
void*
MemoryAllocate2Rel(
	IN uint32 Bytes, 
	IN uint32 flags,
	IN uint32 Tag )
{
	return MemoryAllocate2Dbg( "Unknown", 0, Bytes, flags, Tag );
}

#if defined(VXWORKS)
void*
MemoryAllocatePhysContDbg(
	IN const char *pFileName, 
	IN int32 nLine, 
	IN uint32 Bytes )
{
	void			*pMem;

	/* no footer on PhysCont allocates, they tend to be a full page
	 * and a footer would waste another page, TBD we could round up
	 * provided resulting Bytes was still same number of pages
	 */

	// Allocate the memory first, so that we give the user's allocation 
	// priority over the the header allocation.
	pMem = MEMORY_ALLOCATE_PHYS_CONT_PRIV( Bytes );

	if( !pMem )
		return NULL;

	MemoryTrackerTrackAllocation(pFileName, nLine, Bytes, IBA_MEM_FLAG_PREMPTABLE, pMem, NULL);

	return pMem;
}

void*
MemoryAllocatePhysContRel(
	IN uint32 Bytes )
{
	return MemoryAllocatePhysContDbg( "Unknown", 0, Bytes );
}
#endif /* defined(VXWORKS) */
#else	// !MEM_TRACK_ON
void*
MemoryAllocateDbg(
	IN const char *pFileName, 
	IN int32 nLine, 
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag )
{
	return MEMORY_ALLOCATE_PRIV( Bytes,
					(IsPageable?IBA_MEM_FLAG_PAGEABLE:IBA_MEM_FLAG_NONE)
					|IBA_MEM_FLAG_LEGACY, Tag );
}

void*
MemoryAllocate2Dbg(
	IN const char *pFileName, 
	IN int32 nLine, 
	IN uint32 Bytes, 
	IN uint32 flags,
	IN uint32 Tag )
{
	return MEMORY_ALLOCATE_PRIV( Bytes, flags, Tag);
}
void*
MemoryAllocateRel(
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag )
{
	return MEMORY_ALLOCATE_PRIV( Bytes, 
					(IsPageable?IBA_MEM_FLAG_PAGEABLE:IBA_MEM_FLAG_NONE)
					|IBA_MEM_FLAG_LEGACY, Tag );
}

void*
MemoryAllocate2Rel(
	IN uint32 Bytes, 
	IN uint32 flags,
	IN uint32 Tag )
{
	return MEMORY_ALLOCATE_PRIV( Bytes, flags, Tag);
}

#if defined(VXWORKS)
void*
MemoryAllocatePhysContDbg(
	IN const char *pFileName, 
	IN int32 nLine, 
	IN uint32 Bytes )
{
	return MEMORY_ALLOCATE_PHYS_CONT_PRIV( Bytes );
}

void*
MemoryAllocatePhysContRel(
	IN uint32 Bytes )
{
	return MEMORY_ALLOCATE_PHYS_CONT_PRIV( Bytes );
}
#endif /* defined(VXWORKS) */
#endif	// MEM_TRACK_ON


#if defined(MEM_TRACK_ON)
void*
MemoryAllocateAndClearDbg( 
	IN const char *pFileName, 
	IN int32 nLine,
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag )
{
	return MemoryAllocate2AndClearDbg(pFileName, nLine, Bytes,
					(IsPageable?IBA_MEM_FLAG_PAGEABLE:IBA_MEM_FLAG_NONE)
					|IBA_MEM_FLAG_LEGACY, Tag );
}
void*
MemoryAllocate2AndClearDbg( 
	IN const char *pFileName, 
	IN int32 nLine,
	IN uint32 Bytes, 
	IN uint32 flags,
	IN uint32 Tag )
{
	void	*pBuffer;

	if( (pBuffer = MemoryAllocate2Dbg( pFileName, nLine, Bytes, flags, Tag )) != NULL )
	{
		MemoryClear( pBuffer, Bytes );
	}

	return pBuffer;
}

void*
MemoryAllocateAndClearRel( 
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag )
{
	return MemoryAllocateAndClearDbg("Unknown", 0, Bytes, IsPageable, Tag);
}

void*
MemoryAllocate2AndClearRel( 
	IN uint32 Bytes, 
	IN uint32 flags,
	IN uint32 Tag )
{
	return MemoryAllocate2AndClearDbg("Unknown", 0, Bytes, flags, Tag);
}

#else	// !MEM_TRACK_ON
void*
MemoryAllocateAndClearDbg( 
	IN const char *pFileName, 
	IN int32 nLine,
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag )
{
	return MemoryAllocate2AndClear(Bytes,
					(IsPageable?IBA_MEM_FLAG_PAGEABLE:IBA_MEM_FLAG_NONE)
					|IBA_MEM_FLAG_LEGACY, Tag );
}

void*
MemoryAllocate2AndClearDbg( 
	IN const char *pFileName, 
	IN int32 nLine,
	IN uint32 Bytes, 
	IN uint32 flags, 
	IN uint32 Tag )
{
	return MemoryAllocate2AndClear(Bytes, flags, Tag);
}

void*
MemoryAllocateAndClearRel( 
	IN uint32 Bytes, 
	IN boolean IsPageable, 
	IN uint32 Tag )
{
	return MemoryAllocate2AndClear(Bytes,
					(IsPageable?IBA_MEM_FLAG_PAGEABLE:IBA_MEM_FLAG_NONE)
					|IBA_MEM_FLAG_LEGACY, Tag );
}

void*
MemoryAllocate2AndClearRel( 
	IN uint32 Bytes, 
	IN uint32 flags,
	IN uint32 Tag )
{
	void	*pBuffer;

	if( (pBuffer = MEMORY_ALLOCATE_PRIV( Bytes, flags, Tag )) != NULL )
	{
		MemoryClear( pBuffer, Bytes );
	}

	return pBuffer;
}
#endif	// !MEM_TRACK_ON


int
MemoryDeallocate( 
	IN void *pMemory )
{
#if defined(MEM_TRACK_ON)
	int result;

	if (pMemory == NULL)
		return 0;

	result = MemoryTrackerTrackDeallocate(pMemory );
	MEMORY_DEALLOCATE_PRIV( pMemory );
	return result;
#else
	MEMORY_DEALLOCATE_PRIV( pMemory );
	return 0;
#endif	// MEM_TRACK_ON

}

#if defined(VXWORKS)
void
MemoryDeallocatePhysCont( 
	IN void *pMemory )
{
#if defined(MEM_TRACK_ON)
	(void)MemoryTrackerTrackDeallocate(pMemory );
#endif	// MEM_TRACK_ON
	MEMORY_DEALLOCATE_PHYS_CONT_PRIV( pMemory );
}
#endif /* defined(VXWORKS) */

void
MemoryClear( 
	IN void *pMemory, 
	IN uint32 Bytes )
{
	MemoryFill( pMemory, 0, Bytes );
}


#if defined(MEM_TRACK_ON)
void*
MemoryAllocateObjectArrayRel(
	IN uint32 ObjectCount, 
	IN OUT uint32 *pObjectSize,  
	IN uint32 ByteAlignment, 
	IN uint32 AlignmentOffset,
	IN boolean IsPageable, 
	IN uint32 Tag,
	OUT void **ppFirstObject, 
	OUT uint32 *pArraySize )
{
	return MemoryAllocateObjectArrayDbg("Unknown", 0, ObjectCount, pObjectSize,
						ByteAlignment, AlignmentOffset, IsPageable, Tag,
						ppFirstObject, pArraySize);
}

void*
MemoryAllocateObjectArrayDbg(
	IN const char *pFileName, 
	int32 nLine,
	IN uint32 ObjectCount, 
	IN OUT uint32 *pObjectSize,  
	IN uint32 ByteAlignment, 
	IN uint32 AlignmentOffset,
	IN boolean IsPageable, 
	IN uint32 Tag,
	OUT void **ppFirstObject, 
	OUT uint32 *pArraySize )
#else	// !MEM_TRACK_ON
void*
MemoryAllocateObjectArrayDbg(
	IN const char *pFileName, 
	int32 nLine,
	IN uint32 ObjectCount, 
	IN OUT uint32 *pObjectSize,  
	IN uint32 ByteAlignment, 
	IN uint32 AlignmentOffset,
	IN boolean IsPageable, 
	IN uint32 Tag,
	OUT void **ppFirstObject, 
	OUT uint32 *pArraySize )
{
	return MemoryAllocateObjectArrayRel(ObjectCount, pObjectSize, ByteAlignment,
						AlignmentOffset, IsPageable, Tag,
						ppFirstObject, pArraySize);
}

void*
MemoryAllocateObjectArrayRel(
	IN uint32 ObjectCount, 
	IN OUT uint32 *pObjectSize,  
	IN uint32 ByteAlignment, 
	IN uint32 AlignmentOffset,
	IN boolean IsPageable, 
	IN uint32 Tag,
	OUT void **ppFirstObject, 
	OUT uint32 *pArraySize )
#endif	// MEM_TRACK_ON
{
	void	*pArray;

	ASSERT( ObjectCount && *pObjectSize && AlignmentOffset < *pObjectSize );

	if( ByteAlignment > 1)
	{
		// Fixup the object size based on the alignment specified.
		*pObjectSize = ((*pObjectSize) + ByteAlignment - 1) -
			(((*pObjectSize) + ByteAlignment - 1) % ByteAlignment);
	}

	// Determine the size of the buffer to allocate.
	*pArraySize = (ObjectCount * (*pObjectSize)) + ByteAlignment;

	// Allocate the array of objects.
#if defined(MEM_TRACK_ON)
	if( !(pArray = MemoryAllocateAndClearDbg( pFileName, nLine, *pArraySize, 
		IsPageable, Tag )) )
#else	// !MEM_TRACK_ON
	if( !(pArray = MemoryAllocateAndClear( *pArraySize, IsPageable, Tag )) )

#endif	// MEM_TRACK_ON
	{
		*pArraySize = 0;
		return NULL;
	}

	if( ByteAlignment > 1 )
	{
		// Calculate the pointer to the first object that is properly aligned.
		*ppFirstObject = (void*)(
			((uchar*)pArray + AlignmentOffset + ByteAlignment - 1) - 
			(((uintn)pArray + AlignmentOffset + ByteAlignment - 1) %
			ByteAlignment ));
	}
	else
	{
		*ppFirstObject = pArray;
	}
	return pArray;
}

