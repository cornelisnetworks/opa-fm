/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/
/* [ICS VERSION STRING: unknown] */

#include <iba/ipublic.h>
#include <iba/ib_sm_priv.h>
#if !defined(VXWORKS) || defined(BUILD_DMC)
#include <iba/ib_dm.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <ctype.h>
#define _GNU_SOURCE

#include "ixml.h"
#ifdef VXWORKS
#include "config_compression.h"
int             snprintf(char *str, size_t count, const char *fmt, ...);
int             vsnprintf(char *str, size_t count, const char *fmt,
                          va_list arg);
#endif

#define MYTAG MAKE_MEM_TAG('l','x', 'm', 'l')

/* should be defined in ib_dm.h */
#ifndef IOC_IDSTRING_SIZE
#define IOC_IDSTRING_SIZE 64
#endif
#ifndef IOC_SERVICE_NAME_SIZE
#define IOC_SERVICE_NAME_SIZE 40
#endif

extern void IXmlPrintFileError(const char *input_file, IXmlParserPrintMessage printError);

/****************************************************************************/
/* XML Init */

/* indent is additional indent per level */
void
IXmlInit(IXmlOutputState_t *state, FILE *file, unsigned indent,
				IXmlOutputFlags_t flags, void *context)
{
	state->file = file;
	state->indent = indent;
	state->flags = flags;
	state->cur_indent = 0;
	state->context = context;
}

/****************************************************************************/
/* XML Output */

/* indent is additional indent per level */
FSTATUS
IXmlOutputInit(IXmlOutputState_t *state, FILE *file, unsigned indent,
				IXmlOutputFlags_t flags, void *context)
{
	IXmlInit(state, file, indent, flags, context);
	IXmlOutputPrint(state, "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n");
	state->flags &= ~IXML_OUTPUT_FLAG_HAD_CONTENT;	// should not be content
	return (IXmlOutputFailed(state)?FERROR:FSUCCESS);
}

void
IXmlOutputDestroy(IXmlOutputState_t *state)
{
	state->file = NULL;	// make sure can't be used by mistake
	state->context = NULL;	// make sure can't be used by mistake
}

// output to output file
void IXmlOutputPrint(IXmlOutputState_t *state, const char *format, ...)
{
	va_list args;

	if (state->flags & IXML_OUTPUT_FLAG_IN_START_TAG) {
		fprintf(state->file, ">");
		state->flags &= ~IXML_OUTPUT_FLAG_IN_START_TAG;
	}

	va_start(args, format);

	state->flags &= ~IXML_OUTPUT_FLAG_START_NEED_NL;
	state->flags |= IXML_OUTPUT_FLAG_HAD_CONTENT;	// could be content
	vfprintf(state->file, format, args);
	va_end(args);
}

// output to output file with present indent preceeding output
void IXmlOutputPrintIndent(IXmlOutputState_t *state, const char *format, ...)
{
	va_list args;

	va_start(args, format);

	if (state->flags & IXML_OUTPUT_FLAG_IN_START_TAG) {
		fprintf(state->file, ">");
		state->flags &= ~IXML_OUTPUT_FLAG_IN_START_TAG;
	}

	if (state->flags & IXML_OUTPUT_FLAG_START_NEED_NL) {
		fprintf(state->file, "\n");
		state->flags &= ~IXML_OUTPUT_FLAG_START_NEED_NL;
	}
	state->flags &= ~IXML_OUTPUT_FLAG_HAD_CONTENT;	// should not be content
	fprintf(state->file, "%*s", state->cur_indent, "");
	vfprintf(state->file, format, args);
	va_end(args);
}

void IXmlOutputNoop(IXmlOutputState_t *state, const char *tag, void *data)
{
}

void IXmlOutputStartTag(IXmlOutputState_t *state, const char *tag)
{
	IXmlOutputPrintIndent(state, "<%s", tag);
	state->flags |= IXML_OUTPUT_FLAG_IN_START_TAG;
	state->flags |= IXML_OUTPUT_FLAG_START_NEED_NL;
	state->flags &= ~IXML_OUTPUT_FLAG_HAD_CONTENT;	// no content yet
	state->cur_indent += state->indent;
}

void IXmlOutputStartAttrTag(IXmlOutputState_t *state, const char *tag, void *data, IXML_FORMAT_ATTR_FUNC attr_func)
{
	if (attr_func) {
		IXmlOutputPrintIndent(state, "<%s", tag);
		(*attr_func)(state, data);
		IXmlOutputPrint(state, ">");
		// clear flags after attr_func in case attr_func calls OutputPrint
		state->flags |= IXML_OUTPUT_FLAG_START_NEED_NL;
		state->flags &= ~IXML_OUTPUT_FLAG_HAD_CONTENT;	// no content yet
		state->cur_indent += state->indent;
	} else {
		IXmlOutputStartTag(state, tag);
	}
}

FSTATUS IXmlOutputAttr(IXmlOutputState_t *state, const char *attr,
	const char *val)
{
	return IXmlOutputAttrFmt(state, attr, "%s", val);
}

FSTATUS IXmlOutputAttrFmt(IXmlOutputState_t *state, const char *attr,
	const char *format, ...)
{
	va_list args;

	if (!(state->flags & IXML_OUTPUT_FLAG_IN_START_TAG))
		return FERROR;

	va_start(args, format);

	fprintf(state->file, " %s=\"", attr);
	vfprintf(state->file, format, args);
	fprintf(state->file, "\"");
	va_end(args);

	return FSUCCESS;
}

void IXmlOutputEndTag(IXmlOutputState_t *state, const char *tag)
{
	if (state->cur_indent > state->indent)
		state->cur_indent -= state->indent;
	else
		state->cur_indent = 0;
	// if there was content output we can close
	// tag on same line as the content, otherwise it was a list
	// and we output end tag on a new line with indent
	// Note: output of "empty content tags" should do an IXmlOutputPrint
	// or IXmlOutputPrintStr with an empty string so flags indicate intent for
	// tag to have content
	if (state->flags & IXML_OUTPUT_FLAG_HAD_CONTENT)
		IXmlOutputPrint(state, "</%s>\n", tag);
	else
		IXmlOutputPrintIndent(state, "</%s>\n", tag);
	state->flags &= ~IXML_OUTPUT_FLAG_HAD_CONTENT;	// closed tag
}

void IXmlOutputEndTagWithLineno(IXmlOutputState_t *state, const char *tag, unsigned long lineno)
{
	if (state->cur_indent > state->indent)
		state->cur_indent -= state->indent;
	else
		state->cur_indent = 0;
	// if there was content output we can close
	// tag on same line as the content, otherwise it was a list
	// and we output end tag on a new line with indent
	// Note: output of "empty content tags" should do an IXmlOutputPrint
	// or IXmlOutputPrintStr with an empty string so flags indicate intent for
	// tag to have content
	if (state->flags & IXML_OUTPUT_FLAG_HAD_CONTENT)
		IXmlOutputPrint(state, "</%s><!--line %lu-->\n", tag, lineno);
	else
		IXmlOutputPrintIndent(state, "</%s><!--line %lu-->\n", tag, lineno);
	state->flags &= ~IXML_OUTPUT_FLAG_HAD_CONTENT;	// closed tag
}

void IXmlOutputHexPad8(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	IXmlOutputPrintIndent(state, "<%s>0x%02x</%s>\n", tag, value, tag);
}

// only output if value != 0
void IXmlOutputOptionalHexPad8(IXmlOutputState_t *state, const char *tag, uint8 value)
{
	if (value)
		IXmlOutputHexPad8(state, tag, value);
}

void IXmlOutputHexPad16(IXmlOutputState_t *state, const char *tag, uint16 value)
{
	IXmlOutputPrintIndent(state, "<%s>0x%04x</%s>\n", tag, value, tag);
}

// only output if value != 0
void IXmlOutputOptionalHexPad16(IXmlOutputState_t *state, const char *tag, uint16 value)
{
	if (value)
		IXmlOutputHexPad16(state, tag, value);
}

void IXmlOutputHexPad32(IXmlOutputState_t *state, const char *tag, uint32 value)
{
	IXmlOutputPrintIndent(state, "<%s>0x%08x</%s>\n", tag, value, tag);
}

// only output if value != 0
void IXmlOutputOptionalHexPad32(IXmlOutputState_t *state, const char *tag, uint32 value)
{
	if (value)
		IXmlOutputHexPad32(state, tag, value);
}

void IXmlOutputHexPad64(IXmlOutputState_t *state, const char *tag, uint64 value)
{
	IXmlOutputPrintIndent(state, "<%s>0x%016"PRIx64"</%s>\n", tag, value, tag);
}

// only output if value != 0
void IXmlOutputOptionalHexPad64(IXmlOutputState_t *state, const char *tag, uint64 value)
{
	if (value)
		IXmlOutputHexPad64(state, tag, value);
}

void IXmlOutputInt(IXmlOutputState_t *state, const char *tag, int value)
{
	IXmlOutputPrintIndent(state, "<%s>%d</%s>\n", tag, value, tag);
}

// only output if value != 0
void IXmlOutputOptionalInt(IXmlOutputState_t *state, const char *tag, int value)
{
	if (value)
		IXmlOutputInt(state, tag, value);
}

void IXmlOutputInt64(IXmlOutputState_t *state, const char *tag, int64 value)
{
	IXmlOutputPrintIndent(state, "<%s>%"PRId64"</%s>\n", tag, value, tag);
}

// only output if value != 0
void IXmlOutputOptionalInt64(IXmlOutputState_t *state, const char *tag, int64 value)
{
	if (value)
		IXmlOutputInt64(state, tag, value);
}

static void IXmlOutputIntValue(IXmlOutputState_t *state, const char *tag, int value)
{
	IXmlOutputPrintIndent(state, "<%s_Int>%d</%s_Int>\n", tag, value, tag);
}

void IXmlOutputUint(IXmlOutputState_t *state, const char *tag, unsigned value)
{
	IXmlOutputPrintIndent(state, "<%s>%u</%s>\n", tag, value, tag);
}

// only output if value != 0
void IXmlOutputOptionalUint(IXmlOutputState_t *state, const char *tag, unsigned value)
{
	if (value)
		IXmlOutputUint(state, tag, value);
}

void IXmlOutputUint64(IXmlOutputState_t *state, const char *tag, uint64 value)
{
	IXmlOutputPrintIndent(state, "<%s>%"PRIu64"</%s>\n", tag, value, tag);
}

// only output if value != 0
void IXmlOutputOptionalUint64(IXmlOutputState_t *state, const char *tag, uint64 value)
{
	if (value)
		IXmlOutputUint64(state, tag, value);
}

static void IXmlOutputUintValue(IXmlOutputState_t *state, const char *tag, int value)
{
	IXmlOutputPrintIndent(state, "<%s_Int>%u</%s_Int>\n", tag, value, tag);
}

void IXmlOutputHex(IXmlOutputState_t *state, const char *tag, unsigned value)
{
	IXmlOutputPrintIndent(state, "<%s>0x%x</%s>\n", tag, value, tag);
}

// only output if value != 0
void IXmlOutputOptionalHex(IXmlOutputState_t *state, const char *tag, unsigned value)
{
	if (value)
		IXmlOutputHex(state, tag, value);
}

void IXmlOutputHex64(IXmlOutputState_t *state, const char *tag, uint64 value)
{
	IXmlOutputPrintIndent(state, "<%s>0x%"PRIx64"</%s>\n", tag, value, tag);
}

// only output if value != 0
void IXmlOutputOptionalHex64(IXmlOutputState_t *state, const char *tag, uint64 value)
{
	if (value)
		IXmlOutputHex64(state, tag, value);
}

void IXmlOutputPrintStrLen(IXmlOutputState_t *state, const char* value, int len)
{
	state->flags &= ~IXML_OUTPUT_FLAG_START_NEED_NL;
	state->flags |= IXML_OUTPUT_FLAG_HAD_CONTENT;	// should be content
	/* print string taking care to translate special XML characters */
	for (;len && *value; --len, ++value) {
		if (*value == '&')
			IXmlOutputPrint(state, "&amp;");
		else if (*value == '<')
			IXmlOutputPrint(state, "&lt;");
		else if (*value == '>')
			IXmlOutputPrint(state, "&gt;");
		else if (*value == '\'')
			IXmlOutputPrint(state, "&apos;");
		else if (*value == '"')
			IXmlOutputPrint(state, "&quot;");
		else if (*value != '\n' && iscntrl(*value)) {
			//table in asciitab.h indiciates character codes permitted in XML strings
			//Only 3 control characters below 0x1f are permitted:
			//0x9 (BT_S), 0xa (BT_LRF), and 0xd (BT_CR)
			if ((unsigned char)*value <= 0x08
				|| ((unsigned char)*value >= 0x0b
						 && (unsigned char)*value <= 0x0c)
				|| ((unsigned char)*value >= 0x0e
						 && (unsigned char)*value <= 0x1f)) {
				// characters which XML does not permit in character fields
				IXmlOutputPrint(state, "!");
			} else {
				IXmlOutputPrint(state, "&#x%x;", (unsigned)(unsigned char)*value);
			}
		} else if ((unsigned char)*value > 0x7f)
			// permitted but generate 2 characters back after parsing, so omit
			IXmlOutputPrint(state, "!");
		else
			fputc((int)(unsigned)(unsigned char)*value, state->file);
	}
}

void IXmlOutputPrintStr(IXmlOutputState_t *state, const char* value)
{
	IXmlOutputPrintStrLen(state, value, IB_INT32_MAX);
}

void IXmlOutputStrLen(IXmlOutputState_t *state, const char *tag, const char* value, int len)
{
	IXmlOutputPrintIndent(state, "<%s>", tag);
	IXmlOutputPrintStrLen(state, value, len);
	IXmlOutputPrint(state, "</%s>\n", tag);
	state->flags &= ~IXML_OUTPUT_FLAG_HAD_CONTENT;	// should not be content
}

// only output if value != ""
void IXmlOutputOptionalStrLen(IXmlOutputState_t *state, const char *tag, const char* value, int len)
{
	if (*value)
		IXmlOutputStrLen(state, tag, value, len);
}

void IXmlOutputStr(IXmlOutputState_t *state, const char *tag, const char* value)
{
	IXmlOutputStrLen(state, tag, value, IB_INT32_MAX);
}

// only output if value != ""
void IXmlOutputOptionalStr(IXmlOutputState_t *state, const char *tag, const char* value)
{
	if (*value)
		IXmlOutputStrLen(state, tag, value, IB_INT32_MAX);
}

void IXmlOutputStrUint(IXmlOutputState_t *state, const char *tag, const char* str, unsigned value)
{
	/* when serializing, we omit the string output tag */
	if (! (state->flags & IXML_OUTPUT_FLAG_SERIALIZE)) {
		IXmlOutputStr(state, tag, str);
	}
	IXmlOutputUintValue(state, tag, value);
}

// only output if value != 0
void IXmlOutputOptionalStrUint(IXmlOutputState_t *state, const char *tag, const char* str, unsigned value)
{
	if (value)
		IXmlOutputStrUint(state, tag, str, value);
}

void IXmlOutputStrInt(IXmlOutputState_t *state, const char *tag, const char* str, int value)
{
	/* when serializing, we omit the string output and the _Int tag */
	if (! (state->flags & IXML_OUTPUT_FLAG_SERIALIZE)) {
		IXmlOutputStr(state, tag, str);
	}
	IXmlOutputIntValue(state, tag, value);
}

// only output if value != 0
void IXmlOutputOptionalStrInt(IXmlOutputState_t *state, const char *tag, const char* str, int value)
{
	if (value)
		IXmlOutputStrInt(state, tag, str, value);
}



void IXmlOutputStruct(IXmlOutputState_t *state, const char *tag, void *data,
			IXML_FORMAT_ATTR_FUNC attr_func, const IXML_FIELD *fields)
{
	void *p;

	/* early out test so we don't waste time traversing heirarchy */
	if (IXmlOutputFailed(state))
		return;
	IXmlOutputStartAttrTag(state, tag, data, attr_func);
	for (;fields->tag; ++fields) {
		p = (void *)((uintn)data + fields->offset);
		if (fields->format_func) {
			(*fields->format_func)(state, fields->tag, p);
		} else {
			switch (fields->format) {
			case 'D':
				switch (fields->size) {
				case 1:
					IXmlOutputInt(state, fields->tag, *(int8*)p);
					break;
				case 2:
					IXmlOutputInt(state, fields->tag, *(int16*)p);
					break;
				case 4:
					IXmlOutputInt(state, fields->tag, *(int32*)p);
					break;
				case 8:
					IXmlOutputInt64(state, fields->tag, *(int64*)p);
					break;
				default:
					ASSERT(0);
					break;
				}
				break;
			case 'd':
				switch (fields->size) {
				case 1:
					IXmlOutputOptionalInt(state, fields->tag, *(int8*)p);
					break;
				case 2:
					IXmlOutputOptionalInt(state, fields->tag, *(int16*)p);
					break;
				case 4:
					IXmlOutputOptionalInt(state, fields->tag, *(int32*)p);
					break;
				case 8:
					IXmlOutputOptionalInt64(state, fields->tag, *(int64*)p);
					break;
				default:
					ASSERT(0);
					break;
				}
				break;
			case 'U':
				switch (fields->size) {
				case 1:
					IXmlOutputUint(state, fields->tag, *(uint8*)p);
					break;
				case 2:
					IXmlOutputUint(state, fields->tag, *(uint16*)p);
					break;
				case 4:
					IXmlOutputUint(state, fields->tag, *(uint32*)p);
					break;
				case 8:
					IXmlOutputUint64(state, fields->tag, *(uint64*)p);
					break;
				default:
					ASSERT(0);
					break;
				}
				break;
			case 'u':
				switch (fields->size) {
				case 1:
					IXmlOutputOptionalUint(state, fields->tag, *(uint8*)p);
					break;
				case 2:
					IXmlOutputOptionalUint(state, fields->tag, *(uint16*)p);
					break;
				case 4:
					IXmlOutputOptionalUint(state, fields->tag, *(uint32*)p);
					break;
				case 8:
					IXmlOutputOptionalUint64(state, fields->tag, *(uint64*)p);
					break;
				default:
					ASSERT(0);
					break;
				}
				break;
			case 'X':
				switch (fields->size) {
				case 1:
					IXmlOutputHex(state, fields->tag, *(uint8*)p);
					break;
				case 2:
					IXmlOutputHex(state, fields->tag, *(uint16*)p);
					break;
				case 4:
					IXmlOutputHex(state, fields->tag, *(uint32*)p);
					break;
				case 8:
					IXmlOutputHex64(state, fields->tag, *(uint64*)p);
					break;
				default:
					ASSERT(0);
					break;
				}
				break;
			case 'x':
				switch (fields->size) {
				case 1:
					IXmlOutputOptionalHex(state, fields->tag, *(uint8*)p);
					break;
				case 2:
					IXmlOutputOptionalHex(state, fields->tag, *(uint16*)p);
					break;
				case 4:
					IXmlOutputOptionalHex(state, fields->tag, *(uint32*)p);
					break;
				case 8:
					IXmlOutputOptionalHex64(state, fields->tag, *(uint64*)p);
					break;
				default:
					ASSERT(0);
					break;
				}
				break;
			case 'H':
				switch (fields->size) {
				case 1:
					IXmlOutputHexPad8(state, fields->tag, *(uint8*)p);
					break;
				case 2:
					IXmlOutputHexPad16(state, fields->tag, *(uint16*)p);
					break;
				case 4:
					IXmlOutputHexPad32(state, fields->tag, *(uint32*)p);
					break;
				case 8:
					IXmlOutputHexPad64(state, fields->tag, *(uint64*)p);
					break;
				default:
					ASSERT(0);
					break;
				}
				break;
			case 'h':
				switch (fields->size) {
				case 1:
					IXmlOutputOptionalHexPad8(state, fields->tag, *(uint8*)p);
					break;
				case 2:
					IXmlOutputOptionalHexPad16(state, fields->tag, *(uint16*)p);
					break;
				case 4:
					IXmlOutputOptionalHexPad32(state, fields->tag, *(uint32*)p);
					break;
				case 8:
					IXmlOutputOptionalHexPad64(state, fields->tag, *(uint64*)p);
					break;
				default:
					ASSERT(0);
					break;
				}
				break;
			case 'S':
				IXmlOutputStr(state, fields->tag, (const char*)p);
				break;
			case 's':
				IXmlOutputOptionalStr(state, fields->tag, (const char*)p);
				break;
			case 'P':
				ASSERT(*(const char**)p);
				IXmlOutputStrLen(state, fields->tag, *(const char**)p, fields->size);
			case 'p':
				// skip output of NULL string
				if (*(const char**)p) {
					IXmlOutputStrLen(state, fields->tag, *(const char**)p, fields->size);
				}
				break;
			case 'C':
				IXmlOutputStrLen(state, fields->tag, (const char*)p, fields->size);
				break;
			case 'c':
				IXmlOutputOptionalStrLen(state, fields->tag, (const char*)p, fields->size);
				break;
			case 'k':
			case 'K':
			case 't':
			case 'T':
			case 'w':
			case 'W':
			case 'y':
			case 'Y':
				ASSERT(! fields->format_func);	// tested above
				break;
			default:
				ASSERT(0);
				break;
			}
		}
	}
	IXmlOutputEndTag(state, tag);
}

// only output if data != NULL
void IXmlOutputOptionalStruct(IXmlOutputState_t *state, const char *tag, void *data, IXML_FORMAT_ATTR_FUNC attr_func, IXML_FIELD *fields)
{
	if (data)
		IXmlOutputStruct(state, tag, data, attr_func, fields);
}

/****************************************************************************/
/* XML parser.  This uses expat to parse an XML file.  The format of the XML
 * has some limitations and is intended for serializing/deserializing
 * structures and configuration information
 */
#define BUFFSIZE        8192

/* default callback by the parser to output errors and warnings */
void IXmlPrintMessage(const char *message)
{
	fprintf(stderr, "%s\n", message);
}

// output an error message and increment error_cnt and stop parser
void IXmlParserPrintError(IXmlParserState_t *state, const char *format, ...)
{
	va_list args;
	char buf[1024];
	char buf2[1024];

	va_start(args, format);

	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);

	if (state->current.tag)
		snprintf(buf2, sizeof(buf2),
					"Parse error at line %"PRIu64" in tag '%s': %s",
					(uint64)XML_GetCurrentLineNumber(state->parser),
					state->current.tag, buf);
	else
		snprintf(buf2, sizeof(buf2), "Parse error at line %"PRIu64": %s",
					(uint64)XML_GetCurrentLineNumber(state->parser), buf);
	(state->printError)(buf2);
	// future: could use XML_GetInputContext to get line which failed and
	// use offset into line to display line with a ^ underit
	++state->error_cnt;

	// fatally abort parser
	XML_StopParser(state->parser, 0);
}

void IXmlParserPrintErrorString(IXmlParserState_t *state)
{
	char buf[1024];
	const char *errMsg = XML_ErrorString(XML_GetErrorCode(state->parser));

	snprintf(buf, sizeof(buf), "Parse error at line %"PRIu64": %s",
					(uint64)XML_GetCurrentLineNumber(state->parser),
					errMsg?errMsg:"");
	(state->printError)(buf);
	// future: could use XML_GetInputContext to get line which failed and
	// use offset into line to display line with a ^ underit
}

/* for use internally to identify when end_func call is needed */
static _inline boolean IXmlParserFailed(IXmlParserState_t *state)
{
	return (state->error_cnt > 0);
}

// output a warning message and increment warning_cnt
void IXmlParserPrintWarning(IXmlParserState_t *state, const char *format, ...)
{
	va_list args;
	char buf[1024];
	char buf2[1024];

	va_start(args, format);

	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);

	snprintf(buf2, sizeof(buf2),
				   	"Parse warning at line %"PRIu64" in tag '%s': %s",
					(uint64)XML_GetCurrentLineNumber(state->parser),
					state->current.tag, buf);
	(state->printWarning)(buf2);
	// future: could use XML_GetInputContext to get line which failed and
	// use offset into line to display line with a ^ underit
	++state->warning_cnt;
}

static void IXmlParserPush(IXmlParserState_t *state)
{
	ASSERT(state->stack.sp < STACK_DEPTH-1);
	state->stack.entries[state->stack.sp] = state->current;
	state->stack.sp++;
	state->current.tag = NULL;	// be paranoid and ensure no double reference
}

static void IXmlParserPop(IXmlParserState_t *state)
{
	ASSERT(state->stack.sp >= 1);
	state->stack.sp--;
	if (state->current.tag)
		free(state->current.tag);
	state->current = state->stack.entries[state->stack.sp];
}

// peek parent object
static void *IXmlParserPeek(IXmlParserState_t *state)
{
	ASSERT(state->stack.sp >= 1);
	return state->stack.entries[state->stack.sp-1].object;
}

/* get parser option flags */
extern int IXmlParserGetFlags(IXmlParserState_t *state)
{
	return state->flags;
}

/* get current tag name, returns NULL if no current tag */
const char* IXmlParserGetCurrentTag(IXmlParserState_t *state)
{
	if (state->stack.sp == 0)
		return NULL;
	return state->current.tag;
}

/* get parent tag name, returns NULL if no parent tag */
const char* IXmlParserGetParentTag(IXmlParserState_t *state)
{
	// 0-> no current tag
	// 1-> no parent tag
	// >=2 -> parent and current tag
	if (state->stack.sp <= 1)
		return NULL;
	return state->stack.entries[state->stack.sp-1].tag;
}

/* get current Full dotted tag name, returns NULL if no current tag,
 * returns pointer to a static buffer which is invalid after next call
 * to this function
 * In the event of overflow, the string "overflow" is returned
 */
const char* IXmlParserGetCurrentFullTag(IXmlParserState_t *state)
{
	static char fulltag[STACK_DEPTH*50];
	char *p = fulltag;
	size_t len;
	unsigned i;

	if (state->stack.sp == 0)
		return NULL;
	/* start at 1, entry 0 is invalid */
	for (i=1; i< state->stack.sp; i++) {
		len = strlen(state->stack.entries[i].tag);
		if ((p-fulltag) + len + 2 > sizeof(fulltag))
			return "overflow";
		strcpy(p, state->stack.entries[i].tag);
		p += len;
		*p++='.';
	}
	len = strlen(state->current.tag);
	if ((p-fulltag) + len + 1 > sizeof(fulltag))
		return "overflow";
	strcpy(p, state->current.tag);

	return fulltag;
}

/* get count of child tags to current tag, typically called in ParserEnd */
extern unsigned IXmlParserGetChildTagCount(IXmlParserState_t *state)
{
	return state->current.tags_found;
}

// set subfields for present tag, only valid to call within ParserStart
// callback.  This will override the subfields which were specified for
// the current IXML_FIELD structure
void IXmlParserSetSubfields(IXmlParserState_t *state, const IXML_FIELD *subfields)
{
	state->current.subfields = subfields;
}

// set field for present tag, only valid to call within ParserStart
// callback.  This will override the field which was specified for
// the matching IXML_FIELD structure
// Beware the ParserEndFunc in field will be called instead of the one in
// the original subfields list which matched this tag
// The current subfields are also set to field->subfields
void IXmlParserSetField(IXmlParserState_t *state, const IXML_FIELD *field)
{
	state->current.field = field;
	state->current.subfields = field->subfields;
}

/* return TRUE if current field's contents are empty or all whitespace */
boolean IXmlIsWhitespace(const XML_Char *str, boolean *hasNewline)
{
	*hasNewline = FALSE;
	// str is NULL when tag is empty (has subfields but no text)
	if (str != NULL) {
		for (;*str;str++) {
			if (! isspace(*str))
				return FALSE;
			if (*str == '\n' || *str == '\r')
				*hasNewline = TRUE;
		}
	}
	return TRUE;
}

/* discard leading and trailing whitespace in str, return new length
 * str modified in place
 */
unsigned IXmlTrimWhitespace(XML_Char *str, unsigned len)
{
	unsigned i;

	// trim trailing whitespace
	while (len && isspace(str[len-1]))
		len--;
	str[len] = 0;

	// trim leading whitespace
	for (i=0; i<len && isspace(str[i]); ++i)
		;
	if (i) {
		// overlapping move
		memmove(str, str+i, len+1-i);
		len -= i;
	}
	return len;
}

// discard leading and trailing whitespace in present tag's contents
void IXmlParserTrimWhitespace(IXmlParserState_t *state)
{
	state->len = IXmlTrimWhitespace(state->content, state->len);
}

/* start_func for a simple structure.  Allocates and zeros the structure using
 * size specified in XML_FIELD
 */
void *IXmlParserStartStruct(IXmlParserState_t *state, void *parent, const char **attr)
{
	void *p = MemoryAllocate2AndClear(state->current.field->size, IBA_MEM_FLAG_PREMPTABLE, MYTAG);
	if (! p) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}
	return p;
}

/* end tag function which is a noop.  Use this with start_func=NULL
 * to define XML_FIELDs which are output only
 */
void IXmlParserEndNoop(struct IXmlParserState *state, 
				const IXML_FIELD *field,
				void *object, void *parent, XML_Char *content, unsigned len,
				boolean valid)
{
}

/* helper functions to aid parsing of unsigned hex or decimal fields */
/* return TRUE on success, FALSE on failure
 * on failure IXmlParserPrintError called to describe error and move
 * parser to Failed state
 */
boolean IXmlParseUint8(IXmlParserState_t *state, XML_Char *content, unsigned len, uint8 *value)
{
	uint64 temp;

	if (! IXmlParseUint64(state, content, len, &temp))
		return FALSE;

	if (temp > IB_UINT8_MAX) {
		IXmlParserPrintError(state, "Value out of range");
		return FALSE;
	}
	*value = (uint8)temp;
	return TRUE;
}

boolean IXmlParseUint16(IXmlParserState_t *state, XML_Char *content, unsigned len, uint16 *value)
{
	uint64 temp;

	if (! IXmlParseUint64(state, content, len, &temp))
		return FALSE;

	if (temp > IB_UINT16_MAX) {
		IXmlParserPrintError(state, "Value out of range");
		return FALSE;
	}
	*value = (uint16)temp;
	return TRUE;
}

boolean IXmlParseUint32(IXmlParserState_t *state, XML_Char *content, unsigned len, uint32 *value)
{
	uint64 temp;

	if (! IXmlParseUint64(state, content, len, &temp))
		return FALSE;

	if (temp > IB_UINT32_MAX) {
		IXmlParserPrintError(state, "Value out of range");
		return FALSE;
	}
	*value = (uint32)temp;
	return TRUE;
}

boolean IXmlParseUint64(IXmlParserState_t *state, XML_Char *content, unsigned len, uint64 *value)
{
	FSTATUS status;

	if (! len) {
		IXmlParserPrintError(state, "Empty contents");
		return FALSE;
	}
	status = StringToUint64(value, content, NULL, 0, TRUE /*skip_trail_whitespace */);
	if (status == FINVALID_SETTING) {
		IXmlParserPrintError(state, "Value out of range");
		return FALSE;
	} else if (status != FSUCCESS) {
		IXmlParserPrintError(state, "Invalid contents");
		return FALSE;
	}
	return TRUE;
}

/* helper functions to aid parsing of signed decimal fields */
boolean IXmlParseInt8(IXmlParserState_t *state, XML_Char *content, unsigned len, int8 *value)
{
	int64 temp;

	if (! IXmlParseInt64(state, content, len, &temp))
		return FALSE;

	if (temp < IB_INT8_MIN || temp > IB_INT8_MAX) {
		IXmlParserPrintError(state, "Value out of range");
		return FALSE;
	}
	*value = (int8)temp;
	return TRUE;
}

boolean IXmlParseInt16(IXmlParserState_t *state, XML_Char *content, unsigned len, int16 *value)
{
	int64 temp;

	if (! IXmlParseInt64(state, content, len, &temp))
		return FALSE;

	if (temp < IB_INT16_MIN || temp > IB_INT16_MAX) {
		IXmlParserPrintError(state, "Value out of range");
		return FALSE;
	}
	*value = (int16)temp;
	return TRUE;
}

boolean IXmlParseInt32(IXmlParserState_t *state, XML_Char *content, unsigned len, int32 *value)
{
	int64 temp;

	if (! IXmlParseInt64(state, content, len, &temp))
		return FALSE;

	if (temp < IB_INT32_MIN || temp > IB_INT32_MAX) {
		IXmlParserPrintError(state, "Value out of range");
		return FALSE;
	}
	*value = (int32)temp;
	return TRUE;
}

boolean IXmlParseInt64(IXmlParserState_t *state, XML_Char *content, unsigned len, int64 *value)
{
	FSTATUS status;

	if (! len) {
		IXmlParserPrintError(state, "Empty contents");
		return FALSE;
	}
	status = StringToInt64(value, content, NULL, 0, TRUE /*skip_trail_whitespace */);
	if (status == FINVALID_SETTING) {
		IXmlParserPrintError(state, "Value out of range");
		return FALSE;
	} else if (status != FSUCCESS) {
		IXmlParserPrintError(state, "Invalid contents");
		return FALSE;
	}
	return TRUE;
}

/* process field into object based on state->current.field->format, etc */
static void IXmlParseField(IXmlParserState_t *state)
{
	const IXML_FIELD *field = state->current.field;
	void *p = IXmlParserGetField(field, state->current.object);

	// a custom or wildcard format without an end_func ends up being an output
	// only field which is ignored on input or a field with subfields and no
	// special processing at end of all subfields
	if (tolower(field->format) == 'k'
		|| tolower(field->format) == 't'
		|| tolower(field->format) == 'w'
		|| tolower(field->format) == 'y')
		return;

	DEBUG_ASSERT(! IXmlParserFailed(state));

	ASSERT(! field->start_func);

	switch (tolower(field->format)) {
	case 'd':
		switch (field->size) {
		case 1:
			(void)IXmlParseInt8(state, state->content, state->len, (int8 *)p);
			break;
		case 2:
			(void)IXmlParseInt16(state, state->content, state->len, (int16 *)p);
			break;
		case 4:
			(void)IXmlParseInt32(state, state->content, state->len, (int32 *)p);
			break;
		case 8:
			(void)IXmlParseInt64(state, state->content, state->len, (int64 *)p);
			break;
		default:
			//ASSERT(0);
			break;
		}
		break;
	case 'u':
	case 'x':
	case 'h':
		switch (field->size) {
		case 1:
			(void)IXmlParseUint8(state, state->content, state->len, (uint8 *)p);
			break;
		case 2:
			(void)IXmlParseUint16(state, state->content, state->len, (uint16 *)p);
			break;
		case 4:
			(void)IXmlParseUint32(state, state->content, state->len, (uint32 *)p);
			break;
		case 8:
			(void)IXmlParseUint64(state, state->content, state->len, (uint64 *)p);
			break;
		default:
			//ASSERT(0);
			break;
		}
		break;
	case 's':
		// Presently empty s, p and c formats are allowed.  To make an error:
		//if (! state->len) {
		//	IXmlParserPrintError(state, "Empty contents");
		//	return;
		//}
		if (state->len > field->size-1) {
			IXmlParserPrintWarning(state, "String too long, truncated");
		}
		if (state->len)
			MemoryCopy((char*)p, state->content, MIN(field->size, state->len+1));
		else
			*(char*)p = '\0';
		((char*)p)[field->size-1] = '\0';
		break;
	case 'p':
		{
			int len = MIN(field->size, state->len);
			// Presently empty s, p and c formats are allowed.  To make an error:
			//if (! state->len) {
			//	IXmlParserPrintError(state, "Empty contents");
			//	return;
			//}
			if (state->len > field->size) {
				IXmlParserPrintWarning(state, "String too long, truncated");
			}
			*(char**)p = MemoryAllocate2(len+1, IBA_MEM_FLAG_PREMPTABLE, MYTAG);
			if (*(char**)p) {
				if (len)
					MemoryCopy(*(char**)p, state->content, len);
				(*(char**)p)[len] = '\0';
			} else {
				IXmlParserPrintError(state, "Unable to allocate memory");
			}
		}
		break;
	case 'c':
		// Presently empty s, p and c formats are allowed.  To make an error:
		//if (! state->len) {
		//	IXmlParserPrintError(state, "Empty contents");
		//	return;
		//}
		if (state->len > field->size) {
			IXmlParserPrintWarning(state, "String too long, truncated");
		}
		if (state->len)
			MemoryCopy((char*)p, state->content, MIN(field->size, state->len));
		else
			*(char*)p = '\0';
		break;
	case 'k':
	case 't':
	case 'w':
	case 'y':
		ASSERT(0); // tested above should not get here
		break;
	default:
		ASSERT(0);
		break;
	}
}

static void XMLCALL
IXmlParserStartTag(void *data, const char *el, const char **attr)
{
	IXmlParserState_t *state = (IXmlParserState_t *) data;

#if DEBUG_IXML_PARSER
	{
		int i;
		/* show tag and attributes */
		for (i = 0; i < state->depth; i++)
			printf("  ");
		printf("%s", el);
		for (i = 0; attr[i]; i += 2) {
			printf(" %s='%s'", attr[i], attr[i + 1]);
		}
		printf("\n");
	}
#endif

	ASSERT(! IXmlParserFailed(state));

	/* process start of tag */
	if (state->current.field->start_func) {
		state->current.object = (*state->current.field->start_func)(state, state->current.object, attr);
		// if a simple tag needed a start func it can return parent object
	} else {
		/* start of a simple tag */
		// ASSERT(state->current.object != NULL)
		// processing is done in end for contents of tag
	}
}

// verify manditory subfields (and don't permit contents on tags with subfields)
// tags_found and fields_found optionally return counts (0 if no subfields)
static FSTATUS
IXmlParserCheckSubfields(IXmlParserState_t *state,
							unsigned *tags_found, unsigned *fields_found)
{
	FSTATUS ret = FSUCCESS;

	if (state->current.subfields && ! IXmlParserFailed(state)) {
		const IXML_FIELD *p;
		unsigned i;
		unsigned field_count;

		// presently we don't permit content on "containers"
		// and we require the toplevel document (field==NULL) to have a tag
		// When format of field is 'w' or 'y' (wildcard) we allow container to
		// have content or subfields and expect EndTag function to sort it out
		// In future if content on containers is needed, we could test other
		// aspects of state->current.field such as format, end_func and/or size.
		// Perhaps existance of field->size != 0 would indicate content was allowed
		if (state->len
			&& (state->current.tags_found	/* found child tags */
				|| state->current.field == NULL	/* top level tag */
				|| (state->current.field->format != 'w'
					&& state->current.field->format != 'y'))) {
			IXmlParserPrintWarning(state, "Unexpected contents ignored: %s", state->content);
			ret = FERROR;
		}

		// check for manditory subfields
#if DEBUG_IXML_PARSER
		printf("fields_found=0x%"PRIx64"\n", state->current.fields_found);
#endif
		for (i=0,p=state->current.subfields,field_count=0; p && p->tag && i < 64; p++,i++) {
			if (state->current.fields_found & ((uint64)1<<i)) {
				field_count++;
			} else if (isupper(p->format)) {
				/* manditory field not found */
				IXmlParserPrintError(state, "Mandatory Tag Not Found: %s", p->tag);
				ret = FERROR;
			}
		}
		if (tags_found)
			*tags_found = state->current.tags_found;
		if (fields_found)
			*fields_found = field_count;
	} else {
		/* subfields not applicable */
		if (tags_found)
			*tags_found = 0;
		if (fields_found)
			*fields_found = 0;
	}
	return ret;
}

static void XMLCALL
IXmlParserEndTag(void *data, const char *el)
{
	IXmlParserState_t *state = (IXmlParserState_t *) data;

	// for c, p, s, t and w formats we keep leading and trailing spaces
	// we know we output these tags without any extra spaces
	// if tag had child tags, we also discard whitespace
	if (state->len
		&& (state->current.tags_found
			|| (tolower(state->current.field->format) != 'c'
				&& tolower(state->current.field->format) != 'p'
				&& tolower(state->current.field->format) != 's'
				&& tolower(state->current.field->format) != 't'
				&& tolower(state->current.field->format) != 'w'))) {
		IXmlParserTrimWhitespace(state);
	}

#if DEBUG_IXML_PARSER
	// show content to aid debug
	if (state->len)
		printf("End %s: '%s'\n", el, state->content);
	else
		printf("End %s\n", el);
#endif

	(void)IXmlParserCheckSubfields(state, NULL, NULL);

	/* if there is no start function, we assume its a simple end_func and
	 * does not need to be called to cleanup if IXmlParserFailed
	 * also if object is NULL and we have failed, we don't call end_func
	 */
	if ((state->current.field->start_func && state->current.object != NULL)
		|| ! IXmlParserFailed(state)) {
		// real end processing
		if (state->current.field->end_func) {
			(*state->current.field->end_func)(state, state->current.field, state->current.object, IXmlParserPeek(state), state->content, state->len, ! IXmlParserFailed(state));
		} else {
			IXmlParseField(state);
		}
	}

	if (state->content) {
		free(state->content);
		state->content = NULL;
		state->len = 0;
	}
}

static void
IXmlParserRawStart(void *data, const char *el, const char **attr) {
	IXmlParserState_t *state = (IXmlParserState_t *) data;
	const IXML_FIELD *p;
	unsigned i;

#if DEBUG_IXML_PARSER
	printf("start field %s\n", el);
#endif
	// trim whitespace in parent tag's content
	// we do not permit container tags to have text contents as well
	// generic XML would permit, but it can get confusing especially for
	// embedded whitespace.
	// If we later decide we need this, could include in StackEntry
	if (state->len)
		IXmlParserTrimWhitespace(state);	// remove indentation and newlines
	if (state->len) {
		if (state->current.subfields)
			IXmlParserPrintError(state, "Tag is a container and can't have a value");
		else
			IXmlParserPrintError(state, "Tag has a value and contains %s Tag", el);
		return;
	}
	if (! state->skip && state->current.subfields) {
		for (i=0,p=state->current.subfields; p->tag; p++,i++) {
			if (strcmp(el, p->tag) == 0 || strcmp("*", p->tag) == 0) {
				char *tagname;
				if (i < 64)
					state->current.fields_found |= ((uint64)1)<<i;
				state->current.tags_found++;
#if DEBUG_IXML_PARSER
				printf("tags_found=%u fields_found=0x%"PRIx64"\n", state->current.tags_found, state->current.fields_found);
#endif
				tagname = strdup(el);
				if (! tagname) {
					IXmlParserPrintError(state, "Unable to allocate memory");
					return;
				}
				IXmlParserPush(state);
				state->current.tag = tagname;
				state->current.field = p;
				state->current.subfields = p->subfields;
				state->current.fields_found = 0;
				state->current.tags_found = 0;
				IXmlParserStartTag(state, state->current.tag, attr);   /* rest of start handling */
				break;
			}
		}
		if (! (p && p->tag)) {
			/* unknown tag, skip it and child tags */
			if (state->flags & IXML_PARSER_FLAG_STRICT) {
				IXmlParserPrintWarning(state, "Unexpected tag ignored: %s", el);
			}
			state->skip = state->depth;
		}
	}

	state->depth++;
}

static void
IXmlParserRawEnd(void *data, const char *el) {
	IXmlParserState_t *state = (IXmlParserState_t *) data;

	ASSERT(state->depth > 1);
	state->depth--;

	if (! state->skip) {
		IXmlParserEndTag(state, el);              /* do rest of end handling */
		IXmlParserPop(state);
	}

	if (state->skip == state->depth)
		state->skip = 0;
}

static void
IXmlParserCharHandler(void *data, const XML_Char *buf, int len)
{
	IXmlParserState_t *state = (IXmlParserState_t *) data;

	// for c, p, s, t and w formats we keep leading and trailing spaces
	// we know we output these tags without any extra spaces
	if (! state->content
		&& tolower(state->current.field->format) != 'c'
		&& tolower(state->current.field->format) != 'p'
		&& tolower(state->current.field->format) != 's'
		&& tolower(state->current.field->format) != 't'
		&& tolower(state->current.field->format) != 'w') {
		/* skip leading spaces */
		while (len && isspace(*buf)) {
			buf++; len--;
		}
	}
	if (len) {
		state->content = realloc(state->content, (state->len + len + 1)*sizeof(XML_Char));
		if (state->content) {
			MemoryCopy(state->content+state->len, buf, len);
			state->len += len;
			state->content[state->len] = 0;
		} else {
			(state->printError)("Couldn't allocate memory for content.");
		}
	}
}

static void
IXmlParserRawCharHandler(void *data, const XML_Char *buf, int len)
{
	IXmlParserState_t *state = (IXmlParserState_t *) data;

	if (! state->skip) {
		IXmlParserCharHandler(data, buf, len);
	}
}

FSTATUS
IXmlParserInit(IXmlParserState_t *state, IXmlParserFlags_t flags, const IXML_FIELD *subfields, void *object, void *context, IXmlParserPrintMessage printError, IXmlParserPrintMessage printWarning, XML_Memory_Handling_Suite *memsuite)
{
	MemoryClear(state, sizeof(*state));
	state->flags = flags;
	state->skip = 0;
	state->depth = 1;
	state->content = NULL;
	state->len = 0;
	state->current.tag = NULL;
	state->current.field = NULL;
	state->current.subfields = subfields;
	state->current.object = object;
	state->current.fields_found = 0;
	state->current.tags_found = 0;
	state->stack.sp = 0;
	state->error_cnt = 0;
	state->warning_cnt = 0;
	state->context = context;
	if (printError)
		state->printError = printError;
	else
		state->printError = IXmlPrintMessage;
	if (printWarning)
		state->printWarning = printWarning;
	else
		state->printWarning = IXmlPrintMessage;

	if (memsuite == NULL)
		state->parser = XML_ParserCreate(NULL);
	else 
		state->parser = XML_ParserCreate_MM(NULL, memsuite, NULL);
		
	if (! state->parser) {
		(state->printError)("Couldn't allocate memory for parser");
		return FINSUFFICIENT_MEMORY;
	}

	XML_SetElementHandler(state->parser, IXmlParserRawStart, IXmlParserRawEnd);
	XML_SetCharacterDataHandler(state->parser, IXmlParserRawCharHandler);
	XML_SetUserData(state->parser, state);

	return FSUCCESS;
}

FSTATUS IXmlParserReadFile(IXmlParserState_t *state, FILE *file)
{
	for (;;) {
		int n;
		int done;
		void *buf = XML_GetBuffer(state->parser, BUFFSIZE);
		if (buf == NULL) {
			/* handle error */
			(state->printError)("GetBuffer error");
			return FINSUFFICIENT_MEMORY;
		}

#ifndef VXWORKS
		n = (int)fread(buf, 1, BUFFSIZE, file);
#else
		n = (int)readUncompressedBytes(file, buf, BUFFSIZE);
#endif

		if (ferror(file) || (n<0)) {
			(state->printError)("Read error");
			return FERROR;
		}
		done = feof(file) || (n==0);

		if (XML_ParseBuffer(state->parser, n, done) == XML_STATUS_ERROR) {
			/* if IXmlParserFailed, we already output an error */
			if (! IXmlParserFailed(state))
				IXmlParserPrintErrorString(state);
			return FINVALID_STATE;
		}

		if (done)
			break;
	}
	return FSUCCESS;
}

void
IXmlParserDestroy(IXmlParserState_t *state) {

	// unwind any "open" tags in progress
	while (state->depth > 1) {
		if (state->current.field)
			IXmlParserRawEnd(state, state->current.tag);	// also pops stack
		else {
			state->depth--;

			if (! state->skip)
				IXmlParserPop(state);

			if (state->skip == state->depth)
				state->skip = 0;
		}
	}
	XML_ParserFree(state->parser);
	state->parser = NULL;	// make sure not used by mistake after destroy
	state->context = NULL;	// make sure not used by mistake after destroy
	if (state->current.tag)
		free(state->current.tag);
	state->current.tag = NULL;	// make sure not used by mistake after destroy
}

#ifndef VXWORKS

// parse supplied file.  filename is only used in error messages
FSTATUS
IXmlParseFile(FILE *file, const char* filename, IXmlParserFlags_t flags, const IXML_FIELD *subfields, void *object, void *context, IXmlParserPrintMessage printError, IXmlParserPrintMessage printWarning, unsigned* tags_found, unsigned* fields_found)
{
	IXmlParserState_t state;

	if (FSUCCESS != IXmlParserInit(&state, flags, subfields, object, context, printError, printWarning, NULL)) {
		(printError?printError:IXmlPrintMessage)("Couldn't initialize parser");
		goto failinit;
	}
	if (FSUCCESS != IXmlParserReadFile(&state, file)
		|| FSUCCESS != IXmlParserCheckSubfields(&state, tags_found, fields_found)) {
		IXmlParserPrintError(&state, "Fatal error parsing file '%s'", filename);
		goto failread;
	}
	IXmlParserDestroy(&state);

	return FSUCCESS;

failread:
	IXmlParserDestroy(&state);
failinit:
    return FERROR;
}

// open and parse input_file
FSTATUS
IXmlParseInputFile(const char *input_file, IXmlParserFlags_t flags, const IXML_FIELD *subfields, void *object, void *context, IXmlParserPrintMessage printError, IXmlParserPrintMessage printWarning, unsigned* tags_found, unsigned* fields_found)
{
	FILE *input;

	input = fopen(input_file, "r");
	if (! input) {
		IXmlPrintFileError(input_file, printError);
		goto failopen;
	}

	if (FSUCCESS != IXmlParseFile(input, input_file, flags, subfields, object, context, printError, printWarning, tags_found, fields_found)) {
		goto failread;
	}
	fclose(input);

	return FSUCCESS;

failread:
	fclose(input);
failopen:
    return FERROR;
}

#else

// parse supplied file.  filename is only used in error messages
FSTATUS
IXmlParseFile(FILE *file, const char* filename, IXmlParserFlags_t flags, const IXML_FIELD *subfields, void *object, void *context, IXmlParserPrintMessage printError, IXmlParserPrintMessage printWarning, unsigned* tags_found, unsigned* fields_found, XML_Memory_Handling_Suite* memsuite)
{
	IXmlParserState_t state;

	if (FSUCCESS != IXmlParserInit(&state, flags, subfields, object, context, printError, printWarning, memsuite)) {
		(printError?printError:IXmlPrintMessage)("Couldn't initialize parser");
		goto failinit;
	}
	if (FSUCCESS != IXmlParserReadFile(&state, file)
		|| FSUCCESS != IXmlParserCheckSubfields(&state, tags_found, fields_found)) {
		IXmlParserPrintError(&state, "Fatal error parsing file '%s'", filename);
		goto failread;
	}
	IXmlParserDestroy(&state);

	return FSUCCESS;

failread:
	IXmlParserDestroy(&state);
failinit:
    return FERROR;
}

// open and parse input_file
FSTATUS
IXmlParseInputFile(const char *input_file, IXmlParserFlags_t flags, const IXML_FIELD *subfields, void *object, void *context, IXmlParserPrintMessage printError, IXmlParserPrintMessage printWarning, unsigned* tags_found, unsigned* fields_found, XML_Memory_Handling_Suite* memsuite)
{
	FILE *input;

	input = openUncompressedFile((char*)input_file,NULL);
	if (! input) {
		IXmlPrintFileError(input_file, printError);
		goto failopen;
	}

	if (FSUCCESS != IXmlParseFile(input, input_file, flags, subfields, object, context, printError, printWarning, tags_found, fields_found, memsuite)) {
		goto failread;
	}
	closeUncompressedFile(input);

	return FSUCCESS;

failread:
	closeUncompressedFile(input);
failopen:
    return FERROR;
}

#endif
