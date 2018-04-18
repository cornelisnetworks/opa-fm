/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

** END_ICS_COPYRIGHT2   ****************************************/

/* [ICS VERSION STRING: unknown] */

#ifndef _IBA_PUBLIC_IXML_H_
#define _IBA_PUBLIC_IXML_H_

#include <iba/ipublic.h>
#include <stdarg.h>
#include <ctype.h>
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif

#ifdef VXWORKS
#include <XmlParser/expat.h>
#else
#include <expat.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_IXML_PARSER 0	/* set to 1 to enable DbgOuts in Xml Parser */

/*****************************************************************************/
/* XML declarations common to parser and output */
struct IXmlOutputState;
struct IXmlParserState;
struct _IXML_FIELD;

typedef void (*IXML_FORMAT_FIELD_FUNC)(struct IXmlOutputState *state,
				const char *tag, void *data);
/* ATTR_FUNC should output a leading space before 1st attribute
 * if there are no attributes to output it can simply do nothing
 */
typedef void (*IXML_FORMAT_ATTR_FUNC)(struct IXmlOutputState *state,
				void *data);
typedef void *(*IXML_START_TAG_FUNC)(struct IXmlParserState *state, 
				void *parent, const char **attr);
typedef void (*IXML_END_TAG_FUNC)(struct IXmlParserState *state, 
				const struct _IXML_FIELD *field,
				void *object, void *parent, XML_Char *content, unsigned len,
				boolean valid);

typedef struct _IXML_FIELD {
	const char *tag;		/* tag name in XML file or "*" */
	char format;			/* format for tag, see below for choices */
	int offset;				/* offset in C structure for tag's value */
	int size;				/* size of field in C structure */
	IXML_FORMAT_FIELD_FUNC format_func;	/* custom function to format output */
	struct _IXML_FIELD *subfields;	/* fields within a container tag */
	IXML_START_TAG_FUNC start_func;	/* function when input <tag> */
	IXML_END_TAG_FUNC end_func;	/* function when input </tag> */
} IXML_FIELD;

// helper macro, given a structure datatype and a member in structure
// this provides initializers for IXML_FIELD offset and size
#define IXML_FIELD_INFO(TYPE, MEMBER) offset:offsetof(TYPE, MEMBER), size:sizeof(((TYPE*)0)->MEMBER)
// similar helper for 'p' formats, here the size can be specified as upper bound
// for the string.  Use IB_INT32_MAX if there is no practical limit
#define IXML_P_FIELD_INFO(TYPE, MEMBER, SIZE) offset:offsetof(TYPE, MEMBER), size:SIZE

/* formats:
 * d - %d format for given field size (1, 2, 4, 8)
 * 		Leading and trailing whitespace ignored.
 * 		Undefined if also specify subfields
 * u - %u format for given field size (1, 2, 4, 8)
 * 		Leading and trailing whitespace ignored.
 * 		Undefined if also specify subfields
 * x - 0x%x format for given field size (1, 2, 4, 8) w/o 0 pad
 * 		Leading and trailing whitespace ignored.
 * 		Undefined if also specify subfields
 * h - 0x%x format for given field size (1, 2, 4, 8) with 0 pad
 * 		Leading and trailing whitespace ignored.
 * 		Undefined if also specify subfields
 * s - '\0' terminated array of chars, %s format, output until '\0' encountered
 * 		use this for arrays of characters directly in the parent structure
 * 		which are to be '\0' terminated.  size indicates size of array
 * 		including space for the '\0' terminator (eg. string is limited to
 * 		size-1 characters).
 * 		Leading and trailing whitespace not trimmed.
 * 		Undefined if also specify subfields
 * p - pointer to string, %s format, output until '\0' encountered
 * 		use this for pointers to strings in the parent structure.  On input
 * 		the pointer will be allocated.  Caller must use MemoryDeallocate
 * 		to free the string when done with it. size indicates upper bound for
 * 		string (not including '\0').  strings longer than this will be truncated
 * 		on input and output, but still '\0' terminated.  NULL pointers will
 * 		be skipped on output and not output the tag at all.  For optional input
 * 		fields when field is not found will default to value set in structure
 * 		StartTag function, typically NULL.
 * 		Leading and trailing whitespace not trimmed.
 * 		Undefined if also specify subfields
 * c - fixed length string input/output until size characters or '\0'
 * 		use this for arrays of characters directly in the parent structure
 * 		which need not be '\0' terminated (such as in IB SA Records).
 * 		size indicates size of array.  A '\0' terminator is added only if the
 * 		input string is less than size.
 * 		Leading and trailing whitespace not trimmed.
 * 		Undefined if also specify subfields
 * k - invoke 'kustom' format_func for output, can use subfields for input
 * 		Leading and trailing whitespace trimmed.
 * 		Enforces that if subfields are specified, no content is	allowed
 *		It is expected that fields without subfields will have explicit
 *		end_func to process them.
 * t - invoke 'kustom' format_func for output, can use subfields for input
 * 		Leading and trailing whitespace not trimmed.
 * 		Enforces that if subfields are specified, no content is	allowed
 *		It is expected that fields without subfields will have explicit
 *		end_func to process them.
 * w - wildcarded tag format.
 * 		Leading and trailing whitespace not trimmed.
 * 		Enforces that if subfields are encountered, no content is allowed
 * 		An end_func should be supplied to handle input, especially when
 * 		no subfields.
 *		Typically used in conjunction with tag name of '*' to permit IXml
 *		parser to implement a stack but manually do tag processing in Start
 *		and End functions.
 * y - trimmed wildcarded tag format.
 * 		Leading and trailing whitespace trimmed.
 * 		Enforces that if subfields are encountered, no content is allowed
 * 		An end_func should be supplied to handle input, especially when
 * 		no subfields.
 *		Typically used in conjunction with tag name of '*' to permit IXml
 *		parser to implement a stack but manually do tag processing in Start
 *		and End functions.
 *
 * The IXmlParserState and IXmlOutputState structures should be treated
 * as opaque structures.  format_func, end_func and start_func should not
 * use fields in these structures.  The necessary information from the state
 * is supplied as explicit arguments.
 *
 * Output:
 * each field is processed in order.
 * If format_func is specified it will be called (regardless of format)
 * with data pointer pointing to present object + offset.
 * If no format_func is specified, the offset and size will be used along
 * with the format to automatically fetch the field and generate output
 * 'k', 't', 'w' and 'y' formats must supply a format_func.
 *
 * Input Parsing:
 * On input parsing, capital letters for format indicate manditory fields
 * Present implementation is limited to no more than 64 manditory fields
 * per object
 *
 * On input parsing, if a start_func is supplied an end_func must also
 * be supplied.  If a end_func is supplied, format is not used to parse
 * data but affects whitespace and subfield processing.
 * An end_func should be supplied for 'k'/'t'/'w'/'y' formats, otherwise
 * the input field will be ignored (recommend use of IXmlParserEndNoop
 * rather than NULL for future compatibility).
 *
 * On output processing, capital letters for format indicate manditory fields
 * which are always output regardless of value.  lower case leters for format
 * are optional fields which will only be output if non-zero/non-NULL.
 * For 'k', 't', 'w' and 'y' fields, its up to the supplied format_func to
 * decide.
 * There are "Optional" variations of most generic format functions which may
 * be used by format_func if desired.
 *
 * For an output only field:
 * 	type/format_func define output
 * 	start_func=NULL, end_func=IXmlParserEndNoop
 * For an input only field:
 * 	type/start_func/end_func define input
 * 	format_func=IXmlOutputNoop
 * For an input/output field:
 * 	type/start_func/end_func define input
 * 	type/format_func define output
 *
 * Error handling:
 * Failures in XML syntax or calls to IXmlParserPrintError will stop
 * the parser with an error.  For such parsing failures, the parser
 * will unwind and invoke the end_func's for all objects whose start_func
 * had been called.  Such calls will have valid=FALSE.  In this case
 * the end_func should undo any operations which the start_func did
 * (free object, remove object from lists in parent to which start func added it
 * etc).
 *
 * If a start_func or end_func encounters a fatal error, it should call
 * IXmlParserPrintError with an appropriate message.  This will output
 * a message including input file line, present tag and the message
 * provided by the caller.  It will also start the parser error and unwind
 * sequence.
 *
 * Once a parser error is detected, no further start_func's will be called.
 *
 * Object handling:
 * The parser assists in tracking object heirachies and provides objects
 * to the start_func and end_func.  The start_func is expected to return an
 * object pointer.  This pointer is typically an object/structure allocated
 * by the start function to hold the subsequent tags.  The pointer returned by
 * the start_func will be provided to the end_func as the "object" argument.
 * Also if there are nested tags (eg. subfields), the object returned by
 * the start_func will also be provided to the nested tags' start_func and
 * end_func as the "parent".
 *
 * Typical operation would be for the start_func to allocate and initialize
 * the empty object.  For simple structures which simply need to be
 * allocated and zeroed, the IXmlParserStartStruct function is provided.
 * Then the end_func would verify the consistency of the fields and link the
 * object into its parent (add to a list or pointer).  If the end_func discovers
 * an error it should call IXmlParserPrintError (or Warning if its non-fatal)
 * and free the object.
 *
 * For simple fields (bitfields, needing special handling, etc).  The
 * start_func can be NULL and the end_func will be given the object and
 * parent applicable to the preceeding tag.
 *
 * The code in ixml_ib.c is a good example of use of the parser.
 *
 * xml_sample provides a simple example of a parser which uses the
 * wildcards to parse and dump any XML file given.
 *
 * opaxmlindent expands on xml_sample by dumping the XML with nice indentation
 */

/*****************************************************************************/
/* XML Output Declarations */
typedef enum {
	/* flags which can be passed to IXmlInit and IXmlOutputInit */
	IXML_OUTPUT_FLAG_NONE = 0,
	IXML_OUTPUT_FLAG_SERIALIZE = 1,	/* compact serialized format */
	/* these flags are for internal use only */
	IXML_OUTPUT_FLAG_START_NEED_NL = 0x10000,	/* start tag output without newline */
	IXML_OUTPUT_FLAG_HAD_CONTENT = 0x20000,	/* tag had content output */
	IXML_OUTPUT_FLAG_IN_START_TAG = 0x40000, /* For printing multiple attrs */
} IXmlOutputFlags_t;

/* these structures should not be directly used by callers */
typedef struct IXmlOutputState {
// TBD - later support output to a memory buffer
	FILE *file;		/* output file */
	unsigned indent;	/* level of indent */
	unsigned cur_indent;	/* level of indent */
	int flags;
	void *context;	/* caller supplied context */
} IXmlOutputState_t;

/* for use in output calls so can early exit */
/* note that IXmlOutputStruct tests this, so in general it does not need to
 * be called in other functions
 */
static _inline boolean IXmlOutputFailed(IXmlOutputState_t *state)
{
	return (ferror(state->file) != 0);
}

/* get access to caller supplied context for output */
static _inline void* IXmlOutputGetContext(IXmlOutputState_t *state)
{
	return state->context;
}

/* indent is additional indent per level */
extern void IXmlInit(IXmlOutputState_t *state, FILE *file,
				unsigned indent, IXmlOutputFlags_t flags, void *context);
extern FSTATUS IXmlOutputInit(IXmlOutputState_t *state, FILE *file,
				unsigned indent, IXmlOutputFlags_t flags, void *context);
extern void IXmlOutputDestroy(IXmlOutputState_t *state);
extern void IXmlOutputPrint(IXmlOutputState_t *state, const char *format, ...);
extern void IXmlOutputPrintIndent(IXmlOutputState_t *state, const char *format, ...);
extern void IXmlOutputNoop(IXmlOutputState_t *state, const char *tag, void *data);
extern void IXmlOutputStartTag(IXmlOutputState_t *state, const char *tag);

/*
 * Print start tag with single attribute and value. Use
 * IXmlOutputStartTag() and IXmlOutputAttr*() if you need to print
 * multiple attributes.
 */
extern void IXmlOutputStartAttrTag(IXmlOutputState_t *state, const char *tag, void *data, IXML_FORMAT_ATTR_FUNC attr_func);

/*
 * Add @attr with @val to current element tag definition.
 * This doesn't do any sanitizing on @attr or @val so be careful
 * what you put in. Should be called after a call to IXmlOutputStartTag().
 * Any call (direct or indirect) to IXmlOutputPrint() or
 * IXmlOutputPrintIndent() closes the start tag.
 *
 * @return FSUCCESS on success, FERROR if not in the start of a tag
 */
extern FSTATUS IXmlOutputAttr(IXmlOutputState_t *state, const char *attr, const char *val);

/*
 * Like IXmlOutputAttr() but takes a format string and variable args.
 * As with IXmlOutputAttr(), does not sanitize inputs.
 *
 * @return FSUCCESS on success, FERROR if not in the start of a tag
 */
extern FSTATUS IXmlOutputAttrFmt(IXmlOutputState_t *state, const char *attr, const char *format, ...);
extern void IXmlOutputEndTag(IXmlOutputState_t *state, const char *tag);
extern void IXmlOutputEndTagWithLineno(IXmlOutputState_t *state, const char *tag, unsigned long lineno);
extern void IXmlOutputHexPad8(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputOptionalHexPad8(IXmlOutputState_t *state, const char *tag, uint8 value);
extern void IXmlOutputHexPad16(IXmlOutputState_t *state, const char *tag, uint16 value);
extern void IXmlOutputOptionalHexPad16(IXmlOutputState_t *state, const char *tag, uint16 value);
extern void IXmlOutputHexPad32(IXmlOutputState_t *state, const char *tag, uint32 value);
extern void IXmlOutputOptionalHexPad32(IXmlOutputState_t *state, const char *tag, uint32 value);
extern void IXmlOutputHexPad64(IXmlOutputState_t *state, const char *tag, uint64 value);
extern void IXmlOutputOptionalHexPad64(IXmlOutputState_t *state, const char *tag, uint64 value);
extern void IXmlOutputInt(IXmlOutputState_t *state, const char *tag, int value);
extern void IXmlOutputOptionalInt(IXmlOutputState_t *state, const char *tag, int value);
extern void IXmlOutputInt64(IXmlOutputState_t *state, const char *tag, int64 value);
extern void IXmlOutputOptionalInt64(IXmlOutputState_t *state, const char *tag, int64 value);
extern void IXmlOutputUint(IXmlOutputState_t *state, const char *tag, unsigned value);
extern void IXmlOutputOptionalUint(IXmlOutputState_t *state, const char *tag, unsigned value);
extern void IXmlOutputUint64(IXmlOutputState_t *state, const char *tag, uint64 value);
extern void IXmlOutputOptionalUint64(IXmlOutputState_t *state, const char *tag, uint64 value);
extern void IXmlOutputHex(IXmlOutputState_t *state, const char *tag, unsigned value);
extern void IXmlOutputOptionalHex(IXmlOutputState_t *state, const char *tag, unsigned value);
extern void IXmlOutputHex64(IXmlOutputState_t *state, const char *tag, uint64 value);
extern void IXmlOutputOptionalHex64(IXmlOutputState_t *state, const char *tag, uint64 value);
extern void IXmlOutputPrintStrLen(IXmlOutputState_t *state, const char* value, int len);
extern void IXmlOutputPrintStr(IXmlOutputState_t *state, const char* value);
extern void IXmlOutputStrLen(IXmlOutputState_t *state, const char *tag, const char* value, int len);
extern void IXmlOutputOptionalStrLen(IXmlOutputState_t *state, const char *tag, const char* value, int len);
extern void IXmlOutputStr(IXmlOutputState_t *state, const char *tag, const char* value);
extern void IXmlOutputOptionalStr(IXmlOutputState_t *state, const char *tag, const char* value);
extern void IXmlOutputStrUint(IXmlOutputState_t *state, const char *tag, const char* str, unsigned value);
extern void IXmlOutputOptionalStrUint(IXmlOutputState_t *state, const char *tag, const char* str, unsigned value);
extern void IXmlOutputStrInt(IXmlOutputState_t *state, const char *tag, const char* str, int value);
extern void IXmlOutputOptionalStrInt(IXmlOutputState_t *state, const char *tag, const char* str, int value);
extern void IXmlOutputStruct(IXmlOutputState_t *state, const char *tag, void *data,
				IXML_FORMAT_ATTR_FUNC attr_func, const IXML_FIELD *fields);
extern void IXmlOutputOptionalStruct(IXmlOutputState_t *state, const char *tag, void *data, IXML_FORMAT_ATTR_FUNC attr_func, IXML_FIELD *fields);

/*****************************************************************************/
/* XML Parser declarations */
/* these structures should not be directly used by callers */
typedef struct IXmlParserStackEntry {
	char *tag;				// locally allocated copy
	const IXML_FIELD *field;
	const IXML_FIELD *subfields;
	void *object;
	unsigned tags_found;	// total subfield tags encountered including dups
	uint64 fields_found;	// bit mask of indexes into subfields
} IXmlParserStackEntry_t;

#define STACK_DEPTH 100

typedef struct IXmlParserStack {
	/* TOS is kept in state->current instead */
	unsigned sp;	/* if 0, stack is empty */
	IXmlParserStackEntry_t entries[STACK_DEPTH];	/* entry 0 is unused */
} IXmlParserStack_t;

/* callbacks by the parser to output errors and warnings */
typedef void (*IXmlParserPrintMessage)(const char *message);

typedef struct IXmlParserState {
// TBD - later support input from a memory buffer
	XML_Parser parser;
	int			flags;	/* parser option flags */
	unsigned	depth;	/* how many nested elements deep >= 1 */
	unsigned 	skip;	/* skip all elements until we return to this depth */
	IXmlParserStackEntry_t current; /* state of current element being parsed */
	XML_Char *content;	/* text contents of current element */
	unsigned len;		/* number of characters in content */
	IXmlParserStack_t stack;	/* stack of parent elements' states */
	unsigned error_cnt;
	unsigned warning_cnt;
	void *context;	/* caller supplied context */
	IXmlParserPrintMessage printError;
	IXmlParserPrintMessage printWarning;
} IXmlParserState_t;

/* get access to caller supplied context for input */
static _inline void* IXmlParserGetContext(IXmlParserState_t *state)
{
	return state->context;
}

typedef enum {
	/* flags which can be passed to IXmlInit and IXmlOutputInit */
	IXML_PARSER_FLAG_NONE = 0,
	IXML_PARSER_FLAG_STRICT = 1,	/* provide warnings for unknown tags, etc */
} IXmlParserFlags_t;

/* get parser option flags */
extern int IXmlParserGetFlags(IXmlParserState_t *state);

/* get current tag name, returns NULL if no current tag */
extern const char* IXmlParserGetCurrentTag(IXmlParserState_t *state);

/* get parent tag name, returns NULL if no parent tag */
extern const char* IXmlParserGetParentTag(IXmlParserState_t *state);

/* get current Full dotted tag name, returns NULL if no current tag,
 * returns pointer to a static buffer which is invalid after next call
 * to this function
 * In the event of overflow, the string "overflow" is returned
 */
extern const char* IXmlParserGetCurrentFullTag(IXmlParserState_t *state);

/* get count of child tags to current tag, typically called in ParserEnd */
extern unsigned  IXmlParserGetChildTagCount(IXmlParserState_t *state);

// set subfields for present tag, only valid to call within ParserStart
// callback.  This will override the subfields which were specified for
// the current IXML_FIELD structure
extern void IXmlParserSetSubfields(IXmlParserState_t *state, const IXML_FIELD *subfields);

// set field for present tag, only valid to call within ParserStart
// callback.  This will override the field which was specified for
// the matching IXML_FIELD structure
// Beware the ParserEndFunc in field will be called instead of the one in
// the original subfields list which matched this tag
// The current subfields are also set to field->subfields
extern void IXmlParserSetField(IXmlParserState_t *state, const IXML_FIELD *field);

// output an error message and increment error_cnt
extern void IXmlParserPrintError(IXmlParserState_t *state, const char *format, ...);

// output a warning message and increment warning_cnt
extern void IXmlParserPrintWarning(IXmlParserState_t *state, const char *format, ...);

/* helper functions to aid getting field pointer in end_func callbacks */
/* this is useful when implementing reusable end_func's which don't know exact
 * datatype of object
 */
static _inline void* IXmlParserGetField(const IXML_FIELD* field, void *object)
{
	return (void*)((uintn)object + field->offset);
}

/* helper functions to aid parsing of unsigned hex or decimal fields */
/* return TRUE on success, FALSE on failure
 * on failure IXmlParserPrintError called to describe error and move
 * parser to Failed state
 */
extern boolean IXmlParseUint8(IXmlParserState_t *state,
							XML_Char *content, unsigned len, uint8 *value);
extern boolean IXmlParseUint16(IXmlParserState_t *state,
							XML_Char *content, unsigned len, uint16 *value);
extern boolean IXmlParseUint32(IXmlParserState_t *state,
							XML_Char *content, unsigned len, uint32 *value);
extern boolean IXmlParseUint64(IXmlParserState_t *state,
							XML_Char *content, unsigned len, uint64 *value);

/* helper functions to aid parsing of signed decimal fields */
extern boolean IXmlParseInt8(IXmlParserState_t *state,
							XML_Char *content, unsigned len, int8 *value);
extern boolean IXmlParseInt16(IXmlParserState_t *state,
							XML_Char *content, unsigned len, int16 *value);
extern boolean IXmlParseInt32(IXmlParserState_t *state,
							XML_Char *content, unsigned len, int32 *value);
extern boolean IXmlParseInt64(IXmlParserState_t *state,
							XML_Char *content, unsigned len, int64 *value);

/* start_func for a simple structure.  Allocates and zeros the structure using
 * size specified in XML_FIELD
 */
extern void *IXmlParserStartStruct(IXmlParserState_t *state, void *parent, const char **attr);

/* end tag function which is a noop.  Use this with start_func=NULL
 * to define XML_FIELDs which are output only
 */
extern void IXmlParserEndNoop(struct IXmlParserState *state, 
				const IXML_FIELD *field,
				void *object, void *parent, XML_Char *content, unsigned len,
				boolean valid);

/* return TRUE if current field's contents are empty or all whitespace */
boolean IXmlIsWhitespace(const XML_Char *str, boolean *hasNewline);

/* discard leading and trailing whitespace in str, return new length
 * str modified in place
 */
unsigned IXmlTrimWhitespace(XML_Char *str, unsigned len);

extern FSTATUS IXmlParserInit(IXmlParserState_t *state, IXmlParserFlags_t flags,
				const IXML_FIELD *subfields, void *object, void *context,
				IXmlParserPrintMessage printError,
			   	IXmlParserPrintMessage printWarning,
				XML_Memory_Handling_Suite* memsuite);

extern FSTATUS IXmlParserReadFile(IXmlParserState_t *state, FILE *file);
extern void IXmlParserDestroy(IXmlParserState_t *state);

#ifndef VXWORKS
// parse supplied file.  filename is only used in error messages
extern FSTATUS IXmlParseFile(FILE *file, const char* filename,
			   	IXmlParserFlags_t flags,
				const IXML_FIELD *subfields, void *object, void *context,
				IXmlParserPrintMessage printError,
			   	IXmlParserPrintMessage printWarning,
				unsigned* tags_found, unsigned *fields_found);
// open and parse input_file
extern FSTATUS IXmlParseInputFile(const char *input_file,
			   	IXmlParserFlags_t flags,
				const IXML_FIELD *subfields, void *object, void *context,
				IXmlParserPrintMessage printError,
			   	IXmlParserPrintMessage printWarning,
				unsigned* tags_found, unsigned *fields_found);
#else
// parse supplied file.  filename is only used in error messages
extern FSTATUS IXmlParseFile(FILE *file, const char* filename,
			   	IXmlParserFlags_t flags,
				const IXML_FIELD *subfields, void *object, void *context,
				IXmlParserPrintMessage printError,
			   	IXmlParserPrintMessage printWarning,
				unsigned* tags_found, unsigned *fields_found,
				XML_Memory_Handling_Suite* memsuite);
// open and parse input_file
extern FSTATUS IXmlParseInputFile(const char *input_file,
			   	IXmlParserFlags_t flags,
				const IXML_FIELD *subfields, void *object, void *context,
				IXmlParserPrintMessage printError,
			   	IXmlParserPrintMessage printWarning,
				unsigned* tags_found, unsigned *fields_found,
				XML_Memory_Handling_Suite* memsuite);
#endif

#ifdef __cplusplus
};
#endif

#endif /* _IBA_PUBLIC_IXML_H_ */
