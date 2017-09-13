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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <fnmatch.h>
#include <ctype.h>
#include <fnmatch.h>
#include <getopt.h>
#define _GNU_SOURCE

#include <ixml.h>

boolean g_trim = FALSE;
boolean g_keep_newline = FALSE;
boolean g_add_lineno = FALSE;

boolean g_filter = FALSE;

#define MAX_ELEMENTS 100
char *g_elements[MAX_ELEMENTS] = { 0 };
int g_numElements = 0;

/* example of simple non-predefined parser */
/* for an example of a predefined parser, see opareport/topology.c */

static void *FieldXmlParserStart(IXmlParserState_t *input_state, void *parent, const char **attr);
static void FieldXmlParserEnd(IXmlParserState_t *input_state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid);

static IXML_FIELD UntrimmedFields[] = {
	{ tag:"*", format:'w', subfields:UntrimmedFields, start_func:FieldXmlParserStart, end_func:FieldXmlParserEnd }, // wildcard to traverse xml tree, keep all whitespace
	{ NULL }
};

#if 0
static IXML_FIELD TrimmedFields[] = {
	{ tag:"*", format:'y', subfields:TrimmedFields, start_func:FieldXmlParserStart, end_func:FieldXmlParserEnd }, // wildcard to traverse xml tree, trim whitespace
	{ NULL }
};
#endif

static void FieldXmlFormatAttr(IXmlOutputState_t *output_state, void *data)
{
	const char **attr = (const char **)data;
	int i;

	for (i = 0; attr[i]; i += 2) {
		IXmlOutputPrint(output_state, " %s=\"", attr[i]);
		IXmlOutputPrintStr(output_state, attr[i+1]);
		IXmlOutputPrint(output_state, "\"");
	}
}

static boolean compareElement(const char* element)
{
	int i;
	for (i=0; i<g_numElements; ++i) {
		if (! fnmatch(g_elements[i], element, 0))
			return TRUE;
	}
	return FALSE;
}

static void *FieldXmlParserStart(IXmlParserState_t *input_state, void *parent, const char **attr)
{
	IXmlOutputState_t *output_state = (IXmlOutputState_t*)IXmlParserGetContext(input_state);

	if (g_filter)
		return NULL;	// we are filtering this element and any sub-tags
	if (! g_filter) {
		char *element = (char *)IXmlParserGetCurrentFullTag(input_state);
		if (compareElement(element)) {
			g_filter = TRUE;
			return &g_filter;	// so end knows this is where filtering started
		}
	}
	// if no attr, could use:
	// IXmlOutputStartTag(output_state, IXmlParserGetCurrentTag(input_state));
	IXmlOutputStartAttrTag(output_state, IXmlParserGetCurrentTag(input_state), (void*)attr, FieldXmlFormatAttr);
	// all tags processed via Fields which uses wildcard format
	// and hence will keep lead/trail whitespace or permit a container
	return NULL;	// pointer returned here will be passed as object to ParserEnd function below
}

static void FieldXmlParserEnd(IXmlParserState_t *input_state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	IXmlOutputState_t *output_state = (IXmlOutputState_t*)IXmlParserGetContext(input_state);

	if (! valid) {
		// syntax error during tag (or its children tags), cleanup
		//fprintf(stderr, "Cleanup %s", IXmlParserGetCurrentFullTag(input_state));
	} else {
		boolean hasNewline;
		boolean white;
		if (object) {
			// end of filtered element
			g_filter = FALSE;
			return;	// we filter end tag
		}
		if (g_filter)
			return;	// inside a filtered element

		// depending on tag and information from Start (object), process content
		white = IXmlIsWhitespace(content, &hasNewline);
		if (len && g_trim)
			len = IXmlTrimWhitespace(content, len);
		if (white && ! g_keep_newline && hasNewline) {
			// no real content, but has a newline, probably an empty list
			// no output here, and EndTag will be on a fresh line
		} else if (len) {
			// tag had content, output it with appropriate XML escapes for
			// special characters
			IXmlOutputPrintStr(output_state, content);
		} else if (! IXmlParserGetChildTagCount(input_state)) {
			// if there were no child tags and no content,
			// we output this as an empty tag
			// this way tags with no content stay that way
			// if g_trim && ! g_keep_newline, empty lists get listed on one line
			IXmlOutputPrintStr(output_state, "");
		}
		// close out the StartTag
		if (g_add_lineno)
			IXmlOutputEndTagWithLineno(output_state, IXmlParserGetCurrentTag(input_state), (unsigned long)XML_GetCurrentLineNumber(input_state->parser));
		else
			IXmlOutputEndTag(output_state, IXmlParserGetCurrentTag(input_state));
	}
}

FSTATUS Xml2ParseInputFile(const char *input_file, void *context, IXML_FIELD *fields)
{
	if (strcmp(input_file, "-") == 0) {
		fprintf(stderr, "Parsing stdin...\n");
		if (FSUCCESS != IXmlParseFile(stdin, "stdin", IXML_PARSER_FLAG_NONE, fields, NULL, context, NULL, NULL, NULL, NULL)) {
			return FERROR;
		}
	} else {
		fprintf(stderr, "Parsing %s...\n", input_file);
		if (FSUCCESS != IXmlParseInputFile(input_file, IXML_PARSER_FLAG_NONE, fields, NULL, context, NULL, NULL, NULL, NULL)) {
			return FERROR;
		}
	}
	return FSUCCESS;
}

void Usage(int exitcode)
{
	fprintf(stderr, "Usage: opaxmlfilter [-t|-k] [-l] [-i indent] [-s element] [input_file]\n");
	fprintf(stderr, "           or\n");
	fprintf(stderr, "       opaxmlfilter --help\n");
	fprintf(stderr, "       --help - produce full help text\n");
	fprintf(stderr, "       -t - trim leading and trailing whitespace in tag contents\n");
	fprintf(stderr, "       -k - in tags with purely whitespace which contain newlines,\n");
	fprintf(stderr, "            keep newlines as is (default is to format as an empty list)\n");
	fprintf(stderr, "       -l - add comments with line numbers after each end tag\n");
	fprintf(stderr, "            This can make comparison of resulting files easier\n");
	fprintf(stderr, "            since original line numbers will be available\n");
	fprintf(stderr, "       -i indent - set indentation to use per level (default 4)\n");
	fprintf(stderr, "       -s element - name of XML element to suppress\n");
	fprintf(stderr, "                    can be used multiple times (max 100), order does not matter\n");

	fprintf(stderr, "       input_file - xml file to read.  default is stdin\n");
	exit(exitcode);
}

static void addElement(const char *element)
{
	int len = 0;

	if (g_numElements >= MAX_ELEMENTS) {
		fprintf(stderr, "opaxmlfilter: Too many suppressed elements, limit of %d\n", MAX_ELEMENTS);
		exit(1);
	}

	if (! element || ! (len = strlen(element))) {
		fprintf(stderr, "opaxmlfilter: Missing element.\n");
		Usage(2);
	}

	len += 3;	// allow for \0 and possible wildcards
	g_elements[g_numElements] = malloc(len);
	if (! g_elements[g_numElements]) {
		fprintf(stderr, "opaxmlfilter: Unable to allocate memory\n");
		exit(1);
	}

	snprintf(g_elements[g_numElements], len, "%s%s", strchr(element, '*') ? "" : "*.", element);
	// no issue with duplicates, so no need to check
	g_numElements++;
}

int main(int argc, char **argv)
{
	IXmlOutputState_t output_state;
	int exit_code = 0;
	uint32 indent = 4;
	const char *opts="tkli:s:";
	const struct option longopts[] = {{"help", 0, 0, '$'},
						{0, 0, 0, 0}};
	char *filename = "-";	// default to stdin
	IXML_FIELD *fields = UntrimmedFields;
	int c;

	while (-1 != (c = getopt_long(argc, argv, opts, longopts, NULL))) {
		switch (c) {
			case '$':
				Usage(0);
			case 't':
				// TrimmedFields treats empty list as tag with no content
				//fields = TrimmedFields;
				g_trim = TRUE;
				break;
			case 'i':
				if (FSUCCESS != StringToUint32(&indent, optarg, NULL, 0, TRUE)) {
					fprintf(stderr, "opaxmlfilter: Invalid indent: %s\n", optarg);
					Usage(2);
				}
				break;
			case 'k':
				g_keep_newline = TRUE;
				break;
			case 'l':
				g_add_lineno = TRUE;
				break;
			case 's':
				addElement(optarg);
				break;
			default:
				Usage(2);
		}
	}
	if (g_trim && g_keep_newline) {
		fprintf(stderr, "opaxmlfilter: Can't use -k and -t together\n");
		Usage(2);
	}
	if (argc > optind)
		filename = argv[optind++];
	if (argc > optind)
		Usage(2);
	if (FSUCCESS != IXmlOutputInit(&output_state, stdout, indent, IXML_OUTPUT_FLAG_NONE, NULL))
		exit(1);
	if (FSUCCESS != Xml2ParseInputFile(filename, &output_state, fields))
		exit_code = 1;
	IXmlOutputDestroy(&output_state);
	exit(exit_code);
}
