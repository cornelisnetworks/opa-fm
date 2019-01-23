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
#define _GNU_SOURCE

#include <ixml.h>

/* example of simple non-predefined parser */
/* for an example of a predefined parser, see opareport/topology.c */

static void *FieldXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr);
static void FieldXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid);

static IXML_FIELD Fields[] = {
	{ tag:"*", format:'w', subfields:Fields, start_func:FieldXmlParserStart, end_func:FieldXmlParserEnd }, // wildcard to traverse xml tree, keep all whitespace
	{ NULL }
};

static IXML_FIELD StrField = 
	{ tag:"*", format:'s', start_func:FieldXmlParserStart, end_func:FieldXmlParserEnd }; // keep all whitespace

static IXML_FIELD OtherField = 
	{ tag:"*", format:'k', start_func:FieldXmlParserStart, end_func:FieldXmlParserEnd }; // trim leading/trailing whitespace


static void *FieldXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	int i;
	const char *tag=IXmlParserGetCurrentTag(state);
	const char *parent_tag=IXmlParserGetParentTag(state);	// can be NULL at top level tag
	const char *full_tag = IXmlParserGetCurrentFullTag(state);

	if (!tag) {
		fprintf(stderr, "tag pointer is null.\n");
		return NULL;
	}

	// process attributes as needed
	printf("Start %s in %s", full_tag?full_tag:"NONE", parent_tag?parent_tag:"NONE");
	for (i = 0; attr[i]; i += 2) {
		printf(" %s='%s'", attr[i], attr[i+1]);
	}
	printf("\n");
	if (strcmp("NodeGUID", tag) == 0) {
		// example of changing parsing format
		// here is a tag we want lead/trail whitespace trimmed for
		IXmlParserSetField(state, &OtherField);
	} else if (strcmp("NodeDesc", tag) == 0) {
		// example of changing parsing format
		// here is a tag we want to keep lead/trail whitespace
		IXmlParserSetField(state, &StrField);
	} else {
		// all other tags processed via Fields which uses 'w' format
		// and hence will keep lead/trail whitespace or permit a container
	}
	return NULL;	// pointer returned here will be passed as object to ParserEnd function below
}

static void FieldXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	const char *full_tag = IXmlParserGetCurrentFullTag(state);
	if (! valid) {
		// syntax error during tag (or its children tags), cleanup
		printf("Cleanup %s", full_tag?full_tag:"<nil>");
	} else {
		// depending on tag and information from Start (object), process content
		printf("End %s: '%s'\n", full_tag?full_tag:"<nil>", content?content:"<nil>");
	}
}

FSTATUS Xml2ParseInputFile(const char *input_file)
{
	if (strcmp(input_file, "-") == 0) {
		printf("Parsing stdin...\n");
		if (FSUCCESS != IXmlParseFile(stdin, "stdin", IXML_PARSER_FLAG_NONE, Fields, NULL, NULL, NULL, NULL, NULL, NULL)) {
			return FERROR;
		}
	} else {
		printf("Parsing %s...\n", input_file);
		if (FSUCCESS != IXmlParseInputFile(input_file, IXML_PARSER_FLAG_NONE, Fields, NULL, NULL, NULL, NULL, NULL, NULL)) {
			return FERROR;
		}
	}
	return FSUCCESS;
}

void Usage(void)
{
	fprintf(stderr, "Usage: xml_sample input_file\n");
	exit(2);
}

int main(int argc, char **argv)
{
	char *filename;
	if (argc != 2)
		Usage();
	filename = argv[1];
	if (!filename) {
		fprintf(stderr, "Error: null input filename\n");
		exit(2);
	}
	if (FSUCCESS == Xml2ParseInputFile(filename))
		exit(0);
	else
		exit(1);
}
