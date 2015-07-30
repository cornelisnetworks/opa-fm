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

#define MYTAG MAKE_MEM_TAG('s','a', 'm', 'p') 

/* This sample uses the parser to parse a specific file format
 * sample.xml is an example of the file format
 */

/* here is our internal structure for the sample config file
 * notice how it maps very well to the xml format.  The closer it maps,
 * the easier it is to implement the parser and output
 */
typedef struct MySmInstance_s {
	LIST_ITEM SmInstancesEntry;
	uint8 Instance;
	uint8 Port;
	uint64 MKey;
} MySmInstance_t;

typedef struct MyConfig_s {
	unsigned Mode;
	unsigned LogMask;
	QUICK_LIST SmInstances;
} MyConfig_t;

// in this example this is a simple global.  Could just as easily be
// dynamically allocated
MyConfig_t g_config;

void InitConfig(void)
{
	QListInit(&g_config.SmInstances);
}

// fields within "SM" tag
// In this example, MKey is optional and others are manditory
static IXML_FIELD SMFields[] = {
	{ tag:"Instance", format:'U', IXML_FIELD_INFO(MySmInstance_t, Instance) },
	{ tag:"Port", format:'U', IXML_FIELD_INFO(MySmInstance_t, Port) },
	{ tag:"MKey", format:'x', IXML_FIELD_INFO(MySmInstance_t, MKey) },
	{ NULL }
};

// Output the SmInstance pointed to by data
// caller will specify the tag (typically "SM")
static void SmXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (MySmInstance_t*)data, NULL, SMFields);
}

void SmFree(MyConfig_t *configp, MySmInstance_t *smp)
{
	if (ListItemIsInAList(&smp->SmInstancesEntry))
		QListRemoveItem(&configp->SmInstances, &smp->SmInstancesEntry);
	MemoryDeallocate(smp);
}

// This function will be called each time we encounter an SM start tag
// we can allocate an Sm Instance and process/validate any attributes
// in the start tag
static void *SmXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	int i;
	//boolean gotattr = FALSE;

	// Dynamically allocate the SM instance
	MySmInstance_t *smp = (MySmInstance_t*)MemoryAllocate2AndClear(sizeof(MySmInstance_t), IBA_MEM_FLAG_PREMPTABLE, MYTAG);

	if (! smp) {
		IXmlParserPrintError(state, "Unable to allocate memory");
		return NULL;
	}

	// if desired we could process attributes of SM here.
	// even indexes are attribute names and odd indexes are their values
	// attr[0] = name, attr[1] = value, etc
	// process unixtime attribute and update g_time
	// In this example, perhaps Instance could be an attribute instead of a tag
	for (i = 0; attr[i]; i += 2) {
		if (strcmp(attr[i], "Instance") == 0) {
			// process string in attr[i+1]
			//if (invalid)
			//	IXmlParserPrintError(state, "Invalid Instance attribute");
			//else
			//	gotattr=TRUE;
		}
		// silently ignore unrecognized attributes
	}
	// as needed check for manditory attributes being found
	//if (! gotattr) {
	//	IXmlParserPrintError(state, "Missing Instance attribute");
	//}

	// basic initialization of the structure
	ListItemInitState(&smp->SmInstancesEntry);
	QListSetObj(&smp->SmInstancesEntry, smp);

	// fill in any default values for optional fields here
	// MKey is optional, we could set a non-zero default here if desired

	return smp;	// will be passed to SmXmlParserEnd as object
}

// this function is called when we encounter a SM end tag
// it will validate the object if self consistent and put it into
// the parent's list.  Note that we do not add it to the list
// until the end tag, this way incomplete or inaccurate SmInstances
// are never placed in the parent's list
static void SmXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	MySmInstance_t *smp = (MySmInstance_t*)object;
	MyConfig_t *configp = (MyConfig_t*)parent;
	
	if (! valid)	// missing manditory fields or parse error in child
		goto fail;

	// validate smp, for example make sure smp->Instance is unique
	// Note Xml parser has ensured all manditory fields were supplied
	QListInsertTail(&configp->SmInstances, &smp->SmInstancesEntry);

	return;

fail:
	SmFree(configp, smp);
}

// for output, this outputs all the SMs in the MyConfig
static void ConfigXmlOutputSms(IXmlOutputState_t *state, const char *tag, void *data)
{
	MyConfig_t *configp = (MyConfig_t*)data;
	LIST_ITEM *p;

	// free all link data
	for (p=QListHead(&configp->SmInstances); p != NULL; p = QListNext(&configp->SmInstances, p)) {
		SmXmlOutput(state, "SM", QListObj(p));
	}
}

// The set of fields permitted within the Config tag
// in this example Mode is optional but LogMask is manditory
// SM is an example of a more complex tag which can occur multiple
// times and will become part of a list of SMs
static IXML_FIELD ConfigFields[] = {
	{ tag:"Mode", format:'u', IXML_FIELD_INFO(MyConfig_t, Mode) },
	{ tag:"LogMask", format:'X', IXML_FIELD_INFO(MyConfig_t, LogMask) },
	{ tag:"SM", format:'k', subfields:SMFields, format_func:ConfigXmlOutputSms, start_func:SmXmlParserStart, end_func:SmXmlParserEnd }, // zero or more structures
	{ NULL }
};

// output the Config, will be typically called with tag=="Config"
static void ConfigXmlOutput(IXmlOutputState_t *state, const char *tag, void *data)
{
	IXmlOutputStruct(state, tag, (MyConfig_t*)data, NULL, ConfigFields);
}

void SMFreeAll(MyConfig_t *configp)
{
	LIST_ITEM *p;

	for (p=QListHead(&configp->SmInstances); p != NULL; p = QListHead(&configp->SmInstances)) {
		SmFree(configp, (MySmInstance_t*)QListObj(p));
	}
}

// Config start tag
static void *ConfigXmlParserStart(IXmlParserState_t *state, void *parent, const char **attr)
{
	int i;
	//boolean gotattr = FALSE;

	// if desired we could process attributes of Config here.
	// even indexes are attribute names and odd indexes are their values
	// attr[0] = name, attr[1] = value, etc
	// process unixtime attribute and update g_time
	for (i = 0; attr[i]; i += 2) {
		if (strcmp(attr[i], "myattrname") == 0) {
			// process string in attr[i+1]
			//if (invalid)
			//	IXmlParserPrintError(state, "Invalid myattrname attribute");
			//else
			//	gotattr=TRUE;
		}
		// silently ignore unrecognized attributes
	}
	// as needed check for manditory attributes being found
	//if (! gotattr) {
	//	IXmlParserPrintError(state, "Missing myattrname attribute");
	//}
	return &g_config;	// will be parent for our child tags
}

// Config end tag
static void ConfigXmlParserEnd(IXmlParserState_t *state, const IXML_FIELD *field, void *object, void *parent, XML_Char *content, unsigned len, boolean valid)
{
	MyConfig_t *configp = (MyConfig_t*)object;

	if (! valid) {
		SMFreeAll(configp);	// make sure we cleanup completely
	} else {
		// as needed process or validate self consistency of config
	}
}

// top level tag
static IXML_FIELD TopLevelFields[] = {
	{ tag:"Config", format:'K', subfields:ConfigFields, start_func:ConfigXmlParserStart, end_func:ConfigXmlParserEnd },
	{ NULL }
};

// main output function, this will iterate down the heirarchy 1 level at a time
// If desired we could pass a context into the parser which would
// be available to all the parse functions above via IXmlOutputGetContext
// for example the context might set the scope or style of output
FSTATUS Xml2PrintConfig(FILE *file)
{
	IXmlOutputState_t state;

	if (FSUCCESS != IXmlOutputInit(&state, file, 4, IXML_OUTPUT_FLAG_NONE, NULL ))
		goto fail;

	ConfigXmlOutput(&state, "Config", &g_config);

	IXmlOutputDestroy(&state);

	return FSUCCESS;

fail:
	return FERROR;
}

// main parser.  If desired we could pass a context into the parser which would
// be available to all the parse functions above via IXmlParserGetContext
// for example we may have global settings which control how tags are processed
// or we could have dynamically allocated MyConfig_t and passed the pointer
// in as the context
FSTATUS Xml2ParseConfig(const char *input_file)
{
	return IXmlParseInputFile(input_file, IXML_PARSER_FLAG_NONE, TopLevelFields, NULL, NULL, NULL, NULL, NULL, NULL);
}

void Usage(void)
{
	fprintf(stderr, "Usage: xml_sample2 input_file\n");
	fprintf(stderr, "       input_file - xml file to read.\n");
	exit(2);
}

int main(int argc, char **argv)
{
	int exit_code = 0;
	char *input_file;
	int c;

	while (-1 != (c = getopt(argc, argv, ""))) {
		switch (c) {
		}
	}
	if (argc == optind)
		Usage();	// require input_file argument
	input_file = argv[optind++];
	if (argc > optind)
		Usage();

	InitConfig();	// initialize g_config

	// parse input_file
	fprintf(stderr, "Parsing %s...\n", input_file);
	if (FSUCCESS != Xml2ParseConfig(input_file)) {
		exit_code = 1;
		goto fail;
	}

	// spit it back out to prove we parsed it
	if (FSUCCESS != Xml2PrintConfig(stdout))
		exit_code = 1;

fail:
	exit(exit_code);
}
