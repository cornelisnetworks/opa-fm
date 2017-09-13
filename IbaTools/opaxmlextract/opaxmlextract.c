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


/*******************************************************************************
 *
 * File: opaxmlextract.c
 *
 * Description:
 *	This file implements the opaxmlextract program.  opaxmlextract takes
 *	well-formed XML as input, extracts element values as specified by user
 *	input, and outputs data as lines (records) of data in CSV format.
 *
 *	opaxmlextract uses XML Lib to parse the XML.  It uses the folowing library
 *	functions:
 *	  IXmlParseFile()
 *
 *	opaxmlextract implements the following call-backs (handlers):
 *	  IXML_START_TAG_FUNC()
 *	  IXML_END_TAG_FUNC()
 *
 *	opaxmlextract takes 2 types of element names as input from the user - elements
 *	to be extracted, and those for which extraction is to be suppressed.  Both
 *	types of elements are stored in the Element Table (tbElements[]).  As
 *	opaxmlextract processes the XML (through the call-backs), it saves the values
 *	of elements to be extracted.  opaxmlextract outputs a record containing those
 *	values under the following conditions:
 *	  1 or more elements being extracted and containing a non-null value go out
 *	    of scope (the element containing those elements is ended)
 *	  An element changes value while it is in scope
 *
 *	During operation, opaxmlextract maintains the hierarchical "level" at which
 *	it is extracting (top-most level is 1).  Each nested element increases the
 *	level by 1.  opaxmlextract performs extraction on a (specified) element
 *	regardless of the level at which the element is found.
 *
 *	When an element is specified for suppression then no extraction will be
 *	performed during the extent of that element (begin through end).
 *	Suppression is maintained for elements specified inside the suppressed
 *	element, even if those elements have extraction specified.  Suppressed
 *	elements can be nested - opaxmlextract will resume extraction after the
 *	outermost suppressed element is ended.
 *
 *	Element names for extraction or suppression can be made context sensitive
 *	with an enclosing element name using syntax 'elem1.elem2'.  elem2 will
 *	be extracted (or extraction will be suppressed) when elem2 is enclosed
 *	by elem1.  The wildcard handling capability of XML Lib allows '*' to
 *	be specified as all or part of an element name.  The following are valid:
 *	        *.elem3  -  elem3 enclosed by any sequence of elements
 *	                    (elem1.elem3 or elem1.elem2.elem3)
 *	  elem1.*.elem3  -  elem3 enclosed by elem1 with any number of (but at
 *	                    least 1) intermediate elements.
 *
 *	opaxmlextract prepends any entered element name not containing '*'
 *	(anywhere) with '*.'.
 *
 *	NOTES:
 *	opaxmlextract performs extraction using XML Lib without maintaining
 *	its own stack of elements or element values.  Therefore it has the
 *	following characteristics:
 *	  Enclosing elements cannot contain values;
 *
 *	  If an element name specification is such that the element can be
 *	  extracted at multiple levels within the same context (enclosing
 *	  element), then when the inner-most element specification ends,
 *	  the element value becomes null (it does not revert to the outer
 *	  value).
 */


/*******************************************************************************
 *
 * INCLUDES
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <iba/ipublic.h>
#include <getopt.h>
#include "ixml.h"
#include "fnmatch.h"


/*******************************************************************************
 *
 * DEFINES
 */
#define ALLOW_MULTI_MATCH		1		// allow 1 tag to match mult elements
#define SUPPRESS_AND_EXTRACT	0		// allow 1 tag to suppress and extract
										// this option not fully implemented
										// procElementEnd would need more work

#define OK  0
#define ERROR (-1)

#define NAME_PROG  "opaxmlextract"		// Program name
#define MAX_PARAMS_FILE  512			// Max number of param file parameters
#define MAX_ELEMENTS  100				// Max number of elements parsable
#define MAX_LEVEL_PARSE  MAX_ELEMENTS	// Max parsing level
#define MAX_DELIMIT_CHARS  16			// Max size of delimiter string
#define MAX_INPUT_BUF  8192				// Max size of input buffer
#define MAX_PARAM_BUF  8192				// Max size of parameter file
#define MAX_FILENAME_CHARS  256			// Max size of file name
#define WHITE_SPACE  " \t\r\n"			// White space characters
#define OPTION_LEN 2			// string length of an option(eg. -P)

// Element table entry, elements selected by user and present parsed value
typedef struct 
{
	char	*pName;						// Element name
	char	*pAttrName;					// optional Attribute Name
	char	*pAttrValue;				// optional Attribute Value
	int		lenValue;					// Length of element value
										//   < 0 : pValue alloc'd but not in use
	char	*pValue;					// Element value
	int		flags;						// Flags (see ELEM_xxx below)
	int		level;						// Element nesting level (0 - not open)
} ELEMENT_TABLE_ENTRY;

// Element table flags:
#define ELEM_NAMEPREPEND  0x80			// Data prepended to element name

// list of elements which a given tag matched and the attribute value
// which the element selected.  This is generated by the start tag handler
// and used by the end tag handler for a given "matched" tag
typedef struct
{
	int		ix;							// element table index
	int		lenAttrValue;				// length of value
	char	*pAttrValue;				// value, NULL if not outputting attr
} ELEMENT_LIST_ENTRY;

typedef struct
{
	int		numElements;			// number of entrys in list
	ELEMENT_LIST_ENTRY Elements[1];	// variable length list
} ELEMENT_LIST;


/*******************************************************************************
 *
 * LOCAL FUNCTION PROTOTYPES
 */

int delWsLead(char *pStr);
int delWsTrail(char *pStr);
void dispExtractionRecord();
void dispHeaderRecord(int argc, char ** argv);
void errUsage(void);
int findElement(int start, const char *pElement, const char **pAttrib,
				char **pValue, int *length);
int findElementDup(const ELEMENT_TABLE_ENTRY *pElement);
void getRecu_opt( int argc, char ** argv, const char *pOptShort,
	struct option tbOptLong[] );
void *procElementBeg(IXmlParserState_t *pParserState, void *pParent, const char **ppAttrib);
void procElementEnd( IXmlParserState_t *pParserState, const IXML_FIELD *pField,
	void *pObject, void *pParent, XML_Char *pContent, unsigned length,
	boolean flValid );


/*******************************************************************************
 *
 * GLOBAL VARIABLES
 */

int		g_exitstatus  = 0;
int		g_verbose  = 0;
int		g_quiet  = 0;


/*******************************************************************************
 *
 * LOCAL VARIABLES
 */

int  levelParse  = 0;					// Parse nesting level
int  numElementsTable  = 0;				// Number of elements in tbElements[]
int  numElementsExtract  = 0;			// Number of elements to extract
int  ctElementsSuppress  = 0;			// Count of suppress elements being parsed
int  flElementChangeLast = FALSE;		// tbElement[] value change flag (last line)
int  flElementChange  = FALSE;			// tbElement[] value change flag
int  flHeaderOutput  = TRUE;			// Header output flag
XML_Parser  pParser  = NULL;			// Pointer to expat parser object

FILE  * hFileInput  = NULL;				// Input file handle (default stdin)
										// Input file name (default stdin)
char  nameFileInput[MAX_FILENAME_CHARS + 1]  = "stdin";
uint32  fvDebugProg  = 0;			// Debug/Trace variable as:
										//  0MMM MMMM NNNN NNNN  SSSS 21RC SIEB SNEB

// Command line option table, each has a short and long flag name
static const char* tbShortOptions="#vHe:s:X:P:d:Z:";
struct option tbOptions[] =
{
	// Basic controls
	{ "help", no_argument, NULL, 'h' },
	{ "verbose", no_argument, NULL, 'v' },
	{ "extract", required_argument, NULL, 'e' },
	{ "suppress", required_argument, NULL, 's' },
	{ "infile", required_argument, NULL, 'X' },
	{ "pfile", required_argument, NULL, 'P' },
	{ "delimit", required_argument, NULL, 'd' },
	{ "noheader", no_argument, NULL, 'H' },
	{ "debug", required_argument, NULL, 'Z' },
	{ 0 }
};

// Element table; contains information about each element of interest.
//  Elements to be extracted are contained at the beginning of the table
//  (0 - numElementsExtract-1); elements for which extraction is suppressed
//  are contained at the end of the table
//  (numElementsExtract - numElementsTable-1).
ELEMENT_TABLE_ENTRY  tbElements[MAX_ELEMENTS + 1];

// IXML_FIELD table; contains information for the XML Lib parser.  For
// purposes of opaxmlextract, a call-back on every start tag and end tag
// is required; therefore IXML_FIELD tag is set to "*" and format to 'w'.
IXML_FIELD  tbFields[] =
{
	{tag:"*", format:'w', subfields:tbFields, start_func:procElementBeg,
		end_func:procElementEnd},
	{NULL}
};

// Delimiter string buffer
char  bfDelimit[MAX_DELIMIT_CHARS + 1] = ";";

// Input buffer
char  bfInput[MAX_INPUT_BUF];

void *mymalloc(size_t size, const char* desc)
{
	void *ret = malloc(size);
	if (! ret)
	{
		fprintf(stderr, NAME_PROG ": Unable to Allocate %s Memory\n", desc);
		exit(1);
	}
	return ret;
}

void *myrealloc(void* ptr, size_t size, const char* desc)
{
	void *ret = realloc(ptr, size);
	if (! ret)
	{
		fprintf(stderr, NAME_PROG ": Unable to Re-Allocate %s Memory\n", desc);
		exit(1);
	}
	return ret;
}

/*******************************************************************************
 *
 * delWsLead()
 *
 * Description:
 *	Delete leading white space from a string.  White space is defined by the
 *	string WHITE_SPACE.
 *
 * Inputs:
 *	   pStr - Pointer to string from which to delete leading white space
 *
 * Outputs:
 *	ERROR - pStr NULL
 *	   OK - white space deleted, no white space to delete
 */
int delWsLead(char *pStr)
{
	int		nChar;
	char	*pChar;

	if (!pStr)
		return (ERROR);

	nChar = strspn(pStr, WHITE_SPACE);
	if (nChar != 0)
	{
		pChar = pStr + nChar;
		memmove(pStr, pChar, strlen(pChar) + 1);
	}

	return (OK);

}	// End of delWsLead()


/*******************************************************************************
 *
 * delWsTrail()
 *
 * Description:
 *	Delete trailing white space from a string.  White space is defined by the
 *	string WHITE_SPACE.
 *
 * Inputs:
 *	   pStr - Pointer to string from which to delete trailing white space
 *
 * Outputs:
 *	ERROR - pStr NULL
 *	   OK - white space deleted, no white space to delete
 */
int delWsTrail(char *pStr)
{
	int		nChar;
	char	*pChar;

	if (!(pChar = pStr))
		return (ERROR);					// pStr is NULL

	while (TRUE)
	{
		// Find next non-white space char
		nChar = strspn(pChar, WHITE_SPACE);
		if (pChar[nChar])
		{
			// Find next white space char
			pChar += nChar;
			pChar += strcspn(pChar, WHITE_SPACE);
			if (!*pChar)
				break;				// No trailing white space
		}

		// Found first white space char after last non-white space char
		else
		{
			*pChar = 0;
			break;
		}

	}	// End of while (TRUE)

	return (OK);

}	// End of delWsTrail()


/*******************************************************************************
 *
 * dispExtractionRecord()
 *
 * Description:
 *	Display (output) 1 record of extraction data if element change flag
 *	indicates an element has changed since the last output.
 *
 * Inputs:
 *	none
 *
 * Outputs:
 *	none
 */
void dispExtractionRecord()
{
	int		ix;							// Loop index

	// Check element change flag
	if (!flElementChangeLast)
		return;

	// Output extraction record: element values
	for (ix = 0; ix < numElementsExtract; ix++)
	{
		(void)delWsTrail(tbElements[ix].pValue);

		if (ix > 0)
			printf("%s", bfDelimit);

		printf("%s", tbElements[ix].pValue ? tbElements[ix].pValue : "" );
	}
	printf("\n");
	flElementChangeLast = FALSE;

}	// End of dispExtractionRecord()

void printElement(FILE *f, ELEMENT_TABLE_ENTRY *pElement, int verbose)
{
	// Output complete name string if verbose or name not prepended
	if (verbose || !(pElement->flags & ELEM_NAMEPREPEND))
		fprintf(f, "%s", pElement->pName);
	else
		fprintf(f, "%s", &pElement->pName[2]);
	if (pElement->pAttrName)
	{
		fprintf(f, ":%s", pElement->pAttrName);
		if (pElement->pAttrValue)
			fprintf(f, ":%s", pElement->pAttrValue);
	}
}

/*******************************************************************************
 *
 * dispHeaderRecord()
 *
 * Description:
 *	Display (output) header line containing names of elements extracted.  This
 *	line preceeds the extraction data.  If verbose is set, preceed the header
 *	line with a line containing the program name and command line options.
 *
 * Inputs:
 *	argc, argv - Inputs to main()
 *
 * Outputs:
 *	none
 */
void dispHeaderRecord(int argc, char ** argv)
{
	int		ix;							// Loop index

	// Output header record only if output enabled
	if (flHeaderOutput)
	{
		// Output element names as a header
		for (ix = 0; ix < numElementsExtract; ix++)
		{
			if (ix > 0)
				printf("%s", bfDelimit);

			printElement(stdout,&tbElements[ix], g_verbose);
		}
		printf("\n");
	}

}	// End of dispHeaderRecord()


/*******************************************************************************
 *
 * errUsage()
 *
 * Description:
 *	Output information about program usage and parameters.
 *
 * Inputs:
 *	none
 *
 * Outputs:
 *	none
 */
void errUsage(void)
{
	fprintf(stderr, "Usage: " NAME_PROG " [-v][-H][-d delimiter][-e element][-s element]\n");
	fprintf(stderr, "                        [-X input_file] [-P param_file]\n");
	fprintf(stderr, "  -e/--extract element      - name of an XML element to extract\n");
	fprintf(stderr, "                              can be used multiple times\n");
	fprintf(stderr, "                              elements can be nested in any order, but will\n");
	fprintf(stderr, "                              be output in the order specified\n");
	fprintf(stderr, "                              optionally an attribute or\n");
	fprintf(stderr, "                              attribute and value can be specified\n");
	fprintf(stderr, "                              -e element\n");
	fprintf(stderr, "                              -e element:attrName\n");
	fprintf(stderr, "                              -e element:attrName:attrValue\n\n");
	fprintf(stderr, "                              Note that elements can be compound values\n");
	fprintf(stderr, "                              separated by a dot. For example,\n");
	fprintf(stderr, "                              'Switches.Node' is a Node element contained\n"); 
	fprintf(stderr, "                              within a Switches element.\n\n"); 
	fprintf(stderr, "                              If its desired to output the attribute value as\n");
	fprintf(stderr, "                              opposed to the element value, a specification such\n");
	fprintf(stderr, "                              as '-e FIs.Node:id' can be used, which will return\n");
	fprintf(stderr, "                              the value of the id attribute of any Node elements\n");
	fprintf(stderr, "                              within FIs element. If desired a specific element\n");
	fprintf(stderr, "                              can be selected by its attribute value, such as\n");
	fprintf(stderr, "                              '-e MulticastFDB.Value:LID:0xc000', which will\n");
	fprintf(stderr, "                              return the value of the Value element within\n");
	fprintf(stderr, "                              Multicast FDB element where the Value element has\n");
	fprintf(stderr, "                              an attribute of LID with a value of 0xc000.\n\n");

#if ALLOW_MULTI_MATCH
	fprintf(stderr, "                              A given element can be specified multiple\n");
	fprintf(stderr, "                              times each with a different AttrName or attrValue\n");
#else
	fprintf(stderr, "                              a given element can be specified only once\n");
#endif
	fprintf(stderr, "  -s/--suppress element     - name of XML element to suppress extraction\n");
	fprintf(stderr, "                              can be used multiple times, order does not matter\n");
	fprintf(stderr, "                              supports same syntaxes as -e\n");
	fprintf(stderr, "  -d/--delimit delimiter    - delimiter output between element names and values\n");
	fprintf(stderr, "                              default is semicolon\n");
	fprintf(stderr, "  -X/--infile input_file    - parse XML in input_file\n");
	fprintf(stderr, "  -P/--pfile param_file     - read command parameters from param_file\n");
	fprintf(stderr, "  -H/--noheader             - do not output element name header record\n");
	fprintf(stderr, "  -v/--verbose              - verbose output: progress reports during extraction\n");
	fprintf(stderr, "                              and element name prepended wildcard characters\n");
	fprintf(stderr, "  --help                    - print this usage text.\n");

	if (hFileInput && (hFileInput != stdin))
		fclose(hFileInput);

	exit(2);
}	// End of errUsage()


/*******************************************************************************
 *
 * findElement()
 *
 * Description:
 *	Find specified element name in element table using wildcard matching.
 *	Search suppression elements first by starting search at end of table
 *	and searching toward beginning.
 *
 * Inputs:
 *	start - first element in table to check (end of table)
 *	pElement - Pointer to element name
 *	ppAttrib - attribute list from tag being processed
 *
 * Outputs:
 *	Index of element table containing element name, else
 *	ERROR - Element name not found
 *
 *	*pValue, *length - if matched a Name:attrName style specifier, the value of
 *						the attribute matched.  Otherwise NULL,0
 *						(caller must free *pValue if non-NULL)
 */
int findElement(int start, const char *pElement, const char **ppAttrib,
				char **pValue, int *length)
{
	int		ix;

	*pValue = NULL;
	*length = 0;
	for (ix = start; ix >= 0; ix--)
	{
		ELEMENT_TABLE_ENTRY *p = &tbElements[ix];
		if (!fnmatch(p->pName, pElement, 0))
	   	{
			int i;
			if (! p->pAttrName)
				return (ix);	// match only on tag name

			// search for matching attribute
			// even index in ppAttrib is name, odd is corresponding value
			for (i=0; ppAttrib[i]; i += 2)
			{
				if (!fnmatch(p->pAttrName, ppAttrib[i], 0))
				{
					if (! p->pAttrValue)
					{
						// match on Attr Name and return attr value
						*length = strlen(ppAttrib[i+1]);
						*pValue = mymalloc(*length+1, "Attribute Value");
						strcpy(*pValue, ppAttrib[i+1]);
						return (ix);
					}
					// match on attr value
					if (! fnmatch(p->pAttrValue, ppAttrib[i+1], 0))
						return (ix);
				}
			}
		}
	}

	return (ERROR);

}	// End of findElement()


/*******************************************************************************
 *
 * findElementDup()
 *
 * Description:
 *	Find specified element name in element table using duplicate (2-way)
 *	wildcard matching.  Search element table from beginning to end since
 *	there is no need to check suppression elements first.
 *
 * Inputs:
 *	pElement - Pointer to element name
 *
 * Outputs:
 *	Index of element table containing element name, else
 *	ERROR - Element name not found
 */
int findElementDup(const ELEMENT_TABLE_ENTRY *pElement)
{
	int		ix;

	for (ix = 0; ix < numElementsTable; ix++)
	{
		ELEMENT_TABLE_ENTRY *p = &tbElements[ix];
		if (0 == fnmatch(p->pName, pElement->pName, 0)
			|| 0 == fnmatch(pElement->pName, p->pName, 0))
		{
#if ALLOW_MULTI_MATCH
			if ((p->pAttrName == NULL) != (pElement->pAttrName == NULL))
				continue;
			if (p->pAttrName && pElement->pAttrName)
			{
				if ( 0 == fnmatch(p->pAttrName, pElement->pAttrName, 0) ||
						0 == fnmatch(pElement->pAttrName, p->pAttrName, 0) )
				{
					if ((p->pAttrValue == NULL) != (pElement->pAttrValue == NULL))
						continue;
					if ( p->pAttrValue && pElement->pAttrValue &&
							! ( 0 == fnmatch(p->pAttrValue, pElement->pAttrValue, 0) ||
							0 == fnmatch(pElement->pAttrValue, p->pAttrValue, 0) ) )
						continue;

					return (ix);
				}

				continue;
			}
#else
			// allow different values for same attr because won't both match
			// the same tag
			if (p->pAttrName && pElement->pAttrName
				&& (0 == fnmatch(p->pAttrName, pElement->pAttrName, 0)
					|| 0 == fnmatch(pElement->pAttrName, p->pAttrName, 0))
				&& p->pAttrValue && pElement->pAttrValue
				&& ! (0 == fnmatch(p->pAttrValue, pElement->pAttrValue, 0)
						|| 0 == fnmatch(pElement->pAttrValue, p->pAttrValue, 0)))
					continue;
#endif
			return (ix);

		}	// End of if (0 == fnmatch(p->pName, pElement->pName, 0)

	}	// End of for (ix = 0; ix < numElementsTable; ix++)

	return (ERROR);

}	// End of findElementDup()

void getNextField(const char *arg, int extra, char **field, const char **rest)
{
	const char *p = rest?strchr(arg, ':'):NULL;
	int len;

	if (p)
   	{
		len = (p-arg);
		*rest = p+1;
	} else {
		len = strlen(arg);
		if (rest)
			*rest = NULL;
	}
	*field = mymalloc(len+1+extra, "Name");
	strncpy(*field, arg, len);
	(*field)[len] = '\0';
}


/*******************************************************************************
 *
 * getRecu_opt()
 *
 * Description:
 *	Get command line options (recursively).  Parses command line options
 *	using short and long option (parameter) definitions.  Parameters (global
 *	variables) are updated appropriately.  The parameter '-P param_file' which
 *	takes command line options from a file is also handled.  The -P parameter
 *	can be used within a parameter file; in such cases the function will call
 *	itself recursively.
 *
 * Inputs:
 *	     argc - Number of input parameters
 *	     argv - Array of pointers to input parameters
 *	pOptShort - Pointer to string of short option definitions
 *	tbOptLong - Array of long option definitions
 * 
 * Outputs:
 *	none
 */
void getRecu_opt( int argc, char ** argv, const char *pOptShort,
	struct option tbOptLong[] )
{
	int		ix;
	int		cOpt;						// Option parsing char                         
	int		ixOpt;						// Option parsing index
	int		lenStr;						// String length
	char	*pChar;						// Pointer to char
	ELEMENT_TABLE_ENTRY *pElement1,		// Pointers to element entries
			*pElement2;
	int		argc_recu;					// Recursive argc
	char	*argv_recu[MAX_PARAMS_FILE];  // Recursive argv
	int		optind_recu;				// Recursive optind
	char	*optarg_recu;				// Recursive optarg

	int		ctCharParam;				// Parameter file char count
	FILE	*hFileParam = NULL;			// Parameter file handle
	char	nameFileParam[MAX_FILENAME_CHARS +1];  // Parameter file name
	char	bfParam[MAX_PARAM_BUF + 1];

	// Input command line arguments
	while ((cOpt = getopt_long(argc, argv, pOptShort, tbOptLong, &ixOpt)) != -1)
    {
		lenStr = 0;
        switch (cOpt)
        {
		// Verbose
		case 'v':
			g_verbose = 1;
			break;

		// Extract element name specification
		case 'e':
		// Suppress element name specification
		case 's':
			// syntaxes supported:
			// Name    - match Tag with given name
			// Name:attr - select attr field of Tag with given name and attr
			// Name:attr:value - match Tag with given name and attr value
			// All three components allow "glob" style patterns
			if (numElementsTable == MAX_ELEMENTS)
			{
				fprintf(stderr, NAME_PROG ": Too many Elements\n");
				errUsage();
			}

			else if (!optarg || ! *optarg)
			{
				fprintf(stderr, NAME_PROG ": Missing Element\n");
				errUsage();
			}

			else
			{
				ELEMENT_TABLE_ENTRY *pElement = &tbElements[numElementsTable];
				const char *rest;

				// Tag Name
				getNextField(optarg, 2, &pElement->pName, &rest);
				if (! strchr(optarg, '*'))
				{
					// insert *. before string (extra 2 above allowed space)
					memmove(pElement->pName+2, pElement->pName, strlen(pElement->pName)+1);
					pElement->pName[0] = '*'; pElement->pName[1] = '.';
					pElement->flags |= ELEM_NAMEPREPEND;
				}
				if (rest) {
					// optional AttrName
					if (! *rest) // trailing colon
					{
						fprintf(stderr, NAME_PROG ": Attr Name missing: %s\n", optarg);
						errUsage();
					}
					getNextField(rest, 0, &pElement->pAttrName, &rest);
					if (rest) {
						// optional AttrValue
						if (! *rest) // trailing colon
						{
							fprintf(stderr, NAME_PROG ": Attr Value missing: %s\n", optarg);
							errUsage();
						}
						getNextField(rest, 0, &pElement->pAttrValue, NULL);
					}
				}
				if ((ix = findElementDup(&tbElements[numElementsTable])) != ERROR)
				{
					fprintf(stderr, NAME_PROG ": Duplicate (matching) Element: %s (matches: ", optarg);
					printElement(stderr, &tbElements[ix], FALSE);
					fprintf(stderr, ")\n");
					errUsage();
				}

			}

			// If suppress specification, leave at end of tbElements[]
			if (cOpt == 's')
				numElementsTable++;

			// If extraction specification w/no suppress elements, leave at
			//  end of tbElements[]
			else if ((cOpt == 'e') && (numElementsExtract == numElementsTable))
			{
				numElementsTable++;
				numElementsExtract++;
			}

			// If extraction specification w/suppress elements, move extraction
			//  to end of other extractions
			else if ((cOpt == 'e') && (numElementsExtract < numElementsTable))
			{
				// Swap last element table entry with first suppress element entry
				//  (NOTE: lenValue, pValue and level are zero, so no swap needed)
				pElement1 = &tbElements[numElementsExtract++];
				pElement2 = &tbElements[numElementsTable++];

				pChar = pElement1->pName;
				pElement1->pName = pElement2->pName;
				pElement2->pName = pChar;

				pChar = pElement1->pAttrName;
				pElement1->pAttrName = pElement2->pAttrName;
				pElement2->pAttrName = pChar;

				pChar = pElement1->pAttrValue;
				pElement1->pAttrValue = pElement2->pAttrValue;
				pElement2->pAttrValue = pChar;

				lenStr = pElement1->flags;
				pElement1->flags = pElement2->flags;
				pElement2->flags = lenStr;
			}

			break;

		// Delimiter string specification
		case 'd':
			if ( !optarg || ((lenStr = strlen(optarg)) == 0) ||
					(lenStr > MAX_DELIMIT_CHARS) )
			{
				fprintf(stderr, NAME_PROG ": Invalid delimiter size: %d\n", lenStr);
				errUsage();
			}

			strcpy(bfDelimit, optarg);
			break;

		// Input file specification
		case 'X':
			if ( !optarg || ((lenStr = strlen(optarg)) == 0) ||
					(lenStr > MAX_FILENAME_CHARS) )
			{
				fprintf(stderr, NAME_PROG ": Missing Input File Name.\n");
				errUsage();
			}

			else if (hFileInput != stdin)
			{
				fprintf( stderr, NAME_PROG ": Multiple Input Files: %s and %s\n",
					nameFileInput, optarg );
				errUsage();
			}

			strcpy(nameFileInput, optarg);
			hFileInput = fopen(nameFileInput, "r");

			if (!hFileInput)
			{
				fprintf( stderr, NAME_PROG ": Unable to Open Input File: %s\n",
					nameFileInput );
				perror("");
				errUsage();
			}
			break;

		// Parameter file specification
		case 'P':
			if (!optarg || ((lenStr = strlen(optarg)) == 0) ||
					(lenStr > MAX_FILENAME_CHARS) )
			{
				fprintf(stderr, NAME_PROG ": Missing Parameter File Name.\n");
				errUsage();
			}

			// Open parameter file
			strcpy(nameFileParam, optarg);
			hFileParam = fopen(nameFileParam, "r");

			if (!hFileParam)
			{
				fprintf( stderr, NAME_PROG ": Unable to Open Parameter File: %s\n",
					nameFileParam );
				perror("");
				errUsage();
			}

			// Read parameter file
			if (g_verbose)
				fprintf(stderr, NAME_PROG ": Reading Param File: %s\n", nameFileParam);

			ctCharParam = (int)fread(bfParam, 1, MAX_PARAM_BUF, hFileParam);

			if (ferror(hFileParam))
			{
				fclose(hFileParam);
				fprintf(stderr, NAME_PROG ": Read Error: %s\n", nameFileParam);
				errUsage();
			}

			if ((ctCharParam == MAX_PARAM_BUF) && !feof(hFileParam))
			{
				fclose(hFileParam);
				fprintf( stderr, NAME_PROG ": Parameter file (%s) too large (> %d bytes)\n",
					nameFileParam, MAX_PARAM_BUF );
				errUsage();
			}

			// Make sure the buffer is null terminated.
			bfParam[MAX_PARAM_BUF] = 0;

			fclose(hFileParam);

			if (ctCharParam > 0)
			{
				// Parse parameter file into tokens
				argc_recu = 2;
				argv_recu[0] = nameFileParam;
			
				for (ix = 1; ; ix++, argc_recu++)
				{
					if ( (argv_recu[ix] = strtok((ix == 1) ? bfParam : NULL, WHITE_SPACE))
							== NULL )
					{
						argc_recu--;
						break;
					}

					if (!strncmp(argv_recu[ix], "-P", OPTION_LEN))
					{
						fprintf( stderr, NAME_PROG ": Parameter file (%s) cannot contain -P option\n", nameFileParam);
						errUsage();
					}

					if ( (argv_recu[ix] - bfParam + strlen(argv_recu[ix]) + 1) ==
							ctCharParam )
						break;

					if (ix == MAX_PARAMS_FILE)
					{
						fprintf( stderr, NAME_PROG ": Parameter file (%s) has too many parameters (> %d)\n",
							nameFileParam, MAX_PARAMS_FILE );
						errUsage();
					}

				}

				// Process parameter file parameters
				if (argc_recu > 1)
				{
					optind_recu = optind;	// Save optind & optarg
					optarg_recu = optarg;
					optind = 1;   			// Init optind & optarg
					optarg = NULL;
					getRecu_opt(argc_recu, argv_recu, pOptShort, tbOptLong);
					optind = optind_recu;	// Restore optind & optarg
					optarg = optarg_recu;
				}

			}	// End of if (ctCharParam > 0)

			break;

		// No Header
		case 'H':
			flHeaderOutput = FALSE;
			break;

		// Debug trace specification
		case 'Z':
			if (FSUCCESS != StringToUint32(&fvDebugProg, optarg, NULL, 0, TRUE)) {
				fprintf(stderr, NAME_PROG ": Invalid Debug Setting: %s\n", optarg);
				errUsage();
			}
			break;

		case '#':
		default:
			errUsage();
			break;

        }	// End of switch (cOpt)

    }	// End of while ( ( cOpt = getopt_long( argc, argv,

	// Validate command line arguments
	if (optind < argc)
	{
		fprintf(stderr, "%s: invalid argument %s\n", NAME_PROG, argv[optind]);
		errUsage();
	}

}	// End of getRecu_opt()


/*******************************************************************************
 *
 * main()
 *
 * Description:
 *	main function for program.
 *
 * Inputs:
 *	argc - Number of input parameters
 *	argv - Array of pointers to input parameters
 *
 * Outputs:
 *	Exit status 0 - no errors
 *	Exit status 2 - error (input parameter, resources, file I/O, XML parsing)
 */
int main(int argc, char ** argv)
{
	int		ix;							// Loop index

	// Initialize for extraction
	hFileInput = stdin;

	for (ix = 0; ix < MAX_ELEMENTS; ix++)
	{
		tbElements[ix].pName = NULL;
		tbElements[ix].pAttrName = NULL;
		tbElements[ix].pAttrValue = NULL;
		tbElements[ix].lenValue = 0;
		tbElements[ix].pValue = NULL;
		tbElements[ix].flags = 0;
		tbElements[ix].level = 0;
	}
	
	// Get and validate command line arguments
	getRecu_opt(argc, argv, tbShortOptions, tbOptions);

	dispHeaderRecord(argc, argv);

	// Output progress report if g_verbose
	if (g_verbose)
		fprintf(stderr, NAME_PROG ": Parsing XML buffer\n");

	if ( IXmlParseFile(hFileInput, nameFileInput, IXML_PARSER_FLAG_NONE, tbFields, NULL, NULL, NULL, NULL, NULL, NULL) !=
			FSUCCESS )
	{
		fprintf(stderr, NAME_PROG ": XML Parse error\n");
		exit(1);
	}

	if (hFileInput && (hFileInput != stdin))
		fclose(hFileInput);

	return (g_exitstatus);

}	// End of main()

void ProcessMatchedElement( ELEMENT_TABLE_ENTRY *pElement, char *pValue,
	int length, int level )
{
	if (!length)
	{
		// Existing element value being replaced by NULL
		if (pElement->lenValue > 0)
		{
			dispExtractionRecord();
			pElement->pValue[0] = 0;
			pElement->level = level;
			flElementChange = TRUE;
		}
	}
	else
	{
		// Null element value being replaced by new value
		if (!pElement->lenValue)
		{
			// Allocate block of memory x2
			pElement->pValue = mymalloc(length + length, "Value");
			strcpy(pElement->pValue, pValue);
			pElement->lenValue = length + length;
			pElement->level = level;
			flElementChange = TRUE;
		}
		else if (pElement->lenValue < 0)
		{
			if (-pElement->lenValue <= length)
			{
				// Reallocate to larger block of memory x2
				pElement->pValue =
						myrealloc(pElement->pValue, length + length, "Value");
				pElement->lenValue = -(length + length);
			}

			strcpy(pElement->pValue, pValue);
			pElement->lenValue = -pElement->lenValue;
			pElement->level = level;
			flElementChange = TRUE;
		}

		else
		{
			// Existing element value being replaced by new (non-equal) value
			if (strcmp(pElement->pValue, pValue))
			{
				dispExtractionRecord();
				if (pElement->lenValue <= length)
				{
					// Reallocate to larger block of memory x2
					pElement->pValue =
							myrealloc(pElement->pValue, length + length, "Value");
					pElement->lenValue = length + length;
				}

				strcpy(pElement->pValue, pValue);
				pElement->level = level;
				flElementChange = TRUE;

			}	// End of if (strcmp(pElement->pValue, pValue))

		}	// End of else (else if (pElement->lenValue < 0))

	}	// End of else (!length)

}	// End of ProcessMatchedElement()

/*******************************************************************************
 *
 * procElementBeg()
 *
 * Description:
 *	Process begin element specification.  Check for whether element suppresses
 *	extraction.
 *
 * Inputs:
 *	pParserState - Pointer to IXml parser state
 *	     pParent - Pointer to element parent
 *	    ppAttrib - Pointer to pointer to element attribute data
 *
 * Outputs:
 *	none
 */
void *procElementBeg(IXmlParserState_t *pParserState, void *pParent, const char **ppAttrib)
{
	int		ix;
	char	*pNameFull;
	ELEMENT_LIST* pList = NULL;
	char *pValue;
	int length;

	if (levelParse++ > MAX_LEVEL_PARSE)
	{
		fprintf( stderr, NAME_PROG ": Max Parsing Level (%d) Exceeded\n",
			MAX_LEVEL_PARSE );
		exit(1);
	}

	pNameFull = (char *)IXmlParserGetCurrentFullTag(pParserState);
	ix = numElementsTable - 1;
	do {
		if ((ix = findElement(ix, pNameFull, ppAttrib, &pValue, &length)) >= 0)
		{
#if ! SUPPRESS_AND_EXTRACT
			if (pList && ix >= numElementsExtract) // Check for suppression Element
			{
				int i;

				// discard matched extractions; replace with this suppression (below)
				for (i=0; i<pList->numElements; i++)
				{
					if (pList->Elements[i].pAttrValue)
						free(pList->Elements[i].pAttrValue);
				}
				free(pList);
				pList=NULL;
			}
#endif
			// Process extraction of attribute value immediately so that value
			//  is available with (potential) extraction values of nested lines
			if (pValue && ix < numElementsExtract)
			{
				dispExtractionRecord();
				ProcessMatchedElement( &tbElements[ix], pValue, length,
					levelParse + 1 );
				free(pValue);
			}

			// Add to pList: extraction of element value
			//               extraction of element:attribute:value
			//               any suppression specification
			else
			{
				if (pList)
				{
					// sizeof(*pList) includes 1st element
					pList = myrealloc(pList, sizeof(*pList)
											+ pList->numElements
													* sizeof(pList->Elements[0]),
											"object list");
				}
				else
				{
					pList = mymalloc(sizeof(*pList), "object list");
					pList->numElements = 0;
				}
				// findElement has malloced pValue as needed
				pList->Elements[pList->numElements].ix = ix;
				pList->Elements[pList->numElements].lenAttrValue = length;
				pList->Elements[pList->numElements].pAttrValue = pValue;
				pList->numElements++;

				// Check for suppression Element
				if (ix >= numElementsExtract)
				{
					ctElementsSuppress++;
					break;	// no use going past 1st match of a suppressed
				}
			}

			ix--;

		}	// End of if ((ix = findElement(ix, pNameFull, ppAttrib

#if ALLOW_MULTI_MATCH
	} while (ix >= 0);
#else	/* just use 1st match, like the old code did */
	} while (0);
#endif

	// return list of elements matched (NULL if none matched)
	return (void*)pList;

}	// End of procElementBeg()


/*******************************************************************************
 *
 * procElementEnd()
 *
 * Description:
 *	Process end element specification.  Check for whether element is being
 *	extracted, whether element suppresses extraction, or neither.  If the
 *	element being ended is an extraction element and is the outermost enclosing
 *	element for a group of extraction elements, output an extraction record.
 *
 * Inputs:
 *	pParserState - Pointer to IXml parser state
 *	      pField - Pointer to field
 *	     pObject - Pointer to object
 *	     pParent - Pointer to element parent
 *	      pValue - Pointer to element value (NULL terminated)
 *	      length - Length of value
 *	    ppAttrib - Pointer to pointer to element attribute data
 *	     flValid - Valid flag
 *
 * Outputs:
 *	none
 */
void procElementEnd( IXmlParserState_t *pParserState, const IXML_FIELD *pField,
	void *pObject, void *pParent, XML_Char *pValue, unsigned length,
	boolean flValid )
{
	int		ix;
	ELEMENT_TABLE_ENTRY *pElement = NULL;
	ELEMENT_LIST* pList = (ELEMENT_LIST*)pObject;

	if (pList)
	{
		// this tag Matched one or more elements
		int i;
		for (i=0; i<pList->numElements; i++)
		{
			ix = pList->Elements[i].ix;
			ASSERT(ix >= 0);
			pElement = &tbElements[ix];

			// Check for extracted element
			if (ix < numElementsExtract)
			{
				// Check for suppression, ignore matches within suppression
				if (ctElementsSuppress == 0)
				{
					// matched extraction element
					if (!pElement->pAttrName || pElement->pAttrValue)
					{
						// operate on element value
						ProcessMatchedElement( pElement, pValue, length,
							levelParse );
					}

				}	// End of if (ctElementsSuppress == 0)

			}	// End of if (ix < numElementsExtract)
			else
			{
				// Suppress extraction element
				ctElementsSuppress--;
			}

			if (pList->Elements[i].pAttrValue)
				free(pList->Elements[i].pAttrValue);

		}	// End of for (i=0; i<pList->numElements; i++)
		free(pList);

	}	// End of if (pList)

	// Merge current element change flag into last change flag
	if (!ctElementsSuppress)
		flElementChangeLast |= flElementChange;

	flElementChange = FALSE;

	// Check for end of element which encloses extracted element(s)
	for (ix = 0; ix < numElementsExtract; ix++)
	{
		if (tbElements[ix].level > levelParse)
		{
			dispExtractionRecord();
			tbElements[ix].lenValue = -tbElements[ix].lenValue;
			tbElements[ix].pValue[0] = 0;
		   	tbElements[ix].level = 0;
		}
	}

	levelParse--;

}	// End of procElementEnd()


// End of file

