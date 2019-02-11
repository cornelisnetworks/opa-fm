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

** END_ICS_COPYRIGHT5   ****************************************/

//==============================================================================//
//                                                                              //
// FILE NAME                                                                    //
//    config_convert.c                                                          //
//                                                                              //
// DESCRIPTION                                                                  //
//    Convert an iview_fm.config file to a opafm.xml file using
//    opafm_src.xml file as a guide on how to translate
//    This capability used to be used for upgrades from old iview_fm
//    releases, but is also a convenient way to generate new FM config
//    files with just a few changes relative to the default.
//    The FM config generate tools use this mechanism.
//                                                                              //
// opafm_src.xml (SRC) lines can be in one of the following formats:
// <tagOut>valueOut</tagOut>         <!--oldKey-->
// <tagOut>valueOut</tagOut>         <!--oldKey==defaultValue-->
// <tagOut>valueOut</tagOut>         <!--oldKey^=defaultValue-->
// <tagOut>valueOut</tagOut>         <!--oldKey^=-->
// <!--<tagOut>valueOut</tagOut>-->  <!--oldKey-->
// <!--<tagOut>valueOut</tagOut>-->  <!--oldKey==defaultValue-->
// <!--<tagOut>valueOut</tagOut>-->  <!--oldKey^=defaultValue-->
// <!--<tagOut>valueOut</tagOut>-->  <!--#oldKey-->
// <!--<tagOut>valueOut</tagOut>-->  <!--#oldKey==defaultValue-->
// <!--<tagOut>valueOut</tagOut>-->  <!--#oldKey^=defaultValue-->
// <!--<tagOut>valueOut</tagOut>-->  <!--#oldKey^=-->
// text                              <!--#oldKey#=-->
// text                              <!--#oldKey#!-->
// text with nothing in col 120+
//
// iview_fm.config (OLD) is old style format, essentially a bash script of
// comments or oldKey=oldValue
//
// oldValue conversion:
// oldKey can optionally include a format conversion.  Format conversion
// occurs for the oldValue prior to comparing it to valueOut and defaultValue
// The following format conversions are supported:
// :hex - output as hex
// :dec - output as dec
// :mtu - convert an MTU enum to byte count (256, 2048, ...)
// :rate - convert a rate enum to rate text (5g, 10g, ...)
// :plus - add one to old value
// :gid - convert 128 bit number into 64:64 format
// :2n - convert a 2^n timeout multiplier to actual time (8ms, 33ms, ...)
// :2ninf - convert a 2^n timeout multiplier to actual time (8ms, 33ms, ...)
//          but, values >= IB_LIFETIME_MAX (19) output as "infinite"
//
// Rules for conversion:
//  - SRC text >= col 120 is never output, nor is trailing whitespace <= col 119
//  - SRC Lines with nothing in col 120+ are output as is
//  - for lines with oldKey control in col 120+, comparision is described
//    below and will control edit/deletion of lines based on oldKey's value
//  - lines with text in col 120+ which do not match one of the above formats
//    is a fatal error.  <!-- must start in exactly col 120 (tab characters not
//    allowed).
//
// Summary of meanings:
// <!--#oldKey...
//        line will be deleted if oldKey "matches"
// ...oldKey-->
//        comparison is against valueOut. If no match, output oldValue
// ...oldKey==defaultValue-->
//        comparison is against valueOut and defaultValue. Match if == either.
//        If no match, output oldValue
// ...oldKey^=defaultValue-->
//        comparison is exclusively against defaultValue.
//        If no match, output oldValue
// ...oldKey^=-->
//        Only test oldKey presence. If present, output oldValue.
// ...#oldKey#=-->
//        Only test oldKey presence. If present, output unmodified text
//        Otherwise delete line.
// ...#oldKey#!-->
//        Only test oldKey presence. If not present, output unmodified text
//        Otherwise delete line.
//
// Usage model:
// <tagOut>valueOut</tagOut>         <!--oldKey-->
// <!--<tagOut>valueOut</tagOut>-->  <!--oldKey-->
// <!--<tagOut>valueOut</tagOut>-->  <!--#oldKey-->
//     use one of the above when valueOut has same default as OldValue did
//
//     OldKey is compared to valueOut, if oldKey missing, treat as match
//     First - replace with oldKey's value if no match
//     Second - uncomment with oldKey's value if no match
//     Third - if match, delete line.
//     			Otherwise uncomment with oldKey's value
//
// <tagOut>valueOut</tagOut>         <!--oldKey==defaultValue-->
// <!--<tagOut>valueOut</tagOut>-->  <!--oldKey==defaultValue-->
// <!--<tagOut>valueOut</tagOut>-->  <!--#oldKey==defaultValue-->
//     use one of the above when valueOut has a different default
//     than oldValue did and want to keep new default unless user edited
//     oldValue in old config file
//     Also in this case, if oldKey has valueOut, config left as default
//     (possibly commented out)
//
//     OldKey is compared to valueOut and defaultValue treat as == either,
//     if oldKey missing treat as match
//     First - replace with oldKey's value if no match
//     Second - uncomment with oldKey's value if no match
//     Third - if match, delete line.
//     			Otherwise uncomment with oldKey's value
//
// <tagOut>valueOut</tagOut>         <!--oldKey^=defaultValue-->
// <!--<tagOut>valueOut</tagOut>-->  <!--oldKey^=defaultValue-->
// <!--<tagOut>valueOut</tagOut>-->  <!--#oldKey^=defaultValue-->
//     use one of the above when valueOut has a different default
//     than oldValue did and want to keep new default unless user edited
//     oldValue in old config file
//     Also in this case, if oldKey is not compared to valueOut
//     (possibly commented out)
//     This format is most useful for showing commented out samples
//     were sample valueOut is not defaultValue
//
//     OldKey is only compared to defaultValue, if oldKey missing
//     treat as match
//     First - replace with oldKey's value if no match
//     Second - uncomment with oldKey's value if no match
//     Third - if match, delete line.
//     			Otherwise uncomment with oldKey's value
//
// <tagOut>valueOut</tagOut>         <!--oldKey^=-->
// <!--<tagOut>valueOut</tagOut>-->  <!--oldKey^=-->
// <!--<tagOut>valueOut</tagOut>-->  <!--#oldKey^=-->
//     use one of the above when valueOut was commented out in old config
//     and want to keep new example unless user provided an oldValue
//     in old config file
//     Also in this case, if oldKey is not compared to valueOut
//     (possibly commented out)
//     This format is most useful for showing commented out samples
//     were sample valueOut was commented out in old config
//
//     OldKey is only tested for presence in old config
//     First - replace with oldKey's value if oldKey present
//     Second - uncomment with oldKey's value if oldkey present
//     Third - if oldKey not present, delete line.
//     			Otherwise uncomment with oldKey's value
//
// text                              <!--#oldKey#=-->
//     use this for any text (comments, block start tags, block end tags)
//     which should be output only if oldKey exists in the old config file
//
//     oldKey is only tested for presence in old config
//     text is output (unchanged) if oldKey is present, otherwise delete line
//
// text                              <!--#oldKey#!-->
//     use this for any text (comments, block start tags, block end tags)
//     which should be output only if oldKey not in the old config file
//
//     oldKey is only tested for presence in old config
//     text is output (unchanged) if oldKey not present, otherwise delete line
//
// The #oldKey style should be used for lines in SRC which are for conversion
// but are not desired in the opafm.xml file unless required
// This can be used to avoid clutter while still supporting full conversion
// of old parameters
//
// DATA STRUCTURES                                                              //
//    None                                                                      //
//                                                                              //
// FUNCTIONS                                                                    //
//    main        main method that convert the old ocnfig file into the         //
//                      new XML format.                                         //
//                                                                              //
// DEPENDENCIES                                                                 //
//    ???                                                                       //
//                                                                              //
//                                                                              //
// HISTORY                                                                      //
//                                                                              //
//    NAME    DATE  REMARKS                                                     //
//     pjs   10/15/08  Initial creation of file.                                //
//                                                                              //
//==============================================================================//

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "cs_g.h"
#include "iba/ib_helper.h"
#include "iba/stl_helper.h"

#ifndef FALSE
	#define FALSE 0
#endif

#ifndef TRUE
	#define TRUE 1
#endif


#define MAX_INSTANCES 8
#define MAX_DETAIL    2048
#define MAX_START     256
#define MAX_UNIQUE    1024

typedef struct _Ag_Man {
	uint8_t		instance;		// instance number
	uint8_t		system [4];		// Agent or Manager identifier
	uint8_t		key [100];		// key
	uint8_t		value [100];		// value
} Ag_Man_t;

uint8_t		debug;				// debugging mode - default is FALSE

char		config_file [256];		// location and name of the config file
uint8_t		read_buffer [256];		// a buffer for reading the file a line-at-a-time
uint8_t		buffer [256];			// cleaned up read_buffer
uint32_t	line_number;			// line number in config file

uint8_t		line_instance;			// instance number
uint8_t		line_system [4];		// Agent or Manager
uint8_t		line_key [100];			// key, of key=value pair
uint8_t		line_value [200];		// value, of key=value pair

uint16_t	ag_man_detail_count;		// number of entries - detail record
uint16_t	ag_man_start_count;		// number of entries - start records
uint16_t	ag_man_unique_count;		// number of entries - unique system/instance pairs

Ag_Man_t	*ag_man_detail;			// Agent and Manager information - detail records
Ag_Man_t	*ag_man_start;			// Agent and Manager information - start records
Ag_Man_t	*ag_man_unique;			// Agent and Manager information - unique system/instance
Ag_Man_t	*ag_manp;			// Agent and Manager information - temp pointer

FILE		*fd_config;			// file handle for reading the config file


//	STARTUP-related function prototypes.

Status_t 	readAndParseConfigFile		(bool_t debug);

#define OLD_TAG_COL 120
void dump_ap(char *desc,int line, Ag_Man_t *ap);

void configConvertUsage(void);
Status_t convertSrcXmlFile (char *,int);
Ag_Man_t * find_detail_rec (char *system, int instance, char*key); 
int parseLine(char *lineIn,int lineNumber, char *tagOut, char *valueOut, char *first, char *last, char* firstTag, char* lastTag, char *oldKey, char *defaultValue, int *exclusiveDefault, int *presence, int buf_sizes);
char* getOldValue(char *keyDescIn,int);
int matching_system (Ag_Man_t *ag_start, char *which_one);

int g_debug = FALSE;

//------------------------------------------------------------------------------//
//                                                                              //
// Routine      : main (entry point for Linux)                                  //
//                                                                              //
// Description  : Entry point for utility                                       //
// Pre-process  : None                                                          //
// Input        : Command-line arguments                                        //
// Output       : A general status indicator                                    //
//                  0 - conversion ran to completion                            //
//                  1 - an error was found during conversion                    //
// Post-process : Call startup_main function readAndParseConfigFile to parse the//
//                vieo_fm.rc.config file then call convertSrcXmlFile to write   //
//                out the XML data to stdout                                    //
// Reference    :                                                               // 
//                                                                              //
//------------------------------------------------------------------------------//

int main (int argc, char *argv []) 
{
    Status_t    status;        // return status from startup_main
    int c;
    char* oldFile;
    char* xmlFile;
    int showMissing=FALSE;

    while (-1 != (c = getopt(argc, argv, "dD"))) {
        switch (c) {
            case 'd':
                showMissing = TRUE;
                break;
            case 'D':
                g_debug = TRUE;
                break;
            default:
                configConvertUsage();
                break;
        }
    }
    if (argc - optind != 2) {
        configConvertUsage();
    }
    oldFile = argv[optind++];
    xmlFile = argv[optind++];

//
//    Default values.
//
    
    debug       = FALSE;
    line_number = 0;
    
    snprintf(config_file, 256, "%s", oldFile);
    
    status=readAndParseConfigFile(FALSE);
    if (status!=VSTATUS_OK)
    {
        exit (status);
    }

    status=convertSrcXmlFile(xmlFile,showMissing);
    exit(status);
}

//------------------------------------------------------------------------------//
//                                                                              //
// Routine      : configConvertUsage                                            //
//                                                                              //
// Description  : Print the usage parameters for config_convert.                //
// Pre-process  : None                                                          //
// Input        : None                                                          //
// Output       : None                                                          //
// Post-process : None                                                          //
// Reference    :                                                               //
//                                                                              //
//------------------------------------------------------------------------------//

void configConvertUsage (void) 
{
    fprintf (stderr,"Usage: config_convert [-d] [-D] old_file src_xml_file\n");
    fprintf (stderr,"   -d                 show tags not found in old file\n");
    fprintf (stderr,"   -D                 extra debug output to stderr\n");
    fprintf (stderr,"   old_file           old iview_fm.config style file to convert\n");
    fprintf (stderr,"   src_xml_file       new opafm_src.xml style file to guide conversion\n");
    fprintf (stderr,"The old_file (typically iview_fm.config) from an older release is converted\n");
    fprintf (stderr,"to an XML file for this release, using src_xml_file\n");
    fprintf (stderr,"(typically opafm_src.xml) to define how to translate parameters\n");
    exit(2);
}

//------------------------------------------------------------------------------//
//                                                                              //
// Routine      : convertSrcXmlFile                                             //
//                                                                              //
// Description  : read in the xml src file and replace the tags in col with the //
//          spaces and put the data from the old config into the tag on the left//
//
// Pre-process  : None                                                          //
// Input        : None                                                          //
// Output       : None                                                          //
// Post-process : None                                                          //
// Reference    :                                                               //
//                                                                              //
//------------------------------------------------------------------------------//

Status_t convertSrcXmlFile (char *xmlFileIn,int showMissing) 
{
    char buffIn[255];

    FILE *xmlIn=fopen(xmlFileIn,"rt");
    if (xmlIn==NULL)
    {
        fprintf(stderr,"Unable to open file %s\n",xmlFileIn);
        exit(1);
    }
    int lineNumber=1;
    while (!feof(xmlIn))
    {
        if (fgets(buffIn,255,xmlIn)==0)
        {
            break;
        }
        char tag[255];
        char value[255];
        char first[255];
        char last[255];
        char firstTag[255];
        char lastTag[255];
        char oldKey[255];
        char defaultValue[255];
        int exclusiveDefault;
        int presence;

        if (g_debug)
            fprintf(stderr, "%s\n", buffIn);
        if (parseLine(buffIn,lineNumber++,tag,value,first,last,firstTag, lastTag, oldKey,defaultValue, &exclusiveDefault, &presence, 255))
        {
            char* oldValue=getOldValue(oldKey,showMissing);
            if (g_debug) {
                fprintf(stderr, "tag         =%s\n", tag);
                fprintf(stderr, "value       =%s\n", value);
                fprintf(stderr, "first       =%s\n", first);
                fprintf(stderr, "last        =%s\n", last);
                fprintf(stderr, "firstTag    =%s\n", firstTag);
                fprintf(stderr, "lastTag     =%s\n", lastTag);
                fprintf(stderr, "oldKey      =%s\n", oldKey);
                fprintf(stderr, "defaultValue=%s\n", defaultValue);
                fprintf(stderr, "oldValue    =%s\n", (oldValue)?oldValue:"<null>");
                fprintf(stderr, "exclusive   =%d\n", exclusiveDefault);
                fprintf(stderr, "presence    =%d\n", presence);
            }
            if (presence > 0) {
                // text                              <!--#oldKey#=-->
                // key starting with # means to skip line if old not present
                if (oldValue || oldKey[0] != '#') {
                    fprintf(stdout,"%s\n",first);
                    if (g_debug) fprintf(stderr,"%s\n",first);
                }
            } else if (presence < 0) {
                // text                              <!--#oldKey#!-->
                // key starting with # means to skip line if old present
                if (! oldValue || oldKey[0] != '#') {
                    fprintf(stdout,"%s\n",first);
                    if (g_debug) fprintf(stderr,"%s\n",first);
                }
            } else if (oldValue==NULL || (! exclusiveDefault && 0==strcmp(value,oldValue)))
            {
                // parameter not in old config or same as new config
                // key starting with # means to skip line if default
                if (oldKey[0] != '#') {
                    // output default XML
                    fprintf(stdout,"%s%s%s\n",first,value,last);
                    if (g_debug) fprintf(stderr,"%s%s%s\n",first,value,last);
                }
            } else if (strlen(defaultValue)==0) {
                // <tagOut>valueOut</tagOut>         <!--oldKey-->
                // <tagOut>valueOut</tagOut>         <!--oldKey==-->
                // <tagOut>valueOut</tagOut>         <!--oldKey^=-->
                // <!--<tagOut>valueOut</tagOut>-->  <!--oldKey-->
                // <!--<tagOut>valueOut</tagOut>-->  <!--oldKey==-->
                // <!--<tagOut>valueOut</tagOut>-->  <!--oldKey^=-->
                // value doesn't match, no defaultValue
                // output XML replacing valueOut with oldValue
                fprintf(stdout,"%s%s%s\n",firstTag,oldValue,lastTag);
                if (g_debug) fprintf(stderr,"%s%s%s\n",firstTag,oldValue,lastTag);
            } else if (strcmp(defaultValue,oldValue)==0) {
                // <tagOut>valueOut</tagOut>         <!--oldKey==defaultValue-->
                // <tagOut>valueOut</tagOut>         <!--oldKey^=defaultValue-->
                // <!--<tagOut>valueOut</tagOut>-->  <!--oldKey==defaultValue-->
                // <!--<tagOut>valueOut</tagOut>-->  <!--oldKey^=defaultValue-->
                // oldValue matches defaultValue
                // key starting with # means to skip line if default
                if (oldKey[0] != '#') {
                    // output default XML
                    fprintf(stdout,"%s%s%s\n",first,value,last);
                    if (g_debug) fprintf(stderr,"%s%s%s\n",first,value,last);
                }
            } else {
                // <tagOut>valueOut</tagOut>         <!--oldKey==defaultValue-->
                // <tagOut>valueOut</tagOut>         <!--oldKey^=defaultValue-->
                // <!--<tagOut>valueOut</tagOut>-->  <!--oldKey==defaultValue-->
                // <!--<tagOut>valueOut</tagOut>-->  <!--oldKey^=defaultValue-->
                // oldValue doesn't match defaultValue
                // output XML replacing valueOut with oldValue
                fprintf(stdout,"%s%s%s\n",firstTag,oldValue,lastTag);
                if (g_debug) fprintf(stderr,"%s%s%s\n",firstTag,oldValue,lastTag);
            }
        }
        else
        {
            // this line doens't have a tag in col 120 so dump it back out
            fprintf(stdout,"%s",buffIn);
        }
    }

	fclose(xmlIn);
    return VSTATUS_OK;
}


int getOldUllValue(Ag_Man_t *sp, unsigned long long *pValue)
{
    char *p;
    errno = 0;
    *pValue=strtoull((char*)sp->value, &p, 0);
    if (errno || *p) {
        fprintf(stderr,"WARNING: invalid value for %s_%d_%s=%s\n",
                        sp->system, sp->instance, sp->key, sp->value);
        return FALSE;
    }
    return TRUE;
}

typedef enum {
    FORMAT_NONE=0,
    FORMAT_HEX,
    FORMAT_DEC,
    FORMAT_MTU,
    FORMAT_RATE,
    FORMAT_PLUS,
    FORMAT_2N,
    FORMAT_2NINF,
    FORMAT_GID,
} OldFormat_t;

char *formatOldValue(Ag_Man_t *sp, OldFormat_t format)
{
    static char buf[50];    // used for edited values
    unsigned long long value;
    unsigned long long value2;

    switch (format) {
    default:
    case FORMAT_NONE:
        return (char*)sp->value;
    case FORMAT_HEX:
        if (FALSE == getOldUllValue(sp, &value))
            return (char*)sp->value;
        snprintf(buf,sizeof(buf),"0x%Lx",value);
        return buf;
    case FORMAT_DEC:
        if (FALSE == getOldUllValue(sp, &value))
            return (char*)sp->value;
        snprintf(buf,sizeof(buf),"%Ld",value);
        return buf;
    case FORMAT_MTU:
        if (FALSE == getOldUllValue(sp, &value))
            return (char*)sp->value;
        snprintf(buf, 50, "%s", IbMTUToText((IB_MTU)value));
        return buf;
    case FORMAT_RATE:
        if (FALSE == getOldUllValue(sp, &value))
            return (char*)sp->value;
        if (value == IB_STATIC_RATE_DONTCARE)
            return (char*)sp->value;
        snprintf(buf,sizeof(buf),"%s",StlStaticRateToText((IB_STATIC_RATE)value));
        return buf;
    case FORMAT_PLUS:
        if (FALSE == getOldUllValue(sp, &value))
            return (char*)sp->value;
        snprintf(buf,sizeof(buf),"%lld",value+1);
        return buf;
    case FORMAT_2N:
    case FORMAT_2NINF:
        // do the 2^n conversion.
        if (FALSE == getOldUllValue(sp, &value))
            return (char*)sp->value;
        if (format == FORMAT_2NINF && value > IB_LIFETIME_MAX)
            return "infinite";
        FormatTimeoutMult(buf,value);
        /* compress out whitespace */
        int s=0; int d=0;
        do {
            if (buf[s] != ' ')
                buf[d++] = buf[s];
        } while (buf[s++]);
        return buf;
    case FORMAT_GID:
        {
            char *p;

            // compress out whitespace
            int s=0; int d=0;
            do {
                if (d >= sizeof(buf)) {
                    fprintf(stderr,"WARNING: invalid value for %s_%d_%s=%s\n",
                                sp->system, sp->instance, sp->key, sp->value);
                    return (char*)sp->value;
                }
                if (sp->value[s] != ' ')
                    buf[d++] = sp->value[s];
            } while (sp->value[s++]);

            int len = strlen(buf);
            if (len != 34) {
                fprintf(stderr,"WARNING: invalid value for %s_%d_%s=%s\n",
                                sp->system, sp->instance, sp->key, sp->value);
                return (char*)sp->value;
            }

            // get last 16 hex digits
            errno = 0;
            value2=strtoull(buf+18, &p, 16);
            if (errno || *p) {
                fprintf(stderr,"WARNING invalid value for %s_%d_%s=%s\n",
                                sp->system, sp->instance, sp->key, sp->value);
                return (char*)sp->value;
            }

            // get first 16 hex digits
            buf[18] = '\0';
            errno = 0;
            value=strtoull(buf, &p, 16);
            if (errno || *p) {
                fprintf(stderr,"WARNING invalid value for %s_%d_%s=%s\n",
                                sp->system, sp->instance, sp->key, sp->value);
                return (char*)sp->value;
            }
            snprintf(buf,sizeof(buf),"0x%016llx:0x%016llx",value,value2);
            return buf;
        }
    }
}

//------------------------------------------------------------------------------//
//                                                                              //
// Routine      : getOldValue                                                   //
//                                                                              //
// Description  : Given a key like SM_0_debug retrieve the data form Ag_Man_t   //
//          structs.                                                            //
// Pre-process  : readAndParseConfigFile should have been called                //
// Input        : keyDescIn old config key with optional format descriptor      //
// Input        : showMissing ouput to stderr the keys that were not found in   //
//                the old config file. Debug only                               //
// Output       : None                                                          //
// Post-process : None                                                          //
// Reference    :                                                               //
//                                                                              //
//------------------------------------------------------------------------------//
char* getOldValue(char *keyDescIn,int showMissing)
{
    char system[3];
	static const int INSTANCE_LEN=10;
    char instance[INSTANCE_LEN+1];
	static const int KEY_LEN=50;
    char keyIn[KEY_LEN+1];
    char key[KEY_LEN+1];
    OldFormat_t format = FORMAT_NONE;

    // strip format specified off keyIn
    if (keyDescIn[0] == '#')
        strncpy(keyIn, keyDescIn+1, KEY_LEN);
    else
        strncpy(keyIn, keyDescIn, KEY_LEN);

	keyIn[sizeof(keyIn)-1]='\0';

    char *formatDelim=strchr(keyIn, ':');
    if (formatDelim) {
        *formatDelim++='\0';
        if (0==strcmp(formatDelim, "hex"))
            format = FORMAT_HEX;
        else if (0==strcmp(formatDelim, "dec"))
            format = FORMAT_DEC;
        else if (0==strcmp(formatDelim, "mtu"))
            format = FORMAT_MTU;
        else if (0==strcmp(formatDelim, "rate"))
            format = FORMAT_RATE;
        else if (0==strcmp(formatDelim, "plus"))
            format = FORMAT_PLUS;
        else if (0==strcmp(formatDelim, "2n"))
            format = FORMAT_2N;
        else if (0==strcmp(formatDelim, "2ninf"))
            format = FORMAT_2NINF;
        else if (0==strcmp(formatDelim, "gid"))
            format = FORMAT_GID;
        else {
            fprintf(stderr,"Invalid format: (%s) %s\n",formatDelim, keyDescIn);
            exit(1);
        }
    }

    // Is is of this type SS_I_KEY?
    if ((keyIn[2]=='_')&&(keyIn[4]=='_'))
    {
        //Parse out the system, instance an key
        StringCopy(system,keyIn,sizeof(system));

        char *secondUnderScore=strchr(&keyIn[3],'_');
	if (!secondUnderScore) return NULL;

        int len=(int)(secondUnderScore-&keyIn[3]);
	if (len>=INSTANCE_LEN) return NULL;

        StringCopy(instance,&keyIn[3],sizeof(instance));

        StringCopy(key,secondUnderScore+1,sizeof(key));

    //    fprintf(stdout,"tag:%s sys:%s, instance:%s, key:%s\n",keyIn,system,instance,key);
        int instanceValue=atoi(instance);
        //get the data out of the structs
        if (strncmp(key,"start",6)==0)
        {
            uint16_t    i;
            Ag_Man_t    *ap;
            // find a start record
            for (i = 0, ap = ag_man_start; i < ag_man_start_count; i++, ap++) 
            {
                if (matching_system (ap, (void *)system)) 
                {
                    if (ap->instance == instanceValue) 
                    {
                        return "1";
                    }
                }
            }
            return "0";
        }
        Ag_Man_t *sp=find_detail_rec(system,instanceValue,key);
        if (sp!=NULL)
            return formatOldValue(sp, format);
        // Couldn't find the variable. Display to the user if they set -d
        if (showMissing)
        {
            fprintf(stderr,"Unable to find %s %s,%d,%s\n",keyIn,system,instanceValue,key);
        }
    }
    else
    {
        int i=0;
        // special tags are in all caps
        for (i=0;i<strlen(keyIn);i++)
        {
            if (!isupper(keyIn[i]) && (keyIn[i]!='_'))
            {
                return NULL;
            }
        }
        // This is a specail tag like SUBNET_SIZE
        Ag_Man_t *sp=find_detail_rec("sm",0,keyIn);
        if (sp!=NULL)
            return formatOldValue(sp, format);
        // Couldn't find the variable. Display to the user if they set -d
        if (showMissing)
        {
            fprintf(stderr,"Unable to find special key %s\n",keyIn);
        }
    }
    return NULL;
}

//------------------------------------------------------------------------------//
//                                                                              //
// Routine      : find_detail_rec                                               //
//                                                                              //
// Description  : Loop through the structs and find the old config key.         //
//                                                                              //
// Pre-process  : readAndParseConfigFile should have been called                //
// Input        : system two char system key (e.g. "SM")                        //
// Input        : instance if the system key                                    //
// Input        : key - the key to be found (e.g. "debug")                      //
// Output       : a pointer to the detail record                                //
// Post-process : None                                                          //
// Reference    :                                                               //
//                                                                              //
//------------------------------------------------------------------------------//
Ag_Man_t * find_detail_rec (char *system, int instance, char*key) 
{
    int        i;
    Ag_Man_t     *ag_detail;
    
    for (i = 0, ag_detail = ag_man_detail; i < ag_man_detail_count; i++, ag_detail++) 
    {
        if (strcasecmp ((void *)system, (void *)ag_detail->system) == 0) 
        {
            if (instance == ag_detail->instance) 
            {
                if (strcasecmp((char*)ag_detail->key,key)==0)
                {
                    return ag_detail;
                }
            }
        }
    }
    
    return NULL;
}

//------------------------------------------------------------------------------//
//                                                                              //
// Routine      : parseLine                                                     //
//                                                                              //
// Description  : Parse a line of the src xml file                              //
// Lines can be in one of 5 formats:
// <tagOut>valueOut</tagOut>         <!--oldKey-->
// <tagOut>valueOut</tagOut>         <!--oldKey==defaultValue-->
// <tagOut>valueOut</tagOut>         <!--oldKey^=defaultValue-->
// <!--<tagOut>valueOut</tagOut>-->  <!--oldKey-->
// <!--<tagOut>valueOut</tagOut>-->  <!--oldKey==defaultValue-->
// <!--<tagOut>valueOut</tagOut>-->  <!--oldKey^=defaultValue-->
// text with nothing in col 120+
//
// Note: <!--oldKey starts in OLD_TAG_COL (120)
// Note: # prefix to oldKey handled by caller
//                                                                              //
// Pre-process  : none                                                          //
// Input        : line - raw xml line                                           //
// Input        : lineNumber - line number of the file                          //
// Output       : tagOut - name of XML tag on line                              //
// Output       : valueOut - value of XML tag on line                           //
// Output       : first - the start of the XML line up to "valueOut" on the line//
// Output       : firstTag - the start of the XML line up to "valueOut" on the line//
//                           with any comment open removed                      //
// Output       : last - the end of the XML line, after "valueOut"              //
// Output       : lastTag - the end of the XML line, after "valueOut"              //
//                           with any comment close removed                      //
// Output       : oldKey - the old key in the comment at col 120
// Output       : exclusiveDefault - only compare old key to default
// Output       : return a 0 if the line is does not have text after col 120 or 1 if it does//
// Output       : Size of char buffers - all expected to have same size         //
// Post-process : None                                                          //
// Reference    :                                                               //
//                                                                              //
//------------------------------------------------------------------------------//
int parseLine(char *line, 
              int lineNumber, 
              char *tagOut, 
              char *valueOut, 
              char *first, 
              char *last, 
              char *firstTag, 
              char *lastTag, 
              char *oldKey,
              char *defaultValue,
              int *exclusiveDefault,
              int *presence,
			  int buf_size)
{
    *exclusiveDefault=0;
    *presence=0;
    if (strlen(line)<OLD_TAG_COL)
    {
        // simple XML, no 120+ conversion info
        return 0;
    }
    // examine col 120 and above.  We expect one of these formats:
    // <!--oldKey-->
    // <!--oldKey==defaultValue-->
    // <!--oldKey^=defaultValue-->
    if (strstr(&line[OLD_TAG_COL-1],"<!--")!=&line[OLD_TAG_COL-1])
    {
        fprintf(stderr,"Line longer than %d char without comment tag at column 120. Line %d\n",OLD_TAG_COL, lineNumber);
        exit(1);
    }
    char *endOfComment=strstr(&line[OLD_TAG_COL+3],"-->");
    if (endOfComment==NULL)
    {
        fprintf(stderr,"Missing close of comment on line %d\n",lineNumber);
        exit(1);
    }
    int oldKeyLength=(int)(endOfComment-&line[OLD_TAG_COL+3]);
    if (oldKeyLength<1)
    {
        fprintf(stderr,"Empty comment starting at column %d on line %d\n",OLD_TAG_COL, lineNumber);
        exit(1);
    }
    char* equals=strstr(&line[OLD_TAG_COL+3],"^=");
    if (equals != NULL)
        *exclusiveDefault=1;
    else if (NULL != (equals=strstr(&line[OLD_TAG_COL+3],"#=")))
        *presence=1;
    else if (NULL != (equals=strstr(&line[OLD_TAG_COL+3],"#!")))
        *presence=-1;
    else
        equals=strstr(&line[OLD_TAG_COL+3],"==");
    if (equals!=NULL)
    {
        if (equals>=endOfComment)
        {
            fprintf(stderr,"== after close of comment on line %d\n",lineNumber);
            exit(1);
        }
        *equals='\0';
        equals+=2;
        int defLen=endOfComment-equals;
        if (defLen)
            strncpy(defaultValue,equals,defLen);
        defaultValue[defLen]='\0';
        oldKeyLength-=2+strlen(defaultValue);
    }
    else
    {
        *defaultValue='\0';
    }
    strncpy(oldKey,&line[OLD_TAG_COL+3],oldKeyLength);
    oldKey[oldKeyLength]='\0';

    char lineIn[OLD_TAG_COL+1];
    strncpy(lineIn,line,OLD_TAG_COL);
    lineIn[OLD_TAG_COL-1]='\0';

    if (*presence) {
        // text                              <!--#oldKey#=-->
        // text                              <!--#oldKey#!-->
        // simple case, we need to output first and firstTag but "" for all else
        StringCopy(first,lineIn, buf_size);
        int firstLen = strlen(first);
        while (firstLen > 1 && first[firstLen-1]==' ')
        {
            first[--firstLen]='\0';
        }
        StringCopy(tagOut,"", buf_size);
        StringCopy(valueOut,"", buf_size);
        StringCopy(last,"", buf_size);
        StringCopy(firstTag, first, buf_size);
        StringCopy(lastTag, last, buf_size);
        return 1;
    }

    // for lines with a oldKey, we expect to find either:
    // <tag>value</tag>
    // <!--<tag>value</tag>-->
    static const int TAG_LENGTH=128;
    char tag[TAG_LENGTH];
    char tag2[TAG_LENGTH];
    char *leftBracket1=strchr(lineIn,'<');
    char *startValue;
    int valueLength;
    if (leftBracket1==NULL)
    {
        fprintf(stderr,"Missing start of tag on line %d\n%s\n",lineNumber,line);
        exit(1);
    }
    if (leftBracket1[1]=='!')
    {
        //then this is the start of a comment. skip ahead to the tag.
        leftBracket1=strchr(leftBracket1+1,'<');
        if (leftBracket1==NULL)
        {
            fprintf(stderr,"Missing start of tag on line %d\n%s\n",lineNumber,line);
            exit(1);
        }
    }
    char*rightBracket1=strstr(leftBracket1,">");
    if (rightBracket1==NULL)
    {
        fprintf(stderr,"Left bracket with no right bracket on line %d\n%s\n",lineNumber,line);
        exit(1);
    }
    int tagLength=(int)(rightBracket1-leftBracket1)-1;
    if (tagLength<0||tagLength>TAG_LENGTH)
    {
        fprintf(stderr,"Invalid tag length %d\n%s\n",lineNumber,line);
        exit(1);
    }
    StringCopy(tag,leftBracket1+1,tagLength < sizeof(tag) ? (tagLength+1) : sizeof(tag));
    startValue=rightBracket1+1;
    char *leftBracket2=strstr(rightBracket1+1,"</");
    if (leftBracket2==NULL)
    {
        fprintf(stderr,"No closing tag on line %d\n%s\n",lineNumber,line);
        exit(1);
    }
    valueLength=leftBracket2-startValue;
    char *rightBracket2=strstr(leftBracket2,">");
    if (rightBracket2==NULL)
    {
        fprintf(stderr,"No right bracket on closing tag on line %d\n%s\n",lineNumber,line);
        exit(1);
    }
    int tagLength2=(int)((rightBracket2-leftBracket2)-2);
    if (tagLength!=tagLength2)
    {
        fprintf(stderr,"Closing tag does not match opening tag on line %d. %d!=%d\n%s\n",lineNumber,tagLength,tagLength2,line);
        fprintf(stderr,"left=%s right=%s",tag,rightBracket2);
        exit(1);
    }
    StringCopy(tag2,leftBracket2+2,tagLength < sizeof(tag2) ? (tagLength+1) : sizeof(tag2));
    if (strncmp(tag,tag2,tagLength)!=0)
    {
        fprintf(stderr,"Closing tag does not match opening tag on line %d. %s!=%s\n%s\n",lineNumber,tag,tag2,line);
        exit(1);
    }
    int firstLen=rightBracket1-lineIn+1;
    strncpy(first,lineIn,firstLen);
    first[firstLen]='\0';

    int lastLen=strlen(lineIn)-(firstLen+valueLength);
    strncpy(last,&lineIn[firstLen+valueLength],lastLen);
    last[lastLen]='\0';
    while (lastLen > 1 && last[lastLen-1]==' ')
    {
        last[--lastLen]='\0';
    }

    strncpy(tagOut,tag,255);
    strncpy(valueOut,startValue,valueLength);
    valueOut[valueLength]='\0';

    // output firstTag and lastTag as Uncommented versions of first and last
    char *commentStart=strstr(first,"<!--"); // start of comment open
    if (commentStart==NULL) {
        // no commented out tag
        StringCopy(firstTag, first, buf_size);
        StringCopy(lastTag, last, buf_size);
    } else {
        // remove the comment, keep everything else
        strncpy(firstTag, first, commentStart-first);
        char *tagStart=strchr(commentStart+4,'<');    // start of tag
        if (tagStart==NULL)
        {
            fprintf(stderr,"Missing open tag for commented out tag. Line %d\n",lineNumber-1);
            exit(1);
        }
        StringCopy(firstTag+(commentStart-first), tagStart, buf_size - (commentStart-first));

        // find the end of the comment
        char *tagEnd=strchr(last,'>');    // end of tag
        if (tagEnd==NULL)
        {
            fprintf(stderr,"Missing close tag for commented out tag. Line %d\n",lineNumber-1);
            exit(1);
        }
        commentStart=strstr(tagEnd+1,"-->");    // start of comment close
        if (commentStart==NULL)
        {
            fprintf(stderr,"Missing close comment for commented out tag. Line %d\n",lineNumber-1);
            exit(1);
        }
        StringCopy(lastTag, last, commentStart-last+1);
        StringCopy(lastTag+(commentStart-last), commentStart+3, buf_size - (commentStart-last));
        lastLen = strlen(lastTag);
        while (lastLen > 1 && lastTag[lastLen-1]==' ')
        {
            lastTag[--lastLen]='\0';
        }

    StringCopy(tagOut,tag,255);
    }

    return 1;
}

//------------------------------------------------------------------------------//
//                                                                              //
// Routine      : dump_ap                                                       //
// Description  : dump a record of Ag_Man_t to stderr (for debug only)          //
//                                                                              //
// Pre-process  : none                                                          //
//------------------------------------------------------------------------------//
void dump_ap(char *desc,int line, Ag_Man_t *ap) 
{
    fprintf(stderr, "%s(%d)= instance %u system %s key %s value %s", desc,line,
            (uint32_t)ap->instance, 
            (char*)ap->system, (char*)ap->key, ap->value);
}


//==============================================================================//
//    This parses the vieo_fm.config file
//										//

Status_t	alloc_mem		(void);
void		free_mem		(void);
Status_t	read_line		(void);
Status_t	parse			(void);
int		is_special		(void);
int		is_valid		(void);
int		is_manager		(void);
Status_t	add_information		(void);
int		matching_system		(Ag_Man_t *ag_start, char *which_one);
void		log_debug		(char *format, ...);
void		log_error		(char *format, ...);

//------------------------------------------------------------------------------//
//										//
// Routine      : alloc_mem							//
//										//
// Description  : Allocate memory.						//
// Input        : None								//
// Output       : Status of the startup process					//
//                  0 - memory was allocated					//
//                  1 - error allocating memory					//
//										//
//------------------------------------------------------------------------------//

Status_t
alloc_mem (void) {
	
//
//	Allocate enough space for 2,048 lines of detailed config information.
//
	
	ag_man_detail = (Ag_Man_t*)malloc (MAX_DETAIL * sizeof (Ag_Man_t));
	if (NULL == ag_man_detail) {
		return (VSTATUS_BAD);
	}
	ag_manp = ag_man_detail;
	ag_man_detail_count = 0;
	
//
//	Allocate enough space for 256 Agents and Managers to start
//
	
	ag_man_start = (Ag_Man_t*)malloc (MAX_START * sizeof (Ag_Man_t));
	if (NULL == ag_man_start) {
		free ((void *)ag_man_detail);
		return (VSTATUS_BAD);
	}
	ag_man_start_count = 0;
	
//
//	Allocate enough space for 1,024 unique system/instance.
//
	
	ag_man_unique = (Ag_Man_t*)malloc (MAX_UNIQUE * sizeof (Ag_Man_t));
	if (NULL == ag_man_unique) {
		free ((void *)ag_man_start);
		free ((void *)ag_man_detail);
		return (VSTATUS_BAD);
	}
	ag_man_unique_count = 0;
	
	return (VSTATUS_OK);
}


//------------------------------------------------------------------------------//
//										//
// Routine      : free_mem							//
//										//
// Description  : Free up allocated memory.					//
// Input        : None								//
// Output       : None								//
//										//
//------------------------------------------------------------------------------//

void
free_mem (void) {
	
	free ((void *)ag_man_unique);
	free ((void *)ag_man_start);
	free ((void *)ag_man_detail);
	
	return;
}


//------------------------------------------------------------------------------//
//										//
// Routine      : read_line							//
//										//
// Description  : Read a line of input, ignoring blank lines and lines that	//
//                start with a pound sign (#).					//
// Input        : None								//
// Output       : Status of the startup process					//
//                  0 - a line of content was read				//
//                  1 - found EOF						//
//										//
//------------------------------------------------------------------------------//

Status_t
read_line (void) {
	uint8_t		*slash;			// start of line comments
	uint8_t		*start;			// start of read buffer
	uint8_t		*stop;			// end of read buffer
	Status_t	status;			// return code status
	
	while (TRUE) {
		// clean memory is good memory
		memset (read_buffer, 0, 256);
		memset (buffer,      0, 256);
		
		// NULL means we read past EOF
		if (fgets ((void *)read_buffer, 255, fd_config) == NULL) {
			status = VSTATUS_BAD;
			break;
		}
		line_number++;
		
		start = read_buffer;
		stop  = read_buffer + strlen ((void *)read_buffer);
		
		// terminate after line comments (//)
		slash = (uint8_t *)strstr ((void *)start, "//");
		if (slash != NULL) {
			*slash = '\0';
			stop = read_buffer + strlen ((void *)read_buffer);
		}
		
		// remove leading whitespace
		while (isspace (*start)) {
			start++;
		}
		
		// ignore lines that start with a pound sign (#)
		if (*start == '#') {
			continue;
		}
		
		// remove trailing whitespace
		while (stop > start && isspace (*--stop)) {
			*stop = '\0';
		}
		
		// ignore blank lines
		if (*start == '\0') {
			continue;
		}
		
		// we have a line of input to work with
		strncpy ((void *)buffer, (void *)start, 256);
		
		// see if we found an EOF (used by ATI)
		if (strncasecmp ((void *)buffer, "EOF", 3) == 0) {
			status = VSTATUS_BAD;
		} else {
			status = VSTATUS_OK;
		}
		break;
	}
	
	return (status);
}

//------------------------------------------------------------------------------//
//										//
// Routine      : parse								//
//										//
// Description  : Parse a line of input and create data structures.		//
// Input        : None								//
// Output       : None, bad lines are printed and dropped			//
//										//
//------------------------------------------------------------------------------//

Status_t
parse (void) {
	Status_t	status;
	
//
//	Capture special lines before we toss them, i.e., VIEO_FM_BASE=some_directory.
//
	
	if (is_special () == TRUE) {
		return (VSTATUS_OK);
	}
	
//
//	Might be a good line, see if it's a valid format (system_instance_key=value).
//
	
	if (is_valid () == FALSE) {
//		log_debug ("Warning: invalid configuration data, line %d\nLine   : %s\n\n", line_number, read_buffer);
		return (VSTATUS_OK);
	}
	
//
//	Add this information to the data structures
//
	
	status = add_information ();
	
	return (status);
}

static void setBase(char *mgr, int instance) {
    memset (line_system, 0, 4);
    strncpy ((void *)line_system, mgr, 2);
    line_instance = instance;
    (void) add_information ();
}
//------------------------------------------------------------------------------//
//										//
// Routine      : is_special							//
//										//
// Description  : Look for 'special' lines.					//
// Input        : None								//
// Output       : TRUE  - special line found					//
//                FALSE - nothing special here					//
//										//
//------------------------------------------------------------------------------//

int
is_special (void) {
	char	*equals;			// location of equals sign (=)
    uint8_t subnetSize[16];		// subnet size
    uint8_t syslogMode[16];		// syslog mode
	int		i;

	if (strncasecmp ((void *)buffer, "HARDWARE_MFG", 12) == 0) {
		return (TRUE);
	} else if (strncasecmp ((void *)buffer, "HARDWARE_TYPE", 13) == 0) {
		return (TRUE);
	} else if (strncasecmp ((void *)buffer, "START_AFTER_ERROR", 17) == 0) {
		return (TRUE);
	} else if (strncasecmp ((void *)buffer, "STARTUP_LOG", 11) == 0) {
		return (TRUE);
	} else if (strncasecmp ((void *)buffer, "SUBNET_SIZE", 11) == 0) {
		equals = index ((void *)buffer, '=');
		if (equals == NULL) {
			// no equals sign (=)
			return (FALSE);
		}
		// we'll use VIEO_FM_BASE for creating the output file
		StringCopy ((void *)subnetSize, equals + 1, sizeof(subnetSize));
		/* add SUBNET_SIZE to each manager environment */
		memset (line_key, 0, 100);
		StringCopy ((void *)line_key, "SUBNET_SIZE", strlen("SUBNET_SIZE") + 1);
		memset (line_value, 0, 200);
		strncpy ((void *)line_value, (void *)subnetSize, 128);
		for (i = 0; i < MAX_INSTANCES; i++) {
			setBase("sm", i);
			setBase("bm", i);
			setBase("pm", i);
			setBase("fe", i);
		}
		return (TRUE);
	} else if (strncasecmp ((void *)buffer, "SYSLOG_MODE", 11) == 0) {
        equals = index ((void *)buffer, '=');
		if (equals == NULL) {
			// no equals sign (=)
			return (FALSE);
		}
		/* add SYSLOG_MODE to each manager's environment */
		StringCopy ((void *)syslogMode, equals + 1, sizeof(syslogMode));
		memset (line_key, 0, 100);
		StringCopy ((void *)line_key, "SYSLOG_MODE", strlen("SYSLOG_MODE") + 1);
		memset (line_value, 0, 200);
		strncpy ((void *)line_value, (void *)syslogMode, 16);
		for (i = 0; i < MAX_INSTANCES; i++) {
			setBase("sm", i);
			setBase("bm", i);
			setBase("pm", i);
			setBase("fe", i);
		}
		return (TRUE);
	}
	
	return (FALSE);
}


//------------------------------------------------------------------------------//
//										//
// Routine      : is_valid							//
//										//
// Description  : Make a best-effort to validate the format of the line. It	//
//                should contain a key=value. Where the key is three parts:	//
//                system_instance_dbkey. 'System' is either an Agent or a	//
//                Manager identifier. 'Instance' is a number from 0-255		//
//                inclusive. 'db_key' is a arbitrary string.			//
// Input        : None								//
// Output       : TRUE  - looks like a valid line				//
//                FALSE - something wierd is going on here			//
//										//
//------------------------------------------------------------------------------//

int
is_valid (void) {
	int	scan_count;		// number of sscanf parameters
	int	instance;		// instance number (0 - 256)
	char	*equals;		// location of equals sign (=)
	char	*underscore_1;		// location of underscore (_)
	char	*underscore_2;		// location of underscore (_)
	char	scan_system [256];	// Agent or Manager indicator
	char	scan_instance [256];	// instance number
	char	scan_key [256];		// key
	char	scan_value [256];	// value
	
//
//	See if this is a Manager.
//
	
	if (!is_manager ()) {
		// not an Manager line
		return (FALSE);
	}
	
//
//	Find the first underscore (_), it terminates the 'system'.
//
	
	underscore_1 = index ((void *)buffer, '_');
	if (underscore_1 == NULL) {
		// no first underscore, bad news
		return (FALSE);
	}
	*underscore_1 = ' ';
	underscore_1++;
	
//
//	Find the second underscore, it separates the 'system' from the 'instance'.
//
	
	underscore_2 = index (underscore_1, '_');
	if (underscore_2 == NULL) {
		// no second underscore, bad news
		return (FALSE);
	}
	*underscore_2 = ' ';
	underscore_2++;
	
//
//	Find the equals sign (=), it separates the 'variable' from the 'value'.
//
	
	equals = index (underscore_2, '=');
	if (equals == NULL) {
		// no equals sign between the variable and the value
		return (FALSE);
	}
	*equals = ' ';
	
//
//	Extract elements of the string.
//
	
	scan_count = sscanf ((void *)buffer, "%255s %255s %255s %255s", scan_system, scan_instance, scan_key, scan_value);
	if (scan_count != 4) {
		// need 4 sets of information (system instance key value)
		return (FALSE);
	}
	scan_system[255]=0;
	scan_instance[255]=0;
	scan_key[255]=0;
	scan_value[255]=0;
	
//
//	Convert the instance number to an integer.
//
	
	scan_count = sscanf (scan_instance, "%d", &instance);
	if (scan_count != 1) {
		// couldn't scan an instance number
		return (FALSE);
	}
	
	if (instance < 0 || instance > 255) {
		// instance number out of range
		return (FALSE);
	}
	
//
//	Save the system (Agent or Manager) information.
//
	
	memset (line_system, 0, 4);
	strncpy ((void *)line_system, scan_system, 3);
	
//
//	Save the instance number.
//
	
	line_instance = instance;
	
//
//	Save the key information.
//
	
	memset (line_key, 0, 100);
	strncpy ((void *)line_key, scan_key, 99);
	
//
//	Save the parameter (value) information.
//
	
	memset (line_value, 0, 200);
	strncpy ((void *)line_value, scan_value, 199);
	
	return (TRUE);
}


//------------------------------------------------------------------------------//
//										//
// Routine      : is_manager							//
//										//
// Description  : Determine if this is a Manager line.				//
//                Managers are: BM, FE, PM, SM	//
// Input        : None								//
// Output       : TRUE  - contains a Manager identifier				//
//                FALSE - not a Manager identifier				//
//										//
//------------------------------------------------------------------------------//

int
is_manager (void) {
	
	if (strncasecmp ((void *)buffer, "BM", 2) == 0) {
		return (TRUE);
	} else if (strncasecmp ((void *)buffer, "FE", 2) == 0) {
		return (TRUE);
	} else if (strncasecmp ((void *)buffer, "PM", 2) == 0) {
		return (TRUE);
	} else if (strncasecmp ((void *)buffer, "SM", 2) == 0) {
		return (TRUE);
	} else {
		return (FALSE);
	}
}

//------------------------------------------------------------------------------//
//										//
// Routine      : add_information						//
//										//
// Description  : Add a line of information to our data structures.		//
// Input        : None								//
// Output       : None								//
//										//
//------------------------------------------------------------------------------//

Status_t
add_information (void) {
	uint16_t	i;
	Ag_Man_t	*ap;

	
//
//	Keep track of the Agents and Managers that are specifically started (...start=yes).
//
	if ((strcasecmp ((void *)line_key, "start") == 0) && (strcasecmp ((void *)line_value, "yes") == 0)) {
		// find a start record
		for (i = 0, ap = ag_man_start; i < ag_man_start_count; i++, ap++) {
			if (matching_system (ap, (void *)line_system)) {
				if (ap->instance == line_instance) {
					// already have this one recorded
					return (VSTATUS_OK);
				}
			}
		}
		
		// need to add a new starter record
		ap->instance   = line_instance;
		strncpy ((void *)ap->system, (void *)line_system, 3);
		ag_man_start_count++;
		
		if (ag_man_start_count > MAX_START) {
			log_error ("Warning: too many 'start=yes' entries, line %d\nLine : %s\n", line_number, read_buffer);
			return (VSTATUS_BAD);
		}
		return (VSTATUS_OK);
	}


//
//	Add to the detailed information.
//
	
	ag_manp->instance   = line_instance;
	
	strncpy ((void *)ag_manp->system, (void *)line_system, 3);
	strncpy ((void *)ag_manp->key,    (void *)line_key,   99);
	strncpy ((void *)ag_manp->value,  (void *)line_value, 99);

	ag_manp++;
	ag_man_detail_count++;
	
	if (ag_man_detail_count > MAX_DETAIL) {
		log_error ("Warning: too many detailed entries, line %d\nLine : %s\n", line_number, read_buffer);
		return (VSTATUS_BAD);
	}
	
//
//	See if we need to add this to the unique information.
//	Unique information is a unique system (BM, PM, SM) and instance identifier.
//
	
	for (i = 0, ap = ag_man_unique; i < ag_man_unique_count; i++, ap++) {
		if (matching_system (ap, (void *)line_system)) {
			if (ap->instance == line_instance) {
				// already have this one recorded
				return (VSTATUS_OK);
			}
		}
	}
	
//
//	Add to the unique information.
//
	
	ap->instance   = line_instance;
	strncpy ((void *)ap->system, (void *)line_system, 3);
	ag_man_unique_count++;
	
	if (ag_man_unique_count > MAX_UNIQUE) {
		log_error ("Warning: too many unique entries, line %d\nLine : %s\n", line_number, read_buffer);
		return (VSTATUS_BAD);
	}
	
	return (VSTATUS_OK);
}


int
matching_system (Ag_Man_t *ag_start, char *which_one) {
	
	if (strcasecmp ((void *)ag_start->system, which_one) == 0) {
		return (TRUE);
	} else {
		return (FALSE);
	}
}

//------------------------------------------------------------------------------//
//										//
// Routine      : log_debug							//
//										//
// Description  : Print debugging information. Instead of embedding printf	//
//                statments everywhere, we'll do all printing from one routine	//
//                so that in the future we can re-direct output to some other	//
//                device if we're not Linux-based.				//
// Input        : A format and argument list, like printf			//
// Output       : None								//
//										//
//------------------------------------------------------------------------------//

void
log_debug (char *format, ...) {
	va_list		args;
	
	if (debug == TRUE) {
		va_start (args, format);
		(void) vprintf (format, args);
		va_end (args);
	}
	
	return;
}


//------------------------------------------------------------------------------//
//										//
// Routine      : log_error							//
//										//
// Description  : Print error information. Instead of embedding printf		//
//                statments everywhere, we'll do all printing from one routine	//
//                so that in the future we can re-direct output to some other	//
//                device if we're not Linux-based.				//
// Input        : A format and argument list, like printf			//
// Output       : None								//
//										//
//------------------------------------------------------------------------------//

void
log_error (char *format, ...) {
	va_list		args;
	
	va_start (args, format);
	(void) vprintf (format, args);
	va_end (args);
	
	return;
}

Status_t readAndParseConfigFile(bool_t debug)
{
//
//	Open the config file.
//
	
	fd_config = fopen (config_file, "r");
	if (fd_config == NULL) {
		log_error ("Could not open the config file ('%s') for reading\n", config_file);
		return (VSTATUS_BAD);
	}

//
//	Allocate memory for reading and parsing the config file.
//
	if (alloc_mem () != VSTATUS_OK) {
		return (VSTATUS_BAD);
	}

//
//	Read the config file, parse the lines, and create data structures.
//
	int line=0;
	while (read_line () == VSTATUS_OK) {
		if (debug) fprintf(stdout,"test line %d->%s\n",line++,buffer);
		if (parse () != VSTATUS_OK) {
			break;
		}
	}
//
//	Close the config file.
//
	(void) fclose (fd_config);
	return VSTATUS_OK;
}
