/*!
 @file    $Source: /cvs/ics/MakeTools/patch_version/patch_version.c,v $
 $Name:  $
 $Revision: 1.14 $
 $Date: 2015/01/22 18:03:59 $
 @brief   Utility to manage version and branding strings in executable code
 */
/* BEGIN_ICS_COPYRIGHT7 ****************************************

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

 * ** END_ICS_COPYRIGHT7   ****************************************/
/* [ICS VERSION STRING: unknown] */

/*!
This program can be invoked in 2 ways:

When invoked as patch_version, this program has 2 main uses:
In its simplest form, it can take a tag number of the form: R#_#M#P#I#
and convert it to #.#.#.# format (it does allow for alpha and beta
release of the form: R#_#A#I# and R#_#B#I#, they get converted to
#.#A# or #.#B#).

In addition, if given an executable, compiled program with an embedded
version string, it can alter the embedded version string.  This can be
useful to change version strings after the build, especially when the
decisions for the final version # are changing, and/or the product takes
a long amount of time to compile.  

When invoked as patch_brand, this program has a simple purpose:

In its simplest form it can take a brand (or an empty string) as its argument
and it will report on stdout the brand which will be used (and will verify the
brand's length).  This can be useful to obtain the default brand.

In addition, if given an executable, compiled program with an embedded
brand string, it can alter the embedded brand string.  This can be
useful to change brand strings after the build, especially when the
decisions for the final brand are changing (or to brand for multiple customers),
and/or the product takes a long amount of time to compile.  

@heading Usage
	patch_version release_tag [file ...]
	patch_brand brand [file ...]

@param release_tag - version tag of the form R#_#M#P#I# or R#_#A#I# or R#_#B#I#
@param brand - a branding string such as "Intel"
@param file - object/exe file(s) to place the #.#.#.# style version string in
		If no files are specified, the program merely
		performs the simpler behaviour of outputing the #.#.#.#
		style version string.

@example
	The version string and brand in programs to be patched
	should be coded as follows:

	// GetCodeVersion
	// -----------------
	// GetCodeVersion - retrieve the code version as set by build process.
	// The static string contained in this function will be changed
	// by patch_version after the build is complete and the string
	// has been compiled into the application.  This string will be
	// modified as necessary to set the version number to match that of
	// the build cycle.  This routine extracts the version number from
	// the string and returns it.
	// It accepts no arguments and returns NULL if the string cannot
	// be evaluated.
	// The version number created can also be extracted by the strings command
	// or the internal whatversion tool
	//
	#define ICS_BUILD_VERSION "THIS_IS_THE_ICS_VERSION_NUMBER:@(#)000.000.000.000B000"
	const char* GetCodeVersion(void)
	{
		static const char* BuildVersion=ICS_BUILD_VERSION;
		static char* version;
		static int built=0;
		if (!built)
		{
			// locate start of version string,
			// its the first digit in BuildVersion
			version = strpbrk(BuildVersion, "0123456789");
			built=1;
		}
		return(version);
	}

	#define ICS_BUILD_BRAND "THIS_IS_THE_ICS_BRAND:Intel\000                    "
	const char* GetCodeBrand(void)
	{
		static const char* BuildBrand=ICS_BUILD_BRAND;
		static char* brand;
		static int built=0;
		if (!built)
		{
			// locate start of brand string,
			// its the first : in BuildBrand
			brand = strpbrk(BuildBrand, ":")+1;
			built=1;
		}
		return(brand);
	}

	Then use the brand and version string as follows:
		printf("%s XYZ version %s\n", GetCodeBrand(), GetCodeVersion());

	The version strings can actually be more flexibly specified than
	outlined above.

@see UiUtil::GetCodeVersion();
@see UiUtil::GetCodeBrand();
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>

#define ICS_BUILD_VERSION "THIS_IS_THE_ICS_VERSION_NUMBER:@(#)000.000.000.000B000"
#define ICS_BUILD_INTERNAL_VERSION "THIS_IS_THE_ICS_INTERNAL_VERSION_NUMBER:@(#)000.000.000.000B000I0000"
#define ICS_BUILD_BRAND "THIS_IS_THE_ICS_BRAND:Intel\000           "
#define DEFAULT_BRAND "Intel"

typedef enum { B_FALSE=0, B_TRUE } boolean;

#define BUFFER_SIZE 8192

typedef enum {
	NO_ERROR=0,		/* Program terminated normally */
	INTERNAL_ERROR,	/* Error occurred in processing */
	USAGE_ERROR		/* Improper commandline arguments */
} ApplicationExitcode;

/*!
display a usage message to the user if the incorrect
set of arguments is passed to the application.
*/
void Usage(const char *program_name)
{
	fprintf(stderr,"\nUsage:     %s [-i][-m marker][-n version] release_tag [file ...]\n",program_name);
	exit(USAGE_ERROR);
}
void UsageBrand(const char *program_name)
{
	fprintf(stderr,"\nUsage:     %s [-m marker] brand [file ...]\n",program_name);
	exit(USAGE_ERROR);
}

/*!
parse the command line argument for the release tag into the external version
numbers.

The following transformations on the release tag are done:
	- All non-digits up to the 1st digit are dropped
	- _ is changed to .
	- M is changed to .
	- P is changed to .
	- I and all characters after it are dropped if the -i option is used

Note to properly conform to the release numbering scheme
Tags must include an M level if they include a P level.
To work properly with this and other programs, the 1st character
in the resulting version # must be a digit (0-9).  Hence when
using code names as the temporary version string during
development they should be started with a digit.  For example:
	R0capemay
will result in a version number of 0capemay

This would be the typical way to build releases for a product
with a code name of "capemay".

For Example:
	release_tag	Version String
	R1_0I5		1.0
	R1_0M1I2	1.0.1
	R1_0B1I2	1.0B1
	R1_0M1A1I3	1.0.1A1
	R1_0M0P4I5	1.0.0.4
	R1_0B1P4	1.0B1.4
	R0capemayI7	0capemay
	R0capemayP4I1	0capemay.4
	ICS_R2_0I5	2.0

@param string the string to be parsed.
@param dropI should I level be omitted

@return pointer to a static string containing the constructed version string.

@heading Special Cases and Error Handling
	prints an error message and returns NULL on failure.
*/
const char* ParseReleaseTag(const char* string, int dropI)
{
	static char version[100];
	const char* p;
	char* q;
	int done;
	int processedM = 0;
	int processedP = 0;

	if (! string || ! *string)
	{
		fprintf(stderr,"\nInvalid release tag format, empty string\n");
		return(NULL);
	}
	for (p=string; *p && ! isdigit(*p); p++)
		;
	for (done=0, q=version; ! done && *p; p++)
	{
		switch (*p)
		{
			case 'I':
				/* I starts the trailer for the release tag, it is not placed
				 * in the version string
			 	 */
				if (dropI)
					done=1;
				else
				{
					/* replaced with a . */

					/* if no M tag was processed, must insert a zero placeholder */
					if (!processedM)
					{
						*q++ = '.';
						*q++ = '0';
					}
					/* if no P tag was processed, must insert a zero placeholder */
					if (!processedP)
					{
						*q++ = '.';
						*q++ = '0';
					}
					*q++ = '.';
				}
				break;
			case '_':
				/* replaced with a . */
				*q++ = '.';
				break;
			case 'M':
				/* replaced with a . */
				*q++ = '.';
				processedM = 1;
				break;
			case 'P':
				/* replaced with a . */
				/* if no M tag was processed, must insert a zero placeholder */
				if (!processedM)
				{
					*q++ = '.';
					*q++ = '0';
					processedM = 1;
				}
				*q++ = '.';
				processedP = 1;
				break;
			default:
				*q++ = *p;
				break;
		}
	}
	/* version strings must start with a digit */
	if (*version < '0' || *version > '9')
	{
		fprintf(stderr,"\nInvalid release tag format yields: %s\n", version);
		return(NULL);
	}
	return(version);
}

/*!
append a marker character to the string

@param string - string to add marker to
@param marker - marker character to add

@return new string

@heading Special Cases and Error Handling
*/
char* append_marker(const char* string, char marker)
{
	char *result;

	result = malloc(strlen(string)+2);
	if (result) {
		strcpy(result, string);
		if (marker)
			strncat(result, &marker, 1);
	}
	return result;
}

/*!
build the pattern string to search for and verifies the
version string will fit within the patch area

@param version - version string intended to be placed into file

@return pattern to search for

@heading Special Cases and Error Handling
	if version is larger than space alloted in Pattern, outputs error and
	returns NULL
*/
const char* BuildPattern(char* dest, const char* pattern, const char* version)
{
	int length;

	strcpy(dest, pattern);
	/* determine length of pattern itself */
	length = strcspn(dest, "0123456789");
	/* confirm the version can be used to overwrite the rest of the
	 * default version string
	 */
	if (strlen(dest) - length < strlen(version))
	{
		fprintf(stderr,"\nInvalid release tag format, too long, yields: %s\n", version);
		return(NULL); /* won't fit */
	}

	dest[length] = '\0';
	return(dest);
}

/*!
build the pattern string to search for and verifies the
brand string will fit within the patch area

@param brand - brand string intended to be placed into file

@return pattern to search for

@heading Special Cases and Error Handling
	if brand is larger than space alloted in Pattern, outputs error and
	returns NULL
*/
const char* BuildBrandPattern(char* dest, const char* pattern, size_t brand_size, const char* brand)
{
	int length;

	strcpy(dest, pattern);
	/* determine length of pattern itself */
	length = strcspn(dest, ":")+1;
	/* confirm the brand can be used to overwrite the rest of the
	 * default brand string
	 * brand_size includes \0 terminator
	 */
	if ((brand_size-1) - length < strlen(brand))
	{
		fprintf(stderr,"\nInvalid brand, too long: '%s'\n", brand);
		return(NULL); /* won't fit */
	}

	dest[length] = '\0';
	return(dest);
}

/*!
Patch the File

This function does an lseek, hence the position in fp is not important
when called.  However it will leave fp after the bytes written.

@param fp - file to patch.
@param offset - offset in file to place version
@param version - version string to write
@param verlen - strlen(version)+1

@return B_TRUE on success, or prints an error message and returns B_FALSE
	on error.
*/
boolean PerformPatch(FILE* fp, long offset, const char* version, size_t verlen)
{
	if (fp)
	{
		long current=ftell(fp);
		if (!fseek(fp,offset,SEEK_SET))
		{
			if (fwrite(version,sizeof(char),verlen,fp)==verlen)
			{
				if (!fseek(fp,current,SEEK_SET))
				{
					return(B_TRUE);
				}
				fprintf(stderr,"\nRead seek on file (stream) failed\n");
			} else {
				fprintf(stderr,"\nWrite of patched data failed\n");
			}
		} else {
			fprintf(stderr,"\nWrite seek on file (stream) failed\n");
		}
	}
	return(B_FALSE);
}

/*!
Patch the version string into the given filename at the position where pattern
is found.

The function opens the filename specified, searches the file for the
matching ID string, patches the version number and closes the file.

@param filename - file to patch
@param pattern - search pattern
@param patlen - strlen(pattern)
@param version - version string
@param verlen - strlen(version)+1

@return B_TRUE on success, otherwise it prints an error message
	 and returns B_FALSE.
*/
boolean PatchFilename(const char* filename, const char* pattern, int patlen,
					  const char* version, size_t verlen)
{
	FILE* fp = NULL;
	char buffer[BUFFER_SIZE];
	char* bufp;
	char* bufp2;
	const char* patp;
	int i;
	boolean patched=B_FALSE;
	boolean finished=B_FALSE;
	if (filename && *filename)
	{
		if ((fp=fopen(filename,"r+"))==NULL)
		{
			perror(filename);
			goto patch_error;
		}
		while (!finished)
		{
			int nbytes=fread(buffer,sizeof(char),BUFFER_SIZE,fp);
			bufp=buffer;
			if (!nbytes) /* End of file */
			{
				finished=B_TRUE;
				continue;
			}
			if (nbytes<0) /* Error */
			{
				fprintf(stderr,"\nPremature EOF: %s\n",filename);
				goto patch_error;
			}
			while ((bufp-buffer)+patlen<=nbytes)
			{
			  i = 0;
			  patp = pattern;
			  bufp2 = bufp;
			  while ((*bufp++ == *patp++) && (i < patlen)) i++; 

			  if (i == patlen)
			  {
				long match_offset=(ftell(fp)-nbytes)+(bufp2-buffer)+patlen;
				if (!PerformPatch(fp,match_offset,version, verlen)) 
				{
				  goto patch_error;
				}
				patched=B_TRUE;
			  }

			  bufp = bufp2 + 1;
			}
			if (nbytes <= patlen)
			{
				/* must have just processed the last part of the file */
				finished=B_TRUE;
				continue;
			}
			/* go to next buffer, but include patlen bytes from previous
			 * in case version string crosses a buffer boundary
			 */
			if (fseek(fp, -patlen, SEEK_CUR) == -1)
			{
				fprintf(stderr,"\nCan't seek: %s\n",filename);
				goto patch_error;
			}
		}
		fclose(fp);
		fp = NULL;
		if (!patched)
		{
			fprintf(stderr,"\nVersion string search pattern not found: %.*s\n", patlen, pattern);
			goto patch_error;
		}
		return(B_TRUE);
	}
patch_error:
	if (fp != NULL)
	{
		fclose(fp);
	}
	fprintf(stderr,"Patch failed for filename: %s\n\n",(filename)?filename:"NULL");
	return(B_FALSE);
}

char *basename(char *filename)
{
	char *p;

	p = strrchr(filename, '/');
	if (p == NULL)
		return filename;
	else
		return p+1;
}

/*!
search through binary a.out files looking for the ICS version string
and updating the internal field so that the executables will have the correct
version string information usable internally from a post build scenario.
This application is not used to place a version reference in all a.out files,
but specifically in a few with a certain internal static buffer.
The program accepts two or more arguments on the command line.

The first argument must specify the source repository tag.

The other arguments specify filenames.

The tag is split into four numeric fields (major, minor, patch,
and integration build).  All four fields
must be valid and the tag argument is required.  At least one filename must
also be specified and all filename arguments are opened in sequence and the
patch performed.  The program returns a 0 on success and a positive number
on error.  (1=Error in processing, 2=Usage).
*/
int main(int argc,char* argv[])
{
	ApplicationExitcode result=NO_ERROR;
	const char* version = NULL;
	const char* internalVersion;
	int position;
	int numArgs;
	int rc;
	int dropI = 0;
	char pattern[1024];
	char marker = '\0';
	/*char pattern[MAX(sizeof(ICS_BUILD_VERSION)+1,sizeof(ICS_BUILD_BRAND)+1]; */
	char internalPattern[sizeof(ICS_BUILD_INTERNAL_VERSION)+1];
	int brand = 0;
  
	brand = (strcmp(basename(argv[0]), "patch_brand") == 0);

	while ((rc = getopt(argc, argv, "im:n:")) != -1)
	{
		switch (rc)
		{
			case 'i':
				dropI = 1;
				break;
			case 'm':
				marker = optarg[0];
				break;
			case 'n':
				/* Override ParseReleaseTag() functionality by supplying an already-formatted version string. */
				version = optarg;
				break;
			default:
				break;
		}
	}
	numArgs = argc - optind;
	if (brand)
	{
		if (numArgs<1)
		{
			UsageBrand(argv[0]);
		}
		version = argv[optind];
		if (strlen(version) == 0)
		{
			version = DEFAULT_BRAND;
		}
		version = append_marker(version, marker);
		if (!version)
		{
			exit(INTERNAL_ERROR);
		}
		internalVersion = NULL;
		if ((BuildBrandPattern(pattern, ICS_BUILD_BRAND, sizeof(ICS_BUILD_BRAND), version)) == NULL)
		{
			exit(USAGE_ERROR);
		}
	} else {
		if (numArgs<1)
		{
			Usage(argv[0]);
		}

		if (version == NULL && (version = ParseReleaseTag(argv[optind], dropI)) == NULL)
		{
			exit(USAGE_ERROR);
		}
		version = append_marker(version, marker);
		if (!version)
		{
			exit(INTERNAL_ERROR);
		}
		internalVersion = append_marker(argv[optind], marker);
		if (!internalVersion)
		{
			exit(INTERNAL_ERROR);
		}
		if ((BuildPattern(pattern, ICS_BUILD_VERSION, version)) == NULL)
		{
			exit(USAGE_ERROR);
		}
		if ((BuildPattern(internalPattern, ICS_BUILD_INTERNAL_VERSION, internalVersion)) == NULL)
		{
			exit(USAGE_ERROR);
		}
	}
	if (numArgs == 1)
	{
		/* no program names, just put version string to stdout */
		printf("%s\n", version);
		exit(NO_ERROR);
	}
	for (position = optind + 1; position < argc; position++)
	{
		/* include the '\0' in the version in the patch so that the version string
		 * is properly terminated when version is shorter than the space allowed
		 */
		if (brand)
			printf("Patching %s with brand '%s'\n", argv[position], version);
		else
			printf("Patching %s with version strings %s and %s\n", argv[position], version, 
				internalVersion?internalVersion:"");
		if (!PatchFilename(argv[position], pattern, strlen(pattern), version, strlen(version)+1))
		{
			result=INTERNAL_ERROR;
		}
		if (internalVersion)
		{
			if (!PatchFilename(argv[position], internalPattern, strlen(internalPattern), internalVersion, strlen(internalVersion)+1))
			{
				result=INTERNAL_ERROR;
			}
		}
	}
	exit(result);
}
