/* BEGIN_ICS_COPYRIGHT7 ****************************************

Copyright (c) 2017, Intel Corporation

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The functions here can report version and brand strings which are
// post processed into the executable by invoking patch_version and patch_brand
// during the build
//
// Logs, menus and other interfaces can then use the brand and version strings
// such as follows:
//		printf("%s XYZ version %s\n", GetCodeBrand(), GetCodeVersion());

// GetCodeVersion
// -----------------
// GetCodeVersion - retrieve the code version as set by build process.
// The static string contained in this function will be changed
// by patch_version after the build is complete and the string
// has been compiled into the application.  This string will be
// modified as necessary to set the version number to match that of
// the build cycle.  This routine extracts the version number from
// the string and returns it.
// It accepts no arguments and returns "Unknown" if the string cannot
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
	return(version?version:"Unknown");
}

// GetCodeInternalVersion
// -----------------
// GetCodeInternalVersion - retrieve the code version as set by build process.
// The static string contained in this function will be changed
// by patch_version after the build is complete and the string
// has been compiled into the application.  This string will be
// modified as necessary to set the version to match that of
// the build cycle.  This routine extracts the version from
// the string and returns it.
// It accepts no arguments and returns "Unknown" if the string cannot
// be evaluated.
// The version number created can also be extracted by the strings command
// or the internal whatversion tool
//
#define ICS_BUILD_INTERNAL_VERSION "THIS_IS_THE_ICS_INTERNAL_VERSION_NUMBER:@(#)000.000.000.000B000I0000"
const char* GetCodeInternalVersion(void)
{
	static const char* BuildVersion=ICS_BUILD_INTERNAL_VERSION;
	static char* version;
	static int built=0;
	if (!built)
	{
		// locate start of version string,
		// its the first digit in BuildVersion
		version = strpbrk(BuildVersion, ")") + 1;
		built=1;
	}
	return(version);
}

// GetCodeBrand
// -----------------
// GetCodeBrand - retrieve the code brand as set by build process.
// The static string contained in this function will be changed
// by patch_brand after the build is complete and the string
// has been compiled into the application.  This string will be
// modified as necessary to set the brand name to match that of
// the build cycle.  This routine extracts the brand name from
// the string and returns it.
// It accepts no arguments.
// The brand name created can also be extracted by the strings command
// or the internal whatversion tool
//
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
