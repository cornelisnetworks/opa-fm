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
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <libgen.h>
#ifdef WIN32
#include <io.h>
#include <process.h>
#endif
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <utime.h>
#include <fcntl.h>

typedef int boolean;
#define TRUE 1
#define FALSE 0

int  Mkdir(const char* dir);
int  InstallTarget(const char* targetDir, const char* srcFileName, const boolean forceFlag, unsigned modeMask);
boolean CopyFile( const char* targetFileName, const char* srcFileName, struct stat srcStat, unsigned modeMask );
boolean CompareFiles( const char* targetFileName, const char* srcFileName );

void Usage(const char* progname)
{
	printf("usage: %s -d <target dir> [ -f ] [file name 1] ... [file name n]\n", progname);
    printf("\n");
    exit(1);
}

int main(int argc, char *argv[])
   {
    const char* targetDir = NULL;
    boolean        forceFlag = FALSE;
	unsigned	modeMask = ~0u;
    int argNum = 0;

    while( ++argNum < argc )
    {
        switch( argv[argNum][0] )
        {
        case '-':
            switch( argv[argNum][1] )
            {
            case 'd':
                /* target directory specified */
                if( ++argNum == argc)
                {
                    Usage(argv[0]);
                    return 1;
                }
                targetDir = argv[argNum];
                if( Mkdir(targetDir) == -1 )
                {
                    printf("...Can't create target directory %s \n", targetDir);
                    return 1;
                }
                break;
            
            case 'f':
                /* force flag specified; error in accessing any source file is ignored */
                forceFlag = TRUE;
                break;
            case 'r':
                /* readonly flag specified; make destination file readonly */
				modeMask = ~(S_IWUSR|S_IWGRP|S_IWOTH);
                break;
            
            default:
                Usage(argv[0]);
                return 1;
            }
            break;

        default:
            if( targetDir == NULL )
            {
                Usage(argv[0]);
                return 1;
            }
            if( InstallTarget(targetDir, argv[argNum], forceFlag, modeMask) != 0 )
            {
                printf("...Unable to install file %s \n", argv[argNum]);
                return 1;
            }
            break;
        }
    }
    
    return 0;
}  /*  End main function  */


int Mkdir(const char* dir)
{
    int result = 0;
    int lenPlusOne = 0;
    char* newDir;
    const char* pDir = dir;
    char* pNewDir;

    /* No relative path permitted.  Will return an error result if it occurs. */
    if( dir == NULL || (lenPlusOne = strlen(dir)+1) == 1 || *dir == '.' )
    {
        return -1;
    }

    /* If the directory already exists, no need to do anything else */
    if( access(dir, 0) == 0 )
    {
        return 0;
    }

    newDir = (char*)malloc(lenPlusOne);
	if (!newDir) {
		return -1;
	}

    for( pNewDir = newDir; lenPlusOne--; ++pDir, ++pNewDir )
    {
        /* If a dir component delimiter is found AND it is not the 1st char of dir spec AND */
        /* the previous dir spec char was not a colon (ie drive letter delimiter), then create */
        /* the new directory if not already existent.  Note this parsing will not handle relative */
        /* paths. */
        if( (*pDir == '/' || *pDir == '\\' || *pDir == '\0') &&
            pDir != dir &&
            *(pDir-1) != ':' )
        {
            *pNewDir = '\0';
            if( access(newDir, 0) == -1 && mkdir(newDir, 0755) == -1 )
            {
                result = -1;
                break;
            }
        }
        *pNewDir = *pDir;
    }

    free(newDir);
    return result;
}


int InstallTarget(const char* targetDir, const char* srcFileName, const boolean forceFlag, unsigned modeMask)
{
    struct stat srcStat;
    struct stat targetStat;
	char	     targetFileName[2000];
    boolean copyResult = TRUE;
    int     res = 0;

    /* If the source file does not exist as a regular file, return an error unless the */
    /* force flag is set. */
    if( stat(srcFileName, &srcStat) != 0 || ! S_ISREG(srcStat.st_mode) )
    {
        if( forceFlag )
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }

    /* Build the target file name spec */
    res = snprintf(targetFileName, 2000, "%s/%s", targetDir, basename((char*)srcFileName));
    if(res <= 0 || res > 2000)
        return 1;   /* Path could not be created or is too long (very unlikely) */

    /* If the target file does not exist OR the source file modified time is greater than */
    /* the target file modified time, OR the source file size is not equal to the target */
    /* file size, no need to compare, just do the copy */
#if defined(linux) || defined(__DARWIN__)
	if( stat(targetFileName, &targetStat) != 0 ||
        ! S_ISREG(targetStat.st_mode) ||
        srcStat.st_mtime > targetStat.st_mtime ||
        srcStat.st_size  != targetStat.st_size )
#else
	if( _stat(targetFileName, &targetStat) != 0 ||
        ! S_ISREG(targetStat.st_mode) ||
        srcStat.st_mtime > targetStat.st_mtime ||
        srcStat.st_size  != targetStat.st_size )
#endif
    {
        copyResult = CopyFile( targetFileName, srcFileName, srcStat, modeMask);
    }
    else
    {
        /* If the compare says the source and target files are different, do the copy */
        if( CompareFiles(targetFileName, srcFileName) )
        {
            copyResult = CopyFile( targetFileName, srcFileName, srcStat, modeMask );
        }
    }

#if 0
    if( forceflag )
	{
		copyResult = TRUE;
	}
#endif
	
	return copyResult ? 0 : 1;
}


boolean CopyFile( const char* targetFileName, const char* srcFileName, struct stat srcStat, const unsigned modeMask )
{
    FILE *srcFile = NULL;
    FILE *targetFile = NULL;
    boolean copyResult = TRUE;
    int  srcChr;
	unsigned mode = srcStat.st_mode & modeMask;
        
    printf("Copying %s to %s...", srcFileName, targetFileName);
    srcFile = fopen(srcFileName, "rb");
	chmod(targetFileName, mode|(S_IWUSR|S_IWGRP));
    targetFile = fopen(targetFileName, "wb");
    if( srcFile != NULL && targetFile != NULL )
    {
        while ( (srcChr = fgetc(srcFile)) != EOF )
        {
            if( putc( srcChr, targetFile) == EOF )
            {
                copyResult = FALSE;
                break;
            }
        }
    }
    else
    {
        copyResult = FALSE;
    }

    if( srcFile )
	{
		fclose(srcFile);
	}

    if( targetFile )
	{
		fclose(targetFile);
		chmod(targetFileName, mode);
	}

	if ( copyResult )
	{
		static struct utimbuf utb;
		int rval;

#if defined(__DARWIN__)
		utb.actime = srcStat.st_atimespec.tv_sec;
		utb.modtime = srcStat.st_mtimespec.tv_sec;
#else
		utb.actime = srcStat.st_atime;
		utb.modtime = srcStat.st_mtime;
#endif
		if ((rval=utime(targetFileName,&utb))) {
			printf("Could not set utime: %d/%s..",rval,strerror(rval));
		} 
	}

    if( copyResult )
    {
        printf("Copy Complete!!!\n");
    }
    else
    {
        printf("Copy Failed!!!\n");
    }

    return copyResult;
}

boolean CompareFiles( const char* targetFileName, const char* srcFileName )
{
    FILE *srcFile = NULL;
    FILE *targetFile = NULL;
    int  srcChr, targetChr;
    unsigned long limitByteCount = 1000;
    boolean filesAreDifferent;

    /* Byte check the 1st 1000 bytes */
    srcFile = fopen(srcFileName, "rb");
    targetFile = fopen(targetFileName, "rb");
	if ( !srcFile && targetFile) {
		printf("Could not open %s\n", srcFileName);
		fclose(targetFile);
		return TRUE;
	} else if ( srcFile && !targetFile) {
		printf("Could not open %s\n", targetFileName);
		fclose(srcFile);
		return TRUE;
	} else if ( !srcFile && !targetFile) {
		printf("Could not open either %s, %s\n", srcFileName, targetFileName);
		return FALSE;
	}
    while ( (srcChr = fgetc(srcFile)) != EOF && limitByteCount )
    {
        /* If the target is smaller than the source file or the byte comparison fails, */
        /*   then the files are different. */
        if( (targetChr = fgetc(targetFile)) == EOF || srcChr != targetChr )
        {
            fclose(srcFile);
            fclose(targetFile);
            return TRUE;
        }
        /* If the limit number of bytes to be compared are all equal, assume the files */
        /* are the same. */
        else if( --limitByteCount == 0 )
        {
            fclose(srcFile);
            fclose(targetFile);
            return FALSE;
        }
    }

    filesAreDifferent = fgetc(targetFile) != EOF;
    fclose(srcFile);
    fclose(targetFile);

    return filesAreDifferent;
}
