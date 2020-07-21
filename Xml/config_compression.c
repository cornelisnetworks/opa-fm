/* BEGIN_ICS_COPYRIGHT5 ****************************************

Copyright (c) 2015-2020, Intel Corporation

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

/* THIS IS USED ONLY IN STL1 AND LATER */

#include "vxWorks.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "taskLib.h"
#include "time.h"
#include "tms/common/rdHelper.h"
#include "tms/idb/icsUiConfigMib.h"
#include "tms/common/usrSecLib.h"
#include "bspcommon/h/sysPrintf.h"
#include "bspcommon/h/icsBspUtil.h"
#include "bspcommon/h/sysFlash.h"

#include "config_compression.h"

extern int UiUtil_GetLocalTime ();

static int debug_comp;

#define COPYBUFFER_SIZE			2048
#define UNCOMPRESSED_ARRAY_SIZE 2048
#define	COMPRESSED_ARRAY_SIZE	2048

#define SCP_LOG( fmt, func, args... ) \
	do{ \
		if( debug_comp ) { \
			sysPrintf("task=%s %s : " fmt "\n", taskName(taskIdSelf()), func, ## args ); \
		} \
	} while(0)

// unfortunately we can't #include zlib.h or we have multiple definitions for crc32
#undef compress
extern int compress(char *dest, int *destLen, char *src, int sourceLen);

typedef struct compBlockHeader {
	int compressedBytes;
	int uncompressedBytes;
} compBlockHeader_t;

#define COMP_VERSION 0

typedef struct compFileHeader {
	int version;
	time_t creationTime;
} compFileHeader_t;

typedef struct {
	FILE *fdin;
	int leftOverBytes;
	int endOfCompressedFile;
	char uncompressedBytesArray[UNCOMPRESSED_ARRAY_SIZE];
	char compressedBytesArray[COMPRESSED_ARRAY_SIZE];
} compFile_t;

// TEST CODE START
//SCP_LOG("ATTEMPTING TEST TO TEST.XML", __FUNCTION__);
//	FILE *fileIn = openUncompressedFile(outfilename);
//	FILE *fileOut = fopen("/firmware/test.xml", "wb");
//	int totalBytes = 0;
//	while (!feof(fileIn)) {
//		char testBuffer[2049];
//		int bytesRead = readUncompressedBytes(fileIn, testBuffer, sizeof(testBuffer));
//		totalBytes += bytesRead;
//		SCP_LOG("READ %d BYTES UNCOMPRESSED", bytesRead, __FUNCTION__);
//		fwrite((void*)testBuffer, 1, bytesRead, fileOut);
//		if (bytesRead < sizeof(testBuffer))
//			 break;
//	};
// 	SCP_LOG("TOTAL BYTES %d", totalBytes, __FUNCTION__);
//	fclose(fileOut);
//	closeUncompressedFile(fileIn);
// TES TCODE END


int copyFile(char *src, char*dst, int compressFlag, long *compressedFileSize) {
	// compress = 1 means inflate
	// compress = 0 means just copy
	// compress = -1 means deflate
	time_t timeStamp;
	char *copyBuffer1 = NULL;
	char *copyBuffer2 = NULL;
	char *uncompressedBytesArray = NULL;
	char *compressedBytesArray = NULL;

	copyBuffer1 = malloc(COPYBUFFER_SIZE);
	copyBuffer2 = malloc(COPYBUFFER_SIZE);
	uncompressedBytesArray = malloc(UNCOMPRESSED_ARRAY_SIZE);
	compressedBytesArray = malloc(COMPRESSED_ARRAY_SIZE);

	if ((copyBuffer1 == NULL) || (copyBuffer2 == NULL) ||
			(uncompressedBytesArray == NULL) || (compressedBytesArray == NULL)) {
		if (copyBuffer1 != NULL)
			free(copyBuffer1);
		if (copyBuffer2 != NULL)
			free(copyBuffer2);
		if (uncompressedBytesArray != NULL)
			free(uncompressedBytesArray);
		if (compressedBytesArray != NULL)
			free(compressedBytesArray);
		SCP_LOG("out of memory in copyFile", __FUNCTION__);
	}
	SCP_LOG("CopyFile src:%s, dst:%s, compressFlag:%d", __FUNCTION__, src, dst, compressFlag);
	size_t bufsize = COPYBUFFER_SIZE;

	int retVal = 0;
	int didFileHeader = 0;

	FILE *fdout;

	if (compressedFileSize != NULL)
		*compressedFileSize = 0;

	fdout = fopen(dst, "w");
	// if we are out of space, try removing the original first and reopening the file
	if (fdout == NULL) {
		FileRemove(dst);
		fdout = fopen(dst, "w");
	}
	if (fdout != NULL) {
		SCP_LOG("OPENED! output file %s", __FUNCTION__, dst);
		FILE *fdin;
		if (NULL != (fdin = fopen(src, "r"))) {
			SCP_LOG("OPENED! input %s", __FUNCTION__, src);
			while (!feof(fdin)) {
				int bytes = 0;
				if (compressFlag == 0) {
					bytes = fread((void*)copyBuffer1, 1, bufsize, fdin);
					SCP_LOG("READ %d bytes when tried %d bytes", __FUNCTION__, bytes, bufsize);
				}
				if (compressFlag < 0) {
					if (!didFileHeader) {
						compFileHeader_t header;
						if(UiUtil_GetLocalTime( &timeStamp) == 0){
							header.creationTime = timeStamp;
						}
						else {
							header.creationTime = time(NULL);
						}
						header.version = COMP_VERSION;
						int bout = fwrite((void*)&header, 1, sizeof(compFileHeader_t), fdout);
						if (bout != sizeof(compFileHeader_t)) {
							SCP_LOG("Tried to write %d bytes to %s and only wrote %d", __FUNCTION__, sizeof(compFileHeader_t), dst, bout);
							retVal = -1;
							break;
						} else {
							SCP_LOG("Wrote file header to %s", __FUNCTION__, dst);
						}
						didFileHeader = 1;
					}

					bytes = fread((void*)copyBuffer1, 1, bufsize, fdin);
					SCP_LOG("READ %d bytes when tried %d bytes", __FUNCTION__, bytes, bufsize);
					int compLen = bytes;
					compress(copyBuffer2, &compLen, copyBuffer1, bytes);
					SCP_LOG("Compressed %d bytes into %d bytes", __FUNCTION__, bytes, compLen);
					if (bytes == compLen) {
						SCP_LOG("No compression needed for this block.", __FUNCTION__);
						memcpy(copyBuffer2, copyBuffer1, bytes);
					}
					compBlockHeader_t *header = (compBlockHeader_t*)copyBuffer1;
					header->compressedBytes = compLen;
					header->uncompressedBytes = bytes;
					char *startOfData = copyBuffer1+sizeof(compBlockHeader_t);
					memcpy(startOfData, copyBuffer2, compLen);
					bytes = sizeof(compBlockHeader_t)+compLen;
					if (compressedFileSize != NULL) {
						*compressedFileSize += (long)bytes;;
					}
				}
				if (compressFlag > 0) {
					if (!didFileHeader) {
						compFileHeader_t header;
						if (fread((void*)&header, 1, sizeof(compFileHeader_t), fdin) != sizeof(compFileHeader_t)) {
							SCP_LOG("Tried to READ %d bytes of header and didn't", __FUNCTION__, sizeof(compFileHeader_t));
							break;
						}
						if (header.version != COMP_VERSION) {
							SCP_LOG("Unsuported compress file version %d", __FUNCTION__, header.version);
							retVal = -1;
							break;
						}
						didFileHeader = 1;
					}
					compBlockHeader_t header;
					if (fread((void*)&header, 1, sizeof(compBlockHeader_t), fdin) != sizeof(compBlockHeader_t)) {
						SCP_LOG("Tried to READ %d bytes of header and didn't", __FUNCTION__, sizeof(compBlockHeader_t));
						break;
					}
					SCP_LOG("READ %d bytes of header. %d compressed bytes and %d uncompressed bytes", __FUNCTION__, sizeof(compBlockHeader_t), header.compressedBytes, header.uncompressedBytes);
					if ((header.compressedBytes == 0) && (header.uncompressedBytes == 0)) {
						SCP_LOG("Tail header read. EOF!", __FUNCTION__);
						break;
					}
					if ((header.compressedBytes > COMPRESSED_ARRAY_SIZE) || (header.compressedBytes <= 0)) {
						SCP_LOG("Compressed bytes should be 0-%d and it's %d", __FUNCTION__, COMPRESSED_ARRAY_SIZE, header.compressedBytes);
						retVal = -1;
						break;
					}
					if ((header.uncompressedBytes > UNCOMPRESSED_ARRAY_SIZE) || (header.uncompressedBytes <= 0)) {
						SCP_LOG("Uncompressed bytes should be 1-%d and it's %d", __FUNCTION__, UNCOMPRESSED_ARRAY_SIZE, header.uncompressedBytes);
						retVal = -1;
						break;
					}
					if (fread(copyBuffer1, 1, header.compressedBytes, fdin) != header.compressedBytes) {
						SCP_LOG("Tried to READ %d bytes of data and didn't", __FUNCTION__, header.compressedBytes);
						retVal = -1;
						break;
					}
					SCP_LOG("READ %d bytes of data.", __FUNCTION__, header.compressedBytes);
					if (header.uncompressedBytes > header.compressedBytes) {
							SCP_LOG("About to decompress %d bytes", __FUNCTION__, header.compressedBytes);
						int uncompBytes = inflater(copyBuffer2, COPYBUFFER_SIZE, copyBuffer1, header.compressedBytes);
						SCP_LOG("Decompressed %d bytes into %d bytes", __FUNCTION__, header.compressedBytes, uncompBytes);
						memcpy(copyBuffer1, copyBuffer2, uncompBytes);
						SCP_LOG("DECOMP:%2048s", __FUNCTION__, dst);
						bytes = uncompBytes;
					} else {
						SCP_LOG("NO DECOMP NEEDED.", __FUNCTION__);
						bytes = header.compressedBytes;
					}
				}
				if (bytes > 0) {
					SCP_LOG("Writing %d bytes to %s", __FUNCTION__, bytes, dst);
					int bout = fwrite((void*)copyBuffer1, 1, bytes, fdout);
					if (bout != bytes) {
						SCP_LOG("Tried to write %d bytes to %s and only wrote %d", __FUNCTION__, bytes, dst, bout);
						retVal = -1;
						break;
					} else {
						SCP_LOG("Wrote %d bytes to %s", __FUNCTION__, bytes, dst);
					}
				} else {break;}
			}
			if (compressFlag < 0) {
				compBlockHeader_t tailHeader;
				tailHeader.compressedBytes = 0;
				tailHeader.uncompressedBytes = 0;
				int bout = fwrite((void*)&tailHeader, 1, sizeof(compBlockHeader_t), fdout);
				if (bout != sizeof(compBlockHeader_t)) {
					SCP_LOG("Tried to write tail header to %s and only wrote %d", __FUNCTION__, dst, bout);
					retVal = -1;
				} else {
					if (compressedFileSize != NULL) {
						*compressedFileSize += (long)bout;
					}
					SCP_LOG("Wrote tail header to %s", __FUNCTION__, dst);
				}
			}
			fclose(fdin);
			SCP_LOG("Closed fdin", __FUNCTION__);
		} else {
			SCP_LOG("Unable to fopen input file %s", __FUNCTION__, src);
			retVal = -1;
		}

		fclose(fdout);
		SCP_LOG("Closed fdout", __FUNCTION__);
	} else {
		SCP_LOG("Unable to fopen %s as writeable.", __FUNCTION__, dst);
		retVal = -1;
	}
	SCP_LOG("CopyFile returning %d", __FUNCTION__, retVal);
	free(compressedBytesArray);
	free(uncompressedBytesArray);
	free(copyBuffer2);
	free(copyBuffer1);

	return retVal;
}

FILE* openUncompressedFile(char *filename, time_t *time) {
	compFile_t *cf;
	SCP_LOG("openUncompressedFile:%s", __FUNCTION__, filename);

	cf = (compFile_t *)calloc(1, sizeof(compFile_t));
	if (cf == NULL)
		return NULL;

	cf->fdin = fopen(filename, "r");
	if (cf->fdin != NULL) {
		compFileHeader_t header;
		if (fread(&header, 1, sizeof(compFileHeader_t), cf->fdin) == sizeof(compFileHeader_t)) {
			if (time != NULL)
				*time = header.creationTime;
			return (FILE *)cf;
		}
		fclose(cf->fdin);
		SCP_LOG("openUncompressedFile: Unable to read file header for %s.", __FUNCTION__, filename);
	}
	free(cf);
	return NULL;
}

void closeUncompressedFile(FILE *fileIn) {
	compFile_t *cf = (compFile_t *)fileIn;
	SCP_LOG("closeUncompressedFile", __FUNCTION__);
	fclose(cf->fdin);
	free(cf);
}

int readUncompressedBytes(FILE *fileIn, char *buffer, int bufsize) {
	compFile_t *cf = (compFile_t *)fileIn;
	compBlockHeader_t header;
	int bufferIndex = 0;

	SCP_LOG("readUncompressedBytes bufsize:%d", __FUNCTION__, bufsize);
	if (cf->leftOverBytes > 0) {
		SCP_LOG("LEFT OVER BYTES: %d bytes", __FUNCTION__, cf->leftOverBytes);
		if (cf->leftOverBytes >= bufsize) {
			SCP_LOG("COPYING %d BYTES TO BUFFER", __FUNCTION__, bufsize);
			memcpy(buffer, cf->uncompressedBytesArray, bufsize);
			cf->leftOverBytes -= bufsize;
			SCP_LOG("SLIDDING UP uncompressedBytesArray buffer from %d to 0.", __FUNCTION__, bufsize);
			memcpy(cf->uncompressedBytesArray, &cf->uncompressedBytesArray[bufsize], cf->leftOverBytes);
			SCP_LOG("RETURNING %d BYTES READ", __FUNCTION__, bufsize);
			return bufsize;
		} else {
			SCP_LOG("COPYING %d BYTES TO BUFFER", __FUNCTION__, cf->leftOverBytes);
			memcpy(buffer, cf->uncompressedBytesArray, cf->leftOverBytes);
			bufferIndex = cf->leftOverBytes;
			SCP_LOG("bufferIndex=%d", __FUNCTION__, bufferIndex);
			cf->leftOverBytes = 0;
		}
	}

	while((bufferIndex < bufsize) && !feof(cf->fdin) && !cf->endOfCompressedFile) {
		SCP_LOG("bufferIndex=%d", __FUNCTION__, bufferIndex);
		int bread = fread(&header, 1, sizeof(compBlockHeader_t), cf->fdin);
		if (bread != sizeof(compBlockHeader_t)) {
			SCP_LOG("Tried to read header of %d bytes and read %d!", __FUNCTION__, sizeof(compBlockHeader_t), bread);
			// should have got a tail header!
			return -1;
		}
		SCP_LOG("header.uncompressedBytes=%d header.compressedBytes=%d", __FUNCTION__, header.uncompressedBytes, header.compressedBytes);
		if ((header.uncompressedBytes == 0) && (header.compressedBytes == 0)) {
			// no more blocks
			cf->endOfCompressedFile = 1;
			SCP_LOG("Read tail header. EOF!", __FUNCTION__);
			break;
		}
		if ((header.uncompressedBytes > UNCOMPRESSED_ARRAY_SIZE) || (header.uncompressedBytes <= 0)) {
			SCP_LOG("Uncompressed bytes should be 1-%d and it's %d", __FUNCTION__, UNCOMPRESSED_ARRAY_SIZE, header.uncompressedBytes);
			return -1;
		}
		if ((header.compressedBytes > COMPRESSED_ARRAY_SIZE) || (header.compressedBytes <= 0)) {
			SCP_LOG("Compressed bytes should be 1-%d and it's %d", __FUNCTION__, COMPRESSED_ARRAY_SIZE, header.compressedBytes);
			return -1;
		}
		if (fread(cf->compressedBytesArray, 1, header.compressedBytes, cf->fdin) != header.compressedBytes) {
			SCP_LOG("Unable to read %d compressed bytes", __FUNCTION__, header.compressedBytes);
			return -1;
		}
		SCP_LOG("READ %d COMPRESSED BYTES", __FUNCTION__, header.compressedBytes);
		int bytesToCopy = 0;
		int uncompBytes = header.compressedBytes;
		if (header.compressedBytes < header.uncompressedBytes) {
			uncompBytes = inflater(cf->uncompressedBytesArray, UNCOMPRESSED_ARRAY_SIZE,
					cf->compressedBytesArray, header.compressedBytes);
			SCP_LOG("DECOMPRESSED %d BYTES INTO %d BYTES", __FUNCTION__, header.compressedBytes, uncompBytes);
			if (uncompBytes < 0)
				return -1;
			bytesToCopy = uncompBytes;
		} else {
			// this block is not compressed, just copy it
			memcpy(cf->uncompressedBytesArray, cf->compressedBytesArray, header.compressedBytes);
			bytesToCopy = header.compressedBytes;
		}
		if (uncompBytes + bufferIndex > bufsize) {
			// we have extra bytes. Save them for the next call.
			bytesToCopy = bufsize - bufferIndex;
			cf->leftOverBytes = uncompBytes - bytesToCopy;
		}
		SCP_LOG("COPYING %d BYTES FROM uncompressedBytesArray TO BUFFER[%d]", __FUNCTION__, bytesToCopy, bufferIndex);
		memcpy(&buffer[bufferIndex], cf->uncompressedBytesArray, bytesToCopy);
		if (cf->leftOverBytes > 0) {
			SCP_LOG("LEFT OVER BYTES. COPYING %d BYTES FROM %d TO 0.", __FUNCTION__, cf->leftOverBytes, bytesToCopy);
			memcpy(cf->uncompressedBytesArray, &cf->uncompressedBytesArray[bytesToCopy], cf->leftOverBytes);
		}
		bufferIndex += bytesToCopy;
		SCP_LOG("bufferIndex is now:%d", __FUNCTION__, bufferIndex);
	}
	SCP_LOG("EXIT: bufferIndex is now:%d", __FUNCTION__, bufferIndex);
	if (bufferIndex > 0 ) {
		if (bufferIndex < bufsize) {
			buffer[bufferIndex] = 0;
		}
		SCP_LOG("RETURNING: %s", __FUNCTION__, buffer);
	}
	return bufferIndex;
}
