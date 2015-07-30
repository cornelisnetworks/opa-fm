/* BEGIN_ICS_COPYRIGHT6 ****************************************

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

** END_ICS_COPYRIGHT6   ****************************************/

#include "datatypes.h"
#include "idebug.h"
#include "ibyteswap.h"

struct buffer {
	char data[80];
	int len;
};

#define BUF_PRINT(buf, format, args...) \
	(buf)->len += sprintf((buf)->data+(buf)->len, format, ## args)

// output character representation of len bytes
static void FormatChars(struct buffer* buf, const uint8* addr, uint32 len)
{
	uint32 j;

	BUF_PRINT(buf, " ");
	for (j = 0; j<len; ++j)
	{
		if (addr[j] >= ' ' && addr[j] <= '~')
		{
			BUF_PRINT(buf, "%c", addr[j]);
		} else {
			BUF_PRINT(buf, ".");
		}
	}
}

void PrintDump( void* addr, uint32 len, dbgDumpFormat_t dumpFormat)
{
	PrintDump2( 0, addr, len, dumpFormat);
}

void PrintDump2( uint32 level, void* addr, uint32 len, dbgDumpFormat_t dumpFormat)
{
	// not very efficient due to numerous IbLogPrintf calls
	// and retesting format each loop but adequate for now
	uint32 i;
	struct buffer buf;

	IbLogPrintf(level, " Dump:\n");
	buf.len=0;
	for (i=0; i<len; )
	{
		if ((i & 15) == 0)
		{
			if (i != 0)
			{
				// output character representation of last 16 bytes
				FormatChars(&buf, &((uint8*)addr)[i-16], 16);
				IbLogPrintf(level, "%s\n", buf.data);
			}
			buf.len = 0;
			BUF_PRINT(&buf, "%4.4x:", i);
		}
		switch (dumpFormat)
		{
			default:
			case _DBG_DUMP_FORMAT_BYTES:
				BUF_PRINT(&buf, " %2.2x", (uint32)((uint8*)addr)[i]);
				++i;
				break;
			case _DBG_DUMP_FORMAT_U64:
				BUF_PRINT(&buf, " %16.16"PRIx64, GetU64(&((uint8*)addr)[i]));
				i+= 8;
				break;
			case _DBG_DUMP_FORMAT_U32:
				BUF_PRINT(&buf, " %8.8x", GetU32(&((uint8*)addr)[i]));
				i+= 4;
				break;
			case _DBG_DUMP_FORMAT_U16:
				BUF_PRINT(&buf, " %4.4x", (uint32)GetU16(&((uint8*)addr)[i]));
				i+= 2;
				break;
			case _DBG_DUMP_FORMAT_LE_U64:
				BUF_PRINT(&buf, " %16.16"PRIx64, GetLeU64(&((uint8*)addr)[i]));
				i+= 8;
				break;
			case _DBG_DUMP_FORMAT_LE_U32:
				BUF_PRINT(&buf, " %8.8x", GetLeU32(&((uint8*)addr)[i]));
				i+= 4;
				break;
			case _DBG_DUMP_FORMAT_LE_U16:
				BUF_PRINT(&buf, " %4.4x", (uint32)GetLeU16(&((uint8*)addr)[i]));
				i+= 2;
				break;
			case _DBG_DUMP_FORMAT_BE_U64:
				BUF_PRINT(&buf, " %16.16"PRIx64, GetBeU64(&((uint8*)addr)[i]));
				i+= 8;
				break;
			case _DBG_DUMP_FORMAT_BE_U32:
				BUF_PRINT(&buf, " %8.8x", GetBeU32(&((uint8*)addr)[i]));
				i+= 4;
				break;
			case _DBG_DUMP_FORMAT_BE_U16:
				BUF_PRINT(&buf, " %4.4x", (uint32)GetBeU16(&((uint8*)addr)[i]));
				i+= 2;
				break;
		}
	}
	if (i != 0)
	{
		// output character representation of last <=16 bytes
		// i is multiple of format length and may exceed len
		// len is actual length desired
		uint32 left = i&15;
		if (left == 0)
		{
			left = 16;
		} else {
			// we need leading spaces for alignment
			uint32 spaces = 0;
			switch (dumpFormat)
			{
				case _DBG_DUMP_FORMAT_BYTES:
					spaces = (3*16) - (left*3);
					break;
				case _DBG_DUMP_FORMAT_U64:
				case _DBG_DUMP_FORMAT_LE_U64:
				case _DBG_DUMP_FORMAT_BE_U64:
					spaces = (17*16/sizeof(uint64)) - ((left/sizeof(uint64))*17);
					break;
				case _DBG_DUMP_FORMAT_U32:
				case _DBG_DUMP_FORMAT_LE_U32:
				case _DBG_DUMP_FORMAT_BE_U32:
					spaces = (9*16/sizeof(uint32)) - ((left/sizeof(uint32))*9);
					break;
				case _DBG_DUMP_FORMAT_U16:
				case _DBG_DUMP_FORMAT_LE_U16:
				case _DBG_DUMP_FORMAT_BE_U16:
					spaces = (5*16/sizeof(uint16)) - ((left/sizeof(uint16))*5);
					break;
			}
			if(spaces)
			{
				BUF_PRINT(&buf, "%*c", spaces, ' ');
			}
		}
		FormatChars(&buf, &((uint8*)addr)[i-left], left - (i-len));
		IbLogPrintf(level, "%s\n", buf.data);
		buf.len = 0;
		BUF_PRINT(&buf, "%4.4x:", i);
	}
}
