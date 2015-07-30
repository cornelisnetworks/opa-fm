/* BEGIN_ICS_COPYRIGHT4 ****************************************

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

** END_ICS_COPYRIGHT4   ****************************************/

#include "imemory.h"
#include "idebug.h"
#include "iproc_fs.h"

// General purpose function to support proc file reads
//
// config_data, config_alloc, total_size - static variables in proc function
//		will be managed here to maintain context across multiple read calls to
//		this file.
// func - function to allocate space to text contents and format it
// func_data - will be passed as argument to provide context of what to format
// buf, start, offset, maxlen, eof - standard proc read arguments

IBA_PROC_RET
iba_proc_read_wrapper(char **config_data, char **config_alloc, int *total_size,
		IBA_PROC_FORMAT_FUNC func, void* func_data,
 		char *buf, char **start, IBA_PROC_OFFSET offset, int maxlen,
		int *eof)
{
	int cur_size;

	if (! *config_data) {
		*total_size = 0;	// in case format fails to allocate space
		*config_data = *config_alloc = (*func)(func_data, total_size);
	}

	/*
	 * How much data is available for read: OS spits reads into 512b blocks
	 * and asks each individualy via a separate call to you each time.
	 */
	cur_size = (*total_size <= maxlen ? *total_size : maxlen);

	/*
	 * Is this the last read.. do something to generate
	 * an EOF
	 */
	if (cur_size <= 0) {
		*config_data = NULL;
		if (*config_alloc != NULL)
			MemoryDeallocate(*config_alloc);
		*config_alloc = NULL;
		*eof = 1;
		return 0;
	}
	MemoryCopy((void *)buf, (void *)*config_data, cur_size);

	/*
	 * setup for next call
	 */
	*total_size -= cur_size; // Compute #bytes left to deliver.
	*config_data += cur_size;

	if (*total_size <= 0)
		*eof = 1;
	else
		*eof = 0;

	*start = (char *)(uintn)cur_size;
	return cur_size;
}

void
SimpleProcInitState(SIMPLE_PROC *pProc)
{
	MemoryClear(pProc, sizeof(*pProc));
}

// internal format function if callback occurs for
// /proc object which is uninitialize (eg. being destroyed)
static
char * SimpleProcFormatUninit(void *data, int* total_size)
{
	return NULL;
}

static
IBA_PROC_RET SimpleProcRead(char *buf, char **start, IBA_PROC_OFFSET offset, int maxlen, int *eof, void *data)
{
	SIMPLE_PROC *pProc = (SIMPLE_PROC*)data;

	return iba_proc_read_wrapper(&pProc->config_data, &pProc->config_alloc,
			&pProc->total_size,
			pProc->Initialized ? pProc->FormatFunc
					: SimpleProcFormatUninit,
			pProc->Context,
			buf, start, offset, maxlen, eof);
}

static
IBA_PROC_RET SimpleProcWrite(IBA_PROC_FILE *file, const char *buffer, unsigned long length, void *data)
{
	SIMPLE_PROC *pProc = (SIMPLE_PROC*)data;

	if (pProc->Initialized);
		(*pProc->ClearFunc)(pProc->Context);
	return length;
}

// create and initialize /proc file
// Inputs:
//	pProc - object to initialize
//	Name - relative filename in ProcDir
//	ProcDir - parent directory
//	Context - context to pass to FormatFunc and ClearFunc
//	FormatFunc - function to allocate and format /proc read data
//	ClearFunc - function to clear data in response to /proc write
// Outputs:
//	None
// Returns:
//	true - initialized and callbacks will begin on user access to /proc
//	false - unable to create
//
// must be called in a preemptable context
//
boolean
SimpleProcInit(SIMPLE_PROC *pProc, const char* Name, IBA_PROC_NODE *ProcDir, 
		void *Context, IBA_PROC_FORMAT_FUNC FormatFunc,
		OPTIONAL SIMPLE_PROC_CLEAR_FUNC ClearFunc)
{
	IBA_PROC_NODE 		*entry;

	ASSERT(! pProc->Initialized);
	ASSERT(FormatFunc);
	ASSERT(Name);
	pProc->Context = Context;
	strncpy(pProc->Name, Name, SIMPLE_PROC_NAME_LEN-1);
	pProc->Name[SIMPLE_PROC_NAME_LEN-1] = '\0';
	pProc->ProcDir = ProcDir;
	pProc->FormatFunc = FormatFunc;
	pProc->ClearFunc = ClearFunc;
	
	entry = iba_create_proc_entry(pProc->Name,
				IBA_PROC_MODE_IFREG | IBA_PROC_MODE_IRUGOWUG,
				pProc->ProcDir);
	if (! entry)
	{
		MsgOut("Cannot create proc entry: %s\n", pProc->Name);
		return FALSE;
	}
	iba_set_proc_data(entry, pProc);
	iba_set_proc_read_func(entry, SimpleProcRead);
	iba_set_proc_write_func(entry, ClearFunc?SimpleProcWrite:NULL);
	pProc->Initialized = TRUE;
	return TRUE;
}

void
SimpleProcDestroy(SIMPLE_PROC *pProc)
{
	if (pProc->Initialized)
		iba_remove_proc_entry(pProc->Name, pProc->ProcDir);
	MemoryClear(pProc, sizeof(*pProc));
}
