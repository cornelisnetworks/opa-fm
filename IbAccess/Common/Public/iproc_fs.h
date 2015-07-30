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

/* [ICS VERSION STRING: unknown] */

#ifndef _IPROC_FS_H_
#define _IPROC_FS_H_

#include "iba/public/iproc_fs_osd.h"

#ifdef __cplusplus
extern "C" {
#endif


#define NULL_NODE   NULL

/* Below are descriptions of the functions which are provided by iproc_fs_osd.h
 * On some platforms these may be defines, on others they may be actual
 * functions.
 */

/**********************************************************
 *     IBA_PROC_READ_FUNC - "/proc node" read handler
 * 
 * A driver implements a function of this type to handle reads
 * from its registered "/proc node". It should fill in the
 * user data buffer with the data to be returned.
 * 
 * The 'off' argument indicates the byte position within
 * the buffer where the current segment begins. This will
 * always be zero on Darwin, since the entire user buffer
 * is read in a single pass.
 * 
 * The 'eof' argument points to an End Of File flag that the
 * driver should set to 1.
 * 
 * This function should return the number of bytes written into
 * the user data buffer, to be displayed as read data.
 * 
 * Equivalent to Linux read_func_t:
 * 
 * "The read function should write its information into the page.
 * For proper use, the function should start writing at an offset of off in
 * page and write at most count bytes, but because most read functions are
 * quite simple and only return a small amount of information, these two
 * parameters are usually ignored (it breaks pagers like more
 * and less, but cat still works).
 * If the off and count parameters are properly used, eof should be used to
 * signal that the end of the file has been reached by writing 1 to the memory
 * location eof points to.
 * 
 * The read_func function must return the number of bytes written into the page.
 * or a IBA_PROC_RET value to reflect an error code"
 * typedef IBA_PROC_RET (IBA_PROC_READ_FUNC) (
 *     char *buffer,           // User data buffer
 *     char **start,           // Not used
 *     IBA_PROC_OFFSET off,    // Byte offset into buffer
 *     int count,              // max bytes to write
 *     int *eof,               // Returned EOF flag
 *     void *priv              ///Kernel private data
 * );
 ***********************************************************/

/**********************************************************
 *     IBA_PROC_WRITE_FUNC - "/proc node" write handler
 * 
 * A driver implements a function of this type to handle
 * writes to its registered "/proc node". It should copy the
 * data in the user buffer for its own internal use.
 * 
 * Equivalent to Linux write_func_t:
 * 
 * "The write function should read count bytes at maximum from the buffer."
 * 
 * This function returns the number of bytes written
 * or a IBA_PROC_RET value to reflect an error code
 * typedef IBA_PROC_RET (IBA_PROC_WRITE_FUNC) (
 *     IBA_PROC_FILE * file,   // File pointer
 *     const char *buffer,     // User data buffer
 *     unsigned long count,    // Max bytes to read
 *     void *priv              // Kernel private data
 * );
 ***********************************************************/

/**********************************************************
 *     iba_proc_mkdir() - Create a "/proc directory node".
 * 
 * A driver calls this function to create a directory node
 * within the emulated "/proc file system" in IbAccess.  The
 * directory node will be created with the specified file name,
 * under the directory node specified by 'parent'.
 * A NULL value for 'parent' implies the root node ("/proc").
 * 
 * IbAccess returns an opaque handle to that node.
 * 
 * IBA_PROC_NODE *
 * iba_proc_mkdir(
 *     const char              *name,      // File name
 *     IBA_PROC_NODE           *parent     // Parent directory
 * );
 ***********************************************************/

/**********************************************************
 *     iba_create_proc_entry() - Create a "/proc file node".
 * 
 * A driver calls this function to create a file node
 * within the emulated "/proc file system" in IbAccess.  The
 * file node will be created with the specified file name,
 * under the directory node specified by 'parent'.
 * A NULL value for 'parent' implies the root node ("/proc").
 * 
 * IbAccess returns an opaque handle to that node, which can
 * be used later to remove it.
 * 
 * After the node has been created, IbAccess will accept
 * PROC_READ and PROC_WRITE User Client commands directed to
 * this node, calling the driver's registered read and write
 * handlers, respectively. If a NULL handler was registered,
 * then IbAccess will return an Access Error to the user
 * instead. The 'priv' argument is a driver private data
 * structure that IbAccess will associate with the node and
 * return to the driver when calling its read or write
 * handler.
 * 
 * IBA_PROC_NODE *
 * iba_create_proc_entry(
 *     const char              *name,      // File name
 *     IBA_PROC_MODE           mode,       // File mode (N/U on Darwin)
 *     IBA_PROC_NODE           *parent     // Parent directory
 * );
 ***********************************************************/

/**********************************************************
 *     iba_get_proc_*(), iba_set_proc_*()
 * 
 * These are accessor functions for the opaque IBA_PROC_NODE
 * structure.
 * 
 * void
 * iba_set_proc_read_func(
 *     IBA_PROC_NODE           *node,      // File proc node
 *     IBA_PROC_READ_FUNC      *rd_func    // Read handler function
 * );
 * 
 * void
 * iba_set_proc_write_func(
 *     IBA_PROC_NODE           *node,      // File proc node
 *     IBA_PROC_WRITE_FUNC     *wr_func    // Write handler function
 * );
 * 
 * // return >=0 for byte count on success, <0 is a PROC_IBA_RET_* value
 * static _inline IBA_PROC_RET
 * iba_proc_copy_user_data(
 * 	char				*dest,			// kernel destination buffer
 * 	const char*			userBuffer,		// user buffer from IBA_PROC_WRITE_FUNC
 * 	unsigned			bytes			// number of bytes to copy
 * );
 * 
 * void
 * iba_set_proc_data(
 *     IBA_PROC_NODE           *node,      // File proc node
 *     void                    *data       // Private data struct
 * );
 * 
 * void *
 * iba_get_proc_data(
 *     IBA_PROC_NODE           *node       // File proc node
 * );
 * 
 * const char *
 * iba_get_proc_name(
 *     IBA_PROC_NODE           *node       // File proc node
 * );
 ***********************************************************/

/**********************************************************
 *     iba_remove_proc_entry() - Remove a "/proc node".
 * 
 * A driver calls this function to remove a previously created
 * "/proc node", of either file or directory type, specified
 * by the node name and parent directory node.
 * A NULL value for 'parent' implies the root node ("/proc").
 * 
 * void
 * iba_remove_proc_entry(
 *     const char              *name,      // Node name
 *     IBA_PROC_NODE           *parent     // Parent directory
 * );
 ***********************************************************/

/* support function type for iba_proc_read_wrapper
 * called in a premptable context
 *
 * Inputs:
 *	data - context information
 * Outputs:
 *	total_size - returns size of proc text returned (not including any \0)
 * Returns:
 *   	a pointer to text array of data to output for /proc read
 *	can be NULL if unable to allocate or no data available
 *	if NULL is returned, *total_size should be unchanged or 0
 *	caller will MemoryDeallocate when done
 */
typedef char * (*IBA_PROC_FORMAT_FUNC)(void *data, int* total_size);

/* General purpose function to support proc file reads
 * for simple /proc files, this can be provided as the read_func
 *
 * config_data, config_alloc, total_size - static variables in proc function
 *		will be managed here to maintain context across multiple read calls to
 *		this file.
 * func - function to allocate space to text contents and format it
 * func_data - will be passed as argument to provide context of what to format
 * buf, start, offset, maxlen, eof - standard proc read arguments
 */

extern IBA_PROC_RET
iba_proc_read_wrapper(char **config_data, char **config_alloc, int *total_size,
		IBA_PROC_FORMAT_FUNC func, void* func_data,
 		char *buf, char **start, IBA_PROC_OFFSET offset, int maxlen,
		int *eof);

/* for simpler /proc interfaces, this provides a simplified
 * interface.  Caller need only supply a format function and an optional
 * clear function (to be called by write)
 * The function provides all the necessary initialization and handler functions
 * for the /proc interface itself
 */

/* function type for optional clear function, used on write to /proc file */
typedef void (*SIMPLE_PROC_CLEAR_FUNC)(void *context);

#define SIMPLE_PROC_NAME_LEN 80
typedef struct _SIMPLE_PROC {
	boolean			Initialized;
	void 			*Context;	/* callback's context */
	char 			Name[SIMPLE_PROC_NAME_LEN];/* relative file name */
	IBA_PROC_NODE 		*ProcDir;	/* parent directory */
	IBA_PROC_FORMAT_FUNC	FormatFunc;	/* callback function */
	SIMPLE_PROC_CLEAR_FUNC 	ClearFunc;	/* callback function */
	char 			*config_data;	/* for use in read */
	char 			*config_alloc;	/* for use in read */
	int 			total_size;	/* for use in read */
} SIMPLE_PROC;

extern void
SimpleProcInitState(SIMPLE_PROC *pProc);

/* create and initialize /proc file
 * Inputs:
 *	pProc - object to initialize
 *	Name - relative filename in ProcDir
 *	ProcDir - parent directory
 *	Context - context to pass to FormatFunc and ClearFunc
 *	FormatFunc - function to allocate and format /proc read data
 *	ClearFunc - function to clear data in response to /proc write
 * Outputs:
 *	None
 * Returns:
 *	true - initialized and callbacks will begin on user access to /proc
 *	false - unable to create
 *
 * must be called in a preemptable context
 */
extern boolean
SimpleProcInit(SIMPLE_PROC *pProc, const char* Name, IBA_PROC_NODE *ProcDir, 
		void *Context, IBA_PROC_FORMAT_FUNC FormatFunc,
		OPTIONAL SIMPLE_PROC_CLEAR_FUNC ClearFunc);

extern void
SimpleProcDestroy(SIMPLE_PROC *pProc);

#ifdef __cplusplus
}
#endif

#endif /* _IPROC_FS_H_ */
