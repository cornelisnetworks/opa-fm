/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/************************************************************************
* 
* FILE NAME
*   netblob.h
*
* DESCRIPTION
*   Libnet protocol buffer structure definitions
*
*
* HISTORY
*
* NAME  DATE            REMARKS
* ----  ---------       ----------------------------------------------
* joc   02/12/2000      Initial file for checkin to source control
* jrw   11/27/2001      Added comments and prologues 
* 
*
***********************************************************************/

#ifndef NET_BLOB_H
#define NET_BLOB_H

/*
 * A NetBlob encapsulates the raw data of a message along with state
 * information needed to retrieve the message.
 *
 * On send, we pack a magic#,len in front of the user data.
 * On recv, we read magic#,len into magic[] then put user data into data.
 * Be careful about this difference!
 */
struct NetBlob_t {
    int len;            /* length of what data points to                    */
    char * data;        /* ptr to userdata send or receive                  */
    int bytesLeft;      /* # bytes of this msg left to send/recv            */
    char * curPtr;      /* ptr into buf where next byte sent/recvd will go  */
    int magic[2];       /* buffer for reading magic#, message len           */
    struct NetBlob_t * next;    /* next blob in the queue                   */
};
typedef struct NetBlob_t NetBlob;

#ifdef WINDOWS
    #ifdef __cplusplus
        #define EXT extern "C"
    #else
        #define EXT
    #endif
#else
    #define EXT
#endif

NetBlob * fe_net_new_blob(int len);
EXT void fe_net_free_buf(char * buf);
void fe_net_free_blob(NetBlob * blob);
int fe_net_adjust_blob_curptr(NetBlob * blob, int bytesSent);

#endif
