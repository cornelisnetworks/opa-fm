/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/************************************************************************
* 
* FILE NAME
* 	fe_main.h
*
* DESCRIPTION
* 	Header file for Fabric Executive
*
* DATA STRUCTURES
*	X      
*
* FUNCTIONS
*	X
*
* DEPENDENCIES
* 	X
*
*
* HISTORY
*
* NAME	DATE        	REMARKS
* ----  ---------       ----------------------------------------------
* joc	01/17/2000	Initial file for checkin to source control
*
***********************************************************************/

#ifndef FEMAIN_H
#define FEMAIN_H

				/* Standard include libs		*/
#ifdef __VXWORKS__	 /* VxWorks include files are in a different location. */

#include <stdio.h>
#include <time.h>
#include <types.h>
#include <errno.h>
#include <sys/stat.h>
#include <signal.h>

#else
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/signal.h>
#endif

#define SUCCESS	0
#define FAILED  1

#define LISTEN_PORT	9876	/* Temporary listen port (maybe)	*/

#define NETBUF_SIZ	2048	/* Size of reusable message buffer	*/

#define	ACTIVE		0x01	/* Conn/Seq is active			*/
#define PENDING		0x02	/* Pending sequence			*/
#define COMPLETE	0x03	/* Completed sequence			*/
#define BAD_CONN	0x01	/* Connection is bad or doesn't exist	*/
#define CON_DISC	0x02	/* Schedule a port to disconnect	*/

#endif
