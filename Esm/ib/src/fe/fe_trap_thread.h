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

#ifndef FE_TRAP_THREAD_H
#define FE_TRAP_THREAD_H

#include "fe_main.h"
#include "fe.h"

#define FE_TRAP_THREAD_STACK_SIZE 64*1024

#define FE_TRAP_THREAD_SLEEP_TIME 5*VTIMER_1S

#define FE_TRAP_THREAD_DATA_LEN 256

typedef struct
{
	Thread_t      thread_id;
	Lock_t        lock;
	FE_Trap_t    *trap_list, *trap_list_end;
	FE_Trap_t     trap128;
	uint32_t      count;
	int           trap128_received;
	unsigned char data[FE_TRAP_THREAD_DATA_LEN];
} FE_TrapThreadData_t;

extern uint32_t fe_trap_thread_create(void);

extern uint32_t fe_trap_thread_get_trapcount(void);
extern void fe_trap_thread_get_traplist(FE_Trap_t **list, uint32_t *count);
extern void fe_trap_thread_free_traps(void);
extern uint32_t fe_trap_thread_pause(void);
extern void fe_trap_thread_resume(void);

#endif // FE_TRAP_THREAD_H
