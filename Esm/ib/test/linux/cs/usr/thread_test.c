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

/* [ICS VERSION STRING: unknown] */
#include <ib_types.h>
#include <time.h>

uint32_t must_supply_stack;

extern void
test_thread_create_1 (void);
extern void
test_thread_name_1 (void);
extern void
test_thread_exit_1 (void);
extern void
test_thread_groupname_1 (void);
extern void
test_thread_sleep_1 (void);
extern void
test_thread_kill_1 (void);
extern void
test_thread_join_1 (void);

void  generic_sleep(uint64_t usecs)
{
    struct  timespec  ts;

    ts.tv_sec  =  (time_t) usecs / (time_t) 1000000;
    ts.tv_nsec = (long) usecs % (long) 1000000L;
    (void) nanosleep(&ts, (struct timespec *) 0);
    return;
}

int main (void)
{
    must_supply_stack = 0;
    test_thread_create_1 ();
    test_thread_name_1 ();
    test_thread_exit_1 ();
    test_thread_groupname_1 ();
    test_thread_sleep_1 ();
    test_thread_kill_1 ();
    test_thread_join_1 ();
    return 0;
}
