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
#ifndef OSA_MESSAGES_H_INCLUDED
#define OSA_MESSAGES_H_INCLUDED

/*!
 @file    Osa/Test_Messages.h
 @brief   Nationalizable Strings and Messages for Osa Package generated from Test.msg
 */

#include "Gen_Arch.h"
#include "Gen_Macros.h"
#include "Log.h"

/* Messages */
#define OSA_MSG_ASSERT_FAILED             LOG_BUILD_MSGID(MOD_OSA, 0)
#define OSA_MSG_TEST1                     LOG_BUILD_MSGID(MOD_OSA, 1)
#define OSA_MSG_TEST2                     LOG_BUILD_MSGID(MOD_OSA, 2)
#define OSA_MSG_TEST3                     LOG_BUILD_MSGID(MOD_OSA, 3)
#define OSA_MSG_TEST4                     LOG_BUILD_MSGID(MOD_OSA, 4)
#define OSA_MSG_TEST5                     LOG_BUILD_MSGID(MOD_OSA, 5)
#define OSA_MSG_TEST6                     LOG_BUILD_MSGID(MOD_OSA, 6)
#define OSA_MSG_TEST7                     LOG_BUILD_MSGID(MOD_OSA, 7)
#define OSA_MSG_TEST8                     LOG_BUILD_MSGID(MOD_OSA, 8)
#define OSA_MSG_TEST9                     LOG_BUILD_MSGID(MOD_OSA, 9)
#define OSA_MSG_TEST10                    LOG_BUILD_MSGID(MOD_OSA, 10)
#define OSA_MSG_TEST11                    LOG_BUILD_MSGID(MOD_OSA, 11)
#define OSA_MSG_TEST12                    LOG_BUILD_MSGID(MOD_OSA, 12)

/* Logging Macros */
/* OSA_MSG_ASSERT_FAILED
 * Arguments: 
 *	expr: text for failed expression
 */
#define OSA_LOG_FINAL_ASSERT_FAILED(expr)\
	LOG_FATAL(MOD_OSA, OSA_MSG_ASSERT_FAILED, expr, 0, 0, 0, 0, 0)
/* OSA_MSG_TEST1
 * Arguments: 
 *	obj1
 */
#define OSA_LOG_FINAL_TEST1(obj1)\
	LOG_FATAL(MOD_OSA, OSA_MSG_TEST1, LOG_FORMATTABLE(obj1), 0, 0, 0, 0, 0)
/* OSA_MSG_TEST2
 * Arguments: 
 *	arg1
 *	arg2
 *	obj1
 *	obj2
 */
#define OSA_LOG_FINAL_TEST2(arg1, arg2, obj1, obj2)\
	LOG_FATAL(MOD_OSA, OSA_MSG_TEST2, arg1, arg2, LOG_FORMATTABLE(obj1), LOG_FORMATTABLE(obj2), 0, 0)
/* OSA_MSG_TEST3
 * Arguments: 
 *	obj1
 *	obj2
 *	width1
 *	width2
 */
#define OSA_LOG_FINAL_TEST3(obj1, obj2, width1, width2)\
	LOG_FATAL(MOD_OSA, OSA_MSG_TEST3, LOG_FORMATTABLE(obj1), LOG_FORMATTABLE(obj2), width1, width2, 0, 0)
/* OSA_MSG_TEST4
 * Arguments: 
 *	obj1
 *	obj2
 *	width1
 *	width2
 */
#define OSA_LOG_FINAL_TEST4(obj1, obj2, width1, width2)\
	LOG_FATAL(MOD_OSA, OSA_MSG_TEST4, LOG_FORMATTABLE(obj1), LOG_FORMATTABLE(obj2), width1, width2, 0, 0)
/* OSA_MSG_TEST5
 * Arguments: 
 *	width1
 *	obj1
 *	width2
 *	obj2
 */
#define OSA_LOG_FINAL_TEST5(width1, obj1, width2, obj2)\
	LOG_FATAL(MOD_OSA, OSA_MSG_TEST5, width1, LOG_FORMATTABLE(obj1), width2, LOG_FORMATTABLE(obj2), 0, 0)
/* OSA_MSG_TEST6
 * Arguments: 
 *	width1
 *	obj1
 *	width2
 *	obj2
 */
#define OSA_LOG_FINAL_TEST6(width1, obj1, width2, obj2)\
	LOG_FATAL(MOD_OSA, OSA_MSG_TEST6, width1, LOG_FORMATTABLE(obj1), width2, LOG_FORMATTABLE(obj2), 0, 0)
/* OSA_MSG_TEST7
 * Arguments: 
 *	width1a
 *	width1b
 *	obj1
 *	width2a
 *	width2b
 *	obj2
 */
#define OSA_LOG_FINAL_TEST7(width1a, width1b, obj1, width2a, width2b, obj2)\
	LOG_FATAL(MOD_OSA, OSA_MSG_TEST7, width1a, width1b, LOG_FORMATTABLE(obj1), width2a, width2b, LOG_FORMATTABLE(obj2))
/* OSA_MSG_TEST8
 * Arguments: 
 *	dec
 *	obj2
 *	width1
 *	width2
 */
#define OSA_LOG_FINAL_TEST8(dec, obj2, width1, width2)\
	LOG_FATAL(MOD_OSA, OSA_MSG_TEST8, dec, LOG_FORMATTABLE(obj2), width1, width2, 0, 0)
/* OSA_MSG_TEST9 */
#define OSA_LOG_PART_TEST9()\
	LOG_ADD_PART(MOD_OSA, OSA_MSG_TEST9, 0, 0, 0, 0, 0, 0)
/* OSA_MSG_TEST10 */
#define OSA_LOG_PART_TEST10()\
	LOG_ADD_PART(MOD_OSA, OSA_MSG_TEST10, 0, 0, 0, 0, 0, 0)
/* OSA_MSG_TEST11 */
#define OSA_LOG_PART_TEST11()\
	LOG_ADD_PART(MOD_OSA, OSA_MSG_TEST11, 0, 0, 0, 0, 0, 0)
/* OSA_MSG_TEST12 */
#define OSA_LOG_PART_TEST12()\
	LOG_ADD_PART(MOD_OSA, OSA_MSG_TEST12, 0, 0, 0, 0, 0, 0)

/* Constant Strings for use as substitutionals in Messages */
#define OSA_STR_MODNAME                   LOG_BUILD_STRID(MOD_OSA, 0)

GEN_EXTERNC(extern void Osa_AddMessages();)

#endif /* OSA_MESSAGES_H_INCLUDED */
