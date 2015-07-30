/* BEGIN_ICS_COPYRIGHT3 ****************************************

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

** END_ICS_COPYRIGHT3   ****************************************/

/* [ICS VERSION STRING: unknown] */

/* Suppress duplicate loading of this file */
#ifndef _IBA_IB_DEBUG_H_
#define _IBA_IB_DEBUG_H_

/*
 * GLOBAL LEVELS
 *
 *
 * Range reserved for the component specific values is
 * 0x00000001 - 0x00FFFFFF
 */

#include "iba/public/statustext.h"

#ifdef VXWORKS
#undef ICS_LOGGING
#define ICS_LOGGING 1
#endif

/* This macro helps to catch macro calls with too many arguments
 * if arg is not 0, it should cause a syntax error
 * by causing a 0x5 for a good arg,
 * even numeric non-zero values will cause syntax errors
 */
#define _ICS_MUST_BE_ZERO(arg) \
	{	if (arg##x5 != 0x5) \
		{ \
		} \
	}

#ifdef ICS_LOGGING
#include "Log.h"
#include "Osa_Context.h"

/* Macro to cast a logging pointer argument as needed */
#define _DBG_PTR(arg) LOG_PTR(arg)
#define _TRC_PTR(arg) LOG_PTR(arg)

#else /* ICS_LOGGING */

#define _DBG_PTR(arg) (arg)
#define _TRC_PTR(arg) (arg)

#endif /* ICS_LOGGING */

/* These levels are available in debug and release builds */
#define	_DBG_LVL_DISABLE		0
#define	_DBG_LVL_INFO			0x10000000
#define	_DBG_LVL_WARN			0x20000000
#define	_DBG_LVL_ERROR			0x40000000
#define	_DBG_LVL_FATAL			0x80000000
#define	_DBG_LVL_ALL			0xFFFFFFFF
#define	_DBG_LVL_NONDEBUG		0xF0000000	/* options allowed in release build */

#if defined( DBG) || defined( IB_DEBUG)

#define	_DBG_LVL_FUNC_TRACE		0x01000000
#define	_DBG_LVL_MEM_TRACK		0x02000000
#define	_DBG_LVL_MASK			_DBG_LVL_ALL	/* levels to allow */

#else
/* disabled debug levels */
#define	_DBG_LVL_FUNC_TRACE		0
#define	_DBG_LVL_MEM_TRACK		0
#define	_DBG_LVL_MASK			_DBG_LVL_NONDEBUG	/* levels to allow */

#endif	/* DBG || IB_DEBUG */

#ifdef IB_TRACE
#define	_TRC_LVL_FUNC_TRACE		0x80000000
#define	_TRC_LVL_MEM_TRACK		0x40000000
#define	_TRC_LVL_MASK			0xffffffff
#else /* IB_TRACE */
/* disable trace levels */
#define	_TRC_LVL_FUNC_TRACE		0
#define	_TRC_LVL_MEM_TRACK		0
#define	_TRC_LVL_MASK			0
#endif /* IB_TRACE */

#define	_DBG_BREAK_ENABLE		TRUE
#define	_DBG_BREAK_DISABLE		FALSE


#ifdef __cplusplus
extern "C" {
#endif

typedef struct _IB_DBG_PARAMS{
	uint32	debug_level;		/* Current debug output level */
#if defined(DBG) || defined( IB_DEBUG) || defined( ICS_LOGGING)
	uint32	break_enable;		/* Should breakpoints be taken? */
#endif
	uint32	mem_tag;			/* memory tag value // should this be here? */
	char	*module;			/* name of component */
#ifdef ICS_LOGGING
	Mod_Id_t	modId;			/* ICS module Id of component */
#endif
	uint32	trace_level;		/* Current trace output level */
	uint32	trace_id;			/* registered trace id of component */
} IB_DBG_PARAMS;

#ifdef __cplusplus
};
#endif

/*
 * Reference this macro in only one .c file of your component
 * it builds the module specific debug table, such that macros below
 * can determine the module Id, Mem Tag, etc.
 * For ICS_LOGGING it also creates the Ics logging subsystem
 * initialization code and string table with module Id
 * MOD_PREFIX should be the unquoted mixed case prefix for the module.  It will
 * prefix the name of the Initialize method
 *
 * The use of this macro is typically done with _ib_dbg_params redefined
 * as a module specific name, for uniqueness, so linker won't find duplicates
 */
#ifdef ICS_LOGGING
#define _IB_DBG_PARAM_BLOCK(LEVEL,BREAK,MEM_TAG,MOD_NAME,MOD_ID,MOD_PREFIX) \
	IB_DBG_PARAMS _ib_dbg_params = {LEVEL, BREAK, MEM_TAG, MOD_NAME ":",MOD_ID, 0, 0}; \
	static Log_StringEntry_t _ics_stringTable[] = \
	{ \
		{	LOG_BUILD_STRID(MOD_ID, 0), \
			{ MOD_NAME, \
			} \
		} \
	}; \
	void MOD_PREFIX##_Initialize() \
	{ \
		Log_AddStringTable(MOD_ID, _ics_stringTable, 1); \
	} \
	void MOD_PREFIX##_SetDebugLevel(uint32 level) \
	{ \
		__DBG_LEVEL__ = level; \
	} \
	uint32 MOD_PREFIX##_GetDebugLevel() \
	{ \
		return __DBG_LEVEL__; \
	} \
	void MOD_PREFIX##_SetBreakFlag(uint32 enable) \
	{ \
		__DBG_BRK_LEVEL__ = enable; \
	} \
	uint32 MOD_PREFIX##_GetBreakFlag() \
	{ \
		return __DBG_BRK_LEVEL__; \
	}
#else /* ICS_LOGGING */
#if defined( DBG) ||defined( IB_DEBUG)
#define _IB_DBG_PARAM_BLOCK(LEVEL,BREAK,MEM_TAG,MOD_NAME) \
	IB_DBG_PARAMS _ib_dbg_params = {LEVEL, BREAK, MEM_TAG, MOD_NAME ":", 0, 0};
#else
#define _IB_DBG_PARAM_BLOCK(LEVEL,BREAK,MEM_TAG,MOD_NAME) \
	IB_DBG_PARAMS _ib_dbg_params = {LEVEL, MEM_TAG, MOD_NAME ":", 0, 0};
#endif /* DBG || IB_DEBUG */
#endif /* ICS_LOGGING */

/* Declare debug param block as extern for those object modules
 * not declaring the block.
 */
extern IB_DBG_PARAMS	_ib_dbg_params;

#define	__DBG_LEVEL__		_ib_dbg_params.debug_level
#define	__DBG_BRK_LEVEL__	_ib_dbg_params.break_enable
#define	__DBG_MEM_TAG__		_ib_dbg_params.mem_tag
#define	__MOD_NAME__		_ib_dbg_params.module
#define	__MOD_ID__			_ib_dbg_params.modId
#define	__DBG_TRACE_LEVEL__	_ib_dbg_params.trace_level
#define _DBG_DEFINE_FUNC(NAME)						\
	static const char	__FUNC__[] = #NAME;
    
#ifdef  ICS_LOGGING
/* =============================================================================
 * Here we go, it takes a lot of preprocessor work to translate the
 * intel variable argument (fmt, args) style STRING arg into
 * the ICS style fixed argument list, which also includes a module Id
 */

#define _ICS_DBG3_CALL(suffix, fmt, arg1, arg2, arg3, arg4, arg5, arg6, arg7, extraargs...) \
	do { \
		_ICS_MUST_BE_ZERO(arg7) \
		LOG_##suffix(__MOD_ID__, fmt, arg1, arg2, arg3, arg4, arg5, arg6); \
	} while (0)

/* we now have STRING as normal args, and can adjust
 * get at least 6 real arguments (and a 7th zero to check for undesired extra
 * arguments) to format string
 */
#define _ICS_DBG2_DUMP(fmt, args...) \
	_ICS_DBG3_CALL(DUMP, fmt, ## args, 0,0,0,0,0,0,0)
#define _ICS_DBG2_DEBUG1(fmt, args...) \
	_ICS_DBG3_CALL(DEBUG1, fmt, ## args, 0,0,0,0,0,0,0)
#define _ICS_DBG2_DEBUG2(fmt, args...) \
	_ICS_DBG3_CALL(DEBUG2, fmt, ## args, 0,0,0,0,0,0,0)
#define _ICS_DBG2_DEBUG3(fmt, args...) \
	_ICS_DBG3_CALL(DEBUG3, fmt, ## args, 0,0,0,0,0,0,0)
#define _ICS_DBG2_DEBUG4(fmt, args...) \
	_ICS_DBG3_CALL(DEBUG4, fmt, ## args, 0,0,0,0,0,0,0)
#define _ICS_DBG2_DEBUG5(fmt, args...) \
	_ICS_DBG3_CALL(DEBUG5, fmt, ## args, 0,0,0,0,0,0,0)

/* redefine MsgOut from idebug.h for those that call it directly */
#undef MsgOut
#define MsgOut(fmt, args...) \
	_ICS_DBG3_CALL(DUMP, fmt, ## args, 0,0,0,0,0,0,0)

/* redefine DbgOut from idebug.h for those that call it directly */
#undef DbgOut
#define DbgOut(fmt, args...) \
	_ICS_DBG3_CALL(DEBUG1, fmt, ## args, 0,0,0,0,0,0,0)

/* grab () off string and use to form macro call, so we can extract args */
#define _ICS_DBG_DUMP(STRING) \
	_ICS_DBG2_DUMP  STRING
#define _ICS_DBG_DEBUG1(STRING) \
	_ICS_DBG2_DEBUG1  STRING
#define _ICS_DBG_DEBUG2(STRING) \
	_ICS_DBG2_DEBUG2  STRING
#define _ICS_DBG_DEBUG3(STRING) \
	_ICS_DBG2_DEBUG3  STRING
#define _ICS_DBG_DEBUG4(STRING) \
	_ICS_DBG2_DEBUG4  STRING
#define _ICS_DBG_DEBUG5(STRING) \
	_ICS_DBG2_DEBUG5  STRING

/* translate from Intel LEVELs to Ics Severities
 * switch statement should be dealing with constants and optimize into
 * only 1 case worth of code.  switch is needed since we can't
 * slam the level into STRING within its parens
 */
#if defined(DBG) || defined(IB_DEBUG)
#define _ICS_DBG_OUT(LEVEL,STRING) \
		switch (LEVEL & _DBG_LVL_MASK)				\
		{											\
			case 0:									\
				break;								\
			case _DBG_LVL_FATAL:					\
				_ICS_DBG_DUMP(STRING);				\
				break;								\
			case _DBG_LVL_ERROR:					\
				_ICS_DBG_DEBUG1(STRING);			\
				break;								\
			case _DBG_LVL_WARN:						\
				_ICS_DBG_DEBUG2(STRING);			\
				break;								\
			case _DBG_LVL_INFO:						\
				_ICS_DBG_DEBUG3(STRING);			\
				break;								\
			case _DBG_LVL_MEM_TRACK:				\
				_ICS_DBG_DEBUG4(STRING);			\
				break;								\
			default:								\
			case _DBG_LVL_FUNC_TRACE:				\
				_ICS_DBG_DEBUG5(STRING);			\
				break;								\
		}
#else
#define _ICS_DBG_OUT(LEVEL,STRING) \
		switch (LEVEL & _DBG_LVL_MASK)				\
		{											\
			case 0:									\
				break;								\
			case _DBG_LVL_FATAL:					\
				_ICS_DBG_DUMP(STRING);				\
				break;								\
			case _DBG_LVL_ERROR:					\
				_ICS_DBG_DEBUG1(STRING);			\
				break;								\
			case _DBG_LVL_WARN:						\
				_ICS_DBG_DEBUG2(STRING);			\
				break;								\
			case _DBG_LVL_INFO:						\
				_ICS_DBG_DEBUG3(STRING);			\
				break;								\
			default:								\
				_ICS_DBG_DEBUG5(STRING);			\
				break;								\
		}
#endif

/* PROLOG and EPILOG were used by this module, however all
 * callers define them as empty macros
 * Hence the use in the ICS_LOGGING version has been removed
 * As a result the __DBG_LEVEL__ is not used, the ICS Logging controls
 * are the primary and only control for these logging settings
 * Hence the initial settings in TTS for __DBG_LEVEL__ are ignored
 */
#define _DBG_PRINT(LEVEL,STRING)	 _ICS_DBG_OUT(LEVEL,STRING)

#define _DBG_DUMP_MEMORY(LEVEL,STRING,ADDR,LENGTH)	\
	_ICS_DBG_OUT(LEVEL, ("%s:\n%F", _DBG_PTR(STRING), Log_MemoryDump(ADDR, LENGTH)))

/* TBD - BUGBUG - need to translate DUMPFORMAT from _DBG_DUMP_FORMAT_*
 * to LOG_DUMP_FORMAT_*
 */
#if 0
#define _DBG_DUMP_MEMORY_FORMAT(LEVEL,STRING,ADDR,LENGTH,DUMPFORMAT)
	_ICS_DBG_OUT(LEVEL, ("%s:\n%F", _DBG_PTR(STRING), Log_MemoryDumpFormat(ADDR, LENGTH,DUMPFORMAT)))
#endif

#define _DBG_BREAK									\
	if (__DBG_BRK_LEVEL__)							\
	{												\
		LOG_DUMP (__MOD_ID__, "=>Breakpoint at %s() in %s(%d)\n",_DBG_PTR(__FUNCTION__),_DBG_PTR(__FILE__),(uintn)__LINE__,0,0,0);	\
		Osa_BreakPoint();							\
	}

#define _DBG_INIT	(void)0

#if defined(DBG) || defined(IB_DEBUG)
#define _ICS_DBG2_ENTER_FUNC(FORMAT, ARGS...) \
	_ICS_DBG3_CALL(DEBUG5, "%s(" FORMAT ") [\n", \
					_DBG_PTR(__FUNCTION__), ## ARGS,0,0,0,0,0,0,0)

#define _ICS_DBG_ENTER_FUNC(STRING) _ICS_DBG2_ENTER_FUNC STRING
/*
 *    FOR DEBUG BUILDS
 * Include One of these functions at the start of your function code
 * This must follow the variable declarations but preceed any 
 * functional code in the function
 */
#define _DBG_ENTER_LVL(LEVEL,NAME) \
    _DBG_DEFINE_FUNC(NAME) \
    (void)__FUNC__; \
	_DBG_PRINT (LEVEL, ( "%s() [\n" , _DBG_PTR(__FUNCTION__) ));

#define _DBG_ENTER_LVL_ARGS(LEVEL,NAME,STRING) \
    _DBG_DEFINE_FUNC(NAME) \
     (void)__FUNC__; \
	_ICS_DBG_ENTER_FUNC(STRING)

#define _DBG_ENTER_FUNC(NAME) \
    _DBG_DEFINE_FUNC(NAME) \
    (void)__FUNC__; \
	_DBG_PRINT (_DBG_LVL_FUNC_TRACE, ("%s() [\n",_DBG_PTR(__FUNCTION__)));

#define _DBG_ENTER_FUNC_ARGS(NAME, STRING) \
    _DBG_DEFINE_FUNC(NAME) \
    (void)__FUNC__; \
	_ICS_DBG_ENTER_FUNC(STRING)

#define _DBG_ENTER_EXT(LEVEL,NAME,IRQL) \
    _DBG_DEFINE_FUNC(NAME) \
    (void)__FUNC__; \
	do {											\
		_DBG_PRINT(LEVEL, ("%s() [\n",_DBG_PTR(__FUNCTION__))); \
		_DBG_CHK_IRQL(IRQL);						\
	} while (0)

#define _DBG_ENTER_EXT_ARGS(LEVEL,NAME,IRQL,STRING) \
    _DBG_DEFINE_FUNC(NAME) \
    (void)__FUNC__; \
	do {											\
		_ICS_DBG_ENTER_FUNC(STRING); \
		_DBG_CHK_IRQL(IRQL);						\
	} while (0)
#else
#define _ICS_DBG_ENTER_FUNC(STRING) (void)0
/*
 *    FOR RELEASE BUILDS
 * Include One of these functions at the start of your function code
 * This must follow the variable declarations but preceed any 
 * functional code in the function
 */
#define _DBG_ENTER_LVL(LEVEL,NAME) \
	_DBG_PRINT (LEVEL, ( "%s() [\n" , _DBG_PTR(__FUNCTION__) ));

#define _DBG_ENTER_LVL_ARGS(LEVEL,NAME,STRING) \
	_ICS_DBG_ENTER_FUNC(STRING)

#define _DBG_ENTER_FUNC(NAME) \
	_DBG_PRINT (_DBG_LVL_FUNC_TRACE, ("%s() [\n",_DBG_PTR(__FUNCTION__)));

#define _DBG_ENTER_FUNC_ARGS(NAME, STRING) \
	_ICS_DBG_ENTER_FUNC(STRING)

#define _DBG_ENTER_EXT(LEVEL,NAME,IRQL) \
	do {											\
		_DBG_PRINT(LEVEL, ("%s() [\n",_DBG_PTR(__FUNCTION__))); \
		_DBG_CHK_IRQL(IRQL);						\
	} while (0)

#define _DBG_ENTER_EXT_ARGS(LEVEL,NAME,IRQL,STRING) \
	do {											\
		_ICS_DBG_ENTER_FUNC(STRING); \
		_DBG_CHK_IRQL(IRQL);						\
	} while (0)
#endif  /* release */


/*
 * Include one of these functions at the exit of your function 
 */
#define	_DBG_LEAVE_FUNC()											\
	_DBG_PRINT (_DBG_LVL_FUNC_TRACE, ("%s() ]\n",_DBG_PTR(__FUNCTION__)));

#define	_DBG_RETURN_FUNC(VALUE)	\
	_DBG_PRINT (_DBG_LVL_FUNC_TRACE, ("%s() returning 0x%x ]\n",_DBG_PTR(__FUNCTION__), VALUE))

#define	_DBG_FSTATUS_RETURN_FUNC(STATUS) \
	_DBG_PRINT (_DBG_LVL_FUNC_TRACE, ("%s() returning %s (0x%x) ]\n",_DBG_PTR(__FUNCTION__), _DBG_PTR(iba_fstatus_msg(STATUS)), STATUS))

#define	_DBG_LEAVE_LVL(LEVEL) \
	_DBG_PRINT (LEVEL, ("%s() ]\n",_DBG_PTR(__FUNCTION__)));

#define	_DBG_RETURN_LVL(LEVEL, VALUE) \
	_DBG_PRINT (LEVEL, ("%s() returning 0x%x ]\n",_DBG_PTR(__FUNCTION__), VALUE))

#define	_DBG_FSTATUS_RETURN_LVL(LEVEL, STATUS) \
	_DBG_PRINT (LEVEL, ("%s() returning %s (0x%x) ]\n",_DBG_PTR(__FUNCTION__), _DBG_PTR(iba_fstatus_msg(STATUS)), STATUS))

#define	_DBG_LEAVE_EXT _DBG_LEAVE_LVL
#define	_DBG_RETURN_EXT _DBG_RETURN_LVL
#define	_DBG_FSTATUS_RETURN_EXT _DBG_FSTATUS_RETURN_LVL

#define _DBG_INFO(STRING) _DBG_PRINT (_DBG_LVL_INFO, STRING);

#define _DBG_WARN(STRING)	_ICS_DBG_OUT(_DBG_LVL_WARN,STRING)

#define _DBG_OUT	_DBG_INFO	

#define _DBG_ERROR(STRING)	 _ICS_DBG_OUT(_DBG_LVL_ERROR,STRING)

#define _DBG_FATAL(STRING)											\
	do { 															\
		_ICS_DBG_OUT(_DBG_LVL_FATAL,STRING)							\
		_DBG_BREAK													\
	} while(0)

#define	_DBG_MEM(STRING)	_ICS_DBG_OUT(_DBG_LVL_MEM_TRACK,STRING)

#define _DBG_STRDUP Log_StrDup

#else /* ICS_LOGGING */

#define _DBG2_CALL(fmt, args...) \
	IbLogPrintf(_local_dbg_level, fmt, ## args); \

/* grab () off string and use to form macro call, so we can add to args */
#define _DBG1_CALL(LEVEL, STRING) \
	do { \
		/* local is for _DBG2_CALL to use */ \
		/* compiler will not actually allocate stack space */ \
		uint32 _local_dbg_level = LEVEL; \
		_DBG2_CALL  STRING \
	} while (0)

#if  defined(DBG) || defined( IB_DEBUG)
/* =============================================================================
 * In these function-like macros, we enclose the multi-line expansion with
 * do {} while(0) This causes the macro to behave as a single statement
 * and avoid surprises if the macro is used without {} in an if
 * such as:
 *		if (x)
 *			_DBG_PRINT(("x is true\n"));
 */
			
#define _DBG_PRINT(LEVEL,STRING)					\
	do {											\
		_DBG_PRINT_PROLOG(LEVEL,STRING)				\
		if (__DBG_LEVEL__ & LEVEL)					\
		{											\
			_DBG1_CALL(LEVEL, (__MOD_NAME__));		\
			_DBG1_CALL(LEVEL, STRING);				\
		}											\
		_DBG_PRINT_EPILOG(LEVEL,STRING)				\
	} while (0)

#define _DBG_DUMP_MEMORY(LEVEL,STRING,ADDR,LENGTH)	\
	do {											\
		_DBG_PRINT_PROLOG(LEVEL,STRING)				\
		if (__DBG_LEVEL__ & LEVEL)					\
		{											\
			_DBG1_CALL(LEVEL,  (__MOD_NAME__));		\
			_DBG1_CALL(LEVEL,  STRING);				\
			DbgDump (LEVEL, ADDR, LENGTH,_DBG_DUMP_FORMAT_BYTES);\
		}											\
		_DBG_PRINT_EPILOG(LEVEL,STRING)				\
	} while (0)

#define _DBG_DUMP_MEMORY_FORMAT(LEVEL,STRING,ADDR,LENGTH,DUMPFORMAT)	\
	do {											\
		_DBG_PRINT_PROLOG(LEVEL,STRING)				\
		if (__DBG_LEVEL__ & LEVEL)					\
		{											\
			_DBG1_CALL(LEVEL,  (__MOD_NAME__));		\
			_DBG1_CALL(LEVEL,  STRING);				\
			DbgDump (LEVEL, ADDR, LENGTH, DUMPFORMAT);\
		}											\
		_DBG_PRINT_EPILOG(LEVEL,STRING)				\
	} while (0)
	
#define _DBG_BREAK									\
	do {											\
		if (__DBG_BRK_LEVEL__)						\
		{											\
			DbgOut ("=>Breakpoint at %s() in %s(%d)\n", \
					_DBG_PTR(__FUNC__),_DBG_PTR(__FILE__),__LINE__);	\
			DbgBreakPoint();						\
		}											\
	} while (0)

/*
 * This may be referenced at the start of your code like DriverEntry, main, etc.
 * to locate the debug param block.
 *
 * Do not change the string. It will be used in debugger extensions.
 */
#define _DBG_INIT									\
	do {											\
		DbgOut ("[IbDebug] module %s at 0x%p\n",	\
				 _DBG_PTR(__MOD_NAME__), _DBG_PTR(&__DBG_LEVEL__));	\
	} while (0)



/*
 * Include One of these functions at the start of your function code
 * This must follow the variable declarations but preceed any 
 * functional code in the function
 */

#define _DBG_ENTER_LVL(LEVEL,NAME) \
	_DBG_DEFINE_FUNC(NAME);							\
	_DBG_PRINT (LEVEL, ("%s() [\n",_DBG_PTR(__FUNC__)))

#define _DBG_ENTER_LVL_ARGS(LEVEL,NAME,STRING) \
	_DBG_DEFINE_FUNC(NAME); \
	do {											\
		_DBG_PRINT_PROLOG(LEVEL, ("%s() [\n", _DBG_PTR(__FUNC__)))	\
		if (__DBG_LEVEL__ & LEVEL)					\
		{											\
			_DBG1_CALL(LEVEL, (__MOD_NAME__));		\
			_DBG1_CALL(LEVEL, ("%s(" ,_DBG_PTR(__FUNC__)) );	\
			_DBG1_CALL(LEVEL, STRING);				\
			_DBG1_CALL(LEVEL, (") [\n"));			\
		}											\
		_DBG_PRINT_EPILOG(LEVEL,("%s() [\n", _DBG_PTR(__FUNC__)))	\
	} while (0)

#define _DBG_ENTER_FUNC(NAME) \
	_DBG_ENTER_LVL(_DBG_LVL_FUNC_TRACE,NAME)

#define _DBG_ENTER_FUNC_ARGS(NAME, STRING) \
	_DBG_ENTER_LVL_ARGS(_DBG_LVL_FUNC_TRACE,NAME,STRING)

#define _DBG_ENTER_EXT(LEVEL,NAME,IRQL) \
	_DBG_DEFINE_FUNC(NAME);							\
	do {											\
		_DBG_PRINT(LEVEL, ("%s() [\n",_DBG_PTR(__FUNC__)));	\
		_DBG_CHK_IRQL(IRQL);						\
	} while (0)

#define _DBG_ENTER_EXT_ARGS(LEVEL,NAME,IRQL,STRING) \
	_DBG_DEFINE_FUNC(NAME); \
	do {											\
		_DBG_PRINT_PROLOG(LEVEL, ("%s() [\n", _DBG_PTR(__FUNC__)))	\
		if (__DBG_LEVEL__ & LEVEL)					\
		{											\
			_DBG1_CALL(LEVEL, (__MOD_NAME__));		\
			_DBG1_CALL(LEVEL, ("%s(" ,_DBG_PTR(__FUNC__)) );	\
			_DBG1_CALL(LEVEL, STRING);				\
			_DBG1_CALL(LEVEL, (") [\n"));			\
		}											\
		_DBG_PRINT_EPILOG(LEVEL,("%s() [\n", _DBG_PTR(__FUNC__)))	\
		_DBG_CHK_IRQL(IRQL);						\
	} while (0)

/*
 * Include one of these functions at the exit of your function 
 */
#define	_DBG_LEAVE_FUNC() \
	_DBG_PRINT (_DBG_LVL_FUNC_TRACE, ("%s() ]\n",_DBG_PTR(__FUNC__)))

#define	_DBG_RETURN_FUNC(VALUE)	\
	_DBG_PRINT (_DBG_LVL_FUNC_TRACE, ("%s() returning 0x%x ]\n",_DBG_PTR(__FUNC__), VALUE))

#define	_DBG_FSTATUS_RETURN_FUNC(STATUS) \
	_DBG_PRINT (_DBG_LVL_FUNC_TRACE, ("%s() returning %s (0x%x) ]\n",_DBG_PTR(__FUNC__), _DBG_PTR(iba_fstatus_msg(STATUS)), STATUS))

#define	_DBG_LEAVE_LVL(LEVEL) \
	_DBG_PRINT (LEVEL, ("%s() ]\n",_DBG_PTR(__FUNC__)))

#define	_DBG_RETURN_LVL(LEVEL, VALUE) \
	_DBG_PRINT (LEVEL, ("%s() returning 0x%x ]\n",_DBG_PTR(__FUNC__), VALUE))

#define	_DBG_FSTATUS_RETURN_LVL(LEVEL, STATUS) \
	_DBG_PRINT (LEVEL, ("%s() returning %s (0x%x) ]\n",_DBG_PTR(__FUNC__), _DBG_PTR(iba_fstatus_msg(STATUS)), STATUS))

#define	_DBG_LEAVE_EXT _DBG_LEAVE_LVL
#define	_DBG_RETURN_EXT _DBG_RETURN_LVL
#define	_DBG_FSTATUS_RETURN_EXT _DBG_FSTATUS_RETURN_LVL


#define _DBG_INFO(STRING)											\
	_DBG_PRINT (_DBG_LVL_INFO, STRING)

#define _DBG_WARN(STRING)											\
	do {															\
		_DBG_PRINT_PROLOG(_DBG_LVL_WARN,STRING)						\
		if (__DBG_LEVEL__ & _DBG_LVL_WARN)							\
		{															\
			_DBG1_CALL(_DBG_LVL_WARN, ("%s%s() !WARNING! ",			\
					 _DBG_PTR(__MOD_NAME__),_DBG_PTR(__FUNC__)));	\
			_DBG1_CALL(_DBG_LVL_WARN, STRING);						\
		}															\
		_DBG_PRINT_EPILOG(_DBG_LVL_WARN,STRING)						\
	} while (0)

#define _DBG_OUT	_DBG_INFO	

#define _DBG_ERROR(STRING)											\
	do {															\
		_DBG_ERROR_PROLOG(_DBG_LVL_ERROR,STRING)					\
		if (__DBG_LEVEL__ & _DBG_LVL_ERROR)							\
		{															\
			_DBG1_CALL(_DBG_LVL_ERROR, ("%s%s() !ERROR! ",			\
					 _DBG_PTR(__MOD_NAME__),_DBG_PTR(__FUNC__)));	\
			_DBG1_CALL(_DBG_LVL_ERROR, STRING);						\
		}															\
		_DBG_ERROR_EPILOG(_DBG_LVL_ERROR,STRING)					\
	} while (0)

#define _DBG_FATAL(STRING)											\
	do {															\
		_DBG_FATAL_PROLOG(_DBG_LVL_FATAL,STRING)					\
		if (__DBG_LEVEL__ & _DBG_LVL_FATAL)							\
		{															\
			_DBG1_CALL(_DBG_LVL_FATAL, ("%s%s() !!!FATAL ERROR!!! ",\
					 _DBG_PTR(__MOD_NAME__),_DBG_PTR(__FUNC__)));	\
			_DBG1_CALL(_DBG_LVL_FATAL, STRING);						\
			_DBG_BREAK												\
		}															\
		_DBG_FATAL_EPILOG(_DBG_LVL_FATAL,STRING)					\
	} while(0)

#define	_DBG_MEM(STRING)											\
	do {															\
		if (__DBG_LEVEL__ & _DBG_LVL_MEM_TRACK)						\
		{															\
			_DBG1_CALL(_DBG_LVL_MEM_TRACK, ("%s%s()[mem] ",			\
				 		_DBG_PTR(__MOD_NAME__),_DBG_PTR(__FUNC__)));\
			_DBG1_CALL(_DBG_LVL_MEM_TRACK, STRING);					\
		}															\
	} while(0)

#else	/* DBG || IB_DEBUG */

/* =============================================================================
 * release builds include this
 */

#define _DBG_PRINT(LEVEL,STRING)					\
	do {											\
	  if (LEVEL) {									\
		_DBG_PRINT_PROLOG(LEVEL,STRING)				\
		if (__DBG_LEVEL__ & LEVEL & _DBG_LVL_MASK)	\
		{											\
			_DBG1_CALL(LEVEL, (__MOD_NAME__));		\
			_DBG1_CALL(LEVEL, STRING);				\
		}											\
		_DBG_PRINT_EPILOG(LEVEL,STRING)				\
	  }												\
	} while (0)

#define _DBG_DUMP_MEMORY(LEVEL,STRING,ADDR,LENGTH)	\
	do {											\
	  if (LEVEL & _DBG_LVL_MASK) {					\
		_DBG_PRINT_PROLOG(LEVEL,STRING)				\
		if (__DBG_LEVEL__ & LEVEL)					\
		{											\
			_DBG1_CALL(LEVEL,  (__MOD_NAME__));		\
			_DBG1_CALL(LEVEL,  STRING);				\
			DbgDump (LEVEL, ADDR, LENGTH,_DBG_DUMP_FORMAT_BYTES);\
		}											\
		_DBG_PRINT_EPILOG(LEVEL,STRING)				\
	  }												\
	} while (0)

#define _DBG_DUMP_MEMORY_FORMAT(LEVEL,STRING,ADDR,LENGTH,DUMPFORMAT)	\
	do {											\
	  if (LEVEL & _DBG_LVL_MASK) {					\
		_DBG_PRINT_PROLOG(LEVEL,STRING)				\
		_DBG_PRINT_PROLOG(LEVEL,STRING)				\
		if (__DBG_LEVEL__ & LEVEL)					\
		{											\
			_DBG1_CALL(LEVEL,  (__MOD_NAME__));		\
			_DBG1_CALL(LEVEL,  STRING);				\
			DbgDump (LEVEL, ADDR, LENGTH, DUMPFORMAT);\
		}											\
		_DBG_PRINT_EPILOG(LEVEL,STRING)				\
	  }												\
	} while (0)
#define _DBG_INIT									(void)0
#define _DBG_BREAK									(void)0
#define _DBG_ENTER_FUNC(NAME)						(void)0
#define _DBG_ENTER_FUNC_ARGS(NAME, STRING)			(void)0
#define _DBG_ENTER_EXT(LEVEL,NAME,IRQL)				(void)0
#define _DBG_ENTER_EXT_ARGS(LEVEL,NAME,IRQL,STRING) (void)0
#define _DBG_ENTER_LVL(LEVEL,NAME)					(void)0
#define _DBG_ENTER_LVL_ARGS(LEVEL,NAME,STRING)		(void)0
#define	_DBG_LEAVE_FUNC()							(void)0
#define	_DBG_RETURN_FUNC(VALUE)						(void)0
#define	_DBG_FSTATUS_RETURN_FUNC(STATUS)			(void)0
#define _DBG_LEAVE_LVL(LEVEL)						(void)0
#define _DBG_RETURN_LVL(LEVEL, VALUE)				(void)0
#define _DBG_FSTATUS_RETURN_LVL(LEVEL, STATUS)		(void)0
#define _DBG_LEAVE_EXT(LEVEL)						(void)0
#define _DBG_RETURN_EXT(LEVEL, VALUE)  				(void)0
#define _DBG_FSTATUS_RETURN_EXT(LEVEL, STATUS)		(void)0

#define _DBG_INFO(STRING)											\
	_DBG_PRINT (_DBG_LVL_INFO, STRING)

#define _DBG_WARN(STRING)											\
	do {															\
		_DBG_PRINT_PROLOG(_DBG_LVL_WARN,STRING)						\
		if (__DBG_LEVEL__ & _DBG_LVL_WARN)							\
		{															\
			_DBG1_CALL(_DBG_LVL_WARN, ("%s !WARNING! ",				\
									 _DBG_PTR(__MOD_NAME__)));		\
			_DBG1_CALL(_DBG_LVL_WARN, STRING);						\
		}															\
		_DBG_PRINT_EPILOG(_DBG_LVL_WARN,STRING)						\
	} while (0)

#define _DBG_ERROR(STRING)											\
	do {															\
		_DBG_ERROR_PROLOG(_DBG_LVL_ERROR,STRING)					\
		if (__DBG_LEVEL__ & _DBG_LVL_ERROR)							\
		{															\
			_DBG1_CALL(_DBG_LVL_ERROR, ("%s !ERROR! ",				\
								 _DBG_PTR(__MOD_NAME__)));			\
			_DBG1_CALL(_DBG_LVL_ERROR, STRING);						\
		}															\
		_DBG_ERROR_EPILOG(_DBG_LVL_ERROR,STRING)					\
	} while (0)

#define _DBG_FATAL(STRING)											\
	do {															\
		_DBG_FATAL_PROLOG(_DBG_LVL_FATAL,STRING)					\
		if (__DBG_LEVEL__ & _DBG_LVL_FATAL)							\
		{															\
			_DBG1_CALL(_DBG_LVL_FATAL, ("%s !!!FATAL ERROR!!! ",	\
										 _DBG_PTR(__MOD_NAME__)));	\
			_DBG1_CALL(_DBG_LVL_FATAL, STRING);						\
		}															\
		_DBG_FATAL_EPILOG(_DBG_LVL_FATAL,STRING)					\
	} while(0)

#define	_DBG_MEM(STRING)				 (void)0

#endif	/* DBG || IB_DEBUG */

#endif	/* ICS_LOGGING */

#define _DBG_WARNING(STRING)	_DBG_WARN(STRING)

/* -------------------------------------------------------------------------
 * Tracing macros
 */
#ifdef IB_TRACE

#ifdef  ICS_LOGGING
#define _TRC_DBG_CALL(STRING) \
	_ICS_DBG2_DEBUG5  STRING
#define _TRC_REGISTER()					(void)0
#define _TRC_UNREGISTER()				(void)0

#else /* ICS_LOGGING */

/* for simplicity for now
 * trace calls from user space simply go to standard log (which could be stdout)
 */
#define _TRC_DBG_CALL(STRING) \
	do { \
		DbgOut("%s: ", __MOD_NAME__); \
		DbgOut  STRING; \
	} while (0)
#define _TRC_REGISTER()					(void)0
#define _TRC_UNREGISTER()				(void)0

#endif /* ICS_LOGGING */

#define _TRC_PRINT(LEVEL,STRING)							\
	do {													\
	  if (LEVEL) {											\
		if (__DBG_TRACE_LEVEL__ & LEVEL & _TRC_LVL_MASK)	\
		{													\
			_TRC_DBG_CALL(STRING);							\
		}													\
	  }														\
	} while (0)

#else /* IB_TRACE */
#define _TRC_REGISTER()					(void)0
#define _TRC_UNREGISTER()				(void)0
#define _TRC_PRINT(LEVEL,STRING)		(void)0
#endif /* IB_TRACE */

#endif /* _IBA_IB_DEBUG_H_ */
