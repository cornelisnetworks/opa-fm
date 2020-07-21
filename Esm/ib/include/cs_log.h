/* BEGIN_ICS_COPYRIGHT2 ****************************************

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

 * ** END_ICS_COPYRIGHT2   ****************************************/

/************************************************************************
 *
 * FILE NAME
 *    cs_log.h
 *
 * DESCRIPTION
 *    Logging and Tracing subsystem definition
 *
 * DATA STRUCTURES
 *
 * FUNCTIONS
 *   vs_log_control
 *
 * DEPENDENCIES
 *    ib_types.h
 *
 *
 * HISTORY
 *
 *    NAME      DATE  REMARKS
 *     paw  10/18/01  Initial creation of file.
 *     joc  10/26/01  Updates based on discussions and internal review
 *   golio   3/20/01  Added VIEO_VSNET_MOD_ID.
 *
 *
 ************************************************************************/

#ifndef __VIEO_VS_LOG__
#define __VIEO_VS_LOG__
#include <os_g.h>
#include <ib_types.h>
#include <ib_status.h>
#include <cs_csm_log.h>
#include <stdio.h>

#ifndef stringize
#define stringize(x) #x
#endif
#ifndef add_quotes
#define add_quotes(x) stringize(x)
#endif

#ifdef __VXWORKS__

#include <ESM_Messages.h>
#include <Log.h>
#include "bspcommon/h/sysPrintf.h"

#define CS64	"ll"
#define LogVal_t uint32_t
#define LogVal64_t uint64_t

#else

#define Log_StrDup(x)	(x)

#define sysPrintf printf

#include <bits/wordsize.h>

#if __WORDSIZE == 64
#define CS64	"l"
#define LogVal_t uint64_t
#define LogVal64_t uint64_t
#define LOG_PTR(x)		((uint64_t)(x))
#elif __WORDSIZE == 32
#define CS64	"L"
#define LogVal_t uint32_t
#define LogVal64_t uint64_t
#define LOG_PTR(x)		((uint32_t)(x))
#else
#error "__WORDSIZE not 64 nor 32"
#endif

#endif

#define CS64x	CS64"x"
#define CS64X	CS64"X"
#define CS64d	CS64"d"
#define CS64o	CS64"o"
#define CS64u	CS64"u"

/*
 * Convienience formatting macros for standardizing log messages
 */
#define FMT_U64 "0x%.16" CS64x

#define FMT_GID "0x%.16" CS64x ":%.16" CS64x

#define GIDPRINTARGS(GID) ntoh64(alignArray64(&GID[0])), ntoh64(alignArray64(&GID[8]))

static __inline__ uint64_t
alignArray64(uint8_t * array) {
	uint64_t rv;
	memcpy(&rv, array, sizeof(rv));
	return rv;
}

/************************************************************************
  Severity mask usage.

  Severity mask values are used to allow selection of logging information
  to be output to the error/trace log. 
  It is possible to add new log severity values as
  requirements are discovered in the future without impacting existing
  code.

 ************************************************************************/

/************************************************************************/
/** Define Severity values **********************************************/
/************************************************************************/
/** The VS_LOG_CSM_* constants below are only for internal use in      **/
/** smCsm functions.  DO NOT USE THEM DIRECTLY IN ANY OTHER MACROS/FUNCS*/
/************************************************************************/

#define VS_LOG_NONE 	 (0)		/* special for vs_syslog_output_message */
#define VS_LOG_FATAL     (0x00001)	/* Fatal error */
#define VS_LOG_CSM_ERROR (0x00002)	/* Actionable error */
#define VS_LOG_CSM_WARN  (0x00004)	/* Actionable warning */
#define VS_LOG_CSM_NOTICE (0x00008)	/* Actionable notice */
#define VS_LOG_CSM_INFO  (0x00010)	/* Actionable info */
#define VS_LOG_ERROR     (0x00020)  	/* Recoverable error */
#define VS_LOG_WARN      (0x00040)  /* Log warn information */
#define VS_LOG_NOTICE    (0x00080)  /* Notice level */
#define VS_LOG_INFINI_INFO (0x00100) /* Infini Info Debug information msgs */
#define VS_LOG_INFO      (0x00200)  /* Debug information Messages */
#define VS_LOG_VERBOSE   (0x00400)  /* Verbose information */
#define VS_LOG_DATA      (0x00800)  /* Binary data         */
#define VS_LOG_DEBUG1    (0x01000)  /* debug1 information */
#define VS_LOG_DEBUG2    (0x02000)  /* debug2 information */
#define VS_LOG_DEBUG3    (0x04000)  /* debug3 information */
#define VS_LOG_DEBUG4    (0x08000)  /* debug4 information */
#define VS_LOG_ENTER     (0x10000)	/* Function entry */
#define VS_LOG_ARGS      (0x20000)	/* function entry with more args */
#define VS_LOG_EXIT      (0x40000)	/* Function exit */

/* combination flags which can be used when initialize mask */
#define VS_LOG_OFF	    (0x00000000)	/* Disable all logging */
#define VS_LOG_TRACE	(VS_LOG_ARGS|VS_LOG_ENTER|VS_LOG_EXIT) /* Tracing ON .. enter/exit/args */
#define VS_LOG_ALL	    (0xFFFFFFFF)	/* Enable all debugging info */

#define NONDEBUG_LOG_MASK (VS_LOG_FATAL|VS_LOG_CSM_ERROR|VS_LOG_CSM_WARN \
			|VS_LOG_CSM_NOTICE|VS_LOG_CSM_INFO|VS_LOG_ERROR|VS_LOG_WARN \
			|VS_LOG_NOTICE|VS_LOG_INFINI_INFO)
#define DEFAULT_LOG_MASK NONDEBUG_LOG_MASK

/************************************************************************/
/** Define Module IDs ***************************************************/
/*  Module IDs are used to identify the module that generated the error.*/
/*  These values may be used in the output facility for filtering.      */
/************************************************************************/

#define VIEO_NONE_MOD_ID 		 0	// special for vs_syslog_output_message
									// LogMask for this module is not used
									// this must be modid 0
#define VIEO_CS_MOD_ID           1	/* Library Modules */
#define VIEO_MAI_MOD_ID          2
#define VIEO_CAL_MOD_ID          3
#define VIEO_DRIVER_MOD_ID       4

#define VIEO_IF3_MOD_ID          5

#define VIEO_SM_MOD_ID           6	/* Subnet Mgr */
#define VIEO_SA_MOD_ID           7	/* Subnet Administrator */

#define VIEO_PM_MOD_ID           8	/* Performance Mgr */
#define VIEO_PA_MOD_ID           9	/* Performance Administrator */
#define VIEO_BM_MOD_ID           10	/* Baseboard Mgr */

#define VIEO_FE_MOD_ID           11	/* Fabric Executive */

#define VIEO_APP_MOD_ID          12	/* Generic VIEO mod id */
									/* also used for all CSM messages */
#define VIEO_LAST_MOD_ID         13	/* last in list, used to size array */
// the present implementation is limited to 32 module ids (0-31)
// However that limitation only occurs in cs_log_set_mods_mask API
// and the internals of cs_log_translate_level

#define VS_MOD_ALL		        (0xffffffff)	/* all modules */

#define DEFAULT_LOG_MODS    (VS_MOD_ALL & ~(1<<VIEO_CS_MOD_ID))	/* all but CS */

/***************************************************************************
 * Set the default module ID. Local libriaries should redefine this
 * locally so that context is appropriately identified.
 *
 * #undef LOCAL_MOD_ID
 * #define LOCAL_MOD_ID         MY_MODULE_ID
 *
 ***************************************************************************/

#ifndef	LOCAL_MOD_ID
#define LOCAL_MOD_ID            VIEO_APP_MOD_ID
#endif

/************************************************************************
  Global variables for logging support.
 ************************************************************************/

extern uint32_t cs_log_masks[VIEO_LAST_MOD_ID+1];

/************************************************************************/
/** Set the logging severity mask  **************************************/
/************************************************************************/
#define IB_SET_LOG_MASK(x) \
		cs_log_set_mods_mask(DEFAULT_LOG_MODS, (x) | VS_LOG_FATAL, cs_log_masks)

/***********************************************************************
 * Log data type. Changing this type will allow us to move to 64 bits
 * later on without changing the logging subsystem.
 ***********************************************************************/

/** Define the default debug and trace macro set ***********************/
#ifdef __VXWORKS__
// sev is typically a constant, in which case compiler will optimize this
// directly to the return value
#define vs_log_translate_sev(sev) \
		( ((sev) == VS_LOG_FATAL)? LOG_SEV_FATAL: \
		  ((sev) == VS_LOG_CSM_ERROR)? LOG_SEV_ERROR: \
		  ((sev) == VS_LOG_CSM_WARN)? LOG_SEV_WARNING: \
		  ((sev) == VS_LOG_CSM_NOTICE)? LOG_SEV_NOTICE: \
		  ((sev) == VS_LOG_CSM_INFO)? LOG_SEV_PROGRAM_INFO: \
		  ((sev) == VS_LOG_ERROR)? LOG_SEV_ERROR: \
		  ((sev) == VS_LOG_WARN)? LOG_SEV_WARNING: \
		  ((sev) == VS_LOG_NOTICE)? LOG_SEV_NOTICE: \
		  ((sev) == VS_LOG_INFINI_INFO)? LOG_SEV_PROGRAM_INFO: \
		  ((sev) == VS_LOG_INFO)? LOG_SEV_DEBUG1: \
		  ((sev) == VS_LOG_VERBOSE)? LOG_SEV_DEBUG2: \
		  ((sev) == VS_LOG_DATA)? LOG_SEV_DEBUG3: \
		  ((sev) == VS_LOG_DEBUG1)? LOG_SEV_DEBUG4: \
		  ((sev) == VS_LOG_DEBUG2)? LOG_SEV_DEBUG4: \
		  ((sev) == VS_LOG_DEBUG3)? LOG_SEV_DEBUG5: \
		  ((sev) == VS_LOG_DEBUG4)? LOG_SEV_DEBUG5: \
		  ((sev) == VS_LOG_ENTER)? LOG_SEV_DEBUG5: \
		  ((sev) == VS_LOG_ARGS)? LOG_SEV_DEBUG5: \
		  ((sev) == VS_LOG_EXIT)? LOG_SEV_DEBUG5: \
		  		LOG_SEV_DEBUG1)

#define IB_LOG_IS_INTERESTED_MOD(sev, modid) \
		(Log_IsInterested(MOD_ESM, vs_log_translate_sev(sev)) \
		 && ((sev) & cs_log_masks[modid]))

#define IB_LOG_IS_INTERESTED(sev) IB_LOG_IS_INTERESTED_MOD(sev, LOCAL_MOD_ID)

// TBD supply __FUNCTION__ as 1st arg, especially for debug macros

// The IB_LOG_FMT* macros are for internal use in this header file only

// We can safely wash a Log_Arg_t from Log_StrDup() or a char* through LOG_PTR
// so we use that for p1.  In either case the actual value will be 32 bits
// because presently VxWorks has 32 bit pointers.
// p2 is numeric and could already be 64 bits, so use LOG_ARG for it.
#define IB_LOG_FMT_DATA(sev, fmt, p1, p2) \
		do { if (IB_LOG_IS_INTERESTED(sev) ) \
			vs_log_output_data(sev, LOCAL_MOD_ID, fmt, __func__, NULL, LOG_PTR(p1), LOG_ARG(p2)); \
	   	} while (0)
#define IB_LOG_MOD_FMT_DATA(sev, modid, fmt, p1, p2) \
		do { if (IB_LOG_IS_INTERESTED_MOD(sev, modid) ) \
			vs_log_output_data(sev, modid, fmt, __func__, NULL, LOG_PTR(p1), LOG_ARG(p2)); \
	   	} while (0)
#define IB_LOG_FMT0(sev, p1) \
	IB_LOG_FMT_DATA(sev, "%s%s%s%s%s", p1, 0)
#define IB_LOG_MOD_FMT0(sev, modid, p1) \
	IB_LOG_MOD_FMT_DATA(sev, modid, "%s%s%s%s%s", p1, 0)
#define IB_LOG_FMTU(sev, p1, p2) \
	IB_LOG_FMT_DATA(sev, "%s%s%s%s%s %u", p1, (LogVal_t)p2)
#define IB_LOG_MOD_FMTU(sev, modid, p1, p2) \
	IB_LOG_MOD_FMT_DATA(sev, modid, "%s%s%s%s%s %u", p1, (LogVal_t)p2)
#define IB_LOG_FMTX(sev, p1, p2) \
	IB_LOG_FMT_DATA(sev, "%s%s%s%s%s 0x%x", p1, (LogVal_t)p2)
#define IB_LOG_MOD_FMTX(sev, modid, p1, p2) \
	IB_LOG_MOD_FMT_DATA(sev, modid, "%s%s%s%s%s 0x%x", p1, (LogVal_t)p2)
#define IB_LOG_FMTLX(sev, p1, p2) \
	IB_LOG_FMT_DATA(sev, "%s%s%s%s%s 0x%"CS64x, p1, (LogVal64_t)p2)
#define IB_LOG_MOD_FMTLX(sev, modid, p1, p2) \
	IB_LOG_MOD_FMT_DATA(sev, modid, "%s%s%s%s%s 0x%"CS64x, p1, (LogVal64_t)p2)
#define IB_LOG_FMT64(sev, p1, p2) \
	IB_LOG_FMT_DATA(sev, "%s%s%s%s%s %"CS64u, p1, (LogVal64_t)p2)
#define IB_LOG_MOD_FMT64(sev, modid, p1, p2) \
	IB_LOG_MOD_FMT_DATA(sev, modid, "%s%s%s%s%s %"CS64u, p1, (LogVal64_t)p2)
#define IB_LOG_FMTSTR(sev, p1, p2) \
	IB_LOG_FMT_DATA(sev, "%s%s%s%s%s %s", p1, LOG_PTR(p2))
#define IB_LOG_MOD_FMTSTR(sev, modid, p1, p2) \
	IB_LOG_MOD_FMT_DATA(sev, modid, "%s%s%s%s%s %s", p1, LOG_PTR(p2))
#else /* __VXWORKS__ */

#define IB_LOG_IS_INTERESTED_MOD(sev, modid) ((sev) & cs_log_masks[modid])

#define IB_LOG_IS_INTERESTED(sev) IB_LOG_IS_INTERESTED_MOD(sev, LOCAL_MOD_ID)

// The IB_LOG_FMT* macros are for internal use in this header file only
// TBD supply __FUNCTION__ as 1st arg, especially for debug macros
#define IB_LOG_FMT(sev, fmt...) \
		do { if (IB_LOG_IS_INTERESTED(sev) ) \
			vs_log_output(sev, LOCAL_MOD_ID, __func__, NULL, ## fmt); \
	   	} while (0)
#define IB_LOG_MOD_FMT(sev, modid, fmt...) \
		do { if (IB_LOG_IS_INTERESTED_MOD(sev, modid) ) \
			vs_log_output(sev, modid, __func__, NULL, ## fmt); \
	   	} while (0)
#define IB_LOG_FMT0(sev, p1) \
	IB_LOG_FMT(sev, "%s", p1)
#define IB_LOG_MOD_FMT0(sev, modid, p1) \
	IB_LOG_MOD_FMT(sev, modid, "%s", p1)
#define IB_LOG_FMTU(sev, p1, p2) \
	IB_LOG_FMT(sev, "%s %"CS64u, p1, (LogVal64_t)p2)
#define IB_LOG_MOD_FMTU(sev, modid, p1, p2) \
	IB_LOG_MOD_FMT(sev, modid, "%s %"CS64u, p1, (LogVal64_t)p2)
#define IB_LOG_FMTX(sev,p1, p2) \
	IB_LOG_FMT(sev, "%s 0x%"CS64x, p1, (LogVal64_t)p2)
#define IB_LOG_MOD_FMTX(sev,modid,p1, p2) \
	IB_LOG_MOD_FMT(sev, modid, "%s 0x%"CS64x, p1, (LogVal64_t)p2)
#define IB_LOG_FMTLX(sev, p1, p2) \
	IB_LOG_FMT(sev, "%s 0x%"CS64x, p1, (LogVal64_t)p2)
#define IB_LOG_MOD_FMTLX(sev, modid, p1, p2) \
	IB_LOG_MOD_FMT(sev, modid, "%s 0x%"CS64x, p1, (LogVal64_t)p2)
#define IB_LOG_FMT64(sev, p1, p2) \
	IB_LOG_FMT(sev, "%s %"CS64u, p1, (LogVal64_t)p2)
#define IB_LOG_MOD_FMT64(sev, modid, p1, p2) \
	IB_LOG_MOD_FMT(sev, modid, "%s %"CS64u, p1, (LogVal64_t)p2)
#define IB_LOG_FMTSTR(sev, p1, p2) \
	IB_LOG_FMT(sev, "%s %s", p1, p2)
#define IB_LOG_MOD_FMTSTR(sev, modid, p1, p2) \
	IB_LOG_MOD_FMT(sev, modid, "%s %s", p1, p2)
#endif

#define IB_LOG_FMTRC(sev, p1, p2) IB_LOG_FMTSTR(sev, p1, cs_convert_status(p2))
#define IB_LOG_MOD_FMTRC(sev, modid, p1, p2) IB_LOG_MOD_FMTSTR(sev, modid, p1, cs_convert_status(p2))

// Its prefered to use the IB_LOG_x_FMT and IB_LOG_x_FMT_VF macros.
// arguments and format string can be variables.
// Log_StrDup will be performed internally as part of formatting the message
#define cs_log(sev, function, fmt...) \
		do { if (IB_LOG_IS_INTERESTED(sev) ) \
			vs_log_output(sev, LOCAL_MOD_ID, function, NULL, ## fmt); \
	   	} while (0)

#define cs_log_mod(sev, modid, function, fmt...) \
		do { if (IB_LOG_IS_INTERESTED_MOD(sev, modid) ) \
			vs_log_output(sev, modid, function, NULL, ## fmt); \
	   	} while (0)

#define cs_log_vf(sev, vf, function, fmt...) \
		do { if (IB_LOG_IS_INTERESTED(sev) ) \
			vs_log_output(sev, LOCAL_MOD_ID, function, vf, ## fmt); \
	   	} while (0)

#define cs_log_mod_vf(sev, modid, vf, function, fmt...) \
		do { if (IB_LOG_IS_INTERESTED_MOD(sev, modid) ) \
			vs_log_output(sev, modid, function, vf, ## fmt); \
	   	} while (0)


/*******************************
 * FATAL ERROR
 ******************************/
#ifdef __VXWORKS__
#include <stdlib.h>
#define IB_FATAL_ERROR_NODUMP(p1) do {\
	smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_SM_SHUTDOWN, NULL, NULL, p1); \
	ESM_LOG_FINAL_ESMFATAL(p1); \
} while (0)
#define IB_FATAL_ERROR(p1) do {\
	IB_FATAL_ERROR_NODUMP(p1);  \
    abort(); \
} while (0)
#else
#define IB_FATAL_ERROR_NODUMP(p1) do {						\
	smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_SM_SHUTDOWN, NULL, NULL, p1); \
	vs_log_output(VS_LOG_FATAL, LOCAL_MOD_ID, __func__, NULL, "%s", p1); \
    exit(2); \
} while (0)
#define IB_FATAL_ERROR(p1) do {						\
	smCsmLogMessage(CSM_SEV_NOTICE, CSM_COND_SM_SHUTDOWN, NULL, NULL, p1); \
	vs_log_output(VS_LOG_FATAL, LOCAL_MOD_ID, __func__, NULL, "%s", p1); \
    vs_fatal_error((uint8_t *)(p1)); \
} while (0)
#endif /* __VXWORKS__ */

/*******************************
 * RECOVERABLE ERROR
 ******************************/

#ifdef __VXWORKS__
// TBD - test LOG_IS_INTERESTED to pick up Mask & Filter
// TBD output module name in VxWorks messages
#define IB_LOG_ERROR0(p1)				ESM_LOG_FINAL_ESMERROR0(p1)
#define IB_LOG_MOD_ERROR0(modid, p1)	ESM_LOG_FINAL_ESMERROR0(p1)
#define IB_LOG_ERROR(p1,p2)				ESM_LOG_FINAL_ESMERROR(p1, p2)
#define IB_LOG_MOD_ERROR(modid, p1,p2)	ESM_LOG_FINAL_ESMERROR(p1, p2)
#define IB_LOG_ERRORX(p1,p2)			ESM_LOG_FINAL_ESMERRORX(p1, p2)
#define IB_LOG_MOD_ERRORX(modid, p1,p2)	ESM_LOG_FINAL_ESMERRORX(p1, p2)
#define IB_LOG_ERRORLX(p1,p2)			ESM_LOG_FINAL_ESMERRORLX(p1, p2)
#define IB_LOG_MOD_ERRORLX(modid, p1,p2)	ESM_LOG_FINAL_ESMERRORLX(p1, p2)
#define IB_LOG_ERROR64(p1,p2)			ESM_LOG_FINAL_ESMERROR64(p1, p2)
#define IB_LOG_MOD_ERROR64(modid, p1,p2)	ESM_LOG_FINAL_ESMERROR64(p1, p2)
#define IB_LOG_ERRORSTR(p1,p2)			ESM_LOG_FINAL_ESMERRORSTR(p1, p2)
#define IB_LOG_MOD_ERRORSTR(modid, p1,p2)	ESM_LOG_FINAL_ESMERRORSTR(p1, p2)
#define IB_LOG_ERRORRC(p1,p2)			ESM_LOG_FINAL_ESMERRORSTR(p1, cs_convert_status(p2))
#define IB_LOG_MOD_ERRORRC(modid, p1,p2)	ESM_LOG_FINAL_ESMERRORSTR(p1, cs_convert_status(p2))
#else
#define IB_LOG_ERROR0(p1)	 				IB_LOG_FMT0(VS_LOG_ERROR, p1)
#define IB_LOG_MOD_ERROR0(modid, p1)		IB_LOG_MOD_FMT0(VS_LOG_ERROR, modid, p1)
#define IB_LOG_ERROR(p1, p2)				IB_LOG_FMTU(VS_LOG_ERROR, p1, p2)
#define IB_LOG_MOD_ERROR(modid, p1, p2)		IB_LOG_MOD_FMTU(VS_LOG_ERROR, modid, p1, p2)
#define IB_LOG_ERRORX(p1, p2)				IB_LOG_FMTX(VS_LOG_ERROR, p1, p2)
#define IB_LOG_MOD_ERRORX(modid, p1, p2)	IB_LOG_MOD_FMTX(VS_LOG_ERROR, modid, p1, p2)
#define IB_LOG_ERRORLX(p1, p2)				IB_LOG_FMTLX(VS_LOG_ERROR, p1, p2)
#define IB_LOG_MOD_ERRORLX(modid, p1, p2)	IB_LOG_MOD_FMTLX(VS_LOG_ERROR, modid, p1, p2)
#define IB_LOG_ERROR64(p1, p2)				IB_LOG_FMT64(VS_LOG_ERROR, p1, p2)
#define IB_LOG_MOD_ERROR64(modid, p1, p2)	IB_LOG_MOD_FMT64(VS_LOG_ERROR, modid, p1, p2)
#define IB_LOG_ERRORSTR(p1, p2)				IB_LOG_FMTSTR(VS_LOG_ERROR, p1, p2)
#define IB_LOG_MOD_ERRORSTR(modid, p1, p2)	IB_LOG_MOD_FMTSTR(VS_LOG_ERROR, modid, p1, p2)
#define IB_LOG_ERRORRC(p1, p2)				IB_LOG_FMTRC(VS_LOG_ERROR, p1, p2)
#define IB_LOG_MOD_ERRORRC(modid, p1, p2)	IB_LOG_MOD_FMTRC(VS_LOG_ERROR, modid, p1, p2)
#endif /* __VXWORKS__ */
// for cs_log* and hence these macros
// Log_StrDup will be performed internally as part of formatting the message
#define IB_LOG_ERROR_FMT(func, fmt...) \
						cs_log(VS_LOG_ERROR, func, ## fmt)
#define IB_LOG_MOD_ERROR_FMT(modid, func, fmt...) \
						cs_log_mod(VS_LOG_ERROR, modid, func, ## fmt)
#define IB_LOG_ERROR_FMT_VF(vf, func, fmt...) \
						cs_log_vf(VS_LOG_ERROR, vf, func, ## fmt)
#define IB_LOG_MOD_ERROR_FMT_VF(modid, vf, func, fmt...) \
						cs_log_mod_vf(VS_LOG_ERROR, modid, vf, func, ## fmt)

/*******************************
 * WARNING
 ******************************/

#ifdef __VXWORKS__
// TBD - test LOG_IS_INTERESTED to pick up Mask & Filter
// TBD output module name in VxWorks messages
#define IB_LOG_WARN0(p1)					ESM_LOG_FINAL_ESMWARNING0(p1)
#define IB_LOG_MOD_WARN0(modid, p1)			ESM_LOG_FINAL_ESMWARNING0(p1)
#define IB_LOG_WARN(p1, p2)					ESM_LOG_FINAL_ESMWARNING(p1, p2)
#define IB_LOG_MOD_WARN(modid, p1, p2)		ESM_LOG_FINAL_ESMWARNING(p1, p2)
#define IB_LOG_WARNX(p1, p2)				ESM_LOG_FINAL_ESMWARNINGX(p1, p2)
#define IB_LOG_MOD_WARNX(modid, p1, p2)		ESM_LOG_FINAL_ESMWARNINGX(p1, p2)
#define IB_LOG_WARNLX(p1, p2)				ESM_LOG_FINAL_ESMWARNINGLX(p1, p2)
#define IB_LOG_MOD_WARNLX(modid, p1, p2)	ESM_LOG_FINAL_ESMWARNINGLX(p1, p2)
#define IB_LOG_WARN64(p1, p2)				ESM_LOG_FINAL_ESMWARNING64(p1, p2)
#define IB_LOG_MOD_WARN64(modid, p1, p2)	ESM_LOG_FINAL_ESMWARNING64(p1, p2)
#define IB_LOG_WARNSTR(p1, p2)				ESM_LOG_FINAL_ESMWARNINGSTR(p1, p2)
#define IB_LOG_MOD_WARNSTR(modid, p1, p2)	ESM_LOG_FINAL_ESMWARNINGSTR(p1, p2)
#define IB_LOG_WARNRC(p1, p2)				ESM_LOG_FINAL_ESMWARNINGSTR(p1, cs_convert_status(p2))
#define IB_LOG_MOD_WARNRC(p1, p2)			ESM_LOG_FINAL_ESMWARNINGSTR(p1, cs_convert_status(p2))
#else
#define IB_LOG_WARN0(p1)	 				IB_LOG_FMT0(VS_LOG_WARN, p1)
#define IB_LOG_MOD_WARN0(modid, p1)	 		IB_LOG_MOD_FMT0(VS_LOG_WARN, modid, p1)
#define IB_LOG_WARN(p1, p2)					IB_LOG_FMTU(VS_LOG_WARN, p1, p2)
#define IB_LOG_MOD_WARN(modid, p1, p2)		IB_LOG_MOD_FMTU(VS_LOG_WARN, modid, p1, p2)
#define IB_LOG_WARNX(p1, p2)				IB_LOG_FMTX(VS_LOG_WARN, p1, p2)
#define IB_LOG_MOD_WARNX(modid, p1, p2)		IB_LOG_MOD_FMTX(VS_LOG_WARN, modid, p1, p2)
#define IB_LOG_WARNLX(p1, p2)				IB_LOG_FMTLX(VS_LOG_WARN, p1, p2)
#define IB_LOG_MOD_WARNLX(modid, p1, p2)	IB_LOG_MOD_FMTLX(VS_LOG_WARN, modid, p1, p2)
#define IB_LOG_WARN64(p1, p2)				IB_LOG_FMT64(VS_LOG_WARN, p1, p2)
#define IB_LOG_MOD_WARN64(modid, p1, p2)	IB_LOG_MOD_FMT64(VS_LOG_WARN, modid, p1, p2)
#define IB_LOG_WARNSTR(p1, p2)				IB_LOG_FMTSTR(VS_LOG_WARN, p1, p2)
#define IB_LOG_MOD_WARNSTR(modid, p1, p2)	IB_LOG_MOD_FMTSTR(VS_LOG_WARN, modid, p1, p2)
#define IB_LOG_WARNRC(p1, p2)				IB_LOG_FMTRC(VS_LOG_WARN, p1, p2)
#define IB_LOG_MOD_WARNRC(modid, p1, p2)	IB_LOG_MOD_FMTRC(VS_LOG_WARN, modid, p1, p2)
#endif /* __VXWORKS__ */
// for cs_log* and hence these macros
// Log_StrDup will be performed internally as part of formatting the message
#define IB_LOG_WARN_FMT(func, fmt...) \
							cs_log(VS_LOG_WARN, func, ## fmt)
#define IB_LOG_MOD_WARN_FMT(modid, func, fmt...) \
							cs_log_mod(VS_LOG_WARN, modid, func, ## fmt)
#define IB_LOG_WARN_FMT_VF(vf, func, fmt...) \
							cs_log_vf(VS_LOG_WARN, vf, func, ## fmt)
#define IB_LOG_MOD_WARN_FMT_VF(modid, vf, func, fmt...) \
							cs_log_mod_vf(VS_LOG_WARN, modid, vf, func, ## fmt)

/*******************************
 * NOTICE
 ******************************/

#ifdef __VXWORKS__
// TBD - test LOG_IS_INTERESTED to pick up Mask & Filter
// TBD output module name in VxWorks messages
#define IB_LOG_NOTICE0(p1)					ESM_LOG_FINAL_ESMNOTICE0(p1)
#define IB_LOG_MOD_NOTICE0(modid, p1)		ESM_LOG_FINAL_ESMNOTICE0(p1)
#define IB_LOG_NOTICE(p1, p2)				ESM_LOG_FINAL_ESMNOTICE(p1, p2)
#define IB_LOG_MOD_NOTICE(modid, p1, p2)	ESM_LOG_FINAL_ESMNOTICE(p1, p2)
#define IB_LOG_NOTICEX(p1, p2)				ESM_LOG_FINAL_ESMNOTICEX(p1, p2)
#define IB_LOG_MOD_NOTICEX(modid, p1, p2)	ESM_LOG_FINAL_ESMNOTICEX(p1, p2)
#define IB_LOG_NOTICELX(p1, p2)				ESM_LOG_FINAL_ESMNOTICELX(p1, p2)
#define IB_LOG_MOD_NOTICELX(modid, p1, p2)	ESM_LOG_FINAL_ESMNOTICELX(p1, p2)
#define IB_LOG_NOTICE64(p1, p2)				ESM_LOG_FINAL_ESMNOTICE64(p1, p2)
#define IB_LOG_MOD_NOTICE64(modid, p1, p2)	ESM_LOG_FINAL_ESMNOTICE64(p1, p2)
#define IB_LOG_NOTICESTR(p1, p2)			ESM_LOG_FINAL_ESMNOTICESTR(p1, p2)
#define IB_LOG_MOD_NOTICESTR(modid, p1, p2)	ESM_LOG_FINAL_ESMNOTICESTR(p1, p2)
#define IB_LOG_NOTICERC(p1, p2)				ESM_LOG_FINAL_ESMNOTICESTR(p1, cs_convert_status(p2))
#define IB_LOG_MOD_NOTICERC(modid, p1, p2)	ESM_LOG_FINAL_ESMNOTICESTR(p1, cs_convert_status(p2))
#else
#define IB_LOG_NOTICE0(p1)	 				IB_LOG_FMT0(VS_LOG_NOTICE, p1)
#define IB_LOG_MOD_NOTICE0(modid, p1)	 	IB_LOG_MOD_FMT0(VS_LOG_NOTICE, modid, p1)
#define IB_LOG_NOTICE(p1, p2)				IB_LOG_FMTU(VS_LOG_NOTICE, p1, p2)
#define IB_LOG_MOD_NOTICE(modid, p1, p2)	IB_LOG_MOD_FMTU(VS_LOG_NOTICE, modid, p1, p2)
#define IB_LOG_NOTICEX(p1, p2)				IB_LOG_FMTX(VS_LOG_NOTICE, p1, p2)
#define IB_LOG_MOD_NOTICEX(modid, p1, p2)	IB_LOG_MOD_FMTX(VS_LOG_NOTICE, modid, p1, p2)
#define IB_LOG_NOTICELX(p1, p2)				IB_LOG_FMTLX(VS_LOG_NOTICE, p1, p2)
#define IB_LOG_MOD_NOTICELX(modid, p1, p2)	IB_LOG_MOD_FMTLX(VS_LOG_NOTICE, modid, p1, p2)
#define IB_LOG_NOTICE64(p1, p2)				IB_LOG_FMT64(VS_LOG_NOTICE, p1, p2)
#define IB_LOG_MOD_NOTICE64(modid, p1, p2)	IB_LOG_MOD_FMT64(VS_LOG_NOTICE, modid, p1, p2)
#define IB_LOG_NOTICESTR(p1, p2)			IB_LOG_FMTSTR(VS_LOG_NOTICE, p1, p2)
#define IB_LOG_MOD_NOTICESTR(modid, p1, p2)	IB_LOG_MOD_FMTSTR(VS_LOG_NOTICE, modid, p1, p2)
#define IB_LOG_NOTICERC(p1, p2)				IB_LOG_FMTRC(VS_LOG_NOTICE, p1, p2)
#define IB_LOG_MOD_NOTICERC(modid, p1, p2)	IB_LOG_MOD_FMTRC(VS_LOG_NOTICE, modid, p1, p2)
#endif /* __VXWORKS__ */
// for cs_log* and hence these macros
// Log_StrDup will be performed internally as part of formatting the message
#define IB_LOG_NOTICE_FMT(func, fmt...) \
						cs_log(VS_LOG_NOTICE, func, ## fmt)
#define IB_LOG_MOD_NOTICE_FMT(modid, func, fmt...) \
						cs_log_mod(VS_LOG_NOTICE, modid, func, ## fmt)
#define IB_LOG_NOTICE_FMT_VF(vf, func, fmt...) \
						cs_log_vf(VS_LOG_NOTICE, vf, func, ## fmt)
#define IB_LOG_MOD_NOTICE_FMT_VF(modid, vf, func, fmt...) \
						cs_log_mod_vf(VS_LOG_NOTICE, modid, vf, func, ## fmt)

/*******************************
 * INFO    INFINI_INFO
 ******************************/

#ifdef __VXWORKS__
// TBD - test LOG_IS_INTERESTED to pick up Mask & Filter
// TBD output module name in VxWorks messages

// we put it in our logging system's Program_Info
#define IB_LOG_INFINI_INFO0(p1)					ESM_LOG_ESMINFO0(p1)
#define IB_LOG_MOD_INFINI_INFO0(modid, p1)		ESM_LOG_ESMINFO0(p1)
#define IB_LOG_INFINI_INFO(p1, p2)				ESM_LOG_ESMINFO(p1, p2)
#define IB_LOG_MOD_INFINI_INFO(modid, p1, p2)	ESM_LOG_ESMINFO(p1, p2)
#define IB_LOG_INFINI_INFOX(p1, p2)				ESM_LOG_ESMINFOX(p1, p2)
#define IB_LOG_MOD_INFINI_INFOX(modid, p1, p2)	ESM_LOG_ESMINFOX(p1, p2)
#define IB_LOG_INFINI_INFOLX(p1, p2)			ESM_LOG_ESMINFOLX(p1, p2)
#define IB_LOG_MOD_INFINI_INFOLX(modid, p1, p2)	ESM_LOG_ESMINFOLX(p1, p2)
#define IB_LOG_INFINI_INFO64(p1, p2)			ESM_LOG_ESMINFO64(p1, p2)
#define IB_LOG_MOD_INFINI_INFO64(modid, p1, p2)	ESM_LOG_ESMINFO64(p1, p2)
#define IB_LOG_INFINI_INFOSTR(p1, p2)			ESM_LOG_ESMINFOSTR(p1, p2)
#define IB_LOG_MOD_INFINI_INFOSTR(modid, p1, p2)	ESM_LOG_ESMINFOSTR(p1, p2)
#define IB_LOG_INFINI_INFORC(p1, p2)			ESM_LOG_ESMINFOSTR(p1, cs_convert_status(p2))
#define IB_LOG_MOD_INFINI_INFORC(modid, p1, p2)	ESM_LOG_ESMINFOSTR(p1, cs_convert_status(p2))
#else
#define IB_LOG_INFINI_INFO0(p1)	 				IB_LOG_FMT0(VS_LOG_INFINI_INFO, p1)
#define IB_LOG_MOD_INFINI_INFO0(modid, p1)	 	IB_LOG_MOD_FMT0(VS_LOG_INFINI_INFO, modid, p1)
#define IB_LOG_INFINI_INFO(p1, p2)				IB_LOG_FMTU(VS_LOG_INFINI_INFO, p1, p2)
#define IB_LOG_MOD_INFINI_INFO(modid, p1, p2)	IB_LOG_MOD_FMTU(VS_LOG_INFINI_INFO, modid, p1, p2)
#define IB_LOG_INFINI_INFOX(p1, p2)				IB_LOG_FMTX(VS_LOG_INFINI_INFO, p1, p2)
#define IB_LOG_MOD_INFINI_INFOX(modid, p1, p2)	IB_LOG_MOD_FMTX(VS_LOG_INFINI_INFO, modid, p1, p2)
#define IB_LOG_INFINI_INFOLX(p1, p2) 			IB_LOG_FMTLX(VS_LOG_INFINI_INFO, p1, p2)
#define IB_LOG_MOD_INFINI_INFOLX(modid, p1, p2) IB_LOG_MOD_FMTLX(VS_LOG_INFINI_INFO, modid, p1, p2)
#define IB_LOG_INFINI_INFO64(p1, p2) 			IB_LOG_FMT64(VS_LOG_INFINI_INFO, p1, p2)
#define IB_LOG_MOD_INFINI_INFO64(modid, p1, p2) IB_LOG_MOD_FMT64(VS_LOG_INFINI_INFO, modid, p1, p2)
#define IB_LOG_INFINI_INFOSTR(p1, p2) 			IB_LOG_FMTSTR(VS_LOG_INFINI_INFO, p1, p2)
#define IB_LOG_MOD_INFINI_INFOSTR(modid, p1, p2) IB_LOG_MOD_FMTSTR(VS_LOG_INFINI_INFO, modid, p1, p2)
#define IB_LOG_INFINI_INFORC(p1, p2) 			IB_LOG_FMTRC(VS_LOG_INFINI_INFO, p1, p2)
#define IB_LOG_MOD_INFINI_INFORC(modid, p1, p2) IB_LOG_MOD_FMTRC(VS_LOG_INFINI_INFO, modid, p1, p2)
#endif /* __VXWORKS__ */
// for cs_log* and hence these macros
// Log_StrDup will be performed internally as part of formatting the message
#define IB_LOG_INFINI_INFO_FMT(func, fmt...) \
					cs_log(VS_LOG_INFINI_INFO, func, ## fmt)
#define IB_LOG_MOD_INFINI_INFO_FMT(modid, func, fmt...) \
					cs_log_mod(VS_LOG_INFINI_INFO, modid, func, ## fmt)
#define IB_LOG_INFINI_INFO_FMT_VF(vf, func, fmt...) \
					cs_log_vf(VS_LOG_INFINI_INFO, vf, func, ## fmt)
#define IB_LOG_MOD_INFINI_INFO_FMT_VF(modid, vf, func, fmt...) \
					cs_log_mod_vf(VS_LOG_INFINI_INFO, modid, vf, func, ## fmt)

#ifdef VIEO_TRACE_DISABLED
#define IB_LOG_INFO0(p1)	 				do { } while(0)
#define IB_LOG_MOD_INFO0(modid, p1)	 		do { } while(0)
#define IB_LOG_INFO(p1, p2)					do { } while(0)
#define IB_LOG_MOD_INFO(modid, p1, p2)		do { } while(0)
#define IB_LOG_INFOX(p1, p2)				do { } while(0)
#define IB_LOG_MOD_INFOX(modid, p1, p2)		do { } while(0)
#define IB_LOG_INFOLX(p1, p2)				do { } while(0)
#define IB_LOG_MOD_INFOLX(modid, p1, p2)	do { } while(0)
#define IB_LOG_INFO64(p1, p2)				do { } while(0)
#define IB_LOG_MOD_INFO64(modid, p1, p2)	do { } while(0)
#define IB_LOG_INFOSTR(p1, p2)				do { } while(0)
#define IB_LOG_MOD_INFOSTR(modid, p1, p2)	do { } while(0)
#define IB_LOG_INFORC(p1, p2)				do { } while(0)
#define IB_LOG_MOD_INFORC(modid, p1, p2)	do { } while(0)
#define IB_LOG_INFO_FMT(func, fmt...) 		do { } while(0)
#define IB_LOG_MOD_INFO_FMT(modid, func, fmt...) do { } while(0)
#define IB_LOG_INFO_FMT_VF(vf, func, fmt...) do { } while(0)
#define IB_LOG_MOD_INFO_FMT_VF(modid, vf, func, fmt...) do { } while(0)
#else
#define IB_LOG_INFO0(p1)	 				IB_LOG_FMT0(VS_LOG_INFO, p1)
#define IB_LOG_MOD_INFO0(modid, p1)	 		IB_LOG_MOD_FMT0(VS_LOG_INFO, modid, p1)
#define IB_LOG_INFO(p1, p2)					IB_LOG_FMTU(VS_LOG_INFO, p1, p2)
#define IB_LOG_MOD_INFO(modid, p1, p2)		IB_LOG_MOD_FMTU(VS_LOG_INFO, modid, p1, p2)
#define IB_LOG_INFOX(p1, p2)				IB_LOG_FMTX(VS_LOG_INFO, p1, p2)
#define IB_LOG_MOD_INFOX(modid, p1, p2)		IB_LOG_MOD_FMTX(VS_LOG_INFO, modid, p1, p2)
#define IB_LOG_INFOLX(p1, p2) 				IB_LOG_FMTLX(VS_LOG_INFO, p1, p2)
#define IB_LOG_MOD_INFOLX(modid, p1, p2) 	IB_LOG_MOD_FMTLX(VS_LOG_INFO, modid, p1, p2)
#define IB_LOG_INFO64(p1, p2) 				IB_LOG_FMT64(VS_LOG_INFO, p1, p2)
#define IB_LOG_MOD_INFO64(modid, p1, p2) 	IB_LOG_MOD_FMT64(VS_LOG_INFO, modid, p1, p2)
#define IB_LOG_INFOSTR(p1, p2) 				IB_LOG_FMTSTR(VS_LOG_INFO, p1, p2)
#define IB_LOG_MOD_INFOSTR(modid, p1, p2) 	IB_LOG_MOD_FMTSTR(VS_LOG_INFO, modid, p1, p2)
#define IB_LOG_INFORC(p1, p2) 				IB_LOG_FMTRC(VS_LOG_INFO, p1, p2)
#define IB_LOG_MOD_INFORC(modid, p1, p2) 	IB_LOG_MOD_FMTRC(VS_LOG_INFO, modid, p1, p2)
// for cs_log* and hence these macros
// Log_StrDup will be performed internally as part of formatting the message
#define IB_LOG_INFO_FMT(func, fmt...) \
						cs_log(VS_LOG_INFO, func, ## fmt)
#define IB_LOG_MOD_INFO_FMT(modid, func, fmt...) \
						cs_log_mod(VS_LOG_INFO, modid, func, ## fmt)
#define IB_LOG_INFO_FMT_VF(vf, func, fmt...) \
						cs_log_vf(VS_LOG_INFO, vf, func, ## fmt)
#define IB_LOG_MOD_INFO_FMT_VF(modid, vf, func, fmt...) \
						cs_log_mod_vf(VS_LOG_INFO, modid, vf, func, ## fmt)
#endif

/*******************************
 * DATA
 ******************************/

#ifdef VIEO_TRACE_DISABLED
#define IB_LOG_DATA(p1, addr, len) 				do {} while(0)
#define IB_LOG_MOD_DATA(modid, p1, addr, len) 	do {} while(0)
#else
#ifdef __VXWORKS__
// TBD output module name in VxWorks messages
#define IB_LOG_DATA(p1, addr, len) 	\
		do { if (IB_LOG_IS_INTERESTED(VS_LOG_DATA) && ! vs_log_filter() ) \
			LOG_DEBUG2(MOD_ESM, "%s: %F", LOG_PTR(p1), Log_MemoryDump(addr, len),0,0,0,0); \
		} while (0)
#define IB_LOG_MOD_DATA(modid, p1, addr, len) 	\
		do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_DATA, modid) && ! vs_log_filter() ) \
			LOG_DEBUG2(MOD_ESM, "%s: %F", LOG_PTR(p1), Log_MemoryDump(addr, len),0,0,0,0); \
		} while (0)
#else
#define IB_LOG_DATA(p1, addr, len) 	\
		do { if (IB_LOG_IS_INTERESTED(VS_LOG_DATA) ) \
			vs_log_output_memory(VS_LOG_DATA, LOCAL_MOD_ID, __func__, NULL, p1, addr, len); \
		} while (0)
#define IB_LOG_MOD_DATA(modid, p1, addr, len) 	\
		do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_DATA, modid) ) \
			vs_log_output_memory(VS_LOG_DATA, modid, __func__, NULL, p1, addr, len); \
		} while (0)
#endif
#endif

/*******************************
 * VERBOSE
 ******************************/

#ifdef VIEO_TRACE_DISABLED
#define IB_LOG_VERBOSE0(p1)	 					do { } while(0)
#define IB_LOG_MOD_VERBOSE0(modid, p1)	 		do { } while(0)
#define IB_LOG_VERBOSE(p1, p2)					do { } while(0)
#define IB_LOG_MOD_VERBOSE(modid, p1, p2)		do { } while(0)
#define IB_LOG_VERBOSEX(p1, p2)					do { } while(0)
#define IB_LOG_MOD_VERBOSEX(modid, p1, p2)		do { } while(0)
#define IB_LOG_VERBOSELX(p1, p2)				do { } while(0)
#define IB_LOG_MOD_VERBOSELX(modid, p1, p2)		do { } while(0)
#define IB_LOG_VERBOSE64(p1, p2)				do { } while(0)
#define IB_LOG_MOD_VERBOSE64(modid, p1, p2)		do { } while(0)
#define IB_LOG_VERBOSESTR(p1, p2)				do { } while(0)
#define IB_LOG_MOD_VERBOSESTR(modid, p1, p2)	do { } while(0)
#define IB_LOG_VERBOSERC(p1, p2)				do { } while(0)
#define IB_LOG_MOD_VERBOSERC(modid, p1, p2)		do { } while(0)
#define IB_LOG_VERBOSE_FMT(func, fmt...) 		do { } while(0)
#define IB_LOG_MOD_VERBOSE_FMT(modid, func, fmt...) do { } while(0)
#define IB_LOG_VERBOSE_FMT_VF(vf, func, fmt...) do { } while(0)
#define IB_LOG_MOD_VERBOSE_FMT_VF(modid, vf, func, fmt...) do { } while(0)
#else
#define IB_LOG_VERBOSE0(p1)	 					IB_LOG_FMT0(VS_LOG_VERBOSE, p1)
#define IB_LOG_MOD_VERBOSE0(modid, p1)	 		IB_LOG_MOD_FMT0(VS_LOG_VERBOSE, modid, p1)
#define IB_LOG_VERBOSE(p1, p2)					IB_LOG_FMTU(VS_LOG_VERBOSE, p1, p2)
#define IB_LOG_MOD_VERBOSE(modid, p1, p2)		IB_LOG_MOD_FMTU(VS_LOG_VERBOSE, modid, p1, p2)
#define IB_LOG_VERBOSEX(p1, p2)					IB_LOG_FMTX(VS_LOG_VERBOSE, p1, p2)
#define IB_LOG_MOD_VERBOSEX(modid, p1, p2)		IB_LOG_MOD_FMTX(VS_LOG_VERBOSE, modid, p1, p2)
#define IB_LOG_VERBOSELX(p1, p2) 				IB_LOG_FMTLX(VS_LOG_VERBOSE, p1, p2)
#define IB_LOG_MOD_VERBOSELX(modid, p1, p2) 	IB_LOG_MOD_FMTLX(VS_LOG_VERBOSE, modid, p1, p2)
#define IB_LOG_VERBOSE64(p1, p2) 				IB_LOG_FMT64(VS_LOG_VERBOSE, p1, p2)
#define IB_LOG_MOD_VERBOSE64(modid, p1, p2) 	IB_LOG_MOD_FMT64(VS_LOG_VERBOSE, modid, p1, p2)
#define IB_LOG_VERBOSESTR(p1, p2) 				IB_LOG_FMTSTR(VS_LOG_VERBOSE, p1, p2)
#define IB_LOG_MOD_VERBOSESTR(modid, p1, p2) 	IB_LOG_MOD_FMTSTR(VS_LOG_VERBOSE, modid, p1, p2)
#define IB_LOG_VERBOSERC(p1, p2) 				IB_LOG_FMTRC(VS_LOG_VERBOSE, p1, p2)
#define IB_LOG_MOD_VERBOSERC(modid, p1, p2) 	IB_LOG_MOD_FMTRC(VS_LOG_VERBOSE, modid, p1, p2)
// for cs_log* and hence these macros
// Log_StrDup will be performed internally as part of formatting the message
#define IB_LOG_VERBOSE_FMT(func, fmt...) \
						cs_log(VS_LOG_VERBOSE, func, ## fmt)
#define IB_LOG_MOD_VERBOSE_FMT(modid, func, fmt...) \
						cs_log_mod(VS_LOG_VERBOSE, modid, func, ## fmt)
#define IB_LOG_VERBOSE_FMT_VF(vf, func, fmt...) \
						cs_log_vf(VS_LOG_VERBOSE, vf, func, ## fmt)
#define IB_LOG_MOD_VERBOSE_FMT_VF(modid, vf, func, fmt...) \
						cs_log_mod_vf(VS_LOG_VERBOSE, modid, vf, func, ## fmt)
#endif

/*******************************
 * DEBUG1
 ******************************/

#ifdef VIEO_TRACE_DISABLED
#define IB_LOG_DEBUG1_0(p1)	 					do { } while(0)
#define IB_LOG_MOD_DEBUG1_0(modid, p1)	 		do { } while(0)
#define IB_LOG_DEBUG1(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG1(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG1X(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG1X(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG1LX(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG1LX(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG1_64(p1, p2)				do { } while(0)
#define IB_LOG_MOD_DEBUG1_64(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG1STR(p1, p2)				do { } while(0)
#define IB_LOG_MOD_DEBUG1STR(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG1RC(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG1RC(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG1_FMT(func, fmt...) 		do { } while(0)
#define IB_LOG_MOD_DEBUG1_FMT(modid, func, fmt...) do { } while(0)
#define IB_LOG_DEBUG1_FMT_VF(vf, func, fmt...) 	do { } while(0)
#define IB_LOG_MOD_DEBUG1_FMT_VF(modid, vf, func, fmt...) do { } while(0)
#else
#define IB_LOG_DEBUG1_0(p1)	 					IB_LOG_FMT0(VS_LOG_DEBUG1, p1)
#define IB_LOG_MOD_DEBUG1_0(modid, p1)	 		IB_LOG_MOD_FMT0(VS_LOG_DEBUG1, modid, p1)
#define IB_LOG_DEBUG1(p1, p2)					IB_LOG_FMTU(VS_LOG_DEBUG1, p1, p2)
#define IB_LOG_MOD_DEBUG1(modid, p1, p2)		IB_LOG_MOD_FMTU(VS_LOG_DEBUG1, modid, p1, p2)
#define IB_LOG_DEBUG1X(p1, p2)					IB_LOG_FMTX(VS_LOG_DEBUG1, p1, p2)
#define IB_LOG_MOD_DEBUG1X(modid, p1, p2)		IB_LOG_MOD_FMTX(VS_LOG_DEBUG1, modid, p1, p2)
#define IB_LOG_DEBUG1LX(p1, p2) 				IB_LOG_FMTLX(VS_LOG_DEBUG1, p1, p2)
#define IB_LOG_MOD_DEBUG1LX(modid, p1, p2) 		IB_LOG_MOD_FMTLX(VS_LOG_DEBUG1, modid, p1, p2)
#define IB_LOG_DEBUG1_64(p1, p2) 				IB_LOG_FMT64(VS_LOG_DEBUG1, p1, p2)
#define IB_LOG_MOD_DEBUG1_64(modid, p1, p2) 	IB_LOG_MOD_FMT64(VS_LOG_DEBUG1, modid, p1, p2)
#define IB_LOG_DEBUG1STR(p1, p2) 				IB_LOG_FMTSTR(VS_LOG_DEBUG1, p1, p2)
#define IB_LOG_MOD_DEBUG1STR(modid, p1, p2) 	IB_LOG_MOD_FMTSTR(VS_LOG_DEBUG1, modid, p1, p2)
#define IB_LOG_DEBUG1RC(p1, p2) 				IB_LOG_FMTRC(VS_LOG_DEBUG1, p1, p2)
#define IB_LOG_MOD_DEBUG1RC(modid, p1, p2) 		IB_LOG_MOD_FMTRC(VS_LOG_DEBUG1, modid, p1, p2)
// for cs_log* and hence these macros
// Log_StrDup will be performed internally as part of formatting the message
#define IB_LOG_DEBUG1_FMT(func, fmt...) \
	   					cs_log(VS_LOG_DEBUG1, func, ## fmt)
#define IB_LOG_MOD_DEBUG1_FMT(modid,func, fmt...) \
	   					cs_log_mod(VS_LOG_DEBUG1, modid, func, ## fmt)
#define IB_LOG_DEBUG1_FMT_VF(vf, func, fmt...) \
						cs_log_vf(VS_LOG_DEBUG1, vf, func, ## fmt)
#define IB_LOG_MOD_DEBUG1_FMT_VF(modid,vf, func, fmt...) \
						cs_log_mod_vf(VS_LOG_DEBUG1, modid, vf, func, ## fmt)
#endif

/*******************************
 * DEBUG2
 ******************************/

#ifdef VIEO_TRACE_DISABLED
#define IB_LOG_DEBUG2_0(p1)	 					do { } while(0)
#define IB_LOG_MOD_DEBUG2_0(modid, p1)	 		do { } while(0)
#define IB_LOG_DEBUG2(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG2(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG2X(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG2X(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG2LX(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG2LX(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG2_64(p1, p2)				do { } while(0)
#define IB_LOG_MOD_DEBUG2_64(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG2STR(p1, p2)				do { } while(0)
#define IB_LOG_MOD_DEBUG2STR(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG2RC(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG2RC(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG2_FMT(func, fmt...) 		do { } while(0)
#define IB_LOG_MOD_DEBUG2_FMT(modid, func, fmt...) do { } while(0)
#define IB_LOG_DEBUG2_FMT_VF(vf, func, fmt...) 	do { } while(0)
#define IB_LOG_MOD_DEBUG2_FMT_VF(modid, vf, func, fmt...) do { } while(0)
#else
#define IB_LOG_DEBUG2_0(p1)	 					IB_LOG_FMT0(VS_LOG_DEBUG2, p1)
#define IB_LOG_MOD_DEBUG2_0(modid, p1)	 		IB_LOG_MOD_FMT0(VS_LOG_DEBUG2, modid, p1)
#define IB_LOG_DEBUG2(p1, p2)					IB_LOG_FMTU(VS_LOG_DEBUG2, p1, p2)
#define IB_LOG_MOD_DEBUG2(modid, p1, p2)		IB_LOG_MOD_FMTU(VS_LOG_DEBUG2, modid, p1, p2)
#define IB_LOG_DEBUG2X(p1, p2)					IB_LOG_FMTX(VS_LOG_DEBUG2, p1, p2)
#define IB_LOG_MOD_DEBUG2X(modid, p1, p2)		IB_LOG_MOD_FMTX(VS_LOG_DEBUG2, modid, p1, p2)
#define IB_LOG_DEBUG2LX(p1, p2) 				IB_LOG_FMTLX(VS_LOG_DEBUG2, p1, p2)
#define IB_LOG_MOD_DEBUG2LX(modid, p1, p2) 		IB_LOG_MOD_FMTLX(VS_LOG_DEBUG2, modid, p1, p2)
#define IB_LOG_DEBUG2_64(p1, p2) 				IB_LOG_FMT64(VS_LOG_DEBUG2, p1, p2)
#define IB_LOG_MOD_DEBUG2_64(modid, p1, p2) 	IB_LOG_MOD_FMT64(VS_LOG_DEBUG2, modid, p1, p2)
#define IB_LOG_DEBUG2STR(p1, p2) 				IB_LOG_FMTSTR(VS_LOG_DEBUG2, p1, p2)
#define IB_LOG_MOD_DEBUG2STR(modid, p1, p2) 	IB_LOG_MOD_FMTSTR(VS_LOG_DEBUG2, modid, p1, p2)
#define IB_LOG_DEBUG2RC(p1, p2) 				IB_LOG_FMTRC(VS_LOG_DEBUG2, p1, p2)
#define IB_LOG_MOD_DEBUG2RC(modid, p1, p2) 		IB_LOG_MOD_FMTRC(VS_LOG_DEBUG2, modid, p1, p2)
// for cs_log* and hence these macros
// Log_StrDup will be performed internally as part of formatting the message
#define IB_LOG_DEBUG2_FMT(func, fmt...) \
	   					cs_log(VS_LOG_DEBUG2, func, ## fmt)
#define IB_LOG_MOD_DEBUG2_FMT(modid,func, fmt...) \
	   					cs_log_mod(VS_LOG_DEBUG2, modid, func, ## fmt)
#define IB_LOG_DEBUG2_FMT_VF(vf, func, fmt...) \
						cs_log_vf(VS_LOG_DEBUG2, vf, func, ## fmt)
#define IB_LOG_MOD_DEBUG2_FMT_VF(modid,vf, func, fmt...) \
						cs_log_mod_vf(VS_LOG_DEBUG2, modid, vf, func, ## fmt)
#endif

/*******************************
 * DEBUG3
 ******************************/

#ifdef VIEO_TRACE_DISABLED
#define IB_LOG_DEBUG3_0(p1)	 					do { } while(0)
#define IB_LOG_MOD_DEBUG3_0(modid, p1)	 		do { } while(0)
#define IB_LOG_DEBUG3(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG3(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG3X(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG3X(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG3LX(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG3LX(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG3_64(p1, p2)				do { } while(0)
#define IB_LOG_MOD_DEBUG3_64(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG3STR(p1, p2)				do { } while(0)
#define IB_LOG_MOD_DEBUG3STR(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG3RC(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG3RC(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG3_FMT(func, fmt...) 		do { } while(0)
#define IB_LOG_MOD_DEBUG3_FMT(modid, func, fmt...) do { } while(0)
#define IB_LOG_DEBUG3_FMT_VF(vf, func, fmt...) 	do { } while(0)
#define IB_LOG_MOD_DEBUG3_FMT_VF(modid, vf, func, fmt...) do { } while(0)
#else
#define IB_LOG_DEBUG3_0(p1)	 					IB_LOG_FMT0(VS_LOG_DEBUG3, p1)
#define IB_LOG_MOD_DEBUG3_0(modid, p1)	 		IB_LOG_MOD_FMT0(VS_LOG_DEBUG3, modid, p1)
#define IB_LOG_DEBUG3(p1, p2)					IB_LOG_FMTU(VS_LOG_DEBUG3, p1, p2)
#define IB_LOG_MOD_DEBUG3(modid, p1, p2)		IB_LOG_MOD_FMTU(VS_LOG_DEBUG3, modid, p1, p2)
#define IB_LOG_DEBUG3X(p1, p2)					IB_LOG_FMTX(VS_LOG_DEBUG3, p1, p2)
#define IB_LOG_MOD_DEBUG3X(modid, p1, p2)		IB_LOG_MOD_FMTX(VS_LOG_DEBUG3, modid, p1, p2)
#define IB_LOG_DEBUG3LX(p1, p2) 				IB_LOG_FMTLX(VS_LOG_DEBUG3, p1, p2)
#define IB_LOG_MOD_DEBUG3LX(modid, p1, p2) 		IB_LOG_MOD_FMTLX(VS_LOG_DEBUG3, modid, p1, p2)
#define IB_LOG_DEBUG3_64(p1, p2) 				IB_LOG_FMT64(VS_LOG_DEBUG3, p1, p2)
#define IB_LOG_MOD_DEBUG3_64(modid, p1, p2) 	IB_LOG_MOD_FMT64(VS_LOG_DEBUG3, modid, p1, p2)
#define IB_LOG_DEBUG3STR(p1, p2) 				IB_LOG_FMTSTR(VS_LOG_DEBUG3, p1, p2)
#define IB_LOG_MOD_DEBUG3STR(modid, p1, p2) 	IB_LOG_MOD_FMTSTR(VS_LOG_DEBUG3, modid, p1, p2)
#define IB_LOG_DEBUG3RC(p1, p2) 				IB_LOG_FMTRC(VS_LOG_DEBUG3, p1, p2)
#define IB_LOG_MOD_DEBUG3RC(modid, p1, p2) 		IB_LOG_MOD_FMTRC(VS_LOG_DEBUG3, modid, p1, p2)
// for cs_log* and hence these macros
// Log_StrDup will be performed internally as part of formatting the message
#define IB_LOG_DEBUG3_FMT(func, fmt...) \
	   					cs_log(VS_LOG_DEBUG3, func, ## fmt)
#define IB_LOG_MOD_DEBUG3_FMT(modid,func, fmt...) \
	   					cs_log_mod(VS_LOG_DEBUG3, modid, func, ## fmt)
#define IB_LOG_DEBUG3_FMT_VF(vf, func, fmt...) \
						cs_log_vf(VS_LOG_DEBUG3, vf, func, ## fmt)
#define IB_LOG_MOD_DEBUG3_FMT_VF(modid,vf, func, fmt...) \
						cs_log_mod_vf(VS_LOG_DEBUG3, modid, vf, func, ## fmt)
#endif

/*******************************
 * DEBUG4
 ******************************/

#ifdef VIEO_TRACE_DISABLED
#define IB_LOG_DEBUG4_0(p1)	 					do { } while(0)
#define IB_LOG_MOD_DEBUG4_0(modid, p1)	 		do { } while(0)
#define IB_LOG_DEBUG4(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG4(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG4X(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG4X(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG4LX(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG4LX(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG4_64(p1, p2)				do { } while(0)
#define IB_LOG_MOD_DEBUG4_64(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG4STR(p1, p2)				do { } while(0)
#define IB_LOG_MOD_DEBUG4STR(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG4RC(p1, p2)					do { } while(0)
#define IB_LOG_MOD_DEBUG4RC(modid, p1, p2)		do { } while(0)
#define IB_LOG_DEBUG4_FMT(func, fmt...) 		do { } while(0)
#define IB_LOG_MOD_DEBUG4_FMT(modid, func, fmt...) do { } while(0)
#define IB_LOG_DEBUG4_FMT_VF(vf, func, fmt...) 	do { } while(0)
#define IB_LOG_MOD_DEBUG4_FMT_VF(modid, vf, func, fmt...) do { } while(0)
#else
#define IB_LOG_DEBUG4_0(p1)	 					IB_LOG_FMT0(VS_LOG_DEBUG4, p1)
#define IB_LOG_MOD_DEBUG4_0(modid, p1)	 		IB_LOG_MOD_FMT0(VS_LOG_DEBUG4, modid, p1)
#define IB_LOG_DEBUG4(p1, p2)					IB_LOG_FMTU(VS_LOG_DEBUG4, p1, p2)
#define IB_LOG_MOD_DEBUG4(modid, p1, p2)		IB_LOG_MOD_FMTU(VS_LOG_DEBUG4, modid, p1, p2)
#define IB_LOG_DEBUG4X(p1, p2)					IB_LOG_FMTX(VS_LOG_DEBUG4, p1, p2)
#define IB_LOG_MOD_DEBUG4X(modid, p1, p2)		IB_LOG_MOD_FMTX(VS_LOG_DEBUG4, modid, p1, p2)
#define IB_LOG_DEBUG4LX(p1, p2) 				IB_LOG_FMTLX(VS_LOG_DEBUG4, p1, p2)
#define IB_LOG_MOD_DEBUG4LX(modid, p1, p2) 		IB_LOG_MOD_FMTLX(VS_LOG_DEBUG4, modid, p1, p2)
#define IB_LOG_DEBUG4_64(p1, p2) 				IB_LOG_FMT64(VS_LOG_DEBUG4, p1, p2)
#define IB_LOG_MOD_DEBUG4_64(modid, p1, p2) 	IB_LOG_MOD_FMT64(VS_LOG_DEBUG4, modid, p1, p2)
#define IB_LOG_DEBUG4STR(p1, p2) 				IB_LOG_FMTSTR(VS_LOG_DEBUG4, p1, p2)
#define IB_LOG_MOD_DEBUG4STR(modid, p1, p2) 	IB_LOG_MOD_FMTSTR(VS_LOG_DEBUG4, modid, p1, p2)
#define IB_LOG_DEBUG4RC(p1, p2) 				IB_LOG_FMTRC(VS_LOG_DEBUG4, p1, p2)
#define IB_LOG_MOD_DEBUG4RC(modid, p1, p2) 		IB_LOG_MOD_FMTRC(VS_LOG_DEBUG4, modid, p1, p2)
// for cs_log* and hence these macros
// Log_StrDup will be performed internally as part of formatting the message
#define IB_LOG_DEBUG4_FMT(func, fmt...) \
	   					cs_log(VS_LOG_DEBUG4, func, ## fmt)
#define IB_LOG_MOD_DEBUG4_FMT(modid,func, fmt...) \
	   					cs_log_mod(VS_LOG_DEBUG4, modid, func, ## fmt)
#define IB_LOG_DEBUG4_FMT_VF(vf, func, fmt...) \
						cs_log_vf(VS_LOG_DEBUG4, vf, func, ## fmt)
#define IB_LOG_MOD_DEBUG4_FMT_VF(modid,vf, func, fmt...) \
						cs_log_mod_vf(VS_LOG_DEBUG4, modid, vf, func, ## fmt)
#endif

/*******************************
 * ENTER
 ******************************/

// TBD - rename this as IB_LOG_ENTER
#if (! defined(VIEO_DEBUG) && ! defined(DEBUG) ) || defined(VIEO_TRACE_DISABLED)
#define IB_MOD_ENTER(modid, p1,p2,p3,p4,p5)		((void)(p1))
// additional arguments on function entry
#define IB_LOG_MOD_ARGS1(modid,p1) 				do { } while (0)
#define IB_LOG_MOD_ARGS2(modid,p1,p2) 			do { } while (0)
#define IB_LOG_MOD_ARGS3(modid,p1,p2,p3) 		do { } while (0)
#define IB_LOG_MOD_ARGS4(modid,p1,p2,p3,p4) 	do { } while (0)
#define IB_LOG_MOD_ARGS5(modid,p1,p2,p3,p4,p5)	do { } while (0)
#else
#ifdef __VXWORKS__
// TBD - remove LogVal_t (32 bit) so can show 64 bit args too
// but will require callers to use LOG_PTR macro
#define IB_MOD_ENTER(modid, p1,p2,p3,p4,p5) 					\
	do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) && ! vs_log_filter() ) \
		LOG_DEBUG5(MOD_ESM, "SM Enter: %s%s %llu %llu %llu %llu", \
			LOG_PTR(cs_log_get_module_prefix(modid)),LOG_PTR(p1), \
			LOG_ARG((LogVal64_t)(LogVal_t)p2), LOG_ARG((LogVal64_t)(LogVal_t)p3), \
			LOG_ARG((LogVal64_t)(LogVal_t)p4), LOG_ARG((LogVal64_t)(LogVal_t)p5)); \
	} while (0)
#define IB_LOG_MOD_ARGS1(modid, p1) 					\
	do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) && ! vs_log_filter() ) \
		LOG_DEBUG5(MOD_ESM, "SM Args: %s %llu", \
			LOG_PTR(cs_log_get_module_prefix(modid)), \
			LOG_ARG((LogVal64_t)p1), 0, 0, 0, 0); \
	} while (0)
#define IB_LOG_MOD_ARGS2(modid, p1,p2) 					\
	do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) && ! vs_log_filter() ) \
		LOG_DEBUG5(MOD_ESM, "SM Args: %s %llu %llu", \
			LOG_PTR(cs_log_get_module_prefix(modid)), \
			LOG_ARG((LogVal64_t)p1), LOG_ARG((LogVal64_t)p2), 0, 0, 0); \
	} while (0)
#define IB_LOG_MOD_ARGS3(modid, p1,p2,p3) 					\
	do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) && ! vs_log_filter() ) \
		LOG_DEBUG5(MOD_ESM, "SM Args: %s %llu %llu %llu", \
			LOG_PTR(cs_log_get_module_prefix(modid)), \
			LOG_ARG((LogVal64_t)p1), LOG_ARG((LogVal64_t)p2), \
			LOG_ARG((LogVal64_t)p3), 0, 0); \
	} while (0)
#define IB_LOG_MOD_ARGS4(modid, p1,p2,p3,p4) 					\
	do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) && ! vs_log_filter() ) \
		LOG_DEBUG5(MOD_ESM, "SM Args: %s %llu %llu %llu %llu", \
			LOG_PTR(cs_log_get_module_prefix(modid)), \
			LOG_ARG((LogVal64_t)p1), LOG_ARG((LogVal64_t)p2), \
			LOG_ARG((LogVal64_t)p3), LOG_ARG((LogVal64_t)p4), 0); \
	} while (0)
#define IB_LOG_MOD_ARGS5(modid, p1,p2,p3,p4,p5) 					\
	do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) && ! vs_log_filter() ) \
		LOG_DEBUG5(MOD_ESM, "SM Args: %s %llu %llu %llu %llu %llu", \
			LOG_PTR(cs_log_get_module_prefix(modid)), \
			LOG_ARG((LogVal64_t)p1), LOG_ARG((LogVal64_t)p2), \
			LOG_ARG((LogVal64_t)p3), LOG_ARG((LogVal64_t)p4), \
			LOG_ARG((LogVal64_t)p5)); \
	} while (0)
#else
#define IB_MOD_ENTER(modid,p1,p2,p3,p4,p5) 					\
		do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) ) \
			vs_log_output(VS_LOG_ENTER, modid, __func__, NULL, \
					"%s %"CS64u" %"CS64u" %"CS64u" %"CS64u, \
					p1, (LogVal64_t)p2, (LogVal64_t)p3, (LogVal64_t)p4, \
					(LogVal64_t)p5); \
		} while (0)
#define IB_LOG_MOD_ARGS1(modid, p1) 					\
		do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) ) \
			vs_log_output(VS_LOG_ARGS, modid, __func__, NULL, \
					"%"CS64u, (LogVal64_t)p1); \
		} while (0)
#define IB_LOG_MOD_ARGS2(modid, p1,p2) 					\
		do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) ) \
			vs_log_output(VS_LOG_ARGS, modid, __func__, NULL, \
					"%"CS64u" %"CS64u, (LogVal64_t)p1, (LogVal64_t)p2); \
		} while (0)
#define IB_LOG_MOD_ARGS3(modid, p1,p2,p3) 					\
		do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) ) \
			vs_log_output(VS_LOG_ARGS, modid, __func__, NULL, \
					"%"CS64u" %"CS64u" %"CS64u, \
					(LogVal64_t)p1, (LogVal64_t)p2, (LogVal64_t)p3); \
		} while (0)
#define IB_LOG_MOD_ARGS4(modid, p1,p2,p3,p4) 					\
		do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) ) \
			vs_log_output(VS_LOG_ARGS, modid, __func__, NULL, \
					"%"CS64u" %"CS64u" %"CS64u" %"CS64u, \
					(LogVal64_t)p1, (LogVal64_t)p2, (LogVal64_t)p3, \
					(LogVal64_t)p4); \
		} while (0)
#define IB_LOG_MOD_ARGS5(modid, p1,p2,p3,p4,p5) 					\
		do { if (IB_LOG_IS_INTERESTED_MOD(VS_LOG_ENTER, modid) ) \
			vs_log_output(VS_LOG_ARGS, modid, __func__, NULL, \
					"%"CS64u" %"CS64u" %"CS64u" %"CS64u" %"CS64u, \
					(LogVal64_t)p1, (LogVal64_t)p2, (LogVal64_t)p3, \
					(LogVal64_t)p4, (LogVal64_t)p5); \
		} while (0)
#endif
#endif

#define IB_ENTER(p1,p2,p3,p4,p5)	IB_MOD_ENTER(LOCAL_MOD_ID,p1,p2,p3,p4,p5)
#define IB_LOG_ARGS1(p1) 			IB_LOG_MOD_ARGS1(LOCAL_MOD_ID,p1)
#define IB_LOG_ARGS2(p1,p2) 		IB_LOG_MOD_ARGS2(LOCAL_MOD_ID,p1,p2)
#define IB_LOG_ARGS3(p1,p2,p3)		IB_LOG_MOD_ARGS3(LOCAL_MOD_ID,p1,p2,p3)
#define IB_LOG_ARGS4(p1,p2,p3,p4)	IB_LOG_MOD_ARGS4(LOCAL_MOD_ID,p1,p2,p3,p4)
#define IB_LOG_ARGS5(p1,p2,p3,p4,p5) \
	   	IB_LOG_MOD_ARGS5(LOCAL_MOD_ID,p1,p2,p3,p4,p5)


/*******************************
 * EXIT
 ******************************/

// TBD - rename this as IB_LOG_EXIT
#if (! defined(VIEO_DEBUG) && ! defined(DEBUG) ) || defined(VIEO_TRACE_DISABLED)
#define IB_MOD_EXIT0(modid, p1)		((void)(p1))
#define IB_MOD_EXIT(modid, p1,p2)	((void)(p1))
#define IB_MOD_EXITRC(modid, p1,p2)	((void)(p1))
#else
#define IB_MOD_EXIT0(modid, p1)		IB_LOG_MOD_FMT0(VS_LOG_EXIT,modid,p1)
#define IB_MOD_EXIT(modid, p1, p2)	IB_LOG_MOD_FMTU(VS_LOG_EXIT,modid,p1,p2)
#define IB_MOD_EXITRC(modid, p1, p2) IB_LOG_MOD_FMTRC(VS_LOG_EXIT,modid,p1,p2)
#endif

#define IB_EXIT0(p1)		IB_MOD_EXIT0(LOCAL_MOD_ID,p1)
#define IB_EXIT(p1,p2)		IB_MOD_EXIT(LOCAL_MOD_ID,p1,p2)
#define IB_EXITRC(p1,p2)	IB_MOD_EXITRC(LOCAL_MOD_ID,p1,p2)

/**************************************************************************/
/******************************** API *************************************/
/**************************************************************************/

/** Define Commands accepted by vs_log_control call. **********************/

#define VS_LOG_SETMASK    		(1)
#define VS_LOG_SETOUTPUTFILE   	(6) /* Causes log messages to go to the file. Passing in an empty string sets log back to syslog */
#define VS_LOG_SETSYSLOGNAME	(7)	/* Sets the name that will appear in syslog messages. */
#define VS_LOG_SETFACILITY		(8)	/* Sets the syslog facility. */
#define VS_LOG_STARTSYSLOG		(9)	/* Starts Syslog. */

/*
 * vs_log_control
 *    Function used to pass commands to the logging subsystem.
 *
 * INPUTS
 *    cmd               Defines the action to carried out in the subsystem
 *    arg1              First arg type
 *    arg2              Second arg
 *    arg3              Second arg
 *
 * cmd == VS_LOG_STARTSYSLOG
 *
 *        arg1, arg2 and arg3 is NULL.
 *
 * cmd == VS_LOG_SETFACILITY
 *
 *        arg1 Contains facility
 *        arg2 and arg3 is NULL.
 *
 * cmd == VS_LOG_SETMASK
 *
 *        arg1 Contains a new debug mask
 *        arg2 NULL
 *        arg3 NULL
 *
 * RETURNS
 *      VSTATUS_OK
 *      VSTATUS_BAD
 */

Status_t        vs_log_control(int cmd, void *arg1, void *arg2,
			       void *arg3);

// module name in form of "name: ", useful for messages where name is optional
// for VIEO_NONE_MOD_ID, returns ""
const char *
cs_log_get_module_prefix(uint32_t modid);

// module name in simple form, useful to put in middle of other messages
// names provided also match prefixes used in LogMask config file names
// for VIEO_NONE_MOD_ID, returns "NONE"
const char *
cs_log_get_module_name(uint32_t modid);

const char*
cs_log_get_sev_name(uint32_t sev);

// fill log_masks based on mod_mask and sev_mask
// not recommended for general use, use cs_log_set_log_masks
void
cs_log_set_mods_mask(uint32_t mod_mask, uint32_t sev_mask, uint32_t log_masks[VIEO_LAST_MOD_ID+1]);

// build log_masks based on level and mode
void
cs_log_set_log_masks(uint32_t level, int mode, uint32_t log_masks[VIEO_LAST_MOD_ID+1]);

// convert module name to modid
// names provided also match prefixes used in LogMask config file names
// returns 0 if invalid name given
uint32_t
cs_log_get_module_id(const char * mod);

// set mask for a given module specified by name
void
cs_log_set_log_mask(const char* mod, uint32_t mask, uint32_t log_masks[VIEO_LAST_MOD_ID+1]);

// get mask for a given module specified by name
uint32_t
cs_log_get_log_mask(const char* mod, uint32_t log_masks[VIEO_LAST_MOD_ID+1]);

char *
cs_convert_status (Status_t status);

// always does Log_StrDup of msg for VxWorks
void
vs_log_output_message(char *msg, int show_masks);

void
vs_log_set_log_mode(int mode);

void
vs_log_output(uint32_t sev, /* severity */
	    uint32_t modid,	/* optional Module id */
		const char *function, /* optional function name */
		const char *vf,	/* optional vFabric name */
		const char *format, ...
		) __attribute__((format(printf,5,6)));

FILE* vs_log_get_logfile_fd(void);

#ifdef __VXWORKS__
// used for single argument data output so we can avoid Log_StrDup for
// IB_LOG_FMT* macros
void
vs_log_output_data(uint32_t sev, /* severity */
		uint32_t modid,	/* optional Module id */
		const char *format,	// "%s%s%s%s%s 0x%x", 
							// have some #defines for format
		const char *function, /* optional function name */	// TBD if can use
		const char *vf,	/* optional vFabric name */	// TBD if can use
	   	Log_Arg_t message, Log_Arg_t arg
		);

// check for single thread filtering
int vs_log_filter(void);

#else
// hex dump of a memory buffer at addr[0] - addr[len-1]
void
vs_log_output_memory(uint32_t sev, /* severity */
		uint32_t modid,	/* optional Module id */
		const char *function, /* optional function name */
		const char *vf,	/* optional vFabric name */
		const char *prefix,
		const void* addr,
		uint32_t len
		);

// fatal abort with core dump
void vs_fatal_error(uint8_t *string);
#endif

//
// IB_*_NOREPEAT(lastMst, msgNum, [argument list]) 
//
// Prevent the same error from coming out over and over by tracking the
// previous error that was logged. Functions using these macros should locally
// declare an (optionally static) unsigned variable to track the last message
// sent and clear that variable before returning successfully. In addition, 
// each invocation of IB_WARN_NOREPEAT and IB_ERROR_NOREPEAT in that function should 
// be assigned a unique msgNum.
#define IB_WARN_NOREPEAT(lastMsg, msgNum, fmt...) \
	if (lastMsg != msgNum) { \
		cs_log(VS_LOG_WARN,__func__, ## fmt); \
		lastMsg = msgNum; \
	}

#define IB_ERROR_NOREPEAT(lastMsg, msgNum, fmt...) \
	if (lastMsg != msgNum) { \
		cs_log(VS_LOG_ERROR,__func__, ## fmt); \
		lastMsg = msgNum; \
	}

#endif /*__VIEO_VS_LOG__*/
