/* BEGIN_ICS_COPYRIGHT5 ****************************************

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

 * ** END_ICS_COPYRIGHT5   ****************************************/

/***********************************************************************
* 
* FILE NAME
*      cs_string.c
*
* DESCRIPTION
*      This file contains the implementation of the String Conversion
*      services.
*
* DATA STRUCTURES
*
* FUNCTIONS
*      cs_strtoui8
*      cs_strtoui16
*      cs_strtoui32
*      cs_strtoui64
*      cs_strtoi8
*      cs_strtoi16
*      cs_strtoui32
*      cs_strtoi64
*      cs_parse_gid
*
* DEPENDENCIES
*      cs_g.h
*
*
* HISTORY
*
* NAME      DATE        REMARKS
* MGR       05/01/02    PR1754.  Initial creation of file.
* MGR       05/06/02    PR1754.  Added support for functions.
* MGR       05/07/02    PR1812.  Clean-up lint errors.
*
***********************************************************************/
#include "ib_types.h"
#include "ib_status.h"
#include "cs_g.h"
#include "vs_g.h"
#include "cs_log.h"
#define function __FUNCTION__
#ifdef LOCAL_MOD_ID
#undef LOCAL_MOD_ID
#endif
#define LOCAL_MOD_ID VIEO_CS_MOD_ID




/**********************************************************************
*
* FUNCTION
*    cs_strtoui8
*
* DESCRIPTION
*    Convert a string to an unsigned 8-bit integer.
*
* INPUTS
*    nptr         - A pointer to the string to convert.
*    return_error - Value to return on error.
*
* OUTPUTS
*    cvalue       - On success, the converted value is returned.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     05/01/02    Initial creation of function.
*
**********************************************************************/
uint8_t
cs_strtoui8 (const char *nptr, uint8_t return_error)
{
  uint32_t          base = 0;
  int               overflow = 0;
  uint8_t           digit;
  uint8_t           cvalue = 0;
  uint32_t          maxdiv;
  uint32_t          maxrem;

  IB_ENTER (function,
            (unint)nptr,
            (uint32_t)return_error,
            (uint32_t)0U,
	    (uint32_t)0U);

  /*
  ** Validate string pointer
  */
  if (nptr == (char *)0)
  {
    IB_LOG_ERROR0 ("NULL pointer");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  /*
  ** Skip leading spaces
  */
  while (*nptr == ' ')
  {
    nptr++;
  }

  /*
  ** Allow a leading '+'; however negative values are not allowed.
  */
  if (*nptr == '+')
  {
    nptr++;
  }
  else if (*nptr == '-')
  {
    IB_LOG_ERROR0 ("negative values are invalid");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  /*
   * Determine the base
   */
  if (*nptr != '0')
  {
    /* decimal string */
    base = 10;
    IB_LOG_INFO ("string is decimal", (uint32_t)base);
  }
  else
  {
    /* first 'real' character is a 0 */
    if (nptr[1] == 'X' || nptr[1] == 'x')
    {
      /* hexadecimal string */
      base = 16;
      nptr += 2;
      IB_LOG_INFO ("string is hexadecimal", (uint32_t)base);
    }
    else
    {
      /* decimal string */
      base = 10;
      IB_LOG_INFO ("string is decimal", (uint32_t)base);
    }
  }

  maxdiv = UINT8_MAX / base;
  maxrem = UINT8_MAX % base;

  while ((digit = *nptr) != '\0')
  {
    if (digit >= '0' && digit < ('0' + base))
    {
      digit -= '0';
    }
    else
    {
      if (base > 10)
      {
        if (digit >= 'a' && digit < ('a' + (base - 10)))
        {
          digit = digit - 'a' + 10;
        }
        else if (digit >= 'A' && digit < ('A' + (base - 10)))
        {
          digit = digit - 'A' + 10;
        }
        else
        {
          IB_LOG_ERROR ("invalid digit:", (uint32_t)digit);
          IB_EXIT (function, (uint32_t)return_error);
          return return_error;
        }
      }
      else
      {
        IB_LOG_ERROR ("invalid digit:", (uint32_t)digit);
        IB_EXIT (function, (uint32_t)return_error);
        return return_error;
      }
    }

    if (cvalue > maxdiv || (cvalue == maxdiv && digit > maxrem))
    {
      overflow = 1;
    }

    cvalue = (cvalue * base) + digit;
    nptr++;
  }
	
  if (overflow)
  {
    IB_LOG_ERROR0 ("string value is too large");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  IB_EXIT (function, (uint32_t)cvalue);
  return cvalue;
}

/**********************************************************************
*
* FUNCTION
*    cs_strtoui16
*
* DESCRIPTION
*    Convert a string to an unsigned 16-bit integer.
*
* INPUTS
*    nptr         - A pointer to the string to convert.
*    return_error - Value to return on error.
*
* OUTPUTS
*    cvalue       - On success, the converted value is returned.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     05/01/02    Initial creation of function.
*
**********************************************************************/
uint16_t
cs_strtoui16 (const char *nptr, uint16_t return_error)
{
  uint32_t          base = 0;
  int               overflow = 0;
  uint8_t           digit;
  uint16_t          cvalue = 0;
  uint32_t          maxdiv;
  uint32_t          maxrem;

  IB_ENTER (function,
            (unint)nptr,
            (uint32_t)return_error,
            (uint32_t)0U,
	    (uint32_t)0U);

  /*
  ** Validate string pointer
  */
  if (nptr == (char *)0)
  {
    IB_LOG_ERROR0 ("NULL pointer");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  /*
  ** Skip leading spaces
  */
  while (*nptr == ' ')
  {
    nptr++;
  }

  /*
  ** Allow a leading '+'; however negative values are not allowed.
  */
  if (*nptr == '+')
  {
    nptr++;
  }
  else if (*nptr == '-')
  {
    IB_LOG_ERROR0 ("negative values are invalid");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  /*
   * Determine the base
   */
  if (*nptr != '0')
  {
    /* decimal string */
    base = 10;
    IB_LOG_INFO ("string is decimal", (uint32_t)base);
  }
  else
  {
    /* first 'real' character is a 0 */
    if (nptr[1] == 'X' || nptr[1] == 'x')
    {
      /* hexadecimal string */
      base = 16;
      nptr += 2;
      IB_LOG_INFO ("string is hexadecimal", (uint32_t)base);
    }
    else
    {
      /* decimal string */
      base = 10;
      IB_LOG_INFO ("string is decimal", (uint32_t)base);
    }
  }

  maxdiv = UINT16_MAX / base;
  maxrem = UINT16_MAX % base;

  while ((digit = *nptr) != '\0')
  {
    if (digit >= '0' && digit < ('0' + base))
    {
      digit -= '0';
    }
    else
    {
      if (base > 10)
      {
        if (digit >= 'a' && digit < ('a' + (base - 10)))
        {
          digit = digit - 'a' + 10;
        }
        else if (digit >= 'A' && digit < ('A' + (base - 10)))
        {
          digit = digit - 'A' + 10;
        }
        else
        {
          IB_LOG_ERROR ("invalid digit:", (uint32_t)digit);
          IB_EXIT (function, (uint32_t)return_error);
          return return_error;
        }
      }
      else
      {
        IB_LOG_ERROR ("invalid digit:", (uint32_t)digit);
        IB_EXIT (function, (uint32_t)return_error);
        return return_error;
      }
    }

    if (cvalue > maxdiv || (cvalue == maxdiv && digit > maxrem))
    {
      overflow = 1;
    }

    cvalue = (cvalue * base) + digit;
    nptr++;
  }
	
  if (overflow)
  {
    IB_LOG_ERROR0 ("string value is too large");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  IB_EXIT (function, (uint32_t)cvalue);
  return cvalue;
}

/**********************************************************************
*
* FUNCTION
*    cs_strtoui32
*
* DESCRIPTION
*    Convert a string to an unsigned 32-bit integer.
*
* INPUTS
*    nptr         - A pointer to the string to convert.
*    return_error - Value to return on error.
*
* OUTPUTS
*    cvalue       - On success, the converted value is returned.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     05/01/02    Initial creation of function.
*
**********************************************************************/
uint32_t
cs_strtoui32 (const char *nptr, uint32_t return_error)
{
  uint32_t          base = 0;
  int               overflow = 0;
  uint8_t           digit;
  uint32_t          cvalue = 0;
  uint32_t          maxdiv;
  uint32_t          maxrem;

  IB_ENTER (function,
            (unint)nptr,
            (uint32_t)return_error,
            (uint32_t)0U,
	    (uint32_t)0U);

  /*
  ** Validate string pointer
  */
  if (nptr == (char *)0)
  {
    IB_LOG_ERROR0 ("NULL pointer");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  /*
  ** Skip leading spaces
  */
  while (*nptr == ' ')
  {
    nptr++;
  }

  /*
  ** Allow a leading '+'; however negative values are not allowed.
  */
  if (*nptr == '+')
  {
    nptr++;
  }
  else if (*nptr == '-')
  {
    IB_LOG_ERROR0 ("negative values are invalid");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  /*
   * Determine the base
   */
  if (*nptr != '0')
  {
    /* decimal string */
    base = 10;
    IB_LOG_INFO ("string is decimal", (uint32_t)base);
  }
  else
  {
    /* first 'real' character is a 0 */
    if (nptr[1] == 'X' || nptr[1] == 'x')
    {
      /* hexadecimal string */
      base = 16;
      nptr += 2;
      IB_LOG_INFO ("string is hexadecimal", (uint32_t)base);
    }
    else
    {
      /* decimal string */
      base = 10;
      IB_LOG_INFO ("string is decimal", (uint32_t)base);
    }
  }

  maxdiv = UINT32_MAX / base;
  maxrem = UINT32_MAX % base;

  while ((digit = *nptr) != '\0')
  {
    if (digit >= '0' && digit < ('0' + base))
    {
      digit -= '0';
    }
    else
    {
      if (base > 10)
      {
        if (digit >= 'a' && digit < ('a' + (base - 10)))
        {
          digit = digit - 'a' + 10;
        }
        else if (digit >= 'A' && digit < ('A' + (base - 10)))
        {
          digit = digit - 'A' + 10;
        }
        else
        {
          IB_LOG_ERROR ("invalid digit:", (uint32_t)digit);
          IB_EXIT (function, (uint32_t)return_error);
          return return_error;
        }
      }
      else
      {
        IB_LOG_ERROR ("invalid digit:", (uint32_t)digit);
        IB_EXIT (function, (uint32_t)return_error);
        return return_error;
      }
    }

    if (cvalue > maxdiv || (cvalue == maxdiv && digit > maxrem))
    {
      overflow = 1;
    }

    cvalue = (cvalue * base) + digit;
    nptr++;
  }
	
  if (overflow)
  {
    IB_LOG_ERROR0 ("string value is too large");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  IB_EXIT (function, (uint32_t)cvalue);
  return cvalue;
}

/**********************************************************************
*
* FUNCTION
*    cs_strtoui64
*
* DESCRIPTION
*    Convert a string to an unsigned 64-bit integer.
*
* INPUTS
*    nptr         - A pointer to the string to convert.
*    return_error - Value to return on error.
*
* OUTPUTS
*    cvalue       - On success, the converted value is returned.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     05/01/02    Initial creation of function.
*   PJG     05/28/02    PR2166 Suffix 64-bit constants with ll to
*                         eliminate warnings.
*
**********************************************************************/
uint64_t
cs_strtoui64 (const char *nptr, uint64_t return_error)
{
  uint32_t          base = 0;
  int               overflow = 0;
  uint8_t           digit;
  uint64_t          cvalue = 0;
  uint64_t          maxdiv = 0;
  uint64_t          maxrem = 0;

  IB_ENTER (function,
            (unint)nptr,
            (uint32_t)return_error,
            (uint32_t)0U,
	    (uint32_t)0U);

  /*
  ** Validate string pointer
  */
  if (nptr == (char *)0)
  {
    IB_LOG_ERROR0 ("NULL pointer");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  /*
  ** Skip leading spaces
  */
  while (*nptr == ' ')
  {
    nptr++;
  }

  /*
  ** Allow a leading '+'; however negative values are not allowed.
  */
  if (*nptr == '+')
  {
    nptr++;
  }
  else if (*nptr == '-')
  {
    IB_LOG_ERROR0 ("negative values are invalid");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  /*
   * Determine the base
   */
  if (*nptr != '0')
  {
    /* decimal string */
    base = 10;
    IB_LOG_INFO ("string is decimal", (uint32_t)base);
  }
  else
  {
    /* first 'real' character is a 0 */
    if (nptr[1] == 'X' || nptr[1] == 'x')
    {
      /* hexadecimal string */
      base = 16;
      nptr += 2;
      IB_LOG_INFO ("string is hexadecimal", (uint32_t)base);
    }
    else
    {
      /* decimal string */
      base = 10;
      IB_LOG_INFO ("string is decimal", (uint32_t)base);
    }
  }

  /* Hardcode these values since it causes module load errors */
  if (base == 10)
  {
    maxdiv = 1844674407370955161ll;
    maxrem = 5;
  }
  else if (base == 16)
  {
    maxdiv = 1152921504606846975ll;
    maxrem = 15;
  }

  while ((digit = *nptr) != '\0')
  {
    if (digit >= '0' && digit < ('0' + base))
    {
      digit -= '0';
    }
    else
    {
      if (base > 10)
      {
        if (digit >= 'a' && digit < ('a' + (base - 10)))
        {
          digit = digit - 'a' + 10;
        }
        else if (digit >= 'A' && digit < ('A' + (base - 10)))
        {
          digit = digit - 'A' + 10;
        }
        else
        {
          IB_LOG_ERROR ("invalid digit:", (uint32_t)digit);
          IB_EXIT (function, (uint32_t)return_error);
          return return_error;
        }
      }
      else
      {
        IB_LOG_ERROR ("invalid digit:", (uint32_t)digit);
        IB_EXIT (function, (uint32_t)return_error);
        return return_error;
      }
    }

    if (cvalue > maxdiv || (cvalue == maxdiv && digit > maxrem))
    {
      overflow = 1;
    }

    cvalue = (cvalue * base) + digit;
    nptr++;
  }
	
  if (overflow)
  {
    IB_LOG_ERROR0 ("string value is too large");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  IB_EXIT (function, (uint32_t)cvalue);
  return cvalue;
}

/**********************************************************************
*
* FUNCTION
*    cs_strtoi8
*
* DESCRIPTION
*    Convert a string to a signed 8-bit integer.
*
* INPUTS
*    nptr         - A pointer to the string to convert.
*    return_error - Value to return on error.
*
* OUTPUTS
*    cvalue       - On success, the converted value is returned.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     05/01/02    Initial creation of function.
*
**********************************************************************/
int8_t
cs_strtoi8 (const char *nptr, int8_t return_error)
{
  int               base = 0;
  int               negate = 0;
  int8_t            cvalue = 0;
  int8_t            digit;

  IB_ENTER (function,
            nptr,
            (uint32_t)(int32_t)return_error,
            0U,
            0U);

  /*
  ** Validate string pointer
  */
  if (nptr == (char *)0)
  {
    IB_LOG_ERROR0 ("NULL pointer");
    IB_EXIT (function, (uint32_t)(int32_t)return_error);
    return return_error;
  }

  /*
  ** Skip leading spaces
  */
  while (*nptr == ' ')
  {
    nptr++;
  }

  /*
  ** Allow a leading '+'; however negative values are not allowed.
  */
  if (*nptr == '+')
  {
    nptr++;
  }
  else if (*nptr == '-')
  {
    negate = 1;
    nptr++;
  }

  /*
   * Determine the base
   */
  if (*nptr != '0')
  {
    /* decimal string */
    base = 10;
    IB_LOG_INFO ("string is decimal", (uint32_t)base);
  }
  else
  {
    /* first 'real' character is a 0 */
    if (nptr[1] == 'X' || nptr[1] == 'x')
    {
      /* hexadecimal string */
      base = 16;
      nptr += 2;
      IB_LOG_INFO ("string is hexadecimal", (uint32_t)base);
    }
    else
    {
      /* decimal string */
      base = 10;
      IB_LOG_INFO ("string is decimal", (uint32_t)base);
    }
  }

  while ((digit = *nptr) != '\0')
  {
    if (digit >= '0' && digit < ('0' + base))
    {
      digit -= '0';
    }
    else
    {
      if (base > 10)
      {
        if (digit >= 'a' && digit < ('a' + (base - 10)))
        {
          digit = digit - 'a' + 10;
        }
        else if (digit >= 'A' && digit < ('A' + (base - 10)))
        {
          digit = digit - 'A' + 10;
        }
        else
        {
          IB_LOG_ERROR ("invalid digit:", (uint32_t)(int32_t)digit);
          IB_EXIT (function, (uint32_t)(int32_t)return_error);
          return return_error;
        }
      }
      else
      {
        IB_LOG_ERROR ("invalid digit:", (uint32_t)(int32_t)digit);
        IB_EXIT (function, (uint32_t)(int32_t)return_error);
        return return_error;
      }
    }

    if (negate == 1)
    {
      digit = -digit;
      if ((cvalue > (INT8_MIN / base)) ||
          ((cvalue == (INT8_MIN / base)) && (digit >= (INT8_MIN % base))))
      {
        cvalue = cvalue * base;
        cvalue += digit;
      }
      else
      {
        IB_LOG_ERROR0 ("string value is too small");
        IB_EXIT (function, (uint32_t)(int32_t)return_error);
        return return_error;
      }
    }
    else
    {
      if ((cvalue < (INT8_MAX / base)) ||
          ((cvalue == (INT8_MAX / base)) && (digit <= (INT8_MAX % base))))
      {
        cvalue = cvalue * base;
        cvalue += digit;
      }
      else
      {
        IB_LOG_ERROR0 ("string value is too large");
        IB_EXIT (function, (uint32_t)(int32_t)return_error);
        return return_error;
      }
    }
    nptr++;
  }

  IB_EXIT (function, (uint32_t)(int32_t)cvalue);
  return cvalue;
}

/**********************************************************************
*
* FUNCTION
*    cs_strtoi16
*
* DESCRIPTION
*    Convert a string to a signed 16-bit integer.
*
* INPUTS
*    nptr         - A pointer to the string to convert.
*    return_error - Value to return on error.
*
* OUTPUTS
*    cvalue       - On success, the converted value is returned.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     05/01/02    Initial creation of function.
*
**********************************************************************/
int16_t
cs_strtoi16 (const char *nptr, int16_t return_error)
{
  int               base = 0;
  int               negate = 0;
  int16_t           cvalue = 0;
  int8_t            digit;

  IB_ENTER (function,
            (unint)nptr,
            (uint32_t)(int32_t)return_error,
            (uint32_t)0U,
	    (uint32_t)0U);

  /*
  ** Validate string pointer
  */
  if (nptr == (char *)0)
  {
    IB_LOG_ERROR0 ("NULL pointer");
    IB_EXIT (function, (uint32_t)(int32_t)return_error);
    return return_error;
  }

  /*
  ** Skip leading spaces
  */
  while (*nptr == ' ')
  {
    nptr++;
  }

  /*
  ** Allow a leading '+'; however negative values are not allowed.
  */
  if (*nptr == '+')
  {
    nptr++;
  }
  else if (*nptr == '-')
  {
    negate = 1;
    nptr++;
  }

  /*
   * Determine the base
   */
  if (*nptr != '0')
  {
    /* decimal string */
    base = 10;
    IB_LOG_INFO ("string is decimal", (uint32_t)base);
  }
  else
  {
    /* first 'real' character is a 0 */
    if (nptr[1] == 'X' || nptr[1] == 'x')
    {
      /* hexadecimal string */
      base = 16;
      nptr += 2;
      IB_LOG_INFO ("string is hexadecimal", (uint32_t)base);
    }
    else
    {
      /* decimal string */
      base = 10;
      IB_LOG_INFO ("string is decimal", (uint32_t)base);
    }
  }

  while ((digit = *nptr) != '\0')
  {
    if (digit >= '0' && digit < ('0' + base))
    {
      digit -= '0';
    }
    else
    {
      if (base > 10)
      {
        if (digit >= 'a' && digit < ('a' + (base - 10)))
        {
          digit = digit - 'a' + 10;
        }
        else if (digit >= 'A' && digit < ('A' + (base - 10)))
        {
          digit = digit - 'A' + 10;
        }
        else
        {
          IB_LOG_ERROR ("invalid digit:", (uint32_t)(int32_t)digit);
          IB_EXIT (function, (uint32_t)(int32_t)return_error);
          return return_error;
        }
      }
      else
      {
        IB_LOG_ERROR ("invalid digit:", (uint32_t)(int32_t)digit);
        IB_EXIT (function, (uint32_t)(int32_t)return_error);
        return return_error;
      }
    }

    if (negate == 1)
    {
      digit = -digit;
      if ((cvalue > (INT16_MIN / base)) ||
          ((cvalue == (INT16_MIN / base)) && (digit >= (INT16_MIN % base))))
      {
        cvalue = cvalue * base;
        cvalue += digit;
      }
      else
      {
        IB_LOG_ERROR0 ("string value is too small");
        IB_EXIT (function, (uint32_t)(int32_t)return_error);
        return return_error;
      }
    }
    else
    {
      if ((cvalue < (INT16_MAX / base)) ||
          ((cvalue == (INT16_MAX / base)) && (digit <= (INT16_MAX % base))))
      {
        cvalue = cvalue * base;
        cvalue += digit;
      }
      else
      {
        IB_LOG_ERROR0 ("string value is too large");
        IB_EXIT (function, (uint32_t)(int32_t)return_error);
        return return_error;
      }
    }
    nptr++;
  }

  IB_EXIT (function, (uint32_t)(int32_t)cvalue);
  return cvalue;
}

/**********************************************************************
*
* FUNCTION
*    cs_strtoi32
*
* DESCRIPTION
*    Convert a string to a signed 32-bit integer.
*
* INPUTS
*    nptr         - A pointer to the string to convert.
*    return_error - Value to return on error.
*
* OUTPUTS
*    cvalue       - On success, the converted value is returned.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     05/01/02    Initial creation of function.
*
**********************************************************************/
int32_t
cs_strtoi32 (const char *nptr, int32_t return_error)
{
  int               base = 0;
  int               negate = 0;
  int32_t           cvalue = 0;
  int8_t            digit;

  IB_ENTER (function,
            (unint)nptr,
            (uint32_t)return_error,
            (uint32_t)0U,
	    (uint32_t)0U);

  /*
  ** Validate string pointer
  */
  if (nptr == (char *)0)
  {
    IB_LOG_ERROR0 ("NULL pointer");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  /*
  ** Skip leading spaces
  */
  while (*nptr == ' ')
  {
    nptr++;
  }

  /*
  ** Allow a leading '+'; however negative values are not allowed.
  */
  if (*nptr == '+')
  {
    nptr++;
  }
  else if (*nptr == '-')
  {
    negate = 1;
    nptr++;
  }

  /*
   * Determine the base
   */
  if (*nptr != '0')
  {
    /* decimal string */
    base = 10;
    IB_LOG_INFO ("string is decimal", (uint32_t)base);
  }
  else
  {
    /* first 'real' character is a 0 */
    if (nptr[1] == 'X' || nptr[1] == 'x')
    {
      /* hexadecimal string */
      base = 16;
      nptr += 2;
      IB_LOG_INFO ("string is hexadecimal", (uint32_t)base);
    }
    else
    {
      /* decimal string */
      base = 10;
      IB_LOG_INFO ("string is decimal", (uint32_t)base);
    }
  }

  while ((digit = *nptr) != '\0')
  {
    if (digit >= '0' && digit < ('0' + base))
    {
      digit -= '0';
    }
    else
    {
      if (base > 10)
      {
        if (digit >= 'a' && digit < ('a' + (base - 10)))
        {
          digit = digit - 'a' + 10;
        }
        else if (digit >= 'A' && digit < ('A' + (base - 10)))
        {
          digit = digit - 'A' + 10;
        }
        else
        {
          IB_LOG_ERROR ("invalid digit:", (uint32_t)(int32_t)digit);
          IB_EXIT (function, (uint32_t)return_error);
          return return_error;
        }
      }
      else
      {
        IB_LOG_ERROR ("invalid digit:", (uint32_t)(int32_t)digit);
        IB_EXIT (function, (uint32_t)return_error);
        return return_error;
      }
    }

    if (negate == 1)
    {
      digit = -digit;
      if ((cvalue > (INT32_MIN / base)) ||
          ((cvalue == (INT32_MIN / base)) && (digit >= (INT32_MIN % base))))
      {
        cvalue = cvalue * base;
        cvalue += digit;
      }
      else
      {
        IB_LOG_ERROR0 ("string value is too small");
        IB_EXIT (function, (uint32_t)return_error);
        return return_error;
      }
    }
    else
    {
      if ((cvalue < (INT32_MAX / base)) ||
          ((cvalue == (INT32_MAX / base)) && (digit <= (INT32_MAX % base))))
      {
        cvalue = cvalue * base;
        cvalue += digit;
      }
      else
      {
        IB_LOG_ERROR0 ("string value is too large");
        IB_EXIT (function, (uint32_t)return_error);
        return return_error;
      }
    }
    nptr++;
  }

  IB_EXIT (function, (uint32_t)cvalue);
  return cvalue;
}

/**********************************************************************
*
* FUNCTION
*    cs_strtoi64
*
* DESCRIPTION
*    Convert a string to a signed 64-bit integer.
*
* INPUTS
*    nptr         - A pointer to the string to convert.
*    return_error - Value to return on error.
*
* OUTPUTS
*    cvalue       - On success, the converted value is returned.
*
*
* HISTORY
*
*   NAME    DATE        REMARKS
*   MGR     05/01/02    Initial creation of function.
*   PJG     05/28/02    PR2166 Suffix 64-bit constants with ll to
*                         eliminate warnings.
*
**********************************************************************/
int64_t
cs_strtoi64 (const char *nptr, int64_t return_error)
{
  int               base = 0;
  int               negate = 0;
  int8_t            digit;
  int64_t           cvalue = 0;

  IB_ENTER (function,
            (unint)nptr,
            (uint32_t)return_error,
            (uint32_t)0U,
	    (uint32_t)0U);

  /*
  ** Validate string pointer
  */
  if (nptr == (char *)0)
  {
    IB_LOG_ERROR0 ("NULL pointer");
    IB_EXIT (function, (uint32_t)return_error);
    return return_error;
  }

  /*
  ** Skip leading spaces
  */
  while (*nptr == ' ')
  {
    nptr++;
  }

  /*
  ** Allow a leading '+'; however negative values are not allowed.
  */
  if (*nptr == '+')
  {
    nptr++;
  }
  else if (*nptr == '-')
  {
    negate = 1;
    nptr++;
  }

  /*
   * Determine the base
   */
  if (*nptr != '0')
  {
    /* decimal string */
    base = 10;
    IB_LOG_INFO ("string is decimal", (uint32_t)base);
  }
  else
  {
    /* first 'real' character is a 0 */
    if (nptr[1] == 'X' || nptr[1] == 'x')
    {
      /* hexadecimal string */
      base = 16;
      nptr += 2;
      IB_LOG_INFO ("string is hexadecimal", (uint32_t)base);
    }
    else
    {
      /* decimal string */
      base = 10;
      IB_LOG_INFO ("string is decimal", (uint32_t)base);
    }
  }

  while ((digit = *nptr) != '\0')
  {
    if (digit >= '0' && digit < ('0' + base))
    {
      digit -= '0';
    }
    else
    {
      if (base > 10)
      {
        if (digit >= 'a' && digit < ('a' + (base - 10)))
        {
          digit = digit - 'a' + 10;
        }
        else if (digit >= 'A' && digit < ('A' + (base - 10)))
        {
          digit = digit - 'A' + 10;
        }
        else
        {
          IB_LOG_ERROR ("invalid digit:", (uint32_t)(int32_t)digit);
          IB_EXIT (function, (uint32_t)return_error);
          return return_error;
        }
      }
      else
      {
        IB_LOG_ERROR ("invalid digit:", (uint32_t)(int32_t)digit);
        IB_EXIT (function, (uint32_t)return_error);
        return return_error;
      }
    }

    if (negate == 1)
    {
      digit = -digit;
      if ((cvalue > -922337203685477580ll) ||
          ((cvalue == -922337203685477580ll) && (digit >= -8)))
      {
        cvalue = cvalue * base;
        cvalue += digit;
      }
      else
      {
        IB_LOG_ERROR0 ("string value is too small");
        IB_EXIT (function, (uint32_t)return_error);
        return return_error;
      }
    }
    else
    {
      if ((cvalue < 922337203685477580ll) ||
          ((cvalue == 922337203685477580ll) && (digit <= 7)))
      {
        cvalue = cvalue * base;
        cvalue += digit;
      }
      else
      {
        IB_LOG_ERROR0 ("string value is too large");
        IB_EXIT (function, (uint32_t)return_error);
        return return_error;
      }
    }
    nptr++;
  }

  IB_EXIT (function, (uint32_t)cvalue);
  return cvalue;
}

//==============================================================================//
//
// FUNCTION
//   cs_parse_gid
//
// DESCRIPTION
//   Converts an asci representation of a GID in hex of the format:
//     (0[xX])?[[:hex:]]{32}
//   into a Gid_t. Returns VSTATUS_OK upon success.
//
// INPUTS
//    str - the string to parse
//    gid - the gid in which to store the value
//
// OUTPUTS
//
//
// HISTORY
//
//    NAME	DATE  REMARKS
//
//==============================================================================//
Status_t
cs_parse_gid(const char * str, Gid_t gid)
{
	uint8_t * tmp = (uint8_t *) gid;
	int i = 0;
	const char * myPtr = NULL;
	char myNum[3];
	Status_t rc = VSTATUS_OK;

	IB_ENTER(__FUNCTION__, str, gid, 0, 0);

	if (  (str[0] == '0')
	   && ( (str[1] == 'x') || (str[1] == 'X') ) )
		myPtr = str + 2;
	else
		myPtr = str;

	if (strlen(myPtr) == 32)
	{
		for (i = 0; i < 16; ++i)
		{
			memcpy(myNum, myPtr + (i << 1), 2);
			myNum[2] = '\0';

			tmp[i] = strtoul(myNum, NULL, 16);
		}
	} else
	{
		rc = VSTATUS_ILLPARM;
	}

	IB_EXIT(__FUNCTION__, rc);
	return rc;
}
