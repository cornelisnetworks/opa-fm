/* $Id: util.c,v 1.3 2013/10/01 01:46:29 sbreyer Exp $ */
/* ----------------------------------------------------------------------- *
 *   
 *   Copyright 2001 H. Peter Anvin - All Rights Reserved
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version; incorporated herein by reference.
 *
 * ----------------------------------------------------------------------- */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "mkzftree.h"

/* Convenience functions */
void *xmalloc(size_t size)
{
  void *p = malloc(size);

  if ( !p ) {
    perror(program);
    exit(EX_OSERR);
  }

  return p;
}

char *xstrdup(const char *str)
{
  char *s = strdup(str);

  if ( !s ) {
    perror(program);
    exit(EX_OSERR);
  }
  
  return s;
}

void message(enum verbosity level, const char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  if ( opt.verbosity >= level )
    vfprintf(stderr, format, ap);
  va_end(ap);
}

