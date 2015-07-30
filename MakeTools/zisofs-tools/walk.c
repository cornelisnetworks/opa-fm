#ident "$Id: walk.c,v 1.4 2014/12/17 18:50:18 mwheinz Exp $"
/* ----------------------------------------------------------------------- *
 *   
 *   Copyright 2001-2002 H. Peter Anvin - All Rights Reserved
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version; incorporated herein by reference.
 *
 * ----------------------------------------------------------------------- */

/*
 * walk.c
 *
 * Functions to walk the file tree
 */

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <utime.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mkzftree.h"
#include "iso9660.h"

static int munge_file(const char *inpath, const char *outpath,
		      const char *cribpath, struct stat *st)
{
  FILE *in, *out;
  int err = 0;
  struct utimbuf ut;

  if ( cribpath ) {
    struct stat cst;
    struct compressed_file_header cfh;

    /* Compare as much as we realistically can */
    if ( !stat(cribpath, &cst) &&
	 st->st_mode == cst.st_mode &&
	 st->st_uid == cst.st_uid &&
	 st->st_gid == cst.st_gid &&
	 st->st_mtime == cst.st_mtime ) {
      if ( (in = fopen(cribpath, "rb")) ) {
	int e = fread(&cfh, 1, sizeof cfh, in);
	fclose(in);
	/* Attempt to restore the atime */
	ut.actime  = cst.st_atime;
	ut.modtime = cst.st_mtime;
	utime(cribpath, &ut);

	if ( (e == sizeof cfh &&
	      !memcmp(cfh.magic, zisofs_magic, sizeof zisofs_magic) &&
	      (off_t)get_731(cfh.uncompressed_len) == st->st_size) ||
	     (st->st_size == cst.st_size &&
	      (e < (int)(sizeof zisofs_magic) ||
	       memcmp(cfh.magic, zisofs_magic, sizeof zisofs_magic))) ) {
	  /* File is cribbable.  Steal it. */
	  if ( !link(cribpath, outpath) ) {
	    message(vl_crib, "crib: %s -> %s\n", cribpath, outpath);
	    ut.actime  = st->st_atime;
	    ut.modtime = st->st_mtime;
	    utime(outpath, &ut);	/* Set the the atime */
	    
	    return 0;
	  }
	}
      }
    }
  }

  in = fopen(inpath, "rb");
  if ( !in )
    return EX_NOINPUT;
  out = fopen(outpath, "wb");
  if ( !out ) {
    fclose(in);
    return EX_CANTCREAT;
  }

  if ( spawn_worker() ) {
    err = opt.munger(in, out, st->st_size);
    fclose(in);
    fclose(out);
    
    chown(outpath, st->st_uid, st->st_gid);
    chmod(outpath, st->st_mode);
    ut.actime  = st->st_atime;
    ut.modtime = st->st_mtime;
    utime(outpath, &ut);
    
    end_worker(err);
  } else {
    fclose(in);
    fclose(out);
  }

  return err;
}

int munge_tree(const char *intree, const char *outtree, const char *cribtree)
{
  char *in_path = NULL, *out_path = NULL, *crib_path = NULL;
  char *in_file = NULL, *out_file = NULL, *crib_file = NULL;
  DIR *thisdir;
  struct dirent *dirent;
  struct stat dirst;
  int err = 0;
  
  /* Construct buffers with the common filename prefix, and point to the end */

  in_path = xmalloc(strlen(intree) + NAME_MAX + 2);
  strcpy(in_path, intree);
  in_file = strchr(in_path, '\0');
  *in_file++ = '/';

  out_path = xmalloc(strlen(outtree) + NAME_MAX + 2);
  strcpy(out_path, outtree);
  out_file = strchr(out_path, '\0');
  *out_file++ = '/';

  if ( cribtree ) {
    crib_path = xmalloc(strlen(cribtree) + NAME_MAX + 2);
    strcpy(crib_path, cribtree);
    crib_file = strchr(crib_path, '\0');
    *crib_file++ = '/';
  } else {
    crib_path = crib_file = NULL;
  }

  /* Get directory information */
  if ( stat(intree, &dirst) ) {
    message(vl_error, "%s: Failed to stat directory %s: %s\n",
	    program, intree, strerror(errno));
	if (crib_path) free(crib_path);
	if (in_path) free(in_path);
	if (out_path) free(out_path);
    return EX_NOINPUT;
  }

  /* Open the directory */
  thisdir = opendir(intree);
  if ( !thisdir ) {
    message(vl_error, "%s: Failed to open directory %s: %s\n",
	    program, intree, strerror(errno));
	if (crib_path) free(crib_path);
	if (in_path) free(in_path);
	if (out_path) free(out_path);
    return EX_NOINPUT;
  }

  /* Create output directory */
  if ( mkdir(outtree, 0700) ) {
    message(vl_error, "%s: Cannot create output directory %s: %s\n",
	    program, outtree, strerror(errno));
	if (crib_path) free(crib_path);
	if (in_path) free(in_path);
	if (out_path) free(out_path);
    return EX_CANTCREAT;
  }

  while ( (dirent = readdir(thisdir)) != NULL ) {
    if ( !strcmp(dirent->d_name, ".") ||
	 !strcmp(dirent->d_name, "..") )
      continue;			/* Ignore . and .. */
    
    strcpy(in_file, dirent->d_name);
    strcpy(out_file, dirent->d_name);
    if ( crib_file )
      strcpy(crib_file, dirent->d_name);
    
    err = munge_entry(in_path, out_path, crib_path, &dirst);
    if ( err )
      break;
  }
  closedir(thisdir);
  
  if (crib_path) free(crib_path);
  if (in_path) free(in_path);
  if (out_path) free(out_path);
  
  return err;
}


int munge_entry(const char *in_path, const char *out_path,
		const char *crib_path, const struct stat *dirst)
{
  struct stat st;
  struct utimbuf ut;
  int err = 0;

  message(vl_filename, "%s -> %s\n", in_path, out_path);

  if ( lstat(in_path, &st) ) {
    message(vl_error, "%s: Failed to stat file %s: %s\n",
	    program, in_path, strerror(errno));
    return EX_NOINPUT;
  }
  
  if ( S_ISREG(st.st_mode) ) {
    if ( st.st_nlink > 1 ) {
      /* Hard link. */
      const char *linkname;
      
      if ( (linkname = hash_find_file(&st)) != NULL ) {
	/* We've seen it before, hard link it */
	
	if ( link(linkname, out_path) ) {
	  message(vl_error, "%s: hard link %s -> %s failed: %s\n",
		  program, out_path, linkname, strerror(errno));
	  return EX_CANTCREAT;
	}
      } else {
	/* First encounter, compress and enter into hash */
	if ( (err = munge_file(in_path, out_path, crib_path, &st)) != 0 ) {
	  message(vl_error, "%s: %s: %s", program, in_path, strerror(errno));
	  return err;
	}
	hash_insert_file(&st, out_path);
      }
    } else {
      /* Singleton file; no funnies */
      if ( (err = munge_file(in_path, out_path, crib_path, &st)) != 0 ) {
	message(vl_error, "%s: %s: %s", program, in_path, strerror(errno));
	return err;
      }
    }
  } else if ( S_ISDIR(st.st_mode) ) {
    /* Recursion: see recursion */
    if ( !opt.onedir &&
	 (!opt.onefs || (dirst && dirst->st_dev == st.st_dev)) ) {
      if ( (err = munge_tree(in_path, out_path, crib_path)) != 0 )
	return err;
    } else if ( opt.do_mkdir ) {
      /* Create stub directories */
      if ( mkdir(out_path, st.st_mode) ) {
	message(vl_error, "%s: %s: %s", program, out_path, strerror(errno));
	return EX_CANTCREAT;
      }
    }
  } else if ( S_ISLNK(st.st_mode) ) {
    int chars;
#ifdef PATH_MAX
#define BUFFER_SLACK PATH_MAX
#else
#define BUFFER_SLACK BUFSIZ
#endif
    int buffer_len = st.st_size + BUFFER_SLACK + 1;
    char *buffer = xmalloc(buffer_len);
    if (!buffer) {
      message(vl_error, "%s: xmalloc failed for %s: %d\n",
	      program, buffer_len, strerror(errno));
      return EX_CANTCREAT;
    }
    if ( (chars = readlink(in_path, buffer, buffer_len)) < 0 ) {
      message(vl_error, "%s: readlink failed for %s: %s\n",
	      program, in_path, strerror(errno));
      free(buffer);
      return EX_NOINPUT;
    }
    buffer[chars] = '\0';
    if ( symlink(buffer, out_path) ) {
      message(vl_error, "%s: symlink %s -> %s failed: %s\n",
	      program, out_path, buffer, strerror(errno));
      free(buffer);
      return EX_CANTCREAT;
    }
    free(buffer);
  } else {
    if ( st.st_nlink > 1 ) {
      /* Hard link. */
      const char *linkname;
      
      if ( (linkname = hash_find_file(&st)) != NULL ) {
	/* We've seen it before, hard link it */
	
	if ( link(linkname, out_path) ) {
	  message(vl_error, "%s: hard link %s -> %s failed: %s\n",
		  program, out_path, linkname, strerror(errno));
	  return EX_CANTCREAT;
	}
      } else {
	/* First encounter, create and enter into hash */
	if ( mknod(out_path, st.st_mode, st.st_rdev) ) {
	  message(vl_error, "%s: mknod failed for %s: %s\n",
		  program, out_path, strerror(errno));
	  return EX_CANTCREAT;
	}
	hash_insert_file(&st, out_path);
      }
    } else {
      /* Singleton node; no funnies */
      if ( mknod(out_path, st.st_mode, st.st_rdev) ) {
	message(vl_error, "%s: mknod failed for %s: %s\n",
		program, out_path, strerror(errno));
	return EX_CANTCREAT;
      }
    }
  }
  
  /* This is done by munge_file() for files */
  if ( !S_ISREG(st.st_mode) ) {
#ifdef HAVE_LCHOWN
    if ( lchown(out_path, st.st_uid, st.st_gid) && opt.sloppy && !err ) {
      message(vl_error, "%s: %s: %s", program, out_path, strerror(errno));
      err = EX_CANTCREAT;
    }
#endif
    if ( !S_ISLNK(st.st_mode) ) {
#ifndef HAVE_LCHOWN
      if ( chown(out_path, st.st_uid, st.st_gid) && !opt.sloppy && !err ) {
	message(vl_error, "%s: %s: %s", program, out_path, strerror(errno));
	err = EX_CANTCREAT;
      }
#endif
      if ( chmod(out_path, st.st_mode) && !opt.sloppy && !err ) {
	message(vl_error, "%s: %s: %s", program, out_path, strerror(errno));
	err = EX_CANTCREAT;
      }
      ut.actime  = st.st_atime;
      ut.modtime = st.st_mtime;
      if ( utime(out_path, &ut) && !opt.sloppy && !err ) {
	message(vl_error, "%s: %s: %s", program, out_path, strerror(errno));
	err = EX_CANTCREAT;
      }
    }
  }

  return err;
}

