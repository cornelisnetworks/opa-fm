#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015, Intel Corporation
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor the names of its contributors
#       may be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# END_ICS_COPYRIGHT8   ****************************************
##
## build_srpms
## -----------
## create SRPMS from an expanded source tree
## designed to work with trees built by expand_source.  Supports OFED source.
##
## Usage:
##	build_srpms [-c] [-d SRPMS_dir] [-s source_dir] [-p] [-V kernel_ver] [package...]
##
## Arguments:
##	-c - cleanup SOURCES, SPECS and SRPMS in each package when done
##	-d - SRPMS directory to create, default is ./SRPMS
##	-s - source for build, default is ., ignored if package(s) specified
##	-p - unapply patches for ofa_kernel
##	-V - kernel version to unapply patches for, default is `uname -r`
##  package - a package source directory
## Environment:

#. $ICSBIN/funcs.sh

Usage()
{
	# include "ERROR" in message so weeklybuild catches it in error log
	echo "ERROR: build_srpms failed" >&2
	echo "Usage: build_srpms [-c] [-d SRPMS_dir] [-s source_dir] [-p] [-V kernel_ver] [package...]" >&2
	echo "    -c - cleanup SOURCES, SPECS and SRPMS in each package when done" >&2
	echo "    -d - SRPMS directory to create, default is ./SRPMS" >&2
	echo "    -s - source for build, default is ., ignored if package(s) specified" >&2
	echo "    -p - unapply patches for ofa_kernel" >&2
	echo "    -V - kernel version to unapply patches for, default is `uname -r`" >&2
	echo "    -v - verbose output" >&2
	echo "    package - a package source directory" >&2
	exit 2
}

# convert $1 to an absolute path
fix_dir()
{
	if [ x$(echo $1|cut -c1) != x"/" ]
	then
		echo $PWD/$1
	else
		echo $1
	fi
}

build_flags=
patch_kernel=n
kernel_ver=`uname -r`
srpms_dir=$PWD/SRPMS
source_dir=$PWD
vopt=""
packages=
clean=n
while getopts cs:d:pV:v param
do
	case $param in
	c)	clean=y;;
	d)	srpms_dir="$OPTARG";;
	s)	source_dir="$OPTARG";;
	p)	patch_kernel=y;;
	V)	kernel_ver="$OPTARG";;
	v)	vopt="-v";;
	*)	Usage;;
	esac
done
shift $(($OPTIND -1))

unset IFS # use default so enable word expansion

source_dir=$(fix_dir $source_dir)
srpms_dir=$(fix_dir $srpms_dir)

echo "Building SRPMS in $srpms_dir from $source_dir..."

mkdir -p $srpms_dir

gettaropt()
{
	case $1 in
	*.tgz)	taropt=z;;
	*.tar.gz)	taropt=z;;
	*.tar.bz2)	taropt=j;;
	*)	echo "Unknown tarfile format: $tarfile"; exit 1
	esac
}

checkfiles()
{
	local ret

	ret=0
	if [ ! -f SPECS/$specfile ]
	then
		echo "$srpm: missing file: SPECS/$specfile"
		ret=1
	fi
	for f in $tarfile $otherfiles
	do
		if [ ! -f SOURCES/$f ]
		then
			echo "$srpm: missing file: SOURCES/$f"
			ret=1
		fi
	done
	return $ret
}

if [ $# -eq 0 ]
then
	#if [ ! -f package_list ]
	#then
	#	echo "ERROR: build_srpms: package_list not found in $source_dir" >&2
	#	echo "ERROR: build_srpms: source_dir must be created by expand_source" >&2
	#	exit 1
	#fi

	#set -- $(grep -v '^#' package_list)
	#set -- $(cat package_list)

	cd $source_dir
	set -- $(ls */namemap|cut -f1 -d /)

	if [ $# -eq 0 ]
	then
		echo "build_srpms: ERROR: No packages found in $source_dir" >&2
		exit 1
	fi
fi

for package in "$@"
do
	(
	if [ ! -f $package/namemap -o ! -f $package/make_symlinks ]
	then
		echo "ERROR: build_srpms: namemap not found in $source_dir/$package" >&2
		echo "ERROR: build_srpms: source_dir must be created by expand_source" >&2
		exit 1
	fi
	cd $package
	bash -x ./make_symlinks

	# tail is just to be paranoid, should only be 1 non-comment line
	grep -v "^#" namemap|tail -1|while read package srpm specfile srcdir tarfile otherfiles
	do
		echo "Generating $srpm from $package..."

		gettaropt $tarfile

		case $srpm in
		*ofa_kernel*)
				if [ $patch_kernel = y ]
				then
					echo "Un-Patching ofa_kernel for $kernel_ver..."
					(
						cd src
						# TBD - undo ofed_scripts/ofed_patch.sh
						# need an unapply option or a separate unpatch script
						echo "build_srpms: ERROR: -p option not implemented" >&2; exit 1
					)
				fi
				;;
		esac

		rm -rf SOURCES SPECS SRPMS
		mkdir SOURCES SPECS SRPMS

		# copy all the source files to the proper directory name
		# omit CVS directories
		# Note we can't symlink because we need tar below to not follow symlinks
		(
			mkdir SOURCES/$srcdir;
			cd src; find . -name CVS -prune -o -name '*~' -prune -o -print|cpio -pdum ../SOURCES/$srcdir
			if [ $? != 0 -o ! -e ../SOURCES/$srcdir ]
			then
				echo "$srpm: ERROR: unable to create: $package/SOURCES/$srcdir" >&2
				exit 1
			fi
		)
		[ $? -eq 0 ] || exit 1

		# create the final tarfile which will be included in the srpm
		(
			cd SOURCES
			# set timestamp order for files which are typically pregenerated
			# this avoids the need to upgrade to autoconf 2.60 on older distros
			find $srcdir -name 'configure.in'|xargs -r touch
			sleep 1
			find $srcdir -name '*.m4'|xargs -r touch
			sleep 1
			find $srcdir -name 'configure'|xargs -r touch
			sleep 1
			find $srcdir -name 'config.status'|xargs -r touch
			sleep 1
			find $srcdir -name 'Makefile.in'|xargs -r touch
			sleep 1
			find $srcdir -name '*h.in'|xargs -r touch
			sleep 1

			tar cf$taropt $tarfile $vopt $srcdir
			if [ $? != 0 -o ! -e $tarfile ]
			then
				echo "$srpm: ERROR: unable to create: $package/SOURCES/$tarfile" >&2
				exit 1
			fi
			rm -rf $srcdir
		)
		[ $? -eq 0 ] || exit 1

		if [ x"$otherfiles" != x ]
		then
			cp $otherfiles SOURCES/
			if [ $? != 0 ]
			then
				echo "$srpm: ERROR: unable to copy: $otherfiles to $package/SOURCES" >&2
				exit 1
			fi
		fi

		cp $specfile SPECS/$specfile
		if [ $? != 0 -o ! -e SPECS/$specfile ]
		then
			echo "$srpm: ERROR: unable to create: $package/SPECS/$specfile" >&2
			exit 1
		fi

		checkfiles || exit 1
		rpmbuild --nodeps -bs --define "_topdir `pwd`" SPECS/$specfile
		if [ $? != 0 -o ! -e SRPMS/$srpm ]
		then
			echo "$srpm: ERROR: unable to create: $package/SRPMS/$srpm" >&2
			exit 1
		fi
		cp SRPMS/$srpm $srpms_dir
		echo "Built $package/SRPMS/$srpm"
		if [ $? != 0 -o ! -e $srpms_dir/$srpm ]
		then
			echo "$srpm: ERROR: unable to create: $srpms_dir/$srpm" >&2
			exit 1
		fi
		if [ "$clean" = y ]
		then
			rm -rf SOURCES SPECS SRPMS
		fi
		echo "Copied $package/SRPMS/$srpm to $srpms_dir/$srpm"
	done
	)
	if [ $? -ne 0 ]
	then
		echo "build_srpms: ERROR building $package" >&2
		exit 1
	fi
done
