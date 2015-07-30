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
## expand_source
## -----------
## expand the SRPMs found in a SRPMS source tree into their component files
## designed to work with OFED packagings.  build_srpms can rebuild the srpms.
##
## Usage:
##	expand_source [-a] [-s source_dir] [-d dest_dir] [-p] [-L] [-V kernel_ver] [srpm ...]
##
## Arguments:
##	-s - source directory to expand, default is ., ignored if srpm(s) specified
##	-a - expand all of OFED, -s is OFED install tree
##	-d - destination for expansion, default is ./SOURCE_TREE
##	-p - apply patches for ofa_kernel
##	-V - kernel version to apply patches for, default is `uname -r`
##	-L - clean any symlinks
##  srpm - srpm file to expand
## Environment:

#. $ICSBIN/funcs.sh

Usage()
{
	# include "ERROR" in message so weeklybuild catches it in error log
	echo "ERROR: expand_source failed" >&2
	echo "Usage: expand_source [-a] [-s source_dir] [-d dest_dir] [-p] [-L] [-V kernel_ver] [srpm ...]" >&2
	echo "    -s - source directory to expand, default is ., ignored if srpm(s) specified" >&2
	echo "    -a - expand all of OFED, -s is OFED install tree" >&2
	echo "    -d - destination for expansion, default is ./SOURCE_TREE" >&2
	echo "    -p - apply patches for ofa_kernel" >&2
	echo "    -V - kernel version to apply patches for, default is `uname -r`" >&2
	echo "    -v - verbose output" >&2
	echo "    -L - clean any symlinks" >&2
	echo "    srpm - srpm file to expand" >&2
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

build_make_symlinks()
{
	# $1 = directory to check
	# $2 = make_symlinks file to create or append to
	dir=$1
	outfile=$2
	# outputs make_symlinks contents
	(
		cd $dir
		# file outputs:
		#     ./makefile:  symbolic link to `ofed_scripts/makefile'
		find . -type l|xargs -r file|sed -e 's/:[ 	]*symbolic link to `/ /' -e 's/:[ 	]*broken symbolic link to `/ /' -e "s/'\$//"|sort|while read dest source
		do
			subdir=$(dirname $dest)
			if [ "$subdir" = "." ]
			then
				echo "[ -h $dest ] || ln -s $source $dest"
			else
				echo "[ -h $dest ] || { mkdir -p $subdir && ln -s $source $dest;  }"
			fi
		done
	) >> $outfile
	chmod +x $outfile
}


build_flags=
patch_kernel=n
kernel_ver=`uname -r`
source_dir=$PWD
dest_dir=$PWD/SOURCE_TREE
vopt=""
aflag=n
cleansym=n
while getopts as:d:pLV:v param
do
	case $param in
	a)	aflag=y;;
	s)	source_dir="$OPTARG";;
	d)	dest_dir="$OPTARG";;
	p)	patch_kernel=y;;
	L)	cleansym=y;;
	V)	kernel_ver="$OPTARG";;
	v)	vopt="-v";;
	*)	Usage;;
	esac
done
shift $(($OPTIND -1))

unset IFS # use default so enable word expansion

dest_dir=$(fix_dir $dest_dir)
source_dir=$(fix_dir $source_dir)

mkdir -p $dest_dir


gettaropt()
{
	# $1 is tarfile name
	case $1 in
	*.tgz)	taropt=z;;
	*.tar.gz)	taropt=z;;
	*.tar.bz2)	taropt=j;;
	*)	echo "Unknown tarfile format: $tarfile"; exit 1
	esac
}

querysrpm()
{
	# $1 = src.rpm file
	package=$(rpm -q --queryformat '[%{NAME}\n]' -p $1)
	[ x"$package" != x ] || exit 1
	# OFED has 2 versions of open-iscsi-generic.  Separate out the RH4 version
	if [ $(basename $1 .src.rpm) = "open-iscsi-generic-2.0-754.1" ]
	then
		package="open-iscsi-generic-rh4"
	fi
	specfile=$(rpm -q --queryformat '[%{FILENAMES}\n]' -p $1|grep '.spec$')
	[ x"$specfile" != x ] || exit 1
	tarfile=$(rpm -q --queryformat '[%{FILENAMES}\n]' -p $1|egrep '.tgz$|.tar.gz$|.tar.bz2$')
	[ x"$tarfile" != x ] || exit 1
	otherfiles=$(rpm -q --queryformat '[%{FILENAMES}\n]' -p $1|egrep -v '.spec$|.tgz$|.tar.gz$|.tar.bz2$'|tr '\n' ' ')

	gettaropt $tarfile
}

copy_ofed_misc()
{
	# source_dir is a OFED install tree
	package=ofed_misc
	(
	rm -rf $dest_dir/$package
	mkdir $dest_dir/$package
	cd $source_dir
	#cp -r docs BUILD_ID LICENSE README.txt install.pl uninstall.sh $dest_dir/$package
	find . -type f ! -name '*.src.rpm'| sed -e 's|^\./||'|sort > $dest_dir/$package/FILELIST
	build_make_symlinks . $dest_dir/$package/make_symlinks
	cat $dest_dir/$package/FILELIST|cpio -pdv $dest_dir/$package
	)
	[ -d $dest_dir/$package ] || exit 1
}

if [ $# -eq 0 ]
then
	if [ "$aflag" = y ]
	then
		copy_ofed_misc
		srpms_dir=$source_dir/SRPMS
		rm -f $dest_dir/package_list
	else
		srpms_dir=$source_dir
	fi

	echo "Expanding SRPMS in $srpms_dir into $dest_dir..."

	cd $srpms_dir
	set $(echo *.src.rpm)
	if [ x"$1" = x'*src.rpm' ]
	then
		echo "expand_source: No SRPMS found in $srpms_dir" >&2
		exit 2
	fi
fi

for srpm in "$@"
do
	if [ ! -e $srpm ]
	then
		echo "expand_source: $srpm: Not Found" >&2
		exit 1
	fi

	echo "Expanding $srpm into $dest_dir..."
	querysrpm $srpm
	srpm_basename=$(basename $srpm)

	(
		# OFED has some packages with the same .spec filename (eg. dapl and
		# compat-dapl), so we need to keep each package completely separated
		rm -rf $dest_dir/$package
		mkdir $dest_dir/$package
		rpm -ivh --define "_topdir $dest_dir/$package" $srpm
		if [ ! -d $dest_dir/$package ]
		then
			echo "expand_source: unable to install $srpm to $dest_dir/$package" >&2
			exit 1
		fi
		cd $dest_dir/$package

		cd SOURCES
		# assume tarfile creates one srcdir
		srcdir=$(tar tf$taropt $tarfile|sed -e 's/\/.*//'|sort -u)
		rm -rf $srcdir
		tar xf$taropt $tarfile $vopt
		if [ $? -ne 0 -o ! -d "$srcdir" ]
		then
			echo "expand_source: $srpm: Failed to untar $tarfile into $srcdir"
			exit 1
		fi
		rm -f $tarfile
		mv $srcdir ../src
		if [ x"$otherfiles" != x ]
		then
			mv $otherfiles ..
		fi
		cd ..

		mv SPECS/$specfile .
		rmdir SOURCES SPECS

		# add $package to package list
		if [ -f $dest_dir/package_list ]
		then
			echo "$package"|cat - $dest_dir/package_list|sort -u > $dest_dir/.temp
			mv $dest_dir/.temp $dest_dir/package_list
		else
			echo "$package" > $dest_dir/package_list
		fi

		echo "#package	srpm	specfile	srcdir	tarfile	otherfiles" > $dest_dir/$package/namemap
		echo "$package	$srpm_basename	$specfile	$srcdir	$tarfile	$otherfiles" >> $dest_dir/$package/namemap

		# the source control system will not store symlinks
		# so make a file which can re-create symbolic links.
		echo "cd src" > $dest_dir/$package/make_symlinks
		build_make_symlinks $dest_dir/$package/src $dest_dir/$package/make_symlinks

		# optional patching of ofa_kernel
		case $srpm in
		*ofa_kernel*)
			if [ $patch_kernel = y ]
			then
				echo "Patching ofa_kernel for $kernel_ver..."
				cd src
					ofed_scripts/ofed_patch.sh --kernel-version=$kernel_ver
					rm -rf kernel_patches */patches
				cd ..
			fi
			;;
		esac
		if [ $cleansym = y ]
		then
			find . -type l|xargs rm -f
		fi
		exit 0
	)
	[ $? -ne 0 ] && exit 1
done
exit 0
