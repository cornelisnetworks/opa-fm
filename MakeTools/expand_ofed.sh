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
## expand_ofed
## -----------
## expand an OFED tgz in preparation for source control in OFED vendor branch
##
## Usage:
##	expand_ofed [-d dest_dir] [-c compare_dir] ofed.tgz
##
## Arguments:
##	-d - destination for expansion, default is ./SOURCE_TREE
##	-c - optional expanded ofed to compare to
##  ofed.tgz - ofed tgz to expand
## Environment:

tempdir=/usr/tmp/expand$$

# handle various places we could find our tools
if [ -e ./expand_source.sh ]
then
	expand_source=$PWD/expand_source.sh
#elif [ -e $TL_DIR/MakeTools/expand_source.sh ]
#then
#	expand_source=$TL_DIR/MakeTools/expand_source.sh
else
	expand_source=expand_source
fi
if ! type $expand_source
then
	exit 1
fi

Usage()
{
	echo "Usage: expand_ofed [-d dest_dir] [-c compare_dir] ofed.tgz" >&2
	echo "    -d - destination for expansion, default is ./SOURCE_TREE" >&2
	echo "    -c - optional expanded ofed to compare to" >&2
	echo "    ofed.tgz - ofed tgz to expand" >&2
	exit 2
}

# convert $1 to an absolute path
fix_dir()
{
	local dir
	if [ x$(echo $1|cut -c1) != x"/" ]
	then
		dir="$PWD/$1"
	else
		dir="$1"
	fi
	# drop trailing /
	if [ "$dir" != "/" ]
	then
		echo "$dir"|sed -e 's|/*$||'
	else
		echo "$dir"
	fi
}

dest_dir=$PWD/SOURCE_TREE
compare_dir=
while getopts d:c: param
do
	case $param in
	d)	dest_dir="$OPTARG";;
	c)	compare_dir="$OPTARG";;
	*)	Usage;;
	esac
done
shift $(($OPTIND -1))
if [ $# -ne 1 ]
then
	Usage
fi
ofed_tgz=$1

if [ ! -f $ofed_tgz ]
then
	echo "expand_ofed: Can't find: $ofed_tgz" >&2
	exit 1
fi
dest_dir=$(fix_dir $dest_dir)
ofed_tgz=$(fix_dir $ofed_tgz)

mkdir -p $dest_dir

# assume tarfile creates one dir
ofed_dir=$(tar tfz $ofed_tgz|sed -e 's/\/.*//'|sort -u)
if [ x"$ofed_dir" = x ]
then
	echo "expand_ofed: Can't figure out top dir of $ofed_tgz" >&2
	exit 1
fi

mkdir -p $tempdir
tar xvfz $ofed_tgz -C $tempdir
if [ $? -ne 0 -o ! -d $tempdir/$ofed_dir ]
then
	echo "expand_ofed: unable to untar $ofed_tgz" >&2
	exit 1
fi
$expand_source -a -s $tempdir/$ofed_dir -d $dest_dir
if [ $? -ne 0 ]
then
	echo "expand_ofed: expand_source failed" >&2
	exit 1
fi
rm -rf $tempdir

if [ x"$compare_dir" != x ]
then

	echo "Comparing $compare_dir and $dest_dir..."
	compare_dir=$(fix_dir $compare_dir)
	diff -r --exclude CVS $compare_dir $dest_dir > ./diffs

	# list of new files in drop relative to $compare_dir drop
	grep "^Only in $dest_dir[:/]" < ./diffs|sed -e 's/^Only in //' -e 's/: /\//' -e "s|$dest_dir|.|" > ./added_files

	# list of files moved/removed in drop relative to $compare_dir drop
	grep "^Only in $compare_dir[:/]" < ./diffs|grep -v ': CVS'|sed -e 's/^Only in //' -e 's/: /\//' -e "s|$compare_dir|.|" > ./removed_files

	# summary
	echo "New files in this drop is in ./added_files: $(wc -l < ./added_files) files"
	echo "Files removed in this drop is in ./removed_files: $(wc -l < ./removed_files) files"
	echo "All diffs are in ./diffs: $(wc -l < ./diffs) lines"
fi

exit 0
