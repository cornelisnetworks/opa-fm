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
# Environment variable: BUILD_ISO=y will enable generation of ISO Images,
# Any other value disables hence saving disk space for developer builds
# If BUILD_ISO is not defined, it defaults based on RELEASE_TAG
#	y - if release tag starts with R (eg. an offical build)
#	n - if release tag does not start with R (eg. a developer build)

MKZFTREE=${MKZFTREE:-$TL_DIR/MakeTools/zisofs-tools/mkzftree}
PATCH_BRAND=${PATCH_BRAND:-$TL_DIR/MakeTools/patch_version/patch_brand}

Usage()
{
	echo "Usage: makeiso.sh [-z] [-l 'label'] file.iso dir" >&2
	echo "    default label is dir name" >&2
	echo "    -z - create compressed CD" >&2
	exit 2
}

label=
opts=
zflag=n
while getopts zl: param
do
	case $param in
	l)
		label="$OPTARG";;
	z)
		zflag=y
		opts="$opts -z";;
	?)
		Usage;;
	esac
done
shift $((OPTIND -1))

if [ $# -ne 2 ]
then
	Usage
fi

if [ x"$BUILD_ISO" = x ]
then
	# default based on RELEASE_TAG
	if [ x`echo $RELEASE_TAG|cut -c1` = xR ]
	then
		BUILD_ISO=y
	else
		BUILD_ISO=n
	fi
fi
if [ x"$BUILD_ISO" != xy ]
then
	echo "Iso Image Generation Disabled"
	echo "export BUILD_ISO=y to enable"
	exit 0
fi

iso=$1
dir=$2

if [ ! -d $dir ]
then
	echo "makeiso.sh: $dir is not a directory" >&2
	Usage
fi

if [ -z "$label" ]
then
	label="$dir"
fi

if [ "$zflag" = y ]
then
	echo "Compressing $dir into $dir.comp..."
	$MKZFTREE -p 5 $dir $dir.comp
	srcdir=$dir.comp
else
	srcdir=$dir
fi
echo "Creating $iso using $srcdir, Label is $label..."

# TBD -copyright file
# TBD -abstract file
author=`$PATCH_BRAND "$BUILD_BRAND"`
mkisofs $opts -A "$label" -p "$author" -P "$author" -sysid "$label" -V "$label" -volset "$label" -volset-size 1 -volset-seqno 1 -o $iso -r -L $srcdir

if [ "$zflag" = y ]
then
	rm -rf $dir.comp
fi
