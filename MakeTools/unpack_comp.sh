#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
#
# Copyright (c) 2015-2020, Intel Corporation
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
## unpack_comp
## -----------
## copy a component previously downloaded by get_comp to the stage area
## the component is processed based on the files identified by
## the WrappedProducts configuration script.
##
## Usage:
##	unpack_comp component bundle downloadsdir tempdir
## Arguments:
##	component - one of OFED, TRUESCALE
##	bundle - ALL is only supported value for now
##	downloadsdir - intermediate local directory 
##		component files were be downloaded into
##		actual directory per component will be of the form:
##			downloadsdir/component-bundle
##	tempdir - stage sub-directory to copy component to
## Required Environment (as set via setenv or similar tools):
##	TL_DIR
## 	BUILD_PLATFORM_OS_VENDOR_VERSION
##	BUILD_TARGET
##	BUILD_TARGET_OS_VENDOR
## Additional Environment:
##

. $ICSBIN/funcs.sh

Usage()
{
	# include "ERROR" in message so weeklybuild catches it in error log
	echo "ERROR: unpack_comp failed" >&2
	echo "Usage: unpack_comp component bundle downloadsdir tempdir" >&2
	exit 2
}

if [ $# != 4 ]
then
	Usage
fi

# $1 = Component Product Name
# $2 = Packaging Bundle
component=$1
bundle=$2
downloadsdir=$3
tempdir=$4

. $TL_DIR/$PROJ_FILE_DIR/WrappedProducts

retcode=0

localdir=$downloadsdir/$component-$bundle
files=$(cat $localdir/filelist)
dir=""
# 'STAGE COMPONENTS ' keyword looked for by build_relnotes
# put $files outside quotes so shell translates newlines to spaces
echo "STAGE COMPONENTS $component $bundle Files:" $files
for file in $files
do
	if [ "$file" != "NONE" ]
	then
		localfile=$localdir/$(basename $file)
		if [ ! -e $localfile ]
		then
			echo "ERROR - $localfile: Not Found"
			retcode=1
			continue
		fi
		case $localfile in
		*.tgz)
			dir=$(basename $localfile .tgz)
			(cd $tempdir; tar xvfz $localfile) ;;
		*)
			echo "ERROR: unable to process $component file: $file"
			retcode=1
			continue
		esac
	fi
done

if [ "$dir" = "" ]
then
	echo "Warning: Skipping $component for $bundle: No files"
elif [ $retcode = 0 ]
then
	# now process the component by installing or extracting what we need
	$TL_DIR/$PROJ_FILE_DIR/extract_comp $component $bundle $tempdir/$dir
	if [ $? != 0 ]
	then
		echo "ERROR: unable to extract $component for $bundle"
		retcode=1;
	fi
fi

exit $retcode
