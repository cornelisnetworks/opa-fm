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
# This script is similar to the ar command, however it handles .a arguments
# as well as .o arguments.  It is used when LIBFILES != "" (ie. for module
# makefiles) to assist in combining multiple archice files into one .a file
#
# This is coded for use in VxWorks, Cygwin and Linux builds.  All of which
# use variations of ar and use .o and .a suffixes.
#
# This is not used for BUILD_CONFIG=loadable, where .rel files are created

Usage()
{
	echo "Usage: mkarlib ar_tool ar_options archive file ..." >&2
	exit 2
}

if [ $# -lt 4 ]
then
	Usage
fi

tempDir=tmp$$
cwd=`pwd`
trap 'rm -rf $cwd/$tempDir' 0 1 2 9 15

ar=$1
options=$2
archive=$3
shift
shift
shift
rm -f $archive
rm -rf $tempDir
mkdir $tempDir
for file in $*
do
	if expr "$file" : ".*\.a$" > /dev/null
	then
		(
			echo "expanding archive " $file
			# for now assumes $file is a full pathname
			cd $tempDir
			$ar x $file
		)
		if [ $? != 0 ]
		then
			rm -f $cwd/$archive
			exit 1
		fi
	elif expr "$file" : ".*\.o" > /dev/null
	then
		echo "copying file " $file
		cp $file $tempDir
		if [ $? != 0 ]
		then
			rm -f $cwd/$archive
			exit 1
		fi
	else
		echo "Unsupported file type: $file" >&2
		rm -f $cwd/$archive
		exit 1
	fi
done
echo "done"
ls $tempDir
$ar $options $archive $tempDir/*.o
if [ $? != 0 ]
then
	rm -f $cwd/$archive
	exit 1
else
	exit 0
fi
