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

# [ICS VERSION STRING: unknown]
## prep
## ----
## This script will mark all the files in a release with the appropriate
## release tag
##
## Usage:
##	cd $CODE_DIR; find . -print|prep [version_marker]
##
## Arguments:
##	version_marker - the string to look for at the start of each
##		version string in scripts, and the string to add (with tag
##		appended) to mark all files.  For example: "ICS VERSION STRING"
##		The default is "ICS VERSION STRING"
##
## Environment expected:
##	TL_DIR = top level directory
##	RELEASE_TAG = cvs release tag to mark in each file
##	MCS = mcs program to use (defaults to mcs)
##	AR = ar program to use (default to ar)
##	DATE = date string to include in file (defaults to current date/time)
##
## Method of marking:
##	archives, executables, dynamic libraries, relocatable .o's
##		On Linux/Cygwin:
##		- version string appended at end of file
##		Future/Other:
##		- mcs used to put version string in .comment section of ELF file
##	other files (text, etc)
##		- sed used to replace "$version_marker:.*]" with the version
##		  string.  Note that the version string in text files ends in a
##		  ] and will include "$version_marker:", hence prep can be
##		  run multiple times against a given file/tree

USAGE="Usage: $0 [version_marker] < filelist"

trap 'rm -f /tmp/prep$$; exit 1' 1 2 3 9 15

MCS=${MCS:-mcs}
AR=${AR:-ar}
if [ -e $TL_DIR/MakeTools/rm_version/rm_version ]
then
	RM_VERSION=${RM_VERSION:-$TL_DIR/MakeTools/rm_version/rm_version}
else
	RM_VERSION=${RM_VERSION:-rm_version}
fi

if [ "${RELEASE_TAG:-}" = "" ]
then
	echo "$0: RELEASE_TAG must be exported"
	exit 1
fi

DATE_FMT='%m/%d/%y %H:%M'
[ -z "$SOURCE_DATE_EPOCH" ] ||\
	DATE=${DATE:-"`date -u -d@$SOURCE_DATE_EPOCH "+$DATE_FMT"`"}
DATE=${DATE:-"`date "+$DATE_FMT"`"}

if [ "$#" = 1 ]
then
	version_marker="$1"
	shift
else
	version_marker="ICS VERSION STRING"
fi
if [ "$#" != 0 ]
then
	echo "$USAGE"
	exit 2
fi
# determine gcc version, 2.96 level of tools can have some problems
gcc_major=`gcc -v 2>&1|fgrep 'gcc version'|cut -f3 -d' '|cut -f1 -d'.'`

echo "$0: $RELEASE_TAG [$DATE]"

# stdin is the input to xargs
xargs file | sed -e 's/:[ 	]/ /'| \
   while read name filetype
   do
	if [ -L $name ]
	then
		# skip symbolic links
		continue
	fi
	VERSION_STRING="@(#) $name $RELEASE_TAG [$DATE]"
	case "$name" in
	*.uid)
		# no mcs command equivalent for uid files
		# however appending to the end the .uid file
		# doesn't seem to hurt
		$RM_VERSION -m "$version_marker" $name >/dev/null
		echo "$version_marker: $VERSION_STRING" >> $name
		echo "Version string added to: $name"
		continue;;
	*.gz|*.tgz)
		# skip compressed files
		continue;;
	*.res)
		# skip result files
		continue;;
	esac
	case "$filetype" in
	*directory*)
		echo "Processing Directory $name..."
		;;
	*text*|*script*)
		# only sed files which have the pattern,
		# this avoids corrupting data files
 		# -q causes grep to exit on first match (for performance)
		if grep -q "$version_marker:" $name >/dev/null
		then
			sed -e "s|$version_marker:.*]|$version_marker: $VERSION_STRING|" < $name > /tmp/prep$$
			# this keeps original permissions on the file
			cat /tmp/prep$$ > $name
			echo "Version string added to: $name"
		else
			> /dev/null
			#echo "warning: No version string in file: $name"
		fi;;
	*ELF*|*executable*|*archive*|*relocatable*|*dynamic?lib*|*shared?lib*)
		if false
		then
			# no objcopy/msc command equivalent
			# however appending to the end of a.out or .sl file
			# doesn't seem to hurt
			$RM_VERSION -m "$version_marker" $name >/dev/null
			echo "$version_marker: $VERSION_STRING" >> $name
			echo "Version string added to: $name"
		else
			#$MCS -d -a "$VERSION_STRING" $name
			#case "$filetype" in
			#*archive*)
			#	echo "Adding symbol table back to $name ($AR ts $name)"
			#	$AR ts $name >/dev/null
			#	;;
			#esac
			echo "$version_marker: $VERSION_STRING" > /tmp/prep$$
			#set -x
			# RH7 tools have an issue here, not sure if its gcc 2.96 or
			# objcopy 2.11 issue, key off gcc version for now
			if [ "$gcc_major" -gt 2 ]
			then
				objcopy --remove-section=.ics_comment --add-section .ics_comment=/tmp/prep$$ $name
			fi
			#set +x
			echo "Version string added to: $name"
		fi
		;;
	*)
		# only sed files which have the pattern,
		# this avoids corrupting data files
 		# -q causes grep to exit on first match (for performance)
		if grep -q "$version_marker:" $name >/dev/null
		then
			sed -e "s|$version_marker:.*]|$version_marker: $VERSION_STRING|" < $name > /tmp/prep$$
			# this keeps original permissions on the file
			cat /tmp/prep$$ > $name
			echo "Version string added to: $name"
		else
			> /dev/null
			#echo "warning: No version string in file: $name"
		fi
	esac
   done

rm -f /tmp/prep$$
exit 0
