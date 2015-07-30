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
## optionaldir
## ----
## This script is for use in toplevel makefiles to filter out optional
## directories
##
## Usage:
##	optionaldir dir [ulp ...]
## Usage in a makefile:
##	DIRS = $(shell $OPTIONALDIR Sdp SDP Cluster)
##
## Arguments:
##	dir - directory which is optional.
##	ulp - one or more ulps/components which this directory is part of
##		if any of these components are enabled, this the directory will be
##		output.  If no ulps are specified, the directory is enabled.
##
## Environment expected:
##	BUILD_ULPS - ulps/components to be enabled (defaults to all)
##	BUILD_SKIP_ULPS - ulps/components to be disabled (defaults to none)
##	if a ulp matches both, it will be skipped
##
## Additional Information:
##	If the directory is enabled and present, this script will
##	output the directory name.
##	If the directory is disabled or not present, this script will
##	output nothing.
USAGE="Usage: $0 dir [ulps ...]"

BUILD_ULPS=${BUILD_ULPS:-all}
BUILD_SKIP_ULPS=${BUILD_SKIP_ULPS:-none}

if [ "$#" -lt 1 ]
then
	# omit usage string so we don't mess up Make
	#echo "$USAGE" >&2	
	exit 2
fi

dir=$1
shift

if [ ! -d $dir ]
then
	exit 0
fi

if [ "$#" -eq 0 ]
then
	echo "$dir"
	exit 0
fi

enabled=0
disabled=0
while [ "$#" -gt 0 ]
do
	ulp="$1"
	if echo "$BUILD_ULPS"|egrep "$ulp|all" > /dev/null 2>/dev/null
	then
		enabled=1
	fi
	if echo "$BUILD_SKIP_ULPS"|egrep "$ulp|all" > /dev/null 2>/dev/null
	then
		disabled=1
	fi
	shift
done

if [ "$disabled" = 0 -a "$enabled" = 1 ]
then
	echo "$dir"
fi

exit 0
