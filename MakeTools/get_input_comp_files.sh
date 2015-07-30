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
## get_input_comp_files
## -----------
## function to lookup the files needed for a specific bundle/component combo
## get value of appropriate product variable in WrappedProducts
## returns value of the line (eg. files for Component-Bundle combo)
##
## Usage:
##	get_input_comp_files component bundle
##
## Arguments:
##	component - one of OFED, SRP, VNIC, FF, FV or FM
##	bundle - one of Basic or IFS or TestTools or TESTS, PRs or RELEASE_NOTES
## Required Environment (as set via setenv or similar tools):
##	TL_DIR
## 	BUILD_PLATFORM_OS_VENDOR_VERSION
##	BUILD_TARGET
##	BUILD_TARGET_OS_VENDOR

. $ICSBIN/funcs.sh

Usage()
{
	# include "ERROR" in message so weeklybuild catches it in error log
	echo "ERROR: get_input_comp_files failed" >&2
	echo "Usage: get_input_comp_files component bundle" >&2
	exit 2
}

if [ $# != 2 ]
then
	Usage
fi

if [ "$1" = "IFS" ]
then
	# special mistake which can trigger use of $IFS below and cause confusing
	# result
	Usage
fi

# $1 = Component Product Name
# $2 = Packaging Bundle
component=$1
bundle=$2

. $TL_DIR/$PROJ_FILE_DIR/WrappedProducts

# workaround broken versions of target command
if [ x"$BUILD_TARGET_OS_VENDOR_VERSION" = x"" ]
then
	BUILD_TARGET_OS_VENDOR_VERSION=$BUILD_PLATFORM_OS_VENDOR_VERSION
fi
os_vendor_ver=$(echo $BUILD_TARGET_OS_VENDOR_VERSION | tr '.' '_')
os_vendor_brief_ver=$(echo $os_vendor_ver|sed -e 's/_.*//')
os_id="${BUILD_TARGET}_${BUILD_TARGET_OS_VENDOR}_$os_vendor_ver"
# also handle the case of os_vendor_ver with minor rev part removed
brief_os_id="${BUILD_TARGET}_${BUILD_TARGET_OS_VENDOR}_$os_vendor_brief_ver"

value=""
for parameter in ${bundle}_${component}_${os_id} ${bundle}_${component}_${brief_os_id} ${bundle}_${component} ALL_${component}_${os_id} ALL_${component}_${brief_os_id} ALL_$component
do
	value=$(eval echo \"\$"$parameter"\")
	if [ ! -z "$value" ]
	then
		break
	fi
done
if [ -z "$value" ]
then
	echo "ERROR: $component not found in WrappedProducts" >&2
	echo "ERROR: Tried: ${bundle}_${component}_${os_id}" >&2
	echo "              ${bundle}_${component}_${brief_os_id}" >&2
	echo "              ${bundle}_${component}" >&2
	echo "              ALL_${component}_${os_id}" >&2
	echo "              ALL_${component}_${brief_os_id}" >&2
	echo "              ALL_$component" >&2
	exit 1
fi
echo "$value"
exit 0
