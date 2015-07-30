#!/bin/sh
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

engineer=`echo $USER | cut -c-4`
timestamp='_'`date +%m%d%y_%H%M`
echo "R0$engineer$timestamp" > .ICSBOOTROMVERSIONSTRING
if [ $# != 0 ]
then
	# handle case of embedded builds which don't build MakeTools patch_version
	if [ -e $TL_DIR/MakeTools/patch_version/patch_version ]
	then
		PATCH_VERSION=${PATCH_VERSION:-$TL_DIR/MakeTools/patch_version/patch_version}
	else
		PATCH_VERSION=${PATCH_VERSION:-patch_version}
	fi
	$PATCH_VERSION R0$engineer$timestamp $1
else
	if [ -e $TL_DIR/MakeTools/convert_releasetag.pl ] 
	then
		$TL_DIR/MakeTools/convert_releasetag.pl R0$engineer$timestamp
	else
		convert_releasetag R0$engineer$timestamp
	fi
fi
