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
#
# File:		finddspfiles
#
# Purpose:	outputs a list of dsp files for use in building a dsw file
#
# The DSP names provided as arguments can be actual dsp filenames or
# the names of directories (relative to TL_DIR)
# When a directory name is provided, the corresponding dsp filename is
# constructed by appending DSP_SUFFIX to to.
#
# At present absolute path names for dsp files are not supported
# $TL_DIR will be prefixed to all names given.  In the future
# absolute paths could be supported (would they be C: or /cygwin style?)
#
# The output will be full pathnames to stdout
#
# Typical use is:
#	NAMES=Gen Osa Something/Dir/project.dsp
#	builddsw -f dswfile `finddspfiles $TL_DIR $DSP_SUFFIX $NAMES`
#

Usage() {
   cmd="Usage: ${0} \$TL_DIR \$DSP_SUFFIX name1 name2 ..."
   echo -e ${cmd}
   return
}

if [ $# -le 2 ]
then
	Usage
	exit 2
fi

TL_DIR="${1}"
DSP_SUFFIX="${2}"
shift
shift
while [ "${1}" != "" ]
do
	pathname="${TL_DIR}/${1}"
	if [ -d "$pathname" ]
	then
		# append project.dsp, where project is last directories name
		project=`basename "$pathname"`
		echo "$pathname/${project}${DSP_SUFFIX}"
	else
		# assume its a full dsp filename
		echo "$pathname"
	fi
	shift
done
exit 0
