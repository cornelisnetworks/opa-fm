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
# return BUILD_SMP option value based on present system
# 0->uniproc, 1->smp, 2->enterprise, 3->bigmem, 4->debug 5->kgdb 6->kgdbsmp
if [ `uname -s` = Darwin ]
then
	# all MAC kernels are SMP
	echo 1
else
	name=`uname -r`
	case $name in
		*smp*) rval="1";;
		*enterprise*) rval="2";;
		*bigmem*) rval="3";;
		*debug*) rval="4";;
		*kgdbsmp*) rval="6";;
		*kgdb*) rval="5";;
		*) rval="0";;
	esac
	if [ "$BUILD_TARGET_OS_VENDOR_VERSION" = "ES3" ]
	then
		if [ "$BUILD_TARGET" = "IA64" -o "$BUILD_TARGET" = "EM64T" ]
		then
			rval="1"
		fi
	fi
	if [ "$BUILD_TARGET_OS_VENDOR_VERSION" = "ES9" ]
	then
		if [ "$BUILD_TARGET" = "IA64" ]
		then
			rval="1"
		fi
	fi
fi
echo $rval
