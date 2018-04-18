#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015-2017, Intel Corporation
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

if [ -f /etc/os-release ]
then
	id=$(grep ^ID= /etc/os-release | cut -f2 -d= | cut -f2 -d\")
	versionid=$(grep ^VERSION_ID= /etc/os-release | cut -f2 -d\")
else
    if [ `uname -s` == "Darwin" ]
    then
        # Apple Mac
        rval=apple
    else
        filelist=`'ls' /etc/*-release | egrep -v lsb | egrep -v os`
        rval=""
        for file in $filelist
        do
	    if [ -f $file ]
	    then
		    rval=`basename $file -release`
		    if [ $rval = 'SuSE' ]
		    then
			    if [ -f /etc/UnitedLinux-release ]
			    then
				    rval=UnitedLinux
			    fi
			elif [ $rval = 'centos' ]
			then
				rval=redhat
			elif [ $rval != 'os' ]
			then
				break
		    fi
	    fi
        done
    fi
    case $rval in
	redhat)
		id=rhel
		;;
	SuSE)
		id=sles
		;;
	*)
		id=""
		;;
	esac

	case $id in
	rhel)
		if grep -qi advanced /etc/redhat-release
		then
			rval=`cat /etc/redhat-release | cut -d' ' -f7`
		elif grep -qi enterprise /etc/redhat-release
		then
			# /etc/redhat-release = "Red Hat Enterprise Linux Server release $a.$b ($c)"
			rval=`cat /etc/redhat-release | cut -d' ' -f7 | cut -d. -f1`
			major=`cat /etc/redhat-release | cut -d' ' -f7 | cut -d. -f1`
			minor=`cat /etc/redhat-release | cut -d' ' -f7 | cut -d. -f2`
			if [ \( $major -ge 7 -a $minor -ne 0 \) -o \( $major -eq 6 -a $minor -ge 7 \) ]
			then
				rval=$rval.$minor
			fi
		elif grep -qi centos /etc/redhat-release
		then
			# CentOS 
			rval=`cat /etc/redhat-release | sed -r 's/^.+([[:digit:]])\.([[:digit:]]).+$/\1.\2/'`
		elif grep -qi scientific /etc/redhat-release
		then
			# Scientific Linux.
			rval=`cat /etc/redhat-release | sed -r 's/^.+([[:digit:]])\.([[:digit:]]).+$/\1.\2/'`
		else
			rval=`cat /etc/redhat-release | cut -d' ' -f5`
		fi
		;;
	sles)
		v1=$(grep VERSION /etc/SuSE-release | cut -d' ' -f3)
		v2=$(grep PATCHLEVEL /etc/SuSE-release | cut -d' ' -f3)
		rval=${v1}.$v2
		;;
	*)
		rval=""
	esac
	versionid=$rval
fi

echo $id $versionid
exit 0
