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

# Run opareport with standard options and pipe output to opaxmlextract to extract
#  error counts

Usage_full()
{
	echo "Usage: opaextracterror [opareport options]" >&2
	echo "              or" >&2
	echo "       opaextracterror --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   opareport options - options will be passed to opareport." >&2
	echo "for example:" >&2
	echo "   opaextracterror" >&2
	echo "   opaextracterror -h 1 -p 2" >&2
	exit 0
}

Usage()
{
	echo "Usage: opaextracterror" >&2
	echo "              or" >&2
	echo "       opaextracterror --help" >&2
	echo "   --help - produce full help text" >&2
	echo "for example:" >&2
	echo "   opaextracterror" >&2
	exit 2
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

BINDIR=${BINDIR:-/usr/sbin}


$BINDIR/opareport -o comps -s -x -d 10 $@ | $BINDIR/opaxmlextract -d \; -e NodeDesc -e SystemImageGUID -e PortNum -e LinkSpeedActive -e LinkWidthDnGradeTxActive -e LinkWidthDnGradeRxActive -e LinkQualityIndicator -e RcvSwitchRelayErrors -e LocalLinkIntegrityErrors -e RcvErrors -e ExcessiveBufferOverruns -e FMConfigErrors -e LinkErrorRecovery -e LinkDowned -e UncorrectableErrors -e RcvConstraintErrors -e XmitConstraintErrors -s Neighbor -s SMs
exit 0
