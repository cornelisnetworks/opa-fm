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
#  performance counts



Usage_full()
{
	echo "Usage: ${cmd} [opareport options]" >&2
	echo "              or" >&2
	echo "       ${cmd} --help" >&2
	echo "   --help - produce full help text." >&2
	echo "   opareport options - options will be passed to opareport." >&2
	echo >&2
	echo "${cmd} provides a report of all performance counters in a " >&2
	echo "CVS format suitable for importing into a spreadsheet or parsed by" >&2
	echo "other scripts." >&2
	echo >&2
	echo "for example:" >&2
	echo "   ${cmd}" >&2
	echo >&2
	echo "   ${cmd} -h 1 -p 2" >&2
	echo >&2
	echo "See the man page for \"opareport\" for the full set of options.">&2
	echo >&2
	exit 0
}

Usage()
{
	echo "Usage: ${cmd} [opareport options]" >&2
	echo "              or" >&2
	echo "       ${cmd} --help" >&2
	echo "   --help - produce full help text." >&2
	echo "   opareport options - options will be passed to opareport." >&2
	echo "for example:" >&2
	echo "   ${cmd}" >&2
	echo >&2
	exit 2
}

## Main function

cmd=`basename $0`
if [ x"$1" = "x--help" ]
then
	Usage_full
fi

/usr/sbin/opareport -o comps -s -x -d 10 "$@" | /usr/sbin/opaxmlextract -d \; -e NodeDesc -e SystemImageGUID -e PortNum -e LinkSpeedActive -e LinkWidthDnGradeTxActive -e LinkWidthDnGradeRxActive -e XmitDataMB -e XmitData -e XmitPkts -e MulticastXmitPkts -e RcvDataMB -e RcvData -e RcvPkts -e MulticastRcvPkts -e XmitWait -e CongDiscards -e XmitTimeCong -e MarkFECN -e RcvFECN -e RcvBECN -e RcvBubble -e XmitWastedBW -e XmitWaitData -e LinkQualityIndicator -e LocalLinkIntegrityErrors -e RcvErrors -e ExcessiveBufferOverruns -e LinkErrorRecovery -e LinkDowned -e UncorrectableErrors -e FMConfigErrors -e XmitConstraintErrors -e RcvConstraintErrors -e RcvSwitchRelayErrors -e XmitDiscards -e RcvRemotePhysicalErrors -s Neighbor -s SMs
if [ $? -ne 0 ]; then
	echo "${cmd}: Unable to get performance report" >&2
	Usage
	exit 1
fi
exit 0
