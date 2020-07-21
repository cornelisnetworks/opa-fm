#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
#
# Copyright (c) 2015-2020, Intel Corporation
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

# Run opareport with standard options and pipe output to opaxmlextract to
#  extract all performance counters into a csv file format
tempfile="$(mktemp)"
trap "rm -f $tempfile; exit 1" SIGHUP SIGTERM SIGINT
trap "rm -f $tempfile" EXIT


Usage_full()
{
	echo "Usage: ${cmd} [opareport options]" >&2
	echo "              or" >&2
	echo "       ${cmd} --help" >&2
	echo "   --help - produce full help text." >&2
	echo "   opareport options - options will be passed to opareport." >&2
	echo >&2
	echo "${cmd} provides a report of all performance counters in a " >&2
	echo "CSV format suitable for importing into a spreadsheet or parsed by" >&2
	echo "other scripts." >&2
	echo >&2
	echo "for example:" >&2
	echo "   ${cmd}" >&2
	echo >&2
	echo "   ${cmd} -h 1 -p 2" >&2
	echo >&2
	echo "See the man page for \"opareport\" for the full set of options.">&2
	echo "Do no use \"-o/--output\" report option." >&2
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

IFS=';'
/usr/sbin/opareport -o comps -s -x -Q -d 10 "$@" > $tempfile
if [ -s $tempfile ]
then
	# minor reformatting of header line to condense column names
	echo "NodeDesc;NodeType;NodeGUID;PortNum;nNodeDesc;nNodeType;nNodeGUID;nPortNum;LinkSpeedActive;LinkWidthDnGradeTxActive;LinkWidthDnGradeRxActive;XmitDataMB;XmitData;XmitPkts;MulticastXmitPkts;RcvDataMB;RcvData;RcvPkts;MulticastRcvPkts;XmitWait;CongDiscards;XmitTimeCong;MarkFECN;RcvFECN;RcvBECN;RcvBubble;XmitWastedBW;XmitWaitData;LinkQualityIndicator;LocalLinkIntegrityErrors;RcvErrors;ExcessiveBufferOverruns;LinkErrorRecovery;LinkDowned;UncorrectableErrors;FMConfigErrors;XmitConstraintErrors;RcvConstraintErrors;RcvSwitchRelayErrors;XmitDiscards;RcvRemotePhysicalErrors"
	cat $tempfile | /usr/sbin/opaxmlextract -H -d \; -e Node.NodeDesc -e Node.NodeType -e Node.NodeGUID -e PortInfo.PortNum -e Neighbor.Port.NodeDesc -e Neighbor.Port.NodeType -e Neighbor.Port.NodeGUID -e Neighbor.Port.PortNum -e LinkSpeedActive -e LinkWidthDnGradeTxActive -e LinkWidthDnGradeRxActive -e XmitDataMB -e XmitData -e XmitPkts -e MulticastXmitPkts -e RcvDataMB -e RcvData -e RcvPkts -e MulticastRcvPkts -e XmitWait -e CongDiscards -e XmitTimeCong -e MarkFECN -e RcvFECN -e RcvBECN -e RcvBubble -e XmitWastedBW -e XmitWaitData -e LinkQualityIndicator -e LocalLinkIntegrityErrors -e RcvErrors -e ExcessiveBufferOverruns -e LinkErrorRecovery -e LinkDowned -e UncorrectableErrors -e FMConfigErrors -e XmitConstraintErrors -e RcvConstraintErrors -e RcvSwitchRelayErrors -e XmitDiscards -e RcvRemotePhysicalErrors -s SMs | \
	while read NodeDesc NodeType NodeGUID PortNum nNodeDesc nNodeType nNodeGUID nPortNum LinkSpeedActive LinkWidthDnGradeTxActive LinkWidthDnGradeRxActive XmitDataMB XmitData XmitPkts MulticastXmitPkts RcvDataMB RcvData RcvPkts MulticastRcvPkts XmitWait CongDiscards XmitTimeCong MarkFECN RcvFECN RcvBECN RcvBubble XmitWastedBW XmitWaitData LinkQualityIndicator LocalLinkIntegrityErrors RcvErrors ExcessiveBufferOverruns LinkErrorRecovery LinkDowned UncorrectableErrors FMConfigErrors XmitConstraintErrors RcvConstraintErrors RcvSwitchRelayErrors XmitDiscards RcvRemotePhysicalErrors
	do
		# output will be:
		# for switch port 0
		#   just 1 line, no neighbor
		# for other links
		#	1st line with neighbor
		#	2nd line with most fields except neighbor fields
		#	both lines will have NodeDesc, NodeType, NodeGUID, PortNum
		lineno=$(($lineno + 1))
		if [ x"$nNodeGUID" = x ]
		then
			# must be a port without a neighbor (switch port 0)
			echo "$NodeDesc;$NodeType;$NodeGUID;$PortNum;$nNodeDesc;$nNodeType;$nNodeGUID;$nPortNum;$LinkSpeedActive;$LinkWidthDnGradeTxActive;$LinkWidthDnGradeRxActive;$XmitDataMB;$XmitData;$XmitPkts;$MulticastXmitPkts;$RcvDataMB;$RcvData;$RcvPkts;$MulticastRcvPkts;$XmitWait;$CongDiscards;$XmitTimeCong;$MarkFECN;$RcvFECN;$RcvBECN;$RcvBubble;$XmitWastedBW;$XmitWaitData;$LinkQualityIndicator;$LocalLinkIntegrityErrors;$RcvErrors;$ExcessiveBufferOverruns;$LinkErrorRecovery;$LinkDowned;$UncorrectableErrors;$FMConfigErrors;$XmitConstraintErrors;$RcvConstraintErrors;$RcvSwitchRelayErrors;$XmitDiscards;$RcvRemotePhysicalErrors"
		else
			# port with a neighbor will have a second line
			read NodeDesc_2 NodeType_2 NodeGUID_2 PortNum_2 nNodeDesc_2 nNodeType_2 nNodeGUID_2 nPortNum_2 LinkSpeedActive_2 LinkWidthDnGradeTxActive_2 LinkWidthDnGradeRxActive_2 XmitDataMB_2 XmitData_2 XmitPkts_2 MulticastXmitPkts_2 RcvDataMB_2 RcvData_2 RcvPkts_2 MulticastRcvPkts_2 XmitWait_2 CongDiscards_2 XmitTimeCong_2 MarkFECN_2 RcvFECN_2 RcvBECN_2 RcvBubble_2 XmitWastedBW_2 XmitWaitData_2 LinkQualityIndicator_2 LocalLinkIntegrityErrors_2 RcvErrors_2 ExcessiveBufferOverruns_2 LinkErrorRecovery_2 LinkDowned_2 UncorrectableErrors_2 FMConfigErrors_2 XmitConstraintErrors_2 RcvConstraintErrors_2 RcvSwitchRelayErrors_2 XmitDiscards_2 RcvRemotePhysicalErrors_2
			if [ x"$NodeGUID" != x"$NodeGUID_2" -o x"$PortNum" != x"$PortNum_2" ]
			then
				echo "line: $lineno: Out of synchronization" >&2
			fi
			echo "$NodeDesc;$NodeType;$NodeGUID;$PortNum;$nNodeDesc;$nNodeType;$nNodeGUID;$nPortNum;$LinkSpeedActive_2;$LinkWidthDnGradeTxActive_2;$LinkWidthDnGradeRxActive_2;$XmitDataMB_2;$XmitData_2;$XmitPkts_2;$MulticastXmitPkts_2;$RcvDataMB_2;$RcvData_2;$RcvPkts_2;$MulticastRcvPkts_2;$XmitWait_2;$CongDiscards_2;$XmitTimeCong_2;$MarkFECN_2;$RcvFECN_2;$RcvBECN_2;$RcvBubble_2;$XmitWastedBW_2;$XmitWaitData_2;$LinkQualityIndicator_2;$LocalLinkIntegrityErrors_2;$RcvErrors_2;$ExcessiveBufferOverruns_2;$LinkErrorRecovery_2;$LinkDowned_2;$UncorrectableErrors_2;$FMConfigErrors_2;$XmitConstraintErrors_2;$RcvConstraintErrors_2;$RcvSwitchRelayErrors_2;$XmitDiscards_2;$RcvRemotePhysicalErrors_2"
		fi
	done
	res=0
else
	echo "${cmd}: Unable to get performance report" >&2
	Usage
	res=1
fi
rm -rf $tempfile
exit $res
