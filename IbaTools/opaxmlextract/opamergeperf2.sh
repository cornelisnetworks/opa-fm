#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2019, Intel Corporation
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

# take output from two runs of opaextractperf2 and merge them by computing
# differences in performance counters
tempfile="$(mktemp)"
trap "rm -f ${tempfile}.prefix1 ${tempfile}.prefix2; exit 1" SIGHUP SIGTERM SIGINT
trap "rm -f ${tempfile}.prefix1 ${tempfile}.prefix2" EXIT


Usage_full()
{
	echo "Usage: ${cmd} before.csv after.csv" >&2
	echo "              or" >&2
	echo "       ${cmd} --help" >&2
	echo "   --help - produce full help text." >&2
	echo "   before.csv - output from a opaextractperf2 run" >&2
	echo "   after.csv - output from a opaextractperf2 run" >&2
	echo >&2
	echo "${cmd} merges output from two opaextactperf2 runs from the same fabric" >&2
	echo "Counters for matching links will be computed (before subtracted from after)" >&2
	echo "and a CSV file equivalent to opaextractperf2's output format will be generated" >&2
	echo "suitable for importing into a spreadsheet or parsing by other scripts." >&2
	echo >&2
	echo "The before.csv and after.csv input files must be generated from the same" >&2
	echo "fabric, with before.csv containing counters prior to after.csv." >&2
	echo "Both files must have been generated to contain the running counters without" >&2
	echo "any counter clears between before.csv and after.csv and without using any" >&2
	echo "interval options to opareport which might report delta counters as" >&2
	echo "opposed to running counters, such as opareport -i" >&2
	echo "or opareport with both --begin and --end options." >&2
	echo >&2
	echo "for example:" >&2
	echo "   ${cmd} before.csv after.csv > delta.csv" >&2
	echo >&2
	exit 0
}

Usage()
{
	echo "Usage: ${cmd} before.csv after.csv" >&2
	echo "              or" >&2
	echo "       ${cmd} --help" >&2
	echo "   --help - produce full help text." >&2
	echo "   before.csv - output from a opaextractperf2 run" >&2
	echo "   after.csv - output from a opaextractperf2 run" >&2
	echo >&2
	echo "for example:" >&2
	echo "   ${cmd} before.csv after.csv > delta.csv" >&2
	echo >&2
	exit 2
}

## Main function
res=0

cmd=`basename $0`
if [ x"$1" = "x--help" ]
then
	Usage_full
fi

if [ "$#" != 2 ]
then
	Usage
fi

f1="$1"
f2="$2"
if [ ! -e "$f1" ]
then
	echo "$f1: Not Found" >&2
	Usage
fi
if [ ! -e "$f2" ]
then
	echo "$f2: Not Found" >&2
	Usage
fi

IFS=';'

# omit heading line
# add sequencing number and line number before desc;type;guid;port so same
# device in each file will sort next to eachother and f1 sorts before f2.
nl -s';' "$f1"|tail -n+2 | sed -e 's/^ */1;/'|sort --field-separator=';' --key='5,6' > ${tempfile}.prefix1
nl -s';' "$f2"|tail -n+2 | sed -e 's/^ */2;/'|sort --field-separator=';' --key='5,6' > ${tempfile}.prefix2
# some debug code which can help identify nodes which are not in both files
#cut -f3-6 -d';' ${tempfile}.prefix1 > ${tempfile}.nodes1
#cut -f3-6 -d';' ${tempfile}.prefix2 > ${tempfile}.nodes2
#if ! diff ${tempfile}.nodes1 ${tempfile}.nodes2 > /dev/null
#then
#	echo "$cmd: Warning: node list mismatch" >&2
#	#diff -c ${tempfile}.nodes1 ${tempfile}.nodes2 | head -20
#fi

IFS=';'
{ 
# output heading
head -1 "$f1"
# compute counter differences f2 - f1
use_2=n # do _2 variables potentially have 1st line ("1") for a port
# sort by guid;port then prefix, this puts same ports next to eachother with
# line from f1 before line from f2
# the loop must handle cases where a device appears only in f1 or f2
# as well as where the neighbor to a device has changed between f1 and f2
# in a stable fabric these will not happen.  However in a fabric with a few
# bad links, devices offline or where cables have been moved, this can happen
( cat ${tempfile}.prefix1 ${tempfile}.prefix2|sort --field-separator=';' --key='5,6' --key '1,1'; echo "END" ) | \
	while read num_1 lineno_1 NodeDesc_1 NodeType_1 NodeGUID_1 PortNum_1 nNodeDesc_1 nNodeType_1 nNodeGUID_1 nPortNum_1 LinkSpeedActive_1 LinkWidthDnGradeTxActive_1 LinkWidthDnGradeRxActive_1 XmitDataMB_1 XmitData_1 XmitPkts_1 MulticastXmitPkts_1 RcvDataMB_1 RcvData_1 RcvPkts_1 MulticastRcvPkts_1 XmitWait_1 CongDiscards_1 XmitTimeCong_1 MarkFECN_1 RcvFECN_1 RcvBECN_1 RcvBubble_1 XmitWastedBW_1 XmitWaitData_1 LinkQualityIndicator_1 LocalLinkIntegrityErrors_1 RcvErrors_1 ExcessiveBufferOverruns_1 LinkErrorRecovery_1 LinkDowned_1 UncorrectableErrors_1 FMConfigErrors_1 XmitConstraintErrors_1 RcvConstraintErrors_1 RcvSwitchRelayErrors_1 XmitDiscards_1 RcvRemotePhysicalErrors_1
do
	if [ x"$num_1" = x"END" ]
	then
		# normal end of file
		break
	fi
	if ! [[ x"$RcvRemotePhysicalErrors_1" =~ ^x[0-9]+$ ]]
	then
		# lines with insufficient columns, extra columns or non-numeric in
		# last column will fail the test.  To balance performance and input
		# checking we only check the last column since invalid input files
		# will typically have the wrong number of columns
		if [ x"$num_1" = x"1" ]
		then
			echo "$cmd: Invalid input line: $lineno_1 file: $f1" >&2
		else
			echo "$cmd: Invalid input line: $lineno_1 file: $f2" >&2
		fi
		exit 1
	fi
	if [ x"$num_1" = x"2" ]
	then
		#echo "process num_1=2" >&2
		if [ "$use_2" != y  ]
		then
			# "2" without a preceeding "1", skip line just read into _1
			echo "$cmd: Warning: skipping unmatched line $lineno_1 in after: $NodeDesc_1;$NodeType_1;$NodeGUID_1;$PortNum_1" >&2
		else
			# _2 has our "1", _1 has our "2"
			if [ x"$NodeGUID_1" != x"$NodeGUID_2" -o  x"$PortNum_1" != x"$PortNum_2" ]
			then
				# got a "1" followed by a "2" and they don't match, skip both
				echo "$cmd: Warning: skipping unmatched line $lineno_2 in before: $NodeDesc_2;$NodeType_2;$NodeGUID_2;$PortNum_2" >&2
				echo "$cmd: Warning: skipping unmatched line $lineno_1 in after: $NodeDesc_1;$NodeType_1;$NodeGUID_1;$PortNum_1" >&2
			elif [ x"$nNodeGUID_1" != x"$nNodeGUID_2" -o  x"$nPortNum_1" != x"$nPortNum_2" ]
			then
				# we got a "1" followed by "2", however neighbor changed
				echo "$cmd: Warning: skipping changed line $lineno_2 in before: $NodeDesc_2;$NodeType_2;$NodeGUID_2;$PortNum_2;$nNodeDesc_2;$nNodeType_2;$nNodeGUID_2;$nPortNum_2" >&2
				echo "$cmd: Warning: skipping changed line $lineno_1 in after: $NodeDesc_1;$NodeType_1;$NodeGUID_1;$PortNum_1;$nNodeDesc_1;$nNodeType_1;$nNodeGUID_1;$nPortNum_1" >&2
			else
				#echo "process num_1=2 use_2" >&2
				# use _2 as before and _1 as after when compute delta
				# for fields not subtracted, show after (_1) as more recent
				echo "$NodeDesc_1;$NodeType_1;$NodeGUID_1;$PortNum_1;$nNodeDesc_1;$nNodeType_1;$nNodeGUID_1;$nPortNum_1;$LinkSpeedActive_1;$LinkWidthDnGradeTxActive_1;$LinkWidthDnGradeRxActive_1;$(($XmitDataMB_1 - $XmitDataMB_2));$(($XmitData_1 - $XmitData_2));$(($XmitPkts_1 - $XmitPkts_2));$(($MulticastXmitPkts_1 - $MulticastXmitPkts_2));$(($RcvDataMB_1 - $RcvDataMB_2));$(($RcvData_1 - $RcvData_2));$(($RcvPkts_1 - $RcvPkts_2));$(($MulticastRcvPkts_1 - $MulticastRcvPkts_2));$(($XmitWait_1 - $XmitWait_2));$(($CongDiscards_1 - $CongDiscards_2));$(($XmitTimeCong_1 - $XmitTimeCong_2));$(($MarkFECN_1 - $MarkFECN_2));$(($RcvFECN_1 - $RcvFECN_2));$(($RcvBECN_1 - $RcvBECN_2));$(($RcvBubble_1 - $RcvBubble_2));$(($XmitWastedBW_1 - $XmitWastedBW_2));$(($XmitWaitData_1 - $XmitWaitData_2));$(($LinkQualityIndicator_1 < $LinkQualityIndicator_2?$LinkQualityIndicator_1:$LinkQualityIndicator_2));$(($LocalLinkIntegrityErrors_1 - $LocalLinkIntegrityErrors_2));$(($RcvErrors_1 - $RcvErrors_2));$(($ExcessiveBufferOverruns_1 - $ExcessiveBufferOverruns_2));$(($LinkErrorRecovery_1 - $LinkErrorRecovery_2));$(($LinkDowned_1 - $LinkDowned_2));$(($UncorrectableErrors_1 - $UncorrectableErrors_2));$(($FMConfigErrors_1 - $FMConfigErrors_2));$(($XmitConstraintErrors_1 - $XmitConstraintErrors_2));$(($RcvConstraintErrors_1 - $RcvConstraintErrors_2));$(($RcvSwitchRelayErrors_1 - $RcvSwitchRelayErrors_2));$(($XmitDiscards_1 - $XmitDiscards_2));$(($RcvRemotePhysicalErrors_1 - $RcvRemotePhysicalErrors_2))"
			fi
		fi
	else
		# num_1 is "1"
		read num_2 lineno_2 NodeDesc_2 NodeType_2 NodeGUID_2 PortNum_2 nNodeDesc_2 nNodeType_2 nNodeGUID_2 nPortNum_2 LinkSpeedActive_2 LinkWidthDnGradeTxActive_2 LinkWidthDnGradeRxActive_2 XmitDataMB_2 XmitData_2 XmitPkts_2 MulticastXmitPkts_2 RcvDataMB_2 RcvData_2 RcvPkts_2 MulticastRcvPkts_2 XmitWait_2 CongDiscards_2 XmitTimeCong_2 MarkFECN_2 RcvFECN_2 RcvBECN_2 RcvBubble_2 XmitWastedBW_2 XmitWaitData_2 LinkQualityIndicator_2 LocalLinkIntegrityErrors_2 RcvErrors_2 ExcessiveBufferOverruns_2 LinkErrorRecovery_2 LinkDowned_2 UncorrectableErrors_2 FMConfigErrors_2 XmitConstraintErrors_2 RcvConstraintErrors_2 RcvSwitchRelayErrors_2 XmitDiscards_2 RcvRemotePhysicalErrors_2
		#echo "process num_2=$num_2" >&2
		if [ x"$num_2" = x"END" ]
		then
			# "1" without a "2" at end of file
			echo "$cmd: Warning: skipping unmatched line $lineno_1 in before: $NodeDesc_1;$NodeType_1;$NodeGUID_1;$PortNum_1" >&2
			break
		fi
		if ! [[ x"$RcvRemotePhysicalErrors_2" =~ ^x[0-9]+$ ]]
		then
			# lines with insufficient columns, extra columns or non-numeric in
			# last column will fail the test.  To balance performance and input
			# checking we only check the last column since invalid input files
			# will typically have the wrong number of columns
			if [ x"$num_2" = x"1" ]
			then
				echo "$cmd: Invalid input line: $lineno_2 file: $f1" >&2
			else
				echo "$cmd: Invalid input line: $lineno_2 file: $f2" >&2
			fi
			exit 1
		fi
		if [ x"$num_2" = x"1" ]
		then
			# two "1" in a row, skip _1, use this line for next pairing
			echo "$cmd: Warning: skipping unmatched line $lineno_1 in before: $NodeDesc_1;$NodeType_1;$NodeGUID_1;$PortNum_1" >&2
			use_2=y
			continue
		elif [ x"$num_1" != "x1" -o x"$num_2" != "x2" ]	# paranoid check
		then
			echo "$num_1 $num_2" >&2
			echo "$cmd: Script Error: Incorrect sort order" >&2
			exit 1
		else
			if [ x"$NodeGUID_1" != x"$NodeGUID_2" -o  x"$PortNum_1" != x"$PortNum_2" ]
			then
				# "1" followed by non-matching "2", skip both
				echo "$cmd: Warning: skipping unmatched line $lineno_1 in before: $NodeDesc_1;$NodeType_1;$NodeGUID_1;$PortNum_1" >&2
				echo "$cmd: Warning: skipping unmatched line $lineno_2 in after: $NodeDesc_2;$NodeType_2;$NodeGUID_2;$PortNum_2" >&2
			elif [ x"$nNodeGUID_1" != x"$nNodeGUID_2" -o  x"$nPortNum_1" != x"$nPortNum_2" ]
			then
				# we got a "1" followed by "2", however neighbor changed
				echo "$cmd: Warning: skipping changed line $lineno_1 in before: $NodeDesc_1;$NodeType_1;$NodeGUID_1;$PortNum_1;$nNodeDesc_1;$nNodeType_1;$nNodeGUID_1;$nPortNum_1" >&2
				echo "$cmd: Warning: skipping changed line $lineno_2 in after: $NodeDesc_2;$NodeType_2;$NodeGUID_2;$PortNum_2;$nNodeDesc_2;$nNodeType_2;$nNodeGUID_2;$nPortNum_2" >&2
			else
				# for fields not subtracted, show after (_2) as more recent
				echo "$NodeDesc_2;$NodeType_2;$NodeGUID_2;$PortNum_2;$nNodeDesc_2;$nNodeType_2;$nNodeGUID_2;$nPortNum_2;$LinkSpeedActive_2;$LinkWidthDnGradeTxActive_2;$LinkWidthDnGradeRxActive_2;$(($XmitDataMB_2 - $XmitDataMB_1));$(($XmitData_2 - $XmitData_1));$(($XmitPkts_2 - $XmitPkts_1));$(($MulticastXmitPkts_2 - $MulticastXmitPkts_1));$(($RcvDataMB_2 - $RcvDataMB_1));$(($RcvData_2 - $RcvData_1));$(($RcvPkts_2 - $RcvPkts_1));$(($MulticastRcvPkts_2 - $MulticastRcvPkts_1));$(($XmitWait_2 - $XmitWait_1));$(($CongDiscards_2 - $CongDiscards_1));$(($XmitTimeCong_2 - $XmitTimeCong_1));$(($MarkFECN_2 - $MarkFECN_1));$(($RcvFECN_2 - $RcvFECN_1));$(($RcvBECN_2 - $RcvBECN_1));$(($RcvBubble_2 - $RcvBubble_1));$(($XmitWastedBW_2 - $XmitWastedBW_1));$(($XmitWaitData_2 - $XmitWaitData_1));$(($LinkQualityIndicator_2 < $LinkQualityIndicator_1?$LinkQualityIndicator_2:$LinkQualityIndicator_1));$(($LocalLinkIntegrityErrors_2 - $LocalLinkIntegrityErrors_1));$(($RcvErrors_2 - $RcvErrors_1));$(($ExcessiveBufferOverruns_2 - $ExcessiveBufferOverruns_1));$(($LinkErrorRecovery_2 - $LinkErrorRecovery_1));$(($LinkDowned_2 - $LinkDowned_1));$(($UncorrectableErrors_2 - $UncorrectableErrors_1));$(($FMConfigErrors_2 - $FMConfigErrors_1));$(($XmitConstraintErrors_2 - $XmitConstraintErrors_1));$(($RcvConstraintErrors_2 - $RcvConstraintErrors_1));$(($RcvSwitchRelayErrors_2 - $RcvSwitchRelayErrors_1));$(($XmitDiscards_2 - $XmitDiscards_1));$(($RcvRemotePhysicalErrors_2 - $RcvRemotePhysicalErrors_1))"
			fi
		fi
	fi
	use_2=n
done
}
res=$?
rm -rf ${tempfile}.prefix1 ${tempfile}.prefix2
exit $res
