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

# Run opareport -o errors with topology XML, then:
#   Extract values (including port statistics) for both ports of each link
#   Remove redundant information for each link and combine link port information

## Local functions:

# Usage()
#
# Description:
#   Output information about program usage and parameters
#
# Inputs:
#   none
#
# Outputs:
#   Information about program usage and parameters



usage()
{
    echo "Usage:  ${cmd} topology_file [opareport options]" >&2
    echo "       or  ${cmd} --help" >&2
    echo "   --help - produce full help text" >&2
    echo "   [opareport options] - options will be passed to opareport." >&2
    echo >&2
    exit 2
}

Usage_full()
{
    echo "Usage: ${cmd} topology_file [opareport options]" >&2
    echo "       or  ${cmd} --help" >&2
    echo "   --help - produce full help text" >&2
    echo "   [opareport options] - options will be passed to opareport." >&2
    echo >&2
    echo "${cmd} is a front end to the opareport and opaxmlextract tools that" >&2
    echo "performs an error analysis of a fabric and provides augmented information" >&2
    echo "from a topology file including all error counters." >&2
    echo "The output is in a CSV format suitable for importing into a spreadsheet or" >&2
    echo "parsed by other scripts." >&2
    echo >&2
    echo "for example" >&2
    echo "	${cmd} topology_file" >&2
    echo >&2
    echo "	${cmd} topology_file -c my_opamon.conf" >&2
    echo >&2
    echo "See the man page for \"opareport\" for the full set of options." >&2
    echo >&2
	exit 0

}

## Main function:

cmd=`basename $0`
if [ x"$1" = "x--help" ]
then
	Usage_full
fi

if [[ $# -lt 1 || "$1" == -* ]]
then
    usage
fi

# NOTE: opaxmlextract produces the following CSV for each port:
#    4 Link values (CSV 1-4)
#    3 Cable values (CSV 5-7)
#    6 Port values (CSV 8-13)
#   14 Stat values (CSV 14-27)

# Combine 2 ports for each link onto 1 line, removing redundant Link and Cable values

ix=0

/usr/sbin/opareport -x -d 10 -s -o errors -T "$@" | \
  /usr/sbin/opaxmlextract -d \; -e Rate -e MTU -e Internal -e LinkDetails \
  -e CableLength -e CableLabel -e CableDetails -e Port.NodeGUID \
  -e Port.PortGUID -e Port.PortNum -e Port.PortType -e Port.NodeDesc \
  -e Port.PortDetails \
  -e XmitData.Value -e XmitPkts.Value -e PortMulticastXmitPkts.Value \
  -e RcvData.Value -e RcvPkts.Value -e MulticastRcvPkts.Value \
  -e XmitWait.Value -e CongDiscards.Value -e XmitTimeCong.Value \
  -e MarkFECN.Value -e RcvFECN.Value -e RcvBECN.Value \
  -e RcvBubble.Value -e XmitWastedBW.Value -e XmitWaitData.Value \
  -e LinkQualityIndicator.Value -e LocalLinkIntegrityErrors.Value \
  -e RcvErrors.Value -e ExcessiveBufferOverruns.Value \
  -e LinkErrorRecovery.Value -e LinkDowned.Value -e UncorrectableErrors.Value \
  -e FMConfigErrors.Value -e XmitConstraintErrors.Value \
  -e RcvConstraintErrors.Value -e RcvSwitchRelayErrors.Value \
  -e XmitDiscards.Value -e RcvRemotePhysicalErrors.Value | \
  while read line
do
  case $ix in
  0)
    echo $line";"`echo $line | cut -d \; -f 8-`
    ix=$((ix+1))
    ;;

  1)
    line1=$line
    ix=$((ix+1))
    ;;

  2)
    line2=`echo $line | cut -d \; -f 8-`
    echo $line1";"$line2
    ix=1
    ;;
  esac

done

exit 0

