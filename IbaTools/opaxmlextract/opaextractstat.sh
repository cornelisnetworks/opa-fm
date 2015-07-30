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

TOOLSDIR=${TOOLSDIR:-/opt/opa/tools}
BINDIR=${BINDIR:-/usr/sbin}

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
    echo "Usage: opaextractstat topology_file" >&2
    exit 2
}


## Main function:

if [ $# -ne 1 ]
then
    usage
fi

# NOTE: opaxmlextract produces the following CSV for each port:
#    3 Link values (CSV 1-3)
#    3 Cable values (CSV 4-6)
#    2 Port values (CSV 7-8)
#    1 Stat value (CSV 9)

# Combine 2 ports for each link onto 1 line, removing redundant Link and Cable values

ix=0

$BINDIR/opareport -x -d 10 -s -o errors -T $@ | \
  $BINDIR/opaxmlextract -d \; -e Rate -e MTU -e LinkDetails -e CableLength \
  -e CableLabel -e CableDetails -e Port.NodeDesc -e Port.PortNum \
  -e LinkQualityIndicator.Value | while read line
do
  case $ix in
  0)
    echo $line";"`echo $line | cut -d \; -f 7-`
    ix=$((ix+1))
    ;;

  1)
    line1=$line
    ix=$((ix+1))
    ;;

  2)
    line2=`echo $line | cut -d \; -f 7-`
    echo $line1";"$line2
    ix=1
    ;;
  esac

done

exit 0

