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

# Run opareport -o links, then:
#   Extract optional cable values, and values for both ports of each link
#   Remove redundant information and combine cable and port information

cmd=`basename $0`
Usage_full()
{
	echo >&2
	echo "Usage: ${cmd} [--help]|[opareport options]" >&2
	echo "   --help - produce full help text" >&2
	echo "   [opareport options] - options will be passed to opareport." >&2
	echo >&2
	echo "${cmd} is a front end to the opareport tool that generates" >&2
	echo "a report listing all or some of the links in the fabric." >&2
	echo "The output is in a CSV format suitable for importing into a spreadsheet or" >&2
	echo "parsed by other scripts." >&2
	echo >&2
	echo "for example:" >&2
	echo "   List all the links in the fabric:" >&2
	echo "      ${cmd}" >&2
	echo >&2
	echo "   List all the links to a switch named \"OmniPth00117501ffffffff\":" >&2
	echo "      ${cmd} -F \"node:OmniPth00117501ffffffff\"" >&2
	echo >&2
	echo "   List all the links to end-nodes:" >&2
	echo "      ${cmd} -F \"nodetype:FI\"" >&2
	echo >&2
	echo "   List all the links on the 2nd HFI's fabric of a multi-plane fabric:" >&2
	echo "      ${cmd} -h 2" >&2
	echo >&2
	echo "See the man page for \"opareport\" for the full set of options." >&2
	echo >&2
	exit 0
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

## Main function:

# NOTE: opaxmlextract produces the following CSV for each port:
#    2 Link values (CSV 1-2)
#    3 Cable values (CSV 3-5)
#    2 Port values (CSV 6-7)

ix=0

/usr/sbin/opareport -x -o links -d 3 "$@" | \
  /usr/sbin/opaxmlextract -d \; -e Rate -e LinkDetails -e CableLength \
  -e CableLabel -e CableDetails -e DeviceTechShort -e CableInfo.Length \
  -e CableInfo.VendorName -e CableInfo.VendorPN -e CableInfo.VendorRev \
  -e Port.NodeDesc -e Port.PortNum | while read line
do
  case $ix in
  0)
    echo $line";"`echo $line | cut -d \; -f 11-`
    ix=$((ix+1))
    ;;

  1)
    line1=`echo $line | cut -d \; -f 1-10`
    if echo "$line1" | cut -d \; -f3-5 | grep ";;" >/dev/null 2>&1
      then
      line2=`echo $line | cut -d \; -f 11-`
      ix=3
    else
      ix=$((ix+1))
    fi
    ;;

  2)
    line2=`echo $line | cut -d \; -f 11-`
    ix=$((ix+1))
    ;;

  3)
    line3=`echo $line | cut -d \; -f 11-`
    echo $line1";"$line2";"$line3
    ix=1
    ;;
  esac

done

exit 0

