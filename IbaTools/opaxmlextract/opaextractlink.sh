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
	echo "   List all the links to a switch named \"coresw1\":" >&2
	echo "      ${cmd} -F \"node:coresw1\"" >&2
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

ix=0


/usr/sbin/opareport -x -o links "$@" -d 3 | \
  /usr/sbin/opaxmlextract -d \; -e Rate -e LinkDetails -e CableLength \
  -e CableLabel -e CableDetails -e DeviceTechShort -e CableInfo.Length \
  -e CableInfo.VendorName -e CableInfo.VendorPN -e CableInfo.VendorRev \
  -e Port.NodeDesc -e Port.PortNum | while read line
do
  # opareport -o links generates XML output of this general form:
  #  <Link>
  #     <Rate>....
  #     <LinkDetails>....
  #     <Cable>
  #       ... cable information from topology.xml
  #     </Cable>
  #     <Port>
  #       .. information about 1st port excluding its CableInfo
  #     </Port>
  #     <Port>
  #       .. information about 2nd port excluding its CableInfo
  #     </Port>
  #     <CableInfo>
  #       .. information about the CableInfo for the cable between the two ports
  #     </CableInfo>
  #  </Link>
  # opaxmlextract produces the following CSV format on each line:
  #    2 Link values (CSV 1-2) (Rate, LinkDetails)
  #    3 Cable values (CSV 3-5) (CableLength, CableLabel, CableDetails)
  #    5 CableInfo values (CSV 6-10) (DeviceTechShort, ... VendorRev)
  #    2 Port values (CSV 11-12) (NodeDesc, PortNum)
  # due to the nesting of tags, opaxmlextract will output the following
  #    a line with Link and Cable values but empty CableInfo and Port values
  #    a line for 1st port with Link and Port but empty Cable values
  #    a line for 2nd port with Link and Port but empty Cable values
  #    a line which contains the link and CableInfo.
  # if the topology.xml file is not supplied or has no information for the
  # given link, opareport will not output the <Cable> section and the
  # 1st line above will not be output by opaxmlextract for the given link
  case $ix in
  0)
     # process heading and build new per link heading for this report
     echo $line";"`echo $line | cut -d \; -f 11-`
     ix=$((ix+1))
     ;;

  1)
     # process 1st line in a given link
     line1=`echo $line | cut -d \; -f 1-10`
     if [ `echo "$line1" | cut -d \; -f 3-5` = ";;" ]
       then
       # no topology file, we have port information here
       line2=`echo $line | cut -d \; -f 11-`
       ix=3
     else
      # have a topology file, port information on next line
      ix=$((ix+1))
     fi
     ;;

  2)
     # process 1st port in a given link when we have a topology file
     line2=`echo $line | cut -d \; -f 11-`
     ix=$((ix+1))
     ;;

  3)
     # process 2nd port in a given link when we have a topology file
     # and output a single line for the given link
     line3=`echo $line | cut -d \; -f 11-`
     ix=4
     ;;

  4)
     #process the third line for the CableInfo and extract the part of line1 before the CableInfo details in the csv format
     line4=`echo $line | cut -d \; -f 6-10`
     line1=`echo $line1 | cut -d \; -f 1-5`
     # combine all extracted parts from the three input lines into a single output line.
     echo $line1";"$line4";"$line2";"$line3
     ix=1
     ;;

  esac

done

exit 0
