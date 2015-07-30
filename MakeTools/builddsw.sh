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
#
# File:		builddsw
#
# Purpose:	Creates a Dev Studio Workspace (.dsw) file
#		which references the given list of dsp files
#

Usage() {
   cmd="Usage: ${0} -f <dswFile> [file1.dsp file2.dsp ...]"
   echo -e ${cmd}
   return
}

createDswHeader() {
   echo -e "Microsoft Developer Studio Workspace File, Format Version 6.00" > ${dswFile}
   echo -e "# WARNING: DO NOT EDIT OR DELETE THIS WORKSPACE FILE!\n" >> ${dswFile}
   echo -e "###############################################################################" >> ${dswFile}
}

createDswTrailer() {
   echo -e "\nGlobal:\n" >> ${dswFile}
   echo -e "Package=<5>" >> ${dswFile}
   echo -e "{{{" >> ${dswFile}
   echo -e "}}}\n" >> ${dswFile}
   echo -e "Package=<3>" >> ${dswFile}
   echo -e "{{{" >> ${dswFile}
   echo -e "}}}\n" >> ${dswFile}
   echo -e "###############################################################################" >> ${dswFile}
}


# Function to create a .dsp entry in the workspace file.
createDswEntry() {
   dspDir=`dirname  ${1}`
   #the version of sed below occasionally dumps on DOS, so explicitly
   # remove .dsp suffix using basename
   #dspName=`basename ${1} | sed 's/\..*$//'`
   dspName=`basename ${1} .dsp`

   echo -e "\nProject: \"${dspName}\"=${1} - Package Owner=<4>\n" >> ${dswFile}
   echo -e "Package=<5>" >> ${dswFile}
   echo -e "{{{" >> ${dswFile}
   echo -e "    ${dspName}" >> ${dswFile}
   echo -e "    ${dspDir}" >> ${dswFile}
   echo -e "}}}\n" >> ${dswFile}
   echo -e "Package=<4>" >> ${dswFile}
   echo -e "{{{" >> ${dswFile}
   echo -e "}}}\n" >> ${dswFile}
   echo -e "###############################################################################" >> ${dswFile}
}

# Script main starts here
dswFile=""
needHeader=0

while [ "${1}" != "" ] ; do
   case "${1}" in
      -f)   shift
            dswFile=${1}
            shift
            ;;
	
      *)    if [ "${dswFile}" = "" ] ; then
		     	Usage
		     	exit 1
	    	else
		 		if [ ${needHeader} -eq 0 ] ; then
		     		createDswHeader
		     		needHeader=1
		 		fi
		 		createDswEntry ${1}
           	fi
            shift
            ;;

   esac
done

if [ "${dswFile}" = "" -o ${needHeader} -eq 0 ] ; then
   Usage
   exit 1
fi

if [ ${needHeader} -eq 1 ] ; then
   createDswTrailer
fi

exit 0
