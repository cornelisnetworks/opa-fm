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
# File:		makedepend
#
# Purpose:	Creates a dependency make file for Visual Studio Source
#


# Function to create a dependency file for a given source file.
create_depend() {
   stem=`echo ${2} | ${sed} 's/\(^.*\)\..*/\1/'`
   suffix=`echo ${2} | ${sed} 's/^.*\.\(.*$\)/\1/'`

   # Do not process IDL dependencies with the C/C++ compiler.
   if [[ "${suffix}" = "idl" ]] ; then
     return
   fi

   depend=`cl /E ${1} ${2} |\
           ${grep} '^[ \t]*#line[ \t].' | \
           ${egrep} -v -i 'devstudio|VC98|Microsoft Platform SDK' | \
           ${sed} -e 's/^[ \t]*#/#/'\
               -e 's/^.*#line[ \t].*[ \t]"\(.*\)".*/\1/g' \
               -e "/${2}/d" | \
           ${sort} -n | \
           ${uniq}`
   if [[ "${suffix}" = "rc" || "${suffix}" = "RC" ]] ; then
      object="${stem}.res"
   else
      object="${stem}.obj"
   fi
   depend="${object} : ${2} ${depend}"
   echo ${depend} | ${sed} 's#\\\\#/#g' > ${3}
}

Usage() {
   cmd="Usage: ${0} -p <tool_path> -f <out-file> -- <CFLAGS> -- [file1 file2 ...]"
   echo ${cmd}
   return
}

if [[ "${1}" = "" ]] ; then
   Usage
   exit 1
fi

while [[ "${1}" != "" ]] ; do
   case "${1}" in
      '')   Usage
            exit 1;;

      -p)   shift
            tool_path=${1}
            sed=${tool_path}/sed
            grep=${tool_path}/grep
            egrep=${tool_path}/egrep
            sort=${tool_path}/sort
            uniq=${tool_path}/uniq
            shift
            ;;

      -f)   shift
            out_file=${1}
            shift
            ;;
	
      --)   if [[ "${flag_start}" = "" ]] ; then
               flag_start=start
               flags=/E
            else
               flag_stop=stop
            fi
            shift
            ;;

      *)    if [[ "${flag_start}" = "start" ]] ; then
               if [[ "${flag_stop}" = "" ]] ; then
                  flags="${flags} ${1}"
               else
	              if [[ "${tool_path}" = "" || "${out_file}" = "" ]] ; then
		              Usage
		              exit 1
		          fi
                  create_depend "${flags}" "${1}" "${out_file}"
               fi
	    else
	       Usage
	       exit 1
        fi
        shift
        ;;

   esac
done

if [[ "${flag_start}" != "start" || "${flag_stop}" != "stop" ]] ; then
   Usage
   exit 1
fi
exit 0
