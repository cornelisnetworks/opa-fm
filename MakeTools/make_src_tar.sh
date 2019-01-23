# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015-2017, Intel Corporation
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

#[ICS VERSION STRING: unknown]
#!/bin/sh
# This script is used to to create source tar for rpm
# Required arguments: $RELEASE_TYPE $SrcRoot $USE_UNIFDEF $ARCHIVE $FILES_TO_TAR $FILE_TO_EXCLUDE
set -x
RELEASE_TYPE="$1"
SrcRoot="$2"
USE_UNIFDEF="$3"
ARCHIVE="$4"
FILES_TO_TAR="$5"
FILE_TO_EXCLUDE="$6"

comp_arg=
if echo $ARCHIVE | grep opa-fm.tar.gz > /dev/null
then
	comp_arg=HFM
fi
if echo $ARCHIVE | grep opa.tgz > /dev/null
then
	comp_arg=FF
fi
if echo $ARCHIVE | grep opa-sim.tgz > /dev/null
then
	comp_arg=SIM
fi

if [[ "${RELEASE_TYPE}" == "EMBARGOED" ]]
then
        echo "Building Embargoed Release."
        export OPA_LICENSE_DIR=${TL_DIR}/CodeTemplates/EmbargoedNotices
        export LICENSE_FUNCS=${TL_DIR}/CodeTemplates/license_funcs.sh
        # use tar command to copy files to temp dir
        # then run update_all_licenses on files in temp dir
        TMP=$(mktemp -d)
        tar c -C ${SrcRoot} --exclude-vcs --ignore-case $FILE_TO_EXCLUDE ${FILES_TO_TAR} | \
        	tar x -C ${TMP}
        ${TL_DIR}/CodeTemplates/update_all_licenses.sh -c -q $TMP
        SrcRoot=${TMP}
else
        echo "Building Public Release."
fi

SrcTmp=$(mktemp -d)

# Use tar command to copy files to temp dirs
# For unifdef, then run unifdef on files in temp dir

tar c -C ${SrcRoot} --exclude-vcs --ignore-case $FILE_TO_EXCLUDE ${FILES_TO_TAR} | \
   	tar x -C ${SrcTmp}
if [[ $USE_UNIFDEF = "yes" ]] ; then
        # Check if unifdef tool exists
        type unifdef > /dev/null 2>&1 || { echo "error: unifdef tool missing"; exit 1; }
        find ${SrcTmp}/ -type f -regex ".*\.[ch][p]*$" -exec unifdef -m -f "${TL_DIR}/Fd/buildFeatureDefs" {} \;
        # Delete content of buildFeatureDefs before creating source tar for rpm
        mkdir -p ${SrcTmp}/Fd
        >${SrcTmp}/Fd/buildFeatureDefs
        >${SrcTmp}/Fd/buildFeatureDefs.base
fi

# Copy processed base files as .base 

while read comp fn
do
	if echo $comp | grep $comp_arg > /dev/null
	then
		cp ${TL_DIR}/$fn ${SrcTmp}/${fn}.base
	fi
done < ${TL_DIR}/Fd/BaseFiles

tar cvzf ${ARCHIVE} -C ${SrcTmp} --exclude-vcs --ignore-case $FILE_TO_EXCLUDE ${FILES_TO_TAR}

# Remove temp dir
if [[ "${RELEASE_TYPE}" == "EMBARGOED" ]] ; then
        rm -rf ${TMP}
fi
rm -rf ${SrcTmp}
