#!/bin/bash
#
# Replace existing copyright notices with current text.
#
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

LICENSEDIR="${TL_DIR}/CodeTemplates"
TMP="/var/tmp"

C1="${TMP}/cm1"
C2="${TMP}/cm2"
C3="${TMP}/cm3"
C4="${TMP}/cm4"
C5="${TMP}/cm5"
C6="${TMP}/cm6"
C7="${TMP}/cm7"
C8="${TMP}/cm8"
C9="${TMP}/cm9"

BEGIN="BEGIN_ICS_COPYRIGHT"
END="END_ICS_COPYRIGHT"

TEST=n			# If "n", simulate the conversion process without altering files.
SHOW=n			# If "y", list the files that would be changed but doe not change them.
ERROR=n			# If "y", keep going after a failure
CLEANUP=n		# If "y", remove the backup files.
VERBOSE=n		# If "y", display files with no copyright banner
VERYVERBOSE=n	# If "y", display all files during copyright replacement
SOURCE="${PWD}" # defaults to the current directory.
CONLY=n			# If "y", only look at C language files.

CMD=`basename $0`

USAGE="Usage: $CMD [-fecvVh] <directory>"

function usage {
	echo >&2
	echo $USAGE >&2
	echo >&2
	echo "  -t Test (tests the update process but does not alter the files.)" >&2
	echo "  -s Show (Lists files that will be changed. Does not make changes.)" >&2
	echo "  -c Cleanup (delete backup files on success)" >&2
	echo "  -e Continue on error (default is to terminate.)" >&2
	echo "  -v Verbose (print warning for files with no copyright)" >&2
	echo "  -V VeryVerbose (print all files with or without copyright)" >&2
	echo "  -C Consider only C language files (\".c\" and "\.h" files)" >&2
	echo "  -h Help (display help)" >&2
	echo "  <directory> Top of directory to work in." >&2
	echo >&2
}
	
function help {
	usage
	echo "	Copyrights will be inserted from files:" >&2
	echo "	$C1" >&2
	echo "	$C2" >&2
	echo "	$C3" >&2
	echo "	$C4" >&2
	echo "	$C5" >&2
	echo "	$C6" >&2
	echo "	$C7" >&2
	echo "	$C8" >&2
	echo "	$C9" >&2
	echo >&2
}
	
function build_license_files {
	for i in 1 2 3 4 5 6 7 8 9; do
		awk "/$BEGIN/ {flag=1; next} /$END/{flag=0} flag" ${LICENSEDIR}/cm${i} >${TMP}/cm${i}
	done
}

function cleanup_license_files {
	rm -f ${TMP}/cm[1-9]
}

COPYRIGHT_LIST="${TMP}/copyright_list.txt"
NO_COPYRIGHT_LIST="${TMP}/no_copyright_list.txt"

function build_file_lists {
	FILE="$*"
	if [ "$VERBOSE" == "y" ]; then
		echo "Testing ${FILE}"
	else
		echo -n "."
	fi

	grep -q "${BEGIN}" "${FILE}"
	if [ $? -eq 0 ]; then
		echo $* >>${COPYRIGHT_LIST}
	else
		echo $* >>${NO_COPYRIGHT_LIST}
	fi
}

function replace_copyright {
	FILE="$*"
	grep -q "${BEGIN}" "$FILE"
	if [ $? -eq 0 ]; then
		if [ "${VERBOSE}" == "y" ]; then
			echo "Replacing license in $FILE"
		else
			echo -n "."
		fi

		if [ "${TEST}" == "y" ]; then
			OUTFILE="/dev/null"
		else
			OUTFILE="${FILE}"
		fi
	
		cp "${FILE}" "${FILE}.bak"
		awk -v W=$VERBOSE -v V=$VERYVERBOSE "\
			{ print } \
				/${BEGIN}1/ { W=N; system(\"cat $C1\") ; do { getline X } while ( X !~ /${END}1/ ); print X} \
				/${BEGIN}2/ { W=N; system(\"cat $C2\") ; do { getline X } while ( X !~ /${END}2/ ); print X} \
				/${BEGIN}3/ { W=N; system(\"cat $C3\") ; do { getline X } while ( X !~ /${END}3/ ); print X} \
				/${BEGIN}4/ { W=N; system(\"cat $C4\") ; do { getline X } while ( X !~ /${END}4/ ); print X} \
				/${BEGIN}5/ { W=N; system(\"cat $C5\") ; do { getline X } while ( X !~ /${END}5/ ); print X} \
				/${BEGIN}6/ { W=N; system(\"cat $C6\") ; do { getline X } while ( X !~ /${END}6/ ); print X} \
				/${BEGIN}7/ { W=N; system(\"cat $C7\") ; do { getline X } while ( X !~ /${END}7/ ); print X} \
				/${BEGIN}8/ { W=N; system(\"cat $C8\") ; do { getline X } while ( X !~ /${END}8/ ); print X} \
				/${BEGIN}9/ { W=N; system(\"cat $C9\") ; do { getline X } while ( X !~ /${END}9/ ); print X}"\
			"${FILE}.bak" >"${OUTFILE}"
		if [ $? -ne 0 ]; then
			echo "ERROR: Failed to insert copyright." >&2
			if [ "${ERROR}" -eq "y" ]; then
				exit 2;
			fi
		fi
	elif [ "${VERBOSE}" == "y" ]; then
		echo "WARNING: '$FILE' contains no copyright banner" >&2
	fi
}

#
# Process command line arguments.
#
while getopts "tscevVCh" arg ; do
	case $arg in
	t ) TEST=y
		;;
	s ) SHOW=y
		;;
	c ) CLEANUP=y
		;;
	e ) ERROR=y
		;;
	v ) VERBOSE=y
		;;
	V ) VERBOSE=y
		VERYVERBOSE=y
		;;
	C ) CONLY=y
		;;
	h ) help
		exit 0
		;;
	* ) usage
		exit 2
		;;
	esac
done

shift $(($OPTIND - 1))

if [ $# -gt 1 ] ; then
	usage
	exit 2
elif [ $# -eq 1 ]; then
	SOURCE=$1
fi

if [ -z ${TL_DIR} ]; then
	echo "ERROR: You must set the build environment before running this script."
	exit 2
fi

#
# Verify that source tree exists
#
if [ ! -e "$SOURCE" ] ; then
	echo "Error: No $SOURCE found." >&2
	exit 2
fi

#
# What files will we consider?
#
if [ "${CONLY}" == "y" ]; then
	SOURCEPATTERN='*.[c,h]'
else
	SOURCEPATTERN='*'
fi

if [ "${SHOW}" == "y" ]; then
	rm -f ${COPYRIGHT_LIST} ${NO_COPYRIGHT_LIST}

	find ${SOURCE} -type f -name "${SOURCEPATTERN}" | grep -v ".bak" | grep -v "CVS" | while read file ; do
		build_file_lists "${file}"
	done

	echo "------------------------------------------"
	echo "Files which have copyright notices:"
	echo "------------------------------------------"
	sort ${COPYRIGHT_LIST}
	echo "------------------------------------------"
	echo "Files which DO NOT have copyright notices:"
	echo "------------------------------------------"
	sort ${NO_COPYRIGHT_LIST}

	if [ "${CLEANUP}" == "y" ]; then
		rm -f ${COPYRIGHT_LIST} ${NO_COPYRIGHT_LIST}
	fi
else
	build_license_files

	#
	# Insert copyrights.
	#
	echo "Inserting copyright statements:"
	find $SOURCE -type f -name "${SOURCEPATTERN}" | grep -v ".bak" | grep -v "CVS" | while read file ; do
		replace_copyright "${file}"
	done
	echo
	echo "Done inserting copyright statements."
	echo

	if [ "${CLEANUP}" == "y" ]; then
		cleanup_license_files
		find $SOURCE -name "*.bak" -type f | xargs rm
	fi
fi
exit 0
