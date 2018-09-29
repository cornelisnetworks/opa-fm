#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
#
# Copyright (c) 2018, Intel Corporation
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
PROG_NAME=$0

TEMP_CONFIG="$(mktemp opafm-XXXXXX.xml)"
TEMP_CHECK="check.${TEMP_CONFIG}.out"

Handle_Exit()
{
    RET=$?
    if [ $RET -ne 2 ]; then
        rm -f $TEMP_CONFIG $TEMP_CHECK
    fi
}
Handle_Signal()
{
    rm -f $TEMP_CONFIG $TEMP_CHECK
    exit 1
}
trap "Handle_Exit" EXIT
trap "Handle_Signal" SIGHUP SIGTERM SIGINT

PP_FILE=/etc/opa-fm/opafm_pp.xml
OUTPUT_XML=/etc/opa-fm/opafm.xml
BACKUP=0
SKIP_CHECK=0
FORCE_WR=0

Usage()
{
    echo "Usage: $(basename $PROG_NAME) [-p ppfile] [-o output] [-b] [-s] [-f] [-h]" >&2
    echo " " >&2
    echo "      -p   File to preprocess [Default=\"${PP_FILE}\"]" >&2
    echo "      -o   Output XML file [Default=\"${OUTPUT_XML}\"]" >&2
    echo "      -b   Backup old output file if exists [Default=${BACKUP}]" >&2
    echo "      -s   Skip running Config Check on output [Default=${SKIP_CHECK}]" >&2
    echo "      -f   Force overwrite of old output file [Default=${FORCE_WR}]" >&2
    echo "      -h   Print this help message" >&2
    echo "Notes:" >&2
    echo " Output file will contain 2 comments at the top to indicate it was" >&2
    echo "    generated and the local time it was generated" >&2
    echo " The special include comments should follow this style: " >&2
    echo "    <!-- INCLUDE:DG_DIR=\$DG_DIR --> " >&2
    echo "    <!-- INCLUDE:VF_DIR=\$VF_DIR --> " >&2
    echo " Examples: " >&2
    echo "    <!-- INCLUDE:VF_DIR=/etc/opa-fm/vfs --> " >&2
    echo "    <!-- INCLUDE:VF_DIR=/etc/opa-fm/vfs_fm1 --> " >&2
    echo "    <!-- INCLUDE:DG_DIR=/etc/opa-fm/dgs --> " >&2
    echo "    <!-- INCLUDE:DG_DIR=/etc/opa-fm/dgs_fm0 --> " >&2
    echo " Include comments will be replaced with contents of given directories" >&2
    echo " " >&2

    exit 1
}

# This function will parse the given file (ARG 1) and echo to stdout
Parse_PP()
{
    if [ ! -f "$1" ]; then
        echo "Error: preprocess file could not be found: \"$1\"" >&2
        exit 1
    fi

    echo "Parsing $1 ..." >&2

    # Keep first line at top of file
    sed -n "1p" $1

    # Add Generated Comments to the Top of the File
    echo "<!-- Generated file. Do not edit. -->"
    echo "<!-- Generated Date: $(date) -->"

    # start at line 2
    LN=2

    # Parse the rest of the pp file
    sed "1d" $1 | while IFS= read -r line; do

        DG_DIR=$(echo $line | \
            sed -n "s/^[[:space:]]*<[!]--[[:space:]]*INCLUDE:DG_DIR=\(.*\)-->/\1/p" | \
            sed "s/[[:space:]]*$//") # Trailing whitespace needs to be run again
        if [ -d "$DG_DIR" ]; then
            echo "Found DG_DIR at line $LN, Adding DeviceGroups from \"$DG_DIR\"" >&2
            cat "${DG_DIR}"/*
            continue
        fi
        VF_DIR=$(echo $line | \
            sed -n "s/^[[:space:]]*<[!]--[[:space:]]*INCLUDE:VF_DIR=\(.*\)-->/\1/p" | \
            sed "s/[[:space:]]*$//")
        if [ -d "$VF_DIR" ]; then
            echo "Found VF_DIR at line $LN, Adding VirtualFabrics from \"$VF_DIR\"" >&2
            cat "${VF_DIR}"/*
            continue
        fi
        echo "$line"
        LN=$((LN +1))
    done
}

# This function will attempt to copy the Temp XML file (ARG 2) to the
# Output XML file (ARG 3) and if needed backup (ARG 1) the old Output XML file
Config_Copy()
{
    BK=$1
    IN_XML=$2
    OUT_XML=$3

    if [ -f "$OUT_XML" -a $BK -eq 1 ]; then
        echo "Backing up old $OUT_XML to ${OUT_XML}.bak"
        cp "$OUT_XML" "${OUT_XML}.bak"
    fi
    cp "$IN_XML" "$OUT_XML"
}

while getopts ":hp:o:bfs" opt; do
    case $opt in
    h)
        Usage
        ;;
    p)
        PP_FILE=$OPTARG
        ;;
    o)
        OUTPUT_XML=$OPTARG
        ;;
    b)
        BACKUP=1
        ;;
    f)
        FORCE_WR=1
        ;;
    s)
        SKIP_CHECK=1
        ;;
    ?)
        echo "Error: $OPTARG is not a valid argument!" >&2
        Usage
    esac
done
shift $((OPTIND -1))

if [ ! -f "$PP_FILE" ]; then
    echo "Error: preprocess file not found: \"$PP_FILE\"" >&2
    Usage
fi

while [ $FORCE_WR -eq 0 -a $BACKUP -eq 0 -a -f "$OUTPUT_XML" ]; do
    echo "Warning: $OUTPUT_XML will be overwritten!"
    echo -n "Do you want to continue? [y/n] "
    read INPUT
    if [[ $INPUT =~ [nN] ]]; then
        exit 0
    elif [[ $INPUT =~ [yY] ]]; then
        break
    fi
done

Parse_PP "$PP_FILE" > "$TEMP_CONFIG"

if [ $SKIP_CHECK -ne 1 ]; then
    echo "Checking Config ..." >&2
    # Check Config (-s/strict = MC and VF checking)
    /usr/lib/opa-fm/bin/config_check -c "$TEMP_CONFIG" -s 2>&1 | tee "$TEMP_CHECK"
    # Get return code from config_check
    RET=${PIPESTATUS[0]}
    if [ $RET -eq 0 ]; then
        echo "Config Check Passed!"
    else
        echo "Error: Config Check Failed: Review \"$TEMP_CHECK\"" >&2
        exit 2
   fi
fi

Config_Copy $BACKUP "$TEMP_CONFIG" "$OUTPUT_XML"

exit 0
