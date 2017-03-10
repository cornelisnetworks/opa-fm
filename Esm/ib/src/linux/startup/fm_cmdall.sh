#!/bin/sh
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

CONFIG_DIR=/etc
CONFIG_FILE=$CONFIG_DIR/opa-fm/opafm.xml
IFS_FM_BASE=/usr/lib/opa-fm # default

PROGNAME=`basename $0`

Usage() {
	echo "Usage: $PROGNAME [-i fm_instance] cmd [args]" >&2
	echo "      or $PROGNAME --help " >&2
	echo " " >&2
	echo "	--help           - show this help text." >&2
	echo "	-i fm_instance   - FM instance number (0-7) to act on" >&2
	echo "                     specify multiple -i options as needed" >&2
	echo "                     default is all instances" >&2
	echo " " >&2
	echo " See opafmcmd for more details." >&2
	exit 2
}

Usage_full() {
	echo "Usage: $PROGNAME [-i fm_instance] cmd [args]" >&2
	echo "      or $PROGNAME --help " >&2
	echo " " >&2
	echo "	--help		 - show this help text." >&2
	echo "	-i fm_instance	 - FM instance number (0-7) to act on" >&2
	echo "			   specify multiple -i options as needed" >&2
	echo "			   default is all instances" >&2
	echo " " >&2
	echo "$PROGNAME executes a command to all FM instances listed" >&2
	echo "in the options. By default, the command is sent to all" >&2
	echo "instances of the FM. See opafmcmd for more details" >&2 
	exit 0
}

die () {
	echo "$@" > /dev/stderr

	echo "removing $dumpLocation"
	rm -rf $dumpLocation
	exit 1
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

INSTANCES=
while getopts i: param
do
	case $param in
	i)	INSTANCES="$INSTANCES $OPTARG";;
	?)	Usage;;
	esac
done
shift $((OPTIND -1))

if [ $# -le 0 ]
then
	echo "$0: Error: Must specify a command" >&2 
	Usage
fi

if [ -z "$INSTANCES" ]
then
	INSTANCES="0 1 2 3 4 5 6 7"
	default=y
else
	default=n
fi

running_manager()
# $1 = instance number
{
	pgrep -f "(fe|sm) (-D )?-e (fe|sm)_$1" > /dev/null 2>&1
}

found_one=n
for i in $INSTANCES
do
	if running_manager $i
	then
		found_one=y
		$IFS_FM_BASE/bin/fm_cmd -i $i "$@"
		if [ $? -eq 255 ]
		then
			break	# 255 is used as a Usage return, no use iterating
		fi
	else
		if [ "$default" != "y" ]
		then
			echo "Skipping FM instance $i: Not Running"
		fi
	fi
done

if [ "$found_one" = n -a "$default" = y ]
then
	echo "No running FM instances found"
fi
