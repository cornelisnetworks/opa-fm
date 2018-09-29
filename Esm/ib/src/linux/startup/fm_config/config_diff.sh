#!/bin/bash
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

# [ICS VERSION STRING: unknown]

TEMP1="$(mktemp)"
TEMP2="$(mktemp)"
PROG_NAME=$0
trap "rm -f $TEMP1 $TEMP2; exit 1" SIGHUP SIGTERM SIGINT
trap "rm -f $TEMP1 $TEMP2" EXIT

Usage()
{
	echo "Usage: $(basename $PROG_NAME) [-f][-l] [-d 'diff_args'] file1 file2" >&2
	echo "       or $(basename $PROG_NAME) --help" >&2
	echo " " >&2
	echo "      --help - produce full help text" >&2
	echo "      -f	   - filter out FM parameters which are not part of consistency check" >&2
	echo "               Removes config tags that do not cause consistency checks on the" >&2
	echo "               FM to fail from diff" >&2
	echo "      -l     - include comments in XML to indicate original line numbers" >&2
	echo "      -d 'diff_args' - additional arguments to diff command" >&2
	echo "               e.g. -uw for unified format ignoring whitespace." >&2
	echo " " >&2
	exit 2
}

Usage_full()
{
	echo "Usage: $(basename $PROG_NAME) [-f][-l] [-d 'diff_args'] file1 file2" >&2
	echo "       or $(basename $PROG_NAME) --help" >&2
	echo " " >&2
	echo "      --help - produce full help text" >&2
	echo "      -f     - filter out FM parameters which are not part of consistency check" >&2
	echo "               Removes config tags that do not cause consistency checks on the" >&2
	echo "               FM to fail from diff" >&2
	echo "      -l     - include comments in XML to indicate original line numbers" >&2
	echo "      -d 'diff_args' - additional arguments to diff command" >&2
	echo "               e.g. -uw for unified format ignoring whitespace." >&2
	echo " " >&2
	echo "$(basename $PROG_NAME) performs a difference between two config files corresponding" >&2
	echo "to two FM instances described by file1 and file2" >&2
	echo " " >&2
	echo "Examples:" >&2
	echo "  $(basename $PROG_NAME) /etc/opa-fm/opafm.xml /usr/share/opa-fm/opafm.xml" >&2
	echo "  $(basename $PROG_NAME) -f /etc/opa-fm/opafm.xml /usr/share/opa-fm/opafm.xml" >&2
	echo "  $(basename $PROG_NAME) -d -uw /etc/opa-fm/opafm.xml /usr/share/opa-fm/opafm.xml" >&2
	exit 0

}

# Main function

if [ x"$1" = "x--help" ]
then
    Usage_full
fi

IFS_FM_BASE=/usr/lib/opa-fm
tooldir=$IFS_FM_BASE/bin

if [ x"$1" = "x--help" ]
then
        Usage
fi

filter=n
filter_args=
diff_args=
while getopts fld: param
do
	case $param in
	f)	filter=y;;
	l)	filter_args="$filter_args -l";;
	d)	diff_args="$diff_args $OPTARG";;
	?)	Usage;;
	esac
done
shift $((OPTIND -1))

if [ $# -lt 2 ]
then
	Usage
fi
file1=$1
file2=$2

if [ ! -e $file1 ]
then
	echo "$file1: Not Found" >&2
	Usage
fi
if [ ! -e $file2 ]
then
	echo "$file2: Not Found" >&2
	Usage
fi

if [ x"$filter" = xy ]
then
	filter_args="$filter_args -s Start -s Hca -s Port -s Debug -s RmppDebug -s Priority -s ElevatedPriority -s LogLevel -s LogFile -s LogMode -s SyslogFacility 
-s *_LogMask -s Name -s PortGUID -s Sm.TrapLogSuppressTriggerInterval -s Sm.SmPerfDebug -s Sm.SaPerfDebug -s Sm.DebugDor -s Sm.DebugVf -s Sm.DebugJm -s Sm.DebugLidAssign -s Sm.LoopTestOn -s Sm.LoopTestPacket -s Sm.LID -s Sm.DynamicPortAlloc -s Sm.SaRmppChecksum -s Pm.ThresholdsExceededMsgLimit -s Pm.SweepErrorsLogThreshold -s Bm.DebugFlag -s Fe.TcpPort"
fi

$tooldir/opafmxmlfilter -i 4 $filter_args $file1 > $TEMP1
[ $? != 0 ] && exit 1

$tooldir/opafmxmlfilter -i 4 $filter_args $file2 > $TEMP2
[ $? != 0 ] && exit 1

diff $diff_args $TEMP1 $TEMP2
res=$?

rm -f $TEMP1 $TEMP
exit $res
