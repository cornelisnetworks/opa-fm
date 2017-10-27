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
# run a command on all hosts or chassis

# optional override of defaults
if [ -f /etc/opa/opafastfabric.conf ]
then
	. /etc/opa/opafastfabric.conf
fi

. /usr/lib/opa/tools/opafastfabric.conf.def

TOOLSDIR=${TOOLSDIR:-/usr/lib/opa/tools}
BINDIR=${BINDIR:-/usr/sbin}

. $TOOLSDIR/ff_funcs

trap "exit 1" SIGHUP SIGTERM SIGINT

Usage_full()
{
	echo "Usage: opacmdall [-CpqPS] [-f hostfile] [-F chassisfile] [-h 'hosts']" >&2
	echo "                    [-H 'chassis'] [-u user] [-m 'marker'] [-T timelimit]" >&2
	echo "                    'cmd'" >&2
	echo "              or" >&2
	echo "       opacmdall --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -C - perform command against chassis, default is hosts" >&2
	echo "   -p - run command in parallel on all hosts/chassis" >&2
	echo "   -q - quiet mode, don't show command to execute" >&2
	echo "   -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "   -h hosts - list of hosts to execute command on" >&2
	echo "   -H chassis - list of chassis to execute command on" >&2
	echo "   -u user - user to perform cmd as" >&2
	echo "           for hosts default is current user code" >&2
	echo "           for chassis default is admin" >&2
	echo "   -S - securely prompt for password for user on chassis" >&2
	echo "   -m 'marker' - marker for end of chassis command output" >&2
	echo "           if omitted defaults to chassis command prompt" >&2
	echo "           this may be a regular expression" >&2
	echo "   -T timelimit - timelimit in seconds when running host commands" >&2
	echo "           default is -1 (infinite)" >&2
	echo "   -P      output hostname/chassis name as prefix to each output line" >&2
	echo "           this can make script processing of output easier" >&2
	echo " Environment:" >&2
	echo "   HOSTS - list of hosts, used if -h option not supplied" >&2
	echo "   CHASSIS - list of chassis, used if -C used and -H and -F options not supplied" >&2
	echo "   HOSTS_FILE - file containing list of hosts, used in absence of -f and -h" >&2
	echo "   CHASSIS_FILE - file containing list of chassis, used in absence of -F and -H" >&2
	echo "   FF_MAX_PARALLEL - when -p option is used, maximum concurrent operations" >&2
	echo "   FF_SERIALIZE_OUTPUT - serialize output of parallel operations (yes or no)" >&2
	echo "   FF_CHASSIS_LOGIN_METHOD - how to login to chassis: telnet or ssh" >&2
	echo "   FF_CHASSIS_ADMIN_PASSWORD - password for chassis, used in absence of -S" >&2
	echo "for example:" >&2
	echo "  Operations on hosts" >&2
	echo "   opacmdall date" >&2
	echo "   opacmdall 'uname -a'" >&2
	echo "   opacmdall -h 'elrond arwen' date" >&2
	echo "   HOSTS='elrond arwen' opacmdall date" >&2
	echo "  Operations on chassis" >&2
	echo "   opacmdall -C 'ismPortStats -noprompt'" >&2
	echo "   opacmdall -C -H 'chassis1 chassis2' 'ismPortStats -noprompt'" >&2
	echo "   CHASSIS='chassis1 chassis2' opacmdall -C 'ismPortStats -noprompt'" >&2
	exit 0
}

Usage()
{
	echo "Usage: opacmdall [-Cpq] [-f hostfile] [-F chassisfile] [-u user] [-S]" >&2
	echo "              [-T timelimit] [-P] 'cmd'" >&2
	echo "              or" >&2
	echo "       opacmdall --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -C - perform command against chassis, default is hosts" >&2
	echo "   -p - run command in parallel on all hosts/chassis" >&2
	echo "   -q - quiet mode, don't show command to execute" >&2
	echo "   -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "   -u user - user to perform cmd as" >&2
	echo "           for hosts default is current user code" >&2
	echo "           for chassis default is admin, this is ignored" >&2
	echo "   -S - securely prompt for password for user on chassis" >&2
	echo "   -T timelimit - timelimit in seconds when running host commands" >&2
	echo "           default is -1 (infinite)" >&2
	echo "   -P - output hostname/chassis name as prefix to each output line" >&2
	echo "           this can make script processing of output easier" >&2
	echo "for example:" >&2
	echo "  Operations on hosts" >&2
	echo "   opacmdall date" >&2
	echo "   opacmdall 'uname -a'" >&2
	echo "  Operations on chassis" >&2
	echo "   opacmdall -C 'ismPortStats -noprompt'" >&2
	exit 2
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

user=`id -u -n`
uopt=n
quiet=0
host=0
chassis=0
parallel=0
Sopt=n
marker=""
timelimit="-1"
output_prefix=0
while getopts Cqph:H:f:F:u:Sm:T:P param
do
	case $param in
	C)
		chassis=1;;
	q)
		quiet=1;;
	p)
		parallel=1;;
	h)
		host=1
		HOSTS="$OPTARG";;
	H)
		chassis=1
		CHASSIS="$OPTARG";;
	f)
		host=1
		HOSTS_FILE="$OPTARG";;
	F)
		chassis=1
		CHASSIS_FILE="$OPTARG";;
	u)
		uopt=y
		user="$OPTARG";;
	S)
		Sopt=y;;
	m)
		marker="$OPTARG";;
	T)
		timelimit="$OPTARG";;
	P)
		output_prefix=1;;
	?)
		Usage;;
	esac
done
shift $((OPTIND -1))

if [ $# -ne 1 ] 
then
	Usage
fi
if [[ $(($chassis+$host)) -gt 1 ]]
then
	echo "opacmdall: conflicting arguments, host and chassis both specified" >&2
	Usage
fi
if [[ $(($chassis+$host)) -eq 0 ]]
then
	host=1
fi
if [ x"$marker" != "x" -a $chassis -eq 0 ]
then
	echo "opacmdall: -m option only applicable to chassis, ignored" >&2
fi
if [ "$timelimit" -le 0 ]
then
	timelimit=0	# some old versions of expect mistakenly treat -1 as a option
fi

export TEST_MAX_PARALLEL="$FF_MAX_PARALLEL"
export TEST_TIMEOUT_MULT="$FF_TIMEOUT_MULT"
export CFG_LOGIN_METHOD="$FF_LOGIN_METHOD"
export CFG_PASSWORD="$FF_PASSWORD"
export CFG_ROOTPASS="$FF_ROOTPASS"
export CFG_CHASSIS_LOGIN_METHOD="$FF_CHASSIS_LOGIN_METHOD"
export CFG_CHASSIS_ADMIN_PASSWORD="$FF_CHASSIS_ADMIN_PASSWORD"
export TEST_SERIALIZE_OUTPUT="$FF_SERIALIZE_OUTPUT"

if [ $chassis -eq 0 ]
then
	check_host_args opacmdall
	if [ $parallel -eq 0 ]
	then
		tclproc=hosts_run_cmd
	else
		tclproc=hosts_parallel_run_cmd
	fi
	$TOOLSDIR/tcl_proc $tclproc "$HOSTS" "$user" "$1" $quiet $timelimit $output_prefix
else
	if [ "$uopt" = n ]
	then
		user=admin
	fi
	check_chassis_args opacmdall
	if [ "$Sopt" = y ]
	then
		echo -n "Password for $user on all chassis: " > /dev/tty
		stty -echo < /dev/tty > /dev/tty
		password=
		read password < /dev/tty
		stty echo < /dev/tty > /dev/tty
		echo > /dev/tty
		export CFG_CHASSIS_ADMIN_PASSWORD="$password"
	fi
	# pardon my spelling need plural chassis that is distinct from singular
	if [ $parallel -eq 0 ]
	then
		tclproc=chassises_run_cmd
	else
		tclproc=chassises_parallel_run_cmd
	fi
	$TOOLSDIR/tcl_proc $tclproc "$CHASSIS" "$user" "$1" $quiet "$marker" "$output_prefix"
fi
