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

# optional override of defaults
if [ -f /etc/opa/opafastfabric.conf ]
then
	. /etc/opa/opafastfabric.conf
fi

. /usr/lib/opa/tools/opafastfabric.conf.def

. /usr/lib/opa/tools/ff_funcs

trap "exit 1" SIGHUP SIGTERM SIGINT

Usage_full()
{
	echo "Usage: opapingall [-Cp] [-f hostfile] [-F chassisfile] [-h 'hosts']" >&2
	echo "                     [-H 'chassis']" >&2
	echo "              or" >&2
	echo "       opapingall --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -C - perform ping against chassis, default is hosts" >&2
	echo "        Merely selects applicable list, can ping both hosts and chassis" >&2
	echo "   -p - ping all hosts/chassis in parallel" >&2
	echo "   -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "        default is $CONFIG_DIR/opa/chassis" >&2
	echo "   -h hosts - list of hosts to ping" >&2
	echo "   -H chassis - list of chassis to ping" >&2
	echo " Environment:" >&2
	echo "   HOSTS - list of hosts, used if -h option not supplied" >&2
	echo "   CHASSIS - list of chassis, used if -H option not supplied" >&2
	echo "   HOSTS_FILE - file containing list of hosts, used in absence of -f and -h" >&2
	echo "   CHASSIS_FILE - file containing list of chassis, used in absence of -F and -H" >&2
	echo "   FF_MAX_PARALLEL - when -p option is used, maximum concurrent operations" >&2
	echo "example:">&2
	echo "   opapingall" >&2
	echo "   opapingall -h 'arwen elrond'" >&2
	echo "   HOSTS='arwen elrond' opapingall" >&2
	echo "   opapingall -C" >&2
	echo "   opapingall -C -H 'chassis1 chassis2'" >&2
	echo "   CHASSIS='chassis1 chassis2' opapingall -C" >&2
	exit 0
}

Usage()
{
	echo "Usage: opapingall [-Cp] [-f hostfile] [-F chassisfile]" >&2
	echo "              or" >&2
	echo "       opapingall --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -C - perform ping against chassis, default is hosts" >&2
	echo "        Merely selects applicable list, can ping both hosts and chassis" >&2
	echo "   -p - ping all hosts/chassis in parallel" >&2
	echo "   -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "        default is $CONFIG_DIR/opa/chassis" >&2
	echo "example:">&2
	echo "   opapingall" >&2
	echo "   opapingall -C" >&2
	exit 2
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

ping_dest()
{
	# $1 is the destination to ping
	ping_host $1
	if [ $? != 0 ]
	then
		echo "$1: doesn't ping"
	else
		echo "$1: is alive"
	fi
}

popt=n
host=0
chassis=0
while getopts Cf:F:h:H:p param
do
	case $param in
	C)
		chassis=1;;
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
	p)
		popt=y;;
	?)
		Usage;;
	esac
done
shift $((OPTIND -1))
if [ $# -gt 0 ]
then
	Usage
fi
if [[ $(($chassis+$host)) -gt 1 ]]
then
	echo "opapingall: conflicting arguments, host and chassis both specified" >&2
	Usage
fi
if [[ $(($chassis+$host)) -eq 0 ]]
then
	host=1
fi
if [ $chassis -eq 0 ]
then
	check_host_args opapingall
	DESTS="$HOSTS"
else
	check_chassis_args opapingall
	DESTS="$CHASSIS"
fi

running=0
for dest in $DESTS
do
	if [ $chassis -ne 0 ]
	then
		dest=`strip_chassis_slots "$dest"`
	fi
	if [ "$popt" = "y" ]
	then
		if [ $running -ge $FF_MAX_PARALLEL ]
		then
			wait
			running=0
		fi
		ping_dest $dest &
		running=$(($running + 1))
	else
		ping_dest $dest
	fi
done
wait
