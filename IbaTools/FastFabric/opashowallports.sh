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
	echo "Usage: opashowallports [-C] [-f hostfile] [-F chassisfile]" >&2
	echo "                    [-h 'hosts'] [-H 'chassis'] [-S]" >&2
	echo "              or" >&2
	echo "       opashowallports --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -C - perform operation against chassis, default is hosts" >&2
	echo "   -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "   -h hosts - list of hosts to show ports for" >&2
	echo "   -H chassis - list of chassis to show ports for" >&2
	echo "   -S - securely prompt for password for admin on chassis" >&2
	echo " Environment:" >&2
	echo "   HOSTS - list of hosts, used if -h option not supplied" >&2
	echo "   CHASSIS - list of chassis, used if -H option not supplied" >&2
	echo "   HOSTS_FILE - file containing list of hosts, used in absence of -f and -h" >&2
	echo "   CHASSIS_FILE - file containing list of chassis, used in absence of -F and -H" >&2
	echo "   FF_CHASSIS_LOGIN_METHOD - how to login to chassis: telnet or ssh" >&2
	echo "   FF_CHASSIS_ADMIN_PASSWORD - admin password for chassis, used in absence of -S" >&2
	echo "example:">&2
	echo "   opashowallports" >&2
	echo "   opashowallports -h 'elrond arwen'" >&2
	echo "   HOSTS='elrond arwen' opashowallports" >&2
	echo "   opashowallports -C" >&2
	echo "   opashowallports -H 'chassis1 chassis2'" >&2
	echo "   CHASSIS='chassis1 chassis2' opashowallports -C" >&2
	exit 0
}


Usage()
{
	echo "Usage: opashowallports [-C] [-f hostfile] [-F chassisfile] [-S]" >&2
	echo "              or" >&2
	echo "       opashowallports --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -C - perform operation against chassis, default is hosts" >&2
	echo "   -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "   -S - securely prompt for password for admin on chassis" >&2
	echo "example:">&2
	echo "   opashowallports" >&2
	echo "   opashowallports -C" >&2
	exit 2
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

host=0
chassis=0
Sopt=n
while getopts Cf:F:h:H:S param
do
	case $param in
	C)
		chassis=1;;
	f)
		host=1
		HOSTS_FILE="$OPTARG";;
	F)
		chassis=1
		CHASSIS_FILE="$OPTARG";;
	h)
		host=1
		HOSTS="$OPTARG";;
	H)
		chassis=1
		CHASSIS="$OPTARG";;
	S)
		Sopt=y;;
	?)
		Usage;;
	esac
done
shift $((OPTIND -1))
if [[ $# -gt 0 ]]
then
	Usage
fi
if [[ $(($chassis+$host)) -gt 1 ]]
then
	echo "opashowallports: conflicting arguments, both host and chassis specified" >&2
	Usage
fi
if [[ $(($chassis+$host)) -eq 0 ]]
then
	host=1
fi

if [ "$chassis" -eq 0 ]
then

	check_host_args opashowallports
	for hostname in $HOSTS
	do
		echo "--------------------------------------------------------------------"
		echo "$hostname:"
		/usr/lib/opa/tools/tcl_proc hosts_run_cmd "$hostname" "root" '/usr/sbin/opainfo' 1
	done
else

	check_chassis_args opashowallports
	export CFG_CHASSIS_LOGIN_METHOD=$FF_CHASSIS_LOGIN_METHOD
	export CFG_CHASSIS_ADMIN_PASSWORD=$FF_CHASSIS_ADMIN_PASSWORD
	if [ "$Sopt" = y ]
	then
		echo -n "Password for admin on all chassis: " > /dev/tty
		stty -echo < /dev/tty > /dev/tty
		password=
		read password < /dev/tty
		stty echo < /dev/tty > /dev/tty
		echo > /dev/tty
		export CFG_CHASSIS_ADMIN_PASSWORD="$password"
	fi
	for chassis in $CHASSIS
	do
		chassis=`strip_chassis_slots "$chassis"`
		echo "--------------------------------------------------------------------"
		echo "$chassis:"
		/usr/lib/opa/tools/tcl_proc chassises_run_cmd "$chassis" "admin" 'ismPortStats -noprompt' 1 2>&1|egrep 'FAIL|Port State|Link Qual|Link Width|Link Speed|^[[:space:]]|^Name' | egrep -v 'Tx|Rx'
	done
fi
