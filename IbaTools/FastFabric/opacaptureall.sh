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
# perform an opacapture on all hosts/chassis and upload to this host

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
	echo "Usage: opacaptureall [-Cp] [-f hostfile] [-F chassisfile]" >&2
	echo "                  [ -h 'hosts'] [-H 'chassis'] [-t portsfile]" >&2
    echo "                  [-d upload_dir] [-S] [-D detail_level] [file]" >&2
	echo "              or" >&2
	echo "       opacaptureall --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -C - perform capture against chassis, default is hosts" >&2
	echo "   -p - perform capture upload in parallel on all hosts/chassis" >&2
	echo "   -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "   -h hosts - list of hosts to perform capture of" >&2
	echo "   -H chassis - list of chassis to perform capture of" >&2
	echo "   -t portsfile - file with list of local HFI ports used to access" >&2
	echo "           fabric(s) for switch access, default is /etc/opa/ports" >&2
	echo "   -d upload_dir - directory to upload to, default is uploads" >&2
	echo "   -S - securely prompt for password for admin on chassis" >&2
	echo "   -D detail_level - level of detail passed to host opacapture" >&2
	echo "           1-Local 2-Fabric 3-Fabric+FDB 4-Analysis" >&2 
	echo "   file - name for capture file [.tgz will be appended]" >&2
	echo " Environment:" >&2
	echo "   HOSTS - list of hosts, used if -h option not supplied" >&2
	echo "   CHASSIS - list of chassis, used if -C used and -h option not supplied" >&2
	echo "   HOSTS_FILE - file containing list of hosts, used in absence of -f and -h" >&2
	echo "   CHASSIS_FILE - file containing list of chassis, used in absence of -F and -H" >&2
	echo "   UPLOADS_DIR - directory to upload to, used in absence of -d" >&2
	echo "   FF_MAX_PARALLEL - when -p option is used, maximum concurrent operations" >&2
	echo "   FF_CHASSIS_LOGIN_METHOD - how to login to chassis: telnet or ssh" >&2
	echo "   FF_CHASSIS_ADMIN_PASSWORD - admin password for chassis, used in absence of -S" >&2
	echo "example:">&2
	echo "  Operations on hosts" >&2
	echo "   opacaptureall" >&2
	echo "   opacaptureall mycapture" >&2
	echo "   opacaptureall -h 'arwen elrond' 030127capture" >&2
	echo "  Operations on chassis" >&2
	echo "   opacaptureall -C" >&2
	echo "   opacaptureall -C mycapture" >&2
	echo "   opacaptureall -C -H 'chassis1 chassis2' 030127capture" >&2
	echo "For hosts:" >&2
	echo "   opacapture will be run to create the specified capture file within ~root" >&2
	echo "   on each host (with the .tgz suffix added). The files will be" >&2
	echo "   uploaded and unpacked into a matching directory name within" >&2
	echo "   upload_dir/hostname/ on the local system" >&2
	echo "   default file name is hostcapture" >&2
	echo "For Chassis:" >&2
	echo "   The capture CLI command will be run on each chassis and its output will be" >&2
	echo "   saved to upload_dir/chassisname/file on the local system" >&2
	echo "   default file name is chassiscapture" >&2
	echo "The uploaded captures will be combined into a tgz file with the file name" >&2
	echo "specified and the suffix .all.tgz added" >&2
	exit 0
}

Usage()
{
	echo "Usage: opacaptureall [-Cp] [-f hostfile] [-F chassisfile] [-S]" >&2
	echo "                  [-D detail_level] [file]" >&2
	echo "              or" >&2
	echo "       opacaptureall --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -C - perform capture against chassis, default is hosts" >&2
	echo "   -p - perform capture upload in parallel on all hosts/chassis" >&2
	echo "   -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "   -S - securely prompt for password for admin on chassis or switch" >&2
	echo "   -D detail_level - level of detail passed to host opacapture" >&2
	echo "   file - name for capture file [.tgz will be appended]" >&2
	echo "example:">&2
	echo "  Operations on hosts" >&2
	echo "   opacaptureall" >&2
	echo "   opacaptureall mycapture" >&2
	echo "  Operations on chassis" >&2
	echo "   opacaptureall -C" >&2
	echo "   opacaptureall -C mycapture" >&2
	echo "For hosts:" >&2
	echo "   opacapture will be run to create the specified capture file within ~root" >&2
	echo "   on each host (with the .tgz suffix added). The files will be" >&2
	echo "   uploaded and unpacked into a matching directory name within" >&2
	echo "   uploads/hostname/ on the local system" >&2
	echo "   default file name is hostcapture" >&2
	echo "For Chassis:" >&2
	echo "   The capture CLI command will be run on each chassis and its output will be" >&2
	echo "   saved to uploads/chassisname/file on the local system" >&2
	echo "   default file name is chassiscapture" >&2
	echo "The uploaded captures will be combined into a tgz file with the file name" >&2
	echo "specified and the suffix .all.tgz added" >&2
	exit 2
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

popt=n
chassis=0
host=0
Sopt=n
Dopt=
while getopts Cf:F:h:H:t:d:pD:S param
do
	case $param in
	C)
		chassis=1;;
	f)
		host=1
		export HOSTS_FILE="$OPTARG";;
	F)
		chassis=1
		export CHASSIS_FILE="$OPTARG";;
	h)
		host=1
		export HOSTS="$OPTARG";;
	H)
		chassis=1
		CHASSIS="$OPTARG";;
	t)
		export PORTS_FILE="$OPTARG";;
	d)
		export UPLOADS_DIR="$OPTARG";;
	p)
		popt=y;;
	D)
		Dopt="-d $OPTARG";;
	S)
		Sopt=y;;
	?)
		Usage;;
	esac
done
shift $((OPTIND -1))
if [[ $# -gt 1 ]]
then
	Usage
fi
if [[ $(($chassis+$host)) -gt 1 ]]
then
	echo "opacaptureall: conflicting arguments, more than one of host, and chassis specified" >&2
	Usage
fi
if [[ $(($chassis+$host)) -eq 0 ]]
then
	host=1
fi

if [[ $# -gt 0 ]]
then
	file="$1"
else
	if [ $chassis -eq 0 ]
	then
		file=hostcapture
	else
		file=chassiscapture
	fi
fi
if [ `dirname $file` != . ]
then
	echo "filename for capture must not specify a directory" >&2
	Usage
fi
file=`basename $file .tgz`	# remove .tgz suffix if given
host_local=""

expand_host_capture()
{
# $1 = hostname
	if [ ! -f $UPLOADS_DIR/$1/${file}.tgz ]
	then
		echo "ERROR: capture and/or upload from $1 failed"
	else
		# chmod so we can remove
		chmod -R u+w $UPLOADS_DIR/$1/$file >/dev/null 2>/dev/null
		rm -rf $UPLOADS_DIR/$1/$file
		mkdir -p $UPLOADS_DIR/$1/$file
		( cd $UPLOADS_DIR/$1/$file; tar xfz ../${file}.tgz; chmod -R u+w . )
	fi
}

if [ $chassis -eq 0 ]
then
	check_host_args opacaptureall

	# remove local host from $HOSTS so that opacmdall will not run opacapture locally
	hosts_no_local=""
	lc_myhostname=$(hostname -s|tr A-Z a-z)
	for hostname in $HOSTS
	do
		lc_hostname=$(echo $hostname|tr A-Z a-z)
		if [ "$lc_hostname" != "$lc_myhostname" -a "$lc_hostname" != "localhost" ]
		then
			hosts_no_local="$hosts_no_local $hostname"
		else
			host_local="$hostname"
		fi
	done

	# run capture in parallel since its time consuming
	echo "Running capture on all non-local hosts ..."
	/usr/sbin/opacmdall -p -h "$hosts_no_local" -u "root" "rm -f ~root/${file}.tgz; opacapture ~root/${file}.tgz"

	echo "Uploading capture from each host ..."
	running=0
	captures=
	for hostname in $hosts_no_local
	do
		captures="$captures $hostname/$file"
		if [ "$popt" = "y" ]
		then
			if [ $running -ge $FF_MAX_PARALLEL ]
			then
				wait
				running=0
			fi
			(
			/usr/sbin/opauploadall -d $UPLOADS_DIR -u root -h "$hostname" ${file}.tgz .
			expand_host_capture "$hostname"
			) &
			running=$(( $running + 1))
		else
			/usr/sbin/opauploadall -d $UPLOADS_DIR -u root -h "$hostname" ${file}.tgz .
			expand_host_capture "$hostname"
		fi
	done
else
	check_chassis_args opacaptureall
	if [ "$Sopt" = y ]
	then
		echo -n "Password for admin on all chassis: " > /dev/tty
		stty -echo < /dev/tty > /dev/tty
		password=
		read password < /dev/tty
		stty echo < /dev/tty > /dev/tty
		echo > /dev/tty
		export FF_CHASSIS_ADMIN_PASSWORD="$password"
	fi

	echo "Running capture on all chassis ..."
	running=0
	captures=
	for chassis in $CHASSIS
	do
		chassis=`strip_chassis_slots "$chassis"`
		captures="$captures $chassis/$file"
		if [ "$popt" = "y" ]
		then
			if [ $running -ge $FF_MAX_PARALLEL ]
			then
				wait
				running=0
			fi
			(
			mkdir -p $UPLOADS_DIR/$chassis
			# filter out carriage returns
			/usr/sbin/opacmdall -C -H "$chassis" -m 'End of Capture' "capture" |tr -d '\015' > $UPLOADS_DIR/$chassis/$file
			if [ $? != 0 ]
			then
				echo "ERROR: capture from $chassis failed"
			fi
			) &
			running=$(( $running + 1))
		else
			mkdir -p $UPLOADS_DIR/$chassis
			# filter out carriage returns
			/usr/sbin/opacmdall -C -H "$chassis" -m 'End of Capture' "capture" |tr -d '\015' > $UPLOADS_DIR/$chassis/$file
			if [ $? != 0 ]
			then
				echo "ERROR: capture from $chassis failed"
			fi
		fi
	done
fi

if [ "$host_local" == "" -a "x$Dopt" != "x" ]
then
	host_local=$(hostname -s)
fi
if [ "$host_local" != "" ]
then
	echo "Running capture on local host ..."
	rm -f $UPLOADS_DIR/$host_local/$file.tgz
	mkdir -p $UPLOADS_DIR/$host_local
	/usr/sbin/opacapture $Dopt $UPLOADS_DIR/$host_local/$file.tgz
	expand_host_capture "$host_local"
	captures="$captures $host_local/$file"
fi

wait
echo "Combining captured files into $UPLOADS_DIR/$file.all.tgz ..."
(cd $UPLOADS_DIR; rm -rf $file.all.tgz; tar --format=gnu -czf $file.all.tgz $captures --warning=no-file-changed)
echo "Done."
