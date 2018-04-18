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
	echo "Usage: opadownloadall [-rp] [-f hostfile] [-d download_dir] [-h 'hosts']" >&2
	echo "                         [-u user] source_file ... dest_file" >&2
	echo "              or" >&2
	echo "       opadownloadall --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -p - perform copy in parallel on all hosts" >&2
	echo "   -r - recursive download of directories" >&2
	echo "   -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "   -h hosts - list of hosts to download to" >&2
	echo "   -u user - user to perform copy to, default is current user code" >&2
	echo "   -d download_dir - directory to download from, default is downloads" >&2
	echo "   source_file - list of source files to copy" >&2
	echo "   dest_file - destination for copy" >&2
	echo "        If more than 1 source file, this must be a directory" >&2
	echo " Environment:" >&2
	echo "   HOSTS - list of hosts, used if -h option not supplied" >&2
	echo "   HOSTS_FILE - file containing list of hosts, used in absence of -f and -h" >&2
	echo "   DOWNLOADS_DIR - directory to download from, used in absence of -d" >&2
	echo "   FF_MAX_PARALLEL - when -p option is used, maximum concurrent operations" >&2
	echo "example:">&2
	echo "   opadownloadall -h 'arwen elrond' irqbalance vncservers $CONFIG_DIR" >&2
	echo "   opadownloadall -p irqbalance vncservers $CONFIG_DIR" >&2
	echo "user@ syntax cannot be used in filenames specified" >&2
	echo "A local directory within download_dir/ must exist for each hostname." >&2
	echo "Source file will be download_dir/hostname/source_file within the local system." >&2
	echo "To copy files from hosts in the cluster to this host use opauploadall." >&2
	exit 0
}

Usage()
{
	echo "Usage: opadownloadall [-rp] [-f hostfile] [-d download_dir]" >&2
	echo "                         source_file ... dest_file" >&2
	echo "              or" >&2
	echo "       opadownloadall --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -p - perform copy in parallel on all hosts" >&2
	echo "   -r - recursive download of directories" >&2
	echo "   -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "   -d download_dir - directory to download from, default is downloads" >&2
	echo "   source_file - list of source files to copy" >&2
	echo "   dest_file - destination for copy" >&2
	echo "        If more than 1 source file, this must be a directory" >&2
	echo "example:">&2
	echo "   opadownloadall -p irqbalance vncservers $CONFIG_DIR" >&2
	echo "user@ syntax cannot be used in filenames specified" >&2
	echo "A local directory within downloads/ must exist for each hostname." >&2
	echo "Source file will be downloads/hostname/source_file within the local system." >&2
	echo "To copy files from hosts in the cluster to this host use opauploadall." >&2
	exit 2
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

user=`id -u -n`
opts=
popt=n
while getopts f:d:h:u:rp param
do
	case $param in
	f)
		HOSTS_FILE="$OPTARG";;
	d)
		DOWNLOADS_DIR="$OPTARG";;
	h)
		HOSTS="$OPTARG";;
	u)
		user="$OPTARG";;
	r)
		opts="$opts -r";;
	p)
		opts="$opts -q"
		popt=y;;
	?)
		Usage;;
	esac
done
shift $((OPTIND -1))
if [ $# -lt 2 ]
then
	Usage
fi

check_host_args opadownloadall

# remove last name from the list
files=
dest=
for file in "$@"
do
	if [ ! -z "$dest" ]
	then
		files="$files $dest"
	fi
	dest="$file"
done

running=0
pids=""
stat=0
for hostname in $HOSTS
do
	src_files=
	for file in $files
	do
		src_files="$src_files $DOWNLOADS_DIR/$hostname/$file"
	done
		
	if [ "$popt" = "y" ]
	then
		if [ $running -ge $FF_MAX_PARALLEL ]
		then
			for pid in $pids; do
				wait $pid
				if [ "$?" -ne 0 ]; then
					stat=1
				fi
			done
			pids=""
			running=0
		fi
		echo "scp $opts $src_files $user@[$hostname]:$dest"
		scp $opts $src_files $user@\[$hostname\]:$dest &
		pid=$!
		pids="$pids $pid"
		running=$(( $running + 1))
	else
		echo "scp $opts $src_files $user@[$hostname]:$dest"
		scp $opts $src_files $user@\[$hostname\]:$dest
		if [ "$?" -ne 0 ]; then
			stat=1
		fi
	fi
done

for pid in $pids; do
	wait $pid
	if [ "$?" -ne 0 ]; then
		stat=1
	fi
done
exit $stat
