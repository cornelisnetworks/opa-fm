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
OPA_FM_BASE=/usr/lib/opa-fm # default
GDB=/usr/bin/gdb
SM_EXEC=/usr/lib/opa-fm/runtime/sm
IFS_FM_BASE=/usr/lib/opa-fm

PROGNAME="$0"
dirName="smdump-"`date '+%d%b%y%H%M%S'`
dumpLocation="/tmp/$dirName"

usage() {
	echo "Usage:"
	echo "$PROGNAME [fm_instance ...]"
	exit 2
}

die () {
	echo "$@" > /dev/stderr

	echo "removing $dumpLocation"
	rm -rf $dumpLocation
	exit 1
}

INSTANCES=
while [ $# -ne 0 ]
do
	if echo "$1" | egrep '^[0-9]+$' > /dev/null
	then
		INSTANCES="$INSTANCES $1"
	else
		usage
	fi
	shift
done
if [ "x$INSTANCES" = "x" ]
then
	INSTANCES="0 1 2 3 4 5 6 7"
fi

running_manager()
# $1 = instance number
# $2 = manager name in lowercase
{
	pgrep -f "$2 (-D )?-e ${2}_$1" > /dev/null 2>&1
}
dump_sm_instance()
{
	# $1 = instance
	# $2 =dir
	echo "Getting SM $1 counters..."
	$OPA_FM_BASE/bin/fm_cmd -i$1 smShowCounters > $2/smShowCounters

	echo "Getting PM $1 counters..."
	$OPA_FM_BASE/bin/fm_cmd -i$1 pmShowCounters > $2/pmShowCounters

	#echo "Dumping SM $1 state..."
	#$OPA_FM_BASE/bin/fm_cmd -i$1 smStateDump $2

}

dump_sm_core()
{
	# Force an SM core dump (without stopping SM)
	# $1 = instance
	# $2 = manager name in lowercase
	# $3 = dir
	if [ -x $GDB ]
	then
		echo "Getting SM $1 run-time core file"
		SMPID=`pgrep -f "$2 (-D )?-e ${2}_$1"` >/dev/null 2>&1
		#Create specialized GDB core dumping file.
		echo -ne "generate-core-file $3/SM_$1_running_core\ndetach\nquit" > $3/sm_gdb_inst.txt
		$GDB $SM_EXEC $SMPID -x $3/sm_gdb_inst.txt -batch >/dev/null 2>&1
	else
		echo "GDB Not installed, unable to get SM_$1 runtime core"
	fi
}

echo "Creating dump directory..."
mkdir -p $dumpLocation || \
  die "Could not create directory $dumpLocation"
if [ $(command -v systemctl) ]; then
	echo "Getting systemd information..."
	systemctl show opafm > $dumpLocation/systemd_show
fi

echo "Getting FM rpm version..."
rpm -qa|egrep 'opa-fm' > $dumpLocation/version

echo "Copying FM configuration..."
cp $CONFIG_FILE $dumpLocation

echo "Copying FM core dumps..."
did_default=n
did_core_copy=n
for dir in /var/crash/opafm $($OPA_FM_BASE/bin/opaxmlextract -H -e CoreDumpDir < $CONFIG_FILE 2>/dev/null|sort -u)
do
	if [ -d $dir ]
	then
		if [ "$dir" = "/var/crash/opafm" ]
		then
			[ $did_default = y ] && continue
			did_default=y
		fi
		mkdir -p $dumpLocation/crash
		cp -r $dir $dumpLocation/crash
		did_core_copy=y
	fi
done
# It is possible to have the core files in a different directory. /proc/sys/kernel/core_pattern gives the core pattern and location.
# If absolute path is specified as in "/tmp/cores/core.%e.%p.%h.%t" then the core files in the specified directory are copied.
# The core file can only be in the sub directory that map to options %p%u%g%s%t%h%e.
core_path=`cat /proc/sys/kernel/core_pattern`
if [ -n "$core_path" -a "${core_path:0:1}" = "/" ]
then
	if [ $did_core_copy == n ]
	then
		mkdir -p $dumpLocation/crash
	fi
	# If the core_pattern does not contain the %options, then copy the file as is.
	if [[ "$core_path" != *"%"* ]]
	then
		if [ -f $core_path* ]
		then
			cp $core_path* $dumpLocation/crash
			did_core_copy=y
		fi
	# If the sub directory does not fall into the standard list throw an error
	elif [[ ${core_path%/*} =~ [%][^pugsthe] ]]
	then
		echo "The core pattern "${core_path%/*}" is not in the standard format"
	# Match the sub directory as per the provided pattern and copy the core file
	else
		# Replace all possible pattern option with ^/
		REGEX=$(echo ${core_path%/*} | sed -e "s#[%][pugsthe]#[^/]*#g")

		# Add escape char for '/'
		REGEX_2=$(echo ${REGEX} | sed -n "s#[/]\([^]]\)#\\\/\1#gp")

		# Get the file name without the options
		core_file_pattern=${core_path##*/}
		core_file=${core_file_pattern%%%*}

		# Add escape char for '/'
		file_pattern="$REGEX_2\/$core_file"

		# Find if the file exists in the path
		for file in `find ${REGEX%%[*} -type f | sed -n "/${file_pattern}*/p" `
		do
			if [ -f $file ]
			then
				cp --parents $file $dumpLocation/crash
				did_core_copy=y
			fi
		done
	fi
fi

if [ $did_core_copy == n ]
then
	echo "No core dumps present in default location, /var/crash/opafm"
	echo "No core dumps present in opafm.xml specified location, $($OPA_FM_BASE/etc/opaxmlextract -H -e CoreDumpDir < $CONFIG_FILE 2>/dev/null|sort -u) "
	if [ -d ${core_path%/*} ]
	then
		echo "No core dumps present in $core_path"
	fi
fi


for i in $INSTANCES
do
	dir=$dumpLocation/$i
	if [ -d $dir ]
	then
		echo "Skipping SM $i: already captured"
	elif running_manager $i sm
	then
		mkdir $dir || die "Could not create directory $dir"
		dump_sm_instance $i $dir
		dump_sm_core $i sm $dir
	else
		echo "Skipping SM $i: Not Running"
	fi
done

echo "Packaging capture file..."
tar --format=gnu -czf ./${dirName}.tgz --directory=/tmp $dirName --warning=no-file-changed || \
  die "Could not create state dump tarball"

echo "Saved FM capture as ${dirName}.tgz"

rm -rf $dumpLocation
