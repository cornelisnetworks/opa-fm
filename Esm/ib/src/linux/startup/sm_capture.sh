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

CONFIG_DIR=/etc/sysconfig
CONFIG_FILE=$CONFIG_DIR/opafm.xml
OPA_FM_BASE=/usr/lib/opa-fm # default
GDB=/usr/bin/gdb
SM_EXEC=/usr/lib/opa-fm/runtime/sm 

if [ -s $CONFIG_DIR/opa/opafm.info ]
then
	# get OPA_FM_BASE
	. $CONFIG_DIR/opa/opafm.info
else
	echo "Error: $CONFIG_DIR/opa/opafm.info not found: using $OPA_FM_BASE" >&2
fi


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
for dir in /var/crash/opafm $($OPA_FM_BASE/etc/opaxmlextract -H -e CoreDumpDir < $CONFIG_FILE 2>/dev/null|sort -u)
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
	fi
done

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
