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
# Analyze chassis for errors and/or changes relative to baseline

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
	echo "Usage: opachassisanalysis [-b|-e] [-s] [-d dir] [-F chassisfile] [-H 'chassis']" >&2
	echo "              or" >&2
	echo "       opachassisanalysis --help" >&2
	echo "Check configuration and health for the Intel Omni-Path Fabric Chassis" >&2
	echo "   --help - produce full help text" >&2
	echo "   -b - baseline mode, default is compare/check mode" >&2
	echo "   -e - evaluate health only, default is compare/check mode" >&2
	echo "   -s - save history of failures (errors/differences)" >&2
	echo "   -d dir - top level directory for saving baseline and history of failed checks" >&2
	echo "                  default is /var/usr/lib/opa/analysis" >&2
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "   -H chassis - list of chassis to analyze" >&2
	echo " Environment:" >&2
	echo "   CHASSIS - list of chassis, used if -F and -H options not supplied" >&2
	echo "   CHASSIS_FILE - file containing list of chassis, used if -F and -H options">&2
	echo "                   not supplied" >&2
	echo "   FF_ANALYSIS_DIR - top level directory for baselines and failed health checks" >&2
	echo "   FF_CHASSIS_CMDS - list of commands to issue during analysis," >&2
	echo "                      unused if -e option supplied" >&2
	echo "   FF_CHASSIS_HEALTH - single command to issue to check overall health during">&2
	echo "                        analysis, unused if -b option supplied" >&2
	echo "for example:" >&2
	echo "   opachassisanalysis" >&2
	exit 0
}

Usage()
{
	echo "Usage: opachassisanalysis [-b|-e] [-s] [-F chassisfile]" >&2
	echo "              or" >&2
	echo "       opachassisanalysis --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -b - baseline mode, default is compare/check mode" >&2
	echo "   -e - evaluate health only, default is compare/check mode" >&2
	echo "   -s - save history of failures (errors/differences)" >&2
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "for example:" >&2
	echo "   opachassisanalysis" >&2
	exit 2
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

getbaseline=n
healthonly=n
savehistory=n
status=ok
while getopts besd:H:F: param
do
	case $param in
	b)	getbaseline=y;;
	e)	healthonly=y;;
	s)	savehistory=y;;
	d)	export FF_ANALYSIS_DIR="$OPTARG";;
	H)	export CHASSIS="$OPTARG";;
	F)	export CHASSIS_FILE="$OPTARG";;
	?)	Usage;;
	esac
done
shift $((OPTIND -1))
if [ $# -ge 1 ]
then
	Usage
fi
if [ "$getbaseline" = y -a "$healthonly" = y ]
then
	Usage
fi

check_chassis_args opachassisanalysis

#-----------------------------------------------------------------
# Set up file paths
#-----------------------------------------------------------------
baseline_dir="$FF_ANALYSIS_DIR/baseline"
latest_dir="$FF_ANALYSIS_DIR/latest"
export FF_CURTIME="${FF_CURTIME:-`date +%Y-%m-%d-%H:%M:%S`}"
failures_dir="$FF_ANALYSIS_DIR/$FF_CURTIME"

#-----------------------------------------------------------------
save_failures()
{
	if [ "$savehistory" = y ]
	then
		mkdir -p $failures_dir
		cp $* $failures_dir
		echo "opachassisanalysis: Failure information saved to: $failures_dir/" >&2
	fi
}

baseline=$baseline_dir/chassis
latest=$latest_dir/chassis

# fake loop so we can use break/continue to skip to end of script
for loop in 1
do
	if [[ $getbaseline == n  && $healthonly == n ]]
	then
		for cmd in $FF_CHASSIS_CMDS
		do
			if [ ! -f $baseline.$cmd ]
			then
				echo "opachassisanalysis: Error: Previous baseline run required" >&2
				status=bad
				break
			fi
		done
		if [ "$status" != "ok" ]
		then
			continue
		fi
	fi

	if [[ $healthonly == n ]]
	then
		# get a new snapshot
		mkdir -p $latest_dir

		rm -rf $latest.*

		for cmd in $FF_CHASSIS_CMDS
		do
			/usr/sbin/opacmdall -C $cmd > $latest.$cmd 2>&1
			if [ $? != 0 ]
			then
				echo "opachassisanalysis: Error: Unable to issue chassis command. See $latest.$cmd" >&2
				status=bad
				break
			elif grep FAILED < $latest.$cmd > /dev/null
			then
				echo "opachassisanalysis: Warning: $cmd command failed for 1 or more chassis. See $latest.$cmd" >&2
				continue
			fi
		done
		if [ "$status" != "ok" ]
		then
			continue
		fi

		if [[ $getbaseline == y ]]
		then
			mkdir -p $baseline_dir
			rm -rf $baseline.*

			for cmd in $FF_CHASSIS_CMDS
			do
				cp $latest.$cmd $baseline_dir
			done
		fi
	fi

	if [[ $getbaseline == n ]]
	then
		# check chassis health/running
		mkdir -p $latest_dir

		/usr/sbin/opacmdall -C "$FF_CHASSIS_HEALTH" > $latest.$FF_CHASSIS_HEALTH 2>&1
		if [ $? != 0 ]
		then
			echo "opachassisanalysis: Error: Unable to issue chassis command: $FF_CHASSIS_HEALTH. See $latest.$FF_CHASSIS_HEALTH" >&2
			status=bad
			save_failures $latest.$FF_CHASSIS_HEALTH
		elif grep FAILED < $latest.$FF_CHASSIS_HEALTH > /dev/null
		then
			echo "opachassisanalysis: Error: Chassis error. See $latest.$FF_CHASSIS_HEALTH" >&2
			status=bad
			save_failures $latest.$FF_CHASSIS_HEALTH
		fi
	fi

	if [[ $getbaseline == n  && $healthonly == n ]]
	then
		# compare to baseline
		for cmd in $FF_CHASSIS_CMDS
		do
			$FF_DIFF_CMD $baseline.$cmd $latest.$cmd > $latest.$cmd.diff 2>&1
			if [ -s $latest.$cmd.diff ]
			then
				echo "opachassisanalysis: Chassis configuration changed.  See $latest.$cmd.diff" >&2
				status=bad
				save_failures $latest.$cmd $latest.$cmd.diff
			else
				rm -f $latest.$cmd.diff
			fi
		done
	fi
done

if [ "$status" != "ok" ]
then
	if [[ $healthonly == n ]]
	then
		echo "opachassisanalysis: Possible Chassis errors or changes found" >&2
	else
		echo "opachassisanalysis: Possible Chassis errors found" >&2
	fi
	exit 1
else
	if [[ $getbaseline == n  ]]
	then
		echo "opachassisanalysis: Chassis OK"
	else
		echo "opachassisanalysis: Baselined"
	fi
	exit 0
fi
