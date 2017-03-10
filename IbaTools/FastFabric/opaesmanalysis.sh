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
# Analyze Embedded SMs for errors and/or changes relative to baseline

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
	echo "Usage: opaesmanalysis [-b|-e] [-s] [-d dir] [-G esmchassisfile]" >&2
	echo "                         [-E 'esmchassis']" >&2
	echo "              or" >&2
	echo "       opaesmanalysis --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -b - baseline mode, default is compare/check mode" >&2
	echo "   -e - evaluate health only, default is compare/check mode" >&2
	echo "   -s - save history of failures (errors/differences)" >&2
	echo "   -d dir - top level directory for saving baseline and history of failed checks" >&2
	echo "           default is /var/usr/lib/opa/analysis" >&2
	echo "   -G esmchassisfile - file with SM chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/esm_chassis" >&2
	echo "   -E esmchassis - list of SM chassis to analyze" >&2
	echo " Environment:" >&2
	echo "   ESM_CHASSIS - list of SM chassis, used if -G and -E not supplied" >&2
	echo "   ESM_CHASSIS_FILE - file containing list of SM chassis, used if -G and -E not" >&2
	echo "           supplied" >&2
	echo "   FF_ANALYSIS_DIR - top level directory for baselines and failed health checks" >&2
	echo "for example:" >&2
	echo "   opaesmanalysis" >&2
	exit 0
}

Usage()
{
	echo "Usage: opaesmanalysis [-b|-e] [-s] [-G esmchassisfile]" >&2
	echo "              or" >&2
	echo "       opaesmanalysis --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -b - baseline mode, default is compare/check mode" >&2
	echo "   -e - evaluate health only, default is compare/check mode" >&2
	echo "   -s - save history of failures (errors/differences)" >&2
	echo "   -G esmchassisfile - file with SM chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/esm_chassis" >&2
	echo "for example:" >&2
	echo "   opaesmanalysis" >&2
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
while getopts besd:E:G: param
do
	case $param in
	b)	getbaseline=y;;
	e)	healthonly=y;;
	s)	savehistory=y;;
	d)	export FF_ANALYSIS_DIR="$OPTARG";;
	E)	export ESM_CHASSIS="$OPTARG";;
	G)	export ESM_CHASSIS_FILE="$OPTARG";;
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

check_esm_chassis_args opaesmanalysis

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
		echo "opaesmanalysis: Failure information saved to: $failures_dir/" >&2
	fi
}

baseline=$baseline_dir/esm
latest=$latest_dir/esm

# fake loop so we can use break/continue to skip to end of script
for loop in 1
do
	if [[ $getbaseline == n  && $healthonly == n ]]
	then
		for cmd in $FF_ESM_CMDS
		do
			if [ ! -f $baseline.$cmd ]
			then
				echo "opaesmanalysis: Error: Previous baseline run required" >&2
				status=bad
				break
			fi
		done
		for chassis in $ESM_CHASSIS
		do
			chassis=`strip_chassis_slots "$chassis"`
			if [ ! -f $baseline.$chassis.smConfig ]
			then
				echo "opaesmanalysis: Error: Previous baseline run required" >&2
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

		# cleanup old snapshot
		rm -rf $latest.* test.log test.res test_tmp* save_tmp temp

		for cmd in $FF_ESM_CMDS
		do
			if [ "$cmd" == "smConfig" ]
			then
				/usr/sbin/opacmdall -C -H "$ESM_CHASSIS" "smConfig query" > $latest.$cmd 2>&1
			else
				/usr/sbin/opacmdall -C -H "$ESM_CHASSIS" $cmd > $latest.$cmd 2>&1
			fi

			if [ $? != 0 ]
			then
				echo "opaesmanalysis: Error: Unable to issue chassis command. See $latest.$cmd" >&2
				status=bad
				break
			elif grep FAILED < $latest.$cmd > /dev/null
			then
				echo "opaesmanalysis: Warning: $cmd command failed for 1 or more chassis. See $latest.$cmd" >&2
				continue
			fi
		done
		for chassis in $ESM_CHASSIS
		do
			chassis=`strip_chassis_slots "$chassis"`
			/usr/sbin/opacmdall -C -H "$chassis" "showCapability -key smConfig" > $latest.$chassis.smConfig 2>&1
			if [ $? != 0 ]
			then
				echo "opaesmanalysis: Error: Unable to issue chassis command. See $latest.$chassis.smConfig" >&2
				status=bad
			elif ! egrep 'FAILED|unavailable' < $latest.$chassis.smConfig >/dev/null
			then
				# supports XML config for esm
				(
				cd $latest_dir
				mkdir temp
				rm -rf test.log test.res test_tmp* save_tmp
				export FF_RESULT_DIR=.
				/usr/sbin/opachassisadmin -H "$chassis" -d $latest_dir/temp fmgetconfig > $latest.$chassis.fmgetconfig 2>&1
				if [ $? != 0 ] || grep FAILED < test.res >/dev/null
				then
					cat test.log >> $latest.$chassis.fmgetconfig 2>/dev/null
					echo "opaesmanalysis: Error: Unable to get FM config file. See $latest.$chassis.fmgetconfig"
					status=bad
				else
					mv $latest_dir/temp/$chassis/opafm.xml $latest.$chassis.opafm.xml
				fi
				rm -rf test.log test.res test_tmp* save_tmp temp
				)
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
			for cmd in $FF_ESM_CMDS
			do
				cp $latest.$cmd $baseline_dir
			done
			for chassis in $ESM_CHASSIS
			do
				chassis=`strip_chassis_slots "$chassis"`
				cp $latest.$chassis.smConfig $baseline_dir
				[ -e $latest.$chassis.fmgetconfig ] && cp $latest.$chassis.fmgetconfig $baseline_dir
				[ -e $latest.$chassis.opafm.xml ] && cp $latest.$chassis.opafm.xml $baseline_dir
			done
		fi
	fi

	if [[ $getbaseline == n ]]
	then
		# check SM health/running
		mkdir -p $latest_dir

		/usr/sbin/opacmdall -C -H "$ESM_CHASSIS" 'smControl status' > $latest.smstatus 2>&1
		if [ $? != 0 ]
		then
			echo "opaesmanalysis: Error: Unable to issue chassis command. See $latest.smstatus" >&2
			status=bad
			save_failures $latest.smstatus
		elif grep stopped < $latest.smstatus > /dev/null
		then
			echo "opaesmanalysis: Error: SM(s) not running. See $latest.smstatus" >&2
			status=bad
			save_failures $latest.smstatus
		fi
	fi

	if [[ $getbaseline == n  && $healthonly == n ]]
	then
		# compare to baseline
		for cmd in $FF_ESM_CMDS
		do
			$FF_DIFF_CMD $baseline.$cmd $latest.$cmd > $latest.$cmd.diff 2>&1
			if [ -s $latest.$cmd.diff ]
			then
				echo "opaesmanalysis: SM configuration changed.  See $latest.$cmd.diff" >&2
				status=bad
				save_failures $latest.$cmd $latest.$cmd.diff
			else
				rm -f $latest.$cmd.diff
			fi
		done
		for chassis in $ESM_CHASSIS
		do
			chassis=`strip_chassis_slots "$chassis"`
			if [ -f $latest.$chassis.opafm.xml -o -f $baseline.$chassis.opafm.xml ]
			then
				$FF_DIFF_CMD $baseline.$chassis.opafm.xml $latest.$chassis.opafm.xml > $latest.$chassis.opafm.xml.diff 2>&1
				if [ -s $latest.$chassis.opafm.xml.diff ]
				then
					echo "opaesmanalysis: SM configuration changed.  See $latest.$chassis.opafm.xml.diff" >&2
					status=bad
					save_failures $latest.$chassis.opafm.xml $latest.$chassis.opafm.xml.diff
				else
					rm -f $latest.$chassis.opafm.xml.diff
				fi
			fi
		done
	fi
done

if [ "$status" != "ok" ]
then
	if [[ $healthonly == n ]]
	then
		echo "opaesmanalysis: Possible SM errors or changes found" >&2
	else
		echo "opaesmanalysis: Possible SM errors found" >&2
	fi
	exit 1
else
	if [[ $getbaseline == n  ]]
	then
		echo "opaesmanalysis: SM(s) OK"
	else
		echo "opaesmanalysis: Baselined"
	fi
	exit 0
fi
