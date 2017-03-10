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
# Analyze host SM for errors and/or changes relative to baseline

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
	echo "Usage: opahostsmanalysis [-b|-e] [-s] [-d dir]" >&2
	echo "              or" >&2
	echo "       opahostsmanalysis --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -b - baseline mode, default is compare/check mode" >&2
	echo "   -e - evaluate health only, default is compare/check mode" >&2
	echo "   -s - save history of failures (errors/differences)" >&2
	echo "   -d dir - top level directory for saving baseline and history of failed checks" >&2
	echo "            default is /var/usr/lib/opa/analysis" >&2
	echo " Environment:" >&2
	echo "   FF_ANALYSIS_DIR - top level directory for baselines and failed health checks" >&2
	echo "for example:" >&2
	echo "   opahostsmanalysis" >&2
	exit 0
}

Usage()
{
	echo "Usage: opahostsmanalysis [-b|-e] [-s]" >&2
	echo "              or" >&2
	echo "       opahostsmanalysis --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -b - baseline mode, default is compare/check mode" >&2
	echo "   -e - evaluate health only, default is compare/check mode" >&2
	echo "   -s - save history of failures (errors/differences)" >&2
	echo "for example:" >&2
	echo "   opahostsmanalysis" >&2
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
while getopts besd: param
do
	case $param in
	b)	getbaseline=y;;
	e)	healthonly=y;;
	s)	savehistory=y;;
	d)	export FF_ANALYSIS_DIR="$OPTARG";;
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

#-----------------------------------------------------------------
# Set up file paths
#-----------------------------------------------------------------
# newer versions of the SM (which support XML config)
# 
SM_CONFIG_FILE=/etc/opa-fm/opafm.xml

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
		echo "opahostsmanalysis: Failure information saved to: $failures_dir/" >&2
	fi
}

baseline=$baseline_dir/hostsm
latest=$latest_dir/hostsm

# fake loop so we can use continue to skip to end of script
for loop in 1
do
	if [[ $getbaseline == n  && $healthonly == n ]]
	then
		if [ ! -f $baseline.smver -o ! -f $baseline.smconfig ]
		then
			echo "opahostsmanalysis: Error: Previous baseline run required" >&2
			status=bad
			continue
		fi
	fi

	if [[ $healthonly == n ]]
	then
		# get a new snapshot
		mkdir -p $latest_dir
		rm -rf $latest.*

		rpm -q opa-fm > $latest.smver 2>&1
		if [ $? != 0 ]
		then
			echo "opahostsmanalysis: Error: Host SM not installed" >&2
			status=bad
			continue
		fi

		cp $SM_CONFIG_FILE $latest.smconfig
		if [ $? != 0 ]
		then
			echo "opahostsmanalysis: Error: Unable to copy Host SM Config" >&2
			status=bad
			continue
		fi

		if [[ $getbaseline == y ]]
		then
			mkdir -p $baseline_dir
			rm -rf $baseline.*
			cp $latest.smver $latest.smconfig $baseline_dir
		fi
	fi
	
	if [[ $getbaseline == n ]]
	then
		# check SM health/running
		mkdir -p $latest_dir

		/usr/lib/opa-fm/bin/fm_cmd smShowCounters > $latest.smstatus 2>&1
		r=$?

		if [ $r != 0 ]
		then
			echo "opahostsmanalysis: Error: Host SM not running. See $latest.smstatus" >&2
			status=bad
			save_failures $latest.smstatus
		fi
	fi
	
	if [[ $getbaseline == n  && $healthonly == n ]]
	then
		# compare to baseline
		$FF_DIFF_CMD $baseline.smver $latest.smver > $latest.smver.diff 2>&1
		$FF_DIFF_CMD $baseline.smconfig $latest.smconfig > $latest.smconfig.diff 2>&1
		if [ -s $latest.smver.diff -o -s $latest.smconfig.diff ]
		then
			echo "opahostsmanalysis: SM configuration changed.  See $latest.smconfig.diff and $latest.smver.diff" >&2
			status=bad
			save_failures $latest.smver $latest.smver.diff $latest.smconfig $latest.smconfig.diff
		else
			rm -f $latest.smver.diff $latest.smconfig.diff
		fi
	fi
done
	
if [ "$status" != "ok" ]
then
	if [[ $healthonly == n ]]
	then
		echo "opahostsmanalysis: Possible Host SM errors or changes found" >&2
	else
		echo "opahostsmanalysis: Possible Host SM errors found" >&2
	fi
	exit 1
else
	if [[ $getbaseline == n  ]]
	then
		echo "opahostsmanalysis: Host SM(s) OK"
	else
		echo "opahostsmanalysis: Baselined"
	fi
	exit 0
fi
