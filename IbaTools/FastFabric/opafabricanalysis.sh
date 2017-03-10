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
# Analyze fabric for errors and/or changes relative to baseline

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
	echo "Usage: opafabricanalysis [-b|-e] [-s] [-d dir] [-c file] [-t portsfile]" >&2
	echo "                            [-p ports] [-T topology_input]" >&2
	echo "              or" >&2
	echo "       opafabricanalysis --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -b - baseline mode, default is compare/check mode" >&2
	echo "   -e - evaluate health only, default is compare/check mode" >&2
	echo "   -s - save history of failures (errors/differences)" >&2
	echo "   -d dir - top level directory for saving baseline and history of failed checks" >&2
	echo "            default is /var/usr/lib/opa/analysis" >&2
	echo "   -c file - error thresholds config file" >&2
	echo "             default is $CONFIG_DIR/opa/opamon.conf" >&2
	echo "   -t portsfile - file with list of local HFI ports used to access" >&2
	echo "                  fabric(s) for analysis, default is $CONFIG_DIR/opa/ports" >&2
	echo "   -p ports - list of local HFI ports used to access fabric(s) for analysis" >&2
	echo "              default is 1st active port" >&2
	echo "              This is specified as hfi:port" >&2
	echo "                 0:0 = 1st active port in system" >&2
	echo "                 0:y = port y within system" >&2
	echo "                 x:0 = 1st active port on HFI x" >&2
	echo "                 x:y = HFI x, port y" >&2
	echo "              The first HFI in the system is 1.  The first port on an HFI is 1." >&2
	echo "   -T topology_input - name of a topology input file to use." >&2
	echo "              Any %P markers in this filename will be replaced with the" >&2
	echo "              hfi:port being operated on (such as 0:0 or 1:2)" >&2
	echo "              default is $CONFIG_DIR/opa/topology.%P.xml" >&2
	echo "              if NONE is specified, will not use any topology_input files" >&2
	echo "              See opareport for more information on topology_input files" >&2
	echo " Environment:" >&2
	echo "   PORTS - list of ports, used in absence of -t and -p" >&2
	echo "   PORTS_FILE - file containing list of ports, used in absence of -t and -p" >&2
	echo "   FF_ANALYSIS_DIR - top level directory for baselines and failed health checks" >&2
	echo "   FF_TOPOLOGY_FILE - file containing topology_input, used in absence of -T" >&2
	echo "for example:" >&2
	echo "   opafabricanalysis" >&2
	echo "   opafabricanalysis -p '1:1 1:2 2:1 2:2'" >&2
	exit 0
}

Usage()
{
	echo "Usage: opafabricanalysis [-b|-e] [-s]" >&2
	echo "              or" >&2
	echo "       opafabricanalysis --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -b - baseline mode, default is compare/check mode" >&2
	echo "   -e - evaluate health only, default is compare/check mode" >&2
	echo "   -s - save history of failures (errors/differences)" >&2
	echo "for example:" >&2
	echo "   opafabricanalysis" >&2
	exit 2
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

getbaseline=n
healthonly=n
savehistory=n
configfile=$CONFIG_DIR/opa/opamon.conf
status=ok
while getopts besd:c:p:t:T: param
do
	case $param in
	b)	getbaseline=y;;
	e)	healthonly=y;;
	s)	savehistory=y;;
	d)	export FF_ANALYSIS_DIR="$OPTARG";;
	c)	configfile="$OPTARG";;
	p)	export PORTS="$OPTARG";;
	t)	export PORTS_FILE="$OPTARG";;
	T)	export FF_TOPOLOGY_FILE="$OPTARG";;
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

check_ports_args opafabricanalysis

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
		echo "opafabricanalysis: Failure information saved to: $failures_dir/" >&2
	fi
}

for hfi_port in $PORTS
do
	hfi=$(expr $hfi_port : '\([0-9]*\):[0-9]*')
	port=$(expr $hfi_port : '[0-9]*:\([0-9]*\)')
	if [ "$hfi" = "" -o "$port" = "" ]
	then
		echo "opafabricanalysis: Error: Invalid port specification: $hfi_port" >&2
		status=bad
		continue
	fi
	resolve_topology_file opafabricanalysis "$hfi:$port"
	topt=""
	if [ "$TOPOLOGY_FILE" != "" ]
	then
		topt="-T $TOPOLOGY_FILE"
	fi


	baseline=$baseline_dir/fabric.$hfi_port
	latest=$latest_dir/fabric.$hfi_port

	if [ "$port" -eq 0 ]
	then
		port_opts="-h $hfi"	# default port to 1st active
	else
		port_opts="-h $hfi -p $port"
	fi
	
	if [[ $getbaseline == n  && $healthonly == n ]]
	then
		if [ ! -f $baseline.links -o ! -f $baseline.comps ]
		then
			echo "opafabricanalysis: Port $hfi_port Error: Previous baseline run required" >&2
			status=bad
			continue
		fi
	fi

	if [[ $healthonly == n ]]
	then
		# get a new snapshot
		mkdir -p $latest_dir
		rm -rf $latest.*

		/usr/sbin/opareport $port_opts -o snapshot -q > $latest.snapshot.xml 2> $latest.snapshot.stderr
		if [ $? != 0 ]
		then
			echo "opafabricanalysis: Port $hfi_port Error: Unable to access fabric. See $latest.snapshot.stderr" >&2
			status=bad
			save_failures $latest.snapshot.xml $latest.snapshot.stderr
			continue
		fi

		/usr/sbin/opareport -X $latest.snapshot.xml -q -o links -P > $latest.links 2> $latest.links.stderr
		if [ $? != 0 ]
		then
			echo "opafabricanalysis: Port $hfi_port Error: Unable to analyze fabric snapshot. See $latest.links.stderr" >&2
			status=bad
			save_failures $latest.snapshot.xml $latest.links $latest.links.stderr
			continue
		fi

		/usr/sbin/opareport -X $latest.snapshot.xml -q -o comps -P -d 4 > $latest.comps 2> $latest.comps.stderr
		if [ $? != 0 ]
		then
			echo "opafabricanalysis: Port $hfi_port Error: Unable to analyze fabric snapshot. See $latest.comps.stderr" >&2
			status=bad
			save_failures $latest.snapshot.xml $latest.comps $latest.comps.stderr
			continue
		fi

		if [[ $getbaseline == y ]]
		then
			mkdir -p $baseline_dir
			rm -rf $baseline.*
			cp $latest.snapshot.xml $latest.links $latest.comps $baseline_dir
		fi
	fi

	if [[ $getbaseline == n ]]
	then
		# check fabric health 
		mkdir -p $latest_dir

		/usr/sbin/opareport $port_opts $topt -c "$configfile" -q $FF_FABRIC_HEALTH > $latest.errors 2>$latest.errors.stderr
		if [ $? != 0 ]
		then
			echo "opafabricanalysis: Port $hfi_port Error: Unable to access fabric. See $latest.errors.stderr" >&2
			status=bad
			save_failures $latest.errors $latest.errors.stderr
		elif grep 'Errors found' < $latest.errors | grep -v ' 0 Errors found' > /dev/null
		then
			echo "opafabricanalysis: Port $hfi_port: Fabric possible errors found.  See $latest.errors" >&2
			status=bad
			save_failures $latest.errors $latest.errors.stderr
		fi
	fi
	
	if [[ $getbaseline == n  && $healthonly == n ]]
	then
		# compare to baseline

		# cleanup old files
		rm -f $latest.links.changes $latest.links.changes.stderr
		rm -f $latest.links.rebase $latest.links.rebase.stderr
		rm -f $latest.comps.changes $latest.comps.changes.stderr
		rm -f $latest.comps.rebase $latest.comps.rebase.stderr

		base_comps=$baseline.comps
		$FF_DIFF_CMD $baseline.links $latest.links > $latest.links.diff 2>&1
		if [ -s $latest.links.diff ]
		then
			if [ ! -e $baseline.snapshot.xml ]
			then
				echo "opafabricanalysis: Port $hfi_port: New baseline required" >&2
			else
				# see if change is only due to new FF version
				/usr/sbin/opareport -X $baseline.snapshot.xml -q -o links -P > $latest.links.rebase 2> $latest.links.rebase.stderr
				if [ $? != 0 ]
				then
					# there are other issues
					rm -f $latest.links.rebase $latest.links.rebase.stderr
				else
					cmp -s $latest.links $latest.links.rebase >/dev/null 2>&1
					if [ $? == 0 ]
					then
						# change is due to FF version
						# use new output format
						echo "opafabricanalysis: Port $hfi_port: New baseline recommended" >&2
						$FF_DIFF_CMD $latest.links.rebase $latest.links > $latest.links.diff 2>&1
						/usr/sbin/opareport -X $baseline.snapshot.xml -q -o comps -P -d 4 > $latest.comps.rebase 2> $latest.comps.rebase.stderr
						if [ $? != 0 ]
						then
							# there are other issues
							rm -f $latest.comps.rebase $latest.comps.rebase.stderr
						else
							base_comps=$latest.comps.rebase
						fi
					fi
				fi
			fi
		fi
		if [ -s $latest.links.diff -a -e $baseline.snapshot.xml ]
		then
			# use the baseline snapshot to generate an XML links report
			# we then use that as the expect topology and compare the
			# latest snapshot to the expected topology to generate a more
			# precise and understandable summary of the differences
			( /usr/sbin/opareport -X $baseline.snapshot.xml $topt -q -o links -P -x|
				/usr/sbin/opareport -T - -X $latest.snapshot.xml -q -o verifylinks -P > $latest.links.changes ) 2> $latest.links.changes.stderr
		fi

		$FF_DIFF_CMD $base_comps $latest.comps > $latest.comps.diff 2>&1
		if [ -s $latest.comps.diff -a -e $baseline.snapshot.xml ]
		then
			# use the baseline snapshot to generate an XML nodes and SMs report
			# we then use that as the expect topology and compare the
			# latest snapshot to the expected topology to generate a more
			# precise and understandable summary of the differences
			( /usr/sbin/opareport -X $baseline.snapshot.xml $topt -q -o brnodes -d 1 -P -x|
				/usr/sbin/opareport -T - -X $latest.snapshot.xml -q -o verifynodes -o verifysms -P > $latest.comps.changes ) 2> $latest.comps.changes.stderr
		fi

		if [ -s $latest.links.changes -o -s $latest.comps.changes ]
		then
			echo "opafabricanalysis: Port $hfi_port: Fabric configuration changed.  See $latest.links.changes, $latest.comps.changes and/or $latest.comps.diff" >&2
			status=bad
			files="$latest.links $latest.links.diff $latest.comps $latest.comps.diff"
			[ -e $latest.links.changes ] && files="$files $latest.links.changes $latest.links.changes.stderr"
			[ -e $latest.comps.changes ] && files="$files $latest.comps.changes $latest.comps.changes.stderr"
			save_failures $files
		elif [ -s $latest.links.diff -o -s $latest.comps.diff ]
		then
			echo "opafabricanalysis: Port $hfi_port: Fabric configuration changed.  See $latest.links.diff and/or $latest.comps.diff" >&2
			status=bad
			save_failures $latest.links $latest.links.diff $latest.comps $latest.comps.diff
		else
			rm -f $latest.links.diff $latest.links.changes $latest.links.changes.stderr $latest.comps.diff $latest.comps.changes $latest.comps.changes.stderr
		fi
	fi
done

if [ "$status" != "ok" ]
then
	if [[ $healthonly == n ]]
	then
		echo "opafabricanalysis: Possible fabric errors or changes found" >&2
	else
		echo "opafabricanalysis: Possible fabric errors found" >&2
	fi
	exit 1
else
	if [[ $getbaseline == n  ]]
	then
		echo "opafabricanalysis: Fabric(s) OK"
	else
		echo "opafabricanalysis: Baselined"
	fi
	exit 0
fi
