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
# Analyze fabric, chassis and SMs for errors and/or changes relative to baseline

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
	echo "Usage: opaallanalysis [-b|-e] [-s] [-d dir] [-c file] [-t portsfile] [-p ports]" >&2
	echo "                    [-F chassisfile] [-H 'chassis']">&2
	echo "                    [-G esmchassisfile] [-E 'esmchassis']" >&2
	echo "                    [-T topology_input]" >&2
	echo "              or" >&2
	echo "       opaallanalysis --help" >&2
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
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "   -H chassis - list of chassis to analyze" >&2
	echo "   -G esmchassisfile - file with SM chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/esm_chassis" >&2
	echo "   -E esmchassis - list of SM chassis to analyze" >&2
	echo " Environment:" >&2
	echo "   PORTS - list of ports, used in absence of -t and -p" >&2
	echo "   PORTS_FILE - file containing list of ports, used in absence of -t and -p" >&2
	echo "   FF_TOPOLOGY_FILE - file containing topology_input, used in absence of -T" >&2
	echo "   CHASSIS - list of chassis, used if -F and -H options not supplied" >&2
	echo "   CHASSIS_FILE - file containing list of chassis, used if -F and -H options not" >&2
	echo "                  supplied" >&2
	echo "   ESM_CHASSIS - list of SM chassis, used if -G and -E not supplied" >&2
	echo "   ESM_CHASSIS_FILE - file containing list of SM chassis, used if -G and -E not" >&2
	echo "                      supplied" >&2
	echo "   FF_ANALYSIS_DIR - top level directory for baselines and failed health checks" >&2
	echo "   FF_CHASSIS_CMDS - list of commands to issue during analysis," >&2 
	echo "                     unused if -e option supplied" >&2
	echo "   FF_CHASSIS_HEALTH - single command to issue to check overall health during analysis," >&2
	echo "                       unused if -b option supplied" >&2
	echo "for example:" >&2
	echo "   opaallanalysis" >&2
	echo "   opaallanalysis -p '1:1 1:2 2:1 2:2'" >&2
	exit 0
}

Usage()
{
	echo "Usage: opaallanalysis [-b|-e] [-s] [-F chassisfile] [-G esmchassisfile]" >&2
	echo "              or" >&2
	echo "       opaallanalysis --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -b - baseline mode, default is compare/check mode" >&2
	echo "   -e - evaluate health only, default is compare/check mode" >&2
	echo "   -s - save history of failures (errors/differences)" >&2
	echo "   -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "   -G esmchassisfile - file with SM chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/esm_chassis" >&2
	echo "for example:" >&2
	echo "   opaallanalysis" >&2
	exit 2
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

getbaseline=n
healthonly=n
savehistory=n
opts=""
configfile=$CONFIG_DIR/opa/opamon.conf
status=ok
while getopts besd:c:p:t:T:H:F:E:G: param
do
	case $param in
	b)	opts="$opts -b"; getbaseline=y;;
	e)	opts="$opts -e"; healthonly=y;;
	s)	opts="$opts -s"; savehistory=y;;
	d)	export FF_ANALYSIS_DIR="$OPTARG";;
	c)	configfile="$OPTARG";;
	p)	export PORTS="$OPTARG";;
	t)	export PORTS_FILE="$OPTARG";;
	T)	export FF_TOPOLOGY_FILE="$OPTARG";;
	H)	export CHASSIS="$OPTARG";;
	F)	export CHASSIS_FILE="$OPTARG";;
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


# first verify all arguments
for i in $FF_ALL_ANALYSIS
do
	case $i in
	fabric)
		check_ports_args opaallanalysis;;
	chassis)
		check_chassis_args opaallanalysis;;
	hostsm)
		;;
	esm)
		check_esm_chassis_args opaallanalysis;;
	*)
		echo "opaallanalysis: Invalid setting in FF_ALL_ANALYSIS: $i" >&2
		exit 1;;
	esac
done
	
export FF_CURTIME="${FF_CURTIME:-`date +%Y-%m-%d-%H:%M:%S`}"

#-----------------------------------------------------------------
for i in $FF_ALL_ANALYSIS
do
	case $i in
	fabric)
		/usr/sbin/opafabricanalysis $opts -c "$configfile"
		if [ $? != 0 ]
		then
			status=bad
		fi
		;;
	chassis)
		/usr/sbin/opachassisanalysis $opts
		if [ $? != 0 ]
		then
			status=bad
		fi
		;;
	hostsm)
		/usr/sbin/opahostsmanalysis $opts
		if [ $? != 0 ]
		then
			status=bad
		fi
		;;
	esm)
		/usr/sbin/opaesmanalysis $opts
		if [ $? != 0 ]
		then
			status=bad
		fi
		;;
	esac
done

if [ "$status" != "ok" ]
then
	if [[ $healthonly == n ]]
	then
		echo "opaallanalysis: Possible errors or changes found" >&2
	else
		echo "opaallanalysis: Possible errors found" >&2
	fi
	exit 1
else
	if [[ $getbaseline == n  ]]
	then
		echo "opaallanalysis: All OK"
	else
		echo "opaallanalysis: Baselined"
	fi
	exit 0
fi
