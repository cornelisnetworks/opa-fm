#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
#
# Copyright (c) 2015-2020, Intel Corporation
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

# opafm     start and stop opafm
#
# chkconfig: - 80 20
# description: IFS InfiniBand Fabric Manager
# config: /etc/opa-fm/opafm.xml
# pidfile: /var/lock/subsys/opafm
### BEGIN INIT INFO
# Provides:       opafm
# Required-Start: rdma
# Required-Stop:  rdma
# Default-Stop:	  0 1 2
# Description:    Start the IFS InfiniBand Fabric Manager
### END INIT INFO

PATH=/bin:/sbin:/usr/bin:/usr/sbin


# just in case no functions script
echo_success() { echo "[  OK  ]"; }
echo_failure() { echo "[FAILED]"; }

my_rc_status_v()
{
	res=$?
	if [ $res -eq 0 ]
	then
		echo_success
	else
		echo_failure
	fi
	echo
	my_rc_status_all=$(($my_rc_status_all || $res))
}

if [ -f /etc/init.d/functions ]; then
    . /etc/init.d/functions
elif [ -f /etc/rc.d/init.d/functions ] ; then
    . /etc/rc.d/init.d/functions
elif [ -s /etc/rc.status ]; then
	. /etc/rc.status
	rc_reset
	my_rc_status_v()
	{
		res=$?
		[ $res -eq 0 ] || false
		rc_status -v
		my_rc_status_all=$(($my_rc_status_all || $res))
	}
fi

# pull in sysconfig settings                                                                  
if [ -f /etc/opafm.env ]; then
    . /etc/opafm.env
fi

my_rc_status_all=0
my_rc_exit()     { exit $my_rc_status_all; }
my_rc_status()   { my_rc_status_all=$(($my_rc_status_all || $?)); }

temp=$(mktemp "/tmp/ifsfmXXXXX")

trap "rm -f $temp; exit 1" 1 2 3 9
trap "rm -f $temp" EXIT

CONFIG_DIR=/etc
CONFIG_FILE=$CONFIG_DIR/opa-fm/opafm.xml	# opafm.info can override
MAX_INSTANCE=7	# largest instance number, for when config file bad

cmd=$1
force=n	# flag to force start even if running
quiet=n	# don't show per manager messages on stop, used by install
[ "$cmd" = "quietstop" ] && quiet=y	# special case for install
shift

Usage()
{
	echo "Usage: $0 [start|stop|restart|reload|sweep|status]" >&2
	echo "               [-i instance] [-f] [component|compname|insname] ..." >&2
	echo " start       - start the selected instances/managers" >&2
	echo " stop        - stop the selected instances/managers" >&2
	echo " restart     - restart (eg. stop then start) the selected instances/managers" >&2
	echo " reload      - reload configuration for the selected instances/managers" >&2
	echo " sweep       - force a fabric resweep for the selected instances/managers" >&2
	echo " status      - show status (running/not running/disabled) for the selected">&2
    echo "               instances/managers" >&2
	echo >&2
	echo " -i instance - an instance to start components for" >&2
	echo "               specified by number (0 to n) or Name" >&2
	echo " -f          - force startup even if appears already started" >&2
	echo " component   - component (sm, or fe) to start for all selected instances" >&2
	echo " compname    - a specific component specified by Name_sm or Name_fe" >&2
	echo " insname     - a specific instance by Name" >&2
	echo >&2
	echo " reload presently only supports changes to Start parameters" >&2
	echo " When -i is used, only the specified components in the instance are started">&2
	echo " -i may be specified more than once to select multiple instances" >&2
	echo " When a compname is specified, that component is started (regardless of -i)" >&2
	echo " When a insname is specified, all components of that instance are started" >&2
	echo " If components are specified, then all components in selected instances are">&2
   	echo " started">&2
	echo " If no arguments are specified, all instances are acted on" >&2
	echo " start, restart and sweep will only act on instances and managers enabled" >&2
	echo " in config file" >&2
	echo " Examples:" >&2
	echo " $0 start -i 0 sm fe" >&2
	echo "     just start sm and fe for instance 0" >&2
	echo " $0 start -i 0 sm" >&2
	echo "     just start sm for instance 0" >&2
	echo " $0 start -i 0 -i 1 sm fe" >&2
	echo "     just start sm and fe for instance 0,1" >&2
	echo " $0 start -i 0 -i 1 sm" >&2
	echo "     just start sm for instance 0,1" >&2
	echo " $0 start -i 0 -i fm1" >&2
	echo "     start all for instance 0,1" >&2
	echo " $0 start -i 0 fm1_sm" >&2
	echo "     start all for instance 0, just sm for instance 1" >&2
	echo " $0 start -i 1 fm0 sm" >&2
	echo "     start sm for instance 1, all for fm0" >&2
	exit 2
}

init_config()
{
	local i

	# set up some default instances to facilitate stop
	i=0
	while [ $i -le $MAX_INSTANCE ]
	do
		INSTANCES_NAME[$i]="fm$i"
		INSTANCES_EXIST[$i]=0
		INSTANCES_SM[$i]=0
		INSTANCES_FE[$i]=0
		i=$(($i + 1))
	done
}

invalid_config()
{
	local i

	VALID_CONFIG=n
	# set up some default instances to facilitate stop
	init_config
}

test_start()
{
	# $1=parameter name
	# $2=value, must be 0 or 1
	# test value is a valid start value (0 or 1)
	if [ "$2" -eq 0 -o "$2" -eq 1 ]
	then
		return 0
	else
		[ "$quiet" != y ] && echo "Error: $CONFIG_FILE Invalid $1 value: $2" >&2
		invalid_config
		return 1
	fi
}

# examine config file for Start tags
# build arrays of instance names and SM/FE start values
# outputs arrays indicating final enabled/disabled Start status for
# each manager in each instance:
# INSTANCES_NAME, INSTANCES_SM, INSTANCES_FE
get_config()
{
	local COMMON_SM_START COMMON_FE_START
	local INSTANCES_START
	local i IFS EXTRACT_CMD

	VALID_CONFIG=y

	# start values from <Common> section
	COMMON_SM_START=1	# Common.Sm.Start
	COMMON_FE_START=1	# Common.Fe.Start

	INSTANCES_START=	# Fm.Shared.Start

	# these arrays are the final output of this function
	# arrays of information about each instance from <Fm> section
	INSTANCES_NAME=		# Fm.Shared.Name
	INSTANCES_EXIST=	# was instance found in config file
	INSTANCES_SM=		# final Start enable for Sm in instance
	INSTANCES_FE=		# final Start enable for Fe in instance
	# manager start requires Fm.Shared.Start
	# Common.x.Start is overall setting, then Fm.X.Start can override

	init_config
	IFS_FM_BASE=/usr/lib/opa-fm

	if [ "$CONFIG_FILE" != $CONFIG_DIR/opafm.xml ]
	then
		Xopt="-X $CONFIG_FILE"
	fi

	$IFS_FM_BASE/bin/config_check -c $CONFIG_FILE
	if [ $? != 0 ]
	then
		[ "$quiet" != y ] && echo "Error: $CONFIG_FILE Invalid" >&2
		invalid_config
		return
	fi

	i=-1
	export IFS=\;
	EXTRACT_CMD="$IFS_FM_BASE/bin/opafmxmlextract -H -e Common.Sm.Start -e Common.Fe.Start -e Fm.Shared.Name -e Fm.Shared.Start -e Fm.Sm.Start -e Fm.Fe.Start"
	if [ "$quiet" != y ]
	then
		eval $EXTRACT_CMD < $CONFIG_FILE > $temp
	else
		eval $EXTRACT_CMD < $CONFIG_FILE > $temp 2>/dev/null
	fi
	if [ $? != 0 ]
	then
		[ "$quiet" != y ] && echo "Error: $CONFIG_FILE Invalid" >&2
		invalid_config
		return
	fi
	#cat $temp
	while read com_sm_start com_fe_start ins_name ins_start sm_start fe_start
	do
		if [ ! -z "$com_sm_start" ]
		then
			# Common.Sm section in XML
			if ! test_start Common.Sm.Start "$com_sm_start"; then return; fi
			COMMON_SM_START="$com_sm_start"
		elif [ ! -z "$com_fe_start" ]
		then
			# Common.Fe section in XML
			if ! test_start Common.Fe.Start "$com_fe_start"; then return; fi
			COMMON_FE_START="$com_fe_start"
		elif [ ! -z "$ins_name" ]
		then
			i=$(($i + 1))
			# Fm.Shared section in XML
			INSTANCES_NAME[$i]="$ins_name"
			INSTANCES_EXIST[$i]=1
			if [ -z "$ins_start" ]
			then
				ins_start=0
			fi
			if ! test_start "$ins_name Start" "$ins_start"; then return; fi
			INSTANCES_START[$i]=$ins_start
			INSTANCES_SM[$i]=$((${INSTANCES_START[$i]} && $COMMON_SM_START))
			INSTANCES_FE[$i]=$((${INSTANCES_START[$i]} && $COMMON_FE_START))
		elif [ ! -z "$sm_start" ]
		then
			if ! test_start "$ins_name Sm.Start" "$sm_start"; then return; fi
			INSTANCES_SM[$i]=$((${INSTANCES_START[$i]} && $sm_start))
		elif [ ! -z "$fe_start" ]
		then
			if ! test_start "$ins_name Fe.Start" "$fe_start"; then return; fi
			INSTANCES_FE[$i]=$((${INSTANCES_START[$i]} && $fe_start))
		else
			# unexpected case
			[ "$quiet" != y ] && echo "unexpected"
		fi
	done < $temp
	rm -f $temp
	i=$(($i + 1))
	while [ $i -le $MAX_INSTANCE ]
	do
		INSTANCES_NAME[$i]="fm$i"
		INSTANCES_EXIST[$i]=0
		INSTANCES_SM[$i]=0
		INSTANCES_FE[$i]=0
		i=$(($i + 1))
	done
}

# -i options processed into a temp
# matches to instance name or instance_process added immediately to startup
# sm, fe added to targets list
# after processing all args, add to startup list all targets for all -i instances
# [-i instance_num] [-i instance_name] process_name ...
# start -i 0 sm fe
#	 just start sm and fe for instance 0
# start -i 0 -i 1 sm
# 	just start sm for instance 0,1
# start -i 0 -i 1
# 	start all for instance 0,1
# start -i 0 fm1_sm
# 	start all for instance 0, just sm for instance 1
# start -i 1 fm0 sm
#	start sm for instance 1, all for fm0
# outputs arrays indicating which managers are selected:
#	SEL_SM, SEL_FE
parse_args()
{
	local SM_SEL FE_SEL INSTANCES_SEL
	local ALL_SEL ALL_INSTANCES i param found list

	ALL_INSTANCES=y	# are all instances selected
	INSTANCES_SEL=	# instances selected via -i or name

	ALL_SEL=y		# are all managers selected
	SM_SEL=n		# sm manager explicitly selected
	FE_SEL=n		# fe manager explicitly selected

	# these arrays are the final output of this function
	SEL_SM=			# array of which Sm instances are selected
	SEL_FE=			# array of which Fe instances are selected

	OPTIND=1
	while getopts fi: param
	do
		case $param in
		f)	force=y;;
		i)
			ALL_INSTANCES=n
			if [ $OPTARG -ge 0 ] 2>/dev/null
			then
				INSTANCES_SEL=(${INSTANCES_SEL[@]} $OPTARG)
			else
				i=0
				found=0
				while [ $i -lt ${#INSTANCES_NAME[@]} ]
				do
					if [ "$OPTARG" = "${INSTANCES_NAME[$i]}" ]
					then
						INSTANCES_SEL=(${INSTANCES_SEL[@]} $i)
						found=1
					fi
					i=$(($i + 1))
				done
				if [ $found != 1 ]
				then
					echo "Ignoring unknown instance/target: $OPTARG" >&2
				fi
			fi
			;;
		?) Usage;;
		esac
	done
	shift $(($OPTIND - 1))
	while [ ! -z "$1" ]
	do
		if [ "$1" = "sm" ]
		then
			SM_SEL=y
			ALL_SEL=n
		elif [ "$1" = "fe" ]
		then
			FE_SEL=y
			ALL_SEL=n
		else
			i=0
			found=0
			while [ $i -lt ${#INSTANCES_NAME[@]} ]
			do
				if [ "$1" = "${INSTANCES_NAME[$i]}" ]
				then
					INSTANCES_SEL=(${INSTANCES_SEL[@]} $i)
					SEL_SM[$i]=1
					SEL_FE[$i]=1
					found=1
				elif [ "$1" = "${INSTANCES_NAME[$i]}"_sm ]
				then
					SEL_SM[$i]=1
					found=1
				elif [ "$1" = "${INSTANCES_NAME[$i]}"_fe ]
				then
					SEL_FE[$i]=1
					found=1
				fi
				i=$(($i + 1))
			done
			if [ $found != 1 ]
			then
				echo "Ignoring unknown instance/target: $1" >&2
			else
				ALL_INSTANCES=n
			fi
		fi
		shift
	done

	# now combine -i and other args
	if [ "$VALID_CONFIG" != "y" ]
	then
		# so stop can work even if invalid config
		list="$(seq 0 $MAX_INSTANCE)"
	elif [ "$ALL_INSTANCES" = y ]
	then
		#echo "All Instances"
		list="$(seq 0 $(( ${#INSTANCES_NAME[@]} -1)) )"
	else
		list="${INSTANCES_SEL[*]}"
		#echo "Just Instances: $list"
	fi
	set -- $list
	for i in "$@"
	do
		#echo "setup $i"
		#set -x
		if [ "$VALID_CONFIG" != "y" ]
		then
			# assume it exists and is enabled
			INSTANCES_EXIST[$i]=1
			INSTANCES_SM[$i]=1
			INSTANCES_FE[$i]=1
		fi
		[ "$SM_SEL" = y -o "$ALL_SEL" = y ] && SEL_SM[$i]=1
		[ "$FE_SEL" = y -o "$ALL_SEL" = y ] && SEL_FE[$i]=1
		#set +x
	done
}

# for debug
show()
{
	local i

	i=0
	while [ $i -lt ${#INSTANCES_NAME[@]} ]
	do
		if [ ${INSTANCES_EXIST[$i]} = 1 ]
		then
			echo "Instance $i: ${INSTANCES_NAME[$i]}: "
			echo "   SM Enable: ${INSTANCES_SM[$i]}   SM Select: ${SEL_SM[$i]}"
			echo "   FE Enable: ${INSTANCES_FE[$i]}   FE Select: ${SEL_FE[$i]}"
		fi
		i=$(($i + 1))
	done
}

must_be_valid_config()
{
	if [ "$VALID_CONFIG" != "y" ]
	then
		echo "ERROR: Missing or Invalid config file"
		exit 1
	fi
}

allow_invalid_config()
{
	if [ "$VALID_CONFIG" != "y" ]
	then
		echo "Will proceed assuming no more than $(($MAX_INSTANCE+1)) fm instances"
	fi
}

running_manager()
# $1 = instance number
# $2 = manager name in lowercase
{
	pgrep -f "$2 -D -e ${2}_$1" > /dev/null 2>&1
}

# start a specific manager instance
start_manager()
# $1 = instance number
# $2 = manager name in lowercase
{
	local upcase pid res

	upcase=$(echo $2|tr a-z A-Z)
	echo -n "Starting $upcase $1: ${INSTANCES_NAME[$1]}_$2: "
	if running_manager $1 $2 && [ "$force" = "n" ]
	then
		# already running
		echo "Already running"
		res=1; false
		my_rc_status
	else
 		$IFS_FM_BASE/runtime/$2 -D -e ${2}_$1 $Xopt &
		res=$?; pid=$!
		[ $res -eq 0 ] || false
		my_rc_status_v
	fi
	if [ $res -eq 0 ]
	then
		touch /var/lock/subsys/opafm
	fi
	return $res
}

# stop a specific manager instance
stop_manager()
# $1 = instance number
# $2 = manager name in lowercase
# $3 = is manager start enabled
{
	local upcase res i was_running exists

	upcase=$(echo $2|tr a-z A-Z)
	exists=${INSTANCES_EXIST[$1]}
	was_running=0
	running_manager $1 $2 && was_running=1
	if [ $was_running = 1 ]
	then
		if [ "$quiet" != y ]
		then
			echo -n "Stopping $upcase $1: ${INSTANCES_NAME[$1]}_$2: "
		fi
		#killproc "$2 -D -e ${2}_$1"
		res=0
		pkill -f "$2 -D -e ${2}_$1"
		# give it up to a minute to exit
		i=0
		while [ $i -lt 60 ]
		do
			if ! running_manager $1 $2
			then
				break
			fi
			sleep 1
			i=$(($i + 1))
		done
		if [ $i -ge 60 ]
		then
			# still running, kill it hard
			pkill -9 -f "$2 -D -e ${2}_$1"
			sleep 1
		fi
		res=0; running_manager $1 $2 && res=1
		if [ "$quiet" != y ]
		then
			[ $res -eq 0 ] || false
			my_rc_status_v
		else
			[ $res -eq 0 ] || false
			my_rc_status
		fi
	else
		if [ "$exists" = 1 -a "$3" = 1  -a "$quiet" != y ]
		then
			echo -n "Stopping $upcase $1: ${INSTANCES_NAME[$1]}_$2: "
			echo "Not Running"
			res=0
			#my_rc_status_v
			my_rc_status
		else
			res=0
			my_rc_status
		fi
	fi
	return $res
}

# force resweep of a specific manager instance
# should only be called for managers which are enabled and support sweep
sweep_manager()
# $1 = instance number
# $2 = manager name in lowercase
{
	local upcase pid res

	upcase=$(echo $2|tr a-z A-Z)
	echo -n "Force Sweep $upcase $1: ${INSTANCES_NAME[$1]}_$2: "
	if running_manager $1 $2
	then
 		$IFS_FM_BASE/bin/fm_cmd -i $1 ${2}ForceSweep >/dev/null
		res=$?
		[ $res -eq 0 ] || false
		my_rc_status_v
	else
		echo "Not Running"
		res=1; false
		my_rc_status
	fi
	return $res
}

# used to report error when exclusively FE resweep requested
sweep_na()
# $1 = instance number
# $2 = manager name in lowercase
{
	local upcase

	upcase=$(echo $2|tr a-z A-Z)
	echo -n "Force Sweep $upcase $1: ${INSTANCES_NAME[$1]}_$2: "
	echo "NA for $upcase"; false
	#my_rc_status_v
	my_rc_status
	return 1
}

# check a specific manager instance
status_manager()
# $1 = instance number
# $2 = manager name in lowercase
# $3 = is manager start enabled
{
	local upcase res

	upcase=$(echo $2|tr a-z A-Z)
	if [ ${INSTANCES_EXIST[$1]} = 1 ]
	then
		echo -n "Checking $upcase $1: ${INSTANCES_NAME[$1]}_$2: "
		if running_manager $1 $2
		then
			echo "Running"
			res=0
		elif [ "$3" = 1 ]
		then
			echo "Not Running"
			res=1; false
		else
			echo "Disabled"
			res=0
		fi
	fi
	my_rc_status
	return $res
}

# start all processes which are enabled and selected via command line
start()
{
	local ret i

	echo "Starting IFS Fabric Manager"

	modprobe ib_uverbs

	ret=0
	i=0
	while [ $i -lt ${#INSTANCES_NAME[@]} ]
	do
		if [ 1 = "${INSTANCES_SM[$i]}" -a 1 = "${SEL_SM[$i]}" ]
		then
			start_manager $i sm
			ret=$(($ret || $?))
			sleep 1
		fi
		if [ 1 = "${INSTANCES_FE[$i]}" -a 1 = "${SEL_FE[$i]}" ]
		then
			start_manager $i fe
			ret=$(($ret || $?))
			sleep 1
		fi
		i=$(($i + 1))
	done
	return $ret
}

# stop all processes which are selected via command line
stop()
{
	local ret i

	if [ "$quiet" = y ]
	then
		echo "Stopping all instances of IFS Fabric Manager"
	else
		echo "Stopping IFS Fabric Manager"
	fi
	ret=0
	i=0
	while [ $i -lt ${#INSTANCES_NAME[@]} ]
	do
		if [ 1 = "${SEL_FE[$i]}" ]
		then
			stop_manager $i fe "${INSTANCES_FE[$i]}"
			ret=$(($ret || $?))
		fi
		if [ 1 = "${SEL_SM[$i]}" ]
		then
			stop_manager $i sm "${INSTANCES_SM[$i]}"
			ret=$(($ret || $?))
		fi
		i=$(($i + 1))
	done
	if ! pgrep -f "sm -D -e sm_" > /dev/null 2>&1 \
		&& ! pgrep -f "fe -D -e fe_" > /dev/null 2>&1
	then
		rm -f /var/lock/subsys/opafm
	fi
	return $ret
}

# reload config for all processes which are enabled and selected via command line
reload()
{
	echo "Sending signal to dynamically reconfigure IFS Fabric Manager"
	pkill -SIGHUP sm
	res=$?
	[ $res -eq 0 ] || false
	my_rc_status_v
}

# force sweep for all managers which are enabled and selected via command line
sweep()
{
	local ret i swept

	echo "Forcing IFS Fabric Manager Sweep"

	ret=0
	i=0
	while [ $i -lt ${#INSTANCES_NAME[@]} ]
	do
		swept=0
		if [ 1 = "${INSTANCES_SM[$i]}" -a 1 = "${SEL_SM[$i]}" ]
		then
			sweep_manager $i sm
			ret=$(($ret || $?))
			swept=1
		fi
		if [ 1 = "${INSTANCES_FE[$i]}" -a 1 = "${SEL_FE[$i]}" ]
		then
			#sweep_manager $i fe # resweep NA for FE
			#ret=$(($ret || $?))
			[ $swept = 0 ] && sweep_na $i fe
		fi
		i=$(($i + 1))
	done
	return $ret
}

# check all processes which are enabled and selected via command line
status()
{
	local ret i

	echo "Checking IFS Fabric Manager"
	ret=0
	i=0
	while [ $i -lt ${#INSTANCES_NAME[@]} ]
	do
		if [ 1 = "${SEL_SM[$i]}" ]
		then
			status_manager $i sm "${INSTANCES_SM[$i]}"
			ret=$(($ret || $?))
		fi
		if [ 1 = "${SEL_FE[$i]}" ]
		then
			status_manager $i fe "${INSTANCES_FE[$i]}"
			ret=$(($ret || $?))
		fi
		i=$(($i + 1))
	done
	return $ret
}

get_config
#echo "INSTANCES_NAME=${INSTANCES_NAME[@]}"
#echo "INSTANCES_SM=${INSTANCES_SM[@]}"
#echo "INSTANCES_FE=${INSTANCES_FE[@]}"

parse_args "$@"

#show

case "$cmd" in
	start)
		must_be_valid_config
		start;;
	quietstop|stop)
		allow_invalid_config
		stop;;
	restart)
		must_be_valid_config
		stop; start;;
	reload)
		must_be_valid_config
		reload;;
	sweep)
		must_be_valid_config
		sweep;;
	status)
		allow_invalid_config
		status;;
	*)
		Usage;;
esac

my_rc_exit
