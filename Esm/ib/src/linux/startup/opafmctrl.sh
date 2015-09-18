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

PATH=/bin:/sbin:/usr/bin:/usr/sbin

CONFIG_DIR=/etc/sysconfig
CONFIG_FILE=$CONFIG_DIR/opafm.xml	# opafm.info can override
MAX_INSTANCE=7	# largest instance number, for when config file bad

cmd=$1
force=n	# flag to force start even if running
quiet=n	# don't show per manager messages on stop, used by install
[ "$cmd" = "quietstop" ] && quiet=y	# special case for install
shift

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
	pgrep -f "$2 -e ${2}_$1" > /dev/null 2>&1
}

my_rc_status_all=0
my_rc_exit()     { exit $my_rc_status_all; }
my_rc_status()   { my_rc_status_all=$(($my_rc_status_all || $?)); }

temp=$(mktemp "/tmp/ifsfmXXXXX")
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
	
	if [ -s $CONFIG_DIR/opa/opafm.info ]
	then
		# get IFS_FM_BASE
		. $CONFIG_DIR/opa/opafm.info
	else
		IFS_FM_BASE=/opt/opafm
		if [ "$quiet" != y ]
		then
			echo "Error: $CONFIG_DIR/opa/opafm.info not found" >&2
		fi
		invalid_config
		return
	fi
	if [ "$CONFIG_FILE" != $CONFIG_DIR/opafm.xml ]
	then
		Xopt="-X $CONFIG_FILE"
	fi

	$IFS_FM_BASE/etc/config_check -c $CONFIG_FILE
	if [ $? != 0 ]
	then
		[ "$quiet" != y ] && echo "Error: $CONFIG_FILE Invalid" >&2
		invalid_config
		return
	fi

	i=-1
	export IFS=\;
	EXTRACT_CMD="$IFS_FM_BASE/etc/opaxmlextract -H -e Common.Sm.Start -e Common.Fe.Start -e Fm.Shared.Name -e Fm.Shared.Start -e Fm.Sm.Start -e Fm.Fe.Start"
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
 		#$IFS_FM_BASE/runtime/$2 -D -e ${2}_$1 $Xopt &
 		#systemctl start opa$2@$1
		if [ "$2" = "sm" ]; then
		    # start up the SM
		    echo "start 1 $1" > /var/run/opafmd
		else
		    # start up the FE
		    echo "start 2 $1" > /var/run/opafmd
		fi
		res=$?; #pid=$!
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
		#pkill -f "$2 -D -e ${2}_$1"
		#systemctl stop opa$2@$1
		if [ "$2" = "sm" ]; then
		    # start up the SM
		    echo "stop 1 $1" > /var/run/opafmd
		else
		    # start up the FE
		    echo "stop 2 $1" > /var/run/opafmd
		fi
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
			pkill -9 -f "$2 -e ${2}_$1"
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

# start all processes which are enabled and selected via command line
start()
{
	local ret i

	echo "Starting IFS Fabric Manager"

	modprobe ib_uverbs

	ret=0
	i=0
	if ! pgrep -f "opafmd -D" > /dev/null 2>&1
	then
		echo "OPA Fabric Manager service not running"
		echo "Please start opafm service and try again."
		exit 1
	fi
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
	if ! pgrep -f "sm -e sm_" > /dev/null 2>&1 \
		&& ! pgrep -f "fe -e fe_" > /dev/null 2>&1
	then
		rm -f /var/lock/subsys/opafm
	fi
	return $ret
}

get_config
#echo "INSTANCES_NAME=${INSTANCES_NAME[@]}"
#echo "INSTANCES_SM=${INSTANCES_SM[@]}"
#echo "INSTANCES_FE=${INSTANCES_FE[@]}"

parse_args "$@"

case "$cmd" in
	start)
		must_be_valid_config
		start;;
	quietstop|stop)
		allow_invalid_config
		stop;;
esac
