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

# This is a bash sourced file for support functions

if [ "$CONFIG_DIR" = "" ]
then
	CONFIG_DIR=/etc
	export CONFIG_DIR
fi

resolve_file()
{
	# $1 is command name
	# $2 is file name
	# outputs full path to file, or exits with Usage if not found
	if [ -f "$2" ]
	then
		echo "$2"
	elif [ -f "$CONFIG_DIR/opa/$2" ]
	then
		echo "$CONFIG_DIR/opa/$2"
	else
		echo "$1: $2 not found" >&2
	fi
}

# filter out blank and comment lines
ff_filter_comments()
{
	egrep -v '^[[:space:]]*#'|egrep -v '^[[:space:]]*$'
}

expand_file()
{
	# $1 is command name
	# $2 is file name
	# outputs list of all non-comment lines in file
	# expands any included files
	local file

	# tabs and spaces are permitted as field splitters for include lines
	# however spaces are permitted in other lines (to handle node descriptions)
	# and tabs must be used for any comments on non-include lines
	# any line whose 1st non-whitespace character is # is a comment
	cat "$2"|ff_filter_comments|while read line
	do
		f1=$(expr "$line" : '\([^ 	]*\).*')
		if [ x"$f1" = x"include" ]
		then
			f2=$(expr "$line" : "[^ 	]*[ 	][ 	]*\([^ 	]*\).*")
			file=`resolve_file "$1" "$f2"`
			if [ "$file" != "" ]
			then
				expand_file "$1" "$file"
			fi
		else
			echo "$line"|cut -f1
		fi
	done
}

check_host_args()
{
	# $1 is command name
	# uses $HOSTS and $HOSTS_FILE
	# sets $HOSTS or calls Usage which should exit
	local l_hosts_file

	if [ "$HOSTS_FILE" = "" ]
	then
		HOSTS_FILE=$CONFIG_DIR/opa/hosts
	fi
	if [ "$HOSTS" = "" ]
	then
		l_hosts_file=$HOSTS_FILE
		HOSTS_FILE=`resolve_file "$1" "$HOSTS_FILE"`
		if [ "$HOSTS_FILE" = "" ]
		then
			echo "$1: HOSTS env variable is empty and the file $l_hosts_file does not exist" >&2 
			echo "$1: Must export HOSTS or HOSTS_FILE or use -f or -h option" >&2
			Usage
		fi
		HOSTS=`expand_file "$1" "$HOSTS_FILE"`
		if [ "$HOSTS" = "" ]
		then
			echo "$1: HOSTS env variable and the file $HOSTS_FILE are both empty" >&2 
			echo "$1: Must export HOSTS or HOSTS_FILE or use -f or -h option" >&2
			Usage
		fi
	
	fi
}

check_chassis_args()
{
	# $1 is command name
	# uses $CHASSIS and $CHASSIS_FILE
	# sets $CHASSIS or calls Usage which should exit
	local l_chassis_file

	if [ "$CHASSIS_FILE" = "" ]
	then
		CHASSIS_FILE=$CONFIG_DIR/opa/chassis
	fi
	if [ "$CHASSIS" = "" ]
	then
		l_chassis_file=$CHASSIS_FILE
		CHASSIS_FILE=`resolve_file "$1" "$CHASSIS_FILE"`
		if [ "$CHASSIS_FILE" = "" ]
		then
			echo "$1: CHASSIS env variable is empty and the file $l_chassis_file does not exist" >&2 
			echo "$1: Must export CHASSIS or CHASSIS_FILE or use -F or -H option" >&2
			Usage
		fi
		CHASSIS=`expand_file "$1" "$CHASSIS_FILE"`
		if [ "$CHASSIS" = "" ]
		then
			echo "$1: CHASSIS env variable and the file $CHASSIS_FILE are both empty" >&2 
			echo "$1: Must export CHASSIS or CHASSIS_FILE or use -F or -H option" >&2
			Usage
		fi
	fi
	
	export CFG_CHASSIS_LOGIN_METHOD=$FF_CHASSIS_LOGIN_METHOD
	export CFG_CHASSIS_ADMIN_PASSWORD=$FF_CHASSIS_ADMIN_PASSWORD
}

check_ib_transport_args()
{
	# $1 is command name
	# uses $OPASWITCHES and $OPASWITCHES_FILE
	# sets $OPASWITCHES or calls Usage which should exit
	local l_opaswitches_file

	if [ "$OPASWITCHES_FILE" = "" ]
	then
		OPASWITCHES_FILE=$CONFIG_DIR/opa/switches
	fi
	if [ "$OPASWITCHES" = "" ]
	then
		l_opaswitches_file=$OPASWITCHES_FILE
		OPASWITCHES_FILE=`resolve_file "$1" "$OPASWITCHES_FILE"`
		if [ "$OPASWITCHES_FILE" = "" ]
		then
			echo "$1: OPASWITCHES env variable is empty and the file $l_opaswitches_file does not exist" >&2 
			echo "$1: Must export OPASWITCHES or OPASWITCHES_FILE or use -L or -N option" >&2
			Usage
		fi
		OPASWITCHES=`expand_file "$1" "$OPASWITCHES_FILE"`
		if [ "$OPASWITCHES" = "" ]
		then
			echo "$1: OPASWITCHES env variable and the file $OPASWITCHES_FILE are both empty" >&2 
			echo "$1: Must export OPASWITCHES or OPASWITCHES_FILE or use -L or -N option" >&2
			Usage
		fi
	fi
	
}

check_ports_args()
{
	# $1 is command name
	# uses $PORTS and $PORTS_FILE
	# sets $PORTS or calls Usage which should exit
	local have_file_name

	if [ "$PORTS_FILE" = "" ]
	then
		PORTS_FILE=$CONFIG_DIR/opa/ports
	fi
	if [ "$PORTS" = "" ]
	then
		# allow case where PORTS_FILE is not found (ignore stderr)
		if [ "$PORTS_FILE" != "$CONFIG_DIR/opa/ports" ]
		then
			PORTS_FILE=`resolve_file "$1" "$PORTS_FILE"`
			have_file_name=1
		else
			# quietly hide a missing ports file
			PORTS_FILE=`resolve_file "$1" "$PORTS_FILE" 2>/dev/null`
			have_file_name=0
		fi
		if [ "$PORTS_FILE" = "" ]
		then
			if [ "$have_file_name" = 1 ]
			then
				Usage
			fi
		else
			PORTS=`expand_file "$1" "$PORTS_FILE"`
		fi
	fi
	if [ "$PORTS" = "" ]
	then
		PORTS="0:0"	# default to 1st active port
		#echo "$1: Must export PORTS or PORTS_FILE or use -l or -p option" >&2
		#Usage
	fi
	
}

resolve_topology_file()
{
	# $1 is command name, $2 is hfi_port fabric selector (0:0, 1:2, etc)
	# uses $FF_TOPOLOGY_FILE
	# sets $TOPOLOGY_FILE or calls Usage which should exit
	# if topology check is disabled, sets TOPOLOGY_FILE to ""

	if [ "$FF_TOPOLOGY_FILE" = "" -o "$FF_TOPOLOGY_FILE" = "NONE" ]
	then
		TOPOLOGY_FILE=""
		# topology check disabled
		return
	fi
	# expand marker
	file=$(echo "$FF_TOPOLOGY_FILE"|sed -e "s/%P/$2/g")
	# allow case where FF_TOPOLOGY_FILE is not found (ignore stderr)
	TOPOLOGY_FILE=`resolve_file "$1" "$file" 2>/dev/null`
}

check_esm_chassis_args()
{
	# $1 is command name
	# uses $ESM_CHASSIS and $ESM_CHASSIS_FILE
	# sets $ESM_CHASSIS or calls Usage which should exit
	local l_esm_chassis_file

	if [ "$ESM_CHASSIS_FILE" = "" ]
	then
		ESM_CHASSIS_FILE=$CONFIG_DIR/opa/esm_chassis
	fi
	if [ "$ESM_CHASSIS" = "" ]
	then
		l_esm_chassis_file=$ESM_CHASSIS_FILE
		ESM_CHASSIS_FILE=`resolve_file "$1" "$ESM_CHASSIS_FILE"`
		if [ "$ESM_CHASSIS_FILE" = "" ]
		then
			echo "$1: ESM_CHASSIS env variable is empty and the file $l_esm_chassis_file does not exist" >&2 
			echo "$1: Must export ESM_CHASSIS or ESM_CHASSIS_FILE or use -G or -E option" >&2
			Usage
		fi
		ESM_CHASSIS=`expand_file "$1" "$ESM_CHASSIS_FILE"`
		if [ "$ESM_CHASSIS" = "" ]
		then
			echo "$1: ESM_CHASSIS env variable and the file $ESM_CHASSIS_FILE are both empty" >&2 
			echo "$1: Must export ESM_CHASSIS or ESM_CHASSIS_FILE or use -G or -E option" >&2
			Usage
		fi
	fi
	
}


strip_chassis_slots()
{
	# removes any slot numbers and returns chassis network name
	case "$1" in
	*\[*\]*:*) # [chassis]:slot format
			#echo "$1"|awk -F \[ '{print $2}'|awk -F \] '{print $1}'
			echo "$1"|sed -e 's/.*\[//' -e 's/\].*//'
			;;
	*\[*\]) # [chassis] format
			#echo "$1"|awk -F \[ '{print $2}'|awk -F \] '{print $1}'
			echo "$1"|sed -e 's/.*\[//' -e 's/\].*//'
			;;
	*:*:*) # ipv6 without [] nor slot
			echo "$1"
			;;
	*:*) # chassis:slot format
			echo "$1"|cut -f1 -d:
			;;
	*) # chassis without [] nor slot
			echo "$1"
			;;
	esac
}

strip_switch_name()
{
	# $1 is a switches entry
	# removes any node name and returns node GUID
	echo "$1"|cut -f1 -d,
}

ping_host()
{
	#$1 is the destination to ping
	#return 1 if dest doesn't respond: unknown host or unreachable

	if type /usr/lib/opa/tools/opagetipaddrtype >/dev/null 2>&1
	then
		iptype=`/usr/lib/opa/tools/opagetipaddrtype $1 2>/dev/null`
		if [ x"$iptype" = x ]
		then
			iptype='ipv4'
		fi
	else
		iptype="ipv4"
	fi
	if [ "$iptype" == "ipv4" ]
	then
		ping -c 2 -w 4 $1 >/dev/null 2>&1
	else
		ping6 -c 2 -w 4 $1 >/dev/null 2>&1
	fi
	return $?
}


# convert the supplied $1 list into a one line per entry style output
# this is useful to take a parsed input like "$HOSTS" and convert it
# to a pipeline for use in some of the functions below or other
# basic shell commands which use stdin
ff_var_to_stdout()
{
	# translate spaces to newlines and get rid of any blank lines caused by
	# extra spaces
	echo "$1"|tr -s ' ' '\n'|sed -e '/^$/d'
}

# take the list on stdin and convert to lower case
ff_to_lc()
{
	tr A-Z a-z
}

# take the list on stdin and convert to lowercase,
# sort alphabetically filtering any dups
# assumed the list is a set of TCP/IP names which are hence case insensitive
ff_filter_dups()
{
	ff_to_lc|sort -u
}

# convert the supplied $1 list into a one line per entry style output
# and convert to lowercase, filter dups and alphabetically sort
ff_var_filter_dups_to_stdout()
{
	ff_var_to_stdout "$1"|ff_filter_dups
}
