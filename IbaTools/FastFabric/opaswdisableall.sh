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
# This script provides a simple way to disable all unused switch ports
# or re-enable all presently unused (or disabled) switch ports
# it uses opareport to gather the information from the SM

# optional override of defaults
if [ -f /etc/opa/opafastfabric.conf ]
then
	. /etc/opa/opafastfabric.conf
fi

. /usr/lib/opa/tools/opafastfabric.conf.def

if [ -f /usr/lib/opa/tools/ff_funcs ]
then
	. /usr/lib/opa/tools/ff_funcs
fi

trap "exit 1" SIGHUP SIGTERM SIGINT

tool="/usr/sbin/opaportconfig"
cmd=`basename $0`
if [ "$cmd" == "opaswdisableall" ]
then
  subcmd="disable"
  capsubcmd="Disable"
  extra=""
  operation="Disabling"
elif [ "$cmd" == "opaswenableall" ]
then
  subcmd="enable"
  capsubcmd="Re-enable"
  extra=" (or disabled)"
  operation="Enabling"
  cmd="opaswenableall"
else
  echo "$cmd: Invalid rename of command.  Must be opaswenableall or opaswdisableall" >&2
  exit 1
fi

Usage_full()
{
	echo "Usage: $cmd [-t portsfile] [-p ports] [-F focus]" >&2
	echo "              or" >&2
	echo "       $cmd --help" >&2
	echo "$capsubcmd all unused$extra switch ports" >&2
	echo "   --help - produce full help text" >&2
	echo "   -t portsfile - file with list of local HFI ports used to access" >&2
	echo "                  fabric(s) for operation, default is $CONFIG_DIR/opa/ports" >&2
	echo "   -p ports - list of local HFI ports used to access fabric(s) for operation" >&2
	echo "              default is 1st active port" >&2
	echo "              This is specified as hfi:port" >&2
	echo "                 0:0 = 1st active port in system" >&2
	echo "                 0:y = port y within system" >&2
	echo "                 x:0 = 1st active port on HFI x" >&2
	echo "                 x:y = HFI x, port y" >&2
	echo "              The first HFI in the system is 1.  The first port on an HFI is 1." >&2
	echo "   -F focus -  an opareport style focus argument to limit scope of operation" >&2
	echo "               See opareport --help for more information" >&2
	echo " Environment:" >&2
	echo "   PORTS - list of ports, used in absence of -t and -p" >&2
	echo "   PORTS_FILE - file containing list of ports, used in absence of -t and -p" >&2
	echo "for example:" >&2
	echo "   $cmd" >&2
	echo "   $cmd -p '1:1 1:2 2:1 2:2'" >&2
	exit 0
}

Usage()
{
	echo "Usage: $cmd" >&2
	echo "              or" >&2
	echo "       $cmd --help" >&2
	echo "$capsubcmd all unused$extra switch ports" >&2
	echo "   --help - produce full help text" >&2
	echo "for example:" >&2
	echo "   $cmd" >&2
	exit 2
}

## Main function:

focus=""

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

while getopts p:t:F: param
do
  case $param in
  p)
    export PORTS="$OPTARG";;

  t)
    export PORTS_FILE="$OPTARG";;

  F)
    focus="-F '$OPTARG'";;

  *)
    Usage;;

  esac

done

shift $((OPTIND -1))
if [ $# -ge 1 ]
then
    Usage
fi
check_ports_args $cmd

change_swports()
{
  # $1 = hfi
  # $2 = port
  if [ "$port" -eq 0 ]
  then
    port_opts="-h $hfi"	# default port to 1st active
  else
    port_opts="-h $hfi -p $port"
  fi

  echo "$operation Ports on Fabric $hfi:$port:"
  echo "NodeGUID          Port Name"

  IFS=';'
  eval /usr/sbin/opareport $port_opts -q -x -o otherports "$focus" | \
       /usr/sbin/opaxmlextract -d \; -e Switches.OtherPort.NodeGUID -e Switches.OtherPort.PortNum -e Switches.OtherPort.NodeDesc | \
       tail -n +2 | while read guid port nodedesc
  do
    lid=`eval /usr/sbin/opasaquery $port_opts -o lid -n $guid 2>/dev/null`
    # silently skip node guids we can't find in SA anymore, may be tranisient
	if [ "$?" -eq 0 -a "$lid" != "No Records Returned" ]
    then
      printf "%18s %3s %s\n" "$guid" "$port" "$nodedesc"
      eval "$tool" $port_opts -l "$lid" -m "$port" "$subcmd" > /dev/null
    fi
  done
  echo "-------------------------------------------------------------------------------"
}

status=ok

for hfi_port in $PORTS
do
	hfi=$(expr $hfi_port : '\([0-9]*\):[0-9]*')
	port=$(expr $hfi_port : '[0-9]*:\([0-9]*\)')
	if [ "$hfi" = "" -o "$port" = "" ]
	then
		echo "$cmd: Error: Invalid port specification: $hfi_port" >&2
		status=bad
		continue
	fi

	change_swports "$hfi" "$port"
done

if [ "$status" != "ok" ]
then
	exit 1
else
	exit 0
fi
