#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
#
# Copyright (c) 2015-2018, Intel Corporation
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

# Usage: opacablehealthcron
# At regular intervals executes cablehealth and saves the data in a file

Usage_full()
{
	echo "Usage: opacablehealthcron" >&2
	echo "       opacablehealthcron --help" >&2
	echo "   --help - produce full help text" >&2
	echo "This script will at regular intervals execute opareport -o cablehealth" >&2
	echo "The output of the program is stored in an already specified location" >&2
	echo "It is defined in FF_CABLE_HEALTH_REPORT_DIR of opafastfabric.conf" >&2
}

Usage()
{
	Usage_full
	exit 2
}

if [ x"$1" = "x--help" ]
then
	Usage_full
	exit 0
fi


if [ -f /usr/lib/opa/tools/ff_funcs ]
then
	. /usr/lib/opa/tools/ff_funcs
	ff_available=y
else
	ff_available=n
fi

if [ $ff_available = "y" ]
then
	if [ -f $CONFIG_DIR/opa/opafastfabric.conf  ]
	then
		. $CONFIG_DIR/opa/opafastfabric.conf
	fi
	. /usr/lib/opa/tools/opafastfabric.conf.def
fi

# Check if is the master FM
# For nodes with FM running as master, BaseLID and SMLId will be same
# In Multi-rail the SM could be linked to any one of the HFI, PORT combinations
# In Multi-plane master SM for each fabric could be linked to any of the HFI, PORT combinations
if [ "$ff_available" = y ]
then
    check_ports_args opacablehealthcron
else
	PORTS='0:0'
fi

for hfi_port in $PORTS
do
	hfi=$(expr $hfi_port : '\([0-9]*\):[0-9]*')
	port=$(expr $hfi_port : '[0-9]*:\([0-9]*\)')
	if [ "$hfi" = "" -o "$port" = "" ]
	then
		echo "opacablehealthcron: Error: Invalid port specification: $hfi_port" >&2
		continue
	fi

	## fixup name so winzip won't complain
	hfi_port_dir=${hfi}_${port}

	# For nodes with FM running as master, LID and MasterSMLID will be same
	baseLid=`/usr/sbin/opasmaquery -h $hfi -p $port -o portinfo -g 2>/dev/null| sed -ne '/\bLID\b/ s/.*: *//p'`
	smLid=`/usr/sbin/opasmaquery -h $hfi -p $port -o portinfo -g 2>/dev/null| sed -ne '/\bMasterSMLID\b/ s/.*: *//p'`
	#Store HFI of device connected to SM
	if [ "$baseLid" != "$smLid" ]
	then
		continue;
	fi

	echo "Periodic generation of Cable Health Report"
	# create cable health report directory and generate report
	if [ $ff_available = "y" ] && [ -n "${FF_CABLE_HEALTH_REPORT_DIR}" ]
	then
		echo "Gathering Cable Health Report ..."
		# Directory name with HFI and PORT
		FF_CABLE_HEALTH_REPORT_DIR_WITH_HFI_PORT="${FF_CABLE_HEALTH_REPORT_DIR}/${hfi_port_dir}/"
		mkdir -p ${FF_CABLE_HEALTH_REPORT_DIR_WITH_HFI_PORT}
		# Maintain the number of cable health report files at FF_CABLE_HEALTH_MAX_FILES
		if [ -n "${FF_CABLE_HEALTH_MAX_FILES}" ]
		then
			files_in_dir=`ls ${FF_CABLE_HEALTH_REPORT_DIR_WITH_HFI_PORT} | wc -l `
			files_to_delete=`expr $files_in_dir - ${FF_CABLE_HEALTH_MAX_FILES}`
			if [ $files_to_delete -gt 0 ]
			then
				find ${FF_CABLE_HEALTH_REPORT_DIR_WITH_HFI_PORT} -name "cablehealth*" -maxdepth 1 -type f 2>/dev/null | xargs -r ls -1tr | head -n $files_to_delete | xargs -r rm
			fi
		fi
		/usr/sbin/opareport -h $hfi -p $port -o cablehealth 2>/dev/null >${FF_CABLE_HEALTH_REPORT_DIR_WITH_HFI_PORT}/cablehealth$(date "+%Y%m%d%H%M%S").csv
	fi
done

echo "Done."

exit 0
