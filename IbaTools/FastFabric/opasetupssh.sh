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
# setup password-less ssh on a group of hosts

# optional override of defaults
if [ -f /etc/opa/opafastfabric.conf ]
then
	. /etc/opa/opafastfabric.conf
fi

. /usr/lib/opa/tools/opafastfabric.conf.def

. /usr/lib/opa/tools/ff_funcs

trap "exit 1" SIGHUP SIGTERM SIGINT

if [ x"$FF_IPOIB_SUFFIX" = xNONE ]
then
	export FF_IPOIB_SUFFIX=""
fi

Usage_full()
{
	echo "Usage: opasetupssh [-CpU] [-f hostfile] [-F chassisfile] [-h 'hosts']" >&2
	echo "                      [-H 'chassis'] [-i ipoib_suffix] [-u user] [-S]" >&2
	echo "                      [-RP]" >&2
	echo "              or" >&2
	echo "       opasetupssh --help" >&2
	echo >&2
	echo "   --help - produce full help text" >&2
	echo "   -C - perform operation against chassis, default is hosts" >&2
	echo "   -p - perform operation against all chassis/hosts in parallel" >&2
	echo "   -U - only perform connect (to enter in local hosts knownhosts)" >&2
	echo "        (when run in this mode the -S option is ignored)" >&2
    echo >&2
	echo "   -f hostfile     - file with hosts in cluster, default is " >&2
	echo "                     $CONFIG_DIR/opa/hosts" >&2
	echo "   -F chassisfile  - file with chassis in cluster, default is" >&2
	echo "                     $CONFIG_DIR/opa/chassis" >&2
	echo "   -h hosts        - list of hosts to setup" >&2
	echo "   -H chassis      - list of chassis to setup" >&2
	echo >&2
	echo "   -i ipoib_suffix - suffix to apply to host names to create ipoib" >&2
	echo "                     host names default is '$FF_IPOIB_SUFFIX'" >&2
	echo "   -u user         - user on remote system to allow this user to ssh to," >&2
	echo "                     default is current user code for host(s) and admin" >&2
	echo "                     for chassis" >&2
	echo "   -S - securely prompt for password for user on remote system" >&2
	echo "   -R - skip setup of ssh to localhost" >&2
	echo "   -P - skip ping of host (for ssh to devices on internet with ping firewalled)" >&2
	echo >&2
	echo " Environment:" >&2
	echo "   HOSTS_FILE - file containing list of hosts, used in absence of -f and -h" >&2
	echo "   CHASSIS_FILE - file containing list of chassis, used in absence of -F and -H" >&2
	echo "   HOSTS - list of hosts, used if -h option not supplied" >&2
	echo "   CHASSIS - list of chassis, used if -C used and -H and -F options not supplied" >&2
	echo "   FF_MAX__PARALLEL - when -p option is used, maximum concurrent operations" >&2
	echo "   FF_IPOIB_SUFFIX - suffix to apply to hostnames to create ipoib hostnames," >&2
	echo "         used in absence of -i" >&2
	echo "   FF_CHASSIS_LOGIN_METHOD - how to login to chassis: telnet or ssh" >&2
	echo "   FF_CHASSIS_ADMIN_PASSWORD - password for chassis, used in absence of -S" >&2
	echo >&2
	echo "example:">&2
	echo "  Operations on hosts" >&2
	echo "   opasetupssh -S -i''" >&2
	echo "   opasetupssh -U" >&2
	echo "   opasetupssh -h 'arwen elrond' -U" >&2
	echo "   HOSTS='arwen elrond' opasetupssh -U" >&2
	echo "  Operations on chassis" >&2
	echo "   opasetupssh -C" >&2
	echo "   opasetupssh -C -H 'chassis1 chassis2'" >&2
	echo "   CHASSIS='chassis1 chassis2' opasetupssh -C" >&2
	exit 0
}

Usage()
{
	echo "Usage: opasetupssh [-CpU] [-f hostfile] [-F chassisfile]" >&2
	echo "                      [-i ipoib_suffix] [-S]" >&2
	echo "              or" >&2
	echo "       opasetupssh --help" >&2
	echo "   --help - produce full help text" >&2
	echo "   -C - perform operation against chassis, default is hosts" >&2
	echo "   -p - perform operation against all chassis/hosts in parallel" >&2
	echo "   -U - only perform connect (to enter in local hosts knownhosts)" >&2
	echo "         (when run in this mode, the -S options is ignored)" >&2
	echo "   -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "   -F chassisfile - file with chassis in cluster, default is" >&2
	echo "         $CONFIG_DIR/opa/chassis" >&2
	echo "   -i ipoib_suffix - suffix to apply to host names to create ipoib host names" >&2
	echo "         default is '$FF_IPOIB_SUFFIX'" >&2
	echo "   -S - securely prompt for password for user on remote system" >&2
	echo >&2
	echo "example:">&2
	echo "  Operations on hosts" >&2
	echo "   opasetupssh -S -i''" >&2
	echo "   opasetupssh -U" >&2
	echo "  Operations on chassis" >&2
	echo "   opasetupssh -C" >&2
	exit 2
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

myuser=`id -u -n`
user=$myuser
Uopt=n
popt=n
Ropt=n
Sopt=n
uopt=n
password=
chassis=0
host=0
ipoib=0
skip_ping=n
bracket=''
close_bracket=''

shellcmd="ssh -o StrictHostKeyChecking=no"
copycmd=scp
bracket='['
close_bracket=']'

while getopts CpUi:f:h:u:H:F:SRP param
do
	case $param in
	C)
		chassis=1;;
	p)
		popt=y;;
	U)
		Uopt=y;;
	i)
		ipoib=1
		FF_IPOIB_SUFFIX="$OPTARG";;
	h)
		host=1
		HOSTS="$OPTARG";;
	H)
		chassis=1
		CHASSIS="$OPTARG";;
	f)
		host=1
		HOSTS_FILE="$OPTARG";;
	F)
		chassis=1
		CHASSIS_FILE="$OPTARG";;
	u)
		uopt=y
		user="$OPTARG";;
	S)
		Sopt=y;;
	R)
		Ropt=y;;
	P)
		skip_ping=y;;
	?)
		Usage;;
	esac
done
shift $((OPTIND -1))
if [ $# -gt 0 ]
then
	Usage
fi
if [[ $(($chassis+$host)) -gt 1 ]]
then
	echo "opasetupssh: conflicting arguments, host and chassis both specified" >&2
	Usage
fi
if [[ $(($chassis+$host)) -eq 0 ]]
then
	host=1
fi
if [[ $(($chassis+$ipoib)) -gt 1 ]]
then
	echo "opasetupssh: conflicting arguments, IPOIB and chassis both specified" >&2
	Usage
fi
if [ "$Uopt" = y -a "$Sopt" = y ]
then
	echo "opasetupssh: Warning: -S option ignored in conjunction with -U" >&2
	Sopt=n
fi
if [ "$Uopt" = y ]
then
	# need commands setup so can run setup_self_ssh on remote host
	shellcmd="ssh -o StrictHostKeyChecking=no"
	copycmd=scp
	bracket='['
	close_bracket=']'
fi
if [ "$Sopt" = y ]
then
        bracket='\['
        close_bracket='\]'
fi
if [ "$chassis" = 1 -a "$Ropt" = y ]
then
	echo "opasetupssh: Warning: -R option ignored for chassis" >&2
	Ropt=n
fi
# for chassis, make the default user to be admin (unless specified)
if [ $chassis = 1 -a  "$uopt" = n ]
then
	user="admin"
fi

if [ $chassis -eq 1 ]
then
	shellcmd=ssh
	copycmd=scp
	export CFG_CHASSIS_LOGIN_METHOD="$FF_CHASSIS_LOGIN_METHOD"
	export CFG_CHASSIS_ADMIN_PASSWORD="$FF_CHASSIS_ADMIN_PASSWORD"
	if [ "$Sopt" = y ]
	then
		given_pwd='entered'
	else		
		given_pwd='default'
	fi
fi
#  if user requested to enter a password securely, prompt user and save password
if [ $host -eq 1 ]
then
	check_host_args opasetupssh
	if [ "$Sopt" = y ]
	then
		echo -n "Password for $user on all hosts: " > /dev/tty
		stty -echo < /dev/tty > /dev/tty
		password=
		read password < /dev/tty
		stty echo < /dev/tty > /dev/tty
		echo > /dev/tty
		export password
	 fi
else
	check_chassis_args opasetupssh
	if [ "$Sopt" = y ]
	then
		echo -n "Password for $user on all chassis: " > /dev/tty
		stty -echo < /dev/tty > /dev/tty
		password=
		read password < /dev/tty
		stty echo < /dev/tty > /dev/tty
		echo > /dev/tty
		export CFG_CHASSIS_ADMIN_PASSWORD="$password"
	fi
fi

# connect to host via ssh
connect_to_host()
{
	# $1 = user
	# $2 = host

	# We use an alternate file to build up the new keys
	# this way parallel calls can let ssh itself handle file locking
	# then update_known_hosts can replace data in real known_hosts file
	ssh -o 'UserKnownHostsFile=~/.ssh/.known_hosts-ffnew' -o 'StrictHostKeyChecking=no' $1@$2 echo "$2: Connected"
}

# update known_hosts file with information from connect_to_host calls
update_known_hosts()
{
	if [ -e ~/.ssh/.known_hosts-ffnew ]
	then
		if [ -e ~/.ssh/known_hosts ]
		then
			(
			IFS=" ,	"
			while read name trash
			do
				# remove old entry from known hosts in case key changed
				if grep "^$name[, ]" < ~/.ssh/known_hosts > /dev/null 2>&1
				then
					grep -v "^$name[, ]" < ~/.ssh/known_hosts > ~/.ssh/.known_hosts-fftmp
					mv ~/.ssh/.known_hosts-fftmp ~/.ssh/known_hosts
				fi
			done < ~/.ssh/.known_hosts-ffnew
			)
		fi
		total_processed=$(( $total_processed + $(wc -l < ~/.ssh/.known_hosts-ffnew) ))
		cat ~/.ssh/.known_hosts-ffnew >> ~/.ssh/known_hosts
		chmod go-w ~/.ssh/known_hosts

		rm -rf ~/.ssh/.known_hosts-ffnew ~/.ssh/.known_hosts-fftmp
	fi
}

# Generate public and private SSH key pairs
cd ~
mkdir -m 0700 -p ~/.ssh ~/.ssh2
permission_ssh=$(stat -c %a ~/.ssh)
if [ "$permission_ssh" -ne 700 ]; then
		chmod 700 ~/.ssh
        echo "Warning: ~/.ssh dir permissions are $permission_ssh. Changed to 700.."
fi

permission_ssh2=$(stat -c %a ~/.ssh2)
if [ "$permission_ssh2" -ne 700 ]; then
		chmod 700 ~/.ssh2
        echo "Warning: ~/.ssh2 dir permissions are $permission_ssh2. Changed to 700."
fi

if [ ! -f ~/.ssh/id_rsa.pub -o ! -f ~/.ssh/id_rsa ]
then
	ssh-keygen -P "" -t rsa -f ~/.ssh/id_rsa
fi
if [ ! -f .ssh/id_dsa.pub -o ! -f .ssh/id_dsa ]
then
	ssh-keygen -P "" -t dsa -f ~/.ssh/id_dsa
fi
# recreate public key in Reflection format for ssh2
if [ ! -f .ssh2/ssh2_id_dsa.pub ]
then
	# older distros may not support this, ignore error
	ssh-keygen -P "" -O ~/.ssh/id_dsa.pub -o ~/.ssh2/ssh2_id_dsa.pub 2>/dev/null
fi

# send command to the host and handle any password prompt if user supplied secure password
run_host_cmd()
{
if [ "$Sopt" = "n" ]
then
	$*
else
	expect -c "
global env
spawn -noecho $*
expect {
{assword:} {
		puts -nonewline stdout \"<password supplied>\"
		flush stdout
		exp_send \"\$env(password)\\r\"
		interact
		wait
	}
{assphrase for key} {
		puts stdout \"\nError: PassPhrase not supported. Remove PassPhrase\"
		flush stdout
		exit
        }
{continue connecting} { exp_send \"yes\\r\"
		exp_continue
	}
}
"
fi
}

# send command to the chassis and always expect to be prompted for password
run_chassis_cmd()
{
expect -c "
global env
spawn -noecho $*
expect {
{assword:} {
		puts -nonewline stdout \"<password supplied>\"
		flush stdout
		exp_send \"$CFG_CHASSIS_ADMIN_PASSWORD\r\"
		interact
		wait
	}
{assphrase for key} {
		puts stdout \"\nError: PassPhrase not supported. Remove PassPhrase\"
		flush stdout
		exit
        }
{continue connecting} { exp_send \"yes\\r\"
		exp_continue
	}
}
"
}

# connect to chassis via ssh
connect_to_chassis()
{
	# $1 = user
	# $2 = chassis
	# $3 = 1 if display connected

	# We use an alternate file to build up the new keys
	# this way parallel calls can let ssh itself handle file locking
	# then update_known_hosts can replace data in real known_hosts file
	ssh -o 'UserKnownHostsFile=~/.ssh/.known_hosts-ffnew' -o 'StrictHostKeyChecking=no' $1@$2 "chassisQuery" 2>&1| grep -q 'slots:'
	if [ $? -eq 0 ]
	then
		if [ $3 = 1 ]
		then 
			echo "$2: Connected"
		fi
		return 0
	else
		if [ $3 = 1 ]
		then 
			echo "$2: Can't Connect"
		fi
		return 1
	fi
}

# do selected opasetupssh operation for a single host
process_host()
{
	local hostname=$1
	local setup_self_ssh

	thost=`ff_host_basename $hostname`
	thost_ib=`ff_host_basename_to_ipoib $thost`
	#setup_self_ssh='[ -x /usr/lib/opa/tools/setup_self_ssh ] && /usr/lib/opa/tools/setup_self_ssh'
	setup_self_ssh='/tmp/setup_self_ssh'
	if [ "$thost" != "$thost_ib" ]
	then
		setup_self_ssh="$setup_self_ssh -i $thost_ib"
	fi
	[ "$skip_ping" = "y" ] || ping_host $thost
	if [ $? != 0 ]
	then
		echo "Couldn't ping $thost"
		sleep 1
	else
		if [ "$Uopt" = n ]
		then
			echo "Configuring $thost..."
			run_host_cmd $shellcmd -l $user $thost mkdir -m 0700 -p '~/.ssh' '~/.ssh2'
			run_host_cmd $shellcmd -l $user $thost "stat -c %a ~/.ssh > /tmp/perm_ssh &&  stat -c %a ~/.ssh2 > /tmp/perm_ssh2"
			run_host_cmd $copycmd $user@$bracket$thost$close_bracket:/tmp/perm_ssh /tmp/perm_ssh.$user.$thost
			run_host_cmd $copycmd $user@$bracket$thost$close_bracket:/tmp/perm_ssh2 /tmp/perm_ssh2.$user.$thost
			run_host_cmd $shellcmd -l $user $thost "rm -f /tmp/perm_ssh /tmp/perm_ssh2"

			if [ -f /tmp/perm_ssh.$user.$thost ]
			then
				permission_ssh=$(cat /tmp/perm_ssh.$user.$thost)
			else
				echo "Warning: /tmp/perm_ssh.$user.$thost: No such File."
			fi

			if [ -f /tmp/perm_ssh2.$user.$thost ]
			then
				permission_ssh2=$(cat /tmp/perm_ssh2.$user.$thost)
			else
				echo "Warning: /tmp/perm_ssh2.$user.$thost: No such File."
			fi

			if [ "$permission_ssh" -ne 700 -o "$permission_ssh2" -ne 700 ]; then
				run_host_cmd $shellcmd -l $user $thost chmod 700 '~/.ssh' '~/.ssh2'
				echo "Warning: $user@$thost:~/.ssh, ~/.ssh2 dir permissions are $permission_ssh/$permission_ssh2. Changed to 700.."
			fi
			rm -f /tmp/perm_ssh.$user.$thost /tmp/perm_ssh2.$user.$thost
			run_host_cmd $copycmd ~/.ssh/id_rsa.pub $user@$bracket$thost$close_bracket:.ssh/$ihost.$myuser.id_rsa.pub
			run_host_cmd $copycmd ~/.ssh/id_dsa.pub $user@$bracket$thost$close_bracket:.ssh/$ihost.$myuser.id_dsa.pub

			run_host_cmd $shellcmd -l $user $thost ">> ~/.ssh/authorized_keys"
			run_host_cmd $shellcmd -l $user $thost "cat ~/.ssh/authorized_keys ~/.ssh/$ihost.$myuser.id_rsa.pub ~/.ssh/$ihost.$myuser.id_dsa.pub |sort -u > ~/.ssh/.tmp_keys"
			run_host_cmd $shellcmd -l $user $thost "mv ~/.ssh/.tmp_keys ~/.ssh/authorized_keys"

			run_host_cmd $shellcmd -l $user $thost ">> ~/.ssh/authorized_keys2"
			run_host_cmd $shellcmd -l $user $thost "cat ~/.ssh/authorized_keys2 ~/.ssh/$ihost.$myuser.id_rsa.pub ~/.ssh/$ihost.$myuser.id_dsa.pub |sort -u > ~/.ssh/.tmp_keys2"
			run_host_cmd $shellcmd -l $user $thost "mv ~/.ssh/.tmp_keys2 ~/.ssh/authorized_keys2"

			if [ -f ~/.ssh2/ssh2_id_dsa.pub ]
			then
				run_host_cmd $copycmd ~/.ssh2/ssh2_id_dsa.pub $user@$thost:.ssh2/$ihost.$myuser.ssh2_id_dsa.pub
			else
				# older distros may not support this, ignore error
				run_host_cmd $shellcmd -l $user $thost "ssh-keygen -P "" -O ~/.ssh/$ihost.$myuser.id_dsa.pub -o ~/.ssh2/$ihost.$myuser.ssh2_id_dsa.pub 2>/dev/null"
			fi
			run_host_cmd $shellcmd -l $user $thost "test -f ~/.ssh2/$ihost.$myuser.ssh2_id_dsa.pub && ! grep '^Key $ihost.$myuser.ssh2_id_dsa.pub\$' ~/.ssh2/authorization >/dev/null 2>&1 && echo 'Key $ihost.$myuser.ssh2_id_dsa.pub' >> .ssh2/authorization"

			run_host_cmd $shellcmd -l $user $thost "chmod go-w ~/.ssh/authorized_keys ~/.ssh/authorized_keys2"
			run_host_cmd $shellcmd -l $user $thost "test -f ~/.ssh2/authorization && chmod go-w ~/.ssh2/authorization"
			connect_to_host $user $thost
			if [ "$thost" != "$thost_ib" ]
			then
				connect_to_host $user $thost_ib
			fi
			if [ "$Ropt" = "n" ]
			then
				run_host_cmd $copycmd /usr/lib/opa/tools/setup_self_ssh $user@$bracket$thost$close_bracket:/tmp/setup_self_ssh
				run_host_cmd $shellcmd -l $user $thost "$setup_self_ssh"
				run_host_cmd $shellcmd -l $user $thost "rm -f /tmp/setup_self_ssh"
			fi
			echo "Configured $thost"
		else
			echo "Connecting to $thost..."
			connect_to_host $user $thost
			if [ "$thost" != "$thost_ib" ]
			then
				connect_to_host $user $thost_ib
			fi
			if [ "$Ropt" = "n" ]
			then
				run_host_cmd $copycmd /usr/lib/opa/tools/setup_self_ssh $user@$bracket$thost$close_bracket:/tmp/setup_self_ssh
				run_host_cmd $shellcmd -l $user $thost "$setup_self_ssh -U"
				run_host_cmd $shellcmd -l $user $thost "rm -f /tmp/setup_self_ssh"
			fi
		fi
	fi
}

wait_chassis_scp()
{
	local tchassis user
	tchassis="$1"
	user="$2"

	waited=0
	while true
	do
		# since chassis is returning incorrectly for scp response
	   	# we must use their workaround by checking RetCode
		ret=$($chassis_cmd $tchassis $user "showLastScpRetCode -all" 2>&1)
		case "$ret" in
		*In\ Progress*)
			sleep 1
			waited=$(($waited + 1))
			if [ $waited -gt 30 ]	# 30 seconds is more than enough
			then
				echo "$tchassis: Timeout waiting for scp complete"
				return 1
			fi;;
		*Success*)
			return 0;;
		*)
			echo "$tchassis: $ret"
			return 1;;
		esac
	done
}

# do selected opasetupssh operation for a single chassis
process_chassis()
{
	local chassisname=$1
	local tempfile=~/.ssh/.tmp_chassis_keys.$running

	tchassis=`strip_chassis_slots  $chassisname`
	[ "$skip_ping" = "y" ] || ping_host $tchassis
	if [ $? != 0 ]
	then
		echo "Couldn't ping $tchassis"
		sleep 1
	else
		if [ "$Uopt" = n ]
		then
			echo "Configuring $tchassis..."
			# ensure that we can login successfully by trying any simple command with expected response
			$chassis_cmd $tchassis $user "chassisQuery" 2>&1| grep -q 'slots:'
			if [ $? -ne 0 ]
			then
				echo "Login to $tchassis failed for the $given_pwd password, skipping..."
				continue
			fi
			rm -f $tempfile $tempfile.2 2>/dev/null
			/usr/lib/opa/tools/tcl_proc chassis_sftp_cmd "sftp $user@\[${tchassis}\]:" "get /firmware/$user/authorized_keys $tempfile" 2>&1| grep -q 'FAILED'
			if [ $? -eq 0 ] || [ ! -f $tempfile ]
			then
				echo "Unable to configure $tchassis for password-less ssh, skipping..."
				continue
			fi
			cat ~/.ssh/id_rsa.pub $tempfile | sort -u > $tempfile.2
			/usr/lib/opa/tools/tcl_proc chassis_sftp_cmd "sftp $user@\[${tchassis}\]:" "put $tempfile.2 /firmware/$user/authorized_keys" 2>&1| grep -q 'FAILED'
			if [ $? -eq 0 ]
			then
				echo "$tchassis password-less ssh config failed, skipping..."
			else
				connect_to_chassis $user $tchassis 1
				if [ $? -eq 0 ]
				then
					echo "Configured $tchassis"
				fi
			fi
			rm -f $tempfile $tempfile.2 2>/dev/null
		else
			echo "Connecting to $tchassis..."
			connect_to_chassis $user $tchassis 1
		fi
	fi
} 

# configure ssh on the host(s) or chassis
running=0
total_processed=0
rm -rf ~/.ssh/.known_hosts-ffnew ~/.ssh/.known_hosts-fftmp

stty_settings=`stty -g`

if [ $host -eq 1 ]
then
	ihost=`hostname | cut -f1 -d.`
	ihost_ib=`ff_host_basename_to_ipoib $ihost`
	# setup ssh to ourselves
	# This can also help if the .ssh directory is in a shared filesystem
	rm -f ~/.ssh/.tmp_keys$$

	>> ~/.ssh/authorized_keys
	cat ~/.ssh/authorized_keys ~/.ssh/id_rsa.pub ~/.ssh/id_dsa.pub|sort -u > ~/.ssh/.tmp_keys$$
	mv ~/.ssh/.tmp_keys$$ ~/.ssh/authorized_keys

	>> ~/.ssh/authorized_keys2
	cat ~/.ssh/authorized_keys2 ~/.ssh/id_rsa.pub ~/.ssh/id_dsa.pub|sort -u > ~/.ssh/.tmp_keys$$
	mv ~/.ssh/.tmp_keys$$ ~/.ssh/authorized_keys2

	# set up ssh2 DSA authorization
	# older distros may not support this
	[ -f ~/.ssh2/ssh2_id_dsa.pub ] && !  grep '^Key ssh2_id_dsa.pub$' ~/.ssh2/authorization >/dev/null 2>&1 && echo "Key ssh2_id_dsa.pub" >> ~/.ssh2/authorization

	chmod go-w ~/.ssh/authorized_keys ~/.ssh/authorized_keys2
	test -f ~/.ssh2/authorization && chmod go-w ~/.ssh2/authorization

	if [ "$Ropt" = "n" ]
	then
		echo "Verifying localhost ssh..."
		if [ "$uopt" = n ]
		then
			# make sure we can ssh ourselves so ipoibping test works
			connect_to_host $user localhost
			connect_to_host $user $ihost
			# make sure we can ssh to ourselves over ipoib, so MPI can be run on master
			connect_to_host $user $ihost_ib
		else
			run_host_cmd $shellcmd -l $user localhost echo localhost connected.. 
			run_host_cmd $shellcmd -l $user $ihost echo $ihost connected..
			run_host_cmd $shellcmd -l $user $ihost_ib echo $ihost_ib connected..
		fi

		stty $stty_settings
		update_known_hosts
	fi

	for hostname in $HOSTS
	do
		if [ "$popt" = y ]
		then
			if [ $running -ge $FF_MAX_PARALLEL ]
			then
				wait
				stty $stty_settings
				update_known_hosts
				running=0
			fi
			process_host $hostname < /dev/tty &
			running=$(($running +1))
		else
			process_host $hostname
			stty $stty_settings
			update_known_hosts
		fi
	done
else
	chassis_cmd='/usr/lib/opa/tools/tcl_proc chassis_run_cmd'
	for chassisname in $CHASSIS
	do
		if [ "$popt" = y ]
		then
			if [ $running -ge $FF_MAX_PARALLEL ]
			then
				wait
				stty $stty_settings
				update_known_hosts
				running=0
			fi
			process_chassis $chassisname < /dev/tty &
			running=$(($running +1))
		else
			process_chassis $chassisname
			stty $stty_settings
			update_known_hosts
		fi
	done
fi
wait
stty $stty_settings
update_known_hosts

echo "Successfully processed: $total_processed"
