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
# perform installation verification on hosts in a cluster

# optional override of defaults
if [ -f /etc/opa/opafastfabric.conf ]
then
	. /etc/opa/opafastfabric.conf
fi

. /usr/lib/opa/tools/opafastfabric.conf.def

. /usr/lib/opa/tools/ff_funcs

if [ x"$FF_IPOIB_SUFFIX" = xNONE ]
then
	export FF_IPOIB_SUFFIX=""
fi

# identify how we are being run, affects valid options and usage
mode=invalid
cmd=`basename $0`
case $cmd in
opahostadmin|opaswitchadmin|opachassisadmin) mode=$cmd;;
esac

if [ "$mode" = "invalid" ]
then
	echo "Invalid executable name for this file: $cmd; expected opahostadmin, opaswitchadmin or opachassisadmin" >&2
	exit 1
fi

Usage_opahostadmin_full()
{
	echo "Usage: opahostadmin [-c] [-i ipoib_suffix] [-f hostfile] [-h 'hosts'] " >&2
	echo "              [-r release] [-I install_options] [-U upgrade_options] [-d dir]" >&2
	echo "              [-T product] [-P packages] [-m netmask] [-S] operation ..." >&2
	echo "              or" >&2
	echo "       opahostadmin --help" >&2
	echo "  --help - produce full help text" >&2
	echo "  -c - clobber result files from any previous run before starting this run" >&2
	echo "  -i ipoib_suffix - suffix to apply to host names to create ipoib host names" >&2
	echo "                    default is '$FF_IPOIB_SUFFIX'" >&2
	echo "  -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "  -h hosts - list of hosts to execute operation against" >&2
	echo "  -r release - IntelOPA release to load/upgrade to, default is $FF_PRODUCT_VERSION" >&2
	echo "  -d dir - directory to get product.release.tgz from for load/upgrade" >&2
	echo "  -I install_options - IntelOPA install options" >&2
	echo "  -U upgrade_options - IntelOPA upgrade options" >&2
	echo "  -T product - IntelOPA product type to install" >&2
	echo "               default is $FF_PRODUCT" >&2
	echo "               Other options include: IntelOPA-Basic.<distro>," >&2
	echo "                                      IntelOPA-IFS.<distro>" >&2
	echo "               Where <distro> is the distro and CPU, such as RHEL7-x86_64" >&2
	echo "  -P packages - IntelOPA packages to install, default is '$FF_PACKAGES'" >&2
	echo "                See IntelOPA INSTALL -C for a complete list of packages" >&2
	echo "  -m netmask - IPoIB netmask to use for configipoib" >&2
	echo "  -S - securely prompt for password for user on remote system" >&2
	echo "  operation - operation to perform. operation can be one or more of:" >&2
	echo "              load - initial install of all hosts" >&2
	echo "              upgrade - upgrade install of all hosts" >&2
	echo "              configipoib - create ifcfg-ib1 using host IP addr from /etc/hosts" >&2
	echo "              reboot - reboot hosts, ensure they go down and come back" >&2
	echo "              sacache - confirm sacache has all hosts in it" >&2
	echo "              ipoibping - verify this host can ping each host via IPoIB" >&2
	echo "              mpiperf - verify latency and bandwidth for each host" >&2
	echo "              mpiperfdeviation - check for latency and bandwidth tolerance" >&2
	echo "                                 deviation between hosts" >&2
	echo " Environment:" >&2
	echo "   HOSTS - list of hosts, used if -h option not supplied" >&2
	echo "   HOSTS_FILE - file containing list of hosts, used in absence of -f and -h" >&2
	echo "   FF_MAX_PARALLEL - maximum concurrent operations" >&2
	echo "   FF_SERIALIZE_OUTPUT - serialize output of parallel operations (yes or no)" >&2
	echo "   FF_TIMEOUT_MULT - Multiplier for all timeouts associated with this command." >&2
	echo "                     Used if the systems are slow for some reason." >&2
	echo "for example:" >&2
	echo "   opahostadmin -c reboot" >&2
	echo "   opahostadmin upgrade" >&2
	echo "   opahostadmin -h 'elrond arwen' reboot" >&2
	echo "   HOSTS='elrond arwen' opahostadmin reboot" >&2
	echo "During run the following files are produced:" >&2
	echo "  test.res - appended with summary results of run" >&2
	echo "  test.log - appended with detailed results of run" >&2
	echo "  save_tmp/ - contains a directory per failed operation with detailed logs" >&2
	echo "  test_tmp*/ - intermediate result files while operation is running" >&2
	echo "-c option will remove all of the above" >&2
	exit 0
}
Usage_opachassisadmin_full()
{
	echo "Usage: opachassisadmin [-c] [-F chassisfile] [-H 'chassis'] " >&2
	echo "              [-P packages] [-a action] [-I fm_bootstate]" >&2
	echo "              [-S] [-d upload_dir] [-s securityfiles] operation ..." >&2
	echo "              or" >&2
	echo "       opachassisadmin --help" >&2
	echo "  --help - produce full help text" >&2
	echo "  -c - clobber result files from any previous run before starting this run" >&2
	echo "  -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "  -H chassis - list of chassis to execute operation against" >&2
	echo "  -P packages - filenames/directories of firmware" >&2
	echo "                   images to install.  For directories specified, all" >&2
	echo "                   .pkg, .dpkg and .spkg files in directory tree will be used." >&2
	echo "                   shell wildcards may also be used within quotes." >&2
	echo "                or for fmconfig, filename of FM config file to use" >&2
	echo "                or for fmgetconfig, filename to upload to (default" >&2
	echo "                   opafm.xml)" >&2
	echo "  -a action - action for supplied file" >&2
	echo "              For Chassis upgrade:" >&2
	echo "                 push   - ensure firmware is in primary or alternate" >&2
	echo "                 select - ensure firmware is in primary" >&2
	echo "                 run    - ensure firmware is in primary and running" >&2
	echo "                 default is push" >&2
	echo "              For Chassis fmconfig:" >&2
	echo "                 push   - ensure config file is in chassis" >&2
	echo "                 run    - after push, restart FM on master, stop on slave" >&2
	echo "                 runall - after push, restart FM on all MM" >&2
	echo "              For Chassis fmcontrol:" >&2
	echo "                 stop   - stop FM on all MM" >&2
	echo "                 run    - make sure FM running on master, stopped on slave" >&2
	echo "                 runall - make sure FM running on all MM" >&2
	echo "                 restart- restart FM on master, stop on slave" >&2
	echo "                 restartall- restart FM on all MM" >&2
	echo "              For Chassis fmsecurityfiles:" >&2
	echo "                 push   - ensure FM security files are in chassis" >&2
	echo "                 restart- after push, restart FM on master, stop on slave" >&2
	echo "                 restartall - after push, restart FM on all MM" >&2
	echo "  -I fm_bootstate fmconfig and fmcontrol install options" >&2
	echo "                 disable - disable FM start at chassis boot" >&2
	echo "                 enable - enable FM start on master at chassis boot" >&2
	echo "                 enableall - enable FM start on all MM at chassis boot" >&2
	echo "  -d upload_dir - directory to upload FM config files to, default is uploads" >&2
	echo "  -S - securely prompt for password for admin on chassis" >&2
	echo "  -s securityFiles - security files to install, default is '*.pem'" >&2
	echo "                For Chassis fmsecurityfiles, filenames/directories of" >&2
	echo "                   security files to install.  For directories specified," >&2
	echo "                   all security files in directory tree will be used." >&2
	echo "                   shell wildcards may also be used within quotes." >&2
	echo "                or for Chassis fmgetsecurityfiles, filename to upload to" >&2
	echo "                   (default *.pem)" >&2
	echo "  operation - operation to perform. operation can be one or more of:" >&2
	echo "     reboot - reboot chassis, ensure they go down and come back" >&2
	echo "     configure - run wizard to set up chassis configuration" >&2
	echo "     upgrade - upgrade install of all chassis" >&2
	echo "     getconfig - get basic configuration of chassis" >&2
	echo "     fmconfig - FM config operation on all chassis" >&2
	echo "     fmgetconfig - Fetch FM config from all chassis" >&2
	echo "     fmcontrol - Control FM on all chassis" >&2
	echo "     fmsecurityfiles - FM security files operation on all chassis" >&2
	echo "     fmgetsecurityfiles - Fetch FM security files from all chassis" >&2
	echo " Environment:" >&2
	echo "   CHASSIS - list of chassis, used if -H and -F option not supplied" >&2
	echo "   CHASSIS_FILE - file containing list of chassis, used in absence of -F and -H" >&2
	echo "   FF_MAX_PARALLEL - maximum concurrent operations" >&2
	echo "   FF_SERIALIZE_OUTPUT - serialize output of parallel operations (yes or no)" >&2
	echo "   FF_TIMEOUT_MULT - Multiplier for all timeouts associated with this command." >&2
	echo "                     Used if the systems are slow for some reason." >&2
	echo "   UPLOADS_DIR - directory to upload to, used in absence of -d" >&2
	echo "for example:" >&2
	echo "   opachassisadmin -c reboot" >&2
	echo "   opachassisadmin -P /root/ChassisFw4.2.0.0.1 upgrade" >&2
	echo "   opachassisadmin -H 'chassis1 chassis2' reboot" >&2
	echo "   CHASSIS='chassis1 chassis2' opachassisadmin reboot" >&2
	echo "   opachassisadmin -a run -P '*.pkg' upgrade" >&2
	echo "During run the following files are produced:" >&2
	echo "  test.res - appended with summary results of run" >&2
	echo "  test.log - appended with detailed results of run" >&2
	echo "  save_tmp/ - contains a directory per failed operation with detailed logs" >&2
	echo "  test_tmp*/ - intermediate result files while operation is running" >&2
	echo "-c option will remove all of the above" >&2
	exit 0
}
Usage_opaswitchadmin_full()
{
	echo "Usage: opaswitchadmin [-c] [-N 'nodes'] [-L nodefile] [-O] [-P packages]" >&2
	echo "              [-a action] [-t portsfile] [-p ports] operation ..." >&2
	echo "              or" >&2
	echo "       opaswitchadmin --help" >&2
	echo "  --help - produce full help text" >&2
	echo "  -c - clobber result files from any previous run before starting this run" >&2
	echo "  -N nodes - list of OPA switches to execute operation against" >&2
	echo "  -L nodefile - file with OPA switches in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/switches" >&2
	echo "  -P packages - for upgrade, filename/directory of firmware" >&2
	echo "                   image to install.  For directory specified," >&2
	echo "                   .emfw file in directory tree will be used." >&2
	echo "                   shell wildcards may also be used within quotes." >&2
	echo "  -t portsfile - file with list of local HFI ports used to access" >&2
	echo "           fabric(s) for switch access, default is /etc/opa/ports" >&2
	echo "  -p ports - list of local HFI ports used to access fabric(s) for switch access" >&2
	echo "           default is 1st active port" >&2
	echo "           This is specified as hfi:port" >&2
	echo "              0:0 = 1st active port in system" >&2
	echo "              0:y = port y within system" >&2
	echo "              x:0 = 1st active port on HFI x" >&2
	echo "              x:y = HFI x, port y" >&2
	echo "              The first HFI in the system is 1.  The first port on an HFI is 1." >&2
	echo "  -a action - action for firmware file for Switch upgrade" >&2
	echo "              select - ensure firmware is in primary" >&2
	echo "              run    - ensure firmware is in primary and running" >&2
	echo "              default is select" >&2
	echo "  -O - override: for firmware upgrade, bypass version checks and force update" >&2
	echo "           unconditionally" >&2
	echo "  operation - operation to perform. operation can be one or more of:" >&2
	echo "     reboot - reboot switches, ensure they go down and come back" >&2
	echo "     configure - run wizard to set up switch configuration" >&2
	echo "            NOTE: You must reboot the switch for any new settings to be applied." >&2
	echo "     upgrade - upgrade install of all switches" >&2
	echo "     info - report f/w & h/w version, part number, and data rate capability of" >&2
	echo "            all OPA switches" >&2
	echo "     hwvpd - complete hardware VPD report of all OPA switches" >&2
	echo "     ping - ping all OPA switches - test for presence" >&2
	echo "     fwverify - report integrity of failsafe firmware of all OPA switches" >&2
	echo "     getconfig - get port configurations of a externally managed switch" >&2
	echo " Environment:" >&2
	echo "   OPASWITCHES - list of nodes, used if -N and -L option not supplied" >&2
	echo "   OPASWITCHES_FILE - file containing list of nodes, used in absence of -N and" >&2
	echo "                      -L" >&2
	echo "   FF_MAX_PARALLEL - maximum concurrent operations" >&2
	echo "   FF_SERIALIZE_OUTPUT - serialize output of parallel operations (yes or no)" >&2
	echo "   FF_TIMEOUT_MULT - Multiplier for all timeouts associated with this command." >&2
	echo "                     Used if the systems are slow for some reason." >&2
	echo "for example:" >&2
	echo "   opaswitchadmin -c reboot" >&2
	echo "   opaswitchadmin -P /root/ChassisFw4.2.0.0.1 upgrade" >&2
	echo "   opaswitchadmin -a run -P '*.emfw' upgrade" >&2
	echo "During run the following files are produced:" >&2
	echo "  test.res - appended with summary results of run" >&2
	echo "  test.log - appended with detailed results of run" >&2
	echo "  save_tmp/ - contains a directory per failed operation with detailed logs" >&2
	echo "  test_tmp*/ - intermediate result files while operation is running" >&2
	echo "-c option will remove all of the above" >&2
	exit 0
}
Usage_full()
{
	case $mode in
	opahostadmin) Usage_opahostadmin_full;;
	opachassisadmin) Usage_opachassisadmin_full;;
	opaswitchadmin) Usage_opaswitchadmin_full;;
	esac
}
Usage_opahostadmin()
{
	echo "Usage: opahostadmin [-c] [-f hostfile] [-r release] [-d dir]" >&2
	echo "              [-T product] [-P packages] [-S] operation ..." >&2
	echo "              or" >&2
	echo "       opahostadmin --help" >&2
	echo "  --help - produce full help text" >&2
	echo "  -c - clobber result files from any previous run before starting this run" >&2
	echo "  -f hostfile - file with hosts in cluster, default is $CONFIG_DIR/opa/hosts" >&2
	echo "  -r release - IntelOPA release to load/upgrade to, default is $FF_PRODUCT_VERSION" >&2
	echo "  -d dir - directory to get product.release.tgz from for load/upgrade" >&2
	echo "  -T product - IntelOPA product type to install" >&2
	echo "               default is $FF_PRODUCT" >&2
	echo "               Other options include: IntelOPA-Basic.<distro>, IntelOPA-IFS.<distro>" >&2
	echo "               Where <distro> is the distro and CPU, such as RHEL7-x86_64" >&2
	echo "  -P packages - IntelOPA packages to install, default is '$FF_PACKAGES'" >&2
	echo "                See IntelOPA INSTALL -C for a complete list of packages" >&2
	echo "  -S - securely prompt for password for user on remote system" >&2
	echo "  operation - operation to perform. operation can be one or more of:" >&2
	echo "              load - initial install of all hosts" >&2
	echo "              upgrade - upgrade install of all hosts" >&2
	echo "              configipoib - create ifcfg-ib1 using host IP addr from /etc/hosts" >&2
	echo "              reboot - reboot hosts, ensure they go down and come back" >&2
	echo "              sacache - confirm sacache has all hosts in it" >&2
	echo "              ipoibping - verify this host can ping each host via IPoIB" >&2
	echo "              mpiperf - verify latency and bandwidth for each host" >&2
	echo "              mpiperfdeviation - check for latency and bandwidth tolerance" >&2
	echo "                        	       deviation between hosts" >&2
	echo "for example:" >&2
	echo "   opahostadmin  -c reboot" >&2
	echo "   opahostadmin  upgrade" >&2
	echo "During run the following files are produced:" >&2
	echo "  test.res - appended with summary results of run" >&2
	echo "  test.log - appended with detailed results of run" >&2
	echo "  save_tmp/ - contains a directory per failed test with detailed logs" >&2
	echo "  test_tmp*/ - intermediate result files while test is running" >&2
	echo "-c option will remove all of the above" >&2
	exit 2
}
Usage_opachassisadmin()
{
	echo "Usage: opachassisadmin [-c] [-F chassisfile] " >&2
	echo "              [-P packages] [-I fm_bootstate] [-a action]" >&2
    echo "              [-S] [-d upload_dir] [-s securityfiles] operation ..." >&2
	echo "              or" >&2
	echo "       opachassisadmin --help" >&2
	echo "  --help - produce full help text" >&2
	echo "  -c - clobber result files from any previous run before starting this run" >&2
	echo "  -F chassisfile - file with chassis in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/chassis" >&2
	echo "  -P packages - filenames/directories of firmware" >&2
	echo "                   images to install.  For directories specified, all" >&2
	echo "                   .pkg, .dpkg and .spkg files in directory tree will be used." >&2
	echo "                   shell wildcards may also be used within quotes." >&2
	echo "                or for fmconfig, filename of FM config file to use" >&2
	echo "                or for fmgetconfig, filename to upload to (default" >&2
	echo "                   opafm.xml)" >&2
	echo "  -a action - action for supplied file" >&2
	echo "              For Chassis upgrade:" >&2
	echo "                 push   - ensure firmware is in primary or alternate" >&2
	echo "                 select - ensure firmware is in primary" >&2
	echo "                 run    - ensure firmware is in primary and running" >&2
	echo "                 default is push" >&2
	echo "              For Chassis fmconfig:" >&2
	echo "                 push   - ensure config file is in chassis" >&2
	echo "                 run    - after push, restart FM on master, stop on slave" >&2
	echo "                 runall - after push, restart FM on all MM" >&2
	echo "              For Chassis fmcontrol:" >&2
	echo "                 stop   - stop FM on all MM" >&2
	echo "                 run    - make sure FM running on master, stopped on slave" >&2
	echo "                 runall - make sure FM running on all MM" >&2
	echo "                 restart- restart FM on master, stop on slave" >&2
	echo "                 restartall- restart FM on all MM" >&2
	echo "              For Chassis fmsecurityfiles:" >&2
	echo "                 push   - ensure FM security files are in chassis" >&2
	echo "                 restart- after push, restart FM on master, stop on slave" >&2
	echo "                 restartall - after push, restart FM on all MM" >&2
	echo "  -I fm_bootstate fmconfig and fmcontrol install options" >&2
	echo "                 disable - disable FM start at chassis boot" >&2
	echo "                 enable - enable FM start on master at chassis boot" >&2
	echo "                 enableall - enable FM start on all MM at chassis boot" >&2
	echo "  -d upload_dir - directory to upload FM config files to, default is uploads" >&2
	echo "  -S - securely prompt for password for admin on chassis" >&2
	echo "  -s securityFiles - security files to install, default is '*.pem'" >&2
	echo "                For Chassis fmsecurityfiles, filenames/directories of" >&2
	echo "                   security files to install.  For directories specified," >&2
	echo "                   all security files in directory tree will be used." >&2
	echo "                   shell wildcards may also be used within quotes." >&2
	echo "                or for Chassis fmgetsecurityfiles, filename to upload to" >&2
	echo "                   (default *.pem)" >&2
	echo "  operation - operation to perform. operation can be one or more of:" >&2
	echo "     reboot - reboot chassis, ensure they go down and come back" >&2
	echo "     configure - run wizard to set up chassis configuration" >&2
	echo "     upgrade - upgrade install of all chassis" >&2
	echo "     getconfig - get basic configuration of chassis" >&2
	echo "     fmconfig - FM config operation on all chassis" >&2
	echo "     fmgetconfig - Fetch FM config from all chassis" >&2
	echo "     fmcontrol - Control FM on all chassis" >&2
	echo "     fmsecurityfiles - FM security files operation on all chassis" >&2
	echo "     fmgetsecurityfiles - Fetch FM security files from all chassis" >&2
	echo "for example:" >&2
	echo "   opachassisadmin -c reboot" >&2
	echo "   opachassisadmin -P /root/ChassisFw4.2.0.0.1 upgrade" >&2
	echo "   opachassisadmin -a run -P '*.pkg' upgrade" >&2
	echo "During run the following files are produced:" >&2
	echo "  test.res - appended with summary results of run" >&2
	echo "  test.log - appended with detailed results of run" >&2
	echo "  save_tmp/ - contains a directory per failed operation with detailed logs" >&2
	echo "  test_tmp*/ - intermediate result files while operation is running" >&2
	echo "-c option will remove all of the above" >&2
	exit 2
}
Usage_opaswitchadmin()
{
	echo "Usage: opaswitchadmin [-c] [-L nodefile] [-O] [-P packages]" >&2
	echo "                        [-a action] operation ..." >&2
	echo "              or" >&2
	echo "       opaswitchadmin --help" >&2
	echo "  --help - produce full help text" >&2
	echo "  -c - clobber result files from any previous run before starting this run" >&2
	echo "  -L nodefile - file with OPA switches in cluster" >&2
	echo "           default is $CONFIG_DIR/opa/switches" >&2
	echo "  -P packages - for upgrade, filename/directory of firmware" >&2
	echo "                   image to install.  For directory specified," >&2
	echo "                   .emfw file in directory tree will be used." >&2
	echo "                   shell wildcards may also be used within quotes." >&2
	echo "  -a action - action for firmware file for Switch upgrade" >&2
	echo "              select - ensure firmware is in primary" >&2
	echo "              run    - ensure firmware is in primary and running" >&2
	echo "              default is select" >&2
	echo "  -O - override: for firmware upgrade, bypass version checks and force update" >&2
	echo "           unconditionally" >&2
	echo "  operation - operation to perform. operation can be one or more of:" >&2
	echo "     reboot - reboot switches, ensure they go down and come back" >&2
	echo "     configure - run wizard to set up switch configuration" >&2
	echo "            NOTE: You must reboot the switch for any new settings to be applied." >&2
	echo "     upgrade - upgrade install of all switches" >&2
	echo "     info - report f/w & h/w version, part number, and data rate capability of" >&2
	echo "            all OPA switches" >&2
	echo "     hwvpd - complete hardware VPD report of all OPA switches" >&2
	echo "     ping - ping all OPA switches - test for presence" >&2
	echo "     fwverify - report integrity of failsafe firmware of all OPA switches" >&2
	echo "     getconfig - get port configurations of a externally managed switch" >&2
	echo "for example:" >&2
	echo "   opaswitchadmin -c reboot" >&2
	echo "   opaswitchadmin -P /root/ChassisFw4.2.0.0.1 upgrade" >&2
	echo "   opaswitchadmin -a run -P '*.emfw' upgrade" >&2
	echo "During run the following files are produced:" >&2
	echo "  test.res - appended with summary results of run" >&2
	echo "  test.log - appended with detailed results of run" >&2
	echo "  save_tmp/ - contains a directory per failed operation with detailed logs" >&2
	echo "  test_tmp*/ - intermediate result files while operation is running" >&2
	echo "-c option will remove all of the above" >&2
	exit 2
}
Usage()
{
	case $mode in
	opahostadmin) Usage_opahostadmin;;
	opachassisadmin) Usage_opachassisadmin;;
	opaswitchadmin) Usage_opaswitchadmin;;
	esac
}

if [ x"$1" = "x--help" ]
then
	Usage_full
fi

# default to install wrapper version
if [ -e /etc/opa/version_wrapper ]
then
	CFG_RELEASE=`cat /etc/opa/version_wrapper 2>/dev/null`;
fi
if [ x"$CFG_RELEASE" = x ]
then
# if no wrapper, use version of FF itself as filled in at build time
# version string is filled in by prep, special marker format for it to use
CFG_RELEASE="THIS_IS_THE_ICS_VERSION_NUMBER:@(#)000.000.000.000B000"
fi
export CFG_RELEASE=`echo $CFG_RELEASE|sed -e 's/THIS_IS_THE_ICS_VERSION_NUMBER:@(#.//' -e 's/%.*//'`
# THIS_IS_THE_ICS_INTERNAL_VERSION_NUMBER:@(#)000.000.000.000B000
# test automation configuration defaults
export CFG_INIC_SUFFIX=
export CFG_IPOIB_SUFFIX="$FF_IPOIB_SUFFIX"
export CFG_USERNAME="$FF_USERNAME"
export CFG_PASSWORD="$FF_PASSWORD"
export CFG_ROOTPASS="$FF_ROOTPASS"
export CFG_LOGIN_METHOD="$FF_LOGIN_METHOD"
export CFG_CHASSIS_LOGIN_METHOD="$FF_CHASSIS_LOGIN_METHOD"
export CFG_CHASSIS_ADMIN_PASSWORD="$FF_CHASSIS_ADMIN_PASSWORD"
export CFG_FAILOVER="n"
export CFG_FTP_SERVER=""
export CFG_IPOIB="y"
export CFG_IPOIB_MTU="2030"
export CFG_IPOIB_COMBOS=TBD
export CFG_INIC=n
export CFG_SDP=n
export CFG_SRP=n
export CFG_MPI=y
export CFG_UDAPL=n
export TEST_TIMEOUT_MULT="$FF_TIMEOUT_MULT"
export TEST_RESULT_DIR="$FF_RESULT_DIR"
export TEST_MAX_PARALLEL="$FF_MAX_PARALLEL"
export TEST_CONFIG_FILE="/dev/null"
export TL_DIR=/usr/lib/opa/tools
export TEST_IDENTIFY=no
export TEST_SHOW_CONFIG=no
export TEST_SHOW_START=yes
export CFG_PRODUCT="${FF_PRODUCT:-IntelOPA-Basic}"
export CFG_INSTALL_OPTIONS="$FF_INSTALL_OPTIONS"
export CFG_UPGRADE_OPTIONS="$FF_UPGRADE_OPTIONS"
export CFG_IPOIB_NETMASK="$FF_IPOIB_NETMASK"
export CFG_IPOIB_CONNECTED="$FF_IPOIB_CONNECTED"
export CFG_MPI_ENV="$FF_MPI_ENV"
export TEST_SERIALIZE_OUTPUT="$FF_SERIALIZE_OUTPUT"

clobber=n
host=0
chassis=0
opaswitch=0
dir=.
packages="notsupplied"
action=default
Sopt=n
sopt=n
bypassSwitchCheck=n
fwOverride=n
securityFiles="notsupplied"
case $mode in
opahostadmin) host=1; options='cd:h:f:i:r:I:U:P:T:m:S';;
opachassisadmin) chassis=1; options='a:I:cH:F:P:d:Ss:';;
opaswitchadmin) opaswitch=1; options='a:Bcd:P:p:t:L:N:O';;
esac
while getopts "$options"  param
do
	case $param in
	a)
		action="$OPTARG";;
	B)
		bypassSwitchCheck=y;;
	c)
		clobber=y;;
	d)
		dir="$OPTARG"
		export UPLOADS_DIR="$dir";;
	h)
		host=1
		HOSTS="$OPTARG";;
	H)
		chassis=1
		CHASSIS="$OPTARG";;
	N)
		opaswitch=1
		OPASWITCHES="$OPTARG";;
	f)
		host=1
		HOSTS_FILE="$OPTARG";;
	F)
		chassis=1
		CHASSIS_FILE="$OPTARG";;
	L)
		opaswitch=1
		OPASWITCHES_FILE="$OPTARG";;
	i)
		export CFG_IPOIB_SUFFIX="$OPTARG"
		export FF_IPOIB_SUFFIX="$OPTARG";;
	r)
		export FF_PRODUCT_VERSION="$OPTARG";;
	I)
		export CFG_INSTALL_OPTIONS="$OPTARG";;
	U)
		export CFG_UPGRADE_OPTIONS="$OPTARG";;
	P)
		packages="$OPTARG";;
	T)
		export CFG_PRODUCT="$OPTARG";;
	m)
		export CFG_IPOIB_NETMASK="$OPTARG";;
	p)
		export PORTS="$OPTARG";;
	t)
		export PORTS_FILE="$OPTARG";;
	s)
		securityFiles="$OPTARG";;
	S)
		Sopt=y;;
	O)
		fwOverride=y;;
	?)
		Usage;;
	esac
done
shift $((OPTIND -1))

if [ $# -lt 1 ] 
then
	Usage
fi

for tkn in $*
do
	if [[ $tkn == -* ]]
	then
		Usage
	fi
done
# given optarg selections, this error should not be able to happen
if [[ $(($chassis+$host+$opaswitch)) -gt 1 ]]
then
	echo "$cmd: conflicting arguments, more than one of host, chassis or opaswitches specified" >&2
	Usage
fi
# given mode checks, this error should not be able to happen
if [[ $(($chassis+$host+$opaswitch)) -eq 0 ]]
then
	host=1
fi
if [ ! -d "$FF_RESULT_DIR" ]
then
	echo "$cmd: Invalid FF_RESULT_DIR: $FF_RESULT_DIR: No such directory" >&2
	exit 1
fi
if [ $chassis -eq 1 ]
then
	check_chassis_args $cmd
	if [ "$action" = "default" ]
	then
		action=push
	fi
	if [ "$CFG_INSTALL_OPTIONS" = "$FF_INSTALL_OPTIONS" ]
	then
		export CFG_INSTALL_OPTIONS=
	fi
elif [ $opaswitch -eq 1 ]
then
	check_ib_transport_args $cmd
	check_ports_args $cmd
	if [ "$action" = "default" ]
	then
		action=select
	fi
else
	check_host_args $cmd

	if [ "$packages" = "notsupplied" ]
	then
		packages="$FF_PACKAGES"
	fi
	if [ "x$packages" != "x" ]
	then
		for p in $packages
		do
			CFG_INSTALL_OPTIONS="$CFG_INSTALL_OPTIONS -i $p"
		done
	fi
	if [ "x$CFG_INSTALL_OPTIONS" = "x" ]
	then
		CFG_INSTALL_OPTIONS='-i iba -i ipoib -i mpi'
	fi
fi

export CFG_HOSTS="$HOSTS"
export CFG_CHASSIS="$CHASSIS"
export CFG_OPASWITCHES="$OPASWITCHES"
export CFG_PORTS="$PORTS"
export CFG_MPI_PROCESSES="$HOSTS"
#export CFG_PERF_PAIRS=TBD
export CFG_SCPFROMDIR="$dir"
if [ x"$FF_PRODUCT_VERSION" != x ]
then
	CFG_RELEASE="$FF_PRODUCT_VERSION"
fi

# use NONE so ff_function's inclusion of defaults works properly
if [ x"$FF_IPOIB_SUFFIX" = x ]
then
	export FF_IPOIB_SUFFIX="NONE"
	export CFG_IPOIB_SUFFIX="NONE"
fi

if [ "$clobber" = "y" ]
then
	( cd $TEST_RESULT_DIR; rm -rf test.res save_tmp test.log test_tmp* *.dmp )
fi

# create an empty test.log file
( cd $TEST_RESULT_DIR; >> test.log )

run_test()
{
	# $1 = test suite name
	TCLLIBPATH="$TL_DIR /usr/lib/tcl" expect -f $TL_DIR/$1.exp | tee -a $TEST_RESULT_DIR/test.res
}

if [ $chassis -eq 1 ]
then
	if [ "$Sopt" = y ]
	then
		read -sp "Password for admin on all chassis: " password
		echo
		export CFG_CHASSIS_ADMIN_PASSWORD="$password"
	fi
	for test_suite in $*
	do
		case $test_suite in
		reboot)
			run_test chassis_$test_suite;;
		configure)
			export CFG_CFGTEMPDIR="$(mktemp -d --tmpdir opacfgtmp.XXXXXX)"
			# Update traps to delete all temporary directories.
			trap "rm -rf $CFG_CFGTEMPDIR; exit 1" 1 2 3 9 15
			trap "rm -rf $CFG_CFGTEMPDIR" EXIT

			/usr/lib/opa/tools/chassis_setup $CFG_CFGTEMPDIR $CFG_CHASSIS
			if [ $? = 0 ]
			then
				export SYSLOG_SERVER=`grep "Syslog Server IP_Address" $CFG_CFGTEMPDIR/.chassisSetup.out | cut -d : -f 2`
				export SYSLOG_PORT=`grep "Syslog Port" $CFG_CFGTEMPDIR/.chassisSetup.out | cut -d : -f 2`
				export SYSLOG_FACILITY=`grep "Syslog Facility" $CFG_CFGTEMPDIR/.chassisSetup.out | cut -d : -f 2`
				export NTP_SERVER=`grep "NTP Server" $CFG_CFGTEMPDIR/.chassisSetup.out | cut -d : -f 2`
				export TZ_OFFSET=`grep "Timezone offset" $CFG_CFGTEMPDIR/.chassisSetup.out | cut -d : -f 2`
				export DST_START=`grep "Start DST" $CFG_CFGTEMPDIR/.chassisSetup.out | cut -d : -f 2`
				export DST_END=`grep "End DST" $CFG_CFGTEMPDIR/.chassisSetup.out | cut -d : -f 2`
				export LINKWIDTH_SETTING=`grep "Link Width Selection" $CFG_CFGTEMPDIR/.chassisSetup.out | cut -d : -f 2`
				export SET_NAME=`grep "Set OPA Node Desc" $CFG_CFGTEMPDIR/.chassisSetup.out | cut -d : -f 2`
				export LINKCRCMODE=`grep "Link CRC Mode" $CFG_CFGTEMPDIR/.chassisSetup.out | cut -d : -f 2`
				run_test chassis_$test_suite
			else
				echo "Chassis setup wizard exited abnormally ... aborting"
		fi;;
		getconfig)
			run_test chassis_$test_suite;;
		upgrade)
			if [ "$packages" = "notsupplied" -o "$packages" = "" ]
			then
				echo "$cmd: -P option required for chassis upgrade" >&2
				Usage
			fi
			if [ "$action" != "push" -a "$action" != "select" -a "$action" != "run" ]
			then
				echo "$cmd: Invalid firmware upgrade action: $action" >&2
				Usage
			fi
			# check fw files exist, expand directories
			CFG_FWFILES=""
			for fwfile in $packages
			do
				# expand directory, also filters files without .pkg/.dpkg/.spkg suffix
				# this also expands wildcards in "$packages"
				fwfiles=`find $fwfile -type f -name '*.pkg'`
				fwfiles="$fwfiles `find $fwfile -type f -name '*.[ds]pkg'`"
				if [ $? != 0 -o x"$fwfiles" == x -o x"$fwfiles" == x" " ]
				then
					echo "$cmd: $fwfile: No .pkg nor .dpkg nor .spkg files found" >&2
					Usage
				fi
				CFG_FWFILES="$CFG_FWFILES $fwfiles"
			done
			export CFG_FWFILES
			export CFG_FWACTION="$action"
			run_test chassis_$test_suite;;
		fmconfig)
			if [ "$packages" = "notsupplied" -o "$packages" = "" ]
			then
				echo "$cmd: -P option required for chassis fmconfig" >&2
				Usage
			fi
			if [ "$action" != "push" -a "$action" != "run" -a "$action" != "runall" ]
			then
				echo "$cmd: Invalid FM config action: $action" >&2
				Usage
			fi
			if [ "$CFG_INSTALL_OPTIONS" != "" -a "$CFG_INSTALL_OPTIONS" != "disable" -a "$CFG_INSTALL_OPTIONS" != "enable" -a "$CFG_INSTALL_OPTIONS" != "enableall" ]
			then
				echo "$cmd: Invalid FM bootstate: $CFG_INSTALL_OPTIONS" >&2
				Usage
			fi
			export CFG_FMFILE="$packages"
			export CFG_FWACTION="$action"
			run_test chassis_$test_suite;;
		fmcontrol)
			if [ "$action" != "stop" -a "$action" != "run" -a "$action" != "runall" -a "$action" != "restart" -a "$action" != "restartall" ]
			then
				echo "$cmd: Invalid FM config action: $action" >&2
				Usage
			fi
			if [ "$CFG_INSTALL_OPTIONS" != "" -a "$CFG_INSTALL_OPTIONS" != "disable" -a "$CFG_INSTALL_OPTIONS" != "enable" -a "$CFG_INSTALL_OPTIONS" != "enableall" ]
			then
				echo "$cmd: Invalid FM bootstate: $CFG_INSTALL_OPTIONS" >&2
			fi
			export CFG_FWACTION="$action"
			run_test chassis_$test_suite;;
		fmgetconfig)
			if [ "$packages" = "notsupplied" -o "$packages" = "" ]
			then
				packages="opafm.xml"
				#echo "$cmd: -P option required for chassis fmgetconfig" >&2
				#Usage
			fi
			export CFG_FMFILE="$packages"
			run_test chassis_$test_suite;;
		fmsecurityfiles)
			if [ "$securityFiles" = "notsupplied" -o "$securityFiles" = "" ]
			then
				echo "$cmd: -s option required for chassis fmsecurityfiles" >&2
				Usage
			fi
			if [ "$action" != "push" -a "$action" != "restart" -a "$action" != "restartall" ]
			then
				echo "$cmd: Invalid security files upgrade action: $action" >&2
				Usage
			fi
			# check security files exist, expand directories
			CFG_SECFILES=""
			for securityfile in $securityFiles
			do
				# expand directory, also filters files without .pem suffix
				# this also expands wildcards in "$securityFiles"
				securityfiles=`find $securityfile -type f -name '*.pem'`
				if [ $? != 0 -o x"$securityfiles" == x ]
				then
					echo "$cmd: $securityfile: No .pem files found" >&2
					Usage
				fi
				CFG_SECFILES="$CFG_SECFILES $securityfiles"
			done
			export CFG_SECFILES
			export CFG_SECACTION="$action"
			run_test chassis_$test_suite;;
		fmgetsecurityfiles)
			if [ "$securityFiles" = "notsupplied" -o "$securityFiles" = "" ]
			then
    			securityFiles="*.pem"
			fi
			export CFG_SECFILE="$securityFiles"
			run_test chassis_$test_suite;;
		*)
			echo "Invalid Operation name: $test_suite" >&2
			Usage;
			;;
		esac
	done
elif [ $opaswitch -eq 1 ]
then
	if [ "$bypassSwitchCheck" = y ]
	then
		export CFG_SWITCH_BYPASS_SWITCH_CHECK=y
	else
		export CFG_SWITCH_BYPASS_SWITCH_CHECK=n
	fi
	for test_suite in $*
	do

		case $test_suite in
      reboot)
         run_test switch_$test_suite;;
		info)
			run_test switch_$test_suite;;
		hwvpd)
			run_test switch_$test_suite;;
		ping)
			run_test switch_$test_suite;;
		fwverify)
			run_test switch_$test_suite;;
		configure)
			CFG_CFGTEMPDIR="$(mktemp -d --tmpdir opacfgtmp.XXXXXX)"
			# Update traps to delete all temporary directories.
			trap "rm -rf $CFG_CFGTEMPDIR; exit 1" 1 2 3 9 15
			trap "rm -rf $CFG_CFGTEMPDIR" EXIT
			/usr/lib/opa/tools/switch_setup $CFG_CFGTEMPDIR
			if [ $? = 0 ]
			then
				export LINKWIDTH_SETTING=`grep "Link Width Selection" $CFG_CFGTEMPDIR/.switchSetup.out | cut -d : -f 2`
				export NODEDESC_SETTING=`grep "Node Description Selection" $CFG_CFGTEMPDIR/.switchSetup.out | cut -d : -f 2`
				export FMENABLED_SETTING=`grep "FM Enabled Selection" $CFG_CFGTEMPDIR/.switchSetup.out | cut -d : -f 2`
				export LINKCRCMODE_SETTING=`grep "Link CRC Mode Selection" $CFG_CFGTEMPDIR/.switchSetup.out | cut -d : -f 2`
				run_test switch_$test_suite
			else
				echo "Ext mgd switch setup wizard exited abnormally ... aborting"
			fi;;
		upgrade)
			if [ "$packages" = "notsupplied" -o "$packages" = "" ]
			then
				echo "$cmd: -P option required for switch upgrade" >&2
				Usage
			fi
			if [ "$action" != "select" -a "$action" != "run" ]
			then
				echo "$cmd: Invalid firmware upgrade action: $action" >&2
				echo "$cmd: 'run' and 'select' are the only supported actions" >&2
				Usage
			fi

			# check fw files exist, expand directories
			CFG_FWFILES=""
			CFG_FWBINFILES=""

			CFG_TMPDIR_LIST=""
			for fwfile in $packages
			do

				echo "$cmd: processing package file: $fwfile" >&2
				# expand directory, also filters files without .emfw suffix
				# this also expands wildcards in "$packages"
				fwfiles=`find $fwfile -type f -name '*.emfw'`
				if [ $? != 0 -o x"$fwfiles" == x ]
				then
					echo "$cmd: $fwfile: No .emfw files found" >&2
					Usage
				fi
				CFG_FWFILES="$fwfiles"

				echo "$cmd: found package file: $fwfiles" >&2
				# copy file to temporary directory
			for tarball in $CFG_FWFILES
			do
				# create temporary work directory
				CFG_FWFILE="$tarball"
				CFG_FWTEMPDIR="$(mktemp -d --tmpdir opafwtmp.XXXXXX)"

				# Update traps to delete all temporary directories.
				CFG_TMPDIR_LIST="$CFG_TMPDIR_LIST $CFG_FWTEMPDIR"
				trap "rm -rf $CFG_TMPDIR_LIST; exit 1" 1 2 3 9 15
				trap "rm -rf $CFG_TMPDIR_LIST" EXIT

				CFG_FWRELEASEFILE="$CFG_FWTEMPDIR/release.emfw.txt"

				cp -f $tarball $CFG_FWTEMPDIR

				# remove previous firmware image .bin files, and extract
				# .bin files from .emfw file.
				rm -rf '$CFG_FWTEMPDIR/*.bin'
				tar --directory $CFG_FWTEMPDIR -zxf $tarball

				# search for text file that contains release related information
            # about the firmware image .bin files
            if [ ! -f "$CFG_FWRELEASEFILE" ]
				then
					echo "$cmd: No release.emfw.txt file found for package file: $tarball" >&2
					Usage
				fi

            fwreleaseinfo=`cat $CFG_FWRELEASEFILE`
            CFG_FWRELINFO="$fwreleaseinfo"
            fwreleaseversioninfo=`cat $CFG_FWRELEASEFILE | grep _ | sed "s/_/./g"`
            CFG_FWRELEASEVERSIONINFO=$fwreleaseversioninfo
            CFG_FWSPEED=`cat $CFG_FWRELEASEFILE | grep DR`
			if [ "$CFG_FWSPEED" = "" ]
			then
				CFG_FWSPEED="QDR"
			fi
            CFG_FWASICVER=`grep "V[0-9]" $CFG_FWRELEASEFILE`
			if [ "$CFG_FWASICVER" = "" ]
			then
				CFG_FWASICVER="V0"
			fi
			CFG_SWITCH_DEVICE=`head -n 1 $CFG_FWRELEASEFILE`

				# expand directory, also filters files without .bin suffix
				fwfiles=`find $CFG_FWTEMPDIR -type f -name '*.bin'`
				if [ $? != 0 -o x"$fwfiles" == x ]
				then
					echo "$cmd: $tarball: No .bin files found" >&2
					Usage
				fi

				CFG_FWBINFILES="$fwfiles"
				CFG_FWOVERRIDE=$fwOverride

				export CFG_FWFILES
				export CFG_FWFILE
				export CFG_FWBINFILES
				export CFG_FWTEMPDIR
				export CFG_FWRELINFO
				export CFG_FWSPEED
				export CFG_FWASICVER
				export CFG_SWITCH_DEVICE
				export CFG_FWRELEASEVERSIONINFO
				export CFG_FWACTION="$action"
				export CFG_FWOVERRIDE

				echo "$cmd: upgrading with switch firmware image: $tarball : version $fwreleaseversioninfo" >&2
				run_test switch_$test_suite

			done
			done
			;;
		getconfig)
			run_test switch_$test_suite;;
		*)
			echo "Invalid Operation name: $test_suite" >&2
			Usage;
			;;
		esac
	done
else
	if [ "$Sopt" = y ]
	then
		read -sp "Password for $CFG_USERNAME on all hosts: " password
		echo
		export CFG_PASSWORD="$password"
		if [ "$CFG_USERNAME" != "root" ]
		then
			read -sp "Password for root on all hosts: " password
			echo
			export CFG_ROOTPASS="$password"
		else
			export CFG_ROOTPASS="$CFG_PASSWORD"
		fi
	fi
	for test_suite in $*
	do
		case $test_suite in
		load|upgrade)
			if [ ! -f "$dir/$CFG_PRODUCT.$CFG_RELEASE.tgz" ]
			then
				echo "$cmd: $dir/$CFG_PRODUCT.$CFG_RELEASE.tgz not found" >&2
				exit 1
			fi
			run_test $test_suite;;
		reboot|sacache|configipoib|ipoibping|mpiperf|mpiperfdeviation)
			run_test $test_suite;;
		*)
			echo "Invalid Operation name: $test_suite" >&2
			Usage;
			;;
		esac
	done
fi
