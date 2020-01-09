#!/usr/bin/perl
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
use strict;
#use Term::ANSIColor;
#use Term::ANSIColor qw(:constants);
#use File::Basename;
#use Math::BigInt;

# ==========================================================================
# Fast Fabric installation

my $FF_CONF_FILE = "/usr/lib/opa/tools/opafastfabric.conf";
my $FF_TLS_CONF_FILE = "/etc/opa/opaff.xml";

sub get_rpms_dir_fastfabric
{
	my $srcdir=$ComponentInfo{'fastfabric'}{'SrcDir'};
	return "$srcdir/RPMS/*";
}

sub available_fastfabric
{
	my $srcdir=$ComponentInfo{'fastfabric'}{'SrcDir'};
	return ((rpm_resolve("$srcdir/RPMS/*/opa-mpi-apps", "any") ne "") &&
			(rpm_resolve("$srcdir/RPMS/*/opa-fastfabric", "any") ne ""));
}

sub installed_fastfabric
{
	return rpm_is_installed("opa-fastfabric", "any");
}

# only called if installed_fastfabric is true
sub installed_version_fastfabric
{
	my $version = rpm_query_version_release_pkg("opa-fastfabric");
	return dot_version("$version");
}

# only called if available_fastfabric is true
sub media_version_fastfabric
{
	my $srcdir=$ComponentInfo{'fastfabric'}{'SrcDir'};
	my $rpmfile = rpm_resolve("$srcdir/RPMS/*/opa-fastfabric", "any");
	my $version= rpm_query_version_release("$rpmfile");
	# assume media properly built with matching versions for all rpms
	return dot_version("$version");
}

sub build_fastfabric
{
	my $osver = $_[0];
	my $debug = $_[1];	# enable extra debug of build itself
	my $build_temp = $_[2];	# temp area for use by build
	my $force = $_[3];	# force a rebuild
	return 0;	# success
}

sub need_reinstall_fastfabric($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return "no";
}

sub check_os_prereqs_fastfabric
{	
	return rpm_check_os_prereqs("fastfabric", "user");
}

sub preinstall_fastfabric
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled

	return 0;	# success
}

sub install_fastfabric
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled

	my $depricated_dir = "/etc/sysconfig/opa";

	my $version=media_version_fastfabric();
	chomp $version;
	printf("Installing $ComponentInfo{'fastfabric'}{'Name'} $version $DBG_FREE...\n");
	LogPrint "Installing $ComponentInfo{'fastfabric'}{'Name'} $version $DBG_FREE for $CUR_DISTRO_VENDOR $CUR_VENDOR_VER\n";

	install_comp_rpms('fastfabric', " -U ", $install_list);

	# TBD - spec file should do this
	check_dir("/usr/share/opa/samples");
	system "chmod ug+x /usr/share/opa/samples/hostverify.sh";
	system "rm -f /usr/share/opa/samples/nodeverify.sh";

	check_rpm_config_file("$FF_TLS_CONF_FILE");
	printf("Default opaff.xml can be found in '/usr/share/opa/samples/opaff.xml-sample'\n");
	check_rpm_config_file("$CONFIG_DIR/opa/opamon.conf", $depricated_dir);
	check_rpm_config_file("$CONFIG_DIR/opa/opafastfabric.conf", $depricated_dir);
	check_rpm_config_file("$CONFIG_DIR/opa/allhosts", $depricated_dir);
	check_rpm_config_file("$CONFIG_DIR/opa/chassis", $depricated_dir);
	check_rpm_config_file("$CONFIG_DIR/opa/hosts", $depricated_dir);
	check_rpm_config_file("$CONFIG_DIR/opa/ports", $depricated_dir);
	check_rpm_config_file("$CONFIG_DIR/opa/switches", $depricated_dir);
# TBD - this should not be a config file
	check_rpm_config_file("/usr/lib/opa/tools/osid_wrapper");

	# TBD - spec file should remove this
	system("rm -rf $OPA_CONFIG_DIR/iba_stat.conf");	# old config

	$ComponentWasInstalled{'fastfabric'}=1;
}

sub postinstall_fastfabric
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled
}

sub uninstall_fastfabric
{
	my $install_list = $_[0];	# total that will be left installed when done
	my $uninstalling_list = $_[1];	# what items are being uninstalled

	uninstall_comp_rpms('fastfabric', '', $install_list, $uninstalling_list, 'verbose');

	NormalPrint("Uninstalling $ComponentInfo{'fastfabric'}{'Name'}...\n");
	remove_conf_file("$ComponentInfo{'fastfabric'}{'Name'}", "$FF_CONF_FILE");
	remove_conf_file("$ComponentInfo{'fastfabric'}{'Name'}", "$OPA_CONFIG_DIR/iba_stat.conf");
	remove_conf_file("$ComponentInfo{'fastfabric'}{'Name'}", "$FF_TLS_CONF_FILE");
	
	# remove samples we installed (or user compiled), however do not remove
	# any logs or other files the user may have created
	remove_installed_files "/usr/share/opa/samples";
	system "rmdir /usr/share/opa/samples 2>/dev/null";	# remove only if empty
	# just in case, newer rpms should clean these up

	system("rm -rf /usr/lib/opa/.comp_fastfabric.pl");
	system "rmdir /usr/lib/opa 2>/dev/null";	# remove only if empty
	system "rmdir $BASE_DIR 2>/dev/null";	# remove only if empty
	system "rmdir $OPA_CONFIG_DIR 2>/dev/null";	# remove only if empty
	$ComponentWasInstalled{'fastfabric'}=0;
}

#############################################################################
##
##    OPAMGT SDK

sub get_rpms_dir_opamgt_sdk
{
	my $srcdir=$ComponentInfo{'opamgt_sdk'}{'SrcDir'};
	return "$srcdir/RPMS/*";
}

sub available_opamgt_sdk
{
	my $srcdir = $ComponentInfo{'opamgt_sdk'}{'SrcDir'};
	return ( rpm_exists("$srcdir/RPMS/*/opa-libopamgt-devel", "any") &&
		     rpm_exists("$srcdir/RPMS/*/opa-libopamgt", "any"));
}

sub installed_opamgt_sdk
{
	return ( rpm_is_installed("opa-libopamgt-devel", "any") &&
		     rpm_is_installed("opa-libopamgt", "any"));
}

sub installed_version_opamgt_sdk
{
	my $version = rpm_query_version_release_pkg("opa-libopamgt-devel");
	return dot_version("$version");
}

sub media_version_opamgt_sdk
{
	my $srcdir = $ComponentInfo{'opamgt_sdk'}{'SrcDir'};
	my $rpm = rpm_resolve("$srcdir/RPMS/*/opa-libopamgt-devel", "any");
	my $version = rpm_query_version_release($rpm);
	return dot_version("$version");
}

sub build_opamgt_sdk
{
	return 0;
}

sub need_reinstall_opamgt_sdk
{
	return "no";
}

sub check_os_prereqs_opamgt_sdk
{
	return rpm_check_os_prereqs("opamgt_sdk", "user");
}

sub preinstall_opamgt_sdk
{
	return 0;
}

sub install_opamgt_sdk
{
	my $install_list = $_[0];       # total that will be installed when done
	my $installing_list = $_[1];    # what items are being installed/reinstalled

	my $version=media_version_opamgt_sdk();
	chomp $version;
	printf("Installing $ComponentInfo{'opamgt_sdk'}{'Name'} $version $DBG_FREE...\n");
	LogPrint "Installing $ComponentInfo{'opamgt_sdk'}{'Name'} $version $DBG_FREE for $CUR_DISTRO_VENDOR $CUR_VENDOR_VER\n";

	install_comp_rpms('opamgt_sdk', "", $install_list);

	$ComponentWasInstalled{'opamgt_sdk'}=1;
}

sub postinstall_opamgt_sdk
{

}

sub uninstall_opamgt_sdk
{
	my $install_list = $_[0];	# total that will be left installed when done
	my $uninstalling_list = $_[1];	# what items are being uninstalled

	uninstall_comp_rpms('opamgt_sdk', '', $install_list, $uninstalling_list, 'verbose');
	$ComponentWasInstalled{'opamgt_sdk'}=0;
}
