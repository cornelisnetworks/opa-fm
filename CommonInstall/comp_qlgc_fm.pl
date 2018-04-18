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
# OPA FM for OFED installation

# autostart functions are per subcomponent
# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_opafm()
{
	return IsAutostart("opafm");
}
sub autostart_desc_opafm()
{
	return "$ComponentInfo{'opafm'}{'Name'} (opafm)";
}
# enable autostart for the given capability
sub enable_autostart2_opafm()
{
	enable_autostart("opafm");
}
# disable autostart for the given capability
sub disable_autostart2_opafm()
{
	disable_autostart("opafm");
}

sub available_opafm
{
	my $srcdir=$ComponentInfo{'opafm'}{'SrcDir'};
	return (rpm_resolve("$srcdir/RPMS/*/", "any", "opa-fm") ne "");
}

sub installed_opafm
{
	my $driver_subdir=$ComponentInfo{'opafm'}{'DriverSubdir'};
	return rpm_is_installed("opa-fm", "any");
}

# only called if installed_opafm is true
sub installed_version_opafm
{
	my $version = rpm_query_version_release_pkg("opa-fm");
	return dot_version("$version");
}

# only called if available_opafm is true
sub media_version_opafm
{
	my $srcdir=$ComponentInfo{'opafm'}{'SrcDir'};
	my $rpmfile = rpm_resolve("$srcdir/RPMS/*/", "any", "opa-fm");
	my $version= rpm_query_version_release("$rpmfile");
	# assume media properly built with matching versions
	return dot_version("$version");
}

sub build_opafm
{
	my $osver = $_[0];
	my $debug = $_[1];	# enable extra debug of build itself
	my $build_temp = $_[2];	# temp area for use by build
	my $force = $_[3];	# force a rebuild
	my $srcdir=$ComponentInfo{'opafm'}{'SrcDir'};

	return 0;	# success
}

sub need_reinstall_opafm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return "no";
}

sub preinstall_opafm
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled
	my $srcdir=$ComponentInfo{'opafm'}{'SrcDir'};

	return 0;	# success
}

sub install_opafm
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled

	my $srcdir=$ComponentInfo{'opafm'}{'SrcDir'};
	my $driver_subdir=$ComponentInfo{'opafm'}{'DriverSubdir'};
	my $rc;
	my $keep;
	my $diff_src_dest;

	my $version=media_version_opafm();
	chomp $version;

	printf("Installing $ComponentInfo{'opafm'}{'Name'} $version $DBG_FREE...\n");
	LogPrint "Installing $ComponentInfo{'opafm'}{'Name'} $version $DBG_FREE for $CUR_OS_VER\n";

	# because RPM will change autostart settings of opafm we need to save
	# and restore the settings
	my $fm_start = IsAutostart2_opafm();

	# Install the rpm
	my $rpmfile = rpm_resolve("$srcdir/RPMS/*/", "user", "opa-fm");
	rpm_run_install($rpmfile, "user", "-U");
	$rpmfile = rpm_resolve("$srcdir/RPMS/*/", "user", "opa-fm-debuginfo");
	if ($rpmfile) {
		rpm_run_install($rpmfile, "user", " -U ");
	}

	check_rpm_config_file("$CONFIG_DIR/opa-fm/opafm.xml", "/etc/sysconfig");
	check_dir("/usr/lib/opa");

	if ($fm_start) {
		enable_autostart("opafm");
	} else {
		disable_autostart("opafm");
	}

	$ComponentWasInstalled{'opafm'}=1;
}

sub postinstall_opafm
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled
}

sub uninstall_opafm
{
	my $install_list = $_[0];	# total that will be left installed when done
	my $uninstalling_list = $_[1];	# what items are being uninstalled

	my $driver_subdir=$ComponentInfo{'opafm'}{'DriverSubdir'};
	NormalPrint("Uninstalling $ComponentInfo{'opafm'}{'Name'}...\n");

	rpm_uninstall_list("any", "verbose", ( "opa-fm", "opa-fm-debuginfo") );
	system("rm -rf $ROOT/usr/lib/opa/.comp_opafm.pl");
	system("rmdir -p $ROOT/opt/iba/fm_tools 2>/dev/null");  # remove only if empty
	system("rm -rf $ROOT/usr/lib/opa-fm");
	$ComponentWasInstalled{'opafm'}=0;
}

sub check_os_prereqs_opafm
{
	return rpm_check_os_prereqs("opafm", "user");
}

