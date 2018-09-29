#!/usr/bin/perl
## BEGIN_ICS_COPYRIGHT8 ****************************************
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

#

#============================================================================
# initialization
use strict;
use Term::ANSIColor;
use Term::ANSIColor qw(:constants);
use File::Basename;
use Math::BigInt;
use Cwd;

#Setup some defaults
my $exit_code=0;

my $Force_Install = 0;# force option used to force install on unsupported distro
my $GPU_Install = 0;
my $HFI2_INSTALL = 0; # indicate whether we shall install HFI2
# When --user-space is selected we are targeting a user space container for
# installation and will skip kernel modules and firmware
my $user_space_only = 0; # can be set to 1 by --user-space argument

# some options specific to OFA builds
my $OFED_force_rebuild=0;
my $OFED_prefix="/usr";

my $CUR_OS_VER = `uname -r`;
chomp $CUR_OS_VER;

# related to HFI2_INSTALL. The following code is introduced to help us check b-s-m kernel for HFI2.
# After we merge HFI2 into ifs-kernel-updates, we shall remove the following.
my $CUR_OS_VER_SHORT = '';
if ($CUR_OS_VER =~ /([0-9]+\.[0-9]+)/ ) {
	$CUR_OS_VER_SHORT=$1;
} else {
	die "\n\nKernel version \"${CUR_OS_VER}\" doesn't have an extractable major/minor version!\n\n";
}

my $RPMS_SUBDIR = "RPMS";
my $SRPMS_SUBDIR = "SRPMS";

# firmware and data files
my $OLD_BASE_DIR = "/etc/sysconfig/opa";
my $BASE_DIR = "/etc/opa";
# iba editable config scripts
my $OPA_CONFIG_DIR = "/etc/opa";

my $UVP_CONF_FILE = "$BASE_DIR/uvp.conf";
my $UVP_CONF_FILE_SOURCE = "uvp.conf";
my $DAT_CONF_FILE_SOURCE = "dat.conf";
my $NETWORK_CONF_DIR = "/etc/sysconfig/network-scripts";
my $ROOT = "/";	# TBD prepared to make this "" removes some // prompts and logs

my $BIN_DIR = "/usr/sbin";

#This string is compared in verify_os_rev for correct revision of
#kernel release.
my $CUR_DISTRO_VENDOR = "";
my $CUR_VENDOR_VER = "";	# full version (such as ES5.1)
my $CUR_VENDOR_MAJOR_VER = "";    # just major number part (such as ES5)
my $ARCH = `uname -m | sed -e s/ppc/PPC/ -e s/powerpc/PPC/ -e s/i.86/IA32/ -e s/ia64/IA64/ -e s/x86_64/X86_64/`;
chomp $ARCH;
my $DRIVER_SUFFIX=".o";
if (substr($CUR_OS_VER,0,3) eq "2.6" || substr($CUR_OS_VER,0,2) eq "3.")
{
	$DRIVER_SUFFIX=".ko";
}
my $DBG_FREE="release";


# Command paths
my $RPM = "/bin/rpm";

# a few key commands to verify exist
my @verify_cmds = ( "uname", "mv", "cp", "rm", "ln", "cmp", "yes", "echo", "sed", "chmod", "chown", "chgrp", "mkdir", "rmdir", "grep", "diff", "awk", "find", "xargs", "sort", "chroot");

# opa-scripts expects the following env vars to be 0 or 1. We set them to the default value here
setup_env("OPA_INSTALL_CALLER", 0);
default_opascripts_env_vars();

sub Abort(@);
sub NormalPrint(@);
sub LogPrint(@);
sub HitKeyCont();
# ============================================================================
# General utility functions

# verify the given command can be found in the PATH
sub check_cmd_exists($)
{
	my $cmd=shift();
	return (0 == system("which $cmd >/dev/null 2>/dev/null"));
}

sub check_root_user()
{
	my $user;
	$user=`/usr/bin/id -u`;
	if ($user != 0)
	{
		die "\n\nYou must be \"root\" to run this install program\n\n";
	}

	@verify_cmds = (@verify_cmds, rpm_get_cmds_for_verification());
	# verify basic commands are in path
	foreach my $cmd ( @verify_cmds ) {
		if (! check_cmd_exists($cmd)) {
			die "\n\n$cmd not found in PATH.\nIt is required to login as root or su - root.\nMake sure the path includes /sbin and /usr/sbin\n\n";
		}
	}
}

sub my_tolower($)
{
	my($str) = shift();

	$str =~ tr/[A-Z]/[a-z]/;
	return "$str";
}

sub ROOT_is_set()
{
	return ("$ROOT" ne "/" && "$ROOT" ne "");
}

# ============================================================================
# Version and branding

# version string is filled in by prep, special marker format for it to use
my $VERSION = "THIS_IS_THE_ICS_VERSION_NUMBER:@(#)000.000.000.000B000";
$VERSION =~ s/THIS_IS_THE_ICS_VERSION_NUMBER:@\(#\)//;
$VERSION =~ s/%.*//;
my $INT_VERSION = "THIS_IS_THE_ICS_INTERNAL_VERSION_NUMBER:@(#)000.000.000.000B000";
$INT_VERSION =~ s/THIS_IS_THE_ICS_INTERNAL_VERSION_NUMBER:@\(#\)//;
$INT_VERSION =~ s/%.*//;
my $BRAND = "THIS_IS_THE_ICS_BRAND:Intel%                    ";
# backslash before : is so patch_brand doesn't replace string
$BRAND =~ s/THIS_IS_THE_ICS_BRAND\://;
$BRAND =~ s/%.*//;

# convert _ and - in version to dots
sub dot_version($)
{
	my $version = shift();

	$version =~ tr/_-/./;
	return $version;
}

# ============================================================================
# installation paths

# where to install libraries
my $LIB_DIR = "/lib";
my $UVP_LIB_DIR = "/lib";
my $USRLOCALLIB_DIR = "/usr/local/lib";
my $OPTIBALIB_DIR = "/usr/lib/opa/lib";
my $USRLIB_DIR = "/usr/lib";
# if different from $LIB_DIR, where to remove libraries from past release
my $OLD_LIB_DIR = "/lib";
my $OLD_USRLOCALLIB_DIR = "/usr/local/lib";
my $OLD_UVP_LIB_DIR = "/lib";

sub set_libdir()
{
	if ( -d "$ROOT/lib64" )
	{
		$LIB_DIR = "/lib64";
		$UVP_LIB_DIR = "/lib64";
		$UVP_CONF_FILE_SOURCE = "uvp.conf.64";
		$DAT_CONF_FILE_SOURCE = "dat.conf.64";
		$USRLOCALLIB_DIR = "/usr/local/lib64";
		$OPTIBALIB_DIR = "/usr/lib/opa/lib64";
		$USRLIB_DIR = "/usr/lib64";
	}
}

# determine the os vendor release level based on build system
# this script is stolen from funcs-ext.sh and should
# be maintained in parallel
sub os_vendor_version($)
{
	my $vendor = shift();

	my $rval = "";
	my $mn = "";
	if ( -e "/etc/os-release" ) {
		if ($vendor eq "ubuntu") {
			$rval=`cat /etc/os-release | grep VERSION_ID | cut -d'=' -f2 | tr -d [\\"\\.]`;
		} else {
			$rval=`cat /etc/os-release | grep VERSION_ID | cut -d'=' -f2 | tr -d [\\"\\.0]`;
		}
		chop($rval);
		$rval="ES".$rval;
		if ( -e "/etc/redhat-release" ) {
			if (!system("grep -qi centos /etc/redhat-release")) {
				$rval = `cat /etc/redhat-release | cut -d' ' -f4`;
				$rval =~ m/(\d+).(\d+)/;
				$rval="ES".$1.$2;
			}	
		}
	} elsif ($vendor eq "apple") {
		$rval=`sw_vers -productVersion|cut -f1-2 -d.`;
		chop($rval);
	} elsif ($vendor eq "rocks") {
		$rval=`cat /etc/rocks-release | cut -d' ' -f3`;
		chop($rval);
	} elsif ($vendor eq "scyld") {
		$rval=`cat /etc/scyld-release | cut -d' ' -f4`;
		chop($rval);
	} elsif ($vendor eq "mandrake") {
		$rval=`cat /etc/mandrake-release | cut -d' ' -f4`;
		chop($rval);
	} elsif ($vendor eq "fedora") {
		$rval=`cat /etc/fedora-release | cut -d' ' -f4`;
		chop($rval);
	} elsif ($vendor eq "redhat") {
		if (!system("grep -qi advanced /etc/redhat-release")) {
			$rval=`cat /etc/redhat-release | cut -d' ' -f7`;
			chop($rval);
		} elsif (!system("grep -qi centos /etc/redhat-release")) {
			# Find a number of the form "#.#" and output the portion
			# to the left of the decimal point.
			$rval = `cat /etc/redhat-release`;
			$rval =~ m/(\d+).(\d+)/;
			$rval="ES".$1.$2;
		} elsif (!system("grep -qi Scientific /etc/redhat-release")) {
			# Find a number of the form "#.#" and output the portion
			# to the left of the decimal point.
			$rval = `cat /etc/redhat-release`;
			$rval =~ m/(\d+).(\d+)/;
			$rval="ES".$1;
		} elsif (!system("grep -qi enterprise /etc/redhat-release")) {
			# Red Hat Enterprise Linux Server release $a.$b (name)
			#PR 110926
			$rval=`cat /etc/redhat-release | cut -d' ' -f7 | cut -d'.' -f1`;
			$mn=`cat /etc/redhat-release | cut -d' ' -f7 | cut -d'.' -f2`;
			chop($rval);
			if ( (($rval >= 7) && ($mn > 0)) ||
				(($rval == 6) && ($mn >= 7)) ) {
				chomp($mn);
				$rval=join "","ES","$rval","$mn";
			} else {
				$rval="ES".$rval;
			}
		} else {
			$rval=`cat /etc/redhat-release | cut -d' ' -f5`;
			chop($rval);
		}
	} elsif ($vendor eq "UnitedLinux") {
		$rval=`grep United /etc/UnitedLinux-release | cut -d' ' -f2`;
		chop($rval);
	} elsif ($vendor eq "SuSE") {
		if (!system("grep -qi enterprise /etc/SuSE-release")) {
			$rval=`grep -i enterprise /etc/SuSE-release | cut -d' ' -f5`;
			chop($rval);
			$rval="ES".$rval;
		} else {
			$rval=`grep SuSE /etc/SuSE-release | cut -d' ' -f3`;
			chop($rval);
		}
	} elsif ($vendor eq "turbolinux") {
		$rval=`cat /etc/turbolinux-release | cut -d' ' -f3`;
		chop($rval);
	}
	return $rval;
}

# Get OS distribution, vendor and vendor version
# set CUR_DISTRO_VENDOR, CUR_VENDOR_VER, CUR_VENDOR_MAJOR_VER
sub determine_os_version()
{
	# we use the current system to select the distribution
	# TBD we expect client image for diskless client install to have these files
	my $os_release_file = "/etc/os-release";
	if ( -e "/etc/redhat-release" && !(-l "/etc/redhat-release") ) {
		$CUR_DISTRO_VENDOR = "redhat";
	} elsif ( -s "/etc/centos-release" ) {
		$CUR_DISTRO_VENDOR = "redhat";
	} elsif ( -s "/etc/UnitedLinux-release" ) {
		$CUR_DISTRO_VENDOR = "UnitedLinux";
		$NETWORK_CONF_DIR = "/etc/sysconfig/network";
	} elsif ( -s "/etc/SuSE-release" ) {
		$CUR_DISTRO_VENDOR = "SuSE";
		$NETWORK_CONF_DIR = "/etc/sysconfig/network";
	} elsif ( -e "/usr/bin/lsb_release" ) {
		$CUR_DISTRO_VENDOR = `/usr/bin/lsb_release -is`;
		chop($CUR_DISTRO_VENDOR);
		$CUR_DISTRO_VENDOR = lc($CUR_DISTRO_VENDOR);
		if ($CUR_DISTRO_VENDOR eq "suse") {
			$CUR_DISTRO_VENDOR = "SuSE";
		}
	} elsif ( -e $os_release_file) {
		my %distroVendor = (
			"rhel" => "redhat",
			"centos" => "redhat",
			"sles" => "SuSE"
		);
		my %network_conf_dir  = (
			"rhel" => $NETWORK_CONF_DIR,
			"centos" => $NETWORK_CONF_DIR,
			"sles" => "/etc/sysconfig/network"
		);
		my $os_id = `cat $os_release_file | grep '^ID=' | cut -d'=' -f2 | tr -d [\\"\\.0] | tr -d ["\n"]`;
		$CUR_DISTRO_VENDOR = $distroVendor{$os_id};
		$NETWORK_CONF_DIR = $network_conf_dir{$os_id};
		print "Current Distro: $CUR_DISTRO_VENDOR\n";
	} else {
		# autodetermine the distribution
		open DISTRO_VENDOR, "ls /etc/*-release|grep -v lsb\|^os 2>/dev/null |"
			|| die "Unable to open pipe\n";
		$CUR_DISTRO_VENDOR="";
		while (<DISTRO_VENDOR>) {
			chop;
			if (!(-l $_)) {
				my $CDV  = fileparse($_, '-release');
				if ( "$CDV" ne "rocks" ) {
					$CUR_DISTRO_VENDOR = $CDV;
				}
			}
		}
		close DISTRO_VENDOR;
		if ( $CUR_DISTRO_VENDOR eq "" )
		{
			NormalPrint "Unable to determine current Linux distribution.\n";
			Abort "Please contact your support representative...\n";
		} elsif ($CUR_DISTRO_VENDOR eq "SuSE") {
			$NETWORK_CONF_DIR = "/etc/sysconfig/network";
		} elsif ($CUR_DISTRO_VENDOR eq "centos") {
			$CUR_DISTRO_VENDOR = "redhat";
		}
	}
	$CUR_VENDOR_VER = os_vendor_version($CUR_DISTRO_VENDOR);
	$CUR_VENDOR_MAJOR_VER = $CUR_VENDOR_VER;
	$CUR_VENDOR_MAJOR_VER =~ s/\..*//;	# remove any . version suffix
}

# verify distrib of this system matches files indicating supported
#  arch, distro, distro_version
sub verify_distrib_files
{
	my $supported_arch=`cat ./arch 2>/dev/null`;
	chomp($supported_arch);
	my $supported_distro_vendor=`cat ./distro 2>/dev/null`;
	chomp($supported_distro_vendor);
	my $supported_distro_vendor_ver=`cat ./distro_version 2>/dev/null`;
	chomp($supported_distro_vendor_ver);

	if ( "$supported_arch" eq "" || $supported_distro_vendor eq ""
		|| $supported_distro_vendor_ver eq "") {
		NormalPrint "Unable to proceed: installation image corrupted or install not run as ./INSTALL\n";
		NormalPrint "INSTALL must be run from within untar'ed install image directory\n";
		Abort "Please contact your support representative...\n";
	}

	my $archname;
	my $supported_archname;
	if ( "$supported_arch" ne "$ARCH"
		|| "$supported_distro_vendor" ne "$CUR_DISTRO_VENDOR"
		|| ("$supported_distro_vendor_ver" ne "$CUR_VENDOR_VER"
			&& "$supported_distro_vendor_ver" ne "$CUR_VENDOR_MAJOR_VER"))
	{
		#LogPrint "Unable to proceed, $CUR_DISTRO_VENDOR $CUR_VENDOR_VER not supported by $INT_VERSION media\n";

		$archname=$ARCH;
		if ( $ARCH eq "IA32") {
			$archname="the Pentium Family";
		} 
		if ( $ARCH eq "IA64" ) {
			$archname="the Itanium family";
		} 
		if ( $ARCH eq "X86_64" ) {
			$archname="the EM64T or Opteron";
		} 
		if ( $ARCH eq "PPC64" ) {
			$archname="the PowerPC 64 bit";
		} 
		if ( $ARCH eq "PPC" ) {
			$archname="the PowerPC";
		} 
		
		NormalPrint "$CUR_DISTRO_VENDOR $CUR_VENDOR_VER for $archname is not supported by this installation\n";
		NormalPrint "This installation supports the following Linux Distributions:\n";
		$supported_archname=$ARCH;
		if ( $supported_arch eq "IA32") {
			$supported_archname="the Pentium Family";
		} 
		if ( $supported_arch eq "IA64" ) {
			$supported_archname="the Itanium family";
		} 
		if ( $supported_arch eq "X86_64" ) {
			$supported_archname="the EM64T or Opteron";
		} 
		if ( $supported_arch eq "PPC64" ) {
			$supported_archname="the PowerPC 64 bit";
		} 
		if ( $supported_arch eq "PPC" ) {
			$supported_archname="the PowerPC";
		} 
		NormalPrint "For $supported_archname: $supported_distro_vendor.$supported_distro_vendor_ver\n";
		if ( $Force_Install ) {
			NormalPrint "Installation Forced, will proceed with risk of undefined results\n";
			HitKeyCont;
		} else {
			Abort "Please contact your support representative...\n";
		}
	}
}

# set the env vars to their default value, when the first we install opa-scripts, we will have proper configs
sub default_opascripts_env_vars()
{
	setup_env("OPA_UDEV_RULES", 1);
	setup_env("OPA_LIMITS_CONF", 1);
	setup_env("OPA_ARPTABLE_TUNING", 1);
	setup_env("OPA_SRP_LOAD", 0);
	setup_env("OPA_SRPT_LOAD", 0);
	setup_env("OPA_IRQBALANCE", 1);
}

# set the env vars for ALL opa-scripts system env variables. Use this subroutine with caution because it will set ALL
# the env vars. The typical use case for the routine is seting all vars to -1, so opa-scripts will skip config on them.
# This is useful when we intend to keep previous conf.
sub set_opascript_env_vars($)
{
    my $env_value = shift();
	setup_env("OPA_UDEV_RULES", $env_value);
	setup_env("OPA_LIMITS_CONF", $env_value);
	setup_env("OPA_ARPTABLE_TUNING", $env_value);
	setup_env("OPA_SRP_LOAD", $env_value);
	setup_env("OPA_SRPT_LOAD", $env_value);
	setup_env("OPA_IRQBALANCE", $env_value);
}

# this will be replaced in component specific INSTALL with any special
# overrides of things in main*pl
sub overrides()
{
}
