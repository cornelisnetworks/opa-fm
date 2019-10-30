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

# =============================================================================
# The functions and constants below assist in editing modules.conf or
# modprobe.conf to add IB specific entries

my $MODULE_CONF_FILE = "/etc/modules.conf";
my $MODULE_CONF_DIST_FILE;
my $DRACUT_EXE_FILE = "/usr/bin/dracut";
if (substr($CUR_OS_VER,0,3) eq "2.6")
{
	$MODULE_CONF_FILE = "/etc/modprobe.conf";
	$MODULE_CONF_DIST_FILE = "/etc/modprobe.conf.dist";
	$DRACUT_EXE_FILE = "/sbin/dracut";
}
if (-f "$(MODULE_CONF_FILE).local")
{
	$MODULE_CONF_FILE = "$(MODULE_CONF_FILE).local";
}
my $OPA_CONF="modules.conf";	## additions to modules.conf

# marker strings used in MODULES_CONF_FILE
# for entries added by installation
my $START_DRIVER_MARKER="OPA Drivers Start here";
my $END_DRIVER_MARKER="OPA Drivers End here";

# Keep track of whether we already did edits to avoid repeated edits
my $DidConfig=0;

# This code is from OFED, it removes lines related to IB from
# modprobe.conf.  Used to prevent distro specific effects on OFED.
sub disable_distro_ofed()
{
	if ( "$MODULE_CONF_DIST_FILE" ne "" && -f "$MODULE_CONF_DIST_FILE" ) {
		my $res;

		$res = open(MDIST, "$MODULE_CONF_DIST_FILE");
		if ( ! $res ) {
			NormalPrint("Can't open $MODULE_CONF_DIST_FILE for input: $!");
			return;
		}
		my @mdist_lines;
		while (<MDIST>) {
			push @mdist_lines, $_;
		}
		close(MDIST);

		$res = open(MDIST, ">$MODULE_CONF_DIST_FILE");
		if ( ! $res ) {
			NormalPrint("Can't open $MODULE_CONF_DIST_FILE for output: $!");
			return;
		}
		foreach my $line (@mdist_lines) {
			chomp $line;
			if ($line =~ /^\s*install ib_core|^\s*alias ib|^\s*alias net-pf-26 ib_sdp/) {
				print MDIST "# $line\n";
			} else {
				print MDIST "$line\n";
			}
		}
		close(MDIST);
	}
}

# =============================================================================
# The functions and constants below assist in editing limits.conf
# to add IB specific entries related to memory locking

my $LIMITS_CONF_FILE = "/etc/security/limits.conf";
my $LIMITS_CONF="limits.conf";	## additions to limits.conf

# marker strings used in LIMITS_CONF_FILE
# for entries added by installation
my $START_LIMITS_MARKER="OPA Settings Start here";
my $END_LIMITS_MARKER="OPA Settings End here";

# Keep track of whether we already did edits to avoid repeated edits
my $DidLimits=0;

# Path to opasystemconfig
my $OPA_SYSTEMCFG_FILE = "/sbin/opasystemconfig";

# remove iba entries from modules.conf
sub remove_limits_conf()
{
	$DidLimits = 0;
	if ( -e "$LIMITS_CONF_FILE") {
		if (check_keep_config($LIMITS_CONF_FILE, "", "y"))
		{
			print "Keeping /$LIMITS_CONF_FILE changes ...\n";
		} else {
			print "Modifying $LIMITS_CONF_FILE ...\n";
			if ( -e "$OPA_SYSTEMCFG_FILE" ) {
			     system("$OPA_SYSTEMCFG_FILE --disable Memory_Limit");
			}
		}
	}
}

#
# Override the system's standard udev configuration to allow
# different access rights to some of the infiniband device files.
#
my $UDEV_RULES_DIR ="/etc/udev/rules.d";
my $UDEV_RULES_FILE = "05-opa.rules";
my $Default_UserQueries = 0;

my $udev_perm_string = "Allow non-root users to access the UMAD interface?";

AddAnswerHelp("UserQueries", "$udev_perm_string");

sub install_udev_permissions($)
{
	my ($srcdir) = shift(); # source directory.
	my $SourceFile;
	my $Context;
	my $Cnt;

	if ($Default_UserQueries == 0) {
		$Default_UserQueries = GetYesNoWithMemory("UserQueries",0,"$udev_perm_string", "y");
	}

	if ($Default_UserQueries > 0) {
                # Installation of udev will be taken care during RPM installation, we just have to set
                setup_env("OPA_UDEV_RULES", 1);
	} elsif ( -e "$UDEV_RULES_DIR/$UDEV_RULES_FILE" ) {
		#update environment variable accordingly
		setup_env("OPA_UDEV_RULES", 0);
	} else {
		# do nothing
		setup_env("OPA_UDEV_RULES", -1);
	}
}

sub remove_udev_permissions()
{
	remove_file("$UDEV_RULES_DIR/$UDEV_RULES_FILE");
}

#
# Ensures OPA drivers are incorporated in the initial ram disk.
#
my $CallDracut = 0;

sub rebuild_ramdisk()
{
	# Just increase the count. We will do the rebuild at the end of the script.
	$CallDracut++;
}

sub do_rebuild_ramdisk()
{
	if ($CallDracut && -d '/boot') {
		my $cmd = $DRACUT_EXE_FILE . ' --stdlog 0';
		if ( -d '/dev') {
			$cmd = $DRACUT_EXE_FILE;
		}
		#name of initramfs may vary between distros, so need to get it from lsinitrd
		my $current_initrd = `lsinitrd 2>&1 | head -n 1`;
		my ($initrd_prefix) = $current_initrd =~ m/\/boot\/(\w+)-[\w\-\.]+.*/;
		my $initrd_suffix = "";
                if ($current_initrd =~ m/\.img[':]/) {
                        $initrd_suffix = ".img";
                }
		my $tmpfile = "/tmp/$initrd_prefix-$CUR_OS_VER$initrd_suffix";

		if ( -e $cmd ) {
			do {
				NormalPrint("Rebuilding boot image with \"$cmd -f $tmpfile $CUR_OS_VER\"...\n");
				# Try to build a temporary image first as a dry-run to make sure
				# a failed run will not destroy an existing image.
				if (system("$cmd -f $tmpfile $CUR_OS_VER") == 0 && system("mv -f $tmpfile /boot/") == 0) {
					NormalPrint("New initramfs installed in /boot.\n");
					NormalPrint("done.\n");
					return;
				} else {
					NormalPrint("failed.\n");
				}
			} while(GetYesNo("Do you want to retry?", "n"));
			$exit_code = 1;
		} else {
			NormalPrint("$cmd not found, cannot update initial ram disk.");
			$exit_code = 1;
		}
	}
}

