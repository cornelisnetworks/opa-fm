#!/usr/bin/perl
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
if (substr($CUR_OS_VER,0,3) eq "2.6")
{
	$MODULE_CONF_FILE = "/etc/modprobe.conf";
	$MODULE_CONF_DIST_FILE = "/etc/modprobe.conf.dist";
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

sub	edit_modconf($)
{
	my($srcdir) = shift();	# directory containing $OPA_CONF file

	if ($DidConfig == 1)
	{
		return;
	}
	edit_conf_file("$srcdir/$OPA_CONF.$CUR_DISTRO_VENDOR", "$MODULE_CONF_FILE",
		"module dependencies", "$START_DRIVER_MARKER",  "$END_DRIVER_MARKER");
	$DidConfig = 1;
}

# remove iba entries from modules.conf
sub remove_modules_conf()
{
	$DidConfig = 0;
	if (check_keep_config($MODULE_CONF_FILE, "", "y"))
	{
		print "Keeping $ROOT/$MODULE_CONF_FILE changes ...\n";
	} else {
		print "Modifying $ROOT$MODULE_CONF_FILE ...\n";
		del_marks ("$START_DRIVER_MARKER", "$END_DRIVER_MARKER", 0, "$ROOT$MODULE_CONF_FILE");
	}
}

# This code is from OFED, it removes lines related to IB from
# modprobe.conf.  Used to prevent distro specific effects on OFED.
sub disable_distro_ofed()
{
	if ( "$MODULE_CONF_DIST_FILE" ne "" && -f "$ROOT$MODULE_CONF_DIST_FILE" ) {
		my $res;

		$res = open(MDIST, "$ROOT$MODULE_CONF_DIST_FILE");
		if ( ! $res ) {
			NormalPrint("Can't open $ROOT$MODULE_CONF_DIST_FILE for input: $!");
			return;
		}
		my @mdist_lines;
		while (<MDIST>) {
			push @mdist_lines, $_;
		}
		close(MDIST);

		$res = open(MDIST, ">$ROOT$MODULE_CONF_DIST_FILE");
		if ( ! $res ) {
			NormalPrint("Can't open $ROOT$MODULE_CONF_DIST_FILE for output: $!");
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

sub	edit_limitsconf($)
{
	my($srcdir) = shift();	# directory containing $LIMITS_CONF file

	my $SourceFile;

	if ($DidLimits == 1)
	{
		return;
	}
	if (! -e "$ROOT$LIMITS_CONF_FILE")
	{
		# older distros don't have this file
		return;
	}
	# not all distros will have a update for this file
	if ( -e "$srcdir/$LIMITS_CONF.$CUR_DISTRO_VENDOR.$CUR_VENDOR_VER" ) {
		$SourceFile="$srcdir/$LIMITS_CONF.$CUR_DISTRO_VENDOR.$CUR_VENDOR_VER";
	} elsif ( -e "$srcdir/$LIMITS_CONF.$CUR_DISTRO_VENDOR.$CUR_VENDOR_MAJOR_VER" ) {
		$SourceFile="$srcdir/$LIMITS_CONF.$CUR_DISTRO_VENDOR.$CUR_VENDOR_MAJOR_VER";
	} elsif ( -e "$srcdir/$LIMITS_CONF.$CUR_DISTRO_VENDOR" )
	{
		$SourceFile="$srcdir/$LIMITS_CONF.$CUR_DISTRO_VENDOR";
	} else {
		$SourceFile="/dev/null";
	}
	edit_conf_file("$SourceFile", "$LIMITS_CONF_FILE",
		"memory locking limits", "$START_LIMITS_MARKER",  "$END_LIMITS_MARKER");
	$DidLimits = 1;
}

# remove iba entries from modules.conf
sub remove_limits_conf()
{
	$DidLimits = 0;
	if ( -e "$ROOT$LIMITS_CONF_FILE") {
		if (check_keep_config($LIMITS_CONF_FILE, "", "y"))
		{
			print "Keeping $ROOT/$LIMITS_CONF_FILE changes ...\n";
		} else {
			print "Modifying $ROOT$LIMITS_CONF_FILE ...\n";
			del_marks ("$START_LIMITS_MARKER", "$END_LIMITS_MARKER", 0, "$ROOT$LIMITS_CONF_FILE");
		}
	}
}

#
# Override the system's standard udev configuration to allow
# different access rights to some of the infiniband device files.
#
my $UDEV_PERMISSIONS_DIR = "/etc/udev/permissions.d";
my $UDEV_PERMISSIONS_FILE = "05-qib.permissions";
my $OLD_UDEV_PERMISSIONS_FILE = "05-qlogic.permissions";
my $UDEV_RULES_DIR ="/etc/udev/rules.d";
my $UDEV_RULES_FILE = "05-umad.rules";
my $OLD_UDEV_RULES_FILE = "05-qib.rules";
my $Default_UserQueries = 0;

my $udev_perm_string = "Would you like the umadX device files?\n" .
     				"(NOTE: Allowing access to umadX device files may present a security risk.\n" .
					" However, this allows tools such as opasaquery, opaportinfo, etc \n" .
					"to be used by non-root users.)";

AddAnswerHelp("UserQueries", "$udev_perm_string");

sub install_udev_permissions($)
{
	my ($srcdir) = shift(); # source directory.
	my $SourceFile;

	if ($Default_UserQueries == 0) {
		$Default_UserQueries = GetYesNoWithMemory("UserQueries",0,"$udev_perm_string", "y");
	}

	if ( -e "$UDEV_RULES_DIR/$OLD_UDEV_RULES_FILE" ) {
		remove_file("$UDEV_RULES_DIR/$OLD_UDEV_RULES_FILE");
	}

	if ($Default_UserQueries > 0) {
		if ( -e "$ROOT$UDEV_RULES_DIR" ) {
			$SourceFile="$srcdir/udev.rules";
			print "Updating udev rules.\n";
			#removing older file
			copy_file("$SourceFile",
					  "$UDEV_RULES_DIR/$UDEV_RULES_FILE",
					  "root", "root",
					  "0644");
		}
	} elsif ( -e "$UDEV_RULES_DIR/$UDEV_RULES_FILE" ) {
		remove_files("$UDEV_RULES_DIR/$UDEV_RULES_FILE");
	}
}

sub remove_udev_permissions()
{
	remove_file("$UDEV_PERMISSIONS_DIR/$OLD_UDEV_PERMISSIONS_FILE");
	remove_file("$UDEV_RULES_DIR/$OLD_UDEV_RULES_FILE");
	remove_file("$UDEV_PERMISSIONS_DIR/$UDEV_PERMISSIONS_FILE");
	remove_file("$UDEV_RULES_DIR/$UDEV_RULES_FILE");
}

#
# Ensures OPA drivers are incorporated in the initial ram disk.
#
sub rebuild_ramdisk()
{
	my $cmd = "/usr/bin/dracut";

	if ( -e $cmd ) {
		NormalPrint("Rebuilding boot image with \"$cmd -f\"...");
		if (system("$cmd -f") == 0) {
			NormalPrint("done.\n");
		} else {
			NormalPrint("failed.\n");
		}
	} else {
		NormalPrint("$cmd not found, cannot update initial ram disk.");
	}
		
}
