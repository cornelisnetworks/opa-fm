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

# Path to opasystemconfig
my $OPA_SYSTEMCFG_FILE = "/sbin/opasystemconfig";

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
			if ( -e "$ROOT$OPA_SYSTEMCFG_FILE" ) {
			     system("$ROOT$OPA_SYSTEMCFG_FILE --disable Memory_Limit");
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
		my $kver = `uname -r | xargs echo -n`;
		my $tmpfile = "/tmp/initramfs-$kver.img";

		if ( -e $cmd ) {
DO_BOOT_IMG_BUILD:
			NormalPrint("Rebuilding boot image with \"$cmd -f\"...");
			# Try to build a temporary image first as a dry-run to make sure
			# a failed run will not destroy an existing image.
			if (system("$cmd -f $tmpfile") == 0 && system("mv -f $tmpfile /boot/") == 0) {
				NormalPrint("done.\n");
			} else {
				NormalPrint("failed.\n");
				if (GetYesNo("Do you want to retry?", "n")) {
					goto DO_BOOT_IMG_BUILD;
				} else {
					$exit_code = 1;
				}
			}
		} else {
			NormalPrint("$cmd not found, cannot update initial ram disk.");
			$exit_code = 1;
		}
	}
}

my $OPA_MODPROBE_DIR = "/etc/modprobe.d";

sub enable_mod_force_load_file($$)
{
	my ($module) = shift(); # module name
	my ($conf_file) = shift(); # modprobe configuration file
	my ($retval);

	# Create the configuration file if not existing
	if (! -e "$conf_file" ) {
		system ("touch $conf_file");
		system ("echo 'install $module modprobe -i -f $module \$CMDLINE_OPTS' > $conf_file");
		return;
	}
	# Check if we have an entry for this module in the config file
	$retval=`grep -e "install $module" -e "modprobe -i $module" $conf_file  2>/dev/null | grep -v "#"`;
	if ("$retval" eq "") {
		system ("echo 'install $module modprobe -i -f $module \$CMDLINE_OPTS' >> $conf_file");
	} else {
		system("sed -i 's/modprobe -i $module/modprobe -i -f $module/g' $conf_file");
	}
}

sub enable_mod_force_load($)
{
	my ($module) = shift(); # module name
	my ($file);
	my ($retval);
	my ($found) = 0;
	my (@conf_files) = ( `ls $OPA_MODPROBE_DIR` );

	# Check if there is any config file that contains install entry for the module
	# if yes, update it; otherwise, create a new conf file.
	chomp(@conf_files);
	foreach $file (@conf_files) {
		$retval=`grep "install $module" $OPA_MODPROBE_DIR/$file 2>/dev/null | grep -v "#"`;
		if ("$retval" ne "") {
			$found = 1;
			$retval=`echo "$retval" | grep "modprobe -i $module"`;
			if ("$retval" ne "") {
				enable_mod_force_load_file($module, "$OPA_MODPROBE_DIR/$file");
			}
		}
	}

	if (!$found) {
		enable_mod_force_load_file($module, 
			"$OPA_MODPROBE_DIR/$module.conf");
	}
}

my $OPA_IRQBALANCE_FILE = "/etc/sysconfig/irqbalance";
my $OPA_IRQBALANCE_BAK = "/etc/sysconfig/irqbalance.bak";
sub set_opairqbalance()
{
	print "Updating $ROOT$OPA_IRQBALANCE_FILE\n";

	# Look up the current arguments. Note that this may be an empty string.
	my ($original_line) = `egrep -e '^IRQBALANCE_ARGS=' $ROOT$OPA_IRQBALANCE_FILE`;
	chomp($original_line);

	if ($original_line =~ /--hintpolicy=exact/ || $original_line =~ /-h exact/) {
		# Already set to exact. No action is needed.
	} else {
		# Make a backup.
		copy_data_file("$ROOT$OPA_IRQBALANCE_FILE", "$ROOT/$OPA_IRQBALANCE_BAK");

		# Replace the existing hint policy with the new one.
		my ($original_args) = $original_line;
		$original_args =~ s/IRQBALANCE_ARGS=//;
		$original_args =~ s/["']//g;
		my ($new_args) = $original_args;
		$new_args =~ s/--hintpolicy=[a-z]*//;
		$new_args =~ s/-h [a-z]*//;

		$new_args = "$new_args --hintpolicy=exact";

		if ($original_line eq "") {
			# If there were no arguments in the existing file, just append.
			open (OUTPUT, ">>$ROOT$OPA_IRQBALANCE_FILE");
			select (OUTPUT);
			print "IRQBALANCE_ARGS=$new_args\n";
			select(STDOUT);
			close(OUTPUT);
		} else {
			# Otherwise, rewrite the existing line.
			open (INPUT, "$ROOT$OPA_IRQBALANCE_BAK");
			open (OUTPUT, ">$ROOT$OPA_IRQBALANCE_FILE");
			select (OUTPUT);

			while (($_=<INPUT>)) {
				if (/^$original_line/) {
					print "# $_";
					print "IRQBALANCE_ARGS=$new_args\n";
				} else {
					print $_;
				}
			}
			select(STDOUT);

			close (INPUT);
			close (OUTPUT);
			unlink("$ROOT$OPA_IRQBALANCE_BAK");
		}
	}

	# Make sure irqbalance is enabled and started.
	if (substr($CUR_OS_VER,0,3) eq "2.6") {
                `/sbin/chkconfig irqbalance on; service irqbalance restart`
        }
        else {
                `systemctl enable irqbalance; systemctl restart irqbalance`
        }

}
