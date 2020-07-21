#!/usr/bin/perl
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

#

use strict;
#use Term::ANSIColor;
#use Term::ANSIColor qw(:constants);
#use File::Basename;
#use Math::BigInt;

# ========================================================================
# Basic configuration files

my $CONFIG_DIR = "/etc"; # general driver config
my $OFED_CONFIG_DIR = "/etc/infiniband"; # general driver config
my $OFED_CONFIG = "$OFED_CONFIG_DIR/openib.conf";
my $OPA_CONFIG_DIR = "/etc/rdma"; 
my $OPA_CONFIG = "$OPA_CONFIG_DIR/rdma.conf";
my %KeepConfig = (); # keep track of past questions to config questions
my $Default_RpmConfigKeepOld=0; # -O option used to select current rpm config file
my $Default_RpmConfigUseNew=0; # -N option used to select new rpm config file

my $TMP_CONF="/tmp/conf.$$";	# scratch file

sub check_config_dirs()
{
	check_dir("$BASE_DIR");
	check_dir("$OPA_CONFIG_DIR");
}

sub check_keep_config($$$)
{
	my($configFile) = shift();
	my($configDesc) = shift();
	my($default) = shift();

	my($prompt) = "";
	my $keep;

	if ("$configDesc" eq "")
	{
		$prompt="$configFile";
	} else {
		$prompt="$configDesc ($configFile)";
	}

	if (! exists $KeepConfig{"$configFile"})
	{
		$keep=system "diff $configFile $configFile-sample > /dev/null 2>&1";
		if ($keep != 0)
		{
			NormalPrint "You have a modified $configFile configuration file\n";
			$KeepConfig{"$configFile"} = GetYesNo("Do you want to keep $prompt?", "$default");
		} else {
			$KeepConfig{"$configFile"} = $keep;
		}
	}
	return $KeepConfig{"$configFile"};
}

# This is to update the old path when upgrading from 10.3 to newer version
sub replace_old_path_in($)
{
	my $filename = shift();
	open(FILE, "<$filename") || die "\nUnable to locate $filename\n";
	my @lines = <FILE>;
	close(FILE);

	# Replace /etc/sysconfig/opa/ with /etc/opa/
	my @newlines;
	foreach(@lines) {
		$_ =~ s/\/etc\/sysconfig\/opa\//\/etc\/opa\//g;
		push(@newlines,$_);
	}

	open(FILE, ">$filename") || die "\nUnable to locate $filename\n";
	print FILE @newlines;
	close(FILE);
}

# This subroutine is used to check if we should keep a modified rpm config file
# Optional second parameter used to check if a file exist in a previous (now deprecated)
# file location.
sub check_rpm_config_file($;$)
{
	my($config) = shift();
	my($old_config_dir) = shift();
	my($file_name) = basename($config);

	if ( $old_config_dir ne "" && !$Default_RpmConfigUseNew) {
		if ( -e "$old_config_dir/$file_name") {
			if ($Default_RpmConfigKeepOld || GetYesNo ("$file_name found in old file path $old_config_dir, move to new location?", "y")) {
				# This is to update the old path when upgrading from 10.3 to newer version
				replace_old_path_in ("$old_config_dir/$file_name");
				system "mv -f $old_config_dir/$file_name $config";
				return;
			}
		} elsif ( -e "$old_config_dir/$file_name.rpmsave") {
			if ($Default_RpmConfigKeepOld || GetYesNo ("$file_name.rpmsave found in old file path $old_config_dir, move to new location?", "y")) {
				# This is to update the old path when upgrading from 10.3 to newer version
				replace_old_path_in ("$old_config_dir/$file_name.rpmsave");
				system "mv -f $old_config_dir/$file_name.rpmsave $config";
				return;
			}
		}
	}

	if ( -e "$config.rpmsave") {
		if ($Default_RpmConfigKeepOld) {
			system "mv -f $config.rpmsave $config";
		} elsif ($Default_RpmConfigUseNew){
			system "rm -f $config.rpmsave";
		} elsif (GetYesNo ("Do you want to keep $config?", "y")) {
			system "mv -f $config.rpmsave $config";
		} else {
			system "rm -f $config.rpmsave";
		}
	} elsif ( -e "$config.rpmnew") {
		if ($Default_RpmConfigKeepOld) {
			system "rm -f $config.rpmnew";
		} elsif ($Default_RpmConfigUseNew){
			system "mv -f $config.rpmnew $config";
		} elsif (GetYesNo ("Do you want to keep $config?", "y")) {
			system "rm -f $config.rpmnew";
		} else {
			system "mv -f $config.rpmnew $config";
		}
	}
}

sub clear_keep_config($)
{
	my($configFile) = shift();

	delete $KeepConfig{"$configFile"};
}

sub remove_conf_file($$)
{
	my($WhichDriver) = shift();
	my($ConfFile) = shift();

	if (-e "$ConfFile") 
	{
		if (check_keep_config("$ConfFile", "$WhichDriver configuration file", "y"))
		{
			NormalPrint "Keeping $WhichDriver configuration file ($ConfFile) ...\n";
		} else {
			system "rm -rf $ConfFile";
		}
	}
	system "rm -rf ${ConfFile}-sample";
}

# remove a config file where a previous release used a different name for
# the file
sub remove_renamed_conf_file($$;@)
{
	my($WhichDriver) = shift();
	my($ConfFile) = shift();
	my(@OldConfFile_list) = @_;	# old config file names

	foreach my $OldConfFile ( @OldConfFile_list) {
		remove_conf_file("$WhichDriver", "$OldConfFile");
	}
	remove_conf_file("$WhichDriver", "$ConfFile");
}

sub install_conf_file($$$;@)
{
	my($WhichDriver) = shift();
	my($ConfFileDest) = shift();
	my($ConfFileSrc) = shift();
	my(@OldConfFileDest_list) = @_;	# optional
	my $diff_src_dest;
	my $diff_src_sample;
	my $diff_dest_sample;
	my $keep;
	my $need_copy = 1;

	# install an appropriate file into $ConfFileDest
	foreach my $OldConfFileDest ( @OldConfFileDest_list, $ConfFileDest ) {
		DebugPrint("Checking $OldConfFileDest\n");
		next if ( ! -e "$OldConfFileDest" );

		$diff_src_dest = system "diff $ConfFileSrc $OldConfFileDest > /dev/null 2>&1";

		if ( ! -e "${OldConfFileDest}-sample" )
		{
			DebugPrint("No ${OldConfFileDest}-sample\n");
			$diff_src_sample=1;
			$diff_dest_sample=1;
		} else {
			$diff_src_sample = system "diff $ConfFileSrc ${OldConfFileDest}-sample > /dev/null 2>&1";
			$diff_dest_sample = system "diff $OldConfFileDest ${OldConfFileDest}-sample > /dev/null 2>&1";
		}
		DebugPrint("Comparisons: src vs old=$diff_src_dest, src vs oldsample=$diff_src_sample, old vs oldsample=$diff_dest_sample\n");

		if ($diff_src_dest != 0)
		{
			if ($diff_dest_sample == 0)
			{
				NormalPrint "You have an unmodified $WhichDriver configuration file from an earlier install\n";
				$keep =check_keep_config("$OldConfFileDest", "", "n");
			} elsif ($diff_src_sample != 0)
			{
				NormalPrint "You have a $WhichDriver configuration file from an earlier install\n";
				$keep=check_keep_config("$OldConfFileDest", "", "y");
			} else {
				NormalPrint "You have a modified $WhichDriver configuration file\n";
				$keep=check_keep_config("$OldConfFileDest", "", "y");
			}
			if ( $keep ) {
				if ("$OldConfFileDest" ne "$ConfFileDest") {
					# we are working on an old config file
					copy_data_file("$OldConfFileDest", "$ConfFileDest");
					NormalPrint "Using $OldConfFileDest as $ConfFileDest...\n";
				} else {
					# we are working against the new supplied config file
					NormalPrint "Leaving $OldConfFileDest unchanged...\n";
				}
				$need_copy=0;
				last;
			} # otherwise onto next file, if needed will copy new file below
		}
	}
	if ( $need_copy) {
		# no old files kept (or not found), use new release version
		NormalPrint "Updating $ConfFileDest ...\n";
		copy_data_file("$ConfFileSrc", "$ConfFileDest");
	}
	foreach my $OldConfFileDest ( @OldConfFileDest_list ) {
		system "rm -rf ${OldConfFileDest}-sample";
	}
	copy_file("$ConfFileSrc", "${ConfFileDest}-sample", "$OWNER", "$GROUP", "ugo=r,u=r");
}

# after doing an rpm install, config files will be either installed (if new)
# or retained with the new version in .rpmnew
# this allows user to select which to keep as their present config file
sub copy_rpm_conf_file($$;@)
{
	my($WhichDriver) = shift();
	my($ConfFileDest) = shift();
	my(@OldConfFileDest_list) = @_;	# optional
	my($ConfFileSrc) = "$ConfFileDest.rpmnew";

	if ( ! -e "$ConfFileDest.rpmnew" )
	{
		# must be first install, Dest is the newest file
		# save a copy in case we overwrite it with an OldConfFileDest below
		system("cp $ConfFileDest $TMP_CONF");
		$ConfFileSrc="$TMP_CONF"
	}
	install_conf_file($WhichDriver, $ConfFileDest, $ConfFileSrc, @OldConfFileDest_list);
	system("rm -f $ConfFileSrc");
}

# After an RPM install, config files from previous install (denoted via .rpmsave)
# may need to be preserved. Ask the user if they want to keep.
sub preserve_prev_rpm_conf($)
{
	my($ConfFileDest) = shift();
	my($ConfFileSave) = "$ConfFileDest.rpmsave";
	
	if ( -e "$ConfFileSave" )
	{
		#install_conf_file($WhichDriver, $ConfFileDest, $ConfFileSave);
		#system("rm -f $ConfFileSave");
		my $diff_src_dest = system "diff $ConfFileDest $ConfFileSave > /dev/null 2>&1";
		if ($diff_src_dest != 0) {
			printf("You have a modified $ConfFileDest configuration file \n");
			my $keep = GetYesNo("Do you want to keep $ConfFileDest?", "y");
			if ($keep) {
				printf("Using the modified $ConfFileDest file \n");
				copy_data_file("$ConfFileSave", "$ConfFileDest");
			}
			else {
				printf("Using the new $ConfFileDest file \n");
			}
		}

		system "rm -f $ConfFileSave"
	}
}

# install a config file where a previous release used a different name for
# the file
sub install_renamed_conf_file($$$;@)
{
	my($WhichDriver) = shift();
	my($ConfFileDest) = shift();
	my($ConfFileSrc) = shift();
	my(@OldConfFileDest_list) = @_;	# in order that we consider them

	my $diff_src_dest;
	my $diff_src_sample;
	my $diff_dest_sample;

	if (! -e "$ConfFileDest") {
		# first upgrade install, we consider old files as relevant
		# we want to do the install_conf_file algorithm
		# but use OldConfFileDest for all but file destination of copy
		install_conf_file("$WhichDriver","$ConfFileDest","$ConfFileSrc",@OldConfFileDest_list);
	} else {
		# don't touch old (if any), just perform normal upgrade/install
		foreach my $OldConfFileDest ( @OldConfFileDest_list ) {
			system "rm -rf ${OldConfFileDest}-sample";
		}
		install_conf_file("$WhichDriver","$ConfFileDest","$ConfFileSrc");
	}
}

# ===========================================================================
# functions to insert/remove config file additions bounded by markers

# 
# Deletes contents from start marker to end marker
# arg0 : Start marker
# arg1 : End Marker
# arg2 : Leave Marks in file after removing contents
# arg3 : Filename to operate on
#

sub del_marks($$$$)
{
	my($StartMark) = shift();
	my($EndMark) = shift();
	my($LeaveMarks) = shift();
	my($FileName) = shift();

	my($found_mark)=0;

	open (INPUT, "$FileName");
	open (OUTPUT, ">>$TMP_CONF");

	select (OUTPUT);
	while (($found_mark == 0) && ($_=<INPUT>)) {
		if (/$StartMark/) {
			$found_mark=1;
			if ($LeaveMarks) {
				print $_;
			}
		} else {
			print $_;
		}
	}

	while (($found_mark == 1) && ($_=<INPUT>)) {
		if (/$EndMark/) {
			$found_mark = 0;
			if ($LeaveMarks) {
				print $_;
			}
		} 
	}

	while ($_=<INPUT>) {
		print $_;
	}

	select(STDOUT);

	close (INPUT);
	close (OUTPUT);

	system "mv $TMP_CONF $FileName";
}

# gets contents including marks and compares to specified file
# returns 0 on match, 1 if different
sub compare_marks($$$$)
{
	my($StartMark) = shift();
	my($EndMark) = shift();
	my($FileName) = shift();
	my($CompareTo) = shift();

	my $is_different;
	my $found_inf;

	system "awk '/$StartMark/, /$EndMark/ {print}' $FileName > /tmp/tmp.conf";
	$found_inf = `diff /tmp/tmp.conf $CompareTo`;
	system "rm -f /tmp/tmp.conf";

	if ($found_inf eq "") 
	{
		$is_different = 0;
	} else 
	{
		$is_different = 1;
	}
	return $is_different
}

# ===========================================================================
# functions to edit simple config files

# This function supports reading parameters from config file which are
# actually shell scripts.  openib.conf, opafastfabric.conf and the Linux network
# config files are examples of such.
# If the file or parameter does not exist, returns ""
sub read_simple_config_param($$)
{
	my $config_file=shift();
	my $parameter=shift();

	if ( ! -e "$config_file" ) {
		return "";
	}
	my $value=`. $config_file >/dev/null 2>/dev/null; echo \$$parameter`;
	chomp($value);
	return "$value";
}

sub edit_simple_config_file($$)
{
	my($FileName) = shift();
	my($changes) = shift();	# set of sed commands

	my $rc;

	$rc = system("sed $changes < $FileName > $TMP_CONF");
	if ($rc != 0) {
		system ("rm -rf $TMP_CONF >/dev/null 2>&1");
		return $rc;
	}

	$rc = system "mv $TMP_CONF $FileName";
	return $rc;
}

# ==========================================================================
# configuration file

# OFED & OPA configuration files are structured as a list of lines of the form:
# 	parameter=value
# There may be additional comment and whitespace lines
sub change_conf_param($$$)
{
	my $parameter=shift();
	my $newvalue=shift();
	my $conf=shift();

	VerbosePrint("edit $conf: '$parameter=$newvalue'\n");
	if (0 != system("grep '^$parameter=' $conf >/dev/null 2>/dev/null")) {
		# add parameter to file
		system ("echo '$parameter=$newvalue' >> $conf 2>/dev/null");;
	} else {
		return edit_simple_config_file("$conf",
			"-e 's/^$parameter=.*/$parameter=$newvalue/g'");
	}
}

sub change_openib_conf_param($$)
{
	my $p1=shift();
	my $p2=shift();
	change_conf_param($p1,$p2,"/$OFED_CONFIG");
}

sub change_opa_conf_param($$)
{
	my $p1=shift();
	my $p2=shift();
	change_conf_param($p1,$p2,"/$OPA_CONFIG");
}

sub read_openib_conf_param($$)
{
	my $parameter=shift();
	my $config_file=shift();	# if "", defaults to standard file

	if ( "$config_file" eq "" ) {
		$config_file="/$OFED_CONFIG";
	}
	return read_simple_config_param($config_file, $parameter);
}

sub read_opa_conf_param($$)
{
	my $parameter=shift();
	my $config_file=shift();	# if "", defaults to standard file

	if ( "$config_file" eq "" ) {
		$config_file="/$OPA_CONFIG";
	}
	return read_simple_config_param($config_file, $parameter);
}
# Function to setup environment variable. used to setup environment variables for RPM post install
# configuration
sub setup_env($$)
{
        my ($env_type) = shift();  # environment variable
        my ($env_value) = shift(); # value to be set
        $ENV{$env_type}="$env_value";
}

sub prompt_conf_param($$$$;$)
{
	my $parameter=shift();
	my $parameterDesc=shift();
	my $default=shift();
	my $conf=shift();
	my $OPA_INSTALL_ENV=shift();

	my $prompt;
	my $value;
	my $env_value;

	if ("$parameterDesc" eq "") {
		$prompt="$parameter";
	} else {
		$prompt="$parameterDesc ($parameter)";
	}
	if (GetYesNoWithMemory($parameter, 0, "Enable $prompt?", $default) == 1) {
		$value="yes";
		$env_value=1;
	} else {
		$value="no";
		$env_value=0;
	}
        # some configuation parameter are configured directly, some are by RPM. for RPM we will setup env variable.
        if ( $OPA_INSTALL_ENV eq "OPA_SRPHA_ENABLE" || $OPA_INSTALL_ENV eq "OPA_RENICE_IB_MAD" || $OPA_INSTALL_ENV eq "OPA_SET_IPOIB_CM" ) {
            change_conf_param($parameter, $value, $conf);
        } else {
            # setup env variable, RPM installation will configure these
           setup_env("$OPA_INSTALL_ENV", $env_value);
        }
}

sub prompt_openib_conf_param($$$;$)
{
	my $parameter=shift();
	my $parameterDesc=shift();
	my $default=shift();
	my $OPA_INSTALL_ENV=shift();

	prompt_conf_param($parameter, $parameterDesc, $default, 
		"/$OFED_CONFIG", $OPA_INSTALL_ENV);
}

sub prompt_opa_conf_param($$$;$)
{
	my $parameter=shift();
	my $parameterDesc=shift();
	my $default=shift();
	my $OPA_INSTALL_ENV=shift();

	prompt_conf_param($parameter, $parameterDesc, $default, 
		"/$OPA_CONFIG", $OPA_INSTALL_ENV);
}
