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
# The functions and constants below assist in using DEB files
my $rpm_check_dependencies = 1;	# can be set to 0 to disable rpm_check_os_prereqs

my $RPM_ARCH=my_tolower("$ARCH");
my $RPM_KERNEL_ARCH = `uname -m`; # typically matches $RPM_ARCH
chomp $RPM_KERNEL_ARCH;

# return array of cmds for verification during init
sub rpm_get_cmds_for_verification()
{
	return ("dpkg", "dpkg-deb", "dpkg-query");
}

sub rpm_to_deb_option_trans($)
{
	my ($options) = shift();

	#Update option is not required for deb, remove it
	$options =~ s/-U//g;
	#Installation despite lacking dependencies should not be necessary on Ubuntu
	$options =~ s/--nodeps/--force-depends/g;

	return $options;
}

# version string for kernel used in RPM filenames uses _ instead of -
sub rpm_tr_os_version($)
{
	my $osver = shift();	# uname -r style output
	$osver =~ s/-/_/g;
	return "$osver";
}

# translation from rpm package format to deb one
sub rpm_to_deb_pkg_trans($)
{
	my ($pkg) = shift(); # package name
	$pkg =~ s/-devel/-dev/g;
	return $pkg;
}

# Attribute translation
sub rpm_to_deb_attr_trans($)
{
	my $attr = shift();	# attribute name: such as NAME or VERSION
	if($attr eq 'RELEASE'){
		$attr = 'Version';
	}

	if($attr eq 'NAME'){
		$attr = 'Package';
	}

	if($attr eq 'VERSION'){
		$attr = 'Version';
	}

	if($attr eq 'INSTALLPREFIX'){
		# TODO: no equivalent in dpkg
		$attr = '/usr/mpi';
	}

	return $attr;
}

# query a parameter of RPM tools themselves
sub	rpm_query_param($)
{
	my $param = shift();	# parameter name: such as _mandir, _sysconfdir, _target_cpu
	my $ret = '';

	if($param eq 'arch'){
		$ret = `dpkg --print-architecture`;
	} elsif ($param eq '_mandir'){
		$ret = '/usr/share/man';
	} elsif ($param eq '_sysconfdir'){
		$ret = '/etc';
	} elsif ($param eq 'optflags'){
		return "";
	} else {
		return "";
	}

	chomp $ret;
	return $ret;
}

# query an attribute of the RPM file
sub	rpm_query_attr($$)
{
	my $debfile = shift();	# .deb file
	my $attr = shift();	# attribute name: such as NAME or VERSION

	$attr = rpm_to_deb_attr_trans($attr);

	if ( ! -f "$debfile" ) {
		return "";
	}

	my $query = 'dpkg-deb --showformat=\'${'.$attr.'}\n\' --show '.$debfile.' 2>/dev/null';
	my $val = `$query`;

	return $val;
}

# query an attribute of the installed RPM package
sub	rpm_query_attr_pkg($$)
{
	my $package = shift();	# installed package
	my $attr = shift();	# attribute name: such as NAME or VERSION

	$attr = rpm_to_deb_attr_trans($attr);
	$package = rpm_to_deb_pkg_trans($package);

	my $val = `chroot /$ROOT dpkg-query --showformat='\${$attr}\n' --show $package 2>/dev/null`;

	return $val; 
}

# get name of deb package
sub rpm_query_name($)
{
	my $debfile = shift();	# .deb file
	return rpm_query_attr($debfile, "Package");
}

# return NAME-VERSION-RELEASE.ARCH
sub rpm_query_full_name($)
{
	my $debfile = shift();	# .deb file

	if ( ! -f "$debfile" ) {
		return "";
	}
	my $var = `dpkg-deb --showformat='[\${Package}-\${Version}.\${Architecture}]' --show $debfile 2>/dev/null`;
	return $var;
}

# get list of package debs installed
# the list could include multiple versions and/or multiple architectures
# and/or multiple kernel releases
# all entries in returned list are complete package names in the form:
#	NAME-VERSION-RELEASE.ARCH
sub rpms_installed_pkg($$)
{
	my $package=shift();
	my $mode = shift();	# "user" or kernel rev or "firmware"
											# "any"- checks if any variation of package is installed
	my @lines=();
	my $cpu;

	$package = rpm_to_deb_pkg_trans($package);

	$cpu = rpm_get_cpu_arch($mode);
	my $cmd = 'chroot /'.$ROOT.' dpkg-query --showformat=\'${Package}:${Architecture}\' --show '.$package.' 2>/dev/null\n';
	if ( "$mode" eq "any" ) {
		DebugPrint($cmd."|");
		open(debs, $cmd."|");
	} elsif ("$mode" eq "user" || "$mode" eq "firmware") {
		DebugPrint($cmd."|egrep '\.$cpu\$' 2>/dev/null\n");
		open(debs, $cmd."|egrep '\.$cpu\$' 2>/dev/null|");
	} else {
		# $mode is kernel rev, verify proper kernel version is installed
		# for kernel packages, RELEASE is kernel rev
		my $release = rpm_tr_os_version($mode);
		DebugPrint($cmd."|egrep '-$release\.$cpu\$' 2>/dev/null\n");
		open(debs, $cmd."|egrep '-$release\.$cpu\$' 2>/dev/null|");
	}
	@lines=<debs>;
	close(debs);
	if ( $? != 0) {
		# query command failed, package must not be installed
		@lines=();
	}
	chomp(@lines);
	DebugPrint("package $package $mode: installed: @lines\n");
	return @lines;
}

# get list of package debs installed
# the list could include multiple versions and/or multiple architectures
# and/or multiple kernel releases
# all entries in returned list are complete package names in the form:
#	NAME:ARCH
sub rpms_variations_installed_pkg($$)
{
	my $package=shift();
	my $mode = shift();

	my @result=();

	@result = ( @result, rpms_installed_pkg($package, $mode) );
	return @result;
}

sub rpm_query_all($$)
{
	my $package = shift();
	my $filter = shift();

	$package = rpm_to_deb_pkg_trans($package);

	my $res=`chroot $ROOT dpkg --get-selections 2>/dev/null |grep -v '\sdeinstall\s'|grep -i '$package'|grep '$filter'`;
	$res=~s/\n/ /g;
	return $res;
}

# determine if rpm parameters will allow srpms to build debuginfo rpms
sub rpm_will_build_debuginfo()
{

}

sub rpm_query_version_release($)
{
	my $debfile = shift();	# .deb file
	return rpm_query_attr($debfile, "VERSION");
}

# get VERSION-RELEASE of installed RPM package
sub rpm_query_version_release_pkg($)
{
	my $package = shift();	# installed package
	return rpm_query_attr_pkg($package, "VERSION");
}

sub rpm_get_cpu_arch($)
{
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- checks if any variation of package is installed
	return rpm_query_param("arch");
}

my $last_checked;	# last checked for in rpm_is_installed
sub rpm_is_installed($$)
{
	my $package = shift();	# package name
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- checks if any variation of package is installed
	my $rc;
	my $cpu;

	$package = rpm_to_deb_pkg_trans($package);

	if ($user_space_only && "$mode" ne "user" && "$mode" ne "any") {
		return 1;
	}

	$cpu = rpm_get_cpu_arch($mode);
	if ("$mode" eq "any" ) {
		# any variation is ok
		my $cmd = "chroot /$ROOT dpkg -l '$package' | egrep '^ii' > /dev/null 2>&1";
		DebugPrint $cmd."\n";
		$rc = system($cmd);
		$last_checked = "any variation";
	} elsif ("$mode" eq "user" || "$mode" eq "firmware") {
		# verify $cpu version or any is installed
		my $cmd = "chroot /$ROOT dpkg -l '$package' | egrep '^ii' 2>/dev/null|egrep '^$cpu\$|' >/dev/null 2>&1";
		DebugPrint $cmd."\n";
		$rc = system $cmd;
		$last_checked = "for $mode $cpu or noarch";
	} else {
		# $mode is kernel rev, verify proper kernel version is installed
		# for kernel packages, RELEASE is kernel rev
		my $release = rpm_tr_os_version($mode);
		my $cmd = "chroot /$ROOT dpkg  --get-selections | grep $package.*'[%{VERSION}\\n]' | grep -v 'deinstall' 2>/dev/null|egrep '$release' >/dev/null 2>&1";
		DebugPrint $cmd."\n";
		$rc = system $cmd;
		$last_checked = "for kernel $release";
	}
	DebugPrint("Checked if $package $mode is installed: ".(($rc==0)?"yes":"no")."\n");
	return 0 == $rc;
}

sub rpm_is_installed_list($@)
{
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- verifies any variation of package is installed
	my @package_list = @_;	# package names

	foreach my $package ( @package_list )
	{
		if ( !  rpm_is_installed($package, $mode) ) {
			return 0;
		}
	}
	return 1
}

# return 0 on success, number of errors otherwise
sub rpm_check_os_prereqs_internal($$@)
{
	my $mode = shift();	# default mode
	my $message = shift();
	my @package_list = @_;	# package names
	my $err=0;
	
	if ( ! $rpm_check_dependencies ) {
		return 0;
	}
	# Check installation requirements
	DebugPrint "Checking prereqs $message: @package_list\n";
	for my $package_info ( @package_list )
	{
		# expand any alias in package_info
		my $full_package_info="";
		my @alternatives = (split ('\|',$package_info));
		for my $altpackage_info ( @alternatives )
		{
			my ($package, $details) = (split (' ',$altpackage_info,2));
			if ($details ne "") {
				$details=" $details";
			}
			if ( "$full_package_info" ne "" ) {
				$full_package_info = "$full_package_info|";
			}

			# expand "g77" alias for various fortran compilers
			# and "libgfortran" alias for various fortran library
			if ("$package" eq "g77" ) {
				# on Ubuntu 17.04: gfortran
				$full_package_info = "${full_package_info}gfortran$details";
			} elsif ("$package" eq "libgfortran") {
				# on Ubuntu 17.04: libgfortran
				$full_package_info = "${full_package_info}libgfortran$details";
			} else {
				$full_package_info = "${full_package_info}$altpackage_info";
			}
		}

		# now process the expanded list of alternatives
		DebugPrint "Checking prereq: $full_package_info\n";
		my $errmessage="";
		@alternatives = (split ('\|',$full_package_info));
		for my $altpackage_info ( @alternatives )
		{
			DebugPrint "Checking prereq: $altpackage_info\n";
			my ($package, $version, $pkgmode) = (split (' ',$altpackage_info));
			if ("$pkgmode" eq "") {
				$pkgmode = $mode;
			}

			$package = rpm_to_deb_pkg_trans($package);
			if (! rpm_is_installed($package, $pkgmode)) {
				if ( $errmessage ne "") {
					$errmessage="$errmessage\n OR ";
				}
				$errmessage = "$errmessage$package ($last_checked)";
			} else {
				$errmessage="";
				last;
			}
		}
		if ( $errmessage ne "" ) {
			if (scalar(@alternatives) > 1) {
				$errmessage="$errmessage\n";
			}
			NormalPrint("$errmessage is required$message\n");
			$err++;
		}
	}
	return $err;
}

sub rpm_check_os_prereqs($$)
{
	no strict 'refs';
	my $comp = shift();
	my $mode = shift();
	my $list_name;
	my $array_ref;
	my @rpm_list = {};
	my $prereq_check = 0;

	if ( ! $rpm_check_dependencies ) {
		return 0;
	}

	$list_name=$comp."_prereq";
	$array_ref = $comp_prereq_hash{$list_name};
	#checking if prereq array exists
	#it assumed that it has no prereqs it array does not exist
	if ( $array_ref ) {
		@rpm_list = @{ $array_ref };
	}
	else{
		return 0;
	}

	#checking whether each entry in rpm_list is installed
	DebugPrint "Checking prereqs for $comp\n";
	foreach (@rpm_list){
		DebugPrint "Checking installation of $_\n";
		#Don't check dependencies for kernel RPMS if their installation is skipped
		if($user_space_only == 1){
			if( "$_" =~ /kernel/ || "$_" =~ /kmod/ || "$_" eq "pciutils" ) {
				DebugPrint("Skipping check for $_ \n");
				next;
			}
		}
		if(!rpm_is_installed($_, $mode)){
			NormalPrint("--> $comp requires $_ \n");
			$prereq_check = 1;
		}
	}

	return $prereq_check;
}

sub rpm_check_build_os_prereqs($$@)
{
	my $mode = shift();
	my $build_info = shift();
	my @package_list = @_;	# package names
	return rpm_check_os_prereqs_internal($mode, " to build $build_info", @package_list);
}

# uninstall all packages which match package partial name and filter
# returns 0 on sucess (or package not installed), != 0 on failure
sub rpm_uninstall_matches($$$;$)
{
	my $name = shift();	# for use only in log messages
	my $package = shift();	# part of package name
	my $filter = shift();	# any additional grep filter
	my $options = shift();	# additional rpm command options

	$options = rpm_to_deb_option_trans($options);
	my $rpms = rpm_query_all("$package", "$filter");

	if ( "$rpms" ne "" ) {
		LogPrint "uninstalling $name: dpkg -r $options $rpms\n";
		my $out =`chroot /$ROOT dpkg -r $options $rpms 2>&1`;
		my $rc=$?;
		NormalPrint("$out");
		if ($rc != 0) {
			$exit_code = 1;
		}
		return $rc;
	} else {
		return 0;
	}
}

sub rpm_run_install($$$)
{
	my $debfile = shift();	# .rpm file
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- will install any variation found
	my $options = shift();	# additional rpm command options

	$options = rpm_to_deb_option_trans($options);

	my $chrootcmd="";

	if ($user_space_only && "$mode" ne "user" && "$mode" ne "any") {
		return;
	}

	if ( ! -e $debfile ) {
		NormalPrint "Not Found: $debfile $mode\n";
		return;
	}

	if (ROOT_is_set()) {
		LogPrint "  cp $debfile $ROOT/var/tmp/rpminstall.tmp.deb\n";
		if (0 != system("cp $debfile $ROOT/var/tmp/rpminstall.tmp.deb")) {
			LogPrint "Unable to copy $debfile $ROOT/var/tmp/rpminstall.tmp.deb\n";
			return;
		}
		# just to new ROOT relative name for use in commands below
		$debfile = "/var/tmp/rpminstall.tmp.deb";
		$chrootcmd="chroot $ROOT ";
	}

	my $package = rpm_query_name($debfile);
	my $fullname = rpm_query_full_name($debfile);
	my $out;

	NormalPrint "installing ${fullname}...\n";

	my $cmd = "$chrootcmd dpkg -i $options $debfile";
	LogPrint $cmd."\n";
	$out=`$cmd 2>&1`;
	if ( $? == 0 ) {
		NormalPrint("$out");
	} else {
		NormalPrint("ERROR - Failed to install $debfile\n");
		NormalPrint("$out");
		$exit_code = 1;
		HitKeyCont;
	}

	if (ROOT_is_set()) {
		system("rm -f $ROOT/var/tmp/rpminstall.tmp.deb");
	}
}

# returns 0 on sucess (or package not installed), != 0 on failure
# uninstalls all variations of given package
sub rpm_uninstall($$$$)
{
	my $package = shift();	# package name
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- checks if any variation of package is installed
	my $options = shift();	# additional rpm command options
	my $verbosity = shift();	# verbose or silent
	my @fullnames = rpms_variations_installed_pkg($package, $mode); # already adjusts mode
	$options = rpm_to_deb_option_trans($options);
	my $rc = 0;

	foreach my $fullname (@fullnames) {
		if ( "$verbosity" ne "silent" ) {
			print "Uninstalling ${fullname}...\n";
		}
		my $cmd = "chroot /$ROOT dpkg -r $options $fullname";
		LogPrint $cmd."\n";
		my $out=`$cmd 2>&1`;
		$rc |= $?;
		NormalPrint("$out");
		if ($rc != 0) {
			$exit_code = 1;
		}
	}
	return $rc;
}

# resolve rpm package filename for $mode
# package_path is a glob style absolute or relative path to the package
# including the package name, but excluding package version, architecture and
# .rpm suffix
sub rpm_resolve($$)
{
	my $debpath = shift();
	my $mode = shift();
	my $debfile;
	my $cpu;

	$package = rpm_to_deb_pkg_trans($package);
	
	$cpu = rpm_get_cpu_arch($mode);
	# we expect 0-1 match, ignore all other filenames returned
	DebugPrint("Checking for User Deb: ${debpath}_*-*_${cpu}.deb\n");
	$debfile = file_glob("${debpath}_*-*_${cpu}.deb");
	if ( "$debfile" eq "" || ! -e "$debfile" ) {
		# we expect 0-1 match, ignore all other filenames returned
		DebugPrint("Checking for User Deb: ${debpath}_*-*.any.deb\n");
		$debfile = file_glob("${debpath}_*-*.any.deb");
	}
	if ( "$debfile" eq "" || ! -e "$debfile" ) {
		# we expect 0-1 match, ignore all other filenames returned
		DebugPrint("Checking for User Deb: ${debpath}_*-*_all.deb\n");
		$debfile = file_glob("${debpath}_*-*_all.deb");
	}
	VerbosePrint("Resolved $debpath $mode: $debfile\n");
	return $debfile;
}

sub rpm_exists($$)
{
	my $debpath = shift();
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- any variation found
	my $debfile;

	$debfile = rpm_resolve("$debpath", $mode);
	return ("$debfile" ne "" && -e "$debfile")
}

sub rpm_install_with_options($$$)
{
	my $debpath = shift();
	my $mode = shift();	# "user" or kernel rev or "firmware"
											# "any"- will install any variation found
	my $options = shift();	# additional deb command options
	my $debfile;

	if ($user_space_only && "$mode" ne "user" && "$mode" ne "any") {
		return;
	}

	$debfile = rpm_resolve("$debpath", $mode);
	if ( "$debfile" eq "" || ! -e "$debfile" ) {
		NormalPrint "Not Found: $debpath $mode\n";
	} else {
		rpm_run_install($debfile, $mode, $options);
	}
}

# TBD - phase out this function (or get all callers to use same options)
sub rpm_install($$)
{
	my $debpath = shift();
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- will install any variation found
	rpm_install_with_options($debpath, $mode, "");
}


# verify the rpmfiles exist for all the RPMs listed
sub rpm_exists_list($$@)
{
	my $debdir = shift();
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- any variation found
	my @package_list = @_;	# package names

	foreach my $package ( @package_list )
	{
		if (! rpm_exists("$debdir/$package", $mode) ) {
			return 0;
		}
	}
	return 1;
}

sub rpm_install_list_with_options($$$@)
{
	my $debdir = shift();
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- will install any variation found
	my $options = shift();	# additional rpm command options
	my @package_list = @_;	# package names
	
	foreach my $package ( @package_list )
	{
		rpm_install_with_options("$debdir/$package", $mode, $options);
	}
}

# TBD - phase out this function (or get all callers to use same options)
sub rpm_install_list($$@)
{
	my $debdir = shift();
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- will install any variation found
	my @package_list = @_;	# package names

	rpm_install_list_with_options($debdir, $mode, "", @package_list);
}

# debpath_list is a list of glob style absolute or relative path to the packages
# including the package name, but excluding package version, architecture and
# .deb suffix
sub rpm_install_path_list_with_options($@)
{
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- will install any variation found
	my $options = shift();	# additional rpm command options
	my @debpath_list = @_;	# deb pathnames

	foreach my $debpath ( @debpath_list )
	{
		rpm_install_with_options("$debpath", $mode, $options);
	}
}

# debpath_list is a list of glob style absolute or relative path to the packages
# including the package name, but excluding package version, architecture and
# .deb suffix
# TBD - phase out this function (or get all callers to use same options)
sub rpm_install_path_list($@)
{
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- will install any variation found
	my @debpath_list = @_;	# deb pathnames

	rpm_install_path_list_with_options($mode, "", @debpath_list);
}

# returns 0 on success (or rpms not installed), != 0 on failure
sub rpm_uninstall_list2($$$@)
{
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- checks if any variation of package is installed
	my $options = shift();	# additional rpm command options
	my $verbosity = shift();	# verbose or silent
	my @package_list = @_;	# package names
	my $package;
	my $ret = 0;	# assume success

	foreach $package ( reverse(@package_list) )
	{
		$ret |= rpm_uninstall($package, $mode, $options, $verbosity);
	}
	return $ret;
}

# returns 0 on success (or rpms not installed), != 0 on failure
sub rpm_uninstall_list($$@)
{
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- checks if any variation of package is installed
	my $verbosity = shift();	# verbose or silent
	my @package_list = @_;	# package names
	my $options = "";
	return rpm_uninstall_list2($mode, $options, $verbosity, @package_list);
}


# uninstall all rpms which match any of the supplied package names
# and based on mode and distro/cpu,
# uninstall is done in a single command
# returns 0 on success (or rpms not installed), != 0 on failure
# Force to uninstall without any concern for dependency.
sub rpm_uninstall_all_list_with_options($$$@)
{
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- checks if any variation of package is installed
	my $options = shift();	# additional rpm command options
	my $verbosity = shift();	# verbose or silent
	my @package_list = @_;	# package names
	return rpm_uninstall_list2($mode, $options, $verbosity, @package_list);
}

# uninstall all rpms which match any of the supplied package names
# and based on mode and distro/cpu.
# uninstall is done in a single command so dependency issues handled
# returns 0 on success (or rpms not installed), != 0 on failure
# # TBD - phase out this function (or get all callers to use same options)
sub rpm_uninstall_all_list($$@)
{
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- checks if any variation of package is installed
	my $verbosity = shift();	# verbose or silent
	my @package_list = @_;	# package names
	return rpm_uninstall_all_list_with_options($mode, "", $verbosity, (@package_list));
}

# see if prereqs for building kernel modules are installed
# return 0 on success, 1 if missing dependencies
sub check_kbuild_dependencies($$)
{
	my $osver = shift();	# kernel rev
	my $srpm = shift();	# what trying to build
	my $dir = "/lib/modules/$osver/build";

	if ( ! $rpm_check_dependencies ) {
		return 0;
	}
	if ( ! -d "$dir/scripts" ) {
		NormalPrint "Unable to build $srpm: $dir/scripts: not found\n";
		NormalPrint "linux-source or linux-headers-generic is required to build $srpm\n";
		return 1;	# failure
	}
	return 0;
}

# see if basic prereqs for building RPMs are installed
# return 0 on success, or number of missing dependencies
sub check_rpmbuild_dependencies($)
{
	my $srpm = shift();	# what trying to build
	my $err = 0;
	if (! rpm_is_installed("devscripts", "any")) {
		NormalPrint("devscripts is required to build $srpm\n");
		$err++;
	}
}

# see if basic prereqs for building are installed
# return 0 on success, or number of missing dependencies
sub check_build_dependencies($)
{
	my $build_info = shift();	# what trying to build
	my $err = 0;

	$err = rpm_check_build_os_prereqs('user', $build_info, ('libc-dev'));
	return $err;
}
