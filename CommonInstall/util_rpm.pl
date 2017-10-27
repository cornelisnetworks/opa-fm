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
# The functions and constants below assist in using RPM files

# on platforms without 32 bit app on 64 bit OS support, these are ''
# _glob is a glob pattern version which can be used to match rpm names
# when building rpms the simple _cpu32 values should be used
my $suffix_64bit = "";	# suffix for rpm name for 64 bit rpms

my $rpm_check_dependencies = 1;	# can be set to 0 to disable rpm_check_os_prereqs
my $skip_kernel = 0; # can be set to 1 by --user-space argument

sub rpm_query_param($);
# TBD or
# my $RPM_ARCH = rpm_query_param('_target_cpu');
# chomp $RPM_ARCH;
my $RPM_ARCH=my_tolower("$ARCH");
if ( "$RPM_ARCH" eq "ia32" ) {
	$RPM_ARCH = rpm_query_param('_target_cpu');
	#$RPM_ARCH=`uname -m`; # or i386
	chomp $RPM_ARCH;
}
my $RPM_KERNEL_ARCH = `uname -m`; # typically matches $RPM_ARCH
chomp $RPM_KERNEL_ARCH;

my $RPM_DIST=`rpm -qf /etc/issue 2>/dev/null|head -1`;
chomp $RPM_DIST;
if ( "$RPM_DIST" eq "" ) {
	$RPM_DIST="unsupported";
} else {
	# This effectively gets rid of any CPU architecture suffix in RPM_DIST
	$RPM_DIST=`rpm -q --queryformat "[%{NAME}]-[%{VERSION}]-[%{RELEASE}]" $RPM_DIST 2>/dev/null`;
	chomp $RPM_DIST;
	if ( "$RPM_DIST" eq "" ) {
		$RPM_DIST="unsupported";
	}
}
sub rpm_query_release_pkg($);
my $RPM_DIST_REL = rpm_query_release_pkg($RPM_DIST);

# version string for kernel used in RPM filenames uses _ instead of -
sub rpm_tr_os_version($)
{
	my($osver) = shift();	# uname -r style output
	$osver =~ s/-/_/g;
	return "$osver";
}

# query a parameter of RPM tools themselves
sub	rpm_query_param($)
{
	my($param) = shift();	# parameter name: such as _mandir, _sysconfdir, _target_cpu
	my $ret = `chroot /$ROOT $RPM --eval "%{$param}"`;
	chomp $ret;
	return $ret;
}

# query an attribute of the RPM file
sub	rpm_query_attr($$)
{
	my($rpmfile) = shift();	# .rpm file
	my($attr) = shift();	# attribute name: such as NAME or VERSION

	if ( ! -f "$rpmfile" ) {
		return "";
	}
	return `$RPM --queryformat "[%{$attr}]" -qp $rpmfile 2>/dev/null`;
}

# query an attribute of the installed RPM package
sub	rpm_query_attr_pkg($$)
{
	my($package) = shift();	# installed package
	my($attr) = shift();	# attribute name: such as NAME or VERSION

	return `chroot /$ROOT $RPM --queryformat "[%{$attr}]" -q $package 2>/dev/null`;
}

# determine if rpm parameters will allow srpms to build debuginfo rpms
sub rpm_will_build_debuginfo()
{
	# If debug_package is set to nil in .rpmmacros, srpms will not generate
	# -debuginfo rpms, so they will not be available to install
	return (rpm_query_param('debug_package') ne "");
}

# get NAME of RPM file
sub rpm_query_name($)
{
	my($rpmfile) = shift();	# .rpm file

	return rpm_query_attr($rpmfile, "NAME");
}

# get RELEASE of installed RPM package
sub rpm_query_release_pkg($)
{
	my($package) = shift();	# installed package

	return rpm_query_attr_pkg($package, "RELEASE");
}

sub rpm_query_version_release($)
{
	my($rpmfile) = shift();	# .rpm file

	my $ver = rpm_query_attr($rpmfile, "VERSION");
	my $rel = rpm_query_attr($rpmfile, "RELEASE");
	return "${ver}-${rel}";
}

# get VERSION-RELEASE of installed RPM package
sub rpm_query_version_release_pkg($)
{
	my($package) = shift();	# installed package

	my $ver = rpm_query_attr_pkg($package, "VERSION");
	my $rel = rpm_query_attr_pkg($package, "RELEASE");
	return "${ver}-${rel}";
}

# return NAME-VERSION-RELEASE.ARCH
sub rpm_query_full_name($)
{
	my($rpmfile) = shift();	# .rpm file

	if ( ! -f "$rpmfile" ) {
		return "";
	}
	return `$RPM --queryformat '[%{NAME}-%{VERSION}-%{RELEASE}.%{ARCH}]' -qp $rpmfile 2>/dev/null`;
}

# beware, this could return more than 1 name
#sub rpm_query_full_name_pkg($)
#{
#	my($package) = shift();	# installed package
#	return `chroot /$ROOT $RPM --queryformat '[%{NAME}-%{VERSION}-%{RELEASE}.%{ARCH}]' -q $package 2>/dev/null`;
#}

sub rpm_adjust_mode_cpu($$)
{
	my($package) = shift();	# package name
	my $mode = shift();	# "user" or "kernel rev"
						# "any"- checks if any variation of package is installed

	if ("$mode" eq "user" || "$mode" eq "any") {
			return ($mode, $RPM_ARCH);
		} else {
			return ($mode, $RPM_KERNEL_ARCH);
		}
	}

my $last_checked;	# last checked for in rpm_is_installed
sub rpm_is_installed($$)
{
	my($package) = shift();	# package name
	my $mode = shift();	# "user" or kernel rev
						# "any"- checks if any variation of package is installed
	my $rc;
	my $cpu;

	if ($skip_kernel && "$mode" ne "user" && "$mode" ne "any") {
               return 1;
	}

	( $mode, $cpu ) = rpm_adjust_mode_cpu($package, $mode);
	if ("$mode" eq "any" ) {
		# any variation is ok
		DebugPrint("$RPM -q $package > /dev/null 2>&1\n");
		$rc = system("chroot /$ROOT $RPM -q $package > /dev/null 2>&1");
		$last_checked = "any variation";
	} elsif ("$mode" eq "user") {
		# verify $cpu version or noarch is installed
		DebugPrint "chroot /$ROOT $RPM --queryformat '[%{ARCH}\\n]' -q $package 2>/dev/null|egrep '^$cpu\$|^noarch\$' >/dev/null 2>&1\n";
		$rc = system "chroot /$ROOT $RPM --queryformat '[%{ARCH}\\n]' -q $package 2>/dev/null|egrep '^$cpu\$|^noarch\$' >/dev/null 2>&1";
		$last_checked = "for $cpu or noarch";
	} else {
		# $mode is kernel rev, verify proper kernel version is installed
		# for kernel packages, RELEASE is kernel rev
		my $release = rpm_tr_os_version($mode);
		DebugPrint "chroot /$ROOT $RPM --queryformat '[%{VERSION}\\n]' -q $package 2>/dev/null|egrep '$release' >/dev/null 2>&1\n";
		$rc = system "chroot /$ROOT $RPM --queryformat '[%{VERSION}\\n]' -q $package 2>/dev/null|egrep '$release' >/dev/null 2>&1";
		$last_checked = "for kernel $release";
	}
	DebugPrint("Checked if $package $mode is installed: ".(($rc==0)?"yes":"no")."\n");
	return 0 == $rc;
}

sub rpm_is_installed_list($@)
{
	my $mode = shift();	# "user" or kernel rev
						# "any"- verifies any variation of package is installed
	my(@package_list) = @_;	# package names

	foreach my $package ( @package_list )
	{
		if ( !  rpm_is_installed($package, $mode) ) {
			return 0;
		}
	}
	return 1;
}

# return 0 on success, number of errors otherwise
sub rpm_check_os_prereqs_internal($$@)
{
	my $mode = shift();	# default mode
	my $message = shift();
	my(@package_list) = @_;	# package names
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
				# on RHEL4u5-6: gcc4-gfortran, gcc-g77
				# on RHEL5.1: gcc-gfortran, compat-gcc-34-g77
				# on RHEL5.2: gcc-gfortran
				# on RHEL5.3: gcc43-gfortran
				# on sles10sp1-2: gcc-fortran
				# on sles11: gcc-fortran
				$full_package_info = "${full_package_info}gcc4-gfortran$details|gcc-g77$details|gcc-gfortran$details|compat-gcc-34-g77$details|gcc43-gfortran$details|gcc42-gfortran$details|gcc-fortran$details|gcc42-fortran$details|gcc43-fortran$details"

			} elsif ("$package" eq "libgfortran") {
				# on RHEL4u5-6: libgfortran
				# on RHEL5.1: libgfortran
				# on RHEL5.2: libgfortran
				# on RHEL5.3: libgfortran43
				# on sles10sp1-2: libgfortran
				# on sles11sp0-1: libgfortran43
				# on sles11sp2: libgfortran46
				# TBD - openSUSE11.2 has libgfortran44
                                # sles12sp0: libgfortran3-4
				$full_package_info = "${full_package_info}libgfortran$details|libgfortran42$details|libgfortran43$details|libgfortran46$details|compat-gcc-34-g77$details|libgfortran3$details";
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

			# keep things simple for callers, handle distro specific namings
			
			# distro specific naming of libstdc++
			if ("$package" eq "libstdc++" && "$CUR_DISTRO_VENDOR" eq 'SuSE'
					&& "$CUR_VENDOR_VER" eq 'ES11') {
				# hack because CUR_VENDOR_VER doesn't actually contain the minor 
				# version any more...
				my $CUR_VENDOR_MINOR_VER = `grep PATCHLEVEL /etc/SuSE-release | cut -d' ' -f 3`;
				chop($CUR_VENDOR_MINOR_VER);
				if ($CUR_VENDOR_MINOR_VER eq '0' || $CUR_VENDOR_MINOR_VER eq '1') { $package="libstdc++43"; }
				if ($CUR_VENDOR_MINOR_VER eq '2' ) { $package="libstdc++46"; }
				if ($CUR_VENDOR_MINOR_VER eq '3' ) { $package="libstdc++6"; }
				# SLES11 SP3 has renamed libstdc++47 as libstdc++6
				# TBD - openSUSE has libstdc++42
				# TBD - openSUSE11.2 has libstdc++44
			} elsif ("$package" eq "libstdc++" && "$CUR_DISTRO_VENDOR" eq 'SuSE'
					&& ("$CUR_VENDOR_VER" eq 'ES12' || "$CUR_VENDOR_VER" eq 'ES121' || "$CUR_VENDOR_VER" eq 'ES122')) {
				$package="libstdc++6";
		 	} elsif ("$package" eq "libstdc++-devel" && "$CUR_DISTRO_VENDOR" eq 'SuSE'
					&& "$CUR_VENDOR_VER" eq 'ES11') {
				# hack because CUR_VENDOR_VER doesn't actually contain the minor 
				# version any more...
				my $CUR_VENDOR_MINOR_VER = `grep PATCHLEVEL /etc/SuSE-release | cut -d' ' -f 3`;
				chop($CUR_VENDOR_MINOR_VER);
				if ($CUR_VENDOR_MINOR_VER eq '0' || $CUR_VENDOR_MINOR_VER eq '1') { $package="libstdc++43-devel"; }
				if ($CUR_VENDOR_MINOR_VER eq '2' ) { $package="libstdc++46-devel"; }
				if ($CUR_VENDOR_MINOR_VER eq '3') { $package="libstdc++47-devel"; }
				# TBD - openSUSE has libstdc++42-devel
				# TBD - openSUSE11.2 has libstdc++44-devel
			}

			# distro specific naming of sysfsutils
			if ("$package" eq "sysfsutils" ) {
				if ("$CUR_DISTRO_VENDOR" ne 'SuSE' && "$CUR_DISTRO_VENDOR" ne 'redhat'
			    	&& "$CUR_DISTRO_VENDOR" ne 'fedora' && "$CUR_DISTRO_VENDOR" ne 'rocks') {
					$package="libsysfs";
				} elsif (($CUR_DISTRO_VENDOR eq "redhat" and $CUR_VENDOR_VER eq "ES6")) {
        			$package = "libsysfs";
				}
			}

			# distro specific naming of sysfsutils-devel
			# RHEL4 used sysfsutils-devel
			# RHEL6 uses libsysfs
			# SuSE uses sysfsutils
			# others use libsysfs-devel
			if ("$package" eq "sysfsutils-devel" ) {
				if (($CUR_DISTRO_VENDOR eq "SuSE")
					or ($CUR_DISTRO_VENDOR eq "redhat" and $CUR_VENDOR_VER eq "ES5")
	) {
        			$package = "sysfsutils";
				} elsif (($CUR_DISTRO_VENDOR eq "redhat" and $CUR_VENDOR_VER eq "ES6")) {
        			$package = "libsysfs";
    			} elsif (($CUR_DISTRO_VENDOR eq "rocks") or
        			($CUR_DISTRO_VENDOR eq "fedora") or
        			($CUR_DISTRO_VENDOR eq "redhat")) {
        				$package = "sysfsutils-devel";
    			} else {
        			$package = "libsysfs-devel";
    			}

			}

			if ("$pkgmode" eq "user") {
				# SLES10 and 11 -64bit rpms report ppc as ARCH
				$package="$package$suffix_64bit";
				if ( "$suffix_64bit" ne "" ) {
					$pkgmode="any";
				}
			}
			if (! rpm_is_installed($package, $pkgmode)) {
				if ( $errmessage ne "") {
					$errmessage="$errmessage\n OR ";
				}
				$errmessage = "$errmessage$package ($last_checked)";
			#} elsif ("$version" ne "" && "$version" ne "any") {
			#	# TBD - could be multiple versions of package installed
			#	# in which case this returns merged strings, not good
			#	my $ver = rpm_query_attr_pkg($package, "VERSION");
			#	TBD - string compare not the same as proper numeric compare
			#	if ($ver lt $version) {
			#		if ( $errmessage ne "") {
			#			$errmessage="$errmessage\n OR ";
			#		}
			#		$errmessage = "$errmessage$package version $version ($last_checked)";
			#	}
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
		if($skip_kernel == 1){
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
	my(@package_list) = @_;	# package names
	return rpm_check_os_prereqs_internal($mode, " to build $build_info", @package_list);
}

# get list of package rpms installed
# the list could include multiple versions and/or multiple architectures
# and/or multiple kernel releases
# all entries in returned list are complete package names in the form:
#	NAME-VERSION-RELEASE.ARCH
sub rpms_installed_pkg($$)
{
	my $package=shift();
	my $mode = shift();	# "user" or kernel rev
						# "any"- checks if any variation of package is installed

	my @lines=();
	my $cpu;

	( $mode, $cpu ) = rpm_adjust_mode_cpu($package, $mode);
	if ( "$mode" eq "any" ) {
		DebugPrint("chroot /$ROOT $RPM --queryformat '[%{NAME}-%{VERSION}-%{RELEASE}.%{ARCH}\n]' -q $package 2>/dev/null\n");
		open(rpms, "chroot /$ROOT $RPM --queryformat '[%{NAME}-%{VERSION}-%{RELEASE}.%{ARCH}\n]' -q $package 2>/dev/null|");
	} elsif ("$mode" eq "user" ) {
		DebugPrint("chroot /$ROOT $RPM --queryformat '[%{NAME}-%{VERSION}-%{RELEASE}.%{ARCH}\n]' -q $package 2>/dev/null|egrep '\.$cpu\$|\.noarch\$' 2>/dev/null\n");
		open(rpms, "chroot /$ROOT $RPM --queryformat '[%{NAME}-%{VERSION}-%{RELEASE}.%{ARCH}\n]' -q $package 2>/dev/null|egrep '\.$cpu\$|\.noarch\$' 2>/dev/null|");
	} else {
		# $mode is kernel rev, verify proper kernel version is installed
		# for kernel packages, RELEASE is kernel rev
		my $release = rpm_tr_os_version($mode);
		DebugPrint("chroot /$ROOT $RPM --queryformat '[%{NAME}-%{VERSION}-%{RELEASE}.%{ARCH}\n]' -q $package 2>/dev/null|egrep '-$release\.$cpu\$' 2>/dev/null\n");
		open(rpms, "chroot /$ROOT $RPM --queryformat '[%{NAME}-%{VERSION}-%{RELEASE}.%{ARCH}\n]' -q $package 2>/dev/null|egrep '-$release\.$cpu\$' 2>/dev/null|");
	}
	@lines=<rpms>;
	close(rpms);
	if ( $? != 0) {
		# query command failed, package must not be installed
		@lines=();
	}
	chomp(@lines);
	DebugPrint("package $package $mode: installed: @lines\n");
	return @lines;
}

# identify all possible variations of the package
sub rpm_variations($$)
{
	my $package=shift();
	my $mode = shift();	# "user", kernel rev or "any"
						# "any" checks if any variation of package is installed
	my @variations = ( $package );

	if ( ("$mode" eq "any" || "$mode" eq "user") && $suffix_64bit ne "") {
		@variations = (@variations, "$package$suffix_64bit");
	}
	return @variations;
}

# get list of package rpms installed
# the list could include multiple versions and/or multiple architectures
# and/or multiple kernel releases
# all entries in returned list are complete package names in the form:
#	NAME-VERSION-RELEASE.ARCH
sub rpms_variations_installed_pkg($$)
{
	my $package=shift();
	my $mode = shift();	

	my @result=();

	foreach my $p ( rpm_variations($package, $mode) ) {
		@result = ( @result, rpms_installed_pkg($p, $mode) );
	}
	return @result;
}

# determine if given rpm contains the given file
sub rpm_has_file($$)
{
	my($rpmfile) = shift();	# .rpm file
	my($file) = shift();		# file to look for in rpm

	if ( ! -f "$rpmfile" ) {
		return 0;
	}
	return (! system("$RPM -qlp $rpmfile 2>/dev/null | grep '/${file}\$' > /dev/null 2>&1"));
}

# return names of all installed RPMs matching the given partial name and filter
sub rpm_query_all($$)
{
	my $package = shift();
	my $filter = shift();

	my $res=`chroot /$ROOT $RPM -qa 2>/dev/null|grep -i '$package'|grep '$filter'`;
	$res=~s/\n/ /g;
	return $res;
}

# uninstall all packages which match package partial name and filter
# returns 0 on sucess (or package not installed), != 0 on failure
sub rpm_uninstall_matches($$$;$)
{
	my $name = shift();	# for use only in log messages
	my $package = shift();	# part of package name
	my $filter = shift();	# any additional grep filter
	my $options = shift();	# additional rpm command options
	my $rpms = rpm_query_all("$package", "$filter");

	if ( "$rpms" ne "" ) {
		LogPrint "uninstalling $name: $RPM -e $rpms\n";
		my $out =`chroot /$ROOT $RPM -e $options $rpms 2>&1`;
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
	my($rpmfile) = shift();	# .rpm file
	my $mode = shift();	# "user" or kernel rev
						# "any"- will install any variation found
	my($options) = shift();	# additional rpm command options
						# if " -U " is part of these options, -U will be
						# unconditionally used, even if the rpm is not
						# already installed.  Otherwise -U will only be
						# used if the rpm is installed.  Note that some OFED
						# rpms do not properly handle -U for actual upgrades.
						# Hence for OFED we tend to uninstall the old rpms first
						# and the rpm -i is used to install here.

	my $chrootcmd="";
	my $Uoption= 0;

	if ($skip_kernel && "$mode" ne "user" && "$mode" ne "any") {
		return;
	} elsif ($skip_kernel && $rpmfile =~ /\/hfi1-firmware/) {
		return;
	}

	# We require whitespace around -U so its not mistaken for filenames or other
	# multi-letter options
	if ($options =~ / -U /) {
		$Uoption=1;
		$options =~ s/ -U //;
	}
	
	if ( ! -e $rpmfile ) {
		NormalPrint "Not Found: $rpmfile $mode\n";
		return;
	}

	if (ROOT_is_set()) {
		LogPrint "  cp $rpmfile $ROOT/var/tmp/rpminstall.tmp.rpm\n";
		if (0 != system("cp $rpmfile $ROOT/var/tmp/rpminstall.tmp.rpm")) {
			LogPrint "Unable to copy $rpmfile $ROOT/var/tmp/rpminstall.tmp.rpm\n";
			return;
		}
		# just to new ROOT relative name for use in commands below
		$rpmfile = "/var/tmp/rpminstall.tmp.rpm";
		$chrootcmd="chroot $ROOT ";
	}

	my $package = rpm_query_name($rpmfile);
	my $fullname = rpm_query_full_name($rpmfile);
	my $out;

	NormalPrint "installing ${fullname}...\n";

	my $upgrade=rpm_is_installed($package, $mode);
	if ( ! $upgrade ) {
		my @obsoletes = split /[[:space:]]+/, `$RPM --queryformat '[%{OBSOLETES}\\n]' -qp $rpmfile 2>/dev/null`;
		for my $p ( @obsoletes ) {
			next if ( "$p" eq "(none)" );	
			$upgrade |= rpm_is_installed($p, $mode);
			if ($upgrade) {
				last;
			}
		}
	}

	if( $Uoption || $upgrade ) {	
		# -U option will only update this exact package and architecture
		# when multiple architectures are installed, other architecture rpms
		# are not affected by -U, however --force is needed in that case
		# also need --force for reinstall case
		LogPrint "  $chrootcmd$RPM -U --force $options $rpmfile\n";
		$out=`$chrootcmd$RPM -U --force $options $rpmfile 2>&1`;
		if ( $? == 0 ) {
			NormalPrint("$out");
		} else {
			NormalPrint("ERROR - Failed to install $rpmfile\n");
			NormalPrint("$out");
			$exit_code = 1;
			HitKeyCont;
		}

	} else {
		# initial install of rpm
		# force not required, even if other architectures already installed
		if ("$ARCH" eq 'PPC64') {
			# PPC64 SLES10 needs --force
			$options='--force '."$options";
		}
		LogPrint "  $chrootcmd$RPM -i $options $rpmfile\n";
		$out=`$chrootcmd$RPM -i $options $rpmfile 2>&1`;
		if ( $? == 0 ) {
			NormalPrint("$out");
		} else {
			NormalPrint("ERROR - Failed to install $rpmfile\n");
			NormalPrint("$out");
			$exit_code = 1;
			HitKeyCont;
		}
	}
	if (ROOT_is_set()) {
		system("rm -f $ROOT/var/tmp/rpminstall.tmp.rpm");
	}
}

# returns 0 on sucess (or package not installed), != 0 on failure
# uninstalls all variations of given package
sub rpm_uninstall($$$$)
{
	my($package) = shift();	# package name
	my $mode = shift();	# "user" or kernel rev
						# "any"- checks if any variation of package is installed
	my($options) = shift();	# additional rpm command options
	my($verbosity) = shift();	# verbose or silent
	my @fullnames = rpms_variations_installed_pkg($package, $mode); # already adjusts mode
	my $rc = 0;
	foreach my $fullname (@fullnames) {
		if ( "$verbosity" ne "silent" ) {
			print "uninstalling ${fullname}...\n";
		}
		LogPrint "chroot /$ROOT $RPM -e $options $fullname\n";
		my $out=`chroot /$ROOT $RPM -e $options $fullname 2>&1`;
		$rc |= $?;
		NormalPrint("$out");
		if ($rc != 0) {
			$exit_code = 1;
		}
	}
	return $rc;
}

# resolve rpm package filename within $rpmdir for $mode
sub rpm_resolve($$$)
{
	my $rpmdir = shift();
	my $mode = shift();	# "user" or kernel rev
						# "any"- any variation found
	my $package = shift();	# package name
	my $rpmfile;
	my $cpu;

	( $mode, $cpu ) = rpm_adjust_mode_cpu($package, $mode);
	if ("$mode" eq "user" ) {
		# we expect 0-1 match, ignore all other filenames returned
		DebugPrint("Checking for User Rpm: $rpmdir/${package}-[0-9]*.${cpu}.rpm\n");
		$rpmfile = file_glob("$rpmdir/${package}-[0-9]*.${cpu}.rpm");
		if ( "$rpmfile" eq "" || ! -e "$rpmfile" ) {
			DebugPrint("Checking for User Rpm: $rpmdir/${package}-r[0-9]*.${cpu}.rpm\n");
			$rpmfile = file_glob("$rpmdir/${package}-r[0-9]*.${cpu}.rpm");
		}
		if ( "$rpmfile" eq "" || ! -e "$rpmfile" ) {
			DebugPrint("Checking for User Rpm: $rpmdir/${package}-trunk-[0-9]*.${cpu}.rpm\n");
			$rpmfile = file_glob("$rpmdir/${package}-trunk-[0-9]*.${cpu}.rpm");
		}
		if ( "$rpmfile" eq "" || ! -e "$rpmfile" ) {
			# we expect 0-1 match, ignore all other filenames returned
			DebugPrint("Checking for User Rpm: $rpmdir/${package}-[0-9]*.noarch.rpm\n");
			$rpmfile = file_glob("$rpmdir/${package}-[0-9]*.noarch.rpm");
			if ( "$rpmfile" eq "" || ! -e "$rpmfile" ) {
				DebugPrint("Checking for User Rpm: $rpmdir/${package}-r[0-9]*.noarch.rpm\n");
				$rpmfile = file_glob("$rpmdir/${package}-r[0-9]*.noarch.rpm");
			}
			if ( "$rpmfile" eq "" || ! -e "$rpmfile" ) {
				DebugPrint("Checking for User Rpm: $rpmdir/${package}-trunk-[0-9]*.noarch.rpm\n");
				$rpmfile = file_glob("$rpmdir/${package}-trunk-[0-9]*.noarch.rpm");
			}
		}
	} elsif ("$mode" eq "any" ) {
		# we expect 0-1 match, ignore all other filenames returned
		DebugPrint("Checking for User Rpm: $rpmdir/${package}-[0-9]*.*.rpm\n");
		$rpmfile = file_glob("$rpmdir/${package}-[0-9]*.*.rpm");
		if ( "$rpmfile" eq "" || ! -e "$rpmfile" ) {
			DebugPrint("Checking for User Rpm: $rpmdir/${package}-r[0-9]*.*.rpm\n");
			$rpmfile = file_glob("$rpmdir/${package}-r[0-9]*.*.rpm");
		}
		if ( "$rpmfile" eq "" || ! -e "$rpmfile" ) {
			DebugPrint("Checking for User Rpm: $rpmdir/${package}-trunk-[0-9]*.*.rpm\n");
			$rpmfile = file_glob("$rpmdir/${package}-trunk-[0-9]*.*.rpm");
		}
	} else {
		my $osver = rpm_tr_os_version("$mode");	# OS version
		# we expect 1 match, ignore all other filenames returned
		if ( "$CUR_VENDOR_VER" eq 'ES122' || "$CUR_VENDOR_VER" eq 'ES123') {
			DebugPrint("Checking for Kernel Rpm: $rpmdir/${package}-${osver}_k*.${cpu}.rpm\n");
			$rpmfile = file_glob("$rpmdir/${package}-${osver}_k*.${cpu}.rpm");
		} else {
			DebugPrint("Checking for Kernel Rpm: $rpmdir/${package}-[0-9]*.[0-9][0-9].${osver}-[0-9]*.${cpu}.rpm\n");
			$rpmfile = file_glob("$rpmdir/${package}-[0-9]*.[0-9][0-9].${osver}-[0-9]*.${cpu}.rpm");
		}
		if ( "$rpmfile" eq "" || ! -e "$rpmfile" ) {
			DebugPrint("Checking for Kernel Rpm: $rpmdir/${package}-${osver}-[0-9]*.${cpu}.rpm\n");
			$rpmfile = file_glob("$rpmdir/${package}-${osver}-[0-9]*.${cpu}.rpm");
		}
		if ( "$rpmfile" eq "" || ! -e "$rpmfile" ) {
			DebugPrint("Checking for Kernel Rpm: $rpmdir/${package}-trunk-[0-9]*-${osver}.${cpu}.rpm\n");
			$rpmfile = file_glob("$rpmdir/${package}-trunk-[0-9]*-${osver}.${cpu}.rpm");
		}
	}
	VerbosePrint("Resolved $package $mode: $rpmfile\n");
	return $rpmfile;
}

sub rpm_exists($$$)
{
	my $rpmdir = shift();
	my $mode = shift();	# "user" or kernel rev
						# "any"- any variation found
	my $package = shift();	# package name
	my $rpmfile;

	$rpmfile = rpm_resolve($rpmdir, $mode, $package);
	return ("$rpmfile" ne "" && -e "$rpmfile");
}

sub rpm_install($$$)
{
	my $rpmdir = shift();
	my $mode = shift();	# "user" or kernel rev
						# "any"- will install any variation found
	my $package = shift();	# package name
	my $rpmfile;

	# use a different directory for BUILD_ROOT to limit conflict with OFED
	my $build_temp = "/var/tmp/IntelOPA-DELTA";
	my $BUILD_ROOT="$build_temp/build";
	my $RPM_DIR="$build_temp/DELTARPMS";
	my $RPMS_SUBDIR = "RPMS";
	my $prefix=$OFED_prefix;

	if ($skip_kernel && "$mode" ne "user" && "$mode" ne "any") {
		return;
	} elsif ($skip_kernel && $rpmfile =~ /\/hfi1-firmware/) {
		return;
	}

RPM_RES:

	$rpmfile = rpm_resolve($rpmdir, $mode, $package);
	if ( "$rpmfile" eq "" || ! -e "$rpmfile" ) {
		NormalPrint "Not Found: $package $mode\n";
		if ( "$mode" ne "user" && "$mode" ne "any" ) # kernel mode
		{
			NormalPrint "Rebuilding $package SRPM $mode\n";
			if (0 == build_srpm($package, $RPM_DIR, $BUILD_ROOT, $prefix, "append", $mode)) {
				delta_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmdir");
				goto RPM_RES;
			}
		}
	} else {
		if ("$mode" eq "user" || "$mode" eq "any" ) {
			rpm_run_install($rpmfile, $mode, "");
		} else {
			# kernel install
			if ("$CUR_DISTRO_VENDOR" eq 'SuSE') {
				# ofed1.3 only uses --nodeps on SuSE to workaround ksym
				# dependencies.
				rpm_run_install($rpmfile, $mode, "--nodeps");
			} else {
				rpm_run_install($rpmfile, $mode, "");
			}
		}
	}
}

sub rpm_install_with_options($$$$)
{
	my $rpmdir = shift();
	my $mode = shift();	# "user" or kernel rev
						# "any"- will install any variation found
	my $package = shift();	# package name
	my($options) = shift();	# additional rpm command options
	my $rpmfile;

	# use a different directory for BUILD_ROOT to limit conflict with OFED
	my $build_temp = "/var/tmp/IntelOPA-DELTA";
	my $BUILD_ROOT="$build_temp/build";
	my $RPM_DIR="$build_temp/DELTARPMS";
	my $RPMS_SUBDIR = "RPMS";
	my $prefix=$OFED_prefix;

	if ($skip_kernel && "$mode" ne "user" && "$mode" ne "any") {
		return;
	} elsif ($skip_kernel && $rpmfile =~ /\/hfi1-firmware/) {
		return;
	}

RPM_RES:

	$rpmfile = rpm_resolve($rpmdir, $mode, $package);
	if ( "$rpmfile" eq "" || ! -e "$rpmfile" ) {
		NormalPrint "Not Found: $package $mode\n";
		if ( "$mode" ne "user" && "$mode" ne "any" ) # kernel mode
		{
			NormalPrint "Rebuilding $package SRPM $mode\n";
			if (0 == build_srpm($package, $RPM_DIR, $BUILD_ROOT, $prefix, "append", $mode)) {
				delta_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmdir");
				goto RPM_RES;
			}
		}
	} else {
		if ("$mode" eq "user" || "$mode" eq "any" ) {
			rpm_run_install($rpmfile, $mode, $options);
		} else {
			# kernel install
			if ("$CUR_DISTRO_VENDOR" eq 'SuSE' && 
			    $options !~ / --nodeps /) {
				# ofed1.3 only uses --nodeps on SuSE to workaround ksym
				# dependencies.
				rpm_run_install($rpmfile, $mode, "--nodeps $options");
			} else {
				rpm_run_install($rpmfile, $mode, $options);
			}
		}
	}
}

# verify the rpmfiles exist for all the RPMs listed
sub rpm_exists_list($$@)
{
	my $rpmdir = shift();
	my $mode = shift();	# "user" or kernel rev
						# "any"- any variation found
	my(@package_list) = @_;	# package names

	foreach my $package ( @package_list )
	{
		if (! rpm_exists($rpmdir, $mode, $package) ) {
			return 0;
		}
	}
	return 1;
}

sub rpm_install_list($$@)
{
	my $rpmdir = shift();
	my $mode = shift();	# "user" or kernel rev
						# "any"- will install any variation found
	my(@package_list) = @_;	# package names

	foreach my $package ( @package_list )
	{
		rpm_install($rpmdir, $mode, $package);
	}
}

sub rpm_install_list_with_options($$$@)
{
	my $rpmdir = shift();
	my $mode = shift();	# "user" or kernel rev
						# "any"- will install any variation found
	my($options) = shift();	# additional rpm command options
	my(@package_list) = @_;	# package names

	foreach my $package ( @package_list )
	{
		rpm_install_with_options($rpmdir, $mode, $package, $options);
	}
}

# returns 0 on success (or rpms not installed), != 0 on failure
sub rpm_uninstall_list2($$$@)
{
	my $mode = shift();	# "user" or kernel rev
						# "any"- checks if any variation of package is installed
	my($options) = shift();	# additional rpm command options
	my($verbosity) = shift();	# verbose or silent
	my(@package_list) = @_;	# package names
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
	my $mode = shift();	# "user" or kernel rev
						# "any"- checks if any variation of package is installed
	my($verbosity) = shift();	# verbose or silent
	my(@package_list) = @_;	# package names

	return rpm_uninstall_list2($mode, "", $verbosity, (@package_list));
}


# uninstall all rpms which match any of the supplied package names
# and based on mode and distro/cpu.
# uninstall is done in a single command so dependency issues handled
# returns 0 on success (or rpms not installed), != 0 on failure
sub rpm_uninstall_all_list($$@)
{
	my $mode = shift();	# "user" or kernel rev
						# "any"- checks if any variation of package is installed
	my($verbosity) = shift();	# verbose or silent
	my(@package_list) = @_;	# package names
	my @uninstall = ();
	my $options = "";

	if ("$mode" eq "any") {
		$options = "--allmatches";
	}
	foreach my $package ( @package_list )
	{
		foreach my $p ( rpm_variations($package, $mode) )
		{
			if (rpm_is_installed($p, $mode)) {
				if ("$mode" eq "any") {
					@uninstall = (@uninstall, $p);
				} else {
					@uninstall = (@uninstall, rpms_installed_pkg($p, $mode));
				}
			}
		}
	}
	
	if (scalar(@uninstall) != 0) {
		LogPrint "chroot /$ROOT $RPM -e $options @uninstall\n";
		my $out=`chroot /$ROOT $RPM -e $options @uninstall 2>&1`;
		my $rc=$?;
		NormalPrint("$out");
		if ($rc != 0) {
			$exit_code = 1;
		}
		return $rc;
	} else {
		LogPrint "None Found\n";
		return 0;	# nothing to do
	}
}

# uninstall all rpms which match any of the supplied package names
# and based on mode and distro/cpu, 
# uninstall is done in a single command
# returns 0 on success (or rpms not installed), != 0 on failure
# Force to uninstall without any concern for dependency.
sub rpm_uninstall_all_list_with_options($$$@)
{
	my $mode = shift();	# "user" or kernel rev
						# "any"- checks if any variation of package is installed
	my($options) = shift();	# additional rpm command options
	my($verbosity) = shift();	# verbose or silent
	my(@package_list) = @_;	# package names
	my @uninstall = ();

	if ("$mode" eq "any") {
		$options .= "--allmatches";
	}
	foreach my $package ( @package_list )
	{
		foreach my $p ( rpm_variations($package, $mode) )
		{
			if (rpm_is_installed($p, $mode)) {
				if ("$mode" eq "any") {
					@uninstall = (@uninstall, $p);
				} else {
					@uninstall = (@uninstall, rpms_installed_pkg($p, $mode));
				}
			}
		}
	}
	
	if (scalar(@uninstall) != 0) {
		LogPrint "chroot /$ROOT $RPM -e $options @uninstall\n";
		my $out=`chroot /$ROOT $RPM -e $options @uninstall 2>&1`;
		my $rc=$?;
		NormalPrint("$out");
		if ($rc != 0) {
			$exit_code = 1;
		}
		return $rc;
	} else {
		LogPrint "None Found\n";
		return 0;	# nothing to do
	}
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
		NormalPrint "kernel-source or kernel-devel is required to build $srpm\n";
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

	if ("$CUR_DISTRO_VENDOR" eq 'redhat') {
		if (! rpm_is_installed("rpm-build", "any") ) {
			NormalPrint("rpm-build ($last_checked) is required to build $srpm\n");
			$err++;
		}
	}
	if ( "$CUR_DISTRO_VENDOR" ne "SuSE" && rpm_will_build_debuginfo()) {
		if (! rpm_is_installed("redhat-rpm-config", "any")) {
			NormalPrint("redhat-rpm-config ($last_checked) is required to build $srpm\n");
			$err++;
		}
	}
	return $err;
}

# see if basic prereqs for building are installed
# return 0 on success, or number of missing dependencies
sub check_build_dependencies($)
{
	my $build_info = shift();	# what trying to build
	my $err = 0;

	$err = rpm_check_build_os_prereqs('user', $build_info, ('glibc-devel'));
	return $err;
}
