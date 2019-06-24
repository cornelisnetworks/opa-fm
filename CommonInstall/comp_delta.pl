#!/usr/bin/perl
# BEGIN_ICS_COPYRIGHT8
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
# END_ICS_COPYRIGHT8

# This file incorporates work covered by the following copyright and permission notice

#
# Copyright (c) 2006 Mellanox Technologies. All rights reserved.
#
# This Software is licensed under one of the following licenses:
#
# 1) under the terms of the "Common Public License 1.0" a copy of which is
#    available from the Open Source Initiative, see
#    http://www.opensource.org/licenses/cpl.php.
#
# 2) under the terms of the "The BSD License" a copy of which is
#    available from the Open Source Initiative, see
#    http://www.opensource.org/licenses/bsd-license.php.
#
# 3) under the terms of the "GNU General Public License (GPL) Version 2" a
#    copy of which is available from the Open Source Initiative, see
#    http://www.opensource.org/licenses/gpl-license.php.
#
# Licensee has the right to choose one of the above licenses.
#
# Redistributions of source code must retain the above copyright
# notice and one of the license notices.
#
# Redistributions in binary form must reproduce both the above copyright
# notice, one of the license notices in the documentation
# and/or other materials provided with the distribution.

use strict;
#use Term::ANSIColor;
#use Term::ANSIColor qw(:constants);
#use File::Basename;
#use Math::BigInt;

# ==========================================================================
# OFA_Delta installation, includes Intel value-adding packages only

# all kernel srpms
# these are in the order we must build/process them to meet basic dependencies
my @delta_kernel_srpms_rhel72 = ( 'kmod-ifs-kernel-updates' );
# the kernel srpm for HFI2
my @delta_kernel_srpms_rhel74_hfi2 = ( 'hfi2' );
my @delta_kernel_srpms_sles12_sp2 = ( 'ifs-kernel-updates-kmp-default' );
my @delta_kernel_srpms_sles12_sp3 = ( 'ifs-kernel-updates-kmp-default' );
my @delta_kernel_srpms_sles12_sp4 = ( 'ifs-kernel-updates-kmp-default' );
my @delta_kernel_srpms_sles15 = ( 'ifs-kernel-updates-kmp-default' );
my @delta_kernel_srpms_sles15_sp1 = ( 'ifs-kernel-updates-kmp-default' );
my @delta_kernel_srpms_rhel73 = ( 'kmod-ifs-kernel-updates' );
my @delta_kernel_srpms_rhel74 = ( 'kmod-ifs-kernel-updates' );
my @delta_kernel_srpms_rhel75 = ( 'kmod-ifs-kernel-updates' );
my @delta_kernel_srpms_rhel76 = ( 'kmod-ifs-kernel-updates' );
my @delta_kernel_srpms_rhel8 = ( 'kmod-ifs-kernel-updates' );
my @delta_kernel_srpms = ( );

# This provides information for all kernel srpms
# Only srpms listed in @delta_kernel_srpms are considered for install
# As such, this may list some srpms which are N/A to the selected distro
#
# Fields:
#	Available => indicate which platforms each srpm can be built for
#	Builds => list of kernel and user rpms built from each srpm
#				caller must know if user/kernel rpm is expected
#	PartOf => Components which the rpms built from this srpm are part of
my %delta_srpm_info = (
		# only used in sles12sp2
	"ifs-kernel-updates-kmp-default" =>        { Available => "",
					Builds => "ifs-kernel-updates-kmp-default ifs-kernel-updates-devel",
					PartOf => " opa_stack opa_stack_dev",
					},
		# only used in rhel72 and rhel73
	"kmod-ifs-kernel-updates" =>    { Available => "",
					Builds => "kmod-ifs-kernel-updates ifs-kernel-updates-devel",
					PartOf => " opa_stack opa_stack_dev",
					},
		# only use for HFI2. right now only in rhel74
	"hfi2" =>    { Available => "",
					Builds => "hfi2",
					PartOf => " opa_stack",
					},
);

my %delta_autostart_save = ();
# ==========================================================================
# Delta opa_stack build in prep for installation

# based on %delta_srpm_info{}{'Available'} determine if the given SRPM is
# buildable and hence available on this CPU for $osver combination
# "user" and kernel rev values for mode are treated same
sub available_srpm($$$)
{
	my $srpm = shift();
	# $mode can be any other value,
	# only used to select Available
	my $mode = shift();	# "user" or kernel rev
	my $osver = shift();
	my $avail ="Available";

	DebugPrint("checking $srpm $mode $osver against '$delta_srpm_info{$srpm}{$avail}'\n");
	return arch_kernel_is_allowed($osver, $delta_srpm_info{$srpm}{$avail});
}

# initialize delta srpm list based on specified osver
# for present system
sub init_delta_info($)
{
	my $osver = shift();

	# filter components by distro
	if ("$CUR_DISTRO_VENDOR" eq 'SuSE'
		&& "$CUR_VENDOR_VER" eq 'ES122') {
		@delta_kernel_srpms = ( @delta_kernel_srpms_sles12_sp2 );
	} elsif ("$CUR_DISTRO_VENDOR" eq 'SuSE'
		&& "$CUR_VENDOR_VER" eq 'ES123') {
		@delta_kernel_srpms = ( @delta_kernel_srpms_sles12_sp3 );
	} elsif ("$CUR_DISTRO_VENDOR" eq 'SuSE'
		&& "$CUR_VENDOR_VER" eq 'ES124') {
		@delta_kernel_srpms = ( @delta_kernel_srpms_sles12_sp4 );
	} elsif ("$CUR_DISTRO_VENDOR" eq 'SuSE'
		&& "$CUR_VENDOR_VER" eq 'ES15') {
		@delta_kernel_srpms = ( @delta_kernel_srpms_sles15 );
	} elsif ("$CUR_DISTRO_VENDOR" eq 'SuSE'
		&& "$CUR_VENDOR_VER" eq 'ES151') {
		@delta_kernel_srpms = ( @delta_kernel_srpms_sles15_sp1 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES74" ) {
		if ($HFI2_INSTALL) {
			@delta_kernel_srpms = (@delta_kernel_srpms_rhel74_hfi2);
		} else {
			@delta_kernel_srpms = (@delta_kernel_srpms_rhel74);
		}
	} elsif ( "$CUR_VENDOR_VER" eq "ES8" ) {
		@delta_kernel_srpms = ( @delta_kernel_srpms_rhel8 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES76" ) {
		@delta_kernel_srpms = ( @delta_kernel_srpms_rhel76 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES75" ) {
		@delta_kernel_srpms = ( @delta_kernel_srpms_rhel75 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES73" ) {
		@delta_kernel_srpms = ( @delta_kernel_srpms_rhel73 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES72" ) {
		@delta_kernel_srpms = ( @delta_kernel_srpms_rhel72 );
	} else {
		# unknown distro, leave empty
		@delta_kernel_srpms = ( );
	}

	if (DebugPrintEnabled() ) {
		# dump all SRPM info
		DebugPrint "\nSRPMs:\n";
		foreach my $srpm ( @delta_kernel_srpms ) {
			DebugPrint("$srpm => Builds: '$delta_srpm_info{$srpm}{'Builds'}'\n");
			DebugPrint("           Available: '$delta_srpm_info{$srpm}{'Available'}'\n");
			DebugPrint("           Available: ".available_srpm($srpm, "user", $osver)." PartOf: '$delta_srpm_info{$srpm}{'PartOf'}'\n");
		}
		DebugPrint "\n";
	}
}

sub get_rpms_dir_delta($);

# verify the rpmfiles exist for all the RPMs listed
sub delta_rpm_exists_list($@)
{
	my $mode = shift();	#  "user" or kernel rev or "firmware"
	my(@package_list) = @_;	# package names

	foreach my $package ( @package_list )
	{
		next if ( "$package" eq '' );
		my $rpmdir = get_rpms_dir_delta("$package");
		if (! rpm_exists("$rpmdir/$package", $mode) ) {
			return 0;
		}
	}
	return 1;
}

# resolve filename within $srcdir/$SRPMS_SUBDIR
# and return filename relative to $srcdir
sub delta_srpm_file($$)
{
	my $srcdir = shift();
	my $globname = shift(); # in $srcdir
	my $result;

	if ( $GPU_Install == 1 ) {
		if ( -d $srcdir."/SRPMS/CUDA" ) {
			$result = file_glob("$srcdir/$SRPMS_SUBDIR/CUDA/$globname");
		} else {
			NormalPrint("CUDA specific SRPMs do not exist\n");
			exit 0;
		}
	} else {
		$result = file_glob("$srcdir/$SRPMS_SUBDIR/$globname");
	}

	$result =~ s|^$srcdir/||;
	return $result;
}

# indicate where DELTA built RPMs can be found
sub delta_rpms_dir()
{
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};

	if (-d "$srcdir/$RPMS_SUBDIR/$CUR_DISTRO_VENDOR-$CUR_VENDOR_VER" ) {
		return "$srcdir/$RPMS_SUBDIR/$CUR_DISTRO_VENDOR-$CUR_VENDOR_VER";
	} else {
		return "$srcdir/$RPMS_SUBDIR/$CUR_DISTRO_VENDOR-$CUR_VENDOR_MAJOR_VER";
	}
}

# package is supplied since for a few packages (such as GPU Direct specific
# packages) we may pick a different directory based on package name
sub get_rpms_dir_delta($)
{
	my $package = shift();

	my $rpmsdir = delta_rpms_dir();

	# Note no need to check kernel levels since GPU Direct now supported
	# for all distros.  If we add a distro without this, such as Ubuntu,
	# perhaps argument parser should turn off GPU_Install on that distro.
	if ( $GPU_Install == 1
		 && ( $package =~ /ifs-kernel-updates/ || $package =~ /libpsm/) ) {
			if ( -d $rpmsdir."/CUDA" ) {
				$rpmsdir=$rpmsdir."/CUDA";
			} else {
				NormalPrint("CUDA specific packages do not exist\n");
				exit 0;
			}
	}
	return $rpmsdir;
}

# verify if all rpms have been built from the given srpm
sub is_built_srpm($$)
{
	my $srpm = shift();	# srpm name prefix
	my $mode = shift();	# "user" or kernel rev

	my @package_list = split /[[:space:]]+/, $delta_srpm_info{$srpm}{'Builds'};
	return ( delta_rpm_exists_list($mode, @package_list) );
}

# see if srpm is part of any of the components being installed/reinstalled
sub need_srpm_for_install($$$$)
{
	my $srpm = shift();	# srpm name prefix
	my $mode = shift();	# "user" or kernel rev
	my $osver = shift();
	# add space at start and end so can search
	# list with spaces around searched comp
	my $installing_list = " ".shift()." "; # items being installed/reinstalled

	if (! available_srpm($srpm, $mode, $osver)) {
		DebugPrint("$srpm $mode $osver not available\n");
		return 0;
	}

	my @complist = split /[[:space:]]+/, $delta_srpm_info{$srpm}{'PartOf'};
	foreach my $comp (@complist) {
		next if ("$comp" eq '');
		DebugPrint("Check for $comp in ( $installing_list )\n");
		if ($installing_list =~ / $comp /) {
			return 1;
		}
	}
	return 0;
}

sub need_build_srpm($$$$$$)
{
	my $srpm = shift();	# srpm name prefix
	my $mode = shift();	# "user" or kernel rev
	my $osver = shift();	# kernel rev
	my $installing_list = shift(); # what items are being installed/reinstalled
	my $force = shift();	# force a rebuild
	my $prompt = shift();	# prompt (only used if ! $force)

	return ( need_srpm_for_install($srpm, $mode, $osver, $installing_list)
			&& ($force
				|| ! is_built_srpm($srpm, $mode)
				|| ($prompt && GetYesNo("Rebuild $srpm src RPM for $mode?", "n"))));
}

# move rpms from build tree (srcdir) to install tree (destdir)
sub delta_move_rpms($$)
{
	my $srcdir = shift();
	my $destdir = shift();

	system("mkdir -p $destdir");
	if (file_glob("$srcdir/$RPM_ARCH/*") ne "" ) {
		system("mv $srcdir/$RPM_ARCH/* $destdir");
	}
	if (file_glob("$srcdir/$RPM_KERNEL_ARCH/*") ne "" ) {
		system("mv $srcdir/$RPM_KERNEL_ARCH/* $destdir");
	}
	if (file_glob("$srcdir/noarch/*") ne "" ) {
		system("mv $srcdir/noarch/* $destdir");
	}
}

# build all OFA components specified in installing_list
# if already built, prompt user for option to build
sub build_delta($$$$$$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift(); # what items are being installed/reinstalled
	my $K_VER = shift();	# osver
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild

	my $prompt_srpm = 0;	# prompt per SRPM
	my $force_srpm = $force;	# force SRPM rebuild
	my $force_kernel_srpm = 0;
	my $rpmsdir = delta_rpms_dir();

	# we only support building kernel code, so if user wants to skip install
	# of kernel, we have nothing to do here
	# TBD, if its selected to install opa_stack_dev and user_space_only we still
	# don't rebuild the kernel srpm, even though it creates the -devel package
	# However, that package is not kernel specific, so we should be ok
	if($user_space_only) {
		return 0;	# success
	}

	if (! $force && ! $Default_Prompt && ! $force_kernel_srpm) {
		my $choice = GetChoice("Rebuild OFA kernel SRPM (a=all, p=prompt per SRPM, n=only as needed?)", "n", ("a", "p", "n"));
		if ("$choice" eq "a") {
			$force_srpm=1;
		} elsif ("$choice" eq "p") {
			$prompt_srpm=1;
		} elsif ("$choice" eq "n") {
			$prompt_srpm=0;
		}
	}

	# -------------------------------------------------------------------------
	# do all rebuild prompting first so user doesn't have to wait 5 minutes
	# between prompts
	my %build_kernel_srpms = ();
	my $need_build = 0;
	my $build_compat_rdma = 0;

	# there will be exactly 1 srpm in delta_kernel_srpms
	foreach my $kernel_srpm ( @delta_kernel_srpms ) {
		$build_kernel_srpms{"${kernel_srpm}_build_kernel"} = need_build_srpm($kernel_srpm, "$K_VER", "$K_VER",
							$installing_list,
							$force_srpm || $force_kernel_srpm,
							$prompt_srpm);
		$need_build |= $build_kernel_srpms{"${kernel_srpm}_build_kernel"};
	}

	if (! $need_build) {
		return 0;	# success
	}

	# -------------------------------------------------------------------------
	# check OS dependencies for all srpms which we will build
	my $dep_error = 0;

	NormalPrint "Checking OS Dependencies needed for builds...\n";

	foreach my $srpm ( @delta_kernel_srpms ) {
		next if ( ! $build_kernel_srpms{"${srpm}_build_kernel"} );

		VerbosePrint("check dependencies for $srpm\n");
		if (check_kbuild_dependencies($K_VER, $srpm )) {
			DebugPrint "$srpm kbuild dependency failure\n";
			$dep_error = 1;
		}
		if (check_rpmbuild_dependencies($srpm)) {
			DebugPrint "$srpm rpmbuild dependency failure\n";
			$dep_error = 1;
		}
	}

	if ($dep_error) {
		NormalPrint "ERROR - unable to perform builds due to need for additional OS rpms\n";
		return 1;	# failure
	}

	# -------------------------------------------------------------------------
	# perform the builds
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};

	# use a different directory for BUILD_ROOT to limit conflict with OFED
	if ("$build_temp" eq "" ) {
		$build_temp = "/var/tmp/IntelOPA-DELTA";
	}
	my $BUILD_ROOT="$build_temp/build";
	my $RPM_DIR="$build_temp/DELTARPMS";
	my $resfileop = "replace";	# replace for 1st build, append for rest

	system("rm -rf ${build_temp}");

	# use a different directory for BUILD_ROOT to limit conflict with OFED
	if (0 != system("mkdir -p $BUILD_ROOT $RPM_DIR/BUILD $RPM_DIR/RPMS $RPM_DIR/SOURCES $RPM_DIR/SPECS $RPM_DIR/SRPMS")) {
		NormalPrint "ERROR - mkdir -p $BUILD_ROOT $RPM_DIR/BUILD $RPM_DIR/RPMS $RPM_DIR/SOURCES $RPM_DIR/SPECS $RPM_DIR/SRPMS FAILED\n";
		return 1;	# failure
	}

	foreach my $srpm ( @delta_kernel_srpms ) {
		VerbosePrint("process $srpm\n");

		next if (! $build_kernel_srpms{"${srpm}_build_kernel"});

		my $SRC_RPM = delta_srpm_file($srcdir, "$srpm*.src.rpm");

		# Deal with SLES renaming
		if ("$srpm" eq "kmod-ifs-kernel-updates"
			|| "$srpm" eq "ifs-kernel-updates-kmp-default") {
			$SRC_RPM = delta_srpm_file($srcdir, "ifs-kernel-updates*.src.rpm");
		}

		my $cmd = "rpmbuild --rebuild --define '_topdir $RPM_DIR'";
		$cmd .=		" --define 'dist  %{nil}'";
		$cmd .=		" --target $RPM_KERNEL_ARCH";
		# IFS - also set build_root so we can cleanup and avoid conflicts
		$cmd .=		" --buildroot '${BUILD_ROOT}'";
		$cmd .=		" --define 'build_root ${BUILD_ROOT}'";

	   	$cmd .=		" --define 'kver $K_VER'";

		$cmd .=		" $SRC_RPM";

		if (0 != run_build("$srcdir $SRC_RPM $RPM_KERNEL_ARCH $K_VER", "$srcdir", $cmd, "$resfileop")) {
			return 1;	# failure
		}
		$resfileop = "append";
		if ( $GPU_Install == 1 ) {
			delta_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmsdir/CUDA");
		} else {
			delta_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmsdir");
		}
	}

	if (! $debug) {
		system("rm -rf ${build_temp}");
	} else {
		LogPrint "Build remnants left in $BUILD_ROOT and $RPM_DIR\n";
	}

	return 0;	# success
}

# forward declarations
sub installed_delta_opa_stack();

# TBD - might not need any more
# return 0 on success, != 0 otherwise
sub uninstall_old_delta_rpms($$$)
{
	my $mode = shift();	# "user" or kernel rev or "firmware"
						# "any"- checks if any variation of package is installed
	my $verbosity = shift();
	my $message = shift();

	my $ret = 0;	# assume success
	my @packages = ();
	my @prev_release_rpms = ( "hfi1-psm-compat-devel","hfi1-psm","hfi1-psm-devel","hfi1-psm-debuginfo","libhfi1verbs","libhfi1verbs-devel", "ifs-kernel-updates" );

	if ("$message" eq "" ) {
		$message = "previous OFA Delta";
	}
	NormalPrint "\nUninstalling $message RPMs\n";

	# uninstall all present version OFA rpms, just to be safe
	foreach my $i ( "mpiRest", reverse(@Components) ) {
  		next if (! $ComponentInfo{$i}{'IsOFA'});
		@packages = (@packages, @{ $ComponentInfo{$i}{'DebugRpms'}});
		@packages = (@packages, @{ $ComponentInfo{$i}{'UserRpms'}});
		@packages = (@packages, @{ $ComponentInfo{$i}{'FirmwareRpms'}});
		@packages = (@packages, @{ $ComponentInfo{$i}{'KernelRpms'}});
	}

	# workaround LAM and other MPIs usng mpi-selector
	# we uninstall mpi-selector separately and ignore failures for its uninstall
	my @filtered_packages = ();
	my @rest_packages = ();
	foreach my $i ( @packages ) {
		if (scalar(grep /^$i$/, (@filtered_packages, @rest_packages)) > 0) {
			# skip, already in list
		} elsif ( "$i" eq "mpi-selector" ) {
			@rest_packages = (@rest_packages, "$i");
		} else {
			@filtered_packages = (@filtered_packages, "$i");
		}
	}

	# get rpms we are going to remove
	my $installed_mpi_rpms = "";
	foreach my $rpm (@{ $ComponentInfo{'mpiRest'}{'UserRpms'}}) {
		if (rpm_is_installed($rpm, "user")) {
			$installed_mpi_rpms .= " $rpm";
		}
	}

	$ret ||= rpm_uninstall_all_list_with_options($mode, " --nodeps ", $verbosity, @filtered_packages);

	if ($ret == 0 && $installed_mpi_rpms ne "") {
		NormalPrint "The following MPIs were removed because they are built from a different version of OPA.\n";
		NormalPrint "Please rebuild these MPIs if you still need them.\n  $installed_mpi_rpms\n";
	}

# TBD can simply handle mpi_selector component specially at end as a component
	# ignore errors uninstalling mpi-selector
	if (scalar(@rest_packages) != 0) {
		if (rpm_uninstall_all_list_with_options($mode, " --nodeps ", $verbosity, @rest_packages) && ! $ret) {
			NormalPrint "The previous errors can be ignored\n";
		}
	}

	if (rpm_uninstall_all_list_with_options($mode, " --nodeps ", $verbosity, @prev_release_rpms) && ! $ret) {
		NormalPrint "The previous errors can be ignored\n";
	}

	if ( $ret ) {
		NormalPrint "Unable to uninstall $message RPMs\n";
	}
	return $ret;
}


# TBD - might not need anymore
# remove any old stacks or old versions of the stack
# this is necessary before doing builds to ensure we don't use old dependent
# rpms
sub uninstall_prev_versions()
{
	if (! installed_delta_opa_stack) {
		return 0;
	} elsif (! comp_is_uptodate('opa_stack')) { # all delta_comp same version
		if (0 != uninstall_old_delta_rpms("any", "silent", "previous OFA DELTA")) {
			return 1;
		}
	}
	return 0;
}

sub has_version_delta()
{
	# check both current and old location
	# NOTE: When we upgrade/downgrade, the previous installation may have
	# different BASE_DIR. Ideally we shall figure out previous installation
	# location and then check version_delta there. For now, we are doing it
	# in static way by checking all possible locations.
	return -e "$BASE_DIR/version_delta"
		|| -e "$OLD_BASE_DIR/version_delta";
}

sub media_version_delta()
{
	# all OFA components at same version as opa_stack
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};
	return `cat "$srcdir/Version"`;
}

sub delta_save_autostart()
{
	foreach my $comp ( @Components ) {
  		next if (! $ComponentInfo{$comp}{'IsOFA'});
  		if ($ComponentInfo{$comp}{'HasStart'}
			&& $ComponentInfo{$comp}{'StartupScript'} ne "") {
			$delta_autostart_save{$comp} = comp_IsAutostart2($comp);
		} else {
			$delta_autostart_save{$comp} = 0;
		}
	}
}

sub delta_restore_autostart($)
{
	my $comp = shift();

	if ( $delta_autostart_save{$comp} ) {
		comp_enable_autostart2($comp, 1);
	} else {
  		if ($ComponentInfo{$comp}{'HasStart'}
			&& $ComponentInfo{$comp}{'StartupScript'} ne "") {
			comp_disable_autostart2($comp, 1);
		}
	}
}

# makes sure needed OFA components are already built, builts/rebuilds as needed
# called for every delta component's preinstall, noop for all but
# first OFA component in installing_list
sub preinstall_delta($$$)
{
	my $comp = shift();			# calling component
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	# ignore non-delta components at start of installing_list
	my @installing = split /[[:space:]]+/, $installing_list;
	while (scalar(@installing) != 0
			&& ("$installing[0]" eq ""
  				|| ! $ComponentInfo{$installing[0]}{'IsOFA'} )) {
		shift @installing;
	}
	# now, only do work if $comp is the 1st delta component in installing_list
	if ("$comp" eq "$installing[0]") {
		delta_save_autostart();
		init_delta_info($CUR_OS_VER);

# TBD - do we really need this any more?
		# Before we do any builds make sure old stacks removed so we don't
		# build against the wrong version of dependent rpms
		if (0 != uninstall_prev_versions()) {
			return 1;
		}
		print_separator;
		my $version=media_version_delta();
		chomp $version;
		printf("Preparing OFA $version $DBG_FREE for Install...\n");
		LogPrint "Preparing OFA $version $DBG_FREE for Install for $CUR_OS_VER\n";
		return build_delta("$install_list", "$installing_list", "$CUR_OS_VER",0,"",$OFED_force_rebuild);
	} else {
		return 0;
	}
}

# ==========================================================================
# OFA DELTA generic routines

# OFA has a single start script but controls which ULPs are loaded via
# entries in $OPA_CONFIG (rdma.conf)
# change all StartupParams for given delta component to $newvalue
sub delta_comp_change_opa_conf_param($$)
{
	my $comp=shift();
	my $newvalue=shift();

	VerbosePrint("edit /$OPA_CONFIG $comp StartUp set to '$newvalue'\n");
	foreach my $p ( @{ $ComponentInfo{$comp}{'StartupParams'} } ) {
		change_opa_conf_param($p, $newvalue);
	}
}

# generic functions to handle autostart needs for delta components with
# more complex rdma.conf based startup needs.  These assume opa_stack handles
# the actual startup script.  Hence these focus on the rdma.conf parameters
# determine if the given capability is configured for Autostart at boot
sub IsAutostart_delta_comp2($)
{
	my $comp = shift();	# component to check
	my $WhichStartup = $ComponentInfo{$comp}{'StartupScript'};
	my $ret = $WhichStartup eq "" ? 1 : IsAutostart($WhichStartup);	# just to be safe, test this too

	# to be true, all parameters must be yes
	foreach my $p ( @{ $ComponentInfo{$comp}{'StartupParams'} } ) {
			$ret &= ( read_opa_conf_param($p, "") eq "yes");
	}
	return $ret;
}
sub autostart_desc_delta_comp($)
{
	my $comp = shift();	# component to describe
	my $WhichStartup = $ComponentInfo{$comp}{'StartupScript'};
	if ( "$WhichStartup" eq "" ) {
		return "$ComponentInfo{$comp}{'Name'}"
	} else {
		return "$ComponentInfo{$comp}{'Name'} ($WhichStartup)";
	}
}
# enable autostart for the given capability
sub enable_autostart_delta_comp2($)
{
	my $comp = shift();	# component to enable
	#my $WhichStartup = $ComponentInfo{$comp}{'StartupScript'};

	#opa_stack handles this: enable_autostart($WhichStartup);
	delta_comp_change_opa_conf_param($comp, "yes");
}
# disable autostart for the given capability
sub disable_autostart_delta_comp2($)
{
	my $comp = shift();	# component to disable
	#my $WhichStartup = $ComponentInfo{$comp}{'StartupScript'};

	delta_comp_change_opa_conf_param($comp, "no");
}

# helper to determine if we need to reinstall due to parameter change
sub need_reinstall_delta_comp($$$)
{
	my $comp = shift();
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	if (! comp_is_uptodate('opa_stack')) { # all delta_comp same version
		# on upgrade force reinstall to recover from uninstall of old rpms
		return "all";
	} else {
		return "no";
	}
}

# OFA has all the drivers in a single RPM.  This function installs that RPM
# ==========================================================================
# OFA Delta opa_stack installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_opa_stack()
{
	# opa_stack is tricky, there are multiple parameters.  We just test
	# the things we control here, if user has edited rdma.conf they
	# could end up with startup still disabled by having disabled all
	# the individual HCA drivers
	return IsAutostart_delta_comp2("opa_stack");
}
sub autostart_desc_opa_stack()
{
	return autostart_desc_delta_comp('opa_stack');
}
# enable autostart for the given capability
sub enable_autostart2_opa_stack()
{
	enable_autostart("opa");
}
# disable autostart for the given capability
sub disable_autostart2_opa_stack()
{
	disable_autostart("opa");
}

sub get_rpms_dir_opa_stack($)
{
	my $package = shift();
	return get_rpms_dir_delta($package)
}

sub available_opa_stack()
{
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};
# TBD better checks for available?
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_delta_opa_stack()
{
	if ( "$CUR_VENDOR_VER" eq "ES72" ) {
		return ( has_version_delta()
				&& (rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER)
				    || rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)));
	} elsif ( "$CUR_VENDOR_VER" eq "ES73" ) {
		return ( has_version_delta()
				&& rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES74" ) {
		if ($HFI2_INSTALL) {
			return ( has_version_delta()
			      && rpm_is_installed("hfi2", $CUR_OS_VER));
		} else {
			return ( has_version_delta()
			      && rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
		}
	} elsif ( "$CUR_VENDOR_VER" eq "ES8" ) {
		return ( has_version_delta()
				&& rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES76" ) {
		return ( has_version_delta()
				&& rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES75" ) {
		return ( has_version_delta()
				&& rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq 'ES122' ) {
		return ( has_version_delta()
				&& rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq 'ES123' ) {
		return ( has_version_delta()
				&& rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq 'ES124' ) {
		return ( has_version_delta()
				&& rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq 'ES15' ) {
		return ( has_version_delta()
				&& rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq 'ES151' ) {
		return ( has_version_delta()
				&& rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} else {
		return 0;
	}
}

sub installed_opa_stack()
{
	return (installed_delta_opa_stack);
}

# only called if installed_opa_stack is true
sub installed_version_opa_stack()
{
	if ( -e "$BASE_DIR/version_delta" ) {
		return `cat $BASE_DIR/version_delta`;
	} else {
		return 'NONE';
	}
}

# only called if available_opa_stack is true
sub media_version_opa_stack()
{
	return media_version_delta();
}

# return 0 on success, 1 on failure
sub run_uninstall($$$)
{
	my $stack = shift();
	my $cmd = shift();
	my $cmdargs = shift();

	if ( "$cmd" ne "" && -e "$cmd" ) {
		NormalPrint "\nUninstalling $stack: $cmd $cmdargs\n";
		if (0 != system("yes | $cmd $cmdargs")) {
			NormalPrint "Unable to uninstall $stack\n";
			return 1;
		}
	}
	return 0;
}

# return 0 on success, !=0 on failure
sub build_opa_stack($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild

	init_delta_info($osver);

	# We can't remove old ib stack because we need them for build
	# OFA_DELTA. More importantly, we don't know where are the rpms.

	# Before we do any builds make sure old stacks removed so we don't
	# build against the wrong version of dependent rpms
	#if (0 != uninstall_prev_versions()) {
	#	return 1;
	#}

	return build_delta("@Components", "@Components", $osver, $debug,$build_temp,$force);
}

sub need_reinstall_opa_stack($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_delta_comp('opa_stack', $install_list, $installing_list));
}

sub check_os_prereqs_opa_stack
{
	return rpm_check_os_prereqs("opa_stack", "any");
}

sub preinstall_opa_stack($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("opa_stack", $install_list, $installing_list);
}

sub install_opa_stack($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};

	print_comp_install_banner('opa_stack');

	# do the following before we set any system env vars
	if ( $Default_SameAutostart ) {
		# set env vars to -1 for SRP and SRPT, so we keep the autostart option
		setup_env("OPA_SRP_LOAD", -1);
		setup_env("OPA_SRPT_LOAD", -1);
	}

	#override the udev permissions.
	install_udev_permissions("$srcdir/config");

	# setup environment variable so that RPM can configure limits conf
	setup_env("OPA_LIMITS_CONF", 1);
	# so setting up envirnment to install driver for this component. actual install is done by rpm
	setup_env("OPA_INSTALL_CALLER", 1);

	# Check $BASE_DIR directory ...exist
	check_config_dirs();
	check_dir("/usr/lib/opa");

	prompt_opa_conf_param('ARPTABLE_TUNING', 'Adjust kernel ARP table size for large fabrics', "y", 'OPA_ARPTABLE_TUNING');

	install_comp_rpms('opa_stack', " -U --nodeps ", $install_list);
	# rdma.conf values not directly associated with driver startup are left
	# untouched delta rpm install will keep existing value

	# prevent distro's open IB from loading
	#add_blacklist("ib_mthca");
	#add_blacklist("ib_ipath");
	disable_distro_ofed();

	# Take care of the configuration files for srptools
	check_rpm_config_file("/etc/srp_daemon.conf");
	check_rpm_config_file("/etc/logrotate.d/srp_daemon");
	check_rpm_config_file("/etc/rsyslog.d/srp_daemon.conf");

	need_reboot();
	$ComponentWasInstalled{'opa_stack'}=1;
}

sub postinstall_opa_stack($$)
{
	my $old_conf = 0;	# do we have an existing conf file
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	if ( -e "/$OPA_CONFIG" ) {
		if (0 == system("cp /$OPA_CONFIG /$OPA_CONFIG-save")) {
			$old_conf=1;
		}
	}

	# adjust rdma.conf autostart settings
	foreach my $c ( @Components )
	{
  		next if (! $ComponentInfo{$c}{'IsOFA'});
		if ($install_list !~ / $c /) {
			# disable autostart of uninstalled components
			# opa_stack is at least installed
			delta_comp_change_opa_conf_param($c, "no");
		} else {
			# retain previous setting for components being installed
			# set to no if initial install
			# TBD - should move this to rpm .spec file
			# the rpm might do this for us, repeat just to be safe
			foreach my $p ( @{ $ComponentInfo{$c}{'StartupParams'} } ) {
				my $old_value = "";
				if ( $old_conf ) {
					$old_value = read_opa_conf_param($p, "/$OPA_CONFIG-save");
				}
				if ( "$old_value" eq "" ) {
					$old_value = "no";
				}
				change_opa_conf_param($p, $old_value);
			}
		}
	}

	delta_restore_autostart('opa_stack');
}

# Do we need to do any of the stuff???
sub uninstall_opa_stack($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_comp_uninstall_banner('opa_stack');
	remove_blacklist("ib_qib");

	# allow open IB to load
	#remove_blacklist("ib_mthca");
	#remove_blacklist("ib_ipath");

	# the following is the work around for STL-14186. hfi2 rpm doesn't call depmod on uninstall, so we need to do it
	# here. After we resolve the issue, we shall remove the following 3 lines
	if ($HFI2_INSTALL) {
		set_run_depmod();
	}

	# uninstall mpiRest
	uninstall_comp_rpms('mpiRest', ' --nodeps ', $install_list, $uninstalling_list, 'verbose');

	# If we call uninstall intel_hfi before this function, OPA_INSTALL_CALLER will be 1.
	# So we reset it back to zero here to ensure we will skip check in opa-scripts
	setup_env("OPA_INSTALL_CALLER", 0);
	uninstall_comp_rpms('opa_stack', ' --nodeps ', $install_list, $uninstalling_list, 'verbose');
	remove_limits_conf;

	remove_udev_permissions;

	system("rm -rf $BASE_DIR/version_delta");
	system("rm -rf $OLD_BASE_DIR/version_delta");
	system("rm -rf /usr/lib/opa/.comp_delta.pl");
	system "rmdir /usr/lib/opa 2>/dev/null";	# remove only if empty
	system "rmdir $BASE_DIR 2>/dev/null";	# remove only if empty
	system "rmdir $OPA_CONFIG_DIR 2>/dev/null";	# remove only if empty

	need_reboot();
	$ComponentWasInstalled{'opa_stack'}=0;
}
# ==========================================================================
# intel_hfi installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_intel_hfi()
{
    return (! is_blacklisted('hfi1'));
}
sub autostart_desc_intel_hfi()
{
    return autostart_desc_delta_comp('intel_hfi');
}
# enable autostart for the given capability
sub enable_autostart2_intel_hfi()
{
	if (! IsAutostart2_intel_hfi()) {
		remove_blacklist('hfi1');
		rebuild_ramdisk();
	}
}
# disable autostart for the given capability
sub disable_autostart2_intel_hfi()
{
	if (IsAutostart2_intel_hfi()) {
		add_blacklist('hfi1');
		rebuild_ramdisk();
	}
}

sub get_rpms_dir_intel_hfi($)
{
	my $package = shift();
	return get_rpms_dir_delta($package)
}

sub available_intel_hfi()
{
    my $srcdir=$ComponentInfo{'intel_hfi'}{'SrcDir'};
        return (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_intel_hfi()
{
	if (!rpm_is_installed("hfi1-firmware", "firmware")) {
		return 0;
	}

	if ( "$CUR_VENDOR_VER" eq "ES72" || "$CUR_VENDOR_VER" eq "ES73" ) {
        return (rpm_is_installed("libhfi1", "user")
		     && has_version_delta()
		     && rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES122" ) {
		return (rpm_is_installed("libhfi1verbs-rdmav2", "user")
		     && has_version_delta()
		     && rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES74" ) {
		if ($HFI2_INSTALL) {
			return (has_version_delta()
			     && rpm_is_installed("hfi2", $CUR_OS_VER));
		} else {
			return (has_version_delta()
			     && rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
		}
	} elsif ( "$CUR_VENDOR_VER" eq "ES75" ) {
		return (has_version_delta()
		     && rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES76" ) {
		return (has_version_delta()
		     && rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES8" ) {
		return (has_version_delta()
		     && rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES123" ) {
		return (has_version_delta()
		     && rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES124" ) {
		return (has_version_delta()
		     && rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES15" ) {
		return (has_version_delta()
		     && rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES151" ) {
		return (has_version_delta()
		     && rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} else {
		return 0;
	}
}

# only called if installed_intel_hfi is true
sub installed_version_intel_hfi()
{
	if ( -e "$BASE_DIR/version_delta" ) {
		return `cat $BASE_DIR/version_delta`;
	} else {
		return "";
	}
}

# only called if available_intel_hfi is true
sub media_version_intel_hfi()
{
    return media_version_delta();
}

sub build_intel_hfi($$$$)
{
    my $osver = shift();
    my $debug = shift();    # enable extra debug of build itself
    my $build_temp = shift();       # temp area for use by build
    my $force = shift();    # force a rebuild
    return 0;       # success
}

sub need_reinstall_intel_hfi($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled

    return (need_reinstall_delta_comp('intel_hfi', $install_list, $installing_list));
}

sub preinstall_intel_hfi($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled

    return preinstall_delta("intel_hfi", $install_list, $installing_list);
}

sub install_intel_hfi($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled

    print_comp_install_banner('intel_hfi');
    setup_env("OPA_INSTALL_CALLER", 1);

    install_comp_rpms('intel_hfi', " -U --nodeps ", $install_list);

    need_reboot();
    $ComponentWasInstalled{'intel_hfi'}=1;
}

sub postinstall_intel_hfi($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled
    delta_restore_autostart('intel_hfi');

	rebuild_ramdisk();
}

sub uninstall_intel_hfi($$)
{
    my $install_list = shift();     # total that will be left installed when done
    my $uninstalling_list = shift();        # what items are being uninstalled

    print_comp_uninstall_banner('intel_hfi');
    setup_env("OPA_INSTALL_CALLER", 1);
    uninstall_comp_rpms('intel_hfi', ' --nodeps ', $install_list, $uninstalling_list, 'verbose');
    need_reboot();
    $ComponentWasInstalled{'intel_hfi'}=0;
    remove_blacklist('hfi1');
    rebuild_ramdisk();
}

sub check_os_prereqs_intel_hfi
{
	return rpm_check_os_prereqs("intel_hfi", "any");
}

# ==========================================================================
# OFA opa_stack development installation

sub get_rpms_dir_opa_stack_dev($)
{
	my $package = shift();
	return get_rpms_dir_delta($package)
}

sub available_opa_stack_dev()
{
	my $srcdir=$ComponentInfo{'opa_stack_dev'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_opa_stack_dev()
{
	return (rpm_is_installed("ifs-kernel-updates-devel", $CUR_OS_VER) &&
		has_version_delta());
}

# only called if installed_opa_stack_dev is true
sub installed_version_opa_stack_dev()
{
	if ( -e "$BASE_DIR/version_delta" ) {
		return `cat $BASE_DIR/version_delta`;
	} else {
		return "";
	}
}

# only called if available_opa_stack_dev is true
sub media_version_opa_stack_dev()
{
	return media_version_delta();
}

sub build_opa_stack_dev($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_opa_stack_dev($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_delta_comp('opa_stack_dev', $install_list, $installing_list));
}

sub preinstall_opa_stack_dev($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("opa_stack_dev", $install_list, $installing_list);
}

sub install_opa_stack_dev($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_comp_install_banner('opa_stack_dev');
	install_comp_rpms('opa_stack_dev', " -U --nodeps ", $install_list);

	$ComponentWasInstalled{'opa_stack_dev'}=1;
}

sub postinstall_opa_stack_dev($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#delta_restore_autostart('opa_stack_dev');
}

sub uninstall_opa_stack_dev($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_comp_uninstall_banner('opa_stack_dev');
	uninstall_comp_rpms('opa_stack_dev', ' --nodeps ', $install_list, $uninstalling_list, 'verbose');
	$ComponentWasInstalled{'opa_stack_dev'}=0;
}

# ==========================================================================
# OFA delta_ipoib installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_delta_ipoib()
{
	return IsAutostart_delta_comp2('delta_ipoib') || IsAutostart("ipoib");
}
sub autostart_desc_delta_ipoib()
{
	return autostart_desc_delta_comp('delta_ipoib');
}
# enable autostart for the given capability
sub enable_autostart2_delta_ipoib()
{
	enable_autostart_delta_comp2('delta_ipoib');
}
# disable autostart for the given capability
sub disable_autostart2_delta_ipoib()
{
	disable_autostart_delta_comp2('delta_ipoib');
	if (Exist_ifcfg("ib")) {
		print "$ComponentInfo{'delta_ipoib'}{'Name'} will autostart if ifcfg files exists\n";
		print "To fully disable autostart, it's recommended to also remove related ifcfg files\n";
		Remove_ifcfg("ib_ipoib","$ComponentInfo{'delta_ipoib'}{'Name'}","ib");
	}
}

sub get_rpms_dir_delta_ipoib($)
{
	my $package = shift();
	return get_rpms_dir_delta($package)
}

sub available_delta_ipoib()
{
	my $srcdir=$ComponentInfo{'delta_ipoib'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_delta_ipoib()
{
	return 1;
}

# only called if installed_delta_ipoib is true
sub installed_version_delta_ipoib()
{
	if ( -e "$BASE_DIR/version_delta" ) {
		return `cat $BASE_DIR/version_delta`;
	} else {
		return 'Unknown';
	}
}

# only called if available_delta_ipoib is true
sub media_version_delta_ipoib()
{
	return media_version_delta();
}

sub build_delta_ipoib($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_delta_ipoib($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_delta_comp('delta_ipoib', $install_list, $installing_list));
}

sub preinstall_delta_ipoib($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("delta_ipoib", $install_list, $installing_list);
}

sub install_delta_ipoib($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_comp_install_banner('delta_ipoib');
	install_comp_rpms('delta_ipoib', " -U --nodeps ", $install_list);

	# bonding is more involved, require user to edit to enable that
	Config_ifcfg(1,"$ComponentInfo{'delta_ipoib'}{'Name'}","ib", "$FirstIPoIBInterface",1);
	check_network_config;
	#Config_IPoIB_cfg;
	need_reboot();
	$ComponentWasInstalled{'delta_ipoib'}=1;
}

sub postinstall_delta_ipoib($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	delta_restore_autostart('delta_ipoib');
}

sub uninstall_delta_ipoib($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_comp_uninstall_banner('delta_ipoib');
	uninstall_comp_rpms('delta_ipoib', ' --nodeps ', $install_list, $uninstalling_list, 'verbose');
	Remove_ifcfg("ib_ipoib","$ComponentInfo{'delta_ipoib'}{'Name'}","ib");
	need_reboot();
	$ComponentWasInstalled{'delta_ipoib'}=0;
}

# ==========================================================================
# OFA DELTA mpi-selector installation

sub get_rpms_dir_mpi_selector($)
{
	my $package = shift();
	return get_rpms_dir_delta($package)
}

sub available_mpi_selector()
{
	my $srcdir=$ComponentInfo{'mpi_selector'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_mpi_selector()
{
	return (rpm_is_installed("mpi-selector", "user")
			&& has_version_delta());
}

# only called if installed_mpi_selector is true
sub installed_version_mpi_selector()
{
	if ( -e "$BASE_DIR/version_delta" ) {
		return `cat $BASE_DIR/version_delta`;
	} else {
		return "";
	}
}

# only called if available_mpi_selector is true
sub media_version_mpi_selector()
{
	return media_version_delta();
}

sub build_mpi_selector($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_mpi_selector($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_delta_comp('mpi_selector', $install_list, $installing_list));
}

sub check_os_prereqs_mpi_selector
{
	return rpm_check_os_prereqs("mpi_selector", "any");
}

sub preinstall_mpi_selector($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("mpi_selector", $install_list, $installing_list);
}

sub install_mpi_selector($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_comp_install_banner('mpi_selector');
	install_comp_rpms('mpi_selector', " -U --nodeps ", $install_list);

	$ComponentWasInstalled{'mpi_selector'}=1;
}

sub postinstall_mpi_selector($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#delta_restore_autostart('mpi_selector');
}

sub uninstall_mpi_selector($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_comp_uninstall_banner('mpi_selector');
	uninstall_comp_rpms('mpi_selector', '', $install_list, $uninstalling_list, 'verbose');
	$ComponentWasInstalled{'mpi_selector'}=0;
}

# ==========================================================================
# OFA sandiashmem for gcc installation

sub get_rpms_dir_sandiashmem($)
{
	my $package = shift();
	return get_rpms_dir_delta($package)
}

sub available_sandiashmem()
{
	my $srcdir=$ComponentInfo{'sandiashmem'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_sandiashmem()
{
	return ((rpm_is_installed("sandia-openshmem_gcc_hfi", "user")
			&& has_version_delta()));
}

# only called if installed_openshmem is true
sub installed_version_sandiashmem()
{
	if ( -e "$BASE_DIR/version_delta" ) {
		return `cat $BASE_DIR/version_delta`;
	} else {
		return 'Unknown';
	}
}

# only called if available_openshmem is true
sub media_version_sandiashmem()
{
	return media_version_delta();
}

sub build_sandiashmem($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_sandiashmem($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_delta_comp('sandiashmem', $install_list, $installing_list));
}

sub preinstall_sandiashmem($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("sandiashmem", $install_list, $installing_list);
}

sub install_sandiashmem($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_comp_install_banner('sandiashmem');

	# make sure any old potentially custom built versions of sandiashmem  are uninstalled
	rpm_uninstall_list2("any", " --nodeps ", 'silent', @{ $ComponentInfo{'sandiashmem'}{'UserRpms'}});
	my $rpmfile = rpm_resolve(delta_rpms_dir() . "/sandia-openshmem_gcc_hfi", "any");
	if ( "$rpmfile" ne "" && -e "$rpmfile" ) {
		# try to remove existed SOS folders. Note this is the workaround for PR 144408. We shall remove the following
		# code after we resolve the PR.
		my @installed_pkgs = glob("/usr/shmem/gcc/sandia-openshmem-*-hfi");
		if (@installed_pkgs) {
			foreach my $installed_pkg (@installed_pkgs) {
				if (GetYesNo ("Remove $installed_pkg directory?", "y")) {
					LogPrint "rm -rf $installed_pkg\n";
					system("rm -rf $installed_pkg");
				}
			}
		}
	}

	install_comp_rpms('sandiashmem', " -U --nodeps ", $install_list);

	$ComponentWasInstalled{'sandiashmem'}=1;
}

sub postinstall_sandiashmem($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#delta_restore_autostart('sandiashmem');
}

sub uninstall_sandiashmem($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_comp_uninstall_banner('sandiashmem');
	uninstall_comp_rpms('sandiashmem', ' --nodeps ', $install_list, $uninstalling_list, 'verbose');
	$ComponentWasInstalled{'sandiashmem'}=0;
}

sub check_os_prereqs_sandiashmem
{
	return rpm_check_os_prereqs("sandiashmem", "user");
}

# ==========================================================================
# OFA delta_debug installation

# this is an odd component.  It consists of the debuginfo files which
# are built and identified in DebugRpms in other components.  Installing this
# component installs the debuginfo files for the installed components.
# uninstalling this component gets rid of all debuginfo files.
# uninstalling other components will get rid of individual debuginfo files
# for those components

sub get_rpms_dir_delta_debug($)
{
	my $package = shift();
	return get_rpms_dir_delta($package)
}

sub available_delta_debug()
{
	my $srcdir=$ComponentInfo{'delta_debug'}{'SrcDir'};
	return (( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS")
			&& ( "$CUR_DISTRO_VENDOR" ne "SuSE" && rpm_will_build_debuginfo()));
}

sub installed_delta_debug()
{
	return (rpm_is_installed("libibumad-debuginfo", "user") || rpm_is_installed("libpsm2-debuginfo", "user")
			&& has_version_delta());
}

# only called if installed_delta_debug is true
sub installed_version_delta_debug()
{
	if ( -e "$BASE_DIR/version_delta" ) {
		return `cat $BASE_DIR/version_delta`;
	} else {
		return "";
	}
}

# only called if available_delta_debug is true
sub media_version_delta_debug()
{
	return media_version_delta();
}

sub build_delta_debug($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_delta_debug($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	my $reins = need_reinstall_delta_comp('delta_debug', $install_list, $installing_list);
	if ("$reins" eq "no" ) {
		# if delta components with DebugRpms have been added we need to reinstall
		# this component.  Note uninstall for individual components will
		# get rid of associated debuginfo files
		foreach my $comp ( @Components ) {
			# TBD can remove IsOFA test, the only
			# components with DebugRpms are for OFA delta debug
  			next if (! $ComponentInfo{$comp}{'IsOFA'});
			if ( " $installing_list " =~ / $comp /
				 && 0 != scalar(@{ $ComponentInfo{$comp}{'DebugRpms'}})) {
				return "this";
			}
		}
		
	}
	return $reins;
}

sub preinstall_delta_debug($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("delta_debug", $install_list, $installing_list);
}

sub install_delta_debug($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	my @list;

	print_comp_install_banner('delta_debug');
	install_comp_rpms('delta_debug', " -U --nodeps ", $install_list);

	# install DebugRpms for each installed component
	foreach my $comp ( @Components ) {
		# TBD can remove IsOFA test, the only
		# components with DebugRpms are for OFA delta debug
  		next if (! $ComponentInfo{$comp}{'IsOFA'});
		if ( " $install_list " =~ / $comp / ) {
			install_comp_rpm_list("$comp", "user", " -U --nodeps ",
							@{ $ComponentInfo{$comp}{'DebugRpms'}});
		}
	}

	$ComponentWasInstalled{'delta_debug'}=1;
}

sub postinstall_delta_debug($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#delta_restore_autostart('delta_debug');
}

sub uninstall_delta_debug($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_comp_uninstall_banner('delta_debug');

	uninstall_comp_rpms('delta_debug', ' --nodeps ', $install_list, $uninstalling_list, 'verbose');
	# uninstall debug rpms for all components
	foreach my $comp ( reverse(@Components) ) {
  		next if (! $ComponentInfo{$comp}{'IsOFA'});
		rpm_uninstall_list2("any", " --nodeps ", 'verbose',
					 @{ $ComponentInfo{$comp}{'DebugRpms'}});
	}
	$ComponentWasInstalled{'delta_debug'}=0;
}

# ==========================================================================
# OFA DELTA ibacm installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_ibacm()
{
	return IsAutostart_delta_comp2('ibacm');
}
sub autostart_desc_ibacm()
{
	return autostart_desc_delta_comp('ibacm');
}
# enable autostart for the given capability
sub enable_autostart2_ibacm()
{
	enable_autostart($ComponentInfo{'ibacm'}{'StartupScript'});
}
# disable autostart for the given capability
sub disable_autostart2_ibacm()
{
	disable_autostart($ComponentInfo{'ibacm'}{'StartupScript'});
}

sub get_rpms_dir_ibacm($)
{
	my $package = shift();
	return get_rpms_dir_delta($package)
}

sub available_ibacm()
{
	my $srcdir=$ComponentInfo{'ibacm'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_ibacm()
{
	return (rpm_is_installed("ibacm", "user")
			&& has_version_delta());
}

# only used on RHEL72, for other distros ibacm is only a SubComponent
# only called if installed_ibacm is true
sub installed_version_ibacm()
{
	if ( -e "$BASE_DIR/version_delta" ) {
		return `cat $BASE_DIR/version_delta`;
	} else {
		return "";
	}
}

# only called if available_ibacm is true
sub media_version_ibacm()
{
	return media_version_delta();
}

sub build_ibacm($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ibacm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_delta_comp('ibacm', $install_list, $installing_list));
}

sub preinstall_ibacm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("ibacm", $install_list, $installing_list);
}

sub install_ibacm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_comp_install_banner('ibacm');
	install_comp_rpms('ibacm', " -U --nodeps ", $install_list);

	$ComponentWasInstalled{'ibacm'}=1;
}

sub postinstall_ibacm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	delta_restore_autostart('ibacm');
}

sub uninstall_ibacm($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_comp_uninstall_banner('ibacm');

	uninstall_comp_rpms('ibacm', ' --nodeps ', $install_list, $uninstalling_list, 'verbose');
	$ComponentWasInstalled{'ibacm'}=0;
}

sub check_os_prereqs_ibacm
{
	return rpm_check_os_prereqs("ibacm", "user");
}

# ------------------------------------------------------------------
# # subroutines for rdma-ndd component
# # -----------------------------------------------------------------
sub installed_rdma_ndd()
{
        return (rpm_is_installed("infiniband-diags", "user"));
}

sub enable_autostart2_rdma_ndd()
{
        system "systemctl enable rdma-ndd >/dev/null 2>&1";
}

sub disable_autostart2_rdma_ndd()
{
        system "systemctl disable rdma-ndd >/dev/null 2>&1";
}

sub IsAutostart2_rdma_ndd()
{
        my $status = `systemctl is-enabled rdma-ndd`;
        if ( $status eq "disabled\n" || $status eq "" ){
                return 0;
        }
        else{
                return 1;
        }
}

# ------------------------------------------------------------------
# # subroutines for delta_srp component
# # -----------------------------------------------------------------
sub installed_delta_srp()
{
	if ( -f "/etc/rdma/rdma.conf" ) {
                return 1;
        }
        else {
                return 0;
        }

}

sub enable_autostart2_delta_srp()
{
	change_opa_conf_param("SRP_LOAD", "yes");
}

sub disable_autostart2_delta_srp()
{
	change_opa_conf_param("SRP_LOAD", "no");
}

sub IsAutostart2_delta_srp()
{
	my $status = read_opa_conf_param("SRP_LOAD", "");
	if ( $status eq "yes" ){
                return 1;
        }
        else{
                return 0;
        }
}

# ------------------------------------------------------------------
# # subroutines for delta_srpt component
# # -----------------------------------------------------------------
sub installed_delta_srpt()
{
	if ( -f "/etc/rdma/rdma.conf" ) {
		return 1;
	}
	else {
		return 0;
	}
}

sub enable_autostart2_delta_srpt()
{
	change_opa_conf_param("SRPT_LOAD", "yes");
}

sub disable_autostart2_delta_srpt()
{
	change_opa_conf_param("SRPT_LOAD", "no");
}

sub IsAutostart2_delta_srpt()
{
	my $status = read_opa_conf_param("SRPT_LOAD", "");
        if ( $status eq "yes" ){
                return 1;
        }
        else{
                return 0;
        }
}

