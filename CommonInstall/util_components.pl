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

# =========================================================================
# Component installation state

# constants for component state/action tracking during menus
my $State_Uninstall = 0;
my $State_Install = 1;
my $State_UpToDate = 2;
my $State_DoNotInstall = 3;
my $State_DoNotAutoInstall = 4;	# valid only in DefaultInstall, must explicitly select

sub disable_components(@);
my %comp_prereq_hash;
# indicate if given state reflects a component which will be on the system
# after processing the present menu selections
# states:
# $State_Uninstall - make sure not on system (Uninstall)
# $State_Install - install on system (Install/Upgrade/Re-Install)
# $State_UpToDate - leave as is on system (Installed/Up To Date)
# $State_DoNotInstall - leave as is not on system (Do Not Install)
sub will_be_on_system($)
{
	my $state = shift();

	return ($state == $State_Install || $state == $State_UpToDate);
}

# state returned:
# $State_Uninstall - make sure not on system (Uninstall)
# $State_Install - install on system (Install/Upgrade/Re-Install)
# $State_UpToDate - leave as is on system (Installed/Up To Date)
# $State_DoNotInstall - leave as is not on system (Do Not Install)
sub setstate($$$$)
{
	my $isinstalled = shift();   
	my $available   = shift();
	my $isupgrade   = shift();
	my $defaultstate= shift();	# default if available and ! installed

	if (! $available && $isinstalled && !$isupgrade){ return $State_UpToDate; }
	if (! $available && ! $isinstalled )			{ return $State_DoNotInstall; }
	if (! $available )				                { return $State_Uninstall; }
	if ($isinstalled && !$isupgrade)				{ return $State_UpToDate; }
	if ($isinstalled)								{ return $State_Install; }
	# available and not installed
	if ($defaultstate == $State_DoNotAutoInstall)	{ return $State_DoNotInstall; }
	return $defaultstate;	# typcally State_Install
}

# state returned:
# $State_Uninstall - make sure not on system (Uninstall)
# $State_Install - install on system (Install/Upgrade/Re-Install)
# $State_UpToDate - leave as is on system (Installed/Up To Date)
# $State_DoNotInstall - leave as is not on system (Do Not Install)
sub shiftstate($$$$)
{
	my $state       = shift();
	my $isinstalled = shift();   
	my $available   = shift();
	my $isupgrade   = shift();

	if (! $available && $isinstalled && ! $isupgrade) {
		return ($state==$State_UpToDate)?$State_Uninstall:$State_UpToDate;
	}
	if (! $available && ! $isinstalled ) {
		return ($state==$State_DoNotInstall)?$State_Uninstall:$State_DoNotInstall;
	}
	if (! $available ) {
		return $State_Uninstall;
	}
	if ($isinstalled && !$isupgrade) {
		return ($state==$State_UpToDate)?$State_Install
				:($state==$State_Install)?$State_Uninstall
				:$State_UpToDate;
	}
	if ($isinstalled ) {
		return ($state==$State_Install)?$State_Uninstall:$State_Install;
	}
	# available and not installed
	return ($state==$State_Uninstall)?$State_Install
			:($state==$State_Install)?$State_DoNotInstall:$State_Uninstall;
}

sub InstallStateToStr($)
{
	my $state = shift();

	if ($state == $State_Uninstall) {
		return "Uninstall";
	} elsif ($state == $State_Install) {
		return "Install";
	} elsif ($state == $State_UpToDate) {
		return "UpToDate";
	} elsif ($state == $State_DoNotInstall) {
		return "DoNotInstall";
	} elsif ($state == $State_DoNotAutoInstall) {
		return "DoNotAutoInstall";
	} else {
		return "INVALID";
	}
}

# caller has output 24 characters, we have 55 left to play with
# (avoid using last column so don't autowrap terminal)
sub printInstallState($$$$)
{
	my $installed = shift();
	my $uninstall = shift();
	my $boldmessage = shift();
	my $message   = shift();

	if ($uninstall )
	{
		print   RED, "[  Uninstall  ] ", RESET;
	} elsif ($installed ) {
		print GREEN, "[  Installed  ] ", RESET;
	} else {
		print        "[Not Installed] ";
	}
	if ("$message" ne "" && $boldmessage) {
		print BOLD, RED "$message", RESET, "\n";
	} else {
		print "$message\n";
	}
	return;
}

# states:
# $State_Uninstall - make sure not on system (Uninstall)
# $State_Install - install on system (Install/Upgrade/Re-Install)
# $State_UpToDate - leave as is on system (Installed/Up To Date)
# $State_DoNotInstall - leave as is not on system (Do Not Install)
sub printInstallAvailableState($$$$$$)
{
	my $installed = shift();
	my $available = shift();
	my $isupgrade = shift();
	my $state     = shift();
	my $boldmessage = shift();
	my $message   = shift();

	if ( $state == $State_Uninstall )
	{
		print RED, "[  Uninstall  ]", RESET;
	} elsif ( $state == $State_Install) {
		if ($installed)
		{           
			if ( $isupgrade ) 
			{       
				print GREEN, "[   Upgrade   ]", RESET;
			} else {
				print "[ Re-Install  ]";
			}
		} else {
			print           "[   Install   ]";
		}
	} elsif ($state == $State_UpToDate) {
		print "[ Up To Date  ]";
	} elsif ($state == $State_DoNotInstall) {
		print "[Don't Install]";
	}

	if ( $available )
	{
		print "[Available] ";
	} else {
		print BOLD, RED "[Not Avail] ", RESET;
	}
	if ("$message" ne "" && $boldmessage) {
		print BOLD, RED "$message", RESET, "\n";
	} else {
		print "$message\n";
	}

	return;
}

# =======================================================================
# Component processing

# function must be supplied if any components set ComponentInfo{}{HasFirmware}

# these variables must be initialized by main program to control the
# data driven component menus and functions in this section
my $Default_Install=0;	# -a option used to select default install of All
my $Default_Upgrade=0;	# -U option used to select default upgrade of installed
my $Default_Uninstall=0;	# -u option used to select default uninstall of All
my $Default_SameAutostart=0; # -n option used to default install, preserving autostart values
my $Default_CompInstall=0;	# -i option used to select default install of Default_Components
my $Default_CompUninstall=0;	# -e option used to select default uninstall of Default_Components
my %Default_Components = ();		# components selected by -i or -e
my $Skip_FirmwareUpgrade=1;	# -f option used to select skipping firmware upgrade
my $Default_Autostart=0;	# -s option used to select default autostart of All
my $Default_DisableAutostart=0; # -D option used to select default disabling of autostart for Default_DisabledComponents
my %Default_DisabledComponents = ();		# components selected by -D
my $Default_EnableAutostart=0; # -E option used to select default enabling of autostart for Default_EnabledComponents
my %Default_EnabledComponents = ();		# components selected by -E
my $Default_ShowCompInfo=0; # -c option used to select default component information display of Default_Components

	# Names of supported install components
	# must be listed in depdency order such that prereqs appear 1st
my @Components = ();

# Sub components for autostart processing
my @SubComponents = ();

# components/subcomponents for autostart processing
my @AutostartComponents = ();

	# an additional "special" component which is always installed when
	# we install/upgrade anything and which is only uninstalled when everything
	# else has been uninstalled.  Typically this will be the opaconfig
	# file and related files absolutely required by it (such as wrapper_version)
my $WrapperComponent = "";

# This provides more detailed information about each Component in @Components
# since hashes are not retained in the order specified, we still need
# @Components and @SubComponents to control install and menu order
# Only items listed in @Components and @SubComponents are considered for install
# As such, this may list some components which are N/A to the selected distro
# Fields are as follows:
#	Name => full name of component/subcomponent for prompts
# 	DefaultInstall => default installation (State_DoNotInstall,
# 					  State_DoNotAutoInstall or State_Install)
#					  used only when available and ! installed
#	SrcDir => directory name within install media for component
# 	PreReq => other components which are prereqs of a given
#				component/subcomponent
#				Need space before and after each component name to
#				facilitate compares so that compares do not mistake names
#				such as mpidev for mpi
#	CoReq =>  other components which are coreqs of a given component
#				Note that CoReqs can also be listed as prereqs to provide
#				install verification as each component is installed,
#				however prereqs should only refer to items earlier in the list
#				Need space before and after each component name to
#				facilitate compares so that compares do not mistake names
#				such as mpidev for mpi
#	Hidden => component should not be shown in menus/prompts
#			  used for hidden PreReq.  Hidden components can't HasStart
#			  nor HasFirmware
#	Disabled =>  components/subcomponents which are disabled from installation
#	IsOFA =>  is an in-distro OFA component we upgrade (excludes MPI)
#	KernelRpms => kernel rpms for given component, in dependency order
#				These are rpms which are kernel version specific and
#				will have kernel uname -r in rpm package name.
#				For a given distro a separate version of each of these rpms
#				may exist per kernel.  These are always architecture dependent
#				Note KernelRpms are always installed before FirmwareRpms and
#				UserRpms
#	FirmwareRpms => firmware rpms for given component, in dependency order
#				These are rpms which are not kernel specific.  For a given
#				distro a single version of each of these rpms will
#				exist per distro/arch combination.  In most cases they will
#				be architecture independent (noarch).
#				These are rpms which are installed in user space but
#				ultimately end up in hardware such as HFI firmware, TMM firmware
#				BIOS firmware, etc.
#	UserRpms => user rpms for given component, in dependency order
#				These are rpms which are not kernel specific.  For a given
#				distro a single version of each of these rpms will
#				exist per distro/arch combination.  Some of these may
#				be architecture independent (noarch).
#	DebugRpms => user rpms for component which should be installed as part
#				of delta_debug component.
#	HasStart =>  components/subcomponents which have autostart capability
#	DefaultStart =>  should autostart default to Enable (1) or Disable (0)
#				Not needed/ignored unless HasStart=1
# 	StartPreReq => other components/subcomponents which must be autostarted
#				before autostarting this component/subcomponent
#				Not needed/ignored unless HasStart=1
#				Need space before and after each component name to
#				facilitate compares so that compares do not mistake names
#				such as mpidev for mpi
#	StartComponents => components/subcomponents with start for this component
#				if a component has a start script 
#				list the component as a subcomponent of itself
#	StartupScript => name of startup script which controls startup of this
#				component
#	StartupParams => list of parameter names in $OPA_CONFIG which control
#				startup of this component (set to yes/no values)
#	HasFirmware =>  components which need HCA firmware update after installed
#
# Note both Components and SubComponents are included in the list below.
# Components require all fields be supplied
# SubComponents only require the following fields:
#	Name, PreReq (should reference only components), HasStart, StartPreReq,
#	DefaultStart, and optionally StartupScript, StartupParams
#	Also SubComponents only require the IsAutostart2_X, autostart_desc_X,
#	enable_autostart2_X, disable_autostart2_X and installed_X functions.
#	Typically installed_X for a SubComponent will simply call installed_X for
#	the component which contains it.
my %ComponentInfo = ();
	# translate from startup script name to component/subcomponent name
my %StartupComponent = ();
	# has component been installed since last configured autostart
my %ComponentWasInstalled = ();

# constants for autostart functions $configure argument
my $Start_Unspecified=0;
my $Start_NoStart=1;
my $Start_Start=2;

sub ShowComponents(;$)
{
	my $print_param = shift || \*STDOUT; #STDOUT as default parameter
	foreach my $comp ( @Components )
	{
		if (! $ComponentInfo{$comp}{'Hidden'}) {
			print $print_param " $comp";
		}
	}
	print $print_param "\n";
}

sub ShowCompInfo(;$)
{
	my $print_param = shift || \*STDOUT; #STDOUT as default parameter
	print $print_param "[\n";
	my $first_comp = 1;
	my $first_item = 1;
	foreach my $comp ( @Components )
	{
		if ( $Default_Components{$comp} ) {
			if ($first_comp == 1) {
				$first_comp = 0;
				print $print_param " {\n";
			} else {
				print $print_param ",\n {\n";
			}
			print $print_param "  \"id\": \"$comp\",\n";
			my $description = $ComponentInfo{$comp}{'Name'};
			print $print_param "  \"description\": \"$description\",\n";
			if ( $comp eq "delta_debug" ) {
				print $print_param "  \"error\": \"Not apply. We ship debug rpms in meta pkg <component>_debuginfo.\"\n";
				print $print_param " },\n";
				next;
			}
			if (comp_is_available($comp)) {
				print $print_param "  \"available\": \"yes\",\n";
			} else {
				print $print_param "  \"available\": \"no\",\n";
			}
			my $full_ver = comp_media_version($comp);
			my ($version, $release) = GetVerRel($full_ver);
			print $print_param "  \"version\": \"$version\",\n";
			print $print_param "  \"release\": \"$release\",\n";
			print $print_param "  \"prereqs\": [\n";
			$first_item = 1;
			my @reqs = split(/ /, $ComponentInfo{$comp}{'PreReq'});
			foreach my $req (@reqs) {
				# ignore req that is not a component. This is for the case we do not
				# have mpi_selector for SLES
				if ( $req ne '' && grep( /^$req$/, @Components) ) {
					my $reqver = comp_media_version($req);
					my ($version, $release) = GetVerRel($reqver);
					$reqver = "$version-$release";
					if ($first_item == 1) {
						$first_item = 0;
						print $print_param "   {\"id\": \"$req\", \"version\": \"$reqver\"}";
					} else {
						print $print_param ",\n   {\"id\": \"$req\", \"version\": \"$reqver\"}";
					}
				}
			}
			print $print_param "\n  ],\n";
			@reqs = split(/ /, $ComponentInfo{$comp}{'CoReq'});
			print $print_param "  \"coreqs\": [\n";
			$first_item = 1;
			foreach my $req (@reqs) {
				# ignore req that is not a component. This is for the case we do not
				# have mpi_selector for SLES
				if ( $req ne '' && grep( /^$req$/, @Components) ) {
					my $reqver = comp_media_version($req);
					my ($version, $release) = GetVerRel($reqver);
					$reqver = "$version-$release";
					if ($first_item == 1) {
						$first_item = 0;
						print $print_param "   {\"id\": \"$req\", \"version\": \"$reqver\"}";
					} else {
						print $print_param ",\n   {\"id\": \"$req\", \"version\": \"$reqver\"}";
					}
				}
			}
			print $print_param "\n  ],\n";
			if ( $comp eq "mpisrc" ) {
				print $print_param "  \"srcrpms\": {\n";
				print $print_param "   \"dest\": \"/usr/src/opa/MPI/\",\n";
				my $srcdir=$ComponentInfo{'mpisrc'}{'SrcDir'};
				print $print_param "   \"source\": \"$srcdir\",\n";
				print $print_param "   \"resources\": [\n";
				$first_item = 1;
				foreach my $srpm ( "mvapich2", "openmpi", "mpitests" ) {
					my $srpmfile = file_glob("$srcdir/${srpm}-*.src.rpm");
					if ( "$srpmfile" ne "" ) {
						my $file = my_basename($srpmfile);
						if ($first_item == 1) {
							$first_item = 0;
							print $print_param "    \"$file\"";
						} else {
							print $print_param ",\n    \"$file\"";
						}
					}
				}
				print $print_param "\n   ]\n";
				print $print_param "  },\n";
				print $print_param "  \"tools\": {\n";
				print $print_param "   \"dest\": \"/usr/src/opa/MPI/\",\n";
				print $print_param "   \"source\": \"$srcdir\",\n";
				print $print_param "   \"resources\": [\n";
				print $print_param "    \"do_build\",\n";
				print $print_param "    \"do_mvapich2_build\",\n";
				print $print_param "    \"do_openmpi_build\"\n";
				print $print_param "   ]\n";
				print $print_param "  },\n";
				print $print_param "  \"misc\": [\n";
				print $print_param "   {\"dest\": \"/usr/src/opa/MPI/.version\", \"source\": \"$srcdir\", \"resource\": \"version\"}\n";
				print $print_param "  ]\n";
			} else {
				print $print_param "  \"userrpms\": [\n";
				ShowRpmList($print_param, "   ", "user", @{$ComponentInfo{$comp}{'UserRpms'}});
				print $print_param "\n  ],\n";
				print $print_param "  \"kernelrpms\": [\n";
				ShowRpmList($print_param, "   ", $CUR_OS_VER, @{$ComponentInfo{$comp}{'KernelRpms'}});
				print $print_param "\n  ],\n";
				print $print_param "  \"firmwarerpms\": [\n";
				ShowRpmList($print_param, "   ", "firmware", @{$ComponentInfo{$comp}{'FirmwareRpms'}});
				print $print_param "\n  ],\n";
				print $print_param "  \"debugrpms\": [\n";
				ShowRpmList($print_param, "   ", "any", @{$ComponentInfo{$comp}{'DebugRpms'}});
				print $print_param "\n  ]\n";
			}

			print $print_param " }";
		}
	}
	print $print_param "\n]\n";
}

sub GetVerRel($)
{
	my $full_ver = shift();
	my ($version, $release) = split('-', $full_ver, 2);
	if ("$release" eq "") {
		# assume the version is in expected format, and it shall be.
		my @segs = split('\.', $version);
		my $last = scalar(@segs)-1;
		$version = join('.', @segs[0..3]);
		$release = join('.', @segs[4..$last]);
	}
	return ($version, $release);
}

sub ShowRpmList($$$$@)
{
	my $print_param = shift();
	my $prefix = shift();
	my $mode = shift();
	my @rpms = @_;
	my $rpmsdir = "";
	my $first_item = 1;
	foreach my $rpm (@rpms) {
		if ( $GPU_Install && -d file_glob("./repos/OPA_PKGS_CUDA") ) {
			$rpmsdir=file_glob("./repos/OPA_PKGS_CUDA/RPMS");
		} else {
			$rpmsdir=file_glob("./repos/OPA_PKGS/RPMS");
		}
		my $rpm_file = rpm_resolve("$rpmsdir/$rpm", $mode);
		if ( $rpm_file ne '' ) {
			my $version = rpm_query_version_release($rpm_file);
			my $epoch = rpm_query_attr($rpm_file, "EPOCH");
			if ("$epoch" ne "") {
				$version = "$epoch:$version";
			}
			if ($first_item == 1) {
				$first_item = 0;
				print $print_param "$prefix\{\"id\": \"$rpm\", \"version\": \"$version\"}";
			} else {
				print $print_param ",\n$prefix\{\"id\": \"$rpm\", \"version\": \"$version\"}";
			}
		} else {
			DebugPrint "Not found $rpm";
		}
	}
}

# return 1 if $comp has a prereq of $prereq, 0 otherwise
sub comp_has_prereq_of($$)
{
	my($comp) = shift();
	my($prereq) = shift();

	if ($ComponentInfo{$comp}{'PreReq'} =~ / $prereq /) {
		return 1;
	} else {
		return 0;
	}
}

# return 1 if $comp has a coreq of $coreq, 0 otherwise
sub comp_has_coreq_of($$)
{
	my($comp) = shift();
	my($coreq) = shift();

	if ($ComponentInfo{$comp}{'CoReq'} =~ / $coreq /) {
		return 1;
	} else {
		return 0;
	}
}

# return 1 if $comp has a prereq or coreq of $req, 0 otherwise
sub comp_has_req_of($$)
{
	my($comp) = shift();
	my($req) = shift();

	return (comp_has_prereq_of($comp, $req) || comp_has_coreq_of($comp, $req));
}
# return 1 if $comp has a prereq of $prereq, 0 otherwise
sub comp_has_startprereq_of($$)
{
	my($comp) = shift();
	my($prereq) = shift();

	if ($ComponentInfo{$comp}{'StartPreReq'} =~ / $prereq /) {
		return 1;
	} else {
		return 0;
	}
}

# package is supplied since for a few packages (such as GPU Direct specific
# packages) we may pick a different directory based on package name
sub comp_get_rpms_dir($$)
{
	my $comp = shift();
	my $package = shift();
	return eval "get_rpms_dir_$comp('$package')";
}

# for comp_is_available we specially handle the case of function not found
# in which case eval returns ""
# all other component functions against install media will have called
# comp_is_available first, so they need not be protected by this test
sub comp_is_available($)
{
	my $comp = shift();
	my $rc;

	if ($ComponentInfo{$comp}{'Disabled'}) {
		return 0;
	}
	$rc = eval "available_$comp()";
	if ( "$rc" eq "" ) {
		return 0;	# $comp not loaded
	} else {
		return $rc;
	}
}

# return version string for component on install media
# only valid if comp_is_available is TRUE
sub comp_media_version($)
{
	my $comp = shift();

	my $result = eval "media_version_$comp()";
	chomp $result;
	return $result;
}

# for comp_is_installed we specially handle the case of function not found
# in which case eval returns ""
# all other component functions against system itself will have called
# comp_is_installed first, so they need not be protected by this test
sub comp_is_installed($)
{
	my $comp = shift();
	my $rc;

	$rc = eval "installed_$comp()";
	if ( "$rc" eq "" ) {
		return 0;	# $comp not loaded
	} else {
		return $rc;
	}
}

# return version string for component on system
# only valid if comp_is_installed is TRUE
sub comp_installed_version($)
{
	my $comp = shift();

	my $result = eval "installed_version_$comp()";
	chomp $result;
	return $result;
}

# return TRUE if component is installed and up to date
# return FALSE if not installed, or not same version as media
# typically only called when component is available, returns
# FALSE if component not available
sub comp_is_uptodate($)
{
	my $comp = shift();

	# safety check
	if (! comp_is_available("$comp") ) {
		return 0;
	}
	if (! comp_is_installed("$comp")) {
		return 0;
	}
	# available and installed, compare versions
	return (comp_installed_version($comp) eq comp_media_version($comp));
}

# check if all prereqs of $comp have been installed
# returns: 1 - all ok, 0 - some prereqs missing
sub check_prereqs($)
{
	my($comp) = shift();

	foreach my $c ( @Components )
	{
		if (comp_has_prereq_of($comp, $c) && ! comp_is_installed("$c") )
		{
			NormalPrint "Unable to Install $ComponentInfo{$comp}{'Name'}, prereq $ComponentInfo{$c}{'Name'} not installed\n";
			HitKeyCont;
			return 0;
		}
	}
	return 1;
}

# build processing for the given component
# returns 0 on success, 1 on failure
# only valid if comp_is_available is TRUE
sub comp_build($$$$$)
{
	my $comp = shift();
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild

	LogPrint "Building $ComponentInfo{$comp}{'Name'} for $osver using '$build_temp'\n";
	return eval "build_$comp('$osver',$debug,'$build_temp',$force)";
}

# special hook for build processing for the given component within OFED build
# returns 1 if build should be done, 0 if not
# only valid for subset of components and if comp_is_available is TRUE
sub comp_ofed_need_build($$$$)
{
	my $comp = shift();
	my $osver = shift();
	my $force = shift();	# should we force build
	my $prompt = shift();	# should we prompt user about build

	return eval "ofed_need_build_$comp('$osver',$force,$prompt)";
}

# special hook for build processing for the given component within OFED build
# returns 0 on success, >0 on failure
# only valid for subset of components and if comp_is_available is TRUE
sub comp_ofed_check_build_dependencies($$)
{
	my $comp = shift();
	my $osver = shift();

	return eval "ofed_check_build_dependencies_$comp('$osver')";
}

# special hook for build processing for the given component within OFED build
# returns 0 on success, 1 on failure
# only valid for subset of components and if comp_is_available is TRUE
sub comp_ofed_build($$$$)
{
	my $comp = shift();
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build

	LogPrint "Building $ComponentInfo{$comp}{'Name'} for $osver using '$build_temp'\n";
	return eval "ofed_build_$comp('$osver',$debug,'$build_temp')";
}

# before preinstall determine if present settings will require all
# installed components to be reinstalled
# returns "all" if need to reinstall all UpToDate components
# returns "this" if need to reinstall this component even if up to date
# returns "no" if no need to reinstall this component even if up to date
# only valid if comp_is_available is TRUE
sub comp_need_reinstall($$$)
{
	my $comp = shift();
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return eval "need_reinstall_$comp('$install_list', '$installing_list')";
}

# check presence of OS prereqs for the given component
# returns 0 on success, 1 on failure
# only valid if comp_is_available is TRUE
# for backward compatibility with old comp.pl files
# we specially handle the case of function not found
# in which case eval returns ""
sub comp_check_os_prereqs($)
{
	my $comp = shift();
	my $rc;

	DebugPrint "Checking OS Prereqs for $ComponentInfo{$comp}{'Name'}\n";
	$rc = eval "check_os_prereqs_$comp()";
	DebugPrint "Done: rc='$rc'\n";
	if ( "$rc" eq "" ) {
		return 0;	# $comp not loaded
	} else {
		return $rc;
	}
}

# preinstall processing for the given component
# returns 0 on success, 1 on failure
# only valid if comp_is_available is TRUE
sub comp_preinstall($$$)
{
	my $comp = shift();
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return eval "preinstall_$comp('$install_list', '$installing_list')";
}

# install the given component
# only valid if comp_is_available is TRUE
sub comp_install($$$)
{
	my $comp = shift();
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	# sanity check, should not get here if false
	if (! comp_is_available($comp) ) {
		$exit_code = 1;
		return;
	}
	print_separator;
	# this will catch failures in install of prereqs
	if (! check_prereqs($comp)) {
		$exit_code = 1;
		return;
	}

	eval "install_$comp('$install_list', '$installing_list')";
}

# postinstall processing for the given component
# only valid if comp_is_available is TRUE
sub comp_postinstall($$$)
{
	my $comp = shift();
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	eval "postinstall_$comp('$install_list', '$installing_list')";
}

# uninstall the given component
# typically only called if comp_is_available is TRUE
# or comp_is_installed is TRUE
# however could be called when not available and not installed in which
# case could be an empty function, in which case we can simply ignore
# eval's "" return
sub comp_uninstall($$$)
{
	my $comp = shift();
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_separator;
	if ($ComponentInfo{$comp}{'HasStart'}) {
		disable_components(@{ $ComponentInfo{$comp}{'StartComponents'} });
	}
	eval "uninstall_$comp('$install_list', '$uninstalling_list')";
}

# determine if the given component is configured for Autostart at boot
sub comp_IsAutostart2($)
{
	my $comp = shift();

	return eval "IsAutostart2_$comp()";
}
# get autostart desc for the given component
sub comp_autostart_desc($)
{
	my $comp = shift();

	my $desc = eval "autostart_desc_$comp()";
	if ( "$desc" eq "" ) {
		$desc = $ComponentInfo{$comp}{'Name'};
	}
	return $desc;
}
# enable autostart for the given component
sub comp_enable_autostart2($$)
{
	my $comp = shift();
	my $quiet = shift();

	if ( ! comp_IsAutostart2($comp) == 1 )
	{
		if (! $quiet) {
			my $desc = comp_autostart_desc($comp);
			NormalPrint "Enabling autostart for $desc\n";
		}
	}
	eval "enable_autostart2_$comp()";
}
# disable autostart for the given component
sub comp_disable_autostart2($$)
{
	my $comp = shift();
	my $quiet = shift();

	if ( comp_IsAutostart2($comp) == 1 )
	{
		if (! $quiet) {
			my $desc = comp_autostart_desc($comp);
			NormalPrint "Disabling autostart for $desc\n";
		}
	}

	eval "disable_autostart2_$comp()";
}

sub DumpComponents($)
{
	my $allow_install = shift();

	if (! DebugPrintEnabled() ) {
		return;
	}
	DebugPrint("\nComponents:\n");
	foreach my $comp ( @Components, "mpiRest" ) {
		DebugPrint("   $comp: '$ComponentInfo{$comp}{'Name'}' SrcDir: $ComponentInfo{$comp}{'SrcDir'}\n");
		DebugPrint("           DefaultInstall: ".InstallStateToStr($ComponentInfo{$comp}{'DefaultInstall'})."\n");
		DebugPrint("           PreReq: $ComponentInfo{$comp}{'PreReq'} CoReq: $ComponentInfo{$comp}{'CoReq'}\n");
		DebugPrint("           Hidden: $ComponentInfo{$comp}{'Hidden'} Disabled: $ComponentInfo{$comp}{'Disabled'} HasFirmware: $ComponentInfo{$comp}{'HasFirmware'} IsOFA: $ComponentInfo{$comp}{'IsOFA'}\n");
		DebugPrint("           KernelRpms: @{ $ComponentInfo{$comp}{'KernelRpms'}}\n");
		DebugPrint("           FirmwareRpms @{ $ComponentInfo{$comp}{'FirmwareRpms'}}\n");
		DebugPrint("           UserRpms: @{ $ComponentInfo{$comp}{'UserRpms'}}\n");
		DebugPrint("           DebugRpms: @{ $ComponentInfo{$comp}{'DebugRpms'}}\n");
		DebugPrint("           HasStart: $ComponentInfo{$comp}{'HasStart'} DefaultStart: $ComponentInfo{$comp}{'DefaultStart'}\n");
		DebugPrint("           StartPreReq: $ComponentInfo{$comp}{'StartPreReq'} StartComponents: @{ $ComponentInfo{$comp}{'StartComponents'}}\n");
		DebugPrint("           StartupScript: $ComponentInfo{$comp}{'StartupScript'} StartupParams: @{ $ComponentInfo{$comp}{'StartupParams'}}\n");
		if ($allow_install) {
			my $avail = comp_is_available($comp);
			DebugPrint("           IsAvailable: $avail");
			if ($avail) {
				DebugPrint("           MediaVersion: ".comp_media_version($comp));
			}
			DebugPrint("\n");
		}
		my $installed = comp_is_installed($comp);
		DebugPrint("           IsInstalled: $installed");
		if ($installed) {
			DebugPrint("           InstalledVersion: ".comp_installed_version($comp));
		}
		DebugPrint("\n");
	}

	DebugPrint("\nSubComponents:\n");
	foreach my $comp ( @SubComponents ) {
		DebugPrint("   $comp: '$ComponentInfo{$comp}{'Name'}'\n");
		DebugPrint("           PreReq: $ComponentInfo{$comp}{'PreReq'} CoReq: $ComponentInfo{$comp}{'CoReq'}\n");
		DebugPrint("           HasStart: $ComponentInfo{$comp}{'HasStart'}\n");
		DebugPrint("           StartPreReq: $ComponentInfo{$comp}{'StartPreReq'}\n");
		DebugPrint("           StartupScript: $ComponentInfo{$comp}{'StartupScript'} StartupParams: @{ $ComponentInfo{$comp}{'StartupParams'}}\n");
	}
}

# output standard banner for component install
# This function can be called within a components install_X function
sub print_comp_install_banner($)
{
	my $comp = shift();

	my $version=comp_media_version($comp);
	chomp $version;
	printf("Installing $ComponentInfo{$comp}{'Name'} $version $DBG_FREE...\n");
	LogPrint "Installing $ComponentInfo{$comp}{'Name'} $version $DBG_FREE for $CUR_DISTRO_VENDOR $CUR_VENDOR_VER $CUR_OS_VER\n";
}

# install the given list of rpms for a component
sub install_comp_rpm_list($$$@)
{
	my $comp = shift();
	my $mode = shift();	# "user" or kernel rev or "firmware"
	my $options = shift();	# additional rpm command options
	my @package_list = @_;	# package names

	my @rpmpath_list = ();

	foreach my $package ( @package_list )
	{
		my $rpmsdir=comp_get_rpms_dir($comp, $package);
		@rpmpath_list = ( @rpmpath_list, "$rpmsdir/$package" );
	}
	rpm_install_path_list_with_options($mode, "$options", @rpmpath_list);
}

# helper which does most of the work for installing rpms for a component
# This function can be called within a components install_X function
# installs KernelRpms, FirmwareRpms and UserRpms
# caller must handle any non-RPM files
sub install_comp_rpms($$$)
{
	my $comp = shift();
	my $options = shift();	# additional rpm command options
	my $install_list = shift();	# total that will be installed when done

	if (! $user_space_only ) {
		install_comp_rpm_list($comp, $CUR_OS_VER, "$options",
								@{ $ComponentInfo{$comp}{'KernelRpms'}} );
		install_comp_rpm_list($comp, "firmware", "$options",
								@{ $ComponentInfo{$comp}{'FirmwareRpms'}} );
	}
	install_comp_rpm_list($comp, "user", "$options",
								@{ $ComponentInfo{$comp}{'UserRpms'}} );
	# DebugRpms are installed as part of 'delta_debug' component
}

# output standard banner for component uninstall
# This function can be called within a components install_X function
sub print_comp_uninstall_banner($)
{
	my $comp = shift();

	NormalPrint("Uninstalling $ComponentInfo{$comp}{'Name'}...\n");
}

# helper which does most of the work for uinstalling rpms for a component
# This function can be called within a components uninstall_X function
# uninstalls KernelRpms, FirmwareRpms, UserRpms and DebugRpms
# caller must handle any non-RPM files
sub uninstall_comp_rpms($$$$$)
{
	my $comp = shift();
	# the rpm command options
	my $option = shift();
	my $install_list = shift();
	my $uninstalling_list = shift();
	my $verbosity = shift();

	# try to uninstall meta pkg if it exists
	my $metapkg = "opameta_$comp";
	rpm_uninstall_matches($metapkg, $metapkg, '', "");

	rpm_uninstall_list2("any", "$option", $verbosity,
					 @{ $ComponentInfo{$comp}{'DebugRpms'}});
	rpm_uninstall_list2("any", "$option", $verbosity,
					 @{ $ComponentInfo{$comp}{'UserRpms'}});
	if (! $user_space_only ) {
		rpm_uninstall_list2("any", "$option", $verbosity,
					 @{ $ComponentInfo{$comp}{'FirmwareRpms'}});
		rpm_uninstall_list2("any", "$option", $verbosity,
					 @{ $ComponentInfo{$comp}{'KernelRpms'}});
	}
}

# build @AutostartComponents listing all components and subcomponents which
# have autostart capability.  They are listed in prefered startup order
sub build_autostart_components_list()
{
	my $comp;
	# only need to do once
	if (scalar(@AutostartComponents) != 0 ) {
		return;
	}
	foreach $comp ( @Components )
	{
		if ( $ComponentInfo{$comp}{'HasStart'} )
		{
			# StartComponents will list Components and SubComponents which
			# have start capability
			@AutostartComponents = ( @AutostartComponents, @{ $ComponentInfo{$comp}{'StartComponents'} })
		}
	}
}

# disable autostart for a list of components and
# all components dependent on given component
sub disable_components(@)
{
	my @disabled = ( @_ );
	my %need_disable = ();
	my %dont_need_disable = ();
	my $comp;

	build_autostart_components_list;
	foreach $comp ( @disabled ) {
		$need_disable{$comp} = 1;
		DebugPrint("disable $comp explicitly\n");
	}
	my $done = 0;
	while ( ! $done) {
		$done=1;
		foreach $comp ( @disabled ) {
			# comp will be disabled, any which depend on it as a
			# startprereqs will also be disabled
			foreach my $c ( @AutostartComponents ) {
				if (comp_has_startprereq_of($c, $comp)
					&& ! $need_disable{$c} && ! $dont_need_disable{$c} ) {
					if (comp_is_installed("$c") ) {
						@disabled = ( @disabled, $c );
						$need_disable{$c}=1;
						DebugPrint("also disable $c\n");
						$done=0;
					} else {
						$dont_need_disable{$c}=1;
						DebugPrint("no need to disable $c\n");
					}
				}
			}
		}
	}
	foreach $comp ( reverse(@AutostartComponents) ) {
		foreach my $c ( @disabled ) {
			if ( "$comp" eq "$c" ) {
				DebugPrint("invoke disable $c\n");
				comp_disable_autostart2($c,0);
			}
		}
	}
}

# enable autostart for a list of components and
# all components they depend on
sub enable_components(@)
{
	my @enabled = ( @_ );
	my %need_enable = ();
	my $comp;

	build_autostart_components_list;
	foreach $comp ( @enabled ) {
		$need_enable{$comp} = 1;
		DebugPrint("enable $comp explicitly\n");
	}
	my $done = 0;
	while ( ! $done) {
		$done=1;
		foreach $comp ( @enabled ) {
			# comp will be enabled, make sure all its
			# startprereqs will also be enabled
			foreach my $c ( @AutostartComponents ) {
				if (comp_has_startprereq_of($comp, $c) && ! $need_enable{$c} ) {
					DebugPrint("also enable $c\n");
					@enabled = ( $c, @enabled );
					$need_enable{$c}=1;
					$done=0;
				}
			}
		}
	}
	foreach $comp ( @AutostartComponents ) {
		foreach my $c ( @enabled ) {
			if ( "$comp" eq "$c" ) {
				DebugPrint("invoke enable $c\n");
				comp_enable_autostart2($c,0);
			}
		}
	}
}

# run build for all components
sub build_all_components($$$$)
{
	my $osver = shift();	# Kernel OS Version to build for
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a full rebuild

	my $comp;
	my $ret = 0;

	foreach $comp ( @Components )
	{
		my $rc = comp_build($comp,$osver,$debug,$build_temp,$force);
		if (0 != $rc) {
			NormalPrint "Unable to Build $ComponentInfo{$comp}{'Name'}\n";
		}
		$ret |= $rc;
	}
	return $ret;
}

sub count_hidden_comps()
{
	my $i;
	my $ret=0;

	for($i=0; $i < scalar(@Components); $i++)
	{
		my $comp = $Components[$i];
		if ($ComponentInfo{$comp}{'Hidden'}) {
			$ret++;
		}
	}
	return $ret;
}

# convert a screen selection into a Components subscript, accounting for Hidden
sub get_comp_subscript($$$)
{
	my $firstline=shift();
	my $maxlines=shift();
	my $selection=shift();
	my $index=0;
	my $i;

	for ($i=0; $i < scalar(@Components); $i++)
	{
		my $comp = $Components[$i];
		if ($index >= $firstline && $index < $firstline+$maxlines) {
			if (! $ComponentInfo{$comp}{'Hidden'}) {
				if ($index - $firstline == $selection) {
					return $i;
				}
				$index++;
			}
		} else {
			if (! $ComponentInfo{$comp}{'Hidden'}) {
				$index++;
			}
		}
	}
	return $i	# invalid entry return 1 past number of Components
}

sub show_install_menu($)
{
	my $showversion = shift();	# should per comp versions be shown

	my $inp;
	my %available = ();
	my %installed = ();
	my %installed_version = ();
	my %media_version = ();
	my %isupgrade = ();
	my %installState = ();
	my %statusMessage = ();
	my %boldMessage = ();
	my $comp;
	my $i;
	my $newstate = $State_Uninstall;	# most recent state change for a comp
	my $firstline = 0;
	my $maxlines=14;
	my $num_hidden_comps = count_hidden_comps();

	print "Determining what is installed on system...\n";
	DumpComponents(1);
	foreach $comp ( @Components )
	{
		$available{$comp} = comp_is_available("$comp");
		$installed{$comp} = comp_is_installed("$comp");
		if ( $available{$comp} ) {
			$media_version{$comp} = comp_media_version($comp);
		} else {
			$media_version{$comp} = "";
		}
		if ( $installed{$comp} ) {
			$installed_version{$comp} = comp_installed_version($comp);
			$isupgrade{$comp}= ($installed_version{$comp} ne $media_version{$comp});
		} else {
			$installed_version{$comp} = "";
			$isupgrade{$comp}=1;
		}
		$installState{$comp}= setstate($installed{$comp}, $available{$comp},
							$isupgrade{$comp}, $ComponentInfo{$comp}{'DefaultInstall'});
	}
	# to avoid a subtle effect, mark as not available any items whose
	# coreqs are not available.  Such packagings should not
	# occur, but better to be safe
	foreach $comp ( @Components )
	{
		if (! $available{$comp}) {
			foreach my $c ( @Components )
			{
				if (comp_has_coreq_of($c, $comp) && $available{$c}) {
					$available{$c}=0;
				}
			}
		}
	}

	do {{
		if ( $Default_Install) {
			# setstate set UnInstall or DoNotInstall for those not available
			# force install for all available even if UpToDate
			# or ComponentInfo{comp}{'DefaultInstall'} is DoNotInstall
			foreach $comp ( @Components )
			{
				if ( $available{$comp}
					&& ($installState{$comp} != $State_DoNotInstall
						|| $ComponentInfo{$comp}{'DefaultInstall'} != $State_DoNotAutoInstall)) {
					$installState{$comp} = $State_Install;
				}
			}
			$newstate = $State_Install;
			$inp='P';
		} elsif ( $Default_Upgrade) {
			foreach $comp ( @Components )
			{
				if ( $installed{$comp} )
				{
					if ( $available{$comp} )
					{
						$installState{$comp} = $State_Install;
					} elsif ( ! $isupgrade{$comp} ) {
						$installState{$comp} = $State_UpToDate;
					} else {
						$installState{$comp} = $State_Uninstall; # we must uninstall
					}
				} else {
					$installState{$comp} = $State_DoNotInstall;
				}
			}
			$newstate = $State_Install;
			$inp='P';
		} elsif ( $Default_CompInstall) {
			foreach $comp ( @Components )
			{
				if ( $Default_Components{$comp} )
				{
					if ( $available{$comp} )
					{
						$installState{$comp} = $State_Install;
					} else {
						NormalPrint "Unable to install $ComponentInfo{$comp}{'Name'}, Not Available\n";
						# setstate provided a reasonable default action
					}
				} else {
					# setstate defaulted to install all available and not installed
					# for version updates re-install to be safe
					if ( $installState{$comp} == $State_Install
						&& ( ! $installed{$comp} || ! $isupgrade{$comp} ) )
					{
						# Do Not Install
						$installState{$comp} = $State_DoNotInstall;
					}
				}
			}
			$newstate = $State_Install;
			$inp='P';
		} else {
			system "clear";
			printf ("$BRAND OPA Install ($VERSION $DBG_FREE) Menu\n\n");
			my $screens = int((scalar(@Components) - $num_hidden_comps + $maxlines-1)/$maxlines);

			if($GPU_Install == 1) {
				printf ("Install GPU Direct components, ensure nvidia drivers + SDK are present \n\n");
			}
			if ($screens > 1 ) {
				printf ("Please Select Install Action (screen %d of $screens):\n",
							$firstline/$maxlines+1);
			} else {
				printf ("Please Select Install Action:\n");
			}
			my $index=0;
			for ($i=0; $i < scalar(@Components); $i++)
			{
				$comp = $Components[$i];
				if ($index >= $firstline && $index < $firstline+$maxlines) {
					if ($showversion && "$statusMessage{$comp}" eq "") {
						if ($available{$comp}) {
							$statusMessage{$comp} = $media_version{$comp};
							$boldMessage{$comp} = 0;
						} elsif($installed{$comp}) {
							$statusMessage{$comp} = $installed_version{$comp};
							$boldMessage{$comp} = 0;
						}
					}
					if (! $ComponentInfo{$comp}{'Hidden'}) {
						printf ("%x) %-20s", $index-$firstline, $ComponentInfo{$comp}{'Name'});
						printInstallAvailableState($installed{$comp},
										$available{$comp},
										$isupgrade{$comp},
										$installState{$comp},
										$boldMessage{$comp}, $statusMessage{$comp});
						$index++;
					}
				} elsif (! $ComponentInfo{$comp}{'Hidden'}) {
					$index++;
				}
			}

			printf ("\n");
			if ($screens > 1 ) {
				printf ("N) Next Screen\n");
			}
			printf (  "P) Perform the selected actions       I) Install All\n");
			printf (  "R) Re-Install All                     U) Uninstall All\n");
			printf (  "X) Return to Previous Menu (or ESC)\n");	

			$inp = getch();

			if ($inp =~ /[qQ]/ || $inp =~ /[xX]/ || ord($inp) == $KEY_ESC )
			{
				return;
			}

			# do not clear status messages when jump between screens
			if ($inp !~ /[nN]/ ) {
				%statusMessage = ();
				%boldMessage = ();
			}
		}

		if ($inp =~ /[nN]/ )
		{
			if (scalar(@Components) > $maxlines) {
				$firstline += $maxlines;
				if ($firstline >= scalar(@Components)) {
					$firstline=0;
				}
			}
		} elsif ($inp =~ /[rR]/ )
		{
			foreach $comp ( @Components )
			{
				if ( $installed{$comp} && $available{$comp}) {
					$installState{$comp} = $State_Install;
				}
			}
		} elsif ($inp =~ /[iI]/ )
		{
			foreach $comp ( @Components )
			{
				if ( $available{$comp} ) {
					$installState{$comp} = $State_Install;
				}
			}
			$newstate = $State_Install;
		} elsif ($inp =~ /[uU]/ )
		{
			foreach $comp ( @Components )
			{
				$installState{$comp} = $State_Uninstall;
			}
			$newstate = $State_Uninstall;
		} elsif ($inp =~ /[0123456789abcdefABCDEF]/) {
			my $value = hex($inp);
			my $index = get_comp_subscript($firstline, $maxlines, $value);
			if ( $value < $maxlines && $index < scalar(@Components)) {
				my $selected = $Components[$index];
				$installState{$selected} = shiftstate($installState{$selected},$installed{$selected},$available{$selected},$isupgrade{$selected});
				$newstate = $installState{$selected};
			}
		}

		if (! will_be_on_system($newstate)) {
			# something is being uninstalled or not installed
			# loop through a few times to catch all coreqs, prereqs
			# and coreqs of prereqs
			my $done = 0;	# we are done if we loop through with no changes
			while (! $done) {
				$done=1;
				foreach $comp ( @Components )
				{
					if (! will_be_on_system($installState{$comp})) {
						# comp will not be on system, make sure all its
						# pre/coreqs will also not be on system
						foreach my $c ( @Components )
						{
							if (comp_has_req_of($c, $comp)
								&& will_be_on_system($installState{$c})) {
								$statusMessage{$c}="requires $ComponentInfo{$comp}{'Name'}";
								$boldMessage{$c}=1;
								$installState{$c}=$installed{$c}?$State_Uninstall:$State_DoNotInstall;
								$done=0;
							}
						}
					}
					if ($ComponentInfo{$comp}{'Hidden'}) {
						# if no longer needed, also remove it
						my $needed = 0;	# is $comp needed
						foreach my $c ( @Components )
						{
							if (comp_has_req_of($c, $comp)
								&& will_be_on_system($installState{$c})) {
								$needed=1;
							}
						}
						if (! $needed && will_be_on_system($installState{$comp})) {
							$installState{$comp}=$installed{$comp}?$State_Uninstall:$State_DoNotInstall;
							$done=0;
						}
					}
				}
			}
		} elsif (will_be_on_system($newstate)) {
			# something is being installed or left on system
			# loop through a few times to catch all coreqs, prereqs
			# and coreqs of prereqs
			my $done = 0;	# we are done if we loop through with no changes
			my $forcedone = 0;	# used to break infinite loop in subtle issue below
			while (! $forcedone && ! $done) {
				$done=1;
				foreach $comp ( @Components )
				{
					if (will_be_on_system($installState{$comp})) {
						# comp will be on system, make sure all its
						# pre/coreqs will also be on system
						foreach my $c ( @Components )
						{
							if (comp_has_req_of($comp, $c)
								&& ! will_be_on_system($installState{$c})) {
								if (! $available{$c} ) {
									# pre/coreq $c not available, can't install comp
									$statusMessage{$comp}="requires $ComponentInfo{$c}{'Name'}";
									$boldMessage{$comp}=1;
									$installState{$comp}=$installed{$comp}?$State_Uninstall:$State_DoNotInstall;
									# this should not occur for coreqs
									# however there are subtle effects here.
									# If we install an item which has a prereq
									# whose prereq is not available we could get
									# stuck with an invalid combination since we
									# will have marked the 1st prereq installed,
									# found the 2nd prereq problem
									# unmarked the 1st prereq but still have
									# left the original item to be installed
									# bail to avoid infinite loop
									$forcedone=1;
									last;
								} else {
									# also install $c
									$statusMessage{$c}="needed by $ComponentInfo{$comp}{'Name'}";
									$boldMessage{$c}=1;
									$installState{$c}=$State_Install;
								}
								$done=0;
							}
						}
					}
					if ($ComponentInfo{$comp}{'Hidden'} && $available{$comp}
						&& will_be_on_system($installState{$comp})) {
						# if all prereqs being reinstalled also reinstall it
						my $reinstall = 1;	# no already installed dependents
						foreach my $c ( @Components )
						{
							if (comp_has_req_of($c, $comp)
								&& will_be_on_system($installState{$c})
								&& $installState{$c} != $State_Install) {
								$reinstall=0;
							}
						}
						if ($reinstall) {
							$installState{$comp}=$State_Install;
							# no need to force another loop, didn't change
							# will_be_on_system($comp)
						}
					}
				}
			}
		}

		if ($inp =~ /[pP]/) {
			# perform the install
			my $updateFirmware=0;

			# build a list of what will be installed after installation completes
			# use a space separate string so easier to pass and can be "grep'ed"
			my $install_list = "";
			my $installing_list = "";
			my $uninstalling_list = "";
			my $have_some_uptodate = 0;
			foreach $comp ( @Components )
			{
				if ($installState{$comp} == $State_UpToDate) {
					$have_some_uptodate = 1;
				}
				if ($installState{$comp} == $State_Install
					|| $installState{$comp} == $State_UpToDate) {
					$install_list .= " $comp ";
				}
				if ($installState{$comp} == $State_Install) {
					$installing_list .= " $comp ";
				}
				if ($installState{$comp} == $State_Uninstall) {
					$uninstalling_list .= " $comp ";
				}
			}

			# remove any alias pkgs if it exists. Meta pkgs will be handled in comp uninstall.
			rpm_uninstall_matches("opanode_", "opanode_", "", "");

			# first uninstall what will be removed, do this in reverse order
			# so dependency issues are avoided
			foreach $comp ( reverse(@Components) ) {
				if ($installState{$comp} == $State_Uninstall) {
					comp_uninstall($comp, $install_list, $uninstalling_list);
					$installed{$comp} = 0;
				}
			}
			if ( "$WrapperComponent" ne "" && "$installing_list" eq "" && "$install_list" eq "" ) {
				comp_uninstall($WrapperComponent, $install_list, $uninstalling_list);
			}

			if ($have_some_uptodate ) {
				# determine if some up to date components need to be reinstalled
				# for example due to a change in install prefix, install options ...
				# to determine this run need_reinstall for all components which
				# will be installed or are up to date.  Net result is we may change
				# some or all components in State_UpToDate to State_Install

				my $need_reinstall_all = 0;
				my $need_reinstall_some = 0;
				foreach $comp ( @Components )
				{
					if (($installState{$comp} == $State_Install
							|| $installState{$comp} == $State_UpToDate)
						&& $available{$comp} ) {
						my $reins = comp_need_reinstall($comp,$install_list,$installing_list);
						if ("$reins" eq "all") {
							VerbosePrint("$comp needs reinstall all\n");
							$need_reinstall_all = 1;
							last;
						} elsif ("$reins" eq "this") {
							if ($installState{$comp} == $State_UpToDate
								&& $available{$comp} ) {
								VerbosePrint("$comp needs reinstall\n");
								$installState{$comp} = $State_Install;
								$need_reinstall_some = 1;
							}
						}
					}
				}
				if ($need_reinstall_all || $need_reinstall_some) {
					if ($need_reinstall_all) {
						NormalPrint "INSTALL options require Reinstall of all UpToDate Components\n";
					} else {
						NormalPrint "Reinstall of some UpToDate Components is Required\n";
					}
					$installing_list  = "";	# recompute
					foreach $comp ( @Components )
					{
						if ($need_reinstall_all
							&& $installState{$comp} == $State_UpToDate
							&& $available{$comp} ) {
							$installState{$comp} = $State_Install;
						}
						if ($installState{$comp} == $State_Install) {
							$installing_list .= " $comp ";
						}
					}
				}
			}
			# check OS pre-reqs for all components which will be installed
			my $have_all_os_prereqs=1;
			foreach $comp ( @Components )
			{
				if ($installState{$comp} == $State_Install) {
					if (0 != comp_check_os_prereqs($comp)) {
						$have_all_os_prereqs=0;
						NormalPrint "Lacking OS Prereqs for $ComponentInfo{$comp}{'Name'}\n";
					}
				}
			}
			if (! $have_all_os_prereqs) {
				HitKeyCont;
				last;
			}

			# run pre-install for all components which will be installed
			# Reverse the order to avoid dependency issues
			foreach $comp ( reverse(@Components) )
			{
				if ($installState{$comp} == $State_Install) {
					if (0 != comp_preinstall($comp,$install_list,$installing_list)) {
						NormalPrint "Unable to Prepare $ComponentInfo{$comp}{'Name'} for Install\n";
						HitKeyCont;
						last;
					}
				}
			}

			# Now install components
			if ( "$WrapperComponent" ne "" && "$installing_list" ne "" ) {
				comp_install($WrapperComponent, $install_list, $installing_list);
			}
			foreach $comp ( @Components )
			{
				if ($installState{$comp} == $State_Install) {
					comp_install($comp, $install_list, $installing_list);
					if ( $ComponentInfo{$comp}{'HasFirmware'} ) {
						$updateFirmware=1;
					}
				}
			}

			# run post-install for all components which were installed
			foreach $comp ( @Components )
			{
				if ($installState{$comp} == $State_Install) {
					comp_postinstall($comp, $install_list, $installing_list);
				}
			}

			%installed = ();
			if ( $Default_Prompt ) {
				foreach $comp ( @Components )
				{
					$installed{$comp} = 0;
				}
				# limit autostart to newly installed components with a
				# default start.  Also include their start prereqs.
				# Note start prereqs are all inclusive, so we don't need to
				# find prereqs of prereqs
				foreach $comp ( @Components )
				{
					if ($installState{$comp} == $State_Install
						&& $ComponentInfo{$comp}{'HasStart'}
						&& ($Default_SameAutostart
							|| $ComponentInfo{$comp}{'DefaultStart'}) ) {
						$installed{$comp} = 1;
						foreach my $c ( @Components ) {
							if (comp_has_startprereq_of($comp, $c)) {
								$installed{$c} = 1;
							}
						}
					}
				}
			} else {
				foreach $comp ( @Components )
				{
					if ($installState{$comp} == $State_Install
						|| $installState{$comp} == $State_UpToDate) {
						$installed{$comp} = 1;
					} else {
						$installed{$comp} = 0;
					}
				}
			}
			show_autostart_menu(0);

			# show_autostart_menu unconditionally requested user hit return,
			# update_hca_firmware will also as needed, so only request return below
			# if one of these functions does something.
			my $need_ret = 0;
			$need_ret |= check_depmod;
			$need_ret |= check_ldconfig;
			$need_ret |= check_need_reboot;
			if ($need_ret) {
				HitKeyCont;
			}
			return;
		}
	}} until ($Default_Prompt);
	# we had an error above
	$exit_code = 1;
}

sub show_installed($)
{
	my $showversion = shift();	# should per comp versions be shown

	my %installed = ();
	my %statusMessage = ();
	my $comp;
	my $i;

	system "clear";
	printf ("$BRAND OPA Installed Software ($VERSION)\n\n");
	my $index=0;
	for ($i=0; $i < scalar(@Components); $i++)
	{
		if ($index > 0 && ($index % 20 == 0) ) {
			HitKeyCont;
		}
		$comp = $Components[$i];
		$installed{$comp} = comp_is_installed("$comp");
		if ( $showversion && $installed{$comp} ) {
			$statusMessage{$comp} = comp_installed_version($comp);
		}
		if (! $ComponentInfo{$comp}{'Hidden'}) {
			printf ("   %-20s ", $ComponentInfo{$comp}{'Name'});
			printInstallState($installed{$comp}, 0, 0, "$statusMessage{$comp}");
			$index++;
		}
	}
	HitKeyCont;
}

sub show_uninstall_menu($)
{
	my $showversion = shift();	# should per comp versions be shown

	my %installed = ();
	my %installed_version = ();
	my %installState = ();
	my %statusMessage = ();
	my %boldMessage = ();
	my $inp;
	my $comp;
	my $i;
	my $newstate;	# most recent state change for a comp
	my $firstline = 0;
	my $maxlines=14;
	my $num_hidden_comps = count_hidden_comps();

	print "Determining what is installed on system...\n";
	DumpComponents(0);
	foreach $comp ( @Components )
	{
		$installed{$comp} = comp_is_installed("$comp");
		if ($installed{$comp}) {
			$installed_version{$comp} = comp_installed_version("$comp");
		}
		$installState{$comp}= setstate($installed{$comp},0,0,$State_DoNotInstall);
	}
	do {
		if ( $Default_Uninstall) {
			foreach $comp ( @Components )
			{
				$installState{$comp} = $State_Uninstall;
			}
			$newstate = $State_Uninstall;
			$inp="P";
		} elsif ( $Default_CompUninstall) {
			foreach $comp ( @Components )
			{
				if ( $Default_Components{$comp} )
				{
					$installState{$comp} = $State_Uninstall;
				}
			}
			$newstate = $State_Uninstall;
			$inp="P";
		} else {
			system "clear";
			printf ("$BRAND OPA Uninstall Menu ($VERSION)\n\n");
			my $screens = int((scalar(@Components)-$num_hidden_comps + $maxlines-1)/$maxlines);
			if ($screens > 1 ) {
				printf ("Please Select Uninstall Action (screen %d of $screens):\n",
							$firstline/$maxlines+1);
			} else {
				printf ("Please Select Uninstall Action:\n");
			}
			my $index=0;
			for($i=0; $i < scalar(@Components); $i++)
			{
				$comp = $Components[$i];
				if ($index >= $firstline && $index < $firstline+$maxlines) {
					if ($showversion && "$statusMessage{$comp}" eq ""
						&& $installed{$comp}) {
						$statusMessage{$comp} = $installed_version{$comp};
						$boldMessage{$comp} = 0;
					}
					if (! $ComponentInfo{$comp}{'Hidden'}) {
						printf ("%x) %-20s ", $index-$firstline, $ComponentInfo{$comp}{'Name'});
						printInstallState($installed{$comp}, ($installState{$comp} == $State_Uninstall),
							$boldMessage{$comp}, $statusMessage{$comp});
						$index++;
					}
				} elsif (! $ComponentInfo{$comp}{'Hidden'}) {
					$index++;
				}
			}

			printf ("\n");
			if ($screens > 1 ) {
				printf ("N) Next Screen\n");
			}
			printf (  "P) Perform the selected actions\n");
			printf (  "U) Uninstall All\n");
			printf (  "X) Return to Previous Menu (or ESC)\n");
				

			$inp = getch();
		
			if ($inp =~ /[qQ]/ || $inp =~ /[xX]/ || ord($inp) == $KEY_ESC)
			{
				return;
			}

			# do not clear status messages when jump between screens
			if ($inp !~ /[nN]/ )
			{
				%statusMessage = ();
				%boldMessage = ();
			}
		}

		if ($inp =~ /[nN]/ )
		{
			if (scalar(@Components) > $maxlines) {
				$firstline += $maxlines;
				if ($firstline >= scalar(@Components)) {
					$firstline=0;
				}
			}
		} elsif ($inp =~ /[uU]/ )
		{
			foreach $comp ( @Components )
			{
				$installState{$comp} = $State_Uninstall;
			}
			$newstate = $State_Uninstall;
		}
		elsif ($inp =~ /[0123456789abcdefABCDEF]/)
		{
			my $value = hex($inp);
			my $index = get_comp_subscript($firstline, $maxlines, $value);
			if ( $value < $maxlines && $index < scalar(@Components)) {
				my $selected = $Components[$index];
				$installState{$selected} = shiftstate($installState{$selected},$installed{$selected},0,0);
				$newstate = $installState{$selected};
			}
		}

		if (! will_be_on_system($newstate)) {
			my $done = 0;	# we are done if we loop through with no changes
			# something is being uninstalled or not installed
			# loop through a few times to catch all coreqs, prereqs
			# and coreqs of prereqs
			while (! $done) {
				$done=1;
				foreach $comp ( @Components )
				{
					if (! will_be_on_system($installState{$comp})) {
						# comp will not be on system, make sure all its
						# pre/coreqs will also not be on system
						foreach my $c ( @Components )
						{
							if (comp_has_req_of($c, $comp)
								&& will_be_on_system($installState{$c})) {
								$statusMessage{$c}="requires $ComponentInfo{$comp}{'Name'}";
								$boldMessage{$c}=1;
								$installState{$c}=$installed{$c}?$State_Uninstall:$State_DoNotInstall;
								$done=0;
							}
						}
					}
					if ($ComponentInfo{$comp}{'Hidden'}) {
						# if no longer needed, also remove it
						my $needed = 0;	# is $comp needed
						foreach my $c ( @Components )
						{
							if (comp_has_req_of($c, $comp)
								&& will_be_on_system($installState{$c})) {
								$needed=1;
							}
						}
						if (! $needed && will_be_on_system($installState{$comp})) {
							$installState{$comp}=$installed{$comp}?$State_Uninstall:$State_DoNotInstall;
							$done=0;
						}
					}
				}
			}
		} elsif (will_be_on_system($newstate)) {
			# something is being left on system
			# loop through a few times to catch all coreqs, prereqs
			# and coreqs of prereqs
			my $done = 0;	# we are done if we loop through with no changes
			my $forcedone = 0;	# used to force infinite loop in subtle issue below
			while (! $forcedone && ! $done) {
				$done=1;
				foreach $comp ( @Components )
				{
					if (will_be_on_system($installState{$comp})) {
						# comp will be on system, make sure all its
						# pre/coreqs will also be on system
						foreach my $c ( @Components )
						{
							if (comp_has_req_of($comp, $c)
								&& ! will_be_on_system($installState{$c})) {
								if (! $installed{$c} ) {
									# pre/coreq $c not installed, can't keep comp
									# this is not expected to occur, but is
									# a safety net for corrupted systems
									$statusMessage{$comp}="requires $ComponentInfo{$c}{'Name'}";
									$boldMessage{$comp}=1;
									$installState{$comp}=$installed{$comp}?$State_Uninstall:$State_DoNotInstall;
									# bail to avoid infinite loop, especially
									# for prereq chains and missing coreqs
									# see install_menu discussion for a similar
									# related issue
									$forcedone=1;
									last;
								} else {
									# also install $c
									$statusMessage{$c}="needed by $ComponentInfo{$comp}{'Name'}";
									$boldMessage{$c}=1;
									$installState{$c}=$State_Install;
								}
								$done=0;
							}
						}
					}
				}
			}
		}

		if ($inp =~ /[pP]/)
		{
			# build a list of what will be installed after installation completes
			# use a space separate string so easier to pass and can be "grep'ed"
			my $install_list = "";
			my $uninstalling_list = "";
			foreach $comp ( @Components )
			{
				if ($installState{$comp} == $State_Install
					|| $installState{$comp} == $State_UpToDate) {
					$install_list .= " $comp ";
				}
				if ($installState{$comp} == $State_Uninstall) {
					$uninstalling_list .= " $comp ";
				}
			}

			# remove any alias pkgs if it exists. Meta pkgs will be handled in comp uninstall.
			rpm_uninstall_matches("opanode_", "opanode_", "", "");

			# perform the uninstall, work backwards through list
			foreach $comp ( reverse(@Components) )
			{
				if ($installState{$comp} == $State_Uninstall)
				{
					comp_uninstall($comp, $install_list, $uninstalling_list);
					$installed{$comp} = 0;
				}
			}
			if ( "$WrapperComponent" ne "" && "$install_list" eq "" ) {
				comp_uninstall($WrapperComponent, $install_list, $uninstalling_list);
			}
			# since we did an uninstall should request a return
			my $need_ret = 1;
			$need_ret |= check_depmod;
			$need_ret |= check_ldconfig;
			$need_ret |= check_need_reboot;
			if ($need_ret) {
				HitKeyCont;
			}
			return;
		}
	}while( !$Default_Prompt )
}

sub reconfig_autostart()
{
	my %installed = ();
	my $comp;

	build_autostart_components_list;
	foreach $comp ( @Components ) {
		$installed{$comp} = comp_is_installed("$comp");
		DebugPrint("installed($comp)=$installed{$comp}\n");
		# fill in subcomponents to simplify tests for Default_*Components below
		# if show_autostart_menu is called, it will ignore these extra entries
		foreach my $c ( @{ $ComponentInfo{$comp}{'StartComponents'} }) {
			$installed{$c} = $installed{$comp};
		}
	}
	if (! $Default_Autostart) {
		# interactive menu, use previous value as default
		show_autostart_menu(1);
	} else {
		if ( $Default_DisableAutostart) {
			# build list of components/subcomponents to disable
			my @disabled = ();
			foreach $comp ( @AutostartComponents ) {
				if ( $installed{$comp} ) {
					if ($Default_DisabledComponents{$comp}) {
						@disabled = ( @disabled, $comp );
					}
				}
			}
			disable_components(@disabled);
		}
		if ( $Default_EnableAutostart) {
			# build list of components/subcomponents to enable
			my @enabled = ();
			foreach $comp ( @AutostartComponents ) {
				if ( $installed{$comp} ) {
					if ($Default_EnabledComponents{$comp}) {
						@enabled = ( @enabled, $comp );
					}
				} else {
				}
			}
			enable_components(@enabled);
		}
		if (! $Default_DisableAutostart && ! $Default_EnableAutostart) {
			# use component specific default as value
			show_autostart_menu(2);
		}
	}
}

# states:
# $Start_Start - Enable Autostart
# $Start_NoStart - Disable Autostart
sub printStartState($$$$)
{
	my $enabled     = shift();
	my $boldmessage = shift();
	my $message   = shift();
	my $state_change = shift();

	if ( $enabled )
	{
		print GREEN, "[Enable ]", RESET;
	} else {
		print RED, "[Disable]", RESET;
	}
	if ( $state_change )
	{
		print " *";
	} else {
		print "  ";
	}
	if ("$message" ne "" && $boldmessage) {
		print BOLD, RED "$message", RESET, "\n";
	} else {
		print "$message\n";
	}

	return;
}

# convert a screen selection into a Autostart subscript
sub get_subscript($$$@)
{
	my $firstline=shift();
	my $maxlines=shift();
	my $selection=shift();
	my @list = ( @_ );
	my $index=0;
	my $i;

	for ($i=0; $i < scalar(@list); $i++)
	{
		my $comp = $list[$i];
		if ($index >= $firstline && $index < $firstline+$maxlines) {
			if ($index - $firstline == $selection) {
				return $i;
			}
			$index++;
		} else {
			$index++;
		}
	}
	return $i	# invalid entry return 1 past number of Components
}
sub show_autostart_menu($)
{
	my $sel_mode = shift(); # possible values are: 1: keep previous, 0: use default, 2: enable

	my $inp;
	my @PromptAutostart = ();
	my %enabled = ();
	my %statusMessage = ();
	my %boldMessage = ();
	my $comp;
	my $i;
	my $newenabled = 0;	# most recent state change for a comp
	my $firstline = 0;
	my $maxlines=13;

	# figure out which to prompt for
	# while selections may include SubComponents, we only look
	# at components and include all their subcomponents in the prompts
	foreach $comp ( @Components )
	{
		if ( comp_is_installed("$comp") && $ComponentInfo{$comp}{'HasStart'} )
		{
			# StartComponents will list Components and SubComponents which
			# have start capability
			@PromptAutostart = ( @PromptAutostart, @{ $ComponentInfo{$comp}{'StartComponents'} })
		}
	}
	if ( $Default_SameAutostart ) {
		foreach $comp ( @PromptAutostart )
		{
			my $state;
			# we recreate startup files to ensure they are in the
			# startup order of the new release
			if ( comp_IsAutostart2($comp) == 1 )
			{
				comp_enable_autostart2($comp,1);
				$state = "enabled";
			} else {
				comp_disable_autostart2($comp,1);
				$state = "disabled";
			}
			my $desc = comp_autostart_desc($comp);
			NormalPrint "Leaving autostart for $desc at its previous value: $state\n";
		}
		return;
	}

	foreach $comp ( @PromptAutostart )
	{
		DebugPrint("prompt for $comp\n");
		if ($sel_mode == 1) {
			# keep previous
			$enabled{$comp} = comp_IsAutostart2($comp);
		} elsif ($sel_mode == 2) {
			# enable
			$enabled{$comp} = 1;
		} else {
			# use default
			$enabled{$comp} = $ComponentInfo{$comp}{'DefaultStart'};
		}
	}
	while() {
		if ( $Default_Prompt) {
			$inp='P';
		} else {
			system "clear";        
			printf ("$BRAND OPA Autostart ($VERSION $DBG_FREE) Menu\n\n");
			my $screens = int((scalar(@PromptAutostart) + $maxlines-1)/$maxlines);
			if ($screens > 1 ) {
				printf ("Please Select Autostart Option (screen %d of $screens):\n",
							$firstline/$maxlines+1);
			} else {
				printf ("Please Select Autostart Option:\n");
			}
			my $index=0;
			for ($i=0; $i < scalar(@PromptAutostart); $i++)
			{
				$comp = $PromptAutostart[$i];
				my $state_change = 0;
				if( comp_IsAutostart2($comp) != $enabled{$comp} )
				{
					$state_change = 1;
				}
				if ($index >= $firstline && $index < $firstline+$maxlines) {
					printf ("%x) %-32s", $index-$firstline, comp_autostart_desc($comp));
					printStartState($enabled{$comp},
									$boldMessage{$comp}, $statusMessage{$comp}, $state_change);
					$index++;
				} else {
					$index++;
				}
			}
			printf("\n*: new desired state\n");
			printf ("\n");
			if ($screens > 1 ) {
				printf ("N) Next Screen\n");
			}
			printf (  "P) Perform the autostart changes\n");
			printf (  "S) Autostart All                     R) Autostart None\n");
			printf (  "X) Return to Previous Menu (or ESC)\n");	

			$inp = getch();                
			
			if ($inp =~ /[qQ]/ || $inp =~ /[xX]/ || ord($inp) == $KEY_ESC ) 
			{
				return;
			}

			# do not clear status messages when jump between screens
			if ($inp !~ /[nN]/ ) {
				%statusMessage = ();
				%boldMessage = ();
			}

		}

		if ($inp =~ /[nN]/ )
		{
			if (scalar(@PromptAutostart) > $maxlines) {
				$firstline += $maxlines;
				if ($firstline >= scalar(@PromptAutostart)) {
					$firstline=0;
				}
			}
		} elsif ($inp =~ /[sS]/ )
		{
			foreach $comp ( @PromptAutostart )
			{
				$enabled{$comp} = 1;
			}
			$newenabled = 1;
		} elsif ($inp =~ /[rR]/ )
		{
			foreach $comp ( @PromptAutostart )
			{
				$enabled{$comp} = 0;
			}
			$newenabled = 0;
		} elsif ($inp =~ /[0123456789abcdefABCDEF]/) {
			my $value = hex($inp);
			my $index = get_subscript($firstline, $maxlines, $value, @PromptAutostart);
			if ( $value < $maxlines && $index < scalar(@PromptAutostart)) {
				my $selected = $PromptAutostart[$index];
				$enabled{$selected} = ! $enabled{$selected};
				$newenabled = $enabled{$selected};
			}
		}

		if (! $newenabled) {
			# something is being disabled
			# loop through a few times to catch all prereqs
			# and prereqs of prereqs
			my $done = 0;	# we are done if we loop through with no changes
			while (! $done) {
				$done=1;
				foreach $comp ( @PromptAutostart )
				{
					if (! $enabled{$comp}) {
						# comp will be disabled, make sure all its
						# startprereqs will also be disabled
						foreach my $c ( @PromptAutostart )
						{
							if (comp_has_startprereq_of($c, $comp) && $enabled{$c}) {
								$statusMessage{$c}="requires $ComponentInfo{$comp}{'Name'}";
								$boldMessage{$c}=1;
								$enabled{$c}=0;
								$done=0;
							}
						}
					}
				}
			}
		} else {
			# something is being disabled
			# loop through a few times to catch all prereqs
			my $done = 0;	# we are done if we loop through with no changes
			while (! $done) {
				$done=1;
				foreach $comp ( @PromptAutostart )
				{
					if ($enabled{$comp}) {
						# comp will be enabled, make sure all its
						# startprereqs will also be enabled
						foreach my $c ( @PromptAutostart )
						{
							if (comp_has_startprereq_of($comp, $c) && ! $enabled{$c}) {
								# also enable $c
								$statusMessage{$c}="needed by $ComponentInfo{$comp}{'Name'}";
								$boldMessage{$c}=1;
								$enabled{$c}=1;
								$done=0;
							}
						}
					}
				}
			}
		}

		if ($inp =~ /[pP]/) {
			# perform the changes

			# first disable, do this in reverse order
			# so dependency issues are avoided
			foreach $comp ( reverse(@PromptAutostart) ) {
				if (! $enabled{$comp}) {
					comp_disable_autostart2($comp,0);
				}
			}
			# now enable
			foreach $comp ( @PromptAutostart ) {
				if ($enabled{$comp}) {
					comp_enable_autostart2($comp,0);
				}
			}
			HitKeyCont;
			return;
		}
		if ( $Default_Prompt ) {
			return;
		}
	}
		$exit_code = 1;	# unexpected
}

