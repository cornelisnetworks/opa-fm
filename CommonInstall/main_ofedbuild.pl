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

# ===========================================================================
# Main menus, option handling and version handling for FF for OFED install
#

$MainInstall="ofedbuild";

my $Build_OsVer=$CUR_OS_VER;
my $Build_Debug=0;	# should we provide more info for debug
my $Build_Temp="";	# temp area to use for build
my $Build_Force = 0;# rebuild option used to force full rebuild

my $FirstIPoIBInterface=0; # first device is ib0

	# Names of supported install components
	# must be listed in depdency order such that prereqs appear 1st
@Components = ( "opa_stack", "ofed_mlx4", "opa_stack_dev", "ofed_ipoib",
	   			"ofed_ib_bonding", "mpi_selector", 
			   	"mvapich2", "openmpi", "ofed_mpisrc", "ofed_udapl", "ofed_rds",
			   	"ofed_srp", "ofed_srpt", "ofed_iser",
			   	"ofed_iwarp", "opensm", "ofed_nfsrdma", "ofed_debug" );
				"opensm", "ofed_nfsrdma", "ofed_debug", );
# ofed_nfsrdma dropped in OFED 1.5.3rc2
# ofed_debug must be last

@SubComponents = ( );

	# an additional "special" component which is always installed when
	# we install/upgrade anything and which is only uninstalled when everything
	# else has been uninstalled.  Typically this will be the opaconfig
	# file and related files absolutely required by it (such as wrapper_version)
$WrapperComponent = "";

# This provides more detailed information about each Component in @Components
# since hashes are not retained in the order specified, we still need
# @Components and @SubComponens to control install and menu order
# Fields are as follows:
#	Name => full name of component/subcomponent for prompts
# 	DefaultInstall => default installation (State_DoNotInstall or State_Install)
#					  used only when available and ! installed
#	SrcDir => directory name within install media for component
#	DriverSubdir => directory within /lib/modules/$CUR_OS_VERSION/ to
#					install driver to.  Set to "" if no drivers in component
#					set to "." if no subdir
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
#	HasFirmware =>  components which need HCA firmware update after installed
#
# Note both Components and SubComponents are included in the list below.
# Components require all fields be supplied
# SubComponents only require the following fields:
#	Name, PreReq (should reference only components), HasStart, StartPreReq,
#	DefaultStart
%ComponentInfo = (
	"opa_stack" => 	{ Name => "OFA OPA Stack",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => "", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => "",
					  StartComponents => [ "opa_stack" ],
					},
	"ofed_mlx4" => 	{ Name => "OFA mlx4 Driver",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_mlx4" ],
					},
	"opa_stack_dev" => { Name => "OFA OPA Development",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"ofed_ipoib" => { Name => "OFA IP over IB",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_ipoib" ],
					},
	"ofed_ib_bonding" =>	{ Name => "OFA IB Bonding",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack ofed_ipoib ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"ofed_rds" =>	{ Name => "OFA RDS",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ofed_ipoib ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_rds" ],
					},
	"ofed_udapl" =>	{ Name => "OFA uDAPL",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack ofed_ipoib ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"mpi_selector" =>	{ Name => "MPI selector",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 1, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"mvapich2" =>	{ Name => "MVAPICH2 for gcc",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"openmpi" =>	{ Name => "OpenMPI for gcc",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"ofed_mpisrc" =>{ Name => "MPI Source",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack opa_stack_dev mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"ofed_srp" =>	{ Name => "OFA SRP",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_srp" ],
					},
	"ofed_srpt" =>	{ Name => "OFA SRP Target",
					  DefaultInstall => $State_DoNotAutoInstall,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_srpt" ],
					},
	"ofed_iser" =>	{ Name => "OFA iSER",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_iser" ],
					},
	"ofed_iwarp" =>	{ Name => "OFA iWARP",
					  DefaultInstall => $State_DoNotAutoInstall,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_iwarp" ],
					},
	"opensm" =>		{ Name => "OFA Open SM",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "opensm" ],
					},
	"ofed_nfsrdma" =>{ Name => "OFA NFS RDMA",
					  DefaultInstall => $State_DoNotAutoInstall,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ofed_ipoib ", CoReq => "",
					  # for now, skip startup option for nfs and nfslock
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"ofed_debug" =>	{ Name => "OFA Debug Info",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	);
	# translate from startup script name to component/subcomponent name
%StartupComponent = (
				"openibd" => "opa_stack",
				"opensmd" => "opensm",
				"ipoib" => "ipoib",
				"udapl" => "udapl",
				"ics_sdp" => "sdp",
				"rds" => "rds"
			);
	# has component been loaded since last configured autostart
%ComponentWasInstalled = (
				"opa_stack" => 0,
				"ofed_mlx4" => 0,
				"opa_stack_dev" => 0,
				"ofed_ipoib" => 0,
				"ofed_ib_bonding" => 0,
				"ofed_rds" => 0,
				"ofed_udapl" => 0,
				"mpi_selector" => 0,
				"mvapich2" => 0,
				"openmpi" => 0,
				"ofed_mpisrc" => 0,
				"ofed_srp" => 0,
				"ofed_srpt" => 0,
				"ofed_iser" => 0,
				"ofed_iwarp" => 0,
				"opensm" => 0,
				"ofed_nfsrdma" => 0,
				"ofed_debug" => 0,
			);

sub Usage
{
	#printf STDERR "Usage: $0 [-i] [-v|-vv] [-V osver] [-t tempdir] [-d] [--user_configure_options 'options'] [--kernel_configure_options 'options'] [--prefix dir] [--without-depcheck] [--rebuild] [--answer keyword=value] [--debug]\n";
	printf STDERR "Usage: $0 [-i] [-v|-vv] [-V osver] [-t tempdir] [-d] [--user_configure_options 'options'] [--kernel_configure_options 'options'] [--prefix dir] [--without-depcheck] [--rebuild] [--answer keyword=value] [--debug]\n";
	printf STDERR "       -i - interactive prompts, default is to build without prompts\n";
	printf STDERR "       -v - verbose logging\n";
	printf STDERR "       -vv - very verbose debug logging\n";
	printf STDERR "       -V osver - kernel version to build for, default is $CUR_OS_VER\n";
	printf STDERR "       -t - temp area for use by builds, only valid with -B\n";
	printf STDERR "       -d - enable build debugging assists, only valid with -B\n";
	printf STDERR "       --user_configure_options 'options' - specify additional OFA build\n";
	printf STDERR "             options for user space srpms.  Causes rebuild of all user srpms\n";
	printf STDERR "       --kernel_configure_options 'options' - specify additional OFA build\n";
	printf STDERR "             options for driver srpms.  Causes rebuild of all driver srpms\n";
	printf STDERR "       --prefix dir - specify alternate directory prefix for install\n";
	printf STDERR "             default is /usr.  Causes rebuild of needed srpms\n";
	printf STDERR "       --without-depcheck - disable check of OS dependencies\n";
	printf STDERR "       --rebuild - force full rebuild\n";
	printf STDERR "       --debug - build a debug version of modules\n";
	showAnswerHelp();
	exit (2);
}

$Default_Prompt=1;

sub process_args
{
	my $arg;
	my $last_arg;
	my $setosver = 0;
	my $setbuildtemp = 0;
	my $setuseroptions = 0;
	my $setkerneloptions = 0;
	my $setprefix = 0;
	my $setanswer = 0;
	my $osver = 0;

	if (scalar @ARGV >= 1) {
		foreach $arg (@ARGV) {
			if ( $setosver ) {
				$Build_OsVer="$arg";
				$setosver=0;
			} elsif ( $setbuildtemp ) {
				$Build_Temp="$arg";
				$setbuildtemp=0;
			} elsif ( $setuseroptions ) {
				$OFED_user_configure_options="$arg";
				$setuseroptions=0;
			} elsif ( $setkerneloptions ) {
				$OFED_kernel_configure_options="$arg";
				$setkerneloptions=0;
			} elsif ( $setprefix ) {
				$OFED_prefix="$arg";
				$setprefix=0;
			} elsif ( $setanswer ) {
				my @pair = split /=/,$arg;
				if ( scalar(@pair) != 2 ) {
					printf STDERR "Invalid --answer keyword=value: '$arg'\n";
					Usage;
				}
				set_answer($pair[0], $pair[1]);
				$setanswer=0;
			} elsif ( "$arg" eq "-v" ) {
				$LogLevel=1;
			} elsif ( "$arg" eq "-vv" ) {
				$LogLevel=2;
			} elsif ( "$arg" eq "-V" ) {
				$setosver=1;
			} elsif ( "$arg" eq "-d" ) {
				$Build_Debug=1;
			} elsif ( "$arg" eq "-t" ) {
				$setbuildtemp=1;
			} elsif ( "$arg" eq "-i" ) {
				$Default_Prompt=0;
			} elsif ( "$arg" eq "--user_configure_options" ) {
				$setuseroptions=1;
			} elsif ( "$arg" eq "--kernel_configure_options" ) {
				$setkerneloptions=1;
			} elsif ( "$arg" eq "--prefix" ) {
				$setprefix=1;
			} elsif ( "$arg" eq "--without-depcheck" ) {
				$rpm_check_dependencies=0;
			} elsif ( "$arg" eq "--rebuild" ) {
				$Build_Force=1;
			} elsif ( "$arg" eq "--answer" ) {
				$setanswer=1;
			} elsif ( "$arg" eq "--debug" ) {
				$OFED_debug=1;
			} else {
				printf STDERR "Invalid option: $arg\n";
				Usage;
			}
			$last_arg=$arg;
		}
	}
	if ( $setosver || $setbuildtemp || $setuseroptions || $setkerneloptions || $setprefix || $setanswer) {
		printf STDERR "Missing argument for option: $last_arg\n";
		Usage;
	}
}

process_args;
check_root_user;
open_log("./build.log");

determine_os_version;
set_libdir;
init_ofed_rpm_info($CUR_OS_VER);

$exit_code = build_all_components($Build_OsVer, $Build_Debug, $Build_Temp, $Build_Force);
close_log;
exit $exit_code;
