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

$MainInstall="ofed";

@supported_kernels = ( $CUR_OS_VER );	# TBD how do we verify OS
my $Build_OsVer=$CUR_OS_VER;
my $Build_Debug=0;	# should we provide more info for debug
my $Build_Temp="";	# temp area to use for build
my $Default_Build = 0;	# -B option used to select build
my $Build_Force = 0;# rebuild option used to force full rebuild

$FirstIPoIBInterface=0; # first device is ib0

	# Names of supported install components
	# must be listed in depdency order such that prereqs appear 1st
# TBD ofed_vnic
@Components = ( "opa_stack", "intel_hfi", "ofed_mlx4", "mpi_selector",
		"ib_wfr_lite",
	   	"opa_stack_dev",
		"ofed_ipoib", "ofed_ib_bonding", 
	   	"mvapich2", "openmpi", "gasnet", "openshmem",
	   	"ofed_mpisrc", "ofed_udapl", "ofed_rds",
		"ofed_srp", "ofed_srpt", "ofed_iser", "ofed_iwarp",
		"opensm", "ofed_nfsrdma", "ofed_debug", "ibacm",);
# ofed_nfsrdma dropped in OFED 1.5.3rc2
# ofed_debug must be last

@SubComponents = ( );

	# an additional "special" component which is always installed when
	# we install/upgrade anything and which is only uninstalled when everything
	# else has been uninstalled.  Typically this will be the opaconfig
	# file and related files absolutely required by it (such as wrapper_version)
$WrapperComponent = "opa_config_ofed";

# This provides more detailed information about each Component in @Components
# since hashes are not retained in the order specified, we still need
# @Components and @SubComponents to control install and menu order
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
		# our special WrapperComponent, limited use
	"opa_config_ofed" =>	{ Name => "opa_config_ofed",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => "", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"opa_stack" =>	{ Name => "OFA OPA Stack",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => "", CoReq => "",
 						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => "",
					  StartComponents => [ "opa_stack" ],
					},
	"intel_hfi" =>	{ Name => "Intel HFI Driver",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
 						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "intel_hfi" ],
					},
	"ofed_mlx4" =>	{ Name => "OFED mlx4 Driver",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
 						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_mlx4" ],
					},
	"ib_wfr_lite" =>	{ Name => "WFR Lite Driver",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
 						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ib_wfr_lite" ],
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
	"opa_stack_dev" => { Name => "OFA OPA Development",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"ofed_vnic" =>	{ Name => "OFED Virtual NIC",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_vnic" ],
					},
	"ofed_ipoib" =>	{ Name => "OFA IP over IB",
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
	"ofed_rds" =>	{ Name => "OFED RDS",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ofed_ipoib ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_rds" ],
					},
	"ofed_udapl" =>	{ Name => "OFED uDAPL",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack ofed_ipoib ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"mvapich2" =>	{ Name => "MVAPICH2 (verbs,gcc)",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " intel_hfi opa_stack mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"openmpi" =>	{ Name => "OpenMPI (verbs,gcc)",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " intel_hfi opa_stack mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"gasnet" =>	{ Name => "Gasnet (hfi,gcc)",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " intel_hfi opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"openshmem" =>	{ Name => "OpenSHMEM (hfi,gcc)",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " intel_hfi opa_stack gasnet", CoReq => "",
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
	"ofed_srp" =>	{ Name => "OFED SRP",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_srp" ],
					},
	"ofed_srpt" =>	{ Name => "OFED SRP Target",
					  DefaultInstall => $State_DoNotAutoInstall,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_srpt" ],
					},
	"ofed_iser" =>	{ Name => "OFED iSER",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_iser" ],
					},
	"ofed_iwarp" =>	{ Name => "OFED iWARP",
					  DefaultInstall => $State_DoNotAutoInstall,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ofed_iwarp" ],
					},
	"opensm" =>		{ Name => "OFED Open SM",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "opensm" ],
					},
	"ofed_nfsrdma" =>{ Name => "OFED NFS RDMA",
					  DefaultInstall => $State_DoNotAutoInstall,
					  SrcDir => ".", DriverSubdir => "updates",
					  PreReq => " opa_stack ofed_ipoib ", CoReq => "",
					  # for now, skip startup option for nfs and nfslock
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
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
   "ibacm" =>		{ Name => "OFA IBACM",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ibacm" ],
					},
	);
	# translate from startup script name to component/subcomponent name
%StartupComponent = (
				"openibd" => "opa_stack",
				"opensmd" => "opensm",
				"ipoib" => "ipoib",
				"udapl" => "udapl",
				"rds" => "rds"
			);
	# has component been loaded since last configured autostart
%ComponentWasInstalled = (
				"opa_stack" => 0,
				"intel_hfi" => 0,
				"ofed_mlx4" => 0,
				"mpi_selector" => 0,
				"opa_stack_dev" => 0,
				"ofed_vnic" => 0,
				"ofed_ipoib" => 0,
				"ofed_ib_bonding" => 0,
				"ofed_rds" => 0,
				"ofed_udapl" => 0,
				"mvapich2" => 0,
				"openmpi" => 0,
				"gasnet" => 0,
				"openshmem" => 0,
				"ofed_mpisrc" => 0,
				"ofed_srp" => 0,
				"ofed_srpt" => 0,
				"ofed_iser" => 0,
				"ofed_iwarp" => 0,
				"opensm" => 0,
				"ofed_nfsrdma" => 0,
				"ofed_debug" => 0,
				"ibacm" => 0,
			);

# ==========================================================================
# opa_config_ofed installation
# This is a special WrapperComponent which only needs:
#	available, install and uninstall
# it cannot have startup scripts, dependencies, prereqs, etc

sub available_opa_config_ofed
{
	my $srcdir=$ComponentInfo{'opa_config_ofed'}{'SrcDir'};
	return ( -e "$srcdir/INSTALL" );
}

sub install_opa_config_ofed
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled

	my $srcdir=$ComponentInfo{'opa_config_ofed'}{'SrcDir'};
	NormalPrint("Installing $ComponentInfo{'opa_config_ofed'}{'Name'}...\n");

	check_config_dirs();
	copy_systool_file("$srcdir/INSTALL", "/sbin/opa_config_ofed");
	# no need for a version file, ofed will have its own

	# if IFS OFED is not installed, create opa_config too
	if ( ! -e "$BASE_DIR/version_wrapper" ) {
		copy_systool_file("$srcdir/INSTALL", "/sbin/opaconfig");
	}

	$ComponentWasInstalled{'opa_config_ofed'}=1;
}

sub uninstall_opa_config_ofed
{
	my $install_list = $_[0];	# total that will be left installed when done
	my $uninstalling_list = $_[1];	# what items are being uninstalled

	NormalPrint("Uninstalling $ComponentInfo{'opa_config_ofed'}{'Name'}...\n");
	system("rm -rf $ROOT/sbin/opa_config_ofed");
	# if IFS OFED is not installed, remove opaconfig too
	if ( ! -e "$BASE_DIR/version_wrapper" ) {
		system("rm -rf $ROOT/sbin/opaconfig");
	}
	$ComponentWasInstalled{'opa_config_ofed'}=0;
}

my $allow_install;
if ( my_basename($0) ne "INSTALL" )
{
	if ( my_basename($0) eq "ics_ib" )
	{
		printf("Warning: ics_ib is depricated, use opaconfig\n");
		HitKeyCont;
	}
	$allow_install=0;
} else {
	$allow_install=1;
}

#
# Check if all basic requirements are ok
#
sub verify_os_ver 
{
	determine_os_version;
	#my $supported_kernel;
	#
	#if (!$allow_install)
	#{
	#	my $catkernels = `cat $ROOT$BASE_DIR/supported_kernels`;
	#	@supported_kernels = split( / +/, $catkernels);
	#} else {
	#	system("echo -n @supported_kernels > ./config/supported_kernels");
	#}
	#
	#foreach my $supported_kernel_path ( @supported_kernels ) 
	#{    
	#	$supported_kernel = my_basename($supported_kernel_path);
	#
	#	$module_dirs[++$#module_dirs] = "/lib/modules/$supported_kernel";
	#}
	# make sure current OS version included so remove_driver works
	$module_dirs[++$#module_dirs] = "/lib/modules/$CUR_OS_VER";
}

sub Usage
{
	if ( $allow_install ) {
		#printf STDERR "Usage: $0 [-r root] [-v|-vv] -R osver -B osver [-d][-t tempdir] [--user_configure_options 'options'] [--kernel_configure_options 'options'] [--prefix dir] [--without-depcheck] [--rebuild] [--force] [--answer keyword=value] [--debug]\n";
		#printf STDERR "               or\n";
		#printf STDERR "Usage: $0 [-r root] [-v|-vv] [-a|-n|-U|-u|-s|-i comp|-e comp] [-E comp] [-D comp] [-l] [--user_configure_options 'options'] [--kernel_configure_options 'options'] [--prefix dir] [--without-depcheck] [--rebuild] [--force] [--answer keyword=value] [--debug]\n";
		#printf STDERR "Usage: $0 [-r root] [-v|-vv] [-a|-n|-U|-u|-s|-i comp|-e comp|-E comp|-D comp] [--user_configure_options 'options'] [--kernel_configure_options 'options'] [--prefix dir] [--without-depcheck] [--rebuild] [--force] [--answer=value]\n";
		printf STDERR "Usage: $0 [-r root] [-v|-vv] [-a|-n|-U|-u|-s|-i comp|-e comp|-E comp|-D comp] [--user_configure_options 'options'] [--kernel_configure_options 'options'] [--prefix dir] [--without-depcheck] [--rebuild] [--force] [--answer=value]\n";
	} else {
		printf STDERR "Usage: $0 [-r root] [-v|-vv] [-u|-s|-e comp] [-E comp] [-D comp] [--answer=value]\n";
		printf STDERR "          [--user_queries|--no_user_queries]\n";
	}
	printf STDERR "               or\n";
	printf STDERR "Usage: $0 -C\n";
	printf STDERR "               or\n";
	printf STDERR "Usage: $0 -V\n";
	if ( $allow_install ) {
		printf STDERR "       -a - install all ULPs and drivers with default options\n";
		printf STDERR "       -n - install all ULPs and drivers with default options\n";
		printf STDERR "            but with no change to autostart options\n";
		printf STDERR "       -U - upgrade/re-install all presently installed ULPs and drivers with\n";
		printf STDERR "            default options and no change to autostart options\n";
		printf STDERR "       -i comp - install the given component with default options\n";
		printf STDERR "            can appear more than once on command line\n";
		#printf STDERR "       -l - skip creating/removing symlinks to /usr/local from /usr/lib/opa\n";
		printf STDERR "       --user_configure_options 'options' - specify additional OFED build\n";
		printf STDERR "             options for user space srpms.  Causes rebuild of all user srpms\n";
		printf STDERR "       --kernel_configure_options 'options' - specify additional OFED build\n";
		printf STDERR "             options for driver srpms.  Causes rebuild of all driver srpms\n";
		printf STDERR "       --prefix dir - specify alternate directory prefix for install\n";
		printf STDERR "             default is /usr.  Causes rebuild of needed srpms\n";
	 	printf STDERR "       --without-depcheck - disable check of OS dependencies\n";
		printf STDERR "       --rebuild - force OFED rebuild\n";
		printf STDERR "       --force - force install even if distos don't match\n";
		printf STDERR "                 Use of this option can result in undefined behaviors\n";
		# --debug, -B, -t and -d options are purposely not documented
		#printf STDERR "       --debug - build a debug version of modules\n";
		#printf STDERR "       -B osver - run build for all components targetting kernel osver\n";
		#printf STDERR "       -t - temp area for use by builds, only valid with -B\n";
		#printf STDERR "       -d - enable build debugging assists, only valid with -B\n";
		#printf STDERR "       -R osver - force install for kernel osver rather than running kernel.\n";
	}
	printf STDERR "       -u - uninstall all ULPs and drivers with default options\n";
	printf STDERR "       -s - enable autostart for all installed drivers\n";
	printf STDERR "       -r - specify alternate root directory, default is /\n";
	printf STDERR "       -e comp - uninstall the given component with default options\n";
	printf STDERR "            can appear more than once on command line\n";
	printf STDERR "       -E comp - enable autostart of given component\n";
	printf STDERR "            can appear with -D or more than once on command line\n";
	printf STDERR "       -D comp - disable autostart of given component\n";
	printf STDERR "            can appear with -E or more than once on command line\n";
	printf STDERR "       -v - verbose logging\n";
	printf STDERR "       -vv - very verbose debug logging\n";
	printf STDERR "       -C - output list of supported components\n";
	printf STDERR "       -V - output Version\n";

	printf STDERR "       --user_queries - permit non-root users to query the fabric. (default)\n";
	printf STDERR "       --no_user_queries - non root users cannot query the fabric.\n";
	showAnswerHelp();

	printf STDERR "       default options retain existing configuration files\n";
	printf STDERR "       supported component names:\n";
	printf STDERR "            ";
	foreach my $comp ( @Components )
	{
		printf STDERR " $comp";
	}
	printf STDERR "\n";
	if (scalar(@SubComponents) > 0) {
		printf STDERR "       additional component names allowed for -E and -D options:\n";
		printf STDERR "            ";
		foreach my $comp ( @SubComponents )
		{
			printf STDERR " $comp";
		}
		printf STDERR "\n";
	}
	exit (2);
}

# translate an ibaccess component name into the corresponding list of ofed comps
# if the given name is invalid or has no corresponding OFED component
# returns an empty list
sub translate_comp
{
	my($arg)=$_[0];
	if ("$arg" eq "ibdev")			{ return ( "opa_stack_dev" );
    } elsif ("$arg" eq "opa")       { return ( "oftools",
		"mvapich2_gcc_hfi", "openmpi_gcc_hfi", "mvapich2_intel_hfi",
		"openmpi_intel_hfi", "mvapich2_pgi_hfi", "openmpi_pgi_hfi" );
	} elsif ("$arg" eq "ifibre")	{ return ( "ofed_srp" );
	} elsif ("$arg" eq "ipoib")		{ return ( "ofed_ipoib", "ib_bonding_marker" );
	} elsif ("$arg" eq "mpi")		{ return ( "mvapich2", "openmpi", 
		"mvapich2_gcc_hfi", "openmpi_gcc_hfi", "mvapich2_intel_hfi", 
		"openmpi_intel_hfi", "mvapich2_pgi_hfi", "openmpi_pgi_hfi" );
	} elsif ("$arg" eq "mpisrc")	{ return ( "ofed_mpisrc" );
	} elsif ("$arg" eq "udapl")		{ return ( "ofed_udapl" );
	} elsif ("$arg" eq "rds")		{ return ( "ofed_rds" );
		# no ibaccess argument equivalent for:  ofed_srpt, ofed_iser,
		#	openmpi, ofed_iwarp, opensm, ofed_nfsrdma, ofed_debug
		#
	} else {
		return ();	# invalid name
	}
}

sub process_args
{
	my $arg;
	my $last_arg;
	my $setroot = 0;
	my $install_opt = 0;
	my $setcomp = 0;
	my $setenabled = 0;
	my $setdisabled = 0;
	my $setosver = 0;
	my $setbuildtemp = 0;
	my $comp = 0;
	my $osver = 0;
	my $setuseroptions = 0;
	my $setcurosver = 0;
	my $setkerneloptions = 0;
	my $setprefix = 0;
	my $setanswer = 0;
	my $patch_ofed=0;

	if (scalar @ARGV >= 1) {
		foreach $arg (@ARGV) {
			if ( $setroot ) {
				$ROOT="$arg";
				if (substr($ROOT,0,1) ne "/") {
					printf STDERR "Invalid -r root (must be absolute path): '$ROOT'\n";
					Usage;
				}
				if (! -d $ROOT) {
					printf STDERR "Invalid -r root (directory not found): '$ROOT'\n";
					Usage;
				}
				$setroot=0;
			} elsif ( $setanswer ) {
				my @pair = split /=/,$arg;
				if ( scalar(@pair) != 2 ) {
					printf STDERR "Invalid --answer keyword=value: '$arg'\n";
					Usage;
				}
				set_answer($pair[0], $pair[1]);
				$setanswer=0;
			} elsif ( $setcomp ) {
				foreach $comp ( @Components )
				{
					if ( "$arg" eq "$comp" )
					{
						$Default_Components{$arg} = 1;
						$setcomp=0;
					}
				}
				if ( $setcomp )
				{
					my @comps = translate_comp($arg);
					# if empty list returned, we will not clear setcomp and
					# will get error below
					foreach $comp ( @comps )
					{
						$Default_Components{$comp} = 1;
						$setcomp=0;
					}
				}
				if ( $setcomp )
				{
					printf STDERR "Invalid component: $arg\n";
					Usage;
				}
			} elsif ( $setenabled ) {
				foreach $comp ( @Components, @SubComponents )
				{
					if ( "$arg" eq "$comp" )
					{
						$Default_EnabledComponents{$arg} = 1;
						$setenabled=0;
					}
				}
				if ( $setenabled )
				{
					my @comps = translate_comp($arg);
					# if empty list returned, we will not clear setcomp and
					# will get error below
					foreach $comp ( @comps )
					{
						$Default_EnabledComponents{$comp} = 1;
						$setenabled=0;
					}
				}
				if ( $setenabled )
				{
					printf STDERR "Invalid component: $arg\n";
					Usage;
				}
			} elsif ( $setdisabled ) {
				foreach $comp ( @Components, @SubComponents )
				{
					if ( "$arg" eq "$comp" )
					{
						$Default_DisabledComponents{$arg} = 1;
						$setdisabled=0;
					}
				}
				if ( $setdisabled )
				{
					my @comps = translate_comp($arg);
					# if empty list returned, we will not clear setcomp and
					# will get error below
					foreach $comp ( @comps )
					{
						$Default_DisabledComponents{$comp} = 1;
						$setdisabled=0;
					}
				}
				if ( $setdisabled )
				{
					printf STDERR "Invalid component: $arg\n";
					Usage;
				}
			} elsif ( $setosver ) {
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
			} elsif ( $setcurosver ) {
				$CUR_OS_VER="$arg";
				@supported_kernels = ( $CUR_OS_VER );
				$setcurosver=0;
			} elsif ( "$arg" eq "-r" ) {
				$setroot=1;
			} elsif ( "$arg" eq "-v" ) {
				$LogLevel=1;
			} elsif ( "$arg" eq "-vv" ) {
				$LogLevel=2;
			} elsif ( "$arg" eq "-i" ) {
				$Default_CompInstall=1;
				$Default_Prompt=1;
				$setcomp=1;
				if ($install_opt || $Default_CompUninstall || $Default_Build) {
					# can't mix -i with other install controls
					printf STDERR "Invalid combination of options: $arg not permitted with previous options\n";
					Usage;
				}
			} elsif ( "$arg" eq "-e" ) {
				$Default_CompUninstall=1;
				$Default_Prompt=1;
				$setcomp=1;
				if ($install_opt || $Default_CompInstall || $Default_Build) {
					# can't mix -e with other install controls
					printf STDERR "Invalid combination of options: $arg not permitted with previous options\n";
					Usage;
				}
			} elsif ( "$arg" eq "-E" ) {
				$Default_Autostart=1;
				$Default_EnableAutostart=1;
				$Default_Prompt=1;
				$setenabled=1;
				if ($Default_Build) {
					# can't mix -E with other install controls
					printf STDERR "Invalid combination of options: $arg not permitted with previous options\n";
					Usage;
				}
			} elsif ( "$arg" eq "-D" ) {
				$Default_Autostart=1;
				$Default_DisableAutostart=1;
				$Default_Prompt=1;
				$setdisabled=1;
				if ($Default_Build) {
					# can't mix -D with other install controls
					printf STDERR "Invalid combination of options: $arg not permitted with previous options\n";
					Usage;
				}
			} elsif ( "$arg" eq "--user_configure_options" ) {
				$setuseroptions=1;
			} elsif ( "$arg" eq "--kernel_configure_options" ) {
				$setkerneloptions=1;
			} elsif ( "$arg" eq "--prefix" ) {
				$setprefix=1;
			} elsif ( "$arg" eq "--answer" ) {
 				$setanswer=1;
			} elsif ( "$arg" eq "--without-depcheck" ) {
				$rpm_check_dependencies=0;
			} elsif ( "$arg" eq "--force" ) {
				$Force_Install=1;
			} elsif ( "$arg" eq "-C" ) {
				ShowComponents;
				exit(0);
			} elsif ( "$arg" eq "-V" ) {
				printf "$VERSION\n";
				exit(0);
			} elsif ( "$arg" eq "-R" ) {
				$setcurosver=1;
			} elsif ( "$arg" eq "-B" ) {
				# undocumented option to do a build for specific OS
				$Default_Build=1;
				$Default_Prompt=1;
				$setosver=1;
				if ($install_opt || $Default_CompInstall || $Default_CompUninstall || $Default_Autostart) {
					# can't mix -B with install
					printf STDERR "Invalid combination of options: $arg not permitted with previous options\n";
					Usage;
				}
			} elsif ( "$arg" eq "-d" ) {
				# undocumented option to aid debug of build
				$Build_Debug=1;
				if ($install_opt || $Default_CompInstall || $Default_CompUninstall || $Default_Autostart) {
					# can't mix -d with install
					printf STDERR "Invalid combination of options: $arg not permitted with previous options\n";
					Usage;
				}
			} elsif ( "$arg" eq "-t" ) {
				# undocumented option to aid debug of build
				$setbuildtemp=1;
				if ($install_opt || $Default_CompInstall || $Default_CompUninstall || $Default_Autostart) {
					# can't mix -t with install
					printf STDERR "Invalid combination of options: $arg not permitted with previous options\n";
					Usage;
				}
			} elsif ( "$arg" eq "--rebuild" ) {
				# force rebuild
				$Build_Force=1;
				$OFED_force_rebuild=1;
			} elsif ( "$arg" eq "--debug" ) {
				$OFED_debug=1;
			} elsif ( "$arg" eq "--user_queries" ) {
				$Default_UserQueries=1;
			} elsif ( "$arg" eq "--no_user_queries" ) {
				$Default_UserQueries=-1;
			} elsif ( "$arg" eq "--patch_ofed" ) {
				$patch_ofed=1;
			} else {
				# Install options
				if ( "$arg" eq "-a" ) {
					$Default_Install=1;
				} elsif ( "$arg" eq "-u" ) {
					$Default_Uninstall=1;
				} elsif ( "$arg" eq "-s" ) {
					$Default_Autostart=1;
				} elsif ( "$arg" eq "-n" ) {
					$Default_Install=1;
					$Default_SameAutostart=1;
				} elsif ( "$arg" eq "-U" ) {
					$Default_Upgrade=1;
					$Default_SameAutostart=1;
				} else {
					printf STDERR "Invalid option: $arg\n";
					Usage;
				}
				if ($install_opt || $Default_CompInstall
					 || $Default_CompUninstall) {
					# only one of the above install selections
					printf STDERR "Invalid combination of options: $arg not permitted with previous options\n";
					Usage;
				}
				$install_opt=1;
				$Default_Prompt=1;
			}
			$last_arg=$arg;
		}
	}
	if ( $setroot || $setcomp || $setenabled || $setdisabled  || $setosver || $setbuildtemp || $setuseroptions || $setkerneloptions || $setprefix || $setanswer) {
		printf STDERR "Missing argument for option: $last_arg\n";
		Usage;
	}
	if ( ($Default_Install || $Default_CompInstall || $Default_Upgrade
			|| $Force_Install)
		 && ! $allow_install) {
		printf STDERR "Installation options not permitted in this mode\n";
		Usage;
	}
	if ( ($Default_Build || $OFED_force_rebuild || $OFED_debug
		 || ($OFED_user_configure_options ne '')
			 || ($OFED_kernel_configure_options ne '')
			 || ($OFED_prefix ne '/usr'))
		&& ! $allow_install) {
		printf STDERR "Build options not permitted in this mode\n";
		Usage;
	}
	if ( $patch_ofed) {
		NormalPrint("Patching OFED...\n");
		LogPrint("Executing: cd $ComponentInfo{'opa_stack'}{'SrcDir'}; ./patch_ofed3\n");
		system("cd $ComponentInfo{'opa_stack'}{'SrcDir'}; ./patch_ofed3");
		HitKeyCont;
	}
}

my @INSTALL_CHOICES= ();
sub show_menu
{
	my $inp;
	my $max_inp;

	@INSTALL_CHOICES= ();
	if ( $Default_Install ) {
		NormalPrint ("Installing All OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 1);
	}
	if ( $Default_CompInstall ) {
		NormalPrint ("Installing Selected OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 1);
	}
	if ( $Default_Upgrade ) {
		NormalPrint ("Upgrading/Re-Installing OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 1);
	}
	if ( $Default_Uninstall ) {
		NormalPrint ("Uninstalling All OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 4);
	}
	if ($Default_CompUninstall ) {
		NormalPrint ("Uninstalling Selected OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 4);
	}
	if ($Default_Autostart) {
		NormalPrint ("Configuring autostart for Selected installed OPA Drivers\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES,  3);
	}
	if (scalar(@INSTALL_CHOICES) > 0) {
		return;
	}
START:	
	system "clear";
	printf ("$BRAND OPA $VERSION Software\n\n");
	if ($allow_install) {
		printf ("   1) Install/Uninstall Software\n");
	} else {
		printf ("   1) Show Installed Software\n");
	}
	printf ("   2) Reconfigure $ComponentInfo{'ofed_ipoib'}{'Name'}\n");
	printf ("   3) Reconfigure Driver Autostart \n");
	$max_inp=3;
	if (!$allow_install)
	{
		printf ("   4) Uninstall Software\n");
		$max_inp=4;
	}
	printf ("\n   X) Exit\n");


	$inp = getch();      

	if ($inp =~ /[qQ]/ || $inp =~ /[Xx]/ ) {
		@INSTALL_CHOICES = ( 99999 );	# indicates exit, none selected
		return;
	}
	if (ord($inp) == $KEY_ENTER) {           
		goto START;
	}

	if ($inp =~ /[0123456789abcdefABCDEF]/)
	{
		$inp = hex($inp);
	}

	if ($inp < 1 || $inp > $max_inp) 
	{
#		printf ("Invalid choice...Try again\n");
		goto START;
	}
	@INSTALL_CHOICES = ( $inp );
}

process_args;
check_root_user;
if ( ! $Default_Build ) {
	open_log("");
} else {
	open_log("./build.log");
}

if ( ! $Default_Build ) {
	verify_os_ver;
	verify_modtools;
	#if ($allow_install) {
	#	verify_distrib_files;
	#}
} else {
	determine_os_version;
}
# post process ib_bonding_marker selection via command line args so
# can check Available for ib-bonding
if ( $Default_Components{'ib_bonding_marker'} ) {
	if ( comp_is_available('ofed_ib_bonding') ) {
		$Default_Components{'ofed_ib_bonding'} = 1;
	}
}
if ( $Default_EnabledComponents{'ib_bonding_marker'} ) {
	if ( comp_is_available('ofed_ib_bonding') ) {
		$Default_EnabledComponents{'ofed_ib_bonding'} = 1;
	}
}
if ( $Default_DisabledComponents{'ib_bonding_marker'} ) {
	if ( comp_is_available('ofed_ib_bonding') ) {
		$Default_DisabledComponents{'ofed_ib_bonding'} = 1;
	}
}

set_libdir;
init_ofed_rpm_info($CUR_OS_VER);

RESTART:
if ($Default_Build) {
	$exit_code = build_all_components($Build_OsVer, $Build_Debug, $Build_Temp, $Build_Force);
	goto DONE;
} else {
	show_menu;
}

foreach my $INSTALL_CHOICE ( @INSTALL_CHOICES )
{
	if ($allow_install && $INSTALL_CHOICE == 1) 
	{
		select_debug_release(".");
		show_install_menu(1);
		if (! $Default_Prompt) {
			goto RESTART;
		} else {
			if ($exit_code == 0) {
				print "Done Installing OPA Software.\n"
			} else {
				print "Failed to install all OPA software.\n"
			}
		}
	} 
	elsif ($INSTALL_CHOICE == 1) {
		show_installed(1);
		goto RESTART;
	}
	elsif ($INSTALL_CHOICE == 4)
	{
		show_uninstall_menu(1);
		if (! $Default_Prompt ) {
			goto RESTART;
		} else {
			if ($exit_code == 0) {
				print "Done Uninstalling OPA Software.\n"
			} else {
				print "Failed to uninstall all OPA Software.\n"
			}
		}
	}
	elsif ($INSTALL_CHOICE == 2) 
	{     
		Config_ifcfg(1,"$ComponentInfo{'ofed_ipoib'}{'Name'}","ib", "$FirstIPoIBInterface",1);
		check_network_config;
		#Config_IPoIB_cfg;
		goto RESTART;
	}
	elsif ($INSTALL_CHOICE == 3) 
	{
		reconfig_autostart;
		if (! $Default_Prompt ) {
			goto RESTART;
		} else {
			print "Done OPA Driver Autostart Configuration.\n"
		}
	}
}
DONE:
close_log;
exit $exit_code;
