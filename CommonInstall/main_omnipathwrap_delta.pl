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

@supported_kernels = ( $CUR_OS_VER );	# TBD how do we verify OS
my $Build_OsVer=$CUR_OS_VER;
my $Build_Debug=0;	# should we provide more info for debug
my $Build_Temp="";	# temp area to use for build
my $Default_Build = 0;	# -B option used to select build
my $Build_Force = 0;# rebuild option used to force full rebuild

$FirstIPoIBInterface=0; # first device is ib0

	# Names of supported install components
	# must be listed in depdency order such that prereqs appear 1st

my @OmniPathAllComponents = ( "mvapich2_gcc_hfi",
		   			"mvapich2_intel_hfi",
					"openmpi_gcc_hfi",
				   	"openmpi_intel_hfi",
 					);

# these are now gone, list them so they get uninstalled
my @Components_other = ( "opa_stack", "ibacm", "mpi_selector", "intel_hfi",
		"oftools", "opa_stack_dev", "fastfabric", "rdma_ndd",
		"delta_ipoib", "opafm", "opamgt_sdk",
	   	@OmniPathAllComponents, 
		"gasnet", "openshmem", "sandiashmem",
	   	"mvapich2", "openmpi",
	   	"delta_mpisrc", "hfi1_uefi", "delta_debug", );
my @Components_rhel67 = ( "opa_stack", "ibacm", "mpi_selector", "intel_hfi",
                "oftools", "opa_stack_dev", "fastfabric", "rdma_ndd",
                "delta_ipoib", "opafm", "opamgt_sdk",
                @OmniPathAllComponents,
                "gasnet", "openshmem",
                "mvapich2", "openmpi",
                "delta_mpisrc", "hfi1_uefi", "delta_debug", );
my @Components_rhel72 = ( "opa_stack", "ibacm", "mpi_selector", "intel_hfi",
		"oftools", "opa_stack_dev", "fastfabric", "rdma_ndd",
		"delta_ipoib", "opafm", "opamgt_sdk",
	   	@OmniPathAllComponents,
		"openmpi_gcc_cuda_hfi", "gasnet", "openshmem", "sandiashmem",
	   	"mvapich2", "openmpi",
	   	"delta_mpisrc", "hfi1_uefi", "delta_debug", );
my @Components_sles12_sp2 = ( "opa_stack", "ibacm", "mpi_selector", "intel_hfi",
		"oftools", "opa_stack_dev", "fastfabric", "rdma_ndd",
		"delta_ipoib", "opafm", "opamgt_sdk",
	   	@OmniPathAllComponents,
		"openmpi_gcc_cuda_hfi", "gasnet", "openshmem", "sandiashmem",
	   	"mvapich2", "openmpi",
	   	"delta_mpisrc", "hfi1_uefi", "delta_debug", );
my @Components_rhel73 = ( "opa_stack", "ibacm", "mpi_selector", "intel_hfi",
		"oftools", "opa_stack_dev", "fastfabric", "rdma_ndd",
		"delta_ipoib", "opafm", "opamgt_sdk",
	   	@OmniPathAllComponents,
		"openmpi_gcc_cuda_hfi",	"gasnet", "openshmem", "sandiashmem",
	   	"mvapich2", "openmpi",
	   	"delta_mpisrc", "hfi1_uefi", "delta_debug", );
my @Components_sles12_sp3 = ( "opa_stack", "ibacm", "mpi_selector", "intel_hfi",
		"oftools", "opa_stack_dev", "fastfabric", "rdma_ndd",
		"delta_ipoib", "opafm", "opamgt_sdk",
	   	@OmniPathAllComponents,
		"openmpi_gcc_cuda_hfi", "gasnet", "openshmem", "sandiashmem",
	   	"mvapich2", "openmpi",
	   	"delta_mpisrc", "hfi1_uefi", "delta_debug", );
my @Components_rhel74 = ( "opa_stack", "ibacm", "mpi_selector", "intel_hfi",
		"oftools", "opa_stack_dev", "fastfabric", "rdma_ndd",
		"delta_ipoib", "opafm", "opamgt_sdk",
	   	@OmniPathAllComponents,
		"openmpi_gcc_cuda_hfi",	"gasnet", "openshmem", "sandiashmem",
	   	"mvapich2", "openmpi",
	   	"delta_mpisrc", "hfi1_uefi", "delta_debug", );
@Components = ( );

# delta_debug must be last

@SubComponents = ( );

	# an additional "special" component which is always installed when
	# we install/upgrade anything and which is only uninstalled when everything
	# else has been uninstalled.  Typically this will be the opaconfig
	# file and related files absolutely required by it (such as wrapper_version)
$WrapperComponent = "opaconfig";

# This provides more detailed information about each Component in @Components
# since hashes are not retained in the order specified, we still need
# @Components and @SubComponents to control install and menu order
# Only items listed in @Components and @SubComponents are considered for install
# As such, this may list some components which are N/A to the selected distro
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
	"opaconfig" =>	{ Name => "opaconfig",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => "", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
		# "ofed" is only used for source_comp
	"ofed_delta" =>	{ Name => "OFA_DELTA",
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					},
	"opa_stack" =>	{ Name => "OFA OPA Stack",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  # DriverSubdir varies per distro
					  PreReq => "", CoReq => " oftools ",
 						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => "",
					  StartComponents => [ "opa_stack" ],
					},
		# ibacm is used in rhel72 and other only
	"ibacm" =>		{ Name => "OFA IBACM",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ibacm" ],
					},
	"intel_hfi" =>	{ Name => "Intel HFI Components",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => " oftools ",
 						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "intel_hfi" ],
					},
	"oftools" =>	{ Name => "OPA Tools",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-Tools*.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => " opa_stack ",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ", # TBD
					  StartComponents => [ ],
					},
	"mpi_selector" =>	{ Name => "MPI selector",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 1, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"opa_stack_dev" => { Name => "OFA OPA Development",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"fastfabric" =>	{ Name => "FastFabric",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-Tools*.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack oftools ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ ],
					},
	"delta_ipoib" =>	{ Name => "OFA IP over IB",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "updates",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "delta_ipoib" ],
					},
	"mvapich2" =>	{ Name => "MVAPICH2 (verbs,gcc)",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack mpi_selector intel_hfi ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"openmpi" =>	{ Name => "OpenMPI (verbs,gcc)",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack mpi_selector intel_hfi ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"gasnet" =>	{ Name => "GASNet (hfi,gcc)",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack intel_hfi ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"openshmem" =>	{ Name => "OpenSHMEM (hfi,gcc)",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack gasnet intel_hfi ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"sandiashmem" =>	{ Name => "Sandia-OpenSHMEM ",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack intel_hfi ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"openmpi_gcc_cuda_hfi" =>{ Name => "OpenMPI (cuda,gcc)",
                                         DefaultInstall => $State_Install,
                                         SrcDir => file_glob ("./OFED_MPIS.*"),
                                         DriverSubdir => "",
                                         PreReq => " opa_stack intel_hfi mpi_selector ", CoReq => "",
                                         Hidden => 0, Disabled => 0,
                                         HasStart => 0, HasFirmware => 0, DefaultStart => 0,
                                         StartPreReq => "",
                                         StartComponents => [ ],
					},
	"mvapich2_gcc_hfi" =>	{ Name => "MVAPICH2 (hfi,gcc)",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob ("./OFED_MPIS.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack intel_hfi mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"mvapich2_intel_hfi" =>	{ Name => "MVAPICH2 (hfi,Intel)",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob ("./OFED_MPIS.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack intel_hfi mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"openmpi_gcc_hfi" =>	{ Name => "OpenMPI (hfi,gcc)",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob ("./OFED_MPIS.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack intel_hfi mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"openmpi_intel_hfi" =>	{ Name => "OpenMPI (hfi,Intel)",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob ("./OFED_MPIS.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack intel_hfi mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"delta_mpisrc" =>{ Name => "MPI Source",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack opa_stack_dev mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"opafm" =>	{ Name => "OPA FM",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-FM.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  #StartComponents => [ "qlgc_fm", "qlgc_fm_snmp"],
					  StartComponents => [ "opafm" ],
					},
	"opamgt_sdk" => { Name => "OPA Management SDK",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-Tools*.*"),
					  DriverSubdir => "",
					  Prereq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ ],
				  },
	"delta_debug" =>	{ Name => "OFA Debug Info",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"hfi1_uefi" =>		{ Name => "Pre-Boot Components",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
					  DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"rdma_ndd" =>              { Name => "RDMA NDD",
                                          DefaultInstall => $State_Install,
                                          SrcDir => file_glob("./IntelOPA-OFED_DELTA.*"),
                                          DriverSubdir => "",
                                          PreReq => " opa_stack ", CoReq => "",
                                          Hidden => 1, Disabled => 1,
                                          HasStart => 1, HasFirmware => 0, DefaultStart => 0,
                                          StartPreReq => " opa_stack ",
                                          StartComponents => [ "rdma_ndd" ],
                                        },
	);

	# translate from startup script name to component/subcomponent name
%StartupComponent = (
				"opa_stack" => "opa_stack",
				"opafm" => "opafm",
			);
	# has component been loaded since last configured autostart
%ComponentWasInstalled = (
				"opa_stack" => 0,
				"ibacm" => 0,
				"oftools" => 0,
				"mpi_selector" => 0,
				"opa_stack_dev" => 0,
				"fastfabric" => 0,
				"delta_ipoib" => 0,
				"mvapich2" => 0,
				"openmpi" => 0,
				"mvapich2_gcc_hfi" => 0,
				"mvapich2_intel_hfi" => 0,
				"openmpi_gcc_hfi" => 0,
				"openmpi_gcc_cuda_hfi" => 0,
				"openmpi_intel_hfi" => 0,
				"delta_mpisrc" => 0,
				"opafm" => 0,
				"opamgt_sdk" => 0,
				"hfi1_uefi" => 0,
				"delta_debug" => 0,
				"rdma_ndd" => 0,
				"sandiashmem" => 0,
			);

sub init_components
{
	# The component list has slight variations per distro
	# TBD - the DriverSubDir should be moved to a global per distro variable
	# and ComponentInfo.DriverSubDir should be changed to a simple boolean
	# to indicate if the given component has some drivers
	if ( "$CUR_VENDOR_VER" eq "ES72" ) {
		@Components = ( @Components_rhel72 );
		$ComponentInfo{'opa_stack'}{'DriverSubdir'} = "extra/ifs-kernel-updates";
		$ComponentInfo{'oftools'}{'PreReq'} = " opastack ibacm ";
	} elsif ( "$CUR_VENDOR_VER" eq "ES122" ) {
		@Components = ( @Components_sles12_sp2 );
		$ComponentInfo{'opa_stack'}{'DriverSubdir'} = "updates/ifs-kernel-updates";
		$ComponentInfo{'ibacm'}{'Hidden'} = 1;
		$ComponentInfo{'ibacm'}{'Disabled'} = 1;
	} elsif ( "$CUR_VENDOR_VER" eq "ES73" ) {
		@Components = ( @Components_rhel73 );
		$ComponentInfo{'opa_stack'}{'DriverSubdir'} = "extra/ifs-kernel-updates";
		$ComponentInfo{'ibacm'}{'Hidden'} = 1;
		$ComponentInfo{'ibacm'}{'Disabled'} = 1;
	} elsif ( "$CUR_VENDOR_VER" eq "ES123" ) {
		@Components = ( @Components_sles12_sp3 );
		$ComponentInfo{'opa_stack'}{'DriverSubdir'} = "updates/ifs-kernel-updates";
		$ComponentInfo{'ibacm'}{'Hidden'} = 1;
		$ComponentInfo{'ibacm'}{'Disabled'} = 1;
	} elsif ( "$CUR_VENDOR_VER" eq "ES74" ) {
		@Components = ( @Components_rhel74 );
		$ComponentInfo{'opa_stack'}{'DriverSubdir'} = "extra/ifs-kernel-updates";
		$ComponentInfo{'ibacm'}{'Hidden'} = 1;
		$ComponentInfo{'ibacm'}{'Disabled'} = 1;
	} elsif ( "$CUR_VENDOR_VER" eq "ES67" ) {
                @Components = ( @Components_rhel67 );
                $ComponentInfo{'opa_stack'}{'DriverSubdir'} = "updates";
                $ComponentInfo{'oftools'}{'PreReq'} = " opastack ibacm ";
	} else {
		@Components = ( @Components_other );
		$ComponentInfo{'opa_stack'}{'DriverSubdir'} = "updates";
		$ComponentInfo{'oftools'}{'PreReq'} = " opastack ibacm ";
	}
}

# ==========================================================================
# opaconfig installation
# This is a special WrapperComponent which only needs:
#	available, install and uninstall
# it cannot have startup scripts, dependencies, prereqs, etc

sub available_opaconfig
{
	my $srcdir=$ComponentInfo{'opaconfig'}{'SrcDir'};
	return (rpm_resolve("$srcdir/*/", "any", "opaconfig"));
}

sub installed_opaconfig
{
	return rpm_is_installed("opaconfig", "any");
}

sub installed_version_opaconfig
{
	my $version = rpm_query_version_release_pkg("opaconfig");
	return dot_version("$version");
}

sub media_version_opaconfig
{
	my $srcdir = $ComponentInfo{'opaconfig'}{'SrcDir'};
	my $rpm = rpm_resolve("$srcdir/RPMS/*/", "any", "opaconfig");
	my $version = rpm_query_version_release($rpm);
	return dot_version("$version");
}

sub build_opaconfig
{
	my $osver = $_[0];
	my $debug = $_[1];      # enable extra debug of build itself
	my $build_temp = $_[2]; # temp area for use by build
	my $force = $_[3];      # force a rebuild
	return 0;       # success
}

sub need_reinstall_opaconfig($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return "no";
}

sub check_os_prereqs_opaconfig
{
	return rpm_check_os_prereqs("opaconfig", "user");
}

sub preinstall_opaconfig
{
        my $install_list = $_[0];       # total that will be installed when done
        my $installing_list = $_[1];    # what items are being installed/reinstalled

        return 0;       # success
}

sub install_opaconfig
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled

	my $srcdir=$ComponentInfo{'opaconfig'}{'SrcDir'};
	NormalPrint("Installing $ComponentInfo{'opaconfig'}{'Name'}...\n");

	# New Install Code
	my $rpmfile = rpm_resolve("$srcdir/RPMS/*/", "any", "opaconfig");
	rpm_run_install($rpmfile, "any", " -U ");
	# New Install Code

	# remove the old style version file
	system("rm -rf $ROOT/$BASE_DIR/version");
	# version_wrapper is only for support (fetched in opacapture)
	system("echo '$VERSION' > $BASE_DIR/version_wrapper 2>/dev/null");
	# there is no ideal answer here, if we install updates separately
	# then upgrade or reinstall with wrapper, make sure we cleanup possibly old
	# opaconfig_* files
	system("rm -rf $ROOT/sbin/opa_config_ff");
	system("rm -rf $ROOT/sbin/opa_config_fm");
	system("rm -rf $ROOT/sbin/opa_config_srp");
	system("rm -rf $ROOT/sbin/opa_config_vnic");
	system("rm -rf $ROOT/sbin/opa_config_delta");

	$ComponentWasInstalled{'opaconfig'}=1;
}

sub postinstall_opaconfig
{
        my $install_list = $_[0];       # total that will be installed when done
        my $installing_list = $_[1];    # what items are being installed/reinstalled
}

sub uninstall_opaconfig
{
	my $install_list = $_[0];	# total that will be left installed when done
	my $uninstalling_list = $_[1];	# what items are being uninstalled

	NormalPrint("Uninstalling $ComponentInfo{'opaconfig'}{'Name'}...\n");

	# New Uninstall Code
	rpm_uninstall_list("any", "verbose", ("opaconfig") );
	# New Uninstall Code

	system("rm -rf $ROOT$BASE_DIR/version_wrapper");
	system("rm -rf $ROOT$BASE_DIR/osid_wrapper");
	# remove the old style version file
	system("rm -rf $ROOT/$BASE_DIR/version");
	system("rm -rf $ROOT/sbin/opaconfig");
	# there is no ideal answer here, if we install updates separately
	# then uninstall all with wrapper, make sure we cleanup
	system("rm -rf $ROOT/sbin/opa_config_ff");
	system("rm -rf $ROOT/sbin/opa_config_fm");
	system("rm -rf $ROOT/sbin/opa_config_srp");
	system("rm -rf $ROOT/sbin/opa_config_vnic");
	system("rm -rf $ROOT/sbin/opa_config_delta");
	system "rmdir $ROOT$BASE_DIR 2>/dev/null";	# remove only if empty
	system "rmdir $ROOT$OPA_CONFIG_DIR 2>/dev/null";	# remove only if empty
	$ComponentWasInstalled{'opaconfig'}=0;
}

# source a component specific comp.pl file which will provide the
# API for managing the component's installation state
sub source_comp
{
	my($comp)=$_[0];
	my($allow_install)=$_[1];	# if not INSTALL, don't use present dir
	#print "$comp: $ComponentInfo{$comp}{'SrcDir'}/comp.pl\n"; sleep 10;
	if ( $allow_install && -e "$ComponentInfo{$comp}{'SrcDir'}/comp.pl" ) {
		# TBD - odd that require treats as package, do seems to not eval
		# the ugly eval approach seems to work
		#do "$ComponentInfo{$comp}{'SrcDir'}/comp.pl";
		#print "do $ComponentInfo{$comp}{'SrcDir'}/comp.pl\n"; sleep 5;
		#require "$ComponentInfo{$comp}{'SrcDir'}/comp.pl";
		#do "$ComponentInfo{$comp}{'SrcDir'}/comp.pl";
		# print "$ComponentInfo{$comp}{'SrcDir'}/comp.pl\n"; sleep 10;
		eval `cat "$ComponentInfo{$comp}{'SrcDir'}/comp.pl"`;
		if ( "$@" ne "" ) {
			NormalPrint "$@\n";
			Abort "Corrupted $ComponentInfo{$comp}{'Name'} script: $ComponentInfo{$comp}{'SrcDir'}/comp.pl";
		} else {
			LogPrint "Loaded $ComponentInfo{$comp}{'Name'} script: $ComponentInfo{$comp}{'SrcDir'}/comp.pl\n";
		}
		#eval "available_$comp";
	} elsif ( -e "$ROOT/usr/lib/opa/.comp_$comp.pl" ) {
		# source the installed file, mainly to aid uninstall
		#print "$ROOT/usr/lib/opa/.comp_$comp.pl\n"; sleep 10;
		eval `cat "$ROOT/usr/lib/opa/.comp_$comp.pl"`;
		if ( "$@" ne "" ) {
			NormalPrint "$@\n";
			NormalPrint "Warning: Ignoring Corrupted $ComponentInfo{$comp}{'Name'} script: $ROOT/usr/lib/opa/.comp_$comp.pl\n";
			HitKeyCont;
		} else {
			LogPrint "Loaded $ComponentInfo{$comp}{'Name'} script: $ROOT/usr/lib/opa/.comp_$comp.pl\n";
		}
	} else {
		# component not available and not installed
		LogPrint "$ComponentInfo{$comp}{'Name'} script not available\n";
	}
}

my $allow_install;
if ( my_basename($0) ne "INSTALL" )
{
	$allow_install=0;
} else {
	$allow_install=1;
	$FabricSetupScpFromDir="..";
}

#
# Check if all basic requirements are ok
#
sub verify_os_ver 
{
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
		#printf STDERR "Usage: $0 [-r root] [-v|-vv] [-a|-n|-U|-F|-u|-s|-i comp|-e comp] [-E comp] [-D comp] [-f] [--fwupdate asneeded|always] [-l] [--user_configure_options 'options'] [--kernel_configure_options 'options'] [--prefix dir] [--without-depcheck] [--rebuild] [--force] [--answer keyword=value] [--debug]\n";
		#printf STDERR "Usage: $0 [-r root] [-v|-vv] [-a|-n|-U|-F|-u|-s|-i comp|-e comp] [-E comp] [-D comp] [-f] [--fwupdate asneeded|always] [--user_configure_options 'options'] [--kernel_configure_options 'options'] [--prefix dir] [--without-depcheck] [--rebuild] [--force] [--answer keyword=value]\n";
		printf STDERR "Usage: $0 [-r root] [-v|-vv] -R osver -B osver [-a|-n|-U|-u|-s|-O|-N|-i comp|-e comp] [-E comp] [-D comp] [--user-space] [--user_configure_options 'options'] [--kernel_configure_options 'options'] [--prefix dir] [--without-depcheck] [--rebuild] [--force] [--answer keyword=value]\n";
	} else {
#		printf STDERR "Usage: $0 [-r root] [-v|-vv] [-F|-u|-s|-e comp] [-E comp] [-D comp]\n";
#		printf STDERR "          [--fwupdate asneeded|always] [--user_queries|--no_user_queries] [--answer keyword=value]\n";
		printf STDERR "Usage: $0 [-r root] [-v|-vv] [-u|-s|-e comp] [-E comp] [-D comp]\n";
		printf STDERR "          [--user_queries|--no_user_queries] [--answer keyword=value]\n";
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
#		printf STDERR "       -f - skip HCA firmware upgrade during install\n";
		printf STDERR "       --user-space - Skip kernel space components during installation\n";
		#printf STDERR "       -l - skip creating/removing symlinks to /usr/local from /usr/lib/opa\n";
		printf STDERR "       --user_configure_options 'options' - specify additional OFA build\n";
		printf STDERR "             options for user space srpms.  Causes rebuild of all user srpms\n";
		printf STDERR "       --kernel_configure_options 'options' - specify additional OFA build\n";
		printf STDERR "             options for driver srpms.  Causes rebuild of all driver srpms\n";
		printf STDERR "       --prefix dir - specify alternate directory prefix for install of OFA\n";
		printf STDERR "             default is /usr.  Causes rebuild of needed srpms\n";
		printf STDERR "       --without-depcheck - disable check of OS dependencies\n";
		printf STDERR "       --rebuild - force OFA Delta rebuild\n";
		printf STDERR "       --force - force install even if distos don't match\n";
		printf STDERR "                 Use of this option can result in undefined behaviors\n";
		printf STDERR "       -O - Keep current modified rpm config file\n";
		printf STDERR "       -N - Use new default rpm config file\n";
		# --debug, -t and -d options are purposely not documented
		#printf STDERR "       --debug - build a debug version of modules\n";
		printf STDERR "       -B osver - run build for all components targetting kernel osver\n";
		#printf STDERR "       -t - temp area for use by builds, only valid with -B\n";
		#printf STDERR "       -d - enable build debugging assists, only valid with -B\n";
		printf STDERR "       -R osver - force install for kernel osver rather than running kernel.\n";
	}
#	printf STDERR "       -F - upgrade HCA Firmware with default options\n";
#	printf STDERR "       --fwupdate asneeded|always - select fw update auto update mode\n";
#	printf STDERR "            asneeded - update or downgrade to match version in this release\n";
#	printf STDERR "            always - rewrite with this releases version even if matches\n";
#	printf STDERR "            default is to upgrade as needed but not downgrade\n";
#	printf STDERR "            this option is ignored for interactive install\n";
	printf STDERR "       -u - uninstall all ULPs and drivers with default options\n";
	printf STDERR "       -s - enable autostart for all installed drivers\n";
	printf STDERR "       -r - specify alternate root directory, default is /\n";
	printf STDERR "       -e comp - uninstall the given component with default options\n";
	printf STDERR "            can appear more than once on command line\n";
	printf STDERR "       -E comp - enable autostart of given component\n";
	printf STDERR "            can appear with -D or more than once on command line\n";
	printf STDERR "       -D comp - disable autostart of given component\n";
	printf STDERR "            can appear with -E or more than once on command line\n";
	printf STDERR "       -G - install GPU Direct components(must have NVidia drivers installed)\n";
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
	printf STDERR "       supported component name aliases:\n";
	printf STDERR "            opa ipoib mpi psm_mpi verbs_mpi pgas mpisrc opadev\n";
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

my $Default_FirmwareUpgrade=0;	# -F option used to select default firmware upgrade

# translate an ibaccess component name into the corresponding list of ofed comps
# if the given name is invalid or has no corresponding OFED component
# returns an empty list
sub translate_comp
{
	my($arg)=$_[0];
	if ("$arg" eq "opadev")			{ return ( "opa_stack_dev" );
	} elsif ("$arg" eq "opa")		{ return ( "oftools",
		"mvapich2_gcc_hfi", "openmpi_gcc_hfi", "mvapich2_intel_hfi",
		"openmpi_intel_hfi");
	} elsif ("$arg" eq "ipoib")		{ return ( "delta_ipoib" );
	} elsif ("$arg" eq "mpi")		{ return ( "mvapich2", "openmpi", 
		"mvapich2_gcc_hfi", "openmpi_gcc_hfi", "mvapich2_intel_hfi", 
		"openmpi_intel_hfi");
	} elsif ("$arg" eq "psm_mpi")	{ return ( "mvapich2_gcc_hfi",
		"openmpi_gcc_hfi", "mvapich2_intel_hfi", 
		"openmpi_intel_hfi");
	} elsif ("$arg" eq "verbs_mpi")	{ return ( "mvapich2", "openmpi" );
	} elsif ("$arg" eq "pgas")		{ return ( "gasnet", "openshmem", "sandiashmem" );
	} elsif ("$arg" eq "mpisrc")	{ return ( "delta_mpisrc" );
		# no ibaccess argument equivalent for:  
		#	openmpi, delta_debug
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
	my $setanswer = 0;
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
	my $setfwmode = 0;
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
#			} elsif ( $setfwmode ) {
#				if ( "$arg" eq "always" || "$arg" eq "asneeded") {
#					$Default_FirmwareUpgradeMode="$arg";
#				} else {
#					printf STDERR "Invalid --fwupdate mode: $arg\n";
#					Usage;
#				}
#				$setfwmode = 0;
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
#			} elsif ( "$arg" eq "-f" ) {
#				$Skip_FirmwareUpgrade=1;
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
			} elsif ( "$arg" eq "--fwupdate" ) {
				$setfwmode=1;
			} elsif ( "$arg" eq "--answer" ) {
				$setanswer=1;
			} elsif ( "$arg" eq "--without-depcheck" ) {
				$rpm_check_dependencies=0;
			} elsif ( "$arg" eq "--user-space" ) {
				$skip_kernel = 1;
			} elsif ( "$arg" eq "--force" ) {
				$Force_Install=1;
			} elsif ( "$arg" eq "-G" ) {
				$GPU_Install=1;
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
#				} elsif ( "$arg" eq "-F" ) {
#					$Default_FirmwareUpgrade=1;
				} elsif ("$arg" eq "-O") {
					$Default_RpmConfigKeepOld=1;
					$Default_RpmConfigUseNew=0;
				} elsif ("$arg" eq "-N") {
					$Default_RpmConfigKeepOld=0;
					$Default_RpmConfigUseNew=1;
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
	if ( $setroot || $setcomp || $setenabled || $setdisabled  || $setosver || $setbuildtemp || $setuseroptions || $setkerneloptions || $setprefix || $setfwmode || $setanswer) {
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
		NormalPrint("Patching OFA...\n");
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
#   	if ( $Default_FirmwareUpgrade ) {
#		NormalPrint ("Upgrading HCA Firmware\n");
#		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 4);
#	}
   	if ($Default_Uninstall ) {
		NormalPrint ("Uninstalling All OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 6);
	}
   	if ($Default_CompUninstall ) {
		NormalPrint ("Uninstalling Selected OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 6);
	}
   	if ($Default_Autostart) {
		NormalPrint ("Configuring autostart for Selected installed OPA Drivers\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 3);
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
	printf ("   2) Reconfigure $ComponentInfo{'delta_ipoib'}{'Name'}\n");
	printf ("   3) Reconfigure Driver Autostart \n");
	printf ("   4) Generate Supporting Information for Problem Report\n");
	printf ("   5) FastFabric (Host/Chassis/Switch Setup/Admin)\n");
#	printf ("   6) Update HCA Firmware\n");
	$max_inp=5;
	if (!$allow_install)
	{
#		printf ("   7) Uninstall Software\n");
		printf ("   6) Uninstall Software\n");
		$max_inp=6;
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

determine_os_version;
init_components;

# when this is used as main for a component specific INSTALL
# the component can provide some overrides of global settings such as Components
overrides;

process_args;
check_root_user;
if ( ! $Default_Build ) {
	open_log("");
} else {
	open_log("./build.log");
}

source_comp("ofed_delta", $allow_install);
source_comp("oftools", $allow_install);	# allow oftools installed w/o fastfabric
source_comp("fastfabric", $allow_install);
source_comp("opafm", $allow_install);

if ( ! $Default_Build ) {
	verify_os_ver;
	verify_modtools;
	if ($allow_install) {
		verify_distrib_files;
	}
}

set_libdir;
init_delta_rpm_info($CUR_OS_VER);

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
	elsif ($INSTALL_CHOICE == 6)
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
		Config_ifcfg(1,"$ComponentInfo{'delta_ipoib'}{'Name'}","ib", "$FirstIPoIBInterface",1);
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
#	elsif ($INSTALL_CHOICE == 6)
#	{
#		# Update HFI Firmware
#		update_hca_firmware;
#		if (! $Default_Prompt ) {
#			goto RESTART;
#		} else {
#			print "Done HFI Firmware Update.\n"
#		}
#	}
	elsif ($INSTALL_CHOICE == 4) 
	{
		# Generate Supporting Information for Problem Report
		capture_report($ComponentInfo{'oftools'}{'Name'});
		goto RESTART;
	}
	elsif ($INSTALL_CHOICE == 5) 
	{
		# FastFabric (Host/Chassis/Switch Setup/Admin)
		if ( "$ROOT" eq "" || "$ROOT" eq "/" ) {
			run_fastfabric($ComponentInfo{'fastfabric'}{'Name'});
		} else {
			print "Unable to run fastfabric when using an alternate root (-r)\n";
			HitKeyCont;
		}
		goto RESTART;
	}
}
DONE:
close_log;
exit $exit_code;
