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

# ===========================================================================
# Main menus, option handling and version handling for FF for OFA install
#

my $Build_OsVer=$CUR_OS_VER;
my $Build_Debug=0;	# should we provide more info for debug
my $Build_Temp="";	# temp area to use for build
my $Default_Build = 0;	# -B option used to select build
my $Build_Force = 0;# rebuild option used to force full rebuild

$FirstIPoIBInterface=0; # first device is ib0

	# Names of supported install components
	# must be listed in dependency order such that prereqs appear 1st
	# delta_debug must be last

my @OmniPathAllComponents = (
	"oftools", "intel_hfi", "opa_stack_dev", "fastfabric",
	"delta_ipoib", "opafm", "opamgt_sdk",
	"mvapich2_gcc_hfi", "mvapich2_intel_hfi",
	"openmpi_gcc_hfi", "openmpi_intel_hfi",
	"openmpi_gcc_cuda_hfi", "sandiashmem",
	"mvapich2_gcc",	"openmpi_gcc", #"mpiRest",
	"mpisrc", "delta_debug"	);

my @Components_rhel72 = ( "opa_stack", "ibacm", "mpi_selector",
		@OmniPathAllComponents);
my @Components_sles12_sp2 = ( "opa_stack",
		@OmniPathAllComponents );
my @Components_rhel73 = ( "opa_stack", "mpi_selector",
		@OmniPathAllComponents );
my @Components_sles12_sp3 = ( "opa_stack",
		@OmniPathAllComponents );
my @Components_sles12_sp4 = ( "opa_stack",
		@OmniPathAllComponents );
my @Components_sles15 = ( "opa_stack",
		@OmniPathAllComponents );
my @Components_sles15_sp1 = ( "opa_stack",
		@OmniPathAllComponents );
my @Components_rhel74 = ( "opa_stack", "mpi_selector",
		@OmniPathAllComponents );
my @Components_rhel75 = ( "opa_stack", "mpi_selector",
		@OmniPathAllComponents );
my @Components_rhel76 = ( "opa_stack", "mpi_selector",
		@OmniPathAllComponents );
my @Components_rhel8 = ( "opa_stack", "mpi_selector",
		@OmniPathAllComponents );

@Components = ( );

# RHEL7.2, ibacm is a full component with rpms to install
my @SubComponents_older = ( "rdma_ndd", "delta_srp", "delta_srpt" );
# RHEL7.3 and newer AND SLES12.2 and newer
my @SubComponents_newer = ( "ibacm", "rdma_ndd", "delta_srp", "delta_srpt" );
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
%ComponentInfo = (
		# our special WrapperComponent, limited use
	"opaconfig" =>	{ Name => "opaconfig",
					  DefaultInstall => $State_Install,
					  SrcDir => ".",
					  PreReq => "", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]

					},
	"oftools" =>	{ Name => "OPA Tools",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-Tools*.*"),
					  PreReq => " opa_stack ", CoReq => " opa_stack ",
					  Hidden => 0, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ", # TBD
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"mpi_selector" =>	{ Name => "MPI selector",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 1, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "mpi-selector" ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"fastfabric" =>	{ Name => "FastFabric",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-Tools*.*"),
					  PreReq => " opa_stack oftools ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
# TBD - only a startup script, should act like a subcomponent only
	"delta_ipoib" =>	{ Name => "OFA IP over IB",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "delta_ipoib" ],
					  StartupScript => "",
					  StartupParams => [ "IPOIB_LOAD" ]
					},
	"mvapich2_gcc" =>	{ Name => "MVAPICH2 (verbs,gcc)",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => file_glob ("./OFA_MPIS.*"),
					  PreReq => " opa_stack mpi_selector intel_hfi ", CoReq => "",
					  Hidden => 1, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"openmpi_gcc" =>	{ Name => "OpenMPI (verbs,gcc)",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => file_glob ("./OFA_MPIS.*"),
					  PreReq => " opa_stack mpi_selector intel_hfi ", CoReq => "",
					  Hidden => 1, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"sandiashmem" =>	{ Name => "Sandia-OpenSHMEM ",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack intel_hfi ", CoReq => "",
# TBD - should IsOFA be 0?
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "sandia-openshmem_gcc_hfi",
									"sandia-openshmem_gcc_hfi-devel",
									"sandia-openshmem_gcc_hfi-tests"
								],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"openmpi_gcc_cuda_hfi" =>{ Name => "OpenMPI (cuda,gcc)",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob ("./OFA_MPIS.*"),
					  PreReq => " opa_stack intel_hfi mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"mvapich2_gcc_hfi" =>	{ Name => "MVAPICH2 (hfi,gcc)",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob ("./OFA_MPIS.*"),
					  PreReq => " opa_stack intel_hfi mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"mvapich2_intel_hfi" =>	{ Name => "MVAPICH2 (hfi,Intel)",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob ("./OFA_MPIS.*"),
					  PreReq => " opa_stack intel_hfi mpi_selector ", CoReq => "",
					  Hidden => 1, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"openmpi_gcc_hfi" =>	{ Name => "OpenMPI (hfi,gcc)",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob ("./OFA_MPIS.*"),
					  PreReq => " opa_stack intel_hfi mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"openmpi_intel_hfi" =>	{ Name => "OpenMPI (hfi,Intel)",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob ("./OFA_MPIS.*"),
					  PreReq => " opa_stack intel_hfi mpi_selector ", CoReq => "",
					  Hidden => 1, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
# rest of MPI stuff which customer can build via do_build
# this is included here so we can uninstall
# special case use in comp_delta.pl, omitted from Components list
# TBD - how to best refactor this so we remove these (do we still need to remove them?)
	"mpiRest" =>	{ Name => "MpiRest (pgi,Intel)",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => file_glob ("./OFA_MPIS.*"),
					  PreReq => "", CoReq => "",
					  Hidden => 1, Disabled => 1, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "mvapich2_pgi",
									"mvapich2_intel", "mvapich2_pathscale",
									"mvapich2_pgi_qlc", "mvapich2_gcc_qlc",
									"mvapich2_intel_qlc", "mvapich2_pathscale_qlc",
									"mvapich2_pgi_hfi", "mvapich2_pathscale_hfi",
									"mvapich2_pgi_cuda_hfi", "mvapich2_pathscale_cuda_hfi",
									"openmpi_pgi",
									"openmpi_intel", "openmpi_pathscale",
									"openmpi_pgi_qlc", "openmpi_gcc_qlc",
									"openmpi_intel_qlc", "openmpi_pathscale_qlc",
									"openmpi_pgi_hfi", "openmpi_pathscale_hfi",
									"openmpi_pgi_cuda_hfi", "openmpi_pathscale_cuda_hfi",
									"mpitests_mvapich2_pgi",
									"mpitests_mvapich2_intel",
									"mpitests_mvapich2_pathscale",
									"mpitests_mvapich2_pgi_qlc", "mpitests_mvapich2_gcc_qlc",
									"mpitests_mvapich2_intel_qlc", "mpitests_mvapich2_pathscale_qlc",
									"mpitests_mvapich2_pgi_hfi", "mpitests_mvapich2_pathscale_hfi",
									"mpitests_mvapich2_pgi_cuda_hfi", "mpitests_mvapich2_pathscale_cuda_hfi",
									"mpitests_openmpi_pgi",
									"mpitests_openmpi_intel",
									"mpitests_openmpi_pathscale",
									"mpitests_openmpi_pgi_qlc", "mpitests_openmpi_gcc_qlc",
									"mpitests_openmpi_intel_qlc", "mpitests_openmpi_pathscale_qlc",
									"mpitests_openmpi_pgi_hfi", "mpitests_openmpi_pathscale_hfi",
									"mpitests_openmpi_pgi_cuda_hfi", "mpitests_openmpi_pathscale_cuda_hfi",
								],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"mpisrc" =>{ Name => "MPI Source",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob ("./OFA_MPIS.*"),
					  PreReq => " opa_stack opa_stack_dev mpi_selector ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"opafm" =>	{ Name => "OPA FM",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-FM.*"),
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  #StartComponents => [ "qlgc_fm", "qlgc_fm_snmp"],
					  StartComponents => [ "opafm" ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	"opamgt_sdk" => { Name => "OPA Management SDK",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-Tools*.*"),
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 0,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
				  },
	"delta_debug" =>	{ Name => "OFA Debug Info",
					  DefaultInstall => $State_DoNotInstall,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ ],
					  DebugRpms => [ ],	# listed per comp
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
		# rdma_ndd is a subcomponent
		# it has a startup, but is considered part of opa_stack
	"rdma_ndd" =>		{ Name => "RDMA NDD",
					  PreReq => " opa_stack ",
					  HasStart => 1, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "rdma_ndd" ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
		# delta_srp is a subcomponent
		# it has a startup, but is considered part of opa_stack
	"delta_srp" =>		{ Name => "OFA SRP",
					  PreReq => " opa_stack ",
					  HasStart => 1, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "delta_srp" ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
		# delta_srpt is a subcomponent
		# it has a startup, but is considered part of opa_stack
	"delta_srpt" =>		{ Name => "OFA SRPT",
					  PreReq => " opa_stack ",
					  HasStart => 1, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "delta_srpt" ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
	);

# one of these opa_stack comp_info gets appended to ComponentInfo
# for RHEL72
my %opa_stack_rhel72_comp_info = (
	"opa_stack" =>	{ Name => "OFA OPA Stack",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => "", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ "kmod-ifs-kernel-updates" ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "opa-scripts",
									#"libibumad",
									"srptools",
									"libibmad",
									"infiniband-diags",
								],
					  DebugRpms => [ #"libibumad-debuginfo",
									"srptools-debuginfo",
								],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => "",
					  StartComponents => [ "opa_stack", "rdma_ndd", "delta_srp", "delta_srpt" ],
					  StartupScript => "opa",
					  StartupParams => [ "ARPTABLE_TUNING" ]
					},
);
# for RHEL73
my %opa_stack_rhel73_comp_info = (
	"opa_stack" =>	{ Name => "OFA OPA Stack",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => "", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ "kmod-ifs-kernel-updates" ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "opa-scripts",
									#"libibumad",
									#"srptools",
									#"libibmad",
									"infiniband-diags",
								],
					  DebugRpms => [ #"libibumad-debuginfo",
									#"srptools-debuginfo",
								],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => "",
					  StartComponents => [ "opa_stack", "ibacm", "rdma_ndd", "delta_srp", "delta_srpt" ],
					  StartupScript => "opa",
					  StartupParams => [ "ARPTABLE_TUNING" ]
					},
);
# for RHEL74 and above
my %opa_stack_rhel_comp_info = (
	"opa_stack" =>	{ Name => "OFA OPA Stack",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => "", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ "kmod-ifs-kernel-updates" ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "opa-scripts",
									#"libibumad",
									#"srptools",
									#"libibmad",
									#"infiniband-diags",
								],
					  DebugRpms => [ #"libibumad-debuginfo",
									#"srptools-debuginfo",
								],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => "",
					  StartComponents => [ "opa_stack", "ibacm", "rdma_ndd", "delta_srp", "delta_srpt" ],
					  StartupScript => "opa",
					  StartupParams => [ "ARPTABLE_TUNING" ]
					},
);
# for RHEL74 HFI2
my %opa_stack_rhel74_hfi2_comp_info = (
	"opa_stack" =>	{ Name => "OFA OPA Stack",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => "", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ "hfi2" ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "opa-scripts",
									#"libibumad",
									#"srptools",
									#"libibmad",
									#"infiniband-diags",
								],
					  DebugRpms => [ #"libibumad-debuginfo",
									#"srptools-debuginfo",
								],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => "",
					  StartComponents => [ "opa_stack", "ibacm", "rdma_ndd", "delta_srp", "delta_srpt" ],
					  StartupScript => "opa",
					  StartupParams => [ "ARPTABLE_TUNING" ]
					},
);
# for SLES12sp2
my %opa_stack_sles12_sp2_comp_info = (
	"opa_stack" =>	{ Name => "OFA OPA Stack",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => "", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ "ifs-kernel-updates-kmp-default" ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "opa-scripts",
									#"libibumad3",
									#"srptools",
									#"libibmad5",
									"infiniband-diags",
								],
					  DebugRpms => [ ],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => "",
					  StartComponents => [ "opa_stack", "ibacm", "rdma_ndd", "delta_srp", "delta_srpt" ],
					  StartupScript => "opa",
					  StartupParams => [ "ARPTABLE_TUNING" ]
					},
);
# for SLES12sp3
my %opa_stack_sles12_sp3_comp_info = (
	"opa_stack" =>	{ Name => "OFA OPA Stack",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => "", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ "ifs-kernel-updates-kmp-default" ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "opa-scripts",
									#"libibumad3",
									#"srptools",
									#"libibmad5",
									#"infiniband-diags",
								],
					  DebugRpms => [ ],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => "",
					  StartComponents => [ "opa_stack", "ibacm", "rdma_ndd", "delta_srp", "delta_srpt" ],
					  StartupScript => "opa",
					  StartupParams => [ "ARPTABLE_TUNING" ]
					},
);

# for SLES12sp4
my %opa_stack_sles12_sp4_comp_info = (
	"opa_stack" =>	{ Name => "OFA OPA Stack",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => "", CoReq => " oftools ",
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ "ifs-kernel-updates-kmp-default" ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "opa-scripts",
								],
					  DebugRpms => [ ],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => "",
					  StartComponents => [ "opa_stack", "ibacm", "rdma_ndd", "delta_srp", "delta_srpt" ],
					  StartupScript => "opa",
					  StartupParams => [ "ARPTABLE_TUNING" ]
					},
);

# for SLES15.x
my %opa_stack_sles15_comp_info = (
	"opa_stack" =>	{ Name => "OFA OPA Stack",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => "", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ "ifs-kernel-updates-kmp-default" ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "opa-scripts",
									#"libibumad3",
									#"srptools",
									#"libibmad5",
									#"infiniband-diags",
								],
					  DebugRpms => [ ],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => "",
					  StartComponents => [ "opa_stack", "ibacm", "rdma_ndd", "delta_srp", "delta_srpt" ],
					  StartupScript => "opa",
					  StartupParams => [ "ARPTABLE_TUNING" ]
					},
);


# one of these ibacm comp_info gets appended to ComponentInfo
# for RHEL72
my %ibacm_older_comp_info = (
	"ibacm" =>		{ Name => "OFA IBACM",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "ibacm",],
					  DebugRpms => [ "ibacm-debuginfo" ],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ibacm" ],
					  StartupScript => "ibacm",
					  StartupParams => [ ]
					},
);

# For RHEL73, SLES12sp2 and other newer distros
		# ibacm is a subcomponent
		# it has a startup, but is considered part of opa_stack
my %ibacm_comp_info = (
		# TBD - should be a StartComponent only for these distros
	"ibacm" =>		{ Name => "OFA IBACM",
					  PreReq => " opa_stack ",
					  HasStart => 1, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "ibacm" ],
					  StartupScript => "ibacm",
					  StartupParams => [ ]
					},
);


# one of these intel_hfi comp_info gets appended to ComponentInfo
# for RHEL72 which lacks libhfi1
my %intel_hfi_older_comp_info = (
	"intel_hfi" =>	{ Name => "Intel HFI Components",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [
									"hfi1-firmware", "hfi1-firmware_debug"
								],
					  UserRpms =>	  [ "libhfi1", "libhfi1-static",
									"libpsm2",
									"libpsm2-devel", "libpsm2-compat",
									"libfabric", "libfabric-devel",
									"libfabric-psm",
									"libfabric-psm2", "libfabric-verbs",
									"hfi1-diagtools-sw", "hfidiags",
								],
					  DebugRpms =>  [ "hfi1_debuginfo",
									"hfi1-diagtools-sw-debuginfo",
									"libpsm2-debuginfo", "libhfi1-debuginfo"
								],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "intel_hfi" ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
);
# for RHEL73, SLES12.2 and other newer distros which include libhfi1
my %intel_hfi_comp_info = (
	"intel_hfi" =>	{ Name => "Intel HFI Components",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [
									"hfi1-firmware", "hfi1-firmware_debug"
								],
					  UserRpms => [ #"libhfi1", "libhfi1-static",
									"libpsm2",
									"libpsm2-devel", "libpsm2-compat",
									"libfabric", "libfabric-devel",
									"libfabric-psm",
									"libfabric-psm2", "libfabric-verbs",
									"hfi1-diagtools-sw", "hfidiags",
								],
					  DebugRpms =>  [ #"hfi1_debuginfo",
									"hfi1-diagtools-sw-debuginfo",
									"libpsm2-debuginfo", #"libhfi1-debuginfo"
								],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "intel_hfi" ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
);
#
# for RHEL8, which does not currently support opa_stack and does not require libfabric-psm
my %intel_hfi_rhel8_comp_info = (
	"intel_hfi" =>	{ Name => "Intel HFI Components",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [
									"hfi1-firmware", "hfi1-firmware_debug"
								],
					  UserRpms => [ #"libhfi1", "libhfi1-static",
									"libpsm2",
									"libpsm2-devel", "libpsm2-compat",
									"libfabric", "libfabric-devel",
									"libfabric-psm2", "libfabric-verbs",
									"hfi1-diagtools-sw", "hfidiags",
								],
					  DebugRpms =>  [ #"hfi1_debuginfo",
									"hfi1-diagtools-sw-debuginfo",
									"libpsm2-debuginfo", #"libhfi1-debuginfo"
								],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "intel_hfi" ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
);

# For SLES12sp3 that has different name for libpsm2
my %intel_hfi_sles123_comp_info = (
	"intel_hfi" =>	{ Name => "Intel HFI Components",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [
									"hfi1-firmware", "hfi1-firmware_debug"
								],
					  UserRpms => [ #"libhfi1", "libhfi1-static",
									"libpsm2-2",
									"libpsm2-devel", "libpsm2-compat",
									"libfabric", "libfabric-devel",
									"libfabric-psm",
									"libfabric-psm2", "libfabric-verbs",
									"hfi1-diagtools-sw", "hfidiags",
								],
					  DebugRpms =>  [ #"hfi1_debuginfo",
									"hfi1-diagtools-sw-debuginfo",
									"libpsm2-debuginfo", #"libhfi1-debuginfo"
								],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "intel_hfi" ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
);

my %intel_hfi_sles124_comp_info = (
	"intel_hfi" =>	{ Name => "Intel HFI Components",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => " oftools ",
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [
									"hfi1-firmware", "hfi1-firmware_debug"
								],
					  UserRpms => [ 		"libpsm2-2",
									"libpsm2-devel", "libpsm2-compat",
									"libfabric", "libfabric-devel",
									"libfabric-psm",
									"libfabric-psm2", "libfabric-verbs",
									"hfi1-diagtools-sw", "hfidiags",
								],
					  DebugRpms =>  [ 		"hfi1-diagtools-sw-debuginfo",
									"libpsm2-debuginfo", 
								],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "intel_hfi" ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
);

# For SLES15.x that has different name for libpsm2
my %intel_hfi_sles15_comp_info = (
	"intel_hfi" =>	{ Name => "Intel HFI Components",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [
									"hfi1-firmware", "hfi1-firmware_debug"
								],
					  UserRpms => [ #"libhfi1", "libhfi1-static",
									"libpsm2-2",
									"libpsm2-devel", "libpsm2-compat",
									"libfabric", "libfabric-devel",
									"libfabric-psm",
									"libfabric-psm2", "libfabric-verbs",
									"hfi1-diagtools-sw", "hfidiags",
								],
					  DebugRpms =>  [ #"hfi1_debuginfo",
									"hfi1-diagtools-sw-debuginfo",
									"libpsm2-debuginfo", #"libhfi1-debuginfo"
								],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "intel_hfi" ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
);

# for RHEL74 HFI2
my %intel_hfi__rhel74_hfi2_comp_info = (
	"intel_hfi" =>	{ Name => "Intel HFI Components",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => " oftools ",
						# TBD - HasFirmware - FW update
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ ],
					  FirmwareRpms => [
									"hfi1-firmware", "hfi1-firmware_debug"
								],
					  UserRpms => [ #"libhfi1", "libhfi1-static",
									"libpsm2",
									"libpsm2-devel", "libpsm2-compat",
									"libfabric", "libfabric-devel",
									"libfabric-fxr",
									"libfabric-psm",
									"libfabric-psm2", "libfabric-verbs",
									"hfi1-diagtools-sw", "hfidiags",
								],
					  DebugRpms =>  [ #"hfi1_debuginfo",
									"hfi1-diagtools-sw-debuginfo",
									"libpsm2-debuginfo", #"libhfi1-debuginfo"
								],
					  HasStart => 1, HasFirmware => 0, DefaultStart => 1,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "intel_hfi" ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
);
# one of these opa_stack_dev comp_info gets appended to ComponentInfo
# for RHEL72
my %opa_stack_dev_rhel72_comp_info = (
	"opa_stack_dev" => { Name => "OFA OPA Development",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ "ifs-kernel-updates-devel" ],
					  FirmwareRpms => [ ],
					  UserRpms => [ "ibacm-devel",
									#"libibumad-devel", "libibumad-static",
									#"libibmad-devel", "libibmad-static"
									],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
					},
);
# for RHEL73 and SLES12sp2 and newer distros
my %opa_stack_dev_comp_info = (
	"opa_stack_dev" => { Name => "OFA OPA Development",
					  DefaultInstall => $State_Install,
					  SrcDir => file_glob("./IntelOPA-OFA_DELTA.*"),
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0, IsOFA => 1,
					  KernelRpms => [ "ifs-kernel-updates-devel" ],
					  FirmwareRpms => [ ],
					  UserRpms => [ #"ibacm-devel",
									#"libibumad-devel", "libibumad-static",
									#"libibmad-devel", "libibmad-static"
									],
					  DebugRpms => [ ],
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					  StartupScript => "",
					  StartupParams => [ ]
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
				"mvapich2_gcc" => 0,
				"openmpi_gcc" => 0,
				"mvapich2_gcc_hfi" => 0,
				"mvapich2_intel_hfi" => 0,
				"openmpi_gcc_hfi" => 0,
				"openmpi_gcc_cuda_hfi" => 0,
				"openmpi_intel_hfi" => 0,
				"mpiRest" => 0,
				"mpisrc" => 0,
				"opafm" => 0,
				"opamgt_sdk" => 0,
				"delta_debug" => 0,
				"rdma_ndd" => 0,
				"delta_srp" => 0,
				"delta_srpt" => 0,
				"sandiashmem" => 0,
			);

# check whether we support HFI2
sub support_HFI2()
{
	# right now we support HFI2 only in RHEL7.4 with kVer >= 4.6
	if ("$CUR_VENDOR_VER" eq "ES74" && $CUR_OS_VER_SHORT >= 4.6) {
		# We prefer to check it based on feature (i.e. hfi2 pkg) rather than version (i.e. content in version file)
		my $rpmdir = "$RPMS_SUBDIR/$CUR_DISTRO_VENDOR-$CUR_VENDOR_VER";
		my $srcdir = $opa_stack_rhel74_hfi2_comp_info{'opa_stack'}{'SrcDir'};
		my @rpmlist = @{$opa_stack_rhel74_hfi2_comp_info{'opa_stack'}{'KernelRpms'}};
		# this function get called before overrides, so we need to consider the case we run it from DELTA pkg that
		# srcdir is current location, i.e. "."
		return rpm_exists_list("$srcdir/$rpmdir", $CUR_OS_VER, @rpmlist) ||
		       rpm_exists_list("./$rpmdir", $CUR_OS_VER, @rpmlist);
	}
	return 0;
}

sub init_components
{
	$HFI2_INSTALL = support_HFI2();

	# The component list has slight variations per distro
	if ( "$CUR_VENDOR_VER" eq "ES72" ) {
		@Components = ( @Components_rhel72 );
		@SubComponents = ( @SubComponents_older );
		%ComponentInfo = ( %ComponentInfo, %ibacm_older_comp_info,
						%intel_hfi_older_comp_info,
						%opa_stack_dev_rhel72_comp_info,
						%opa_stack_rhel72_comp_info,
						);
		$ComponentInfo{'oftools'}{'PreReq'} = " opastack ibacm ";
	} elsif ( "$CUR_VENDOR_VER" eq "ES122" ) {
		@Components = ( @Components_sles12_sp2 );
		@SubComponents = ( @SubComponents_newer );
		%ComponentInfo = ( %ComponentInfo, %ibacm_comp_info,
						%intel_hfi_comp_info,
						%opa_stack_dev_comp_info,
						%opa_stack_sles12_sp2_comp_info,
						);
	} elsif ( "$CUR_VENDOR_VER" eq "ES73" ) {
		@Components = ( @Components_rhel73 );
		@SubComponents = ( @SubComponents_newer );
		%ComponentInfo = ( %ComponentInfo, %ibacm_comp_info,
						%intel_hfi_comp_info,
						%opa_stack_dev_comp_info,
						%opa_stack_rhel73_comp_info,
						);
	} elsif ( "$CUR_VENDOR_VER" eq "ES123" ) {
		@Components = ( @Components_sles12_sp3 );
		@SubComponents = ( @SubComponents_newer );
		%ComponentInfo = ( %ComponentInfo, %ibacm_comp_info,
						%intel_hfi_sles123_comp_info,
						%opa_stack_dev_comp_info,
						%opa_stack_sles12_sp3_comp_info,
						);
	} elsif ( "$CUR_VENDOR_VER" eq "ES124" ) {
		@Components = ( @Components_sles12_sp4 );
		@SubComponents = ( @SubComponents_newer );
		%ComponentInfo = ( %ComponentInfo, %ibacm_comp_info,
						%intel_hfi_sles124_comp_info,
						%opa_stack_dev_comp_info,
						%opa_stack_sles12_sp4_comp_info,
						);
	} elsif ( "$CUR_VENDOR_VER" eq "ES74" ) {
		@SubComponents = ( @SubComponents_newer );
		@Components = ( @Components_rhel74 );
		if ($HFI2_INSTALL) {
			%ComponentInfo = ( %ComponentInfo, %ibacm_comp_info,
							%intel_hfi__rhel74_hfi2_comp_info,
							%opa_stack_dev_comp_info,
							%opa_stack_rhel74_hfi2_comp_info,
							);
		} else {
			%ComponentInfo = ( %ComponentInfo, %ibacm_comp_info,
							%intel_hfi_comp_info,
							%opa_stack_dev_comp_info,
							%opa_stack_rhel_comp_info,
							);
		}
	} elsif ( "$CUR_VENDOR_VER" eq "ES75" ) {
		@Components = ( @Components_rhel75 );
		@SubComponents = ( @SubComponents_newer );
		%ComponentInfo = ( %ComponentInfo, %ibacm_comp_info,
						%intel_hfi_comp_info,
						%opa_stack_dev_comp_info,
						%opa_stack_rhel_comp_info,
						);
	} elsif ( "$CUR_VENDOR_VER" eq "ES76" ) {
		@Components = ( @Components_rhel76 );
		@SubComponents = ( @SubComponents_newer );
		%ComponentInfo = ( %ComponentInfo, %ibacm_comp_info,
						%intel_hfi_comp_info,
						%opa_stack_dev_comp_info,
						%opa_stack_rhel_comp_info,
						);
	} elsif ( "$CUR_VENDOR_VER" eq "ES8" ) {
		@Components = ( @Components_rhel8 );
		@SubComponents = ( @SubComponents_newer );
		%ComponentInfo = ( %ComponentInfo, %ibacm_comp_info,
						%intel_hfi_rhel8_comp_info,
						%opa_stack_dev_comp_info,
						%opa_stack_rhel_comp_info,
						);
	} elsif ( "$CUR_VENDOR_VER" eq "ES15" ) {
		@Components = ( @Components_sles15 );
		@SubComponents = ( @SubComponents_newer );
		%ComponentInfo = ( %ComponentInfo, %ibacm_comp_info,
						%intel_hfi_sles15_comp_info,
						%opa_stack_dev_comp_info,
						%opa_stack_sles15_comp_info,
						);
	} elsif ( "$CUR_VENDOR_VER" eq "ES151" ) {
		@Components = ( @Components_sles15_sp1 );
		@SubComponents = ( @SubComponents_newer );
		%ComponentInfo = ( %ComponentInfo, %ibacm_comp_info,
						%intel_hfi_sles15_comp_info,
						%opa_stack_dev_comp_info,
						%opa_stack_sles15_comp_info,
						);
	} else {
		# unknown or unsupported distro, leave lists empty
		# verify_distrib_files will catch unsupported distro
		@Components = ( );
		@SubComponents = ( );
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
	return (rpm_resolve("$srcdir/*/opaconfig", "any"));
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
	my $rpm = rpm_resolve("$srcdir/RPMS/*/opaconfig", "any");
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
	my $rpmfile = rpm_resolve("$srcdir/RPMS/*/opaconfig", "any");
	rpm_run_install($rpmfile, "any", " -U ");
	# New Install Code

	# remove the old style version file
	system("rm -rf /$BASE_DIR/version");
	# version_wrapper is only for support (fetched in opacapture)
	system("echo '$VERSION' > $BASE_DIR/version_wrapper 2>/dev/null");
	# make sure we cleanup possibly old opaconfig_* files
	system("rm -rf /sbin/opa_config_ff");
	system("rm -rf /sbin/opa_config_fm");
	system("rm -rf /sbin/opa_config_srp");
	system("rm -rf /sbin/opa_config_vnic");
	system("rm -rf /sbin/opa_config_delta");

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

	system("rm -rf $BASE_DIR/version_wrapper");
	system("rm -rf $BASE_DIR/osid_wrapper");
	# remove the old style version file
	system("rm -rf /$BASE_DIR/version");
	system("rm -rf /sbin/opaconfig");
	# there is no ideal answer here, if we install updates separately
	# then uninstall all with wrapper, make sure we cleanup
	system("rm -rf /sbin/opa_config_ff");
	system("rm -rf /sbin/opa_config_fm");
	system("rm -rf /sbin/opa_config_srp");
	system("rm -rf /sbin/opa_config_vnic");
	system("rm -rf /sbin/opa_config_delta");
	system "rmdir $BASE_DIR 2>/dev/null";	# remove only if empty
	system "rmdir $OPA_CONFIG_DIR 2>/dev/null";	# remove only if empty
	$ComponentWasInstalled{'opaconfig'}=0;
}

my $allow_install;
if ( my_basename($0) ne "INSTALL" )
{
	$allow_install=0;
} else {
	$allow_install=1;
	$FabricSetupScpFromDir="..";
}

sub Usage
{
	if ( $allow_install ) {
		#printf STDERR "Usage: $0 [-r root] [-v|-vv] -R osver -B osver [-d][-t tempdir] [--prefix dir] [--without-depcheck] [--rebuild] [--force] [--answer keyword=value]\n";
		#printf STDERR "               or\n";
		#printf STDERR "Usage: $0 [-r root] [-v|-vv] [-a|-n|-U|-F|-u|-s|-i comp|-e comp] [-E comp] [-D comp] [-f] [--fwupdate asneeded|always] [-l] [--prefix dir] [--without-depcheck] [--rebuild] [--force] [--answer keyword=value]\n";
		#printf STDERR "Usage: $0 [-r root] [-v|-vv] [-a|-n|-U|-F|-u|-s|-i comp|-e comp] [-E comp] [-D comp] [-f] [--fwupdate asneeded|always] [--prefix dir] [--without-depcheck] [--rebuild] [--force] [--answer keyword=value]\n";
		printf STDERR "Usage: $0 [-v|-vv] -R osver [-a|-n|-U|-u|-s|-O|-N|-i comp|-e comp] [-G] [-E comp] [-D comp] [--user-space] [--without-depcheck] [--rebuild] [--force] [--answer keyword=value]\n";
	} else {
#		printf STDERR "Usage: $0 [-r root] [-v|-vv] [-F|-u|-s|-e comp] [-E comp] [-D comp]\n";
#		printf STDERR "          [--fwupdate asneeded|always] [--user_queries|--no_user_queries] [--answer keyword=value]\n";
		printf STDERR "Usage: $0 [-v|-vv] [-u|-s|-e comp] [-E comp] [-D comp]\n";
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
		printf STDERR "       --user-space - Skip kernel space and firmware packages during installation\n";
		printf STDERR "            can be useful when installing into a container\n";
		#printf STDERR "       -l - skip creating/removing symlinks to /usr/local from /usr/lib/opa\n";
		printf STDERR "       --without-depcheck - disable check of OS dependencies\n";
		printf STDERR "       --rebuild - force OFA Delta rebuild\n";
		printf STDERR "       --force - force install even if distro don't match\n";
		printf STDERR "                 Use of this option can result in undefined behaviors\n";
		printf STDERR "       -O - Keep current modified rpm config file\n";
		printf STDERR "       -N - Use new default rpm config file\n";

		# -B, -t and -d options are purposely not documented
		#printf STDERR "       -B osver - run build for all components targetting kernel osver\n";
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
	ShowComponents(\*STDERR);
	printf STDERR "       supported component name aliases:\n";
	printf STDERR "            opa ipoib mpi psm_mpi verbs_mpi pgas opadev\n";
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

# translate an ibaccess component name into the corresponding list of OFA comps
# if the given name is invalid or has no corresponding OFA component
# returns an empty list
sub translate_comp
{
	my($arg)=$_[0];
	if ("$arg" eq "opadev") {
		return ( "opa_stack_dev" );
	} elsif ("$arg" eq "opa"){
		return ("oftools","mvapich2_gcc_hfi", "openmpi_gcc_hfi");
	} elsif ("$arg" eq "ipoib"){
		return ( "delta_ipoib" );
	} elsif ("$arg" eq "mpi"){
		return ( "mvapich2_gcc_hfi","openmpi_gcc_hfi");
	} elsif ("$arg" eq "psm_mpi"){
		return ( "mvapich2_gcc_hfi","openmpi_gcc_hfi");
	} elsif ("$arg" eq "pgas"){
	        return ( "sandiashmem" );
	} elsif ("$arg" eq "delta_mpisrc"){
		return ( "mpisrc" ); # legacy
		# no ibaccess argument equivalent for:
		#	delta_debug
		#
	} else {
		return ();	# invalid name
	}
}

sub process_args
{
	my $arg;
	my $last_arg;
	my $install_opt = 0;
	my $setcomp = 0;
	my $setanswer = 0;
	my $setenabled = 0;
	my $setdisabled = 0;
	my $setosver = 0;
	my $setbuildtemp = 0;
	my $comp = 0;
	my $osver = 0;
	my $setcurosver = 0;
	my $setfwmode = 0;
	my $patch_ofed=0;

	if (scalar @ARGV >= 1) {
		foreach $arg (@ARGV) {
			if ( $setanswer ) {
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
				$setcurosver=0;
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
			} elsif ( "$arg" eq "--fwupdate" ) {
				$setfwmode=1;
			} elsif ( "$arg" eq "--answer" ) {
				$setanswer=1;
			} elsif ( "$arg" eq "--without-depcheck" ) {
				$rpm_check_dependencies=0;
			} elsif ( "$arg" eq "--user-space" ) {
				$user_space_only = 1;
			} elsif ( "$arg" eq "--force" ) {
				$Force_Install=1;
			} elsif ( "$arg" eq "-G") {
				if ($HFI2_INSTALL) {
					printf STDERR "Invalid option: $arg not supported in 11.x OPA for HFI2.\n";
					Usage;
				} else {
					$GPU_Install=1;
				}
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
				if ( $Default_RpmConfigKeepOld || $Default_RpmConfigUseNew) {
					$Default_Prompt=0;
				} else {
					$Default_Prompt=1;
				}
			}
			$last_arg=$arg;
		}
	}
	if ( $setcomp || $setenabled || $setdisabled  || $setosver || $setbuildtemp || $setfwmode || $setanswer) {
		printf STDERR "Missing argument for option: $last_arg\n";
		Usage;
	}
	if ( ($Default_Install || $Default_CompInstall || $Default_Upgrade
			|| $Force_Install)
         && ! $allow_install) {
		printf STDERR "Installation options not permitted in this mode\n";
		Usage;
	}
	if ( ($Default_Build || $OFED_force_rebuild ) && ! $allow_install) {
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

	while( $inp < 1 || $inp > $max_inp) {
		$inp = getch();

		if ($inp =~ /[qQ]/ || $inp =~ /[Xx]/ ) {
			return 1;
		}
		if ($inp =~ /[0123456789abcdefABCDEF]/)
		{
			$inp = hex($inp);
		}
	}
	@INSTALL_CHOICES = ( $inp );
	return 0;
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

if ( ! $Default_Build ) {
	verify_modtools;
	if ($allow_install) {
		verify_distrib_files;
	}
}

set_libdir;
init_delta_info($CUR_OS_VER);

do{
	if ($Default_Build) {
		$exit_code = build_all_components($Build_OsVer, $Build_Debug, $Build_Temp, $Build_Force);
		done();
	} else {
		if ( show_menu != 0) {
		done();
		}
	}

	foreach my $INSTALL_CHOICE ( @INSTALL_CHOICES )
	{
		if ($allow_install && $INSTALL_CHOICE == 1)
		{
			select_debug_release(".");
			show_install_menu(1);
			if ($Default_Prompt) {
				if ($exit_code == 0) {
					print "Done Installing OPA Software.\n"
				} else {
					print "Failed to install all OPA software.\n"
				}
			}
		}
		elsif ($INSTALL_CHOICE == 1) {
			show_installed(1);
		}
		elsif ($INSTALL_CHOICE == 6)
		{
			show_uninstall_menu(1);
			if ( $Default_Prompt ) {
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
		}
		elsif ($INSTALL_CHOICE == 3)
		{
			reconfig_autostart;
			if ( $Default_Prompt ) {
				print "Done OPA Driver Autostart Configuration.\n"
			}
		}
		elsif ($INSTALL_CHOICE == 4)
		{
			# Generate Supporting Information for Problem Report
			capture_report($ComponentInfo{'oftools'}{'Name'});
		}
		elsif ($INSTALL_CHOICE == 5)
		{
			# FastFabric (Host/Chassis/Switch Setup/Admin)
			run_fastfabric($ComponentInfo{'fastfabric'}{'Name'});
		}
	}
}while( !$Default_Prompt );
done();
sub done() {
	if ( not $user_space_only ) {
		do_rebuild_ramdisk;
	}
	close_log;
	exit $exit_code;
}
