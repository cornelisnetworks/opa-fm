#!/usr/bin/perl
# BEGIN_ICS_COPYRIGHT8
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
# OFED_Delta installation, includes Intel value-adding packages only

my $default_prefix="/usr";

my $RPMS_SUBDIR = "RPMS";	# within ComponentInfo{'opa_stack'}{'SrcDir'}
my $SRPMS_SUBDIR = "SRPMS";	# within ComponentInfo{'opa_stack'}{'SrcDir'}

# list of components which are part of OFED
# can be a superset or subset of @Components
# ofed_vnic
my @delta_components_other = (
				"opa_stack", 		# Kernel drivers.
				"ibacm", 			# OFA IB communication manager assistant.
				"intel_hfi", 		# HFI drivers
				"delta_ipoib", 		# ipoib module.
				"mpi_selector",
				"mvapich2",
				"openmpi",
				"gasnet",
				"openshmem",
				"opa_stack_dev", 	# dev libraries.
				"delta_mpisrc", 	# Source bundle for MPIs.
				"mpiRest",			# PGI, Intel mpi variants.
				"hfi1_uefi",
				"delta_debug",		# must be last real component
);

my @delta_components_rhel72 = ( @delta_components_other );

my @delta_components_rhel67 = ( @delta_components_other );

my @delta_components_rhel70 = (
				"opa_stack", 		# Kernel drivers.
				"ibacm", 		# OFA IB communication manager assistant.
				"intel_hfi", 		# HFI drivers
				"delta_ipoib", 		# ipoib module.
				"mpi_selector",
				"mvapich2",
				"openmpi",
				"gasnet",
				"openshmem",
				"opa_stack_dev", 	# dev libraries.
				"delta_mpisrc", 	# Source bundle for MPIs.
				"mpiRest",			# PGI, Intel mpi variants.
				"delta_debug",		# must be last real component
);

my @delta_components_sles = ( @delta_components_other );

my @delta_components_rhel70 = (
				"opa_stack", 		# Kernel drivers.
				"ibacm", 		# OFA IB communication manager assistant.
				"intel_hfi", 		# HFI drivers
				"delta_ipoib", 		# ipoib module.
				"mpi_selector",
				"mvapich2",
				"openmpi",
				"gasnet",
				"openshmem",
				"opa_stack_dev", 	# dev libraries.
				"delta_mpisrc", 	# Source bundle for MPIs.
				"mpiRest",			# PGI, Intel mpi variants.
				"delta_debug",		# must be last real component
);

my @delta_components_sles12_sp2 = (
				"opa_stack", 		# Kernel drivers.
				"ibacm",
				"intel_hfi", 		# HFI drivers
				"delta_ipoib", 		# ipoib module.
				"mpi_selector",
				"mvapich2",
				"openmpi",
				"gasnet",
				"openshmem",
				"opa_stack_dev", 	# dev libraries.
				"delta_mpisrc", 	# Source bundle for MPIs.
				"mpiRest",			# PGI, Intel mpi variants.
				"hfi1_uefi",
				"delta_debug",		# must be last real component
);

my @delta_components_rhel73 = (
				"opa_stack", 		# Kernel drivers.
				"ibacm",
				"intel_hfi", 		# HFI drivers
				"delta_ipoib", 		# ipoib module.
				"mpi_selector",
				"mvapich2",
				"openmpi",
				"gasnet",
				"openshmem",
				"opa_stack_dev", 	# dev libraries.
				"delta_mpisrc", 	# Source bundle for MPIs.
				"mpiRest",			# PGI, Intel mpi variants.
				"hfi1_uefi",
				"delta_debug",		# must be last real component
);

my @delta_components = ( );


# information about each component in delta_components
# Fields:
#	KernelRpms => kernel rpms for given component, in dependency order
#				  These are package packages which are kernel version specific and
#				  will have kernel uname -r in rpm package name.
#				  For a given distro a separate version of each of these rpms
#				  may exist per kernel.  These are always architecture dependent
#	UserRpms => user rpms for given component, in dependency order
#				These are rpms which are not kernel specific.  For a given
#				distro a single version of each of these rpms will
#				exist per distro/arch combination.  Some of these may
#				be architecture independent (noarch).
#	Drivers => used so we can load compat-rdma rpm, then remove drivers for
#				ULPs which are not desired.  Ugly but a workaround for fact
#				that compat-rdma has all the ULP drivers in it
#				specifies drivers and subdirs which are within compat-rdma and
#				specific to each component
#	StartupScript => name of startup script which controls startup of this
#					component
#	StartupParams => list of parameter names in $OPA_CONFIG which control
#					startup of this component (set to yes/no values)
#
# Note KernelRpms are always installed before UserRpms
my %ibacm_comp_info = (
	'ibacm' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "ibacm", ],
					DebugRpms =>  [ "ibacm-debuginfo" ],
					Drivers => "", # none
					StartupScript => "ibacm",
					StartupParams => [ ]
					},
);

my %ibacm_sles12_sp2_comp_info = (
        'ibacm' => {
                                        KernelRpms => [ ],
                                        UserRpms =>       [ ],
                                        DebugRpms =>  [ ],
                                        Drivers => "", # none
                                        StartupScript => "ibacm",
                                        StartupParams => [ ]
                                        },
);

my %ibacm_rhel73_comp_info = (
        'ibacm' => {
                                        KernelRpms => [ ],
                                        UserRpms =>       [ ],
                                        DebugRpms =>  [ ],
                                        Drivers => "", # none
                                        StartupScript => "ibacm",
                                        StartupParams => [ ]
                                        },
);

my %intel_hfi_comp_info = (
	'intel_hfi' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "libhfi1", "libhfi1-static",
							    "libpsm2", 
							    "libpsm2-devel", "libpsm2-compat",
							    "hfi1-diagtools-sw", "hfidiags", 
							    "hfi1-firmware", "hfi1-firmware_debug" 
					    ],
					DebugRpms =>  [ "hfi1_debuginfo",
							"hfi1-diagtools-sw-debuginfo",
							"libpsm2-debuginfo", "libhfi1-debuginfo" 
					    ],
					Drivers => "",
					StartupScript => "",
					StartupParams => [  ],
					},
);

my %intel_hfi_sles12_sp2_comp_info = (
	'intel_hfi' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "libpsm2", 
							    "libpsm2-devel", "libpsm2-compat",
							    "hfi1-diagtools-sw", "hfidiags", 
							    "hfi1-firmware", "hfi1-firmware_debug" 
					    ],
					DebugRpms =>  [ "hfi1-diagtools-sw-debuginfo",
							"libpsm2-debuginfo" 
					    ],
					Drivers => "",
					StartupScript => "",
					StartupParams => [  ],
					},
);

my %intel_hfi_rhel73_comp_info = (
	'intel_hfi' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "libpsm2", 
							    "libpsm2-devel", "libpsm2-compat",
							    "hfi1-diagtools-sw", "hfidiags", 
							    "hfi1-firmware", "hfi1-firmware_debug" 
					    ],
					DebugRpms =>  [ "hfi1-diagtools-sw-debuginfo",
							"libpsm2-debuginfo" 
					    ],
					Drivers => "",
					StartupScript => "",
					StartupParams => [  ],
					},
);

my %ib_wfr_lite_comp_info = (
	'ib_wfr_lite' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "ib_wfr_lite", ],
					DebugRpms =>  [ "ib_wfr_lite-debuginfo", ],
					Drivers => "",
					StartupScript => "",
					StartupParams => [  ],
					},
);

my %delta_ipoib_comp_info = (
	'delta_ipoib' => {
					KernelRpms => [ ],
					UserRpms =>	  [ ],
					DebugRpms =>  [ ],
					Drivers => "",
					StartupScript => "",
					StartupParams => [ "IPOIB_LOAD" ],
					},
);

my %mpi_selector_comp_info = (
	'mpi_selector' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "mpi-selector" ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %mvapich2_comp_info = (
	'mvapich2' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "mvapich2_gcc", "mpitests_mvapich2_gcc", ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %openmpi_comp_info = (
	'openmpi' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "openmpi_gcc", "mpitests_openmpi_gcc" ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %gasnet_comp_info = (
	'gasnet' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "gasnet_gcc_hfi", "gasnet_gcc_hfi-devel" , "gasnet_gcc_hfi-tests" ],
					DebugRpms =>  [ "gasnet_gcc_hfi-debuginfo" ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %openshmem_comp_info = (
	'openshmem' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "openshmem_gcc_hfi", "openshmem-test-suite_gcc_hfi", "shmem-benchmarks_gcc_hfi"  ],
					DebugRpms =>  [ "openshmem_gcc_hfi-debuginfo", "openshmem-test-suite_gcc_hfi-debuginfo",
							"shmem-benchmarks_gcc_hfi-debuginfo" ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %opa_stack_dev_comp_info = (
	'opa_stack_dev' => {
					KernelRpms => [ "" ],
					UserRpms =>	  [ "compat-rdma-devel", "ibacm-devel",
							    "libibumad-devel", "libibumad-static",
							    "libibmad-devel", "libibmad-static"
								  ],
					DebugRpms =>  [  ],
					Drivers => "", 	# none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %opa_stack_dev_rhel70_comp_info = (
        'opa_stack_dev' => {
                                        KernelRpms => [ "" ],
                                        UserRpms =>       [ "compat-rdma-devel", "ibacm-devel",
                                                            "libibumad-devel", "libibumad-static",
                                                                  ],
                                        DebugRpms =>  [  ],
                                        Drivers => "",  # none
                                        StartupScript => "",
                                        StartupParams => [ ],
                                        },
);

my %opa_stack_dev_rhel67_comp_info = (
	'opa_stack_dev' => {
					KernelRpms => [ "ifs-kernel-updates-devel" ],
					UserRpms =>	  [ "ibacm-devel",
						            "libibumad-devel", "libibumad-static"
								  ],
					DebugRpms =>  [  ],
					Drivers => "", 	# none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %opa_stack_dev_rhel72_comp_info = (
	'opa_stack_dev' => {
					KernelRpms => [ ],
					UserRpms =>  [ "ifs-kernel-updates-devel", "ibacm-devel" ],
					DebugRpms =>  [  ],
					Drivers => "", 	# none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %opa_stack_dev_sles12_sp2_comp_info = (
	'opa_stack_dev' => {
					KernelRpms => [ ],
					UserRpms =>  [ "ifs-kernel-updates-devel" ],
					DebugRpms =>  [  ],
					Drivers => "", 	# none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %opa_stack_dev_rhel73_comp_info = (
	'opa_stack_dev' => {
					KernelRpms => [ ],
					UserRpms =>  [ "ifs-kernel-updates-devel" ],
					DebugRpms =>  [  ],
					Drivers => "", 	# none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %delta_mpisrc_comp_info = (
	'delta_mpisrc' => {	# nothing to build, just copies srpms
					KernelRpms => [ ],
					UserRpms =>	  [ ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %mpiRest_comp_info = (
	'mpiRest' => {	# rest of MPI stuff which customer can build via do_build
					# this is included here so we can uninstall
					KernelRpms => [ ],
					UserRpms =>	  [
									"mvapich2_pgi",
									"mvapich2_intel", "mvapich2_pathscale",
									"openmpi_pgi",
									"openmpi_intel", "openmpi_pathscale",
									"mpitests_mvapich2_pgi",
									"mpitests_mvapich2_intel",
									"mpitests_mvapich2_pathscale",
									"mpitests_openmpi_pgi",
									"mpitests_openmpi_intel",
									"mpitests_openmpi_pathscale",
								  ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %hfi1_uefi_comp_info = (
	'hfi1_uefi' =>  {
					KernelRpms => [ ],
					UserRpms =>   [ "hfi1-uefi" ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %delta_debug_comp_info = (
	'delta_debug' => {
					KernelRpms => [ ],
					UserRpms =>	  [ ],
					DebugRpms =>  [ ],	# listed by comp above
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
);

my %opa_stack_other_comp_info = (
	'opa_stack' => {
					KernelRpms => [ "compat-rdma" ], # special case
					UserRpms =>	  [ "opa-scripts",
								"libibumad",
								"srptools",
								"libibmad",
								"infiniband-diags",
								  ],
					DebugRpms =>  [ "libibumad-debuginfo",
									"srptools-debuginfo",
								  ],
					Drivers => "", 
					StartupScript => "opa",
					StartupParams => [ "ARPTABLE_TUNING" ],
					},
);

my %opa_stack_rhel67_comp_info = (
	'opa_stack' => {
					KernelRpms => [ "ifs-kernel-updates" ], # special case
					UserRpms =>	  [ "opa-scripts",
								"libibumad",
								"srptools",
								"libibmad",
								"infiniband-diags",
								  ],
					DebugRpms =>  [ "libibumad-debuginfo",
									"srptools-debuginfo",
								  ],
					Drivers => "", 
					StartupScript => "opa",
					StartupParams => [ "ARPTABLE_TUNING" ],
					},
);

my %opa_stack_rhel72_comp_info = (
	'opa_stack' => {
					KernelRpms => [ "kmod-ifs-kernel-updates" ], # special case
					UserRpms =>	  [ "opa-scripts",
								"srptools",
								"libibmad",
								"infiniband-diags",
								  ],
					DebugRpms =>  [ "srptools-debuginfo",
								  ],
					Drivers => "",
					StartupScript => "opa",
					StartupParams => [ "ARPTABLE_TUNING" ],
					},
);

my %opa_stack_rhel70_comp_info = (
	'opa_stack' => {
					KernelRpms => [ "compat-rdma" ], # special case
					UserRpms =>	  [ "opa-scripts",
								"libibumad",
								"srptools",
								"libibmad",
								"infiniband-diags",
								  ],
					DebugRpms =>  [ "libibumad-debuginfo",
									"srptools-debuginfo",
								  ],
					Drivers => "", 
					StartupScript => "opa",
					StartupParams => [ "ARPTABLE_TUNING" ],
					},
);

my %opa_stack_sles_comp_info = (
	'opa_stack' => {
					KernelRpms => [ "compat-rdma" ], # special case
					UserRpms =>	  [ "opa-scripts",
								"libibumad3",
								"srptools",
								"libibmad5",
								"infiniband-diags",
								  ],
					DebugRpms =>  [ ],
					Drivers => "", 
					StartupScript => "opa",
					StartupParams => [ "ARPTABLE_TUNING" ],
					},
);

my %opa_stack_sles12_sp2_comp_info = (
	'opa_stack' => {
					KernelRpms => [ "ifs-kernel-updates-kmp-default" ], # special case
					UserRpms =>   [ "opa-scripts",
								"infiniband-diags" ],
					DebugRpms =>  [  ],
					Drivers => "",
					StartupScript => "opa",
					StartupParams => [ "ARPTABLE_TUNING" ],
					},
);

my %opa_stack_rhel73_comp_info = (
	'opa_stack' => {
					KernelRpms => [ "kmod-ifs-kernel-updates" ], # special case
					UserRpms =>   [ "opa-scripts",
								"infiniband-diags" ],
					DebugRpms =>  [  ],
					Drivers => "",
					StartupScript => "opa",
					StartupParams => [ "ARPTABLE_TUNING" ],
					},
);

my %delta_comp_info_other = (
	%opa_stack_other_comp_info,
	%ibacm_comp_info,
	%intel_hfi_comp_info,
	%ib_wfr_lite_comp_info,
	%delta_ipoib_comp_info,
	%mpi_selector_comp_info,
	%mvapich2_comp_info,
	%openmpi_comp_info,
	%gasnet_comp_info,
	%openshmem_comp_info,
	%opa_stack_dev_comp_info,
	%delta_mpisrc_comp_info,
	%mpiRest_comp_info,
	%hfi1_uefi_comp_info,
	%delta_debug_comp_info,
);

my %delta_comp_info_rhel67 = (
	%opa_stack_rhel67_comp_info,
	%ibacm_comp_info,
	%intel_hfi_comp_info,
	%ib_wfr_lite_comp_info,
	%delta_ipoib_comp_info,
	%mpi_selector_comp_info,
	%mvapich2_comp_info,
	%openmpi_comp_info,
	%gasnet_comp_info,
	%openshmem_comp_info,
	%opa_stack_dev_rhel67_comp_info,
	%delta_mpisrc_comp_info,
	%mpiRest_comp_info,
	%hfi1_uefi_comp_info,
	%delta_debug_comp_info,
);

my %delta_comp_info_rhel72 = (
	%opa_stack_rhel72_comp_info,
	%ibacm_comp_info,
	%intel_hfi_comp_info,
	%ib_wfr_lite_comp_info,
	%delta_ipoib_comp_info,
	%mpi_selector_comp_info,
	%mvapich2_comp_info,
	%openmpi_comp_info,
	%gasnet_comp_info,
	%openshmem_comp_info,
	%opa_stack_dev_rhel72_comp_info,
	%delta_mpisrc_comp_info,
	%mpiRest_comp_info,
	%hfi1_uefi_comp_info,
	%delta_debug_comp_info,
);

my %delta_comp_info_rhel70 = (
	%opa_stack_rhel70_comp_info,
	%ibacm_comp_info,
	%intel_hfi_comp_info,
	%ib_wfr_lite_comp_info,
	%delta_ipoib_comp_info,
	%mpi_selector_comp_info,
	%mvapich2_comp_info,
	%openmpi_comp_info,
	%gasnet_comp_info,
	%openshmem_comp_info,
	%opa_stack_dev_rhel70_comp_info,
	%delta_mpisrc_comp_info,
	%mpiRest_comp_info,
	%delta_debug_comp_info,
);

my %delta_comp_info_sles = (
	%opa_stack_sles_comp_info,
	%ibacm_comp_info,
	%intel_hfi_comp_info,
	%ib_wfr_lite_comp_info,
	%delta_ipoib_comp_info,
	%mpi_selector_comp_info,
	%mvapich2_comp_info,
	%openmpi_comp_info,
	%gasnet_comp_info,
	%openshmem_comp_info,
	%opa_stack_dev_comp_info,
	%delta_mpisrc_comp_info,
	%mpiRest_comp_info,
	%hfi1_uefi_comp_info,
	%delta_debug_comp_info,
);

my %delta_comp_info_sles12_sp2 = (
	%opa_stack_sles12_sp2_comp_info,
	%intel_hfi_sles12_sp2_comp_info,
	%ib_wfr_lite_comp_info,
	%delta_ipoib_comp_info,
	%mpi_selector_comp_info,
	%mvapich2_comp_info,
	%openmpi_comp_info,
	%gasnet_comp_info,
	%openshmem_comp_info,
	%opa_stack_dev_sles12_sp2_comp_info,
	%delta_mpisrc_comp_info,
	%mpiRest_comp_info,
	%hfi1_uefi_comp_info,
	%delta_debug_comp_info,
	%ibacm_sles12_sp2_comp_info,
);

my %delta_comp_info_rhel73 = (
	%opa_stack_rhel73_comp_info,
	%intel_hfi_rhel73_comp_info,
	%ib_wfr_lite_comp_info,
	%delta_ipoib_comp_info,
	%mpi_selector_comp_info,
	%mvapich2_comp_info,
	%openmpi_comp_info,
	%gasnet_comp_info,
	%openshmem_comp_info,
	%opa_stack_dev_rhel73_comp_info,
	%delta_mpisrc_comp_info,
	%mpiRest_comp_info,
	%hfi1_uefi_comp_info,
	%delta_debug_comp_info,
	%ibacm_rhel73_comp_info,
);

my %delta_comp_info = ( );

# options for building compat-rdma srpm and what platforms they
# are each available for
my %delta_kernel_ib_options = (
	# build option		# arch & kernels supported on
						# (see util_build.pl for more info on format)
	# NOTE: the compat-rdma build takes no arguments right now.
	# This list is now empty, but will probably get used as time goes on.
);

# all kernel srpms
# these are in the order we must build/process them to meet basic dependencies
my @delta_kernel_srpms_other = ( 'compat-rdma' );
my @delta_kernel_srpms_rhel72 = ( 'kmod-ifs-kernel-updates' );
my @delta_kernel_srpms_rhel70 = ( 'compat-rdma' );
my @delta_kernel_srpms_rhel67 = ( 'ifs-kernel-updates' );
my @delta_kernel_srpms_sles = ( 'compat-rdma' );
my @delta_kernel_srpms_sles12_sp2 = ( 'ifs-kernel-updates-kmp-default' );
my @delta_kernel_srpms_rhel73 = ( 'kmod-ifs-kernel-updates' );
my @delta_kernel_srpms = ( );

# all user space srpms
# these are in the order we must build/process them to meet basic dependencies
my @delta_user_srpms_other = (
		"opa-scripts", "libibumad", "ibacm", "mpi-selector",
		"libhfi1", "libpsm2", "hfi1-diagtools-sw", "hfidiags", "hfi1-firmware", "hfi1-firmware_debug",
 		"mvapich2", "openmpi", "gasnet", "openshmem", "openshmem-test-suite",
		"shmem-benchmarks", "srptools", "libibmad", "infiniband-diags", "hfi1_uefi"
);
my @delta_user_srpms_rhel67 = (
		"opa-scripts", "libibumad", "ibacm", "mpi-selector",
		"libhfi1", "libpsm2", "hfi1-diagtools-sw", "hfidiags", "hfi1-firmware", "hfi1-firmware_debug",
 		"mvapich2", "openmpi", "gasnet", "openshmem", "openshmem-test-suite",
		"shmem-benchmarks", "srptools", "libibmad", "infiniband-diags", "hfi1_uefi"
);
my @delta_user_srpms_rhel72 = (
		"opa-scripts", "mpi-selector", "ibacm",
		"libhfi1", "libpsm2", "hfi1-diagtools-sw", "hfidiags", "hfi1-firmware", "hfi1-firmware_debug",
 		"mvapich2", "openmpi", "gasnet", "openshmem", "openshmem-test-suite",
		"shmem-benchmarks", "srptools", "libibmad", "infiniband-diags", "hfi1_uefi"
);
my @delta_user_srpms_rhel70 = (
		"opa-scripts", "libibumad", "ibacm", "mpi-selector",
		"libhfi1", "libpsm2", "hfi1-diagtools-sw", "hfidiags", "hfi1-firmware", "hfi1-firmware_debug",
 		"mvapich2", "openmpi", "gasnet", "openshmem", "openshmem-test-suite",
		"shmem-benchmarks", "srptools", "libibmad", "infiniband-diags"
);
my @delta_user_srpms_sles = (
		"opa-scripts", "libibumad3", "ibacm", "mpi-selector",
		"libhfi1", "libpsm2", "hfi1-diagtools-sw", "hfidiags", "hfi1-firmware", "hfi1-firmware_debug",
 		"mvapich2", "openmpi", "gasnet", "openshmem", "openshmem-test-suite",
		"shmem-benchmarks", "srptools", "libibmad5", "infiniband-diags", "hfi1_uefi"
);
my @delta_user_srpms_sles12_sp2 = (
		"opa-scripts", "mpi-selector",
		"libpsm2", "hfi1-diagtools-sw", "hfidiags", "hfi1-firmware", "hfi1-firmware_debug",
 		"mvapich2", "openmpi", "gasnet", "openshmem", "openshmem-test-suite",
		"shmem-benchmarks", "infiniband-diags", "hfi1_uefi"
);
my @delta_user_srpms_rhel73 = (
		"opa-scripts", "mpi-selector",
		"libpsm2", "hfi1-diagtools-sw", "hfidiags", "hfi1-firmware", "hfi1-firmware_debug",
 		"mvapich2", "openmpi", "gasnet", "openshmem", "openshmem-test-suite",
		"shmem-benchmarks", "infiniband-diags", "hfi1_uefi"
);
my @delta_user_srpms = ( );

# rpms not presently automatically built
my @delta_other_srpms = ( );

my @delta_srpms = ( );

# This provides information for all kernel and user space srpms
# Fields:
#	Available => indicate which platforms each srpm can be built for
#	PostReq => after building each srpm, some of its generated rpms will
#				need to be installed so that later srpms will build properly
#				this lists all the rpms which may be needed by subsequent srpms
#	Builds => list of kernel and user rpms built from each srpm
#				caller must know if user/kernel rpm is expected
#	PartOf => delta_components this srpm is part of
#			  this is filled in at runtime based on delta_comp_info
#			  cross referenced to Builds
#	BuildPrereq => OS prereqs needed to build this srpm
#				List in the form of "rpm version mode" for each.
#				version is optional, default is "any"
#				mode is optional, default is "any"
#				Typically mode should be "user" for libraries
#				See rpm_is_installed for more information about mode
my %compat_rdma_srpm_info = (
	"compat-rdma" =>        { Available => "",
					Builds => "compat-rdma compat-rdma-devel",
					PostReq => "compat-rdma compat-rdma-devel",
					PartOf => "", # filled in at runtime
					BuildPrereq => [],
					},
);

my %hfi1_psm_srpm_info = (
	"libpsm2" =>	{ Available => "",
					  Builds => "libpsm2 libpsm2-devel libpsm2-compat",
					  PostReq => "libpsm2 libpsm2-devel libpsm2-compat",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
);

my %libhfi1_srpm_info = (
	"libhfi1" =>	{ Available => "",
					  Builds => "libhfi1 libhfi1-static",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
);

my %hfi1_diagtools_sw_srpm_info = (
	"hfi1-diagtools-sw" =>	{ Available => "",
					  Builds => "hfi1-diagtools-sw hfi1-diagtools-sw-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'readline-devel', 'ncurses-devel', ],
					},
);

my %hfidiags_srpm_info = (
	"hfidiags" =>	{ Available => "",
					  Builds => "hfidiags",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
);

my %hfi1_firmware_srpm_info = (
	"hfi1-firmware" =>	{ Available => "",
					  Builds => "hfi1-firmware",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
);

my %hfi1_firmware_debug_srpm_info = (
	"hfi1-firmware_debug" =>	{ Available => "",
					  Builds => "hfi1-firmware_debug",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
);

my %ib_wfr_lite_srpm_info = (
	"ib_wfr_lite" =>	{ Available => "",
					  Builds => "ib_wfr_lite",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
);

my %opa_scripts_srpm_info = (
	"opa-scripts" => { Available => "",
					  Builds => "opa-scripts",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
);

my %libibumad_srpm_info = (
	"libibumad" =>	{ Available => "",
					  Builds => "libibumad libibumad-devel libibumad-static libibumad-debuginfo",
					  PostReq => "libibumad libibumad-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'libtool any user' ],
					},
);

my %libibumad3_srpm_info = (
	"libibumad3" =>	{ Available => "",
					  Builds => "libibumad3 libibumad-devel libibumad-static",
					  PostReq => "libibumad3 libibumad-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'libtool any user' ],
					},
);

my %ibacm_srpm_info = (
	"ibacm" =>		{ Available => "",
					  Builds => "ibacm ibacm-devel",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
);

my %mpi_selector_srpm_info = (
	"mpi-selector" => { Available => "",
					  Builds => "mpi-selector",
					  PostReq => "mpi-selector",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'tcsh' ],
					},
);

my %mvapich2_srpm_info = (
	"mvapich2" =>	{ Available => "",
						# mpitests are built by do_mvapich2_build
		 			  Builds => " mvapich2_gcc mpitests_mvapich2_gcc",
		 			  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'libstdc++ any user',
					  				   'libstdc++-devel any user',
									   'sysfsutils any user',
					  				   'g77', 'libgfortran any user'
								   	],
					},
);

my %openmpi_srpm_info = (
	"openmpi" =>	{ Available => "",
						# mpitests are built by do_openmpi_build
		 			  Builds => "openmpi_gcc mpitests_openmpi_gcc",
		 			  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'libstdc++ any user',
					  				   'libstdc++-devel any user',
					  				   'g77', 'libgfortran any user',
									   'binutils'
								   	],
					},
);

my %gasnet_srpm_info = (
	"gasnet" =>		{ Available => "",
		 			  Builds => "gasnet_gcc_hfi gasnet_gcc_hfi-devel gasnet_gcc_hfi-tests",
		 			  PostReq => "gasnet_gcc_hfi gasnet_gcc_hfi-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ ],
					},
);

my %openshmem_srpm_info = (
	"openshmem" =>	{ Available => "",
		 			  Builds => "openshmem_gcc_hfi",
		 			  PostReq => "openshmem_gcc_hfi",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ ],
					},
);

my %openshmem_test_suite_srpm_info = (
	"openshmem-test-suite" =>	{ Available => "",
					  Builds => "openshmem-test-suite_gcc_hfi",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
);

my %shmem_benchmarks_srpm_info = (
	"shmem-benchmarks" =>	{ Available => "",
					  Builds => "shmem-benchmarks_gcc_hfi",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
);

my %srptools_srpm_info = (
	"srptools" =>	{ Available => "",
					  Builds => "srptools srptools-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'libtool any user' ],
					},
);

my %libibmad_srpm_info = (
	"libibmad" =>  { Available => "",
					  Builds => "libibmad libibmad-devel libibmad-static",
					  PostReq => "libibmad libibmad-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'libtool any user' ],
					},
);

my %libibmad5_srpm_info = (
	"libibmad5" =>  { Available => "",
					  Builds => "libibmad5 libibmad-devel libibmad-static",
					  PostReq => "libibmad5 libibmad-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'libtool any user' ],
					},
);

my %infiniband_diags_srpm_info = (
	"infiniband-diags" =>  { Available => "",
					  Builds => "infiniband-diags infiniband-diags-compat",
					  PostReq => "infiniband-diags",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'opensm-devel any user',
							   'glib2-devel any user',
						  	 ],
					},
);

my %hfi1_uefi_srpm_info = (
	"hfi1_uefi" => { Available => "",
					Builds => "hfi1-uefi",
					PostReq => "",
					PartOf => "",
					BuildPrereq => [],
					},
);

my %kmp_ifs_kernel_updates_srpm_info = (
	"ifs-kernel-updates-kmp-default" =>        { Available => "",
					Builds => "ifs-kernel-updates-kmp-default ifs-kernel-updates-devel",
					PostReq => "ifs-kernel-updates-devel",
					PartOf => "", # filled in at runtime
					BuildPrereq => [],
					},
);

my %kmod_ifs_kernel_updates_srpm_info = (
	"kmod-ifs-kernel-updates" =>    { Available => "",
					Builds => "kmod-ifs-kernel-updates ifs-kernel-updates-devel",
					PostReq => "ifs-kernel-updates-devel",
					PartOf => "", # filled in at runtime
					BuildPrereq => [],
					},
);

my %ifs_kernel_updates_rhel67_srpm_info = (
        "ifs-kernel-updates" =>        { Available => "",
                                        Builds => "ifs-kernel-updates ifs-kernel-updates-devel",
                                        PostReq => "ifs-kernel-updates-devel",
                                        PartOf => "", # filled in at runtime
                                        BuildPrereq => [],
                                        },
);

my %delta_srpm_info_other = (
	%compat_rdma_srpm_info,
	%hfi1_psm_srpm_info,
	%libhfi1_srpm_info,
	%hfi1_diagtools_sw_srpm_info,
	%hfidiags_srpm_info,
	%hfi1_firmware_srpm_info,
	%hfi1_firmware_debug_srpm_info,
	%ib_wfr_lite_srpm_info,
	%opa_scripts_srpm_info,
	%libibumad_srpm_info,
	%ibacm_srpm_info,
	%mpi_selector_srpm_info,
	%mvapich2_srpm_info,
	%openmpi_srpm_info,
	%gasnet_srpm_info,
	%openshmem_srpm_info,
	%openshmem_test_suite_srpm_info,
	%shmem_benchmarks_srpm_info,
	%srptools_srpm_info,
	%libibmad_srpm_info,
	%infiniband_diags_srpm_info,
	%hfi1_uefi_srpm_info,
);

my %delta_srpm_info_rhel67 = (
	%ifs_kernel_updates_rhel67_srpm_info,
	%hfi1_psm_srpm_info,
	%libhfi1_srpm_info,
	%hfi1_diagtools_sw_srpm_info,
	%hfidiags_srpm_info,
	%hfi1_firmware_srpm_info,
	%hfi1_firmware_debug_srpm_info,
	%ib_wfr_lite_srpm_info,
	%opa_scripts_srpm_info,
	%libibumad_srpm_info,
	%ibacm_srpm_info,
	%mpi_selector_srpm_info,
	%mvapich2_srpm_info,
	%openmpi_srpm_info,
	%gasnet_srpm_info,
	%openshmem_srpm_info,
	%openshmem_test_suite_srpm_info,
	%shmem_benchmarks_srpm_info,
	%srptools_srpm_info,
	%libibmad_srpm_info,
	%infiniband_diags_srpm_info,
	%hfi1_uefi_srpm_info,
);

my %delta_srpm_info_rhel72 = (
	%kmod_ifs_kernel_updates_srpm_info,
	%hfi1_psm_srpm_info,
	%libhfi1_srpm_info,
	%hfi1_diagtools_sw_srpm_info,
	%hfidiags_srpm_info,
	%hfi1_firmware_srpm_info,
	%hfi1_firmware_debug_srpm_info,
	%ib_wfr_lite_srpm_info,
	%opa_scripts_srpm_info,
	%libibumad_srpm_info,
	%ibacm_srpm_info,
	%mpi_selector_srpm_info,
	%mvapich2_srpm_info,
	%openmpi_srpm_info,
	%gasnet_srpm_info,
	%openshmem_srpm_info,
	%openshmem_test_suite_srpm_info,
	%shmem_benchmarks_srpm_info,
	%srptools_srpm_info,
	%libibmad_srpm_info,
	%infiniband_diags_srpm_info,
	%hfi1_uefi_srpm_info,
);

my %delta_srpm_info_rhel70 = (
	%compat_rdma_srpm_info,
	%hfi1_psm_srpm_info,
	%libhfi1_srpm_info,
	%hfi1_diagtools_sw_srpm_info,
	%hfidiags_srpm_info,
	%hfi1_firmware_srpm_info,
	%hfi1_firmware_debug_srpm_info,
	%ib_wfr_lite_srpm_info,
	%opa_scripts_srpm_info,
	%libibumad_srpm_info,
	%ibacm_srpm_info,
	%mpi_selector_srpm_info,
	%mvapich2_srpm_info,
	%openmpi_srpm_info,
	%gasnet_srpm_info,
	%openshmem_srpm_info,
	%openshmem_test_suite_srpm_info,
	%shmem_benchmarks_srpm_info,
	%srptools_srpm_info,
#	%libibmad_srpm_info,
#	%infiniband_diags_srpm_info,
);

my %delta_srpm_info_sles = (
	%compat_rdma_srpm_info,
	%hfi1_psm_srpm_info,
	%libhfi1_srpm_info,
	%hfi1_diagtools_sw_srpm_info,
	%hfidiags_srpm_info,
	%hfi1_firmware_srpm_info,
	%hfi1_firmware_debug_srpm_info,
	%ib_wfr_lite_srpm_info,
	%opa_scripts_srpm_info,
	%libibumad3_srpm_info,
	%ibacm_srpm_info,
	%mpi_selector_srpm_info,
	%mvapich2_srpm_info,
	%openmpi_srpm_info,
	%gasnet_srpm_info,
	%openshmem_srpm_info,
	%openshmem_test_suite_srpm_info,
	%shmem_benchmarks_srpm_info,
	%srptools_srpm_info,
	%libibmad5_srpm_info,
	%infiniband_diags_srpm_info,
	%hfi1_uefi_srpm_info,
);

my %delta_srpm_info_sles12_sp2 = (
	%kmp_ifs_kernel_updates_srpm_info,
	%hfi1_psm_srpm_info,
	%hfi1_diagtools_sw_srpm_info,
	%hfidiags_srpm_info,
	%hfi1_firmware_srpm_info,
	%hfi1_firmware_debug_srpm_info,
	%opa_scripts_srpm_info,
	%mpi_selector_srpm_info,
	%mvapich2_srpm_info,
	%openmpi_srpm_info,
	%gasnet_srpm_info,
	%openshmem_srpm_info,
	%openshmem_test_suite_srpm_info,
	%shmem_benchmarks_srpm_info,
	%infiniband_diags_srpm_info,
	%hfi1_uefi_srpm_info,
);

my %delta_srpm_info_rhel73 = (
	%kmod_ifs_kernel_updates_srpm_info,
	%hfi1_psm_srpm_info,
	%hfi1_diagtools_sw_srpm_info,
	%hfidiags_srpm_info,
	%hfi1_firmware_srpm_info,
	%hfi1_firmware_debug_srpm_info,
	%opa_scripts_srpm_info,
	%mpi_selector_srpm_info,
	%mvapich2_srpm_info,
	%openmpi_srpm_info,
	%gasnet_srpm_info,
	%openshmem_srpm_info,
	%openshmem_test_suite_srpm_info,
	%shmem_benchmarks_srpm_info,
	%infiniband_diags_srpm_info,
	%hfi1_uefi_srpm_info,
);

my %delta_srpm_info = ( );

# This provides information for all kernel and user space rpms
# This is built based on information in delta_srpm_info and delta_comp_info
# Fields:
#	Available => boolean indicating if rpm can be available for this platform
#	Parent => srpm which builds this rpm
#	PartOf => space separate list of components this rpm is part of
#	Mode => mode of rpm (kernel or user)
my %delta_rpm_info = ();
my @delta_rpms = ();

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
	my $avail;

	$avail="Available";

	DebugPrint("checking $srpm $mode $osver against '$delta_srpm_info{$srpm}{$avail}'\n");
	return arch_kernel_is_allowed($osver, $delta_srpm_info{$srpm}{$avail});
}

# initialize delta_rpm_info based on specified osver for present system
sub init_delta_rpm_info($)
{
	my $osver = shift();

	%delta_rpm_info = ();	# start fresh

	# filter components by distro
	if ("$CUR_DISTRO_VENDOR" eq 'SuSE'
		&& ("$CUR_VENDOR_VER" eq 'ES12' || "$CUR_VENDOR_VER" eq 'ES121')) {
		@delta_components = ( @delta_components_sles );
		%delta_comp_info = ( %delta_comp_info_sles );
		@delta_kernel_srpms = ( @delta_kernel_srpms_sles );
		@delta_user_srpms = ( @delta_user_srpms_sles );
		%delta_srpm_info = ( %delta_srpm_info_sles );
	} elsif ("$CUR_DISTRO_VENDOR" eq 'SuSE'
		&& "$CUR_VENDOR_VER" eq 'ES122') {
		@delta_components = ( @delta_components_sles12_sp2 );
		%delta_comp_info = ( %delta_comp_info_sles12_sp2 );
		@delta_kernel_srpms = ( @delta_kernel_srpms_sles12_sp2 );
		@delta_user_srpms = ( @delta_user_srpms_sles12_sp2 );
		%delta_srpm_info = ( %delta_srpm_info_sles12_sp2 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES73" ) {
		@delta_components = ( @delta_components_rhel73 );
		%delta_comp_info = ( %delta_comp_info_rhel73 );
		@delta_kernel_srpms = ( @delta_kernel_srpms_rhel73 );
		@delta_user_srpms = ( @delta_user_srpms_rhel73 );
		%delta_srpm_info = ( %delta_srpm_info_rhel73 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES72" ) {
		@delta_components = ( @delta_components_rhel72 );
		%delta_comp_info = ( %delta_comp_info_rhel72 );
		@delta_kernel_srpms = ( @delta_kernel_srpms_rhel72 );
		@delta_user_srpms = ( @delta_user_srpms_rhel72 );
		%delta_srpm_info = ( %delta_srpm_info_rhel72 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES7" ) {
		@delta_components = ( @delta_components_rhel70 );
		%delta_comp_info = ( %delta_comp_info_rhel70 );
		@delta_kernel_srpms = ( @delta_kernel_srpms_rhel70 );
		@delta_user_srpms = ( @delta_user_srpms_rhel70 );
		%delta_srpm_info = ( %delta_srpm_info_rhel70 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES67" ) {
		@delta_components = ( @delta_components_rhel67 );
		%delta_comp_info = ( %delta_comp_info_rhel67 );
		@delta_kernel_srpms = ( @delta_kernel_srpms_rhel67 );
		@delta_user_srpms = ( @delta_user_srpms_rhel67 );
		%delta_srpm_info = ( %delta_srpm_info_rhel67 );
	} else {
		@delta_components = ( @delta_components_other );
		%delta_comp_info = ( %delta_comp_info_other );
		@delta_kernel_srpms = ( @delta_kernel_srpms_other );
		@delta_user_srpms = ( @delta_user_srpms_other );
		%delta_srpm_info = ( %delta_srpm_info_other );
	}

	@delta_srpms = ( @delta_kernel_srpms, @delta_user_srpms, @delta_other_srpms );

	# first get information based on all srpms
	foreach my $srpm ( @delta_srpms ) {
		my @package_list = split /[[:space:]]+/, $delta_srpm_info{$srpm}{'Builds'};
		foreach my $package ( @package_list ) {
			next if ( "$package" eq '' );	# handle leading spaces
			$delta_rpm_info{$package}{'Available'} = available_srpm($srpm, "user", $osver);
			$delta_rpm_info{$package}{'Parent'} = "$srpm";
		}
	}

	# now fill in PartOf and Mode based on all components
	# allow for the case where a package could be part of more than 1 component
	# however assume User vs Kernel is consistent
	foreach my $comp ( @delta_components ) {
		foreach my $package ( @{ $delta_comp_info{$comp}{'UserRpms'}} ) {
			$delta_rpm_info{$package}{'PartOf'} .= " $comp";
			$delta_rpm_info{$package}{'Mode'} = "user";
		}
		foreach my $package ( @{ $delta_comp_info{$comp}{'KernelRpms'}} ) {
			$delta_rpm_info{$package}{'PartOf'} .= " $comp";
			$delta_rpm_info{$package}{'Mode'} = "kernel";
		}
		foreach my $package ( @{ $delta_comp_info{$comp}{'DebugRpms'}} ) {
			$delta_rpm_info{$package}{'PartOf'} .= " delta_debug";
			# this is the only debuginfo with kernel rev in its name
			if ( "$package" eq "ib-bonding-debuginfo" ) {
				$delta_rpm_info{$package}{'Mode'} = "kernel";
			} else {
				$delta_rpm_info{$package}{'Mode'} = "user";
			}
		}
	}
	@delta_rpms = ( keys %delta_rpm_info );	# list of all rpms
	# fixup Available for those in "*Rest" but not in delta_srpm_info{*}{Builds}
	foreach my $package ( @delta_rpms ) {
		if ("$delta_rpm_info{$package}{'Available'}" eq "" ) {
			$delta_rpm_info{$package}{'Available'}=0;
		}
	}

	# -debuginfo is not presently supported on SuSE or Centos
	# also .rpmmacros overrides can disable debuginfo availability
	if ( "$CUR_DISTRO_VENDOR" eq "SuSE"
		|| ! rpm_will_build_debuginfo()) {
		foreach my $package ( @delta_rpms ) {
			if ($package =~ /-debuginfo/) {
				$delta_rpm_info{$package}{'Available'} = 0;
			}
		}
	}

	# disable libibmad and infiniband-diags for RHEL7.0
	if ( "$CUR_VENDOR_VER" eq "ES7" ) {
		foreach my $package ( @delta_rpms ) {
			if ($package =~ /libibmad/ ||
			    $package =~ /infiniband-diag/ ) {
				$delta_rpm_info{$package}{'Available'} = 0;
			}
		}
	}
	# every package must be part of some component (could be a dummy component)
	foreach my $package ( @delta_rpms ) {
		if ( "$delta_rpm_info{$package}{'PartOf'}" eq "" ) {
			LogPrint "$package: Not PartOf anything\n";
		}
	}

	# build $delta_srpm_info{}{PartOf}
	foreach my $srpm ( @delta_srpms ) {
		$delta_srpm_info{$srpm}{'PartOf'} = '';
	}
	foreach my $package ( @delta_rpms ) {
		my @complist = split /[[:space:]]+/, $delta_rpm_info{$package}{'PartOf'};
		my $srpm = $delta_rpm_info{$package}{'Parent'};
		foreach my $comp ( @complist ) {
			next if ( "$comp" eq '' );	# handle leading spaces
			if ( " $delta_srpm_info{$srpm}{'PartOf'} " !~ / $comp / ) {
				$delta_srpm_info{$srpm}{'PartOf'} .= " $comp";
			}
		}
	}
	# fixup special case for compat-rdma, its part of all delta comp w/ Drivers
	#my $srpm = $delta_rpm_info{'compat-rdma'}{'Parent'};
	#foreach my $comp ( @delta_components ) {
	#	if ("$delta_comp_info{$comp}{'Drivers'}" ne "" ) {
	#		if ( " $delta_srpm_info{$srpm}{'PartOf'} " !~ / $comp / ) {
	#			$delta_srpm_info{$srpm}{'PartOf'} .= " $comp";
	#		}
	#	}
	#}

	if (DebugPrintEnabled() ) {
		# dump all SRPM info
		DebugPrint "\nSRPMs:\n";
		foreach my $srpm ( @delta_srpms ) {
			DebugPrint("$srpm => Builds: '$delta_srpm_info{$srpm}{'Builds'}'\n");
			DebugPrint("           PostReq: '$delta_srpm_info{$srpm}{'PostReq'}'\n");
			DebugPrint("           Available: '$delta_srpm_info{$srpm}{'Available'}'\n");
			DebugPrint("           Available: ".available_srpm($srpm, "user", $osver)." PartOf '$delta_srpm_info{$srpm}{'PartOf'}'\n");
		}

		# dump all RPM info
		DebugPrint "\nRPMs:\n";
		foreach my $package ( @delta_rpms ) {
			DebugPrint("$package => Parent: '$delta_rpm_info{$package}{'Parent'}' Available: $delta_rpm_info{$package}{'Available'} \n");
			DebugPrint("           Mode: $delta_rpm_info{$package}{'Mode'} PartOf: '$delta_rpm_info{$package}{'PartOf'}'\n");
		}

		# dump all DELTA component info
		DebugPrint "\nDELTA Components:\n";
		foreach my $comp ( @delta_components ) {
			DebugPrint("   $comp: KernelRpms: @{ $delta_comp_info{$comp}{'KernelRpms'}}\n");
			DebugPrint("           UserRpms: @{ $delta_comp_info{$comp}{'UserRpms'}}\n");
			DebugPrint("           DebugRpms: @{ $delta_comp_info{$comp}{'DebugRpms'}}\n");
			DebugPrint("           Drivers: $delta_comp_info{$comp}{'Drivers'}\n");
			DebugPrint("           StartupParams: @{ $delta_comp_info{$comp}{'StartupParams'}}\n");
		}
		DebugPrint "\n";
	}
}

# delta specific rpm functions,
# install available user or kernel rpms in list
# returns 1 if hfi1 was installed
sub delta_rpm_install_list($$$@)
{
	my $rpmdir = shift();
	my $rpmdir_t;
	my $osver = shift();	# OS version
	my $skip_kernelib = shift();	# should compat-rdma be skipped if in package_list
	my(@package_list) = @_;	# package names
	my $ret = 0;

	# user space RPM installs
	foreach my $package ( @package_list )
	{
		$rpmdir_t=$rpmdir;
		if ($delta_rpm_info{$package}{'Available'} ) {
			if ( "$delta_rpm_info{$package}{'Mode'}" eq "kernel" ) {
				if ( "$CUR_VENDOR_VER" eq "ES72" || "$CUR_VENDOR_VER" eq "ES73" || "$CUR_VENDOR_VER" eq "ES122" ) {
					if ( $package =~ /ifs-kernel-updates/ ) {
						if ( $GPU_Install == 1 ) {
                                                        $rpmdir_t=$rpmdir."/CUDA";
                                                }
						next if ( $skip_kernelib);
						$ret = 1;
					}
				} elsif ( "$CUR_VENDOR_VER" eq "ES67" ) {
					if ( " $package " =~ / ifs-kernel-updates / ) {
                                                next if ( $skip_kernelib);
                                                $ret = 1;
                                        }
				} else {
					if ( " $package " =~ / compat-rdma / ) {
						next if ( $skip_kernelib);
						$ret = 1;
					}
				}
				rpm_install_with_options($rpmdir_t, $osver, $package, " -U --nodeps ");
			} else {
				if ( $GPU_Install == 1 ) {
                                        if ( -d $rpmdir."/CUDA" ) {
                                                if ( $package =~ /libpsm/ || $package =~ /ifs-kernel-updates/) {
                                                        $rpmdir_t=$rpmdir."/CUDA";
                                                }
                                        } else {
                                                NormalPrint("CUDA specific packages do not exist\n");
                                                exit 0;
                                        }
                                }
				rpm_install_with_options($rpmdir_t, "user", $package, " -U --nodeps ");
			}
		}
	}
	return $ret;
}

# verify all rpms in list are installed
sub delta_rpm_is_installed_list($@)
{
	my $osver = shift();
	my(@package_list) = @_;	# package names

	foreach my $package ( @package_list )
	{
		if ($delta_rpm_info{$package}{'Available'} ) {
			if ( "$delta_rpm_info{$package}{'Mode'}" eq "kernel" ) {
				if (! rpm_is_installed($package, $osver) ) {
					return 0;
				}
			} else {
				if (! rpm_is_installed($package, "user") ) {
					return 0;
				}
			}
		}
	}
	return 1;
}
# verify the rpmfiles exist for all the kernel RPMs listed
sub delta_rpm_exists_list($$@)
{
	my $rpmdir = shift();
	my $mode = shift();	#  "user" or kernel rev
	my(@package_list) = @_;	# package names
	my $avail;

	$avail="Available";

	foreach my $package ( @package_list )
	{
		next if ( "$package" eq '' );
		if ($delta_rpm_info{$package}{$avail} ) {
			if (! rpm_exists($rpmdir, $mode, $package) ) {
				return 0;
			}
		}
	}
	return 1;
}

# returns install prefix for presently installed DELTA
sub delta_get_prefix()
{
	my $prefix = "/usr";	# default
	return "$prefix";
}

# unfortunately OFED mpitests leaves empty directories on uninstall
# this can confuse IFS MPI tools because correct MPI to use
# cannot be identified.  This remove such empty directories for all
# compilers in all possible prefixes for OFED
sub delta_cleanup_mpitests()
{
	my $prefix = ofed_get_prefix();

	if ( -e "$ROOT$prefix/mpi") {
		system("cd '$ROOT$prefix/mpi'; rmdir -p */*/tests/* >/dev/null 2>&1");
	}
	if ( -e "$ROOT/usr/mpi") {
		system("cd $ROOT/usr/mpi; rmdir -p */*/tests/* >/dev/null 2>&1");
	}
	if ( -e "$ROOT/$OFED_prefix/mpi") {
		system("cd '$ROOT/$OFED_prefix/mpi'; rmdir -p */*/tests/* >/dev/null 2>&1");
	}
}

# uninstall rpms which are in package_list and are not needed by
# any components in install_list
# all variations of the specified packages are uninstalled
sub delta_rpm_uninstall_not_needed_list($$$$@)
{
	my $install_list = shift();	# components which will remain on system
	my $uninstalling_list = shift();	# components which are being uninstalled
	my $comp = shift();	# component being uninstalled
	my $verbosity = shift();
	my(@package_list) = @_;	# package names to consider for uninstall

RPM: foreach my $package ( reverse(@package_list) ) {
		my @install_list = split /[[:space:]]+/, $install_list;
		foreach my $c ( @install_list ) {
			next if ( "$c" eq '' ); # handling leading spaces
			# see if package is part of a component we are interested in
			if ( " @delta_components " =~ / $c /
				 && " $delta_rpm_info{$package}{'PartOf'} " =~ / $c / ) {
				next RPM;	# its still needed, leave it installed
			}
		}
		if ( $delta_rpm_info{$package}{'Available'} == 0 ) {
			next RPM; # package was not installed.
		}
		# if we get here, package is not in any component we are interested in
		if ( "$uninstalling_list" ne "" && "$comp" ne "" ) {
			# we are doing an explicit uninstall, we must be careful
			# about rpms which are part of more than 1 component
			# uninstalling_list is in dependency order and is executed
			# backwards, so once we get to processing the 1st component
			# in uninstalling list which has this package, we know its
			# safe to remove the package
			my @uninstalling = split /[[:space:]]+/, $uninstalling_list;
			foreach my $c ( @uninstalling ) {
				next if ( "$c" eq '' ); # handling leading spaces
				if ( " @delta_components " =~ / $c /
					&& " $delta_rpm_info{$package}{'PartOf'} " =~ / $c / ) {
					# found 1st uninstalled component with package
					if ("$c" ne "$comp") {
						next RPM;	# don't remove til get to $c's uninstall
					} else {
						last;	# exit this loop and uninstall package
					}
				}
			}
		}
		rpm_uninstall($package, "any", " --nodeps ", $verbosity);
	}
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
                        if ("$globname" eq "libpsm2*.src.rpm") {
                                $result = file_glob("$srcdir/$SRPMS_SUBDIR/CUDA/$globname");
                        } elsif ("$globname" eq "ifs-kernel-updates*.src.rpm"){
                                $result = file_glob("$srcdir/$SRPMS_SUBDIR/CUDA/$globname");
                        }
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
	# we purposely use a different directory than OFED, this way
	# runs of OFED install or build scripts will not confuse the IFS
	# wrapped install
	##return "$srcdir/$RPMS_SUBDIR/$RPM_DIST/";
	if (-d "$srcdir/$RPMS_SUBDIR/$CUR_DISTRO_VENDOR-$CUR_VENDOR_VER"
		|| ! -d "$srcdir/$RPMS_SUBDIR/$CUR_DISTRO_VENDOR-$CUR_VENDOR_MAJOR_VER") {
		return "$srcdir/$RPMS_SUBDIR/$CUR_DISTRO_VENDOR-$CUR_VENDOR_VER";
	} else {
		return "$srcdir/$RPMS_SUBDIR/$CUR_DISTRO_VENDOR-$CUR_VENDOR_MAJOR_VER";
	}
}

#
# get prefix used when last built rpms
sub get_delta_rpm_prefix($)
{
	my $rpmsdir = shift();

	my $prefix_file = "$rpmsdir/Prefix";
	if ( ! -e "$prefix_file") {
		return "invalid and unknown";	# unknown, play it safe and rebuild
	} else {
		my $prefix = `cat $prefix_file 2>/dev/null`;
		chomp $prefix;
		return $prefix;
	}
}
sub save_delta_rpm_prefix($$)
{
	my $rpmsdir = shift();
	my $prefix = shift();

	my $prefix_file = "$rpmsdir/Prefix";
	system("mkdir -p $rpmsdir");
	system "echo $prefix > $prefix_file";
}

# verify if all rpms have been built from the given srpm
sub is_built_srpm($$)
{
	my $srpm = shift();	# srpm name prefix
	my $mode = shift();	# "user" or kernel rev
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};
	my $rpmsdir = delta_rpms_dir();

	my @rpmlist = split /[[:space:]]+/, $delta_srpm_info{$srpm}{'Builds'};
	return ( delta_rpm_exists_list("$rpmsdir", $mode, @rpmlist) );
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

sub need_install_rpm_list($$$@)
{
	my $osver = shift();
	my $force = shift();
	my $prompt = shift();	# prompt (only used if ! $force)
	my(@package_list) = @_;	# package names
	my $found = 0;
	my @available_list = ();

	foreach my $package ( @package_list ) {
		if ($delta_rpm_info{$package}{'Available'}) {
			@available_list = ( @available_list, $package );
		}
	}
	if (! scalar(@available_list)) {
		return 0;	# nothing to consider for install
	}
	return ($force
			|| ! delta_rpm_is_installed_list($osver, @available_list)
			|| ($prompt && GetYesNo("Reinstall @available_list for use during build?", "n")));
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

sub remove_unneeded_kernel_ib_drivers($);

# install rpms which were PostReqs of previous srpms
sub delta_install_needed_rpms($$$$$@)
{
	my $install_list = shift();	# total that will be installed when done
	my $osver = shift();	# kernel rev
	my $force_rpm = shift();
	my $prompt_rpm = shift();
	my $rpmsdir = shift();
	my @need_install = @_;

	if (need_install_rpm_list($osver, $force_rpm, $prompt_rpm, @need_install)) {
		if (delta_rpm_install_list("$rpmsdir", $osver, 0, @need_install)) {
			remove_unneeded_kernel_ib_drivers($install_list);
		}
	}
}

# Build RPM from source RPM
# build a specific SRPM
# this is heavily based on build_rpm in OFED install.pl
# main changes from cut and paste are marked with # IFS commands
# this has an srpm orientation and is only called when we really want to
# build the srpm
sub build_srpm($$$$$)
{
	my $srpm = shift();	# the srpm package name
	my $TOPDIR = shift();	# top directory for build
	my $BUILD_ROOT = shift();	# temp directory for build
	my $prefix = shift();	# prefix for install path
	my $resfileop = shift(); # append or replace build.res file
	my $configure_options = '';	# delta keeps per srpm, but only initializes here
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};
	my $SRC_RPM = delta_srpm_file($srcdir, "$srpm*.src.rpm");

	# Deal with SLES renaming
	if ("$srpm" eq "libibumad3") {
		$SRC_RPM = delta_srpm_file($srcdir, "libibumad-*.src.rpm");
	} elsif ("$srpm" eq "libibmad5") {
		$SRC_RPM = delta_srpm_file($srcdir, "libibmad-*.src.rpm");
	} elsif ("$srpm" eq "openshmem") {
		$SRC_RPM = delta_srpm_file($srcdir, "${srpm}_*.src.rpm");
	} elsif ("$srpm" eq "kmod-ifs-kernel-updates") {
		$SRC_RPM = delta_srpm_file($srcdir, "ifs-kernel-updates*.src.rpm");
	} elsif ("$srpm" eq "ifs-kernel-updates-kmp-default") {
		$SRC_RPM = delta_srpm_file($srcdir, "ifs-kernel-updates*.src.rpm");
	}

	# convert a few variables into the names used in OFED's build_rpm
	my $parent = $srpm;
	my $distro = "$CUR_DISTRO_VENDOR";
    my $dist_rpm_rel  = $RPM_DIST_REL;
    my $target_cpu = $RPM_ARCH;
    my $optflags = rpm_query_param('optflags');
    my $mandir = rpm_query_param('_mandir');
    my $sysconfdir = rpm_query_param('_sysconfdir');
    my $arch = my_tolower($ARCH);

    my $cmd;

    my $ldflags;
    my $cflags;
    my $cppflags;
    my $cxxflags;
    my $fflags;
    my $ldlibs;
    my $openmpi_comp_env;

    my $pref_env;
    if ($prefix ne $default_prefix) {
        $pref_env .= " LD_LIBRARY_PATH=$prefix/lib64:$prefix/lib:\$LD_LIBRARY_PATH";
        if ($parent ne "mvapich2" and $parent ne "openmpi") {
            $ldflags .= "$optflags -L$prefix/lib64 -L$prefix/lib";
            $cflags .= "$optflags -I$prefix/include";
            $cppflags .= "$optflags -I$prefix/include";
        }
    }

	# IFS - OFED tested rpm_exist.  We only get here if we
	# want to build the rpm, so we force true and proceed
	# (keeps indentation same as OFED install.pl for easier cut/paste)
    if (1) {
        if ($ldflags) {
            $pref_env   .= " LDFLAGS='$ldflags'";
        }
        if ($cflags) {
            $pref_env   .= " CFLAGS='$cflags'";
        }
        if ($cppflags) {
            $pref_env   .= " CPPFLAGS='$cppflags'";
        }
        if ($cxxflags) {
            $pref_env   .= " CXXFLAGS='$cxxflags'";
        }
        if ($fflags) {
            $pref_env   .= " FFLAGS='$fflags'";
        }
        if ($ldlibs) {
            $pref_env   .= " LDLIBS='$ldlibs'";
        }

        $cmd = "$pref_env rpmbuild --rebuild --define '_topdir $TOPDIR'";
        $cmd .= " --define 'dist  %{nil}'";
        $cmd .= " --target $target_cpu";
		# IFS - also set build_root so we can cleanup and avoid conflicts
    	$cmd .= " --buildroot '${BUILD_ROOT}'";
    	$cmd .= " --define 'build_root ${BUILD_ROOT}'";

        # Prefix should be defined per package
		# IFS - dropped MPIs, built via do_X_build scripts instead
        if ($parent eq "mpi-selector") {
            $cmd .= " --define '_prefix $prefix'";
            $cmd .= " --define '_exec_prefix $prefix'";
            $cmd .= " --define '_sysconfdir $sysconfdir'";
            $cmd .= " --define '_usr $prefix'";
            $cmd .= " --define 'shell_startup_dir /etc/profile.d'";
        }
# TBD - odd that prefix, exec_prefix, sysconfdir and usr not defined
# IFS - may want to add these 4 just to be safe, they are not in OFED
#            $cmd .= " --define '_prefix $prefix'";
#            $cmd .= " --define '_exec_prefix $prefix'";
#            $cmd .= " --define '_sysconfdir $sysconfdir'";
#            $cmd .= " --define '_usr $prefix'";
        else {
            $cmd .= " --define '_prefix $prefix'";
            $cmd .= " --define '_exec_prefix $prefix'";
            $cmd .= " --define '_sysconfdir $sysconfdir'";
            $cmd .= " --define '_usr $prefix'";
        }

		# IFS - keep configure_options as a local
        if ($configure_options or $OFED_user_configure_options) {
            $cmd .= " --define 'configure_options $configure_options $OFED_user_configure_options'";
        }

		# IFS - use SRC_RPM (computed above) instead of srpmpath_for_distro
#       $cmd .= " $main_packages{$parent}{'srpmpath'}";
		$cmd .= " $SRC_RPM";

	if ("$srpm" eq "gasnet") {
	    $cmd .= " --define '_name gasnet_openmpi_hfi'";
	    $cmd .= " --define '_prefix /usr/shmem/gcc/gasnet-1.28.2-openmpi-hfi'";
	    $cmd .= " --define '_name gasnet_gcc_hfi'";
	    $cmd .= " --define 'spawner mpi'";
	    $cmd .= " --define 'mpi_prefix /usr/mpi/gcc/openmpi-1.10.4-hfi'";
	}

	if ("$srpm" eq "openshmem") {
	    $cmd .= " --define '_name openshmem_gcc_hfi'";
	    $cmd .= " --define '_prefix /usr/shmem/gcc/openshmem-1.3-hfi'";
	    $cmd .= " --define 'gasnet_prefix /usr/shmem/gcc/gasnet-1.28.2-openmpi-hfi'";
	    $cmd .= " --define 'configargs --with-gasnet-threnv=seq'";
	}

	if ("$srpm" eq "openshmem-test-suite") {
	    $cmd .= " --define '_name openshmem-test-suite_gcc_hfi'";
	    $cmd .= " --define '_prefix /usr/shmem/gcc/openshmem-1.3-hfi'";
	    $cmd .= " --define 'openshmem_prefix /usr/shmem/gcc/openshmem-1.3-hfi'";
	}

	if ("$srpm" eq "shmem-benchmarks") {
	    $cmd .= " --define '_prefix /usr/shmem/gcc/openshmem-1.3-hfi'";
	    $cmd .= " --define 'openshmem_prefix /usr/shmem/gcc/openshmem-1.3-hfi'";
	}

		return run_build("$srcdir $SRC_RPM $RPM_ARCH", "$srcdir", $cmd, "$resfileop");
	}
	# NOTREACHED
}

# build all OFED components specified in installing_list
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
	my $prompt_rpm = 0;	# prompt per RPM
	my $force_srpm = $force;	# force SRPM rebuild
	my $force_kernel_srpm = ("$OFED_kernel_configure_options" ne "");
	my $force_user_srpm = ("$OFED_user_configure_options" ne "");
	my $force_rpm = $force;	# force dependent RPM reinstall
	my $rpmsdir = delta_rpms_dir();

	my $prefix=$OFED_prefix;
	if ("$prefix" ne get_delta_rpm_prefix($rpmsdir)) {
		$force_kernel_srpm = 1;
		$force_user_srpm = 1;
	}

	if (! $force && ! $Default_Prompt && ! ($force_user_srpm && $force_kernel_srpm)) {
		my $choice = GetChoice("Rebuild OFA SRPMs (a=all, p=prompt per SRPM, n=only as needed?)", "n", ("a", "p", "n"));
		if ("$choice" eq "a") {
			$force_srpm=1;
		} elsif ("$choice" eq "p") {
			$prompt_srpm=1;
		} elsif ("$choice" eq "n") {
			$prompt_srpm=0;
		}
	}
	# we base our decision on status of opa_stack.  Possible risk if
	# opa_stack is partially upgraded and was interrupted.
	if (! comp_is_uptodate('opa_stack') || $force_srpm  || $force_user_srpm || $force_kernel_srpm) {
		$force_rpm = 1;
	} elsif (! $Default_Prompt) {
		my $choice = GetChoice("Reinstall OFA dependent RPMs (a=all, p=prompt per RPM, n=only as needed?)", "n", ("a", "p", "n"));
		if ("$choice" eq "a") {
			$force_rpm=1;
		} elsif ("$choice" eq "p") {
			$prompt_rpm=1;
		} elsif ("$choice" eq "n") {
			$prompt_rpm=0;
		}
	}

	# -------------------------------------------------------------------------
	# do all rebuild prompting first so user doesn't have to wait 5 minutes
	# between prompts
	my %build_kernel_srpms = ();
	my $need_build = 0;
	my $build_compat_rdma = 0;

	if(!$skip_kernel) {
		foreach my $kernel_srpm ( @delta_kernel_srpms ) {
			$build_kernel_srpms{"${kernel_srpm}_build_kernel"} = need_build_srpm($kernel_srpm, "$K_VER", "$K_VER",
								$installing_list,
								$force_srpm || $force_kernel_srpm || $OFED_debug,
								$prompt_srpm);
			if ("$kernel_srpm" eq "compat-rdma" &&
				$build_kernel_srpms{"${kernel_srpm}_build_kernel"}) {
				$build_compat_rdma = 1;
			}
			$need_build |= $build_kernel_srpms{"${kernel_srpm}_build_kernel"};
		}
	}

	my %build_user_srpms = ();
	foreach my $srpm ( @delta_user_srpms ) {
		VerbosePrint("check if need to build $srpm\n");
		$build_user_srpms{"${srpm}_build_user"} = 0;

		# mpitests is built as part of mvapich, openmpi and mvapich2
		next if ( "$srpm" eq "mpitests" );

			$build_user_srpms{"${srpm}_build_user"} = 
					need_build_srpm($srpm, "user", "$K_VER", $installing_list,
							$force_srpm || $force_user_srpm,$prompt_srpm);
		$need_build |= $build_user_srpms{"${srpm}_build_user"};
	}

	if (! $need_build) {
		return 0;	# success
	}

	# -------------------------------------------------------------------------
	# check OS dependencies for all srpms which we will build
	my $dep_error = 0;

	NormalPrint "Checking OS Dependencies needed for builds...\n";

	if(!$skip_kernel) {
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
	}

	foreach my $srpm ( @delta_user_srpms ) {
		# mpitests is built as part of mvapich, openmpi and mvapich2
		next if ( "$srpm" eq "mpitests" );

		my $build_user = $build_user_srpms{"${srpm}_build_user"};

		next if ( ! ($build_user));

		VerbosePrint("check dependencies for $srpm\n");

		if (check_rpmbuild_dependencies($srpm)) {
			DebugPrint "$srpm rpmbuild dependency failure\n";
			$dep_error = 1;
		}
		if ($build_user) {
			DebugPrint "Check $srpm user build prereqs\n";
			if (check_build_dependencies($srpm)) {
				$dep_error = 1;
			}
			if (rpm_check_build_os_prereqs("any", $srpm, 
						@{ $delta_srpm_info{$srpm}{'BuildPrereq'}})) {
				DebugPrint "$srpm prereqs dependency failure\n";
				$dep_error = 1;
			}
		}
	}
	if ($dep_error) {
		NormalPrint "ERROR - unable to perform builds due to need for additional OS rpms\n";
		return 1;	# failure
	}

	# -------------------------------------------------------------------------
	# perform the builds
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};

	my $must_force_rpm = 0;	# set if we rebuild something so force updates
	my @need_install = ( );	# keep track of PostReqs not yet installed
	my @installed = ();	# rpms we installed to facilitate builds

	if ("$prefix" ne get_delta_rpm_prefix($rpmsdir)) {
		#system("rm -rf $rpmsdir");	# get rid of stuff with old prefix
		save_delta_rpm_prefix($rpmsdir, $prefix);
	}

	# use a different directory for BUILD_ROOT to limit conflict with OFED
	if ("$build_temp" eq "" ) {
		$build_temp = "/var/tmp/IntelOPA-DELTA";
	}
	my $BUILD_ROOT="$build_temp/build";
	my $RPM_DIR="$build_temp/DELTARPMS";
	my $mandir = rpm_query_param("_mandir");
	my $resfileop = "replace";	# replace for 1st build, append for rest

	system("rm -rf ${build_temp}");

	# use a different directory for BUILD_ROOT to limit conflict with OFED
	if (0 != system("mkdir -p $BUILD_ROOT $RPM_DIR/BUILD $RPM_DIR/RPMS $RPM_DIR/SOURCES $RPM_DIR/SPECS $RPM_DIR/SRPMS")) {
		NormalPrint "ERROR - mkdir -p $BUILD_ROOT $RPM_DIR/BUILD $RPM_DIR/RPMS $RPM_DIR/SOURCES $RPM_DIR/SPECS $RPM_DIR/SRPMS FAILED\n";
		return 1;	# failure
	}

	# OFED has all the ULPs in a single compat-rdma RPM.  We build that
	# RPM here from the compat-rdma SRPM with all ULPs included.
	# Later during install we remove ULPs not desired after installing
	# the compat-rdma RPM
	if ($build_compat_rdma)
	{
		my $OFA_KERNEL_SRC_RPM=delta_srpm_file($srcdir,"compat-rdma*.src.rpm");
		my $K_SRC = "/lib/modules/$K_VER/build";
		my $configure_options_kernel;
		my $cok_macro;
		my $rpm_release = rpm_query_attr("$srcdir/$OFA_KERNEL_SRC_RPM", "RELEASE");

		$configure_options_kernel = get_build_options($K_VER, %delta_kernel_ib_options);
		if ( $OFED_debug ) {
			# TBD --with-memtrack
			#$configure_options_kernel .= " --with-memtrack";
		}
		my $conf_opts = "--with-core-mod --with-user_mad-mod --with-user_access-mod --with-addr_trans-mod --with-ipoib-mod  --with-rdmavt-mod --with-hfi1-mod  --with-qib-mod  --with-srp-mod  --with-srp-target-mod";
		VerbosePrint("OS specific kernel configure options: '$configure_options_kernel'\n");

		if ($configure_options_kernel != "") {
			$cok_macro=" --define 'configure_options ${configure_options_kernel} $OFED_kernel_configure_options'";
		} else {
			$cok_macro=" --define 'configure_options ${conf_opts}'";
		}

		if (0 != run_build("$srcdir $OFA_KERNEL_SRC_RPM $RPM_KERNEL_ARCH $K_VER", "$srcdir",
				 "rpmbuild --rebuild --define '_topdir ${RPM_DIR}'"
        		.		" --target $RPM_KERNEL_ARCH"
				.		" --define '_prefix ${prefix}'"
				.		" --buildroot '${BUILD_ROOT}'"
				.		" --define 'build_root ${BUILD_ROOT}'"
				.		$cok_macro
				.		" --define 'KVERSION ${K_VER}'"
				.		" --define 'KSRC ${K_SRC}'"
				.		" --define 'build_kernel_ib 1'"
				.		" --define 'build_kernel_ib_devel 1'"
				.		" --define 'network_dir ${NETWORK_CONF_DIR}'"
            	.		" --define '__arch_install_post %{nil}'"
				.		" --define '_release $rpm_release'"
				.		" ${OFA_KERNEL_SRC_RPM}",
				"$resfileop"
				)) {
			return 1;	# failure
		}
		$must_force_rpm=1;
		$resfileop = "append";
		delta_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmsdir");
	}
	@need_install = ( @need_install, split /[[:space:]]+/, $delta_srpm_info{'compat-rdma'}{'PostReq'});

	if(!$skip_kernel) {
		foreach my $srpm ( @delta_kernel_srpms ) {
			VerbosePrint("process $srpm\n");

			# compat-rdma is special cased above skip it here
			next if ( "$srpm" eq "compat-rdma" );

			my $build_kernel = $build_kernel_srpms{"${srpm}_build_kernel"};

			if ($build_kernel) {
				$resfileop = "append";
				if (0 != build_srpm($srpm, $RPM_DIR, $BUILD_ROOT, $prefix, $resfileop)) {
					return 1;	# failure
				}
				@need_install = ( @need_install, split /[[:space:]]+/, $delta_srpm_info{$srpm}{'PostReq'});
				$must_force_rpm=1;
				delta_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmsdir");
			}
		}
	}

	foreach my $srpm ( @delta_user_srpms ) {
		VerbosePrint("process $srpm\n");

		# mpitests is built as part of mvapich, openmpi and mvapich2
		next if ( "$srpm" eq "mpitests" );

		my $build_user = $build_user_srpms{"${srpm}_build_user"};

			if ($build_user) {
				# load rpms which are were PostReqs from previous srpm builds
				delta_install_needed_rpms($install_list, $K_VER, $force_rpm||$must_force_rpm, $prompt_rpm, $rpmsdir, @need_install);
				@installed = ( @installed, @need_install);
				@need_install = ();
				$must_force_rpm=0;

				# build it
				if ("$srpm" eq "mvapich2" ) {
					if (0 != run_build("mvapich2_gcc and mpitests_mvapich2_gcc", "$srcdir", "STACK_PREFIX='$prefix' BUILD_DIR='$build_temp' MPICH_PREFIX= CONFIG_OPTIONS='$OFED_user_configure_options' INSTALL_ROOT='$ROOT' ./do_mvapich2_build -d -i gcc", $resfileop)) {
						return 1;	# failure
					}
					# build already installed mvapich2_gcc
					@installed = ( @installed, split /[[:space:]]+/, 'mvapich2_gcc');
					$resfileop = "append";
					$must_force_rpm=1;

					delta_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmsdir");

					if (0 != run_build("mvapich2_gcc_hfi and mpitests_mvapich2_gcc_hfi", "$srcdir", "STACK_PREFIX='$prefix' BUILD_DIR='$build_temp' MPICH_PREFIX= CONFIG_OPTIONS='$OFED_user_configure_options' INSTALL_ROOT='$ROOT' ./do_mvapich2_build -d -i -O  gcc", $resfileop)) {
						return 1;	# failure
					}
					# build already installed mvapich2_gcc_hfi
					@installed = ( @installed, split /[[:space:]]+/, 'mvapich2_gcc_hfi');
					$resfileop = "append";
					$must_force_rpm=1;

				} elsif ("$srpm" eq "openmpi" ) {
					if (0 != run_build("openmpi_gcc and mpitests_openmpi_gcc", "$srcdir", "STACK_PREFIX='$prefix' BUILD_DIR='$build_temp' MPICH_PREFIX= CONFIG_OPTIONS='$OFED_user_configure_options' INSTALL_ROOT='$ROOT' ./do_openmpi_build -d -i gcc", $resfileop)) {
						return 1;	# failure
					}
					# build already installed openmpi_gcc
					@installed = ( @installed, split /[[:space:]]+/, 'openmpi_gcc');
					$resfileop = "append";
					$must_force_rpm=1;

					delta_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmsdir");

					if (0 != run_build("openmpi_gcc_hfi and mpitests_openmpi_gcc_hfi", "$srcdir", "STACK_PREFIX='$prefix' BUILD_DIR='$build_temp' MPICH_PREFIX= CONFIG_OPTIONS='$OFED_user_configure_options' INSTALL_ROOT='$ROOT' ./do_openmpi_build -d -i -O gcc", $resfileop)) {
						return 1;	# failure
					}
					# build already installed openmpi_gcc
					@installed = ( @installed, split /[[:space:]]+/, 'openmpi_gcc_hfi');
					$resfileop = "append";
					$must_force_rpm=1;
				} else {	# all non-MPI user RPMs
					if ($build_user) {
						if (0 != build_srpm($srpm, $RPM_DIR, $BUILD_ROOT, $prefix, $resfileop)) {
							return 1;	# failure
						}
						$resfileop = "append";
					}
					$resfileop = "append";
					@need_install = ( @need_install, split /[[:space:]]+/, $delta_srpm_info{$srpm}{'PostReq'});
					$must_force_rpm=1;
				}
				delta_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmsdir");
			} else {
				@need_install = ( @need_install, split /[[:space:]]+/, $delta_srpm_info{$srpm}{'PostReq'});
			}
		}

	# get rid of rpms we installed to enable builds but are not desired to stay
	# eg. uninstall rpms which were installed but are not part of install_list
	delta_rpm_uninstall_not_needed_list($install_list, "", "", "verbose", @installed);

	if (! $debug) {
		system("rm -rf ${build_temp}");
	} else {
		LogPrint "Build remnants left in $BUILD_ROOT and $RPM_DIR\n";
	}

	return 0;	# success
}

# forward declarations
sub installed_delta_opa_stack();

# track if install_kernel_ib function was used so we only install
# compat-rdma once in a given "Perform" of install menu
my $install_kernel_ib_was_run = 0;

# return 0 on success, != 0 otherwise
sub uninstall_old_delta_rpms($$$)
{
	my $mode = shift();	# "user" or kernel rev
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

	# SLES11 includes an old version of OpenMPI that other packages may depend on, but 
	# which must be removed to prevent conflicts with the new version that we are installing. 
	#if ("$CUR_DISTRO_VENDOR" eq 'SuSE' && "$CUR_VENDOR_VER" eq 'ES11') {
	#	DebugPrint("Forcing Uninstall of SLES11 OpenMPI\n");
	#	if (rpm_uninstall_matches("SLES11 OpenMPI", "openmpi-1.2.8", "", "--nodeps")) {
	#		NormalPrint "Unable to uninstall existing openmpi installation.\n";
	#	}
	#}
	# SLES11 includes an old version of OpenMPI that other packages may depend on, but needs to be unselected in mpi-selector
	if ("$CUR_DISTRO_VENDOR" eq 'SuSE' && "$CUR_VENDOR_VER" eq 'ES11' && $mode eq 'any') {
		LogPrint "mpi-selector --unset --system >/dev/null 2>/dev/null\n";
		system("mpi-selector --unset --system >/dev/null 2>/dev/null");
		LogPrint "mpi-selector --unset --user >/dev/null 2>/dev/null\n";
		system("mpi-selector --unset --user >/dev/null 2>/dev/null");
	}

	# uninstall all present version OFA rpms, just to be safe
	foreach my $i ( reverse(@delta_components) ) {
		@packages = (@packages, @{ $delta_comp_info{$i}{'DebugRpms'}});
		@packages = (@packages, @{ $delta_comp_info{$i}{'UserRpms'}});
		@packages = (@packages, @{ $delta_comp_info{$i}{'KernelRpms'}});
	}

	# workaround LAM and other MPIs usng mpi-selector
	# we uninstall mpi-selector separately and ignore failures for its uninstall
	my @filtered_packages = ();
	my @rest_packages = ();
	foreach my $i ( @packages ) {
		if ( $delta_rpm_info{$i}{'Available'} == 0 ) {
			next; # skip, rpm was not installed.
		}
		if (scalar(grep /^$i$/, (@filtered_packages, @rest_packages)) > 0) {
			# skip, already in list
		} elsif ( "$i" eq "mpi-selector" ) {
			@rest_packages = (@rest_packages, "$i");
		} elsif ("$i" eq "openmpi") {
			if ("$CUR_DISTRO_VENDOR" eq 'SuSE' && "$CUR_VENDOR_VER" eq 'ES11') {
				# SLES11 openmpi is used by boost
				# keep openmpi for now
				#@filtered_packages = (@filtered_packages, "boost-devel", "libboost_mpi1_36_0", "$i");
				# try to remove it, but ignore errors
				@rest_packages = (@rest_packages, "$i");
			} else {
				@filtered_packages = (@filtered_packages, "$i");
			}
		} else {
			@filtered_packages = (@filtered_packages, "$i");
		}
	}

	$ret ||= rpm_uninstall_all_list_with_options($mode, " --nodeps ", $verbosity, @filtered_packages);

	# ignore errors uninstalling mpi-selector
	if (scalar(@rest_packages) != 0) {
		if (rpm_uninstall_all_list_with_options($mode, " --nodeps ", $verbosity, @rest_packages) && ! $ret) {
			NormalPrint "The previous errors can be ignored\n";
		}
	}

	if (rpm_uninstall_all_list_with_options($mode, " --nodeps ", $verbosity, @prev_release_rpms) && ! $ret) {
		NormalPrint "The previous errors can be ignored\n";
	}

	delta_cleanup_mpitests();

	if ( $ret ) {
		NormalPrint "Unable to uninstall $message RPMs\n";
	}
	return $ret;
}


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

sub media_version_delta()
{
	# all OFED components at same version as opa_stack
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};
	return `cat "$srcdir/Version"`;
}

sub delta_save_autostart()
{
	foreach my $comp ( @delta_components ) {
  		if ($ComponentInfo{$comp}{'HasStart'}
			&& $delta_comp_info{$comp}{'StartupScript'} ne "") { 
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
			&& $delta_comp_info{$comp}{'StartupScript'} ne "") { 
			comp_disable_autostart2($comp, 1);
		}
	}
}

# makes sure needed OFED components are already built, builts/rebuilds as needed
# called for every delta component's preinstall, noop for all but
# first OFED component in installing_list
sub preinstall_delta($$$)
{
	my $comp = shift();			# calling component
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	# make sure flag cleared so will install this if part of an installed comp
	$install_kernel_ib_was_run = 0;

	# ignore non-delta components at start of installing_list
	my @installing = split /[[:space:]]+/, $installing_list;
	while (scalar(@installing) != 0
			&& ("$installing[0]" eq ""
			 	|| " @delta_components " !~ / $installing[0] /)) {
		shift @installing;
	}
	# now, only do work if $comp is the 1st delta component in installing_list
	if ("$comp" eq "$installing[0]") {
		delta_save_autostart();
		init_delta_rpm_info($CUR_OS_VER);

		# Before we do any builds make sure old stacks removed so we don't
		# build against the wrong version of dependent rpms
		if (0 != uninstall_prev_versions()) {
			return 1;
		}
		if (ROOT_is_set()) {
			# we will build in current image, so must uninstall
			# in / as well as $ROOT (above)
			my $save_ROOT=$ROOT;
			$ROOT='/';
			my $rc = uninstall_prev_versions();
			$ROOT=$save_ROOT;
			if (0 != $rc) {
				return 1;
			}
		}
			
		print_separator;
		my $version=media_version_delta();
		chomp $version;
		printf("Preparing OFA $version $DBG_FREE for Install...\n");
		LogPrint "Preparing OFA $version $DBG_FREE for Install for $CUR_OS_VER\n";
		if (ROOT_is_set()) {
			# this directory is used during rpm installs, create it as needed
			if (0 != system("mkdir -p $ROOT/var/tmp")) {
				NormalPrint("Unable to create $ROOT/var/tmp\n");
				return 1;
			}
		}
	
		if (ROOT_is_set()) {
			# build in current image
			my $save_ROOT=$ROOT;
			$ROOT='/';
			my $rc = build_delta("$install_list", "$installing_list", "$CUR_OS_VER",0,"",$OFED_force_rebuild);
			$ROOT=$save_ROOT;
			return $rc;
		} else {
			return build_delta("$install_list", "$installing_list", "$CUR_OS_VER",0,"",$OFED_force_rebuild);
		}
	} else {
		return 0;
	}
}

# wrapper for installed_driver to handle the fact that OFED drivers used to
# be installed under updates/kernel and now are simply installed under updates
# So we check both places so that upgrade installs can properly detect drivers
# which are in the old location
sub installed_delta_driver($$$)
{
	my $WhichDriver = shift();
	my $driver_subdir = shift();
	my $subdir = shift();

	return installed_driver($WhichDriver, "$driver_subdir/$subdir")
			|| installed_driver($WhichDriver, "$driver_subdir/kernel/$subdir");
}

# ==========================================================================
# OFED DELTA generic installation routines

# since most of OFED components are simply data driven by Rpm lists in
# delta_comp_info, we can implement these support functions which do
# most of the real work for install and uninstall of components

# OFED has a single start script but controls which ULPs are loaded via
# entries in $OPA_CONFIG (rdma.conf)
# change all StartupParams for given delta component to $newvalue
sub delta_comp_change_opa_conf_param($$)
{
	my $comp=shift();
	my $newvalue=shift();

	VerbosePrint("edit $ROOT/$OPA_CONFIG $comp StartUp set to '$newvalue'\n");
	foreach my $p ( @{ $delta_comp_info{$comp}{'StartupParams'} } ) {
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
	my $WhichStartup = $delta_comp_info{$comp}{'StartupScript'};
	my $ret = IsAutostart($WhichStartup);	# just to be safe, test this too

	# to be true, all parameters must be yes
	foreach my $p ( @{ $delta_comp_info{$comp}{'StartupParams'} } ) {
			$ret &= ( read_opa_conf_param($p, "") eq "yes");
	}
	return $ret;
}
sub autostart_desc_delta_comp($)
{
	my $comp = shift();	# component to describe
	my $WhichStartup = $delta_comp_info{$comp}{'StartupScript'};
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
	#my $WhichStartup = $delta_comp_info{$comp}{'StartupScript'};

	#opa_stack handles this: enable_autostart($WhichStartup);
	delta_comp_change_opa_conf_param($comp, "yes");
}
# disable autostart for the given capability
sub disable_autostart_delta_comp2($)
{
	my $comp = shift();	# component to disable
	#my $WhichStartup = $delta_comp_info{$comp}{'StartupScript'};

	delta_comp_change_opa_conf_param($comp, "no");
}

# OFED has all the ULPs in a single RPM.  This function removes the
# drivers from compat-rdma specific to the given delta_component
# this is a hack, but its better than uninstall and reinstall compat-rdma
# every time a ULP is added/removed
sub remove_delta_kernel_ib_drivers($$)
{
	my $comp = shift();	# component to remove
	my $verbosity = shift();
	# cheat on driver_subdir so delta_components can have some stuff not
	# in @Components
	# we know driver_subdir is same for all delta components in compat-rdma
	my $driver_subdir=$ComponentInfo{'opa_stack'}{'DriverSubdir'};

	my $i;
	my @list = split /[[:space:]]+/, $delta_comp_info{$comp}{'Drivers'};
	for ($i=0; $i < scalar(@list); $i+=2)
	{
		my $driver=$list[$i];
		my $subdir=$list[$i+1];
		remove_driver("$driver", "$driver_subdir/$subdir", $verbosity);
		remove_driver_dirs("$driver_subdir/$subdir");
	}
}

sub print_install_banner_delta_comp($)
{
	my $comp = shift();

	my $version=media_version_delta();
	chomp $version;
	printf("Installing $ComponentInfo{$comp}{'Name'} $version $DBG_FREE...\n");
	# all OFED components at same version as opa_stack
	LogPrint "Installing $ComponentInfo{$comp}{'Name'} $version $DBG_FREE for $CUR_OS_VER\n";
}

# helper to determine if we need to reinstall due to parameter change
sub need_reinstall_delta_comp($$$)
{
	my $comp = shift();
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	if (get_delta_rpm_prefix(delta_rpms_dir()) ne "$OFED_prefix" ) {
		return "all";
	} elsif (! comp_is_uptodate('opa_stack')) { # all delta_comp same version
		# on upgrade force reinstall to recover from uninstall of old rpms
		return "all";
	} else {
		return "no";
	}
}

# helper which does most of the work for installing rpms and drivers
# for an OFED DELTA component
# installs compat-rdma drivers, KernelRpms and UserRpms
# caller must handle any non-RPM files
sub install_delta_comp($$)
{
	my $comp = shift();
	my $install_list = shift();	# total that will be installed when done

	# special handling for compat-rdma
	if ($ComponentInfo{$comp}{'DriverSubdir'} ne "" ) {
		install_kernel_ib(delta_rpms_dir(), $install_list);
	}
	# skip compat-rdma if in KernelRpms/UserRpms, already handled above
	delta_rpm_install_list(delta_rpms_dir(), $CUR_OS_VER, 1,
							( @{ $delta_comp_info{$comp}{'KernelRpms'}},
							@{ $delta_comp_info{$comp}{'UserRpms'}}) );
	# DebugRpms are installed as part of 'delta_debug' component

}

sub print_uninstall_banner_delta_comp($)
{
	my $comp = shift();

	NormalPrint("Uninstalling $ComponentInfo{$comp}{'Name'}...\n");
}

# uninstall all rpms associated with an OFED Delta component
sub uninstall_delta_comp_rpms($$$$)
{
	my $comp = shift();
	my $install_list = shift();
	my $uninstalling_list = shift();
	my $verbosity = shift();

	# debuginfo never in >1 component, so do explicit uninstall since
	# have an odd PartOf relationship which confuses uninstall_not_needed_list
	rpm_uninstall_list2("any", " --nodeps ", $verbosity,
					 @{ $delta_comp_info{$comp}{'DebugRpms'}});
	delta_rpm_uninstall_not_needed_list($install_list, $uninstalling_list, $comp,
				 	$verbosity, @{ $delta_comp_info{$comp}{'UserRpms'}});
	delta_rpm_uninstall_not_needed_list($install_list, $uninstalling_list, $comp,
				 	$verbosity, @{ $delta_comp_info{$comp}{'KernelRpms'}});
}

# helper which does most of the work for uninstalling rpms and drivers
# for an OFED component
# caller must handle any non-RPM files
sub uninstall_delta_comp($$$$)
{
	my $comp = shift();
	my $install_list = shift();
	my $uninstalling_list = shift();
	my $verbosity = shift();

	uninstall_delta_comp_rpms($comp, $install_list, $uninstalling_list, $verbosity);
	remove_delta_kernel_ib_drivers($comp, $verbosity);
}

# remove compat-rdma drivers for components which will not be installed
sub remove_unneeded_kernel_ib_drivers($)
{
	my $install_list = shift();	# total that will be installed when done
	foreach my $c ( @delta_components )
	{
		if ($install_list !~ / $c /) {
			remove_delta_kernel_ib_drivers($c, "verbose");
		}
	}
}

# OFED has all the ULPs in a single RPM.  This function installs that
# RPM and removes the undesired ULP drivers
sub install_kernel_ib($$)
{
	my $rpmdir = shift();
	my $install_list = shift();	# total that will be installed when done
	my $rpmdir_t = $rpmdir;

	my $driver_subdir=$ComponentInfo{'opa_stack'}{'DriverSubdir'};	# same for all delta components

	if ( $install_kernel_ib_was_run) {
		return;
	}

	$rpmdir_t = $rpmdir;
	if ( $GPU_Install == 1 ) {
                if ( -d $rpmdir."/CUDA" ) {
                        $rpmdir_t=$rpmdir."/CUDA";
                } else {
                        NormalPrint("CUDA specific packages do not exist\n");
                        exit 0;
                }
        }

	if(!$skip_kernel) {
		foreach my $srpm ( @delta_kernel_srpms ) {
			rpm_install_with_options("$rpmdir_t", $CUR_OS_VER, $srpm, " -U --nodeps ");
		}
	}
	remove_unneeded_kernel_ib_drivers($install_list);

	# rdma.conf values not directly associated with driver startup are left
	# untouched delta rpm install will keep existing value

	$install_kernel_ib_was_run = 1;
}

# ==========================================================================
# OFED opa_stack installation

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

sub start_opa_stack()
{
	my $driver_subdir=$ComponentInfo{'opa_stack'}{'DriverSubdir'};
	start_driver($ComponentInfo{'opa_stack'}{'Name'}, "ib_core", "$driver_subdir/drivers/infiniband/core", "");
}

sub stop_opa_stack()
{
	stop_driver($ComponentInfo{'opa_stack'}{'Name'}, "ib_core", "");
}

sub available_opa_stack()
{
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};
# TBD better checks for available?
# check file_glob("$srcdir/SRPMS/compat-rdma*.src.rpm") ne ""
#			|| rpm_exists($rpmsdir, $CUR_OS_VER, "compat-rdma")
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_delta_opa_stack()
{
	my $driver_subdir=$ComponentInfo{'opa_stack'}{'DriverSubdir'};
	if ( "$CUR_VENDOR_VER" eq "ES67" ) {
		return ( -e "$ROOT$BASE_DIR/version_delta" 
				&& rpm_is_installed("libibumad", "user")
				&& rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES72" ) {
		return ( -e "$ROOT$BASE_DIR/version_delta"
				&& rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER)
				|| rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES73" ) {
		return ( -e "$ROOT$BASE_DIR/version_delta"
				&& rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq 'ES122' ) {
		return ( -e "$ROOT$BASE_DIR/version_delta"
				&& rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq 'ES12' || "$CUR_VENDOR_VER" eq 'ES121' ) {
		return (rpm_is_installed("libibumad3", "user")
				&& -e "$ROOT$BASE_DIR/version_delta"
				&& rpm_is_installed("compat-rdma", $CUR_OS_VER));
	} else {
		return (rpm_is_installed("libibumad", "user")
				&& -e "$ROOT$BASE_DIR/version_delta"
				&& rpm_is_installed("compat-rdma", $CUR_OS_VER));
	}
}

sub installed_opa_stack()
{
	return (installed_delta_opa_stack);
}

# only called if installed_opa_stack is true
sub installed_version_opa_stack()
{
	if ( -e "$ROOT$BASE_DIR/version_delta" ) {
		return `cat $ROOT$BASE_DIR/version_delta`;
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
		NormalPrint "\nUninstalling $stack: chroot /$ROOT $cmd $cmdargs\n";
		if (0 != system("yes | chroot /$ROOT $cmd $cmdargs")) {
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

	init_delta_rpm_info($osver);

	# We can't remove old ib stack because we need them for build
	# OFED_DELTA. More importantly, we don't know where are the rpms.

	# Before we do any builds make sure old stacks removed so we don't
	# build against the wrong version of dependent rpms
	#if (0 != uninstall_prev_versions()) {
	#	return 1;
	#}

	if (ROOT_is_set()) {
		# build in current image
		my $save_ROOT=$ROOT;
		$ROOT='/';
		my $rc =  build_delta("@Components", "@Components", $osver, $debug,$build_temp,$force);
		$ROOT=$save_ROOT;
		return $rc;
	} else {
		return build_delta("@Components", "@Components", $osver, $debug,$build_temp,$force);
	}
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

	print_install_banner_delta_comp('opa_stack');

	#override the udev permissions.
	install_udev_permissions("$srcdir/config");

	# setup environment variable so that RPM can configure limits conf
        setup_env("OPA_LIMITS_CONF", 1);
        # so setting up envirnment to install driver for this component. actual install is done by rpm
	setup_env("OPA_INSTALL_CALLER", 1);

	# Check $BASE_DIR directory ...exist 
	check_config_dirs();
	check_dir("/usr/lib/opa");

        prompt_opa_conf_param('ARPTABLE_TUNING', 'Adjust kernel ARP table size for large fabrics?', "y", 'OPA_ARPTABLE_TUNING');
        prompt_opa_conf_param('SRP_LOAD', 'SRP initiator autoload?', "n", 'OPA_SRP_LOAD');
        prompt_opa_conf_param('SRPT_LOAD', 'SRP target autoload?', "n", 'OPA_SRPT_LOAD');

	install_delta_comp('opa_stack', $install_list);

	# prevent distro's open IB from loading
	#add_blacklist("ib_mthca");
	#add_blacklist("ib_ipath");
	disable_distro_ofed();

	# Take care of the configuration files for srptools
	check_rpm_config_file("/etc/srp_daemon.conf");
	check_rpm_config_file("/etc/logrotate.d/srp_daemon");
	check_rpm_config_file("/etc/rsyslog.d/srp_daemon.conf");

	# Start rdma on run level 235 on RHEL67
	run_rdma_on_startup();

	need_reboot();
	$ComponentWasInstalled{'opa_stack'}=1;
}

sub postinstall_opa_stack($$)
{
	my $old_conf = 0;	# do we have an existing conf file
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	if ( -e "$ROOT/$OPA_CONFIG" ) {
		if (0 == system("cp $ROOT/$OPA_CONFIG $ROOT/$OPA_CONFIG-save")) {
			$old_conf=1;
		}
	}

	# For RHEL 7.1 and SLES 12, enable force loading for some distros modules
	if ( "$CUR_VENDOR_VER" eq "ES71" || "$CUR_VENDOR_VER" eq "ES12") {
		enable_mod_force_load("mlx4_ib");
		enable_mod_force_load("ib_qib");
	}

	# adjust rdma.conf autostart settings
	foreach my $c ( @delta_components )
	{
		if ($install_list !~ / $c /) {
			# disable autostart of uninstalled components
			# opa_stack is at least installed
			delta_comp_change_opa_conf_param($c, "no");
		} else {
			# retain previous setting for components being installed
			# set to no if initial install
			# it seems that compat-rdma rpm will do this for us,
			# repeat just to be safe
			foreach my $p ( @{ $delta_comp_info{$c}{'StartupParams'} } ) {
				my $old_value = "";
				if ( $old_conf ) {
					$old_value = read_opa_conf_param($p, "$ROOT/$OPA_CONFIG-save");
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

	my $driver_subdir=$ComponentInfo{'opa_stack'}{'DriverSubdir'};
	print_uninstall_banner_delta_comp('opa_stack');
	stop_opa_stack;
	remove_blacklist("ib_qib");

	# allow open IB to load
	#remove_blacklist("ib_mthca");
	#remove_blacklist("ib_ipath");

	uninstall_delta_comp('opa_stack', $install_list, $uninstalling_list, 'verbose');
	remove_driver_dirs($driver_subdir);
	#remove_modules_conf;
	remove_limits_conf;

	remove_udev_permissions;

	system("rm -rf $ROOT$BASE_DIR/version_delta");
	system("rm -rf $ROOT/usr/lib/opa/.comp_delta.pl");
	system "rmdir $ROOT/usr/lib/opa 2>/dev/null";	# remove only if empty
	system "rmdir $ROOT$BASE_DIR 2>/dev/null";	# remove only if empty
	system "rmdir $ROOT$OPA_CONFIG_DIR 2>/dev/null";	# remove only if empty

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

sub available_intel_hfi()
{
    my $srcdir=$ComponentInfo{'intel_hfi'}{'SrcDir'};
        return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" )
		 && build_option_is_allowed($CUR_OS_VER, "", %delta_kernel_ib_options));
}

sub installed_intel_hfi()
{
    my $driver_subdir=$ComponentInfo{'intel_hfi'}{'DriverSubdir'};
    if ( "$CUR_VENDOR_VER" eq "ES67" ) {
	return ( -e "$ROOT$BASE_DIR/version_delta"
			&& rpm_is_installed("libhfi1", "user")
                        && rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER));
    } elsif ( "$CUR_VENDOR_VER" eq "ES72" || "$CUR_VENDOR_VER" eq "ES73" ) {
        return (rpm_is_installed("libhfi1", "user")
                        && -e "$ROOT$BASE_DIR/version_delta"
                        && rpm_is_installed("kmod-ifs-kernel-updates", $CUR_OS_VER));
	} elsif ( "$CUR_VENDOR_VER" eq "ES122" ) {
		return (rpm_is_installed("libhfi1verbs-rdmav2", "user")
                        && -e "$ROOT$BASE_DIR/version_delta"
                        && rpm_is_installed("ifs-kernel-updates-kmp-default", $CUR_OS_VER));
	} else {
        return (rpm_is_installed("libhfi1", "user")
                        && -e "$ROOT$BASE_DIR/version_delta"
                        && rpm_is_installed("compat-rdma", $CUR_OS_VER));
	}
}

# only called if installed_intel_hfi is true
sub installed_version_intel_hfi()
{
    if ( -e "$ROOT$BASE_DIR/version_delta" ) {
	return `cat $ROOT$BASE_DIR/version_delta`;
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

my $irq_perm_string = "Set IrqBalance to Exact?";
AddAnswerHelp("IrqBalance", "$irq_perm_string");
my $Default_IrqBalance = 1;

sub install_intel_hfi($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled

    print_install_banner_delta_comp('intel_hfi');

    # Adjust irqbalance
    if ( -e "/etc/sysconfig/irqbalance" ) {
		print "Intel strongly recommends that the irqbalance service be enabled\n";
		print "and run using the --hintpolicy=exact option.\n";
        $Default_IrqBalance = GetYesNoWithMemory("IrqBalance", 1, "$irq_perm_string", "y");
        if ( $Default_IrqBalance == 1 ) {
            #set env variable so that RPM can do post install configuration of IRQBALANCE
            #  if opasystemconfig already exists, set it manually
            if ( -f "/sbin/opasystemconfig" ) {
                system("/sbin/opasystemconfig --enable Irq_Balance");
            } else {
                setup_env("OPA_IRQBALANCE", 1);
            }
        }
    }
    install_delta_comp('intel_hfi', $install_list);

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

    print_uninstall_banner_delta_comp('intel_hfi');
        # TBD stop_intel_hfi;
    uninstall_delta_comp('intel_hfi', $install_list, $uninstalling_list, 'verbose');
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
# ib_wfr_lite installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_ib_wfr_lite()
{
    my $WhichStartup = $delta_comp_info{'ib_wfr_lite'}{'StartupScript'};
	my $ret = IsAutostart($WhichStartup);

	return ($ret && ! is_blacklisted('ib_wfr_lite'));
}
sub autostart_desc_ib_wfr_lite()
{
    return autostart_desc_delta_comp('ib_wfr_lite');
}
# enable autostart for the given capability
sub enable_autostart2_ib_wfr_lite()
{
	remove_blacklist('ib_wfr_lite');
	rebuild_ramdisk();
}
# disable autostart for the given capability
sub disable_autostart2_ib_wfr_lite()
{
	add_blacklist('ib_wfr_lite');
	rebuild_ramdisk();
}

sub available_ib_wfr_lite()
{
    my $srcdir=$ComponentInfo{'ib_wfr_lite'}{'SrcDir'};
        return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" )
		 && build_option_is_allowed($CUR_OS_VER, "", %delta_kernel_ib_options));
}

sub installed_ib_wfr_lite()
{
    my $driver_subdir=$ComponentInfo{'ib_wfr_lite'}{'DriverSubdir'};
        return (rpm_is_installed("ib_wfr_lite", "user")
                        && -e "$ROOT$BASE_DIR/version_delta"
                        #&& rpm_is_installed("compat-rdma", $CUR_OS_VER)
	    );
}

# only called if installed_ib_wfr_lite is true
sub installed_version_ib_wfr_lite()
{
    if ( -e "$ROOT$BASE_DIR/version_delta" ) {
	return `cat $ROOT$BASE_DIR/version_delta`;
    } else {
	return "";
    }
}

# only called if available_ib_wfr_lite is true
sub media_version_ib_wfr_lite()
{
    return media_version_delta();
}

sub build_ib_wfr_lite($$$$)
{
    my $osver = shift();
    my $debug = shift();    # enable extra debug of build itself
    my $build_temp = shift();       # temp area for use by build
    my $force = shift();    # force a rebuild
    return 0;       # success
}

sub need_reinstall_ib_wfr_lite($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled

    return (need_reinstall_delta_comp('ib_wfr_lite', $install_list, $installing_list));
}

sub preinstall_ib_wfr_lite($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled

    return preinstall_delta("ib_wfr_lite", $install_list, $installing_list);
}

sub install_ib_wfr_lite($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled

    print_install_banner_delta_comp('ib_wfr_lite');
    install_delta_comp('ib_wfr_lite', $install_list);

    # because the ib_wfr_lite and qib drivers are attached to the same hardware IDs
    # only one can be active in the system at a given time so blacklist the one that
    # won't currently be in use.
    remove_blacklist("ib_wfr_lite");
    add_blacklist("ib_qib");

    need_reboot();
    $ComponentWasInstalled{'ib_wfr_lite'}=1;
}

sub postinstall_ib_wfr_lite($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled
    delta_restore_autostart('ib_wfr_lite');
}

sub uninstall_ib_wfr_lite($$)
{
    my $install_list = shift();     # total that will be left installed when done
    my $uninstalling_list = shift();        # what items are being uninstalled

    print_uninstall_banner_delta_comp('ib_wfr_lite');
    uninstall_delta_comp('ib_wfr_lite', $install_list, $uninstalling_list, 'verbose');

    # because the ib_wfr_lite and qib drivers are attached to the same hardware IDs
    # only one can be active in the system at a given time so blacklist the one that
    # won't currently be in use.
    add_blacklist("ib_wfr_lite");
    remove_blacklist("ib_qib");

    need_reboot();
    $ComponentWasInstalled{'ib_wfr_lite'}=0;
}

# ==========================================================================
# OFED opa_stack development installation

sub available_opa_stack_dev()
{
	my $srcdir=$ComponentInfo{'opa_stack_dev'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_opa_stack_dev()
{
	return (rpm_is_installed("libibumad-devel", "user")
			&& -e "$ROOT$BASE_DIR/version_delta");
}

# only called if installed_opa_stack_dev is true
sub installed_version_opa_stack_dev()
{
	if ( -e "$ROOT$BASE_DIR/version_delta" ) {
		return `cat $ROOT$BASE_DIR/version_delta`;
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

	print_install_banner_delta_comp('opa_stack_dev');
	install_delta_comp('opa_stack_dev', $install_list);

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

	print_uninstall_banner_delta_comp('opa_stack_dev');
	uninstall_delta_comp('opa_stack_dev', $install_list, $uninstalling_list, 'verbose');
	$ComponentWasInstalled{'opa_stack_dev'}=0;
}

# ==========================================================================
# OFED delta_ipoib installation

my $FirstIPoIBInterface=0; # first device is ib0

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

sub available_delta_ipoib()
{
	my $srcdir=$ComponentInfo{'delta_ipoib'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_delta_ipoib()
{
	if ( "$CUR_VENDOR_VER" eq "ES67" ) {
		return (( -e "$ROOT$BASE_DIR/version_delta"
			&& rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)));
	}
	return 1;
}

# only called if installed_delta_ipoib is true
sub installed_version_delta_ipoib()
{
	return `cat $ROOT$BASE_DIR/version_delta`;
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

	print_install_banner_delta_comp('delta_ipoib');
	install_delta_comp('delta_ipoib', $install_list);

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

	print_uninstall_banner_delta_comp('delta_ipoib');
	# TBD stop_delta_ipoib;
	uninstall_delta_comp('delta_ipoib', $install_list, $uninstalling_list, 'verbose');
	Remove_ifcfg("ib_ipoib","$ComponentInfo{'delta_ipoib'}{'Name'}","ib");
	need_reboot();
	$ComponentWasInstalled{'delta_ipoib'}=0;
}

# ==========================================================================
# OFED DELTA mpi-selector installation

sub available_mpi_selector()
{
	my $srcdir=$ComponentInfo{'mpi_selector'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_mpi_selector()
{
	return (rpm_is_installed("mpi-selector", "user")
			&& -e "$ROOT$BASE_DIR/version_delta");
}

# only called if installed_mpi_selector is true
sub installed_version_mpi_selector()
{
	if ( -e "$ROOT$BASE_DIR/version_delta" ) {
		return `cat $ROOT$BASE_DIR/version_delta`;
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

	print_install_banner_delta_comp('mpi_selector');
	install_delta_comp('mpi_selector', $install_list);

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

	uninstall_delta_comp('mpiRest', $install_list, $uninstalling_list, 'verbose');
	print_uninstall_banner_delta_comp('mpi_selector');
	uninstall_delta_comp('mpi_selector', $install_list, $uninstalling_list, 'verbose');
	delta_cleanup_mpitests();
	$ComponentWasInstalled{'mpi_selector'}=0;
}

# ==========================================================================
# OFED MVAPICH2 for gcc installation

sub available_mvapich2()
{
	my $srcdir=$ComponentInfo{'mvapich2'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_mvapich2()
{
	return ((rpm_is_installed("mvapich2_gcc", "user")
			&& -e "$ROOT$BASE_DIR/version_delta"));
}

# only called if installed_mvapich2 is true
sub installed_version_mvapich2()
{
	return `cat $ROOT$BASE_DIR/version_delta`;
}

# only called if available_mvapich2 is true
sub media_version_mvapich2()
{
	return media_version_delta();
}

sub build_mvapich2($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_mvapich2($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_delta_comp('mvapich2', $install_list, $installing_list));
}

sub check_os_prereqs_mvapich2
{
	return rpm_check_os_prereqs("mvapich2", "user");
}

sub preinstall_mvapich2($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("mvapich2", $install_list, $installing_list);
}

sub install_mvapich2($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_delta_comp('mvapich2');

	# make sure any old potentially custom built versions of mpi are uninstalled
	rpm_uninstall_list2("any", " --nodeps ", 'silent', @{ $delta_comp_info{'mvapich2'}{'UserRpms'}});
	my $rpmfile = rpm_resolve(delta_rpms_dir(), "any", "mvapich2_gcc");
	if ( "$rpmfile" ne "" && -e "$rpmfile" ) {
		my $mpich_prefix= "$OFED_prefix/mpi/gcc/mvapich2-"
	   							. rpm_query_attr($rpmfile, "VERSION");
		if ( -d "$mpich_prefix" && GetYesNo ("Remove $mpich_prefix directory?", "y")) {
			LogPrint "rm -rf $mpich_prefix\n";
			system("rm -rf $mpich_prefix");
		}
	}

	install_delta_comp('mvapich2', $install_list);

	$ComponentWasInstalled{'mvapich2'}=1;
}

sub postinstall_mvapich2($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#delta_restore_autostart('mvapich2');
}

sub uninstall_mvapich2($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_delta_comp('mvapich2');
	uninstall_delta_comp('mvapich2', $install_list, $uninstalling_list, 'verbose');
	delta_cleanup_mpitests();
	$ComponentWasInstalled{'mvapich2'}=0;
}

# ==========================================================================
# OFED OpenMpi for gcc installation

sub available_openmpi()
{
	my $srcdir=$ComponentInfo{'openmpi'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_openmpi()
{
	return ((rpm_is_installed("openmpi_gcc_hfi", "user")
			&& -e "$ROOT$BASE_DIR/version_delta"));
}

# only called if installed_openmpi is true
sub installed_version_openmpi()
{
	return `cat $ROOT$BASE_DIR/version_delta`;
}

# only called if available_openmpi is true
sub media_version_openmpi()
{
	return media_version_delta();
}

sub build_openmpi($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_openmpi($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_delta_comp('openmpi', $install_list, $installing_list));
}

sub check_os_prereqs_openmpi
{
	return rpm_check_os_prereqs("openmpi", "user");
}

sub preinstall_openmpi($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("openmpi", $install_list, $installing_list);
}

sub install_openmpi($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_delta_comp('openmpi');

	# make sure any old potentially custom built versions of mpi are uninstalled
	rpm_uninstall_list2("any", " --nodeps ", 'silent', @{ $delta_comp_info{'openmpi'}{'UserRpms'}});
	my $rpmfile = rpm_resolve(delta_rpms_dir(), "any", "openmpi_gcc");
	if ( "$rpmfile" ne "" && -e "$rpmfile" ) {
		my $mpich_prefix= "$OFED_prefix/mpi/gcc/openmpi-"
	   							. rpm_query_attr($rpmfile, "VERSION");
		if ( -d "$mpich_prefix" && GetYesNo ("Remove $mpich_prefix directory?", "y")) {
			LogPrint "rm -rf $mpich_prefix\n";
			system("rm -rf $mpich_prefix");
		}
	}

	install_delta_comp('openmpi', $install_list);

	$ComponentWasInstalled{'openmpi'}=1;
}

sub postinstall_openmpi($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#delta_restore_autostart('openmpi');
}

sub uninstall_openmpi($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_delta_comp('openmpi');
	uninstall_delta_comp('openmpi', $install_list, $uninstalling_list, 'verbose');
	delta_cleanup_mpitests();
	$ComponentWasInstalled{'openmpi'}=0;
}

# ==========================================================================
# OFED gasnet for gcc installation

sub available_gasnet()
{
	my $srcdir=$ComponentInfo{'gasnet'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_gasnet()
{
	return ((rpm_is_installed("gasnet_gcc_hfi", "user")
			&& -e "$ROOT$BASE_DIR/version_delta"));
}

# only called if installed_gasnet is true
sub installed_version_gasnet()
{
	return `cat $ROOT$BASE_DIR/version_delta`;
}

# only called if available_gasnet is true
sub media_version_gasnet()
{
	return media_version_delta();
}

sub build_gasnet($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_gasnet($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_delta_comp('gasnet', $install_list, $installing_list));
}

sub preinstall_gasnet($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("gasnet", $install_list, $installing_list);
}

sub install_gasnet($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_delta_comp('gasnet');

	# make sure any old potentially custom built versions of mpi are uninstalled
	rpm_uninstall_list2("any", " --nodeps ", 'silent', @{ $delta_comp_info{'gasnet'}{'UserRpms'}});
	my $rpmfile = rpm_resolve(delta_rpms_dir(), "any", "gasnet");
	if ( "$rpmfile" ne "" && -e "$rpmfile" ) {
		my $mpich_prefix= "$OFED_prefix/shmem/gcc/gasnet-"
	   							. rpm_query_attr($rpmfile, "VERSION");
		if ( -d "$mpich_prefix" && GetYesNo ("Remove $mpich_prefix directory?", "y")) {
			LogPrint "rm -rf $mpich_prefix\n";
			system("rm -rf $mpich_prefix");
		}
	}

	install_delta_comp('gasnet', $install_list);

	$ComponentWasInstalled{'gasnet'}=1;
}

sub postinstall_gasnet($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#delta_restore_autostart('gasnet');
}

sub uninstall_gasnet($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_delta_comp('gasnet');
	uninstall_delta_comp('gasnet', $install_list, $uninstalling_list, 'verbose');
#	delta_cleanup_mpitests();
	$ComponentWasInstalled{'gasnet'}=0;
}

sub check_os_prereqs_gasnet()
{
	return rpm_check_os_prereqs("gasnet", "user");
}

# ==========================================================================
# OFED openshmem for gcc installation

sub available_openshmem()
{
	my $srcdir=$ComponentInfo{'openshmem'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_openshmem()
{
	return ((rpm_is_installed("openshmem_gcc_hfi", "user")
			&& -e "$ROOT$BASE_DIR/version_delta"));
}

# only called if installed_openshmem is true
sub installed_version_openshmem()
{
	return `cat $ROOT$BASE_DIR/version_delta`;
}

# only called if available_openshmem is true
sub media_version_openshmem()
{
	return media_version_delta();
}

sub build_openshmem($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_openshmem($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_delta_comp('openshmem', $install_list, $installing_list));
}

sub preinstall_openshmem($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("openshmem", $install_list, $installing_list);
}

sub install_openshmem($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_delta_comp('openshmem');

	# make sure any old potentially custom built versions of mpi are uninstalled
	rpm_uninstall_list2("any", " --nodeps ", 'silent', @{ $delta_comp_info{'openshmem'}{'UserRpms'}});
	my $rpmfile = rpm_resolve(delta_rpms_dir(), "any", "openshmem");
	if ( "$rpmfile" ne "" && -e "$rpmfile" ) {
		my $mpich_prefix= "$OFED_prefix/shmem/gcc/openshmem-"
	   							. rpm_query_attr($rpmfile, "VERSION");
		if ( -d "$mpich_prefix" && GetYesNo ("Remove $mpich_prefix directory?", "y")) {
			LogPrint "rm -rf $mpich_prefix\n";
			system("rm -rf $mpich_prefix");
		}
	}

	install_delta_comp('openshmem', $install_list);

	$ComponentWasInstalled{'openshmem'}=1;
}

sub postinstall_openshmem($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#delta_restore_autostart('openshmem');
}

sub uninstall_openshmem($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_delta_comp('openshmem');
	uninstall_delta_comp('openshmem', $install_list, $uninstalling_list, 'verbose');
#	delta_cleanup_mpitests();
	$ComponentWasInstalled{'openshmem'}=0;
}

sub check_os_prereqs_openshmem
{
	return rpm_check_os_prereqs("openshmem", "user");
}

# ==========================================================================
# OFED DELTA delta_mpisrc installation

sub available_delta_mpisrc()
{
	my $srcdir=$ComponentInfo{'delta_mpisrc'}{'SrcDir'};
# TBD better checks for available?
# check file_glob("$srcdir/SRPMS/mvapich-*.src.rpm") ne ""
# check file_glob("$srcdir/SRPMS/mvapich2-*.src.rpm") ne ""
# check file_glob("$srcdir/SRPMS/openmpi-*.src.rpm") ne ""
	return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" ) );
}

sub installed_delta_mpisrc()
{
	return ((-e "$ROOT$BASE_DIR/version_delta"
			&& file_glob("$ROOT/usr/src/opa/MPI/mvapich*.src.rpm") ne ""
			&& file_glob("$ROOT/usr/src/opa/MPI/openmpi*.src.rpm") ne ""
			&& file_glob("$ROOT/usr/src/opa/MPI/mpitests*.src.rpm") ne ""));
}

# only called if installed_delta_mpisrc is true
sub installed_version_delta_mpisrc()
{
	return `cat $ROOT$BASE_DIR/version_delta`;
}

# only called if available_delta_mpisrc is true
sub media_version_delta_mpisrc()
{
	return media_version_delta();
}

sub build_delta_mpisrc($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_delta_mpisrc($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_delta_comp('delta_mpisrc', $install_list, $installing_list));
}

sub check_os_prereqs_delta_mpisrc
{
	return rpm_check_os_prereqs("delta_mpisrc", "any");
}

sub preinstall_delta_mpisrc($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_delta("delta_mpisrc", $install_list, $installing_list);
}

sub install_delta_mpisrc($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	my $srcdir=$ComponentInfo{'delta_mpisrc'}{'SrcDir'};

	print_install_banner_delta_comp('delta_mpisrc');
	install_delta_comp('delta_mpisrc', $install_list);
	check_dir("/usr/src/opa");
	check_dir("/usr/src/opa/MPI");
	# remove old versions (.src.rpm and built .rpm files too)
	system "rm -rf $ROOT/usr/src/opa/MPI/mvapich[-_]*.rpm 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/mvapich2[-_]*.rpm 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/openmpi[-_]*.rpm 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/mpitests[-_]*.rpm 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/make.*.res 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/make.*.err 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/make.*.warn 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/.mpiinfo 2>/dev/null";

	# install new versions
	foreach my $srpm ( "mvapich2", "openmpi", "mpitests" ) {
		my $srpmfile = file_glob("$srcdir/$SRPMS_SUBDIR/${srpm}-*.src.rpm");
		if ( "$srpmfile" ne "" ) {
			my $file = my_basename($srpmfile);
			copy_data_file($srpmfile, "/usr/src/opa/MPI/$file");
		}
	}
	copy_systool_file("$srcdir/do_build", "/usr/src/opa/MPI/do_build");
	copy_systool_file("$srcdir/do_mvapich2_build", "/usr/src/opa/MPI/do_mvapich2_build");
	copy_systool_file("$srcdir/do_openmpi_build", "/usr/src/opa/MPI/do_openmpi_build");

	$ComponentWasInstalled{'delta_mpisrc'}=1;
}

sub postinstall_delta_mpisrc($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#delta_restore_autostart('delta_mpisrc');
}

sub uninstall_delta_mpisrc($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_delta_comp('delta_mpisrc');

	# remove old versions (.src.rpm and built .rpm files too)
	system "rm -rf $ROOT/usr/src/opa/MPI/mvapich2[-_]*.rpm 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/openmpi[-_]*.rpm 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/mpitests[-_]*.rpm 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/make.*.res 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/make.*.err 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/make.*.warn 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/.mpiinfo 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/do_build 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/do_mvapich2_build 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/do_openmpi_build 2>/dev/null";
	system "rm -rf $ROOT/usr/src/opa/MPI/.mpiinfo 2>/dev/null";

	uninstall_delta_comp('delta_mpisrc', $install_list, $uninstalling_list, 'verbose');
	system "rmdir $ROOT/usr/src/opa/MPI 2>/dev/null"; # remove only if empty
	system "rmdir $ROOT/usr/src/opa 2>/dev/null"; # remove only if empty
	$ComponentWasInstalled{'delta_mpisrc'}=0;
}

# ==========================================================================
# OFED delta_debug installation

# this is an odd component.  It consists of the debuginfo files which
# are built and identified in DebugRpms in other components.  Installing this
# component installs the debuginfo files for the installed components.
# uninstalling this component gets rid of all debuginfo files.
# uninstalling other components will get rid of individual debuginfo files
# for those components

sub available_delta_debug()
{
	my $srcdir=$ComponentInfo{'delta_debug'}{'SrcDir'};
	return (( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS")
			&&	$delta_rpm_info{'libibumad-debuginfo'}{'Available'});
}

sub installed_delta_debug()
{
	return (rpm_is_installed("libibumad-debuginfo", "user")
			&& -e "$ROOT$BASE_DIR/version_delta");
}

# only called if installed_delta_debug is true
sub installed_version_delta_debug()
{
	if ( -e "$ROOT$BASE_DIR/version_delta" ) {
		return `cat $ROOT$BASE_DIR/version_delta`;
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
		foreach my $comp ( @delta_components ) {
			if ( " $installing_list " =~ / $comp /
				 && 0 != scalar(@{ $delta_comp_info{$comp}{'DebugRpms'}})) {
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

	print_install_banner_delta_comp('delta_debug');
	install_delta_comp('delta_debug', $install_list);

	# install DebugRpms for each installed component
	foreach my $comp ( @delta_components ) {
		if ( " $install_list " =~ / $comp / ) {
			delta_rpm_install_list(delta_rpms_dir(), $CUR_OS_VER, 1,
							( @{ $delta_comp_info{$comp}{'DebugRpms'}}));
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

	print_uninstall_banner_delta_comp('delta_debug');

	uninstall_delta_comp('delta_debug', $install_list, $uninstalling_list, 'verbose');
	# uninstall debug rpms for all components
	# debuginfo never in >1 component, so do explicit uninstall since
	# have an odd PartOf relationship which confuses uninstall_not_needed_list
	foreach my $comp ( reverse(@delta_components) ) {
		rpm_uninstall_list2("any", " --nodeps ", 'verbose',
					 @{ $delta_comp_info{$comp}{'DebugRpms'}});
	}
	$ComponentWasInstalled{'delta_debug'}=0;
}

# ==========================================================================
# OFED DELTA ibacm installation

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
	enable_autostart($delta_comp_info{'ibacm'}{'StartupScript'});
}
# disable autostart for the given capability
sub disable_autostart2_ibacm()
{
	disable_autostart($delta_comp_info{'ibacm'}{'StartupScript'});
}

sub start_ibacm()
{
	my $prefix = delta_get_prefix();
	start_utility($ComponentInfo{'ibacm'}{'Name'}, "$prefix/sbin", "ibacm", "ibacm");
}

sub stop_ibacm()
{
	stop_utility($ComponentInfo{'ibacm'}{'Name'}, "ibacm", "ibacm");
}

sub available_ibacm()
{
	my $srcdir=$ComponentInfo{'ibacm'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_ibacm()
{
	return (rpm_is_installed("ibacm", "user")
			&& -e "$ROOT$BASE_DIR/version_delta");
}

# only called if installed_ibacm is true
sub installed_version_ibacm()
{
	if ( -e "$ROOT$BASE_DIR/version_delta" ) {
		return `cat $ROOT$BASE_DIR/version_delta`;
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

	print_install_banner_delta_comp('ibacm');
	install_delta_comp('ibacm', $install_list);

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

	print_uninstall_banner_delta_comp('ibacm');
	stop_ibacm;

	uninstall_delta_comp('ibacm', $install_list, $uninstalling_list, 'verbose');
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
        if ( $status eq "enabled\n" ){
                return 1;
        }
        else{
                return 0;
        }
}

# ------------------------------------------------------------------
# subroutines for hfi1_uefi component
# ------------------------------------------------------------------
sub available_hfi1_uefi()
{
	my $srcdir=$ComponentInfo{'hfi1_uefi'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_hfi1_uefi()
{
	return (rpm_is_installed("hfi1-uefi", "user")
		&& -e "$ROOT$BASE_DIR/version_delta");
}

# only called if installed_xxxx is true
sub installed_version_hfi1_uefi()
{
	if ( -e "$ROOT$BASE_DIR/version_delta" ) {
		return `cat $ROOT$BASE_DIR/version_delta`;
	} else {
		return "";
	}
}
# only called if available_xxxxxis true
sub media_version_hfi1_uefi()
{
	return media_version_delta();
}

sub need_reinstall_hfi1_uefi($$)
{
	my $install_list = shift();     # total that will be installed when done
	my $installing_list = shift();  # what items are being installed/reinstalled

	return (need_reinstall_delta_comp('hfi1_uefi', $install_list, $installing_list));
}

sub preinstall_hfi1_uefi($$)
{
	my $install_list = shift();     # total that will be installed when done
	my $installing_list = shift();  # what items are being installed/reinstalled

	return preinstall_delta("hfi1_uefi", $install_list, $installing_list);
}

sub install_hfi1_uefi($$)
{
	my $install_list = shift();     # total that will be installed when done
	my $installing_list = shift();  # what items are being installed/reinstalled

	print_install_banner_delta_comp('hfi1_uefi');
	install_delta_comp('hfi1_uefi', $install_list);

	$ComponentWasInstalled{'hfi1_uefi'}=1;
}

sub postinstall_hfi1_uefi($$)
{

}
sub uninstall_hfi1_uefi($$)
{
	my $install_list = shift();     # total that will be left installed when done
	my $uninstalling_list = shift();        # what items are being uninstalled

	print_uninstall_banner_delta_comp('hfi1_uefi');
	uninstall_delta_comp('hfi1_uefi', $install_list, $uninstalling_list, 'verbose');
	$ComponentWasInstalled{'hfi1_uefi'}=0;
}

